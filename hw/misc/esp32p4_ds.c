/*
 * ESP32-P4 RSA digital signature peripheral (RSA_DS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 32 (RSA Digital Signature Peripheral), ESP-IDF's ds_reg.h for v3 (which
 * puts DS_MD_ERROR in bit 0 of DS_QUERY_CHECK_REG and DS_PADDING_BAD in
 * bit 1, as the TRM's text has it, where its register figure swaps them),
 * ds_ll.h, esp_ds.c and the Key Manager tests' parameter generator.
 *
 * RSA signatures Z = X^Y mod M with a private key software cannot see. The
 * private key parameters are the ciphertext C = AES-256-CBC(P, RSA_DS_KEY,
 * IV) of P = Y || M || r || MD || M' || L || 8 bytes of 0x08, Y, M and r
 * 4096-bit little-endian numbers, M' and L 32-bit, and MD = SHA-256(Y ||
 * M || r || M' || L || IV) (TRM 32.3.3). RSA_DS_KEY is the HMAC
 * accelerator's downstream result, HMAC-SHA-256 of 32 bytes of 0xFF with
 * the eFuse key (DS_KEY_SOURCE_REG 0), or the Key Manager's RSA_DS key
 * (1), which the Key Manager holds byte-reversed.
 *
 * DS_SET_START_REG activates the peripheral, which occupies the AES, SHA
 * and RSA accelerators until DS_SET_FINISH_REG, and waits, busy, until it
 * has RSA_DS_KEY: from HMAC when the downstream calculation has derived
 * it (DS_QUERY_KEY_WRONG_REG reads 0 while HMAC has not been asked to, 1
 * once it has but has not delivered), unless EFUSE_FORCE_USE_KEY_MANAGER_KEY
 * forbids RSA_DS the eFuses' keys; or the Key Manager's, while it is
 * deployed and selected (KEYMNG_USE_EFUSE_KEY's RSA_DS bit clear). With
 * C in DS_Y_MEM to DS_BOX_MEM, the IV and X written, DS_SET_CONTINUE_REG
 * (SET_ME) decrypts C, checks MD (DS_MD_ERROR) and the padding
 * (DS_PADDING_BAD, which does not stop the calculation) and, if MD
 * matches, computes Z of (L + 1) 32-bit words into DS_Z_MEM as the RSA
 * accelerator does. DS_SET_FINISH_REG clears every memory and register
 * and returns the peripheral to idle.
 *
 * The memories hold bytes in address order; DS_Y_MEM to DS_X_MEM and the
 * IV are write-only and read as zero, DS_Z_MEM read-only.
 *
 * Timing, in cycles of CRYPTO_DS_CLK: taking the key 8; the calculation
 * the AES decryption (16 a block), the SHA-256 of MD (72 a block) and the
 * RSA accelerator's constant-time exponentiation (esp32p4_rsa.c); the
 * finish 8. DS_QUERY_BUSY_REG reads 1 meanwhile and writes are dropped
 * and logged. The calculation starts as DS_SET_CONTINUE_REG is written,
 * however long the model takes to make it, and register accesses see it
 * end at its time, however late its QEMU timer runs.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "crypto/aes.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_ds.h"
#include "migration/vmstate.h"

#define A_Y_MEM             0x000
#define A_BOX_MEM           0x600
#define A_IV_MEM            0x630
#define A_X_MEM             0x800
#define A_Z_MEM             0xa00
#define A_SET_START         0xe00
#define A_SET_CONTINUE      0xe04
#define A_SET_FINISH        0xe08
#define A_QUERY_BUSY        0xe0c
#define A_QUERY_KEY_WRONG   0xe10
#define A_QUERY_CHECK       0xe14
#define A_KEY_SOURCE        0xe18
#define A_DATE              0xe20

#define DATE_RESET          539166977
#define CHECK_MD_ERROR      BIT(0)
#define CHECK_PADDING_BAD   BIT(1)
#define STEP_CYCLES         8

enum {
    DS_IDLE,
    DS_KEY_WAIT,        /* started; waiting for RSA_DS_KEY */
    DS_KEY_TAKE,        /* taking the key */
    DS_READY,           /* the key taken; waiting for SET_ME */
    DS_SIGNING,
    DS_DONE,
    DS_FINISHING,
};

static void ds_occupy(ESP32P4DsState *s, bool on)
{
    if (s->aes) {
        esp32p4_aes_occupy(s->aes, on);
    }
    if (s->sha) {
        esp32p4_sha_occupy(s->sha, on);
    }
    if (s->rsa) {
        esp32p4_rsa_occupy(s->rsa, on);
    }
}

/* RSA_DS_KEY from its source, if it is there to take. */
static bool ds_get_key(ESP32P4DsState *s, uint8_t key[32])
{
    uint8_t slot[32];
    int i;

    if (s->key_source & 1) {
        if (!s->keymng ||
            !esp32p4_keymng_selects_own(s->keymng, ESP32P4_KM_USER_DS) ||
            !esp32p4_keymng_key(s->keymng, ESP32P4_KM_PURPOSE_DS, slot)) {
            return false;
        }
        for (i = 0; i < 32; i++) {
            key[i] = slot[31 - i];
        }
        return true;
    }
    if (!s->hmac || (s->keymng &&
                     esp32p4_keymng_forced(s->keymng, ESP32P4_KM_USER_DS))) {
        return false;
    }
    return esp32p4_hmac_ds_key(s->hmac, key);
}

static void ds_try_key(ESP32P4DsState *s)
{
    if (s->phase == DS_KEY_WAIT && ds_get_key(s, s->key)) {
        s->phase = DS_KEY_TAKE;
        esp32p4_crypto_timer_start(&s->op, STEP_CYCLES);
    }
}

/*
 * The calculation, whose results software sees when its time is over:
 * returns that time, in cycles.
 */
/* [spec:nuos:req:emu.esp32p4.crypto] */
static uint64_t ds_sign(ESP32P4DsState *s)
{
    uint8_t p[ESP32P4_DS_C_BYTES];
    uint8_t md_in[3 * 512 + 8 + 16];
    uint8_t md[32];
    uint32_t y[ESP32P4_RSA_WORDS], m[ESP32P4_RSA_WORDS];
    uint32_t r[ESP32P4_RSA_WORDS];
    uint8_t chain[16], blk[16];
    const uint8_t *box;
    uint32_t mprime, l;
    AES_KEY dk;
    unsigned i, j, n;
    uint64_t cycles = (ESP32P4_DS_C_BYTES / 16) * 16 +
                      DIV_ROUND_UP(sizeof(md_in) + 9, 64) * 72;

    AES_set_decrypt_key(s->key, 256, &dk);
    memcpy(chain, s->iv, 16);
    for (i = 0; i < ESP32P4_DS_C_BYTES; i += 16) {
        AES_decrypt(s->c + i, blk, &dk);
        for (j = 0; j < 16; j++) {
            p[i + j] = blk[j] ^ chain[j];
        }
        memcpy(chain, s->c + i, 16);
    }
    box = p + 3 * 512;
    mprime = ldl_le_p(box + 32);
    l = ldl_le_p(box + 36);

    memcpy(md_in, p, 3 * 512);
    memcpy(md_in + 3 * 512, box + 32, 8);
    memcpy(md_in + 3 * 512 + 8, s->iv, 16);
    esp32p4_sha256(md_in, sizeof(md_in), md);

    s->check = 0;
    if (memcmp(md, box, 32)) {
        s->check |= CHECK_MD_ERROR;
    }
    for (i = 40; i < 48; i++) {
        if (box[i] != 0x08) {
            s->check |= CHECK_PADDING_BAD;
        }
    }
    memset(s->z, 0, sizeof(s->z));
    if (s->check & CHECK_MD_ERROR) {
        return cycles;
    }
    n = l + 1;
    if (n > ESP32P4_RSA_WORDS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the parameters' length L = %u "
                      "is beyond 4096 bits; no signature is computed\n",
                      TYPE_ESP32P4_DS, l);
        return cycles;
    }
    esp32p4_bn_from_le(y, ESP32P4_RSA_WORDS, p, 512);
    esp32p4_bn_from_le(m, ESP32P4_RSA_WORDS, p + 512, 512);
    esp32p4_bn_from_le(r, ESP32P4_RSA_WORDS, p + 1024, 512);
    return cycles + esp32p4_rsa_modexp(s->z, s->x, y, m, r, mprime, n,
                                       32 * n - 1, true, NULL);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ds_op_done(void *opaque)
{
    ESP32P4DsState *s = opaque;

    switch (s->phase) {
    case DS_KEY_TAKE:
        s->phase = DS_READY;
        s->busy = false;
        break;
    case DS_SIGNING:
        s->phase = DS_DONE;
        s->busy = false;
        break;
    case DS_FINISHING:
        s->phase = DS_IDLE;
        s->busy = false;
        break;
    }
}

static void ds_clear(ESP32P4DsState *s)
{
    memset(s->c, 0, sizeof(s->c));
    memset(s->iv, 0, sizeof(s->iv));
    memset(s->x, 0, sizeof(s->x));
    memset(s->z, 0, sizeof(s->z));
    memset(s->key, 0, sizeof(s->key));
    s->check = 0;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ds_hmac_key(void *opaque, int n, int level)
{
    ESP32P4DsState *s = opaque;

    if (level && !(s->key_source & 1)) {
        ds_try_key(s);
    }
}

static uint64_t ds_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4DsState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    esp32p4_crypto_timer_sync(&s->op, esp32_line_time());
    switch (reg) {
    case A_Z_MEM ... A_Z_MEM + 0x1ff:
        if (s->phase == DS_DONE) {
            v = s->z[(reg - A_Z_MEM) / 4];
        }
        break;
    case A_QUERY_BUSY:
        v = s->busy;
        break;
    case A_QUERY_KEY_WRONG:
        if (s->phase == DS_KEY_WAIT && !(s->key_source & 1) && s->hmac &&
            esp32p4_hmac_ds_tried(s->hmac)) {
            v = 1;
        }
        break;
    case A_QUERY_CHECK:
        if (s->phase == DS_DONE) {
            v = s->check;
        }
        break;
    case A_KEY_SOURCE:
        v = s->key_source;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_Y_MEM ... A_IV_MEM + 0xf:
    case A_X_MEM ... A_X_MEM + 0x1ff:
    case A_SET_START:
    case A_SET_CONTINUE:
    case A_SET_FINISH:
        /* Write-only. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DS, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ds_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    ESP32P4DsState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);
    int64_t now = esp32_line_time();

    esp32p4_crypto_timer_sync(&s->op, now);
    if (s->in_reset || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the peripheral is %s\n", TYPE_ESP32P4_DS,
                      addr, s->in_reset ? "held in reset"
                                        : "not clocked (CRYPTO_DS_CLK)");
        return;
    }
    if (s->busy && reg != A_SET_FINISH) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while the peripheral is busy is dropped\n",
                      TYPE_ESP32P4_DS, addr);
        return;
    }

    switch (reg) {
    case A_Y_MEM ... A_BOX_MEM + 0x2f:
        stl_le_p(s->c + reg, deposit32(ldl_le_p(s->c + reg), shift,
                                       size * 8, value));
        break;
    case A_IV_MEM ... A_IV_MEM + 0xf:
        stl_le_p(s->iv + reg - A_IV_MEM,
                 deposit32(ldl_le_p(s->iv + reg - A_IV_MEM), shift,
                           size * 8, value));
        break;
    case A_X_MEM ... A_X_MEM + 0x1ff:
        s->x[(reg - A_X_MEM) / 4] = deposit32(s->x[(reg - A_X_MEM) / 4],
                                              shift, size * 8, value);
        break;
    case A_SET_START:
        if (!(v & 1)) {
            break;
        }
        if (s->phase != DS_IDLE) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DS_SET_START_REG while the "
                          "peripheral is active is ignored\n",
                          TYPE_ESP32P4_DS);
            break;
        }
        ds_occupy(s, true);
        s->phase = DS_KEY_WAIT;
        s->busy = true;
        ds_try_key(s);
        break;
    case A_SET_CONTINUE:
        if (!(v & 1)) {
            break;
        }
        if (s->phase != DS_READY) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DS_SET_CONTINUE_REG without "
                          "RSA_DS_KEY taken is ignored\n", TYPE_ESP32P4_DS);
            break;
        }
        s->phase = DS_SIGNING;
        s->busy = true;
        esp32p4_crypto_timer_start_at(&s->op, ds_sign(s), now);
        break;
    case A_SET_FINISH:
        if (!(v & 1)) {
            break;
        }
        if (s->phase == DS_IDLE || s->phase == DS_FINISHING) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DS_SET_FINISH_REG while the "
                          "peripheral is not active is ignored\n",
                          TYPE_ESP32P4_DS);
            break;
        }
        if (s->phase == DS_SIGNING) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DS_SET_FINISH_REG during the "
                          "calculation is dropped\n", TYPE_ESP32P4_DS);
            break;
        }
        ds_clear(s);
        ds_occupy(s, false);
        s->phase = DS_FINISHING;
        s->busy = true;
        esp32p4_crypto_timer_start(&s->op, STEP_CYCLES);
        break;
    case A_KEY_SOURCE:
        s->key_source = deposit32(s->key_source, shift, size * 8, value) & 1;
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value) & 0x3fffffff;
        break;
    case A_Z_MEM ... A_Z_MEM + 0x1ff:
    case A_QUERY_BUSY:
    case A_QUERY_KEY_WRONG:
    case A_QUERY_CHECK:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DS, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DS, addr);
        break;
    }
}

static const MemoryRegionOps ds_ops = {
    .read = ds_read,
    .write = ds_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ds_reset_line(void *opaque, int n, int level)
{
    ESP32P4DsState *s = opaque;

    if (s->in_reset != level) {
        s->in_reset = level;
        device_cold_reset(DEVICE(s));
    }
}

static void ds_clk_update(void *opaque, ClockEvent event)
{
    esp32p4_crypto_timer_clock_update(&ESP32P4_DS(opaque)->op);
}

static void ds_reset_hold(Object *obj, ResetType type)
{
    ESP32P4DsState *s = ESP32P4_DS(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    if (s->phase != DS_IDLE && s->phase != DS_FINISHING) {
        ds_occupy(s, false);
    }
    ds_clear(s);
    s->key_source = 0;
    s->date = DATE_RESET;
    s->busy = false;
    s->phase = DS_IDLE;
}

static void ds_init(Object *obj)
{
    ESP32P4DsState *s = ESP32P4_DS(obj);

    memory_region_init_io(&s->iomem, obj, &ds_ops, s, "esp32p4.ds",
                          ESP32P4_DS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", ds_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), ds_reset_line, ESP32P4_DS_RESET, 1);
    qdev_init_gpio_in_named(DEVICE(obj), ds_hmac_key, ESP32P4_DS_HMAC_KEY, 1);
}

static void ds_realize(DeviceState *dev, Error **errp)
{
    ESP32P4DsState *s = ESP32P4_DS(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, ds_op_done, s);
}

static const Property ds_properties[] = {
    DEFINE_PROP_LINK("hmac", ESP32P4DsState, hmac, TYPE_ESP32P4_HMAC,
                     ESP32P4HmacState *),
    DEFINE_PROP_LINK("keymng", ESP32P4DsState, keymng, TYPE_ESP32P4_KEYMNG,
                     ESP32P4KeyMngState *),
    DEFINE_PROP_LINK("aes", ESP32P4DsState, aes, TYPE_ESP32P4_AES,
                     ESP32P4AesState *),
    DEFINE_PROP_LINK("sha", ESP32P4DsState, sha, TYPE_ESP32P4_SHA,
                     ESP32P4ShaDevState *),
    DEFINE_PROP_LINK("rsa", ESP32P4DsState, rsa, TYPE_ESP32P4_RSA,
                     ESP32P4RsaState *),
};

static const VMStateDescription vmstate_esp32p4_ds = {
    .name = TYPE_ESP32P4_DS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4DsState),
        VMSTATE_BOOL(in_reset, ESP32P4DsState),
        VMSTATE_UINT8_ARRAY(c, ESP32P4DsState, ESP32P4_DS_C_BYTES),
        VMSTATE_UINT8_ARRAY(iv, ESP32P4DsState, 16),
        VMSTATE_UINT32_ARRAY(x, ESP32P4DsState, ESP32P4_RSA_WORDS),
        VMSTATE_UINT32_ARRAY(z, ESP32P4DsState, ESP32P4_RSA_WORDS),
        VMSTATE_UINT32(key_source, ESP32P4DsState),
        VMSTATE_UINT32(check, ESP32P4DsState),
        VMSTATE_UINT32(date, ESP32P4DsState),
        VMSTATE_BOOL(busy, ESP32P4DsState),
        VMSTATE_UINT32(phase, ESP32P4DsState),
        VMSTATE_UINT8_ARRAY(key, ESP32P4DsState, 32),
        VMSTATE_END_OF_LIST()
    },
};

static void ds_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ds_realize;
    dc->vmsd = &vmstate_esp32p4_ds;
    device_class_set_props(dc, ds_properties);
    rc->phases.hold = ds_reset_hold;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo ds_info = {
    .name          = TYPE_ESP32P4_DS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4DsState),
    .instance_init = ds_init,
    .class_init    = ds_class_init,
};

static void ds_register_types(void)
{
    type_register_static(&ds_info);
}
type_init(ds_register_types)
