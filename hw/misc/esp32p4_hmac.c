/*
 * ESP32-P4 HMAC accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 29 (HMAC Accelerator), ESP-IDF's hmac_reg.h for v3, hmac_ll.h,
 * hmac_hal.c and esp_hmac.c.
 *
 * HMAC-SHA-256 (RFC 2104) with a 256-bit key from an eFuse key block,
 * KEY0 to KEY5 (HMAC_SET_PARA_KEY_REG 0 to 5), or the key the Key Manager
 * deployed for HMAC (7). A session: HMAC_SET_START_REG; the purpose and
 * key; HMAC_SET_PARA_FINISH_REG, after which HMAC_QUERY_ERROR_REG reads 1
 * if the key block's purpose does not allow the configured one (TRM table
 * 29.2-1: upstream 8 takes a key of purpose 8, downstream JTAG 6 one of
 * purpose 6 or 5, downstream RSA_DS 7 one of 7 or 5, 5 one of 5), if
 * EFUSE_FORCE_USE_KEY_MANAGER_KEY forbids HMAC the eFuses' keys, or if
 * the Key Manager key is chosen but not deployed or not selected
 * (KEYMNG_USE_EFUSE_KEY); the Key Manager's key is for upstream only.
 * After an error HMAC_SET_RESULT_FINISH_REG ends the session.
 *
 * Upstream: the accelerator hashes the key block (K xor ipad), then each
 * block software writes to HMAC_WR_MESSAGE_n_REG with
 * HMAC_SET_MESSAGE_ONE_REG; after a block, HMAC_SET_MESSAGE_ING_REG
 * expects another, HMAC_SET_MESSAGE_PAD_REG the last one, padded by
 * software, after which the accelerator finishes by itself,
 * HMAC_SET_MESSAGE_END_REG has the accelerator pad a message of whole
 * blocks and finish, and HMAC_ONE_BLOCK_REG finishes a message whose one
 * block software padded. Software pads as ESP-IDF does, the length
 * counting the key block. The result, in HMAC_RD_RESULT_n_REG, is read
 * until HMAC_SET_RESULT_FINISH_REG clears it and ends the session.
 *
 * Downstream: the accelerator computes the HMAC of 32 bytes of 0x00 (JTAG)
 * or of 0xFF (the RSA_DS key, which goes to the RSA_DS peripheral) and
 * returns to idle; purpose 5 computes both. HMAC_SET_INVALIDATE_JTAG_REG
 * and _DS_REG clear them. With HMAC_SOFT_JTAG_CTRL_REG software then
 * writes a token of eight words to HMAC_WR_JTAG_REG, each the next four
 * bytes of the HMAC as a big-endian word; if they match, "jtag-enable"
 * rises until the JTAG result is invalidated. (Nothing in the model gates
 * JTAG on it.) Released from reset, the accelerator derives the RSA_DS key
 * by itself from the first key block of purpose 7, if there is one (TRM
 * 29.2).
 *
 * The registers are byte strings: byte 0 of a block or result in bits
 * 7:0 of the first register. The control registers act on any write (WS,
 * TRM "Access Types": ESP-IDF finishes a session by writing 2).
 *
 * Timing: each SHA-256 block takes 72 cycles of CRYPTO_HMAC_CLK (the
 * model's figure), and HMAC_QUERY_BUSY_REG reads 1 while the accelerator
 * computes, during which writes are dropped and logged. A session
 * occupies the SHA accelerator from HMAC_SET_START_REG until it ends, as
 * does a downstream calculation until it is done; the SHA accelerator
 * resets when released.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_hmac.h"
#include "migration/vmstate.h"

#define A_SET_START             0x40
#define A_SET_PARA_PURPOSE      0x44
#define A_SET_PARA_KEY          0x48
#define A_SET_PARA_FINISH       0x4c
#define A_SET_MESSAGE_ONE       0x50
#define A_SET_MESSAGE_ING       0x54
#define A_SET_MESSAGE_END       0x58
#define A_SET_RESULT_FINISH     0x5c
#define A_SET_INVALIDATE_JTAG   0x60
#define A_SET_INVALIDATE_DS     0x64
#define A_QUERY_ERROR           0x68
#define A_QUERY_BUSY            0x6c
#define A_WR_MESSAGE            0x80
#define A_RD_RESULT             0xc0
#define A_SET_MESSAGE_PAD       0xf0
#define A_ONE_BLOCK             0xf4
#define A_SOFT_JTAG_CTRL        0xf8
#define A_WR_JTAG               0xfc
#define A_DATE                  0x1fc

#define DATE_RESET              539166977
#define BLOCK_CYCLES            72

/* EFUSE_KEY_PURPOSE values (TRM table 29.2-1). */
#define PURPOSE_DOWN_ALL        5
#define PURPOSE_DOWN_JTAG       6
#define PURPOSE_DOWN_DS         7
#define PURPOSE_UP              8

#define KEY_SEL_KM              7

enum {
    PH_IDLE,
    PH_STARTED,         /* HMAC_SET_START_REG; configuring */
    PH_ERROR,           /* the configuration failed */
    PH_UP_MSG,          /* waiting for a message block */
    PH_UP_CTRL,         /* a block done; waiting for what follows */
    PH_UP_DONE,         /* the result is ready */
};

/* What the calculation in progress does when it ends. */
enum {
    P_NONE,
    P_KEY_UP,           /* the key block, for upstream */
    P_DOWN,             /* a whole downstream calculation */
    P_BLOCK,            /* a message block */
    P_FINAL,            /* the outer hash */
    P_PAD_FINAL,        /* the hardware's padding block, then the outer */
    P_AUTO_DS,          /* the RSA_DS key derivation after reset */
};

static void hmac_sha_hold(ESP32P4HmacState *s, bool hold)
{
    if (s->sha && hold != s->sha_held) {
        esp32p4_sha_occupy(s->sha, hold);
    }
    s->sha_held = hold;
}

static void hmac_set_ds_key(ESP32P4HmacState *s, const uint8_t *key)
{
    if (key) {
        memcpy(s->ds_key, key, 32);
    } else {
        memset(s->ds_key, 0, 32);
    }
    s->ds_key_valid = key != NULL;
    qemu_set_irq(s->ds_key_out, s->ds_key_valid);
}

static void hmac_set_jtag(ESP32P4HmacState *s, bool enabled)
{
    s->jtag_enabled = enabled;
    qemu_set_irq(s->jtag_out, enabled);
}

static void hmac_end_session(ESP32P4HmacState *s)
{
    s->phase = PH_IDLE;
    memset(s->result, 0, sizeof(s->result));
    memset(s->k0, 0, sizeof(s->k0));
    memset(&s->inner, 0, sizeof(s->inner));
    hmac_sha_hold(s, false);
}

static void hmac_compute(ESP32P4HmacState *s, uint32_t pending,
                         unsigned blocks)
{
    s->pending = pending;
    s->busy = true;
    esp32p4_crypto_timer_start(&s->op, (uint64_t)blocks * BLOCK_CYCLES);
}

/*
 * The key for the configured purpose, and whether it may be used: the
 * check HMAC_QUERY_ERROR_REG reports.
 */
static bool hmac_key(ESP32P4HmacState *s, uint8_t key[32])
{
    uint32_t kp;
    unsigned block;

    if (s->key_sel == KEY_SEL_KM) {
        /* The Key Manager's key makes the session an upstream one. */
        s->purpose = PURPOSE_UP;
        return s->keymng &&
               esp32p4_keymng_selects_own(s->keymng, ESP32P4_KM_USER_HMAC) &&
               esp32p4_keymng_key(s->keymng, ESP32P4_KM_PURPOSE_HMAC, key);
    }
    if (s->key_sel >= ESP32P4_EFUSE_KEY_BLOCKS || !s->efuse) {
        return false;
    }
    if (s->keymng && esp32p4_keymng_forced(s->keymng, ESP32P4_KM_USER_HMAC)) {
        return false;
    }
    block = ESP32P4_EFUSE_KEY_BLOCK0 + s->key_sel;
    kp = esp32p4_efuse_key_purpose(s->efuse, block);
    switch (s->purpose) {
    case PURPOSE_UP:
        if (kp != PURPOSE_UP) {
            return false;
        }
        break;
    case PURPOSE_DOWN_JTAG:
    case PURPOSE_DOWN_DS:
        if (kp != s->purpose && kp != PURPOSE_DOWN_ALL) {
            return false;
        }
        break;
    case PURPOSE_DOWN_ALL:
        if (kp != PURPOSE_DOWN_ALL) {
            return false;
        }
        break;
    default:
        return false;
    }
    esp32p4_efuse_key(s->efuse, block, key);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void hmac_para_finish(ESP32P4HmacState *s)
{
    uint8_t key[32];
    uint8_t pad[64];
    int i;

    if (s->purpose == PURPOSE_DOWN_DS || s->purpose == PURPOSE_DOWN_ALL) {
        s->ds_tried = true;
    }
    if (!hmac_key(s, key)) {
        s->query_error = true;
        s->phase = PH_ERROR;
        return;
    }
    memset(s->k0, 0, sizeof(s->k0));
    memcpy(s->k0, key, 32);
    if (s->purpose != PURPOSE_UP) {
        hmac_compute(s, P_DOWN, 4);
        return;
    }
    for (i = 0; i < 64; i++) {
        pad[i] = s->k0[i] ^ 0x36;
    }
    esp32p4_sha_init(ESP32P4_SHA256, &s->inner);
    esp32p4_sha_compress(ESP32P4_SHA256, &s->inner, pad);
    s->blocks = 0;
    s->last_next = false;
    hmac_compute(s, P_KEY_UP, 1);
}

/* The outer hash of the inner one into the result. */
static void hmac_outer(ESP32P4HmacState *s)
{
    uint8_t h1[32];
    uint8_t pad[64];
    ESP32P4ShaState outer;
    int i;

    esp32p4_sha_state_to_bytes(ESP32P4_SHA256, &s->inner, h1);
    for (i = 0; i < 64; i++) {
        pad[i] = s->k0[i] ^ 0x5c;
    }
    esp32p4_sha_init(ESP32P4_SHA256, &outer);
    esp32p4_sha_compress(ESP32P4_SHA256, &outer, pad);
    memset(pad, 0, sizeof(pad));
    memcpy(pad, h1, 32);
    pad[32] = 0x80;
    stq_be_p(pad + 56, (64 + 32) * 8);
    esp32p4_sha_compress(ESP32P4_SHA256, &outer, pad);
    esp32p4_sha_state_to_bytes(ESP32P4_SHA256, &outer, s->result);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void hmac_op_done(void *opaque)
{
    ESP32P4HmacState *s = opaque;
    uint32_t pending = s->pending;
    uint8_t msg[32], out[32], key[32];
    uint8_t pad[64];
    int block;

    s->pending = P_NONE;
    s->busy = false;
    switch (pending) {
    case P_KEY_UP:
        s->phase = PH_UP_MSG;
        break;
    case P_DOWN:
        if (s->purpose != PURPOSE_DOWN_DS) {
            memset(msg, 0, sizeof(msg));
            esp32p4_hmac_sha256(s->k0, 32, msg, sizeof(msg), s->jtag_hmac);
            s->jtag_valid = true;
        }
        if (s->purpose != PURPOSE_DOWN_JTAG) {
            memset(msg, 0xff, sizeof(msg));
            esp32p4_hmac_sha256(s->k0, 32, msg, sizeof(msg), out);
            hmac_set_ds_key(s, out);
        }
        hmac_end_session(s);
        break;
    case P_BLOCK:
        esp32p4_sha_compress(ESP32P4_SHA256, &s->inner, s->msg);
        s->blocks++;
        if (s->last_next) {
            s->last_next = false;
            hmac_compute(s, P_FINAL, 2);
        } else {
            s->phase = PH_UP_CTRL;
        }
        break;
    case P_PAD_FINAL:
        memset(pad, 0, sizeof(pad));
        pad[0] = 0x80;
        stq_be_p(pad + 56, (uint64_t)(s->blocks + 1) * 512);
        esp32p4_sha_compress(ESP32P4_SHA256, &s->inner, pad);
        QEMU_FALLTHROUGH;
    case P_FINAL:
        hmac_outer(s);
        s->phase = PH_UP_DONE;
        break;
    case P_AUTO_DS:
        if (!s->efuse) {
            break;
        }
        block = esp32p4_efuse_find_purpose(s->efuse, PURPOSE_DOWN_DS);
        if (block >= 0) {
            esp32p4_efuse_key(s->efuse, block, key);
            memset(msg, 0xff, sizeof(msg));
            esp32p4_hmac_sha256(key, 32, msg, sizeof(msg), out);
            hmac_set_ds_key(s, out);
        }
        break;
    }
}

bool esp32p4_hmac_ds_key(ESP32P4HmacState *s, uint8_t key[32])
{
    if (!s->ds_key_valid) {
        return false;
    }
    memcpy(key, s->ds_key, 32);
    return true;
}

bool esp32p4_hmac_ds_tried(ESP32P4HmacState *s)
{
    return s->ds_tried;
}

static uint64_t hmac_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4HmacState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    switch (reg) {
    case A_QUERY_ERROR:
        v = s->query_error;
        break;
    case A_QUERY_BUSY:
        v = s->busy;
        break;
    case A_RD_RESULT ... A_RD_RESULT + 0x1c:
        if (s->phase == PH_UP_DONE) {
            v = ldl_le_p(s->result + reg - A_RD_RESULT);
        }
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_SET_START ... A_SET_INVALIDATE_DS:
    case A_WR_MESSAGE ... A_WR_MESSAGE + 0x3c:
    case A_SET_MESSAGE_PAD ... A_WR_JTAG:
        /* Write-only. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_HMAC, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

static void hmac_wrong_phase(const char *what)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s outside the phase that takes it "
                  "is ignored\n", TYPE_ESP32P4_HMAC, what);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void hmac_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4HmacState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);

    if (s->in_reset || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the accelerator is %s\n", TYPE_ESP32P4_HMAC,
                      addr, s->in_reset ? "held in reset"
                                        : "not clocked (CRYPTO_HMAC_CLK)");
        return;
    }
    if (s->busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while the accelerator is busy is dropped\n",
                      TYPE_ESP32P4_HMAC, addr);
        return;
    }

    switch (reg) {
    case A_SET_START:
        if (s->phase != PH_IDLE) {
            hmac_wrong_phase("HMAC_SET_START_REG");
            break;
        }
        s->phase = PH_STARTED;
        s->query_error = false;
        hmac_sha_hold(s, true);
        break;
    case A_SET_PARA_PURPOSE:
        s->purpose = deposit32(s->purpose, shift, size * 8, value) & 0xf;
        break;
    case A_SET_PARA_KEY:
        s->key_sel = deposit32(s->key_sel, shift, size * 8, value) & 7;
        break;
    case A_SET_PARA_FINISH:
        if (s->phase != PH_STARTED) {
            hmac_wrong_phase("HMAC_SET_PARA_FINISH_REG");
            break;
        }
        hmac_para_finish(s);
        break;
    case A_SET_MESSAGE_ONE:
        if (s->phase != PH_UP_MSG) {
            hmac_wrong_phase("HMAC_SET_MESSAGE_ONE_REG");
            break;
        }
        hmac_compute(s, P_BLOCK, 1);
        break;
    case A_SET_MESSAGE_ING:
        if (s->phase != PH_UP_CTRL) {
            hmac_wrong_phase("HMAC_SET_MESSAGE_ING_REG");
            break;
        }
        s->phase = PH_UP_MSG;
        break;
    case A_SET_MESSAGE_PAD:
        if (s->phase != PH_UP_CTRL) {
            hmac_wrong_phase("HMAC_SET_MESSAGE_PAD_REG");
            break;
        }
        s->last_next = true;
        s->phase = PH_UP_MSG;
        break;
    case A_SET_MESSAGE_END:
        if (s->phase != PH_UP_CTRL) {
            hmac_wrong_phase("HMAC_SET_MESSAGE_END_REG");
            break;
        }
        hmac_compute(s, P_PAD_FINAL, 3);
        break;
    case A_ONE_BLOCK:
        if (s->phase != PH_UP_CTRL) {
            hmac_wrong_phase("HMAC_ONE_BLOCK_REG");
            break;
        }
        hmac_compute(s, P_FINAL, 2);
        break;
    case A_SET_RESULT_FINISH:
        if (s->phase != PH_UP_DONE && s->phase != PH_ERROR) {
            hmac_wrong_phase("HMAC_SET_RESULT_FINISH_REG");
            break;
        }
        hmac_end_session(s);
        break;
    case A_SET_INVALIDATE_JTAG:
        s->jtag_valid = false;
        memset(s->jtag_hmac, 0, sizeof(s->jtag_hmac));
        s->jtag_mode = false;
        hmac_set_jtag(s, false);
        break;
    case A_SET_INVALIDATE_DS:
        hmac_set_ds_key(s, NULL);
        s->ds_tried = false;
        break;
    case A_WR_MESSAGE ... A_WR_MESSAGE + 0x3c:
        stl_le_p(s->msg + reg - A_WR_MESSAGE,
                 deposit32(ldl_le_p(s->msg + reg - A_WR_MESSAGE), shift,
                           size * 8, value));
        break;
    case A_SOFT_JTAG_CTRL:
        s->jtag_mode = true;
        s->jtag_count = 0;
        break;
    case A_WR_JTAG:
        if (!s->jtag_mode) {
            hmac_wrong_phase("HMAC_WR_JTAG_REG");
            break;
        }
        s->jtag_words[s->jtag_count++] = v;
        if (s->jtag_count == 8) {
            bool match = s->jtag_valid;
            int i;

            for (i = 0; i < 8; i++) {
                match &= s->jtag_words[i] == ldl_be_p(s->jtag_hmac + 4 * i);
            }
            s->jtag_mode = false;
            hmac_set_jtag(s, match);
        }
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value) & 0x3fffffff;
        break;
    case A_QUERY_ERROR:
    case A_QUERY_BUSY:
    case A_RD_RESULT ... A_RD_RESULT + 0x1c:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_HMAC, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_HMAC, addr);
        break;
    }
}

static const MemoryRegionOps hmac_ops = {
    .read = hmac_read,
    .write = hmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void hmac_reset_line(void *opaque, int n, int level)
{
    ESP32P4HmacState *s = opaque;

    if (s->in_reset != level) {
        s->in_reset = level;
        device_cold_reset(DEVICE(s));
    }
}

static void hmac_clk_update(void *opaque, ClockEvent event)
{
    esp32p4_crypto_timer_clock_update(&ESP32P4_HMAC(opaque)->op);
}

static void hmac_reset_hold(Object *obj, ResetType type)
{
    ESP32P4HmacState *s = ESP32P4_HMAC(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    hmac_sha_hold(s, false);
    s->purpose = 0;
    s->key_sel = 0;
    memset(s->msg, 0, sizeof(s->msg));
    memset(s->result, 0, sizeof(s->result));
    s->busy = false;
    s->query_error = false;
    s->date = DATE_RESET;
    s->phase = PH_IDLE;
    s->pending = P_NONE;
    memset(s->k0, 0, sizeof(s->k0));
    memset(&s->inner, 0, sizeof(s->inner));
    s->blocks = 0;
    s->last_next = false;
    memset(s->ds_key, 0, sizeof(s->ds_key));
    s->ds_key_valid = false;
    s->ds_tried = false;
    memset(s->jtag_hmac, 0, sizeof(s->jtag_hmac));
    s->jtag_valid = false;
    s->jtag_mode = false;
    s->jtag_count = 0;
    memset(s->jtag_words, 0, sizeof(s->jtag_words));
    s->jtag_enabled = false;
}

static void hmac_reset_exit(Object *obj, ResetType type)
{
    ESP32P4HmacState *s = ESP32P4_HMAC(obj);

    qemu_set_irq(s->ds_key_out, 0);
    qemu_set_irq(s->jtag_out, 0);
    /* The eFuse controller has loaded its blocks in its reset hold. */
    if (!s->in_reset && s->efuse &&
        esp32p4_efuse_find_purpose(s->efuse, PURPOSE_DOWN_DS) >= 0) {
        hmac_compute(s, P_AUTO_DS, 4);
    }
}

static void hmac_init(Object *obj)
{
    ESP32P4HmacState *s = ESP32P4_HMAC(obj);

    memory_region_init_io(&s->iomem, obj, &hmac_ops, s, "esp32p4.hmac",
                          ESP32P4_HMAC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", hmac_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), hmac_reset_line, ESP32P4_HMAC_RESET,
                            1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->ds_key_out,
                             ESP32P4_HMAC_DS_KEY, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->jtag_out,
                             ESP32P4_HMAC_JTAG_ENABLE, 1);
}

static void hmac_realize(DeviceState *dev, Error **errp)
{
    ESP32P4HmacState *s = ESP32P4_HMAC(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, hmac_op_done, s);
}

static const Property hmac_properties[] = {
    DEFINE_PROP_LINK("efuse", ESP32P4HmacState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
    DEFINE_PROP_LINK("keymng", ESP32P4HmacState, keymng,
                     TYPE_ESP32P4_KEYMNG, ESP32P4KeyMngState *),
    DEFINE_PROP_LINK("sha", ESP32P4HmacState, sha, TYPE_ESP32P4_SHA,
                     ESP32P4ShaDevState *),
};

static const VMStateDescription vmstate_esp32p4_hmac_sha = {
    .name = "esp32p4-hmac-sha",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64_ARRAY(h, ESP32P4ShaState, 8),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_hmac = {
    .name = TYPE_ESP32P4_HMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4HmacState),
        VMSTATE_BOOL(in_reset, ESP32P4HmacState),
        VMSTATE_UINT32(purpose, ESP32P4HmacState),
        VMSTATE_UINT32(key_sel, ESP32P4HmacState),
        VMSTATE_UINT8_ARRAY(msg, ESP32P4HmacState, 64),
        VMSTATE_UINT8_ARRAY(result, ESP32P4HmacState, 32),
        VMSTATE_BOOL(busy, ESP32P4HmacState),
        VMSTATE_BOOL(query_error, ESP32P4HmacState),
        VMSTATE_UINT32(date, ESP32P4HmacState),
        VMSTATE_UINT32(phase, ESP32P4HmacState),
        VMSTATE_UINT32(pending, ESP32P4HmacState),
        VMSTATE_UINT8_ARRAY(k0, ESP32P4HmacState, 64),
        VMSTATE_STRUCT(inner, ESP32P4HmacState, 1, vmstate_esp32p4_hmac_sha,
                       ESP32P4ShaState),
        VMSTATE_UINT32(blocks, ESP32P4HmacState),
        VMSTATE_BOOL(last_next, ESP32P4HmacState),
        VMSTATE_BOOL(sha_held, ESP32P4HmacState),
        VMSTATE_UINT8_ARRAY(ds_key, ESP32P4HmacState, 32),
        VMSTATE_BOOL(ds_key_valid, ESP32P4HmacState),
        VMSTATE_BOOL(ds_tried, ESP32P4HmacState),
        VMSTATE_UINT8_ARRAY(jtag_hmac, ESP32P4HmacState, 32),
        VMSTATE_BOOL(jtag_valid, ESP32P4HmacState),
        VMSTATE_BOOL(jtag_mode, ESP32P4HmacState),
        VMSTATE_UINT32(jtag_count, ESP32P4HmacState),
        VMSTATE_UINT32_ARRAY(jtag_words, ESP32P4HmacState, 8),
        VMSTATE_BOOL(jtag_enabled, ESP32P4HmacState),
        VMSTATE_END_OF_LIST()
    },
};

static void hmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = hmac_realize;
    dc->vmsd = &vmstate_esp32p4_hmac;
    device_class_set_props(dc, hmac_properties);
    rc->phases.hold = hmac_reset_hold;
    rc->phases.exit = hmac_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo hmac_info = {
    .name          = TYPE_ESP32P4_HMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4HmacState),
    .instance_init = hmac_init,
    .class_init    = hmac_class_init,
};

static void hmac_register_types(void)
{
    type_register_static(&hmac_info);
}
type_init(hmac_register_types)
