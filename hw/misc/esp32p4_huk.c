/*
 * ESP32-P4 HUK Generator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 36 (Key Manager: HUK Generator), ESP-IDF's huk_reg.h for v3 and
 * huk_hal.c, and the boot ROM's esp_rom_km_huk_conf(), which moves the 660
 * bytes of huk_info through HUK_INFO_MEM (the header gives the memory 384
 * bytes; the TRM and the ROM 660).
 *
 * The Hardware Unique Key comes from the chip's SRAM PUF. A process runs
 * IDLE, PREP (BUSY), LOAD, PROC (BUSY), GAIN, POST (BUSY) back to IDLE:
 * HUK_START from IDLE, HUK_CONTINUE from LOAD and from GAIN, with
 * HUK_PREP_DONE_INT, _PROC_DONE_INT and _POST_DONE_INT at the ends of the
 * busy phases. HUK_MODE 1 with an even number of bits set in
 * EFUSE_KM_HUK_GEN_STATE generates a new HUK and its huk_info, which
 * software reads from HUK_INFO_MEM in GAIN; otherwise the process recovers
 * the HUK from the huk_info software writes there in LOAD (TRM table
 * 36.6-1). HUK_STATUS is then 1 (valid), or 2 (invalid) when the huk_info
 * is not this chip's, and the HUK goes to the Key Manager.
 *
 * The model's PUF: the chip's PUF response is SHA-256 of a constant, the
 * "puf-seed" property and EFUSE_MAC, so the HUK of an emulated chip
 * recovers in any later run with the same eFuses (an eFuse file) and
 * property, and huk_info from another chip, or corrupted, does not. A new
 * HUK is random. huk_info is a random 128-bit nonce, the HUK masked with
 * HMAC-SHA-256(response, "mask" || nonce), helper data derived from the
 * response and the nonce, and in its last 32 bytes a check value,
 * HMAC-SHA-256(response, "check" || the rest); it holds nothing of the HUK
 * in the clear, and a change anywhere in it fails the recovery. The
 * emulated PUF has no noisy bits: HUK_RISK_LEVEL is 0 and HUK_UPDATE_REQ 0
 * after a recovery, and 7 with HUK_STATUS 2 after a failed one.
 *
 * Timing, in cycles of LP_DYN_FAST_CLK (the model's figures): PREP 128,
 * PROC 512, POST 64. HUK_INFO_MEM takes writes in LOAD and reads back in
 * LOAD and GAIN; other accesses are dropped and logged, and it clears when
 * the process ends. HUK_CLK_EN and HUK_MEM_CG_FORCE_ON only keep clocks
 * running, and are held. Its reset (LP_CLKRST_RST_EN_LP_HUK) loses the HUK.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_huk.h"
#include "migration/vmstate.h"

#define A_CLK           0x04
#define A_INT_RAW       0x08
#define A_INT_ST        0x0c
#define A_INT_ENA       0x10
#define A_INT_CLR       0x14
#define A_CONF          0x20
#define A_START         0x24
#define A_STATE         0x28
#define A_STATUS        0x34
#define A_DATE          0xfc
#define A_INFO          0x100
#define A_INFO_END      (A_INFO + ESP32P4_HUK_INFO_BYTES)

#define CLK_RESET       0x1
#define DATE_RESET      37765232

#define START_START     BIT(0)
#define START_CONTINUE  BIT(1)

#define INT_PREP_DONE   BIT(0)
#define INT_PROC_DONE   BIT(1)
#define INT_POST_DONE   BIT(2)
#define INT_ALL         0x7

#define STATUS_NONE     0
#define STATUS_VALID    1
#define STATUS_INVALID  2
#define STATUS_RISK_S   2

#define PREP_CYCLES     128
#define PROC_CYCLES     512
#define POST_CYCLES     64

/* The layout of huk_info. */
#define INFO_NONCE      0
#define INFO_MASKED     16
#define INFO_HELPER     48
#define INFO_CHECK      (ESP32P4_HUK_INFO_BYTES - 32)

enum {
    HUK_STATE_IDLE,
    HUK_STATE_LOAD,
    HUK_STATE_GAIN,
    HUK_STATE_BUSY,
};

enum {
    PHASE_IDLE,
    PHASE_PREP,
    PHASE_LOAD,
    PHASE_PROC,
    PHASE_GAIN,
    PHASE_POST,
};

static void huk_update_irq(ESP32P4HukState *s)
{
    qemu_set_irq(s->irq, s->int_raw & s->int_ena);
}

static void huk_raise(ESP32P4HukState *s, uint32_t bits)
{
    s->int_raw |= bits;
    huk_update_irq(s);
}

/* The chip's SRAM PUF response. */
static void huk_puf(ESP32P4HukState *s, uint8_t out[32])
{
    static const char tag[] = "ESP32-P4 SRAM PUF";
    uint8_t buf[sizeof(tag) + 8 + 8];

    memcpy(buf, tag, sizeof(tag));
    stq_le_p(buf + sizeof(tag), s->puf_seed);
    stl_le_p(buf + sizeof(tag) + 8,
             s->efuse ? esp32p4_efuse_word(s->efuse, 1, 0) : 0);
    stl_le_p(buf + sizeof(tag) + 12,
             s->efuse ? esp32p4_efuse_word(s->efuse, 1, 1) : 0);
    esp32p4_sha256(buf, sizeof(buf), out);
}

/* HMAC-SHA-256(response, label || data). */
static void huk_derive(const uint8_t puf[32], const char *label,
                       const uint8_t *data, size_t len, uint8_t out[32])
{
    uint8_t buf[16 + ESP32P4_HUK_INFO_BYTES];
    size_t l = strlen(label);

    assert(l <= 16 && len <= ESP32P4_HUK_INFO_BYTES);
    memcpy(buf, label, l);
    memcpy(buf + l, data, len);
    esp32p4_hmac_sha256(puf, 32, buf, l + len, out);
}

/* The check value over the rest of huk_info. */
static void huk_check(const uint8_t puf[32], const uint8_t *info,
                      uint8_t out[32])
{
    huk_derive(puf, "check", info, INFO_CHECK, out);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void huk_generate(ESP32P4HukState *s)
{
    uint8_t puf[32], mask[32], block[32];
    uint8_t ctr[16 + 4];
    unsigned i, off;

    huk_puf(s, puf);
    qemu_guest_getrandom_nofail(s->huk, sizeof(s->huk));
    qemu_guest_getrandom_nofail(s->info + INFO_NONCE, 16);
    huk_derive(puf, "mask", s->info + INFO_NONCE, 16, mask);
    for (i = 0; i < 32; i++) {
        s->info[INFO_MASKED + i] = s->huk[i] ^ mask[i];
    }
    memcpy(ctr, s->info + INFO_NONCE, 16);
    for (off = INFO_HELPER, i = 0; off < INFO_CHECK; i++) {
        stl_le_p(ctr + 16, i);
        huk_derive(puf, "helper", ctr, sizeof(ctr), block);
        memcpy(s->info + off, block, MIN(32, INFO_CHECK - off));
        off += 32;
    }
    huk_check(puf, s->info, s->info + INFO_CHECK);
    s->status = STATUS_VALID;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void huk_recover(ESP32P4HukState *s)
{
    uint8_t puf[32], mask[32], check[32];
    unsigned i;

    huk_puf(s, puf);
    huk_check(puf, s->info, check);
    if (memcmp(check, s->info + INFO_CHECK, 32)) {
        memset(s->huk, 0, sizeof(s->huk));
        s->status = STATUS_INVALID | (7u << STATUS_RISK_S);
        return;
    }
    huk_derive(puf, "mask", s->info + INFO_NONCE, 16, mask);
    for (i = 0; i < 32; i++) {
        s->huk[i] = s->info[INFO_MASKED + i] ^ mask[i];
    }
    s->status = STATUS_VALID;
}

bool esp32p4_huk_key(ESP32P4HukState *s, uint8_t key[32])
{
    if ((s->status & 3) != STATUS_VALID) {
        return false;
    }
    memcpy(key, s->huk, 32);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void huk_op_done(void *opaque)
{
    ESP32P4HukState *s = opaque;

    switch (s->phase) {
    case PHASE_PREP:
        s->phase = PHASE_LOAD;
        s->state = HUK_STATE_LOAD;
        huk_raise(s, INT_PREP_DONE);
        break;
    case PHASE_PROC:
        if (s->generate) {
            huk_generate(s);
        } else {
            huk_recover(s);
        }
        s->phase = PHASE_GAIN;
        s->state = HUK_STATE_GAIN;
        huk_raise(s, INT_PROC_DONE);
        break;
    case PHASE_POST:
        memset(s->info, 0, sizeof(s->info));
        s->phase = PHASE_IDLE;
        s->state = HUK_STATE_IDLE;
        huk_raise(s, INT_POST_DONE);
        break;
    }
}

/* Generation needs HUK_MODE and an even EFUSE_KM_HUK_GEN_STATE. */
static bool huk_generation_mode(ESP32P4HukState *s)
{
    uint32_t gen_state = s->efuse ? extract32(esp32p4_efuse_word(s->efuse,
                                                                  0, 1),
                                              26, 5) : 0;

    return (s->conf & 1) && !(ctpop32(gen_state) & 1);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void huk_start_reg(ESP32P4HukState *s, uint32_t v)
{
    if ((v & START_START) && s->phase == PHASE_IDLE) {
        s->generate = huk_generation_mode(s);
        s->phase = PHASE_PREP;
        s->state = HUK_STATE_BUSY;
        memset(s->info, 0, sizeof(s->info));
        esp32p4_crypto_timer_start(&s->op, PREP_CYCLES);
    } else if ((v & START_CONTINUE) && s->phase == PHASE_LOAD) {
        s->phase = PHASE_PROC;
        s->state = HUK_STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, PROC_CYCLES);
    } else if ((v & START_CONTINUE) && s->phase == PHASE_GAIN) {
        s->phase = PHASE_POST;
        s->state = HUK_STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, POST_CYCLES);
    } else if (v & (START_START | START_CONTINUE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: HUK_START_REG 0x%x in state %u "
                      "is ignored\n", TYPE_ESP32P4_HUK, v, s->state);
    }
}

static bool huk_info_ok(ESP32P4HukState *s, hwaddr addr, bool write)
{
    if (s->phase == PHASE_LOAD || (!write && s->phase == PHASE_GAIN)) {
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of HUK_INFO_MEM at 0x%"
                  HWADDR_PRIx " in state %u is refused\n", TYPE_ESP32P4_HUK,
                  write ? "write" : "read", addr, s->state);
    return false;
}

static uint64_t huk_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4HukState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    if (reg >= A_INFO && reg < A_INFO_END) {
        if (huk_info_ok(s, addr, false)) {
            v = ldl_le_p(s->info + reg - A_INFO);
        }
        return extract32(v, (addr & 3) * 8, size * 8);
    }
    switch (reg) {
    case A_CLK:
        v = s->clk_reg;
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_CONF:
        v = s->conf;
        break;
    case A_STATE:
        v = s->state;
        break;
    case A_STATUS:
        v = s->status;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_INT_CLR:
    case A_START:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_HUK, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void huk_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4HukState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);

    if (reg >= A_INFO && reg < A_INFO_END) {
        uint8_t *p = s->info + reg - A_INFO;

        if (huk_info_ok(s, addr, true)) {
            stl_le_p(p, deposit32(ldl_le_p(p), shift, size * 8, value));
        }
        return;
    }
    switch (reg) {
    case A_CLK:
        s->clk_reg = deposit32(s->clk_reg, shift, size * 8, value) & 3;
        break;
    case A_INT_ENA:
        s->int_ena = deposit32(s->int_ena, shift, size * 8, value) & INT_ALL;
        huk_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_ALL);
        huk_update_irq(s);
        break;
    case A_CONF:
        if (s->phase != PHASE_IDLE) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: HUK_CONF_REG written outside "
                          "IDLE; dropped\n", TYPE_ESP32P4_HUK);
            break;
        }
        s->conf = deposit32(s->conf, shift, size * 8, value) & 1;
        break;
    case A_START:
        huk_start_reg(s, v);
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value) & 0x0fffffff;
        break;
    case A_INT_RAW:
    case A_INT_ST:
    case A_STATE:
    case A_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_HUK, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_HUK, addr);
        break;
    }
}

static const MemoryRegionOps huk_ops = {
    .read = huk_read,
    .write = huk_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void huk_clk_update(void *opaque, ClockEvent event)
{
    esp32p4_crypto_timer_clock_update(&ESP32P4_HUK(opaque)->op);
}

static void huk_reset_hold(Object *obj, ResetType type)
{
    ESP32P4HukState *s = ESP32P4_HUK(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    s->clk_reg = CLK_RESET;
    s->int_raw = 0;
    s->int_ena = 0;
    s->conf = 0;
    s->state = HUK_STATE_IDLE;
    s->status = STATUS_NONE;
    s->date = DATE_RESET;
    memset(s->info, 0, sizeof(s->info));
    memset(s->huk, 0, sizeof(s->huk));
    s->phase = PHASE_IDLE;
    s->generate = false;
}

static void huk_reset_exit(Object *obj, ResetType type)
{
    huk_update_irq(ESP32P4_HUK(obj));
}

static void huk_init(Object *obj)
{
    ESP32P4HukState *s = ESP32P4_HUK(obj);

    memory_region_init_io(&s->iomem, obj, &huk_ops, s, "esp32p4.lp-huk",
                          ESP32P4_HUK_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", huk_clk_update, s,
                                ClockUpdate);
}

static void huk_realize(DeviceState *dev, Error **errp)
{
    ESP32P4HukState *s = ESP32P4_HUK(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, huk_op_done, s);
}

static const Property huk_properties[] = {
    DEFINE_PROP_LINK("efuse", ESP32P4HukState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
    DEFINE_PROP_UINT64("puf-seed", ESP32P4HukState, puf_seed, 0),
};

static const VMStateDescription vmstate_esp32p4_huk = {
    .name = TYPE_ESP32P4_HUK,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4HukState),
        VMSTATE_UINT32(clk_reg, ESP32P4HukState),
        VMSTATE_UINT32(int_raw, ESP32P4HukState),
        VMSTATE_UINT32(int_ena, ESP32P4HukState),
        VMSTATE_UINT32(conf, ESP32P4HukState),
        VMSTATE_UINT32(state, ESP32P4HukState),
        VMSTATE_UINT32(status, ESP32P4HukState),
        VMSTATE_UINT32(date, ESP32P4HukState),
        VMSTATE_UINT8_ARRAY(info, ESP32P4HukState, ESP32P4_HUK_INFO_BYTES),
        VMSTATE_UINT8_ARRAY(huk, ESP32P4HukState, 32),
        VMSTATE_UINT32(phase, ESP32P4HukState),
        VMSTATE_BOOL(generate, ESP32P4HukState),
        VMSTATE_END_OF_LIST()
    },
};

static void huk_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = huk_realize;
    dc->vmsd = &vmstate_esp32p4_huk;
    device_class_set_props(dc, huk_properties);
    rc->phases.hold = huk_reset_hold;
    rc->phases.exit = huk_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo huk_info = {
    .name          = TYPE_ESP32P4_HUK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4HukState),
    .instance_init = huk_init,
    .class_init    = huk_class_init,
};

static void huk_register_types(void)
{
    type_register_static(&huk_info);
}
type_init(huk_register_types)
