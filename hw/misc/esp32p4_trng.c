/*
 * ESP32-P4 random number generator (LP_TRNG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 35 (Random Number Generator), ESP-IDF's trng_reg.h for v3, rng_ll.h and
 * hw_random.c.
 *
 * Each read of RNG_DATA_REG (or of LP_SYSTEM_REG_RNG_DATA_REG, which
 * reads the same generator) gives a new 32-bit number from the host's
 * CSPRNG (qemu_guest_getrandom), while the generator runs: RNG_CLK_EN
 * (bit 31 of RNG_DATE_REG, clear from reset; ESP-IDF's rng_ll_enable()
 * sets it), RNG_RSTN set and its clock running. Otherwise the register
 * holds its last value, zero from reset; RNG_RSTN low clears it.
 *
 * The TRM's sources of entropy (the SAR ADC, RC_FAST_CLK, the BUF_CHAIN)
 * decide whether the numbers are true or pseudo random, which software
 * cannot tell apart; the host's numbers stand for both. The TRM's limit
 * on the rate of reads (1 MHz for full entropy) is advice to software.
 * RNG_SAMPLE_CNT counts the BUF_CHAIN's samples, one each
 * RNG_TIMER_PSCALE + 1 cycles of the clock while RNG_SAMPLE_ENABLE and
 * RNG_TIMER_EN are set and the generator runs.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32p4_trng.h"
#include "migration/vmstate.h"

#define A_CFG               0x00
#define A_DATA              ESP32P4_TRNG_DATA
#define A_RSTN              0x08
#define A_DATE              0x0c

#define CFG_SAMPLE_ENABLE   BIT(0)
#define CFG_PSCALE(c)       extract32(c, 1, 8)
#define CFG_TIMER_EN        BIT(9)
#define CFG_RW_MASK         0x3ff
#define CFG_RESET           ((255u << 1) | CFG_TIMER_EN)
#define DATE_CLK_EN         BIT(31)
#define DATE_RESET          2425091

static bool trng_running(ESP32P4TrngState *s)
{
    return (s->date & DATE_CLK_EN) && (s->rstn & 1) &&
           clock_is_enabled(s->clk);
}

static bool trng_sampling(ESP32P4TrngState *s)
{
    return trng_running(s) && (s->cfg & CFG_SAMPLE_ENABLE) &&
           (s->cfg & CFG_TIMER_EN);
}

/* The samples taken since sample_ns, at the current configuration. */
static uint32_t trng_new_samples(ESP32P4TrngState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!trng_sampling(s) || now <= s->sample_ns) {
        return 0;
    }
    return clock_ns_to_ticks(s->clk, now - s->sample_ns) /
           (CFG_PSCALE(s->cfg) + 1);
}

/* Count the samples so far, before what paces them changes. */
static void trng_settle(ESP32P4TrngState *s)
{
    s->samples += trng_new_samples(s);
    s->sample_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
uint32_t esp32p4_trng_read(ESP32P4TrngState *s)
{
    if (trng_running(s)) {
        qemu_guest_getrandom_nofail(&s->data, sizeof(s->data));
    }
    return s->data;
}

static uint64_t trng_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4TrngState *s = opaque;
    uint32_t v;

    switch (addr & ~3) {
    case A_CFG:
        v = deposit32(s->cfg, 24, 8, s->samples + trng_new_samples(s));
        break;
    case A_DATA:
        v = esp32p4_trng_read(s);
        break;
    case A_RSTN:
        v = s->rstn;
        break;
    case A_DATE:
        v = s->date;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TRNG, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void trng_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4TrngState *s = opaque;
    int shift = (addr & 3) * 8;

    trng_settle(s);
    switch (addr & ~3) {
    case A_CFG:
        s->cfg = deposit32(s->cfg, shift, size * 8, value) & CFG_RW_MASK;
        break;
    case A_RSTN:
        s->rstn = deposit32(s->rstn, shift, size * 8, value) & 1;
        if (!(s->rstn & 1)) {
            s->data = 0;
            s->samples = 0;
        }
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value);
        break;
    case A_DATA:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only RNG_DATA_REG\n",
                      TYPE_ESP32P4_TRNG);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TRNG, addr);
        break;
    }
}

static const MemoryRegionOps trng_ops = {
    .read = trng_read,
    .write = trng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void trng_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4TrngState *s = opaque;

    if (event == ClockPreUpdate) {
        trng_settle(s);
    } else {
        s->sample_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static void trng_reset_hold(Object *obj, ResetType type)
{
    ESP32P4TrngState *s = ESP32P4_TRNG(obj);

    s->cfg = CFG_RESET;
    s->data = 0;
    s->rstn = 1;
    s->date = DATE_RESET;
    s->samples = 0;
    s->sample_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void trng_init(Object *obj)
{
    ESP32P4TrngState *s = ESP32P4_TRNG(obj);

    memory_region_init_io(&s->iomem, obj, &trng_ops, s, "esp32p4.lp-trng",
                          ESP32P4_TRNG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", trng_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
}

static const VMStateDescription vmstate_esp32p4_trng = {
    .name = TYPE_ESP32P4_TRNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, ESP32P4TrngState),
        VMSTATE_UINT32(data, ESP32P4TrngState),
        VMSTATE_UINT32(rstn, ESP32P4TrngState),
        VMSTATE_UINT32(date, ESP32P4TrngState),
        VMSTATE_UINT32(samples, ESP32P4TrngState),
        VMSTATE_INT64(sample_ns, ESP32P4TrngState),
        VMSTATE_END_OF_LIST()
    },
};

static void trng_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_esp32p4_trng;
    rc->phases.hold = trng_reset_hold;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo trng_info = {
    .name          = TYPE_ESP32P4_TRNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4TrngState),
    .instance_init = trng_init,
    .class_init    = trng_class_init,
};

static void trng_register_types(void)
{
    type_register_static(&trng_info);
}
type_init(trng_register_types)
