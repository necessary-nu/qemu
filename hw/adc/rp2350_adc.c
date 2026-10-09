/*
 * RP2350 analogue-digital converter (ADC) and temperature sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "ADC and Temperature Sensor", and the
 * pico-sdk adc.h register descriptions.
 *
 * A 12-bit SAR ADC behind a five- or nine-way input mux, clocked by
 * clk_adc. A conversion samples its input when it starts and completes 96
 * clk_adc cycles later, when RESULT is updated, the sample is pushed to
 * the eight-entry FIFO if FCS.EN is set, and, with CS.RROBIN non-zero,
 * AINSEL moves on to the next channel whose RROBIN bit is set.
 * CS.START_ONCE starts one conversion; CS.START_MANY starts one whenever
 * the ADC is ready (DIV = 0), or at each tick of the pacing timer, which
 * ticks every DIV.INT + 1 + DIV.FRAC / 256 cycles on average (first-order
 * delta-sigma: each interval is INT + 1 or INT + 2 cycles). A tick while a
 * conversion is in progress is ignored. The pacing timer starts with a
 * tick, so the first conversion starts when START_MANY is set.
 *
 * The ideal transfer function is used: code = floor(4096 * V / AVDD),
 * clamped to 0..4095. External inputs read the voltage of their pin; the
 * temperature sensor reads Vbe = 0.706 V - 1.721 mV/C * (T - 27 C) while
 * CS.TS_EN biases it, and nothing (0 V) otherwise.
 *
 * Conversions are not ticked: the block holds the cycle its conversion in
 * progress ends and is run forward whenever it is observed, with a timer
 * at each conversion end and each paced start.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/adc/rp2350_adc.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "migration/vmstate.h"

#define A_CS                0x00
#define A_RESULT            0x04
#define A_FCS               0x08
#define A_FIFO              0x0c
#define A_DIV               0x10
#define A_INTR              0x14
#define A_INTE              0x18
#define A_INTF              0x1c
#define A_INTS              0x20

#define CS_EN               (1u << 0)
#define CS_TS_EN            (1u << 1)
#define CS_START_ONCE       (1u << 2)
#define CS_START_MANY       (1u << 3)
#define CS_READY            (1u << 8)
#define CS_ERR              (1u << 9)
#define CS_ERR_STICKY       (1u << 10)
#define CS_AINSEL_SHIFT     12
#define CS_AINSEL           (0xfu << CS_AINSEL_SHIFT)
#define CS_RROBIN_SHIFT     16
#define CS_RROBIN           (0x1ffu << CS_RROBIN_SHIFT)
#define CS_RW               (CS_EN | CS_TS_EN | CS_START_MANY | CS_AINSEL | \
                             CS_RROBIN)

#define FCS_EN              (1u << 0)
#define FCS_SHIFT           (1u << 1)
#define FCS_ERR             (1u << 2)
#define FCS_DREQ_EN         (1u << 3)
#define FCS_EMPTY           (1u << 8)
#define FCS_FULL            (1u << 9)
#define FCS_UNDER           (1u << 10)
#define FCS_OVER            (1u << 11)
#define FCS_LEVEL_SHIFT     16
#define FCS_THRESH_SHIFT    24
#define FCS_THRESH          (0xfu << FCS_THRESH_SHIFT)
#define FCS_RW              (FCS_EN | FCS_SHIFT | FCS_ERR | FCS_DREQ_EN | \
                             FCS_THRESH)
#define FCS_WC              (FCS_UNDER | FCS_OVER)

#define FIFO_ERR            (1u << 15)
#define RESULT_MASK         0xfffu
#define DIV_MASK            0xffffffu
#define DIV_INT(v)          ((v) >> 8)
#define DIV_FRAC(v)         ((v) & 0xff)

/* clk_adc cycles per conversion. */
#define ADC_CONV_CYCLES     96

/* Temperature sensor: Vbe at 27 C, and its slope, in uV and uV per C. */
#define TS_VBE_27C_UV       706000
#define TS_SLOPE_UV_PER_C   1721

/* The first GPIO of the external inputs, by package. */
#define QFN60_FIRST_PIN     26
#define QFN80_FIRST_PIN     40

static uint64_t adc_cycle_at(RP2350ADCState *s, int64_t ns)
{
    if (ns <= s->base_ns) {
        return s->base_cycle;
    }
    return s->base_cycle + clock_ns_to_ticks(s->clk, ns - s->base_ns);
}

/* The first virtual time at which clk_adc has reached `cycle`. */
static int64_t adc_ns_at(RP2350ADCState *s, uint64_t cycle)
{
    uint64_t d, ns;

    if (cycle <= s->base_cycle) {
        return s->base_ns;
    }
    d = cycle - s->base_cycle;
    ns = clock_ticks_to_ns(s->clk, d);
    while (clock_ns_to_ticks(s->clk, ns) < d) {
        ns++;
    }
    return s->base_ns + MIN(ns, (uint64_t)INT64_MAX / 2);
}

static uint64_t adc_now(RP2350ADCState *s)
{
    return adc_cycle_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static uint32_t adc_ainsel(RP2350ADCState *s)
{
    return (s->cs & CS_AINSEL) >> CS_AINSEL_SHIFT;
}

static uint32_t adc_thresh(RP2350ADCState *s)
{
    return (s->fcs & FCS_THRESH) >> FCS_THRESH_SHIFT;
}

static bool adc_running(RP2350ADCState *s)
{
    return (s->cs & CS_EN) && clock_is_enabled(s->clk);
}

/* Whether free-running sampling starts conversions. */
static bool adc_free_running(RP2350ADCState *s)
{
    return (s->cs & CS_START_MANY) && adc_running(s);
}

/*
 * The FIFO's IRQ and DREQ condition: the FIFO holds data, at least THRESH
 * samples of it. INTR resets to 0, so an empty FIFO never meets THRESH 0.
 */
static bool adc_fifo_irq(RP2350ADCState *s)
{
    return s->fifo_level && s->fifo_level >= adc_thresh(s);
}

/* [spec:nuos:req:emu.adc] */
static void adc_update(RP2350ADCState *s)
{
    bool fifo = adc_fifo_irq(s);

    qemu_set_irq(s->irq, (fifo && s->inte) || s->intf);
    qemu_set_irq(s->dreq, fifo && (s->fcs & FCS_DREQ_EN));
}

static uint32_t adc_code(RP2350ADCState *s, int64_t uv)
{
    int64_t code;

    if (uv <= 0 || !s->avdd_uv) {
        return 0;
    }
    code = uv * 4096 / s->avdd_uv;
    return MIN(code, RESULT_MASK);
}

/*
 * The voltage on channel `ch`, in microvolts. The datasheet requires an
 * ADC pin's digital input to be disabled (pad IE low) and its output
 * driver off. A pin with its input enabled still converts, and the misuse
 * is reported; a pin the chip drives reads the level it drives.
 */
/* [spec:nuos:req:emu.adc] */
static int64_t adc_input_uv(RP2350ADCState *s, uint32_t ch)
{
    uint32_t first = s->ext_channels == RP2350_ADC_MAX_EXT ? QFN80_FIRST_PIN
                                                           : QFN60_FIRST_PIN;

    if (ch < s->ext_channels) {
        int pin = first + ch;
        int drive = s->gpio ? rp2350_gpio_pin_drive(s->gpio, pin) : -1;

        if (s->gpio && rp2350_gpio_pin_input_enabled(s->gpio, pin)) {
            if (!(s->reported & (1u << ch))) {
                s->reported |= 1u << ch;
                qemu_log_mask(LOG_GUEST_ERROR, "rp2350-adc: channel %u: "
                              "GPIO %d has its digital input enabled\n",
                              ch, pin);
            }
        } else {
            s->reported &= ~(1u << ch);
        }
        if (drive >= 0) {
            return drive ? s->iovdd_uv : 0;
        }
        return s->ain_uv[ch];
    }
    if (ch == s->ext_channels) {
        if (!(s->cs & CS_TS_EN)) {
            if (!(s->reported & (1u << ch))) {
                s->reported |= 1u << ch;
                qemu_log_mask(LOG_GUEST_ERROR, "rp2350-adc: temperature "
                              "sensor converted without its bias "
                              "(CS.TS_EN)\n");
            }
            return 0;
        }
        s->reported &= ~(1u << ch);
        return TS_VBE_27C_UV -
               ((int64_t)s->temp_mc - 27000) * TS_SLOPE_UV_PER_C / 1000;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-adc: AINSEL %u selects no input "
                  "in this package\n", ch);
    return 0;
}

/* Start a conversion of the selected channel in cycle `cycle`. */
static void adc_start(RP2350ADCState *s, uint64_t cycle)
{
    s->busy = true;
    s->conv_ch = adc_ainsel(s);
    s->conv_code = adc_code(s, adc_input_uv(s, s->conv_ch));
    s->conv_err = s->err_channels & (1u << s->conv_ch);
    s->conv_end = cycle + ADC_CONV_CYCLES;
}

static void adc_fifo_push(RP2350ADCState *s, uint16_t v)
{
    if (s->fifo_level == RP2350_ADC_FIFO_DEPTH) {
        /* A full FIFO keeps its contents; the sample is lost. */
        s->fcs |= FCS_OVER;
        return;
    }
    s->fifo[(s->fifo_head + s->fifo_level) % RP2350_ADC_FIFO_DEPTH] = v;
    s->fifo_level++;
}

/* The next channel after `ch` whose RROBIN bit is set. */
static uint32_t adc_rrobin_next(uint32_t rrobin, uint32_t ch)
{
    int i;

    for (i = 1; i <= 16; i++) {
        uint32_t next = (ch + i) % 16;

        if (rrobin & (1u << next)) {
            return next;
        }
    }
    return ch;
}

/* [spec:nuos:req:emu.adc] */
static void adc_complete(RP2350ADCState *s)
{
    uint32_t rrobin = (s->cs & CS_RROBIN) >> CS_RROBIN_SHIFT;

    s->busy = false;
    s->idle_since = s->conv_end;
    s->result = s->conv_code;
    if (s->conv_err) {
        s->cs |= CS_ERR | CS_ERR_STICKY;
    } else {
        s->cs &= ~CS_ERR;
    }
    if (s->fcs & FCS_EN) {
        uint16_t v = (s->fcs & FCS_SHIFT) ? s->conv_code >> 4 : s->conv_code;

        if ((s->fcs & FCS_ERR) && s->conv_err) {
            v |= FIFO_ERR;
        }
        adc_fifo_push(s, v);
    }
    if (rrobin) {
        s->cs = (s->cs & ~CS_AINSEL) |
                adc_rrobin_next(rrobin, adc_ainsel(s)) << CS_AINSEL_SHIFT;
    }
}

/* The pacing timer's tick interval, in 1/256ths of a cycle. */
static uint64_t adc_period(RP2350ADCState *s)
{
    return ((uint64_t)DIV_INT(s->div) + 1) * 256 + DIV_FRAC(s->div);
}

/* The cycle of pacing timer tick k, counting from the tick at pace_base. */
static uint64_t adc_tick(RP2350ADCState *s, uint64_t k)
{
    return s->pace_base + k * adc_period(s) / 256;
}

/* The index of the first pacing timer tick at or after cycle `c`. */
static uint64_t adc_tick_index(RP2350ADCState *s, uint64_t c)
{
    if (c <= s->pace_base) {
        return 0;
    }
    return DIV_ROUND_UP((c - s->pace_base) * 256, adc_period(s));
}

/* The cycle the next free-running conversion starts in. */
static uint64_t adc_next_start(RP2350ADCState *s)
{
    if (!s->div) {
        return s->idle_since;
    }
    return adc_tick(s, adc_tick_index(s, MAX(s->idle_since,
                                             s->pace_base)));
}

/*
 * Run the ADC up to clk_adc cycle `now`. The FIFO's interrupt and DREQ
 * follow each completed conversion, so a DMA channel paced by the DREQ
 * drains the FIFO between conversions however far the block is run.
 */
/* [spec:nuos:req:emu.adc] */
static void adc_sync(RP2350ADCState *s, uint64_t now)
{
    if (s->syncing) {
        return;
    }
    s->syncing = true;
    for (;;) {
        if (s->busy) {
            if (s->conv_end > now) {
                break;
            }
            adc_complete(s);
            adc_update(s);
        } else {
            uint64_t start;

            if (!adc_free_running(s)) {
                break;
            }
            start = adc_next_start(s);
            if (start > now) {
                break;
            }
            adc_start(s, start);
        }
    }
    s->syncing = false;
}

static void adc_schedule(RP2350ADCState *s)
{
    if (s->busy && adc_running(s)) {
        timer_mod(s->timer, adc_ns_at(s, s->conv_end));
    } else if (adc_free_running(s)) {
        timer_mod(s->timer, adc_ns_at(s, adc_next_start(s)));
    } else {
        timer_del(s->timer);
    }
}

static void adc_timer_cb(void *opaque)
{
    RP2350ADCState *s = opaque;

    adc_sync(s, adc_now(s));
    adc_update(s);
    adc_schedule(s);
}

/* Restart the pacing timer (and the ready time) at cycle `now`. */
static void adc_pace_restart(RP2350ADCState *s, uint64_t now)
{
    s->pace_base = now;
    if (!s->busy) {
        s->idle_since = now;
    }
}

static void adc_write_cs(RP2350ADCState *s, hwaddr addr, uint32_t value,
                         uint64_t now)
{
    uint32_t old = s->cs;
    uint32_t v = rp2350_atomic_apply(addr, old & ~CS_ERR_STICKY, value);
    bool was_running = adc_free_running(s);

    s->cs = (old & ~CS_RW) | (v & CS_RW);
    /* ERR_STICKY is write-1-to-clear: see the FCS flags. */
    switch (addr / RP2350_ATOMIC_ALIAS_SIZE) {
    case 0:
    case 2:
        s->cs &= ~(value & CS_ERR_STICKY);
        break;
    default:
        break;
    }

    if (!(s->cs & CS_EN)) {
        /* Powering down abandons a conversion in progress. */
        s->busy = false;
    }
    if (adc_free_running(s) && !was_running) {
        adc_pace_restart(s, now);
    }
    /* START_ONCE is ignored while free-running or converting. */
    if ((v & CS_START_ONCE) && adc_running(s) && !(s->cs & CS_START_MANY) &&
        !s->busy) {
        adc_start(s, now);
    }
}

/* [spec:nuos:req:emu.adc] */
static uint32_t adc_read_reg(RP2350ADCState *s, hwaddr addr)
{
    uint32_t v = 0;

    adc_sync(s, adc_now(s));
    switch (rp2350_atomic_reg(addr)) {
    case A_CS:
        v = s->cs;
        if ((s->cs & CS_EN) && !s->busy) {
            v |= CS_READY;
        }
        break;
    case A_RESULT:
        v = s->result;
        break;
    case A_FCS:
        v = s->fcs | s->fifo_level << FCS_LEVEL_SHIFT;
        if (!s->fifo_level) {
            v |= FCS_EMPTY;
        }
        if (s->fifo_level == RP2350_ADC_FIFO_DEPTH) {
            v |= FCS_FULL;
        }
        break;
    case A_FIFO:
        if (!s->fifo_level) {
            s->fcs |= FCS_UNDER;
            break;
        }
        v = s->fifo[s->fifo_head];
        s->fifo_head = (s->fifo_head + 1) % RP2350_ADC_FIFO_DEPTH;
        s->fifo_level--;
        break;
    case A_DIV:
        v = s->div;
        break;
    case A_INTR:
        v = adc_fifo_irq(s);
        break;
    case A_INTE:
        v = s->inte;
        break;
    case A_INTF:
        v = s->intf;
        break;
    case A_INTS:
        v = (adc_fifo_irq(s) && s->inte) || s->intf;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-adc: read of bad offset 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
    adc_update(s);
    adc_schedule(s);
    return v;
}

/*
 * Narrow reads read the whole register and return their byte lanes, so
 * a byte or halfword read of FIFO (DMA into a byte buffer with FCS.SHIFT)
 * pops one sample.
 */
static uint64_t rp2350_adc_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t v = adc_read_reg(opaque, addr & ~3);

    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.adc] */
static void adc_write_reg(RP2350ADCState *s, hwaddr addr, uint32_t value)
{
    uint64_t now = adc_now(s);

    adc_sync(s, now);
    switch (rp2350_atomic_reg(addr)) {
    case A_CS:
        adc_write_cs(s, addr, value, now);
        break;
    case A_FCS:
        s->fcs = (s->fcs & ~FCS_RW) |
                 (rp2350_atomic_apply(addr, s->fcs, value) & FCS_RW);
        /*
         * Writing 1 clears a flag. The atomic aliases act per written
         * bit: a SET alias write writes 1s, so it clears; CLR and XOR
         * alias writes never write a 1 to a set bit.
         */
        switch (addr / RP2350_ATOMIC_ALIAS_SIZE) {
        case 0:
        case 2:
            s->fcs &= ~(value & FCS_WC);
            break;
        default:
            break;
        }
        break;
    case A_DIV: {
        uint32_t div = rp2350_atomic_apply(addr, s->div, value) & DIV_MASK;

        if (div != s->div && adc_free_running(s)) {
            /*
             * The pacing timer runs on from its last tick at the new
             * interval.
             */
            uint64_t k = adc_tick_index(s, now);

            if (!s->div) {
                s->pace_base = now;
            } else if (adc_tick(s, k) > now) {
                s->pace_base = adc_tick(s, k - 1);
            } else {
                s->pace_base = now;
            }
        }
        s->div = div;
        break;
    }
    case A_INTE:
        s->inte = rp2350_atomic_apply(addr, s->inte, value) & 1;
        break;
    case A_INTF:
        s->intf = rp2350_atomic_apply(addr, s->intf, value) & 1;
        break;
    case A_RESULT:
    case A_FIFO:
    case A_INTR:
    case A_INTS:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-adc: write to bad offset 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
    adc_update(s);
    adc_schedule(s);
}

/*
 * The register ignores the width of a write: a narrow write's value is
 * replicated across the 32-bit data bus and written to the whole register.
 */
static void rp2350_adc_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    uint32_t v = value;

    if (size == 1) {
        v = (v & 0xff) * 0x01010101u;
    } else if (size == 2) {
        v = (v & 0xffff) * 0x00010001u;
    }
    adc_write_reg(opaque, addr & ~3, v);
}

static const MemoryRegionOps rp2350_adc_ops = {
    .read = rp2350_adc_read,
    .write = rp2350_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * A change of clk_adc: run the ADC to the change at the old rate, then
 * count from there at the new one.
 */
static void rp2350_adc_clk_update(void *opaque, ClockEvent event)
{
    RP2350ADCState *s = opaque;

    if (event == ClockPreUpdate) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t cycle = adc_cycle_at(s, now);

        adc_sync(s, cycle);
        s->base_ns = now;
        s->base_cycle = cycle;
    } else {
        adc_update(s);
        adc_schedule(s);
    }
}

static void rp2350_adc_hold_reset(Object *obj, ResetType type)
{
    RP2350ADCState *s = RP2350_ADC(obj);

    timer_del(s->timer);
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_cycle = 0;
    s->cs = 0;
    s->result = 0;
    s->fcs = 0;
    memset(s->fifo, 0, sizeof(s->fifo));
    s->fifo_head = 0;
    s->fifo_level = 0;
    s->div = 0;
    s->inte = 0;
    s->intf = 0;
    s->busy = false;
    s->conv_ch = 0;
    s->conv_code = 0;
    s->conv_err = false;
    s->conv_end = 0;
    s->pace_base = 0;
    s->idle_since = 0;
    s->reported = 0;
    adc_update(s);
}

static void adc_get_int32(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    visit_type_int32(v, name, opaque, errp);
}

static void adc_set_int32(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    visit_type_int32(v, name, opaque, errp);
}

static void adc_get_uint32(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    visit_type_uint32(v, name, opaque, errp);
}

static void adc_set_uint32(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    visit_type_uint32(v, name, opaque, errp);
}

static void rp2350_adc_init(Object *obj)
{
    RP2350ADCState *s = RP2350_ADC(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &rp2350_adc_ops, s, TYPE_RP2350_ADC,
                          RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, &s->dreq, RP2350_ADC_DREQ, 1);
    s->clk = qdev_init_clock_in(dev, "clk", rp2350_adc_clk_update, s,
                                ClockPreUpdate | ClockUpdate);

    /*
     * The analogue world outside the chip: not device state software can
     * reset, so these are set once here and only ever by the user after.
     */
    s->temp_mc = 27000;
    s->avdd_uv = 3300000;
    s->iovdd_uv = 3300000;
    for (i = 0; i < RP2350_ADC_MAX_EXT; i++) {
        g_autofree char *name = g_strdup_printf("ain%d-uv", i);

        object_property_add(obj, name, "int32", adc_get_int32, adc_set_int32,
                            NULL, &s->ain_uv[i]);
    }
    object_property_add(obj, "temperature", "int32", adc_get_int32,
                        adc_set_int32, NULL, &s->temp_mc);
    object_property_add(obj, "avdd-uv", "uint32", adc_get_uint32,
                        adc_set_uint32, NULL, &s->avdd_uv);
    object_property_add(obj, "iovdd-uv", "uint32", adc_get_uint32,
                        adc_set_uint32, NULL, &s->iovdd_uv);
    object_property_add(obj, "conversion-errors", "uint32", adc_get_uint32,
                        adc_set_uint32, NULL, &s->err_channels);
}

static void rp2350_adc_realize(DeviceState *dev, Error **errp)
{
    RP2350ADCState *s = RP2350_ADC(dev);

    if (s->ext_channels != 4 && s->ext_channels != RP2350_ADC_MAX_EXT) {
        error_setg(errp, "external-channels must be 4 (QFN-60) or 8 (QFN-80)");
        return;
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, adc_timer_cb, s);
}

static const Property rp2350_adc_properties[] = {
    DEFINE_PROP_UINT32("external-channels", RP2350ADCState, ext_channels, 4),
    DEFINE_PROP_LINK("gpio", RP2350ADCState, gpio, TYPE_RP2350_GPIO,
                     RP2350GPIOState *),
};

static const VMStateDescription vmstate_rp2350_adc = {
    .name = TYPE_RP2350_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, RP2350ADCState),
        VMSTATE_TIMER_PTR(timer, RP2350ADCState),
        VMSTATE_INT32_ARRAY(ain_uv, RP2350ADCState, RP2350_ADC_MAX_EXT),
        VMSTATE_INT32(temp_mc, RP2350ADCState),
        VMSTATE_UINT32(avdd_uv, RP2350ADCState),
        VMSTATE_UINT32(iovdd_uv, RP2350ADCState),
        VMSTATE_UINT32(err_channels, RP2350ADCState),
        VMSTATE_UINT32(cs, RP2350ADCState),
        VMSTATE_UINT32(result, RP2350ADCState),
        VMSTATE_UINT32(fcs, RP2350ADCState),
        VMSTATE_UINT16_ARRAY(fifo, RP2350ADCState, RP2350_ADC_FIFO_DEPTH),
        VMSTATE_UINT32(fifo_head, RP2350ADCState),
        VMSTATE_UINT32(fifo_level, RP2350ADCState),
        VMSTATE_UINT32(div, RP2350ADCState),
        VMSTATE_UINT32(inte, RP2350ADCState),
        VMSTATE_UINT32(intf, RP2350ADCState),
        VMSTATE_INT64(base_ns, RP2350ADCState),
        VMSTATE_UINT64(base_cycle, RP2350ADCState),
        VMSTATE_BOOL(busy, RP2350ADCState),
        VMSTATE_UINT32(conv_ch, RP2350ADCState),
        VMSTATE_UINT32(conv_code, RP2350ADCState),
        VMSTATE_BOOL(conv_err, RP2350ADCState),
        VMSTATE_UINT64(conv_end, RP2350ADCState),
        VMSTATE_UINT64(pace_base, RP2350ADCState),
        VMSTATE_UINT64(idle_since, RP2350ADCState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_adc_realize;
    rc->phases.hold = rp2350_adc_hold_reset;
    dc->vmsd = &vmstate_rp2350_adc;
    device_class_set_props(dc, rp2350_adc_properties);
}

/* [spec:nuos:req:emu.adc] */
static const TypeInfo rp2350_adc_info = {
    .name          = TYPE_RP2350_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350ADCState),
    .instance_init = rp2350_adc_init,
    .class_init    = rp2350_adc_class_init,
};

static void rp2350_adc_register_types(void)
{
    type_register_static(&rp2350_adc_info);
}

type_init(rp2350_adc_register_types)
