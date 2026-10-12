/*
 * ESP32-P4 temperature sensor (LP_TSENS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 64 (Temperature Sensor); ESP-IDF's tsens_reg.h for v3, the authority
 * for the registers, their reset values and access types, with the TRM's
 * TSENS_CTRL2 (0x004), which the header leaves out; regi2c_saradc.h for
 * I2C_SARADC_TSENS_DAC; and its hal temperature_sensor_ll.h,
 * temperature_sensor_periph.c and esp_driver_tsens, which show how
 * software drives the sensor and converts its output.
 *
 * The sensor runs while TSENS_POWER_UP powers it, or TSENS_XPD_FORCE
 * forces it on (2), unless TSENS_XPD_FORCE forces it off (3). From
 * power-up it waits TSENS_XPD_WAIT cycles of its clock, LP_PERI_CLK, then
 * measures continuously: TSENS_READY reads 1 and TSENS_OUT follows the
 * die temperature, inverted by TSENS_IN_INV, and holds its last value once
 * the sensor is powered down. The TRM's settling of the output over the
 * first 100 us is not modelled: it reads the temperature at once.
 *
 * The output is the TRM's formula, T = 0.4386 VALUE - 27.88 offset -
 * 20.52, inverted, where the offset is that of the measurement range
 * I2C_SARADC_TSENS_DAC selects (TRM table 64.4-1, ESP-IDF's
 * temperature_sensor_attributes: DAC 5, 7, 15, 11 and 10 for offsets -2 to
 * 2), with the chip's own error, the eFuse's TEMPERATURE_SENSOR in tenths
 * of a degree, which ESP-IDF subtracts, added. A DAC setting outside the
 * table is logged and measures as offset 0. VALUE saturates at 0 and 255.
 *
 * The automatic monitor samples every TSENS_SAMPLE_RATE cycles of the
 * clock while sampling is on: from TSENS_SAMPLE_EN set, or the ETM task
 * TMPSNSR_TASK_START_SAMPLE, to TSENS_SAMPLE_EN cleared, or
 * TMPSNSR_TASK_STOP_SAMPLE. With TSENS_WAKEUP_EN, a sample above
 * TSENS_WAKEUP_TH_HIGH or below _TH_LOW (TSENS_WAKEUP_MODE 0), or rising
 * or falling by more than them since the last (1), records which in
 * TSENS_WAKEUP_OVER_UPPER_TH, raises TSENS_COCPU_TSENS_WAKE_INT while
 * TSENS_INT_EN, and pulses TMPSNSR_EVT_OVER_LIMIT. Samples that would do
 * nothing, all of them while the temperature stays between the thresholds
 * or does not change, are not run, so that the sensor costs nothing while
 * it watches a steady temperature; nor are the samples a timer running
 * late on the host's clock has missed (without icount), which would
 * otherwise come all at once.
 *
 * Kept as written: TSENS_CLK_DIV, TSENS_CLK_INV, TSENS_POWER_UP_FORCE,
 * TSENS_WAKEUP_MASK and TSENS_CLK_EN, which change nothing the model
 * shows.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_ana.h"
#include "hw/misc/esp32p4_tsens.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "migration/vmstate.h"

#define A_CTRL                  0x00
#define A_CTRL2                 0x04
#define A_INT_RAW               0x08
#define A_INT_ST                0x0c
#define A_INT_ENA               0x10
#define A_INT_CLR               0x14
#define A_CLK_CONF              0x18
#define A_INT_ENA_W1TS          0x1c
#define A_INT_ENA_W1TC          0x20
#define A_WAKEUP_CTRL           0x24
#define A_SAMPLE_RATE           0x28

/* CTRL */
#define CTRL_OUT_MASK           0xffu
#define CTRL_READY              BIT(8)
#define CTRL_SAMPLE_EN          BIT(9)
#define CTRL_INT_EN             BIT(12)
#define CTRL_IN_INV             BIT(13)
#define CTRL_POWER_UP           BIT(22)
/* CTRL2 */
#define CTRL2_XPD_WAIT(v)       extract32((v), 0, 12)
#define CTRL2_XPD_FORCE(v)      extract32((v), 12, 2)
#define XPD_FORCE_ON            2
#define XPD_FORCE_OFF           3
/* INT_* */
#define INT_WAKE                BIT(0)
/* WAKEUP_CTRL */
#define WAKEUP_TH_LOW(v)        extract32((v), 0, 8)
#define WAKEUP_TH_HIGH(v)       extract32((v), 14, 8)
#define WAKEUP_OVER_UPPER_TH    BIT(29)
#define WAKEUP_EN               BIT(30)
#define WAKEUP_MODE             BIT(31)
/* SAMPLE_RATE */
#define SAMPLE_RATE(v)          extract32((v), 0, 16)

#define REG(s, a)               ((s)->regs[(a) / 4])

/* The analog sensors' register holding I2C_SARADC_TSENS_DAC (bits 3:0). */
#define ANA_TSENS_DAC           6
/* The eFuse's TEMPERATURE_SENSOR: BLOCK10 bits 96 to 105, sign in bit 8. */
#define EFUSE_TSENS_BLOCK       10
#define EFUSE_TSENS_WORD        3

typedef struct TsensReg {
    uint32_t reset;
    uint32_t rw;
} TsensReg;

static const TsensReg tsens_regs[ESP32P4_TSENS_NREGS] = {
    [A_CTRL / 4] = { 0x00019400, 0x00fff600 },
    [A_CTRL2 / 4] = { 0x00004002, 0x00007fff },
    [A_INT_RAW / 4] = { 0, 0 },
    [A_INT_ST / 4] = { 0, 0 },
    [A_INT_ENA / 4] = { 0, INT_WAKE },
    [A_INT_CLR / 4] = { 0, 0 },
    [A_CLK_CONF / 4] = { 0, 0x00000001 },
    [A_INT_ENA_W1TS / 4] = { 0, 0 },
    [A_INT_ENA_W1TC / 4] = { 0, 0 },
    [A_WAKEUP_CTRL / 4] = { 0x003fc000, 0xc03fc0ff },
    [A_SAMPLE_RATE / 4] = { 0x00000014, 0x0000ffff },
};

/* The measurement ranges' DAC settings and offsets (TRM table 64.4-1). */
static const struct {
    uint8_t dac;
    int8_t offset;
} tsens_ranges[] = {
    { 5, -2 }, { 7, -1 }, { 15, 0 }, { 11, 1 }, { 10, 2 },
};

static void tsens_update_irq(ESP32P4TsensState *s)
{
    qemu_set_irq(s->irq, !!(REG(s, A_INT_RAW) & REG(s, A_INT_ENA)));
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The sensor's output for the die temperature in the range the DAC
 * selects (see the top of the file).
 */
static uint32_t tsens_measure(ESP32P4TsensState *s)
{
    uint8_t dac = s->ana->regs[ESP32P4_ANA_SLAVE_SAR][ANA_TSENS_DAC] & 0xf;
    uint32_t cal = extract32(esp32p4_efuse_word(s->efuse, EFUSE_TSENS_BLOCK,
                                                EFUSE_TSENS_WORD), 0, 9);
    int err = cal & BIT(8) ? -(int)(cal & 0xff) : (int)(cal & 0xff);
    int offset = 0;
    bool found = false;
    double v;
    long code;

    for (unsigned i = 0; i < ARRAY_SIZE(tsens_ranges); i++) {
        if (tsens_ranges[i].dac == dac) {
            offset = tsens_ranges[i].offset;
            found = true;
        }
    }
    if (!found) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: I2C_SARADC_TSENS_DAC %u selects "
                      "no measurement range the TRM gives; the sensor "
                      "measures as with offset 0\n", TYPE_ESP32P4_TSENS, dac);
    }
    v = (s->temp_mc / 1000.0 + err / 10.0 + 27.88 * offset + 20.52) / 0.4386;
    code = MIN(MAX(lround(v), 0), 255);
    if (REG(s, A_CTRL) & CTRL_IN_INV) {
        code = ~code & 0xff;
    }
    return code;
}

/* TSENS_OUT: the live measurement once ready, else the last one. */
static uint32_t tsens_out(ESP32P4TsensState *s)
{
    uint32_t *ctrl = &REG(s, A_CTRL);

    if (s->ready) {
        *ctrl = (*ctrl & ~CTRL_OUT_MASK) | tsens_measure(s);
    }
    return *ctrl & CTRL_OUT_MASK;
}

static bool tsens_monitoring(ESP32P4TsensState *s)
{
    return s->sampling && s->ready && clock_is_enabled(s->clk) &&
           (REG(s, A_WAKEUP_CTRL) & WAKEUP_EN) &&
           SAMPLE_RATE(REG(s, A_SAMPLE_RATE));
}

/* Whether a sample of @v would do nothing. */
static bool tsens_quiet(ESP32P4TsensState *s, uint32_t v)
{
    uint32_t w = REG(s, A_WAKEUP_CTRL);

    if (w & WAKEUP_MODE) {
        return s->prev_valid && v == s->prev;
    }
    return v >= WAKEUP_TH_LOW(w) && v <= WAKEUP_TH_HIGH(w);
}

/*
 * Arm the timer for the first sample after @now on the sampling period's
 * grid that would do something: sample k comes as the clock completes
 * k periods of SAMPLE_RATE cycles from the anchor, at its edges (a rate of
 * 0 samples every nanosecond).
 */
static void tsens_schedule(ESP32P4TsensState *s, int64_t now)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->clk, s->anchor_ns);
    uint64_t rate = SAMPLE_RATE(REG(s, A_SAMPLE_RATE));
    uint64_t k;

    if (!tsens_monitoring(s) || tsens_quiet(s, tsens_out(s))) {
        timer_del(&s->sample_timer);
        return;
    }
    if (!rate) {
        s->next_ns = MAX(now, s->anchor_ns) + 1;
    } else {
        k = esp32p4_clk_edges_between(&e, s->anchor_ns, now) / rate + 1;
        s->next_ns = esp32p4_clk_edges_after(&e, s->anchor_ns, k * rate);
    }
    timer_mod(&s->sample_timer, s->next_ns);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void tsens_sample(void *opaque)
{
    ESP32P4TsensState *s = opaque;
    int64_t when = s->next_ns;
    uint32_t *w = &REG(s, A_WAKEUP_CTRL);
    uint32_t v = tsens_out(s);
    uint32_t hi = WAKEUP_TH_HIGH(*w), lo = WAKEUP_TH_LOW(*w);
    bool wake = false, upper = false;

    if (*w & WAKEUP_MODE) {
        if (s->prev_valid) {
            if (v > s->prev && v - s->prev > hi) {
                wake = upper = true;
            } else if (v < s->prev && s->prev - v > lo) {
                wake = true;
            }
        }
    } else if (v > hi) {
        wake = upper = true;
    } else if (v < lo) {
        wake = true;
    }
    s->prev = v;
    s->prev_valid = true;
    if (wake) {
        *w = (*w & ~WAKEUP_OVER_UPPER_TH) | (upper ? WAKEUP_OVER_UPPER_TH : 0);
        if (REG(s, A_CTRL) & CTRL_INT_EN) {
            REG(s, A_INT_RAW) |= INT_WAKE;
            tsens_update_irq(s);
        }
        esp32_line_set_at(s->etm_event, 1, when);
        esp32_line_set_at(s->etm_event, 0, when);
    }
    /* A timer running late on the host's clock skips what it has missed. */
    tsens_schedule(s, MAX(when, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
}

static void tsens_set_sampling(ESP32P4TsensState *s, bool on, int64_t when)
{
    if (on && !s->sampling) {
        s->anchor_ns = when;
        s->prev_valid = false;
    }
    s->sampling = on;
    tsens_schedule(s, when);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void tsens_update_power(ESP32P4TsensState *s, int64_t now)
{
    uint32_t force = CTRL2_XPD_FORCE(REG(s, A_CTRL2));
    bool on = force == XPD_FORCE_ON ||
              (force != XPD_FORCE_OFF && (REG(s, A_CTRL) & CTRL_POWER_UP));
    uint32_t wait = CTRL2_XPD_WAIT(REG(s, A_CTRL2));

    if (on == s->powered) {
        return;
    }
    if (!on) {
        tsens_out(s);
        s->ready = false;
        REG(s, A_CTRL) &= ~CTRL_READY;
        esp32p4_clkwait_stop(&s->xpd_wait);
    } else if (wait) {
        esp32p4_clkwait_start(&s->xpd_wait, now, wait);
    } else {
        s->ready = true;
        REG(s, A_CTRL) |= CTRL_READY;
    }
    s->powered = on;
    tsens_schedule(s, now);
}

static void tsens_xpd_done(void *opaque)
{
    ESP32P4TsensState *s = opaque;
    int64_t when = esp32p4_clkwait_expire(&s->xpd_wait);

    s->ready = true;
    REG(s, A_CTRL) |= CTRL_READY;
    tsens_schedule(s, when);
}

static uint64_t tsens_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4TsensState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    if (reg >= ESP32P4_TSENS_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TSENS, addr);
        return 0;
    }
    switch (reg) {
    case A_CTRL:
        tsens_out(s);
        v = REG(s, A_CTRL);
        break;
    case A_INT_ST:
        v = REG(s, A_INT_RAW) & REG(s, A_INT_ENA);
        break;
    case A_INT_CLR:
    case A_INT_ENA_W1TS:
    case A_INT_ENA_W1TC:
        v = 0;
        break;
    default:
        v = s->regs[reg / 4];
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void tsens_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    ESP32P4TsensState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t *r, old, rw;

    if (reg >= ESP32P4_TSENS_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TSENS, addr);
        return;
    }
    r = &s->regs[reg / 4];
    old = *r;
    rw = tsens_regs[reg / 4].rw;

    switch (reg) {
    case A_INT_RAW:
    case A_INT_CLR:
        REG(s, A_INT_RAW) &= ~(v & INT_WAKE);
        tsens_update_irq(s);
        return;
    case A_INT_ENA_W1TS:
        REG(s, A_INT_ENA) |= v & INT_WAKE;
        tsens_update_irq(s);
        return;
    case A_INT_ENA_W1TC:
        REG(s, A_INT_ENA) &= ~(v & INT_WAKE);
        tsens_update_irq(s);
        return;
    case A_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TSENS, addr);
        return;
    default:
        break;
    }
    *r = (*r & ~(rw & lanes)) | (v & rw);

    switch (reg) {
    case A_CTRL:
        if ((*r ^ old) & CTRL_SAMPLE_EN) {
            tsens_set_sampling(s, *r & CTRL_SAMPLE_EN, now);
        }
        tsens_update_power(s, now);
        tsens_schedule(s, now);
        break;
    case A_CTRL2:
        tsens_update_power(s, now);
        break;
    case A_INT_ENA:
        tsens_update_irq(s);
        break;
    case A_WAKEUP_CTRL:
        if ((*r ^ old) & WAKEUP_MODE) {
            s->prev_valid = false;
        }
        tsens_schedule(s, now);
        break;
    case A_SAMPLE_RATE:
        s->anchor_ns = now;
        tsens_schedule(s, now);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps tsens_ops = {
    .read = tsens_read,
    .write = tsens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * [spec:nuos:req:emu.esp32p4.etm]
 * The ETM tasks, at their events' time.
 */
static void tsens_etm_task(void *opaque, int n, int level)
{
    ESP32P4TsensState *s = opaque;

    if (level) {
        tsens_set_sampling(s, n == ESP32P4_TSENS_TASK_START_SAMPLE,
                           esp32_line_time());
    }
}

/* The range may have changed: the monitor may have something to see. */
static void tsens_dac_update(void *opaque, int n, int level)
{
    ESP32P4TsensState *s = opaque;

    if (level) {
        tsens_schedule(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static void tsens_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4TsensState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        esp32p4_clkwait_fold(&s->xpd_wait, now);
        return;
    }
    esp32p4_clkwait_resched(&s->xpd_wait);
    s->anchor_ns = now;
    tsens_schedule(s, now);
}

static void tsens_get_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    visit_type_int32(v, name, &ESP32P4_TSENS(obj)->temp_mc, errp);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void tsens_set_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    ESP32P4TsensState *s = ESP32P4_TSENS(obj);
    int32_t value;

    if (!visit_type_int32(v, name, &value, errp)) {
        return;
    }
    if (value < -273150) {
        error_setg(errp, "%s: %d m°C is below absolute zero", name, value);
        return;
    }
    s->temp_mc = value;
    if (DEVICE(obj)->realized) {
        tsens_schedule(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static void esp32p4_tsens_reset_hold(Object *obj, ResetType type)
{
    ESP32P4TsensState *s = ESP32P4_TSENS(obj);

    for (int i = 0; i < ESP32P4_TSENS_NREGS; i++) {
        s->regs[i] = tsens_regs[i].reset;
    }
    s->powered = false;
    s->ready = false;
    s->sampling = false;
    s->prev_valid = false;
    s->prev = 0;
    s->anchor_ns = 0;
    esp32p4_clkwait_stop(&s->xpd_wait);
    timer_del(&s->sample_timer);
}

static void esp32p4_tsens_reset_exit(Object *obj, ResetType type)
{
    tsens_update_irq(ESP32P4_TSENS(obj));
}

static void esp32p4_tsens_init(Object *obj)
{
    ESP32P4TsensState *s = ESP32P4_TSENS(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tsens_ops, s, "esp32p4.lp-tsens",
                          ESP32P4_TSENS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(dev, tsens_etm_task, ESP32P4_TSENS_ETM_TASK,
                            ESP32P4_TSENS_ETM_TASKS);
    qdev_init_gpio_in_named(dev, tsens_dac_update, ESP32P4_TSENS_DAC_UPDATE,
                            1);
    qdev_init_gpio_out_named(dev, &s->etm_event, ESP32P4_TSENS_ETM_EVENT, 1);
    s->clk = qdev_init_clock_in(dev, "clk", tsens_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    esp32p4_clkwait_init(&s->xpd_wait, s->clk, tsens_xpd_done, s);
    timer_init_ns(&s->sample_timer, QEMU_CLOCK_VIRTUAL, tsens_sample, s);

    s->temp_mc = 25000;
    object_property_add(obj, "temp-mc", "int32", tsens_get_temp,
                        tsens_set_temp, NULL, NULL);
    object_property_set_description(obj, "temp-mc",
        "The die temperature, in thousandths of a degree Celsius");
}

static void esp32p4_tsens_realize(DeviceState *dev, Error **errp)
{
    ESP32P4TsensState *s = ESP32P4_TSENS(dev);

    if (!s->ana || !s->efuse) {
        error_setg(errp, "%s: the ana and efuse links must be set",
                   TYPE_ESP32P4_TSENS);
    }
}

static const VMStateDescription vmstate_esp32p4_tsens = {
    .name = TYPE_ESP32P4_TSENS,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4TsensState, ESP32P4_TSENS_NREGS),
        VMSTATE_BOOL(powered, ESP32P4TsensState),
        VMSTATE_BOOL(ready, ESP32P4TsensState),
        VMSTATE_ESP32P4_CLKWAIT(xpd_wait, ESP32P4TsensState),
        VMSTATE_BOOL(sampling, ESP32P4TsensState),
        VMSTATE_INT64(anchor_ns, ESP32P4TsensState),
        VMSTATE_UINT32(prev, ESP32P4TsensState),
        VMSTATE_BOOL(prev_valid, ESP32P4TsensState),
        VMSTATE_TIMER(sample_timer, ESP32P4TsensState),
        VMSTATE_INT64(next_ns, ESP32P4TsensState),
        VMSTATE_INT32(temp_mc, ESP32P4TsensState),
        VMSTATE_CLOCK(clk, ESP32P4TsensState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32p4_tsens_properties[] = {
    DEFINE_PROP_LINK("ana", ESP32P4TsensState, ana, TYPE_ESP32P4_ANA,
                     ESP32P4AnaState *),
    DEFINE_PROP_LINK("efuse", ESP32P4TsensState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
};

static void esp32p4_tsens_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_tsens_reset_hold;
    rc->phases.exit = esp32p4_tsens_reset_exit;
    dc->realize = esp32p4_tsens_realize;
    dc->vmsd = &vmstate_esp32p4_tsens;
    device_class_set_props(dc, esp32p4_tsens_properties);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static const TypeInfo esp32p4_tsens_info = {
    .name = TYPE_ESP32P4_TSENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4TsensState),
    .instance_init = esp32p4_tsens_init,
    .class_init = esp32p4_tsens_class_init,
};

static void esp32p4_tsens_register_types(void)
{
    type_register_static(&esp32p4_tsens_info);
}

type_init(esp32p4_tsens_register_types)
