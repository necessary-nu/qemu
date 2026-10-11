/*
 * ESP32-P4 RTC timer (LP_TIMER)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 19 (RTC Timer), and ESP-IDF's lp_timer_reg.h for v3, which gives the
 * registers' layout, access and reset values; its hal rtc_timer_ll.h,
 * rtc_time.c and the LP core's lp_timer show how software drives it.
 *
 * A 48-bit counter of LP_DYN_SLOW_CLK, which in the active state is
 * LP_SLOW_CLK (RC_SLOW_CLK out of reset; ESP-IDF's RTC slow clock), runs
 * from the LP system's power-on reset, wrapping at 2^48, and counts on
 * through every other reset (TRM 19.2): a system reset or HP core reset
 * leaves the timer as it is. Only a chip reset and the block's own
 * LP_CLKRST reset reset it. It does not count while its clock is stopped.
 *
 * Logging. Writing RTC_TIMER_MAIN_TIMER_UPDATE logs the count, as do the
 * events whose logging LP_TIMER_UPDATE_REG enables: an HP system reset
 * (MAIN_TIMER_SYS_RST: an HP core reset, or a system reset), the stall of
 * the core LP_SYSTEM_SYSTIMER_STALL_SEL selects starting or ending
 * (MAIN_TIMER_SYS_STALL), and the crystal powering down or up
 * (MAIN_TIMER_XTAL_OFF). Each log moves BUF0 to BUF1 and puts the count
 * in BUF0, so the last two remain. ESP-IDF reads the time this way.
 *
 * Targets. Writing MAIN_TIMER_TAR_EN0 or EN1 (write-only) enables target
 * 0 or 1, which the counter reaches when it equals TAR_HIGHn:TAR_LOWn: at
 * once if it does already, and one already passed only after the counter
 * wraps, the comparison being an equality. A target fires once and is
 * then disabled; writing EN as 0 changes nothing. Target 0 sets
 * SOC_WAKEUP_INT_RAW (MAIN_TIMER_INT in the TRM), which is the PMU's
 * RTC_TIMER_TAR0 wakeup source while it holds, and pulses the ETM event
 * RTC_EVT_CMP; target 1 sets MAIN_TIMER_LP_INT_RAW, the PMU's
 * RTC_TIMER_TAR1 wakeup source while it holds. The overflow sets
 * OVERFLOW_RAW and MAIN_TIMER_OVERFLOW_LP_INT_RAW and pulses RTC_EVT_OVF.
 * The INT_* registers drive LP_TIMER_REG_0_INTR and the LP_INT_* ones
 * LP_TIMER_REG_1_INTR, which is also the LP core's interrupt; their raw
 * bits clear when written with 1 (as the header has them) or through the
 * clear registers. RTC_EVT_TICK, an event on every count, is the
 * "etm-tick" clock output.
 *
 * Not modelled: MAIN_TIMER_ALARM_LOAD (LP_TIMER_MAIN_OVERFLOW_REG), which
 * neither the TRM nor ESP-IDF describes, is logged as unimplemented. The
 * ETM tasks ESP-IDF's soc_etm_source.h lists for the RTC (RTC_TASK_START,
 * ...) are not taken: the TRM says the RTC timer takes none.
 * LP_TIMER_DATE's CLK_EN has no effect software can see. Byte and halfword
 * accesses read and write their bytes of a register.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/timer/esp32p4_lp_timer.h"
#include "migration/vmstate.h"

#define A_TAR_LOW(n)        (0x00 + 8 * (n))
#define A_TAR_HIGH(n)       (0x04 + 8 * (n))
#define A_UPDATE            0x10
#define A_BUF0_LOW          0x14
#define A_BUF0_HIGH         0x18
#define A_BUF1_LOW          0x1c
#define A_BUF1_HIGH         0x20
#define A_MAIN_OVERFLOW     0x24
#define A_INT_RAW           0x28
#define A_INT_ST            0x2c
#define A_INT_ENA           0x30
#define A_INT_CLR           0x34
#define A_LP_INT_RAW        0x38
#define A_LP_INT_ST         0x3c
#define A_LP_INT_ENA        0x40
#define A_LP_INT_CLR        0x44
#define A_DATE              0x3fc

#define TAR_HIGH_MASK       0x0000ffff
#define TAR_EN              BIT(31)
#define UPDATE_UPDATE       BIT(28)
#define UPDATE_XTAL_OFF     BIT(29)
#define UPDATE_SYS_STALL    BIT(30)
#define UPDATE_SYS_RST      BIT(31)
#define UPDATE_RW           0xe0000000
#define ALARM_LOAD          BIT(31)
#define INT_OVERFLOW        BIT(30)
#define INT_TARGET          BIT(31)
#define INT_MASK            (INT_OVERFLOW | INT_TARGET)
#define DATE_RESET          0x02111150

#define COUNT_BITS          48
#define COUNT_MASK          MAKE_64BIT_MASK(0, COUNT_BITS)
#define ALARM_OVF           ESP32P4_LP_TIMER_TARGETS

/* The least time in which @clk makes @n cycles, capped for timer_mod(). */
static int64_t lp_timer_cycles_ns(Clock *clk, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(clk, n);

    while (ns < INT64_MAX / 4 && clock_ns_to_ticks(clk, ns) < n) {
        ns++;
    }
    return MIN(ns, INT64_MAX / 4);
}

/* The count at @now: with the clock stopped, as last synced. */
static uint64_t lp_timer_count(ESP32P4LPTimerState *s, int64_t now)
{
    if (!clock_is_enabled(s->slow) || now <= s->sync_ns) {
        return s->count;
    }
    return (s->count + clock_ns_to_ticks(s->slow, now - s->sync_ns)) &
           COUNT_MASK;
}

/* Fold the counts up to @now, keeping the sync point on a clock edge. */
static void lp_timer_sync(ESP32P4LPTimerState *s, int64_t now)
{
    uint64_t cycles;

    if (!clock_is_enabled(s->slow)) {
        s->sync_ns = now;
        return;
    }
    if (now <= s->sync_ns) {
        return;
    }
    cycles = clock_ns_to_ticks(s->slow, now - s->sync_ns);
    s->count = (s->count + cycles) & COUNT_MASK;
    s->sync_ns += lp_timer_cycles_ns(s->slow, cycles);
}

/*
 * The interrupts, and the PMU's wakeups: the targets' raw bits, which hold
 * until software clears them (ESP-IDF's rtc_timer_ll_set_wakeup_time()
 * clears the bit before it arms the target, so that no earlier alarm
 * wakes the chip).
 */
static void lp_timer_update_irqs(ESP32P4LPTimerState *s)
{
    bool lp = (s->lp_int_raw & s->lp_int_ena) != 0;

    qemu_set_irq(s->irq[0], (s->int_raw & s->int_ena) != 0);
    qemu_set_irq(s->irq[1], lp);
    qemu_set_irq(s->lp_core_int, lp);
    qemu_set_irq(s->soc_wakeup, (s->int_raw & INT_TARGET) != 0);
    qemu_set_irq(s->lp_wakeup, (s->lp_int_raw & INT_TARGET) != 0);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_timer_log(ESP32P4LPTimerState *s, int64_t now)
{
    s->buf[1] = s->buf[0];
    s->buf[0] = lp_timer_count(s, now);
}

/* Schedule alarm @n, after a sync: a target, or the overflow. */
static void lp_timer_schedule(ESP32P4LPTimerState *s, int n)
{
    ESP32P4LPTimerAlarm *a = &s->alarm[n];
    uint64_t ahead;

    timer_del(&a->timer);
    if (!clock_is_enabled(s->slow) || (n != ALARM_OVF && !s->armed[n])) {
        return;
    }
    if (n == ALARM_OVF) {
        ahead = (1ull << COUNT_BITS) - s->count;
    } else {
        ahead = (s->target[n] - s->count) & COUNT_MASK;
    }
    a->due_ns = s->sync_ns + lp_timer_cycles_ns(s->slow, ahead);
    timer_mod(&a->timer, a->due_ns);
}

static void lp_timer_schedule_all(ESP32P4LPTimerState *s)
{
    int n;

    for (n = 0; n <= ALARM_OVF; n++) {
        lp_timer_schedule(s, n);
    }
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_timer_fire(ESP32P4LPTimerState *s, int n)
{
    if (n == ALARM_OVF) {
        s->int_raw |= INT_OVERFLOW;
        s->lp_int_raw |= INT_OVERFLOW;
        lp_timer_update_irqs(s);
        qemu_irq_pulse(s->etm_event[ESP32P4_LP_TIMER_EVT_OVF]);
        return;
    }
    s->armed[n] = false;
    if (n == 0) {
        s->int_raw |= INT_TARGET;
        lp_timer_update_irqs(s);
        qemu_irq_pulse(s->etm_event[ESP32P4_LP_TIMER_EVT_CMP]);
    } else {
        s->lp_int_raw |= INT_TARGET;
        lp_timer_update_irqs(s);
    }
}

/* Alarm @n falls due at its due time, which a late QEMU timer has passed. */
static void lp_timer_due(ESP32P4LPTimerState *s, int n)
{
    ESP32P4LPTimerAlarm *a = &s->alarm[n];

    timer_del(&a->timer);
    lp_timer_sync(s, a->due_ns);
    lp_timer_fire(s, n);
    lp_timer_schedule(s, n);
}

static void lp_timer_alarm_cb(void *opaque)
{
    ESP32P4LPTimerAlarm *a = opaque;
    ESP32P4LPTimerState *s = a->s;

    lp_timer_due(s, a - s->alarm);
}

/* Run the alarms that fell due by @now, in order. */
static void lp_timer_catch_up(ESP32P4LPTimerState *s, int64_t now)
{
    for (;;) {
        int next = -1;
        int n;

        for (n = 0; n <= ALARM_OVF; n++) {
            ESP32P4LPTimerAlarm *a = &s->alarm[n];

            if (timer_pending(&a->timer) && a->due_ns <= now &&
                (next < 0 || a->due_ns < s->alarm[next].due_ns)) {
                next = n;
            }
        }
        if (next < 0) {
            return;
        }
        lp_timer_due(s, next);
    }
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_timer_slow_update(void *opaque, ClockEvent event)
{
    ESP32P4LPTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        lp_timer_catch_up(s, now);
        lp_timer_sync(s, now);
        return;
    }
    s->sync_ns = now;
    lp_timer_schedule_all(s);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_timer_set_hp_sys_reset(void *opaque, int n, int level)
{
    ESP32P4LPTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    lp_timer_catch_up(s, now);
    if (level && !s->hp_sys_reset && (s->update & UPDATE_SYS_RST)) {
        lp_timer_log(s, now);
    }
    s->hp_sys_reset = level;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_timer_set_stall(void *opaque, int core, int level)
{
    ESP32P4LPTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    lp_timer_catch_up(s, now);
    if (s->stall[core] != !!level && core == s->stall_sel &&
        (s->update & UPDATE_SYS_STALL)) {
        lp_timer_log(s, now);
    }
    s->stall[core] = level;
}

static void lp_timer_set_stall_sel(void *opaque, int n, int level)
{
    ESP32P4LPTimerState *s = opaque;

    s->stall_sel = level;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_timer_set_xtal_off(void *opaque, int n, int level)
{
    ESP32P4LPTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    lp_timer_catch_up(s, now);
    if (s->xtal_off != !!level && (s->update & UPDATE_XTAL_OFF)) {
        lp_timer_log(s, now);
    }
    s->xtal_off = level;
}

/* The register at word offset @addr; false for a reserved offset. */
static bool lp_timer_read_reg(ESP32P4LPTimerState *s, hwaddr addr,
                              uint32_t *v)
{
    switch (addr) {
    case A_TAR_LOW(0):
    case A_TAR_LOW(1):
        *v = s->target[(addr - A_TAR_LOW(0)) / 8];
        return true;
    case A_TAR_HIGH(0):
    case A_TAR_HIGH(1):
        *v = s->target[(addr - A_TAR_HIGH(0)) / 8] >> 32;
        return true;
    case A_UPDATE:
        *v = s->update;
        return true;
    case A_BUF0_LOW:
    case A_BUF1_LOW:
        *v = s->buf[(addr - A_BUF0_LOW) / 8];
        return true;
    case A_BUF0_HIGH:
    case A_BUF1_HIGH:
        *v = s->buf[(addr - A_BUF0_HIGH) / 8] >> 32;
        return true;
    case A_MAIN_OVERFLOW:
    case A_INT_CLR:
    case A_LP_INT_CLR:
        *v = 0;
        return true;
    case A_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_LP_INT_RAW:
        *v = s->lp_int_raw;
        return true;
    case A_LP_INT_ST:
        *v = s->lp_int_raw & s->lp_int_ena;
        return true;
    case A_LP_INT_ENA:
        *v = s->lp_int_ena;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    default:
        *v = 0;
        return false;
    }
}

static uint64_t lp_timer_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LPTimerState *s = opaque;
    uint32_t v;

    lp_timer_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (!lp_timer_read_reg(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_TIMER, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Write the register at word offset @addr: @value is its new value, the
 * written byte lanes merged into the old, for its R/W fields; @ones has
 * the bits written as 1, for its write-trigger and write-1-to-clear ones.
 */
static void lp_timer_write_reg(ESP32P4LPTimerState *s, hwaddr addr,
                               uint32_t value, uint32_t ones, int64_t now)
{
    int n;

    switch (addr) {
    case A_TAR_LOW(0):
    case A_TAR_LOW(1):
        n = (addr - A_TAR_LOW(0)) / 8;
        s->target[n] = deposit64(s->target[n], 0, 32, value);
        lp_timer_sync(s, now);
        lp_timer_schedule(s, n);
        break;
    case A_TAR_HIGH(0):
    case A_TAR_HIGH(1):
        n = (addr - A_TAR_HIGH(0)) / 8;
        s->target[n] = deposit64(s->target[n], 32, 32,
                                 value & TAR_HIGH_MASK);
        lp_timer_sync(s, now);
        if (ones & TAR_EN) {
            s->armed[n] = true;
            if (s->count == s->target[n] && clock_is_enabled(s->slow)) {
                lp_timer_fire(s, n);
            }
        }
        lp_timer_schedule(s, n);
        break;
    case A_UPDATE:
        s->update = value & UPDATE_RW;
        if (ones & UPDATE_UPDATE) {
            lp_timer_log(s, now);
        }
        break;
    case A_MAIN_OVERFLOW:
        if (ones & ALARM_LOAD) {
            qemu_log_mask(LOG_UNIMP, "%s: MAIN_TIMER_ALARM_LOAD, which is "
                          "not documented, is not modelled\n",
                          TYPE_ESP32P4_LP_TIMER);
        }
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(ones & INT_MASK);
        lp_timer_update_irqs(s);
        break;
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        lp_timer_update_irqs(s);
        break;
    case A_LP_INT_RAW:
    case A_LP_INT_CLR:
        s->lp_int_raw &= ~(ones & INT_MASK);
        lp_timer_update_irqs(s);
        break;
    case A_LP_INT_ENA:
        s->lp_int_ena = value & INT_MASK;
        lp_timer_update_irqs(s);
        break;
    case A_DATE:
        s->date = value;
        break;
    case A_BUF0_LOW ... A_BUF1_HIGH:
    case A_INT_ST:
    case A_LP_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_TIMER, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_TIMER, addr);
        break;
    }
}

static void lp_timer_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned size)
{
    ESP32P4LPTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t ones = (data << shift) & lanes;
    uint32_t old;

    lp_timer_catch_up(s, now);
    lp_timer_read_reg(s, addr & ~3, &old);
    lp_timer_write_reg(s, addr & ~3, (old & ~lanes) | ones, ones, now);
}

static const MemoryRegionOps lp_timer_ops = {
    .read = lp_timer_read,
    .write = lp_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * A system reset leaves the timer counting as it was, logging the time if
 * MAIN_TIMER_SYS_RST is set: the reset and clock control still holds the
 * system reset's cause here, before its own reset records it. A chip
 * reset, or the block's own, resets it.
 */
static void esp32p4_lp_timer_reset_enter(Object *obj, ResetType type)
{
    ESP32P4LPTimerState *s = ESP32P4_LP_TIMER(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->keep = esp32p4_clkrst_system_reset_cause(s->clkrst) != 0;
    if (s->keep) {
        lp_timer_catch_up(s, now);
        if (s->update & UPDATE_SYS_RST) {
            lp_timer_log(s, now);
        }
    }
}

static void esp32p4_lp_timer_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPTimerState *s = ESP32P4_LP_TIMER(obj);
    int n;

    if (s->keep) {
        return;
    }
    for (n = 0; n < ESP32P4_LP_TIMER_TARGETS; n++) {
        s->target[n] = 0;
        s->armed[n] = false;
    }
    for (n = 0; n <= ALARM_OVF; n++) {
        timer_del(&s->alarm[n].timer);
    }
    s->update = 0;
    s->buf[0] = 0;
    s->buf[1] = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->lp_int_raw = 0;
    s->lp_int_ena = 0;
    s->date = DATE_RESET;
    s->count = 0;
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void esp32p4_lp_timer_reset_exit(Object *obj, ResetType type)
{
    ESP32P4LPTimerState *s = ESP32P4_LP_TIMER(obj);

    if (!s->keep) {
        lp_timer_schedule_all(s);
    }
    s->keep = false;
    lp_timer_update_irqs(s);
}

static void esp32p4_lp_timer_init(Object *obj)
{
    ESP32P4LPTimerState *s = ESP32P4_LP_TIMER(obj);
    DeviceState *dev = DEVICE(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &lp_timer_ops, s,
                          "esp32p4.lp-timer", ESP32P4_LP_TIMER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (n = 0; n < 2; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[n]);
    }
    s->slow = qdev_init_clock_in(dev, "slow", lp_timer_slow_update, s,
                                 ClockPreUpdate | ClockUpdate);
    s->etm_tick = qdev_init_clock_out(dev, "etm-tick");
    clock_set_source(s->etm_tick, s->slow);
    for (n = 0; n <= ALARM_OVF; n++) {
        s->alarm[n].s = s;
        timer_init_ns(&s->alarm[n].timer, QEMU_CLOCK_VIRTUAL,
                      lp_timer_alarm_cb, &s->alarm[n]);
    }
    qdev_init_gpio_out_named(dev, &s->lp_core_int,
                             ESP32P4_LP_TIMER_LP_CORE_INT, 1);
    qdev_init_gpio_out_named(dev, &s->soc_wakeup, ESP32P4_LP_TIMER_SOC_WAKEUP,
                             1);
    qdev_init_gpio_out_named(dev, &s->lp_wakeup, ESP32P4_LP_TIMER_LP_WAKEUP,
                             1);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_LP_TIMER_ETM_EVENT,
                             ESP32P4_LP_TIMER_EVTS);
    qdev_init_gpio_in_named(dev, lp_timer_set_hp_sys_reset,
                            ESP32P4_LP_TIMER_HP_SYS_RESET, 1);
    qdev_init_gpio_in_named(dev, lp_timer_set_stall,
                            ESP32P4_LP_TIMER_CORE_STALL,
                            ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_in_named(dev, lp_timer_set_stall_sel,
                            ESP32P4_LP_TIMER_STALL_SEL, 1);
    qdev_init_gpio_in_named(dev, lp_timer_set_xtal_off,
                            ESP32P4_LP_TIMER_XTAL_OFF, 1);
}

static void esp32p4_lp_timer_realize(DeviceState *dev, Error **errp)
{
    if (!ESP32P4_LP_TIMER(dev)->clkrst) {
        error_setg(errp, "%s: the \"clkrst\" link is not set",
                   TYPE_ESP32P4_LP_TIMER);
    }
}

static const VMStateDescription vmstate_esp32p4_lp_timer_alarm = {
    .name = TYPE_ESP32P4_LP_TIMER "/alarm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4LPTimerAlarm),
        VMSTATE_INT64(due_ns, ESP32P4LPTimerAlarm),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_lp_timer = {
    .name = TYPE_ESP32P4_LP_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(slow, ESP32P4LPTimerState),
        VMSTATE_UINT64_ARRAY(target, ESP32P4LPTimerState,
                             ESP32P4_LP_TIMER_TARGETS),
        VMSTATE_BOOL_ARRAY(armed, ESP32P4LPTimerState,
                           ESP32P4_LP_TIMER_TARGETS),
        VMSTATE_UINT32(update, ESP32P4LPTimerState),
        VMSTATE_UINT64_ARRAY(buf, ESP32P4LPTimerState, 2),
        VMSTATE_UINT32(int_raw, ESP32P4LPTimerState),
        VMSTATE_UINT32(int_ena, ESP32P4LPTimerState),
        VMSTATE_UINT32(lp_int_raw, ESP32P4LPTimerState),
        VMSTATE_UINT32(lp_int_ena, ESP32P4LPTimerState),
        VMSTATE_UINT32(date, ESP32P4LPTimerState),
        VMSTATE_UINT64(count, ESP32P4LPTimerState),
        VMSTATE_INT64(sync_ns, ESP32P4LPTimerState),
        VMSTATE_STRUCT_ARRAY(alarm, ESP32P4LPTimerState,
                             ESP32P4_LP_TIMER_TARGETS + 1, 1,
                             vmstate_esp32p4_lp_timer_alarm,
                             ESP32P4LPTimerAlarm),
        VMSTATE_BOOL(hp_sys_reset, ESP32P4LPTimerState),
        VMSTATE_BOOL_ARRAY(stall, ESP32P4LPTimerState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL(stall_sel, ESP32P4LPTimerState),
        VMSTATE_BOOL(xtal_off, ESP32P4LPTimerState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_lp_timer_properties[] = {
    DEFINE_PROP_LINK("clkrst", ESP32P4LPTimerState, clkrst,
                     TYPE_ESP32P4_CLKRST, ESP32P4ClkRstState *),
};

static void esp32p4_lp_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_lp_timer_realize;
    dc->vmsd = &vmstate_esp32p4_lp_timer;
    device_class_set_props(dc, esp32p4_lp_timer_properties);
    rc->phases.enter = esp32p4_lp_timer_reset_enter;
    rc->phases.hold = esp32p4_lp_timer_reset_hold;
    rc->phases.exit = esp32p4_lp_timer_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static const TypeInfo esp32p4_lp_timer_info = {
    .name          = TYPE_ESP32P4_LP_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4LPTimerState),
    .instance_init = esp32p4_lp_timer_init,
    .class_init    = esp32p4_lp_timer_class_init,
};

static void esp32p4_lp_timer_register_types(void)
{
    type_register_static(&esp32p4_lp_timer_info);
}
type_init(esp32p4_lp_timer_register_types)
