/*
 * ESP32 "Timer Group" peripheral
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/registerfields.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-clock.h"
#include "hw/timer/esp32_timg.h"


#define TIMG_REGFILE_SIZE 0x100

static uint64_t esp32_timg_timer_get_count(Esp32TimgTimerState *s, uint64_t ns_now);
static uint64_t esp32_timg_timer_count_to_ns(Esp32TimgTimerState *s, uint64_t count);
static int esp32_timg_timer_direction(Esp32TimgTimerState *s);
static uint64_t esp32_timg_apb_ns(Esp32TimgState *s, uint64_t ticks,
                                  uint32_t div);
static void esp32_timg_timer_update_config(Esp32TimgTimerState *ts);
static void esp32_timg_timer_update_alarm(Esp32TimgTimerState *ts, uint64_t ns_now);
static void esp32_timg_timer_reload(Esp32TimgTimerState *ts, uint64_t ns_now);
static void esp32_timg_cal_write(Esp32TimgState *s, uint32_t value,
                                 int64_t now);
static void esp32_timg_cal_advance(Esp32TimgState *s, int64_t now);
static void esp32_timg_cal_begin(Esp32TimgState *s, bool oneshot,
                                 int64_t now);
static void esp32_timg_cal_pause(Esp32TimgState *s, int64_t now);
static void esp32_timg_cal_rebase(Esp32TimgState *s, int64_t now);
static void esp32_timg_int_update(Esp32TimgState *s);
static void esp32_timg_timer_cb(void *opaque);
static void esp32_timg_wdt_cb(void *opaque);
static bool esp32_timg_wdt_protected(Esp32TimgWdtState *ws);
static void esp32_timg_wdt_update_config(Esp32TimgWdtState *ws);
static void esp32_timg_wdt_feed(Esp32TimgWdtState *ws);
static void esp32_timg_wdt_arm(Esp32TimgWdtState *ws, uint64_t ns_now);
static uint64_t esp32_timg_wdt_get_count(Esp32TimgWdtState *ws,
                                         uint64_t ns_now);


#define TIMG_DEBUG_LOG(...) // qemu_log(__VA_ARGS__)

/*
 * A watchdog stage callback that runs this far after its deadline, as the
 * host can make it without icount, starts the next stage from the present
 * rather than replaying every stage it missed.
 */
#define TIMG_CATCHUP_NS (10 * SCALE_MS)

static inline void set_low_word(uint64_t *dst, uint32_t word)
{
    *dst = (*dst & ~UINT32_MAX) | word;
}

static inline void set_high_word(uint64_t *dst, uint32_t word)
{
    *dst = (*dst & UINT32_MAX) | (((uint64_t)word) << 32);
}

static inline qemu_irq get_level_irq(Esp32TimgState* s, Esp32TimgInterruptType it)
{
    return s->irqs[it];
}

static inline qemu_irq get_edge_irq(Esp32TimgState* s, Esp32TimgInterruptType it)
{
    return s->irqs[TIMG_INT_MAX + it];
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * A register access first makes the timers' alarms, and the watchdog's
 * stages ending in an interrupt, due by its moment take effect, each at
 * its own time, so that what software reads and writes (INT_RAW, a
 * reloaded count, an INT_CLR) does not depend on how promptly the host
 * runs their QEMU timers. A stage that resets is left to its timer: a
 * reset is not made from within an access.
 */
static void esp32_timg_catch_up(Esp32TimgState *s)
{
    Esp32TimgTimerState *timers[] = { &s->t0, &s->t1, &s->lact };
    Esp32TimgWdtState *ws = &s->wdt;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    while (timer_pending(&ws->stage_timer) &&
           timer_expire_time_ns(&ws->stage_timer) <= now &&
           (ws->mode[ws->cur_stage] == WDT_MODE_INT ||
            ws->mode[ws->cur_stage] == WDT_MODE_OFF)) {
        timer_del(&ws->stage_timer);
        esp32_timg_wdt_cb(ws);
    }

    for (int i = 0; i < ARRAY_SIZE(timers); i++) {
        Esp32TimgTimerState *ts = timers[i];

        while (timer_pending(&ts->alarm_timer) &&
               timer_expire_time_ns(&ts->alarm_timer) <= now) {
            timer_del(&ts->alarm_timer);
            esp32_timg_timer_cb(ts);
        }
    }
}

static uint64_t esp32_timg_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32TimgState *s = ESP32_TIMG(opaque);
    Esp32TimgTimerState *ts = NULL;

    esp32_timg_catch_up(s);
    if (addr <= A_TIMG_T0LOAD) {
        ts = &s->t0;
    } else if (addr <= A_TIMG_T1LOAD) {
        ts = &s->t1;
    } else if (addr >= A_TIMG_LACTCONFIG && addr < A_TIMG_LACTLOAD) {
        ts = &s->lact;
    }
    uint64_t r = 0;
    switch (addr) {
    case A_TIMG_T0CONFIG:
    case A_TIMG_T1CONFIG:
    case A_TIMG_LACTCONFIG:
        r = ts->config_reg;
        break;
    case A_TIMG_T0LO:
    case A_TIMG_T1LO:
    case A_TIMG_LACTLO:
        r = ts->last_val & UINT32_MAX;
        break;
    case A_TIMG_T0HI:
    case A_TIMG_T1HI:
    case A_TIMG_LACTHI:
        r = ts->last_val >> 32;
        break;
    case A_TIMG_T0LOADLO:
    case A_TIMG_T1LOADLO:
    case A_TIMG_LACTLOADLO:
        r = ts->load_val & UINT32_MAX;
        break;
    case A_TIMG_T0LOADHI:
    case A_TIMG_T1LOADHI:
    case A_TIMG_LACTLOADHI:
        r = ts->load_val >> 32;
        break;
    case A_TIMG_T0ALARMLO:
    case A_TIMG_T1ALARMLO:
    case A_TIMG_LACTALARMLO:
        r = ts->alarm_val & UINT32_MAX;
        break;
    case A_TIMG_T0ALARMHI:
    case A_TIMG_T1ALARMHI:
    case A_TIMG_LACTALARMHI:
        r = ts->alarm_val >> 32;
        break;

    case A_TIMG_WDTCONFIG0:
        r = s->wdt.config0_reg;
        break;
    case A_TIMG_WDTCONFIG1:
        r = FIELD_DP32(r, TIMG_WDTCONFIG1, PRESCALE, s->wdt.prescale);
        break;
    case A_TIMG_WDTCONFIG2:
    case A_TIMG_WDTCONFIG3:
    case A_TIMG_WDTCONFIG4:
    case A_TIMG_WDTCONFIG5: {
        int stage = (addr - A_TIMG_WDTCONFIG2) / 4;
        r = s->wdt.timeout[stage];
        break;
    }
    case A_TIMG_WDTPROTECT:
        r = s->wdt.protect_reg;
        break;

    case A_TIMG_RTCCALICFG:
        esp32_timg_cal_advance(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        r = FIELD_DP32(r, TIMG_RTCCALICFG, START, s->rtc_cal_start);
        r = FIELD_DP32(r, TIMG_RTCCALICFG, MAX, s->rtc_cal_max);
        r = FIELD_DP32(r, TIMG_RTCCALICFG, RDY, s->rtc_cal_ready);
        r = FIELD_DP32(r, TIMG_RTCCALICFG, CLK_SEL, s->rtc_cal_clk_sel);
        r = FIELD_DP32(r, TIMG_RTCCALICFG, START_CYCLING,
                       s->rtc_cal_start_cycling);
        break;
    case A_TIMG_RTCCALICFG1:
        esp32_timg_cal_advance(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        r = FIELD_DP32(0, TIMG_RTCCALICFG1, VALUE, s->rtc_cal_value);
        break;

    case A_TIMG_INT_ENA:
        r = s->int_ena;
        break;
    case A_TIMG_INT_RAW:
        r = s->int_raw;
        break;
    case A_TIMG_INT_ST:
        r = s->int_ena & s->int_raw;
        break;
    }
    return r;
}

static void esp32_timg_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32TimgState *s = ESP32_TIMG(opaque);
    Esp32TimgTimerState *ts = NULL;

    esp32_timg_catch_up(s);
    if (addr <= A_TIMG_T0LOAD) {
        ts = &s->t0;
    } else if (addr <= A_TIMG_T1LOAD) {
        ts = &s->t1;
    } else if (addr >= A_TIMG_LACTCONFIG && addr <= A_TIMG_LACTLOAD) {
        ts = &s->lact;
    }

    switch (addr) {
    case A_TIMG_T0CONFIG:
    case A_TIMG_T1CONFIG:
    case A_TIMG_LACTCONFIG:
        ts->config_reg = value;
        esp32_timg_timer_update_config(ts);
        break;
    case A_TIMG_T0UPDATE:
    case A_TIMG_T1UPDATE:
    case A_TIMG_LACTUPDATE: {
        uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        ts->last_val = esp32_timg_timer_get_count(ts, ns_now);
        break;
    }
    case A_TIMG_T0LOADLO:
    case A_TIMG_T1LOADLO:
    case A_TIMG_LACTLOADLO:
        set_low_word(&ts->load_val, value);
        break;
    case A_TIMG_T0LOADHI:
    case A_TIMG_T1LOADHI:
    case A_TIMG_LACTLOADHI:
        set_high_word(&ts->load_val, value);
        break;
    case A_TIMG_T0LOAD:
    case A_TIMG_T1LOAD:
    case A_TIMG_LACTLOAD: {
        uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        esp32_timg_timer_reload(ts, ns_now);
        break;
    }
    case A_TIMG_T0ALARMLO:
    case A_TIMG_T1ALARMLO:
    case A_TIMG_LACTALARMLO:
        set_low_word(&ts->alarm_val, value);
        break;
    case A_TIMG_T0ALARMHI:
    case A_TIMG_T1ALARMHI:
    case A_TIMG_LACTALARMHI:
        set_high_word(&ts->alarm_val, value);
        break;

    case A_TIMG_WDTCONFIG0:
        if (!esp32_timg_wdt_protected(&s->wdt)) {
            s->wdt.config0_reg = value;
            esp32_timg_wdt_update_config(&s->wdt);
        } else {
            TIMG_DEBUG_LOG("failed to write TIMG_WDTCONFIG0, write protected (0x%08x)\n", s->wdt.protect_reg);
        }
        break;
    case A_TIMG_WDTCONFIG1:
        if (!esp32_timg_wdt_protected(&s->wdt)) {
            s->wdt.config1_reg = value;
            esp32_timg_wdt_update_config(&s->wdt);
        }
        break;
    case A_TIMG_WDTCONFIG2:
    case A_TIMG_WDTCONFIG3:
    case A_TIMG_WDTCONFIG4:
    case A_TIMG_WDTCONFIG5: {
        if (!esp32_timg_wdt_protected(&s->wdt)) {
            int stage = (addr - A_TIMG_WDTCONFIG2) / 4;
            s->wdt.timeout[stage] = value;
        }
        break;
    }
    case A_TIMG_WDTFEED:
        if (!esp32_timg_wdt_protected(&s->wdt)) {
            esp32_timg_wdt_feed(&s->wdt);
        }
        break;
    case A_TIMG_WDTPROTECT:
        s->wdt.protect_reg = value;
        break;


    case A_TIMG_RTCCALICFG: {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        esp32_timg_cal_advance(s, now);
        esp32_timg_cal_write(s, value, now);
        break;
    }

    case A_TIMG_INT_ENA:
        s->int_ena = value;
        esp32_timg_int_update(s);
        break;
    case A_TIMG_INT_CLR:
        s->int_raw &= ~value;
        esp32_timg_int_update(s);
        break;
    }
}

static const MemoryRegionOps esp32_timg_ops = {
    .read =  esp32_timg_read,
    .write = esp32_timg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_timg_timer_reset(Esp32TimgTimerState* ts)
{
    timer_del(&ts->alarm_timer);
    ts->config_reg = R_TIMG_T0CONFIG_INCREASE_MASK
        | R_TIMG_T0CONFIG_AUTORELOAD_MASK
        | (1 << R_TIMG_T0CONFIG_DIVIDER_SHIFT);
    ts->alarm_val = 0;
    ts->load_val = 0;
    ts->count_base = 0;
    ts->ns_base = 0;
    esp32_timg_timer_update_config(ts);
}

static void esp32_timg_wdt_reset(Esp32TimgWdtState* ws)
{
    timer_del(&ws->stage_timer);

    ws->config0_reg = 0x0004c000;
    ws->config1_reg = 0x00010000;
    ws->timeout[0] = 0x018cba80;
    ws->timeout[1] = 0x07ffffff;
    ws->timeout[2] = 0x000fffff;
    ws->timeout[3] = 0x000fffff;
    ws->protect_reg = ESP32_TIMG_WDT_PROTECT_WORD;

    if (ws->parent->wdt_en_at_reset) {
        /* On reset, stage0 is configured as system reset, however this is done by
         * hardware state, not in config0 register. Emulate this by temporarily setting STG0 field here.
         */
        ws->config0_reg |= (WDT_MODE_SYSRESET << R_TIMG_WDTCONFIG0_STG0_SHIFT);
    }
    esp32_timg_wdt_update_config(ws);
    if (ws->parent->wdt_en_at_reset) {
        ws->config0_reg &= (~R_TIMG_WDTCONFIG0_STG0_MASK);
    }
}

static void esp32_timg_reset_hold(Object *obj, ResetType type)
{
    Esp32TimgState *s = ESP32_TIMG(obj);

    /* RTCCALICFG resets with cycling mode on: one 8MD256 cycle at a time */
    s->rtc_cal_start = false;
    s->rtc_cal_start_cycling = true;
    s->rtc_cal_max = 1;
    s->rtc_cal_clk_sel = ESP32_TIMG_CAL_8MD256;
    s->rtc_cal_ready = false;
    s->rtc_cal_value = 0;
    esp32_timg_cal_begin(s, false, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

    esp32_timg_timer_reset(&s->t0);
    esp32_timg_timer_reset(&s->t1);
    esp32_timg_timer_reset(&s->lact);
    esp32_timg_wdt_reset(&s->wdt);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * The counters and the watchdog count APB_CLK cycles. The SoC stops the
 * clock (0 Hz) while DPORT gates the group's clock or holds it in reset;
 * the counts then freeze. Before a change of rate the counts are folded
 * into their bases at the old rate; afterwards the alarms and the watchdog
 * stage are re-armed at the new one.
 */
static void esp32_timg_apb_clk_update(void *opaque, ClockEvent event)
{
    Esp32TimgState *s = ESP32_TIMG(opaque);
    Esp32TimgTimerState *timers[] = { &s->t0, &s->t1, &s->lact };
    uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (int i = 0; i < ARRAY_SIZE(timers); i++) {
        Esp32TimgTimerState *ts = timers[i];

        if (event == ClockPreUpdate) {
            ts->count_base = esp32_timg_timer_get_count(ts, ns_now);
            ts->ns_base = ns_now;
        } else {
            esp32_timg_timer_update_alarm(ts, ns_now);
        }
    }
    if (event == ClockPreUpdate) {
        s->wdt.count_base = esp32_timg_wdt_get_count(&s->wdt, ns_now);
        s->wdt.ns_base = ns_now;
        esp32_timg_cal_advance(s, ns_now);
        s->cal_apb_was_running = clock_is_enabled(s->apb_clk);
    } else {
        esp32_timg_wdt_arm(&s->wdt, ns_now);
        if (s->cal_apb_was_running && !clock_is_enabled(s->apb_clk)) {
            esp32_timg_cal_pause(s, ns_now);
        } else if (!s->cal_apb_was_running &&
                   clock_is_enabled(s->apb_clk)) {
            esp32_timg_cal_rebase(s, ns_now);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * [spec:nuos:req:emu.esp32.rtc]
 * RTC slow clock calibration (TRM, Timer Group, TIMGn_RTCCALICFG_REG and
 * TIMGn_RTCCALICFG1_REG; ESP-IDF's rtc_clk_cal_internal and
 * rtc_clk_wait_for_slow_cycle). The group counts XTAL_CLK cycles over
 * RTC_CALI_MAX cycles of the clock RTC_CALI_CLK_SEL picks: 0, RTC_SLOW_CLK
 * as RTC_CNTL selects it; 1, RC_FAST_DIV_CLK (8MD256); 2, XTAL32K_CLK.
 * Counting starts at the measured clock's first edge after the
 * measurement begins and ends at its RTC_CALI_MAX'th edge after that (at
 * that first edge for 0, as rtc_clk_wait_for_slow_cycle relies on);
 * RTC_CALI_RDY rises then, with the count of XTAL_CLK edges between in
 * RTC_CALI_VALUE (25 bits, wrapping). Each clock's edges are whole
 * periods apart from time 0, or from its last change of rate, each at its
 * exact time. A clock that does not run, such as XTAL32K_CLK with no
 * crystal fitted, never completes a measurement: RDY stays low and
 * software's wait times out, as ESP-IDF's does. XTAL_CLK's edges count
 * while the measured clock is stopped, and a measured clock that changes
 * rate goes on at its new edges. Nothing counts while the group's clock
 * (APB_CLK) or XTAL_CLK is stopped: a stop of the group's clock keeps the
 * whole cycles of the measured clock counted by then, with XTAL_CLK's
 * edges up to the last of them, and counting goes on from the measured
 * clock's first edge after the clock runs again, so that the value stays
 * a count over whole cycles.
 *
 * A rising edge of RTC_CALI_START starts a measurement, dropping RDY; a
 * falling one abandons it. Cycling mode, RTC_CALI_START_CYCLING (set at
 * reset), measures continuously, each measurement beginning where the
 * last ended and each completion updating the value and RDY. The clock
 * and cycle count are latched as a measurement starts.
 *
 * Nothing outside the group sees a measurement end, so the model keeps no
 * QEMU timer for it: a register access, or a change of a clock it counts,
 * brings it up to that moment from the clocks' edges
 * (esp32_timg_cal_advance()), passing over a cycling run's measurements
 * in one go, so that what it costs the host does not depend on the
 * clocks' rates.
 */

/* @ns as Clock units (2^-32 ns), @hi:@lo. */
static void esp32_timg_cal_units(int64_t ns, uint64_t *hi, uint64_t *lo)
{
    *lo = (uint64_t)ns << 32;
    *hi = (uint64_t)ns >> 32;
}

/*
 * The edges after @origin ns of a clock of @period Clock units with an
 * edge there, by @hi:@lo Clock units: 0 for a stopped clock.
 */
static uint64_t esp32_timg_cal_edges_by(uint64_t period, int64_t origin,
                                        uint64_t hi, uint64_t lo)
{
    uint64_t o_hi, o_lo;

    if (!period) {
        return 0;
    }
    esp32_timg_cal_units(origin, &o_hi, &o_lo);
    if (hi < o_hi || (hi == o_hi && lo <= o_lo)) {
        return 0;
    }
    hi -= o_hi + (lo < o_lo);
    lo -= o_lo;
    if (hi >= period) {
        return UINT64_MAX;
    }
    divu128(&lo, &hi, period);
    return lo;
}

static bool esp32_timg_cal_counts(Esp32TimgState *s)
{
    return clock_is_enabled(s->apb_clk) && clock_is_enabled(s->xtal_clk);
}

/* The measured clock's edges by @now, since its last change. */
static uint64_t esp32_timg_cal_edges(Esp32TimgState *s, int64_t now)
{
    uint64_t hi, lo;

    esp32_timg_cal_units(now, &hi, &lo);
    return esp32_timg_cal_edges_by(clock_get(s->cal_clk[s->cal_sel]),
                                   s->cal_origin[s->cal_sel], hi, lo);
}

static uint64_t esp32_timg_cal_xtal_edges(Esp32TimgState *s, int64_t now)
{
    uint64_t hi, lo;

    esp32_timg_cal_units(now, &hi, &lo);
    return esp32_timg_cal_edges_by(clock_get(s->xtal_clk), s->xtal_origin,
                                   hi, lo);
}

/* The time of the measured clock's edge @n, as Clock units @hi:@lo. */
static void esp32_timg_cal_edge_time(Esp32TimgState *s, uint64_t n,
                                     uint64_t *hi, uint64_t *lo)
{
    uint64_t o_hi, o_lo;

    mulu64(lo, hi, n, clock_get(s->cal_clk[s->cal_sel]));
    esp32_timg_cal_units(s->cal_origin[s->cal_sel], &o_hi, &o_lo);
    *lo += o_lo;
    *hi += o_hi + (*lo < o_lo);
}

/* XTAL_CLK's edges by the measured clock's edge @n, at its exact time. */
static uint64_t esp32_timg_cal_xtal_by(Esp32TimgState *s, uint64_t n)
{
    uint64_t hi, lo;

    esp32_timg_cal_edge_time(s, n, &hi, &lo);
    return esp32_timg_cal_edges_by(clock_get(s->xtal_clk), s->xtal_origin,
                                   hi, lo);
}

/*
 * Begin a measurement at Clock units @hi:@lo, of the clock and the cycles
 * selected now, from that clock's first edge after.
 */
static void esp32_timg_cal_begin_at(Esp32TimgState *s, bool oneshot,
                                    uint64_t hi, uint64_t lo)
{
    s->cal_active = true;
    s->cal_oneshot = oneshot;
    s->cal_counting = false;
    s->cal_sel = s->rtc_cal_clk_sel;
    s->cal_target = s->rtc_cal_max;
    s->cal_cycles = 0;
    s->cal_xtal = 0;
    s->cal_first = esp32_timg_cal_edges_by(clock_get(s->cal_clk[s->cal_sel]),
                                           s->cal_origin[s->cal_sel],
                                           hi, lo) + 1;
    if (oneshot) {
        s->rtc_cal_ready = false;
    }
}

static void esp32_timg_cal_begin(Esp32TimgState *s, bool oneshot,
                                 int64_t now)
{
    uint64_t hi, lo;

    esp32_timg_cal_units(now, &hi, &lo);
    esp32_timg_cal_begin_at(s, oneshot, hi, lo);
}

/*
 * A cycling measurement has just begun: it and the ones after it end
 * every RTC_CALI_MAX + 1 edges of the measured clock, those that have by
 * @now_k passed over at once, the value of the last of them taken from
 * its edges.
 */
static void esp32_timg_cal_skip(Esp32TimgState *s, uint64_t now_k)
{
    uint64_t span = (uint64_t)s->cal_target + 1;
    uint64_t begin = s->cal_first - 1, e, hi, lo;

    if (now_k < begin || now_k - begin < span) {
        return;
    }
    e = begin + (now_k - begin) / span * span;
    s->rtc_cal_value = extract64(esp32_timg_cal_xtal_by(s, e) -
                                 esp32_timg_cal_xtal_by(s, e - s->cal_target),
                                 0, R_TIMG_RTCCALICFG1_VALUE_LENGTH);
    s->rtc_cal_ready = true;
    esp32_timg_cal_edge_time(s, e, &hi, &lo);
    esp32_timg_cal_begin_at(s, false, hi, lo);
}

/*
 * Bring the measurement up to @now: every end by then, each at the
 * measured clock's edge it comes at, with the clocks as they have run
 * since they last changed.
 */
static void esp32_timg_cal_advance(Esp32TimgState *s, int64_t now)
{
    uint64_t now_k, e, at_first, hi, lo;

    while (s->cal_active && esp32_timg_cal_counts(s) &&
           clock_is_enabled(s->cal_clk[s->cal_sel])) {
        now_k = esp32_timg_cal_edges(s, now);
        e = s->cal_first + s->cal_target - s->cal_cycles;
        if (e > now_k) {
            return;
        }
        at_first = s->cal_counting ? s->cal_xtal_first :
                   esp32_timg_cal_xtal_by(s, s->cal_first);
        s->rtc_cal_value = extract64(s->cal_xtal +
                                     esp32_timg_cal_xtal_by(s, e) - at_first,
                                     0, R_TIMG_RTCCALICFG1_VALUE_LENGTH);
        s->rtc_cal_ready = true;
        s->cal_active = false;
        if (!s->rtc_cal_start_cycling) {
            return;
        }
        esp32_timg_cal_edge_time(s, e, &hi, &lo);
        esp32_timg_cal_begin_at(s, false, hi, lo);
        if (clock_is_enabled(s->cal_clk[s->cal_sel])) {
            esp32_timg_cal_skip(s, esp32_timg_cal_edges(s, now));
        }
    }
}

/*
 * Ahead of a change at @now of the group's clock, XTAL_CLK or the
 * measured clock, with the measurement up to then: count its progress so
 * far, the measured clock's cycles and XTAL_CLK's.
 */
static void esp32_timg_cal_fold(Esp32TimgState *s, int64_t now)
{
    uint64_t now_k;

    if (!s->cal_active || !esp32_timg_cal_counts(s)) {
        return;
    }
    if (clock_is_enabled(s->cal_clk[s->cal_sel])) {
        now_k = esp32_timg_cal_edges(s, now);
        if (!s->cal_counting && s->cal_first <= now_k) {
            s->cal_counting = true;
            s->cal_xtal_first = esp32_timg_cal_xtal_by(s, s->cal_first);
        }
        if (s->cal_counting) {
            s->cal_cycles += now_k - s->cal_first;
        }
    }
    if (s->cal_counting) {
        s->cal_xtal += esp32_timg_cal_xtal_edges(s, now) - s->cal_xtal_first;
    }
}

/*
 * The group's clock stops at @now, with the measurement up to then: keep
 * its whole cycles, and XTAL_CLK's edges up to the last of them; it goes
 * on from the measured clock's first edge after the clock runs again.
 */
static void esp32_timg_cal_pause(Esp32TimgState *s, int64_t now)
{
    uint64_t now_k;

    if (!s->cal_active || !clock_is_enabled(s->xtal_clk)) {
        return;
    }
    if (!clock_is_enabled(s->cal_clk[s->cal_sel])) {
        if (s->cal_counting) {
            s->cal_xtal += esp32_timg_cal_xtal_edges(s, now) -
                           s->cal_xtal_first;
            s->cal_counting = false;
        }
        return;
    }
    now_k = esp32_timg_cal_edges(s, now);
    if (!s->cal_counting) {
        if (s->cal_first > now_k) {
            return;
        }
        s->cal_xtal_first = esp32_timg_cal_xtal_by(s, s->cal_first);
    }
    s->cal_cycles += now_k - s->cal_first;
    s->cal_xtal += esp32_timg_cal_xtal_by(s, now_k) - s->cal_xtal_first;
    s->cal_counting = false;
}

/*
 * After the change: the measurement goes on from @now at the clocks' new
 * rates, one waiting for its first edge taking the next one.
 */
static void esp32_timg_cal_rebase(Esp32TimgState *s, int64_t now)
{
    uint64_t now_k;

    if (!s->cal_active) {
        return;
    }
    now_k = esp32_timg_cal_edges(s, now);
    s->cal_first = s->cal_counting ? now_k : now_k + 1;
    s->cal_xtal_first = esp32_timg_cal_xtal_edges(s, now);
}

static void esp32_timg_cal_write(Esp32TimgState *s, uint32_t value,
                                 int64_t now)
{
    bool start = FIELD_EX32(value, TIMG_RTCCALICFG, START);
    bool cycling = FIELD_EX32(value, TIMG_RTCCALICFG, START_CYCLING);
    bool old_start = s->rtc_cal_start;
    bool old_cycling = s->rtc_cal_start_cycling;
    uint32_t sel = FIELD_EX32(value, TIMG_RTCCALICFG, CLK_SEL);

    if (sel >= ESP32_TIMG_CAL_CLK_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_timg: reserved RTC_CALI_CLK_SEL %u\n", sel);
        sel = s->rtc_cal_clk_sel;
    }
    s->rtc_cal_start = start;
    s->rtc_cal_start_cycling = cycling;
    s->rtc_cal_clk_sel = sel;
    s->rtc_cal_max = FIELD_EX32(value, TIMG_RTCCALICFG, MAX);

    if (start && !old_start) {
        esp32_timg_cal_begin(s, true, now);
    } else if (!start && old_start && s->cal_active && s->cal_oneshot) {
        s->cal_active = false;
    }
    if (cycling && !old_cycling && !s->cal_active) {
        esp32_timg_cal_begin(s, false, now);
    } else if (!cycling && old_cycling && s->cal_active && !s->cal_oneshot) {
        s->cal_active = false;
    }
}

/*
 * A clock of the calibration, XTAL_CLK (@sel < 0) or a measured one, is
 * about to change rate, or has: its edges begin afresh from the change.
 */
static void esp32_timg_cal_clk_update(Esp32TimgState *s, int sel,
                                      ClockEvent event)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool counted = sel < 0 || (s->cal_active && s->cal_sel == sel);

    if (event == ClockPreUpdate) {
        esp32_timg_cal_advance(s, now);
        if (sel < 0 || (s->cal_active && s->cal_sel == sel)) {
            esp32_timg_cal_fold(s, now);
        }
        return;
    }
    if (sel < 0) {
        s->xtal_origin = now;
    } else {
        s->cal_origin[sel] = now;
    }
    if (counted) {
        esp32_timg_cal_rebase(s, now);
    }
}

static void esp32_timg_xtal_clk_update(void *opaque, ClockEvent event)
{
    esp32_timg_cal_clk_update(opaque, -1, event);
}

static void esp32_timg_rtc_slow_clk_update(void *opaque, ClockEvent event)
{
    esp32_timg_cal_clk_update(opaque, ESP32_TIMG_CAL_RTC_MUX, event);
}

static void esp32_timg_8md256_clk_update(void *opaque, ClockEvent event)
{
    esp32_timg_cal_clk_update(opaque, ESP32_TIMG_CAL_8MD256, event);
}

static void esp32_timg_xtal32k_clk_update(void *opaque, ClockEvent event)
{
    esp32_timg_cal_clk_update(opaque, ESP32_TIMG_CAL_32K_XTAL, event);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * When the alarm of timer @ts fell due: the time the counter reached
 * ALARM, which the QEMU timer's callback, running late, may be past
 * however far. An alarm fires once (ALARM_EN clears), so there are no
 * periods to replay. ns_base is when the alarm was armed from.
 */
static uint64_t esp32_timg_timer_alarm_ns(Esp32TimgTimerState *ts,
                                          uint64_t ns_now)
{
    int64_t count_to_alarm = ((int64_t)ts->alarm_val -
                              (int64_t)ts->count_base) *
                             esp32_timg_timer_direction(ts);
    uint64_t due;

    if (count_to_alarm <= 0 || !clock_is_enabled(ts->parent->apb_clk)) {
        return ns_now;
    }
    due = ts->ns_base + esp32_timg_timer_count_to_ns(ts, count_to_alarm);
    return MIN(due, ns_now);
}

static void esp32_timg_timer_cb(void *opaque)
{
    Esp32TimgTimerState *ts = (Esp32TimgTimerState*) opaque;
    Esp32TimgState *s = ts->parent;
    /*
     * The alarm acts at its own time: an autoreload restarts the count
     * from then, so a periodic alarm keeps its period however late the
     * host runs the callback.
     */
    uint64_t ns_now = esp32_timg_timer_alarm_ns(
        ts, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

    TIMG_DEBUG_LOG("%s: TG%d ns=0x%llx\n", __func__, s->id, ns_now);
    uint32_t int_mask = 1 << (ts->int_type);

    if (ts->level_int_en) {
        s->int_raw |= int_mask;
        if (s->int_ena & int_mask) {
            qemu_irq_raise(get_level_irq(s, ts->int_type));
        }
    }

    if (ts->edge_int_en) {
        if (s->int_ena & int_mask) {
            qemu_irq_pulse(get_edge_irq(s, ts->int_type));
        }
    }

    ts->alarm = false;

    if (ts->autoreload) {
        esp32_timg_timer_reload(ts, ns_now);
    } else {
        /* ignore overflow modulo 64 bits */
        timer_del(&ts->alarm_timer);
    }
}

static void esp32_timg_int_update_inttype(Esp32TimgState *s, uint32_t int_st, Esp32TimgInterruptType it)
{
    if (int_st & (1 << it)) {
        qemu_irq_raise(get_level_irq(s, it));
    } else {
        qemu_irq_lower(get_level_irq(s, it));
    }
}

static void esp32_timg_int_update(Esp32TimgState *s)
{
    uint32_t int_st = s->int_ena & s->int_raw;
    esp32_timg_int_update_inttype(s, int_st, TIMG_T0_INT);
    esp32_timg_int_update_inttype(s, int_st, TIMG_T1_INT);
    esp32_timg_int_update_inttype(s, int_st, TIMG_WDT_INT);
    esp32_timg_int_update_inttype(s, int_st, TIMG_LACT_INT);
}

static int esp32_timg_timer_direction(Esp32TimgTimerState *s)
{
    if (!s->en) {
        return 0;
    }
    if (s->inc) {
        return 1;
    } else {
        return -1;
    }
}

/* APB_CLK cycles counted between two points of virtual time. */
static uint64_t esp32_timg_apb_ticks(Esp32TimgState *s, uint64_t ns)
{
    return clock_ns_to_ticks(s->apb_clk, ns);
}

/* Virtual time taken by a number of APB_CLK cycles. */
static uint64_t esp32_timg_apb_ns(Esp32TimgState *s, uint64_t ticks,
                                  uint32_t prescale)
{
    if (ticks > UINT64_MAX / prescale) {
        return INT64_MAX;
    }
    return clock_ticks_to_ns(s->apb_clk, ticks * prescale);
}

static uint64_t esp32_timg_timer_get_count(Esp32TimgTimerState *s, uint64_t ns_now)
{
    if (!s->en || !clock_is_enabled(s->parent->apb_clk)) {
        return s->count_base;
    }
    uint64_t ns_from_base = ns_now - s->ns_base;
    uint64_t ticks_from_base =
        esp32_timg_apb_ticks(s->parent, ns_from_base) / s->divider;
    uint64_t count = esp32_timg_timer_direction(s) * ticks_from_base + s->count_base;
    return count;
}

static uint64_t esp32_timg_timer_count_to_ns(Esp32TimgTimerState *s, uint64_t count)
{
    return esp32_timg_apb_ns(s->parent, count, s->divider);
}

static uint32_t esp32_timg_timer_div_from_reg(uint32_t reg_val)
{
    if (reg_val == 0) {
        return 65536;
    }
    if (reg_val == 1 || reg_val == 2) {
        return 2;
    }
    return reg_val;
}

static void esp32_timg_timer_update_config(Esp32TimgTimerState *ts)
{
    uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ts->count_base = esp32_timg_timer_get_count(ts, ns_now);
    ts->ns_base = ns_now;

    ts->en = FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, EN);
    ts->inc = FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, INCREASE);
    ts->autoreload = FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, AUTORELOAD);
    ts->divider = esp32_timg_timer_div_from_reg(FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, DIVIDER));
    ts->edge_int_en = FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, EDGE_INT);
    ts->level_int_en = FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, LEVEL_INT);
    ts->alarm = FIELD_EX32(ts->config_reg, TIMG_T0CONFIG, ALARM);

    TIMG_DEBUG_LOG("%s: TG%d base=0x%llx ns=0x%llx en=%d inc=%d autoreload=%d div=%d li=%d ei=%d alarm=%d\n", __func__, ts->parent->id,
             ts->count_base, ts->ns_base, ts->en, ts->inc, ts->autoreload, ts->divider,
             ts->level_int_en, ts->edge_int_en, ts->alarm);

    esp32_timg_timer_update_alarm(ts, ns_now);
}

static void esp32_timg_timer_reload(Esp32TimgTimerState *ts, uint64_t ns_now)
{
    timer_del(&ts->alarm_timer);

    ts->ns_base = ns_now;
    ts->count_base = ts->load_val;

    TIMG_DEBUG_LOG("%s: TG%d base=0x%llx ns=0x%llx\n", __func__, ts->parent->id,
             ts->count_base, ts->ns_base);

    esp32_timg_timer_update_alarm(ts, ns_now);
}

static void esp32_timg_timer_update_alarm(Esp32TimgTimerState *ts, uint64_t ns_now)
{
    if (!ts->en || !ts->alarm || !clock_is_enabled(ts->parent->apb_clk)) {
        timer_del(&ts->alarm_timer);
        return;
    }

    int64_t count_to_alarm = ((int64_t) ts->alarm_val - (int64_t) ts->count_base)
                                * esp32_timg_timer_direction(ts);
    if (count_to_alarm <= 0) {
        /* ignore overflow modulo 64 bits */
        timer_del(&ts->alarm_timer);
        return;
    }

    uint64_t ns_to_alarm = esp32_timg_timer_count_to_ns(ts, count_to_alarm);

    TIMG_DEBUG_LOG("%s: TG%d count_to_alarm=0x%llx ns_to_alarm=0x%llx\n", __func__, ts->parent->id,
                 count_to_alarm, ns_to_alarm);

    timer_mod_anticipate_ns(&ts->alarm_timer, ns_now + ns_to_alarm);
}

static bool esp32_timg_wdt_protected(Esp32TimgWdtState *ws)
{
    return ws->protect_reg != ESP32_TIMG_WDT_PROTECT_WORD;
}

static uint64_t esp32_timg_wdt_get_count(Esp32TimgWdtState *ws, uint64_t ns_now)
{
    if (!ws->en || !clock_is_enabled(ws->parent->apb_clk)) {
        return ws->count_base;
    }
    uint64_t ns_from_base = ns_now - ws->ns_base;
    uint64_t ticks_from_base =
        esp32_timg_apb_ticks(ws->parent, ns_from_base) / MAX(ws->prescale, 1);
    uint64_t count = ticks_from_base + ws->count_base;
    return count;
}

static void esp32_timg_wdt_update_config(Esp32TimgWdtState *ws)
{
    Esp32TimgState *s = ws->parent;

    uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ws->count_base = esp32_timg_wdt_get_count(ws, ns_now);
    ws->ns_base = ns_now;

    bool old_en = ws->en;
    ws->en = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, EN);
    ws->mode[0] = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, STG0);
    ws->mode[1] = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, STG1);
    ws->mode[2] = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, STG2);
    ws->mode[3] = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, STG3);
    ws->edge_int_en = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, EDGE_INT);
    ws->level_int_en = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, LEVEL_INT);
    ws->flashboot_en = FIELD_EX32(ws->config0_reg, TIMG_WDTCONFIG0, FLASHBOOT_MODE_EN);

    ws->prescale = FIELD_EX32(ws->config1_reg, TIMG_WDTCONFIG1, PRESCALE);

    if (ws->en && !old_en) {
        ws->cur_stage = 0;
        ws->count_base = 0;
    } else if (!ws->en && old_en) {
        qemu_irq_lower(get_level_irq(s, TIMG_WDT_INT));
    }

    TIMG_DEBUG_LOG("%s: TG%d config 0x%08x prescale=0x%08x en=%d fb_en=%d level_int_en=%d\n", __func__, ws->parent->id,
                   ws->config0_reg, ws->prescale, ws->en, ws->flashboot_en, ws->level_int_en);
    esp32_timg_wdt_arm(ws, ns_now);
}

static void esp32_timg_wdt_feed(Esp32TimgWdtState *ws)
{
    TIMG_DEBUG_LOG("%s TG%d\n", __func__, ws->parent->id);
    uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ws->cur_stage = 0;
    ws->ns_base = ns_now;
    ws->count_base = 0;
    esp32_timg_wdt_arm(ws, ns_now);
}

static void esp32_timg_wdt_arm(Esp32TimgWdtState *ws, uint64_t ns_now)
{
    timer_del(&ws->stage_timer);

    if (ws->parent->wdt_disable || !(ws->en || (ws->flashboot_en && ws->parent->flash_boot_mode))) {
        return;
    }
    if (!clock_is_enabled(ws->parent->apb_clk)) {
        return;
    }

    uint32_t stage_timeout = ws->timeout[ws->cur_stage];
    uint32_t cur_count = esp32_timg_wdt_get_count(ws, ns_now);
    uint32_t count_to_timeout = stage_timeout - cur_count;
    uint64_t ns_to_timeout = esp32_timg_apb_ns(ws->parent, count_to_timeout,
                                               MAX(ws->prescale, 1));
    TIMG_DEBUG_LOG("%s: TG%d ns=0x%08llx stage %d count=0x%08x count_to_timeout=0x%08x ns_to_timeout=0x%08llx\n",
                   __func__, ws->parent->id, ns_now, ws->cur_stage, cur_count, count_to_timeout, ns_to_timeout);
    timer_mod_anticipate_ns(&ws->stage_timer, ns_now + ns_to_timeout);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * When the watchdog's current stage ended: the next stage counts from
 * then, not from when the QEMU timer's callback, running late, got to it.
 */
static uint64_t esp32_timg_wdt_stage_end_ns(Esp32TimgWdtState *ws,
                                            uint64_t ns_now)
{
    uint32_t left = ws->timeout[ws->cur_stage] - ws->count_base;
    uint64_t due;

    if (!clock_is_enabled(ws->parent->apb_clk) ||
        ws->count_base > ws->timeout[ws->cur_stage]) {
        return ns_now;
    }
    due = ws->ns_base + esp32_timg_apb_ns(ws->parent, left,
                                          MAX(ws->prescale, 1));
    if (due > ns_now || ns_now - due > TIMG_CATCHUP_NS) {
        return ns_now;
    }
    return due;
}

static void esp32_timg_wdt_cb(void *opaque)
{
    Esp32TimgWdtState *ws = (Esp32TimgWdtState*) opaque;
    Esp32TimgState *s = ws->parent;
    Esp32TimgWdtStageMode mode = ws->mode[ws->cur_stage];
    TIMG_DEBUG_LOG("%s: TG%d stage %d timeout mode %d\n", __func__, s->id, ws->cur_stage, mode);
    if (mode == WDT_MODE_INT) {
        uint32_t mask = 1 << TIMG_WDT_INT;
        if (ws->level_int_en) {
            s->int_raw |= mask;
            if (true) { /* should be checking s->int_ena & mask, but ESP32 seems to raise interrupt regardless? */
                qemu_irq_raise(get_level_irq(s, TIMG_WDT_INT));
            }
        }
        if (ws->edge_int_en) {
            if (s->int_ena & mask) {
                qemu_irq_pulse(get_edge_irq(s, TIMG_WDT_INT));
            }
        }
    } else if (mode == WDT_MODE_CPURESET) {
        qemu_irq_pulse(s->wdt_cpu_reset_req);
    } else if (mode == WDT_MODE_SYSRESET) {
        qemu_irq_pulse(s->wdt_sys_reset_req);
    }

    int next_stage = (ws->cur_stage + 1) % ESP32_TIMG_WDT_STAGE_COUNT;
    uint64_t ns_now = esp32_timg_wdt_stage_end_ns(
        ws, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    ws->count_base = 0;
    ws->cur_stage = next_stage;
    ws->ns_base = ns_now;
    esp32_timg_wdt_arm(ws, ns_now);
}

static void esp32_timg_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_timg_timer_init(Esp32TimgState *s, Esp32TimgTimerState *ts, Esp32TimgInterruptType int_type) {
    ts->parent = s;
    timer_init_ns(&ts->alarm_timer, QEMU_CLOCK_VIRTUAL, esp32_timg_timer_cb, ts);
    ts->int_type = int_type;
}

static void esp32_timg_init(Object *obj)
{
    Esp32TimgState *s = ESP32_TIMG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_timg_ops, s,
                          TYPE_ESP32_TIMG, TIMG_REGFILE_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out_named(DEVICE(sbd), s->irqs, SYSBUS_DEVICE_GPIO_IRQ, 2*TIMG_INT_MAX);

    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb",
                                    esp32_timg_apb_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);

    s->xtal_clk = qdev_init_clock_in(DEVICE(obj), ESP32_TIMG_XTAL_CLK,
                                     esp32_timg_xtal_clk_update, s,
                                     ClockPreUpdate | ClockUpdate);
    s->cal_clk[ESP32_TIMG_CAL_RTC_MUX] =
        qdev_init_clock_in(DEVICE(obj), ESP32_TIMG_RTC_SLOW_CLK,
                           esp32_timg_rtc_slow_clk_update, s,
                           ClockPreUpdate | ClockUpdate);
    s->cal_clk[ESP32_TIMG_CAL_8MD256] =
        qdev_init_clock_in(DEVICE(obj), ESP32_TIMG_8MD256_CLK,
                           esp32_timg_8md256_clk_update, s,
                           ClockPreUpdate | ClockUpdate);
    s->cal_clk[ESP32_TIMG_CAL_32K_XTAL] =
        qdev_init_clock_in(DEVICE(obj), ESP32_TIMG_XTAL32K_CLK,
                           esp32_timg_xtal32k_clk_update, s,
                           ClockPreUpdate | ClockUpdate);

    esp32_timg_timer_init(s, &s->t0, TIMG_T0_INT);
    esp32_timg_timer_init(s, &s->t1, TIMG_T1_INT);
    esp32_timg_timer_init(s, &s->lact, TIMG_LACT_INT);

    s->wdt.parent = s;
    timer_init_ns(&s->wdt.stage_timer, QEMU_CLOCK_VIRTUAL, esp32_timg_wdt_cb, &s->wdt);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->wdt_cpu_reset_req, ESP32_TIMG_WDT_CPU_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->wdt_sys_reset_req, ESP32_TIMG_WDT_SYS_RESET_GPIO, 1);
}

static const Property esp32_timg_properties[] = {
    DEFINE_PROP_BOOL("wdt_disable", Esp32TimgState, wdt_disable, false),
};

static void esp32_timg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_timg_reset_hold;
    dc->realize = esp32_timg_realize;
    device_class_set_props(dc, esp32_timg_properties);
}

static const TypeInfo esp32_timg_info = {
    .name = TYPE_ESP32_TIMG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32TimgState),
    .instance_init = esp32_timg_init,
    .class_init = esp32_timg_class_init
};

static void esp32_timg_register_types(void)
{
    type_register_static(&esp32_timg_info);
}

type_init(esp32_timg_register_types)
