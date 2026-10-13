/*
 * ESP32-P4 timer group (TIMG): two general-purpose timers, the MWDT and
 * the RTC calibration unit
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapters
 * 17 (Timer Group) and 18 (Watchdog Timers), and ESP-IDF's
 * timer_group_reg.h for v3, which gives the registers' layout, access and
 * reset values; its hal timer_ll.h, mwdt_ll.h and wdt_hal_iram.c, and
 * esp_hw_support's rtc_time.c, show how software drives the block.
 *
 * Timers. T0 and T1 each count TB_CLK, their function clock
 * (TIMERGRPn_Tx_CLK: XTAL_CLK, RC_FAST_CLK or PLL_F80M_CLK, which
 * HP_SYS_CLKRST selects and gates) divided by TIMG_Tx_DIVIDER (0 dividing
 * by 65536, 1 and 2 by 2), in a 54-bit counter, up or down
 * (TIMG_Tx_INCREASE, which turns it at once) while TIMG_Tx_EN is set and
 * the function clock runs. TIMG_Tx_DIVCNT_RST restarts the prescaler's
 * count; changing the divider while the timer counts is logged as a guest
 * error, the TRM calling the result unpredictable, and takes effect from
 * the next TB_CLK cycle. Writing TIMG_TxUPDATE_REG latches the count into
 * TIMG_TxLO/HI_REG (at once: UPDATE reads as clear when software polls
 * it), and writing TIMG_TxLOAD_REG loads TIMG_Tx_LOAD_LO/HI.
 *
 * Alarms. With TIMG_Tx_ALARM_EN set, the timer alarms when its count
 * reaches the alarm value, or at once if the count is 0 to 2^53 - 1 counts
 * past it in its direction (TRM tables 17.3-1 and 17.3-2: a value further
 * ahead is reached after the counter wraps); the comparison needs the
 * function clock to run, but not TIMG_Tx_EN. An alarm clears ALARM_EN,
 * sets TIMG_Tx_INT_RAW, reloads the count from LOAD_LO/HI if
 * TIMG_Tx_AUTORELOAD is set, and while TIMG_ETM_EN is set pulses the
 * timer's ETM event. The ETM tasks START (sets EN), STOP (clears it),
 * ALARM_START (sets ALARM_EN), RELOAD and CAP (latches the count, as
 * UPDATE does) act while TIMG_ETM_EN is set. TIMG1 has the ETM tasks and
 * events of both of its timers too, as ESP-IDF's soc_etm_source.h has
 * them (the TRM lists timer 0's).
 *
 * MWDT (TRM 18.2). The watchdog counts its function clock
 * (TIMERGRPn_WDT_CLK) divided by TIMG_WDT_CLK_PRESCALE (0 taken as 65536,
 * as the timers' divider), through four stages in a loop: in each its
 * 32-bit counter counts from 0 until it reaches the stage's hold value,
 * TIMG_WDT_STGn_HOLD (a hold of 0, or one below a count already reached,
 * being reached after the counter wraps), then the stage's action is taken,
 * the counter restarts from 0 and the next stage begins. The actions
 * (TIMG_WDT_STGn): 0, none; 1, an interrupt (TIMG_WDT_INT_RAW); 2, a CPU
 * reset of core 0 if TIMG_WDT_PROCPU_RESET_EN is set and core 1 if
 * APPCPU_RESET_EN is (each core taking it only from the MWDT
 * HP_SYS_CLKRST_HPCORE<n>_WDT_RESET_SOURCE_SEL picks); 3, an HP core reset
 * (cause 0x07), which the header calls "reset system", held for
 * TIMG_WDT_SYS_RESET_LENGTH's time (100 ns to 3.2 us). The CPU reset's
 * length is HP_SYS_CLKRST's (HPWDT_CORE<n>_RST_CTRL0), so
 * TIMG_WDT_CPU_RESET_LENGTH is held as written. Feeding (any write to
 * TIMG_WDTFEED_REG) restarts stage 0 from 0; a disabled watchdog holds its
 * stage and count at 0. TIMG_WDT_EN takes effect at once; the stage
 * actions and holds, the lengths, the CPU reset enables, flash boot
 * protection and the prescaler take effect when TIMG_WDT_CONF_UPDATE_EN
 * is written, as ESP-IDF's mwdt_ll writes it after each change.
 * TIMG_WDT_DIVCNT_RST restarts the prescaler's count. Every MWDT register
 * but TIMG_WDTWPROTECT_REG takes writes only while that holds 0x50D83AA1
 * (its reset value). Flash boot protection: while
 * TIMG_WDT_FLASHBOOT_MOD_EN is in effect and the chip booted in SPI boot
 * mode, MWDT0 runs although TIMG_WDT_EN is clear, its stage 0 resetting
 * the HP core. The block's own reset, its reset line or an HP core reset,
 * leaves it as the chip resets it, so flash boot protection runs again.
 *
 * RTC calibration (TRM 17.3.6, 17.4.5). The unit counts XTAL_CLK over
 * TIMG_RTC_CALI_MAX cycles of the clock it measures, TIMG0_CALI_CLK,
 * which on the P4 is HP_SYS_CLKRST's TIMERGRP0_TGRT_CLK (its source and
 * divider select the clock; TIMG_RTC_CALI_CLK_SEL, from earlier chips, is
 * held as written and selects nothing). Counting starts at the measured
 * clock's first edge after the measurement begins and ends at its
 * RTC_CALI_MAX'th edge after that (at that first edge for 0); the count of
 * XTAL_CLK edges between goes to TIMG_RTC_CALI_VALUE (25 bits, wrapping),
 * each clock's edges at their exact times, at whatever phase to the other.
 * A rising edge of TIMG_RTC_CALI_START starts a one-shot measurement,
 * clearing TIMG_RTC_CALI_RDY; clearing START abandons it.
 * TIMG_RTC_CALI_START_CYCLING (set at reset) measures continuously, each
 * measurement beginning where the last ended and each end setting
 * TIMG_RTC_CALI_CYCLING_DATA_VLD, which a rising edge of START_CYCLING
 * clears. The end of every measurement, one-shot or cycling, sets RDY.
 * The TRM calls RDY the one-shot's, but ESP-IDF's rtc_clk_cal_internal()
 * waits on RDY for a cycling measurement under way, with RDY clear from
 * reset, and in its first versions (v4.2 on the ESP32-S2, v4.3 on the C3
 * and S3) had no other way out of that wait; on the P4, with MPLL_CLK
 * measured as from reset, no cycling measurement times out at the
 * threshold of 1 it sets for it. A measurement that has run more
 * XTAL_CLK cycles than TIMG_RTC_CALI_TIMEOUT_THRES without ending is
 * abandoned with TIMG_RTC_CALI_TIMEOUT set, which stays set until
 * software starts a measurement; a cycling one starts again
 * TIMG_RTC_CALI_TIMEOUT_RST_CNT XTAL_CLK cycles later. Each measurement's
 * XTAL_CLK cycles are counted from its own beginning. A measured clock
 * that does not run (a source that is powered down or gated, or timer
 * group 1's, which has none) never ends, so only the timeout does. Clock
 * changes during a measurement are folded in, the measurement going on at
 * the new edges from the change.
 * The clock and cycle count are latched as a measurement starts. Nothing
 * outside the block sees a measurement end (the unit has no interrupt or
 * ETM event), so the model keeps no QEMU timer for it: an access to its
 * registers, or a change of either clock, brings it up to that moment
 * from the two clocks' edges (timg_cal_advance()). Cycling from reset on
 * the 500 MHz MPLL_CLK, a measurement ends every 4 ns; the measurements
 * whose outcome the two clocks' periods settle are passed over in one go,
 * so that what the unit costs the host does not depend on the clocks'
 * rates.
 *
 * Interrupts. TIMG_Tx_INT and TIMG_WDT_INT are levels, each its raw bit
 * and enable (sources TGn_T0, TGn_T1 and TGn_WDT of the interrupt matrix).
 * TIMG_REGCLK_REG's CLK_EN, the register file's clock gating, has no
 * effect software can see. The registers are clocked by
 * TIMERGRPn_APB_CLK; the machine maps them behind it and the group's
 * reset. Byte and halfword accesses read and write their bytes of a
 * register.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/timer/esp32p4_timg.h"
#include "migration/vmstate.h"

#define A_TCONFIG(x)        (0x00 + 0x24 * (x))
#define TIMER_REGS_END      0x48
#define R_TCONFIG           0x00
#define R_TLO               0x04
#define R_THI               0x08
#define R_TUPDATE           0x0c
#define R_TALARMLO          0x10
#define R_TALARMHI          0x14
#define R_TLOADLO           0x18
#define R_TLOADHI           0x1c
#define R_TLOAD             0x20
#define A_WDTCONFIG0        0x48
#define A_WDTCONFIG1        0x4c
#define A_WDTCONFIG2        0x50
#define A_WDTCONFIG5        0x5c
#define A_WDTFEED           0x60
#define A_WDTWPROTECT       0x64
#define A_RTCCALICFG        0x68
#define A_RTCCALICFG1       0x6c
#define A_INT_ENA           0x70
#define A_INT_RAW           0x74
#define A_INT_ST            0x78
#define A_INT_CLR           0x7c
#define A_RTCCALICFG2       0x80
#define A_DATE              0xf8
#define A_REGCLK            0xfc

#define TCONF_ALARM_EN      BIT(10)
#define TCONF_DIVCNT_RST    BIT(12)
#define TCONF_DIVIDER(v)    extract32((v), 13, 16)
#define TCONF_AUTORELOAD    BIT(29)
#define TCONF_INCREASE      BIT(30)
#define TCONF_EN            BIT(31)
#define TCONF_RW            0xffffe400
#define TCONF_RESET         0x60002000
#define TUPDATE_UPDATE      BIT(31)
#define TIMER_HI_MASK       0x003fffff
#define COUNT_MASK          MAKE_64BIT_MASK(0, 54)
#define COUNT_HALF          (1ull << 53)

#define WCONF0_APPCPU_RESET_EN  BIT(12)
#define WCONF0_PROCPU_RESET_EN  BIT(13)
#define WCONF0_FLASHBOOT_MOD_EN BIT(14)
#define WCONF0_SYS_RESET_LENGTH(v) extract32((v), 15, 3)
#define WCONF0_CONF_UPDATE_EN   BIT(22)
#define WCONF0_STG(v, n)        extract32((v), 29 - 2 * (n), 2)
#define WCONF0_EN               BIT(31)
#define WCONF0_RW               0xff9ff000
#define WCONF0_RESET            0x0004c000
#define WCONF1_DIVCNT_RST       BIT(0)
#define WCONF1_PRESCALE(v)      extract32((v), 16, 16)
#define WCONF1_RW               0xffff0000
#define WCONF1_RESET            0x00010000
#define WDT_WKEY                0x50d83aa1

enum {
    STG_OFF,
    STG_INT,
    STG_CPU_RESET,
    STG_CORE_RESET,
};

#define CAL_START_CYCLING   BIT(12)
#define CAL_RDY             BIT(15)
#define CAL_MAX(v)          extract32((v), 16, 15)
#define CAL_START           BIT(31)
#define CAL_RW              0xffff7000
#define CAL_RESET           0x00011000
#define CAL1_VLD            BIT(0)
#define CAL1_VALUE_SHIFT    7
#define CAL_VALUE_BITS      25
#define CAL2_TIMEOUT        BIT(0)
#define CAL2_RST_CNT(v)     extract32((v), 3, 4)
#define CAL2_THRES(v)       extract32((v), 7, 25)
#define CAL2_RW             0xfffffff8
#define CAL2_RESET          0xffffff98

#define INT_WDT             BIT(ESP32P4_TIMG_TIMERS)
#define INT_MASK            0x7
#define DATE_MASK           0x0fffffff
#define DATE_RESET          0x02209142
#define REGCLK_ETM_EN       BIT(28)
#define REGCLK_RW           0x90000000
#define REGCLK_RESET        0x10000000

static const uint32_t wdt_hold_reset[ESP32P4_TIMG_WDT_STAGES] = {
    26000000, 0x07ffffff, 0x000fffff, 0x000fffff,
};

/* The HP core reset's length for each TIMG_WDT_SYS_RESET_LENGTH. */
static const uint16_t wdt_reset_length_ns[8] = {
    100, 200, 300, 400, 500, 800, 1600, 3200,
};

static void timg_update_irqs(ESP32P4TimgState *s)
{
    uint32_t st = s->int_raw & s->int_ena;
    int i;

    for (i = 0; i <= ESP32P4_TIMG_TIMERS; i++) {
        qemu_set_irq(s->irq[i], extract32(st, i, 1));
    }
}

/* The timers ---------------------------------------------------------- */

static uint32_t timg_divisor(uint32_t config)
{
    uint32_t div = TCONF_DIVIDER(config);

    return div == 0 ? 65536 : div < 3 ? 2 : div;
}

/* Function clock cycles timer @t has counted since its sync point. */
static uint64_t timg_timer_cycles(ESP32P4TimgTimer *t, int64_t now)
{
    uint64_t cycles;

    if (!t->running) {
        return 0;
    }
    cycles = esp32p4_clk_count_at(&t->edges, now);
    return cycles > t->synced ? cycles - t->synced : 0;
}

static uint64_t timg_timer_count(ESP32P4TimgTimer *t, int64_t now)
{
    uint64_t ticks = (t->pre + timg_timer_cycles(t, now)) /
                     timg_divisor(t->config);

    if (t->config & TCONF_INCREASE) {
        return (t->count + ticks) & COUNT_MASK;
    }
    return (t->count - ticks) & COUNT_MASK;
}

/*
 * Fold the cycles counted up to @now into the count and the prescaler. A
 * sync at a time before the last, for a task whose event came earlier,
 * changes nothing; while the timer is stopped, it notes the time, from
 * which a start counts.
 */
static void timg_timer_sync(ESP32P4TimgTimer *t, int64_t now)
{
    uint64_t cycles = timg_timer_cycles(t, now);

    t->count = timg_timer_count(t, now);
    if (!t->running) {
        if (now > t->edges.origin_ns) {
            esp32p4_clk_count_rebase(&t->edges, now);
        }
        return;
    }
    t->pre = (t->pre + cycles) % timg_divisor(t->config);
    t->synced += cycles;
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Timer @x alarms at @when: its ETM event carries that time, which a
 * QEMU timer running late, or a register access catching up, has passed.
 */
static void timg_timer_fire(ESP32P4TimgState *s, int x, int64_t when)
{
    ESP32P4TimgTimer *t = &s->t[x];

    t->config &= ~TCONF_ALARM_EN;
    if (t->config & TCONF_AUTORELOAD) {
        t->count = t->load;
    }
    s->int_raw |= BIT(x);
    timg_update_irqs(s);
    if (s->regclk & REGCLK_ETM_EN) {
        esp32_line_set_at(s->etm_event[x], 1, when);
        esp32_line_set_at(s->etm_event[x], 0, when);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * With the count synced at @now, alarm at once if the count has reached
 * or passed the alarm value, or schedule the alarm.
 */
static void timg_timer_check(ESP32P4TimgState *s, int x, int64_t now)
{
    ESP32P4TimgTimer *t = &s->t[x];
    uint32_t div = timg_divisor(t->config);
    uint64_t ahead, cycles;

    timer_del(&t->timer);
    if (!(t->config & TCONF_ALARM_EN) || !clock_is_enabled(t->clk)) {
        return;
    }
    if (t->config & TCONF_INCREASE) {
        ahead = (t->alarm - t->count) & COUNT_MASK;
    } else {
        ahead = (t->count - t->alarm) & COUNT_MASK;
    }
    if (ahead == 0 || ahead > COUNT_HALF) {
        timg_timer_fire(s, x, now);
        return;
    }
    if (!t->running) {
        return;
    }
    if (ahead > (UINT64_MAX - t->synced) / div) {
        t->due_ns = INT64_MAX / 4;
    } else {
        cycles = ahead * div - t->pre;
        t->due_ns = MIN(esp32p4_clk_count_time(&t->edges,
                                               t->synced + cycles),
                        INT64_MAX / 4);
    }
    timer_mod(&t->timer, t->due_ns);
}

/*
 * Re-evaluate whether timer @x counts, after a sync at @now. A timer
 * started by a task whose event came before the timer's last change, as
 * one from a block whose QEMU timer ran late can without icount, counts
 * from that change: a start never undoes what software saw since.
 */
static void timg_timer_update(ESP32P4TimgState *s, int x, int64_t now)
{
    ESP32P4TimgTimer *t = &s->t[x];
    bool running = (t->config & TCONF_EN) && clock_is_enabled(t->clk);

    if (running && !t->running) {
        esp32p4_clk_count_start(&t->edges, t->clk,
                                MAX(now, t->edges.origin_ns));
        t->synced = 0;
    }
    t->running = running;
    timg_timer_check(s, x, now);
}

/*
 * Timer @x's alarm falls due: run it as of its due time, which a QEMU
 * timer running late has passed.
 */
static void timg_timer_due(ESP32P4TimgState *s, int x)
{
    ESP32P4TimgTimer *t = &s->t[x];

    timer_del(&t->timer);
    timg_timer_sync(t, t->due_ns);
    timg_timer_check(s, x, t->due_ns);
}

/* Run timer @x's alarm if it fell due by @now. */
static void timg_timer_catch_up(ESP32P4TimgState *s, int x, int64_t now)
{
    ESP32P4TimgTimer *t = &s->t[x];

    if (timer_pending(&t->timer) && t->due_ns <= now) {
        timg_timer_due(s, x);
    }
}

static void timg_timer_cb(void *opaque)
{
    ESP32P4TimgTimer *t = opaque;

    timg_timer_due(t->s, t - t->s->t);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_timer_write_config(ESP32P4TimgState *s, int x,
                                    uint32_t value, int64_t now)
{
    ESP32P4TimgTimer *t = &s->t[x];
    uint32_t div;

    timg_timer_sync(t, now);
    if ((t->config & TCONF_EN) && (value & TCONF_EN) &&
        TCONF_DIVIDER(value) != TCONF_DIVIDER(t->config)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s%u: timer %d's divider changed "
                      "while it counts\n", TYPE_ESP32P4_TIMG, s->group, x);
    }
    t->config = value & TCONF_RW;
    div = timg_divisor(t->config);
    if ((value & TCONF_DIVCNT_RST) || t->pre >= div) {
        t->pre = 0;
    }
    timg_timer_update(s, x, now);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_timer_clk_update(ESP32P4TimgState *s, int x,
                                  ClockEvent event)
{
    ESP32P4TimgTimer *t = &s->t[x];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        timg_timer_catch_up(s, x, now);
        timg_timer_sync(t, now);
        return;
    }
    esp32p4_clk_count_start(&t->edges, t->clk, now);
    t->synced = 0;
    timg_timer_update(s, x, now);
}

static void timg_t0_clk_update(void *opaque, ClockEvent event)
{
    timg_timer_clk_update(opaque, 0, event);
}

static void timg_t1_clk_update(void *opaque, ClockEvent event)
{
    timg_timer_clk_update(opaque, 1, event);
}

/* The MWDT ------------------------------------------------------------ */

static uint32_t timg_wdt_prescale(ESP32P4TimgWdt *w)
{
    return w->act_prescale ? w->act_prescale : 65536;
}

/* Flash boot protection: the MWDT runs with TIMG_WDT_EN clear. */
static bool timg_wdt_flashboot(ESP32P4TimgState *s)
{
    return !(s->wdt.config0 & WCONF0_EN) && s->spi_boot &&
           (s->wdt.act_config0 & WCONF0_FLASHBOOT_MOD_EN);
}

static bool timg_wdt_enabled(ESP32P4TimgState *s)
{
    return (s->wdt.config0 & WCONF0_EN) || timg_wdt_flashboot(s);
}

static uint64_t timg_wdt_cycles(ESP32P4TimgWdt *w, int64_t now)
{
    uint64_t cycles;

    if (!w->running) {
        return 0;
    }
    cycles = esp32p4_clk_count_at(&w->edges, now);
    return cycles > w->synced ? cycles - w->synced : 0;
}

static void timg_wdt_sync(ESP32P4TimgWdt *w, int64_t now)
{
    uint64_t cycles = timg_wdt_cycles(w, now);
    uint32_t ps = timg_wdt_prescale(w);

    w->count += (w->pre + cycles) / ps;
    w->pre = (w->pre + cycles) % ps;
    w->synced += cycles;
}

/* Schedule the current stage's timeout, after a sync. */
static void timg_wdt_schedule(ESP32P4TimgState *s)
{
    ESP32P4TimgWdt *w = &s->wdt;
    uint64_t ahead;

    timer_del(&w->timer);
    if (!w->running) {
        return;
    }
    ahead = (uint32_t)(w->act_hold[w->stage] - w->count);
    if (!ahead) {
        ahead = 1ull << 32;
    }
    w->due_ns = MIN(esp32p4_clk_count_time(&w->edges, w->synced +
                                           ahead * timg_wdt_prescale(w) -
                                           w->pre),
                    INT64_MAX / 4);
    timer_mod(&w->timer, w->due_ns);
}

/* Re-evaluate whether the MWDT runs, after a sync at @now. */
static void timg_wdt_update(ESP32P4TimgState *s, int64_t now)
{
    ESP32P4TimgWdt *w = &s->wdt;
    bool enabled = timg_wdt_enabled(s);
    bool running = enabled && clock_is_enabled(w->clk);

    if (!enabled) {
        w->stage = 0;
        w->count = 0;
        w->pre = 0;
    }
    if (running && !w->running) {
        esp32p4_clk_count_start(&w->edges, w->clk, now);
        w->synced = 0;
    }
    w->running = running;
    timg_wdt_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Take a stage's action. An HP core reset resets this block too, so it
 * comes last.
 */
static void timg_wdt_act(ESP32P4TimgState *s, unsigned action)
{
    ESP32P4TimgWdt *w = &s->wdt;

    switch (action) {
    case STG_INT:
        s->int_raw |= INT_WDT;
        timg_update_irqs(s);
        break;
    case STG_CPU_RESET:
        if (w->act_config0 & WCONF0_PROCPU_RESET_EN) {
            qemu_irq_pulse(s->cpu_reset[0]);
        }
        if (w->act_config0 & WCONF0_APPCPU_RESET_EN) {
            qemu_irq_pulse(s->cpu_reset[1]);
        }
        break;
    case STG_CORE_RESET:
        esp32p4_clkrst_hp_core_reset(s->clkrst,
            ESP32P4_HP_CORE_RESET_MWDT0 + s->group,
            wdt_reset_length_ns[WCONF0_SYS_RESET_LENGTH(w->act_config0)]);
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * The current stage times out at @due: the next begins, and the stage's
 * action is taken.
 */
static void timg_wdt_timeout(ESP32P4TimgState *s, int64_t due)
{
    ESP32P4TimgWdt *w = &s->wdt;
    unsigned action;

    timg_wdt_sync(w, due);
    action = timg_wdt_flashboot(s) ? (w->stage ? STG_OFF : STG_CORE_RESET)
                                   : WCONF0_STG(w->act_config0, w->stage);
    w->stage = (w->stage + 1) % ESP32P4_TIMG_WDT_STAGES;
    w->count = 0;
    timg_wdt_schedule(s);
    timg_wdt_act(s, action);
}

static void timg_wdt_cb(void *opaque)
{
    ESP32P4TimgState *s = opaque;

    timg_wdt_timeout(s, s->wdt.due_ns);
}

static void timg_wdt_catch_up(ESP32P4TimgState *s, int64_t now)
{
    if (timer_pending(&s->wdt.timer) && s->wdt.due_ns <= now) {
        timer_del(&s->wdt.timer);
        timg_wdt_timeout(s, s->wdt.due_ns);
    }
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_wdt_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4TimgState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        timg_wdt_catch_up(s, now);
        timg_wdt_sync(&s->wdt, now);
        return;
    }
    esp32p4_clk_count_start(&s->wdt.edges, s->wdt.clk, now);
    s->wdt.synced = 0;
    timg_wdt_update(s, now);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_set_spi_boot(void *opaque, int n, int level)
{
    ESP32P4TimgState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timg_wdt_catch_up(s, now);
    timg_wdt_sync(&s->wdt, now);
    s->spi_boot = level;
    timg_wdt_update(s, now);
}

/* The RTC calibration unit -------------------------------------------- */

static bool timg_cal_xtal_runs(ESP32P4TimgState *s)
{
    return esp32p4_clk_count_running(&s->xtal_edges);
}

static bool timg_cal_clk_runs(ESP32P4TimgState *s)
{
    return esp32p4_clk_count_running(&s->cal_edges);
}

/* XTAL_CLK's cycles (of @xtal_edges) by the end of measured cycle @k. */
static uint64_t timg_cal_xtal_by(ESP32P4TimgState *s, uint64_t k)
{
    return esp32p4_clk_count_at_cycle(&s->xtal_edges, &s->cal_edges, k);
}

/* The measured clock's cycles (of @cal_edges) by XTAL_CLK's cycle @i. */
static uint64_t timg_cal_clk_by(ESP32P4TimgState *s, uint64_t i)
{
    return esp32p4_clk_count_at_cycle(&s->cal_edges, &s->xtal_edges, i);
}

/* The XTAL_CLK cycles that time a measurement out: one past the threshold. */
static uint64_t timg_cal_limit(ESP32P4TimgCal *c)
{
    return (uint64_t)CAL2_THRES(c->cfg2) + 1;
}

static void timg_cal_stop(ESP32P4TimgState *s)
{
    s->cal.active = false;
    s->cal.restart_pending = false;
}

static void timg_cal_start(ESP32P4TimgState *s, bool oneshot)
{
    ESP32P4TimgCal *c = &s->cal;

    c->active = true;
    c->restart_pending = false;
    c->oneshot = oneshot;
    c->counting = false;
    c->target = CAL_MAX(c->cfg);
    c->cycles = 0;
    c->xtal_at_first = 0;
    c->since_start = 0;
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Begin a measurement at @now; one software starts clears
 * TIMG_RTC_CALI_TIMEOUT.
 */
static void timg_cal_begin(ESP32P4TimgState *s, bool oneshot,
                           bool by_software, int64_t now)
{
    ESP32P4TimgCal *c = &s->cal;

    timg_cal_start(s, oneshot);
    c->first = esp32p4_clk_count_at(&s->cal_edges, now) + 1;
    c->xtal_first = esp32p4_clk_count_at(&s->xtal_edges, now);
    if (by_software) {
        c->timeout = false;
    }
    if (oneshot) {
        c->rdy = false;
    }
}

/* A cycling measurement begins at the end of measured cycle @k. */
static void timg_cal_begin_at_clk(ESP32P4TimgState *s, uint64_t k)
{
    timg_cal_start(s, false);
    s->cal.first = k + 1;
    s->cal.xtal_first = timg_cal_xtal_by(s, k);
}

/* A cycling measurement begins at the end of XTAL_CLK's cycle @i. */
static void timg_cal_begin_at_xtal(ESP32P4TimgState *s, uint64_t i)
{
    timg_cal_start(s, false);
    s->cal.first = timg_cal_clk_by(s, i) + 1;
    s->cal.xtal_first = i;
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * The measurement ends at the end of measured cycle @e: the value is
 * XTAL_CLK's edges since its first edge, and RDY is set, by a cycling
 * measurement as by a one-shot. A cycling one sets CYCLING_DATA_VLD too
 * and begins again there.
 */
static void timg_cal_end(ESP32P4TimgState *s, uint64_t e)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t at_first = c->counting ? c->xtal_at_first :
                        s->xtal_base + timg_cal_xtal_by(s, c->first);

    c->value = extract64(s->xtal_base + timg_cal_xtal_by(s, e) - at_first,
                         0, CAL_VALUE_BITS);
    timg_cal_stop(s);
    c->rdy = true;
    if (c->oneshot) {
        return;
    }
    c->vld = true;
    if (c->cfg & CAL_START_CYCLING) {
        timg_cal_begin_at_clk(s, e);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * The measurement times out at the end of XTAL_CLK's cycle @x. A cycling
 * one begins again TIMG_RTC_CALI_TIMEOUT_RST_CNT cycles later.
 */
static void timg_cal_time_out(ESP32P4TimgState *s, uint64_t x)
{
    ESP32P4TimgCal *c = &s->cal;
    bool cycling = !c->oneshot;

    timg_cal_stop(s);
    c->timeout = true;
    if (cycling && (c->cfg & CAL_START_CYCLING)) {
        c->restart_pending = true;
        c->restart_at = x + CAL2_RST_CNT(c->cfg2);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A cycling measurement has just begun at the end of the measured cycle
 * at which the last one ended. Its (THRES + 1)th XTAL_CLK edge comes
 * before its end, RTC_CALI_MAX + 1 measured cycles on, only if THRES
 * XTAL_CLK cycles last less than those: if they do not, at whatever phase
 * the two clocks' edges have, no measurement times out, and every one
 * ends RTC_CALI_MAX + 1 measured cycles after the last. Those that have
 * ended by measured cycle @now_k are passed over at once, the value of
 * the last of them taken from its edges.
 */
static void timg_cal_skip_ends(ESP32P4TimgState *s, uint64_t now_k)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t span = (uint64_t)c->target + 1;
    uint64_t begin = c->first - 1, e;

    if (now_k - begin < span ||
        esp32p4_clk_cycles_cmp(s->cal_edges.edges.period, span,
                               s->xtal_edges.edges.period,
                               timg_cal_limit(c) - 1) > 0) {
        return;
    }
    e = begin + (now_k - begin) / span * span;
    c->value = extract64(timg_cal_xtal_by(s, e) -
                         timg_cal_xtal_by(s, e - c->target),
                         0, CAL_VALUE_BITS);
    timg_cal_begin_at_clk(s, e);
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A cycling measurement has just begun at an XTAL_CLK edge, after a
 * timeout. Its end comes RTC_CALI_MAX measured cycles after the measured
 * clock's first edge, by its (THRES + 1)th XTAL_CLK edge only if those
 * last less than THRES + 1 XTAL_CLK cycles: if they do not, or the
 * measured clock is stopped, no measurement ends, and every one times out
 * THRES + 1 XTAL_CLK cycles after it begins and begins again
 * TIMG_RTC_CALI_TIMEOUT_RST_CNT cycles later. Those that have begun again
 * by XTAL_CLK's cycle @now_x are passed over at once.
 */
static void timg_cal_skip_timeouts(ESP32P4TimgState *s, uint64_t now_x)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t limit = timg_cal_limit(c);
    uint64_t period = limit + CAL2_RST_CNT(c->cfg2);
    uint64_t begin = c->xtal_first;

    if (now_x - begin < period ||
        (timg_cal_clk_runs(s) &&
         esp32p4_clk_cycles_cmp(s->cal_edges.edges.period, c->target,
                                s->xtal_edges.edges.period, limit) < 0)) {
        return;
    }
    timg_cal_begin_at_xtal(s, begin + (now_x - begin) / period * period);
}

/*
 * Between those bounds, with THRES within a cycle of either clock of the
 * measurement's length, whether a cycling measurement ends or times out
 * depends on where it begins among the two clocks' edges. Each begins at
 * an edge of one of them, the last one's end or its timeout's restart,
 * and two that begin at an edge of the same clock, a whole number of both
 * clocks' periods apart, begin the same sequence of measurements: the
 * clocks' periods are in a ratio of whole numbers, so the beginnings come
 * round again. Brent's cycle finding (R. P. Brent, "An improved Monte
 * Carlo factorization algorithm", BIT 20, 1980) finds the repeat as the
 * measurements are stepped through, and the whole repeats that have run
 * by then are passed over at once: the values, CYCLING_DATA_VLD and
 * TIMEOUT that each leaves are those the first one left.
 */
typedef struct TimgCalCycle {
    /* The marked beginning: at a measured clock edge, or XTAL_CLK's. */
    bool at_clk;
    /* Both clocks' cycles by then. */
    uint64_t k;
    uint64_t x;
    uint64_t power;
    uint64_t len;
    bool done;
} TimgCalCycle;

static void timg_cal_cycle_step(ESP32P4TimgState *s, TimgCalCycle *cy,
                                bool at_clk, uint64_t now_k, uint64_t now_x)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t k = c->first - 1, x = c->xtal_first, dk, dx;

    if (cy->done) {
        return;
    }
    if (cy->len && cy->at_clk == at_clk) {
        dk = k - cy->k;
        dx = x - cy->x;
        if (esp32p4_clk_cycles_cmp(s->cal_edges.edges.period, dk,
                                   s->xtal_edges.edges.period, dx) == 0) {
            if (at_clk) {
                timg_cal_begin_at_clk(s, k + (now_k - k) / dk * dk);
            } else {
                timg_cal_begin_at_xtal(s, x + (now_x - x) / dx * dx);
            }
            cy->done = true;
            return;
        }
    }
    if (!cy->len || cy->len == cy->power) {
        if (cy->len) {
            cy->power *= 2;
        }
        cy->at_clk = at_clk;
        cy->k = k;
        cy->x = x;
        cy->len = 0;
    }
    cy->len++;
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * [spec:nuos:req:emu.esp32p4.clocks]
 * Bring the unit up to @now: every end and timeout by then, in order,
 * each at the exact edge it comes at, of the clocks as they have run
 * since they last changed. Neither counter counts without XTAL_CLK.
 */
static void timg_cal_advance(ESP32P4TimgState *s, int64_t now)
{
    ESP32P4TimgCal *c = &s->cal;
    TimgCalCycle cy = { .power = 1 };
    uint64_t now_k, now_x, limit, x, e;

    if (!timg_cal_xtal_runs(s)) {
        return;
    }
    now_k = esp32p4_clk_count_at(&s->cal_edges, now);
    now_x = esp32p4_clk_count_at(&s->xtal_edges, now);
    limit = timg_cal_limit(c);
    for (;;) {
        if (!c->active) {
            if (!c->restart_pending || c->restart_at > now_x) {
                return;
            }
            timg_cal_begin_at_xtal(s, c->restart_at);
            timg_cal_skip_timeouts(s, now_x);
            timg_cal_cycle_step(s, &cy, false, now_k, now_x);
            continue;
        }
        x = c->xtal_first + (c->since_start < limit ?
                             limit - c->since_start : 0);
        e = c->first + c->target - c->cycles;
        if (c->since_start < limit && timg_cal_clk_runs(s) &&
            timg_cal_clk_by(s, x) >= e) {
            if (e > now_k) {
                return;
            }
            timg_cal_end(s, e);
            if (c->active) {
                timg_cal_skip_ends(s, now_k);
                timg_cal_cycle_step(s, &cy, true, now_k, now_x);
            }
        } else {
            if (x > now_x) {
                return;
            }
            timg_cal_time_out(s, x);
        }
    }
}

/*
 * Ahead of a change of either clock at @now, with the unit up to then:
 * count the measurement's progress in the clocks' cycles so far, the
 * measured clock's in @cycles and XTAL_CLK's in @since_start, and the
 * XTAL_CLK cycles a restart still waits.
 */
static void timg_cal_fold(ESP32P4TimgState *s, int64_t now)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t now_k = esp32p4_clk_count_at(&s->cal_edges, now);
    uint64_t now_x = esp32p4_clk_count_at(&s->xtal_edges, now);

    if (c->restart_pending) {
        c->restart_at -= now_x;
    }
    if (!c->active || !timg_cal_xtal_runs(s)) {
        return;
    }
    if (timg_cal_clk_runs(s)) {
        if (!c->counting && c->first <= now_k) {
            c->counting = true;
            c->xtal_at_first = s->xtal_base +
                               timg_cal_xtal_by(s, c->first);
        }
        if (c->counting) {
            c->cycles += now_k - c->first;
        }
    }
    c->since_start += now_x - c->xtal_first;
}

/*
 * After the change: the measurement goes on from @now at the clocks' new
 * edges, a measurement waiting for its first edge taking the next one.
 */
static void timg_cal_rebase(ESP32P4TimgState *s, int64_t now)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t now_k = esp32p4_clk_count_at(&s->cal_edges, now);
    uint64_t now_x = esp32p4_clk_count_at(&s->xtal_edges, now);

    if (c->restart_pending) {
        c->restart_at += now_x;
    }
    c->first = c->counting ? now_k : now_k + 1;
    c->xtal_first = now_x;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_cal_write(ESP32P4TimgState *s, uint32_t value, int64_t now)
{
    ESP32P4TimgCal *c = &s->cal;
    uint32_t old = c->cfg;

    c->cfg = value & CAL_RW;
    if (c->cfg & ~old & CAL_START) {
        timg_cal_begin(s, true, true, now);
    } else if (old & ~c->cfg & CAL_START && c->active && c->oneshot) {
        timg_cal_stop(s);
    }
    if (c->cfg & ~old & CAL_START_CYCLING) {
        c->vld = false;
        if (!c->active) {
            timg_cal_begin(s, false, true, now);
        }
    } else if (old & ~c->cfg & CAL_START_CYCLING && !c->oneshot) {
        timg_cal_stop(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * A threshold lowered to below the XTAL_CLK cycles a measurement has run
 * times it out at once.
 */
static void timg_cal_write2(ESP32P4TimgState *s, uint32_t value, int64_t now)
{
    ESP32P4TimgCal *c = &s->cal;
    uint64_t now_x;

    c->cfg2 = value & CAL2_RW;
    if (!c->active || !timg_cal_xtal_runs(s)) {
        return;
    }
    now_x = esp32p4_clk_count_at(&s->xtal_edges, now);
    if (c->since_start + (now_x - c->xtal_first) >= timg_cal_limit(c)) {
        timg_cal_time_out(s, now_x);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A clock of the calibration is about to change rate, or has: the
 * measurement is counted up to the change at the old edges and goes on at
 * the new.
 */
static void timg_cal_clock_update(ESP32P4TimgState *s, ClockEvent event,
                                  bool xtal)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        timg_cal_advance(s, now);
        timg_cal_fold(s, now);
        if (xtal) {
            s->xtal_base += esp32p4_clk_count_at(&s->xtal_edges, now);
        }
        return;
    }
    if (xtal) {
        esp32p4_clk_count_start(&s->xtal_edges, s->xtal, now);
    } else {
        esp32p4_clk_count_start(&s->cal_edges, s->cal_clk, now);
    }
    timg_cal_rebase(s, now);
}

static void timg_xtal_update(void *opaque, ClockEvent event)
{
    timg_cal_clock_update(opaque, event, true);
}

static void timg_cal_clk_update(void *opaque, ClockEvent event)
{
    timg_cal_clock_update(opaque, event, false);
}

/* Registers ----------------------------------------------------------- */

/* Run every event that fell due by @now; false if it reset the block. */
static bool timg_catch_up(ESP32P4TimgState *s, int64_t now)
{
    uint32_t resets = s->reset_count;
    int x;

    for (x = 0; x < ESP32P4_TIMG_TIMERS; x++) {
        timg_timer_catch_up(s, x, now);
    }
    timg_wdt_catch_up(s, now);
    return s->reset_count == resets;
}

/* Bring the calibration unit up to @now for an access to its registers. */
static void timg_cal_access(ESP32P4TimgState *s, hwaddr addr, int64_t now)
{
    if (addr == A_RTCCALICFG || addr == A_RTCCALICFG1 ||
        addr == A_RTCCALICFG2) {
        timg_cal_advance(s, now);
    }
}

/* The register at word offset @addr; false for a reserved offset. */
static bool timg_read_reg(ESP32P4TimgState *s, hwaddr addr, uint32_t *v)
{
    ESP32P4TimgTimer *t;

    if (addr < TIMER_REGS_END) {
        t = &s->t[addr / 0x24];
        switch (addr % 0x24) {
        case R_TCONFIG:
            *v = t->config;
            return true;
        case R_TLO:
            *v = t->latched;
            return true;
        case R_THI:
            *v = t->latched >> 32;
            return true;
        case R_TALARMLO:
            *v = t->alarm;
            return true;
        case R_TALARMHI:
            *v = t->alarm >> 32;
            return true;
        case R_TLOADLO:
            *v = t->load;
            return true;
        case R_TLOADHI:
            *v = t->load >> 32;
            return true;
        default:
            /* UPDATE completes at once; LOAD is write-only. */
            *v = 0;
            return true;
        }
    }
    switch (addr) {
    case A_WDTCONFIG0:
        *v = s->wdt.config0;
        return true;
    case A_WDTCONFIG1:
        *v = s->wdt.config1;
        return true;
    case A_WDTCONFIG2 ... A_WDTCONFIG5:
        *v = s->wdt.hold[(addr - A_WDTCONFIG2) / 4];
        return true;
    case A_WDTFEED:
    case A_INT_CLR:
        *v = 0;
        return true;
    case A_WDTWPROTECT:
        *v = s->wdt.wkey;
        return true;
    case A_RTCCALICFG:
        *v = s->cal.cfg | (s->cal.rdy ? CAL_RDY : 0);
        return true;
    case A_RTCCALICFG1:
        *v = (s->cal.value << CAL1_VALUE_SHIFT) | (s->cal.vld ? CAL1_VLD : 0);
        return true;
    case A_RTCCALICFG2:
        *v = s->cal.cfg2 | (s->cal.timeout ? CAL2_TIMEOUT : 0);
        return true;
    case A_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    case A_REGCLK:
        *v = s->regclk;
        return true;
    default:
        *v = 0;
        return false;
    }
}

static uint64_t timg_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4TimgState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v;

    timg_catch_up(s, now);
    timg_cal_access(s, addr & ~3, now);
    if (!timg_read_reg(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s%u: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TIMG, s->group, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_write_timer(ESP32P4TimgState *s, int x, hwaddr reg,
                             uint32_t value, int64_t now)
{
    ESP32P4TimgTimer *t = &s->t[x];

    switch (reg) {
    case R_TCONFIG:
        timg_timer_write_config(s, x, value, now);
        break;
    case R_TUPDATE:
        t->latched = timg_timer_count(t, now);
        break;
    case R_TALARMLO:
        t->alarm = deposit64(t->alarm, 0, 32, value);
        timg_timer_sync(t, now);
        timg_timer_check(s, x, now);
        break;
    case R_TALARMHI:
        t->alarm = deposit64(t->alarm, 32, 32, value & TIMER_HI_MASK);
        timg_timer_sync(t, now);
        timg_timer_check(s, x, now);
        break;
    case R_TLOADLO:
        t->load = deposit64(t->load, 0, 32, value);
        break;
    case R_TLOADHI:
        t->load = deposit64(t->load, 32, 32, value & TIMER_HI_MASK);
        break;
    case R_TLOAD:
        timg_timer_sync(t, now);
        t->count = t->load;
        timg_timer_check(s, x, now);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s%u: write to timer %d's read-only "
                      "register at 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_TIMG,
                      s->group, x, reg);
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void timg_write_wdt(ESP32P4TimgState *s, hwaddr addr, uint32_t value,
                           int64_t now)
{
    ESP32P4TimgWdt *w = &s->wdt;
    int i;

    if (w->wkey != WDT_WKEY) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s%u: write to MWDT register 0x%"
                      HWADDR_PRIx " while write-protected\n",
                      TYPE_ESP32P4_TIMG, s->group, addr);
        return;
    }
    timg_wdt_sync(w, now);
    switch (addr) {
    case A_WDTCONFIG0:
        w->config0 = value & WCONF0_RW;
        if (value & WCONF0_CONF_UPDATE_EN) {
            w->act_config0 = w->config0;
            w->act_prescale = WCONF1_PRESCALE(w->config1);
            for (i = 0; i < ESP32P4_TIMG_WDT_STAGES; i++) {
                w->act_hold[i] = w->hold[i];
            }
            if (w->pre >= timg_wdt_prescale(w)) {
                w->pre = 0;
            }
        }
        break;
    case A_WDTCONFIG1:
        w->config1 = value & WCONF1_RW;
        if (value & WCONF1_DIVCNT_RST) {
            w->pre = 0;
        }
        break;
    case A_WDTCONFIG2 ... A_WDTCONFIG5:
        w->hold[(addr - A_WDTCONFIG2) / 4] = value;
        break;
    case A_WDTFEED:
        w->stage = 0;
        w->count = 0;
        break;
    }
    timg_wdt_update(s, now);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Write the register at word offset @addr: @value is its new value, the
 * written byte lanes merged into the old, for its R/W fields; @ones has
 * the bits written as 1, for its write-trigger and write-1-to-clear ones.
 */
static void timg_write_reg(ESP32P4TimgState *s, hwaddr addr, uint32_t value,
                           uint32_t ones, int64_t now)
{
    if (addr < TIMER_REGS_END) {
        hwaddr reg = addr % 0x24;

        timg_write_timer(s, addr / 0x24, reg, value, now);
        return;
    }
    switch (addr) {
    case A_WDTCONFIG0 ... A_WDTFEED:
        timg_write_wdt(s, addr, value, now);
        break;
    case A_WDTWPROTECT:
        s->wdt.wkey = value;
        break;
    case A_RTCCALICFG:
        timg_cal_write(s, value, now);
        break;
    case A_RTCCALICFG2:
        timg_cal_write2(s, value, now);
        break;
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        timg_update_irqs(s);
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(ones & INT_MASK);
        timg_update_irqs(s);
        break;
    case A_DATE:
        s->date = value & DATE_MASK;
        break;
    case A_REGCLK:
        s->regclk = value & REGCLK_RW;
        break;
    case A_RTCCALICFG1:
    case A_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s%u: write to read-only register "
                      "at 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_TIMG, s->group,
                      addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s%u: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TIMG, s->group, addr);
        break;
    }
}

static void timg_write(void *opaque, hwaddr addr, uint64_t data,
                       unsigned size)
{
    ESP32P4TimgState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t ones = (data << shift) & lanes;
    uint32_t old;

    if (!timg_catch_up(s, now)) {
        return;
    }
    timg_cal_access(s, addr & ~3, now);
    timg_read_reg(s, addr & ~3, &old);
    timg_write_reg(s, addr & ~3, (old & ~lanes) | ones, ones, now);
}

static const MemoryRegionOps timg_ops = {
    .read = timg_read,
    .write = timg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * An ETM task acts at its event's time, which its pulse carries: two
 * tasks of one event start, or capture, at the same count.
 */
static void timg_etm_task(void *opaque, int line, int level)
{
    ESP32P4TimgState *s = opaque;
    int64_t now = esp32_line_time();
    int x = line / ESP32P4_TIMG_TASKS;
    ESP32P4TimgTimer *t = &s->t[x];

    if (!level || !(s->regclk & REGCLK_ETM_EN) || !timg_catch_up(s, now)) {
        return;
    }
    timg_timer_sync(t, now);
    switch (line % ESP32P4_TIMG_TASKS) {
    case ESP32P4_TIMG_TASK_CNT_START:
        t->config |= TCONF_EN;
        break;
    case ESP32P4_TIMG_TASK_ALARM_START:
        t->config |= TCONF_ALARM_EN;
        break;
    case ESP32P4_TIMG_TASK_CNT_STOP:
        t->config &= ~TCONF_EN;
        break;
    case ESP32P4_TIMG_TASK_CNT_RELOAD:
        t->count = t->load;
        break;
    case ESP32P4_TIMG_TASK_CNT_CAP:
        t->latched = t->count;
        break;
    }
    timg_timer_update(s, x, now);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void esp32p4_timg_reset_hold(Object *obj, ResetType type)
{
    ESP32P4TimgState *s = ESP32P4_TIMG(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ESP32P4TimgWdt *w = &s->wdt;
    int i;

    s->reset_count++;
    for (i = 0; i < ESP32P4_TIMG_TIMERS; i++) {
        ESP32P4TimgTimer *t = &s->t[i];

        timer_del(&t->timer);
        t->config = TCONF_RESET;
        t->alarm = 0;
        t->load = 0;
        t->latched = 0;
        t->count = 0;
        t->pre = 0;
        esp32p4_clk_count_start(&t->edges, t->clk, now);
        t->synced = 0;
        t->running = false;
    }
    timer_del(&w->timer);
    w->config0 = WCONF0_RESET;
    w->config1 = WCONF1_RESET;
    w->wkey = WDT_WKEY;
    w->act_config0 = WCONF0_RESET;
    w->act_prescale = WCONF1_PRESCALE(WCONF1_RESET);
    for (i = 0; i < ESP32P4_TIMG_WDT_STAGES; i++) {
        w->hold[i] = wdt_hold_reset[i];
        w->act_hold[i] = wdt_hold_reset[i];
    }
    w->stage = 0;
    w->count = 0;
    w->pre = 0;
    esp32p4_clk_count_start(&w->edges, w->clk, now);
    w->synced = 0;
    w->running = false;

    timg_cal_stop(s);
    esp32p4_clk_count_start(&s->xtal_edges, s->xtal, now);
    esp32p4_clk_count_start(&s->cal_edges, s->cal_clk, now);
    s->xtal_base = 0;
    s->cal.cfg = CAL_RESET;
    s->cal.cfg2 = CAL2_RESET;
    s->cal.rdy = false;
    s->cal.vld = false;
    s->cal.timeout = false;
    s->cal.value = 0;

    s->int_ena = 0;
    s->int_raw = 0;
    s->date = DATE_RESET;
    s->regclk = REGCLK_RESET;
}

/*
 * The timers, the watchdog (which runs on its own for flash boot
 * protection) and the cycling calibration (on at reset) start from their
 * reset state.
 */
static void esp32p4_timg_reset_exit(Object *obj, ResetType type)
{
    ESP32P4TimgState *s = ESP32P4_TIMG(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    for (i = 0; i < ESP32P4_TIMG_TIMERS; i++) {
        timg_timer_update(s, i, now);
    }
    timg_wdt_update(s, now);
    timg_cal_begin(s, false, true, now);
    timg_update_irqs(s);
}

static void esp32p4_timg_init(Object *obj)
{
    ESP32P4TimgState *s = ESP32P4_TIMG(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &timg_ops, s, "esp32p4.timg",
                          ESP32P4_TIMG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->t[0].clk = qdev_init_clock_in(dev, "t0", timg_t0_clk_update, s,
                                     ClockPreUpdate | ClockUpdate);
    s->t[1].clk = qdev_init_clock_in(dev, "t1", timg_t1_clk_update, s,
                                     ClockPreUpdate | ClockUpdate);
    s->wdt.clk = qdev_init_clock_in(dev, "wdt", timg_wdt_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->xtal = qdev_init_clock_in(dev, "xtal", timg_xtal_update, s,
                                 ClockPreUpdate | ClockUpdate);
    s->cal_clk = qdev_init_clock_in(dev, "cal", timg_cal_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    for (i = 0; i < ESP32P4_TIMG_TIMERS; i++) {
        s->t[i].s = s;
        timer_init_ns(&s->t[i].timer, QEMU_CLOCK_VIRTUAL, timg_timer_cb,
                      &s->t[i]);
    }
    timer_init_ns(&s->wdt.timer, QEMU_CLOCK_VIRTUAL, timg_wdt_cb, s);
    for (i = 0; i <= ESP32P4_TIMG_TIMERS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_in_named(dev, timg_set_spi_boot, ESP32P4_TIMG_SPI_BOOT, 1);
    qdev_init_gpio_in_named(dev, timg_etm_task, ESP32P4_TIMG_ETM_TASK,
                            ESP32P4_TIMG_TIMERS * ESP32P4_TIMG_TASKS);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_TIMG_ETM_EVENT,
                             ESP32P4_TIMG_TIMERS);
    qdev_init_gpio_out_named(dev, s->cpu_reset, ESP32P4_TIMG_CPU_RESET,
                             ESP32P4_CLKRST_HP_CORES);
}

static void esp32p4_timg_realize(DeviceState *dev, Error **errp)
{
    ESP32P4TimgState *s = ESP32P4_TIMG(dev);

    if (!s->clkrst) {
        error_setg(errp, "%s: the \"clkrst\" link is not set",
                   TYPE_ESP32P4_TIMG);
        return;
    }
    if (s->group >= ESP32P4_CLKRST_MWDTS) {
        error_setg(errp, "%s: \"group\" must be 0 or 1", TYPE_ESP32P4_TIMG);
    }
}

static const VMStateDescription vmstate_esp32p4_timg_timer = {
    .name = TYPE_ESP32P4_TIMG "/timer",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4TimgTimer),
        VMSTATE_TIMER(timer, ESP32P4TimgTimer),
        VMSTATE_UINT32(config, ESP32P4TimgTimer),
        VMSTATE_UINT64(alarm, ESP32P4TimgTimer),
        VMSTATE_UINT64(load, ESP32P4TimgTimer),
        VMSTATE_UINT64(latched, ESP32P4TimgTimer),
        VMSTATE_UINT64(count, ESP32P4TimgTimer),
        VMSTATE_UINT32(pre, ESP32P4TimgTimer),
        VMSTATE_ESP32P4_CLK_COUNT(edges, ESP32P4TimgTimer),
        VMSTATE_UINT64(synced, ESP32P4TimgTimer),
        VMSTATE_BOOL(running, ESP32P4TimgTimer),
        VMSTATE_INT64(due_ns, ESP32P4TimgTimer),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_timg_wdt = {
    .name = TYPE_ESP32P4_TIMG "/wdt",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4TimgWdt),
        VMSTATE_TIMER(timer, ESP32P4TimgWdt),
        VMSTATE_UINT32(config0, ESP32P4TimgWdt),
        VMSTATE_UINT32(config1, ESP32P4TimgWdt),
        VMSTATE_UINT32_ARRAY(hold, ESP32P4TimgWdt, ESP32P4_TIMG_WDT_STAGES),
        VMSTATE_UINT32(wkey, ESP32P4TimgWdt),
        VMSTATE_UINT32(act_config0, ESP32P4TimgWdt),
        VMSTATE_UINT32(act_prescale, ESP32P4TimgWdt),
        VMSTATE_UINT32_ARRAY(act_hold, ESP32P4TimgWdt,
                             ESP32P4_TIMG_WDT_STAGES),
        VMSTATE_UINT32(stage, ESP32P4TimgWdt),
        VMSTATE_UINT32(count, ESP32P4TimgWdt),
        VMSTATE_UINT32(pre, ESP32P4TimgWdt),
        VMSTATE_ESP32P4_CLK_COUNT(edges, ESP32P4TimgWdt),
        VMSTATE_UINT64(synced, ESP32P4TimgWdt),
        VMSTATE_BOOL(running, ESP32P4TimgWdt),
        VMSTATE_INT64(due_ns, ESP32P4TimgWdt),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_timg_cal = {
    .name = TYPE_ESP32P4_TIMG "/cal",
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, ESP32P4TimgCal),
        VMSTATE_UINT32(cfg2, ESP32P4TimgCal),
        VMSTATE_BOOL(rdy, ESP32P4TimgCal),
        VMSTATE_BOOL(vld, ESP32P4TimgCal),
        VMSTATE_BOOL(timeout, ESP32P4TimgCal),
        VMSTATE_UINT32(value, ESP32P4TimgCal),
        VMSTATE_BOOL(active, ESP32P4TimgCal),
        VMSTATE_BOOL(oneshot, ESP32P4TimgCal),
        VMSTATE_BOOL(counting, ESP32P4TimgCal),
        VMSTATE_UINT32(target, ESP32P4TimgCal),
        VMSTATE_UINT64(cycles, ESP32P4TimgCal),
        VMSTATE_UINT64(xtal_at_first, ESP32P4TimgCal),
        VMSTATE_UINT64(first, ESP32P4TimgCal),
        VMSTATE_UINT64(since_start, ESP32P4TimgCal),
        VMSTATE_UINT64(xtal_first, ESP32P4TimgCal),
        VMSTATE_BOOL(restart_pending, ESP32P4TimgCal),
        VMSTATE_UINT64(restart_at, ESP32P4TimgCal),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_timg = {
    .name = TYPE_ESP32P4_TIMG,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(t, ESP32P4TimgState, ESP32P4_TIMG_TIMERS, 2,
                             vmstate_esp32p4_timg_timer, ESP32P4TimgTimer),
        VMSTATE_STRUCT(wdt, ESP32P4TimgState, 2, vmstate_esp32p4_timg_wdt,
                       ESP32P4TimgWdt),
        VMSTATE_STRUCT(cal, ESP32P4TimgState, 4, vmstate_esp32p4_timg_cal,
                       ESP32P4TimgCal),
        VMSTATE_CLOCK(xtal, ESP32P4TimgState),
        VMSTATE_CLOCK(cal_clk, ESP32P4TimgState),
        VMSTATE_ESP32P4_CLK_COUNT(xtal_edges, ESP32P4TimgState),
        VMSTATE_ESP32P4_CLK_COUNT(cal_edges, ESP32P4TimgState),
        VMSTATE_UINT64(xtal_base, ESP32P4TimgState),
        VMSTATE_UINT32(int_ena, ESP32P4TimgState),
        VMSTATE_UINT32(int_raw, ESP32P4TimgState),
        VMSTATE_UINT32(date, ESP32P4TimgState),
        VMSTATE_UINT32(regclk, ESP32P4TimgState),
        VMSTATE_BOOL(spi_boot, ESP32P4TimgState),
        VMSTATE_UINT32(reset_count, ESP32P4TimgState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_timg_properties[] = {
    DEFINE_PROP_LINK("clkrst", ESP32P4TimgState, clkrst, TYPE_ESP32P4_CLKRST,
                     ESP32P4ClkRstState *),
    DEFINE_PROP_UINT32("group", ESP32P4TimgState, group, 0),
};

static void esp32p4_timg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_timg_realize;
    dc->vmsd = &vmstate_esp32p4_timg;
    device_class_set_props(dc, esp32p4_timg_properties);
    rc->phases.hold = esp32p4_timg_reset_hold;
    rc->phases.exit = esp32p4_timg_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static const TypeInfo esp32p4_timg_info = {
    .name          = TYPE_ESP32P4_TIMG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4TimgState),
    .instance_init = esp32p4_timg_init,
    .class_init    = esp32p4_timg_class_init,
};

static void esp32p4_timg_register_types(void)
{
    type_register_static(&esp32p4_timg_info);
}
type_init(esp32p4_timg_register_types)
