/*
 * ESP32 RTC_CNTL: low-power management, the RTC timer and watchdog, and the
 * RTC fast and slow memories
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * Reference: ESP32 TRM, "Low-Power Management (RTC_CNTL)" and "Watchdog
 * Timers"; ESP-IDF's soc/rtc_cntl_reg.h, rom/rtc.h and esp_hw_support's
 * rtc_sleep.c, which is how software drives the sleep modes.
 *
 * RTC_CNTL is in the RTC domain. A reset of the digital domain (a software
 * system reset, a timer-group watchdog's system reset, or the power-down of
 * deep sleep) leaves it, the RTC memories and the RTC timer as they are;
 * only a power-on, the RTC watchdog's RTC reset and the brownout reset
 * reset its registers, and only a power-on resets the RTC timer (TRM
 * 9.3.12).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/crc32.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/gpio/esp32_rtcio.h"
#include "migration/vmstate.h"

#define TIME_MASK MAKE_64BIT_MASK(0, 48)

/* SW_STALL_{PRO,APP}CPU_C1 << 2 | _C0 that stalls a CPU */
#define SW_STALL_MAGIC 0x86

/* RTC watchdog stage actions, RTC_CNTL_WDT_STGn */
enum {
    WDT_STG_OFF,
    WDT_STG_INT,
    WDT_STG_CPU_RESET,
    WDT_STG_SYS_RESET,
    WDT_STG_RTC_RESET,
};

/* RTC_CNTL_REJECT_CAUSE values (TRM register 9.15) */
#define REJECT_CAUSE_GPIO 2
#define REJECT_CAUSE_SDIO 3

/* Wakeup sources that can end a deep sleep (TRM table 9.3-2) */
#define WAKEUP_DEEP_SLEEP_MASK  (BIT(ESP32_RTC_WAKEUP_EXT0) | \
                                 BIT(ESP32_RTC_WAKEUP_EXT1) | \
                                 BIT(ESP32_RTC_WAKEUP_GPIO) | \
                                 BIT(ESP32_RTC_WAKEUP_TIMER) | \
                                 BIT(ESP32_RTC_WAKEUP_TOUCH) | \
                                 BIT(ESP32_RTC_WAKEUP_ULP))
/* Wakeup sources driven by blocks that are not modelled */
#define WAKEUP_UNMODELLED_MASK  (BIT(ESP32_RTC_WAKEUP_SDIO) | \
                                 BIT(ESP32_RTC_WAKEUP_WIFI) | \
                                 BIT(ESP32_RTC_WAKEUP_UART0) | \
                                 BIT(ESP32_RTC_WAKEUP_UART1) | \
                                 BIT(ESP32_RTC_WAKEUP_BT))

/*
 * What a powered-down RTC memory holds when it comes back. Its contents are
 * lost; the pattern makes software that relies on them fail visibly.
 */
#define LOST_MEM_PATTERN 0xa5

typedef struct RegInfo {
    uint32_t reset;
    uint32_t writable;
} RegInfo;

#define R(reg) [A_##reg / 4]

/*
 * Reset values and software-writable bits (ESP-IDF's rtc_cntl_reg.h
 * "default" and access annotations). Registers with side effects or
 * read-only fields are handled in the access functions.
 */
static const RegInfo reg_info[ESP32_RTC_CNTL_REG_COUNT] = {
    R(RTC_CNTL_OPTIONS0) = { 0x1c492000, 0x7fffffcf },
    R(RTC_CNTL_SLP_TIMER0) = { 0, 0xffffffff },
    R(RTC_CNTL_SLP_TIMER1) = { 0, 0x0001ffff },
    R(RTC_CNTL_STATE0) = { 0x00300000, 0xe1f00000 },
    R(RTC_CNTL_TIMER1) = { 0x28140403, 0xffffffff },
    R(RTC_CNTL_TIMER2) = { 0x01080000, 0xffff8000 },
    R(RTC_CNTL_TIMER3) = { 0x14160a08, 0xffffffff },
    R(RTC_CNTL_TIMER4) = { 0x10200a08, 0xffffffff },
    R(RTC_CNTL_TIMER5) = { 0x12148001, 0xffffffff },
    R(RTC_CNTL_ANA_CONF) = { 0x00800000, 0xdf800000 },
    R(RTC_CNTL_RESET_STATE) = { 0x00003000, 0x00003000 },
    R(RTC_CNTL_WAKEUP_STATE) = { 0x00006000, 0x007ff800 },
    R(RTC_CNTL_INT_ENA) = { 0, 0x000001ff },
    R(RTC_CNTL_STORE0) = { 0, 0xffffffff },
    R(RTC_CNTL_STORE1) = { 0, 0xffffffff },
    R(RTC_CNTL_STORE2) = { 0, 0xffffffff },
    R(RTC_CNTL_STORE3) = { 0, 0xffffffff },
    R(RTC_CNTL_EXT_XTL_CONF) = { 0, 0xc0000000 },
    R(RTC_CNTL_EXT_WAKEUP_CONF) = { 0, 0xc0000000 },
    R(RTC_CNTL_SLP_REJECT_CONF) = { 0, 0x0f000000 },
    R(RTC_CNTL_CPU_PERIOD_CONF) = { 0, 0xe0000000 },
    R(RTC_CNTL_SDIO_ACT_CONF) = { 0, 0xffc00000 },
    R(RTC_CNTL_CLK_CONF) = { ESP32_RTC_CNTL_CLK_CONF_RESET, 0xfffffff0 },
    R(RTC_CNTL_SDIO_CONF) = { 0x02a00000, 0xfee00000 },
    R(RTC_CNTL_BIAS_CONF) = { 0, 0xff000000 },
    R(RTC_CNTL_VREG) = { 0xa9002400, 0xffffff80 },
    R(RTC_CNTL_PWC) = { 0x00012925, 0x001fffff },
    R(RTC_CNTL_DIG_PWC) = { 0x00155550, 0xff1ffff8 },
    R(RTC_CNTL_DIG_ISO) = { 0xaaaa5000, 0xfffff980 },
    R(RTC_CNTL_WDTCONFIG0) = { 0x00004c80, 0xffffff80 },
    R(RTC_CNTL_WDTCONFIG1) = { 128000, 0xffffffff },
    R(RTC_CNTL_WDTCONFIG2) = { 80000, 0xffffffff },
    R(RTC_CNTL_WDTCONFIG3) = { 0xfff, 0xffffffff },
    R(RTC_CNTL_WDTCONFIG4) = { 0xfff, 0xffffffff },
    R(RTC_CNTL_WDTWPROTECT) = { ESP32_RTC_WDT_WKEY, 0xffffffff },
    R(RTC_CNTL_TEST_MUX) = { 0, 0xe0000000 },
    R(RTC_CNTL_SW_CPU_STALL) = { 0, 0xfff00000 },
    R(RTC_CNTL_STORE4) = { 0, 0xffffffff },
    R(RTC_CNTL_STORE5) = { 0, 0xffffffff },
    R(RTC_CNTL_STORE6) = { 0, 0xffffffff },
    R(RTC_CNTL_STORE7) = { 0, 0xffffffff },
    R(RTC_CNTL_HOLD_FORCE) = { 0, 0x0003ffff },
    R(RTC_CNTL_EXT_WAKEUP1) = { 0, 0x0003ffff },
    R(RTC_CNTL_BROWN_OUT) = { 0x13ff0000, 0x7fffc000 },
    R(RTC_MEM_CONF) = { 0, 0x7fffffff },
    R(RTC_CNTL_DATE) = { 0x01604280, 0x0fffffff },
};

#define REG(s, name) ((s)->regs[A_##name / 4])

static void esp32_rtc_update_irq(Esp32RtcCntlState *s);
static void esp32_rtc_check_wakeup(Esp32RtcCntlState *s);

/* RTC_SLOW_CLK cycles since power-on, modulo 2^48 */
static uint64_t esp32_rtc_time(Esp32RtcCntlState *s, int64_t now)
{
    return (s->time_base + muldiv64(now - s->time_base_ns,
                                    s->rtc_slowclk_freq,
                                    NANOSECONDS_PER_SECOND)) & TIME_MASK;
}

/*
 * Virtual time at which the RTC timer reads ticks more than at base_ns.
 * Only meaningful while RTC_SLOW_CLK runs.
 */
static int64_t esp32_rtc_ticks_to_ns(Esp32RtcCntlState *s, int64_t base_ns,
                                     uint64_t ticks)
{
    assert(s->rtc_slowclk_freq);
    return base_ns + muldiv64(ticks, NANOSECONDS_PER_SECOND,
                              s->rtc_slowclk_freq) + 1;
}

static void esp32_rtc_raise(Esp32RtcCntlState *s, Esp32RtcInt n)
{
    REG(s, RTC_CNTL_INT_RAW) |= BIT(n);
    esp32_rtc_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * RTC_CNTL's interrupts reach the interrupt matrix as one level-triggered
 * source, RTC_CORE: INT_ST is INT_RAW masked by INT_ENA.
 */
static void esp32_rtc_update_irq(Esp32RtcCntlState *s)
{
    qemu_set_irq(s->irq,
                 (REG(s, RTC_CNTL_INT_RAW) & REG(s, RTC_CNTL_INT_ENA)) != 0);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * The RTC timer's alarm: MAIN_TIMER_INT fires as the 48-bit timer reaches
 * SLP_TIMER1:SLP_TIMER0 with MAIN_TIMER_ALARM_EN set. The comparison is an
 * equality, so an alarm set at or before the timer's current value does
 * not fire.
 */
static void esp32_rtc_alarm_arm(Esp32RtcCntlState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t t = esp32_rtc_time(s, now);
    uint64_t alarm = REG(s, RTC_CNTL_SLP_TIMER0) |
        ((uint64_t)FIELD_EX32(REG(s, RTC_CNTL_SLP_TIMER1), RTC_CNTL_SLP_TIMER1,
                              SLP_VAL_HI) << 32);

    timer_del(&s->alarm_timer);
    if (!FIELD_EX32(REG(s, RTC_CNTL_SLP_TIMER1), RTC_CNTL_SLP_TIMER1,
                    MAIN_TIMER_ALARM_EN) || alarm <= t ||
        !s->rtc_slowclk_freq) {
        return;
    }
    timer_mod(&s->alarm_timer, esp32_rtc_ticks_to_ns(s, now, alarm - t));
}

static void esp32_rtc_alarm_cb(void *opaque)
{
    Esp32RtcCntlState *s = opaque;

    esp32_rtc_raise(s, ESP32_RTC_INT_MAIN_TIMER);
    if (s->sleep != ESP32_RTC_AWAKE) {
        s->timer_wakeup = true;
        esp32_rtc_check_wakeup(s);
    }
}

/*
 * TIME_UPDATE latches the timer into TIME0/TIME1. TIME_VALID drops and comes
 * back once the latch has crossed into the slow clock's domain, one
 * RTC_SLOW_CLK cycle later, raising TIME_VALID_INT.
 */
static void esp32_rtc_valid_cb(void *opaque)
{
    Esp32RtcCntlState *s = opaque;

    s->valid_pending = false;
    REG(s, RTC_CNTL_TIME_UPDATE) |= R_RTC_CNTL_TIME_UPDATE_VALID_MASK;
    esp32_rtc_raise(s, ESP32_RTC_INT_TIME_VALID);
}

static void esp32_rtc_time_update(Esp32RtcCntlState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->time_latched = esp32_rtc_time(s, now);
    REG(s, RTC_CNTL_TIME_UPDATE) &= ~R_RTC_CNTL_TIME_UPDATE_VALID_MASK;
    s->valid_pending = true;
    timer_del(&s->valid_timer);
    if (s->rtc_slowclk_freq) {
        timer_mod(&s->valid_timer, esp32_rtc_ticks_to_ns(s, now, 1));
    }
}

static bool esp32_rtc_wdt_active(Esp32RtcCntlState *s)
{
    uint32_t cfg = REG(s, RTC_CNTL_WDTCONFIG0);

    return FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, EN) ||
           (FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, FLASHBOOT_MOD_EN) &&
            s->flash_boot_mode);
}

static bool esp32_rtc_wdt_counting(Esp32RtcCntlState *s)
{
    return esp32_rtc_wdt_active(s) &&
           !(s->sleep != ESP32_RTC_AWAKE &&
             FIELD_EX32(REG(s, RTC_CNTL_WDTCONFIG0), RTC_CNTL_WDTCONFIG0,
                        PAUSE_IN_SLP));
}

static uint64_t esp32_rtc_wdt_count(Esp32RtcCntlState *s, int64_t now)
{
    if (!s->wdt_running) {
        return s->wdt_count_base;
    }
    return s->wdt_count_base + muldiv64(now - s->wdt_base_ns,
                                        s->rtc_slowclk_freq,
                                        NANOSECONDS_PER_SECOND);
}

/*
 * The action of the watchdog's current stage. In flash boot mode with the
 * watchdog not otherwise enabled, stage 0 resets the RTC (TRM 11.3.4).
 */
static unsigned esp32_rtc_wdt_action(Esp32RtcCntlState *s)
{
    uint32_t cfg = REG(s, RTC_CNTL_WDTCONFIG0);

    if (!FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, EN) && s->wdt_stage == 0) {
        return WDT_STG_RTC_RESET;
    }
    return extract32(cfg, R_RTC_CNTL_WDTCONFIG0_STG0_SHIFT - 3 * s->wdt_stage,
                     3);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * Bring the RTC watchdog's count up to now and arm its stage timer. It
 * counts RTC_SLOW_CLK cycles while enabled (WDT_EN, or WDT_FLASHBOOT_MOD_EN
 * when the chip boots from flash), pausing in sleep if WDT_PAUSE_IN_SLP.
 */
static void esp32_rtc_wdt_update(Esp32RtcCntlState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t count = esp32_rtc_wdt_count(s, now);
    uint32_t hold;

    s->wdt_count_base = count;
    s->wdt_base_ns = now;
    s->wdt_running = esp32_rtc_wdt_counting(s);
    timer_del(&s->wdt_timer);
    if (!s->wdt_running || !s->rtc_slowclk_freq) {
        return;
    }
    hold = s->regs[A_RTC_CNTL_WDTCONFIG1 / 4 + s->wdt_stage];
    timer_mod(&s->wdt_timer,
              hold > count ? esp32_rtc_ticks_to_ns(s, now, hold - count) :
                             now);
}

static void esp32_rtc_wdt_restart(Esp32RtcCntlState *s)
{
    s->wdt_stage = 0;
    s->wdt_count_base = 0;
    s->wdt_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->wdt_running = false;
    esp32_rtc_wdt_update(s);
}

/* Reset the digital domain, the RTC domain too if rtc, for cause. */
static void esp32_rtc_system_reset(Esp32RtcCntlState *s, Esp32ResetCause cause,
                                   bool rtc)
{
    for (int i = 0; i < ESP32_CPU_COUNT; i++) {
        s->reset_cause[i] = cause;
    }
    if (rtc) {
        s->rtc_reset_pending = true;
        qemu_irq_pulse(s->rtc_reset_req);
    } else {
        qemu_irq_pulse(s->dig_reset_req);
    }
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * A watchdog stage expired: carry out its action and move to the next
 * stage, after the fourth back to the first. The CPU reset resets the CPUs
 * WDT_PROCPU_RESET_EN and WDT_APPCPU_RESET_EN select.
 */
static void esp32_rtc_wdt_cb(void *opaque)
{
    Esp32RtcCntlState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t cfg = REG(s, RTC_CNTL_WDTCONFIG0);
    uint32_t hold = s->regs[A_RTC_CNTL_WDTCONFIG1 / 4 + s->wdt_stage];
    unsigned action;

    if (esp32_rtc_wdt_count(s, now) < hold) {
        esp32_rtc_wdt_update(s);
        return;
    }
    action = esp32_rtc_wdt_action(s);
    s->wdt_stage = (s->wdt_stage + 1) % ESP32_RTC_WDT_STAGES;
    s->wdt_count_base = 0;
    s->wdt_base_ns = now;
    esp32_rtc_wdt_update(s);

    switch (action) {
    case WDT_STG_INT:
        if (FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, LEVEL_INT_EN) ||
            FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, EDGE_INT_EN)) {
            esp32_rtc_raise(s, ESP32_RTC_INT_WDT);
        }
        break;
    case WDT_STG_CPU_RESET:
        if (FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, PROCPU_RESET_EN)) {
            s->reset_cause[0] = ESP32_RTCWDT_CPU_RESET;
            qemu_irq_pulse(s->cpu_reset_req[0]);
        }
        if (FIELD_EX32(cfg, RTC_CNTL_WDTCONFIG0, APPCPU_RESET_EN)) {
            s->reset_cause[1] = ESP32_RTCWDT_CPU_RESET;
            qemu_irq_pulse(s->cpu_reset_req[1]);
        }
        break;
    case WDT_STG_SYS_RESET:
        esp32_rtc_system_reset(s, ESP32_RTCWDT_SYS_RESET, false);
        break;
    case WDT_STG_RTC_RESET:
        esp32_rtc_system_reset(s, ESP32_RTCWDT_RTC_RESET, true);
        break;
    case WDT_STG_OFF:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_cntl: reserved RTC watchdog stage action "
                      "%u\n", action);
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * The brownout detector: RTC_CNTL_BROWN_OUT_DET follows the supply being
 * below the threshold; with BROWN_OUT_ENA a drop raises BROWN_OUT_INT, and
 * with BROWN_OUT_RST_ENA too the chip resets after BROWN_OUT_RST_WAIT
 * RTC_SLOW_CLK cycles if the supply has not recovered.
 */
static void esp32_rtc_brownout_cb(void *opaque)
{
    Esp32RtcCntlState *s = opaque;
    uint32_t bo = REG(s, RTC_CNTL_BROWN_OUT);

    if (s->brownout_line && FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, ENA) &&
        FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, RST_ENA)) {
        esp32_rtc_system_reset(s, ESP32_RTCWDT_BROWN_OUT_RESET, true);
    }
}

static void esp32_rtc_brownout(void *opaque, int n, int level)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    uint32_t bo = REG(s, RTC_CNTL_BROWN_OUT);
    bool rising = level && !s->brownout_line;

    s->brownout_line = level;
    REG(s, RTC_CNTL_BROWN_OUT) = FIELD_DP32(bo, RTC_CNTL_BROWN_OUT, DET,
                                            level ? 1 : 0);
    if (!rising || !FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, ENA)) {
        return;
    }
    esp32_rtc_raise(s, ESP32_RTC_INT_BROWN_OUT);
    if (FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, RST_ENA) && s->rtc_slowclk_freq) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        timer_mod(&s->brownout_timer, esp32_rtc_ticks_to_ns(s, now,
                  FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, RST_WAIT)));
    }
}

static void esp32_rtc_update_cpu_stall(Esp32RtcCntlState *s)
{
    uint32_t c0 = REG(s, RTC_CNTL_OPTIONS0);
    uint32_t c1 = REG(s, RTC_CNTL_SW_CPU_STALL);
    uint32_t pro = (FIELD_EX32(c1, RTC_CNTL_SW_CPU_STALL, PROCPU_C1) << 2) |
                   FIELD_EX32(c0, RTC_CNTL_OPTIONS0, SW_STALL_PROCPU_C0);
    uint32_t app = (FIELD_EX32(c1, RTC_CNTL_SW_CPU_STALL, APPCPU_C1) << 2) |
                   FIELD_EX32(c0, RTC_CNTL_OPTIONS0, SW_STALL_APPCPU_C0);
    bool asleep = s->sleep != ESP32_RTC_AWAKE;
    bool stall[ESP32_CPU_COUNT] = {
        pro == SW_STALL_MAGIC || asleep,
        app == SW_STALL_MAGIC || asleep,
    };

    for (int i = 0; i < ESP32_CPU_COUNT; i++) {
        if (stall[i] != s->cpu_stall_state[i]) {
            s->cpu_stall_state[i] = stall[i];
            qemu_set_irq(s->cpu_stall_req[i], stall[i]);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * RC_FAST_CLK's oscillator is powered while neither ENB_CK8M nor
 * CK8M_FORCE_PD is set. CK8M_FORCE_PU and CK8M_FORCE_NOGATING only keep
 * it up through sleep, in which the model stops the digital clocks anyway.
 */
static bool esp32_rtc_rc_fast_powered(uint32_t conf)
{
    return !FIELD_EX32(conf, RTC_CNTL_CLK_CONF, ENB_CK8M) &&
           !FIELD_EX32(conf, RTC_CNTL_CLK_CONF, CK8M_FORCE_PD);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * RC_FAST_CLK's frequency as CK8M_DFREQ trims it. The TRM gives only its
 * 8 MHz default (CK8M_DFREQ 0, the reset value) and ESP-IDF only the
 * 8.5 MHz of its trim, CK8M_DFREQ 172; the model takes the frequency to be
 * linear in CK8M_DFREQ through those two points. Software that needs the
 * real rate calibrates against XTAL_CLK, as ESP-IDF's rtc_clk_cal does
 * with 8MD256, and sees whatever the model gives here.
 */
static uint32_t esp32_rtc_rc_fast_hz(uint32_t conf)
{
    uint32_t dfreq = FIELD_EX32(conf, RTC_CNTL_CLK_CONF, CK8M_DFREQ);

    return ESP32_RC_FAST_CLK_HZ +
        (uint64_t)dfreq * (ESP32_RC_FAST_TRIMMED_HZ - ESP32_RC_FAST_CLK_HZ) /
        ESP32_RC_FAST_TRIM_DFREQ;
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * [spec:nuos:req:emu.esp32.rtc]
 * Decode the clock selection and the slow clock sources' state.
 *
 * RC_FAST_CLK, the internal oscillator, runs once it is powered and has
 * started up (esp32_rtc_rc_fast_arm). It feeds the SoC clock mux directly
 * and the digital peripherals, LEDC's RC_FAST source, behind
 * DIG_CLK8M_EN; RTC_FAST_CLK takes it through the divider CK8M_DIV_SEL + 1
 * (ESP-IDF's clk_ll_rc_fast_set_divider: "the output from the divider is
 * passed into rtc_fast_clk MUX"). RC_FAST_DIV_CLK (8MD256) is it divided
 * by 128 << CK8M_DIV, or undivided with ENB_CK8M_DIV. XTAL32K_CLK runs
 * only with a crystal fitted and the oscillator powered up by RTCIO's
 * XPD_XTAL_32K. RTC_SLOW_CLK is whichever ANA_CLK_RTC_SEL picks,
 * RC_SLOW_CLK running always; it stops if its source does. The digital
 * domain sees 8MD256 and XTAL32K_CLK only through DIG_CLK8M_D256_EN and
 * DIG_XTAL32K_EN. Crystal start-up time and the oscillators' deviation
 * from nominal are not modelled.
 */
static void esp32_rtc_decode_clk_conf(Esp32RtcCntlState *s)
{
    uint32_t conf = REG(s, RTC_CNTL_CLK_CONF);
    uint32_t rc_fast = s->rc_fast_ready && esp32_rtc_rc_fast_powered(conf) ?
                       esp32_rtc_rc_fast_hz(conf) : 0;
    const uint32_t fastclk_freq[] = {
        s->xtal_apb_freq / 4,
        rc_fast / (FIELD_EX32(conf, RTC_CNTL_CLK_CONF, CK8M_DIV_SEL) + 1),
    };
    bool xpd_32k = s->rtcio &&
        (s->rtcio->regs[A_RTCIO_XTAL_32K_PAD / 4] &
         R_RTCIO_XTAL_32K_PAD_XPD_XTAL_32K_MASK);
    uint32_t slowclk_freq[3];

    s->soc_clk = FIELD_EX32(conf, RTC_CNTL_CLK_CONF, SOC_CLK_SEL);
    s->rtc_fastclk = FIELD_EX32(conf, RTC_CNTL_CLK_CONF, FAST_CLK_RTC_SEL);
    s->rtc_slowclk = FIELD_EX32(conf, RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL);
    s->rtc_fastclk_freq = fastclk_freq[s->rtc_fastclk];

    s->rc_fast_freq = rc_fast;
    s->rc_fast_dig_freq = FIELD_EX32(conf, RTC_CNTL_CLK_CONF, DIG_CLK8M_EN) ?
                          rc_fast : 0;
    if (FIELD_EX32(conf, RTC_CNTL_CLK_CONF, ENB_CK8M_DIV)) {
        s->rc_fast_d256_freq = rc_fast;
    } else {
        s->rc_fast_d256_freq = rc_fast /
            (128 << FIELD_EX32(conf, RTC_CNTL_CLK_CONF, CK8M_DIV));
    }
    s->xtal32k_freq = s->xtal32k_fitted && xpd_32k ? ESP32_XTAL32K_CLK_HZ : 0;

    slowclk_freq[ESP32_SLOW_CLK_RC] = ESP32_RC_SLOW_CLK_HZ;
    slowclk_freq[ESP32_SLOW_CLK_32KXTAL] = s->xtal32k_freq;
    slowclk_freq[ESP32_SLOW_CLK_8MD256] = s->rc_fast_d256_freq;
    s->rtc_slowclk_freq = slowclk_freq[s->rtc_slowclk];
}

/* Drive the clock outputs from the decoded state */
static void esp32_rtc_update_clock_outputs(Esp32RtcCntlState *s)
{
    uint32_t conf = REG(s, RTC_CNTL_CLK_CONF);

    clock_update_hz(s->xtal_clk, s->xtal_apb_freq);
    clock_update_hz(s->slow_clk, s->rtc_slowclk_freq);
    clock_update_hz(s->fast_clk, s->rtc_fastclk_freq);
    clock_update_hz(s->d256_dig_clk,
                    FIELD_EX32(conf, RTC_CNTL_CLK_CONF, DIG_CLK8M_D256_EN) ?
                    s->rc_fast_d256_freq : 0);
    clock_update_hz(s->xtal32k_dig_clk,
                    FIELD_EX32(conf, RTC_CNTL_CLK_CONF, DIG_XTAL32K_EN) ?
                    s->xtal32k_freq : 0);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * The clock selection or a slow clock source changed. The RTC timer and
 * watchdog count on at the old rate up to now and at the new one after; a
 * stopped RTC_SLOW_CLK freezes them, and holds off TIME_VALID and the
 * brownout reset's wait, which go on once it runs again.
 */
static void esp32_rtc_clocks_changed(Esp32RtcCntlState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t old_freq = s->rtc_slowclk_freq;
    uint32_t bo;

    s->time_base = esp32_rtc_time(s, now);
    s->time_base_ns = now;
    s->wdt_count_base = esp32_rtc_wdt_count(s, now);
    s->wdt_base_ns = now;
    esp32_rtc_decode_clk_conf(s);
    esp32_rtc_update_clock_outputs(s);
    if (s->rtc_slowclk_freq == old_freq) {
        return;
    }
    esp32_rtc_alarm_arm(s);
    esp32_rtc_wdt_update(s);

    timer_del(&s->valid_timer);
    if (s->valid_pending && s->rtc_slowclk_freq) {
        timer_mod(&s->valid_timer, esp32_rtc_ticks_to_ns(s, now, 1));
    }
    bo = REG(s, RTC_CNTL_BROWN_OUT);
    if (!s->rtc_slowclk_freq) {
        timer_del(&s->brownout_timer);
    } else if (!old_freq && s->brownout_line &&
               FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, ENA) &&
               FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, RST_ENA)) {
        timer_mod(&s->brownout_timer, esp32_rtc_ticks_to_ns(s, now,
                  FIELD_EX32(bo, RTC_CNTL_BROWN_OUT, RST_WAIT)));
    }
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * RC_FAST_CLK's oscillator, once powered, gives no clock until CK8M_WAIT
 * cycles of RTC_SLOW_CLK have passed; a write to CK8M_WAIT during the wait
 * moves its end. ESP-IDF's rtc_clk_8m_enable clears ENB_CK8M, sets
 * CK8M_WAIT to 5 and waits 50 us before using the clock, which covers the
 * 33 us of 5 cycles of the 150 kHz RC_SLOW_CLK. While RTC_SLOW_CLK is
 * stopped, as it is when it is 8MD256 from this very oscillator, the
 * wait is counted at RC_SLOW_CLK's nominal rate.
 */
static void esp32_rtc_rc_fast_arm(Esp32RtcCntlState *s)
{
    uint32_t wait = FIELD_EX32(REG(s, RTC_CNTL_TIMER1), RTC_CNTL_TIMER1,
                               CK8M_WAIT);
    uint32_t hz = s->rtc_slowclk_freq ? s->rtc_slowclk_freq :
                                        ESP32_RC_SLOW_CLK_HZ;

    timer_mod(&s->rc_fast_timer,
              s->rc_fast_on_ns + muldiv64(wait, NANOSECONDS_PER_SECOND, hz));
}

static void esp32_rtc_rc_fast_cb(void *opaque)
{
    Esp32RtcCntlState *s = opaque;

    s->rc_fast_ready = true;
    esp32_rtc_clocks_changed(s);
    qemu_irq_pulse(s->clk_update);
}

/*
 * A powered-down RTC memory loses its contents: on FORCE_PD, or in sleep
 * with PD_EN unless FORCE_PU holds it up, or in light sleep FORCE_LPU.
 */
static bool esp32_rtc_mem_lost(uint32_t pwc, unsigned pd_en, unsigned fpu,
                               unsigned fpd, unsigned flpu, bool deep)
{
    return extract32(pwc, fpd, 1) ||
           (extract32(pwc, pd_en, 1) && !extract32(pwc, fpu, 1) &&
            (deep || !extract32(pwc, flpu, 1)));
}

static void esp32_rtc_mem_power_down(Esp32RtcCntlState *s, bool deep)
{
    uint32_t pwc = REG(s, RTC_CNTL_PWC);

    if (esp32_rtc_mem_lost(pwc, R_RTC_CNTL_PWC_FASTMEM_PD_EN_SHIFT,
                           R_RTC_CNTL_PWC_FASTMEM_FORCE_PU_SHIFT,
                           R_RTC_CNTL_PWC_FASTMEM_FORCE_PD_SHIFT, 8, deep)) {
        memset(memory_region_get_ram_ptr(&s->fast_mem), LOST_MEM_PATTERN,
               ESP32_RTC_FAST_MEM_SIZE);
        memory_region_set_dirty(&s->fast_mem, 0, ESP32_RTC_FAST_MEM_SIZE);
    }
    if (esp32_rtc_mem_lost(pwc, R_RTC_CNTL_PWC_SLOWMEM_PD_EN_SHIFT,
                           R_RTC_CNTL_PWC_SLOWMEM_FORCE_PU_SHIFT,
                           R_RTC_CNTL_PWC_SLOWMEM_FORCE_PD_SHIFT, 11, deep)) {
        memset(memory_region_get_ram_ptr(&s->slow_mem), LOST_MEM_PATTERN,
               ESP32_RTC_SLOW_MEM_SIZE);
        memory_region_set_dirty(&s->slow_mem, 0, ESP32_RTC_SLOW_MEM_SIZE);
    }
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * Leave sleep for cause, a set of RTC_CNTL_WAKEUP_ENA bits. From light
 * sleep the CPUs carry on where they stopped; from deep sleep they start
 * from their reset vectors, the digital domain having powered up in its
 * reset state, with DEEPSLEEP_RESET as their reset cause.
 */
static void esp32_rtc_wake(Esp32RtcCntlState *s, uint32_t cause)
{
    if (s->sleep == ESP32_RTC_DEEP_SLEEP) {
        for (int i = 0; i < ESP32_CPU_COUNT; i++) {
            s->reset_cause[i] = ESP32_DEEPSLEEP_RESET;
        }
    }
    REG(s, RTC_CNTL_WAKEUP_STATE) = FIELD_DP32(REG(s, RTC_CNTL_WAKEUP_STATE),
                                               RTC_CNTL_WAKEUP_STATE,
                                               WAKEUP_CAUSE, cause);
    REG(s, RTC_CNTL_STATE0) &= ~R_RTC_CNTL_STATE0_SLEEP_EN_MASK;
    s->sleep = ESP32_RTC_AWAKE;
    s->timer_wakeup = false;
    s->dig_clk_gated = false;
    esp32_rtc_wdt_update(s);
    qemu_irq_pulse(s->clk_update);
    esp32_rtc_raise(s, ESP32_RTC_INT_SLP_WAKEUP);
    esp32_rtc_update_cpu_stall(s);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * Watch the enabled wakeup sources while asleep (TRM table 9.3-2):
 * - EXT0: the RTC GPIO RTCIO_EXT_WAKEUP0_SEL names at the level
 *   RTC_CNTL_EXT_WAKEUP0_LV;
 * - EXT1: with EXT_WAKEUP1_LV, any of the RTC GPIOs EXT_WAKEUP1_SEL selects
 *   high; without it, all of them low. EXT_WAKEUP1_STATUS records the pads
 *   that woke the chip;
 * - GPIO: an RTC GPIO's wakeup level, and in light sleep a digital GPIO's;
 * - the RTC timer's alarm;
 * - the touch, ULP, SDIO, Wi-Fi, UART and Bluetooth requests, from their
 *   blocks; in deep sleep only touch and the ULP.
 * Software can also wake the chip by setting RTC_CNTL_SLP_WAKEUP.
 */
static void esp32_rtc_check_wakeup(Esp32RtcCntlState *s)
{
    bool deep = s->sleep == ESP32_RTC_DEEP_SLEEP;
    uint32_t ena = FIELD_EX32(REG(s, RTC_CNTL_WAKEUP_STATE),
                              RTC_CNTL_WAKEUP_STATE, WAKEUP_ENA);
    uint32_t ext = REG(s, RTC_CNTL_EXT_WAKEUP_CONF);
    uint32_t sel1 = FIELD_EX32(REG(s, RTC_CNTL_EXT_WAKEUP1),
                               RTC_CNTL_EXT_WAKEUP1, SEL);
    uint32_t in, active, cause, ext1_pads;
    unsigned sel0;

    if (s->sleep == ESP32_RTC_AWAKE || s->sleep_reset_pending) {
        return;
    }
    in = esp32_rtcio_inputs(s->rtcio);
    active = s->wakeup_lines & (deep ? WAKEUP_DEEP_SLEEP_MASK : ~0u);
    active &= ~(BIT(ESP32_RTC_WAKEUP_EXT0) | BIT(ESP32_RTC_WAKEUP_EXT1) |
                BIT(ESP32_RTC_WAKEUP_GPIO) | BIT(ESP32_RTC_WAKEUP_TIMER));

    sel0 = esp32_rtcio_ext0_sel(s->rtcio);
    if (sel0 < ESP32_RTCIO_PAD_COUNT &&
        extract32(in, sel0, 1) ==
        FIELD_EX32(ext, RTC_CNTL_EXT_WAKEUP_CONF, EXT_WAKEUP0_LV)) {
        active |= BIT(ESP32_RTC_WAKEUP_EXT0);
    }
    if (FIELD_EX32(ext, RTC_CNTL_EXT_WAKEUP_CONF, EXT_WAKEUP1_LV)) {
        ext1_pads = in & sel1;
    } else {
        ext1_pads = (in & sel1) ? 0 : sel1;
    }
    if (ext1_pads) {
        active |= BIT(ESP32_RTC_WAKEUP_EXT1);
    }
    if (esp32_rtcio_gpio_wakeup(s->rtcio) ||
        (!deep && s->gpio_wakeup_line)) {
        active |= BIT(ESP32_RTC_WAKEUP_GPIO);
    }
    if (s->timer_wakeup) {
        active |= BIT(ESP32_RTC_WAKEUP_TIMER);
    }

    cause = active & ena;
    if (!cause && !FIELD_EX32(REG(s, RTC_CNTL_STATE0), RTC_CNTL_STATE0,
                              SLP_WAKEUP)) {
        return;
    }
    if (cause & BIT(ESP32_RTC_WAKEUP_EXT1)) {
        REG(s, RTC_CNTL_EXT_WAKEUP1_STATUS) |= ext1_pads;
    }
    esp32_rtc_wake(s, cause);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * SLEEP_EN starts the sleep. RTC_CNTL_DG_WRAP_PD_EN, powering the digital
 * domain down, makes it a deep sleep; otherwise it is a light sleep, in
 * which the CPUs stall and the digital domain's clocks stop. A GPIO or SDIO
 * request already pending rejects the sleep when the reject is enabled
 * for that sleep and source (TRM 9.3.11), as does RTC_CNTL_SLP_REJECT.
 */
static void esp32_rtc_sleep(Esp32RtcCntlState *s)
{
    uint32_t rej = REG(s, RTC_CNTL_SLP_REJECT_CONF);
    uint32_t ena = FIELD_EX32(REG(s, RTC_CNTL_WAKEUP_STATE),
                              RTC_CNTL_WAKEUP_STATE, WAKEUP_ENA);
    bool deep = FIELD_EX32(REG(s, RTC_CNTL_DIG_PWC), RTC_CNTL_DIG_PWC,
                           DG_WRAP_PD_EN);
    bool reject_en = deep ?
        FIELD_EX32(rej, RTC_CNTL_SLP_REJECT_CONF, DEEP_SLP_REJECT_EN) :
        FIELD_EX32(rej, RTC_CNTL_SLP_REJECT_CONF, LIGHT_SLP_REJECT_EN);
    unsigned reject = 0;

    if (reject_en &&
        FIELD_EX32(rej, RTC_CNTL_SLP_REJECT_CONF, GPIO_REJECT_EN) &&
        (esp32_rtcio_gpio_wakeup(s->rtcio) ||
         (!deep && s->gpio_wakeup_line))) {
        reject = REJECT_CAUSE_GPIO;
    } else if (reject_en &&
               FIELD_EX32(rej, RTC_CNTL_SLP_REJECT_CONF, SDIO_REJECT_EN) &&
               (s->wakeup_lines & BIT(ESP32_RTC_WAKEUP_SDIO))) {
        reject = REJECT_CAUSE_SDIO;
    }
    if (reject || FIELD_EX32(REG(s, RTC_CNTL_STATE0), RTC_CNTL_STATE0,
                             SLP_REJECT)) {
        REG(s, RTC_CNTL_SLP_REJECT_CONF) =
            FIELD_DP32(rej, RTC_CNTL_SLP_REJECT_CONF, REJECT_CAUSE, reject);
        REG(s, RTC_CNTL_STATE0) &= ~R_RTC_CNTL_STATE0_SLEEP_EN_MASK;
        esp32_rtc_raise(s, ESP32_RTC_INT_SLP_REJECT);
        return;
    }

    if (ena & WAKEUP_UNMODELLED_MASK) {
        qemu_log_mask(LOG_UNIMP,
                      "esp32_rtc_cntl: SDIO, Wi-Fi, UART and Bluetooth "
                      "wakeup (WAKEUP_ENA 0x%x) are not modelled\n", ena);
    }
    REG(s, RTC_CNTL_WAKEUP_STATE) = FIELD_DP32(REG(s, RTC_CNTL_WAKEUP_STATE),
                                               RTC_CNTL_WAKEUP_STATE,
                                               WAKEUP_CAUSE, 0);
    s->timer_wakeup = false;
    s->sleep = deep ? ESP32_RTC_DEEP_SLEEP : ESP32_RTC_LIGHT_SLEEP;
    s->dig_clk_gated = true;
    esp32_rtc_update_cpu_stall(s);
    esp32_rtc_wdt_update(s);
    esp32_rtc_mem_power_down(s, deep);
    qemu_irq_pulse(s->clk_update);
    if (deep) {
        if (FIELD_EX32(REG(s, RTC_CNTL_DIG_ISO), RTC_CNTL_DIG_ISO,
                       DG_PAD_AUTOHOLD_EN)) {
            REG(s, RTC_CNTL_DIG_ISO) |= R_RTC_CNTL_DIG_ISO_DG_PAD_AUTOHOLD_MASK;
            qemu_log_mask(LOG_UNIMP,
                          "esp32_rtc_cntl: holding the digital pads through "
                          "deep sleep is not modelled\n");
        }
        s->dig_powered_down = true;
        s->sleep_reset_pending = true;
        qemu_irq_pulse(s->sleep_reset_req);
        return;
    }
    esp32_rtc_check_wakeup(s);
}

void esp32_rtc_cntl_sleep_reset_done(Esp32RtcCntlState *s)
{
    s->sleep_reset_pending = false;
    esp32_rtc_check_wakeup(s);
}

void esp32_rtc_cntl_rtcio_changed(Esp32RtcCntlState *s)
{
    bool xpd_32k = s->rtcio->regs[A_RTCIO_XTAL_32K_PAD / 4] &
                   R_RTCIO_XTAL_32K_PAD_XPD_XTAL_32K_MASK;

    if (s->xtal32k_fitted && xpd_32k != (s->xtal32k_freq != 0)) {
        esp32_rtc_clocks_changed(s);
    }
    esp32_rtc_check_wakeup(s);
}

uint32_t esp32_rtc_cntl_hold_force(Esp32RtcCntlState *s)
{
    return REG(s, RTC_CNTL_HOLD_FORCE);
}

bool esp32_rtc_cntl_dg_pad_force_hold(Esp32RtcCntlState *s)
{
    return FIELD_EX32(REG(s, RTC_CNTL_DIG_ISO), RTC_CNTL_DIG_ISO,
                      DG_PAD_FORCE_HOLD);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * The RTC fast memory's CRC engine (RTC_MEM_CONF, RTC_MEM_CRC_RES; ESP-IDF
 * soc/rtc_cntl_reg.h), with which the ROM checks a deep-sleep wake stub
 * before running it (TRM 9.3.13). Setting CRC_START computes the CRC of
 * words CRC_ADDR to CRC_ADDR + CRC_LEN of the fast memory and sets
 * CRC_FINISH; clearing CRC_START clears CRC_FINISH. The TRM does not give
 * the polynomial: the model computes the IEEE 802.3 CRC-32 of the words'
 * bytes in address order, which the ROM's check, made with the same engine,
 * accepts.
 */
static void esp32_rtc_mem_crc(Esp32RtcCntlState *s, uint32_t old)
{
    uint32_t conf = REG(s, RTC_MEM_CONF);
    bool start = FIELD_EX32(conf, RTC_MEM_CONF, CRC_START);
    const uint8_t *mem = memory_region_get_ram_ptr(&s->fast_mem);
    uint32_t first, len, crc = 0xffffffff;

    if (!start) {
        REG(s, RTC_MEM_CONF) &= ~R_RTC_MEM_CONF_CRC_FINISH_MASK;
        return;
    }
    if (FIELD_EX32(old, RTC_MEM_CONF, CRC_START)) {
        return;
    }
    first = FIELD_EX32(conf, RTC_MEM_CONF, CRC_ADDR) * 4;
    len = (FIELD_EX32(conf, RTC_MEM_CONF, CRC_LEN) + 1) * 4;
    if (first + len > ESP32_RTC_FAST_MEM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_cntl: RTC memory CRC range 0x%x+0x%x runs "
                      "past the fast memory\n", first, len);
        len = ESP32_RTC_FAST_MEM_SIZE - MIN(first, ESP32_RTC_FAST_MEM_SIZE);
    }
    for (uint32_t i = 0; i < len; i++) {
        crc = crc32_table[(crc ^ mem[first + i]) & 0xff] ^ (crc >> 8);
    }
    REG(s, RTC_MEM_CRC_RES) = ~crc;
    REG(s, RTC_MEM_CONF) |= R_RTC_MEM_CONF_CRC_FINISH_MASK;
}

static uint64_t esp32_rtc_cntl_read(void *opaque, hwaddr addr,
                                    unsigned int size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    uint32_t r;

    switch (addr) {
    case A_RTC_CNTL_TIME0:
        return s->time_latched & UINT32_MAX;
    case A_RTC_CNTL_TIME1:
        return s->time_latched >> 32;
    case A_RTC_CNTL_RESET_STATE:
        r = REG(s, RTC_CNTL_RESET_STATE);
        r = FIELD_DP32(r, RTC_CNTL_RESET_STATE, RESET_CAUSE_PROCPU,
                       s->reset_cause[0]);
        r = FIELD_DP32(r, RTC_CNTL_RESET_STATE, RESET_CAUSE_APPCPU,
                       s->reset_cause[1]);
        return r;
    case A_RTC_CNTL_INT_ST:
        return REG(s, RTC_CNTL_INT_RAW) & REG(s, RTC_CNTL_INT_ENA);
    case A_RTC_CNTL_INT_CLR:
    case A_RTC_CNTL_WDTFEED:
        return 0;
    case A_RTC_CNTL_LOW_POWER_ST:
        r = 0;
        r = FIELD_DP32(r, RTC_CNTL_LOW_POWER_ST, MAIN_STATE_IN_IDLE,
                       s->sleep == ESP32_RTC_AWAKE);
        r = FIELD_DP32(r, RTC_CNTL_LOW_POWER_ST, RDY_FOR_WAKEUP,
                       s->sleep != ESP32_RTC_AWAKE);
        return r;
    default:
        if (addr <= A_RTC_CNTL_BROWN_OUT || addr == A_RTC_MEM_CONF ||
            addr == A_RTC_MEM_CRC_RES || addr == A_RTC_CNTL_DATE) {
            return s->regs[addr / 4];
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_cntl: read of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static bool esp32_rtc_wdt_locked(Esp32RtcCntlState *s, hwaddr addr)
{
    if (REG(s, RTC_CNTL_WDTWPROTECT) == ESP32_RTC_WDT_WKEY) {
        return false;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32_rtc_cntl: write to RTC watchdog register 0x%"
                  HWADDR_PRIx " while it is write-protected\n", addr);
    return true;
}

static void esp32_rtc_cntl_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    uint32_t *r = &s->regs[addr / 4];
    uint32_t v = value;
    uint32_t old = *r;
    uint32_t writable = reg_info[addr / 4].writable;

    switch (addr) {
    case A_RTC_CNTL_OPTIONS0:
        *r = v & writable;
        if (v & R_RTC_CNTL_OPTIONS0_SW_SYS_RESET_MASK) {
            esp32_rtc_system_reset(s, ESP32_SW_SYS_RESET, false);
        }
        if (v & R_RTC_CNTL_OPTIONS0_SW_APPCPU_RESET_MASK) {
            s->reset_cause[1] = ESP32_SW_CPU_RESET;
            qemu_irq_pulse(s->cpu_reset_req[1]);
        }
        if (v & R_RTC_CNTL_OPTIONS0_SW_PROCPU_RESET_MASK) {
            s->reset_cause[0] = ESP32_SW_CPU_RESET;
            qemu_irq_pulse(s->cpu_reset_req[0]);
        }
        esp32_rtc_update_cpu_stall(s);
        /* [spec:nuos:req:emu.esp32.analog] The BBPLL's power */
        qemu_irq_pulse(s->clk_update);
        break;

    case A_RTC_CNTL_ANA_CONF:
        *r = v & writable;
        /* [spec:nuos:req:emu.esp32.analog] The APLL's power */
        qemu_irq_pulse(s->clk_update);
        break;

    case A_RTC_CNTL_SLP_TIMER0:
    case A_RTC_CNTL_SLP_TIMER1:
        *r = v & writable;
        esp32_rtc_alarm_arm(s);
        break;

    case A_RTC_CNTL_TIME_UPDATE:
        if (v & R_RTC_CNTL_TIME_UPDATE_UPDATE_MASK) {
            esp32_rtc_time_update(s);
        }
        break;

    case A_RTC_CNTL_STATE0:
        *r = (v & writable) | (old & ~writable);
        if (((v & ~old) & R_RTC_CNTL_STATE0_SLEEP_EN_MASK) &&
            s->sleep == ESP32_RTC_AWAKE) {
            esp32_rtc_sleep(s);
        } else {
            esp32_rtc_check_wakeup(s);
        }
        qemu_set_irq(s->touch_timer_en,
                     FIELD_EX32(*r, RTC_CNTL_STATE0, TOUCH_SLP_TIMER_EN));
        /* [spec:nuos:req:emu.esp32.ulp] */
        qemu_set_irq(s->ulp_timer_en,
                     FIELD_EX32(*r, RTC_CNTL_STATE0, ULP_CP_SLP_TIMER_EN));
        break;

    case A_RTC_CNTL_RESET_STATE:
        *r = v & writable;
        s->stat_vector_sel[0] = FIELD_EX32(v, RTC_CNTL_RESET_STATE,
                                           PROCPU_STAT_VECTOR_SEL);
        s->stat_vector_sel[1] = FIELD_EX32(v, RTC_CNTL_RESET_STATE,
                                           APPCPU_STAT_VECTOR_SEL);
        break;

    case A_RTC_CNTL_WAKEUP_STATE:
        *r = (v & writable) | (old & ~writable);
        esp32_rtc_check_wakeup(s);
        break;

    case A_RTC_CNTL_INT_ENA:
        *r = v & writable;
        esp32_rtc_update_irq(s);
        break;

    case A_RTC_CNTL_INT_CLR:
        REG(s, RTC_CNTL_INT_RAW) &= ~(v & reg_info[A_RTC_CNTL_INT_ENA / 4]
                                          .writable);
        esp32_rtc_update_irq(s);
        break;

    case A_RTC_CNTL_SLP_REJECT_CONF:
    case A_RTC_CNTL_DIG_ISO:
        *r = (v & writable) | (old & ~writable);
        if (addr == A_RTC_CNTL_DIG_ISO) {
            if (v & R_RTC_CNTL_DIG_ISO_CLR_DG_PAD_AUTOHOLD_MASK) {
                *r &= ~R_RTC_CNTL_DIG_ISO_DG_PAD_AUTOHOLD_MASK;
            }
            esp32_rtcio_hold_changed(s->rtcio);
        }
        break;

    case A_RTC_CNTL_EXT_WAKEUP_CONF:
        *r = v & writable;
        esp32_rtc_check_wakeup(s);
        break;

    case A_RTC_CNTL_CLK_CONF:
        if (FIELD_EX32(v, RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL) == 3) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_rtc_cntl: reserved RTC_SLOW_CLK source 3\n");
            v = FIELD_DP32(v, RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL,
                           s->rtc_slowclk);
        }
        *r = v & writable;
        if (esp32_rtc_rc_fast_powered(*r) && !esp32_rtc_rc_fast_powered(old)) {
            s->rc_fast_on_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->rc_fast_ready = false;
            esp32_rtc_rc_fast_arm(s);
        } else if (!esp32_rtc_rc_fast_powered(*r)) {
            s->rc_fast_ready = false;
            timer_del(&s->rc_fast_timer);
        }
        esp32_rtc_clocks_changed(s);
        qemu_irq_pulse(s->clk_update);
        break;

    case A_RTC_CNTL_TIMER1:
        *r = v & writable;
        if (esp32_rtc_rc_fast_powered(REG(s, RTC_CNTL_CLK_CONF)) &&
            !s->rc_fast_ready) {
            esp32_rtc_rc_fast_arm(s);
        }
        break;

    case A_RTC_CNTL_PWC:
        *r = v & writable;
        if (v & (R_RTC_CNTL_PWC_FASTMEM_FORCE_PD_MASK |
                 R_RTC_CNTL_PWC_SLOWMEM_FORCE_PD_MASK)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_rtc_cntl: forcing an RTC memory off while "
                          "awake is not modelled\n");
        }
        break;

    case A_RTC_CNTL_WDTCONFIG0:
        if (esp32_rtc_wdt_locked(s, addr)) {
            break;
        }
        *r = v & writable;
        if (FIELD_EX32(v, RTC_CNTL_WDTCONFIG0, EN) &&
            !FIELD_EX32(old, RTC_CNTL_WDTCONFIG0, EN)) {
            esp32_rtc_wdt_restart(s);
        } else {
            esp32_rtc_wdt_update(s);
        }
        break;

    case A_RTC_CNTL_WDTCONFIG1 ... A_RTC_CNTL_WDTCONFIG4:
        if (esp32_rtc_wdt_locked(s, addr)) {
            break;
        }
        *r = v;
        esp32_rtc_wdt_update(s);
        break;

    case A_RTC_CNTL_WDTFEED:
        if (esp32_rtc_wdt_locked(s, addr)) {
            break;
        }
        if (v & R_RTC_CNTL_WDTFEED_FEED_MASK) {
            esp32_rtc_wdt_restart(s);
        }
        break;

    case A_RTC_CNTL_SW_CPU_STALL:
        *r = v & writable;
        esp32_rtc_update_cpu_stall(s);
        break;

    case A_RTC_CNTL_HOLD_FORCE:
        *r = v & writable;
        esp32_rtcio_hold_changed(s->rtcio);
        break;

    case A_RTC_CNTL_EXT_WAKEUP1:
        *r = v & writable;
        if (v & R_RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_MASK) {
            REG(s, RTC_CNTL_EXT_WAKEUP1_STATUS) = 0;
        }
        esp32_rtc_check_wakeup(s);
        break;

    case A_RTC_CNTL_BROWN_OUT:
        *r = (v & writable) | (old & ~writable);
        break;

    case A_RTC_MEM_CONF:
        *r = (v & writable) | (old & ~writable);
        esp32_rtc_mem_crc(s, old);
        break;

    case A_RTC_CNTL_TIME0:
    case A_RTC_CNTL_TIME1:
    case A_RTC_CNTL_INT_RAW:
    case A_RTC_CNTL_INT_ST:
    case A_RTC_CNTL_LOW_POWER_ST:
    case A_RTC_CNTL_DIAG1:
    case A_RTC_CNTL_EXT_WAKEUP1_STATUS:
    case A_RTC_MEM_CRC_RES:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_cntl: write to read-only register 0x%"
                      HWADDR_PRIx "\n", addr);
        break;

    case A_RTC_CNTL_TIMER2 ... A_RTC_CNTL_TIMER5:
    case A_RTC_CNTL_STORE0 ... A_RTC_CNTL_EXT_XTL_CONF:
    case A_RTC_CNTL_CPU_PERIOD_CONF ... A_RTC_CNTL_SDIO_ACT_CONF:
    case A_RTC_CNTL_SDIO_CONF ... A_RTC_CNTL_VREG:
    case A_RTC_CNTL_DIG_PWC:
    case A_RTC_CNTL_WDTWPROTECT ... A_RTC_CNTL_TEST_MUX:
    case A_RTC_CNTL_STORE4 ... A_RTC_CNTL_STORE7:
    case A_RTC_CNTL_DATE:
        *r = v & writable;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_cntl: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps esp32_rtc_cntl_ops = {
    .read =  esp32_rtc_cntl_read,
    .write = esp32_rtc_cntl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_rtc_wakeup_line(void *opaque, int n, int level)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);

    s->wakeup_lines = deposit32(s->wakeup_lines, n, 1, level != 0);
    esp32_rtc_check_wakeup(s);
}

static void esp32_rtc_gpio_wakeup_line(void *opaque, int n, int level)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);

    s->gpio_wakeup_line = level;
    esp32_rtc_check_wakeup(s);
}

static void esp32_rtc_int_line(void *opaque, int n, int level)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);

    if (level) {
        esp32_rtc_raise(s, n);
    }
}

static void esp32_rtc_cntl_reset_hold(Object *obj, ResetType type)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool power_on = !s->rtc_reset_pending;

    timer_del(&s->alarm_timer);
    timer_del(&s->valid_timer);
    timer_del(&s->wdt_timer);
    timer_del(&s->brownout_timer);
    timer_del(&s->rc_fast_timer);

    if (power_on) {
        s->time_base = 0;
        s->time_latched = 0;
        for (int i = 0; i < ESP32_CPU_COUNT; i++) {
            s->reset_cause[i] = ESP32_POWERON_RESET;
        }
    } else {
        s->time_base = esp32_rtc_time(s, now);
    }
    s->time_base_ns = now;
    s->rtc_reset_pending = false;
    s->valid_pending = false;

    for (int i = 0; i < ESP32_RTC_CNTL_REG_COUNT; i++) {
        s->regs[i] = reg_info[i].reset;
    }
    REG(s, RTC_CNTL_BROWN_OUT) = FIELD_DP32(REG(s, RTC_CNTL_BROWN_OUT),
                                            RTC_CNTL_BROWN_OUT, DET,
                                            s->brownout_line);
    for (int i = 0; i < ESP32_CPU_COUNT; i++) {
        s->stat_vector_sel[i] = true;
    }
    /* RC_FAST_CLK, powered at reset, has started up by its end */
    s->rc_fast_on_ns = now;
    s->rc_fast_ready = true;
    esp32_rtc_decode_clk_conf(s);

    s->sleep = ESP32_RTC_AWAKE;
    s->sleep_reset_pending = false;
    s->timer_wakeup = false;
    s->dig_clk_gated = false;
    s->dig_powered_down = false;
    s->wdt_stage = 0;
    s->wdt_count_base = 0;
    s->wdt_base_ns = now;
    s->wdt_running = false;
}

static void esp32_rtc_cntl_reset_exit(Object *obj, ResetType type)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(obj);

    esp32_rtc_update_clock_outputs(s);
    esp32_rtc_wdt_update(s);
    esp32_rtc_update_irq(s);
    esp32_rtc_update_cpu_stall(s);
    qemu_set_irq(s->touch_timer_en,
                 FIELD_EX32(REG(s, RTC_CNTL_STATE0), RTC_CNTL_STATE0,
                            TOUCH_SLP_TIMER_EN));
    qemu_set_irq(s->ulp_timer_en,
                 FIELD_EX32(REG(s, RTC_CNTL_STATE0), RTC_CNTL_STATE0,
                            ULP_CP_SLP_TIMER_EN));
}

static void esp32_rtc_cntl_realize(DeviceState *dev, Error **errp)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(dev);

    if (!s->rtcio) {
        error_setg(errp, "esp32_rtc_cntl: rtcio link not set");
        return;
    }
    if (!memory_region_init_ram(&s->fast_mem, OBJECT(dev), "esp32.rtcfast",
                                ESP32_RTC_FAST_MEM_SIZE, errp)) {
        return;
    }
    if (!memory_region_init_ram(&s->slow_mem, OBJECT(dev), "esp32.rtcslow",
                                ESP32_RTC_SLOW_MEM_SIZE, errp)) {
        return;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->fast_mem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->slow_mem);
    esp32_rtc_decode_clk_conf(s);
    esp32_rtc_update_clock_outputs(s);
}

static void esp32_rtc_cntl_init(Object *obj)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rtc_cntl_ops, s,
                          TYPE_ESP32_RTC_CNTL, ESP32_RTC_CNTL_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->dig_reset_req,
                             ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_out_named(dev, &s->rtc_reset_req,
                             ESP32_RTC_RTC_RESET_GPIO, 1);
    qdev_init_gpio_out_named(dev, &s->sleep_reset_req,
                             ESP32_RTC_SLEEP_RESET_GPIO, 1);
    qdev_init_gpio_out_named(dev, &s->cpu_reset_req[0],
                             ESP32_RTC_CPU_RESET_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_out_named(dev, &s->cpu_stall_req[0],
                             ESP32_RTC_CPU_STALL_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_out_named(dev, &s->clk_update,
                             ESP32_RTC_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_out_named(dev, &s->touch_timer_en,
                             ESP32_RTC_TOUCH_TIMER_GPIO, 1);
    qdev_init_gpio_out_named(dev, &s->ulp_timer_en,
                             ESP32_RTC_ULP_TIMER_GPIO, 1);
    qdev_init_gpio_in_named(dev, esp32_rtc_wakeup_line,
                            ESP32_RTC_WAKEUP_IN, ESP32_RTC_WAKEUP_COUNT);
    qdev_init_gpio_in_named(dev, esp32_rtc_gpio_wakeup_line,
                            ESP32_RTC_GPIO_WAKEUP_IN, 1);
    qdev_init_gpio_in_named(dev, esp32_rtc_int_line,
                            ESP32_RTC_INT_IN, ESP32_RTC_INT_COUNT);
    qdev_init_gpio_in_named(dev, esp32_rtc_brownout,
                            ESP32_RTC_BROWNOUT_IN, 1);
    s->xtal_clk = qdev_init_clock_out(dev, ESP32_RTC_XTAL_CLK);
    s->slow_clk = qdev_init_clock_out(dev, ESP32_RTC_SLOW_CLK);
    s->fast_clk = qdev_init_clock_out(dev, ESP32_RTC_FAST_CLK);
    s->d256_dig_clk = qdev_init_clock_out(dev, ESP32_RTC_D256_DIG_CLK);
    s->xtal32k_dig_clk = qdev_init_clock_out(dev, ESP32_RTC_XTAL32K_DIG_CLK);

    timer_init_ns(&s->alarm_timer, QEMU_CLOCK_VIRTUAL, esp32_rtc_alarm_cb, s);
    timer_init_ns(&s->valid_timer, QEMU_CLOCK_VIRTUAL, esp32_rtc_valid_cb, s);
    timer_init_ns(&s->wdt_timer, QEMU_CLOCK_VIRTUAL, esp32_rtc_wdt_cb, s);
    timer_init_ns(&s->brownout_timer, QEMU_CLOCK_VIRTUAL,
                  esp32_rtc_brownout_cb, s);
    timer_init_ns(&s->rc_fast_timer, QEMU_CLOCK_VIRTUAL,
                  esp32_rtc_rc_fast_cb, s);
    s->rc_fast_ready = true;

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        s->reset_cause[i] = ESP32_POWERON_RESET;
        s->stat_vector_sel[i] = true;
    }
    s->xtal_apb_freq = 40000000;
    for (int i = 0; i < ESP32_RTC_CNTL_REG_COUNT; i++) {
        s->regs[i] = reg_info[i].reset;
    }
    esp32_rtc_decode_clk_conf(s);
}

static int esp32_rtc_cntl_post_load(void *opaque, int version_id)
{
    esp32_rtc_decode_clk_conf(opaque);
    esp32_rtc_update_clock_outputs(opaque);
    return 0;
}

static const VMStateDescription vmstate_esp32_rtc_cntl = {
    .name = TYPE_ESP32_RTC_CNTL,
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = esp32_rtc_cntl_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Esp32RtcCntlState,
                             ESP32_RTC_CNTL_REG_COUNT),
        VMSTATE_TIMER(alarm_timer, Esp32RtcCntlState),
        VMSTATE_TIMER(valid_timer, Esp32RtcCntlState),
        VMSTATE_TIMER(wdt_timer, Esp32RtcCntlState),
        VMSTATE_TIMER(brownout_timer, Esp32RtcCntlState),
        VMSTATE_TIMER(rc_fast_timer, Esp32RtcCntlState),
        VMSTATE_INT64(rc_fast_on_ns, Esp32RtcCntlState),
        VMSTATE_BOOL(rc_fast_ready, Esp32RtcCntlState),
        VMSTATE_UINT64(time_base, Esp32RtcCntlState),
        VMSTATE_INT64(time_base_ns, Esp32RtcCntlState),
        VMSTATE_UINT64(time_latched, Esp32RtcCntlState),
        VMSTATE_BOOL(valid_pending, Esp32RtcCntlState),
        VMSTATE_UINT32(wdt_stage, Esp32RtcCntlState),
        VMSTATE_UINT64(wdt_count_base, Esp32RtcCntlState),
        VMSTATE_INT64(wdt_base_ns, Esp32RtcCntlState),
        VMSTATE_BOOL(wdt_running, Esp32RtcCntlState),
        VMSTATE_UINT32(sleep, Esp32RtcCntlState),
        VMSTATE_BOOL(sleep_reset_pending, Esp32RtcCntlState),
        VMSTATE_BOOL(timer_wakeup, Esp32RtcCntlState),
        VMSTATE_UINT32(wakeup_lines, Esp32RtcCntlState),
        VMSTATE_BOOL(gpio_wakeup_line, Esp32RtcCntlState),
        VMSTATE_BOOL(brownout_line, Esp32RtcCntlState),
        VMSTATE_BOOL(rtc_reset_pending, Esp32RtcCntlState),
        VMSTATE_BOOL_ARRAY(cpu_stall_state, Esp32RtcCntlState,
                           ESP32_CPU_COUNT),
        VMSTATE_BOOL(dig_clk_gated, Esp32RtcCntlState),
        VMSTATE_BOOL(dig_powered_down, Esp32RtcCntlState),
        VMSTATE_UINT32_ARRAY(reset_cause, Esp32RtcCntlState, ESP32_CPU_COUNT),
        VMSTATE_BOOL_ARRAY(stat_vector_sel, Esp32RtcCntlState,
                           ESP32_CPU_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32_rtc_cntl_properties[] = {
    DEFINE_PROP_LINK("rtcio", Esp32RtcCntlState, rtcio, TYPE_ESP32_RTCIO,
                     Esp32RtcIoState *),
    DEFINE_PROP_BOOL("xtal32k", Esp32RtcCntlState, xtal32k_fitted, false),
};

static void esp32_rtc_cntl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_rtc_cntl_reset_hold;
    rc->phases.exit = esp32_rtc_cntl_reset_exit;
    dc->realize = esp32_rtc_cntl_realize;
    dc->vmsd = &vmstate_esp32_rtc_cntl;
    device_class_set_props(dc, esp32_rtc_cntl_properties);
}

/* [spec:nuos:req:emu.esp32.rtc] */
static const TypeInfo esp32_rtc_cntl_info = {
    .name = TYPE_ESP32_RTC_CNTL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RtcCntlState),
    .instance_init = esp32_rtc_cntl_init,
    .class_init = esp32_rtc_cntl_class_init
};

static void esp32_rtc_cntl_register_types(void)
{
    type_register_static(&esp32_rtc_cntl_info);
}

type_init(esp32_rtc_cntl_register_types)
