/*
 * QTest testcase for the ESP32's watchdogs with stage holds of 0: the
 * timer groups' MWDTs and the RTC watchdog
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Each watchdog's 32-bit expiry counter counts from 0 as a stage begins
 * and the stage ends when the counter, counting on, comes to its hold
 * (TRM 11.3.1): a hold of 0, or one the counter is at or past when it is
 * written, is reached after the counter wraps, 2^32 ticks on. The MWDTs
 * here count APB_CLK, 40 MHz from reset (25 ns), over a prescaler of 1;
 * the RTC watchdog RTC_SLOW_CLK, the 150 kHz RC oscillator from reset.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define TIMG0                   0x3ff5f000
#define TIMG1                   0x3ff60000
#define T_WDTCONFIG0            0x48
#define T_WDTCONFIG1            0x4c
#define T_WDTCONFIG(n)          (0x50 + 4 * (n))
#define T_WDTFEED               0x60
#define T_WDTWPROTECT           0x64
#define T_INT_RAW               0x9c
#define T_INT_CLR               0xa4
#define MW_EN                   (1u << 31)
#define MW_STG(n, a)            ((uint32_t)(a) << (29 - 2 * (n)))
#define MW_LEVEL_INT            (1u << 21)
#define MW_PRESCALE(p)          ((uint32_t)(p) << 16)
#define MW_INT_WDT              (1u << 2)

#define RTC_CNTL                0x3ff48000
#define RESET_STATE             (RTC_CNTL + 0x34)
#define INT_RAW                 (RTC_CNTL + 0x40)
#define INT_CLR                 (RTC_CNTL + 0x48)
#define STORE0                  (RTC_CNTL + 0x4c)
#define WDTCONFIG0              (RTC_CNTL + 0x8c)
#define WDTCONFIG(n)            (RTC_CNTL + 0x90 + 4 * (n))
#define WDTFEED                 (RTC_CNTL + 0xa0)
#define WDTWPROTECT             (RTC_CNTL + 0xa4)
#define RW_EN                   (1u << 31)
#define RW_STG(n, a)            ((uint32_t)(a) << (28 - 3 * (n)))
#define RW_LEVEL_INT            (1u << 17)
#define RW_PROCPU_RESET_EN      (1u << 9)
#define RW_APPCPU_RESET_EN      (1u << 8)
#define RW_FLASHBOOT            (1u << 10)
#define RW_INT_WDT              (1u << 3)

#define WKEY                    0x50d83aa1

/* Stage actions: the MWDT's take 0 to 3, the RTC watchdog's 0 to 4. */
#define STG_OFF                 0
#define STG_INT                 1
#define STG_CPU_RESET           2
#define STG_SYS_RESET           3
#define STG_RTC_RESET           4

#define POWERON_RESET           1
#define TG1WDT_SYS_RESET        8
#define RTCWDT_SYS_RESET        9
#define TGWDT_CPU_RESET         11
#define RTCWDT_CPU_RESET        13
#define RTCWDT_RTC_RESET        16

#define WRAP                    (1ull << 32)
#define APB_NS                  25
/*
 * When the RTC watchdog's counter has counted 2^32 cycles of the 150 kHz
 * RTC_SLOW_CLK: 2^32 * 20000/3 ns is 28633115306666 2/3 ns, so at the
 * first whole ns after.
 */
#define RW_WRAP_NS              28633115306667ll

static int64_t now;

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    now = 0;
    return qts;
}

static void step(QTestState *qts, int64_t ns)
{
    now = qtest_clock_step(qts, ns);
}

static void step_to(QTestState *qts, int64_t t)
{
    g_assert_cmpint(t, >=, now);
    step(qts, t - now);
}

static uint32_t rd(QTestState *qts, uint64_t a)
{
    return qtest_readl(qts, a);
}

static void wr(QTestState *qts, uint64_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static void reset_cause(QTestState *qts, uint32_t pro, uint32_t app)
{
    uint32_t st = rd(qts, RESET_STATE);

    g_assert_cmpuint(st & 0x3f, ==, pro);
    g_assert_cmpuint((st >> 6) & 0x3f, ==, app);
}

/*
 * Stop the flash boot protection of MWDT0 and of the RTC watchdog, which
 * would reset the chip long before any hold of 0 is reached.
 */
static void flashboot_off(QTestState *qts)
{
    wr(qts, TIMG0 + T_WDTCONFIG0, 0);
    wr(qts, WDTWPROTECT, WKEY);
    wr(qts, WDTCONFIG0, rd(qts, WDTCONFIG0) & ~RW_FLASHBOOT);
}

/* MWDT1 enabled at @now with every hold 0 and the stage actions @stg. */
static void mwdt1_zero(QTestState *qts, uint32_t stg)
{
    for (int i = 0; i < 4; i++) {
        wr(qts, TIMG1 + T_WDTCONFIG(i), 0);
    }
    wr(qts, TIMG1 + T_WDTCONFIG1, MW_PRESCALE(1));
    wr(qts, TIMG1 + T_WDTCONFIG0, MW_EN | MW_LEVEL_INT | stg);
}

/*
 * MWDT1 with every hold 0: no stage ends at the instant it begins, nor at
 * any time before its counter wraps. Stage 0 interrupts 2^32 ticks after
 * the watchdog starts, stage 1, off, ends 2^32 ticks later, and stage 2
 * interrupts after another 2^32; a feed starts stage 0 again from 0.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_mwdt_zero_int(void)
{
    QTestState *qts = start();
    int64_t t0;

    flashboot_off(qts);
    mwdt1_zero(qts, MW_STG(0, STG_INT) | MW_STG(1, STG_OFF) |
                    MW_STG(2, STG_INT) | MW_STG(3, STG_OFF));
    t0 = now;
    step(qts, 1000);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, 0);
    step_to(qts, t0 + WRAP * APB_NS - 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, MW_INT_WDT);
    wr(qts, TIMG1 + T_INT_CLR, MW_INT_WDT);
    step_to(qts, t0 + 3 * WRAP * APB_NS - 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, MW_INT_WDT);
    wr(qts, TIMG1 + T_INT_CLR, MW_INT_WDT);

    step(qts, 12345);
    wr(qts, TIMG1 + T_WDTFEED, 1);
    t0 = now;
    step_to(qts, t0 + WRAP * APB_NS - 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, MW_INT_WDT);
    qtest_quit(qts);
}

/*
 * A hold written below the count already reached: stage 0 of MWDT1 at
 * 2000 ticks when its hold goes from 0x10000 to 1000 interrupts when the
 * counter has wrapped and come to 1000.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_mwdt_hold_passed(void)
{
    QTestState *qts = start();
    int64_t t0;

    flashboot_off(qts);
    wr(qts, TIMG1 + T_WDTCONFIG(0), 0x10000);
    wr(qts, TIMG1 + T_WDTCONFIG1, MW_PRESCALE(1));
    wr(qts, TIMG1 + T_WDTCONFIG0, MW_EN | MW_LEVEL_INT | MW_STG(0, STG_INT));
    t0 = now;
    step(qts, 2000 * APB_NS);
    wr(qts, TIMG1 + T_WDTCONFIG(0), 1000);
    step_to(qts, t0 + 0x10000 * APB_NS);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, 0);
    step_to(qts, t0 + (WRAP + 1000) * APB_NS - 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, TIMG1 + T_INT_RAW), ==, MW_INT_WDT);
    qtest_quit(qts);
}

/*
 * MWDT1's resets with a hold of 0: the CPU reset (of the APP CPU, MWDT1's
 * own) and the system reset, each 2^32 ticks after the watchdog starts.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_mwdt_zero_resets(void)
{
    QTestState *qts = start();
    int64_t t0;

    flashboot_off(qts);
    mwdt1_zero(qts, MW_STG(0, STG_CPU_RESET));
    t0 = now;
    step(qts, 1000);
    reset_cause(qts, POWERON_RESET, POWERON_RESET);
    step_to(qts, t0 + WRAP * APB_NS - 1);
    reset_cause(qts, POWERON_RESET, POWERON_RESET);
    step(qts, 1);
    reset_cause(qts, POWERON_RESET, TGWDT_CPU_RESET);
    qtest_quit(qts);

    qts = start();
    flashboot_off(qts);
    mwdt1_zero(qts, MW_STG(0, STG_SYS_RESET));
    t0 = now;
    step(qts, 1000);
    reset_cause(qts, POWERON_RESET, POWERON_RESET);
    step_to(qts, t0 + WRAP * APB_NS - 1);
    reset_cause(qts, POWERON_RESET, POWERON_RESET);
    step(qts, 1);
    reset_cause(qts, TG1WDT_SYS_RESET, TG1WDT_SYS_RESET);
    qtest_quit(qts);
}

/* The RTC watchdog enabled at @now with every hold 0 and actions @stg. */
static void rwdt_zero(QTestState *qts, uint32_t stg)
{
    for (int i = 0; i < 4; i++) {
        wr(qts, WDTCONFIG(i), 0);
    }
    wr(qts, WDTCONFIG0, 0);
    wr(qts, WDTCONFIG0, RW_EN | RW_LEVEL_INT | RW_PROCPU_RESET_EN |
                        RW_APPCPU_RESET_EN | stg);
}

/*
 * The RTC watchdog with every hold 0: stage 0 interrupts 2^32 cycles of
 * RTC_SLOW_CLK after the watchdog starts; stage 1, off, ends 2^32 later;
 * stage 2 interrupts after 2^32 more.
 */
/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_rwdt_zero_int(void)
{
    QTestState *qts = start();
    int64_t t0;

    flashboot_off(qts);
    rwdt_zero(qts, RW_STG(0, STG_INT) | RW_STG(1, STG_OFF) |
                   RW_STG(2, STG_INT) | RW_STG(3, STG_OFF));
    t0 = now;
    step(qts, 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, 0);
    step_to(qts, t0 + RW_WRAP_NS - 1);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, RW_INT_WDT);
    wr(qts, INT_CLR, RW_INT_WDT);
    step(qts, RW_WRAP_NS);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, 0);
    step(qts, RW_WRAP_NS);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, RW_INT_WDT);
    qtest_quit(qts);
}

/*
 * The RTC watchdog's resets with a hold of 0, each 2^32 cycles of
 * RTC_SLOW_CLK after the watchdog starts: the CPU reset of both CPUs, the
 * system reset, which leaves the RTC domain (STORE0), and the RTC reset,
 * which does not.
 */
/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_rwdt_zero_resets(void)
{
    static const struct {
        uint32_t action;
        uint32_t pro, app;
        uint32_t store0;
    } cases[] = {
        { STG_CPU_RESET, RTCWDT_CPU_RESET, RTCWDT_CPU_RESET, 0x55aa55aa },
        { STG_SYS_RESET, RTCWDT_SYS_RESET, RTCWDT_SYS_RESET, 0x55aa55aa },
        { STG_RTC_RESET, RTCWDT_RTC_RESET, RTCWDT_RTC_RESET, 0 },
    };

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        QTestState *qts = start();
        int64_t t0;

        flashboot_off(qts);
        wr(qts, STORE0, 0x55aa55aa);
        rwdt_zero(qts, RW_STG(0, cases[i].action));
        t0 = now;
        step(qts, 1000);
        reset_cause(qts, POWERON_RESET, POWERON_RESET);
        step_to(qts, t0 + RW_WRAP_NS - 1);
        reset_cause(qts, POWERON_RESET, POWERON_RESET);
        step(qts, 1);
        reset_cause(qts, cases[i].pro, cases[i].app);
        g_assert_cmphex(rd(qts, STORE0), ==, cases[i].store0);
        qtest_quit(qts);
    }
}

/*
 * A hold written at or below the RTC watchdog's count: at 3000 cycles,
 * stage 0's hold of 0x10000 goes to 3000, reached when the counter has
 * wrapped and come to 3000 again.
 */
/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_rwdt_hold_passed(void)
{
    QTestState *qts = start();
    int64_t t0;

    flashboot_off(qts);
    wr(qts, WDTCONFIG(0), 0x10000);
    wr(qts, WDTCONFIG0, 0);
    wr(qts, WDTCONFIG0, RW_EN | RW_LEVEL_INT | RW_STG(0, STG_INT));
    t0 = now;
    step(qts, 3000 * 20000 / 3);
    wr(qts, WDTCONFIG(0), 3000);
    step_to(qts, t0 + 0x10000ll * 20000 / 3 + 1);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, 0);
    step_to(qts, t0 + RW_WRAP_NS + 3000 * 20000 / 3 - 1);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW) & RW_INT_WDT, ==, RW_INT_WDT);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/wdt/mwdt-zero-int", test_mwdt_zero_int);
    qtest_add_func("/esp32/wdt/mwdt-hold-passed", test_mwdt_hold_passed);
    qtest_add_func("/esp32/wdt/mwdt-zero-resets", test_mwdt_zero_resets);
    qtest_add_func("/esp32/wdt/rwdt-zero-int", test_rwdt_zero_int);
    qtest_add_func("/esp32/wdt/rwdt-zero-resets", test_rwdt_zero_resets);
    qtest_add_func("/esp32/wdt/rwdt-hold-passed", test_rwdt_hold_passed);
    return g_test_run();
}
