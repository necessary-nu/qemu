/*
 * QTest testcase for the ESP32-P4 RTC timer (LP_TIMER)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The timer counts LP_DYN_SLOW_CLK, RC_SLOW_CLK's 150 kHz out of reset,
 * from the machine's reset at virtual time 0: count 150 comes at 1 ms.
 * Interrupts are seen in the interrupt matrix's status of sources 2
 * (LP_TIMER_REG_0) and 3 (LP_TIMER_REG_1).
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/misc/esp32p4_clkrst.h"

#define LP_TIMER            0x50112000
#define TAR_LOW(n)          (0x00 + 8 * (n))
#define TAR_HIGH(n)         (0x04 + 8 * (n))
#define UPDATE              0x10
#define BUF0_LOW            0x14
#define BUF0_HIGH           0x18
#define BUF1_LOW            0x1c
#define BUF1_HIGH           0x20
#define MAIN_OVERFLOW       0x24
#define INT_RAW             0x28
#define INT_ST              0x2c
#define INT_ENA             0x30
#define INT_CLR             0x34
#define LP_INT_RAW          0x38
#define LP_INT_ST           0x3c
#define LP_INT_ENA          0x40
#define LP_INT_CLR          0x44
#define DATE                0x3fc

#define TAR_EN              (1u << 31)
#define UPDATE_UPDATE       (1u << 28)
#define UPDATE_XTAL_OFF     (1u << 29)
#define UPDATE_SYS_STALL    (1u << 30)
#define UPDATE_SYS_RST      (1u << 31)
#define INT_OVERFLOW        (1u << 30)
#define INT_TARGET          (1u << 31)

#define LP_TIMER_PATH       "/machine/lp-timer"
#define CLKRST              "/machine/clkrst"
#define LP_CLKRST           0x50111000
#define LP_CLK_CONF         (LP_CLKRST + 0x00)
#define LP_RST_EN           (LP_CLKRST + 0x0c)
#define RST_EN_LP_TIMER     (1u << 27)
#define TIMG(g)             (0x500c2000 + 0x1000 * (g))
#define TIMG_WDTCONFIG0     0x48
#define TIMG_WDTCONFIG1     0x4c
#define TIMG_WDTCONFIG2     0x50
#define TIMG_RTCCALICFG     0x68
#define LP_WDT              0x50116000
#define LP_WDT_CONFIG0      0x00
#define LP_WDT_CONFIG1      0x04
#define LP_WDT_WPROTECT     0x18
#define LP_WDT_SWD_CONFIG   0x1c
#define LP_WDT_SWD_WPROTECT 0x20
#define WKEY                0x50d83aa1u
#define INTMTX_STATUS0      0x500d6200

#define US                  1000ll
#define MS                  (1000 * US)
#define S                   (1000 * MS)

static int64_t now;

/*
 * A board strapped for Joint Download Boot (GPIO36 high, GPIO35 low), so
 * that the watchdogs' flash boot protection does not run.
 */
static QTestState *start(void)
{
    now = 0;
    return qtest_init("-M esp32p4,strap-mode=0x04");
}

static void step(QTestState *qts, int64_t ns)
{
    now = qtest_clock_step(qts, ns);
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, LP_TIMER + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, LP_TIMER + off, v);
}

static uint64_t buf(QTestState *qts, int n)
{
    return ((uint64_t)rd(qts, BUF0_HIGH + 8 * n) << 32) |
           rd(qts, BUF0_LOW + 8 * n);
}

/* Log the count and read it. */
static uint64_t count(QTestState *qts)
{
    wr(qts, UPDATE, rd(qts, UPDATE) | UPDATE_UPDATE);
    return buf(qts, 0);
}

static void set_target(QTestState *qts, int n, uint64_t v, bool enable)
{
    wr(qts, TAR_LOW(n), (uint32_t)v);
    wr(qts, TAR_HIGH(n), (v >> 32) | (enable ? TAR_EN : 0));
}

static bool source(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> src) & 1;
}

/* Every register at its reset value. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_reset(void)
{
    QTestState *qts = start();
    uint32_t off;

    for (off = 0; off < 0x48; off += 4) {
        g_assert_cmphex(rd(qts, off), ==, 0);
    }
    g_assert_cmphex(rd(qts, DATE), ==, 0x02111150);
    qtest_quit(qts);
}

/*
 * The count, logged by MAIN_TIMER_UPDATE into BUF0 with the last log
 * moving to BUF1; from RC32K_CLK when LP_SLOW_CLK selects it.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_counting(void)
{
    QTestState *qts = start();

    step(qts, 1 * MS - 1);
    g_assert_cmpuint(count(qts), ==, 149);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 150);
    g_assert_cmpuint(buf(qts, 1), ==, 149);
    step(qts, 1 * S);
    g_assert_cmpuint(count(qts), ==, 150150);
    g_assert_cmpuint(buf(qts, 1), ==, 150);
    /* The logs hold until the next. */
    step(qts, 1 * S);
    g_assert_cmpuint(buf(qts, 0), ==, 150150);

    /* LP_SLOW_CLK from RC32K_CLK, powered by the PMU: 32768 Hz. */
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_RC32K, 1);
    qtest_writel(qts, LP_CLK_CONF, (qtest_readl(qts, LP_CLK_CONF) & ~3u) | 2);
    g_assert_cmpuint(count(qts), ==, 300150);
    step(qts, 1 * S);
    g_assert_cmpuint(count(qts), ==, 300150 + 32768);
    qtest_quit(qts);
}

/*
 * Target 0 sets SOC_WAKEUP_INT_RAW and drives LP_TIMER_REG_0; target 1
 * sets MAIN_TIMER_LP_INT_RAW and drives LP_TIMER_REG_1; each fires once,
 * at once if the count equals it already, and not for a target passed.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_targets(void)
{
    QTestState *qts = start();

    wr(qts, INT_ENA, INT_TARGET);
    wr(qts, LP_INT_ENA, INT_TARGET);
    set_target(qts, 0, 300, true);
    set_target(qts, 1, 450, true);
    g_assert_cmphex(rd(qts, TAR_HIGH(0)), ==, 0);
    g_assert_cmphex(rd(qts, TAR_LOW(1)), ==, 450);
    step(qts, 2 * MS - 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert(!source(qts, 2));
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TARGET);
    g_assert_cmphex(rd(qts, INT_ST), ==, INT_TARGET);
    g_assert(source(qts, 2));
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 0);
    step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, INT_TARGET);
    g_assert_cmphex(rd(qts, LP_INT_ST), ==, INT_TARGET);
    g_assert(source(qts, 3));
    wr(qts, INT_CLR, INT_TARGET);
    wr(qts, LP_INT_RAW, INT_TARGET);
    g_assert(!source(qts, 2));
    g_assert(!source(qts, 3));

    /* A target is disabled once it fires. */
    step(qts, 10 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);

    /* Writing EN as 0 changes nothing; the target's value can change. */
    set_target(qts, 0, count(qts) + 150, true);
    set_target(qts, 0, count(qts) + 300, false);
    step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TARGET);
    wr(qts, INT_CLR, INT_TARGET);

    /* Equal already, it fires at once; passed, it does not. */
    set_target(qts, 0, count(qts), true);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TARGET);
    wr(qts, INT_CLR, INT_TARGET);
    set_target(qts, 1, count(qts) - 1, true);
    step(qts, 100 * MS);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 0);
    qtest_quit(qts);
}

/*
 * The 48-bit counter's overflow sets OVERFLOW_RAW and
 * MAIN_TIMER_OVERFLOW_LP_INT_RAW, after 2^48 cycles, some 59 years; the
 * blocks that run periodically, the calibration units and the super
 * watchdog, are stopped so that virtual time can get there.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_overflow(void)
{
    QTestState *qts = start();
    int64_t ovf = (int64_t)((1ull << 48) * (1e9 / 150000.0));

    qtest_writel(qts, TIMG(0) + TIMG_RTCCALICFG, 1u << 16);
    qtest_writel(qts, TIMG(1) + TIMG_RTCCALICFG, 1u << 16);
    qtest_writel(qts, LP_WDT + LP_WDT_SWD_WPROTECT, WKEY);
    qtest_writel(qts, LP_WDT + LP_WDT_SWD_CONFIG, 1u << 30);
    wr(qts, INT_ENA, INT_OVERFLOW);
    wr(qts, LP_INT_ENA, INT_OVERFLOW);
    step(qts, ovf - 1 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmphex(count(qts) >> 40, ==, 0xff);
    step(qts, 2 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_OVERFLOW);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, INT_OVERFLOW);
    g_assert(source(qts, 2));
    g_assert(source(qts, 3));
    g_assert_cmpuint(count(qts), <, 300);
    qtest_quit(qts);
}

/*
 * Logging the time of events: an HP core reset and a system reset with
 * MAIN_TIMER_SYS_RST, the selected core's stall with
 * MAIN_TIMER_SYS_STALL, the crystal powering down with
 * MAIN_TIMER_XTAL_OFF. The timer counts on through both resets; its own
 * reset in LP_CLKRST resets it.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_logging(void)
{
    QTestState *qts = start();

    /* An HP core reset at 1 ms, from MWDT0. */
    wr(qts, UPDATE, UPDATE_SYS_RST);
    qtest_writel(qts, TIMG(0) + TIMG_WDTCONFIG1, 40u << 16);
    qtest_writel(qts, TIMG(0) + TIMG_WDTCONFIG2, 1000);
    qtest_writel(qts, TIMG(0) + TIMG_WDTCONFIG0,
                 (1u << 31) | (3u << 29) | (1u << 22));
    step(qts, 1 * MS);
    g_assert_cmpuint(buf(qts, 0), ==, 150);
    step(qts, 1 * MS);
    g_assert_cmpuint(count(qts), ==, 300);

    /* A system reset at 3 ms, from the RTC watchdog. */
    qtest_writel(qts, LP_WDT + LP_WDT_WPROTECT, WKEY);
    qtest_writel(qts, LP_WDT + LP_WDT_CONFIG1, 75);
    qtest_writel(qts, LP_WDT + LP_WDT_CONFIG0, (1u << 31) | (4u << 28));
    step(qts, 1 * MS);
    qtest_qmp_eventwait(qts, "RESET");
    g_assert_cmpuint(buf(qts, 0), ==, 450);
    g_assert_cmpuint(buf(qts, 1), ==, 300);
    g_assert_cmphex(rd(qts, UPDATE), ==, UPDATE_SYS_RST);
    step(qts, 1 * MS);
    g_assert_cmpuint(count(qts), ==, 600);

    /* Core 0's stall starting and ending, and not core 1's. */
    wr(qts, UPDATE, UPDATE_SYS_STALL);
    step(qts, 1 * MS);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 0, 1);
    g_assert_cmpuint(buf(qts, 0), ==, 750);
    step(qts, 1 * MS);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 0, 0);
    g_assert_cmpuint(buf(qts, 0), ==, 900);
    step(qts, 1 * MS);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 1);
    g_assert_cmpuint(buf(qts, 0), ==, 900);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 0);
    /* LP_SYSTEM_SYSTIMER_STALL_SEL picks core 1. */
    qtest_set_irq_in(qts, LP_TIMER_PATH, "stall-sel", 0, 1);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 1);
    g_assert_cmpuint(buf(qts, 0), ==, 1050);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 0);

    /* The crystal powering down and up. */
    wr(qts, UPDATE, UPDATE_XTAL_OFF);
    step(qts, 1 * MS);
    qtest_set_irq_in(qts, LP_TIMER_PATH, "xtal-off", 0, 1);
    g_assert_cmpuint(buf(qts, 0), ==, 1200);
    wr(qts, UPDATE, 0);
    step(qts, 1 * MS);
    qtest_set_irq_in(qts, LP_TIMER_PATH, "xtal-off", 0, 0);
    g_assert_cmpuint(buf(qts, 0), ==, 1200);

    /* The block's reset resets it. */
    qtest_writel(qts, LP_RST_EN, RST_EN_LP_TIMER);
    qtest_writel(qts, LP_RST_EN, 0);
    step(qts, 1 * MS);
    g_assert_cmpuint(count(qts), ==, 150);

    /* MAIN_TIMER_ALARM_LOAD is not documented, and changes nothing. */
    wr(qts, MAIN_OVERFLOW, 1u << 31);
    g_assert_cmphex(rd(qts, MAIN_OVERFLOW), ==, 0);
    qtest_quit(qts);
}

/*
 * RC_SLOW_CLK's period is 6666 2/3 ns: count n comes at exactly n * 20000/3
 * ns, the first nanosecond at or after it, however the count was folded
 * on the way. Each write of a target folds the count; one at a fraction of
 * a nanosecond past an edge must not move the edges after it. Count 2
 * comes at 13333 1/3 ns, count 4 at 26666 2/3 ns and count 150000, and
 * the target, at 1 s.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_exact_cycles(void)
{
    QTestState *qts = start();

    wr(qts, INT_ENA, INT_TARGET);
    set_target(qts, 0, 150000, true);
    step(qts, 13334);
    g_assert_cmpuint(count(qts), ==, 2);
    set_target(qts, 0, 150000, true);
    step(qts, 26666 - now);
    g_assert_cmpuint(count(qts), ==, 3);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 4);
    set_target(qts, 0, 150000, true);
    step(qts, 40001 - now);
    g_assert_cmpuint(count(qts), ==, 6);
    set_target(qts, 0, 150000, true);
    step(qts, 1 * S - 1 - now);
    g_assert_cmpuint(count(qts), ==, 149999);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 150000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TARGET);
    qtest_quit(qts);
}

/*
 * Two 32768 Hz oscillators of one period, 1/32768 s, at different phases:
 * RC32K powered up at 1000 ns, XTAL32K at 11000 ns, each with its first
 * edge there. LP_SLOW_CLK switched from RC32K to XTAL32K in one write
 * moves the timer's counting to XTAL32K's edges. Switched at 1 ms, it
 * counts the next one, at 11000 + 33 periods, 1018080.078125 ns, as the
 * next count, and the 100th, at 11000 + 132 periods, 4039320.3125 ns, as
 * the target 100 on: its alarm comes at 4039321 ns. At RC32K's edges it
 * would come at 4029321 ns, and at edges counted from the switch to
 * RC32K at 20000 ns, 4048321 ns.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_clock_phase(void)
{
    QTestState *qts = start();
    uint64_t c;

    step(qts, 1000);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_RC32K, 1);
    step(qts, 10000);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_XTAL32K, 1);
    step(qts, 9000);
    qtest_writel(qts, LP_CLK_CONF, (qtest_readl(qts, LP_CLK_CONF) & ~3u) | 2);
    step(qts, 1 * MS - now);
    qtest_writel(qts, LP_CLK_CONF, (qtest_readl(qts, LP_CLK_CONF) & ~3u) | 1);
    c = count(qts);
    wr(qts, INT_ENA, INT_TARGET);
    set_target(qts, 0, c + 100, true);
    step(qts, 1018080 - now);
    g_assert_cmpuint(count(qts), ==, c);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, c + 1);
    step(qts, 4039320 - now);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TARGET);
    g_assert_cmpuint(count(qts), ==, c + 100);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/lp-timer/reset", test_reset);
    qtest_add_func("/esp32p4/lp-timer/counting", test_counting);
    qtest_add_func("/esp32p4/lp-timer/targets", test_targets);
    qtest_add_func("/esp32p4/lp-timer/overflow", test_overflow);
    qtest_add_func("/esp32p4/lp-timer/logging", test_logging);
    qtest_add_func("/esp32p4/lp-timer/exact-cycles", test_exact_cycles);
    qtest_add_func("/esp32p4/lp-timer/clock-phase", test_clock_phase);
    return g_test_run();
}
