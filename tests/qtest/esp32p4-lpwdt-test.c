/*
 * QTest testcase for the ESP32-P4 RTC watchdog and super watchdog
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The RTC watchdog counts LP_DYN_SLOW_CLK, RC_SLOW_CLK's 150 kHz out of
 * reset, so 150 cycles are 1 ms; its stage 0 hold is doubled, as
 * EFUSE_WDT_DELAY_SEL is 0. The super watchdog requests a feed after
 * 860 ms and resets the system at 960 ms, as LP_SYSTEM's FIB control lets
 * it from reset; a test running longer has it feed itself or disables it.
 * System resets are awaited as QMP RESET events; interrupts are seen in
 * the interrupt matrix's status of source 1.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/misc/esp32p4_clkrst.h"

#define LP_WDT              0x50116000
#define CONFIG0             0x00
#define CONFIG(n)           (0x04 + 4 * (n))
#define FEED                0x14
#define WPROTECT            0x18
#define SWD_CONFIG          0x1c
#define SWD_WPROTECT        0x20
#define INT_RAW             0x24
#define INT_ST              0x28
#define INT_ENA             0x2c
#define INT_CLR             0x30
#define DATE                0x3fc

#define C0_PAUSE_IN_SLP     (1u << 9)
#define C0_APPCPU_RESET_EN  (1u << 10)
#define C0_PROCPU_RESET_EN  (1u << 11)
#define C0_FLASHBOOT        (1u << 12)
#define C0_STG(n, a)        ((uint32_t)(a) << (28 - 3 * (n)))
#define C0_EN               (1u << 31)
#define C0_RESET            0x00013200u
#define FEED_FEED           (1u << 31)
#define WKEY                0x50d83aa1u
#define STG_INT             1
#define STG_CPU             2
#define STG_CORE            3
#define STG_SYS             4

#define SWD_RESET_FLAG      (1u << 0)
#define SWD_AUTO_FEED       (1u << 18)
#define SWD_RST_FLAG_CLR    (1u << 19)
#define SWD_DISABLE         (1u << 30)
#define SWD_FEED            (1u << 31)
#define SWD_RESET           0x12c00000u
#define INT_SWD             (1u << 30)
#define INT_RWDT            (1u << 31)

#define LP_WDT_PATH         "/machine/lp-wdt"
#define LP_SYS_CTRL         0x50110008
#define SYS_CTRL_RESET      0x1fffc7f8u
#define DIG_FIB_SWD         (1u << 5)
#define LP_FIB_SEL_SWD      (1u << 23)
#define HP_CPU0             "/machine/hp-cluster/cpu[0]"
#define HP_CPU1             "/machine/hp-cluster/cpu[1]"
#define LP_CLKRST           0x50111000
#define RESET_CAUSE         (LP_CLKRST + 0x10)
#define HPCPU_RESET_CTRL0   (LP_CLKRST + 0x14)
#define LP_WDT_HPCORE0_RESET_EN (1u << 4)
#define CAUSE_CLEAR_ALL     0xfc000000u
#define CORE0_CAUSE(v)      (((v) >> 7) & 0x3f)
#define CORE1_CAUSE(v)      (((v) >> 14) & 0x3f)
#define LP_CAUSE(v)         ((v) & 0x3f)
#define HP_RST_EN0          0x500e60c0
#define LP_TIMER_UPDATE     0x50112010
#define LP_TIMER_BUF0_LO    0x50112014
#define INTMTX_STATUS0      0x500d6200
#define SRC_LP_WDT          1

#define US                  1000ll
#define MS                  (1000 * US)

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
    return qtest_readl(qts, LP_WDT + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, LP_WDT + off, v);
}

static void swd_write(QTestState *qts, uint32_t v)
{
    wr(qts, SWD_WPROTECT, WKEY);
    wr(qts, SWD_CONFIG, v);
    wr(qts, SWD_WPROTECT, 0);
}

static uint32_t lp_time(QTestState *qts)
{
    qtest_writel(qts, LP_TIMER_UPDATE, 1u << 28);
    return qtest_readl(qts, LP_TIMER_BUF0_LO);
}

static bool wdt_irq(QTestState *qts)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> SRC_LP_WDT) & 1;
}

/* Step until a system reset has run, as its QMP event shows. */
static void await_reset(QTestState *qts, int64_t ns)
{
    step(qts, ns);
    qtest_qmp_eventwait(qts, "RESET");
}

/* Every register at its reset value, the watchdog locked. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_reset(void)
{
    QTestState *qts = start();
    static const uint32_t holds[] = { 200000, 80000, 4095, 4095 };
    int n;

    g_assert_cmphex(rd(qts, CONFIG0), ==, C0_RESET);
    for (n = 0; n < 4; n++) {
        g_assert_cmphex(rd(qts, CONFIG(n)), ==, holds[n]);
    }
    g_assert_cmphex(rd(qts, FEED), ==, 0);
    g_assert_cmphex(rd(qts, WPROTECT), ==, 0);
    g_assert_cmphex(rd(qts, SWD_CONFIG), ==, SWD_RESET);
    g_assert_cmphex(rd(qts, SWD_WPROTECT), ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, INT_ST), ==, 0);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0);
    g_assert_cmphex(rd(qts, DATE), ==, 0x02212060);

    /* Locked, the watchdog's registers take no writes. */
    wr(qts, CONFIG0, C0_EN);
    wr(qts, CONFIG(1), 1);
    wr(qts, SWD_CONFIG, SWD_DISABLE);
    g_assert_cmphex(rd(qts, CONFIG0), ==, C0_RESET);
    g_assert_cmphex(rd(qts, CONFIG(1)), ==, 80000);
    g_assert_cmphex(rd(qts, SWD_CONFIG), ==, SWD_RESET);
    wr(qts, WPROTECT, WKEY);
    wr(qts, CONFIG(1), 1);
    g_assert_cmphex(rd(qts, CONFIG(1)), ==, 1);
    qtest_quit(qts);
}

/*
 * The RTC watchdog's stages: stage 0's hold doubled, an interrupt,
 * feeding; it pauses in sleep with PAUSE_IN_SLP.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_stages(void)
{
    QTestState *qts = start();

    wr(qts, WPROTECT, WKEY);
    wr(qts, INT_ENA, INT_RWDT);
    wr(qts, CONFIG(0), 75);
    wr(qts, CONFIG(1), 300);
    wr(qts, CONFIG0, C0_PAUSE_IN_SLP | C0_STG(0, STG_INT) |
                     C0_STG(1, STG_INT) | C0_EN);
    step(qts, 1 * MS - 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RWDT);
    g_assert_cmphex(rd(qts, INT_ST), ==, INT_RWDT);
    g_assert(wdt_irq(qts));
    wr(qts, INT_CLR, INT_RWDT);
    g_assert(!wdt_irq(qts));
    step(qts, 2 * MS - 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RWDT);
    wr(qts, INT_RAW, INT_RWDT);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);

    /* Feeding restarts stage 0. */
    wr(qts, FEED, FEED_FEED);
    step(qts, 900 * US);
    wr(qts, FEED, FEED_FEED);
    step(qts, 900 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 100 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RWDT);
    wr(qts, INT_CLR, INT_RWDT);

    /* Asleep it pauses, with PAUSE_IN_SLP; without, it runs on. */
    wr(qts, FEED, FEED_FEED);
    qtest_set_irq_in(qts, LP_WDT_PATH, "sleep", 0, 1);
    step(qts, 5 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    qtest_set_irq_in(qts, LP_WDT_PATH, "sleep", 0, 0);
    step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RWDT);
    wr(qts, INT_CLR, INT_RWDT);
    wr(qts, FEED, FEED_FEED);
    wr(qts, CONFIG0, rd(qts, CONFIG0) & ~C0_PAUSE_IN_SLP);
    qtest_set_irq_in(qts, LP_WDT_PATH, "sleep", 0, 1);
    step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RWDT);
    qtest_quit(qts);
}

/*
 * The RTC watchdog's CPU reset (cause 0x0D), of the cores its
 * *_RESET_EN and LP_CLKRST's LP_WDT_HPCORE<n>_RESET_EN let it reach, and
 * its HP core reset (cause 0x09), which leaves the LP system running.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_resets(void)
{
    QTestState *qts = start();
    uint32_t cause, before;

    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    wr(qts, WPROTECT, WKEY);
    wr(qts, CONFIG(0), 75);
    wr(qts, CONFIG0, C0_PROCPU_RESET_EN | C0_STG(0, STG_CPU) | C0_EN);
    step(qts, 1 * MS);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);

    qtest_writel(qts, HPCPU_RESET_CTRL0,
                 qtest_readl(qts, HPCPU_RESET_CTRL0) |
                 LP_WDT_HPCORE0_RESET_EN);
    wr(qts, FEED, FEED_FEED);
    step(qts, 1 * MS);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    cause = qtest_readl(qts, RESET_CAUSE);
    g_assert_cmphex(CORE0_CAUSE(cause), ==, 0x0d);
    g_assert_cmphex(CORE1_CAUSE(cause), ==, 0);
    /* LP_CLKRST's reset length, a cycle of LP_DYN_FAST_CLK (20 MHz). */
    step(qts, 50);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));

    /* The HP core reset; the RTC watchdog, an LP block, runs on. */
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    qtest_writel(qts, HP_RST_EN0, 0);
    before = lp_time(qts);
    wr(qts, CONFIG0, C0_STG(0, STG_CORE) | C0_EN);
    wr(qts, FEED, FEED_FEED);
    step(qts, 1 * MS);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    cause = qtest_readl(qts, RESET_CAUSE);
    g_assert_cmphex(CORE0_CAUSE(cause), ==, 0x09);
    g_assert_cmphex(CORE1_CAUSE(cause), ==, 0x09);
    g_assert_cmphex(LP_CAUSE(cause), ==, 0);
    step(qts, 100);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmphex(rd(qts, CONFIG0), ==, C0_STG(0, STG_CORE) | C0_EN);
    g_assert_cmpuint(lp_time(qts) - before, ==, 150);
    qtest_quit(qts);
}

/*
 * The RTC watchdog's system reset, cause 0x10 for every core: the LP_WDT
 * returns to its reset state; the RTC timer counts on through it.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_system_reset(void)
{
    QTestState *qts = start();
    uint32_t cause, before;

    step(qts, 10 * MS);
    before = lp_time(qts);
    wr(qts, WPROTECT, WKEY);
    wr(qts, CONFIG(0), 75);
    wr(qts, CONFIG0, C0_STG(0, STG_SYS) | C0_EN);
    await_reset(qts, 1 * MS);
    cause = qtest_readl(qts, RESET_CAUSE);
    g_assert_cmphex(CORE0_CAUSE(cause), ==, 0x10);
    g_assert_cmphex(CORE1_CAUSE(cause), ==, 0x10);
    g_assert_cmphex(LP_CAUSE(cause), ==, 0x10);
    g_assert_cmphex(rd(qts, CONFIG0), ==, C0_RESET);
    g_assert_cmphex(rd(qts, WPROTECT), ==, 0);
    g_assert_cmpuint(lp_time(qts) - before, ==, 150);
    qtest_quit(qts);
}

/*
 * The super watchdog. From reset LP_SYSTEM's FIB control leaves its reset
 * to the analog state, which enables it: unfed, it requests a feed at
 * 860 ms and resets the system at 960 ms (cause 0x12), setting
 * SWD_RESET_FLAG, which that reset keeps and SWD_RST_FLAG_CLR clears
 * without the key. With the FIB's software bit picked and clear, the
 * timeout starts it again instead. Fed, it does not reset; feeding
 * itself, it requests and never resets; disabled, it does neither.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_swd(void)
{
    QTestState *qts = start();

    wr(qts, INT_ENA, INT_SWD);
    step(qts, 860 * MS - 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_SWD);
    g_assert(wdt_irq(qts));
    step(qts, 100 * MS - 1);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x01);
    await_reset(qts, 1);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x12);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x12);
    g_assert_cmphex(rd(qts, SWD_CONFIG), ==, SWD_RESET | SWD_RESET_FLAG);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    wr(qts, SWD_CONFIG, SWD_RESET | SWD_RST_FLAG_CLR);
    g_assert_cmphex(rd(qts, SWD_CONFIG), ==, SWD_RESET);

    /* Its reset disabled, the timeout starts it again. */
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    qtest_writel(qts, LP_SYS_CTRL,
                 SYS_CTRL_RESET & ~(DIG_FIB_SWD | LP_FIB_SEL_SWD));
    step(qts, 960 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_SWD);
    wr(qts, INT_CLR, INT_SWD);
    step(qts, 860 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_SWD);
    wr(qts, INT_CLR, INT_SWD);

    /* Its reset enabled again, feeding keeps it off. */
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET);
    swd_write(qts, SWD_RESET | SWD_FEED);
    step(qts, 959 * MS);
    swd_write(qts, SWD_RESET | SWD_FEED);
    step(qts, 959 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);

    /* Feeding itself, it requests and never resets. */
    swd_write(qts, SWD_RESET | SWD_AUTO_FEED | SWD_FEED);
    wr(qts, INT_CLR, INT_SWD);
    step(qts, 3000 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_SWD);

    /* Disabled, it neither requests nor resets. */
    swd_write(qts, SWD_DISABLE);
    wr(qts, INT_CLR, INT_SWD);
    step(qts, 3000 * MS);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    qtest_quit(qts);
}

/*
 * Flash boot protection: booted in SPI boot mode, the RTC watchdog runs
 * with RTC_WDT_EN clear and resets the system after stage 0's 200000 << 1
 * cycles; clearing FLASHBOOT_MOD_EN stops it.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_flashboot(void)
{
    QTestState *qts = start();

    swd_write(qts, SWD_DISABLE);
    qtest_set_irq_in(qts, LP_WDT_PATH, "spi-boot", 0, 1);
    step(qts, 2666 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x01);
    await_reset(qts, 1 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x10);

    swd_write(qts, SWD_DISABLE);
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    wr(qts, WPROTECT, WKEY);
    wr(qts, CONFIG0, C0_RESET & ~C0_FLASHBOOT);
    step(qts, 3000 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    qtest_quit(qts);
}

/*
 * Strapped for SPI boot, as by default, and booting through the ROM (no
 * -kernel), MWDT0's flash boot protection resets the HP core every
 * 650 ms and the RTC watchdog's resets the system after 2.67 s. The super
 * watchdog feeds itself, as ESP-IDF's bootloader has it.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_spi_boot_strap(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    now = 0;
    swd_write(qts, SWD_RESET | SWD_AUTO_FEED);
    step(qts, 650 * MS - 1);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x01);
    step(qts, 1);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x07);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x01);
    step(qts, 2666 * MS - now);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x01);
    await_reset(qts, 1 * MS);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x10);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/lp-wdt/reset", test_reset);
    qtest_add_func("/esp32p4/lp-wdt/stages", test_stages);
    qtest_add_func("/esp32p4/lp-wdt/resets", test_resets);
    qtest_add_func("/esp32p4/lp-wdt/system-reset", test_system_reset);
    qtest_add_func("/esp32p4/lp-wdt/swd", test_swd);
    qtest_add_func("/esp32p4/lp-wdt/flashboot", test_flashboot);
    qtest_add_func("/esp32p4/lp-wdt/spi-boot-strap", test_spi_boot_strap);
    return g_test_run();
}
