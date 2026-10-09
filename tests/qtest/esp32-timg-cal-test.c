/*
 * QTest testcase for the ESP32 timer groups' RTC slow clock calibration
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RTC_CNTL                0x3ff48000
#define TIME_UPDATE             (RTC_CNTL + 0x0c)
#define CLK_CONF                (RTC_CNTL + 0x70)
#define RTC_WDTCONFIG0          (RTC_CNTL + 0x8c)
#define RTC_WDTWPROTECT         (RTC_CNTL + 0xa4)
#define XTAL_32K_PAD            (0x3ff48400 + 0x8c)

#define TIMG0                   0x3ff5f000
#define TIMG1                   0x3ff60000
#define WDTCONFIG0(g)           ((g) + 0x48)
#define WDTPROTECT(g)           ((g) + 0x64)
#define RTCCALICFG(g)           ((g) + 0x68)
#define RTCCALICFG1(g)          ((g) + 0x6c)

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define DPORT_TIMG0_CLK_EN      (1u << 13)

#define CALI_START              (1u << 31)
#define CALI_MAX(n)             ((uint32_t)(n) << 16)
#define CALI_RDY                (1u << 15)
#define CALI_CLK_SEL(n)         ((uint32_t)(n) << 13)
#define CALI_START_CYCLING      (1u << 12)
#define CALI_VALUE(v)           ((v) >> 7)

#define SEL_RTC_MUX             0
#define SEL_8MD256              1
#define SEL_32K_XTAL            2

#define ANA_CLK_RTC_SEL(n)      ((uint32_t)(n) << 30)
#define ANA_CLK_RTC_SEL_MASK    (3u << 30)
#define DIG_CLK8M_D256_EN       (1u << 9)
#define DIG_XTAL32K_EN          (1u << 8)
#define ENB_CK8M_DIV            (1u << 7)
#define ENB_CK8M                (1u << 6)
#define CK8M_DIV_MASK           (3u << 4)
#define XPD_XTAL_32K            (1u << 19)
#define TIME_UPDATE_BIT         (1u << 31)
#define TIME_VALID              (1u << 30)

#define WKEY                    0x50d83aa1
#define RTC_WDT_FLASHBOOT       (1u << 10)

#define XTAL_HZ                 40000000ull
#define NS_PER_S                1000000000ull

static QTestState *start(bool xtal32k)
{
    QTestState *qts = qtest_init(xtal32k ?
                                 "-M esp32 -nic none -global "
                                 "driver=misc.esp32.rtc_cntl,"
                                 "property=xtal32k,value=on" :
                                 "-M esp32 -nic none");

    /* Keep the boot watchdogs from resetting the chip during long waits */
    qtest_writel(qts, RTC_WDTWPROTECT, WKEY);
    qtest_writel(qts, RTC_WDTCONFIG0,
                 qtest_readl(qts, RTC_WDTCONFIG0) & ~RTC_WDT_FLASHBOOT);
    qtest_writel(qts, RTC_WDTWPROTECT, 0);
    qtest_writel(qts, WDTPROTECT(TIMG0), WKEY);
    qtest_writel(qts, WDTCONFIG0(TIMG0), 0);
    qtest_writel(qts, WDTPROTECT(TIMG0), 0);
    return qts;
}

/* Expected RTC_CALI_VALUE for max cycles of a clock of hz */
static uint64_t xtal_cycles(uint32_t max, uint64_t hz)
{
    return max * XTAL_HZ / hz;
}

/* Time a measurement of max cycles of a clock of hz takes, plus one cycle */
static uint64_t cal_ns(uint32_t max, uint64_t hz)
{
    return (max + 1) * NS_PER_S / hz + 1;
}

/* ESP-IDF's sequence: stop cycling, set the clock and count, pulse START */
static void cal_start(QTestState *qts, uint32_t sel, uint32_t max)
{
    uint32_t v = CALI_CLK_SEL(sel) | CALI_MAX(max);

    qtest_writel(qts, RTCCALICFG(TIMG0), v);
    qtest_writel(qts, RTCCALICFG(TIMG0), v | CALI_START);
}

static void assert_value(QTestState *qts, uint64_t expected)
{
    uint64_t v = CALI_VALUE(qtest_readl(qts, RTCCALICFG1(TIMG0)));

    g_assert_cmpuint(v + 1, >=, expected);
    g_assert_cmpuint(v, <=, expected + 1);
}

static void set_slow_clk(QTestState *qts, uint32_t sel)
{
    uint32_t conf = qtest_readl(qts, CLK_CONF) & ~ANA_CLK_RTC_SEL_MASK;

    qtest_writel(qts, CLK_CONF, conf | ANA_CLK_RTC_SEL(sel));
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * RTCCALICFG resets in cycling mode measuring one 8MD256 cycle; RDY comes
 * up once the first measurement has finished, and is read-only.
 */
static void test_reset_cycling(void)
{
    QTestState *qts = start(false);

    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG1)), ==,
                    CALI_MAX(1) | CALI_CLK_SEL(SEL_8MD256) |
                    CALI_START_CYCLING);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG1(TIMG1)), ==, 0);
    qtest_clock_step(qts, 100 * 1000);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG1)), ==,
                    CALI_MAX(1) | CALI_CLK_SEL(SEL_8MD256) |
                    CALI_START_CYCLING | CALI_RDY);
    g_assert_cmpuint(CALI_VALUE(qtest_readl(qts, RTCCALICFG1(TIMG1))), ==,
                     xtal_cycles(1, 31250));

    /* RDY is read-only: a oneshot drops it until the count is reached */
    qtest_writel(qts, RTCCALICFG(TIMG0), CALI_MAX(100) |
                 CALI_CLK_SEL(SEL_8MD256) | CALI_RDY);
    qtest_writel(qts, RTCCALICFG(TIMG0), CALI_MAX(100) |
                 CALI_CLK_SEL(SEL_8MD256) | CALI_RDY | CALI_START);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * [spec:nuos:req:emu.esp32.clock-gating/test]
 * Each source measured against XTAL_CLK: the count matches the source's
 * frequency, and RDY comes up only after the measurement's duration.
 */
static void test_sources(void)
{
    QTestState *qts = start(false);

    /* RTC_SLOW_CLK from the 150 kHz RC oscillator */
    cal_start(qts, SEL_RTC_MUX, 1024);
    qtest_clock_step(qts, 1024 * NS_PER_S / 150000 - 1000);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    qtest_clock_step(qts, 2 * NS_PER_S / 150000 + 1000);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(1024, 150000));

    cal_start(qts, SEL_8MD256, 100);
    qtest_clock_step(qts, cal_ns(100, 31250));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(100, 31250));

    /* RTC_SLOW_CLK switched to 8MD256: the mux follows the selection */
    set_slow_clk(qts, 2);
    cal_start(qts, SEL_RTC_MUX, 100);
    qtest_clock_step(qts, cal_ns(100, 31250));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(100, 31250));
    set_slow_clk(qts, 0);

    /* CK8M_DIV = 0 divides RC_FAST_CLK by 128 */
    qtest_writel(qts, CLK_CONF, qtest_readl(qts, CLK_CONF) & ~CK8M_DIV_MASK);
    cal_start(qts, SEL_8MD256, 100);
    qtest_clock_step(qts, cal_ns(100, 62500));
    assert_value(qts, xtal_cycles(100, 62500));

    /* Without DIG_CLK8M_D256_EN the digital domain sees no 8MD256 */
    qtest_writel(qts, CLK_CONF,
                 qtest_readl(qts, CLK_CONF) & ~DIG_CLK8M_D256_EN);
    cal_start(qts, SEL_8MD256, 10);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    /* ...and the measurement goes on once it does */
    qtest_writel(qts, CLK_CONF,
                 qtest_readl(qts, CLK_CONF) | DIG_CLK8M_D256_EN);
    qtest_clock_step(qts, cal_ns(10, 62500));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(10, 62500));

    /* ENB_CK8M stops the oscillator */
    qtest_writel(qts, CLK_CONF, qtest_readl(qts, CLK_CONF) | ENB_CK8M);
    cal_start(qts, SEL_8MD256, 10);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * RTC_CALI_MAX = 0 waits for the next RTC_SLOW_CLK edge, as ESP-IDF's
 * rtc_clk_wait_for_slow_cycle relies on.
 */
static void test_wait_for_slow_cycle(void)
{
    QTestState *qts = start(false);

    qtest_writel(qts, RTCCALICFG(TIMG0), CALI_CLK_SEL(SEL_RTC_MUX));
    qtest_writel(qts, RTCCALICFG(TIMG0),
                 CALI_CLK_SEL(SEL_RTC_MUX) | CALI_START);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    qtest_clock_step(qts, NS_PER_S / 150000 + 1);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    g_assert_cmpuint(qtest_readl(qts, RTCCALICFG1(TIMG0)), ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * No 32 kHz crystal fitted, as on the ESP32-WROOM-32E: XTAL32K_CLK never
 * runs, so a measurement of it never completes and software times out.
 * Selected as RTC_SLOW_CLK it stops the slow clock, and the RTC timer.
 */
static void test_no_xtal32k(void)
{
    QTestState *qts = start(false);

    qtest_writel(qts, XTAL_32K_PAD,
                 qtest_readl(qts, XTAL_32K_PAD) | XPD_XTAL_32K);
    qtest_writel(qts, CLK_CONF, qtest_readl(qts, CLK_CONF) | DIG_XTAL32K_EN);
    cal_start(qts, SEL_32K_XTAL, 1024);
    qtest_clock_step(qts, NS_PER_S);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);

    set_slow_clk(qts, 1);
    cal_start(qts, SEL_RTC_MUX, 1024);
    qtest_writel(qts, TIME_UPDATE, TIME_UPDATE_BIT);
    qtest_clock_step(qts, NS_PER_S);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, TIME_UPDATE) & TIME_VALID, ==, 0);

    /* Back on the RC oscillator, both complete */
    set_slow_clk(qts, 0);
    qtest_clock_step(qts, cal_ns(1024, 150000));
    g_assert_cmphex(qtest_readl(qts, TIME_UPDATE) & TIME_VALID, ==,
                    TIME_VALID);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * With a crystal fitted XTAL32K_CLK runs once RTCIO powers the oscillator
 * up, and the timer group sees it behind DIG_XTAL32K_EN.
 */
static void test_xtal32k_fitted(void)
{
    QTestState *qts = start(true);

    qtest_writel(qts, CLK_CONF, qtest_readl(qts, CLK_CONF) | DIG_XTAL32K_EN);
    cal_start(qts, SEL_32K_XTAL, 1024);
    qtest_clock_step(qts, cal_ns(1024, 32768));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);

    qtest_writel(qts, XTAL_32K_PAD,
                 qtest_readl(qts, XTAL_32K_PAD) | XPD_XTAL_32K);
    qtest_clock_step(qts, cal_ns(1024, 32768));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(1024, 32768));

    /* As RTC_SLOW_CLK, through the mux, without the digital enable */
    qtest_writel(qts, CLK_CONF,
                 qtest_readl(qts, CLK_CONF) & ~DIG_XTAL32K_EN);
    set_slow_clk(qts, 1);
    cal_start(qts, SEL_RTC_MUX, 100);
    qtest_clock_step(qts, cal_ns(100, 32768));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(100, 32768));

    cal_start(qts, SEL_32K_XTAL, 100);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating/test]
 * Nothing counts while DPORT gates the group's clock.
 */
static void test_gated(void)
{
    QTestState *qts = start(false);
    uint32_t en = qtest_readl(qts, DPORT_PERIP_CLK_EN);

    cal_start(qts, SEL_8MD256, 100);
    qtest_clock_step(qts, 1000 * 1000);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, en & ~DPORT_TIMG0_CLK_EN);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, en | DPORT_TIMG0_CLK_EN);
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==, 0);
    qtest_clock_step(qts, cal_ns(100, 31250));
    g_assert_cmphex(qtest_readl(qts, RTCCALICFG(TIMG0)) & CALI_RDY, ==,
                    CALI_RDY);
    assert_value(qts, xtal_cycles(100, 31250));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32-timg-cal/reset-cycling", test_reset_cycling);
    qtest_add_func("esp32-timg-cal/sources", test_sources);
    qtest_add_func("esp32-timg-cal/wait-for-slow-cycle",
                   test_wait_for_slow_cycle);
    qtest_add_func("esp32-timg-cal/no-xtal32k", test_no_xtal32k);
    qtest_add_func("esp32-timg-cal/xtal32k-fitted", test_xtal32k_fitted);
    qtest_add_func("esp32-timg-cal/gated", test_gated);

    return g_test_run();
}
