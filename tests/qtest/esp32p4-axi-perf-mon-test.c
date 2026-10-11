/*
 * QTest testcase for the ESP32-P4 AXI performance monitor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * No block reports its AXI transactions to the monitor, so the ports it
 * watches are idle: its instantaneous bandwidth windows, timed on AXI_CLK
 * (SYS_CLK), measure no beats, which the range statistics count and the
 * under-limit interrupt (source 130, seen in the interrupt matrix's
 * status) reports.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define APM                 0x500a8000
#define APM_PATH            "/machine/axi-perf-mon"
#define CLK_EN              (APM + 0x000)
#define AGENT_SELECT        (APM + 0x004)
#define COUNTER(c, n)       (APM + 0x008 + 8 * (c) + 4 * (n))
#define RANGE(m, n)         (APM + 0x038 + 8 * (m) + 4 * (n))
#define RD_INFO(x, n)       (APM + 0x068 + 8 * (x) + 4 * (n))
#define INS_DATA(x, n)      (APM + 0x0a8 + 8 * (x) + 4 * (n))
#define METRIC_RANGE(m, n)  (APM + 0x0c8 + 8 * (m) + 4 * (n))
#define METRIC_SELECT(k, n) (APM + 0x158 + 8 * (k) + 4 * (n))
#define ADDR_FILTER_EN      (APM + 0x170)
#define SW_RECORD_STOP_EN   (APM + 0x174)
#define SW_RECORD_STOP_CLR  (APM + 0x178)
#define INS_BANDW_TEST_EN   (APM + 0x17c)
#define STOP_DATA_LIMIT(n)  (APM + 0x180 + 4 * (n))
#define ID_MASK(n)          (APM + 0x188 + 4 * (n))
#define BANDW_TEST_EN       (APM + 0x198)
#define INS_TIME_THR(n)     (APM + 0x1b8 + 4 * (n))
#define INT_RAW             (APM + 0x1c0)
#define INT_ST              (APM + 0x1c4)
#define INT_ENA             (APM + 0x1c8)
#define INT_CLR             (APM + 0x1cc)

#define METRIC_RD_INS_DATA  14
#define METRIC_RD_INS_TIME  15
#define RANGE_RD_INS        (1u << 5)

#define HP_SYS_CLKRST_APM   0x500e60f0
#define APM_SYS_CLK_EN      (1u << 0)
#define APM_SYS_RST_EN      (1u << 1)

#define INTMTX_STATUS4      0x500d6220
#define SRC_APM_BIT         (1u << (130 - 128))

/* The window, in AXI_CLK cycles. */
#define WINDOW              1000

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M");

    qtest_writel(qts, HP_SYS_CLKRST_APM, APM_SYS_CLK_EN);
    return qts;
}

/* A window's length in nanoseconds, rounded up. */
static uint64_t window_ns(QTestState *qts)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         APM_PATH "/axi");
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    g_assert_cmpuint(period, !=, 0);
    return (period * WINDOW + 0xffffffffu) >> 32;
}

static bool apm_irq(QTestState *qts)
{
    return qtest_readl(qts, INTMTX_STATUS4) & SRC_APM_BIT;
}

/* The registers at their reset values, and their writable bits. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_registers(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CLK_EN), ==, 1);
    g_assert_cmphex(qtest_readl(qts, AGENT_SELECT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, COUNTER(0, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0);

    qtest_writel(qts, ADDR_FILTER_EN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, ADDR_FILTER_EN), ==, 0xf);
    qtest_writel(qts, STOP_DATA_LIMIT(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, STOP_DATA_LIMIT(1)), ==, 0xffff);
    qtest_writel(qts, ID_MASK(0), 0x12345678);
    g_assert_cmphex(qtest_readl(qts, ID_MASK(0)), ==, 0x5678);
    qtest_writel(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0x3);
    qtest_writel(qts, COUNTER(0, 0), 0xffffffff);
    qtest_writel(qts, INT_RAW, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, COUNTER(0, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);

    /* Its reset clears it. */
    qtest_writel(qts, HP_SYS_CLKRST_APM, APM_SYS_CLK_EN | APM_SYS_RST_EN);
    qtest_writel(qts, HP_SYS_CLKRST_APM, APM_SYS_CLK_EN);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ID_MASK(0)), ==, 0);
    qtest_quit(qts);
}

/*
 * Instantaneous bandwidth windows on an idle port: each window's length
 * and its zero beats go to the counters, the range statistics and the
 * records; nothing was transferred, so the event records stay empty.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_windows(void)
{
    QTestState *qts = start();
    uint64_t ns = window_ns(qts);

    qtest_writel(qts, AGENT_SELECT, 1);
    qtest_writel(qts, METRIC_SELECT(0, 0),
                 METRIC_RD_INS_TIME | (RANGE_RD_INS | METRIC_RD_INS_DATA) << 8);
    qtest_writel(qts, METRIC_RANGE(1, 0), 0x00000005);
    qtest_writel(qts, METRIC_RANGE(2, 0), 0x00010005);
    qtest_writel(qts, INS_TIME_THR(0), WINDOW);
    qtest_writel(qts, INS_BANDW_TEST_EN, 1);

    qtest_clock_step(qts, ns / 2);
    g_assert_cmphex(qtest_readl(qts, COUNTER(0, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 0);
    qtest_clock_step(qts, 3 * ns - ns / 2 + 2);
    g_assert_cmphex(qtest_readl(qts, COUNTER(0, 0)), ==,
                    WINDOW | WINDOW << 16);
    g_assert_cmphex(qtest_readl(qts, COUNTER(1, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 3);
    g_assert_cmphex(qtest_readl(qts, RANGE(2, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INS_DATA(0, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RD_INFO(0, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, COUNTER(0, 1)), ==, 0);

    /* Stopped, the windows end. */
    qtest_writel(qts, INS_BANDW_TEST_EN, 0);
    qtest_clock_step(qts, 4 * ns);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 3);

    /* A metric selected afresh starts over. */
    qtest_writel(qts, METRIC_SELECT(0, 0), METRIC_RD_INS_TIME);
    g_assert_cmphex(qtest_readl(qts, COUNTER(1, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 0);
    qtest_writel(qts, METRIC_SELECT(0, 0), 0);
    g_assert_cmphex(qtest_readl(qts, COUNTER(0, 0)), ==, 0);
    qtest_quit(qts);
}

/*
 * A read window under SW_RECORD_STOP_RD_DATA_LIMIT raises the unit's
 * RECORD_CNT_UNDER_LIMIT interrupt and stops both units' windows until
 * SW_RECORD_STOP_CLR; SW_RECORD_STOP_EN stops a unit as well.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_under_limit(void)
{
    QTestState *qts = start();
    uint64_t ns = window_ns(qts);

    qtest_writel(qts, METRIC_SELECT(0, 0), RANGE_RD_INS << 8);
    qtest_writel(qts, METRIC_RANGE(1, 0), 0x00000005);
    qtest_writel(qts, METRIC_SELECT(0, 1), RANGE_RD_INS << 8);
    qtest_writel(qts, METRIC_RANGE(1, 1), 0x00000005);
    qtest_writel(qts, INS_TIME_THR(0), WINDOW);
    qtest_writel(qts, INS_TIME_THR(1), WINDOW * 4);
    qtest_writel(qts, STOP_DATA_LIMIT(0), 1);
    qtest_writel(qts, INT_ENA, 0x1);
    qtest_writel(qts, INS_BANDW_TEST_EN, 0x5);

    qtest_clock_step(qts, ns + 1);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0x1);
    g_assert_cmphex(qtest_readl(qts, INT_ST), ==, 0x1);
    g_assert_true(apm_irq(qts));
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 1);

    /* Both units stopped: unit 1's longer window never ends. */
    qtest_clock_step(qts, 8 * ns);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 1);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 1)), ==, 0);

    qtest_writel(qts, INT_CLR, 0x1);
    g_assert_false(apm_irq(qts));

    /* Unit 1 started again, under no limit. */
    qtest_writel(qts, SW_RECORD_STOP_CLR, 0x2);
    qtest_clock_step(qts, 4 * ns + 4);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 1)), ==, 1);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 1);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);

    /* Software stops it. */
    qtest_writel(qts, SW_RECORD_STOP_EN, 0x2);
    qtest_clock_step(qts, 8 * ns);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 1)), ==, 1);

    /* Unit 0 started again, and stopped again by its limit. */
    qtest_writel(qts, SW_RECORD_STOP_CLR, 0x1);
    qtest_clock_step(qts, ns + 1);
    g_assert_cmphex(qtest_readl(qts, RANGE(1, 0)), ==, 2);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0x1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/axi-perf-mon/registers", test_registers);
    qtest_add_func("esp32p4/axi-perf-mon/windows", test_windows);
    qtest_add_func("esp32p4/axi-perf-mon/under-limit", test_under_limit);

    return g_test_run();
}
