/*
 * QTest testcase for the ESP32 timer groups' 64-bit timers: the
 * prescaler across CONFIG writes, and a divider changed while counting
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIMG0's T0 counts APB_CLK, 40 MHz from reset (25 ns), over its
 * prescaler: with TIMG_T0_DIVIDER 80 a tick every 2 us. Every step here
 * is a whole number of APB_CLK cycles from reset, so that the edges fall
 * where the test says.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define TIMG0                   0x3ff5f000
#define T0CONFIG                (TIMG0 + 0x00)
#define T0LO                    (TIMG0 + 0x04)
#define T0HI                    (TIMG0 + 0x08)
#define T0UPDATE                (TIMG0 + 0x0c)
#define T0ALARMLO               (TIMG0 + 0x10)
#define T0ALARMHI               (TIMG0 + 0x14)
#define INT_RAW                 (TIMG0 + 0x9c)
#define INT_CLR                 (TIMG0 + 0xa4)

#define CFG_EN                  (1u << 31)
#define CFG_INCREASE            (1u << 30)
#define CFG_DIVIDER(d)          ((uint32_t)(d) << 13)
#define CFG_LEVEL_INT           (1u << 11)
#define CFG_ALARM_EN            (1u << 10)
#define INT_T0                  (1u << 0)

#define APB_NS                  25

static int64_t now;

static QTestState *start(const char *log)
{
    QTestState *qts = qtest_initf("-M esp32 -nic none -d guest_errors -D %s",
                                  log);

    now = 0;
    return qts;
}

static void step(QTestState *qts, int64_t cycles)
{
    now = qtest_clock_step(qts, cycles * APB_NS);
}

static uint64_t count(QTestState *qts)
{
    qtest_writel(qts, T0UPDATE, 1);
    return qtest_readl(qts, T0LO) | (uint64_t)qtest_readl(qts, T0HI) << 32;
}

static int occurrences(const char *log, const char *what)
{
    g_autofree char *text = NULL;
    int n = 0;

    g_assert_true(g_file_get_contents(log, &text, NULL, NULL));
    for (const char *p = strstr(text, what); p; p = strstr(p + 1, what)) {
        n++;
    }
    return n;
}

/*
 * The prescaler's cycles towards its next tick carry over a CONFIG write:
 * written again (as ESP-IDF's alarm handler rewrites it) every half tick,
 * the timer still ticks every 80 cycles.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_config_keeps_prescaler(void)
{
    g_autofree char *log = NULL;
    int fd = g_file_open_tmp("esp32-timg-XXXXXX.log", &log, NULL);
    uint32_t cfg = CFG_EN | CFG_INCREASE | CFG_DIVIDER(80);
    QTestState *qts;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = start(log);
    qtest_writel(qts, T0CONFIG, cfg);
    for (int i = 0; i < 10; i++) {
        step(qts, 40);
        qtest_writel(qts, T0CONFIG, cfg);
        step(qts, 40);
        g_assert_cmpuint(count(qts), ==, i + 1);
    }

    /*
     * An alarm armed half way into a tick comes when the count reaches
     * it: 40 cycles to the next tick, 80 to the one after.
     */
    step(qts, 40);
    qtest_writel(qts, T0ALARMLO, 12);
    qtest_writel(qts, T0ALARMHI, 0);
    qtest_writel(qts, T0CONFIG, cfg | CFG_LEVEL_INT | CFG_ALARM_EN);
    step(qts, 119);
    g_assert_cmpuint(count(qts), ==, 11);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_T0, ==, 0);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 12);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_T0, ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    g_assert_cmpint(occurrences(log, "divider changed"), ==, 0);
    qtest_quit(qts);
    unlink(log);
}

/*
 * The TRM (10.2.1) has the timer disabled before its divider changes, the
 * result otherwise unpredictable: a change while it counts is a guest
 * error, and the model goes on as the ESP32-P4's does, the prescaler's
 * cycles counting towards the new divisor. A change while it is stopped
 * is the documented way, and is not logged.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_divider_change(void)
{
    g_autofree char *log = NULL;
    int fd = g_file_open_tmp("esp32-timg-XXXXXX.log", &log, NULL);
    uint32_t cfg = CFG_EN | CFG_INCREASE;
    QTestState *qts;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = start(log);
    qtest_writel(qts, T0CONFIG, cfg | CFG_DIVIDER(80));
    step(qts, 120);
    g_assert_cmpuint(count(qts), ==, 1);

    /* 40 cycles counted: 60 more to a divisor of 100. */
    qtest_writel(qts, T0CONFIG, cfg | CFG_DIVIDER(100));
    g_assert_cmpint(occurrences(log, "timer.esp32.timg0: timer T0's "
                                "divider changed while it counts"), ==, 1);
    step(qts, 59);
    g_assert_cmpuint(count(qts), ==, 1);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 2);
    step(qts, 100);
    g_assert_cmpuint(count(qts), ==, 3);

    /* 50 counted: a divisor of 40 they have passed restarts the count. */
    step(qts, 50);
    qtest_writel(qts, T0CONFIG, cfg | CFG_DIVIDER(40));
    g_assert_cmpint(occurrences(log, "divider changed"), ==, 2);
    step(qts, 39);
    g_assert_cmpuint(count(qts), ==, 3);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 4);

    /* Stopped, the divider changes as the TRM has it: not logged. */
    qtest_writel(qts, T0CONFIG, CFG_INCREASE | CFG_DIVIDER(40));
    qtest_writel(qts, T0CONFIG, CFG_INCREASE | CFG_DIVIDER(80));
    qtest_writel(qts, T0CONFIG, cfg | CFG_DIVIDER(80));
    step(qts, 80);
    g_assert_cmpuint(count(qts), ==, 5);
    g_assert_cmpint(occurrences(log, "divider changed"), ==, 2);
    qtest_quit(qts);
    unlink(log);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32-timg/config-keeps-prescaler",
                   test_config_keeps_prescaler);
    qtest_add_func("esp32-timg/divider-change", test_divider_change);
    return g_test_run();
}
