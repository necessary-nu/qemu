/*
 * QTest testcase for the ESP32 timer groups' 64-bit timers: the
 * prescaler across CONFIG writes, a divider changed while counting, and
 * alarms the count has already reached
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
#define T0LOADLO                (TIMG0 + 0x18)
#define T0LOADHI                (TIMG0 + 0x1c)
#define T0LOAD                  (TIMG0 + 0x20)
#define LACTCONFIG              (TIMG0 + 0x70)
#define LACTLO                  (TIMG0 + 0x78)
#define LACTUPDATE              (TIMG0 + 0x80)
#define LACTALARMLO             (TIMG0 + 0x84)
#define LACTALARMHI             (TIMG0 + 0x88)
#define INT_RAW                 (TIMG0 + 0x9c)
#define INT_CLR                 (TIMG0 + 0xa4)

#define CFG_EN                  (1u << 31)
#define CFG_INCREASE            (1u << 30)
#define CFG_AUTORELOAD          (1u << 29)
#define CFG_DIVIDER(d)          ((uint32_t)(d) << 13)
#define CFG_LEVEL_INT           (1u << 11)
#define CFG_ALARM_EN            (1u << 10)
#define INT_T0                  (1u << 0)
#define INT_LACT                (1u << 3)

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

static void load(QTestState *qts, uint64_t v)
{
    qtest_writel(qts, T0LOADLO, (uint32_t)v);
    qtest_writel(qts, T0LOADHI, v >> 32);
    qtest_writel(qts, T0LOAD, 1);
}

static void set_alarm(QTestState *qts, uint64_t v)
{
    qtest_writel(qts, T0ALARMHI, v >> 32);
    qtest_writel(qts, T0ALARMLO, (uint32_t)v);
}

static uint32_t t0_alarmed(QTestState *qts)
{
    return qtest_readl(qts, INT_RAW) & INT_T0;
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

/*
 * TRM 10.2.3: an alarm the count has already reached, as ALARM_EN is set,
 * as the alarm registers are loaded or as the count is reloaded, comes at
 * once, and ALARM_EN clears itself as an alarm occurs.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_alarm_passed(void)
{
    QTestState *qts = start("/dev/null");
    uint32_t cfg = CFG_EN | CFG_INCREASE | CFG_DIVIDER(80) | CFG_LEVEL_INT;

    /* Armed with the count past it */
    qtest_writel(qts, T0CONFIG, cfg);
    set_alarm(qts, 5);
    step(qts, 800);
    g_assert_cmpuint(count(qts), ==, 10);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);
    qtest_writel(qts, T0CONFIG, cfg | CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    g_assert_cmphex(qtest_readl(qts, T0CONFIG), ==, cfg);
    qtest_writel(qts, INT_CLR, INT_T0);
    qtest_writel(qts, T0CONFIG, qtest_readl(qts, T0CONFIG));
    step(qts, 800);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);

    /* Armed ahead, then loaded behind the count: at once */
    set_alarm(qts, 1000);
    qtest_writel(qts, T0CONFIG, cfg | CFG_ALARM_EN);
    step(qts, 80);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);
    g_assert_cmpuint(count(qts), ==, 21);
    qtest_writel(qts, T0ALARMLO, 21);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    /* Armed ahead, then moved further ahead: it comes at the new value */
    set_alarm(qts, 23);
    qtest_writel(qts, T0CONFIG, cfg | CFG_ALARM_EN);
    qtest_writel(qts, T0ALARMLO, 25);
    step(qts, 4 * 80 - 1);
    g_assert_cmpuint(count(qts), ==, 24);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);
    step(qts, 1);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    /* Armed ahead, the count reloaded past it: at once */
    set_alarm(qts, 100);
    qtest_writel(qts, T0CONFIG, cfg | CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);
    load(qts, 200);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    /*
     * With AUTORELOAD, an alarm armed late reloads at once: the count
     * goes on from LOAD, its next tick 80 cycles on (the reload falls on
     * a tick, 2000 cycles from the start).
     */
    qtest_writel(qts, T0CONFIG, cfg | CFG_AUTORELOAD);
    load(qts, 300);
    qtest_writel(qts, T0LOADLO, 1000);
    qtest_writel(qts, T0CONFIG, cfg | CFG_AUTORELOAD | CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    g_assert_cmpuint(count(qts), ==, 1000);
    step(qts, 79);
    g_assert_cmpuint(count(qts), ==, 1000);
    step(qts, 1);
    g_assert_cmpuint(count(qts), ==, 1001);
    qtest_writel(qts, INT_CLR, INT_T0);

    /* Counting down, an alarm above the count has been passed */
    qtest_writel(qts, T0CONFIG, CFG_EN | CFG_DIVIDER(80) | CFG_LEVEL_INT);
    set_alarm(qts, 2000);
    qtest_writel(qts, T0CONFIG, CFG_EN | CFG_DIVIDER(80) | CFG_LEVEL_INT |
                 CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_quit(qts);
}

/*
 * The alarm and load values' words are separate registers: either may be
 * written first, as ESP-IDF's timer_ll writes the alarm's high word first
 * and the load value's low word first.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_value_words(void)
{
    QTestState *qts = start("/dev/null");

    qtest_writel(qts, T0ALARMHI, 0x12);
    qtest_writel(qts, T0ALARMLO, 0x345);
    g_assert_cmphex(qtest_readl(qts, T0ALARMHI), ==, 0x12);
    g_assert_cmphex(qtest_readl(qts, T0ALARMLO), ==, 0x345);
    qtest_writel(qts, T0LOADHI, 0x67);
    qtest_writel(qts, T0LOADLO, 0x89);
    qtest_writel(qts, T0LOAD, 1);
    g_assert_cmphex(count(qts), ==, 0x6700000089ull);
    qtest_quit(qts);
}

/*
 * Choices where the TRM is silent. The comparison runs while the count
 * is frozen (TIMGn_Tx_EN clear): an alarm the frozen count has reached
 * comes at once, one ahead waits for counting. Higher and lower are the
 * 64-bit values' own, with no window: an alarm is never left for the
 * counter to come round to after wrapping.
 */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_alarm_choices(void)
{
    QTestState *qts = start("/dev/null");
    uint32_t cfg = CFG_INCREASE | CFG_DIVIDER(80) | CFG_LEVEL_INT;

    load(qts, 10);
    set_alarm(qts, 10);
    qtest_writel(qts, T0CONFIG, cfg | CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    set_alarm(qts, 12);
    qtest_writel(qts, T0CONFIG, cfg | CFG_ALARM_EN);
    step(qts, 8000);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);
    qtest_writel(qts, T0CONFIG, cfg | CFG_EN | CFG_ALARM_EN);
    step(qts, 159);
    g_assert_cmphex(t0_alarmed(qts), ==, 0);
    step(qts, 1);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    qtest_writel(qts, T0CONFIG, cfg);
    load(qts, UINT64_MAX - 15);
    set_alarm(qts, 5);
    qtest_writel(qts, T0CONFIG, cfg | CFG_EN | CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_writel(qts, INT_CLR, INT_T0);

    qtest_writel(qts, T0CONFIG, CFG_DIVIDER(80) | CFG_LEVEL_INT);
    load(qts, 5);
    set_alarm(qts, UINT64_MAX - 15);
    qtest_writel(qts, T0CONFIG, CFG_EN | CFG_DIVIDER(80) | CFG_LEVEL_INT |
                 CFG_ALARM_EN);
    g_assert_cmphex(t0_alarmed(qts), ==, INT_T0);
    qtest_quit(qts);
}

/* The LACT's alarm keeps the same rules. */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_lact_alarm_passed(void)
{
    QTestState *qts = start("/dev/null");
    uint32_t cfg = CFG_EN | CFG_INCREASE | CFG_DIVIDER(80) | CFG_LEVEL_INT;

    qtest_writel(qts, LACTCONFIG, cfg);
    qtest_writel(qts, LACTALARMHI, 0);
    qtest_writel(qts, LACTALARMLO, 5);
    step(qts, 800);
    qtest_writel(qts, LACTUPDATE, 1);
    g_assert_cmpuint(qtest_readl(qts, LACTLO), ==, 10);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_LACT, ==, 0);
    qtest_writel(qts, LACTCONFIG, cfg | CFG_ALARM_EN);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_LACT, ==, INT_LACT);
    g_assert_cmphex(qtest_readl(qts, LACTCONFIG), ==, cfg);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32-timg/config-keeps-prescaler",
                   test_config_keeps_prescaler);
    qtest_add_func("esp32-timg/divider-change", test_divider_change);
    qtest_add_func("esp32-timg/value-words", test_value_words);
    qtest_add_func("esp32-timg/alarm-passed", test_alarm_passed);
    qtest_add_func("esp32-timg/alarm-choices", test_alarm_choices);
    qtest_add_func("esp32-timg/lact-alarm-passed", test_lact_alarm_passed);
    return g_test_run();
}
