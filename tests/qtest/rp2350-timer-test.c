/*
 * QTest testcase for the RP2350 system timers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define TIMER0      0x400b0000
#define TIMER1      0x400b8000
#define TIMEHW      0x00
#define TIMELW      0x04
#define TIMEHR      0x08
#define TIMELR      0x0c
#define ALARM(n)    (0x10 + 4 * (n))
#define ARMED       0x20
#define TIMERAWH    0x24
#define TIMERAWL    0x28
#define PAUSE       0x30
#define LOCKED      0x34
#define SOURCE      0x38
#define INTR        0x3c
#define INTE        0x40
#define INTF        0x44
#define INTS        0x48

#define TICKS       0x40108000
#define TICK_CTRL(n) (TICKS + (n) * 0xc)
#define TICK_CYCLES(n) (TICKS + (n) * 0xc + 4)
#define TICK_TIMER0 2
#define TICK_TIMER1 3

#define US          1000

/*
 * Start TICKS generator n at one tick every `cycles` cycles of the 12 MHz
 * clk_ref; 12 gives the 1 us tick pico-sdk configures.
 */
static void tick_start(QTestState *qts, int n, uint32_t cycles)
{
    qtest_writel(qts, TICK_CYCLES(n), cycles);
    qtest_writel(qts, TICK_CTRL(n), 1);
}

static char *rom_path;

static QTestState *start(bool intercept)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    if (intercept) {
        qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    }
    return qts;
}

/* [spec:nuos:req:emu.timer/test] */
static void test_counter(void)
{
    QTestState *qts = start(false);

    /* The counter needs its tick generator. */
    qtest_clock_step(qts, 1000 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 0);

    tick_start(qts, TICK_TIMER0, 12);
    qtest_clock_step(qts, 5 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 5);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWH), ==, 0);

    /* TIMER1 has its own generator, still stopped. */
    g_assert_cmpuint(qtest_readl(qts, TIMER1 + TIMERAWL), ==, 0);

    /* Stopping the tick stops the counter. */
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 5);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);

    /* PAUSE stops it too. */
    qtest_writel(qts, TIMER0 + PAUSE, 1);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 5);
    qtest_writel(qts, TIMER0 + PAUSE, 0);
    qtest_clock_step(qts, 3 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 8);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_latching_and_writes(void)
{
    QTestState *qts = start(false);

    tick_start(qts, TICK_TIMER0, 12);
    qtest_writel(qts, TIMER0 + TIMELW, 0xfffffffe);
    qtest_writel(qts, TIMER0 + TIMEHW, 0x7);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMERAWH), ==, 0x7);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 0xfffffffe);

    /* Reading TIMELR latches TIMEHR across the carry. */
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMELR), ==, 0xfffffffe);
    qtest_clock_step(qts, 3 * US);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMEHR), ==, 0x7);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMERAWH), ==, 0x8);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 0x1);

    /* LOCKED makes the counter read-only, until reset. */
    qtest_writel(qts, TIMER0 + LOCKED, 1);
    qtest_writel(qts, TIMER0 + LOCKED, 0);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + LOCKED), ==, 1);
    qtest_writel(qts, TIMER0 + TIMELW, 0);
    qtest_writel(qts, TIMER0 + TIMEHW, 0);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + TIMERAWH), ==, 0x8);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_alarm(void)
{
    QTestState *qts = start(true);
    uint32_t now;

    tick_start(qts, TICK_TIMER0, 12);
    qtest_clock_step(qts, 50 * US);
    now = qtest_readl(qts, TIMER0 + TIMERAWL);

    qtest_writel(qts, TIMER0 + INTE, 1u << 2);
    qtest_writel(qts, TIMER0 + ALARM(2), now + 100);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + ARMED), ==, 1u << 2);

    qtest_clock_step(qts, 99 * US);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + INTR), ==, 0);
    g_assert_false(qtest_get_irq(qts, 2));

    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + INTR), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + INTS), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + ARMED), ==, 0);
    g_assert_true(qtest_get_irq(qts, 2));

    /* INTR is write-1-to-clear. */
    qtest_writel(qts, TIMER0 + INTR, 1u << 2);
    g_assert_false(qtest_get_irq(qts, 2));

    /* Disarming cancels a pending alarm. */
    now = qtest_readl(qts, TIMER0 + TIMERAWL);
    qtest_writel(qts, TIMER0 + ALARM(2), now + 10);
    qtest_writel(qts, TIMER0 + ARMED, 1u << 2);
    qtest_clock_step(qts, 20 * US);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + INTR), ==, 0);

    /* INTF forces the interrupt. */
    qtest_writel(qts, TIMER0 + INTF, 1u << 2);
    g_assert_true(qtest_get_irq(qts, 2));
    qtest_writel(qts, TIMER0 + INTF, 0);
    g_assert_false(qtest_get_irq(qts, 2));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_alarm_overshoot(void)
{
    QTestState *qts = start(true);

    /*
     * Stepping well past the target in one go, as a busy host may run the
     * alarm callback late, still fires the alarm.
     */
    tick_start(qts, TICK_TIMER0, 12);
    qtest_writel(qts, TIMER0 + INTE, 1u << 3);
    qtest_writel(qts, TIMER0 + ALARM(3), 2000);
    qtest_clock_step(qts, 5000 * US);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + INTR), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, TIMER0 + ARMED), ==, 0);
    g_assert_true(qtest_get_irq(qts, 3));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_timer1_alarm(void)
{
    QTestState *qts = start(true);

    tick_start(qts, TICK_TIMER1, 12);
    qtest_writel(qts, TIMER1 + INTE, 1u << 0);
    qtest_writel(qts, TIMER1 + ALARM(0), 10);
    qtest_clock_step(qts, 10 * US);
    g_assert_true(qtest_get_irq(qts, 4));
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_sysclk_source(void)
{
    QTestState *qts = start(false);

    /* clk_sys counts without a tick generator. */
    qtest_writel(qts, TIMER0 + SOURCE, 1);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 300);

    /* Back on a tick of CYCLES 24 it counts every 2 us, from 300. */
    tick_start(qts, TICK_TIMER0, 24);
    qtest_writel(qts, TIMER0 + SOURCE, 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 305);

    /* clk_sys ignores the tick's CYCLES and its enable. */
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 0);
    qtest_writel(qts, TIMER0 + SOURCE, 1);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 455);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_tick_cycles(void)
{
    QTestState *qts = start(false);

    /* CYCLES resets to 0, which never completes a tick. */
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 0);

    /* 12 MHz / 24: a tick every 2 us. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 24);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 5);

    /* 12 MHz / 6: a tick every 500 ns, carrying on from 5. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 6);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 25);

    /* 12 MHz / 12: the 1 us tick. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 35);

    /* Back to CYCLES 0: the generator stops, and the counter with it. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 35);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_tick_fraction(void)
{
    QTestState *qts = start(false);
    int i;

    /*
     * At CYCLES 7 a tick is 583.3 ns. Rewriting CYCLES with the same value
     * every 300 ns keeps the rate, and must not lose the time already
     * spent towards the next tick: 30 x 300 ns is 15.4 ticks.
     */
    tick_start(qts, TICK_TIMER0, 7);
    for (i = 0; i < 30; i++) {
        qtest_clock_step(qts, 300);
        qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 7);
    }
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, 15);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.timer/test] */
static void test_alarm_cycles(void)
{
    QTestState *qts = start(true);
    uint32_t now;

    /* At CYCLES 24 an alarm 100 counts ahead is 200 us away. */
    tick_start(qts, TICK_TIMER0, 24);
    qtest_clock_step(qts, 7 * US);
    now = qtest_readl(qts, TIMER0 + TIMERAWL);
    g_assert_cmpuint(now, ==, 3);
    qtest_writel(qts, TIMER0 + INTE, 1u << 1);
    qtest_writel(qts, TIMER0 + ALARM(1), now + 100);
    qtest_clock_step(qts, 198 * US);
    g_assert_false(qtest_get_irq(qts, 1));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 1));
    g_assert_cmpuint(qtest_readl(qts, TIMER0 + TIMERAWL), ==, now + 100);
    qtest_writel(qts, TIMER0 + INTR, 1u << 1);

    /*
     * Changing CYCLES while an alarm is armed reschedules it: 100 counts
     * at CYCLES 6 is 50 us.
     */
    now = qtest_readl(qts, TIMER0 + TIMERAWL);
    qtest_writel(qts, TIMER0 + ALARM(1), now + 100);
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 6);
    qtest_clock_step(qts, 49 * US);
    g_assert_false(qtest_get_irq(qts, 1));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 1));
    qtest_writel(qts, TIMER0 + INTR, 1u << 1);

    /* A stopped generator holds an armed alarm until it restarts. */
    now = qtest_readl(qts, TIMER0 + TIMERAWL);
    qtest_writel(qts, TIMER0 + ALARM(1), now + 10);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 0);
    qtest_clock_step(qts, 100 * US);
    g_assert_false(qtest_get_irq(qts, 1));
    g_assert_cmphex(qtest_readl(qts, TIMER0 + ARMED), ==, 1u << 1);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    qtest_clock_step(qts, 5 * US);
    g_assert_true(qtest_get_irq(qts, 1));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-timer-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/timer/counter", test_counter);
    qtest_add_func("/rp2350/timer/latching", test_latching_and_writes);
    qtest_add_func("/rp2350/timer/alarm", test_alarm);
    qtest_add_func("/rp2350/timer/alarm-overshoot", test_alarm_overshoot);
    qtest_add_func("/rp2350/timer/timer1-alarm", test_timer1_alarm);
    qtest_add_func("/rp2350/timer/sysclk-source", test_sysclk_source);
    qtest_add_func("/rp2350/timer/tick-cycles", test_tick_cycles);
    qtest_add_func("/rp2350/timer/tick-fraction", test_tick_fraction);
    qtest_add_func("/rp2350/timer/alarm-cycles", test_alarm_cycles);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
