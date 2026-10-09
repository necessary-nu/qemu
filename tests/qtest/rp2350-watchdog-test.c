/*
 * QTest testcase for the RP2350 watchdog and power-on state machine
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define WATCHDOG        0x400d8000
#define CTRL            0x00
#define LOAD            0x04
#define REASON          0x08
#define SCRATCH(n)      (0x0c + 4 * (n))
#define CTRL_TRIGGER    (1u << 31)
#define CTRL_ENABLE     (1u << 30)
#define CTRL_PAUSE      0x07000000u
#define CTRL_TIME       0x00ffffffu
#define REASON_TIMER    1
#define REASON_FORCE    2

#define PSM             0x40018000
#define FRCE_ON         0x0
#define FRCE_OFF        0x4
#define WDSEL           0x8
#define DONE            0xc
#define PSM_ALL         0x01ffffffu
#define PSM_PROC_COLD   (1u << 0)
#define PSM_CLOCKS      (1u << 5)
#define PSM_SIO         (1u << 21)
#define PSM_PROC0       (1u << 23)
#define PSM_PROC1       (1u << 24)

#define XOR             0x1000
#define SET             0x2000
#define CLR             0x3000

#define RESETS          0x40020000
#define RESETS_RESET    0x0
#define RESETS_WDSEL    0x4
#define RESET_TIMER0    (1u << 23)

#define TICKS           0x40108000
#define TICK_CTRL(n)    (TICKS + (n) * 0xc)
#define TICK_CYCLES(n)  (TICKS + (n) * 0xc + 4)
#define TICK_TIMER0     2
#define TICK_WATCHDOG   4

#define TIMER0_TIMERAWL 0x400b0028

#define SIO_SPINLOCK_ST 0xd000005c
#define SIO_SPINLOCK(n) (0xd0000100 + 4 * (n))

#define US              1000

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

static uint32_t wd_read(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, WATCHDOG + reg);
}

static void wd_write(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, WATCHDOG + reg, value);
}

static uint32_t wd_time(QTestState *qts)
{
    return wd_read(qts, CTRL) & CTRL_TIME;
}

/* Start the WATCHDOG tick generator, dividing 12 MHz clk_ref by `cycles`. */
static void start_tick(QTestState *qts, uint32_t cycles)
{
    qtest_writel(qts, TICK_CYCLES(TICK_WATCHDOG), cycles);
    qtest_writel(qts, TICK_CTRL(TICK_WATCHDOG), 1);
}

/* [spec:nuos:req:emu.watchdog/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int i;

    g_assert_cmphex(wd_read(qts, CTRL), ==, CTRL_PAUSE);
    g_assert_cmphex(wd_read(qts, LOAD), ==, 0);
    g_assert_cmphex(wd_read(qts, REASON), ==, 0);
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(wd_read(qts, SCRATCH(i)), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, PSM + FRCE_ON), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PSM + FRCE_OFF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PSM + WDSEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PSM + DONE), ==, PSM_ALL);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.watchdog/test] */
static void test_registers(void)
{
    QTestState *qts = start();

    /* Scratch registers are plain RW, with the atomic aliases. */
    wd_write(qts, SCRATCH(3), 0x12345678);
    g_assert_cmphex(wd_read(qts, SCRATCH(3)), ==, 0x12345678);
    wd_write(qts, SET + SCRATCH(3), 0x80000001);
    wd_write(qts, CLR + SCRATCH(3), 0x00000008);
    wd_write(qts, XOR + SCRATCH(3), 0x0000ff00);
    g_assert_cmphex(wd_read(qts, SCRATCH(3)), ==, 0x9234a971);

    /* CTRL: TIME is read-only, reserved bits read 0. */
    wd_write(qts, CTRL, 0x3fffffff);
    g_assert_cmphex(wd_read(qts, CTRL), ==, CTRL_PAUSE);
    wd_write(qts, CLR + CTRL, CTRL_PAUSE);
    g_assert_cmphex(wd_read(qts, CTRL), ==, 0);

    /* LOAD is write-only; the count shows in CTRL.TIME. */
    wd_write(qts, LOAD, 0xffffffff);
    g_assert_cmphex(wd_read(qts, LOAD), ==, 0);
    g_assert_cmphex(wd_time(qts), ==, 0xffffff);

    /* REASON is read-only. */
    wd_write(qts, REASON, 3);
    g_assert_cmphex(wd_read(qts, REASON), ==, 0);

    /* PSM: 25 stage bits, with aliases; DONE is read-only. */
    qtest_writel(qts, PSM + WDSEL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PSM + WDSEL), ==, PSM_ALL);
    qtest_writel(qts, PSM + CLR + WDSEL, PSM_PROC_COLD);
    g_assert_cmphex(qtest_readl(qts, PSM + WDSEL), ==, PSM_ALL - 1);
    qtest_writel(qts, PSM + FRCE_ON, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PSM + FRCE_ON), ==, PSM_ALL);
    qtest_writel(qts, PSM + DONE, 0);
    g_assert_cmphex(qtest_readl(qts, PSM + DONE), ==, PSM_ALL);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.watchdog/test] */
static void test_countdown(void)
{
    QTestState *qts = start();

    wd_write(qts, LOAD, 1000);
    wd_write(qts, SET + CTRL, CTRL_ENABLE);

    /* The TICKS generator is stopped at reset, so nothing counts. */
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(wd_time(qts), ==, 1000);

    /* 12 clk_ref cycles per tick: one count per microsecond. */
    start_tick(qts, 12);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(wd_time(qts), ==, 990);

    /* Partial ticks are not lost across register accesses. */
    qtest_clock_step(qts, US / 2);
    g_assert_cmpuint(wd_time(qts), ==, 990);
    qtest_clock_step(qts, US / 2);
    g_assert_cmpuint(wd_time(qts), ==, 989);

    /* CYCLES sets the rate. */
    qtest_writel(qts, TICK_CYCLES(TICK_WATCHDOG), 6);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(wd_time(qts), ==, 969);
    qtest_writel(qts, TICK_CYCLES(TICK_WATCHDOG), 120);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(wd_time(qts), ==, 959);
    qtest_writel(qts, TICK_CYCLES(TICK_WATCHDOG), 12);

    /* Clearing ENABLE pauses the count. */
    wd_write(qts, CLR + CTRL, CTRL_ENABLE);
    qtest_clock_step(qts, 50 * US);
    g_assert_cmpuint(wd_time(qts), ==, 959);
    wd_write(qts, SET + CTRL, CTRL_ENABLE);

    /* So does stopping the tick generator. */
    qtest_writel(qts, TICK_CTRL(TICK_WATCHDOG), 0);
    qtest_clock_step(qts, 50 * US);
    g_assert_cmpuint(wd_time(qts), ==, 959);
    qtest_writel(qts, TICK_CTRL(TICK_WATCHDOG), 1);

    /* Reloading restarts the count. */
    qtest_clock_step(qts, 9 * US);
    g_assert_cmpuint(wd_time(qts), ==, 950);
    wd_write(qts, LOAD, 500);
    g_assert_cmpuint(wd_time(qts), ==, 500);
    qtest_clock_step(qts, 5 * US);
    g_assert_cmpuint(wd_time(qts), ==, 495);
    g_assert_cmphex(wd_read(qts, REASON), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.watchdog/test] */
static void test_expiry(void)
{
    QTestState *qts = start();
    int i;

    for (i = 0; i < 8; i++) {
        wd_write(qts, SCRATCH(i), 0x5c000000 + i);
    }
    qtest_writel(qts, PSM + WDSEL, PSM_ALL & ~PSM_PROC_COLD);
    start_tick(qts, 12);
    wd_write(qts, LOAD, 50);
    wd_write(qts, CTRL, CTRL_ENABLE);

    /* The reset comes on the tick that takes the counter to zero. */
    qtest_clock_step(qts, 49 * US);
    g_assert_cmpuint(wd_time(qts), ==, 1);
    g_assert_cmphex(wd_read(qts, REASON), ==, 0);
    qtest_clock_step(qts, US);
    g_assert_cmphex(wd_read(qts, REASON), ==, REASON_TIMER);

    /* The reset disarms the watchdog; scratch survives. */
    g_assert_cmphex(wd_read(qts, CTRL), ==, 0);
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(wd_read(qts, SCRATCH(i)), ==, 0x5c000000 + i);
    }
    /* CLOCKS was in the sequence, and with it TICKS. */
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_WATCHDOG)), ==, 0);
    /* The PSM is reset only by a chip-level reset. */
    g_assert_cmphex(qtest_readl(qts, PSM + WDSEL), ==,
                    PSM_ALL & ~PSM_PROC_COLD);

    /* Loading zero with the watchdog enabled resets on the next tick. */
    start_tick(qts, 12);
    wd_write(qts, LOAD, 0);
    wd_write(qts, CTRL, CTRL_ENABLE);
    wd_write(qts, SCRATCH(0), 0);
    qtest_writel(qts, PSM + WDSEL, 0);
    qtest_clock_step(qts, US);
    g_assert_cmphex(wd_read(qts, CTRL) & CTRL_ENABLE, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.watchdog/test] */
static void test_trigger(void)
{
    QTestState *qts = start();

    wd_write(qts, SCRATCH(7), 0xcafef00d);
    start_tick(qts, 12);
    wd_write(qts, LOAD, 1000);
    wd_write(qts, CTRL, CTRL_ENABLE);
    qtest_writel(qts, PSM + WDSEL, PSM_PROC0 | PSM_PROC1);

    wd_write(qts, SET + CTRL, CTRL_TRIGGER);
    g_assert_cmphex(wd_read(qts, REASON), ==, REASON_FORCE);
    g_assert_cmphex(wd_read(qts, CTRL) & (CTRL_TRIGGER | CTRL_ENABLE), ==, 0);
    g_assert_cmphex(wd_read(qts, SCRATCH(7)), ==, 0xcafef00d);
    /* Only the processors were reset: TICKS kept its configuration. */
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_WATCHDOG)) & 1, ==, 1);

    /* REASON records the last reset only. */
    wd_write(qts, LOAD, 3);
    wd_write(qts, CTRL, CTRL_ENABLE);
    qtest_clock_step(qts, 3 * US);
    g_assert_cmphex(wd_read(qts, REASON), ==, REASON_TIMER);

    /* A chip-level reset resets the watchdog, scratch and REASON too. */
    qtest_system_reset(qts);
    g_assert_cmphex(wd_read(qts, REASON), ==, 0);
    g_assert_cmphex(wd_read(qts, SCRATCH(7)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PSM + WDSEL), ==, 0);
    qtest_quit(qts);
}

/*
 * The watchdog resets the stage WDSEL selects and every stage after it,
 * and the subsystems RESETS.WDSEL selects.
 */
/* [spec:nuos:req:emu.watchdog/test] */
static void test_domains(void)
{
    QTestState *qts = start();

    /* SIO keeps its state when only the processors are reset. */
    g_assert_cmphex(qtest_readl(qts, SIO_SPINLOCK(3)), !=, 0);
    qtest_writel(qts, PSM + WDSEL, PSM_PROC1);
    wd_write(qts, CTRL, CTRL_TRIGGER);
    g_assert_cmphex(qtest_readl(qts, SIO_SPINLOCK_ST), ==, 1u << 3);

    /* Selecting CLOCKS reaches SIO further down the sequence. */
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    qtest_writel(qts, PSM + WDSEL, PSM_CLOCKS);
    wd_write(qts, CTRL, CTRL_TRIGGER);
    g_assert_cmphex(qtest_readl(qts, SIO_SPINLOCK_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 0);
    /* RESETS comes before CLOCKS in the sequence, so it was not reset. */
    qtest_writel(qts, RESETS + RESETS_RESET, 0);
    qtest_writel(qts, PSM + WDSEL, PSM_SIO);
    wd_write(qts, CTRL, CTRL_TRIGGER);
    g_assert_cmphex(qtest_readl(qts, RESETS + RESETS_RESET), ==, 0);

    /* RESETS.WDSEL resets a subsystem alone; RESET keeps its value. */
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 100);
    qtest_writel(qts, PSM + WDSEL, 0);
    qtest_writel(qts, RESETS + RESETS_WDSEL, RESET_TIMER0);
    wd_write(qts, CTRL, CTRL_TRIGGER);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RESETS + RESETS_RESET), ==, 0);
    qtest_clock_step(qts, 7 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 7);

    /* Resetting the RESETS stage asserts every subsystem reset. */
    qtest_writel(qts, RESETS + RESETS_WDSEL, 0);
    qtest_writel(qts, PSM + WDSEL, 1u << 4);
    wd_write(qts, CTRL, CTRL_TRIGGER);
    g_assert_cmphex(qtest_readl(qts, RESETS + RESETS_RESET), ==, 0x1fffffff);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.watchdog/test] */
static void test_frce_off(void)
{
    QTestState *qts = start();

    /* A processor stage is forced off on its own. */
    qtest_writel(qts, PSM + SET + FRCE_OFF, PSM_PROC1);
    g_assert_cmphex(qtest_readl(qts, PSM + FRCE_OFF), ==, PSM_PROC1);
    g_assert_cmphex(qtest_readl(qts, PSM + DONE), ==, PSM_ALL & ~PSM_PROC1);
    qtest_writel(qts, PSM + CLR + FRCE_OFF, PSM_PROC1);
    g_assert_cmphex(qtest_readl(qts, PSM + DONE), ==, PSM_ALL);

    /* Forcing off an earlier stage holds every later one too. */
    g_assert_cmphex(qtest_readl(qts, SIO_SPINLOCK(0)), !=, 0);
    qtest_writel(qts, PSM + FRCE_OFF, PSM_SIO);
    g_assert_cmphex(qtest_readl(qts, PSM + DONE), ==, PSM_SIO - 1);
    qtest_writel(qts, PSM + FRCE_OFF, 0);
    g_assert_cmphex(qtest_readl(qts, PSM + DONE), ==, PSM_ALL);
    g_assert_cmphex(qtest_readl(qts, SIO_SPINLOCK_ST), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-watchdog-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/watchdog/reset-values", test_reset_values);
    qtest_add_func("/rp2350/watchdog/registers", test_registers);
    qtest_add_func("/rp2350/watchdog/countdown", test_countdown);
    qtest_add_func("/rp2350/watchdog/expiry", test_expiry);
    qtest_add_func("/rp2350/watchdog/trigger", test_trigger);
    qtest_add_func("/rp2350/watchdog/domains", test_domains);
    qtest_add_func("/rp2350/watchdog/frce-off", test_frce_off);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
