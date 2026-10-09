/*
 * QTest testcase for the RP2350 DORMANT state
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * An oscillator stopped by its DORMANT register stops the clocks running
 * from it until a wake event: a GPIO interrupt to the dormant_wake
 * destination, here a pin driven from outside through the GPIO block's
 * "pad-in" lines, or the AON timer alarm. qtest accesses stand in for
 * whatever bus master is still clocked.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define SET 0x2000
#define CLR 0x3000

#define GPIO_PATH       "/machine/soc/gpio"

#define CLOCKS          0x40010000
#define CLK_REF_CTRL    (CLOCKS + 0x30)
#define CLK_SYS_CTRL    (CLOCKS + 0x3c)
#define CLK_REF_XOSC    2

#define XOSC            0x40048000
#define XOSC_CTRL       (XOSC + 0x0)
#define XOSC_STATUS     (XOSC + 0x4)
#define XOSC_DORMANT    (XOSC + 0x8)
#define XOSC_STARTUP    (XOSC + 0xc)
#define XOSC_STABLE     (1u << 31)
#define XOSC_BADWRITE   (1u << 24)
#define XOSC_ENABLED    (1u << 12)

#define ROSC            0x400e8000
#define ROSC_DORMANT    (ROSC + 0x10)
#define ROSC_STATUS     (ROSC + 0x1c)
#define ROSC_COUNT      (ROSC + 0x24)
#define ROSC_STABLE     (1u << 31)
#define ROSC_DIV_RUNNING (1u << 16)
#define ROSC_ENABLED    (1u << 12)

#define DORMANT_DORMANT 0x636f6d61u
#define DORMANT_WAKE    0x77616b65u

#define TICKS           0x40108000
#define TICK_CTRL(n)    (TICKS + (n) * 0xc)
#define TICK_CYCLES(n)  (TICKS + (n) * 0xc + 4)
#define TICK_TIMER0     2
#define TICK_RUNNING    (1u << 1)
#define TIMER0_TIMERAWL 0x400b0028
#define TIMER0_SOURCE   0x400b0038

#define POWMAN          0x40100000
#define PM_SET_TIME(n)  (POWMAN + 0x60 + 4 * (3 - (n)))
#define PM_READ_TIME_LOWER (POWMAN + 0x74)
#define PM_ALARM_TIME(n) (POWMAN + 0x78 + 4 * (3 - (n)))
#define PM_TIMER        (POWMAN + 0x88)
#define PM_PASSWORD     0x5afe0000u
#define TIMER_RUN       (1u << 1)
#define TIMER_ALARM_ENAB (1u << 4)
#define TIMER_ALARM     (1u << 6)
#define TIMER_USE_LPOSC (1u << 8)
#define TIMER_USE_XOSC  (1u << 9)

#define IO_BANK0        0x40028000
#define PADS_BANK0      0x40038000
#define GPIO_CTRL(p)    (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define INTR0           (IO_BANK0 + 0x230)
#define DORMANT_WAKE_INTE0 (IO_BANK0 + 0x2d8)
#define FUNC_SIO        5
#define PAD_IE          (1u << 6)
#define EDGE_HIGH       0x8
#define LEVEL_HIGH      0x2
#define PIN             5

/* STARTUP.DELAY 0x2f: 0x2f * 256 periods of the 12 MHz crystal. */
#define XOSC_STARTUP_NS 1002667

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    return qts;
}

/* xosc_init(), then clk_sys from clk_ref from the XOSC. */
static void run_from_xosc(QTestState *qts)
{
    qtest_writel(qts, XOSC_CTRL, 0xaa0);
    qtest_writel(qts, XOSC_STARTUP, 0x2f);
    qtest_writel(qts, XOSC_CTRL + SET, 0xfab << 12);
    qtest_clock_step(qts, XOSC_STARTUP_NS + 1);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==,
                    XOSC_STABLE);
    qtest_writel(qts, CLK_SYS_CTRL, 0);
    qtest_writel(qts, CLK_REF_CTRL, CLK_REF_XOSC);
}

static void aon_start(QTestState *qts, uint32_t use)
{
    int i;

    for (i = 0; i < 4; i++) {
        qtest_writel(qts, PM_SET_TIME(i), PM_PASSWORD);
    }
    qtest_writel(qts, PM_TIMER, PM_PASSWORD | TIMER_RUN | use);
}

static void aon_alarm(QTestState *qts, uint32_t t)
{
    int i;

    for (i = 0; i < 4; i++) {
        qtest_writel(qts, PM_ALARM_TIME(i),
                     PM_PASSWORD | (i < 2 ? (t >> (16 * i)) & 0xffff : 0));
    }
    qtest_writel(qts, PM_TIMER + SET, PM_PASSWORD | TIMER_ALARM_ENAB);
}

/*
 * The XOSC stops until a GPIO edge wakes it, then restarts with its
 * startup delay; TIMER0 counts clk_ref ticks, so it stands still from
 * entry until the XOSC is stable again.
 */
/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.gpio/test] */
static void test_xosc_gpio_wake(void)
{
    QTestState *qts = start();
    uint32_t edge = EDGE_HIGH << (4 * PIN);
    uint32_t us;

    run_from_xosc(qts);
    qtest_writel(qts, GPIO_CTRL(PIN), FUNC_SIO);
    qtest_writel(qts, PAD(PIN), PAD_IE);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", PIN, 0);
    qtest_writel(qts, INTR0, edge);
    qtest_writel(qts, DORMANT_WAKE_INTE0, edge);

    qtest_writel(qts, XOSC_DORMANT, DORMANT_DORMANT);
    us = qtest_readl(qts, TIMER0_TIMERAWL);
    g_assert_cmphex(qtest_readl(qts, XOSC_DORMANT), ==, DORMANT_DORMANT);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) &
                    (XOSC_STABLE | XOSC_ENABLED), ==, XOSC_ENABLED);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)) &
                    TICK_RUNNING, ==, 0);
    qtest_clock_step(qts, 50 * 1000 * 1000);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, us);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);

    /* The rising edge wakes it; the clocks wait for the startup delay. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", PIN, 1);
    g_assert_cmphex(qtest_readl(qts, XOSC_DORMANT), ==, DORMANT_WAKE);
    qtest_clock_step(qts, XOSC_STARTUP_NS - 1000);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, us);
    qtest_clock_step(qts, 1001);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==,
                    XOSC_STABLE);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)) &
                    TICK_RUNNING, ==, TICK_RUNNING);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL) - us, ==, 1000);

    /*
     * The edge stays latched in INTR, so the wake event is still asserted:
     * the next DORMANT restarts the XOSC at once.
     */
    qtest_writel(qts, XOSC_DORMANT, DORMANT_DORMANT);
    g_assert_cmphex(qtest_readl(qts, XOSC_DORMANT), ==, DORMANT_WAKE);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);
    qtest_clock_step(qts, XOSC_STARTUP_NS + 1);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==,
                    XOSC_STABLE);
    qtest_quit(qts);
}

/*
 * A level interrupt wakes too, as does an AON timer alarm when the timer
 * runs from the LPOSC. An AON timer running from the XOSC stops with it,
 * so its alarm cannot wake the chip.
 */
/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.powman/test] */
static void test_xosc_alarm_wake(void)
{
    QTestState *qts = start();
    uint32_t t;

    run_from_xosc(qts);
    aon_start(qts, TIMER_USE_XOSC);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    aon_alarm(qts, 20);
    qtest_writel(qts, XOSC_DORMANT, DORMANT_DORMANT);
    t = qtest_readl(qts, PM_READ_TIME_LOWER);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmpuint(qtest_readl(qts, PM_READ_TIME_LOWER), ==, t);
    g_assert_cmphex(qtest_readl(qts, PM_TIMER) & TIMER_ALARM, ==, 0);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);

    /* Wake with a level interrupt instead. */
    qtest_writel(qts, GPIO_CTRL(PIN), FUNC_SIO);
    qtest_writel(qts, PAD(PIN), PAD_IE);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", PIN, 1);
    qtest_writel(qts, DORMANT_WAKE_INTE0, LEVEL_HIGH << (4 * PIN));
    qtest_clock_step(qts, XOSC_STARTUP_NS + 1);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==,
                    XOSC_STABLE);
    qtest_writel(qts, DORMANT_WAKE_INTE0, 0);
    qtest_quit(qts);

    /* The AON timer on the LPOSC: its alarm wakes the XOSC. */
    qts = start();
    run_from_xosc(qts);
    aon_start(qts, TIMER_USE_LPOSC);
    aon_alarm(qts, 20);
    qtest_writel(qts, XOSC_DORMANT, DORMANT_DORMANT);
    qtest_clock_step(qts, 19 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);
    qtest_clock_step(qts, 2 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, PM_TIMER) & TIMER_ALARM, ==,
                    TIMER_ALARM);
    g_assert_cmphex(qtest_readl(qts, XOSC_DORMANT), ==, DORMANT_WAKE);
    qtest_clock_step(qts, XOSC_STARTUP_NS + 1);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==,
                    XOSC_STABLE);
    qtest_quit(qts);
}

/*
 * clk_ref and clk_sys run from the ROSC from reset. DORMANT stops it
 * (COUNT and TIMER0, which counts clk_sys with SOURCE set, stand still)
 * until the alarm restarts it about 1us later.
 */
/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_rosc(void)
{
    QTestState *qts = start();
    uint32_t count, us;

    aon_start(qts, TIMER_USE_LPOSC);
    aon_alarm(qts, 5);
    qtest_writel(qts, TIMER0_SOURCE, 1);
    qtest_writel(qts, ROSC_COUNT, 0xffff);
    qtest_writel(qts, ROSC_DORMANT, DORMANT_DORMANT);
    count = qtest_readl(qts, ROSC_COUNT);
    us = qtest_readl(qts, TIMER0_TIMERAWL);
    g_assert_cmphex(qtest_readl(qts, ROSC_DORMANT), ==, DORMANT_DORMANT);
    g_assert_cmphex(qtest_readl(qts, ROSC_STATUS) &
                    (ROSC_STABLE | ROSC_ENABLED | ROSC_DIV_RUNNING), ==,
                    ROSC_ENABLED);
    qtest_clock_step(qts, 4 * 1000 * 1000);
    g_assert_cmpuint(qtest_readl(qts, ROSC_COUNT), ==, count);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, us);

    qtest_clock_step(qts, 2 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, ROSC_DORMANT), ==, DORMANT_WAKE);
    qtest_clock_step(qts, 1001);
    g_assert_cmphex(qtest_readl(qts, ROSC_STATUS) &
                    (ROSC_STABLE | ROSC_ENABLED | ROSC_DIV_RUNNING), ==,
                    ROSC_STABLE | ROSC_ENABLED | ROSC_DIV_RUNNING);
    qtest_clock_step(qts, 1000);
    g_assert_cmpuint(qtest_readl(qts, ROSC_COUNT), <, count);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), !=, us);
    qtest_quit(qts);
}

/* An invalid DORMANT write selects WAKE and sets BADWRITE. */
/* [spec:nuos:req:emu.clocks/test] */
static void test_badwrite(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, XOSC_DORMANT), ==, DORMANT_WAKE);
    qtest_writel(qts, XOSC_DORMANT, 0x1234);
    g_assert_cmphex(qtest_readl(qts, XOSC_DORMANT), ==, DORMANT_WAKE);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_BADWRITE, ==,
                    XOSC_BADWRITE);
    qtest_writel(qts, XOSC_STATUS + CLR, XOSC_BADWRITE);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_BADWRITE, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-dormant-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/dormant/xosc-gpio-wake", test_xosc_gpio_wake);
    qtest_add_func("/rp2350/dormant/xosc-alarm-wake", test_xosc_alarm_wake);
    qtest_add_func("/rp2350/dormant/rosc", test_rosc);
    qtest_add_func("/rp2350/dormant/badwrite", test_badwrite);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
