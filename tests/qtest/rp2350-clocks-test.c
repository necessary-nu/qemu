/*
 * QTest testcase for RP2350 clock generation: CLOCKS, XOSC, PLLs, TICKS
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Each test replays the register sequence pico-sdk uses and checks the
 * status bits it then waits on.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SET 0x2000
#define CLR 0x3000

#define CLOCKS          0x40010000
#define CLK_CTRL(n)     (CLOCKS + (n) * 0xc)
#define CLK_DIV(n)      (CLOCKS + (n) * 0xc + 4)
#define CLK_SELECTED(n) (CLOCKS + (n) * 0xc + 8)
#define CLK_REF         4
#define CLK_SYS         5
#define CLK_PERI        6
#define FC0_SRC         (CLOCKS + 0xa0)
#define FC0_STATUS      (CLOCKS + 0xa4)

#define XOSC            0x40048000
#define XOSC_CTRL       (XOSC + 0x0)
#define XOSC_STATUS     (XOSC + 0x4)
#define XOSC_STARTUP    (XOSC + 0xc)
#define XOSC_STABLE     (1u << 31)
#define XOSC_ENABLED    (1u << 12)

#define PLL_SYS         0x40050000
#define PLL_USB         0x40058000
#define PLL_CS          0x0
#define PLL_PWR         0x4
#define PLL_FBDIV_INT   0x8
#define PLL_PRIM        0xc
#define PLL_LOCK        (1u << 31)
#define PLL_PWR_PD      (1u << 0)
#define PLL_PWR_VCOPD   (1u << 5)
#define PLL_PWR_POSTDIVPD (1u << 3)

#define TICKS           0x40108000
#define TICK_CTRL(n)    (TICKS + (n) * 0xc)
#define TICK_CYCLES(n)  (TICKS + (n) * 0xc + 4)
#define TICK_TIMER0     2
#define TICK_RUNNING    (1u << 1)

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* [spec:nuos:req:emu.clocks/test] */
static void test_xosc(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);

    /* xosc_init(): range, startup delay, enable, wait for STABLE. */
    qtest_writel(qts, XOSC_CTRL, 0xaa0);
    qtest_writel(qts, XOSC_STARTUP, 0x2f);
    qtest_writel(qts, XOSC_CTRL + SET, 0xfab << 12);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) &
                    (XOSC_STABLE | XOSC_ENABLED), ==,
                    XOSC_STABLE | XOSC_ENABLED);

    /* xosc_disable(): write the DISABLE keyword, wait for !STABLE. */
    qtest_writel(qts, XOSC_CTRL, (0xd1e << 12) | 0xaa0);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);
    qtest_quit(qts);
}

static void pll_init(QTestState *qts, uint32_t pll, uint32_t fbdiv)
{
    qtest_writel(qts, pll + PLL_PWR, 0x2d);
    qtest_writel(qts, pll + PLL_FBDIV_INT, 0);
    qtest_writel(qts, pll + PLL_CS, 1);
    qtest_writel(qts, pll + PLL_FBDIV_INT, fbdiv);
    qtest_writel(qts, pll + PLL_PWR + CLR, PLL_PWR_PD | PLL_PWR_VCOPD);
}

/* [spec:nuos:req:emu.clocks/test] */
static void test_pll(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS), ==, 1);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_PWR), ==, 0x2d);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_PRIM), ==, 0x77000);

    /* pll_init(pll_sys, 1, 1500 MHz, 5, 2): lock once powered. */
    pll_init(qts, PLL_SYS, 125);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) & PLL_LOCK, ==,
                    PLL_LOCK);
    qtest_writel(qts, PLL_SYS + PLL_PRIM, (5 << 16) | (2 << 12));
    qtest_writel(qts, PLL_SYS + PLL_PWR + CLR, PLL_PWR_POSTDIVPD);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_PWR), ==, 0x4);

    /* The USB PLL is independent, and won't lock out of range. */
    g_assert_cmphex(qtest_readl(qts, PLL_USB + PLL_CS) & PLL_LOCK, ==, 0);
    pll_init(qts, PLL_USB, 8);
    g_assert_cmphex(qtest_readl(qts, PLL_USB + PLL_CS) & PLL_LOCK, ==, 0);
    pll_init(qts, PLL_USB, 100);
    g_assert_cmphex(qtest_readl(qts, PLL_USB + PLL_CS) & PLL_LOCK, ==,
                    PLL_LOCK);

    /* Powering down loses lock. */
    qtest_writel(qts, PLL_SYS + PLL_PWR + SET, PLL_PWR_PD);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) & PLL_LOCK, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.clocks/test] */
static void test_clock_muxes(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CLK_DIV(CLK_SYS)), ==, 0x10000);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_SYS)), ==, 0x41);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_SYS)), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x1);

    /* runtime_init_clocks(): move clk_sys and clk_ref off aux sources. */
    qtest_writel(qts, CLK_CTRL(CLK_SYS) + CLR, 0x1);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_SYS)), ==, 0x1);
    qtest_writel(qts, CLK_CTRL(CLK_REF) + CLR, 0x3);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x1);

    /* clock_configure(clk_ref, XOSC): SELECTED follows SRC. */
    qtest_writel(qts, CLK_CTRL(CLK_REF), 0x2);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x4);

    /* Clocks without a glitchless mux read SELECTED as 1. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI), 0x800);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_PERI)), ==, 0x1);

    /* SELECTED is read-only. */
    qtest_writel(qts, CLK_SELECTED(CLK_REF), 0x8);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x4);

    /* frequency_count_khz(): the counter is done at once. */
    qtest_writel(qts, FC0_SRC, 1);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS) & 0x110, ==, 0x10);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.clocks/test] */
static void test_ticks(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 0);

    /* tick_start(TICK_TIMER0, 12): cycles, then enable. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==,
                    1 | TICK_RUNNING);
    g_assert_cmphex(qtest_readl(qts, TICK_CYCLES(TICK_TIMER0)), ==, 12);

    qtest_writel(qts, TICK_CTRL(TICK_TIMER0) + CLR, 1);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 0);

    /* CYCLES is nine bits; RUNNING is read-only. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 0xffff);
    g_assert_cmphex(qtest_readl(qts, TICK_CYCLES(TICK_TIMER0)), ==, 0x1ff);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), TICK_RUNNING);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-clocks-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/clocks/xosc", test_xosc);
    qtest_add_func("/rp2350/clocks/pll", test_pll);
    qtest_add_func("/rp2350/clocks/muxes", test_clock_muxes);
    qtest_add_func("/rp2350/clocks/ticks", test_ticks);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
