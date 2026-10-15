/*
 * QTest helper for RP2350 machines: bringing the clock tree up
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TESTS_QTEST_RP2350_CLOCKS_H
#define TESTS_QTEST_RP2350_CLOCKS_H

#include "libqtest.h"

#define RP2350_CLOCKS_BASE        0x40010000
#define RP2350_CLK_CTRL(n)        (RP2350_CLOCKS_BASE + (n) * 0xc)
#define RP2350_CLK_DIV(n)         (RP2350_CLOCKS_BASE + (n) * 0xc + 0x4)
#define RP2350_CLK_SELECTED(n)    (RP2350_CLOCKS_BASE + (n) * 0xc + 0x8)
#define RP2350_CLK_REF            4
#define RP2350_CLK_SYS            5
#define RP2350_CLK_PERI           6
#define RP2350_CLK_HSTX           7
#define RP2350_CLK_USB            8
#define RP2350_CLK_ADC            9
#define RP2350_CLK_CTRL_ENABLE    (1u << 11)
#define RP2350_CLK_CTRL_ENABLED   (1u << 28)
#define RP2350_ALIAS_SET          0x2000
#define RP2350_ALIAS_CLR          0x3000

#define RP2350_XOSC_BASE          0x40048000
#define RP2350_XOSC_STABLE        (1u << 31)
/* STARTUP.DELAY 47: 47 * 256 periods of the 12 MHz crystal, about 1 ms. */
#define RP2350_XOSC_STARTUP_NS    1002667

#define RP2350_PLL_SYS_BASE       0x40050000
#define RP2350_PLL_USB_BASE       0x40058000
#define RP2350_PLL_LOCK           (1u << 31)
/* 400 cycles of the 12 MHz reference. */
#define RP2350_PLL_LOCK_NS        33334

#define RP2350_CLOCKS_RESETS_CLR  0x40023000
#define RP2350_CLOCKS_RESETS_DONE 0x40020008
#define RP2350_RESET_PLL_SYS      (1u << 14)
#define RP2350_RESET_PLL_USB      (1u << 15)

/* pico-sdk's pll_init() with REFDIV 1; returns the virtual time after. */
static inline int64_t rp2350_pll_init(QTestState *qts, uint32_t pll,
                                      uint32_t fbdiv, uint32_t pd1,
                                      uint32_t pd2)
{
    int64_t now;

    qtest_writel(qts, pll + 0x0, 1);
    qtest_writel(qts, pll + 0x8, fbdiv);
    qtest_writel(qts, pll + 0x4 + 0x3000, 0x21);
    now = qtest_clock_step(qts, RP2350_PLL_LOCK_NS);
    g_assert_cmphex(qtest_readl(qts, pll) & RP2350_PLL_LOCK, ==,
                    RP2350_PLL_LOCK);
    qtest_writel(qts, pll + 0xc, (pd1 << 16) | (pd2 << 12));
    qtest_writel(qts, pll + 0x4 + 0x3000, 0x8);
    return now;
}

/*
 * Write a generator's CTRL through `alias` (0, or the SET or CLR alias),
 * then poll a glitchless mux's SELECTED until it reads `selected`,
 * stepping the clock 10 ns at a time, as software waits for the switch.
 * Returns the virtual time after, or `now` if no step was needed.
 */
static inline int64_t rp2350_clk_switch(QTestState *qts, int64_t now, int clk,
                                        uint32_t alias, uint32_t value,
                                        uint32_t selected)
{
    int i;

    qtest_writel(qts, RP2350_CLK_CTRL(clk) + alias, value);
    for (i = 0; qtest_readl(qts, RP2350_CLK_SELECTED(clk)) != selected; i++) {
        g_assert_cmpint(i, <, 100000);
        now = qtest_clock_step(qts, 10);
    }
    return now;
}

/*
 * Stop a generator without a glitchless mux as pico-sdk's clock_stop()
 * does, then poll ENABLED until it clears (two cycles of its source),
 * stepping the clock 10 ns at a time. Returns the virtual time after, or
 * `now` if no step was needed.
 */
static inline int64_t rp2350_clk_stop(QTestState *qts, int64_t now, int clk)
{
    int i;

    qtest_writel(qts, RP2350_CLK_CTRL(clk) + RP2350_ALIAS_CLR,
                 RP2350_CLK_CTRL_ENABLE);
    for (i = 0; qtest_readl(qts, RP2350_CLK_CTRL(clk)) &
                RP2350_CLK_CTRL_ENABLED; i++) {
        g_assert_cmpint(i, <, 100000);
        now = qtest_clock_step(qts, 10);
    }
    return now;
}

/*
 * Bring the clock tree up as pico-sdk's runtime_init_clocks() does on a
 * Pico 2: clk_ref from the 12 MHz XOSC; clk_sys from PLL_SYS at 150 MHz,
 * and clk_peri and clk_hstx from clk_sys; clk_usb and clk_adc from PLL_USB
 * at 48 MHz, each auxiliary mux changed only while its generator does not
 * pass it. The TICKS generators are left stopped, for each test to start
 * the ones it uses. Returns the virtual time after, a whole number of
 * microseconds.
 */
static inline int64_t rp2350_clocks_init(QTestState *qts)
{
    uint32_t done = RP2350_RESET_PLL_SYS | RP2350_RESET_PLL_USB;
    int64_t now;
    int i;

    qtest_writel(qts, RP2350_XOSC_BASE + 0x0, 0xaa0);
    qtest_writel(qts, RP2350_XOSC_BASE + 0xc, 47);
    qtest_writel(qts, RP2350_XOSC_BASE + 0x2000, 0xfab << 12);
    qtest_clock_step(qts, RP2350_XOSC_STARTUP_NS);
    g_assert_cmphex(qtest_readl(qts, RP2350_XOSC_BASE + 0x4) &
                    RP2350_XOSC_STABLE, ==, RP2350_XOSC_STABLE);

    /* clk_sys and clk_ref switched cleanly off their auxiliary muxes. */
    rp2350_clk_switch(qts, 0, RP2350_CLK_SYS, RP2350_ALIAS_CLR, 0x1, 0x1);
    rp2350_clk_switch(qts, 0, RP2350_CLK_REF, RP2350_ALIAS_CLR, 0x3, 0x1);

    qtest_writel(qts, RP2350_CLOCKS_RESETS_CLR, done);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, RP2350_CLOCKS_RESETS_DONE) & done, ==,
                    done);
    rp2350_pll_init(qts, RP2350_PLL_SYS_BASE, 125, 5, 2);
    now = rp2350_pll_init(qts, RP2350_PLL_USB_BASE, 100, 5, 5);

    now = rp2350_clk_switch(qts, now, RP2350_CLK_REF, 0, 2, 4);
    qtest_writel(qts, RP2350_CLK_DIV(RP2350_CLK_REF), 1 << 16);
    /* clk_sys's AUXSRC to PLL_SYS while off it, then SRC to it. */
    qtest_writel(qts, RP2350_CLK_DIV(RP2350_CLK_SYS), 1 << 16);
    qtest_writel(qts, RP2350_CLK_CTRL(RP2350_CLK_SYS), 0);
    now = rp2350_clk_switch(qts, now, RP2350_CLK_SYS, 0, 1, 2);
    for (i = RP2350_CLK_PERI; i <= RP2350_CLK_ADC; i++) {
        qtest_writel(qts, RP2350_CLK_CTRL(i), RP2350_CLK_CTRL_ENABLE);
        qtest_writel(qts, RP2350_CLK_DIV(i), 1 << 16);
    }
    /*
     * End on a whole microsecond, a whole number of cycles of each clock,
     * so that tests counting cycles from power-on stay aligned to them.
     */
    if (now % 1000) {
        now = qtest_clock_step(qts, 1000 - now % 1000);
    }
    return now;
}

#endif
