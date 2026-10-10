/*
 * QTest testcase for RP2350 clock generation: CLOCKS, XOSC, PLLs, TICKS
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Each test replays the register sequence pico-sdk uses and checks the
 * status bits it then waits on, and the frequencies that result, as the
 * frequency counter measures them.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define SET 0x2000
#define CLR 0x3000

#define CLOCKS          0x40010000
#define CLK_CTRL(n)     (CLOCKS + (n) * 0xc)
#define CLK_DIV(n)      (CLOCKS + (n) * 0xc + 4)
#define CLK_SELECTED(n) (CLOCKS + (n) * 0xc + 8)
#define CLK_GPOUT0      0
#define CLK_REF         4
#define CLK_SYS         5
#define CLK_PERI        6
#define CLK_HSTX        7
#define CLK_USB         8
#define CLK_ADC         9
#define CTRL_ENABLED    (1u << 28)
#define CTRL_ENABLE     (1u << 11)
#define CTRL_KILL       (1u << 10)
#define RESUS_CTRL      (CLOCKS + 0x84)
#define RESUS_STATUS    (CLOCKS + 0x88)
#define RESUS_CLEAR     (1u << 16)
#define RESUS_ENABLE    (1u << 8)
#define FC0_REF_KHZ     (CLOCKS + 0x8c)
#define FC0_MIN_KHZ     (CLOCKS + 0x90)
#define FC0_MAX_KHZ     (CLOCKS + 0x94)
#define FC0_DELAY       (CLOCKS + 0x98)
#define FC0_INTERVAL    (CLOCKS + 0x9c)
#define FC0_SRC         (CLOCKS + 0xa0)
#define FC0_STATUS      (CLOCKS + 0xa4)
#define FC0_RESULT      (CLOCKS + 0xa8)
#define WAKE_EN0        (CLOCKS + 0xac)
#define ENABLED0        (CLOCKS + 0xbc)
#define INTR            (CLOCKS + 0xc4)
#define INTE            (CLOCKS + 0xc8)
#define INTS            (CLOCKS + 0xd0)

#define FC0_PASS        (1u << 0)
#define FC0_DONE        (1u << 4)
#define FC0_RUNNING     (1u << 8)
#define FC0_WAITING     (1u << 12)
#define FC0_FAIL        (1u << 16)
#define FC0_SLOW        (1u << 20)
#define FC0_FAST        (1u << 24)
#define FC0_DIED        (1u << 28)

/* FC0_SRC values. */
#define FC_PLL_SYS      0x01
#define FC_PLL_USB      0x02
#define FC_ROSC         0x03
#define FC_ROSC_PH      0x04
#define FC_XOSC         0x05
#define FC_GPIN0        0x06
#define FC_CLK_REF      0x08
#define FC_CLK_SYS      0x09
#define FC_CLK_PERI     0x0a
#define FC_CLK_USB      0x0b
#define FC_CLK_ADC      0x0c
#define FC_CLK_HSTX     0x0d
#define FC_LPOSC        0x0e

#define XOSC            0x40048000
#define XOSC_CTRL       (XOSC + 0x0)
#define XOSC_STATUS     (XOSC + 0x4)
#define XOSC_STARTUP    (XOSC + 0xc)
#define XOSC_COUNT      (XOSC + 0x10)
#define XOSC_STABLE     (1u << 31)
#define XOSC_BADWRITE   (1u << 24)
#define XOSC_ENABLED    (1u << 12)

#define ROSC_DIV        0x400e8014

#define PLL_SYS         0x40050000
#define PLL_USB         0x40058000
#define PLL_CS          0x0
#define PLL_PWR         0x4
#define PLL_FBDIV_INT   0x8
#define PLL_PRIM        0xc
#define PLL_INTR        0x10
#define PLL_INTE        0x14
#define PLL_INTS        0x1c
#define PLL_LOCK        (1u << 31)
#define PLL_LOCK_N      (1u << 30)
#define PLL_BYPASS      (1u << 8)
#define PLL_PWR_PD      (1u << 0)
#define PLL_PWR_VCOPD   (1u << 5)
#define PLL_PWR_POSTDIVPD (1u << 3)

#define TICKS           0x40108000
#define TICK_CTRL(n)    (TICKS + (n) * 0xc)
#define TICK_CYCLES(n)  (TICKS + (n) * 0xc + 4)
#define TICK_COUNT(n)   (TICKS + (n) * 0xc + 8)
#define TICK_TIMER0     2
#define TICK_RUNNING    (1u << 1)

#define TIMER0_TIMERAWL 0x400b0028

#define CLOCKS_IRQ      30
#define PLL_SYS_IRQ     42

#define US              1000

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    return qts;
}

static void fc0_start(QTestState *qts, uint32_t src, uint32_t ref_khz)
{
    qtest_writel(qts, FC0_REF_KHZ, ref_khz);
    qtest_writel(qts, FC0_INTERVAL, 10);
    qtest_writel(qts, FC0_MIN_KHZ, 0);
    qtest_writel(qts, FC0_MAX_KHZ, 0x1ffffff);
    qtest_writel(qts, FC0_SRC, src);
}

/*
 * frequency_count_khz(): count `src` for the 1 ms interval against a
 * clk_ref of `ref_khz`, returning FC0_RESULT once DONE.
 */
static uint32_t fc0_count(QTestState *qts, uint32_t src, uint32_t ref_khz)
{
    int i;

    fc0_start(qts, src, ref_khz);
    for (i = 0; i < 100 && (qtest_readl(qts, FC0_STATUS) & FC0_RUNNING);
         i++) {
        qtest_clock_step(qts, 100 * US);
    }
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS) &
                    (FC0_DONE | FC0_RUNNING), ==, FC0_DONE);
    return qtest_readl(qts, FC0_RESULT);
}

/* A count of a stopped clock waits for it to start, with no result. */
static void fc0_waits(QTestState *qts, uint32_t src, uint32_t ref_khz)
{
    fc0_start(qts, src, ref_khz);
    qtest_clock_step(qts, 2000 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_RUNNING | FC0_WAITING);
    g_assert_cmphex(qtest_readl(qts, FC0_RESULT), ==, 0);
}

static uint32_t fc0_khz(QTestState *qts, uint32_t src)
{
    return fc0_count(qts, src, 12000) >> 5;
}

/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_xosc(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);

    /*
     * xosc_init(): range, startup delay, enable, wait for STABLE, which
     * comes 0x2f * 256 periods of the 12 MHz crystal (1.003 ms) later.
     * The range write's ENABLE field is no valid code: the XOSC keeps
     * its setting and flags BADWRITE.
     */
    qtest_writel(qts, XOSC_CTRL, 0xaa0);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS), ==, XOSC_BADWRITE);
    qtest_writel(qts, XOSC_STATUS, XOSC_BADWRITE);
    qtest_writel(qts, XOSC_STARTUP, 0x2f);
    qtest_writel(qts, XOSC_CTRL + SET, 0xfab << 12);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) &
                    (XOSC_STABLE | XOSC_ENABLED), ==, XOSC_ENABLED);
    /* No clock until STABLE: the counter waits for it. */
    qtest_writel(qts, FC0_REF_KHZ, 12419);
    qtest_writel(qts, FC0_INTERVAL, 4);
    qtest_writel(qts, FC0_SRC, FC_XOSC);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_RUNNING | FC0_WAITING);
    qtest_clock_step(qts, 902 * US);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) &
                    (XOSC_STABLE | XOSC_ENABLED), ==,
                    XOSC_STABLE | XOSC_ENABLED);
    qtest_clock_step(qts, 50 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS) & (FC0_DONE | FC0_DIED), ==,
                    FC0_DONE);

    /* COUNT runs down at 12 MHz, reading 1 until it reaches zero. */
    qtest_writel(qts, XOSC_COUNT, 1200);
    g_assert_cmphex(qtest_readl(qts, XOSC_COUNT), ==, 1);
    qtest_clock_step(qts, 99 * US);
    g_assert_cmphex(qtest_readl(qts, XOSC_COUNT), ==, 1);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, XOSC_COUNT), ==, 0);

    /* An invalid FREQ_RANGE keeps the range in force. */
    qtest_writel(qts, XOSC_CTRL, (0xfab << 12) | 0x123);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) &
                    (XOSC_STABLE | XOSC_BADWRITE | 3), ==,
                    XOSC_STABLE | XOSC_BADWRITE);

    /* xosc_disable(): write the DISABLE keyword, wait for !STABLE. */
    qtest_writel(qts, XOSC_CTRL, (0xd1e << 12) | 0xaa0);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==, 0);
    qtest_quit(qts);
}

static void pll_power(QTestState *qts, uint32_t pll, uint32_t fbdiv)
{
    qtest_writel(qts, pll + PLL_PWR, 0x2d);
    qtest_writel(qts, pll + PLL_FBDIV_INT, 0);
    qtest_writel(qts, pll + PLL_CS, 1);
    qtest_writel(qts, pll + PLL_FBDIV_INT, fbdiv);
    qtest_writel(qts, pll + PLL_PWR + CLR, PLL_PWR_PD | PLL_PWR_VCOPD);
}

static void xosc_start(QTestState *qts)
{
    qtest_writel(qts, XOSC_CTRL, 0xaa0);
    qtest_writel(qts, XOSC_STARTUP, 0x2f);
    qtest_writel(qts, XOSC_CTRL + SET, 0xfab << 12);
    qtest_clock_step(qts, 1003 * US);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_STABLE, ==,
                    XOSC_STABLE);
}

/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_pll(void)
{
    QTestState *qts = start();

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS), ==, 1);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_PWR), ==, 0x2d);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_PRIM), ==, 0x77000);

    /* Without its reference, the XOSC, the VCO never locks. */
    pll_power(qts, PLL_SYS, 125);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) & PLL_LOCK, ==, 0);

    /*
     * With it, pll_init(pll_sys, 1, 1500 MHz, 5, 2) locks 400 reference
     * cycles later.
     */
    xosc_start(qts);
    pll_power(qts, PLL_SYS, 125);
    qtest_clock_step(qts, 33 * US);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) & PLL_LOCK, ==, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) & PLL_LOCK, ==,
                    PLL_LOCK);
    /* No output until the post dividers are powered. */
    qtest_writel(qts, FC0_REF_KHZ, 12419);
    qtest_writel(qts, FC0_SRC, FC_PLL_SYS);
    qtest_clock_step(qts, 500 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_RUNNING | FC0_WAITING);
    qtest_writel(qts, PLL_SYS + PLL_PRIM, (5 << 16) | (2 << 12));
    qtest_writel(qts, PLL_SYS + PLL_PWR + CLR, PLL_PWR_POSTDIVPD);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_PWR), ==, 0x4);

    /* clk_ref from the XOSC times the frequency counter. */
    qtest_writel(qts, CLK_CTRL(CLK_REF), 2);
    g_assert_cmpuint(fc0_khz(qts, FC_PLL_SYS), ==, 150000);
    /* POSTDIV2 3: 100 MHz. */
    qtest_writel(qts, PLL_SYS + PLL_PRIM, (5 << 16) | (3 << 12));
    g_assert_cmpuint(fc0_khz(qts, FC_PLL_SYS), ==, 100000);
    /* BYPASS passes the reference through. */
    qtest_writel(qts, PLL_SYS + PLL_CS + SET, PLL_BYPASS);
    g_assert_cmpuint(fc0_khz(qts, FC_PLL_SYS), ==, 12000);
    qtest_writel(qts, PLL_SYS + PLL_CS + CLR, PLL_BYPASS);

    /* The USB PLL is independent, and won't lock out of range. */
    g_assert_cmphex(qtest_readl(qts, PLL_USB + PLL_CS) & PLL_LOCK, ==, 0);
    pll_power(qts, PLL_USB, 8);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(qtest_readl(qts, PLL_USB + PLL_CS) & PLL_LOCK, ==, 0);
    pll_power(qts, PLL_USB, 100);
    qtest_clock_step(qts, 34 * US);
    g_assert_cmphex(qtest_readl(qts, PLL_USB + PLL_CS) & PLL_LOCK, ==,
                    PLL_LOCK);

    /*
     * Powering down loses lock, setting LOCK_N and its sticky interrupt,
     * PLL_SYS_IRQ where enabled. Each clears by writing 1.
     */
    qtest_writel(qts, PLL_SYS + PLL_INTE, 1);
    qtest_writel(qts, PLL_SYS + PLL_PWR + SET, PLL_PWR_PD);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) &
                    (PLL_LOCK | PLL_LOCK_N), ==, PLL_LOCK_N);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_INTS), ==, 1);
    g_assert_true(qtest_get_irq(qts, PLL_SYS_IRQ));
    fc0_waits(qts, FC_PLL_SYS, 12000);
    qtest_writel(qts, PLL_SYS + PLL_CS + SET, PLL_LOCK_N);
    g_assert_cmphex(qtest_readl(qts, PLL_SYS + PLL_CS) & PLL_LOCK_N, ==, 0);
    qtest_writel(qts, PLL_SYS + PLL_INTR, 1);
    g_assert_false(qtest_get_irq(qts, PLL_SYS_IRQ));

    /* A PLL held in reset has no output. */
    qtest_writel(qts, 0x40022000, RP2350_RESET_PLL_USB);
    fc0_waits(qts, FC_PLL_USB, 12000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
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

    /*
     * A glitchless mux completes a switch only once the new source runs:
     * switched to the stopped XOSC, clk_ref stops with SELECTED 0, and
     * the tick generators it drives stop running.
     */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==,
                    1 | TICK_RUNNING);
    qtest_writel(qts, CLK_CTRL(CLK_REF), 0x2);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_SYS)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 1);
    xosc_start(qts);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_SYS)), ==, 0x1);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==,
                    1 | TICK_RUNNING);

    /* Clocks without a glitchless mux read SELECTED as 1. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_PERI)), ==, 0x1);

    /* SELECTED is read-only. */
    qtest_writel(qts, CLK_SELECTED(CLK_REF), 0x8);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x4);
    qtest_quit(qts);
}

/*
 * The frequencies from reset: clk_ref and clk_sys from the ROSC's 8-way
 * divided output, about 12.4 MHz in the model, clk_peri and the other
 * gated clocks stopped. After pico-sdk's clock set-up, every clock at its
 * nominal frequency, and following each divider change.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_frequencies(void)
{
    QTestState *qts = start();
    uint32_t rosc;

    /* clk_ref from the ROSC, so FC0_REF_KHZ is its nominal rate. */
    rosc = fc0_count(qts, FC_ROSC, 12419) >> 5;
    g_assert_cmpuint(rosc, >=, 12417);
    g_assert_cmpuint(rosc, <=, 12421);
    g_assert_cmpuint(fc0_count(qts, FC_CLK_SYS, 12419) >> 5, ==, rosc);
    g_assert_cmpuint(fc0_count(qts, FC_CLK_REF, 12419) >> 5, ==, rosc);
    g_assert_cmpuint(fc0_count(qts, FC_LPOSC, 12419) >> 5, >=, 30);
    g_assert_cmpuint(fc0_count(qts, FC_LPOSC, 12419) >> 5, <=, 34);
    /* A NULL source counts nothing, but completes. */
    g_assert_cmpuint(fc0_count(qts, 0, 12419), ==, 0);
    /* clk_peri is stopped from reset: the count waits for it. */
    qtest_writel(qts, FC0_SRC, FC_CLK_PERI);
    qtest_clock_step(qts, 2000 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_RUNNING | FC0_WAITING);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_PERI)) & CTRL_ENABLED, ==,
                    0);
    /* ENABLE starts it, from clk_sys; ENABLED follows. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_PERI)), ==,
                    CTRL_ENABLE | CTRL_ENABLED);
    qtest_clock_step(qts, 2000 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS) & FC0_DONE, ==, FC0_DONE);
    g_assert_cmpuint(qtest_readl(qts, FC0_RESULT) >> 5, ==, rosc);
    /* KILL stops it at once. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI) + SET, CTRL_KILL);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_PERI)) & CTRL_ENABLED, ==,
                    0);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), 0);

    rp2350_clocks_init(qts);
    g_assert_cmpuint(fc0_khz(qts, FC_XOSC), ==, 12000);
    g_assert_cmpuint(fc0_khz(qts, FC_ROSC), ==, 12418);
    g_assert_cmpuint(fc0_khz(qts, FC_ROSC_PH), ==, 12418);
    g_assert_cmpuint(fc0_khz(qts, FC_PLL_SYS), ==, 150000);
    g_assert_cmpuint(fc0_khz(qts, FC_PLL_USB), ==, 48000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_REF), ==, 12000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), ==, 150000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 150000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_USB), ==, 48000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_ADC), ==, 48000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_HSTX), ==, 150000);
    fc0_waits(qts, FC_GPIN0, 12000);

    /* clk_sys divided by 2.5 (fractional), and clk_peri, from it, too. */
    qtest_writel(qts, CLK_DIV(CLK_SYS), 0x28000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), ==, 60000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 60000);
    /* clk_peri's own divider has two integer bits; 0 divides by 4. */
    qtest_writel(qts, CLK_DIV(CLK_SYS), 0x10000);
    qtest_writel(qts, CLK_DIV(CLK_PERI), 0x30000);
    g_assert_cmphex(qtest_readl(qts, CLK_DIV(CLK_PERI)), ==, 0x30000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 50000);
    qtest_writel(qts, CLK_DIV(CLK_PERI), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLK_DIV(CLK_PERI)), ==, 0x30000);
    qtest_writel(qts, CLK_DIV(CLK_PERI), 0);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 37500);
    /* clk_peri from PLL_USB through its aux mux. */
    qtest_writel(qts, CLK_DIV(CLK_PERI), 0x10000);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE | (2 << 5));
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 48000);
    /* clk_adc divided by 3. */
    qtest_writel(qts, CLK_DIV(CLK_ADC), 0x30000);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_ADC), ==, 16000);
    /* clk_usb stopped by its enable. */
    qtest_writel(qts, CLK_CTRL(CLK_USB) + CLR, CTRL_ENABLE);
    fc0_waits(qts, FC_CLK_USB, 12000);
    /* A slower clk_ref lengthens the interval; FC0_REF_KHZ scales it. */
    qtest_writel(qts, CLK_DIV(CLK_REF), 0x20000);
    g_assert_cmpuint(fc0_count(qts, FC_CLK_SYS, 6000) >> 5, ==, 150000);
    g_assert_cmpuint(fc0_count(qts, FC_CLK_SYS, 12000) >> 5, ==, 300000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.clock-tree/test] */
static void test_fc0(void)
{
    QTestState *qts = start();
    uint32_t st;

    rp2350_clocks_init(qts);
    g_assert_cmphex(qtest_readl(qts, FC0_DELAY), ==, 1);
    g_assert_cmphex(qtest_readl(qts, FC0_INTERVAL), ==, 8);
    g_assert_cmphex(qtest_readl(qts, FC0_MAX_KHZ), ==, 0x1ffffff);

    /*
     * The count takes FC0_DELAY plus the interval in clk_ref cycles:
     * 1 + 12000 << 8 >> 10 = 3001 cycles of 12 MHz, 250.08 us.
     */
    qtest_writel(qts, FC0_REF_KHZ, 12000);
    qtest_writel(qts, FC0_SRC, FC_CLK_SYS);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==, FC0_RUNNING);
    qtest_clock_step(qts, 250 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==, FC0_RUNNING);
    g_assert_cmphex(qtest_readl(qts, FC0_RESULT), ==, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==, FC0_DONE | FC0_PASS);
    /* 150 MHz in kHz with five fraction bits. */
    g_assert_cmphex(qtest_readl(qts, FC0_RESULT), ==, 150000 << 5);

    /* Pass and fail against MIN and MAX. */
    qtest_writel(qts, FC0_MIN_KHZ, 149000);
    qtest_writel(qts, FC0_MAX_KHZ, 151000);
    qtest_writel(qts, FC0_SRC, FC_CLK_SYS);
    qtest_clock_step(qts, 300 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==, FC0_DONE | FC0_PASS);
    qtest_writel(qts, FC0_SRC, FC_CLK_USB);
    qtest_clock_step(qts, 300 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_DONE | FC0_FAIL | FC0_SLOW);
    qtest_writel(qts, FC0_MIN_KHZ, 0);
    qtest_writel(qts, FC0_MAX_KHZ, 100000);
    qtest_writel(qts, FC0_SRC, FC_CLK_SYS);
    qtest_clock_step(qts, 300 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_DONE | FC0_FAIL | FC0_FAST);
    qtest_writel(qts, FC0_MAX_KHZ, 0x1ffffff);

    /* A clock that stops during the count dies: half its edges. */
    qtest_writel(qts, FC0_SRC, FC_CLK_ADC);
    qtest_clock_step(qts, 125 * US);
    qtest_writel(qts, CLK_CTRL(CLK_ADC) + CLR, CTRL_ENABLE);
    qtest_clock_step(qts, 200 * US);
    st = qtest_readl(qts, FC0_STATUS);
    g_assert_cmphex(st & (FC0_DONE | FC0_DIED), ==, FC0_DONE | FC0_DIED);
    g_assert_cmpuint(qtest_readl(qts, FC0_RESULT) >> 5, >=, 23900);
    g_assert_cmpuint(qtest_readl(qts, FC0_RESULT) >> 5, <=, 24100);

    /* The shortest interval: about 1 us, to a resolution of 1024 kHz. */
    qtest_writel(qts, FC0_INTERVAL, 0);
    qtest_writel(qts, FC0_SRC, FC_CLK_SYS);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS) & FC0_DONE, ==, FC0_DONE);
    g_assert_cmpuint(qtest_readl(qts, FC0_RESULT) >> 5, >=, 148000);
    g_assert_cmpuint(qtest_readl(qts, FC0_RESULT) >> 5, <=, 152000);
    qtest_quit(qts);
}

/*
 * The resuscitation circuit: clk_sys switched to a source that never
 * runs (GPIN0, unconnected) stops; after TIMEOUT clk_ref cycles resus
 * runs it from clk_ref, flagging RESUSSED and raising CLOCKS_IRQ, until
 * CLEAR.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_resus(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    qtest_writel(qts, INTE, 1);
    qtest_writel(qts, RESUS_CTRL, RESUS_ENABLE | 100);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), 0);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), 4 << 5);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), (4 << 5) | 1);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_SYS)), ==, 0);
    qtest_clock_step(qts, 8 * US);
    g_assert_cmphex(qtest_readl(qts, RESUS_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(qts, CLOCKS_IRQ));
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, RESUS_STATUS), ==, 1);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, 1);
    g_assert_true(qtest_get_irq(qts, CLOCKS_IRQ));
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), ==, 12000);

    /* clocks_handle_resus(): clk_sys from clk_ref, then CLEAR. */
    qtest_writel(qts, CLK_CTRL(CLK_SYS), 0);
    qtest_writel(qts, RESUS_CTRL + SET, RESUS_CLEAR);
    qtest_writel(qts, RESUS_CTRL + CLR, RESUS_CLEAR);
    g_assert_cmphex(qtest_readl(qts, RESUS_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(qts, CLOCKS_IRQ));
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), ==, 12000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
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
    /* COUNT: the clk_ref cycles left before the next tick. */
    g_assert_cmpuint(qtest_readl(qts, TICK_COUNT(TICK_TIMER0)), ==, 12);

    qtest_writel(qts, TICK_CTRL(TICK_TIMER0) + CLR, 1);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 0);

    /* CYCLES is nine bits; RUNNING is read-only. */
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 0xffff);
    g_assert_cmphex(qtest_readl(qts, TICK_CYCLES(TICK_TIMER0)), ==, 0x1ff);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), TICK_RUNNING);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 0);

    /*
     * TIMER0 counts its tick at clk_ref's frequency: from the 12 MHz
     * XOSC, 12 cycles a microsecond; with clk_ref divided by 2, half
     * that; with clk_ref stopped (from GPIN0), not at all.
     */
    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 100);
    qtest_writel(qts, CLK_DIV(CLK_REF), 0x20000);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 150);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), 1);
    qtest_writel(qts, CLK_CTRL(CLK_REF), (1 << 5) | 1);
    g_assert_cmphex(qtest_readl(qts, TICK_CTRL(TICK_TIMER0)), ==, 1);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL), ==, 150);
    qtest_quit(qts);
}

/*
 * After a direct load the clocks are as the boot ROM's early boot path
 * leaves them: ROSC DIV 2 and clk_ref divided by 4.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
/* [spec:nuos:req:emu.direct-load/test] */
static void test_direct_load(void)
{
    static const uint32_t image[2] = { 0x20001000, 0x10000009 };
    char *path;
    QTestState *qts;
    GError *err = NULL;
    uint32_t khz;
    int fd;

    fd = g_file_open_tmp("rp2350-clocks-image-XXXXXX.bin", &path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, image, sizeof(image)), ==, sizeof(image));
    close(fd);
    qts = qtest_initf("-M rp2350,flash-size=4M -kernel %s", path);

    g_assert_cmphex(qtest_readl(qts, ROSC_DIV), ==, 0xaa02);
    g_assert_cmphex(qtest_readl(qts, CLK_DIV(CLK_REF)), ==, 0x40000);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_SYS)), ==, 0x41);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_PERI)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, XOSC_STATUS) & XOSC_ENABLED, ==, 0);
    /* The ROSC at a quarter of its divisor, clk_ref divided by 4. */
    khz = fc0_count(qts, FC_CLK_SYS, 12419) >> 5;
    g_assert_cmpuint(khz, >=, 49670);
    g_assert_cmpuint(khz, <=, 49680);
    khz = fc0_count(qts, FC_CLK_REF, 12419) >> 5;
    g_assert_cmpuint(khz, >=, 12417);
    g_assert_cmpuint(khz, <=, 12421);
    qtest_quit(qts);
    unlink(path);
    g_free(path);
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
    qtest_add_func("/rp2350/clocks/frequencies", test_frequencies);
    qtest_add_func("/rp2350/clocks/fc0", test_fc0);
    qtest_add_func("/rp2350/clocks/resus", test_resus);
    qtest_add_func("/rp2350/clocks/ticks", test_ticks);
    qtest_add_func("/rp2350/clocks/direct-load", test_direct_load);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
