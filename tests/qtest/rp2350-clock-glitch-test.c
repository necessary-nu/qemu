/*
 * QTest testcase for RP2350 clock glitches: changing an auxiliary mux's
 * select while its generator passes it
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Each auxiliary mux switches with a glitch (datasheet, "Multiplexers").
 * Each test changes one the documented way (the glitchless mux switched
 * off it and the switch waited for, the generator stopped and the stop
 * waited for, or the destinations held in reset), which keeps everything
 * running, and the ways that break the rule, which stop the processors'
 * clock (clk_sys) or leave the peripherals clocked undefined: here, not
 * answering until they are reset.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define SET             RP2350_ALIAS_SET
#define CLR             RP2350_ALIAS_CLR
#define US              1000

#define CLK_CTRL(n)     RP2350_CLK_CTRL(n)
#define CLK_DIV(n)      RP2350_CLK_DIV(n)
#define CLK_SELECTED(n) RP2350_CLK_SELECTED(n)
#define CLK_GPOUT0      0
#define CLK_REF         RP2350_CLK_REF
#define CLK_SYS         RP2350_CLK_SYS
#define CLK_PERI        RP2350_CLK_PERI
#define CLK_HSTX        RP2350_CLK_HSTX
#define CLK_USB         RP2350_CLK_USB
#define CLK_ADC         RP2350_CLK_ADC
#define CTRL_ENABLE     RP2350_CLK_CTRL_ENABLE
#define CTRL_ENABLED    RP2350_CLK_CTRL_ENABLED
#define CTRL_KILL       (1u << 10)
#define AUX(v)          ((v) << 5)
#define REF_XOSC        2
#define REF_AUX         1
#define REF_AUX_PLL_USB 0
#define REF_AUX_XOSC    3       /* clksrc_pll_usb_primary_ref_opcg */
#define SYS_AUX_PLL_SYS 0
#define SYS_AUX_ROSC    2
#define SYS_AUX_GPIN0   4
#define PERI_CLK_SYS    0
#define PERI_PLL_USB    2
#define GPOUT_XOSC      6
#define GPOUT_CLK_SYS   8

#define CLOCKS          RP2350_CLOCKS_BASE
#define RESUS_CTRL      (CLOCKS + 0x84)
#define RESUS_STATUS    (CLOCKS + 0x88)
#define RESUS_ENABLE    (1u << 8)
#define RESUS_CLEAR     (1u << 16)
#define FC0_REF_KHZ     (CLOCKS + 0x8c)
#define FC0_MIN_KHZ     (CLOCKS + 0x90)
#define FC0_MAX_KHZ     (CLOCKS + 0x94)
#define FC0_INTERVAL    (CLOCKS + 0x9c)
#define FC0_SRC         (CLOCKS + 0xa0)
#define FC0_STATUS      (CLOCKS + 0xa4)
#define FC0_RESULT      (CLOCKS + 0xa8)
#define FC0_DONE        (1u << 4)
#define FC0_RUNNING     (1u << 8)
#define FC0_WAITING     (1u << 12)
#define FC_CLK_REF      0x08
#define FC_CLK_SYS      0x09
#define FC_CLK_PERI     0x0a

#define RESETS_SET      0x40022000
#define RESET_ADC       (1u << 0)
#define RESET_HSTX      (1u << 3)
#define RESET_SPI0      (1u << 18)
#define RESET_SPI1      (1u << 19)
#define RESET_TIMER0    (1u << 23)
#define RESET_UART0     (1u << 26)
#define RESET_UART1     (1u << 27)
#define RESET_USBCTRL   (1u << 28)
#define PERI_DESTS      (RESET_UART0 | RESET_UART1 | RESET_SPI0 | RESET_SPI1)

/* Registers that read non-zero while their block answers. */
#define UART0_PERIPHID0 0x40070fe0
#define UART1_PERIPHID0 0x40078fe0
#define SPI0_PERIPHID0  0x40080fe0
#define SPI1_PERIPHID0  0x40088fe0
#define PL011_ID0       0x11
#define PL022_ID0       0x22
#define ADC_DIV         0x400a0010
#define HSTX_CSR        0x400c0000
#define HSTX_CSR_RESET  0x10050600
#define USB_MUXING      0x50110074

#define TICKS           0x40108000
#define TICK_CTRL(n)    (TICKS + (n) * 0xc)
#define TICK_CYCLES(n)  (TICKS + (n) * 0xc + 4)
#define TICK_TIMER0     2
#define TIMER0_TIMERAWL 0x400b0028

#define WD_CTRL         0x400d8000
#define WD_CTRL_TRIGGER (1u << 31)
#define PSM_WDSEL       0x40018008
#define PSM_CLOCKS      (1u << 5)

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* frequency_count_khz() against a 12 MHz clk_ref. */
static void fc0_start(QTestState *qts, uint32_t src)
{
    qtest_writel(qts, FC0_REF_KHZ, 12000);
    qtest_writel(qts, FC0_INTERVAL, 10);
    qtest_writel(qts, FC0_MIN_KHZ, 0);
    qtest_writel(qts, FC0_MAX_KHZ, 0x1ffffff);
    qtest_writel(qts, FC0_SRC, src);
}

static uint32_t fc0_khz(QTestState *qts, uint32_t src)
{
    int i;

    fc0_start(qts, src);
    for (i = 0; i < 100 && (qtest_readl(qts, FC0_STATUS) & FC0_RUNNING);
         i++) {
        qtest_clock_step(qts, 100 * US);
    }
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS) &
                    (FC0_DONE | FC0_RUNNING), ==, FC0_DONE);
    return qtest_readl(qts, FC0_RESULT) >> 5;
}

/* A count of a stopped clock waits for it to start. */
static void fc0_waits(QTestState *qts, uint32_t src)
{
    fc0_start(qts, src);
    qtest_clock_step(qts, 2000 * US);
    g_assert_cmphex(qtest_readl(qts, FC0_STATUS), ==,
                    FC0_RUNNING | FC0_WAITING);
    qtest_writel(qts, FC0_SRC, 0);
    qtest_clock_step(qts, 2000 * US);
}

static void expect_sys_runs(QTestState *qts, uint32_t khz)
{
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), ==, khz);
}

static void expect_peri_answer(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, UART0_PERIPHID0), ==, PL011_ID0);
    g_assert_cmphex(qtest_readl(qts, UART1_PERIPHID0), ==, PL011_ID0);
    g_assert_cmphex(qtest_readl(qts, SPI0_PERIPHID0), ==, PL022_ID0);
    g_assert_cmphex(qtest_readl(qts, SPI1_PERIPHID0), ==, PL022_ID0);
}

static void expect_peri_silent(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, UART0_PERIPHID0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, UART1_PERIPHID0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI0_PERIPHID0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI1_PERIPHID0), ==, 0);
}

/* Reset `blocks` through RESETS and take them out of reset again. */
static void reset_blocks(QTestState *qts, uint32_t blocks)
{
    qtest_writel(qts, RESETS_SET, blocks);
    rp2350_unreset(qts, blocks);
}

/* A watchdog reset of the PSM's CLOCKS stage alone. */
static void reset_clocks(QTestState *qts)
{
    qtest_writel(qts, PSM_WDSEL, PSM_CLOCKS);
    qtest_writel(qts, WD_CTRL, WD_CTRL_TRIGGER);
}

/*
 * pico-sdk's clock set-up, every subsystem out of reset, then clk_sys
 * from clk_ref (the 12 MHz XOSC), so that a glitch on clk_ref reaches
 * the processors' clock; clk_peri and clk_hstx run from clk_sys.
 */
static QTestState *start_sys_on_ref(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    rp2350_clk_switch(qts, 0, CLK_SYS, CLR, 1, 1);
    expect_sys_runs(qts, 12000);
    expect_peri_answer(qts);
    return qts;
}

/*
 * clk_ref onto its aux mux the documented way: AUXSRC changed while the
 * glitchless mux is off the aux mux (on the XOSC), then SRC.
 */
static void ref_onto_aux(QTestState *qts)
{
    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_XOSC) | REF_XOSC);
    rp2350_clk_switch(qts, 0, CLK_REF, 0, AUX(REF_AUX_XOSC) | REF_AUX,
                      1u << REF_AUX);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_REF), ==, 12000);
}

/*
 * pico-sdk's clock_configure() of clk_ref onto PLL_USB / 4 through the
 * aux mux, with clk_sys running from clk_ref: the glitchless mux switched
 * off the aux mux, which takes two cycles of it (12 MHz) with SELECTED
 * still showing it, and two of the new input (the ROSC's 12.4 MHz
 * rosc_clksrc_ph) with SELECTED 0; then AUXSRC, then SRC back. No glitch.
 */
/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_ref_safe(void)
{
    QTestState *qts = start_sys_on_ref();

    ref_onto_aux(qts);
    qtest_writel(qts, CLK_DIV(CLK_REF), 4 << 16);
    qtest_writel(qts, CLK_CTRL(CLK_REF) + CLR, 3);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x2);
    qtest_clock_step(qts, 160);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x2);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0);
    qtest_clock_step(qts, 150);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0);
    qtest_clock_step(qts, 20);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0x1);
    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_PLL_USB));
    rp2350_clk_switch(qts, 0, CLK_REF, 0, AUX(REF_AUX_PLL_USB) | REF_AUX,
                      1u << REF_AUX);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_REF), ==, 12000);
    expect_sys_runs(qts, 12000);
    expect_peri_answer(qts);
    qtest_quit(qts);
}

/*
 * One write of clk_ref's CTRL that switches its glitchless mux off the
 * aux mux and changes AUXSRC with it: the mux still passes the aux mux
 * as it glitches. The glitch reaches clk_sys, running from clk_ref, which
 * stops, and clk_peri, from clk_sys, whose blocks stop answering.
 * clk_ref runs on, and with it the tick generators and TIMER0. The
 * resuscitation circuit does not restart clk_sys, whose generator still
 * runs; a reset of CLOCKS does.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_ref_one_write(void)
{
    QTestState *qts = start_sys_on_ref();
    uint32_t t;

    ref_onto_aux(qts);
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    qtest_writel(qts, RESUS_CTRL, RESUS_ENABLE | 10);

    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_PLL_USB) | REF_XOSC);
    fc0_waits(qts, FC_CLK_SYS);
    fc0_waits(qts, FC_CLK_PERI);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_REF), ==, 12000);
    expect_peri_silent(qts);
    g_assert_cmphex(qtest_readl(qts, RESUS_STATUS), ==, 0);
    t = qtest_readl(qts, TIMER0_TIMERAWL);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL) - t, ==, 100);

    /* A reset of CLOCKS restarts clk_sys, from the ROSC. */
    reset_clocks(qts);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), >, 11000);
    /* The blocks stay undefined until they are reset themselves. */
    expect_peri_silent(qts);
    reset_blocks(qts, PERI_DESTS);
    expect_peri_answer(qts);
    qtest_quit(qts);
}

/*
 * A switch off the aux mux is complete only once SELECTED shows the new
 * input: AUXSRC changed while SELECTED still shows the aux mux (its first
 * two cycles), or reads 0 (two cycles of the new input), glitches; once
 * SELECTED shows the new input, it does not.
 */
/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_ref_window(void)
{
    static const int64_t waits[] = { 0, 100, 170, 300 };
    QTestState *qts;
    int i;

    for (i = 0; i < ARRAY_SIZE(waits); i++) {
        qts = start_sys_on_ref();
        ref_onto_aux(qts);
        qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_XOSC) | REF_XOSC);
        if (waits[i]) {
            qtest_clock_step(qts, waits[i]);
        }
        g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==,
                        waits[i] < 167 ? 0x2 : 0);
        qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_PLL_USB) | REF_XOSC);
        fc0_waits(qts, FC_CLK_SYS);
        qtest_quit(qts);
    }

    qts = start_sys_on_ref();
    ref_onto_aux(qts);
    rp2350_clk_switch(qts, 0, CLK_REF, 0, AUX(REF_AUX_XOSC) | REF_XOSC,
                      1u << REF_XOSC);
    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_PLL_USB) | REF_XOSC);
    expect_sys_runs(qts, 12000);
    expect_peri_answer(qts);
    qtest_quit(qts);
}

/*
 * One write that switches clk_ref's glitchless mux onto the aux mux and
 * changes AUXSRC with it does not glitch: the mux takes the aux mux up
 * only two cycles of its old input later, by when the aux mux has
 * settled. AUXSRC changed while the mux takes it up (SELECTED 0) does.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_ref_onto_aux(void)
{
    QTestState *qts = start_sys_on_ref();

    rp2350_clk_switch(qts, 0, CLK_REF, 0, AUX(REF_AUX_XOSC) | REF_AUX,
                      1u << REF_AUX);
    expect_sys_runs(qts, 12000);
    expect_peri_answer(qts);
    qtest_quit(qts);

    qts = start_sys_on_ref();
    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_XOSC) | REF_AUX);
    qtest_clock_step(qts, 170);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_REF)), ==, 0);
    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_PLL_USB) | REF_AUX);
    fc0_waits(qts, FC_CLK_SYS);
    qtest_quit(qts);
}

/*
 * A glitch on clk_ref while clk_sys runs from PLL_SYS reaches nothing
 * modelled: the processors and peripherals run on, and so does TIMER0.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_ref_alone(void)
{
    QTestState *qts = start();
    uint32_t t;

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, TICK_CYCLES(TICK_TIMER0), 12);
    qtest_writel(qts, TICK_CTRL(TICK_TIMER0), 1);
    ref_onto_aux(qts);
    qtest_writel(qts, CLK_CTRL(CLK_REF), AUX(REF_AUX_PLL_USB) | REF_XOSC);
    expect_sys_runs(qts, 150000);
    expect_peri_answer(qts);
    t = qtest_readl(qts, TIMER0_TIMERAWL);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_TIMERAWL) - t, ==, 100);
    qtest_quit(qts);
}

/*
 * clk_sys as reset leaves it, from the ROSC through its aux mux, written
 * 0 in one go: SRC to clk_ref and AUXSRC away from the ROSC together.
 * The processors' clock stops, and clk_peri, from it, leaves its blocks
 * undefined; a reset of CLOCKS restarts clk_sys.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_sys_one_write(void)
{
    QTestState *qts = start();

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_SYS)), ==,
                    AUX(SYS_AUX_ROSC) | 1);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), >, 11000);
    expect_peri_answer(qts);

    qtest_writel(qts, CLK_CTRL(CLK_SYS), 0);
    fc0_waits(qts, FC_CLK_SYS);
    fc0_waits(qts, FC_CLK_PERI);
    expect_peri_silent(qts);

    reset_clocks(qts);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), >, 11000);
    qtest_quit(qts);
}

/*
 * runtime_init_clocks(): clk_sys's glitchless mux switched off the aux
 * mux (SRC alone cleared) and the switch waited for, then AUXSRC away
 * from the ROSC: no glitch. An AUXSRC change before the switch completes
 * glitches.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_sys_safe(void)
{
    QTestState *qts = start();

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE);
    rp2350_clk_switch(qts, 0, CLK_SYS, CLR, 1, 1);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), AUX(SYS_AUX_PLL_SYS));
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_SYS), >, 11000);
    expect_peri_answer(qts);
    qtest_quit(qts);

    qts = start();
    qtest_writel(qts, CLK_CTRL(CLK_SYS) + CLR, 1);
    g_assert_cmphex(qtest_readl(qts, CLK_SELECTED(CLK_SYS)), ==, 0x2);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), AUX(SYS_AUX_PLL_SYS));
    fc0_waits(qts, FC_CLK_SYS);
    qtest_quit(qts);
}

/*
 * While the resuscitation circuit has switched clk_sys to clk_ref, the
 * aux mux is not passed: clocks_handle_resus() changing AUXSRC with SRC
 * still on the aux mux does not glitch.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_sys_resus(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    qtest_writel(qts, RESUS_CTRL, RESUS_ENABLE | 100);
    rp2350_clk_switch(qts, 0, CLK_SYS, 0, AUX(SYS_AUX_PLL_SYS), 1);
    qtest_writel(qts, CLK_CTRL(CLK_SYS), AUX(SYS_AUX_GPIN0));
    rp2350_clk_switch(qts, 0, CLK_SYS, 0, AUX(SYS_AUX_GPIN0) | 1, 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(qtest_readl(qts, RESUS_STATUS), ==, 1);
    expect_sys_runs(qts, 12000);

    qtest_writel(qts, CLK_CTRL(CLK_SYS), AUX(SYS_AUX_PLL_SYS) | 1);
    qtest_writel(qts, RESUS_CTRL + SET, RESUS_CLEAR);
    qtest_writel(qts, RESUS_CTRL, 0);
    expect_sys_runs(qts, 150000);
    qtest_quit(qts);
}

/*
 * clk_peri, which has no glitchless mux: AUXSRC changed while it runs
 * leaves UART0/1 and SPI0/1 undefined, not answering, though RESET_DONE
 * reads them out of reset, until each is reset; so does a change within
 * the two source cycles ENABLED takes to clear after ENABLE does, or in
 * the write that clears ENABLE. Stopped and the stop waited for, or with
 * its blocks held in reset, it switches cleanly.
 */
/* [spec:nuos:req:emu.clocks/test] */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_peri(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    expect_peri_answer(qts);

    /* Running. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE | AUX(PERI_PLL_USB));
    expect_peri_silent(qts);
    g_assert_cmphex(qtest_readl(qts, RP2350_RESETS_RESET_DONE) & PERI_DESTS,
                    ==, PERI_DESTS);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 48000);
    reset_blocks(qts, RESET_UART0);
    g_assert_cmphex(qtest_readl(qts, UART0_PERIPHID0), ==, PL011_ID0);
    g_assert_cmphex(qtest_readl(qts, SPI0_PERIPHID0), ==, 0);
    reset_blocks(qts, PERI_DESTS);
    expect_peri_answer(qts);

    /* Stopping: ENABLED reads set for two cycles of PLL_USB, 41.7 ns. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI) + CLR, CTRL_ENABLE);
    qtest_clock_step(qts, 40);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_PERI)) & CTRL_ENABLED, ==,
                    CTRL_ENABLED);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), AUX(PERI_CLK_SYS));
    expect_peri_silent(qts);
    reset_blocks(qts, PERI_DESTS);

    /* ENABLE cleared and AUXSRC changed in one write. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE | AUX(PERI_CLK_SYS));
    qtest_writel(qts, CLK_CTRL(CLK_PERI), AUX(PERI_PLL_USB));
    expect_peri_silent(qts);
    reset_blocks(qts, PERI_DESTS);

    /* clock_configure(): stopped, the stop waited for, then switched. */
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE | AUX(PERI_CLK_SYS));
    rp2350_clk_stop(qts, 0, CLK_PERI);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), AUX(PERI_PLL_USB));
    qtest_writel(qts, CLK_CTRL(CLK_PERI) + SET, CTRL_ENABLE);
    expect_peri_answer(qts);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 48000);

    /* Its blocks held in reset while it switches running. */
    qtest_writel(qts, RESETS_SET, PERI_DESTS);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE | AUX(PERI_CLK_SYS));
    rp2350_unreset(qts, PERI_DESTS);
    expect_peri_answer(qts);
    g_assert_cmpuint(fc0_khz(qts, FC_CLK_PERI), ==, 150000);
    qtest_quit(qts);
}

/*
 * KILL stops a generator asynchronously, which can glitch it: set while
 * clk_peri runs, it leaves its blocks undefined, unless they are held in
 * reset.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_kill(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, CLK_CTRL(CLK_PERI) + SET, CTRL_KILL);
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_PERI)) & CTRL_ENABLED, ==,
                    0);
    expect_peri_silent(qts);
    reset_blocks(qts, PERI_DESTS);
    qtest_writel(qts, CLK_CTRL(CLK_PERI), CTRL_ENABLE);
    expect_peri_answer(qts);

    qtest_writel(qts, RESETS_SET, PERI_DESTS);
    qtest_writel(qts, CLK_CTRL(CLK_PERI) + SET, CTRL_KILL);
    rp2350_unreset(qts, PERI_DESTS);
    expect_peri_answer(qts);
    qtest_quit(qts);
}

/*
 * clk_hstx, clk_usb and clk_adc, each switched running: its block stops
 * answering until it is reset.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_other_peripheral_clocks(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);

    g_assert_cmphex(qtest_readl(qts, HSTX_CSR), ==, HSTX_CSR_RESET);
    qtest_writel(qts, CLK_CTRL(CLK_HSTX), CTRL_ENABLE | AUX(1));
    g_assert_cmphex(qtest_readl(qts, HSTX_CSR), ==, 0);
    reset_blocks(qts, RESET_HSTX);
    g_assert_cmphex(qtest_readl(qts, HSTX_CSR), ==, HSTX_CSR_RESET);

    g_assert_cmphex(qtest_readl(qts, USB_MUXING), ==, 1);
    qtest_writel(qts, CLK_CTRL(CLK_USB), CTRL_ENABLE | AUX(1));
    g_assert_cmphex(qtest_readl(qts, USB_MUXING), ==, 0);
    reset_blocks(qts, RESET_USBCTRL);
    g_assert_cmphex(qtest_readl(qts, USB_MUXING), ==, 1);

    qtest_writel(qts, ADC_DIV, 0x100);
    g_assert_cmphex(qtest_readl(qts, ADC_DIV), ==, 0x100);
    qtest_writel(qts, CLK_CTRL(CLK_ADC), CTRL_ENABLE | AUX(1));
    g_assert_cmphex(qtest_readl(qts, ADC_DIV), ==, 0);
    reset_blocks(qts, RESET_ADC);
    qtest_writel(qts, ADC_DIV, 0x100);
    g_assert_cmphex(qtest_readl(qts, ADC_DIV), ==, 0x100);

    /* The others are untouched. */
    expect_peri_answer(qts);
    expect_sys_runs(qts, 150000);
    qtest_quit(qts);
}

/*
 * A GPOUT's glitch reaches only the pin it drives, off the chip: switched
 * running, nothing on the chip is disturbed.
 */
/* [spec:nuos:req:emu.clock-tree/test] */
static void test_gpout(void)
{
    QTestState *qts = start();

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, CLK_CTRL(CLK_GPOUT0), CTRL_ENABLE | AUX(GPOUT_CLK_SYS));
    qtest_writel(qts, CLK_CTRL(CLK_GPOUT0), CTRL_ENABLE | AUX(GPOUT_XOSC));
    g_assert_cmphex(qtest_readl(qts, CLK_CTRL(CLK_GPOUT0)), ==,
                    CTRL_ENABLED | CTRL_ENABLE | AUX(GPOUT_XOSC));
    expect_peri_answer(qts);
    expect_sys_runs(qts, 150000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-clock-glitch-test-XXXXXX.bin", &rom_path,
                         &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/clock-glitch/ref-safe", test_ref_safe);
    qtest_add_func("/rp2350/clock-glitch/ref-one-write", test_ref_one_write);
    qtest_add_func("/rp2350/clock-glitch/ref-window", test_ref_window);
    qtest_add_func("/rp2350/clock-glitch/ref-onto-aux", test_ref_onto_aux);
    qtest_add_func("/rp2350/clock-glitch/ref-alone", test_ref_alone);
    qtest_add_func("/rp2350/clock-glitch/sys-one-write", test_sys_one_write);
    qtest_add_func("/rp2350/clock-glitch/sys-safe", test_sys_safe);
    qtest_add_func("/rp2350/clock-glitch/sys-resus", test_sys_resus);
    qtest_add_func("/rp2350/clock-glitch/peri", test_peri);
    qtest_add_func("/rp2350/clock-glitch/kill", test_kill);
    qtest_add_func("/rp2350/clock-glitch/hstx-usb-adc",
                   test_other_peripheral_clocks);
    qtest_add_func("/rp2350/clock-glitch/gpout", test_gpout);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
