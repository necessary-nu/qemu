/*
 * QTest testcase for the ESP32-P4 timer groups: timers, MWDT and RTC
 * calibration
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Every clock here runs from reset at its reset frequency: the timers and
 * watchdogs on the 40 MHz XTAL_CLK (25 ns a cycle), RC_SLOW_CLK at
 * 150 kHz, RC_FAST_CLK at 20 MHz. Interrupts are seen in the interrupt
 * matrix's status of sources 46 to 51. Without -kernel the super watchdog
 * resets the system after 960 ms, so no test runs that long.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/misc/esp32p4_clkrst.h"

#define TIMG(g)             (0x500c2000 + 0x1000 * (g))
#define TCONFIG(x)          (0x00 + 0x24 * (x))
#define TLO(x)              (0x04 + 0x24 * (x))
#define THI(x)              (0x08 + 0x24 * (x))
#define TUPDATE(x)          (0x0c + 0x24 * (x))
#define TALARMLO(x)         (0x10 + 0x24 * (x))
#define TALARMHI(x)         (0x14 + 0x24 * (x))
#define TLOADLO(x)          (0x18 + 0x24 * (x))
#define TLOADHI(x)          (0x1c + 0x24 * (x))
#define TLOAD(x)            (0x20 + 0x24 * (x))
#define WDTCONFIG0          0x48
#define WDTCONFIG1          0x4c
#define WDTCONFIG(n)        (0x50 + 4 * (n))
#define WDTFEED             0x60
#define WDTWPROTECT         0x64
#define RTCCALICFG          0x68
#define RTCCALICFG1         0x6c
#define INT_ENA             0x70
#define INT_RAW             0x74
#define INT_ST              0x78
#define INT_CLR             0x7c
#define RTCCALICFG2         0x80
#define DATE                0xf8
#define REGCLK              0xfc

#define T_ALARM_EN          (1u << 10)
#define T_DIVCNT_RST        (1u << 12)
#define T_DIVIDER(d)        ((uint32_t)(d) << 13)
#define T_AUTORELOAD        (1u << 29)
#define T_INCREASE          (1u << 30)
#define T_EN                (1u << 31)
#define T_RESET             0x60002000u

#define W_APPCPU_RESET_EN   (1u << 12)
#define W_PROCPU_RESET_EN   (1u << 13)
#define W_FLASHBOOT         (1u << 14)
#define W_SYS_LEN(n)        ((uint32_t)(n) << 15)
#define W_CONF_UPDATE       (1u << 22)
#define W_STG(n, a)         ((uint32_t)(a) << (29 - 2 * (n)))
#define W_EN                (1u << 31)
#define W_RESET0            0x0004c000u
#define W_PRESCALE(p)       ((uint32_t)(p) << 16)
#define WKEY                0x50d83aa1u
#define STG_INT             1
#define STG_CPU             2
#define STG_CORE            3

#define CAL_CYCLING         (1u << 12)
#define CAL_RDY             (1u << 15)
#define CAL_MAX(n)          ((uint32_t)(n) << 16)
#define CAL_START           (1u << 31)
#define CAL1_VLD            1u
#define CAL1_VALUE(v)       ((v) >> 7)
#define CAL2_TIMEOUT        1u
#define CAL2_RST_CNT(n)     ((uint32_t)(n) << 3)
#define CAL2_THRES(n)       ((uint32_t)(n) << 7)
#define CAL2_RESET          0xffffff98u

#define TIMG0_PATH          "/machine/timg[0]"
#define CLKRST              "/machine/clkrst"
#define HP_CPU0             "/machine/hp-cluster/cpu[0]"
#define HP_CPU1             "/machine/hp-cluster/cpu[1]"

#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL2       (HP_CLKRST + 0x1c)
#define PERI_CLK_CTRL20     (HP_CLKRST + 0x94)
#define PERI_CLK_CTRL21     (HP_CLKRST + 0x98)
#define HP_RST_EN0          (HP_CLKRST + 0xc0)
#define HP_RST_EN1          (HP_CLKRST + 0xc4)
#define HPWDT_CORE0_RST     (HP_CLKRST + 0xd4)
#define WDT_RESET_SOURCE0   (HP_CLKRST + 0xec)
#define PERI_CLK_CTRL20_RESET 0xc9000000u
#define T0_SRC_RC_FAST      (1u << 22)
#define T0_CLK_EN           (1u << 24)
#define TGRT_SEL(s)         (s)
#define TGRT_DIV(d)         ((uint32_t)(d) << 4)
#define TGRT_MASK           0x000fffffu
#define RST_EN_CORE1        (1u << 8)
#define RST_EN_TIMERGRP0    (1u << 6)
#define TIMERGRP0_APB_CLK_EN (1u << 21)

#define LP_CLKRST           0x50111000
#define RESET_CAUSE         (LP_CLKRST + 0x10)
#define HPCPU_RESET_CTRL0   (LP_CLKRST + 0x14)
#define CORE0_CAUSE(v)      (((v) >> 7) & 0x3f)
#define CORE0_FLAG(v)       (((v) >> 13) & 1)
#define CORE1_CAUSE(v)      (((v) >> 14) & 0x3f)
#define CAUSE_CLEAR_ALL     0xf0000000u

#define LP_WDT_SWD_CONFIG   0x5011601c
#define LP_WDT_SWD_WPROTECT 0x50116020
#define SWD_DISABLE         (1u << 30)
#define LP_TIMER_UPDATE     0x50112010
#define LP_TIMER_BUF0_LO    0x50112014

#define INTMTX_MAP(src)     (0x500d6000 + 4 * (src))
#define INTMTX_STATUS1      0x500d6204
#define SRC_TG0_T0          46

#define XTAL_NS             25
#define US                  1000ll
#define MS                  (1000 * US)

static int64_t now;

/*
 * A board strapped for Joint Download Boot (GPIO36 high, GPIO35 low), so
 * that the watchdogs' flash boot protection does not run.
 */
static QTestState *start(void)
{
    now = 0;
    return qtest_init("-M esp32p4,strap-mode=0x04");
}

static void step(QTestState *qts, int64_t ns)
{
    now = qtest_clock_step(qts, ns);
}

static uint32_t rd(QTestState *qts, int g, uint32_t off)
{
    return qtest_readl(qts, TIMG(g) + off);
}

static void wr(QTestState *qts, int g, uint32_t off, uint32_t v)
{
    qtest_writel(qts, TIMG(g) + off, v);
}

/* Latch and read timer @x of group @g. */
static uint64_t count(QTestState *qts, int g, int x)
{
    wr(qts, g, TUPDATE(x), 1);
    g_assert_cmphex(rd(qts, g, TUPDATE(x)), ==, 0);
    return ((uint64_t)rd(qts, g, THI(x)) << 32) | rd(qts, g, TLO(x));
}

static void load(QTestState *qts, int g, int x, uint64_t v)
{
    wr(qts, g, TLOADLO(x), (uint32_t)v);
    wr(qts, g, TLOADHI(x), v >> 32);
    wr(qts, g, TLOAD(x), 1);
}

static void set_alarm(QTestState *qts, int g, int x, uint64_t v)
{
    wr(qts, g, TALARMLO(x), (uint32_t)v);
    wr(qts, g, TALARMHI(x), v >> 32);
}

/* Source @src's level in the interrupt matrix. */
static bool source(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS1) >> (src - 32)) & 1;
}

static bool qom_bool(QTestState *qts, const char *path, const char *prop)
{
    return qtest_qom_get_bool(qts, path, prop);
}

/* Disable the super watchdog, for a test longer than its timeout. */
static void swd_off(QTestState *qts)
{
    qtest_writel(qts, LP_WDT_SWD_WPROTECT, WKEY);
    qtest_writel(qts, LP_WDT_SWD_CONFIG,
                 qtest_readl(qts, LP_WDT_SWD_CONFIG) | SWD_DISABLE);
}

/* Every register of both groups at its reset value. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_reset(void)
{
    QTestState *qts = start();
    static const uint32_t holds[] = { 26000000, 0x7ffffff, 0xfffff,
                                      0xfffff };
    int g, x, n;

    for (g = 0; g < 2; g++) {
        for (x = 0; x < 2; x++) {
            g_assert_cmphex(rd(qts, g, TCONFIG(x)), ==, T_RESET);
            for (n = 1; n < 9; n++) {
                g_assert_cmphex(rd(qts, g, TCONFIG(x) + 4 * n), ==, 0);
            }
        }
        g_assert_cmphex(rd(qts, g, WDTCONFIG0), ==, W_RESET0);
        g_assert_cmphex(rd(qts, g, WDTCONFIG1), ==, 0x00010000);
        for (n = 0; n < 4; n++) {
            g_assert_cmphex(rd(qts, g, WDTCONFIG(n)), ==, holds[n]);
        }
        g_assert_cmphex(rd(qts, g, WDTFEED), ==, 0);
        g_assert_cmphex(rd(qts, g, WDTWPROTECT), ==, WKEY);
        g_assert_cmphex(rd(qts, g, RTCCALICFG), ==, 0x00011000);
        g_assert_cmphex(rd(qts, g, RTCCALICFG1), ==, 0);
        g_assert_cmphex(rd(qts, g, RTCCALICFG2), ==, CAL2_RESET);
        g_assert_cmphex(rd(qts, g, INT_ENA), ==, 0);
        g_assert_cmphex(rd(qts, g, INT_RAW), ==, 0);
        g_assert_cmphex(rd(qts, g, INT_ST), ==, 0);
        g_assert_cmphex(rd(qts, g, DATE), ==, 0x02209142);
        g_assert_cmphex(rd(qts, g, REGCLK), ==, 0x10000000);
    }
    qtest_quit(qts);
}

/*
 * A timer counts TB_CLK, its function clock divided by TIMG_Tx_DIVIDER,
 * up or down; 0 divides by 65536, 1 and 2 by 2.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_counting(void)
{
    QTestState *qts = start();

    wr(qts, 0, TCONFIG(0), T_EN | T_INCREASE | T_DIVIDER(40));
    step(qts, 1000 * US - 1);
    g_assert_cmpuint(count(qts, 0, 0), ==, 999);
    step(qts, 1);
    g_assert_cmpuint(count(qts, 0, 0), ==, 1000);
    /* The latch holds until the next update. */
    step(qts, 10 * US);
    g_assert_cmphex(rd(qts, 0, TLO(0)), ==, 1000);

    /* Turned round, it counts down at once; stopped, it holds. */
    wr(qts, 0, TCONFIG(0), T_EN | T_DIVIDER(40));
    step(qts, 20 * US);
    g_assert_cmpuint(count(qts, 0, 0), ==, 990);
    wr(qts, 0, TCONFIG(0), T_DIVIDER(40));
    step(qts, 20 * US);
    g_assert_cmpuint(count(qts, 0, 0), ==, 990);

    /* Below 0 it wraps at 54 bits. */
    load(qts, 0, 0, 1);
    wr(qts, 0, TCONFIG(0), T_EN | T_DIVIDER(40) | T_DIVCNT_RST);
    step(qts, 3 * US);
    g_assert_cmphex(count(qts, 0, 0), ==, 0x3ffffffffffffeull);
    g_assert_cmphex(rd(qts, 0, THI(0)), ==, 0x3fffff);

    /* The divider's 0, 1 and 2. */
    wr(qts, 0, TCONFIG(1), T_EN | T_INCREASE | T_DIVIDER(1));
    step(qts, 100 * XTAL_NS);
    g_assert_cmpuint(count(qts, 0, 1), ==, 50);
    wr(qts, 0, TCONFIG(1), T_INCREASE | T_DIVIDER(2) | T_DIVCNT_RST);
    load(qts, 0, 1, 0);
    wr(qts, 0, TCONFIG(1), T_EN | T_INCREASE | T_DIVIDER(2));
    step(qts, 100 * XTAL_NS);
    g_assert_cmpuint(count(qts, 0, 1), ==, 50);
    wr(qts, 0, TCONFIG(1), T_INCREASE | T_DIVIDER(0) | T_DIVCNT_RST);
    load(qts, 0, 1, 0);
    wr(qts, 0, TCONFIG(1), T_EN | T_INCREASE | T_DIVIDER(0));
    step(qts, 65536 * XTAL_NS * 3 - 1);
    g_assert_cmpuint(count(qts, 0, 1), ==, 2);
    step(qts, 1);
    g_assert_cmpuint(count(qts, 0, 1), ==, 3);

    /* Its function clock: gated it stops; from RC_FAST_CLK, 20 MHz. */
    wr(qts, 0, TCONFIG(1), T_INCREASE | T_DIVIDER(20) | T_DIVCNT_RST);
    load(qts, 0, 1, 0);
    wr(qts, 0, TCONFIG(1), T_EN | T_INCREASE | T_DIVIDER(20));
    qtest_writel(qts, PERI_CLK_CTRL20, PERI_CLK_CTRL20_RESET & ~(1u << 27));
    step(qts, 10 * US);
    g_assert_cmpuint(count(qts, 0, 1), ==, 0);
    qtest_writel(qts, PERI_CLK_CTRL20, PERI_CLK_CTRL20_RESET);
    step(qts, 10 * US);
    g_assert_cmpuint(count(qts, 0, 1), ==, 20);
    qtest_writel(qts, PERI_CLK_CTRL20,
                 PERI_CLK_CTRL20_RESET | (1u << 25));
    step(qts, 10 * US);
    g_assert_cmpuint(count(qts, 0, 1), ==, 30);
    qtest_quit(qts);
}

/*
 * An alarm at the count, which clears ALARM_EN and raises TGn_Tx_INT;
 * auto-reload; an alarm value already passed; a down-counting alarm.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_alarm(void)
{
    QTestState *qts = start();
    uint32_t cfg = T_EN | T_INCREASE | T_DIVIDER(2);

    wr(qts, 0, INT_ENA, 1);
    load(qts, 0, 0, 0);
    set_alarm(qts, 0, 0, 1000);
    wr(qts, 0, TCONFIG(0), cfg | T_AUTORELOAD | T_ALARM_EN);
    step(qts, 2000 * XTAL_NS - 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    g_assert(!source(qts, SRC_TG0_T0));
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 1);
    g_assert_cmphex(rd(qts, 0, INT_ST), ==, 1);
    g_assert(source(qts, SRC_TG0_T0));
    g_assert_cmphex(rd(qts, 0, TCONFIG(0)), ==, cfg | T_AUTORELOAD);
    /* Reloaded at the alarm, it counts on from 0. */
    step(qts, 20 * XTAL_NS);
    g_assert_cmpuint(count(qts, 0, 0), ==, 10);
    wr(qts, 0, INT_CLR, 1);
    g_assert(!source(qts, SRC_TG0_T0));

    /* Without auto-reload it counts on past the alarm. */
    set_alarm(qts, 0, 0, 100);
    wr(qts, 0, TCONFIG(0), cfg | T_ALARM_EN);
    step(qts, 180 * XTAL_NS);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 1);
    g_assert_cmpuint(count(qts, 0, 0), ==, 100);
    step(qts, 20 * XTAL_NS);
    g_assert_cmpuint(count(qts, 0, 0), ==, 110);
    wr(qts, 0, INT_RAW, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);

    /*
     * An alarm value up to 2^53 - 1 behind alarms at once; one 2^53 or
     * more behind is reached after the counter wraps.
     */
    set_alarm(qts, 0, 0, 50);
    wr(qts, 0, TCONFIG(0), cfg | T_ALARM_EN);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 1);
    wr(qts, 0, INT_CLR, 1);
    load(qts, 0, 0, (1ull << 53) + 10);
    set_alarm(qts, 0, 0, 10);
    wr(qts, 0, TCONFIG(0), cfg | T_ALARM_EN);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    /* Writing the alarm value compares the new one. */
    set_alarm(qts, 0, 0, 11);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 1);
    wr(qts, 0, INT_CLR, 1);

    /* The comparison needs the function clock, but not EN. */
    load(qts, 0, 0, 500);
    set_alarm(qts, 0, 0, 400);
    wr(qts, 0, TCONFIG(0), T_INCREASE | T_DIVIDER(2) | T_ALARM_EN);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 1);
    wr(qts, 0, INT_CLR, 1);

    /* Counting down, the alarm comes as the count falls to it. */
    load(qts, 0, 0, 1000);
    set_alarm(qts, 0, 0, 900);
    wr(qts, 0, TCONFIG(0), T_EN | T_DIVIDER(2) | T_ALARM_EN);
    step(qts, 200 * XTAL_NS - 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 1);
    wr(qts, 0, INT_CLR, 1);

    /* Timer 1 and timer group 1's interrupts are their own sources. */
    wr(qts, 0, INT_ENA, 2);
    wr(qts, 1, INT_ENA, 3);
    load(qts, 0, 1, 0);
    set_alarm(qts, 0, 1, 1);
    load(qts, 1, 0, 0);
    set_alarm(qts, 1, 0, 1);
    load(qts, 1, 1, 0);
    set_alarm(qts, 1, 1, 1);
    wr(qts, 0, TCONFIG(1), cfg | T_ALARM_EN);
    wr(qts, 1, TCONFIG(0), cfg | T_ALARM_EN);
    wr(qts, 1, TCONFIG(1), cfg | T_ALARM_EN);
    step(qts, 2 * XTAL_NS);
    g_assert(source(qts, SRC_TG0_T0 + 1));
    g_assert(source(qts, SRC_TG0_T0 + 3));
    g_assert(source(qts, SRC_TG0_T0 + 4));
    g_assert(!source(qts, SRC_TG0_T0));
    qtest_quit(qts);
}

/* The timers' ETM tasks act while TIMG_ETM_EN is set. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_etm_tasks(void)
{
    QTestState *qts = start();
    int t1 = 5;

    wr(qts, 0, TCONFIG(1), T_INCREASE | T_DIVIDER(2));
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 0, 1);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 0, 0);
    g_assert_cmphex(rd(qts, 0, TCONFIG(1)), ==,
                    T_EN | T_INCREASE | T_DIVIDER(2));
    step(qts, 20 * XTAL_NS);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 4, 1);
    g_assert_cmphex(rd(qts, 0, TLO(1)), ==, 10);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 4, 0);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 2, 1);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 2, 0);
    g_assert_cmphex(rd(qts, 0, TCONFIG(1)), ==, T_INCREASE | T_DIVIDER(2));
    wr(qts, 0, TLOADLO(1), 77);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 3, 1);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 3, 0);
    g_assert_cmpuint(count(qts, 0, 1), ==, 77);
    set_alarm(qts, 0, 1, 70);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 1, 1);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 1, 0);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 2);

    /* Without TIMG_ETM_EN they do nothing. */
    wr(qts, 0, REGCLK, 0);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 0, 1);
    qtest_set_irq_in(qts, TIMG0_PATH, "etm-task", t1 + 0, 0);
    g_assert_cmphex(rd(qts, 0, TCONFIG(1)), ==, T_INCREASE | T_DIVIDER(2));
    qtest_quit(qts);
}

/* The MWDT's write protection, stages, feeding and configuration update. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_wdt_stages(void)
{
    QTestState *qts = start();

    /* Out of reset it is unlocked; another key locks it. */
    wr(qts, 0, WDTWPROTECT, 0);
    wr(qts, 0, WDTCONFIG0, W_EN);
    wr(qts, 0, WDTCONFIG(0), 5);
    g_assert_cmphex(rd(qts, 0, WDTCONFIG0), ==, W_RESET0);
    g_assert_cmphex(rd(qts, 0, WDTCONFIG(0)), ==, 26000000);
    wr(qts, 0, WDTWPROTECT, WKEY);

    /*
     * A 1 us tick: stage 0 interrupts after 100, stage 1 does nothing
     * after 50, stage 2 interrupts after 20, stage 3 does nothing after 30.
     */
    wr(qts, 0, INT_ENA, 4);
    wr(qts, 0, WDTCONFIG1, W_PRESCALE(40));
    wr(qts, 0, WDTCONFIG(0), 100);
    wr(qts, 0, WDTCONFIG(1), 50);
    wr(qts, 0, WDTCONFIG(2), 20);
    wr(qts, 0, WDTCONFIG(3), 30);
    wr(qts, 0, WDTCONFIG0, W_STG(0, STG_INT) | W_STG(2, STG_INT) |
                           W_SYS_LEN(1) | (1u << 18) | W_CONF_UPDATE);
    g_assert_cmphex(rd(qts, 0, WDTCONFIG0), ==,
                    W_STG(0, STG_INT) | W_STG(2, STG_INT) | W_SYS_LEN(1) |
                    (1u << 18));
    step(qts, 1000 * US);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    wr(qts, 0, WDTCONFIG0, rd(qts, 0, WDTCONFIG0) | W_EN);
    step(qts, 100 * US - 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    g_assert(source(qts, SRC_TG0_T0 + 2));
    wr(qts, 0, INT_CLR, 4);
    step(qts, 70 * US - 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    wr(qts, 0, INT_CLR, 4);
    /* Through stage 3 and round to stage 0 again. */
    step(qts, 130 * US - 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    wr(qts, 0, INT_CLR, 4);

    /* Feeding restarts stage 0. */
    wr(qts, 0, WDTFEED, 0);
    step(qts, 90 * US);
    wr(qts, 0, WDTFEED, 0);
    step(qts, 99 * US);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    step(qts, 1 * US);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    wr(qts, 0, INT_CLR, 4);
    /* A locked watchdog cannot be fed. */
    wr(qts, 0, WDTWPROTECT, 1);
    step(qts, 50 * US);
    wr(qts, 0, WDTFEED, 0);
    step(qts, 20 * US);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    wr(qts, 0, INT_CLR, 4);
    wr(qts, 0, WDTWPROTECT, WKEY);

    /* Holds and the prescaler change only with CONF_UPDATE_EN. */
    wr(qts, 0, WDTFEED, 0);
    wr(qts, 0, WDTCONFIG(0), 10);
    wr(qts, 0, WDTCONFIG1, W_PRESCALE(4));
    step(qts, 20 * US);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    wr(qts, 0, WDTFEED, 0);
    wr(qts, 0, WDTCONFIG0, rd(qts, 0, WDTCONFIG0) | W_CONF_UPDATE);
    step(qts, 10 * 4 * XTAL_NS - 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    wr(qts, 0, INT_CLR, 4);

    /* Disabled, it holds stage 0 at 0. */
    wr(qts, 0, WDTCONFIG0, rd(qts, 0, WDTCONFIG0) & ~W_EN);
    step(qts, 10 * MS);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 0);
    wr(qts, 0, WDTCONFIG0, rd(qts, 0, WDTCONFIG0) | W_EN);
    step(qts, 10 * 4 * XTAL_NS);
    g_assert_cmphex(rd(qts, 0, INT_RAW), ==, 4);
    qtest_quit(qts);
}

/*
 * The MWDT's CPU reset: of each core its *_RESET_EN lets the reset reach
 * and whose HPCORE<n>_WDT_RESET_SOURCE_SEL picks the MWDT, cause 0x0B.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_wdt_cpu_reset(void)
{
    QTestState *qts = start();

    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    wr(qts, 0, WDTCONFIG1, W_PRESCALE(40));
    wr(qts, 0, WDTCONFIG(0), 10);
    wr(qts, 0, WDTCONFIG0, W_STG(0, STG_CPU) | W_CONF_UPDATE | W_EN);
    step(qts, 10 * US);
    g_assert(!qom_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);

    wr(qts, 0, WDTCONFIG0, W_PROCPU_RESET_EN | W_STG(0, STG_CPU) |
                           W_CONF_UPDATE | W_EN);
    wr(qts, 0, WDTFEED, 0);
    step(qts, 10 * US);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x0b);
    g_assert_cmphex(CORE0_FLAG(qtest_readl(qts, RESET_CAUSE)), ==, 1);
    /* HPWDT_CORE0_RST_CTRL0: a stall of 8 XTAL_CLK cycles, a reset of 8. */
    g_assert(!qom_bool(qts, HP_CPU0, "in-reset"));
    g_assert(!qom_bool(qts, HP_CPU0, "clock-running"));
    step(qts, 8 * XTAL_NS);
    g_assert(qom_bool(qts, HP_CPU0, "in-reset"));
    step(qts, 8 * XTAL_NS);
    g_assert(!qom_bool(qts, HP_CPU0, "in-reset"));
    g_assert(qom_bool(qts, HP_CPU0, "clock-running"));

    /* Core 0 takes MWDT1's only if its source select picks MWDT1. */
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    wr(qts, 0, WDTCONFIG0, 0);
    wr(qts, 1, WDTCONFIG1, W_PRESCALE(40));
    wr(qts, 1, WDTCONFIG(0), 10);
    wr(qts, 1, WDTCONFIG0, W_PROCPU_RESET_EN | W_STG(0, STG_CPU) |
                           W_CONF_UPDATE | W_EN);
    step(qts, 10 * US);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    qtest_writel(qts, WDT_RESET_SOURCE0, 3);
    wr(qts, 1, WDTFEED, 0);
    step(qts, 10 * US);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x0b);
    qtest_quit(qts);
}

/*
 * The MWDT's HP core reset, cause 0x07 for both HP cores: the HP blocks,
 * the HP cores and HP_SYS_CLKRST reset; the LP system does not.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_wdt_core_reset(void)
{
    QTestState *qts = start();
    uint32_t cause, lp_before;

    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    qtest_writel(qts, HP_RST_EN0, 0);
    qtest_writel(qts, HPCPU_RESET_CTRL0,
                 qtest_readl(qts, HPCPU_RESET_CTRL0) & ~(1u << 15));
    qtest_writel(qts, PERI_CLK_CTRL20, PERI_CLK_CTRL20_RESET | 0x7f8);
    qtest_writel(qts, INTMTX_MAP(SRC_TG0_T0), 20);
    qtest_writel(qts, LP_TIMER_UPDATE, 1u << 28);
    lp_before = qtest_readl(qts, LP_TIMER_BUF0_LO);
    wr(qts, 0, TALARMLO(1), 1234);

    wr(qts, 0, WDTCONFIG1, W_PRESCALE(40));
    wr(qts, 0, WDTCONFIG(0), 100);
    wr(qts, 0, WDTCONFIG0, W_STG(0, STG_CORE) | W_SYS_LEN(7) |
                           W_CONF_UPDATE | W_EN);
    step(qts, 100 * US - 1);
    g_assert(!qom_bool(qts, HP_CPU0, "in-reset"));
    step(qts, 1);
    g_assert(qom_bool(qts, HP_CPU0, "in-reset"));
    g_assert(qom_bool(qts, HP_CPU1, "in-reset"));
    cause = qtest_readl(qts, RESET_CAUSE);
    g_assert_cmphex(CORE0_CAUSE(cause), ==, 0x07);
    g_assert_cmphex(CORE1_CAUSE(cause), ==, 0x07);
    /* The LP core's cause is still the chip reset's. */
    g_assert_cmphex(cause & 0x7f, ==, 0x41);
    /* Held for SYS_RESET_LENGTH 7, 3.2 us; then core 1 is held again. */
    step(qts, 3200 - 1);
    g_assert(qom_bool(qts, HP_CPU0, "in-reset"));
    step(qts, 1);
    g_assert(!qom_bool(qts, HP_CPU0, "in-reset"));
    g_assert(qom_bool(qts, HP_CPU1, "in-reset"));

    /* The HP side reset: the timer group, HP_SYS_CLKRST, the HP cores. */
    g_assert_cmphex(rd(qts, 0, TALARMLO(1)), ==, 0);
    g_assert_cmphex(rd(qts, 0, WDTCONFIG0), ==, W_RESET0);
    g_assert_cmphex(qtest_readl(qts, PERI_CLK_CTRL20), ==,
                    PERI_CLK_CTRL20_RESET);
    g_assert_cmphex(qtest_readl(qts, HP_RST_EN0), ==, RST_EN_CORE1);
    g_assert_cmphex(qtest_readl(qts, INTMTX_MAP(SRC_TG0_T0)), ==, 0);
    /* The LP side not: LP_CLKRST and the RTC timer run on. */
    g_assert_cmphex(qtest_readl(qts, HPCPU_RESET_CTRL0) & (1u << 15), ==, 0);
    qtest_writel(qts, LP_TIMER_UPDATE, 1u << 28);
    g_assert_cmpuint(qtest_readl(qts, LP_TIMER_BUF0_LO) - lp_before, ==, 15);

    /* MWDT1's is ignored while neither core's source select picks it. */
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    qtest_writel(qts, WDT_RESET_SOURCE0, 0);
    wr(qts, 1, WDTCONFIG1, W_PRESCALE(40));
    wr(qts, 1, WDTCONFIG(0), 10);
    wr(qts, 1, WDTCONFIG0, W_STG(0, STG_CORE) | W_CONF_UPDATE | W_EN);
    step(qts, 10 * US);
    g_assert(!qom_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    qtest_quit(qts);
}

/*
 * Flash boot protection: booted in SPI boot mode, MWDT0 runs with
 * TIMG_WDT_EN clear and resets the HP core after stage 0's 26000000
 * cycles; clearing FLASHBOOT_MOD_EN stops it.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_wdt_flashboot(void)
{
    QTestState *qts = start();

    swd_off(qts);
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    step(qts, 1 * MS);
    qtest_set_irq_in(qts, TIMG0_PATH, "spi-boot", 0, 1);
    step(qts, 26000000ll * XTAL_NS - 1);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    step(qts, 1);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x07);
    g_assert(qom_bool(qts, HP_CPU0, "in-reset"));
    step(qts, 1000);

    /*
     * After the reset it runs again, until software clears
     * FLASHBOOT_MOD_EN, which takes effect with CONF_UPDATE_EN.
     */
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    step(qts, 100 * MS);
    wr(qts, 0, WDTCONFIG0, (W_RESET0 & ~W_FLASHBOOT) | W_CONF_UPDATE);
    step(qts, 700 * MS);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    wr(qts, 0, WDTCONFIG0, W_RESET0 | W_CONF_UPDATE);
    step(qts, 26000000ll * XTAL_NS - 1);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    step(qts, 1);
    g_assert_cmphex(CORE0_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x07);
    qtest_quit(qts);
}

/* Start a one-shot calibration of @max cycles of TIMERGRP0_TGRT_CLK. */
static void cal_start(QTestState *qts, uint32_t max)
{
    wr(qts, 0, RTCCALICFG, CAL_MAX(max));
    wr(qts, 0, RTCCALICFG, CAL_MAX(max) | CAL_START);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, 0);
}

/* The calibration clock: source @sel of TIMERGRP0_TGRT_CLK, over @div. */
static void cal_clock(QTestState *qts, uint32_t sel, uint32_t div)
{
    qtest_writel(qts, PERI_CLK_CTRL21,
                 (qtest_readl(qts, PERI_CLK_CTRL21) & ~TGRT_MASK) |
                 TGRT_SEL(sel) | TGRT_DIV(div - 1));
}

static void assert_cal(QTestState *qts, uint32_t want)
{
    uint32_t got = CAL1_VALUE(rd(qts, 0, RTCCALICFG1));

    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, CAL_RDY);
    g_assert_cmpuint(got, >=, want);
    g_assert_cmpuint(got, <=, want + 1);
}

/*
 * RTC calibration: XTAL_CLK's count over RTC_CALI_MAX cycles of each
 * clock the calibration clock selects, as ESP-IDF's rtc_clk_cal drives
 * it, giving the modelled frequencies; a stopped clock times out.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_calibration(void)
{
    QTestState *qts = start();

    /* Out of reset it cycles on MPLL_CLK, stopped: a threshold of 1 ends it. */
    step(qts, 1 * US);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG2) & CAL2_TIMEOUT, ==, 0);
    wr(qts, 0, RTCCALICFG2, CAL2_RST_CNT(3) | CAL2_THRES(1));
    g_assert_cmphex(rd(qts, 0, RTCCALICFG2) & CAL2_TIMEOUT, ==, 1);
    wr(qts, 0, RTCCALICFG, CAL_MAX(1));
    wr(qts, 0, RTCCALICFG2, CAL2_RESET);

    /* RC_SLOW_CLK, 150 kHz: 1024 cycles are 273066.7 XTAL_CLK cycles. */
    cal_clock(qts, 8, 1);
    cal_start(qts, 1024);
    step(qts, 6800 * US);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, 0);
    step(qts, 50 * US);
    assert_cal(qts, 273066);

    /* RC_FAST_CLK over 50, 400 kHz: 102400. */
    cal_clock(qts, 7, 50);
    cal_start(qts, 1024);
    step(qts, 2600 * US);
    assert_cal(qts, 102399);

    /*
     * RC32K_CLK, which the PMU has powered down: with ESP-IDF's
     * threshold, cycles << 12, it times out.
     */
    cal_clock(qts, 9, 1);
    wr(qts, 0, RTCCALICFG2, CAL2_RST_CNT(3) | CAL2_THRES(1024 << 12));
    cal_start(qts, 1024);
    step(qts, (1024 << 12) * XTAL_NS);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG2) & CAL2_TIMEOUT, ==, 0);
    step(qts, XTAL_NS);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG2) & CAL2_TIMEOUT, ==, 1);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, 0);

    /* Powered, 32768 Hz: 1250000; a new start clears the timeout. */
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_RC32K, 1);
    cal_start(qts, 1024);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG2) & CAL2_TIMEOUT, ==, 0);
    step(qts, 32 * MS);
    assert_cal(qts, 1250000);

    /* XTAL32K_CLK, the board's 32768 Hz crystal, likewise. */
    cal_clock(qts, 10, 1);
    cal_start(qts, 1024);
    step(qts, 32 * MS);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, 0);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_XTAL32K, 1);
    cal_start(qts, 1024);
    step(qts, 32 * MS);
    assert_cal(qts, 1250000);

    /* Cycling: each measurement's end sets CYCLING_DATA_VLD. */
    cal_clock(qts, 8, 1);
    wr(qts, 0, RTCCALICFG, CAL_MAX(150));
    wr(qts, 0, RTCCALICFG, CAL_MAX(150) | CAL_CYCLING);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG1) & CAL1_VLD, ==, 0);
    step(qts, 2 * MS + 10 * US);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG1) & CAL1_VLD, ==, CAL1_VLD);
    g_assert_cmpuint(CAL1_VALUE(rd(qts, 0, RTCCALICFG1)), >=, 39999);
    g_assert_cmpuint(CAL1_VALUE(rd(qts, 0, RTCCALICFG1)), <=, 40001);

    /* Timer group 1 has no calibration clock: it only times out. */
    wr(qts, 1, RTCCALICFG2, CAL2_RST_CNT(3) | CAL2_THRES(100));
    wr(qts, 1, RTCCALICFG, CAL_MAX(1));
    wr(qts, 1, RTCCALICFG, CAL_MAX(1) | CAL_START);
    step(qts, 101 * XTAL_NS);
    g_assert_cmphex(rd(qts, 1, RTCCALICFG2) & CAL2_TIMEOUT, ==, 1);
    g_assert_cmphex(rd(qts, 1, RTCCALICFG) & CAL_RDY, ==, 0);
    qtest_quit(qts);
}

/*
 * The value counts XTAL_CLK's edges at their own times, at whatever phase
 * they have to the measured clock's. RC_SLOW_CLK, 20000/3 ns a cycle,
 * runs from power-on, its edges at multiples of its period, and
 * TIMERGRP0_TGRT_CLK, switched to it at 1000 ns, divides it by 1 from its
 * first edge on: the measurement's first edge comes at 6666 2/3 ns and
 * its second at 13333 1/3 ns, and the XTAL_CLK edges in between, at 6675
 * to 13325 ns, are 267, though the cycle lasts 266 2/3 XTAL_CLK cycles.
 * The measurement ends at the first ns after.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_calibration_phase(void)
{
    QTestState *qts = start();

    step(qts, 1000);
    cal_clock(qts, 8, 1);
    cal_start(qts, 1);
    step(qts, 13333 - now);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, 0, RTCCALICFG) & CAL_RDY, ==, CAL_RDY);
    g_assert_cmpuint(CAL1_VALUE(rd(qts, 0, RTCCALICFG1)), ==, 267);
    qtest_quit(qts);
}

/*
 * The block's reset line and bus clock; and byte accesses, which read and
 * write their bytes of a register.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_gating(void)
{
    QTestState *qts = start();

    wr(qts, 0, TALARMLO(0), 0x12345678);
    qtest_writel(qts, HP_RST_EN1, RST_EN_TIMERGRP0);
    g_assert_cmphex(rd(qts, 0, TALARMLO(0)), ==, 0);
    qtest_writel(qts, HP_RST_EN1, 0);
    wr(qts, 0, TALARMLO(0), 0x12345678);
    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) & ~TIMERGRP0_APB_CLK_EN);
    wr(qts, 0, TALARMLO(0), 0);
    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) | TIMERGRP0_APB_CLK_EN);
    g_assert_cmphex(rd(qts, 0, TALARMLO(0)), ==, 0x12345678);

    qtest_writeb(qts, TIMG(0) + TALARMLO(0) + 1, 0xab);
    g_assert_cmphex(rd(qts, 0, TALARMLO(0)), ==, 0x1234ab78);
    g_assert_cmphex(qtest_readw(qts, TIMG(0) + TALARMLO(0) + 2), ==, 0x1234);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/timg/reset", test_reset);
    qtest_add_func("/esp32p4/timg/counting", test_counting);
    qtest_add_func("/esp32p4/timg/alarm", test_alarm);
    qtest_add_func("/esp32p4/timg/etm-tasks", test_etm_tasks);
    qtest_add_func("/esp32p4/timg/wdt-stages", test_wdt_stages);
    qtest_add_func("/esp32p4/timg/wdt-cpu-reset", test_wdt_cpu_reset);
    qtest_add_func("/esp32p4/timg/wdt-core-reset", test_wdt_core_reset);
    qtest_add_func("/esp32p4/timg/wdt-flashboot", test_wdt_flashboot);
    qtest_add_func("/esp32p4/timg/calibration", test_calibration);
    qtest_add_func("/esp32p4/timg/calibration-phase",
                   test_calibration_phase);
    qtest_add_func("/esp32p4/timg/gating", test_gating);
    return g_test_run();
}
