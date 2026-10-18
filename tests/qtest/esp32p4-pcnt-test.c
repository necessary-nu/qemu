/*
 * QTest testcase for the ESP32-P4 pulse count controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Pulses come from outside the chip on pads routed through the GPIO
 * matrix to a unit's inputs, at exact virtual times, or from the RMT. The
 * filter counts PCNT_APB_CLK, APB_CLK from reset: 10 MHz, 100 ns a cycle.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define PCNT                0x500c9000u
#define CONF0(u)            (PCNT + 0x00 + 0x0c * (u))
#define CONF1(u)            (PCNT + 0x04 + 0x0c * (u))
#define CONF2(u)            (PCNT + 0x08 + 0x0c * (u))
#define CNT(u)              (PCNT + 0x30 + 4 * (u))
#define INT_RAW             (PCNT + 0x40)
#define INT_ST              (PCNT + 0x44)
#define INT_ENA             (PCNT + 0x48)
#define INT_CLR             (PCNT + 0x4c)
#define STATUS(u)           (PCNT + 0x50 + 4 * (u))
#define CTRL                (PCNT + 0x60)
#define CHANGE_CONF(u)      (PCNT + 0x70 - 4 * (u))
#define DATE                (PCNT + 0xfc)

#define FILTER(t)           (t)
#define FILTER_EN           (1u << 10)
#define THR_ZERO_EN         (1u << 11)
#define THR_H_LIM_EN        (1u << 12)
#define THR_L_LIM_EN        (1u << 13)
#define THR_THRES0_EN       (1u << 14)
#define THR_THRES1_EN       (1u << 15)
/* Channel ch's modes */
#define NEG_MODE(ch, m)     ((m) << (16 + 8 * (ch)))
#define POS_MODE(ch, m)     ((m) << (18 + 8 * (ch)))
#define HCTRL_MODE(ch, m)   ((m) << (20 + 8 * (ch)))
#define LCTRL_MODE(ch, m)   ((m) << (22 + 8 * (ch)))
#define INC                 1
#define DEC                 2
#define KEEP                0
#define INVERT              1
#define HOLD                2

#define ST_ZERO_MODE(v)     ((v) & 3)
#define ST_THRES1           (1u << 2)
#define ST_THRES0           (1u << 3)
#define ST_L_LIM            (1u << 4)
#define ST_H_LIM            (1u << 5)
#define ST_ZERO             (1u << 6)

#define CNT_RST(u)          (1u << (2 * (u)))
#define CNT_PAUSE(u)        (1u << (2 * (u) + 1))
#define DELTA_CHANGE_EN(u)  (1u << (8 + (u)))

#define CLKRST              0x500e6000u
#define ROOT_CLK_CTRL0      (CLKRST + 0x04)
#define ROOT_CLK_CTRL2      (CLKRST + 0x0c)
#define SOC_CLK_DIV_UPDATE  (1u << 4)
#define APB_DIV_SHIFT       16
#define APB_DIV_MASK        (0xffu << APB_DIV_SHIFT)
#define SOC_CLK_CTRL2       (CLKRST + 0x1c)
#define SOC_CLK_CTRL3       (CLKRST + 0x20)
#define PERI_CLK_CTRL22     (CLKRST + 0x9c)
#define HP_RST_EN1          (CLKRST + 0xc4)
#define RMT_SYS_CLK_EN      (1u << 0)
#define PCNT_APB_CLK_EN     (1u << 30)
#define RMT_CLK_EN          (1u << 5)
#define LEDC_APB_CLK_EN     (1u << 0)
#define LEDC_CLK_EN         (1u << 2)
#define RST_EN_PCNT         (1u << 30)

#define GPIO                0x500e0000u
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004u + 4 * (n))
#define MCU_SEL_GPIO        (1u << 12)
#define USJ_CONF0           0x500d2018u
#define USJ_PAD_ENABLE      (1u << 14)
#define FUN_IE              (1u << 9)
#define FUN_DRV2            (2u << 10)
#define GPIO_PATH           "/machine/gpio"

#define SIG_CH0(u)          (141 + (u))
#define SIG_CH1(u)          (145 + (u))
#define CTRL_CH0(u)         (149 + (u))
#define CTRL_CH1(u)         (153 + (u))
#define RST_IN(u)           (63 + (u))

#define INTMTX_STATUS3      0x500d620cu
#define SRC_PCNT_BIT        (1u << 15)

/* The RMT, for pulses from inside the chip */
#define RMT                 0x500a2000u
#define RMT_TX_CONF0(n)     (RMT + 0x20 + 4 * (n))
#define RMT_TX_LIM(n)       (RMT + 0xa0 + 4 * (n))
#define RMT_INT_RAW         (RMT + 0x70)
#define RMT_SYS_CONF        (RMT + 0xc0)
#define RMT_TX_SIM          (RMT + 0xc4)
#define RMT_REF_CNT_RST     (RMT + 0xc8)
#define RMT_RAM(w)          (RMT + 0x800 + 4 * (w))

/* The LEDC, for a wave the PCNT takes in bulk */
#define LEDC                0x500d3000u
#define LEDC_CH_CONF0(n)    (LEDC + 0x00 + 0x14 * (n))
#define LEDC_CH_HPOINT(n)   (LEDC + 0x04 + 0x14 * (n))
#define LEDC_CH_DUTY(n)     (LEDC + 0x08 + 0x14 * (n))
#define LEDC_TIMER_CONF(x)  (LEDC + 0xa0 + 8 * (x))
#define LEDC_SIG_OUT_EN     (1u << 2)
#define LEDC_PARA_UP        (1u << 4)
#define LEDC_TCONF(res, div) ((res) | ((uint32_t)(div) << 5))
#define LEDC_T_RST          (1u << 24)
#define SIG_LEDC(n)         (126 + (n))

#define APB_NS              100

static QTestState *qts;
static int64_t now;

static uint32_t rd(uint32_t a)
{
    return qtest_readl(qts, a);
}

static void wr(uint32_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static void clock_to(int64_t t)
{
    g_assert_cmpint(t, >=, now);
    if (t > now) {
        now = qtest_clock_set(qts, t);
    }
}

static void step(int64_t ns)
{
    clock_to(now + ns);
}

static int16_t count(int u)
{
    return (int16_t)rd(CNT(u));
}

/* Pads 20 to 24 driven from outside and routed to signals. */
static void pad_to(int pad, int sig)
{
    wr(IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE | FUN_DRV2);
    wr(GPIO_FUNC_IN(sig), IN_SIG_IN_SEL | pad);
}

static void drive(int pad, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", pad, level);
}

static void start_args(const char *args)
{
    qts = qtest_initf("-M esp32p4,strap-mode=0 %s", args);
    now = 0;
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | PCNT_APB_CLK_EN);
    for (int pad = 20; pad < 25; pad++) {
        drive(pad, 0);
    }
}

static void start(void)
{
    start_args("");
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

static void pulses(int pad, int n, int64_t hi, int64_t lo)
{
    for (int i = 0; i < n; i++) {
        drive(pad, 1);
        step(hi);
        drive(pad, 0);
        step(lo);
    }
}

/* Configure unit u, latch its limits by clearing it, and let it count. */
static void unit_setup(int u, uint32_t conf0, uint32_t conf1, uint32_t conf2)
{
    wr(CONF0(u), conf0);
    wr(CONF1(u), conf1);
    wr(CONF2(u), conf2);
    wr(CTRL, rd(CTRL) | CNT_RST(u));
    wr(CTRL, rd(CTRL) & ~CNT_RST(u));
}

static uint32_t lims(int16_t h, int16_t l)
{
    return (uint16_t)h | ((uint32_t)(uint16_t)l << 16);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_reset_values(void)
{
    start();
    for (int u = 0; u < 4; u++) {
        g_assert_cmphex(rd(CONF0(u)), ==, 0x3c10);
        g_assert_cmphex(rd(CONF1(u)), ==, 0);
        g_assert_cmphex(rd(CONF2(u)), ==, 0);
        g_assert_cmphex(rd(CNT(u)), ==, 0);
        g_assert_cmphex(rd(STATUS(u)), ==, 0);
        g_assert_cmphex(rd(CHANGE_CONF(u)), ==, 0);
    }
    g_assert_cmphex(rd(CTRL), ==, 0x55);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    g_assert_cmphex(rd(INT_ENA), ==, 0);
    g_assert_cmphex(rd(DATE), ==, 0x02211301);
    wr(CTRL, 0xffffffff);
    g_assert_cmphex(rd(CTRL), ==, 0x10fff);
    /* The step registers are in reverse order: unit 3 first. */
    wr(PCNT + 0x64, 0x1234);
    g_assert_cmphex(rd(CHANGE_CONF(3)), ==, 0x1234);
    stop();
}

/*
 * Edge and level actions: rising edges counted up with the control input
 * low, down with it high (inverted), not at all while held; falling edges
 * of channel 1 counted down.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_counting(void)
{
    start();
    pad_to(20, SIG_CH0(1));
    pad_to(21, CTRL_CH0(1));
    pad_to(22, SIG_CH1(1));
    unit_setup(1, POS_MODE(0, INC) | LCTRL_MODE(0, KEEP) |
               HCTRL_MODE(0, INVERT) | NEG_MODE(1, DEC), 0,
               lims(1000, -1000));
    pulses(20, 7, 500, 500);
    g_assert_cmpint(count(1), ==, 7);
    drive(21, 1);
    step(500);
    pulses(20, 3, 500, 500);
    g_assert_cmpint(count(1), ==, 4);
    wr(CONF0(1), POS_MODE(0, INC) | HCTRL_MODE(0, HOLD) | NEG_MODE(1, DEC));
    pulses(20, 3, 500, 500);
    g_assert_cmpint(count(1), ==, 4);
    pulses(22, 6, 500, 500);
    g_assert_cmpint(count(1), ==, -2);
    g_assert_cmphex(ST_ZERO_MODE(rd(STATUS(1))), ==, 2);

    /* Paused, the counter holds; PULSE_CNT_RST clears it. */
    wr(CTRL, rd(CTRL) | CNT_PAUSE(1));
    pulses(22, 3, 500, 500);
    g_assert_cmpint(count(1), ==, -2);
    wr(CTRL, rd(CTRL) & ~CNT_PAUSE(1));
    pulses(22, 3, 500, 500);
    g_assert_cmpint(count(1), ==, -5);
    wr(CTRL, rd(CTRL) | CNT_RST(1));
    g_assert_cmpint(count(1), ==, 0);
    pulses(22, 3, 500, 500);
    g_assert_cmpint(count(1), ==, 0);
    stop();
}

/*
 * The glitch filter: a change counts once it has held for FILTER_THRES
 * APB cycles, so shorter pulses are lost and edges are counted that much
 * later.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter(void)
{
    int64_t t0;

    start();
    pad_to(20, SIG_CH0(0));
    unit_setup(0, POS_MODE(0, INC) | FILTER_EN | FILTER(5), 0,
               lims(1000, -1000));
    step(1000);
    pulses(20, 4, 400, 1000);
    g_assert_cmpint(count(0), ==, 0);
    t0 = now;
    drive(20, 1);
    clock_to(t0 + 5 * APB_NS - 1);
    g_assert_cmpint(count(0), ==, 0);
    clock_to(t0 + 5 * APB_NS);
    g_assert_cmpint(count(0), ==, 1);
    drive(20, 0);
    step(10 * APB_NS);
    /* A low glitch within a high level is lost as well. */
    drive(20, 1);
    step(1000);
    drive(20, 0);
    step(300);
    drive(20, 1);
    step(1000);
    g_assert_cmpint(count(0), ==, 2);
    drive(20, 0);
    step(1000);

    /* Without FILTER_EN every pulse counts. */
    wr(CONF0(0), POS_MODE(0, INC) | FILTER(5));
    pulses(20, 4, 50, 50);
    g_assert_cmpint(count(0), ==, 6);
    stop();
}

/*
 * The watch points: thresholds and zero on equality, limits clearing the
 * counter, each match latched in STATUS with the unit's interrupt; the
 * limits taking effect only when the counter is cleared or a watch point
 * matches, thresholds at once.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_watch_points(void)
{
    start();
    pad_to(20, SIG_CH0(2));
    pad_to(21, SIG_CH1(2));
    unit_setup(2, POS_MODE(0, INC) | POS_MODE(1, DEC) | THR_ZERO_EN |
               THR_H_LIM_EN | THR_L_LIM_EN | THR_THRES0_EN | THR_THRES1_EN,
               (uint16_t)3 | ((uint32_t)(uint16_t)-2 << 16), lims(5, -4));
    wr(INT_ENA, 1u << 2);
    pulses(20, 2, 500, 500);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    pulses(20, 1, 500, 500);
    g_assert_cmpint(count(2), ==, 3);
    g_assert_cmphex(rd(INT_RAW), ==, 1u << 2);
    g_assert_cmphex(rd(INT_ST), ==, 1u << 2);
    g_assert_cmphex(rd(INTMTX_STATUS3) & SRC_PCNT_BIT, ==, SRC_PCNT_BIT);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_THRES0 | 3);
    wr(INT_CLR, 1u << 2);
    g_assert_cmphex(rd(INTMTX_STATUS3) & SRC_PCNT_BIT, ==, 0);

    /* The high limit at 5 clears the counter. */
    pulses(20, 2, 500, 500);
    g_assert_cmpint(count(2), ==, 0);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_H_LIM | 0);
    wr(INT_CLR, 1u << 2);

    /* Down to -2 (THRES1), through zero, to the low limit. */
    pulses(21, 2, 500, 500);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_THRES1 | 2);
    pulses(21, 1, 500, 500);
    pulses(21, 1, 500, 500);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_L_LIM | 1);
    g_assert_cmpint(count(2), ==, 0);
    pulses(20, 1, 500, 500);
    g_assert_cmpint(count(2), ==, 1);
    pulses(21, 1, 500, 500);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_ZERO | 0);

    /*
     * A new THRES0 matches at once; a new high limit waits for that match,
     * so 2 is passed and 3 clears.
     */
    wr(CONF2(2), lims(2, -4));
    wr(CONF1(2), 2);
    pulses(20, 2, 500, 500);
    g_assert_cmpint(count(2), ==, 2);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_THRES0 | 3);
    pulses(20, 1, 500, 500);
    g_assert_cmpint(count(2), ==, 0);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_H_LIM | 0);
    stop();
}

/* A unit's clear signal holds its counter at 0 while high. */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_clear_signal(void)
{
    start();
    pad_to(20, SIG_CH0(3));
    pad_to(23, RST_IN(3));
    unit_setup(3, POS_MODE(0, INC), 0, lims(1000, -1000));
    pulses(20, 10, 500, 500);
    g_assert_cmpint(count(3), ==, 10);
    drive(23, 1);
    step(100);
    g_assert_cmpint(count(3), ==, 0);
    pulses(20, 2, 500, 500);
    g_assert_cmpint(count(3), ==, 0);
    drive(23, 0);
    pulses(20, 2, 500, 500);
    g_assert_cmpint(count(3), ==, 2);
    stop();
}

/*
 * The step comparator: every 25 counts down from the last step point
 * raises the unit's interrupt, and the step limit clears the counter.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_step(void)
{
    int events = 0;

    start();
    pad_to(20, SIG_CH0(0));
    unit_setup(0, POS_MODE(0, DEC), 0, lims(100, -1000));
    wr(CHANGE_CONF(0), (uint16_t)-25 | ((uint32_t)(uint16_t)-100 << 16));
    wr(CTRL, rd(CTRL) | DELTA_CHANGE_EN(0));
    for (int i = 1; i <= 110; i++) {
        pulses(20, 1, 200, 200);
        if (rd(INT_RAW) & 1) {
            wr(INT_CLR, 1);
            events++;
            g_assert_true(i % 25 == 0 || i == 100);
        }
    }
    /* -25, -50, -75, -100 (the step limit, clearing), then -10 */
    g_assert_cmpint(events, ==, 4);
    g_assert_cmpint(count(0), ==, -10);
    stop();
}

/*
 * The RMT's pulses counted as they are sent: 25 pulses of 2 ticks a level
 * at 40 MHz, each counted at the time of its edge whenever the counter is
 * read.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_rmt_pulses(void)
{
    uint32_t conf = (1 << 8) | (1 << 16) | (1 << 6);
    int64_t t0;

    start();
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | RMT_SYS_CLK_EN);
    wr(PERI_CLK_CTRL22, RMT_CLK_EN);
    wr(RMT_SYS_CONF, rd(RMT_SYS_CONF) | 1);
    /*
     * GPIO24 taken from the USB Serial/JTAG PHY, which has it from reset,
     * as ESP-IDF's gpio_ll_func_sel() does.
     */
    wr(USJ_CONF0, rd(USJ_CONF0) & ~USJ_PAD_ENABLE);
    wr(IO_MUX_GPIO(24), MCU_SEL_GPIO | FUN_IE | FUN_DRV2);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-release", 24, 1);
    wr(GPIO_FUNC_OUT(24), 246);
    wr(GPIO_FUNC_IN(SIG_CH0(0)), IN_SIG_IN_SEL | 24);
    unit_setup(0, POS_MODE(0, INC), 0, lims(1000, -1000));
    for (int w = 0; w < 25; w++) {
        wr(RMT_RAM(w), 0x8002 | (0x0002 << 16));
    }
    wr(RMT_RAM(25), 0);
    wr(RMT_TX_CONF0(0), conf | (1 << 1) | (1 << 2));
    wr(RMT_TX_CONF0(0), conf | (1 << 24));
    step(1000);
    t0 = now;
    wr(RMT_REF_CNT_RST, 1);
    wr(RMT_TX_CONF0(0), conf | 1);
    /* Pulse k rises at t0 + 25 + 100 k ns. */
    clock_to(t0 + 25 + 100 * 9);
    g_assert_cmpint(count(0), ==, 10);
    clock_to(t0 + 25 + 100 * 9 + 99);
    g_assert_cmpint(count(0), ==, 10);
    clock_to(t0 + 25 + 100 * 10);
    g_assert_cmpint(count(0), ==, 11);
    step(100000);
    g_assert_cmpint(count(0), ==, 25);
    g_assert_cmphex(rd(RMT_INT_RAW) & 1, ==, 1);
    stop();
}

/* Held in reset, the block is back at its reset values. */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_block_reset(void)
{
    start();
    wr(CONF0(0), 0x1234);
    wr(CTRL, 0);
    wr(HP_RST_EN1, RST_EN_PCNT);
    wr(HP_RST_EN1, 0);
    g_assert_cmphex(rd(CONF0(0)), ==, 0x3c10);
    g_assert_cmphex(rd(CTRL), ==, 0x55);
    stop();
}

/* APB_CLK = SYS_CLK (20 MHz) / div, as clk_ll_apb_set_divider() does. */
static void apb_div(uint32_t div)
{
    wr(ROOT_CLK_CTRL2, (rd(ROOT_CLK_CTRL2) & ~APB_DIV_MASK) |
                       ((div - 1) << APB_DIV_SHIFT));
    wr(ROOT_CLK_CTRL0, rd(ROOT_CLK_CTRL0) | SOC_CLK_DIV_UPDATE);
}

/*
 * Unit 0's count, `was`, steps as the rising edge waiting in its filter
 * comes through at t, not a nanosecond sooner; then pad 20 goes low for
 * long enough that the fall comes through too.
 */
static void expect_edge_at(int64_t t, int16_t was)
{
    clock_to(t - 1);
    g_assert_cmpint(count(0), ==, was);
    clock_to(t);
    g_assert_cmpint(count(0), ==, was + 1);
    drive(20, 0);
    step(10000);
}

/*
 * The filter counts APB_CLK's edges, the old clock's up to a change and
 * the new clock's after it: a change held back across a change of the
 * clock's rate, or across the clock stopped and started again by its
 * gate, keeps the edges it held for before, the count not restarting.
 * SYS_CLK's edges are every 50 ns from time 0, APB_CLK's every 100 ns
 * from reset; a new divider takes effect at the edge of SYS_CLK that
 * would have ended its period.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter_span(void)
{
    int64_t t;

    start();
    pad_to(20, SIG_CH0(0));
    unit_setup(0, POS_MODE(0, INC) | FILTER_EN | FILTER(10), 0,
               lims(1000, -1000));

    /*
     * 3 edges at 10 MHz, then 7 at 20 MHz from t + 400: t + 700, not
     * t + 550 (10 of 20 MHz's).
     */
    clock_to(10000);
    t = now;
    clock_to(t + 50);
    drive(20, 1);
    clock_to(t + 350);
    apb_div(1);
    expect_edge_at(t + 700, 0);

    /*
     * 3 edges at 20 MHz, then 7 at 10 MHz from t + 250: t + 850, not
     * t + 950.
     */
    t = now;
    clock_to(t + 25);
    drive(20, 1);
    clock_to(t + 175);
    apb_div(2);
    expect_edge_at(t + 850, 1);

    /*
     * 3 edges at 10 MHz, the clock gated from t + 350 to t + 2350, then 7
     * from t + 2400: t + 3000, not t + 3300.
     */
    t = now;
    clock_to(t + 50);
    drive(20, 1);
    clock_to(t + 350);
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) & ~PCNT_APB_CLK_EN);
    clock_to(t + 2350);
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | PCNT_APB_CLK_EN);
    expect_edge_at(t + 3000, 2);

    /*
     * A change made while the clock is gated counts from its restart, at
     * t + 1050: 10 edges from t + 1100.
     */
    t = now;
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) & ~PCNT_APB_CLK_EN);
    clock_to(t + 50);
    drive(20, 1);
    clock_to(t + 1050);
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | PCNT_APB_CLK_EN);
    expect_edge_at(t + 2000, 3);
    stop();
}

/*
 * The filter sees the pad only at APB_CLK's edges, each seeing the level
 * the pad had up to it, and passes a change at the FILTER_THRES-th edge in
 * a row that sees it. Unit 0 counts rising edges of pad 20 through a
 * filter of `thres`; APB_CLK's edges are at every multiple of 100 ns.
 */
static void start_sampled(uint32_t thres)
{
    start();
    pad_to(20, SIG_CH0(0));
    unit_setup(0, POS_MODE(0, INC) | FILTER_EN | FILTER(thres), 0,
               lims(1000, -1000));
}

/*
 * A dip back to the filtered level that no edge sees leaves the count
 * running on; one that an edge sees restarts it. FILTER_THRES 10.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter_dip(void)
{
    int64_t t;

    start_sampled(10);

    /*
     * A dip from t + 410 to t + 450, between the edges at t + 400 and
     * t + 500, is not seen: the rise at t is seen at the 10 edges to
     * t + 1000 and passes there. Counted afresh from t + 450 it would be
     * lost.
     */
    t = 10000;
    clock_to(t);
    drive(20, 1);
    clock_to(t + 410);
    drive(20, 0);
    clock_to(t + 450);
    drive(20, 1);
    expect_edge_at(t + 1000, 0);

    /* A dip the edge at t + 500 sees restarts the count: 10 from t + 600 */
    t = 30000;
    clock_to(t);
    drive(20, 1);
    clock_to(t + 410);
    drive(20, 0);
    clock_to(t + 510);
    drive(20, 1);
    expect_edge_at(t + 1500, 1);
    stop();
}

/*
 * A change counts from the first edge after it, so a pulse is judged by
 * the edges it spans rather than its length. FILTER_THRES 10.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter_start(void)
{
    int64_t t;

    start_sampled(10);

    /*
     * A rise just after an edge counts from the next: 999 ns, less than
     * 10 cycles, are seen at the 10 edges from t + 100 to t + 1000 and
     * pass.
     */
    t = 10000;
    clock_to(t + 1);
    drive(20, 1);
    expect_edge_at(t + 1000, 0);

    /* The same 999 ns from an edge are seen at 9 and lost */
    t = 30000;
    clock_to(t);
    drive(20, 1);
    clock_to(t + 999);
    drive(20, 0);
    step(10000);
    g_assert_cmpint(count(0), ==, 1);
    stop();
}

/*
 * The clock gated from t + 350 to t + 2350, after 3 edges have seen a
 * rise at t: a dip while it is stopped is seen at no edge, and the rise
 * passes at the 7th edge after the restart, t + 3000. FILTER_THRES 10.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter_gated_dip(void)
{
    int64_t t = 10000;

    start_sampled(10);
    clock_to(t);
    drive(20, 1);
    clock_to(t + 350);
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) & ~PCNT_APB_CLK_EN);
    clock_to(t + 1000);
    drive(20, 0);
    clock_to(t + 1500);
    drive(20, 1);
    clock_to(t + 2350);
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | PCNT_APB_CLK_EN);
    expect_edge_at(t + 3000, 0);
    stop();
}

/*
 * FILTER_THRES 0 passes a change at the first edge that sees it: a pulse
 * from t + 5 to t + 60 is seen at none, one from t + 60 to t + 120 at
 * t + 100.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter_thres_zero(void)
{
    int64_t t = 10000;

    start_sampled(0);
    clock_to(t + 5);
    drive(20, 1);
    clock_to(t + 60);
    drive(20, 0);
    g_assert_cmpint(count(0), ==, 0);
    drive(20, 1);
    clock_to(t + 99);
    g_assert_cmpint(count(0), ==, 0);
    clock_to(t + 100);
    g_assert_cmpint(count(0), ==, 1);
    clock_to(t + 120);
    drive(20, 0);
    stop();
}

/*
 * LEDC timer x from XTAL_CLK, a count every 25 ns, 3 bits: started at t,
 * its 200 ns cycles count from t; a channel set up then takes its settings
 * in at the first overflow, at t + 200, which is returned.
 */
static int64_t ledc_timer_start(int x, int64_t t)
{
    clock_to(t);
    wr(LEDC_TIMER_CONF(x), LEDC_TCONF(3, 256) | LEDC_T_RST);
    wr(LEDC_TIMER_CONF(x), LEDC_TCONF(3, 256));
    return t + 200;
}

/*
 * LEDC channel n on timer x, high for the first 7 counts of each cycle
 * and low for the last, on pad `pad`.
 */
static void ledc_dip_channel(int n, int x, int pad)
{
    wr(LEDC_CH_HPOINT(n), 0);
    wr(LEDC_CH_DUTY(n), 7u << 4);
    wr(LEDC_CH_CONF0(n), x | LEDC_SIG_OUT_EN | LEDC_PARA_UP);
    wr(IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE | FUN_DRV2);
    wr(GPIO_FUNC_OUT(pad), SIG_LEDC(n));
}

/* Unit u's channel 0 signal from pad `pad`. */
static void route(int pad, int u)
{
    wr(GPIO_FUNC_IN(SIG_CH0(u)), IN_SIG_IN_SEL | pad);
}

static void ledc_on(void)
{
    wr(SOC_CLK_CTRL3, rd(SOC_CLK_CTRL3) | LEDC_APB_CLK_EN);
    /* LEDC_CLK from XTAL_CLK */
    wr(PERI_CLK_CTRL22, (rd(PERI_CLK_CTRL22) & ~3u) | LEDC_CLK_EN);
}

/*
 * At t, units 0 and 1 have passed the rise at c at the third edge of
 * APB_CLK after it, c + 275, and unit 2 has counted every rise.
 */
static void wave_dips_at(int64_t c, int64_t t)
{
    clock_to(t);
    g_assert_cmpint(count(0), ==, t >= c + 275);
    g_assert_cmpint(count(1), ==, t >= c + 275);
    g_assert_cmpint(count(2), ==, t < c ? 0 : 1 + (t - c) / 200);
}

/*
 * A wave whose lows are shorter than APB_CLK's period (100 ns, its edges
 * at every multiple of 100 ns), from the LEDC, taken by a filter of 3 from
 * the output in bulk (unit 0, pad 26), which takes such a wave's edges one
 * by one, and from a pad the board wires the same wave to (unit 1, pad 29
 * from pad 28), which the output drives edge by edge; unit 2 counts every
 * rise of pad 26. Channels 0 and 1, on timer 0, start their cycles at c,
 * 25 ns after an edge, so that no edge sees their lows, from
 * c + 175 + 200 k to c + 200 (k + 1): units 0 and 1 see their pads high
 * from the first rise on, and pass it at the third edge after c, alike.
 * Channel 2, on timer 1, starts its cycles at an edge, the edge at the end
 * of each low sees it, and each high, seen at one edge, is lost (unit 3,
 * pad 30).
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_filter_wave_dips(void)
{
    int64_t c, c1, t;

    start_args("-global esp32p4-gpio.pad-wires=28:29");
    ledc_on();
    for (int u = 0; u < 4; u++) {
        uint32_t filter = u == 2 ? 0 : FILTER_EN | FILTER(3);

        unit_setup(u, POS_MODE(0, INC) | filter, 0, lims(1000, -1000));
    }
    wr(IO_MUX_GPIO(29), MCU_SEL_GPIO | FUN_IE | FUN_DRV2);
    c = ledc_timer_start(0, 10025);
    ledc_dip_channel(0, 0, 26);
    ledc_dip_channel(1, 0, 28);
    route(26, 0);
    route(29, 1);
    route(26, 2);
    c1 = ledc_timer_start(1, 10100);
    ledc_dip_channel(2, 1, 30);
    route(30, 3);

    /* Every 25 ns through the first cycles, and a ns before the pass */
    for (t = c - 25; t < c + 1000; t += 25) {
        wave_dips_at(c, t);
        if (t == c + 250) {
            wave_dips_at(c, c + 274);
        }
    }

    /* 500 cycles on: one rise seen, 501 made; channel 2's all lost */
    clock_to(c + 100000);
    g_assert_cmpint(count(0), ==, 1);
    g_assert_cmpint(count(1), ==, 1);
    g_assert_cmpint(count(2), ==, 501);
    clock_to(c1 + 100000);
    g_assert_cmpint(count(3), ==, 0);
    stop();
}

/*
 * A wave taken in bulk whose level changes as it is taken up, the LEDC
 * channel going high as its first cycle starts, at c: unit 2 counts that
 * rise there, unfiltered, and its step to 1 meets THRES0, latched and
 * raising the unit's interrupt at that moment.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_wave_join_watch(void)
{
    int64_t c;

    start();
    ledc_on();
    unit_setup(2, POS_MODE(0, INC) | THR_THRES0_EN, 1, lims(1000, -1000));
    c = ledc_timer_start(0, 10025);
    ledc_dip_channel(0, 0, 26);
    route(26, 2);
    clock_to(c - 1);
    g_assert_cmpint(count(2), ==, 0);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    clock_to(c);
    g_assert_cmpint(count(2), ==, 1);
    g_assert_cmphex(rd(INT_RAW), ==, 1u << 2);
    g_assert_cmphex(rd(STATUS(2)), ==, ST_THRES0 | 3);
    clock_to(c + 1000);
    g_assert_cmpint(count(2), ==, 6);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/pcnt/reset_values", test_reset_values);
    qtest_add_func("esp32p4/pcnt/counting", test_counting);
    qtest_add_func("esp32p4/pcnt/filter", test_filter);
    qtest_add_func("esp32p4/pcnt/watch_points", test_watch_points);
    qtest_add_func("esp32p4/pcnt/clear_signal", test_clear_signal);
    qtest_add_func("esp32p4/pcnt/step", test_step);
    qtest_add_func("esp32p4/pcnt/rmt_pulses", test_rmt_pulses);
    qtest_add_func("esp32p4/pcnt/block_reset", test_block_reset);
    qtest_add_func("esp32p4/pcnt/filter_span", test_filter_span);
    qtest_add_func("esp32p4/pcnt/filter_dip", test_filter_dip);
    qtest_add_func("esp32p4/pcnt/filter_start", test_filter_start);
    qtest_add_func("esp32p4/pcnt/filter_gated_dip", test_filter_gated_dip);
    qtest_add_func("esp32p4/pcnt/filter_thres_zero", test_filter_thres_zero);
    qtest_add_func("esp32p4/pcnt/filter_wave_dips", test_filter_wave_dips);
    qtest_add_func("esp32p4/pcnt/wave_join_watch", test_wave_join_watch);
    return g_test_run();
}
