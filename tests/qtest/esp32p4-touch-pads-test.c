/*
 * QTest testcase for the ESP32-P4 touch sensor's pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A touch channel's pad left floating, as ESP-IDF's gpio_config_as_analog()
 * leaves it, measures its capacitance; a pad a digital driver holds gives
 * no pulse, and the measurement ends only by its timeout; pulls on the pad
 * skew the count, or stop the pulses where they outpull the sensor's
 * current; an input buffer left on sees the sawtooth, reading low while
 * the pad is measured. Each acts from the moment it changes.
 *
 * The model's sensor charges and discharges the pad with I = 8 uA
 * (DRV_HS + 1) + 1 uA DRV_LS between 1.4 V and 1.9 V of VDD_LP's 3.3 V
 * (hw/misc/esp32p4_touch.c): a cycle of C x 1 V / I with no pull.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qobject/qdict.h"

#define TOUCH               0x50128000u
#define INT_RAW             (TOUCH + 0x000)
#define INT_ENA             (TOUCH + 0x008)
#define INT_CLR             (TOUCH + 0x00c)
#define CHN_STATUS          (TOUCH + 0x010)
#define STATUS(n)           (TOUCH + 0x014 + 4 * (n))
#define INT_SCAN_DONE       (1u << 0)
#define INT_DONE            (1u << 1)
#define INT_TIMEOUT         (1u << 4)
#define MEAS_DONE           (1u << 15)
#define DATA(v)             ((v) & 0xffffu)

#define LP_ANA              0x50113000u
#define SCAN_CTRL2          (LP_ANA + 0x104)
#define WORK                (LP_ANA + 0x108)
#define WORK_MEAS_NUM       (LP_ANA + 0x10c)
#define FREQ_PARA(f)        (LP_ANA + 0x12c + 4 * (f))
#define ANA_PARA            (LP_ANA + 0x138)
#define MUX0                (LP_ANA + 0x13c)
#define MUX1                (LP_ANA + 0x140)

#define TIMEOUT(n)          ((uint32_t)(n) << 6 | 1u << 22)
#define OUT_SEL             (1u << 25)
#define OUT_GATE            (1u << 27)
#define MEAS_NUM(f, n)      ((uint32_t)(n) << (20 - 10 * (f)))
#define DRV(hs, ls)         ((uint32_t)(hs) << 13 | (uint32_t)(ls) << 9)
#define EN_CAL              (1u << 3)
#define DCAP_CAL(c)         ((uint32_t)(c) << 4)
#define DATA_SEL(s)         ((uint32_t)(s) << 8)
#define DONE_FORCE          (1u << 28)
#define FSM_EN              (1u << 29)
#define START_EN            (1u << 30)
#define START_FORCE         (1u << 31)
#define TOUCH_START(m)      (m)
#define TOUCH_XPD(m)        ((uint32_t)(m) << 15)

#define GPIO                0x500e0000u
#define GPIO_OUT_W1TS       (GPIO + 0x008)
#define GPIO_OUT_W1TC       (GPIO + 0x00c)
#define GPIO_ENABLE_W1TS    (GPIO + 0x024)
#define GPIO_ENABLE_W1TC    (GPIO + 0x028)
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define SIG_GPIO_OUT        0x100
#define OEN_SEL             (1u << 10)
#define IO_MUX_GPIO(n)      (0x500e1004u + 4 * (n))
#define FUN_WPD             (1u << 7)
#define FUN_WPU             (1u << 8)
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((uint32_t)(d) << 10)
#define MCU_SEL_GPIO        (1u << 12)
#define LP_IOMUX_PAD(n)     (0x5012b008u + 4 * (n))
#define LP_PAD_RUE          (1u << 3)
#define LP_PAD_MUX_SEL      (1u << 4)
#define LP_PAD_FUN_SEL(f)   ((uint32_t)(f) << 5)

#define CLKRST_PATH         "/machine/clkrst"
#define GPIO_PATH           "/machine/gpio"

/* The model's sensor: its swing, and the pads' own pulls. */
#define VDD_V               3.3
#define VL_V                1.4
#define VH_V                1.9
#define WEAK_PULL_OHM       45000.0

static QTestState *qts;
static uint64_t fast_hz;

static uint32_t rd(uint32_t a)
{
    return qtest_readl(qts, a);
}

static void wr(uint32_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static void step(int64_t ns)
{
    qtest_clock_step(qts, ns);
}

static uint64_t clk_hz(const char *name)
{
    g_autofree char *path = g_strdup_printf(CLKRST_PATH "/%s", name);
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         path);
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    return period ? (1000000000ull << 32) / period : 0;
}

/*
 * One cycle's length, in ns, of a @ff pad at @ua with pulls of @up and
 * @down ohms (0: none).
 */
static double cycle_ns(double ff, double ua, double up, double down)
{
    double gu = up ? 1e6 / up : 0, gd = down ? 1e6 / down : 0;
    double g = gu + gd, vp, charge, discharge;

    if (!g) {
        return ff / ua;
    }
    vp = VDD_V * gu / g;
    charge = ua + g * (vp - VH_V);
    discharge = ua - g * (vp - VL_V);
    g_assert(charge > 0 && discharge > 0);
    return ff / g * (log1p(g * (VH_V - VL_V) / charge) +
                     log1p(g * (VH_V - VL_V) / discharge));
}

/* The count of 500 cycles of such a pad. */
static uint32_t expect(double ff, double ua, double up, double down)
{
    int64_t ns = llround(500 * cycle_ns(ff, ua, up, down));

    return (uint64_t)ns * fast_hz / 1000000000ull;
}

static void start(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    fast_hz = clk_hz("lp-dyn-fast");
    g_assert_cmpuint(fast_hz, >, 0);
    wr(WORK, OUT_GATE | OUT_SEL);
    wr(WORK_MEAS_NUM, MEAS_NUM(0, 500));
    wr(FREQ_PARA(0), DRV(1, 1));
    wr(INT_ENA, 0x7f);
}

/*
 * ESP-IDF's touch_channel_pin_init(): gpio_config_as_analog() leaves the
 * pad floating: input and output off, the output from GPIO_ENABLE, no
 * pulls, the GPIO function, and the pad back from the LP IO MUX.
 */
static void config_as_analog(int pad)
{
    wr(GPIO_ENABLE_W1TC, 1u << pad);
    wr(GPIO_FUNC_OUT(pad), SIG_GPIO_OUT | OEN_SEL);
    wr(IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_DRV(2));
    wr(LP_IOMUX_PAD(pad), rd(LP_IOMUX_PAD(pad)) & ~LP_PAD_MUX_SEL);
}

/* Start a software measurement of channel @ch. */
static void begin(unsigned ch)
{
    wr(MUX0, (rd(MUX0) & ~FSM_EN) | DONE_FORCE | START_FORCE);
    wr(MUX1, TOUCH_START(1u << ch) | TOUCH_XPD(1u << ch));
    wr(MUX0, rd(MUX0) | START_EN);
    wr(MUX0, rd(MUX0) & ~START_EN);
}

/* Measure channel @ch, returning its count, or 0 if it never ends. */
static uint32_t measure(unsigned ch)
{
    begin(ch);
    for (int i = 0; i < 20000 && !(rd(CHN_STATUS) & MEAS_DONE); i++) {
        step(1000);
    }
    if (!(rd(CHN_STATUS) & MEAS_DONE)) {
        return 0;
    }
    wr(MUX0, rd(MUX0) & ~DATA_SEL(3));
    return DATA(rd(STATUS(ch)));
}

static void assert_count(uint32_t got, uint32_t want)
{
    g_assert_cmpint(abs((int)got - (int)want), <=, 1);
}

/* [spec:nuos:req:emu.esp32p4.analog/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_floating(void)
{
    start();
    config_as_analog(6);
    assert_count(measure(5), expect(10000, 17, 0, 0));
    qtest_quit(qts);
}

/*
 * A pad driven high or low by its GPIO output, or from outside, gives no
 * pulse: the measurement never ends; with the timeout it raises
 * TOUCH_TIMEOUT_INT and gives no data. The internal capacitor
 * (TOUCH_EN_CAL) is measured whatever drives the pad.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_driven(void)
{
    start();
    config_as_analog(6);
    wr(GPIO_ENABLE_W1TS, 1u << 6);
    g_assert_cmpuint(measure(5), ==, 0);
    g_assert_false(rd(INT_RAW) & INT_DONE);
    qtest_quit(qts);

    start();
    config_as_analog(6);
    wr(GPIO_OUT_W1TS, 1u << 6);
    wr(GPIO_ENABLE_W1TS, 1u << 6);
    wr(SCAN_CTRL2, (rd(SCAN_CTRL2) & ~(0xffffu << 6)) | TIMEOUT(1000));
    begin(5);
    step(4 * 1000000000ll / fast_hz + 1000 * 1000000000ll / fast_hz + 100);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TIMEOUT);
    g_assert_false(rd(CHN_STATUS) & MEAS_DONE);
    g_assert_cmpuint(DATA(rd(STATUS(5))), ==, 0);
    qtest_quit(qts);

    start();
    config_as_analog(6);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 6, 1);
    g_assert_cmpuint(measure(5), ==, 0);
    qtest_quit(qts);

    start();
    config_as_analog(6);
    wr(GPIO_ENABLE_W1TS, 1u << 6);
    /* 100 steps of 20 fF: 2 pF, off the pad. */
    wr(ANA_PARA, EN_CAL | DCAP_CAL(100));
    assert_count(measure(5), expect(2000, 17, 0, 0));
    qtest_quit(qts);
}

/*
 * The pad's own 45 kOhm pull-up, at 17 uA, holds the pad above the lower
 * threshold: no pulse comes. At 72 uA the pulses come, each the longer for
 * the pull, as for the pull-down, and for the LP IO MUX's pull-up on a pad
 * it has. A board's pull-up joins the pad's own.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_pulls(void)
{
    uint32_t free_count;

    start();
    config_as_analog(6);
    wr(IO_MUX_GPIO(6), MCU_SEL_GPIO | FUN_DRV(2) | FUN_WPU);
    g_assert_cmpuint(measure(5), ==, 0);
    qtest_quit(qts);

    start();
    config_as_analog(6);
    wr(FREQ_PARA(0), DRV(8, 0));
    free_count = measure(5);
    assert_count(free_count, expect(10000, 72, 0, 0));
    wr(IO_MUX_GPIO(6), MCU_SEL_GPIO | FUN_DRV(2) | FUN_WPU);
    assert_count(measure(5), expect(10000, 72, WEAK_PULL_OHM, 0));
    g_assert_cmpuint(measure(5), >, free_count);
    wr(IO_MUX_GPIO(6), MCU_SEL_GPIO | FUN_DRV(2) | FUN_WPD);
    assert_count(measure(5), expect(10000, 72, 0, WEAK_PULL_OHM));
    /* Both: the divider at half the supply, between the thresholds. */
    wr(IO_MUX_GPIO(6), MCU_SEL_GPIO | FUN_DRV(2) | FUN_WPU | FUN_WPD);
    assert_count(measure(5), expect(10000, 72, WEAK_PULL_OHM,
                                    WEAK_PULL_OHM));
    /* The LP IO MUX's pull-up, the pad on its LP GPIO function. */
    wr(IO_MUX_GPIO(6), MCU_SEL_GPIO | FUN_DRV(2));
    wr(LP_IOMUX_PAD(6), LP_PAD_MUX_SEL | LP_PAD_FUN_SEL(1) | LP_PAD_RUE);
    assert_count(measure(5), expect(10000, 72, WEAK_PULL_OHM, 0));
    config_as_analog(6);
    assert_count(measure(5), expect(10000, 72, 0, 0));
    /* A 100 kOhm pull-up on the board. */
    qtest_set_irq_in(qts, GPIO_PATH, "board-pull-up", 6, 100000);
    assert_count(measure(5), expect(10000, 72, 100000, 0));
    qtest_quit(qts);
}

/*
 * GPIO4 (T3), MTMS, has its input enabled from reset: the count is the
 * same, and the input reads the sawtooth, low, while the pad is measured,
 * where its pull-up has it high otherwise.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_input_buffer(void)
{
    start();
    wr(FREQ_PARA(0), DRV(8, 0));
    wr(IO_MUX_GPIO(4), MCU_SEL_GPIO | FUN_DRV(2) | FUN_IE | FUN_WPU);
    g_assert_true(rd(GPIO_IN) & (1u << 4));
    begin(3);
    step(10000);
    g_assert_false(rd(CHN_STATUS) & MEAS_DONE);
    g_assert_false(rd(GPIO_IN) & (1u << 4));
    while (!(rd(CHN_STATUS) & MEAS_DONE)) {
        step(1000);
    }
    g_assert_true(rd(GPIO_IN) & (1u << 4));
    wr(MUX0, rd(MUX0) & ~DATA_SEL(3));
    assert_count(DATA(rd(STATUS(3))), expect(10000, 72, WEAK_PULL_OHM, 0));
    qtest_quit(qts);
}

/*
 * The pad driven for 20 us in the middle of a measurement: no pulse comes
 * while it is, and the measurement ends 20 us later, its count 20 us of
 * LP_DYN_FAST_CLK more.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_driven_for_a_while(void)
{
    int64_t hold = 20000;
    uint32_t want;

    start();
    config_as_analog(6);
    want = expect(10000, 17, 0, 0) + hold * fast_hz / 1000000000ull;
    begin(5);
    step(100000);
    wr(GPIO_ENABLE_W1TS, 1u << 6);
    step(hold);
    wr(GPIO_ENABLE_W1TC, 1u << 6);
    while (!(rd(CHN_STATUS) & MEAS_DONE)) {
        step(1000);
    }
    wr(MUX0, rd(MUX0) & ~DATA_SEL(3));
    g_assert_cmpint(abs((int)DATA(rd(STATUS(5))) - (int)want), <=, 2);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/touch-pads/floating", test_floating);
    qtest_add_func("esp32p4/touch-pads/driven", test_driven);
    qtest_add_func("esp32p4/touch-pads/pulls", test_pulls);
    qtest_add_func("esp32p4/touch-pads/input_buffer", test_input_buffer);
    qtest_add_func("esp32p4/touch-pads/driven_for_a_while",
                   test_driven_for_a_while);
    return g_test_run();
}
