/*
 * QTest testcase for the ESP32-P4 touch sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The pads' capacitances are set with qom-set; the sensor is configured
 * through LP_ANALOG_PERI and the PMU as ESP-IDF's touch_ll does. A
 * measurement counts LP_DYN_FAST_CLK for MEAS_NUM pulses of the pad, each
 * C / I long, I being 8 uA (DRV_HS + 1) + 1 uA DRV_LS.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qobject/qdict.h"

#define TOUCH               0x50128000u
#define INT_RAW             (TOUCH + 0x000)
#define INT_ST              (TOUCH + 0x004)
#define INT_ENA             (TOUCH + 0x008)
#define INT_CLR             (TOUCH + 0x00c)
#define CHN_STATUS          (TOUCH + 0x010)
#define STATUS(n)           (TOUCH + 0x014 + 4 * (n))
#define SLP_STATUS          (TOUCH + 0x050)
#define DATE                (TOUCH + 0x100)
#define INT_SCAN_DONE       (1u << 0)
#define INT_DONE            (1u << 1)
#define INT_ACTIVE          (1u << 2)
#define INT_INACTIVE        (1u << 3)
#define INT_TIMEOUT         (1u << 4)
#define INT_BENCH_UPDATE    (1u << 6)
#define PAD_ACTIVE(v)       ((v) & 0x7fffu)
#define MEAS_DONE           (1u << 15)
#define SCAN_CURR(v)        (((v) >> 16) & 0xfu)
#define DATA(v)             ((v) & 0xffffu)

#define LP_ANA              0x50113000u
#define SCAN_CTRL1          (LP_ANA + 0x100)
#define SCAN_CTRL2          (LP_ANA + 0x104)
#define WORK                (LP_ANA + 0x108)
#define WORK_MEAS_NUM       (LP_ANA + 0x10c)
#define FILTER1             (LP_ANA + 0x110)
#define FILTER2             (LP_ANA + 0x114)
#define FILTER3             (LP_ANA + 0x118)
#define TOUCH_CLR           (LP_ANA + 0x124)
#define FREQ_PARA(f)        (LP_ANA + 0x12c + 4 * (f))
#define ANA_PARA            (LP_ANA + 0x138)
#define MUX0                (LP_ANA + 0x13c)
#define MUX1                (LP_ANA + 0x140)
#define PAD_TH(n, f)        (LP_ANA + 0x144 + 12 * (n) + 4 * (f))
#define TOUCH_CTRL          (LP_ANA + 0x1fc)

#define SCAN_MAP(m)         ((uint32_t)(m) << 2)
#define XPD_WAIT(n)         ((uint32_t)(n) << 17)
#define TIMEOUT(n)          ((uint32_t)(n) << 6 | 1u << 22)
#define FREQ_SCAN(n)        (1u << 27 | (uint32_t)(n) << 28)
#define DIV_NUM(f, d)       ((uint32_t)(d) << (22 - 3 * (f)))
#define OUT_SEL             (1u << 25)
#define OUT_GATE            (1u << 27)
#define MEAS_NUM(f, n)      ((uint32_t)(n) << (20 - 10 * (f)))
#define FILTER_EN           (1u << 16)
#define DEBOUNCE(n)         ((uint32_t)(n) << 29)
#define BENCH_SW(v)         (v)
#define UPDATE_BENCH_SW     (1u << 16)
#define STATUS_CLR          (1u << 15)
#define DRV(hs, ls)         ((uint32_t)(hs) << 13 | (uint32_t)(ls) << 9)
#define EN_CAL              (1u << 3)
#define DCAP_CAL(c)         ((uint32_t)(c) << 4)
#define DATA_SEL(s)         ((uint32_t)(s) << 8)
#define FREQ_SEL(f)         ((uint32_t)(f) << 10)
#define DONE_FORCE          (1u << 28)
#define FSM_EN              (1u << 29)
#define START_EN            (1u << 30)
#define START_FORCE         (1u << 31)
#define TOUCH_START(m)      (m)
#define TOUCH_XPD(m)        ((uint32_t)(m) << 15)
#define TH(v)               ((uint32_t)(v) << 16)
#define BM_PAD(n)           ((uint32_t)(n) << 2)

#define PMU_TOUCH_PWR_CNTL  0x50115210u
#define PWR_WAIT(n)         ((uint32_t)(n) << 5)
#define PWR_SLEEP(n)        ((uint32_t)(n) << 14)
#define PWR_TIMER_EN        (1u << 31)

#define LPPERI_RESET_EN     0x50120008u
#define RST_EN_LP_TOUCH     (1u << 22)
#define CLKRST_PATH         "/machine/clkrst"
#define TOUCH_PATH          "/machine/touch"

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

static void set_ff(unsigned ch, uint32_t ff)
{
    g_autofree char *prop = g_strdup_printf("ch%u-ff", ch);
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': %s, 'property': %s, 'value': %u } }",
                         TOUCH_PATH, prop, ff);

    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
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

/* The count of @n pulses divided by @div of a @ff pad at DRV(1, 1). */
static uint32_t expect(uint32_t n, uint32_t div, uint32_t ff)
{
    int64_t ns = llround((double)n * div * ff / 17.0);

    return (uint64_t)ns * fast_hz / 1000000000ull;
}

static void start(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    fast_hz = clk_hz("lp-dyn-fast");
    g_assert_cmpuint(fast_hz, >, 0);
    /* touch_hal_config_controller(), with one sample configuration. */
    wr(WORK, OUT_GATE | OUT_SEL | DIV_NUM(0, 0));
    wr(WORK_MEAS_NUM, MEAS_NUM(0, 500));
    wr(FREQ_PARA(0), DRV(1, 1));
    wr(INT_ENA, 0x7f);
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

/* touch_sensor_trigger_oneshot_scanning() for channel @ch. */
static void oneshot(unsigned ch)
{
    wr(MUX0, (rd(MUX0) & ~FSM_EN) | DONE_FORCE | START_FORCE);
    wr(MUX1, TOUCH_START(1u << ch) | TOUCH_XPD(1u << ch));
    wr(MUX0, rd(MUX0) | START_EN);
    wr(MUX0, rd(MUX0) & ~START_EN);
    g_assert_false(rd(CHN_STATUS) & MEAS_DONE);
    for (int i = 0; i < 20000 && !(rd(CHN_STATUS) & MEAS_DONE); i++) {
        step(1000);
    }
    g_assert_true(rd(CHN_STATUS) & MEAS_DONE);
}

static uint32_t chan_data(unsigned ch, unsigned sel, unsigned f)
{
    wr(MUX0, (rd(MUX0) & ~(DATA_SEL(3) | FREQ_SEL(3))) | DATA_SEL(sel) |
             FREQ_SEL(f));
    return DATA(rd(STATUS(ch)));
}

/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_reset_values(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    g_assert_cmphex(rd(INT_ENA), ==, 0);
    g_assert_cmphex(rd(CHN_STATUS), ==, 0x000f0000);
    g_assert_cmphex(rd(STATUS(3)), ==, 0);
    g_assert_cmphex(rd(DATE), ==, 0);
    g_assert_cmphex(rd(SCAN_CTRL1), ==, 0x00080000);
    g_assert_cmphex(rd(MUX0), ==, 0x20000000);
    wr(DATE, 0xffffffff);
    g_assert_cmphex(rd(DATE), ==, 0x80000000);
    stop();
}

/*
 * A software measurement: XPD_WAIT, then the pulses; the count follows
 * the capacitance set from QEMU, and the sensor's own capacitor with
 * TOUCH_EN_CAL.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_oneshot(void)
{
    int64_t t;
    uint32_t raw;

    start();
    wr(MUX0, (rd(MUX0) & ~FSM_EN) | DONE_FORCE | START_FORCE);
    wr(MUX1, TOUCH_START(1u << 3) | TOUCH_XPD(1u << 3));
    wr(MUX0, rd(MUX0) | START_EN);
    wr(MUX0, rd(MUX0) & ~START_EN);
    g_assert_cmpuint(SCAN_CURR(rd(CHN_STATUS)), ==, 3);
    t = 4 * 1000000000ll / fast_hz + llround(500 * 10000 / 17.0);
    step(t - 100);
    g_assert_false(rd(CHN_STATUS) & MEAS_DONE);
    step(200);
    g_assert_true(rd(CHN_STATUS) & MEAS_DONE);
    g_assert_cmphex(rd(INT_RAW), ==, INT_DONE | INT_SCAN_DONE);
    raw = chan_data(3, 0, 0);
    g_assert_cmpint(abs((int)raw - (int)expect(500, 1, 10000)), <=, 1);
    /* Benchmark and smoothed data start at the first count. */
    g_assert_cmpuint(chan_data(3, 2, 0), ==, raw);
    g_assert_cmpuint(chan_data(3, 3, 0), ==, raw);

    set_ff(3, 15000);
    oneshot(3);
    g_assert_cmpint(abs((int)chan_data(3, 0, 0) -
                        (int)expect(500, 1, 15000)), <=, 1);
    /* Divided by DIV_NUM0 + 1. */
    wr(WORK, OUT_GATE | OUT_SEL | DIV_NUM(0, 1));
    oneshot(3);
    g_assert_cmpint(abs((int)chan_data(3, 0, 0) -
                        (int)expect(500, 2, 15000)), <=, 1);
    wr(WORK, OUT_GATE | OUT_SEL);
    /* 100 steps of 20 fF: 2 pF. */
    wr(ANA_PARA, EN_CAL | DCAP_CAL(100));
    oneshot(3);
    g_assert_cmpint(abs((int)chan_data(3, 0, 0) - (int)expect(500, 1, 2000)),
                    <=, 1);
    stop();
}

/*
 * The FSM scanning its map at the touch timer's pace: one channel a
 * START, round, SCAN_DONE after the last.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_timer_scan(void)
{
    int i;

    start();
    set_ff(2, 8000);
    set_ff(5, 16000);
    wr(SCAN_CTRL1, XPD_WAIT(4) | SCAN_MAP((1u << 2) | (1u << 5)));
    wr(MUX0, FSM_EN);
    wr(PMU_TOUCH_PWR_CNTL, PWR_WAIT(2) | PWR_SLEEP(3) | PWR_TIMER_EN);
    for (i = 0; i < 100000 && !(rd(INT_RAW) & INT_DONE); i++) {
        step(1000);
    }
    g_assert_cmpuint(SCAN_CURR(rd(CHN_STATUS)), ==, 2);
    g_assert_false(rd(INT_RAW) & INT_SCAN_DONE);
    wr(INT_CLR, 0x7f);
    for (i = 0; i < 100000 && !(rd(INT_RAW) & INT_SCAN_DONE); i++) {
        step(1000);
    }
    g_assert_cmpuint(SCAN_CURR(rd(CHN_STATUS)), ==, 5);
    g_assert_cmpint(abs((int)chan_data(2, 0, 0) - (int)expect(500, 1, 8000)),
                    <=, 1);
    g_assert_cmpint(abs((int)chan_data(5, 0, 0) -
                        (int)expect(500, 1, 16000)), <=, 1);
    wr(INT_CLR, 0x7f);
    for (i = 0; i < 100000 && !(rd(INT_RAW) & INT_DONE); i++) {
        step(1000);
    }
    g_assert_cmpuint(SCAN_CURR(rd(CHN_STATUS)), ==, 2);
    /* Stopped, the timer starts nothing more. */
    wr(PMU_TOUCH_PWR_CNTL, 0);
    step(2000000);
    wr(INT_CLR, 0x7f);
    step(2000000);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    stop();
}

/*
 * A capacitance change touches the channel, through the threshold, the
 * hysteresis and the debounce; and releases it.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_detection(void)
{
    uint32_t base;

    start();
    wr(FILTER1, FILTER_EN | DEBOUNCE(2));
    wr(PAD_TH(3, 0), TH(500));
    oneshot(3);
    base = chan_data(3, 2, 0);
    oneshot(3);
    g_assert_cmpuint(chan_data(3, 2, 0), ==, base);
    wr(INT_CLR, 0x7f);
    /* 20 % more: the count rises some 1200, past 500 + 4/32 of it. */
    set_ff(3, 12000);
    oneshot(3);
    g_assert_false(rd(INT_RAW) & INT_ACTIVE);
    oneshot(3);
    g_assert_true(rd(INT_ST) & INT_ACTIVE);
    g_assert_cmphex(PAD_ACTIVE(rd(CHN_STATUS)), ==, 1u << 3);
    /* The benchmark holds while touched. */
    g_assert_cmpuint(chan_data(3, 2, 0), ==, base);
    wr(INT_CLR, 0x7f);
    set_ff(3, 10000);
    oneshot(3);
    oneshot(3);
    g_assert_true(rd(INT_ST) & INT_INACTIVE);
    g_assert_cmphex(PAD_ACTIVE(rd(CHN_STATUS)), ==, 0);
    /* Its output disabled, the channel raises nothing. */
    wr(FILTER2, rd(FILTER2) & ~(1u << (15 + 3)));
    wr(INT_CLR, 0x7f);
    set_ff(3, 12000);
    oneshot(3);
    oneshot(3);
    g_assert_false(rd(INT_RAW) & INT_ACTIVE);
    g_assert_cmphex(PAD_ACTIVE(rd(CHN_STATUS)), ==, 0);
    stop();
}

/*
 * A benchmark set by software (TOUCH_UPDATE_BENCHMARK_SW) well below the
 * count: the next measurement touches; TOUCH_STATUS_CLR clears it.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_benchmark_update(void)
{
    start();
    wr(FILTER1, FILTER_EN | DEBOUNCE(1));
    wr(PAD_TH(4, 0), TH(500));
    oneshot(4);
    wr(TOUCH_CTRL, BM_PAD(4) | 0x40);
    wr(FILTER3, BENCH_SW(1234) | UPDATE_BENCH_SW);
    g_assert_cmphex(rd(FILTER3), ==, 1234);
    g_assert_true(rd(INT_RAW) & INT_BENCH_UPDATE);
    g_assert_cmpuint(chan_data(4, 2, 0), ==, 1234);
    oneshot(4);
    g_assert_true(rd(INT_RAW) & INT_ACTIVE);
    g_assert_cmphex(PAD_ACTIVE(rd(CHN_STATUS)), ==, 1u << 4);
    wr(TOUCH_CLR, STATUS_CLR);
    g_assert_cmphex(PAD_ACTIVE(rd(CHN_STATUS)), ==, 0);
    stop();
}

/* Two frequency modes a measurement; the timeout. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_freq_hop_and_timeout(void)
{
    start();
    wr(SCAN_CTRL2, (rd(SCAN_CTRL2) & ~(0x7u << 27)) | FREQ_SCAN(2));
    wr(WORK, OUT_GATE | OUT_SEL | DIV_NUM(0, 0) | DIV_NUM(1, 3));
    wr(WORK_MEAS_NUM, MEAS_NUM(0, 500) | MEAS_NUM(1, 250));
    wr(FREQ_PARA(1), DRV(1, 1));
    oneshot(6);
    g_assert_cmpint(abs((int)chan_data(6, 0, 0) - (int)expect(500, 1, 10000)),
                    <=, 1);
    g_assert_cmpint(abs((int)chan_data(6, 0, 1) - (int)expect(250, 4, 10000)),
                    <=, 1);
    /* A timeout of 1000 cycles stops a measurement of some 5000. */
    wr(SCAN_CTRL2, rd(SCAN_CTRL2) & ~(1u << 27));
    wr(SCAN_CTRL2, (rd(SCAN_CTRL2) & ~(0xffffu << 6)) | TIMEOUT(1000));
    wr(INT_CLR, 0x7f);
    wr(MUX1, TOUCH_START(1u << 7) | TOUCH_XPD(1u << 7));
    wr(MUX0, rd(MUX0) | START_EN);
    wr(MUX0, rd(MUX0) & ~START_EN);
    step(4 * 1000000000ll / fast_hz + 1000 * 1000000000ll / fast_hz + 100);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TIMEOUT);
    g_assert_false(rd(CHN_STATUS) & MEAS_DONE);
    g_assert_cmpuint(chan_data(7, 0, 0), ==, 0);
    stop();
}

/* RST_EN_LP_TOUCH clears the data and the FSM. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_block_reset(void)
{
    start();
    oneshot(3);
    g_assert_cmpuint(chan_data(3, 0, 0), >, 0);
    wr(LPPERI_RESET_EN, RST_EN_LP_TOUCH);
    wr(LPPERI_RESET_EN, 0);
    g_assert_cmpuint(chan_data(3, 0, 0), ==, 0);
    g_assert_cmphex(rd(CHN_STATUS), ==, 0x000f0000);
    g_assert_cmphex(rd(INT_ENA), ==, 0);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/touch/reset_values", test_reset_values);
    qtest_add_func("esp32p4/touch/oneshot", test_oneshot);
    qtest_add_func("esp32p4/touch/timer_scan", test_timer_scan);
    qtest_add_func("esp32p4/touch/detection", test_detection);
    qtest_add_func("esp32p4/touch/benchmark_update", test_benchmark_update);
    qtest_add_func("esp32p4/touch/freq_hop_and_timeout",
                   test_freq_hop_and_timeout);
    qtest_add_func("esp32p4/touch/block_reset", test_block_reset);
    return g_test_run();
}
