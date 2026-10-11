/*
 * QTest testcase for the ESP32-P4 temperature sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The die temperature is set with qom-set; the sensor's range goes through
 * the analog I2C master, and every reading is checked through ESP-IDF's
 * conversion (temperature_sensor_hal_get_raw_value(), less the eFuse's
 * calibration), which must give back the temperature.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qobject/qdict.h"

#define TSENS               0x5012f000u
#define CTRL                (TSENS + 0x00)
#define CTRL2               (TSENS + 0x04)
#define INT_RAW             (TSENS + 0x08)
#define INT_ST              (TSENS + 0x0c)
#define INT_ENA             (TSENS + 0x10)
#define INT_CLR             (TSENS + 0x14)
#define INT_ENA_W1TS        (TSENS + 0x1c)
#define INT_ENA_W1TC        (TSENS + 0x20)
#define WAKEUP_CTRL         (TSENS + 0x24)
#define SAMPLE_RATE         (TSENS + 0x28)

#define OUT(v)              ((v) & 0xffu)
#define READY               (1u << 8)
#define SAMPLE_EN           (1u << 9)
#define INT_EN              (1u << 12)
#define IN_INV              (1u << 13)
#define POWER_UP            (1u << 22)
#define XPD_WAIT(n)         (n)
#define XPD_FORCE_OFF       (3u << 12)
#define WAKE_TH(lo, hi)     ((uint32_t)(lo) | (uint32_t)(hi) << 14)
#define OVER_UPPER          (1u << 29)
#define WAKE_EN             (1u << 30)
#define WAKE_MODE           (1u << 31)

#define LPPERI_RESET_EN     0x50120008u
#define RST_EN_LP_TSENS     (1u << 18)
#define CLKRST_PATH         "/machine/clkrst"
#define TSENS_PATH          "/machine/tsens"

#define I2C_ANA_MST         0x50124000u
#define I2C0_CTRL           (I2C_ANA_MST + 0x00)
#define ANA_CONF1           (I2C_ANA_MST + 0x1c)
#define ANA_CONF2           (I2C_ANA_MST + 0x20)
#define REGI2C_WRITE        (1u << 24)
#define REGI2C_BUSY         (1u << 25)
#define REGI2C_SAR_MST_SEL  (1u << 7)
#define I2C_SAR             0x69u
#define ANA_TSENS_DAC       6

#define EFUSE_SYS_PART2_DATA3 0x5012d168u

static QTestState *qts;

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

static void set_temp(int32_t mc)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': %s, 'property': 'temp-mc', "
                         "'value': %d } }", TSENS_PATH, mc);

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

/* temperature_sensor_ll_set_range() */
static void set_range(uint8_t dac)
{
    wr(ANA_CONF2, 0);
    wr(ANA_CONF1, 0);
    wr(ANA_CONF2, REGI2C_SAR_MST_SEL);
    wr(I2C0_CTRL, I2C_SAR | ANA_TSENS_DAC << 8 | (uint32_t)dac << 16 |
                  REGI2C_WRITE);
    for (int i = 0; i < 1000 && (rd(I2C0_CTRL) & REGI2C_BUSY); i++) {
        step(1000);
    }
    g_assert_false(rd(I2C0_CTRL) & REGI2C_BUSY);
}

/* ESP-IDF's range table: DAC setting and offset. */
static const struct {
    uint8_t dac;
    int offset;
    int min, max;
} ranges[] = {
    { 5, -2, 50, 125 }, { 7, -1, 20, 100 }, { 15, 0, -10, 80 },
    { 11, 1, -30, 50 }, { 10, 2, -40, 20 },
};

/*
 * temperature_sensor_get_celsius(): the HAL's integer conversion, less the
 * eFuse's calibration in tenths of a degree.
 */
static double esp_idf_celsius(uint32_t out, int offset)
{
    uint32_t cal = rd(EFUSE_SYS_PART2_DATA3) & 0x1ff;
    int delta = cal & 0x100 ? -(int)(cal & 0xff) : (int)(cal & 0xff);
    int degree = (4386 * (int)out - 278800 * offset - 205200) / 10000;

    return degree - delta / 10.0;
}

static void start(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_reset_values(void)
{
    start();
    g_assert_cmphex(rd(CTRL), ==, 0x00019400);
    g_assert_cmphex(rd(CTRL2), ==, 0x00004002);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    g_assert_cmphex(rd(WAKEUP_CTRL), ==, 0x003fc000);
    g_assert_cmphex(rd(SAMPLE_RATE), ==, 0x00000014);
    wr(CTRL, 0xffffffff);
    g_assert_cmphex(rd(CTRL) & ~0xffu, ==, 0x00fff600);
    wr(INT_ENA_W1TS, 1);
    g_assert_cmphex(rd(INT_ENA), ==, 1);
    wr(INT_ENA_W1TC, 1);
    g_assert_cmphex(rd(INT_ENA), ==, 0);
    wr(LPPERI_RESET_EN, RST_EN_LP_TSENS);
    wr(LPPERI_RESET_EN, 0);
    g_assert_cmphex(rd(CTRL), ==, 0x00019400);
    stop();
}

/*
 * Powered up, the sensor is ready after TSENS_XPD_WAIT cycles of its
 * clock; then every range reads, through ESP-IDF's conversion, the
 * temperature set from QEMU within a degree.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_readings(void)
{
    uint64_t hz;
    int64_t wait;

    start();
    hz = clk_hz("lp-tsens");
    g_assert_cmpuint(hz, >, 0);
    wr(CTRL2, XPD_WAIT(100));
    set_range(15);
    wr(CTRL, rd(CTRL) | POWER_UP);
    wait = 100 * 1000000000ll / hz;
    step(wait - 10);
    g_assert_false(rd(CTRL) & READY);
    step(20);
    g_assert_true(rd(CTRL) & READY);
    for (unsigned r = 0; r < ARRAY_SIZE(ranges); r++) {
        set_range(ranges[r].dac);
        for (int t = ranges[r].min; t <= ranges[r].max; t += 5) {
            double got;

            set_temp(t * 1000 + 300);
            got = esp_idf_celsius(OUT(rd(CTRL)), ranges[r].offset);
            /* ESP-IDF's integer division truncates toward zero. */
            if (fabs(got - (t + 0.3)) > (t < 0 ? 1.5 : 1.0)) {
                g_test_message("range %u: %d.3 C reads %.1f C", r, t, got);
                g_assert_not_reached();
            }
        }
    }
    /* TSENS_IN_INV inverts; powered down, the output holds. */
    set_range(15);
    set_temp(25000);
    wr(CTRL, rd(CTRL) | IN_INV);
    g_assert_cmpint(fabs(esp_idf_celsius(~rd(CTRL) & 0xff, 0) - 25), <=, 1);
    wr(CTRL, (rd(CTRL) & ~IN_INV) & ~POWER_UP);
    g_assert_false(rd(CTRL) & READY);
    set_temp(60000);
    g_assert_cmpint(fabs(esp_idf_celsius(OUT(rd(CTRL)), 0) - 25), <=, 1);
    /* XPD_FORCE 3 forces it off whatever TSENS_POWER_UP says. */
    wr(CTRL2, XPD_WAIT(0) | XPD_FORCE_OFF);
    wr(CTRL, rd(CTRL) | POWER_UP);
    g_assert_false(rd(CTRL) & READY);
    wr(CTRL2, XPD_WAIT(0));
    g_assert_true(rd(CTRL) & READY);
    g_assert_cmpint(fabs(esp_idf_celsius(OUT(rd(CTRL)), 0) - 60), <=, 1);
    stop();
}

/* The thresholds' codes for temperatures, as ESP-IDF computes them. */
static uint32_t regval(double celsius, int offset)
{
    return (celsius + 27.88 * offset + 20.52) / 0.4386;
}

/*
 * The automatic monitor in absolute mode: every TSENS_SAMPLE_RATE cycles,
 * a temperature above the high threshold raises the wake interrupt,
 * recorded as upper, and again once cleared; below the low one, lower.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_monitor_absolute(void)
{
    uint64_t hz;
    int64_t period;

    start();
    hz = clk_hz("lp-tsens");
    period = 1000 * 1000000000ll / hz;
    set_range(15);
    set_temp(30000);
    wr(CTRL2, XPD_WAIT(2));
    wr(SAMPLE_RATE, 1000);
    wr(WAKEUP_CTRL, WAKE_TH(regval(10, 0), regval(50, 0)) | WAKE_EN);
    wr(INT_ENA, 1);
    wr(CTRL, rd(CTRL) | POWER_UP | SAMPLE_EN);
    step(100 * period);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    set_temp(55000);
    step(period - period / 10);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    step(period / 5);
    g_assert_cmphex(rd(INT_ST), ==, 1);
    g_assert_true(rd(WAKEUP_CTRL) & OVER_UPPER);
    wr(INT_CLR, 1);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    step(period);
    g_assert_cmphex(rd(INT_RAW), ==, 1);
    set_temp(0);
    wr(INT_CLR, 1);
    step(period);
    g_assert_cmphex(rd(INT_RAW), ==, 1);
    g_assert_false(rd(WAKEUP_CTRL) & OVER_UPPER);
    /* TSENS_INT_EN clear: no interrupt; SAMPLE_EN clear: no sample. */
    wr(CTRL, rd(CTRL) & ~INT_EN);
    wr(INT_CLR, 1);
    step(10 * period);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    wr(CTRL, (rd(CTRL) | INT_EN) & ~SAMPLE_EN);
    step(10 * period);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    stop();
}

/* Change mode: a rise by more than the high threshold since the last sample. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_monitor_delta(void)
{
    uint64_t hz;
    int64_t period;

    start();
    hz = clk_hz("lp-tsens");
    period = 1000 * 1000000000ll / hz;
    set_range(15);
    set_temp(30000);
    wr(CTRL2, XPD_WAIT(2));
    wr(SAMPLE_RATE, 1000);
    /* Changes of more than 5 C (11 codes) either way. */
    wr(WAKEUP_CTRL, WAKE_TH(11, 11) | WAKE_EN | WAKE_MODE);
    wr(CTRL, rd(CTRL) | POWER_UP | SAMPLE_EN);
    step(100 * period);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    set_temp(32000);
    step(5 * period);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    set_temp(40000);
    step(period);
    g_assert_cmphex(rd(INT_RAW), ==, 1);
    g_assert_true(rd(WAKEUP_CTRL) & OVER_UPPER);
    wr(INT_CLR, 1);
    step(100 * period);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    set_temp(30000);
    step(period);
    g_assert_cmphex(rd(INT_RAW), ==, 1);
    g_assert_false(rd(WAKEUP_CTRL) & OVER_UPPER);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/tsens/reset_values", test_reset_values);
    qtest_add_func("esp32p4/tsens/readings", test_readings);
    qtest_add_func("esp32p4/tsens/monitor_absolute", test_monitor_absolute);
    qtest_add_func("esp32p4/tsens/monitor_delta", test_monitor_delta);
    return g_test_run();
}
