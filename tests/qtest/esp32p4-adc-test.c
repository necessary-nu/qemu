/*
 * QTest testcase for the ESP32-P4 SAR ADCs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The pads' voltages are set with qom-set; the SARs' initial codes go
 * through the analog I2C master as ESP-IDF's regi2c functions send them,
 * and every conversion is checked through ESP-IDF's calibration (its
 * curve fitting in integer arithmetic, from the eFuses as loaded), which
 * must give back the pad's voltage.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define ADC                 0x500de000u
#define CTRL                (ADC + 0x000)
#define CTRL2               (ADC + 0x004)
#define FILTER_CTRL1        (ADC + 0x008)
#define FSM_WAIT            (ADC + 0x00c)
#define PATT_TAB(u, n)      (ADC + 0x018 + 0x10 * (u) + 4 * (n))
#define ARB_CTRL            (ADC + 0x038)
#define FILTER_CTRL0        (ADC + 0x03c)
#define DATA_STATUS(u)      (ADC + ((u) ? 0x064 : 0x040))
#define THRES_CTRLX(x)      (ADC + 0x044 + 4 * (x))
#define THRES_CTRL          (ADC + 0x04c)
#define INT_ENA             (ADC + 0x050)
#define INT_RAW             (ADC + 0x054)
#define INT_ST              (ADC + 0x058)
#define INT_CLR             (ADC + 0x05c)
#define DMA_CONF            (ADC + 0x060)
#define CTRL_DATE           (ADC + 0x3fc)

#define WORK_MODE(m)        ((uint32_t)(m) << 2)
#define SAR_SEL             (1u << 4)
#define SAR_CLK_GATED       (1u << 5)
#define SAR_CLK_DIV(d)      ((uint32_t)(d) << 6)
#define PATT_LEN(u, n)      ((uint32_t)(n) << (14 + 4 * (u)))
#define XPD_SAR_ON(u)       (3u << (26 + 2 * (u)))
#define DATA_SAR_SEL        (1u << 24)
#define TIMER_SEL           (1u << 11)
#define TIMER_TARGET(t)     ((uint32_t)(t) << 12)
#define TIMER_EN            (1u << 24)
#define MEAS_NUM_LIMIT      (1u << 0)
#define MAX_MEAS_NUM(n)     ((uint32_t)(n) << 1)
#define FILTER_FACTOR0(f)   ((uint32_t)(f) << 29)
#define FILTER_CHANNEL0(c)  ((uint32_t)(c) << 19)
#define FILTER_CHANNEL1(c)  ((uint32_t)(c) << 14)
#define THRES(ch, hi, lo)   ((ch) | (uint32_t)(hi) << 5 | (uint32_t)(lo) << 18)
#define THRES0_EN           (1u << 31)
#define INT_THRES0_LOW      (1u << 27)
#define INT_THRES0_HIGH     (1u << 29)
#define INT_DONE(u)         (1u << (31 - (u)))
#define DMA_RESET_FSM       (1u << 30)

#define LP_ADC              0x50127000u
#define READER_CTRL(u)      (LP_ADC + ((u) ? 0x024 : 0x000))
#define MEAS_CTRL2(u)       (LP_ADC + ((u) ? 0x030 : 0x00c))
#define MEAS_MUX(u)         (LP_ADC + ((u) ? 0x034 : 0x010))
#define ATTEN(u)            (LP_ADC + ((u) ? 0x038 : 0x014))
#define FORCE_WPD_SAR       (LP_ADC + 0x03c)
#define LP_INT_RAW          (LP_ADC + 0x048)
#define LP_INT_ENA          (LP_ADC + 0x04c)
#define LP_INT_ST           (LP_ADC + 0x050)
#define LP_INT_CLR          (LP_ADC + 0x054)
#define LP_INT_W1TS         (LP_ADC + 0x058)
#define LP_INT_W1TC         (LP_ADC + 0x05c)
#define WAKEUP(u)           (LP_ADC + 0x060 + 4 * (u))
#define HW_WAKEUP(u)        (LP_ADC + 0x06c + 4 * (u))

#define MEAS_DATA(v)        ((v) & 0xffffu)
#define MEAS_DONE           (1u << 16)
#define START_SAR           (1u << 17)
#define START_FORCE         (1u << 18)
#define EN_PAD(p)           ((1u << (p)) << 19)
#define EN_PAD_FORCE        (1u << 31)
#define MUX_FORCE           (1u << 31)
#define READER_DIV(d)       (d)
#define LP_INT_DONE(u)      (1u << (u))
#define LP_INT_ERROR(u)     (1u << (2 + (u)))
#define LP_INT_WAKE(u)      (1u << (4 + (u)))
#define WAKE_TH(lo, hi)     ((uint32_t)(lo) | (uint32_t)(hi) << 14)
#define WAKE_OVER_UPPER     (1u << 29)
#define WAKE_EN             (1u << 30)
#define WAKE_MODE           (1u << 31)
#define HW_READ(rate)       (1u | (uint32_t)(rate) << 1)

#define CLKRST              0x500e6000u
#define SOC_CLK_CTRL2       (CLKRST + 0x1c)
#define PERI_CLK_CTRL23     (CLKRST + 0xa0)
#define HP_RST_EN2          (CLKRST + 0xc8)
#define ADC_APB_CLK_EN      (1u << 5)
#define ADC_CLK_EN          (1u << 0)
#define RST_EN_ADC          (1u << 10)
#define LPPERI_RESET_EN     0x50120008u
#define RST_EN_LP_ADC       (1u << 24)
#define CLKRST_PATH         "/machine/clkrst"

#define I2C_ANA_MST         0x50124000u
#define I2C0_CTRL           (I2C_ANA_MST + 0x00)
#define ANA_CONF1           (I2C_ANA_MST + 0x1c)
#define ANA_CONF2           (I2C_ANA_MST + 0x20)
#define REGI2C_WRITE        (1u << 24)
#define REGI2C_BUSY         (1u << 25)
#define REGI2C_SAR_MST_SEL  (1u << 7)
#define I2C_SAR             0x69u
#define ANA_ENCAL           7
#define ANA_ENCAL_GND(u)    (1u << ((u) ? 7 : 5))

#define EFUSE_BLOCK2        0x5012d05cu
#define EFUSE_BLOCK10       0x5012d15cu

#define ADC_PATH            "/machine/adc"

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

static void qom_set_uint(const char *path, const char *prop, uint64_t v)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': %s, 'property': %s, 'value': %" PRIu64
                         " } }", path, prop, v);

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

static void set_mv(unsigned unit, unsigned ch, uint32_t mv)
{
    g_autofree char *prop = g_strdup_printf("adc%u-ch%u-mv", unit + 1, ch);

    qom_set_uint(ADC_PATH, prop, mv);
}

/* ESP-IDF's regi2c_impl.c: hand the SAR's block to master 0, then write. */
static void ana_write(uint8_t reg, uint8_t data)
{
    wr(ANA_CONF2, 0);
    wr(ANA_CONF1, 0);
    wr(ANA_CONF2, REGI2C_SAR_MST_SEL);
    wr(I2C0_CTRL, I2C_SAR | reg << 8 | (uint32_t)data << 16 | REGI2C_WRITE);
    for (int i = 0; i < 1000 && (rd(I2C0_CTRL) & REGI2C_BUSY); i++) {
        step(1000);
    }
    g_assert_false(rd(I2C0_CTRL) & REGI2C_BUSY);
}

static void set_init_code(unsigned unit, uint32_t code)
{
    ana_write(unit ? 4 : 1, code >> 8);
    ana_write(unit ? 3 : 0, code & 0xff);
}

static void set_encal_gnd(unsigned unit, bool on)
{
    ana_write(ANA_ENCAL, on ? ANA_ENCAL_GND(unit) : 0);
}

/* ---- ESP-IDF's calibration, as esp_adc computes it for the P4 ---- */

static uint32_t efuse_field(uint32_t base, unsigned bit, unsigned width)
{
    uint64_t v = rd(base + 4 * (bit / 32));

    if (bit % 32 + width > 32) {
        v |= (uint64_t)rd(base + 4 * (bit / 32 + 1)) << 32;
    }
    return (v >> (bit % 32)) & ((1u << width) - 1);
}

static int sign_mag(uint32_t v, unsigned sign_bit)
{
    return v & (1u << sign_bit) ? -(int)(v & ~(1u << sign_bit)) : (int)v;
}

/* esp_efuse_rtc_calib_get_init_code() */
static uint32_t efuse_init_code(unsigned unit, unsigned atten)
{
    return efuse_field(EFUSE_BLOCK2, 128 + 40 * unit + 10 * atten, 10) + 1400;
}

/* curve_fitting_coefficients.c, calibration version 1 */
static const uint64_t err_coef[2][4][2][2] = {
    {
        { { 7170501832480995ull, 10000000000000000ull },
          { 10598497992115ull, 10000000000000000ull } },
        { { 9960085535084866ull, 10000000000000000ull },
          { 15840076608145ull, 10000000000000000ull } },
        { { 14711053224678996ull, 10000000000000000ull },
          { 1594266424857ull, 100000000000000000ull } },
        { { 28811493455181565ull, 10000000000000000ull },
          { 10082311568625ull, 10000000000000000ull } },
    },
    {
        { { 4900967548489932ull, 10000000000000000ull },
          { 5037402667913ull, 10000000000000000ull } },
        { { 7296214814536025ull, 10000000000000000ull },
          { 11021577596635ull, 10000000000000000ull } },
        { { 10991620450220592ull, 10000000000000000ull },
          { 11623930881896ull, 10000000000000000ull } },
        { { 24421401024626730ull, 10000000000000000ull },
          { 9458501263393ull, 10000000000000000ull } },
    },
};

/* The channel's offset, which ESP-IDF's calibration takes off the code. */
static int chan_comp(unsigned unit, unsigned ch, unsigned atten)
{
    return sign_mag(efuse_field(EFUSE_BLOCK10, 40 + 32 * unit + 4 * ch, 4),
                    3) * (4 - (int)atten);
}

/* adc_cali_curve_fitting.c's cali_raw_to_voltage() */
static int cali_mv(unsigned unit, unsigned atten, unsigned ch, int raw)
{
    static const uint32_t ref_mv[4] = { 600, 800, 1200, 2300 };
    static const uint32_t chk[4] = { 2300, 2300, 2350, 2350 };
    uint32_t hi = unit ? efuse_field(EFUSE_BLOCK10, 10 * atten, 10)
                       : efuse_field(EFUSE_BLOCK2, 208 + 10 * atten, 10);
    uint32_t digi = chk[atten] + sign_mag(hi, 9);
    int comp = chan_comp(unit, ch, atten);
    uint32_t coeff_a = 65536 * ref_mv[atten] / digi;
    uint64_t v1, term0, term1;
    int32_t error;

    raw -= comp;
    raw = raw < 0 ? 0 : raw > 4095 ? 4095 : raw;
    v1 = (uint64_t)raw * coeff_a / 65536;
    if (v1 == 0) {
        return 0;
    }
    term0 = err_coef[unit][atten][0][0] / err_coef[unit][atten][0][1];
    term1 = v1 * err_coef[unit][atten][1][0] / err_coef[unit][atten][1][1];
    error = -(int32_t)term0 + (int32_t)term1;
    return (int32_t)v1 - error;
}

/* ---- The LP controller ---- */

static void start(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | ADC_APB_CLK_EN);
    wr(PERI_CLK_CTRL23, rd(PERI_CLK_CTRL23) | ADC_CLK_EN);
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

/* As adc_oneshot_hal_setup(): SW start and pad select, attenuation. */
static void lp_setup(unsigned unit, unsigned ch, unsigned atten)
{
    unsigned pos = unit ? ch + 2 : ch;
    uint32_t a = rd(ATTEN(unit));

    a = (a & ~(3u << (2 * pos))) | atten << (2 * pos);
    wr(ATTEN(unit), a);
    wr(MEAS_MUX(unit), unit ? MUX_FORCE : 0);
    wr(MEAS_CTRL2(unit), START_FORCE | EN_PAD_FORCE | EN_PAD(pos));
}

static uint32_t lp_convert(unsigned unit)
{
    uint32_t m = rd(MEAS_CTRL2(unit)) & ~START_SAR;

    wr(MEAS_CTRL2(unit), m);
    wr(MEAS_CTRL2(unit), m | START_SAR);
    g_assert_false(rd(MEAS_CTRL2(unit)) & MEAS_DONE);
    for (int i = 0; i < 10000 && !(rd(MEAS_CTRL2(unit)) & MEAS_DONE); i++) {
        step(1000);
    }
    g_assert_true(rd(MEAS_CTRL2(unit)) & MEAS_DONE);
    return MEAS_DATA(rd(MEAS_CTRL2(unit)));
}

/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_reset_values(void)
{
    start();
    g_assert_cmphex(rd(CTRL), ==, 0x403fc120);
    g_assert_cmphex(rd(CTRL2), ==, 0x0000a1fe);
    g_assert_cmphex(rd(FSM_WAIT), ==, 0x00ff0808);
    g_assert_cmphex(rd(ARB_CTRL), ==, 0x00000900);
    g_assert_cmphex(rd(FILTER_CTRL0), ==, 0x006b4000);
    g_assert_cmphex(rd(THRES_CTRLX(0)), ==, 0x0003ffed);
    g_assert_cmphex(rd(THRES_CTRLX(1)), ==, 0x0003ffed);
    g_assert_cmphex(rd(DMA_CONF), ==, 0x000000ff);
    g_assert_cmphex(rd(ADC + 0x068), ==, 0x00008000);
    g_assert_cmphex(rd(CTRL_DATE), ==, 0x02212260);
    g_assert_cmphex(rd(READER_CTRL(0)), ==, 0x20000002);
    g_assert_cmphex(rd(READER_CTRL(1)), ==, 0x40010002);
    g_assert_cmphex(rd(ATTEN(0)), ==, 0xffffffff);
    g_assert_cmphex(rd(ATTEN(1)), ==, 0xffffffff);
    g_assert_cmphex(rd(WAKEUP(0)), ==, 0x03ffc000);
    g_assert_cmphex(rd(HW_WAKEUP(1)), ==, 0x000000c8);
    /* Read-only fields and write-1-to-clear raws. */
    wr(MEAS_CTRL2(0), 0xffffffff);
    g_assert_cmphex(rd(MEAS_CTRL2(0)) & 0x1ffff, ==, 0);
    wr(LP_INT_W1TS, 0x3f);
    g_assert_cmphex(rd(LP_INT_ENA), ==, 0x3f);
    wr(LP_INT_W1TC, 0x05);
    g_assert_cmphex(rd(LP_INT_ENA), ==, 0x3a);
    stop();
}

/*
 * Every attenuation of both SARs, across its range: ESP-IDF's calibration
 * gives back the voltage, with the eFuse's initial codes set, and the
 * voltage set from QEMU at run time.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_oneshot_calibrated(void)
{
    static const uint32_t max_mv[4] = { 950, 1250, 1750, 2450 };

    start();
    for (unsigned unit = 0; unit < 2; unit++) {
        unsigned ch = unit ? 3 : 5;

        for (unsigned atten = 0; atten < 4; atten++) {
            set_init_code(unit, efuse_init_code(unit, atten));
            lp_setup(unit, ch, atten);
            for (uint32_t mv = 150; mv <= max_mv[atten]; mv += 200) {
                int got;

                set_mv(unit, ch, mv);
                got = cali_mv(unit, atten, ch, lp_convert(unit));
                if (abs(got - (int)mv) > 3) {
                    g_test_message("ADC%u ch%u atten %u: %u mV reads %d mV",
                                   unit + 1, ch, atten, mv, got);
                    g_assert_not_reached();
                }
            }
        }
    }
    /* Without its initial code the SAR reads its offset. */
    set_init_code(0, 0);
    lp_setup(0, 0, 0);
    set_mv(0, 0, 0);
    g_assert_cmpuint(lp_convert(0), ==,
                     efuse_init_code(0, 0) + chan_comp(0, 0, 0));
    /* Full scale saturates; SARn_DATA_INV inverts. */
    set_init_code(0, efuse_init_code(0, 0));
    set_mv(0, 0, 1500);
    g_assert_cmpuint(lp_convert(0), ==, 4095);
    wr(READER_CTRL(0), rd(READER_CTRL(0)) | (1u << 28));
    g_assert_cmpuint(lp_convert(0), ==, 0);
    stop();
}

/*
 * ESP-IDF's self-calibration (adc_hal_self_calibration()): with the SAR on
 * ground and no pad selected, the smallest initial code reading 0 is the
 * one the eFuse records.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_self_calibration(void)
{
    start();
    for (unsigned unit = 0; unit < 2; unit++) {
        for (unsigned atten = 0; atten < 4; atten++) {
            uint32_t h = 4096, l = 0, code;

            lp_setup(unit, 0, atten);
            wr(MEAS_CTRL2(unit), START_FORCE | EN_PAD_FORCE);
            set_encal_gnd(unit, true);
            while (h - l > 1) {
                code = (h + l) / 2;
                set_init_code(unit, code);
                if (lp_convert(unit) == 0) {
                    h = code;
                } else {
                    l = code;
                }
            }
            g_assert_cmpuint(h, ==, efuse_init_code(unit, atten));
            set_encal_gnd(unit, false);
        }
    }
    stop();
}

/* A conversion takes 25 cycles of the LP SAR clock, divided by SARn_CLK_DIV. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_oneshot_timing(void)
{
    uint64_t hz;
    int64_t t25;

    start();
    hz = clk_hz("lp-adc-sar1");
    g_assert_cmpuint(hz, >, 0);
    for (unsigned div = 1; div <= 4; div *= 2) {
        uint32_t m;

        wr(READER_CTRL(0), (rd(READER_CTRL(0)) & ~0xffu) | READER_DIV(div));
        lp_setup(0, 1, 3);
        m = rd(MEAS_CTRL2(0));
        wr(MEAS_CTRL2(0), m | START_SAR);
        t25 = 25 * div * 1000000000ll / hz;
        step(t25 - 50);
        g_assert_false(rd(MEAS_CTRL2(0)) & MEAS_DONE);
        step(100);
        g_assert_true(rd(MEAS_CTRL2(0)) & MEAS_DONE);
        g_assert_true(rd(LP_INT_RAW) & LP_INT_DONE(0));
        wr(LP_INT_CLR, LP_INT_DONE(0));
    }
    /* SAR1_INT_EN clear: done, without the interrupt. */
    wr(READER_CTRL(0), rd(READER_CTRL(0)) & ~(1u << 29));
    lp_convert(0);
    g_assert_false(rd(LP_INT_RAW) & LP_INT_DONE(0));
    stop();
}

/*
 * SAR1_DIG_FORCE gives ADC1 to the HP controller: the LP controller's
 * start is refused with its error interrupt.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_lp_arbitration(void)
{
    start();
    wr(LP_INT_ENA, 0x3f);
    lp_setup(0, 0, 0);
    wr(MEAS_MUX(0), MUX_FORCE);
    wr(MEAS_CTRL2(0), rd(MEAS_CTRL2(0)) | START_SAR);
    step(1000000);
    g_assert_false(rd(MEAS_CTRL2(0)) & MEAS_DONE);
    g_assert_cmphex(rd(LP_INT_ST), ==, LP_INT_ERROR(0));
    /* A SAR forced off reads 0. */
    wr(MEAS_MUX(0), 0);
    wr(FORCE_WPD_SAR, 2);
    set_init_code(0, efuse_init_code(0, 0));
    set_mv(0, 0, 500);
    g_assert_cmpuint(lp_convert(0), ==, 0);
    wr(FORCE_WPD_SAR, 3);
    g_assert_cmpuint(lp_convert(0), >, 1000);
    stop();
}

/*
 * The LP automatic monitor: every HW_READ_RATE cycles of LPADC_CLK, a
 * wakeup above the high threshold, then, in change mode, on a fall.
 * Each sample's conversion takes 250 cycles of LPADC_CLK.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_lp_monitor(void)
{
    uint64_t hz;
    int64_t period;

    start();
    hz = clk_hz("lp-adc-func");
    period = 1000 * 1000000000ll / hz;
    set_init_code(0, efuse_init_code(0, 3));
    lp_setup(0, 4, 3);
    set_mv(0, 4, 1000);
    wr(LP_INT_ENA, LP_INT_WAKE(0));
    wr(WAKEUP(0), WAKE_TH(500, 2000) | WAKE_EN);
    wr(HW_WAKEUP(0), HW_READ(1000));
    step(3 * period);
    g_assert_false(rd(LP_INT_RAW) & LP_INT_WAKE(0));
    g_assert_cmpuint(MEAS_DATA(rd(MEAS_CTRL2(0))), >, 900);
    g_assert_false(rd(MEAS_CTRL2(0)) & MEAS_DONE);
    set_mv(0, 4, 2200);
    step(2 * period);
    g_assert_true(rd(LP_INT_ST) & LP_INT_WAKE(0));
    g_assert_true(rd(WAKEUP(0)) & WAKE_OVER_UPPER);
    wr(LP_INT_CLR, LP_INT_WAKE(0));
    /* Change mode: steady, nothing; a fall of more than 100 codes. */
    wr(WAKEUP(0), WAKE_TH(100, 100) | WAKE_EN | WAKE_MODE);
    step(3 * period);
    g_assert_false(rd(LP_INT_RAW) & LP_INT_WAKE(0));
    set_mv(0, 4, 1800);
    step(2 * period);
    g_assert_true(rd(LP_INT_RAW) & LP_INT_WAKE(0));
    g_assert_false(rd(WAKEUP(0)) & WAKE_OVER_UPPER);
    stop();
}

/* ---- The HP controller ---- */

static uint32_t patt(unsigned pos, unsigned atten)
{
    return pos << 2 | atten;
}

/* Entries 0 to 3 of a pattern table, entry 0 in bits 23:18. */
static uint32_t patt4(uint32_t e0, uint32_t e1, uint32_t e2, uint32_t e3)
{
    return e0 << 18 | e1 << 12 | e2 << 6 | e3;
}

/*
 * Timer sampling of ADC1 through a two-entry pattern: the data status,
 * the done interrupt, the threshold monitor and the timing.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_hp_pattern(void)
{
    uint64_t hz;
    int64_t tick, conv;
    uint32_t st;

    start();
    hz = clk_hz("adc");
    g_assert_cmpuint(hz, >, 0);
    set_init_code(0, efuse_init_code(0, 3));
    set_mv(0, 2, 1200);
    set_mv(0, 6, 2000);
    wr(MEAS_MUX(0), MUX_FORCE);
    wr(PATT_TAB(0, 0), patt4(patt(2, 3), patt(6, 3), 0, 0));
    wr(CTRL, SAR_CLK_GATED | SAR_CLK_DIV(1) | PATT_LEN(0, 1) |
             PATT_LEN(1, 0) | XPD_SAR_ON(0) | DATA_SAR_SEL);
    wr(THRES_CTRLX(0), THRES(6, 1800, 0));
    wr(THRES_CTRL, THRES0_EN);
    wr(INT_ENA, INT_DONE(0) | INT_THRES0_HIGH);
    /* 100 SAR clock cycles a sample, 25 a conversion. */
    tick = 100 * 2 * 1000000000ll / hz;
    conv = 25 * 2 * 1000000000ll / hz;
    wr(CTRL2, TIMER_SEL | TIMER_TARGET(100) | TIMER_EN);
    step(tick + conv - 50);
    g_assert_false(rd(INT_RAW) & INT_DONE(0));
    step(100);
    g_assert_cmphex(rd(INT_ST), ==, INT_DONE(0));
    st = rd(DATA_STATUS(0));
    g_assert_cmpuint((st >> 13) & 0xf, ==, 2);
    g_assert_cmpint(abs(cali_mv(0, 3, 2, st & 0xfff) - 1200), <=, 3);
    wr(INT_CLR, INT_DONE(0));
    step(tick);
    st = rd(DATA_STATUS(0));
    g_assert_cmpuint((st >> 13) & 0xf, ==, 6);
    g_assert_cmpint(abs(cali_mv(0, 3, 6, st & 0xfff) - 2000), <=, 3);
    /* 2000 mV reads above the monitor's 1800 codes. */
    g_assert_true(rd(INT_ST) & INT_THRES0_HIGH);
    step(tick);
    g_assert_cmpuint((rd(DATA_STATUS(0)) >> 13) & 0xf, ==, 2);
    /* TIMER_EN clear stops sampling. */
    wr(CTRL2, TIMER_SEL | TIMER_TARGET(100));
    wr(INT_CLR, 0xffffffff);
    step(4 * tick);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    stop();
}

/* Both SARs at once, and in turn, with MEAS_NUM_LIMIT stopping the timer. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_hp_work_modes(void)
{
    uint64_t hz;
    int64_t tick;

    start();
    hz = clk_hz("adc");
    tick = 200 * 2 * 1000000000ll / hz;
    wr(MEAS_MUX(0), MUX_FORCE);
    wr(PATT_TAB(0, 0), patt4(patt(1, 0), 0, 0, 0));
    wr(PATT_TAB(1, 0), patt4(patt(4, 0), 0, 0, 0));
    wr(CTRL, SAR_CLK_GATED | SAR_CLK_DIV(1) | WORK_MODE(1) |
             XPD_SAR_ON(0) | XPD_SAR_ON(1) | DATA_SAR_SEL);
    wr(CTRL2, TIMER_SEL | TIMER_TARGET(200) | TIMER_EN);
    step(tick + tick / 2);
    g_assert_cmphex(rd(INT_RAW), ==, INT_DONE(0) | INT_DONE(1));
    g_assert_cmphex(rd(DATA_STATUS(1)) >> 13, ==, 4);
    wr(CTRL2, 0);
    wr(INT_CLR, 0xffffffff);
    /* Alternate: FSM1 first. */
    wr(DMA_CONF, rd(DMA_CONF) | DMA_RESET_FSM);
    wr(DMA_CONF, rd(DMA_CONF) & ~DMA_RESET_FSM);
    wr(CTRL, SAR_CLK_GATED | SAR_CLK_DIV(1) | WORK_MODE(2) |
             XPD_SAR_ON(0) | XPD_SAR_ON(1));
    wr(CTRL2, TIMER_SEL | TIMER_TARGET(200) | TIMER_EN | MEAS_NUM_LIMIT |
              MAX_MEAS_NUM(3));
    step(tick + tick / 2);
    g_assert_cmphex(rd(INT_RAW), ==, INT_DONE(0));
    step(tick);
    g_assert_cmphex(rd(INT_RAW), ==, INT_DONE(0) | INT_DONE(1));
    wr(INT_CLR, 0xffffffff);
    step(tick);
    g_assert_cmphex(rd(INT_RAW), ==, INT_DONE(0));
    wr(INT_CLR, 0xffffffff);
    step(4 * tick);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    stop();
}

/*
 * Filter 0 on ADC2's channel 1 (position 3, filter channel 11), k = 4: a
 * step settles a quarter at a time; the monitor's low threshold.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_hp_filter_monitor(void)
{
    uint64_t hz;
    int64_t tick;
    uint32_t prev, cur, base;

    start();
    hz = clk_hz("adc");
    tick = 100 * 2 * 1000000000ll / hz;
    set_init_code(1, efuse_init_code(1, 0));
    set_mv(1, 1, 200);
    wr(PATT_TAB(1, 0), patt4(patt(3, 0), 0, 0, 0));
    wr(CTRL, SAR_CLK_GATED | SAR_CLK_DIV(1) | SAR_SEL | PATT_LEN(1, 0) |
             XPD_SAR_ON(1) | DATA_SAR_SEL);
    wr(FILTER_CTRL0, FILTER_CHANNEL0(11) | FILTER_CHANNEL1(13));
    wr(FILTER_CTRL1, FILTER_FACTOR0(2));
    wr(THRES_CTRLX(0), THRES(11, 8191, 300));
    wr(THRES_CTRL, THRES0_EN);
    wr(CTRL2, TIMER_SEL | TIMER_TARGET(100) | TIMER_EN);
    /* Between samples, each converted by now. */
    step(3 * tick + tick / 2);
    base = rd(DATA_STATUS(1)) & 0xfff;
    g_assert_cmphex(rd(DATA_STATUS(1)) >> 13, ==, 3);
    /* 200 mV at 0 dB, some 770 codes: above the low threshold. */
    g_assert_false(rd(INT_RAW) & INT_THRES0_LOW);
    set_mv(1, 1, 800);
    prev = base;
    for (int i = 0; i < 4; i++) {
        step(tick);
        cur = rd(DATA_STATUS(1)) & 0xfff;
        g_assert_cmpuint(cur, >, prev);
        prev = cur;
    }
    /* Far from settled after four samples at k = 4. */
    set_mv(1, 1, 50);
    for (int i = 0; i < 30; i++) {
        step(tick);
    }
    g_assert_true(rd(INT_RAW) & INT_THRES0_LOW);
    stop();
}

/* RST_EN_ADC and RST_EN_LP_ADC reset each controller's registers. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_block_resets(void)
{
    start();
    wr(CTRL2, 0x5);
    wr(ATTEN(0), 0);
    wr(HP_RST_EN2, RST_EN_ADC);
    wr(CTRL2, 0x7);
    wr(HP_RST_EN2, 0);
    g_assert_cmphex(rd(CTRL2), ==, 0x0000a1fe);
    g_assert_cmphex(rd(ATTEN(0)), ==, 0);
    wr(LPPERI_RESET_EN, RST_EN_LP_ADC);
    wr(LPPERI_RESET_EN, 0);
    g_assert_cmphex(rd(ATTEN(0)), ==, 0xffffffff);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/adc/reset_values", test_reset_values);
    qtest_add_func("esp32p4/adc/oneshot_calibrated", test_oneshot_calibrated);
    qtest_add_func("esp32p4/adc/self_calibration", test_self_calibration);
    qtest_add_func("esp32p4/adc/oneshot_timing", test_oneshot_timing);
    qtest_add_func("esp32p4/adc/lp_arbitration", test_lp_arbitration);
    qtest_add_func("esp32p4/adc/lp_monitor", test_lp_monitor);
    qtest_add_func("esp32p4/adc/hp_pattern", test_hp_pattern);
    qtest_add_func("esp32p4/adc/hp_work_modes", test_hp_work_modes);
    qtest_add_func("esp32p4/adc/hp_filter_monitor", test_hp_filter_monitor);
    qtest_add_func("esp32p4/adc/block_resets", test_block_resets);
    return g_test_run();
}
