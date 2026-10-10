/*
 * QTest testcase for the ESP32 analog blocks: the analog I2C bus with the
 * BBPLL and APLL, the SAR ADCs, the DACs, the touch sensor and the Hall
 * sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define ANA                     0x3ff4e000
#define ANA_HOST(n)             (ANA + 4 * (n))
#define ANA_CONFIG              (ANA + 0x44)
#define ANA_APB_ALIAS           0x6000e000
#define HOST_APLL               3
#define HOST_BBPLL              4
#define SLAVE_BBPLL             0x66
#define SLAVE_APLL              0x6d
#define I2C_WR                  (1u << 24)
#define I2C_BUSY                (1u << 25)

#define RTC_CNTL                0x3ff48000
#define OPTIONS0                (RTC_CNTL + 0x00)
#define STATE0                  (RTC_CNTL + 0x18)
#define ANA_CONF                (RTC_CNTL + 0x30)
#define INT_RAW                 (RTC_CNTL + 0x40)
#define INT_CLR                 (RTC_CNTL + 0x48)
#define CLK_CONF                (RTC_CNTL + 0x70)
#define BIAS_I2C_FORCE_PD       (1u << 18)
#define BBPLL_FORCE_PD          (1u << 10)
#define TOUCH_SLP_TIMER_EN      (1u << 23)
#define PLLA_FORCE_PU           (1u << 24)
#define PLLA_FORCE_PD           (1u << 23)
#define INT_TOUCH               (1u << 6)
#define SOC_CLK_SEL_SHIFT       27
#define ANA_CLK_RTC_SEL_MASK    (3u << 30)
#define ANA_CLK_RTC_SEL_8MD256  (2u << 30)
#define FAST_CLK_RTC_SEL_8M     (1u << 29)
#define CK8M_DIV_SEL_MASK       (7u << 12)
#define ENB_CK8M                (1u << 6)
/* RC_FAST_CLK's start-up: the reset CK8M_WAIT, 16 cycles of RC_SLOW_CLK */
#define CK8M_STARTUP_NS         106667

#define RTCIO                   0x3ff48400
#define RTCIO_HALL_SENS         (RTCIO + 0x78)
#define RTCIO_PAD_DAC1          (RTCIO + 0x84)
#define RTCIO_PAD_DAC2          (RTCIO + 0x88)
#define RTCIO_TOUCH_PAD(n)      (RTCIO + 0x94 + 4 * (n))
#define XPD_HALL                (1u << 31)
#define HALL_PHASE              (1u << 30)
#define PDAC_DAC(v)             ((v) << 19)
#define PDAC_XPD_DAC            (1u << 18)
#define PDAC_XPD_FORCE          (1u << 10)
#define TOUCH_XPD               (1u << 20)

#define SENS                    0x3ff48800
#define READ_CTRL               (SENS + 0x00)
#define START_FORCE             (SENS + 0x2c)
#define ATTEN1                  (SENS + 0x34)
#define SLAVE_ADDR1             (SENS + 0x3c)
#define MEAS_START1             (SENS + 0x54)
#define TOUCH_CTRL1             (SENS + 0x58)
#define TOUCH_THRES(n)          (SENS + 0x5c + 4 * ((n) / 2))
#define TOUCH_OUT(n)            (SENS + 0x70 + 4 * ((n) / 2))
#define TOUCH_CTRL2             (SENS + 0x84)
#define TOUCH_ENABLE            (SENS + 0x8c)
#define READ_CTRL2              (SENS + 0x90)
#define MEAS_START2             (SENS + 0x94)
#define DAC_CTRL1               (SENS + 0x98)
#define DAC_CTRL2               (SENS + 0x9c)
#define SARDATE                 (SENS + 0xfc)
#define DATA_INV                (1u << 28)
#define DIG_FORCE1              (1u << 27)
#define PWDET_FORCE2            (1u << 27)
#define DATA_INV2               (1u << 29)
#define MEAS_DONE               (1u << 16)
#define MEAS_START_SAR          (1u << 17)
#define MEAS_START_FORCE        (1u << 18)
#define EN_PAD(ch)              ((1u << (ch)) << 19)
#define EN_PAD_FORCE            (1u << 31)
#define MEAS_STATUS(v)          (((v) >> 22) & 0xff)
#define XPD_HALL_FORCE          (1u << 26)
#define HALL_PHASE_FORCE        (1u << 27)
#define TOUCH_MEAS_DONE         (1u << 10)
#define TOUCH_START_EN          (1u << 12)
#define TOUCH_START_FORCE       (1u << 13)
#define TOUCH_MEAS_EN_CLR       (1u << 30)
#define SW_TONE_EN              (1u << 16)
#define DAC_DIG_FORCE           (1u << 22)
#define CW_EN1                  (1u << 24)
#define INV1(v)                 ((v) << 20)

#define SYSCON                  0x3ff66000
#define SARADC_CTRL             (SYSCON + 0x10)
#define SARADC_CTRL2            (SYSCON + 0x14)
#define SAR1_PATT_TAB1          (SYSCON + 0x1c)
#define DATA_TO_I2S             (1u << 26)
#define DATA_SAR_SEL            (1u << 25)
#define SAR1_PATT_P_CLEAR       (1u << 23)
#define SAR1_PATT_LEN(n)        (((n) - 1) << 15)
#define SARADC_CTRL_RESET       ((15u << 19) | (4u << 7) | (1u << 6))
#define SAR1_INV                (1u << 9)

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define DPORT_CPU_PER_CONF      0x3ff0003c
#define PERIP_I2S0              (1u << 4)
#define I2S0                    0x3ff4f000
#define I2S_FIFO_WR             (I2S0 + 0x00)
#define I2S_FIFO_RD             (I2S0 + 0x04)
#define I2S_CONF                (I2S0 + 0x08)
#define I2S_FIFO_CONF           (I2S0 + 0x20)
#define I2S_CONF_CHAN           (I2S0 + 0x2c)
#define I2S_CONF2               (I2S0 + 0xa8)
#define I2S_CLKM_CONF           (I2S0 + 0xac)
#define I2S_SAMPLE_RATE_CONF    (I2S0 + 0xb0)
#define I2S_TX_START            (1u << 4)
#define I2S_RX_START            (1u << 5)
#define I2S_TX_FIFO_MOD(m)      ((m) << 13)
#define I2S_LCD_EN              (1u << 5)
#define I2S_FIFO_FORCE          ((1u << 19) | (1u << 20))
#define I2S_RX_FIFO_MOD(m)      ((m) << 16)
#define I2S_SAMPLE_PERIOD_NS    1250

#define TIMG0                   0x3ff5f000
#define T0CONFIG_RUN            (0xc0000000u | (2u << 13))

#define GPIO_PATH               "/machine/soc/gpio"
#define SENS_PATH               "/machine/soc/sens"
#define PAD_IN                  "esp32-gpio-pad-in"

#define US                      1000

static QTestState *start(void)
{
    return qtest_init("-M esp32 -nic none");
}

static uint32_t rd(QTestState *qts, uint64_t a)
{
    return qtest_readl(qts, a);
}

static void wr(QTestState *qts, uint64_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

/*
 * RTC_FAST_CLK from the 8 MHz oscillator undivided, as ESP-IDF's
 * rtc_clk_init leaves it. It resets to XTAL / 4, 10 MHz, and CK8M_DIV_SEL
 * to 2 (divide by 3).
 */
static void fast_clk_8m(QTestState *qts)
{
    wr(qts, CLK_CONF, (rd(qts, CLK_CONF) & ~CK8M_DIV_SEL_MASK) |
       FAST_CLK_RTC_SEL_8M);
}

/* Power RC_FAST_CLK's oscillator down or up */
static void rc_fast_power(QTestState *qts, bool on)
{
    uint32_t v = rd(qts, CLK_CONF);

    wr(qts, CLK_CONF, on ? v & ~ENB_CK8M : v | ENB_CK8M);
}

static void set_prop(QTestState *qts, const char *prop, int64_t value)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                           "{ 'path': %s, 'property': %s, "
                           "'value': %" PRId64 " } }", SENS_PATH, prop,
                           value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

static int64_t get_prop(QTestState *qts, const char *prop)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                           "{ 'path': %s, 'property': %s } }", SENS_PATH,
                           prop);
    int64_t v;

    g_assert(qdict_haskey(rsp, "return"));
    v = qdict_get_int(rsp, "return");
    qobject_unref(rsp);
    return v;
}

static uint8_t i2c_read(QTestState *qts, unsigned host, uint8_t slave,
                        uint8_t reg)
{
    uint32_t v;

    wr(qts, ANA_HOST(host), slave | (reg << 8));
    v = rd(qts, ANA_HOST(host));
    g_assert_cmphex(v & I2C_BUSY, ==, 0);
    return v >> 16;
}

static void i2c_write(QTestState *qts, unsigned host, uint8_t slave,
                      uint8_t reg, uint8_t data)
{
    wr(qts, ANA_HOST(host), slave | (reg << 8) | (data << 16) | I2C_WR);
    g_assert_cmphex(rd(qts, ANA_HOST(host)) & I2C_BUSY, ==, 0);
}

/* Timer group 0 cycles, at APB_CLK / 2, over 100 us, as APB_CLK cycles */
static uint32_t apb_per_100us(QTestState *qts)
{
    uint32_t before, after;

    wr(qts, TIMG0 + 0x0c, 1);
    before = rd(qts, TIMG0 + 0x04);
    qtest_clock_step(qts, 100 * US);
    wr(qts, TIMG0 + 0x0c, 1);
    after = rd(qts, TIMG0 + 0x04);
    return (after - before) * 2;
}

static void select_soc_clk(QTestState *qts, uint32_t sel)
{
    uint32_t v = rd(qts, CLK_CONF) & ~(3u << SOC_CLK_SEL_SHIFT);

    wr(qts, CLK_CONF, v | (sel << SOC_CLK_SEL_SHIFT));
}

/*
 * The I2C master reaches the BBPLL and the APLL; the BBPLL's registers
 * come up with the 320 MHz configuration, its lock outputs read high and
 * ignore writes, and a calibration completes when started. A slave that
 * ANA_CONFIG disconnects, or a bus that is powered down, does not answer.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_analog_i2c(void)
{
    QTestState *qts = start();

    g_assert_cmphex(i2c_read(qts, HOST_BBPLL, SLAVE_BBPLL, 11), ==, 0x43);
    g_assert_cmphex(i2c_read(qts, HOST_BBPLL, SLAVE_BBPLL, 3), ==, 32);
    g_assert_cmphex(i2c_read(qts, HOST_BBPLL, SLAVE_BBPLL, 6), ==, 0x03);
    i2c_write(qts, HOST_BBPLL, SLAVE_BBPLL, 6, 0);
    g_assert_cmphex(i2c_read(qts, HOST_BBPLL, SLAVE_BBPLL, 6), ==, 0x03);

    /* Through the APB alias, as the ROM's rom_i2c_writeReg goes */
    qtest_writel(qts, ANA_APB_ALIAS + 4 * HOST_APLL,
                 SLAVE_APLL | (7 << 8) | (6 << 16) | I2C_WR);
    g_assert_cmphex(i2c_read(qts, HOST_APLL, SLAVE_APLL, 7), ==, 6);

    /* APLL calibration: IR_CAL_START with IR_CAL_RSTB sets OR_CAL_END */
    g_assert_cmphex(i2c_read(qts, HOST_APLL, SLAVE_APLL, 3) & 0x80, ==, 0);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 0, 0x0f);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 0, 0x3f);
    g_assert_cmphex(i2c_read(qts, HOST_APLL, SLAVE_APLL, 3) & 0x80, ==, 0x80);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 0, 0x0f);
    g_assert_cmphex(i2c_read(qts, HOST_APLL, SLAVE_APLL, 3) & 0x80, ==, 0);

    wr(qts, ANA_CONFIG, 1u << 17);
    g_assert_cmphex(i2c_read(qts, HOST_BBPLL, SLAVE_BBPLL, 11), ==, 0);
    i2c_write(qts, HOST_BBPLL, SLAVE_BBPLL, 11, 0xc3);
    wr(qts, ANA_CONFIG, 0);
    g_assert_cmphex(i2c_read(qts, HOST_BBPLL, SLAVE_BBPLL, 11), ==, 0x43);

    wr(qts, OPTIONS0, rd(qts, OPTIONS0) | BIAS_I2C_FORCE_PD);
    g_assert_cmphex(i2c_read(qts, HOST_APLL, SLAVE_APLL, 7), ==, 0);
    wr(qts, OPTIONS0, rd(qts, OPTIONS0) & ~BIAS_I2C_FORCE_PD);
    g_assert_cmphex(i2c_read(qts, HOST_APLL, SLAVE_APLL, 7), ==, 6);
    qtest_quit(qts);
}

/*
 * APLL_CLK = 40 MHz * (4 + SDM2 + SDM1 / 256 + SDM0 / 65536) / (2 (ODIV +
 * 2)), seen as APB_CLK = APLL_CLK / 8 with CPU_CLK on the APLL and
 * CPUPERIOD_SEL 0. The APLL is off after reset, which stops APB_CLK; so
 * does powering the BBPLL down with CPU_CLK on PLL_CLK.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_apll_frequency(void)
{
    QTestState *qts = start();
    uint32_t apb;

    wr(qts, TIMG0, T0CONFIG_RUN);
    wr(qts, DPORT_CPU_PER_CONF, 0);
    select_soc_clk(qts, 3);
    g_assert_cmpuint(apb_per_100us(qts), ==, 0);

    wr(qts, ANA_CONF, (rd(qts, ANA_CONF) & ~PLLA_FORCE_PD) | PLLA_FORCE_PU);
    g_assert_cmpuint(apb_per_100us(qts), ==, 500);          /* 40 MHz */
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 7, 6);
    g_assert_cmpuint(apb_per_100us(qts), ==, 1250);         /* 100 MHz */
    /* SDM1 counts only with the sigma-delta modulator out of reset */
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 8, 128);
    g_assert_cmpuint(apb_per_100us(qts), ==, 1250);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 5, 0x09);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 5, 0x49);
    apb = apb_per_100us(qts);                               /* 105 MHz */
    g_assert_cmpuint(apb, >=, 1311);
    g_assert_cmpuint(apb, <=, 1314);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 8, 0);
    i2c_write(qts, HOST_APLL, SLAVE_APLL, 4, 1);
    apb = apb_per_100us(qts);                               /* 66.7 MHz */
    g_assert_cmpuint(apb, >=, 832);
    g_assert_cmpuint(apb, <=, 835);

    wr(qts, ANA_CONF, rd(qts, ANA_CONF) | PLLA_FORCE_PD);
    g_assert_cmpuint(apb_per_100us(qts), ==, 0);

    select_soc_clk(qts, 1);
    g_assert_cmpuint(apb_per_100us(qts), ==, 8000);
    wr(qts, OPTIONS0, rd(qts, OPTIONS0) | BBPLL_FORCE_PD);
    g_assert_cmpuint(apb_per_100us(qts), ==, 0);
    wr(qts, OPTIONS0, rd(qts, OPTIONS0) & ~BBPLL_FORCE_PD);
    g_assert_cmpuint(apb_per_100us(qts), ==, 8000);
    qtest_quit(qts);
}

/* Start a one-shot conversion of ADC1 channel ch by the RTC controller */
static void adc1_start(QTestState *qts, unsigned ch)
{
    uint32_t v = EN_PAD_FORCE | EN_PAD(ch) | MEAS_START_FORCE;

    wr(qts, MEAS_START1, v);
    wr(qts, MEAS_START1, v | MEAS_START_SAR);
}

static uint32_t adc1_read(QTestState *qts, unsigned ch)
{
    adc1_start(qts, ch);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, MEAS_DONE);
    return rd(qts, MEAS_START1) & 0xffff;
}

static uint32_t adc2_read(QTestState *qts, unsigned ch)
{
    uint32_t v = EN_PAD_FORCE | EN_PAD(ch) | MEAS_START_FORCE;

    wr(qts, MEAS_START2, v);
    wr(qts, MEAS_START2, v | MEAS_START_SAR);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(rd(qts, MEAS_START2) & MEAS_DONE, ==, MEAS_DONE);
    return rd(qts, MEAS_START2) & 0xffff;
}

/*
 * RTC controller one-shot conversions: 1 V on ADC1 channel 6 at 11 dB
 * (3.9 V full scale) reads 1050 at 12 bits and 131 at 9 bits, after 44
 * RTC_FAST_CLK cycles (XTAL / 4) with the reset SAMPLE_CYCLE and CLK_DIV;
 * the raw output is inverted unless DATA_INV. A pad driven high reads as
 * the 3.3 V supply. ADC2 does not convert while PWDET has it.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_adc_oneshot(void)
{
    QTestState *qts = start();

    g_assert_cmphex(rd(qts, READ_CTRL), ==, 0x00070902);
    g_assert_cmphex(rd(qts, START_FORCE), ==, 0x0000000f);
    g_assert_cmphex(rd(qts, ATTEN1), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, SARDATE), ==, 0x01605180);

    set_prop(qts, "adc1-ch6-mv", 1000);
    wr(qts, READ_CTRL, rd(qts, READ_CTRL) | DATA_INV);
    adc1_start(qts, 6);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, 0);
    g_assert_cmpuint(MEAS_STATUS(rd(qts, SLAVE_ADDR1)), !=, 0);
    qtest_clock_step(qts, 4 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 400);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, MEAS_DONE);
    g_assert_cmpuint(MEAS_STATUS(rd(qts, SLAVE_ADDR1)), ==, 0);
    g_assert_cmpuint(rd(qts, MEAS_START1) & 0xffff, ==, 1050);
    /* The result is read-only */
    wr(qts, MEAS_START1, 0);
    g_assert_cmpuint(rd(qts, MEAS_START1) & 0xffff, ==, 1050);

    wr(qts, START_FORCE, rd(qts, START_FORCE) & ~3u);
    g_assert_cmpuint(adc1_read(qts, 6), ==, 131);
    wr(qts, START_FORCE, rd(qts, START_FORCE) | 3u);
    wr(qts, READ_CTRL, rd(qts, READ_CTRL) & ~DATA_INV);
    g_assert_cmpuint(adc1_read(qts, 6), ==, 4095 - 1050);

    /* ADC2 channel 0 is GPIO4, driven high from outside the chip */
    wr(qts, READ_CTRL2, rd(qts, READ_CTRL2) | DATA_INV2);
    g_assert_cmpuint(adc2_read(qts, 0), ==, 0);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 4, 1);
    g_assert_cmpuint(adc2_read(qts, 0), ==, 3465);

    /* With PWDET holding ADC2, a start converts nothing */
    wr(qts, READ_CTRL2, rd(qts, READ_CTRL2) | PWDET_FORCE2);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 4, 0);
    wr(qts, MEAS_START2, EN_PAD_FORCE | EN_PAD(0) | MEAS_START_FORCE);
    wr(qts, MEAS_START2, EN_PAD_FORCE | EN_PAD(0) | MEAS_START_FORCE |
       MEAS_START_SAR);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(rd(qts, MEAS_START2) & 0xffff, ==, 3465);
    wr(qts, READ_CTRL2, rd(qts, READ_CTRL2) & ~PWDET_FORCE2);
    g_assert_cmpuint(adc2_read(qts, 0), ==, 0);
    qtest_quit(qts);
}

/*
 * The Hall sensor adds its output to SENSOR_VP and takes it from
 * SENSOR_VN, the sign following HALL_PHASE: ESP-IDF's reading, (VP1 - VP0)
 * - (VN1 - VN0) at 0 dB, is four times the output.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_hall(void)
{
    QTestState *qts = start();
    int32_t vp[2], vn[2], value;

    set_prop(qts, "adc1-ch0-mv", 500);
    set_prop(qts, "adc1-ch3-mv", 500);
    set_prop(qts, "hall-field-ut", 10000);
    wr(qts, READ_CTRL, rd(qts, READ_CTRL) | DATA_INV);
    wr(qts, ATTEN1, 0xffffffff & ~3u & ~(3u << 6));
    wr(qts, TOUCH_CTRL1, rd(qts, TOUCH_CTRL1) | XPD_HALL_FORCE |
       HALL_PHASE_FORCE);

    g_assert_cmpuint(adc1_read(qts, 0), ==, adc1_read(qts, 3));
    for (int phase = 0; phase < 2; phase++) {
        wr(qts, RTCIO_HALL_SENS, XPD_HALL | (phase ? HALL_PHASE : 0));
        vp[phase] = adc1_read(qts, 0);
        vn[phase] = adc1_read(qts, 3);
    }
    value = (vp[1] - vp[0]) - (vn[1] - vn[0]);
    g_assert_cmpint(value, >=, 294);
    g_assert_cmpint(value, <=, 302);

    set_prop(qts, "hall-field-ut", -10000);
    for (int phase = 0; phase < 2; phase++) {
        wr(qts, RTCIO_HALL_SENS, XPD_HALL | (phase ? HALL_PHASE : 0));
        vp[phase] = adc1_read(qts, 0);
        vn[phase] = adc1_read(qts, 3);
    }
    g_assert_cmpint((vp[1] - vp[0]) - (vn[1] - vn[0]), ==, -value);
    qtest_quit(qts);
}

/*
 * DAC1 on GPIO25 from RTCIO's PDAC1_DAC, observed as a QOM property and by
 * ADC2 channel 8 on the same pad; then from the cosine generator at
 * RTC_FAST_CLK (8 MHz) * SW_FSTEP / 65536, offset binary (DAC_INV1 2).
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_dac(void)
{
    QTestState *qts = start();

    fast_clk_8m(qts);

    g_assert_cmphex(rd(qts, DAC_CTRL2), ==, 0x03000000);
    wr(qts, RTCIO_PAD_DAC1, PDAC_DAC(128));
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 0);
    wr(qts, RTCIO_PAD_DAC1, PDAC_DAC(128) | PDAC_XPD_DAC | PDAC_XPD_FORCE);
    wr(qts, DAC_CTRL2, 0);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);
    wr(qts, READ_CTRL2, rd(qts, READ_CTRL2) | DATA_INV2);
    g_assert_cmpuint(adc2_read(qts, 8), ==, 1739);

    wr(qts, DAC_CTRL2, CW_EN1 | INV1(2));
    wr(qts, DAC_CTRL1, SW_TONE_EN | 1);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 3300);
    qtest_clock_step(qts, 2048 * US);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);
    qtest_clock_step(qts, 2048 * US);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 12);
    /* Stopped, the generator holds its DC offset: code 0x80 */
    wr(qts, DAC_CTRL1, 1);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);
    qtest_quit(qts);
}

static void touch_sw_start(QTestState *qts)
{
    uint32_t v = rd(qts, TOUCH_CTRL2) | TOUCH_START_FORCE;

    wr(qts, TOUCH_CTRL2, v & ~TOUCH_START_EN);
    wr(qts, TOUCH_CTRL2, v | TOUCH_START_EN);
}

/*
 * A software-started touch measurement: TOUCH_XPD_WAIT (4) and
 * TOUCH_MEAS_DELAY (0x1000) cycles of the 8 MHz RTC_FAST_CLK. Pad T3 with
 * the reset slope (4 uA) and DREFH - DREFL (2.2 V) counts 46 cycles at
 * 10 pF and 93 at 5 pF; below its threshold it is touched, which raises
 * RTC_CNTL's TOUCH interrupt. Channel 9 measures pad T8.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_touch(void)
{
    QTestState *qts = start();

    fast_clk_8m(qts);

    g_assert_cmphex(rd(qts, TOUCH_CTRL1), ==, 0x02041000);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2), ==, 0x00400800);
    g_assert_cmphex(rd(qts, TOUCH_ENABLE), ==, 0x3fffffff);

    wr(qts, RTCIO_TOUCH_PAD(3), rd(qts, RTCIO_TOUCH_PAD(3)) | TOUCH_XPD);
    wr(qts, RTCIO_TOUCH_PAD(9), rd(qts, RTCIO_TOUCH_PAD(9)) | TOUCH_XPD);
    set_prop(qts, "touch8-ff", 5000);
    touch_sw_start(qts);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 512 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==,
                    TOUCH_MEAS_DONE);
    g_assert_cmpuint(rd(qts, TOUCH_OUT(3)) & 0xffff, ==, 46);
    g_assert_cmpuint(rd(qts, TOUCH_OUT(9)) & 0xffff, ==, 93);
    g_assert_cmpuint(rd(qts, TOUCH_OUT(8)) >> 16, ==, 0);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & 0x3ff, ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TOUCH, ==, 0);

    wr(qts, TOUCH_THRES(3), 60);
    touch_sw_start(qts);
    qtest_clock_step(qts, 600 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & 0x3ff, ==, 1u << 3);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TOUCH, ==, INT_TOUCH);
    wr(qts, INT_CLR, INT_TOUCH);
    wr(qts, TOUCH_CTRL2, rd(qts, TOUCH_CTRL2) | TOUCH_MEAS_EN_CLR);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & 0x3ff, ==, 0);

    /* A touch outside SET1 raises no interrupt */
    wr(qts, TOUCH_ENABLE, 0x3fffffff & ~(1u << 23));
    touch_sw_start(qts);
    qtest_clock_step(qts, 600 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & 0x3ff, ==, 1u << 3);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TOUCH, ==, 0);
    qtest_quit(qts);
}

/*
 * The touch FSM's timer: with RTC_CNTL_TOUCH_SLP_TIMER_EN and
 * TOUCH_START_FORCE clear, a measurement every TOUCH_SLEEP_CYCLES (0x100)
 * of the 150 kHz RTC_SLOW_CLK, 1.71 ms, plus the measurement's 0.51 ms.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_touch_timer(void)
{
    QTestState *qts = start();

    fast_clk_8m(qts);

    wr(qts, RTCIO_TOUCH_PAD(3), rd(qts, RTCIO_TOUCH_PAD(3)) | TOUCH_XPD);
    wr(qts, TOUCH_THRES(3), 60);
    wr(qts, STATE0, rd(qts, STATE0) | TOUCH_SLP_TIMER_EN);
    qtest_clock_step(qts, 2200 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 50 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==,
                    TOUCH_MEAS_DONE);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TOUCH, ==, INT_TOUCH);
    wr(qts, INT_CLR, INT_TOUCH);
    qtest_clock_step(qts, 2250 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TOUCH, ==, INT_TOUCH);

    wr(qts, STATE0, rd(qts, STATE0) & ~TOUCH_SLP_TIMER_EN);
    qtest_clock_step(qts, 600 * US);
    wr(qts, INT_CLR, INT_TOUCH);
    qtest_clock_step(qts, 5000 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TOUCH, ==, 0);
    qtest_quit(qts);
}

/*
 * The touch FSM counts RTC_FAST_CLK as it runs. At the reset XTAL / 4
 * (10 MHz) a measurement of 4 + 0x1000 cycles takes 410 us and pad T3
 * counts 37 rather than 8 MHz's 46, the pad oscillating on its own for a
 * shorter time. Switched to 8 MHz halfway, the rest runs at 8 MHz. Stopped
 * by powering RC_FAST_CLK down, the measurement holds until the
 * oscillator runs again, and the pad counts over the stretched time.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_touch_rtc_fast(void)
{
    QTestState *qts = start();

    wr(qts, RTCIO_TOUCH_PAD(3), rd(qts, RTCIO_TOUCH_PAD(3)) | TOUCH_XPD);

    /* 10 MHz: done after 410 us */
    touch_sw_start(qts);
    qtest_clock_step(qts, 409 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==,
                    TOUCH_MEAS_DONE);
    g_assert_cmpuint(rd(qts, TOUCH_OUT(3)) & 0xffff, ==, 37);

    /*
     * 2050 cycles at 10 MHz (205 us), then RC_FAST_CLK undivided: the
     * other 2050 take 256.25 us
     */
    wr(qts, CLK_CONF, rd(qts, CLK_CONF) & ~CK8M_DIV_SEL_MASK);
    touch_sw_start(qts);
    qtest_clock_step(qts, 205 * US);
    wr(qts, CLK_CONF, rd(qts, CLK_CONF) | FAST_CLK_RTC_SEL_8M);
    qtest_clock_step(qts, 255 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==,
                    TOUCH_MEAS_DONE);
    /* 460.85 us of measurement */
    g_assert_cmpuint(rd(qts, TOUCH_OUT(3)) & 0xffff, ==, 41);

    /* 8 MHz, stopped for 1 ms after 100 us */
    touch_sw_start(qts);
    qtest_clock_step(qts, 100 * US);
    rc_fast_power(qts, false);
    qtest_clock_step(qts, 1000 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    rc_fast_power(qts, true);
    qtest_clock_step(qts, CK8M_STARTUP_NS + 411 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==,
                    TOUCH_MEAS_DONE);
    g_assert_cmpuint(rd(qts, TOUCH_OUT(3)) & 0xffff, >, 46 * 2);
    qtest_quit(qts);
}

/*
 * The touch FSM's sleep counts RTC_SLOW_CLK: stopped (8MD256 with the
 * oscillator down), it holds, and goes on with the cycles left once
 * RTC_SLOW_CLK runs again.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_touch_slow_clk_stopped(void)
{
    QTestState *qts = start();
    uint32_t conf;

    fast_clk_8m(qts);
    wr(qts, RTCIO_TOUCH_PAD(3), rd(qts, RTCIO_TOUCH_PAD(3)) | TOUCH_XPD);
    wr(qts, TOUCH_THRES(3), 60);
    wr(qts, STATE0, rd(qts, STATE0) | TOUCH_SLP_TIMER_EN);
    /* 150 of the sleep's 256 cycles */
    qtest_clock_step(qts, 1000 * US);
    conf = rd(qts, CLK_CONF);
    wr(qts, CLK_CONF, (conf & ~ANA_CLK_RTC_SEL_MASK) |
       ANA_CLK_RTC_SEL_8MD256 | ENB_CK8M);
    qtest_clock_step(qts, 5000 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    /* RC_SLOW_CLK again: 106 cycles (707 us) of sleep, then 512.5 us */
    wr(qts, CLK_CONF, conf);
    qtest_clock_step(qts, 1150 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 150 * US);
    g_assert_cmphex(rd(qts, TOUCH_CTRL2) & TOUCH_MEAS_DONE, ==,
                    TOUCH_MEAS_DONE);
    qtest_quit(qts);
}

/*
 * The cosine generator steps its phase once per RTC_FAST_CLK cycle: a
 * quarter period of SW_FSTEP 1 is 1.6384 ms at the reset 10 MHz and
 * 2.048 ms at 8 MHz. With RTC_FAST_CLK stopped the phase holds.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_dac_rtc_fast(void)
{
    QTestState *qts = start();

    wr(qts, RTCIO_PAD_DAC1, PDAC_XPD_DAC | PDAC_XPD_FORCE);
    wr(qts, DAC_CTRL2, CW_EN1 | INV1(2));
    wr(qts, DAC_CTRL1, SW_TONE_EN | 1);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 3300);
    /* 10 MHz: phase 16384 */
    qtest_clock_step(qts, 1638400);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);

    /* RTC_FAST_CLK from the oscillator, powered down: stopped */
    rc_fast_power(qts, false);
    wr(qts, CLK_CONF, (rd(qts, CLK_CONF) & ~CK8M_DIV_SEL_MASK) |
       FAST_CLK_RTC_SEL_8M);
    qtest_clock_step(qts, 3000 * US);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);

    /* Back on XTAL / 4: phase 32768 */
    wr(qts, CLK_CONF, rd(qts, CLK_CONF) & ~FAST_CLK_RTC_SEL_8M);
    qtest_clock_step(qts, 1638400);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 12);

    /*
     * The oscillator starts up while XTAL / 4 runs the generator (phase
     * 49152), then runs it at 8 MHz (phase 0)
     */
    rc_fast_power(qts, true);
    qtest_clock_step(qts, 1638400);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);
    wr(qts, CLK_CONF, rd(qts, CLK_CONF) | FAST_CLK_RTC_SEL_8M);
    qtest_clock_step(qts, 2048 * US);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 3300);
    qtest_quit(qts);
}

/*
 * An RTC controller conversion counts RTC_FAST_CLK: 44 cycles at 8 MHz
 * are 5.5 us, but with the oscillator powered down the conversion waits
 * for it to start up.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_adc_rtc_fast_stopped(void)
{
    QTestState *qts = start();

    fast_clk_8m(qts);
    set_prop(qts, "adc1-ch6-mv", 1000);
    wr(qts, READ_CTRL, rd(qts, READ_CTRL) | DATA_INV);
    adc1_start(qts, 6);
    qtest_clock_step(qts, 5 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, MEAS_DONE);

    rc_fast_power(qts, false);
    adc1_start(qts, 6);
    qtest_clock_step(qts, 1000 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, 0);
    rc_fast_power(qts, true);
    qtest_clock_step(qts, CK8M_STARTUP_NS + 5 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(rd(qts, MEAS_START1) & MEAS_DONE, ==, MEAS_DONE);
    g_assert_cmpuint(rd(qts, MEAS_START1) & 0xffff, ==, 1050);
    qtest_quit(qts);
}

/*
 * I2S0's ADC mode: each WS cycle of the LCD master receiver scans the next
 * entry of ADC1's pattern table; with RX_FIFO_MOD 1 two 16-bit DMA words
 * share a FIFO word. Type I words carry the channel and 12 bits; type II
 * (DATA_SAR_SEL) the unit, the channel and 11 bits.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_dig_adc(void)
{
    QTestState *qts = start();

    set_prop(qts, "adc1-ch6-mv", 1000);
    set_prop(qts, "adc1-ch0-mv", 3900);
    wr(qts, READ_CTRL, rd(qts, READ_CTRL) | DIG_FORCE1);
    /* Entries: channel 6 then channel 0, 12 bits, 11 dB */
    wr(qts, SAR1_PATT_TAB1, 0x6f0f0000);
    wr(qts, SARADC_CTRL2, rd(qts, SARADC_CTRL2) | SAR1_INV);
    wr(qts, SARADC_CTRL, (SARADC_CTRL_RESET & ~(15u << 15)) |
       SAR1_PATT_LEN(2) | DATA_TO_I2S);

    wr(qts, DPORT_PERIP_CLK_EN, rd(qts, DPORT_PERIP_CLK_EN) | PERIP_I2S0);
    wr(qts, I2S_CLKM_CONF, 10);
    wr(qts, I2S_SAMPLE_RATE_CONF, 10 | (10 << 6) | (16 << 12) | (16 << 18));
    wr(qts, I2S_FIFO_CONF, I2S_FIFO_FORCE | I2S_RX_FIFO_MOD(1));
    wr(qts, I2S_CONF_CHAN, 1 << 3);
    wr(qts, I2S_CONF2, I2S_LCD_EN);
    wr(qts, I2S_CONF, rd(qts, I2S_CONF) | I2S_RX_START);
    qtest_clock_step(qts, 4 * I2S_SAMPLE_PERIOD_NS);
    g_assert_cmphex(rd(qts, I2S_FIFO_RD), ==, 0x641a0fff);
    g_assert_cmphex(rd(qts, I2S_FIFO_RD), ==, 0x641a0fff);

    /* Clearing the pointer restarts the table; type II words */
    wr(qts, I2S_CONF, rd(qts, I2S_CONF) & ~I2S_RX_START);
    wr(qts, SARADC_CTRL, rd(qts, SARADC_CTRL) | DATA_SAR_SEL |
       SAR1_PATT_P_CLEAR);
    wr(qts, SARADC_CTRL, rd(qts, SARADC_CTRL) & ~SAR1_PATT_P_CLEAR);
    wr(qts, I2S_CONF, rd(qts, I2S_CONF) | I2S_RX_START);
    qtest_clock_step(qts, 2 * I2S_SAMPLE_PERIOD_NS);
    g_assert_cmphex(rd(qts, I2S_FIFO_RD), ==, 0x320d07ff);
    qtest_quit(qts);
}

/*
 * I2S0's DAC mode: with DAC_DIG_FORCE the LCD master transmitter's data
 * feeds the DACs, the right channel's top 8 bits DAC1 and the left's
 * DAC2. Out of reset the transmitter sends the right channel first, the
 * first 16-bit half of each FIFO word.
 */
/* [spec:nuos:req:emu.esp32.analog/test] */
static void test_dac_dma(void)
{
    QTestState *qts = start();

    wr(qts, RTCIO_PAD_DAC1, PDAC_XPD_DAC | PDAC_XPD_FORCE);
    wr(qts, RTCIO_PAD_DAC2, PDAC_XPD_DAC | PDAC_XPD_FORCE);
    wr(qts, DAC_CTRL1, DAC_DIG_FORCE);
    wr(qts, DPORT_PERIP_CLK_EN, rd(qts, DPORT_PERIP_CLK_EN) | PERIP_I2S0);
    wr(qts, I2S_CLKM_CONF, 10);
    wr(qts, I2S_SAMPLE_RATE_CONF, 10 | (10 << 6) | (16 << 12) | (16 << 18));
    wr(qts, I2S_FIFO_CONF, I2S_FIFO_FORCE | I2S_TX_FIFO_MOD(1));
    wr(qts, I2S_CONF_CHAN, 0);
    wr(qts, I2S_CONF2, I2S_LCD_EN);
    wr(qts, I2S_FIFO_WR, 0x80004000);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 0);
    wr(qts, I2S_CONF, rd(qts, I2S_CONF) | I2S_TX_START);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 1656);
    g_assert_cmpint(get_prop(qts, "dac2-mv"), ==, 828);

    /* Without DAC_DIG_FORCE the DACs go back to their other sources */
    wr(qts, DAC_CTRL1, 0);
    wr(qts, DAC_CTRL2, 0);
    wr(qts, RTCIO_PAD_DAC1, PDAC_DAC(255) | PDAC_XPD_DAC | PDAC_XPD_FORCE);
    g_assert_cmpint(get_prop(qts, "dac1-mv"), ==, 3300);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32/analog/i2c", test_analog_i2c);
    qtest_add_func("esp32/analog/apll-frequency", test_apll_frequency);
    qtest_add_func("esp32/analog/adc-oneshot", test_adc_oneshot);
    qtest_add_func("esp32/analog/hall", test_hall);
    qtest_add_func("esp32/analog/dac", test_dac);
    qtest_add_func("esp32/analog/touch", test_touch);
    qtest_add_func("esp32/analog/touch-timer", test_touch_timer);
    qtest_add_func("esp32/analog/touch-rtc-fast", test_touch_rtc_fast);
    qtest_add_func("esp32/analog/touch-slow-clk-stopped",
                   test_touch_slow_clk_stopped);
    qtest_add_func("esp32/analog/dac-rtc-fast", test_dac_rtc_fast);
    qtest_add_func("esp32/analog/adc-rtc-fast-stopped",
                   test_adc_rtc_fast_stopped);
    qtest_add_func("esp32/analog/dig-adc", test_dig_adc);
    qtest_add_func("esp32/analog/dac-dma", test_dac_dma);

    return g_test_run();
}
