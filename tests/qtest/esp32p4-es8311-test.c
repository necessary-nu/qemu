/*
 * QTest testcase for the ESP32-P4 board's audio: the ES8311 codec on the
 * ESP_I2C wires (GPIO7 SDA, GPIO8 SCL) and the I2S pads GPIO9 to GPIO13,
 * its NS4150B amplifier on GPIO53, and the speaker and microphone as the
 * machine's audiodev
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * I2C0 drives the codec's control port at 400 kHz from the 40 MHz
 * crystal, through the board's 2.2 kOhm pull-ups. I2S0's units run from
 * the crystal / 5, 8 MHz, which is also its MCLK out; BCK is that / 8, 1
 * MHz, and a frame two 16-bit slots: LRCK 31250 Hz, MCLK 256 times LRCK.
 * GDMA-AHB moves the data through descriptors in L2MEM past the L2 cache.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"
#include "esp32p4-board-codec.h"

#define MEM                 0x4ff40000

/* I2C0 */
#define I2C0                0x500c4000
#define SCL_LOW_PERIOD      0x00
#define CTR                 0x04
#define SR                  0x08
#define FIFO_CONF           0x18
#define DATA                0x1c
#define INT_RAW             0x20
#define INT_CLR             0x24
#define SDA_HOLD            0x30
#define SDA_SAMPLE          0x34
#define SCL_HIGH_PERIOD     0x38
#define SCL_START_HOLD      0x40
#define SCL_RSTART_SETUP    0x44
#define SCL_STOP_HOLD       0x48
#define SCL_STOP_SETUP      0x4c
#define COMD(n)             (0x58 + 4 * (n))
#define CTR_RX_FULL_ACK     (1u << 3)
#define CTR_MS_MODE         (1u << 4)
#define CTR_TRANS_START     (1u << 5)
#define CTR_CONF_UPGATE     (1u << 11)
#define FIFO_RX_RST         (1u << 12)
#define FIFO_TX_RST         (1u << 13)
#define INT_TRANS_COMPLETE  (1u << 7)
#define INT_NACK            (1u << 10)
#define SR_RXFIFO_CNT(v)    (((v) >> 8) & 0x3f)
#define CMD_WRITE           1
#define CMD_STOP            2
#define CMD_READ            3
#define CMD_RSTART          6
#define CMD(op, n, chk, exp, ack) \
    ((n) | ((chk) << 8) | ((exp) << 9) | ((ack) << 10) | ((op) << 11))

/* I2S0 and the LP I2S */
#define I2S0                0x500c6000
#define LP_I2S              0x50125000
#define RX_CONF             0x20
#define TX_CONF             0x24
#define RX_CONF1            0x28
#define TX_CONF1            0x2c
#define RX_TDM_CTRL         0x50
#define TX_TDM_CTRL         0x54
#define RXEOF_NUM           0x64
#define C_RESET             (1u << 0)
#define C_FIFO_RESET        (1u << 1)
#define C_START             (1u << 2)
#define C_SLAVE             (1u << 3)
#define C_UPDATE            (1u << 8)
#define C_TDM_EN            (1u << 19)
#define C_BCK_DIV(m)        ((uint32_t)((m) - 1) << 21)
#define C_SIG_LOOPBACK      (1u << 30)
#define CONF1(ws, bits, half, chan) \
    ((uint32_t)((ws) - 1) | (uint32_t)((bits) - 1) << 14 | \
     (uint32_t)((half) - 1) << 19 | (uint32_t)((chan) - 1) << 27)
#define TDM(tot, mask)      ((uint32_t)((tot) - 1) << 16 | (mask))
/* CONF's reset values, but for BCK_DIV_NUM */
#define TX_BASE             0x0000f210u
#define RX_BASE             0x0000b600u
/* Philips: WS low for the left channel, 16 bits, one cycle late */
#define STD_CONF1           CONF1(16, 16, 16, 16)
#define STD_TDM             TDM(2, 0x3)
#define LP_VAD_CONF         0x00
#define LP_VAD_RESULT       0x04
#define LP_RX_MEM_CONF      0x08
#define LP_INT_RAW          0x0c
#define LP_RX_CONF          0x20
#define LP_RX_CONF1         0x28
#define LP_RX_TDM_CTRL      0x50
#define LP_VAD_PARAM(n)     (0x80 + 4 * (n))
#define LP_RAM              0xc00
#define LP_INT_VAD_DONE     (1u << 3)
#define LP_CONF1(ws, div, bits, half, chan, shift) \
    ((uint32_t)((ws) - 1) | (uint32_t)((div) - 1) << 7 | \
     (uint32_t)((bits) - 1) << 13 | (uint32_t)((half) - 1) << 18 | \
     (uint32_t)((chan) - 1) << 24 | (shift ? 1u << 29 : 0))

/* Clocks */
#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL2       (HP_CLKRST + 0x1c)
#define PERI_CLK_CTRL10     (HP_CLKRST + 0x40)
#define HP_RST_EN1          (HP_CLKRST + 0xc4)
#define I2C0_APB_CLK_EN     (1u << 12)
#define I2C0_CLK_EN         (1u << 1)
#define RST_EN_I2C0         (1u << 22)
#define I2S0_APB_CLK_EN     (1u << 14)
#define LPPERI              0x50120000
#define LPPERI_CLK_EN       (LPPERI + 0x00)
#define LPPERI_CORE_CLK_SEL (LPPERI + 0x04)
#define LPPERI_I2S_RX_NUM   (LPPERI + 0x30)

/* GDMA-AHB */
#define AHB                 0x50085000
#define AHB_IN_INT(ch)      (AHB + 0x10 * (ch))
#define AHB_OUT_INT(ch)     (AHB + 0x30 + 0x10 * (ch))
#define AHB_IN(ch)          (AHB + 0x70 + 0xc0 * (ch))
#define AHB_OUT(ch)         (AHB + 0xd0 + 0xc0 * (ch))
#define AHB_IN_LINK_ADDR(ch)  (AHB + 0x3ac + 4 * (ch))
#define AHB_OUT_LINK_ADDR(ch) (AHB + 0x3b8 + 4 * (ch))
#define G_CONF0             0x00
#define G_LINK              0x10
#define G_PERI_SEL          0x30
#define G_CLR               0xc
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define CONF0_RST           (1u << 0)
#define DW0(size, len)      ((size) | (uint32_t)(len) << 12)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)
#define PERI_I2S0           3

/* GPIO matrix, IO MUX and LP IO */
#define GPIO                0x500e0000
#define GPIO_OUT1_W1TS      (GPIO + 0x014)
#define GPIO_OUT1_W1TC      (GPIO + 0x018)
#define GPIO_ENABLE1_W1TS   (GPIO + 0x030)
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_PIN(n)         (GPIO + 0x074 + 4 * (n))
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define PIN_PAD_DRIVER      (1u << 2)
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_WPU             (1u << 8)
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((d) << 10)
#define MCU_SEL_GPIO        (1u << 12)
#define SIG_GPIO_OUT        256
#define LP_GPIO_FUNC_IN(s)  (0x5012a074 + 4 * (s))
#define LP_GPIO_FUNC_OUT(n) (0x5012a0f4 + 4 * (n))
#define LP_IN_SIG_IN_SEL    (1u << 1)
#define LP_IN_SEL(p)        ((p) << 2)
#define LP_OUT_SEL(s)       ((s) << 3)
#define LP_IOMUX_PAD(n)     (0x5012b008 + 4 * (n))
#define LP_PAD_MUX_SEL      (1u << 4)
#define LP_PAD_FUN_SEL(f)   ((f) << 5)
#define LP_PAD_FUN_IE       (1u << 10)

/* Matrix signals */
#define SIG_I2C0_SCL        68
#define SIG_I2C0_SDA        69
#define SIG_O_BCK(n)        (25 + 6 * (n))
#define SIG_MCLK(n)         (26 + 6 * (n))
#define SIG_O_WS(n)         (27 + 6 * (n))
#define SIG_SD(n)           (28 + 6 * (n))
#define SIG_I_BCK(n)        (29 + 6 * (n))
#define SIG_I_WS(n)         (30 + 6 * (n))
#define LP_SIG_I_BCK        9
#define LP_SIG_I_SD         10
#define LP_SIG_I_WS         11

/* The board */
#define PAD_SDA             7
#define PAD_SCL             8
#define PAD_DSDIN           9
#define PAD_LRCK            10
#define PAD_ASDOUT          11
#define PAD_SCLK            12
#define PAD_MCLK            13
#define PAD_PA_CTRL         53
#define CODEC               0x18

/* ES8311 registers */
#define ES_RESET            0x00
#define ES_CLK1             0x01
#define ES_CLK2             0x02
#define ES_CLK3             0x03
#define ES_CLK4             0x04
#define ES_CLK5             0x05
#define ES_CLK6             0x06
#define ES_CLK7             0x07
#define ES_CLK8             0x08
#define ES_SDPIN            0x09
#define ES_SDPOUT           0x0a
#define ES_SYS0B            0x0b
#define ES_SYS0C            0x0c
#define ES_SYS0D            0x0d
#define ES_SYS0E            0x0e
#define ES_SYS10            0x10
#define ES_SYS11            0x11
#define ES_SYS12            0x12
#define ES_SYS13            0x13
#define ES_SYS14            0x14
#define ES_ADC15            0x15
#define ES_ADC16            0x16
#define ES_ADC17            0x17
#define ES_ADC1B            0x1b
#define ES_ADC1C            0x1c
#define ES_DAC31            0x31
#define ES_DAC32            0x32
#define ES_DAC37            0x37
#define ES_GPIO44           0x44
#define ES_GP45             0x45
#define ES_I2C              0xfa
#define ES_FLAG             0xfc

/* The codec's sample rate here: MCLK 8 MHz / 256 */
#define RATE                31250
#define FRAME_NS            32000

static QTestState *start(const char *extra)
{
    return qtest_initf("-M esp32p4,strap-mode=0 %s", extra ? extra : "");
}

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t off)
{
    return qtest_readl(qts, base + off);
}

static void wr(QTestState *qts, uint32_t base, uint32_t off, uint32_t v)
{
    qtest_writel(qts, base + off, v);
}

/* ---- I2C0 on the board's ESP_I2C wires ---- */

/* An open-drain pad with its input enabled, the board's pull-up on it */
static void od_pad(QTestState *qts, int pad)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    qtest_writel(qts, GPIO_PIN(pad), PIN_PAD_DRIVER);
}

/* I2C0 a master at 400 kHz, SCL and SDA on @scl_pad and @sda_pad. */
static void i2c_setup(QTestState *qts, int scl_pad, int sda_pad)
{
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 I2C0_APB_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL10, qtest_readl(qts, PERI_CLK_CTRL10) |
                 I2C0_CLK_EN);
    qtest_writel(qts, HP_RST_EN1, qtest_readl(qts, HP_RST_EN1) &
                 ~RST_EN_I2C0);
    od_pad(qts, scl_pad);
    od_pad(qts, sda_pad);
    qtest_writel(qts, GPIO_FUNC_OUT(scl_pad), SIG_I2C0_SCL);
    qtest_writel(qts, GPIO_FUNC_OUT(sda_pad), SIG_I2C0_SDA);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SCL), IN_SIG_IN_SEL | scl_pad);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SDA), IN_SIG_IN_SEL | sda_pad);
    wr(qts, I2C0, SCL_LOW_PERIOD, 39);
    wr(qts, I2C0, SCL_HIGH_PERIOD, 29 | (29 << 9));
    wr(qts, I2C0, SDA_HOLD, 9);
    wr(qts, I2C0, SDA_SAMPLE, 34);
    wr(qts, I2C0, SCL_START_HOLD, 19);
    wr(qts, I2C0, SCL_RSTART_SETUP, 19);
    wr(qts, I2C0, SCL_STOP_HOLD, 19);
    wr(qts, I2C0, SCL_STOP_SETUP, 19);
    wr(qts, I2C0, CTR, CTR_MS_MODE | CTR_RX_FULL_ACK);
    wr(qts, I2C0, CTR, CTR_MS_MODE | CTR_RX_FULL_ACK | CTR_CONF_UPGATE);
}

static void i2c_board(QTestState *qts)
{
    i2c_setup(qts, PAD_SCL, PAD_SDA);
}

static void i2c_run(QTestState *qts)
{
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_TRANS_START);
    for (int i = 0; i < 1000; i++) {
        if (rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE) {
            return;
        }
        qtest_clock_step(qts, 10000);
    }
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, !=, 0);
}

static void i2c_fifo_reset(QTestState *qts)
{
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) | FIFO_RX_RST |
       FIFO_TX_RST);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) &
       ~(FIFO_RX_RST | FIFO_TX_RST));
    wr(qts, I2C0, INT_CLR, 0x7ffff);
}

/* Write @n bytes to @addr; whether every byte was acknowledged. */
static bool i2c_write(QTestState *qts, uint8_t addr, const uint8_t *buf,
                      int n)
{
    i2c_fifo_reset(qts);
    wr(qts, I2C0, DATA, addr << 1);
    for (int i = 0; i < n; i++) {
        wr(qts, I2C0, DATA, buf[i]);
    }
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, n + 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    i2c_run(qts);
    return !(rd(qts, I2C0, INT_RAW) & INT_NACK);
}

/* Read @n bytes from register @reg of @addr, as ESP-IDF's driver does. */
static bool i2c_read(QTestState *qts, uint8_t addr, uint8_t reg,
                     uint8_t *buf, int n)
{
    i2c_fifo_reset(qts);
    wr(qts, I2C0, DATA, addr << 1);
    wr(qts, I2C0, DATA, reg);
    wr(qts, I2C0, DATA, (addr << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_WRITE, 1, 1, 0, 0));
    if (n > 1) {
        wr(qts, I2C0, COMD(4), CMD(CMD_READ, n - 1, 0, 0, 0));
        wr(qts, I2C0, COMD(5), CMD(CMD_READ, 1, 0, 0, 1));
        wr(qts, I2C0, COMD(6), CMD(CMD_STOP, 0, 0, 0, 0));
    } else {
        wr(qts, I2C0, COMD(4), CMD(CMD_READ, 1, 0, 0, 1));
        wr(qts, I2C0, COMD(5), CMD(CMD_STOP, 0, 0, 0, 0));
    }
    i2c_run(qts);
    if (rd(qts, I2C0, INT_RAW) & INT_NACK) {
        return false;
    }
    g_assert_cmpuint(SR_RXFIFO_CNT(rd(qts, I2C0, SR)), ==, n);
    for (int i = 0; i < n; i++) {
        buf[i] = rd(qts, I2C0, DATA);
    }
    return true;
}

static void es_w(QTestState *qts, uint8_t reg, uint8_t v)
{
    uint8_t b[2] = { reg, v };

    g_assert_true(i2c_write(qts, CODEC, b, 2));
}

static uint8_t es_r(QTestState *qts, uint8_t reg)
{
    uint8_t v;

    g_assert_true(i2c_read(qts, CODEC, reg, &v, 1));
    return v;
}

/*
 * The codec in slave mode, I2S format both ways at @wl_reg's word length,
 * clocked from MCLK 256 times LRCK, both converters on, as Espressif's
 * esp_codec_dev es8311 driver opens, configures and starts it, with the
 * ADC's ramp off: DAC volume @vol, ADCDAT_SEL @adcdat, PGA and ADC scale
 * at 0 dB.
 */
static void es_setup(QTestState *qts, uint8_t vol, uint8_t adcdat)
{
    /* es8311_open() */
    es_w(qts, ES_SYS0D, 0xfa);
    es_w(qts, ES_GPIO44, 0x08);
    es_w(qts, ES_GPIO44, 0x08);
    es_w(qts, ES_CLK1, 0x30);
    es_w(qts, ES_CLK2, 0x00);
    es_w(qts, ES_CLK3, 0x10);
    es_w(qts, ES_ADC16, 0x24);
    es_w(qts, ES_CLK4, 0x10);
    es_w(qts, ES_CLK5, 0x00);
    es_w(qts, ES_SYS0B, 0x00);
    es_w(qts, ES_SYS0C, 0x00);
    es_w(qts, ES_SYS10, 0x1f);
    es_w(qts, ES_SYS11, 0x7f);
    es_w(qts, ES_RESET, 0x80);
    es_w(qts, ES_RESET, es_r(qts, ES_RESET) & 0xbf);
    es_w(qts, ES_CLK1, 0x3f);
    es_w(qts, ES_CLK6, es_r(qts, ES_CLK6) & ~0x20);
    es_w(qts, ES_SYS13, 0x10);
    es_w(qts, ES_ADC1B, 0x0a);
    es_w(qts, ES_ADC1C, 0x6a);
    es_w(qts, ES_GPIO44, 0x08 | adcdat << 4);
    /* es8311_set_fs(): 16 bits, I2S; the clocks for 256 fs */
    es_w(qts, ES_SDPIN, (es_r(qts, ES_SDPIN) | 0x0c) & 0xfc);
    es_w(qts, ES_SDPOUT, (es_r(qts, ES_SDPOUT) | 0x0c) & 0xfc);
    es_w(qts, ES_CLK2, es_r(qts, ES_CLK2) & 0x07);
    es_w(qts, ES_CLK5, 0x00);
    es_w(qts, ES_CLK3, 0x10);
    es_w(qts, ES_CLK4, 0x10);
    /* es8311_start() */
    es_w(qts, ES_RESET, 0x80);
    es_w(qts, ES_CLK1, 0x3f);
    es_w(qts, ES_SDPIN, es_r(qts, ES_SDPIN) & 0xbf);
    es_w(qts, ES_SDPOUT, es_r(qts, ES_SDPOUT) & 0xbf);
    es_w(qts, ES_ADC17, 0xbf);
    es_w(qts, ES_SYS0E, 0x02);
    es_w(qts, ES_SYS12, 0x00);
    es_w(qts, ES_SYS14, 0x10);
    es_w(qts, ES_SYS0D, 0x01);
    es_w(qts, ES_ADC15, 0x00);
    es_w(qts, ES_DAC37, 0x08);
    es_w(qts, ES_GP45, 0x00);
    es_w(qts, ES_ADC16, 0x20);
    es_w(qts, ES_DAC32, vol);
}

/* ---- I2S0 on the codec's pads ---- */

static void i2s_w(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, I2S0 + reg, v);
}

/* A field of HP_SYS_CLKRST */
typedef struct Fld {
    uint32_t reg;
    int shift;
    int width;
} Fld;

#define F(r, s, w)  { HP_CLKRST + (r), (s), (w) }

static void set_fld(QTestState *qts, Fld f, uint32_t v)
{
    uint32_t mask = ((1u << f.width) - 1) << f.shift;
    uint32_t r = qtest_readl(qts, f.reg);

    qtest_writel(qts, f.reg, (r & ~mask) | ((v << f.shift) & mask));
}

/*
 * I2S0's bus clock, and both its units on the crystal / 5 (8 MHz): the
 * enable, source, N, and the fraction's X, Y, Z and YN1 of
 * HP_SYS_CLKRST_PERI_CLK_CTRL13 to 15, RX then TX.
 */
static void i2s_clocks(QTestState *qts)
{
    static const Fld unit[2][7] = {
        { F(0x44, 24, 1), F(0x44, 25, 2), F(0x48, 0, 8), F(0x48, 8, 9),
          F(0x48, 17, 9), F(0x4c, 0, 9), F(0x4c, 9, 1) },
        { F(0x4c, 10, 1), F(0x4c, 11, 2), F(0x4c, 13, 8), F(0x4c, 21, 9),
          F(0x50, 0, 9), F(0x50, 9, 9), F(0x50, 18, 1) },
    };

    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 I2S0_APB_CLK_EN);
    for (int t = 0; t < 2; t++) {
        set_fld(qts, unit[t][1], 0);
        set_fld(qts, unit[t][3], 0);
        set_fld(qts, unit[t][5], 0);
        set_fld(qts, unit[t][4], 1);
        set_fld(qts, unit[t][6], 0);
        set_fld(qts, unit[t][2], 5);
        set_fld(qts, unit[t][0], 1);
    }
}

static void pad_out(QTestState *qts, int pad, int sig)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(pad), sig);
}

static void pad_in(QTestState *qts, int pad, int sig)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_IN(sig), IN_SIG_IN_SEL | pad);
}

/*
 * I2S0's pins as the board wires them to the codec (ESP-IDF's
 * i2s_std_gpio_config_t for the board): MCLK GPIO13, BCK GPIO12, WS
 * GPIO10, DOUT GPIO9, DIN GPIO11.
 */
static void i2s_pads(QTestState *qts, int mclk, int bck, int ws, int dout,
                     int din)
{
    pad_out(qts, mclk, SIG_MCLK(0));
    pad_out(qts, bck, SIG_O_BCK(0));
    pad_out(qts, ws, SIG_O_WS(0));
    pad_out(qts, dout, SIG_SD(0));
    pad_in(qts, din, SIG_SD(0));
}

static void i2s_board_pads(QTestState *qts)
{
    i2s_pads(qts, PAD_MCLK, PAD_SCLK, PAD_LRCK, PAD_DSDIN, PAD_ASDOUT);
}

static void unit_setup(QTestState *qts, bool tx, uint32_t conf)
{
    i2s_w(qts, tx ? TX_CONF1 : RX_CONF1, STD_CONF1);
    i2s_w(qts, tx ? TX_TDM_CTRL : RX_TDM_CTRL, STD_TDM);
    i2s_w(qts, tx ? TX_CONF : RX_CONF, conf | C_RESET | C_FIFO_RESET);
    i2s_w(qts, tx ? TX_CONF : RX_CONF, conf | C_UPDATE);
}

/* Descriptors of up to 4092 bytes each, linked, from @desc on. */
static void dma_chain(QTestState *qts, uint32_t desc, uint32_t buf,
                      uint32_t len, bool tx)
{
    while (len) {
        uint32_t n = MIN(len, 4092);

        len -= n;
        qtest_writel(qts, desc, (tx ? DW0(n, n) | (len ? 0 : SUC_EOF)
                                    : DW0(n, 0)) | OWNER);
        qtest_writel(qts, desc + 4, buf);
        qtest_writel(qts, desc + 8, len ? desc + 12 : 0);
        desc += 12;
        buf += n;
    }
}

static void dma_tx(QTestState *qts, int ch, uint32_t desc, uint32_t buf,
                   uint32_t len)
{
    qtest_writel(qts, AHB_OUT(ch) + G_CONF0, CONF0_RST);
    qtest_writel(qts, AHB_OUT(ch) + G_CONF0, 0);
    qtest_writel(qts, AHB_OUT_INT(ch) + G_CLR, 0x7f);
    qtest_writel(qts, AHB_OUT(ch) + G_PERI_SEL, PERI_I2S0);
    dma_chain(qts, desc, buf, len, true);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(ch), desc);
    qtest_writel(qts, AHB_OUT(ch) + G_LINK, OUT_LINK_START);
}

static void dma_rx(QTestState *qts, int ch, uint32_t desc, uint32_t buf,
                   uint32_t size)
{
    qtest_writel(qts, AHB_IN(ch) + G_CONF0, CONF0_RST);
    qtest_writel(qts, AHB_IN(ch) + G_CONF0, 0);
    qtest_writel(qts, AHB_IN_INT(ch) + G_CLR, 0xff);
    qtest_writel(qts, AHB_IN(ch) + G_PERI_SEL, PERI_I2S0);
    dma_chain(qts, desc, buf, size, false);
    qtest_writel(qts, AHB_IN_LINK_ADDR(ch), desc);
    qtest_writel(qts, AHB_IN(ch) + G_LINK, IN_LINK_START);
}

/*
 * I2S0 full duplex as ESP-IDF sets it up: the transmitter the master,
 * the receiver on its clock (SIG_LOOPBACK), BCK = 8 MHz / 8; @n frames of
 * @tx out (16-bit stereo), as many into @rx_buf.
 */
static void i2s_duplex(QTestState *qts, const int16_t *tx, int n,
                       uint32_t rx_buf)
{
    uint32_t txc = TX_BASE | C_TDM_EN | C_BCK_DIV(8) | C_SIG_LOOPBACK;
    uint32_t rxc = RX_BASE | C_TDM_EN | C_BCK_DIV(8);

    qtest_memwrite(qts, MEM + 0x1000, tx, 4 * n);
    i2s_w(qts, RXEOF_NUM, 2 * n - 1);
    unit_setup(qts, false, rxc);
    unit_setup(qts, true, txc);
    dma_rx(qts, 1, MEM, rx_buf, 4 * n);
    dma_tx(qts, 0, MEM + 0x100, MEM + 0x1000, 4 * n);
    qtest_clock_step(qts, 10000);
    i2s_w(qts, RX_CONF, rxc | C_START);
    i2s_w(qts, TX_CONF, txc | C_START);
}

static void i2s_stop(QTestState *qts)
{
    i2s_w(qts, TX_CONF, TX_BASE | C_TDM_EN | C_BCK_DIV(8));
    i2s_w(qts, RX_CONF, RX_BASE | C_TDM_EN | C_BCK_DIV(8));
}

/* GPIO53, PA_CTRL, driven at @level. */
static void pa(QTestState *qts, bool level)
{
    pad_out(qts, PAD_PA_CTRL, SIG_GPIO_OUT);
    qtest_writel(qts, level ? GPIO_OUT1_W1TS : GPIO_OUT1_W1TC,
                 1u << (PAD_PA_CTRL - 32));
    qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << (PAD_PA_CTRL - 32));
}

/* ---- WAV files ---- */

static char *tmp_wav(void)
{
    char *path;
    int fd = g_file_open_tmp("esp32p4-es8311-XXXXXX.wav", &path, NULL);

    g_assert(fd >= 0);
    close(fd);
    return path;
}

/* A mono 16-bit WAV file of @n samples at @rate. */
static void write_wav(const char *path, const int16_t *s, int n, int rate)
{
    uint8_t hdr[44] = "RIFF\0\0\0\0WAVEfmt ";
    g_autofree uint8_t *buf = g_malloc(sizeof(hdr) + 2 * n);

    stl_le_p(hdr + 4, 36 + 2 * n);
    stl_le_p(hdr + 16, 16);
    stw_le_p(hdr + 20, 1);
    stw_le_p(hdr + 22, 1);
    stl_le_p(hdr + 24, rate);
    stl_le_p(hdr + 28, rate * 2);
    stw_le_p(hdr + 32, 2);
    stw_le_p(hdr + 34, 16);
    memcpy(hdr + 36, "data", 4);
    stl_le_p(hdr + 40, 2 * n);
    memcpy(buf, hdr, sizeof(hdr));
    for (int i = 0; i < n; i++) {
        stw_le_p(buf + sizeof(hdr) + 2 * i, s[i]);
    }
    g_assert_true(g_file_set_contents(path, (char *)buf, sizeof(hdr) + 2 * n,
                                      NULL));
}

/*
 * A 16-bit WAV file's samples, its left channel's if stereo; their count
 * in *n.
 */
static int16_t *read_wav(const char *path, int *n)
{
    g_autofree gchar *wav = NULL;
    unsigned ch;
    int16_t *s;
    gsize len;

    g_assert_true(g_file_get_contents(path, &wav, &len, NULL));
    g_assert_cmpuint(len, >=, 44);
    ch = lduw_le_p(wav + 22);
    g_assert_cmpuint(ldl_le_p(wav + 24), ==, RATE);
    g_assert_cmpuint(lduw_le_p(wav + 34), ==, 16);
    *n = (len - 44) / (2 * ch);
    s = g_new(int16_t, *n + 1);
    for (int i = 0; i < *n; i++) {
        s[i] = lduw_le_p(wav + 44 + 2 * ch * i);
    }
    return s;
}

static int count_nonzero(const int16_t *s, int n)
{
    int k = 0;

    for (int i = 0; i < n; i++) {
        k += s[i] != 0;
    }
    return k;
}

/* ---- The control port ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2c/test]
 * The codec at 0x18 on GPIO7 and GPIO8: its chip ID, its registers' reset
 * values and widths, the read-only and undefined ones, INI_REG.
 */
static void test_registers(void)
{
    static const uint8_t reset[0x46] = {
        [0x00] = 0x1f, [0x03] = 0x10, [0x04] = 0x10, [0x06] = 0x03,
        [0x08] = 0xff, [0x0c] = 0x20, [0x0d] = 0xfc, [0x0e] = 0x6a,
        [0x10] = 0x13, [0x11] = 0x7c, [0x12] = 0x02, [0x13] = 0x40,
        [0x14] = 0x10, [0x16] = 0x04, [0x1b] = 0x0c, [0x1c] = 0x4c,
        [0x37] = 0x08,
    };
    QTestState *qts = start(NULL);
    uint8_t buf[3];

    i2c_board(qts);
    g_assert_cmphex(es_r(qts, 0xfd), ==, 0x83);
    g_assert_cmphex(es_r(qts, 0xfe), ==, 0x11);
    g_assert_cmphex(es_r(qts, 0xff), ==, 0x00);
    for (int r = 0; r < 0x46; r++) {
        g_assert_cmphex(es_r(qts, r), ==, reset[r]);
    }
    g_assert_cmphex(es_r(qts, ES_I2C), ==, 0);
    g_assert_cmphex(es_r(qts, ES_FLAG), ==, 0);

    /* Eight bits each; the chip ID and flags read-only; 0x46 undefined */
    es_w(qts, ES_DAC32, 0xa5);
    es_w(qts, ES_CLK7, 0xff);
    es_w(qts, 0xfd, 0);
    es_w(qts, 0x46, 0x55);
    g_assert_cmphex(es_r(qts, ES_DAC32), ==, 0xa5);
    g_assert_cmphex(es_r(qts, ES_CLK7), ==, 0xff);
    g_assert_cmphex(es_r(qts, 0xfd), ==, 0x83);
    g_assert_cmphex(es_r(qts, 0x46), ==, 0);

    /*
     * A transfer's further bytes stay on its register: written to it,
     * read from it again.
     */
    g_assert_true(i2c_write(qts, CODEC,
                            (uint8_t[]) { ES_DAC32, 0x11, 0x22 }, 3));
    g_assert_cmphex(es_r(qts, ES_DAC32), ==, 0x22);
    g_assert_cmphex(es_r(qts, 0x33), ==, 0);
    g_assert_true(i2c_read(qts, CODEC, 0xfd, buf, 3));
    g_assert_cmphex(buf[0], ==, 0x83);
    g_assert_cmphex(buf[1], ==, 0x83);
    g_assert_cmphex(buf[2], ==, 0x83);

    /* INI_REG holds every register but itself at its default. */
    es_w(qts, ES_I2C, 0x01);
    g_assert_cmphex(es_r(qts, ES_DAC32), ==, 0);
    g_assert_cmphex(es_r(qts, ES_CLK7), ==, 0);
    es_w(qts, ES_DAC32, 0x5a);
    g_assert_cmphex(es_r(qts, ES_DAC32), ==, 0);
    g_assert_cmphex(es_r(qts, ES_I2C), ==, 0x01);
    es_w(qts, ES_I2C, 0x00);
    es_w(qts, ES_DAC32, 0x5a);
    g_assert_cmphex(es_r(qts, ES_DAC32), ==, 0x5a);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c/test]
 * The codec answers 0x18 only (CE strapped low): not 0x19, not the
 * general call; and only on the board's wires, GPIO8 SCL and GPIO7 SDA.
 */
static void test_address(void)
{
    QTestState *qts = start(NULL);
    uint8_t v;

    i2c_board(qts);
    g_assert_true(i2c_read(qts, CODEC, 0xfd, &v, 1));
    g_assert_false(i2c_read(qts, 0x19, 0xfd, &v, 1));
    g_assert_false(i2c_write(qts, 0x00, (uint8_t[]) { 0x00, 0x00 }, 2));
    g_assert_cmphex(es_r(qts, ES_RESET), ==, 0x1f);

    /* SCL and SDA the other way round: the codec sees no transfer. */
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_SCL), SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_SDA), SIG_GPIO_OUT);
    i2c_setup(qts, PAD_SDA, PAD_SCL);
    g_assert_false(i2c_read(qts, CODEC, 0xfd, &v, 1));

    /* I2C0 on other pads, with their pull-ups: nothing there. */
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_SCL), SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_SDA), SIG_GPIO_OUT);
    i2c_setup(qts, 20, 21);
    qtest_writel(qts, IO_MUX_GPIO(20), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2) |
                 FUN_WPU);
    qtest_writel(qts, IO_MUX_GPIO(21), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2) |
                 FUN_WPU);
    g_assert_false(i2c_read(qts, CODEC, 0xfd, &v, 1));
    qtest_writel(qts, GPIO_FUNC_OUT(20), SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_FUNC_OUT(21), SIG_GPIO_OUT);

    /* Back on the board's wires, it answers. */
    i2c_board(qts);
    g_assert_true(i2c_read(qts, CODEC, 0xfd, &v, 1));
    g_assert_cmphex(v, ==, 0x83);
    qtest_quit(qts);
}

/* ---- The serial port ---- */

/*
 * The receiver's frames after the codec's echo of the transmitter's
 * (ADCDAT_SEL 6, DACL + DACR): the left word a frame late, the right two,
 * as the codec loads ASDOUT's words at the frame's start and the right
 * word's last bit, one cycle late in I2S, comes after it.
 */
static void check_echo(const int16_t *tx, const int16_t *rx, int n)
{
    int at = -1;

    for (int f = 0; f + 1 < n && at < 0; f++) {
        if (rx[2 * (f + 1)] == tx[0]) {
            at = f + 1;
        }
    }
    g_assert_cmpint(at, >=, 1);
    for (int f = 0; at + f + 2 < n; f++) {
        g_assert_cmphex((uint16_t)rx[2 * (at + f)], ==, (uint16_t)tx[2 * f]);
        g_assert_cmphex((uint16_t)rx[2 * (at + f + 1) + 1], ==,
                        (uint16_t)tx[2 * f + 1]);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * I2S0 full duplex on the codec's pads: DSDIN takes the transmitter's
 * words and ASDOUT gives them back (ADCDAT_SEL 6) to the receiver, word
 * for word.
 */
static void test_echo(void)
{
    QTestState *qts = start(NULL);
    int16_t tx[2 * 256], rx[2 * 256];

    for (int i = 0; i < 256; i++) {
        tx[2 * i] = 0x1234 + 77 * i;
        tx[2 * i + 1] = -0x2345 - 51 * i;
    }
    i2c_board(qts);
    es_setup(qts, 0xbf, 6);
    i2s_clocks(qts);
    i2s_board_pads(qts);
    i2s_duplex(qts, tx, 256, MEM + 0x4000);
    qtest_clock_step(qts, 300 * FRAME_NS);
    qtest_memread(qts, MEM + 0x4000, rx, sizeof(rx));
    check_echo(tx, rx, 256);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * [spec:nuos:req:emu.esp32p4.gpio/test]
 * The codec sees only its pads: I2S0 on other pads reaches nothing, and
 * its receiver reads its own data pad; the codec's words in a format
 * other than the I2S unit's (left-justified against Philips) come back a
 * bit out; a reserved word length moves no data.
 */
static void test_pads_and_formats(void)
{
    QTestState *qts = start(NULL);
    int16_t tx[2 * 128], rx[2 * 128];
    int shifted = 0;

    for (int i = 0; i < 128; i++) {
        tx[2 * i] = 0x1111 + 0x101 * i;
        tx[2 * i + 1] = 0x2222 + 0x33 * i;
    }
    i2c_board(qts);
    es_setup(qts, 0xbf, 6);
    i2s_clocks(qts);

    /* BCK and WS on GPIO20 and GPIO21: the codec has no clock. */
    i2s_pads(qts, PAD_MCLK, 20, 21, PAD_DSDIN, PAD_ASDOUT);
    i2s_duplex(qts, tx, 128, MEM + 0x4000);
    qtest_clock_step(qts, 200 * FRAME_NS);
    qtest_memread(qts, MEM + 0x4000, rx, sizeof(rx));
    for (int i = 0; i < 2 * 128; i++) {
        g_assert_cmphex(rx[i], ==, 0);
    }
    i2s_stop(qts);

    /*
     * The codec takes DSDIN left-justified: its left word (LRCK high) is
     * the I2S unit's left LSB, a cycle late in Philips, then the right
     * word's top 15 bits; ASDOUT gives that back as its left word.
     */
    i2s_board_pads(qts);
    es_w(qts, ES_SDPIN, (es_r(qts, ES_SDPIN) & 0xfc) | 1);
    i2s_duplex(qts, tx, 128, MEM + 0x4000);
    qtest_clock_step(qts, 200 * FRAME_NS);
    qtest_memread(qts, MEM + 0x4000, rx, sizeof(rx));
    for (int f = 0; f + 1 < 128 && shifted == 0; f++) {
        uint16_t want = (tx[0] & 1) << 15 | (uint16_t)tx[1] >> 1;

        if ((uint16_t)rx[2 * (f + 1)] == want) {
            shifted = f + 1;
        }
    }
    g_assert_cmpint(shifted, >, 0);
    for (int f = 0; shifted + f < 128; f++) {
        uint16_t want = (tx[2 * f] & 1) << 15 | (uint16_t)tx[2 * f + 1] >> 1;

        g_assert_cmphex((uint16_t)rx[2 * (shifted + f)], ==, want);
    }
    i2s_stop(qts);

    /* A reserved word length: ASDOUT stays low. */
    es_w(qts, ES_SDPIN, 0x0c);
    es_w(qts, ES_SDPOUT, 0x1c);
    i2s_duplex(qts, tx, 128, MEM + 0x4000);
    qtest_clock_step(qts, 200 * FRAME_NS);
    qtest_memread(qts, MEM + 0x4000, rx, sizeof(rx));
    for (int i = 0; i < 2 * 128; i++) {
        g_assert_cmphex(rx[i], ==, 0);
    }
    qtest_quit(qts);
}

/* ---- Playback ---- */

/*
 * Play @n frames of a ramp (both channels the same) through the codec at
 * volume @vol into a WAV file; its samples, *count of them.
 */
static int16_t *play(const char *codec_extra, uint8_t vol, bool pa_on,
                     int16_t *tx, int n, int *count,
                     void (*tweak)(QTestState *qts))
{
    g_autofree char *path = tmp_wav();
    QTestState *qts;
    int16_t *out;

    qts = qtest_initf("-M esp32p4,strap-mode=0,audiodev=snd0 "
                      "-audiodev wav,id=snd0,path=%s,out.frequency=%d,"
                      "out.format=s16 %s", path, RATE,
                      codec_extra ? codec_extra : "");
    i2c_board(qts);
    es_setup(qts, vol, 0);
    i2s_clocks(qts);
    i2s_board_pads(qts);
    if (tweak) {
        tweak(qts);
    }
    pa(qts, pa_on);
    /* The amplifier's start-up, 30 ms */
    qtest_clock_step(qts, 40 * 1000 * 1000);
    i2s_duplex(qts, tx, n, MEM + 0x8000);
    qtest_clock_step(qts, (n + 2000) * (uint64_t)FRAME_NS);
    qtest_quit(qts);
    out = read_wav(path, count);
    unlink(path);
    return out;
}

static void make_tone(int16_t *tx, int n)
{
    for (int i = 0; i < n; i++) {
        int16_t v = 2000 + 37 * i;

        tx[2 * i] = v;
        tx[2 * i + 1] = -v;
    }
}

/*
 * Whether the tone's left channel is in @s, times @gain, +-1 for the
 * mixer's rounding: all of it but the first few samples, which go by as
 * the codec powers up and takes LRCK's rate.
 */
static bool has_tone(const int16_t *tx, int n, const int16_t *s, int count,
                     double gain)
{
    for (int j = 0; j < 8; j++) {
        for (int at = 0; at + n - j <= count; at++) {
            bool ok = true;

            for (int i = 0; i < n - j && ok; i++) {
                ok = abs(s[at + i] - (int)(tx[2 * (j + i)] * gain)) <= 1;
            }
            if (ok) {
                return true;
            }
        }
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The DAC through the NS4150B to the speaker, the audiodev's playback
 * stream: the left channel at 31250 Hz, sample for sample, at the codec's
 * volume (0 dB, then -6 dB); nothing while PA_CTRL holds the amplifier off.
 */
static void test_playback(void)
{
    int16_t tx[2 * 512], *out;
    int count;

    make_tone(tx, 512);
    out = play(NULL, 0xbf, true, tx, 512, &count, NULL);
    g_assert_true(has_tone(tx, 512, out, count, 1.0));
    g_free(out);

    out = play(NULL, 0xbf - 12, true, tx, 512, &count, NULL);
    g_assert_true(has_tone(tx, 512, out, count, 0.501187233627272));
    g_free(out);

    out = play(NULL, 0xbf, false, tx, 512, &count, NULL);
    g_assert_cmpint(count_nonzero(out, count), ==, 0);
    g_assert_cmpint(count, >=, 500);
    g_free(out);
}

static void tweak_no_mclk(QTestState *qts)
{
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_MCLK), SIG_GPIO_OUT);
}

static void tweak_slow_dac_clock(QTestState *qts)
{
    /* The DAC's clock MCLK / 2: 128 times LRCK */
    es_w(qts, ES_CLK5, 0x01);
}

static void tweak_mute(QTestState *qts)
{
    es_w(qts, ES_DAC31, 0x60);
}

static void tweak_powered_down(QTestState *qts)
{
    es_w(qts, ES_SYS12, 0x02);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * What silences the DAC: no MCLK on GPIO13, its clock below 256 times
 * LRCK, its mutes, PDN_DAC.
 */
static void test_playback_rejects(void)
{
    static void (*const tweaks[])(QTestState *) = {
        tweak_no_mclk, tweak_slow_dac_clock, tweak_mute,
        tweak_powered_down,
    };
    int16_t tx[2 * 256], *out;
    int count;

    make_tone(tx, 256);
    for (int i = 0; i < ARRAY_SIZE(tweaks); i++) {
        out = play(NULL, 0xbf, true, tx, 256, &count, tweaks[i]);
        g_assert_cmpint(count, >=, 240);
        g_assert_cmpint(count_nonzero(out, count), ==, 0);
        g_free(out);
    }
}

/* ---- Capture ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The microphone, the audiodev's capture stream from a WAV file, through
 * the ADC (PGA and scale at 0 dB, volume 0 dB, then +6 dB) and ASDOUT
 * into I2S0's receiver.
 */
static void test_capture(void)
{
    g_autofree char *path = tmp_wav();
    g_autofree char *dummy = tmp_wav();
    int16_t mic[8192], tx[2 * 2048] = { 0 }, rx[2 * 2048];
    QTestState *qts;
    int at, k;

    for (int i = 0; i < 8192; i++) {
        mic[i] = (i % 400) * 40 - 8000;
    }
    write_wav(path, mic, 8192, RATE);
    qts = qtest_initf("-M esp32p4,strap-mode=0,audiodev=snd0 "
                      "-audiodev wav,id=snd0,path=%s,in-path=%s", dummy,
                      path);
    i2c_board(qts);
    es_setup(qts, 0xbf, 0);
    i2s_clocks(qts);
    i2s_board_pads(qts);
    i2s_duplex(qts, tx, 2048, MEM + 0x8000);
    qtest_clock_step(qts, 2100 * (uint64_t)FRAME_NS);
    qtest_memread(qts, MEM + 0x8000, rx, sizeof(rx));
    /* The microphone's samples in both slots, +-1 for the mixer */
    at = -1;
    for (int f = 0; f < 2048 && at < 0; f++) {
        if (rx[2 * f] != 0) {
            at = f;
        }
    }
    g_assert_cmpint(at, >=, 0);
    k = -1;
    for (int i = 0; i < 400 && k < 0; i++) {
        if (abs(rx[2 * (at + 10)] - mic[i]) <= 1 &&
            abs(rx[2 * (at + 11)] - mic[i + 1]) <= 1 &&
            abs(rx[2 * (at + 12)] - mic[i + 2]) <= 1) {
            k = i;
        }
    }
    g_assert_cmpint(k, >=, 0);
    for (int f = at + 10; f < 2048; f++) {
        int want = mic[k + f - at - 10];

        g_assert_cmpint(abs(rx[2 * f] - want), <=, 1);
        g_assert_cmpint(rx[2 * f + 1], ==, rx[2 * f]);
    }
    qtest_quit(qts);
    unlink(path);
    unlink(dummy);
}

/* ---- The LP I2S and its VAD ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The LP I2S, master on the LP pads GPIO10 (WS) and GPIO12 (BCK) through
 * the LP IO MUX, takes the codec's ADC on GPIO11; the codec's MCLK is
 * SCLK (MCLK_SEL), 2 MHz, x4: the microphone's tone reaches the LP I2S
 * memory and the VAD calls it speech.
 */
static void test_lp_i2s(void)
{
    g_autofree char *path = tmp_wav();
    g_autofree char *dummy = tmp_wav();
    int16_t mic[RATE];
    QTestState *qts;
    uint32_t w;
    bool nonzero = false;

    for (int i = 0; i < RATE; i++) {
        mic[i] = i < RATE / 4 ? 0 : ((i / 8) % 2 ? 9000 : -9000);
    }
    write_wav(path, mic, RATE, RATE);
    qts = qtest_initf("-M esp32p4,strap-mode=0,audiodev=snd0 "
                      "-audiodev wav,id=snd0,path=%s,in-path=%s", dummy,
                      path);
    i2c_board(qts);
    es_setup(qts, 0xbf, 0);
    /* MCLK from SCLK: 64 cycles a frame (32-bit slots), x4 */
    es_w(qts, ES_CLK1, 0xbf);
    es_w(qts, ES_CLK2, 2 << 3);
    es_w(qts, ES_SDPOUT, 0x0c);

    /* The LP pads in the LP IO MUX, through the LP matrix */
    qtest_writel(qts, LP_IOMUX_PAD(PAD_LRCK), LP_PAD_MUX_SEL |
                 LP_PAD_FUN_SEL(1) | LP_PAD_FUN_IE | 2);
    qtest_writel(qts, LP_IOMUX_PAD(PAD_SCLK), LP_PAD_MUX_SEL |
                 LP_PAD_FUN_SEL(1) | LP_PAD_FUN_IE | 2);
    qtest_writel(qts, LP_IOMUX_PAD(PAD_ASDOUT), LP_PAD_MUX_SEL |
                 LP_PAD_FUN_SEL(1) | LP_PAD_FUN_IE | 2);
    qtest_writel(qts, LP_GPIO_FUNC_OUT(PAD_LRCK), LP_OUT_SEL(LP_SIG_I_WS));
    qtest_writel(qts, LP_GPIO_FUNC_OUT(PAD_SCLK), LP_OUT_SEL(LP_SIG_I_BCK));
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_I_SD), LP_IN_SIG_IN_SEL |
                 LP_IN_SEL(PAD_ASDOUT));

    /* LP_I2S_RX_CLK: XTAL_D2 / 2, 10 MHz; BCK / 5, 2 MHz */
    qtest_writel(qts, LPPERI_CORE_CLK_SEL,
                 (qtest_readl(qts, LPPERI_CORE_CLK_SEL) & ~(3u << 26)) |
                 1u << 26);
    qtest_writel(qts, LPPERI_I2S_RX_NUM, 2u << 24);
    qtest_writel(qts, LP_I2S + LP_VAD_PARAM(0), 4u << 16 | 100);
    qtest_writel(qts, LP_I2S + LP_VAD_CONF, 1);
    qtest_writel(qts, LP_I2S + LP_RX_TDM_CTRL, TDM(2, 0x1));
    qtest_writel(qts, LP_I2S + LP_RX_CONF1, LP_CONF1(32, 5, 16, 32, 32,
                                                     true));
    qtest_writel(qts, LP_I2S + LP_RX_CONF, 0x9600 | C_TDM_EN | C_UPDATE);
    qtest_writel(qts, LP_I2S + LP_RX_CONF, 0x9600 | C_TDM_EN | C_START);
    qtest_clock_step(qts, 600 * 1000 * 1000ull);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_INT_RAW) & LP_INT_VAD_DONE,
                    ==, LP_INT_VAD_DONE);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_VAD_RESULT) & 1, ==, 1);
    for (int i = 0; i < 64; i++) {
        w = qtest_readl(qts, LP_I2S + LP_RAM);
        nonzero |= (int16_t)w > 8000 || (int16_t)w < -8000;
    }
    g_assert_true(nonzero);
    qtest_quit(qts);
    unlink(path);
    unlink(dummy);
}

/* ---- Master mode ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The codec as master (MSC): SCLK = MCLK / 8 on GPIO12 and LRCK = MCLK /
 * 256 on GPIO10, edge by edge; I2S0's units, slaves on those pads, send
 * DSDIN and receive its echo on ASDOUT.
 */
static void test_master(void)
{
    QTestState *qts = start(NULL);
    int16_t tx[2 * 64], rx[2 * 64];
    uint32_t txc = TX_BASE | C_TDM_EN | C_SLAVE;
    uint32_t rxc = RX_BASE | C_TDM_EN | C_SLAVE;

    for (int i = 0; i < 64; i++) {
        tx[2 * i] = 0x0f0f + 0x111 * i;
        tx[2 * i + 1] = 0x7070 - 0x99 * i;
    }
    i2c_board(qts);
    es_setup(qts, 0xbf, 6);
    i2s_clocks(qts);
    pad_out(qts, PAD_MCLK, SIG_MCLK(0));
    pad_out(qts, PAD_DSDIN, SIG_SD(0));
    pad_in(qts, PAD_ASDOUT, SIG_SD(0));
    pad_in(qts, PAD_SCLK, SIG_O_BCK(0));
    pad_in(qts, PAD_LRCK, SIG_O_WS(0));
    pad_in(qts, PAD_SCLK, SIG_I_BCK(0));
    pad_in(qts, PAD_LRCK, SIG_I_WS(0));
    qtest_memwrite(qts, MEM + 0x1000, tx, sizeof(tx));
    i2s_w(qts, RXEOF_NUM, 2 * 64 - 1);
    unit_setup(qts, false, rxc);
    unit_setup(qts, true, txc);
    dma_rx(qts, 1, MEM, MEM + 0x4000, sizeof(rx));
    dma_tx(qts, 0, MEM + 0x100, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 10000);
    i2s_w(qts, RX_CONF, rxc | C_START);
    i2s_w(qts, TX_CONF, txc | C_START);

    /* SCLK MCLK / 8, LRCK MCLK / 256; then master mode on */
    es_w(qts, ES_CLK6, 7);
    es_w(qts, ES_CLK7, 0);
    es_w(qts, ES_CLK8, 255);
    es_w(qts, ES_RESET, 0xc0);
    qtest_clock_step(qts, 100 * FRAME_NS);
    qtest_memread(qts, MEM + 0x4000, rx, sizeof(rx));
    check_echo(tx, rx, 64);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c/test]
 * The codec answers whatever drives its wires: GPIO8 and GPIO7 bit-banged
 * as open-drain GPIOs write a register that I2C0 then reads back.
 */
static void test_bitbang_i2c(void)
{
    QTestState *qts = start(NULL);

    esp32p4_codec_write(qts, ES_DAC32, 0x9a);
    i2c_board(qts);
    g_assert_cmphex(es_r(qts, ES_DAC32), ==, 0x9a);
    qtest_quit(qts);
}

static void drive(QTestState *qts, int pad, int level)
{
    qtest_set_irq_in(qts, "/machine/gpio", "pad-in", pad, level);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * [spec:nuos:req:emu.esp32p4.gpio/test]
 * The codec clocked edge by edge from its pads (here driven from outside
 * the chip, as a bit-banged master would): DSDIN sampled as SCLK rises,
 * ASDOUT changing as it falls, read back on GPIO11. With ADCDAT_SEL 6 a
 * frame's left word comes back in the next.
 */
static void test_edge_clocked(void)
{
    QTestState *qts = start(NULL);
    static const uint16_t left[4] = { 0x1234, 0xa5c3, 0x0ff1, 0x8421 };
    uint16_t got = 0;

    i2c_board(qts);
    es_setup(qts, 0xbf, 6);
    qtest_writel(qts, IO_MUX_GPIO(PAD_ASDOUT), MCU_SEL_GPIO | FUN_IE);
    for (int f = 0; f < 4; f++) {
        for (int i = 0; i < 32; i++) {
            /* Philips: WS low for the left word, its MSB a cycle late */
            int b = i == 0 ? -1 : i - 1;
            bool bit = b >= 0 && b < 16 && ((left[f] >> (15 - b)) & 1);

            drive(qts, PAD_LRCK, i >= 16);
            drive(qts, PAD_DSDIN, bit);
            drive(qts, PAD_SCLK, 0);
            qtest_clock_step(qts, 500);
            drive(qts, PAD_SCLK, 1);
            if (f == 3 && i >= 1 && i <= 16) {
                got |= ((qtest_readl(qts, GPIO_IN) >> PAD_ASDOUT) & 1) <<
                       (16 - i);
            }
            qtest_clock_step(qts, 500);
        }
    }
    g_assert_cmphex(got, ==, left[2]);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The NS4150B's start-up: the speaker plays only 30 ms after PA_CTRL
 * rises; the samples before go unheard.
 */
static void test_pa_startup(void)
{
    g_autofree char *path = tmp_wav();
    QTestState *qts;
    int16_t tx[2 * 2048], *out;
    int count, first = -1;

    for (int i = 0; i < 2048; i++) {
        tx[2 * i] = tx[2 * i + 1] = 1000 + i;
    }
    qts = qtest_initf("-M esp32p4,strap-mode=0,audiodev=snd0 "
                      "-audiodev wav,id=snd0,path=%s,out.frequency=%d,"
                      "out.format=s16", path, RATE);
    i2c_board(qts);
    es_setup(qts, 0xbf, 0);
    i2s_clocks(qts);
    i2s_board_pads(qts);
    i2s_duplex(qts, tx, 2048, MEM + 0x8000);
    pa(qts, true);
    qtest_clock_step(qts, 2100 * (uint64_t)FRAME_NS);
    qtest_quit(qts);
    out = read_wav(path, &count);
    unlink(path);
    for (int i = 0; i < count && first < 0; i++) {
        if (out[i]) {
            first = i;
        }
    }
    /* 30 ms is 937.5 frames at 31250 Hz. */
    g_assert_cmpint(first, >=, 0);
    g_assert_cmpint(abs(out[first] - 1000 - 937), <=, 4);
    g_free(out);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/es8311/registers", test_registers);
    qtest_add_func("/esp32p4/es8311/address", test_address);
    qtest_add_func("/esp32p4/es8311/echo", test_echo);
    qtest_add_func("/esp32p4/es8311/pads-and-formats",
                   test_pads_and_formats);
    qtest_add_func("/esp32p4/es8311/playback", test_playback);
    qtest_add_func("/esp32p4/es8311/playback-rejects",
                   test_playback_rejects);
    qtest_add_func("/esp32p4/es8311/capture", test_capture);
    qtest_add_func("/esp32p4/es8311/lp-i2s", test_lp_i2s);
    qtest_add_func("/esp32p4/es8311/master", test_master);
    qtest_add_func("/esp32p4/es8311/bitbang-i2c", test_bitbang_i2c);
    qtest_add_func("/esp32p4/es8311/edge-clocked", test_edge_clocked);
    qtest_add_func("/esp32p4/es8311/pa-startup", test_pa_startup);
    return g_test_run();
}
