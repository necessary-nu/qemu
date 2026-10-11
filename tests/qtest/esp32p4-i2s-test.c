/*
 * QTest testcase for the ESP32-P4 I2S controllers and the LP I2S with its
 * voice activity detection
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The controllers run from the 40 MHz crystal divided in HP_SYS_CLKRST,
 * move data through GDMA-AHB descriptors in L2MEM (past the 128 KiB the
 * L2 cache holds from reset) and reach each other through pads the GPIO
 * matrices route; virtual time is stepped while frames go by.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"
#include "qobject/qdict.h"

#define I2S(n)              (0x500c6000 + 0x1000 * (n))
#define LP_I2S              0x50125000
#define MEM                 0x4ff40000

/* HP I2S registers */
#define INT_RAW             0x0c
#define INT_ST              0x10
#define INT_ENA             0x14
#define INT_CLR             0x18
#define RX_CONF             0x20
#define TX_CONF             0x24
#define RX_CONF1            0x28
#define TX_CONF1            0x2c
#define TX_PCM2PDM_CONF     0x40
#define TX_PCM2PDM_CONF1    0x44
#define RX_PDM2PCM_CONF     0x48
#define RX_TDM_CTRL         0x50
#define TX_TDM_CTRL         0x54
#define RX_TIMING           0x58
#define TX_TIMING           0x5c
#define LC_HUNG_CONF        0x60
#define RXEOF_NUM           0x64
#define SINGLE_DATA         0x68
#define STATE               0x6c
#define ETM_CONF            0x70
#define FIFO_CNT            0x74
#define BCK_CNT             0x78
#define CLK_GATE            0x7c
#define DATE                0x80

#define INT_RX_DONE         (1u << 0)
#define INT_TX_DONE         (1u << 1)
#define INT_RX_HUNG         (1u << 2)
#define INT_TX_HUNG         (1u << 3)

#define C_RESET             (1u << 0)
#define C_FIFO_RESET        (1u << 1)
#define C_START             (1u << 2)
#define C_SLAVE             (1u << 3)
#define C_STOP_MODE(m)      ((m) << 4)
#define C_TX_STOP_EN        (1u << 4)
#define C_TX_CHAN_EQUAL     (1u << 5)
#define C_TX_MONO           (1u << 6)
#define C_BIG_ENDIAN        (1u << 7)
#define C_UPDATE            (1u << 8)
#define C_MONO_FST_VLD      (1u << 9)
#define C_PCM_CONF(c)       ((c) << 10)
#define C_PCM_BYPASS        (1u << 12)
#define C_MSB_SHIFT         (1u << 13)
#define C_LEFT_ALIGN        (1u << 15)
#define C_WS_IDLE_POL       (1u << 17)
#define C_BIT_ORDER         (1u << 18)
#define C_TDM_EN            (1u << 19)
#define C_PDM_EN            (1u << 20)
#define C_BCK_DIV(m)        ((uint32_t)((m) - 1) << 21)
#define C_TX_CHAN_MOD(m)    ((uint32_t)(m) << 27)
#define C_SIG_LOOPBACK      (1u << 30)

#define CONF1(ws, bits, half, chan) \
    ((uint32_t)((ws) - 1) | (uint32_t)((bits) - 1) << 14 | \
     (uint32_t)((half) - 1) << 19 | (uint32_t)((chan) - 1) << 27)
#define TDM(tot, mask)      ((uint32_t)((tot) - 1) << 16 | (mask))
#define TDM_SKIP_MSK        (1u << 20)

#define P2D_OSR2(n)         ((uint32_t)(n) << 1)
#define P2D_SHIFTS_X1       (1u << 13 | 1u << 15 | 1u << 17 | 1u << 19)
#define P2D_CONV_EN         (1u << 25)
#define D2P_EN              (1u << 19)
#define D2P_DSR_16          (1u << 20)
#define D2P_AMPLIFY(n)      ((uint32_t)(n) << 21)
#define D2P_HP_BYPASS       (1u << 25)

/* LP I2S registers */
#define LP_VAD_CONF         0x00
#define LP_VAD_RESULT       0x04
#define LP_RX_MEM_CONF      0x08
#define LP_INT_RAW          0x0c
#define LP_INT_ST           0x10
#define LP_INT_ENA          0x14
#define LP_INT_CLR          0x18
#define LP_RX_CONF          0x20
#define LP_RX_CONF1         0x28
#define LP_RX_TDM_CTRL      0x50
#define LP_RXEOF_NUM        0x64
#define LP_RX_PDM_CONF      0x70
#define LP_ECO_HIGH         0x78
#define LP_VAD_PARAM(n)     (0x80 + 4 * (n))
#define LP_VAD_OB(n)        (0xb0 + 4 * (n))
#define LP_CLK_GATE         0xf8
#define LP_DATE             0xfc
#define LP_RAM              0xc00

#define LP_INT_RX_DONE      (1u << 0)
#define LP_INT_RX_HUNG      (1u << 1)
#define LP_INT_UDF          (1u << 2)
#define LP_INT_VAD_DONE     (1u << 3)
#define LP_INT_VAD_RESET    (1u << 4)
#define LP_INT_THRESHOLD    (1u << 5)
#define LP_C_FIFOMEM_RESET  (1u << 4)
#define LP_CONF1(ws, div, bits, half, chan, shift) \
    ((uint32_t)((ws) - 1) | (uint32_t)((div) - 1) << 7 | \
     (uint32_t)((bits) - 1) << 13 | (uint32_t)((half) - 1) << 18 | \
     (uint32_t)((chan) - 1) << 24 | (shift ? 1u << 29 : 0))

#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SOC_CLK_CTRL2       (HP_CLKRST + 0x1c)
#define SOC_CLK_CTRL3       (HP_CLKRST + 0x20)
#define HP_RST_EN1          (HP_CLKRST + 0xc4)
#define I2S_APB_CLK_EN(n)   (1u << (14 + (n)))
#define LP_CLKRST           0x50111000
#define LP_HP_CLK_CTRL      (LP_CLKRST + 0x40)
#define PAD_I2S_MCLK_EN(n)  (1u << (12 - (n)))
#define LPPERI              0x50120000
#define LPPERI_CLK_EN       (LPPERI + 0x00)
#define LPPERI_CORE_CLK_SEL (LPPERI + 0x04)
#define LPPERI_I2S_RX_NUM   (LPPERI + 0x30)
#define LPPERI_I2S_RX_XYZ   (LPPERI + 0x34)

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
#define G_RAW               0x0
#define G_CLR               0xc
#define IN_SUC_EOF          (1u << 1)
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define CONF0_RST           (1u << 0)
#define DW0(size, len)      ((size) | (uint32_t)(len) << 12)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)
#define PERI_I2S(n)         (3 + (n))

/* GPIO matrix, IO MUX and LP matrix */
#define GPIO                0x500e0000
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (1u << 12)
#define LP_GPIO_FUNC_IN(s)  (0x5012a074 + 4 * (s))
#define LP_SIG_IN_SEL       (1u << 1)

/* Matrix signals of I2Sn */
#define SIG_O_BCK(n)        (25 + 6 * (n))
#define SIG_MCLK(n)         (26 + 6 * (n))
#define SIG_O_WS(n)         (27 + 6 * (n))
#define SIG_SD(n)           (28 + 6 * (n))
#define SIG_I_BCK(n)        (29 + 6 * (n))
#define SIG_I_WS(n)         (30 + 6 * (n))
#define LP_SIG_I_BCK        9
#define LP_SIG_I_SD         10
#define LP_SIG_I_WS         11

/* The PMU's sleep request and rejection */
#define PMU                 0x50115000
#define PMU_SLP_WAKEUP_CNTL0 (PMU + 0x120)
#define PMU_SLP_WAKEUP_CNTL1 (PMU + 0x124)
#define PMU_SLP_WAKEUP_CNTL4 (PMU + 0x130)
#define PMU_SLP_WAKEUP_STATUS1 (PMU + 0x148)

/* The interrupt matrix's source status, core 0 */
#define INTMTX_STATUS(w)    (0x500d6000 + 0x200 + 4 * (w))
#define SRC_LP_I2S          12
#define SRC_I2S(n)          (27 + (n))

/* The ETM */
#define ETM                 0x500d5000
#define ETM_CH_ENA_SET      (ETM + 0x004)
#define ETM_CH_EVT_ID(n)    (ETM + 0x018 + 8 * (n))
#define ETM_CH_TASK_ID(n)   (ETM + 0x01c + 8 * (n))
#define ETM_I2S_EVT_X_WORDS_SENT(n) (152 + 4 * (n))
#define ETM_I2S_TASK_STOP_TX(n)     (170 + 4 * (n))

/* A unit's clock: XTAL / n, from HP_SYS_CLKRST_PERI_CLK_CTRL11 to 19. */
typedef struct Fld {
    uint32_t reg;
    int shift;
    int width;
} Fld;

typedef struct UnitClk {
    Fld en, sel, n, x, y, z, yn1;
} UnitClk;

#define F(r, s, w)  { HP_CLKRST + (r), (s), (w) }

static const UnitClk unit_clk[3][2] = {
    {   /* I2S0 RX, TX */
        { F(0x44, 24, 1), F(0x44, 25, 2), F(0x48, 0, 8), F(0x48, 8, 9),
          F(0x48, 17, 9), F(0x4c, 0, 9), F(0x4c, 9, 1) },
        { F(0x4c, 10, 1), F(0x4c, 11, 2), F(0x4c, 13, 8), F(0x4c, 21, 9),
          F(0x50, 0, 9), F(0x50, 9, 9), F(0x50, 18, 1) },
    },
    {
        { F(0x50, 20, 1), F(0x50, 21, 2), F(0x50, 23, 8), F(0x54, 0, 9),
          F(0x54, 9, 9), F(0x54, 18, 9), F(0x54, 27, 1) },
        { F(0x54, 28, 1), F(0x54, 29, 2), F(0x58, 0, 8), F(0x58, 8, 9),
          F(0x58, 17, 9), F(0x5c, 0, 9), F(0x5c, 9, 1) },
    },
    {
        { F(0x5c, 11, 1), F(0x5c, 12, 2), F(0x5c, 14, 8), F(0x5c, 22, 9),
          F(0x60, 0, 9), F(0x60, 9, 9), F(0x60, 18, 1) },
        { F(0x60, 19, 1), F(0x60, 20, 2), F(0x60, 22, 8), F(0x64, 0, 9),
          F(0x64, 9, 9), F(0x64, 18, 9), F(0x64, 27, 1) },
    },
};

/* HP_SYS_CLKRST_I2Sn_MST_CLK_SEL: 1 for the TX clock */
static const Fld mst_sel[3] = {
    F(0x50, 19, 1), F(0x5c, 10, 1), F(0x64, 28, 1),
};

#define SRC_XTAL            0
#define SRC_PAD_MCLK        2

static QTestState *start(void)
{
    return qtest_init("-M esp32p4");
}

static void set_fld(QTestState *qts, Fld f, uint32_t v)
{
    uint32_t mask = ((1u << f.width) - 1) << f.shift;
    uint32_t r = qtest_readl(qts, f.reg);

    qtest_writel(qts, f.reg, (r & ~mask) | ((v << f.shift) & mask));
}

/* Unit (n, tx) on source src divided by div, an integer divider. */
static void set_unit_clk(QTestState *qts, int n, int tx, int src, int div)
{
    const UnitClk *c = &unit_clk[n][tx];

    set_fld(qts, c->sel, src);
    set_fld(qts, c->x, 0);
    set_fld(qts, c->z, 0);
    set_fld(qts, c->y, 1);
    set_fld(qts, c->yn1, 0);
    set_fld(qts, c->n, div);
    set_fld(qts, c->en, 1);
}

/* I2Sn's bus clock, and both its units at 8 MHz. */
static void clocks_on(QTestState *qts, int n)
{
    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) | I2S_APB_CLK_EN(n));
    set_unit_clk(qts, n, 0, SRC_XTAL, 5);
    set_unit_clk(qts, n, 1, SRC_XTAL, 5);
}

static void i2s_w(QTestState *qts, int n, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, I2S(n) + reg, v);
}

static uint32_t i2s_r(QTestState *qts, int n, uint32_t reg)
{
    return qtest_readl(qts, I2S(n) + reg);
}

/* Pad @pad, its input enabled, driven by output signal @out_sig. */
static void pad_out(QTestState *qts, int pad, int out_sig)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(pad), out_sig);
}

/* Input signal @in_sig taken from pad @pad. */
static void pad_in(QTestState *qts, int pad, int in_sig)
{
    qtest_writel(qts, GPIO_FUNC_IN(in_sig), IN_SIG_IN_SEL | pad);
}

static void wire(QTestState *qts, int pad, int out_sig, int in_sig)
{
    pad_out(qts, pad, out_sig);
    pad_in(qts, pad, in_sig);
}

/*
 * A unit's configuration, its unit and FIFO reset and the configuration
 * carried into its clock domain by UPDATE; then START. A transmitter's
 * FIFO reset drops what GDMA has put there, so DMA starts in between.
 */
static void unit_setup(QTestState *qts, int n, bool tx, uint32_t conf,
                       uint32_t conf1, uint32_t tdm)
{
    i2s_w(qts, n, tx ? TX_CONF1 : RX_CONF1, conf1);
    i2s_w(qts, n, tx ? TX_TDM_CTRL : RX_TDM_CTRL, tdm);
    i2s_w(qts, n, tx ? TX_CONF : RX_CONF, conf | C_RESET | C_FIFO_RESET);
    i2s_w(qts, n, tx ? TX_CONF : RX_CONF, conf | C_UPDATE);
    g_assert_cmphex(i2s_r(qts, n, tx ? TX_CONF : RX_CONF) & C_UPDATE, ==, 0);
}

static void unit_start(QTestState *qts, int n, bool tx, uint32_t conf)
{
    i2s_w(qts, n, tx ? TX_CONF : RX_CONF, conf | C_START);
}

static void unit_go(QTestState *qts, int n, bool tx, uint32_t conf,
                    uint32_t conf1, uint32_t tdm)
{
    unit_setup(qts, n, tx, conf, conf1, tdm);
    unit_start(qts, n, tx, conf);
}

/* GDMA-AHB channel @ch's outlink: @len bytes at @buf for peripheral. */
static void dma_tx(QTestState *qts, int ch, int periph, uint32_t desc,
                   uint32_t buf, uint32_t len)
{
    qtest_writel(qts, AHB_OUT(ch) + G_CONF0, CONF0_RST);
    qtest_writel(qts, AHB_OUT(ch) + G_CONF0, 0);
    qtest_writel(qts, AHB_OUT_INT(ch) + G_CLR, 0x7f);
    qtest_writel(qts, AHB_OUT(ch) + G_PERI_SEL, periph);
    qtest_writel(qts, desc, DW0(len, len) | SUC_EOF | OWNER);
    qtest_writel(qts, desc + 4, buf);
    qtest_writel(qts, desc + 8, 0);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(ch), desc);
    qtest_writel(qts, AHB_OUT(ch) + G_LINK, OUT_LINK_START);
}

/* GDMA-AHB channel @ch's inlink: a buffer of @size bytes at @buf. */
static void dma_rx(QTestState *qts, int ch, int periph, uint32_t desc,
                   uint32_t buf, uint32_t size)
{
    qtest_writel(qts, AHB_IN(ch) + G_CONF0, CONF0_RST);
    qtest_writel(qts, AHB_IN(ch) + G_CONF0, 0);
    qtest_writel(qts, AHB_IN_INT(ch) + G_CLR, 0xff);
    qtest_writel(qts, AHB_IN(ch) + G_PERI_SEL, periph);
    qtest_writel(qts, desc, DW0(size, 0) | OWNER);
    qtest_writel(qts, desc + 4, buf);
    qtest_writel(qts, desc + 8, 0);
    qtest_writel(qts, AHB_IN_LINK_ADDR(ch), desc);
    qtest_writel(qts, AHB_IN(ch) + G_LINK, IN_LINK_START);
}

static void fill(QTestState *qts, uint32_t addr, const uint8_t *data,
                 size_t len)
{
    qtest_memwrite(qts, addr, data, len);
}

static bool src_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

/* CONF's reset values, but for BCK_DIV_NUM */
#define TX_BASE     0x0000f210u
#define RX_BASE     0x0000b600u

/* Standard Philips: WS low for the left channel, 16 bits, one late */
#define STD_CONF1   CONF1(16, 16, 16, 16)
#define STD_TDM     TDM(2, 0x3)

/* ---- Registers ---- */

/* [spec:nuos:req:emu.esp32p4.i2s/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    for (int n = 0; n < 3; n++) {
        clocks_on(qts, n);
        g_assert_cmphex(i2s_r(qts, n, INT_RAW), ==, 0);
        g_assert_cmphex(i2s_r(qts, n, RX_CONF), ==, 0x00c0b600);
        g_assert_cmphex(i2s_r(qts, n, TX_CONF), ==, 0x00c0f210);
        g_assert_cmphex(i2s_r(qts, n, RX_CONF1), ==, 0x787bc000);
        g_assert_cmphex(i2s_r(qts, n, TX_CONF1), ==, 0x787bc000);
        g_assert_cmphex(i2s_r(qts, n, TX_PCM2PDM_CONF), ==, 0x004aa004);
        g_assert_cmphex(i2s_r(qts, n, TX_PCM2PDM_CONF1), ==, 0x03f783c0);
        g_assert_cmphex(i2s_r(qts, n, RX_PDM2PCM_CONF), ==, 0xf8200000);
        g_assert_cmphex(i2s_r(qts, n, RX_TDM_CTRL), ==, 0xffff);
        g_assert_cmphex(i2s_r(qts, n, TX_TDM_CTRL), ==, 0xffff);
        g_assert_cmphex(i2s_r(qts, n, LC_HUNG_CONF), ==, 0x810);
        g_assert_cmphex(i2s_r(qts, n, RXEOF_NUM), ==, 0x40);
        g_assert_cmphex(i2s_r(qts, n, STATE), ==, 1);
        g_assert_cmphex(i2s_r(qts, n, ETM_CONF), ==, 0x10040);
        g_assert_cmphex(i2s_r(qts, n, DATE), ==, 0x02303240);
    }
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_CONF), ==, 0x9600);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_CONF1), ==, 0x2f3de300);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_MEM_CONF), ==, 0x7e00);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_TDM_CTRL), ==, 3);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_PDM_CONF), ==,
                    0xf8200000);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_ECO_HIGH), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_VAD_PARAM(0)), ==,
                    0x00c81388);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_VAD_PARAM(8)), ==,
                    0x20005040);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_CLK_GATE), ==, 0xa);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_DATE), ==, 0x02305040);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * Access types: reserved bits, the WT reset bits reading 0, INT_RAW's
 * write-1-to-clear, the counters' reset bits, the RO registers; and a
 * gated bus clock dropping writes.
 */
static void test_access(void)
{
    QTestState *qts = start();

    /* Gated: the write is dropped. */
    i2s_w(qts, 1, TX_CONF1, 0);
    clocks_on(qts, 1);
    g_assert_cmphex(i2s_r(qts, 1, TX_CONF1), ==, 0x787bc000);

    i2s_w(qts, 1, RX_CONF, 0xffffffff & ~C_UPDATE & ~C_START);
    g_assert_cmphex(i2s_r(qts, 1, RX_CONF), ==,
                    0x07ffbffc & ~C_START & ~C_UPDATE);
    i2s_w(qts, 1, TX_CONF, 0x80000003);
    g_assert_cmphex(i2s_r(qts, 1, TX_CONF), ==, 0);
    i2s_w(qts, 1, RX_CONF1, 0xffffffff);
    g_assert_cmphex(i2s_r(qts, 1, RX_CONF1), ==, 0xffffc1ff);
    i2s_w(qts, 1, TX_TDM_CTRL, 0xffffffff);
    g_assert_cmphex(i2s_r(qts, 1, TX_TDM_CTRL), ==, 0x1fffff);
    i2s_w(qts, 1, RX_TIMING, 0xffffffff);
    g_assert_cmphex(i2s_r(qts, 1, RX_TIMING), ==, 0x33333333);
    i2s_w(qts, 1, TX_TIMING, 0xffffffff);
    g_assert_cmphex(i2s_r(qts, 1, TX_TIMING), ==, 0x33330033);
    i2s_w(qts, 1, STATE, 0);
    g_assert_cmphex(i2s_r(qts, 1, STATE), ==, 1);
    i2s_w(qts, 1, FIFO_CNT, 0x7fffffff);
    g_assert_cmphex(i2s_r(qts, 1, FIFO_CNT), ==, 0);
    i2s_w(qts, 1, INT_CLR, 0xf);
    g_assert_cmphex(i2s_r(qts, 1, INT_CLR), ==, 0);
    i2s_w(qts, 1, DATE, 0xffffffff);
    g_assert_cmphex(i2s_r(qts, 1, DATE), ==, 0x0fffffff);

    /* The LP I2S: RX_MEM_FIFO_CNT is read-only. */
    qtest_writel(qts, LP_I2S + LP_RX_MEM_CONF, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_MEM_CONF), ==, 0x1fe00);
    qtest_writel(qts, LP_I2S + LP_RX_CONF, 0xffffffff & ~(1u << 8));
    g_assert_cmphex(qtest_readl(qts, LP_I2S + LP_RX_CONF), ==,
                    0x001ffeac);
    qtest_writel(qts, LP_I2S + LP_RX_CONF, 0x9600);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * TX_UPDATE carries the configuration into the unit's clock domain: it
 * waits while the TX clock is stopped, and clears as the clock starts;
 * a unit started without it works from the configuration last carried.
 */
static void test_update(void)
{
    QTestState *qts = start();
    const UnitClk *c = &unit_clk[2][1];

    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) | I2S_APB_CLK_EN(2));
    i2s_w(qts, 2, TX_CONF, 0x00c0f210 | C_UPDATE);
    g_assert_cmphex(i2s_r(qts, 2, TX_CONF) & C_UPDATE, ==, C_UPDATE);
    set_unit_clk(qts, 2, 1, SRC_XTAL, 5);
    g_assert_cmphex(i2s_r(qts, 2, TX_CONF) & C_UPDATE, ==, 0);

    /* Started in TDM mode without the update: neither mode, so idle. */
    i2s_w(qts, 2, TX_CONF, 0x00c0f210 | C_TDM_EN | C_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(i2s_r(qts, 2, STATE), ==, 1);
    g_assert_cmphex(i2s_r(qts, 2, BCK_CNT), ==, 0);
    set_fld(qts, c->en, 0);
    set_fld(qts, c->en, 1);
    qtest_quit(qts);
}

/* ---- Transfers ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * Standard Philips mode: I2S0 transmits as master, I2S1 receives as slave
 * through three pads, both through GDMA-AHB; every sample arrives. The
 * receiver ends its RX_EOF_NUM run with GDMA's IN_SUC_EOF and RX_DONE;
 * the master, out of data with TX_STOP_EN, stops with TX_DONE and goes
 * idle.
 */
static void test_std_loopback(void)
{
    QTestState *qts = start();
    uint8_t tx[128], rx[128];

    for (int i = 0; i < sizeof(tx); i++) {
        tx[i] = i * 37 + 5;
    }
    fill(qts, MEM + 0x1000, tx, sizeof(tx));
    clocks_on(qts, 0);
    clocks_on(qts, 1);
    wire(qts, 20, SIG_O_BCK(0), SIG_I_BCK(1));
    wire(qts, 21, SIG_O_WS(0), SIG_I_WS(1));
    wire(qts, 22, SIG_SD(0), SIG_SD(1));

    i2s_w(qts, 1, RXEOF_NUM, sizeof(tx) / 2 - 1);
    unit_go(qts, 1, false, RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1,
            STD_TDM);
    dma_rx(qts, 1, PERI_I2S(1), MEM, MEM + 0x2000, sizeof(rx));
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8));
    g_assert_cmphex(i2s_r(qts, 0, STATE), ==, 0);

    /* 32 frames of 32 BCK cycles at 1 MHz */
    qtest_clock_step(qts, 2 * 1000 * 1000);
    qtest_memread(qts, MEM + 0x2000, rx, sizeof(rx));
    g_assert_cmpmem(rx, sizeof(rx), tx, sizeof(tx));
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + G_RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, INT_RX_DONE);
    g_assert_cmphex(i2s_r(qts, 0, INT_RAW) & INT_TX_DONE, ==, INT_TX_DONE);
    g_assert_cmphex(i2s_r(qts, 0, STATE), ==, 1);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, sizeof(tx) / 2);
    /* With the cycle that carries the last frame's last bit. */
    g_assert_cmpuint(i2s_r(qts, 0, BCK_CNT), ==, sizeof(tx) / 4 * 32 + 1);

    /* INT_RAW is write-1-to-clear; INT_CLR clears too. */
    i2s_w(qts, 0, INT_RAW, INT_TX_DONE);
    g_assert_cmphex(i2s_r(qts, 0, INT_RAW) & INT_TX_DONE, ==, 0);
    i2s_w(qts, 1, INT_CLR, INT_RX_DONE);
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, 0);
    qtest_quit(qts);
}

/*
 * Run a transfer: unit @rx_n's receiver, then unit @tx_n's transmitter
 * started, @tx_len bytes of @tx through GDMA-AHB channel 0 and @rx_len
 * received through channel 1, for @us microseconds.
 */
static void transfer(QTestState *qts, int tx_n, uint32_t tx_conf,
                     uint32_t tx_conf1, uint32_t tx_tdm, int rx_n,
                     uint32_t rx_conf, uint32_t rx_conf1, uint32_t rx_tdm,
                     const uint8_t *tx, size_t tx_len, uint8_t *rx,
                     size_t rx_len, uint32_t rx_bytes, int us)
{
    fill(qts, MEM + 0x1000, tx, tx_len);
    i2s_w(qts, rx_n, RXEOF_NUM, rx_len / rx_bytes - 1);
    unit_go(qts, rx_n, false, rx_conf, rx_conf1, rx_tdm);
    dma_rx(qts, 1, PERI_I2S(rx_n), MEM, MEM + 0x2000, rx_len);
    unit_setup(qts, tx_n, true, tx_conf, tx_conf1, tx_tdm);
    dma_tx(qts, 0, PERI_I2S(tx_n), MEM + 0x10, MEM + 0x1000, tx_len);
    qtest_clock_step(qts, 1000);
    unit_start(qts, tx_n, true, tx_conf);
    qtest_clock_step(qts, (int64_t)us * 1000);
    qtest_memread(qts, MEM + 0x2000, rx, rx_len);
}

/* I2S @a's transmitter to I2S @b's receiver, slave, through pads 20-22. */
static void wire_tx_to_rx(QTestState *qts, int a, int b)
{
    wire(qts, 20, SIG_O_BCK(a), SIG_I_BCK(b));
    wire(qts, 21, SIG_O_WS(a), SIG_I_WS(b));
    wire(qts, 22, SIG_SD(a), SIG_SD(b));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * TDM, PCM short frame sync (WS high for one cycle before the frame),
 * four 16-bit channels in a 64-cycle frame: a disabled TX channel sends
 * CONF_SINGLE_DATA, or with TX_CHAN_EQUAL the channel before it, and
 * with TX_TDM_SKIP_MSK_EN still takes its data from memory; a disabled
 * RX channel is not stored.
 */
static void test_tdm_slots(void)
{
    static const struct {
        uint32_t tx_extra;
        uint32_t tx_skip;
        uint32_t rx_mask;
    } runs[] = {
        { 0, 0, 0xf },
        { C_TX_CHAN_EQUAL, 0, 0xf },
        { 0, TDM_SKIP_MSK, 0xf },
        { 0, 0, 0xb },
    };
    uint32_t conf1 = CONF1(1, 16, 32, 16);
    uint16_t tx[32], rx[32], want[32];

    for (int r = 0; r < ARRAY_SIZE(runs); r++) {
        QTestState *qts = start();
        int frames = 8, per = runs[r].tx_skip ? 4 : 3, nw = 0, k = 0;
        int nrx = frames * (runs[r].rx_mask == 0xf ? 4 : 3);

        for (int i = 0; i < 32; i++) {
            tx[i] = 0x1000 + i * 0x111;
        }
        for (int f = 0; f < frames; f++) {
            const uint16_t *d = &tx[f * per];
            uint16_t slot[4];

            slot[0] = d[0];
            slot[1] = d[1];
            slot[2] = runs[r].tx_extra ? d[1] : 0xabcd;
            slot[3] = runs[r].tx_skip ? d[3] : d[2];
            for (int c = 0; c < 4; c++) {
                if (runs[r].rx_mask & (1 << c)) {
                    want[nw++] = slot[c];
                }
            }
        }
        clocks_on(qts, 2);
        clocks_on(qts, 0);
        wire_tx_to_rx(qts, 2, 0);
        i2s_w(qts, 2, SINGLE_DATA, 0x5555abcd);
        transfer(qts, 2, TX_BASE | C_TDM_EN | C_BCK_DIV(8) | C_WS_IDLE_POL |
                 runs[r].tx_extra, conf1, TDM(4, 0xb) | runs[r].tx_skip,
                 0, RX_BASE | C_TDM_EN | C_SLAVE | C_WS_IDLE_POL, conf1,
                 TDM(4, runs[r].rx_mask), (uint8_t *)tx,
                 frames * per * 2, (uint8_t *)rx, nrx * 2, 2, 1000);
        for (k = 0; k < nrx; k++) {
            g_assert_cmphex(rx[k], ==, want[k]);
        }
        g_assert_cmphex(i2s_r(qts, 0, INT_RAW) & INT_RX_DONE, ==,
                        INT_RX_DONE);
        qtest_quit(qts);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * Data formats: 24-bit data packed in three bytes in 32-bit channels,
 * left-aligned; the transmitter's BIG_ENDIAN byte order and BIT_ORDER
 * as the receiver sees them; MSB alignment (no MSB_SHIFT).
 */
static void test_formats(void)
{
    QTestState *qts = start();
    uint8_t tx[48], rx[48];
    uint16_t tx16[16], rx16[16];

    for (int i = 0; i < sizeof(tx); i++) {
        tx[i] = i * 29 + 3;
    }
    clocks_on(qts, 1);
    clocks_on(qts, 2);
    wire_tx_to_rx(qts, 1, 2);
    transfer(qts, 1, TX_BASE | C_TDM_EN | C_BCK_DIV(8),
             CONF1(32, 24, 32, 32), STD_TDM, 2,
             RX_BASE | C_TDM_EN | C_SLAVE, CONF1(32, 24, 32, 32), STD_TDM,
             tx, sizeof(tx), rx, sizeof(rx), 3, 1000);
    g_assert_cmpmem(rx, sizeof(rx), tx, sizeof(tx));
    qtest_quit(qts);

    /* BIG_ENDIAN swaps each 16-bit sample's bytes as it is read. */
    qts = start();
    for (int i = 0; i < 16; i++) {
        tx16[i] = 0x1234 + i * 0x0f0f;
    }
    clocks_on(qts, 1);
    clocks_on(qts, 2);
    wire_tx_to_rx(qts, 1, 2);
    transfer(qts, 1, (TX_BASE & ~C_MSB_SHIFT) | C_TDM_EN | C_BCK_DIV(8) |
             C_BIG_ENDIAN, STD_CONF1, STD_TDM, 2,
             (RX_BASE & ~C_MSB_SHIFT) | C_TDM_EN | C_SLAVE, STD_CONF1,
             STD_TDM, (uint8_t *)tx16, sizeof(tx16), (uint8_t *)rx16,
             sizeof(rx16), 2, 1000);
    for (int i = 0; i < 16; i++) {
        g_assert_cmphex(rx16[i], ==, bswap16(tx16[i]));
    }
    qtest_quit(qts);

    /* BIT_ORDER sends LSB first: the receiver stores it reversed. */
    qts = start();
    clocks_on(qts, 1);
    clocks_on(qts, 2);
    wire_tx_to_rx(qts, 1, 2);
    transfer(qts, 1, TX_BASE | C_TDM_EN | C_BCK_DIV(8) | C_BIT_ORDER,
             STD_CONF1, STD_TDM, 2, RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1,
             STD_TDM, (uint8_t *)tx16, sizeof(tx16), (uint8_t *)rx16,
             sizeof(rx16), 2, 1000);
    for (int i = 0; i < 16; i++) {
        uint16_t v = 0;

        for (int b = 0; b < 16; b++) {
            v |= ((tx16[i] >> b) & 1) << (15 - b);
        }
        g_assert_cmphex(rx16[i], ==, v);
    }
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * A master receiver clocks a slave transmitter: I2S1's RX drives BCK and
 * WS on I2SnI_BCK_out and _WS_out, I2S0's TX follows them on its
 * I2SnO_BCK_in and _WS_in.
 */
static void test_rx_master(void)
{
    QTestState *qts = start();
    uint8_t tx[64], rx[64];

    for (int i = 0; i < sizeof(tx); i++) {
        tx[i] = 0xa5 ^ (i * 13);
    }
    fill(qts, MEM + 0x1000, tx, sizeof(tx));
    clocks_on(qts, 0);
    clocks_on(qts, 1);
    wire(qts, 20, SIG_I_BCK(1), SIG_O_BCK(0));
    wire(qts, 21, SIG_I_WS(1), SIG_O_WS(0));
    wire(qts, 22, SIG_SD(0), SIG_SD(1));
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_SLAVE);
    i2s_w(qts, 1, RXEOF_NUM, sizeof(rx) / 2 - 1);
    unit_setup(qts, 1, false, RX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1,
               STD_TDM);
    dma_rx(qts, 1, PERI_I2S(1), MEM, MEM + 0x2000, sizeof(rx));
    unit_start(qts, 1, false, RX_BASE | C_TDM_EN | C_BCK_DIV(8));
    qtest_clock_step(qts, 1000 * 1000);
    qtest_memread(qts, MEM + 0x2000, rx, sizeof(rx));
    g_assert_cmpmem(rx, sizeof(rx), tx, sizeof(tx));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * SIG_LOOPBACK: I2S2's receiver shares its transmitter's BCK and WS, with
 * no pad for them; the data goes out and back in through one pad.
 */
static void test_sig_loopback(void)
{
    QTestState *qts = start();
    uint8_t tx[64], rx[64];

    for (int i = 0; i < sizeof(tx); i++) {
        tx[i] = i * 7 + 100;
    }
    clocks_on(qts, 2);
    wire(qts, 23, SIG_SD(2), SIG_SD(2));
    transfer(qts, 2, TX_BASE | C_TDM_EN | C_BCK_DIV(8) | C_SIG_LOOPBACK,
             STD_CONF1, STD_TDM, 2, RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1,
             STD_TDM, tx, sizeof(tx), rx, sizeof(rx), 2, 1000);
    g_assert_cmpmem(rx, sizeof(rx), tx, sizeof(tx));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The sample rate: XTAL / 5 = 8 MHz, BCK / 8 = 1 MHz, 32 BCK cycles a
 * frame: a frame every 32 us, each taking two channels' data; the counters
 * follow exactly. A fractional divider N + b/a (4 + 1/2) speeds it in
 * proportion.
 */
static void test_timing(void)
{
    QTestState *qts = start();
    uint8_t tx[1024];

    memset(tx, 0x5a, sizeof(tx));
    clocks_on(qts, 0);
    fill(qts, MEM + 0x1000, tx, sizeof(tx));
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8));
    qtest_clock_step(qts, 32000 * 10 - 1);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, 18);
    g_assert_cmpuint(i2s_r(qts, 0, BCK_CNT), ==, 9 * 32);
    qtest_clock_step(qts, 1);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, 20);
    g_assert_cmpuint(i2s_r(qts, 0, BCK_CNT), ==, 10 * 32);
    qtest_clock_step(qts, 32000 * 100);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, 220);

    /* N = 4, b/a = 1/2: x = 1, y = 0, z = 2; 8.889 MHz, 28.8 us frames */
    i2s_w(qts, 0, FIFO_CNT, 1u << 31);
    set_fld(qts, unit_clk[0][1].x, 1);
    set_fld(qts, unit_clk[0][1].y, 0);
    set_fld(qts, unit_clk[0][1].z, 2);
    set_fld(qts, unit_clk[0][1].n, 4);
    qtest_clock_step(qts, 28800 * 10 - 1);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, 18);
    qtest_clock_step(qts, 1);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, 20);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * Raw PDM: I2S1 transmits two channels' 16-bit words, I2S2 receives them
 * as a slave on the PDM clock (WS) alone; every bit arrives.
 */
static void test_pdm_raw(void)
{
    QTestState *qts = start();
    uint16_t tx[32], rx[32];
    uint32_t conf1 = CONF1(16, 16, 16, 16);

    for (int i = 0; i < 32; i++) {
        tx[i] = 0x8001 ^ (i * 0x1357);
    }
    clocks_on(qts, 1);
    clocks_on(qts, 2);
    wire(qts, 21, SIG_O_WS(1), SIG_I_WS(2));
    wire(qts, 22, SIG_SD(1), SIG_SD(2));
    transfer(qts, 1, TX_BASE | C_PDM_EN | C_BCK_DIV(8), conf1, STD_TDM, 2,
             RX_BASE | C_PDM_EN | C_SLAVE, conf1, TDM(2, 0x3),
             (uint8_t *)tx, sizeof(tx), (uint8_t *)rx, sizeof(rx), 2, 1000);
    for (int i = 0; i < 32; i++) {
        g_assert_cmphex(rx[i], ==, tx[i]);
    }
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * I2S0's PCM-to-PDM converter to its PDM-to-PCM converter, sharing the
 * PDM clock with SIG_LOOPBACK, the bits through a pad: a square wave of
 * 32 samples comes back, after the decimator's delay, close to what was
 * sent away from its edges.
 */
static void test_pdm_conv(void)
{
    QTestState *qts = start();
    int16_t tx[256], rx[256];
    uint32_t conf1 = CONF1(16, 16, 16, 16);
    int best = 0, best_d = 0;

    for (int i = 0; i < 128; i++) {
        int16_t v = (i / 16) % 2 ? -12000 : 12000;

        tx[2 * i] = v;
        tx[2 * i + 1] = -v;
    }
    clocks_on(qts, 0);
    wire(qts, 23, SIG_SD(0), SIG_SD(0));
    i2s_w(qts, 0, TX_PCM2PDM_CONF, (0x004aa004 & ~(0xfu << 1)) |
          P2D_OSR2(1) | P2D_CONV_EN);
    i2s_w(qts, 0, RX_PDM2PCM_CONF, 0xf8200000 | D2P_EN);
    transfer(qts, 0, TX_BASE | C_PDM_EN | C_BCK_DIV(8) | C_SIG_LOOPBACK,
             conf1, STD_TDM, 0, RX_BASE | C_PDM_EN | C_SLAVE, conf1,
             TDM(2, 0x3), (uint8_t *)tx, sizeof(tx), (uint8_t *)rx,
             sizeof(rx), 2, 20000);
    for (int d = 0; d < 6; d++) {
        int good = 0;

        for (int i = 32; i + d < 128; i++) {
            int16_t want = tx[2 * i], got = rx[2 * (i + d)];

            if (i % 16 > 3 && abs(got - want) < 1500 &&
                abs(rx[2 * (i + d) + 1] + want) < 1500) {
                good++;
            }
        }
        if (good > best) {
            best = good;
            best_d = d;
        }
    }
    g_assert_cmpint(best_d, <=, 3);
    /* Of the 96 samples checked, the 72 away from the edges */
    g_assert_cmpint(best, >=, 70);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The A-law and u-law units: the transmitter compresses 16-bit samples,
 * the receiver stores the codes; then it expands them.
 */
static void test_alaw(void)
{
    static const int16_t pcm[8] = {
        0, -1, 1000, -1000, 32767, -32768, 12345, 0,
    };
    static const uint16_t alaw[8] = {
        0xd5, 0x55, 0xfa, 0x7a, 0xaa, 0x2a, 0xbd, 0xd5,
    };
    static const uint16_t ulaw[8] = {
        0xff, 0x7e, 0xce, 0x4e, 0x80, 0x00, 0x97, 0xff,
    };
    static const int16_t alaw_back[8] = {
        8, -8, 1008, -1008, 32256, -32256, 12544, 8,
    };
    uint16_t rx[8];
    int16_t back[8];

    for (int law = 0; law < 2; law++) {
        QTestState *qts = start();

        clocks_on(qts, 0);
        clocks_on(qts, 1);
        wire_tx_to_rx(qts, 0, 1);
        transfer(qts, 0, (TX_BASE & ~C_PCM_BYPASS) | C_PCM_CONF(law ? 3 : 1) |
                 C_TDM_EN | C_BCK_DIV(8), STD_CONF1, STD_TDM, 1,
                 RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1, STD_TDM,
                 (const uint8_t *)pcm, sizeof(pcm), (uint8_t *)rx,
                 sizeof(rx), 2, 1000);
        for (int i = 0; i < 8; i++) {
            g_assert_cmphex(rx[i], ==, law ? ulaw[i] : alaw[i]);
        }
        qtest_quit(qts);
    }

    /* The receiver expands A-law codes. */
    {
        QTestState *qts = start();

        clocks_on(qts, 0);
        clocks_on(qts, 1);
        wire_tx_to_rx(qts, 0, 1);
        transfer(qts, 0, TX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1,
                 STD_TDM, 1, (RX_BASE & ~C_PCM_BYPASS & ~C_PCM_CONF(3)) |
                 C_PCM_CONF(0) | C_TDM_EN | C_SLAVE, STD_CONF1, STD_TDM,
                 (const uint8_t *)alaw, sizeof(alaw), (uint8_t *)back,
                 sizeof(back), 2, 1000);
        for (int i = 0; i < 8; i++) {
            g_assert_cmpint(back[i], ==, alaw_back[i]);
        }
        qtest_quit(qts);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * LC_HUNG_CONF's timeout: a slave transmitter with data and no BCK raises
 * TX_HUNG, a slave receiver receiving nothing RX_HUNG, after TIMEOUT
 * ticks of 88000 >> SHIFT bus clock cycles; and the interrupt reaches the
 * interrupt matrix as I2Sn_INTR while enabled.
 */
static void test_hung(void)
{
    QTestState *qts = start();
    uint8_t tx[64] = { 0 };

    clocks_on(qts, 0);
    fill(qts, MEM + 0x1000, tx, sizeof(tx));
    /* 2 ticks of 88000 >> 7 = 687 cycles */
    i2s_w(qts, 0, LC_HUNG_CONF, (1u << 11) | (7u << 8) | 2);
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_SLAVE);
    unit_go(qts, 0, false, RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1, STD_TDM);
    qtest_clock_step(qts, 10 * 1000);
    g_assert_cmphex(i2s_r(qts, 0, INT_RAW) & (INT_TX_HUNG | INT_RX_HUNG), ==,
                    0);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(i2s_r(qts, 0, INT_RAW) & (INT_TX_HUNG | INT_RX_HUNG), ==,
                    INT_TX_HUNG | INT_RX_HUNG);
    g_assert_false(src_level(qts, SRC_I2S(0)));
    i2s_w(qts, 0, INT_ENA, INT_TX_HUNG);
    g_assert_cmphex(i2s_r(qts, 0, INT_ST), ==, INT_TX_HUNG);
    g_assert_true(src_level(qts, SRC_I2S(0)));
    i2s_w(qts, 0, INT_CLR, INT_TX_HUNG);
    g_assert_false(src_level(qts, SRC_I2S(0)));
    /* Once per stall */
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(i2s_r(qts, 0, INT_RAW) & INT_TX_HUNG, ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The ETM: I2S0_EVT_X_WORDS_SENT, at ETM_TX_SEND_WORD_NUM words, through
 * a channel to I2S0_TASK_STOP_TX, which clears TX_START.
 */
static void test_etm(void)
{
    QTestState *qts = start();
    uint8_t tx[256];

    memset(tx, 0x33, sizeof(tx));
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | (1u << 24));
    qtest_writel(qts, SOC_CLK_CTRL3,
                 qtest_readl(qts, SOC_CLK_CTRL3) | (1u << 2));
    qtest_writel(qts, ETM_CH_EVT_ID(0), ETM_I2S_EVT_X_WORDS_SENT(0));
    qtest_writel(qts, ETM_CH_TASK_ID(0), ETM_I2S_TASK_STOP_TX(0));
    qtest_writel(qts, ETM_CH_ENA_SET, 1);
    clocks_on(qts, 0);
    i2s_w(qts, 0, ETM_CONF, 10);
    fill(qts, MEM + 0x1000, tx, sizeof(tx));
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8));
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmpuint(i2s_r(qts, 0, FIFO_CNT), ==, 10);
    g_assert_cmphex(i2s_r(qts, 0, TX_CONF) & C_START, ==, 0);
    g_assert_cmphex(i2s_r(qts, 0, STATE), ==, 1);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * I2Sn_MCLK_in from another controller's I2Sn_MCLK_out through a pad:
 * I2S1's receiver runs from PAD_I2S1_MCLK, I2S0's TX clock (8 MHz) that
 * I2S0_MCLK_out carries, and its frames come at that rate.
 */
static void test_mclk(void)
{
    QTestState *qts = start();
    uint8_t rx[64];

    clocks_on(qts, 0);
    clocks_on(qts, 1);
    set_fld(qts, mst_sel[0], 1);
    wire(qts, 24, SIG_MCLK(0), SIG_MCLK(1));
    qtest_writel(qts, LP_HP_CLK_CTRL,
                 qtest_readl(qts, LP_HP_CLK_CTRL) | PAD_I2S_MCLK_EN(1));
    set_unit_clk(qts, 1, 0, SRC_PAD_MCLK, 2);
    /*
     * I2S1 RX master at 8 MHz / 2 / 4 = 1 MHz: a frame every 32 us, its
     * channels MSB-aligned, so that each frame's data is in by its end.
     */
    i2s_w(qts, 1, RXEOF_NUM, 7);
    unit_go(qts, 1, false, (RX_BASE & ~C_MSB_SHIFT) | C_TDM_EN |
            C_BCK_DIV(4), STD_CONF1, STD_TDM);
    dma_rx(qts, 1, PERI_I2S(1), MEM, MEM + 0x2000, sizeof(rx));
    qtest_clock_step(qts, 32000 * 4 - 1);
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, INT_RX_DONE);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + G_RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);
    qtest_quit(qts);
}

/* A clock's period, in Clock units of 2^-32 ns. */
static uint64_t clock_period(QTestState *qts, const char *path)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         path);
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    return period;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * [spec:nuos:req:emu.esp32p4.clocks/test]
 * A clock through a pad loop keeps its exact period: I2S0's TX clock,
 * XTAL_CLK by 4 + 2/3, 350/3 ns, which is no whole number of Clock units,
 * goes out on I2S0_MCLK_out and back in to PAD_I2S1_MCLK; I2S1's receiver
 * divides it by 2, 700/3 ns, a Clock period of 700/3 * 2^32 rounded down
 * once (not twice I2S0's rounded period). Its frames, 4 BCK cycles of 8
 * of those and 32 BCK cycles a frame, are 89600/3 ns: the 100th ends at
 * 2986666 2/3 ns after the start, and RX_DONE comes at the first
 * nanosecond after.
 */
static void test_mclk_exact(void)
{
    QTestState *qts = start();
    int64_t t0;

    clocks_on(qts, 0);
    clocks_on(qts, 1);
    /* N 4, b/a 2/3: x 0, y 1, z 2 */
    set_fld(qts, unit_clk[0][1].x, 0);
    set_fld(qts, unit_clk[0][1].y, 1);
    set_fld(qts, unit_clk[0][1].z, 2);
    set_fld(qts, unit_clk[0][1].n, 4);
    set_fld(qts, mst_sel[0], 1);
    wire(qts, 24, SIG_MCLK(0), SIG_MCLK(1));
    qtest_writel(qts, LP_HP_CLK_CTRL,
                 qtest_readl(qts, LP_HP_CLK_CTRL) | PAD_I2S_MCLK_EN(1));
    set_unit_clk(qts, 1, 0, SRC_PAD_MCLK, 2);
    g_assert_cmpuint(clock_period(qts, "/machine/clkrst/i2s1-rx"), ==,
                     700ull * (1ull << 32) / 3);

    i2s_w(qts, 1, RXEOF_NUM, 2 * 100 - 1);
    dma_rx(qts, 1, PERI_I2S(1), MEM, MEM + 0x2000, 1024);
    t0 = qtest_clock_step(qts, 1);
    unit_go(qts, 1, false, (RX_BASE & ~C_MSB_SHIFT) | C_TDM_EN |
            C_BCK_DIV(4), STD_CONF1, STD_TDM);
    g_assert_cmpint(qtest_clock_step(qts, 2986666), ==, t0 + 2986666);
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, INT_RX_DONE);
    qtest_quit(qts);
}

/* ---- The LP I2S and its VAD ---- */

static void lp_w(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, LP_I2S + reg, v);
}

static uint32_t lp_r(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, LP_I2S + reg);
}

/* A VAD frame of 256 samples into the memory: silence or a loud tone. */
static void lp_frame(QTestState *qts, bool loud)
{
    for (int i = 0; i < 128; i++) {
        int16_t a = loud ? ((i % 8) < 4 ? 8000 : -8000) : (i % 2 ? 3 : -3);
        int16_t b = loud ? -a : a;

        lp_w(qts, LP_RAM, (uint16_t)a | (uint32_t)(uint16_t)b << 16);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The LP I2S memory read through LP_I2S_RAM_BASE, oldest first, with
 * RX_MEM_FIFO_CNT, RX_MEM_THRESHOLD_INT and RX_FIFOMEM_UDF_INT, and its
 * overwriting of the oldest word when full.
 */
static void test_lp_mem(void)
{
    QTestState *qts = start();

    lp_w(qts, LP_RX_MEM_CONF, 4u << 9);
    for (uint32_t i = 0; i < 4; i++) {
        lp_w(qts, LP_RAM, 0x1000 + i);
    }
    g_assert_cmphex(lp_r(qts, LP_INT_RAW), ==, 0);
    lp_w(qts, LP_RAM, 0x1004);
    g_assert_cmphex(lp_r(qts, LP_INT_RAW), ==, LP_INT_THRESHOLD);
    g_assert_cmphex(lp_r(qts, LP_RX_MEM_CONF) & 0x1ff, ==, 5);
    for (uint32_t i = 0; i < 5; i++) {
        g_assert_cmphex(lp_r(qts, LP_RAM), ==, 0x1000 + i);
    }
    g_assert_cmphex(lp_r(qts, LP_RX_MEM_CONF) & 0x1ff, ==, 0);
    lp_r(qts, LP_RAM);
    g_assert_cmphex(lp_r(qts, LP_INT_RAW) & LP_INT_UDF, ==, LP_INT_UDF);

    /* 300 words into 256: the first 44 are overwritten. */
    for (uint32_t i = 0; i < 300; i++) {
        lp_w(qts, LP_RAM, i);
    }
    g_assert_cmphex(lp_r(qts, LP_RX_MEM_CONF) & 0x1ff, ==, 256);
    g_assert_cmphex(lp_r(qts, LP_RAM), ==, 44);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The VAD in manual mode: frames written to the memory and processed with
 * VAD_FORCE_START. Two quiet initial frames learn the noise; a quiet
 * frame then is not speech; loud frames are, and after
 * PARAM_HANGOVER_SPEECH + 1 of them VAD_FLAG rises; frames below
 * PARAM_MIN_ENERGY are not speech and, past PARAM_HANGOVER_SILENT, VAD_FLAG
 * falls. VAD_DONE marks each frame, VAD_RESET_DONE the reset; VAD_FLAG,
 * LP_I2S_WAKEUP, reaches the PMU as its wakeup source 19.
 */
static void test_vad_manual(void)
{
    QTestState *qts = start();
    uint32_t p1;

    /* Two initial frames */
    lp_w(qts, LP_VAD_PARAM(0), 2u << 16 | 5000);
    /* MIN_SPEECH 1, MAX_SPEECH 100, HANGOVER_SPEECH 2, HANGOVER_SILENT 3 */
    p1 = 1 | 100u << 4 | 2u << 11 | 3u << 16 | 40u << 24;
    lp_w(qts, LP_VAD_PARAM(1), p1);
    lp_w(qts, LP_VAD_CONF, 1u << 1);
    g_assert_cmphex(lp_r(qts, LP_INT_RAW) & LP_INT_VAD_RESET, ==,
                    LP_INT_VAD_RESET);
    for (int i = 0; i < 2; i++) {
        lp_frame(qts, false);
        lp_w(qts, LP_VAD_CONF, 1u << 2);
    }
    g_assert_cmphex(lp_r(qts, LP_INT_RAW) & LP_INT_VAD_DONE, ==,
                    LP_INT_VAD_DONE);
    lp_frame(qts, false);
    lp_w(qts, LP_VAD_CONF, 1u << 2);
    g_assert_cmphex(lp_r(qts, LP_VAD_RESULT), ==, 0);
    for (int i = 0; i < 3; i++) {
        g_assert_cmphex(lp_r(qts, LP_VAD_RESULT) & 1, ==, 0);
        lp_frame(qts, true);
        lp_w(qts, LP_VAD_CONF, 1u << 2);
        g_assert_cmphex(lp_r(qts, LP_VAD_RESULT) & 2, ==, 2);
        g_assert_cmphex(lp_r(qts, LP_VAD_OB(0)) & 0xff, ==, i + 1);
    }
    g_assert_cmphex(lp_r(qts, LP_VAD_RESULT), ==, 3);
    /*
     * VAD_FLAG is LP_I2S_WAKEUP, the PMU's wakeup source 19: enabled to
     * reject sleep, it rejects a sleep request.
     */
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL1, (1u << 31) | (1u << 19));
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL0, 1u << 31);
    g_assert_cmphex(qtest_readl(qts, PMU_SLP_WAKEUP_STATUS1), ==, 1u << 19);
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL1, 0);
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL4, 1u << 31);
    g_assert_cmphex(lp_r(qts, LP_VAD_OB(0)) >> 16, ==, 8000);
    /* Energy of the frame: 256 * 8000^2 */
    g_assert_cmphex(lp_r(qts, LP_VAD_OB(7)), ==,
                    (uint32_t)(256ull * 8000 * 8000));
    g_assert_cmphex(lp_r(qts, LP_VAD_OB(8)), ==,
                    (uint32_t)((256ull * 8000 * 8000) >> 32));
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(lp_r(qts, LP_VAD_RESULT) & 1, ==, 1);
        lp_frame(qts, false);
        lp_w(qts, LP_VAD_CONF, 1u << 2);
        g_assert_cmphex(lp_r(qts, LP_VAD_RESULT) & 2, ==, 0);
    }
    g_assert_cmphex(lp_r(qts, LP_VAD_RESULT), ==, 0);

    /* The LP I2S interrupt through the matrix */
    g_assert_false(src_level(qts, SRC_LP_I2S));
    lp_w(qts, LP_INT_ENA, LP_INT_VAD_DONE);
    g_assert_true(src_level(qts, SRC_LP_I2S));
    lp_w(qts, LP_INT_RAW, LP_INT_VAD_DONE);
    g_assert_false(src_level(qts, SRC_LP_I2S));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The LP I2S receiving as a slave from I2S0's transmitter through pads
 * the HP matrix drives and the LP matrix reads, 16-bit mono samples into
 * its memory; the VAD, enabled, takes each 256 samples by itself and
 * finds the speech; RX_DONE ends each RX_EOF_NUM run.
 */
static void test_lp_from_hp(void)
{
    QTestState *qts = start();
    static int16_t tx[2048];
    uint32_t w0, w1;

    for (int i = 0; i < 2048; i++) {
        /* Quiet for the first 512 samples, then a loud square wave */
        tx[i] = i < 512 ? (i % 2 ? 2 : -2) : ((i / 4) % 2 ? 9000 : -9000);
    }
    clocks_on(qts, 0);
    pad_out(qts, 4, SIG_O_BCK(0));
    pad_out(qts, 5, SIG_O_WS(0));
    pad_out(qts, 6, SIG_SD(0));
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_I_BCK), LP_SIG_IN_SEL | 4 << 2);
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_I_WS), LP_SIG_IN_SEL | 5 << 2);
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_I_SD), LP_SIG_IN_SEL | 6 << 2);

    lp_w(qts, LP_VAD_PARAM(0), 2u << 16 | 5000);
    lp_w(qts, LP_VAD_PARAM(1), 1 | 100u << 4 | 1u << 11 | 3u << 16);
    lp_w(qts, LP_VAD_CONF, 1);
    lp_w(qts, LP_RXEOF_NUM, 255);
    lp_w(qts, LP_RX_TDM_CTRL, TDM(2, 0x1));
    lp_w(qts, LP_RX_CONF1, LP_CONF1(16, 7, 16, 16, 16, true));
    lp_w(qts, LP_RX_CONF, 0x9600 | C_TDM_EN | C_SLAVE | C_UPDATE);
    g_assert_cmphex(lp_r(qts, LP_RX_CONF) & C_UPDATE, ==, 0);
    lp_w(qts, LP_RX_CONF, 0x9600 | C_TDM_EN | C_SLAVE | C_START);

    /* I2S0 sends the samples in the left channel, 32 us a frame */
    fill(qts, MEM + 0x1000, (uint8_t *)tx, 4000);
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8) |
               C_TX_CHAN_EQUAL, STD_CONF1, TDM(2, 0x1));
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, 4000);
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8) |
               C_TX_CHAN_EQUAL);
    qtest_clock_step(qts, 32000 * 256 + 1000);
    g_assert_cmphex(lp_r(qts, LP_INT_RAW) & LP_INT_RX_DONE, ==,
                    LP_INT_RX_DONE);
    /* The newest two samples */
    g_assert_cmphex(lp_r(qts, LP_RX_MEM_CONF) & 0x1ff, ==, 128);
    for (int i = 0; i < 127; i++) {
        lp_r(qts, LP_RAM);
    }
    w0 = lp_r(qts, LP_RAM);
    g_assert_cmphex(w0, ==, (uint16_t)tx[254] | (uint32_t)(uint16_t)tx[255]
                            << 16);
    g_assert_cmphex(lp_r(qts, LP_VAD_RESULT) & 1, ==, 0);
    qtest_clock_step(qts, 32000 * 1024);
    g_assert_cmphex(lp_r(qts, LP_VAD_RESULT) & 1, ==, 1);
    w1 = lp_r(qts, LP_VAD_OB(0));
    g_assert_cmphex(w1 >> 16, ==, 9000);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The audio backend as a codec on the bus: a master transmitter's frames
 * play at the rate and in the format the guest configures, here 16-bit
 * stereo at 31250 Hz into a WAV file, sample for sample.
 */
static void test_audio_out(void)
{
    g_autofree char *path = NULL;
    g_autofree char *args = NULL;
    g_autofree gchar *wav = NULL;
    QTestState *qts;
    int16_t tx[256];
    gsize len;
    int fd, start = -1;

    fd = g_file_open_tmp("esp32p4-i2s-XXXXXX.wav", &path, NULL);
    g_assert(fd >= 0);
    close(fd);
    args = g_strdup_printf("-audiodev wav,id=snd0,path=%s,out.frequency=31250,"
                           "out.channels=2,out.format=s16 "
                           "-M esp32p4,audiodev=snd0", path);
    qts = qtest_init(args);
    for (int k = 0; k < 128; k++) {
        tx[2 * k] = 0x1000 + 3 * k;
        tx[2 * k + 1] = -(0x1000 + 3 * k);
    }
    clocks_on(qts, 0);
    fill(qts, MEM + 0x1000, (uint8_t *)tx, sizeof(tx));
    unit_setup(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_TDM_EN | C_BCK_DIV(8));
    qtest_clock_step(qts, 100 * 1000 * 1000);
    qtest_quit(qts);

    g_assert_true(g_file_get_contents(path, &wav, &len, NULL));
    unlink(path);
    g_assert_cmpuint(len, >=, 44 + sizeof(tx));
    /* The header: PCM, two channels, 31250 Hz, 16 bits */
    g_assert_cmpuint(lduw_le_p(wav + 22), ==, 2);
    g_assert_cmpuint(ldl_le_p(wav + 24), ==, 31250);
    g_assert_cmpuint(lduw_le_p(wav + 34), ==, 16);
    for (gsize i = 44; i + 4 <= len; i += 4) {
        if ((int16_t)lduw_le_p(wav + i) == tx[0] &&
            (int16_t)lduw_le_p(wav + i + 2) == tx[1]) {
            start = i;
            break;
        }
    }
    g_assert_cmpint(start, >=, 44);
    g_assert_cmpuint(len - start, >=, sizeof(tx));
    for (int k = 0; k < 256; k++) {
        g_assert_cmpint((int16_t)lduw_le_p(wav + start + 2 * k), ==, tx[k]);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The LP I2S as the slave PDM receiver of I2S0's PCM-to-PDM converter,
 * the PDM clock and data through two pads from the HP matrix to the LP
 * matrix, its PDM-to-PCM converter making 16-bit samples of the left
 * channel: a square wave comes back into the LP I2S memory.
 */
static void test_lp_pdm(void)
{
    QTestState *qts = start();
    int16_t tx[512];
    int highs = 0, lows = 0;

    for (int i = 0; i < 256; i++) {
        tx[2 * i] = (i / 16) % 2 ? -12000 : 12000;
        tx[2 * i + 1] = 0;
    }
    clocks_on(qts, 0);
    pad_out(qts, 7, SIG_O_WS(0));
    pad_out(qts, 8, SIG_SD(0));
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_I_WS), LP_SIG_IN_SEL | 7 << 2);
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_I_SD), LP_SIG_IN_SEL | 8 << 2);
    lp_w(qts, LP_RX_PDM_CONF, 0xf8200000 | D2P_EN);
    lp_w(qts, LP_RX_TDM_CTRL, TDM(2, 0x1));
    lp_w(qts, LP_RX_CONF1, LP_CONF1(16, 7, 16, 16, 16, false));
    lp_w(qts, LP_RX_CONF, 0x9600 | C_PDM_EN | C_SLAVE | C_UPDATE);
    lp_w(qts, LP_RX_CONF, 0x9600 | C_PDM_EN | C_SLAVE | C_START);

    i2s_w(qts, 0, TX_PCM2PDM_CONF, (0x004aa004 & ~(0xfu << 1)) |
          P2D_OSR2(1) | P2D_CONV_EN);
    fill(qts, MEM + 0x1000, (uint8_t *)tx, sizeof(tx));
    unit_setup(qts, 0, true, TX_BASE | C_PDM_EN | C_BCK_DIV(8),
               CONF1(16, 16, 16, 16), STD_TDM);
    dma_tx(qts, 0, PERI_I2S(0), MEM + 0x10, MEM + 0x1000, sizeof(tx));
    qtest_clock_step(qts, 1000);
    unit_start(qts, 0, true, TX_BASE | C_PDM_EN | C_BCK_DIV(8));
    /* 256 samples of 64 us */
    qtest_clock_step(qts, 20 * 1000 * 1000);
    g_assert_cmphex(lp_r(qts, LP_RX_MEM_CONF) & 0x1ff, ==, 128);
    for (int i = 0; i < 128; i++) {
        uint32_t w = lp_r(qts, LP_RAM);

        for (int h = 0; h < 2; h++) {
            int16_t v = w >> (16 * h);

            highs += v > 10000 && v < 14000;
            lows += v < -10000 && v > -14000;
        }
    }
    /* Most of the 256 samples, all but those at the 16 edges */
    g_assert_cmpint(highs, >=, 100);
    g_assert_cmpint(lows, >=, 100);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * The LP I2S as master receiver on LP_I2S_RX_CLK with nothing of the chip
 * on its data input: it records from the audio backend (silence from
 * "none"), and its VAD takes the frames: VAD_DONE, no energy, no speech.
 */
static void test_lp_master_audio(void)
{
    QTestState *qts = qtest_init("-audiodev none,id=snd0 "
                                 "-M esp32p4,audiodev=snd0");

    lp_w(qts, LP_VAD_PARAM(0), 1u << 16 | 5000);
    lp_w(qts, LP_VAD_CONF, 1);
    lp_w(qts, LP_RX_TDM_CTRL, TDM(2, 0x3));
    lp_w(qts, LP_RX_CONF1, LP_CONF1(16, 8, 16, 16, 16, true));
    lp_w(qts, LP_RX_CONF, 0x9600 | C_TDM_EN | C_UPDATE);
    lp_w(qts, LP_RX_CONF, 0x9600 | C_TDM_EN | C_START);
    qtest_clock_step(qts, 200 * 1000 * 1000);
    g_assert_cmphex(lp_r(qts, LP_INT_RAW) & (LP_INT_VAD_DONE | LP_INT_RX_DONE),
                    ==, LP_INT_VAD_DONE | LP_INT_RX_DONE);
    g_assert_cmphex(lp_r(qts, LP_VAD_RESULT), ==, 0);
    g_assert_cmphex(lp_r(qts, LP_RAM), ==, 0);
    qtest_quit(qts);
}

/* Pad @pad driven from outside the chip at @level. */
static void pad_drive(QTestState *qts, int pad, int level)
{
    qtest_set_irq_in(qts, "/machine/gpio", "pad-in", pad, level);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * Slaves clocked by a master outside the chip, through the pads: I2S1's
 * receiver samples GPIO22 on each rising edge of GPIO20, its BCK, framed
 * by GPIO21, its WS (Philips: the MSB one cycle after WS falls); I2S2's
 * transmitter drives its next bit onto GPIO23 on each falling edge.
 */
static void test_pad_slave(void)
{
    QTestState *qts = start();
    static const uint16_t data[4] = { 0xa55a, 0x1234, 0x0ff0, 0x8001 };
    uint16_t rx[4], got[4] = { 0 };
    uint8_t txd[64];
    int cycle = 0;

    for (int i = 0; i < sizeof(txd); i++) {
        txd[i] = 0x3c ^ (i * 5);
    }
    clocks_on(qts, 1);
    clocks_on(qts, 2);
    for (int pad = 20; pad < 23; pad++) {
        qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    }
    pad_in(qts, 20, SIG_I_BCK(1));
    pad_in(qts, 21, SIG_I_WS(1));
    pad_in(qts, 22, SIG_SD(1));
    pad_in(qts, 20, SIG_O_BCK(2));
    pad_in(qts, 21, SIG_O_WS(2));
    pad_out(qts, 23, SIG_SD(2));
    pad_drive(qts, 20, 1);
    pad_drive(qts, 21, 1);
    pad_drive(qts, 22, 0);

    i2s_w(qts, 1, RXEOF_NUM, 3);
    unit_go(qts, 1, false, RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1, STD_TDM);
    dma_rx(qts, 1, PERI_I2S(1), MEM, MEM + 0x2000, sizeof(rx));
    fill(qts, MEM + 0x1000, txd, sizeof(txd));
    unit_setup(qts, 2, true, TX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1,
               STD_TDM);
    dma_tx(qts, 0, PERI_I2S(2), MEM + 0x10, MEM + 0x1000, sizeof(txd));
    qtest_clock_step(qts, 10000);
    unit_start(qts, 2, true, TX_BASE | C_TDM_EN | C_SLAVE);

    /* Two frames and the cycle that carries the last bit */
    for (int f = 0; f < 2; f++) {
        for (int i = 0; i < 32; i++, cycle++) {
            int ch = 2 * f + (i >= 1 && i < 17 ? 0 : 1);
            int b = (i + 31) % 16;
            int bit = 0;

            if (f == 0 && i == 0) {
                ch = -1;
            } else if (i == 0) {
                ch = 2 * f - 1;
            }
            if (ch >= 0) {
                bit = (data[ch] >> (15 - b)) & 1;
            }
            pad_drive(qts, 20, 0);
            pad_drive(qts, 21, i >= 16);
            pad_drive(qts, 22, bit);
            qtest_clock_step(qts, 500);
            pad_drive(qts, 20, 1);
            /* The transmitter's bit, MSB first from the second cycle */
            if (i >= 1 && i < 17 && f == 0) {
                got[0] |= ((qtest_readl(qts, GPIO_IN) >> 23) & 1) << (16 - i);
            }
            qtest_clock_step(qts, 500);
        }
    }
    pad_drive(qts, 20, 0);
    pad_drive(qts, 21, 0);
    pad_drive(qts, 22, data[3] & 1);
    qtest_clock_step(qts, 500);
    pad_drive(qts, 20, 1);
    qtest_clock_step(qts, 100000);
    qtest_memread(qts, MEM + 0x2000, rx, sizeof(rx));
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(rx[i], ==, data[i]);
    }
    g_assert_cmphex(i2s_r(qts, 1, INT_RAW) & INT_RX_DONE, ==, INT_RX_DONE);
    g_assert_cmphex(got[0], ==, txd[0] | txd[1] << 8);
    g_assert_cmpuint(cycle, ==, 64);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s/test]
 * Controllers on different pads that the board wires together (the GPIO
 * block's pad-wires): I2S2's transmitter on GPIO20 to GPIO22, I2S1's
 * receiver on GPIO30 to GPIO32.
 */
static void test_pad_wires(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -global "
                                 "esp32p4-gpio.pad-wires=20:30,21:31,22:32");
    uint8_t tx[64], rx[64];

    for (int i = 0; i < sizeof(tx); i++) {
        tx[i] = i * 11 + 7;
    }
    clocks_on(qts, 1);
    clocks_on(qts, 2);
    pad_out(qts, 20, SIG_O_BCK(2));
    pad_out(qts, 21, SIG_O_WS(2));
    pad_out(qts, 22, SIG_SD(2));
    for (int pad = 30; pad < 33; pad++) {
        qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    }
    pad_in(qts, 30, SIG_I_BCK(1));
    pad_in(qts, 31, SIG_I_WS(1));
    pad_in(qts, 32, SIG_SD(1));
    transfer(qts, 2, TX_BASE | C_TDM_EN | C_BCK_DIV(8), STD_CONF1, STD_TDM,
             1, RX_BASE | C_TDM_EN | C_SLAVE, STD_CONF1, STD_TDM,
             tx, sizeof(tx), rx, sizeof(rx), 2, 1000);
    g_assert_cmpmem(rx, sizeof(rx), tx, sizeof(tx));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4-i2s/reset-values", test_reset_values);
    qtest_add_func("/esp32p4-i2s/access", test_access);
    qtest_add_func("/esp32p4-i2s/update", test_update);
    qtest_add_func("/esp32p4-i2s/std-loopback", test_std_loopback);
    qtest_add_func("/esp32p4-i2s/tdm-slots", test_tdm_slots);
    qtest_add_func("/esp32p4-i2s/formats", test_formats);
    qtest_add_func("/esp32p4-i2s/rx-master", test_rx_master);
    qtest_add_func("/esp32p4-i2s/sig-loopback", test_sig_loopback);
    qtest_add_func("/esp32p4-i2s/timing", test_timing);
    qtest_add_func("/esp32p4-i2s/pdm-raw", test_pdm_raw);
    qtest_add_func("/esp32p4-i2s/pdm-conv", test_pdm_conv);
    qtest_add_func("/esp32p4-i2s/alaw", test_alaw);
    qtest_add_func("/esp32p4-i2s/hung", test_hung);
    qtest_add_func("/esp32p4-i2s/etm", test_etm);
    qtest_add_func("/esp32p4-i2s/mclk", test_mclk);
    qtest_add_func("/esp32p4-i2s/mclk-exact", test_mclk_exact);
    qtest_add_func("/esp32p4-i2s/lp-mem", test_lp_mem);
    qtest_add_func("/esp32p4-i2s/vad-manual", test_vad_manual);
    qtest_add_func("/esp32p4-i2s/lp-from-hp", test_lp_from_hp);
    qtest_add_func("/esp32p4-i2s/audio-out", test_audio_out);
    qtest_add_func("/esp32p4-i2s/lp-pdm", test_lp_pdm);
    qtest_add_func("/esp32p4-i2s/lp-master-audio", test_lp_master_audio);
    qtest_add_func("/esp32p4-i2s/pad-slave", test_pad_slave);
    qtest_add_func("/esp32p4-i2s/pad-wires", test_pad_wires);

    return g_test_run();
}
