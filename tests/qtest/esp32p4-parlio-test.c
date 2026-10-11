/*
 * QTest testcase for the ESP32-P4 parallel IO controller (PARLIO)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The units' pad signals go through the GPIO matrix to pads, where the
 * test reads them (GPIO_IN) at exact virtual times, or drives them from
 * outside (the matrix's "pad-in" inputs) to play a transmitter. DMA goes
 * through GDMA-AXI's channel 0 with descriptors in L2MEM past the L2
 * cache's 128 KiB.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define PARLIO              0x500cf000
#define RX_MODE_CFG         (PARLIO + 0x00)
#define RX_DATA_CFG         (PARLIO + 0x04)
#define RX_GENRL_CFG        (PARLIO + 0x08)
#define RX_START_CFG        (PARLIO + 0x0c)
#define TX_DATA_CFG         (PARLIO + 0x10)
#define TX_START_CFG        (PARLIO + 0x14)
#define TX_GENRL_CFG        (PARLIO + 0x18)
#define FIFO_CFG            (PARLIO + 0x1c)
#define REG_UPDATE          (PARLIO + 0x20)
#define ST                  (PARLIO + 0x24)
#define INT_ENA             (PARLIO + 0x28)
#define INT_RAW             (PARLIO + 0x2c)
#define INT_ST              (PARLIO + 0x30)
#define INT_CLR             (PARLIO + 0x34)
#define RX_ST0              (PARLIO + 0x38)
#define RX_ST1              (PARLIO + 0x3c)
#define TX_ST0              (PARLIO + 0x40)
#define RX_CLK_CFG          (PARLIO + 0x44)
#define TX_CLK_CFG          (PARLIO + 0x48)
#define TX_CS_CFG           (PARLIO + 0x4c)
#define PARLIO_CLK          (PARLIO + 0x120)
#define VERSION             (PARLIO + 0x3fc)

#define EXT_EN_SEL(n)       ((uint32_t)(n) << 21)
#define SW_EN               (1u << 25)
#define EXT_EN_INV          (1u << 26)
#define SUBMODE(n)          ((uint32_t)(n) << 27)
#define SMP_LEVEL           (0u << 30)
#define SMP_PULSE           (1u << 30)
#define SMP_SW              (2u << 30)
#define BITLEN(n)           ((uint32_t)(n) << 9)
#define ORDER_INV           (1u << 28)
#define WID(sel)            ((uint32_t)(sel) << 29)
#define RX_GATING_EN        (1u << 12)
#define TIMEOUT_THRES(n)    ((uint32_t)(n) << 13)
#define TIMEOUT_EN          (1u << 29)
#define RX_EOF_BY_EN        (1u << 30)
#define START               (1u << 31)
#define TX_EOF_BY_DMA       (1u << 13)
#define IDLE(v)             ((uint32_t)(v) << 14)
#define TX_GATING_EN        (1u << 30)
#define VALID_OUTPUT_EN     (1u << 31)
#define TX_FIFO_SRST        (1u << 30)
#define RX_FIFO_SRST        (1u << 31)
#define RX_REG_UPDATE       (1u << 31)
#define TX_READY            (1u << 31)
#define INT_TX_REMPTY       (1u << 0)
#define INT_RX_WOVF         (1u << 1)
#define INT_TX_EOF          (1u << 2)
#define CLK_I_INV           (1u << 30)
#define CLK_O_INV           (1u << 31)
#define CS_DELAYS(start, stop) (((uint32_t)(start) << 16) | (stop))

/* Clocks and resets. */
#define CLKRST              0x500e6000
#define SOC_CLK_CTRL1       (CLKRST + 0x18)
#define SOC_CLK_CTRL2       (CLKRST + 0x1c)
#define PERI_CLK_CTRL117    (CLKRST + 0x84)
#define PERI_CLK_CTRL118    (CLKRST + 0x88)
#define HP_RST_EN2          (CLKRST + 0xc8)
#define PARLIO_SYS_CLK_EN   (1u << 23)
#define PARLIO_APB_CLK_EN   (1u << 31)
#define RST_EN_PARLIO       (1u << 1)
#define RST_EN_PARLIO_RX    (1u << 2)
#define RST_EN_PARLIO_TX    (1u << 3)

/* The GPIO matrix and IO MUX. */
#define GPIO                0x500e0000
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IN_INV              (1u << 6)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (1u << 12)
#define SIG_RX_CLK          186
#define SIG_TX_CLK          187
#define SIG_DATA(n)         (188 + (n))
#define SIG_TX_CS           242

/* Pads: TXD/RXD n on pad n, the TX clock on 16, CS on 17, RX's on 18. */
#define PAD_CLK             16
#define PAD_CS              17
#define PAD_RX_CLK          18

/* GDMA-AXI channel 0, PARLIO's. */
#define AXI                 0x5008a000
#define AXI_IN              (AXI + 0x00)
#define AXI_OUT             (AXI + 0x138)
#define X_RAW               0x00
#define X_CLR               0x0c
#define X_CONF0             0x10
#define X_LINK1             0x20
#define X_LINK2             0x24
#define X_PERI_SEL          0x44
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define IN_DONE             (1u << 0)
#define IN_SUC_EOF          (1u << 1)
#define IN_ERR_EOF          (1u << 2)
#define OUT_TOTAL_EOF       (1u << 3)
#define PERI_PARLIO         3
#define DW0(size, len)      ((size) | (uint32_t)(len) << 12)
#define SUC_EOF             (1u << 30)
#define ERR_EOF             (1u << 28)
#define OWNER               (1u << 31)

#define INTMTX_STATUS(w)    (0x500d6000 + 0x200 + 4 * (w))
#define SRC_PARLIO_RX       113
#define SRC_PARLIO_TX       114

#define MEM                 0x4ff40000
#define TX_DESC             MEM
#define RX_DESC             (MEM + 0x100)
#define SRC                 (MEM + 0x1000)
#define DST                 (MEM + 0x4000)

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M");

    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | PARLIO_SYS_CLK_EN);
    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) | PARLIO_APB_CLK_EN);
    return qts;
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

/* PARLIO_TX_CLK and PARLIO_RX_CLK: XTAL (40 MHz) / @div, or a pad. */
static void tx_clock(QTestState *qts, int src, unsigned div, bool on)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL118) & ~(0xffffu << 16);

    qtest_writel(qts, PERI_CLK_CTRL118, v | (uint32_t)src << 16 |
                 (uint32_t)(div - 1) << 19 | (on ? 1u << 18 : 0));
}

static void rx_clock(QTestState *qts, int src, unsigned div, bool on)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL117) & ~(0x7ffu << 17);

    qtest_writel(qts, PERI_CLK_CTRL117, v | (uint32_t)src << 17 |
                 (uint32_t)(div - 1) << 20 | (on ? 1u << 19 : 0));
}

static void pad_out(QTestState *qts, int pad, int sig)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(pad), sig);
}

static void pad_in(QTestState *qts, int sig, int pad, uint32_t extra)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_IN(sig), IN_SIG_IN_SEL | pad | extra);
}

/* Drive pad @n from outside the chip. */
static void drive(QTestState *qts, int n, int level)
{
    qtest_set_irq_in(qts, "/machine/gpio", "pad-in", n, level);
}

static uint32_t pads(QTestState *qts)
{
    return qtest_readl(qts, GPIO_IN);
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

/* Start GDMA-AXI TX channel 0 for PARLIO on @len bytes at SRC. */
static void dma_tx(QTestState *qts, uint32_t len)
{
    desc(qts, TX_DESC, OWNER | SUC_EOF | DW0(len, len), SRC, 0);
    qtest_writel(qts, AXI_OUT + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1u << 3);
    qtest_writel(qts, AXI_OUT + X_PERI_SEL, PERI_PARLIO);
    qtest_writel(qts, AXI_OUT + X_LINK2, TX_DESC);
    qtest_writel(qts, AXI_OUT + X_LINK1, OUT_LINK_START);
}

/* Start GDMA-AXI RX channel 0 for PARLIO into @size bytes at DST. */
static void dma_rx(QTestState *qts, uint32_t size)
{
    desc(qts, RX_DESC, OWNER | DW0(size, 0), DST, 0);
    qtest_writel(qts, AXI_IN + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_IN + X_CONF0, 1);
    qtest_writel(qts, AXI_IN + X_CONF0, 0);
    qtest_writel(qts, AXI_IN + X_PERI_SEL, PERI_PARLIO);
    qtest_writel(qts, AXI_IN + X_LINK2, RX_DESC);
    qtest_writel(qts, AXI_IN + X_LINK1, IN_LINK_START);
}

static void fill(QTestState *qts, uint8_t *b, uint32_t len, uint8_t seed)
{
    for (uint32_t i = 0; i < len; i++) {
        b[i] = seed + i * 37 + (i >> 3);
    }
    qtest_memwrite(qts, SRC, b, len);
}

/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, RX_MODE_CFG), ==, 0x00e00000);
    g_assert_cmphex(qtest_readl(qts, RX_DATA_CFG), ==, 0x60000000);
    g_assert_cmphex(qtest_readl(qts, RX_GENRL_CFG), ==, 0x21ffe000);
    g_assert_cmphex(qtest_readl(qts, RX_START_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TX_DATA_CFG), ==, 0x60000000);
    g_assert_cmphex(qtest_readl(qts, TX_START_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TX_GENRL_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, FIFO_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, REG_UPDATE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RX_ST0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RX_ST1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TX_ST0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RX_CLK_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TX_CLK_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TX_CS_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PARLIO_CLK), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VERSION), ==, 0x2409230);

    /* Writable bits only; REG_UPDATE and INT_CLR read 0. */
    qtest_writel(qts, RX_MODE_CFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RX_MODE_CFG), ==, 0xffe00000);
    qtest_writel(qts, RX_GENRL_CFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RX_GENRL_CFG), ==, 0x7ffff000);
    qtest_writel(qts, TX_GENRL_CFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TX_GENRL_CFG), ==, 0xffffe000);
    qtest_writel(qts, TX_CS_CFG, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, TX_CS_CFG), ==, 0x12345678);
    qtest_writel(qts, RX_CLK_CFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RX_CLK_CFG), ==, 0xc0000000);
    qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);
    g_assert_cmphex(qtest_readl(qts, REG_UPDATE), ==, 0);
    qtest_writel(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 7);
    qtest_writel(qts, VERSION, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VERSION), ==, 0x0fffffff);

    /* RST_EN_PARLIO resets the block. */
    qtest_writel(qts, HP_RST_EN2, RST_EN_PARLIO);
    qtest_writel(qts, HP_RST_EN2, 0);
    g_assert_cmphex(qtest_readl(qts, RX_MODE_CFG), ==, 0x00e00000);
    g_assert_cmphex(qtest_readl(qts, TX_CS_CFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VERSION), ==, 0x2409230);
    qtest_quit(qts);
}

/*
 * TX at 8 bits, XTAL / 4 (10 MHz): CS, the clock and the data on the pads
 * at their exact times, the idle value around the frame, TX_EOF by
 * length, its interrupt through the matrix.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_tx_waveform(void)
{
    QTestState *qts = start();
    uint8_t b[4] = { 0x5a, 0xc3, 0x01, 0xfe };
    int i;

    qtest_memwrite(qts, SRC, b, 4);
    for (i = 0; i < 8; i++) {
        pad_out(qts, i, SIG_DATA(i));
    }
    pad_out(qts, PAD_CLK, SIG_TX_CLK);
    pad_out(qts, PAD_CS, SIG_TX_CS);
    tx_clock(qts, 0, 4, false);
    qtest_writel(qts, TX_DATA_CFG, WID(3) | BITLEN(32));
    qtest_writel(qts, TX_GENRL_CFG, IDLE(0x81));
    qtest_writel(qts, TX_CS_CFG, CS_DELAYS(2, 1));
    qtest_writel(qts, INT_ENA, INT_TX_EOF);
    g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x20081);

    dma_tx(qts, 4);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, ST), ==, TX_READY);
    qtest_writel(qts, TX_START_CFG, START);
    tx_clock(qts, 0, 4, true);

    /* Rising edges at 50, 150, ... ns: CS falls at the first. */
    qtest_clock_step(qts, 49);
    g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x20081);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x10081);
    qtest_clock_step(qts, 50);
    g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x00081);
    /* Two edges of start delay, then a byte an edge, seen mid-cycle. */
    for (i = 0; i < 4; i++) {
        qtest_clock_step(qts, i ? 100 : 175);
        g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x10000 | b[i]);
        g_assert_false(qtest_readl(qts, INT_RAW) & INT_TX_EOF);
    }
    /* The idle value at the next edge, CS up one edge later with EOF. */
    qtest_clock_step(qts, 100);
    g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x10081);
    g_assert_false(qtest_readl(qts, INT_RAW) & INT_TX_EOF);
    g_assert_cmphex(qtest_readl(qts, TX_ST0) >> 13, ==, 32);
    qtest_clock_step(qts, 100);
    g_assert_cmphex(pads(qts) & 0x300ff & ~0x10000, ==, 0x20081);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_TX_EOF);
    g_assert_cmphex(qtest_readl(qts, INT_ST), ==, INT_TX_EOF);
    g_assert_true(source_level(qts, SRC_PARLIO_TX));
    g_assert_false(source_level(qts, SRC_PARLIO_RX));
    g_assert_cmphex(qtest_readl(qts, AXI_OUT + X_RAW) & OUT_TOTAL_EOF, ==,
                    OUT_TOTAL_EOF);
    /* The clock stops low; nothing more until TX_START goes 0 and 1. */
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(pads(qts) & 0x300ff, ==, 0x20081);
    qtest_writel(qts, INT_CLR, INT_TX_EOF);
    g_assert_false(source_level(qts, SRC_PARLIO_TX));
    qtest_quit(qts);
}

/*
 * TX into RX through the pads at every width: RX clocked by TX's clock
 * (its pad source), sampling on the falling edge, in software enable mode
 * with TX's clock gated by CS, so that it sees exactly the frame's cycles;
 * the data reaches memory exactly, with GDMA's SUC EOF from RX_BITLEN.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_loopback_widths(void)
{
    static const struct {
        unsigned sel, width;
        bool inv;
    } cases[] = {
        { 0, 1, false }, { 1, 2, false }, { 2, 4, false }, { 2, 4, true },
        { 3, 8, false }, { 4, 16, false },
    };
    QTestState *qts = start();
    uint8_t want[64], got[64];
    int i, c;

    for (i = 0; i < 16; i++) {
        pad_out(qts, i, SIG_DATA(i));
        pad_in(qts, SIG_DATA(i), i, 0);
    }
    pad_out(qts, PAD_CLK, SIG_TX_CLK);
    pad_in(qts, SIG_RX_CLK, PAD_CLK, 0);
    pad_out(qts, PAD_CS, SIG_TX_CS);
    rx_clock(qts, 3, 1, true);

    for (c = 0; c < ARRAY_SIZE(cases); c++) {
        uint32_t len = 16 + 8 * c;
        uint32_t order = 0;

        if (cases[c].inv) {
            order = ORDER_INV;
        }
        fill(qts, want, len, c);
        memset(got, 0, sizeof(got));
        qtest_memwrite(qts, DST, got, sizeof(got));
        tx_clock(qts, 0, 2, false);
        qtest_writel(qts, TX_START_CFG, 0);
        qtest_writel(qts, RX_START_CFG, 0);
        qtest_writel(qts, TX_DATA_CFG, WID(cases[c].sel) |
                     BITLEN(len * 8) | order);
        qtest_writel(qts, TX_GENRL_CFG, TX_GATING_EN | VALID_OUTPUT_EN);
        qtest_writel(qts, RX_MODE_CFG, SMP_SW | SW_EN);
        qtest_writel(qts, RX_DATA_CFG, WID(cases[c].sel) |
                     BITLEN(len * 8) | order);
        qtest_writel(qts, RX_GENRL_CFG, 0);
        qtest_writel(qts, RX_CLK_CFG, CLK_I_INV);
        qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);
        qtest_writel(qts, INT_CLR, 7);
        dma_rx(qts, 64);
        dma_tx(qts, len);
        qtest_clock_step(qts, 10000);
        qtest_writel(qts, RX_START_CFG, START);
        qtest_writel(qts, TX_START_CFG, START);
        tx_clock(qts, 0, 2, true);
        /* 20 MHz: 50 ns a cycle, and two to spare. */
        qtest_clock_step(qts, (len * 8 / cases[c].width + 2) * 50 + 10000);
        g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_TX_EOF);
        g_assert_cmphex(qtest_readl(qts, AXI_IN + X_RAW), ==,
                        IN_DONE | IN_SUC_EOF);
        g_assert_cmphex(qtest_readl(qts, RX_DESC), ==,
                        SUC_EOF | DW0(64, len));
        qtest_memread(qts, DST, got, len);
        g_assert_cmpmem(want, len, got, len);
    }
    qtest_quit(qts);
}

/*
 * RX in level enable mode, its clock and data driven on the pads by the
 * test: data while RXD[15] is high, the frame ended by the enable falling
 * (RX_EOF_GEN_SEL), a partial byte padded; RX_REG_UPDATE taking the
 * configuration; then the receive timeout's ERR EOF.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_rx_level_and_timeout(void)
{
    QTestState *qts = start();
    static const uint8_t nibbles[] = { 0x1, 0x2, 0x3, 0x4, 0x5 };
    uint8_t got[4];
    int i;

    for (i = 0; i < 4; i++) {
        pad_in(qts, SIG_DATA(i), i, 0);
    }
    pad_in(qts, SIG_DATA(15), 15, 0);
    pad_in(qts, SIG_RX_CLK, PAD_RX_CLK, 0);
    for (i = 0; i < 16; i++) {
        drive(qts, i, 0);
    }
    drive(qts, PAD_RX_CLK, 0);
    rx_clock(qts, 3, 1, true);
    qtest_writel(qts, RX_MODE_CFG, SMP_LEVEL | EXT_EN_SEL(15));
    qtest_writel(qts, RX_DATA_CFG, WID(2) | BITLEN(64));
    qtest_writel(qts, RX_GENRL_CFG, RX_EOF_BY_EN);
    dma_rx(qts, 64);
    qtest_writel(qts, RX_START_CFG, START);

    /* Before RX_REG_UPDATE the reset configuration (8 bits, RXD[7]). */
    drive(qts, 15, 1);
    drive(qts, 0, 1);
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, RX_ST0) >> 13, ==, 0);
    drive(qts, 15, 0);
    qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);

    /* Clocked on the rising edge: the enable, then five nibbles. */
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    for (i = 0; i < ARRAY_SIZE(nibbles); i++) {
        for (int k = 0; k < 4; k++) {
            drive(qts, k, (nibbles[i] >> k) & 1);
        }
        drive(qts, 15, 1);
        drive(qts, PAD_RX_CLK, 1);
        drive(qts, PAD_RX_CLK, 0);
    }
    g_assert_cmphex(qtest_readl(qts, RX_ST0) >> 13, ==, 20);
    drive(qts, 15, 0);
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, RX_DESC), ==, SUC_EOF | DW0(64, 3));
    qtest_memread(qts, DST, got, 3);
    g_assert_cmphex(got[0], ==, 0x21);
    g_assert_cmphex(got[1], ==, 0x43);
    g_assert_cmphex(got[2], ==, 0x05);

    /* The timeout: 100 PARLIO_SYS_CLK cycles after the last data. */
    qtest_writel(qts, RX_GENRL_CFG, TIMEOUT_EN | TIMEOUT_THRES(100));
    qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);
    dma_rx(qts, 64);
    drive(qts, 15, 1);
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    drive(qts, 15, 0);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AXI_IN + X_RAW) & IN_ERR_EOF, ==,
                    IN_ERR_EOF);
    g_assert_cmphex(qtest_readl(qts, RX_DESC), ==, ERR_EOF | DW0(64, 1));
    qtest_quit(qts);
}

/*
 * RX in pulse enable mode, sub-mode 1 (start pulse's data in, end pulse's
 * out) with EOF at the end pulse, MSB first at 1 bit; then sub-mode 5
 * (start excluded, end by length).
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_rx_pulse(void)
{
    QTestState *qts = start();
    static const uint8_t bits[] = { 1, 0, 1, 1, 0, 0, 1, 0, 1, 1 };
    uint8_t got[2];
    int i;

    pad_in(qts, SIG_DATA(0), 0, 0);
    pad_in(qts, SIG_DATA(3), 3, 0);
    pad_in(qts, SIG_RX_CLK, PAD_RX_CLK, 0);
    drive(qts, 0, 0);
    drive(qts, 3, 0);
    drive(qts, PAD_RX_CLK, 0);
    rx_clock(qts, 3, 1, true);
    qtest_writel(qts, RX_MODE_CFG, SMP_PULSE | SUBMODE(1) | EXT_EN_SEL(3));
    qtest_writel(qts, RX_DATA_CFG, WID(0) | ORDER_INV | BITLEN(100));
    qtest_writel(qts, RX_GENRL_CFG, RX_EOF_BY_EN);
    qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);
    dma_rx(qts, 64);
    qtest_writel(qts, RX_START_CFG, START);

    /* Idle cycles, then the start pulse with the first bit. */
    for (i = 0; i < 3; i++) {
        drive(qts, PAD_RX_CLK, 1);
        drive(qts, PAD_RX_CLK, 0);
    }
    for (i = 0; i < ARRAY_SIZE(bits); i++) {
        drive(qts, 3, i == 0 || i == ARRAY_SIZE(bits) - 1);
        drive(qts, 0, bits[i]);
        drive(qts, PAD_RX_CLK, 1);
        drive(qts, PAD_RX_CLK, 0);
    }
    qtest_clock_step(qts, 10000);
    /* Nine bits: 1011 0010, then 1 padded. */
    g_assert_cmphex(qtest_readl(qts, RX_DESC), ==, SUC_EOF | DW0(64, 2));
    qtest_memread(qts, DST, got, 2);
    g_assert_cmphex(got[0], ==, 0xb2);
    g_assert_cmphex(got[1], ==, 0x80);

    /* Sub-mode 5: the pulse's bit out, 8 bits after it, LSB first. */
    drive(qts, 3, 0);
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    qtest_writel(qts, RX_MODE_CFG, SMP_PULSE | SUBMODE(5) | EXT_EN_SEL(3));
    qtest_writel(qts, RX_DATA_CFG, WID(0) | BITLEN(8));
    qtest_writel(qts, RX_GENRL_CFG, 0);
    qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);
    dma_rx(qts, 64);
    for (i = 0; i < 9; i++) {
        drive(qts, 3, i == 0);
        drive(qts, 0, i == 0 || i == 1 || i == 8);
        drive(qts, PAD_RX_CLK, 1);
        drive(qts, PAD_RX_CLK, 0);
    }
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, RX_DESC), ==, SUC_EOF | DW0(64, 1));
    qtest_memread(qts, DST, got, 1);
    g_assert_cmphex(got[0], ==, 0x81);
    qtest_quit(qts);
}

/*
 * TX as a slave: its clock from its pad, driven by the test, divided by
 * two; it shifts on the falling edge (TX_CLK_I_INV). TX_EOF from GDMA's
 * EOF (TX_EOF_GEN_SEL) at 2 bits, an odd length; the FIFO running empty
 * raises TX_FIFO_REMPTY; RST_EN_PARLIO_TX returns TXD to idle.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_tx_external_clock(void)
{
    QTestState *qts = start();
    uint8_t b[3] = { 0xe4, 0x1b, 0x9c };
    uint32_t seen = 0;
    int i;

    qtest_memwrite(qts, SRC, b, 3);
    pad_out(qts, 0, SIG_DATA(0));
    pad_out(qts, 1, SIG_DATA(1));
    pad_in(qts, SIG_TX_CLK, PAD_CLK, 0);
    drive(qts, PAD_CLK, 0);
    tx_clock(qts, 3, 2, true);
    qtest_writel(qts, TX_DATA_CFG, WID(1));
    qtest_writel(qts, TX_GENRL_CFG, TX_EOF_BY_DMA);
    qtest_writel(qts, TX_CLK_CFG, CLK_I_INV);
    dma_tx(qts, 3);
    qtest_clock_step(qts, 10000);
    qtest_writel(qts, TX_START_CFG, START);

    /*
     * Divided by two the clock rises at the pad's first rising edge and
     * falls at its second; TX shifts at that fall: a 2-bit cycle every
     * two pad cycles, sampled here before the next.
     */
    for (i = 0; i < 26; i++) {
        drive(qts, PAD_CLK, 1);
        drive(qts, PAD_CLK, 0);
        if (i % 2 == 1 && i / 2 < 12) {
            seen |= (pads(qts) & 3) << (2 * (i / 2));
        }
    }
    g_assert_cmphex(seen & 0xffffff, ==, 0x9c1be4);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_TX_EOF);
    g_assert_cmphex(qtest_readl(qts, TX_ST0) >> 13, ==, 24);

    /* A frame whose data the DMA never brings: REMPTY. */
    qtest_writel(qts, TX_START_CFG, 0);
    qtest_writel(qts, INT_CLR, 7);
    qtest_writel(qts, TX_START_CFG, START);
    for (i = 0; i < 4; i++) {
        drive(qts, PAD_CLK, 1);
        drive(qts, PAD_CLK, 0);
    }
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_TX_REMPTY);
    qtest_writel(qts, HP_RST_EN2, RST_EN_PARLIO_TX);
    qtest_writel(qts, HP_RST_EN2, 0);
    g_assert_cmphex(pads(qts) & 3, ==, 0);
    qtest_quit(qts);
}

/*
 * RX's FIFO overflowing: bytes keep coming after the RX channel's list
 * has ended and the channel's FIFO is full, and RX_FIFO_WOVF is raised,
 * through the matrix as HP_PARLIO_RX_INTR. RX_FIFO_SRST empties the FIFO.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_rx_overflow(void)
{
    QTestState *qts = start();
    int i;

    pad_in(qts, SIG_DATA(0), 0, 0);
    pad_in(qts, SIG_RX_CLK, PAD_RX_CLK, 0);
    drive(qts, 0, 1);
    drive(qts, PAD_RX_CLK, 0);
    rx_clock(qts, 3, 1, true);
    qtest_writel(qts, RX_MODE_CFG, SMP_SW | SW_EN);
    qtest_writel(qts, RX_DATA_CFG, WID(3) | BITLEN(8 * 16));
    qtest_writel(qts, RX_GENRL_CFG, 0);
    qtest_writel(qts, REG_UPDATE, RX_REG_UPDATE);
    qtest_writel(qts, INT_ENA, INT_RX_WOVF);
    dma_rx(qts, 16);
    qtest_writel(qts, RX_START_CFG, START);
    for (i = 0; i < 200 && !(qtest_readl(qts, INT_RAW) & INT_RX_WOVF); i++) {
        drive(qts, PAD_RX_CLK, 1);
        drive(qts, PAD_RX_CLK, 0);
        qtest_clock_step(qts, 100);
    }
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_RX_WOVF);
    g_assert_true(source_level(qts, SRC_PARLIO_RX));
    /* The channel took its 16 bytes and its own FIFO's 64, PARLIO 16. */
    g_assert_cmpint(i, ==, 16 + 64 + 16 + 1);
    g_assert_cmphex(qtest_readl(qts, RX_DESC), ==, SUC_EOF | DW0(16, 16));
    qtest_writel(qts, INT_CLR, INT_RX_WOVF);
    g_assert_false(source_level(qts, SRC_PARLIO_RX));
    qtest_writel(qts, FIFO_CFG, RX_FIFO_SRST);
    qtest_writel(qts, FIFO_CFG, 0);
    drive(qts, PAD_RX_CLK, 1);
    drive(qts, PAD_RX_CLK, 0);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/parlio/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/parlio/tx-waveform", test_tx_waveform);
    qtest_add_func("/esp32p4/parlio/loopback-widths", test_loopback_widths);
    qtest_add_func("/esp32p4/parlio/rx-level-timeout",
                   test_rx_level_and_timeout);
    qtest_add_func("/esp32p4/parlio/rx-pulse", test_rx_pulse);
    qtest_add_func("/esp32p4/parlio/tx-external-clock",
                   test_tx_external_clock);
    qtest_add_func("/esp32p4/parlio/rx-overflow", test_rx_overflow);
    return g_test_run();
}
