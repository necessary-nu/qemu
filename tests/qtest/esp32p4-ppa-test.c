/*
 * QTest testcase for the ESP32-P4 pixel-processing accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The tests set the PPA and the 2D-DMA up as ESP-IDF's PPA driver does
 * (descriptors in L2MEM past the 128 KiB the L2 cache holds from reset),
 * run fills, blends and scale-rotate-mirror operations and step virtual
 * time while they work, checking the pixels against values computed here
 * from the arithmetic the model documents, the registers and the timing.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"
#include "qobject/qdict.h"

#define PPA             0x50087000
#define DMA2D           0x50088000
#define OUT(ch)         (DMA2D + 0x100 * (ch))
#define IN(ch)          (DMA2D + 0x500 + 0x100 * (ch))
#define MEM             0x4ff40000

/* PPA registers. */
#define BLEND0_CLUT_DATA    (PPA + 0x000)
#define BLEND1_CLUT_DATA    (PPA + 0x004)
#define CLUT_CONF           (PPA + 0x00c)
#define INT_RAW             (PPA + 0x010)
#define INT_ST              (PPA + 0x014)
#define INT_ENA             (PPA + 0x018)
#define INT_CLR             (PPA + 0x01c)
#define SR_COLOR_MODE       (PPA + 0x020)
#define BLEND_COLOR_MODE    (PPA + 0x024)
#define SR_BYTE_ORDER       (PPA + 0x028)
#define BLEND_BYTE_ORDER    (PPA + 0x02c)
#define BLEND_TRANS_MODE    (PPA + 0x034)
#define SR_FIX_ALPHA        (PPA + 0x038)
#define BLEND_TX_SIZE       (PPA + 0x03c)
#define BLEND_FIX_ALPHA     (PPA + 0x040)
#define BLEND_RGB           (PPA + 0x048)
#define BLEND_FIX_PIXEL     (PPA + 0x04c)
#define CK_FG_LOW           (PPA + 0x050)
#define CK_FG_HIGH          (PPA + 0x054)
#define CK_BG_LOW           (PPA + 0x058)
#define CK_BG_HIGH          (PPA + 0x05c)
#define CK_DEFAULT          (PPA + 0x060)
#define SR_SCAL_ROTATE      (PPA + 0x064)
#define SR_MEM_PD           (PPA + 0x068)
#define CLUT_CNT            (PPA + 0x070)
#define BLEND_ST            (PPA + 0x074)
#define SR_PARAM_ERR_ST     (PPA + 0x078)
#define RGB2GRAY            (PPA + 0x098)
#define DATE                (PPA + 0x100)
#define CLUT0_MEM           (PPA + 0x400)
#define CLUT1_MEM           (PPA + 0x800)

#define INT_SR_EOF          (1u << 0)
#define INT_BLEND_EOF       (1u << 1)
#define INT_SR_PARAM_ERR    (1u << 2)
#define INT_BLEND_PARAM_ERR (1u << 3)

/* BLEND_TRANS_MODE. */
#define BLEND_EN            (1u << 0)
#define BLEND_BYPASS        (1u << 1)
#define BLEND_FILL          (1u << 2)
#define BLEND_UPDATE        (1u << 3)
#define BLEND_RST           (1u << 4)

/* SR_SCAL_ROTATE. */
#define SCALE(xi, xf, yi, yf) ((xi) | (xf) << 8 | (yi) << 12 | (yf) << 20)
#define ROT(a)              ((uint32_t)(a) << 24)
#define SR_RST              (1u << 26)
#define SR_START            (1u << 27)
#define MIRROR_X            (1u << 28)
#define MIRROR_Y            (1u << 29)

/* Color modes. */
#define CM_ARGB8888         0
#define CM_RGB888           1
#define CM_RGB565           2
#define CM_L8               4
#define CM_L4               5
#define CM_A8               6
#define CM_A4               7
#define CM_YUV420           8
#define CM_YUV422           9
#define CM_GRAY             12

/* 2D-DMA channel registers and bits. */
#define CONF0               0x00
#define D_INT_RAW           0x04
#define D_INT_CLR           0x10
#define D_FIFO_STATUS       0x14
#define D_STATE             0x24
#define LINK_CONF           0x1c
#define LINK_ADDR           0x20
#define OUT_PERI_SEL        0x38
#define OUT_PORT_BLK        0x6c
#define IN_PERI_SEL         0x3c
#define OUT_LINK_START      (1u << 21)
#define IN_LINK_START       (1u << 22)
#define IN_AUTO_RET         (1u << 20)
#define PORT_EN             (1u << 11)
#define BURST128            (4u << 6)
#define PAGE_BOUND          (1u << 12)
#define OUT_EOF_MODE        (1u << 1)
#define CONF0_RST           (1u << 24)
#define IN_SUC_EOF          (1u << 1)
#define OUT_DSCR_ERR        (1u << 2)

#define OWNER               (1u << 31)
#define EOF_BIT             (1u << 30)
#define EN_2D               (1u << 29)

#define PB_HALF             0
#define PB_1                1
#define PB_1_5              2
#define PB_2                3
#define PB_3                4
#define PB_4                5

/* The PPA's interrupt source, 96, in core 0's interrupt matrix status. */
#define INTMTX_STATUS3      (0x500d6000 + 0x20c)
#define SRC_PPA_BIT         (96 - 96)

#define SOC_CLK_CTRL1       0x500e6018
#define DMA2D_SYS_CLK_EN    (1u << 6)
#define PPA_SYS_CLK_EN      (1u << 9)

/* Where the tests put things. */
#define TX0_DESC            (MEM + 0x000)
#define TX1_DESC            (MEM + 0x040)
#define RX_DESC             (MEM + 0x080)
#define BUF_A               (MEM + 0x1000)
#define BUF_B               (MEM + 0x9000)
#define BUF_OUT             (MEM + 0x11000)

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_writel(qts, SOC_CLK_CTRL1, qtest_readl(qts, SOC_CLK_CTRL1) |
                 DMA2D_SYS_CLK_EN | PPA_SYS_CLK_EN);
    return qts;
}

static void desc_2d(QTestState *qts, uint32_t addr, uint32_t flags,
                    uint32_t hb, uint32_t vb, uint32_t ha, uint32_t va,
                    uint32_t x, uint32_t y, uint32_t pbyte, uint32_t buf)
{
    qtest_writel(qts, addr, OWNER | EN_2D | flags | hb << 14 | vb);
    qtest_writel(qts, addr + 4, pbyte << 28 | ha << 14 | va);
    qtest_writel(qts, addr + 8, x << 14 | y);
    qtest_writel(qts, addr + 12, buf);
    qtest_writel(qts, addr + 16, 0);
}

static void tx_chan(QTestState *qts, int ch, uint32_t sel, uint32_t conf,
                    uint32_t desc)
{
    qtest_writel(qts, OUT(ch) + CONF0, CONF0_RST);
    qtest_writel(qts, OUT(ch) + CONF0, 0);
    qtest_writel(qts, OUT(ch) + D_INT_CLR, 0x1fff);
    qtest_writel(qts, OUT(ch) + OUT_PERI_SEL, sel);
    qtest_writel(qts, OUT(ch) + CONF0, OUT_EOF_MODE | BURST128 | PAGE_BOUND |
                 conf);
    qtest_writel(qts, OUT(ch) + LINK_ADDR, desc);
    qtest_writel(qts, OUT(ch) + LINK_CONF, OUT_LINK_START);
}

static void rx_chan(QTestState *qts, int ch, uint32_t sel, uint32_t conf,
                    uint32_t desc)
{
    qtest_writel(qts, IN(ch) + CONF0, CONF0_RST);
    qtest_writel(qts, IN(ch) + CONF0, 0);
    qtest_writel(qts, IN(ch) + D_INT_CLR, 0x3fff);
    qtest_writel(qts, IN(ch) + IN_PERI_SEL, sel);
    qtest_writel(qts, IN(ch) + CONF0, BURST128 | PAGE_BOUND | conf);
    qtest_writel(qts, IN(ch) + LINK_ADDR, desc);
    qtest_writel(qts, IN(ch) + LINK_CONF, IN_LINK_START);
}

/* Step virtual time until @bits are all set in @reg; false on timeout. */
static bool wait_bits(QTestState *qts, uint32_t reg, uint32_t bits)
{
    for (int i = 0; i < 20000; i++) {
        if ((qtest_readl(qts, reg) & bits) == bits) {
            return true;
        }
        qtest_clock_step(qts, 1000);
    }
    return false;
}

static void fill_mem(QTestState *qts, uint32_t addr, uint8_t v, uint32_t len)
{
    g_autofree uint8_t *b = g_malloc(len);

    memset(b, v, len);
    qtest_memwrite(qts, addr, b, len);
}

/* Reference arithmetic, after the model's documentation */

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static int fdiv256(int v)
{
    return v >= 0 ? v / 256 : -((-v + 255) / 256);
}

static void ref_rgb2yuv601(uint8_t r, uint8_t g, uint8_t b, uint8_t yuv[3])
{
    yuv[0] = clamp8(fdiv256(4096 + 66 * r + 129 * g + 25 * b));
    yuv[1] = clamp8(fdiv256(32768 - 38 * r - 74 * g + 112 * b));
    yuv[2] = clamp8(fdiv256(32768 + 112 * r - 94 * g - 18 * b));
}

static void ref_yuv2rgb601(int y, int u, int v, uint8_t rgb[3])
{
    rgb[0] = clamp8(fdiv256(298 * y + 409 * v - 56906));
    rgb[1] = clamp8(fdiv256(298 * y - 100 * u - 208 * v + 34707));
    rgb[2] = clamp8(fdiv256(298 * y + 516 * u - 70836));
}

/* The blend of the model's documentation: ARGB8888 words. */
static uint32_t ref_blend(uint32_t b, uint32_t f)
{
    uint32_t ab = b >> 24, af = f >> 24;
    uint32_t abp = (ab * (255 - af) + 127) / 255, aout = af + abp;
    uint32_t r = aout ? 65536 / aout : 0, out = aout << 24;

    for (int i = 0; i < 24; i += 8) {
        uint32_t cb = (b >> i) & 0xff, cf = (f >> i) & 0xff;
        uint32_t c = ((cf * af + cb * abp) * r + 32768) >> 16;

        out |= (c > 255 ? 255 : c) << i;
    }
    return out;
}

/* Registers */

/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, DATE), ==, 0x20241220);
    g_assert_cmphex(qtest_readl(qts, SR_FIX_ALPHA), ==, 0x80);
    g_assert_cmphex(qtest_readl(qts, BLEND_FIX_ALPHA), ==, 0x8080);
    g_assert_cmphex(qtest_readl(qts, BLEND_RGB), ==, 0x808080);
    g_assert_cmphex(qtest_readl(qts, CK_FG_LOW), ==, 0xffffff);
    g_assert_cmphex(qtest_readl(qts, CK_BG_LOW), ==, 0xffffff);
    g_assert_cmphex(qtest_readl(qts, CK_FG_HIGH), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR_SCAL_ROTATE), ==, 0x1001);
    g_assert_cmphex(qtest_readl(qts, RGB2GRAY), ==, 0x555655);
    g_assert_cmphex(qtest_readl(qts, PPA + 0x084), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PPA + 0x08c), ==, 4896);
    /* Masks; the start bits read 0. */
    qtest_writel(qts, SR_COLOR_MODE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SR_COLOR_MODE), ==, 0x3fff);
    qtest_writel(qts, SR_COLOR_MODE, 0);
    qtest_writel(qts, BLEND_COLOR_MODE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, BLEND_COLOR_MODE), ==, 0x3ffff);
    qtest_writel(qts, BLEND_COLOR_MODE, 0);
    qtest_writel(qts, SR_FIX_ALPHA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SR_FIX_ALPHA), ==, 0x7ff);
    qtest_writel(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0xf);
    qtest_writel(qts, INT_ENA, 0);
    qtest_quit(qts);
}

/* Fill */

/*
 * A fill of an 80 x 80 RGB565 block at (0, 40) of an 80 x 120 picture,
 * as ESP-IDF's ppa_fill_basic_data_correctness_check, its pixel the ARGB
 * color's top bits; the rest of the picture untouched; then a YUV422 fill
 * with the raw pattern.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_fill(void)
{
    QTestState *qts = start();
    const uint32_t w = 80, h = 120, bw = 80, bh = 80, by = 40;
    g_autofree uint16_t *pic = g_malloc(w * h * 2);
    uint16_t want = (0xff >> 3) << 11 | (0x55 >> 2) << 5 | 0xaa >> 3;

    fill_mem(qts, BUF_OUT, 0xff, w * h * 2);
    desc_2d(qts, RX_DESC, 0, bw, bh, w, h, 0, by, PB_2, BUF_OUT);
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, BLEND_COLOR_MODE, CM_RGB565 << 8);
    qtest_writel(qts, BLEND_FIX_PIXEL, 0x80ff55aa);
    qtest_writel(qts, BLEND_TX_SIZE, bw | bh << 14);
    qtest_writel(qts, INT_ENA, INT_BLEND_EOF);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL | BLEND_UPDATE);
    g_assert_true(wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF));
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_BLEND_EOF, ==,
                    INT_BLEND_EOF);
    g_assert_true(qtest_readl(qts, INTMTX_STATUS3) & 1);
    qtest_memread(qts, BUF_OUT, pic, w * h * 2);
    for (uint32_t i = 0; i < w * h; i++) {
        g_assert_cmphex(pic[i], ==, i >= w * by ? want : 0xffff);
    }
    qtest_writel(qts, INT_CLR, INT_BLEND_EOF);
    g_assert_false(qtest_readl(qts, INTMTX_STATUS3) & 1);

    /* YUV422: the fix pixel's bytes for each pair. */
    desc_2d(qts, RX_DESC, 0, 8, 2, 8, 2, 0, 0, PB_2, BUF_OUT);
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, BLEND_COLOR_MODE, CM_YUV422 << 8);
    qtest_writel(qts, BLEND_FIX_PIXEL, 0xffaaff55);
    qtest_writel(qts, BLEND_TX_SIZE, 8 | 2 << 14);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL | BLEND_UPDATE);
    g_assert_true(wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF));
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, BUF_OUT + 4 * i), ==, 0xffaaff55);
    }
    qtest_quit(qts);
}

/* Blend */

/*
 * ESP-IDF's ppa_blend_basic_data_correctness_check: a 1 x 1 block at
 * (1, 1) of 2 x 2 pictures, the background RGB888 with its alpha scaled
 * by 204/256, the foreground ARGB8888 with its alpha inverted, into the
 * foreground's own buffer: the pixel silicon gives (0x5B, 0x53, 0x96,
 * 0xD8) and the rest of the buffer untouched. Twice: BLEND started at
 * once, and started after the TX channels have read their blocks into
 * their FIFOs and gone idle, as a core configuring the PPA after starting
 * the channels finds them; the blocks' descriptors are still the ones
 * their bytes came from.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_blend_idf(void)
{
    QTestState *qts = start();
    static const uint8_t bg[12] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0x80, 0x40, 0xa0,
    };
    static const uint8_t fg[16] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0x00, 0x80, 0x80, 0xc0,
    };
    static const uint8_t want[16] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0x5b, 0x53, 0x96, 0xd8,
    };
    uint8_t got[16];

    for (int late = 0; late < 2; late++) {
        qtest_memwrite(qts, BUF_A, bg, sizeof(bg));
        qtest_memwrite(qts, BUF_B, fg, sizeof(fg));
        desc_2d(qts, TX0_DESC, EOF_BIT, 1, 1, 2, 2, 1, 1, PB_3, BUF_A);
        desc_2d(qts, TX1_DESC, EOF_BIT, 1, 1, 2, 2, 1, 1, PB_4, BUF_B);
        desc_2d(qts, RX_DESC, 0, 1, 1, 2, 2, 1, 1, PB_4, BUF_B);
        qtest_writel(qts, INT_CLR, 0xf);
        tx_chan(qts, 0, 2, 0, TX0_DESC);
        tx_chan(qts, 1, 3, 0, TX1_DESC);
        rx_chan(qts, 0, 2, 0, RX_DESC);
        if (late) {
            qtest_clock_step(qts, 20000);
            g_assert_cmphex(qtest_readl(qts, OUT(1) + D_STATE) & 0x1fffff,
                            ==, 0);
            g_assert_cmphex(qtest_readl(qts, OUT(1) + D_FIFO_STATUS) >> 24,
                            ==, 4);
        }
        qtest_writel(qts, BLEND_COLOR_MODE, CM_RGB888 | CM_ARGB8888 << 4 |
                     CM_ARGB8888 << 8);
        /* bg: scale by 204 (0.8 * 256); fg: invert. */
        qtest_writel(qts, BLEND_FIX_ALPHA, 204 | 2 << 16 | 1 << 21);
        qtest_writel(qts, BLEND_TX_SIZE, 1 | 1 << 14);
        qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_UPDATE);
        g_assert_true(wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF));
        g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_BLEND_EOF, ==,
                        INT_BLEND_EOF);
        qtest_memread(qts, BUF_B, got, sizeof(got));
        g_assert_cmpmem(got, sizeof(got), want, sizeof(want));
    }
    qtest_quit(qts);
}

/* SRM */

/*
 * ESP-IDF's ppa_srm_basic_data_correctness_check: a 3 x 3 RGB565 block
 * at (1, 1) of a 4 x 4 picture rotated 90 degrees counterclockwise into
 * (1, 0) of another, the input's rows becoming the output's columns.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_srm_idf(void)
{
    QTestState *qts = start();
    static const uint16_t in[16] = {
        0xffff, 0xffff, 0xffff, 0xffff,
        0xffff, 0x8080, 0x8080, 0x8080,
        0xffff, 0x8f80, 0x8f80, 0x8f80,
        0xffff, 0xff80, 0xff80, 0xff80,
    };
    static const uint16_t want[16] = {
        0x0000, 0x8080, 0x8f80, 0xff80,
        0x0000, 0x8080, 0x8f80, 0xff80,
        0x0000, 0x8080, 0x8f80, 0xff80,
        0x0000, 0x0000, 0x0000, 0x0000,
    };
    uint16_t got[16];
    uint8_t gray[16];

    qtest_memwrite(qts, BUF_A, in, sizeof(in));
    fill_mem(qts, BUF_OUT, 0, sizeof(got));
    desc_2d(qts, TX0_DESC, EOF_BIT, 3, 3, 4, 4, 1, 1, PB_2, BUF_A);
    desc_2d(qts, RX_DESC, 0, 2, 2, 4, 4, 1, 0, PB_2, BUF_OUT);
    tx_chan(qts, 0, 1, PORT_EN, TX0_DESC);
    qtest_writel(qts, OUT(0) + OUT_PORT_BLK, 34 | 34 << 14);
    rx_chan(qts, 0, 1, PORT_EN, RX_DESC);
    qtest_writel(qts, SR_COLOR_MODE, CM_RGB565 | CM_RGB565 << 4);
    qtest_writel(qts, SR_SCAL_ROTATE, SCALE(1, 0, 1, 0) | ROT(1) | SR_START);
    g_assert_true(wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF));
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_SR_EOF, ==, INT_SR_EOF);
    qtest_memread(qts, BUF_OUT, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), want, sizeof(want));
    /* The RX descriptor written back with the output block's extent. */
    g_assert_cmphex(qtest_readl(qts, RX_DESC) & 0x7fffffff, ==,
                    EOF_BIT | EN_2D | 3 << 14 | 3);

    /*
     * The same block unrotated to GRAY8 with weights (100, 56, 100), as
     * that test goes on to do: rows of three bytes at an odd offset, the
     * RX descriptor's pbyte 0 (the driver leaves it at whatever it was;
     * the PPA gives the pixel size).
     */
    fill_mem(qts, BUF_OUT, 0, 16);
    desc_2d(qts, TX0_DESC, EOF_BIT, 3, 3, 4, 4, 1, 1, PB_2, BUF_A);
    desc_2d(qts, RX_DESC, 0, 2, 2, 4, 4, 1, 0, PB_HALF, BUF_OUT);
    tx_chan(qts, 0, 1, PORT_EN, TX0_DESC);
    qtest_writel(qts, OUT(0) + OUT_PORT_BLK, 34 | 34 << 14);
    rx_chan(qts, 0, 1, PORT_EN, RX_DESC);
    qtest_writel(qts, INT_CLR, 0xf);
    qtest_writel(qts, RGB2GRAY, 100 | 56 << 8 | 100 << 16);
    qtest_writel(qts, SR_COLOR_MODE, CM_RGB565 | CM_GRAY << 4);
    qtest_writel(qts, SR_SCAL_ROTATE, SCALE(1, 0, 1, 0) | SR_START);
    g_assert_true(wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, BUF_OUT, gray, sizeof(gray));
    for (int i = 0; i < 16; i++) {
        int bx = i % 4 - 1, by = i / 4;
        uint8_t want_gray = 0;

        if (bx >= 0 && by < 3) {
            uint16_t pix = in[(by + 1) * 4 + bx + 1];
            uint32_t r = (pix >> 8) & 0xf8, g = (pix >> 3) & 0xfc;
            uint32_t b = (pix << 3) & 0xf8;

            want_gray = (r * 100 + g * 56 + b * 100) >> 8;
        }
        g_assert_cmpint(gray[i], ==, want_gray);
    }
    qtest_quit(qts);
}

/* SRM, more */

typedef struct SrmOp {
    int in_cm;
    int out_cm;
    uint32_t in_pb;
    uint32_t out_pb;
    uint32_t iw, ih, bw, bh, bx, by;      /* input picture and block */
    uint32_t ow, oh, ox, oy;              /* output picture and offset */
    uint32_t scale;                       /* SR_SCAL_ROTATE's scale bits */
    uint32_t rot;
    bool mirror_x;
    bool mirror_y;
    uint32_t mode_extra;                  /* SR_COLOR_MODE's other bits */
    uint32_t byte_order;                  /* SR_BYTE_ORDER */
    uint32_t fix_alpha;                   /* SR_FIX_ALPHA */
    uint32_t port_w, port_h;
} SrmOp;

/* Run an SRM operation as ESP-IDF's driver does; false if no EOF came. */
static bool run_srm(QTestState *qts, const SrmOp *op, const void *in,
                    size_t in_len, void *out, size_t out_len)
{
    qtest_memwrite(qts, BUF_A, in, in_len);
    fill_mem(qts, BUF_OUT, 0, out_len);
    desc_2d(qts, TX0_DESC, EOF_BIT, op->bw, op->bh, op->iw, op->ih, op->bx,
            op->by, op->in_pb, BUF_A);
    desc_2d(qts, RX_DESC, 0, 2, 2, op->ow, op->oh, op->ox, op->oy,
            op->out_pb, BUF_OUT);
    qtest_writel(qts, SR_SCAL_ROTATE, SR_RST);
    qtest_writel(qts, SR_SCAL_ROTATE, 0);
    tx_chan(qts, 0, 1, PORT_EN, TX0_DESC);
    qtest_writel(qts, OUT(0) + OUT_PORT_BLK, op->port_w | op->port_h << 14);
    rx_chan(qts, 0, 1, PORT_EN, RX_DESC);
    qtest_writel(qts, INT_CLR, 0xf);
    qtest_writel(qts, SR_COLOR_MODE, op->in_cm | op->out_cm << 4 |
                 op->mode_extra);
    qtest_writel(qts, SR_BYTE_ORDER, op->byte_order);
    qtest_writel(qts, SR_FIX_ALPHA, op->fix_alpha);
    qtest_writel(qts, SR_SCAL_ROTATE, op->scale | ROT(op->rot) |
                 (op->mirror_x ? MIRROR_X : 0) |
                 (op->mirror_y ? MIRROR_Y : 0) | SR_START);
    if (!wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF)) {
        return false;
    }
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_SR_EOF, ==, INT_SR_EOF);
    qtest_memread(qts, BUF_OUT, out, out_len);
    return true;
}

/*
 * The model's SRM arithmetic, computed here on ARGB8888 input pixels
 * @in (@w x @h): output pixel (u, v) of a block scaled by @sx16 / 16 and
 * @sy16 / 16, rotated by @rot quarter turns counterclockwise and
 * mirrored.
 */
static uint32_t ref_srm(const uint32_t *in, int w, int h, int sx16, int sy16,
                        int rot, bool mx, bool my, int u, int v)
{
    int sw = w * sx16 / 16, sh = h * sy16 / 16;
    int wo = rot & 1 ? sh : sw, ho = rot & 1 ? sw : sh;
    int a = mx ? wo - 1 - u : u, b = my ? ho - 1 - v : v, p, q;
    int64_t nx, ny, x0, y0, fx, fy;
    uint32_t c[4], out = 0;

    switch (rot) {
    case 0:
        p = a;
        q = b;
        break;
    case 1:
        q = a;
        p = sw - 1 - b;
        break;
    case 2:
        p = sw - 1 - a;
        q = sh - 1 - b;
        break;
    default:
        q = sh - 1 - a;
        p = b;
        break;
    }
    nx = (int64_t)(2 * p + 1) * 16 - sx16;
    ny = (int64_t)(2 * q + 1) * 16 - sy16;
    x0 = nx >= 0 ? nx / (2 * sx16) : -((-nx + 2 * sx16 - 1) / (2 * sx16));
    y0 = ny >= 0 ? ny / (2 * sy16) : -((-ny + 2 * sy16 - 1) / (2 * sy16));
    fx = (nx - x0 * 2 * sx16) * 256 / (2 * sx16);
    fy = (ny - y0 * 2 * sy16) * 256 / (2 * sy16);
    for (int j = 0; j < 4; j++) {
        int x = MIN(MAX(x0 + (j & 1), 0), w - 1);
        int y = MIN(MAX(y0 + (j >> 1), 0), h - 1);

        c[j] = in[y * w + x];
    }
    for (int k = 0; k < 32; k += 8) {
        uint32_t c00 = c[0] >> k & 0xff, c01 = c[1] >> k & 0xff;
        uint32_t c10 = c[2] >> k & 0xff, c11 = c[3] >> k & 0xff;
        uint32_t top = c00 * (256 - fx) + c01 * fx;
        uint32_t bot = c10 * (256 - fx) + c11 * fx;

        out |= ((top * (256 - fy) + bot * fy + 32768) >> 16) << k;
    }
    return out;
}

/*
 * Rotation and mirroring at scale 1 of a 70 x 40 ARGB8888 block (three
 * macroblocks by two, the last ones partial), every combination checked
 * against the permutation of the input, and the RX descriptor written
 * back with the output block's size.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_srm_rotate_mirror(void)
{
    QTestState *qts = start();
    const int w = 70, h = 40;
    g_autofree uint32_t *in = g_new(uint32_t, w * h);
    g_autofree uint32_t *out = g_new(uint32_t, w * h);

    for (int i = 0; i < w * h; i++) {
        in[i] = 0x01000193u * (i + 7) ^ 0x5a5a00ffu;
    }
    for (int rot = 0; rot < 4; rot++) {
        for (int m = 0; m < 4; m++) {
            int ow = rot & 1 ? h : w, oh = rot & 1 ? w : h;
            SrmOp op = {
                .in_cm = CM_ARGB8888, .out_cm = CM_ARGB8888,
                .in_pb = PB_4, .out_pb = PB_4,
                .iw = w, .ih = h, .bw = w, .bh = h,
                .ow = ow, .oh = oh,
                .scale = SCALE(1, 0, 1, 0), .rot = rot,
                .mirror_x = m & 1, .mirror_y = m & 2,
                .fix_alpha = 0x80, .port_w = 34, .port_h = 34,
            };

            if ((rot == 1 || rot == 3) && m == 3) {
                continue;
            }
            g_assert_true(run_srm(qts, &op, in, w * h * 4, out,
                                  w * h * 4));
            for (int v = 0; v < oh; v++) {
                for (int u = 0; u < ow; u++) {
                    int a = op.mirror_x ? ow - 1 - u : u;
                    int b = op.mirror_y ? oh - 1 - v : v;
                    int p, q;

                    switch (rot) {
                    case 0:
                        p = a;
                        q = b;
                        break;
                    case 1:
                        q = a;
                        p = w - 1 - b;
                        break;
                    case 2:
                        p = w - 1 - a;
                        q = h - 1 - b;
                        break;
                    default:
                        q = h - 1 - a;
                        p = b;
                        break;
                    }
                    if (out[v * ow + u] != in[q * w + p]) {
                        g_test_message("rot %d m %d at (%d, %d)", rot, m, u,
                                       v);
                    }
                    g_assert_cmphex(out[v * ow + u], ==, in[q * w + p]);
                }
            }
            g_assert_cmphex(qtest_readl(qts, RX_DESC) & 0x0fffffff, ==,
                            ow << 14 | oh);
        }
    }
    qtest_quit(qts);
}

/*
 * Scaling, by 2, 1.5 (x) and 0.5 (y), and 0.75 with a 90 degree turn, of a
 * 37 x 35 RGB888 block at (2, 3) of a 41 x 40 picture into (1, 2) of the
 * output picture: every pixel as the model's bilinear arithmetic gives
 * it, and the output picture's other pixels untouched.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_srm_scale(void)
{
    QTestState *qts = start();
    const int iw = 41, ih = 40, bw = 37, bh = 35, bx = 2, by = 3;
    static const struct {
        int xi, xf, yi, yf, rot;
    } cases[] = {
        { 2, 0, 2, 0, 0 },
        { 1, 8, 0, 8, 0 },
        { 0, 12, 0, 12, 1 },
        { 3, 4, 1, 0, 2 },
    };
    g_autofree uint8_t *pic = g_malloc(iw * ih * 3);
    g_autofree uint32_t *blk = g_new(uint32_t, bw * bh);

    for (int i = 0; i < iw * ih * 3; i++) {
        pic[i] = (i * 29 + (i / 123) * 7) & 0xff;
    }
    for (int y = 0; y < bh; y++) {
        for (int x = 0; x < bw; x++) {
            const uint8_t *p = &pic[((by + y) * iw + bx + x) * 3];

            blk[y * bw + x] = 0xffu << 24 | p[2] << 16 | p[1] << 8 | p[0];
        }
    }
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        int sx16 = cases[i].xi * 16 + cases[i].xf;
        int sy16 = cases[i].yi * 16 + cases[i].yf;
        int sw = bw * sx16 / 16, sh = bh * sy16 / 16;
        bool quarter = cases[i].rot & 1;
        int wo = quarter ? sh : sw;
        int ho = quarter ? sw : sh;
        int ow = wo + 3, oh = ho + 4;
        g_autofree uint8_t *out = g_malloc(ow * oh * 3);
        SrmOp op = {
            .in_cm = CM_RGB888, .out_cm = CM_RGB888,
            .in_pb = PB_3, .out_pb = PB_3,
            .iw = iw, .ih = ih, .bw = bw, .bh = bh, .bx = bx, .by = by,
            .ow = ow, .oh = oh, .ox = 1, .oy = 2,
            .scale = SCALE(cases[i].xi, cases[i].xf, cases[i].yi,
                           cases[i].yf),
            .rot = cases[i].rot, .fix_alpha = 0x80,
            .port_w = 34, .port_h = 34,
        };

        g_assert_true(run_srm(qts, &op, pic, iw * ih * 3, out, ow * oh * 3));
        for (int v = 0; v < oh; v++) {
            for (int u = 0; u < ow; u++) {
                const uint8_t *px = &out[(v * ow + u) * 3];
                uint32_t want = 0;

                if (u >= 1 && u < 1 + wo && v >= 2 && v < 2 + ho) {
                    want = ref_srm(blk, bw, bh, sx16, sy16, cases[i].rot,
                                   false, false, u - 1, v - 2) & 0xffffff;
                }
                if ((px[0] | px[1] << 8 | px[2] << 16) != want) {
                    g_test_message("case %d at (%d, %d)", i, u, v);
                }
                g_assert_cmphex(px[0] | px[1] << 8 | px[2] << 16, ==, want);
            }
        }
    }
    qtest_quit(qts);
}

/*
 * Color: YUV420 (BT.601, limited) to RGB565, the U and V of each 2 x 2
 * square from its rows' first bytes; RGB565 to GRAY8 as ESP-IDF's test
 * computes it (components widened with zeros, (r 100 + g 56 + b 100) >> 8);
 * RGB888 to YUV422 (U Y0 V Y1, the first pixel's U and V); ARGB8888 with
 * RGB and byte swaps, and each alpha mode; and 16 x 16 macroblocks.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_srm_color(void)
{
    QTestState *qts = start();
    uint8_t yuv[8 * 4 * 3 / 2], gray[16];
    uint16_t rgb565[16], out565[32];
    uint8_t rgb[8 * 2 * 3], out422[8 * 2 * 2];
    uint32_t argb[4], outargb[4];

    /* YUV420 8 x 4 to RGB565. */
    for (int i = 0; i < (int)sizeof(yuv); i++) {
        yuv[i] = 16 + (i * 37) % 220;
    }
    {
        SrmOp op = {
            .in_cm = CM_YUV420, .out_cm = CM_RGB565,
            .in_pb = PB_1_5, .out_pb = PB_2,
            .iw = 8, .ih = 4, .bw = 8, .bh = 4, .ow = 8, .oh = 4,
            .scale = SCALE(1, 0, 1, 0), .fix_alpha = 0x80,
            .port_w = 36, .port_h = 36,
        };

        g_assert_true(run_srm(qts, &op, yuv, sizeof(yuv), out565, 64));
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 8; x++) {
                const uint8_t *row = &yuv[y * 12], *even = &yuv[(y & ~1) * 12];
                const uint8_t *odd = &yuv[(y | 1) * 12];
                int yy = row[(x / 2) * 3 + 1 + (x & 1)];
                uint8_t c[3];

                ref_yuv2rgb601(yy, even[(x / 2) * 3], odd[(x / 2) * 3], c);
                g_assert_cmphex(out565[y * 8 + x], ==,
                                (c[0] >> 3) << 11 | (c[1] >> 2) << 5 |
                                c[2] >> 3);
            }
        }
    }

    /* RGB565 4 x 4 to GRAY8 with weights 100, 56, 100. */
    for (int i = 0; i < 16; i++) {
        rgb565[i] = 0x8080 + i * 0x0f31;
    }
    qtest_writel(qts, RGB2GRAY, 100 | 56 << 8 | 100 << 16);
    {
        SrmOp op = {
            .in_cm = CM_RGB565, .out_cm = CM_GRAY,
            .in_pb = PB_2, .out_pb = PB_1,
            .iw = 4, .ih = 4, .bw = 4, .bh = 4, .ow = 4, .oh = 4,
            .scale = SCALE(1, 0, 1, 0), .fix_alpha = 0x80,
            .port_w = 34, .port_h = 34,
        };

        g_assert_true(run_srm(qts, &op, rgb565, sizeof(rgb565), gray, 16));
        for (int i = 0; i < 16; i++) {
            uint8_t r = (rgb565[i] >> 8) & 0xf8, g = (rgb565[i] >> 3) & 0xfc;
            uint8_t b = (rgb565[i] << 3) & 0xf8;

            g_assert_cmpint(gray[i], ==, (r * 100 + g * 56 + b * 100) >> 8);
        }
    }

    /* RGB888 8 x 2 to YUV422, with 16 x 16 macroblocks (20 x 18 blocks). */
    for (int i = 0; i < (int)sizeof(rgb); i++) {
        rgb[i] = i * 53 + 9;
    }
    {
        SrmOp op = {
            .in_cm = CM_RGB888, .out_cm = CM_YUV422,
            .in_pb = PB_3, .out_pb = PB_2,
            .iw = 8, .ih = 2, .bw = 8, .bh = 2, .ow = 8, .oh = 2,
            .scale = SCALE(1, 0, 1, 0), .byte_order = 1 << 3,
            .fix_alpha = 0x80, .port_w = 18, .port_h = 18,
        };

        g_assert_true(run_srm(qts, &op, rgb, sizeof(rgb), out422,
                              sizeof(out422)));
        for (int i = 0; i < 16; i += 2) {
            uint8_t a[3], b[3];

            ref_rgb2yuv601(rgb[i * 3 + 2], rgb[i * 3 + 1], rgb[i * 3], a);
            ref_rgb2yuv601(rgb[i * 3 + 5], rgb[i * 3 + 4], rgb[i * 3 + 3], b);
            g_assert_cmpint(out422[i * 2], ==, a[1]);
            g_assert_cmpint(out422[i * 2 + 1], ==, a[0]);
            g_assert_cmpint(out422[i * 2 + 2], ==, a[2]);
            g_assert_cmpint(out422[i * 2 + 3], ==, b[0]);
        }
    }

    /* ARGB8888 2 x 2: swaps and alpha modes. */
    argb[0] = 0x11223344;
    argb[1] = 0x80ff0040;
    argb[2] = 0xc0102030;
    argb[3] = 0x00a0b0c0;
    {
        static const struct {
            uint32_t byte_order, fix_alpha;
        } am[] = {
            { 0, 0x080 },               /* none */
            { 0, 0x155 },               /* fix 0x55 */
            { 0, 0x2c0 },               /* scale by 0xc0 */
            { 0, 0x400 },               /* invert */
            { 0, 0x6c0 },               /* invert, then scale */
            { 2, 0x080 },               /* RGB swap */
            { 1, 0x080 },               /* byte swap */
        };

        for (int k = 0; k < ARRAY_SIZE(am); k++) {
            SrmOp op = {
                .in_cm = CM_ARGB8888, .out_cm = CM_ARGB8888,
                .in_pb = PB_4, .out_pb = PB_4,
                .iw = 2, .ih = 2, .bw = 2, .bh = 2, .ow = 2, .oh = 2,
                .scale = SCALE(1, 0, 1, 0), .byte_order = am[k].byte_order,
                .fix_alpha = am[k].fix_alpha, .port_w = 34, .port_h = 34,
            };

            g_assert_true(run_srm(qts, &op, argb, 16, outargb, 16));
            for (int i = 0; i < 4; i++) {
                uint32_t p = argb[i], a = p >> 24, fix = am[k].fix_alpha;

                if (am[k].byte_order == 2) {
                    p = bswap32(p);
                } else if (am[k].byte_order == 1) {
                    p = (p & 0xff00ff00) >> 8 | (p & 0x00ff00ff) << 8;
                }
                a = p >> 24;
                if (fix & 0x400) {
                    a = 255 - a;
                }
                switch ((fix >> 8) & 3) {
                case 1:
                    a = fix & 0xff;
                    break;
                case 2:
                    a = (a * (fix & 0xff)) >> 8;
                    break;
                }
                g_assert_cmphex(outargb[i], ==, (p & 0xffffff) | a << 24);
            }
        }
    }
    qtest_quit(qts);
}

/*
 * The SRM's parameter checks (TRM table 39.6-1): a scale of 0 in X, and a
 * scaled block that does not fit the output picture: SR_PARAM_CFG_ERR,
 * SR_PARAM_ERR_ST's bits, and no output.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_srm_param_errors(void)
{
    QTestState *qts = start();
    uint32_t in[16] = { 0 }, out[16];
    SrmOp op = {
        .in_cm = CM_ARGB8888, .out_cm = CM_ARGB8888,
        .in_pb = PB_4, .out_pb = PB_4,
        .iw = 4, .ih = 4, .bw = 4, .bh = 4, .ow = 4, .oh = 4,
        .scale = SCALE(0, 0, 1, 0), .fix_alpha = 0x80,
        .port_w = 34, .port_h = 34,
    };

    qtest_writel(qts, INT_ENA, INT_SR_PARAM_ERR);
    g_assert_false(run_srm(qts, &op, in, sizeof(in), out, sizeof(out)));
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & 0xf, ==, INT_SR_PARAM_ERR);
    /* X scale 0, and the scaled width 0. */
    g_assert_cmphex(qtest_readl(qts, SR_PARAM_ERR_ST), ==,
                    1u << 6 | 1u << 8);
    g_assert_true(qtest_readl(qts, INTMTX_STATUS3) & 1);

    op.scale = SCALE(2, 0, 1, 0);
    g_assert_false(run_srm(qts, &op, in, sizeof(in), out, sizeof(out)));
    /* The 8-wide output does not fit in the 4-wide picture. */
    g_assert_cmphex(qtest_readl(qts, SR_PARAM_ERR_ST), ==, 1u << 1);
    qtest_quit(qts);
}

/* BLEND, more */

typedef struct BlendOp {
    uint32_t color_mode;      /* BLEND_COLOR_MODE */
    uint32_t bg_pb, fg_pb, out_pb;
    uint32_t w, h;
    uint32_t fix_alpha;
    uint32_t trans;           /* BLEND_TRANS_MODE's mode bits */
} BlendOp;

static bool run_blend(QTestState *qts, const BlendOp *op, const void *bg,
                      size_t bg_len, const void *fg, size_t fg_len,
                      void *out, size_t out_len)
{
    qtest_memwrite(qts, BUF_A, bg, bg_len);
    if (fg) {
        qtest_memwrite(qts, BUF_B, fg, fg_len);
    }
    fill_mem(qts, BUF_OUT, 0xcc, out_len);
    desc_2d(qts, TX0_DESC, EOF_BIT, op->w, op->h, op->w, op->h, 0, 0,
            op->bg_pb, BUF_A);
    desc_2d(qts, TX1_DESC, EOF_BIT, op->w, op->h, op->w, op->h, 0, 0,
            op->fg_pb, BUF_B);
    desc_2d(qts, RX_DESC, 0, op->w, op->h, op->w, op->h, 0, 0, op->out_pb,
            BUF_OUT);
    tx_chan(qts, 0, 2, 0, TX0_DESC);
    if (fg) {
        tx_chan(qts, 1, 3, 0, TX1_DESC);
    }
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, INT_CLR, 0xf);
    qtest_writel(qts, BLEND_COLOR_MODE, op->color_mode);
    qtest_writel(qts, BLEND_FIX_ALPHA, op->fix_alpha);
    qtest_writel(qts, BLEND_TX_SIZE, op->w | op->h << 14);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | op->trans | BLEND_UPDATE);
    if (!wait_bits(qts, IN(0) + D_INT_RAW, IN_SUC_EOF)) {
        return false;
    }
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_BLEND_EOF, ==,
                    INT_BLEND_EOF);
    qtest_memread(qts, BUF_OUT, out, out_len);
    return true;
}

/*
 * Blending ARGB8888 over ARGB8888, every pixel by the model's arithmetic;
 * the color keys (TRM 39.5.4.1): each of the four cases, and
 * COLORKEY_FG_BG_REVERSE; bypass with an A8 foreground replacing the
 * alpha.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_blend_keys(void)
{
    QTestState *qts = start();
    uint32_t bg[64], fg[64], out[64];
    uint8_t a8[64];
    BlendOp op = {
        .color_mode = CM_ARGB8888 | CM_ARGB8888 << 4 | CM_ARGB8888 << 8,
        .bg_pb = PB_4, .fg_pb = PB_4, .out_pb = PB_4, .w = 8, .h = 8,
    };

    for (int i = 0; i < 64; i++) {
        bg[i] = (i * 0x0b2f3d17u) ^ 0x30000000u;
        fg[i] = (i * 0x1f00a335u) ^ 0x00c0ffeeu;
        a8[i] = i * 4 + 3;
    }
    g_assert_true(run_blend(qts, &op, bg, sizeof(bg), fg, sizeof(fg), out,
                            sizeof(out)));
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(out[i], ==, ref_blend(bg[i], fg[i]));
    }

    /* Keys: bg pixels with R below 0x80, fg pixels with G below 0x80. */
    qtest_writel(qts, CK_BG_LOW, 0x000000);
    qtest_writel(qts, CK_BG_HIGH, 0x7fffff);
    qtest_writel(qts, CK_FG_LOW, 0x000000);
    qtest_writel(qts, CK_FG_HIGH, 0xff7fff);
    qtest_writel(qts, CK_DEFAULT, 0x123456);
    for (int reverse = 0; reverse < 2; reverse++) {
        qtest_writel(qts, CK_DEFAULT, 0x123456 | reverse << 24);
        g_assert_true(run_blend(qts, &op, bg, sizeof(bg), fg, sizeof(fg),
                                out, sizeof(out)));
        for (int i = 0; i < 64; i++) {
            bool bk = ((bg[i] >> 16) & 0xff) < 0x80;
            bool fk = ((fg[i] >> 8) & 0xff) < 0x80;
            uint32_t want = ref_blend(bg[i], fg[i]);

            if (fk && !bk) {
                want = bg[i];
            } else if (bk && !fk) {
                want = reverse ? fg[i] : bg[i];
            } else if (bk && fk) {
                want = (want & 0xff000000) | 0x123456;
            }
            g_assert_cmphex(out[i], ==, want);
        }
    }
    qtest_writel(qts, CK_BG_LOW, 0xffffff);
    qtest_writel(qts, CK_BG_HIGH, 0);
    qtest_writel(qts, CK_FG_LOW, 0xffffff);
    qtest_writel(qts, CK_FG_HIGH, 0);

    /* Bypass with an A8 foreground: the background, its alpha replaced. */
    op.color_mode = CM_ARGB8888 | CM_A8 << 4 | CM_ARGB8888 << 8;
    op.fg_pb = PB_1;
    op.trans = BLEND_BYPASS;
    g_assert_true(run_blend(qts, &op, bg, sizeof(bg), a8, sizeof(a8), out,
                            sizeof(out)));
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(out[i], ==, (bg[i] & 0xffffff) | a8[i] << 24);
    }
    qtest_quit(qts);
}

/*
 * The CLUTs: an L8 background through CLUT0 written in FIFO mode, an L4
 * foreground (low nibble first) through CLUT1 written in memory mode, an
 * A4 foreground with BLEND_RGB (alpha a * 17); BLEND1_CLUT_DATA's pointers
 * and CLUT_CNT; and a CLUT powered down losing its contents.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_blend_clut(void)
{
    QTestState *qts = start();
    uint32_t clut0[16], clut1[16], out[32];
    uint8_t l8[32], l4[16], a4[16];
    BlendOp op = {
        .color_mode = CM_L8 | CM_L4 << 4 | CM_ARGB8888 << 8,
        .bg_pb = PB_1, .fg_pb = PB_HALF, .out_pb = PB_4, .w = 8, .h = 4,
    };

    for (int i = 0; i < 16; i++) {
        clut0[i] = 0xff000000u | (i * 0x00102030u);
        clut1[i] = (i * 0x10u) << 24 | (0x00ffffff - i * 0x00010203u);
    }
    qtest_writel(qts, CLUT_CONF, 1 << 1 | 1 << 2);
    qtest_writel(qts, CLUT_CONF, 0);
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, BLEND0_CLUT_DATA, clut0[i]);
    }
    g_assert_cmphex(qtest_readl(qts, CLUT_CNT), ==, 16);
    /* Read back from the start. */
    qtest_writel(qts, CLUT_CONF, 1 << 3);
    qtest_writel(qts, CLUT_CONF, 0);
    for (int i = 0; i < 16; i++) {
        g_assert_cmphex(qtest_readl(qts, BLEND0_CLUT_DATA), ==, clut0[i]);
    }
    qtest_writel(qts, CLUT_CONF, 1);
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, CLUT1_MEM + 4 * i, clut1[i]);
    }
    g_assert_cmphex(qtest_readl(qts, CLUT0_MEM + 4 * 5), ==, clut0[5]);
    for (int i = 0; i < 32; i++) {
        l8[i] = (i * 7) & 15;
    }
    for (int i = 0; i < 16; i++) {
        l4[i] = (i * 3 & 15) | ((i * 5 + 1) & 15) << 4;
        a4[i] = (i & 15) | ((15 - i) & 15) << 4;
    }
    g_assert_true(run_blend(qts, &op, l8, sizeof(l8), l4, sizeof(l4), out,
                            sizeof(out)));
    for (int i = 0; i < 32; i++) {
        uint32_t f = clut1[(l4[i / 2] >> (4 * (i & 1))) & 15];

        g_assert_cmphex(out[i], ==, ref_blend(clut0[l8[i]], f));
    }

    op.color_mode = CM_L8 | CM_A4 << 4 | CM_ARGB8888 << 8;
    qtest_writel(qts, BLEND_RGB, 0x405060);
    g_assert_true(run_blend(qts, &op, l8, sizeof(l8), a4, sizeof(a4), out,
                            sizeof(out)));
    for (int i = 0; i < 32; i++) {
        uint32_t a = (a4[i / 2] >> (4 * (i & 1))) & 15;

        g_assert_cmphex(out[i], ==,
                        ref_blend(clut0[l8[i]], a * 17 << 24 | 0x405060));
    }

    /* Powered down (FORCE_PD), the CLUTs lose their contents. */
    qtest_writel(qts, CLUT_CONF, 1 | 1 << 5);
    g_assert_cmphex(qtest_readl(qts, CLUT0_MEM + 4 * 5), ==, 0);
    qtest_writel(qts, CLUT_CONF, 1);
    g_assert_cmphex(qtest_readl(qts, CLUT0_MEM + 4 * 5), ==, 0);
    qtest_quit(qts);
}

/*
 * YUV backgrounds and outputs: a YUV422 (Y0 U Y1 V, order 2) background
 * to RGB888 with a transparent foreground; YUV420 full range (BT.709) to
 * ARGB8888; and ARGB8888 to GRAY8 and to YUV420 (BT.601, limited); and a
 * size mismatch's BLEND_PARAM_CFG_ERR.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_blend_yuv(void)
{
    QTestState *qts = start();
    uint8_t yuv[8 * 4 * 2], out[8 * 4 * 4];
    uint32_t fg[32], argb[32];
    BlendOp op = {
        .color_mode = CM_YUV422 | CM_ARGB8888 << 4 | CM_RGB888 << 8 |
                      2 << 16,
        .bg_pb = PB_2, .fg_pb = PB_4, .out_pb = PB_3, .w = 8, .h = 4,
    };

    for (int i = 0; i < (int)sizeof(yuv); i++) {
        yuv[i] = 16 + (i * 41) % 220;
    }
    for (int i = 0; i < 32; i++) {
        fg[i] = 0x00ffffff;
        argb[i] = 0xff000000u | (i * 0x070b0d);
    }
    g_assert_true(run_blend(qts, &op, yuv, sizeof(yuv), fg, sizeof(fg), out,
                            8 * 4 * 3));
    for (int i = 0; i < 32; i++) {
        const uint8_t *p = &yuv[(i & ~1) * 2];
        uint8_t c[3];

        /* Order 2: Y0 U Y1 V. */
        ref_yuv2rgb601(p[(i & 1) * 2], p[1], p[3], c);
        g_assert_cmpint(out[i * 3], ==, c[2]);
        g_assert_cmpint(out[i * 3 + 1], ==, c[1]);
        g_assert_cmpint(out[i * 3 + 2], ==, c[0]);
    }

    /* ARGB8888 to GRAY8 (default weights 85, 86, 85). */
    op.color_mode = CM_ARGB8888 | CM_ARGB8888 << 4 | CM_GRAY << 8;
    op.bg_pb = PB_4;
    op.out_pb = PB_1;
    g_assert_true(run_blend(qts, &op, argb, sizeof(argb), fg, sizeof(fg),
                            out, 32));
    for (int i = 0; i < 32; i++) {
        uint32_t p = argb[i];

        g_assert_cmpint(out[i], ==,
                        (((p >> 16) & 0xff) * 85 + ((p >> 8) & 0xff) * 86 +
                         (p & 0xff) * 85) >> 8);
    }

    /* ARGB8888 to YUV420: U Y0 Y1 on even rows, V Y0 Y1 on odd. */
    op.color_mode = CM_ARGB8888 | CM_ARGB8888 << 4 | CM_YUV420 << 8;
    op.out_pb = PB_1_5;
    g_assert_true(run_blend(qts, &op, argb, sizeof(argb), fg, sizeof(fg),
                            out, 48));
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 8; x += 2) {
            uint32_t p0 = argb[y * 8 + x], p1 = argb[y * 8 + x + 1];
            uint8_t a[3], b[3];
            const uint8_t *o = &out[y * 12 + (x / 2) * 3];

            ref_rgb2yuv601(p0 >> 16, p0 >> 8, p0, a);
            ref_rgb2yuv601(p1 >> 16, p1 >> 8, p1, b);
            g_assert_cmpint(o[0], ==, y & 1 ? a[2] : a[1]);
            g_assert_cmpint(o[1], ==, a[0]);
            g_assert_cmpint(o[2], ==, b[0]);
        }
    }

    /* The foreground's block differs from the background's. */
    op.color_mode = CM_ARGB8888 | CM_ARGB8888 << 4 | CM_ARGB8888 << 8;
    op.out_pb = PB_4;
    qtest_memwrite(qts, BUF_A, argb, sizeof(argb));
    desc_2d(qts, TX0_DESC, EOF_BIT, 8, 4, 8, 4, 0, 0, PB_4, BUF_A);
    desc_2d(qts, TX1_DESC, EOF_BIT, 4, 4, 8, 4, 0, 0, PB_4, BUF_B);
    desc_2d(qts, RX_DESC, 0, 8, 4, 8, 4, 0, 0, PB_4, BUF_OUT);
    tx_chan(qts, 0, 2, 0, TX0_DESC);
    tx_chan(qts, 1, 3, 0, TX1_DESC);
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, INT_CLR, 0xf);
    qtest_writel(qts, BLEND_COLOR_MODE, op.color_mode);
    qtest_writel(qts, BLEND_TX_SIZE, 0);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_UPDATE);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & 0xf, ==, INT_BLEND_PARAM_ERR);
    g_assert_cmphex(qtest_readl(qts, BLEND_ST), ==, 1);
    g_assert_cmphex(qtest_readl(qts, IN(0) + D_INT_RAW) & IN_SUC_EOF, ==, 0);
    qtest_quit(qts);
}

/* Timing */

static uint64_t clk_hz(QTestState *qts, const char *path)
{
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
 * The engines' pace at their clock: a 64 x 64 ARGB8888 blend takes a cycle
 * a pixel at least, the 2D-DMA reading two layers and writing one at 8
 * bytes a cycle at the same clock taking longer; a fill of a 64 x 64
 * RGB565 block, the 2D-DMA's 8 bytes a cycle; an SRM of 64 x 64 a cycle an
 * output pixel at least. No EOF comes before.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_timing(void)
{
    QTestState *qts = start();
    uint64_t hz = clk_hz(qts, "/machine/ppa/clk");
    uint64_t dhz = clk_hz(qts, "/machine/dma2d/clk");
    int64_t cycle, t0, t;
    g_autofree uint32_t *pic = g_new0(uint32_t, 64 * 64);
    BlendOp op = {
        .color_mode = CM_ARGB8888 | CM_ARGB8888 << 4 | CM_ARGB8888 << 8,
        .bg_pb = PB_4, .fg_pb = PB_4, .out_pb = PB_4, .w = 64, .h = 64,
    };

    g_assert_cmpuint(hz, ==, dhz);
    cycle = 1000000000 / hz;
    qtest_memwrite(qts, BUF_A, pic, 64 * 64 * 4);
    qtest_memwrite(qts, BUF_B, pic, 64 * 64 * 4);
    desc_2d(qts, TX0_DESC, EOF_BIT, 64, 64, 64, 64, 0, 0, PB_4, BUF_A);
    desc_2d(qts, TX1_DESC, EOF_BIT, 64, 64, 64, 64, 0, 0, PB_4, BUF_B);
    desc_2d(qts, RX_DESC, 0, 64, 64, 64, 64, 0, 0, PB_4, BUF_OUT);
    tx_chan(qts, 0, 2, 0, TX0_DESC);
    tx_chan(qts, 1, 3, 0, TX1_DESC);
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, BLEND_COLOR_MODE, op.color_mode);
    qtest_writel(qts, BLEND_TX_SIZE, 64 | 64 << 14);
    t0 = qtest_clock_step(qts, 1);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_UPDATE);
    t = t0;
    while (!(qtest_readl(qts, IN(0) + D_INT_RAW) & IN_SUC_EOF)) {
        t = qtest_clock_step(qts, cycle);
    }
    g_test_message("blend 64 x 64: %" PRId64 " cycles", (t - t0) / cycle);
    /* Two layers of 16 KiB in at 8 bytes a cycle, sharing the reads. */
    g_assert_cmpint(t - t0, >=, 4096 * cycle);
    g_assert_cmpint(t - t0, <=, 4096 * cycle + 4096 * cycle / 4);

    /* A fill of 64 x 64 RGB565: 8 KiB at 8 bytes a cycle. */
    desc_2d(qts, RX_DESC, 0, 64, 64, 64, 64, 0, 0, PB_2, BUF_OUT);
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, BLEND_COLOR_MODE, CM_RGB565 << 8);
    t0 = qtest_clock_step(qts, 1);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL | BLEND_UPDATE);
    t = t0;
    while (!(qtest_readl(qts, IN(0) + D_INT_RAW) & IN_SUC_EOF)) {
        t = qtest_clock_step(qts, cycle);
    }
    g_test_message("fill 64 x 64: %" PRId64 " cycles", (t - t0) / cycle);
    g_assert_cmpint(t - t0, >=, 1024 * cycle);
    g_assert_cmpint(t - t0, <=, 1024 * cycle + 1024 * cycle / 4);

    /* An SRM of 64 x 64 ARGB8888 at scale 1. */
    {
        SrmOp sop = {
            .in_cm = CM_ARGB8888, .out_cm = CM_ARGB8888,
            .in_pb = PB_4, .out_pb = PB_4,
            .iw = 64, .ih = 64, .bw = 64, .bh = 64, .ow = 64, .oh = 64,
            .scale = SCALE(1, 0, 1, 0), .fix_alpha = 0x80,
            .port_w = 34, .port_h = 34,
        };

        desc_2d(qts, TX0_DESC, EOF_BIT, 64, 64, 64, 64, 0, 0, PB_4, BUF_A);
        desc_2d(qts, RX_DESC, 0, 2, 2, 64, 64, 0, 0, PB_4, BUF_OUT);
        tx_chan(qts, 0, 1, PORT_EN, TX0_DESC);
        qtest_writel(qts, OUT(0) + OUT_PORT_BLK, 34 | 34 << 14);
        rx_chan(qts, 0, 1, PORT_EN, RX_DESC);
        qtest_writel(qts, SR_COLOR_MODE, 0);
        t0 = qtest_clock_step(qts, 1);
        qtest_writel(qts, SR_SCAL_ROTATE, sop.scale | SR_START);
        t = t0;
        while (!(qtest_readl(qts, IN(0) + D_INT_RAW) & IN_SUC_EOF)) {
            t = qtest_clock_step(qts, cycle);
        }
        g_test_message("SRM 64 x 64: %" PRId64 " cycles", (t - t0) / cycle);
        g_assert_cmpint(t - t0, >=, 4096 * cycle);
        g_assert_cmpint(t - t0, <=, 4096 * cycle * 3);
    }
    qtest_quit(qts);
}

/*
 * The engines' resets: SCAL_ROTATE_RST and BLEND_RST abandon an operation
 * under way, with no EOF; a start while held in reset starts nothing;
 * DBG_REPLACE_SEL 6 replaces BLEND's output with DBG_REPLACE_DATA.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_reset_debug(void)
{
    QTestState *qts = start();
    uint32_t bg[16] = { 0 }, out[16];
    BlendOp op = {
        .color_mode = CM_ARGB8888 | CM_ARGB8888 << 4 | CM_ARGB8888 << 8,
        .bg_pb = PB_4, .fg_pb = PB_4, .out_pb = PB_4, .w = 4, .h = 4,
    };

    qtest_writel(qts, PPA + 0x090, 6);
    qtest_writel(qts, PPA + 0x094, 0x12345678);
    g_assert_true(run_blend(qts, &op, bg, sizeof(bg), bg, sizeof(bg), out,
                            sizeof(out)));
    for (int i = 0; i < 16; i++) {
        g_assert_cmphex(out[i], ==, 0x12345678);
    }
    qtest_writel(qts, PPA + 0x090, 0);

    /* A fill abandoned. */
    desc_2d(qts, RX_DESC, 0, 64, 64, 64, 64, 0, 0, PB_4, BUF_OUT);
    rx_chan(qts, 0, 2, 0, RX_DESC);
    qtest_writel(qts, INT_CLR, 0xf);
    qtest_writel(qts, BLEND_COLOR_MODE, 0);
    qtest_writel(qts, BLEND_TX_SIZE, 64 | 64 << 14);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL | BLEND_UPDATE);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL | BLEND_RST);
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL);
    qtest_clock_step(qts, 10000000);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_BLEND_EOF, ==, 0);
    g_assert_cmphex(qtest_readl(qts, IN(0) + D_INT_RAW) & IN_SUC_EOF, ==, 0);
    /* Held in reset, an update starts nothing. */
    qtest_writel(qts, BLEND_TRANS_MODE, BLEND_EN | BLEND_FILL | BLEND_RST |
                 BLEND_UPDATE);
    qtest_clock_step(qts, 10000000);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_BLEND_EOF, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32p4/ppa/reset-values", test_reset_values);
    qtest_add_func("esp32p4/ppa/fill", test_fill);
    qtest_add_func("esp32p4/ppa/blend-idf", test_blend_idf);
    qtest_add_func("esp32p4/ppa/srm-idf", test_srm_idf);
    qtest_add_func("esp32p4/ppa/srm-rotate-mirror", test_srm_rotate_mirror);
    qtest_add_func("esp32p4/ppa/srm-scale", test_srm_scale);
    qtest_add_func("esp32p4/ppa/srm-color", test_srm_color);
    qtest_add_func("esp32p4/ppa/srm-param-errors", test_srm_param_errors);
    qtest_add_func("esp32p4/ppa/blend-keys", test_blend_keys);
    qtest_add_func("esp32p4/ppa/blend-clut", test_blend_clut);
    qtest_add_func("esp32p4/ppa/blend-yuv", test_blend_yuv);
    qtest_add_func("esp32p4/ppa/timing", test_timing);
    qtest_add_func("esp32p4/ppa/reset-debug", test_reset_debug);
    return g_test_run();
}
