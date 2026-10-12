/*
 * QTest testcase for the ESP32-P4 2D-DMA controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The tests build descriptors in L2MEM, past the 128 KiB the L2 cache
 * holds from reset, run memory-to-memory transfers and play the JPEG
 * codec through the controller's "periph" hook, stepping virtual time
 * while the channels move the data at DMA2D_SYS_CLK. Interrupts are seen
 * in the interrupt matrix's status registers, which follow every source's
 * level.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define DMA2D           0x50088000
#define OUT(ch)         (DMA2D + 0x100 * (ch))
#define IN(ch)          (DMA2D + 0x500 + 0x100 * (ch))
#define MEM             0x4ff40000

/* Within a channel's block (the TX layout; RX's differs past 0x24). */
#define CONF0           0x00
#define INT_RAW         0x04
#define INT_ENA         0x08
#define INT_ST          0x0c
#define INT_CLR         0x10
#define FIFO_STATUS     0x14
#define PUSHPOP         0x18
#define LINK_CONF       0x1c
#define LINK_ADDR       0x20
#define STATE           0x24
#define OUT_EOF_DES     0x28
#define OUT_DSCR        0x2c
#define OUT_DSCR_BF0    0x30
#define OUT_PERI_SEL    0x38
#define OUT_ARB         0x3c
#define OUT_RO_STATUS   0x40
#define OUT_RO_PD_CONF  0x44
#define OUT_COLOR       0x48
#define OUT_SCRAMBLE    0x4c
#define OUT_PARAM(n)    (0x50 + 4 * (n))
#define OUT_ETM_CONF    0x68
#define OUT_PORT_BLK    0x6c
#define IN_SUC_EOF_DES  0x28
#define IN_ERR_EOF_DES  0x2c
#define IN_DSCR         0x30
#define IN_PERI_SEL     0x3c
#define IN_ARB          0x40
#define IN_RO_PD_CONF   0x48
#define IN_COLOR        0x4c
#define IN_SCRAMBLE     0x50
#define IN_PARAM(n)     (0x54 + 4 * (n))
#define IN_ETM_CONF     0x6c

#define AXI_ERR         (DMA2D + 0xa00)
#define RST_CONF        (DMA2D + 0xa04)
#define INTR_START      (DMA2D + 0xa08)
#define INTR_END        (DMA2D + 0xa0c)
#define EXTR_START      (DMA2D + 0xa10)
#define EXTR_END        (DMA2D + 0xa14)
#define OUT_ARB_CONFIG  (DMA2D + 0xa18)
#define IN_ARB_CONFIG   (DMA2D + 0xa1c)
#define DATE            (DMA2D + 0xa2c)

/* Interrupt bits. */
#define OUT_DONE        (1u << 0)
#define OUT_EOF         (1u << 1)
#define OUT_DSCR_ERR    (1u << 2)
#define OUT_TOTAL_EOF   (1u << 3)
#define OUT_TASK_OVF    (1u << 12)
#define IN_DONE         (1u << 0)
#define IN_SUC_EOF      (1u << 1)
#define IN_ERR_EOF      (1u << 2)
#define IN_DSCR_ERR     (1u << 3)
#define IN_DSCR_EMPTY   (1u << 10)

/* Link and configuration bits. */
#define OUT_LINK_STOP   (1u << 20)
#define OUT_LINK_START  (1u << 21)
#define OUT_LINK_RESTART (1u << 22)
#define OUT_LINK_PARK   (1u << 23)
#define IN_AUTO_RET     (1u << 20)
#define IN_LINK_START   (1u << 22)
#define IN_LINK_PARK    (1u << 24)
#define OUT_AUTO_WRBACK (1u << 0)
#define OUT_EOF_MODE    (1u << 1)
#define IN_MEM_TRANS    (1u << 0)
#define CHECK_OWNER     (1u << 4)
#define BURST(n)        ((n) << 6)      /* 0: 8 bytes ... 4: 128 */
#define MB_SIZE(n)      ((n) << 9)      /* 0: 8x8, 1: 16x8, 2: 16x16 */
#define PORT_EN         (1u << 11)
#define REORDER_EN      (1u << 16)
#define CONF0_RST       (1u << 24)

/* Descriptor words. */
#define OWNER           (1u << 31)
#define EOF_BIT         (1u << 30)
#define EN_2D           (1u << 29)
#define ERR_EOF         (1u << 28)
#define MOD1            (1u << 28)

/* pbyte. */
#define PB_HALF         0
#define PB_1            1
#define PB_1_5          2
#define PB_2            3
#define PB_3            4
#define PB_4            5

/*
 * Interrupt sources, in core 0's interrupt matrix status: words 0 to 3 at
 * 0x200, the sources from 128 at 0x220.
 */
#define INTMTX_STATUS(w)    (0x500d6000 + ((w) < 4 ? 0x200 + 4 * (w) : 0x220))
#define SRC_DMA2D_IN0       104
#define SRC_DMA2D_OUT0      106
#define SRC_DMA2D_IN2       128
#define SRC_DMA2D_OUT3      129

#define SOC_CLK_CTRL1       0x500e6018
#define DMA2D_SYS_CLK_EN    (1u << 6)
#define HP_RST_EN0          0x500e60c0
#define RST_EN_DMA2D        (1u << 31)

#define DMA_PMS             0x500a6000
#define DMA_DMA2D_DUMMY_W   (DMA_PMS + 0x210)

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | DMA2D_SYS_CLK_EN);
    return qts;
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

/* A descriptor of five words. */
static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t dw1,
                 uint32_t dw2, uint32_t buf, uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, dw1);
    qtest_writel(qts, addr + 8, dw2);
    qtest_writel(qts, addr + 12, buf);
    qtest_writel(qts, addr + 16, next);
}

/* 1D: length and size of up to 28 bits each. */
static void desc_1d(QTestState *qts, uint32_t addr, uint32_t flags,
                    uint32_t len, uint32_t size, uint32_t buf, uint32_t next)
{
    desc(qts, addr, flags | (len & 0x3fff) << 14 | (size & 0x3fff),
         (len >> 14) << 14 | size >> 14, 0, buf, next);
}

/* 2D: block hb x vb at (x, y) of picture ha x va. */
static void desc_2d(QTestState *qts, uint32_t addr, uint32_t flags,
                    uint32_t hb, uint32_t vb, uint32_t ha, uint32_t va,
                    uint32_t x, uint32_t y, uint32_t pbyte, uint32_t mod,
                    uint32_t buf, uint32_t next)
{
    desc(qts, addr, flags | EN_2D | hb << 14 | vb,
         pbyte << 28 | ha << 14 | va, mod | x << 14 | y, buf, next);
}

static void fill(QTestState *qts, uint32_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *b = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        b[i] = seed + i * 7 + (i >> 8);
    }
    qtest_memwrite(qts, addr, b, len);
}

static void memset_q(QTestState *qts, uint32_t addr, uint8_t v, uint32_t len)
{
    g_autofree uint8_t *b = g_malloc(len);

    memset(b, v, len);
    qtest_memwrite(qts, addr, b, len);
}

/* Step virtual time until @bits are all set in @reg; false on timeout. */
static bool wait_bits(QTestState *qts, uint32_t reg, uint32_t bits)
{
    for (int i = 0; i < 4000; i++) {
        if ((qtest_readl(qts, reg) & bits) == bits) {
            return true;
        }
        qtest_clock_step(qts, 1000);
    }
    return false;
}

/* Reset channel @ch of both directions and set it up for M2M. */
static void m2m_setup(QTestState *qts, int ch, uint32_t out_conf,
                      uint32_t in_conf, uint32_t out_desc, uint32_t in_desc)
{
    qtest_writel(qts, OUT(ch) + CONF0, CONF0_RST);
    qtest_writel(qts, OUT(ch) + CONF0, 0);
    qtest_writel(qts, IN(ch) + CONF0, CONF0_RST);
    qtest_writel(qts, IN(ch) + CONF0, 0);
    qtest_writel(qts, OUT(ch) + INT_CLR, 0x1fff);
    qtest_writel(qts, IN(ch) + INT_CLR, 0x3fff);
    qtest_writel(qts, OUT(ch) + OUT_PERI_SEL, 4 + ch);
    qtest_writel(qts, IN(ch) + IN_PERI_SEL, 3 + ch);
    qtest_writel(qts, OUT(ch) + CONF0, out_conf);
    qtest_writel(qts, IN(ch) + CONF0, in_conf | IN_MEM_TRANS);
    qtest_writel(qts, OUT(ch) + LINK_ADDR, out_desc);
    qtest_writel(qts, IN(ch) + LINK_ADDR, in_desc);
}

static void m2m_go(QTestState *qts, int ch)
{
    qtest_writel(qts, OUT(ch) + LINK_CONF, OUT_LINK_START);
    qtest_writel(qts, IN(ch) + LINK_CONF, IN_LINK_START | IN_AUTO_RET);
}

/* Registers and reset values */

/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, DATE), ==, 0x2412190);
    g_assert_cmphex(qtest_readl(qts, INTR_START), ==, 0x30100000);
    g_assert_cmphex(qtest_readl(qts, INTR_END), ==, 0x8fffffff);
    g_assert_cmphex(qtest_readl(qts, EXTR_START), ==, 0x30100000);
    g_assert_cmphex(qtest_readl(qts, EXTR_END), ==, 0x8fffffff);
    g_assert_cmphex(qtest_readl(qts, DMA2D + 0xa24), ==, 0xffffffff);
    for (int ch = 0; ch < 4; ch++) {
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + CONF0), ==, 0x2);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + LINK_CONF), ==,
                        OUT_LINK_PARK);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + STATE), ==, 0x01000000);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_PERI_SEL), ==, 7);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_ARB), ==, 0x11);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + FIFO_STATUS), ==,
                        0x817f82);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_COLOR), ==, 0x38);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_PARAM(0)), ==,
                        0x12a);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_PARAM(1)), ==,
                        0xc86d999);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_PARAM(5)), ==,
                        0xbad3000);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_ETM_CONF), ==, 0x4);
        g_assert_cmphex(qtest_readl(qts, OUT(ch) + OUT_PORT_BLK), ==,
                        18 | 18 << 14);
    }
    g_assert_cmphex(qtest_readl(qts, OUT(0) + OUT_RO_PD_CONF), ==, 0x20);
    for (int ch = 0; ch < 3; ch++) {
        g_assert_cmphex(qtest_readl(qts, IN(ch) + CONF0), ==, 0);
        g_assert_cmphex(qtest_readl(qts, IN(ch) + LINK_CONF), ==,
                        IN_AUTO_RET | IN_LINK_PARK);
        g_assert_cmphex(qtest_readl(qts, IN(ch) + STATE), ==, 0x00800000);
        g_assert_cmphex(qtest_readl(qts, IN(ch) + IN_PERI_SEL), ==, 7);
        g_assert_cmphex(qtest_readl(qts, IN(ch) + PUSHPOP), ==, 0x400);
        g_assert_cmphex(qtest_readl(qts, IN(ch) + FIFO_STATUS), ==,
                        0x810002);
        g_assert_cmphex(qtest_readl(qts, IN(ch) + IN_ETM_CONF), ==, 0x4);
    }
    g_assert_cmphex(qtest_readl(qts, IN(0) + IN_COLOR), ==, 0x38);
    g_assert_cmphex(qtest_readl(qts, IN(0) + IN_PARAM(2)), ==, 0x1e712a);
    g_assert_cmphex(qtest_readl(qts, IN(0) + IN_RO_PD_CONF), ==, 0x20);
    /* Channel 0 alone of the RX channels converts. */
    g_assert_cmphex(qtest_readl(qts, IN(1) + IN_COLOR), ==, 0);
    qtest_writel(qts, IN(1) + IN_COLOR, 0x3f);
    g_assert_cmphex(qtest_readl(qts, IN(1) + IN_COLOR), ==, 0);

    /* Masks, write-1-to-clear and read-only registers. */
    qtest_writel(qts, OUT(1) + CONF0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OUT(1) + CONF0), ==, 0x07011fff);
    qtest_writel(qts, OUT(1) + CONF0, 0x2);
    qtest_writel(qts, IN(2) + CONF0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, IN(2) + CONF0), ==, 0x07011ffd);
    qtest_writel(qts, IN(2) + CONF0, 0);
    qtest_writel(qts, OUT(0) + INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_ENA), ==, 0x1fff);
    qtest_writel(qts, IN(0) + INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, IN(0) + INT_ENA), ==, 0x3fff);
    qtest_writel(qts, OUT(0) + OUT_DSCR, 0x1234);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + OUT_DSCR), ==, 0);
    qtest_writel(qts, OUT(2) + OUT_PARAM(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + OUT_PARAM(1)), ==, 0xfffffff);
    qtest_writel(qts, OUT(2) + OUT_PARAM(0), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + OUT_PARAM(0)), ==, 0x1fffff);
    qtest_quit(qts);
}

/* Memory to memory */

/*
 * 1D: a chain of two TX descriptors into two RX descriptors, the
 * write-backs (owner, length, suc_eof), the EOF descriptor addresses and
 * the interrupts through the matrix, and the transfer's duration.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_m2m_1d(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x8000;
    g_autofree uint8_t *a = g_malloc(20000);
    g_autofree uint8_t *b = g_malloc(20000);

    fill(qts, src, 20000, 3);
    memset_q(qts, dst, 0, 20000);
    desc_1d(qts, tx, OWNER, 16384 + 616, 16384 + 616, src, tx + 0x20);
    desc_1d(qts, tx + 0x20, OWNER | EOF_BIT, 3000, 3000, src + 17000, 0);
    desc_1d(qts, rx, OWNER, 0, 12000, dst, rx + 0x20);
    desc_1d(qts, rx + 0x20, OWNER, 0, 10000, dst + 12000, 0);
    m2m_setup(qts, 0, OUT_AUTO_WRBACK | OUT_EOF_MODE | BURST(4), BURST(4),
              tx, rx);
    qtest_writel(qts, IN(0) + INT_ENA, IN_SUC_EOF);
    qtest_writel(qts, OUT(0) + INT_ENA, OUT_TOTAL_EOF);
    m2m_go(qts, 0);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + LINK_CONF) & OUT_LINK_PARK, ==,
                    0);
    g_assert_cmphex(qtest_readl(qts, IN(0) + INT_RAW) & IN_SUC_EOF, ==, 0);
    g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF | IN_DONE));
    g_assert_true(source_level(qts, SRC_DMA2D_IN0));
    g_assert_true(source_level(qts, SRC_DMA2D_OUT0));

    qtest_memread(qts, src, a, 20000);
    qtest_memread(qts, dst, b, 20000);
    g_assert_cmpmem(a, 17000, b, 12000 + 5000);
    g_assert_cmpmem(a + 17000, 3000, b + 17000, 3000);
    /* RX write-backs: owner (AUTO_RET), suc_eof and the lengths. */
    g_assert_cmphex(qtest_readl(qts, rx), ==, OWNER | (12000 << 14) | 12000);
    g_assert_cmphex(qtest_readl(qts, rx + 0x20), ==,
                    OWNER | EOF_BIT | (8000 << 14) | 10000);
    /* TX write-backs: owner cleared. */
    g_assert_cmphex(qtest_readl(qts, tx) & OWNER, ==, 0);
    g_assert_cmphex(qtest_readl(qts, tx + 0x20) & OWNER, ==, 0);
    g_assert_cmphex(qtest_readl(qts, IN(0) + IN_SUC_EOF_DES), ==, rx + 0x20);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + OUT_EOF_DES), ==, tx + 0x20);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_RAW) &
                    (OUT_DONE | OUT_EOF | OUT_TOTAL_EOF), ==,
                    OUT_DONE | OUT_EOF | OUT_TOTAL_EOF);
    g_assert_cmphex(qtest_readl(qts, IN(0) + LINK_CONF) & IN_LINK_PARK, ==,
                    IN_LINK_PARK);
    g_assert_cmphex(qtest_readl(qts, IN(0) + STATE) & 0x700000, ==, 0);
    qtest_writel(qts, IN(0) + INT_CLR, IN_SUC_EOF);
    g_assert_false(source_level(qts, SRC_DMA2D_IN0));
    qtest_quit(qts);
}

/*
 * 2D-MOD0: an 8 x 2 block of RGB565 pixels into (8, 0) of a 24 x 8
 * picture, as ESP-IDF's DMA2D_M2M_2D_window test does, leaving the rest
 * of the picture untouched; then a 3 x 2 block of RGB888 out of (5, 3)
 * of a 10 x 6 picture into the top-left of another.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_m2m_2d_window(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;
    uint8_t s[32], d[24 * 8 * 2], p[10 * 6 * 3], q[4 * 4 * 3];

    for (int i = 0; i < 32; i++) {
        s[i] = 0x55 + i;
    }
    qtest_memwrite(qts, src, s, sizeof(s));
    memset_q(qts, dst, 0xff, sizeof(d));
    desc_2d(qts, tx, OWNER | EOF_BIT, 8, 2, 8, 2, 0, 0, PB_2, 0, src, 0);
    desc_2d(qts, rx, OWNER, 8, 2, 24, 8, 8, 0, PB_2, 0, dst, 0);
    m2m_setup(qts, 1, OUT_EOF_MODE | BURST(4), BURST(4), tx, rx);
    m2m_go(qts, 1);
    g_assert_true(wait_bits(qts, IN(1) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, dst, d, sizeof(d));
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 24; x++) {
            uint8_t *px = &d[(y * 24 + x) * 2];

            if (y < 2 && x >= 8 && x < 16) {
                g_assert_cmpint(px[0], ==, s[(y * 8 + x - 8) * 2]);
                g_assert_cmpint(px[1], ==, s[(y * 8 + x - 8) * 2 + 1]);
            } else {
                g_assert_cmpint(px[0], ==, 0xff);
                g_assert_cmpint(px[1], ==, 0xff);
            }
        }
    }
    /* The written-back 2D descriptor keeps its fields. */
    g_assert_cmphex(qtest_readl(qts, rx), ==,
                    OWNER | EOF_BIT | EN_2D | 8 << 14 | 2);

    for (int i = 0; i < (int)sizeof(p); i++) {
        p[i] = i;
    }
    qtest_memwrite(qts, src, p, sizeof(p));
    memset_q(qts, dst, 0, sizeof(q));
    desc_2d(qts, tx, OWNER | EOF_BIT, 3, 2, 10, 6, 5, 3, PB_3, 0, src, 0);
    desc_2d(qts, rx, OWNER, 3, 2, 4, 4, 0, 0, PB_3, 0, dst, 0);
    m2m_setup(qts, 2, OUT_EOF_MODE | BURST(0), BURST(1), tx, rx);
    m2m_go(qts, 2);
    g_assert_true(wait_bits(qts, IN(2) + INT_RAW, IN_SUC_EOF));
    g_assert_true(source_level(qts, SRC_DMA2D_IN2) == false);
    qtest_memread(qts, dst, q, sizeof(q));
    for (int y = 0; y < 2; y++) {
        for (int x = 0; x < 3; x++) {
            for (int c = 0; c < 3; c++) {
                g_assert_cmpint(q[(y * 4 + x) * 3 + c], ==,
                                p[((3 + y) * 10 + 5 + x) * 3 + c]);
            }
        }
        g_assert_cmpint(q[(y * 4 + 3) * 3], ==, 0);
    }
    qtest_quit(qts);
}

/* Color space conversion */

static void rgb565_to_888_ref(uint16_t v, uint8_t out[3])
{
    uint8_t c0 = v & 0x1f, c1 = (v >> 5) & 0x3f, c2 = v >> 11;

    out[0] = c0 << 3 | (c0 & 7);
    out[1] = c1 << 2 | (c1 & 3);
    out[2] = c2 << 3 | (c2 & 7);
}

/*
 * TX color space conversion in a memory-to-memory transfer: RGB565 to
 * RGB888, each component's low bits repeated, as ESP-IDF's
 * DMA2D_M2M_1D_RGB565_to_RGB888 expects; RGB888 to RGB565, the top bits;
 * and UYVY to BGR24 by the BT.601 table with (sum + 128) >> 8, as its
 * async color convert test checks.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_m2m_csc(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x3000;
    uint16_t px565[64];
    uint8_t got[64 * 3], want[3], rgb[48 * 3];
    static const uint8_t uyvy[12] = {
        128, 16, 128, 235, 90, 81, 240, 145, 240, 200, 16, 54,
    };
    static const int bt601[3][4] = {
        { 298, 0, 409, -56906 },
        { 298, -100, -208, 34707 },
        { 298, 516, 0, -70836 },
    };

    for (int i = 0; i < 64; i++) {
        px565[i] = 0x1234 * i + 0x0841 * (i & 7);
    }
    qtest_memwrite(qts, src, px565, sizeof(px565));
    desc_1d(qts, tx, OWNER | EOF_BIT, 128, 128, src, 0);
    desc_1d(qts, rx, OWNER, 0, 192, dst, 0);
    m2m_setup(qts, 0, OUT_EOF_MODE | BURST(2), BURST(2), tx, rx);
    /* Input RGB565, output directly. */
    qtest_writel(qts, OUT(0) + OUT_COLOR, 0 << 3 | 2);
    m2m_go(qts, 0);
    g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, dst, got, sizeof(got));
    for (int i = 0; i < 64; i++) {
        rgb565_to_888_ref(px565[i], want);
        g_assert_cmpmem(&got[i * 3], 3, want, 3);
    }
    g_assert_cmphex(qtest_readl(qts, rx) & 0xfffc000, ==, 192 << 14);

    /* Back: RGB888 to RGB565 truncates. */
    for (int i = 0; i < 48 * 3; i++) {
        rgb[i] = i * 37 + 11;
    }
    qtest_memwrite(qts, src, rgb, sizeof(rgb));
    desc_1d(qts, tx, OWNER | EOF_BIT, sizeof(rgb), sizeof(rgb), src, 0);
    desc_1d(qts, rx, OWNER, 0, 96, dst, 0);
    m2m_setup(qts, 0, OUT_EOF_MODE | BURST(3), BURST(3), tx, rx);
    qtest_writel(qts, OUT(0) + OUT_COLOR, 3 << 3 | 0);
    m2m_go(qts, 0);
    g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, dst, got, 96);
    for (int i = 0; i < 48; i++) {
        uint16_t v = (rgb[i * 3 + 2] >> 3) << 11 | (rgb[i * 3 + 1] >> 2) << 5 |
                     rgb[i * 3] >> 3;

        g_assert_cmphex(got[2 * i] | got[2 * i + 1] << 8, ==, v);
    }

    /* UYVY to BGR24, BT.601, as a 2D block of 6 x 1 pixels. */
    qtest_memwrite(qts, src, uyvy, sizeof(uyvy));
    desc_2d(qts, tx, OWNER | EOF_BIT, 6, 1, 6, 1, 0, 0, PB_2, 0, src, 0);
    desc_2d(qts, rx, OWNER, 6, 1, 6, 1, 0, 0, PB_3, 0, dst, 0);
    m2m_setup(qts, 1, OUT_EOF_MODE | BURST(1), BURST(1), tx, rx);
    qtest_writel(qts, OUT(1) + OUT_COLOR, 1 << 3 | 1 << 2 | 2);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, OUT(1) + OUT_PARAM(2 * i),
                     (bt601[i][0] & 0x3ff) | (bt601[i][1] & 0x7ff) << 10);
        qtest_writel(qts, OUT(1) + OUT_PARAM(2 * i + 1),
                     (bt601[i][2] & 0x3ff) | (bt601[i][3] & 0x3ffff) << 10);
    }
    m2m_go(qts, 1);
    g_assert_true(wait_bits(qts, IN(1) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, dst, got, 18);
    for (int i = 0; i < 6; i++) {
        int pair = i / 2, y = uyvy[pair * 4 + 1 + 2 * (i & 1)];
        int u = uyvy[pair * 4], v = uyvy[pair * 4 + 2];

        for (int c = 0; c < 3; c++) {
            int q = (bt601[2 - c][0] * y + bt601[2 - c][1] * u +
                     bt601[2 - c][2] * v + bt601[2 - c][3] + 128) >> 8;

            g_assert_cmpint(got[i * 3 + c], ==, MIN(MAX(q, 0), 255));
        }
    }
    qtest_quit(qts);
}

/* The JPEG codec's paths, played through the "periph" hook */

static char *periph(QTestState *qts, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

static char *periph(QTestState *qts, const char *fmt, ...)
{
    va_list ap;
    g_autofree char *cmd = NULL;
    QDict *r;
    char *out;

    va_start(ap, fmt);
    cmd = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': { 'path': "
                  "'/machine/dma2d', 'property': 'periph', 'value': %s } }",
                  cmd);
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': { 'path': "
                  "'/machine/dma2d', 'property': 'periph' } }");
    g_assert(qdict_haskey(r, "return"));
    out = g_strdup(qdict_get_str(r, "return"));
    qobject_unref(r);
    return out;
}

/*
 * Pull up to @len bytes from the TX channel serving peripheral @sel into
 * @buf, stepping time while it reads: how many came, *eof set at a frame's
 * end.
 */
static uint32_t pull_all(QTestState *qts, int sel, uint8_t *buf,
                         uint32_t len, bool *eof)
{
    uint32_t got = 0;

    *eof = false;
    for (int i = 0; i < 20000 && got < len && !*eof; i++) {
        g_autofree char *r = periph(qts, "pull %d %u", sel, len - got);
        unsigned n, e;
        char *hex;

        g_assert_cmpint(sscanf(r, "%u %u", &n, &e), ==, 2);
        hex = strrchr(r, ' ') + 1;
        for (unsigned k = 0; k < n; k++) {
            unsigned v;

            sscanf(hex + 2 * k, "%2x", &v);
            buf[got + k] = v;
        }
        got += n;
        *eof = e;
        if (!n) {
            qtest_clock_step(qts, 200);
        }
    }
    return got;
}

/* Push @len bytes to the RX channel serving @sel, the last with @flags. */
static void push_all(QTestState *qts, int sel, const uint8_t *buf,
                     uint32_t len, unsigned flags)
{
    uint32_t done = 0;

    for (int i = 0; i < 20000 && done < len; i++) {
        uint32_t n = MIN(len - done, 400);
        GString *hex = g_string_new(NULL);
        g_autofree char *r = NULL;
        unsigned took;

        for (uint32_t k = 0; k < n; k++) {
            g_string_append_printf(hex, "%02x", buf[done + k]);
        }
        r = periph(qts, "push %d %s %u", sel, hex->str,
                   done + n == len ? flags : 0);
        g_string_free(hex, true);
        took = atoi(r);
        done += took;
        if (took < n) {
            qtest_clock_step(qts, 200);
        }
    }
    g_assert_cmpuint(done, ==, len);
}

/*
 * The JPEG encoder's input: TX channel 0 in 2D-MOD1 with 8 x 8
 * macroblocks reordered, hb = 16 and vb = 8, over a 20 x 10 RGB888
 * picture. The blocks go left to right and down; each macroblock comes
 * whole, row by row; the right column of blocks (4 pixels) is padded to
 * 8 with the last pixel, the bottom row (2 rows) to 8 with the last row;
 * the frame ends after the last block.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_jpeg_tx_mod1(void)
{
    QTestState *qts = start();
    const int w = 20, h = 10;
    uint8_t pic[20 * 10 * 3], got[4096], want[4096];
    uint32_t n = 0, have;
    bool eof;

    for (int i = 0; i < w * h * 3; i++) {
        pic[i] = i * 13 + (i >> 5);
    }
    qtest_memwrite(qts, MEM + 0x1000, pic, sizeof(pic));
    desc_2d(qts, MEM, OWNER | EOF_BIT, 16, 8, w, h, 0, 0, PB_3, MOD1,
            MEM + 0x1000, 0);
    qtest_writel(qts, OUT(0) + CONF0, CONF0_RST);
    qtest_writel(qts, OUT(0) + CONF0, 0);
    qtest_writel(qts, OUT(0) + OUT_PERI_SEL, 0);
    qtest_writel(qts, OUT(0) + CONF0, OUT_EOF_MODE | BURST(4) | MB_SIZE(0) |
                 REORDER_EN);
    qtest_writel(qts, OUT(0) + LINK_ADDR, MEM);
    qtest_writel(qts, OUT(0) + LINK_CONF, OUT_LINK_START);

    for (int by = 0; by < h; by += 8) {
        for (int bx = 0; bx < w; bx += 16) {
            int pw = MIN(16, w - bx) <= 8 ? 8 : 16;

            for (int mb = 0; mb < pw / 8; mb++) {
                for (int r = 0; r < 8; r++) {
                    for (int c = 0; c < 8; c++) {
                        int x = MIN(bx + mb * 8 + c, w - 1);
                        int y = MIN(by + r, h - 1);

                        memcpy(&want[n], &pic[(y * w + x) * 3], 3);
                        n += 3;
                    }
                }
            }
        }
    }
    have = pull_all(qts, 0, got, sizeof(got), &eof);
    g_assert_true(eof);
    g_assert_cmpuint(have, ==, n);
    g_assert_cmpmem(got, have, want, n);
    g_assert_true(wait_bits(qts, OUT(0) + INT_RAW,
                            OUT_DONE | OUT_EOF | OUT_TOTAL_EOF));
    /* The reorder status: 3 bytes a pixel. */
    qtest_quit(qts);
}

/*
 * The JPEG decoder's output: RX channel 0 in 2D-MOD1 with 16 x 16
 * macroblocks reordered takes YUV420 MCUs (Y0..Y3, U, V) and writes
 * YUV444 (IN_COLOR_INPUT_SEL 0, out directly: V U Y), then the same MCUs
 * converted to RGB888 by the BT.601 table, (sum + 128) >> 8.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_jpeg_rx_mcu(void)
{
    QTestState *qts = start();
    static const int bt601[3][4] = {
        { 298, 0, 409, -56906 },
        { 298, -100, -208, 34707 },
        { 298, 516, 0, -70836 },
    };
    uint8_t mcu[2 * 384], got[32 * 16 * 3];

    for (int i = 0; i < (int)sizeof(mcu); i++) {
        mcu[i] = 16 + (i * 7) % 220;
    }
    for (int pass = 0; pass < 2; pass++) {
        memset_q(qts, MEM + 0x2000, 0, sizeof(got));
        desc_2d(qts, MEM, OWNER, 32, 16, 32, 16, 0, 0, PB_3, MOD1,
                MEM + 0x2000, 0);
        qtest_writel(qts, IN(0) + CONF0, CONF0_RST);
        qtest_writel(qts, IN(0) + CONF0, 0);
        qtest_writel(qts, IN(0) + INT_CLR, 0x3fff);
        qtest_writel(qts, IN(0) + IN_PERI_SEL, 0);
        qtest_writel(qts, IN(0) + CONF0, BURST(4) | MB_SIZE(2) | REORDER_EN);
        if (pass == 0) {
            qtest_writel(qts, IN(0) + IN_COLOR, 0 << 3 | 1);
        } else {
            qtest_writel(qts, IN(0) + IN_COLOR, 0 << 3 | 1 << 2 | 1);
        }
        qtest_writel(qts, IN(0) + LINK_ADDR, MEM);
        qtest_writel(qts, IN(0) + LINK_CONF, IN_LINK_START | IN_AUTO_RET);
        g_free(periph(qts, "mcu 0 3"));
        push_all(qts, 0, mcu, sizeof(mcu), 1);
        g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
        qtest_memread(qts, MEM + 0x2000, got, sizeof(got));
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 32; x++) {
                const uint8_t *m = &mcu[(x / 16) * 384];
                int lx = x % 16;
                int yy = m[((y / 8) * 2 + lx / 8) * 64 + (y % 8) * 8 + lx % 8];
                int u = m[256 + (y / 2) * 8 + lx / 2];
                int v = m[320 + (y / 2) * 8 + lx / 2];
                uint8_t *px = &got[(y * 32 + x) * 3];

                if (pass == 0) {
                    g_assert_cmpint(px[0], ==, v);
                    g_assert_cmpint(px[1], ==, u);
                    g_assert_cmpint(px[2], ==, yy);
                    continue;
                }
                for (int c = 0; c < 3; c++) {
                    int q = (bt601[2 - c][0] * yy + bt601[2 - c][1] * u +
                             bt601[2 - c][2] * v + bt601[2 - c][3] + 128) >> 8;

                    g_assert_cmpint(px[c], ==, MIN(MAX(q, 0), 255));
                }
            }
        }
    }
    qtest_quit(qts);
}

/*
 * DSCR-PORT, as the PPA's SRM uses it: the TX channel gives 4 x 3 pixel
 * blocks of the 5 x 4 image block at (2, 1) of a 10 x 8 picture where the
 * peripheral asks, padded with the image block's edge on every side; the
 * RX channel writes the blocks pushed where the peripheral says and writes
 * back their extent.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_dscr_port(void)
{
    QTestState *qts = start();
    const int w = 10, h = 8, bx = 2, by = 1, bw = 5, bh = 4;
    uint8_t pic[80], got[24], want[24], blk[12];
    g_autofree char *r = NULL;
    bool eof;
    uint32_t n;

    for (int i = 0; i < w * h; i++) {
        pic[i] = i + 1;
    }
    qtest_memwrite(qts, MEM + 0x1000, pic, sizeof(pic));
    desc_2d(qts, MEM, OWNER | EOF_BIT, bw, bh, w, h, bx, by, PB_1, 0,
            MEM + 0x1000, 0);
    qtest_writel(qts, OUT(1) + CONF0, CONF0_RST);
    qtest_writel(qts, OUT(1) + CONF0, 0);
    qtest_writel(qts, OUT(1) + OUT_PERI_SEL, 0);
    qtest_writel(qts, OUT(1) + CONF0, OUT_EOF_MODE | BURST(4) | PORT_EN);
    qtest_writel(qts, OUT(1) + OUT_PORT_BLK, 4 | 3 << 14);
    qtest_writel(qts, OUT(1) + LINK_ADDR, MEM);
    qtest_writel(qts, OUT(1) + LINK_CONF, OUT_LINK_START);
    qtest_clock_step(qts, 10000);
    r = periph(qts, "desc 1 0");
    g_assert_cmpstr(r, ==, "1 5 4 10 8 2 1 1 0 1 0x4ff41000");
    g_free(r);
    r = periph(qts, "port-tx 0 -1 -1 0");
    g_assert_cmpstr(r, ==, "1");
    g_free(r);
    r = periph(qts, "port-tx 0 3 2 1");
    g_assert_cmpstr(r, ==, "1");
    g_free(r);
    /* The queue holds two. */
    r = periph(qts, "port-tx 0 0 0 0");
    g_assert_cmpstr(r, ==, "0");
    n = 0;
    for (int b = 0; b < 2; b++) {
        int x0 = b ? 3 : -1, y0 = b ? 2 : -1;

        for (int yy = 0; yy < 3; yy++) {
            for (int xx = 0; xx < 4; xx++) {
                int x = MIN(MAX(x0 + xx, 0), bw - 1);
                int y = MIN(MAX(y0 + yy, 0), bh - 1);

                want[n++] = pic[(by + y) * w + bx + x];
            }
        }
    }
    g_assert_cmpuint(pull_all(qts, 0, got, sizeof(got), &eof), ==, 24);
    g_assert_true(eof);
    g_assert_cmpmem(got, 24, want, 24);
    g_assert_true(wait_bits(qts, OUT(1) + INT_RAW, OUT_EOF | OUT_TOTAL_EOF));

    /* The RX side: two blocks into a 10 x 8 picture at (1, 1). */
    memset_q(qts, MEM + 0x2000, 0, 80);
    desc_2d(qts, MEM + 0x40, OWNER, 2, 2, w, h, 1, 1, PB_1, 0, MEM + 0x2000,
            0);
    qtest_writel(qts, IN(1) + CONF0, CONF0_RST);
    qtest_writel(qts, IN(1) + CONF0, 0);
    qtest_writel(qts, IN(1) + IN_PERI_SEL, 0);
    qtest_writel(qts, IN(1) + CONF0, BURST(4) | PORT_EN);
    qtest_writel(qts, IN(1) + LINK_ADDR, MEM + 0x40);
    qtest_writel(qts, IN(1) + LINK_CONF, IN_LINK_START | IN_AUTO_RET);
    g_free(periph(qts, "port-rx 0 0 0 3 2 2 0"));
    g_free(periph(qts, "port-rx 0 3 1 2 2 2 1"));
    for (int i = 0; i < 10; i++) {
        blk[i] = 0xa0 + i;
    }
    push_all(qts, 0, blk, 6, 0);
    push_all(qts, 0, blk + 6, 4, 1);
    g_assert_true(wait_bits(qts, IN(1) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, MEM + 0x2000, got, 24);
    {
        uint8_t full[80];

        qtest_memread(qts, MEM + 0x2000, full, 80);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                uint8_t v = 0;

                if (y >= 1 && y < 3 && x >= 1 && x < 4) {
                    v = blk[(y - 1) * 3 + x - 1];
                } else if (y >= 2 && y < 4 && x >= 4 && x < 6) {
                    v = blk[6 + (y - 2) * 2 + x - 4];
                }
                g_assert_cmpint(full[y * w + x], ==, v);
            }
        }
    }
    /* Written back: the blocks reached 5 x 3 of the image block. */
    g_assert_cmphex(qtest_readl(qts, MEM + 0x40), ==,
                    OWNER | EOF_BIT | EN_2D | 5 << 14 | 3);
    qtest_quit(qts);
}

/* Descriptors */

/*
 * Descriptor checks (TRM 8.4.9): misaligned, outside the accessible
 * memory, owned by the CPU with CHECK_OWNER, HA 0 in 2D mode, size 0 in
 * 1D mode: DSCR_ERR, and the channel idle; a restart from LINK_START.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_dscr_errors(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, src = MEM + 0x1000;

    fill(qts, src, 64, 1);
    /* Misaligned. */
    desc_1d(qts, tx + 4, OWNER | EOF_BIT, 64, 64, src, 0);
    qtest_writel(qts, OUT(2) + OUT_PERI_SEL, 0);
    qtest_writel(qts, OUT(2) + LINK_ADDR, tx + 4);
    qtest_writel(qts, OUT(2) + LINK_CONF, OUT_LINK_START);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + INT_RAW), ==, OUT_DSCR_ERR);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + LINK_CONF) & OUT_LINK_PARK, ==,
                    OUT_LINK_PARK);
    qtest_writel(qts, OUT(2) + INT_CLR, OUT_DSCR_ERR);

    /* Outside the accessible memory. */
    qtest_writel(qts, INTR_END, 0x4ff3ffff);
    qtest_writel(qts, EXTR_END, 0x4ff3ffff);
    desc_1d(qts, tx, OWNER | EOF_BIT, 64, 64, src, 0);
    qtest_writel(qts, OUT(2) + LINK_ADDR, tx);
    qtest_writel(qts, OUT(2) + LINK_CONF, OUT_LINK_START);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + INT_RAW), ==, OUT_DSCR_ERR);
    qtest_writel(qts, OUT(2) + INT_CLR, OUT_DSCR_ERR);
    qtest_writel(qts, INTR_END, 0x8fffffff);
    qtest_writel(qts, EXTR_END, 0x8fffffff);

    /* Owned by the CPU, with the owner checked. */
    desc_1d(qts, tx, EOF_BIT, 64, 64, src, 0);
    qtest_writel(qts, OUT(2) + CONF0, OUT_EOF_MODE | CHECK_OWNER);
    qtest_writel(qts, OUT(2) + LINK_CONF, OUT_LINK_START);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + INT_RAW), ==, OUT_DSCR_ERR);
    qtest_writel(qts, OUT(2) + INT_CLR, OUT_DSCR_ERR);

    /* 2D with HA 0; 1D with size 0. */
    desc_2d(qts, tx, OWNER | EOF_BIT, 4, 4, 0, 4, 0, 0, PB_1, 0, src, 0);
    qtest_writel(qts, OUT(2) + LINK_CONF, OUT_LINK_START);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + INT_RAW), ==, OUT_DSCR_ERR);
    qtest_writel(qts, OUT(2) + INT_CLR, OUT_DSCR_ERR);
    desc_1d(qts, MEM + 0x100, OWNER, 0, 0, MEM + 0x2000, 0);
    qtest_writel(qts, IN(2) + IN_PERI_SEL, 0);
    qtest_writel(qts, IN(2) + LINK_ADDR, MEM + 0x100);
    qtest_writel(qts, IN(2) + LINK_CONF, IN_LINK_START);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, IN(2) + INT_RAW), ==, IN_DSCR_ERR);
    g_assert_cmphex(qtest_readl(qts, IN(2) + IN_DSCR), ==, 0);

    /* A good descriptor after an error runs. */
    desc_1d(qts, tx, OWNER | EOF_BIT, 64, 64, src, 0);
    qtest_writel(qts, OUT(2) + LINK_CONF, OUT_LINK_START);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, OUT(2) + INT_RAW) & OUT_DSCR_ERR, ==, 0);
    qtest_quit(qts);
}

/*
 * STOP, appending a descriptor and RESTART (TRM 8.4.8): the channel ends
 * its list, takes the new DW4 on RESTART and goes on with it; the
 * DSCR_EMPTY of data beyond the RX list's end.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_restart(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;
    uint8_t a[512], b[512];

    fill(qts, src, 512, 9);
    memset_q(qts, dst, 0, 512);
    desc_1d(qts, tx, OWNER, 256, 256, src, 0);
    desc_1d(qts, rx, OWNER, 0, 512, dst, 0);
    m2m_setup(qts, 0, OUT_EOF_MODE | BURST(4), BURST(4), tx, rx);
    m2m_go(qts, 0);
    g_assert_true(wait_bits(qts, OUT(0) + INT_RAW, OUT_DONE));
    qtest_clock_step(qts, 5000);
    /* The TX list ended; RX holds its descriptor, waiting for more. */
    g_assert_cmphex(qtest_readl(qts, OUT(0) + LINK_CONF) & OUT_LINK_PARK, ==,
                    OUT_LINK_PARK);
    g_assert_cmphex(qtest_readl(qts, IN(0) + INT_RAW) & IN_DONE, ==, 0);
    /* Append and restart. */
    desc_1d(qts, tx + 0x20, OWNER | EOF_BIT, 256, 256, src + 256, 0);
    qtest_writel(qts, tx + 16, tx + 0x20);
    qtest_writel(qts, OUT(0) + LINK_CONF, OUT_LINK_RESTART);
    g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, src, a, 512);
    qtest_memread(qts, dst, b, 512);
    g_assert_cmpmem(a, 512, b, 512);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + OUT_EOF_DES), ==, tx + 0x20);

    /* More data than the RX list holds: DSCR_EMPTY. */
    desc_1d(qts, tx, OWNER | EOF_BIT, 512, 512, src, 0);
    desc_1d(qts, rx, OWNER, 0, 128, dst, 0);
    m2m_setup(qts, 1, OUT_EOF_MODE | BURST(4), BURST(4), tx, rx);
    m2m_go(qts, 1);
    g_assert_true(wait_bits(qts, IN(1) + INT_RAW, IN_DSCR_EMPTY | IN_DONE));
    g_assert_cmphex(qtest_readl(qts, IN(1) + INT_RAW) & IN_SUC_EOF, ==, 0);
    qtest_quit(qts);
}

/* ETM */

/*
 * ETM tasks: with ETM_EN, LINK_START does nothing and the IN_START and
 * OUT_START tasks start the channels; with ETM_LOOP_EN, each descriptor
 * after the first waits for a DSCR_READY task, one coming early is kept,
 * and one more than DSCR_TASK_MAK + 1 overflows.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_etm(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;

    fill(qts, src, 192, 5);
    memset_q(qts, dst, 0, 192);
    desc_1d(qts, tx, OWNER, 64, 64, src, tx + 0x20);
    desc_1d(qts, tx + 0x20, OWNER, 64, 64, src + 64, tx + 0x40);
    desc_1d(qts, tx + 0x40, OWNER | EOF_BIT, 64, 64, src + 128, 0);
    desc_1d(qts, rx, OWNER, 0, 192, dst, 0);
    m2m_setup(qts, 0, OUT_EOF_MODE | BURST(4), BURST(4), tx, rx);
    qtest_writel(qts, OUT(0) + OUT_ETM_CONF, 1 | 2 | 0 << 2);
    qtest_writel(qts, IN(0) + IN_ETM_CONF, 1);
    m2m_go(qts, 0);
    qtest_clock_step(qts, 5000);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + LINK_CONF) & OUT_LINK_PARK, ==,
                    OUT_LINK_PARK);
    /* IN_START_CH0 (line 0), OUT_START_CH0 (line 4). */
    qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 0, 1);
    qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 0, 0);
    qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 4, 1);
    qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 4, 0);
    qtest_clock_step(qts, 5000);
    /* The first descriptor went; the second waits for DSCR_READY. */
    g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_RAW) & OUT_DONE, ==,
                    OUT_DONE);
    qtest_writel(qts, OUT(0) + INT_CLR, OUT_DONE);
    qtest_clock_step(qts, 5000);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_RAW) & OUT_DONE, ==, 0);
    /* OUT_DSCR_READY_CH0 (line 7): the second goes. */
    qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 7, 1);
    qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 7, 0);
    qtest_clock_step(qts, 5000);
    g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_RAW) & OUT_DONE, ==,
                    OUT_DONE);
    /*
     * Three tasks at once: the first lets the third descriptor go, the
     * second is kept (DSCR_TASK_MAK 0 keeps one), the third overflows.
     */
    for (int i = 0; i < 3; i++) {
        g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_RAW) & OUT_TASK_OVF,
                        ==, 0);
        qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 7, 1);
        qtest_set_irq_in(qts, "/machine/dma2d", "etm-task", 7, 0);
    }
    g_assert_cmphex(qtest_readl(qts, OUT(0) + INT_RAW) & OUT_TASK_OVF, ==,
                    OUT_TASK_OVF);
    g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
    qtest_quit(qts);
}

/* Arbitration and timing */

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
 * The virtual time from @t0 until @bits are set in @reg, stepping @step
 * ns: to within a step.
 */
static int64_t time_to(QTestState *qts, int64_t t0, uint32_t reg,
                       uint32_t bits, int64_t step)
{
    int64_t t = t0;

    for (int i = 0; i < 1000000; i++) {
        if ((qtest_readl(qts, reg) & bits) == bits) {
            return t - t0;
        }
        t = qtest_clock_step(qts, step);
    }
    g_assert_not_reached();
}

/*
 * A transfer's duration: 4 KiB memory to memory in 128-byte bursts takes
 * a bus cycle per 8 bytes each way, read and write side by side, the
 * descriptors' reads and write-backs besides: 512 cycles and a few, at
 * DMA2D_SYS_CLK.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_timing(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x3000;
    uint64_t hz = clk_hz(qts, "/machine/dma2d/clk");
    int64_t t, cycle;

    g_assert_cmpuint(hz, >, 0);
    cycle = 1000000000 / hz;
    fill(qts, src, 4096, 1);
    desc_1d(qts, tx, OWNER | EOF_BIT, 4096, 4096, src, 0);
    desc_1d(qts, rx, OWNER, 0, 4096, dst, 0);
    m2m_setup(qts, 0, OUT_EOF_MODE | BURST(4), BURST(4), tx, rx);
    t = qtest_clock_step(qts, 1);
    m2m_go(qts, 0);
    t = time_to(qts, t, IN(0) + INT_RAW, IN_SUC_EOF, cycle);
    g_test_message("4 KiB: %" PRId64 " ns at %" PRIu64 " Hz", t, hz);
    g_assert_cmpint(t, >=, 512 * cycle);
    g_assert_cmpint(t, <=, 540 * cycle);
    /* Bursts of 8 bytes: the same cycles, as each beat is one. */
    qtest_quit(qts);
}

/*
 * Priority (TRM 8.4.12): two memory-to-memory transfers at once on
 * channels 0 and 1, channel 1 of higher priority: it is served first and
 * finishes first though it started second; weighted arbitration with
 * channel 0 holding the more tokens turns that round.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_priority(void)
{
    QTestState *qts = start();

    for (int pass = 0; pass < 2; pass++) {
        uint32_t raw0, raw1;

        fill(qts, MEM + 0x1000, 8192, 3);
        desc_1d(qts, MEM, OWNER | EOF_BIT, 8192, 8192, MEM + 0x1000, 0);
        desc_1d(qts, MEM + 0x40, OWNER | EOF_BIT, 8192, 8192, MEM + 0x1000,
                0);
        desc_1d(qts, MEM + 0x80, OWNER, 0, 8192, MEM + 0x4000, 0);
        desc_1d(qts, MEM + 0xc0, OWNER, 0, 8192, MEM + 0x6000, 0);
        m2m_setup(qts, 0, OUT_EOF_MODE | BURST(4), BURST(4), MEM, MEM + 0x80);
        m2m_setup(qts, 1, OUT_EOF_MODE | BURST(4), BURST(4), MEM + 0x40,
                  MEM + 0xc0);
        if (pass == 0) {
            qtest_writel(qts, OUT(1) + OUT_ARB, 0x51);
            qtest_writel(qts, IN(1) + IN_ARB, 0x51);
        } else {
            /* Equal priorities, channel 0 with 8 tokens a slot to 1's 1. */
            qtest_writel(qts, OUT(1) + OUT_ARB, 0x11);
            qtest_writel(qts, IN(1) + IN_ARB, 0x11);
            qtest_writel(qts, OUT(0) + OUT_ARB, 0x18);
            qtest_writel(qts, IN(0) + IN_ARB, 0x18);
            qtest_writel(qts, OUT_ARB_CONFIG, 1 << 16 | 1000);
            qtest_writel(qts, IN_ARB_CONFIG, 1 << 16 | 1000);
        }
        m2m_go(qts, 0);
        m2m_go(qts, 1);
        for (int i = 0; i < 100000; i++) {
            raw0 = qtest_readl(qts, IN(0) + INT_RAW) & IN_SUC_EOF;
            raw1 = qtest_readl(qts, IN(1) + INT_RAW) & IN_SUC_EOF;
            if (raw0 || raw1) {
                break;
            }
            qtest_clock_step(qts, 100);
        }
        if (pass == 0) {
            g_assert_true(raw1 && !raw0);
        } else {
            g_assert_true(raw0 && !raw1);
        }
        g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
        g_assert_true(wait_bits(qts, IN(1) + INT_RAW, IN_SUC_EOF));
    }
    qtest_quit(qts);
}

/* Push and pop, reset, permissions */

/*
 * OUT_PUSH puts a byte into a TX FIFO, the peripheral takes it; IN_POP
 * takes from an RX FIFO what the peripheral pushed, 0x400 once empty with
 * INFIFO_UDF; a channel held in reset is idle and takes nothing; the
 * controller's reset (HP_SYS_CLKRST_RST_EN_DMA2D) restores the registers.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_pushpop_reset(void)
{
    QTestState *qts = start();
    g_autofree char *r = NULL;

    qtest_writel(qts, OUT(3) + OUT_PERI_SEL, 0);
    qtest_writel(qts, OUT(3) + PUSHPOP, 0x5a | 1 << 10);
    g_assert_cmphex(qtest_readl(qts, OUT(3) + FIFO_STATUS) >> 24 & 0x1f, ==,
                    1);
    r = periph(qts, "pull 0 4");
    g_assert_cmpstr(r, ==, "1 0 5a");
    g_free(r);

    qtest_writel(qts, IN(2) + IN_PERI_SEL, 0);
    desc_1d(qts, MEM, OWNER, 0, 64, MEM + 0x1000, 0);
    qtest_writel(qts, IN(2) + LINK_ADDR, MEM);
    /* The RX DMA stopped by CMD_DISABLE, the bytes stay to pop. */
    qtest_writel(qts, IN(2) + CONF0, 0);
    qtest_writel(qts, IN(2) + LINK_CONF, IN_LINK_START);
    r = periph(qts, "push 0 c3d4 0");
    g_assert_cmpstr(r, ==, "2");
    g_free(r);
    qtest_writel(qts, IN(2) + CONF0, 1u << 25);
    qtest_writel(qts, IN(2) + PUSHPOP, 1 << 11);
    g_assert_cmphex(qtest_readl(qts, IN(2) + PUSHPOP) & 0x7ff, ==, 0xc3);
    qtest_writel(qts, IN(2) + PUSHPOP, 1 << 11);
    g_assert_cmphex(qtest_readl(qts, IN(2) + PUSHPOP) & 0x7ff, ==, 0xd4);
    qtest_writel(qts, IN(2) + PUSHPOP, 1 << 11);
    g_assert_cmphex(qtest_readl(qts, IN(2) + PUSHPOP) & 0x7ff, ==, 0x400);
    g_assert_cmphex(qtest_readl(qts, IN(2) + INT_RAW) & (1u << 5), ==,
                    1u << 5);
    /* Held in reset: no pushes taken. */
    qtest_writel(qts, IN(2) + CONF0, CONF0_RST);
    r = periph(qts, "push 0 c3 0");
    g_assert_cmpstr(r, ==, "0");
    qtest_writel(qts, IN(2) + CONF0, 0);

    qtest_writel(qts, OUT(1) + OUT_ARB, 0x77);
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) | RST_EN_DMA2D);
    qtest_writel(qts, HP_RST_EN0,
                 qtest_readl(qts, HP_RST_EN0) & ~RST_EN_DMA2D);
    g_assert_cmphex(qtest_readl(qts, OUT(1) + OUT_ARB), ==, 0x11);
    g_assert_cmphex(qtest_readl(qts, IN(2) + IN_PERI_SEL), ==, 7);
    qtest_quit(qts);
}

/*
 * The DMA access permission manager refuses the 2D-DMA's DUMMY master
 * writes: the data and the descriptor's write-back are lost, the AXI write
 * responses counted in AXI_ERR, and the AXI ICM records the first.
 */
/* [spec:nuos:req:emu.esp32p4.ppa/test] */
static void test_pms(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;
    uint8_t got[64];

    fill(qts, src, 64, 7);
    memset_q(qts, dst, 0, 64);
    desc_1d(qts, tx, OWNER | EOF_BIT, 64, 64, src, 0);
    desc_1d(qts, rx, OWNER, 0, 64, dst, 0);
    qtest_writel(qts, DMA_DMA2D_DUMMY_W, 0);
    m2m_setup(qts, 0, OUT_EOF_MODE | BURST(4), BURST(4), tx, rx);
    m2m_go(qts, 0);
    g_assert_true(wait_bits(qts, IN(0) + INT_RAW, IN_SUC_EOF));
    qtest_memread(qts, dst, got, 64);
    for (int i = 0; i < 64; i++) {
        g_assert_cmpint(got[i], ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, rx), ==, OWNER | 64);
    g_assert_cmpuint(qtest_readl(qts, AXI_ERR) >> 8 & 0xf, ==, 2);
    g_assert_cmphex(qtest_readl(qts, 0x500a4000 + 0x0c) & 2, ==, 2);
    /* The ICM holds the first refused access: the data's. */
    g_assert_cmphex(qtest_readl(qts, 0x500a4000 + 0x38), ==, dst);
    g_assert_cmphex(qtest_readl(qts, 0x500a4000 + 0x3c), ==,
                    0x100 | 0xa0 | 3);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32p4/dma2d/reset-values", test_reset_values);
    qtest_add_func("esp32p4/dma2d/m2m-1d", test_m2m_1d);
    qtest_add_func("esp32p4/dma2d/m2m-2d-window", test_m2m_2d_window);
    qtest_add_func("esp32p4/dma2d/m2m-csc", test_m2m_csc);
    qtest_add_func("esp32p4/dma2d/jpeg-tx-mod1", test_jpeg_tx_mod1);
    qtest_add_func("esp32p4/dma2d/jpeg-rx-mcu", test_jpeg_rx_mcu);
    qtest_add_func("esp32p4/dma2d/dscr-port", test_dscr_port);
    qtest_add_func("esp32p4/dma2d/dscr-errors", test_dscr_errors);
    qtest_add_func("esp32p4/dma2d/restart", test_restart);
    qtest_add_func("esp32p4/dma2d/etm", test_etm);
    qtest_add_func("esp32p4/dma2d/timing", test_timing);
    qtest_add_func("esp32p4/dma2d/priority", test_priority);
    qtest_add_func("esp32p4/dma2d/pushpop-reset", test_pushpop_reset);
    qtest_add_func("esp32p4/dma2d/pms", test_pms);
    return g_test_run();
}
