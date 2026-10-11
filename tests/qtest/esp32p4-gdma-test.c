/*
 * QTest testcase for the ESP32-P4 GDMA-AHB and GDMA-AXI controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The tests build descriptor chains in L2MEM, past the 128 KiB the L2
 * cache holds from reset, run memory-to-memory transfers and step virtual
 * time while the channels move the data at the controllers' bus clock.
 * Interrupts are seen in the interrupt matrix's status registers, which
 * follow every source's level.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define AHB             0x50085000
#define AXI             0x5008a000
#define MEM             0x4ff40000

/* GDMA-AHB. */
#define AHB_IN_INT(ch)      (AHB + 0x10 * (ch))
#define AHB_OUT_INT(ch)     (AHB + 0x30 + 0x10 * (ch))
#define AHB_MISC_CONF       (AHB + 0x64)
#define AHB_DATE            (AHB + 0x68)
#define AHB_IN(ch)          (AHB + 0x70 + 0xc0 * (ch))
#define AHB_OUT(ch)         (AHB + 0xd0 + 0xc0 * (ch))
#define AHB_OUT_CRC(ch)     (AHB + 0x2bc + 0x28 * (ch))
#define AHB_IN_CRC(ch)      (AHB + 0x334 + 0x28 * (ch))
#define AHB_IN_LINK_ADDR(ch)  (AHB + 0x3ac + 4 * (ch))
#define AHB_OUT_LINK_ADDR(ch) (AHB + 0x3b8 + 4 * (ch))
#define AHB_INTR_MEM_START  (AHB + 0x3c4)
#define AHB_INTR_MEM_END    (AHB + 0x3c8)
#define AHB_ARB_TIMEOUT     (AHB + 0x3dc)
#define AHB_WEIGHT_EN       (AHB + 0x400)
#define AHB_MODULE_CLK_EN   (AHB + 0x404)
#define AHB_RESP_ERR0       (AHB + 0x408)
#define AHB_RESP_ERR1       (AHB + 0x40c)
#define AHB_IN_DONE_DES(ch) (AHB + 0x410 + 8 * (ch))
#define AHB_OUT_DONE_DES(ch) (AHB + 0x414 + 8 * (ch))

/* Within an AHB channel's block. */
#define CONF0           0x00
#define CONF1           0x04
#define FIFO_STATUS     0x08
#define PUSHPOP         0x0c
#define LINK            0x10
#define STATE           0x14
#define EOF_DES         0x18
#define EOF_BFR_DES     0x1c
#define DSCR            0x20
#define DSCR_BF0        0x24
#define DSCR_BF1        0x28
#define PRI             0x2c
#define PERI_SEL        0x30

/* Within an interrupt block. */
#define RAW             0x0
#define ST              0x4
#define ENA             0x8
#define CLR             0xc

/* Within a CRC block. */
#define CRC_INIT        0x00
#define CRC_WIDTH       0x04
#define CRC_CLEAR       0x08
#define CRC_FINAL       0x0c
#define CRC_EN_WR       0x10
#define CRC_EN_ADDR     0x14
#define CRC_DATA_EN_WR  0x18
#define CRC_DATA_EN_ADDR 0x1c
#define ARB_WEIGHT      0x20
#define ARB_OPT_DIS     0x24

/* GDMA-AXI: every channel register in one block. */
#define AXI_IN(ch)          (AXI + 0x68 * (ch))
#define AXI_OUT(ch)         (AXI + 0x138 + 0x68 * (ch))
#define X_RAW           0x00
#define X_ST            0x04
#define X_ENA           0x08
#define X_CLR           0x0c
#define X_CONF0         0x10
#define X_CONF1         0x14
#define X_FIFO_STATUS   0x18
#define X_LINK1         0x20
#define X_LINK2         0x24
#define X_EOF_DES       0x2c
#define X_PRI           0x40
#define X_PERI_SEL      0x44
#define X_CRC           0x48
#define AXI_INTR_START  (AXI + 0x27c)
#define AXI_INTR_END    (AXI + 0x280)
#define AXI_EXTR_START  (AXI + 0x284)
#define AXI_EXTR_END    (AXI + 0x288)
#define AXI_RESET_AVAIL (AXI + 0x28c)
#define AXI_MISC_CONF   (AXI + 0x2a8)
#define AXI_DATE        (AXI + 0x2d8)
#define AXI_LINK_SWITCH (AXI + 0x2dc)

/* Interrupt bits. */
#define IN_DONE         (1u << 0)
#define IN_SUC_EOF      (1u << 1)
#define IN_DSCR_ERR     (1u << 3)
#define IN_DSCR_EMPTY   (1u << 4)
#define IN_FIFO_UDF     (1u << 6)
#define IN_RESP_ERR     (1u << 7)
#define OUT_DONE        (1u << 0)
#define OUT_EOF         (1u << 1)
#define OUT_DSCR_ERR    (1u << 2)
#define OUT_TOTAL_EOF   (1u << 3)
#define OUT_LINK_SWITCH (1u << 10)

/* Link bits: AHB_DMA_IN_LINK and AHB_DMA_OUT_LINK, and the AXI LINK1s. */
#define IN_LINK_START   (1u << 2)
#define IN_LINK_RESTART (1u << 3)
#define IN_LINK_PARK    (1u << 4)
#define OUT_LINK_START  (1u << 1)
#define OUT_LINK_RESTART (1u << 2)
#define OUT_LINK_PARK   (1u << 3)

#define AHB_MEM_TRANS   (1u << 4)
#define AHB_DATA_BURST  (1u << 3)
#define AXI_MEM_TRANS   (1u << 2)
#define OUT_AUTO_WRBACK (1u << 2)
#define CHECK_OWNER     (1u << 12)
#define CONF0_RST       (1u << 0)

/* Descriptor DW0. */
#define DW0(size, len)  ((size) | (len) << 12)
#define SUC_EOF         (1u << 30)
#define OWNER           (1u << 31)

/* A dummy peripheral, as ESP-IDF picks for memory-to-memory transfers. */
#define AHB_DUMMY       1
#define AXI_DUMMY       6

/* Interrupt sources, in core 0's interrupt matrix status. */
#define INTMTX_STATUS(w)    (0x500d6000 + 0x200 + 4 * (w))
#define SRC_AHB_IN(ch)      (56 + (ch))
#define SRC_AHB_OUT(ch)     (59 + (ch))
#define SRC_AXI_IN(ch)      (62 + (ch))

#define HP_SYS_CLKRST_SOC_CLK_CTRL1 0x500e6018
#define AHB_PDMA_SYS_CLK_EN (1u << 3)

#define DMA_PMS         0x500a6000
#define DMA_AHB_DUMMY_W (DMA_PMS + 0x218)
#define DMA_AXI_DUMMY_R (DMA_PMS + 0x21c)
#define ICM             0x500a4000
#define ICM_INT_RAW     (ICM + 0x0c)
#define ICM_INT_CLR     (ICM + 0x18)
#define ICM_SYS_ADDR    (ICM + 0x38)
#define ICM_SYS_INFO    (ICM + 0x3c)
#define ICM_CPU_ADDR    (ICM + 0x40)
#define ICM_CPU_INFO    (ICM + 0x44)
#define ICM_INT_SYS     0x2
#define ICM_INT_CPU     0x4
#define ICM_WR          0x100

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

static void fill(QTestState *qts, uint32_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *b = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        b[i] = seed + i * 7 + (i >> 8);
    }
    qtest_memwrite(qts, addr, b, len);
}

static void check_copy(QTestState *qts, uint32_t dst, uint32_t src,
                       uint32_t len)
{
    g_autofree uint8_t *a = g_malloc(len);
    g_autofree uint8_t *b = g_malloc(len);

    qtest_memread(qts, src, a, len);
    qtest_memread(qts, dst, b, len);
    g_assert_cmpmem(a, len, b, len);
}

/* The bytes of [addr, addr + len) that are no longer zero. */
static uint32_t nonzero(QTestState *qts, uint32_t addr, uint32_t len)
{
    g_autofree uint8_t *b = g_malloc(len);
    uint32_t n = 0;

    qtest_memread(qts, addr, b, len);
    for (uint32_t i = 0; i < len; i++) {
        n += b[i] != 0;
    }
    return n;
}

/* Set GDMA-AHB channel @ch up for memory-to-memory and start it. */
static void ahb_m2m_start(QTestState *qts, int ch, uint32_t out_desc,
                          uint32_t in_desc)
{
    qtest_writel(qts, AHB_IN_INT(ch) + CLR, 0xff);
    qtest_writel(qts, AHB_OUT_INT(ch) + CLR, 0x7f);
    qtest_writel(qts, AHB_IN(ch) + CONF0, CONF0_RST);
    qtest_writel(qts, AHB_IN(ch) + CONF0, 0);
    qtest_writel(qts, AHB_OUT(ch) + CONF0, CONF0_RST);
    qtest_writel(qts, AHB_OUT(ch) + CONF0, 0);
    qtest_writel(qts, AHB_IN(ch) + PERI_SEL, AHB_DUMMY + ch * 5);
    qtest_writel(qts, AHB_OUT(ch) + PERI_SEL, AHB_DUMMY + ch * 5);
    qtest_writel(qts, AHB_IN(ch) + CONF0, AHB_MEM_TRANS | AHB_DATA_BURST |
                 (3u << 6));
    qtest_writel(qts, AHB_OUT(ch) + CONF0, (1u << 3) | OUT_AUTO_WRBACK |
                 (1u << 5) | (3u << 8));
    qtest_writel(qts, AHB_IN_LINK_ADDR(ch), in_desc);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(ch), out_desc);
    qtest_writel(qts, AHB_IN(ch) + LINK, IN_LINK_START | 1);
    qtest_writel(qts, AHB_OUT(ch) + LINK, OUT_LINK_START);
}

/* Registers and reset values */

/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, AHB_DATE), ==, 0x250220);
    g_assert_cmphex(qtest_readl(qts, AXI_DATE), ==, 0x2303140);
    for (int ch = 0; ch < 3; ch++) {
        g_assert_cmphex(qtest_readl(qts, AHB_IN(ch) + CONF0), ==, 0x40);
        g_assert_cmphex(qtest_readl(qts, AHB_OUT(ch) + CONF0), ==, 0x108);
        g_assert_cmphex(qtest_readl(qts, AHB_IN(ch) + LINK), ==, 0x11);
        g_assert_cmphex(qtest_readl(qts, AHB_OUT(ch) + LINK), ==, 0x8);
        g_assert_cmphex(qtest_readl(qts, AHB_IN(ch) + PERI_SEL), ==, 0x3f);
        g_assert_cmphex(qtest_readl(qts, AHB_OUT(ch) + PERI_SEL), ==, 0x3f);
        g_assert_cmphex(qtest_readl(qts, AHB_IN(ch) + FIFO_STATUS), ==,
                        0x07800003);
        g_assert_cmphex(qtest_readl(qts, AHB_OUT(ch) + FIFO_STATUS), ==,
                        0x07800002);
        g_assert_cmphex(qtest_readl(qts, AHB_IN(ch) + PUSHPOP), ==, 0x800);
        g_assert_cmphex(qtest_readl(qts, AHB_OUT_CRC(ch) + CRC_INIT), ==,
                        0xffffffff);
        g_assert_cmphex(qtest_readl(qts, AXI_IN(ch) + X_CONF0), ==, 0);
        g_assert_cmphex(qtest_readl(qts, AXI_OUT(ch) + X_CONF0), ==, 0x8);
        g_assert_cmphex(qtest_readl(qts, AXI_IN(ch) + X_LINK1), ==, 0x11);
        g_assert_cmphex(qtest_readl(qts, AXI_IN(ch) + X_FIFO_STATUS), ==,
                        0x8803);
        g_assert_cmphex(qtest_readl(qts, AXI_OUT(ch) + X_FIFO_STATUS), ==,
                        0x7f808802);
        g_assert_cmphex(qtest_readl(qts, AXI_RESET_AVAIL + 4 * ch), ==, 1);
    }
    g_assert_cmphex(qtest_readl(qts, AHB_INTR_MEM_START), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AHB_INTR_MEM_END), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AHB_MODULE_CLK_EN), ==, 0x7fff);
    g_assert_cmphex(qtest_readl(qts, AHB_RESP_ERR1), ==, 0x1e);
    g_assert_cmphex(qtest_readl(qts, AXI_INTR_START), ==, 0x30100000);
    g_assert_cmphex(qtest_readl(qts, AXI_INTR_END), ==, 0x8fffffff);
    g_assert_cmphex(qtest_readl(qts, AXI_EXTR_START), ==, 0x30100000);
    g_assert_cmphex(qtest_readl(qts, AXI_EXTR_END), ==, 0x8fffffff);

    /* Masks, write-1-to-clear and read-only registers. */
    qtest_writel(qts, AHB_IN(1) + CONF0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AHB_IN(1) + CONF0), ==, 0xff);
    qtest_writel(qts, AHB_OUT(1) + CONF0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + CONF0), ==, 0x37f);
    qtest_writel(qts, AXI_OUT(1) + X_PRI, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(1) + X_PRI), ==, 0x1ff);
    qtest_writel(qts, AHB_IN_INT(0) + ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(0) + ENA), ==, 0xff);
    qtest_writel(qts, AHB_IN(0) + DSCR, 0x1234);
    g_assert_cmphex(qtest_readl(qts, AHB_IN(0) + DSCR), ==, 0);
    qtest_quit(qts);
}

/* Memory-to-memory with descriptor chains */

/*
 * GDMA-AHB channel 1: three TX descriptors (the last with suc_eof) into
 * two RX descriptors: the data, the RX descriptors written back with the
 * lengths received, owner cleared and suc_eof on the frame's last, the TX
 * descriptors' owner cleared (OUT_AUTO_WRBACK), the descriptor address
 * registers and the interrupts.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_ahb_m2m_chain(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;

    fill(qts, src, 3000, 1);
    desc(qts, tx, OWNER | DW0(1000, 1000), src, tx + 12);
    desc(qts, tx + 12, OWNER | DW0(1500, 1500), src + 1000, tx + 24);
    desc(qts, tx + 24, OWNER | SUC_EOF | DW0(500, 500), src + 2500, 0);
    desc(qts, rx, OWNER | DW0(2048, 0), dst, rx + 12);
    desc(qts, rx + 12, OWNER | DW0(2048, 0), dst + 2048, 0);

    qtest_writel(qts, AHB_IN_INT(1) + ENA, IN_SUC_EOF);
    qtest_writel(qts, AHB_OUT_INT(1) + ENA, OUT_TOTAL_EOF);
    ahb_m2m_start(qts, 1, tx, rx);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + LINK) & OUT_LINK_PARK, ==,
                    0);

    /* The transfer takes time: not done a bus cycle in. */
    qtest_clock_step(qts, 100);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW) & IN_SUC_EOF, ==,
                    0);
    qtest_clock_step(qts, 1000 * 1000);

    check_copy(qts, dst, src, 3000);
    g_assert_cmphex(qtest_readl(qts, rx), ==, DW0(2048, 2048));
    g_assert_cmphex(qtest_readl(qts, rx + 12), ==,
                    SUC_EOF | DW0(2048, 3000 - 2048));
    g_assert_cmphex(qtest_readl(qts, tx), ==, DW0(1000, 1000));
    g_assert_cmphex(qtest_readl(qts, tx + 24), ==, SUC_EOF | DW0(500, 500));

    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_INT(1) + RAW), ==,
                    OUT_DONE | OUT_EOF | OUT_TOTAL_EOF);
    g_assert_cmphex(qtest_readl(qts, AHB_IN(1) + EOF_DES), ==, rx + 12);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + EOF_DES), ==, tx + 24);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + EOF_BFR_DES), ==, tx + 12);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + DSCR_BF0), ==, tx + 24);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + DSCR_BF1), ==, tx + 12);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + DSCR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_DONE_DES(1)), ==, rx + 12);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_DONE_DES(1)), ==, tx + 24);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(1) + LINK) & OUT_LINK_PARK, ==,
                    OUT_LINK_PARK);
    g_assert_cmphex(qtest_readl(qts, AHB_IN(1) + LINK) & IN_LINK_PARK, ==,
                    IN_LINK_PARK);

    /* The masked status, the interrupt lines, and clearing. */
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + ST), ==, IN_SUC_EOF);
    g_assert_true(source_level(qts, SRC_AHB_IN(1)));
    g_assert_true(source_level(qts, SRC_AHB_OUT(1)));
    g_assert_false(source_level(qts, SRC_AHB_IN(2)));
    qtest_writel(qts, AHB_IN_INT(1) + CLR, IN_SUC_EOF);
    g_assert_false(source_level(qts, SRC_AHB_IN(1)));
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==, IN_DONE);
    /* INT_RAW is write-1-to-clear too. */
    qtest_writel(qts, AHB_OUT_INT(1) + RAW, OUT_TOTAL_EOF);
    g_assert_false(source_level(qts, SRC_AHB_OUT(1)));
    qtest_quit(qts);
}

/*
 * A restart: the RX list ends with no EOF yet, raising IN_DSCR_EMPTY;
 * appending a descriptor to it and setting INLINK_RESTART continues the
 * transfer into it.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_ahb_restart(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;

    fill(qts, src, 600, 9);
    desc(qts, tx, OWNER | SUC_EOF | DW0(600, 600), src, 0);
    desc(qts, rx, OWNER | DW0(400, 0), dst, 0);
    ahb_m2m_start(qts, 2, tx, rx);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(2) + RAW), ==,
                    IN_DONE | IN_DSCR_EMPTY);
    g_assert_cmphex(qtest_readl(qts, rx), ==, DW0(400, 400));
    g_assert_cmphex(qtest_readl(qts, AHB_IN(2) + LINK) & IN_LINK_PARK, ==,
                    IN_LINK_PARK);

    desc(qts, rx + 12, OWNER | DW0(400, 0), dst + 400, 0);
    qtest_writel(qts, rx + 8, rx + 12);
    qtest_writel(qts, AHB_IN(2) + LINK, IN_LINK_RESTART | 1);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 600);
    g_assert_cmphex(qtest_readl(qts, rx + 12), ==, SUC_EOF | DW0(400, 200));
    g_assert_cmphex(qtest_readl(qts, AHB_IN(2) + EOF_DES), ==, rx + 12);
    qtest_quit(qts);
}

/*
 * GDMA-AXI channel 0, memory-to-memory, with its double-word-aligned
 * descriptors; a misaligned or unreachable descriptor, or one the CPU
 * owns while owner checking is on, is a descriptor error that halts the
 * channel.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_axi_m2m(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;

    fill(qts, src, 4000, 3);
    desc(qts, tx, OWNER | DW0(2000, 2000), src, tx + 16);
    desc(qts, tx + 16, OWNER | SUC_EOF | DW0(2000, 2000), src + 2000, 0);
    desc(qts, rx, OWNER | DW0(4000, 0), dst, 0);
    qtest_writel(qts, AXI_IN(0) + X_ENA, IN_SUC_EOF);
    qtest_writel(qts, AXI_IN(0) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_OUT(0) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_IN(0) + X_CONF0, AXI_MEM_TRANS | (4u << 4));
    qtest_writel(qts, AXI_OUT(0) + X_CONF0, (1u << 3) | (4u << 5));
    qtest_writel(qts, AXI_IN(0) + X_CONF1, CHECK_OWNER);
    qtest_writel(qts, AXI_OUT(0) + X_CONF1, CHECK_OWNER);
    qtest_writel(qts, AXI_IN(0) + X_LINK2, rx);
    qtest_writel(qts, AXI_OUT(0) + X_LINK2, tx);
    qtest_writel(qts, AXI_IN(0) + X_LINK1, IN_LINK_START | 1);
    qtest_writel(qts, AXI_OUT(0) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 4000);
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(4000, 4000));
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_EOF_DES), ==, rx);
    g_assert_true(source_level(qts, SRC_AXI_IN(0)));
    /* Without OUT_AUTO_WRBACK the TX descriptors keep their owner. */
    g_assert_cmphex(qtest_readl(qts, tx) & OWNER, ==, OWNER);

    /* A misaligned descriptor. */
    qtest_writel(qts, AXI_OUT(0) + X_LINK2, tx + 4);
    qtest_writel(qts, AXI_OUT(0) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(0) + X_RAW) & OUT_DSCR_ERR, ==,
                    OUT_DSCR_ERR);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(0) + X_LINK1) & OUT_LINK_PARK,
                    ==, OUT_LINK_PARK);

    /* One owned by the CPU, with owner checking on. */
    qtest_writel(qts, AXI_OUT(0) + X_CLR, 0x7ff);
    desc(qts, tx, DW0(16, 16), src, 0);
    qtest_writel(qts, AXI_OUT(0) + X_LINK2, tx);
    qtest_writel(qts, AXI_OUT(0) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(0) + X_RAW), ==, OUT_DSCR_ERR);

    /* One outside the accessible memory. */
    qtest_writel(qts, AXI_IN(0) + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_INTR_END, MEM);
    qtest_writel(qts, AXI_EXTR_END, MEM);
    qtest_writel(qts, AXI_IN(0) + X_LINK1, IN_LINK_START | 1);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_RAW), ==, IN_DSCR_ERR);
    qtest_quit(qts);
}

/* CRC */

/*
 * ESP-IDF's gdma_hal_build_parallel_crc_matrix(): shift each LFSR bit and
 * each data bit through the serial CRC to find the rows of the matrix.
 */
static uint32_t lfsr_shift(uint32_t data, uint32_t lfsr, int width,
                           uint32_t poly)
{
    for (int j = 0; j < 8; j++) {
        uint32_t up = lfsr >> (width - 1);

        for (int i = width - 1; i > 0; i--) {
            lfsr &= ~(1u << i);
            if (poly & (1u << i)) {
                lfsr |= (((lfsr >> (i - 1)) ^ up ^ (data >> j)) & 1) << i;
            } else {
                lfsr |= (lfsr & (1u << (i - 1))) << 1;
            }
        }
        lfsr = (lfsr & ~1u) | ((up ^ (data >> j)) & 1);
    }
    return lfsr;
}

static void crc_program(QTestState *qts, uint32_t crc, int width,
                        uint32_t poly, uint32_t init, bool reverse)
{
    uint32_t lfsr_m[32], data_m[8];

    for (int n = 0; n < width; n++) {
        lfsr_m[n] = lfsr_shift(0, 1u << n, width, poly);
    }
    for (int m = 0; m < 8; m++) {
        data_m[7 - m] = lfsr_shift(1u << m, 0, width, poly);
    }
    qtest_writel(qts, crc + CRC_CLEAR, 1);
    qtest_writel(qts, crc + CRC_CLEAR, 0);
    qtest_writel(qts, crc + CRC_INIT, init);
    qtest_writel(qts, crc + CRC_WIDTH, (width - 1) / 8);
    for (int bit = 0; bit < width; bit++) {
        uint32_t lm = 0, dm = 0;

        for (int j = 0; j < width; j++) {
            lm |= ((lfsr_m[j] >> bit) & 1) << j;
        }
        for (int j = 0; j < 8; j++) {
            dm |= ((data_m[j] >> bit) & 1) << j;
        }
        if (reverse) {
            uint32_t r = 0;

            for (int j = 0; j < 8; j++) {
                r |= ((dm >> j) & 1) << (7 - j);
            }
            dm = r;
        }
        qtest_writel(qts, crc + CRC_EN_ADDR, bit);
        qtest_writel(qts, crc + CRC_EN_WR, lm);
        qtest_writel(qts, crc + CRC_DATA_EN_ADDR, bit);
        qtest_writel(qts, crc + CRC_DATA_EN_WR, dm);
        qtest_writel(qts, crc + CRC_WIDTH, (width - 1) / 8 | 4);
        qtest_writel(qts, crc + CRC_WIDTH, (width - 1) / 8);
    }
}

static const char crc_text[] =
    "GDMACRC::TEST::LONGSTRING::REPEAT::GDMACRC::TEST::LONGSTRING::REPEAT::"
    "GDMACRC::TEST::LONGSTRING::REPEAT::GDMACRC::TEST::LONGSTRING::REPEAT::"
    "END!";

/*
 * The CRC of the TX channel over ESP-IDF's test string, for its test's
 * polynomials and expected results.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_crc(void)
{
    static const struct {
        int width;
        uint32_t poly, init, want;
        bool reverse;
    } cases[] = {
        { 8, 0x07, 0x00, 0x08, false },
        { 8, 0x07, 0x00, 0xce, true },
        { 16, 0x1021, 0xffff, 0x0ed7, false },
        { 32, 0x04c11db7, 0xffffffff, 0x6d9bd7d5, false },
    };
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t len = strlen(crc_text);

    qtest_memwrite(qts, src, crc_text, len);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        desc(qts, tx, OWNER | SUC_EOF | DW0(len + 1, len), src, 0);
        desc(qts, rx, OWNER | DW0(len, 0), MEM + 0x2000, 0);
        crc_program(qts, AHB_OUT_CRC(1), cases[i].width, cases[i].poly,
                    cases[i].init, cases[i].reverse);
        ahb_m2m_start(qts, 1, tx, rx);
        qtest_clock_step(qts, 1000 * 1000);
        g_assert_cmphex(qtest_readl(qts, AHB_OUT_CRC(1) + CRC_FINAL), ==,
                        cases[i].want);
    }

    /* GDMA-AXI: CRC-16, at most, here on the RX side. */
    desc(qts, tx, OWNER | SUC_EOF | DW0(len, len), src, 0);
    desc(qts, rx, OWNER | DW0(len, 0), MEM + 0x2000, 0);
    crc_program(qts, AXI_IN(2) + X_CRC, 16, 0x1021, 0xffff, false);
    qtest_writel(qts, AXI_IN(2) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_OUT(2) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_IN(2) + X_CONF0, AXI_MEM_TRANS);
    qtest_writel(qts, AXI_IN(2) + X_LINK2, rx);
    qtest_writel(qts, AXI_OUT(2) + X_LINK2, tx);
    qtest_writel(qts, AXI_IN(2) + X_LINK1, IN_LINK_START);
    qtest_writel(qts, AXI_OUT(2) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(2) + X_CRC + CRC_FINAL), ==,
                    0x0ed7);
    qtest_quit(qts);
}

/* Arbitration */

#define ARB_LEN     4000

/*
 * Memory-to-memory on GDMA-AHB channels 1 and 2, ARB_LEN bytes each from
 * one descriptor, channel 2 started a moment before channel 1; step until
 * either has its suc_eof and return which, with the other's bytes copied
 * in *other.
 */
static int race(QTestState *qts, uint32_t *other)
{
    uint32_t src = MEM + 0x1000, dst[3] = { 0, MEM + 0x4000, MEM + 0x8000 };

    fill(qts, src, ARB_LEN, 5);
    for (int ch = 1; ch <= 2; ch++) {
        uint32_t d = MEM + 0x40 * ch;

        qtest_memset(qts, dst[ch], 0, ARB_LEN);
        desc(qts, d, OWNER | SUC_EOF | DW0(ARB_LEN, ARB_LEN), src, 0);
        desc(qts, d + 0x20, OWNER | DW0(ARB_LEN, 0), dst[ch], 0);
    }
    for (int ch = 2; ch >= 1; ch--) {
        ahb_m2m_start(qts, ch, MEM + 0x40 * ch, MEM + 0x40 * ch + 0x20);
        qtest_clock_step(qts, 50);
    }
    for (int i = 0; i < 100000; i++) {
        for (int ch = 1; ch <= 2; ch++) {
            if (qtest_readl(qts, AHB_IN_INT(ch) + RAW) & IN_SUC_EOF) {
                *other = nonzero(qts, dst[3 - ch], ARB_LEN);
                return ch;
            }
        }
        qtest_clock_step(qts, 200);
    }
    g_assert_not_reached();
}

static void set_pri(QTestState *qts, int ch, uint32_t pri)
{
    qtest_writel(qts, AHB_IN(ch) + PRI, pri);
    qtest_writel(qts, AHB_OUT(ch) + PRI, pri);
}

static void set_weight(QTestState *qts, int ch, uint32_t weight)
{
    qtest_writel(qts, AHB_IN_CRC(ch) + ARB_WEIGHT, weight);
    qtest_writel(qts, AHB_OUT_CRC(ch) + ARB_WEIGHT, weight);
}

/*
 * The channel of higher priority has the bus until it is done; with
 * priority arbitration off they share it; with weighted arbitration the
 * heavier channel takes its share whatever the priorities.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_arbitration(void)
{
    QTestState *qts = start();
    uint32_t other;

    set_pri(qts, 1, 0);
    set_pri(qts, 2, 5);
    g_assert_cmpint(race(qts, &other), ==, 2);
    g_assert_cmpuint(other, <=, 64);

    set_pri(qts, 1, 5);
    set_pri(qts, 2, 0);
    g_assert_cmpint(race(qts, &other), ==, 1);
    g_assert_cmpuint(other, <=, 64);

    /* Round robin: the other is nearly done too. */
    qtest_writel(qts, AHB_MISC_CONF, 1u << 2);
    race(qts, &other);
    g_assert_cmpuint(other, >, ARB_LEN * 9 / 10);
    qtest_writel(qts, AHB_MISC_CONF, 0);

    /* Weights 1 and 3: channel 2 gets three quarters of the bus. */
    set_pri(qts, 1, 5);
    set_pri(qts, 2, 0);
    set_weight(qts, 1, 1);
    set_weight(qts, 2, 3);
    qtest_writel(qts, AHB_ARB_TIMEOUT, 2000);
    qtest_writel(qts, AHB_WEIGHT_EN, 1);
    g_assert_cmpint(race(qts, &other), ==, 2);
    g_assert_cmpuint(other, >, ARB_LEN / 4);
    g_assert_cmpuint(other, <, ARB_LEN / 2);
    qtest_quit(qts);
}

/* The DMA APM */

/*
 * The DMA APM refusing GDMA-AHB's memory-to-memory writes (its DUMMY
 * master): the transfer goes on, both channels raise *_AHBINF_RESP_ERR
 * and AHBINF_RESP_ERR_STATUS records the last refused write, and the ICM
 * the first, the TX descriptor's write-back, with channel 2's fixed
 * master ID, 16 + 12.
 * GDMA-AXI's refused descriptor read is a descriptor error, recorded by
 * the ICM as port 8's.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_apm(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;

    fill(qts, src, 64, 1);
    qtest_memset(qts, dst, 0, 64);
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    qtest_writel(qts, DMA_AHB_DUMMY_W, 0);
    ahb_m2m_start(qts, 2, tx, rx);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmpuint(nonzero(qts, dst, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(2) + RAW), ==,
                    IN_DONE | IN_SUC_EOF | IN_RESP_ERR);
    g_assert_cmphex(qtest_readl(qts, AHB_RESP_ERR0), ==, rx);
    /* The descriptor write-back last: WR, ID 12, RX channel 2. */
    g_assert_cmphex(qtest_readl(qts, AHB_RESP_ERR1), ==,
                    1 | 12 << 1 | 2 << 5);
    /* The descriptor was not written back. */
    g_assert_cmphex(qtest_readl(qts, rx), ==, OWNER | DW0(64, 0));
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW) & ICM_INT_CPU, ==,
                    ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_INT(2) + RAW), ==,
                    OUT_DONE | OUT_EOF | OUT_TOTAL_EOF | (1u << 6));
    g_assert_cmphex(qtest_readl(qts, tx), ==, OWNER | SUC_EOF | DW0(64, 64));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, tx);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | (16 + 12));

    qtest_writel(qts, DMA_AXI_DUMMY_R, 0);
    qtest_writel(qts, AXI_OUT(1) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_OUT(1) + X_LINK2, tx);
    qtest_writel(qts, AXI_OUT(1) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(1) + X_RAW), ==, OUT_DSCR_ERR);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW) & ICM_INT_SYS, ==,
                    ICM_INT_SYS);
    g_assert_cmphex(qtest_readl(qts, ICM_SYS_ADDR), ==, tx);
    g_assert_cmphex(qtest_readl(qts, ICM_SYS_INFO), ==, 0x80 | AXI_DUMMY);
    qtest_quit(qts);
}

/* Clock and FIFO */

/*
 * With AHB_PDMA_SYS_CLK gated a transfer stops where it is and the
 * registers take no writes; it goes on once the clock runs. The FIFO's
 * push and pop registers.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_clock_and_fifo(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000, ctrl1;

    fill(qts, src, 4000, 1);
    qtest_memset(qts, dst, 0, 4000);
    desc(qts, tx, OWNER | SUC_EOF | DW0(4000, 4000), src, 0);
    desc(qts, rx, OWNER | DW0(4000, 0), dst, 0);
    ahb_m2m_start(qts, 1, tx, rx);
    qtest_clock_step(qts, 2000);
    ctrl1 = qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1,
                 ctrl1 & ~AHB_PDMA_SYS_CLK_EN);
    qtest_writel(qts, AHB_IN(0) + PRI, 3);
    g_assert_cmphex(qtest_readl(qts, AHB_IN(0) + PRI), ==, 0);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW) & IN_SUC_EOF, ==,
                    0);
    g_assert_cmpuint(nonzero(qts, dst, 4000), <, 4000);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1, ctrl1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);
    check_copy(qts, dst, src, 4000);

    /* Push into a TX FIFO, pop from an empty RX FIFO. */
    qtest_writel(qts, AHB_OUT(0) + PUSHPOP, 0x5a | (1u << 9));
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(0) + FIFO_STATUS) & 0x7f03, ==,
                    0x100);
    qtest_writel(qts, AHB_IN(0) + PUSHPOP, 1u << 12);
    g_assert_cmphex(qtest_readl(qts, AHB_IN(0) + PUSHPOP), ==, 0x800);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(0) + RAW), ==, IN_FIFO_UDF);
    qtest_quit(qts);
}

/*
 * GDMA-AXI's linked list switch: with LINK_SWITCH_STATE_CHn set, the TX
 * channel's first fetch from a restarted list raises OUT_LINK_SWITCH and
 * clears the bit.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_axi_link_switch(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;

    fill(qts, src, 64, 1);
    desc(qts, tx, OWNER | DW0(32, 32), src, 0);
    desc(qts, rx, OWNER | DW0(128, 0), MEM + 0x2000, 0);
    qtest_writel(qts, AXI_IN(1) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_OUT(1) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_IN(1) + X_CONF0, AXI_MEM_TRANS);
    qtest_writel(qts, AXI_IN(1) + X_LINK2, rx);
    qtest_writel(qts, AXI_OUT(1) + X_LINK2, tx);
    qtest_writel(qts, AXI_IN(1) + X_LINK1, IN_LINK_START);
    qtest_writel(qts, AXI_OUT(1) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(1) + X_RAW), ==, OUT_DONE);

    qtest_writel(qts, AXI_LINK_SWITCH, 1u << 1);
    desc(qts, tx + 16, OWNER | SUC_EOF | DW0(32, 32), src + 32, 0);
    qtest_writel(qts, tx + 8, tx + 16);
    qtest_writel(qts, AXI_OUT(1) + X_LINK1, OUT_LINK_RESTART);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT(1) + X_RAW), ==,
                    OUT_DONE | OUT_EOF | OUT_TOTAL_EOF | OUT_LINK_SWITCH);
    g_assert_cmphex(qtest_readl(qts, AXI_LINK_SWITCH), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(1) + X_RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    check_copy(qts, MEM + 0x2000, src, 64);
    qtest_quit(qts);
}

/*
 * With *_ETM_EN set, *LINK_START is ignored and the ETM tasks
 * IN_START_CH1 and OUT_START_CH1 (task lines 1 and 4) start the channels.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_etm_tasks(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;

    fill(qts, src, 256, 1);
    qtest_memset(qts, dst, 0, 256);
    desc(qts, tx, OWNER | SUC_EOF | DW0(256, 256), src, 0);
    desc(qts, rx, OWNER | DW0(256, 0), dst, 0);
    ahb_m2m_start(qts, 1, tx, rx);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==,
                    IN_DONE | IN_SUC_EOF);

    qtest_writel(qts, AHB_IN_INT(1) + CLR, 0xff);
    qtest_memset(qts, dst, 0, 256);
    desc(qts, tx, OWNER | SUC_EOF | DW0(256, 256), src, 0);
    desc(qts, rx, OWNER | DW0(256, 0), dst, 0);
    qtest_writel(qts, AHB_IN(1) + CONF0, qtest_readl(qts, AHB_IN(1) + CONF0) |
                 (1u << 5));
    qtest_writel(qts, AHB_OUT(1) + CONF0,
                 qtest_readl(qts, AHB_OUT(1) + CONF0) | (1u << 6));
    qtest_writel(qts, AHB_IN(1) + LINK, IN_LINK_START | 1);
    qtest_writel(qts, AHB_OUT(1) + LINK, OUT_LINK_START);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==, 0);

    qtest_set_irq_in(qts, "/machine/ahb-gdma", "etm-task", 1, 1);
    qtest_set_irq_in(qts, "/machine/ahb-gdma", "etm-task", 4, 1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    check_copy(qts, dst, src, 256);
    qtest_quit(qts);
}

/* Play peripheral @periph through the controller's "periph" hook. */
static char *periph_cmd(QTestState *qts, const char *dev, const char *cmd)
{
    QDict *rsp;
    char *r;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': %s, 'property': 'periph', 'value': %s}}",
                    dev, cmd);
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    rsp = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                    "{'path': %s, 'property': 'periph'}}", dev);
    g_assert(qdict_haskey(rsp, "return"));
    r = g_strdup(qdict_get_str(rsp, "return"));
    qobject_unref(rsp);
    return r;
}

static unsigned periph_push(QTestState *qts, unsigned periph,
                            const uint8_t *b, unsigned len, unsigned flags)
{
    g_autoptr(GString) cmd = g_string_new(NULL);
    g_autofree char *r = NULL;

    g_string_printf(cmd, "push %u ", periph);
    for (unsigned i = 0; i < len; i++) {
        g_string_append_printf(cmd, "%02x", b[i]);
    }
    g_string_append_printf(cmd, " %u", flags);
    r = periph_cmd(qts, "/machine/ahb-gdma", cmd->str);
    return atoi(r);
}

/* Pull up to @len bytes into @b: the count, and in *eof the frame's end. */
static unsigned periph_pull(QTestState *qts, unsigned periph, uint8_t *b,
                            unsigned len, bool *eof)
{
    g_autofree char *cmd = g_strdup_printf("pull %u %u", periph, len);
    g_autofree char *r = periph_cmd(qts, "/machine/ahb-gdma", cmd);
    unsigned n, e;
    char *hex;

    g_assert_cmpint(sscanf(r, "%u %u", &n, &e), ==, 2);
    hex = strrchr(r, ' ') + 1;
    for (unsigned i = 0; i < n; i++) {
        char byte[3] = { hex[2 * i], hex[2 * i + 1], 0 };

        b[i] = strtoul(byte, NULL, 16);
    }
    *eof = e;
    return n;
}

#define I2S0            3
#define DMA_I2S0_W      (DMA_PMS + 0x134)

/*
 * A peripheral on GDMA-AHB, I2S0, as a peripheral model drives it: the TX
 * channel selecting it fills its FIFO and waits; the peripheral pulls the
 * bytes, up to each frame's end, and taking the last completes the
 * descriptors; what it pushes the RX channel selecting it writes into its
 * buffers, closing the descriptor at the frame's end. Its accesses are
 * AHB_PDMA_I2S0's, recorded as master 16 + 3 when the DMA APM refuses
 * them.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_periph(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;
    uint8_t want[150], got[150], b[64];
    unsigned n = 0;
    bool eof = false;

    /* No running channel selects it: nothing to pull, no room. */
    g_assert_cmpuint(periph_pull(qts, I2S0, b, 64, &eof), ==, 0);
    g_assert_cmpuint(periph_push(qts, I2S0, b, 4, 0), ==, 0);

    fill(qts, src, 150, 7);
    qtest_memread(qts, src, want, 150);
    desc(qts, tx, OWNER | DW0(100, 100), src, tx + 12);
    desc(qts, tx + 12, OWNER | SUC_EOF | DW0(50, 50), src + 100, 0);
    qtest_writel(qts, AHB_OUT(0) + PERI_SEL, I2S0);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(0), tx);
    qtest_writel(qts, AHB_OUT(0) + LINK, OUT_LINK_START);
    qtest_clock_step(qts, 100000);
    /* The FIFO is full and nothing is sent until the peripheral takes. */
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(0) + FIFO_STATUS) & 0x7f01, ==,
                    0x4001);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_INT(0) + RAW), ==, 0);
    while (!eof && n < 150) {
        n += periph_pull(qts, I2S0, got + n, 150 - n, &eof);
        qtest_clock_step(qts, 10000);
    }
    g_assert_true(eof);
    g_assert_cmpuint(n, ==, 150);
    g_assert_cmpmem(want, 150, got, 150);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_INT(0) + RAW), ==,
                    OUT_DONE | OUT_EOF | OUT_TOTAL_EOF);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT(0) + EOF_DES), ==, tx + 12);

    desc(qts, rx, OWNER | DW0(100, 0), dst, 0);
    qtest_writel(qts, AHB_IN(0) + PERI_SEL, I2S0);
    qtest_writel(qts, AHB_IN_LINK_ADDR(0), rx);
    qtest_writel(qts, AHB_IN(0) + LINK, IN_LINK_START | 1);
    g_assert_cmpuint(periph_push(qts, I2S0, want, 40, 0), ==, 40);
    qtest_clock_step(qts, 100000);
    g_assert_cmpuint(periph_push(qts, I2S0, want + 40, 30, 1), ==, 30);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(100, 70));
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(0) + RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    check_copy(qts, dst, src, 70);

    /* AHB_PDMA_I2S0 may not write: the APM refuses the next frame. */
    qtest_writel(qts, DMA_I2S0_W, 0);
    qtest_writel(qts, AHB_IN_INT(0) + CLR, 0xff);
    desc(qts, rx, OWNER | DW0(100, 0), dst + 0x100, 0);
    qtest_writel(qts, AHB_IN(0) + LINK, IN_LINK_START | 1);
    g_assert_cmpuint(periph_push(qts, I2S0, want, 8, 1), ==, 8);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(0) + RAW), ==,
                    IN_DONE | IN_SUC_EOF | IN_RESP_ERR);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, dst + 0x100);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | (16 + I2S0));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/gdma/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/gdma/ahb-m2m-chain", test_ahb_m2m_chain);
    qtest_add_func("/esp32p4/gdma/ahb-restart", test_ahb_restart);
    qtest_add_func("/esp32p4/gdma/axi-m2m", test_axi_m2m);
    qtest_add_func("/esp32p4/gdma/crc", test_crc);
    qtest_add_func("/esp32p4/gdma/arbitration", test_arbitration);
    qtest_add_func("/esp32p4/gdma/apm", test_apm);
    qtest_add_func("/esp32p4/gdma/clock-and-fifo", test_clock_and_fifo);
    qtest_add_func("/esp32p4/gdma/axi-link-switch", test_axi_link_switch);
    qtest_add_func("/esp32p4/gdma/etm-tasks", test_etm_tasks);
    qtest_add_func("/esp32p4/gdma/periph", test_periph);
    return g_test_run();
}
