/*
 * QTest testcase for the ESP32-P4 VDMA, a DesignWare AXI DMA controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The tests move memory to memory in L2MEM, past the 128 KiB the L2
 * cache holds from reset, stepping virtual time while the channels run
 * at GDMA_SYS_CLK, which they enable first, as ESP-IDF's driver does.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define VDMA            0x50081000
#define MEM             0x4ff40000

#define ID0             (VDMA + 0x00)
#define COMPVER0        (VDMA + 0x08)
#define CFG0            (VDMA + 0x10)
#define CHEN0           (VDMA + 0x18)
#define CHEN1           (VDMA + 0x1c)
#define INTSTATUS0      (VDMA + 0x30)
#define COMMON_INTCLEAR (VDMA + 0x38)
#define COMMON_INTST_ENA (VDMA + 0x40)
#define COMMON_INTSIG_ENA (VDMA + 0x48)
#define COMMON_INTSTATUS (VDMA + 0x50)
#define RESET0          (VDMA + 0x58)
#define LOWPOWER_CFG0   (VDMA + 0x60)
#define LOWPOWER_CFG1   (VDMA + 0x64)

#define CH(n)           (VDMA + 0x100 * (n))
#define SAR0            0x00
#define DAR0            0x08
#define BLOCK_TS0       0x10
#define CTL0            0x18
#define CTL1            0x1c
#define CH_CFG0         0x20
#define CH_CFG1         0x24
#define LLP0            0x28
#define STATUS0         0x30
#define SWHSSRC0        0x38
#define RESUMEREQ       0x48
#define INTST_ENA0      0x80
#define INTST_ENA1      0x84
#define INTSTATUS       0x88
#define INTSIG_ENA0     0x90
#define INTCLEAR0       0x98

#define DMAC_EN         (1u << 0)
#define INT_EN          (1u << 1)

/* CTL0: masters, increments, widths and burst sizes. */
#define SMS2            (1u << 0)
#define DMS2            (1u << 2)
#define SINC_FIXED      (1u << 4)
#define DINC_FIXED      (1u << 6)
#define SRC_W(w)        ((w) << 8)
#define DST_W(w)        ((w) << 11)
#define SRC_MSIZE(m)    ((m) << 14)
#define DST_MSIZE(m)    ((m) << 18)
#define CTL0_M2M        (SMS2 | DMS2 | SRC_W(2) | DST_W(2) | SRC_MSIZE(1) | \
                         DST_MSIZE(1))
/* CTL1. */
#define IOC_BLKTFR      (1u << 26)
#define LLI_LAST        (1u << 30)
#define LLI_VALID       (1u << 31)

/* Channel interrupts. */
#define BLOCK_TFR_DONE  (1u << 0)
#define DMA_TFR_DONE    (1u << 1)
#define SRC_TRANSCOMP   (1u << 3)
#define DST_DEC_ERR     (1u << 6)
#define DST_SLV_ERR     (1u << 8)
#define INVALID_ERR     (1u << 13)
#define MULTIBLKTYPE_ERR (1u << 14)
#define SLVIF_WR2RO     (1u << 17)
#define SLVIF_RD2RWO    (1u << 18)
#define SLVIF_WRONCHEN  (1u << 19)
#define CH_SRC_SUSPENDED (1u << 28)
#define CH_SUSPENDED    (1u << 29)
#define CH_DISABLED     (1u << 30)
#define CH_ABORTED      (1u << 31)
#define CI_WR2RO        (1u << 1)

#define HP_SYS_CLKRST_SOC_CLK_CTRL0 0x500e6014
#define HP_SYS_CLKRST_SOC_CLK_CTRL1 0x500e6018
#define GDMA_CPU_CLK_EN (1u << 13)
#define GDMA_SYS_CLK_EN (1u << 5)

#define INTMTX_STATUS0  (0x500d6000 + 0x200)
#define SRC_DW_GDMA     24

#define DMA_PMS_GDMA_CH0_W (0x500a6000 + 0x10c)
#define ICM             0x500a4000
#define ICM_INT_RAW     (ICM + 0x0c)
#define ICM_SYS_ADDR    (ICM + 0x38)
#define ICM_SYS_INFO    (ICM + 0x3c)
#define ICM_INT_SYS     0x2
#define ICM_WR          0x100

static void clocks_on(QTestState *qts)
{
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0,
                 qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0) |
                 GDMA_CPU_CLK_EN);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1,
                 qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1) |
                 GDMA_SYS_CLK_EN);
}

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M");

    clocks_on(qts);
    qtest_writel(qts, CFG0, DMAC_EN | INT_EN);
    return qts;
}

static void fill(QTestState *qts, uint32_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *b = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        b[i] = seed + i * 13 + (i >> 8) + 1;
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

static bool irq_level(QTestState *qts)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> SRC_DW_GDMA) & 1;
}

static void enable(QTestState *qts, int n, bool on)
{
    qtest_writel(qts, CHEN0, (on ? 0x101u : 0x100u) << (n - 1));
}

/* Program channel @n for one block of @bytes, 32-bit items. */
static void block(QTestState *qts, int n, uint32_t src, uint32_t dst,
                  uint32_t bytes, uint32_t ctl1)
{
    qtest_writel(qts, CH(n) + SAR0, src);
    qtest_writel(qts, CH(n) + DAR0, dst);
    qtest_writel(qts, CH(n) + BLOCK_TS0, bytes / 4 - 1);
    qtest_writel(qts, CH(n) + CTL0, CTL0_M2M);
    qtest_writel(qts, CH(n) + CTL1, ctl1);
    qtest_writel(qts, CH(n) + CH_CFG1, 0x6001b & ~7);
}

/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_reset_values(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M");

    /* Behind GDMA_SYS_CLK, off from reset: no write lands. */
    qtest_writel(qts, CH(1) + SAR0, 0x1234);
    g_assert_cmphex(qtest_readl(qts, CH(1) + SAR0), ==, 0);
    clocks_on(qts);

    g_assert_cmphex(qtest_readl(qts, ID0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, COMPVER0), ==, 0x3230302a);
    g_assert_cmphex(qtest_readl(qts, CFG0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, COMMON_INTST_ENA), ==, 0x1fff8f);
    g_assert_cmphex(qtest_readl(qts, COMMON_INTSIG_ENA), ==, 0x1fff8f);
    g_assert_cmphex(qtest_readl(qts, LOWPOWER_CFG0), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, LOWPOWER_CFG1), ==, 0x404040);
    for (int n = 1; n <= 4; n++) {
        g_assert_cmphex(qtest_readl(qts, CH(n) + CTL0), ==, 0x1200);
        g_assert_cmphex(qtest_readl(qts, CH(n) + CH_CFG1), ==, 0x6001b);
        g_assert_cmphex(qtest_readl(qts, CH(n) + INTST_ENA0), ==,
                        0xfa3f7ffb);
        g_assert_cmphex(qtest_readl(qts, CH(n) + INTSIG_ENA0), ==,
                        0xfa3f7ffb);
        g_assert_cmphex(qtest_readl(qts, CH(n) + INTST_ENA1), ==, 0xf);
    }
    qtest_writel(qts, CH(2) + CTL0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CH(2) + CTL0), ==, 0x7fffff55);
    qtest_writel(qts, CH(2) + CH_CFG1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CH(2) + CH_CFG1), ==, 0x7f8e319f);
    qtest_writel(qts, CH(2) + LLP0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CH(2) + LLP0), ==, 0xffffffc1);

    /* The register file's own errors. */
    qtest_writel(qts, COMPVER0, 0);
    g_assert_cmphex(qtest_readl(qts, COMMON_INTSTATUS), ==, CI_WR2RO);
    qtest_writel(qts, COMMON_INTCLEAR, CI_WR2RO);
    g_assert_cmphex(qtest_readl(qts, COMMON_INTSTATUS), ==, 0);
    qtest_readl(qts, CH(3) + INTCLEAR0);
    qtest_writel(qts, CH(3) + STATUS0, 1);
    g_assert_cmphex(qtest_readl(qts, CH(3) + INTSTATUS), ==,
                    SLVIF_RD2RWO | SLVIF_WR2RO);

    /* DMAC_RST resets the controller and clears itself. */
    qtest_writel(qts, RESET0, 1);
    g_assert_cmphex(qtest_readl(qts, RESET0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(2) + CTL0), ==, 0x1200);
    g_assert_cmphex(qtest_readl(qts, CH(3) + INTSTATUS), ==, 0);
    qtest_quit(qts);
}

/*
 * A single contiguous block: the copy, DMA_TFR_DONE without
 * BLOCK_TFR_DONE (IOC_BLKTFR clear), the channel disabled, the
 * completed size, and the interrupt as INT_EN and the signal enables
 * gate it.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_single_block(void)
{
    QTestState *qts = start();
    uint32_t src = MEM, dst = MEM + 0x1000;

    fill(qts, src, 1024, 1);
    block(qts, 1, src, dst, 1024, 0);
    enable(qts, 1, true);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 1);
    qtest_clock_step(qts, 100);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==, 0);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 1024);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==, DMA_TFR_DONE);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(1) + STATUS0), ==, 256);
    g_assert_cmphex(qtest_readl(qts, INTSTATUS0), ==, 1);
    g_assert_true(irq_level(qts));

    qtest_writel(qts, CFG0, DMAC_EN);
    g_assert_false(irq_level(qts));
    qtest_writel(qts, CFG0, DMAC_EN | INT_EN);
    qtest_writel(qts, CH(1) + INTSIG_ENA0, 0);
    g_assert_false(irq_level(qts));
    g_assert_cmphex(qtest_readl(qts, INTSTATUS0), ==, 0);
    qtest_writel(qts, CH(1) + INTSIG_ENA0, 0xffffffff);
    g_assert_true(irq_level(qts));
    qtest_writel(qts, CH(1) + INTCLEAR0, DMA_TFR_DONE);
    g_assert_false(irq_level(qts));
    qtest_quit(qts);
}

/* LLIs: 64 bytes each, 64-byte aligned. */
static void lli(QTestState *qts, uint32_t at, uint32_t src, uint32_t dst,
                uint32_t bytes, uint32_t next, uint32_t ctl1)
{
    qtest_memset(qts, at, 0, 64);
    qtest_writel(qts, at + 0x00, src);
    qtest_writel(qts, at + 0x08, dst);
    qtest_writel(qts, at + 0x10, bytes / 4 - 1);
    qtest_writel(qts, at + 0x18, next | 1);
    qtest_writel(qts, at + 0x20, CTL0_M2M);
    qtest_writel(qts, at + 0x24, ctl1);
}

/*
 * A linked list of three blocks: each LLI written back with VALID clear,
 * the completed size and BLOCK_TFR_DONE, the last with DMA_TFR_DONE; an
 * LLI not yet valid raises SHADOWREG_OR_LLI_INVALID_ERR, LLP0 pointing at
 * it, until BLK_TFR_RESUMEREQ after it is made valid.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_linked_list(void)
{
    QTestState *qts = start();
    uint32_t list = MEM + 0x8000, src = MEM, dst = MEM + 0x2000;

    fill(qts, src, 768, 2);
    lli(qts, list, src, dst, 256, list + 64, LLI_VALID | IOC_BLKTFR);
    lli(qts, list + 64, src + 256, dst + 256, 256, list + 128,
        IOC_BLKTFR);
    lli(qts, list + 128, src + 512, dst + 512, 256, 0,
        LLI_VALID | LLI_LAST | IOC_BLKTFR);
    qtest_writel(qts, CH(2) + CH_CFG0, 3 | 3 << 2);
    qtest_writel(qts, CH(2) + CH_CFG1, 0);
    qtest_writel(qts, CH(2) + LLP0, list | 1);
    enable(qts, 2, true);
    qtest_clock_step(qts, 1000 * 1000);

    g_assert_cmphex(qtest_readl(qts, CH(2) + INTSTATUS), ==,
                    BLOCK_TFR_DONE | INVALID_ERR);
    g_assert_cmphex(qtest_readl(qts, CH(2) + LLP0) & ~0x3fu, ==, list + 64);
    g_assert_cmphex(qtest_readl(qts, list + 0x24), ==, IOC_BLKTFR);
    g_assert_cmphex(qtest_readl(qts, list + 0x30), ==, 64);
    g_assert_cmphex(qtest_readl(qts, list + 0x34), ==, 1u << 30);
    check_copy(qts, dst, src, 256);

    qtest_writel(qts, CH(2) + INTCLEAR0, 0xffffffff);
    qtest_writel(qts, list + 64 + 0x24, LLI_VALID | IOC_BLKTFR);
    qtest_writel(qts, CH(2) + RESUMEREQ, 1);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 768);
    g_assert_cmphex(qtest_readl(qts, CH(2) + INTSTATUS), ==,
                    BLOCK_TFR_DONE | DMA_TFR_DONE);
    g_assert_cmphex(qtest_readl(qts, list + 128 + 0x24), ==,
                    LLI_LAST | IOC_BLKTFR);
    g_assert_cmphex(qtest_readl(qts, list + 128 + 0x34), ==, 3u << 30);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    qtest_quit(qts);
}

/*
 * Shadow registers, as ESP-IDF's test drives them: the first block
 * valid, not last; at its end the shadow is no longer valid, so the
 * channel waits; marked last and valid, it runs the same block again and
 * ends.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_shadow(void)
{
    QTestState *qts = start();
    uint32_t src = MEM, dst = MEM + 0x1000;

    fill(qts, src, 256, 3);
    qtest_writel(qts, CH(3) + CH_CFG0, 2 | 2 << 2);
    block(qts, 3, src, dst, 256, LLI_VALID);
    enable(qts, 3, true);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 256);
    g_assert_cmphex(qtest_readl(qts, CH(3) + INTSTATUS), ==, INVALID_ERR);
    g_assert_cmphex(qtest_readl(qts, CH(3) + CTL1) & LLI_VALID, ==, 0);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 1u << 2);

    qtest_memset(qts, dst, 0, 256);
    qtest_writel(qts, CH(3) + INTCLEAR0, INVALID_ERR);
    qtest_writel(qts, CH(3) + CTL1, LLI_LAST | LLI_VALID);
    /* A write to the shadow block while it is valid is refused. */
    qtest_writel(qts, CH(3) + SAR0, 0);
    g_assert_cmphex(qtest_readl(qts, CH(3) + INTSTATUS), ==, 1u << 20);
    qtest_writel(qts, CH(3) + RESUMEREQ, 1);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 256);
    g_assert_cmphex(qtest_readl(qts, CH(3) + INTSTATUS), ==,
                    (1u << 20) | DMA_TFR_DONE);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    qtest_quit(qts);
}

/*
 * Auto-reload: each block reloads SAR, DAR and BLOCK_TS, and the
 * channel waits after a block until its BLOCK_TFR_DONE is cleared; the
 * block after which both types are contiguous is the last. The channel's
 * registers but CFG0 take no writes while it runs.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_reload(void)
{
    QTestState *qts = start();
    uint32_t src = MEM, dst = MEM + 0x1000;

    fill(qts, src, 128, 4);
    qtest_writel(qts, CH(4) + CH_CFG0, 1 | 1 << 2);
    block(qts, 4, src, dst, 128, IOC_BLKTFR);
    enable(qts, 4, true);
    for (int i = 0; i < 3; i++) {
        qtest_clock_step(qts, 1000 * 1000);
        check_copy(qts, dst, src, 128);
        g_assert_cmphex(qtest_readl(qts, CH(4) + INTSTATUS), ==,
                        BLOCK_TFR_DONE);
        g_assert_cmphex(qtest_readl(qts, CH(4) + SAR0), ==, src);
        qtest_memset(qts, dst, 0, 128);
        qtest_writel(qts, CH(4) + INTCLEAR0, BLOCK_TFR_DONE);
    }
    qtest_writel(qts, CH(4) + SAR0, src + 4);
    g_assert_cmphex(qtest_readl(qts, CH(4) + INTSTATUS), ==, SLVIF_WRONCHEN);
    qtest_writel(qts, CH(4) + INTCLEAR0, SLVIF_WRONCHEN);
    qtest_writel(qts, CH(4) + CH_CFG0, 0);
    qtest_clock_step(qts, 1000 * 1000);
    check_copy(qts, dst, src, 128);
    g_assert_cmphex(qtest_readl(qts, CH(4) + INTSTATUS), ==,
                    BLOCK_TFR_DONE | DMA_TFR_DONE);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);

    /* Shadow on one side and a linked list on the other do not go. */
    qtest_writel(qts, CH(4) + INTCLEAR0, 0xffffffff);
    qtest_writel(qts, CH(4) + CH_CFG0, 2 | 3 << 2);
    enable(qts, 4, true);
    g_assert_cmphex(qtest_readl(qts, CH(4) + INTSTATUS), ==,
                    MULTIBLKTYPE_ERR);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    qtest_quit(qts);
}

/*
 * Software handshaking from a peripheral source at a fixed address: the
 * channel moves a burst per SWHSSRC request and a single item per single
 * request, clearing each request and raising SRC_TRANSCOMP, until the
 * block is done.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_sw_handshake(void)
{
    QTestState *qts = start();
    uint32_t fifo = MEM, dst = MEM + 0x1000;

    qtest_writel(qts, fifo, 0xdeadbeef);
    qtest_memset(qts, dst, 0, 64);
    block(qts, 1, fifo, dst, 32, 0);
    qtest_writel(qts, CH(1) + CTL0, CTL0_M2M | SINC_FIXED);
    /* P2M with the DMAC as flow controller, software handshake. */
    qtest_writel(qts, CH(1) + CH_CFG1, 2 | (1u << 3));
    enable(qts, 1, true);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0);

    qtest_writel(qts, CH(1) + SWHSSRC0, 0x3);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, CH(1) + SWHSSRC0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==, SRC_TRANSCOMP);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * i), ==, 0xdeadbeef);
    }
    g_assert_cmphex(qtest_readl(qts, dst + 16), ==, 0);

    qtest_writel(qts, CH(1) + SWHSSRC0, 0xc);
    qtest_clock_step(qts, 100000);
    qtest_writel(qts, CH(1) + SWHSSRC0, 0x3);
    qtest_clock_step(qts, 100000);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * i), ==, 0xdeadbeef);
    }
    g_assert_cmphex(qtest_readl(qts, dst + 32), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==,
                    SRC_TRANSCOMP | DMA_TFR_DONE);
    qtest_quit(qts);
}

/*
 * Suspending stops the transfer and resuming continues it; disabling a
 * running channel ends it with CH_DISABLED, aborting with CH_ABORTED.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_suspend_disable_abort(void)
{
    QTestState *qts = start();
    uint32_t src = MEM, dst = MEM + 0x10000, len = 0x8000;
    uint32_t sar;

    fill(qts, src, len, 5);
    block(qts, 1, src, dst, len, 0);
    enable(qts, 1, true);
    qtest_clock_step(qts, 2000);
    qtest_writel(qts, CHEN0, 0x1010000);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==,
                    CH_SRC_SUSPENDED | CH_SUSPENDED);
    sar = qtest_readl(qts, CH(1) + SAR0);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, CH(1) + SAR0), ==, sar);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0x10001);
    qtest_writel(qts, CHEN0, 0x1000000);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    check_copy(qts, dst, src, len);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS) & DMA_TFR_DONE, ==,
                    DMA_TFR_DONE);

    qtest_writel(qts, CH(1) + INTCLEAR0, 0xffffffff);
    enable(qts, 1, true);
    qtest_clock_step(qts, 2000);
    enable(qts, 1, false);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==, CH_DISABLED);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);

    qtest_writel(qts, CH(1) + INTCLEAR0, 0xffffffff);
    block(qts, 1, src, dst, len, 0);
    enable(qts, 1, true);
    qtest_clock_step(qts, 2000);
    qtest_writel(qts, CHEN1, 0x101);
    g_assert_cmphex(qtest_readl(qts, CHEN1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==, CH_ABORTED);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    qtest_quit(qts);
}

/*
 * The DMA APM refusing channel 1's writes: DST_SLV_ERR ends the transfer,
 * and the ICM records the write with master 2's port, 6, and unit 0. A
 * peripheral address through master 2, which reaches only the memories,
 * is a decode error.
 */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_errors(void)
{
    QTestState *qts = start();
    uint32_t src = MEM, dst = MEM + 0x1000;

    fill(qts, src, 256, 6);
    qtest_writel(qts, DMA_PMS_GDMA_CH0_W, 0);
    block(qts, 1, src, dst, 256, 0);
    enable(qts, 1, true);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS), ==, DST_SLV_ERR);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW) & ICM_INT_SYS, ==,
                    ICM_INT_SYS);
    g_assert_cmphex(qtest_readl(qts, ICM_SYS_ADDR), ==, dst);
    g_assert_cmphex(qtest_readl(qts, ICM_SYS_INFO), ==, ICM_WR | 0x60);

    block(qts, 2, src, 0x50105000, 256, 0);
    enable(qts, 2, true);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, CH(2) + INTSTATUS), ==, DST_DEC_ERR);
    g_assert_cmphex(qtest_readl(qts, CHEN0), ==, 0);
    qtest_quit(qts);
}

/* The channel of higher CH_PRIOR has the bus first. */
/* [spec:nuos:req:emu.esp32p4.dma/test] */
static void test_priority(void)
{
    QTestState *qts = start();
    uint32_t src = MEM, len = 0x4000;
    uint32_t dst[2] = { MEM + 0x10000, MEM + 0x20000 };

    fill(qts, src, len, 7);
    for (int n = 1; n <= 2; n++) {
        qtest_memset(qts, dst[n - 1], 0, len);
        block(qts, n, src, dst[n - 1], len, 0);
        qtest_writel(qts, CH(n) + CH_CFG1, (n == 2 ? 3u : 0u) << 17);
    }
    qtest_writel(qts, CHEN0, 0x303);
    for (int i = 0; i < 100000; i++) {
        if (qtest_readl(qts, CH(2) + INTSTATUS) & DMA_TFR_DONE) {
            break;
        }
        qtest_clock_step(qts, 100);
    }
    g_assert_cmphex(qtest_readl(qts, CH(1) + INTSTATUS) & DMA_TFR_DONE, ==,
                    0);
    g_assert_cmphex(qtest_readl(qts, dst[0] + 0x1000), ==, 0);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    check_copy(qts, dst[0], src, len);
    check_copy(qts, dst[1], src, len);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/dwgdma/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/dwgdma/single-block", test_single_block);
    qtest_add_func("/esp32p4/dwgdma/linked-list", test_linked_list);
    qtest_add_func("/esp32p4/dwgdma/shadow", test_shadow);
    qtest_add_func("/esp32p4/dwgdma/reload", test_reload);
    qtest_add_func("/esp32p4/dwgdma/sw-handshake", test_sw_handshake);
    qtest_add_func("/esp32p4/dwgdma/suspend-disable-abort",
                   test_suspend_disable_abort);
    qtest_add_func("/esp32p4/dwgdma/errors", test_errors);
    qtest_add_func("/esp32p4/dwgdma/priority", test_priority);
    return g_test_run();
}
