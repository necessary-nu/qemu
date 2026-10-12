/*
 * QTest testcase for the ESP32-P4 H264 encoder and its dedicated DMA
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The registers' reset values and access types, then pictures coded the
 * way Espressif's esp_h264 driver codes them in GOP mode, with H264_CLK
 * on the 40 MHz XTAL_CLK so that each interrupt's time is a whole number
 * of its 25 ns cycles: a macroblock takes 980 of them. Buffers are in
 * L2MEM past the 128 KiB the L2 cache holds from reset.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define H264            0x50084000
#define DMA             0x500a7000

#define SYS_CTRL        (H264 + 0x000)
#define GOP_CONF        (H264 + 0x004)
#define SEQ(b, n)       (H264 + 0x008 + 0x4c * (b) + 4 * (n))
#define MB_RES          0
#define SYS_CONF        1
#define DECI_SCORE      2
#define RC_CONF0        4
#define RC_CONF1        5
#define DB_BYPASS       6
#define ROI_CONFIG      18
#define RC_STATUS0      (H264 + 0x0a0)
#define RC_STATUS1      (H264 + 0x0a4)
#define RC_STATUS2      (H264 + 0x0a8)
#define SLICE_REMAIN    (H264 + 0x0ac)
#define SLICE_LEN       (H264 + 0x0b0)
#define BS_THRESHOLD    (H264 + 0x0b4)
#define SLICE_BYTE0     (H264 + 0x0b8)
#define SLICE_BYTE1     (H264 + 0x0bc)
#define INT_RAW         (H264 + 0x0c0)
#define INT_ST          (H264 + 0x0c4)
#define INT_ENA         (H264 + 0x0c8)
#define INT_CLR         (H264 + 0x0cc)
#define CONF            (H264 + 0x0d0)
#define MV_MERGE        (H264 + 0x0d4)
#define SYS_STATUS      (H264 + 0x0dc)
#define FRAME_CODE_LEN  (H264 + 0x0e0)
#define DATE            (H264 + 0x0f0)
#define ORI_CONF(b)     (H264 + 0x0f4 + 4 * (b))

#define FRAME_START     (1u << 0)
#define DMA_MOVE_START  (1u << 1)
#define FRAME_MODE      (1u << 2)
#define SYS_RST_PULSE   (1u << 3)

#define DB_TMP_READY    (1u << 0)
#define REC_READY       (1u << 1)
#define FRAME_DONE      (1u << 2)
#define MOVE_DONE       (1u << 3)
#define BS_OVERFLOW     (1u << 4)

#define OUT(n, o)       (DMA + 0x100 * (n) + (o))
#define IN(n, o)        (DMA + 0x500 + 0x100 * (n) + (o))
#define IN5(o)          (DMA + 0xa00 + (o))
#define CONF0           0x00
#define C_INT_RAW       0x04
#define C_INT_ENA       0x08
#define C_INT_ST        0x0c
#define C_INT_CLR       0x10
#define LINK_CONF       0x1c
#define LINK_ADDR       0x20
#define STATE           0x24
#define OUT_EOF_DES     0x28
#define IN_SUC_EOF_DES  0x28
#define IN5_CONF1       0x04
#define IN5_CONF2       0x08
#define IN5_CONF3       0x0c
#define IN5_INT_RAW     0x10
#define IN5_STATE       0x28
#define MEM_ADDR(n)     (DMA + 0xb0c + 4 * (n) + ((n) >= 4 ? 4 : 0))
#define DMA_DATE        (DMA + 0xb3c)
#define COUNTER_RST     (DMA + 0xb50)
#define RX_COUNTER(n)   (DMA + 0xb54 + 4 * (n))
#define PBYTE           (DMA + 0xb64)

#define OUT_EOF_MODE    (1u << 1)
#define OUT_REORDER_EN  (1u << 16)
#define CH_RST          (1u << 24)
#define CH_CMD_DISABLE  (1u << 25)
#define BURST(n)        ((n) << 6)
#define OUTLINK_STOP    (1u << 20)
#define OUTLINK_START   (1u << 21)
#define OUTLINK_PARK    (1u << 23)
#define INLINK_STOP     (1u << 21)
#define INLINK_START    (1u << 22)
#define INLINK_PARK     (1u << 24)
#define IN_DONE         (1u << 0)
#define IN_SUC_EOF      (1u << 1)
#define IN_DSCR_EMPTY   (1u << 8)
#define OUT_DONE        (1u << 0)
#define OUT_DSCR_ERR    (1u << 2)

#define HP_SYS_CLKRST_SOC_CLK_CTRL1     0x500e6018
#define HP_SYS_CLKRST_PERI_CLK_CTRL26   0x500e60ac
#define HP_SYS_CLKRST_HP_RST_EN2        0x500e60c8
#define H264_SYS_CLK_EN     (1u << 31)
#define H264_CLK_SRC_PLL    (1u << 18)
#define H264_CLK_EN         (1u << 19)
#define RST_EN_H264         (1u << 24)

#define INTMTX_STATUS3  (0x500d6000 + 0x20c)
#define SRC_H264_REG    126

/* XTAL_CLK's cycle, and a macroblock's cycles. */
#define CYCLE_NS        25
#define MB_CYCLES       980

/* The memory the pictures use. */
#define MEM             0x4ff40000
#define DSC(n)          (MEM + 16 * (n))
#define D_YUV           0
#define D_BS            1
#define D_TMP_TX        2
#define D_TMP_RX        3
#define D_REF           4
#define D_DB12_TX       5
#define D_DB12_RX       6
#define D_DB4_TX        7
#define D_DB4_RX        8
#define D_MVM           9
#define SRC             (MEM + 0x1000)
#define BS              (MEM + 0x6000)
#define DB              (MEM + 0xb000)
#define REF             (MEM + 0x11000)
#define DBTMP           (MEM + 0x14000)
#define MVM             (MEM + 0x15000)

/* 96 x 80: 6 x 5 macroblocks. */
#define W               96
#define H               80
#define MBW             (W / 16)
#define MBH             (H / 16)

static void clocks_on(QTestState *qts)
{
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1,
                 qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1) |
                 H264_SYS_CLK_EN);
    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL26,
                 (qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL26) &
                  ~H264_CLK_SRC_PLL) | H264_CLK_EN);
}

/* Virtual time, as the last step left it. */
static int64_t now_ns;

static int64_t step(QTestState *qts, int64_t ns)
{
    now_ns = qtest_clock_step(qts, ns);
    return now_ns;
}

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4,strap-mode=0");

    clocks_on(qts);
    step(qts, 1);
    return qts;
}

/* Step virtual time to the next XTAL_CLK edge. */
static void align(QTestState *qts)
{
    if (now_ns % CYCLE_NS) {
        step(qts, CYCLE_NS - now_ns % CYCLE_NS);
    }
}

static void desc(QTestState *qts, int n, bool d2, bool mode1, uint32_t vb,
                 uint32_t hb, bool eof, uint32_t va, uint32_t ha,
                 uint32_t buf, uint32_t next)
{
    uint32_t w[4];

    w[0] = (vb & 0x3fff) | (hb & 0x3fff) << 14 | (uint32_t)d2 << 29 |
           (uint32_t)eof << 30 | 1u << 31;
    w[1] = (va & 0x3fff) | (ha & 0x7fff) << 14 | (uint32_t)mode1 << 29;
    w[2] = buf;
    w[3] = next;
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, DSC(n) + 4 * i, w[i]);
    }
}

/* A 1D descriptor of @size bytes, the length too for a TX one. */
static void desc1d(QTestState *qts, int n, uint32_t size, bool tx,
                   uint32_t buf, uint32_t next)
{
    desc(qts, n, false, false, size & 0x3fff, tx ? size & 0x3fff : 0, true,
         size >> 14, tx ? size >> 14 : 0, buf, next);
}

/* The driver's channel restart: stop, safe reset, start. */
static void out_start(QTestState *qts, int n)
{
    uint32_t c = qtest_readl(qts, OUT(n, CONF0));

    qtest_writel(qts, OUT(n, LINK_CONF), OUTLINK_STOP);
    if (n < 3) {
        qtest_writel(qts, OUT(n, CONF0), c | CH_CMD_DISABLE);
        g_assert_true(qtest_readl(qts, OUT(n, STATE)) & (1u << 24));
        qtest_writel(qts, OUT(n, CONF0), c | CH_CMD_DISABLE | CH_RST);
        qtest_writel(qts, OUT(n, CONF0), c | CH_CMD_DISABLE);
        qtest_writel(qts, OUT(n, CONF0), c);
    }
    qtest_writel(qts, OUT(n, LINK_CONF), OUTLINK_START);
}

static void in_start(QTestState *qts, int n)
{
    uint32_t c = qtest_readl(qts, IN(n, CONF0));

    qtest_writel(qts, IN(n, LINK_CONF), INLINK_STOP);
    qtest_writel(qts, IN(n, CONF0), c | CH_CMD_DISABLE);
    g_assert_true(qtest_readl(qts, IN(n, STATE)) & (1u << 23));
    qtest_writel(qts, IN(n, CONF0), c | CH_CMD_DISABLE | CH_RST);
    qtest_writel(qts, IN(n, CONF0), c | CH_CMD_DISABLE);
    qtest_writel(qts, IN(n, CONF0), c);
    qtest_writel(qts, IN(n, LINK_CONF), INLINK_START);
}

static void in5_reset(QTestState *qts)
{
    uint32_t c = qtest_readl(qts, IN5(CONF0));

    qtest_writel(qts, IN5(CONF0), c | CH_CMD_DISABLE);
    g_assert_true(qtest_readl(qts, IN5(IN5_STATE)) & (1u << 3));
    qtest_writel(qts, IN5(CONF0), c | CH_CMD_DISABLE | CH_RST);
    qtest_writel(qts, IN5(CONF0), c | CH_CMD_DISABLE);
    qtest_writel(qts, IN5(CONF0), c);
}

static void counter_reset(QTestState *qts, uint32_t bits)
{
    qtest_writel(qts, COUNTER_RST, bits);
    qtest_writel(qts, COUNTER_RST, 0);
}

/* A picture whose pattern moves 3 samples right and 1 down a picture. */
static void make_source(QTestState *qts, int k)
{
    uint8_t buf[W * 3 / 2];

    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W / 2; i++) {
            int x = 2 * i - 3 * k, y = j - k;
            uint8_t *p = buf + 3 * i;

            p[0] = (j & 1) ? 128 + ((y >> 2) & 15) : 128 + ((x >> 2) & 15);
            p[1] = 60 + ((x * 5 + y * 3) & 63) +
                   (((x >> 3) ^ (y >> 3)) & 1) * 60;
            p[2] = 60 + (((x + 1) * 5 + y * 3) & 63) +
                   ((((x + 1) >> 3) ^ (y >> 3)) & 1) * 60;
        }
        qtest_memwrite(qts, SRC + j * sizeof(buf), buf, sizeof(buf));
    }
}

/* The driver's encoder set-up, GOP mode, sequence A. */
static void enc_open(QTestState *qts, int qp, int gop)
{
    uint32_t size12 = 256 * MBW * (MBH - 1) + 384 * MBW;
    uint32_t size4 = 128 * MBW * (MBH - 1);

    qtest_writel(qts, CONF, 0xffffffff);
    qtest_writel(qts, SYS_CTRL, SYS_RST_PULSE);
    qtest_writel(qts, INT_ENA, 0);
    qtest_writel(qts, INT_CLR, 0xffffffff);
    qtest_writel(qts, GOP_CONF, gop << 1);
    qtest_writel(qts, SEQ(0, DECI_SCORE), 6 << 10 | 7);
    qtest_writel(qts, SEQ(0, MB_RES), MBH | MBW << 7);
    qtest_writel(qts, SEQ(0, SYS_CONF), (MBW >> 1) | 4 << 7);
    qtest_writel(qts, SEQ(0, RC_CONF0), qp);
    qtest_writel(qts, ORI_CONF(0), 4);
    for (int i = 0; i < 8; i++) {
        qtest_writel(qts, MEM_ADDR(i), i & 1 ? 0xffffffff : 0);
    }
    for (int n = 0; n < 5; n++) {
        qtest_writel(qts, OUT(n, CONF0), OUT_EOF_MODE | BURST(4) |
                     (n == 0 ? OUT_REORDER_EN : 0));
        qtest_writel(qts, IN(n, CONF0), BURST(4));
    }
    qtest_writel(qts, IN5(CONF0), BURST(4));
    qtest_writel(qts, PBYTE, 2);
    desc1d(qts, D_TMP_TX, 128 * MBW, true, DBTMP, DSC(D_TMP_TX));
    desc1d(qts, D_TMP_RX, 128 * MBW, false, DBTMP, DSC(D_TMP_RX));
    qtest_writel(qts, OUT(2, LINK_ADDR), DSC(D_TMP_TX));
    qtest_writel(qts, IN(2, LINK_ADDR), DSC(D_TMP_RX));
    desc(qts, D_REF, true, true, 3, 256, true, 3, 256 * MBW, REF,
         DSC(D_REF));
    qtest_writel(qts, OUT(1, LINK_ADDR), DSC(D_REF));
    qtest_writel(qts, IN5(IN5_CONF1), REF);
    qtest_writel(qts, IN5(IN5_CONF2), 256 * MBW | (128 * MBW) << 16);
    qtest_writel(qts, IN5(IN5_CONF3), 256 | 128 << 14);
    desc1d(qts, D_DB12_TX, size12, true, DB, DSC(D_DB12_TX));
    desc1d(qts, D_DB12_RX, size12, false, DB, DSC(D_DB12_RX));
    desc1d(qts, D_DB4_TX, size4, true, DB + size12, DSC(D_DB4_TX));
    desc1d(qts, D_DB4_RX, size4, false, DB + size12, DSC(D_DB4_RX));
    qtest_writel(qts, OUT(3, LINK_ADDR), DSC(D_DB12_TX));
    qtest_writel(qts, IN(0, LINK_ADDR), DSC(D_DB12_RX));
    qtest_writel(qts, OUT(4, LINK_ADDR), DSC(D_DB4_TX));
    qtest_writel(qts, IN(1, LINK_ADDR), DSC(D_DB4_RX));
    qtest_writel(qts, INT_ENA, 0x1f);
}

/* Start picture @k: its source, descriptors and channels, as the driver. */
static void enc_start(QTestState *qts, int k, uint32_t bs_size)
{
    bool intra = k == 0;

    make_source(qts, k);
    desc(qts, D_YUV, true, true, 16, 64, false, H, W, SRC, 0);
    qtest_writel(qts, OUT(0, LINK_ADDR), DSC(D_YUV));
    desc1d(qts, D_BS, bs_size, false, BS, 0);
    qtest_writel(qts, IN(4, LINK_ADDR), DSC(D_BS));
    /* A slice header's tail: two bytes and three bits. */
    qtest_writel(qts, SLICE_BYTE1, 0x65b80000);
    qtest_writel(qts, SLICE_BYTE0, 0);
    qtest_writel(qts, SLICE_LEN, 2);
    qtest_writel(qts, SLICE_REMAIN, 0xa0 << 3 | 3);
    for (int n = 0; n < 5; n++) {
        qtest_writel(qts, OUT(n, C_INT_CLR), 0xffffffff);
        qtest_writel(qts, IN(n, C_INT_CLR), 0xffffffff);
    }
    qtest_writel(qts, INT_CLR, 0xffffffff);
    counter_reset(qts, 1u << 2);
    if (intra) {
        counter_reset(qts, 1u << 0 | 1u << 1 | 1u << 3);
        out_start(qts, 0);
        in_start(qts, 0);
        in_start(qts, 1);
        in_start(qts, 4);
        out_start(qts, 2);
        in_start(qts, 2);
        qtest_writel(qts, SYS_CTRL, SYS_RST_PULSE);
    } else {
        out_start(qts, 0);
        in_start(qts, 4);
        in_start(qts, 0);
        in_start(qts, 1);
        out_start(qts, 2);
        in_start(qts, 2);
    }
    qtest_writel(qts, SYS_CTRL, FRAME_START);
}

/*
 * Service the encoder's interrupts as the driver's ISR does until
 * FRAME_DONE, stepping time 1 us at a time; the time it came.
 */
static int64_t enc_wait(QTestState *qts, int k)
{
    for (int i = 0; i < 100000; i++) {
        uint32_t st = qtest_readl(qts, INT_ST);

        if (st & DB_TMP_READY) {
            qtest_writel(qts, INT_CLR, DB_TMP_READY);
        } else if (st & REC_READY) {
            qtest_writel(qts, INT_CLR, REC_READY);
            if (k == 0) {
                qtest_writel(qts, OUT(3, LINK_CONF), OUTLINK_STOP);
                qtest_writel(qts, OUT(4, LINK_CONF), OUTLINK_STOP);
                in5_reset(qts);
                qtest_writel(qts, OUT(3, LINK_CONF), OUTLINK_START);
                qtest_writel(qts, OUT(4, LINK_CONF), OUTLINK_START);
                qtest_writel(qts, SYS_CTRL, DMA_MOVE_START);
            }
        } else if (st & MOVE_DONE) {
            qtest_writel(qts, INT_CLR, MOVE_DONE);
            if (k == 0) {
                out_start(qts, 1);
            }
        } else if (st & FRAME_DONE) {
            qtest_writel(qts, INT_CLR, FRAME_DONE);
            return now_ns;
        } else {
            step(qts, 1000);
        }
    }
    g_assert_not_reached();
}

/* [spec:nuos:req:emu.esp32p4.h264/test] */
static void test_reset_values(void)
{
    QTestState *qts = qtest_init("-M esp32p4,strap-mode=0");

    /* Behind H264_SYS_CLK, off from reset: no write lands. */
    qtest_writel(qts, GOP_CONF, 0x1fe);
    g_assert_cmphex(qtest_readl(qts, GOP_CONF), ==, 0);
    clocks_on(qts);

    g_assert_cmphex(qtest_readl(qts, SYS_CTRL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SEQ(0, SYS_CONF)), ==, 0x203);
    g_assert_cmphex(qtest_readl(qts, SEQ(1, SYS_CONF)), ==, 0x203);
    g_assert_cmphex(qtest_readl(qts, BS_THRESHOLD), ==, 0x30);
    g_assert_cmphex(qtest_readl(qts, DATE), ==, 0x2412300);
    g_assert_cmphex(qtest_readl(qts, ORI_CONF(0)), ==, 4);
    g_assert_cmphex(qtest_readl(qts, ORI_CONF(1)), ==, 4);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);

    for (int n = 0; n < 5; n++) {
        g_assert_cmphex(qtest_readl(qts, OUT(n, CONF0)), ==, OUT_EOF_MODE);
        g_assert_cmphex(qtest_readl(qts, OUT(n, LINK_CONF)), ==,
                        OUTLINK_PARK);
        g_assert_cmphex(qtest_readl(qts, OUT(n, 0x14)), ==, 0x20082);
        g_assert_cmphex(qtest_readl(qts, OUT(n, 0x3c)), ==,
                        n == 1 || n == 2 ? 0x41 : 0x11);
        g_assert_cmphex(qtest_readl(qts, OUT(n, 0x68)), ==, 0x4);
        g_assert_cmphex(qtest_readl(qts, OUT(n, STATE)), ==,
                        n < 3 ? 1u << 24 : 0);
        g_assert_cmphex(qtest_readl(qts, IN(n, CONF0)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, IN(n, LINK_CONF)), ==,
                        INLINK_PARK | 1u << 20);
        g_assert_cmphex(qtest_readl(qts, IN(n, 0x18)), ==, 0x400);
        g_assert_cmphex(qtest_readl(qts, IN(n, 0x40)), ==,
                        n == 2 || n == 3 ? 0x41 : 0x51);
        g_assert_cmphex(qtest_readl(qts, IN(n, STATE)), ==, 1u << 23);
        g_assert_cmphex(qtest_readl(qts, IN(n, 0x84)), ==, 7);
    }
    g_assert_cmphex(qtest_readl(qts, OUT(0, 0x44)), ==, 0x20);
    g_assert_cmphex(qtest_readl(qts, OUT(0, 0x78)), ==, 0xff);
    g_assert_cmphex(qtest_readl(qts, OUT(3, 0x78)), ==, 0x3f);
    g_assert_cmphex(qtest_readl(qts, IN5(CONF0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IN5(IN5_CONF2)), ==, 0x3c007800);
    g_assert_cmphex(qtest_readl(qts, IN5(IN5_CONF3)), ==, 0x200100);
    g_assert_cmphex(qtest_readl(qts, IN5(IN5_STATE)), ==, 0x8);
    g_assert_cmphex(qtest_readl(qts, IN5(0x40)), ==, 0x41);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, MEM_ADDR(i)), ==,
                        i & 1 ? 0x8fffffff : 0x30100000);
    }
    g_assert_cmphex(qtest_readl(qts, DMA_DATE), ==, 0x20230403);
    g_assert_cmphex(qtest_readl(qts, PBYTE), ==, 2);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.h264/test] */
static void test_access(void)
{
    QTestState *qts = start();

    /* The WT bits read 0; FRAME_MODE holds. */
    qtest_writel(qts, SYS_CTRL, FRAME_MODE);
    g_assert_cmphex(qtest_readl(qts, SYS_CTRL), ==, FRAME_MODE);
    qtest_writel(qts, GOP_CONF, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GOP_CONF), ==, 0x1ff);
    qtest_writel(qts, SEQ(1, MB_RES), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SEQ(1, MB_RES)), ==, 0x3fff);
    qtest_writel(qts, SEQ(0, RC_CONF1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SEQ(0, RC_CONF1)), ==, 0x7fffffff);
    qtest_writel(qts, SEQ(0, ROI_CONFIG), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SEQ(0, ROI_CONFIG)), ==, 3);
    qtest_writel(qts, SLICE_REMAIN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SLICE_REMAIN), ==, 0x7ff);
    qtest_writel(qts, SLICE_LEN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SLICE_LEN), ==, 0xf);
    /* MB_VALID_NUM is read-only. */
    qtest_writel(qts, MV_MERGE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, MV_MERGE), ==, 0x1f);
    /* Status registers are read-only. */
    qtest_writel(qts, RC_STATUS0, 0xffffffff);
    qtest_writel(qts, SYS_STATUS, 0xffffffff);
    qtest_writel(qts, FRAME_CODE_LEN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RC_STATUS0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SYS_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, FRAME_CODE_LEN), ==, 0);
    qtest_writel(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0x1f);
    g_assert_cmphex(qtest_readl(qts, INT_CLR), ==, 0);

    /* H264_DMA: masks, write-1-to-clear, the counters read-only. */
    qtest_writel(qts, OUT(0, CONF0), 0xffffffff & ~CH_RST);
    g_assert_cmphex(qtest_readl(qts, OUT(0, CONF0)), ==, 0x060111df);
    qtest_writel(qts, OUT(3, CONF0), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OUT(3, CONF0)), ==, 0x040011df);
    qtest_writel(qts, IN(1, CONF0), 0xffffffff & ~CH_RST);
    g_assert_cmphex(qtest_readl(qts, IN(1, CONF0)), ==, 0x060011dc);
    qtest_writel(qts, IN5(CONF0), 0xffffffff & ~CH_RST);
    g_assert_cmphex(qtest_readl(qts, IN5(CONF0)), ==, 0x020011c8);
    qtest_writel(qts, OUT(1, C_INT_ENA), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OUT(1, C_INT_ENA)), ==, 0x1ff);
    qtest_writel(qts, IN(1, C_INT_ENA), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, IN(1, C_INT_ENA)), ==, 0x3ff);
    qtest_writel(qts, IN5(0x14), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, IN5(0x14)), ==, 0x1f);
    qtest_writel(qts, RX_COUNTER(0), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RX_COUNTER(0)), ==, 0);
    qtest_writel(qts, PBYTE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PBYTE), ==, 0xf);
    qtest_quit(qts);
}

/*
 * A descriptor outside the memory ranges: TX channel 2's check fails as it
 * fetches it for the second row of a 1 x 2 picture, which then waits.
 */
/* [spec:nuos:req:emu.esp32p4.h264/test] */
static void test_descriptor_error(void)
{
    QTestState *qts = start();

    qtest_writel(qts, SEQ(0, MB_RES), 2 | 1 << 7);
    qtest_writel(qts, SEQ(0, SYS_CONF), 4 << 7);
    desc(qts, D_YUV, true, true, 16, 16, false, 32, 16, SRC, 0);
    qtest_writel(qts, OUT(0, LINK_ADDR), DSC(D_YUV));
    desc1d(qts, D_BS, 4096, false, BS, 0);
    qtest_writel(qts, IN(4, LINK_ADDR), DSC(D_BS));
    desc1d(qts, D_DB12_RX, 4096, false, DB, 0);
    qtest_writel(qts, IN(0, LINK_ADDR), DSC(D_DB12_RX));
    desc1d(qts, D_DB4_RX, 4096, false, DB + 0x1000, 0);
    qtest_writel(qts, IN(1, LINK_ADDR), DSC(D_DB4_RX));
    desc1d(qts, D_TMP_RX, 128, false, DBTMP, DSC(D_TMP_RX));
    qtest_writel(qts, IN(2, LINK_ADDR), DSC(D_TMP_RX));
    qtest_writel(qts, OUT(2, LINK_ADDR), 0x10000000);
    qtest_writel(qts, OUT(0, LINK_CONF), OUTLINK_START);
    qtest_writel(qts, OUT(2, LINK_CONF), OUTLINK_START);
    for (int n = 0; n < 5; n++) {
        if (n != 3) {
            qtest_writel(qts, IN(n, LINK_CONF), INLINK_START);
        }
    }
    qtest_writel(qts, SYS_CTRL, FRAME_START);
    step(qts, 200 * 1000);
    g_assert_cmphex(qtest_readl(qts, OUT(2, C_INT_RAW)) & OUT_DSCR_ERR, ==,
                    OUT_DSCR_ERR);
    g_assert_cmphex(qtest_readl(qts, OUT(2, LINK_CONF)) & OUTLINK_PARK, ==,
                    OUTLINK_PARK);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & FRAME_DONE, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.h264/test] */
static void test_refused(void)
{
    QTestState *qts = start();

    /* Wider than 120 macroblocks: the picture does not start. */
    qtest_writel(qts, SEQ(0, MB_RES), 1 | 121 << 7);
    qtest_writel(qts, INT_ENA, 0x1f);
    qtest_writel(qts, SYS_CTRL, FRAME_START);
    step(qts, 100 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SYS_STATUS), ==, 0);
    qtest_quit(qts);
}

/*
 * An I picture and two P pictures coded as the driver codes them: each
 * interrupt at its time, the bitstream starting with the slice header's
 * tail from the registers, and the rate control's statistics.
 */
/* [spec:nuos:req:emu.esp32p4.h264/test] */
static void test_gop(void)
{
    QTestState *qts = start();
    int64_t t0, t;
    uint8_t head[4];
    uint32_t len, dw0, dw1, frame_bytes[3];

    enc_open(qts, 26, 30);
    align(qts);
    t0 = now_ns;
    enc_start(qts, 0, 0x5000);
    g_assert_cmphex(qtest_readl(qts, SYS_STATUS), ==, 1u << 10);
    /* DB_TMP_READY when the first row's third macroblock is done. */
    t = t0 + 3 * MB_CYCLES * CYCLE_NS;
    step(qts, t - 1 - now_ns);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, DB_TMP_READY);
    g_assert_cmpuint((qtest_readl(qts, INTMTX_STATUS3) >>
                      (SRC_H264_REG - 96)) & 1, ==, 1);
    qtest_writel(qts, INT_CLR, DB_TMP_READY);
    /* REC_READY and FRAME_DONE when the fifth, last, row is. */
    t = t0 + MBW * MBH * MB_CYCLES * CYCLE_NS;
    step(qts, t - 1 - now_ns);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, REC_READY | FRAME_DONE);
    /* The bitstream: the header's tail, then the slice data. */
    len = qtest_readl(qts, FRAME_CODE_LEN);
    g_assert_cmpuint(len, >, 100);
    qtest_memread(qts, BS, head, sizeof(head));
    g_assert_cmphex(head[0], ==, 0x65);
    g_assert_cmphex(head[1], ==, 0xb8);
    g_assert_cmphex(head[2] & 0xe0, ==, 0xa0);
    dw0 = qtest_readl(qts, DSC(D_BS));
    dw1 = qtest_readl(qts, DSC(D_BS) + 4);
    g_assert_cmpuint(((dw0 >> 14) & 0x3fff) | ((dw1 >> 14) & 0x7fff) << 14,
                     ==, len);
    g_assert_cmphex(dw0 & (1u << 31), ==, 0);
    g_assert_cmphex(dw0 & (1u << 30), ==, 1u << 30);
    g_assert_cmphex(qtest_readl(qts, IN(4, C_INT_RAW)) &
                    (IN_DONE | IN_SUC_EOF), ==, IN_DONE | IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, IN(4, IN_SUC_EOF_DES)), ==, DSC(D_BS));
    g_assert_cmpuint(qtest_readl(qts, RC_STATUS2), ==, 26 * MBW * MBH);
    frame_bytes[0] = len;
    /* The driver's REC_READY handling: DMA_MOVE_START, two rows' time. */
    t0 = now_ns;
    qtest_writel(qts, INT_CLR, REC_READY | FRAME_DONE);
    qtest_writel(qts, OUT(3, LINK_CONF), OUTLINK_STOP);
    qtest_writel(qts, OUT(4, LINK_CONF), OUTLINK_STOP);
    in5_reset(qts);
    qtest_writel(qts, OUT(3, LINK_CONF), OUTLINK_START);
    qtest_writel(qts, OUT(4, LINK_CONF), OUTLINK_START);
    qtest_writel(qts, SYS_CTRL, DMA_MOVE_START);
    t = t0 + 2 * 48 * MBW * CYCLE_NS;
    step(qts, t - 1 - now_ns);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, MOVE_DONE);
    g_assert_cmphex(qtest_readl(qts, IN5(IN5_INT_RAW)) & IN_DONE, ==,
                    IN_DONE);
    qtest_writel(qts, INT_CLR, MOVE_DONE);
    out_start(qts, 1);

    for (int k = 1; k < 3; k++) {
        align(qts);
        t0 = now_ns;
        enc_start(qts, k, 0x5000);
        g_assert_cmphex(qtest_readl(qts, SYS_STATUS), ==, k);
        t = enc_wait(qts, k);
        g_assert_cmpint(t - t0, >=, MBW * MBH * MB_CYCLES * CYCLE_NS);
        g_assert_cmpint(t - t0, <, MBW * MBH * MB_CYCLES * CYCLE_NS + 1000);
        frame_bytes[k] = qtest_readl(qts, FRAME_CODE_LEN);
        /* A P picture of the moved pattern costs far less than the I. */
        g_assert_cmpuint(frame_bytes[k], <, frame_bytes[0] / 2);
        g_assert_cmpuint(qtest_readl(qts, RC_STATUS1), >, 0);
        g_assert_cmpuint(qtest_readl(qts, RC_STATUS1), <,
                         8 * frame_bytes[k]);
    }
    qtest_quit(qts);
}

/*
 * A bitstream buffer too small: IN_DSCR_EMPTY and BS_BUFFER_OVERFLOW,
 * the picture still completing, FRAME_CODE_LENGTH the bytes coded.
 */
/* [spec:nuos:req:emu.esp32p4.h264/test] */
static void test_overflow(void)
{
    QTestState *qts = start();
    uint32_t dw0;

    enc_open(qts, 26, 30);
    enc_start(qts, 0, 64);
    enc_wait(qts, 0);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & BS_OVERFLOW, ==, BS_OVERFLOW);
    g_assert_cmphex(qtest_readl(qts, IN(4, C_INT_RAW)) &
                    (IN_DONE | IN_SUC_EOF | IN_DSCR_EMPTY), ==,
                    IN_DONE | IN_DSCR_EMPTY);
    g_assert_cmpuint(qtest_readl(qts, FRAME_CODE_LEN), >, 64);
    dw0 = qtest_readl(qts, DSC(D_BS));
    g_assert_cmpuint((dw0 >> 14) & 0x3fff, ==, 64);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32p4/h264/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/h264/access", test_access);
    qtest_add_func("/esp32p4/h264/descriptor-error", test_descriptor_error);
    qtest_add_func("/esp32p4/h264/refused", test_refused);
    qtest_add_func("/esp32p4/h264/gop", test_gop);
    qtest_add_func("/esp32p4/h264/overflow", test_overflow);
    return g_test_run();
}
