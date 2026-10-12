/*
 * QTest testcase for the ESP32-P4 image signal processor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Small synthetic RAW8 frames go in through the CSI host's "packet" hook,
 * through CSI_CM (bypassed) into the ISP, and come out of the CSI
 * bridge's port, where the test reads them; each stage of the pipeline is
 * checked on its own against this file's reference, a full-frame
 * implementation of the arithmetic hw/misc/esp32p4_isp_pipe.c states,
 * and so are the statistics, the interrupts, the registers and LUTs,
 * the output's timing, the bypass, the VDMA input and the DVP input.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qobject/qdict.h"

#define NS_PER_US           1000ull

#define CLKRST              0x500e6000
#define SOC_CLK_CTRL0       (CLKRST + 0x14)
#define SOC_CLK_CTRL1       (CLKRST + 0x18)
#define PERI_CLK_CTRL03     (CLKRST + 0x3c)
#define PERI_CLK_CTRL25     (CLKRST + 0xa8)
#define PERI_CLK_CTRL26     (CLKRST + 0xac)
#define HP_RST_EN0          (CLKRST + 0xc0)
#define GDMA_CPU_CLK_EN     (1u << 13)
#define GDMA_SYS_CLK_EN     (1u << 5)
#define CSI_BRG_SYS_CLK_EN  (1u << 10)
#define CSI_HOST_SYS_CLK_EN (1u << 11)

#define INTMTX_STATUS(n)    (0x500d6000 + 0x200 + 4 * ((n) / 32))
#define SRC_ISP             100

#define CSI                 0x5009f000
#define C_N_LANES           (CSI + 0x004)
#define C_CSI2_RESETN       (CSI + 0x008)
#define C_PHY_SHUTDOWNZ     (CSI + 0x040)
#define C_DPHY_RSTZ         (CSI + 0x044)
#define C_PHY_TEST_CTRL0    (CSI + 0x050)
#define C_PHY_TEST_CTRL1    (CSI + 0x054)
#define C_VC_EXTENSION      (CSI + 0x0c8)
#define CB                  0x5009f800
#define CB_CSI_EN           (CB + 0x04)
#define CB_BUF_FLOW_CTL     (CB + 0x0c)
#define CB_DATA_TYPE_CFG    (CB + 0x10)
#define CB_FRAME_CFG        (CB + 0x14)
#define CB_INT_RAW          (CB + 0x1c)
#define CB_HOST_CM_CTRL     (CB + 0x48)
#define CSI_MEM             0x50104000

#define ISP                 0x500a1000
#define I_VER_DATE          (ISP + 0x000)
#define I_CNTL              (ISP + 0x008)
#define I_HSYNC_CNT         (ISP + 0x00c)
#define I_FRAME_CFG         (ISP + 0x010)
#define I_CCM_COEF0         (ISP + 0x014)
#define I_CCM_COEF1         (ISP + 0x018)
#define I_CCM_COEF3         (ISP + 0x01c)
#define I_CCM_COEF4         (ISP + 0x020)
#define I_CCM_COEF5         (ISP + 0x024)
#define I_BF_MATRIX_CTRL    (ISP + 0x028)
#define I_BF_SIGMA          (ISP + 0x02c)
#define I_BF_GAU0           (ISP + 0x030)
#define I_BF_GAU1           (ISP + 0x034)
#define I_DPC_CTRL          (ISP + 0x038)
#define I_DPC_CONF          (ISP + 0x03c)
#define I_DPC_DEADPIX_CNT   (ISP + 0x044)
#define I_LUT_CMD           (ISP + 0x048)
#define I_LUT_WDATA         (ISP + 0x04c)
#define I_LUT_RDATA         (ISP + 0x050)
#define I_LSC_TABLESIZE     (ISP + 0x054)
#define I_DEMOSAIC_GRAD     (ISP + 0x05c)
#define I_INT_RAW           (ISP + 0x064)
#define I_INT_ST            (ISP + 0x068)
#define I_INT_ENA           (ISP + 0x06c)
#define I_INT_CLR           (ISP + 0x070)
#define I_GAMMA_CTRL        (ISP + 0x074)
#define I_GAMMA_Y(ch, i)    (ISP + 0x078 + 0x10 * (ch) + 4 * (i))
#define I_GAMMA_X(ch, i)    (ISP + 0x0a8 + 8 * (ch) + 4 * (i))
#define I_AE_CTRL           (ISP + 0x0c0)
#define I_AE_MONITOR        (ISP + 0x0c4)
#define I_AE_BX             (ISP + 0x0c8)
#define I_AE_BY             (ISP + 0x0cc)
#define I_AE_WINPIXNUM      (ISP + 0x0d0)
#define I_AE_RECIP          (ISP + 0x0d4)
#define I_AE_MEAN(i)        (ISP + 0x0d8 + 4 * (i))
#define I_SHARP_CTRL0       (ISP + 0x0f4)
#define I_SHARP_FILTER(i)   (ISP + 0x0f8 + 4 * (i))
#define I_SHARP_CTRL1       (ISP + 0x108)
#define I_DMA_CNTL          (ISP + 0x10c)
#define I_DMA_RAW_DATA      (ISP + 0x110)
#define I_CAM_CNTL          (ISP + 0x114)
#define I_CAM_CONF          (ISP + 0x118)
#define I_AF_CTRL0          (ISP + 0x11c)
#define I_AF_THRESHOLD      (ISP + 0x130)
#define I_AF_HSCALE(k)      (ISP + 0x134 + 8 * (k))
#define I_AF_VSCALE(k)      (ISP + 0x138 + 8 * (k))
#define I_AF_SUM(k)         (ISP + 0x14c + 4 * (k))
#define I_AF_LUM(k)         (ISP + 0x158 + 4 * (k))
#define I_AWB_MODE          (ISP + 0x164)
#define I_AWB_HSCALE        (ISP + 0x168)
#define I_AWB_VSCALE        (ISP + 0x16c)
#define I_AWB_TH_LUM        (ISP + 0x170)
#define I_AWB_TH_RG         (ISP + 0x174)
#define I_AWB_TH_BG         (ISP + 0x178)
#define I_AWB0_WHITE_CNT    (ISP + 0x17c)
#define I_AWB0_ACC(k)       (ISP + 0x180 + 4 * (k))
#define I_COLOR_CTRL        (ISP + 0x18c)
#define I_BLC_VALUE         (ISP + 0x190)
#define I_BLC_CTRL0         (ISP + 0x194)
#define I_BLC_CTRL1         (ISP + 0x198)
#define I_BLC_CTRL2         (ISP + 0x19c)
#define I_BLC_MEAN          (ISP + 0x1a0)
#define I_HIST_MODE         (ISP + 0x1a4)
#define I_HIST_COEFF        (ISP + 0x1a8)
#define I_HIST_OFFS         (ISP + 0x1ac)
#define I_HIST_SIZE         (ISP + 0x1b0)
#define I_HIST_SEG(i)       (ISP + 0x1b4 + 4 * (i))
#define I_HIST_WEIGHT(i)    (ISP + 0x1c4 + 4 * (i))
#define I_HIST_BIN(i)       (ISP + 0x1e0 + 4 * (i))
#define I_YUV_FORMAT        (ISP + 0x234)
#define I_CROP_CTRL         (ISP + 0x244)
#define I_CROP_Y            (ISP + 0x248)
#define I_CROP_X            (ISP + 0x24c)
#define I_CROP_ERR_ST       (ISP + 0x250)
#define I_WBG(k)            (ISP + 0x254 + 4 * (k))
#define I_COLOR_HUE_CTRL    (ISP + 0x260)
#define I_AWB_BX            (ISP + 0x264)
#define I_AWB_BY            (ISP + 0x268)
#define I_STATE             (ISP + 0x26c)
#define I_SHADOW_REG_CTRL   (ISP + 0x270)

/* CNTL */
#define MIPI_DATA_EN        (1u << 0)
#define ISP_EN              (1u << 1)
#define BLC_EN              (1u << 2)
#define DPC_EN              (1u << 3)
#define BF_EN               (1u << 4)
#define LSC_EN              (1u << 5)
#define DEMOSAIC_EN         (1u << 6)
#define MEDIAN_EN           (1u << 7)
#define CCM_EN              (1u << 8)
#define GAMMA_EN            (1u << 9)
#define RGB2YUV_EN          (1u << 10)
#define SHARP_EN            (1u << 11)
#define COLOR_EN            (1u << 12)
#define YUV2RGB_EN          (1u << 13)
#define AE_EN               (1u << 14)
#define AF_EN               (1u << 15)
#define AWB_EN              (1u << 16)
#define HIST_EN             (1u << 17)
#define CROP_EN             (1u << 18)
#define WBG_EN              (1u << 19)
#define BYTE_ENDIAN_ORDER   (1u << 24)
#define OUT_RAW8            0
#define OUT_YUV422          1
#define OUT_RGB888          2
#define OUT_YUV420          3
#define OUT_RGB565          4
#define CNTL_OUT(t)         ((uint32_t)(t) << 29)

/* INT_RAW */
#define INT_DATA_TYPE_ERR   (1u << 0)
#define INT_BUF_FULL        (1u << 2)
#define INT_HVNUM_ERR       (1u << 3)
#define INT_DT_SETTING_ERR  (1u << 4)
#define INT_HNUM_UNMATCH    (1u << 5)
#define INT_DPC_CHECK_DONE  (1u << 6)
#define INT_GAMMA_XCOORD    (1u << 7)
#define INT_AE_MONITOR      (1u << 8)
#define INT_AE_DONE         (1u << 9)
#define INT_AF_FDONE        (1u << 10)
#define INT_AF_ENV          (1u << 11)
#define INT_AWB_FDONE       (1u << 12)
#define INT_HIST_FDONE      (1u << 13)
#define INT_FRAME           (1u << 14)
#define INT_BLC_FRAME       (1u << 15)
#define INT_LSC_FRAME       (1u << 16)
#define INT_DPC_FRAME       (1u << 17)
#define INT_BF_FRAME        (1u << 18)
#define INT_DEMOSAIC_FRAME  (1u << 19)
#define INT_CCM_FRAME       (1u << 21)
#define INT_GAMMA_FRAME     (1u << 22)
#define INT_RGB2YUV_FRAME   (1u << 23)
#define INT_SHARP_FRAME     (1u << 24)
#define INT_COLOR_FRAME     (1u << 25)
#define INT_YUV2RGB_FRAME   (1u << 26)
#define INT_TAIL_FRAME      (1u << 27)
#define INT_HEADER_FRAME    (1u << 28)
#define INT_CROP_FRAME      (1u << 29)
#define INT_WBG_FRAME       (1u << 30)
#define INT_CROP_ERR        (1u << 31)

/* ---- The machine ---- */

static int64_t vt;

static void step(QTestState *qts, int64_t ns)
{
    vt = qtest_clock_step(qts, ns);
}

static uint8_t ecc(uint32_t h)
{
    static const uint32_t parity[6] = {
        0xf12cb7, 0xf2555b, 0x749a6d, 0xb8e38e, 0xdf03f0, 0xeffc00,
    };
    uint8_t e = 0;

    for (int i = 0; i < 6; i++) {
        e |= (__builtin_popcount(h & parity[i]) & 1) << i;
    }
    return e;
}

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xffff;

    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
        }
    }
    return crc;
}

/* A CSI-2 packet into the CSI host, as it comes off the lanes. */
static void csi_send(QTestState *qts, uint8_t dt, uint16_t wc,
                     const uint8_t *pld)
{
    uint32_t h = dt | (uint32_t)wc << 8, n = 4 + (pld ? wc + 2 : 0);
    g_autofree uint8_t *b = g_malloc(n);
    g_autofree char *hex = g_malloc(2 * n + 1);
    QDict *r;

    b[0] = h;
    b[1] = h >> 8;
    b[2] = h >> 16;
    b[3] = ecc(h);
    if (pld) {
        uint16_t c = crc16(pld, wc);

        memcpy(b + 4, pld, wc);
        b[4 + wc] = c;
        b[5 + wc] = c >> 8;
    }
    for (uint32_t i = 0; i < n; i++) {
        sprintf(hex + 2 * i, "%02x", b[i]);
    }
    r = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': {'path': "
                  "'/machine/csi-host', 'property': 'packet', 'value': %s}}",
                  hex);
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
}

static void csi_phy_write(QTestState *qts, uint8_t code, uint8_t data)
{
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, (1u << 16) | code);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 2);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, data);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 2);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
}

/*
 * The machine with the CSI host up, the bridge taking every RAW and RGB
 * data type through a bypassed CSI_CM, and ISP_CLK at 240 MHz / 6, 40
 * MHz, a cycle 25 ns: slow enough for the bridge, at the boot's bus clock,
 * to take RGB888 rows as fast as the pipeline gives them.
 */
static QTestState *machine(const char *extra)
{
    QTestState *qts = qtest_initf("-M esp32p4,flash-size=16M,strap-mode=0 "
                                  "%s", extra ? extra : "");
    uint32_t v;

    step(qts, 1);
    qtest_writel(qts, SOC_CLK_CTRL0,
                 qtest_readl(qts, SOC_CLK_CTRL0) | GDMA_CPU_CLK_EN);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | GDMA_SYS_CLK_EN |
                 CSI_BRG_SYS_CLK_EN | CSI_HOST_SYS_CLK_EN);
    v = qtest_readl(qts, PERI_CLK_CTRL25);
    qtest_writel(qts, PERI_CLK_CTRL25, (v & ~(3u << 23)) | 1u << 23 |
                 1u << 25);
    v = qtest_readl(qts, PERI_CLK_CTRL26);
    qtest_writel(qts, PERI_CLK_CTRL26, (v & ~0xffu) | 5);

    v = qtest_readl(qts, PERI_CLK_CTRL03);
    qtest_writel(qts, PERI_CLK_CTRL03, (v & ~(3u << 2)) | 1u << 4);
    qtest_writel(qts, C_PHY_SHUTDOWNZ, 0);
    qtest_writel(qts, C_DPHY_RSTZ, 0);
    qtest_writel(qts, C_CSI2_RESETN, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, 1u << 16);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 1);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    csi_phy_write(qts, 0x44, 0);
    qtest_writel(qts, C_PHY_SHUTDOWNZ, 1);
    qtest_writel(qts, C_DPHY_RSTZ, 1);
    qtest_writel(qts, C_CSI2_RESETN, 1);
    qtest_writel(qts, C_N_LANES, 1);
    qtest_writel(qts, C_VC_EXTENSION, 1);
    step(qts, 50 * NS_PER_US);

    qtest_writel(qts, CB_HOST_CM_CTRL, 1 | 2);
    qtest_writel(qts, CB_DATA_TYPE_CFG, 0x2f12);
    qtest_writel(qts, CB_CSI_EN, 1);
    return qts;
}

/* Pop @n bytes of the bridge's buffer. */
static void cb_read(QTestState *qts, uint8_t *out, uint32_t n)
{
    for (uint32_t i = 0; i < n; i += 4) {
        uint32_t w = qtest_readl(qts, CSI_MEM);

        for (int k = 0; k < 4 && i + k < n; k++) {
            out[i + k] = w >> (8 * k);
        }
    }
}

/* ---- The reference ---- */

#define MAXW 32
#define MAXH 16

/* The configuration of a frame, as the registers set it. */
typedef struct Cfg {
    int w;
    int h;
    int bayer;
    uint32_t cntl;
    int out;
    uint8_t blc[4];
    uint8_t stretch;
    uint32_t dpc_ctrl;
    uint32_t dpc_conf;
    int nsta;
    int sta[8][2];
    int sigma;
    uint8_t gau[9];
    uint16_t lsc[4][4];
    int grad;
    int wbg[3];
    int ccm[9];
    uint8_t gx[3][16];
    uint8_t gy[3][16];
    bool bt709;
    bool limit;
    int sh_tl;
    int sh_th;
    int sh_al;
    int sh_ah;
    int coe[9];
    int con;
    int sat;
    int hue;
    int bri;
    int cx0;
    int cx1;
    int cy0;
    int cy1;
} Cfg;

enum { R_, GR_, GB_, B_ };

static int clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static int mir(int i, int n)
{
    if (i < 0) {
        i = -i;
    }
    if (i >= n) {
        i = 2 * (n - 1) - i;
    }
    return i;
}

static int site(const Cfg *c, int x, int y)
{
    static const int cells[4][4] = {
        { B_, GB_, GR_, R_ }, { GB_, B_, R_, GR_ },
        { GR_, R_, B_, GB_ }, { R_, GR_, GB_, B_ },
    };

    return cells[c->bayer][(y & 1) * 2 + (x & 1)];
}

typedef struct Plane {
    int v[MAXH][MAXW];
} Plane;

static int at(const Plane *p, const Cfg *c, int x, int y)
{
    return p->v[mir(y, c->h)][mir(x, c->w)];
}

static int med9(int *a)
{
    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 9; j++) {
            if (a[j] < a[i]) {
                int t = a[i];

                a[i] = a[j];
                a[j] = t;
            }
        }
    }
    return a[4];
}

/* The Bayer stages: BLC, DPC, BF, LSC */
static void ref_bayer(const Cfg *c, const uint8_t *raw, Plane *out)
{
    Plane in, d;

    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int v = raw[y * c->w + x], n = (y & 1) * 2 + (x & 1);

            if (c->cntl & BLC_EN) {
                v = v > c->blc[n] ? v - c->blc[n] : 0;
                if (c->stretch & (8 >> n)) {
                    v = (v * 255 + (255 - c->blc[n]) / 2) / (255 - c->blc[n]);
                }
            }
            in.v[y][x] = v;
        }
    }
    d = in;
    if (c->cntl & DPC_EN) {
        for (int y = 0; y < c->h; y++) {
            for (int x = 0; x < c->w; x++) {
                int v = in.v[y][x], a[9], k = 0, sum = 0, mn = 255, mx = 0;
                int thl = c->dpc_conf & 0xff, thh = (c->dpc_conf >> 8) & 0xff;
                int fd = (c->dpc_conf >> 16) & 0x3f;
                int fb = (c->dpc_conf >> 22) & 0x3f;
                bool bad = false;

                for (int dy = -2; dy <= 2; dy += 2) {
                    for (int dx = -2; dx <= 2; dx += 2) {
                        if (dx || dy) {
                            a[k] = at(&in, c, x + dx, y + dy);
                            sum += a[k];
                            mn = MIN(mn, a[k]);
                            mx = MAX(mx, a[k]);
                            k++;
                        }
                    }
                }
                a[8] = v;
                if (c->dpc_ctrl & 1) {
                    continue;
                }
                for (int i = 0; (c->dpc_ctrl & 2) && i < c->nsta; i++) {
                    bad |= c->sta[i][0] == x && c->sta[i][1] == y;
                }
                if (c->dpc_ctrl & 4) {
                    if (!(c->dpc_ctrl & 16)) {
                        bad |= v > mx + thh || v < mn - thl;
                    } else if (!(v * 32 < mx * (thh & 31) &&
                                 v * 32 > mx * (thl & 31))) {
                        int est = sum / 8, dif = abs(est - v);
                        int avg = (sum + 8 * v) / 16;

                        bad |= (dif * 32 > avg * fd && est >= v) ||
                               (dif * 32 > (255 - avg) * fb && est < v);
                    }
                }
                if (bad) {
                    d.v[y][x] = med9(a);
                }
            }
        }
    }
    *out = d;
    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int v = d.v[y][x];

            if (c->cntl & BF_EN) {
                int64_t num = 0, den = 0, k = 0;

                for (int dy = -2; dy <= 2; dy += 2) {
                    for (int dx = -2; dx <= 2; dx += 2) {
                        int p = at(&d, c, x + dx, y + dy);
                        int r = 3 * c->sigma - abs(p - d.v[y][x]);

                        if (r > 0) {
                            num += c->gau[k] * r * p;
                            den += c->gau[k] * r;
                        }
                        k++;
                    }
                }
                v = den ? (num + den / 2) / den : v;
            }
            if (c->cntl & LSC_EN) {
                int ch = site(c, x, y), fx = (x >> 1) & 31, fy = (y >> 1) & 31;
                const uint16_t *g = c->lsc[ch];
                int64_t gain = (int64_t)g[0] * (32 - fx) * (32 - fy) +
                               (int64_t)g[1] * fx * (32 - fy) +
                               (int64_t)g[2] * (32 - fx) * fy +
                               (int64_t)g[3] * fx * fy;

                gain >>= 10;
                v = MIN(255, (v * gain) >> 8);
            }
            out->v[y][x] = v;
        }
    }
}

typedef struct RGB {
    uint8_t v[MAXH][MAXW][3];
} RGB;

/* Demosaic: green by Hamilton-Adams, red and blue by colour differences */
static void ref_demosaic(const Cfg *c, const Plane *b, RGB *out)
{
    static int gs[MAXH][MAXW], ga[MAXH][MAXW];

    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int s = site(c, x, y), v = b->v[y][x];
            int gw, ge, gn, gsth, gh, gv, dh, dv;

            if (s == GR_ || s == GB_) {
                gs[y][x] = ga[y][x] = 8 * v;
                continue;
            }
            gw = at(b, c, x - 1, y);
            ge = at(b, c, x + 1, y);
            gn = at(b, c, x, y - 1);
            gsth = at(b, c, x, y + 1);
            gh = 4 * (gw + ge) + 2 * (2 * v - at(b, c, x - 2, y) -
                                      at(b, c, x + 2, y));
            gv = 4 * (gn + gsth) + 2 * (2 * v - at(b, c, x, y - 2) -
                                        at(b, c, x, y + 2));
            dh = abs(gw - ge);
            dv = abs(gn - gsth);
            ga[y][x] = (gh + gv) / 2;
            gs[y][x] = dh * c->grad < dv * 16 ? gh :
                       dv * c->grad < dh * 16 ? gv : ga[y][x];
        }
    }
    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int s = site(c, x, y), v = b->v[y][x];
#define CD(X, Y) (8 * at(b, c, X, Y) - ga[mir(Y, c->h)][mir(X, c->w)])
            if (s == R_ || s == B_) {
                int o = 4 * gs[y][x] + CD(x - 1, y - 1) + CD(x + 1, y - 1) +
                        CD(x - 1, y + 1) + CD(x + 1, y + 1);

                out->v[y][x][1] = clamp8((gs[y][x] + 4) >> 3);
                out->v[y][x][s == R_ ? 0 : 2] = v;
                out->v[y][x][s == R_ ? 2 : 0] = clamp8((o + 16) >> 5);
            } else {
                int h = 32 * v + 2 * (CD(x - 1, y) + CD(x + 1, y));
                int vv = 32 * v + 2 * (CD(x, y - 1) + CD(x, y + 1));

                out->v[y][x][1] = v;
                out->v[y][x][0] = clamp8(((s == GR_ ? h : vv) + 16) >> 5);
                out->v[y][x][2] = clamp8(((s == GR_ ? vv : h) + 16) >> 5);
            }
#undef CD
        }
    }
}

static int gamma_ref(const Cfg *c, int ch, int v)
{
    int x0 = 0, y0 = 0;

    for (int n = 0; n < 16; n++) {
        int x1 = x0 + (1 << c->gx[ch][n]), y1 = c->gy[ch][n];

        if (v < x1 || n == 15) {
            if (n == 15) {
                int num = (y1 - y0) * (v - x0), den = 255 - x0;

                return clamp8(y0 + (int)floor((double)num / den));
            }
            return clamp8(y0 + (int)floor((double)((y1 - y0) * (v - x0)) /
                                          (1 << c->gx[ch][n])));
        }
        x0 = x1;
        y0 = y1;
    }
    return v;
}

/* The statistics' sampling points, as the reference passes them */
static Plane tap_raw;
static RGB tap_dem;
static RGB tap_ccm;
static RGB tap_gam;
static int tap_y[MAXH][MAXW];
static int tap_u[MAXH][MAXW];
static int tap_v[MAXH][MAXW];

/* MEDIAN, WBG, CCM and gamma, in place */
static void ref_rgb(const Cfg *c, RGB *p)
{
    if (c->cntl & MEDIAN_EN) {
        RGB in = *p;

        for (int y = 0; y < c->h; y++) {
            for (int x = 0; x < c->w; x++) {
                for (int k = 0; k < 3; k++) {
                    int a[9], n = 0;

                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            a[n++] = in.v[mir(y + dy, c->h)]
                                         [mir(x + dx, c->w)][k];
                        }
                    }
                    p->v[y][x][k] = med9(a);
                }
            }
        }
    }
    tap_dem = *p;
    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            uint8_t *q = p->v[y][x];
            int r, g, b;

            if (c->cntl & WBG_EN) {
                for (int k = 0; k < 3; k++) {
                    q[k] = MIN(255, (q[k] * c->wbg[k]) >> 8);
                }
            }
            r = q[0];
            g = q[1];
            b = q[2];
            if (c->cntl & CCM_EN) {
                for (int k = 0; k < 3; k++) {
                    q[k] = clamp8((int)floor((c->ccm[3 * k] * r +
                                              c->ccm[3 * k + 1] * g +
                                              c->ccm[3 * k + 2] * b) /
                                             256.0));
                }
            }
            memcpy(tap_ccm.v[y][x], q, 3);
            if (c->cntl & GAMMA_EN) {
                for (int k = 0; k < 3; k++) {
                    q[k] = gamma_ref(c, k, q[k]);
                }
            }
        }
    }
    tap_gam = *p;
}

static void to_yuv(bool bt709, const uint8_t *q, int *o)
{
    static const int k[2][9] = {
        { 77, 150, 29, -43, -85, 128, 128, -107, -21 },
        { 54, 183, 19, -29, -99, 128, 128, -116, -12 },
    };
    const int *m = k[bt709];

    for (int i = 0; i < 3; i++) {
        int v = (int)floor((m[3 * i] * q[0] + m[3 * i + 1] * q[1] +
                            m[3 * i + 2] * q[2]) / 256.0);

        o[i] = i ? clamp8(v + 128) : v;
    }
}

static void to_rgb(bool bt709, int y, int u, int v, uint8_t *q)
{
    static const int k[2][4] = { { 359, 88, 183, 454 }, { 403, 48, 120, 475 } };
    const int *m = k[bt709];

    u -= 128;
    v -= 128;
    q[0] = clamp8(y + (int)floor(m[0] * v / 256.0));
    q[1] = clamp8(y + (int)floor((-m[1] * u - m[2] * v) / 256.0));
    q[2] = clamp8(y + (int)floor(m[3] * u / 256.0));
}

/* The whole pipeline, packed as the tail packs it; the bytes' count. */
static uint32_t ref_frame(const Cfg *c, const uint8_t *raw, uint8_t *out)
{
    static Plane b;
    static RGB rgb;
    static int Y[MAXH][MAXW], U[MAXH][MAXW], V[MAXH][MAXW];
    uint32_t n = 0;
    int x0 = 0, x1 = c->w - 1, y0 = 0, y1 = c->h - 1;

    ref_bayer(c, raw, &b);
    tap_raw = b;
    if (c->cntl & CROP_EN) {
        x0 = c->cx0;
        x1 = c->cx1;
        y0 = c->cy0;
        y1 = c->cy1;
    }
    if (!(c->cntl & DEMOSAIC_EN)) {
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                out[n++] = b.v[y][x];
            }
        }
        return n;
    }
    ref_demosaic(c, &b, &rgb);
    ref_rgb(c, &rgb);
    if (c->cntl & RGB2YUV_EN) {
        for (int y = 0; y < c->h; y++) {
            for (int x = 0; x < c->w; x++) {
                int a[3], f[3];

                to_yuv(c->bt709, rgb.v[y][x], a);
                to_yuv(c->bt709, rgb.v[y][x & ~1], f);
                Y[y][x] = tap_y[y][x] = a[0];
                U[y][x] = tap_u[y][x] = f[1];
                V[y][x] = tap_v[y][x] = f[2];
            }
        }
        for (int y = 0; y < c->h; y++) {
            for (int x = 0; x < c->w; x++) {
                int y_ = Y[y][x], u = U[y][x], v = V[y][x];

                if (c->cntl & SHARP_EN) {
                    int sum = 0, ws = 0, l, hf, a;

                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            sum += c->coe[3 * i + j] *
                                   tap_y[mir(y + i - 1, c->h)]
                                        [mir(x + j - 1, c->w)];
                            ws += c->coe[3 * i + j];
                        }
                    }
                    l = ws ? sum / ws : y_;
                    hf = y_ - l;
                    if (abs(hf) < c->sh_tl) {
                        y_ = clamp8(l);
                    } else {
                        a = abs(hf) < c->sh_th ? c->sh_al : c->sh_ah;
                        y_ = clamp8(l + (int)floor(hf * a / 32.0));
                    }
                }
                if (c->cntl & COLOR_EN) {
                    double th = c->hue * M_PI / 180;
                    int sn = (int)lround(256 * sin(th));
                    int cs = (int)lround(256 * cos(th));
                    int uu = u - 128, vv = v - 128;

                    y_ = clamp8((int)floor((y_ - 128) * c->con / 128.0) +
                                128 + c->bri);
                    u = clamp8(128 + (int)floor((uu * cs - vv * sn) *
                                                c->sat / 32768.0));
                    v = clamp8(128 + (int)floor((uu * sn + vv * cs) *
                                                c->sat / 32768.0));
                }
                if (c->limit) {
                    y_ = (y_ * 219) / 256 + 16;
                    u = (int)floor((u - 128) * 224 / 256.0) + 128;
                    v = (int)floor((v - 128) * 224 / 256.0) + 128;
                }
                Y[y][x] = y_;
                U[y][x] = u;
                V[y][x] = v;
            }
        }
    }
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            uint8_t q[3];
            int yy, u, v;

            if (c->cntl & RGB2YUV_EN) {
                yy = Y[y][x];
                u = U[y][x];
                v = V[y][x];
                to_rgb(c->bt709, yy, u, v, q);
            } else {
                int t[3];

                memcpy(q, rgb.v[y][x], 3);
                to_yuv(c->bt709, q, t);
                yy = t[0];
                u = t[1];
                v = t[2];
            }
            switch (c->out) {
            case OUT_YUV422:
                if (!((x - x0) & 1)) {
                    out[n++] = u;
                    out[n++] = yy;
                    out[n++] = v;
                } else {
                    out[n++] = yy;
                }
                break;
            case OUT_YUV420:
                if (!((x - x0) & 1)) {
                    out[n++] = ((y - y0) & 1) ? v : u;
                }
                out[n++] = yy;
                break;
            case OUT_RGB565: {
                uint16_t w = (q[0] >> 3) << 11 | (q[1] >> 2) << 5 | q[2] >> 3;

                out[n++] = w;
                out[n++] = w >> 8;
                break;
            }
            default:
                out[n++] = q[2];
                out[n++] = q[1];
                out[n++] = q[0];
                break;
            }
        }
    }
    return n;
}

/* ---- Driving the ISP ---- */

static Cfg base_cfg(int w, int h, uint32_t cntl, int out)
{
    Cfg c = { .w = w, .h = h, .cntl = cntl, .out = out, .grad = 16 };

    for (int k = 0; k < 3; k++) {
        c.wbg[k] = 256;
        for (int n = 0; n < 16; n++) {
            c.gx[k][n] = 4;
            c.gy[k][n] = n == 15 ? 255 : 16 * (n + 1);
        }
    }
    c.ccm[0] = c.ccm[4] = c.ccm[8] = 256;
    c.sigma = 2;
    c.con = c.sat = 128;
    return c;
}

static uint32_t sm13(int v)
{
    return v < 0 ? (1u << 12) | (uint32_t)-v : (uint32_t)v;
}

/* The registers for @c, every shadowed module's taken at each frame. */
static void apply(QTestState *qts, const Cfg *c)
{
    qtest_writel(qts, I_SHADOW_REG_CTRL, 1u << 30);
    qtest_writel(qts, I_FRAME_CFG, (uint32_t)c->bayer << 27 |
                 3u << 29 | (uint32_t)(c->w - 1) << 12 | (c->h - 1));
    qtest_writel(qts, I_BLC_VALUE, c->blc[0] << 24 | c->blc[1] << 16 |
                 c->blc[2] << 8 | c->blc[3]);
    qtest_writel(qts, I_BLC_CTRL0, c->stretch);
    qtest_writel(qts, I_DPC_CTRL, c->dpc_ctrl);
    qtest_writel(qts, I_DPC_CONF, c->dpc_conf);
    qtest_writel(qts, I_BF_SIGMA, c->sigma);
    qtest_writel(qts, I_BF_GAU0, c->gau[0] << 28 | c->gau[1] << 24 |
                 c->gau[2] << 20 | c->gau[3] << 16 | c->gau[4] << 12 |
                 c->gau[5] << 8 | c->gau[6] << 4 | c->gau[7]);
    qtest_writel(qts, I_BF_GAU1, c->gau[8]);
    qtest_writel(qts, I_DEMOSAIC_GRAD, c->grad);
    for (int k = 0; k < 3; k++) {
        qtest_writel(qts, I_WBG(k), c->wbg[k]);
    }
    qtest_writel(qts, I_CCM_COEF0, sm13(c->ccm[0]) | sm13(c->ccm[1]) << 13);
    qtest_writel(qts, I_CCM_COEF1, sm13(c->ccm[2]) | sm13(c->ccm[3]) << 13);
    qtest_writel(qts, I_CCM_COEF3, sm13(c->ccm[4]) | sm13(c->ccm[5]) << 13);
    qtest_writel(qts, I_CCM_COEF4, sm13(c->ccm[6]) | sm13(c->ccm[7]) << 13);
    qtest_writel(qts, I_CCM_COEF5, sm13(c->ccm[8]));
    for (int ch = 0; ch < 3; ch++) {
        for (int i = 0; i < 4; i++) {
            const uint8_t *y = &c->gy[ch][4 * i];

            qtest_writel(qts, I_GAMMA_Y(ch, i), y[0] << 24 | y[1] << 16 |
                         y[2] << 8 | y[3]);
        }
        for (int i = 0; i < 2; i++) {
            uint32_t v = 0;

            for (int n = 0; n < 8; n++) {
                v |= (uint32_t)c->gx[ch][8 * i + n] << (21 - 3 * n);
            }
            qtest_writel(qts, I_GAMMA_X(ch, i), v);
        }
    }
    /* The curves into effect, 16 cycles later */
    qtest_writel(qts, I_GAMMA_CTRL, 0xf);
    step(qts, NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, I_GAMMA_CTRL), ==, 0xe);
    qtest_writel(qts, I_YUV_FORMAT, c->bt709 | c->limit << 1);
    qtest_writel(qts, I_SHARP_CTRL0, c->sh_tl | c->sh_th << 8 |
                 c->sh_al << 16 | (uint32_t)c->sh_ah << 24);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, I_SHARP_FILTER(i), c->coe[3 * i] |
                     c->coe[3 * i + 1] << 5 | c->coe[3 * i + 2] << 10);
    }
    qtest_writel(qts, I_COLOR_CTRL, c->sat | (c->hue & 0xff) << 8 |
                 c->con << 16 | (uint32_t)(uint8_t)c->bri << 24);
    qtest_writel(qts, I_COLOR_HUE_CTRL, c->hue >> 8);
    qtest_writel(qts, I_CROP_X, c->cx0 | c->cx1 << 12);
    qtest_writel(qts, I_CROP_Y, c->cy0 | c->cy1 << 12);
    qtest_writel(qts, I_CNTL, c->cntl | ISP_EN | MIPI_DATA_EN |
                 CNTL_OUT(c->out));
}

/* Pseudo-random samples, the same each run */
static void noise(uint8_t *p, int n, uint32_t seed)
{
    for (int i = 0; i < n; i++) {
        seed = seed * 1103515245 + 12345;
        p[i] = seed >> 16;
    }
}

/*
 * A frame of @c->w x @c->h RAW8 samples through the ISP, a row every 2
 * us (the pipeline's 40 MHz takes a row of 32 in under 1 us), then time
 * for the rest to come out.
 */
static void run_frame(QTestState *qts, const Cfg *c, const uint8_t *raw)
{
    csi_send(qts, 0x00, 1, NULL);
    for (int y = 0; y < c->h; y++) {
        step(qts, 2 * NS_PER_US);
        csi_send(qts, 0x2a, c->w, raw + y * c->w);
    }
    step(qts, 2 * NS_PER_US);
    csi_send(qts, 0x01, 1, NULL);
    step(qts, 50 * NS_PER_US);
}

static void check_frame(QTestState *qts, const Cfg *c, const uint8_t *raw)
{
    uint8_t want[MAXW * MAXH * 3], got[MAXW * MAXH * 3 + 8];
    uint32_t n = ref_frame(c, raw, want);

    run_frame(qts, c, raw);
    cb_read(qts, got, ROUND_UP(n, 8));
    for (uint32_t i = 0; i < n; i++) {
        if (got[i] != want[i]) {
            g_test_message("byte %u: got 0x%02x, want 0x%02x", i, got[i],
                           want[i]);
            g_assert_cmphex(got[i], ==, want[i]);
        }
    }
}

/* A stage's frame: @c's configuration, its own noise */
static void stage_test(const Cfg *c, uint32_t seed)
{
    QTestState *qts = machine(NULL);
    uint8_t raw[MAXW * MAXH];

    noise(raw, c->w * c->h, seed);
    apply(qts, c);
    check_frame(qts, c, raw);
    qtest_quit(qts);
}


/* A smooth ramp with a little noise */
static void smooth(uint8_t *p, int w, int h, uint32_t seed)
{
    uint8_t n[MAXW * MAXH];

    noise(n, w * h, seed);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            p[y * w + x] = 60 + 5 * x + 7 * y + (n[y * w + x] & 7);
        }
    }
}

/* ---- Each stage on its own ---- */

static void test_raw(void)
{
    Cfg c = base_cfg(16, 8, 0, OUT_RAW8);

    stage_test(&c, 1);
}

static void test_blc(void)
{
    Cfg c = base_cfg(16, 8, BLC_EN, OUT_RAW8);

    c.blc[0] = 10;
    c.blc[1] = 20;
    c.blc[2] = 30;
    c.blc[3] = 200;
    c.stretch = 8 | 2;
    stage_test(&c, 2);
}

static void dpc_run(Cfg *c)
{
    QTestState *qts = machine(NULL);
    uint8_t raw[MAXW * MAXH];

    smooth(raw, c->w, c->h, 3);
    raw[3 * c->w + 5] = 255;
    raw[4 * c->w + 8] = 0;
    raw[2 * c->w + 11] = 250;
    raw[6 * c->w + 2] = 5;
    /* Dim, at 3/8 of its neighbours: algorithm 1's first step keeps it */
    raw[5 * c->w + 10] = raw[5 * c->w + 10] * 3 / 8;
    raw[0] = 255;
    raw[c->w * c->h - 1] = 0;
    apply(qts, c);
    check_frame(qts, c, raw);
    qtest_quit(qts);
}

static void test_dpc_dynamic(void)
{
    Cfg c = base_cfg(16, 8, DPC_EN, OUT_RAW8);

    /* Algorithm 0, thresholds 20 */
    c.dpc_ctrl = 4;
    c.dpc_conf = 20 | 20 << 8 | 16 << 16 | 16 << 22;
    dpc_run(&c);
    /* Algorithm 1, ESP-IDF's test app's ratios and factors */
    c.dpc_ctrl = 4 | 16;
    c.dpc_conf = 8 | 16 << 8 | 16 << 16 | 16 << 22;
    dpc_run(&c);
}

static void lut_write(QTestState *qts, int num, int addr, uint32_t v)
{
    qtest_writel(qts, I_LUT_WDATA, v);
    qtest_writel(qts, I_LUT_CMD, 1u << 16 | num << 12 | addr);
}

static uint32_t lut_read(QTestState *qts, int num, int addr)
{
    qtest_writel(qts, I_LUT_CMD, num << 12 | addr);
    return qtest_readl(qts, I_LUT_RDATA);
}

static void test_dpc_static(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, DPC_EN, OUT_RAW8);
    uint8_t raw[MAXW * MAXH];
    static const int pts[3][2] = { { 5, 3 }, { 8, 4 }, { 2, 6 } };

    c.dpc_ctrl = 2;
    c.nsta = 3;
    memcpy(c.sta, pts, sizeof(pts));
    for (int i = 0; i < 3; i++) {
        lut_write(qts, 1, i, pts[i][1] << 11 | pts[i][0]);
    }
    for (int i = 3; i < 8; i++) {
        lut_write(qts, 1, i, 0xffffffff);
    }
    smooth(raw, c.w, c.h, 4);
    apply(qts, &c);
    check_frame(qts, &c, raw);
    qtest_quit(qts);
}

/*
 * Static calibration on a black image: the samples above DPC_THRESHOLD_L
 * into the LUT in raster order, counted; with its output, the frame as it
 * came, without, nothing.
 */
static void test_dpc_check(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, DPC_EN, OUT_RAW8);
    uint8_t raw[MAXW * MAXH];

    memset(raw, 5, sizeof(raw));
    raw[1 * 16 + 3] = 200;
    raw[4 * 16 + 9] = 12;
    raw[7 * 16 + 15] = 255;
    c.dpc_ctrl = 1 | 2 | 8 | 32;
    c.dpc_conf = 10;
    apply(qts, &c);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    check_frame(qts, &c, raw);
    g_assert_cmpuint(qtest_readl(qts, I_DPC_DEADPIX_CNT), ==, 3);
    g_assert_cmphex(lut_read(qts, 1, 0), ==, 1 << 11 | 3);
    g_assert_cmphex(lut_read(qts, 1, 1), ==, 4 << 11 | 9);
    g_assert_cmphex(lut_read(qts, 1, 2), ==, 7 << 11 | 15);
    g_assert_true(qtest_readl(qts, I_INT_RAW) & INT_DPC_CHECK_DONE);

    c.dpc_ctrl &= ~32u;
    apply(qts, &c);
    run_frame(qts, &c, raw);
    g_assert_cmpuint(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, ==, 0);
    qtest_quit(qts);
}

static void test_bf(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, BF_EN, OUT_RAW8);
    uint8_t raw[MAXW * MAXH];
    static const uint8_t t[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };

    memcpy(c.gau, t, 9);
    c.sigma = 5;
    noise(raw, 16 * 8, 5);
    for (int i = 0; i < 16 * 8; i++) {
        raw[i] = 100 + (raw[i] & 31);
    }
    apply(qts, &c);
    check_frame(qts, &c, raw);
    /* SIGMA past 20: the strongest */
    c.sigma = 20;
    apply(qts, &c);
    qtest_writel(qts, I_BF_SIGMA, 40);
    check_frame(qts, &c, raw);
    qtest_quit(qts);
}

static void test_lsc(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, LSC_EN, OUT_RAW8);
    uint8_t raw[MAXW * MAXH];
    static const uint16_t g[4][4] = {
        { 256, 300, 280, 400 }, { 240, 256, 512, 600 },
        { 128, 200, 300, 1000 }, { 900, 700, 256, 260 },
    };

    memcpy(c.lsc, g, sizeof(g));
    qtest_writel(qts, I_LSC_TABLESIZE, 2);
    for (int v = 0; v < 4; v++) {
        lut_write(qts, 0, 1 << 10 | v, g[R_][v] << 10 | g[GR_][v]);
        lut_write(qts, 0, v, g[GB_][v] << 10 | g[B_][v]);
    }
    g_assert_cmphex(lut_read(qts, 0, 1 << 10 | 3), ==, 400 << 10 | 600);
    g_assert_cmphex(lut_read(qts, 0, 3), ==, 1000 << 10 | 260);
    smooth(raw, 16, 8, 6);
    apply(qts, &c);
    check_frame(qts, &c, raw);
    qtest_quit(qts);
}

static void test_demosaic(void)
{
    QTestState *qts = machine(NULL);
    uint8_t raw[MAXW * MAXH];

    noise(raw, MAXW * MAXH, 7);
    for (int bayer = 0; bayer < 4; bayer++) {
        Cfg c = base_cfg(16, 8, DEMOSAIC_EN, OUT_RAW8 + OUT_RGB888);

        c.bayer = bayer;
        c.grad = bayer & 1 ? 40 : 16;
        apply(qts, &c);
        check_frame(qts, &c, raw);
    }
    qtest_quit(qts);
}

static void test_median(void)
{
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | MEDIAN_EN, OUT_RGB888);

    stage_test(&c, 8);
}

/* A concave curve, X intervals 8, 16 and 32 adding up to 256 */
static void gamma_curve(Cfg *c)
{
    static const uint8_t xp[16] = { 3, 3, 3, 3, 4, 4, 4, 4,
                                    4, 4, 4, 4, 4, 5, 5, 4 };
    int x = 0;

    for (int n = 0; n < 16; n++) {
        x += 1 << xp[n];
        for (int ch = 0; ch < 3; ch++) {
            c->gx[ch][n] = xp[n];
            c->gy[ch][n] = MIN(255, (int)(255 * pow(x / 256.0,
                                                    0.5 + 0.1 * ch)));
        }
    }
}

static void test_rgb(void)
{
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | WBG_EN | CCM_EN | GAMMA_EN,
                     OUT_RGB888);
    static const int m[9] = { 300, -30, -14, -20, 290, -14, -10, -40, 306 };

    c.wbg[0] = 300;
    c.wbg[2] = 200;
    memcpy(c.ccm, m, sizeof(m));
    gamma_curve(&c);
    stage_test(&c, 9);
}

static void test_yuv(void)
{
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN, OUT_YUV422);

    stage_test(&c, 10);
    c.bt709 = true;
    stage_test(&c, 11);
    c.limit = true;
    stage_test(&c, 12);
    /* RGB out through YUV, as ESP-IDF configures RGB output */
    c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | YUV2RGB_EN, OUT_RGB888);
    stage_test(&c, 13);
}

static void test_sharpen(void)
{
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | SHARP_EN, OUT_YUV422);
    static const int coe[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };

    memcpy(c.coe, coe, sizeof(coe));
    c.sh_tl = 4;
    c.sh_th = 20;
    c.sh_al = 48;
    c.sh_ah = 80;
    stage_test(&c, 14);
}

static void test_color(void)
{
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | COLOR_EN, OUT_YUV422);

    c.con = 150;
    c.sat = 100;
    c.hue = 30;
    c.bri = -10;
    stage_test(&c, 15);
    c.hue = 300;
    c.bri = 20;
    stage_test(&c, 16);
}

static void test_crop(void)
{
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | YUV2RGB_EN | CROP_EN,
                     OUT_RGB565);

    c.cx0 = 2;
    c.cx1 = 13;
    c.cy0 = 2;
    c.cy1 = 5;
    stage_test(&c, 17);
    c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | CROP_EN, OUT_YUV420);
    c.cx0 = 4;
    c.cx1 = 11;
    c.cy0 = 0;
    c.cy1 = 5;
    stage_test(&c, 18);
}

/* ---- Statistics ---- */

static int luma(const uint8_t *q)
{
    return (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
}

static int ymir(const Cfg *c, int x, int y)
{
    return tap_y[mir(y, c->h)][mir(x, c->w)];
}

/*
 * One frame with BLC's measurement, AE, AF, AWB and HIST all on, each
 * result against the reference's sampling points.
 */
static void test_stats(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | SHARP_EN | BLC_EN |
                     AE_EN | AF_EN | AWB_EN | HIST_EN, OUT_YUV422);
    static const int coe[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };
    static const int af_win[3][4] = {
        { 2, 13, 1, 6 }, { 4, 9, 2, 5 }, { 1, 14, 1, 6 },
    };
    uint8_t raw[MAXW * MAXH], out[MAXW * MAXH * 3];
    uint32_t recip = (1u << 20) / 3, want_ints;
    uint64_t ae[25] = { 0 }, awb[25][4] = { { 0 } }, hist[16] = { 0 };
    uint64_t af_sum[3] = { 0 }, af_lum[3] = { 0 };
    uint32_t awb_cnt = 0, awb_acc[3] = { 0 }, blc_sum[4] = { 0 };
    uint32_t blc_cnt[4] = { 0 }, sharp_max = 0;
    uint8_t hist_w[25];

    memcpy(c.coe, coe, sizeof(coe));
    c.sh_tl = 2;
    c.sh_th = 30;
    c.sh_al = 40;
    c.sh_ah = 60;
    smooth(raw, 16, 8, 21);
    noise(out, 16 * 8, 22);
    for (int i = 0; i < 16 * 8; i++) {
        raw[i] = MIN(255, raw[i] + (out[i] & 63));
    }
    apply(qts, &c);
    qtest_writel(qts, I_BLC_CTRL1, 2 | 2 << 11 | 2 << 22 | 3 << 26 | 1 << 30);
    qtest_writel(qts, I_BLC_CTRL2, 0xc8c8c8c8);
    qtest_writel(qts, I_AE_BX, 3 | 1 << 11);
    qtest_writel(qts, I_AE_BY, 1);
    qtest_writel(qts, I_AE_RECIP, recip);
    qtest_writel(qts, I_AE_CTRL, 1);
    qtest_writel(qts, I_AWB_MODE, 1);
    qtest_writel(qts, I_AWB_HSCALE, 15);
    qtest_writel(qts, I_AWB_VSCALE, 7);
    qtest_writel(qts, I_AWB_TH_LUM, 30 | 700 << 16);
    qtest_writel(qts, I_AWB_TH_RG, 128 | 384 << 16);
    qtest_writel(qts, I_AWB_TH_BG, 64 | 512 << 16);
    qtest_writel(qts, I_AWB_BX, 3);
    qtest_writel(qts, I_AWB_BY, 1);
    qtest_writel(qts, I_HIST_MODE, 5);
    qtest_writel(qts, I_HIST_OFFS, 0);
    qtest_writel(qts, I_HIST_SIZE, 1 | 3 << 16);
    qtest_writel(qts, I_AF_CTRL0, 1);
    qtest_writel(qts, I_AF_THRESHOLD, 8);
    for (int k = 0; k < 3; k++) {
        qtest_writel(qts, I_AF_HSCALE(k), af_win[k][1] | af_win[k][0] << 16);
        qtest_writel(qts, I_AF_VSCALE(k), af_win[k][3] | af_win[k][2] << 16);
    }
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    check_frame(qts, &c, raw);

    for (int i = 0; i < 25; i++) {
        hist_w[i] = i == 12 ? 232 : 1;
    }
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 16; x++) {
            const uint8_t *q = tap_dem.v[y][x];
            int n = (y & 1) * 2 + (x & 1), v = raw[y * 16 + x], bin = 0;
            int sum = q[0] + q[1] + q[2];
            int rg = q[1] ? MIN(q[0] * 256 / q[1], 1023) : 1023;
            int bg = q[1] ? MIN(q[2] * 256 / q[1], 1023) : 1023;
            int l = 0, ws = 0, hf;

            if (x >= 2 && x < 8 && y >= 2 && y < 6 && v <= 200) {
                blc_sum[n] += v;
                blc_cnt[n]++;
            }
            if (x >= 1 && (x - 1) / 3 < 5 && y < 5) {
                ae[y * 5 + (x - 1) / 3] += luma(q);
            }
            if (sum >= 30 && sum <= 700 && rg >= 128 && rg <= 384 &&
                bg >= 64 && bg <= 512) {
                awb_cnt++;
                for (int k = 0; k < 3; k++) {
                    awb_acc[k] += q[k];
                }
                if (x / 3 < 5 && y < 5) {
                    awb[y * 5 + x / 3][0]++;
                    for (int k = 0; k < 3; k++) {
                        awb[y * 5 + x / 3][k + 1] += q[k];
                    }
                }
            }
            if (x / 3 < 5 && y < 5) {
                static const int seg[15] = {
                    16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192,
                    208, 224, 240,
                };

                while (bin < 15 && tap_y[y][x] >= seg[bin]) {
                    bin++;
                }
                hist[bin] += hist_w[y * 5 + x / 3];
            }
            for (int k = 0; k < 3; k++) {
                int g = abs(ymir(&c, x + 1, y) - ymir(&c, x - 1, y)) +
                        abs(ymir(&c, x, y + 1) - ymir(&c, x, y - 1));

                if (x < af_win[k][0] || x > af_win[k][1] ||
                    y < af_win[k][2] || y > af_win[k][3]) {
                    continue;
                }
                af_lum[k] += tap_y[y][x];
                if (g >= 8) {
                    af_sum[k] += g;
                }
            }
            for (int i = 0; i < 9; i++) {
                l += coe[i] * ymir(&c, x + i % 3 - 1, y + i / 3 - 1);
                ws += coe[i];
            }
            hf = abs(tap_y[y][x] - l / ws);
            sharp_max = MAX(sharp_max, MIN(hf, 255));
        }
    }

    for (int n = 0; n < 4; n++) {
        g_assert_cmpuint((qtest_readl(qts, I_BLC_MEAN) >> (24 - 8 * n)) &
                         0xff, ==, blc_cnt[n] ? blc_sum[n] / blc_cnt[n] : 0);
    }
    for (int i = 0; i < 25; i++) {
        uint32_t m = (qtest_readl(qts, I_AE_MEAN(i / 4)) >>
                      (24 - 8 * (i % 4))) & 0xff;

        g_assert_cmpuint(m, ==, MIN(255, (ae[i] * recip) >> 20));
    }
    g_assert_cmpuint(qtest_readl(qts, I_AWB0_WHITE_CNT), ==, awb_cnt);
    for (int k = 0; k < 3; k++) {
        g_assert_cmpuint(qtest_readl(qts, I_AWB0_ACC(k)), ==, awb_acc[k]);
    }
    for (int i = 0; i < 25; i++) {
        for (int k = 0; k < 4; k++) {
            g_assert_cmpuint(lut_read(qts, 2, 4 * i + k), ==, awb[i][k]);
        }
    }
    for (int k = 0; k < 16; k++) {
        g_assert_cmpuint(qtest_readl(qts, I_HIST_BIN(k)), ==, hist[k] >> 8);
    }
    for (int k = 0; k < 3; k++) {
        g_assert_cmpuint(qtest_readl(qts, I_AF_SUM(k)), ==, af_sum[k]);
        g_assert_cmpuint(qtest_readl(qts, I_AF_LUM(k)), ==, af_lum[k]);
    }
    g_assert_cmpuint(qtest_readl(qts, I_SHARP_CTRL1), ==, sharp_max);
    want_ints = INT_FRAME | INT_BLC_FRAME | INT_DEMOSAIC_FRAME |
                INT_RGB2YUV_FRAME | INT_SHARP_FRAME | INT_AE_DONE |
                INT_AF_FDONE | INT_AWB_FDONE | INT_HIST_FDONE |
                INT_TAIL_FRAME | INT_HEADER_FRAME;
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW), ==, want_ints);
    qtest_quit(qts);
}

/*
 * AE's monitor every frame (its means' average outside TL..TH), AWB's
 * one frame a rising edge of AWB_EN, and AF's scene monitor: a change
 * past the threshold twice running.
 */
static void test_monitors(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | AE_EN | AF_EN | AWB_EN,
                     OUT_YUV422);
    uint8_t raw[MAXW * MAXH], out[MAXW * MAXH * 3];
    uint32_t r;

    memset(raw, 100, sizeof(raw));
    apply(qts, &c);
    qtest_writel(qts, I_AE_BX, 3);
    qtest_writel(qts, I_AE_BY, 1);
    qtest_writel(qts, I_AE_RECIP, (1u << 20) / 3);
    qtest_writel(qts, I_AE_MONITOR, 120 | 200 << 8 | 1 << 16);
    qtest_writel(qts, I_AWB_MODE, 1);
    qtest_writel(qts, I_AF_HSCALE(0), 13 | 2 << 16);
    qtest_writel(qts, I_AF_VSCALE(0), 5 | 2 << 16);
    qtest_writel(qts, I_AF_CTRL0, 1 << 4 | 4 << 8 | 1 << 16);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);

    /* Mean 100, below 120: the monitor; AWB's first frame; AF's baseline */
    run_frame(qts, &c, raw);
    r = qtest_readl(qts, I_INT_RAW);
    g_assert_cmphex(r & (INT_AE_MONITOR | INT_AE_DONE | INT_AWB_FDONE |
                         INT_AF_FDONE | INT_AF_ENV), ==,
                    INT_AE_MONITOR | INT_AE_DONE | INT_AWB_FDONE |
                    INT_AF_FDONE);
    /* Three pixels of luma 100 times the reciprocal, rounded down: 99 */
    g_assert_cmpuint(qtest_readl(qts, I_AE_MEAN(0)) >> 24, ==,
                     (300 * ((1u << 20) / 3)) >> 20);
    g_assert_cmpuint(qtest_readl(qts, I_AF_LUM(0)), ==, 100 * 12 * 4);
    cb_read(qts, out, 16 * 8 * 2);

    /* Mean 150: within; no AWB without a new edge; AF's lum up 50% */
    memset(raw, 150, sizeof(raw));
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    run_frame(qts, &c, raw);
    r = qtest_readl(qts, I_INT_RAW);
    g_assert_cmphex(r & (INT_AE_MONITOR | INT_AE_DONE | INT_AWB_FDONE |
                         INT_AF_FDONE | INT_AF_ENV), ==, INT_AE_DONE);
    cb_read(qts, out, 16 * 8 * 2);

    /* The change again: AF's scene monitor; AWB after 0 then 1 */
    qtest_writel(qts, I_CNTL, qtest_readl(qts, I_CNTL) & ~AWB_EN);
    qtest_writel(qts, I_CNTL, qtest_readl(qts, I_CNTL) | AWB_EN);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    run_frame(qts, &c, raw);
    r = qtest_readl(qts, I_INT_RAW);
    g_assert_cmphex(r & (INT_AE_MONITOR | INT_AWB_FDONE | INT_AF_FDONE |
                         INT_AF_ENV), ==, INT_AWB_FDONE | INT_AF_ENV);
    g_assert_cmpuint(qtest_readl(qts, I_AWB0_WHITE_CNT), ==, 16 * 8);
    qtest_quit(qts);
}

/* ---- Interrupts, registers, timing ---- */

static bool isp_intr(QTestState *qts)
{
    return (qtest_readl(qts, INTMTX_STATUS(SRC_ISP)) >> (SRC_ISP % 32)) & 1;
}

static void test_errors(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, 0, OUT_RAW8);
    uint8_t raw[MAXW * MAXH];

    noise(raw, sizeof(raw), 30);
    apply(qts, &c);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    csi_send(qts, 0x00, 1, NULL);
    step(qts, 2 * NS_PER_US);
    /* An RGB888 row, a RAW10 one, a short one */
    csi_send(qts, 0x24, 48, raw);
    step(qts, 2 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW), ==, INT_DATA_TYPE_ERR);
    csi_send(qts, 0x2b, 20, raw);
    step(qts, 2 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW) & INT_DT_SETTING_ERR, ==,
                    INT_DT_SETTING_ERR);
    csi_send(qts, 0x2a, 12, raw);
    step(qts, 2 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW) & INT_HNUM_UNMATCH, ==,
                    INT_HNUM_UNMATCH);
    /* ENA's reset value: the data type error is signalled */
    g_assert_true(isp_intr(qts));
    qtest_writel(qts, I_INT_CLR, INT_DATA_TYPE_ERR);
    g_assert_false(isp_intr(qts));
    csi_send(qts, 0x01, 1, NULL);
    step(qts, 50 * NS_PER_US);

    /* An odd width; a crop window starting on an odd column */
    c.w = 15;
    c.cntl |= CROP_EN;
    c.cx0 = 1;
    c.cx1 = 9;
    c.cy0 = 0;
    c.cy1 = 3;
    apply(qts, &c);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    csi_send(qts, 0x00, 1, NULL);
    step(qts, NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW) &
                    (INT_HVNUM_ERR | INT_CROP_ERR), ==,
                    INT_HVNUM_ERR | INT_CROP_ERR);
    g_assert_cmphex(qtest_readl(qts, I_CROP_ERR_ST), ==, 1 << 5);
    qtest_writel(qts, I_CROP_CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, I_CROP_ERR_ST), ==, 0);
    csi_send(qts, 0x01, 1, NULL);
    step(qts, 50 * NS_PER_US);

    /* Rows faster than the pipeline: the header's buffer fills */
    c = base_cfg(16, 8, 0, OUT_RAW8);
    apply(qts, &c);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    csi_send(qts, 0x00, 1, NULL);
    for (int y = 0; y < 8; y++) {
        csi_send(qts, 0x2a, 16, raw + 16 * y);
    }
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW) & INT_BUF_FULL, ==,
                    INT_BUF_FULL);
    csi_send(qts, 0x01, 1, NULL);
    step(qts, 50 * NS_PER_US);

    /* A row with ISP_CLK stopped: the input's FIFO overflows */
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    csi_send(qts, 0x00, 1, NULL);
    qtest_writel(qts, PERI_CLK_CTRL25,
                 qtest_readl(qts, PERI_CLK_CTRL25) & ~(1u << 25));
    csi_send(qts, 0x2a, 16, raw);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW) & (1u << 1), ==, 1u << 1);
    qtest_quit(qts);
}

static void test_registers(void)
{
    QTestState *qts = machine(NULL);

    g_assert_cmphex(qtest_readl(qts, I_VER_DATE), ==, 0x20210608);
    g_assert_cmphex(qtest_readl(qts, I_CNTL), ==, 0x40002442);
    g_assert_cmphex(qtest_readl(qts, I_FRAME_CFG), ==, 0x601e01e0);
    g_assert_cmphex(qtest_readl(qts, I_INT_ENA), ==, 0xc3);
    g_assert_cmphex(qtest_readl(qts, I_DMA_CNTL), ==, 0x001080a8);
    g_assert_cmphex(qtest_readl(qts, I_CAM_CNTL), ==, 4);
    g_assert_cmphex(qtest_readl(qts, I_SHADOW_REG_CTRL), ==, 0x40000000);
    g_assert_cmphex(qtest_readl(qts, I_GAMMA_Y(0, 3)), ==, 0xd0e0f0ff);
    g_assert_cmphex(qtest_readl(qts, I_AF_THRESHOLD), ==, 0x100);

    /* Read-write masks; write-trigger bits read as 0 */
    qtest_writel(qts, I_HSYNC_CNT, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, I_HSYNC_CNT), ==, 0xff);
    qtest_writel(qts, I_AE_CTRL, 3);
    g_assert_cmphex(qtest_readl(qts, I_AE_CTRL), ==, 2);
    qtest_writel(qts, I_AF_CTRL0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, I_AF_CTRL0), ==, 0x00ff0f01);
    qtest_writel(qts, I_DMA_RAW_DATA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, I_DMA_RAW_DATA), ==, 0x3fffff);
    g_assert_cmphex(qtest_readl(qts, I_LUT_CMD), ==, 0);
    qtest_writel(qts, I_BLC_MEAN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, I_BLC_MEAN), ==, 0);

    /* The LUTs */
    lut_write(qts, 0, 559, 0xfffff);
    lut_write(qts, 0, 1 << 10 | 7, 0x12345);
    g_assert_cmphex(lut_read(qts, 0, 559), ==, 0xfffff);
    g_assert_cmphex(lut_read(qts, 0, 1 << 10 | 7), ==, 0x12345);
    lut_write(qts, 1, 511, 0xdeadbeef);
    g_assert_cmphex(lut_read(qts, 1, 511), ==, 0xdeadbeef);
    lut_write(qts, 2, 5, 0x55);
    g_assert_cmphex(lut_read(qts, 2, 5), ==, 0);

    /* Self-clearing updates, a few ISP_CLK cycles (25 ns) later */
    qtest_writel(qts, I_DMA_CNTL, 0x001080a8 | 2);
    g_assert_cmphex(qtest_readl(qts, I_DMA_CNTL), ==, 0x001080aa);
    step(qts, 100);
    g_assert_cmphex(qtest_readl(qts, I_DMA_CNTL), ==, 0x001080a8);
    qtest_writel(qts, I_CAM_CNTL, 2);
    step(qts, 100);
    g_assert_cmphex(qtest_readl(qts, I_CAM_CNTL), ==, 0);
    /* GAMMA_UPDATE: 16 cycles, 400 ns; X intervals not adding up to 256 */
    qtest_writel(qts, I_GAMMA_X(1, 0), 0);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    qtest_writel(qts, I_GAMMA_CTRL, 0xf);
    step(qts, 350);
    g_assert_cmphex(qtest_readl(qts, I_GAMMA_CTRL), ==, 0xf);
    step(qts, 100);
    g_assert_cmphex(qtest_readl(qts, I_GAMMA_CTRL), ==, 0xe);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW), ==, INT_GAMMA_XCOORD);
    g_assert_true(isp_intr(qts));
    g_assert_cmphex(qtest_readl(qts, I_INT_ST), ==, INT_GAMMA_XCOORD);
    qtest_writel(qts, I_INT_ENA, 0);
    g_assert_false(isp_intr(qts));
    g_assert_cmphex(qtest_readl(qts, I_INT_ST), ==, 0);

    /* RST_EN_ISP: the registers back to their reset values, held there */
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) | 1u << 29);
    g_assert_cmphex(qtest_readl(qts, I_HSYNC_CNT), ==, 7);
    qtest_writel(qts, I_HSYNC_CNT, 9);
    g_assert_cmphex(qtest_readl(qts, I_HSYNC_CNT), ==, 7);
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) & ~(1u << 29));
    qtest_writel(qts, I_HSYNC_CNT, 9);
    g_assert_cmphex(qtest_readl(qts, I_HSYNC_CNT), ==, 9);
    /* The LUTs are memories, kept */
    g_assert_cmphex(lut_read(qts, 1, 511), ==, 0xdeadbeef);
    qtest_quit(qts);
}

/*
 * Shadow registers, SHADOW_UPDATE_SEL 2: a module's registers take effect
 * at the frame start after its *_UPDATE bit, which clears then.
 */
static void test_shadow(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, BLC_EN, OUT_RAW8);
    uint8_t raw[MAXW * MAXH];

    noise(raw, sizeof(raw), 31);
    apply(qts, &c);
    run_frame(qts, &c, raw);
    cb_read(qts, raw + 0, 0);
    qtest_writel(qts, I_SHADOW_REG_CTRL, 2u << 30);
    qtest_writel(qts, I_BLC_VALUE, 0x40404040);
    /* No BLC_UPDATE: the frame keeps the old values (0) */
    {
        uint8_t got[MAXW * MAXH];

        cb_read(qts, got, 16 * 8);
        run_frame(qts, &c, raw);
        cb_read(qts, got, 16 * 8);
        g_assert_cmpmem(got, 16 * 8, raw, 16 * 8);
    }
    qtest_writel(qts, I_SHADOW_REG_CTRL, 2u << 30 | 1);
    c.blc[0] = c.blc[1] = c.blc[2] = c.blc[3] = 0x40;
    check_frame(qts, &c, raw);
    g_assert_cmphex(qtest_readl(qts, I_SHADOW_REG_CTRL), ==, 2u << 30);
    qtest_quit(qts);
}

/*
 * The output's timing, demosaic alone (its windows reach 3 rows below):
 * row 0 leaves 16 + HSYNC_CNT (7) cycles of 25 ns after input row 3
 * comes, 575 ns; with the last input row (rows past the bottom mirror
 * rows within) rows 4 to 7 one after another, the last 2300 ns after it,
 * 300 ns after the end packet, which comes 2 us after it: the frame
 * interrupt then.
 */
static void test_timing(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN, OUT_RGB888);
    uint8_t raw[MAXW * MAXH];

    noise(raw, sizeof(raw), 32);
    apply(qts, &c);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    csi_send(qts, 0x00, 1, NULL);
    for (int y = 0; y < 8; y++) {
        step(qts, 2 * NS_PER_US);
        csi_send(qts, 0x2a, 16, raw + 16 * y);
        if (y == 3) {
            step(qts, 560);
            g_assert_cmpuint(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, ==, 0);
            step(qts, 40);
            g_assert_cmpuint(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, ==, 6);
        }
    }
    step(qts, 2 * NS_PER_US);
    csi_send(qts, 0x01, 1, NULL);
    step(qts, 290);
    g_assert_false(qtest_readl(qts, I_INT_RAW) & INT_FRAME);
    g_assert_cmpuint(qtest_readl(qts, I_STATE), ==, 1);
    step(qts, 20);
    g_assert_true(qtest_readl(qts, I_INT_RAW) & INT_FRAME);
    g_assert_cmpuint(qtest_readl(qts, I_STATE), ==, 0);
    qtest_quit(qts);
}

/*
 * MIPI CSI's data past the ISP: while it is disabled, each 32-bit word's
 * bytes reversed with BYTE_ENDIAN_ORDER; in the reset state (enabled,
 * MIPI_DATA_EN clear); not with another input selected.
 */
static void test_bypass(void)
{
    QTestState *qts = machine(NULL);
    static const uint8_t row[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    static const uint8_t swapped[8] = { 4, 3, 2, 1, 8, 7, 6, 5 };
    uint8_t got[8];

    qtest_writel(qts, I_CNTL, BYTE_ENDIAN_ORDER);
    csi_send(qts, 0x00, 1, NULL);
    csi_send(qts, 0x2a, 8, row);
    csi_send(qts, 0x01, 1, NULL);
    cb_read(qts, got, 8);
    g_assert_cmpmem(got, 8, swapped, 8);
    g_assert_cmphex(qtest_readl(qts, I_INT_RAW), ==,
                    INT_HEADER_FRAME | INT_TAIL_FRAME);

    qtest_writel(qts, I_CNTL, 0x40002442);
    csi_send(qts, 0x00, 1, NULL);
    csi_send(qts, 0x2a, 8, row);
    csi_send(qts, 0x01, 1, NULL);
    cb_read(qts, got, 8);
    g_assert_cmpmem(got, 8, row, 8);

    qtest_writel(qts, I_CNTL, ISP_EN | MIPI_DATA_EN | 2u << 27);
    csi_send(qts, 0x00, 1, NULL);
    csi_send(qts, 0x2a, 8, row);
    csi_send(qts, 0x01, 1, NULL);
    g_assert_cmpuint(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, ==, 0);
    qtest_quit(qts);
}

/* ---- The other inputs ---- */

#define VDMA                0x50081000
#define V_CFG0              (VDMA + 0x10)
#define V_CHEN0             (VDMA + 0x18)
#define V_CH(n)             (VDMA + 0x100 * (n))
#define V_SAR0              0x00
#define V_DAR0              0x08
#define V_BLOCK_TS0         0x10
#define V_CTL0              0x18
#define V_CTL1              0x1c
#define V_CH_CFG0           0x20
#define V_CH_CFG1           0x24
#define V_INTST_ENA0        0x80
#define V_INTSTATUS         0x88
#define MEM                 0x4ff40000

/*
 * Memory to the ISP through VDMA, as ESP-IDF's isp_dma.c sets it up:
 * channel 1 from memory to the CSI bridge's port, the ISP the flow
 * controller on interface 2, bursts of 8 64-bit words; DMA_EN starts the
 * frame. The result comes out of the bridge as from MIPI CSI.
 */
static void test_dma_input(void)
{
    QTestState *qts = machine(NULL);
    Cfg c = base_cfg(16, 8, DEMOSAIC_EN | RGB2YUV_EN | YUV2RGB_EN,
                     OUT_RGB888);
    uint8_t raw[MAXW * MAXH], want[MAXW * MAXH * 3], got[MAXW * MAXH * 3];
    uint32_t n;

    noise(raw, 16 * 8, 33);
    qtest_memwrite(qts, MEM, raw, 16 * 8);
    apply(qts, &c);
    qtest_writel(qts, I_CNTL, c.cntl | ISP_EN | 2u << 27 | CNTL_OUT(c.out));
    qtest_writel(qts, I_DMA_CNTL, 0x2a << 2 | 8 << 8 | 1 << 20 | 2);
    qtest_writel(qts, I_DMA_RAW_DATA, 16 * 8 / 8 | 1u << 31);
    step(qts, 100);

    qtest_writel(qts, V_CFG0, 3);
    qtest_writel(qts, V_CH(1) + V_SAR0, MEM);
    qtest_writel(qts, V_CH(1) + V_DAR0, CSI_MEM);
    qtest_writel(qts, V_CH(1) + V_BLOCK_TS0, 16 * 8 / 8 - 1);
    qtest_writel(qts, V_CH(1) + V_CTL0, 1u << 0 | 1u << 6 | 3u << 8 |
                 3u << 11 | 2u << 14 | 2u << 18);
    qtest_writel(qts, V_CH(1) + V_CTL1, 0);
    qtest_writel(qts, V_CH(1) + V_CH_CFG0, 0);
    qtest_writel(qts, V_CH(1) + V_CH_CFG1, 6 | 1u << 3 | 2u << 12);
    qtest_writel(qts, V_CH(1) + V_INTST_ENA0, 0xffffffff);
    qtest_writel(qts, V_CHEN0, 1 | 1u << 8);
    qtest_writel(qts, I_INT_CLR, 0xffffffff);
    qtest_writel(qts, I_DMA_CNTL, 0x2a << 2 | 8 << 8 | 1 << 20 | 1);

    step(qts, 100 * NS_PER_US);
    g_assert_true(qtest_readl(qts, V_CH(1) + V_INTSTATUS) & 2);
    g_assert_true(qtest_readl(qts, I_INT_RAW) & INT_FRAME);
    n = ref_frame(&c, raw, want);
    cb_read(qts, got, n);
    g_assert_cmpmem(got, n, want, n);
    qtest_quit(qts);
}

/* The camera's bars pattern, RAW8 RGGB, frame @f (image source header) */
static uint8_t bars(uint32_t f, int x, int y, int w)
{
    static const uint8_t rgb[8][3] = {
        { 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 0, 255, 0 },
        { 255, 0, 255 }, { 255, 0, 0 }, { 0, 0, 255 }, { 0, 0, 0 },
    };
    int bar = ((x + f) % w) * 8 / w;
    int k = (y & 1) ? ((x & 1) ? 2 : 1) : ((x & 1) ? 1 : 0);

    if (y < 2 && x < MIN(16 * 2, w)) {
        return (f >> (15 - x / 2)) & 1 ? 255 : 0;
    }
    return rgb[bar][k];
}

/*
 * The board's DVP camera into the ISP (ISP_IN_SRC 1, CAM_EN out of
 * CAM_RESET), RAW8 through unprocessed: a whole frame of the sensor's
 * pattern, from VSYNC to its 8th row.
 */
static void test_dvp(void)
{
    QTestState *qts = machine("-object esp32p4-image-source,id=cam0,"
                              "format=raw8,width=16,height=8,fps=1000,"
                              "cell=2 -M dvp-camera=cam0");
    Cfg c = base_cfg(16, 8, 0, OUT_RAW8);
    uint8_t got[16 * 8];
    int frame = -1;

    apply(qts, &c);
    qtest_writel(qts, I_CNTL, ISP_EN | 1u << 27 | CNTL_OUT(OUT_RAW8));
    qtest_writel(qts, I_CAM_CONF, 0x2a << 2);
    qtest_writel(qts, I_CAM_CNTL, 1 << 1);
    qtest_writel(qts, I_CAM_CNTL, 1 << 0);
    step(qts, 2500 * NS_PER_US);
    g_assert_cmpuint(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, >=, 16);
    cb_read(qts, got, sizeof(got));
    for (int f = 0; f < 4 && frame < 0; f++) {
        bool ok = true;

        for (int i = 0; i < 16 * 8 && ok; i++) {
            ok = got[i] == bars(f, i % 16, i / 16, 16);
        }
        if (ok) {
            frame = f;
        }
    }
    g_assert_cmpint(frame, >=, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32p4/isp/pipe/raw", test_raw);
    qtest_add_func("/esp32p4/isp/pipe/blc", test_blc);
    qtest_add_func("/esp32p4/isp/pipe/dpc-dynamic", test_dpc_dynamic);
    qtest_add_func("/esp32p4/isp/pipe/dpc-static", test_dpc_static);
    qtest_add_func("/esp32p4/isp/pipe/dpc-check", test_dpc_check);
    qtest_add_func("/esp32p4/isp/pipe/bf", test_bf);
    qtest_add_func("/esp32p4/isp/pipe/lsc", test_lsc);
    qtest_add_func("/esp32p4/isp/pipe/demosaic", test_demosaic);
    qtest_add_func("/esp32p4/isp/pipe/median", test_median);
    qtest_add_func("/esp32p4/isp/pipe/rgb", test_rgb);
    qtest_add_func("/esp32p4/isp/pipe/yuv", test_yuv);
    qtest_add_func("/esp32p4/isp/pipe/sharpen", test_sharpen);
    qtest_add_func("/esp32p4/isp/pipe/color", test_color);
    qtest_add_func("/esp32p4/isp/pipe/crop", test_crop);
    qtest_add_func("/esp32p4/isp/stats", test_stats);
    qtest_add_func("/esp32p4/isp/monitors", test_monitors);
    qtest_add_func("/esp32p4/isp/errors", test_errors);
    qtest_add_func("/esp32p4/isp/registers", test_registers);
    qtest_add_func("/esp32p4/isp/shadow", test_shadow);
    qtest_add_func("/esp32p4/isp/timing", test_timing);
    qtest_add_func("/esp32p4/isp/bypass", test_bypass);
    qtest_add_func("/esp32p4/isp/dma-input", test_dma_input);
    qtest_add_func("/esp32p4/isp/dvp", test_dvp);
    return g_test_run();
}
