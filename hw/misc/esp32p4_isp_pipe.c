/*
 * ESP32-P4 ISP: the pipeline's arithmetic and statistics
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The stages of TRM section 38.5.2 in their order (and ESP-IDF's
 * isp.rst diagram), on 8-bit samples, every result an integer. Where the
 * TRM gives no arithmetic, the interpretation is stated with the stage;
 * esp32p4_isp.c sums the device up.
 *
 * Each stage computes a row when the rows its window needs have come
 * through the stage before (3 rows for a 3 x 3 window, 5 for one over a
 * Bayer channel's same-colour neighbours two pixels away), keeping the
 * last few rows its successors read. Past the image's edges a window
 * takes the module's padding: with PADDING_MODE 0 the image mirrored
 * about its edge row or column (which keeps a Bayer site's colour), with
 * 1 PADDING_DATA.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "esp32p4_isp_int.h"

/* Rows each stage keeps */
#define RING_ROWS       8

enum {
    S_IN,       /* the header's rows, BLC applied */
    S_DPC,
    S_BF,       /* BF, then LSC */
    S_G,        /* demosaic: green at every site */
    S_RGB,      /* demosaic: RGB */
    S_YUV,      /* MEDIAN, WBG, CCM, gamma, RGB2YUV */
    S_OUT,      /* SHARP, COLOR, YUV_Limit or YUV2RGB, CROP, ISP_Tail */
    S_NUM,
};

/* Bayer colours */
enum { C_R, C_GR, C_GB, C_B };

typedef struct Stage {
    /* Rows completed */
    uint32_t next;
    uint8_t *buf;
    uint32_t row_bytes;
    bool used;
    /* The stages it reads, and how many rows beyond the current each */
    int in[2];
    uint32_t radius[2];
    int nin;
} Stage;

struct IspPipe {
    IspFrameCfg c;
    IspEmitFn *emit;
    void *opaque;
    int w;
    int h;
    /* The colour of the 2 x 2 cell's sites, (y & 1) * 2 + (x & 1) */
    uint8_t col[4];
    Stage st[S_NUM];
    uint8_t *out;
    bool no_output;
    uint32_t dpc_ptr;
    /* CROP's window, inclusive, when in effect */
    bool crop;
    uint32_t cx0;
    uint32_t cx1;
    uint32_t cy0;
    uint32_t cy1;
    /* Statistics */
    uint64_t blc_sum[4];
    uint32_t blc_cnt[4];
    uint64_t ae_sum[25];
    uint64_t af_sum[3];
    uint64_t af_lum[3];
    uint32_t af_hist[512];
    uint64_t awb_acc[3];
    uint32_t awb_cnt;
    uint64_t awb_sub[25][4];
    uint64_t hist[16];
    uint8_t sharp_max;
    uint32_t dpc_dead;
    IspStats stats;
};

static bool warned_mismatch;
static bool warned_lsc;

static uint32_t reg(const IspPipe *p, hwaddr a)
{
    return p->c.r[R(a)];
}

static uint32_t fld(const IspPipe *p, hwaddr a, int start, int len)
{
    return extract32(reg(p, a), start, len);
}

static bool en(const IspPipe *p, uint32_t bit)
{
    return reg(p, A_CNTL) & bit;
}

static int clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static int mirror(int i, int n)
{
    if (i < 0) {
        i = -i;
    }
    if (i >= n) {
        i = 2 * (n - 1) - i;
    }
    return i < 0 ? 0 : i >= n ? n - 1 : i;
}

static uint8_t *row_of(IspPipe *p, int s, int y)
{
    Stage *t = &p->st[s];

    y = mirror(y, p->h);
    return t->buf + (y % RING_ROWS) * t->row_bytes;
}

/* A sample of stage @s's Bayer rows, padded as @mctrl says. */
static int bayer_px(IspPipe *p, int s, int x, int y, uint32_t mctrl)
{
    if (x < 0 || y < 0 || x >= p->w || y >= p->h) {
        if (mctrl & BIT(24)) {
            return extract32(mctrl, 16, 8);
        }
        x = mirror(x, p->w);
    }
    return row_of(p, s, y)[x];
}

static int colour(const IspPipe *p, int x, int y)
{
    return p->col[(y & 1) * 2 + (x & 1)];
}

/* ---- BLC (TRM 38.5.2.1) ---- */

/*
 * The black level of the sample's channel (R0 top left of each 2 x 2
 * cell, R1 top right, R2 bottom left, R3 bottom right) subtracted,
 * clamped at 0; with the channel's STRETCH, scaled back to 0..255 as
 * v * 255 / (255 - level), rounded to nearest. BLC_MEAN measures the
 * input before it: each channel's samples in BLC_WINDOW_HNUM x
 * BLC_WINDOW_VNUM 2 x 2 cells from (BLC_WINDOW_LEFT, BLC_WINDOW_TOP),
 * with BLC_FILTER_EN only those not above the channel's BLC_Rn_TH,
 * averaged (rounded down; 0 for none).
 */
static void blc_row(IspPipe *p, uint32_t y, const uint8_t *in, uint8_t *out)
{
    uint32_t value = reg(p, A_BLC_VALUE), ctrl0 = reg(p, A_BLC_CTRL0);
    uint32_t ctrl1 = reg(p, A_BLC_CTRL1), th = reg(p, A_BLC_CTRL2);
    uint32_t top = extract32(ctrl1, 0, 11), left = extract32(ctrl1, 11, 11);
    uint32_t vnum = extract32(ctrl1, 22, 4), hnum = extract32(ctrl1, 26, 4);
    bool filter = ctrl1 & BIT(30);

    for (uint32_t x = 0; x < p->w; x++) {
        int n = (y & 1) * 2 + (x & 1);
        int level = extract32(value, 24 - 8 * n, 8);
        int v = in[x];

        if (en(p, CNTL_BLC_EN) && y >= top && y < top + 2 * vnum &&
            x >= left && x < left + 2 * hnum &&
            (!filter || v <= extract32(th, 24 - 8 * n, 8))) {
            p->blc_sum[n] += v;
            p->blc_cnt[n]++;
        }
        if (en(p, CNTL_BLC_EN)) {
            v = MAX(v - level, 0);
            if ((ctrl0 & BIT(3 - n)) && level < 255) {
                v = (v * 255 + (255 - level) / 2) / (255 - level);
            }
        }
        out[x] = clamp8(v);
    }
}

/* ---- DPC (TRM 38.5.2.2) ---- */

static int median9(int *a)
{
    for (int i = 1; i < 9; i++) {
        int v = a[i], j = i - 1;

        while (j >= 0 && a[j] > v) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = v;
    }
    return a[4];
}

/*
 * Each sample against its eight same-colour neighbours, two pixels away
 * in each direction (the 3 x 3 same-colour neighbourhood ESP-IDF's
 * isp.rst describes), DPC_MATRIX_CTRL's padding past the edges. A
 * defective sample is replaced by the neighbourhood's median, of the
 * nine with itself. Dynamic algorithm 0 and 1 are the TRM's formulas,
 * algorithm 1's integer means rounded down (est = sum / 8, avg = (sum +
 * 8 pix) / 16) and its factors with 5 fractional bits (ESP-IDF's
 * format). Its first step's ratios DPC_THRESHOLD_L/H are taken with 5
 * fractional bits too, which is Espressif's silicon (ESP-IDF's DPC test
 * frames: with ESP-IDF's ratios 0.5 and 1.0, written 8 and 16, it leaves
 * a defect of 0x30 among 0x80 as it is and corrects 0x00, 0xD0 and 0xFF,
 * as these thresholds do, where the TRM's and ESP-IDF's 4 fractional
 * bits would correct all four). Static correction walks the
 * LUT in raster order, as the samples stream: an entry ((y << 11) | x,
 * ESP-IDF's format) equal to the sample's coordinates marks it, entries
 * below them are passed, and an entry of all ones ends the list, so the
 * LUT must be sorted. Static calibration (DPC_CHECK_EN, no correction)
 * writes each sample above DPC_THRESHOLD_L (black image) or below
 * DPC_THRESHOLD_H (white) into the LUT in order, up to its 512 entries,
 * counting them all in DPC_DEADPIX_CNT; without DPC_CHECK_OD_EN the
 * frame has no output.
 */
static void dpc_row(IspPipe *p, uint32_t y, uint8_t *out)
{
    const uint8_t *in = row_of(p, S_IN, y);
    uint32_t ctrl = reg(p, A_DPC_CTRL), conf = reg(p, A_DPC_CONF);
    uint32_t mctrl = reg(p, A_DPC_MATRIX_CTRL);
    int thl = extract32(conf, 0, 8), thh = extract32(conf, 8, 8);
    int fdark = extract32(conf, 16, 6), fbrig = extract32(conf, 22, 6);

    if (!en(p, CNTL_DPC_EN)) {
        memcpy(out, in, p->w);
        return;
    }
    for (uint32_t x = 0; x < p->w; x++) {
        int v = in[x], nb[9], k = 0, sum = 0, mn = 255, mx = 0;
        bool bad = false;

        for (int dy = -2; dy <= 2; dy += 2) {
            for (int dx = -2; dx <= 2; dx += 2) {
                if (dx || dy) {
                    nb[k] = bayer_px(p, S_IN, x + dx, y + dy, mctrl);
                    sum += nb[k];
                    mn = MIN(mn, nb[k]);
                    mx = MAX(mx, nb[k]);
                    k++;
                }
            }
        }
        nb[8] = v;
        out[x] = v;
        if (ctrl & BIT(0)) {
            if ((ctrl & BIT(3)) ? v > thl : v < thh) {
                if (p->dpc_dead < ISP_DPC_LUT_DEPTH) {
                    p->c.dpc_lut[p->dpc_dead] = (y << 11) | x;
                }
                p->dpc_dead++;
            }
            continue;
        }
        if (ctrl & BIT(2)) {
            if (!(ctrl & BIT(4))) {
                bad = v > mx + thh || v < mn - thl;
            } else {
                int est = sum >> 3, dif = abs(est - v);
                int avg = (sum + 8 * v) >> 4;

                if (!(v * 32 < mx * (thh & 0x1f) &&
                      v * 32 > mx * (thl & 0x1f))) {
                    bad = (dif * 32 > avg * fdark && est >= v) ||
                          (dif * 32 > (255 - avg) * fbrig && est < v);
                }
            }
        }
        if (ctrl & BIT(1)) {
            uint32_t at = (y << 11) | x;

            while (p->dpc_ptr < ISP_DPC_LUT_DEPTH &&
                   p->c.dpc_lut[p->dpc_ptr] != UINT32_MAX &&
                   p->c.dpc_lut[p->dpc_ptr] < at) {
                p->dpc_ptr++;
            }
            if (p->dpc_ptr < ISP_DPC_LUT_DEPTH &&
                p->c.dpc_lut[p->dpc_ptr] == at) {
                bad = true;
                p->dpc_ptr++;
            }
        }
        if (bad) {
            out[x] = median9(nb);
        }
    }
}

/* ---- BF (TRM 38.5.2.3) and LSC (38.5.2.4) ---- */

/*
 * BF: each sample a weighted mean of its 3 x 3 same-colour neighbourhood
 * (the GAU_TEMPLATE weights, 0..15, coordinate ij the row and column),
 * each neighbour's weight also scaled by how close it is to the sample:
 * by max(0, 3 sigma - |neighbour - sample|), SIGMA 2..20 and 20 for any
 * other value (the TRM: others are the strongest). The mean is rounded
 * to nearest; with no weight the sample stays.
 */
static int bf_px(IspPipe *p, uint32_t x, uint32_t y)
{
    uint32_t mctrl = reg(p, A_BF_MATRIX_CTRL);
    uint32_t g0 = reg(p, A_BF_GAU0), g1 = reg(p, A_BF_GAU1);
    int sigma = fld(p, A_BF_SIGMA, 0, 6);
    int t[9] = {
        extract32(g0, 28, 4), extract32(g0, 24, 4), extract32(g0, 20, 4),
        extract32(g0, 16, 4), extract32(g0, 12, 4), extract32(g0, 8, 4),
        extract32(g0, 4, 4), extract32(g0, 0, 4), extract32(g1, 0, 4),
    };
    int c = row_of(p, S_DPC, y)[x], k = 0;
    int64_t num = 0, den = 0;

    if (sigma < 2 || sigma > 20) {
        sigma = 20;
    }
    for (int dy = -2; dy <= 2; dy += 2) {
        for (int dx = -2; dx <= 2; dx += 2) {
            int v = bayer_px(p, S_DPC, x + dx, y + dy, mctrl);
            int r = 3 * sigma - abs(v - c);

            if (r > 0) {
                num += (int64_t)t[k] * r * v;
                den += (int64_t)t[k] * r;
            }
            k++;
        }
    }
    return den ? (num + den / 2) / den : c;
}

/*
 * LSC: each channel's gain at its pixel by bilinear interpolation
 * between the four LUT vertices around it, the channel's grid being of
 * 32 x 32 of its pixels (2 x 2 cells of the image); vertex (gx, gy) at
 * LUT address gy * LSC_XTABLESIZE + gx (ESP-IDF's order), R and Gr in
 * LUT_R_Gr's bits [19:10] and [9:0], Gb and B in LUT_Gb_B's, Gr being
 * the green on R's rows. Gains have 8 fractional bits; the weights
 * (32 - f) and f per axis; sample = min(255, sample * gain >> 8), the
 * gain's interpolation and the product rounded down.
 */
static uint32_t lsc_gain(IspPipe *p, int c, uint32_t idx)
{
    uint32_t word;

    if (idx >= ISP_LSC_LUT_DEPTH) {
        if (!warned_lsc) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: LSC needs LUT entry %u, "
                          "past its %u\n", TYPE_ESP32P4_ISP, idx,
                          ISP_LSC_LUT_DEPTH);
            warned_lsc = true;
        }
        idx = ISP_LSC_LUT_DEPTH - 1;
    }
    word = (c == C_R || c == C_GR) ? p->c.lsc_r_gr[idx] : p->c.lsc_gb_b[idx];
    return (c == C_R || c == C_GB) ? extract32(word, 10, 10)
                                   : extract32(word, 0, 10);
}

static int lsc_px(IspPipe *p, uint32_t x, uint32_t y, int v)
{
    uint32_t xs = MAX(fld(p, A_LSC_TABLESIZE, 0, 5), 1);
    uint32_t cx = x >> 1, cy = y >> 1;
    uint32_t gx = cx >> 5, gy = cy >> 5, fx = cx & 31, fy = cy & 31;
    uint32_t gx1 = MIN(gx + 1, xs - 1);
    int c = colour(p, x, y);
    uint64_t g;

    gx = MIN(gx, xs - 1);
    g = (uint64_t)lsc_gain(p, c, gy * xs + gx) * (32 - fx) * (32 - fy) +
        (uint64_t)lsc_gain(p, c, gy * xs + gx1) * fx * (32 - fy) +
        (uint64_t)lsc_gain(p, c, (gy + 1) * xs + gx) * (32 - fx) * fy +
        (uint64_t)lsc_gain(p, c, (gy + 1) * xs + gx1) * fx * fy;
    g >>= 10;
    return MIN(255, (v * g) >> 8);
}

static void hist_add(IspPipe *p, uint32_t x, uint32_t y, int v);

static void bf_row(IspPipe *p, uint32_t y, uint8_t *out)
{
    uint32_t mode = fld(p, A_HIST_MODE, 0, 3);
    static const uint8_t hist_col[4] = { C_B, C_GB, C_GR, C_R };

    for (uint32_t x = 0; x < p->w; x++) {
        int v = en(p, CNTL_BF_EN) ? bf_px(p, x, y) : row_of(p, S_DPC, y)[x];

        if (en(p, CNTL_LSC_EN)) {
            v = lsc_px(p, x, y, v);
        }
        out[x] = v;
        if (en(p, CNTL_HIST_EN) && mode < 4 &&
            colour(p, x, y) == hist_col[mode]) {
            hist_add(p, x, y, v);
        }
    }
}

/* ---- Demosaic (TRM 38.5.2.5) ---- */

/*
 * Green first, at every red and blue site, as Hamilton and Adams
 * interpolate it: horizontally (G_W + G_E) / 2 + (2 C - C_WW - C_EE) / 4,
 * vertically likewise, C the site's own colour two pixels away; the
 * direction with the smaller green difference (|G_W - G_E| against
 * |G_N - G_S|) taken, the other's difference scaled by
 * DEMOSAIC_GRAD_RATIO (2.4) in the comparison, and the mean of both
 * when neither is smaller. Then red and blue by colour differences: at
 * a blue (red) site the site's green plus the mean of red (blue) minus
 * green over the four diagonal neighbours; at a green site its green
 * plus the mean over the two neighbours of each colour; the green at
 * the neighbours being both directions' mean. Values are kept in eighths
 * and sixteenths, each output rounded to nearest once. This matches
 * Espressif's silicon captures in ESP-IDF's ISP test app (an impulse at
 * each Bayer site through RGB2YUV and YUV2RGB; 99.5% of pixels exact);
 * the remaining differences are around green impulses' neighbours.
 */
static void g_row(IspPipe *p, uint32_t y, int16_t *out)
{
    uint32_t mctrl = reg(p, A_DEMOSAIC_MATRIX_CTRL);
    int ratio = fld(p, A_DEMOSAIC_GRAD_RATIO, 0, 6);

    for (uint32_t x = 0; x < p->w; x++) {
        int c = colour(p, x, y), v = row_of(p, S_BF, y)[x];
        int gw, ge, gn, gs, gh, gv, dh, dv, avg, sel;

        if (c == C_GR || c == C_GB) {
            out[2 * x] = out[2 * x + 1] = 8 * v;
            continue;
        }
        gw = bayer_px(p, S_BF, x - 1, y, mctrl);
        ge = bayer_px(p, S_BF, x + 1, y, mctrl);
        gn = bayer_px(p, S_BF, x, y - 1, mctrl);
        gs = bayer_px(p, S_BF, x, y + 1, mctrl);
        gh = 4 * (gw + ge) + 2 * (2 * v - bayer_px(p, S_BF, x - 2, y, mctrl) -
                                 bayer_px(p, S_BF, x + 2, y, mctrl));
        gv = 4 * (gn + gs) + 2 * (2 * v - bayer_px(p, S_BF, x, y - 2, mctrl) -
                                 bayer_px(p, S_BF, x, y + 2, mctrl));
        dh = abs(gw - ge);
        dv = abs(gn - gs);
        avg = (gh + gv) / 2;
        if (dh * ratio < dv * 16) {
            sel = gh;
        } else if (dv * ratio < dh * 16) {
            sel = gv;
        } else {
            sel = avg;
        }
        out[2 * x] = sel;
        out[2 * x + 1] = avg;
    }
}

/* Raw sample minus its interpolated green, in eighths, at (@x, @y). */
static int cdiff8(IspPipe *p, int x, int y, uint32_t mctrl)
{
    int16_t *g = (int16_t *)row_of(p, S_G, y);

    return 8 * bayer_px(p, S_BF, x, y, mctrl) - g[2 * mirror(x, p->w) + 1];
}

static void rgb_row(IspPipe *p, uint32_t y, uint8_t *out)
{
    uint32_t mctrl = reg(p, A_DEMOSAIC_MATRIX_CTRL);
    int16_t *g = (int16_t *)row_of(p, S_G, y);

    for (int x = 0; x < p->w; x++) {
        int c = colour(p, x, y), v = row_of(p, S_BF, y)[x];
        int rgb[3];

        if (c == C_R || c == C_B) {
            int o = 4 * g[2 * x] + cdiff8(p, x - 1, y - 1, mctrl) +
                    cdiff8(p, x + 1, y - 1, mctrl) +
                    cdiff8(p, x - 1, y + 1, mctrl) +
                    cdiff8(p, x + 1, y + 1, mctrl);

            rgb[1] = clamp8((g[2 * x] + 4) >> 3);
            rgb[c == C_R ? 0 : 2] = v;
            rgb[c == C_R ? 2 : 0] = clamp8((o + 16) >> 5);
        } else {
            int h = 32 * v + 2 * (cdiff8(p, x - 1, y, mctrl) +
                                  cdiff8(p, x + 1, y, mctrl));
            int vv = 32 * v + 2 * (cdiff8(p, x, y - 1, mctrl) +
                                   cdiff8(p, x, y + 1, mctrl));

            rgb[1] = v;
            rgb[0] = clamp8(((c == C_GR ? h : vv) + 16) >> 5);
            rgb[2] = clamp8(((c == C_GR ? vv : h) + 16) >> 5);
        }
        out[3 * x] = rgb[0];
        out[3 * x + 1] = rgb[1];
        out[3 * x + 2] = rgb[2];
    }
}

/* ---- RGB: MEDIAN, WBG, CCM, gamma ---- */

static int rgb_px(IspPipe *p, int x, int y, int ch, uint32_t mctrl)
{
    if (x < 0 || y < 0 || x >= p->w || y >= p->h) {
        if (mctrl & BIT(8)) {
            return extract32(mctrl, 0, 8);
        }
        x = mirror(x, p->w);
    }
    return row_of(p, S_RGB, y)[3 * x + ch];
}

/*
 * The gamma curve of TRM 38.5.2.8: from (0, 0), point n at X(n) = X(n -
 * 1) + 2^GAMMA_X(n), Y(n) its GAMMA_Y; straight between the points, the
 * fraction rounded down. With *_LAST_CORRECT the last interval ends at
 * 255 rather than at its X (256 when the X intervals add up to 256), so
 * that 255 maps to Y(0F).
 */
static int gamma_px(const uint8_t *xp, const uint8_t *yp, bool last, int v)
{
    int x0 = 0, y0 = 0;

    for (int n = 0; n < 16; n++) {
        int len = 1 << (xp[n] & 7), x1 = x0 + len, y1 = yp[n];

        if (v < x1 || n == 15) {
            if (n == 15 && last) {
                int num = (y1 - y0) * (v - x0), den = 255 - x0;

                if (den <= 0) {
                    return y1;
                }
                return clamp8(y0 + (num >= 0 ? num / den
                                             : -((-num + den - 1) / den)));
            }
            return clamp8(y0 + (((y1 - y0) * (v - x0)) >> (xp[n] & 7)));
        }
        x0 = x1;
        y0 = y1;
    }
    return v;
}

/* AE's luminance of a pixel, and RGB2YUV's Y in BT.601 */
static int luma601(const uint8_t *rgb)
{
    return (77 * rgb[0] + 150 * rgb[1] + 29 * rgb[2]) >> 8;
}

static void ae_add(IspPipe *p, uint32_t x, uint32_t y, const uint8_t *rgb)
{
    uint32_t bx = reg(p, A_AE_BX), by = reg(p, A_AE_BY);
    uint32_t xs = extract32(bx, 11, 11), xb = extract32(bx, 0, 11);
    uint32_t ys = extract32(by, 11, 11), yb = extract32(by, 0, 11);
    uint32_t i, j;

    if (!xb || !yb || x < xs || y < ys) {
        return;
    }
    j = (x - xs) / xb;
    i = (y - ys) / yb;
    if (i < 5 && j < 5) {
        p->ae_sum[i * 5 + j] += luma601(rgb);
    }
}

/*
 * AWB algorithm 0: a white patch is a pixel in the window (LPOINT to
 * RPOINT, TPOINT to BPOINT, inclusive) whose R + G + B is within
 * AWB_MIN_LUM..AWB_MAX_LUM and whose R / G and B / G, as R * 256 / G
 * rounded down (1023 for G 0), are within AWB_MIN_RG..AWB_MAX_RG and
 * AWB_MIN_BG..AWB_MAX_BG, all inclusive; the count and the R, G and B
 * sums of those, and the same for each of the 5 x 5 sub-windows from
 * (AWB_X_START, AWB_Y_START) of AWB_X_BSIZE x AWB_Y_BSIZE.
 */
static void awb_add(IspPipe *p, uint32_t x, uint32_t y, const uint8_t *rgb)
{
    uint32_t hs = reg(p, A_AWB_HSCALE), vs = reg(p, A_AWB_VSCALE);
    uint32_t lum = reg(p, A_AWB_TH_LUM), trg = reg(p, A_AWB_TH_RG);
    uint32_t tbg = reg(p, A_AWB_TH_BG);
    uint32_t sum = rgb[0] + rgb[1] + rgb[2];
    uint32_t rg = rgb[1] ? MIN((rgb[0] << 8) / rgb[1], 1023) : 1023;
    uint32_t bg = rgb[1] ? MIN((rgb[2] << 8) / rgb[1], 1023) : 1023;
    uint32_t bx = reg(p, A_AWB_BX), by = reg(p, A_AWB_BY);
    uint32_t xs = extract32(bx, 12, 12), xb = extract32(bx, 0, 12);
    uint32_t ys = extract32(by, 12, 12), yb = extract32(by, 0, 12);

    if (x < extract32(hs, 16, 12) || x > extract32(hs, 0, 12) ||
        y < extract32(vs, 16, 12) || y > extract32(vs, 0, 12)) {
        return;
    }
    if (sum < extract32(lum, 0, 10) || sum > extract32(lum, 16, 10) ||
        rg < extract32(trg, 0, 10) || rg > extract32(trg, 16, 10) ||
        bg < extract32(tbg, 0, 10) || bg > extract32(tbg, 16, 10)) {
        return;
    }
    p->awb_cnt++;
    for (int k = 0; k < 3; k++) {
        p->awb_acc[k] += rgb[k];
    }
    if (xb && yb && x >= xs && y >= ys &&
        (x - xs) / xb < 5 && (y - ys) / yb < 5) {
        uint64_t *s = p->awb_sub[((y - ys) / yb) * 5 + (x - xs) / xb];

        s[0]++;
        for (int k = 0; k < 3; k++) {
            s[k + 1] += rgb[k];
        }
    }
}

/*
 * HIST: the value of each pixel in the 5 x 5 sub-windows of HIST_X_SIZE
 * x HIST_Y_SIZE from (HIST_X_OFFS, HIST_Y_OFFS) falls in bin k where
 * HIST_SEG_(k-1)_k <= value < HIST_SEG_k_(k+1), and adds its
 * sub-window's weight there; each bin is the sum over 256, rounded down
 * (the weights have 8 fractional bits and add up to 256 by ESP-IDF's
 * rules), HIST_SEG_12_13 to _14_15 and HIST_WEIGHT_44 where the v3
 * headers and the TRM place them, in the low bytes of HIST_SEG3 and
 * HIST_WEIGHT6 (ESP-IDF's isp_ll writes those registers as byte arrays,
 * one byte higher). RAW modes count the samples of their Bayer colour
 * after LSC
 * (ESP-IDF's diagram), RGB the demosaic output weighted by
 * HIST_COEFF_R/G/B over 256, YUV RGB2YUV's output.
 */
static uint32_t hist_seg(IspPipe *p, int i)
{
    if (i < 12) {
        return extract32(p->c.r[R(A_HIST_SEG0) + i / 4], 24 - 8 * (i % 4), 8);
    }
    return extract32(p->c.r[R(A_HIST_SEG0) + 3], 16 - 8 * (i - 12), 8);
}

static uint32_t hist_weight(IspPipe *p, int i)
{
    if (i < 24) {
        return extract32(p->c.r[R(A_HIST_WEIGHT0) + i / 4],
                         24 - 8 * (i % 4), 8);
    }
    return extract32(p->c.r[R(A_HIST_WEIGHT0) + 6], 0, 8);
}

static void hist_add(IspPipe *p, uint32_t x, uint32_t y, int v)
{
    uint32_t offs = reg(p, A_HIST_OFFS), size = reg(p, A_HIST_SIZE);
    uint32_t xo = extract32(offs, 16, 12), yo = extract32(offs, 0, 12);
    uint32_t xs = extract32(size, 16, 9), ys = extract32(size, 0, 9);
    uint32_t i, j, sw, bin = 0;

    if (!xs || !ys || x < xo || y < yo) {
        return;
    }
    j = (x - xo) / xs;
    i = (y - yo) / ys;
    if (i >= 5 || j >= 5) {
        return;
    }
    sw = i * 5 + j;
    while (bin < 15 && v >= hist_seg(p, bin)) {
        bin++;
    }
    p->hist[bin] += hist_weight(p, sw);
}

/* The YUV conversions: full range, coefficients over 256, rounded down */
static void rgb_to_yuv(bool bt709, const uint8_t *rgb, int *yuv)
{
    int r = rgb[0], g = rgb[1], b = rgb[2];

    if (bt709) {
        yuv[0] = (54 * r + 183 * g + 19 * b) >> 8;
        yuv[1] = clamp8(((-29 * r - 99 * g + 128 * b) >> 8) + 128);
        yuv[2] = clamp8(((128 * r - 116 * g - 12 * b) >> 8) + 128);
    } else {
        yuv[0] = (77 * r + 150 * g + 29 * b) >> 8;
        yuv[1] = clamp8(((-43 * r - 85 * g + 128 * b) >> 8) + 128);
        yuv[2] = clamp8(((128 * r - 107 * g - 21 * b) >> 8) + 128);
    }
}

static void yuv_to_rgb(bool bt709, int y, int u, int v, uint8_t *rgb)
{
    u -= 128;
    v -= 128;
    if (bt709) {
        rgb[0] = clamp8(y + ((403 * v) >> 8));
        rgb[1] = clamp8(y + ((-48 * u - 120 * v) >> 8));
        rgb[2] = clamp8(y + ((475 * u) >> 8));
    } else {
        rgb[0] = clamp8(y + ((359 * v) >> 8));
        rgb[1] = clamp8(y + ((-88 * u - 183 * v) >> 8));
        rgb[2] = clamp8(y + ((454 * u) >> 8));
    }
}

static void yuv_row(IspPipe *p, uint32_t y, uint8_t *out)
{
    uint32_t mctrl = reg(p, A_MEDIAN_MATRIX_CTRL);
    bool ae = en(p, CNTL_AE_EN), ae_gamma = fld(p, A_AE_CTRL, 1, 1);
    bool awb = en(p, CNTL_AWB_EN) && (fld(p, A_AWB_MODE, 0, 2) & 1);
    bool awb_ccm = fld(p, A_AWB_MODE, 4, 1);
    uint32_t hmode = fld(p, A_HIST_MODE, 0, 3);
    bool hist = en(p, CNTL_HIST_EN);
    bool bt709 = fld(p, A_YUV_FORMAT, 0, 1);
    uint32_t gctrl = reg(p, A_GAMMA_CTRL);
    int16_t ccm[9];
    static const hwaddr ccm_reg[9] = {
        A_CCM_COEF0, A_CCM_COEF0, A_CCM_COEF1, A_CCM_COEF1, A_CCM_COEF3,
        A_CCM_COEF3, A_CCM_COEF4, A_CCM_COEF4, A_CCM_COEF5,
    };

    for (int k = 0; k < 9; k++) {
        uint32_t f = extract32(reg(p, ccm_reg[k]), (k & 1) ? 13 : 0, 13);

        ccm[k] = (f & BIT(12)) ? -(int)(f & 0xfff) : (int)(f & 0xfff);
    }
    for (int x = 0; x < p->w; x++) {
        uint8_t c[3];

        for (int ch = 0; ch < 3; ch++) {
            if (en(p, CNTL_MEDIAN_EN)) {
                int a[9], k = 0;

                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        a[k++] = rgb_px(p, x + dx, (int)y + dy, ch, mctrl);
                    }
                }
                c[ch] = median9(a);
            } else {
                c[ch] = row_of(p, S_RGB, y)[3 * x + ch];
            }
        }
        if (ae && !ae_gamma) {
            ae_add(p, x, y, c);
        }
        if (awb && !awb_ccm) {
            awb_add(p, x, y, c);
        }
        if (hist && hmode == 4) {
            uint32_t k = reg(p, A_HIST_COEFF);

            hist_add(p, x, y, MIN(255, (c[0] * extract32(k, 16, 8) +
                                        c[1] * extract32(k, 8, 8) +
                                        c[2] * extract32(k, 0, 8)) >> 8));
        }
        if (en(p, CNTL_WBG_EN)) {
            static const hwaddr wbg[3] = {
                A_WBG_COEF_R, A_WBG_COEF_G, A_WBG_COEF_B,
            };

            for (int ch = 0; ch < 3; ch++) {
                c[ch] = MIN(255, (c[ch] * fld(p, wbg[ch], 0, 12)) >> 8);
            }
        }
        if (en(p, CNTL_CCM_EN)) {
            int r = c[0], g = c[1], b = c[2];

            for (int ch = 0; ch < 3; ch++) {
                c[ch] = clamp8((ccm[3 * ch] * r + ccm[3 * ch + 1] * g +
                                ccm[3 * ch + 2] * b) >> 8);
            }
        }
        if (awb && awb_ccm) {
            awb_add(p, x, y, c);
        }
        if (en(p, CNTL_GAMMA_EN)) {
            for (int ch = 0; ch < 3; ch++) {
                c[ch] = gamma_px(p->c.gamma_x[ch], p->c.gamma_y[ch],
                                 gctrl & BIT(3 - ch), c[ch]);
            }
        }
        if (ae && ae_gamma) {
            ae_add(p, x, y, c);
        }
        memcpy(out + 3 * x, c, 3);
    }
    if (!en(p, CNTL_RGB2YUV_EN)) {
        return;
    }
    /* YUV422: each pair's U and V its first pixel's (silicon, as above). */
    for (int x = 0; x < p->w; x += 2) {
        int a[3], b[3] = { 0, 0, 0 };

        rgb_to_yuv(bt709, out + 3 * x, a);
        if (x + 1 < p->w) {
            rgb_to_yuv(bt709, out + 3 * (x + 1), b);
            out[3 * x + 3] = b[0];
            out[3 * x + 4] = a[1];
            out[3 * x + 5] = a[2];
        }
        out[3 * x] = a[0];
        out[3 * x + 1] = a[1];
        out[3 * x + 2] = a[2];
    }
    if (hist && hmode >= 5) {
        for (int x = 0; x < p->w; x++) {
            hist_add(p, x, y, out[3 * x + hmode - 5]);
        }
    }
}

/* ---- Sharpen (38.5.2.10), COLOR (38.5.2.11), YUV_Limit and YUV2RGB ---- */

static int y_px(IspPipe *p, int x, int y, uint32_t mctrl)
{
    if (x < 0 || y < 0 || x >= p->w || y >= p->h) {
        if (mctrl & BIT(24)) {
            return extract32(mctrl, 16, 8);
        }
        x = mirror(x, p->w);
    }
    return row_of(p, S_YUV, y)[3 * x];
}

/*
 * Sharpen, on Y: the low-pass L the SHARP_FILTER_COE template's
 * weighted mean (rounded down; Y itself when the weights are all 0),
 * the high-frequency part H = Y - L; |H| below SHARP_THRESHOLD_LOW is
 * taken as 0, below SHARP_THRESHOLD_HIGH multiplied by
 * SHARP_AMOUNT_LOW, from it by SHARP_AMOUNT_HIGH (3.5 fixed point,
 * ESP-IDF's format), rounded toward minus infinity; the output L + H'.
 * SHARP_GRADIENT_MAX is the frame's largest |H|, up to 255.
 */
static int sharp_px(IspPipe *p, int x, int y)
{
    uint32_t mctrl = reg(p, A_SHARP_MATRIX_CTRL), c0 = reg(p, A_SHARP_CTRL0);
    int tl = extract32(c0, 0, 8), th = extract32(c0, 8, 8);
    int al = extract32(c0, 16, 8), ah = extract32(c0, 24, 8);
    int sum = 0, wsum = 0, l, h, a;

    for (int i = 0; i < 3; i++) {
        uint32_t f = reg(p, A_SHARP_FILTER0 + 4 * i);

        for (int j = 0; j < 3; j++) {
            int k = extract32(f, 5 * j, 5);

            sum += k * y_px(p, x + j - 1, y + i - 1, mctrl);
            wsum += k;
        }
    }
    l = wsum ? sum / wsum : y_px(p, x, y, mctrl);
    h = y_px(p, x, y, mctrl) - l;
    p->sharp_max = MAX(p->sharp_max, MIN(abs(h), 255));
    if (abs(h) < tl) {
        return clamp8(l);
    }
    a = abs(h) < th ? al : ah;
    return clamp8(l + ((h * a) >> 5));
}

/*
 * AF, on RGB2YUV's Y (ESP-IDF's diagram): in each window (LPOINT to
 * RPOINT, TPOINT to BPOINT, inclusive) LUM is the sum of Y, and SUM the
 * sum of the gradients |Y(x+1) - Y(x-1)| + |Y(y+1) - Y(y-1)| that reach
 * the edge threshold (AF_THRESHOLD, or with it 0 the generated one).
 */
static void af_add(IspPipe *p, int x, int y)
{
    static const hwaddr hreg[3] = {
        A_AF_HSCALE_A, A_AF_HSCALE_A + 8, A_AF_HSCALE_A + 16,
    };
    uint32_t mctrl = reg(p, A_SHARP_MATRIX_CTRL);
    int yv = y_px(p, x, y, mctrl);
    int g = abs(y_px(p, x + 1, y, mctrl) - y_px(p, x - 1, y, mctrl)) +
            abs(y_px(p, x, y + 1, mctrl) - y_px(p, x, y - 1, mctrl));

    for (int k = 0; k < 3; k++) {
        uint32_t hs = reg(p, hreg[k]), vs = reg(p, hreg[k] + 4);

        if (x < extract32(hs, 16, 12) || x > extract32(hs, 0, 12) ||
            y < extract32(vs, 16, 12) || y > extract32(vs, 0, 12)) {
            continue;
        }
        p->af_lum[k] += yv;
        p->af_hist[g]++;
        if (g >= p->c.af_threshold) {
            p->af_sum[k] += g;
        }
    }
}

/* Q8 sines of 0 to 90 degrees */
static const uint16_t sin_q8[91] = {
    0, 4, 9, 13, 18, 22, 27, 31, 36, 40, 44, 49, 53, 58, 62, 66, 71, 75, 79,
    83, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 139,
    143, 147, 150, 154, 158, 161, 165, 168, 171, 175, 178, 181, 184, 187,
    190, 193, 196, 199, 202, 204, 207, 210, 212, 215, 217, 219, 222, 224,
    226, 228, 230, 232, 234, 236, 237, 239, 241, 242, 243, 245, 246, 247,
    248, 249, 250, 251, 252, 253, 254, 254, 255, 255, 255, 256, 256, 256,
    256,
};

static int sin_deg(int d)
{
    d %= 360;
    if (d < 0) {
        d += 360;
    }
    if (d <= 90) {
        return sin_q8[d];
    }
    if (d <= 180) {
        return sin_q8[180 - d];
    }
    if (d <= 270) {
        return -sin_q8[d - 180];
    }
    return -sin_q8[360 - d];
}

/*
 * COLOR, on full-range YUV: Y' = (Y - 128) * CONTRAST / 128 + 128 +
 * BRIGHTNESS; U and V, centred on 128, rotated by the hue angle (in
 * degrees, 9 bits, taken modulo 360; positive turns U toward V) with Q8
 * sines and cosines, and scaled by SATURATION / 128 (contrast and
 * saturation have 7 fractional bits); each product rounded down.
 */
static void color_px(IspPipe *p, int *yuv)
{
    uint32_t cc = reg(p, A_COLOR_CTRL);
    int sat = extract32(cc, 0, 8), con = extract32(cc, 16, 8);
    int bri = (int8_t)extract32(cc, 24, 8);
    int hue = extract32(cc, 8, 8) | fld(p, A_COLOR_HUE_CTRL, 0, 1) << 8;
    int s = sin_deg(hue), c = sin_deg(hue + 90);
    int u = yuv[1] - 128, v = yuv[2] - 128;

    yuv[0] = clamp8((((yuv[0] - 128) * con) >> 7) + 128 + bri);
    yuv[1] = clamp8(128 + (((u * c - v * s) * sat) >> 15));
    yuv[2] = clamp8(128 + (((u * s + v * c) * sat) >> 15));
}

static void emit_row(IspPipe *p, uint32_t y, const uint8_t *rgb_or_yuv,
                     bool yuv, const uint8_t *raw);

static void out_row(IspPipe *p, uint32_t y)
{
    uint8_t *row = row_of(p, S_YUV, y);
    bool yuv = en(p, CNTL_RGB2YUV_EN);
    bool bt709 = fld(p, A_YUV_FORMAT, 0, 1);
    bool limit = yuv && !en(p, CNTL_YUV2RGB_EN) && fld(p, A_YUV_FORMAT, 1, 1);
    g_autofree uint8_t *line = g_malloc(3 * p->w);

    if (!yuv) {
        memcpy(line, row, 3 * p->w);
        if ((en(p, CNTL_SHARP_EN) || en(p, CNTL_COLOR_EN)) &&
            !warned_mismatch) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: SHARP and COLOR work on "
                          "YUV, and RGB2YUV is disabled\n", TYPE_ESP32P4_ISP);
            warned_mismatch = true;
        }
        emit_row(p, y, line, false, NULL);
        return;
    }
    for (int x = 0; x < p->w; x++) {
        int v[3] = { row[3 * x], row[3 * x + 1], row[3 * x + 2] };

        if (en(p, CNTL_AF_EN)) {
            af_add(p, x, y);
        }
        if (en(p, CNTL_SHARP_EN)) {
            v[0] = sharp_px(p, x, y);
        }
        if (en(p, CNTL_COLOR_EN)) {
            color_px(p, v);
        }
        if (limit) {
            v[0] = ((v[0] * 219) >> 8) + 16;
            v[1] = (((v[1] - 128) * 224) >> 8) + 128;
            v[2] = (((v[2] - 128) * 224) >> 8) + 128;
        }
        if (en(p, CNTL_YUV2RGB_EN)) {
            yuv_to_rgb(bt709, v[0], v[1], v[2], line + 3 * x);
        } else {
            line[3 * x] = v[0];
            line[3 * x + 1] = v[1];
            line[3 * x + 2] = v[2];
        }
    }
    emit_row(p, y, line, !en(p, CNTL_YUV2RGB_EN), NULL);
}

/* ---- CROP (38.5.2.13) and ISP_Tail (38.5.3, table 38.5-2) ---- */

/*
 * The output row in ISP_OUT_TYPE's layout: RAW8 the samples; RGB888 B,
 * G, R; RGB565 little-endian, R in bits 15:11, each component's top
 * bits; YUV422 U Y0 V Y1 (the pair's chroma, its first pixel's); YUV420
 * U Y0 Y1 on the frame's even output rows and V Y0 Y1 on odd ones, each
 * row's own chroma. A type the pipeline's domain does not give (the
 * TRM asks for demosaic, RGB2YUV and YUV2RGB to match it) is logged and
 * converted: YUV and RGB by the YUV_FORMAT formulas, RAW as grey.
 */
static void emit_row(IspPipe *p, uint32_t y, const uint8_t *px, bool yuv,
                     const uint8_t *raw)
{
    uint32_t type = fld(p, A_CNTL, 29, 3), x0 = 0, x1 = p->w - 1, n = 0;
    uint32_t oy = y;
    bool bt709 = fld(p, A_YUV_FORMAT, 0, 1);
    uint8_t *o = p->out;

    if (p->crop) {
        if (y < p->cy0 || y > p->cy1) {
            p->emit(p->opaque, NULL, 0);
            return;
        }
        x0 = p->cx0;
        x1 = p->cx1;
        oy = y - p->cy0;
    }
    if (p->no_output) {
        p->emit(p->opaque, NULL, 0);
        return;
    }
    if (!warned_mismatch &&
        ((raw && type != OUT_RAW8) || (!raw && type == OUT_RAW8) ||
         (yuv && (type == OUT_RGB888 || type == OUT_RGB565)) ||
         (!raw && !yuv && (type == OUT_YUV422 || type == OUT_YUV420)) ||
         type > OUT_RGB565)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ISP_OUT_TYPE %u does not match "
                      "the pipeline's modules; converted\n",
                      TYPE_ESP32P4_ISP, type);
        warned_mismatch = true;
    }
    for (uint32_t x = x0; x <= x1; x++) {
        uint8_t rgb[3];
        int v[3];

        if (raw) {
            v[0] = v[1] = v[2] = raw[x];
            rgb[0] = rgb[1] = rgb[2] = raw[x];
            if (type != OUT_RAW8) {
                rgb_to_yuv(bt709, rgb, v);
            }
        } else if (yuv) {
            v[0] = px[3 * x];
            v[1] = px[3 * x + 1];
            v[2] = px[3 * x + 2];
            yuv_to_rgb(bt709, v[0], v[1], v[2], rgb);
        } else {
            memcpy(rgb, px + 3 * x, 3);
            rgb_to_yuv(bt709, rgb, v);
            if (x & 1) {
                int first[3];

                rgb_to_yuv(bt709, px + 3 * (x & ~1), first);
                v[1] = first[1];
                v[2] = first[2];
            }
        }
        switch (type) {
        case OUT_RAW8:
            o[n++] = raw ? raw[x] : v[0];
            break;
        case OUT_YUV422:
            if (!((x - x0) & 1)) {
                o[n++] = v[1];
                o[n++] = v[0];
                o[n++] = v[2];
            } else {
                o[n++] = v[0];
            }
            break;
        case OUT_YUV420:
            if (!((x - x0) & 1)) {
                o[n++] = (oy & 1) ? v[2] : v[1];
            }
            o[n++] = v[0];
            break;
        case OUT_RGB565: {
            uint16_t w = (rgb[0] >> 3) << 11 | (rgb[1] >> 2) << 5 |
                         rgb[2] >> 3;

            o[n++] = w;
            o[n++] = w >> 8;
            break;
        }
        default:
            o[n++] = rgb[2];
            o[n++] = rgb[1];
            o[n++] = rgb[0];
            break;
        }
    }
    p->stats.out_rows++;
    p->emit(p->opaque, o, n);
}

/* ---- The stages' order ---- */

static void produce(IspPipe *p, int s, uint32_t y)
{
    uint8_t *dst = s == S_OUT ? NULL : row_of(p, s, y);

    switch (s) {
    case S_DPC:
        dpc_row(p, y, dst);
        break;
    case S_BF:
        bf_row(p, y, dst);
        break;
    case S_G:
        g_row(p, y, (int16_t *)dst);
        break;
    case S_RGB:
        rgb_row(p, y, dst);
        break;
    case S_YUV:
        yuv_row(p, y, dst);
        break;
    case S_OUT:
        if (p->st[S_G].used) {
            out_row(p, y);
        } else {
            emit_row(p, y, NULL, false, row_of(p, S_BF, y));
        }
        break;
    default:
        g_assert_not_reached();
    }
}

/* Whether stage @s can complete its next row now. */
static bool ready(IspPipe *p, int s)
{
    Stage *t = &p->st[s];
    uint32_t y = t->next;

    if (!t->used || y >= p->h) {
        return false;
    }
    for (int i = 0; i < t->nin; i++) {
        if (p->st[t->in[i]].next < MIN(p->h, y + t->radius[i] + 1)) {
            return false;
        }
    }
    /* The row it overwrites must be one no later stage still reads. */
    for (int c = s + 1; c < S_NUM; c++) {
        Stage *u = &p->st[c];

        for (int i = 0; u->used && i < u->nin; i++) {
            if (u->in[i] == s && u->next < p->h &&
                (int)y - RING_ROWS >= 0 &&
                (int)y - RING_ROWS >= (int)u->next - (int)u->radius[i]) {
                return false;
            }
        }
    }
    return true;
}

static void pump(IspPipe *p)
{
    bool progress;

    do {
        progress = false;
        for (int s = S_NUM - 1; s > S_IN; s--) {
            if (ready(p, s)) {
                produce(p, s, p->st[s].next++);
                progress = true;
            }
        }
    } while (progress);
}

static void link_stage(IspPipe *p, int s, int in, uint32_t radius)
{
    Stage *t = &p->st[s];

    t->in[t->nin] = in;
    t->radius[t->nin] = radius;
    t->nin++;
}

uint32_t isp_pipe_latency_rows(const IspFrameCfg *cfg)
{
    uint32_t cntl = cfg->r[R(A_CNTL)], rows = 0;

    rows += (cntl & CNTL_DPC_EN) ? 2 : 0;
    rows += (cntl & CNTL_BF_EN) ? 2 : 0;
    if (cntl & CNTL_DEMOSAIC_EN) {
        rows += 3;
        rows += (cntl & CNTL_MEDIAN_EN) ? 1 : 0;
        rows += (cntl & (CNTL_SHARP_EN | CNTL_AF_EN)) ? 1 : 0;
    }
    return rows;
}

IspPipe *isp_pipe_new(const IspFrameCfg *cfg, IspEmitFn *emit, void *opaque)
{
    IspPipe *p = g_new0(IspPipe, 1);
    uint32_t cntl = cfg->r[R(A_CNTL)];
    uint32_t mode = extract32(cfg->r[R(A_FRAME_CFG)], 27, 2);
    /* The 2 x 2 cell by BAYER_MODE: BG/GR, GB/RG, GR/BG, RG/GB */
    static const uint8_t cells[4][4] = {
        { C_B, C_GB, C_GR, C_R }, { C_GB, C_B, C_R, C_GR },
        { C_GR, C_R, C_B, C_GB }, { C_R, C_GR, C_GB, C_B },
    };
    static const uint32_t bytes[S_NUM] = { 1, 1, 1, 4, 3, 3, 0 };
    uint32_t xc = cfg->r[R(A_CROP_X_CAPTURE)];
    uint32_t yc = cfg->r[R(A_CROP_Y_CAPTURE)];

    p->c = *cfg;
    p->emit = emit;
    p->opaque = opaque;
    p->w = cfg->w;
    p->h = cfg->h;
    memcpy(p->col, cells[mode], 4);
    p->out = g_malloc(3 * p->w + 8);

    p->st[S_IN].used = p->st[S_DPC].used = p->st[S_BF].used = true;
    p->st[S_OUT].used = true;
    link_stage(p, S_DPC, S_IN, (cntl & CNTL_DPC_EN) ? 2 : 0);
    link_stage(p, S_BF, S_DPC, (cntl & CNTL_BF_EN) ? 2 : 0);
    if (cntl & CNTL_DEMOSAIC_EN) {
        p->st[S_G].used = p->st[S_RGB].used = p->st[S_YUV].used = true;
        link_stage(p, S_G, S_BF, 2);
        link_stage(p, S_RGB, S_G, 1);
        link_stage(p, S_RGB, S_BF, 1);
        link_stage(p, S_YUV, S_RGB, (cntl & CNTL_MEDIAN_EN) ? 1 : 0);
        link_stage(p, S_OUT, S_YUV,
                   (cntl & (CNTL_SHARP_EN | CNTL_AF_EN)) ? 1 : 0);
    } else {
        link_stage(p, S_OUT, S_BF, 0);
    }
    for (int s = 0; s < S_NUM; s++) {
        p->st[s].row_bytes = bytes[s] * p->w;
        if (p->st[s].used && bytes[s]) {
            p->st[s].buf = g_malloc0(RING_ROWS * p->st[s].row_bytes);
        }
    }

    if ((cntl & CNTL_DPC_EN) &&
        (cfg->r[R(A_DPC_CTRL)] & BIT(0)) &&
        !(cfg->r[R(A_DPC_CTRL)] & BIT(5))) {
        p->no_output = true;
    }
    if (cntl & CNTL_CROP_EN) {
        uint32_t xs = extract32(xc, 0, 12), xe = extract32(xc, 12, 12);
        uint32_t ys = extract32(yc, 0, 12), ye = extract32(yc, 12, 12);
        uint32_t err = 0;

        err |= (ye >= p->h || ys > ye) ? BIT(0) : 0;
        err |= (xe >= p->w || xs > xe) ? BIT(1) : 0;
        err |= !(ye & 1) ? BIT(2) : 0;
        err |= !(xe & 1) ? BIT(3) : 0;
        err |= (ys & 1) ? BIT(4) : 0;
        err |= (xs & 1) ? BIT(5) : 0;
        p->stats.crop_err = err;
        if (!err) {
            p->crop = true;
            p->cx0 = xs;
            p->cx1 = xe;
            p->cy0 = ys;
            p->cy1 = ye;
        }
    }
    p->stats.out_w = p->crop ? p->cx1 - p->cx0 + 1 : p->w;
    return p;
}

void isp_pipe_row(IspPipe *p, const uint8_t *row)
{
    if (p->st[S_IN].next >= p->h) {
        return;
    }
    blc_row(p, p->st[S_IN].next, row, row_of(p, S_IN, p->st[S_IN].next));
    p->st[S_IN].next++;
    pump(p);
}

void isp_pipe_finish(IspPipe *p)
{
    IspStats *st = &p->stats;
    g_autofree uint8_t *zero = g_malloc0(p->w);
    uint32_t recip = p->c.r[R(A_AE_WIN_RECIPROCAL)] & 0xfffff;
    uint32_t ctl = p->c.r[R(A_AF_CTRL1)] & 0x3fffff;
    uint32_t gen = p->c.r[R(A_AF_GEN_TH_CTRL)];

    while (p->st[S_IN].next < p->h) {
        isp_pipe_row(p, zero);
    }
    pump(p);

    for (int n = 0; n < 4; n++) {
        st->blc_mean[n] = p->blc_cnt[n] ? p->blc_sum[n] / p->blc_cnt[n] : 0;
    }
    for (int i = 0; i < 25; i++) {
        st->ae_mean[i] = MIN(255, (p->ae_sum[i] * recip) >> 20);
    }
    for (int k = 0; k < 3; k++) {
        st->af_sum[k] = MIN(p->af_sum[k], 0x3fffffff);
        st->af_lum[k] = MIN(p->af_lum[k], 0x0fffffff);
    }
    st->af_gen_threshold = p->c.af_threshold;
    if (ctl) {
        uint32_t above = 0, t = 511;

        /* The least threshold that leaves at most THPIXNUM gradients over. */
        while (t > 0 && above + p->af_hist[t] <= ctl) {
            above += p->af_hist[t];
            t--;
        }
        t = MAX(t + 1, extract32(gen, 0, 16));
        st->af_gen_threshold = MIN(t, extract32(gen, 16, 16));
    }
    st->awb_cnt = MIN(p->awb_cnt, 0xffffff);
    for (int k = 0; k < 3; k++) {
        st->awb_acc[k] = MIN(p->awb_acc[k], UINT32_MAX);
    }
    for (int i = 0; i < 25; i++) {
        for (int k = 0; k < 4; k++) {
            st->awb_sub[i][k] = MIN(p->awb_sub[i][k], UINT32_MAX);
        }
    }
    for (int k = 0; k < 16; k++) {
        st->hist[k] = MIN(p->hist[k] >> 8, 0x1ffff);
    }
    st->sharp_max = p->sharp_max;
    st->dpc_dead = MIN(p->dpc_dead, 0x3ff);
}

const IspStats *isp_pipe_stats(const IspPipe *p)
{
    return &p->stats;
}

void isp_pipe_free(IspPipe *p)
{
    if (!p) {
        return;
    }
    for (int s = 0; s < S_NUM; s++) {
        g_free(p->st[s].buf);
    }
    g_free(p->out);
    g_free(p);
}
