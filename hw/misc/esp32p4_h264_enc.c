/*
 * ESP32-P4 H.264 encoder core (ENC_CORE): a baseline-profile CAVLC encoder
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), section
 * 41.5.1, for what the core does; ITU-T H.264 (Advanced video coding for
 * generic audiovisual services) for the syntax and the decoding process
 * the core's reconstruction must match.
 *
 * The core codes what the TRM lists for the hardware (41.3): baseline
 * profile, 4:2:0, CAVLC, I and P pictures, intra 4x4 with all nine modes
 * and intra 16x16 with all four, the four chroma intra modes, quarter-
 * sample motion within [-29.75, +16.75] horizontally and [-13.75, +13.75]
 * vertically, P_Skip, intra macroblocks in P slices, the deblocking
 * filter, decimation of the quantized levels, the macroblock-level rate
 * control and the ROI's QP adjustment. It need not make the same bytes as
 * silicon; it makes a valid bitstream that decodes to its own
 * reconstruction, which is what it writes out as the next reference.
 * The motion search is a simple one: predictor candidates, a small
 * diamond, then half- and quarter-sample refinement, each judged by its
 * SAD or SATD and the bits of its motion vector difference; inter
 * macroblocks are coded as one 16x16 partition (P_L0_16x16) or skipped,
 * so the partition sizes below 16x16 the hardware may choose are not used.
 * The search range is the hardware's, so that the macroblocks a vector
 * reaches are those the hardware's three-row reference window holds: a
 * 16 x 16 block displaced by up to 13.75 lines, with the six-tap filter's
 * extra lines, stays within the row above and the row below.
 *
 * Interpretations where the TRM is silent:
 *
 * - Subsampling to 4:2:0 (41.5.1.2) takes each 2 x 2 block's Cb from its
 *   top left sample and Cr from its bottom left, as the hardware's own
 *   4:2:0 layout (Cb on even lines, Cr on odd) carries them; grey has
 *   both at 128.
 * - Decimation (41.5.1.3) scores levels as x264, which Espressif's
 *   default thresholds (6 for luma, 7 for chroma) follow: per 4x4 block,
 *   9 if any level exceeds 1 in magnitude, else the sum over its +-1
 *   levels of {3, 2, 2, 1, 1, 1, 0...}[zeros before it in scan order].
 *   It zeroes an inter macroblock's luma levels, an intra 16x16's luma AC
 *   levels, and the chroma AC levels; DC levels are kept.
 * - The rate control (41.5.1.5) in P pictures: a macroblock's budget is
 *   RATE_CTRL_U * 15 * MAD_FRAME_PRED / 256 bits, inverting the formula
 *   Espressif's driver computes U from (U = 256 * target bits per
 *   macroblock / predicted MAD / 15); each macroblock's QP is the
 *   picture's plus 6 * log2 of the bits spent so far over the budget so
 *   far (each with one macroblock's budget added), within QP_MIN and
 *   QP_MAX. I pictures keep the picture's QP.
 * - The ROI's QP fields are sign and magnitude, as the TRM's text has
 *   them (bit 6 the sign).
 * - FRAME_MAD_SUM sums each macroblock's mean absolute luma residual
 *   after prediction (its SAD over 256, rounded down), in I and P
 *   pictures; FRAME_ENC_BITS counts the macroblock layer's bits; and
 *   FRAME_QP_SUM each macroblock's QP as decoded (a macroblock without
 *   residual keeps its predecessor's).
 * - A merged motion vector (41.5.1.4) is the 16x16 one for all three
 *   merge types, as no smaller partitions are used; skipped macroblocks
 *   count as P macroblocks of 16 x 16.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "hw/misc/esp32p4_h264_enc.h"

/* The hardware's motion vector range, in quarter samples. */
#define MV_X_MIN    (-119)
#define MV_X_MAX    67
#define MV_Y_MIN    (-55)
#define MV_Y_MAX    55
/* ... and in whole samples. */
#define INT_X_MIN   (-29)
#define INT_X_MAX   16
#define INT_Y_MIN   (-13)
#define INT_Y_MAX   13

/* The lowest QP the hardware codes with (TRM 41.5.1.6). */
#define QP_LOWEST   10

/* The 4x4 blocks in zig-zag scan order, as raster positions. */
static const uint8_t zigzag[16] = {
    0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};

/* luma4x4BlkIdx to raster block position (x + 4 * y). */
static const uint8_t blk_raster[16] = {
    0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15,
};

/* Raster block position to luma4x4BlkIdx. */
static const uint8_t blk_zorder[16] = {
    0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15,
};

/* QPC from qPI (Table 8-15). */
static const uint8_t chroma_qp[52] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34,
    35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 39, 39,
};

/*
 * The forward quantizer's multipliers and the scaling the decoding
 * process applies, for QP % 6, by position: both even, both odd, mixed.
 */
static const int quant_mf[6][3] = {
    { 13107, 5243, 8066 }, { 11916, 4660, 7490 }, { 10082, 4194, 6554 },
    { 9362, 3647, 5825 }, { 8192, 3355, 5243 }, { 7282, 2893, 4559 },
};
static const int dequant_v[6][3] = {
    { 10, 16, 13 }, { 11, 18, 14 }, { 13, 20, 16 },
    { 14, 23, 18 }, { 16, 25, 20 }, { 18, 29, 23 },
};

static int pos_class(int p)
{
    int x = p & 3, y = p >> 2;

    if (!(x & 1) && !(y & 1)) {
        return 0;
    }
    return (x & 1) && (y & 1) ? 1 : 2;
}

/* coded_block_pattern to codeNum (Table 9-4), intra and inter. */
static const uint8_t cbp_code_intra[48] = {
    3, 29, 30, 17, 31, 18, 37, 8, 32, 38, 19, 9, 20, 10, 11, 2,
    16, 33, 34, 21, 35, 22, 39, 4, 36, 40, 23, 5, 24, 6, 7, 1,
    41, 42, 43, 25, 44, 26, 46, 12, 45, 47, 27, 13, 28, 14, 15, 0,
};
static const uint8_t cbp_code_inter[48] = {
    0, 2, 3, 7, 4, 8, 17, 13, 5, 18, 9, 14, 10, 15, 16, 11,
    1, 32, 33, 36, 34, 37, 44, 40, 35, 45, 38, 41, 39, 42, 43, 19,
    6, 24, 25, 20, 26, 21, 46, 28, 27, 47, 22, 29, 23, 30, 31, 12,
};

/*
 * coeff_token (Table 9-5): length and code by nC's table, TotalCoeff and
 * TrailingOnes, [4 * TotalCoeff + TrailingOnes].
 */
static const uint8_t ct_len[3][4 * 17] = {
    {
        1, 0, 0, 0, 6, 2, 0, 0, 8, 6, 3, 0, 9, 8, 7, 5, 10, 9, 8, 6,
        11, 10, 9, 7, 13, 11, 10, 8, 13, 13, 11, 9, 13, 13, 13, 10,
        14, 14, 13, 11, 14, 14, 14, 13, 15, 15, 14, 14, 15, 15, 15, 14,
        16, 15, 15, 15, 16, 16, 16, 15, 16, 16, 16, 16, 16, 16, 16, 16,
    }, {
        2, 0, 0, 0, 6, 2, 0, 0, 6, 5, 3, 0, 7, 6, 6, 4, 8, 6, 6, 4,
        8, 7, 7, 5, 9, 8, 8, 6, 11, 9, 9, 6, 11, 11, 11, 7,
        12, 11, 11, 9, 12, 12, 12, 11, 12, 12, 12, 11, 13, 13, 13, 12,
        13, 13, 13, 13, 13, 14, 13, 13, 14, 14, 14, 13, 14, 14, 14, 14,
    }, {
        4, 0, 0, 0, 6, 4, 0, 0, 6, 5, 4, 0, 6, 5, 5, 4, 7, 5, 5, 4,
        7, 5, 5, 4, 7, 6, 6, 4, 7, 6, 6, 4, 8, 7, 7, 5,
        8, 8, 7, 6, 9, 8, 8, 7, 9, 9, 8, 8, 9, 9, 9, 8,
        10, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    },
};
static const uint8_t ct_code[3][4 * 17] = {
    {
        1, 0, 0, 0, 5, 1, 0, 0, 7, 4, 1, 0, 7, 6, 5, 3, 7, 6, 5, 3,
        7, 6, 5, 4, 15, 6, 5, 4, 11, 14, 5, 4, 8, 10, 13, 4,
        15, 14, 9, 4, 11, 10, 13, 12, 15, 14, 9, 12, 11, 10, 13, 8,
        15, 1, 9, 12, 11, 14, 13, 8, 7, 10, 9, 12, 4, 6, 5, 8,
    }, {
        3, 0, 0, 0, 11, 2, 0, 0, 7, 7, 3, 0, 7, 10, 9, 5, 7, 6, 5, 4,
        4, 6, 5, 6, 7, 6, 5, 8, 15, 6, 5, 4, 11, 14, 13, 4,
        15, 10, 9, 4, 11, 14, 13, 12, 8, 10, 9, 8, 15, 14, 13, 12,
        11, 10, 9, 12, 7, 11, 6, 8, 9, 8, 10, 1, 7, 6, 5, 4,
    }, {
        15, 0, 0, 0, 15, 14, 0, 0, 11, 15, 13, 0, 8, 12, 14, 12,
        15, 10, 11, 11, 11, 8, 9, 10, 9, 14, 13, 9, 8, 10, 9, 8,
        15, 14, 13, 13, 11, 14, 10, 12, 15, 10, 13, 12, 11, 14, 9, 12,
        8, 10, 13, 8, 13, 7, 9, 12, 9, 12, 11, 10, 5, 8, 7, 6,
        1, 4, 3, 2,
    },
};
/* The chroma DC table (nC = -1). */
static const uint8_t ct_dc_len[4 * 5] = {
    2, 0, 0, 0, 6, 1, 0, 0, 6, 6, 3, 0, 6, 7, 7, 6, 6, 8, 8, 7,
};
static const uint8_t ct_dc_code[4 * 5] = {
    1, 0, 0, 0, 7, 1, 0, 0, 4, 6, 1, 0, 3, 3, 2, 5, 2, 3, 2, 0,
};

/* total_zeros for 4x4 blocks (Tables 9-7, 9-8), [TotalCoeff - 1][zeros]. */
static const uint8_t tz_len[15][16] = {
    { 1, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9 },
    { 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 6, 6 },
    { 4, 3, 3, 3, 4, 4, 3, 3, 4, 5, 5, 6, 5, 6 },
    { 5, 3, 4, 4, 3, 3, 3, 4, 3, 4, 5, 5, 5 },
    { 4, 4, 4, 3, 3, 3, 3, 3, 4, 5, 4, 5 },
    { 6, 5, 3, 3, 3, 3, 3, 3, 4, 3, 6 },
    { 6, 5, 3, 3, 3, 2, 3, 4, 3, 6 },
    { 6, 4, 5, 3, 2, 2, 3, 3, 6 },
    { 6, 6, 4, 2, 2, 3, 2, 5 },
    { 5, 5, 3, 2, 2, 2, 4 },
    { 4, 4, 3, 3, 1, 3 },
    { 4, 4, 2, 1, 3 },
    { 3, 3, 1, 2 },
    { 2, 2, 1 },
    { 1, 1 },
};
static const uint8_t tz_code[15][16] = {
    { 1, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 1 },
    { 7, 6, 5, 4, 3, 5, 4, 3, 2, 3, 2, 3, 2, 1, 0 },
    { 5, 7, 6, 5, 4, 3, 4, 3, 2, 3, 2, 1, 1, 0 },
    { 3, 7, 5, 4, 6, 5, 4, 3, 3, 2, 2, 1, 0 },
    { 5, 4, 3, 7, 6, 5, 4, 3, 2, 1, 1, 0 },
    { 1, 1, 7, 6, 5, 4, 3, 2, 1, 1, 0 },
    { 1, 1, 5, 4, 3, 3, 2, 1, 1, 0 },
    { 1, 1, 1, 3, 3, 2, 2, 1, 0 },
    { 1, 0, 1, 3, 2, 1, 1, 1 },
    { 1, 0, 1, 3, 2, 1, 1 },
    { 0, 1, 1, 2, 1, 3 },
    { 0, 1, 1, 1, 1 },
    { 0, 1, 1, 1 },
    { 0, 1, 1 },
    { 0, 1 },
};
/* total_zeros for the 2x2 chroma DC (Table 9-9a). */
static const uint8_t tz_dc_len[3][4] = {
    { 1, 2, 3, 3 }, { 1, 2, 2 }, { 1, 1 },
};
static const uint8_t tz_dc_code[3][4] = {
    { 1, 1, 1, 0 }, { 1, 1, 0 }, { 1, 0 },
};
/* run_before (Table 9-10), [min(zerosLeft, 7) - 1][run_before]. */
static const uint8_t rb_len[7][15] = {
    { 1, 1 },
    { 1, 2, 2 },
    { 2, 2, 2, 2 },
    { 2, 2, 2, 3, 3 },
    { 2, 2, 3, 3, 3, 3 },
    { 2, 3, 3, 3, 3, 3, 3 },
    { 3, 3, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
};
static const uint8_t rb_code[7][15] = {
    { 1, 0 },
    { 1, 1, 0 },
    { 3, 2, 1, 0 },
    { 3, 2, 1, 1, 0 },
    { 3, 2, 3, 2, 1, 0 },
    { 3, 0, 1, 3, 2, 5, 4 },
    { 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
};

/* The deblocking filter's thresholds by indexA, indexB (Table 8-16, 8-17). */
static const uint8_t db_alpha[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 5, 6,
    7, 8, 9, 10, 12, 13, 15, 17, 20, 22, 25, 28, 32, 36, 40, 45,
    50, 56, 63, 71, 80, 90, 101, 113, 127, 144, 162, 182, 203, 226, 255, 255,
};
static const uint8_t db_beta[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3,
    3, 3, 3, 4, 4, 4, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10,
    11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
};
static const uint8_t db_tc0[52][3] = {
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 1 }, { 0, 0, 1 }, { 0, 0, 1 },
    { 0, 0, 1 }, { 0, 1, 1 }, { 0, 1, 1 }, { 1, 1, 1 }, { 1, 1, 1 },
    { 1, 1, 1 }, { 1, 1, 1 }, { 1, 1, 2 }, { 1, 1, 2 }, { 1, 1, 2 },
    { 1, 1, 2 }, { 1, 2, 3 }, { 1, 2, 3 }, { 2, 2, 3 }, { 2, 2, 4 },
    { 2, 3, 4 }, { 2, 3, 4 }, { 3, 3, 5 }, { 3, 4, 6 }, { 3, 4, 6 },
    { 4, 5, 7 }, { 4, 5, 8 }, { 4, 6, 9 }, { 5, 7, 10 }, { 6, 8, 11 },
    { 6, 8, 13 }, { 7, 10, 14 }, { 8, 11, 16 }, { 9, 12, 18 },
    { 10, 13, 20 }, { 11, 15, 23 }, { 13, 17, 25 },
};

/* The cost of a bit, in SAD units, by QP: 0.85 * 2^((QP - 12) / 6). */
static const uint8_t lambda_tab[52] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
    2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 17, 19,
    22, 24, 27, 31, 34, 38, 43, 48, 54, 61, 69, 77,
};

/* 64 * 2^(k / 6) for k from -36 to 36, the rate control's QP steps. */
static const uint16_t rc_ratio[73] = {
    1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 7, 8, 9, 10, 11,
    13, 14, 16, 18, 20, 23, 25, 29, 32, 36, 40, 45, 51, 57, 64, 72, 81,
    91, 102, 114, 128, 144, 161, 181, 203, 228, 256, 287, 323, 362, 406,
    456, 512, 575, 645, 724, 813, 912, 1024, 1149, 1290, 1448, 1625,
    1825, 2048, 2299, 2580, 2896, 3251, 3649, 4096,
};

/* Decimation's score of a +-1 level by the zeros before it. */
static const uint8_t deci_table[16] = {
    3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static inline int clip3(int lo, int hi, int v)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline uint8_t clip1(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* Bit writing */

static void emit_byte(H264Enc *e, uint8_t b)
{
    if (e->rbsp_len == e->rbsp_cap) {
        e->rbsp_cap = MAX(e->rbsp_cap * 2, 4096);
        e->rbsp = g_realloc(e->rbsp, e->rbsp_cap);
    }
    e->rbsp[e->rbsp_len++] = b;
}

static void put_bits(H264Enc *e, int n, uint32_t v)
{
    uint64_t a;
    int bits;

    if (!n) {
        return;
    }
    a = ((uint64_t)e->acc << n) | (v & (uint32_t)((1ull << n) - 1));
    bits = e->acc_bits + n;
    while (bits >= 8) {
        emit_byte(e, a >> (bits - 8));
        bits -= 8;
    }
    e->acc = a & ((1u << bits) - 1);
    e->acc_bits = bits;
    e->bits += n;
}

static int ue_bits(uint32_t k)
{
    return 2 * (31 - clz32(k + 1)) + 1;
}

static int se_bits(int v)
{
    return ue_bits(v > 0 ? 2 * v - 1 : -2 * v);
}

static void put_ue(H264Enc *e, uint32_t k)
{
    int len = ue_bits(k);

    if (len > 32) {
        put_bits(e, len - 32, 0);
        len = 32;
    }
    put_bits(e, len, k + 1);
}

static void put_se(H264Enc *e, int v)
{
    put_ue(e, v > 0 ? 2 * v - 1 : -2 * v);
}

size_t h264e_pending(H264Enc *e)
{
    return e->rbsp_len;
}

size_t h264e_take(H264Enc *e, uint8_t *buf, size_t max)
{
    size_t in = 0, out = 0;

    while (out < max && in < e->rbsp_len) {
        uint8_t b = e->rbsp[in];

        if (e->zeros >= 2 && b <= 3) {
            buf[out++] = 3;
            e->zeros = 0;
            continue;
        }
        buf[out++] = b;
        e->zeros = b ? 0 : e->zeros + 1;
        in++;
    }
    memmove(e->rbsp, e->rbsp + in, e->rbsp_len - in);
    e->rbsp_len -= in;
    return out;
}

/* Planes */

void h264e_init(H264Enc *e)
{
    memset(e, 0, sizeof(*e));
}

void h264e_free(H264Enc *e)
{
    for (int i = 0; i < 3; i++) {
        g_free(e->src[i]);
        g_free(e->ref[i]);
        g_free(e->rec[i]);
        g_free(e->out[i]);
    }
    g_free(e->mb);
    g_free(e->rbsp);
    memset(e, 0, sizeof(*e));
}

static void alloc_planes(H264Enc *e, int mb_w, int mb_h)
{
    e->w = mb_w * 16;
    e->h = mb_h * 16;
    e->cw = mb_w * 8;
    e->ch = mb_h * 8;
    if (mb_w == e->alloc_w && mb_h == e->alloc_h) {
        return;
    }
    e->ly = e->w * e->h;
    e->lc = e->cw * e->ch;
    for (int i = 0; i < 3; i++) {
        size_t n = i ? e->lc : e->ly;

        g_free(e->src[i]);
        g_free(e->ref[i]);
        g_free(e->rec[i]);
        g_free(e->out[i]);
        e->src[i] = g_malloc0(n);
        e->ref[i] = g_malloc0(n);
        e->rec[i] = g_malloc0(n);
        e->out[i] = g_malloc0(n);
    }
    g_free(e->mb);
    e->mb_count = mb_w * mb_h;
    e->mb = g_new0(H264EncMB, e->mb_count);
    e->alloc_w = mb_w;
    e->alloc_h = mb_h;
}

void h264e_begin(H264Enc *e, const H264EncConfig *cfg, const uint8_t *hdr,
                 int hdr_len, uint8_t rem, int rem_len)
{
    e->cfg = *cfg;
    alloc_planes(e, cfg->mb_w, cfg->mb_h);
    memset(e->mb, 0, sizeof(H264EncMB) * cfg->mb_w * cfg->mb_h);
    e->rbsp_len = 0;
    e->acc = 0;
    e->acc_bits = 0;
    e->zeros = 0;
    e->bits = 0;
    for (int i = 0; i < hdr_len; i++) {
        put_bits(e, 8, hdr[i]);
    }
    put_bits(e, rem_len, rem >> (8 - rem_len));
    e->hdr_bits = e->bits;
    e->qp_prev = cfg->qp;
    e->skip_run = 0;
    e->rc_target = 0;
    e->mad_sum = 0;
    e->enc_bits = 0;
    e->qp_sum = 0;
    e->mvm_n = 0;
    e->mvm_total = 0;
}

void h264e_finish(H264Enc *e)
{
    if (!e->cfg.intra && e->skip_run) {
        put_ue(e, e->skip_run);
        e->skip_run = 0;
    }
    e->enc_bits = e->bits - e->hdr_bits;
    /* rbsp_slice_trailing_bits: the stop bit and alignment. */
    put_bits(e, 1, 1);
    if (e->acc_bits) {
        put_bits(e, 8 - e->acc_bits, 0);
    }
}

/* Source conversion */

static void rgb_to_yuv(int r, int g, int b, int *y, int *u, int *v)
{
    *y = (r * 4899 + g * 9617 + b * 1868) >> 14;
    *u = (128 * 16384 + b * 8192 - r * 2764 - g * 5428) >> 14;
    *v = (128 * 16384 + r * 8192 - g * 6860 - b * 1332) >> 14;
    *y = clip1(*y);
    *u = clip1(*u);
    *v = clip1(*v);
}

/* The bytes per pixel, as twice their number, of colour space @cs. */
static int cs_bytes2(int cs)
{
    switch (cs) {
    case H264E_CS_RGB888:
    case H264E_CS_YUV444:
        return 6;
    case H264E_CS_RGB565:
    case H264E_CS_YUV422:
        return 4;
    case H264E_CS_YUV420:
        return 3;
    default:
        return 2;
    }
}

/* Pixel @x of line @l as Y, Cb, Cr; Cb and Cr of 4:2:0 lines as they are. */
static void source_pixel(const uint8_t *line, int l, int x, int cs,
                         bool replace, uint32_t px, int *y, int *u, int *v)
{
    int b0, b1, b2;

    if (replace) {
        b0 = px & 0xff;
        b1 = (px >> 8) & 0xff;
        b2 = (px >> 16) & 0xff;
        switch (cs) {
        case H264E_CS_RGB888:
        case H264E_CS_RGB565:
            rgb_to_yuv(b2, b1, b0, y, u, v);
            break;
        case H264E_CS_GRAY:
            *y = b0;
            *u = *v = 128;
            break;
        default:
            *v = b0;
            *u = b1;
            *y = b2;
            break;
        }
        return;
    }
    switch (cs) {
    case H264E_CS_RGB888:
        rgb_to_yuv(line[3 * x + 2], line[3 * x + 1], line[3 * x], y, u, v);
        break;
    case H264E_CS_RGB565: {
        int w = line[2 * x] | line[2 * x + 1] << 8;

        rgb_to_yuv((w >> 11) << 3, ((w >> 5) & 63) << 2, (w & 31) << 3,
                   y, u, v);
        break;
    }
    case H264E_CS_YUV444:
        *v = line[3 * x];
        *u = line[3 * x + 1];
        *y = line[3 * x + 2];
        break;
    case H264E_CS_YUV422:
        *u = line[4 * (x >> 1)];
        *v = line[4 * (x >> 1) + 2];
        *y = line[4 * (x >> 1) + 1 + 2 * (x & 1)];
        break;
    case H264E_CS_YUV420:
        /* Each pair: the line's chroma sample, then two luma samples. */
        *y = line[3 * (x >> 1) + 1 + (x & 1)];
        *u = *v = line[3 * (x >> 1)];
        break;
    default:
        *y = line[x];
        *u = *v = 128;
        break;
    }
}

void h264e_load_source_row(H264Enc *e, int r, const uint8_t *data,
                           size_t line_bytes, int lines, int width, int cs,
                           bool replace, uint32_t replace_px)
{
    int cap = line_bytes * 2 / cs_bytes2(cs);
    int y, u, v;

    if (cs == H264E_CS_YUV420 || cs == H264E_CS_YUV422) {
        cap &= ~1;
    }
    width = MIN(width, cap);
    if (width <= 0 || lines <= 0) {
        h264e_fill_source_row(e, r, 0, 128, 128);
        return;
    }
    for (int j = 0; j < 16; j++) {
        int l = MIN(j, lines - 1);
        const uint8_t *line = data + l * line_bytes;
        uint8_t *dy = e->src[0] + (size_t)(r * 16 + j) * e->w;

        for (int i = 0; i < e->w; i++) {
            source_pixel(line, l, MIN(i, width - 1), cs, replace,
                         replace_px, &y, &u, &v);
            dy[i] = y;
        }
    }
    for (int j = 0; j < 8; j++) {
        int lu = MIN(2 * j, lines - 1), lv = MIN(2 * j + 1, lines - 1);
        uint8_t *du = e->src[1] + (size_t)(r * 8 + j) * e->cw;
        uint8_t *dv = e->src[2] + (size_t)(r * 8 + j) * e->cw;

        for (int i = 0; i < e->cw; i++) {
            int x = MIN(2 * i, (width - 1) & ~1);

            source_pixel(data + lu * line_bytes, lu, x, cs, replace,
                         replace_px, &y, &u, &v);
            du[i] = u;
            source_pixel(data + lv * line_bytes, lv, x, cs, replace,
                         replace_px, &y, &u, &v);
            dv[i] = v;
        }
    }
}

void h264e_fill_source_row(H264Enc *e, int r, uint8_t y, uint8_t u,
                           uint8_t v)
{
    memset(e->src[0] + (size_t)r * 16 * e->w, y, 16 * e->w);
    memset(e->src[1] + (size_t)r * 8 * e->cw, u, 8 * e->cw);
    memset(e->src[2] + (size_t)r * 8 * e->cw, v, 8 * e->cw);
}

/* The DMA's macroblock layouts */

static void mb_lines(H264Enc *e, uint8_t *const planes[3], int r, int x,
                     uint8_t *buf, int y0, int ny, int c0, int nc,
                     bool put)
{
    for (int j = 0; j < ny; j++) {
        uint8_t *p = planes[0] + (size_t)(r * 16 + y0 + j) * e->w + x * 16;

        put ? memcpy(p, buf, 16) : memcpy(buf, p, 16);
        buf += 16;
    }
    for (int c = 1; c < 3; c++) {
        for (int j = 0; j < nc; j++) {
            uint8_t *p = planes[c] + (size_t)(r * 8 + c0 + j) * e->cw + x * 8;

            put ? memcpy(p, buf, 8) : memcpy(buf, p, 8);
            buf += 8;
        }
    }
}

void h264e_line4_get(H264Enc *e, int r, int x, uint8_t *buf)
{
    mb_lines(e, e->out, r, x, buf, 12, 4, 4, 4, false);
}

void h264e_line4_put(H264Enc *e, int r, int x, const uint8_t *buf)
{
    mb_lines(e, e->out, r, x, (uint8_t *)buf, 12, 4, 4, 4, true);
}

void h264e_mb_get(H264Enc *e, int r, int x, uint8_t *buf, bool top,
                  bool bottom)
{
    if (top) {
        mb_lines(e, e->out, r, x, buf, 0, 12, 0, 4, false);
        buf += H264E_LINE12_BYTES;
    }
    if (bottom) {
        mb_lines(e, e->out, r, x, buf, 12, 4, 4, 4, false);
    }
}

void h264e_ref_put(H264Enc *e, int r, int x, const uint8_t *buf)
{
    mb_lines(e, e->ref, r, x, (uint8_t *)buf, 0, 12, 0, 4, true);
    mb_lines(e, e->ref, r, x, (uint8_t *)buf + H264E_LINE12_BYTES, 12, 4,
             4, 4, true);
}

/* Transforms */

static void fdct4(const int *in, int *out)
{
    int t[16];

    for (int i = 0; i < 4; i++) {
        const int *s = in + 4 * i;
        int a = s[0] + s[3], b = s[1] + s[2];
        int c = s[1] - s[2], d = s[0] - s[3];

        t[4 * i + 0] = a + b;
        t[4 * i + 1] = 2 * d + c;
        t[4 * i + 2] = a - b;
        t[4 * i + 3] = d - 2 * c;
    }
    for (int i = 0; i < 4; i++) {
        int a = t[i] + t[12 + i], b = t[4 + i] + t[8 + i];
        int c = t[4 + i] - t[8 + i], d = t[i] - t[12 + i];

        out[i] = a + b;
        out[4 + i] = 2 * d + c;
        out[8 + i] = a - b;
        out[12 + i] = d - 2 * c;
    }
}

/* The inverse transform of 8.5.12, added to the prediction. */
static void idct4_add(const int *d, const uint8_t *pred, int pstride,
                      uint8_t *dst, int dstride)
{
    int f[16], h[16];

    for (int i = 0; i < 4; i++) {
        const int *s = d + 4 * i;
        int e0 = s[0] + s[2], e1 = s[0] - s[2];
        int e2 = (s[1] >> 1) - s[3], e3 = s[1] + (s[3] >> 1);

        f[4 * i + 0] = e0 + e3;
        f[4 * i + 1] = e1 + e2;
        f[4 * i + 2] = e1 - e2;
        f[4 * i + 3] = e0 - e3;
    }
    for (int j = 0; j < 4; j++) {
        int g0 = f[j] + f[8 + j], g1 = f[j] - f[8 + j];
        int g2 = (f[4 + j] >> 1) - f[12 + j], g3 = f[4 + j] + (f[12 + j] >> 1);

        h[j] = g0 + g3;
        h[4 + j] = g1 + g2;
        h[8 + j] = g1 - g2;
        h[12 + j] = g0 - g3;
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst[i * dstride + j] = clip1(pred[i * pstride + j] +
                                         ((h[4 * i + j] + 32) >> 6));
        }
    }
}

static int quant1(int c, int mf, int qbits, int f)
{
    int l = ((abs(c) * mf) + f) >> qbits;

    return c < 0 ? -l : l;
}

/* 4x4 SATD: the Hadamard transform's absolute sum, halved. */
static int satd4(const uint8_t *a, int as, const uint8_t *b, int bs)
{
    int d[16], t[16], s = 0;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            d[4 * i + j] = a[i * as + j] - b[i * bs + j];
        }
    }
    for (int i = 0; i < 4; i++) {
        int a0 = d[4 * i] + d[4 * i + 1], a1 = d[4 * i] - d[4 * i + 1];
        int a2 = d[4 * i + 2] + d[4 * i + 3], a3 = d[4 * i + 2] - d[4 * i + 3];

        t[4 * i] = a0 + a2;
        t[4 * i + 1] = a1 + a3;
        t[4 * i + 2] = a0 - a2;
        t[4 * i + 3] = a1 - a3;
    }
    for (int j = 0; j < 4; j++) {
        int a0 = t[j] + t[4 + j], a1 = t[j] - t[4 + j];
        int a2 = t[8 + j] + t[12 + j], a3 = t[8 + j] - t[12 + j];

        s += abs(a0 + a2) + abs(a1 + a3) + abs(a0 - a2) + abs(a1 - a3);
    }
    return s >> 1;
}

static int satd_nxn(const uint8_t *a, int as, const uint8_t *b, int bs,
                    int n)
{
    int s = 0;

    for (int y = 0; y < n; y += 4) {
        for (int x = 0; x < n; x += 4) {
            s += satd4(a + y * as + x, as, b + y * bs + x, bs);
        }
    }
    return s;
}

static int sad_nxn(const uint8_t *a, int as, const uint8_t *b, int bs, int n)
{
    int s = 0;

    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            s += abs(a[y * as + x] - b[y * bs + x]);
        }
    }
    return s;
}

/* Intra prediction */

/* The samples around a block, as available. */
typedef struct Edge {
    bool left, top, topleft, topright;
    uint8_t t[17];      /* p[-1,-1], then p[0..15,-1] */
    uint8_t l[17];      /* p[-1,-1], then p[-1,0..15] */
} Edge;

/*
 * The edge of the @n x @n block at (@px, @py) of plane @p (0 luma, else
 * chroma), with @n4 top-right samples for a 4x4 block.
 */
static void get_edge(H264Enc *e, int p, int px, int py, int n, bool tr,
                     Edge *g)
{
    const uint8_t *pl = e->rec[p];
    int stride = p ? e->cw : e->w;

    g->left = px > 0;
    g->top = py > 0;
    g->topleft = g->left && g->top;
    g->topright = tr;
    if (g->topleft) {
        g->t[0] = g->l[0] = pl[(py - 1) * stride + px - 1];
    }
    if (g->top) {
        int nt = n == 4 ? 8 : n;

        for (int i = 0; i < nt; i++) {
            int x = i < n || tr ? px + i : px + n - 1;

            g->t[1 + i] = pl[(py - 1) * stride + x];
        }
    }
    if (g->left) {
        for (int i = 0; i < n; i++) {
            g->l[1 + i] = pl[(py + i) * stride + px - 1];
        }
    }
}

/* p[x,-1] and p[-1,y], x and y from -1. */
#define PT(x) (g->t[(x) + 1])
#define PL(y) (g->l[(y) + 1])

static bool i4_mode_ok(const Edge *g, int m)
{
    switch (m) {
    case 0:
    case 3:
    case 7:
        return g->top;
    case 1:
    case 8:
        return g->left;
    case 2:
        return true;
    default:
        return g->top && g->left && g->topleft;
    }
}

static void pred4(const Edge *g, int m, uint8_t *dst, int ds)
{
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            int v, z;

            switch (m) {
            case 0:
                v = PT(x);
                break;
            case 1:
                v = PL(y);
                break;
            case 2:
                if (g->top && g->left) {
                    v = (PT(0) + PT(1) + PT(2) + PT(3) + PL(0) + PL(1) +
                         PL(2) + PL(3) + 4) >> 3;
                } else if (g->left) {
                    v = (PL(0) + PL(1) + PL(2) + PL(3) + 2) >> 2;
                } else if (g->top) {
                    v = (PT(0) + PT(1) + PT(2) + PT(3) + 2) >> 2;
                } else {
                    v = 128;
                }
                break;
            case 3:
                if (x == 3 && y == 3) {
                    v = (PT(6) + 3 * PT(7) + 2) >> 2;
                } else {
                    v = (PT(x + y) + 2 * PT(x + y + 1) + PT(x + y + 2) + 2)
                        >> 2;
                }
                break;
            case 4:
                if (x > y) {
                    v = (PT(x - y - 2) + 2 * PT(x - y - 1) + PT(x - y) + 2)
                        >> 2;
                } else if (x < y) {
                    v = (PL(y - x - 2) + 2 * PL(y - x - 1) + PL(y - x) + 2)
                        >> 2;
                } else {
                    v = (PT(0) + 2 * PT(-1) + PL(0) + 2) >> 2;
                }
                break;
            case 5:
                z = 2 * x - y;
                if (z >= 0 && !(z & 1)) {
                    v = (PT(x - (y >> 1) - 1) + PT(x - (y >> 1)) + 1) >> 1;
                } else if (z >= 0) {
                    v = (PT(x - (y >> 1) - 2) + 2 * PT(x - (y >> 1) - 1) +
                         PT(x - (y >> 1)) + 2) >> 2;
                } else if (z == -1) {
                    v = (PL(0) + 2 * PL(-1) + PT(0) + 2) >> 2;
                } else {
                    v = (PL(y - 1) + 2 * PL(y - 2) + PL(y - 3) + 2) >> 2;
                }
                break;
            case 6:
                z = 2 * y - x;
                if (z >= 0 && !(z & 1)) {
                    v = (PL(y - (x >> 1) - 1) + PL(y - (x >> 1)) + 1) >> 1;
                } else if (z >= 0) {
                    v = (PL(y - (x >> 1) - 2) + 2 * PL(y - (x >> 1) - 1) +
                         PL(y - (x >> 1)) + 2) >> 2;
                } else if (z == -1) {
                    v = (PL(0) + 2 * PL(-1) + PT(0) + 2) >> 2;
                } else {
                    v = (PT(x - 1) + 2 * PT(x - 2) + PT(x - 3) + 2) >> 2;
                }
                break;
            case 7:
                if (!(y & 1)) {
                    v = (PT(x + (y >> 1)) + PT(x + (y >> 1) + 1) + 1) >> 1;
                } else {
                    v = (PT(x + (y >> 1)) + 2 * PT(x + (y >> 1) + 1) +
                         PT(x + (y >> 1) + 2) + 2) >> 2;
                }
                break;
            default:
                z = x + 2 * y;
                if (z > 5) {
                    v = PL(3);
                } else if (z == 5) {
                    v = (PL(2) + 3 * PL(3) + 2) >> 2;
                } else if (!(z & 1)) {
                    v = (PL(y + (x >> 1)) + PL(y + (x >> 1) + 1) + 1) >> 1;
                } else {
                    v = (PL(y + (x >> 1)) + 2 * PL(y + (x >> 1) + 1) +
                         PL(y + (x >> 1) + 2) + 2) >> 2;
                }
                break;
            }
            dst[y * ds + x] = v;
        }
    }
}

/* Intra 16x16 (m 0..3) and chroma (m 0 DC, 1 H, 2 V, 3 plane) for @n. */
static bool i16_mode_ok(const Edge *g, int m)
{
    switch (m) {
    case 0:
        return g->top;
    case 1:
        return g->left;
    case 2:
        return true;
    default:
        return g->top && g->left && g->topleft;
    }
}

static void pred16(const Edge *g, int m, uint8_t *dst)
{
    int dc = 128, a, b, c, hh = 0, vv = 0;

    switch (m) {
    case 0:
        for (int y = 0; y < 16; y++) {
            memcpy(dst + 16 * y, &PT(0), 16);
        }
        return;
    case 1:
        for (int y = 0; y < 16; y++) {
            memset(dst + 16 * y, PL(y), 16);
        }
        return;
    case 2:
        if (g->top || g->left) {
            int s = 0, n = 0;

            for (int i = 0; i < 16; i++) {
                s += g->top ? PT(i) : 0;
                s += g->left ? PL(i) : 0;
            }
            n = (g->top ? 16 : 0) + (g->left ? 16 : 0);
            dc = (s + n / 2) / n;
        }
        memset(dst, dc, 256);
        return;
    default:
        for (int i = 0; i < 8; i++) {
            hh += (i + 1) * (PT(8 + i) - PT(6 - i));
            vv += (i + 1) * (PL(8 + i) - PL(6 - i));
        }
        a = 16 * (PL(15) + PT(15));
        b = (5 * hh + 32) >> 6;
        c = (5 * vv + 32) >> 6;
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                dst[16 * y + x] = clip1((a + b * (x - 7) + c * (y - 7) + 16)
                                        >> 5);
            }
        }
        return;
    }
}

static int chroma_dc4(const Edge *g, int xo, int yo)
{
    int st = 0, sl = 0;

    for (int i = 0; i < 4; i++) {
        st += g->top ? PT(xo + i) : 0;
        sl += g->left ? PL(yo + i) : 0;
    }
    if ((xo == 0 && yo == 0) || (xo && yo)) {
        if (g->top && g->left) {
            return (st + sl + 4) >> 3;
        }
        if (g->left) {
            return (sl + 2) >> 2;
        }
        if (g->top) {
            return (st + 2) >> 2;
        }
        return 128;
    }
    if (xo) {
        if (g->top) {
            return (st + 2) >> 2;
        }
        if (g->left) {
            return (sl + 2) >> 2;
        }
        return 128;
    }
    if (g->left) {
        return (sl + 2) >> 2;
    }
    if (g->top) {
        return (st + 2) >> 2;
    }
    return 128;
}

static bool chroma_mode_ok(const Edge *g, int m)
{
    switch (m) {
    case 0:
        return true;
    case 1:
        return g->left;
    case 2:
        return g->top;
    default:
        return g->top && g->left && g->topleft;
    }
}

static void pred_chroma(const Edge *g, int m, uint8_t *dst)
{
    int a, b, c, hh = 0, vv = 0;

    switch (m) {
    case 0:
        for (int yo = 0; yo < 8; yo += 4) {
            for (int xo = 0; xo < 8; xo += 4) {
                int dc = chroma_dc4(g, xo, yo);

                for (int y = 0; y < 4; y++) {
                    memset(dst + 8 * (yo + y) + xo, dc, 4);
                }
            }
        }
        return;
    case 1:
        for (int y = 0; y < 8; y++) {
            memset(dst + 8 * y, PL(y), 8);
        }
        return;
    case 2:
        for (int y = 0; y < 8; y++) {
            memcpy(dst + 8 * y, &PT(0), 8);
        }
        return;
    default:
        for (int i = 0; i < 4; i++) {
            hh += (i + 1) * (PT(4 + i) - PT(2 - i));
            vv += (i + 1) * (PL(4 + i) - PL(2 - i));
        }
        a = 16 * (PL(7) + PT(7));
        b = (34 * hh + 32) >> 6;
        c = (34 * vv + 32) >> 6;
        for (int y = 0; y < 8; y++) {
            for (int x = 0; x < 8; x++) {
                dst[8 * y + x] = clip1((a + b * (x - 3) + c * (y - 3) + 16)
                                       >> 5);
            }
        }
        return;
    }
}

#undef PT
#undef PL

/* Inter prediction */

static inline int ref_px(H264Enc *e, int p, int x, int y)
{
    int w = p ? e->cw : e->w, h = p ? e->ch : e->h;

    return e->ref[p][clip3(0, h - 1, y) * w + clip3(0, w - 1, x)];
}

/* The six-tap filter over six samples. */
static inline int tap6(int a, int b, int c, int d, int f, int g)
{
    return a - 5 * b + 20 * c + 20 * d - 5 * f + g;
}

/*
 * The 16 x 16 luma prediction of the block at (@bx, @by) with motion
 * vector (@mvx, @mvy) in quarter samples (8.4.2.2.1).
 */
static void mc_luma(H264Enc *e, int bx, int by, int mvx, int mvy,
                    uint8_t *dst)
{
    int xi = bx + (mvx >> 2), yi = by + (mvy >> 2);
    int xf = mvx & 3, yf = mvy & 3;
    /* Integer samples from (xi - 2, yi - 2), 21 x 21. */
    int G[21][21];
    int b[21][16];      /* horizontal half samples, rows yi - 2.. */
    int h[16][21];      /* vertical half samples, columns xi - 2.. */
    int j[16][17];      /* centre half samples, (x + 1/2, y + 1/2) */

    for (int y = 0; y < 21; y++) {
        for (int x = 0; x < 21; x++) {
            G[y][x] = ref_px(e, 0, xi - 2 + x, yi - 2 + y);
        }
    }
    if (!xf && !yf) {
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                dst[16 * y + x] = G[y + 2][x + 2];
            }
        }
        return;
    }
    /* b1 at (x + 1/2, y) for x 0..16, y -2..18 (unclipped, in b1). */
    {
        int b1[21][17];

        for (int y = 0; y < 21; y++) {
            for (int x = 0; x < 17; x++) {
                b1[y][x] = tap6(G[y][x], G[y][x + 1], G[y][x + 2],
                                G[y][x + 3], G[y][x + 4], G[y][x + 5]);
            }
        }
        for (int y = 0; y < 21; y++) {
            for (int x = 0; x < 16; x++) {
                b[y][x] = clip1((b1[y][x] + 16) >> 5);
            }
        }
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 17; x++) {
                int j1 = tap6(b1[y][x], b1[y + 1][x], b1[y + 2][x],
                              b1[y + 3][x], b1[y + 4][x], b1[y + 5][x]);

                j[y][x] = clip1((j1 + 512) >> 10);
            }
        }
    }
    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 21; x++) {
            int h1 = tap6(G[y][x], G[y + 1][x], G[y + 2][x], G[y + 3][x],
                          G[y + 4][x], G[y + 5][x]);

            h[y][x] = clip1((h1 + 16) >> 5);
        }
    }
    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
            int g0 = G[y + 2][x + 2];
            int bb = b[y + 2][x];           /* (x + 1/2, y) */
            int hh = h[y][x + 2];           /* (x, y + 1/2) */
            int jj = j[y][x];               /* (x + 1/2, y + 1/2) */
            int v;

            switch (yf * 4 + xf) {
            case 1:
                v = (g0 + bb + 1) >> 1;
                break;
            case 2:
                v = bb;
                break;
            case 3:
                v = (G[y + 2][x + 3] + bb + 1) >> 1;
                break;
            case 4:
                v = (g0 + hh + 1) >> 1;
                break;
            case 5:
                v = (bb + hh + 1) >> 1;
                break;
            case 6:
                v = (bb + jj + 1) >> 1;
                break;
            case 7:
                v = (bb + h[y][x + 3] + 1) >> 1;
                break;
            case 8:
                v = hh;
                break;
            case 9:
                v = (hh + jj + 1) >> 1;
                break;
            case 10:
                v = jj;
                break;
            case 11:
                v = (jj + h[y][x + 3] + 1) >> 1;
                break;
            case 12:
                v = (G[y + 3][x + 2] + hh + 1) >> 1;
                break;
            case 13:
                v = (hh + b[y + 3][x] + 1) >> 1;
                break;
            case 14:
                v = (jj + b[y + 3][x] + 1) >> 1;
                break;
            default:
                v = (h[y][x + 3] + b[y + 3][x] + 1) >> 1;
                break;
            }
            dst[16 * y + x] = v;
        }
    }
}

/* The 8 x 8 chroma predictions of plane @p (8.4.2.2.2). */
static void mc_chroma(H264Enc *e, int p, int bx, int by, int mvx, int mvy,
                      uint8_t *dst)
{
    int xi = bx + (mvx >> 3), yi = by + (mvy >> 3);
    int xf = mvx & 7, yf = mvy & 7;

    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            int a = ref_px(e, p, xi + x, yi + y);
            int b = ref_px(e, p, xi + x + 1, yi + y);
            int c = ref_px(e, p, xi + x, yi + y + 1);
            int d = ref_px(e, p, xi + x + 1, yi + y + 1);

            dst[8 * y + x] = ((8 - xf) * (8 - yf) * a + xf * (8 - yf) * b +
                              (8 - xf) * yf * c + xf * yf * d + 32) >> 6;
        }
    }
}

/* Integer-sample SAD of the 16 x 16 block at (@bx, @by) displaced. */
static int sad_int(H264Enc *e, const uint8_t *src, int bx, int by, int dx,
                   int dy, int limit)
{
    int x0 = bx + dx, y0 = by + dy, s = 0;

    if (x0 >= 0 && y0 >= 0 && x0 + 16 <= e->w && y0 + 16 <= e->h) {
        const uint8_t *r = e->ref[0] + (size_t)y0 * e->w + x0;

        for (int y = 0; y < 16 && s < limit; y++) {
            for (int x = 0; x < 16; x++) {
                s += abs(src[16 * y + x] - r[y * e->w + x]);
            }
        }
        return s;
    }
    for (int y = 0; y < 16 && s < limit; y++) {
        for (int x = 0; x < 16; x++) {
            s += abs(src[16 * y + x] - ref_px(e, 0, x0 + x, y0 + y));
        }
    }
    return s;
}

/* Motion vector prediction */

typedef struct Neigh {
    bool avail;         /* in the picture (and slice) */
    bool inter;         /* refIdxL0 is 0 */
    int mv[2];
} Neigh;

static void neigh(H264Enc *e, int x, int y, Neigh *n)
{
    n->avail = x >= 0 && y >= 0 && x < e->cfg.mb_w;
    n->inter = false;
    n->mv[0] = n->mv[1] = 0;
    if (n->avail) {
        H264EncMB *m = &e->mb[y * e->cfg.mb_w + x];

        if (m->type == H264E_MB_P16 || m->type == H264E_MB_PSKIP) {
            n->inter = true;
            n->mv[0] = m->mv[0];
            n->mv[1] = m->mv[1];
        }
    }
}

static int median3(int a, int b, int c)
{
    return MAX(MIN(a, b), MIN(MAX(a, b), c));
}

/* mvpL0 of a 16x16 partition (8.4.1.3), and P_Skip's vector (8.4.1.1). */
static void mv_pred(H264Enc *e, int x, int y, int mvp[2], int skip[2])
{
    Neigh a, b, c;
    int matches;

    neigh(e, x - 1, y, &a);
    neigh(e, x, y - 1, &b);
    neigh(e, x + 1, y - 1, &c);
    if (!c.avail) {
        neigh(e, x - 1, y - 1, &c);
    }
    if (!b.avail && !c.avail && a.avail) {
        b = a;
        c = a;
    }
    matches = a.inter + b.inter + c.inter;
    if (matches == 1) {
        const Neigh *n = a.inter ? &a : b.inter ? &b : &c;

        mvp[0] = n->mv[0];
        mvp[1] = n->mv[1];
    } else {
        mvp[0] = median3(a.mv[0], b.mv[0], c.mv[0]);
        mvp[1] = median3(a.mv[1], b.mv[1], c.mv[1]);
    }
    neigh(e, x - 1, y, &a);
    neigh(e, x, y - 1, &b);
    if (!a.avail || !b.avail ||
        (a.inter && !a.mv[0] && !a.mv[1]) ||
        (b.inter && !b.mv[0] && !b.mv[1])) {
        skip[0] = skip[1] = 0;
    } else {
        skip[0] = mvp[0];
        skip[1] = mvp[1];
    }
}

static bool mv_in_range(int mvx, int mvy)
{
    return mvx >= MV_X_MIN && mvx <= MV_X_MAX &&
           mvy >= MV_Y_MIN && mvy <= MV_Y_MAX;
}

/* CAVLC */

static int nz_at(H264Enc *e, int mbx, int mby, int bx, int by)
{
    /* The luma 4x4 block (bx, by), in 4x4 units, relative to mb (mbx, mby). */
    if (bx < 0) {
        mbx--;
        bx += 4;
    }
    if (by < 0) {
        mby--;
        by += 4;
    }
    if (mbx < 0 || mby < 0) {
        return -1;
    }
    return e->mb[mby * e->cfg.mb_w + mbx].nz[by * 4 + bx];
}

static int nzc_at(H264Enc *e, int c, int mbx, int mby, int bx, int by)
{
    if (bx < 0) {
        mbx--;
        bx += 2;
    }
    if (by < 0) {
        mby--;
        by += 2;
    }
    if (mbx < 0 || mby < 0) {
        return -1;
    }
    return e->mb[mby * e->cfg.mb_w + mbx].nzc[c][by * 2 + bx];
}

static int nc_of(int na, int nb)
{
    if (na >= 0 && nb >= 0) {
        return (na + nb + 1) >> 1;
    }
    if (na >= 0) {
        return na;
    }
    return nb >= 0 ? nb : 0;
}

/*
 * residual_block_cavlc() of @n levels in scan order with context @nc
 * (-1 for chroma DC); returns TotalCoeff.
 */
static int put_block(H264Enc *e, const int *lv, int n, int nc)
{
    int total = 0, t1 = 0, last = -1, zeros = 0;
    int levels[16], runs[16];
    int k, suffix_len;

    for (int i = n - 1; i >= 0; i--) {
        if (lv[i]) {
            if (last < 0) {
                last = i;
            }
            levels[total] = lv[i];
            runs[total] = 0;
            total++;
        } else if (last >= 0) {
            runs[total - 1]++;
            zeros++;
        }
    }
    /* levels[] runs from the highest frequency; runs[i] precede levels[i]. */
    for (k = 0; k < total && k < 3 && abs(levels[k]) == 1; k++) {
        t1++;
    }
    if (nc < 0) {
        put_bits(e, ct_dc_len[4 * total + t1], ct_dc_code[4 * total + t1]);
    } else if (nc >= 8) {
        put_bits(e, 6, total ? ((total - 1) << 2) | t1 : 3);
    } else {
        int t = nc < 2 ? 0 : nc < 4 ? 1 : 2;

        put_bits(e, ct_len[t][4 * total + t1], ct_code[t][4 * total + t1]);
    }
    if (!total) {
        return 0;
    }
    for (k = 0; k < t1; k++) {
        put_bits(e, 1, levels[k] < 0);
    }
    suffix_len = total > 10 && t1 < 3 ? 1 : 0;
    for (k = t1; k < total; k++) {
        int l = levels[k];
        int code = l > 0 ? 2 * l - 2 : -2 * l - 1;

        if (k == t1 && t1 < 3) {
            code -= 2;
        }
        if (!suffix_len) {
            if (code < 14) {
                put_bits(e, code + 1, 1);
            } else if (code < 30) {
                put_bits(e, 15, 1);
                put_bits(e, 4, code - 14);
            } else {
                put_bits(e, 16, 1);
                put_bits(e, 12, MIN(code - 30, 4095));
            }
        } else if (code < (15 << suffix_len)) {
            put_bits(e, (code >> suffix_len) + 1, 1);
            put_bits(e, suffix_len, code & ((1 << suffix_len) - 1));
        } else {
            put_bits(e, 16, 1);
            put_bits(e, 12, MIN(code - (15 << suffix_len), 4095));
        }
        if (!suffix_len) {
            suffix_len = 1;
        }
        if (abs(l) > (3 << (suffix_len - 1)) && suffix_len < 6) {
            suffix_len++;
        }
    }
    if (total < n) {
        if (n == 4) {
            put_bits(e, tz_dc_len[total - 1][zeros],
                     tz_dc_code[total - 1][zeros]);
        } else {
            put_bits(e, tz_len[total - 1][zeros], tz_code[total - 1][zeros]);
        }
    }
    for (k = 0; k < total - 1 && zeros > 0; k++) {
        int z = MIN(zeros, 7) - 1;

        put_bits(e, rb_len[z][runs[k]], rb_code[z][runs[k]]);
        zeros -= runs[k];
    }
    return total;
}

/* Decimation's score of @n levels in scan order. */
static int deci_score(const int *lv, int n)
{
    int i = n - 1, s = 0;

    while (i >= 0 && !lv[i]) {
        i--;
    }
    while (i >= 0) {
        int run = 0;

        if (abs(lv[i--]) > 1) {
            return 9;
        }
        while (i >= 0 && !lv[i]) {
            i--;
            run++;
        }
        s += deci_table[run];
    }
    return s;
}

/* Macroblock coding */

typedef struct MBCode {
    int type;
    int qp;
    int i16mode, cmode;
    int8_t i4mode[16];          /* raster */
    int mv[2], mvp[2];
    int luma[16][16];           /* per raster block, scan order */
    int ldc[16];                /* intra 16x16 DC levels, scan order */
    int cdc[2][4];
    int cac[2][4][16];          /* scan order, index 0 unused */
    int cbp_l, cbp_c;
    uint8_t py[256], pu[64], pv[64];    /* prediction */
    uint8_t ry[256], ru[64], rv[64];    /* reconstruction */
    int sad;                    /* luma SAD of the prediction */
    int cost;
} MBCode;

/* Quantize and reconstruct the chroma of @m, at QPC @qpc, @intra rounding. */
static void code_chroma(H264Enc *e, MBCode *m, const uint8_t *su,
                        const uint8_t *sv, int qpc, bool intra)
{
    int qbits = 15 + qpc / 6, f = (1 << qbits) / (intra ? 3 : 6);
    const int *mf = quant_mf[qpc % 6], *dv = dequant_v[qpc % 6];
    bool ac = false, dc = false;
    int score = 0;

    for (int c = 0; c < 2; c++) {
        const uint8_t *s = c ? sv : su;
        const uint8_t *p = c ? m->pv : m->pu;
        int coef[4][16], dcin[4], t[4];

        for (int b = 0; b < 4; b++) {
            int res[16], bx = (b & 1) * 4, by = (b >> 1) * 4;

            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    res[4 * y + x] = s[8 * (by + y) + bx + x] -
                                     p[8 * (by + y) + bx + x];
                }
            }
            fdct4(res, coef[b]);
            dcin[b] = coef[b][0];
            for (int k = 1; k < 16; k++) {
                int pos = zigzag[k];

                m->cac[c][b][k] = quant1(coef[b][pos], mf[pos_class(pos)],
                                         qbits, f);
            }
            m->cac[c][b][0] = 0;
            score += deci_score(m->cac[c][b] + 1, 15);
        }
        /* The 2x2 Hadamard transform of the DC coefficients. */
        t[0] = dcin[0] + dcin[1] + dcin[2] + dcin[3];
        t[1] = dcin[0] - dcin[1] + dcin[2] - dcin[3];
        t[2] = dcin[0] + dcin[1] - dcin[2] - dcin[3];
        t[3] = dcin[0] - dcin[1] - dcin[2] + dcin[3];
        for (int k = 0; k < 4; k++) {
            m->cdc[c][k] = quant1(t[k], mf[0], qbits + 1, 2 * f);
            dc |= m->cdc[c][k] != 0;
        }
    }
    if (score + (intra ? e->cfg.ic_deci_offset : e->cfg.pc_deci_offset) <
        e->cfg.c_deci_score) {
        memset(m->cac, 0, sizeof(m->cac));
    }
    for (int c = 0; c < 2; c++) {
        for (int b = 0; b < 4; b++) {
            for (int k = 1; k < 16; k++) {
                ac |= m->cac[c][b][k] != 0;
            }
        }
    }
    m->cbp_c = ac ? 2 : dc ? 1 : 0;
    /* Reconstruct as the decoding process does (8.5.11, 8.5.12). */
    for (int c = 0; c < 2; c++) {
        const uint8_t *p = c ? m->pv : m->pu;
        uint8_t *r = c ? m->rv : m->ru;
        int *l = m->cdc[c], f2[4], dcc[4];

        f2[0] = l[0] + l[1] + l[2] + l[3];
        f2[1] = l[0] - l[1] + l[2] - l[3];
        f2[2] = l[0] + l[1] - l[2] - l[3];
        f2[3] = l[0] - l[1] - l[2] + l[3];
        for (int k = 0; k < 4; k++) {
            dcc[k] = ((f2[k] * dv[0] * 16) << (qpc / 6)) >> 5;
        }
        for (int b = 0; b < 4; b++) {
            int d[16], bx = (b & 1) * 4, by = (b >> 1) * 4;

            for (int k = 1; k < 16; k++) {
                int pos = zigzag[k];

                d[pos] = (m->cac[c][b][k] * dv[pos_class(pos)]) << (qpc / 6);
            }
            d[0] = dcc[b];
            idct4_add(d, p + 8 * by + bx, 8, r + 8 * by + bx, 8);
        }
    }
}

/* Luma of an inter or intra 16x16 macroblock from its prediction. */
static void code_luma16(H264Enc *e, MBCode *m, const uint8_t *sy, int qp,
                        bool i16)
{
    int qbits = 15 + qp / 6, f = (1 << qbits) / (i16 ? 3 : 6);
    const int *mf = quant_mf[qp % 6], *dv = dequant_v[qp % 6];
    int coef[16][16], dcin[16], score = 0;

    m->cbp_l = 0;
    for (int b = 0; b < 16; b++) {
        int res[16], bx = (b & 3) * 4, by = (b >> 2) * 4;

        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                res[4 * y + x] = sy[16 * (by + y) + bx + x] -
                                 m->py[16 * (by + y) + bx + x];
            }
        }
        fdct4(res, coef[b]);
        dcin[b] = coef[b][0];
        for (int k = i16 ? 1 : 0; k < 16; k++) {
            int pos = zigzag[k];

            m->luma[b][k] = quant1(coef[b][pos], mf[pos_class(pos)], qbits,
                                   f);
        }
        if (i16) {
            m->luma[b][0] = 0;
            score += deci_score(m->luma[b] + 1, 15);
        } else {
            score += deci_score(m->luma[b], 16);
        }
    }
    if (score + (i16 ? e->cfg.i16_deci_offset : e->cfg.p16_deci_offset) <
        e->cfg.l_deci_score) {
        memset(m->luma, 0, sizeof(m->luma));
    }
    if (i16) {
        int t[16], u[16];

        /* The 4x4 Hadamard transform of the DC coefficients, halved. */
        for (int i = 0; i < 4; i++) {
            int a = dcin[4 * i] + dcin[4 * i + 1];
            int b = dcin[4 * i + 2] + dcin[4 * i + 3];
            int c = dcin[4 * i] - dcin[4 * i + 1];
            int d = dcin[4 * i + 2] - dcin[4 * i + 3];

            t[4 * i] = a + b;
            t[4 * i + 1] = a - b;
            t[4 * i + 2] = c - d;
            t[4 * i + 3] = c + d;
        }
        for (int j = 0; j < 4; j++) {
            int a = t[j] + t[4 + j], b = t[8 + j] + t[12 + j];
            int c = t[j] - t[4 + j], d = t[8 + j] - t[12 + j];

            u[j] = (a + b) >> 1;
            u[4 + j] = (a - b) >> 1;
            u[8 + j] = (c - d) >> 1;
            u[12 + j] = (c + d) >> 1;
        }
        for (int k = 0; k < 16; k++) {
            m->ldc[k] = quant1(u[zigzag[k]], mf[0], qbits + 1, 2 * f);
        }
        for (int b = 0; b < 16; b++) {
            for (int k = 1; k < 16; k++) {
                if (m->luma[b][k]) {
                    m->cbp_l = 15;
                }
            }
        }
    } else {
        for (int b = 0; b < 16; b++) {
            for (int k = 0; k < 16; k++) {
                if (m->luma[b][k]) {
                    m->cbp_l |= 1 << ((b >> 3) * 2 + ((b & 3) >> 1));
                }
            }
        }
    }
    /* Reconstruct (8.5.10, 8.5.12). */
    {
        int dcy[16];

        if (i16) {
            int c[16], t[16];

            for (int k = 0; k < 16; k++) {
                c[zigzag[k]] = m->ldc[k];
            }
            for (int i = 0; i < 4; i++) {
                int a = c[4 * i] + c[4 * i + 1];
                int b = c[4 * i + 2] + c[4 * i + 3];
                int cc = c[4 * i] - c[4 * i + 1];
                int d = c[4 * i + 2] - c[4 * i + 3];

                t[4 * i] = a + b;
                t[4 * i + 1] = a - b;
                t[4 * i + 2] = cc - d;
                t[4 * i + 3] = cc + d;
            }
            for (int j = 0; j < 4; j++) {
                int a = t[j] + t[4 + j], b = t[8 + j] + t[12 + j];
                int cc = t[j] - t[4 + j], d = t[8 + j] - t[12 + j];
                int f4[4] = { a + b, a - b, cc - d, cc + d };

                for (int i = 0; i < 4; i++) {
                    int v = f4[i] * dv[0] * 16;

                    if (qp >= 36) {
                        dcy[4 * i + j] = v << (qp / 6 - 6);
                    } else {
                        dcy[4 * i + j] = (v + (1 << (5 - qp / 6))) >>
                                         (6 - qp / 6);
                    }
                }
            }
        }
        for (int b = 0; b < 16; b++) {
            int d[16], bx = (b & 3) * 4, by = (b >> 2) * 4;

            for (int k = 0; k < 16; k++) {
                int pos = zigzag[k];

                d[pos] = (m->luma[b][k] * dv[pos_class(pos)]) << (qp / 6);
            }
            if (i16) {
                d[0] = dcy[b];
            }
            idct4_add(d, m->py + 16 * by + bx, 16, m->ry + 16 * by + bx, 16);
        }
    }
}

/* The chroma intra mode of the best SATD, with its prediction in @m. */
static void choose_chroma(H264Enc *e, MBCode *m, int mbx, int mby,
                          const uint8_t *su, const uint8_t *sv)
{
    Edge gu, gv;
    uint8_t pu[64], pv[64];
    int best = INT_MAX;

    get_edge(e, 1, mbx * 8, mby * 8, 8, false, &gu);
    get_edge(e, 2, mbx * 8, mby * 8, 8, false, &gv);
    for (int mode = 0; mode < 4; mode++) {
        int c;

        if (!chroma_mode_ok(&gu, mode)) {
            continue;
        }
        pred_chroma(&gu, mode, pu);
        pred_chroma(&gv, mode, pv);
        c = satd_nxn(su, 8, pu, 8, 8) + satd_nxn(sv, 8, pv, 8, 8);
        if (c < best) {
            best = c;
            m->cmode = mode;
            memcpy(m->pu, pu, 64);
            memcpy(m->pv, pv, 64);
        }
    }
}

/* The best intra 16x16 mode, its prediction, and its cost. */
static void try_i16(H264Enc *e, MBCode *m, int mbx, int mby,
                    const uint8_t *sy, int lambda)
{
    Edge g;
    uint8_t p[256];
    int best = INT_MAX;

    get_edge(e, 0, mbx * 16, mby * 16, 16, false, &g);
    for (int mode = 0; mode < 4; mode++) {
        int c;

        if (!i16_mode_ok(&g, mode)) {
            continue;
        }
        pred16(&g, mode, p);
        c = satd_nxn(sy, 16, p, 16, 16);
        if (c < best) {
            best = c;
            m->i16mode = mode;
            memcpy(m->py, p, 256);
        }
    }
    m->type = H264E_MB_I16;
    m->cost = best + 4 * lambda;
    m->sad = sad_nxn(sy, 16, m->py, 16, 16);
}

/* Intra4x4PredMode's prediction for raster block @b of (@mbx, @mby). */
static int i4_pred_mode(H264Enc *e, MBCode *m, int mbx, int mby, int b)
{
    int bx = b & 3, by = b >> 2, ma, mb;

    if (bx) {
        ma = m->i4mode[b - 1];
    } else if (mbx) {
        H264EncMB *n = &e->mb[mby * e->cfg.mb_w + mbx - 1];

        ma = n->type == H264E_MB_I4 ? n->i4[by * 4 + 3] : 2;
    } else {
        return 2;
    }
    if (by) {
        mb = m->i4mode[b - 4];
    } else if (mby) {
        H264EncMB *n = &e->mb[(mby - 1) * e->cfg.mb_w + mbx];

        mb = n->type == H264E_MB_I4 ? n->i4[12 + bx] : 2;
    } else {
        return 2;
    }
    return MIN(ma, mb);
}

/*
 * Code an intra 4x4 macroblock, each block's mode the cheapest by SATD
 * and mode bits, reconstructed in place in the rec plane as it goes (each
 * block predicts from those before it); the caller restores the plane if
 * another type wins.
 */
static void try_i4(H264Enc *e, MBCode *m, int mbx, int mby,
                   const uint8_t *sy, int qp, int lambda)
{
    int qbits = 15 + qp / 6, f = (1 << qbits) / 3;
    const int *mf = quant_mf[qp % 6], *dv = dequant_v[qp % 6];
    uint8_t *rec = e->rec[0];
    int cost = 8 * lambda;

    m->type = H264E_MB_I4;
    m->cbp_l = 0;
    m->sad = 0;
    for (int k = 0; k < 16; k++) {
        int b = blk_raster[k], bx = b & 3, by = b >> 2;
        int px = mbx * 16 + bx * 4, py = mby * 16 + by * 4;
        bool tr;
        Edge g;
        uint8_t p[16];
        int best = INT_MAX, pm = i4_pred_mode(e, m, mbx, mby, b);
        int res[16], coef[16], d[16];

        if (by == 0) {
            tr = mby > 0 && (bx < 3 || mbx + 1 < e->cfg.mb_w);
        } else {
            tr = bx < 3 && blk_zorder[(by - 1) * 4 + bx + 1] < k;
        }
        get_edge(e, 0, px, py, 4, tr, &g);
        for (int mode = 0; mode < 9; mode++) {
            int c;

            if (!i4_mode_ok(&g, mode)) {
                continue;
            }
            pred4(&g, mode, p, 4);
            c = satd4(sy + 16 * by * 4 + bx * 4, 16, p, 4) +
                lambda * (mode == pm ? 1 : 4);
            if (c < best) {
                best = c;
                m->i4mode[b] = mode;
            }
        }
        cost += best;
        pred4(&g, m->i4mode[b], p, 4);
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                res[4 * y + x] = sy[16 * (by * 4 + y) + bx * 4 + x] -
                                 p[4 * y + x];
                m->py[16 * (by * 4 + y) + bx * 4 + x] = p[4 * y + x];
                m->sad += abs(res[4 * y + x]);
            }
        }
        fdct4(res, coef);
        for (int i = 0; i < 16; i++) {
            int pos = zigzag[i];

            m->luma[b][i] = quant1(coef[pos], mf[pos_class(pos)], qbits, f);
            if (m->luma[b][i]) {
                m->cbp_l |= 1 << ((by >> 1) * 2 + (bx >> 1));
            }
            d[pos] = (m->luma[b][i] * dv[pos_class(pos)]) << (qp / 6);
        }
        idct4_add(d, p, 4, m->ry + 16 * by * 4 + bx * 4, 16);
        for (int y = 0; y < 4; y++) {
            memcpy(rec + (size_t)(py + y) * e->w + px,
                   m->ry + 16 * (by * 4 + y) + bx * 4, 4);
        }
    }
    m->cost = cost;
}

/* The bits of a motion vector difference. */
static int mvd_bits(int mvx, int mvy, const int mvp[2])
{
    return se_bits(mvx - mvp[0]) + se_bits(mvy - mvp[1]);
}

/* The best motion vector for (@mbx, @mby) and its prediction and cost. */
static void try_inter(H264Enc *e, MBCode *m, int mbx, int mby,
                      const uint8_t *sy, int lambda)
{
    int bx = mbx * 16, by = mby * 16;
    int cand[6][2], nc = 0, best[2] = { 0, 0 }, bestc = INT_MAX;
    uint8_t p[256];
    static const int dia[4][2] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
    Neigh n;

    cand[nc][0] = 0;
    cand[nc++][1] = 0;
    cand[nc][0] = m->mvp[0];
    cand[nc++][1] = m->mvp[1];
    neigh(e, mbx - 1, mby, &n);
    if (n.inter) {
        cand[nc][0] = n.mv[0];
        cand[nc++][1] = n.mv[1];
    }
    neigh(e, mbx, mby - 1, &n);
    if (n.inter) {
        cand[nc][0] = n.mv[0];
        cand[nc++][1] = n.mv[1];
    }
    neigh(e, mbx + 1, mby - 1, &n);
    if (n.inter) {
        cand[nc][0] = n.mv[0];
        cand[nc++][1] = n.mv[1];
    }
    /* Integer search, in whole samples. */
    for (int i = 0; i < nc; i++) {
        int dx = clip3(INT_X_MIN, INT_X_MAX, (cand[i][0] + 2) >> 2);
        int dy = clip3(INT_Y_MIN, INT_Y_MAX, (cand[i][1] + 2) >> 2);
        int c = sad_int(e, sy, bx, by, dx, dy, bestc) +
                lambda * mvd_bits(dx * 4, dy * 4, m->mvp);

        if (c < bestc) {
            bestc = c;
            best[0] = dx;
            best[1] = dy;
        }
    }
    for (int iter = 0; iter < 32; iter++) {
        int moved = -1;

        for (int i = 0; i < 4; i++) {
            int dx = best[0] + dia[i][0], dy = best[1] + dia[i][1];
            int c;

            if (dx < INT_X_MIN || dx > INT_X_MAX ||
                dy < INT_Y_MIN || dy > INT_Y_MAX) {
                continue;
            }
            c = sad_int(e, sy, bx, by, dx, dy, bestc) +
                lambda * mvd_bits(dx * 4, dy * 4, m->mvp);
            if (c < bestc) {
                bestc = c;
                moved = i;
            }
        }
        if (moved < 0) {
            break;
        }
        best[0] += dia[moved][0];
        best[1] += dia[moved][1];
    }
    /* Half then quarter sample refinement, by SATD. */
    m->mv[0] = best[0] * 4;
    m->mv[1] = best[1] * 4;
    mc_luma(e, bx, by, m->mv[0], m->mv[1], m->py);
    bestc = satd_nxn(sy, 16, m->py, 16, 16) +
            lambda * mvd_bits(m->mv[0], m->mv[1], m->mvp);
    for (int step = 2; step >= 1; step--) {
        int c0[2] = { m->mv[0], m->mv[1] };

        for (int dy = -step; dy <= step; dy += step) {
            for (int dx = -step; dx <= step; dx += step) {
                int mvx = c0[0] + dx, mvy = c0[1] + dy, c;

                if ((!dx && !dy) || !mv_in_range(mvx, mvy)) {
                    continue;
                }
                mc_luma(e, bx, by, mvx, mvy, p);
                c = satd_nxn(sy, 16, p, 16, 16) +
                    lambda * mvd_bits(mvx, mvy, m->mvp);
                if (c < bestc) {
                    bestc = c;
                    m->mv[0] = mvx;
                    m->mv[1] = mvy;
                    memcpy(m->py, p, 256);
                }
            }
        }
    }
    m->type = H264E_MB_P16;
    m->cost = bestc;
    m->sad = sad_nxn(sy, 16, m->py, 16, 16);
}

/* Rate control and ROI */

static int roi_qp(H264Enc *e, int mbx, int mby, int qp)
{
    const H264EncConfig *c = &e->cfg;

    if (!c->roi_en) {
        return qp;
    }
    for (int i = 0; i < 8; i++) {
        int v = c->roi[i].qp & 0x3f;

        if (!c->roi[i].en || mbx < c->roi[i].x || mby < c->roi[i].y ||
            mbx >= c->roi[i].x + c->roi[i].w ||
            mby >= c->roi[i].y + c->roi[i].h) {
            continue;
        }
        if (!c->roi_delta) {
            return c->roi[i].qp;
        }
        return qp + ((c->roi[i].qp & 0x40) ? -v : v);
    }
    return qp + ((c->no_roi_qp & 0x40) ? -(c->no_roi_qp & 0x3f)
                                        : (c->no_roi_qp & 0x3f));
}

static int mb_qp(H264Enc *e, int mbx, int mby)
{
    const H264EncConfig *c = &e->cfg;
    int qp = c->qp;

    if (c->rc_en && !c->intra) {
        uint64_t budget = ((uint64_t)c->rc_u * 15 * c->mad_pred) >> 8;
        uint64_t spent = e->bits - e->hdr_bits + budget;
        uint64_t want = e->rc_target + budget;
        uint64_t ratio;
        int k = 0;

        if (want) {
            ratio = spent * 64 / want;
            while (k < 72 && rc_ratio[k + 1] <= ratio) {
                k++;
            }
            qp += k - 36;
        } else {
            qp += 36;
        }
        qp = clip3(c->qp_min, c->qp_max, qp);
        e->rc_target += budget;
    }
    qp = roi_qp(e, mbx, mby, qp);
    return clip3(QP_LOWEST, 51, qp);
}

/* Writing a macroblock */

static void put_luma_residual(H264Enc *e, MBCode *m, int mbx, int mby,
                              H264EncMB *mb)
{
    for (int k = 0; k < 16; k++) {
        int b = blk_raster[k], bx = b & 3, by = b >> 2;
        int nc;

        if (!(m->cbp_l & (1 << ((by >> 1) * 2 + (bx >> 1))))) {
            continue;
        }
        nc = nc_of(bx ? mb->nz[b - 1] : nz_at(e, mbx, mby, -1, by),
                   by ? mb->nz[b - 4] : nz_at(e, mbx, mby, bx, -1));
        if (m->type == H264E_MB_I16) {
            put_block(e, m->luma[b] + 1, 15, nc);
        } else {
            put_block(e, m->luma[b], 16, nc);
        }
    }
}

static void put_chroma_residual(H264Enc *e, MBCode *m, int mbx, int mby,
                                H264EncMB *mb)
{
    if (m->cbp_c) {
        put_block(e, m->cdc[0], 4, -1);
        put_block(e, m->cdc[1], 4, -1);
    }
    if (m->cbp_c == 2) {
        for (int c = 0; c < 2; c++) {
            for (int b = 0; b < 4; b++) {
                int bx = b & 1, by = b >> 1;
                int nc = nc_of(bx ? mb->nzc[c][b - 1]
                                  : nzc_at(e, c, mbx, mby, -1, by),
                               by ? mb->nzc[c][b - 2]
                                  : nzc_at(e, c, mbx, mby, bx, -1));

                put_block(e, m->cac[c][b] + 1, 15, nc);
            }
        }
    }
}

static int count_nz(const int *lv, int n)
{
    int c = 0;

    for (int i = 0; i < n; i++) {
        c += lv[i] != 0;
    }
    return c;
}

static void commit_mb(H264Enc *e, MBCode *m, int mbx, int mby)
{
    H264EncMB *mb = &e->mb[mby * e->cfg.mb_w + mbx];
    bool intra_slice = e->cfg.intra;
    int qp_eff;

    memset(mb, 0, sizeof(*mb));
    mb->type = m->type;
    if (m->type == H264E_MB_PSKIP) {
        mb->qp = e->qp_prev;
        mb->mv[0] = m->mv[0];
        mb->mv[1] = m->mv[1];
        e->skip_run++;
        return;
    }
    if (!intra_slice) {
        put_ue(e, e->skip_run);
        e->skip_run = 0;
    }
    /* The levels' counts, for the neighbours' contexts. */
    for (int b = 0; b < 16; b++) {
        int bx = b & 3, by = b >> 2;

        if (m->cbp_l & (1 << ((by >> 1) * 2 + (bx >> 1)))) {
            mb->nz[b] = m->type == H264E_MB_I16 ? count_nz(m->luma[b] + 1, 15)
                                                : count_nz(m->luma[b], 16);
        }
    }
    if (m->cbp_c == 2) {
        for (int c = 0; c < 2; c++) {
            for (int b = 0; b < 4; b++) {
                mb->nzc[c][b] = count_nz(m->cac[c][b] + 1, 15);
            }
        }
    }
    switch (m->type) {
    case H264E_MB_I4:
        put_ue(e, intra_slice ? 0 : 5);
        for (int k = 0; k < 16; k++) {
            int b = blk_raster[k];
            int pm = i4_pred_mode(e, m, mbx, mby, b);

            if (m->i4mode[b] == pm) {
                put_bits(e, 1, 1);
            } else {
                put_bits(e, 1, 0);
                put_bits(e, 3, m->i4mode[b] < pm ? m->i4mode[b]
                                                 : m->i4mode[b] - 1);
            }
            mb->i4[b] = m->i4mode[b];
        }
        put_ue(e, m->cmode);
        put_ue(e, cbp_code_intra[m->cbp_l | m->cbp_c << 4]);
        break;
    case H264E_MB_I16:
        put_ue(e, (intra_slice ? 1 : 6) + m->i16mode + 4 * m->cbp_c +
               (m->cbp_l ? 12 : 0));
        put_ue(e, m->cmode);
        break;
    default:
        put_ue(e, 0);
        put_se(e, m->mv[0] - m->mvp[0]);
        put_se(e, m->mv[1] - m->mvp[1]);
        put_ue(e, cbp_code_inter[m->cbp_l | m->cbp_c << 4]);
        mb->mv[0] = m->mv[0];
        mb->mv[1] = m->mv[1];
        break;
    }
    if (m->cbp_l || m->cbp_c || m->type == H264E_MB_I16) {
        put_se(e, m->qp - e->qp_prev);
        qp_eff = m->qp;
        e->qp_prev = m->qp;
    } else {
        qp_eff = e->qp_prev;
    }
    mb->qp = qp_eff;
    if (m->type == H264E_MB_I16) {
        int nc = nc_of(nz_at(e, mbx, mby, -1, 0), nz_at(e, mbx, mby, 0, -1));

        put_block(e, m->ldc, 16, nc);
        if (m->cbp_l) {
            put_luma_residual(e, m, mbx, mby, mb);
        }
    } else {
        put_luma_residual(e, m, mbx, mby, mb);
    }
    put_chroma_residual(e, m, mbx, mby, mb);
}

/* Copy macroblock (@mbx, @mby)'s reconstruction into the rec planes. */
static void store_rec(H264Enc *e, MBCode *m, int mbx, int mby)
{
    for (int y = 0; y < 16; y++) {
        memcpy(e->rec[0] + (size_t)(mby * 16 + y) * e->w + mbx * 16,
               m->ry + 16 * y, 16);
    }
    for (int y = 0; y < 8; y++) {
        memcpy(e->rec[1] + (size_t)(mby * 8 + y) * e->cw + mbx * 8,
               m->ru + 8 * y, 8);
        memcpy(e->rec[2] + (size_t)(mby * 8 + y) * e->cw + mbx * 8,
               m->rv + 8 * y, 8);
    }
}

static void load_mb(H264Enc *e, uint8_t *const planes[3], int mbx, int mby,
                    uint8_t *y16, uint8_t *u8, uint8_t *v8)
{
    for (int y = 0; y < 16; y++) {
        memcpy(y16 + 16 * y,
               planes[0] + (size_t)(mby * 16 + y) * e->w + mbx * 16, 16);
    }
    for (int y = 0; y < 8; y++) {
        memcpy(u8 + 8 * y, planes[1] + (size_t)(mby * 8 + y) * e->cw + mbx * 8,
               8);
        memcpy(v8 + 8 * y, planes[2] + (size_t)(mby * 8 + y) * e->cw + mbx * 8,
               8);
    }
}

static void code_mb(H264Enc *e, int mbx, int mby)
{
    MBCode cand[3], *best, *m;
    uint8_t sy[256], su[64], sv[64], saved[256];
    int qp = mb_qp(e, mbx, mby);
    int qpc = chroma_qp[clip3(0, 51, qp + e->cfg.chroma_qp_offset)];
    int lambda = lambda_tab[qp];
    int mvp[2] = { 0, 0 }, skip[2] = { 0, 0 };

    load_mb(e, e->src, mbx, mby, sy, su, sv);
    for (int y = 0; y < 16; y++) {
        memcpy(saved + 16 * y,
               e->rec[0] + (size_t)(mby * 16 + y) * e->w + mbx * 16, 16);
    }

    if (!e->cfg.intra) {
        mv_pred(e, mbx, mby, mvp, skip);
        /* P_Skip, if its vector is in reach and leaves nothing to code. */
        if (mv_in_range(skip[0], skip[1])) {
            m = &cand[0];
            m->qp = qp;
            m->mvp[0] = mvp[0];
            m->mvp[1] = mvp[1];
            m->mv[0] = skip[0];
            m->mv[1] = skip[1];
            mc_luma(e, mbx * 16, mby * 16, skip[0], skip[1], m->py);
            mc_chroma(e, 1, mbx * 8, mby * 8, skip[0], skip[1], m->pu);
            mc_chroma(e, 2, mbx * 8, mby * 8, skip[0], skip[1], m->pv);
            code_luma16(e, m, sy, qp, false);
            code_chroma(e, m, su, sv, qpc, false);
            if (!m->cbp_l && !m->cbp_c) {
                m->type = H264E_MB_PSKIP;
                m->sad = sad_nxn(sy, 16, m->py, 16, 16);
                best = m;
                goto done;
            }
        }
        m = &cand[0];
        m->mvp[0] = mvp[0];
        m->mvp[1] = mvp[1];
        try_inter(e, m, mbx, mby, sy, lambda);
    }
    try_i16(e, &cand[1], mbx, mby, sy, lambda);
    try_i4(e, &cand[2], mbx, mby, sy, qp, lambda);
    best = cand[2].cost < cand[1].cost ? &cand[2] : &cand[1];
    if (!e->cfg.intra &&
        (uint64_t)cand[0].cost <= (uint64_t)best->cost +
                                  e->cfg.intra_cost_offset) {
        best = &cand[0];
    }
    /* Restore the luma try_i4() reconstructed in place. */
    for (int y = 0; y < 16; y++) {
        memcpy(e->rec[0] + (size_t)(mby * 16 + y) * e->w + mbx * 16,
               saved + 16 * y, 16);
    }
    best->qp = qp;
    if (best->type == H264E_MB_P16) {
        mc_chroma(e, 1, mbx * 8, mby * 8, best->mv[0], best->mv[1], best->pu);
        mc_chroma(e, 2, mbx * 8, mby * 8, best->mv[0], best->mv[1], best->pv);
        code_luma16(e, best, sy, qp, false);
        code_chroma(e, best, su, sv, qpc, false);
        if (!best->cbp_l && !best->cbp_c && best->mv[0] == skip[0] &&
            best->mv[1] == skip[1] && mv_in_range(skip[0], skip[1])) {
            best->type = H264E_MB_PSKIP;
        }
    } else {
        choose_chroma(e, best, mbx, mby, su, sv);
        if (best->type == H264E_MB_I16) {
            code_luma16(e, best, sy, qp, true);
        }
        code_chroma(e, best, su, sv, qpc, true);
    }
done:
    if (best->type == H264E_MB_PSKIP) {
        memcpy(best->ry, best->py, 256);
        memcpy(best->ru, best->pu, 64);
        memcpy(best->rv, best->pv, 64);
    }
    best->qp = qp;
    commit_mb(e, best, mbx, mby);
    store_rec(e, best, mbx, mby);
    e->mad_sum += best->sad / 256;
    e->qp_sum += e->mb[mby * e->cfg.mb_w + mbx].qp;
    if (!e->cfg.intra && e->cfg.mv_merge_en &&
        (best->type == H264E_MB_P16 || best->type == H264E_MB_PSKIP) &&
        e->cfg.mv_merge_type != 3) {
        int mx = best->mv[0], my = best->mv[1];
        bool keep = e->cfg.int_mv_out ? (abs(mx) >= 4 || abs(my) >= 4)
                                      : (mx || my);

        if (keep) {
            e->mvm[e->mvm_n++] = (my & 0x7f) | (mx & 0xff) << 7 |
                                 (uint32_t)(mby & 0x7f) << 15 |
                                 (uint32_t)(mbx & 0x7f) << 22;
            e->mvm_total++;
        }
    }
}

/* The deblocking filter (8.7) */

static int bs_of(H264Enc *e, int mbx_p, int mby_p, int bp, int mbx_q,
                 int mby_q, int bq, bool mb_edge)
{
    H264EncMB *p = &e->mb[mby_p * e->cfg.mb_w + mbx_p];
    H264EncMB *q = &e->mb[mby_q * e->cfg.mb_w + mbx_q];
    bool pi = p->type <= H264E_MB_I16, qi = q->type <= H264E_MB_I16;

    if (pi || qi) {
        return mb_edge ? 4 : 3;
    }
    if (p->nz[bp] || q->nz[bq]) {
        return 2;
    }
    if (abs(p->mv[0] - q->mv[0]) >= 4 || abs(p->mv[1] - q->mv[1]) >= 4) {
        return 1;
    }
    return 0;
}

static void filter_line(uint8_t *s, int step, int bs, int alpha, int beta,
                        int tc0, bool chroma)
{
    int p0 = s[-step], p1 = s[-2 * step], q0 = s[0], q1 = s[step];

    if (abs(p0 - q0) >= alpha || abs(p1 - p0) >= beta ||
        abs(q1 - q0) >= beta) {
        return;
    }
    if (chroma) {
        if (bs < 4) {
            int tc = tc0 + 1;
            int d = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);

            s[-step] = clip1(p0 + d);
            s[0] = clip1(q0 - d);
        } else {
            s[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
            s[0] = (2 * q1 + q0 + p1 + 2) >> 2;
        }
        return;
    }
    {
        int p2 = s[-3 * step], q2 = s[2 * step];
        int ap = abs(p2 - p0), aq = abs(q2 - q0);

        if (bs < 4) {
            int tc = tc0 + (ap < beta) + (aq < beta);
            int d = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);

            s[-step] = clip1(p0 + d);
            s[0] = clip1(q0 - d);
            if (ap < beta) {
                s[-2 * step] = p1 + clip3(-tc0, tc0,
                                          (p2 + ((p0 + q0 + 1) >> 1) -
                                           (p1 << 1)) >> 1);
            }
            if (aq < beta) {
                s[step] = q1 + clip3(-tc0, tc0,
                                     (q2 + ((p0 + q0 + 1) >> 1) -
                                      (q1 << 1)) >> 1);
            }
        } else {
            int p3 = s[-4 * step], q3 = s[3 * step];
            bool strong = abs(p0 - q0) < ((alpha >> 2) + 2);

            if (ap < beta && strong) {
                s[-step] = (p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3;
                s[-2 * step] = (p2 + p1 + p0 + q0 + 2) >> 2;
                s[-3 * step] = (2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3;
            } else {
                s[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
            }
            if (aq < beta && strong) {
                s[0] = (p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3;
                s[step] = (p0 + q0 + q1 + q2 + 2) >> 2;
                s[2 * step] = (2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3;
            } else {
                s[0] = (2 * q1 + q0 + p1 + 2) >> 2;
            }
        }
    }
}

/*
 * Filter one edge of macroblock (@mbx, @mby): vertical (@vert) or
 * horizontal, at luma offset @off (0, 4, 8, 12) within it.
 */
static void filter_edge(H264Enc *e, int mbx, int mby, bool vert, int off)
{
    bool mb_edge = off == 0;
    int pmx = mbx, pmy = mby, bs[4];
    H264EncMB *q = &e->mb[mby * e->cfg.mb_w + mbx], *p;

    if (mb_edge) {
        if (vert) {
            pmx--;
        } else {
            pmy--;
        }
    }
    p = &e->mb[pmy * e->cfg.mb_w + pmx];
    for (int i = 0; i < 4; i++) {
        int bq = vert ? i * 4 + off / 4 : (off / 4) * 4 + i;
        int bp = vert ? i * 4 + (mb_edge ? 3 : off / 4 - 1)
                      : (mb_edge ? 3 : off / 4 - 1) * 4 + i;

        bs[i] = bs_of(e, pmx, pmy, bp, mbx, mby, bq, mb_edge);
    }
    for (int plane = 0; plane < 3; plane++) {
        int qpp, qpq, qpav, ia, ib, alpha, beta, n, stride, coff;
        uint8_t *base;

        if (plane) {
            if (off & 4) {
                continue;
            }
            qpp = chroma_qp[clip3(0, 51, p->qp + e->cfg.chroma_qp_offset)];
            qpq = chroma_qp[clip3(0, 51, q->qp + e->cfg.chroma_qp_offset)];
            stride = e->cw;
            coff = off / 2;
            base = e->out[plane] + (size_t)mby * 8 * stride + mbx * 8;
            n = 8;
        } else {
            qpp = p->qp;
            qpq = q->qp;
            stride = e->w;
            coff = off;
            base = e->out[0] + (size_t)mby * 16 * stride + mbx * 16;
            n = 16;
        }
        qpav = (qpp + qpq + 1) >> 1;
        ia = clip3(0, 51, qpav);
        ib = clip3(0, 51, qpav);
        alpha = db_alpha[ia];
        beta = db_beta[ib];
        for (int k = 0; k < n; k++) {
            int b = bs[plane ? k / 2 : k / 4];
            uint8_t *s;

            if (!b) {
                continue;
            }
            s = vert ? base + k * stride + coff : base + coff * stride + k;
            filter_line(s, vert ? 1 : stride, b, alpha, beta,
                        b < 4 ? db_tc0[ia][b - 1] : 0, plane != 0);
        }
    }
}

static void deblock_row(H264Enc *e, int r)
{
    for (int x = 0; x < e->cfg.mb_w; x++) {
        for (int off = 0; off < 16; off += 4) {
            if (off || x) {
                filter_edge(e, x, r, true, off);
            }
        }
        for (int off = 0; off < 16; off += 4) {
            if (off || r) {
                filter_edge(e, x, r, false, off);
            }
        }
    }
}

void h264e_code_row(H264Enc *e, int r)
{
    e->mvm_n = 0;
    for (int x = 0; x < e->cfg.mb_w; x++) {
        code_mb(e, x, r);
    }
    for (int y = 0; y < 16; y++) {
        memcpy(e->out[0] + (size_t)(r * 16 + y) * e->w,
               e->rec[0] + (size_t)(r * 16 + y) * e->w, e->w);
    }
    for (int c = 1; c < 3; c++) {
        for (int y = 0; y < 8; y++) {
            memcpy(e->out[c] + (size_t)(r * 8 + y) * e->cw,
                   e->rec[c] + (size_t)(r * 8 + y) * e->cw, e->cw);
        }
    }
    if (e->cfg.deblock) {
        deblock_row(e, r);
    }
}
