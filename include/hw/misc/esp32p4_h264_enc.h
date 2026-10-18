/*
 * ESP32-P4 H.264 encoder core (ENC_CORE): a baseline-profile CAVLC encoder
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The encoder algorithm core of the P4's H264 encoder (TRM chapter 41,
 * section 41.5.1), apart from the registers and the dedicated DMA, which
 * hw/misc/esp32p4_h264.c models: it codes a picture one macroblock row at
 * a time, as the hardware's pipeline does, from a source row and the
 * reference rows the DMA brings in, and gives back the bits of the slice
 * data, the reconstructed and deblocked rows the DMA takes out, the
 * motion vectors MV merging keeps, and the rate control statistics.
 *
 * The caller owns the data path. Before coding row r it loads the row's
 * source pixels (h264e_load_source_row() or h264e_fill_source_row()),
 * for a P picture the reference rows r - 1 to r + 1 into the reference
 * planes, and for r > 0 the four bottom lines of row r - 1 as the
 * deblocking filter left them (h264e_line4_put()); h264e_code_row() then
 * codes the row, reconstructs and deblocks it. The deblocked picture is
 * in the "out" planes, from which the caller takes row r's top twelve
 * lines (row r - 1 having been finished by row r's top edge), and the
 * bottom four for the next row.
 *
 * Nothing here depends on QEMU beyond qemu/osdep.h, so that the core can
 * be built and exercised on its own.
 */

#ifndef HW_MISC_ESP32P4_H264_ENC_H
#define HW_MISC_ESP32P4_H264_ENC_H

/* The largest picture the registers can describe, in macroblocks. */
#define H264E_MAX_MB_W      127
#define H264E_MAX_MB_H      127

/* The original picture's colour spaces (H264_x_ORI_COLOR_SPACE). */
enum {
    H264E_CS_RGB888 = 0,
    H264E_CS_RGB565 = 1,
    H264E_CS_YUV444 = 2,
    H264E_CS_YUV422 = 3,
    H264E_CS_YUV420 = 4,
    H264E_CS_GRAY = 5,
};

/* Macroblock types as the core decides them. */
enum {
    H264E_MB_I4 = 0,
    H264E_MB_I16 = 1,
    H264E_MB_P16 = 2,
    H264E_MB_PSKIP = 3,
};

typedef struct H264EncRoi {
    bool en;
    int x, y, w, h;
    uint8_t qp;                 /* the 7-bit field */
} H264EncRoi;

/* What one picture is coded with: the register set of its stream. */
typedef struct H264EncConfig {
    int mb_w, mb_h;
    bool intra;                 /* an I picture */
    int qp;                     /* H264_x_QP: the slice's QP */
    bool rc_en;                 /* H264_x_MB_RATE_CTRL_EN */
    uint32_t rc_u;              /* H264_x_RATE_CTRL_U */
    uint32_t mad_pred;          /* H264_x_MAD_FRAME_PRED */
    int qp_min, qp_max;         /* H264_x_QP_MIN, _MAX */
    int chroma_qp_offset;       /* H264_x_CHROMA_QP_DELTA, signed */
    bool deblock;               /* !H264_x_BYPASS_DB_FILTER */
    uint32_t intra_cost_offset; /* H264_x_INTRA_COST_CMP_OFFSET */
    int l_deci_score, c_deci_score;
    int i16_deci_offset, ic_deci_offset, p16_deci_offset, pc_deci_offset;
    bool roi_en;
    bool roi_delta;             /* H264_x_ROI_MODE: QP offsets */
    H264EncRoi roi[8];
    uint8_t no_roi_qp;          /* the 7-bit field */
    bool mv_merge_en;
    int mv_merge_type;
    bool int_mv_out;
} H264EncConfig;

typedef struct H264EncMB {
    uint8_t type;
    uint8_t qp;                 /* QPY as the decoder takes it */
    int16_t mv[2];              /* quarter samples, inter macroblocks */
    uint8_t nz[16];             /* luma 4x4 blocks' TotalCoeff, raster */
    uint8_t nzc[2][4];          /* chroma AC blocks' TotalCoeff */
    uint8_t i4[16];             /* Intra4x4PredMode, raster */
} H264EncMB;

typedef struct H264Enc {
    H264EncConfig cfg;
    int w, h, cw, ch;           /* the planes, whole macroblocks */
    uint8_t *src[3];            /* the picture being coded */
    uint8_t *ref[3];            /* the reference picture */
    uint8_t *rec[3];            /* reconstruction before deblocking */
    uint8_t *out[3];            /* reconstruction after deblocking */
    uint32_t ly, lc;            /* a luma and a chroma plane's bytes */
    H264EncMB *mb;
    int32_t mb_count;
    int32_t alloc_w, alloc_h;

    /* The RBSP bits not yet taken, and emulation prevention's state. */
    uint8_t *rbsp;
    uint32_t rbsp_len, rbsp_cap;
    uint32_t acc;
    int acc_bits;
    int zeros;

    int qp_prev;                /* QPY,PRED */
    int skip_run;
    uint64_t bits;              /* the bits of the picture so far */
    uint64_t hdr_bits;          /* ... of which its slice header's */
    uint64_t rc_target;         /* the rate control's budget so far */

    /* Rate control status (H264_RC_STATUS0..2). */
    uint32_t mad_sum;
    uint64_t enc_bits;
    uint32_t qp_sum;

    /* The motion vectors MV merging kept from the last row coded. */
    uint32_t mvm[H264E_MAX_MB_W];
    int mvm_n;
    uint32_t mvm_total;
} H264Enc;

void h264e_init(H264Enc *e);
void h264e_free(H264Enc *e);

/*
 * Begin a picture: its configuration, and the first @hdr_len bytes and
 * @rem_len further bits (the top bits of @rem) of its slice header, which
 * the picture's bitstream begins with.
 */
void h264e_begin(H264Enc *e, const H264EncConfig *cfg, const uint8_t *hdr,
                 int hdr_len, uint8_t rem, int rem_len);

/*
 * Load source row @r from @lines lines of @line_bytes bytes each at
 * @data, in colour space @cs, @width pixels wide: converted to 4:2:0 as
 * the TRM gives (41.5.1.2), the macroblocks beyond the picture's right
 * and bottom edges filled by repeating its last column and line. With
 * @replace, every pixel is replaced by @replace_px (the debug function:
 * B, G, R or V, U, Y in bytes 0 to 2, or the grey value in byte 0).
 */
void h264e_load_source_row(H264Enc *e, int r, const uint8_t *data,
                           size_t line_bytes, int lines, int width, int cs,
                           bool replace, uint32_t replace_px);
/* Fill source row @r with one colour. */
void h264e_fill_source_row(H264Enc *e, int r, uint8_t y, uint8_t u,
                           uint8_t v);

/*
 * The four bottom lines of a macroblock row, as the deblocking filter
 * leaves them for the next: 128 bytes per macroblock (16 x 4 luma, then
 * 8 x 4 Cb and 8 x 4 Cr), which the DMA keeps in memory between rows.
 */
#define H264E_LINE4_BYTES   128
#define H264E_LINE12_BYTES  256
#define H264E_MB_BYTES      384

void h264e_line4_get(H264Enc *e, int r, int x, uint8_t *buf);
void h264e_line4_put(H264Enc *e, int r, int x, const uint8_t *buf);
/*
 * The deblocked macroblock (@r, @x) in the layout the DMA writes and the
 * reference fetch reads: its top twelve lines (16 x 12 luma, 8 x 4 Cb,
 * 8 x 4 Cr) and then its bottom four as above.
 */
void h264e_mb_get(H264Enc *e, int r, int x, uint8_t *buf, bool top,
                  bool bottom);
/* Put reference macroblock (@r, @x), in that layout, into the reference. */
void h264e_ref_put(H264Enc *e, int r, int x, const uint8_t *buf);

/* Code row @r, reconstruct and deblock it. */
void h264e_code_row(H264Enc *e, int r);

/* End the picture: its slice's trailing bits. */
void h264e_finish(H264Enc *e);

/*
 * Take up to @max of the bytes of the bitstream completed so far, with
 * emulation prevention bytes in place.
 */
size_t h264e_take(H264Enc *e, uint8_t *buf, size_t max);
/* The bytes h264e_take() would give for all that is pending. */
size_t h264e_take_len(const H264Enc *e);
size_t h264e_pending(H264Enc *e);

#endif
