/*
 * ESP32-P4 JPEG codec: baseline JPEG transforms and entropy coding
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The arithmetic of the codec's pipeline (TRM 37.4), free of the device:
 * colour conversion, the 8x8 forward and inverse DCT, quantisation, and
 * baseline Huffman coding (ITU-T T.81, annexes C, F and K), with the
 * encoder's stuffing and the decoder's handling of markers in the coded
 * data. hw/misc/esp32p4_jpeg.c times and feeds it.
 */

#ifndef HW_MISC_ESP32P4_JPEG_CODEC_H
#define HW_MISC_ESP32P4_JPEG_CODEC_H

/*
 * The natural (row-major) index of each coefficient in zigzag order, with
 * 16 guard entries past the end where a run that overshoots the block's
 * last coefficient lands.
 */
extern const uint8_t esp32p4_jpeg_natural[64 + 16];

/*
 * A Huffman table as a DHT segment gives it: the number of codes of each
 * length 1 to 16, and the symbols in order of their codes.
 */
typedef struct ESP32P4JpegHuffSpec {
    uint8_t bits[16];
    uint8_t vals[256];
} ESP32P4JpegHuffSpec;

/*
 * T.81 annex K's tables (K.3 to K.6): [0] luminance, [1] chrominance.
 * The encoder codes with these; it has no Huffman table of its own.
 */
extern const ESP32P4JpegHuffSpec esp32p4_jpeg_std_dc[2];
extern const ESP32P4JpegHuffSpec esp32p4_jpeg_std_ac[2];

/* The encoder's code for each symbol: size 0 where it has none. */
typedef struct ESP32P4JpegHuffCode {
    uint16_t code[256];
    uint8_t size[256];
} ESP32P4JpegHuffCode;

void esp32p4_jpeg_huff_code_build(ESP32P4JpegHuffCode *c,
                                  const ESP32P4JpegHuffSpec *spec);

/*
 * A decoding table, built from what the decoder's Huffman RAM holds (TRM
 * 37.5.2.4): for each code length L, the number of codes of length 1 to
 * L in total (totlen) and the first code of length L, left-aligned in 16
 * bits, 0xFFFF where there is none (codemin); then the symbols.
 */
typedef struct ESP32P4JpegHuffDec {
    int32_t mincode[17];
    int32_t maxcode[17];        /* -1 where there is no code */
    int32_t valptr[17];
    uint8_t vals[256];
} ESP32P4JpegHuffDec;

void esp32p4_jpeg_huff_dec_build(ESP32P4JpegHuffDec *d,
                                 const uint16_t totlen[16],
                                 const uint16_t codemin[16],
                                 const uint8_t *vals, unsigned nvals);

/*
 * The forward DCT of one data unit of level-shifted samples, in place,
 * as eight times the transform; the inverse DCT of dequantised
 * coefficients to samples, limited to 0..255. Both are the
 * Loeffler-Ligtenberg-Moschytz factorisation in 13-bit fixed point, with
 * 2 extra bits of precision between the passes and rounding at each
 * descaling: libjpeg's "islow" DCTs.
 */
void esp32p4_jpeg_fdct(int32_t blk[64]);
void esp32p4_jpeg_idct(const int32_t coef[64], uint8_t out[64]);

/*
 * Quantise eight times a coefficient by @q, rounding half away from
 * zero, as the encoder's quantizer divides.
 */
static inline int32_t esp32p4_jpeg_quantize(int32_t v, uint32_t q)
{
    uint32_t d = q << 3;

    if (v < 0) {
        return -(int32_t)((uint32_t)(-v + (d >> 1)) / d);
    }
    return (int32_t)((uint32_t)(v + (d >> 1)) / d);
}

/*
 * RGB to YCbCr (TRM 37.5.1.2's formulas) in 16-bit fixed point, rounded
 * to nearest, as JFIF encoders compute them.
 */
void esp32p4_jpeg_rgb_to_ycc(int r, int g, int b, uint8_t *y, uint8_t *cb,
                             uint8_t *cr);

/* Coded data out: Huffman codes packed MSB first, 0xFF stuffed. */
typedef struct ESP32P4JpegBitOut {
    uint8_t *buf;
    size_t len;
    size_t cap;
    uint64_t acc;
    int nbits;
    bool stuff;
} ESP32P4JpegBitOut;

void esp32p4_jpeg_put_bits(ESP32P4JpegBitOut *o, uint32_t bits, int n);
/* Pad the last byte with 1 bits. */
void esp32p4_jpeg_put_pad(ESP32P4JpegBitOut *o);

/*
 * Code one data unit's quantised coefficients (natural order) after the
 * DC prediction @pred, which it updates.
 */
void esp32p4_jpeg_encode_block(ESP32P4JpegBitOut *o, const int32_t q[64],
                               int32_t *pred, const ESP32P4JpegHuffCode *dc,
                               const ESP32P4JpegHuffCode *ac);

/*
 * Coded data in. @buf[@pos..@len) is what has arrived; @eof says no more
 * will. Bits are taken one byte at a time as the decoder needs them, a
 * stuffed 0xFF00 giving 0xFF, fill bytes (0xFF) skipped. A marker stops
 * the data: the reader stays at the marker's first 0xFF.
 */
typedef struct ESP32P4JpegBitIn {
    const uint8_t *buf;
    size_t len;
    size_t pos;
    uint32_t acc;
    int nbits;
    bool eof;
} ESP32P4JpegBitIn;

enum {
    ESP32P4_JPEG_DEC_OK,
    ESP32P4_JPEG_DEC_MORE,      /* out of bytes; more will come */
    ESP32P4_JPEG_DEC_END,       /* out of bytes; none will */
    ESP32P4_JPEG_DEC_MARKER,    /* a marker interrupts the data */
};

/* What decoding an MCU found besides its data. */
#define ESP32P4_JPEG_DEC_RUN_OVERFLOW   (1u << 0)
#define ESP32P4_JPEG_DEC_BAD_CODE       (1u << 1)

/* A component of a scan: its tables and data units per MCU. */
typedef struct ESP32P4JpegDecComp {
    const ESP32P4JpegHuffDec *dc;
    const ESP32P4JpegHuffDec *ac;
    const uint16_t *q;          /* natural order */
    uint32_t qmask;             /* the table's precision */
    int units;
} ESP32P4JpegDecComp;

/*
 * Decode one MCU: each component's data units in turn, 64 samples each,
 * row by row, to @out. Returns ESP32P4_JPEG_DEC_*; on anything but OK the
 * reader and @pred are left part-way, for the caller to restore. ORs
 * ESP32P4_JPEG_DEC_RUN_OVERFLOW and _BAD_CODE into *@flags.
 */
int esp32p4_jpeg_decode_mcu(ESP32P4JpegBitIn *in,
                            const ESP32P4JpegDecComp *comp, int ncomp,
                            int32_t pred[4], uint8_t *out, uint32_t *flags);

/*
 * Drop the bits left of the byte being read, as at the end of a restart
 * interval or of the scan.
 */
static inline void esp32p4_jpeg_bits_align(ESP32P4JpegBitIn *in)
{
    in->nbits = 0;
    in->acc = 0;
}

#endif
