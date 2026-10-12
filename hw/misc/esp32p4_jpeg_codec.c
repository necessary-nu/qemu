/*
 * ESP32-P4 JPEG codec: baseline JPEG transforms and entropy coding
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ITU-T T.81 (the JPEG baseline process), annexes A.3.3 (the
 * DCT), C (Huffman tables), F.1.2 and F.2.2 (baseline coding and
 * decoding) and K (the example tables); ESP32-P4 TRM chapter 37.
 *
 * The TRM does not say how the codec rounds. The model computes as the
 * reference software does: the DCTs are the Loeffler-Ligtenberg-Moschytz
 * factorisation with 13-bit constants (IEEE 1180 accurate), colour
 * conversion is 16-bit fixed point rounded to nearest, and the quantizer
 * rounds half away from zero. An encoder exact to the transform would
 * differ from this one by at most one quantisation step in rare
 * coefficients, and a decoder's samples by at most one level.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "hw/misc/esp32p4_jpeg_codec.h"

/* [spec:nuos:req:emu.esp32p4.jpeg] */
const uint8_t esp32p4_jpeg_natural[64 + 16] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
};

/* T.81 tables K.3 to K.6. */
const ESP32P4JpegHuffSpec esp32p4_jpeg_std_dc[2] = {
    {
        .bits = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
        .vals = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
    }, {
        .bits = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
        .vals = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
    },
};

const ESP32P4JpegHuffSpec esp32p4_jpeg_std_ac[2] = {
    {
        .bits = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d },
        .vals = {
            0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
            0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
            0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
            0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
            0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
            0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
            0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
            0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
            0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
            0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
            0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
            0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
            0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
            0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
            0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
            0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
            0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
            0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
            0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
            0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
            0xf9, 0xfa,
        },
    }, {
        .bits = { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 },
        .vals = {
            0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
            0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
            0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
            0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
            0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
            0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
            0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
            0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
            0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
            0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
            0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
            0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
            0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
            0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
            0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
            0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
            0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
            0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
            0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
            0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
            0xf9, 0xfa,
        },
    },
};

/* T.81 C.2: codes assigned in order of length, counting up. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_huff_code_build(ESP32P4JpegHuffCode *c,
                                  const ESP32P4JpegHuffSpec *spec)
{
    uint32_t code = 0;
    int k = 0;

    memset(c, 0, sizeof(*c));
    for (int len = 1; len <= 16; len++) {
        for (int i = 0; i < spec->bits[len - 1] && k < 256; i++, k++) {
            c->code[spec->vals[k]] = code++;
            c->size[spec->vals[k]] = len;
        }
        code <<= 1;
    }
}

/*
 * T.81 F.2.2.3's MINCODE, MAXCODE and VALPTR from the RAM's totlen and
 * codemin: the codes of length L are codemin's top L bits counting up,
 * totlen[L] - totlen[L - 1] of them, their symbols from totlen[L - 1].
 */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_huff_dec_build(ESP32P4JpegHuffDec *d,
                                 const uint16_t totlen[16],
                                 const uint16_t codemin[16],
                                 const uint8_t *vals, unsigned nvals)
{
    int32_t prev = 0;

    memset(d, 0, sizeof(*d));
    memcpy(d->vals, vals, MIN(nvals, sizeof(d->vals)));
    for (int len = 1; len <= 16; len++) {
        int32_t total = totlen[len - 1];
        int32_t count = total - prev;

        if (count > 0) {
            d->mincode[len] = codemin[len - 1] >> (16 - len);
            d->maxcode[len] = d->mincode[len] + count - 1;
            d->valptr[len] = prev;
        } else {
            d->maxcode[len] = -1;
        }
        if (total > prev) {
            prev = total;
        }
    }
}

#define CONST_BITS  13
#define PASS1_BITS  2

#define FIX_0_298631336 2446
#define FIX_0_390180644 3196
#define FIX_0_541196100 4433
#define FIX_0_765366865 6270
#define FIX_0_899976223 7373
#define FIX_1_175875602 9633
#define FIX_1_501321110 12299
#define FIX_1_847759065 15137
#define FIX_1_961570560 16069
#define FIX_2_053119869 16819
#define FIX_2_562915447 20995
#define FIX_3_072711026 25172

static inline int64_t descale(int64_t x, int n)
{
    return (x + ((int64_t)1 << (n - 1))) >> n;
}

/*
 * One row or column of the forward DCT: @p[@s * i], i = 0..7. The even
 * part comes out as (sum << @even_shift), descaled by @even_descale when
 * that is not 0; the odd part's products descale by @odd_descale.
 */
static void fdct_1d(int32_t *p, int s, int even_shift, int even_descale,
                    int odd_descale)
{
    int64_t tmp0 = p[0 * s] + p[7 * s], tmp7 = p[0 * s] - p[7 * s];
    int64_t tmp1 = p[1 * s] + p[6 * s], tmp6 = p[1 * s] - p[6 * s];
    int64_t tmp2 = p[2 * s] + p[5 * s], tmp5 = p[2 * s] - p[5 * s];
    int64_t tmp3 = p[3 * s] + p[4 * s], tmp4 = p[3 * s] - p[4 * s];
    int64_t tmp10 = tmp0 + tmp3, tmp13 = tmp0 - tmp3;
    int64_t tmp11 = tmp1 + tmp2, tmp12 = tmp1 - tmp2;
    int64_t z1, z2, z3, z4, z5;

    if (even_descale) {
        p[0 * s] = descale(tmp10 + tmp11, even_descale);
        p[4 * s] = descale(tmp10 - tmp11, even_descale);
    } else {
        p[0 * s] = (tmp10 + tmp11) << even_shift;
        p[4 * s] = (tmp10 - tmp11) << even_shift;
    }
    z1 = (tmp12 + tmp13) * FIX_0_541196100;
    p[2 * s] = descale(z1 + tmp13 * FIX_0_765366865, odd_descale);
    p[6 * s] = descale(z1 - tmp12 * FIX_1_847759065, odd_descale);

    z1 = tmp4 + tmp7;
    z2 = tmp5 + tmp6;
    z3 = tmp4 + tmp6;
    z4 = tmp5 + tmp7;
    z5 = (z3 + z4) * FIX_1_175875602;
    tmp4 *= FIX_0_298631336;
    tmp5 *= FIX_2_053119869;
    tmp6 *= FIX_3_072711026;
    tmp7 *= FIX_1_501321110;
    z1 *= -FIX_0_899976223;
    z2 *= -FIX_2_562915447;
    z3 = z3 * -FIX_1_961570560 + z5;
    z4 = z4 * -FIX_0_390180644 + z5;
    p[7 * s] = descale(tmp4 + z1 + z3, odd_descale);
    p[5 * s] = descale(tmp5 + z2 + z4, odd_descale);
    p[3 * s] = descale(tmp6 + z2 + z3, odd_descale);
    p[1 * s] = descale(tmp7 + z1 + z4, odd_descale);
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_fdct(int32_t blk[64])
{
    for (int r = 0; r < 8; r++) {
        fdct_1d(blk + 8 * r, 1, PASS1_BITS, 0, CONST_BITS - PASS1_BITS);
    }
    for (int c = 0; c < 8; c++) {
        fdct_1d(blk + c, 8, 0, PASS1_BITS, CONST_BITS + PASS1_BITS);
    }
}

/*
 * One column or row of the inverse DCT, @in[@s * i] to @out[@s * i],
 * descaling by @n.
 */
static void idct_1d(const int64_t *in, int s, int64_t *out, int n)
{
    int64_t z1, z2, z3, z4, z5;
    int64_t tmp0, tmp1, tmp2, tmp3, tmp10, tmp11, tmp12, tmp13;

    z2 = in[2 * s];
    z3 = in[6 * s];
    z1 = (z2 + z3) * FIX_0_541196100;
    tmp2 = z1 - z3 * FIX_1_847759065;
    tmp3 = z1 + z2 * FIX_0_765366865;
    tmp0 = (in[0] + in[4 * s]) << CONST_BITS;
    tmp1 = (in[0] - in[4 * s]) << CONST_BITS;
    tmp10 = tmp0 + tmp3;
    tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;
    tmp12 = tmp1 - tmp2;

    tmp0 = in[7 * s];
    tmp1 = in[5 * s];
    tmp2 = in[3 * s];
    tmp3 = in[1 * s];
    z1 = tmp0 + tmp3;
    z2 = tmp1 + tmp2;
    z3 = tmp0 + tmp2;
    z4 = tmp1 + tmp3;
    z5 = (z3 + z4) * FIX_1_175875602;
    tmp0 *= FIX_0_298631336;
    tmp1 *= FIX_2_053119869;
    tmp2 *= FIX_3_072711026;
    tmp3 *= FIX_1_501321110;
    z1 *= -FIX_0_899976223;
    z2 *= -FIX_2_562915447;
    z3 = z3 * -FIX_1_961570560 + z5;
    z4 = z4 * -FIX_0_390180644 + z5;
    tmp0 += z1 + z3;
    tmp1 += z2 + z4;
    tmp2 += z2 + z3;
    tmp3 += z1 + z4;

    out[0 * s] = descale(tmp10 + tmp3, n);
    out[7 * s] = descale(tmp10 - tmp3, n);
    out[1 * s] = descale(tmp11 + tmp2, n);
    out[6 * s] = descale(tmp11 - tmp2, n);
    out[2 * s] = descale(tmp12 + tmp1, n);
    out[5 * s] = descale(tmp12 - tmp1, n);
    out[3 * s] = descale(tmp13 + tmp0, n);
    out[4 * s] = descale(tmp13 - tmp0, n);
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_idct(const int32_t coef[64], uint8_t out[64])
{
    int64_t in[64], ws[64], row[8];

    for (int i = 0; i < 64; i++) {
        in[i] = coef[i];
    }
    for (int c = 0; c < 8; c++) {
        idct_1d(in + c, 8, ws + c, CONST_BITS - PASS1_BITS);
    }
    for (int r = 0; r < 8; r++) {
        idct_1d(ws + 8 * r, 1, row, CONST_BITS + PASS1_BITS + 3);
        for (int c = 0; c < 8; c++) {
            out[8 * r + c] = MIN(MAX(row[c] + 128, 0), 255);
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_rgb_to_ycc(int r, int g, int b, uint8_t *y, uint8_t *cb,
                             uint8_t *cr)
{
    const int32_t half = 1 << 15, offset = 128 << 16;

    *y = (19595 * r + 38470 * g + 7471 * b + half) >> 16;
    *cb = (-11059 * r - 21709 * g + 32768 * b + offset + half - 1) >> 16;
    *cr = (32768 * r - 27439 * g - 5329 * b + offset + half - 1) >> 16;
}

static void put_byte(ESP32P4JpegBitOut *o, uint8_t b)
{
    if (o->len < o->cap) {
        o->buf[o->len] = b;
    }
    o->len++;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_put_bits(ESP32P4JpegBitOut *o, uint32_t bits, int n)
{
    if (!n) {
        return;
    }
    o->acc = (o->acc << n) | (bits & ((1u << n) - 1));
    o->nbits += n;
    while (o->nbits >= 8) {
        uint8_t b = o->acc >> (o->nbits - 8);

        o->nbits -= 8;
        put_byte(o, b);
        if (b == 0xff && o->stuff) {
            put_byte(o, 0);
        }
    }
    o->acc &= (1u << o->nbits) - 1;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_put_pad(ESP32P4JpegBitOut *o)
{
    if (o->nbits) {
        esp32p4_jpeg_put_bits(o, 0x7f, 8 - o->nbits);
    }
}

/* The magnitude category of @v and its additional bits (T.81 F.1.2.1). */
static int magnitude(int32_t v, uint32_t *extra)
{
    uint32_t a = v < 0 ? -v : v;
    int n = a ? 32 - clz32(a) : 0;

    *extra = (v < 0 ? (uint32_t)(v - 1) : (uint32_t)v) & ((1u << n) - 1);
    return n;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_encode_block(ESP32P4JpegBitOut *o, const int32_t q[64],
                               int32_t *pred, const ESP32P4JpegHuffCode *dc,
                               const ESP32P4JpegHuffCode *ac)
{
    uint32_t extra;
    int n, run = 0;

    n = magnitude(q[0] - *pred, &extra);
    *pred = q[0];
    esp32p4_jpeg_put_bits(o, dc->code[n], dc->size[n]);
    esp32p4_jpeg_put_bits(o, extra, n);

    for (int k = 1; k < 64; k++) {
        int32_t v = q[esp32p4_jpeg_natural[k]];
        int sym;

        if (!v) {
            run++;
            continue;
        }
        while (run > 15) {
            esp32p4_jpeg_put_bits(o, ac->code[0xf0], ac->size[0xf0]);
            run -= 16;
        }
        n = magnitude(v, &extra);
        sym = (run << 4) | n;
        esp32p4_jpeg_put_bits(o, ac->code[sym], ac->size[sym]);
        esp32p4_jpeg_put_bits(o, extra, n);
        run = 0;
    }
    if (run) {
        esp32p4_jpeg_put_bits(o, ac->code[0], ac->size[0]);
    }
}

/* Take the next byte of coded data into the reader. */
static int bits_fill(ESP32P4JpegBitIn *in)
{
    size_t p;

    if (in->pos >= in->len) {
        return in->eof ? ESP32P4_JPEG_DEC_END : ESP32P4_JPEG_DEC_MORE;
    }
    if (in->buf[in->pos] != 0xff) {
        in->acc = (in->acc << 8) | in->buf[in->pos++];
        in->nbits += 8;
        return ESP32P4_JPEG_DEC_OK;
    }
    for (p = in->pos + 1; p < in->len && in->buf[p] == 0xff; p++) {
        /* fill bytes */
    }
    if (p >= in->len) {
        return in->eof ? ESP32P4_JPEG_DEC_END : ESP32P4_JPEG_DEC_MORE;
    }
    if (in->buf[p]) {
        in->pos = p - 1;
        return ESP32P4_JPEG_DEC_MARKER;
    }
    in->acc = (in->acc << 8) | 0xff;
    in->nbits += 8;
    in->pos = p + 1;
    return ESP32P4_JPEG_DEC_OK;
}

static int get_bit(ESP32P4JpegBitIn *in, uint32_t *bit)
{
    if (!in->nbits) {
        int r = bits_fill(in);

        if (r != ESP32P4_JPEG_DEC_OK) {
            return r;
        }
    }
    in->nbits--;
    *bit = (in->acc >> in->nbits) & 1;
    return ESP32P4_JPEG_DEC_OK;
}

static int get_bits(ESP32P4JpegBitIn *in, int n, uint32_t *v)
{
    uint32_t bit;

    *v = 0;
    for (int i = 0; i < n; i++) {
        int r = get_bit(in, &bit);

        if (r != ESP32P4_JPEG_DEC_OK) {
            return r;
        }
        *v = (*v << 1) | bit;
    }
    return ESP32P4_JPEG_DEC_OK;
}

/*
 * T.81 F.2.2.3's DECODE. A code no table entry matches by 16 bits is
 * taken as symbol 0.
 */
static int huff_decode(ESP32P4JpegBitIn *in, const ESP32P4JpegHuffDec *d,
                       int *sym, uint32_t *flags)
{
    int32_t code = 0;
    uint32_t bit;

    for (int len = 1; len <= 16; len++) {
        int r = get_bit(in, &bit);

        if (r != ESP32P4_JPEG_DEC_OK) {
            return r;
        }
        code = (code << 1) | bit;
        if (code <= d->maxcode[len]) {
            int32_t idx = d->valptr[len] + code - d->mincode[len];

            if (idx < 0 || idx > 255) {
                break;
            }
            *sym = d->vals[idx];
            return ESP32P4_JPEG_DEC_OK;
        }
    }
    *flags |= ESP32P4_JPEG_DEC_BAD_CODE;
    *sym = 0;
    return ESP32P4_JPEG_DEC_OK;
}

/* T.81 F.2.2.1's RECEIVE and EXTEND. */
static int receive_extend(ESP32P4JpegBitIn *in, int s, int32_t *v)
{
    uint32_t x;
    int r;

    if (!s) {
        *v = 0;
        return ESP32P4_JPEG_DEC_OK;
    }
    r = get_bits(in, s, &x);
    if (r != ESP32P4_JPEG_DEC_OK) {
        return r;
    }
    *v = x < (1u << (s - 1)) ? (int32_t)x - (int32_t)((1u << s) - 1)
                             : (int32_t)x;
    return ESP32P4_JPEG_DEC_OK;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
int esp32p4_jpeg_decode_mcu(ESP32P4JpegBitIn *in,
                            const ESP32P4JpegDecComp *comp, int ncomp,
                            int32_t pred[4], uint8_t *out, uint32_t *flags)
{
    int32_t coef[64];
    int sym, r;

    for (int c = 0; c < ncomp; c++) {
        const ESP32P4JpegDecComp *k = &comp[c];

        for (int u = 0; u < k->units; u++) {
            int32_t diff, v;

            memset(coef, 0, sizeof(coef));
            r = huff_decode(in, k->dc, &sym, flags);
            if (r != ESP32P4_JPEG_DEC_OK) {
                return r;
            }
            r = receive_extend(in, sym & 15, &diff);
            if (r != ESP32P4_JPEG_DEC_OK) {
                return r;
            }
            pred[c] += diff;
            coef[0] = pred[c] * (int32_t)(k->q[0] & k->qmask);

            for (int i = 1; i < 64; i++) {
                int run, size;

                r = huff_decode(in, k->ac, &sym, flags);
                if (r != ESP32P4_JPEG_DEC_OK) {
                    return r;
                }
                run = sym >> 4;
                size = sym & 15;
                if (!size) {
                    if (run != 15) {
                        break;
                    }
                    i += 15;
                    if (i > 63) {
                        *flags |= ESP32P4_JPEG_DEC_RUN_OVERFLOW;
                    }
                    continue;
                }
                i += run;
                r = receive_extend(in, size, &v);
                if (r != ESP32P4_JPEG_DEC_OK) {
                    return r;
                }
                if (i > 63) {
                    *flags |= ESP32P4_JPEG_DEC_RUN_OVERFLOW;
                    break;
                }
                coef[esp32p4_jpeg_natural[i]] =
                    v * (int32_t)(k->q[esp32p4_jpeg_natural[i]] & k->qmask);
            }
            esp32p4_jpeg_idct(coef, out);
            out += 64;
        }
    }
    return ESP32P4_JPEG_DEC_OK;
}
