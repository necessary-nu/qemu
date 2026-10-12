/*
 * ESP32-P4 camera sensor: still pictures from PPM, PGM and PNG files
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Pure C over QEMU's own zlib: PNG's image data is a zlib stream, and
 * everything else in the formats is a few lines of parsing.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "hw/misc/esp32p4_image_source.h"
#include "esp32p4_image_decode.h"
#include <zlib.h>

/*
 * The largest picture taken: 2^26 pixels (8192 by 8192), 384 MiB as 16-bit
 * RGB, well past any sensor the chip can receive.
 */
#define PICTURE_MAX_PIXELS  (1u << 26)

static bool picture_alloc(ESP32P4Picture *pic, uint64_t width,
                          uint64_t height, const char *path, Error **errp)
{
    if (!width || !height || width * height > PICTURE_MAX_PIXELS) {
        error_setg(errp, "%s: a picture of %" PRIu64 " by %" PRIu64
                   " pixels is not supported (at most %u pixels)", path,
                   width, height, PICTURE_MAX_PIXELS);
        return false;
    }
    pic->width = width;
    pic->height = height;
    pic->pixels = g_new0(ESP32P4Rgb16, width * height);
    return true;
}

void esp32p4_picture_free(ESP32P4Picture *pic)
{
    g_free(pic->pixels);
    pic->pixels = NULL;
    pic->width = pic->height = 0;
}

/* @v of @max, rounded to 16 bits. */
static uint16_t scale16(uint32_t v, uint32_t max)
{
    return ((uint64_t)v * 65535 + max / 2) / max;
}

/*
 * Netpbm
 * ======
 *
 * "P5" or "P6", then width, height and maxval as decimal numbers separated
 * by whitespace and '#' comments running to the end of their line, then
 * a single whitespace character and the samples: one byte each if maxval
 * is below 256, else two, most significant first.
 */

static bool pnm_number(const uint8_t *buf, size_t len, size_t *pos,
                       uint32_t *out)
{
    uint64_t v = 0;
    size_t start;

    for (;;) {
        if (*pos >= len) {
            return false;
        }
        if (buf[*pos] == '#') {
            while (*pos < len && buf[*pos] != '\n' && buf[*pos] != '\r') {
                (*pos)++;
            }
        } else if (g_ascii_isspace(buf[*pos])) {
            (*pos)++;
        } else {
            break;
        }
    }
    start = *pos;
    while (*pos < len && g_ascii_isdigit(buf[*pos])) {
        v = v * 10 + (buf[*pos] - '0');
        if (v > UINT32_MAX) {
            return false;
        }
        (*pos)++;
    }
    *out = v;
    return *pos > start && *pos < len && g_ascii_isspace(buf[*pos]);
}

static bool pnm_decode(const uint8_t *buf, size_t len, ESP32P4Picture *pic,
                       const char *path, Error **errp)
{
    bool rgb = buf[1] == '6';
    uint32_t width, height, maxval, channels = rgb ? 3 : 1;
    size_t pos = 2, bytes, i, n;

    if (!pnm_number(buf, len, &pos, &width) ||
        !pnm_number(buf, len, &pos, &height) ||
        !pnm_number(buf, len, &pos, &maxval)) {
        error_setg(errp, "%s: a malformed PNM header", path);
        return false;
    }
    pos++;
    if (!maxval || maxval > 65535) {
        error_setg(errp, "%s: PNM maxval %u is not 1 to 65535", path, maxval);
        return false;
    }
    if (!picture_alloc(pic, width, height, path, errp)) {
        return false;
    }
    bytes = maxval > 255 ? 2 : 1;
    n = (size_t)width * height;
    if (len - pos < n * channels * bytes) {
        error_setg(errp, "%s: the PNM data is short of %ux%u pixels", path,
                   width, height);
        esp32p4_picture_free(pic);
        return false;
    }
    for (i = 0; i < n; i++) {
        uint32_t v[3];
        unsigned c;

        for (c = 0; c < channels; c++) {
            const uint8_t *p = buf + pos + (i * channels + c) * bytes;

            v[c] = bytes == 2 ? lduw_be_p(p) : *p;
            if (v[c] > maxval) {
                v[c] = maxval;
            }
        }
        if (!rgb) {
            v[1] = v[2] = v[0];
        }
        pic->pixels[i].r = scale16(v[0], maxval);
        pic->pixels[i].g = scale16(v[1], maxval);
        pic->pixels[i].b = scale16(v[2], maxval);
    }
    return true;
}

/*
 * PNG
 * ===
 *
 * Chunks of a 32-bit length, a type, the data and a CRC-32 of type and
 * data. IHDR comes first; the image data is the zlib stream that the
 * IDAT chunks' data make up together, each row a filter type byte and
 * the filtered row; with Adam7 interlacing, seven passes of rows, each
 * a reduced image of its own.
 */

#define PNG_GRAY        0
#define PNG_RGB         2
#define PNG_PALETTE     3
#define PNG_GRAY_ALPHA  4
#define PNG_RGBA        6

typedef struct PngInfo {
    uint32_t width;
    uint32_t height;
    unsigned depth;
    unsigned type;
    unsigned channels;
    bool interlaced;
    unsigned palette_len;
    uint8_t palette[256][3];
} PngInfo;

/* Adam7: each pass's first column and row and its steps across and down */
static const uint8_t adam7[7][4] = {
    { 0, 0, 8, 8 }, { 4, 0, 8, 8 }, { 0, 4, 4, 8 }, { 2, 0, 4, 4 },
    { 0, 2, 2, 4 }, { 1, 0, 2, 2 }, { 0, 1, 1, 2 },
};

static size_t png_row_bytes(const PngInfo *pi, uint32_t width)
{
    return ((uint64_t)width * pi->channels * pi->depth + 7) / 8;
}

static unsigned png_paeth(int a, int b, int c)
{
    int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);

    if (pa <= pb && pa <= pc) {
        return a;
    }
    return pb <= pc ? b : c;
}

/*
 * Undo row @row's filter in place, against the previous row of the same
 * pass (@prev, NULL for the pass's first). @bpp is the bytes a complete
 * pixel takes, at least 1.
 */
static bool png_unfilter(uint8_t *row, const uint8_t *prev, size_t len,
                         unsigned filter, unsigned bpp)
{
    size_t i;

    for (i = 0; i < len; i++) {
        unsigned a = i >= bpp ? row[i - bpp] : 0;
        unsigned b = prev ? prev[i] : 0;
        unsigned c = prev && i >= bpp ? prev[i - bpp] : 0;

        switch (filter) {
        case 0:
            break;
        case 1:
            row[i] += a;
            break;
        case 2:
            row[i] += b;
            break;
        case 3:
            row[i] += (a + b) / 2;
            break;
        case 4:
            row[i] += png_paeth(a, b, c);
            break;
        default:
            return false;
        }
    }
    return true;
}

/* Sample @i of an unfiltered row, of @depth bits, packed from the MSB. */
static uint32_t png_sample(const uint8_t *row, size_t i, unsigned depth)
{
    size_t bit;

    switch (depth) {
    case 16:
        return lduw_be_p(row + 2 * i);
    case 8:
        return row[i];
    default:
        bit = i * depth;
        return (row[bit / 8] >> (8 - depth - bit % 8)) & ((1u << depth) - 1);
    }
}

static void png_put_row(const PngInfo *pi, ESP32P4Picture *pic,
                        const uint8_t *row, uint32_t y, uint32_t x0,
                        uint32_t dx, uint32_t count)
{
    uint32_t max = (1u << pi->depth) - 1, i;

    for (i = 0; i < count; i++) {
        ESP32P4Rgb16 *px = &pic->pixels[(size_t)y * pic->width + x0 + i * dx];
        size_t s = (size_t)i * pi->channels;

        switch (pi->type) {
        case PNG_GRAY:
        case PNG_GRAY_ALPHA:
            px->r = px->g = px->b = scale16(png_sample(row, s, pi->depth),
                                            max);
            break;
        case PNG_RGB:
        case PNG_RGBA:
            px->r = scale16(png_sample(row, s, pi->depth), max);
            px->g = scale16(png_sample(row, s + 1, pi->depth), max);
            px->b = scale16(png_sample(row, s + 2, pi->depth), max);
            break;
        case PNG_PALETTE: {
            uint32_t idx = png_sample(row, s, pi->depth);
            const uint8_t *e = idx < pi->palette_len ? pi->palette[idx]
                                                     : pi->palette[0];

            px->r = e[0] * 257;
            px->g = e[1] * 257;
            px->b = e[2] * 257;
            break;
        }
        }
    }
}

static bool png_header(const uint8_t *d, uint32_t len, PngInfo *pi,
                       const char *path, Error **errp)
{
    static const uint8_t depths[7] = {
        [PNG_GRAY] = 0x1 | 0x2 | 0x4 | 0x8 | 0x10,
        [PNG_RGB] = 0x8 | 0x10,
        [PNG_PALETTE] = 0x1 | 0x2 | 0x4 | 0x8,
        [PNG_GRAY_ALPHA] = 0x8 | 0x10,
        [PNG_RGBA] = 0x8 | 0x10,
    };
    static const uint8_t channels[7] = { 1, 0, 3, 1, 2, 0, 4 };

    if (len != 13) {
        error_setg(errp, "%s: a malformed PNG IHDR", path);
        return false;
    }
    pi->width = ldl_be_p(d);
    pi->height = ldl_be_p(d + 4);
    pi->depth = d[8];
    pi->type = d[9];
    if (pi->type > PNG_RGBA || !channels[pi->type] ||
        pi->depth > 16 || !is_power_of_2(pi->depth) ||
        !(depths[pi->type] & pi->depth) || d[10] || d[11] || d[12] > 1) {
        error_setg(errp, "%s: PNG colour type %u at %u bits (compression %u,"
                   " filter %u, interlace %u) is not valid", path, pi->type,
                   pi->depth, d[10], d[11], d[12]);
        return false;
    }
    pi->channels = channels[pi->type];
    pi->interlaced = d[12];
    return true;
}

static bool png_inflate(const GByteArray *idat, uint8_t *out, size_t len,
                        const char *path, Error **errp)
{
    z_stream zs = { 0 };
    int ret;

    if (inflateInit(&zs) != Z_OK) {
        error_setg(errp, "%s: zlib: %s", path, zs.msg ? zs.msg : "no memory");
        return false;
    }
    zs.next_in = idat->data;
    zs.avail_in = idat->len;
    zs.next_out = out;
    zs.avail_out = len;
    ret = inflate(&zs, Z_FINISH);
    inflateEnd(&zs);
    if (ret != Z_STREAM_END || zs.avail_out) {
        error_setg(errp, "%s: the PNG image data is %s", path,
                   ret == Z_STREAM_END || ret == Z_BUF_ERROR ?
                   "not the size its header gives" : "corrupt");
        return false;
    }
    return true;
}

static bool png_decode(const uint8_t *buf, size_t len, ESP32P4Picture *pic,
                       const char *path, Error **errp)
{
    g_autoptr(GByteArray) idat = g_byte_array_new();
    g_autofree uint8_t *raw = NULL;
    PngInfo pi = { 0 };
    bool have_header = false, end = false;
    size_t pos = 8, total = 0, off;
    unsigned pass, passes, bpp;

    while (!end) {
        uint32_t clen, type;
        const uint8_t *d;

        if (len - pos < 12) {
            error_setg(errp, "%s: the PNG is cut short", path);
            return false;
        }
        clen = ldl_be_p(buf + pos);
        if (clen > len - pos - 12) {
            error_setg(errp, "%s: the PNG is cut short", path);
            return false;
        }
        type = ldl_be_p(buf + pos + 4);
        d = buf + pos + 8;
        if (crc32(crc32(0, NULL, 0), buf + pos + 4, clen + 4) !=
            (uint32_t)ldl_be_p(d + clen)) {
            error_setg(errp, "%s: a PNG chunk's CRC is wrong", path);
            return false;
        }
        pos += clen + 12;
        if (!have_header && type != 0x49484452) {
            error_setg(errp, "%s: the PNG does not start with IHDR", path);
            return false;
        }
        switch (type) {
        case 0x49484452: /* IHDR */
            if (have_header) {
                error_setg(errp, "%s: two PNG IHDRs", path);
                return false;
            }
            if (!png_header(d, clen, &pi, path, errp)) {
                return false;
            }
            have_header = true;
            break;
        case 0x504c5445: /* PLTE */
            if (clen % 3 || clen > 768) {
                error_setg(errp, "%s: a malformed PNG PLTE", path);
                return false;
            }
            pi.palette_len = clen / 3;
            memcpy(pi.palette, d, clen);
            break;
        case 0x49444154: /* IDAT */
            g_byte_array_append(idat, d, clen);
            break;
        case 0x49454e44: /* IEND */
            end = true;
            break;
        default:
            /* An ancillary chunk (lower-case first letter) can be skipped. */
            if (!(type & 0x20000000)) {
                error_setg(errp, "%s: the PNG has a critical chunk this "
                           "decoder does not know", path);
                return false;
            }
            break;
        }
    }
    if (pi.type == PNG_PALETTE && !pi.palette_len) {
        error_setg(errp, "%s: a palette PNG without PLTE", path);
        return false;
    }
    if (!picture_alloc(pic, pi.width, pi.height, path, errp)) {
        return false;
    }

    passes = pi.interlaced ? 7 : 1;
    for (pass = 0; pass < passes; pass++) {
        uint32_t x0 = pi.interlaced ? adam7[pass][0] : 0;
        uint32_t y0 = pi.interlaced ? adam7[pass][1] : 0;
        uint32_t dx = pi.interlaced ? adam7[pass][2] : 1;
        uint32_t dy = pi.interlaced ? adam7[pass][3] : 1;
        uint32_t pw = pi.width > x0 ? (pi.width - x0 + dx - 1) / dx : 0;
        uint32_t ph = pi.height > y0 ? (pi.height - y0 + dy - 1) / dy : 0;

        if (pw && ph) {
            total += (size_t)ph * (1 + png_row_bytes(&pi, pw));
        }
    }
    raw = g_malloc(total);
    if (!png_inflate(idat, raw, total, path, errp)) {
        esp32p4_picture_free(pic);
        return false;
    }

    bpp = MAX(1, pi.channels * pi.depth / 8);
    off = 0;
    for (pass = 0; pass < passes; pass++) {
        uint32_t x0 = pi.interlaced ? adam7[pass][0] : 0;
        uint32_t y0 = pi.interlaced ? adam7[pass][1] : 0;
        uint32_t dx = pi.interlaced ? adam7[pass][2] : 1;
        uint32_t dy = pi.interlaced ? adam7[pass][3] : 1;
        uint32_t pw = pi.width > x0 ? (pi.width - x0 + dx - 1) / dx : 0;
        uint32_t ph = pi.height > y0 ? (pi.height - y0 + dy - 1) / dy : 0;
        size_t rb = png_row_bytes(&pi, pw);
        uint8_t *prev = NULL;
        uint32_t r;

        if (!pw || !ph) {
            continue;
        }
        for (r = 0; r < ph; r++) {
            uint8_t *row = raw + off + 1;

            if (!png_unfilter(row, prev, rb, raw[off], bpp)) {
                error_setg(errp, "%s: PNG filter type %u is not valid", path,
                           raw[off]);
                esp32p4_picture_free(pic);
                return false;
            }
            png_put_row(&pi, pic, row, y0 + r * dy, x0, dx, pw);
            prev = row;
            off += 1 + rb;
        }
    }
    return true;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
bool esp32p4_picture_load(const char *path, ESP32P4Picture *pic,
                          Error **errp)
{
    static const uint8_t png_sig[8] = {
        0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n',
    };
    ERRP_GUARD();
    g_autofree gchar *buf = NULL;
    g_autoptr(GError) gerr = NULL;
    gsize len;
    bool ok;

    if (!g_file_get_contents(path, &buf, &len, &gerr)) {
        error_setg(errp, "%s", gerr->message);
        ok = false;
    } else if (len >= 8 && !memcmp(buf, png_sig, 8)) {
        ok = png_decode((const uint8_t *)buf, len, pic, path, errp);
    } else if (len >= 3 && buf[0] == 'P' && (buf[1] == '5' || buf[1] == '6') &&
               g_ascii_isspace(buf[2])) {
        ok = pnm_decode((const uint8_t *)buf, len, pic, path, errp);
    } else {
        error_setg(errp, "%s: not a binary PPM or PGM (P6, P5) or a PNG",
                   path);
        ok = false;
    }
    if (!ok) {
        error_prepend(errp, "esp32p4-image-source: ");
    }
    return ok;
}

/*
 * The source position of destination coordinate @i of @dst over @src
 * samples, pixel centres aligned, in 1/65536ths, clamped to the samples.
 */
static uint64_t scale_pos(uint32_t i, uint32_t dst, uint32_t src)
{
    int64_t p = ((2 * (int64_t)i + 1) * src * 65536) / (2 * (int64_t)dst) -
                32768;

    return MIN(MAX(p, 0), ((int64_t)src - 1) * 65536);
}

static uint16_t lerp16(uint32_t a, uint32_t b, uint32_t w)
{
    return (a * (65536 - w) + b * w + 32768) >> 16;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
void esp32p4_picture_scale(const ESP32P4Picture *in, uint32_t width,
                           uint32_t height, ESP32P4Picture *out)
{
    uint32_t x, y;

    out->width = width;
    out->height = height;
    out->pixels = g_new(ESP32P4Rgb16, (size_t)width * height);
    for (y = 0; y < height; y++) {
        uint64_t fy = scale_pos(y, height, in->height);
        uint32_t y0 = fy >> 16, y1 = MIN(y0 + 1, in->height - 1);
        uint32_t wy = fy & 0xffff;

        for (x = 0; x < width; x++) {
            uint64_t fx = scale_pos(x, width, in->width);
            uint32_t x0 = fx >> 16, x1 = MIN(x0 + 1, in->width - 1);
            uint32_t wx = fx & 0xffff;
            const ESP32P4Rgb16 *a = &in->pixels[(size_t)y0 * in->width + x0];
            const ESP32P4Rgb16 *b = &in->pixels[(size_t)y0 * in->width + x1];
            const ESP32P4Rgb16 *c = &in->pixels[(size_t)y1 * in->width + x0];
            const ESP32P4Rgb16 *d = &in->pixels[(size_t)y1 * in->width + x1];
            ESP32P4Rgb16 *o = &out->pixels[(size_t)y * width + x];

            o->r = lerp16(lerp16(a->r, b->r, wx), lerp16(c->r, d->r, wx), wy);
            o->g = lerp16(lerp16(a->g, b->g, wx), lerp16(c->g, d->g, wx), wy);
            o->b = lerp16(lerp16(a->b, b->b, wx), lerp16(c->b, d->b, wx), wy);
        }
    }
}
