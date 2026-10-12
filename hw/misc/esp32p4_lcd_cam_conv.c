/*
 * ESP32-P4 LCD_CAM's YUV-RGB converters
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM (v3.x) section 40.3.6, tables 40.3-5 and 40.3-6,
 * and the LCD_RGB_YUV and CAM_RGB_YUV registers; ESP-IDF's lcd_ll and
 * cam_ll, which name the formats (LCD_COLOR_FMT_YUV422_UYVY,
 * LCD_COLOR_FMT_YUV420_OUYY_EVYY).
 *
 * The conversions (TRANS_MODE, YUV_MODE, YUV2YUV_MODE and the LCD's
 * RGB2RGB_MODE, as the tables give them): RGB565 to and from RGB888 (the
 * LCD's), YUV to RGB565 (RGB888 at the LCD's 24-bit output), RGB565 to YUV
 * (YUV444 at the LCD's 24-bit output) and YUV to YUV, the YUV formats
 * being YUV422 (YUV_MODE 0), YUV420 (1) and YUV411 (2).
 *
 * The TRM gives the formats only as sequences of samples; the model's
 * reading of them, as bytes in the order they come from memory or the
 * camera's data lines:
 *
 *   - The converter takes 16-bit units, each its two bytes in order low
 *     then high (in 8-bit mode as in 16-bit mode, unless
 *     CONV_8BITS_DATA_INV swaps each two bytes first), and the TRM's
 *     sequences list those units' bytes high first: YUV422's "Y1 U1 Y2
 *     V2" is the units {Y1, U1} {Y2, V2}, bytes U1 Y1 V2 Y2, ESP-IDF's
 *     UYVY; YUV411's "Y1 U1 Y2 Y3 V3 Y4" the bytes U1 Y1 Y3 Y2 Y4 V3.
 *   - RGB565 is a 16-bit unit (red in bits 15:11), RGB888 three bytes
 *     blue, green, red (red in bits 23:16 of a 24-bit word), YUV444 three
 *     bytes V, U, Y.
 *   - YUV420 is ESP-IDF's OUYY_EVYY: each pixel pair is three bytes, the
 *     pair's U then its two Ys on the first line of a frame and every
 *     other one after, V then the Ys on the lines between. Decoding takes
 *     the other chroma from the line before (128 for the first).
 *   - Encoding averages the chroma of each pair (four pixels in YUV411),
 *     rounding to nearest.
 *
 * The arithmetic, which the TRM does not give: BT.601 or BT.709
 * (CONV_PROTOCOL_MODE) with the coefficients rounded to 1/65536ths, the
 * results rounded to nearest and clamped to 0..255; RGB888 to RGB565
 * drops each colour's low bits, RGB565 to RGB888 repeats its top bits
 * below. Limited range (DATA_IN_MODE, DATA_OUT_MODE clear) is taken as
 * BT.601's and BT.709's own, Y and RGB 16 to 235 and chroma 16 to 240:
 * the TRM gives RGB 16 to 240, Y 16 to 240 and U and V 16 to 235, which
 * cannot be the scale of chroma centred on 128 and reads as the two
 * swapped. A board check: convert full-range white and a 75% blue on the
 * camera and read the bytes back.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "migration/vmstate.h"
#include "esp32p4_lcd_cam_conv.h"

enum {
    KIND_OFF,
    KIND_RGB_RGB,
    KIND_YUV_RGB,
    KIND_RGB_YUV,
    KIND_YUV_YUV,
    KIND_INVALID,
};

enum {
    FMT_RGB565,
    FMT_RGB888,
    FMT_YUV422,
    FMT_YUV420,
    FMT_YUV411,
    FMT_YUV444,
};

/* Bytes, and pixels, of each format's group. */
static const uint8_t fmt_bytes[] = { 2, 3, 4, 3, 6, 3 };
static const uint8_t fmt_pixels[] = { 1, 1, 2, 2, 4, 1 };

typedef struct ConvMode {
    uint8_t kind;
    uint8_t in;
    uint8_t out;
} ConvMode;

#define CFG_ENABLE(c)       extract32((c)->cfg, 31, 1)
#define CFG_TRANS(c)        extract32((c)->cfg, 30, 1)
#define CFG_8BITS(c)        extract32((c)->cfg, 29, 1)
#define CFG_IN_FULL(c)      extract32((c)->cfg, 28, 1)
#define CFG_OUT_FULL(c)     extract32((c)->cfg, 27, 1)
#define CFG_BT709(c)        extract32((c)->cfg, 26, 1)
#define CFG_YUV_MODE(c)     extract32((c)->cfg, 24, 2)
#define CFG_YUV2YUV(c)      extract32((c)->cfg, 22, 2)

static bool cfg_inv8(const ESP32P4LcdCamConv *c)
{
    return extract32(c->cfg, c->lcd ? 20 : 21, 1);
}

static unsigned cfg_rgb2rgb(const ESP32P4LcdCamConv *c)
{
    return c->lcd ? extract32(c->cfg, 18, 2) : 3;
}

static int yuv_fmt(unsigned mode)
{
    static const int8_t fmt[4] = { FMT_YUV422, FMT_YUV420, FMT_YUV411, -1 };

    return fmt[mode];
}

static ConvMode conv_mode(const ESP32P4LcdCamConv *c)
{
    ConvMode m = { KIND_OFF, 0, 0 };
    int in, out;

    if (!CFG_ENABLE(c)) {
        return m;
    }
    if (!CFG_TRANS(c) && cfg_rgb2rgb(c) < 2) {
        m.kind = KIND_RGB_RGB;
        m.in = cfg_rgb2rgb(c) ? FMT_RGB565 : FMT_RGB888;
        m.out = cfg_rgb2rgb(c) ? FMT_RGB888 : FMT_RGB565;
        return m;
    }
    in = yuv_fmt(CFG_YUV_MODE(c));
    if (!CFG_TRANS(c)) {
        m.kind = in < 0 ? KIND_INVALID : KIND_YUV_RGB;
        m.in = in;
        m.out = c->wire24 ? FMT_RGB888 : FMT_RGB565;
    } else if (CFG_YUV2YUV(c) == 3) {
        m.kind = in < 0 && !c->wire24 ? KIND_INVALID : KIND_RGB_YUV;
        m.in = FMT_RGB565;
        m.out = c->wire24 ? FMT_YUV444 : in;
    } else {
        out = yuv_fmt(CFG_YUV2YUV(c));
        m.kind = in < 0 || out < 0 ? KIND_INVALID : KIND_YUV_YUV;
        m.in = in;
        m.out = c->wire24 ? FMT_YUV444 : out;
    }
    return m;
}

void esp32p4_lcd_cam_conv_setup(ESP32P4LcdCamConv *c, uint32_t cfg,
                                bool lcd, bool wire24, uint32_t line_pixels)
{
    c->cfg = cfg;
    c->lcd = lcd;
    c->wire24 = wire24;
    c->line_pixels = line_pixels;
}

bool esp32p4_lcd_cam_conv_on(const ESP32P4LcdCamConv *c)
{
    ConvMode m = conv_mode(c);

    return m.kind != KIND_OFF && m.kind != KIND_INVALID;
}

bool esp32p4_lcd_cam_conv_valid(const ESP32P4LcdCamConv *c)
{
    return conv_mode(c).kind != KIND_INVALID;
}

void esp32p4_lcd_cam_conv_reset(ESP32P4LcdCamConv *c)
{
    c->swap_held = false;
    c->in_n = 0;
    c->pend_n = 0;
    c->pixel = 0;
    c->line = 0;
    memset(c->chroma, 128, sizeof(c->chroma));
}

void esp32p4_lcd_cam_conv_new_line(ESP32P4LcdCamConv *c)
{
    c->line++;
    c->pixel = 0;
}

/*
 * The arithmetic, in 1/65536ths: RGB to YUV, Y's coefficients for R, G
 * and B, then U's and V's; YUV to RGB, V's for R, U's and V's for G and
 * U's for B.
 */
static const int32_t rgb2yuv[2][9] = {
    { 19595, 38470, 7471, -11059, -21709, 32768, 32768, -27439, -5329 },
    { 13933, 46871, 4732, -7509, -25259, 32768, 32768, -29763, -3005 },
};
static const int32_t yuv2rgb[2][4] = {
    { 91881, 22554, 46802, 116130 },
    { 103206, 12276, 30679, 121609 },
};

static uint8_t round_clamp(int64_t v16)
{
    int64_t v = (v16 + 32768) >> 16;

    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* A luma or RGB sample, and a chroma's distance from 128, at full scale. */
static int64_t luma_in(const ESP32P4LcdCamConv *c, uint8_t v)
{
    return CFG_IN_FULL(c) ? (int64_t)v << 16 :
           ((int64_t)v - 16) * (255 << 16) / 219;
}

static int64_t chroma_in(const ESP32P4LcdCamConv *c, uint8_t v)
{
    return CFG_IN_FULL(c) ? ((int64_t)v - 128) << 16 :
           ((int64_t)v - 128) * (255 << 16) / 224;
}

static uint8_t luma_out(const ESP32P4LcdCamConv *c, int64_t v16)
{
    return round_clamp(CFG_OUT_FULL(c) ? v16 :
                       (16 << 16) + v16 * 219 / 255);
}

static uint8_t chroma_out(const ESP32P4LcdCamConv *c, int64_t v16)
{
    return round_clamp((128 << 16) +
                       (CFG_OUT_FULL(c) ? v16 : v16 * 224 / 255));
}

/* One pixel, @p in the input format's kind, to the output's, in place. */
static void convert_pixel(const ESP32P4LcdCamConv *c, unsigned kind,
                          uint8_t p[3])
{
    const int32_t *k = rgb2yuv[CFG_BT709(c)];
    const int32_t *q = yuv2rgb[CFG_BT709(c)];
    int64_t a, b, d, y, u, v;

    switch (kind) {
    case KIND_RGB_RGB:
        p[0] = luma_out(c, luma_in(c, p[0]));
        p[1] = luma_out(c, luma_in(c, p[1]));
        p[2] = luma_out(c, luma_in(c, p[2]));
        break;
    case KIND_RGB_YUV:
        a = luma_in(c, p[0]);
        b = luma_in(c, p[1]);
        d = luma_in(c, p[2]);
        y = (k[0] * a + k[1] * b + k[2] * d) >> 16;
        u = (k[3] * a + k[4] * b + k[5] * d) >> 16;
        v = (k[6] * a + k[7] * b + k[8] * d) >> 16;
        p[0] = luma_out(c, y);
        p[1] = chroma_out(c, u);
        p[2] = chroma_out(c, v);
        break;
    case KIND_YUV_RGB:
        y = luma_in(c, p[0]);
        u = chroma_in(c, p[1]);
        v = chroma_in(c, p[2]);
        p[0] = luma_out(c, y + ((q[0] * v) >> 16));
        p[1] = luma_out(c, y - ((q[1] * u + q[2] * v) >> 16));
        p[2] = luma_out(c, y + ((q[3] * u) >> 16));
        break;
    case KIND_YUV_YUV:
        p[0] = luma_out(c, luma_in(c, p[0]));
        p[1] = chroma_out(c, chroma_in(c, p[1]));
        p[2] = chroma_out(c, chroma_in(c, p[2]));
        break;
    }
}

static void emit_group(ESP32P4LcdCamConv *c, unsigned fmt,
                       ESP32P4LcdCamEmit *emit, void *opaque)
{
    uint8_t (*p)[3] = c->pend;
    uint8_t out[6];
    unsigned n = fmt_bytes[fmt], u, v;
    uint16_t w;

    switch (fmt) {
    case FMT_RGB565:
        w = (p[0][0] >> 3) << 11 | (p[0][1] >> 2) << 5 | p[0][2] >> 3;
        out[0] = w;
        out[1] = w >> 8;
        break;
    case FMT_RGB888:
        out[0] = p[0][2];
        out[1] = p[0][1];
        out[2] = p[0][0];
        break;
    case FMT_YUV444:
        out[0] = p[0][2];
        out[1] = p[0][1];
        out[2] = p[0][0];
        break;
    case FMT_YUV422:
        out[0] = (p[0][1] + p[1][1] + 1) >> 1;
        out[1] = p[0][0];
        out[2] = (p[0][2] + p[1][2] + 1) >> 1;
        out[3] = p[1][0];
        break;
    case FMT_YUV420:
        out[0] = c->line & 1 ? (p[0][2] + p[1][2] + 1) >> 1 :
                               (p[0][1] + p[1][1] + 1) >> 1;
        out[1] = p[0][0];
        out[2] = p[1][0];
        break;
    default:
        u = (p[0][1] + p[1][1] + p[2][1] + p[3][1] + 2) >> 2;
        v = (p[0][2] + p[1][2] + p[2][2] + p[3][2] + 2) >> 2;
        out[0] = u;
        out[1] = p[0][0];
        out[2] = p[2][0];
        out[3] = p[1][0];
        out[4] = p[3][0];
        out[5] = v;
        break;
    }
    emit(opaque, out, n);
}

static void take_pixel(ESP32P4LcdCamConv *c, const ConvMode *m,
                       const uint8_t px[3], ESP32P4LcdCamEmit *emit,
                       void *opaque)
{
    memcpy(c->pend[c->pend_n], px, 3);
    convert_pixel(c, m->kind, c->pend[c->pend_n]);
    if (++c->pend_n == fmt_pixels[m->out]) {
        emit_group(c, m->out, emit, opaque);
        c->pend_n = 0;
    }
    if (c->line_pixels && ++c->pixel == c->line_pixels) {
        esp32p4_lcd_cam_conv_new_line(c);
    } else if (!c->line_pixels) {
        c->pixel++;
    }
}

/* A whole input group in c->in: its pixels. */
static void take_group(ESP32P4LcdCamConv *c, const ConvMode *m,
                       ESP32P4LcdCamEmit *emit, void *opaque)
{
    const uint8_t *b = c->in;
    uint8_t px[4][3];
    unsigned i, n = fmt_pixels[m->in], pair, other;
    uint16_t w;

    switch (m->in) {
    case FMT_RGB565:
        w = b[0] | b[1] << 8;
        px[0][0] = (w >> 11) << 3 | w >> 13;
        px[0][1] = (w >> 5 & 0x3f) << 2 | (w >> 9 & 3);
        px[0][2] = (w & 0x1f) << 3 | (w >> 2 & 7);
        break;
    case FMT_RGB888:
        px[0][0] = b[2];
        px[0][1] = b[1];
        px[0][2] = b[0];
        break;
    case FMT_YUV422:
        px[0][0] = b[1];
        px[1][0] = b[3];
        px[0][1] = px[1][1] = b[0];
        px[0][2] = px[1][2] = b[2];
        break;
    case FMT_YUV420:
        pair = MIN(c->pixel / 2, ARRAY_SIZE(c->chroma) - 1);
        other = c->chroma[pair];
        c->chroma[pair] = b[0];
        px[0][0] = b[1];
        px[1][0] = b[2];
        px[0][1] = px[1][1] = c->line & 1 ? other : b[0];
        px[0][2] = px[1][2] = c->line & 1 ? b[0] : other;
        break;
    default:
        px[0][0] = b[1];
        px[1][0] = b[3];
        px[2][0] = b[2];
        px[3][0] = b[4];
        for (i = 0; i < 4; i++) {
            px[i][1] = b[0];
            px[i][2] = b[5];
        }
        break;
    }
    for (i = 0; i < n; i++) {
        take_pixel(c, m, px[i], emit, opaque);
    }
}

static void take_byte(ESP32P4LcdCamConv *c, const ConvMode *m, uint8_t b,
                      ESP32P4LcdCamEmit *emit, void *opaque)
{
    c->in[c->in_n++] = b;
    if (c->in_n == fmt_bytes[m->in]) {
        take_group(c, m, emit, opaque);
        c->in_n = 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
void esp32p4_lcd_cam_conv_feed(ESP32P4LcdCamConv *c, const uint8_t *bytes,
                               unsigned n, ESP32P4LcdCamEmit *emit,
                               void *opaque)
{
    ConvMode m = conv_mode(c);
    bool inv8 = CFG_8BITS(c) && cfg_inv8(c);
    unsigned i;

    if (m.kind == KIND_OFF || m.kind == KIND_INVALID) {
        emit(opaque, bytes, n);
        return;
    }
    for (i = 0; i < n; i++) {
        if (!inv8) {
            take_byte(c, &m, bytes[i], emit, opaque);
        } else if (!c->swap_held) {
            c->swap = bytes[i];
            c->swap_held = true;
        } else {
            take_byte(c, &m, bytes[i], emit, opaque);
            take_byte(c, &m, c->swap, emit, opaque);
            c->swap_held = false;
        }
    }
}

void esp32p4_lcd_cam_conv_flush(ESP32P4LcdCamConv *c,
                                ESP32P4LcdCamEmit *emit, void *opaque)
{
    ConvMode m = conv_mode(c);
    static const uint8_t zero[6];

    if (m.kind == KIND_OFF || m.kind == KIND_INVALID) {
        return;
    }
    if (c->swap_held) {
        esp32p4_lcd_cam_conv_feed(c, zero, 1, emit, opaque);
    }
    if (c->in_n) {
        esp32p4_lcd_cam_conv_feed(c, zero, fmt_bytes[m.in] - c->in_n, emit,
                                  opaque);
    }
    while (c->pend_n) {
        static const uint8_t black[3];

        take_pixel(c, &m, black, emit, opaque);
    }
}

const VMStateDescription vmstate_esp32p4_lcd_cam_conv = {
    .name = "esp32p4-lcd-cam-conv",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, ESP32P4LcdCamConv),
        VMSTATE_BOOL(lcd, ESP32P4LcdCamConv),
        VMSTATE_BOOL(wire24, ESP32P4LcdCamConv),
        VMSTATE_UINT32(line_pixels, ESP32P4LcdCamConv),
        VMSTATE_UINT8(swap, ESP32P4LcdCamConv),
        VMSTATE_BOOL(swap_held, ESP32P4LcdCamConv),
        VMSTATE_UINT8_ARRAY(in, ESP32P4LcdCamConv, 6),
        VMSTATE_UINT32(in_n, ESP32P4LcdCamConv),
        VMSTATE_UINT32(pixel, ESP32P4LcdCamConv),
        VMSTATE_UINT32(line, ESP32P4LcdCamConv),
        VMSTATE_UINT8_2DARRAY(pend, ESP32P4LcdCamConv, 4, 3),
        VMSTATE_UINT32(pend_n, ESP32P4LcdCamConv),
        VMSTATE_UINT8_ARRAY(chroma, ESP32P4LcdCamConv,
                            ESP32P4_LCD_CAM_LINE / 2),
        VMSTATE_END_OF_LIST()
    },
};
