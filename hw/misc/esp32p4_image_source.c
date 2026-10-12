/*
 * ESP32-P4 camera sensor: a host image source for the camera inputs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/misc/esp32p4_image_source.h for what the sensor does and
 * how receivers take its stream.
 *
 * The stream is a sequence of events a frame at a time: the frame's
 * VSYNC, HSYNC and DE edges and its lines of pixels on DVP, its packets
 * on CSI. As each frame starts, its events are listed with their times,
 * computed from the stream's start and the frame's place in it, and one
 * QEMU timer walks the list, running each event at its time. A line's
 * pixels are made only when its event runs, a line at a time.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qom/object_interfaces.h"
#include "migration/vmstate.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_image_source.h"
#include "esp32p4_image_decode.h"

enum {
    PATTERN_BARS,
    PATTERN_CHECKER,
};

enum {
    YUV_BT601,
    YUV_BT709,
};

enum {
    YUV_FULL,
    YUV_LIMITED,
};

/* The DVP outputs driven onto pads, in ESP32P4DvpSignal's order. */
enum {
    SYNC_VSYNC,
    SYNC_HSYNC,
    SYNC_DE,
    SYNCS,
};

/* The pads read: the sensor's power-down and reset inputs */
enum {
    PIN_PWDN,
    PIN_RESET,
    PINS,
};

typedef enum EventKind {
    EV_VSYNC_ON,
    EV_VSYNC_OFF,
    EV_HSYNC_ON,
    EV_HSYNC_OFF,
    EV_DE_ON,
    /* DE goes inactive: the line's samples go to the receivers. */
    EV_DVP_LINE,
    /* A CSI packet's last byte arrives. */
    EV_CSI_PACKET,
    /* The next frame starts. */
    EV_FRAME_END,
} EventKind;

typedef struct ImageEvent {
    int64_t ns;
    /* The line of pixels (EV_DVP_LINE, EV_CSI_PACKET); the frame's line */
    uint32_t line;
    uint8_t kind;
    /* CSI: the packet's data type and, for a short packet, data field */
    uint8_t dt;
    uint16_t data;
    /*
     * EV_DVP_LINE: the line's HSYNC start and end, DE start and end;
     * EV_CSI_PACKET: the burst's SoT, data start, end and EoT.
     */
    int64_t t[4];
} ImageEvent;

typedef struct ImageSink {
    const ESP32P4ImageSinkOps *ops;
    void *opaque;
} ImageSink;

struct ESP32P4ImageSource {
    Object parent_obj;

    /*
     * Properties. The numbers are -1 until set; once the object is
     * complete they hold what is in effect, defaults filled in.
     */
    char *file;
    char *image;
    char *data_pads;
    int64_t pattern;
    int64_t cell;
    int64_t format;
    int64_t bayer;
    int64_t yuv_matrix;
    int64_t yuv_range;
    int64_t width;
    int64_t height;
    int64_t fps;
    int64_t pixel_clock;
    int64_t hblank;
    int64_t vblank;
    int64_t hts;
    int64_t vts;
    int64_t hsync;
    int64_t hbp;
    int64_t vsync;
    int64_t vbp;
    int64_t dvp_width;
    int64_t pad[ESP32P4_DVP_SIGNALS];
    int64_t pin_pad[PINS];
    bool vsync_low;
    bool hsync_low;
    bool de_low;
    bool pwdn_low;
    bool reset_high;
    int64_t lanes;
    int64_t lane_rate;
    int64_t vc;
    bool line_sync;
    bool clock_continuous;
    /* "streaming": the sensor's software stream on */
    bool enabled;

    bool complete;
    ESP32P4ImageTiming timing;

    /* What the sensor shows: a file's frames, a still frame, or a pattern */
    GMappedFile *mapped;
    const uint8_t *frames;
    uint32_t nframes;
    size_t frame_bytes;
    uint8_t *still;
    ESP32P4Rgb16 *rgb;
    uint8_t *line_buf;
    uint16_t *samples;
    /* The bars' rows below the counter, even and odd, for one frame */
    uint8_t *bar_row[2];
    uint32_t bar_frame[2];
    bool bar_valid[2];

    GArray *sinks;
    GArray *events;
    QEMUTimer timer;
    bool in_run;

    /* Board wiring */
    bool pads_connected;
    Notifier line_source;
    qemu_irq sync_in[SYNCS];
    qemu_irq sync_release[SYNCS];
    qemu_irq pin_irq[PINS];

    /*
     * The stream: running since epoch_ns, its first frame run_frame0;
     * frame is the one in progress, next_frame the number the next takes,
     * next_ev the frame's next event. stopping: the software stream off
     * ends the stream as the frame ends.
     */
    bool running;
    bool stopping;
    int64_t epoch_ns;
    uint32_t run_frame0;
    uint32_t frame;
    uint32_t next_frame;
    uint32_t next_ev;
    /* The levels on the PWDN and RESETB pads, and the DVP outputs' */
    bool pin_level[PINS];
    bool sync_active[SYNCS];
};

/* D-PHY 1.1 burst timing, as ns plus UI */
#define DPHY_SOT_NS         200     /* T_LPX + T_HS-PREPARE + T_HS-ZERO */
#define DPHY_SOT_UI         19      /* their UI, and the sync sequence */
#define DPHY_TRAIL_NS       60
#define DPHY_TRAIL_UI       4
#define DPHY_EXIT_NS        100
#define CSI2_SHORT_BYTES    4
#define CSI2_LONG_OVERHEAD  6

/* Auto lane rate: the bursts within 80% of a line, from 80 Mbit/s up. */
#define LANE_RATE_STEP      10000000ULL
#define LANE_RATE_MIN       80000000ULL
#define LANE_RATE_MAX       1500000000ULL

static const struct {
    const char *name;
    ESP32P4ImageFormat format;
} format_names[] = {
    { "raw8", ESP32P4_IMAGE_RAW8 },
    { "raw10", ESP32P4_IMAGE_RAW10 },
    { "raw12", ESP32P4_IMAGE_RAW12 },
    { "rgb565", ESP32P4_IMAGE_RGB565 },
    { "rgb565le", ESP32P4_IMAGE_RGB565 },
    { "rgb565be", ESP32P4_IMAGE_RGB565BE },
    { "rgb888", ESP32P4_IMAGE_RGB888 },
    { "bgr888", ESP32P4_IMAGE_BGR888 },
    { "yuv422", ESP32P4_IMAGE_UYVY },
    { "uyvy", ESP32P4_IMAGE_UYVY },
    { "yuyv", ESP32P4_IMAGE_YUYV },
    { "vyuy", ESP32P4_IMAGE_VYUY },
    { "yvyu", ESP32P4_IMAGE_YVYU },
};

static bool format_is_raw(int64_t f)
{
    return f == ESP32P4_IMAGE_RAW8 || f == ESP32P4_IMAGE_RAW10 ||
           f == ESP32P4_IMAGE_RAW12;
}

static bool format_is_yuv(int64_t f)
{
    return f >= ESP32P4_IMAGE_UYVY && f <= ESP32P4_IMAGE_YVYU;
}

static unsigned raw_bits(int64_t f)
{
    return f == ESP32P4_IMAGE_RAW8 ? 8 : f == ESP32P4_IMAGE_RAW10 ? 10 : 12;
}

static uint8_t format_data_type(int64_t f)
{
    switch (f) {
    case ESP32P4_IMAGE_RAW8:
        return ESP32P4_CSI2_DT_RAW8;
    case ESP32P4_IMAGE_RAW10:
        return ESP32P4_CSI2_DT_RAW10;
    case ESP32P4_IMAGE_RAW12:
        return ESP32P4_CSI2_DT_RAW12;
    case ESP32P4_IMAGE_RGB565:
    case ESP32P4_IMAGE_RGB565BE:
        return ESP32P4_CSI2_DT_RGB565;
    case ESP32P4_IMAGE_RGB888:
    case ESP32P4_IMAGE_BGR888:
        return ESP32P4_CSI2_DT_RGB888;
    default:
        return ESP32P4_CSI2_DT_YUV422;
    }
}

static uint32_t format_line_bytes(int64_t f, uint32_t width)
{
    switch (f) {
    case ESP32P4_IMAGE_RAW8:
        return width;
    case ESP32P4_IMAGE_RAW10:
        return width / 4 * 5;
    case ESP32P4_IMAGE_RAW12:
        return width / 2 * 3;
    case ESP32P4_IMAGE_RGB888:
    case ESP32P4_IMAGE_BGR888:
        return width * 3;
    default:
        return width * 2;
    }
}

/*
 * Exact times
 * ===========
 */

/* @from + @ns + @ui unit intervals at @bps, rounded up to a whole ns. */
static int64_t dphy_ns(uint64_t bps, int64_t from, uint64_t ns, uint64_t ui)
{
    uint64_t lo, hi, rem;

    mulu64(&lo, &hi, ui, NANOSECONDS_PER_SECOND);
    lo += ns * bps;
    hi += lo < ns * bps;
    rem = divu128(&lo, &hi, bps);
    return from + lo + (rem != 0);
}

/* The first whole ns by which @clocks cycles at @hz have gone by. */
static int64_t cycles_ns(uint64_t clocks, uint64_t hz)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, clocks, NANOSECONDS_PER_SECOND);
    if (divu128(&lo, &hi, hz)) {
        lo++;
    }
    return lo;
}

static int64_t stream_clock_ns(ESP32P4ImageSource *s, uint32_t run_frame,
                               uint32_t line, uint64_t clock)
{
    const ESP32P4ImageTiming *t = &s->timing;
    uint64_t lines = (uint64_t)run_frame * t->frame_lines + line;

    return s->epoch_ns + cycles_ns(lines * t->line_clocks + clock,
                                   t->clock_hz);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
int64_t esp32p4_image_source_clock_ns(ESP32P4ImageSource *s, uint32_t frame,
                                      uint32_t line, uint64_t clock)
{
    if (!s->running) {
        return INT64_MAX;
    }
    return stream_clock_ns(s, frame - s->run_frame0, line, clock);
}

/*
 * A CSI-2 burst of @bytes starting at @sot: when its data starts, its
 * last byte has arrived, and its lanes are back in LP-11.
 */
static void csi_burst(const ESP32P4ImageTiming *t, int64_t sot,
                      uint32_t bytes, int64_t out[4])
{
    uint64_t ui = DPHY_SOT_UI + 8 * DIV_ROUND_UP(bytes, t->lanes);
    uint64_t trail_ns = DPHY_TRAIL_NS, trail_ui = DPHY_TRAIL_UI;

    /* T_HS-TRAIL is the longer of 8 UI and 60 ns + 4 UI. */
    if (DPHY_TRAIL_NS * t->lane_bps < 4 * NANOSECONDS_PER_SECOND) {
        trail_ns = 0;
        trail_ui = 8;
    }
    out[0] = sot;
    out[1] = dphy_ns(t->lane_bps, sot, DPHY_SOT_NS, DPHY_SOT_UI);
    out[2] = dphy_ns(t->lane_bps, sot, DPHY_SOT_NS, ui);
    out[3] = dphy_ns(t->lane_bps, sot, DPHY_SOT_NS + trail_ns, ui + trail_ui);
}

/*
 * Frame events
 * ============
 */

static void add_event(ESP32P4ImageSource *s, int64_t ns, EventKind kind,
                      uint32_t line)
{
    ImageEvent ev = { .ns = ns, .kind = kind, .line = line };

    g_array_append_val(s->events, ev);
}

static void add_packet(ESP32P4ImageSource *s, int64_t *sot, uint8_t dt,
                       uint16_t data, uint32_t line, uint32_t bytes)
{
    ImageEvent ev = { .kind = EV_CSI_PACKET, .line = line, .dt = dt,
                      .data = data };

    csi_burst(&s->timing, *sot, bytes, ev.t);
    ev.ns = ev.t[2];
    g_array_append_val(s->events, ev);
    *sot = ev.t[3] + DPHY_EXIT_NS;
}

static bool sync_pads_assigned(ESP32P4ImageSource *s)
{
    return s->pad[ESP32P4_DVP_VSYNC] >= 0 || s->pad[ESP32P4_DVP_HSYNC] >= 0 ||
           s->pad[ESP32P4_DVP_DE] >= 0;
}

/* The events of frame s->frame, from its start. */
static void build_frame_events(ESP32P4ImageSource *s)
{
    const ESP32P4ImageTiming *t = &s->timing;
    uint32_t f = s->frame - s->run_frame0, l;
    bool edges = sync_pads_assigned(s);

    g_array_set_size(s->events, 0);
    if (t->port == ESP32P4_IMAGE_PORT_DVP) {
        for (l = 0; l < t->frame_lines; l++) {
            int64_t start = stream_clock_ns(s, f, l, 0);

            if (l == 0) {
                add_event(s, start, EV_VSYNC_ON, l);
            }
            if (l == t->vsync_lines) {
                add_event(s, start, EV_VSYNC_OFF, l);
            }
            if (edges) {
                add_event(s, start, EV_HSYNC_ON, l);
                add_event(s, stream_clock_ns(s, f, l, t->hsync_clocks),
                          EV_HSYNC_OFF, l);
            }
            if (l >= t->active_line && l < t->active_line + t->height) {
                ImageEvent ev = { .kind = EV_DVP_LINE,
                                  .line = l - t->active_line };

                ev.t[0] = start;
                ev.t[1] = stream_clock_ns(s, f, l, t->hsync_clocks);
                ev.t[2] = stream_clock_ns(s, f, l, t->active_start);
                ev.t[3] = stream_clock_ns(s, f, l,
                                          t->active_start + t->active_clocks);
                ev.ns = ev.t[3];
                if (edges) {
                    add_event(s, ev.t[2], EV_DE_ON, l);
                }
                g_array_append_val(s->events, ev);
            }
        }
    } else {
        int64_t sot = stream_clock_ns(s, f, 0, 0);
        uint16_t number = s->frame % 65535 + 1;

        add_packet(s, &sot, ESP32P4_CSI2_DT_FS, number, 0, CSI2_SHORT_BYTES);
        for (l = 0; l < t->height; l++) {
            sot = MAX(sot, stream_clock_ns(s, f, t->active_line + l, 0));
            if (t->line_sync) {
                add_packet(s, &sot, ESP32P4_CSI2_DT_LS, l + 1, l,
                           CSI2_SHORT_BYTES);
            }
            add_packet(s, &sot, t->data_type, t->line_bytes, l,
                       t->line_bytes + CSI2_LONG_OVERHEAD);
            if (t->line_sync) {
                add_packet(s, &sot, ESP32P4_CSI2_DT_LE, l + 1, l,
                           CSI2_SHORT_BYTES);
            }
        }
        add_packet(s, &sot, ESP32P4_CSI2_DT_FE, number, 0, CSI2_SHORT_BYTES);
    }
    add_event(s, stream_clock_ns(s, f, t->frame_lines, 0), EV_FRAME_END, 0);
}

/*
 * Pixels
 * ======
 */

static const uint8_t bar_colours[8][3] = {
    { 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 0, 255, 0 },
    { 255, 0, 255 }, { 255, 0, 0 }, { 0, 0, 255 }, { 0, 0, 0 },
};

/* @v, 16 bits, rounded to @bits. */
static uint32_t scale_bits(uint32_t v, unsigned bits)
{
    return (v * ((1u << bits) - 1) + 32767) / 65535;
}

/*
 * RGB to YCbCr from 8-bit samples, the coefficients in 1/65536ths:
 * Y's for R, G and B, then Cb's and Cr's (Cb's for R and G and Cr's for G
 * and B subtracted), and Y's offset.
 */
static const int32_t yuv_coeffs[2][2][10] = {
    [YUV_BT601] = {
        [YUV_FULL] = { 19595, 38470, 7471, 11059, 21709, 32768,
                       32768, 27439, 5329, 0 },
        [YUV_LIMITED] = { 16829, 33039, 6416, 9714, 19070, 28784,
                          28784, 24103, 4681, 16 },
    },
    [YUV_BT709] = {
        [YUV_FULL] = { 13933, 46871, 4732, 7509, 25259, 32768,
                       32768, 29763, 3005, 0 },
        [YUV_LIMITED] = { 11966, 40254, 4064, 6596, 22188, 28784,
                          28784, 26145, 2639, 16 },
    },
};

static uint8_t clamp8(int32_t v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static void rgb_to_yuv(ESP32P4ImageSource *s, const ESP32P4Rgb16 *p,
                       int32_t *y, int32_t *u, int32_t *v)
{
    const int32_t *k = yuv_coeffs[s->yuv_matrix][s->yuv_range];
    int32_t r = scale_bits(p->r, 8), g = scale_bits(p->g, 8);
    int32_t b = scale_bits(p->b, 8);

    *y = clamp8((k[0] * r + k[1] * g + k[2] * b + (k[9] << 16) + 32768)
                >> 16);
    *u = clamp8((-k[3] * r - k[4] * g + k[5] * b + (128 << 16) + 32768)
                >> 16);
    *v = clamp8((k[6] * r - k[7] * g - k[8] * b + (128 << 16) + 32768)
                >> 16);
}

/* The colour filter at (@x, @y): 0 red, 1 green, 2 blue. */
static unsigned bayer_channel(int64_t bayer, uint32_t x, uint32_t y)
{
    static const uint8_t cfa[4][4] = {
        [ESP32P4_IMAGE_BAYER_RGGB] = { 0, 1, 1, 2 },
        [ESP32P4_IMAGE_BAYER_GRBG] = { 1, 0, 2, 1 },
        [ESP32P4_IMAGE_BAYER_GBRG] = { 1, 2, 0, 1 },
        [ESP32P4_IMAGE_BAYER_BGGR] = { 2, 1, 1, 0 },
    };

    return cfa[bayer][(y & 1) * 2 + (x & 1)];
}

/* Row @y of RGB to a line in the format's layout. */
static void convert_row(ESP32P4ImageSource *s, const ESP32P4Rgb16 *rgb,
                        uint32_t y, uint8_t *out)
{
    uint32_t w = s->width, x;

    switch (s->format) {
    case ESP32P4_IMAGE_RAW8:
    case ESP32P4_IMAGE_RAW10:
    case ESP32P4_IMAGE_RAW12: {
        unsigned bits = raw_bits(s->format);

        for (x = 0; x < w; x++) {
            const ESP32P4Rgb16 *p = &rgb[x];
            unsigned c = bayer_channel(s->bayer, x, y);
            uint32_t v = scale_bits(c == 0 ? p->r : c == 1 ? p->g : p->b,
                                    bits);

            if (bits == 8) {
                out[x] = v;
            } else if (bits == 10) {
                uint8_t *g = out + x / 4 * 5;

                g[x % 4] = v >> 2;
                g[4] = deposit32(x % 4 ? g[4] : 0, 2 * (x % 4), 2, v & 3);
            } else {
                uint8_t *g = out + x / 2 * 3;

                g[x % 2] = v >> 4;
                g[2] = deposit32(x % 2 ? g[2] : 0, 4 * (x % 2), 4, v & 0xf);
            }
        }
        break;
    }
    case ESP32P4_IMAGE_RGB565:
    case ESP32P4_IMAGE_RGB565BE:
        for (x = 0; x < w; x++) {
            uint16_t v = scale_bits(rgb[x].r, 5) << 11 |
                         scale_bits(rgb[x].g, 6) << 5 |
                         scale_bits(rgb[x].b, 5);

            if (s->format == ESP32P4_IMAGE_RGB565) {
                stw_le_p(out + 2 * x, v);
            } else {
                stw_be_p(out + 2 * x, v);
            }
        }
        break;
    case ESP32P4_IMAGE_RGB888:
    case ESP32P4_IMAGE_BGR888: {
        bool bgr = s->format == ESP32P4_IMAGE_BGR888;

        for (x = 0; x < w; x++) {
            uint8_t r = scale_bits(rgb[x].r, 8), b = scale_bits(rgb[x].b, 8);

            out[3 * x] = bgr ? b : r;
            out[3 * x + 1] = scale_bits(rgb[x].g, 8);
            out[3 * x + 2] = bgr ? r : b;
        }
        break;
    }
    default:
        for (x = 0; x + 1 < w; x += 2) {
            int32_t y0, y1, u0, u1, v0, v1, u, v;
            uint8_t *o = out + 2 * x;

            rgb_to_yuv(s, &rgb[x], &y0, &u0, &v0);
            rgb_to_yuv(s, &rgb[x + 1], &y1, &u1, &v1);
            u = (u0 + u1 + 1) >> 1;
            v = (v0 + v1 + 1) >> 1;
            switch (s->format) {
            case ESP32P4_IMAGE_UYVY:
                o[0] = u, o[1] = y0, o[2] = v, o[3] = y1;
                break;
            case ESP32P4_IMAGE_YUYV:
                o[0] = y0, o[1] = u, o[2] = y1, o[3] = v;
                break;
            case ESP32P4_IMAGE_VYUY:
                o[0] = v, o[1] = y0, o[2] = u, o[3] = y1;
                break;
            default:
                o[0] = y0, o[1] = v, o[2] = y1, o[3] = u;
                break;
            }
        }
        break;
    }
}

static void pattern_row(ESP32P4ImageSource *s, uint32_t frame, uint32_t y,
                        ESP32P4Rgb16 *rgb)
{
    uint32_t w = s->width, c = s->cell, x;

    if (s->pattern == PATTERN_BARS) {
        uint64_t step = MAX(w / 64, 1);
        uint32_t shift = (uint64_t)frame * step % w;

        for (x = 0; x < w; x++) {
            const uint8_t *col = bar_colours[(uint64_t)((x + shift) % w) * 8 /
                                             w];

            rgb[x].r = col[0] * 257;
            rgb[x].g = col[1] * 257;
            rgb[x].b = col[2] * 257;
        }
    } else {
        uint32_t period = 2 * c, back = frame % period;
        bool row_white = (y + period - back) % period < c;

        for (x = 0; x < w; x++) {
            bool white = ((x + period - back) % period < c) == row_white;

            rgb[x].r = rgb[x].g = rgb[x].b = white ? 65535 : 0;
        }
    }
    if (y < c) {
        for (x = 0; x < 16 && (x + 1) * c <= w; x++) {
            uint16_t v = frame & (0x8000 >> x) ? 65535 : 0;
            uint32_t i;

            for (i = x * c; i < (x + 1) * c; i++) {
                rgb[i].r = rgb[i].g = rgb[i].b = v;
            }
        }
    }
}

/* Line @y of frame @frame in the format's layout. */
static const uint8_t *frame_line(ESP32P4ImageSource *s, uint32_t frame,
                                 uint32_t y)
{
    const ESP32P4ImageTiming *t = &s->timing;

    if (s->frames) {
        return s->frames + (frame % s->nframes) * s->frame_bytes +
               (size_t)y * t->line_bytes;
    }
    if (s->still) {
        return s->still + (size_t)y * t->line_bytes;
    }
    if (s->pattern == PATTERN_BARS && y >= s->cell) {
        /* Below the counter every row of a frame is the same colours. */
        unsigned odd = y & 1;

        if (!s->bar_valid[odd] || s->bar_frame[odd] != frame) {
            pattern_row(s, frame, y, s->rgb);
            convert_row(s, s->rgb, y, s->bar_row[odd]);
            s->bar_frame[odd] = frame;
            s->bar_valid[odd] = true;
        }
        return s->bar_row[odd];
    }
    pattern_row(s, frame, y, s->rgb);
    convert_row(s, s->rgb, y, s->line_buf);
    return s->line_buf;
}

/* A line on the DVP data lines, a sample a PCLK. */
static void dvp_samples(ESP32P4ImageSource *s, const uint8_t *p,
                        uint16_t *out)
{
    const ESP32P4ImageTiming *t = &s->timing;
    uint32_t i;

    switch (s->format) {
    case ESP32P4_IMAGE_RAW10:
        for (i = 0; i < t->width; i++) {
            const uint8_t *g = p + i / 4 * 5;

            out[i] = g[i % 4] << 2 | extract32(g[4], 2 * (i % 4), 2);
        }
        break;
    case ESP32P4_IMAGE_RAW12:
        for (i = 0; i < t->width; i++) {
            const uint8_t *g = p + i / 2 * 3;

            out[i] = g[i % 2] << 4 | extract32(g[2], 4 * (i % 2), 4);
        }
        break;
    default:
        if (t->dvp_bits == 16 && !format_is_raw(s->format)) {
            for (i = 0; i < t->active_clocks; i++) {
                out[i] = lduw_le_p(p + 2 * i);
            }
        } else {
            for (i = 0; i < t->active_clocks; i++) {
                out[i] = p[i];
            }
        }
        break;
    }
}

/*
 * CSI-2 header ECC and payload CRC
 * ================================
 */

/* [spec:nuos:req:emu.esp32p4.mipi] */
uint8_t esp32p4_csi2_ecc(uint32_t header)
{
    /* The header bits each parity bit covers (CSI-2's ECC table) */
    static const uint32_t parity[6] = {
        0xf12cb7, 0xf2555b, 0x749a6d, 0xb8e38e, 0xdf03f0, 0xeffc00,
    };
    uint8_t ecc = 0;
    int i;

    for (i = 0; i < 6; i++) {
        ecc |= (ctpop32(header & parity[i]) & 1) << i;
    }
    return ecc;
}

/* [spec:nuos:req:emu.esp32p4.mipi] */
uint16_t esp32p4_csi2_crc16(const uint8_t *data, size_t len)
{
    static uint16_t table[256];
    uint16_t crc = 0xffff;
    size_t i;

    if (!table[1]) {
        for (i = 0; i < 256; i++) {
            uint16_t c = i;
            int b;

            for (b = 0; b < 8; b++) {
                c = c & 1 ? (c >> 1) ^ 0x8408 : c >> 1;
            }
            table[i] = c;
        }
    }
    for (i = 0; i < len; i++) {
        crc = (crc >> 8) ^ table[(crc ^ data[i]) & 0xff];
    }
    return crc;
}

/*
 * Running the stream
 * ==================
 */

static bool pin_asserted(ESP32P4ImageSource *s, unsigned pin)
{
    bool active_high = pin == PIN_PWDN ? !s->pwdn_low : s->reset_high;

    if (!s->pin_irq[pin]) {
        return false;
    }
    return s->pin_level[pin] == active_high;
}

static bool powered(ESP32P4ImageSource *s)
{
    return !pin_asserted(s, PIN_PWDN) && !pin_asserted(s, PIN_RESET);
}

static bool sync_level(ESP32P4ImageSource *s, unsigned i)
{
    static const size_t low[SYNCS] = {
        offsetof(ESP32P4ImageSource, vsync_low),
        offsetof(ESP32P4ImageSource, hsync_low),
        offsetof(ESP32P4ImageSource, de_low),
    };

    return s->sync_active[i] != *(bool *)((char *)s + low[i]);
}

/* Drive DVP output @i's level onto its pad, or let the pad go. */
static void sync_pad_update(ESP32P4ImageSource *s, unsigned i)
{
    if (!s->sync_in[i]) {
        return;
    }
    if (powered(s)) {
        qemu_set_irq(s->sync_in[i], sync_level(s, i));
    } else {
        qemu_set_irq(s->sync_release[i], 1);
    }
}

static void set_sync(ESP32P4ImageSource *s, unsigned i, bool active)
{
    s->sync_active[i] = active;
    sync_pad_update(s, i);
}

static void arm(ESP32P4ImageSource *s)
{
    if (s->running && s->next_ev < s->events->len) {
        timer_mod(&s->timer,
                  g_array_index(s->events, ImageEvent, s->next_ev).ns);
    } else {
        timer_del(&s->timer);
    }
}

static void notify_stream(ESP32P4ImageSource *s, bool on, int64_t when)
{
    guint i;

    for (i = 0; i < s->sinks->len; i++) {
        ImageSink *k = &g_array_index(s->sinks, ImageSink, i);

        if (k->ops->stream) {
            k->ops->stream(k->opaque, on, when);
        }
    }
}

static void start_frame(ESP32P4ImageSource *s)
{
    s->frame = s->next_frame++;
    s->next_ev = 0;
    build_frame_events(s);
}

static void start_stream(ESP32P4ImageSource *s, int64_t when)
{
    s->running = true;
    s->stopping = false;
    s->epoch_ns = when;
    s->run_frame0 = s->next_frame;
    start_frame(s);
    notify_stream(s, true, when);
}

static void stop_stream(ESP32P4ImageSource *s, int64_t when)
{
    unsigned i;

    s->running = false;
    s->stopping = false;
    g_array_set_size(s->events, 0);
    s->next_ev = 0;
    for (i = 0; i < SYNCS; i++) {
        set_sync(s, i, false);
    }
    notify_stream(s, false, when);
}

static void run_dvp_line(ESP32P4ImageSource *s, const ImageEvent *ev)
{
    const ESP32P4ImageTiming *t = &s->timing;
    ESP32P4DvpLine line = {
        .frame = s->frame,
        .line = ev->line,
        .samples = s->samples,
        .count = t->active_clocks,
        .hsync_ns = ev->t[0],
        .hsync_end_ns = ev->t[1],
        .de_ns = ev->t[2],
        .de_end_ns = ev->t[3],
        .epoch_ns = s->epoch_ns,
        .clock = ((uint64_t)(s->frame - s->run_frame0) * t->frame_lines +
                  t->active_line + ev->line) * t->line_clocks +
                 t->active_start,
        .pclk_hz = t->clock_hz,
    };
    guint i;

    set_sync(s, SYNC_DE, false);
    if (!s->sinks->len) {
        return;
    }
    dvp_samples(s, frame_line(s, s->frame, ev->line), s->samples);
    for (i = 0; i < s->sinks->len; i++) {
        ImageSink *k = &g_array_index(s->sinks, ImageSink, i);

        if (k->ops->dvp_line) {
            k->ops->dvp_line(k->opaque, &line);
        }
    }
}

static void run_csi_packet(ESP32P4ImageSource *s, const ImageEvent *ev)
{
    const ESP32P4ImageTiming *t = &s->timing;
    bool is_long = ev->dt == t->data_type;
    uint8_t di = t->vc << 6 | ev->dt;
    ESP32P4CsiPacket p = {
        .frame = s->frame,
        .line = ev->line,
        .vc = t->vc,
        .dt = ev->dt,
        .wc = ev->data,
        .ecc = esp32p4_csi2_ecc(di | ev->data << 8),
        .sot_ns = ev->t[0],
        .data_ns = ev->t[1],
        .end_ns = ev->t[2],
        .eot_ns = ev->t[3],
    };
    guint i;

    if (!s->sinks->len) {
        return;
    }
    if (is_long) {
        p.payload = frame_line(s, s->frame, ev->line);
        p.crc = esp32p4_csi2_crc16(p.payload, t->line_bytes);
    }
    for (i = 0; i < s->sinks->len; i++) {
        ImageSink *k = &g_array_index(s->sinks, ImageSink, i);

        if (k->ops->csi_packet) {
            k->ops->csi_packet(k->opaque, &p);
        }
    }
}

static void run_vsync(ESP32P4ImageSource *s, bool active, int64_t when)
{
    guint i;

    set_sync(s, SYNC_VSYNC, active);
    for (i = 0; i < s->sinks->len; i++) {
        ImageSink *k = &g_array_index(s->sinks, ImageSink, i);

        if (k->ops->vsync) {
            k->ops->vsync(k->opaque, s->frame, active, when);
        }
    }
}

/* Run the event at s->next_ev, its time esp32_line_time(). */
static void run_event(void *opaque)
{
    ESP32P4ImageSource *s = opaque;
    ImageEvent ev = g_array_index(s->events, ImageEvent, s->next_ev++);

    switch (ev.kind) {
    case EV_VSYNC_ON:
    case EV_VSYNC_OFF:
        run_vsync(s, ev.kind == EV_VSYNC_ON, ev.ns);
        break;
    case EV_HSYNC_ON:
    case EV_HSYNC_OFF:
        set_sync(s, SYNC_HSYNC, ev.kind == EV_HSYNC_ON);
        break;
    case EV_DE_ON:
        set_sync(s, SYNC_DE, true);
        break;
    case EV_DVP_LINE:
        run_dvp_line(s, &ev);
        break;
    case EV_CSI_PACKET:
        run_csi_packet(s, &ev);
        break;
    case EV_FRAME_END:
        if (s->stopping) {
            stop_stream(s, ev.ns);
        } else {
            start_frame(s);
        }
        break;
    }
}

/* Run every event due by @now, each at its own time. */
static void run_until(ESP32P4ImageSource *s, int64_t now)
{
    if (s->in_run) {
        return;
    }
    s->in_run = true;
    while (s->running && s->next_ev < s->events->len) {
        int64_t when = g_array_index(s->events, ImageEvent, s->next_ev).ns;

        if (when > now) {
            break;
        }
        esp32_line_run_at(when, run_event, s);
    }
    s->in_run = false;
}

/*
 * The timer runs the events at its deadline, the next event's time, and
 * no later ones even if it runs late: with the timers running in deadline
 * order, the sensor's events reach the receivers in order of time with
 * every other device's.
 */
static void image_source_timer(void *opaque)
{
    ESP32P4ImageSource *s = opaque;

    if (s->running && s->next_ev < s->events->len) {
        run_until(s, g_array_index(s->events, ImageEvent, s->next_ev).ns);
    }
    arm(s);
}

/*
 * The stream's controls changed at esp32_line_time(): the events due by
 * then have run, and the stream starts, stops or carries on as they say.
 */
static void update_stream(ESP32P4ImageSource *s)
{
    int64_t now = esp32_line_time();

    if (s->timing.port == ESP32P4_IMAGE_PORT_NONE) {
        return;
    }
    run_until(s, now);
    if (s->enabled && powered(s)) {
        if (!s->running) {
            start_stream(s, now);
        }
        s->stopping = false;
    } else if (s->running) {
        if (powered(s)) {
            s->stopping = true;
        } else {
            stop_stream(s, now);
        }
    }
    arm(s);
}

/*
 * Software samples the pads: the sync outputs on them come up to the
 * present, even with this sensor's timer still to run.
 */
static void image_source_line_sync(Notifier *n, void *data)
{
    ESP32P4ImageSource *s = container_of(n, ESP32P4ImageSource, line_source);

    if (!s->in_run) {
        run_until(s, esp32_line_time());
        arm(s);
    }
}

static void image_source_pin(void *opaque, int n, int level)
{
    ESP32P4ImageSource *s = opaque;
    unsigned i;

    if (s->pin_level[n] == !!level) {
        return;
    }
    s->pin_level[n] = level;
    update_stream(s);
    for (i = 0; i < SYNCS; i++) {
        sync_pad_update(s, i);
    }
}

/*
 * The receivers' API
 * ==================
 */

static bool csi_line_fits(const ESP32P4ImageTiming *t, int64_t line_ns,
                          int64_t *line_end)
{
    int64_t sot = 0, out[4];

    if (t->line_sync) {
        csi_burst(t, sot, CSI2_SHORT_BYTES, out);
        sot = out[3] + DPHY_EXIT_NS;
    }
    csi_burst(t, sot, t->line_bytes + CSI2_LONG_OVERHEAD, out);
    sot = out[3] + DPHY_EXIT_NS;
    if (t->line_sync) {
        csi_burst(t, sot, CSI2_SHORT_BYTES, out);
        sot = out[3] + DPHY_EXIT_NS;
    }
    *line_end = sot;
    return sot <= line_ns;
}

/*
 * Whether the bursts of a CSI frame keep to their lines: each line's
 * within @share percent of the shortest line, FS before the first line of
 * pixels, FE before the frame ends.
 */
static bool csi_frame_fits(const ESP32P4ImageTiming *t, unsigned share)
{
    int64_t line_ns = cycles_ns(t->line_clocks, t->clock_hz) - 1;
    int64_t short_ns, line_end, out[4];
    uint32_t after = t->frame_lines - t->active_line - t->height;

    csi_burst(t, 0, CSI2_SHORT_BYTES, out);
    short_ns = out[3] + DPHY_EXIT_NS;
    return csi_line_fits(t, line_ns * share / 100, &line_end) &&
           short_ns <= line_ns * t->active_line &&
           line_end + short_ns <= line_ns * (1 + after);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
bool esp32p4_image_source_bind(ESP32P4ImageSource *s, ESP32P4ImagePort port,
                               Error **errp)
{
    static const char *const names[] = { "no", "DVP", "MIPI CSI" };
    ESP32P4ImageTiming *t = &s->timing;
    uint32_t i;

    if (t->port == port) {
        return true;
    }
    if (t->port != ESP32P4_IMAGE_PORT_NONE) {
        error_setg(errp, "esp32p4-image-source: the sensor is wired to the "
                   "%s port, not %s", names[t->port], names[port]);
        return false;
    }
    if (port == ESP32P4_IMAGE_PORT_DVP) {
        if (format_is_raw(s->format)) {
            t->dvp_bits = raw_bits(s->format);
            t->dvp_clocks = 1;
        } else if (s->dvp_width == 16) {
            if (s->format == ESP32P4_IMAGE_RGB888 ||
                s->format == ESP32P4_IMAGE_BGR888) {
                error_setg(errp, "esp32p4-image-source: RGB888 goes a byte "
                           "a PCLK (dvp-width=8)");
                return false;
            }
            t->dvp_bits = 16;
            t->dvp_clocks = 1;
        } else {
            t->dvp_bits = 8;
            t->dvp_clocks = t->line_bytes / t->width;
        }
        t->clock_hz = t->pixel_hz * t->dvp_clocks;
        t->line_clocks = s->hts * t->dvp_clocks;
        t->hsync_clocks = s->hsync * t->dvp_clocks;
        t->active_start = (s->hsync + s->hbp) * t->dvp_clocks;
        t->active_clocks = s->width * t->dvp_clocks;
        s->samples = g_new0(uint16_t, t->active_clocks);
    } else {
        for (i = ESP32P4_DVP_PCLK; i < ESP32P4_DVP_SIGNALS; i++) {
            if (s->pad[i] >= 0) {
                error_setg(errp, "esp32p4-image-source: a MIPI CSI sensor "
                           "has no DVP pads");
                return false;
            }
        }
        t->clock_hz = t->pixel_hz;
        t->line_clocks = s->hts;
        t->hsync_clocks = 0;
        t->active_start = 0;
        t->active_clocks = s->width;
        t->lanes = s->lanes;
        if (s->lane_rate) {
            t->lane_bps = s->lane_rate;
            if (!csi_frame_fits(t, 100)) {
                error_setg(errp, "esp32p4-image-source: at %" PRIu64
                           " bit/s on %u lanes the packets do not fit in "
                           "their lines (raise lane-rate or the blanking)",
                           t->lane_bps, t->lanes);
                return false;
            }
        } else {
            for (t->lane_bps = LANE_RATE_MIN;
                 !csi_frame_fits(t, 80); t->lane_bps += LANE_RATE_STEP) {
                if (t->lane_bps >= LANE_RATE_MAX) {
                    error_setg(errp, "esp32p4-image-source: this mode needs "
                               "more than the D-PHY's %llu bit/s on each of "
                               "%u lanes (lower fps or the size, raise the "
                               "blanking, or set lane-rate)",
                               LANE_RATE_MAX, t->lanes);
                    return false;
                }
            }
            s->lane_rate = t->lane_bps;
        }
    }
    t->port = port;
    update_stream(s);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
void esp32p4_image_source_attach(ESP32P4ImageSource *s,
                                 const ESP32P4ImageSinkOps *ops,
                                 void *opaque)
{
    ImageSink k = { .ops = ops, .opaque = opaque };

    assert(s->timing.port != ESP32P4_IMAGE_PORT_NONE);
    g_array_append_val(s->sinks, k);
}

void esp32p4_image_source_detach(ESP32P4ImageSource *s, void *opaque)
{
    guint i;

    for (i = 0; i < s->sinks->len; i++) {
        if (g_array_index(s->sinks, ImageSink, i).opaque == opaque) {
            g_array_remove_index(s->sinks, i);
            return;
        }
    }
}

const ESP32P4ImageTiming *esp32p4_image_source_timing(ESP32P4ImageSource *s)
{
    return &s->timing;
}

bool esp32p4_image_source_streaming(ESP32P4ImageSource *s)
{
    if (!s->in_run) {
        run_until(s, esp32_line_time());
        arm(s);
    }
    return s->running;
}

/*
 * [spec:nuos:req:emu.esp32p4.mipi]
 * Bursts do not overlap and the events are in order of their ends, so a
 * burst in progress is the next event's, or in its trail the last run's.
 */
bool esp32p4_image_source_csi_in_burst(ESP32P4ImageSource *s)
{
    int64_t now = esp32_line_time();
    guint i;

    if (!s->in_run) {
        run_until(s, now);
        arm(s);
    }
    if (!s->running || s->timing.port != ESP32P4_IMAGE_PORT_CSI) {
        return false;
    }
    for (i = MAX(s->next_ev, 1) - 1; i <= s->next_ev && i < s->events->len;
         i++) {
        const ImageEvent *ev = &g_array_index(s->events, ImageEvent, i);

        if (ev->kind == EV_CSI_PACKET && ev->t[0] <= now && now < ev->t[3]) {
            return true;
        }
    }
    return false;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
void esp32p4_image_source_set_streaming(ESP32P4ImageSource *s, bool on)
{
    s->enabled = on;
    update_stream(s);
}

int esp32p4_image_source_dvp_pad(ESP32P4ImageSource *s, ESP32P4DvpSignal sig)
{
    return s->pad[sig];
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
void esp32p4_image_source_connect_pads(ESP32P4ImageSource *s,
                                       DeviceState *gpio)
{
    static const ESP32P4DvpSignal sync_sig[SYNCS] = {
        ESP32P4_DVP_VSYNC, ESP32P4_DVP_HSYNC, ESP32P4_DVP_DE,
    };
    unsigned i;

    assert(!s->pads_connected);
    s->pads_connected = true;
    for (i = 0; i < SYNCS; i++) {
        int64_t pad = s->pad[sync_sig[i]];

        if (pad >= 0) {
            s->sync_in[i] = qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_PAD_IN,
                                                   pad);
            s->sync_release[i] =
                qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_PAD_RELEASE, pad);
        }
    }
    if (sync_pads_assigned(s)) {
        s->line_source.notify = image_source_line_sync;
        esp32_line_add_source(&s->line_source);
    }
    for (i = 0; i < PINS; i++) {
        if (s->pin_pad[i] >= 0) {
            s->pin_irq[i] = qemu_allocate_irq(image_source_pin, s, i);
            qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_PAD_OUT,
                                        s->pin_pad[i], s->pin_irq[i]);
        }
    }
    update_stream(s);
    for (i = 0; i < SYNCS; i++) {
        sync_pad_update(s, i);
    }
}

/*
 * Setting up
 * ==========
 */

static bool setup_content(ESP32P4ImageSource *s, ESP32P4Picture *pic,
                          Error **errp)
{
    const ESP32P4ImageTiming *t = &s->timing;
    uint32_t y;

    s->frame_bytes = (size_t)t->height * t->line_bytes;
    s->rgb = g_new0(ESP32P4Rgb16, t->width);
    s->line_buf = g_malloc0(t->line_bytes);
    if (s->file) {
        g_autoptr(GError) gerr = NULL;
        size_t len;

        s->mapped = g_mapped_file_new(s->file, false, &gerr);
        if (!s->mapped) {
            error_setg(errp, "esp32p4-image-source: %s", gerr->message);
            return false;
        }
        len = g_mapped_file_get_length(s->mapped);
        if (!len || len % s->frame_bytes) {
            error_setg(errp, "esp32p4-image-source: %s is %zu bytes, not a "
                       "whole number of %ux%u frames of %zu bytes", s->file,
                       len, t->width, t->height, s->frame_bytes);
            return false;
        }
        s->frames = (const uint8_t *)g_mapped_file_get_contents(s->mapped);
        s->nframes = MIN(len / s->frame_bytes, UINT32_MAX);
    } else if (pic->pixels) {
        ESP32P4Picture scaled = { 0 };
        const ESP32P4Picture *p = pic;

        if (pic->width != t->width || pic->height != t->height) {
            esp32p4_picture_scale(pic, t->width, t->height, &scaled);
            p = &scaled;
        }
        s->still = g_malloc0(s->frame_bytes);
        for (y = 0; y < t->height; y++) {
            convert_row(s, &p->pixels[(size_t)y * t->width], y,
                        s->still + (size_t)y * t->line_bytes);
        }
        esp32p4_picture_free(&scaled);
    } else {
        s->bar_row[0] = g_malloc0(t->line_bytes);
        s->bar_row[1] = g_malloc0(t->line_bytes);
    }
    return true;
}

static bool parse_data_pads(ESP32P4ImageSource *s, Error **errp)
{
    g_auto(GStrv) items = NULL;
    unsigned i;

    if (!s->data_pads || !*s->data_pads) {
        return true;
    }
    /* ':' as well as ',', which -object's syntax would have doubled */
    items = g_strsplit_set(s->data_pads, ",:", -1);
    if (g_strv_length(items) > 16) {
        error_setg(errp, "esp32p4-image-source: data-pads has more than 16 "
                   "pads");
        return false;
    }
    for (i = 0; items[i]; i++) {
        int64_t pad;

        if (qemu_strtoi64(g_strstrip(items[i]), NULL, 0, &pad) ||
            pad < -1 || pad >= ESP32P4_GPIO_PADS) {
            error_setg(errp, "esp32p4-image-source: data-pads: '%s' is not a "
                       "pad (0 to %d, or -1)", items[i],
                       ESP32P4_GPIO_PADS - 1);
            return false;
        }
        s->pad[ESP32P4_DVP_D0 + i] = pad;
    }
    return true;
}

static bool check_pads(ESP32P4ImageSource *s, Error **errp)
{
    uint64_t used = 0;
    int64_t pads[ESP32P4_DVP_SIGNALS + PINS];
    unsigned i;

    memcpy(pads, s->pad, sizeof(s->pad));
    memcpy(pads + ESP32P4_DVP_SIGNALS, s->pin_pad, sizeof(s->pin_pad));
    for (i = 0; i < ARRAY_SIZE(pads); i++) {
        if (pads[i] < 0) {
            continue;
        }
        if (used & BIT_ULL(pads[i])) {
            error_setg(errp, "esp32p4-image-source: GPIO%" PRId64 " is given "
                       "to two of the sensor's signals", pads[i]);
            return false;
        }
        used |= BIT_ULL(pads[i]);
    }
    return true;
}

static bool image_source_setup(ESP32P4ImageSource *s, ESP32P4Picture *pic,
                               Error **errp)
{
    ESP32P4ImageTiming *t = &s->timing;
    uint64_t total;

    if (s->file && s->image) {
        error_setg(errp, "esp32p4-image-source: give file or image, not "
                   "both");
        return false;
    }
    if (s->pattern >= 0 && (s->file || s->image)) {
        error_setg(errp, "esp32p4-image-source: a pattern is for a sensor "
                   "with no file or image");
        return false;
    }
    if (s->image && !esp32p4_picture_load(s->image, pic, errp)) {
        return false;
    }
    if (s->pattern < 0) {
        s->pattern = PATTERN_BARS;
    }
    if (s->width < 0) {
        s->width = pic->pixels ? pic->width : 640;
    }
    if (s->height < 0) {
        s->height = pic->pixels ? pic->height : 480;
    }
    if ((s->format == ESP32P4_IMAGE_RAW10 && s->width % 4) ||
        ((s->format == ESP32P4_IMAGE_RAW12 || format_is_yuv(s->format)) &&
         s->width % 2)) {
        error_setg(errp, "esp32p4-image-source: a width of %" PRId64
                   " does not divide into the format's groups of %d pixels",
                   s->width, s->format == ESP32P4_IMAGE_RAW10 ? 4 : 2);
        return false;
    }
    if (s->cell < 0) {
        s->cell = 16;
    }

    if (s->hts >= 0) {
        if (s->hts <= s->width ||
            (s->hblank >= 0 && s->hblank != s->hts - s->width)) {
            error_setg(errp, "esp32p4-image-source: hts must be width + "
                       "hblank, more than width");
            return false;
        }
        s->hblank = s->hts - s->width;
    } else {
        if (s->hblank < 0) {
            s->hblank = MAX(s->width / 4, 16);
        }
        s->hts = s->width + s->hblank;
    }
    if (s->hsync < 0) {
        s->hsync = MAX(s->hblank / 8, 1);
    }
    if (s->hbp < 0) {
        s->hbp = s->hblank / 4;
    }
    if (s->vts >= 0) {
        if (s->vts <= s->height ||
            (s->vblank >= 0 && s->vblank != s->vts - s->height)) {
            error_setg(errp, "esp32p4-image-source: vts must be height + "
                       "vblank, more than height");
            return false;
        }
        s->vblank = s->vts - s->height;
    } else {
        if (s->vblank < 0) {
            s->vblank = MAX(DIV_ROUND_UP(s->height, 16), 3);
        }
        s->vts = s->height + s->vblank;
    }
    if (s->vsync < 0) {
        s->vsync = MAX(s->vblank / 10, 1);
    }
    if (s->vbp < 0) {
        s->vbp = s->vblank / 2;
    }
    if (!s->hsync || s->hsync + s->hbp > s->hblank) {
        error_setg(errp, "esp32p4-image-source: hsync (at least 1) and hbp "
                   "must fit in hblank");
        return false;
    }
    if (!s->vsync || s->vsync + s->vbp > s->vblank) {
        error_setg(errp, "esp32p4-image-source: vsync (at least 1) and vbp "
                   "must fit in vblank");
        return false;
    }
    total = (uint64_t)s->hts * s->vts;
    if (s->pixel_clock > 0) {
        if (s->fps > 0 && s->fps * total != s->pixel_clock) {
            error_setg(errp, "esp32p4-image-source: fps %" PRId64 " is not "
                       "pixel-clock / (hts * vts)", s->fps);
            return false;
        }
        s->fps = s->pixel_clock % total ? 0 : s->pixel_clock / total;
    } else {
        if (s->fps < 0) {
            s->fps = 30;
        }
        if (!s->fps) {
            error_setg(errp, "esp32p4-image-source: fps must be at least 1");
            return false;
        }
        s->pixel_clock = s->fps * total;
    }
    if (s->dvp_width != 8 && s->dvp_width != 16) {
        error_setg(errp, "esp32p4-image-source: dvp-width is 8 or 16");
        return false;
    }
    if (s->dvp_width == 16 && format_is_raw(s->format)) {
        error_setg(errp, "esp32p4-image-source: a RAW pixel is one sample "
                   "of its own width; dvp-width is for the other formats");
        return false;
    }
    if (!parse_data_pads(s, errp) || !check_pads(s, errp)) {
        return false;
    }

    t->format = s->format;
    t->bayer = s->bayer;
    t->width = s->width;
    t->height = s->height;
    t->data_type = format_data_type(s->format);
    t->line_bytes = format_line_bytes(s->format, s->width);
    t->pixel_hz = s->pixel_clock;
    t->frame_lines = s->vts;
    t->vsync_lines = s->vsync;
    t->active_line = s->vsync + s->vbp;
    t->vsync_active_low = s->vsync_low;
    t->hsync_active_low = s->hsync_low;
    t->de_active_low = s->de_low;
    t->vc = s->vc;
    t->line_sync = s->line_sync;
    t->clock_continuous = s->clock_continuous;
    return setup_content(s, pic, errp);
}

static const VMStateDescription vmstate_image_source;

static void image_source_complete(UserCreatable *uc, Error **errp)
{
    ESP32P4ImageSource *s = ESP32P4_IMAGE_SOURCE(uc);
    ESP32P4Picture pic = { 0 };
    bool ok = image_source_setup(s, &pic, errp);

    esp32p4_picture_free(&pic);
    if (!ok) {
        return;
    }
    s->complete = true;
    vmstate_register_any(NULL, &vmstate_image_source, s);
}

static bool image_source_prepare_delete(UserCreatable *uc, Error **errp)
{
    if (ESP32P4_IMAGE_SOURCE(uc)->timing.port != ESP32P4_IMAGE_PORT_NONE) {
        error_setg(errp, "esp32p4-image-source: the sensor is wired to the "
                   "board");
        return false;
    }
    return true;
}

static int image_source_post_load(void *opaque, int version_id)
{
    ESP32P4ImageSource *s = opaque;

    if (s->running) {
        uint32_t next = s->next_ev;

        build_frame_events(s);
        s->next_ev = MIN(next, s->events->len);
    } else {
        g_array_set_size(s->events, 0);
    }
    return 0;
}

static const VMStateDescription vmstate_image_source = {
    .name = TYPE_ESP32P4_IMAGE_SOURCE,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = image_source_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(enabled, ESP32P4ImageSource),
        VMSTATE_BOOL(running, ESP32P4ImageSource),
        VMSTATE_BOOL(stopping, ESP32P4ImageSource),
        VMSTATE_INT64(epoch_ns, ESP32P4ImageSource),
        VMSTATE_UINT32(run_frame0, ESP32P4ImageSource),
        VMSTATE_UINT32(frame, ESP32P4ImageSource),
        VMSTATE_UINT32(next_frame, ESP32P4ImageSource),
        VMSTATE_UINT32(next_ev, ESP32P4ImageSource),
        VMSTATE_BOOL_ARRAY(pin_level, ESP32P4ImageSource, PINS),
        VMSTATE_BOOL_ARRAY(sync_active, ESP32P4ImageSource, SYNCS),
        VMSTATE_TIMER(timer, ESP32P4ImageSource),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * Properties
 * ==========
 */

typedef struct IntProp {
    const char *name;
    size_t offset;
    int64_t min;
    int64_t max;
    const char *desc;
} IntProp;

#define INT_PROP(n, field, lo, hi, d) \
    { n, offsetof(ESP32P4ImageSource, field), lo, hi, d }

static const IntProp int_props[] = {
    INT_PROP("width", width, 1, 8192, "Pixels a line (default 640, or "
             "the image's)"),
    INT_PROP("height", height, 1, 8192, "Lines of pixels a frame (default "
             "480, or the image's)"),
    INT_PROP("fps", fps, 0, 10000, "Frames a second (default 30)"),
    INT_PROP("pixel-clock", pixel_clock, 1, 10000000000LL,
             "Pixel clock in Hz (default fps * hts * vts)"),
    INT_PROP("hblank", hblank, 1, 65535, "Pixel periods of horizontal "
             "blanking (default width / 4, at least 16)"),
    INT_PROP("vblank", vblank, 1, 65535, "Lines of vertical blanking "
             "(default height / 16, at least 3)"),
    INT_PROP("hts", hts, 2, 65535, "Pixel periods a line: width + hblank"),
    INT_PROP("vts", vts, 2, 65535, "Lines a frame: height + vblank"),
    INT_PROP("hsync", hsync, 1, 65535, "HSYNC's pixel periods (default "
             "hblank / 8, at least 1)"),
    INT_PROP("hbp", hbp, 0, 65535, "Pixel periods from HSYNC's end to the "
             "first pixel (default hblank / 4)"),
    INT_PROP("vsync", vsync, 1, 65535, "VSYNC's lines (default vblank / 10, "
             "at least 1)"),
    INT_PROP("vbp", vbp, 0, 65535, "Lines from VSYNC's end to the first "
             "line of pixels (default vblank / 2)"),
    INT_PROP("cell", cell, 2, 4096, "The test patterns' square, in pixels "
             "(even; default 16)"),
    INT_PROP("dvp-width", dvp_width, 8, 16, "DVP: data lines for a byte "
             "format, 8 or 16 (default 8)"),
    INT_PROP("pclk-pad", pad[ESP32P4_DVP_PCLK], -1, ESP32P4_GPIO_PADS - 1,
             "DVP: the pad PCLK is wired to"),
    INT_PROP("vsync-pad", pad[ESP32P4_DVP_VSYNC], -1, ESP32P4_GPIO_PADS - 1,
             "DVP: the pad VSYNC is wired to, and driven onto"),
    INT_PROP("hsync-pad", pad[ESP32P4_DVP_HSYNC], -1, ESP32P4_GPIO_PADS - 1,
             "DVP: the pad HSYNC is wired to, and driven onto"),
    INT_PROP("de-pad", pad[ESP32P4_DVP_DE], -1, ESP32P4_GPIO_PADS - 1,
             "DVP: the pad DE (HREF) is wired to, and driven onto"),
    INT_PROP("pwdn-pad", pin_pad[PIN_PWDN], -1, ESP32P4_GPIO_PADS - 1,
             "The pad driving the sensor's PWDN"),
    INT_PROP("reset-pad", pin_pad[PIN_RESET], -1, ESP32P4_GPIO_PADS - 1,
             "The pad driving the sensor's RESETB"),
    INT_PROP("lanes", lanes, 1, 4, "CSI: data lanes (default 2)"),
    INT_PROP("lane-rate", lane_rate, 1, 10000000000LL, "CSI: bit/s on each "
             "lane (default: enough for the mode, from 80 Mbit/s)"),
    INT_PROP("vc", vc, 0, 3, "CSI: the packets' virtual channel"),
};

typedef struct ChoiceProp {
    const char *name;
    size_t offset;
    const char *const *values;
    const char *desc;
} ChoiceProp;

static const char *const pattern_values[] = { "bars", "checker", NULL };
static const char *const bayer_values[] = {
    "rggb", "grbg", "gbrg", "bggr", NULL,
};
static const char *const matrix_values[] = { "bt601", "bt709", NULL };
static const char *const range_values[] = { "full", "limited", NULL };

static const ChoiceProp choice_props[] = {
    { "pattern", offsetof(ESP32P4ImageSource, pattern), pattern_values,
      "The test pattern with no file or image: bars (default), checker" },
    { "bayer", offsetof(ESP32P4ImageSource, bayer), bayer_values,
      "RAW: the colour filter array's first pixels (default rggb)" },
    { "yuv-matrix", offsetof(ESP32P4ImageSource, yuv_matrix), matrix_values,
      "YUV: bt601 (default) or bt709" },
    { "yuv-range", offsetof(ESP32P4ImageSource, yuv_range), range_values,
      "YUV: full (default) or limited" },
};

typedef struct BoolProp {
    const char *name;
    size_t offset;
    const char *desc;
} BoolProp;

static const BoolProp bool_props[] = {
    { "vsync-active-low", offsetof(ESP32P4ImageSource, vsync_low),
      "DVP: VSYNC active low" },
    { "hsync-active-low", offsetof(ESP32P4ImageSource, hsync_low),
      "DVP: HSYNC active low" },
    { "de-active-low", offsetof(ESP32P4ImageSource, de_low),
      "DVP: DE active low" },
    { "pwdn-active-low", offsetof(ESP32P4ImageSource, pwdn_low),
      "PWDN powers the sensor down while low (default: while high)" },
    { "reset-active-high", offsetof(ESP32P4ImageSource, reset_high),
      "RESETB holds the sensor in reset while high (default: while low)" },
    { "line-sync", offsetof(ESP32P4ImageSource, line_sync),
      "CSI: line start and end packets around each line" },
    { "clock-continuous", offsetof(ESP32P4ImageSource, clock_continuous),
      "CSI: the clock lane stays in high speed between bursts (default on)" },
};

typedef struct StrProp {
    const char *name;
    size_t offset;
    const char *desc;
} StrProp;

static const StrProp str_props[] = {
    { "file", offsetof(ESP32P4ImageSource, file),
      "A file of raw frames in the format's layout, played in a loop" },
    { "image", offsetof(ESP32P4ImageSource, image),
      "A still picture: binary PPM or PGM, or PNG" },
    { "data-pads", offsetof(ESP32P4ImageSource, data_pads),
      "DVP: the pads D0 upwards are wired to, such as 10:11:12 (-1 none)" },
};

static bool settable(ESP32P4ImageSource *s, const char *name, Error **errp)
{
    if (s->complete) {
        error_setg(errp, "esp32p4-image-source: '%s' can no longer be set",
                   name);
        return false;
    }
    return true;
}

static void get_int(Object *obj, Visitor *v, const char *name, void *opaque,
                    Error **errp)
{
    const IntProp *p = opaque;

    visit_type_int64(v, name, (int64_t *)((char *)obj + p->offset), errp);
}

static void set_int(Object *obj, Visitor *v, const char *name, void *opaque,
                    Error **errp)
{
    const IntProp *p = opaque;
    int64_t value;

    if (!settable(ESP32P4_IMAGE_SOURCE(obj), name, errp) ||
        !visit_type_int64(v, name, &value, errp)) {
        return;
    }
    if (value < p->min || value > p->max) {
        error_setg(errp, "esp32p4-image-source: '%s' is %" PRId64 " to %"
                   PRId64, name, p->min, p->max);
        return;
    }
    if (!strcmp(name, "cell") && value % 2) {
        error_setg(errp, "esp32p4-image-source: 'cell' must be even, so that "
                   "a square covers whole Bayer and YUV groups");
        return;
    }
    *(int64_t *)((char *)obj + p->offset) = value;
}

static void get_choice(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    const ChoiceProp *p = opaque;
    int64_t value = *(int64_t *)((char *)obj + p->offset);
    char *str = (char *)p->values[MAX(value, 0)];

    visit_type_str(v, name, &str, errp);
}

static void set_choice(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    const ChoiceProp *p = opaque;
    g_autofree char *str = NULL;
    int64_t i;

    if (!settable(ESP32P4_IMAGE_SOURCE(obj), name, errp) ||
        !visit_type_str(v, name, &str, errp)) {
        return;
    }
    for (i = 0; p->values[i]; i++) {
        if (!strcmp(p->values[i], str)) {
            *(int64_t *)((char *)obj + p->offset) = i;
            return;
        }
    }
    error_setg(errp, "esp32p4-image-source: '%s' is not a %s", str, name);
}

static void get_bool_prop(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    const BoolProp *p = opaque;
    bool value = *(bool *)((char *)obj + p->offset);

    visit_type_bool(v, name, &value, errp);
}

static void set_bool_prop(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    const BoolProp *p = opaque;
    bool value;

    if (!settable(ESP32P4_IMAGE_SOURCE(obj), name, errp) ||
        !visit_type_bool(v, name, &value, errp)) {
        return;
    }
    *(bool *)((char *)obj + p->offset) = value;
}

static void get_str(Object *obj, Visitor *v, const char *name, void *opaque,
                    Error **errp)
{
    const StrProp *p = opaque;
    char *str = *(char **)((char *)obj + p->offset);

    str = str ? str : (char *)"";
    visit_type_str(v, name, &str, errp);
}

static void set_str(Object *obj, Visitor *v, const char *name, void *opaque,
                    Error **errp)
{
    const StrProp *p = opaque;
    char **field = (char **)((char *)obj + p->offset);
    char *str;

    if (!settable(ESP32P4_IMAGE_SOURCE(obj), name, errp) ||
        !visit_type_str(v, name, &str, errp)) {
        return;
    }
    g_free(*field);
    *field = str;
}

static char *get_format(Object *obj, Error **errp)
{
    ESP32P4ImageSource *s = ESP32P4_IMAGE_SOURCE(obj);
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(format_names); i++) {
        if (format_names[i].format == s->format) {
            return g_strdup(format_names[i].name);
        }
    }
    g_assert_not_reached();
}

static void set_format(Object *obj, const char *value, Error **errp)
{
    ESP32P4ImageSource *s = ESP32P4_IMAGE_SOURCE(obj);
    unsigned i;

    if (!settable(s, "format", errp)) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(format_names); i++) {
        if (!strcmp(format_names[i].name, value)) {
            s->format = format_names[i].format;
            return;
        }
    }
    error_setg(errp, "esp32p4-image-source: '%s' is not a format: raw8, "
               "raw10, raw12, rgb565, rgb565be, rgb888, bgr888, yuv422, "
               "yuyv, vyuy or yvyu", value);
}

static bool get_streaming(Object *obj, Error **errp)
{
    return ESP32P4_IMAGE_SOURCE(obj)->enabled;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
static void set_streaming(Object *obj, bool value, Error **errp)
{
    ESP32P4ImageSource *s = ESP32P4_IMAGE_SOURCE(obj);

    if (!s->complete) {
        s->enabled = value;
        return;
    }
    esp32p4_image_source_set_streaming(s, value);
}

static void image_source_init(Object *obj)
{
    ESP32P4ImageSource *s = ESP32P4_IMAGE_SOURCE(obj);
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(int_props); i++) {
        *(int64_t *)((char *)s + int_props[i].offset) = -1;
    }
    for (i = 0; i < ARRAY_SIZE(choice_props); i++) {
        *(int64_t *)((char *)s + choice_props[i].offset) = -1;
    }
    for (i = 0; i < ESP32P4_DVP_SIGNALS; i++) {
        s->pad[i] = -1;
    }
    s->bayer = ESP32P4_IMAGE_BAYER_RGGB;
    s->yuv_matrix = YUV_BT601;
    s->yuv_range = YUV_FULL;
    s->format = ESP32P4_IMAGE_RGB565;
    s->dvp_width = 8;
    s->lanes = 2;
    s->lane_rate = 0;
    s->vc = 0;
    s->clock_continuous = true;
    s->enabled = true;
    s->sinks = g_array_new(false, false, sizeof(ImageSink));
    s->events = g_array_new(false, false, sizeof(ImageEvent));
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, image_source_timer, s);
}

static void image_source_finalize(Object *obj)
{
    ESP32P4ImageSource *s = ESP32P4_IMAGE_SOURCE(obj);
    unsigned i;

    if (s->complete) {
        vmstate_unregister(NULL, &vmstate_image_source, s);
    }
    timer_del(&s->timer);
    if (s->line_source.notify) {
        notifier_remove(&s->line_source);
    }
    for (i = 0; i < PINS; i++) {
        qemu_free_irq(s->pin_irq[i]);
    }
    g_array_free(s->sinks, true);
    g_array_free(s->events, true);
    if (s->mapped) {
        g_mapped_file_unref(s->mapped);
    }
    g_free(s->still);
    g_free(s->rgb);
    g_free(s->line_buf);
    g_free(s->samples);
    g_free(s->bar_row[0]);
    g_free(s->bar_row[1]);
    g_free(s->file);
    g_free(s->image);
    g_free(s->data_pads);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
static void image_source_class_init(ObjectClass *oc, const void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);
    unsigned i;

    ucc->complete = image_source_complete;
    ucc->prepare_delete = image_source_prepare_delete;

    object_class_property_add_str(oc, "format", get_format, set_format);
    object_class_property_set_description(oc, "format",
        "The pixels: raw8, raw10, raw12, rgb565 (default), rgb565be, rgb888, "
        "bgr888, yuv422 (uyvy), yuyv, vyuy, yvyu");
    object_class_property_add_bool(oc, "streaming", get_streaming,
                                   set_streaming);
    object_class_property_set_description(oc, "streaming",
        "The sensor's software stream on (default on); settable at run time");
    for (i = 0; i < ARRAY_SIZE(int_props); i++) {
        object_class_property_add(oc, int_props[i].name, "int", get_int,
                                  set_int, NULL, (void *)&int_props[i]);
        object_class_property_set_description(oc, int_props[i].name,
                                              int_props[i].desc);
    }
    for (i = 0; i < ARRAY_SIZE(choice_props); i++) {
        object_class_property_add(oc, choice_props[i].name, "str",
                                  get_choice, set_choice, NULL,
                                  (void *)&choice_props[i]);
        object_class_property_set_description(oc, choice_props[i].name,
                                              choice_props[i].desc);
    }
    for (i = 0; i < ARRAY_SIZE(bool_props); i++) {
        object_class_property_add(oc, bool_props[i].name, "bool",
                                  get_bool_prop, set_bool_prop, NULL,
                                  (void *)&bool_props[i]);
        object_class_property_set_description(oc, bool_props[i].name,
                                              bool_props[i].desc);
    }
    for (i = 0; i < ARRAY_SIZE(str_props); i++) {
        object_class_property_add(oc, str_props[i].name, "str", get_str,
                                  set_str, NULL, (void *)&str_props[i]);
        object_class_property_set_description(oc, str_props[i].name,
                                              str_props[i].desc);
    }
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
/* [spec:nuos:req:emu.esp32p4.mipi] */
static const TypeInfo image_source_types[] = {
    {
        .name = TYPE_ESP32P4_IMAGE_SOURCE,
        .parent = TYPE_OBJECT,
        .instance_size = sizeof(ESP32P4ImageSource),
        .instance_init = image_source_init,
        .instance_finalize = image_source_finalize,
        .class_init = image_source_class_init,
        .interfaces = (const InterfaceInfo[]) {
            { TYPE_USER_CREATABLE },
            { }
        },
    },
};

DEFINE_TYPES(image_source_types)
