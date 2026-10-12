/*
 * QTest testcase for the ESP32-P4 camera sensor (esp32p4-image-source)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Each sensor's stream is taken by an esp32p4-image-probe, which writes
 * every callback a receiver would get, with the virtual time it ran at,
 * to a file; the tests read the file back and check it against what they
 * work out on their own: the times from the timing's definition, to the
 * ns, and the pixels from the pictures and patterns they describe, with
 * conversions, packings, the CSI-2 ECC and CRC of their own.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "qobject/qjson.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include <math.h>
#include <zlib.h>

#define NS_PER_S            1000000000ull

#define GPIO                0x500e0000
#define GPIO_OUT_W1TS       (GPIO + 0x008)
#define GPIO_OUT_W1TC       (GPIO + 0x00c)
#define GPIO_ENABLE_W1TS    (GPIO + 0x024)
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define OEN_SEL             (1u << 10)
#define SIG_GPIO_OUT        0x100
#define IO_MUX_GPIO(n)      (0x500e1000 + 0x004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (1u << 12)

/*
 * The probe's records
 * ===================
 */

typedef struct Rec {
    char type[8];
    bool on;
    uint32_t frame;
    uint32_t line;
    int active;
    /* line: HSYNC, HSYNC end, DE, DE end; packet: SoT, data, end, EoT */
    int64_t t[4];
    int64_t now;
    int64_t epoch;
    uint64_t clock;
    uint64_t pclk;
    uint32_t count;
    unsigned vc, dt, wc, ecc, crc;
    /* line: the samples; packet: the payload */
    uint16_t *samples;
    uint8_t *payload;
    size_t len;
} Rec;

typedef struct Cam {
    char *dir;
    QTestState *qts;
    unsigned nprobes;
} Cam;

static const char *kv(char **tok, const char *key)
{
    size_t n = strlen(key);

    for (; *tok; tok++) {
        if (!strncmp(*tok, key, n) && (*tok)[n] == '=') {
            return *tok + n + 1;
        }
    }
    return NULL;
}

static int64_t kv_int(char **tok, const char *key)
{
    const char *v = kv(tok, key);

    g_assert(v);
    return g_ascii_strtoll(v, NULL, 0);
}

static void kv_pair(char **tok, const char *key, int64_t *a, int64_t *b)
{
    const char *v = kv(tok, key);
    char *end;

    g_assert(v);
    *a = g_ascii_strtoll(v, &end, 10);
    g_assert(*end == ',');
    *b = g_ascii_strtoll(end + 1, NULL, 10);
}

static void free_recs(gpointer p)
{
    Rec *r = p;

    g_free(r->samples);
    g_free(r->payload);
}

/* The records probe @n has written. */
static GArray *read_log(Cam *c, unsigned n)
{
    g_autofree char *path = g_strdup_printf("%s/probe%u.log", c->dir, n);
    g_autofree char *text = NULL;
    g_auto(GStrv) lines = NULL;
    GArray *recs = g_array_new(false, true, sizeof(Rec));
    int i;

    g_array_set_clear_func(recs, free_recs);
    g_assert(g_file_get_contents(path, &text, NULL, NULL));
    lines = g_strsplit(text, "\n", -1);
    for (i = 0; lines[i]; i++) {
        g_auto(GStrv) tok = NULL;
        Rec r = { 0 };
        const char *d;

        if (!*lines[i]) {
            continue;
        }
        tok = g_strsplit(lines[i], " ", -1);
        g_strlcpy(r.type, tok[0], sizeof(r.type));
        if (!strcmp(r.type, "attach")) {
            r.active = kv_int(tok, "running");
            r.now = kv_int(tok, "now");
        } else if (!strcmp(r.type, "stream")) {
            r.on = !strcmp(tok[1], "on");
            r.t[0] = kv_int(tok, "t");
            r.now = kv_int(tok, "now");
        } else if (!strcmp(r.type, "vsync")) {
            r.frame = kv_int(tok, "frame");
            r.active = kv_int(tok, "active");
            r.t[0] = kv_int(tok, "t");
            r.now = kv_int(tok, "now");
        } else if (!strcmp(r.type, "line")) {
            r.frame = kv_int(tok, "frame");
            r.line = kv_int(tok, "line");
            kv_pair(tok, "hsync", &r.t[0], &r.t[1]);
            kv_pair(tok, "de", &r.t[2], &r.t[3]);
            r.epoch = kv_int(tok, "epoch");
            r.clock = kv_int(tok, "clock");
            r.pclk = kv_int(tok, "pclk");
            r.count = kv_int(tok, "count");
            r.now = kv_int(tok, "now");
            d = kv(tok, "data");
            if (d) {
                uint32_t k;

                g_assert_cmpuint(strlen(d), ==, 4 * r.count);
                r.samples = g_new(uint16_t, r.count);
                for (k = 0; k < r.count; k++) {
                    char hex[5] = { d[4 * k], d[4 * k + 1], d[4 * k + 2],
                                    d[4 * k + 3], 0 };

                    r.samples[k] = g_ascii_strtoull(hex, NULL, 16);
                }
            }
        } else if (!strcmp(r.type, "packet")) {
            r.frame = kv_int(tok, "frame");
            r.line = kv_int(tok, "line");
            r.vc = kv_int(tok, "vc");
            r.dt = kv_int(tok, "dt");
            r.wc = kv_int(tok, "wc");
            r.ecc = kv_int(tok, "ecc");
            r.crc = kv_int(tok, "crc");
            r.t[0] = kv_int(tok, "sot");
            r.t[1] = kv_int(tok, "data");
            r.t[2] = kv_int(tok, "end");
            r.t[3] = kv_int(tok, "eot");
            r.now = kv_int(tok, "now");
            d = kv(tok, "payload");
            if (d) {
                size_t k;

                r.len = strlen(d) / 2;
                g_assert_cmpuint(r.len, ==, r.wc);
                r.payload = g_malloc(r.len);
                for (k = 0; k < r.len; k++) {
                    char hex[3] = { d[2 * k], d[2 * k + 1], 0 };

                    r.payload[k] = g_ascii_strtoull(hex, NULL, 16);
                }
            }
        } else {
            g_assert_not_reached();
        }
        g_array_append_val(recs, r);
    }
    return recs;
}

static Rec *rec(GArray *recs, guint i)
{
    return &g_array_index(recs, Rec, i);
}

/* The @n-th record of @type at or after @from, or NULL. */
static Rec *find(GArray *recs, const char *type, guint from, guint *at)
{
    guint i;

    for (i = from; i < recs->len; i++) {
        if (!strcmp(rec(recs, i)->type, type)) {
            if (at) {
                *at = i;
            }
            return rec(recs, i);
        }
    }
    return NULL;
}

/* The line record of frame @f, line @l. */
static Rec *find_line(GArray *recs, uint32_t f, uint32_t l)
{
    guint i;

    for (i = 0; i < recs->len; i++) {
        Rec *r = rec(recs, i);

        if (!strcmp(r->type, "line") && r->frame == f && r->line == l) {
            return r;
        }
    }
    return NULL;
}

/* The long packet (a payload) of frame @f, line @l. */
static Rec *find_long(GArray *recs, uint32_t f, uint32_t l)
{
    guint i;

    for (i = 0; i < recs->len; i++) {
        Rec *r = rec(recs, i);

        if (!strcmp(r->type, "packet") && r->dt >= 0x10 && r->frame == f &&
            r->line == l) {
            return r;
        }
    }
    return NULL;
}

/*
 * QEMU with sensors and their probes: @srcs[i] the properties of sensor
 * cam<i>, @ports[i] the port its probe takes.
 */
static void cam_start(Cam *c, const char *machine, unsigned n,
                      const char *const *srcs, const char *const *ports)
{
    GString *args = g_string_new("-M esp32p4,strap-mode=0");
    unsigned i;

    if (!c->dir) {
        c->dir = g_dir_make_tmp("esp32p4-imgsrc-XXXXXX", NULL);
        g_assert(c->dir);
    }
    if (machine) {
        g_string_append_printf(args, ",%s", machine);
    }
    for (i = 0; i < n; i++) {
        g_string_append_printf(args, " -object esp32p4-image-source,"
                               "id=cam%u,%s", i, srcs[i]);
        if (ports[i]) {
            g_string_append_printf(args, " -object esp32p4-image-probe,"
                                   "id=probe%u,source=cam%u,port=%s,"
                                   "log=%s/probe%u.log", i, i, ports[i],
                                   c->dir, i);
        }
    }
    c->nprobes = n;
    c->qts = qtest_init(args->str);
    g_string_free(args, true);
}

static void cam_start1(Cam *c, const char *src, const char *port)
{
    cam_start(c, NULL, 1, &src, &port);
}

static void cam_stop(Cam *c)
{
    g_autoptr(GDir) d = NULL;
    const char *name;

    qtest_quit(c->qts);
    c->qts = NULL;
    d = g_dir_open(c->dir, 0, NULL);
    while ((name = g_dir_read_name(d))) {
        g_autofree char *p = g_build_filename(c->dir, name, NULL);

        unlink(p);
    }
    rmdir(c->dir);
    g_free(c->dir);
    c->dir = NULL;
}

static char *cam_file(Cam *c, const char *name, const void *data, size_t len)
{
    char *path;

    if (!c->dir) {
        c->dir = g_dir_make_tmp("esp32p4-imgsrc-XXXXXX", NULL);
        g_assert(c->dir);
    }
    path = g_build_filename(c->dir, name, NULL);
    g_assert(g_file_set_contents(path, data, len, NULL));
    return path;
}

static void qom_set_bool(QTestState *qts, const char *path, const char *prop,
                         bool v)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': %s, 'property': %s, 'value': %i } }",
                         path, prop, v);

    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
}

static int64_t qom_get_int(QTestState *qts, const char *path,
                           const char *prop)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': %s } }", path, prop);
    int64_t v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_int(r, "return");
    qobject_unref(r);
    return v;
}

/*
 * Exact times, worked out here
 * ============================
 */

/* The first whole ns by which @clocks cycles at @hz have gone by. */
static int64_t ceil_ns(uint64_t clocks, uint64_t hz)
{
    unsigned __int128 n = (unsigned __int128)clocks * NS_PER_S;

    return (n + hz - 1) / hz;
}

/* @ns + @ui unit intervals at @bps after @from, to the next whole ns. */
static int64_t dphy(int64_t from, uint64_t bps, uint64_t ns, uint64_t ui)
{
    unsigned __int128 n = (unsigned __int128)ns * bps +
                          (unsigned __int128)ui * NS_PER_S;

    return from + (int64_t)((n + bps - 1) / bps);
}

/*
 * A burst of @bytes on @lanes at @bps from @sot (D-PHY 1.1, the sensor's
 * T_LPX 50 ns, T_HS-PREPARE 50 ns + 5 UI, T_HS-ZERO 100 ns + 6 UI, the
 * sync byte, the bytes, T_HS-TRAIL 60 ns + 4 UI at these rates).
 */
static void burst(int64_t sot, uint64_t bps, unsigned lanes, unsigned bytes,
                  int64_t t[4])
{
    uint64_t ui = 19 + 8 * DIV_ROUND_UP(bytes, lanes);

    g_assert(60 * bps >= 4 * NS_PER_S);
    t[0] = sot;
    t[1] = dphy(sot, bps, 200, 19);
    t[2] = dphy(sot, bps, 200, ui);
    t[3] = dphy(sot, bps, 260, ui + 4);
}

/*
 * CSI-2's ECC and CRC, worked out bit by bit
 * ==========================================
 */

static uint8_t ecc_ref(uint32_t h)
{
    /* Each parity bit's header bits, from the CSI-2 specification */
    static const uint8_t bits[6][14] = {
        { 0, 1, 2, 4, 5, 7, 10, 11, 13, 16, 20, 21, 22, 23 },
        { 0, 1, 3, 4, 6, 8, 10, 12, 14, 17, 20, 21, 22, 23 },
        { 0, 2, 3, 5, 6, 9, 11, 12, 15, 18, 20, 21, 22, 255 },
        { 1, 2, 3, 7, 8, 9, 13, 14, 15, 19, 20, 21, 23, 255 },
        { 4, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20, 22, 23, 255 },
        { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 255 },
    };
    uint8_t ecc = 0;
    int p, i;

    for (p = 0; p < 6; p++) {
        unsigned v = 0;

        for (i = 0; i < 14 && bits[p][i] != 255; i++) {
            v ^= (h >> bits[p][i]) & 1;
        }
        ecc |= v << p;
    }
    return ecc;
}

static uint16_t crc_ref(const uint8_t *d, size_t len)
{
    uint16_t crc = 0xffff;
    size_t i;
    int b;

    for (i = 0; i < len; i++) {
        for (b = 0; b < 8; b++) {
            unsigned in = ((d[i] >> b) ^ crc) & 1;

            crc >>= 1;
            if (in) {
                crc ^= 0x8408;
            }
        }
    }
    return crc;
}

/* The CRC against the CSI-2 specification's own examples. */
static void test_crc_reference(void)
{
    static const uint8_t a[24] = {
        0xff, 0x00, 0x00, 0x02, 0xb9, 0xdc, 0xf3, 0x72, 0xbb, 0xd4, 0xb8, 0x5a,
        0xc8, 0x75, 0xc2, 0x7c, 0x81, 0xf8, 0x05, 0xdf, 0xff, 0x00, 0x00, 0x01,
    };
    static const uint8_t b[24] = {
        0xff, 0x00, 0x00, 0x00, 0x1e, 0xf0, 0x1e, 0xc7, 0x4f, 0x82, 0x78, 0xc5,
        0x82, 0xe0, 0x8c, 0x70, 0xd2, 0x3c, 0x78, 0xe9, 0xff, 0x00, 0x00, 0x01,
    };

    g_assert_cmphex(crc_ref(a, sizeof(a)), ==, 0x00f0);
    g_assert_cmphex(crc_ref(b, sizeof(b)), ==, 0xe569);
}

/*
 * Pixels, worked out here
 * =======================
 */

typedef struct Px {
    uint16_t r, g, b;
} Px;

static uint32_t to_bits(uint32_t v16, unsigned bits)
{
    return (v16 * ((1u << bits) - 1) + 32767) / 65535;
}

static uint16_t rgb565(Px p)
{
    return to_bits(p.r, 5) << 11 | to_bits(p.g, 6) << 5 | to_bits(p.b, 5);
}

/* The colour at (@x, @y) of a Bayer pattern named like "rggb". */
static uint16_t bayer_pick(const char *order, Px p, uint32_t x, uint32_t y)
{
    char c = order[(y & 1) * 2 + (x & 1)];

    return c == 'r' ? p.r : c == 'g' ? p.g : p.b;
}

static void pack_raw(unsigned bits, const uint32_t *v, uint32_t n,
                     uint8_t *out)
{
    uint32_t i;

    for (i = 0; i < n; i++) {
        if (bits == 8) {
            out[i] = v[i];
        } else if (bits == 10) {
            out[i / 4 * 5 + i % 4] = v[i] >> 2;
            if (i % 4 == 0) {
                out[i / 4 * 5 + 4] = 0;
            }
            out[i / 4 * 5 + 4] |= (v[i] & 3) << (2 * (i % 4));
        } else {
            out[i / 2 * 3 + i % 2] = v[i] >> 4;
            if (i % 2 == 0) {
                out[i / 2 * 3 + 2] = 0;
            }
            out[i / 2 * 3 + 2] |= (v[i] & 15) << (4 * (i % 2));
        }
    }
}

/* Y, Cb, Cr of an 8-bit pixel, by the standards' formulas */
static void yuv_ref(Px p, bool bt709, bool limited, double *y, double *u,
                    double *v)
{
    double kr = bt709 ? 0.2126 : 0.299, kb = bt709 ? 0.0722 : 0.114;
    double r = to_bits(p.r, 8), g = to_bits(p.g, 8), b = to_bits(p.b, 8);
    double luma = kr * r + (1 - kr - kb) * g + kb * b;
    double cb = (b - luma) / (2 * (1 - kb)), cr = (r - luma) / (2 * (1 - kr));

    if (limited) {
        *y = 16 + luma * 219 / 255;
        *u = 128 + cb * 224 / 255;
        *v = 128 + cr * 224 / 255;
    } else {
        *y = luma;
        *u = 128 + cb;
        *v = 128 + cr;
    }
}

static int clamp_round(double v)
{
    v = floor(v + 0.5);
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/*
 * A line of @w pixels in @format's layout: exact, except for YUV, whose
 * samples may each be 1 off the standards' formula (*@tolerance set).
 */
static void expect_line(const char *format, const char *bayer, bool bt709,
                        bool limited, const Px *px, uint32_t w, uint32_t y,
                        uint8_t *out, bool *tolerance)
{
    uint32_t x, v[8192];

    *tolerance = false;
    if (!strcmp(format, "raw8") || !strcmp(format, "raw10") ||
        !strcmp(format, "raw12")) {
        unsigned bits = atoi(format + 3);

        for (x = 0; x < w; x++) {
            v[x] = to_bits(bayer_pick(bayer, px[x], x, y), bits);
        }
        pack_raw(bits, v, w, out);
    } else if (!strcmp(format, "rgb565")) {
        for (x = 0; x < w; x++) {
            stw_le_p(out + 2 * x, rgb565(px[x]));
        }
    } else if (!strcmp(format, "rgb565be")) {
        for (x = 0; x < w; x++) {
            stw_be_p(out + 2 * x, rgb565(px[x]));
        }
    } else if (!strcmp(format, "rgb888") || !strcmp(format, "bgr888")) {
        bool bgr = format[0] == 'b';

        for (x = 0; x < w; x++) {
            out[3 * x] = to_bits(bgr ? px[x].b : px[x].r, 8);
            out[3 * x + 1] = to_bits(px[x].g, 8);
            out[3 * x + 2] = to_bits(bgr ? px[x].r : px[x].b, 8);
        }
    } else {
        /* Where each of Y0, U, Y1, V goes in a group of four bytes */
        const char *order = !strcmp(format, "yuv422") ? "uyvy" : format;

        *tolerance = true;
        for (x = 0; x + 1 < w; x += 2) {
            double y0, y1, u0, u1, v0, v1;
            int s[4], i, yi = 0;

            yuv_ref(px[x], bt709, limited, &y0, &u0, &v0);
            yuv_ref(px[x + 1], bt709, limited, &y1, &u1, &v1);
            for (i = 0; i < 4; i++) {
                switch (order[i]) {
                case 'y':
                    s[i] = clamp_round(yi++ ? y1 : y0);
                    break;
                case 'u':
                    s[i] = (clamp_round(u0) + clamp_round(u1) + 1) / 2;
                    break;
                default:
                    s[i] = (clamp_round(v0) + clamp_round(v1) + 1) / 2;
                    break;
                }
                out[2 * x + i] = s[i];
            }
        }
    }
}

static uint32_t line_len(const char *format, uint32_t w)
{
    if (!strncmp(format, "raw", 3)) {
        return w * atoi(format + 3) / 8;
    }
    if (!strcmp(format, "rgb888") || !strcmp(format, "bgr888")) {
        return w * 3;
    }
    return w * 2;
}

static void assert_bytes(const uint8_t *got, const uint8_t *want, size_t n,
                         bool tolerance)
{
    size_t i;

    for (i = 0; i < n; i++) {
        if (tolerance) {
            g_assert_cmpint(abs(got[i] - want[i]), <=, 1);
        } else if (got[i] != want[i]) {
            g_assert_cmphex(got[i], ==, want[i]);
        }
    }
}

/* A deterministic pseudo-random sequence */
static uint32_t prng(uint32_t *state)
{
    *state = *state * 1103515245 + 12345;
    return *state >> 8;
}

/*
 * The test patterns, worked out here
 * ==================================
 */

static Px grey(uint16_t v)
{
    return (Px) { v, v, v };
}

/* Pixel (@x, @y) of frame @f of the bars, with the counter. */
static Px bars_px(uint32_t w, uint32_t cell, uint32_t f, uint32_t x,
                  uint32_t y)
{
    static const uint8_t col[8][3] = {
        { 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 0, 255, 0 },
        { 255, 0, 255 }, { 255, 0, 0 }, { 0, 0, 255 }, { 0, 0, 0 },
    };
    uint32_t step = MAX(w / 64, 1), bar;

    if (y < cell && x / cell < 16 && (x / cell + 1) * cell <= w) {
        return grey(f & (0x8000 >> (x / cell)) ? 65535 : 0);
    }
    bar = (uint32_t)(((x + (uint64_t)f * step) % w) * 8 / w);
    return (Px) { col[bar][0] * 257, col[bar][1] * 257, col[bar][2] * 257 };
}

static Px checker_px(uint32_t w, uint32_t cell, uint32_t f, uint32_t x,
                     uint32_t y)
{
    int64_t u = ((int64_t)x - f) % (2 * cell), v = ((int64_t)y - f) %
                                                     (2 * cell);

    if (y < cell && x / cell < 16 && (x / cell + 1) * cell <= w) {
        return grey(f & (0x8000 >> (x / cell)) ? 65535 : 0);
    }
    u += u < 0 ? 2 * cell : 0;
    v += v < 0 ? 2 * cell : 0;
    return grey((u < cell) == (v < cell) ? 65535 : 0);
}

/*
 * DVP timing
 * ==========
 */

/*
 * 64x6 RGB565 a byte a PCLK: 2 PCLKs a pixel. hts 74 and vts 11 at 30
 * fps: a pixel clock of 24420 Hz, PCLK 48840 Hz, whose period is no whole
 * number of ns. Every edge is checked against its definition, and every
 * callback runs at its edge's time.
 */
static void test_dvp_timing(void)
{
    const uint64_t pclk = 30 * 74 * 11 * 2;
    const uint32_t line_clocks = 74 * 2, vts = 11, active_line = 3;
    Cam c = { 0 };
    g_autoptr(GArray) recs = NULL;
    uint32_t f, l, vs = 0, lines = 0;
    guint i;

    cam_start1(&c, "format=rgb565,width=64,height=6,cell=4,hblank=10,"
               "vblank=5,hsync=2,hbp=3,vsync=1,vbp=2", "dvp");
    qtest_clock_step(c.qts, 100 * 1000 * 1000);
    recs = read_log(&c, 0);

    g_assert(find(recs, "attach", 0, NULL)->active);
    for (i = 0; i < recs->len; i++) {
        Rec *r = rec(recs, i);

        if (i) {
            g_assert_cmpint(r->now, >=, rec(recs, i - 1)->now);
        }
        if (!strcmp(r->type, "vsync")) {
            uint64_t at = (uint64_t)r->frame * vts + (r->active ? 0 : 1);

            g_assert_cmpint(r->t[0], ==, ceil_ns(at * line_clocks, pclk));
            g_assert_cmpint(r->now, ==, r->t[0]);
            vs++;
        } else if (!strcmp(r->type, "line")) {
            uint64_t base = ((uint64_t)r->frame * vts + active_line +
                             r->line) * line_clocks;

            g_assert_cmpint(r->epoch, ==, 0);
            g_assert_cmpuint(r->pclk, ==, pclk);
            g_assert_cmpuint(r->count, ==, 128);
            g_assert_cmpuint(r->clock, ==, base + 10);
            g_assert_cmpint(r->t[0], ==, ceil_ns(base, pclk));
            g_assert_cmpint(r->t[1], ==, ceil_ns(base + 4, pclk));
            g_assert_cmpint(r->t[2], ==, ceil_ns(base + 10, pclk));
            g_assert_cmpint(r->t[3], ==, ceil_ns(base + 10 + 128, pclk));
            g_assert_cmpint(r->now, ==, r->t[3]);
            lines++;
        }
    }
    /* Frames 0 to 2 whole, and frame 3's VSYNC at exactly 100 ms */
    g_assert_cmpuint(lines, ==, 3 * 6);
    g_assert_cmpuint(vs, ==, 3 * 2 + 1);
    for (f = 0; f < 3; f++) {
        for (l = 0; l < 6; l++) {
            g_assert(find_line(recs, f, l));
        }
    }
    cam_stop(&c);
}

/*
 * The bars and the counter on DVP: RGB565 a byte a PCLK, low byte first,
 * and two bytes a PCLK (the pixel on D[15:0]); a frame later the bars
 * have moved a pixel left and the counter counts.
 */
static void test_dvp_bars(void)
{
    const char *srcs[] = {
        "format=rgb565,width=64,height=6,cell=4",
        "format=rgb565,width=64,height=6,cell=4,dvp-width=16",
        "format=yuyv,width=64,height=6,cell=2,yuv-range=limited",
    };
    const char *ports[] = { "dvp", "dvp", "dvp" };
    Cam c = { 0 };
    unsigned n;

    cam_start(&c, NULL, 3, srcs, ports);
    qtest_clock_step(c.qts, 200 * 1000 * 1000);
    for (n = 0; n < 3; n++) {
        g_autoptr(GArray) recs = read_log(&c, n);
        uint32_t f, y, x;

        for (f = 0; f < 6; f++) {
            for (y = 0; y < 6; y++) {
                Rec *r = find_line(recs, f, y);
                uint8_t bytes[128];
                Px px[64];
                bool tol;

                g_assert(r);
                for (x = 0; x < 64; x++) {
                    px[x] = bars_px(64, n == 2 ? 2 : 4, f, x, y);
                }
                if (n == 1) {
                    g_assert_cmpuint(r->count, ==, 64);
                    for (x = 0; x < 64; x++) {
                        g_assert_cmphex(r->samples[x], ==, rgb565(px[x]));
                    }
                    continue;
                }
                g_assert_cmpuint(r->count, ==, 128);
                expect_line(n ? "yuyv" : "rgb565", NULL, false, n == 2, px,
                            64, y, bytes, &tol);
                for (x = 0; x < 128; x++) {
                    g_assert_cmphex(r->samples[x], <=, 0xff);
                    if (tol) {
                        g_assert_cmpint(abs(r->samples[x] - bytes[x]), <=, 1);
                    } else {
                        g_assert_cmphex(r->samples[x], ==, bytes[x]);
                    }
                }
            }
        }
    }
    cam_stop(&c);
}

/*
 * MIPI CSI
 * ========
 */

/*
 * 64x4 RAW10, hts 80 and vts 8 at 30 fps, on two lanes at 100 Mbit/s and
 * then one lane at 150 Mbit/s with line start and end packets: each
 * packet's header, ECC, CRC and payload, and each burst's times.
 */
static void test_csi_packets(void)
{
    const char *srcs[] = {
        "format=raw10,bayer=gbrg,width=64,height=4,hblank=16,vblank=4,"
        "vsync=1,vbp=2,cell=4,lanes=2,lane-rate=100000000",
        "format=raw10,bayer=gbrg,width=64,height=4,hblank=16,vblank=4,"
        "vsync=1,vbp=2,cell=4,lanes=1,lane-rate=150000000,line-sync=on,vc=2",
    };
    const char *ports[] = { "csi", "csi" };
    const uint64_t pixel_hz = 30 * 80 * 8;
    Cam c = { 0 };
    unsigned n;

    cam_start(&c, NULL, 2, srcs, ports);
    qtest_clock_step(c.qts, 100 * 1000 * 1000);
    for (n = 0; n < 2; n++) {
        g_autoptr(GArray) recs = read_log(&c, n);
        uint64_t bps = n ? 150000000 : 100000000;
        unsigned lanes = n ? 1 : 2, vc = n ? 2 : 0;
        guint i = 0;
        uint32_t f, l;

        for (f = 0; f < 3; f++) {
            int64_t t[4], sot;
            Rec *r;

            /* FS at the frame's start, its data field the frame number */
            r = find(recs, "packet", i, &i);
            g_assert(r);
            g_assert_cmphex(r->dt, ==, 0x00);
            g_assert_cmpuint(r->wc, ==, f + 1);
            g_assert_cmpuint(r->vc, ==, vc);
            g_assert_cmphex(r->ecc, ==, ecc_ref(vc << 6 | (f + 1) << 8));
            burst(ceil_ns((uint64_t)f * 8 * 80, pixel_hz), bps, lanes, 4, t);
            g_assert_cmpmem(r->t, sizeof(t), t, sizeof(t));
            g_assert_cmpint(r->now, ==, r->t[2]);
            i++;
            for (l = 0; l < 4; l++) {
                uint8_t want[80];
                uint32_t v[64], x;

                sot = ceil_ns(((uint64_t)f * 8 + 3 + l) * 80, pixel_hz);
                if (n) {
                    r = find(recs, "packet", i, &i);
                    g_assert_cmphex(r->dt, ==, 0x02);
                    g_assert_cmpuint(r->wc, ==, l + 1);
                    g_assert_cmphex(r->ecc, ==,
                                    ecc_ref(vc << 6 | 0x02 | (l + 1) << 8));
                    burst(sot, bps, lanes, 4, t);
                    g_assert_cmpmem(r->t, sizeof(t), t, sizeof(t));
                    sot = t[3] + 100;
                    i++;
                }
                r = find(recs, "packet", i, &i);
                g_assert_cmphex(r->dt, ==, 0x2b);
                g_assert_cmpuint(r->line, ==, l);
                g_assert_cmpuint(r->wc, ==, 80);
                g_assert_cmphex(r->ecc, ==,
                                ecc_ref(vc << 6 | 0x2b | 80 << 8));
                burst(sot, bps, lanes, 86, t);
                g_assert_cmpmem(r->t, sizeof(t), t, sizeof(t));
                g_assert_cmpint(r->now, ==, r->t[2]);
                for (x = 0; x < 64; x++) {
                    v[x] = to_bits(bayer_pick("gbrg",
                                              bars_px(64, 4, f, x, l), x, l),
                                   10);
                }
                pack_raw(10, v, 64, want);
                g_assert_cmpmem(r->payload, r->len, want, sizeof(want));
                g_assert_cmphex(r->crc, ==, crc_ref(r->payload, r->len));
                sot = t[3] + 100;
                i++;
                if (n) {
                    r = find(recs, "packet", i, &i);
                    g_assert_cmphex(r->dt, ==, 0x03);
                    g_assert_cmpuint(r->wc, ==, l + 1);
                    burst(sot, bps, lanes, 4, t);
                    g_assert_cmpmem(r->t, sizeof(t), t, sizeof(t));
                    sot = t[3] + 100;
                    i++;
                }
            }
            /* FE after the last line's bursts */
            r = find(recs, "packet", i, &i);
            g_assert_cmphex(r->dt, ==, 0x01);
            g_assert_cmpuint(r->wc, ==, f + 1);
            burst(sot, bps, lanes, 4, t);
            g_assert_cmpmem(r->t, sizeof(t), t, sizeof(t));
            i++;
        }
    }
    cam_stop(&c);
}

static bool in_burst(QTestState *qts)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/objects/probe0', "
                         "'property': 'in-burst' } }");
    bool v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_bool(r, "return");
    qobject_unref(r);
    return v;
}

/*
 * The data lanes leave LP-11 at each burst's SoT and are back at its EoT:
 * what a receiver's D-PHY stop state shows, even before the packet,
 * which reaches it only as it ends.
 */
static void test_csi_in_burst(void)
{
    const uint64_t bps = 100000000, pixel_hz = 30 * 80 * 8;
    Cam c = { 0 };
    int64_t fs[4], ln[4];

    cam_start1(&c, "format=raw8,width=64,height=4,hblank=16,vblank=4,"
               "vsync=1,vbp=2,lanes=2,lane-rate=100000000", "csi");
    burst(0, bps, 2, 4, fs);
    burst(ceil_ns(3 * 80, pixel_hz), bps, 2, 70, ln);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, fs[2]);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, fs[3] - 1);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, fs[3]);
    g_assert(!in_burst(c.qts));
    qtest_clock_set(c.qts, ln[0] - 1);
    g_assert(!in_burst(c.qts));
    qtest_clock_set(c.qts, ln[0]);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, ln[2] - 1);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, ln[2]);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, ln[3] - 1);
    g_assert(in_burst(c.qts));
    qtest_clock_set(c.qts, ln[3]);
    g_assert(!in_burst(c.qts));
    cam_stop(&c);
}

/*
 * The lane rate a mode gets with none given: the lowest multiple of
 * 10 Mbit/s from 80 Mbit/s at which a line's burst ends, with the LP-11
 * time after it, within 80% of the shortest line.
 */
static uint64_t auto_rate(uint64_t pixel_hz, uint32_t hts, uint32_t bytes,
                          unsigned lanes)
{
    int64_t line_ns = ceil_ns(hts, pixel_hz) - 1, t[4];
    uint64_t bps;

    for (bps = 80000000;; bps += 10000000) {
        burst(0, bps, lanes, bytes + 6, t);
        if (t[3] + 100 <= line_ns * 80 / 100) {
            return bps;
        }
    }
}

static void test_csi_lane_rate(void)
{
    const char *srcs[] = {
        "format=rgb565,width=640,height=480",
        "format=raw10,width=1920,height=1080,lanes=2",
        "format=raw8,width=64,height=8",
    };
    const char *ports[] = { "csi", "csi", "csi" };
    Cam c = { 0 };

    cam_start(&c, NULL, 3, srcs, ports);
    /* 640x480: hts 800, vts 510 by default */
    g_assert_cmpint(qom_get_int(c.qts, "/objects/cam0", "lane-rate"), ==,
                    auto_rate(30 * 800 * 510, 800, 1280, 2));
    /* 1920x1080: hts 2400, vts 1148 */
    g_assert_cmpint(qom_get_int(c.qts, "/objects/cam1", "lane-rate"), ==,
                    auto_rate(30ull * 2400 * 1148, 2400, 2400, 2));
    g_assert_cmpint(qom_get_int(c.qts, "/objects/cam2", "lane-rate"), ==,
                    80000000);
    g_assert_cmpint(qom_get_int(c.qts, "/objects/cam1", "hts"), ==, 2400);
    g_assert_cmpint(qom_get_int(c.qts, "/objects/cam1", "vts"), ==, 1148);
    g_assert_cmpint(qom_get_int(c.qts, "/objects/cam1", "pixel-clock"), ==,
                    30ull * 2400 * 1148);
    cam_stop(&c);
}

/*
 * Still pictures
 * ==============
 */

static GByteArray *ppm(uint32_t w, uint32_t h, unsigned maxval, bool grey,
                       const Px *px)
{
    GByteArray *b = g_byte_array_new();
    g_autofree char *hdr = g_strdup_printf("P%c\n# a test picture\n%u %u\n"
                                           "%u\n", grey ? '5' : '6', w, h,
                                           maxval);
    uint32_t i;

    g_byte_array_append(b, (uint8_t *)hdr, strlen(hdr));
    for (i = 0; i < w * h; i++) {
        uint16_t s[3] = { px[i].r, px[i].g, px[i].b };
        unsigned c;

        for (c = 0; c < (grey ? 1u : 3u); c++) {
            uint32_t v = (s[c] * maxval + 32767) / 65535;
            uint8_t be[2] = { v >> 8, v };

            if (maxval > 255) {
                g_byte_array_append(b, be, 2);
            } else {
                g_byte_array_append(b, be + 1, 1);
            }
        }
    }
    return b;
}

/*
 * Through every format: a picture of the sensor's size in random colours
 * reaches the CSI payloads converted as the formats say.
 */
static void test_still_formats(void)
{
    static const char *const fmts[][4] = {
        /* format, bayer, matrix, range */
        { "rgb888" }, { "bgr888" }, { "rgb565" }, { "rgb565be" },
        { "raw8", "rggb" }, { "raw8", "grbg" }, { "raw10", "gbrg" },
        { "raw12", "bggr" }, { "yuv422", NULL, "bt601", "full" },
        { "yuyv", NULL, "bt601", "limited" },
        { "vyuy", NULL, "bt709", "full" },
        { "yvyu", NULL, "bt709", "limited" },
    };
    const unsigned n = ARRAY_SIZE(fmts);
    const char *srcs[ARRAY_SIZE(fmts)], *ports[ARRAY_SIZE(fmts)];
    char *srcbuf[ARRAY_SIZE(fmts)];
    Px px[8 * 4];
    Cam c = { 0 };
    uint32_t seed = 7, i, y;
    g_autoptr(GByteArray) pic = NULL;
    g_autofree char *path = NULL;

    for (i = 0; i < 8 * 4; i++) {
        px[i] = (Px) { (prng(&seed) & 0xff) * 257, (prng(&seed) & 0xff) * 257,
                       (prng(&seed) & 0xff) * 257 };
    }
    pic = ppm(8, 4, 255, false, px);
    path = cam_file(&c, "pic.ppm", pic->data, pic->len);
    for (i = 0; i < n; i++) {
        srcbuf[i] = g_strdup_printf("image=%s,format=%s%s%s%s%s%s%s", path,
                                    fmts[i][0],
                                    fmts[i][1] ? ",bayer=" : "",
                                    fmts[i][1] ? fmts[i][1] : "",
                                    fmts[i][2] ? ",yuv-matrix=" : "",
                                    fmts[i][2] ? fmts[i][2] : "",
                                    fmts[i][3] ? ",yuv-range=" : "",
                                    fmts[i][3] ? fmts[i][3] : "");
        srcs[i] = srcbuf[i];
        ports[i] = "csi";
    }
    cam_start(&c, NULL, n, srcs, ports);
    qtest_clock_step(c.qts, 70 * 1000 * 1000);
    for (i = 0; i < n; i++) {
        g_autoptr(GArray) recs = read_log(&c, i);

        for (y = 0; y < 4; y++) {
            Rec *r = find_long(recs, 1, y);
            uint8_t want[8 * 3];
            bool tol;

            g_assert(r);
            expect_line(fmts[i][0], fmts[i][1], fmts[i][2] &&
                        !strcmp(fmts[i][2], "bt709"), fmts[i][3] &&
                        !strcmp(fmts[i][3], "limited"), px + 8 * y, 8, y,
                        want, &tol);
            g_assert_cmpuint(r->len, ==, line_len(fmts[i][0], 8));
            assert_bytes(r->payload, want, r->len, tol);
        }
        g_free(srcbuf[i]);
    }
    cam_stop(&c);
}

/*
 * 16-bit and grey PNM: a 16-bit PPM keeps RAW12's precision, and a PGM
 * is grey.
 */
static void test_still_pnm(void)
{
    const char *srcs[2], *ports[2] = { "csi", "csi" };
    g_autofree char *s0 = NULL, *s1 = NULL, *p0 = NULL, *p1 = NULL;
    g_autoptr(GByteArray) a = NULL, b = NULL;
    Px px[6 * 2];
    Cam c = { 0 };
    uint32_t seed = 3, i, y;

    for (i = 0; i < 12; i++) {
        px[i] = (Px) { prng(&seed), prng(&seed), prng(&seed) };
    }
    a = ppm(6, 2, 65535, false, px);
    p0 = cam_file(&c, "a.ppm", a->data, a->len);
    for (i = 0; i < 12; i++) {
        px[i] = grey((prng(&seed) % 1001) * 65535 / 1000);
    }
    b = ppm(6, 2, 1000, true, px);
    p1 = cam_file(&c, "b.pgm", b->data, b->len);
    s0 = g_strdup_printf("image=%s,format=raw12,bayer=rggb", p0);
    s1 = g_strdup_printf("image=%s,format=rgb888", p1);
    srcs[0] = s0;
    srcs[1] = s1;
    cam_start(&c, NULL, 2, srcs, ports);
    qtest_clock_step(c.qts, 40 * 1000 * 1000);
    {
        g_autoptr(GArray) r0 = read_log(&c, 0);
        g_autoptr(GArray) r1 = read_log(&c, 1);

        seed = 3;
        for (i = 0; i < 12; i++) {
            px[i] = (Px) { prng(&seed), prng(&seed), prng(&seed) };
        }
        for (y = 0; y < 2; y++) {
            uint8_t want[18];
            bool tol;

            expect_line("raw12", "rggb", false, false, px + 6 * y, 6, y,
                        want, &tol);
            g_assert_cmpmem(find_long(r0, 0, y)->payload, 9, want, 9);
        }
        for (i = 0; i < 12; i++) {
            uint32_t v = prng(&seed) % 1001;

            px[i] = grey(((uint64_t)v * 65535 + 500) / 1000);
        }
        for (y = 0; y < 2; y++) {
            uint8_t want[18];
            bool tol;

            expect_line("rgb888", NULL, false, false, px + 6 * y, 6, y, want,
                        &tol);
            g_assert_cmpmem(find_long(r1, 0, y)->payload, 18, want, 18);
        }
    }
    cam_stop(&c);
}

/*
 * PNG, written here: every row filtered in turn with each of the five
 * filters, the image data split over two IDAT chunks, Adam7 when asked.
 */
typedef struct PngSpec {
    uint32_t w, h;
    unsigned depth, type;
    bool interlace;
    /* Samples, channels to a pixel, row by row */
    const uint32_t *s;
    const uint8_t *palette;
    unsigned npal;
} PngSpec;

static unsigned png_channels(unsigned type)
{
    static const unsigned ch[7] = { 1, 0, 3, 1, 2, 0, 4 };

    return ch[type];
}

static void png_chunk(GByteArray *out, const char *type, const uint8_t *data,
                      size_t len)
{
    uint8_t hdr[8], crc[4];
    uint32_t c;

    stl_be_p(hdr, len);
    memcpy(hdr + 4, type, 4);
    g_byte_array_append(out, hdr, 8);
    g_byte_array_append(out, data, len);
    c = crc32(crc32(0, NULL, 0), hdr + 4, 4);
    if (len) {
        /* (zlib's crc32() of a NULL buffer is its initial value) */
        c = crc32(c, data, len);
    }
    stl_be_p(crc, c);
    g_byte_array_append(out, crc, 4);
}

static unsigned png_pred(unsigned f, int a, int b, int c)
{
    int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);

    switch (f) {
    case 1:
        return a;
    case 2:
        return b;
    case 3:
        return (a + b) / 2;
    case 4:
        return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
    default:
        return 0;
    }
}

static GByteArray *png(const PngSpec *sp)
{
    static const uint8_t adam7[7][4] = {
        { 0, 0, 8, 8 }, { 4, 0, 8, 8 }, { 0, 4, 4, 8 }, { 2, 0, 4, 4 },
        { 0, 2, 2, 4 }, { 1, 0, 2, 2 }, { 0, 1, 1, 2 },
    };
    static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a,
                                    '\n' };
    GByteArray *out = g_byte_array_new();
    g_autoptr(GByteArray) raw = g_byte_array_new();
    unsigned ch = png_channels(sp->type), passes = sp->interlace ? 7 : 1;
    unsigned bpp = MAX(1, ch * sp->depth / 8), pass, row = 0;
    uint8_t ihdr[13];
    uLongf zlen;
    g_autofree uint8_t *z = NULL;

    g_byte_array_append(out, sig, 8);
    stl_be_p(ihdr, sp->w);
    stl_be_p(ihdr + 4, sp->h);
    ihdr[8] = sp->depth;
    ihdr[9] = sp->type;
    ihdr[10] = ihdr[11] = 0;
    ihdr[12] = sp->interlace;
    png_chunk(out, "IHDR", ihdr, 13);
    if (sp->palette) {
        png_chunk(out, "PLTE", sp->palette, 3 * sp->npal);
    }
    png_chunk(out, "tEXt", (const uint8_t *)"Comment\0test", 12);
    for (pass = 0; pass < passes; pass++) {
        uint32_t x0 = sp->interlace ? adam7[pass][0] : 0;
        uint32_t y0 = sp->interlace ? adam7[pass][1] : 0;
        uint32_t dx = sp->interlace ? adam7[pass][2] : 1;
        uint32_t dy = sp->interlace ? adam7[pass][3] : 1;
        uint32_t pw = sp->w > x0 ? (sp->w - x0 + dx - 1) / dx : 0;
        uint32_t ph = sp->h > y0 ? (sp->h - y0 + dy - 1) / dy : 0;
        size_t rb = ((size_t)pw * ch * sp->depth + 7) / 8;
        g_autofree uint8_t *prev = g_malloc0(rb + 1);
        g_autofree uint8_t *cur = g_malloc0(rb + 1);
        uint32_t r, x;

        if (!pw || !ph) {
            continue;
        }
        for (r = 0; r < ph; r++) {
            unsigned f = row++ % 5;
            uint8_t fb = f;
            size_t i;

            memset(cur, 0, rb);
            for (x = 0; x < pw; x++) {
                const uint32_t *s = sp->s + ((size_t)(y0 + r * dy) * sp->w +
                                             x0 + x * dx) * ch;
                unsigned k;

                for (k = 0; k < ch; k++) {
                    size_t bit = ((size_t)x * ch + k) * sp->depth;

                    if (sp->depth == 16) {
                        stw_be_p(cur + bit / 8, s[k]);
                    } else {
                        cur[bit / 8] |= s[k] << (8 - sp->depth - bit % 8);
                    }
                }
            }
            g_byte_array_append(raw, &fb, 1);
            for (i = 0; i < rb; i++) {
                int a = i >= bpp ? cur[i - bpp] : 0;
                int b = r ? prev[i] : 0;
                int cc = r && i >= bpp ? prev[i - bpp] : 0;
                uint8_t v = cur[i] - png_pred(f, a, b, cc);

                g_byte_array_append(raw, &v, 1);
            }
            memcpy(prev, cur, rb);
        }
    }
    zlen = compressBound(raw->len);
    z = g_malloc(zlen);
    g_assert_cmpint(compress2(z, &zlen, raw->data, raw->len, 9), ==, Z_OK);
    png_chunk(out, "IDAT", z, zlen / 2);
    png_chunk(out, "IDAT", z + zlen / 2, zlen - zlen / 2);
    png_chunk(out, "IEND", NULL, 0);
    return out;
}

/* The 16-bit RGB a PNG sample set gives. */
static Px png_px(const PngSpec *sp, uint32_t i)
{
    unsigned ch = png_channels(sp->type);
    const uint32_t *s = sp->s + (size_t)i * ch;
    uint32_t max = (1u << sp->depth) - 1;

    switch (sp->type) {
    case 3:
        return (Px) { sp->palette[3 * s[0]] * 257,
                      sp->palette[3 * s[0] + 1] * 257,
                      sp->palette[3 * s[0] + 2] * 257 };
    case 0:
    case 4:
        return grey(s[0] * 65535 / max);
    default:
        return (Px) { s[0] * 65535 / max, s[1] * 65535 / max,
                      s[2] * 65535 / max };
    }
}

/*
 * PNG of each colour type at its bit depths, plain and interlaced, odd
 * sizes and 16-bit samples through RAW12, the rest through RGB888.
 */
static void test_still_png(void)
{
    static const struct {
        unsigned type, depth;
        bool interlace;
    } kinds[] = {
        { 2, 8, false }, { 2, 16, true }, { 6, 8, true }, { 6, 16, false },
        { 0, 1, true }, { 0, 2, false }, { 0, 4, true }, { 0, 8, false },
        { 0, 16, true }, { 4, 8, false }, { 4, 16, true }, { 3, 1, false },
        { 3, 2, true }, { 3, 4, false }, { 3, 8, true },
    };
    const unsigned n = ARRAY_SIZE(kinds);
    const char *srcs[ARRAY_SIZE(kinds)], *ports[ARRAY_SIZE(kinds)];
    char *srcbuf[ARRAY_SIZE(kinds)];
    PngSpec specs[ARRAY_SIZE(kinds)];
    uint32_t *samples[ARRAY_SIZE(kinds)];
    uint8_t palette[256 * 3];
    Cam c = { 0 };
    uint32_t seed = 11, i, k;

    for (i = 0; i < sizeof(palette); i++) {
        palette[i] = prng(&seed);
    }
    for (k = 0; k < n; k++) {
        bool deep = kinds[k].depth == 16;
        uint32_t w = deep ? 10 : 7, h = deep ? 6 : 5;
        unsigned ch = png_channels(kinds[k].type);
        g_autoptr(GByteArray) file = NULL;
        g_autofree char *name = g_strdup_printf("p%u.png", k);
        g_autofree char *path = NULL;

        samples[k] = g_new(uint32_t, w * h * ch);
        for (i = 0; i < w * h * ch; i++) {
            samples[k][i] = prng(&seed) & ((1u << kinds[k].depth) - 1);
        }
        specs[k] = (PngSpec) {
            .w = w, .h = h, .depth = kinds[k].depth, .type = kinds[k].type,
            .interlace = kinds[k].interlace, .s = samples[k],
            .palette = kinds[k].type == 3 ? palette : NULL,
            .npal = 1u << kinds[k].depth,
        };
        file = png(&specs[k]);
        path = cam_file(&c, name, file->data, file->len);
        srcbuf[k] = g_strdup_printf("image=%s,format=%s", path,
                                    deep ? "raw12" : "rgb888");
        srcs[k] = srcbuf[k];
        ports[k] = "csi";
    }
    cam_start(&c, NULL, n, srcs, ports);
    qtest_clock_step(c.qts, 40 * 1000 * 1000);
    for (k = 0; k < n; k++) {
        g_autoptr(GArray) recs = read_log(&c, k);
        bool deep = kinds[k].depth == 16;
        uint32_t w = specs[k].w, y, x;

        for (y = 0; y < specs[k].h; y++) {
            Rec *r = find_long(recs, 0, y);
            uint8_t want[64];
            Px px[16];
            bool tol;

            g_assert(r);
            for (x = 0; x < w; x++) {
                px[x] = png_px(&specs[k], y * w + x);
            }
            expect_line(deep ? "raw12" : "rgb888", "rggb", false, false, px,
                        w, y, want, &tol);
            g_assert_cmpmem(r->payload, r->len, want,
                            deep ? w * 3 / 2 : w * 3);
        }
        g_free(srcbuf[k]);
        g_free(samples[k]);
    }
    cam_stop(&c);
}

/*
 * A picture of another size is scaled to the frame: bilinear, pixel
 * centres aligned, worked out here in floating point.
 */
static void test_still_scaled(void)
{
    Px px[3 * 2];
    g_autoptr(GByteArray) pic = NULL;
    g_autofree char *path = NULL, *src = NULL;
    g_autoptr(GArray) recs = NULL;
    Cam c = { 0 };
    uint32_t seed = 5, i, x, y;

    for (i = 0; i < 6; i++) {
        px[i] = (Px) { (prng(&seed) & 0xff) * 257, (prng(&seed) & 0xff) * 257,
                       (prng(&seed) & 0xff) * 257 };
    }
    pic = ppm(3, 2, 255, false, px);
    path = cam_file(&c, "s.ppm", pic->data, pic->len);
    src = g_strdup_printf("image=%s,format=rgb888,width=8,height=5", path);
    cam_start1(&c, src, "csi");
    qtest_clock_step(c.qts, 40 * 1000 * 1000);
    recs = read_log(&c, 0);
    for (y = 0; y < 5; y++) {
        Rec *r = find_long(recs, 0, y);

        g_assert(r);
        for (x = 0; x < 8; x++) {
            double fx = MIN(MAX((x + 0.5) * 3 / 8 - 0.5, 0), 2);
            double fy = MIN(MAX((y + 0.5) * 2 / 5 - 0.5, 0), 1);
            int x0 = floor(fx), y0 = floor(fy);
            int x1 = MIN(x0 + 1, 2), y1 = MIN(y0 + 1, 1);
            double wx = fx - x0, wy = fy - y0;
            unsigned ch;

            for (ch = 0; ch < 3; ch++) {
#define S(xx, yy) ((double)(ch == 0 ? px[(yy) * 3 + (xx)].r : \
                            ch == 1 ? px[(yy) * 3 + (xx)].g : \
                                      px[(yy) * 3 + (xx)].b))
                double v = (S(x0, y0) * (1 - wx) + S(x1, y0) * wx) * (1 - wy) +
                           (S(x0, y1) * (1 - wx) + S(x1, y1) * wx) * wy;
#undef S
                int want = floor(v * 255 / 65535 + 0.5);

                g_assert_cmpint(abs(r->payload[3 * x + ch] - want), <=, 1);
            }
        }
    }
    cam_stop(&c);
}

/*
 * Raw frame files
 * ===============
 */

/*
 * Three frames of RAW10 and RAW12, played in a loop: on CSI the payloads
 * are the file's lines, on DVP the samples are its pixels.
 */
static void test_file(void)
{
    const char *srcs[4], *ports[4] = { "csi", "dvp", "csi", "dvp" };
    g_autofree char *s10 = NULL, *s12 = NULL, *p10 = NULL, *p12 = NULL;
    uint8_t f10[3 * 3 * 10], f12[3 * 3 * 12];
    Cam c = { 0 };
    uint32_t seed = 9, i, n;

    for (i = 0; i < sizeof(f10); i++) {
        f10[i] = prng(&seed);
    }
    for (i = 0; i < sizeof(f12); i++) {
        f12[i] = prng(&seed);
    }
    p10 = cam_file(&c, "f10.raw", f10, sizeof(f10));
    p12 = cam_file(&c, "f12.raw", f12, sizeof(f12));
    s10 = g_strdup_printf("file=%s,format=raw10,width=8,height=3", p10);
    s12 = g_strdup_printf("file=%s,format=raw12,width=8,height=3", p12);
    srcs[0] = srcs[1] = s10;
    srcs[2] = srcs[3] = s12;
    cam_start(&c, NULL, 4, srcs, ports);
    qtest_clock_step(c.qts, 170 * 1000 * 1000);
    for (n = 0; n < 4; n++) {
        g_autoptr(GArray) recs = read_log(&c, n);
        const uint8_t *file = n < 2 ? f10 : f12;
        unsigned bits = n < 2 ? 10 : 12, lb = 8 * bits / 8;
        uint32_t f, y, x;

        for (f = 0; f < 5; f++) {
            for (y = 0; y < 3; y++) {
                const uint8_t *line = file + (f % 3) * 3 * lb + y * lb;

                if (n % 2 == 0) {
                    Rec *r = find_long(recs, f, y);

                    g_assert(r);
                    g_assert_cmpmem(r->payload, r->len, line, lb);
                } else {
                    Rec *r = find_line(recs, f, y);

                    g_assert(r);
                    g_assert_cmpuint(r->count, ==, 8);
                    for (x = 0; x < 8; x++) {
                        uint32_t v;

                        if (bits == 10) {
                            v = line[x / 4 * 5 + x % 4] << 2 |
                                ((line[x / 4 * 5 + 4] >> (2 * (x % 4))) & 3);
                        } else {
                            v = line[x / 2 * 3 + x % 2] << 4 |
                                ((line[x / 2 * 3 + 2] >> (4 * (x % 2))) & 15);
                        }
                        g_assert_cmphex(r->samples[x], ==, v);
                    }
                }
            }
        }
    }
    cam_stop(&c);
}

/*
 * The checkerboard
 * ================
 */

static void test_checker(void)
{
    g_autoptr(GArray) recs = NULL;
    Cam c = { 0 };
    uint32_t f, y, x;

    cam_start1(&c, "pattern=checker,format=raw8,bayer=bggr,width=64,"
               "height=12,cell=4", "csi");
    qtest_clock_step(c.qts, 300 * 1000 * 1000);
    recs = read_log(&c, 0);
    for (f = 0; f < 9; f++) {
        for (y = 0; y < 12; y++) {
            Rec *r = find_long(recs, f, y);

            g_assert(r);
            for (x = 0; x < 64; x++) {
                Px p = checker_px(64, 4, f, x, y);

                g_assert_cmphex(r->payload[x], ==,
                                to_bits(bayer_pick("bggr", p, x, y), 8));
            }
        }
    }
    cam_stop(&c);
}

/*
 * Starting and stopping
 * =====================
 */

/*
 * A software stream off ends the stream as the frame in progress ends;
 * on starts a new frame at once, its timing counted from then, the
 * frame numbers carrying on. A sensor created with streaming off waits.
 */
static void test_streaming(void)
{
    const char *srcs[] = {
        "format=raw8,width=16,height=4,cell=2",
        "format=raw8,width=16,height=4,cell=2,streaming=off",
    };
    const char *ports[] = { "dvp", "csi" };
    const uint64_t pixel_hz = 30 * 32 * 7, frame_clocks = 32 * 7;
    g_autoptr(GArray) recs = NULL, recs1 = NULL;
    Cam c = { 0 };
    guint i;
    Rec *r;

    cam_start(&c, NULL, 2, srcs, ports);
    /* Into frame 1, 33.3 to 66.7 ms */
    qtest_clock_step(c.qts, 50 * 1000 * 1000);
    qom_set_bool(c.qts, "/objects/cam0", "streaming", false);
    qtest_clock_step(c.qts, 50 * 1000 * 1000);
    qom_set_bool(c.qts, "/objects/cam0", "streaming", true);
    qom_set_bool(c.qts, "/objects/cam1", "streaming", true);
    qtest_clock_step(c.qts, 20 * 1000 * 1000);
    recs = read_log(&c, 0);

    g_assert(find(recs, "line", 0, NULL));
    r = find(recs, "stream", 0, &i);
    g_assert(r && !r->on);
    /* Frame 1 finished: off where frame 2 would have begun */
    g_assert_cmpint(r->t[0], ==, ceil_ns(2 * frame_clocks, pixel_hz));
    g_assert_cmpint(r->now, ==, r->t[0]);
    g_assert(find_line(recs, 1, 3));
    /* Nothing until the stream starts again */
    r = rec(recs, ++i);
    g_assert(!strcmp(r->type, "stream") && r->on);
    g_assert_cmpint(r->t[0], ==, 100 * 1000 * 1000);
    r = find(recs, "vsync", i, &i);
    g_assert_cmpuint(r->frame, ==, 2);
    g_assert_cmpint(r->t[0], ==, 100 * 1000 * 1000);
    r = find(recs, "line", i, &i);
    g_assert_cmpuint(r->frame, ==, 2);
    g_assert_cmpint(r->epoch, ==, 100 * 1000 * 1000);
    /* Line 0 is frame line vsync + vbp = 2 */
    g_assert_cmpint(r->t[0], ==, 100 * 1000 * 1000 + ceil_ns(2 * 32, pixel_hz));

    recs1 = read_log(&c, 1);
    r = find(recs1, "attach", 0, NULL);
    g_assert(!r->active);
    g_assert(!find(recs1, "packet", 0, NULL) ||
             find(recs1, "packet", 0, NULL)->t[0] >= 100 * 1000 * 1000);
    r = find(recs1, "stream", 0, &i);
    g_assert(r && r->on);
    g_assert_cmpint(r->t[0], ==, 100 * 1000 * 1000);
    r = find(recs1, "packet", i, &i);
    g_assert_cmphex(r->dt, ==, 0x00);
    g_assert_cmpuint(r->frame, ==, 0);
    g_assert_cmpint(r->t[0], ==, 100 * 1000 * 1000);
    cam_stop(&c);
}

/*
 * Two receivers on one sensor (LCD_CAM and the ISP on DVP) take the same
 * lines at the same times.
 */
static void test_two_receivers(void)
{
    g_autofree char *args = NULL;
    g_autoptr(GArray) a = NULL, b = NULL;
    Cam c = { 0 };
    guint i;

    c.dir = g_dir_make_tmp("esp32p4-imgsrc-XXXXXX", NULL);
    args = g_strdup_printf("-M esp32p4,strap-mode=0 "
                           "-object esp32p4-image-source,id=cam0,"
                           "format=rgb565,width=32,height=4,cell=2 "
                           "-object esp32p4-image-probe,id=probe0,"
                           "source=cam0,port=dvp,log=%s/probe0.log "
                           "-object esp32p4-image-probe,id=probe1,"
                           "source=cam0,port=dvp,log=%s/probe1.log",
                           c.dir, c.dir);
    c.qts = qtest_init(args);
    qtest_clock_step(c.qts, 70 * 1000 * 1000);
    a = read_log(&c, 0);
    b = read_log(&c, 1);
    g_assert_cmpuint(a->len, >, 8);
    for (i = 0; i < a->len && strcmp(rec(a, i)->type, "line"); i++) {
    }
    for (; i < a->len; i++) {
        Rec *ra = rec(a, i), *rb = find_line(b, ra->frame, ra->line);

        if (strcmp(ra->type, "line")) {
            continue;
        }
        g_assert(rb);
        g_assert_cmpmem(ra->t, sizeof(ra->t), rb->t, sizeof(rb->t));
        g_assert_cmpmem(ra->samples, 2 * ra->count, rb->samples,
                        2 * rb->count);
    }
    cam_stop(&c);
}

/*
 * The board's pads
 * ================
 */

static uint32_t sync_pads(QTestState *qts)
{
    return (qtest_readl(qts, GPIO_IN) >> 20) & 7;
}

/*
 * VSYNC (GPIO20), DE (GPIO21, active low here) and HSYNC (GPIO22) on
 * their pads at their edges' times; the sensor stops at once as PWDN
 * (GPIO23) rises or RESETB (GPIO24) falls, letting its pads go, and
 * starts again as they return.
 */
static void test_pads(void)
{
    const uint64_t pclk = 30 * 32 * 7 * 2;
    const char *src = "format=rgb565,width=16,height=4,cell=2,hblank=16,"
                      "hsync=2,hbp=4,vblank=3,vsync=1,vbp=1,"
                      "vsync-pad=20,de-pad=21,hsync-pad=22,de-active-low=on,"
                      "pwdn-pad=23,reset-pad=24";
    const char *port = "dvp";
    g_autoptr(GArray) recs = NULL;
    Cam c = { 0 };
    QTestState *qts;
    int64_t de, de_end, t;
    unsigned n, floating;
    guint i;
    Rec *r;

    cam_start(&c, "dvp-camera=cam0", 1, &src, &port);
    qts = c.qts;
    for (n = 20; n <= 24; n++) {
        qtest_writel(qts, IO_MUX_GPIO(n), MCU_SEL_GPIO | FUN_IE);
    }
    qtest_writel(qts, IO_MUX_GPIO(30), MCU_SEL_GPIO | FUN_IE);
    /* An undriven pad, set up as the sync pads are */
    floating = (qtest_readl(qts, GPIO_IN) >> 30) & 1;
    /* The chip drives RESETB high and PWDN low, as a driver does */
    for (n = 23; n <= 24; n++) {
        qtest_writel(qts, GPIO_FUNC_OUT(n), SIG_GPIO_OUT | OEN_SEL);
    }
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 24);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 23);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 3u << 23);

    /*
     * Line 0: VSYNC, HSYNC for 4 PCLKs. Line 1: HSYNC. Line 2, the first
     * of pixels: HSYNC, and DE from PCLK 12 for 32.
     */
    g_assert_cmphex(sync_pads(qts), ==, 7);
    qtest_clock_set(qts, ceil_ns(4, pclk) - 1);
    g_assert_cmphex(sync_pads(qts), ==, 7);
    qtest_clock_set(qts, ceil_ns(4, pclk));
    g_assert_cmphex(sync_pads(qts), ==, 3);
    qtest_clock_set(qts, ceil_ns(64, pclk) - 1);
    g_assert_cmphex(sync_pads(qts), ==, 3);
    qtest_clock_set(qts, ceil_ns(64, pclk));
    g_assert_cmphex(sync_pads(qts), ==, 6);
    qtest_clock_set(qts, ceil_ns(68, pclk));
    g_assert_cmphex(sync_pads(qts), ==, 2);
    de = ceil_ns(128 + 12, pclk);
    de_end = ceil_ns(128 + 12 + 32, pclk);
    qtest_clock_set(qts, de - 1);
    g_assert_cmphex(sync_pads(qts), ==, 2);
    qtest_clock_set(qts, de);
    g_assert_cmphex(sync_pads(qts), ==, 0);
    qtest_clock_set(qts, de_end - 1);
    g_assert_cmphex(sync_pads(qts), ==, 0);
    qtest_clock_set(qts, de_end);
    g_assert_cmphex(sync_pads(qts), ==, 2);

    /* Power down while line 3's DE is active: the pads float */
    t = ceil_ns(3 * 64 + 20, pclk);
    qtest_clock_set(qts, t);
    g_assert_cmphex(sync_pads(qts), ==, 0);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 23);
    g_assert_cmphex(sync_pads(qts), ==, floating * 7);
    qtest_clock_set(qts, 50 * 1000 * 1000);
    g_assert_cmphex(sync_pads(qts), ==, floating * 7);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 23);
    g_assert_cmphex(sync_pads(qts), ==, 7);
    qtest_clock_set(qts, 60 * 1000 * 1000);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 24);
    g_assert_cmphex(sync_pads(qts), ==, floating * 7);
    qtest_clock_set(qts, 70 * 1000 * 1000);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 24);
    g_assert_cmphex(sync_pads(qts), ==, 7);
    qtest_clock_set(qts, 80 * 1000 * 1000);

    /*
     * RESETB floats low until the chip drives it: the sensor is held in
     * reset from power-on, and frame 1 starts as the driver lets it go.
     */
    recs = read_log(&c, 0);
    r = find(recs, "stream", 0, &i);
    g_assert(r && !r->on && r->t[0] == 0);
    r = find(recs, "stream", i + 1, &i);
    g_assert(r && r->on && r->t[0] == 0);
    r = find(recs, "stream", i + 1, &i);
    g_assert(r && !r->on);
    g_assert_cmpint(r->t[0], ==, t);
    g_assert(find_line(recs, 1, 0));
    g_assert(!find_line(recs, 1, 1));
    r = find(recs, "stream", i + 1, &i);
    g_assert(r && r->on);
    g_assert_cmpint(r->t[0], ==, 50 * 1000 * 1000);
    r = find(recs, "stream", i + 1, &i);
    g_assert(r && !r->on);
    g_assert_cmpint(r->t[0], ==, 60 * 1000 * 1000);
    r = find(recs, "stream", i + 1, &i);
    g_assert(r && r->on);
    g_assert_cmpint(r->t[0], ==, 70 * 1000 * 1000);
    r = find(recs, "vsync", i, &i);
    g_assert_cmpint(r->t[0], ==, 70 * 1000 * 1000);
    cam_stop(&c);
}

/*
 * What the sensor refuses
 * =======================
 */

static void expect_refused(QTestState *qts, const char *props)
{
    g_autofree char *json = g_strdup_printf(
        "{ 'qom-type': 'esp32p4-image-source', 'id': 'bad', %s }", props);
    QObject *args = qobject_from_json(json, &error_abort);
    QDict *r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': %p }",
                         args);

    g_assert(qdict_haskey(r, "error"));
    qobject_unref(r);
}

static void test_refused(void)
{
    QTestState *qts = qtest_init("-M esp32p4,strap-mode=0");
    QDict *r;

    expect_refused(qts, "'format': 'raw10', 'width': 30");
    expect_refused(qts, "'format': 'yuv422', 'width': 31");
    expect_refused(qts, "'format': 'nv12'");
    expect_refused(qts, "'bayer': 'rgbg'");
    expect_refused(qts, "'pattern': 'plasma'");
    expect_refused(qts, "'file': '/nonexistent', 'image': '/nonexistent'");
    expect_refused(qts, "'image': '/nonexistent'");
    expect_refused(qts, "'width': 64, 'hts': 64");
    expect_refused(qts, "'width': 64, 'hts': 100, 'hblank': 30");
    expect_refused(qts, "'hblank': 16, 'hsync': 10, 'hbp': 7");
    expect_refused(qts, "'vblank': 3, 'vsync': 2, 'vbp': 2");
    expect_refused(qts, "'fps': 30, 'pixel-clock': 1000");
    expect_refused(qts, "'cell': 3");
    expect_refused(qts, "'dvp-width': 12");
    expect_refused(qts, "'format': 'raw8', 'dvp-width': 16");
    expect_refused(qts, "'vsync-pad': 20, 'de-pad': 20");
    expect_refused(qts, "'data-pads': '1,2,99'");
    expect_refused(qts, "'vc': 4");
    expect_refused(qts, "'lanes': 5");

    /* Port checks happen as the sensor is wired */
    r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': "
                  "{ 'qom-type': 'esp32p4-image-source', 'id': 'c1', "
                  "'width': 1920, 'height': 1080, 'format': 'rgb888', "
                  "'lane-rate': 100000000 } }");
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': "
                  "{ 'qom-type': 'esp32p4-image-probe', 'id': 'p1', "
                  "'source': 'c1', 'port': 'csi', 'log': '/dev/null' } }");
    g_assert(qdict_haskey(r, "error"));
    qobject_unref(r);
    r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': "
                  "{ 'qom-type': 'esp32p4-image-source', 'id': 'c2', "
                  "'de-pad': 5 } }");
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': "
                  "{ 'qom-type': 'esp32p4-image-probe', 'id': 'p2', "
                  "'source': 'c2', 'port': 'csi', 'log': '/dev/null' } }");
    g_assert(qdict_haskey(r, "error"));
    qobject_unref(r);
    r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': "
                  "{ 'qom-type': 'esp32p4-image-source', 'id': 'c3', "
                  "'format': 'rgb888', 'dvp-width': 16 } }");
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    r = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments': "
                  "{ 'qom-type': 'esp32p4-image-probe', 'id': 'p3', "
                  "'source': 'c3', 'port': 'dvp', 'log': '/dev/null' } }");
    g_assert(qdict_haskey(r, "error"));
    qobject_unref(r);
    /* Once made, the sensor's settings are fixed */
    r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                  "{ 'path': '/objects/c1', 'property': 'width', "
                  "'value': 640 } }");
    g_assert(qdict_haskey(r, "error"));
    qobject_unref(r);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
/* [spec:nuos:req:emu.esp32p4.mipi/test] */
int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/image-source/crc-reference", test_crc_reference);
    qtest_add_func("esp32p4/image-source/dvp-timing", test_dvp_timing);
    qtest_add_func("esp32p4/image-source/dvp-bars", test_dvp_bars);
    qtest_add_func("esp32p4/image-source/csi-packets", test_csi_packets);
    qtest_add_func("esp32p4/image-source/csi-in-burst", test_csi_in_burst);
    qtest_add_func("esp32p4/image-source/csi-lane-rate", test_csi_lane_rate);
    qtest_add_func("esp32p4/image-source/still-formats", test_still_formats);
    qtest_add_func("esp32p4/image-source/still-pnm", test_still_pnm);
    qtest_add_func("esp32p4/image-source/still-png", test_still_png);
    qtest_add_func("esp32p4/image-source/still-scaled", test_still_scaled);
    qtest_add_func("esp32p4/image-source/file", test_file);
    qtest_add_func("esp32p4/image-source/checker", test_checker);
    qtest_add_func("esp32p4/image-source/streaming", test_streaming);
    qtest_add_func("esp32p4/image-source/two-receivers", test_two_receivers);
    qtest_add_func("esp32p4/image-source/pads", test_pads);
    qtest_add_func("esp32p4/image-source/refused", test_refused);
    return g_test_run();
}
