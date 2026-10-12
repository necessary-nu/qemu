/*
 * QTest testcase for the ESP32-P4 JPEG codec
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The codec's registers and tables as ESP-IDF's jpeg_ll.h and jpeg_hal.c
 * drive them, its interrupts, errors and timing, and whole frames through
 * its 2D-DMA paths, which the test plays through the codec's "dma"
 * property: the encoder's source in macroblock order with the 2D-DMA's
 * padding (TRM 8.4.3), the decoder's bitstream from SOS as ESP-IDF's
 * jpeg_decode.c hands it over.
 *
 * Codec output is checked by an independent decoder, libjpeg-turbo's
 * djpeg. The encoder's file (ESP-IDF's headers before
 * the codec's scan) must decode to exactly what cjpeg's encoding of the
 * same image with the same quantisation tables, sampling and integer DCT
 * decodes to: the model's arithmetic is the reference software's, so any
 * difference is an encoder fault. The decoder's output, converted to RGB
 * by JFIF's formulas with the chroma replicated, must equal djpeg's
 * decoding (-dct int -nosmooth) of the same cjpeg or ffmpeg file
 * exactly: the model's IDCT is libjpeg's integer IDCT, so the tolerance
 * is 0. A test needing cjpeg, djpeg or ffmpeg skips when it is not on
 * PATH.
 *
 * The codec's clock, SYS_CLK, is 20 MHz from reset (XTAL_CLK 40 MHz, MEM
 * and SYS divided by 2): a cycle is 50 ns.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qobject/qdict.h"

#define JPEG            0x50086000
#define HP_SYS_CLKRST   0x500e6000
#define SOC_CLK_CTRL1   (HP_SYS_CLKRST + 0x18)
#define JPEG_SYS_CLK_EN (1u << 8)
#define HP_RST_EN0      (HP_SYS_CLKRST + 0xc0)
#define RST_EN_JPEG     (1u << 30)
#define INTMTX_STATUS(w) (0x500d6000 + 0x200 + 4 * (w))
#define SRC_JPEG        95

/* L2MEM past the L2 cache's 128 KiB; qtest's accesses miss the caches. */
#define MEM_IN          0x4ff40000u
#define MEM_OUT         0x4ff80000u
#define MEM_IN_SIZE     0x40000u
#define MEM_OUT_SIZE    0x30000u

#define CYCLE_NS        50
/* The model's pipeline: 100 cycles per data unit (esp32p4_jpeg.c). */
#define UNIT            100

/* Registers. */
#define CONFIG          0x00
#define DQT_INFO        0x04
#define PIC_SIZE        0x08
#define EXTD_CONFIG     0x0c
#define TQNR(n)         (0x10 + 4 * (n))
#define DECODE_CONF     0x20
#define COMP(n)         (0x24 + 4 * (n))
#define DHT_INFO        0x34
#define INT_RAW         0x38
#define INT_ENA         0x3c
#define INT_ST          0x40
#define INT_CLR         0x44
#define STATUS0         0x48
#define STATUS2         0x4c
#define DHT_TOTLEN_DC(n)  (0x58 + 0x10 * (n))
#define DHT_VAL_DC(n)     (0x5c + 0x10 * (n))
#define DHT_TOTLEN_AC(n)  (0x60 + 0x10 * (n))
#define DHT_VAL_AC(n)     (0x64 + 0x10 * (n))
#define DHT_CODEMIN_DC(n) (0x78 + 8 * (n))
#define DHT_CODEMIN_AC(n) (0x7c + 8 * (n))
#define DECODER_STATUS5 0x9c
#define ECO_LOW         0xa4
#define ECO_HIGH        0xa8
#define SYS             0xf8
#define VERSION         0xfc
#define QNR_RAM(n, i)       (0x100 + 0x100 * (n) + 4 * (i))
#define TOTLEN_RAM(ac, n, l) (0x500 + 0x40 * (ac) + 0x80 * (n) + 4 * (l))
#define VAL_DC_RAM(n, i)    (0x600 + 0x40 * (n) + 4 * (i))
#define VAL_AC_RAM(n, i)    (0x680 + 0x400 * (n) + 4 * (i))
#define CODEMIN_RAM(ac, n, l) (0xf00 + 0x40 * (ac) + 0x80 * (n) + 4 * (l))

/* JPEG_CONFIG_REG. */
#define FSM_RST         (1u << 0)
#define JPEG_START      (1u << 1)
#define QNR_PRECISION   (1u << 2)
#define FF_CHECK_EN     (1u << 3)
#define SAMPLE_SEL(x)   ((x) << 4)
#define DEBUG_DIRECT    (1u << 7)
#define QNR_FIFO_EN     (1u << 8)
#define LQNR_SEL(x)     ((x) << 9)
#define CQNR_SEL(x)     ((x) << 11)
#define COLOR_SPACE(x)  ((x) << 13)
#define DHT_FIFO_EN     (1u << 15)
#define TMO_THRES(x)    ((x) << 17)
#define TMO_TASK_SEL    (1u << 23)
#define SOFT_RST        (1u << 24)
#define FIFO_RST        (1u << 25)
#define PIXEL_REV       (1u << 26)
#define TAILER_EN       (1u << 27)
#define PAUSE_EN        (1u << 28)
#define MODE_DECODE     (1u << 31)

/* JPEG_INT_RAW_REG. */
#define I_DONE              (1u << 0)
#define I_RLE_PARALLEL_ERR  (1u << 1)
#define I_CID_ERR           (1u << 2)
#define I_DHT_DC_ID_ERR     (1u << 3)
#define I_DHT_AC_ID_ERR     (1u << 4)
#define I_DQT_ID_ERR        (1u << 5)
#define I_RST_UXP_ERR       (1u << 6)
#define I_RST_CHECK_NONE    (1u << 7)
#define I_RST_CHECK_POS     (1u << 8)
#define I_OUT_EOF           (1u << 9)
#define I_DCT_DONE          (1u << 11)
#define I_BS_LAST_BLOCK     (1u << 12)
#define I_SCAN_CHECK_POS    (1u << 14)
#define I_UXP_DET           (1u << 15)
#define I_EN_EOF_ERR        (1u << 16)
#define I_EN_EOF_LACK       (1u << 17)
#define I_DE_EOF_ERR        (1u << 18)
#define I_DE_EOF_LACK       (1u << 19)
#define I_SOS_UNMATCH       (1u << 20)
#define I_MARKER_ERR_FST    (1u << 21)
#define I_UNDET             (1u << 23)
#define I_DECODE_TIMEOUT    (1u << 24)

/* The stand-in's flags: what marked the codec's last byte. */
#define DMA_EOF         1
#define DMA_ERR_EOF     2

/* ESP-IDF's names for the encoder's sources and the coded formats. */
enum { SRC_RGB888, SRC_YUV422, SRC_RGB565, SRC_GRAY, SRC_YUV444, SRC_YUV420 };
enum { SUB_444, SUB_422, SUB_420, SUB_GRAY };

static const int sub_mw[] = { 8, 16, 16, 8 };
static const int sub_mh[] = { 8, 8, 16, 8 };

static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* T.81 K.1 and K.2, natural order. */
static const int base_q[2][64] = {
    {
        16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
        14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
        18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92,
        49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99,
    }, {
        17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
        24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    },
};

/* T.81 K.3 to K.6: bits then values, luminance then chrominance. */
static const uint8_t std_dc_bits[2][16] = {
    { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
};
static const uint8_t std_dc_vals[12] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t std_ac_bits[2][16] = {
    { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d },
    { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 },
};
static const uint8_t std_ac_vals[2][162] = {
    {
        0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41,
        0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91,
        0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24,
        0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a,
        0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38,
        0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53,
        0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
        0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
        0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93,
        0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
        0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7,
        0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
        0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1,
        0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2,
        0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
    }, {
        0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12,
        0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14,
        0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15,
        0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17,
        0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
        0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65,
        0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
        0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a,
        0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
        0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5,
        0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
        0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9,
        0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2,
        0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
    },
};

/* The machine and the codec's registers. */

static QTestState *jpeg_init(void)
{
    QTestState *qts = qtest_init("-M esp32p4,strap-mode=0");

    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | JPEG_SYS_CLK_EN);
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, JPEG + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, JPEG + off, v);
}

static void cfg_set(QTestState *qts, uint32_t clear, uint32_t set)
{
    wr(qts, CONFIG, (rd(qts, CONFIG) & ~clear) | set);
}

/* jpeg_ll_soft_rst(). */
static void soft_rst(QTestState *qts)
{
    cfg_set(qts, 0, SOFT_RST);
    cfg_set(qts, SOFT_RST, 0);
}

static void dma(QTestState *qts, const char *cmd)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': '/machine/jpeg', 'property': 'dma', "
                         "'value': %s } }", cmd);

    g_assert(!qdict_haskey(r, "error"));
    qobject_unref(r);
}

/* The stand-in's TX descriptors left, RX bytes written and flags. */
static void dma_status(QTestState *qts, unsigned *tx_left, unsigned *rx,
                       unsigned *flags)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/machine/jpeg', 'property': 'dma' } }");
    unsigned a, b, c;

    g_assert(qdict_haskey(r, "return"));
    g_assert_cmpint(sscanf(qdict_get_str(r, "return"), "%u %u %u",
                           &a, &b, &c), ==, 3);
    qobject_unref(r);
    if (tx_left) {
        *tx_left = a;
    }
    if (rx) {
        *rx = b;
    }
    if (flags) {
        *flags = c;
    }
}

static void dma_tx(QTestState *qts, uint32_t addr, const void *buf,
                   size_t len, bool eof)
{
    g_autofree char *cmd = g_strdup_printf("tx 0x%x %zu %d", addr, len, eof);

    qtest_memwrite(qts, addr, buf, len);
    dma(qts, cmd);
}

static void dma_rx(QTestState *qts, uint32_t addr, size_t len)
{
    g_autofree char *cmd = g_strdup_printf("rx 0x%x %zu", addr, len);

    dma(qts, cmd);
}

/* The virtual time, a nanosecond on. */
static int64_t now(QTestState *qts)
{
    return qtest_clock_step(qts, 1);
}

/* Step virtual time until INT_RAW has one of @bits, at most @max_ns. */
static bool wait_int(QTestState *qts, uint32_t bits, int64_t max_ns)
{
    for (int64_t t = 0; t < max_ns; t += 10000) {
        if (rd(qts, INT_RAW) & bits) {
            return true;
        }
        qtest_clock_step(qts, 10000);
    }
    return rd(qts, INT_RAW) & bits;
}

/* Host tools. */

static char *find_tool(const char *name)
{
    char *path = g_find_program_in_path(name);

    if (!path) {
        g_autofree char *why = g_strdup_printf("%s not found on PATH", name);

        g_test_skip(why);
    }
    return path;
}

static void run_tool(const char *const *argv)
{
    g_autofree char *out = NULL, *err = NULL;
    g_autoptr(GError) error = NULL;
    int status;

    if (!g_spawn_sync(NULL, (char **)argv, NULL, G_SPAWN_DEFAULT, NULL, NULL,
                      &out, &err, &status, &error)) {
        g_error("%s: %s", argv[0], error->message);
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status)) {
        g_error("%s failed: %s", argv[0], err);
    }
}

/* Images: 1 (grey) or 3 (RGB) bytes per pixel. */

typedef struct Img {
    int w;
    int h;
    int nc;
    uint8_t *px;
} Img;

static Img *img_new(int w, int h, int nc)
{
    Img *im = g_new0(Img, 1);

    im->w = w;
    im->h = h;
    im->nc = nc;
    im->px = g_malloc0(w * h * nc);
    return im;
}

static void img_free(Img *im)
{
    if (im) {
        g_free(im->px);
        g_free(im);
    }
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC(Img, img_free)

/*
 * A test image with every kind of content a codec meets: smooth
 * gradients, hard edges, saturated colours and noise.
 */
static Img *make_image(int w, int h, int nc, uint32_t seed)
{
    Img *im = img_new(w, h, nc);
    uint32_t x = seed * 2654435761u + 1;

    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            for (int k = 0; k < nc; k++) {
                int v;

                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                v = (c * 255 / MAX(w - 1, 1) * (k + 1) +
                     r * 255 / MAX(h - 1, 1) * (3 - k)) / 4;
                if (((c / 7) + (r / 5)) % 3 == k) {
                    v = 255 - v;
                }
                if ((c - w / 2) * (c - w / 2) + (r - h / 2) * (r - h / 2) <
                    (w * h) / 16) {
                    v = k == 1 ? 250 : 10 + 20 * k;
                }
                v += (int)(x % 17) - 8;
                im->px[(r * w + c) * nc + k] = MIN(MAX(v, 0), 255);
            }
        }
    }
    return im;
}

/* The image grown to @w x @h by repeating its last column and row. */
static Img *img_pad(const Img *im, int w, int h)
{
    Img *p = img_new(w, h, im->nc);

    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            memcpy(&p->px[(r * w + c) * im->nc],
                   &im->px[(MIN(r, im->h - 1) * im->w + MIN(c, im->w - 1)) *
                           im->nc], im->nc);
        }
    }
    return p;
}

static void write_pnm(const char *path, const Img *im)
{
    g_autofree char *hdr = g_strdup_printf("P%d\n%d %d\n255\n",
                                           im->nc == 3 ? 6 : 5, im->w, im->h);
    g_autoptr(GByteArray) b = g_byte_array_new();

    g_byte_array_append(b, (const guint8 *)hdr, strlen(hdr));
    g_byte_array_append(b, im->px, im->w * im->h * im->nc);
    g_assert(g_file_set_contents(path, (const char *)b->data, b->len, NULL));
}

static Img *read_pnm(const char *path)
{
    g_autofree char *data = NULL;
    int w, h, max, n, p;
    size_t len;
    Img *im;

    g_assert(g_file_get_contents(path, &data, &len, NULL));
    g_assert(data[0] == 'P' && (data[1] == '5' || data[1] == '6'));
    g_assert_cmpint(sscanf(data + 2, "%d %d %d%n", &w, &h, &max, &n), ==, 3);
    g_assert_cmpint(max, ==, 255);
    p = 2 + n + 1;
    im = img_new(w, h, data[1] == '6' ? 3 : 1);
    g_assert_cmpuint(len, >=, p + w * h * im->nc);
    memcpy(im->px, data + p, w * h * im->nc);
    return im;
}

/* The largest difference of two images' first @w x @h pixels, and PSNR. */
static int img_diff(const Img *a, const Img *b, int w, int h, double *psnr)
{
    double se = 0;
    int max = 0;

    g_assert_cmpint(a->nc, ==, b->nc);
    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            for (int k = 0; k < a->nc; k++) {
                int d = a->px[(r * a->w + c) * a->nc + k] -
                        b->px[(r * b->w + c) * b->nc + k];

                se += d * d;
                max = MAX(max, abs(d));
            }
        }
    }
    if (psnr) {
        se /= (double)w * h * a->nc;
        *psnr = se ? 10 * log10(255.0 * 255.0 / se) : 99;
    }
    return max;
}

/* JFIF's RGB to YCbCr, as libjpeg computes it. */
static void rgb_ycc(int r, int g, int b, uint8_t *y, uint8_t *cb, uint8_t *cr)
{
    *y = (19595 * r + 38470 * g + 7471 * b + 32768) >> 16;
    *cb = (-11059 * r - 21709 * g + 32768 * b + (128 << 16) + 32767) >> 16;
    *cr = (32768 * r - 27439 * g - 5329 * b + (128 << 16) + 32767) >> 16;
}

/* JFIF's YCbCr to RGB, as libjpeg's jdcolor.c computes it. */
static void ycc_rgb(int y, int cb, int cr, uint8_t *rgb)
{
    int x = cr - 128, z = cb - 128;
    int rr = y + ((91881 * x + 32768) >> 16);
    int gg = y + ((-46802 * x - 22554 * z + 32768) >> 16);
    int bb = y + ((116130 * z + 32768) >> 16);

    rgb[0] = MIN(MAX(rr, 0), 255);
    rgb[1] = MIN(MAX(gg, 0), 255);
    rgb[2] = MIN(MAX(bb, 0), 255);
}

/* ESP-IDF's compute_quant_table() (jpeg_emit_marker.c). */
static void quant_tables(int quality, uint32_t q[2][64])
{
    int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;

    for (int t = 0; t < 2; t++) {
        for (int i = 0; i < 64; i++) {
            q[t][i] = MIN(MAX((base_q[t][i] * scale + 50) / 100, 1), 255);
        }
    }
}

/* Encoding. */

typedef struct EncCfg {
    int src;
    int sub;
    int quality;
    bool pixel_rev;
} EncCfg;

/* Bytes per pixel of each source, in halves. */
static const int src_half[] = { 6, 4, 4, 2, 6, 3 };

/*
 * The source in the encoder's format, and the RGB (or grey) image the
 * reference encodes: the same pixels, as the format's conversions give
 * them. A YUV source is made by JFIF's conversion of @rgb and libjpeg's
 * down-sampling (its bias alternating along each row), so that its JPEG
 * decodes as cjpeg's of @rgb does.
 */
static uint8_t *make_source(const EncCfg *e, const Img *rgb, Img **ref)
{
    int w = rgb->w, h = rgb->h;
    uint8_t *src = g_malloc0(w * h * src_half[e->src] / 2 + 4);
    uint8_t *y = g_malloc(w * h), *cb = g_malloc(w * h), *cr = g_malloc(w * h);

    *ref = img_new(w, h, e->src == SRC_GRAY ? 1 : 3);
    memcpy((*ref)->px, rgb->px, w * h * rgb->nc);
    for (int i = 0; i < w * h && rgb->nc == 3; i++) {
        rgb_ycc(rgb->px[3 * i], rgb->px[3 * i + 1], rgb->px[3 * i + 2],
                &y[i], &cb[i], &cr[i]);
    }
    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            int i = r * w + c;
            const uint8_t *p = &rgb->px[3 * i];
            uint8_t *o;
            uint16_t v;
            int rr, gg, bb;

            switch (e->src) {
            case SRC_GRAY:
                src[i] = rgb->px[i];
                break;
            case SRC_RGB888:
                o = &src[3 * i];
                o[0] = e->pixel_rev ? p[0] : p[2];
                o[1] = p[1];
                o[2] = e->pixel_rev ? p[2] : p[0];
                break;
            case SRC_RGB565:
                rr = p[0] >> 3;
                gg = p[1] >> 2;
                bb = p[2] >> 3;
                v = e->pixel_rev ? (bb << 11 | gg << 5 | rr)
                                 : (rr << 11 | gg << 5 | bb);
                src[2 * i] = v;
                src[2 * i + 1] = v >> 8;
                o = &(*ref)->px[3 * i];
                o[0] = rr << 3 | rr >> 2;
                o[1] = gg << 2 | gg >> 4;
                o[2] = bb << 3 | bb >> 2;
                break;
            case SRC_YUV444:
                o = &src[3 * i];
                o[0] = e->pixel_rev ? cr[i] : y[i];
                o[1] = cb[i];
                o[2] = e->pixel_rev ? y[i] : cr[i];
                break;
            case SRC_YUV422:
                if (!(c & 1)) {
                    int bias = (c / 2) & 1;
                    uint8_t u = (cb[i] + cb[i + 1] + bias) >> 1;
                    uint8_t vv = (cr[i] + cr[i + 1] + bias) >> 1;

                    o = &src[2 * i];
                    if (e->pixel_rev) {
                        o[0] = vv;
                        o[1] = y[i];
                        o[2] = u;
                        o[3] = y[i + 1];
                    } else {
                        o[0] = y[i];
                        o[1] = vv;
                        o[2] = y[i + 1];
                        o[3] = u;
                    }
                }
                break;
            case SRC_YUV420:
                if (!(c & 1)) {
                    int j = (r & ~1) * w + c;
                    int bias = (c / 2) & 1 ? 2 : 1;
                    uint8_t u = (cb[j] + cb[j + 1] + cb[j + w] +
                                 cb[j + w + 1] + bias) >> 2;
                    uint8_t vv = (cr[j] + cr[j + 1] + cr[j + w] +
                                  cr[j + w + 1] + bias) >> 2;

                    o = &src[r * (w * 3 / 2) + 3 * (c / 2)];
                    o[0] = r & 1 ? vv : u;
                    o[1] = y[i];
                    o[2] = y[i + 1];
                }
                break;
            }
        }
    }
    g_free(y);
    g_free(cb);
    g_free(cr);
    return src;
}

/*
 * The 2D-DMA's TX side for the encoder: the @w x @h source of @half
 * half-bytes per pixel as macroblocks of @mw x @mh, left to right then
 * top to bottom, each row by row, the image padded by repeating its last
 * pixel's bytes and its last row (TRM 8.4.3).
 */
static uint8_t *macroblocks(const uint8_t *src, int w, int h, int half,
                            int mw, int mh, size_t *len)
{
    int pw = ROUND_UP(w, mw), ph = ROUND_UP(h, mh);
    int row = w * half / 2, prow = pw * half / 2, unit = MAX(half / 2, 1);
    uint8_t *pad = g_malloc(prow * ph), *out = g_malloc(prow * ph);
    size_t n = 0;

    if (half == 3) {
        unit = 3;
    }
    for (int r = 0; r < ph; r++) {
        const uint8_t *s = src + MIN(r, h - 1) * row;
        uint8_t *d = pad + r * prow;

        memcpy(d, s, row);
        for (int b = row; b < prow; b += unit) {
            memcpy(d + b, s + row - unit, unit);
        }
    }
    for (int my = 0; my < ph; my += mh) {
        for (int mx = 0; mx < pw; mx += mw) {
            for (int r = 0; r < mh; r++) {
                memcpy(out + n, pad + (my + r) * prow + mx * half / 2,
                       mw * half / 2);
                n += mw * half / 2;
            }
        }
    }
    g_free(pad);
    *len = n;
    return out;
}

static void put16(GByteArray *b, int v)
{
    uint8_t x[2] = { v >> 8, v };

    g_byte_array_append(b, x, 2);
}

static void put8(GByteArray *b, int v)
{
    uint8_t x = v;

    g_byte_array_append(b, &x, 1);
}

/*
 * The headers ESP-IDF's encoder writes before the codec's output
 * (jpeg_encode.c's s_jpeg_set_header_info()): SOI, APP0, DQT, SOF0, the
 * annex K DHTs, SOS.
 */
static void enc_headers(GByteArray *b, int w, int h, int sub,
                        uint32_t q[2][64])
{
    int nc = sub == SUB_GRAY ? 1 : 3;
    static const uint8_t app0[] = {
        0xff, 0xe0, 0, 16, 'J', 'F', 'I', 'F', 0, 1, 1, 0, 0, 1, 0, 1, 0, 0,
    };
    int h0 = sub == SUB_422 || sub == SUB_420 ? 2 : 1;
    int v0 = sub == SUB_420 ? 2 : 1;

    put16(b, 0xffd8);
    g_byte_array_append(b, app0, sizeof(app0));
    for (int t = 0; t < (nc == 3 ? 2 : 1); t++) {
        put16(b, 0xffdb);
        put16(b, 67);
        put8(b, t);
        for (int i = 0; i < 64; i++) {
            put8(b, q[t][zigzag[i]]);
        }
    }
    put16(b, 0xffc0);
    put16(b, 8 + 3 * nc);
    put8(b, 8);
    put16(b, h);
    put16(b, w);
    put8(b, nc);
    for (int i = 0; i < nc; i++) {
        put8(b, i + 1);
        put8(b, i ? 0x11 : (h0 << 4 | v0));
        put8(b, i > 0);
    }
    for (int t = 0; t < (nc == 3 ? 2 : 1); t++) {
        int n = 0;

        put16(b, 0xffc4);
        put16(b, 2 + 17 + 12);
        put8(b, t);
        g_byte_array_append(b, std_dc_bits[t], 16);
        g_byte_array_append(b, std_dc_vals, 12);
        for (int i = 0; i < 16; i++) {
            n += std_ac_bits[t][i];
        }
        put16(b, 0xffc4);
        put16(b, 2 + 17 + n);
        put8(b, 0x10 | t);
        g_byte_array_append(b, std_ac_bits[t], 16);
        g_byte_array_append(b, std_ac_vals[t], n);
    }
    put16(b, 0xffda);
    put16(b, 6 + 2 * nc);
    put8(b, nc);
    for (int i = 0; i < nc; i++) {
        put8(b, i + 1);
        put8(b, i ? 0x11 : 0x00);
    }
    put8(b, 0);
    put8(b, 63);
    put8(b, 0);
}

/*
 * jpeg_encoder_process()'s register writes: soft reset, encoder mode, the
 * source format and sampling, the size, the pixel order, EOI and 0xFF
 * stuffing, 8-bit tables, and the tables through the FIFO ports
 * (jpeg_hal_set_quantization_coefficient()).
 */
static void enc_setup(QTestState *qts, const EncCfg *e, int w, int h,
                      uint32_t q[2][64])
{
    uint32_t cfg;

    soft_rst(qts);
    cfg_set(qts, MODE_DECODE, 0);
    wr(qts, EXTD_CONFIG, e->src == SRC_YUV444 ? 1 :
                         e->src == SRC_YUV420 ? 3 : 0);
    cfg = rd(qts, CONFIG) & ~(COLOR_SPACE(3) | SAMPLE_SEL(3) | PIXEL_REV |
                              QNR_PRECISION | LQNR_SEL(3) | CQNR_SEL(3));
    cfg |= COLOR_SPACE(e->src == SRC_RGB888 ? 0 : e->src == SRC_YUV422 ? 1 :
                       e->src == SRC_RGB565 ? 2 :
                       e->src == SRC_GRAY ? 3 : 0);
    if (e->sub != SUB_GRAY) {
        cfg |= SAMPLE_SEL(e->sub);
    }
    if (e->pixel_rev) {
        cfg |= PIXEL_REV;
    }
    cfg |= TAILER_EN | FF_CHECK_EN | QNR_FIFO_EN | LQNR_SEL(0) | CQNR_SEL(1);
    wr(qts, CONFIG, cfg);
    wr(qts, PIC_SIZE, w << 16 | h);
    for (int t = 0; t < 4; t++) {
        wr(qts, DQT_INFO, (rd(qts, DQT_INFO) & ~(0xffu << (8 * t))) |
                          (uint32_t)t << (8 * t));
        for (int i = 0; i < 64; i++) {
            wr(qts, TQNR(t), q[t & 1][i]);
        }
    }
}

/*
 * Encode @rgb with the codec as @e says, and check the file ESP-IDF
 * would make of its output against cjpeg's: both decoded by djpeg must
 * be the same image, exactly when @exact. Otherwise, where the codec
 * down-samples in a way cjpeg cannot (a YUV422 source's rows), a
 * difference of one level in a chroma sample can move a coefficient a
 * quantisation step: the two must be within 30 dB of each other and
 * equally far from the source.
 */
static void encode_check(const EncCfg *e, const Img *rgb, bool exact)
{
    g_autofree char *cjpeg = find_tool("cjpeg");
    g_autofree char *djpeg = cjpeg ? find_tool("djpeg") : NULL;
    g_autofree char *dir = NULL;
    g_autofree char *in_pnm = NULL, *ref_jpg = NULL, *ref_pnm = NULL;
    g_autofree char *out_jpg = NULL, *out_pnm = NULL, *qfile = NULL;
    g_autofree uint8_t *src = NULL, *mbs = NULL, *scan = NULL;
    g_autoptr(GByteArray) file = g_byte_array_new();
    g_autoptr(GString) qtxt = g_string_new(NULL);
    g_autoptr(Img) ref = NULL, padded = NULL, a = NULL, b = NULL;
    int mw = sub_mw[e->sub], mh = sub_mh[e->sub], w = rgb->w, h = rgb->h;
    unsigned tx_left, rx, flags;
    uint32_t q[2][64];
    QTestState *qts;
    size_t len;
    double psnr, psnr_codec, psnr_cjpeg;
    int diff;
    const char *sample = e->sub == SUB_420 ? "2x2" :
                         e->sub == SUB_422 ? "2x1" : "1x1";

    if (!djpeg) {
        return;
    }
    dir = g_dir_make_tmp("esp32p4-jpeg-XXXXXX", NULL);
    g_assert(dir);
    quant_tables(e->quality, q);
    src = make_source(e, rgb, &ref);
    mbs = macroblocks(src, w, h, src_half[e->src], mw, mh, &len);
    g_assert_cmpuint(len, <=, MEM_IN_SIZE);

    qts = jpeg_init();
    enc_setup(qts, e, w, h, q);
    wr(qts, INT_ENA, I_RLE_PARALLEL_ERR | I_EN_EOF_ERR);
    dma_tx(qts, MEM_IN, mbs, len, true);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    cfg_set(qts, 0, JPEG_START);
    g_assert(wait_int(qts, I_DONE, 20000000));
    g_assert_cmphex(rd(qts, INT_RAW) & ~(I_OUT_EOF | I_DCT_DONE), ==,
                    I_DONE | I_BS_LAST_BLOCK);
    dma_status(qts, &tx_left, &rx, &flags);
    g_assert_cmpuint(tx_left, ==, 0);
    g_assert_cmpuint(flags, ==, DMA_EOF);
    g_assert_cmpuint(rx, >, 2);
    scan = g_malloc(rx);
    qtest_memread(qts, MEM_OUT, scan, rx);
    qtest_quit(qts);
    g_assert_cmphex(scan[rx - 2], ==, 0xff);
    g_assert_cmphex(scan[rx - 1], ==, 0xd9);

    enc_headers(file, w, h, e->sub, q);
    g_byte_array_append(file, scan, rx);
    out_jpg = g_build_filename(dir, "codec.jpg", NULL);
    out_pnm = g_build_filename(dir, "codec.pnm", NULL);
    g_assert(g_file_set_contents(out_jpg, (const char *)file->data,
                                 file->len, NULL));

    padded = img_pad(ref, ROUND_UP(w, mw), ROUND_UP(h, mh));
    in_pnm = g_build_filename(dir, "in.pnm", NULL);
    ref_jpg = g_build_filename(dir, "ref.jpg", NULL);
    ref_pnm = g_build_filename(dir, "ref.pnm", NULL);
    qfile = g_build_filename(dir, "q.txt", NULL);
    write_pnm(in_pnm, padded);
    for (int t = 0; t < 2; t++) {
        for (int i = 0; i < 64; i++) {
            g_string_append_printf(qtxt, "%u%c", q[t][i],
                                   i % 8 == 7 ? '\n' : ' ');
        }
    }
    g_assert(g_file_set_contents(qfile, qtxt->str, -1, NULL));
    {
        const char *argv[] = {
            cjpeg, "-dct", "int", "-baseline", "-qtables", qfile,
            "-qslots", "0,1,1", "-sample", sample, "-outfile", ref_jpg,
            in_pnm, NULL,
        };

        run_tool(argv);
    }
    {
        const char *argv1[] = {
            djpeg, "-dct", "int", "-nosmooth", "-outfile", ref_pnm, ref_jpg,
            NULL,
        };
        const char *argv2[] = {
            djpeg, "-dct", "int", "-nosmooth", "-outfile", out_pnm, out_jpg,
            NULL,
        };

        run_tool(argv1);
        run_tool(argv2);
    }
    a = read_pnm(out_pnm);
    b = read_pnm(ref_pnm);
    g_assert_cmpint(a->w, ==, w);
    g_assert_cmpint(a->h, ==, h);
    diff = img_diff(a, b, w, h, &psnr);
    img_diff(a, ref, w, h, &psnr_codec);
    img_diff(b, ref, w, h, &psnr_cjpeg);
    if (g_test_verbose()) {
        g_test_message("%dx%d src %d sub %d q%d: %zu bytes, max diff %d "
                       "(%.1f dB) from cjpeg's; from the source %.1f dB, "
                       "cjpeg's %.1f dB", w, h, e->src, e->sub, e->quality,
                       (size_t)rx, diff, psnr, psnr_codec, psnr_cjpeg);
    }
    if (exact) {
        g_assert_cmpint(diff, ==, 0);
    } else {
        g_assert_cmpfloat(psnr, >, 30.0);
        g_assert_cmpfloat(psnr_codec, >, psnr_cjpeg - 0.5);
    }

    unlink(in_pnm);
    unlink(ref_jpg);
    unlink(ref_pnm);
    unlink(out_jpg);
    unlink(out_pnm);
    unlink(qfile);
    rmdir(dir);
}

/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_encode(const void *data)
{
    const EncCfg *e = data;
    static const int sizes[][2] = { { 64, 48 }, { 50, 34 } };

    for (int i = 0; i < ARRAY_SIZE(sizes); i++) {
        int w = sizes[i][0], h = sizes[i][1];
        g_autoptr(Img) im = NULL;

        /* The 2D-DMA pads only whole pixel pairs of the YUV sources. */
        if ((e->src == SRC_YUV422 || e->src == SRC_YUV420) && w % 16) {
            continue;
        }
        im = make_image(w, h, e->src == SRC_GRAY ? 1 : 3, w * 7 + h);
        encode_check(e, im, !(e->src == SRC_YUV422 && e->sub == SUB_420));
        if (g_test_failed()) {
            return;
        }
    }
}

static const EncCfg enc_cases[] = {
    { SRC_RGB888, SUB_444, 90, false },
    { SRC_RGB888, SUB_422, 75, false },
    { SRC_RGB888, SUB_420, 50, false },
    { SRC_RGB888, SUB_420, 95, true },
    { SRC_RGB565, SUB_420, 80, false },
    { SRC_RGB565, SUB_444, 80, true },
    { SRC_GRAY, SUB_GRAY, 85, false },
    { SRC_GRAY, SUB_GRAY, 10, false },
    { SRC_YUV444, SUB_444, 90, false },
    { SRC_YUV444, SUB_420, 70, true },
    { SRC_YUV422, SUB_422, 80, false },
    { SRC_YUV422, SUB_422, 80, true },
    { SRC_YUV422, SUB_420, 80, false },
    { SRC_YUV420, SUB_420, 60, false },
};

/* Decoding. */

/* What ESP-IDF's jpeg_parse_marker() takes from a file's headers. */
typedef struct JpgInfo {
    int w, h, nc;
    int ci[3], hi[3], vi[3], tq[3];
    uint32_t qt[4][64];         /* natural order */
    bool qseen[4];
    uint8_t bits[2][2][16];     /* [DC, AC][table] */
    uint8_t vals[2][2][256];
    bool dht;
    int ri;
    size_t sos;                 /* the offset of the SOS marker */
} JpgInfo;

static int be16(const uint8_t *p)
{
    return p[0] << 8 | p[1];
}

static bool parse_jpeg(const uint8_t *d, size_t len, JpgInfo *j)
{
    size_t p = 2;

    memset(j, 0, sizeof(*j));
    g_assert(d[0] == 0xff && d[1] == 0xd8);
    while (p + 4 <= len) {
        int m, l;

        if (d[p] != 0xff) {
            return false;
        }
        m = d[p + 1];
        if (m == 0xff) {
            p++;
            continue;
        }
        l = be16(d + p + 2);
        if (m == 0xda) {
            j->sos = p;
            return true;
        } else if (m == 0xdb) {
            for (size_t q = p + 4; q < p + 2 + l;) {
                int pq = d[q] >> 4, tq = d[q] & 15;

                q++;
                for (int i = 0; i < 64; i++) {
                    j->qt[tq][zigzag[i]] = pq ? be16(d + q + 2 * i) : d[q + i];
                }
                q += pq ? 128 : 64;
                j->qseen[tq] = true;
            }
        } else if (m == 0xc0) {
            j->h = be16(d + p + 5);
            j->w = be16(d + p + 7);
            j->nc = d[p + 9];
            for (int i = 0; i < j->nc; i++) {
                j->ci[i] = d[p + 10 + 3 * i];
                j->hi[i] = d[p + 11 + 3 * i] >> 4;
                j->vi[i] = d[p + 11 + 3 * i] & 15;
                j->tq[i] = d[p + 12 + 3 * i];
            }
        } else if (m >= 0xc1 && m <= 0xcf && m != 0xc4 && m != 0xc8 &&
                   m != 0xcc) {
            return false;
        } else if (m == 0xc4) {
            for (size_t q = p + 4; q < p + 2 + l;) {
                int tc = d[q] >> 4, th = d[q] & 15, n = 0;

                memcpy(j->bits[tc][th], d + q + 1, 16);
                for (int i = 0; i < 16; i++) {
                    n += d[q + 1 + i];
                }
                memcpy(j->vals[tc][th], d + q + 17, n);
                q += 17 + n;
            }
            j->dht = true;
        } else if (m == 0xdd) {
            j->ri = be16(d + p + 4);
        }
        p += 2 + l;
    }
    return false;
}

/* jpeg_hal_create_minicode_tbl(). */
static void minicode(const uint8_t bits[16], uint32_t huffmin[16])
{
    uint32_t tmp[256], hc = 0;
    int total = 0, j = 0;

    for (int i = 0; i < 16; i++, hc <<= 1) {
        if (!bits[i]) {
            huffmin[i] = 0xffff;
            continue;
        }
        total += bits[i];
        for (int b = 0; b < bits[i]; b++) {
            tmp[j++] = hc++;
        }
        huffmin[i] = tmp[total - bits[i]] << (15 - i);
    }
}

/*
 * jpeg_dht_*_write_codeword() and _write_value(): the counts summed and
 * the first codes interleaved, then the symbols, through the FIFO ports.
 */
static void load_dht(QTestState *qts, int ac, int t, const uint8_t bits[16],
                     const uint8_t *vals)
{
    uint32_t huffmin[16], sum = 0;

    minicode(bits, huffmin);
    for (int i = 0; i < 16; i++) {
        sum += bits[i];
        wr(qts, ac ? DHT_TOTLEN_AC(t) : DHT_TOTLEN_DC(t), sum);
        wr(qts, ac ? DHT_CODEMIN_AC(t) : DHT_CODEMIN_DC(t), huffmin[i]);
    }
    for (int i = 0; i < (ac ? 256 : 16); i++) {
        wr(qts, ac ? DHT_VAL_AC(t) : DHT_VAL_DC(t), vals[i]);
    }
}

static int jpg_format(const JpgInfo *j)
{
    if (j->nc == 1) {
        return SUB_GRAY;
    }
    return j->hi[0] == 1 ? SUB_444 : j->vi[0] == 1 ? SUB_422 : SUB_420;
}

/*
 * jpeg_decoder_process()'s register writes up to the start: soft reset,
 * decoder mode, the size cleared ("digital issue"), the quantisation
 * tables with JPEG_Tn_DQT_INFO set to n, the size rounded up to whole
 * MCUs, the components, the four Huffman tables (annex K's where the
 * file has none) and the restart interval.
 */
static void dec_setup(QTestState *qts, const JpgInfo *j)
{
    int f = jpg_format(j);
    int pw = ROUND_UP(j->w, sub_mw[f]), ph = ROUND_UP(j->h, sub_mh[f]);
    uint8_t bits[2][2][16], vals[2][2][256] = { 0 };

    soft_rst(qts);
    cfg_set(qts, 0, MODE_DECODE);
    wr(qts, PIC_SIZE, 0);
    for (int t = 0; t < 4; t++) {
        if (!j->qseen[t]) {
            continue;
        }
        wr(qts, DQT_INFO, (rd(qts, DQT_INFO) & ~(0xffu << (8 * t))) |
                          (uint32_t)t << (8 * t));
        for (int i = 0; i < 64; i++) {
            wr(qts, TQNR(t), j->qt[t][i]);
        }
    }
    wr(qts, PIC_SIZE, pw << 16 | ph);
    wr(qts, DECODE_CONF, (rd(qts, DECODE_CONF) & ~0xff0000u) | j->nc << 16);
    for (int i = 0; i < j->nc; i++) {
        wr(qts, COMP(i), j->ci[i] << 16 | j->hi[i] << 12 | j->vi[i] << 8 |
                         j->tq[i]);
    }
    if (j->dht) {
        memcpy(bits, j->bits, sizeof(bits));
        memcpy(vals, j->vals, sizeof(vals));
    } else {
        for (int t = 0; t < 2; t++) {
            memcpy(bits[0][t], std_dc_bits[t], 16);
            memcpy(vals[0][t], std_dc_vals, 12);
            memcpy(bits[1][t], std_ac_bits[t], 16);
            memcpy(vals[1][t], std_ac_vals[t], 162);
        }
    }
    for (int t = 0; t < 2; t++) {
        load_dht(qts, 0, t, bits[0][t], vals[0][t]);
    }
    for (int t = 0; t < 2; t++) {
        load_dht(qts, 1, t, bits[1][t], vals[1][t]);
    }
    wr(qts, DECODE_CONF, (rd(qts, DECODE_CONF) & ~0xffffu) | j->ri);
}

/*
 * The decoder's output, MCU by MCU in TRM figure 8.4-4's order, as the
 * image a JFIF decoder makes of it without smoothing: the chroma
 * replicated over its samples, converted by JFIF's formulas.
 */
static Img *dec_image(const uint8_t *out, int w, int h, int f)
{
    int mw = sub_mw[f], mh = sub_mh[f], pw = ROUND_UP(w, mw);
    int ph = ROUND_UP(h, mh), ny = (mw / 8) * (mh / 8);
    int cw = pw / (mw / 8), ch = ph / (mh / 8);
    g_autofree uint8_t *y = g_malloc(pw * ph), *u = g_malloc(cw * ch);
    g_autofree uint8_t *v = g_malloc(cw * ch);
    Img *im = img_new(w, h, f == SUB_GRAY ? 1 : 3);
    const uint8_t *p = out;

    for (int my = 0; my < ph / mh; my++) {
        for (int mx = 0; mx < pw / mw; mx++) {
            for (int b = 0; b < ny; b++, p += 64) {
                int bx = mx * mw + (b % (mw / 8)) * 8;
                int by = my * mh + (b / (mw / 8)) * 8;

                for (int r = 0; r < 8; r++) {
                    memcpy(y + (by + r) * pw + bx, p + 8 * r, 8);
                }
            }
            if (f == SUB_GRAY) {
                continue;
            }
            for (int k = 0; k < 2; k++, p += 64) {
                for (int r = 0; r < 8; r++) {
                    memcpy((k ? v : u) + (my * 8 + r) * cw + mx * 8,
                           p + 8 * r, 8);
                }
            }
        }
    }
    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            int ci = (r / (mh / 8)) * cw + c / (mw / 8);

            if (f == SUB_GRAY) {
                im->px[r * w + c] = y[r * pw + c];
            } else {
                ycc_rgb(y[r * pw + c], u[ci], v[ci], &im->px[3 * (r * w + c)]);
            }
        }
    }
    return im;
}

/*
 * Decode @file with the codec as ESP-IDF's driver does, its bitstream
 * from SOS with the descriptor's suc_eof clear, and return the codec's
 * image; NULL if the decoder did not finish.
 */
static Img *decode_file(const uint8_t *file, size_t len, uint32_t *int_raw,
                        int64_t *ns)
{
    JpgInfo j;
    QTestState *qts;
    unsigned rx, flags;
    g_autofree uint8_t *out = NULL;
    size_t out_len;
    int64_t t0;
    Img *im = NULL;
    int f;

    g_assert(parse_jpeg(file, len, &j));
    f = jpg_format(&j);
    out_len = (size_t)ROUND_UP(j.w, sub_mw[f]) * ROUND_UP(j.h, sub_mh[f]) *
              (f == SUB_GRAY ? 2 : f == SUB_444 ? 6 : f == SUB_422 ? 4 : 3) /
              2;
    g_assert_cmpuint(out_len, <=, MEM_OUT_SIZE);
    g_assert_cmpuint(len - j.sos, <=, MEM_IN_SIZE);

    qts = jpeg_init();
    dec_setup(qts, &j);
    wr(qts, INT_ENA, 0x1fffffe);
    dma_tx(qts, MEM_IN, file + j.sos, len - j.sos, false);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    t0 = now(qts);
    cfg_set(qts, 0, JPEG_START);
    if (wait_int(qts, I_DONE, 20000000)) {
        if (ns) {
            *ns = now(qts) - t0;
        }
        dma_status(qts, NULL, &rx, &flags);
        g_assert_cmpuint(rx, ==, out_len);
        g_assert_cmpuint(flags, ==, DMA_EOF);
        out = g_malloc(out_len);
        qtest_memread(qts, MEM_OUT, out, out_len);
        im = dec_image(out, j.w, j.h, f);
    }
    if (int_raw) {
        *int_raw = rd(qts, INT_RAW);
    }
    qtest_quit(qts);
    return im;
}

/* djpeg's decoding of @path: the reference. */
static Img *djpeg_image(const char *djpeg, const char *dir, const char *path)
{
    g_autofree char *pnm = g_build_filename(dir, "djpeg.pnm", NULL);
    const char *argv[] = {
        djpeg, "-dct", "int", "-nosmooth", "-outfile", pnm, path, NULL,
    };
    Img *im;

    run_tool(argv);
    im = read_pnm(pnm);
    unlink(pnm);
    return im;
}

typedef struct DecCase {
    const char *tool;           /* "cjpeg" or "ffmpeg" */
    const char *opt;            /* cjpeg's -sample or ffmpeg's -pix_fmt */
    int quality;
    const char *restart;        /* cjpeg's -restart, or NULL */
    int w, h;
    bool refused;               /* sampling the decoder does not have */
} DecCase;

/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_decode(const void *data)
{
    const DecCase *d = data;
    bool gray = !strcmp(d->opt, "gray");
    g_autofree char *enc = find_tool(d->tool);
    g_autofree char *djpeg = enc ? find_tool("djpeg") : NULL;
    g_autofree char *dir = NULL, *in = NULL, *jpg = NULL, *qs = NULL;
    g_autofree char *size = NULL, *file = NULL;
    g_autoptr(Img) src = NULL, ref = NULL, got = NULL;
    uint32_t int_raw;
    double psnr;
    size_t len;
    int diff;

    if (!djpeg) {
        return;
    }
    dir = g_dir_make_tmp("esp32p4-jpeg-XXXXXX", NULL);
    in = g_build_filename(dir, strcmp(d->tool, "ffmpeg") ? "in.pnm"
                                                          : "in.rgb", NULL);
    jpg = g_build_filename(dir, "in.jpg", NULL);
    qs = g_strdup_printf("%d", d->quality);
    src = make_image(d->w, d->h, gray && strcmp(d->tool, "ffmpeg") ? 1 : 3,
                     d->w + d->quality);
    if (!strcmp(d->tool, "ffmpeg")) {
        const char *argv[] = {
            enc, "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", size = g_strdup_printf("%dx%d", d->w, d->h), "-i", in,
            "-frames:v", "1", "-c:v", "mjpeg", "-pix_fmt", d->opt, "-q:v", qs,
            jpg, NULL,
        };

        g_assert(g_file_set_contents(in, (const char *)src->px,
                                     d->w * d->h * 3, NULL));
        run_tool(argv);
    } else {
        const char *argv[] = {
            enc, "-dct", "int", "-quality", qs, "-sample", d->opt,
            d->restart ? "-restart" : "-baseline",
            d->restart ? d->restart : "-baseline", "-outfile", jpg, in, NULL,
        };

        if (gray) {
            argv[5] = "-grayscale";
            argv[6] = "-baseline";
        }
        write_pnm(in, src);
        run_tool(argv);
    }
    g_assert(g_file_get_contents(jpg, &file, &len, NULL));
    ref = djpeg_image(djpeg, dir, jpg);
    got = decode_file((const uint8_t *)file, len, &int_raw, NULL);
    if (d->refused) {
        /* Not started: no data unit, no interrupt (TRM table 37.5-5). */
        g_assert(!got);
        g_assert_cmphex(int_raw, ==, 0);
        unlink(in);
        unlink(jpg);
        rmdir(dir);
        return;
    }
    g_assert(got);
    g_assert_cmphex(int_raw & ~(I_DCT_DONE | I_DONE), ==, 0);
    g_assert_cmpint(got->nc, ==, ref->nc);
    diff = img_diff(got, ref, d->w, d->h, &psnr);
    if (g_test_verbose()) {
        g_test_message("%s %s q%d %dx%d: %zu bytes, max diff %d from djpeg's",
                       d->tool, d->opt, d->quality, d->w, d->h, len, diff);
    }
    g_assert_cmpint(diff, ==, 0);
    unlink(in);
    unlink(jpg);
    rmdir(dir);
}

static const DecCase dec_cases[] = {
    { "cjpeg", "1x1", 90, NULL, 64, 48 },
    { "cjpeg", "2x1", 75, NULL, 64, 48 },
    { "cjpeg", "2x2", 75, NULL, 64, 48 },
    { "cjpeg", "2x2", 98, NULL, 37, 29 },
    { "cjpeg", "2x1", 40, NULL, 37, 29 },
    { "cjpeg", "1x1", 60, "2B", 37, 29 },
    { "cjpeg", "2x2", 85, "1", 64, 48 },
    { "cjpeg", "2x2", 85, "1B", 48, 32 },
    { "cjpeg", "gray", 80, NULL, 64, 48 },
    { "cjpeg", "gray", 20, "3B", 41, 23 },
    { "ffmpeg", "yuvj420p", 2, NULL, 64, 48 },
    /*
     * ffmpeg codes 4:2:2 as Y 2x2 and chroma 1x2, and 4:4:4 with every
     * component 1x2: neither is a format of TRM table 37.5-5.
     */
    { "ffmpeg", "yuvj422p", 4, NULL, 64, 48, true },
    { "ffmpeg", "yuvj444p", 3, NULL, 48, 40, true },
    { "ffmpeg", "yuvj420p", 10, NULL, 37, 29 },
};

/* Registers and tables. */

/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_reset_values(void)
{
    QTestState *qts = jpeg_init();
    static const struct {
        uint32_t off, reset;
    } regs[] = {
        { CONFIG, 0x00408958 }, { DQT_INFO, 0x03020100 },
        { PIC_SIZE, 0x028001e0 }, { EXTD_CONFIG, 0 },
        { DECODE_CONF, 0x5f030000 }, { COMP(0), 0x1100 },
        { COMP(1), 0x1100 }, { COMP(2), 0x1100 }, { COMP(3), 0x1100 },
        { DHT_INFO, 0x1010 }, { INT_RAW, 0 }, { INT_ENA, 0 }, { INT_ST, 0 },
        { INT_CLR, 0 }, { STATUS0, 0 }, { STATUS2, 0x08000000 },
        { DECODER_STATUS5, 0 }, { ECO_LOW, 0 }, { ECO_HIGH, 0xffffffff },
        { SYS, 0 }, { VERSION, 0x02412260 },
    };

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(rd(qts, regs[i].off), ==, regs[i].reset);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_access(void)
{
    QTestState *qts = jpeg_init();

    /* CONFIG: the WT bits read 0, DMA_LINKLIST_MODE is 1 whatever. */
    wr(qts, CONFIG, 0xffffffff & ~(JPEG_START | FSM_RST | SOFT_RST |
                                   FIFO_RST));
    g_assert_cmphex(rd(qts, CONFIG), ==, 0xfcffffbc | (1u << 6));
    wr(qts, CONFIG, 0);
    g_assert_cmphex(rd(qts, CONFIG), ==, 1u << 6);
    /* The R/W fields' widths. */
    wr(qts, EXTD_CONFIG, 0xffffffff);
    g_assert_cmphex(rd(qts, EXTD_CONFIG), ==, 3);
    wr(qts, DECODE_CONF, 0);
    g_assert_cmphex(rd(qts, DECODE_CONF), ==, 1u << 24);
    wr(qts, DECODE_CONF, 0xffffffff);
    g_assert_cmphex(rd(qts, DECODE_CONF), ==, 0x7fffffff);
    wr(qts, COMP(2), 0xffffffff);
    g_assert_cmphex(rd(qts, COMP(2)), ==, 0x00ffffff);
    wr(qts, DHT_INFO, 0xffffffff);
    g_assert_cmphex(rd(qts, DHT_INFO), ==, 0xffff);
    wr(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0x01ffffff);
    wr(qts, DECODER_STATUS5, 0xffffffff);
    g_assert_cmphex(rd(qts, DECODER_STATUS5), ==, 1u << 30);
    wr(qts, SYS, 0xffffffff);
    g_assert_cmphex(rd(qts, SYS), ==, 1u << 31);
    wr(qts, VERSION, 0xffffffff);
    g_assert_cmphex(rd(qts, VERSION), ==, 0x0fffffff);
    wr(qts, ECO_LOW, 0x12345678);
    g_assert_cmphex(rd(qts, ECO_LOW), ==, 0x12345678);
    /* Byte writes merge into their register. */
    qtest_writeb(qts, JPEG + PIC_SIZE + 2, 0x40);
    g_assert_cmphex(rd(qts, PIC_SIZE), ==, 0x024001e0);
    /* INT_RAW is not software's to write; INT_ST and the status are RO. */
    wr(qts, INT_RAW, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    wr(qts, INT_ST, 0xffffffff);
    wr(qts, STATUS2, 0);
    g_assert_cmphex(rd(qts, STATUS2), ==, 0x08000000);
    qtest_quit(qts);
}

/*
 * The quantisation tables as jpeg_ll_write_quantization_coefficient_tN()
 * writes them, through JPEG_TnQNR_REG, and read at their addresses.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_qnr_tables(void)
{
    QTestState *qts = jpeg_init();

    soft_rst(qts);
    g_assert(rd(qts, CONFIG) & QNR_FIFO_EN);
    for (int t = 0; t < 4; t++) {
        for (int i = 0; i < 64; i++) {
            wr(qts, TQNR(t), t * 1000 + i + 1);
        }
    }
    for (int t = 0; t < 4; t++) {
        for (int i = 0; i < 64; i++) {
            g_assert_cmpuint(rd(qts, QNR_RAM(t, i)), ==, t * 1000 + i + 1);
        }
    }
    /* The FIFO port is write-only; past entry 63 it wraps to 0. */
    g_assert_cmphex(rd(qts, TQNR(0)), ==, 0);
    wr(qts, TQNR(1), 77);
    g_assert_cmpuint(rd(qts, QNR_RAM(1, 0)), ==, 77);
    /* JPEG_SOFT_RST and JPEG_FIFO_RST restart the port, keep the table. */
    wr(qts, TQNR(2), 55);
    soft_rst(qts);
    wr(qts, TQNR(2), 66);
    g_assert_cmpuint(rd(qts, QNR_RAM(2, 0)), ==, 66);
    g_assert_cmpuint(rd(qts, QNR_RAM(2, 1)), ==, 2002);
    wr(qts, TQNR(3), 1);
    cfg_set(qts, 0, FIFO_RST);
    wr(qts, TQNR(3), 9);        /* dropped: the FIFOs are held */
    cfg_set(qts, FIFO_RST, 0);
    wr(qts, TQNR(3), 2);
    g_assert_cmpuint(rd(qts, QNR_RAM(3, 0)), ==, 2);
    g_assert_cmpuint(rd(qts, QNR_RAM(3, 1)), ==, 3002);
    /* In FIFO mode the addresses do not take writes; outside it they do. */
    wr(qts, QNR_RAM(0, 5), 999);
    g_assert_cmpuint(rd(qts, QNR_RAM(0, 5)), ==, 6);
    cfg_set(qts, QNR_FIFO_EN, 0);
    wr(qts, QNR_RAM(0, 5), 999);
    g_assert_cmpuint(rd(qts, QNR_RAM(0, 5)), ==, 999);
    wr(qts, TQNR(0), 444);      /* dropped: not in FIFO mode */
    g_assert_cmpuint(rd(qts, QNR_RAM(0, 0)), ==, 1);
    qtest_quit(qts);
}

/* The Huffman tables as jpeg_hal_config_*_table() writes them. */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_dht_tables(void)
{
    QTestState *qts = jpeg_init();
    uint8_t vals[256];

    soft_rst(qts);
    for (int t = 0; t < 2; t++) {
        uint32_t huffmin[16], sum = 0;

        memset(vals, 0, sizeof(vals));
        memcpy(vals, std_dc_vals, 12);
        load_dht(qts, 0, t, std_dc_bits[t], vals);
        memcpy(vals, std_ac_vals[t], 162);
        load_dht(qts, 1, t, std_ac_bits[t], vals);

        minicode(std_dc_bits[t], huffmin);
        for (int l = 0; l < 16; l++) {
            sum += std_dc_bits[t][l];
            g_assert_cmpuint(rd(qts, TOTLEN_RAM(0, t, l)), ==, sum);
            g_assert_cmphex(rd(qts, CODEMIN_RAM(0, t, l)), ==, huffmin[l]);
        }
        for (int i = 0; i < 12; i++) {
            g_assert_cmpuint(rd(qts, VAL_DC_RAM(t, i)), ==, std_dc_vals[i]);
        }
        sum = 0;
        minicode(std_ac_bits[t], huffmin);
        for (int l = 0; l < 16; l++) {
            sum += std_ac_bits[t][l];
            g_assert_cmpuint(rd(qts, TOTLEN_RAM(1, t, l)), ==, sum);
            g_assert_cmphex(rd(qts, CODEMIN_RAM(1, t, l)), ==, huffmin[l]);
        }
        for (int i = 0; i < 162; i++) {
            g_assert_cmpuint(rd(qts, VAL_AC_RAM(t, i)), ==,
                             std_ac_vals[t][i]);
        }
    }
    /* Outside FIFO mode the tables take writes at their addresses. */
    cfg_set(qts, DHT_FIFO_EN, 0);
    wr(qts, CODEMIN_RAM(1, 1, 3), 0xabcd);
    wr(qts, VAL_AC_RAM(1, 200), 0x5a);
    wr(qts, DHT_VAL_AC(1), 0x11);       /* dropped */
    g_assert_cmphex(rd(qts, CODEMIN_RAM(1, 1, 3)), ==, 0xabcd);
    g_assert_cmphex(rd(qts, VAL_AC_RAM(1, 200)), ==, 0x5a);
    g_assert_cmphex(rd(qts, VAL_AC_RAM(1, 0)), ==, std_ac_vals[1][0]);
    qtest_quit(qts);
}

/* The bus clock gate and the block reset (HP_SYS_CLKRST). */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_clock_reset(void)
{
    QTestState *qts = jpeg_init();

    wr(qts, ECO_LOW, 0x1234);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) & ~JPEG_SYS_CLK_EN);
    wr(qts, ECO_LOW, 0x5678);
    g_assert_cmphex(rd(qts, ECO_LOW), ==, 0x1234);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | JPEG_SYS_CLK_EN);
    soft_rst(qts);
    wr(qts, TQNR(0), 42);
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) | RST_EN_JPEG);
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) & ~RST_EN_JPEG);
    g_assert_cmphex(rd(qts, ECO_LOW), ==, 0);
    g_assert_cmphex(rd(qts, QNR_RAM(0, 0)), ==, 0);
    qtest_quit(qts);
}

/* Streams made by the test, for the decoder's error paths. */

typedef struct Bits {
    GByteArray *b;
    uint32_t acc;
    int n;
} Bits;

static void bits_put(Bits *w, uint32_t v, int n)
{
    for (int i = n - 1; i >= 0; i--) {
        w->acc = w->acc << 1 | ((v >> i) & 1);
        if (++w->n == 8) {
            put8(w->b, w->acc);
            if ((w->acc & 0xff) == 0xff) {
                put8(w->b, 0);
            }
            w->acc = 0;
            w->n = 0;
        }
    }
}

static void bits_pad(Bits *w)
{
    while (w->n) {
        bits_put(w, 1, 1);
    }
}

/* T.81 C.2's code of symbol @sym in the table @bits / @vals. */
static void huff_code(const uint8_t *bits, const uint8_t *vals, int sym,
                      uint32_t *code, int *len)
{
    uint32_t c = 0;
    int k = 0;

    for (int l = 1; l <= 16; l++, c <<= 1) {
        for (int i = 0; i < bits[l - 1]; i++, k++, c++) {
            if (vals[k] == sym) {
                *code = c;
                *len = l;
                return;
            }
        }
    }
    g_assert_not_reached();
}

/* A data unit: DC difference @dc (category <= 2), then EOB. */
static void put_unit(Bits *w, int t, int dc)
{
    uint32_t code;
    int len, cat = dc == 0 ? 0 : abs(dc) == 1 ? 1 : 2;

    huff_code(std_dc_bits[t], std_dc_vals, cat, &code, &len);
    bits_put(w, code, len);
    if (cat) {
        bits_put(w, dc > 0 ? dc : dc + (1 << cat) - 1, cat);
    }
    huff_code(std_ac_bits[t], std_ac_vals[t], 0x00, &code, &len);
    bits_put(w, code, len);
}

/* A scan header of @ns components, IDs 1.., tables 0 (Y) and 1. */
static void put_sos(GByteArray *b, int ns)
{
    put16(b, 0xffda);
    put16(b, 6 + 2 * ns);
    put8(b, ns);
    for (int i = 0; i < ns; i++) {
        put8(b, i + 1);
        put8(b, i ? 0x11 : 0x00);
    }
    put8(b, 0);
    put8(b, 63);
    put8(b, 0);
}

/*
 * The scan of a 16x16 image, @sub GRAY (four MCUs) or YUV420 (one), each
 * data unit a DC step and EOB, with a restart marker every @ri MCUs, the
 * first @mcus of them, then EOI where @eoi.
 */
static GByteArray *make_scan(int sub, int mcus, int ri, bool eoi)
{
    GByteArray *b = g_byte_array_new();
    Bits w = { .b = b };

    put_sos(b, sub == SUB_GRAY ? 1 : 3);
    for (int m = 0; m < mcus; m++) {
        if (ri && m && !(m % ri)) {
            bits_pad(&w);
            put16(b, 0xffd0 + ((m / ri - 1) & 7));
        }
        for (int u = 0; u < (sub == SUB_GRAY ? 1 : 6); u++) {
            put_unit(&w, u >= 4 ? 1 : 0, u ? 0 : (m & 1 ? -2 : 1));
        }
    }
    bits_pad(&w);
    if (eoi) {
        put16(b, 0xffd9);
    }
    return b;
}

/*
 * The decoder set up for @sub at 16x16 as ESP-IDF's driver does, with
 * annex K's tables and quantisation tables of 1s, all its interrupts
 * enabled, an RX buffer, and @scan from SOS as its TX data.
 */
static QTestState *dec_scan_init(int sub, const GByteArray *scan, bool eof,
                                 int ri)
{
    QTestState *qts = jpeg_init();
    JpgInfo j = { .w = 16, .h = 16, .nc = sub == SUB_GRAY ? 1 : 3, .ri = ri };

    for (int i = 0; i < j.nc; i++) {
        j.ci[i] = i + 1;
        j.hi[i] = !i && sub == SUB_420 ? 2 : 1;
        j.vi[i] = j.hi[i];
        j.tq[i] = i > 0;
    }
    for (int t = 0; t < 2; t++) {
        j.qseen[t] = true;
        for (int i = 0; i < 64; i++) {
            j.qt[t][i] = 1;
        }
    }
    dec_setup(qts, &j);
    wr(qts, INT_ENA, 0x1fffffe);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    if (scan) {
        dma_tx(qts, MEM_IN, scan->data, scan->len, eof);
    }
    return qts;
}

typedef struct ErrCase {
    const char *name;
    int sub;
    int mcus;                   /* MCUs in the stream */
    int stream_ri;              /* its restart interval */
    int reg_ri;                 /* JPEG_RESTART_INTERVAL */
    bool eoi;
    bool eof;
    uint32_t expect;            /* INT_RAW, less DCT_DONE and OUT_EOF */
    unsigned flags;             /* the RX stream's end */
} ErrCase;

static const ErrCase err_cases[] = {
    /* The good streams, for reference. */
    { "good-gray", SUB_GRAY, 4, 0, 0, true, false, I_DONE, DMA_EOF },
    { "good-420", SUB_420, 1, 0, 0, true, false, I_DONE, DMA_EOF },
    { "good-restart", SUB_GRAY, 4, 1, 1, true, false, I_DONE, DMA_EOF },
    /* A restart marker JPEG_RESTART_INTERVAL 0 does not expect. */
    { "rst-unexpected", SUB_GRAY, 4, 1, 0, true, false, I_RST_UXP_ERR, 0 },
    /* A restart marker inside an interval. */
    { "rst-position", SUB_GRAY, 4, 2, 3, true, false, I_RST_CHECK_POS, 0 },
    /* No restart marker where the interval ends. */
    { "rst-missing", SUB_GRAY, 4, 0, 1, true, false, I_RST_CHECK_NONE, 0 },
    /* EOI two MCUs early. */
    { "eoi-early", SUB_GRAY, 2, 0, 0, true, false, I_DE_EOF_ERR, 0 },
    /* The stream's end without EOI, every data unit decoded. */
    { "eoi-missing", SUB_GRAY, 4, 0, 0, false, true,
      I_DE_EOF_LACK | I_DONE, DMA_EOF },
    /* The stream's end two MCUs early. */
    { "data-short", SUB_GRAY, 2, 0, 0, false, true, I_DE_EOF_ERR, 0 },
};

/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_decode_error(const void *data)
{
    const ErrCase *e = data;
    g_autoptr(GByteArray) scan = make_scan(e->sub, e->mcus, e->stream_ri,
                                           e->eoi);
    QTestState *qts = dec_scan_init(e->sub, scan, e->eof, e->reg_ri);
    unsigned flags;

    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(rd(qts, INT_RAW) & ~(I_DCT_DONE | I_OUT_EOF), ==,
                    e->expect);
    g_assert_cmphex(!!(rd(qts, INT_RAW) & I_OUT_EOF), ==, e->eof);
    dma_status(qts, NULL, NULL, &flags);
    g_assert_cmpuint(flags, ==, e->flags);
    g_assert_cmphex(rd(qts, INT_ST), ==, rd(qts, INT_RAW) & 0x1fffffe);
    qtest_quit(qts);
}

/*
 * Scan headers the decoder refuses (TRM 37.5.2.1, 37.6): each error with
 * JPEG_MARKER_ERR_FST_SCAN_INT, and nothing decoded.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_scan_header_errors(void)
{
    static const struct {
        uint32_t reg, val;      /* a register changed from ESP-IDF's */
        int ls_delta;           /* Ls changed by */
        int ns;                 /* the scan's components */
        uint32_t expect;
    } cases[] = {
        { COMP(0), 2 << 16 | 0x1100, 0, 1, I_CID_ERR },
        { DHT_INFO, 0x1032, 0, 1, I_DHT_DC_ID_ERR },
        { DHT_INFO, 0x3210, 0, 1, I_DHT_AC_ID_ERR },
        { COMP(0), 1 << 16 | 0x1105, 0, 1, I_DQT_ID_ERR },
        { 0, 0, 1, 1, I_SOS_UNMATCH },
        { 0, 0, 0, 3, I_CID_ERR },
    };

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autoptr(GByteArray) scan = g_byte_array_new();
        QTestState *qts = dec_scan_init(SUB_GRAY, NULL, false, 0);
        unsigned rx;

        put_sos(scan, cases[i].ns);
        scan->data[3] += cases[i].ls_delta;
        put16(scan, 0xffd9);
        if (cases[i].reg) {
            wr(qts, cases[i].reg, cases[i].val);
        }
        dma_tx(qts, MEM_IN, scan->data, scan->len, false);
        cfg_set(qts, 0, JPEG_START);
        qtest_clock_step(qts, 100000);
        g_assert_cmphex(rd(qts, INT_RAW), ==,
                        cases[i].expect | I_MARKER_ERR_FST);
        dma_status(qts, NULL, &rx, NULL);
        g_assert_cmpuint(rx, ==, 0);
        qtest_quit(qts);
    }
}

/* Markers where the decoder has no use for them (TRM 37.6). */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_marker_errors(void)
{
    static const uint8_t soi[] = { 0xff, 0xd8, 0xff, 0xe0, 0, 4, 0, 0 };
    static const uint8_t junk[] = { 0x12, 0x34, 0x56 };
    QTestState *qts;
    g_autoptr(GByteArray) scan = make_scan(SUB_GRAY, 4, 0, false);

    /* The file's start where SOS should be: SOI is not the decoder's. */
    qts = dec_scan_init(SUB_GRAY, NULL, false, 0);
    dma_tx(qts, MEM_IN, soi, sizeof(soi), false);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, I_UXP_DET);
    qtest_quit(qts);

    /* The bitstream ends with no SOS in it. */
    qts = dec_scan_init(SUB_GRAY, NULL, false, 0);
    dma_tx(qts, MEM_IN, junk, sizeof(junk), true);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, I_UNDET | I_OUT_EOF);
    qtest_quit(qts);

    /* A second scan where EOI should be. */
    put_sos(scan, 1);
    qts = dec_scan_init(SUB_GRAY, scan, false, 0);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW) & ~I_DCT_DONE, ==, I_SCAN_CHECK_POS);
    qtest_quit(qts);

    /* More data than JPEG_HA x JPEG_VA's data units. */
    qts = dec_scan_init(SUB_GRAY, scan, false, 0);
    wr(qts, PIC_SIZE, 16 << 16 | 8);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW) & ~I_DCT_DONE, ==, I_DE_EOF_ERR);
    qtest_quit(qts);
}

/*
 * A run that carries an AC coefficient past the data unit's end
 * (JPEG_RLE_PARALLEL_ERR_INT), and a code no table entry matches: the
 * frame decodes on, its end marked err_eof.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_data_errors(void)
{
    for (int bad = 0; bad < 2; bad++) {
        g_autoptr(GByteArray) b = g_byte_array_new();
        Bits w = { .b = b };
        QTestState *qts;
        uint32_t code;
        unsigned flags;
        int len;

        put_sos(b, 1);
        if (bad) {
            /* Sixteen 1s: longer than any luminance DC code. */
            bits_put(&w, 0xffff, 16);
            huff_code(std_ac_bits[0], std_ac_vals[0], 0, &code, &len);
            bits_put(&w, code, len);
        } else {
            /*
             * DC 0, then four runs of 15 zeros each ending in a 1: the
             * fourth lands on coefficient 64. The data unit ends there.
             */
            huff_code(std_dc_bits[0], std_dc_vals, 0, &code, &len);
            bits_put(&w, code, len);
            for (int i = 0; i < 4; i++) {
                huff_code(std_ac_bits[0], std_ac_vals[0], 0xf1, &code, &len);
                bits_put(&w, code, len);
                bits_put(&w, 1, 1);
            }
        }
        for (int m = 1; m < 4; m++) {
            put_unit(&w, 0, 0);
        }
        bits_pad(&w);
        put16(b, 0xffd9);
        qts = dec_scan_init(SUB_GRAY, b, false, 0);
        cfg_set(qts, 0, JPEG_START);
        qtest_clock_step(qts, 100000);
        g_assert_cmphex(rd(qts, INT_RAW) & ~I_DCT_DONE, ==,
                        I_DONE | (bad ? 0 : I_RLE_PARALLEL_ERR));
        dma_status(qts, NULL, NULL, &flags);
        g_assert_cmpuint(flags, ==, DMA_ERR_EOF);
        qtest_quit(qts);
    }
}

/* Configurations the decoder has no format for are not started. */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_decode_refused(void)
{
    g_autoptr(GByteArray) scan = make_scan(SUB_GRAY, 4, 0, true);
    static const struct {
        int ncomp;
        uint32_t c0;
    } cases[] = {
        { 0, 0x1100 }, { 2, 0x1100 }, { 4, 0x1100 }, { 1, 0x2100 },
        { 3, 0x1200 },
    };

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        QTestState *qts = dec_scan_init(SUB_GRAY, scan, false, 0);

        wr(qts, DECODE_CONF, (rd(qts, DECODE_CONF) & ~0xff0000u) |
                             cases[i].ncomp << 16);
        wr(qts, COMP(0), 1 << 16 | cases[i].c0);
        cfg_set(qts, 0, JPEG_START);
        qtest_clock_step(qts, 100000);
        g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
        qtest_quit(qts);
    }
}

/* The encoder's frame ends against JPEG_HA x JPEG_VA's data units. */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_encode_eof(void)
{
    static const uint8_t px[4 * 64] = { 0 };
    uint32_t q[2][64];
    EncCfg e = { SRC_GRAY, SUB_GRAY, 50, false };
    QTestState *qts;
    unsigned rx, flags;

    quant_tables(50, q);

    /* The 2D-DMA's frame ends after one of four MCUs. */
    qts = jpeg_init();
    enc_setup(qts, &e, 16, 16, q);
    dma_tx(qts, MEM_IN, px, 64, true);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, I_EN_EOF_ERR | I_OUT_EOF);
    dma_status(qts, NULL, &rx, &flags);
    g_assert_cmpuint(flags, ==, 0);
    qtest_quit(qts);

    /* Every MCU but no end of frame from the 2D-DMA. */
    qts = jpeg_init();
    enc_setup(qts, &e, 16, 16, q);
    dma_tx(qts, MEM_IN, px, sizeof(px), false);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW) & ~I_DCT_DONE, ==,
                    I_EN_EOF_LACK | I_BS_LAST_BLOCK | I_DONE);
    dma_status(qts, NULL, &rx, &flags);
    g_assert_cmpuint(flags, ==, DMA_EOF);
    qtest_quit(qts);
}

/* Timing. */

/* Step to @t cycles after @t0 and say whether INT_RAW has @bit. */
static bool raw_at(QTestState *qts, int64_t t0, int64_t cycles, uint32_t bit)
{
    int64_t t = now(qts);

    if (t0 + cycles * CYCLE_NS > t) {
        qtest_clock_step(qts, t0 + cycles * CYCLE_NS - t);
    }
    return rd(qts, INT_RAW) & bit;
}

/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_timing(void)
{
    static const uint8_t px[4 * 64] = { 0 };
    uint32_t q[2][64];
    EncCfg e = { SRC_GRAY, SUB_GRAY, 50, false };
    g_autoptr(GByteArray) scan = make_scan(SUB_420, 1, 0, true);
    QTestState *qts;
    int64_t t0;

    quant_tables(50, q);

    /* Encoding four grey MCUs, a data unit each. */
    qts = jpeg_init();
    enc_setup(qts, &e, 16, 16, q);
    dma_tx(qts, MEM_IN, px, sizeof(px), true);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    t0 = now(qts);
    cfg_set(qts, 0, JPEG_START);
    g_assert(!raw_at(qts, t0, UNIT - 1, I_DCT_DONE));
    g_assert(raw_at(qts, t0, UNIT + 1, I_DCT_DONE));
    g_assert(!raw_at(qts, t0, 4 * UNIT - 1, I_DONE | I_BS_LAST_BLOCK));
    g_assert(raw_at(qts, t0, 4 * UNIT + 1, I_DONE));
    g_assert(rd(qts, INT_RAW) & I_BS_LAST_BLOCK);
    qtest_quit(qts);

    /* Decoding one YUV420 MCU: six data units. */
    qts = dec_scan_init(SUB_420, scan, false, 0);
    t0 = now(qts);
    cfg_set(qts, 0, JPEG_START);
    g_assert(!raw_at(qts, t0, 6 * UNIT - 1, I_DONE));
    g_assert(raw_at(qts, t0, 6 * UNIT + 1, I_DONE));
    qtest_quit(qts);

    /* The clock stopped mid-frame holds the count. */
    qts = jpeg_init();
    enc_setup(qts, &e, 16, 16, q);
    dma_tx(qts, MEM_IN, px, sizeof(px), true);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    t0 = now(qts);
    cfg_set(qts, 0, JPEG_START);
    raw_at(qts, t0, 150, 0);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) & ~JPEG_SYS_CLK_EN);
    qtest_clock_step(qts, 1000000);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | JPEG_SYS_CLK_EN);
    t0 += 1000000;
    g_assert(!raw_at(qts, t0, 4 * UNIT - 2, I_DONE));
    g_assert(raw_at(qts, t0, 4 * UNIT + 2, I_DONE));
    qtest_quit(qts);
}

/*
 * JPEG_DECODE_TIMEOUT_THRES: a decoder with no bitstream times out after
 * 2^THRES - 1 cycles; with TASK_SEL it resets itself, without it goes on
 * when the data comes.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_decode_timeout(void)
{
    g_autoptr(GByteArray) scan = make_scan(SUB_GRAY, 4, 0, true);

    for (int sel = 0; sel < 2; sel++) {
        QTestState *qts = dec_scan_init(SUB_GRAY, NULL, false, 0);
        int64_t t0;

        cfg_set(qts, TMO_THRES(63) | TMO_TASK_SEL,
                TMO_THRES(10) | (sel ? TMO_TASK_SEL : 0));
        t0 = now(qts);
        cfg_set(qts, 0, JPEG_START);
        g_assert(!raw_at(qts, t0, 1021, I_DECODE_TIMEOUT));
        g_assert(raw_at(qts, t0, 1024, I_DECODE_TIMEOUT));
        wr(qts, INT_CLR, I_DECODE_TIMEOUT);
        cfg_set(qts, TMO_THRES(63), TMO_THRES(32));
        dma_tx(qts, MEM_IN, scan->data, scan->len, false);
        qtest_clock_step(qts, 100000);
        if (sel) {
            /* Reset to idle: the data waits for a start. */
            g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
            cfg_set(qts, 0, JPEG_START);
            qtest_clock_step(qts, 100000);
        }
        g_assert_cmphex(rd(qts, INT_RAW) & ~I_DCT_DONE, ==, I_DONE);
        qtest_quit(qts);
    }
}

/*
 * The output: JPEG_PAUSE_EN holds it, and a full RX side stops the
 * pipeline once the pack FIFO has 64 bytes; JPEG_FSM_RST abandons the
 * frame.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_output_flow(void)
{
    uint32_t q[2][64];
    EncCfg e = { SRC_GRAY, SUB_GRAY, 100, false };
    g_autoptr(Img) im = make_image(64, 64, 1, 3);
    g_autofree uint8_t *mbs = NULL;
    QTestState *qts;
    unsigned rx, rx2, flags;
    size_t len;

    quant_tables(100, q);
    mbs = macroblocks(im->px, 64, 64, 2, 8, 8, &len);

    qts = jpeg_init();
    enc_setup(qts, &e, 64, 64, q);
    cfg_set(qts, 0, PAUSE_EN);
    dma_tx(qts, MEM_IN, mbs, len, true);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 2000000);
    dma_status(qts, NULL, &rx, NULL);
    g_assert_cmpuint(rx, ==, 0);
    g_assert(!(rd(qts, INT_RAW) & I_DONE));
    cfg_set(qts, PAUSE_EN, 0);
    g_assert(wait_int(qts, I_DONE, 2000000));
    dma_status(qts, NULL, &rx, &flags);
    g_assert_cmpuint(rx, >, 64 * 64 / 4);
    g_assert_cmpuint(flags, ==, DMA_EOF);
    qtest_quit(qts);

    /* No RX buffer: an MCU or so done, then the pipeline waits. */
    qts = jpeg_init();
    enc_setup(qts, &e, 64, 64, q);
    dma_tx(qts, MEM_IN, mbs, len, true);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 2000000);
    g_assert(rd(qts, INT_RAW) & I_DCT_DONE);
    g_assert(!(rd(qts, INT_RAW) & I_BS_LAST_BLOCK));
    dma_rx(qts, MEM_OUT, 100);
    qtest_clock_step(qts, 2000000);
    dma_status(qts, NULL, &rx, NULL);
    g_assert_cmpuint(rx, ==, 100);
    g_assert(!(rd(qts, INT_RAW) & I_DONE));
    dma_rx(qts, MEM_OUT + 100, MEM_OUT_SIZE - 100);
    g_assert(wait_int(qts, I_DONE, 2000000));

    /* JPEG_FSM_RST mid-frame: nothing more comes. */
    wr(qts, INT_CLR, 0x1ffffff);
    dma(qts, "reset");
    dma_tx(qts, MEM_IN, mbs, len, true);
    dma_rx(qts, MEM_OUT, MEM_OUT_SIZE);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 5000);
    cfg_set(qts, 0, FSM_RST);
    dma_status(qts, NULL, &rx, NULL);
    qtest_clock_step(qts, 2000000);
    dma_status(qts, NULL, &rx2, NULL);
    g_assert_cmpuint(rx2, ==, rx);
    g_assert(!(rd(qts, INT_RAW) & I_DONE));
    qtest_quit(qts);
}

/* JPEG_INTR, interrupt matrix source 95, follows INT_ST. */
/* [spec:nuos:req:emu.esp32p4.jpeg/test] */
static void test_irq(void)
{
    g_autoptr(GByteArray) scan = make_scan(SUB_GRAY, 4, 0, true);
    QTestState *qts = dec_scan_init(SUB_GRAY, scan, false, 0);
    uint32_t bit = 1u << (SRC_JPEG % 32);

    wr(qts, INT_ENA, 0);
    cfg_set(qts, 0, JPEG_START);
    qtest_clock_step(qts, 100000);
    g_assert(rd(qts, INT_RAW) & I_DONE);
    g_assert_cmphex(rd(qts, INT_ST), ==, 0);
    g_assert(!(qtest_readl(qts, INTMTX_STATUS(SRC_JPEG / 32)) & bit));
    wr(qts, INT_ENA, I_DONE);
    g_assert_cmphex(rd(qts, INT_ST), ==, I_DONE);
    g_assert(qtest_readl(qts, INTMTX_STATUS(SRC_JPEG / 32)) & bit);
    wr(qts, INT_CLR, I_DONE);
    g_assert_cmphex(rd(qts, INT_RAW) & I_DONE, ==, 0);
    g_assert(!(qtest_readl(qts, INTMTX_STATUS(SRC_JPEG / 32)) & bit));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/jpeg/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/jpeg/access", test_access);
    qtest_add_func("/esp32p4/jpeg/qnr-tables", test_qnr_tables);
    qtest_add_func("/esp32p4/jpeg/dht-tables", test_dht_tables);
    qtest_add_func("/esp32p4/jpeg/clock-reset", test_clock_reset);
    for (int i = 0; i < ARRAY_SIZE(err_cases); i++) {
        g_autofree char *name = g_strdup_printf(
            "/esp32p4/jpeg/decode-stream/%s", err_cases[i].name);

        qtest_add_data_func(name, &err_cases[i], test_decode_error);
    }
    qtest_add_func("/esp32p4/jpeg/scan-header-errors",
                   test_scan_header_errors);
    qtest_add_func("/esp32p4/jpeg/marker-errors", test_marker_errors);
    qtest_add_func("/esp32p4/jpeg/data-errors", test_data_errors);
    qtest_add_func("/esp32p4/jpeg/decode-refused", test_decode_refused);
    qtest_add_func("/esp32p4/jpeg/encode-eof", test_encode_eof);
    qtest_add_func("/esp32p4/jpeg/timing", test_timing);
    qtest_add_func("/esp32p4/jpeg/decode-timeout", test_decode_timeout);
    qtest_add_func("/esp32p4/jpeg/output-flow", test_output_flow);
    qtest_add_func("/esp32p4/jpeg/irq", test_irq);
    for (int i = 0; i < ARRAY_SIZE(enc_cases); i++) {
        const EncCfg *e = &enc_cases[i];
        g_autofree char *name = g_strdup_printf(
            "/esp32p4/jpeg/encode/src%d-sub%d-q%d%s", e->src, e->sub,
            e->quality, e->pixel_rev ? "-rev" : "");

        qtest_add_data_func(name, e, test_encode);
    }
    for (int i = 0; i < ARRAY_SIZE(dec_cases); i++) {
        const DecCase *d = &dec_cases[i];
        g_autofree char *name = g_strdup_printf(
            "/esp32p4/jpeg/decode/%s-%s-q%d-%dx%d%s%s", d->tool, d->opt,
            d->quality, d->w, d->h, d->restart ? "-restart" : "",
            d->restart ? d->restart : "");

        qtest_add_data_func(name, d, test_decode);
    }
    return g_test_run();
}
