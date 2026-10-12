/*
 * ESP32-P4 pixel-processing accelerator (PPA)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 39
 * (Pixel-Processing Accelerator), with the layout, access types and reset
 * values of ESP-IDF's ppa_reg.h for v3, its ppa_ll driver and the PPA
 * driver (esp_driver_ppa) and its tests.
 *
 * Two engines, each moving its pixels through the 2D-DMA
 * (include/hw/dma/esp32p4_dma2d.h): SRM, which scales, rotates and
 * mirrors an image block (2D-DMA TX selection 1 in, RX selection 1 out),
 * and BLEND, which blends a foreground over a background (TX selections 2
 * and 3 in, RX selection 2 out) or fills a block with one pixel.
 *
 * Pixels (TRM 39.5.1, table 39.5-1). An input pixel becomes ARGB8888 as
 * figure 39.5-1 has it: RGB swap (ARGB stored B, G, R, A read as
 * A, R, G, B; RGB888 likewise; RGB565 with its R and B fields swapped),
 * then byte swap (bytes 0 and 1, 2 and 3 exchanged; none for RGB888), an
 * RGB565 component widened with zeros below (ESP-IDF's GRAY8 SRM test,
 * run on silicon, expects (pix >> 8) & 0xf8 and so on), GRAY to R = G = B,
 * L4 and L8 through their CLUT (a 4-bit index in the low nibble first),
 * A4 and A8 as alpha with the RGB of BLEND_RGB (A4's nibble a as a * 17,
 * which the TRM leaves unsaid), YUV422 by its byte order (0 to 3: U Y0 V
 * Y1, V Y0 U Y1, Y0 U Y1 V, Y0 V Y1 U, the TRM's table read from address
 * 0, as ESP-IDF's driver has them) with both pixels of a pair sharing its
 * U and V, YUV420 as pairs of rows, U Y0 Y1 on the first and V Y0 Y1 on
 * the second, the four pixels of a 2 x 2 square sharing them. A format
 * without alpha has 255; the alpha is then inverted (*_ALPHA_INV),
 * replaced by *_FIX_ALPHA (*_ALPHA_MOD 1) or scaled to its top 8 bits
 * times it (2), as table 39.5-2 has it.
 *
 * YUV and RGB (TRM 39.5.1.1). YUV in full range becomes limited range,
 * Y' = (220 Y + 4096) >> 8, U' = (225 U + 4096) >> 8 and V' likewise, then
 * RGB by BT.601 (R = (298 Y + 409 V - 56906) >> 8, G = (298 Y - 100 U -
 * 208 V + 34707) >> 8, B = (298 Y + 516 U - 70836) >> 8) or BT.709 (298,
 * 459, -63367; 298, -55, -136, 19681; 298, 541, -73918). RGB becomes
 * limited-range YUV by BT.601 (Y = (4096 + 66 R + 129 G + 25 B) >> 8,
 * U = (32768 - 38 R - 74 G + 112 B) >> 8, V = (32768 + 112 R - 94 G -
 * 18 B) >> 8) or BT.709 (47, 157, 16; -26, -86, 112; 112, -102, -10),
 * then in full range Y' = (298 Y - 4768) >> 8, U' = (291 U - 4550) >> 8
 * and V' likewise. Every result is clamped to 0..255. The TRM gives these
 * coefficients over 256 and no rounding: the model drops the fraction (an
 * arithmetic shift), as the PPA's GRAY conversion does on silicon, and
 * takes the full-range offsets as subtracted, which the TRM's rendering
 * prints as added (black would come out at Y 37). GRAY output is
 * (R * RGB2GRAY_R + G * RGB2GRAY_G + B * RGB2GRAY_B) >> 8 (silicon, by the
 * test above). Output: ARGB8888 as B, G, R, A; RGB888 B, G, R; RGB565 the
 * components' top bits; YUV422 as U Y0 V Y1 (ESP-IDF names the output
 * order UYVY; the TRM's "byte order 1" would be V Y0 U Y1), with the first
 * pixel's U and V; YUV420 pairs as U Y0 Y1 on even rows of the block and
 * V Y0 Y1 on odd ones, the chroma the first pixel's on that row. The
 * chroma subsampling is the model's choice.
 *
 * SRM (TRM 39.5.3). The input image block (the TX descriptor's hb x vb)
 * is taken in macroblocks of 32 x 32 pixels (SR_BK_SIZE_SEL 0; 16 x 16
 * with 1), each fetched through the 2D-DMA's DSCR-PORT with a border, as
 * a 34 x 34 block (36 x 34 for YUV422, 36 x 36 for YUV420; 18 and 20 at
 * 16) starting one pixel (two for YUV) above and left of it, the 2D-DMA
 * padding it beyond the image block with the block's edge. The scaled
 * block is floor(hb x Sx) x floor(vb x Sy), Sx = SR_SCAL_X_INT +
 * SR_SCAL_X_FRAG / 16, each macroblock's part of it a whole number of
 * pixels as ESP-IDF's driver computes it; it is rotated counterclockwise
 * by SR_ROTATE_ANGLE x 90 degrees about its center (ESP-IDF's
 * ppa_srm_basic_data_correctness_check, run on silicon, fixes the
 * direction), then mirrored across X (SR_MIRROR_X) and Y. An output pixel
 * (p, q) of the scaled block samples the input at the pixel centers'
 * correspondence, x = (p + 0.5) / Sx - 0.5, bilinearly between the four
 * pixels around it with weights in 1/256 (the fraction rounded down),
 * each component (A, R, G, B) as
 * ((c00 (256 - fx) + c01 fx) (256 - fy) + (c10 (256 - fx) + c11 fx) fy +
 * 32768) >> 16. The TRM says only "bilinear interpolation": the center
 * alignment follows from the borders' one pixel on each side, the weights'
 * precision and rounding are the model's. Output goes out through the
 * 2D-DMA's DSCR-PORT in blocks of at most 32 x 32 pixels, each written at
 * its place in the output picture.
 *
 * BLEND (TRM 39.5.4). The block is BLEND_HB x BLEND_VB (the background's
 * descriptor's hb x vb while those are 0). Per pixel, with alphas Ab, Af
 * and components Cb, Cf of background and foreground (TRM: Aout = Ab +
 * Af - Ab Af, Cout = (Cb Ab (1 - Af) + Cf Af) / Aout): Ab' = (Ab (255 -
 * Af) + 127) / 255, Aout = Af + Ab', Cout = ((Cf Af + Cb Ab') R + 32768) >>
 * 16 with R = 65536 / Aout (0 for Aout 0), all in whole numbers rounded
 * down. ESP-IDF's ppa_blend_basic_data_correctness_check, run on silicon,
 * gives one pixel (Ab 203, Af 63; B 128/0 to 91, G 64/128 to 83, R 160/128
 * to 150, A 216), which this arithmetic reproduces and simpler roundings
 * do not. Color keying: a pixel is in its layer's key range when each of
 * R, G and B lies within CK_*_LOW..CK_*_HIGH inclusive; foreground in and
 * background out gives the background; background in and foreground out,
 * the background, or with COLORKEY_FG_BG_REVERSE the foreground; both in,
 * COLORKEY_DEFAULT's RGB with the blended alpha (the model's choice).
 * BLEND_BYPASS outputs the background, its alpha replaced by an A4 or A8
 * foreground's. Filling (BLEND_FIX_PIXEL_FILL_EN) outputs BLEND_FIX_PIXEL:
 * as ARGB8888 converted to an RGB output (ESP-IDF's fill test: RGB565 is
 * r >> 3, g >> 2, b >> 3), as the raw bytes of a pixel pair for YUV422
 * (four) and YUV420 (three) and of a pixel for GRAY (one).
 *
 * Timing. Each engine works at its clock (PPA_SYS_CLK), on the 2D-DMA's
 * timers so that it runs in order with the DMA's bus: the SRM computes an
 * output pixel a cycle, once a macroblock's input block is in, with two
 * input blocks and two output blocks buffered (the TRM's ping-pong FIFO);
 * BLEND blends a pixel a cycle and fills 8 bytes a cycle. The TRM gives
 * no rates; ESP-IDF's performance tests bound them from below (35, 31.5
 * and 150 Mpixels/s) through PSRAM.
 *
 * Interrupts: SR_EOF and BLEND_EOF when an engine has given its last
 * pixel to the 2D-DMA; SR_PARAM_CFG_ERR and BLEND_PARAM_CFG_ERR when a
 * start finds the parameters TRM tables 39.6-1 and 39.6-2 list wrong
 * (SR_PARAM_ERR_ST and BLEND_ST say which), and the engine then does not
 * run.
 *
 * The CLUTs (TRM 39.5.1.2), 256 words each, through BLEND0/1_CLUT_DATA
 * (FIFO mode, each access at the next address, BLEND*_CLUT_MEM_RST
 * and _RDADDR_RST resetting the addresses while set) or at 0x400 and
 * 0x800 (memory mode, APB_FIFO_MASK). While BLEND_CLUT_MEM_FORCE_PD holds
 * them powered down (FORCE_PU clear) they read 0, take no writes and lose
 * their contents; the SRM's memory so (SR_MEM_FORCE_PD) reads zeros.
 * DBG_REPLACE_SEL replaces the pixels at its stage with DBG_REPLACE_DATA,
 * as ARGB8888.
 *
 * Not modelled: SR_MACRO_BK_RO_BYPASS and the output blocks' rearranging,
 * which change only how the 2D-DMA writes; the redundancy and ECO
 * registers and SRAM_CTRL, kept as written.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/esp32p4_ppa.h"
#include "hw/gpio/esp32_line.h"
#include "migration/vmstate.h"

#define IN  ESP32P4_DMA2D_IN
#define OUT ESP32P4_DMA2D_OUT

/* Registers. */
#define A_BLEND0_CLUT_DATA  0x000
#define A_BLEND1_CLUT_DATA  0x004
#define A_CLUT_CONF         0x00c
#define A_INT_RAW           0x010
#define A_INT_ST            0x014
#define A_INT_ENA           0x018
#define A_INT_CLR           0x01c
#define A_SR_COLOR_MODE     0x020
#define A_BLEND_COLOR_MODE  0x024
#define A_SR_BYTE_ORDER     0x028
#define A_BLEND_BYTE_ORDER  0x02c
#define A_BLEND_TRANS_MODE  0x034
#define A_SR_FIX_ALPHA      0x038
#define A_BLEND_TX_SIZE     0x03c
#define A_BLEND_FIX_ALPHA   0x040
#define A_BLEND_RGB         0x048
#define A_BLEND_FIX_PIXEL   0x04c
#define A_CK_FG_LOW         0x050
#define A_CK_FG_HIGH        0x054
#define A_CK_BG_LOW         0x058
#define A_CK_BG_HIGH        0x05c
#define A_CK_DEFAULT        0x060
#define A_SR_SCAL_ROTATE    0x064
#define A_SR_MEM_PD         0x068
#define A_REG_CONF          0x06c
#define A_CLUT_CNT          0x070
#define A_BLEND_ST          0x074
#define A_SR_PARAM_ERR_ST   0x078
#define A_SR_STATUS         0x07c
#define A_ECO_LOW           0x080
#define A_ECO_HIGH          0x084
#define A_ECO_CELL_CTRL     0x088
#define A_SRAM_CTRL         0x08c
#define A_DEBUG_CTRL0       0x090
#define A_DEBUG_CTRL1       0x094
#define A_RGB2GRAY          0x098
#define A_DATE              0x100
#define A_CLUT0_MEM         0x400
#define A_CLUT1_MEM         0x800
#define A_CLUT_MEM_END      0xc00

#define R(a)                ((a) / 4)

/* Interrupts. */
#define INT_SR_EOF          BIT(0)
#define INT_BLEND_EOF       BIT(1)
#define INT_SR_PARAM_ERR    BIT(2)
#define INT_BLEND_PARAM_ERR BIT(3)

/* CLUT_CONF. */
#define APB_FIFO_MASK       BIT(0)
#define CLUT_MEM_RST(i)     BIT(1 + (i))
#define CLUT_RDADDR_RST(i)  BIT(3 + (i))
#define CLUT_FORCE_PD       BIT(5)
#define CLUT_FORCE_PU       BIT(6)

/* BLEND_TRANS_MODE. */
#define BLEND_EN            BIT(0)
#define BLEND_BYPASS        BIT(1)
#define BLEND_FILL_EN       BIT(2)
#define BLEND_UPDATE        BIT(3)
#define BLEND_RST           BIT(4)

/* SR_SCAL_ROTATE. */
#define SCAL_ROTATE_RST     BIT(26)
#define SCAL_ROTATE_START   BIT(27)

/* SR_MEM_PD. */
#define SR_MEM_FORCE_PD     BIT(1)
#define SR_MEM_FORCE_PU     BIT(2)

/* The color modes. */
enum {
    CM_ARGB8888 = 0,
    CM_RGB888 = 1,
    CM_RGB565 = 2,
    CM_L8 = 4,
    CM_L4 = 5,
    CM_A8 = 6,
    CM_A4 = 7,
    CM_YUV420 = 8,
    CM_YUV422 = 9,
    CM_GRAY = 12,
};

/* DBG_REPLACE_SEL's stages. */
enum {
    DBG_SRM_IN = 1,
    DBG_SRM_BILIN = 2,
    DBG_SRM_OUT = 3,
    DBG_BLEND_FG = 4,
    DBG_BLEND_BG = 5,
    DBG_BLEND_OUT = 6,
};

#define DATE_RESET          0x20241220

static const struct {
    uint16_t addr;
    uint32_t mask;
    uint32_t reset;
} ppa_reg_defs[] = {
    { A_BLEND0_CLUT_DATA, 0xffffffff, 0 },
    { A_BLEND1_CLUT_DATA, 0xffffffff, 0 },
    { A_CLUT_CONF, 0xff, 0 },
    { A_INT_ENA, 0xf, 0 },
    { A_SR_COLOR_MODE, 0x3fff, 0 },
    { A_BLEND_COLOR_MODE, 0x3ffff, 0 },
    { A_SR_BYTE_ORDER, 0xf, 0 },
    { A_BLEND_BYTE_ORDER, 0xf, 0 },
    { A_BLEND_TRANS_MODE, 0x77, 0 },
    { A_SR_FIX_ALPHA, 0x7ff, 0x80 },
    { A_BLEND_TX_SIZE, 0xfffffff, 0 },
    { A_BLEND_FIX_ALPHA, 0x3fffff, 0x8080 },
    { A_BLEND_RGB, 0xffffff, 0x808080 },
    { A_BLEND_FIX_PIXEL, 0xffffffff, 0 },
    { A_CK_FG_LOW, 0xffffff, 0xffffff },
    { A_CK_FG_HIGH, 0xffffff, 0 },
    { A_CK_BG_LOW, 0xffffff, 0xffffff },
    { A_CK_BG_HIGH, 0xffffff, 0 },
    { A_CK_DEFAULT, 0x1ffffff, 0 },
    { A_SR_SCAL_ROTATE, 0x37ffffff, 1 | 1 << 12 },
    { A_SR_MEM_PD, 0x7, 0 },
    { A_REG_CONF, 0x1, 0 },
    { A_ECO_LOW, 0xffffffff, 0 },
    { A_ECO_HIGH, 0xffffffff, 0xffffffff },
    { A_ECO_CELL_CTRL, 0x2, 0 },
    { A_SRAM_CTRL, 0x3fff, 4896 },
    { A_DEBUG_CTRL0, 0x7, 0 },
    { A_DEBUG_CTRL1, 0xffffffff, 0 },
    { A_RGB2GRAY, 0xffffff, 85 | 86 << 8 | 85 << 16 },
    { A_DATE, 0xffffffff, DATE_RESET },
};

static int reg_def(hwaddr addr)
{
    for (int i = 0; i < ARRAY_SIZE(ppa_reg_defs); i++) {
        if (ppa_reg_defs[i].addr == addr) {
            return i;
        }
    }
    return -1;
}

static void ppa_update_irq(ESP32P4PpaState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R(A_INT_RAW)] & s->regs[R(A_INT_ENA)]));
}

static void ppa_raise(ESP32P4PpaState *s, uint32_t bits)
{
    s->regs[R(A_INT_RAW)] |= bits;
    ppa_update_irq(s);
}

/* The time @n PPA clock cycles after @t; INT64_MAX while it is stopped. */
static int64_t ppa_after(ESP32P4PpaState *s, int64_t t, uint64_t n)
{
    if (!esp32p4_clk_edges_running(&s->edges)) {
        return INT64_MAX;
    }
    return esp32p4_clk_edges_after(&s->edges, t, n);
}

/* Pixels */

typedef struct PpaFmt {
    unsigned cm;
    bool rgb_swap;
    bool byte_swap;
    unsigned alpha_mod;
    bool alpha_inv;
    uint8_t fix_alpha;
    bool full;
    bool bt709;
    unsigned order;
    const uint32_t *clut;
    uint32_t fix_rgb;
    uint32_t gray;          /* RGB2GRAY */
    unsigned dbg;           /* the stage DBG_REPLACE_SEL replaces, or 0 */
    uint32_t dbg_data;
} PpaFmt;

/* The nibbles of a pixel of color mode @cm; 0 for a reserved mode. */
static unsigned cm_nibbles(unsigned cm)
{
    switch (cm) {
    case CM_ARGB8888:
        return 8;
    case CM_RGB888:
        return 6;
    case CM_RGB565:
    case CM_YUV422:
        return 4;
    case CM_YUV420:
        return 3;
    case CM_L8:
    case CM_A8:
    case CM_GRAY:
        return 2;
    case CM_L4:
    case CM_A4:
        return 1;
    default:
        return 0;
    }
}

static uint32_t argb(uint32_t a, uint32_t r, uint32_t g, uint32_t b)
{
    return a << 24 | r << 16 | g << 8 | b;
}

static uint8_t clamp8(int32_t v)
{
    return MIN(MAX(v, 0), 255);
}

/* @v / 256 rounded down, for negative @v too. */
static int32_t shr8(int32_t v)
{
    return v >= 0 ? v >> 8 : -((-v + 255) >> 8);
}

static uint32_t yuv_to_rgb(const PpaFmt *f, int32_t y, int32_t u, int32_t v)
{
    int32_t r, g, b;

    if (f->full) {
        y = shr8(220 * y + 4096);
        u = shr8(225 * u + 4096);
        v = shr8(225 * v + 4096);
    }
    if (f->bt709) {
        r = shr8(298 * y + 459 * v - 63367);
        g = shr8(298 * y - 55 * u - 136 * v + 19681);
        b = shr8(298 * y + 541 * u - 73918);
    } else {
        r = shr8(298 * y + 409 * v - 56906);
        g = shr8(298 * y - 100 * u - 208 * v + 34707);
        b = shr8(298 * y + 516 * u - 70836);
    }
    return argb(255, clamp8(r), clamp8(g), clamp8(b));
}

static void rgb_to_yuv(const PpaFmt *f, uint32_t px, uint8_t yuv[3])
{
    int32_t r = extract32(px, 16, 8), g = extract32(px, 8, 8);
    int32_t b = extract32(px, 0, 8), y, u, v;

    if (f->bt709) {
        y = shr8(4096 + 47 * r + 157 * g + 16 * b);
        u = shr8(32768 - 26 * r - 86 * g + 112 * b);
        v = shr8(32768 + 112 * r - 102 * g - 10 * b);
    } else {
        y = shr8(4096 + 66 * r + 129 * g + 25 * b);
        u = shr8(32768 - 38 * r - 74 * g + 112 * b);
        v = shr8(32768 + 112 * r - 94 * g - 18 * b);
    }
    if (f->full) {
        y = shr8(298 * y - 4768);
        u = shr8(291 * u - 4550);
        v = shr8(291 * v - 4550);
    }
    yuv[0] = clamp8(y);
    yuv[1] = clamp8(u);
    yuv[2] = clamp8(v);
}

/* The alpha processing of table 39.5-2 on raw alpha @a. */
static uint32_t alpha_process(const PpaFmt *f, uint32_t a)
{
    if (f->alpha_inv) {
        a = 255 - a;
    }
    switch (f->alpha_mod) {
    case 1:
        return f->fix_alpha;
    case 2:
        return (a * f->fix_alpha) >> 8;
    default:
        return a;
    }
}

/* The Y, U and V of pixel @x of a YUV422 row by byte order @order. */
static void yuv422_px(const uint8_t *row, unsigned order, uint32_t x,
                      int32_t *y, int32_t *u, int32_t *v)
{
    const uint8_t *p = row + (x & ~1u) * 2;
    static const uint8_t pos[4][4] = {
        /* Y0, Y1, U, V */
        { 1, 3, 0, 2 }, { 1, 3, 2, 0 }, { 0, 2, 1, 3 }, { 0, 2, 3, 1 },
    };

    *y = p[pos[order][x & 1]];
    *u = p[pos[order][2]];
    *v = p[pos[order][3]];
}

/*
 * Decode pixel @x of a row of format @f into ARGB8888, before the alpha
 * processing: @row the row's bytes, and for YUV420 @even and @odd the
 * row pair's, @row being one of them.
 */
static uint32_t decode_px(const PpaFmt *f, const uint8_t *row,
                          const uint8_t *even, const uint8_t *odd, uint32_t x)
{
    const uint8_t *p;
    uint32_t v, a;
    int32_t yy, uu, vv;

    switch (f->cm) {
    case CM_RGB888: {
        uint8_t b[3];

        p = row + x * 3;
        memcpy(b, p, 3);
        if (f->rgb_swap) {
            uint8_t t = b[0];

            b[0] = b[2];
            b[2] = t;
        }
        return argb(255, b[2], b[1], b[0]);
    }
    case CM_RGB565:
        p = row + x * 2;
        v = p[0] | p[1] << 8;
        if (f->rgb_swap) {
            v = (v & 0x07e0) | (v & 0x1f) << 11 | v >> 11;
        }
        if (f->byte_swap) {
            v = bswap16(v);
        }
        return argb(255, (v >> 11) << 3, ((v >> 5) & 0x3f) << 2,
                    (v & 0x1f) << 3);
    case CM_GRAY:
        return argb(255, row[x], row[x], row[x]);
    case CM_L8:
        return f->clut[row[x]];
    case CM_L4:
        return f->clut[(row[x / 2] >> (4 * (x & 1))) & 0xf];
    case CM_A8:
        return f->fix_rgb | (uint32_t)row[x] << 24;
    case CM_A4:
        a = (row[x / 2] >> (4 * (x & 1))) & 0xf;
        return f->fix_rgb | a * 17 << 24;
    case CM_YUV422:
        yuv422_px(row, f->order, x, &yy, &uu, &vv);
        return yuv_to_rgb(f, yy, uu, vv);
    case CM_YUV420:
        p = row + (x / 2) * 3;
        yy = p[1 + (x & 1)];
        uu = even[(x / 2) * 3];
        vv = odd[(x / 2) * 3];
        return yuv_to_rgb(f, yy, uu, vv);
    default: {
        uint8_t b[4];

        p = row + x * 4;
        memcpy(b, p, 4);
        if (f->rgb_swap) {
            uint8_t t0 = b[0], t1 = b[1];

            b[0] = b[3];
            b[1] = b[2];
            b[2] = t1;
            b[3] = t0;
        }
        if (f->byte_swap) {
            uint8_t t0 = b[0], t2 = b[2];

            b[0] = b[1];
            b[1] = t0;
            b[2] = b[3];
            b[3] = t2;
        }
        return argb(b[3], b[2], b[1], b[0]);
    }
    }
}

/* Decode and alpha-process; the debug replace at the input stage. */
static uint32_t input_px(const PpaFmt *f, unsigned stage, const uint8_t *row,
                         const uint8_t *even, const uint8_t *odd, uint32_t x)
{
    uint32_t px = decode_px(f, row, even, odd, x);

    px = deposit32(px, 24, 8, alpha_process(f, px >> 24));
    return f->dbg == stage ? f->dbg_data : px;
}

/*
 * Encode the ARGB8888 pixels @px[0..n) of row @row of an output block into
 * @out in format @f: the bytes written. A pixel pair of YUV is encoded
 * whole; a lone last pixel pairs with itself.
 */
static uint32_t encode_px(const PpaFmt *f, const uint32_t *px, uint32_t n,
                          uint32_t row, uint8_t *out)
{
    uint32_t o = 0;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t p = px[i];
        uint8_t yuv[3], yuv1[3];

        switch (f->cm) {
        case CM_RGB888:
            out[o++] = p;
            out[o++] = p >> 8;
            out[o++] = p >> 16;
            break;
        case CM_RGB565: {
            uint32_t v = (extract32(p, 16, 8) >> 3) << 11 |
                         (extract32(p, 8, 8) >> 2) << 5 |
                         extract32(p, 0, 8) >> 3;

            out[o++] = v;
            out[o++] = v >> 8;
            break;
        }
        case CM_GRAY: {
            uint32_t g = extract32(p, 16, 8) * extract32(f->gray, 16, 8) +
                         extract32(p, 8, 8) * extract32(f->gray, 8, 8) +
                         extract32(p, 0, 8) * extract32(f->gray, 0, 8);

            out[o++] = MIN(g >> 8, 255);
            break;
        }
        case CM_YUV422:
        case CM_YUV420:
            rgb_to_yuv(f, p, yuv);
            rgb_to_yuv(f, i + 1 < n ? px[i + 1] : p, yuv1);
            if (f->cm == CM_YUV422) {
                out[o++] = yuv[1];
                out[o++] = yuv[0];
                out[o++] = yuv[2];
            } else {
                out[o++] = row & 1 ? yuv[2] : yuv[1];
                out[o++] = yuv[0];
            }
            out[o++] = yuv1[0];
            i++;
            break;
        default:
            out[o++] = p;
            out[o++] = p >> 8;
            out[o++] = p >> 16;
            out[o++] = p >> 24;
            break;
        }
    }
    return o;
}

static void fmt_in(ESP32P4PpaState *s, PpaFmt *f, unsigned cm,
                   bool rgb_swap, bool byte_swap, unsigned alpha_mod,
                   bool alpha_inv, uint8_t fix_alpha, bool full, bool bt709,
                   unsigned order)
{
    memset(f, 0, sizeof(*f));
    f->cm = cm;
    f->rgb_swap = rgb_swap;
    f->byte_swap = byte_swap;
    f->alpha_mod = alpha_mod;
    f->alpha_inv = alpha_inv;
    f->fix_alpha = fix_alpha;
    f->full = full;
    f->bt709 = bt709;
    f->order = order;
}

/* SRM */

/* The SRM's configuration, as latched at its start. */
static void srm_fmt_in(ESP32P4PpaState *s, PpaFmt *f)
{
    uint32_t cm = s->srm_cfg[0], bo = s->srm_cfg[1], fa = s->srm_cfg[2];

    fmt_in(s, f, extract32(cm, 0, 4), bo & BIT(1), bo & BIT(0),
           extract32(fa, 8, 2), fa & BIT(10), fa & 0xff, cm & BIT(8),
           cm & BIT(10), extract32(cm, 12, 2));
    f->dbg = s->srm_dbg[0];
    f->dbg_data = s->srm_dbg[1];
}

static void srm_fmt_out(ESP32P4PpaState *s, PpaFmt *f)
{
    uint32_t cm = s->srm_cfg[0];

    memset(f, 0, sizeof(*f));
    f->cm = extract32(cm, 4, 4);
    f->full = cm & BIT(9);
    f->bt709 = cm & BIT(11);
    f->gray = s->srm_gray;
}

/* The scale factors in sixteenths, X and Y. */
static uint32_t srm_scale(ESP32P4PpaState *s, int axis)
{
    uint32_t v = s->srm_cfg[3];

    return axis ? extract32(v, 12, 8) * 16 + extract32(v, 20, 4)
                : extract32(v, 0, 8) * 16 + extract32(v, 8, 4);
}

/* The pixel block the 2D-DMA gives for a macroblock, and its border. */
static void srm_block(ESP32P4PpaState *s, uint32_t *bw, uint32_t *bh,
                      uint32_t *bl, uint32_t *bt)
{
    unsigned cm = extract32(s->srm_cfg[0], 0, 4);
    uint32_t m = s->srm_mb;

    *bl = cm == CM_YUV422 || cm == CM_YUV420 ? 2 : 1;
    *bt = cm == CM_YUV420 ? 2 : 1;
    *bw = m + 2 * *bl;
    *bh = m + 2 * *bt;
}

/* The scaled block's size, before the rotation. */
static void srm_scaled(ESP32P4PpaState *s, uint32_t *w, uint32_t *h)
{
    *w = (uint64_t)s->srm_hb * srm_scale(s, 0) / 16;
    *h = (uint64_t)s->srm_vb * srm_scale(s, 1) / 16;
}

static bool srm_rotated(ESP32P4PpaState *s)
{
    return extract32(s->srm_cfg[3], 24, 2) & 1;
}

/* Map scaled-block pixel (p, q) to the output block, and back. */
static void srm_forward(ESP32P4PpaState *s, uint32_t p, uint32_t q,
                        uint32_t *u, uint32_t *v)
{
    uint32_t w, h, wo, ho, a, b;

    srm_scaled(s, &w, &h);
    wo = srm_rotated(s) ? h : w;
    ho = srm_rotated(s) ? w : h;
    switch (extract32(s->srm_cfg[3], 24, 2)) {
    case 0:
        a = p;
        b = q;
        break;
    case 1:
        a = q;
        b = w - 1 - p;
        break;
    case 2:
        a = w - 1 - p;
        b = h - 1 - q;
        break;
    default:
        a = h - 1 - q;
        b = p;
        break;
    }
    *u = s->srm_cfg[3] & BIT(28) ? wo - 1 - a : a;
    *v = s->srm_cfg[3] & BIT(29) ? ho - 1 - b : b;
}

static void srm_backward(ESP32P4PpaState *s, uint32_t u, uint32_t v,
                         uint32_t *p, uint32_t *q)
{
    uint32_t w, h, wo, ho, a, b;

    srm_scaled(s, &w, &h);
    wo = srm_rotated(s) ? h : w;
    ho = srm_rotated(s) ? w : h;
    a = s->srm_cfg[3] & BIT(28) ? wo - 1 - u : u;
    b = s->srm_cfg[3] & BIT(29) ? ho - 1 - v : v;
    switch (extract32(s->srm_cfg[3], 24, 2)) {
    case 0:
        *p = a;
        *q = b;
        break;
    case 1:
        *q = a;
        *p = w - 1 - b;
        break;
    case 2:
        *p = w - 1 - a;
        *q = h - 1 - b;
        break;
    default:
        *q = h - 1 - a;
        *p = b;
        break;
    }
}

/*
 * Macroblock @k: its input rectangle, its part of the scaled block, and
 * that part's rectangle in the output block (false if it has none).
 */
static bool srm_mb_rect(ESP32P4PpaState *s, uint32_t k, uint32_t *ox,
                        uint32_t *oy, uint32_t *ow, uint32_t *oh)
{
    uint32_t m = s->srm_mb, nmbx = DIV_ROUND_UP(s->srm_hb, m);
    uint32_t mx = k % nmbx, my = k / nmbx;
    uint32_t sx = srm_scale(s, 0), sy = srm_scale(s, 1);
    uint32_t sx0 = (uint64_t)mx * m * sx / 16;
    uint32_t sx1 = (uint64_t)MIN((mx + 1) * m, s->srm_hb) * sx / 16;
    uint32_t sy0 = (uint64_t)my * m * sy / 16;
    uint32_t sy1 = (uint64_t)MIN((my + 1) * m, s->srm_vb) * sy / 16;
    uint32_t u0, v0, u1, v1;

    if (sx1 <= sx0 || sy1 <= sy0) {
        return false;
    }
    srm_forward(s, sx0, sy0, &u0, &v0);
    srm_forward(s, sx1 - 1, sy1 - 1, &u1, &v1);
    *ox = MIN(u0, u1);
    *oy = MIN(v0, v1);
    *ow = MAX(u0, u1) - *ox + 1;
    *oh = MAX(v0, v1) - *oy + 1;
    return true;
}

/* The output blocks, of at most 32 x 32, of macroblock @k. */
static uint32_t srm_subs(ESP32P4PpaState *s, uint32_t k)
{
    uint32_t ox, oy, ow, oh;

    if (!srm_mb_rect(s, k, &ox, &oy, &ow, &oh)) {
        return 0;
    }
    return DIV_ROUND_UP(ow, 32) * DIV_ROUND_UP(oh, 32);
}

static void srm_idle(ESP32P4PpaState *s)
{
    s->srm_state = 0;
    s->srm_computing = false;
    s->srm_out_num = 0;
    s->srm_stalled = false;
    if (s->dma2d) {
        esp32p4_dma2d_timer_del(s->dma2d, s->srm_timer);
    }
}

/* The parameter checks of TRM table 39.6-1, as SR_PARAM_ERR_ST's bits. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t srm_check(ESP32P4PpaState *s, ESP32P4Dma2dDesc *in,
                          ESP32P4Dma2dDesc *out)
{
    unsigned rx = extract32(s->srm_cfg[0], 0, 4);
    unsigned tx = extract32(s->srm_cfg[0], 4, 4);
    uint32_t w, h, wo, ho, err = 0;

    srm_scaled(s, &w, &h);
    wo = srm_rotated(s) ? h : w;
    ho = srm_rotated(s) ? w : h;
    if (ho + out->y > out->va) {
        err |= BIT(0);
    }
    if (wo + out->x > out->ha) {
        err |= BIT(1);
    }
    if (!srm_scale(s, 1)) {
        err |= BIT(2);
    }
    if (in->vb + in->y > in->va) {
        err |= BIT(3);
    }
    if (!h) {
        err |= BIT(4);
    }
    if (h > 8191) {
        err |= BIT(5);
    }
    if (!srm_scale(s, 0)) {
        err |= BIT(6);
    }
    if (in->hb + in->x > in->ha) {
        err |= BIT(7);
    }
    if (!w) {
        err |= BIT(8);
    }
    if (w > 8191) {
        err |= BIT(9);
    }
    if ((rx == CM_YUV420 || rx == CM_YUV422) &&
        ((in->ha | in->hb | in->x) & 1)) {
        err |= BIT(10);
    }
    if (rx == CM_YUV420 && ((in->va | in->vb | in->y) & 1)) {
        err |= BIT(11);
    }
    if ((tx == CM_YUV420 || tx == CM_YUV422) &&
        ((out->ha | out->hb | out->x) & 1)) {
        err |= BIT(12);
    }
    if (tx == CM_YUV420 && ((out->va | out->vb | out->y) & 1)) {
        err |= BIT(13);
    }
    return err;
}

static void srm_run(ESP32P4PpaState *s);

/* The color modes the SRM takes in and gives out. */
static bool srm_cm_valid(unsigned cm)
{
    return cm_nibbles(cm) && (cm < CM_L8 || cm > CM_A4);
}

/* Begin, once both descriptors are in: check, then go. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void srm_begin(ESP32P4PpaState *s)
{
    ESP32P4Dma2dDesc in, out;
    uint32_t err, bw, bh, bl, bt, pw, ph;

    if (!esp32p4_dma2d_desc(s->dma2d, OUT, ESP32P4_DMA2D_TX_PPA_SRM, &in) ||
        !esp32p4_dma2d_desc(s->dma2d, IN, ESP32P4_DMA2D_RX_PPA_SRM, &out)) {
        return;
    }
    if (!srm_cm_valid(extract32(s->srm_cfg[0], 0, 4)) ||
        !srm_cm_valid(extract32(s->srm_cfg[0], 4, 4))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SR_COLOR_MODE 0x%x selects a "
                      "color mode the SRM does not take\n", TYPE_ESP32P4_PPA,
                      s->srm_cfg[0]);
    }
    s->srm_hb = in.hb;
    s->srm_vb = in.vb;
    err = srm_check(s, &in, &out);
    s->regs[R(A_SR_PARAM_ERR_ST)] = err;
    if (err) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the SRM's parameters are wrong "
                      "(SR_PARAM_ERR_ST 0x%x)\n", TYPE_ESP32P4_PPA, err);
        srm_idle(s);
        ppa_raise(s, INT_SR_PARAM_ERR);
        return;
    }
    srm_block(s, &bw, &bh, &bl, &bt);
    esp32p4_dma2d_port_block(s->dma2d, ESP32P4_DMA2D_TX_PPA_SRM, &pw, &ph);
    if (pw != bw || ph != bh) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the 2D-DMA's DSCR-PORT block is "
                      "%u x %u, the SRM takes %u x %u\n", TYPE_ESP32P4_PPA,
                      pw, ph, bw, bh);
    }
    s->srm_nmb = DIV_ROUND_UP(s->srm_hb, s->srm_mb) *
                 DIV_ROUND_UP(s->srm_vb, s->srm_mb);
    s->srm_req = 0;
    s->srm_comp = 0;
    s->srm_sub = 0;
    s->srm_slot_fill[0] = s->srm_slot_fill[1] = UINT32_MAX;
    s->srm_out_num = 0;
    s->srm_out_head = 0;
    s->srm_computing = false;
    s->srm_busy_until = esp32_line_time();
    s->srm_state = 2;
    srm_run(s);
}

/* The bytes of an input block. */
static uint32_t srm_in_bytes(ESP32P4PpaState *s)
{
    uint32_t bw, bh, bl, bt, pw, ph;

    srm_block(s, &bw, &bh, &bl, &bt);
    esp32p4_dma2d_port_block(s->dma2d, ESP32P4_DMA2D_TX_PPA_SRM, &pw, &ph);
    return MIN(pw * ph * cm_nibbles(extract32(s->srm_cfg[0], 0, 4)) / 2,
               ESP32P4_PPA_SRM_IN_MAX);
}

/*
 * Compute output block @sub of macroblock @k from its input block in slot
 * @slot into @o.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void srm_compute(ESP32P4PpaState *s, uint32_t k, uint32_t sub,
                        int slot, ESP32P4PpaOutBlock *o)
{
    uint32_t m = s->srm_mb, nmbx = DIV_ROUND_UP(s->srm_hb, m);
    uint32_t mx = k % nmbx, my = k / nmbx;
    uint32_t bw, bh, bl, bt, ox, oy, ow, oh, nsx;
    uint32_t sx = srm_scale(s, 0), sy = srm_scale(s, 1);
    uint32_t nib = cm_nibbles(extract32(s->srm_cfg[0], 0, 4));
    int64_t bx0, by0;
    uint32_t stride, px[32];
    bool ram_off = (s->regs[R(A_SR_MEM_PD)] & SR_MEM_FORCE_PD) &&
                   !(s->regs[R(A_SR_MEM_PD)] & SR_MEM_FORCE_PU);
    static const uint8_t zeros[ESP32P4_PPA_SRM_IN_MAX];
    const uint8_t *in = ram_off ? zeros : s->srm_in[slot];
    PpaFmt fi, fo;

    srm_fmt_in(s, &fi);
    srm_fmt_out(s, &fo);
    srm_block(s, &bw, &bh, &bl, &bt);
    stride = bw * nib / 2;
    bx0 = (int64_t)mx * m - bl;
    by0 = (int64_t)my * m - bt;
    srm_mb_rect(s, k, &ox, &oy, &ow, &oh);
    nsx = DIV_ROUND_UP(ow, 32);
    o->x = ox + (sub % nsx) * 32;
    o->y = oy + (sub / nsx) * 32;
    o->w = MIN(32, ox + ow - o->x);
    o->h = MIN(32, oy + oh - o->y);
    o->len = 0;
    o->pushed = 0;
    o->declared = false;
    for (uint32_t r = 0; r < o->h; r++) {
        for (uint32_t c = 0; c < o->w; c++) {
            uint32_t p, q, out = 0;
            int64_t num, x0, y0, lx, ly;
            uint32_t fx, fy, cs[4];

            srm_backward(s, o->x + c, o->y + r, &p, &q);
            /* x = (2p + 1) 16 / (2 Sx) - 1/2, as num / den. */
            num = (int64_t)(2 * p + 1) * 16 - sx;
            x0 = num >= 0 ? num / (2 * sx) : -((-num + 2 * sx - 1) /
                                                (2 * sx));
            fx = (num - x0 * 2 * sx) * 256 / (2 * sx);
            num = (int64_t)(2 * q + 1) * 16 - sy;
            y0 = num >= 0 ? num / (2 * sy) : -((-num + 2 * sy - 1) /
                                                (2 * sy));
            fy = (num - y0 * 2 * sy) * 256 / (2 * sy);
            for (int j = 0; j < 4; j++) {
                lx = MIN(MAX(x0 + (j & 1) - bx0, 0), bw - 1);
                ly = MIN(MAX(y0 + (j >> 1) - by0, 0), bh - 1);
                cs[j] = input_px(&fi, DBG_SRM_IN, in + ly * stride,
                                 in + (ly & ~1) * stride,
                                 in + (ly | 1) * stride, lx);
            }
            for (int ch = 0; ch < 4; ch++) {
                uint32_t c00 = extract32(cs[0], 8 * ch, 8);
                uint32_t c01 = extract32(cs[1], 8 * ch, 8);
                uint32_t c10 = extract32(cs[2], 8 * ch, 8);
                uint32_t c11 = extract32(cs[3], 8 * ch, 8);
                uint32_t top = c00 * (256 - fx) + c01 * fx;
                uint32_t bot = c10 * (256 - fx) + c11 * fx;

                out |= ((top * (256 - fy) + bot * fy + 32768) >> 16) <<
                       (8 * ch);
            }
            if (s->srm_dbg[0] == DBG_SRM_BILIN ||
                s->srm_dbg[0] == DBG_SRM_OUT) {
                out = s->srm_dbg[1];
            }
            px[c] = out;
        }
        o->len += encode_px(&fo, px, o->w, o->y + r, o->data + o->len);
    }
}

/* Push what the output blocks have to the 2D-DMA. */
static void srm_push(ESP32P4PpaState *s)
{
    while (s->srm_out_num) {
        ESP32P4PpaOutBlock *o = &s->srm_out[s->srm_out_head];
        unsigned nib = cm_nibbles(extract32(s->srm_cfg[0], 4, 4));
        size_t n;

        if (!o->declared) {
            if (!esp32p4_dma2d_port_rx(s->dma2d, ESP32P4_DMA2D_RX_PPA_SRM,
                                       o->x, o->y, o->w, o->h, nib,
                                       o->last)) {
                return;
            }
            o->declared = true;
        }
        n = esp32p4_dma2d_push(s->dma2d, ESP32P4_DMA2D_RX_PPA_SRM,
                               o->data + o->pushed, o->len - o->pushed,
                               o->last ? ESP32P4_DMA2D_EOF : 0);
        o->pushed += n;
        if (o->pushed < o->len) {
            return;
        }
        s->srm_out_head ^= 1;
        s->srm_out_num--;
        if (o->last) {
            srm_idle(s);
            ppa_raise(s, INT_SR_EOF);
            return;
        }
    }
}

/* Ask for input blocks, take their bytes, compute, push: what can go on. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void srm_run(ESP32P4PpaState *s)
{
    uint32_t bytes = srm_in_bytes(s);
    int64_t now = esp32_line_time();
    bool progress = true;

    while (progress && s->srm_state == 2) {
        progress = false;
        /* Ask for the next input block, into a free slot. */
        if (s->srm_req < s->srm_nmb) {
            int slot = s->srm_req & 1;

            if (s->srm_slot_fill[slot] == UINT32_MAX) {
                uint32_t m = s->srm_mb, nmbx = DIV_ROUND_UP(s->srm_hb, m);
                uint32_t bw, bh, bl, bt;

                srm_block(s, &bw, &bh, &bl, &bt);
                if (esp32p4_dma2d_port_tx(s->dma2d, ESP32P4_DMA2D_TX_PPA_SRM,
                                          (s->srm_req % nmbx) * m - bl,
                                          (s->srm_req / nmbx) * m - bt,
                                          s->srm_req + 1 == s->srm_nmb)) {
                    s->srm_slot_mb[slot] = s->srm_req++;
                    s->srm_slot_fill[slot] = 0;
                    progress = true;
                } else if (!s->srm_stalled) {
                    s->srm_stalled = true;
                    qemu_log_mask(LOG_GUEST_ERROR, "%s: the 2D-DMA channel "
                                  "serving the SRM takes no pixel block "
                                  "requests (DSCR-PORT off?)\n",
                                  TYPE_ESP32P4_PPA);
                }
            }
        }
        /* The bytes of the block being received. */
        for (int i = 0; i < 2; i++) {
            int slot = (s->srm_comp + i) & 1;
            bool eof;
            size_t n;

            if (s->srm_slot_fill[slot] == UINT32_MAX ||
                s->srm_slot_fill[slot] >= bytes) {
                continue;
            }
            n = esp32p4_dma2d_pull(s->dma2d, ESP32P4_DMA2D_TX_PPA_SRM,
                                   s->srm_in[slot] + s->srm_slot_fill[slot],
                                   bytes - s->srm_slot_fill[slot], &eof);
            s->srm_slot_fill[slot] += n;
            progress |= n > 0;
            break;
        }
        /* Compute the next output block. */
        if (!s->srm_computing && s->srm_comp < s->srm_nmb &&
            s->srm_out_num < 2) {
            int slot = s->srm_comp & 1;
            uint32_t subs = srm_subs(s, s->srm_comp);

            if (s->srm_slot_fill[slot] != UINT32_MAX &&
                s->srm_slot_fill[slot] >= bytes &&
                s->srm_slot_mb[slot] == s->srm_comp) {
                if (s->srm_sub < subs) {
                    ESP32P4PpaOutBlock *o =
                        &s->srm_out[(s->srm_out_head + s->srm_out_num) & 1];
                    uint32_t w, h;

                    srm_compute(s, s->srm_comp, s->srm_sub, slot, o);
                    srm_scaled(s, &w, &h);
                    s->srm_sub++;
                    o->last = false;
                    s->srm_computing = true;
                    s->srm_busy_until = ppa_after(s, MAX(now,
                                                         s->srm_busy_until),
                                                  o->w * o->h);
                    esp32p4_dma2d_timer_mod(s->dma2d, s->srm_timer,
                                            s->srm_busy_until);
                    progress = true;
                }
                if (s->srm_sub >= subs && !s->srm_computing) {
                    /* The macroblock is done with: its slot is free. */
                    s->srm_slot_fill[slot] = UINT32_MAX;
                    s->srm_comp++;
                    s->srm_sub = 0;
                    progress = true;
                }
            }
        }
    }
    srm_push(s);
}

/* An output block is computed: it can go out. */
static void srm_compute_done(void *opaque)
{
    ESP32P4PpaState *s = opaque;
    ESP32P4PpaOutBlock *o;
    uint32_t subs;
    bool last;

    if (s->srm_state != 2 || !s->srm_computing) {
        return;
    }
    s->srm_computing = false;
    o = &s->srm_out[(s->srm_out_head + s->srm_out_num) & 1];
    subs = srm_subs(s, s->srm_comp);
    last = true;
    if (s->srm_sub < subs) {
        last = false;
    } else {
        for (uint32_t k = s->srm_comp + 1; k < s->srm_nmb; k++) {
            if (srm_subs(s, k)) {
                last = false;
                break;
            }
        }
    }
    o->last = last;
    s->srm_out_num++;
    if (s->srm_sub >= subs) {
        s->srm_slot_fill[s->srm_comp & 1] = UINT32_MAX;
        s->srm_comp++;
        s->srm_sub = 0;
    }
    srm_run(s);
}

/* BLEND */

/* The latched configuration's registers, by their place in blend_cfg[]. */
enum {
    B_COLOR_MODE, B_BYTE_ORDER, B_FIX_ALPHA, B_RGB, B_FIX_PIXEL,
    B_CK_FG_LOW, B_CK_FG_HIGH, B_CK_BG_LOW, B_CK_BG_HIGH, B_CK_DEFAULT,
    B_TX_SIZE, B_TRANS_MODE,
};

static const hwaddr blend_cfg_regs[12] = {
    A_BLEND_COLOR_MODE, A_BLEND_BYTE_ORDER, A_BLEND_FIX_ALPHA, A_BLEND_RGB,
    A_BLEND_FIX_PIXEL, A_CK_FG_LOW, A_CK_FG_HIGH, A_CK_BG_LOW, A_CK_BG_HIGH,
    A_CK_DEFAULT, A_BLEND_TX_SIZE, A_BLEND_TRANS_MODE,
};

static void blend_fmt(ESP32P4PpaState *s, int layer, PpaFmt *f)
{
    uint32_t cm = s->blend_cfg[B_COLOR_MODE], bo = s->blend_cfg[B_BYTE_ORDER];
    uint32_t fa = s->blend_cfg[B_FIX_ALPHA];

    if (layer == 0) {
        fmt_in(s, f, extract32(cm, 0, 4), bo & BIT(2), bo & BIT(0),
               extract32(fa, 16, 2), fa & BIT(20), extract32(fa, 0, 8),
               cm & BIT(12), cm & BIT(14), extract32(cm, 16, 2));
        f->clut = s->clut[0];
    } else {
        fmt_in(s, f, extract32(cm, 4, 4), bo & BIT(3), bo & BIT(1),
               extract32(fa, 18, 2), fa & BIT(21), extract32(fa, 8, 8),
               false, false, 0);
        f->clut = s->clut[1];
        f->fix_rgb = s->blend_cfg[B_RGB] & 0xffffff;
    }
    f->dbg = s->blend_dbg[0];
    f->dbg_data = s->blend_dbg[1];
}

static void blend_fmt_out(ESP32P4PpaState *s, PpaFmt *f)
{
    uint32_t cm = s->blend_cfg[B_COLOR_MODE];

    memset(f, 0, sizeof(*f));
    f->cm = extract32(cm, 8, 4);
    f->full = cm & BIT(13);
    f->bt709 = cm & BIT(15);
    f->gray = s->blend_gray;
}

/* Whether the foreground's stream is taken: blending, or an A4/A8 bypass. */
static bool blend_uses_fg(ESP32P4PpaState *s)
{
    unsigned fg = extract32(s->blend_cfg[B_COLOR_MODE], 4, 4);

    if (s->blend_mode == 2) {
        return false;
    }
    return s->blend_mode == 0 || fg == CM_A8 || fg == CM_A4;
}

static void blend_idle(ESP32P4PpaState *s)
{
    s->blend_state = 0;
    s->batch_pending = false;
    s->out_num = 0;
    s->out_head = 0;
    if (s->dma2d) {
        esp32p4_dma2d_timer_del(s->dma2d, s->blend_timer);
    }
}

static void blend_run(ESP32P4PpaState *s);

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void blend_begin(ESP32P4PpaState *s)
{
    ESP32P4Dma2dDesc bg = { 0 }, fg = { 0 };
    unsigned bgcm = extract32(s->blend_cfg[B_COLOR_MODE], 0, 4);
    uint32_t err = 0;
    PpaFmt fo;

    if (s->blend_mode != 2) {
        if (!esp32p4_dma2d_desc(s->dma2d, OUT, ESP32P4_DMA2D_TX_PPA_BLEND_BG,
                                &bg)) {
            return;
        }
        if (blend_uses_fg(s) &&
            !esp32p4_dma2d_desc(s->dma2d, OUT,
                                ESP32P4_DMA2D_TX_PPA_BLEND_FG, &fg)) {
            return;
        }
    }
    s->blend_w = extract32(s->blend_cfg[B_TX_SIZE], 0, 14);
    s->blend_h = extract32(s->blend_cfg[B_TX_SIZE], 14, 14);
    if (s->blend_mode != 2 && (!s->blend_w || !s->blend_h)) {
        s->blend_w = bg.hb;
        s->blend_h = bg.vb;
    }
    if (s->blend_mode != 2) {
        if (blend_uses_fg(s) && (bg.hb != fg.hb || bg.vb != fg.vb)) {
            err |= BIT(0);
        }
        if ((bgcm == CM_YUV422 || bgcm == CM_YUV420) && (s->blend_w & 1)) {
            err |= BIT(1);
        }
        if (bgcm == CM_YUV420 && (s->blend_h & 1)) {
            err |= BIT(2);
        }
    }
    s->regs[R(A_BLEND_ST)] = err;
    if (err) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND's parameters are wrong "
                      "(BLEND_ST 0x%x)\n", TYPE_ESP32P4_PPA, err);
        blend_idle(s);
        ppa_raise(s, INT_BLEND_PARAM_ERR);
        return;
    }
    blend_fmt_out(s, &fo);
    if (!cm_nibbles(fo.cm) || fo.cm == CM_L8 || fo.cm == CM_L4 ||
        fo.cm == CM_A8 || fo.cm == CM_A4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND_TX_CM %u is no output "
                      "color mode\n", TYPE_ESP32P4_PPA, fo.cm);
    }
    s->blend_row = 0;
    s->blend_col = 0;
    s->bg_fill = 0;
    s->fg_fill = 0;
    s->out_head = 0;
    s->out_num = 0;
    s->out_total = (uint64_t)s->blend_w * s->blend_h *
                   MAX(cm_nibbles(fo.cm), 1) / 2;
    s->batch_pending = false;
    s->blend_busy_until = esp32_line_time();
    s->blend_state = 2;
    if (!s->out_total) {
        /* An empty block: the frame ends at once. */
        esp32p4_dma2d_push(s->dma2d, ESP32P4_DMA2D_RX_PPA_BLEND, NULL, 0,
                           ESP32P4_DMA2D_EOF);
    }
    blend_run(s);
}

/* The bytes of a row of layer @f, @w pixels wide. */
static uint32_t row_bytes(const PpaFmt *f, uint32_t w)
{
    return w * cm_nibbles(f->cm) / 2;
}

/* Pull a layer's bytes into its row buffer, up to the row (pair) it needs. */
static void blend_pull(ESP32P4PpaState *s, unsigned sel, const PpaFmt *f,
                       uint8_t *buf, uint32_t *fill)
{
    uint32_t want = row_bytes(f, s->blend_w) * (f->cm == CM_YUV420 ? 2 : 1);
    bool eof;

    if (*fill < want) {
        *fill += esp32p4_dma2d_pull(s->dma2d, sel, buf + *fill,
                                    want - *fill, &eof);
    }
}

/* The pixels of the current row a layer has whole. */
static uint32_t blend_avail(ESP32P4PpaState *s, const PpaFmt *f,
                            uint32_t fill)
{
    uint32_t rb = row_bytes(f, s->blend_w), nib = cm_nibbles(f->cm);

    if (f->cm == CM_YUV420) {
        /* The row pair, whole. */
        return fill >= 2 * rb ? s->blend_w : 0;
    }
    if (f->cm == CM_YUV422) {
        return MIN(fill / 4 * 2, s->blend_w);
    }
    return MIN(fill * 2 / MAX(nib, 1), s->blend_w);
}

/* Whether pixel @c of key range LOW..HIGH (registers @lo, @hi) holds it. */
static bool in_key(uint32_t px, uint32_t lo, uint32_t hi)
{
    for (int i = 0; i < 24; i += 8) {
        uint32_t c = extract32(px, i, 8);

        if (c < extract32(lo, i, 8) || c > extract32(hi, i, 8)) {
            return false;
        }
    }
    return true;
}

/* Blend background @b and foreground @f, ARGB8888 each. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t blend_px(ESP32P4PpaState *s, uint32_t b, uint32_t f)
{
    uint32_t ab = b >> 24, af = f >> 24, abp, aout, recip, out;
    bool bk = in_key(b, s->blend_cfg[B_CK_BG_LOW], s->blend_cfg[B_CK_BG_HIGH]);
    bool fk = in_key(f, s->blend_cfg[B_CK_FG_LOW], s->blend_cfg[B_CK_FG_HIGH]);
    uint32_t def = s->blend_cfg[B_CK_DEFAULT];

    if (fk && !bk) {
        return b;
    }
    if (bk && !fk) {
        return def & BIT(24) ? f : b;
    }
    abp = (ab * (255 - af) + 127) / 255;
    aout = af + abp;
    recip = aout ? 65536 / aout : 0;
    out = aout << 24;
    for (int i = 0; i < 24; i += 8) {
        uint32_t cb = extract32(b, i, 8), cf = extract32(f, i, 8);
        uint32_t c = ((cf * af + cb * abp) * recip + 32768) >> 16;

        out |= MIN(c, 255) << i;
    }
    if (bk && fk) {
        out = (out & 0xff000000) | (def & 0xffffff);
    }
    return out;
}

/* Put the bytes of a batch into the ring of bytes to push. */
static void out_put(ESP32P4PpaState *s, const uint8_t *buf, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        s->out[(s->out_head + s->out_num) % s->out_buf_size] = buf[i];
        s->out_num++;
    }
}

static void blend_push(ESP32P4PpaState *s)
{
    while (s->out_num) {
        uint32_t n = MIN(s->out_num, s->out_buf_size - s->out_head);
        bool last = n == s->out_num && n == s->out_total;
        size_t got;

        got = esp32p4_dma2d_push(s->dma2d, ESP32P4_DMA2D_RX_PPA_BLEND,
                                 s->out + s->out_head, n,
                                 last ? ESP32P4_DMA2D_EOF : 0);
        s->out_head = (s->out_head + got) % s->out_buf_size;
        s->out_num -= got;
        s->out_total -= got;
        if (got < n) {
            return;
        }
    }
    if (!s->out_total && s->blend_state == 2 && !s->batch_pending) {
        blend_idle(s);
        ppa_raise(s, INT_BLEND_EOF);
    }
}

/*
 * The next batch: as many pixels of the current row as both layers have,
 * the output has room for and the batch holds, blended (or filled) now
 * and given to the 2D-DMA once the engine has spent its cycles on them.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void blend_run(ESP32P4PpaState *s)
{
    PpaFmt fb, ff, fo;
    uint32_t n, onib, room, px[ESP32P4_PPA_BATCH_MAX];
    int64_t now = esp32_line_time();
    uint64_t cycles;
    bool fg = blend_uses_fg(s);

    if (s->blend_state != 2) {
        return;
    }
    blend_fmt(s, 0, &fb);
    blend_fmt(s, 1, &ff);
    blend_fmt_out(s, &fo);
    onib = MAX(cm_nibbles(fo.cm), 1);
    if (s->blend_mode != 2) {
        blend_pull(s, ESP32P4_DMA2D_TX_PPA_BLEND_BG, &fb, s->bg_row,
                   &s->bg_fill);
        if (fg) {
            blend_pull(s, ESP32P4_DMA2D_TX_PPA_BLEND_FG, &ff, s->fg_row,
                       &s->fg_fill);
        }
    }
    if (!s->batch_pending && s->blend_row < s->blend_h) {
        uint32_t row = s->blend_row;

        n = s->blend_w - s->blend_col;
        if (s->blend_mode != 2) {
            n = MIN(n, blend_avail(s, &fb, s->bg_fill) - s->blend_col);
            if (fg) {
                n = MIN(n, blend_avail(s, &ff, s->fg_fill) - s->blend_col);
            }
        }
        room = (s->out_buf_size - s->out_num) * 2 / onib;
        n = MIN(MIN(n, room), ESP32P4_PPA_BATCH_MAX);
        if ((fo.cm == CM_YUV422 || fo.cm == CM_YUV420) &&
            s->blend_col + n < s->blend_w) {
            n &= ~1u;
        }
        if (n) {
            const uint8_t *brow = s->bg_row;
            uint32_t rb = row_bytes(&fb, s->blend_w);

            if (fb.cm == CM_YUV420 && (row & 1)) {
                brow += rb;
            }
            for (uint32_t i = 0; i < n; i++) {
                uint32_t x = s->blend_col + i, b = 0, f = 0, o;

                if (s->blend_mode == 2) {
                    px[i] = s->blend_cfg[B_FIX_PIXEL];
                    continue;
                }
                b = input_px(&fb, DBG_BLEND_BG, brow, s->bg_row,
                             s->bg_row + rb, x);
                if (fg) {
                    f = input_px(&ff, DBG_BLEND_FG, s->fg_row, s->fg_row,
                                 s->fg_row, x);
                }
                if (s->blend_mode == 1) {
                    o = b;
                    if (fg) {
                        o = deposit32(o, 24, 8, f >> 24);
                    }
                } else {
                    o = blend_px(s, b, f);
                }
                if (fb.dbg == DBG_BLEND_OUT) {
                    o = fb.dbg_data;
                }
                px[i] = o;
            }
            if (s->blend_mode == 2 && fo.cm == CM_YUV422) {
                s->batch_len = 0;
                for (uint32_t i = 0; i < n; i += 2) {
                    for (int j = 0; j < 4; j++) {
                        s->batch[s->batch_len++] = px[0] >> (8 * j);
                    }
                }
            } else if (s->blend_mode == 2 && fo.cm == CM_YUV420) {
                s->batch_len = 0;
                for (uint32_t i = 0; i < n; i += 2) {
                    for (int j = 0; j < 3; j++) {
                        s->batch[s->batch_len++] = px[0] >> (8 * j);
                    }
                }
            } else if (s->blend_mode == 2 && fo.cm == CM_GRAY) {
                memset(s->batch, px[0] & 0xff, n);
                s->batch_len = n;
            } else {
                s->batch_len = encode_px(&fo, px, n, row, s->batch);
            }
            s->blend_col += n;
            if (s->blend_col == s->blend_w) {
                /* The row is done; YUV420's pair once its second is. */
                s->blend_col = 0;
                s->blend_row++;
                if (fb.cm != CM_YUV420 || (row & 1)) {
                    s->bg_fill = 0;
                }
                s->fg_fill = 0;
            }
            cycles = s->blend_mode == 2 ? DIV_ROUND_UP(s->batch_len, 8) : n;
            s->batch_pending = true;
            s->blend_busy_until = ppa_after(s, MAX(now, s->blend_busy_until),
                                            cycles);
            esp32p4_dma2d_timer_mod(s->dma2d, s->blend_timer,
                                    s->blend_busy_until);
        }
    }
    blend_push(s);
}

static void blend_batch_done(void *opaque)
{
    ESP32P4PpaState *s = opaque;

    if (s->blend_state != 2 || !s->batch_pending) {
        return;
    }
    s->batch_pending = false;
    out_put(s, s->batch, s->batch_len);
    blend_run(s);
    blend_push(s);
}

/* The 2D-DMA's side */

static void ppa_dma_ready(void *opaque, unsigned sel)
{
    ESP32P4PpaState *s = opaque;

    if (sel == ESP32P4_DMA2D_TX_PPA_SRM) {
        if (s->srm_state == 1) {
            srm_begin(s);
        } else if (s->srm_state == 2) {
            srm_run(s);
        }
    } else {
        if (s->blend_state == 1) {
            blend_begin(s);
        } else if (s->blend_state == 2) {
            blend_run(s);
        }
    }
}

static void ppa_dma_rx_ready(void *opaque, unsigned sel)
{
    ESP32P4PpaState *s = opaque;

    if (sel == ESP32P4_DMA2D_RX_PPA_SRM) {
        if (s->srm_state == 1) {
            srm_begin(s);
        } else if (s->srm_state == 2) {
            srm_run(s);
        }
    } else if (s->blend_state == 1) {
        blend_begin(s);
    } else if (s->blend_state == 2) {
        blend_run(s);
    }
}

static const ESP32P4Dma2dPeriphOps ppa_dma_ops = {
    .tx_ready = ppa_dma_ready,
    .rx_ready = ppa_dma_rx_ready,
};

/* Registers */

static bool clut_powered(ESP32P4PpaState *s)
{
    uint32_t conf = s->regs[R(A_CLUT_CONF)];

    return !(conf & CLUT_FORCE_PD) || (conf & CLUT_FORCE_PU);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void srm_start(ESP32P4PpaState *s)
{
    if (s->srm_state) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SCAL_ROTATE_START while the SRM "
                      "is busy\n", TYPE_ESP32P4_PPA);
        return;
    }
    if (s->regs[R(A_SR_SCAL_ROTATE)] & SCAL_ROTATE_RST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SCAL_ROTATE_START while the SRM "
                      "is held in reset\n", TYPE_ESP32P4_PPA);
        return;
    }
    s->srm_cfg[0] = s->regs[R(A_SR_COLOR_MODE)];
    s->srm_cfg[1] = s->regs[R(A_SR_BYTE_ORDER)];
    s->srm_cfg[2] = s->regs[R(A_SR_FIX_ALPHA)];
    s->srm_cfg[3] = s->regs[R(A_SR_SCAL_ROTATE)];
    s->srm_dbg[0] = s->regs[R(A_DEBUG_CTRL0)];
    s->srm_dbg[1] = s->regs[R(A_DEBUG_CTRL1)];
    s->srm_gray = s->regs[R(A_RGB2GRAY)];
    s->srm_mb = s->srm_cfg[1] & BIT(3) ? 16 : 32;
    s->regs[R(A_SR_PARAM_ERR_ST)] = 0;
    s->srm_state = 1;
    srm_begin(s);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void blend_start(ESP32P4PpaState *s)
{
    uint32_t mode = s->regs[R(A_BLEND_TRANS_MODE)];

    if (!(mode & BLEND_EN)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND_TRANS_MODE_UPDATE with "
                      "BLEND_EN clear starts nothing\n", TYPE_ESP32P4_PPA);
        return;
    }
    if (mode & BLEND_RST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND_TRANS_MODE_UPDATE while "
                      "BLEND is held in reset\n", TYPE_ESP32P4_PPA);
        return;
    }
    if (s->blend_state) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND_TRANS_MODE_UPDATE while "
                      "BLEND is busy\n", TYPE_ESP32P4_PPA);
        return;
    }
    for (int i = 0; i < ARRAY_SIZE(blend_cfg_regs); i++) {
        s->blend_cfg[i] = s->regs[R(blend_cfg_regs[i])];
    }
    s->blend_dbg[0] = s->regs[R(A_DEBUG_CTRL0)];
    s->blend_dbg[1] = s->regs[R(A_DEBUG_CTRL1)];
    s->blend_gray = s->regs[R(A_RGB2GRAY)];
    s->blend_mode = mode & BLEND_FILL_EN ? 2 : mode & BLEND_BYPASS ? 1 : 0;
    s->regs[R(A_BLEND_ST)] = 0;
    s->blend_state = 1;
    blend_begin(s);
}

static uint32_t clut_read(ESP32P4PpaState *s, int i)
{
    uint32_t v;

    if (s->regs[R(A_CLUT_CONF)] & APB_FIFO_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND%d_CLUT_DATA read in memory "
                      "mode\n", TYPE_ESP32P4_PPA, i);
        return 0;
    }
    if (!clut_powered(s)) {
        return 0;
    }
    v = s->clut[i][s->clut_raddr[i] & 0xff];
    if (!(s->regs[R(A_CLUT_CONF)] & (CLUT_MEM_RST(i) | CLUT_RDADDR_RST(i)))) {
        s->clut_raddr[i] = (s->clut_raddr[i] + 1) & 0xff;
    }
    return v;
}

static void clut_write(ESP32P4PpaState *s, int i, uint32_t v)
{
    if (s->regs[R(A_CLUT_CONF)] & APB_FIFO_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BLEND%d_CLUT_DATA written in "
                      "memory mode\n", TYPE_ESP32P4_PPA, i);
        return;
    }
    if (!clut_powered(s) || (s->regs[R(A_CLUT_CONF)] & CLUT_MEM_RST(i))) {
        return;
    }
    s->clut[i][s->clut_waddr[i] & 0xff] = v;
    s->clut_waddr[i] = (s->clut_waddr[i] + 1) & 0xff;
    s->clut_cnt[i] = (s->clut_cnt[i] + 1) & 0x1ff;
}

static void clut_conf_write(ESP32P4PpaState *s, uint32_t v)
{
    bool was_powered = clut_powered(s);

    s->regs[R(A_CLUT_CONF)] = v & 0xff;
    for (int i = 0; i < 2; i++) {
        if (v & CLUT_MEM_RST(i)) {
            s->clut_waddr[i] = 0;
            s->clut_raddr[i] = 0;
            s->clut_cnt[i] = 0;
        }
        if (v & CLUT_RDADDR_RST(i)) {
            s->clut_raddr[i] = 0;
        }
    }
    if (was_powered && !clut_powered(s)) {
        /* Powered down, the CLUTs keep nothing. */
        memset(s->clut, 0, sizeof(s->clut));
    }
}

static uint64_t esp32p4_ppa_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4PpaState *s = opaque;
    hwaddr a = addr & ~3;
    uint32_t v = 0;
    int i;

    esp32p4_dma2d_sync(s->dma2d, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (a >= A_CLUT0_MEM && a < A_CLUT_MEM_END) {
        int c = a >= A_CLUT1_MEM;

        if (!(s->regs[R(A_CLUT_CONF)] & APB_FIFO_MASK)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: CLUT memory read in FIFO "
                          "mode\n", TYPE_ESP32P4_PPA);
        } else if (clut_powered(s)) {
            v = s->clut[c][(a >> 2) & 0xff];
        }
        return extract32(v, (addr & 3) * 8, size * 8);
    }
    switch (a) {
    case A_BLEND0_CLUT_DATA:
    case A_BLEND1_CLUT_DATA:
        v = clut_read(s, a == A_BLEND1_CLUT_DATA);
        break;
    case A_INT_ST:
        v = s->regs[R(A_INT_RAW)] & s->regs[R(A_INT_ENA)];
        break;
    case A_INT_RAW:
    case A_BLEND_ST:
    case A_SR_PARAM_ERR_ST:
        v = s->regs[R(a)];
        break;
    case A_INT_CLR:
        v = 0;
        break;
    case A_CLUT_CNT:
        v = s->clut_cnt[0] | s->clut_cnt[1] << 9;
        break;
    case A_SR_STATUS:
        v = s->srm_state == 2 ? 1 << 2 | 1 << 6 : 0;
        break;
    case A_SR_SCAL_ROTATE:
        v = s->regs[R(a)] & ~SCAL_ROTATE_START;
        break;
    case A_BLEND_TRANS_MODE:
        v = s->regs[R(a)] & ~BLEND_UPDATE;
        break;
    default:
        i = reg_def(a);
        if (i < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                          HWADDR_PRIx "\n", TYPE_ESP32P4_PPA, addr);
            return 0;
        }
        v = s->regs[R(a)];
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void esp32p4_ppa_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4PpaState *s = opaque;
    hwaddr a = addr & ~3;
    uint32_t old = 0, v;
    int i;

    esp32p4_dma2d_sync(s->dma2d, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (a >= A_CLUT0_MEM && a < A_CLUT_MEM_END) {
        int c = a >= A_CLUT1_MEM;

        if (!(s->regs[R(A_CLUT_CONF)] & APB_FIFO_MASK)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: CLUT memory written in FIFO "
                          "mode\n", TYPE_ESP32P4_PPA);
        } else if (clut_powered(s)) {
            v = deposit32(s->clut[c][(a >> 2) & 0xff], (addr & 3) * 8,
                          size * 8, value);
            s->clut[c][(a >> 2) & 0xff] = v;
        }
        return;
    }
    i = reg_def(a);
    if (a == A_INT_CLR) {
        if (value & 0xf) {
            s->regs[R(A_INT_RAW)] &= ~(extract32(value, 0, 4) <<
                                       ((addr & 3) * 8));
            ppa_update_irq(s);
        }
        return;
    }
    if (a == A_INT_RAW) {
        /* Write 1 to clear, as INT_CLR. */
        s->regs[R(A_INT_RAW)] &= ~((uint32_t)value << ((addr & 3) * 8));
        ppa_update_irq(s);
        return;
    }
    if (a == A_INT_ST || a == A_CLUT_CNT || a == A_BLEND_ST ||
        a == A_SR_PARAM_ERR_ST || a == A_SR_STATUS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to a read-only register\n",
                      TYPE_ESP32P4_PPA);
        return;
    }
    if (i < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PPA, addr);
        return;
    }
    if (a != A_BLEND0_CLUT_DATA && a != A_BLEND1_CLUT_DATA) {
        old = s->regs[R(a)];
        if (a == A_SR_SCAL_ROTATE) {
            old &= ~SCAL_ROTATE_START;
        } else if (a == A_BLEND_TRANS_MODE) {
            old &= ~BLEND_UPDATE;
        }
    }
    v = deposit32(old, (addr & 3) * 8, size * 8, value);
    switch (a) {
    case A_BLEND0_CLUT_DATA:
    case A_BLEND1_CLUT_DATA:
        if (size != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: a CLUT entry written in "
                          "part\n", TYPE_ESP32P4_PPA);
        }
        clut_write(s, a == A_BLEND1_CLUT_DATA, v);
        break;
    case A_CLUT_CONF:
        clut_conf_write(s, v);
        break;
    case A_INT_ENA:
        s->regs[R(a)] = v & 0xf;
        ppa_update_irq(s);
        break;
    case A_SR_SCAL_ROTATE:
        s->regs[R(a)] = v & ppa_reg_defs[i].mask & ~SCAL_ROTATE_START;
        if (v & SCAL_ROTATE_RST) {
            srm_idle(s);
        }
        if (v & SCAL_ROTATE_START) {
            srm_start(s);
        }
        break;
    case A_BLEND_TRANS_MODE:
        s->regs[R(a)] = v & 0x77;
        if (v & BLEND_RST) {
            blend_idle(s);
        }
        if (v & BLEND_UPDATE) {
            blend_start(s);
        }
        break;
    case A_ECO_CELL_CTRL:
        s->regs[R(a)] = v & 0x2;
        break;
    default:
        s->regs[R(a)] = v & ppa_reg_defs[i].mask;
        break;
    }
}

static const MemoryRegionOps esp32p4_ppa_ops = {
    .read = esp32p4_ppa_read,
    .write = esp32p4_ppa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void ppa_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4PpaState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        if (s->dma2d) {
            esp32p4_dma2d_sync(s->dma2d, now);
        }
        return;
    }
    s->edges = esp32p4_clk_edges_or_at(s->clk, now);
    /* An engine the stopped clock held goes on from now. */
    if (s->srm_computing && s->srm_busy_until == INT64_MAX) {
        s->srm_busy_until = now;
        esp32p4_dma2d_timer_mod(s->dma2d, s->srm_timer, now);
    }
    if (s->batch_pending && s->blend_busy_until == INT64_MAX) {
        s->blend_busy_until = now;
        esp32p4_dma2d_timer_mod(s->dma2d, s->blend_timer, now);
    }
}

static void esp32p4_ppa_reset_hold(Object *obj, ResetType type)
{
    ESP32P4PpaState *s = ESP32P4_PPA(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (int i = 0; i < ARRAY_SIZE(ppa_reg_defs); i++) {
        s->regs[R(ppa_reg_defs[i].addr)] = ppa_reg_defs[i].reset;
    }
    memset(s->clut_waddr, 0, sizeof(s->clut_waddr));
    memset(s->clut_raddr, 0, sizeof(s->clut_raddr));
    memset(s->clut_cnt, 0, sizeof(s->clut_cnt));
    srm_idle(s);
    blend_idle(s);
    s->edges = esp32p4_clk_edges_or_at(s->clk,
                                       qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void esp32p4_ppa_reset_exit(Object *obj, ResetType type)
{
    ppa_update_irq(ESP32P4_PPA(obj));
}

static void esp32p4_ppa_init(Object *obj)
{
    ESP32P4PpaState *s = ESP32P4_PPA(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_ppa_ops, s, "esp32p4.ppa",
                          ESP32P4_PPA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", ppa_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    s->row_buf_size = 2 * ESP32P4_PPA_ROW_MAX;
    s->out_buf_size = ESP32P4_PPA_OUT_MAX;
    s->bg_row = g_malloc0(s->row_buf_size);
    s->fg_row = g_malloc0(s->row_buf_size);
    s->out = g_malloc0(s->out_buf_size);
}

static void esp32p4_ppa_realize(DeviceState *dev, Error **errp)
{
    ESP32P4PpaState *s = ESP32P4_PPA(dev);

    if (!s->dma2d_obj) {
        error_setg(errp, "%s: the 'dma2d' link is not set", TYPE_ESP32P4_PPA);
        return;
    }
    s->dma2d = ESP32P4_DMA2D(s->dma2d_obj);
    esp32p4_dma2d_attach(s->dma2d, OUT, ESP32P4_DMA2D_TX_PPA_SRM,
                         &ppa_dma_ops, s);
    esp32p4_dma2d_attach(s->dma2d, OUT, ESP32P4_DMA2D_TX_PPA_BLEND_BG,
                         &ppa_dma_ops, s);
    esp32p4_dma2d_attach(s->dma2d, OUT, ESP32P4_DMA2D_TX_PPA_BLEND_FG,
                         &ppa_dma_ops, s);
    esp32p4_dma2d_attach(s->dma2d, IN, ESP32P4_DMA2D_RX_PPA_SRM,
                         &ppa_dma_ops, s);
    esp32p4_dma2d_attach(s->dma2d, IN, ESP32P4_DMA2D_RX_PPA_BLEND,
                         &ppa_dma_ops, s);
    s->srm_timer = esp32p4_dma2d_timer_new(s->dma2d, srm_compute_done, s);
    s->blend_timer = esp32p4_dma2d_timer_new(s->dma2d, blend_batch_done, s);
}

static void esp32p4_ppa_finalize(Object *obj)
{
    ESP32P4PpaState *s = ESP32P4_PPA(obj);

    g_free(s->bg_row);
    g_free(s->fg_row);
    g_free(s->out);
}

static const Property esp32p4_ppa_properties[] = {
    DEFINE_PROP_LINK("dma2d", ESP32P4PpaState, dma2d_obj, TYPE_ESP32P4_DMA2D,
                     Object *),
};

static const VMStateDescription vmstate_esp32p4_ppa_out = {
    .name = "esp32p4-ppa-out",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(x, ESP32P4PpaOutBlock),
        VMSTATE_UINT32(y, ESP32P4PpaOutBlock),
        VMSTATE_UINT32(w, ESP32P4PpaOutBlock),
        VMSTATE_UINT32(h, ESP32P4PpaOutBlock),
        VMSTATE_UINT32(len, ESP32P4PpaOutBlock),
        VMSTATE_UINT32(pushed, ESP32P4PpaOutBlock),
        VMSTATE_BOOL(declared, ESP32P4PpaOutBlock),
        VMSTATE_BOOL(last, ESP32P4PpaOutBlock),
        VMSTATE_UINT8_ARRAY(data, ESP32P4PpaOutBlock,
                            ESP32P4_PPA_SRM_OUT_MAX),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_ppa = {
    .name = TYPE_ESP32P4_PPA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4PpaState, ESP32P4_PPA_REGS),
        VMSTATE_UINT32_2DARRAY(clut, ESP32P4PpaState, 2,
                               ESP32P4_PPA_CLUT_SIZE),
        VMSTATE_UINT32_ARRAY(clut_waddr, ESP32P4PpaState, 2),
        VMSTATE_UINT32_ARRAY(clut_raddr, ESP32P4PpaState, 2),
        VMSTATE_UINT32_ARRAY(clut_cnt, ESP32P4PpaState, 2),
        VMSTATE_UINT32(srm_state, ESP32P4PpaState),
        VMSTATE_UINT32_ARRAY(srm_cfg, ESP32P4PpaState, 4),
        VMSTATE_UINT32_ARRAY(srm_dbg, ESP32P4PpaState, 2),
        VMSTATE_UINT32(srm_gray, ESP32P4PpaState),
        VMSTATE_UINT32(srm_hb, ESP32P4PpaState),
        VMSTATE_UINT32(srm_vb, ESP32P4PpaState),
        VMSTATE_UINT32(srm_mb, ESP32P4PpaState),
        VMSTATE_UINT32(srm_nmb, ESP32P4PpaState),
        VMSTATE_UINT32(srm_req, ESP32P4PpaState),
        VMSTATE_UINT32(srm_comp, ESP32P4PpaState),
        VMSTATE_UINT32(srm_sub, ESP32P4PpaState),
        VMSTATE_UINT32_ARRAY(srm_slot_mb, ESP32P4PpaState, 2),
        VMSTATE_UINT32_ARRAY(srm_slot_fill, ESP32P4PpaState, 2),
        VMSTATE_UINT8_2DARRAY(srm_in, ESP32P4PpaState, 2,
                              ESP32P4_PPA_SRM_IN_MAX),
        VMSTATE_BOOL(srm_computing, ESP32P4PpaState),
        VMSTATE_INT64(srm_busy_until, ESP32P4PpaState),
        VMSTATE_STRUCT_ARRAY(srm_out, ESP32P4PpaState, 2, 1,
                             vmstate_esp32p4_ppa_out, ESP32P4PpaOutBlock),
        VMSTATE_UINT32(srm_out_head, ESP32P4PpaState),
        VMSTATE_UINT32(srm_out_num, ESP32P4PpaState),
        VMSTATE_BOOL(srm_stalled, ESP32P4PpaState),
        VMSTATE_UINT32(blend_state, ESP32P4PpaState),
        VMSTATE_UINT32(blend_mode, ESP32P4PpaState),
        VMSTATE_UINT32_ARRAY(blend_cfg, ESP32P4PpaState, 12),
        VMSTATE_UINT32_ARRAY(blend_dbg, ESP32P4PpaState, 2),
        VMSTATE_UINT32(blend_gray, ESP32P4PpaState),
        VMSTATE_UINT32(blend_w, ESP32P4PpaState),
        VMSTATE_UINT32(blend_h, ESP32P4PpaState),
        VMSTATE_UINT32(blend_row, ESP32P4PpaState),
        VMSTATE_UINT32(blend_col, ESP32P4PpaState),
        VMSTATE_VBUFFER_UINT32(bg_row, ESP32P4PpaState, 1, NULL,
                               row_buf_size),
        VMSTATE_UINT32(bg_fill, ESP32P4PpaState),
        VMSTATE_VBUFFER_UINT32(fg_row, ESP32P4PpaState, 1, NULL,
                               row_buf_size),
        VMSTATE_UINT32(fg_fill, ESP32P4PpaState),
        VMSTATE_VBUFFER_UINT32(out, ESP32P4PpaState, 1, NULL, out_buf_size),
        VMSTATE_UINT32(out_head, ESP32P4PpaState),
        VMSTATE_UINT32(out_num, ESP32P4PpaState),
        VMSTATE_UINT32(out_total, ESP32P4PpaState),
        VMSTATE_UINT8_ARRAY(batch, ESP32P4PpaState,
                            ESP32P4_PPA_BATCH_MAX * 4 + 8),
        VMSTATE_UINT32(batch_len, ESP32P4PpaState),
        VMSTATE_BOOL(batch_pending, ESP32P4PpaState),
        VMSTATE_INT64(blend_busy_until, ESP32P4PpaState),
        VMSTATE_CLOCK(clk, ESP32P4PpaState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_ppa_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_ppa_realize;
    dc->vmsd = &vmstate_esp32p4_ppa;
    device_class_set_props(dc, esp32p4_ppa_properties);
    rc->phases.hold = esp32p4_ppa_reset_hold;
    rc->phases.exit = esp32p4_ppa_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static const TypeInfo esp32p4_ppa_info = {
    .name           = TYPE_ESP32P4_PPA,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(ESP32P4PpaState),
    .instance_init  = esp32p4_ppa_init,
    .instance_finalize = esp32p4_ppa_finalize,
    .class_init     = esp32p4_ppa_class_init,
};

static void esp32p4_ppa_register_types(void)
{
    type_register_static(&esp32p4_ppa_info);
}

type_init(esp32p4_ppa_register_types)
