/*
 * ESP32-P4 JPEG codec
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 37 (JPEG Codec) and chapter 8's figure 8.4-4 (the decoder's output);
 * ESP-IDF's jpeg_reg.h and jpeg_struct.h for v3, its jpeg_ll.h and
 * jpeg_hal.c, and esp_driver_jpeg (jpeg_encode.c, jpeg_decode.c and
 * jpeg_parse_marker.c), which show what software leaves to the hardware.
 *
 * A baseline JPEG encoder or decoder (JPEG_MODE), one frame per
 * JPEG_JPEG_START, its data through the 2D-DMA (include/hw/misc/
 * esp32p4_jpeg.h), its arithmetic in esp32p4_jpeg_codec.c.
 *
 * Encoder. The source formats of TRM table 37.5-1 (JPEG_COLOR_SPACE,
 * JPEG_EXTD_CONFIG, JPEG_PIXEL_REV) to YUV444, YUV422 or YUV420
 * (JPEG_SAMPLE_SEL; a YUV422 source to YUV422 or YUV420 only, YUV420 and
 * GRAY sources as they are), RGB converted by TRM 37.5.1.2's formulas.
 * Each MCU's data units are transformed, quantised with the table
 * JPEG_LQNR_TBL_SEL (Y) or JPEG_CQNR_TBL_SEL (U, V) selects, at
 * JPEG_QNR_PRECISION's 8 or 16 bits, and Huffman coded with T.81 annex
 * K's tables, the luminance tables for Y and the chrominance tables for U
 * and V: the encoder has no Huffman registers, and ESP-IDF's encoder
 * writes those tables into the DHT segment it puts before the codec's
 * output (jpeg_emit_marker.c). The output is the scan's coded data only,
 * 0xFF stuffed with JPEG_FF_CHECK_EN, its last byte padded with 1 bits,
 * then EOI with JPEG_TAILER_EN; software writes the headers. The frame is
 * ceil(JPEG_HA / MCU width) x ceil(JPEG_VA / MCU height) MCUs, HA and VA
 * of 14 bits.
 *
 * Decoder. Software parses the headers and loads the tables (TRM
 * 37.5.2); the decoder takes the bitstream from SOS, parses SOS, the
 * restart markers and EOI, and gives each MCU's data units. The frame's
 * format comes from JPEG_COMPONENT_NUM and the components' factors (TRM
 * table 37.5-5); anything else is refused at JPEG_START. SOS is checked
 * against the components' registers: each scan component's ID
 * (JPEG_CID_ERR_INT), its DC and AC tables against JPEG_DHT_INFO's IDs
 * (JPEG_C_DHT_DC_ID_ERR_INT, _AC_ID_), the component's quantisation table
 * against JPEG_DQT_INFO's IDs (JPEG_C_DQT_ID_ERR_INT), its length against
 * its component count (JPEG_SOS_UNMATCH_ERR_INT), each with
 * JPEG_MARKER_ERR_FST_SCAN_INT. The decoder takes one scan holding every
 * component (TRM 37.5.2.1): a scan of fewer components fails the check of
 * the missing ones' IDs. Restart markers are checked against
 * JPEG_RESTART_INTERVAL (TRM 37.5.2.2), a marker other than RST, EOI and
 * SOS raises JPEG_UXP_DET_INT, and the frame's data units are counted
 * against JPEG_HA x JPEG_VA (JPEG_DE_FRAME_EOF_ERR_INT,
 * JPEG_DE_FRAME_EOF_LACK_INT); JPEG_DECODE_TIMEOUT_THRES and
 * JPEG_DECODE_TIMEOUT_TASK_SEL time the decoder out (TRM 37.5.2.5).
 *
 * Tables. The quantisation tables are 4 x 64 entries, written through
 * JPEG_TnQNR_REG (JPEG_QNR_FIFO_EN) or at their addresses, natural
 * (row-major) order, both as ESP-IDF writes them; the Huffman tables, DC0,
 * DC1, AC0 and AC1, the same way (JPEG_DHT_FIFO_EN). TRM 37.5.1.3 gives
 * table n's addresses as base + n * 0x100, over the registers for table
 * 0; the model has them at base + 0x100 + n * 0x100, the four tables
 * filling 0x100 to 0x4FF up to the Huffman tables at 0x500, as the
 * address map leaves room for. Each FIFO port fills its table from entry
 * 0 after JPEG_SOFT_RST or JPEG_FIFO_RST, wrapping at its end. Writes to
 * a table's addresses while its FIFO mode is on, and to a FIFO port while
 * it is off, are dropped and logged; the tables read back at their
 * addresses in either mode. JPEG_SOFT_RST and JPEG_FIFO_RST keep the
 * tables' contents.
 *
 * Errors the TRM names but no stream reaches in this model:
 * JPEG_SR_COLOR_MODE_ERR_INT ("invalid"), and the multi-scan checks
 * (JPEG_SCAN_CHECK_NONE_ERR_INT, JPEG_MARKER_ERR_OTHER_SCAN_INT), as
 * the decoder refuses a frame of several scans at its first. A run
 * that carries an AC coefficient past the data unit's end sets
 * JPEG_RLE_PARALLEL_ERR_INT, the TRM's "run-length encoding error", and
 * decoding goes on; an SOS where coded data or EOI was due raises
 * JPEG_SCAN_CHECK_POS_ERR_INT. An error in the scan header, a restart
 * marker out of place, an unsupported marker or a frame that ends early
 * stops the decoder: it gives no more data, and no EOF if it has not given
 * the frame's last data unit, and waits for JPEG_SOFT_RST, JPEG_FSM_RST
 * or a new start, as ESP-IDF's driver resets it. The decoder knows the
 * frame's last data unit by the frame's size and marks its last byte as
 * the frame's end, err_eof (ESP32P4_JPEG_DMA_ERR_EOF) where a run went
 * past a data unit's end or a code matched no Huffman table entry; EOI
 * follows, or JPEG_DE_FRAME_EOF_LACK_INT where the bitstream ends
 * without it.
 *
 * Timing. The TRM gives only frame rates, for encoding and decoding alike
 * and excluding headers (37.3): 1080p at 40 fps and 720p at 70, at
 * SYS_CLK, which ESP-IDF runs at 200 MHz. It does not say for which
 * formats; for 1080p in YUV420 and 720p in YUV422 the two are one rate
 * of samples to within 3%. The model's pipeline takes 100 cycles of its
 * clock, SYS_CLK as JPEG_SYS_CLK_EN gates it, per data unit: at 200 MHz
 * 1080p YUV420 in 24.5 ms (40.8 fps) and 720p YUV422 in 14.4 ms (69.4
 * fps). An MCU's data units leave as the MCU's last one finishes, and the
 * next MCU starts as its data has arrived and the output has room for
 * it; headers take no time. JPEG_DCT_DONE_INT rises as each MCU
 * finishes, its last data unit's transform done;
 * JPEG_BS_LAST_BLOCK_EOF_INT as the encoder's last MCU does;
 * JPEG_DONE_INT once the frame's last byte has gone to the 2D-DMA.
 * JPEG_PAUSE_EN holds the output (TRM 37.5.1.1), and the pipeline stops
 * when 64 bytes are waiting, the pack FIFO full.
 *
 * Further interpretations, where the TRM is silent:
 *   - Byte orders of the sources. TRM table 37.5-1 names each pixel's
 *     components from the most significant byte of its little-endian
 *     word down, as tables 8.4-8 to 8.4-10 lay out the same formats in
 *     memory for the 2D-DMA: RGB888 "RGB" is B, G, R in memory; RGB565 a
 *     little-endian word, R in bits 15:11; YUV444 "YUV" is V, U, Y;
 *     YUV422 "YUYV", two 16-bit pixels Y over U and Y over V, is U, Y0,
 *     V, Y1; YUV420 "YYU/YYV" is U Y0 Y1 on even rows and V Y0 Y1 on odd
 *     ones. JPEG_PIXEL_REV reverses each pixel's components: R, G, B; B
 *     in bits 15:11; Y, U, V; Y0, U, Y1, V. RGB565's fields widen by
 *     repeating their top bits. ESP-IDF's FOURCC labels (jpeg_types.h)
 *     agree for RGB888, RGB565 and YUV420 and call YUV444 Y, U, V and
 *     YUV422 Y0, V, Y1, U.
 *   - Down-sampling averages the samples it merges, rounding with a bias
 *     that alternates along each row (0 and 1 horizontally, 1 and 2 for
 *     2x2), and a YUV422 source's rows pairwise rounding up.
 *   - JPEG_SAMPLE_SEL 3 ("invalid, no effect") codes YUV444, logged.
 *   - A quantisation entry of 0 divides as 1, logged.
 *   - JPEG_DEBUG_DIRECT_OUT_EN passes each macroblock through as it came.
 *   - The decoder skips bytes that are not markers before SOS, and fill
 *     bytes before any marker; it takes a restart marker within
 *     JPEG_RST_CHECK_BYTE_NUM + 1 bytes of an interval's end, does not
 *     check its number, and takes one after the last interval. A code no
 *     table entry matches is symbol 0.
 *   - The decoder reads its input 256 bytes at a time as it needs it.
 *   - JPEG_MEM_FORCE_PD/PU and JPEG_MEM_CLK_FORCE_ON do not change what
 *     the model does (logged); the debug status registers read their
 *     reset values.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_jpeg.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#define A_CONFIG            0x00
#define A_DQT_INFO          0x04
#define A_PIC_SIZE          0x08
#define A_EXTD_CONFIG       0x0c
#define A_T0QNR             0x10
#define A_T3QNR             0x1c
#define A_DECODE_CONF       0x20
#define A_C0                0x24
#define A_C3                0x30
#define A_DHT_INFO          0x34
#define A_INT_RAW           0x38
#define A_INT_ENA           0x3c
#define A_INT_ST            0x40
#define A_INT_CLR           0x44
#define A_STATUS0           0x48
#define A_STATUS2           0x4c
#define A_STATUS3           0x50
#define A_STATUS4           0x54
#define A_DHT_TOTLEN_DC0    0x58
#define A_DHT_VAL_AC1       0x74
#define A_DHT_CODEMIN_DC0   0x78
#define A_DHT_CODEMIN_AC1   0x84
#define A_DECODER_STATUS0   0x88
#define A_DECODER_STATUS5   0x9c
#define A_STATUS5           0xa0
#define A_ECO_LOW           0xa4
#define A_ECO_HIGH          0xa8
#define A_SYS               0xf8
#define A_VERSION           0xfc
#define A_QNR_RAM           0x100
#define A_DHT_TOTLEN_RAM    0x500
#define A_DHT_VAL_DC_RAM    0x600
#define A_DHT_VAL_AC_RAM    0x680
#define A_DHT_VAL_AC_END    0xe80
#define A_DHT_CODEMIN_RAM   0xf00

/* JPEG_CONFIG_REG. */
#define CFG_FSM_RST         BIT(0)
#define CFG_JPEG_START      BIT(1)
#define CFG_QNR_PRECISION   BIT(2)
#define CFG_FF_CHECK_EN     BIT(3)
#define CFG_SAMPLE_SEL_SH   4
#define CFG_DMA_LINKLIST    BIT(6)
#define CFG_DEBUG_DIRECT    BIT(7)
#define CFG_QNR_FIFO_EN     BIT(8)
#define CFG_LQNR_SEL_SH     9
#define CFG_CQNR_SEL_SH     11
#define CFG_COLOR_SPACE_SH  13
#define CFG_DHT_FIFO_EN     BIT(15)
#define CFG_MEM_CLK_FORCE   BIT(16)
#define CFG_TMO_THRES_SH    17
#define CFG_TMO_TASK_SEL    BIT(23)
#define CFG_SOFT_RST        BIT(24)
#define CFG_FIFO_RST        BIT(25)
#define CFG_PIXEL_REV       BIT(26)
#define CFG_TAILER_EN       BIT(27)
#define CFG_PAUSE_EN        BIT(28)
#define CFG_MEM_FORCE_PD    BIT(29)
#define CFG_MEM_FORCE_PU    BIT(30)
#define CFG_MODE            BIT(31)
#define CFG_RESET           0x00408958u
#define CFG_RW_MASK         0xffffffbcu

#define DQT_INFO_RESET      0x03020100u
#define PIC_SIZE_RESET      0x028001e0u
#define DECODE_CONF_RESET   0x5f030000u
#define DECODE_CONF_RW      0x7effffffu
#define DECODE_CONF_SW_DHT  BIT(24)
#define COMP_RESET          0x00001100u
#define COMP_RW             0x00ffffffu
#define DHT_INFO_RESET      0x00001010u
#define STATUS2_RESET       0x08000000u
#define DECODER_STATUS5_RW  BIT(30)
#define SYS_RW              BIT(31)
#define VERSION_RESET       0x02412260u
#define VERSION_RW          0x0fffffffu

/* The interrupt sources, as JPEG_INT_RAW_REG has them. */
enum {
    INT_DONE = BIT(0),
    INT_RLE_PARALLEL_ERR = BIT(1),
    INT_CID_ERR = BIT(2),
    INT_C_DHT_DC_ID_ERR = BIT(3),
    INT_C_DHT_AC_ID_ERR = BIT(4),
    INT_C_DQT_ID_ERR = BIT(5),
    INT_RST_UXP_ERR = BIT(6),
    INT_RST_CHECK_NONE_ERR = BIT(7),
    INT_RST_CHECK_POS_ERR = BIT(8),
    INT_OUT_EOF = BIT(9),
    INT_DCT_DONE = BIT(11),
    INT_BS_LAST_BLOCK_EOF = BIT(12),
    INT_SCAN_CHECK_POS_ERR = BIT(14),
    INT_UXP_DET = BIT(15),
    INT_EN_FRAME_EOF_ERR = BIT(16),
    INT_EN_FRAME_EOF_LACK = BIT(17),
    INT_DE_FRAME_EOF_ERR = BIT(18),
    INT_DE_FRAME_EOF_LACK = BIT(19),
    INT_SOS_UNMATCH_ERR = BIT(20),
    INT_MARKER_ERR_FST_SCAN = BIT(21),
    INT_UNDET = BIT(23),
    INT_DECODE_TIMEOUT = BIT(24),
};
#define INT_MASK            0x01ffffffu

/* The frame's phases. */
enum {
    PH_IDLE,
    PH_ENC,             /* encoding MCUs */
    PH_SOS,             /* looking for the scan header */
    PH_MCU,             /* decoding MCUs */
    PH_RST,             /* expecting a restart marker */
    PH_EOI,             /* expecting EOI */
    PH_DRAIN,           /* the frame's output all made, going out */
    PH_HALT,            /* stopped by an error */
};

/* Coded formats. */
enum {
    FMT_YUV444,
    FMT_YUV422,
    FMT_YUV420,
    FMT_GRAY,
};

/* Encoder source formats. */
enum {
    SRC_RGB888,
    SRC_YUV422,
    SRC_RGB565,
    SRC_GRAY,
    SRC_YUV444,
    SRC_YUV420,
};

#define CYCLES_PER_UNIT     100
#define OUT_PACK_FIFO       64
#define DEC_PULL            256

static ESP32P4JpegHuffCode enc_dc[2], enc_ac[2];

static const struct {
    uint8_t w, h, units;
} fmt_mcu[] = {
    [FMT_YUV444] = { 8, 8, 3 },
    [FMT_YUV422] = { 16, 8, 4 },
    [FMT_YUV420] = { 16, 16, 6 },
    [FMT_GRAY] = { 8, 8, 1 },
};

/* Bytes per pixel of each source, in halves. */
static const uint8_t src_half_bytes[] = {
    [SRC_RGB888] = 6,
    [SRC_YUV422] = 4,
    [SRC_RGB565] = 4,
    [SRC_GRAY] = 2,
    [SRC_YUV444] = 6,
    [SRC_YUV420] = 3,
};

static void jpeg_run(ESP32P4JpegState *s);

static bool jpeg_decoding(ESP32P4JpegState *s)
{
    return s->phase >= PH_SOS && s->phase <= PH_EOI;
}

static void jpeg_update_irq(ESP32P4JpegState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_raise(ESP32P4JpegState *s, uint32_t bits)
{
    s->int_raw |= bits;
    jpeg_update_irq(s);
}

static uint32_t mcu_units(ESP32P4JpegState *s)
{
    return fmt_mcu[s->format].units;
}

static uint32_t mb_bytes(ESP32P4JpegState *s)
{
    return fmt_mcu[s->format].w * fmt_mcu[s->format].h *
           src_half_bytes[s->src] / 2;
}

/* The 2D-DMA's paths, or the qtest's stand-in for them. */

static size_t qt_pull(ESP32P4JpegState *s, uint8_t *buf, size_t len,
                      bool *eof)
{
    size_t got = 0;

    *eof = false;
    while (got < len && s->qt_tx_num) {
        ESP32P4JpegQtSeg *g = &s->qt_tx[0];
        size_t n = MIN(len - got, g->len - g->done);

        address_space_read(&address_space_memory, g->addr + g->done,
                           MEMTXATTRS_UNSPECIFIED, buf + got, n);
        got += n;
        g->done += n;
        if (g->done == g->len) {
            *eof = g->eof;
            memmove(&s->qt_tx[0], &s->qt_tx[1],
                    --s->qt_tx_num * sizeof(s->qt_tx[0]));
            if (*eof) {
                break;
            }
        }
    }
    return got;
}

static size_t qt_push(ESP32P4JpegState *s, const uint8_t *buf, size_t len,
                      unsigned flags)
{
    size_t put = 0;

    while (put < len && s->qt_rx_num) {
        ESP32P4JpegQtSeg *g = &s->qt_rx[0];
        size_t n = MIN(len - put, g->len - g->done);

        address_space_write(&address_space_memory, g->addr + g->done,
                            MEMTXATTRS_UNSPECIFIED, buf + put, n);
        put += n;
        g->done += n;
        s->qt_rx_total += n;
        if (g->done == g->len) {
            memmove(&s->qt_rx[0], &s->qt_rx[1],
                    --s->qt_rx_num * sizeof(s->qt_rx[0]));
        }
    }
    if (put == len) {
        s->qt_rx_flags |= flags;
    }
    return put;
}

static size_t dma_pull(ESP32P4JpegState *s, uint8_t *buf, size_t len,
                       bool *eof)
{
    *eof = false;
    if (!len) {
        return 0;
    }
    if (s->dma_ops) {
        return s->dma_ops->pull(s->dma_opaque, buf, len, eof);
    }
    return qt_pull(s, buf, len, eof);
}

static size_t dma_push(ESP32P4JpegState *s, const uint8_t *buf, size_t len,
                       unsigned flags)
{
    if (s->dma_ops) {
        return s->dma_ops->push(s->dma_opaque, buf, len, flags);
    }
    return qt_push(s, buf, len, flags);
}

/* The frame. */

static void jpeg_clear_data(ESP32P4JpegState *s)
{
    s->in_len = 0;
    s->in_pos = 0;
    s->in_eof = false;
    s->stage_len = 0;
    s->out_head = 0;
    s->out_num = 0;
    s->out_final = false;
    s->out_flags = 0;
    s->bit_acc = 0;
    s->bit_count = 0;
}

/* The frame stops: what is in progress is dropped. */
static void jpeg_abort(ESP32P4JpegState *s)
{
    esp32p4_clkwait_stop(&s->work);
    esp32p4_clkwait_stop(&s->tmo_stall);
    esp32p4_clkwait_stop(&s->tmo_frame);
    s->phase = PH_IDLE;
    jpeg_clear_data(s);
}

/*
 * An error stops the frame: the MCU in the pipeline is dropped, and what
 * has reached the output FIFO still leaves, marked as the frame's end
 * only if the frame's last data unit is among it.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_halt(ESP32P4JpegState *s, uint32_t bits)
{
    esp32p4_clkwait_stop(&s->work);
    esp32p4_clkwait_stop(&s->tmo_stall);
    esp32p4_clkwait_stop(&s->tmo_frame);
    s->stage_len = 0;
    s->phase = PH_HALT;
    jpeg_raise(s, bits);
}

static void jpeg_fifo_ptrs_reset(ESP32P4JpegState *s)
{
    memset(s->qnr_ptr, 0, sizeof(s->qnr_ptr));
    memset(s->totlen_ptr, 0, sizeof(s->totlen_ptr));
    memset(s->codemin_ptr, 0, sizeof(s->codemin_ptr));
    memset(s->val_ptr, 0, sizeof(s->val_ptr));
}

static void jpeg_out_append(ESP32P4JpegState *s, const uint8_t *buf,
                            uint32_t len)
{
    assert(s->out_num + len <= ESP32P4_JPEG_OUT_SIZE);
    for (uint32_t i = 0; i < len; i++) {
        s->out[(s->out_head + s->out_num++) % ESP32P4_JPEG_OUT_SIZE] = buf[i];
    }
}

/* Give the 2D-DMA what it takes of the output FIFO. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static bool jpeg_push(ESP32P4JpegState *s)
{
    bool progress = false;

    while (s->out_num && !(s->config & CFG_PAUSE_EN)) {
        uint32_t chunk = MIN(s->out_num, ESP32P4_JPEG_OUT_SIZE - s->out_head);
        bool last = chunk == s->out_num && s->out_final;
        size_t n = dma_push(s, &s->out[s->out_head], chunk,
                            last ? s->out_flags : 0);

        if (!n) {
            break;
        }
        progress = true;
        s->out_head = (s->out_head + n) % ESP32P4_JPEG_OUT_SIZE;
        s->out_num -= n;
        if (n < chunk) {
            break;
        }
    }
    return progress;
}

/* Encoder. */

/*
 * One macroblock's samples: Y at full size, U and V at the source's own
 * resolution (@cw x @ch).
 */
typedef struct EncPlanes {
    uint8_t y[16 * 16];
    uint8_t u[16 * 16];
    uint8_t v[16 * 16];
    int cw;
    int ch;
} EncPlanes;

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void enc_planes(ESP32P4JpegState *s, const uint8_t *mb, EncPlanes *p)
{
    int w = fmt_mcu[s->format].w, h = fmt_mcu[s->format].h;

    p->cw = w;
    p->ch = h;
    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            int i = r * w + c;
            const uint8_t *px;
            int rr, gg, bb;
            uint16_t word;

            switch (s->src) {
            case SRC_RGB888:
                px = mb + 3 * i;
                rr = s->pixel_rev ? px[0] : px[2];
                gg = px[1];
                bb = s->pixel_rev ? px[2] : px[0];
                esp32p4_jpeg_rgb_to_ycc(rr, gg, bb, &p->y[i], &p->u[i],
                                        &p->v[i]);
                break;
            case SRC_RGB565:
                word = lduw_le_p(mb + 2 * i);
                rr = extract32(word, 11, 5);
                gg = extract32(word, 5, 6);
                bb = extract32(word, 0, 5);
                if (s->pixel_rev) {
                    int t = rr;

                    rr = bb;
                    bb = t;
                }
                rr = (rr << 3) | (rr >> 2);
                gg = (gg << 2) | (gg >> 4);
                bb = (bb << 3) | (bb >> 2);
                esp32p4_jpeg_rgb_to_ycc(rr, gg, bb, &p->y[i], &p->u[i],
                                        &p->v[i]);
                break;
            case SRC_YUV444:
                /* V, U, Y; Y, U, V reversed. */
                px = mb + 3 * i;
                p->y[i] = s->pixel_rev ? px[0] : px[2];
                p->u[i] = px[1];
                p->v[i] = s->pixel_rev ? px[2] : px[0];
                break;
            case SRC_YUV422:
                /* U Y0 V Y1, or Y0 U Y1 V reversed, per pixel pair. */
                px = mb + 4 * (i / 2);
                p->y[i] = px[(s->pixel_rev ? 0 : 1) + 2 * (c & 1)];
                if (!(c & 1)) {
                    int j = r * (w / 2) + c / 2;

                    p->u[j] = s->pixel_rev ? px[1] : px[0];
                    p->v[j] = s->pixel_rev ? px[3] : px[2];
                }
                break;
            case SRC_YUV420:
                /* U Y Y on even rows, V Y Y on odd ones. */
                px = mb + r * (w * 3 / 2) + 3 * (c / 2);
                p->y[i] = px[1 + (c & 1)];
                if (!(c & 1)) {
                    int j = (r / 2) * (w / 2) + c / 2;

                    if (r & 1) {
                        p->v[j] = px[0];
                    } else {
                        p->u[j] = px[0];
                    }
                }
                break;
            default:
                p->y[i] = mb[i];
                break;
            }
        }
    }
    if (s->src == SRC_YUV422) {
        p->cw = w / 2;
    } else if (s->src == SRC_YUV420) {
        p->cw = w / 2;
        p->ch = h / 2;
    }
}

/* One 8x8 chroma data unit of the coded format from a plane. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void enc_chroma(const uint8_t *plane, int cw, int ch, uint8_t out[64])
{
    int sx = cw / 8, sy = ch / 8;

    for (int r = 0; r < 8; r++) {
        int bias = sx == 2 && sy == 2 ? 1 : 0;

        for (int c = 0; c < 8; c++) {
            const uint8_t *p = plane + (r * sy) * cw + c * sx;

            if (sx == 2 && sy == 2) {
                out[8 * r + c] = (p[0] + p[1] + p[cw] + p[cw + 1] + bias) >> 2;
                bias ^= 3;
            } else if (sx == 2) {
                out[8 * r + c] = (p[0] + p[1] + bias) >> 1;
                bias ^= 1;
            } else if (sy == 2) {
                out[8 * r + c] = (p[0] + p[cw] + 1) >> 1;
            } else {
                out[8 * r + c] = p[0];
            }
        }
    }
}

static void enc_unit(ESP32P4JpegState *s, ESP32P4JpegBitOut *o,
                     const uint8_t samples[64], int comp)
{
    int t = comp ? 1 : 0;
    int32_t blk[64], q[64];

    for (int i = 0; i < 64; i++) {
        blk[i] = samples[i] - 128;
    }
    esp32p4_jpeg_fdct(blk);
    for (int i = 0; i < 64; i++) {
        q[i] = esp32p4_jpeg_quantize(blk[i], s->enc_q[t][i]);
    }
    esp32p4_jpeg_encode_block(o, q, &s->pred[comp], &enc_dc[t], &enc_ac[t]);
}

/* Code one macroblock into the stage, and the frame's end after the last. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void enc_mcu(ESP32P4JpegState *s, const uint8_t *mb, bool last)
{
    ESP32P4JpegBitOut o = {
        .buf = s->stage,
        .cap = sizeof(s->stage),
        .acc = s->bit_acc,
        .nbits = s->bit_count,
        .stuff = s->ff_check,
    };
    int w = fmt_mcu[s->format].w;
    uint8_t unit[64];
    EncPlanes p;

    if (s->direct) {
        s->stage_len = mb_bytes(s);
        memcpy(s->stage, mb, s->stage_len);
        return;
    }
    enc_planes(s, mb, &p);
    for (int by = 0; by < fmt_mcu[s->format].h / 8; by++) {
        for (int bx = 0; bx < w / 8; bx++) {
            for (int r = 0; r < 8; r++) {
                memcpy(unit + 8 * r, p.y + (by * 8 + r) * w + bx * 8, 8);
            }
            enc_unit(s, &o, unit, 0);
        }
    }
    if (s->format != FMT_GRAY) {
        enc_chroma(p.u, p.cw, p.ch, unit);
        enc_unit(s, &o, unit, 1);
        enc_chroma(p.v, p.cw, p.ch, unit);
        enc_unit(s, &o, unit, 2);
    }
    if (last) {
        esp32p4_jpeg_put_pad(&o);
        if (s->tailer) {
            if (o.len + 2 <= o.cap) {
                o.buf[o.len] = 0xff;
                o.buf[o.len + 1] = 0xd9;
            }
            o.len += 2;
        }
    }
    assert(o.len <= o.cap);
    s->stage_len = o.len;
    s->bit_acc = o.acc;
    s->bit_count = o.nbits;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static bool enc_step(ESP32P4JpegState *s, int64_t now)
{
    uint32_t need = mb_bytes(s);
    bool eof, last = s->mcu + 1 == s->mcus;

    if (s->out_num > OUT_PACK_FIFO) {
        return false;
    }
    if (s->in_len < need) {
        size_t n = dma_pull(s, s->in_buf + s->in_len, need - s->in_len,
                            &eof);

        s->in_len += n;
        if (eof) {
            s->in_eof = true;
            jpeg_raise(s, INT_OUT_EOF);
        }
        if (s->in_len < need) {
            if (s->in_eof) {
                jpeg_halt(s, INT_EN_FRAME_EOF_ERR);
                return true;
            }
            return n > 0;
        }
    }
    if (s->in_eof && !last) {
        jpeg_halt(s, INT_EN_FRAME_EOF_ERR);
        return true;
    }
    if (last && !s->in_eof) {
        jpeg_raise(s, INT_EN_FRAME_EOF_LACK);
    }
    enc_mcu(s, s->in_buf, last);
    s->in_len = 0;
    esp32p4_clkwait_start(&s->work, now, CYCLES_PER_UNIT * mcu_units(s));
    return true;
}

/* Decoder. */

/*
 * Move what has been parsed out of the input FIFO, then take up to
 * DEC_PULL more bytes of the bitstream; the bytes taken.
 */
static size_t dec_fill(ESP32P4JpegState *s)
{
    size_t n;
    bool eof;

    if (s->in_pos) {
        memmove(s->in_buf, s->in_buf + s->in_pos, s->in_len - s->in_pos);
        s->in_len -= s->in_pos;
        s->in_pos = 0;
    }
    if (s->in_eof || s->in_len == ESP32P4_JPEG_IN_SIZE) {
        return 0;
    }
    n = dma_pull(s, s->in_buf + s->in_len,
                 MIN(DEC_PULL, ESP32P4_JPEG_IN_SIZE - s->in_len), &eof);
    s->in_len += n;
    if (n) {
        /* Reading the bitstream restarts the stall's count. */
        esp32p4_clkwait_stop(&s->tmo_stall);
    }
    if (eof) {
        s->in_eof = true;
        jpeg_raise(s, INT_OUT_EOF);
    }
    return n;
}

/* The decoder meets a marker it did not expect in coded data. */
static void dec_marker(ESP32P4JpegState *s, uint8_t m)
{
    if (m == 0xd9) {
        jpeg_halt(s, INT_DE_FRAME_EOF_ERR);
    } else if (m >= 0xd0 && m <= 0xd7) {
        jpeg_halt(s, (s->restart ? INT_RST_CHECK_POS_ERR : INT_RST_UXP_ERR));
    } else if (m == 0xda) {
        jpeg_halt(s, INT_SCAN_CHECK_POS_ERR);
    } else {
        jpeg_halt(s, INT_UXP_DET);
    }
}

static void jpeg_build_hdec(ESP32P4JpegState *s)
{
    for (int t = 0; t < 2; t++) {
        esp32p4_jpeg_huff_dec_build(&s->hdec[0][t],
                                    s->dht_totlen[ESP32P4_JPEG_HT(0, t)],
                                    s->dht_codemin[ESP32P4_JPEG_HT(0, t)],
                                    s->dht_val_dc[t], 16);
        esp32p4_jpeg_huff_dec_build(&s->hdec[1][t],
                                    s->dht_totlen[ESP32P4_JPEG_HT(1, t)],
                                    s->dht_codemin[ESP32P4_JPEG_HT(1, t)],
                                    s->dht_val_ac[t], 256);
    }
}

/*
 * Check the scan header @seg, @ls bytes after its marker, against the
 * registers.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void dec_sos(ESP32P4JpegState *s, const uint8_t *seg, uint32_t ls)
{
    uint32_t ns = seg[4], ncomp = s->ncomp;
    uint32_t err = 0;

    if (!ns || ls != 6 + 2 * ns) {
        jpeg_halt(s, INT_SOS_UNMATCH_ERR | INT_MARKER_ERR_FST_SCAN);
        return;
    }
    if (ns != ncomp) {
        err |= INT_CID_ERR;
    }
    for (uint32_t j = 0; j < MIN(ns, ncomp); j++) {
        uint32_t cs = seg[5 + 2 * j], td = seg[6 + 2 * j] >> 4;
        uint32_t ta = seg[6 + 2 * j] & 15, qsel = s->comp[j] & 0xff;
        bool found = false;

        if (cs != extract32(s->comp[j], 16, 8)) {
            err |= INT_CID_ERR;
        }
        if (td == extract32(s->dht_info, 0, 4)) {
            s->dc_sel[j] = 0;
        } else if (td == extract32(s->dht_info, 4, 4)) {
            s->dc_sel[j] = 1;
        } else {
            err |= INT_C_DHT_DC_ID_ERR;
        }
        if (ta == extract32(s->dht_info, 8, 4)) {
            s->ac_sel[j] = 0;
        } else if (ta == extract32(s->dht_info, 12, 4)) {
            s->ac_sel[j] = 1;
        } else {
            err |= INT_C_DHT_AC_ID_ERR;
        }
        for (int k = 0; k < 4 && !found; k++) {
            uint32_t info = extract32(s->dqt_info, 8 * k, 8);

            if ((info & 15) == qsel) {
                s->q_sel[j] = k;
                s->qmask[j] = (info >> 4) ? 0xffff : 0xff;
                found = true;
            }
        }
        if (!found) {
            err |= INT_C_DQT_ID_ERR;
        }
    }
    if (err) {
        jpeg_halt(s, err | INT_MARKER_ERR_FST_SCAN);
        return;
    }
    jpeg_build_hdec(s);
    memset(s->pred, 0, sizeof(s->pred));
    s->bit_acc = 0;
    s->bit_count = 0;
    s->phase = PH_MCU;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static bool dec_sos_step(ESP32P4JpegState *s)
{
    for (;;) {
        uint32_t avail = s->in_len - s->in_pos, ls;
        const uint8_t *p = s->in_buf + s->in_pos;

        if (avail >= 1 && p[0] != 0xff) {
            s->in_pos++;
            continue;
        }
        if (avail >= 2 && (p[1] == 0xff || p[1] == 0x00)) {
            s->in_pos += p[1] == 0xff ? 1 : 2;
            continue;
        }
        if (avail >= 2 && p[1] != 0xda) {
            jpeg_halt(s, INT_UXP_DET);
            return true;
        }
        if (avail >= 5) {
            ls = lduw_be_p(p + 2);
            if (!p[4] || ls != 6 + 2 * p[4]) {
                jpeg_halt(s, INT_SOS_UNMATCH_ERR | INT_MARKER_ERR_FST_SCAN);
                return true;
            }
            if (avail >= 2 + ls) {
                s->in_pos += 2 + ls;
                dec_sos(s, p, ls);
                return true;
            }
        }
        if (dec_fill(s)) {
            continue;
        }
        if (s->in_eof) {
            jpeg_halt(s, s->in_len - s->in_pos >= 2 ?
                      INT_SOS_UNMATCH_ERR | INT_MARKER_ERR_FST_SCAN :
                      INT_UNDET);
            return true;
        }
        return false;
    }
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static bool dec_mcu_step(ESP32P4JpegState *s, int64_t now)
{
    uint32_t ncomp = s->ncomp, flags = 0;
    ESP32P4JpegDecComp comp[3];
    int32_t pred[4];
    int r;

    if (s->out_num > OUT_PACK_FIFO) {
        return false;
    }
    for (uint32_t j = 0; j < ncomp; j++) {
        comp[j] = (ESP32P4JpegDecComp) {
            .dc = &s->hdec[0][s->dc_sel[j]],
            .ac = &s->hdec[1][s->ac_sel[j]],
            .q = s->qnr[s->q_sel[j]],
            .qmask = s->qmask[j],
            .units = j ? 1 : mcu_units(s) - (ncomp - 1),
        };
    }
    for (;;) {
        ESP32P4JpegBitIn in = {
            .buf = s->in_buf,
            .len = s->in_len,
            .pos = s->in_pos,
            .acc = s->bit_acc,
            .nbits = s->bit_count,
            .eof = s->in_eof,
        };

        memcpy(pred, s->pred, sizeof(pred));
        r = esp32p4_jpeg_decode_mcu(&in, comp, ncomp, pred, s->stage,
                                    &flags);
        switch (r) {
        case ESP32P4_JPEG_DEC_OK:
            memcpy(s->pred, pred, sizeof(pred));
            s->in_pos = in.pos;
            s->bit_acc = in.acc;
            s->bit_count = in.nbits;
            s->stage_len = 64 * mcu_units(s);
            if (flags & ESP32P4_JPEG_DEC_RUN_OVERFLOW) {
                jpeg_raise(s, INT_RLE_PARALLEL_ERR);
            }
            if (flags) {
                s->frame_err = true;
            }
            esp32p4_clkwait_start(&s->work, now,
                                  CYCLES_PER_UNIT * mcu_units(s));
            return true;
        case ESP32P4_JPEG_DEC_MORE:
            flags = 0;
            if (!dec_fill(s)) {
                return false;
            }
            continue;
        case ESP32P4_JPEG_DEC_END:
            jpeg_halt(s, INT_DE_FRAME_EOF_ERR);
            return true;
        default:
            dec_marker(s, s->in_buf[in.pos + 1]);
            return true;
        }
    }
}

/*
 * After an interval's last MCU: the restart marker, within
 * JPEG_RST_CHECK_BYTE_NUM + 1 bytes.
 */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static bool dec_rst_step(ESP32P4JpegState *s)
{
    uint32_t window = extract32(s->decode_conf, 27, 2) + 1, skipped = 0;
    uint32_t i = 0;

    s->bit_acc = 0;
    s->bit_count = 0;
    for (;;) {
        const uint8_t *p = s->in_buf + s->in_pos;
        uint32_t avail = s->in_len - s->in_pos;

        if (i + 1 < avail) {
            if (p[i] == 0xff && p[i + 1] >= 0xd0 && p[i + 1] <= 0xd7) {
                s->in_pos += i + 2;
                memset(s->pred, 0, sizeof(s->pred));
                s->phase = PH_MCU;
                return true;
            }
            if (p[i] == 0xff && p[i + 1] == 0xff) {
                i++;
                continue;
            }
            if (p[i] == 0xff && p[i + 1]) {
                jpeg_halt(s, INT_RST_CHECK_NONE_ERR);
                return true;
            }
            if (++skipped >= window) {
                jpeg_halt(s, INT_RST_CHECK_NONE_ERR);
                return true;
            }
            i++;
            continue;
        }
        if (dec_fill(s)) {
            continue;
        }
        if (s->in_eof) {
            jpeg_halt(s, INT_RST_CHECK_NONE_ERR);
            return true;
        }
        return false;
    }
}

static void dec_complete(ESP32P4JpegState *s)
{
    esp32p4_clkwait_stop(&s->tmo_stall);
    esp32p4_clkwait_stop(&s->tmo_frame);
    s->phase = PH_DRAIN;
}

/* After the frame's last MCU: EOI. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static bool dec_eoi_step(ESP32P4JpegState *s)
{
    s->bit_acc = 0;
    s->bit_count = 0;
    for (;;) {
        const uint8_t *p = s->in_buf + s->in_pos;
        uint32_t avail = s->in_len - s->in_pos;

        if (avail >= 1 && p[0] != 0xff) {
            jpeg_halt(s, INT_DE_FRAME_EOF_ERR);
            return true;
        }
        if (avail >= 2) {
            if (p[1] == 0xff) {
                s->in_pos++;
            } else if (p[1] == 0xd9) {
                s->in_pos += 2;
                dec_complete(s);
                return true;
            } else if (p[1] >= 0xd0 && p[1] <= 0xd7) {
                s->in_pos += 2;
            } else if (p[1] == 0xda) {
                jpeg_halt(s, INT_SCAN_CHECK_POS_ERR);
                return true;
            } else if (p[1] == 0x00) {
                jpeg_halt(s, INT_DE_FRAME_EOF_ERR);
                return true;
            } else {
                jpeg_halt(s, INT_UXP_DET);
                return true;
            }
            continue;
        }
        if (dec_fill(s)) {
            continue;
        }
        if (s->in_eof) {
            jpeg_raise(s, INT_DE_FRAME_EOF_LACK);
            dec_complete(s);
            return true;
        }
        return false;
    }
}

/* The pipeline's MCU is done: its data units go to the output FIFO. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_work_done_at(void *opaque)
{
    ESP32P4JpegState *s = opaque;
    bool enc = s->phase == PH_ENC;

    jpeg_out_append(s, s->stage, s->stage_len);
    s->stage_len = 0;
    s->mcu++;
    jpeg_raise(s, INT_DCT_DONE);
    if (enc) {
        if (s->mcu == s->mcus) {
            jpeg_raise(s, INT_BS_LAST_BLOCK_EOF);
            s->out_final = true;
            s->out_flags = ESP32P4_JPEG_DMA_EOF;
            s->phase = PH_DRAIN;
        }
    } else if (s->mcu == s->mcus) {
        s->out_final = true;
        s->out_flags = s->frame_err ? ESP32P4_JPEG_DMA_ERR_EOF
                                    : ESP32P4_JPEG_DMA_EOF;
        s->phase = PH_EOI;
    } else if (s->restart && !(s->mcu % s->restart)) {
        s->phase = PH_RST;
    }
    jpeg_run(s);
}

static void jpeg_work_done(void *opaque)
{
    ESP32P4JpegState *s = opaque;

    if (esp32p4_clkwait_active(&s->work)) {
        esp32_line_run_at(esp32p4_clkwait_expire(&s->work),
                          jpeg_work_done_at, s);
    }
}

/* JPEG_DECODE_TIMEOUT_INT; the decoder resets with TASK_SEL. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_timeout_at(void *opaque)
{
    ESP32P4JpegState *s = opaque;

    jpeg_raise(s, INT_DECODE_TIMEOUT);
    if (s->config & CFG_TMO_TASK_SEL) {
        jpeg_abort(s);
    }
}

static void jpeg_tmo_stall_cb(void *opaque)
{
    ESP32P4JpegState *s = opaque;

    if (esp32p4_clkwait_active(&s->tmo_stall)) {
        esp32_line_run_at(esp32p4_clkwait_expire(&s->tmo_stall),
                          jpeg_timeout_at, s);
    }
}

static void jpeg_tmo_frame_cb(void *opaque)
{
    ESP32P4JpegState *s = opaque;

    if (esp32p4_clkwait_active(&s->tmo_frame)) {
        esp32p4_clkwait_stop(&s->tmo_stall);
        esp32_line_run_at(esp32p4_clkwait_expire(&s->tmo_frame),
                          jpeg_timeout_at, s);
    }
}

static uint64_t tmo_cycles(ESP32P4JpegState *s)
{
    uint32_t thres = extract32(s->config, CFG_TMO_THRES_SH, 6);

    return (UINT64_C(1) << thres) - 1;
}

static bool jpeg_in_reset(ESP32P4JpegState *s)
{
    return s->config & CFG_SOFT_RST;
}

/*
 * Run the frame as far as its data, the output's room and the pipeline
 * let it, as of esp32_line_time().
 */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_run(ESP32P4JpegState *s)
{
    int64_t now = esp32_line_time();
    bool progress;

    if (!clock_is_enabled(s->clk) || jpeg_in_reset(s)) {
        return;
    }
    if (s->running) {
        s->rerun = true;
        return;
    }
    s->running = true;
    do {
        s->rerun = false;
        progress = jpeg_push(s);
        if (esp32p4_clkwait_active(&s->work)) {
            continue;
        }
        switch (s->phase) {
        case PH_ENC:
            progress |= enc_step(s, now);
            break;
        case PH_SOS:
            progress |= dec_sos_step(s);
            break;
        case PH_MCU:
            progress |= dec_mcu_step(s, now);
            break;
        case PH_RST:
            progress |= dec_rst_step(s);
            break;
        case PH_EOI:
            progress |= dec_eoi_step(s);
            break;
        case PH_DRAIN:
            if (!s->out_num) {
                s->phase = PH_IDLE;
                s->out_final = false;
                jpeg_raise(s, INT_DONE);
            }
            break;
        default:
            break;
        }
    } while (progress || s->rerun);
    s->running = false;

    /* A decoder waiting for its input or its output's room times out. */
    if (jpeg_decoding(s) && !esp32p4_clkwait_active(&s->work)) {
        if (!esp32p4_clkwait_active(&s->tmo_stall)) {
            esp32p4_clkwait_start(&s->tmo_stall, now, tmo_cycles(s));
        }
    } else {
        esp32p4_clkwait_stop(&s->tmo_stall);
    }
}

/* Run what fell due by @now and has not run yet, at its own time. */
static void jpeg_catch_up(ESP32P4JpegState *s, int64_t now)
{
    for (;;) {
        if (esp32p4_clkwait_active(&s->work) &&
            esp32p4_clkwait_end(&s->work) <= now) {
            jpeg_work_done(s);
        } else if (esp32p4_clkwait_active(&s->tmo_frame) &&
                   esp32p4_clkwait_end(&s->tmo_frame) <= now) {
            jpeg_tmo_frame_cb(s);
        } else if (esp32p4_clkwait_active(&s->tmo_stall) &&
                   esp32p4_clkwait_end(&s->tmo_stall) <= now) {
            jpeg_tmo_stall_cb(s);
        } else {
            return;
        }
    }
}

/* JPEG_JPEG_START. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_start(ESP32P4JpegState *s)
{
    uint32_t cfg = s->config, ha, va, mw, mh;
    int64_t now = esp32_line_time();

    if (s->phase != PH_IDLE && s->phase != PH_HALT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: JPEG_JPEG_START while a frame is "
                      "in progress is ignored\n", TYPE_ESP32P4_JPEG);
        return;
    }
    if (cfg & CFG_FIFO_RST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: JPEG_JPEG_START while "
                      "JPEG_FIFO_RST holds the FIFOs is ignored\n",
                      TYPE_ESP32P4_JPEG);
        return;
    }
    jpeg_abort(s);
    if (cfg & (CFG_MEM_FORCE_PD | CFG_MEM_FORCE_PU | CFG_MEM_CLK_FORCE)) {
        qemu_log_mask(LOG_UNIMP, "%s: the memories' power and clock "
                      "controls are not modelled\n", TYPE_ESP32P4_JPEG);
    }
    s->frame_err = false;
    s->mcu = 0;
    memset(s->pred, 0, sizeof(s->pred));

    if (!(cfg & CFG_MODE)) {
        uint32_t cs = extract32(cfg, CFG_COLOR_SPACE_SH, 2);
        uint32_t sel = extract32(cfg, CFG_SAMPLE_SEL_SH, 2);
        uint32_t qmask = cfg & CFG_QNR_PRECISION ? 0xffff : 0xff;

        ha = extract32(s->pic_size, 16, 14);
        va = extract32(s->pic_size, 0, 14);
        if (s->extd_config & 1) {
            s->src = s->extd_config & 2 ? SRC_YUV420 : SRC_YUV444;
        } else {
            static const uint32_t cs_src[] = {
                SRC_RGB888, SRC_YUV422, SRC_RGB565, SRC_GRAY,
            };

            s->src = cs_src[cs];
        }
        switch (s->src) {
        case SRC_GRAY:
            s->format = FMT_GRAY;
            break;
        case SRC_YUV420:
            s->format = FMT_YUV420;
            break;
        case SRC_YUV422:
            s->format = sel == 2 ? FMT_YUV420 : FMT_YUV422;
            break;
        default:
            if (sel == 3) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: JPEG_SAMPLE_SEL 3 is "
                              "invalid; coding YUV444\n", TYPE_ESP32P4_JPEG);
            }
            s->format = sel == 1 ? FMT_YUV422 : sel == 2 ? FMT_YUV420
                                                         : FMT_YUV444;
            break;
        }
        for (int t = 0; t < 2; t++) {
            uint32_t sh = t ? CFG_CQNR_SEL_SH : CFG_LQNR_SEL_SH;
            const uint16_t *tbl = s->qnr[extract32(cfg, sh, 2)];

            for (int i = 0; i < 64; i++) {
                s->enc_q[t][i] = tbl[i] & qmask;
                if (!s->enc_q[t][i]) {
                    qemu_log_mask(LOG_GUEST_ERROR, "%s: quantisation table "
                                  "%u entry %d is 0; dividing by 1\n",
                                  TYPE_ESP32P4_JPEG, extract32(cfg, sh, 2),
                                  i);
                    s->enc_q[t][i] = 1;
                }
            }
        }
        s->pixel_rev = cfg & CFG_PIXEL_REV;
        s->ff_check = cfg & CFG_FF_CHECK_EN;
        s->tailer = cfg & CFG_TAILER_EN;
        s->direct = cfg & CFG_DEBUG_DIRECT;
    } else {
        uint32_t ncomp = extract32(s->decode_conf, 16, 8);
        uint32_t f0 = extract32(s->comp[0], 8, 8);
        bool ones = extract32(s->comp[1], 8, 8) == 0x11 &&
                    extract32(s->comp[2], 8, 8) == 0x11;

        ha = extract32(s->pic_size, 16, 16);
        va = extract32(s->pic_size, 0, 16);
        if (ncomp == 1 && f0 == 0x11) {
            s->format = FMT_GRAY;
        } else if (ncomp == 3 && ones && f0 == 0x11) {
            s->format = FMT_YUV444;
        } else if (ncomp == 3 && ones && f0 == 0x21) {
            s->format = FMT_YUV422;
        } else if (ncomp == 3 && ones && f0 == 0x22) {
            s->format = FMT_YUV420;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: JPEG_COMPONENT_NUM %u with "
                          "these sampling factors is no format the decoder "
                          "has (TRM table 37.5-5); not started\n",
                          TYPE_ESP32P4_JPEG, ncomp);
            return;
        }
        s->restart = extract32(s->decode_conf, 0, 16);
        s->ncomp = ncomp;
    }
    mw = fmt_mcu[s->format].w;
    mh = fmt_mcu[s->format].h;
    if (!ha || !va) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: an image of %ux%u; not started\n",
                      TYPE_ESP32P4_JPEG, ha, va);
        return;
    }
    if ((cfg & CFG_MODE) && (ha % mw || va % mh)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: JPEG_HA %u and JPEG_VA %u must be "
                      "multiples of the MCU's %ux%u (TRM table 37.7-3)\n",
                      TYPE_ESP32P4_JPEG, ha, va, mw, mh);
    }
    s->mcus = DIV_ROUND_UP(ha, mw) * DIV_ROUND_UP(va, mh);
    if (cfg & CFG_MODE) {
        static const int mcu_format[] = {
            [FMT_YUV444] = ESP32P4_JPEG_MCU_YUV444,
            [FMT_YUV422] = ESP32P4_JPEG_MCU_YUV422,
            [FMT_YUV420] = ESP32P4_JPEG_MCU_YUV420,
            [FMT_GRAY] = ESP32P4_JPEG_MCU_GRAY,
        };

        if (s->dma_ops && s->dma_ops->rx_format) {
            s->dma_ops->rx_format(s->dma_opaque, mcu_format[s->format]);
        }
        s->phase = PH_SOS;
        esp32p4_clkwait_start(&s->tmo_frame, now, tmo_cycles(s));
    } else {
        s->phase = PH_ENC;
    }
    jpeg_run(s);
}

/* The tables. */

static bool qnr_fifo(ESP32P4JpegState *s)
{
    return s->config & CFG_QNR_FIFO_EN;
}

static bool dht_fifo(ESP32P4JpegState *s)
{
    return s->config & CFG_DHT_FIFO_EN;
}

/*
 * A table address: its entry, or NULL; *@fifo_mode whether the table
 * fills through its FIFO port.
 */
static void *jpeg_ram(ESP32P4JpegState *s, hwaddr reg, bool *wide,
                      bool *fifo_mode)
{
    int n;

    *wide = true;
    *fifo_mode = dht_fifo(s);
    if (reg >= A_QNR_RAM && reg < A_DHT_TOTLEN_RAM) {
        *fifo_mode = qnr_fifo(s);
        n = (reg - A_QNR_RAM) / 0x100;
        return &s->qnr[n][(reg & 0xff) / 4];
    }
    if (reg >= A_DHT_TOTLEN_RAM && reg < A_DHT_VAL_DC_RAM) {
        n = (reg - A_DHT_TOTLEN_RAM) / 0x80;
        return &s->dht_totlen[ESP32P4_JPEG_HT((reg >> 6) & 1, n)]
                             [(reg & 0x3f) / 4];
    }
    if (reg >= A_DHT_CODEMIN_RAM) {
        n = (reg - A_DHT_CODEMIN_RAM) / 0x80;
        return &s->dht_codemin[ESP32P4_JPEG_HT((reg >> 6) & 1, n)]
                              [(reg & 0x3f) / 4];
    }
    *wide = false;
    if (reg >= A_DHT_VAL_DC_RAM && reg < A_DHT_VAL_AC_RAM) {
        n = (reg - A_DHT_VAL_DC_RAM) / 0x40;
        return &s->dht_val_dc[n][(reg & 0x3f) / 4];
    }
    if (reg >= A_DHT_VAL_AC_RAM && reg < A_DHT_VAL_AC_END) {
        n = (reg - A_DHT_VAL_AC_RAM) / 0x400;
        return &s->dht_val_ac[n][((reg - A_DHT_VAL_AC_RAM) & 0x3ff) / 4];
    }
    return NULL;
}

/* A write to a FIFO port: JPEG_TnQNR_REG or a DHT register. */
/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_fifo_write(ESP32P4JpegState *s, hwaddr reg, uint32_t v)
{
    bool qnr = reg <= A_T3QNR;
    int t, cls, h;

    if (s->config & (CFG_SOFT_RST | CFG_FIFO_RST)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: table write at 0x%" HWADDR_PRIx
                      " while the FIFOs are held in reset is dropped\n",
                      TYPE_ESP32P4_JPEG, reg);
        return;
    }
    if (qnr ? !qnr_fifo(s) : !dht_fifo(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to the FIFO port at 0x%"
                      HWADDR_PRIx " with JPEG_%s_FIFO_EN clear is dropped\n",
                      TYPE_ESP32P4_JPEG, reg, qnr ? "QNR" : "DHT");
        return;
    }
    if (qnr) {
        t = (reg - A_T0QNR) / 4;
        s->qnr[t][s->qnr_ptr[t]] = v;
        s->qnr_ptr[t] = (s->qnr_ptr[t] + 1) % 64;
        return;
    }
    if (reg >= A_DHT_CODEMIN_DC0) {
        /* CODEMIN_DC0, _AC0, _DC1, _AC1 */
        t = (reg - A_DHT_CODEMIN_DC0) / 8;
        h = ESP32P4_JPEG_HT(((reg - A_DHT_CODEMIN_DC0) / 4) & 1, t);
        s->dht_codemin[h][s->codemin_ptr[h]] = v;
        s->codemin_ptr[h] = (s->codemin_ptr[h] + 1) % 16;
        return;
    }
    /* TOTLEN_DC0, VAL_DC0, TOTLEN_AC0, VAL_AC0, then the same for 1 */
    t = (reg - A_DHT_TOTLEN_DC0) / 16;
    cls = ((reg - A_DHT_TOTLEN_DC0) / 8) & 1;
    h = ESP32P4_JPEG_HT(cls, t);
    if (!(reg & 4)) {
        s->dht_totlen[h][s->totlen_ptr[h]] = v;
        s->totlen_ptr[h] = (s->totlen_ptr[h] + 1) % 16;
    } else if (cls) {
        s->dht_val_ac[t][s->val_ptr[h]] = v;
        s->val_ptr[h] = (s->val_ptr[h] + 1) % 256;
    } else {
        s->dht_val_dc[t][s->val_ptr[h]] = v;
        s->val_ptr[h] = (s->val_ptr[h] + 1) % 16;
    }
}

/* Registers. */

static uint64_t jpeg_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4JpegState *s = opaque;
    hwaddr reg = addr & ~3;
    bool wide, fifo_mode;
    uint32_t v = 0;
    void *ram;

    jpeg_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    switch (reg) {
    case A_CONFIG:
        v = s->config;
        break;
    case A_DQT_INFO:
        v = s->dqt_info;
        break;
    case A_PIC_SIZE:
        v = s->pic_size;
        break;
    case A_EXTD_CONFIG:
        v = s->extd_config;
        break;
    case A_DECODE_CONF:
        v = s->decode_conf;
        break;
    case A_C0 ... A_C3:
        v = s->comp[(reg - A_C0) / 4];
        break;
    case A_DHT_INFO:
        v = s->dht_info;
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_CLR:
        break;
    case A_STATUS2:
        v = STATUS2_RESET;
        break;
    case A_STATUS0:
    case A_STATUS3:
    case A_STATUS4:
    case A_DECODER_STATUS0 ... A_DECODER_STATUS5 - 4:
    case A_STATUS5:
        break;
    case A_DECODER_STATUS5:
        v = s->decoder_status5;
        break;
    case A_ECO_LOW:
        v = s->eco_low;
        break;
    case A_ECO_HIGH:
        v = s->eco_high;
        break;
    case A_SYS:
        v = s->sys;
        break;
    case A_VERSION:
        v = s->version;
        break;
    case A_T0QNR ... A_T3QNR:
    case A_DHT_TOTLEN_DC0 ... A_DHT_CODEMIN_AC1:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of the write-only FIFO port "
                      "at 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_JPEG, addr);
        break;
    default:
        ram = reg >= A_QNR_RAM ? jpeg_ram(s, reg, &wide, &fifo_mode) : NULL;
        if (!ram) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                          HWADDR_PRIx "\n", TYPE_ESP32P4_JPEG, addr);
            return 0;
        }
        v = wide ? *(uint16_t *)ram : *(uint8_t *)ram;
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static void jpeg_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4JpegState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);
    uint32_t old;
    bool wide, fifo_mode;
    void *ram;

#define MERGE(old) deposit32((old), shift, size * 8, value)
    jpeg_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    switch (reg) {
    case A_CONFIG:
        old = s->config;
        s->config = (MERGE(old) & CFG_RW_MASK) | CFG_DMA_LINKLIST;
        if ((s->config & CFG_SOFT_RST) && !(old & CFG_SOFT_RST)) {
            jpeg_abort(s);
            jpeg_fifo_ptrs_reset(s);
        }
        if ((s->config & CFG_FIFO_RST) && !(old & CFG_FIFO_RST)) {
            jpeg_fifo_ptrs_reset(s);
            jpeg_clear_data(s);
        }
        if (v & CFG_FSM_RST) {
            jpeg_abort(s);
        }
        if (v & CFG_JPEG_START) {
            if (jpeg_in_reset(s)) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: JPEG_JPEG_START while "
                              "JPEG_SOFT_RST holds the codec is ignored\n",
                              TYPE_ESP32P4_JPEG);
            } else {
                jpeg_start(s);
            }
        }
        if ((old ^ s->config) & (CFG_PAUSE_EN | CFG_SOFT_RST)) {
            jpeg_run(s);
        }
        break;
    case A_DQT_INFO:
        s->dqt_info = MERGE(s->dqt_info);
        break;
    case A_PIC_SIZE:
        s->pic_size = MERGE(s->pic_size);
        break;
    case A_EXTD_CONFIG:
        s->extd_config = MERGE(s->extd_config) & 3;
        break;
    case A_DECODE_CONF:
        s->decode_conf = (MERGE(s->decode_conf) & DECODE_CONF_RW) |
                         DECODE_CONF_SW_DHT;
        break;
    case A_C0 ... A_C3:
        s->comp[(reg - A_C0) / 4] = MERGE(s->comp[(reg - A_C0) / 4]) & COMP_RW;
        break;
    case A_DHT_INFO:
        s->dht_info = MERGE(s->dht_info) & 0xffff;
        break;
    case A_INT_ENA:
        s->int_ena = MERGE(s->int_ena) & INT_MASK;
        jpeg_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_MASK);
        jpeg_update_irq(s);
        break;
    case A_DECODER_STATUS5:
        s->decoder_status5 = MERGE(s->decoder_status5) & DECODER_STATUS5_RW;
        break;
    case A_ECO_LOW:
        s->eco_low = MERGE(s->eco_low);
        break;
    case A_ECO_HIGH:
        s->eco_high = MERGE(s->eco_high);
        break;
    case A_SYS:
        s->sys = MERGE(s->sys) & SYS_RW;
        break;
    case A_VERSION:
        s->version = MERGE(s->version) & VERSION_RW;
        break;
    case A_T0QNR ... A_T3QNR:
    case A_DHT_TOTLEN_DC0 ... A_DHT_CODEMIN_AC1:
        jpeg_fifo_write(s, reg, v);
        break;
    case A_INT_RAW:
    case A_INT_ST:
    case A_STATUS0:
    case A_STATUS2:
    case A_STATUS3:
    case A_STATUS4:
    case A_DECODER_STATUS0 ... A_DECODER_STATUS5 - 4:
    case A_STATUS5:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_JPEG, addr);
        break;
    default:
        ram = reg >= A_QNR_RAM ? jpeg_ram(s, reg, &wide, &fifo_mode) : NULL;
        if (!ram) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                          HWADDR_PRIx "\n", TYPE_ESP32P4_JPEG, addr);
            break;
        }
        if (fifo_mode) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to table address 0x%"
                          HWADDR_PRIx " in FIFO mode is dropped\n",
                          TYPE_ESP32P4_JPEG, addr);
            break;
        }
        if (wide) {
            *(uint16_t *)ram = MERGE(*(uint16_t *)ram);
        } else {
            *(uint8_t *)ram = MERGE(*(uint8_t *)ram);
        }
        break;
    }
#undef MERGE
}

static const MemoryRegionOps jpeg_ops = {
    .read = jpeg_read,
    .write = jpeg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* The 2D-DMA side. */

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_connect_dma(ESP32P4JpegState *s,
                              const ESP32P4JpegDmaOps *ops, void *opaque)
{
    s->dma_ops = ops;
    s->dma_opaque = opaque;
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
void esp32p4_jpeg_dma_ready(void *opaque)
{
    ESP32P4JpegState *s = opaque;

    jpeg_catch_up(s, esp32_line_time());
    jpeg_run(s);
}

void esp32p4_jpeg_dma_sync(void *opaque, int64_t when)
{
    jpeg_catch_up(opaque, when);
}

/*
 * The qtest's stand-in for the 2D-DMA, while none is connected: setting
 * "dma" to "tx <addr> <len> <eof>" queues a TX descriptor, the codec
 * reading its <len> bytes from memory at <addr>, the frame's end after
 * them where <eof> is 1; "rx <addr> <len>" queues an RX buffer in memory;
 * "reset" empties both queues. Reading it gives "<TX descriptors left>
 * <RX bytes written> <flags>", flags ORing ESP32P4_JPEG_DMA_EOF and _ERR_EOF
 * as the codec marked the last byte it gave.
 */
static char *jpeg_get_dma(Object *obj, Error **errp)
{
    ESP32P4JpegState *s = ESP32P4_JPEG(obj);

    return g_strdup_printf("%u %u %u", s->qt_tx_num, s->qt_rx_total,
                           s->qt_rx_flags);
}

static void jpeg_set_dma(Object *obj, const char *value, Error **errp)
{
    ESP32P4JpegState *s = ESP32P4_JPEG(obj);
    g_auto(GStrv) f = g_strsplit(value, " ", 0);
    unsigned int n = g_strv_length(f), addr, len, eof = 0;
    ESP32P4JpegQtSeg *g;

    if (s->dma_ops) {
        error_setg(errp, "dma: the codec's 2D-DMA paths are connected");
        return;
    }
    if (n == 1 && !strcmp(f[0], "reset")) {
        s->qt_tx_num = 0;
        s->qt_rx_num = 0;
        s->qt_rx_total = 0;
        s->qt_rx_flags = 0;
        return;
    }
    if (n < 3 || qemu_strtoui(f[1], NULL, 0, &addr) ||
        qemu_strtoui(f[2], NULL, 0, &len) ||
        (!strcmp(f[0], "tx") && (n < 4 || qemu_strtoui(f[3], NULL, 0, &eof)))
        || (strcmp(f[0], "tx") && strcmp(f[0], "rx"))) {
        error_setg(errp, "dma: want \"tx <addr> <len> <eof>\", \"rx <addr> "
                   "<len>\" or \"reset\"");
        return;
    }
    if (!strcmp(f[0], "tx")) {
        if (s->qt_tx_num == ESP32P4_JPEG_QT_SEGS) {
            error_setg(errp, "dma: TX queue full");
            return;
        }
        g = &s->qt_tx[s->qt_tx_num++];
    } else {
        if (s->qt_rx_num == ESP32P4_JPEG_QT_SEGS) {
            error_setg(errp, "dma: RX queue full");
            return;
        }
        g = &s->qt_rx[s->qt_rx_num++];
    }
    *g = (ESP32P4JpegQtSeg) { .addr = addr, .len = len, .eof = eof };
    esp32p4_jpeg_dma_ready(s);
}

/* Clock and reset. */

static void jpeg_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4JpegState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        jpeg_catch_up(s, now);
        esp32p4_clkwait_fold(&s->work, now);
        esp32p4_clkwait_fold(&s->tmo_stall, now);
        esp32p4_clkwait_fold(&s->tmo_frame, now);
        return;
    }
    esp32p4_clkwait_resched(&s->work);
    esp32p4_clkwait_resched(&s->tmo_stall);
    esp32p4_clkwait_resched(&s->tmo_frame);
    jpeg_run(s);
}

static void jpeg_reset_hold(Object *obj, ResetType type)
{
    ESP32P4JpegState *s = ESP32P4_JPEG(obj);

    jpeg_abort(s);
    s->config = CFG_RESET;
    s->dqt_info = DQT_INFO_RESET;
    s->pic_size = PIC_SIZE_RESET;
    s->extd_config = 0;
    s->decode_conf = DECODE_CONF_RESET;
    for (int i = 0; i < 4; i++) {
        s->comp[i] = COMP_RESET;
    }
    s->dht_info = DHT_INFO_RESET;
    s->int_raw = 0;
    s->int_ena = 0;
    s->decoder_status5 = 0;
    s->eco_low = 0;
    s->eco_high = UINT32_MAX;
    s->sys = 0;
    s->version = VERSION_RESET;
    memset(s->qnr, 0, sizeof(s->qnr));
    memset(s->dht_totlen, 0, sizeof(s->dht_totlen));
    memset(s->dht_codemin, 0, sizeof(s->dht_codemin));
    memset(s->dht_val_dc, 0, sizeof(s->dht_val_dc));
    memset(s->dht_val_ac, 0, sizeof(s->dht_val_ac));
    jpeg_fifo_ptrs_reset(s);
    s->format = FMT_YUV444;
    s->src = SRC_RGB888;
    s->ncomp = 0;
    s->mcus = 0;
    s->mcu = 0;
    s->restart = 0;
    s->frame_err = false;
}

static void jpeg_reset_exit(Object *obj, ResetType type)
{
    jpeg_update_irq(ESP32P4_JPEG(obj));
}

static void jpeg_init(Object *obj)
{
    ESP32P4JpegState *s = ESP32P4_JPEG(obj);

    memory_region_init_io(&s->iomem, obj, &jpeg_ops, s, "esp32p4.jpeg",
                          ESP32P4_JPEG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", jpeg_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    esp32p4_clkwait_init(&s->work, s->clk, jpeg_work_done, s);
    esp32p4_clkwait_init(&s->tmo_stall, s->clk, jpeg_tmo_stall_cb, s);
    esp32p4_clkwait_init(&s->tmo_frame, s->clk, jpeg_tmo_frame_cb, s);
    object_property_add_str(obj, "dma", jpeg_get_dma, jpeg_set_dma);
}

static void jpeg_finalize(Object *obj)
{
    ESP32P4JpegState *s = ESP32P4_JPEG(obj);

    timer_del(&s->work.timer);
    timer_del(&s->tmo_stall.timer);
    timer_del(&s->tmo_frame.timer);
}

static int jpeg_post_load(void *opaque, int version_id)
{
    ESP32P4JpegState *s = opaque;

    jpeg_build_hdec(s);
    if (s->ncomp > 3 || s->in_len > ESP32P4_JPEG_IN_SIZE ||
        s->in_pos > s->in_len ||
        s->out_head >= ESP32P4_JPEG_OUT_SIZE ||
        s->out_num > ESP32P4_JPEG_OUT_SIZE ||
        s->stage_len > ESP32P4_JPEG_STAGE_SIZE ||
        s->format > FMT_GRAY || s->src > SRC_YUV420) {
        return -EINVAL;
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_jpeg = {
    .name = TYPE_ESP32P4_JPEG,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = jpeg_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CLKWAIT(work, ESP32P4JpegState),
        VMSTATE_ESP32P4_CLKWAIT(tmo_stall, ESP32P4JpegState),
        VMSTATE_ESP32P4_CLKWAIT(tmo_frame, ESP32P4JpegState),
        VMSTATE_UINT32(config, ESP32P4JpegState),
        VMSTATE_UINT32(dqt_info, ESP32P4JpegState),
        VMSTATE_UINT32(pic_size, ESP32P4JpegState),
        VMSTATE_UINT32(extd_config, ESP32P4JpegState),
        VMSTATE_UINT32(decode_conf, ESP32P4JpegState),
        VMSTATE_UINT32_ARRAY(comp, ESP32P4JpegState, 4),
        VMSTATE_UINT32(dht_info, ESP32P4JpegState),
        VMSTATE_UINT32(int_raw, ESP32P4JpegState),
        VMSTATE_UINT32(int_ena, ESP32P4JpegState),
        VMSTATE_UINT32(decoder_status5, ESP32P4JpegState),
        VMSTATE_UINT32(eco_low, ESP32P4JpegState),
        VMSTATE_UINT32(eco_high, ESP32P4JpegState),
        VMSTATE_UINT32(sys, ESP32P4JpegState),
        VMSTATE_UINT32(version, ESP32P4JpegState),
        VMSTATE_UINT16_2DARRAY(qnr, ESP32P4JpegState, 4, 64),
        VMSTATE_UINT16_2DARRAY(dht_totlen, ESP32P4JpegState, 4, 16),
        VMSTATE_UINT16_2DARRAY(dht_codemin, ESP32P4JpegState, 4, 16),
        VMSTATE_UINT8_2DARRAY(dht_val_dc, ESP32P4JpegState, 2, 16),
        VMSTATE_UINT8_2DARRAY(dht_val_ac, ESP32P4JpegState, 2, 256),
        VMSTATE_UINT8_ARRAY(qnr_ptr, ESP32P4JpegState, 4),
        VMSTATE_UINT8_ARRAY(totlen_ptr, ESP32P4JpegState, 4),
        VMSTATE_UINT8_ARRAY(codemin_ptr, ESP32P4JpegState, 4),
        VMSTATE_UINT16_ARRAY(val_ptr, ESP32P4JpegState, 4),
        VMSTATE_UINT32(phase, ESP32P4JpegState),
        VMSTATE_UINT32(format, ESP32P4JpegState),
        VMSTATE_UINT32(src, ESP32P4JpegState),
        VMSTATE_BOOL(pixel_rev, ESP32P4JpegState),
        VMSTATE_BOOL(ff_check, ESP32P4JpegState),
        VMSTATE_BOOL(tailer, ESP32P4JpegState),
        VMSTATE_BOOL(direct, ESP32P4JpegState),
        VMSTATE_UINT32(ncomp, ESP32P4JpegState),
        VMSTATE_UINT32(mcus, ESP32P4JpegState),
        VMSTATE_UINT32(mcu, ESP32P4JpegState),
        VMSTATE_UINT32(restart, ESP32P4JpegState),
        VMSTATE_BOOL(in_eof, ESP32P4JpegState),
        VMSTATE_BOOL(frame_err, ESP32P4JpegState),
        VMSTATE_INT32_ARRAY(pred, ESP32P4JpegState, 4),
        VMSTATE_UINT8_ARRAY(dc_sel, ESP32P4JpegState, 4),
        VMSTATE_UINT8_ARRAY(ac_sel, ESP32P4JpegState, 4),
        VMSTATE_UINT8_ARRAY(q_sel, ESP32P4JpegState, 4),
        VMSTATE_UINT32_ARRAY(qmask, ESP32P4JpegState, 4),
        VMSTATE_UINT16_2DARRAY(enc_q, ESP32P4JpegState, 2, 64),
        VMSTATE_UINT64(bit_acc, ESP32P4JpegState),
        VMSTATE_INT32(bit_count, ESP32P4JpegState),
        VMSTATE_UINT8_ARRAY(in_buf, ESP32P4JpegState, ESP32P4_JPEG_IN_SIZE),
        VMSTATE_UINT32(in_len, ESP32P4JpegState),
        VMSTATE_UINT32(in_pos, ESP32P4JpegState),
        VMSTATE_UINT8_ARRAY(stage, ESP32P4JpegState, ESP32P4_JPEG_STAGE_SIZE),
        VMSTATE_UINT32(stage_len, ESP32P4JpegState),
        VMSTATE_UINT8_ARRAY(out, ESP32P4JpegState, ESP32P4_JPEG_OUT_SIZE),
        VMSTATE_UINT32(out_head, ESP32P4JpegState),
        VMSTATE_UINT32(out_num, ESP32P4JpegState),
        VMSTATE_BOOL(out_final, ESP32P4JpegState),
        VMSTATE_UINT32(out_flags, ESP32P4JpegState),
        VMSTATE_END_OF_LIST()
    },
};

static void jpeg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_esp32p4_jpeg;
    rc->phases.hold = jpeg_reset_hold;
    rc->phases.exit = jpeg_reset_exit;
    for (int t = 0; t < 2; t++) {
        esp32p4_jpeg_huff_code_build(&enc_dc[t], &esp32p4_jpeg_std_dc[t]);
        esp32p4_jpeg_huff_code_build(&enc_ac[t], &esp32p4_jpeg_std_ac[t]);
    }
}

/* [spec:nuos:req:emu.esp32p4.jpeg] */
static const TypeInfo jpeg_info = {
    .name          = TYPE_ESP32P4_JPEG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4JpegState),
    .instance_init = jpeg_init,
    .instance_finalize = jpeg_finalize,
    .class_init    = jpeg_class_init,
};

static void jpeg_register_types(void)
{
    type_register_static(&jpeg_info);
}
type_init(jpeg_register_types)
