/*
 * ESP32-P4 H264 encoder and its dedicated DMA (H264_DMA)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 41 (H264 Encoder), with the layout, access types and reset values of
 * ESP-IDF's h264_reg.h and h264_dma_reg.h for v3, and Espressif's esp_h264
 * component (its h264_ll, h264_dma_ll and hardware encoder drivers), which
 * is how software drives the block: ESP-IDF itself has no driver for it.
 *
 * The encoder (ENC_CORE, 41.5.1) codes the pictures, hw/misc/
 * esp32p4_h264_enc.c being its algorithm; this file is its registers, its
 * interrupts, its timing and H264_DMA (41.5.2), through which every byte
 * it reads and writes moves. Software writes the SPS, PPS and the slice
 * header; the encoder starts each picture's bitstream with the slice
 * header's unaligned tail from SLICE_HEADER_BYTE1:BYTE0 (the first
 * SLICE_BYTE_LENGTH bytes, most significant first) and SLICE_REMAIN_BIT
 * (its top SLICE_REMAIN_BITLENGTH bits), then codes the slice data and
 * its trailing bits, with emulation prevention over all it writes. Its
 * slice QP is H264_x_QP, which the slice header must agree with, as its
 * deblocking must with BYPASS_DB_FILTER and its chroma QP offset with
 * CHROMA_QP_DELTA.
 *
 * A picture is coded a macroblock row at a time. At the start of row r
 * the DMA brings in what the row needs: its source lines on TX channel 0
 * (a 2D descriptor over the picture, HA x VA pixels of ORI_PBYTE bytes,
 * read as 16-line rows), for row r > 0 the four bottom lines of row r - 1
 * the deblocking filter left in internal memory (TX channel 2, 128 bytes
 * a macroblock), and in a P picture the reference rows r - 1 to r + 1.
 * The reference is the previous picture's deblocked output, which RX
 * channels 0 and 1 wrote to memory as it was coded: each row's top twelve
 * lines and, a row later, its bottom four (the last row whole, through RX
 * channel 0), in 256-byte and 128-byte blocks a macroblock. RX channel 5
 * moves it back, a row at a time, through TX channels 3 and 4, into a
 * window of three rows in internal memory at BLOCK_START_ADDR_CH5, each
 * macroblock 384 bytes (its 12-line block, then its 4-line block); TX
 * channel 1 fetches the window, a pass over it a row. DMA_MOVE_START
 * moves the first two rows and raises DMA_MOVE_2MB_LINE_DONE; after that
 * a row moves as the window frees one, ahead of the coding, and in GOP
 * mode the moving runs on from one picture's reference into the next's.
 * The window holds rows r - 1 to r + 1, which is why the motion search's
 * vertical range is 13.75 lines. At the end of row r the DMA takes out
 * its results: the row's top twelve lines (RX 0), row r - 1's bottom four
 * now that row r's top edge has filtered them (RX 1), row r's bottom four
 * for the next row (RX 2), the merged motion vectors (RX 3) and the
 * bitstream (RX 4). DB_TMP_READY rises when DB_TMP_READY_TRIGGER_MB_NUM
 * macroblocks of the first row's bottom lines are written, REC_READY
 * after REC_READY_TRIGGER_MB_LINES + 1 rows, FRAME_DONE after the last.
 *
 * Every byte goes through memory, so a reference software corrupts, or a
 * channel it forgets to start, is what the encoder sees: a missing input
 * (a TX channel not started or out of descriptors, the RX channels'
 * counters short of the data a read needs, a window row not yet moved)
 * stalls the encoder until it comes, and an RX channel not started holds
 * the results back; an RX channel whose descriptors run out raises
 * IN_DSCR_EMPTY and the rest of the picture's data for it is lost, the
 * bitstream's with BS_BUFFER_OVERFLOW_INT, the picture still completing
 * (FRAME_CODE_LENGTH counts the bytes coded, written or not), as
 * Espressif's driver relies on for v3.
 *
 * Timing (interpretation; the TRM gives only the rate, 1920 x 1088 at 30
 * frames a second): a macroblock takes 980 cycles of H264_CLK, which at
 * the 240 MHz of PLL_F240M is that rate; a row's results come out when its
 * last macroblock is done, and DB_TMP_READY when the trigger's macroblock
 * is. Moving a reference row takes 48 cycles a macroblock, 8 bytes a
 * cycle, in parallel with the coding; only DMA_MOVE_START's two rows
 * take time of their own before DMA_MOVE_2MB_LINE_DONE. Without H264_CLK
 * nothing advances; the registers need H264_SYS_CLK.
 *
 * GOP mode and dual-stream mode (41.7): the encoder counts the pictures
 * started since SYS_RST_PULSE, alternating between video sequences A and
 * B in dual-stream mode, and codes an I picture at the start of each GOP
 * of GOP_NUM pictures (one I picture then P pictures for GOP_NUM 0),
 * reporting the count, the sequence and the type in SYS_STATUS (the
 * count modulo GOP_NUM times the sequences, which its nine bits hold).
 * Each picture takes the register set of its sequence as it starts.
 *
 * Interpretations where the TRM is silent, besides those of the core:
 *
 * - Picture size: the register fields reach 127 macroblocks, but Espressif
 *   caps the width at 1920 pixels (120 macroblocks), the line buffers'
 *   size; a wider picture, or one of no macroblocks, does not start.
 * - The source's macroblocks beyond HA or VA repeat its last column and
 *   line; HB other than 16 needs OUT_REORDER_EN_CH0, without which the
 *   encoder takes the DMA's blocks in their order as 16-pixel
 *   macroblocks.
 * - TX channel 1 reads the window at 1.5 bytes a pixel (4:2:0), whatever
 *   ORI_PBYTE says, which applies to TX channel 0.
 * - The internal and external memory ranges check descriptor addresses
 *   (always the internal ranges: descriptors are read through AXI_M1) and
 *   buffers (TX channels 1, 2 and RX channels 2, 3, 5 internal; the rest
 *   external); a descriptor outside, misaligned (8 bytes), or not owned
 *   with CHECK_OWNER set, raises DSCR_ERR and halts its channel.
 * - The counters RX_CH0/1/2_CNT hold the 8-byte words RX channels 0, 1
 *   and 2 have written that TX channels 3, 4 and 2 have not yet read;
 *   RX_CH5_CNT those of the window holding a row. A read waits for its
 *   counter. COUNTER_RST's bits hold theirs at 0.
 * - The H264_DMA masters' IDs on the AXI interconnect are ports 11
 *   (AXI_M1, internal) and 12 (AXI_M2, external), with the TX channel's
 *   number, or 8 plus the RX channel's, as unit.
 * - SYS_RST_PULSE stops a picture in progress and resets the picture
 *   count and the reference moving; registers keep their values.
 * - TX channel 0's test mode feeds the encoder OUT_TEST_Y/U/V_VALUE as
 *   every pixel, the descriptors still read.
 * - CHROMA_DC_QP_DELTA other than 0 is not modelled (logged): chroma DC
 *   levels are quantized at the chroma QP, as decoders reconstruct them.
 *
 * Not modelled: the FIFOs and their status, push and pop registers
 * (software access to them is logged); the AXI arbitration, burst lengths
 * and page bounds (kept as written); the register clock gates and reorder
 * RAM power controls; ETM tasks, which no ETM event or task of the chip
 * reaches (a channel with ETM_EN set waits for tasks, so never starts);
 * the debug FSM states, which read as idle.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_h264.h"
#include "hw/misc/esp32p4_pms.h"
#include "migration/vmstate.h"

/* Timing (see above). */
#define CYCLES_PER_MB       980
#define MOVE_CYCLES_PER_MB  48
#define MAX_MB_X            120

/* The encoder's registers. */
#define A_SYS_CTRL          0x000
#define A_GOP_CONF          0x004
#define A_SEQ_A             0x008
#define A_SEQ_B             0x054
#define A_RC_STATUS0        0x0a0
#define A_RC_STATUS1        0x0a4
#define A_RC_STATUS2        0x0a8
#define A_SLICE_REMAIN      0x0ac
#define A_SLICE_LEN         0x0b0
#define A_BS_THRESHOLD      0x0b4
#define A_SLICE_BYTE0       0x0b8
#define A_SLICE_BYTE1       0x0bc
#define A_INT_RAW           0x0c0
#define A_INT_ST            0x0c4
#define A_INT_ENA           0x0c8
#define A_INT_CLR           0x0cc
#define A_CONF              0x0d0
#define A_MV_MERGE          0x0d4
#define A_DEBUG_DMA_SEL     0x0d8
#define A_SYS_STATUS        0x0dc
#define A_FRAME_CODE_LENGTH 0x0e0
#define A_DEBUG_INFO0       0x0e4
#define A_DEBUG_INFO2       0x0ec
#define A_DATE              0x0f0
#define A_ORI_CONF_A        0x0f4
#define A_ORI_CONF_B        0x0f8
#define A_ORI_DEBUG         0x0fc
#define A_REF_DEBUG         0x114

#define SYS_FRAME_START     BIT(0)
#define SYS_DMA_MOVE_START  BIT(1)
#define SYS_FRAME_MODE      BIT(2)
#define SYS_RST_PULSE       BIT(3)

#define GOP_DUAL            BIT(0)
#define GOP_NUM(v)          extract32(v, 1, 8)

#define INT_DB_TMP_READY    BIT(0)
#define INT_REC_READY       BIT(1)
#define INT_FRAME_DONE      BIT(2)
#define INT_MOVE_DONE       BIT(3)
#define INT_BS_OVERFLOW     BIT(4)
#define INT_MASK            0x1f

/* A video sequence's registers, in the order of their addresses. */
enum {
    S_MB_RES, S_SYS_CONF, S_DECI_SCORE, S_DECI_OFFSET, S_RC_CONF0,
    S_RC_CONF1, S_DB_BYPASS, S_ROI0, S_ROI03_QP = S_ROI0 + 8, S_ROI47_QP,
    S_NO_ROI_QP, S_ROI_CONFIG,
};

static const uint32_t seq_mask[ESP32P4_H264_SEQ_REGS] = {
    0x3fff, 0x3fffffff, 0xfffff, 0xffffff, 0x7fffff, 0x7fffffff, 0x1,
    0x1fffffff, 0x1fffffff, 0x1fffffff, 0x1fffffff, 0x1fffffff, 0x1fffffff,
    0x1fffffff, 0x1fffffff, 0x0fffffff, 0x0fffffff, 0x7f, 0x3,
};
/* DB_TMP_READY_TRIGGER_MB_NUM 3, REC_READY_TRIGGER_MB_LINES 4. */
#define SYS_CONF_RESET      (3 | 4 << 7)

#define MV_MERGE_RW         0x1f
#define MV_MERGE_A_EN       BIT(3)
#define MV_MERGE_B_EN       BIT(4)

#define ENC_DATE_RESET      0x2412300
#define ORI_CONF_RESET      H264E_CS_YUV420
#define BS_THRESHOLD_RESET  0x30

static const uint32_t debug_conf_mask[7] = {
    0x1ffffff, 0x1ff, 0x1ff, 0x1ffffff, 0x1ffffff, 0x1ffffff, 0x1ffffff,
};
enum { D_ORI, D_MVM, D_BS, D_DB_WR_TEMP, D_DB_RD_TEMP, D_DB_WR, D_REF };

/* H264_DMA. */
#define DMA_RX_BASE         0x500
#define DMA_RX5_BASE        0xa00
#define DMA_GLOBAL_BASE     0xb00

#define TX_AUTO_WRBACK      BIT(0)
#define TX_EOF_MODE         BIT(1)
#define CH_CHECK_OWNER      BIT(4)
#define TX_REORDER_EN       BIT(16)
#define CH_RST              BIT(24)
#define CH_CMD_DISABLE      BIT(25)

#define TX_LINK_STOP        BIT(20)
#define TX_LINK_START       BIT(21)
#define TX_LINK_RESTART     BIT(22)
#define TX_LINK_PARK        BIT(23)
#define RX_LINK_AUTO_RET    BIT(20)
#define RX_LINK_STOP        BIT(21)
#define RX_LINK_START       BIT(22)
#define RX_LINK_RESTART     BIT(23)
#define RX_LINK_PARK        BIT(24)

#define TX_INT_DONE         BIT(0)
#define TX_INT_EOF          BIT(1)
#define TX_INT_DSCR_ERR     BIT(2)
#define TX_INT_TOTAL_EOF    BIT(3)
#define TX_INT_MASK         0x1ff
#define RX_INT_DONE         BIT(0)
#define RX_INT_SUC_EOF      BIT(1)
#define RX_INT_DSCR_ERR     BIT(3)
#define RX_INT_DSCR_EMPTY   BIT(8)
#define RX_INT_MASK         0x3ff
#define RX5_INT_MASK        0x1f

#define ETM_EN              BIT(0)

/* Descriptor words. */
#define DW0_SIZE_LO(d)      ((d) & 0x3fff)
#define DW0_LEN_LO(d)       (((d) >> 14) & 0x3fff)
#define DW0_2D              BIT(29)
#define DW0_EOF             BIT(30)
#define DW0_OWNER           BIT(31)
#define DW1_SIZE_HI(d)      ((d) & 0x3fff)
#define DW1_LEN_HI(d)       (((d) >> 14) & 0x7fff)
#define DW1_MODE            BIT(29)

#define CNT_RX0             0
#define CNT_RX1             1
#define CNT_RX2             2
#define CNT_RX5             3

static const uint32_t cnt_mask[4] = { 0x7fffff, 0x1fffff, 0x7ff, 0x1ffff };

#define TX_FIFO_STATUS_RESET    0x20082
#define RX5_FIFO_STATUS_RESET   0x2
#define POP_RDATA_EMPTY         0x400
#define MEM_START_RESET         0x30100000
#define MEM_END_RESET           0x8fffffff
#define DMA_DATE_RESET          0x20230403
#define PBYTE_RESET             2
#define BLOCK_ROW_LEN_RESET     (0x7800 | 0x3c00u << 16)
#define BLOCK_LEN_RESET         (0x100 | 0x80 << 14)

/* The picture's progress. */
enum {
    ST_IDLE,
    ST_READ,        /* waiting to read row's inputs */
    ST_CODE,        /* coding the row */
    ST_WRITE,       /* waiting to write the row's results */
};

/* A channel's link. */
enum {
    LINK_IDLE,      /* not started: its data waits */
    LINK_RUN,
    LINK_END,       /* its list ended or failed */
};

/* The AXI interconnect's ports for H264_DMA's two masters. */
#define ICM_CID_INTER       11
#define ICM_CID_EXTER       12

static void h264_kick(ESP32P4H264State *s, int64_t now);

static int64_t h264_now(void)
{
    return esp32_line_time();
}

/* Interrupts */

static void h264_update_irq(ESP32P4H264State *s)
{
    for (int n = 0; n < ESP32P4_H264_TX; n++) {
        qemu_set_irq(s->irq[ESP32P4_H264_IRQ_TX(n)],
                     !!(s->tx[n].int_raw & s->tx[n].int_ena));
    }
    for (int n = 0; n < ESP32P4_H264_RX; n++) {
        qemu_set_irq(s->irq[ESP32P4_H264_IRQ_RX(n)],
                     !!(s->rx[n].int_raw & s->rx[n].int_ena));
    }
    qemu_set_irq(s->irq[ESP32P4_H264_IRQ_REG], !!(s->int_raw & s->int_ena));
}

static void h264_raise(ESP32P4H264State *s, uint32_t bits)
{
    s->int_raw |= bits;
    h264_update_irq(s);
}

/* Channels */

static ESP32P4H264Chan *chan(ESP32P4H264State *s, bool rx, int n)
{
    return rx ? &s->rx[n] : &s->tx[n];
}

static const char *chan_name(bool rx)
{
    return rx ? "RX" : "TX";
}

/* Whether channel @n reaches internal memory (AXI_M1) rather than external. */
static bool chan_internal(bool rx, int n)
{
    return rx ? (n == 2 || n == 3 || n == 5) : (n == 1 || n == 2);
}

/* The CONF0 holding @n's reset and CMD_DISABLE: TX 3, 4 share RX 5's. */
static uint32_t chan_ctrl(ESP32P4H264State *s, bool rx, int n)
{
    return !rx && n >= 3 ? s->rx[5].conf0 : chan(s, rx, n)->conf0;
}

static bool chan_ready(ESP32P4H264State *s, bool rx, int n)
{
    uint32_t c = chan_ctrl(s, rx, n);

    return chan(s, rx, n)->running == LINK_RUN &&
           !(c & (CH_RST | CH_CMD_DISABLE));
}

static MemTxAttrs chan_attrs(bool rx, int n)
{
    return esp32p4_dma_attrs(ESP32P4_DMA_MASTER_H264,
                             ESP32P4_ICM_ID_AXI(chan_internal(rx, n)
                                                ? ICM_CID_INTER
                                                : ICM_CID_EXTER,
                                                rx ? 8 + n : n));
}

/* Whether [@addr, @addr + @len) is in the internal or external ranges. */
static bool h264_in_range(ESP32P4H264State *s, bool internal, uint32_t addr,
                          uint32_t len)
{
    const uint32_t *r = s->mem_addr + (internal ? 0 : 4);
    uint64_t last = (uint64_t)addr + MAX(len, 1) - 1;

    for (int i = 0; i < 2; i++) {
        if (addr >= r[2 * i] && last <= r[2 * i + 1]) {
            return true;
        }
    }
    return false;
}

/* A bus access of channel @n; a refused one reads zeros. */
static bool h264_bus(ESP32P4H264State *s, bool rx, int n, bool internal,
                     uint32_t addr, void *buf, uint32_t len, bool write)
{
    MemTxResult r = address_space_rw(s->as, addr, chan_attrs(rx, n), buf, len,
                                     write);

    if (r == MEMTX_OK) {
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: bus error on a %s of "
                  "%u bytes at 0x%08" PRIx32 "\n", TYPE_ESP32P4_H264,
                  chan_name(rx), n, write ? "write" : "read", len, addr);
    if (!write) {
        memset(buf, 0, len);
    }
    /* *_AXI_ERR's response error counts, 4 bits each. */
    s->axi_err[internal ? 0 : 1] =
        deposit32(s->axi_err[internal ? 0 : 1], write ? 8 : 4, 4,
                  extract32(s->axi_err[internal ? 0 : 1], write ? 8 : 4, 4)
                  + 1);
    return false;
}

/* The debug data replacing channel @n's (H264_DMA_CH_DBG_EN). */
static void chan_dbg(ESP32P4H264State *s, bool rx, int n, uint8_t *buf,
                     uint32_t len, int unit)
{
    int bit = rx ? 16 + n : n;
    int idx = rx ? ESP32P4_H264_TX + n : n;
    uint64_t v;

    if (!(s->ch_dbg_en & BIT(bit))) {
        return;
    }
    v = s->dbg_data[2 * idx] | (uint64_t)s->dbg_data[2 * idx + 1] << 32;
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = v >> (8 * (i % unit));
    }
}

static uint32_t desc_len(const uint32_t *dw)
{
    return DW0_LEN_LO(dw[0]) | DW1_LEN_HI(dw[1]) << 14;
}

static uint32_t desc_size(const uint32_t *dw)
{
    return DW0_SIZE_LO(dw[0]) | DW1_SIZE_HI(dw[1]) << 14;
}

/* TX channel 0's bytes per pixel, doubled, by ORI_PBYTE; 0 if invalid. */
static int pbyte2(ESP32P4H264State *s)
{
    static const int b2[5] = { 1, 2, 3, 4, 6 };

    return s->pbyte < 5 ? b2[s->pbyte] : 0;
}

/* The bytes a 2D descriptor's picture spans. */
static uint64_t desc_2d_extent(ESP32P4H264State *s, int n, const uint32_t *dw)
{
    uint64_t lb = (uint64_t)DW1_LEN_HI(dw[1]) * (n == 0 ? pbyte2(s) : 3) / 2;

    return lb * DW1_SIZE_HI(dw[1]);
}

/*
 * Read the descriptor at @addr for channel @n into @dw, applying the two
 * descriptor checks; false, with the reason in *@why, if it fails them.
 */
static bool desc_read(ESP32P4H264State *s, bool rx, int n, uint32_t addr,
                      uint32_t *dw, const char **why)
{
    uint8_t raw[16];
    uint64_t extent;

    if (addr & 7) {
        *why = "is not 8-byte aligned";
        return false;
    }
    if (!h264_in_range(s, true, addr, 16)) {
        *why = "is outside the internal memory range";
        return false;
    }
    if (!h264_bus(s, rx, n, true, addr, raw, 16, false)) {
        *why = "could not be read";
        return false;
    }
    for (int i = 0; i < 4; i++) {
        dw[i] = ldl_le_p(raw + 4 * i);
    }
    if ((chan_ctrl(s, rx, n) & CH_CHECK_OWNER) && !(dw[0] & DW0_OWNER)) {
        *why = "is not owned by the DMA";
        return false;
    }
    if (dw[0] & DW0_2D) {
        extent = desc_2d_extent(s, n, dw);
    } else {
        extent = rx ? desc_size(dw) : desc_len(dw);
    }
    if (extent && !h264_in_range(s, chan_internal(rx, n), dw[2],
                                 MIN(extent, UINT32_MAX))) {
        *why = "points to a buffer outside the channel's memory range";
        return false;
    }
    return true;
}

/* Fetch channel @n's next descriptor; on a failed check, halt it. */
static bool chan_fetch(ESP32P4H264State *s, bool rx, int n)
{
    ESP32P4H264Chan *c = chan(s, rx, n);
    const char *why;

    if (!c->desc_addr) {
        c->running = LINK_END;
        return false;
    }
    if (!desc_read(s, rx, n, c->desc_addr, c->dw, &why)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: the descriptor at "
                      "0x%08" PRIx32 " %s\n", TYPE_ESP32P4_H264,
                      chan_name(rx), n, c->desc_addr, why);
        c->int_raw |= rx ? RX_INT_DSCR_ERR : TX_INT_DSCR_ERR;
        c->running = LINK_END;
        h264_update_irq(s);
        return false;
    }
    if (rx && (c->dw[0] & DW0_2D)) {
        qemu_log_mask(LOG_UNIMP, "%s: RX channel %d: 2D descriptors are "
                      "taken as 1D\n", TYPE_ESP32P4_H264, n);
    }
    c->dscr_bf1 = c->dscr_bf0;
    c->dscr_bf0 = c->dscr;
    c->dscr = c->dw[3];
    c->have_desc = true;
    c->pos = 0;
    return true;
}

/* A TX descriptor's data has all gone out. */
static void tx_desc_done(ESP32P4H264State *s, int n)
{
    ESP32P4H264Chan *c = &s->tx[n];

    c->int_raw |= TX_INT_DONE;
    if (c->dw[0] & DW0_EOF) {
        c->int_raw |= TX_INT_EOF | TX_INT_TOTAL_EOF;
        c->eof_addr = c->desc_addr;
    }
    if (chan_ctrl(s, false, n) & TX_AUTO_WRBACK) {
        uint8_t w[4];

        stl_le_p(w, c->dw[0] & ~DW0_OWNER);
        h264_bus(s, false, n, true, c->desc_addr, w, 4, true);
    }
    c->desc_addr = c->dw[3];
    c->have_desc = false;
    if (!c->desc_addr) {
        c->running = LINK_END;
    }
    h264_update_irq(s);
}

/*
 * Whether 1D TX channel @n has @need bytes to give: true too if a
 * descriptor on the way fails its checks, for tx_pull() to find.
 */
static bool tx_avail(ESP32P4H264State *s, int n, uint32_t need)
{
    ESP32P4H264Chan *c = &s->tx[n];
    uint32_t have = 0, addr = c->desc_addr, pos = c->pos, dw[4];
    bool hd = c->have_desc;
    const char *why;

    if (!chan_ready(s, false, n)) {
        return false;
    }
    if (hd) {
        memcpy(dw, c->dw, sizeof(dw));
    }
    for (int i = 0; i < 64 && have < need; i++) {
        if (!hd) {
            if (!addr) {
                return false;
            }
            if (!desc_read(s, false, n, addr, dw, &why)) {
                return true;
            }
            pos = 0;
        }
        have += desc_len(dw) - MIN(pos, desc_len(dw));
        addr = dw[3];
        hd = false;
    }
    return have >= need;
}

/* Take @need bytes from 1D TX channel @n, as tx_avail() found them. */
static bool tx_pull(ESP32P4H264State *s, int n, uint8_t *buf, uint32_t need)
{
    ESP32P4H264Chan *c = &s->tx[n];

    for (int guard = 0; need && guard < 256; guard++) {
        uint32_t len, k;

        if (!c->have_desc && !chan_fetch(s, false, n)) {
            return false;
        }
        len = desc_len(c->dw);
        k = MIN(need, len - MIN(c->pos, len));
        if (k) {
            h264_bus(s, false, n, chan_internal(false, n), c->dw[2] + c->pos,
                     buf, k, false);
            chan_dbg(s, false, n, buf, k, 8);
            buf += k;
            need -= k;
            c->pos += k;
        }
        if (c->pos >= len) {
            tx_desc_done(s, n);
        }
    }
    return !need;
}

/* Close RX channel @n's descriptor with what it holds. */
static void rx_desc_close(ESP32P4H264State *s, int n, bool eof)
{
    ESP32P4H264Chan *c = &s->rx[n];
    uint32_t len = c->pos;
    uint8_t w[8];

    c->dw[0] = (c->dw[0] & ~((0x3fffu << 14) | DW0_EOF | DW0_OWNER)) |
               (len & 0x3fff) << 14 | (eof ? DW0_EOF : 0);
    c->dw[1] = (c->dw[1] & ~(0x7fffu << 14)) | ((len >> 14) & 0x7fff) << 14;
    stl_le_p(w, c->dw[0]);
    stl_le_p(w + 4, c->dw[1]);
    h264_bus(s, true, n, true, c->desc_addr, w, 8, true);
    c->int_raw |= RX_INT_DONE | (eof ? RX_INT_SUC_EOF : 0);
    if (eof) {
        c->eof_addr = c->desc_addr;
    }
    c->desc_addr = c->dw[3];
    c->have_desc = false;
    if (!c->desc_addr) {
        c->running = LINK_END;
    }
    h264_update_irq(s);
}

static void cnt_add(ESP32P4H264State *s, int i, int64_t words)
{
    int64_t v = (int64_t)s->cnt[i] + words;

    if (s->counter_rst & BIT(i)) {
        v = 0;
    }
    s->cnt[i] = v < 0 ? 0 : v;
}

/* What became of data given to an RX channel. */
enum {
    PUSH_HELD,      /* the channel is not running: nothing written */
    PUSH_DONE,
    PUSH_LOST,      /* its descriptors did not reach: the rest is lost */
};

/*
 * Give RX channel @n @len bytes, the last of a frame with @eof, written as
 * far as its descriptors reach, the rest lost with IN_DSCR_EMPTY.
 */
static int rx_push(ESP32P4H264State *s, int n, const uint8_t *data,
                   uint32_t len, bool eof)
{
    ESP32P4H264Chan *c = &s->rx[n];
    uint8_t tmp[512];

    if (chan_ctrl(s, true, n) & (CH_RST | CH_CMD_DISABLE) ||
        c->running == LINK_IDLE) {
        return PUSH_HELD;
    }
    for (int guard = 0; (len || eof) && guard < 1 << 20; guard++) {
        uint32_t size, k;

        if (c->running != LINK_RUN) {
            if (len) {
                c->int_raw |= RX_INT_DSCR_EMPTY;
                h264_update_irq(s);
                return PUSH_LOST;
            }
            return PUSH_DONE;
        }
        if (!c->have_desc && !chan_fetch(s, true, n)) {
            continue;
        }
        size = desc_size(c->dw);
        k = MIN(MIN(len, size - MIN(c->pos, size)), sizeof(tmp));
        if (k) {
            memcpy(tmp, data, k);
            chan_dbg(s, true, n, tmp, k, 8);
            h264_bus(s, true, n, chan_internal(true, n), c->dw[2] + c->pos,
                     tmp, k, true);
            if (n <= 2) {
                cnt_add(s, n, k / 8);
            }
            c->pos += k;
            data += k;
            len -= k;
        }
        if (c->pos >= size && (len || !eof)) {
            rx_desc_close(s, n, false);
        } else if (!len && eof) {
            rx_desc_close(s, n, true);
            eof = false;
        }
    }
    return PUSH_DONE;
}

static void chan_reset_link(ESP32P4H264Chan *c)
{
    c->running = LINK_IDLE;
    c->have_desc = false;
    c->pos = 0;
}

/* LINK_START, _STOP and _RESTART on channel @n. */
static void chan_link(ESP32P4H264State *s, bool rx, int n, uint32_t v)
{
    ESP32P4H264Chan *c = chan(s, rx, n);
    uint32_t stop = rx ? RX_LINK_STOP : TX_LINK_STOP;
    uint32_t start = rx ? RX_LINK_START : TX_LINK_START;
    uint32_t restart = rx ? RX_LINK_RESTART : TX_LINK_RESTART;

    if (v & stop) {
        c->running = LINK_IDLE;
    }
    if (!(v & (start | restart))) {
        return;
    }
    if (chan_ctrl(s, rx, n) & CH_RST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d started while held "
                      "in reset\n", TYPE_ESP32P4_H264, chan_name(rx), n);
        return;
    }
    if (c->etm_conf & ETM_EN) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d has ETM_EN set: it "
                      "waits for ETM tasks, which nothing on the chip sends\n",
                      TYPE_ESP32P4_H264, chan_name(rx), n);
        return;
    }
    if (v & start) {
        c->desc_addr = c->link_addr;
        c->dscr = c->link_addr;
    } else if (!c->have_desc) {
        /* Restart: the last descriptor's next pointer, read again. */
        uint32_t dw[4];
        const char *why;

        if (c->dscr_bf0 && desc_read(s, rx, n, c->dscr_bf0, dw, &why)) {
            c->desc_addr = dw[3];
        }
    }
    c->have_desc = v & start ? false : c->have_desc;
    c->pos = v & start ? 0 : c->pos;
    c->running = LINK_RUN;
}

/* The reference window */

static uint32_t block_len12(ESP32P4H264State *s)
{
    return extract32(s->block_len, 0, 14);
}

static uint32_t block_len4(ESP32P4H264State *s)
{
    return extract32(s->block_len, 14, 14);
}

static uint32_t block_row_stride(ESP32P4H264State *s)
{
    return extract32(s->block_row_len, 0, 16) +
           extract32(s->block_row_len, 16, 16);
}

static bool rx_held(ESP32P4H264State *s, int n)
{
    return s->rx[n].running == LINK_IDLE ||
           (chan_ctrl(s, true, n) & (CH_RST | CH_CMD_DISABLE));
}

static bool rx5_ready(ESP32P4H264State *s)
{
    return !(s->rx[5].conf0 & (CH_RST | CH_CMD_DISABLE));
}

/* The macroblock dimensions of video sequence @seq. */
static int seq_mb_w(ESP32P4H264State *s, int seq)
{
    return extract32(s->seq[seq][S_MB_RES], 7, 7);
}

static int seq_mb_h(ESP32P4H264State *s, int seq)
{
    return extract32(s->seq[seq][S_MB_RES], 0, 7);
}

/* The sequence the moving serves: the picture being coded, or the next. */
static int move_seq(ESP32P4H264State *s)
{
    if (s->state != ST_IDLE) {
        return s->stream;
    }
    return (s->gop_conf & GOP_DUAL) ? s->frames & 1 : 0;
}

/* Whether the next reference row can move now. */
static bool move_possible(ESP32P4H264State *s)
{
    int seq = move_seq(s), mbw = seq_mb_w(s, seq), mbh = seq_mb_h(s, seq);
    bool last = s->mv_row == mbh - 1;
    uint32_t b3 = (last ? H264E_MB_BYTES : H264E_LINE12_BYTES) * mbw;
    uint32_t b4 = last ? 0 : H264E_LINE4_BYTES * mbw;

    if (!mbw || !mbh || !rx5_ready(s)) {
        return false;
    }
    if (s->cnt[CNT_RX0] < b3 / 8 || s->cnt[CNT_RX1] < b4 / 8) {
        return false;
    }
    return tx_avail(s, 3, b3) && (!b4 || tx_avail(s, 4, b4));
}

/* The debug data replacing pixels of a macroblock layout's region. */
static void replace_layout(uint8_t *buf, uint32_t len, int ylines,
                           int clines, uint32_t px)
{
    uint32_t ny = 16 * ylines, nc = 8 * clines;

    for (uint32_t i = 0; i < len; i++) {
        uint32_t o = i % (ny + 2 * nc);

        buf[i] = o < ny ? px >> 16 : o < ny + nc ? px >> 8 : px;
    }
}

/* Move the next reference row into the window, through TX 3 and 4. */
static void move_row(ESP32P4H264State *s)
{
    int seq = move_seq(s), mbw = seq_mb_w(s, seq), mbh = seq_mb_h(s, seq);
    bool last = s->mv_row == mbh - 1;
    uint32_t b3 = (last ? H264E_MB_BYTES : H264E_LINE12_BYTES) * mbw;
    uint32_t b4 = last ? 0 : H264E_LINE4_BYTES * mbw;
    g_autofree uint8_t *d3 = g_malloc(b3);
    g_autofree uint8_t *d4 = g_malloc(MAX(b4, 1));
    int slot = s->ring_next;
    uint32_t base = s->block_start + slot * block_row_stride(s);

    tx_pull(s, 3, d3, b3);
    if (b4) {
        tx_pull(s, 4, d4, b4);
    }
    cnt_add(s, CNT_RX0, -(int64_t)(b3 / 8));
    cnt_add(s, CNT_RX1, -(int64_t)(b4 / 8));
    for (int x = 0; x < mbw; x++) {
        uint8_t mb[H264E_MB_BYTES];
        uint32_t a = base + x * (block_len12(s) + block_len4(s));

        if (last) {
            memcpy(mb, d3 + x * H264E_MB_BYTES, H264E_MB_BYTES);
        } else {
            memcpy(mb, d3 + x * H264E_LINE12_BYTES, H264E_LINE12_BYTES);
            memcpy(mb + H264E_LINE12_BYTES, d4 + x * H264E_LINE4_BYTES,
                   H264E_LINE4_BYTES);
        }
        chan_dbg(s, true, 5, mb, H264E_MB_BYTES, 8);
        if (!h264_in_range(s, true, a, H264E_MB_BYTES)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: RX channel 5 writes outside "
                          "the internal memory range at 0x%08" PRIx32 "\n",
                          TYPE_ESP32P4_H264, a);
            continue;
        }
        h264_bus(s, true, 5, true, a, mb, H264E_LINE12_BYTES, true);
        h264_bus(s, true, 5, true, a + block_len12(s),
                 mb + H264E_LINE12_BYTES, H264E_LINE4_BYTES, true);
    }
    if (s->ring_row[slot] >= 0) {
        cnt_add(s, CNT_RX5, -(int64_t)(MOVE_CYCLES_PER_MB * mbw));
    }
    cnt_add(s, CNT_RX5, MOVE_CYCLES_PER_MB * mbw);
    s->ring_row[slot] = s->mv_row;
    s->ring_gen[slot] = s->mv_gen;
    s->ring_next = (slot + 1) % 3;
    s->rx[5].int_raw |= RX_INT_DONE | (last ? RX_INT_SUC_EOF : 0);
    if (++s->mv_row >= mbh) {
        s->mv_row = 0;
        s->mv_gen++;
    }
    h264_update_irq(s);
}

static void h264_move_cb(void *opaque)
{
    ESP32P4H264State *s = opaque;
    int64_t when = esp32p4_clkwait_expire(&s->move_wait);

    if (s->mv_eager <= 0 || !move_possible(s)) {
        return;
    }
    move_row(s);
    if (--s->mv_eager == 0) {
        h264_raise(s, INT_MOVE_DONE);
        h264_kick(s, when);
        return;
    }
    esp32p4_clkwait_start(&s->move_wait, when,
                          MOVE_CYCLES_PER_MB * seq_mb_w(s, move_seq(s)));
}

/* DMA_MOVE_START: begin a new pass over the reference. */
static void h264_move_start(ESP32P4H264State *s, int64_t now)
{
    int seq = move_seq(s), mbw = seq_mb_w(s, seq), mbh = seq_mb_h(s, seq);

    s->mv_gen++;
    s->mv_row = 0;
    s->next_ref_gen = s->mv_gen;
    s->mv_started = true;
    s->mv_eager = MIN(2, mbh);
    if (!mbw || !mbh) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: DMA_MOVE_START with no "
                      "macroblocks configured\n", TYPE_ESP32P4_H264);
        s->mv_eager = 0;
        return;
    }
    esp32p4_clkwait_start(&s->move_wait, now, MOVE_CYCLES_PER_MB * mbw);
}

static bool ring_has(ESP32P4H264State *s, int row, uint32_t gen)
{
    for (int i = 0; i < 3; i++) {
        if (s->ring_row[i] == row && s->ring_gen[i] == gen) {
            return true;
        }
    }
    return false;
}

/*
 * Bring reference rows @r - 1 to @r + 1 into the window, moving as far as
 * the data allows: 1 when there, 0 to wait, -1 if they never can be.
 */
static int h264_ref_rows(ESP32P4H264State *s, int r)
{
    int mbh = s->enc.cfg.mb_h;

    if (!s->mv_started) {
        return -1;
    }
    for (int k = MAX(r - 1, 0); k <= MIN(r + 1, mbh - 1); k++) {
        while (!ring_has(s, k, s->ref_gen)) {
            if (s->mv_gen > s->ref_gen ||
                (s->mv_gen == s->ref_gen && s->mv_row > k)) {
                return -1;
            }
            if (s->mv_eager > 0 || !move_possible(s)) {
                return 0;
            }
            move_row(s);
        }
    }
    return 1;
}

/* Source and reference reads */

/*
 * Read source row @r through TX channel 0 into the core: false if the
 * channel cannot give it now. With @dry, only fetch the descriptor it
 * comes from, as the channel does ahead of its data.
 */
static bool h264_read_source(ESP32P4H264State *s, int r, bool dry)
{
    ESP32P4H264Chan *c = &s->tx[0];
    int pb2 = pbyte2(s), cs = extract32(s->ori_conf[s->stream], 0, 3);
    uint32_t ha, va, lb, lines, need;
    bool test = s->mode_enable & 1;

    if (!chan_ready(s, false, 0)) {
        return false;
    }
    for (int guard = 0;; guard++) {
        if (guard == 64 || (!c->have_desc && !chan_fetch(s, false, 0))) {
            return false;
        }
        if (!(c->dw[0] & DW0_2D)) {
            qemu_log_mask(LOG_UNIMP, "%s: TX channel 0 needs 2D "
                          "descriptors\n", TYPE_ESP32P4_H264);
            return false;
        }
        if (c->pos < DW1_SIZE_HI(c->dw[1])) {
            break;
        }
        tx_desc_done(s, 0);
        if (c->running != LINK_RUN) {
            return false;
        }
    }
    if (dry) {
        return true;
    }
    if (!pb2 || pb2 == 1) {
        qemu_log_mask(LOG_UNIMP, "%s: ORI_PBYTE %u is not modelled\n",
                      TYPE_ESP32P4_H264, s->pbyte);
        pb2 = 3;
    }
    if (!(c->dw[1] & DW1_MODE)) {
        qemu_log_mask(LOG_UNIMP, "%s: TX channel 0's mode 0 descriptors are "
                      "taken as whole pictures\n", TYPE_ESP32P4_H264);
    }
    ha = DW1_LEN_HI(c->dw[1]);
    va = DW1_SIZE_HI(c->dw[1]);
    lb = (uint64_t)ha * pb2 / 2;
    lines = MIN(16, va - c->pos);
    need = 16 * lb;
    if (need > s->lines_size) {
        s->lines = g_realloc(s->lines, need);
        s->lines_size = need;
    }
    for (uint32_t j = 0; j < lines; j++) {
        h264_bus(s, false, 0, false, c->dw[2] + (c->pos + j) * lb,
                 s->lines + j * lb, lb, false);
        chan_dbg(s, false, 0, s->lines + j * lb, lb, pb2 & 1 ? 3 : pb2 / 2);
    }
    {
        uint32_t hb = DW0_LEN_LO(c->dw[0]);

        if (hb != 16 && !(c->conf0 & TX_REORDER_EN) && hb && lines == 16) {
            /*
             * Without reordering the encoder takes the DMA's blocks, each
             * HB pixels by 16 lines in raster order, as 16-pixel
             * macroblocks.
             */
            g_autofree uint8_t *st = g_malloc(need);
            g_autofree uint8_t *mb = g_malloc(need);
            uint32_t o = 0, mbb = 16 * pb2 / 2;

            for (uint32_t x = 0; x < ha; x += hb) {
                uint32_t w = (uint64_t)MIN(hb, ha - x) * pb2 / 2;

                for (int j = 0; j < 16; j++) {
                    memcpy(st + o, s->lines + j * lb + x * pb2 / 2, w);
                    o += w;
                }
            }
            memset(mb, 0, need);
            for (uint32_t m = 0; (m + 1) * mbb <= lb; m++) {
                for (int j = 0; j < 16; j++) {
                    memcpy(mb + j * lb + m * mbb, st + (m * 16 + j) * mbb,
                           mbb);
                }
            }
            memcpy(s->lines, mb, need);
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TX channel 0's blocks are %u "
                          "pixels wide without OUT_REORDER_EN_CH0\n",
                          TYPE_ESP32P4_H264, hb);
        }
    }
    if (test) {
        h264e_fill_source_row(&s->enc, r, extract32(s->mode_yuv, 0, 8),
                              extract32(s->mode_yuv, 8, 8),
                              extract32(s->mode_yuv, 16, 8));
    } else {
        uint32_t d = s->debug_conf[D_ORI];

        h264e_load_source_row(&s->enc, r, s->lines, lb, lines, ha, cs,
                              d & 1, extract32(d, 1, 24));
    }
    c->pos += 16;
    if (c->pos >= va) {
        tx_desc_done(s, 0);
    }
    return true;
}

/*
 * Fetch a pass over the window through TX channel 1 and give the core the
 * rows it holds of the picture's reference.
 */
static bool h264_read_ref(ESP32P4H264State *s, int r, bool dry)
{
    ESP32P4H264Chan *c = &s->tx[1];
    int mbw = s->enc.cfg.mb_w;
    uint32_t lb, va;
    uint32_t d = s->debug_conf[D_REF];

    if (!chan_ready(s, false, 1) ||
        (!c->have_desc && !chan_fetch(s, false, 1))) {
        return false;
    }
    if (dry) {
        return true;
    }
    lb = (uint64_t)DW1_LEN_HI(c->dw[1]) * 3 / 2;
    va = DW1_SIZE_HI(c->dw[1]);
    if (!(c->dw[0] & DW0_2D) || va < 3 || lb < (uint32_t)mbw * H264E_MB_BYTES) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TX channel 1's descriptor does "
                      "not span the reference window\n", TYPE_ESP32P4_H264);
    }
    for (int slot = 0; slot < 3 && slot < (int)va; slot++) {
        int row = s->ring_row[slot];

        if (row < MAX(r - 1, 0) || row > r + 1 ||
            s->ring_gen[slot] != s->ref_gen) {
            continue;
        }
        for (int x = 0; x < mbw; x++) {
            uint8_t mb[H264E_MB_BYTES];

            if ((uint32_t)(x + 1) * H264E_MB_BYTES > lb) {
                memset(mb, 0, sizeof(mb));
            } else {
                h264_bus(s, false, 1, true,
                         c->dw[2] + slot * lb + x * H264E_MB_BYTES, mb,
                         sizeof(mb), false);
                chan_dbg(s, false, 1, mb, sizeof(mb), 8);
            }
            if (d & 1) {
                replace_layout(mb, H264E_LINE12_BYTES, 12, 4,
                               extract32(d, 1, 24));
                replace_layout(mb + H264E_LINE12_BYTES, H264E_LINE4_BYTES, 4,
                               4, extract32(d, 1, 24));
            }
            h264e_ref_put(&s->enc, row, x, mb);
        }
    }
    tx_desc_done(s, 1);
    return true;
}

/* Row r - 1's bottom lines, from TX channel 2. */
static bool h264_read_tmp(ESP32P4H264State *s, int r, bool dry)
{
    int mbw = s->enc.cfg.mb_w;
    uint32_t need = H264E_LINE4_BYTES * mbw;
    g_autofree uint8_t *buf = NULL;
    uint32_t d = s->debug_conf[D_DB_RD_TEMP];

    if (s->cnt[CNT_RX2] < need / 8 || !tx_avail(s, 2, need) ||
        (!s->tx[2].have_desc && !chan_fetch(s, false, 2))) {
        return false;
    }
    if (dry) {
        return true;
    }
    buf = g_malloc(need);
    if (!tx_pull(s, 2, buf, need)) {
        return false;
    }
    cnt_add(s, CNT_RX2, -(int64_t)(need / 8));
    if (d & 1) {
        replace_layout(buf, need, 4, 4, extract32(d, 1, 24));
    }
    for (int x = 0; x < mbw; x++) {
        h264e_line4_put(&s->enc, r - 1, x, buf + x * H264E_LINE4_BYTES);
    }
    return true;
}

/* The picture */

static void h264_row_read(ESP32P4H264State *s, int64_t now);
static void h264_row_write(ESP32P4H264State *s, int64_t now);

static void h264_row_cb(void *opaque)
{
    ESP32P4H264State *s = opaque;
    int64_t when = esp32p4_clkwait_expire(&s->row_wait);

    if (s->state != ST_CODE) {
        return;
    }
    s->state = ST_WRITE;
    h264_row_write(s, when);
}

/* Row 0's bottom lines of its first @n macroblocks, to RX channel 2. */
static bool h264_write_tmp(ESP32P4H264State *s, int r, int from, int to,
                           bool dry)
{
    uint32_t len = H264E_LINE4_BYTES * (to - from);
    g_autofree uint8_t *buf = NULL;
    uint32_t d = s->debug_conf[D_DB_WR_TEMP];

    if (to <= from) {
        return true;
    }
    if (rx_held(s, 2)) {
        return false;
    }
    if (dry) {
        return true;
    }
    buf = g_malloc(len);
    for (int x = from; x < to; x++) {
        h264e_line4_get(&s->enc, r, x, buf + (x - from) * H264E_LINE4_BYTES);
    }
    if (d & 1) {
        replace_layout(buf, len, 4, 4, extract32(d, 1, 24));
    }
    rx_push(s, 2, buf, len, to == s->enc.cfg.mb_w &&
            r == s->enc.cfg.mb_h - 2);
    return true;
}

static void h264_tmp_cb(void *opaque)
{
    ESP32P4H264State *s = opaque;
    int trigger = extract32(s->seq[s->stream][S_SYS_CONF], 0, 7);

    if (esp32p4_clkwait_active(&s->tmp_wait)) {
        esp32p4_clkwait_expire(&s->tmp_wait);
    }
    s->tmp_pending = false;
    if (s->state != ST_CODE || s->row != 0 || s->tmp_ready) {
        return;
    }
    if (h264_write_tmp(s, 0, 0, trigger, false)) {
        s->tmp_written = trigger;
        s->tmp_ready = true;
        h264_raise(s, INT_DB_TMP_READY);
    } else {
        /* Held until RX channel 2 runs. */
        s->tmp_pending = true;
    }
}

static void h264_frame_done(ESP32P4H264State *s, int64_t now)
{
    H264Enc *e = &s->enc;

    s->rc_status[0] = e->mad_sum & 0x1fffff;
    s->rc_status[1] = e->enc_bits & 0x7ffffff;
    s->rc_status[2] = e->qp_sum & 0x7ffff;
    s->frame_code_length = s->code_bytes & 0xffffff;
    s->state = ST_IDLE;
    if (!s->intra && !s->frame_mode) {
        /* GOP mode: the moving runs on into this picture's own reference. */
        s->next_ref_gen++;
    }
    h264_raise(s, INT_FRAME_DONE);
}

/*
 * The end of a row: its results to the RX channels, or held back while
 * one they need is not running.
 */
static void h264_row_write(ESP32P4H264State *s, int64_t now)
{
    H264Enc *e = &s->enc;
    int r = s->row, mbw = e->cfg.mb_w, mbh = e->cfg.mb_h;
    bool last = r == mbh - 1;
    bool mvm = !s->intra && e->cfg.mv_merge_en;
    uint32_t db = s->debug_conf[D_DB_WR];
    uint32_t b0 = (last ? H264E_MB_BYTES : H264E_LINE12_BYTES) * mbw;
    uint32_t b1 = H264E_LINE4_BYTES * mbw;
    g_autofree uint8_t *d0 = NULL;
    g_autofree uint8_t *d1 = NULL;
    g_autofree uint8_t *bs = NULL;
    size_t nbs;

    for (int n = 0; n <= 4; n++) {
        bool needed = n == 0 || n == 4 || (n == 1 && r > 0) ||
                      (n == 2 && !last && s->tmp_written < mbw) ||
                      (n == 3 && mvm);

        if (needed && rx_held(s, n)) {
            return;
        }
    }
    d0 = g_malloc(b0);
    for (int x = 0; x < mbw; x++) {
        h264e_mb_get(e, r, x, d0 + x * (b0 / mbw), true, last);
    }
    if (db & 1) {
        for (int x = 0; x < mbw; x++) {
            replace_layout(d0 + x * (b0 / mbw), H264E_LINE12_BYTES, 12, 4,
                           extract32(db, 1, 24));
            if (last) {
                replace_layout(d0 + x * H264E_MB_BYTES + H264E_LINE12_BYTES,
                               H264E_LINE4_BYTES, 4, 4, extract32(db, 1, 24));
            }
        }
    }
    rx_push(s, 0, d0, b0, last);
    if (r > 0) {
        d1 = g_malloc(b1);
        for (int x = 0; x < mbw; x++) {
            h264e_mb_get(e, r - 1, x, d1 + x * H264E_LINE4_BYTES, false, true);
        }
        if (db & 1) {
            replace_layout(d1, b1, 4, 4, extract32(db, 1, 24));
        }
        rx_push(s, 1, d1, b1, last);
    }
    if (!last) {
        h264_write_tmp(s, r, s->tmp_written, mbw, false);
    }
    s->tmp_written = 0;
    if (r == 0 && !last && !s->tmp_ready &&
        extract32(s->seq[s->stream][S_SYS_CONF], 0, 7) < mbw) {
        s->tmp_ready = true;
        h264_raise(s, INT_DB_TMP_READY);
    }
    if (mvm) {
        uint8_t mv[4 * H264E_MAX_MB_W];
        uint32_t dm = s->debug_conf[D_MVM];

        for (int i = 0; i < e->mvm_n; i++) {
            stl_le_p(mv + 4 * i, e->mvm[i]);
        }
        if (dm & 1) {
            memset(mv, extract32(dm, 1, 8), 4 * e->mvm_n);
        }
        rx_push(s, 3, mv, 4 * e->mvm_n, last);
        s->mv_merge = deposit32(s->mv_merge, 5, 13, e->mvm_total);
    }
    if (last) {
        h264e_finish(e);
    }
    nbs = h264e_pending(e) * 3 / 2 + 16;
    bs = g_malloc(nbs);
    nbs = h264e_take(e, bs, nbs);
    if (s->debug_conf[D_BS] & 1) {
        memset(bs, extract32(s->debug_conf[D_BS], 1, 8), nbs);
    }
    if (rx_push(s, 4, bs, nbs, last) == PUSH_LOST && !s->bs_overflow) {
        s->bs_overflow = true;
        h264_raise(s, INT_BS_OVERFLOW);
    }
    s->code_bytes += nbs;
    if (r == (int)extract32(s->seq[s->stream][S_SYS_CONF], 7, 7)) {
        h264_raise(s, INT_REC_READY);
    }
    if (last) {
        h264_frame_done(s, now);
        h264_kick(s, now);
        return;
    }
    s->row++;
    s->state = ST_READ;
    h264_row_read(s, now);
}

/* The start of a row: its inputs, read only when all are there. */
static void h264_row_read(ESP32P4H264State *s, int64_t now)
{
    H264Enc *e = &s->enc;
    int r = s->row, mbw = e->cfg.mb_w;
    int trigger = extract32(s->seq[s->stream][S_SYS_CONF], 0, 7);

    if (!s->intra) {
        int ok = h264_ref_rows(s, r);

        if (ok < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: the reference rows of row %d "
                          "were never moved into the window (a P picture "
                          "needs DMA_MOVE_START in frame mode, and the "
                          "moving restarted with TX channels 3 and 4)\n",
                          TYPE_ESP32P4_H264, r);
            return;
        }
        if (!ok || !h264_read_ref(s, r, true)) {
            return;
        }
    }
    if (!h264_read_source(s, r, true) ||
        (r > 0 && !h264_read_tmp(s, r, true))) {
        return;
    }
    if ((r > 0 && !h264_read_tmp(s, r, false)) ||
        (!s->intra && !h264_read_ref(s, r, false)) ||
        !h264_read_source(s, r, false)) {
        return;
    }
    h264e_code_row(e, r);
    s->state = ST_CODE;
    esp32p4_clkwait_start(&s->row_wait, now, (uint64_t)mbw * CYCLES_PER_MB);
    if (r == 0 && e->cfg.mb_h > 1 && trigger < mbw) {
        esp32p4_clkwait_start(&s->tmp_wait, now,
                              (uint64_t)trigger * CYCLES_PER_MB);
    }
}

/* Retry whatever waits for a channel, a counter or the moving. */
static void h264_kick(ESP32P4H264State *s, int64_t now)
{
    if (s->mv_eager > 0 && !esp32p4_clkwait_active(&s->move_wait) &&
        move_possible(s)) {
        esp32p4_clkwait_start(&s->move_wait, now,
                              MOVE_CYCLES_PER_MB *
                              seq_mb_w(s, move_seq(s)));
    }
    if (s->state == ST_READ) {
        h264_row_read(s, now);
    } else if (s->state == ST_WRITE) {
        h264_row_write(s, now);
    }
    if (s->tmp_pending) {
        h264_tmp_cb(s);
    }
}

/* SYS_RST_PULSE */
static void h264_enc_reset(ESP32P4H264State *s)
{
    esp32p4_clkwait_stop(&s->row_wait);
    esp32p4_clkwait_stop(&s->tmp_wait);
    esp32p4_clkwait_stop(&s->move_wait);
    s->state = ST_IDLE;
    s->frames = 0;
    s->tmp_pending = false;
    s->mv_started = false;
    s->mv_eager = 0;
    s->mv_row = 0;
}

static void h264_seq_config(ESP32P4H264State *s, int seq, H264EncConfig *c)
{
    const uint32_t *r = s->seq[seq];

    memset(c, 0, sizeof(*c));
    c->mb_w = seq_mb_w(s, seq);
    c->mb_h = seq_mb_h(s, seq);
    c->qp = extract32(r[S_RC_CONF0], 0, 6);
    c->rc_u = extract32(r[S_RC_CONF0], 6, 16);
    c->rc_en = extract32(r[S_RC_CONF0], 22, 1);
    c->chroma_qp_offset = sextract32(r[S_RC_CONF1], 3, 4);
    c->qp_min = extract32(r[S_RC_CONF1], 7, 6);
    c->qp_max = extract32(r[S_RC_CONF1], 13, 6);
    c->mad_pred = extract32(r[S_RC_CONF1], 19, 12);
    c->deblock = !(r[S_DB_BYPASS] & 1);
    c->intra_cost_offset = extract32(r[S_SYS_CONF], 14, 16);
    c->c_deci_score = extract32(r[S_DECI_SCORE], 0, 10);
    c->l_deci_score = extract32(r[S_DECI_SCORE], 10, 10);
    c->i16_deci_offset = extract32(r[S_DECI_OFFSET], 0, 6);
    c->ic_deci_offset = extract32(r[S_DECI_OFFSET], 6, 6);
    c->p16_deci_offset = extract32(r[S_DECI_OFFSET], 12, 6);
    c->pc_deci_offset = extract32(r[S_DECI_OFFSET], 18, 6);
    c->roi_en = r[S_ROI_CONFIG] & 1;
    c->roi_delta = r[S_ROI_CONFIG] & 2;
    for (int i = 0; i < 8; i++) {
        uint32_t v = r[S_ROI0 + i];

        c->roi[i].x = extract32(v, 0, 7);
        c->roi[i].y = extract32(v, 7, 7);
        c->roi[i].w = extract32(v, 14, 7);
        c->roi[i].h = extract32(v, 21, 7);
        c->roi[i].en = extract32(v, 28, 1);
        c->roi[i].qp = extract32(r[i < 4 ? S_ROI03_QP : S_ROI47_QP],
                                 7 * (i % 4), 7);
    }
    c->no_roi_qp = extract32(r[S_NO_ROI_QP], 0, 7);
    c->mv_merge_en = s->mv_merge & (seq ? MV_MERGE_B_EN : MV_MERGE_A_EN);
    c->mv_merge_type = extract32(s->mv_merge, 0, 2);
    c->int_mv_out = extract32(s->mv_merge, 2, 1);
}

/* FRAME_START */
static void h264_frame_start(ESP32P4H264State *s, int64_t now)
{
    bool dual = s->gop_conf & GOP_DUAL;
    uint32_t gop = GOP_NUM(s->gop_conf), pos;
    int seq = dual ? s->frames & 1 : 0;
    H264EncConfig cfg;
    uint8_t hdr[8];
    int hdr_len = extract32(s->slice_len, 0, 4);
    int cs = extract32(s->ori_conf[seq], 0, 3);

    if (s->state != ST_IDLE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: FRAME_START while a picture is "
                      "being coded\n", TYPE_ESP32P4_H264);
        return;
    }
    if (dual && !s->frame_mode) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: dual-stream mode needs "
                      "FRAME_MODE set\n", TYPE_ESP32P4_H264);
    }
    h264_seq_config(s, seq, &cfg);
    if (!cfg.mb_w || !cfg.mb_h || cfg.mb_w > MAX_MB_X) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: FRAME_START refused: a picture "
                      "of %d x %d macroblocks (the width at most %d)\n",
                      TYPE_ESP32P4_H264, cfg.mb_w, cfg.mb_h, MAX_MB_X);
        return;
    }
    if (cs > H264E_CS_GRAY) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: FRAME_START refused: original "
                      "colour space %d is invalid\n", TYPE_ESP32P4_H264, cs);
        return;
    }
    if (cfg.qp > 51) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: QP %d is beyond 51\n",
                      TYPE_ESP32P4_H264, cfg.qp);
    }
    if (extract32(s->seq[seq][S_RC_CONF1], 0, 3)) {
        qemu_log_mask(LOG_UNIMP, "%s: CHROMA_DC_QP_DELTA is not modelled\n",
                      TYPE_ESP32P4_H264);
    }
    if (hdr_len > 8) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SLICE_BYTE_LENGTH %d is beyond "
                      "the 8 bytes SLICE_HEADER_BYTE0/1 hold\n",
                      TYPE_ESP32P4_H264, hdr_len);
        hdr_len = 8;
    }
    pos = dual ? s->frames >> 1 : s->frames;
    cfg.intra = gop ? pos % gop == 0 : pos == 0;
    for (int i = 0; i < 8; i++) {
        hdr[i] = (i < 4 ? s->slice_byte[1] >> (24 - 8 * i)
                        : s->slice_byte[0] >> (56 - 8 * i)) & 0xff;
    }
    s->stream = seq;
    s->intra = cfg.intra;
    s->sys_status = (gop ? s->frames % (gop * (dual ? 2 : 1))
                         : s->frames) & 0x1ff;
    s->sys_status |= seq << 9 | cfg.intra << 10;
    s->frames++;
    h264e_begin(&s->enc, &cfg, hdr, hdr_len,
                extract32(s->slice_remain, 3, 8),
                extract32(s->slice_remain, 0, 3));
    if (!cfg.intra) {
        s->ref_gen = s->next_ref_gen;
    }
    s->mv_merge = deposit32(s->mv_merge, 5, 13, 0);
    s->row = 0;
    s->code_bytes = 0;
    s->bs_overflow = false;
    s->tmp_written = 0;
    s->tmp_ready = false;
    s->tmp_pending = false;
    s->state = ST_READ;
    h264_row_read(s, now);
}

/* The encoder's registers */

static int seq_of(hwaddr a, int *idx)
{
    if (a >= A_SEQ_A && a < A_SEQ_B) {
        *idx = (a - A_SEQ_A) / 4;
        return 0;
    }
    if (a >= A_SEQ_B && a < A_SEQ_B + 4 * ESP32P4_H264_SEQ_REGS) {
        *idx = (a - A_SEQ_B) / 4;
        return 1;
    }
    return -1;
}

static uint64_t h264_enc_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4H264State *s = opaque;
    int idx, seq = seq_of(addr, &idx);

    if (seq >= 0) {
        return s->seq[seq][idx];
    }
    switch (addr) {
    case A_SYS_CTRL:
        return s->frame_mode ? SYS_FRAME_MODE : 0;
    case A_GOP_CONF:
        return s->gop_conf;
    case A_RC_STATUS0:
    case A_RC_STATUS1:
    case A_RC_STATUS2:
        return s->rc_status[(addr - A_RC_STATUS0) / 4];
    case A_SLICE_REMAIN:
        return s->slice_remain;
    case A_SLICE_LEN:
        return s->slice_len;
    case A_BS_THRESHOLD:
        return s->bs_threshold;
    case A_SLICE_BYTE0:
    case A_SLICE_BYTE1:
        return s->slice_byte[(addr - A_SLICE_BYTE0) / 4];
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_CONF:
        return s->conf;
    case A_MV_MERGE:
        return s->mv_merge;
    case A_DEBUG_DMA_SEL:
        return s->debug_dma_sel;
    case A_SYS_STATUS:
        return s->sys_status;
    case A_FRAME_CODE_LENGTH:
        return s->frame_code_length;
    case A_DEBUG_INFO0 ... A_DEBUG_INFO2:
        return 0;
    case A_DATE:
        return s->date;
    case A_ORI_CONF_A:
    case A_ORI_CONF_B:
        return s->ori_conf[(addr - A_ORI_CONF_A) / 4];
    case A_ORI_DEBUG ... A_REF_DEBUG:
        return s->debug_conf[(addr - A_ORI_DEBUG) / 4];
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved encoder "
                      "register 0x%03" HWADDR_PRIx "\n", TYPE_ESP32P4_H264,
                      addr);
        return 0;
    }
}

static void h264_enc_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    ESP32P4H264State *s = opaque;
    uint32_t v = value;
    int64_t now = h264_now();
    int idx, seq = seq_of(addr, &idx);

    if (seq >= 0) {
        s->seq[seq][idx] = v & seq_mask[idx];
        return;
    }
    switch (addr) {
    case A_SYS_CTRL:
        s->frame_mode = v & SYS_FRAME_MODE;
        if (v & SYS_RST_PULSE) {
            h264_enc_reset(s);
        }
        if (v & SYS_DMA_MOVE_START) {
            h264_move_start(s, now);
        }
        if (v & SYS_FRAME_START) {
            h264_frame_start(s, now);
        }
        break;
    case A_GOP_CONF:
        s->gop_conf = v & 0x1ff;
        break;
    case A_SLICE_REMAIN:
        s->slice_remain = v & 0x7ff;
        break;
    case A_SLICE_LEN:
        s->slice_len = v & 0xf;
        break;
    case A_BS_THRESHOLD:
        s->bs_threshold = v & 0x7f;
        break;
    case A_SLICE_BYTE0:
    case A_SLICE_BYTE1:
        s->slice_byte[(addr - A_SLICE_BYTE0) / 4] = v;
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~v;
        h264_update_irq(s);
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_MASK;
        h264_update_irq(s);
        break;
    case A_CONF:
        s->conf = v & 0x0fffffff;
        break;
    case A_MV_MERGE:
        s->mv_merge = (s->mv_merge & ~MV_MERGE_RW) | (v & MV_MERGE_RW);
        if (extract32(v, 0, 2) == 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: MV_MERGE_TYPE 3 is invalid: "
                          "no motion vectors are kept\n", TYPE_ESP32P4_H264);
        }
        break;
    case A_DEBUG_DMA_SEL:
        s->debug_dma_sel = v & 0xff;
        break;
    case A_DATE:
        s->date = v & 0x0fffffff;
        break;
    case A_ORI_CONF_A:
    case A_ORI_CONF_B:
        s->ori_conf[(addr - A_ORI_CONF_A) / 4] = v & 0x7;
        break;
    case A_ORI_DEBUG ... A_REF_DEBUG:
        s->debug_conf[(addr - A_ORI_DEBUG) / 4] =
            v & debug_conf_mask[(addr - A_ORI_DEBUG) / 4];
        break;
    case A_RC_STATUS0:
    case A_RC_STATUS1:
    case A_RC_STATUS2:
    case A_INT_ST:
    case A_SYS_STATUS:
    case A_FRAME_CODE_LENGTH:
    case A_DEBUG_INFO0 ... A_DEBUG_INFO2:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only encoder "
                      "register 0x%03" HWADDR_PRIx "\n", TYPE_ESP32P4_H264,
                      addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved encoder "
                      "register 0x%03" HWADDR_PRIx "\n", TYPE_ESP32P4_H264,
                      addr);
        break;
    }
}

static const MemoryRegionOps h264_enc_ops = {
    .read = h264_enc_read,
    .write = h264_enc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* H264_DMA's registers */

/* Which channel register @a is: false if it is not one. */
static bool dma_decode(hwaddr a, bool *rx, int *n, hwaddr *off)
{
    if (a < DMA_RX_BASE) {
        *rx = false;
        *n = a / 0x100;
        *off = a % 0x100;
        return true;
    }
    if (a < DMA_RX5_BASE) {
        *rx = true;
        *n = (a - DMA_RX_BASE) / 0x100;
        *off = a % 0x100;
        return true;
    }
    if (a < DMA_GLOBAL_BASE) {
        *rx = true;
        *n = 5;
        *off = a - DMA_RX5_BASE;
        return true;
    }
    return false;
}

static uint32_t tx_conf0_mask(int n)
{
    return n == 0 ? 0x070111df : n < 3 ? 0x070011df : 0x040011df;
}

static uint32_t chan_state(ESP32P4H264State *s, bool rx, int n)
{
    ESP32P4H264Chan *c = chan(s, rx, n);
    uint32_t v = extract32(c->desc_addr, 0, 18);

    if (rx) {
        return n == 5 ? BIT(3) : v | BIT(23);
    }
    return n < 3 ? v | BIT(24) : v;
}

static bool dma_chan_read(ESP32P4H264State *s, bool rx, int n, hwaddr o,
                          uint32_t *v)
{
    ESP32P4H264Chan *c = chan(s, rx, n);

    if (rx && n == 5) {
        switch (o) {
        case 0x00:
            *v = c->conf0;
            return true;
        case 0x04:
            *v = s->block_start;
            return true;
        case 0x08:
            *v = s->block_row_len;
            return true;
        case 0x0c:
            *v = s->block_len;
            return true;
        case 0x10:
            *v = c->int_raw;
            return true;
        case 0x14:
            *v = c->int_ena;
            return true;
        case 0x18:
            *v = c->int_raw & c->int_ena;
            return true;
        case 0x1c:
            *v = 0;
            return true;
        case 0x20:
            *v = RX5_FIFO_STATUS_RESET;
            return true;
        case 0x24:
            *v = POP_RDATA_EMPTY;
            return true;
        case 0x28:
            *v = chan_state(s, rx, n);
            return true;
        case 0x40:
            *v = c->arb;
            return true;
        case 0x80:
        case 0x88:
        case 0x8c:
            *v = 0;
            return true;
        case 0x84:
            *v = 0xff;
            return true;
        default:
            return false;
        }
    }
    switch (o) {
    case 0x00:
        *v = c->conf0;
        return true;
    case 0x04:
        *v = c->int_raw;
        return true;
    case 0x08:
        *v = c->int_ena;
        return true;
    case 0x0c:
        *v = c->int_raw & c->int_ena;
        return true;
    case 0x10:
        *v = 0;
        return true;
    case 0x14:
        *v = TX_FIFO_STATUS_RESET;
        return true;
    case 0x18:
        *v = rx ? POP_RDATA_EMPTY : 0;
        return true;
    case 0x1c:
        *v = c->link_conf | (c->running == LINK_RUN ? 0 :
                             rx ? RX_LINK_PARK : TX_LINK_PARK);
        return true;
    case 0x20:
        *v = c->link_addr;
        return true;
    case 0x24:
        *v = chan_state(s, rx, n);
        return true;
    }
    if (!rx) {
        switch (o) {
        case 0x28:
            *v = c->eof_addr;
            return true;
        case 0x2c:
            *v = c->dscr;
            return true;
        case 0x30:
            *v = c->dscr_bf0;
            return true;
        case 0x34:
            *v = c->dscr_bf1;
            return true;
        case 0x3c:
            *v = c->arb;
            return true;
        case 0x40:
            *v = n == 0 ? 0x800 : 0;
            return n == 0;
        case 0x44:
            *v = c->pd_conf;
            return n == 0;
        case 0x50:
            *v = s->mode_enable;
            return n == 0;
        case 0x54:
            *v = s->mode_yuv;
            return n == 0;
        case 0x68:
            *v = c->etm_conf;
            return true;
        case 0x70:
        case 0x74:
        case 0x7c:
            *v = 0;
            return true;
        case 0x78:
            *v = n < 3 ? 0xff : 0x3f;
            return true;
        case 0x80:
            *v = 0;
            return n >= 3;
        default:
            return false;
        }
    }
    switch (o) {
    case 0x28:
        *v = c->eof_addr;
        return true;
    case 0x2c:
        *v = c->err_eof_addr;
        return true;
    case 0x30:
        *v = c->dscr;
        return true;
    case 0x34:
        *v = c->dscr_bf0;
        return true;
    case 0x38:
        *v = c->dscr_bf1;
        return true;
    case 0x40:
        *v = c->arb;
        return true;
    case 0x48:
        *v = n == 0 ? c->pd_conf : c->etm_conf;
        return true;
    case 0x6c:
        *v = c->etm_conf;
        return n == 0;
    case 0x80:
    case 0x88:
    case 0x8c:
        *v = 0;
        return true;
    case 0x84:
        *v = 7;
        return true;
    default:
        return false;
    }
}

static uint64_t h264_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4H264State *s = opaque;
    bool rx;
    int n;
    hwaddr o;
    uint32_t v = 0;

    if (dma_decode(addr, &rx, &n, &o)) {
        if (dma_chan_read(s, rx, n, o, &v)) {
            return v;
        }
    } else {
        switch (addr) {
        case 0xb00:
        case 0xb04:
            return s->axi_err[(addr - 0xb00) / 4];
        case 0xb08:
            return s->rst_conf;
        case 0xb0c ... 0xb18:
            return s->mem_addr[(addr - 0xb0c) / 4];
        case 0xb20 ... 0xb2c:
            return s->mem_addr[4 + (addr - 0xb20) / 4];
        case 0xb30:
        case 0xb34:
            return s->arb_config[(addr - 0xb30) / 4];
        case 0xb3c:
            return s->dma_date;
        case 0xb50:
            return s->counter_rst;
        case 0xb54 ... 0xb60:
            return s->cnt[(addr - 0xb54) / 4] & cnt_mask[(addr - 0xb54) / 4];
        case 0xb64:
            return s->pbyte;
        case 0xb68:
            return s->ch_dbg_en;
        case 0xb6c ... 0xbc0:
            return s->dbg_data[(addr - 0xb6c) / 4];
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved H264_DMA register "
                  "0x%03" HWADDR_PRIx "\n", TYPE_ESP32P4_H264, addr);
    return 0;
}

/* RX channel 5's reset: TX channels 3 and 4 and the window with it. */
static void rx5_reset(ESP32P4H264State *s)
{
    chan_reset_link(&s->tx[3]);
    chan_reset_link(&s->tx[4]);
    for (int i = 0; i < 3; i++) {
        s->ring_row[i] = -1;
    }
    s->ring_next = 0;
}

static void dma_chan_write(ESP32P4H264State *s, bool rx, int n, hwaddr o,
                           uint32_t v, int64_t now)
{
    ESP32P4H264Chan *c = chan(s, rx, n);
    bool ok = true;

    if (rx && n == 5) {
        switch (o) {
        case 0x00:
            c->conf0 = v & 0x030011c8;
            if (c->conf0 & CH_RST) {
                rx5_reset(s);
            }
            break;
        case 0x04:
            s->block_start = v;
            break;
        case 0x08:
            s->block_row_len = v;
            break;
        case 0x0c:
            s->block_len = v & 0x0fffffff;
            break;
        case 0x10:
        case 0x1c:
            c->int_raw &= ~v;
            break;
        case 0x14:
            c->int_ena = v & RX5_INT_MASK;
            break;
        case 0x24:
            if (v & BIT(11)) {
                qemu_log_mask(LOG_UNIMP, "%s: RX channel 5's FIFO pop is not "
                              "modelled\n", TYPE_ESP32P4_H264);
            }
            break;
        case 0x40:
            c->arb = v & 0x1cf;
            break;
        default:
            ok = false;
            break;
        }
    } else {
        switch (o) {
        case 0x00:
            if (!rx) {
                c->conf0 = v & tx_conf0_mask(n);
            } else {
                c->conf0 = v & 0x070011dc;
            }
            if (c->conf0 & CH_RST) {
                chan_reset_link(c);
            }
            break;
        case 0x04:
        case 0x10:
            c->int_raw &= ~v;
            break;
        case 0x08:
            c->int_ena = v & (rx ? RX_INT_MASK : TX_INT_MASK);
            break;
        case 0x18:
            if (v & (rx ? BIT(11) : BIT(10))) {
                qemu_log_mask(LOG_UNIMP, "%s: %s channel %d's FIFO %s is not "
                              "modelled\n", TYPE_ESP32P4_H264, chan_name(rx),
                              n, rx ? "pop" : "push");
            }
            break;
        case 0x1c:
            c->link_conf = v & (rx ? RX_LINK_AUTO_RET : 0);
            chan_link(s, rx, n, v);
            break;
        case 0x20:
            c->link_addr = v;
            break;
        default:
            if (!rx && o == 0x3c) {
                c->arb = v & (n == 1 || n == 2 ? 0x4f : 0x3f);
            } else if (!rx && o == 0x68) {
                c->etm_conf = v & 0xf;
            } else if (!rx && n == 0 && o == 0x44) {
                c->pd_conf = v & 0x70;
            } else if (!rx && n == 0 && o == 0x50) {
                s->mode_enable = v & 1;
            } else if (!rx && n == 0 && o == 0x54) {
                s->mode_yuv = v & 0xffffff;
            } else if (rx && o == 0x40) {
                c->arb = v & (n == 2 || n == 3 ? 0x1cf : 0x1ff);
            } else if (rx && ((n == 0 && o == 0x6c) || (n > 0 && o == 0x48))) {
                c->etm_conf = v & 0xf;
            } else if (rx && n == 0 && o == 0x48) {
                c->pd_conf = v & 0x40;
            } else {
                uint32_t dummy;

                if (dma_chan_read(s, rx, n, o, &dummy)) {
                    qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only "
                                  "%s channel %d register 0x%02" HWADDR_PRIx
                                  "\n", TYPE_ESP32P4_H264, chan_name(rx), n,
                                  o);
                    return;
                }
                ok = false;
            }
            break;
        }
    }
    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved %s channel %d "
                      "register 0x%02" HWADDR_PRIx "\n", TYPE_ESP32P4_H264,
                      chan_name(rx), n, o);
        return;
    }
    h264_update_irq(s);
    h264_kick(s, now);
}

static void h264_dma_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    ESP32P4H264State *s = opaque;
    uint32_t v = value;
    int64_t now = h264_now();
    bool rx;
    int n;
    hwaddr o;

    if (dma_decode(addr, &rx, &n, &o)) {
        dma_chan_write(s, rx, n, o, v, now);
        return;
    }
    switch (addr) {
    case 0xb08:
        s->rst_conf = v & 0x1f;
        break;
    case 0xb0c ... 0xb18:
        s->mem_addr[(addr - 0xb0c) / 4] = v;
        break;
    case 0xb20 ... 0xb2c:
        s->mem_addr[4 + (addr - 0xb20) / 4] = v;
        break;
    case 0xb30:
    case 0xb34:
        s->arb_config[(addr - 0xb30) / 4] = v & 0x1ffff;
        break;
    case 0xb3c:
        s->dma_date = v;
        break;
    case 0xb50:
        s->counter_rst = v & 0xf;
        for (int i = 0; i < 4; i++) {
            if (v & BIT(i)) {
                s->cnt[i] = 0;
            }
        }
        break;
    case 0xb64:
        s->pbyte = v & 0xf;
        if (s->pbyte > 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: ORI_PBYTE %u is invalid\n",
                          TYPE_ESP32P4_H264, s->pbyte);
        }
        break;
    case 0xb68:
        s->ch_dbg_en = v & 0x3f001f;
        break;
    case 0xb6c ... 0xbc0:
        s->dbg_data[(addr - 0xb6c) / 4] = v;
        break;
    case 0xb00:
    case 0xb04:
    case 0xb54 ... 0xb60:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only H264_DMA "
                      "register 0x%03" HWADDR_PRIx "\n", TYPE_ESP32P4_H264,
                      addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved H264_DMA "
                      "register 0x%03" HWADDR_PRIx "\n", TYPE_ESP32P4_H264,
                      addr);
        return;
    }
    h264_kick(s, now);
}

static const MemoryRegionOps h264_dma_ops = {
    .read = h264_dma_read,
    .write = h264_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Clock */

static void h264_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4H264State *s = opaque;
    int64_t now = h264_now();

    if (event == ClockPreUpdate) {
        esp32p4_clkwait_fold(&s->row_wait, now);
        esp32p4_clkwait_fold(&s->tmp_wait, now);
        esp32p4_clkwait_fold(&s->move_wait, now);
        return;
    }
    esp32p4_clkwait_resched(&s->row_wait);
    esp32p4_clkwait_resched(&s->tmp_wait);
    esp32p4_clkwait_resched(&s->move_wait);
}

/* Device */

static void h264_chan_reset(ESP32P4H264Chan *c, bool rx, int n)
{
    memset(c, 0, sizeof(*c));
    c->conf0 = rx ? 0 : TX_EOF_MODE;
    c->link_conf = rx ? RX_LINK_AUTO_RET : 0;
    if (rx) {
        c->arb = n == 2 || n == 3 || n == 5 ? 0x41 : 0x51;
    } else {
        c->arb = n == 1 || n == 2 ? 0x41 : 0x11;
    }
    c->etm_conf = 0x4;
    c->pd_conf = !rx && n == 0 ? 0x20 : 0;
}

static void esp32p4_h264_reset_hold(Object *obj, ResetType type)
{
    ESP32P4H264State *s = ESP32P4_H264(obj);

    esp32p4_clkwait_stop(&s->row_wait);
    esp32p4_clkwait_stop(&s->tmp_wait);
    esp32p4_clkwait_stop(&s->move_wait);
    s->frame_mode = false;
    s->gop_conf = 0;
    for (int q = 0; q < 2; q++) {
        memset(s->seq[q], 0, sizeof(s->seq[q]));
        s->seq[q][S_SYS_CONF] = SYS_CONF_RESET;
        s->ori_conf[q] = ORI_CONF_RESET;
    }
    memset(s->rc_status, 0, sizeof(s->rc_status));
    s->slice_remain = 0;
    s->slice_len = 0;
    s->bs_threshold = BS_THRESHOLD_RESET;
    s->slice_byte[0] = s->slice_byte[1] = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->conf = 0;
    s->mv_merge = 0;
    s->debug_dma_sel = 0;
    s->sys_status = 0;
    s->frame_code_length = 0;
    s->date = ENC_DATE_RESET;
    memset(s->debug_conf, 0, sizeof(s->debug_conf));
    for (int n = 0; n < ESP32P4_H264_TX; n++) {
        h264_chan_reset(&s->tx[n], false, n);
    }
    for (int n = 0; n < ESP32P4_H264_RX; n++) {
        h264_chan_reset(&s->rx[n], true, n);
    }
    s->mode_enable = 0;
    s->mode_yuv = 0;
    s->block_start = 0;
    s->block_row_len = BLOCK_ROW_LEN_RESET;
    s->block_len = BLOCK_LEN_RESET;
    s->rst_conf = 0;
    for (int i = 0; i < 4; i++) {
        s->mem_addr[2 * i] = MEM_START_RESET;
        s->mem_addr[2 * i + 1] = MEM_END_RESET;
    }
    s->arb_config[0] = s->arb_config[1] = 0;
    s->dma_date = DMA_DATE_RESET;
    s->counter_rst = 0;
    s->pbyte = PBYTE_RESET;
    s->ch_dbg_en = 0;
    memset(s->dbg_data, 0, sizeof(s->dbg_data));
    memset(s->cnt, 0, sizeof(s->cnt));
    s->state = ST_IDLE;
    s->stream = 0;
    s->row = 0;
    s->frames = 0;
    s->intra = false;
    s->code_bytes = 0;
    s->bs_overflow = false;
    s->tmp_written = 0;
    s->tmp_ready = false;
    s->tmp_pending = false;
    s->axi_err[0] = s->axi_err[1] = 0;
    s->mv_started = false;
    s->mv_gen = 0;
    s->mv_row = 0;
    s->mv_eager = 0;
    s->next_ref_gen = 0x80000000u;
    s->ref_gen = 0x80000000u;
    rx5_reset(s);
    for (int i = 0; i < 3; i++) {
        s->ring_gen[i] = 0;
    }
}

static void esp32p4_h264_reset_exit(Object *obj, ResetType type)
{
    h264_update_irq(ESP32P4_H264(obj));
}

static void esp32p4_h264_init(Object *obj)
{
    ESP32P4H264State *s = ESP32P4_H264(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem[ESP32P4_H264_MMIO_ENC], obj,
                          &h264_enc_ops, s, "esp32p4.h264",
                          ESP32P4_H264_ENC_SIZE);
    memory_region_init_io(&s->iomem[ESP32P4_H264_MMIO_DMA], obj,
                          &h264_dma_ops, s, "esp32p4.h264-dma",
                          ESP32P4_H264_DMA_SIZE);
    sysbus_init_mmio(sbd, &s->iomem[ESP32P4_H264_MMIO_ENC]);
    sysbus_init_mmio(sbd, &s->iomem[ESP32P4_H264_MMIO_DMA]);
    for (int i = 0; i < ESP32P4_H264_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", h264_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    esp32p4_clkwait_init(&s->row_wait, s->clk, h264_row_cb, s);
    esp32p4_clkwait_init(&s->tmp_wait, s->clk, h264_tmp_cb, s);
    esp32p4_clkwait_init(&s->move_wait, s->clk, h264_move_cb, s);
    h264e_init(&s->enc);
}

static void esp32p4_h264_realize(DeviceState *dev, Error **errp)
{
    ESP32P4H264State *s = ESP32P4_H264(dev);

    if (!s->pms) {
        error_setg(errp, "%s: the 'pms' link is not set", TYPE_ESP32P4_H264);
        return;
    }
    s->as = esp32p4_pms_bus_as(ESP32P4_PMS(s->pms));
}

static void esp32p4_h264_finalize(Object *obj)
{
    ESP32P4H264State *s = ESP32P4_H264(obj);

    h264e_free(&s->enc);
    g_free(s->lines);
}

static const Property esp32p4_h264_properties[] = {
    DEFINE_PROP_LINK("pms", ESP32P4H264State, pms, TYPE_ESP32P4_PMS,
                     Object *),
};

static const VMStateDescription vmstate_h264e_roi = {
    .name = "esp32p4-h264-enc-roi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(en, H264EncRoi),
        VMSTATE_INT32(x, H264EncRoi),
        VMSTATE_INT32(y, H264EncRoi),
        VMSTATE_INT32(w, H264EncRoi),
        VMSTATE_INT32(h, H264EncRoi),
        VMSTATE_UINT8(qp, H264EncRoi),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_h264e_config = {
    .name = "esp32p4-h264-enc-config",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT32(mb_w, H264EncConfig),
        VMSTATE_INT32(mb_h, H264EncConfig),
        VMSTATE_BOOL(intra, H264EncConfig),
        VMSTATE_INT32(qp, H264EncConfig),
        VMSTATE_BOOL(rc_en, H264EncConfig),
        VMSTATE_UINT32(rc_u, H264EncConfig),
        VMSTATE_UINT32(mad_pred, H264EncConfig),
        VMSTATE_INT32(qp_min, H264EncConfig),
        VMSTATE_INT32(qp_max, H264EncConfig),
        VMSTATE_INT32(chroma_qp_offset, H264EncConfig),
        VMSTATE_BOOL(deblock, H264EncConfig),
        VMSTATE_UINT32(intra_cost_offset, H264EncConfig),
        VMSTATE_INT32(l_deci_score, H264EncConfig),
        VMSTATE_INT32(c_deci_score, H264EncConfig),
        VMSTATE_INT32(i16_deci_offset, H264EncConfig),
        VMSTATE_INT32(ic_deci_offset, H264EncConfig),
        VMSTATE_INT32(p16_deci_offset, H264EncConfig),
        VMSTATE_INT32(pc_deci_offset, H264EncConfig),
        VMSTATE_BOOL(roi_en, H264EncConfig),
        VMSTATE_BOOL(roi_delta, H264EncConfig),
        VMSTATE_STRUCT_ARRAY(roi, H264EncConfig, 8, 1, vmstate_h264e_roi,
                             H264EncRoi),
        VMSTATE_UINT8(no_roi_qp, H264EncConfig),
        VMSTATE_BOOL(mv_merge_en, H264EncConfig),
        VMSTATE_INT32(mv_merge_type, H264EncConfig),
        VMSTATE_BOOL(int_mv_out, H264EncConfig),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_h264e_mb = {
    .name = "esp32p4-h264-enc-mb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(type, H264EncMB),
        VMSTATE_UINT8(qp, H264EncMB),
        VMSTATE_INT16_ARRAY(mv, H264EncMB, 2),
        VMSTATE_UINT8_ARRAY(nz, H264EncMB, 16),
        VMSTATE_UINT8_2DARRAY(nzc, H264EncMB, 2, 4),
        VMSTATE_UINT8_ARRAY(i4, H264EncMB, 16),
        VMSTATE_END_OF_LIST()
    },
};

/* The core's buffers come in sized by the stream: let go of its own. */
static int h264e_pre_load(void *opaque)
{
    H264Enc *e = opaque;

    h264e_free(e);
    return 0;
}

#define VMSTATE_H264E_PLANE(f, size) \
    VMSTATE_VBUFFER_ALLOC_UINT32(f, H264Enc, 0, NULL, size)

/*
 * The core's state within a picture: its planes, macroblocks and the bits
 * not yet written out.
 */
static const VMStateDescription vmstate_h264e = {
    .name = "esp32p4-h264-enc",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_load = h264e_pre_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(cfg, H264Enc, 1, vmstate_h264e_config, H264EncConfig),
        VMSTATE_INT32(w, H264Enc),
        VMSTATE_INT32(h, H264Enc),
        VMSTATE_INT32(cw, H264Enc),
        VMSTATE_INT32(ch, H264Enc),
        VMSTATE_UINT32(ly, H264Enc),
        VMSTATE_UINT32(lc, H264Enc),
        VMSTATE_H264E_PLANE(src[0], ly),
        VMSTATE_H264E_PLANE(src[1], lc),
        VMSTATE_H264E_PLANE(src[2], lc),
        VMSTATE_H264E_PLANE(ref[0], ly),
        VMSTATE_H264E_PLANE(ref[1], lc),
        VMSTATE_H264E_PLANE(ref[2], lc),
        VMSTATE_H264E_PLANE(rec[0], ly),
        VMSTATE_H264E_PLANE(rec[1], lc),
        VMSTATE_H264E_PLANE(rec[2], lc),
        VMSTATE_H264E_PLANE(out[0], ly),
        VMSTATE_H264E_PLANE(out[1], lc),
        VMSTATE_H264E_PLANE(out[2], lc),
        VMSTATE_INT32(mb_count, H264Enc),
        VMSTATE_STRUCT_VARRAY_ALLOC(mb, H264Enc, mb_count, 1,
                                    vmstate_h264e_mb, H264EncMB),
        VMSTATE_INT32(alloc_w, H264Enc),
        VMSTATE_INT32(alloc_h, H264Enc),
        VMSTATE_UINT32(rbsp_len, H264Enc),
        VMSTATE_UINT32(rbsp_cap, H264Enc),
        VMSTATE_H264E_PLANE(rbsp, rbsp_cap),
        VMSTATE_UINT32(acc, H264Enc),
        VMSTATE_INT32(acc_bits, H264Enc),
        VMSTATE_INT32(zeros, H264Enc),
        VMSTATE_INT32(qp_prev, H264Enc),
        VMSTATE_INT32(skip_run, H264Enc),
        VMSTATE_UINT64(bits, H264Enc),
        VMSTATE_UINT64(hdr_bits, H264Enc),
        VMSTATE_UINT64(rc_target, H264Enc),
        VMSTATE_UINT32(mad_sum, H264Enc),
        VMSTATE_UINT64(enc_bits, H264Enc),
        VMSTATE_UINT32(qp_sum, H264Enc),
        VMSTATE_UINT32_ARRAY(mvm, H264Enc, H264E_MAX_MB_W),
        VMSTATE_INT32(mvm_n, H264Enc),
        VMSTATE_UINT32(mvm_total, H264Enc),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_h264_chan = {
    .name = "esp32p4-h264-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, ESP32P4H264Chan),
        VMSTATE_UINT32(int_raw, ESP32P4H264Chan),
        VMSTATE_UINT32(int_ena, ESP32P4H264Chan),
        VMSTATE_UINT32(link_conf, ESP32P4H264Chan),
        VMSTATE_UINT32(link_addr, ESP32P4H264Chan),
        VMSTATE_UINT32(arb, ESP32P4H264Chan),
        VMSTATE_UINT32(etm_conf, ESP32P4H264Chan),
        VMSTATE_UINT32(pd_conf, ESP32P4H264Chan),
        VMSTATE_UINT32(eof_addr, ESP32P4H264Chan),
        VMSTATE_UINT32(err_eof_addr, ESP32P4H264Chan),
        VMSTATE_UINT32(dscr, ESP32P4H264Chan),
        VMSTATE_UINT32(dscr_bf0, ESP32P4H264Chan),
        VMSTATE_UINT32(dscr_bf1, ESP32P4H264Chan),
        VMSTATE_INT32(running, ESP32P4H264Chan),
        VMSTATE_BOOL(have_desc, ESP32P4H264Chan),
        VMSTATE_UINT32(desc_addr, ESP32P4H264Chan),
        VMSTATE_UINT32_ARRAY(dw, ESP32P4H264Chan, 4),
        VMSTATE_UINT32(pos, ESP32P4H264Chan),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_h264 = {
    .name = TYPE_ESP32P4_H264,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(frame_mode, ESP32P4H264State),
        VMSTATE_UINT32(gop_conf, ESP32P4H264State),
        VMSTATE_UINT32_2DARRAY(seq, ESP32P4H264State, 2,
                               ESP32P4_H264_SEQ_REGS),
        VMSTATE_UINT32_ARRAY(rc_status, ESP32P4H264State, 3),
        VMSTATE_UINT32(slice_remain, ESP32P4H264State),
        VMSTATE_UINT32(slice_len, ESP32P4H264State),
        VMSTATE_UINT32(bs_threshold, ESP32P4H264State),
        VMSTATE_UINT32_ARRAY(slice_byte, ESP32P4H264State, 2),
        VMSTATE_UINT32(int_raw, ESP32P4H264State),
        VMSTATE_UINT32(int_ena, ESP32P4H264State),
        VMSTATE_UINT32(conf, ESP32P4H264State),
        VMSTATE_UINT32(mv_merge, ESP32P4H264State),
        VMSTATE_UINT32(debug_dma_sel, ESP32P4H264State),
        VMSTATE_UINT32(sys_status, ESP32P4H264State),
        VMSTATE_UINT32(frame_code_length, ESP32P4H264State),
        VMSTATE_UINT32(date, ESP32P4H264State),
        VMSTATE_UINT32_ARRAY(ori_conf, ESP32P4H264State, 2),
        VMSTATE_UINT32_ARRAY(debug_conf, ESP32P4H264State, 7),
        VMSTATE_STRUCT_ARRAY(tx, ESP32P4H264State, ESP32P4_H264_TX, 1,
                             vmstate_esp32p4_h264_chan, ESP32P4H264Chan),
        VMSTATE_STRUCT_ARRAY(rx, ESP32P4H264State, ESP32P4_H264_RX, 1,
                             vmstate_esp32p4_h264_chan, ESP32P4H264Chan),
        VMSTATE_UINT32(mode_enable, ESP32P4H264State),
        VMSTATE_UINT32(mode_yuv, ESP32P4H264State),
        VMSTATE_UINT32(block_start, ESP32P4H264State),
        VMSTATE_UINT32(block_row_len, ESP32P4H264State),
        VMSTATE_UINT32(block_len, ESP32P4H264State),
        VMSTATE_UINT32(rst_conf, ESP32P4H264State),
        VMSTATE_UINT32_ARRAY(mem_addr, ESP32P4H264State, 8),
        VMSTATE_UINT32_ARRAY(arb_config, ESP32P4H264State, 2),
        VMSTATE_UINT32(dma_date, ESP32P4H264State),
        VMSTATE_UINT32(counter_rst, ESP32P4H264State),
        VMSTATE_UINT32(pbyte, ESP32P4H264State),
        VMSTATE_UINT32(ch_dbg_en, ESP32P4H264State),
        VMSTATE_UINT32_ARRAY(dbg_data, ESP32P4H264State, 22),
        VMSTATE_UINT32_ARRAY(cnt, ESP32P4H264State, 4),
        VMSTATE_INT32(state, ESP32P4H264State),
        VMSTATE_INT32(stream, ESP32P4H264State),
        VMSTATE_INT32(row, ESP32P4H264State),
        VMSTATE_UINT32(frames, ESP32P4H264State),
        VMSTATE_BOOL(intra, ESP32P4H264State),
        VMSTATE_UINT64(code_bytes, ESP32P4H264State),
        VMSTATE_BOOL(bs_overflow, ESP32P4H264State),
        VMSTATE_INT32(tmp_written, ESP32P4H264State),
        VMSTATE_BOOL(tmp_ready, ESP32P4H264State),
        VMSTATE_BOOL(tmp_pending, ESP32P4H264State),
        VMSTATE_UINT32_ARRAY(axi_err, ESP32P4H264State, 2),
        VMSTATE_BOOL(mv_started, ESP32P4H264State),
        VMSTATE_ESP32P4_CLKWAIT(row_wait, ESP32P4H264State),
        VMSTATE_ESP32P4_CLKWAIT(tmp_wait, ESP32P4H264State),
        VMSTATE_UINT32(mv_gen, ESP32P4H264State),
        VMSTATE_INT32(mv_row, ESP32P4H264State),
        VMSTATE_INT32(mv_eager, ESP32P4H264State),
        VMSTATE_UINT32(next_ref_gen, ESP32P4H264State),
        VMSTATE_UINT32(ref_gen, ESP32P4H264State),
        VMSTATE_INT32_ARRAY(ring_row, ESP32P4H264State, 3),
        VMSTATE_UINT32_ARRAY(ring_gen, ESP32P4H264State, 3),
        VMSTATE_INT32(ring_next, ESP32P4H264State),
        VMSTATE_ESP32P4_CLKWAIT(move_wait, ESP32P4H264State),
        VMSTATE_STRUCT(enc, ESP32P4H264State, 1, vmstate_h264e, H264Enc),
        VMSTATE_CLOCK(clk, ESP32P4H264State),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_h264_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_h264_realize;
    dc->vmsd = &vmstate_esp32p4_h264;
    device_class_set_props(dc, esp32p4_h264_properties);
    rc->phases.hold = esp32p4_h264_reset_hold;
    rc->phases.exit = esp32p4_h264_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.h264] */
static const TypeInfo esp32p4_h264_info = {
    .name           = TYPE_ESP32P4_H264,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(ESP32P4H264State),
    .instance_init  = esp32p4_h264_init,
    .instance_finalize = esp32p4_h264_finalize,
    .class_init     = esp32p4_h264_class_init,
};

static void esp32p4_h264_register_types(void)
{
    type_register_static(&esp32p4_h264_info);
}
type_init(esp32p4_h264_register_types)
