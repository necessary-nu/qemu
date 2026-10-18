/*
 * ESP32-P4 2D-DMA controller (DMA2D)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 8
 * (2D-DMA Controller) and section 39.5.2 (the PPA's use of it), with the
 * layout, access types and reset values of ESP-IDF's dma2d_reg.h for v3,
 * and its dma2d_ll, dma2d driver, and the PPA, JPEG and async color
 * convert drivers that use it.
 *
 * Four transmit (TX, "out") and three receive (RX, "in") channels. A
 * channel walks a linked list of descriptors in memory, five words each
 * (TRM table 8.4-1): DW0 owner (31), eof (30), 2DEN (29), err_eof (28),
 * hb or length[13:0] (27:14) and vb or size[13:0] (13:0); DW1 pbyte
 * (31:28), HA or length[27:14] (27:14) and VA or size[27:14] (13:0); DW2
 * mod (28), X (27:14) and Y (13:0); DW3 the buffer's address and DW4 the
 * next descriptor's, 0 at the end of the list. Descriptors are 8-byte
 * aligned.
 *
 * What a descriptor moves (TRM 8.4.1): in 1D mode (2DEN clear) length
 * bytes of the buffer for TX, at most size bytes into it for RX; in
 * 2D-MOD0 the hb x vb pixel block at (X, Y) of the HA x VA picture at the
 * buffer, row by row, a row being hb pixels at pbyte's size (0.5, 1, 1.5,
 * 2, 3 or 4 bytes); in 2D-MOD1 the picture from (X, Y) on in blocks of
 * hb x vb, left to right and top to bottom. A 2D-MOD1 block at the
 * picture's right or bottom edge is padded to whole macroblocks
 * (*_MACRO_BLOCK_SIZE: 8 x 8, 16 x 8 or 16 x 16; none pads nothing)
 * (TRM 8.4.3): horizontally with the row's last pixel (the last pixel
 * pair at 1.5 bytes a pixel, the last nibble at 0.5), vertically with the
 * last row; on the receive side what lies beyond the picture is dropped.
 * With OUT_REORDER_EN on TX channel 0 a 2D-MOD1 block goes out macroblock
 * by macroblock, each row by row (TRM 8.4.6.1), rather than row by row
 * across the block.
 *
 * DSCR-PORT (TRM 39.5.2.1, *_DSCR_PORT_EN): the descriptor gives the
 * picture and the image block; the peripheral gives the pixel blocks
 * within it (esp32p4_dma2d_port_tx() and _rx(), include/hw/dma/
 * esp32p4_dma2d.h). A TX pixel block is DSCR_PORT_BLK_H x DSCR_PORT_BLK_V
 * pixels, padded as above where it lies beyond the image block on any
 * side (the left and top with the first pixel and row, which the TRM
 * leaves unsaid). A receive pixel block takes the pixel size the
 * peripheral gives; the descriptor's hb and vb are written back as the
 * extent the blocks covered.
 *
 * Color space conversion (TRM 8.4.7, figure 8.4-5), on every TX channel
 * and RX channel 0, of the stream between memory and the peripheral:
 * stage 1 (*_COLOR_INPUT_SEL) groups the stream into pixels of three
 * bytes, x the most significant, y, z: TX 0 expands RGB565 to RGB888,
 * each 5-bit component c to (c << 3) | (c & 7) and the 6-bit one to
 * (c << 2) | (c & 3) (ESP-IDF's DMA2D_M2M_1D_RGB565_to_RGB888 test, run on
 * silicon, expects the low bits so); TX 1 and RX 0 split a YUV422 pixel
 * pair stored U, Y0, V, Y1 (TRM table 8.4-10) into two pixels (Y, U, V)
 * sharing the pair's U and V; TX 2 and RX 1 take two bytes as y and z
 * (x 0), TX 3 and RX 2 three bytes; 7 passes the stream through
 * untouched. *_SCRAMBLE_SEL_PRE then reorders the pixel's three bytes. Stage
 * 2 (*_COLOR_3B_PROC_EN) computes each output byte as
 * Q = (A x + B y + C z + D + 128) >> 8, clamped to 0..255, with the
 * signed coefficients of COLOR_PARAM_H/M/L0/1: the TRM gives
 * 256 Q = A x + B y + C z + D; ESP-IDF's "async color convert UYVY->BGR24
 * matches reference" test, run on silicon, rounds so. Stage 3
 * (*_COLOR_OUTPUT_SEL) emits the pixel: RGB565 (each component's top
 * bits), YUV422 (a pair as U, Y0, V, Y1 with the first pixel's U and V,
 * as TRM table 8.4-10 labels it), RX YUV420 (the 1.5 bytes a pixel of a
 * pair, U Y0 Y1 on even rows and V Y0 Y1 on odd ones, from the first
 * pixel), or directly (three bytes; two for a two-byte input).
 * DMA2D_IN_SCRAMBLE_SEL_POST reorders RX channel 0's output pixel of two
 * or three bytes, a two-byte one as the low lanes of three. The TRM leaves
 * the chroma subsampling, the two-byte inputs and the output size of
 * "direct" unsaid; the model takes the choices above.
 *
 * The JPEG decoder's MCUs (TRM figure 8.4-4): RX channel 0 in 2D-MOD1
 * with a macroblock size takes its stream as MCUs of 8 x 8 component
 * blocks and assembles each block of hb x vb pixels from them in its
 * reorder RAM: with IN_REORDER_EN the MCUs fill the block macroblock by
 * macroblock, without it they follow one another row by row through the
 * block. A pixel takes its MCU's Y sample and the U and V samples
 * covering it, giving (Y, U, V) for the conversion with
 * IN_COLOR_INPUT_SEL 0 or 2; with no conversion a YUV422 or YUV420 MCU
 * comes out packed as such (U Y0 V Y1; U Y0 Y1 and V Y0 Y1 rows), GRAY as
 * its Y. The decoder names its format (esp32p4_dma2d_rx_mcu_format()).
 *
 * Descriptors are checked as they are read (TRM 8.4.9): a descriptor
 * address not 8-byte aligned or outside the accessible memory
 * (ACCESS_INTR_MEM_START_ADDR to _END_ADDR or the EXTR_MEM range), a
 * buffer outside it, the owner bit clear while *_CHECK_OWNER is set, HA,
 * VA, hb or vb 0 in 2D mode or size 0 in 1D mode, or a bus error on the
 * read, raises DSCR_ERR and leaves the channel idle. A bus error on a
 * data access does not stop the transfer: a read gives zeros and a write
 * is lost, and AXI_ERR counts it. A transaction whose accesses a memory
 * controller's AXI port holds (the flash's or the PSRAM's while it cannot
 * take an access: its clocks stopped, held in reset, AXI_REQ_EN clear;
 * esp32p4_mspi_dma_wait()) waits, and the arbiter that granted it with
 * it, without an error, until the port takes accesses again; it is then
 * made, before the arbiter grants another (chan_stalls()).
 *
 * Every memory access is a 2D-DMA master's on the bus through the access
 * permission managers (include/hw/misc/esp32p4_pms.h): DMA2D_JPEG for a
 * channel serving the JPEG codec, DMA2D_PPA for one serving the PPA, and
 * DMA2D_DUMMY for memory-to-memory transfers and dummy selections, with
 * the AXI ICM's port 10 and the channel's selection as its unit. The DMA
 * reaches memory directly, not through the CPUs' caches.
 *
 * Timing. The channels share the controller's AXI master at its bus clock
 * (DMA2D_SYS_CLK): the TX channels' reads and the RX channels' writes
 * each have an arbiter, which grants one transaction at a time, a
 * descriptor read (five words), a write-back, or a data burst of at most
 * *_MEM_BURST_LENGTH bytes (8 to 128) within one row of the block, not
 * crossing a multiple of the burst length when *_PAGE_BOUND_EN is set.
 * The bus is 64 bits wide (the shortest burst, 8 bytes, is one beat; the
 * TRM does not give the width): a transaction takes a cycle for each 8
 * bytes it touches. Padding takes a cycle for each 8 bytes it makes. The
 * arbiters grant the requesting channel of highest priority
 * (*_ARB_PRIORITY), round robin among equals; with weighted arbitration
 * (*_WEIGHT_EN) a channel spends a token of *_ARB_TOKEN_NUM per
 * transaction and waits without tokens for the time slot
 * (*_ARB_TIMEOUT_NUM bus cycles) to end, or for all tokens to be spent; a
 * channel without requests holds its tokens, and the slot, unless its
 * *_ARB_WEIGHT_OPT_DIS is clear (TRM 8.4.12). The bus's rounds fall on the
 * clock's edges, each at the time it falls due however late the QEMU
 * timer that runs it, in order with the timers the peripherals keep on
 * the controller (include/hw/dma/esp32p4_dma2d.h); register accesses
 * first run what was due before them.
 *
 * The FIFOs are 512 bytes, room for a burst with its conversion and
 * padding; moving bytes between a TX FIFO and the peripheral or the
 * paired RX channel is outside the bus. OUT_DONE, OUT_TOTAL_EOF and (with
 * OUT_EOF_MODE set) OUT_EOF are raised as a descriptor's last byte leaves
 * the TX FIFO; with OUT_EOF_MODE clear, OUT_EOF as it enters. The FIFO
 * status shows the FIFO as the L3 FIFO, its count saturating at 31.
 *
 * Restart (TRM 8.4.8): *LINK_STOP stops the channel once it has finished
 * the descriptor it holds; *LINK_RESTART at the end of the list re-reads
 * the last descriptor's DW4, and continues if it is set.
 *
 * ETM (TRM 8.5): the events are pulsed on "etm-event" at the time of the
 * round or access raising them; a task on "etm-task" starts a channel
 * whose *_ETM_EN is set (*LINK_START is then ignored), or with
 * *_ETM_LOOP_EN lets it go on to its next descriptor. A DSCR_READY task
 * that comes before the channel waits for it is kept, up to
 * *_DSCR_TASK_MAK + 1 of them, one more raising DSCR_TASK_OVF.
 *
 * Not modelled: the FIFO push and pop registers' extra bits beyond a byte
 * and its frame end; the reorder and FIFO RAMs' power controls, kept as
 * written; the 16-byte alignment that accesses to encrypted external
 * memory need (*_ECC_AES_EN, which ESP-IDF never sets); LOOP_TEST.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32p4_dma2d.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_pms.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "migration/vmstate.h"

#define IN  ESP32P4_DMA2D_IN
#define OUT ESP32P4_DMA2D_OUT
#define NTX ESP32P4_DMA2D_TX_CHANNELS
#define NRX ESP32P4_DMA2D_RX_CHANNELS
#define NCH(dir) ((dir) == OUT ? NTX : NRX)

/* The interrupt bits. */
#define OUT_DONE            BIT(0)
#define OUT_EOF             BIT(1)
#define OUT_DSCR_ERR        BIT(2)
#define OUT_TOTAL_EOF       BIT(3)
#define OUT_FIFO_OVF_L1     BIT(4)
#define OUT_DSCR_TASK_OVF   BIT(12)
#define OUT_INT_MASK        0x1fff
#define IN_DONE             BIT(0)
#define IN_SUC_EOF          BIT(1)
#define IN_ERR_EOF          BIT(2)
#define IN_DSCR_ERR         BIT(3)
#define IN_FIFO_UDF_L1      BIT(5)
#define IN_DSCR_EMPTY       BIT(10)
#define IN_DSCR_TASK_OVF    BIT(13)
#define IN_INT_MASK         0x3fff

/* *_CONF0. */
#define OUT_AUTO_WRBACK     BIT(0)
#define OUT_EOF_MODE        BIT(1)
#define IN_MEM_TRANS_EN     BIT(0)
#define CONF0_CHECK_OWNER   BIT(4)
#define CONF0_PORT_EN       BIT(11)
#define CONF0_PAGE_BOUND    BIT(12)
#define CONF0_REORDER_EN    BIT(16)
#define CONF0_RST           BIT(24)
#define CONF0_CMD_DISABLE   BIT(25)
#define CONF0_WEIGHT_OPT_DIS BIT(26)
#define OUT_CONF0_MASK      0x07011fff
#define IN_CONF0_MASK       0x07011ffd
#define OUT_CONF0_RESET     OUT_EOF_MODE

/* *_LINK_CONF. */
#define OUT_LINK_STOP       BIT(20)
#define OUT_LINK_START      BIT(21)
#define OUT_LINK_RESTART    BIT(22)
#define OUT_LINK_PARK       BIT(23)
#define IN_LINK_AUTO_RET    BIT(20)
#define IN_LINK_STOP        BIT(21)
#define IN_LINK_START       BIT(22)
#define IN_LINK_RESTART     BIT(23)
#define IN_LINK_PARK        BIT(24)

/* *_ETM_CONF. */
#define ETM_EN              BIT(0)
#define ETM_LOOP_EN         BIT(1)

/* DMA2D_RST_CONF. */
#define AXIM_RD_RST         BIT(0)
#define AXIM_WR_RST         BIT(1)

/* Descriptor words. */
#define DW0_OWNER           BIT(31)
#define DW0_EOF             BIT(30)
#define DW0_2DEN            BIT(29)
#define DW0_ERR_EOF         BIT(28)
#define DW2_MOD             BIT(28)
#define DESC_BYTES          20

/* The FIFO entries' marks, for what their leaving completes. */
#define F_EOF               BIT(0)  /* the last byte of a frame */
#define F_ERR               BIT(1)  /* ... ended in error */
#define F_DESC_END          BIT(2)  /* a TX descriptor's last byte */
#define F_LIST_END          BIT(3)  /* ... of the last in its list */
#define F_NODATA            BIT(4)  /* a mark carrying no byte */

#define FIFO_SIZE           ESP32P4_DMA2D_FIFO_MAX
/* The room a TX transaction needs: a burst converted, with padding. */
#define TX_ROOM             (128 * 3 / 2 + 32)

#define DATE_RESET          0x2412190
#define MEM_RESET_START     0x30100000
#define MEM_RESET_END       0x8fffffff
#define POP_EMPTY           0x400
#define PORT_BLK_RESET      (18 | 18 << 14)
#define COLOR_CONVERT_RESET (7 << 3)
#define ARB_RESET           0x11
#define ETM_CONF_RESET      (1 << 2)

/* The AXI ICM's port for the 2D-DMA. */
#define DMA2D_ICM_CID       10

static const uint32_t color_param_reset[6] = {
    0x12a, 0xc86d999, 0x1e712a, 0x21e4f30, 0x8112a, 0xbad3000,
};

/* The registers, as the channel blocks and the globals decode to them. */
typedef enum {
    R_NONE,
    R_CONF0, R_INT_RAW, R_INT_ENA, R_INT_ST, R_INT_CLR, R_FIFO_STATUS,
    R_PUSHPOP, R_LINK_CONF, R_LINK_ADDR, R_STATE, R_EOF_DES, R_ERR_EOF_DES,
    R_DSCR, R_DSCR_BF0, R_DSCR_BF1, R_PERI_SEL, R_ARB, R_RO_STATUS,
    R_RO_PD_CONF, R_COLOR_CONVERT, R_SCRAMBLE, R_COLOR_PARAM, R_ETM_CONF,
    R_PORT_BLK,
    R_AXI_ERR, R_RST_CONF, R_INTR_START, R_INTR_END, R_EXTR_START,
    R_EXTR_END, R_OUT_ARB_CONFIG, R_IN_ARB_CONFIG, R_RDN_RESULT,
    R_RDN_ECO_HIGH, R_RDN_ECO_LOW, R_DATE,
} Dma2dReg;

typedef struct Dma2dDecode {
    Dma2dReg reg;
    int dir;
    int ch;
    int idx;
} Dma2dDecode;

static const Dma2dReg out_regs[0x70 / 4] = {
    R_CONF0, R_INT_RAW, R_INT_ENA, R_INT_ST, R_INT_CLR, R_FIFO_STATUS,
    R_PUSHPOP, R_LINK_CONF, R_LINK_ADDR, R_STATE, R_EOF_DES, R_DSCR,
    R_DSCR_BF0, R_DSCR_BF1, R_PERI_SEL, R_ARB, R_RO_STATUS, R_RO_PD_CONF,
    R_COLOR_CONVERT, R_SCRAMBLE, R_COLOR_PARAM, R_COLOR_PARAM,
    R_COLOR_PARAM, R_COLOR_PARAM, R_COLOR_PARAM, R_COLOR_PARAM, R_ETM_CONF,
    R_PORT_BLK,
};

static const Dma2dReg in_regs[0x70 / 4] = {
    R_CONF0, R_INT_RAW, R_INT_ENA, R_INT_ST, R_INT_CLR, R_FIFO_STATUS,
    R_PUSHPOP, R_LINK_CONF, R_LINK_ADDR, R_STATE, R_EOF_DES, R_ERR_EOF_DES,
    R_DSCR, R_DSCR_BF0, R_DSCR_BF1, R_PERI_SEL, R_ARB, R_RO_STATUS,
    R_RO_PD_CONF, R_COLOR_CONVERT, R_SCRAMBLE, R_COLOR_PARAM, R_COLOR_PARAM,
    R_COLOR_PARAM, R_COLOR_PARAM, R_COLOR_PARAM, R_COLOR_PARAM, R_ETM_CONF,
};

static Dma2dDecode dma2d_decode(hwaddr a)
{
    static const Dma2dReg globals[] = {
        R_AXI_ERR, R_RST_CONF, R_INTR_START, R_INTR_END, R_EXTR_START,
        R_EXTR_END, R_OUT_ARB_CONFIG, R_IN_ARB_CONFIG, R_RDN_RESULT,
        R_RDN_ECO_HIGH, R_RDN_ECO_LOW, R_DATE,
    };
    Dma2dDecode d = { R_NONE, 0, 0, 0 };
    hwaddr o = a & 0xff;

    if (a < 0x400 && o < 0x70) {
        d.dir = OUT;
        d.ch = a >> 8;
        d.reg = out_regs[o / 4];
        d.idx = (o - 0x50) / 4;
        /* The reorder RAM's power control is TX channel 0's alone. */
        if (d.reg == R_RO_PD_CONF && d.ch) {
            d.reg = R_NONE;
        }
    } else if (a >= 0x500 && a < 0x800 && o < 0x70) {
        d.dir = IN;
        d.ch = (a - 0x500) >> 8;
        d.reg = in_regs[o / 4];
        d.idx = (o - 0x54) / 4;
        /* RX channel 0 alone converts and reorders. */
        if (d.ch && (d.reg == R_RO_PD_CONF || d.reg == R_COLOR_CONVERT ||
                     d.reg == R_SCRAMBLE || d.reg == R_COLOR_PARAM)) {
            d.reg = R_NONE;
        }
    } else if (a >= 0xa00 && a < 0xa00 + ARRAY_SIZE(globals) * 4) {
        d.reg = globals[(a - 0xa00) / 4];
    }
    return d;
}

/* Channel configuration */

static const char *dir_name(int dir)
{
    return dir == OUT ? "TX" : "RX";
}

static bool chan_etm_en(ESP32P4Dma2dChan *c)
{
    return c->etm_conf & ETM_EN;
}

static bool chan_port(ESP32P4Dma2dChan *c)
{
    return c->conf0 & CONF0_PORT_EN;
}

/* The most bytes a data burst of channel @c moves. */
static uint32_t chan_burst(ESP32P4Dma2dChan *c)
{
    return 8u << MIN(extract32(c->conf0, 6, 3), 4);
}

static uint32_t chan_priority(ESP32P4Dma2dChan *c)
{
    return extract32(c->arb, 4, 4);
}

static uint32_t chan_weight(ESP32P4Dma2dChan *c)
{
    return extract32(c->arb, 0, 4);
}

/* The macroblock size, in pixels; 0 x 0 for none. */
static void chan_mb(ESP32P4Dma2dChan *c, uint32_t *w, uint32_t *h)
{
    static const uint8_t mbw[4] = { 8, 16, 16, 0 };
    static const uint8_t mbh[4] = { 8, 8, 16, 0 };
    unsigned sel = extract32(c->conf0, 9, 2);

    *w = mbw[sel];
    *h = mbh[sel];
}

/*
 * Whether channel @ch's TX output goes to its RX channel: IN_MEM_TRANS_EN
 * on the RX channel of the same number.
 */
static bool chan_m2m(ESP32P4Dma2dState *s, int ch)
{
    return ch < NRX && (s->ch[IN][ch].conf0 & IN_MEM_TRANS_EN);
}

/* Whether selection @sel is a peripheral rather than a dummy. */
static bool periph_real(int dir, uint32_t sel)
{
    return dir == OUT ? sel <= ESP32P4_DMA2D_TX_PPA_BLEND_FG
                      : sel <= ESP32P4_DMA2D_RX_PPA_BLEND;
}

static MemTxAttrs chan_attrs(ESP32P4Dma2dState *s, int dir, int ch)
{
    uint32_t sel = s->ch[dir][ch].peri_sel;
    unsigned master = ESP32P4_DMA_MASTER_DMA2D_DUMMY;

    if (!chan_m2m(s, ch) && periph_real(dir, sel)) {
        master = sel == 0 ? ESP32P4_DMA_MASTER_DMA2D_JPEG
                          : ESP32P4_DMA_MASTER_DMA2D_PPA;
    }
    return esp32p4_dma_attrs(master, ESP32P4_ICM_ID_AXI(DMA2D_ICM_CID, sel));
}

/* Interrupts and events */

static int irq_index(int dir, int ch)
{
    return dir == IN ? ch : NRX + ch;
}

static void dma2d_update_irq(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    if (s->probe) {
        return;
    }
    qemu_set_irq(s->irq[irq_index(dir, ch)], !!(c->int_raw & c->int_ena));
}

static void dma2d_raise(ESP32P4Dma2dState *s, int dir, int ch, uint32_t bits)
{
    s->ch[dir][ch].int_raw |= bits;
    dma2d_update_irq(s, dir, ch);
}

enum {
    EVT_IN_DONE = 0,        /* RX channels 0 and 1 */
    EVT_IN_SUC_EOF = 2,
    EVT_OUT_DONE = 4,       /* TX channels 0 to 2 */
    EVT_OUT_EOF = 7,
    EVT_OUT_TOTAL_EOF = 10,
};

/* Pulse an ETM event, at the time of the round or access raising it. */
static void dma2d_event(ESP32P4Dma2dState *s, int base, int dir, int ch)
{
    int64_t t = esp32_line_time();
    qemu_irq line;

    if (ch >= (dir == IN ? 2 : 3) || s->probe) {
        return;
    }
    line = s->etm_event[base + ch];
    esp32_line_set_at(line, 1, t);
    esp32_line_set_at(line, 0, t);
}

/* FIFOs */

static void fifo_reset(ESP32P4Dma2dFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static uint32_t fifo_room(ESP32P4Dma2dFifo *f)
{
    return FIFO_SIZE - f->num;
}

static void fifo_put(ESP32P4Dma2dFifo *f, uint8_t byte, uint8_t flags,
                     uint32_t desc)
{
    uint32_t i = (f->head + f->num) % FIFO_SIZE;

    assert(f->num < FIFO_SIZE);
    f->data[i] = byte;
    f->flags[i] = flags;
    f->desc[i] = desc;
    f->num++;
}

static void fifo_take(ESP32P4Dma2dFifo *f, uint8_t *byte, uint8_t *flags,
                      uint32_t *desc)
{
    assert(f->num > 0);
    *byte = f->data[f->head];
    *flags = f->flags[f->head];
    *desc = f->desc[f->head];
    f->head = (f->head + 1) % FIFO_SIZE;
    f->num--;
}

/* The flags of the FIFO's last entry, for marks set after it went in. */
static uint8_t *fifo_last_flags(ESP32P4Dma2dFifo *f)
{
    return &f->flags[(f->head + f->num - 1) % FIFO_SIZE];
}

/* Memory */

/* Whether [addr, addr + len) lies in memory the controller may reach. */
static bool dma2d_reachable(ESP32P4Dma2dState *s, uint32_t addr, uint32_t len)
{
    uint64_t last = (uint64_t)addr + MAX(len, 1) - 1;

    return (addr >= s->intr_mem_start && last <= s->intr_mem_end) ||
           (addr >= s->extr_mem_start && last <= s->extr_mem_end);
}

/*
 * An access of channel @ch of direction @dir. While a transaction is
 * worked out (@probe) none is made: an access is only asked about, and
 * marks the transaction held if a memory controller's AXI port holds it;
 * a read gives zeros, which no access after it in the same transaction
 * depends on.
 */
static bool dma2d_bus(ESP32P4Dma2dState *s, int dir, int ch, uint32_t addr,
                      void *buf, uint32_t len, bool write)
{
    MemTxResult r;

    if (!len) {
        return true;
    }
    if (s->probe) {
        if (esp32p4_mspi_dma_wait(s->as, addr, len, chan_attrs(s, dir, ch),
                                  write, &s->bus_wait)) {
            s->probe_held = true;
        }
        if (!write) {
            memset(buf, 0, len);
        }
        return true;
    }
    r = address_space_rw(s->as, addr, chan_attrs(s, dir, ch), buf, len,
                         write);
    if (r == MEMTX_OK) {
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: bus error on a %s at "
                  "0x%08" PRIx32 "\n", TYPE_ESP32P4_DMA2D, dir_name(dir), ch,
                  write ? "write" : "read", addr);
    if (!write) {
        memset(buf, 0, len);
        s->axi_err = deposit32(s->axi_err, 4, 4,
                               extract32(s->axi_err, 4, 4) + 1);
    } else {
        s->axi_err = deposit32(s->axi_err, 8, 4,
                               extract32(s->axi_err, 8, 4) + 1);
    }
    return false;
}

/* The bus cycles a transfer of @n bytes at @addr takes: its 8-byte beats. */
static uint32_t beats(uint32_t addr, uint32_t n)
{
    return n ? DIV_ROUND_UP((addr & 7) + n, 8) : 0;
}

/* The bytes of the next burst from @addr, of at most @left. */
static uint32_t burst_len(ESP32P4Dma2dChan *c, uint32_t addr, uint32_t left)
{
    uint32_t b = chan_burst(c);
    uint32_t n = MIN(left, b);

    if (c->conf0 & CONF0_PAGE_BOUND) {
        n = MIN(n, b - addr % b);
    }
    return n;
}

/* Descriptors */

static void desc_parse_words(const uint32_t *dw, uint32_t addr,
                             ESP32P4Dma2dDesc *d)
{
    uint32_t dw0 = dw[0], dw1 = dw[1], dw2 = dw[2];

    d->addr = addr;
    d->owner = dw0 & DW0_OWNER;
    d->eof = dw0 & DW0_EOF;
    d->en_2d = dw0 & DW0_2DEN;
    d->mod1 = d->en_2d && (dw2 & DW2_MOD);
    d->hb = extract32(dw0, 14, 14);
    d->vb = extract32(dw0, 0, 14);
    d->ha = extract32(dw1, 14, 14);
    d->va = extract32(dw1, 0, 14);
    d->pbyte = extract32(dw1, 28, 4);
    d->x = extract32(dw2, 14, 14);
    d->y = extract32(dw2, 0, 14);
    d->length = d->hb | d->ha << 14;
    d->size = d->vb | d->va << 14;
    d->buf = dw[3];
    d->next = dw[4];
}

static void desc_parse(ESP32P4Dma2dChan *c, uint32_t addr,
                       ESP32P4Dma2dDesc *d)
{
    desc_parse_words(c->dw, addr, d);
}

static ESP32P4Dma2dDesc chan_desc(ESP32P4Dma2dChan *c)
{
    ESP32P4Dma2dDesc d;

    desc_parse(c, c->dscr_bf0, &d);
    return d;
}

/* The nibbles of a pixel of channel @c's memory side. */
static uint32_t desc_nibbles(ESP32P4Dma2dDesc *d)
{
    return esp32p4_dma2d_pbyte_nibbles(d->pbyte);
}

/* Color space conversion */

/* The stage 1 groupings, both directions' INPUT_SEL mapped to them. */
enum { IN_NONE, IN_565, IN_422, IN_2B, IN_3B };
/* The stage 3 outputs, both directions' OUTPUT_SEL mapped to them. */
enum { OUT_565, OUT_422, OUT_420, OUT_DIRECT };

typedef struct CscCfg {
    int in;
    int out;
    bool proc;
    unsigned pre;
    unsigned post;
    int32_t p[3][4];
} CscCfg;

static bool chan_has_csc(int dir, int ch)
{
    return dir == OUT || ch == 0;
}

static void chan_csc(int dir, int ch, ESP32P4Dma2dChan *c, CscCfg *k)
{
    unsigned in = extract32(c->color_convert, 3, 3);
    unsigned out = extract32(c->color_convert, 0, 2);
    static const int tx_in[8] = { IN_565, IN_422, IN_2B, IN_3B, IN_NONE,
                                  IN_NONE, IN_NONE, IN_NONE };
    static const int rx_in[8] = { IN_422, IN_2B, IN_3B, IN_NONE, IN_NONE,
                                  IN_NONE, IN_NONE, IN_NONE };
    static const int tx_out[4] = { OUT_565, OUT_422, OUT_DIRECT,
                                   OUT_DIRECT };
    static const int rx_out[4] = { OUT_565, OUT_DIRECT, OUT_422, OUT_420 };

    memset(k, 0, sizeof(*k));
    if (!chan_has_csc(dir, ch)) {
        k->in = IN_NONE;
        return;
    }
    k->in = dir == OUT ? tx_in[in] : rx_in[in];
    k->out = dir == OUT ? tx_out[out] : rx_out[out];
    k->proc = c->color_convert & BIT(2);
    k->pre = extract32(c->scramble, 0, 3);
    k->post = dir == IN ? extract32(c->scramble, 3, 3) : 0;
    for (int i = 0; i < 3; i++) {
        uint32_t w0 = c->color_param[2 * i], w1 = c->color_param[2 * i + 1];

        k->p[i][0] = sextract32(w0, 0, 10);
        k->p[i][1] = sextract32(w0, 10, 11);
        k->p[i][2] = sextract32(w1, 0, 10);
        k->p[i][3] = sextract32(w1, 10, 18);
    }
}

/* The bytes of a stage 1 input unit. */
static uint32_t csc_unit(int in)
{
    static const uint8_t n[] = { 1, 2, 4, 2, 3 };

    return n[in];
}

static uint8_t expand5(uint32_t c)
{
    return (c << 3) | (c & 7);
}

static uint8_t expand6(uint32_t c)
{
    return (c << 2) | (c & 3);
}

/* Reorder the three lanes of a pixel, lane 2 the most significant. */
static void scramble(uint8_t l[3], unsigned code)
{
    static const uint8_t from[8][3] = {
        { 2, 1, 0 }, { 2, 0, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 0, 2, 1 },
        { 0, 1, 2 }, { 2, 1, 0 }, { 2, 1, 0 },
    };
    uint8_t in[3] = { l[0], l[1], l[2] };

    l[2] = in[from[code][0]];
    l[1] = in[from[code][1]];
    l[0] = in[from[code][2]];
}

/*
 * The output of one pixel (lanes @l, @wide for three bytes of input) into
 * @out: its bytes, or for a subsampled output the pair's once its second
 * pixel comes (*@pend holds the first, bit 24 set). @row: the row's parity
 * for YUV420.
 */
static uint32_t csc_pixel(const CscCfg *k, uint8_t l[3], bool wide,
                          uint32_t *pend, uint32_t row, uint8_t *out)
{
    uint8_t q[3];
    uint32_t n;

    scramble(l, k->pre);
    if (k->proc) {
        for (int i = 0; i < 3; i++) {
            const int32_t *p = k->p[i];
            int32_t v = p[0] * l[2] + p[1] * l[1] + p[2] * l[0] + p[3] + 128;

            q[2 - i] = MIN(MAX(v >> 8, 0), 255);
        }
        memcpy(l, q, 3);
    }
    switch (k->out) {
    case OUT_565: {
        uint32_t v = (l[2] >> 3) << 11 | (l[1] >> 2) << 5 | (l[0] >> 3);

        l[0] = v;
        l[1] = v >> 8;
        l[2] = 0;
        if (k->post) {
            scramble(l, k->post);
        }
        out[0] = l[0];
        out[1] = l[1];
        return 2;
    }
    case OUT_422:
    case OUT_420:
        if (!(*pend & BIT(24))) {
            *pend = BIT(24) | l[2] << 16 | l[1] << 8 | l[0];
            return 0;
        }
        n = 0;
        if (k->out == OUT_422) {
            out[n++] = *pend >> 8;          /* U */
            out[n++] = *pend >> 16;         /* Y0 */
            out[n++] = *pend;               /* V */
        } else {
            out[n++] = row & 1 ? *pend : *pend >> 8;
            out[n++] = *pend >> 16;
        }
        out[n++] = l[2];                    /* Y1 */
        *pend = 0;
        return n;
    default:
        if (k->post) {
            scramble(l, k->post);
        }
        out[0] = l[0];
        out[1] = l[1];
        if (!wide) {
            return 2;
        }
        out[2] = l[2];
        return 3;
    }
}

/*
 * Convert the bytes @in of a stream through channel @c's conversion, the
 * partial pixel kept in its carry, into @out (room for three halves of
 * @n, and 16 bytes): the bytes out.
 */
static uint32_t csc_stream(const CscCfg *k, ESP32P4Dma2dChan *c,
                           const uint8_t *in, uint32_t n, uint8_t *out)
{
    uint32_t unit = csc_unit(k->in), o = 0;

    if (k->in == IN_NONE) {
        memcpy(out, in, n);
        return n;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint8_t l[3], *b = c->carry;

        c->carry[c->ncarry++] = in[i];
        if (c->ncarry < unit) {
            continue;
        }
        c->ncarry = 0;
        switch (k->in) {
        case IN_565: {
            uint32_t v = b[0] | b[1] << 8;

            l[2] = expand5(v >> 11);
            l[1] = expand6(extract32(v, 5, 6));
            l[0] = expand5(v & 0x1f);
            o += csc_pixel(k, l, true, &c->csc_pend, c->csc_row, out + o);
            break;
        }
        case IN_422:
            for (int j = 0; j < 2; j++) {
                l[2] = b[1 + 2 * j];
                l[1] = b[0];
                l[0] = b[2];
                o += csc_pixel(k, l, true, &c->csc_pend, c->csc_row,
                               out + o);
            }
            break;
        case IN_2B:
            l[2] = 0;
            l[1] = b[1];
            l[0] = b[0];
            o += csc_pixel(k, l, false, &c->csc_pend, c->csc_row, out + o);
            break;
        default:
            l[2] = b[2];
            l[1] = b[1];
            l[0] = b[0];
            o += csc_pixel(k, l, true, &c->csc_pend, c->csc_row, out + o);
            break;
        }
    }
    return o;
}

/* Drop a partial pixel at the end of a descriptor's data. */
static void csc_flush(ESP32P4Dma2dState *s, ESP32P4Dma2dChan *c, int dir,
                      int ch)
{
    if ((c->ncarry || (c->csc_pend & BIT(24))) && !s->probe) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: a descriptor ends "
                      "within a pixel the color space conversion needs "
                      "whole\n", TYPE_ESP32P4_DMA2D, dir_name(dir), ch);
    }
    c->ncarry = 0;
    c->csc_pend = 0;
}

/* The link FSM */

static void chan_halt(ESP32P4Dma2dChan *c)
{
    c->active = false;
    c->has_desc = false;
    c->end = false;
    c->restart = false;
    c->stop = false;
    c->wait_ready = false;
}

/* The position within a descriptor, at its start. */
static void chan_cursor_reset(ESP32P4Dma2dChan *c)
{
    c->pos = 0;
    c->blk_x = 0;
    c->blk_y = 0;
    c->mb = 0;
    c->row = 0;
    c->phase = 0;
    c->rpos = 0;
    c->port_w = 0;
    c->port_h = 0;
    c->csc_row = 0;
}

/*
 * The stream through the channel, at its start: it goes on across the
 * descriptors of a list, a pixel or a staged byte carried into the next.
 */
static void chan_stream_reset(ESP32P4Dma2dChan *c)
{
    c->ncarry = 0;
    c->csc_pend = 0;
    c->nstage = 0;
    c->stage_flags = 0;
}

static void chan_reset_fsm(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    chan_halt(c);
    chan_cursor_reset(c);
    chan_stream_reset(c);
    fifo_reset(&c->fifo);
    c->dq_head = 0;
    c->dq_num = 0;
    c->nport = 0;
    c->tasks = 0;
    if (dir == IN && ch == 0) {
        s->ro_num = 0;
        s->ro_out_len = 0;
    }
}

static void dma2d_kick(ESP32P4Dma2dState *s);

static void notify_periph(ESP32P4Dma2dState *s, int dir, int ch)
{
    uint32_t sel = s->ch[dir][ch].peri_sel;

    if ((dir == IN || !chan_m2m(s, ch)) && sel < ESP32P4_DMA2D_PERIPHS) {
        s->notify |= BIT(sel + (dir == IN ? 8 : 0));
    }
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void chan_start(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    if (c->conf0 & CONF0_RST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d started while held "
                      "in reset\n", TYPE_ESP32P4_DMA2D, dir_name(dir), ch);
        return;
    }
    c->active = true;
    c->has_desc = false;
    c->end = false;
    c->restart = false;
    c->stop = false;
    c->wait_ready = false;
    c->empty_raised = false;
    c->desc_next = c->link_addr;
    c->nport = 0;
    chan_cursor_reset(c);
    chan_stream_reset(c);
    if (dir == IN && ch == 0) {
        s->ro_num = 0;
        s->ro_out_len = 0;
    }
    /* A receive channel takes bytes from now on. */
    if (dir == IN && !chan_m2m(s, ch)) {
        notify_periph(s, IN, ch);
    }
    dma2d_kick(s);
}

/* Re-read the DW4 of the list's last descriptor; continue if it is set. */
static void chan_reread_next(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    uint32_t next = 0;

    if (!c->prev_desc) {
        c->end = true;
        return;
    }
    dma2d_bus(s, dir, ch, c->prev_desc + 16, &next, 4, false);
    next = le32_to_cpu(next);
    if (next) {
        c->desc_next = next;
        c->end = false;
        c->empty_raised = false;
    } else {
        c->end = true;
    }
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void chan_restart(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    if (c->conf0 & CONF0_RST) {
        return;
    }
    if (!c->active) {
        c->active = true;
        c->has_desc = false;
        chan_cursor_reset(c);
        if (c->prev_desc) {
            chan_reread_next(s, dir, ch);
        } else {
            c->desc_next = c->link_addr;
            c->end = false;
        }
    } else if (c->end) {
        chan_reread_next(s, dir, ch);
    } else {
        c->restart = true;
    }
    dma2d_kick(s);
}

static void chan_desc_error(ESP32P4Dma2dState *s, int dir, int ch,
                            uint32_t addr, const char *why)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    if (!s->probe) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: the descriptor at "
                      "0x%08" PRIx32 " %s\n", TYPE_ESP32P4_DMA2D,
                      dir_name(dir), ch, addr, why);
    }
    chan_halt(c);
    dma2d_raise(s, dir, ch, dir == IN ? IN_DSCR_ERR : OUT_DSCR_ERR);
}

/*
 * Read the descriptor at the channel's link pointer and check it: 0 on a
 * descriptor error, which leaves the channel idle, otherwise the bus
 * cycles. An RX descriptor in DSCR-PORT mode has no pixel size of its
 * own: the peripheral gives each block's (ESP-IDF leaves the field as it
 * finds it).
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t chan_fetch(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    uint32_t addr = c->desc_next;
    uint32_t w[5];
    ESP32P4Dma2dDesc d;
    const char *why = NULL;

    if (addr & 7) {
        why = "is not 8-byte aligned";
    } else if (!dma2d_reachable(s, addr, DESC_BYTES)) {
        why = "is outside the accessible memory";
    } else if (!dma2d_bus(s, dir, ch, addr, w, sizeof(w), false)) {
        why = "could not be read";
    } else {
        for (int i = 0; i < 5; i++) {
            c->dw[i] = le32_to_cpu(w[i]);
        }
        desc_parse(c, addr, &d);
        if ((c->conf0 & CONF0_CHECK_OWNER) && !d.owner) {
            why = "is owned by the CPU";
        } else if (d.en_2d && (!d.ha || !d.va || !d.hb || !d.vb)) {
            why = "has HA, VA, hb or vb 0";
        } else if (!d.en_2d && !d.size) {
            why = "has size 0";
        } else if (!dma2d_reachable(s, d.buf, 1)) {
            why = "has a buffer outside the accessible memory";
        } else if (d.en_2d && !desc_nibbles(&d) &&
                   !(dir == IN && chan_port(c))) {
            why = "has a reserved pbyte";
        }
        if (!why && !d.en_2d && (c->dw[2] & DW2_MOD) && !s->probe) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: the 1D "
                          "descriptor at 0x%08" PRIx32 " has mod set\n",
                          TYPE_ESP32P4_DMA2D, dir_name(dir), ch, addr);
        }
        if (!why && d.en_2d && !(dir == IN && chan_port(c)) &&
            (desc_nibbles(&d) & 1) && ((d.x | d.hb | d.ha) & 1) &&
            !s->probe) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: the "
                          "descriptor at 0x%08" PRIx32 " has an odd X, hb "
                          "or HA at %u nibbles a pixel\n",
                          TYPE_ESP32P4_DMA2D, dir_name(dir), ch, addr,
                          desc_nibbles(&d));
        }
    }
    c->dscr_bf1 = c->dscr_bf0;
    c->dscr_bf0 = addr;
    c->dscr = why ? 0 : c->dw[4];
    if (why) {
        chan_desc_error(s, dir, ch, addr, why);
        return 0;
    }
    c->has_desc = true;
    chan_cursor_reset(c);
    if (dir == OUT) {
        uint32_t *q = c->dq[(c->dq_head + c->dq_num) %
                            ESP32P4_DMA2D_DESC_QUEUE];

        assert(c->dq_num < ESP32P4_DMA2D_DESC_QUEUE);
        q[0] = addr;
        memcpy(&q[1], c->dw, sizeof(c->dw));
        c->dq_num++;
    }
    if (!chan_m2m(s, ch) || dir == IN) {
        notify_periph(s, dir, ch);
    }
    return beats(addr, DESC_BYTES);
}

/*
 * The descriptor just finished: the list goes on from it, ends there, or
 * (after a RESTART during the run) goes on from its re-read DW4; a STOP
 * taken during it halts the channel, and with ETM_LOOP_EN the channel
 * waits for a DSCR_READY task before it goes on.
 */
static uint32_t chan_advance(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    uint32_t cycles = 0;

    c->prev_desc = c->dscr_bf0;
    c->has_desc = false;
    c->desc_next = c->dw[4];
    if (!c->desc_next) {
        if (c->restart) {
            c->restart = false;
            chan_reread_next(s, dir, ch);
            cycles = 1;
        } else {
            c->end = true;
        }
    }
    if (c->stop) {
        chan_halt(c);
        return cycles;
    }
    if (!c->end && (c->etm_conf & ETM_LOOP_EN)) {
        if (c->tasks) {
            c->tasks--;
        } else {
            c->wait_ready = true;
        }
    }
    return cycles;
}

/* The pixel runs of a 2D descriptor */

/*
 * One row of pixels of a 2D transfer: @lpad pixels before the real ones
 * and @rpad after (TX: padding, the edge unit replicated; RX: dropped),
 * and @real bytes in memory from @addr.
 */
typedef struct Dma2dRun {
    uint32_t addr;
    uint32_t real;
    uint32_t lpad;
    uint32_t rpad;
    uint32_t ledge;     /* the address of the unit padding the left */
    uint32_t redge;     /* ... the right */
    uint32_t nib;
} Dma2dRun;

/* The bytes of the unit padding replicates: a pixel, or a pair at 1.5. */
static uint32_t pad_unit_bytes(uint32_t nib)
{
    return nib == 3 ? 3 : MAX(nib / 2, 1);
}

static uint32_t pad_bytes(uint32_t nib, uint32_t px)
{
    return px * nib / 2;
}

/*
 * The row run of @rx0..@rx1 (pixels, clamped) of row @row of the picture
 * of descriptor @d, @x0 the first pixel asked for and @w the count.
 */
static void make_run(ESP32P4Dma2dDesc *d, uint32_t nib, int64_t row, int64_t x0,
                     int64_t w, int64_t cx0, int64_t cx1, Dma2dRun *r)
{
    uint64_t stride = (uint64_t)d->ha * nib / 2;
    int64_t rx0 = MAX(x0, cx0), rx1 = MIN(x0 + w, cx1);
    uint32_t unit = pad_unit_bytes(nib);
    uint32_t base = d->buf + row * stride;

    r->nib = nib;
    r->lpad = MAX(cx0 - x0, 0);
    r->rpad = MAX(x0 + w - cx1, 0);
    if (r->lpad > w) {
        r->lpad = w;
    }
    if (r->rpad > w) {
        r->rpad = w;
    }
    if (rx1 > rx0) {
        r->addr = base + rx0 * nib / 2;
        r->real = (rx1 - rx0) * nib / 2;
        r->ledge = r->addr;
        r->redge = r->addr + r->real - unit;
    } else {
        /* Wholly outside: every pixel replicates the nearer edge. */
        int64_t ex = x0 + w <= cx0 ? cx0 : cx1 - (nib == 3 ? 2 : 1);

        r->addr = base + ex * nib / 2;
        r->real = 0;
        r->ledge = r->redge = r->addr;
        if (x0 + w <= cx0) {
            r->lpad = w;
            r->rpad = 0;
        } else {
            r->lpad = 0;
            r->rpad = w;
        }
    }
}

/*
 * The current row run of channel @c in 2D mode: false once the
 * descriptor's rows are done, or while a DSCR-PORT channel waits for its
 * next block.
 */
static bool chan_run(ESP32P4Dma2dState *s, int dir, int ch, Dma2dRun *r)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    ESP32P4Dma2dDesc d = chan_desc(c);
    uint32_t nib = desc_nibbles(&d);
    int64_t cx0 = INT64_MIN / 4, cx1 = INT64_MAX / 4;
    int64_t cy0 = INT64_MIN / 4, cy1 = INT64_MAX / 4;
    int64_t row, x0, w;

    if (chan_port(c)) {
        ESP32P4Dma2dPortBlock *b = &c->port[0];

        if (!c->nport) {
            return false;
        }
        if (dir == OUT) {
            uint32_t bw, bh;

            esp32p4_dma2d_port_block(s, c->peri_sel, &bw, &bh);
            if (c->row >= bh) {
                return false;
            }
            cx0 = d.x;
            cx1 = d.x + d.hb;
            cy0 = d.y;
            cy1 = d.y + d.vb;
            row = (int64_t)d.y + b->y + c->row;
            x0 = (int64_t)d.x + b->x;
            w = bw;
        } else {
            if (c->row >= b->h) {
                return false;
            }
            nib = b->nibbles;
            row = (int64_t)d.y + b->y + c->row;
            x0 = (int64_t)d.x + b->x;
            w = b->w;
        }
    } else if (!d.mod1) {
        if (c->row >= d.vb) {
            return false;
        }
        row = (int64_t)d.y + c->row;
        x0 = d.x;
        w = d.hb;
    } else {
        uint32_t bx = d.x + c->blk_x * d.hb, by = d.y + c->blk_y * d.vb;
        uint32_t rw, rh, pw, ph, mbw, mbh;

        if (by >= d.va || bx >= d.ha) {
            return false;
        }
        chan_mb(c, &mbw, &mbh);
        rw = MIN(d.hb, d.ha - bx);
        rh = MIN(d.vb, d.va - by);
        pw = mbw ? ROUND_UP(rw, mbw) : rw;
        ph = mbh ? ROUND_UP(rh, mbh) : rh;
        cx0 = cy0 = 0;
        cx1 = d.ha;
        cy1 = d.va;
        if (dir == OUT && ch == 0 && (c->conf0 & CONF0_REORDER_EN) && mbw) {
            uint32_t nmbx = pw / mbw;

            if (c->mb >= nmbx * (ph / mbh) || c->row >= mbh) {
                return false;
            }
            row = by + (c->mb / nmbx) * mbh + c->row;
            x0 = bx + (c->mb % nmbx) * mbw;
            w = mbw;
        } else {
            if (c->row >= ph) {
                return false;
            }
            row = by + c->row;
            x0 = bx;
            w = pw;
        }
    }
    make_run(&d, nib, MIN(MAX(row, cy0), cy1 - 1), x0, w, cx0, cx1, r);
    if (dir == IN && (row < cy0 || row >= cy1)) {
        /* A row beyond the picture is dropped whole. */
        r->lpad = w;
        r->real = 0;
        r->rpad = 0;
    }
    return true;
}

/*
 * The row just done: the next one of the macroblock, block or picture.
 * True when that was the descriptor's last (in DSCR-PORT, the block's
 * last row, the block being the last).
 */
static bool chan_next_row(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    ESP32P4Dma2dDesc d = chan_desc(c);
    Dma2dRun r;

    c->row++;
    c->csc_row++;
    c->phase = 0;
    c->rpos = 0;
    if (chan_port(c)) {
        ESP32P4Dma2dPortBlock b = c->port[0];
        uint32_t h = b.h;

        if (dir == OUT) {
            uint32_t bw;

            esp32p4_dma2d_port_block(s, c->peri_sel, &bw, &h);
        } else {
            c->port_w = MAX(c->port_w, b.x + b.w);
            c->port_h = MAX(c->port_h, b.y + b.h);
        }
        if (c->row < h) {
            return false;
        }
        c->row = 0;
        c->csc_row = 0;
        c->nport--;
        memmove(&c->port[0], &c->port[1], sizeof(c->port[0]) * c->nport);
        if (b.last) {
            return true;
        }
        if (dir == OUT) {
            notify_periph(s, OUT, ch);
        }
        return false;
    }
    if (!d.mod1) {
        return c->row >= d.vb;
    }
    if (chan_run(s, dir, ch, &r)) {
        return false;
    }
    /* The macroblock, then the block, is done. */
    if (dir == OUT && ch == 0 && (c->conf0 & CONF0_REORDER_EN)) {
        uint32_t mbw, mbh;

        chan_mb(c, &mbw, &mbh);
        if (mbw) {
            c->mb++;
            c->row = 0;
            if (chan_run(s, dir, ch, &r)) {
                return false;
            }
        }
    }
    c->mb = 0;
    c->row = 0;
    c->csc_row = 0;
    c->blk_x++;
    if (d.x + c->blk_x * d.hb >= d.ha) {
        c->blk_x = 0;
        c->blk_y++;
    }
    return !chan_run(s, dir, ch, &r);
}

/* The bytes of part @phase of row run @r: left padding, data, right. */
static uint32_t run_part(Dma2dRun *r, uint32_t phase)
{
    return phase == 0 ? pad_bytes(r->nib, r->lpad)
                      : phase == 1 ? r->real : pad_bytes(r->nib, r->rpad);
}

enum { RUN_MORE, RUN_DONE, RUN_WAIT };

/*
 * Move past the parts of the current row that are done, and the rows: the
 * descriptor's rows are all done (RUN_DONE), a DSCR-PORT channel waits
 * for its next block (RUN_WAIT), or *@r is the row with bytes still to
 * move in its part c->phase (RUN_MORE).
 */
static int run_settle(ESP32P4Dma2dState *s, int dir, int ch, Dma2dRun *r)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    for (;;) {
        if (c->rpos < run_part(r, c->phase)) {
            return RUN_MORE;
        }
        c->rpos = 0;
        if (c->phase < 2) {
            c->phase++;
            continue;
        }
        if (chan_next_row(s, dir, ch)) {
            return RUN_DONE;
        }
        if (!chan_run(s, dir, ch, r)) {
            return RUN_WAIT;
        }
    }
}

/* The most bytes of padding a step makes: whole units, at most 128. */
static uint32_t pad_chunk(Dma2dRun *r)
{
    uint32_t unit = r->nib == 3 ? 3 : MAX(r->nib / 2, 1);

    return 128 / unit * unit;
}

/* TX: memory to FIFO */

/* Put bytes of channel @ch's stream, converted, into its FIFO. */
static void tx_emit(ESP32P4Dma2dState *s, int ch, const uint8_t *buf,
                    uint32_t n)
{
    ESP32P4Dma2dChan *c = &s->ch[OUT][ch];
    uint8_t out[TX_ROOM * 2];
    CscCfg k;
    uint32_t o;

    while (n) {
        uint32_t step = MIN(n, 128);

        chan_csc(OUT, ch, c, &k);
        o = csc_stream(&k, c, buf, step, out);
        assert(o <= fifo_room(&c->fifo));
        for (uint32_t i = 0; i < o; i++) {
            fifo_put(&c->fifo, out[i], 0, c->dscr_bf0);
        }
        buf += step;
        n -= step;
    }
}

/*
 * Padding: @total bytes (whole units) replicating the unit at @edge
 * (@left: its first pixel, otherwise its last).
 */
static void tx_pad(ESP32P4Dma2dState *s, int ch, Dma2dRun *r, uint32_t edge,
                   bool left, uint32_t total)
{
    uint32_t unit = pad_unit_bytes(r->nib);
    uint8_t u[4], buf[64];
    uint32_t n = 0;

    dma2d_bus(s, OUT, ch, edge, u, unit, false);
    if (r->nib == 1) {
        uint8_t v = left ? u[0] & 0xf : u[0] >> 4;

        u[0] = v | v << 4;
    }
    for (uint32_t i = 0; i < total; i++) {
        buf[n++] = u[i % unit];
        if (n == sizeof(buf)) {
            tx_emit(s, ch, buf, n);
            n = 0;
        }
    }
    tx_emit(s, ch, buf, n);
}

/* The descriptor's data has all gone into the FIFO. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t tx_desc_done(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[OUT][ch];
    bool eof = c->dw[0] & DW0_EOF;
    ESP32P4Dma2dFifo *f = &c->fifo;
    uint32_t last = (f->head + f->num - 1) % FIFO_SIZE;
    uint32_t cycles = 0;
    uint8_t *flags;

    if (eof) {
        csc_flush(s, c, OUT, ch);
    }
    /* A descriptor whose last byte has left, or that had none, a mark. */
    if (!f->num || f->desc[last] != c->dscr_bf0 ||
        (f->flags[last] & F_DESC_END)) {
        fifo_put(f, 0, F_NODATA, c->dscr_bf0);
    }
    flags = fifo_last_flags(f);
    *flags |= F_DESC_END | (eof ? F_EOF : 0);
    if (eof && !(c->conf0 & OUT_EOF_MODE)) {
        c->eof_des_addr = c->dscr_bf0;
        dma2d_raise(s, OUT, ch, OUT_EOF);
        dma2d_event(s, EVT_OUT_EOF, OUT, ch);
    }
    if (c->conf0 & OUT_AUTO_WRBACK) {
        uint32_t dw0 = cpu_to_le32(c->dw[0] & ~DW0_OWNER);

        dma2d_bus(s, OUT, ch, c->dscr_bf0, &dw0, 4, true);
        cycles++;
    }
    cycles += chan_advance(s, OUT, ch);
    /* The list's last byte, unless a RESTART has carried it on. */
    if (c->end) {
        *flags |= F_LIST_END;
    }
    return cycles;
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t tx_service(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[OUT][ch];
    ESP32P4Dma2dDesc d;
    uint8_t buf[128];
    uint32_t n, cycles = 0, was = c->fifo.num;
    Dma2dRun r;

    if (!c->has_desc) {
        return chan_fetch(s, OUT, ch);
    }
    d = chan_desc(c);
    if (!d.en_2d) {
        n = burst_len(c, d.buf + c->pos, d.length - c->pos);
        if (n) {
            dma2d_bus(s, OUT, ch, d.buf + c->pos, buf, n, false);
            tx_emit(s, ch, buf, n);
            cycles = beats(d.buf + c->pos, n);
            c->pos += n;
        }
        if (c->pos == d.length) {
            cycles += tx_desc_done(s, ch);
        }
    } else {
        int st = chan_run(s, OUT, ch, &r) ? run_settle(s, OUT, ch, &r)
                                          : chan_port(c) ? RUN_WAIT
                                                         : RUN_DONE;

        if (st == RUN_MORE) {
            uint32_t left = run_part(&r, c->phase) - c->rpos;

            if (c->phase == 1) {
                n = burst_len(c, r.addr + c->rpos, left);
                dma2d_bus(s, OUT, ch, r.addr + c->rpos, buf, n, false);
                tx_emit(s, ch, buf, n);
                cycles = beats(r.addr + c->rpos, n);
            } else {
                n = MIN(left, pad_chunk(&r));
                tx_pad(s, ch, &r, c->phase ? r.redge : r.ledge, !c->phase,
                       n);
                cycles = DIV_ROUND_UP(n, 8);
            }
            c->rpos += n;
            st = run_settle(s, OUT, ch, &r);
        }
        if (st == RUN_DONE) {
            cycles += tx_desc_done(s, ch);
        }
    }
    if (!was && c->fifo.num && !chan_m2m(s, ch)) {
        notify_periph(s, OUT, ch);
    }
    return MAX(cycles, 1);
}

/* The events of a byte, or a mark, leaving a TX FIFO. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void tx_left(ESP32P4Dma2dState *s, int ch, uint8_t flags,
                    uint32_t desc)
{
    ESP32P4Dma2dChan *c = &s->ch[OUT][ch];
    uint32_t bits = 0;

    if (!(flags & F_DESC_END)) {
        return;
    }
    /* The descriptor's last byte has left: the peripheral is past it. */
    if (c->dq_num) {
        c->dq_head = (c->dq_head + 1) % ESP32P4_DMA2D_DESC_QUEUE;
        c->dq_num--;
    }
    if (flags & F_EOF) {
        if (c->conf0 & OUT_EOF_MODE) {
            c->eof_des_addr = desc;
            bits |= OUT_EOF;
            dma2d_event(s, EVT_OUT_EOF, OUT, ch);
        }
        if (flags & F_LIST_END) {
            bits |= OUT_TOTAL_EOF;
            dma2d_event(s, EVT_OUT_TOTAL_EOF, OUT, ch);
        }
    }
    bits |= OUT_DONE;
    dma2d_event(s, EVT_OUT_DONE, OUT, ch);
    dma2d_raise(s, OUT, ch, bits);
}

/* RX: FIFO to memory */

/* Whether RX channel @ch takes bytes into its FIFO. */
static bool rx_accepts(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][ch];

    return c->active && !(c->conf0 & (CONF0_RST | CONF0_CMD_DISABLE));
}

/* Bytes reached an RX channel whose list has ended. */
static void rx_check_empty(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][ch];

    if (c->active && c->end && c->fifo.num && !c->empty_raised) {
        c->empty_raised = true;
        dma2d_raise(s, IN, ch, IN_DSCR_EMPTY);
    }
}

/* Close the RX descriptor: write it back and raise its interrupts. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t rx_close(ESP32P4Dma2dState *s, int ch, bool eof, bool err)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][ch];
    uint32_t dw[2], bits = IN_DONE, n = 1;
    uint32_t addr = c->dscr_bf0;

    dw[0] = c->dw[0] & ~(DW0_OWNER | DW0_EOF | DW0_ERR_EOF);
    dw[1] = c->dw[1];
    if (c->link & IN_LINK_AUTO_RET) {
        dw[0] |= DW0_OWNER;
    }
    dw[0] |= (eof ? DW0_EOF : 0) | (err ? DW0_ERR_EOF : 0);
    if (!(c->dw[0] & DW0_2DEN)) {
        /* The length received, its low bits in DW0 and high in DW1. */
        dw[0] = deposit32(dw[0], 14, 14, c->pos);
        dw[1] = deposit32(dw[1], 14, 14, c->pos >> 14);
        n = 2;
    } else if (chan_port(c)) {
        dw[0] = deposit32(dw[0], 14, 14, c->port_w);
        dw[0] = deposit32(dw[0], 0, 14, c->port_h);
    }
    dw[0] = cpu_to_le32(dw[0]);
    dw[1] = cpu_to_le32(dw[1]);
    dma2d_bus(s, IN, ch, addr, dw, 4 * n, true);
    if (eof || err) {
        csc_flush(s, c, IN, ch);
    }
    dma2d_event(s, EVT_IN_DONE, IN, ch);
    if (eof || err) {
        if (err) {
            c->err_eof_des_addr = addr;
            bits |= IN_ERR_EOF;
        } else {
            c->eof_des_addr = addr;
            bits |= IN_SUC_EOF;
            dma2d_event(s, EVT_IN_SUC_EOF, IN, ch);
        }
    }
    dma2d_raise(s, IN, ch, bits);
    return n + chan_advance(s, IN, ch);
}

/*
 * Convert bytes from the FIFO into the staging buffer until it holds
 * @want or the FIFO is empty or a frame's end has been taken.
 */
static void rx_stage(ESP32P4Dma2dState *s, int ch, uint32_t want)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][ch];
    CscCfg k;
    uint8_t byte, flags;
    uint32_t desc;

    chan_csc(IN, ch, c, &k);
    if (chan_port(c)) {
        k.in = IN_NONE;
    }
    /* A pixel converted takes at most six bytes. */
    want = MIN(want, sizeof(c->stage) - 8);
    while (c->nstage < want && c->fifo.num && !c->stage_flags) {
        fifo_take(&c->fifo, &byte, &flags, &desc);
        if (!(flags & F_NODATA)) {
            c->nstage += csc_stream(&k, c, &byte, 1, c->stage + c->nstage);
        }
        if (flags & (F_EOF | F_ERR)) {
            c->stage_flags = flags & (F_EOF | F_ERR);
        }
    }
}

static void rx_unstage(ESP32P4Dma2dChan *c, uint32_t n)
{
    c->nstage -= n;
    memmove(c->stage, c->stage + n, c->nstage);
}

/*
 * The JPEG decoder's MCUs: the bytes one takes, and its size in pixels,
 * by format.
 */
static void mcu_geometry(int fmt, uint32_t *bytes, uint32_t *w, uint32_t *h)
{
    static const uint16_t b[4] = { 64, 192, 256, 384 };
    static const uint8_t mw[4] = { 8, 8, 16, 16 };
    static const uint8_t mh[4] = { 8, 8, 8, 16 };

    *bytes = b[fmt];
    *w = mw[fmt];
    *h = mh[fmt];
}

static int rx_mcu_format(ESP32P4Dma2dState *s, ESP32P4Dma2dChan *c)
{
    uint32_t mbw, mbh;

    if (s->mcu_format != ESP32P4_DMA2D_MCU_AUTO) {
        return s->mcu_format;
    }
    chan_mb(c, &mbw, &mbh);
    return mbw == 8 ? ESP32P4_DMA2D_MCU_YUV444
                    : mbh == 8 ? ESP32P4_DMA2D_MCU_YUV422
                               : ESP32P4_DMA2D_MCU_YUV420;
}

/* Whether RX channel @ch takes its stream as MCUs into its reorder RAM. */
static bool rx_mcu_mode(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][ch];
    uint32_t mbw, mbh;

    chan_mb(c, &mbw, &mbh);
    return ch == 0 && c->has_desc && (c->dw[0] & DW0_2DEN) &&
           (c->dw[2] & DW2_MOD) && mbw && !chan_port(c);
}

/* The current block of an MCU-mode RX channel: its padded size. */
static bool rx_mcu_block(ESP32P4Dma2dChan *c, uint32_t *pw, uint32_t *ph)
{
    ESP32P4Dma2dDesc d = chan_desc(c);
    uint32_t bx = d.x + c->blk_x * d.hb, by = d.y + c->blk_y * d.vb;
    uint32_t mbw, mbh;

    if (by >= d.va || bx >= d.ha) {
        return false;
    }
    chan_mb(c, &mbw, &mbh);
    *pw = ROUND_UP(MIN(d.hb, d.ha - bx), mbw);
    *ph = ROUND_UP(MIN(d.vb, d.va - by), mbh);
    return true;
}

/*
 * Assemble the block in the reorder RAM into pixels and convert them, in
 * the block's raster order, into the RAM's output half.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void rx_mcu_assemble(ESP32P4Dma2dState *s, uint32_t pw, uint32_t ph)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][0];
    int fmt = rx_mcu_format(s, c);
    bool reorder = c->conf0 & CONF0_REORDER_EN;
    uint32_t mcu_bytes, mw, mh, o = 0;
    CscCfg k;
    bool native;

    mcu_geometry(fmt, &mcu_bytes, &mw, &mh);
    chan_csc(IN, 0, c, &k);
    native = k.in == IN_NONE || k.in == IN_2B;
    c->csc_pend = 0;
    for (uint32_t py = 0; py < ph; py++) {
        for (uint32_t px = 0; px < pw; px++) {
            uint32_t mcu, lx, ly, at;
            const uint8_t *m;
            uint8_t y, u, v, l[3];

            if (reorder) {
                mcu = (py / mh) * (pw / mw) + px / mw;
                lx = px % mw;
                ly = py % mh;
            } else {
                uint32_t i = py * pw + px;

                mcu = i / (mw * mh);
                lx = (i % (mw * mh)) % mw;
                ly = (i % (mw * mh)) / mw;
            }
            at = mcu * mcu_bytes;
            if (at + mcu_bytes > s->ro_num) {
                y = u = v = 0;
                m = NULL;
            } else {
                m = s->ro_buf + at;
            }
            if (m) {
                switch (fmt) {
                case ESP32P4_DMA2D_MCU_GRAY:
                    y = m[ly * 8 + lx];
                    u = v = 128;
                    break;
                case ESP32P4_DMA2D_MCU_YUV444:
                    y = m[ly * 8 + lx];
                    u = m[64 + ly * 8 + lx];
                    v = m[128 + ly * 8 + lx];
                    break;
                case ESP32P4_DMA2D_MCU_YUV422:
                    y = m[(lx / 8) * 64 + ly * 8 + lx % 8];
                    u = m[128 + ly * 8 + lx / 2];
                    v = m[192 + ly * 8 + lx / 2];
                    break;
                default:
                    y = m[((ly / 8) * 2 + lx / 8) * 64 + (ly % 8) * 8 +
                          lx % 8];
                    u = m[256 + (ly / 2) * 8 + lx / 2];
                    v = m[320 + (ly / 2) * 8 + lx / 2];
                    break;
                }
            }
            if (o + 8 > ESP32P4_DMA2D_RO_MAX) {
                break;
            }
            if (native) {
                switch (fmt) {
                case ESP32P4_DMA2D_MCU_GRAY:
                    s->ro_out[o++] = y;
                    break;
                case ESP32P4_DMA2D_MCU_YUV444:
                    s->ro_out[o++] = v;
                    s->ro_out[o++] = u;
                    s->ro_out[o++] = y;
                    break;
                case ESP32P4_DMA2D_MCU_YUV422:
                    if (!(px & 1)) {
                        s->ro_out[o++] = u;
                        s->ro_out[o++] = y;
                        s->ro_out[o++] = v;
                    } else {
                        s->ro_out[o++] = y;
                    }
                    break;
                default:
                    if (!(px & 1)) {
                        s->ro_out[o++] = py & 1 ? v : u;
                    }
                    s->ro_out[o++] = y;
                    break;
                }
                continue;
            }
            l[2] = y;
            l[1] = u;
            l[0] = v;
            o += csc_pixel(&k, l, true, &c->csc_pend, py, s->ro_out + o);
        }
    }
    c->csc_pend = 0;
    s->ro_out_len = o;
}

/* An MCU-mode RX channel: fill the reorder RAM, then write the block. */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t rx_mcu_service(ESP32P4Dma2dState *s)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][0];
    ESP32P4Dma2dDesc d = chan_desc(c);
    uint32_t pw, ph, mcu_bytes, mw, mh, want, nib = desc_nibbles(&d);
    uint8_t byte, flags;
    uint32_t desc;
    Dma2dRun r;

    if (!rx_mcu_block(c, &pw, &ph)) {
        return rx_close(s, 0, c->stage_flags & F_EOF, c->stage_flags & F_ERR);
    }
    if (!s->ro_out_len) {
        mcu_geometry(rx_mcu_format(s, c), &mcu_bytes, &mw, &mh);
        want = (pw / mw) * (ph / mh) * mcu_bytes;
        if (want > ESP32P4_DMA2D_RO_MAX) {
            if (!s->probe) {
                qemu_log_mask(LOG_UNIMP, "%s: an hb x vb block of %u bytes "
                              "of MCUs is larger than the model's reorder "
                              "RAM\n", TYPE_ESP32P4_DMA2D, want);
            }
            want = ESP32P4_DMA2D_RO_MAX;
        }
        while (s->ro_num < want && c->fifo.num && !c->stage_flags) {
            fifo_take(&c->fifo, &byte, &flags, &desc);
            if (!(flags & F_NODATA)) {
                s->ro_buf[s->ro_num++] = byte;
            }
            c->stage_flags = flags & (F_EOF | F_ERR);
        }
        if (s->ro_num < want && !c->stage_flags) {
            return 1;
        }
        rx_mcu_assemble(s, pw, ph);
        s->ro_num = 0;
        return 1;
    }
    /* The block's rows, cropped to the picture. */
    if (!chan_run(s, IN, 0, &r)) {
        return 1;
    }
    {
        uint32_t row_bytes = pad_bytes(nib, pw);
        uint32_t lbytes = pad_bytes(nib, r.lpad);
        uint32_t off = c->row * row_bytes + lbytes + c->rpos;
        uint32_t n = burst_len(c, r.addr + c->rpos, r.real - c->rpos);
        uint32_t cycles = 0;

        if (n && off + n <= s->ro_out_len) {
            dma2d_bus(s, IN, 0, r.addr + c->rpos, s->ro_out + off, n, true);
            cycles = beats(r.addr + c->rpos, n);
        }
        c->rpos += n;
        if (c->rpos < r.real) {
            return MAX(cycles, 1);
        }
        c->rpos = 0;
        c->row++;
        if (c->row < ph) {
            return MAX(cycles, 1);
        }
        c->row = 0;
        s->ro_out_len = 0;
        c->blk_x++;
        if (d.x + c->blk_x * d.hb >= d.ha) {
            c->blk_x = 0;
            c->blk_y++;
        }
        if (c->stage_flags || !rx_mcu_block(c, &pw, &ph)) {
            cycles += rx_close(s, 0, c->stage_flags & F_EOF,
                               c->stage_flags & F_ERR);
            c->stage_flags = 0;
        }
        return MAX(cycles, 1);
    }
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static uint32_t rx_service(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[IN][ch];
    ESP32P4Dma2dDesc d;
    uint32_t n, cycles = 0, room;
    bool full = c->fifo.num == FIFO_SIZE;
    Dma2dRun r;
    int st;

    if (!c->has_desc) {
        return chan_fetch(s, IN, ch);
    }
    if (rx_mcu_mode(s, ch)) {
        cycles = rx_mcu_service(s);
        goto out;
    }
    d = chan_desc(c);
    if (!d.en_2d) {
        room = d.size - c->pos;
        rx_stage(s, ch, MIN(chan_burst(c), room));
        n = burst_len(c, d.buf + c->pos, MIN(room, c->nstage));
        if (n) {
            dma2d_bus(s, IN, ch, d.buf + c->pos, c->stage, n, true);
            cycles = beats(d.buf + c->pos, n);
            c->pos += n;
            rx_unstage(c, n);
        }
        if ((c->stage_flags && !c->nstage) || c->pos == d.size) {
            cycles += rx_close(s, ch, c->stage_flags & F_EOF,
                               c->stage_flags & F_ERR);
            c->stage_flags = 0;
        }
        goto out;
    }
    st = chan_run(s, IN, ch, &r) ? run_settle(s, IN, ch, &r)
                                 : chan_port(c) ? RUN_WAIT : RUN_DONE;
    if (st == RUN_MORE) {
        uint32_t left = run_part(&r, c->phase) - c->rpos;

        if (c->phase == 1) {
            rx_stage(s, ch, MIN(chan_burst(c), left));
            n = burst_len(c, r.addr + c->rpos, MIN(left, c->nstage));
            dma2d_bus(s, IN, ch, r.addr + c->rpos, c->stage, n, true);
            cycles = beats(r.addr + c->rpos, n);
        } else {
            /* Bytes beyond the picture: taken and dropped. */
            rx_stage(s, ch, MIN(left, sizeof(c->stage)));
            n = MIN(left, c->nstage);
        }
        rx_unstage(c, n);
        c->rpos += n;
        st = run_settle(s, IN, ch, &r);
    }
    if (st == RUN_DONE ||
        (st != RUN_DONE && c->stage_flags && !c->nstage)) {
        /* The block is full, or the frame ended within it. */
        cycles += rx_close(s, ch, c->stage_flags & F_EOF,
                           c->stage_flags & F_ERR);
        c->stage_flags = 0;
    }
out:
    rx_check_empty(s, ch);
    if (full && c->fifo.num < FIFO_SIZE && !chan_m2m(s, ch)) {
        notify_periph(s, IN, ch);
    }
    return MAX(cycles, 1);
}

/* Memory-to-memory: TX FIFO to RX FIFO, outside the bus */

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void m2m_move(ESP32P4Dma2dState *s, int ch)
{
    ESP32P4Dma2dChan *tx = &s->ch[OUT][ch];
    ESP32P4Dma2dChan *rx;
    uint8_t byte, flags;
    uint32_t desc;

    if (!chan_m2m(s, ch)) {
        return;
    }
    rx = &s->ch[IN][ch];
    while (tx->fifo.num && rx_accepts(s, ch) && fifo_room(&rx->fifo)) {
        fifo_take(&tx->fifo, &byte, &flags, &desc);
        tx_left(s, ch, flags, desc);
        if (!(flags & F_NODATA) || (flags & F_EOF)) {
            fifo_put(&rx->fifo, byte, (flags & F_NODATA) | (flags & F_EOF),
                     0);
        }
    }
    rx_check_empty(s, ch);
}

/*
 * [spec:nuos:req:emu.esp32p4.dma]
 * Whether the transaction channel @ch of direction @dir would make next
 * waits on a memory controller's AXI port (esp32p4_mspi_dma_wait()): a
 * descriptor read, a data burst or padding read, and the descriptor
 * write-back and next-pointer re-read that close a descriptor. On the
 * chip the transaction is under way and the bus waits with it; the model
 * makes none of it until the port takes them all. The transaction is
 * worked out by running it with its accesses only asked about (@probe),
 * on the channel's state and the reorder RAM, which are then put back:
 * which accesses a transaction makes never depends on what it reads.
 * While no port holds accesses no access can wait, and nothing is asked.
 */
static bool chan_stalls(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *saved;
    uint32_t axi_err = s->axi_err, notify = s->notify;
    uint32_t ro_num = s->ro_num, ro_out_len = s->ro_out_len;
    g_autofree uint8_t *ro_buf = NULL;
    g_autofree uint8_t *ro_out = NULL;
    bool held;

    if (!esp32p4_mspi_dma_holds()) {
        return false;
    }
    saved = g_memdup2(&s->ch[dir][ch], sizeof(s->ch[dir][ch]));
    if (dir == IN && ch == 0) {
        ro_buf = g_memdup2(s->ro_buf, s->ro_buf_size);
        ro_out = g_memdup2(s->ro_out, s->ro_buf_size);
    }
    s->probe = true;
    s->probe_held = false;
    if (dir == OUT) {
        tx_service(s, ch);
    } else {
        rx_service(s, ch);
    }
    held = s->probe_held;
    s->probe = false;
    s->ch[dir][ch] = *saved;
    g_free(saved);
    s->axi_err = axi_err;
    s->notify = notify;
    s->ro_num = ro_num;
    s->ro_out_len = ro_out_len;
    if (ro_buf) {
        memcpy(s->ro_buf, ro_buf, s->ro_buf_size);
        memcpy(s->ro_out, ro_out, s->ro_buf_size);
    }
    return held;
}

/* Arbitration */

static bool chan_requests(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];

    if (!c->active || c->end || c->wait_ready ||
        (c->conf0 & (CONF0_RST | CONF0_CMD_DISABLE)) ||
        (s->rst_conf & (dir == OUT ? AXIM_RD_RST : AXIM_WR_RST))) {
        return false;
    }
    if (!c->has_desc) {
        return dir == IN || c->dq_num < ESP32P4_DMA2D_DESC_QUEUE;
    }
    if (dir == OUT) {
        if (chan_port(c) && !c->nport && (c->dw[0] & DW0_2DEN)) {
            return false;
        }
        return fifo_room(&c->fifo) >= TX_ROOM;
    }
    if (rx_mcu_mode(s, ch)) {
        return s->ro_out_len || c->fifo.num || c->stage_flags;
    }
    if (chan_port(c) && !c->nport && (c->dw[0] & DW0_2DEN)) {
        return false;
    }
    return c->fifo.num || c->nstage || c->stage_flags ||
           (!(c->dw[0] & DW0_2DEN) && c->pos == chan_desc(c).size);
}

static bool arb_weighted(ESP32P4Dma2dState *s, int arb)
{
    return s->arb_config[arb] & BIT(16);
}

static uint32_t arb_slot_cycles(ESP32P4Dma2dState *s, int arb)
{
    return extract32(s->arb_config[arb], 0, 16);
}

/* Arbiter 0 serves the TX channels' reads, arbiter 1 the RX's writes. */
static int arb_dir(int arb)
{
    return arb == 0 ? OUT : IN;
}

static void arb_refill(ESP32P4Dma2dState *s, int arb, int64_t cycle)
{
    int dir = arb_dir(arb);

    for (int ch = 0; ch < NCH(dir); ch++) {
        s->ch[dir][ch].tokens = chan_weight(&s->ch[dir][ch]);
    }
    s->slot_start[arb] = cycle;
}

/*
 * The channel arbiter @arb grants at bus cycle @cycle, or -1; *hold is
 * set when a weighted slot holds the bus for a channel without requests.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static int arb_grant(ESP32P4Dma2dState *s, int arb, int64_t cycle,
                     bool *hold)
{
    int dir = arb_dir(arb), n = NCH(dir), best = -1;
    bool weighted = arb_weighted(s, arb);
    uint32_t best_pri = 0;

    *hold = false;
    if (weighted) {
        bool any = false, held = false;

        if (cycle - s->slot_start[arb] >= arb_slot_cycles(s, arb)) {
            arb_refill(s, arb, cycle);
        }
        for (int ch = 0; ch < n; ch++) {
            ESP32P4Dma2dChan *c = &s->ch[dir][ch];

            if (!c->tokens) {
                continue;
            }
            if (chan_requests(s, dir, ch)) {
                any = true;
            } else if (c->conf0 & CONF0_WEIGHT_OPT_DIS) {
                held = true;
            }
        }
        if (!any) {
            if (held) {
                *hold = true;
                return -1;
            }
            arb_refill(s, arb, cycle);
        }
    }
    for (int k = 1; k <= n; k++) {
        int ch = (s->rr_last[arb] + k) % n;
        ESP32P4Dma2dChan *c = &s->ch[dir][ch];

        if (!chan_requests(s, dir, ch) || (weighted && !c->tokens)) {
            continue;
        }
        if (best < 0 || chan_priority(c) > best_pri) {
            best = ch;
            best_pri = chan_priority(c);
        }
    }
    if (best >= 0) {
        s->rr_last[arb] = best;
        if (weighted) {
            s->ch[dir][best].tokens--;
        }
    }
    return best;
}

/* The peripherals' paths */

/* The channel of direction @dir serving peripheral @sel, or -1. */
static int periph_chan(ESP32P4Dma2dState *s, int dir, unsigned sel)
{
    for (int ch = 0; ch < NCH(dir); ch++) {
        ESP32P4Dma2dChan *c = &s->ch[dir][ch];

        if (c->peri_sel == sel && !chan_m2m(s, ch) &&
            !(c->conf0 & CONF0_RST)) {
            return ch;
        }
    }
    return -1;
}

static void dma2d_notify(ESP32P4Dma2dState *s)
{
    uint32_t pending = s->notify;

    s->notify = 0;
    for (int dir = 0; dir < 2; dir++) {
        for (unsigned p = 0; p < ESP32P4_DMA2D_PERIPHS; p++) {
            const ESP32P4Dma2dPeriphOps *ops = s->periph_ops[dir][p];

            if (!(pending & BIT(p + (dir == IN ? 8 : 0))) || !ops) {
                continue;
            }
            if (dir == OUT && ops->tx_ready) {
                ops->tx_ready(s->periph_opaque[dir][p], p);
            } else if (dir == IN && ops->rx_ready) {
                ops->rx_ready(s->periph_opaque[dir][p], p);
            }
        }
    }
}

/* The engine: bus rounds and the peripherals' timers, in time order */

static void dma2d_rearm(ESP32P4Dma2dState *s)
{
    int64_t t = s->round_due ? s->due_ns : INT64_MAX;

    for (int i = 0; i < s->nptimers; i++) {
        if (s->ptimer_due[i] >= 0) {
            t = MIN(t, s->ptimer_due[i]);
        }
    }
    if (t == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, t);
    }
}

/* A bus round at bus cycle @cycle. */
static void dma2d_arm_round(ESP32P4Dma2dState *s, int64_t cycle)
{
    s->round_due = true;
    s->due_cycle = cycle;
    s->due_ns = esp32p4_clk_count_time(&s->bus_edges, cycle);
    if (!s->running) {
        dma2d_rearm(s);
    }
}

/*
 * One round of the bus, at the bus cycle it falls due, at
 * esp32_line_time(): each arbiter grants a transaction, whose effects are
 * seen at once, and the engine comes back when it is over.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void dma2d_round(void *opaque)
{
    ESP32P4Dma2dState *s = opaque;
    int64_t cycle = s->due_cycle;
    uint32_t cost = 0;
    int64_t wait = 0;

    if (!clock_is_enabled(s->clk)) {
        return;
    }
    for (int ch = 0; ch < NRX; ch++) {
        m2m_move(s, ch);
    }
    for (int arb = 0; arb < 2; arb++) {
        bool hold = false;
        int g = s->stall_grant[arb];

        /* A transaction the AXI port held goes first, once it takes it. */
        s->stall_grant[arb] = -1;
        if (g >= 0 && !chan_requests(s, arb_dir(arb), g)) {
            g = -1;
        }
        if (g < 0) {
            g = arb_grant(s, arb, cycle, &hold);
        }
        if (g >= 0 && chan_stalls(s, arb_dir(arb), g)) {
            s->stall_grant[arb] = g;
        } else if (g >= 0) {
            uint32_t c = arb == 0 ? tx_service(s, g) : rx_service(s, g);

            cost = MAX(cost, MAX(c, 1));
        } else if (hold) {
            int64_t end = s->slot_start[arb] + arb_slot_cycles(s, arb);

            if (end > cycle && (!wait || end - cycle < wait)) {
                wait = end - cycle;
            }
        }
    }
    for (int ch = 0; ch < NRX; ch++) {
        m2m_move(s, ch);
    }
    if (cost) {
        dma2d_arm_round(s, cycle + cost);
    } else if (wait) {
        dma2d_arm_round(s, cycle + wait);
    }
    dma2d_notify(s);
}

/*
 * Run the rounds and the peripherals' timers that fell due by @now, in
 * order, each at its own time; a round and a timer due together, the
 * timer first.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void dma2d_run_due(ESP32P4Dma2dState *s, int64_t now)
{
    if (s->running) {
        return;
    }
    s->running = true;
    for (;;) {
        int64_t tr = s->round_due ? s->due_ns : INT64_MAX;
        int64_t tp = INT64_MAX;
        int p = -1;

        for (int i = 0; i < s->nptimers; i++) {
            if (s->ptimer_due[i] >= 0 && s->ptimer_due[i] < tp) {
                tp = s->ptimer_due[i];
                p = i;
            }
        }
        if (MIN(tr, tp) > now) {
            break;
        }
        if (tp <= tr) {
            s->ptimer_due[p] = -1;
            esp32_line_run_at(tp, s->ptimer_fn[p], s->ptimer_opaque[p]);
            s->running = true;
        } else {
            s->round_due = false;
            esp32_line_run_at(tr, dma2d_round, s);
        }
    }
    s->running = false;
    dma2d_rearm(s);
}

static void dma2d_timer_cb(void *opaque)
{
    ESP32P4Dma2dState *s = opaque;

    dma2d_run_due(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/*
 * Bring everything that moves data through the controller up to @now:
 * the attached peripherals that keep their own time, then the rounds and
 * timers due.
 */
static void dma2d_catch_up(ESP32P4Dma2dState *s, int64_t now)
{
    if (s->running) {
        return;
    }
    for (int dir = 0; dir < 2; dir++) {
        for (int p = 0; p < ESP32P4_DMA2D_PERIPHS; p++) {
            const ESP32P4Dma2dPeriphOps *ops = s->periph_ops[dir][p];

            if (ops && ops->sync) {
                ops->sync(s->periph_opaque[dir][p], now);
            }
        }
    }
    dma2d_run_due(s, now);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
void esp32p4_dma2d_sync(ESP32P4Dma2dState *s, int64_t when)
{
    dma2d_run_due(s, when);
}

/*
 * Let the engine look for work at the bus cycle after the change that
 * calls for it, at esp32_line_time().
 */
static void dma2d_kick(ESP32P4Dma2dState *s)
{
    if (!s->round_due && clock_is_enabled(s->clk)) {
        dma2d_arm_round(s, esp32p4_clk_count_at(&s->bus_edges,
                                                esp32_line_time()) + 1);
    }
}

/*
 * A memory controller's AXI port takes accesses again: the transactions
 * it held go on, from the bus cycle after.
 */
static void dma2d_bus_ready(Notifier *n, void *data)
{
    ESP32P4Dma2dState *s = container_of(n, ESP32P4Dma2dState, bus_wait);

    notifier_remove(n);
    dma2d_kick(s);
}

/*
 * The bus clock changes: the rounds due before run at the old rate, and
 * the engine goes on at the new one, or stops, its cycles counted afresh
 * from the change.
 */
static void dma2d_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4Dma2dState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t cycles;

    if (event == ClockPreUpdate) {
        dma2d_catch_up(s, now);
        return;
    }
    cycles = esp32p4_clk_count_at(&s->bus_edges, now);
    s->slot_start[0] -= cycles;
    s->slot_start[1] -= cycles;
    s->due_cycle -= cycles;
    esp32p4_clk_count_start(&s->bus_edges, s->clk, now);
    if (!clock_is_enabled(s->clk)) {
        s->round_due = false;
        dma2d_rearm(s);
    } else if (s->round_due) {
        dma2d_arm_round(s, MAX(s->due_cycle, 1));
    } else {
        dma2d_kick(s);
    }
}

/* The peripherals' timers. */

int esp32p4_dma2d_timer_new(ESP32P4Dma2dState *s, ESP32P4Dma2dTimerFn *fn,
                            void *opaque)
{
    assert(s->nptimers < ESP32P4_DMA2D_TIMERS);
    s->ptimer_fn[s->nptimers] = fn;
    s->ptimer_opaque[s->nptimers] = opaque;
    s->ptimer_due[s->nptimers] = -1;
    return s->nptimers++;
}

void esp32p4_dma2d_timer_mod(ESP32P4Dma2dState *s, int t, int64_t when)
{
    s->ptimer_due[t] = MAX(when, 0);
    if (!s->running) {
        dma2d_rearm(s);
    }
}

void esp32p4_dma2d_timer_del(ESP32P4Dma2dState *s, int t)
{
    s->ptimer_due[t] = -1;
    if (!s->running) {
        dma2d_rearm(s);
    }
}

/* The peripheral API */

void esp32p4_dma2d_attach(ESP32P4Dma2dState *s, int dir, unsigned sel,
                          const ESP32P4Dma2dPeriphOps *ops, void *opaque)
{
    assert(sel < ESP32P4_DMA2D_PERIPHS && periph_real(dir, sel));
    s->periph_ops[dir][sel] = ops;
    s->periph_opaque[dir][sel] = opaque;
}

bool esp32p4_dma2d_desc(ESP32P4Dma2dState *s, int dir, unsigned sel,
                        ESP32P4Dma2dDesc *d)
{
    int ch = periph_chan(s, dir, sel);
    ESP32P4Dma2dChan *c;

    if (ch < 0) {
        return false;
    }
    c = &s->ch[dir][ch];
    if (dir == OUT) {
        const uint32_t *q = c->dq[c->dq_head];

        if (!c->dq_num) {
            return false;
        }
        desc_parse_words(&q[1], q[0], d);
        return true;
    }
    if (!c->has_desc) {
        return false;
    }
    *d = chan_desc(c);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
size_t esp32p4_dma2d_pull(ESP32P4Dma2dState *s, unsigned sel, uint8_t *buf,
                          size_t len, bool *eof)
{
    int ch = periph_chan(s, OUT, sel);
    ESP32P4Dma2dChan *c;
    size_t n = 0;
    uint8_t byte, flags;
    uint32_t desc;

    *eof = false;
    if (ch < 0) {
        return 0;
    }
    c = &s->ch[OUT][ch];
    while (c->fifo.num && n < len) {
        fifo_take(&c->fifo, &byte, &flags, &desc);
        if (!(flags & F_NODATA)) {
            buf[n++] = byte;
        }
        tx_left(s, ch, flags, desc);
        if (flags & F_EOF) {
            *eof = true;
            break;
        }
    }
    dma2d_kick(s);
    return n;
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
size_t esp32p4_dma2d_push(ESP32P4Dma2dState *s, unsigned sel,
                          const uint8_t *buf, size_t len, unsigned flags)
{
    int ch = periph_chan(s, IN, sel);
    ESP32P4Dma2dChan *c;
    uint8_t mark = (flags & ESP32P4_DMA2D_EOF ? F_EOF : 0) |
                   (flags & ESP32P4_DMA2D_ERR_EOF ? F_ERR : 0);
    size_t n;

    if (ch < 0 || !rx_accepts(s, ch)) {
        return 0;
    }
    c = &s->ch[IN][ch];
    if (!len) {
        if (mark && fifo_room(&c->fifo)) {
            fifo_put(&c->fifo, 0, F_NODATA | mark, 0);
        }
    } else {
        n = MIN(len, fifo_room(&c->fifo));
        for (size_t i = 0; i < n; i++) {
            fifo_put(&c->fifo, buf[i], i == len - 1 ? mark : 0, 0);
        }
        len = n;
    }
    rx_check_empty(s, ch);
    dma2d_kick(s);
    return len;
}

size_t esp32p4_dma2d_tx_pending(ESP32P4Dma2dState *s, unsigned sel)
{
    int ch = periph_chan(s, OUT, sel);

    return ch < 0 ? 0 : s->ch[OUT][ch].fifo.num;
}

size_t esp32p4_dma2d_rx_room(ESP32P4Dma2dState *s, unsigned sel)
{
    int ch = periph_chan(s, IN, sel);

    return ch < 0 || !rx_accepts(s, ch) ? 0
                                         : fifo_room(&s->ch[IN][ch].fifo);
}

void esp32p4_dma2d_port_block(ESP32P4Dma2dState *s, unsigned sel,
                              uint32_t *w, uint32_t *h)
{
    int ch = periph_chan(s, OUT, sel);
    uint32_t v = ch < 0 ? 0 : s->ch[OUT][ch].port_blk;

    *w = extract32(v, 0, 14);
    *h = extract32(v, 14, 14);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
bool esp32p4_dma2d_port_tx(ESP32P4Dma2dState *s, unsigned sel, int32_t x,
                           int32_t y, bool last)
{
    int ch = periph_chan(s, OUT, sel);
    ESP32P4Dma2dChan *c;

    if (ch < 0) {
        return false;
    }
    c = &s->ch[OUT][ch];
    if (!chan_port(c) || c->nport == ESP32P4_DMA2D_PORT_QUEUE) {
        return false;
    }
    c->port[c->nport++] = (ESP32P4Dma2dPortBlock) {
        .x = x, .y = y, .last = last,
    };
    dma2d_kick(s);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
bool esp32p4_dma2d_port_rx(ESP32P4Dma2dState *s, unsigned sel, uint32_t x,
                           uint32_t y, uint32_t w, uint32_t h,
                           unsigned nibbles, bool last)
{
    int ch = periph_chan(s, IN, sel);
    ESP32P4Dma2dChan *c;

    if (ch < 0) {
        return false;
    }
    c = &s->ch[IN][ch];
    if (!chan_port(c) || c->nport == ESP32P4_DMA2D_PORT_QUEUE) {
        return false;
    }
    c->port[c->nport++] = (ESP32P4Dma2dPortBlock) {
        .x = x, .y = y, .w = w, .h = h, .nibbles = nibbles, .last = last,
    };
    dma2d_kick(s);
    return true;
}

void esp32p4_dma2d_rx_mcu_format(ESP32P4Dma2dState *s, unsigned sel,
                                 int format)
{
    s->mcu_format = format;
}

/* Registers */

static uint32_t fifo_status(ESP32P4Dma2dState *s, int dir, int ch)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    uint32_t num = c->fifo.num;
    bool full = num == FIFO_SIZE || (dir == IN && !rx_accepts(s, ch) &&
                                     c->active);

    /* The FIFO as the L3 FIFO; the L1 and L2 FIFOs empty. */
    return BIT(1) | BIT(16) | (full ? BIT(22) : 0) | (num ? 0 : BIT(23)) |
           MIN(num, 31) << 24 | (dir == OUT ? 0xff << 7 : 0);
}

static uint32_t link_read(ESP32P4Dma2dChan *c, int dir)
{
    bool park = !c->active || (c->end && !c->has_desc);

    if (dir == IN) {
        return (c->link & IN_LINK_AUTO_RET) | (park ? IN_LINK_PARK : 0);
    }
    return park ? OUT_LINK_PARK : 0;
}

static uint32_t state_read(ESP32P4Dma2dChan *c, int dir)
{
    bool busy = c->active && !(c->end && !c->has_desc);
    uint32_t addr = (c->has_desc ? c->dscr_bf0 : c->desc_next) & 0x3ffff;
    uint32_t v = addr | (c->has_desc ? 1 << 18 : 0);

    if (dir == OUT) {
        return v | (busy ? 1 << 20 : 0) | BIT(24);
    }
    return v | (busy ? 1 << 20 : 0) | BIT(23);
}

static uint32_t ro_status_read(ESP32P4Dma2dChan *c, int dir, int ch)
{
    static const uint8_t pixel_byte[6] = { 0, 0, 1, 2, 4, 5 };
    ESP32P4Dma2dDesc d;
    uint32_t v = 0, mbw, mbh;

    if (!c->has_desc || !(c->dw[0] & DW0_2DEN)) {
        return 0;
    }
    d = chan_desc(c);
    chan_mb(c, &mbw, &mbh);
    v = pixel_byte[MIN(d.pbyte, 5)];
    if (ch == 0 && (c->conf0 & CONF0_REORDER_EN) && mbw && d.mod1) {
        v |= MIN(d.hb / mbw, 15) << 4;
    }
    return dir == OUT ? v << 10 : v << 9;
}

static bool dma2d_reg_read(ESP32P4Dma2dState *s, Dma2dDecode d, uint32_t *v)
{
    ESP32P4Dma2dChan *c = &s->ch[d.dir][d.ch];

    switch (d.reg) {
    case R_CONF0:
        *v = c->conf0;
        break;
    case R_INT_RAW:
        *v = c->int_raw;
        break;
    case R_INT_ENA:
        *v = c->int_ena;
        break;
    case R_INT_ST:
        *v = c->int_raw & c->int_ena;
        break;
    case R_INT_CLR:
        *v = 0;
        break;
    case R_FIFO_STATUS:
        *v = fifo_status(s, d.dir, d.ch);
        break;
    case R_PUSHPOP:
        *v = c->pushpop;
        break;
    case R_LINK_CONF:
        *v = link_read(c, d.dir);
        break;
    case R_LINK_ADDR:
        *v = c->link_addr;
        break;
    case R_STATE:
        *v = state_read(c, d.dir);
        break;
    case R_EOF_DES:
        *v = c->eof_des_addr;
        break;
    case R_ERR_EOF_DES:
        *v = c->err_eof_des_addr;
        break;
    case R_DSCR:
        *v = c->dscr;
        break;
    case R_DSCR_BF0:
        *v = c->dscr_bf0;
        break;
    case R_DSCR_BF1:
        *v = c->dscr_bf1;
        break;
    case R_PERI_SEL:
        *v = c->peri_sel;
        break;
    case R_ARB:
        *v = c->arb;
        break;
    case R_RO_STATUS:
        *v = ro_status_read(c, d.dir, d.ch);
        break;
    case R_RO_PD_CONF:
        *v = c->ro_pd_conf;
        break;
    case R_COLOR_CONVERT:
        *v = c->color_convert;
        break;
    case R_SCRAMBLE:
        *v = c->scramble;
        break;
    case R_COLOR_PARAM:
        *v = c->color_param[d.idx];
        break;
    case R_ETM_CONF:
        *v = c->etm_conf;
        break;
    case R_PORT_BLK:
        *v = c->port_blk;
        break;
    case R_AXI_ERR:
        *v = s->axi_err;
        break;
    case R_RST_CONF:
        *v = s->rst_conf;
        break;
    case R_INTR_START:
        *v = s->intr_mem_start;
        break;
    case R_INTR_END:
        *v = s->intr_mem_end;
        break;
    case R_EXTR_START:
        *v = s->extr_mem_start;
        break;
    case R_EXTR_END:
        *v = s->extr_mem_end;
        break;
    case R_OUT_ARB_CONFIG:
        *v = s->arb_config[0];
        break;
    case R_IN_ARB_CONFIG:
        *v = s->arb_config[1];
        break;
    case R_RDN_RESULT:
        *v = s->rdn_result;
        break;
    case R_RDN_ECO_HIGH:
        *v = s->rdn_eco_high;
        break;
    case R_RDN_ECO_LOW:
        *v = s->rdn_eco_low;
        break;
    case R_DATE:
        *v = s->date;
        break;
    default:
        *v = 0;
        return false;
    }
    return true;
}

static uint64_t esp32p4_dma2d_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4Dma2dState *s = opaque;
    Dma2dDecode d = dma2d_decode(addr & ~3);
    uint32_t v;

    dma2d_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (!dma2d_reg_read(s, d, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DMA2D, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static void link_write(ESP32P4Dma2dState *s, int dir, int ch, uint32_t v)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    uint32_t stop = dir == IN ? IN_LINK_STOP : OUT_LINK_STOP;
    uint32_t start = dir == IN ? IN_LINK_START : OUT_LINK_START;
    uint32_t restart = dir == IN ? IN_LINK_RESTART : OUT_LINK_RESTART;

    if (dir == IN) {
        c->link = v & IN_LINK_AUTO_RET;
    }
    if (v & stop) {
        /* The channel stops once it has finished its descriptor. */
        if (c->has_desc) {
            c->stop = true;
        } else {
            chan_halt(c);
        }
    }
    if (v & start) {
        if (chan_etm_en(c)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: LINK_START "
                          "is ignored while ETM_EN is set\n",
                          TYPE_ESP32P4_DMA2D, dir_name(dir), ch);
        } else {
            chan_start(s, dir, ch);
        }
    }
    if (v & restart) {
        chan_restart(s, dir, ch);
    }
}

static void pushpop_write(ESP32P4Dma2dState *s, int dir, int ch, uint32_t v)
{
    ESP32P4Dma2dChan *c = &s->ch[dir][ch];
    uint8_t byte, flags;
    uint32_t desc;

    if (dir == OUT) {
        c->pushpop = v & 0x3ff;
        if (!(v & BIT(10))) {
            return;
        }
        if (!fifo_room(&c->fifo)) {
            dma2d_raise(s, OUT, ch, OUT_FIFO_OVF_L1);
            return;
        }
        fifo_put(&c->fifo, v & 0xff, v & BIT(8) ? F_EOF : 0, 0);
        if (c->fifo.num == 1 && !chan_m2m(s, ch)) {
            notify_periph(s, OUT, ch);
        }
        dma2d_kick(s);
        return;
    }
    if (!(v & BIT(11))) {
        return;
    }
    if (!c->fifo.num) {
        c->pushpop = POP_EMPTY;
        dma2d_raise(s, IN, ch, IN_FIFO_UDF_L1);
        return;
    }
    fifo_take(&c->fifo, &byte, &flags, &desc);
    c->pushpop = byte | (flags & (F_EOF | F_ERR) ? BIT(8) : 0);
    dma2d_kick(s);
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static bool dma2d_reg_write(ESP32P4Dma2dState *s, Dma2dDecode d, uint32_t v)
{
    ESP32P4Dma2dChan *c = &s->ch[d.dir][d.ch];

    switch (d.reg) {
    case R_CONF0:
        c->conf0 = v & (d.dir == OUT ? OUT_CONF0_MASK : IN_CONF0_MASK);
        if (c->conf0 & CONF0_RST) {
            chan_reset_fsm(s, d.dir, d.ch);
        }
        dma2d_kick(s);
        break;
    case R_INT_RAW:
    case R_INT_CLR:
        /* INT_RAW is write-1-to-clear, as INT_CLR is. */
        c->int_raw &= ~v;
        dma2d_update_irq(s, d.dir, d.ch);
        break;
    case R_INT_ENA:
        c->int_ena = v & (d.dir == OUT ? OUT_INT_MASK : IN_INT_MASK);
        dma2d_update_irq(s, d.dir, d.ch);
        break;
    case R_PUSHPOP:
        pushpop_write(s, d.dir, d.ch, v);
        break;
    case R_LINK_CONF:
        link_write(s, d.dir, d.ch, v);
        break;
    case R_LINK_ADDR:
        c->link_addr = v;
        break;
    case R_PERI_SEL:
        c->peri_sel = v & 7;
        for (int ch = 0; ch < NCH(d.dir); ch++) {
            if (ch != d.ch && periph_real(d.dir, c->peri_sel) &&
                s->ch[d.dir][ch].peri_sel == c->peri_sel) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channels %d and %d "
                              "both select peripheral %u\n",
                              TYPE_ESP32P4_DMA2D, dir_name(d.dir), ch, d.ch,
                              c->peri_sel);
            }
        }
        dma2d_kick(s);
        break;
    case R_ARB:
        c->arb = v & 0xff;
        break;
    case R_RO_PD_CONF:
        c->ro_pd_conf = v & 0x70;
        break;
    case R_COLOR_CONVERT:
        c->color_convert = v & 0x3f;
        break;
    case R_SCRAMBLE:
        c->scramble = v & (d.dir == OUT ? 0x7 : 0x3f);
        break;
    case R_COLOR_PARAM:
        c->color_param[d.idx] = v & (d.idx & 1 ? 0xfffffff : 0x1fffff);
        break;
    case R_ETM_CONF:
        c->etm_conf = v & 0xf;
        break;
    case R_PORT_BLK:
        c->port_blk = v & 0xfffffff;
        break;
    case R_RST_CONF:
        s->rst_conf = v & 7;
        dma2d_kick(s);
        break;
    case R_INTR_START:
        s->intr_mem_start = v;
        break;
    case R_INTR_END:
        s->intr_mem_end = v;
        break;
    case R_EXTR_START:
        s->extr_mem_start = v;
        break;
    case R_EXTR_END:
        s->extr_mem_end = v;
        break;
    case R_OUT_ARB_CONFIG:
        s->arb_config[0] = v & 0x1ffff;
        dma2d_kick(s);
        break;
    case R_IN_ARB_CONFIG:
        s->arb_config[1] = v & 0x1ffff;
        dma2d_kick(s);
        break;
    case R_RDN_RESULT:
        s->rdn_result = v & 1;
        break;
    case R_RDN_ECO_HIGH:
        s->rdn_eco_high = v;
        break;
    case R_RDN_ECO_LOW:
        s->rdn_eco_low = v;
        break;
    case R_DATE:
        s->date = v;
        break;
    case R_INT_ST:
    case R_FIFO_STATUS:
    case R_STATE:
    case R_EOF_DES:
    case R_ERR_EOF_DES:
    case R_DSCR:
    case R_DSCR_BF0:
    case R_DSCR_BF1:
    case R_RO_STATUS:
    case R_AXI_ERR:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to a read-only register\n",
                      TYPE_ESP32P4_DMA2D);
        break;
    default:
        return false;
    }
    return true;
}

static void esp32p4_dma2d_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    ESP32P4Dma2dState *s = opaque;
    Dma2dDecode d = dma2d_decode(addr & ~3);
    uint32_t old = 0, v;

    dma2d_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    /*
     * A narrow write keeps the other bytes; the write-1 bits of the
     * interrupt clears and the link and push/pop commands act only in the
     * bytes written.
     */
    switch (d.reg) {
    case R_INT_RAW:
    case R_INT_CLR:
        break;
    case R_LINK_CONF:
        old = d.dir == IN ? s->ch[IN][d.ch].link & IN_LINK_AUTO_RET : 0;
        break;
    case R_PUSHPOP:
        old = d.dir == OUT ? s->ch[OUT][d.ch].pushpop & 0x3ff : 0;
        break;
    default:
        dma2d_reg_read(s, d, &old);
        break;
    }
    v = deposit32(old, (addr & 3) * 8, size * 8, value);
    if (!dma2d_reg_write(s, d, v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DMA2D, addr);
    }
    if (!s->running) {
        dma2d_notify(s);
        dma2d_rearm(s);
    }
}

static const MemoryRegionOps esp32p4_dma2d_ops = {
    .read = esp32p4_dma2d_read,
    .write = esp32p4_dma2d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * ETM tasks, on their rising edges, at their events' time: IN_START and
 * OUT_START start a channel whose ETM_EN is set; DSCR_READY lets a
 * channel with ETM_LOOP_EN go on to its next descriptor, or is kept for
 * when it gets there.
 */
/* [spec:nuos:req:emu.esp32p4.ppa] */
static void esp32p4_dma2d_etm_task(void *opaque, int n, int level)
{
    ESP32P4Dma2dState *s = opaque;
    bool rise = level && !s->etm_task_level[n];
    int dir, ch;
    bool ready;
    ESP32P4Dma2dChan *c;

    s->etm_task_level[n] = level;
    if (!rise) {
        return;
    }
    if (n < 4) {
        dir = IN;
        ch = n % 2;
        ready = n >= 2;
    } else {
        dir = OUT;
        ch = (n - 4) % 3;
        ready = n >= 7;
    }
    c = &s->ch[dir][ch];
    dma2d_run_due(s, esp32_line_time());
    if (!ready) {
        if (chan_etm_en(c)) {
            chan_start(s, dir, ch);
        }
    } else if (c->etm_conf & ETM_LOOP_EN) {
        if (c->wait_ready) {
            c->wait_ready = false;
            dma2d_kick(s);
        } else if (c->tasks < extract32(c->etm_conf, 2, 2) + 1) {
            c->tasks++;
        } else {
            dma2d_raise(s, dir, ch, dir == IN ? IN_DSCR_TASK_OVF
                                              : OUT_DSCR_TASK_OVF);
        }
    }
    dma2d_notify(s);
    dma2d_rearm(s);
}

/*
 * The qtest hook through which a test plays a peripheral: setting
 * "periph" to
 *   "push <sel> <hex bytes> <flags>"  pushes as esp32p4_dma2d_push();
 *   "pull <sel> <count>"              pulls, giving "<count> <eof> <hex>";
 *   "desc <dir> <sel>"                gives the descriptor the channel
 *                                     serving it holds, "<ok> hb vb ha va
 *                                     x y pbyte mod 2d buf";
 *   "port-tx <sel> <x> <y> <last>" and
 *   "port-rx <sel> <x> <y> <w> <h> <nibbles> <last>"  queue a DSCR-PORT
 *                                     block, giving 1 or 0;
 *   "mcu <sel> <format>"              names the MCUs' format;
 * and reading it back gives the result.
 */
static char *esp32p4_dma2d_get_periph(Object *obj, Error **errp)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(obj);

    return g_strdup(s->periph_probe ? s->periph_probe : "");
}

static void esp32p4_dma2d_set_periph(Object *obj, const char *value,
                                     Error **errp)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(obj);
    g_auto(GStrv) f = g_strsplit(value, " ", 0);
    unsigned n = g_strv_length(f);
    int64_t a[8] = { 0 };
    g_autofree uint8_t *buf = NULL;
    GString *out;
    bool eof;
    size_t len;

    for (unsigned i = 1; i < n && i <= 7; i++) {
        if (i == 2 && !strcmp(f[0], "push")) {
            continue;
        }
        if (qemu_strtoi64(f[i], NULL, 0, &a[i])) {
            error_setg(errp, "periph: bad number '%s'", f[i]);
            return;
        }
    }
    if (n < 2 || a[1] < 0 || a[1] >= ESP32P4_DMA2D_PERIPHS) {
        error_setg(errp, "periph: no peripheral given");
        return;
    }
    dma2d_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    g_free(s->periph_probe);
    s->periph_probe = NULL;
    if (!strcmp(f[0], "push") && n >= 4) {
        len = strlen(f[2]) / 2;
        buf = g_malloc(len + 1);
        for (size_t i = 0; i < len; i++) {
            char byte[3] = { f[2][2 * i], f[2][2 * i + 1], 0 };
            unsigned long v = 0;

            qemu_strtoul(byte, NULL, 16, &v);
            buf[i] = v;
        }
        s->periph_probe = g_strdup_printf("%zu", esp32p4_dma2d_push(s, a[1],
                                          buf, len, a[3]));
    } else if (!strcmp(f[0], "pull") && n >= 3) {
        buf = g_malloc(a[2] + 1);
        len = esp32p4_dma2d_pull(s, a[1], buf, a[2], &eof);
        out = g_string_new(NULL);
        g_string_printf(out, "%zu %d ", len, eof);
        for (size_t i = 0; i < len; i++) {
            g_string_append_printf(out, "%02x", buf[i]);
        }
        s->periph_probe = g_string_free(out, false);
    } else if (!strcmp(f[0], "desc") && n >= 3) {
        ESP32P4Dma2dDesc d = { 0 };
        bool ok = esp32p4_dma2d_desc(s, a[1] ? OUT : IN, a[2], &d);

        s->periph_probe = g_strdup_printf("%d %u %u %u %u %u %u %u %d %d "
                                          "0x%x", ok, d.hb, d.vb, d.ha, d.va,
                                          d.x, d.y, d.pbyte, d.mod1, d.en_2d,
                                          d.buf);
    } else if (!strcmp(f[0], "port-tx") && n >= 5) {
        s->periph_probe = g_strdup_printf("%d", esp32p4_dma2d_port_tx(s,
                                          a[1], a[2], a[3], a[4]));
    } else if (!strcmp(f[0], "port-rx") && n >= 8) {
        s->periph_probe = g_strdup_printf("%d", esp32p4_dma2d_port_rx(s,
                                          a[1], a[2], a[3], a[4], a[5], a[6],
                                          a[7]));
    } else if (!strcmp(f[0], "mcu") && n >= 3) {
        esp32p4_dma2d_rx_mcu_format(s, a[1], a[2]);
        s->periph_probe = g_strdup("1");
    } else {
        error_setg(errp, "periph: unknown command '%s'", value);
    }
    dma2d_notify(s);
    dma2d_rearm(s);
}

static void esp32p4_dma2d_reset_hold(Object *obj, ResetType type)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(obj);

    s->round_due = false;
    for (int i = 0; i < s->nptimers; i++) {
        s->ptimer_due[i] = -1;
    }
    timer_del(s->timer);
    for (int dir = 0; dir < 2; dir++) {
        for (int ch = 0; ch < ESP32P4_DMA2D_CHANNELS; ch++) {
            ESP32P4Dma2dChan *c = &s->ch[dir][ch];

            memset(c, 0, sizeof(*c));
            c->conf0 = dir == OUT ? OUT_CONF0_RESET : 0;
            c->link = dir == IN ? IN_LINK_AUTO_RET : 0;
            c->peri_sel = ESP32P4_DMA2D_PERI_NONE;
            c->arb = ARB_RESET;
            c->ro_pd_conf = BIT(5);
            c->color_convert = COLOR_CONVERT_RESET;
            memcpy(c->color_param, color_param_reset,
                   sizeof(c->color_param));
            c->etm_conf = ETM_CONF_RESET;
            c->port_blk = dir == OUT ? PORT_BLK_RESET : 0;
            c->pushpop = dir == IN ? POP_EMPTY : 0;
        }
    }
    s->axi_err = 0;
    s->rst_conf = 0;
    s->intr_mem_start = MEM_RESET_START;
    s->intr_mem_end = MEM_RESET_END;
    s->extr_mem_start = MEM_RESET_START;
    s->extr_mem_end = MEM_RESET_END;
    s->arb_config[0] = s->arb_config[1] = 0;
    s->rdn_result = 0;
    s->rdn_eco_high = UINT32_MAX;
    s->rdn_eco_low = 0;
    s->date = DATE_RESET;
    s->rr_last[0] = s->rr_last[1] = 0;
    s->slot_start[0] = s->slot_start[1] = 0;
    s->stall_grant[0] = s->stall_grant[1] = -1;
    if (QLIST_IS_INSERTED(&s->bus_wait, node)) {
        notifier_remove(&s->bus_wait);
    }
    s->notify = 0;
    s->ro_num = 0;
    s->ro_out_len = 0;
    s->mcu_format = ESP32P4_DMA2D_MCU_AUTO;
    if (!esp32p4_clk_count_running(&s->bus_edges)) {
        esp32p4_clk_count_start(&s->bus_edges, s->clk,
                                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static void esp32p4_dma2d_reset_exit(Object *obj, ResetType type)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(obj);

    for (int i = 0; i < ESP32P4_DMA2D_IRQS; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void esp32p4_dma2d_init(Object *obj)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_dma2d_ops, s,
                          "esp32p4.dma2d", ESP32P4_DMA2D_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < ESP32P4_DMA2D_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), s->etm_event,
                             ESP32P4_DMA2D_ETM_EVENT,
                             ESP32P4_DMA2D_ETM_EVENTS);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_dma2d_etm_task,
                            ESP32P4_DMA2D_ETM_TASK, ESP32P4_DMA2D_ETM_TASKS);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", dma2d_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dma2d_timer_cb, s);
    s->bus_wait.notify = dma2d_bus_ready;
    s->stall_grant[0] = s->stall_grant[1] = -1;
    s->ro_buf_size = ESP32P4_DMA2D_RO_MAX;
    s->ro_buf = g_malloc0(ESP32P4_DMA2D_RO_MAX);
    s->ro_out = g_malloc0(ESP32P4_DMA2D_RO_MAX);
    object_property_add_str(obj, "periph", esp32p4_dma2d_get_periph,
                            esp32p4_dma2d_set_periph);
}

static void esp32p4_dma2d_realize(DeviceState *dev, Error **errp)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(dev);

    if (!s->pms) {
        error_setg(errp, "%s: the 'pms' link is not set", TYPE_ESP32P4_DMA2D);
        return;
    }
    s->as = esp32p4_pms_bus_as(ESP32P4_PMS(s->pms));
}

static void esp32p4_dma2d_finalize(Object *obj)
{
    ESP32P4Dma2dState *s = ESP32P4_DMA2D(obj);

    timer_free(s->timer);
    g_free(s->ro_buf);
    g_free(s->ro_out);
    g_free(s->periph_probe);
}

static const Property esp32p4_dma2d_properties[] = {
    DEFINE_PROP_LINK("pms", ESP32P4Dma2dState, pms, TYPE_ESP32P4_PMS,
                     Object *),
};

static const VMStateDescription vmstate_esp32p4_dma2d_fifo = {
    .name = "esp32p4-dma2d-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, ESP32P4Dma2dFifo, ESP32P4_DMA2D_FIFO_MAX),
        VMSTATE_UINT8_ARRAY(flags, ESP32P4Dma2dFifo, ESP32P4_DMA2D_FIFO_MAX),
        VMSTATE_UINT32_ARRAY(desc, ESP32P4Dma2dFifo, ESP32P4_DMA2D_FIFO_MAX),
        VMSTATE_UINT32(head, ESP32P4Dma2dFifo),
        VMSTATE_UINT32(num, ESP32P4Dma2dFifo),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_dma2d_port = {
    .name = "esp32p4-dma2d-port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT32(x, ESP32P4Dma2dPortBlock),
        VMSTATE_INT32(y, ESP32P4Dma2dPortBlock),
        VMSTATE_UINT32(w, ESP32P4Dma2dPortBlock),
        VMSTATE_UINT32(h, ESP32P4Dma2dPortBlock),
        VMSTATE_UINT32(nibbles, ESP32P4Dma2dPortBlock),
        VMSTATE_UINT32(last, ESP32P4Dma2dPortBlock),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_dma2d_chan = {
    .name = "esp32p4-dma2d-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, ESP32P4Dma2dChan),
        VMSTATE_UINT32(int_raw, ESP32P4Dma2dChan),
        VMSTATE_UINT32(int_ena, ESP32P4Dma2dChan),
        VMSTATE_UINT32(link, ESP32P4Dma2dChan),
        VMSTATE_UINT32(link_addr, ESP32P4Dma2dChan),
        VMSTATE_UINT32(peri_sel, ESP32P4Dma2dChan),
        VMSTATE_UINT32(arb, ESP32P4Dma2dChan),
        VMSTATE_UINT32(ro_pd_conf, ESP32P4Dma2dChan),
        VMSTATE_UINT32(color_convert, ESP32P4Dma2dChan),
        VMSTATE_UINT32(scramble, ESP32P4Dma2dChan),
        VMSTATE_UINT32_ARRAY(color_param, ESP32P4Dma2dChan, 6),
        VMSTATE_UINT32(etm_conf, ESP32P4Dma2dChan),
        VMSTATE_UINT32(port_blk, ESP32P4Dma2dChan),
        VMSTATE_UINT32(pushpop, ESP32P4Dma2dChan),
        VMSTATE_UINT32(eof_des_addr, ESP32P4Dma2dChan),
        VMSTATE_UINT32(err_eof_des_addr, ESP32P4Dma2dChan),
        VMSTATE_UINT32(dscr, ESP32P4Dma2dChan),
        VMSTATE_UINT32(dscr_bf0, ESP32P4Dma2dChan),
        VMSTATE_UINT32(dscr_bf1, ESP32P4Dma2dChan),
        VMSTATE_BOOL(active, ESP32P4Dma2dChan),
        VMSTATE_BOOL(has_desc, ESP32P4Dma2dChan),
        VMSTATE_BOOL(end, ESP32P4Dma2dChan),
        VMSTATE_BOOL(restart, ESP32P4Dma2dChan),
        VMSTATE_BOOL(stop, ESP32P4Dma2dChan),
        VMSTATE_BOOL(wait_ready, ESP32P4Dma2dChan),
        VMSTATE_BOOL(empty_raised, ESP32P4Dma2dChan),
        VMSTATE_UINT32(tasks, ESP32P4Dma2dChan),
        VMSTATE_UINT32(prev_desc, ESP32P4Dma2dChan),
        VMSTATE_UINT32(desc_next, ESP32P4Dma2dChan),
        VMSTATE_UINT32_ARRAY(dw, ESP32P4Dma2dChan, 5),
        VMSTATE_UINT32(pos, ESP32P4Dma2dChan),
        VMSTATE_UINT32(blk_x, ESP32P4Dma2dChan),
        VMSTATE_UINT32(blk_y, ESP32P4Dma2dChan),
        VMSTATE_UINT32(mb, ESP32P4Dma2dChan),
        VMSTATE_UINT32(row, ESP32P4Dma2dChan),
        VMSTATE_UINT32(phase, ESP32P4Dma2dChan),
        VMSTATE_UINT32(rpos, ESP32P4Dma2dChan),
        VMSTATE_UINT32(port_w, ESP32P4Dma2dChan),
        VMSTATE_UINT32(port_h, ESP32P4Dma2dChan),
        VMSTATE_UINT8_ARRAY(carry, ESP32P4Dma2dChan, 8),
        VMSTATE_UINT32(ncarry, ESP32P4Dma2dChan),
        VMSTATE_UINT32(csc_pend, ESP32P4Dma2dChan),
        VMSTATE_UINT32(csc_row, ESP32P4Dma2dChan),
        VMSTATE_UINT8_ARRAY(stage, ESP32P4Dma2dChan, ESP32P4_DMA2D_STAGE),
        VMSTATE_UINT32(nstage, ESP32P4Dma2dChan),
        VMSTATE_UINT8(stage_flags, ESP32P4Dma2dChan),
        VMSTATE_STRUCT_ARRAY(port, ESP32P4Dma2dChan, ESP32P4_DMA2D_PORT_QUEUE,
                             1, vmstate_esp32p4_dma2d_port,
                             ESP32P4Dma2dPortBlock),
        VMSTATE_UINT32(nport, ESP32P4Dma2dChan),
        VMSTATE_UINT32(tokens, ESP32P4Dma2dChan),
        VMSTATE_STRUCT(fifo, ESP32P4Dma2dChan, 1, vmstate_esp32p4_dma2d_fifo,
                       ESP32P4Dma2dFifo),
        VMSTATE_UINT32_2DARRAY(dq, ESP32P4Dma2dChan,
                               ESP32P4_DMA2D_DESC_QUEUE, 6),
        VMSTATE_UINT32(dq_head, ESP32P4Dma2dChan),
        VMSTATE_UINT32(dq_num, ESP32P4Dma2dChan),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * A transaction an AXI port held: ask again, which waits on the port anew
 * if it still holds it.
 */
static int esp32p4_dma2d_post_load(void *opaque, int version_id)
{
    ESP32P4Dma2dState *s = opaque;

    if (s->stall_grant[0] >= 0 || s->stall_grant[1] >= 0) {
        dma2d_kick(s);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_dma2d = {
    .name = TYPE_ESP32P4_DMA2D,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = esp32p4_dma2d_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_2DARRAY(ch, ESP32P4Dma2dState, 2,
                               ESP32P4_DMA2D_CHANNELS, 1,
                               vmstate_esp32p4_dma2d_chan, ESP32P4Dma2dChan),
        VMSTATE_UINT32(axi_err, ESP32P4Dma2dState),
        VMSTATE_UINT32(rst_conf, ESP32P4Dma2dState),
        VMSTATE_UINT32(intr_mem_start, ESP32P4Dma2dState),
        VMSTATE_UINT32(intr_mem_end, ESP32P4Dma2dState),
        VMSTATE_UINT32(extr_mem_start, ESP32P4Dma2dState),
        VMSTATE_UINT32(extr_mem_end, ESP32P4Dma2dState),
        VMSTATE_UINT32_ARRAY(arb_config, ESP32P4Dma2dState, 2),
        VMSTATE_UINT32(rdn_result, ESP32P4Dma2dState),
        VMSTATE_UINT32(rdn_eco_high, ESP32P4Dma2dState),
        VMSTATE_UINT32(rdn_eco_low, ESP32P4Dma2dState),
        VMSTATE_UINT32(date, ESP32P4Dma2dState),
        VMSTATE_BOOL_ARRAY(etm_task_level, ESP32P4Dma2dState,
                           ESP32P4_DMA2D_ETM_TASKS),
        VMSTATE_UINT32_ARRAY(rr_last, ESP32P4Dma2dState, 2),
        VMSTATE_INT64_ARRAY(slot_start, ESP32P4Dma2dState, 2),
        VMSTATE_INT32_ARRAY(stall_grant, ESP32P4Dma2dState, 2),
        VMSTATE_UINT32(notify, ESP32P4Dma2dState),
        VMSTATE_VBUFFER_UINT32(ro_buf, ESP32P4Dma2dState, 1, NULL,
                               ro_buf_size),
        VMSTATE_UINT32(ro_num, ESP32P4Dma2dState),
        VMSTATE_VBUFFER_UINT32(ro_out, ESP32P4Dma2dState, 1, NULL,
                               ro_buf_size),
        VMSTATE_UINT32(ro_out_len, ESP32P4Dma2dState),
        VMSTATE_INT32(mcu_format, ESP32P4Dma2dState),
        VMSTATE_INT64_ARRAY(ptimer_due, ESP32P4Dma2dState,
                            ESP32P4_DMA2D_TIMERS),
        VMSTATE_CLOCK(clk, ESP32P4Dma2dState),
        VMSTATE_ESP32P4_CLK_COUNT(bus_edges, ESP32P4Dma2dState),
        VMSTATE_BOOL(round_due, ESP32P4Dma2dState),
        VMSTATE_INT64(due_cycle, ESP32P4Dma2dState),
        VMSTATE_INT64(due_ns, ESP32P4Dma2dState),
        VMSTATE_TIMER_PTR(timer, ESP32P4Dma2dState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_dma2d_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_dma2d_realize;
    dc->vmsd = &vmstate_esp32p4_dma2d;
    device_class_set_props(dc, esp32p4_dma2d_properties);
    rc->phases.hold = esp32p4_dma2d_reset_hold;
    rc->phases.exit = esp32p4_dma2d_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.ppa] */
static const TypeInfo esp32p4_dma2d_info = {
    .name           = TYPE_ESP32P4_DMA2D,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(ESP32P4Dma2dState),
    .instance_init  = esp32p4_dma2d_init,
    .instance_finalize = esp32p4_dma2d_finalize,
    .class_init     = esp32p4_dma2d_class_init,
};

static void esp32p4_dma2d_register_types(void)
{
    type_register_static(&esp32p4_dma2d_info);
}

type_init(esp32p4_dma2d_register_types)
