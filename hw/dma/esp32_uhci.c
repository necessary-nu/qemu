/*
 * ESP32 UHCI (UDMA), the UART DMA engine
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "DMA Controller" (UART DMA, linked lists) and
 * "UART Controller" (UHCI interrupts and registers), with the field
 * layout and reset values of ESP-IDF's soc/esp32 uhci_reg.h.
 *
 * Each UHCI serves the UARTs selected by CONF0.UARTn_CE. The outlink
 * engine reads its descriptor chain from SRAM, passes the data through
 * the encoder (SLIP-style separators and escape sequences) and into its
 * DMA FIFO, which feeds the UART TX FIFO as the UART frees room. The
 * inlink engine takes the bytes the UART receives from its RX FIFO,
 * passes them through the decoder and writes them into its descriptor
 * chain. Bytes move as the UART takes or delivers them, so transfers run
 * at the UART's baud rate.
 *
 * Descriptors are the DMA engine's three words: DW0 holds owner (31),
 * eof (30), length (23:12) and size (11:0); DW1 the buffer address; DW2
 * the next descriptor's address, 0 at the end of the list. The DMA engine
 * reaches internal SRAM 1 and 2 only, 0x3ffae000..0x3fffffff; the link
 * registers hold the low 20 bits of a descriptor address in 0x3ffxxxxx.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32_uhci.h"
#include "hw/dma/esp32_lldesc.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#define A_CONF0                 0x00
#define A_INT_RAW               0x04
#define A_INT_ST                0x08
#define A_INT_ENA               0x0c
#define A_INT_CLR               0x10
#define A_DMA_OUT_STATUS        0x14
#define A_DMA_OUT_PUSH          0x18
#define A_DMA_IN_STATUS         0x1c
#define A_DMA_IN_POP            0x20
#define A_DMA_OUT_LINK          0x24
#define A_DMA_IN_LINK           0x28
#define A_CONF1                 0x2c
#define A_STATE0                0x30
#define A_STATE1                0x34
#define A_OUT_EOF_DES_ADDR      0x38
#define A_IN_SUC_EOF_DES_ADDR   0x3c
#define A_IN_ERR_EOF_DES_ADDR   0x40
#define A_OUT_EOF_BFR_DES_ADDR  0x44
#define A_AHB_TEST              0x48
#define A_IN_DSCR               0x4c
#define A_IN_DSCR_BF1           0x54
#define A_OUT_DSCR              0x58
#define A_OUT_DSCR_BF1          0x60
#define A_ESCAPE_CONF           0x64
#define A_HUNG_CONF             0x68
#define A_ACK_NUM               0x6c
#define A_RX_HEAD               0x70
#define A_QUICK_SENT            0x74
#define A_Q0_WORD0              0x78
#define A_Q6_WORD1              0xac
#define A_ESC_CONF0             0xb0
#define A_ESC_CONF3             0xbc
#define A_PKT_THRES             0xc0
#define A_DATE                  0xfc

/* CONF0 */
#define CONF0_IN_RST            (1u << 0)
#define CONF0_OUT_RST           (1u << 1)
#define CONF0_AHBM_FIFO_RST     (1u << 2)
#define CONF0_AHBM_RST          (1u << 3)
#define CONF0_IN_LOOP_TEST      (1u << 4)
#define CONF0_OUT_LOOP_TEST     (1u << 5)
#define CONF0_OUT_AUTO_WRBACK   (1u << 6)
#define CONF0_OUT_EOF_MODE      (1u << 8)
#define CONF0_UART_CE_SHIFT     9
#define CONF0_MEM_TRANS_EN      (1u << 15)
#define CONF0_SEPER_EN          (1u << 16)
#define CONF0_HEAD_EN           (1u << 17)
#define CONF0_CRC_REC_EN        (1u << 18)
#define CONF0_UART_IDLE_EOF_EN  (1u << 19)
#define CONF0_LEN_EOF_EN        (1u << 20)
#define CONF0_ENCODE_CRC_EN     (1u << 21)
#define CONF0_UART_RX_BRK_EOF_EN (1u << 23)
#define CONF0_MASK              0x00ffffffu
#define CONF0_RESET             0x00370100u
/* Modes the TRM leaves reserved ("initialize to 0") or does not describe */
#define CONF0_UNDOCUMENTED      (CONF0_IN_LOOP_TEST | CONF0_OUT_LOOP_TEST | \
                                 CONF0_MEM_TRANS_EN | CONF0_HEAD_EN | \
                                 CONF0_CRC_REC_EN | CONF0_UART_IDLE_EOF_EN | \
                                 CONF0_LEN_EOF_EN | CONF0_ENCODE_CRC_EN | \
                                 CONF0_UART_RX_BRK_EOF_EN)

/* INT_RAW, INT_ST, INT_ENA and INT_CLR */
#define INT_RX_START            (1u << 0)
#define INT_TX_START            (1u << 1)
#define INT_RX_HUNG             (1u << 2)
#define INT_TX_HUNG             (1u << 3)
#define INT_IN_DONE             (1u << 4)
#define INT_IN_SUC_EOF          (1u << 5)
#define INT_IN_ERR_EOF          (1u << 6)
#define INT_OUT_DONE            (1u << 7)
#define INT_OUT_EOF             (1u << 8)
#define INT_IN_DSCR_ERR         (1u << 9)
#define INT_OUT_DSCR_ERR        (1u << 10)
#define INT_IN_DSCR_EMPTY       (1u << 11)
#define INT_OUTLINK_EOF_ERR     (1u << 12)
#define INT_OUT_TOTAL_EOF       (1u << 13)
#define INT_SEND_S_REG_Q        (1u << 14)
#define INT_SEND_A_REG_Q        (1u << 15)
#define INT_DMA_INFIFO_FULL_WM  (1u << 16)
#define INT_MASK                0x1ffffu

/* DMA_OUT_STATUS and DMA_IN_STATUS */
#define FIFO_STATUS_FULL        (1u << 0)
#define FIFO_STATUS_EMPTY       (1u << 1)

#define OUT_PUSH_WDATA_MASK     0x1ffu
#define OUT_PUSH_PUSH           (1u << 16)
#define IN_POP_RDATA_MASK       0xfffu
#define IN_POP_POP              (1u << 16)

/* DMA_OUT_LINK and DMA_IN_LINK */
#define LINK_ADDR_MASK          0xfffffu
#define IN_LINK_AUTO_RET        (1u << 20)
#define LINK_STOP               (1u << 28)
#define LINK_START              (1u << 29)
#define LINK_RESTART            (1u << 30)
#define LINK_PARK               (1u << 31)

/* CONF1 */
#define CONF1_CHECK_OWNER       (1u << 6)
#define CONF1_INFIFO_FULL_THRS_SHIFT 9
#define CONF1_INFIFO_FULL_THRS_MASK  0xfffu
#define CONF1_MASK              0x001fffffu
#define CONF1_RESET             0x00000033u

/* ESCAPE_CONF: TX_* decode received data, RX_* encode sent data */
#define ESCAPE_TX_SHIFT         0
#define ESCAPE_RX_SHIFT         4
#define ESCAPE_CONF_MASK        0xffu
#define ESCAPE_CONF_RESET       0x33u

/* HUNG_CONF: TXFIFO_* for receiving (TX_HUNG), RXFIFO_* for sending */
#define HUNG_TX_SHIFT           0
#define HUNG_RX_SHIFT           12
#define HUNG_CONF_MASK          0x00ffffffu
#define HUNG_CONF_RESET         0x00810810u

#define QUICK_SINGLE_NUM(v)     ((v) & 7)
#define QUICK_SINGLE_EN         (1u << 3)
#define QUICK_ALWAYS_NUM(v)     (((v) >> 4) & 7)
#define QUICK_ALWAYS_EN         (1u << 7)
#define QUICK_SENT_MASK         0xffu

#define ESC_CONF_MASK           0x00ffffffu
#define PKT_THRES_MASK          0x1fffu
#define PKT_THRES_RESET         0x80u
#define DATE_RESET              0x16041001u
#define AHB_TEST_MASK           0x37u

/*
 * Out FIFO entries: the byte for the UART in bits 7:0 and the 9th bit of
 * DMA_OUT_PUSH.WDATA in bit 8, plus the model's markers of the events the
 * entry's departure raises.
 */
#define OUTF_BYTE               0xffu
#define OUTF_EOF                (1u << 9)   /* last byte of an EOF frame */
#define OUTF_START              (1u << 10)  /* a frame's opening separator */
#define OUTF_TOTAL              (1u << 11)  /* last byte of the chain */
#define OUTF_QUICK_S            (1u << 12)  /* last byte of a single packet */
#define OUTF_QUICK_A            (1u << 13)  /* last byte of an always packet */

/* In FIFO entries: a received byte, or the end of a frame. */
#define INF_EOF                 (1u << 8)

/*
 * An outlink of empty descriptors makes no data; the engine stops after
 * this many in a row until something else wakes it, rather than spin.
 */
#define OUT_EMPTY_DESC_LIMIT    256

/* Ticks of the hang timeouts: 8000 >> shift APB_CLK cycles */
#define HUNG_TICK_CYCLES        8000

static void uhci_kick(Esp32UhciState *s);

static void fifo_reset(Esp32UhciFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static unsigned fifo_free(Esp32UhciFifo *f)
{
    return ESP32_UHCI_FIFO_DEPTH - f->num;
}

static unsigned fifo_tail(Esp32UhciFifo *f)
{
    return (f->head + f->num - 1) % ESP32_UHCI_FIFO_DEPTH;
}

static void fifo_push(Esp32UhciFifo *f, uint16_t v, uint32_t tag0,
                      uint32_t tag1)
{
    unsigned i = (f->head + f->num) % ESP32_UHCI_FIFO_DEPTH;

    assert(f->num < ESP32_UHCI_FIFO_DEPTH);
    f->data[i] = v;
    f->tag[i][0] = tag0;
    f->tag[i][1] = tag1;
    f->num++;
}

static uint16_t fifo_pop(Esp32UhciFifo *f, uint32_t *tag0, uint32_t *tag1)
{
    uint16_t v = f->data[f->head];

    assert(f->num > 0);
    if (tag0) {
        *tag0 = f->tag[f->head][0];
        *tag1 = f->tag[f->head][1];
    }
    f->head = (f->head + 1) % ESP32_UHCI_FIFO_DEPTH;
    f->num--;
    return v;
}

static bool uhci_running(Esp32UhciState *s)
{
    return clock_is_enabled(s->apb_clk);
}

static void uhci_update_irq(Esp32UhciState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static void uhci_raise(Esp32UhciState *s, uint32_t bits)
{
    s->int_raw |= bits;
}

/* The UARTs this UHCI serves, per CONF0.UARTn_CE */
static uint32_t uhci_uart_mask(Esp32UhciState *s)
{
    uint32_t mask = (s->conf0 >> CONF0_UART_CE_SHIFT) & 7;

    for (int i = 0; i < ESP32_UHCI_UART_COUNT; i++) {
        if (!s->uart[i]) {
            mask &= ~(1u << i);
        }
    }
    return mask;
}

static bool uhci_ldl(Esp32UhciState *s, uint32_t addr, uint32_t *v)
{
    MemTxResult r;

    *v = address_space_ldl_le(&s->dma_as, addr, MEMTXATTRS_UNSPECIFIED, &r);
    return r == MEMTX_OK;
}

static void uhci_stl(Esp32UhciState *s, uint32_t addr, uint32_t v)
{
    address_space_stl_le(&s->dma_as, addr, v, MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Read the descriptor at addr. It must be word aligned and in DMA RAM,
 * and, with CONF1.CHECK_OWNER, owned by the DMA engine.
 */
static bool uhci_load_desc(Esp32UhciState *s, uint32_t addr, uint32_t *dw0,
                           uint32_t *buf, uint32_t *next, const char *dir)
{
    if ((addr & 3) || !esp32_dma_ram_contains(addr, 12)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: %slink descriptor at "
                      "0x%08x is not word-aligned DMA RAM\n", dir, addr);
        return false;
    }
    if (!uhci_ldl(s, addr, dw0) || !uhci_ldl(s, addr + 4, buf) ||
        !uhci_ldl(s, addr + 8, next)) {
        return false;
    }
    if ((s->conf1 & CONF1_CHECK_OWNER) && !(*dw0 & ESP32_LLDESC_OWNER)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: %slink descriptor at "
                      "0x%08x is owned by the CPU\n", dir, addr);
        return false;
    }
    return true;
}

static void uhci_log_undocumented(Esp32UhciState *s)
{
    if (s->conf0 & CONF0_UNDOCUMENTED) {
        qemu_log_mask(LOG_UNIMP, "esp32_uhci: CONF0 0x%08x selects modes "
                      "the TRM leaves reserved (packet head, CRC, length, "
                      "idle or break EOF, loop test, memory transfer); "
                      "running without them\n", s->conf0);
    }
}

/* ---- Outlink ---- */

static void uhci_out_reset(Esp32UhciState *s)
{
    s->out_active = false;
    s->out_parked = false;
    s->out_have_desc = false;
    s->out_in_frame = false;
    s->out_have_last = false;
    s->out_desc_next = 0;
    s->out_empty_run = 0;
    s->quick_len = 0;
    s->quick_pos = 0;
    s->quick_in_frame = false;
    fifo_reset(&s->out_fifo);
    timer_del(&s->out_hung_timer);
    s->out_hung_fired = false;
}

static void uhci_out_park(Esp32UhciState *s)
{
    s->out_active = false;
    s->out_parked = true;
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Bytes the encoder sends for a data byte: the byte, or the escape pair
 * of the first ESC_CONFn whose character it is, if ESCAPE_CONF.RX_*
 * enables that escape.
 */
static unsigned uhci_encode(Esp32UhciState *s, uint8_t b, uint8_t out[2])
{
    uint32_t en = (s->escape_conf >> ESCAPE_RX_SHIFT) & 0xf;

    for (int i = 0; i < 4; i++) {
        uint32_t esc = s->esc_conf[i];

        if ((en & (1u << i)) && b == (esc & 0xff)) {
            out[0] = (esc >> 8) & 0xff;
            out[1] = (esc >> 16) & 0xff;
            return 2;
        }
    }
    out[0] = b;
    return 1;
}

/* Queue an encoded data byte; needs two free entries. */
static void uhci_out_data(Esp32UhciState *s, uint8_t b)
{
    uint8_t enc[2];
    unsigned n = uhci_encode(s, b, enc);

    for (unsigned i = 0; i < n; i++) {
        fifo_push(&s->out_fifo, enc[i], 0, 0);
    }
}

static void uhci_out_sep(Esp32UhciState *s, uint16_t flags)
{
    fifo_push(&s->out_fifo, (s->esc_conf[0] & 0xff) | flags, 0, 0);
}

/*
 * Mark the end of a frame or chain on the last queued byte, so that its
 * event fires when that byte leaves for the UART; with nothing queued it
 * has left already.
 */
static void uhci_out_mark(Esp32UhciState *s, uint16_t flags, uint32_t desc,
                          uint32_t buf)
{
    Esp32UhciFifo *f = &s->out_fifo;

    if (f->num) {
        unsigned t = fifo_tail(f);

        f->data[t] |= flags;
        if (flags & OUTF_EOF) {
            f->tag[t][0] = desc;
            f->tag[t][1] = buf;
        }
        return;
    }
    if (flags & OUTF_EOF) {
        s->out_eof_des_addr = desc;
        s->out_eof_bfr_des_addr = buf;
        uhci_raise(s, INT_OUT_EOF);
    }
    if (flags & OUTF_TOTAL) {
        uhci_raise(s, INT_OUT_TOTAL_EOF);
    }
    if (flags & OUTF_QUICK_S) {
        uhci_raise(s, INT_SEND_S_REG_Q);
    }
    if (flags & OUTF_QUICK_A) {
        uhci_raise(s, INT_SEND_A_REG_Q);
    }
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Load a short packet to be sent as a frame. The TRM does not describe
 * the short-packet format; the model sends Qn's WORD0 then WORD1, eight
 * bytes little-endian, through the encoder like outlink data. A single
 * packet clears SINGLE_SEND_EN once taken; an always packet repeats
 * while ALWAYS_SEND_EN is set and the outlink is idle.
 */
static void uhci_quick_load(Esp32UhciState *s, unsigned n, bool always)
{
    stl_le_p(&s->quick_buf[0], s->q_word[n][0]);
    stl_le_p(&s->quick_buf[4], s->q_word[n][1]);
    s->quick_len = sizeof(s->quick_buf);
    s->quick_pos = 0;
    s->quick_always = always;
    s->quick_in_frame = false;
}

/* One step of a short packet; false if the FIFO lacks room. */
static bool uhci_out_quick_step(Esp32UhciState *s)
{
    bool seper = s->conf0 & CONF0_SEPER_EN;
    uint16_t done = s->quick_always ? OUTF_QUICK_A : OUTF_QUICK_S;

    if (seper && !s->quick_in_frame) {
        if (fifo_free(&s->out_fifo) < 1) {
            return false;
        }
        uhci_out_sep(s, OUTF_START);
        s->quick_in_frame = true;
        return true;
    }
    if (s->quick_pos < s->quick_len) {
        if (fifo_free(&s->out_fifo) < 2) {
            return false;
        }
        uhci_out_data(s, s->quick_buf[s->quick_pos++]);
        return true;
    }
    if (seper) {
        if (fifo_free(&s->out_fifo) < 1) {
            return false;
        }
        uhci_out_sep(s, 0);
    }
    uhci_out_mark(s, done, 0, 0);
    s->quick_len = 0;
    s->quick_in_frame = false;
    return true;
}

static void uhci_out_shift_dscr(Esp32UhciState *s, uint32_t addr)
{
    s->out_dscr[2] = s->out_dscr[1];
    s->out_dscr[1] = s->out_dscr[0];
    s->out_dscr[0] = addr;
}

/* The descriptor's data has all been read: finish it. */
static void uhci_out_finish_desc(Esp32UhciState *s)
{
    uint32_t addr = s->out_desc_addr;
    uint32_t dw0 = s->out_desc_dw0;
    bool eof = dw0 & ESP32_LLDESC_EOF;

    if (s->conf0 & CONF0_OUT_AUTO_WRBACK) {
        uhci_stl(s, addr, dw0 & ~ESP32_LLDESC_OWNER);
    }
    uhci_raise(s, INT_OUT_DONE);
    s->out_have_desc = false;
    s->out_have_last = true;
    s->out_last_addr = addr;

    if (eof) {
        s->out_in_frame = false;
        if (s->conf0 & CONF0_OUT_EOF_MODE) {
            /* EOF once the DMA has popped all the frame's data */
            uhci_out_mark(s, OUTF_EOF, addr, s->out_desc_buf);
        } else {
            /* EOF once the DMA has pushed all the frame's data */
            s->out_eof_des_addr = addr;
            s->out_eof_bfr_des_addr = s->out_desc_buf;
            uhci_raise(s, INT_OUT_EOF);
        }
    }

    if (s->out_desc_next == 0) {
        if (eof) {
            uhci_out_mark(s, OUTF_TOTAL, 0, 0);
        } else {
            /* The list ended inside a frame, with no EOF descriptor. */
            uhci_raise(s, INT_OUTLINK_EOF_ERR);
        }
        uhci_out_park(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * One step of the outlink; false if it cannot go on now. The TRM's and
 * ESP-IDF's descriptions of OUT_DSCR_ERR and IN_DSCR_ERR name the
 * opposite link; the model follows the bit names, as for every other
 * IN_ and OUT_ interrupt.
 */
static bool uhci_out_desc_step(Esp32UhciState *s)
{
    bool seper = s->conf0 & CONF0_SEPER_EN;
    uint32_t length;

    if (!s->out_have_desc) {
        uint32_t addr = s->out_desc_next;
        uint32_t dw0, buf, next;

        if (!uhci_load_desc(s, addr, &dw0, &buf, &next, "out") ||
            (ESP32_LLDESC_LENGTH(dw0) &&
             !esp32_dma_ram_contains(buf, ESP32_LLDESC_LENGTH(dw0)))) {
            if (esp32_dma_ram_contains(addr, 12)) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: outlink "
                              "descriptor at 0x%08x is invalid\n", addr);
            }
            uhci_raise(s, INT_OUT_DSCR_ERR);
            uhci_out_park(s);
            return false;
        }
        uhci_out_shift_dscr(s, addr);
        s->out_desc_addr = addr;
        s->out_desc_dw0 = dw0;
        s->out_desc_buf = buf;
        s->out_desc_next = next;
        s->out_pos = 0;
        s->out_have_desc = true;
    }

    length = ESP32_LLDESC_LENGTH(s->out_desc_dw0);
    if (seper && !s->out_in_frame &&
        (s->out_pos < length || (s->out_desc_dw0 & ESP32_LLDESC_EOF))) {
        if (fifo_free(&s->out_fifo) < 1) {
            return false;
        }
        uhci_out_sep(s, OUTF_START);
        s->out_in_frame = true;
        return true;
    }
    if (s->out_pos < length) {
        MemTxResult r;
        uint8_t b;

        if (fifo_free(&s->out_fifo) < 2) {
            return false;
        }
        b = address_space_ldub(&s->dma_as, s->out_desc_buf + s->out_pos,
                               MEMTXATTRS_UNSPECIFIED, &r);
        s->out_pos++;
        uhci_out_data(s, b);
        s->out_empty_run = 0;
        return true;
    }
    if ((s->out_desc_dw0 & ESP32_LLDESC_EOF) && seper) {
        if (fifo_free(&s->out_fifo) < 1) {
            return false;
        }
        uhci_out_sep(s, 0);
    }
    if (length == 0 && ++s->out_empty_run > OUT_EMPTY_DESC_LIMIT) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: outlink runs through "
                      "empty descriptors without end\n");
        s->out_empty_run = 0;
        uhci_out_finish_desc(s);
        return false;
    }
    uhci_out_finish_desc(s);
    return true;
}

/* Fill the out FIFO by one step; false if nothing could be done. */
static bool uhci_out_fill(Esp32UhciState *s)
{
    if (s->quick_len) {
        return uhci_out_quick_step(s);
    }
    if (!s->out_in_frame && (s->quick_sent & QUICK_SINGLE_EN)) {
        unsigned n = QUICK_SINGLE_NUM(s->quick_sent);

        s->quick_sent &= ~QUICK_SINGLE_EN;
        if (n < ESP32_UHCI_QUICK_COUNT) {
            uhci_quick_load(s, n, false);
            return true;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: no short packet Q%u\n",
                      n);
    }
    if (s->out_active) {
        return uhci_out_desc_step(s);
    }
    if (!s->out_in_frame && (s->quick_sent & QUICK_ALWAYS_EN)) {
        unsigned n = QUICK_ALWAYS_NUM(s->quick_sent);

        if (n < ESP32_UHCI_QUICK_COUNT) {
            uhci_quick_load(s, n, true);
            return true;
        }
    }
    return false;
}

static bool uhci_uarts_ready(Esp32UhciState *s, uint32_t mask)
{
    if (!mask) {
        return false;
    }
    for (int i = 0; i < ESP32_UHCI_UART_COUNT; i++) {
        if ((mask & (1u << i)) && !esp32_uart_dma_tx_free(s->uart[i])) {
            return false;
        }
    }
    return true;
}

/* Move one entry from the out FIFO to the UART TX FIFOs. */
static void uhci_out_drain_one(Esp32UhciState *s, uint32_t mask)
{
    uint32_t desc, buf;
    uint16_t e = fifo_pop(&s->out_fifo, &desc, &buf);

    for (int i = 0; i < ESP32_UHCI_UART_COUNT; i++) {
        if (mask & (1u << i)) {
            esp32_uart_dma_tx_push(s->uart[i], e & OUTF_BYTE);
        }
    }
    if (e & OUTF_START) {
        uhci_raise(s, INT_RX_START);
    }
    if (e & OUTF_EOF) {
        s->out_eof_des_addr = desc;
        s->out_eof_bfr_des_addr = buf;
        uhci_raise(s, INT_OUT_EOF);
    }
    if (e & OUTF_TOTAL) {
        uhci_raise(s, INT_OUT_TOTAL_EOF);
    }
    if (e & OUTF_QUICK_S) {
        uhci_raise(s, INT_SEND_S_REG_Q);
    }
    if (e & OUTF_QUICK_A) {
        uhci_raise(s, INT_SEND_A_REG_Q);
    }
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Run the send side as far as the UARTs take data; true on progress.
 * With more than one UART selected, each byte goes to all of them.
 */
static bool uhci_out_run(Esp32UhciState *s)
{
    uint32_t mask = uhci_uart_mask(s);
    bool progress = false;

    if (s->conf0 & (CONF0_OUT_RST | CONF0_AHBM_RST)) {
        return false;
    }
    for (;;) {
        bool step = false;

        while (s->out_fifo.num && uhci_uarts_ready(s, mask)) {
            uhci_out_drain_one(s, mask);
            step = true;
        }
        while (fifo_free(&s->out_fifo) >= 2 && uhci_out_fill(s)) {
            step = true;
        }
        if (!step) {
            break;
        }
        progress = true;
    }
    return progress;
}

/* ---- Inlink ---- */

static void uhci_in_reset(Esp32UhciState *s)
{
    s->in_active = false;
    s->in_parked = false;
    s->in_have_desc = false;
    s->in_have_last = false;
    s->in_desc_next = 0;
    s->in_empty_flagged = false;
    s->in_frame = false;
    s->in_frame_data = false;
    s->in_esc = false;
    fifo_reset(&s->in_fifo);
    timer_del(&s->in_hung_timer);
    s->in_hung_fired = false;
}

static void uhci_in_park(Esp32UhciState *s)
{
    s->in_active = false;
    s->in_parked = true;
}

static void uhci_in_queue(Esp32UhciState *s, uint16_t v)
{
    uint32_t thrs = (s->conf1 >> CONF1_INFIFO_FULL_THRS_SHIFT) &
                    CONF1_INFIFO_FULL_THRS_MASK;

    fifo_push(&s->in_fifo, v, 0, 0);
    if (s->in_fifo.num > thrs) {
        uhci_raise(s, INT_DMA_INFIFO_FULL_WM);
    }
    if (!(v & INF_EOF)) {
        s->in_frame_data = true;
    }
}

/* The byte an escape pair (lead, b) stands for, or -1 */
static int uhci_unescape(Esp32UhciState *s, uint8_t lead, uint8_t b)
{
    uint32_t en = (s->escape_conf >> ESCAPE_TX_SHIFT) & 0xf;

    for (int i = 0; i < 4; i++) {
        uint32_t esc = s->esc_conf[i];

        if ((en & (1u << i)) && lead == ((esc >> 8) & 0xff) &&
            b == ((esc >> 16) & 0xff)) {
            return esc & 0xff;
        }
    }
    return -1;
}

static bool uhci_is_esc_lead(Esp32UhciState *s, uint8_t b)
{
    uint32_t en = (s->escape_conf >> ESCAPE_TX_SHIFT) & 0xf;

    for (int i = 0; i < 4; i++) {
        if ((en & (1u << i)) && b == ((s->esc_conf[i] >> 8) & 0xff)) {
            return true;
        }
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Decode a received byte into the in FIFO, which has room for two
 * entries. With SEPER_EN, a separator opens a frame, more separators
 * before any data are skipped, and the next separator after data closes
 * the frame; bytes outside a frame are dropped. An escape lead with a
 * byte that completes no enabled pair passes through unchanged.
 */
static void uhci_decode(Esp32UhciState *s, uint8_t b)
{
    bool seper = s->conf0 & CONF0_SEPER_EN;

    if (seper && b == (s->esc_conf[0] & 0xff)) {
        if (s->in_esc) {
            uhci_in_queue(s, s->in_esc_char);
            s->in_esc = false;
        }
        if (!s->in_frame) {
            s->in_frame = true;
            s->in_frame_data = false;
            uhci_raise(s, INT_TX_START);
        } else if (s->in_frame_data) {
            uhci_in_queue(s, INF_EOF);
            s->in_frame = false;
            s->in_frame_data = false;
        }
        return;
    }
    if (seper && !s->in_frame) {
        return;
    }
    if (s->in_esc) {
        int dec = uhci_unescape(s, s->in_esc_char, b);

        s->in_esc = false;
        if (dec >= 0) {
            uhci_in_queue(s, dec);
            return;
        }
        uhci_in_queue(s, s->in_esc_char);
    }
    if (uhci_is_esc_lead(s, b)) {
        s->in_esc = true;
        s->in_esc_char = b;
        return;
    }
    uhci_in_queue(s, b);
}

static void uhci_in_shift_dscr(Esp32UhciState *s, uint32_t addr)
{
    s->in_dscr[2] = s->in_dscr[1];
    s->in_dscr[1] = s->in_dscr[0];
    s->in_dscr[0] = addr;
}

/* Hand the descriptor back with in_len bytes, and go on to the next. */
static void uhci_in_complete(Esp32UhciState *s, bool eof)
{
    uint32_t dw0 = s->in_desc_dw0 & ~(ESP32_LLDESC_OWNER | ESP32_LLDESC_EOF |
                                      (0xfffu << ESP32_LLDESC_LENGTH_SHIFT));

    dw0 |= s->in_len << ESP32_LLDESC_LENGTH_SHIFT;
    if (eof) {
        dw0 |= ESP32_LLDESC_EOF;
    }
    uhci_stl(s, s->in_desc_addr, dw0);
    uhci_raise(s, INT_IN_DONE);
    if (eof) {
        s->in_suc_eof_des_addr = s->in_desc_addr;
        uhci_raise(s, INT_IN_SUC_EOF);
    }
    s->in_have_desc = false;
    s->in_have_last = true;
    s->in_last_addr = s->in_desc_addr;
}

/* Take the next inlink descriptor; false if there is none to take. */
static bool uhci_in_load(Esp32UhciState *s)
{
    uint32_t addr = s->in_desc_next;
    uint32_t dw0, buf, next;

    if (addr == 0) {
        /* The list has run out: the DMA has nowhere to put the data. */
        if (!s->in_empty_flagged) {
            s->in_empty_flagged = true;
            uhci_raise(s, INT_IN_DSCR_EMPTY);
        }
        return false;
    }
    if (!uhci_load_desc(s, addr, &dw0, &buf, &next, "in") ||
        ESP32_LLDESC_SIZE(dw0) == 0 || (buf & 3) ||
        !esp32_dma_ram_contains(buf, ESP32_LLDESC_SIZE(dw0))) {
        if (esp32_dma_ram_contains(addr, 12)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: inlink descriptor "
                          "at 0x%08x is invalid: its buffer must be a "
                          "non-empty, word-aligned block of DMA RAM\n", addr);
        }
        uhci_raise(s, INT_IN_DSCR_ERR);
        uhci_in_park(s);
        return false;
    }
    uhci_in_shift_dscr(s, addr);
    s->in_desc_addr = addr;
    s->in_desc_dw0 = dw0;
    s->in_desc_buf = buf;
    s->in_desc_next = next;
    s->in_len = 0;
    s->in_have_desc = true;
    s->in_empty_flagged = false;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Store the head of the in FIFO; false if it must wait. A full
 * descriptor is handed back only when the next entry shows whether the
 * frame ended with it.
 */
static bool uhci_in_store(Esp32UhciState *s, uint16_t e)
{
    if (e & INF_EOF) {
        if (s->in_have_desc) {
            uhci_in_complete(s, true);
        }
        return true;
    }
    if (s->in_have_desc && s->in_len == ESP32_LLDESC_SIZE(s->in_desc_dw0)) {
        uhci_in_complete(s, false);
    }
    if (!s->in_have_desc && !uhci_in_load(s)) {
        return false;
    }
    address_space_stb(&s->dma_as, s->in_desc_buf + s->in_len, e & 0xff,
                      MEMTXATTRS_UNSPECIFIED, NULL);
    s->in_len++;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Run the receive side; true on progress. The UHCI takes bytes from the
 * UART RX FIFO only while its inlink runs, so a UART it does not serve,
 * or serves with the inlink stopped, keeps them for software.
 */
static bool uhci_in_run(Esp32UhciState *s)
{
    uint32_t mask = uhci_uart_mask(s);
    bool progress = false;
    bool step;

    if (s->conf0 & (CONF0_IN_RST | CONF0_AHBM_RST)) {
        return false;
    }
    do {
        step = false;
        while (s->in_active && s->in_fifo.num &&
               uhci_in_store(s, s->in_fifo.data[s->in_fifo.head])) {
            fifo_pop(&s->in_fifo, NULL, NULL);
            step = true;
        }
        if (!s->in_active) {
            break;
        }
        for (int i = 0; i < ESP32_UHCI_UART_COUNT; i++) {
            if (!(mask & (1u << i))) {
                continue;
            }
            while (fifo_free(&s->in_fifo) >= 2 &&
                   esp32_uart_dma_rx_count(s->uart[i])) {
                uhci_decode(s, esp32_uart_dma_rx_pop(s->uart[i]));
                step = true;
            }
        }
        progress |= step;
    } while (step);
    return progress;
}

/* ---- Hang timeouts ---- */

static int64_t uhci_hung_ns(Esp32UhciState *s, unsigned shift)
{
    uint32_t f = s->hung_conf >> shift;
    uint64_t cycles = (uint64_t)(f & 0xff) *
                      (HUNG_TICK_CYCLES >> ((f >> 8) & 7));

    return clock_ticks_to_ns(s->apb_clk, cycles);
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Arm the timeout while the engine holds data it cannot move: the send
 * side waiting for room in the UART, the receive side for a descriptor.
 * It fires once per stall.
 */
static void uhci_hung_update(Esp32UhciState *s, QEMUTimer *t, bool *fired,
                             bool stalled, bool progress, unsigned shift)
{
    bool ena = (s->hung_conf >> shift) & (1u << 11);

    if (!stalled || !ena || !uhci_running(s)) {
        timer_del(t);
        *fired = false;
        return;
    }
    if (progress) {
        timer_del(t);
        *fired = false;
    }
    if (!*fired && !timer_pending(t)) {
        timer_mod_ns(t, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     uhci_hung_ns(s, shift));
    }
}

static void uhci_out_hung_cb(void *opaque)
{
    Esp32UhciState *s = opaque;

    s->out_hung_fired = true;
    uhci_raise(s, INT_RX_HUNG);
    uhci_update_irq(s);
}

static void uhci_in_hung_cb(void *opaque)
{
    Esp32UhciState *s = opaque;

    s->in_hung_fired = true;
    uhci_raise(s, INT_TX_HUNG);
    uhci_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Move data as far as the UARTs, the FIFOs and the descriptor lists
 * allow. Called on register writes and whenever an attached UART frees
 * TX room or receives data; a call from within a call repeats the run.
 */
static void uhci_kick(Esp32UhciState *s)
{
    bool out_progress = false, in_progress = false;

    if (s->in_kick) {
        s->kick_again = true;
        return;
    }
    s->in_kick = true;
    if (uhci_running(s)) {
        do {
            s->kick_again = false;
            out_progress |= uhci_out_run(s);
            in_progress |= uhci_in_run(s);
        } while (s->kick_again);
    }
    s->in_kick = false;

    uhci_hung_update(s, &s->out_hung_timer, &s->out_hung_fired,
                     s->out_fifo.num != 0, out_progress, HUNG_RX_SHIFT);
    uhci_hung_update(s, &s->in_hung_timer, &s->in_hung_fired,
                     s->in_fifo.num != 0, in_progress, HUNG_TX_SHIFT);
    uhci_update_irq(s);
}

static void uhci_uart_notify(Notifier *n, void *data)
{
    Esp32UhciUartLink *link = container_of(n, Esp32UhciUartLink, notifier);

    uhci_kick(link->uhci);
}

/* ---- Registers ---- */

/* [spec:nuos:req:emu.esp32.uhci] */
static void uhci_link_write(Esp32UhciState *s, bool out, uint32_t value)
{
    uint32_t *reg = out ? &s->dma_out_link : &s->dma_in_link;
    uint32_t keep = out ? LINK_ADDR_MASK : LINK_ADDR_MASK | IN_LINK_AUTO_RET;
    uint32_t first = ESP32_DMA_LINK_BASE | (value & LINK_ADDR_MASK);

    /* STOP, START and RESTART act when written as 1 and read as 0. */
    *reg = value & keep;
    if (value & LINK_STOP) {
        if (out) {
            if (s->out_active) {
                uhci_out_park(s);
            }
        } else if (s->in_active) {
            uhci_in_park(s);
        }
    }
    if (value & LINK_START) {
        uhci_log_undocumented(s);
        if (out) {
            s->out_have_desc = false;
            s->out_in_frame = false;
            s->out_desc_next = first;
            s->out_empty_run = 0;
            s->out_active = true;
            s->out_parked = false;
        } else {
            s->in_have_desc = false;
            s->in_frame = false;
            s->in_frame_data = false;
            s->in_esc = false;
            s->in_empty_flagged = false;
            s->in_desc_next = first;
            s->in_active = true;
            s->in_parked = false;
        }
    }
    if (value & LINK_RESTART) {
        /*
         * Go on from where the list stopped: within the descriptor in
         * hand, or at the next field of the last one finished, which
         * software may have pointed at descriptors added since.
         */
        uint32_t next;

        if (out && !s->out_active) {
            if (s->out_have_desc) {
                s->out_active = true;
            } else if (s->out_have_last &&
                       uhci_ldl(s, s->out_last_addr + 8, &next) && next) {
                s->out_desc_next = next;
                s->out_active = true;
            }
            if (s->out_active) {
                s->out_parked = false;
                s->out_empty_run = 0;
            }
        } else if (!out && !s->in_active) {
            if (s->in_have_desc) {
                s->in_active = true;
            } else if (s->in_have_last &&
                       uhci_ldl(s, s->in_last_addr + 8, &next) && next) {
                s->in_desc_next = next;
                s->in_active = true;
            }
            if (s->in_active) {
                s->in_parked = false;
            }
        } else if (!out && s->in_active && !s->in_have_desc &&
                   s->in_desc_next == 0 && s->in_have_last &&
                   uhci_ldl(s, s->in_last_addr + 8, &next)) {
            /* Running out of descriptors: mount those now linked on. */
            s->in_desc_next = next;
        }
    }
}

/* [spec:nuos:req:emu.esp32.uhci] */
static uint64_t esp32_uhci_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32UhciState *s = opaque;

    switch (addr) {
    case A_CONF0:
        return s->conf0;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_DMA_OUT_STATUS:
        return (s->out_fifo.num == ESP32_UHCI_FIFO_DEPTH ?
                FIFO_STATUS_FULL : 0) |
               (s->out_fifo.num == 0 ? FIFO_STATUS_EMPTY : 0);
    case A_DMA_OUT_PUSH:
        return s->dma_out_push;
    case A_DMA_IN_STATUS:
        return (s->in_fifo.num == ESP32_UHCI_FIFO_DEPTH ?
                FIFO_STATUS_FULL : 0) |
               (s->in_fifo.num == 0 ? FIFO_STATUS_EMPTY : 0);
    case A_DMA_IN_POP:
        return s->dma_in_pop;
    case A_DMA_OUT_LINK:
        return s->dma_out_link | (s->out_parked ? LINK_PARK : 0);
    case A_DMA_IN_LINK:
        return s->dma_in_link | (s->in_parked ? LINK_PARK : 0);
    case A_CONF1:
        return s->conf1;
    case A_STATE0:
    case A_STATE1:
        /* Internal state-machine debug words, not documented */
        return 0;
    case A_OUT_EOF_DES_ADDR:
        return s->out_eof_des_addr;
    case A_IN_SUC_EOF_DES_ADDR:
        return s->in_suc_eof_des_addr;
    case A_IN_ERR_EOF_DES_ADDR:
        return s->in_err_eof_des_addr;
    case A_OUT_EOF_BFR_DES_ADDR:
        return s->out_eof_bfr_des_addr;
    case A_AHB_TEST:
        return s->ahb_test;
    case A_IN_DSCR ... A_IN_DSCR_BF1:
        return s->in_dscr[(addr - A_IN_DSCR) / 4];
    case A_OUT_DSCR ... A_OUT_DSCR_BF1:
        return s->out_dscr[(addr - A_OUT_DSCR) / 4];
    case A_ESCAPE_CONF:
        return s->escape_conf;
    case A_HUNG_CONF:
        return s->hung_conf;
    case A_ACK_NUM:
        return s->ack_num;
    case A_RX_HEAD:
        /* Packet heads are not received: HEAD_EN is not modelled. */
        return 0;
    case A_QUICK_SENT:
        return s->quick_sent;
    case A_Q0_WORD0 ... A_Q6_WORD1:
        return s->q_word[(addr - A_Q0_WORD0) / 8][((addr - A_Q0_WORD0) / 4)
                                                  & 1];
    case A_ESC_CONF0 ... A_ESC_CONF3:
        return s->esc_conf[(addr - A_ESC_CONF0) / 4];
    case A_PKT_THRES:
        return s->pkt_thres;
    case A_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: read of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.uhci] */
static void esp32_uhci_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    Esp32UhciState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case A_CONF0:
        s->conf0 = v & CONF0_MASK;
        /* The reset bits hold their part in reset while set. */
        if (v & (CONF0_OUT_RST | CONF0_AHBM_RST)) {
            uhci_out_reset(s);
        }
        if (v & (CONF0_IN_RST | CONF0_AHBM_RST)) {
            uhci_in_reset(s);
        }
        if (v & CONF0_AHBM_FIFO_RST) {
            fifo_reset(&s->out_fifo);
            fifo_reset(&s->in_fifo);
        }
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_MASK;
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_MASK);
        break;
    case A_DMA_OUT_PUSH:
        s->dma_out_push = v & OUT_PUSH_WDATA_MASK;
        if (v & OUT_PUSH_PUSH) {
            if (fifo_free(&s->out_fifo)) {
                fifo_push(&s->out_fifo, v & OUT_PUSH_WDATA_MASK, 0, 0);
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: push to a full "
                              "out FIFO\n");
            }
        }
        break;
    case A_DMA_IN_POP:
        if (v & IN_POP_POP) {
            if (s->in_fifo.num) {
                s->dma_in_pop = fifo_pop(&s->in_fifo, NULL, NULL) &
                                IN_POP_RDATA_MASK;
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: pop from an "
                              "empty in FIFO\n");
            }
        }
        break;
    case A_DMA_OUT_LINK:
        uhci_link_write(s, true, v);
        break;
    case A_DMA_IN_LINK:
        uhci_link_write(s, false, v);
        break;
    case A_CONF1:
        s->conf1 = v & CONF1_MASK;
        break;
    case A_AHB_TEST:
        s->ahb_test = v & AHB_TEST_MASK;
        if (s->ahb_test & 4) {
            qemu_log_mask(LOG_UNIMP, "esp32_uhci: AHB bus test mode not "
                          "modelled\n");
        }
        break;
    case A_ESCAPE_CONF:
        s->escape_conf = v & ESCAPE_CONF_MASK;
        break;
    case A_HUNG_CONF:
        s->hung_conf = v & HUNG_CONF_MASK;
        timer_del(&s->out_hung_timer);
        timer_del(&s->in_hung_timer);
        break;
    case A_ACK_NUM:
        s->ack_num = v;
        break;
    case A_QUICK_SENT:
        s->quick_sent = v & QUICK_SENT_MASK;
        break;
    case A_Q0_WORD0 ... A_Q6_WORD1:
        s->q_word[(addr - A_Q0_WORD0) / 8][((addr - A_Q0_WORD0) / 4) & 1] = v;
        break;
    case A_ESC_CONF0 ... A_ESC_CONF3:
        s->esc_conf[(addr - A_ESC_CONF0) / 4] = v & ESC_CONF_MASK;
        break;
    case A_PKT_THRES:
        s->pkt_thres = v & PKT_THRES_MASK;
        break;
    case A_DATE:
        s->date = v;
        break;
    case A_INT_RAW:
    case A_INT_ST:
    case A_DMA_OUT_STATUS:
    case A_DMA_IN_STATUS:
    case A_STATE0:
    case A_STATE1:
    case A_OUT_EOF_DES_ADDR:
    case A_IN_SUC_EOF_DES_ADDR:
    case A_IN_ERR_EOF_DES_ADDR:
    case A_OUT_EOF_BFR_DES_ADDR:
    case A_IN_DSCR ... A_IN_DSCR_BF1:
    case A_OUT_DSCR ... A_OUT_DSCR_BF1:
    case A_RX_HEAD:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_uhci: write of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    uhci_kick(s);
}

static const MemoryRegionOps esp32_uhci_ops = {
    .read = esp32_uhci_read,
    .write = esp32_uhci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32.uhci] */
static void esp32_uhci_reset_hold(Object *obj, ResetType type)
{
    Esp32UhciState *s = ESP32_UHCI(obj);

    s->conf0 = CONF0_RESET;
    s->int_raw = 0;
    s->int_ena = 0;
    s->dma_out_push = 0;
    s->dma_in_pop = 0;
    s->dma_out_link = 0;
    s->dma_in_link = IN_LINK_AUTO_RET;
    s->conf1 = CONF1_RESET;
    s->out_eof_des_addr = 0;
    s->in_suc_eof_des_addr = 0;
    s->in_err_eof_des_addr = 0;
    s->out_eof_bfr_des_addr = 0;
    s->ahb_test = 0;
    memset(s->in_dscr, 0, sizeof(s->in_dscr));
    memset(s->out_dscr, 0, sizeof(s->out_dscr));
    s->escape_conf = ESCAPE_CONF_RESET;
    s->hung_conf = HUNG_CONF_RESET;
    s->ack_num = 0;
    s->quick_sent = 0;
    memset(s->q_word, 0, sizeof(s->q_word));
    s->esc_conf[0] = 0x00dcdbc0;
    s->esc_conf[1] = 0x00dddbdb;
    s->esc_conf[2] = 0x00dedb11;
    s->esc_conf[3] = 0x00dfdb13;
    s->pkt_thres = PKT_THRES_RESET;
    s->date = DATE_RESET;
    uhci_out_reset(s);
    uhci_in_reset(s);
    s->kick_again = false;
    qemu_irq_lower(s->irq);
}

static void esp32_uhci_clk_update(void *opaque, ClockEvent event)
{
    Esp32UhciState *s = opaque;

    if (!uhci_running(s)) {
        timer_del(&s->out_hung_timer);
        timer_del(&s->in_hung_timer);
        return;
    }
    uhci_kick(s);
}

static void esp32_uhci_init(Object *obj)
{
    Esp32UhciState *s = ESP32_UHCI(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_uhci_ops, s,
                          TYPE_ESP32_UHCI, ESP32_UHCI_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", esp32_uhci_clk_update,
                                    s, ClockUpdate);
    timer_init_ns(&s->out_hung_timer, QEMU_CLOCK_VIRTUAL, uhci_out_hung_cb, s);
    timer_init_ns(&s->in_hung_timer, QEMU_CLOCK_VIRTUAL, uhci_in_hung_cb, s);
}

static void esp32_uhci_realize(DeviceState *dev, Error **errp)
{
    Esp32UhciState *s = ESP32_UHCI(dev);

    if (!s->dma_mr) {
        error_setg(errp, "esp32_uhci: 'dma-mr' link not set");
        return;
    }
    address_space_init(&s->dma_as, s->dma_mr, "esp32-uhci-dma");
    for (int i = 0; i < ESP32_UHCI_UART_COUNT; i++) {
        if (s->uart[i]) {
            s->uart_link[i].uhci = s;
            s->uart_link[i].notifier.notify = uhci_uart_notify;
            esp32_uart_add_dma_notifier(s->uart[i], &s->uart_link[i].notifier);
        }
    }
}

static const Property esp32_uhci_properties[] = {
    DEFINE_PROP_LINK("dma-mr", Esp32UhciState, dma_mr, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("uart0", Esp32UhciState, uart[0], TYPE_ESP32_UART,
                     ESP32UARTState *),
    DEFINE_PROP_LINK("uart1", Esp32UhciState, uart[1], TYPE_ESP32_UART,
                     ESP32UARTState *),
    DEFINE_PROP_LINK("uart2", Esp32UhciState, uart[2], TYPE_ESP32_UART,
                     ESP32UARTState *),
};

static const VMStateDescription vmstate_esp32_uhci_fifo = {
    .name = "esp32-uhci-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(data, Esp32UhciFifo, ESP32_UHCI_FIFO_DEPTH),
        VMSTATE_UINT32_2DARRAY(tag, Esp32UhciFifo, ESP32_UHCI_FIFO_DEPTH, 2),
        VMSTATE_UINT32(head, Esp32UhciFifo),
        VMSTATE_UINT32(num, Esp32UhciFifo),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_uhci = {
    .name = TYPE_ESP32_UHCI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, Esp32UhciState),
        VMSTATE_UINT32(int_raw, Esp32UhciState),
        VMSTATE_UINT32(int_ena, Esp32UhciState),
        VMSTATE_UINT32(dma_out_push, Esp32UhciState),
        VMSTATE_UINT32(dma_in_pop, Esp32UhciState),
        VMSTATE_UINT32(dma_out_link, Esp32UhciState),
        VMSTATE_UINT32(dma_in_link, Esp32UhciState),
        VMSTATE_UINT32(conf1, Esp32UhciState),
        VMSTATE_UINT32(out_eof_des_addr, Esp32UhciState),
        VMSTATE_UINT32(in_suc_eof_des_addr, Esp32UhciState),
        VMSTATE_UINT32(in_err_eof_des_addr, Esp32UhciState),
        VMSTATE_UINT32(out_eof_bfr_des_addr, Esp32UhciState),
        VMSTATE_UINT32(ahb_test, Esp32UhciState),
        VMSTATE_UINT32_ARRAY(in_dscr, Esp32UhciState, 3),
        VMSTATE_UINT32_ARRAY(out_dscr, Esp32UhciState, 3),
        VMSTATE_UINT32(escape_conf, Esp32UhciState),
        VMSTATE_UINT32(hung_conf, Esp32UhciState),
        VMSTATE_UINT32(ack_num, Esp32UhciState),
        VMSTATE_UINT32(quick_sent, Esp32UhciState),
        VMSTATE_UINT32_2DARRAY(q_word, Esp32UhciState,
                               ESP32_UHCI_QUICK_COUNT, 2),
        VMSTATE_UINT32_ARRAY(esc_conf, Esp32UhciState, 4),
        VMSTATE_UINT32(pkt_thres, Esp32UhciState),
        VMSTATE_UINT32(date, Esp32UhciState),
        VMSTATE_BOOL(out_active, Esp32UhciState),
        VMSTATE_BOOL(out_parked, Esp32UhciState),
        VMSTATE_BOOL(out_have_desc, Esp32UhciState),
        VMSTATE_BOOL(out_in_frame, Esp32UhciState),
        VMSTATE_UINT32(out_desc_addr, Esp32UhciState),
        VMSTATE_UINT32(out_desc_dw0, Esp32UhciState),
        VMSTATE_UINT32(out_desc_buf, Esp32UhciState),
        VMSTATE_UINT32(out_desc_next, Esp32UhciState),
        VMSTATE_UINT32(out_pos, Esp32UhciState),
        VMSTATE_UINT32(out_last_addr, Esp32UhciState),
        VMSTATE_BOOL(out_have_last, Esp32UhciState),
        VMSTATE_UINT32(out_empty_run, Esp32UhciState),
        VMSTATE_STRUCT(out_fifo, Esp32UhciState, 1, vmstate_esp32_uhci_fifo,
                       Esp32UhciFifo),
        VMSTATE_UINT8_ARRAY(quick_buf, Esp32UhciState, 8),
        VMSTATE_UINT32(quick_pos, Esp32UhciState),
        VMSTATE_UINT32(quick_len, Esp32UhciState),
        VMSTATE_BOOL(quick_always, Esp32UhciState),
        VMSTATE_BOOL(quick_in_frame, Esp32UhciState),
        VMSTATE_BOOL(in_active, Esp32UhciState),
        VMSTATE_BOOL(in_parked, Esp32UhciState),
        VMSTATE_BOOL(in_have_desc, Esp32UhciState),
        VMSTATE_BOOL(in_empty_flagged, Esp32UhciState),
        VMSTATE_UINT32(in_desc_addr, Esp32UhciState),
        VMSTATE_UINT32(in_desc_dw0, Esp32UhciState),
        VMSTATE_UINT32(in_desc_buf, Esp32UhciState),
        VMSTATE_UINT32(in_desc_next, Esp32UhciState),
        VMSTATE_UINT32(in_len, Esp32UhciState),
        VMSTATE_UINT32(in_last_addr, Esp32UhciState),
        VMSTATE_BOOL(in_have_last, Esp32UhciState),
        VMSTATE_BOOL(in_frame, Esp32UhciState),
        VMSTATE_BOOL(in_frame_data, Esp32UhciState),
        VMSTATE_BOOL(in_esc, Esp32UhciState),
        VMSTATE_UINT8(in_esc_char, Esp32UhciState),
        VMSTATE_STRUCT(in_fifo, Esp32UhciState, 1, vmstate_esp32_uhci_fifo,
                       Esp32UhciFifo),
        VMSTATE_TIMER(out_hung_timer, Esp32UhciState),
        VMSTATE_TIMER(in_hung_timer, Esp32UhciState),
        VMSTATE_BOOL(out_hung_fired, Esp32UhciState),
        VMSTATE_BOOL(in_hung_fired, Esp32UhciState),
        VMSTATE_CLOCK(apb_clk, Esp32UhciState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_uhci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_uhci_reset_hold;
    dc->realize = esp32_uhci_realize;
    dc->vmsd = &vmstate_esp32_uhci;
    device_class_set_props(dc, esp32_uhci_properties);
}

/* [spec:nuos:req:emu.esp32.uhci] */
static const TypeInfo esp32_uhci_info = {
    .name = TYPE_ESP32_UHCI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32UhciState),
    .instance_init = esp32_uhci_init,
    .class_init = esp32_uhci_class_init,
};

static void esp32_uhci_register_types(void)
{
    type_register_static(&esp32_uhci_info);
}

type_init(esp32_uhci_register_types)
