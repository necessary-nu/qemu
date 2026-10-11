/*
 * ESP32-P4 UHCI, the UARTs' interface to GDMA
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM 45.4.10 (GDMA Mode) and the UHCI registers of
 * 45.7.3, with ESP-IDF's rev3 uhci_reg.h and its uhci_ll.h and uhci.c
 * driver for how software uses the block.
 *
 * The five UARTs share one GDMA-AHB channel pair through UHCI0, the UART
 * that CONF0.UART_SEL names (0 to 4; 5 to 7, the reset value 7, none).
 * Sending, the UHCI takes the bytes of the TX channel's outlink as the
 * UART's TX FIFO has room for them, through the encoder: with SEPER_EN a
 * separator (SEPER_CHAR) opens each frame and closes it at the EOF
 * descriptor's last byte, and data bytes that ESCAPE_CONF's RX_*_ESC_EN
 * name are replaced by their escape pair (ESC_CONFn, as in the ESP32
 * model, hw/dma/esp32_uhci.c). Receiving, it takes bytes from the UART's
 * RX FIFO as the RX channel has room, through the decoder: separators
 * delimit frames, escape pairs that TX_*_ESC_EN name are resolved, and a
 * frame ends, closing the inlink descriptor with suc_eof, at its closing
 * separator, after PKT_THRS bytes with LEN_EOF_EN, when the UART has been
 * idle for RX_IDLE_THRHD bit times with UART_IDLE_EOF_EN, or at a NULL
 * character with UART_RX_BRK_EOF_EN.
 *
 * The HCI packet format (HEAD_EN, the CRC, checksum and sequence number
 * checks, ACK numbers) is not described by the TRM and is not modelled:
 * it is logged, and the UHCI moves data as with HEAD_EN clear. Short
 * packets (QUICK_SENT) send Qn's WORD0 then WORD1 little-endian through
 * the encoder as a frame, as the ESP32 model has them. The hang timeouts
 * count from the moment the UHCI holds data it cannot move, in ticks of
 * 1 ms >> TIMEOUT_SHIFT (TRM: "Measurement unit: ms").
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32p4_uhci.h"
#include "migration/vmstate.h"

#define A_CONF0             0x00
#define A_INT_RAW           0x04
#define A_INT_ST            0x08
#define A_INT_ENA           0x0c
#define A_INT_CLR           0x10
#define A_CONF1             0x14
#define A_STATE0            0x18
#define A_STATE1            0x1c
#define A_ESCAPE_CONF       0x20
#define A_HUNG_CONF         0x24
#define A_ACK_NUM           0x28
#define A_RX_HEAD           0x2c
#define A_QUICK_SENT        0x30
#define A_Q0_WORD0          0x34
#define A_Q6_WORD1          0x68
#define A_ESC_CONF0         0x6c
#define A_ESC_CONF3         0x78
#define A_PKT_THRES         0x7c
#define A_DATE              0x80

#define CONF0_TX_RST        BIT(0)
#define CONF0_RX_RST        BIT(1)
#define CONF0_UART_SEL(v)   (((v) >> 2) & 7)
#define CONF0_SEPER_EN      BIT(5)
#define CONF0_HEAD_EN       BIT(6)
#define CONF0_CRC_REC_EN    BIT(7)
#define CONF0_IDLE_EOF_EN   BIT(8)
#define CONF0_LEN_EOF_EN    BIT(9)
#define CONF0_ENCODE_CRC_EN BIT(10)
#define CONF0_BRK_EOF_EN    BIT(12)
#define CONF0_MASK          0x00001fffu
#define CONF0_RESET         0x000006fcu

#define INT_RX_START        BIT(0)
#define INT_TX_START        BIT(1)
#define INT_RX_HUNG         BIT(2)
#define INT_TX_HUNG         BIT(3)
#define INT_SEND_S_REG_Q    BIT(4)
#define INT_SEND_A_REG_Q    BIT(5)
#define INT_OUT_EOF         BIT(6)
#define INT_APP_CTRL0       BIT(7)
#define INT_APP_CTRL1       BIT(8)
#define INT_MASK            0x1ffu
/* The bits software sets and clears in INT_RAW itself (R/W) */
#define INT_APP_MASK        (INT_APP_CTRL0 | INT_APP_CTRL1)

#define CONF1_WAIT_SW_START BIT(7)
#define CONF1_SW_START      BIT(8)
#define CONF1_MASK          0x000000bfu
#define CONF1_RESET         0x00000033u

/* ESCAPE_CONF: TX_* resolve escapes received, RX_* make them for sending */
#define ESCAPE_TX_SHIFT     0
#define ESCAPE_RX_SHIFT     4
#define ESCAPE_CONF_RESET   0x33u

#define HUNG_TX_SHIFT       0
#define HUNG_RX_SHIFT       12
#define HUNG_CONF_RESET     0x00810810u

#define ACK_NUM_MASK        0x7u
#define QUICK_SINGLE_NUM(v) ((v) & 7)
#define QUICK_SINGLE_EN     BIT(3)
#define QUICK_ALWAYS_NUM(v) (((v) >> 4) & 7)
#define QUICK_ALWAYS_EN     BIT(7)
#define QUICK_SENT_MASK     0xf7u

#define PKT_THRES_MASK      0x1fffu
#define PKT_THRES_RESET     0x80u
#define DATE_RESET          0x02201100u

static const uint32_t esc_conf_reset[4] = {
    0x00dcdbc0, 0x00dddbdb, 0x00dedb11, 0x00dfdb13,
};

/* Out queue entries: the byte, and the events its leaving raises. */
#define OUTF_BYTE           0xffu
#define OUTF_START          BIT(8)
#define OUTF_QUICK_S        BIT(9)
#define OUTF_QUICK_A        BIT(10)

/* The decoder may make two entries of one byte: a held lead and an EOF. */
#define IN_ROOM_NEEDED      2

static void uhci_kick(ESP32P4UhciState *s);

static void uhci_update_irq(ESP32P4UhciState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

static void uhci_raise(ESP32P4UhciState *s, uint32_t bits)
{
    s->int_raw |= bits;
    uhci_update_irq(s);
}

static bool uhci_running(ESP32P4UhciState *s)
{
    return clock_is_enabled(s->clk);
}

static ESP32P4UartState *uhci_uart(ESP32P4UhciState *s)
{
    unsigned n = CONF0_UART_SEL(s->conf0);

    return n < ESP32P4_UART_COUNT ? s->uart[n] : NULL;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static void uhci_log_unmodelled(ESP32P4UhciState *s)
{
    if (s->conf0 & (CONF0_HEAD_EN | CONF0_CRC_REC_EN | CONF0_ENCODE_CRC_EN)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4_uhci: CONF0 0x%04x selects the "
                      "HCI packet head or CRC, which the TRM does not "
                      "describe; running without them\n", s->conf0);
    }
}

/* ---- Encoder ---- */

static void out_queue(ESP32P4UhciState *s, uint16_t e)
{
    assert(s->out_q_num < ESP32P4_UHCI_OUT_QUEUE);
    s->out_q[s->out_q_num++] = e;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * A data byte, or the escape pair of the first ESC_CONFn whose character
 * it is, if ESCAPE_CONF.RX_*_ESC_EN enables that escape.
 */
static void out_data(ESP32P4UhciState *s, uint8_t b, uint16_t flags)
{
    uint32_t en = (s->escape_conf >> ESCAPE_RX_SHIFT) & 0xf;

    for (int i = 0; i < 4; i++) {
        uint32_t esc = s->esc_conf[i];

        if ((en & BIT(i)) && b == (esc & 0xff)) {
            out_queue(s, (esc >> 8) & 0xff);
            out_queue(s, ((esc >> 16) & 0xff) | flags);
            return;
        }
    }
    out_queue(s, b | flags);
}

static void out_sep(ESP32P4UhciState *s, uint16_t flags)
{
    out_queue(s, (s->esc_conf[0] & 0xff) | flags);
}

/* Mark the end of a short packet on its last byte. */
static void out_mark_last(ESP32P4UhciState *s, uint16_t flags)
{
    if (s->out_q_num) {
        s->out_q[s->out_q_num - 1] |= flags;
    } else {
        uhci_raise(s, flags & OUTF_QUICK_S ? INT_SEND_S_REG_Q
                                           : INT_SEND_A_REG_Q);
    }
}

static void quick_load(ESP32P4UhciState *s, unsigned n, bool always)
{
    stl_le_p(&s->quick_buf[0], s->q_word[n][0]);
    stl_le_p(&s->quick_buf[4], s->q_word[n][1]);
    s->quick_len = sizeof(s->quick_buf);
    s->quick_pos = 0;
    s->quick_always = always;
    s->quick_in_frame = false;
}

/* One step of a short packet into the empty out queue. */
static void quick_step(ESP32P4UhciState *s)
{
    bool seper = s->conf0 & CONF0_SEPER_EN;
    uint16_t done = s->quick_always ? OUTF_QUICK_A : OUTF_QUICK_S;

    if (seper && !s->quick_in_frame) {
        out_sep(s, OUTF_START);
        s->quick_in_frame = true;
        return;
    }
    if (s->quick_pos < s->quick_len) {
        out_data(s, s->quick_buf[s->quick_pos++], 0);
        return;
    }
    if (seper) {
        out_sep(s, 0);
    }
    out_mark_last(s, done);
    s->quick_len = 0;
    s->quick_in_frame = false;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Fill the empty out queue by one step: a short packet in progress or
 * asked for, or the TX channel's next byte, opening a frame with a
 * separator before it and closing the frame after the EOF descriptor's
 * last byte. With WAIT_SW_START each frame waits for SW_START. Returns
 * false if there is nothing to send.
 */
static bool out_fill(ESP32P4UhciState *s)
{
    bool seper = s->conf0 & CONF0_SEPER_EN;
    bool eof;
    uint8_t b;
    size_t n;

    if (s->quick_len) {
        quick_step(s);
        return true;
    }
    if (!s->out_in_frame && (s->quick_sent & QUICK_SINGLE_EN)) {
        unsigned q = QUICK_SINGLE_NUM(s->quick_sent);

        s->quick_sent &= ~QUICK_SINGLE_EN;
        if (q < ESP32P4_UHCI_QUICK_COUNT) {
            quick_load(s, q, false);
            return true;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_uhci: no short packet Q%u\n",
                      q);
    }
    if (!s->out_in_frame && (s->conf1 & CONF1_WAIT_SW_START) &&
        esp32p4_gdma_tx_pending(s->gdma, ESP32P4_GDMA_AHB_UHCI0)) {
        if (!s->out_sw_start) {
            return false;
        }
        s->out_sw_start = false;
    }
    n = esp32p4_gdma_pull(s->gdma, ESP32P4_GDMA_AHB_UHCI0, &b, 1, &eof);
    if (n || eof) {
        if (seper && !s->out_in_frame) {
            out_sep(s, OUTF_START);
        }
        s->out_in_frame = true;
        if (n) {
            out_data(s, b, 0);
        }
        if (eof) {
            if (seper) {
                out_sep(s, 0);
            }
            s->out_in_frame = false;
        }
        return true;
    }
    if (!s->out_in_frame && (s->quick_sent & QUICK_ALWAYS_EN)) {
        unsigned q = QUICK_ALWAYS_NUM(s->quick_sent);

        if (q < ESP32P4_UHCI_QUICK_COUNT) {
            quick_load(s, q, true);
            return true;
        }
    }
    return false;
}

static void out_drain_one(ESP32P4UhciState *s, ESP32P4UartState *uart)
{
    uint16_t e = s->out_q[0];

    esp32p4_uart_dma_tx_push(uart, e & OUTF_BYTE);
    memmove(s->out_q, s->out_q + 1, --s->out_q_num * sizeof(s->out_q[0]));
    if (e & OUTF_START) {
        uhci_raise(s, INT_RX_START);
    }
    if (e & OUTF_QUICK_S) {
        uhci_raise(s, INT_SEND_S_REG_Q);
    }
    if (e & OUTF_QUICK_A) {
        uhci_raise(s, INT_SEND_A_REG_Q);
    }
}

/* Run the send side as far as the UART takes data; true on progress. */
static bool uhci_out_run(ESP32P4UhciState *s)
{
    ESP32P4UartState *uart = uhci_uart(s);
    bool progress = false;

    if (!uart || (s->conf0 & CONF0_RX_RST)) {
        return false;
    }
    for (;;) {
        while (s->out_q_num && esp32p4_uart_dma_tx_free(uart)) {
            out_drain_one(s, uart);
            progress = true;
        }
        if (s->out_q_num || !esp32p4_uart_dma_tx_free(uart) ||
            !out_fill(s)) {
            break;
        }
        progress = true;
    }
    return progress;
}

/* ---- Decoder ---- */

static bool in_push(ESP32P4UhciState *s, const uint8_t *b, size_t len,
                    unsigned flags)
{
    return esp32p4_gdma_push(s->gdma, ESP32P4_GDMA_AHB_UHCI0, b, len,
                             flags) == len;
}

static void in_byte(ESP32P4UhciState *s, uint8_t b)
{
    uint32_t thrs = s->pkt_thres & PKT_THRES_MASK;
    bool len_eof = (s->conf0 & CONF0_LEN_EOF_EN) &&
                   !(s->conf0 & CONF0_HEAD_EN) && thrs;

    s->in_frame_data = true;
    s->in_count++;
    if (len_eof && s->in_count >= thrs) {
        in_push(s, &b, 1, ESP32P4_GDMA_EOF);
        s->in_count = 0;
        s->in_frame_data = false;
        s->in_skip = s->conf0 & CONF0_SEPER_EN;
        return;
    }
    in_push(s, &b, 1, 0);
}

/* End the frame received so far: suc_eof on its last byte. */
static void in_end_frame(ESP32P4UhciState *s)
{
    if (s->in_esc) {
        in_push(s, &s->in_esc_char, 1, 0);
        s->in_esc = false;
    }
    if (s->in_frame_data) {
        in_push(s, NULL, 0, ESP32P4_GDMA_EOF);
    }
    s->in_frame_data = false;
    s->in_count = 0;
}

static int in_unescape(ESP32P4UhciState *s, uint8_t lead, uint8_t b)
{
    uint32_t en = (s->escape_conf >> ESCAPE_TX_SHIFT) & 0xf;

    for (int i = 0; i < 4; i++) {
        uint32_t esc = s->esc_conf[i];

        if ((en & BIT(i)) && lead == ((esc >> 8) & 0xff) &&
            b == ((esc >> 16) & 0xff)) {
            return esc & 0xff;
        }
    }
    return -1;
}

static bool in_is_esc_lead(ESP32P4UhciState *s, uint8_t b)
{
    uint32_t en = (s->escape_conf >> ESCAPE_TX_SHIFT) & 0xf;

    for (int i = 0; i < 4; i++) {
        if ((en & BIT(i)) && b == ((s->esc_conf[i] >> 8) & 0xff)) {
            return true;
        }
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Decode a byte from the UART, with room for two entries in the RX
 * channel. With SEPER_EN a separator opens a frame (TX_START), more
 * separators before any data are skipped, and the next one after data
 * closes it; bytes outside a frame are dropped. An escape lead with a
 * byte that completes no enabled pair passes through unchanged.
 */
static void in_decode(ESP32P4UhciState *s, uint8_t b)
{
    bool seper = s->conf0 & CONF0_SEPER_EN;

    if (seper && b == (s->esc_conf[0] & 0xff)) {
        if (!s->in_frame) {
            s->in_frame = true;
            s->in_frame_data = false;
            s->in_count = 0;
            uhci_raise(s, INT_TX_START);
        } else if (s->in_frame_data || s->in_esc || s->in_skip) {
            in_end_frame(s);
            s->in_frame = false;
            s->in_skip = false;
        }
        return;
    }
    if (seper && (!s->in_frame || s->in_skip)) {
        return;
    }
    if (s->in_esc) {
        int dec = in_unescape(s, s->in_esc_char, b);

        s->in_esc = false;
        if (dec >= 0) {
            in_byte(s, dec);
            return;
        }
        in_byte(s, s->in_esc_char);
    }
    if (in_is_esc_lead(s, b)) {
        s->in_esc = true;
        s->in_esc_char = b;
        return;
    }
    in_byte(s, b);
}

/*
 * Run the receive side; true on progress. The UHCI takes bytes from the
 * UART's RX FIFO only while the RX channel has room, so with no running
 * channel they stay there for software.
 */
static bool uhci_in_run(ESP32P4UhciState *s)
{
    ESP32P4UartState *uart = uhci_uart(s);
    bool progress = false;
    uint8_t b;

    if (!uart || (s->conf0 & CONF0_TX_RST)) {
        return false;
    }
    while (esp32p4_gdma_rx_room(s->gdma, ESP32P4_GDMA_AHB_UHCI0) >=
           IN_ROOM_NEEDED) {
        if (s->in_eof_pending && !s->in_eof_left) {
            s->in_eof_pending = false;
            in_end_frame(s);
            progress = true;
            continue;
        }
        if (!esp32p4_uart_dma_rx_pop(uart, &b)) {
            break;
        }
        if (s->in_eof_pending) {
            s->in_eof_left--;
        }
        in_decode(s, b);
        progress = true;
    }
    return progress;
}

/* ---- Hang timeouts ---- */

static int64_t uhci_hung_ns(ESP32P4UhciState *s, unsigned shift)
{
    uint32_t f = s->hung_conf >> shift;

    return (int64_t)(f & 0xff) * (SCALE_MS >> ((f >> 8) & 7));
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * A timeout runs while its side holds data it cannot move: the send side
 * a byte for a UART with no room (RX_HUNG, RXFIFO_TIMEOUT), the receive
 * side bytes for an RX channel with no room, or none running (TX_HUNG,
 * TXFIFO_TIMEOUT). It fires once per stall.
 */
static void uhci_hung_update(ESP32P4UhciState *s, QEMUTimer *t, bool *fired,
                             bool stalled, bool progress, unsigned shift)
{
    bool ena = (s->hung_conf >> shift) & BIT(11);

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
    ESP32P4UhciState *s = opaque;

    s->out_hung_fired = true;
    uhci_raise(s, INT_RX_HUNG);
}

static void uhci_in_hung_cb(void *opaque)
{
    ESP32P4UhciState *s = opaque;

    s->in_hung_fired = true;
    uhci_raise(s, INT_TX_HUNG);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Move data as far as the UART, GDMA and the queues allow. Called on
 * register writes, by the UART as its FIFOs change and by GDMA as data or
 * room arrives; a call from within a call repeats the run.
 */
static void uhci_kick(ESP32P4UhciState *s)
{
    ESP32P4UartState *uart = uhci_uart(s);
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
                     uart && s->out_q_num != 0, out_progress, HUNG_RX_SHIFT);
    uhci_hung_update(s, &s->in_hung_timer, &s->in_hung_fired,
                     uart && (s->in_eof_pending ||
                              esp32p4_uart_dma_rx_count(uart)) &&
                     esp32p4_gdma_rx_room(s->gdma, ESP32P4_GDMA_AHB_UHCI0) <
                     IN_ROOM_NEEDED,
                     in_progress, HUNG_TX_SHIFT);
    uhci_update_irq(s);
}

/* ---- The UART's and GDMA's notifications ---- */

static void uhci_uart_room(void *opaque)
{
    ESP32P4UhciLink *l = opaque;

    if (uhci_uart(l->s) == l->uart) {
        uhci_kick(l->s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The UART's idle (UART_IDLE_EOF_EN) or a NULL character
 * (UART_RX_BRK_EOF_EN, the NULL being the frame's last byte where the
 * UART stores it) ends the frame being received, after the bytes of it
 * still in the UART's RX FIFO.
 */
static void uhci_uart_frame_end(ESP32P4UhciLink *l, uint32_t enable)
{
    ESP32P4UhciState *s = l->s;
    unsigned left;

    if (uhci_uart(s) != l->uart || !(s->conf0 & enable) ||
        s->in_eof_pending) {
        return;
    }
    left = esp32p4_uart_dma_rx_count(l->uart);
    if (s->in_frame_data || s->in_esc || left) {
        s->in_eof_pending = true;
        s->in_eof_left = left;
        uhci_kick(s);
    }
}

static void uhci_uart_idle(void *opaque)
{
    uhci_uart_frame_end(opaque, CONF0_IDLE_EOF_EN);
}

static void uhci_uart_break(void *opaque)
{
    uhci_uart_frame_end(opaque, CONF0_BRK_EOF_EN);
}

static const ESP32P4UartDmaOps uhci_uart_ops = {
    .tx_room = uhci_uart_room,
    .rx_data = uhci_uart_room,
    .rx_idle = uhci_uart_idle,
    .rx_break = uhci_uart_break,
};

static void uhci_gdma_ready(void *opaque)
{
    uhci_kick(opaque);
}

static const ESP32P4GdmaPeriphOps uhci_gdma_ops = {
    .tx_ready = uhci_gdma_ready,
    .rx_ready = uhci_gdma_ready,
};

/* ---- Registers ---- */

static void uhci_out_reset(ESP32P4UhciState *s)
{
    s->out_q_num = 0;
    s->out_in_frame = false;
    s->out_sw_start = false;
    s->quick_len = 0;
    s->quick_pos = 0;
    s->quick_in_frame = false;
    timer_del(&s->out_hung_timer);
    s->out_hung_fired = false;
}

static void uhci_in_reset(ESP32P4UhciState *s)
{
    s->in_frame = false;
    s->in_frame_data = false;
    s->in_esc = false;
    s->in_count = 0;
    s->in_skip = false;
    s->in_eof_pending = false;
    s->in_eof_left = 0;
    timer_del(&s->in_hung_timer);
    s->in_hung_fired = false;
}

static uint64_t uhci_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4UhciState *s = opaque;

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
    case A_CONF1:
        return s->conf1;
    case A_STATE0:
        /* No HCI packet errors without HEAD_EN; DECODE_STATE 1 in a frame */
        return (s->in_frame || s->in_frame_data) ? 1u << 3 : 0;
    case A_STATE1:
        return s->out_in_frame || s->quick_len || s->out_q_num ? 1 : 0;
    case A_ESCAPE_CONF:
        return s->escape_conf;
    case A_HUNG_CONF:
        return s->hung_conf;
    case A_ACK_NUM:
        return s->ack_num;
    case A_RX_HEAD:
        return 0;
    case A_QUICK_SENT:
        return s->quick_sent & ~QUICK_SINGLE_EN;
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
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_uhci: read of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static void uhci_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4UhciState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case A_CONF0: {
        uint32_t old = s->conf0;

        s->conf0 = v & CONF0_MASK;
        /* TX_RST resets the decoder, RX_RST the encoder (uhci_reg.h). */
        if (s->conf0 & CONF0_TX_RST) {
            uhci_in_reset(s);
        }
        if (s->conf0 & CONF0_RX_RST) {
            uhci_out_reset(s);
        }
        if (CONF0_UART_SEL(old) != CONF0_UART_SEL(s->conf0)) {
            uhci_out_reset(s);
            uhci_in_reset(s);
        }
        uhci_log_unmodelled(s);
        break;
    }
    case A_INT_RAW:
        s->int_raw = (s->int_raw & ~(v & ~INT_APP_MASK) & ~INT_APP_MASK) |
                     (v & INT_APP_MASK);
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_MASK;
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_MASK);
        break;
    case A_CONF1:
        s->conf1 = v & CONF1_MASK;
        if (v & CONF1_SW_START) {
            s->out_sw_start = true;
        }
        if (v & 0xf) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_uhci: CONF1's HCI packet "
                          "checks are not modelled\n");
        }
        break;
    case A_ESCAPE_CONF:
        s->escape_conf = v & 0xff;
        break;
    case A_HUNG_CONF:
        s->hung_conf = v & 0x00ffffff;
        break;
    case A_ACK_NUM:
        s->ack_num = v & ACK_NUM_MASK;
        break;
    case A_QUICK_SENT:
        s->quick_sent = v & (QUICK_SENT_MASK | QUICK_SINGLE_EN);
        break;
    case A_Q0_WORD0 ... A_Q6_WORD1:
        s->q_word[(addr - A_Q0_WORD0) / 8][((addr - A_Q0_WORD0) / 4) & 1] = v;
        break;
    case A_ESC_CONF0 ... A_ESC_CONF3:
        s->esc_conf[(addr - A_ESC_CONF0) / 4] = v & 0x00ffffff;
        break;
    case A_PKT_THRES:
        s->pkt_thres = v & PKT_THRES_MASK;
        break;
    case A_DATE:
        s->date = v;
        break;
    case A_INT_ST:
    case A_STATE0:
    case A_STATE1:
    case A_RX_HEAD:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_uhci: write of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    uhci_kick(s);
}

static const MemoryRegionOps uhci_ops = {
    .read = uhci_read,
    .write = uhci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void uhci_clk_update(void *opaque, ClockEvent event)
{
    uhci_kick(opaque);
}

static void uhci_reset_hold(Object *obj, ResetType type)
{
    ESP32P4UhciState *s = ESP32P4_UHCI(obj);

    s->conf0 = CONF0_RESET;
    s->int_raw = 0;
    s->int_ena = 0;
    s->conf1 = CONF1_RESET;
    s->escape_conf = ESCAPE_CONF_RESET;
    s->hung_conf = HUNG_CONF_RESET;
    s->ack_num = 0;
    s->quick_sent = 0;
    memset(s->q_word, 0, sizeof(s->q_word));
    memcpy(s->esc_conf, esc_conf_reset, sizeof(s->esc_conf));
    s->pkt_thres = PKT_THRES_RESET;
    s->date = DATE_RESET;
    uhci_out_reset(s);
    uhci_in_reset(s);
}

static void uhci_reset_exit(Object *obj, ResetType type)
{
    uhci_update_irq(ESP32P4_UHCI(obj));
}

static void uhci_realize(DeviceState *dev, Error **errp)
{
    ESP32P4UhciState *s = ESP32P4_UHCI(dev);

    if (!s->gdma) {
        error_setg(errp, "esp32p4-uhci: 'gdma' link not set");
        return;
    }
    for (int i = 0; i < ESP32P4_UART_COUNT; i++) {
        if (!s->uart[i]) {
            error_setg(errp, "esp32p4-uhci: 'uart%d' link not set", i);
            return;
        }
        s->link[i].s = s;
        s->link[i].uart = s->uart[i];
        esp32p4_uart_dma_attach(s->uart[i], &uhci_uart_ops, &s->link[i]);
    }
    esp32p4_gdma_attach(s->gdma, ESP32P4_GDMA_AHB_UHCI0, &uhci_gdma_ops, s);
}

static void uhci_init(Object *obj)
{
    ESP32P4UhciState *s = ESP32P4_UHCI(obj);

    memory_region_init_io(&s->iomem, obj, &uhci_ops, s, "esp32p4.uhci",
                          ESP32P4_UHCI_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", uhci_clk_update, s,
                                ClockUpdate);
    timer_init_ns(&s->out_hung_timer, QEMU_CLOCK_VIRTUAL, uhci_out_hung_cb,
                  s);
    timer_init_ns(&s->in_hung_timer, QEMU_CLOCK_VIRTUAL, uhci_in_hung_cb, s);
    object_property_add_link(obj, "gdma", TYPE_ESP32P4_GDMA,
                             (Object **)&s->gdma,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    for (int i = 0; i < ESP32P4_UART_COUNT; i++) {
        g_autofree char *name = g_strdup_printf("uart%d", i);

        object_property_add_link(obj, name, TYPE_ESP32P4_UART,
                                 (Object **)&s->uart[i],
                                 qdev_prop_allow_set_link_before_realize,
                                 OBJ_PROP_LINK_STRONG);
    }
}

static const VMStateDescription vmstate_esp32p4_uhci = {
    .name = TYPE_ESP32P4_UHCI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4UhciState),
        VMSTATE_UINT32(conf0, ESP32P4UhciState),
        VMSTATE_UINT32(int_raw, ESP32P4UhciState),
        VMSTATE_UINT32(int_ena, ESP32P4UhciState),
        VMSTATE_UINT32(conf1, ESP32P4UhciState),
        VMSTATE_UINT32(escape_conf, ESP32P4UhciState),
        VMSTATE_UINT32(hung_conf, ESP32P4UhciState),
        VMSTATE_UINT32(ack_num, ESP32P4UhciState),
        VMSTATE_UINT32(quick_sent, ESP32P4UhciState),
        VMSTATE_UINT32_2DARRAY(q_word, ESP32P4UhciState,
                               ESP32P4_UHCI_QUICK_COUNT, 2),
        VMSTATE_UINT32_ARRAY(esc_conf, ESP32P4UhciState, 4),
        VMSTATE_UINT32(pkt_thres, ESP32P4UhciState),
        VMSTATE_UINT32(date, ESP32P4UhciState),
        VMSTATE_UINT16_ARRAY(out_q, ESP32P4UhciState, ESP32P4_UHCI_OUT_QUEUE),
        VMSTATE_UINT8(out_q_num, ESP32P4UhciState),
        VMSTATE_BOOL(out_in_frame, ESP32P4UhciState),
        VMSTATE_BOOL(out_sw_start, ESP32P4UhciState),
        VMSTATE_UINT8_ARRAY(quick_buf, ESP32P4UhciState, 8),
        VMSTATE_UINT8(quick_len, ESP32P4UhciState),
        VMSTATE_UINT8(quick_pos, ESP32P4UhciState),
        VMSTATE_BOOL(quick_always, ESP32P4UhciState),
        VMSTATE_BOOL(quick_in_frame, ESP32P4UhciState),
        VMSTATE_BOOL(in_frame, ESP32P4UhciState),
        VMSTATE_BOOL(in_frame_data, ESP32P4UhciState),
        VMSTATE_BOOL(in_esc, ESP32P4UhciState),
        VMSTATE_UINT8(in_esc_char, ESP32P4UhciState),
        VMSTATE_UINT32(in_count, ESP32P4UhciState),
        VMSTATE_BOOL(in_skip, ESP32P4UhciState),
        VMSTATE_BOOL(in_eof_pending, ESP32P4UhciState),
        VMSTATE_UINT32(in_eof_left, ESP32P4UhciState),
        VMSTATE_TIMER(out_hung_timer, ESP32P4UhciState),
        VMSTATE_TIMER(in_hung_timer, ESP32P4UhciState),
        VMSTATE_BOOL(out_hung_fired, ESP32P4UhciState),
        VMSTATE_BOOL(in_hung_fired, ESP32P4UhciState),
        VMSTATE_END_OF_LIST()
    }
};

static void uhci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = uhci_realize;
    dc->vmsd = &vmstate_esp32p4_uhci;
    dc->user_creatable = false;
    rc->phases.hold = uhci_reset_hold;
    rc->phases.exit = uhci_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static const TypeInfo uhci_info = {
    .name = TYPE_ESP32P4_UHCI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4UhciState),
    .instance_init = uhci_init,
    .class_init = uhci_class_init,
};

static void uhci_register_types(void)
{
    type_register_static(&uhci_info);
}

type_init(uhci_register_types)
