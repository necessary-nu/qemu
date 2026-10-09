/*
 * ESP32 UART emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * The QEMU model of nRF51 UART by Julia Suvorova was used as a template.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * The UART works at the line level: its TXD, RTS and DTR outputs and RXD,
 * CTS and DSR inputs are GPIO matrix signals. The transmitter shifts each
 * frame out bit by bit at the baud rate, and the receiver samples frames
 * from its RXD input; what is on the other end of the lines (a terminal,
 * another UART, a loop back through the matrix) is the board's business.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/char/esp32_uart.h"
#include "hw/gpio/esp32_gpio.h"
#include "migration/vmstate.h"

/* INT_RAW bits that latch until INT_CLR clears them */
#define UART_INT_LATCHED (R_UART_INT_RAW_PARITY_ERR_MASK | \
                          R_UART_INT_RAW_FRM_ERR_MASK | \
                          R_UART_INT_RAW_RXFIFO_OVF_MASK | \
                          R_UART_INT_RAW_DSR_CHG_MASK | \
                          R_UART_INT_RAW_CTS_CHG_MASK | \
                          R_UART_INT_RAW_BRK_DET_MASK)

static void uart_tx_start(ESP32UARTState *s);
static void uart_tx_advance(ESP32UARTState *s);

uint64_t esp32_uart_mul_shr(uint64_t a, uint64_t b, unsigned shift)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, a, b);
    if (shift >= 64) {
        return hi >> (shift - 64);
    }
    if (shift == 0) {
        return hi ? UINT64_MAX : lo;
    }
    if (hi >> shift) {
        return UINT64_MAX;
    }
    return (lo >> shift) | (hi << (64 - shift));
}

/* [spec:nuos:req:emu.esp32.gpio] */
unsigned esp32_uart_frame_levels(const Esp32UartFrameFmt *fmt, unsigned data,
                                 uint16_t *levels)
{
    uint16_t v = 0;
    unsigned n = 1, ones = 0;

    for (unsigned i = 0; i < fmt->data_bits; i++, n++) {
        if ((data >> i) & 1) {
            v |= 1u << n;
            ones++;
        }
    }
    if (fmt->parity_en) {
        if ((ones & 1) ^ fmt->parity_odd) {
            v |= 1u << n;
        }
        n++;
    }
    *levels = v;
    return n;
}

/* The middle of bit `bit` of the frame being received, from its start bit */
static int64_t rx_sample_ns(const Esp32UartRx *rx, unsigned bit)
{
    return rx->edge_ns[0] +
           esp32_uart_mul_shr(2 * bit + 1, rx->fmt.bit_q32, 33);
}

static unsigned rx_stop_bit(const Esp32UartRx *rx)
{
    return 1 + rx->fmt.data_bits + rx->fmt.parity_en;
}

/*
 * The line's level at t within the frame: edge 0 is the start bit's
 * falling edge, and each later one toggles the line.
 */
static bool rx_level_at(const Esp32UartRx *rx, int64_t t)
{
    unsigned n = 0;

    while (n < rx->n_edges && rx->edge_ns[n] <= t) {
        n++;
    }
    return !(n & 1);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Sample the frame received and hand it over; then look for the next
 * start bit among the changes since the stop bit's sample, or since the
 * start bit's sample if that found the line high again (a glitch, not a
 * start bit).
 */
static void rx_finish(Esp32UartRx *rx)
{
    unsigned stop = rx_stop_bit(rx);
    int64_t resume = rx_sample_ns(rx, stop);
    int64_t rest[ESP32_UART_RX_EDGES];
    unsigned n_rest = 0, data = 0, flags = 0;
    bool valid = !rx_level_at(rx, rx_sample_ns(rx, 0));

    if (!valid) {
        resume = rx_sample_ns(rx, 0);
    } else {
        unsigned ones = 0;
        bool parity = false, stop_level;

        for (unsigned i = 0; i < rx->fmt.data_bits; i++) {
            if (rx_level_at(rx, rx_sample_ns(rx, 1 + i))) {
                data |= 1u << i;
                ones++;
            }
        }
        if (rx->fmt.parity_en) {
            parity = rx_level_at(rx, rx_sample_ns(rx, stop - 1));
            if (((ones + parity) & 1) != rx->fmt.parity_odd) {
                flags |= ESP32_UART_RX_PARITY_ERR;
            }
        }
        stop_level = rx_level_at(rx, resume);
        if (!stop_level || rx->noise) {
            flags |= ESP32_UART_RX_FRAME_ERR;
            if (!stop_level && data == 0 && !parity) {
                flags |= ESP32_UART_RX_BREAK;
            }
        }
    }

    for (unsigned i = 0; i < rx->n_edges; i++) {
        if (rx->edge_ns[i] > resume) {
            rest[n_rest++] = rx->edge_ns[i];
        }
    }
    rx->level = rx_level_at(rx, resume);
    rx->busy = false;
    rx->noise = false;
    rx->n_edges = 0;
    timer_del(&rx->timer);
    if (valid) {
        rx->deliver(rx->opaque, data, flags);
    }
    for (unsigned i = 0; i < n_rest; i++) {
        esp32_uart_rx_line(rx, !rx->level, rest[i]);
    }
}

/* [spec:nuos:req:emu.esp32.gpio] */
void esp32_uart_rx_line(Esp32UartRx *rx, bool level, int64_t when_ns)
{
    if (level == rx->level) {
        return;
    }
    if (rx->busy && when_ns > rx_sample_ns(rx, rx_stop_bit(rx))) {
        rx_finish(rx);
        if (level == rx->level) {
            return;
        }
    }
    rx->level = level;
    if (rx->busy) {
        /* Stamps from different sources may not be in order */
        when_ns = MAX(when_ns, rx->edge_ns[rx->n_edges - 1]);
        if (rx->n_edges < ESP32_UART_RX_EDGES) {
            rx->edge_ns[rx->n_edges++] = when_ns;
        } else {
            rx->noise = true;
        }
        return;
    }
    if (level || !rx->format(rx->opaque, &rx->fmt) || !rx->fmt.bit_q32) {
        return;
    }
    rx->busy = true;
    rx->n_edges = 1;
    rx->edge_ns[0] = when_ns;
    timer_mod_ns(&rx->timer, rx_sample_ns(rx, rx_stop_bit(rx)));
}

static void rx_timer_cb(void *opaque)
{
    Esp32UartRx *rx = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * Every change up to now must be in before the frame is sampled; one
     * that started while the sources caught up is not due yet.
     */
    esp32_line_sync();
    if (rx->busy && now >= rx_sample_ns(rx, rx_stop_bit(rx))) {
        rx_finish(rx);
    }
}

void esp32_uart_rx_init(Esp32UartRx *rx, Esp32UartRxFormat *format,
                        Esp32UartRxDeliver *deliver, void *opaque)
{
    rx->format = format;
    rx->deliver = deliver;
    rx->opaque = opaque;
    rx->level = true;
    timer_init_ns(&rx->timer, QEMU_CLOCK_VIRTUAL, rx_timer_cb, rx);
}

void esp32_uart_rx_reset(Esp32UartRx *rx, bool level)
{
    timer_del(&rx->timer);
    rx->busy = false;
    rx->noise = false;
    rx->n_edges = 0;
    rx->level = level;
}

const VMStateDescription vmstate_esp32_uart_rx = {
    .name = "esp32-uart-rx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, Esp32UartRx),
        VMSTATE_BOOL(level, Esp32UartRx),
        VMSTATE_BOOL(busy, Esp32UartRx),
        VMSTATE_BOOL(noise, Esp32UartRx),
        VMSTATE_UINT8(fmt.data_bits, Esp32UartRx),
        VMSTATE_BOOL(fmt.parity_en, Esp32UartRx),
        VMSTATE_BOOL(fmt.parity_odd, Esp32UartRx),
        VMSTATE_UINT8(fmt.stop_half_bits, Esp32UartRx),
        VMSTATE_UINT64(fmt.bit_q32, Esp32UartRx),
        VMSTATE_UINT8(n_edges, Esp32UartRx),
        VMSTATE_INT64_ARRAY(edge_ns, Esp32UartRx, ESP32_UART_RX_EDGES),
        VMSTATE_END_OF_LIST()
    }
};

/* The UART only operates while the SoC lets its clocks run. */
static bool uart_running(ESP32UARTState *s)
{
    return clock_is_enabled(s->apb_clk);
}

static bool uart_uses_ref_tick(ESP32UARTState *s)
{
    return !FIELD_EX32(s->reg[R_UART_CONF0], UART_CONF0, TICK_REF_ALWAYS_ON);
}

static Clock *uart_baud_clk(ESP32UARTState *s)
{
    return uart_uses_ref_tick(s) ? s->ref_tick_clk : s->apb_clk;
}

/* The baud divider in 1/16ths: CLKDIV + CLKDIV_FRAG / 16. */
static uint64_t uart_div16(ESP32UARTState *s)
{
    return ((uint64_t)FIELD_EX32(s->reg[R_UART_CLKDIV], UART_CLKDIV, CLKDIV)
            << 4) + FIELD_EX32(s->reg[R_UART_CLKDIV], UART_CLKDIV, CLKDIV_FRAG);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * The bit time in 2^-32 ns: (CLKDIV + CLKDIV_FRAG / 16) periods of the baud
 * clock at its current frequency. 0 when the UART is stopped or the divider
 * is 0.
 */
static uint64_t uart_bit_q32(ESP32UARTState *s)
{
    Clock *clk = uart_baud_clk(s);
    uint64_t div16 = uart_div16(s);

    if (!uart_running(s) || !clock_is_enabled(clk) || div16 == 0) {
        return 0;
    }
    return esp32_uart_mul_shr(clock_get(clk), div16, 4);
}

static void uart_format(ESP32UARTState *s, Esp32UartFrameFmt *fmt)
{
    static const uint8_t stop_half_bits[] = { 2, 2, 3, 4 };
    uint32_t conf0 = s->reg[R_UART_CONF0];

    fmt->data_bits = 5 + FIELD_EX32(conf0, UART_CONF0, BIT_NUM);
    fmt->parity_en = FIELD_EX32(conf0, UART_CONF0, PARITY_EN);
    fmt->parity_odd = FIELD_EX32(conf0, UART_CONF0, PARITY);
    fmt->stop_half_bits =
        stop_half_bits[FIELD_EX32(conf0, UART_CONF0, STOP_BIT_NUM)];
    fmt->bit_q32 = uart_bit_q32(s);
}

/* [spec:nuos:req:emu.esp32.gpio] */
bool esp32_uart_line_format(ESP32UARTState *s, Esp32UartFrameFmt *fmt)
{
    uart_format(s, fmt);
    return fmt->bit_q32 != 0;
}

void esp32_uart_add_format_notifier(ESP32UARTState *s, Notifier *n)
{
    notifier_list_add(&s->format_notifiers, n);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * Baud rate = baud clock * 16 / (CLKDIV * 16 + CLKDIV_FRAG), with the baud
 * clock at its current frequency. 0 when the UART is stopped or the
 * divider is 0.
 */
static void uart_update_baud(ESP32UARTState *s)
{
    Clock *clk = uart_baud_clk(s);
    uint64_t div16 = uart_div16(s);

    if (!uart_running(s) || !clock_is_enabled(clk) || div16 == 0) {
        s->baud_rate = 0;
    } else {
        s->baud_rate = muldiv64(clock_get_hz(clk), 16, div16);
    }
}

void esp32_uart_update_irq(ESP32UARTState *s)
{
    bool irq = false;

    uint32_t tx_empty_raw = (fifo8_num_used(&s->tx_fifo) <= s->tx_empty_threshold);
    uint32_t rx_full_raw = (fifo8_num_used(&s->rx_fifo) >= s->rx_full_threshold);
    uint32_t tx_done_raw = (fifo8_num_used(&s->tx_fifo) == 0) && !s->tx_busy;
    uint32_t rxfifo_tout_raw = (s->rxfifo_tout) ? 1 : 0;

    uint32_t int_raw = s->reg[R_UART_INT_RAW];
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, RXFIFO_FULL, rx_full_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, TXFIFO_EMPTY, tx_empty_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, TX_DONE, tx_done_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, RXFIFO_TOUT, rxfifo_tout_raw);
    s->reg[R_UART_INT_RAW] = int_raw;

    uint32_t int_st = s->reg[R_UART_INT_RAW] & s->reg[R_UART_INT_ENA];
    irq = int_st != 0;
    s->reg[R_UART_INT_ST] = int_st;

    qemu_set_irq(s->irq, irq);
}


void esp32_uart_set_rx_timeout(ESP32UARTState *s)
{
    if (s->rx_tout_ena && s->baud_rate) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t thres = s->rx_tout_thres;
        /*
         * The timeout counter runs from APB_CLK: with REF_TICK as the baud
         * clock, the threshold counts bit times scaled by REF_TICK / APB_CLK.
         */
        if (uart_uses_ref_tick(s)) {
            thres = muldiv64(thres, clock_get_hz(s->ref_tick_clk),
                             MAX(clock_get_hz(s->apb_clk), 1));
        }
        int64_t rx_timeout_ns = now +
            thres * NANOSECONDS_PER_SECOND / s->baud_rate;
        timer_mod_ns(&s->rx_timeout_timer, rx_timeout_ns);
    } else {
        timer_del(&s->rx_timeout_timer);
        s->rxfifo_tout = false;
    }
}

/* The level the receiver sees: RXD, or TXD looped back inside the UART. */
static void uart_rx_input(ESP32UARTState *s, int64_t when_ns)
{
    uint32_t conf0 = s->reg[R_UART_CONF0];
    bool level;

    if (FIELD_EX32(conf0, UART_CONF0, LOOPBACK)) {
        level = s->tx_line;
    } else {
        level = s->rxd_pin ^ FIELD_EX32(conf0, UART_CONF0, RXD_INV);
    }
    esp32_uart_rx_line(&s->rx, level, when_ns);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * CTS and DSR as the UART core sees them: the pins, inverted by CTS_INV and
 * DSR_INV, or in loopback mode the UART's own RTS and DTR. A change raises
 * CTS_CHG or DSR_CHG; CTS going active (low) lets a flow-controlled
 * transmitter start.
 */
static void uart_update_inputs(ESP32UARTState *s, bool raise)
{
    uint32_t conf0 = s->reg[R_UART_CONF0];
    bool lb = FIELD_EX32(conf0, UART_CONF0, LOOPBACK);
    bool cts = lb ? s->rts_line :
               s->cts_pin ^ FIELD_EX32(conf0, UART_CONF0, CTS_INV);
    bool dsr = lb ? s->dtr_line :
               s->dsr_pin ^ FIELD_EX32(conf0, UART_CONF0, DSR_INV);

    if (cts != s->cts_line) {
        s->cts_line = cts;
        if (raise) {
            s->reg[R_UART_INT_RAW] |= R_UART_INT_RAW_CTS_CHG_MASK;
        }
    }
    if (dsr != s->dsr_line) {
        s->dsr_line = dsr;
        if (raise) {
            s->reg[R_UART_INT_RAW] |= R_UART_INT_RAW_DSR_CHG_MASK;
        }
    }
    uart_rx_input(s, esp32_line_time());
    if (raise) {
        uart_tx_start(s);
        esp32_uart_update_irq(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Drive TXD, RTS and DTR. RTS and DTR are active low: SW_RTS and SW_DTR
 * set them active, and with RX_FLOW_EN the receiver drives RTS instead,
 * inactive while the RX FIFO holds RX_FLOW_THRHD bytes or more. The pins
 * carry them inverted by TXD_INV, RTS_INV and DTR_INV.
 */
static void uart_update_lines(ESP32UARTState *s, bool raise)
{
    uint32_t conf0 = s->reg[R_UART_CONF0];
    uint32_t conf1 = s->reg[R_UART_CONF1];

    if (FIELD_EX32(conf1, UART_CONF1, RX_FLOW_EN)) {
        s->rts_line = fifo8_num_used(&s->rx_fifo) >=
                      FIELD_EX32(conf1, UART_CONF1, RX_FLOW_THRHD);
    } else {
        s->rts_line = !FIELD_EX32(conf0, UART_CONF0, SW_RTS);
    }
    s->dtr_line = !FIELD_EX32(conf0, UART_CONF0, SW_DTR);
    qemu_set_irq(s->txd_out,
                 s->tx_line ^ FIELD_EX32(conf0, UART_CONF0, TXD_INV));
    qemu_set_irq(s->rts_out,
                 s->rts_line ^ FIELD_EX32(conf0, UART_CONF0, RTS_INV));
    qemu_set_irq(s->dtr_out,
                 s->dtr_line ^ FIELD_EX32(conf0, UART_CONF0, DTR_INV));
    uart_update_inputs(s, raise);
}

/* Put the transmitter's level on TXD, and on the receiver in loopback. */
static void uart_set_tx_line(ESP32UARTState *s, bool level, int64_t when_ns)
{
    uint32_t conf0 = s->reg[R_UART_CONF0];

    s->tx_line = level;
    esp32_line_set_at(s->txd_out,
                      level ^ FIELD_EX32(conf0, UART_CONF0, TXD_INV), when_ns);
    if (FIELD_EX32(conf0, UART_CONF0, LOOPBACK)) {
        uart_rx_input(s, when_ns);
    }
}

static int64_t tx_time(ESP32UARTState *s, unsigned half_bits)
{
    return s->tx_start_ns + esp32_uart_mul_shr(half_bits, s->tx_q32, 33);
}

static bool tx_bit_level(ESP32UARTState *s, unsigned bit)
{
    return bit < s->tx_nbits ? (s->tx_levels >> bit) & 1 : 1;
}

/* The next bit at which TXD changes, or past tx_nbits if none does. */
static unsigned tx_next_change(ESP32UARTState *s)
{
    unsigned b;

    for (b = s->tx_bit + 1; b <= s->tx_nbits; b++) {
        if (tx_bit_level(s, b) != tx_bit_level(s, b - 1)) {
            break;
        }
    }
    return b;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A frame may start when a byte waits, the baud clock runs and, with
 * TX_FLOW_EN, CTS is active (low).
 */
static bool uart_tx_can_start(ESP32UARTState *s)
{
    return !fifo8_is_empty(&s->tx_fifo) && uart_bit_q32(s) &&
           (!FIELD_EX32(s->reg[R_UART_CONF0], UART_CONF0, TX_FLOW_EN) ||
            !s->cts_line);
}

/* Take the next byte from the TX FIFO and start its frame at start_ns. */
static void uart_tx_start_at(ESP32UARTState *s, int64_t start_ns)
{
    Esp32UartFrameFmt fmt;

    uart_format(s, &fmt);
    s->tx_nbits = esp32_uart_frame_levels(&fmt, fifo8_pop(&s->tx_fifo),
                                          &s->tx_levels);
    s->tx_half = s->tx_nbits * 2 + fmt.stop_half_bits;
    s->tx_q32 = fmt.bit_q32;
    s->tx_start_ns = start_ns;
    s->tx_bit = 0;
    s->tx_busy = true;
    uart_set_tx_line(s, 0, start_ns);
    /* A UHCI feeding this UART can refill the slot just freed. */
    notifier_list_notify(&s->dma_notifiers, s);
}

static void uart_tx_schedule(ESP32UARTState *s)
{
    unsigned b;

    if (!s->tx_busy || !s->tx_q32) {
        timer_del(&s->tx_timer);
        return;
    }
    b = tx_next_change(s);
    timer_mod_ns(&s->tx_timer, b <= s->tx_nbits ? tx_time(s, 2 * b) :
                                                  tx_time(s, s->tx_half));
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Bring the transmitter up to the present: put every bit whose time has
 * passed on TXD, stamped with its own time, and start the next frame right
 * after the stop bits of one that has ended. Register accesses and pad
 * samples call this first, so what software sees does not depend on how
 * promptly the timer callback runs.
 */
static void uart_tx_advance(ESP32UARTState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    while (s->tx_busy && s->tx_q32) {
        unsigned b = tx_next_change(s);
        int64_t t;

        if (b <= s->tx_nbits) {
            t = tx_time(s, 2 * b);
            if (t > now) {
                break;
            }
            s->tx_bit = b;
            uart_set_tx_line(s, tx_bit_level(s, b), t);
            continue;
        }
        t = tx_time(s, s->tx_half);
        if (t > now) {
            break;
        }
        s->tx_busy = false;
        if (uart_tx_can_start(s)) {
            uart_tx_start_at(s, t);
        }
    }
    uart_tx_schedule(s);
    esp32_uart_update_irq(s);
}

/* Start a frame now if the line is idle and a frame may start. */
static void uart_tx_start(ESP32UARTState *s)
{
    if (s->tx_busy || !uart_tx_can_start(s)) {
        return;
    }
    uart_tx_start_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    uart_tx_schedule(s);
    esp32_uart_update_irq(s);
}

/* Note how far the frame in flight has got, before the bit time changes. */
static void uart_tx_freeze(ESP32UARTState *s, int64_t now)
{
    uint64_t lo, hi;

    if (!s->tx_busy || !s->tx_q32) {
        return;
    }
    hi = (uint64_t)(now - s->tx_start_ns);
    lo = hi << 48;
    hi >>= 16;
    divu128(&lo, &hi, s->tx_q32);
    s->tx_pos16 = hi ? UINT64_MAX : lo;
}

/* Continue the frame in flight at the current bit time. */
static void uart_tx_rerate(ESP32UARTState *s, int64_t now)
{
    uint64_t q32 = uart_bit_q32(s);

    if (s->tx_busy && q32) {
        s->tx_start_ns = now - esp32_uart_mul_shr(s->tx_pos16, q32, 48);
    }
    s->tx_q32 = q32;
    uart_tx_schedule(s);
}

static void uart_tx_timer_cb(void *opaque)
{
    uart_tx_advance(ESP32_UART(opaque));
}

static void uart_line_sync(Notifier *n, void *data)
{
    uart_tx_advance(container_of(n, ESP32UARTState, line_source));
}

/* [spec:nuos:req:emu.esp32.clock-gating] */
static void uart_clk_update(void *opaque, ClockEvent event)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        uart_tx_advance(s);
        uart_tx_freeze(s, now);
        return;
    }
    uart_update_baud(s);
    uart_tx_rerate(s, now);
    uart_tx_start(s);
    if (!uart_running(s)) {
        timer_del(&s->rx_timeout_timer);
    }
    notifier_list_notify(&s->format_notifiers, s);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A frame has arrived on RXD. Parity and framing errors and breaks raise
 * their interrupts; with ERR_WR_MASK a frame in error is not stored. A
 * full RX FIFO drops the byte and raises RXFIFO_OVF.
 */
static void uart_rx_deliver(void *opaque, unsigned data, unsigned flags)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    uint32_t conf0 = s->reg[R_UART_CONF0];
    bool bad = flags & (ESP32_UART_RX_PARITY_ERR | ESP32_UART_RX_FRAME_ERR);

    if (flags & ESP32_UART_RX_PARITY_ERR) {
        s->reg[R_UART_INT_RAW] |= R_UART_INT_RAW_PARITY_ERR_MASK;
    }
    if (flags & ESP32_UART_RX_FRAME_ERR) {
        s->reg[R_UART_INT_RAW] |= R_UART_INT_RAW_FRM_ERR_MASK;
    }
    if (flags & ESP32_UART_RX_BREAK) {
        s->reg[R_UART_INT_RAW] |= R_UART_INT_RAW_BRK_DET_MASK;
    }
    if (!(bad && FIELD_EX32(conf0, UART_CONF0, ERR_WR_MASK))) {
        if (fifo8_is_full(&s->rx_fifo)) {
            s->reg[R_UART_INT_RAW] |= R_UART_INT_RAW_RXFIFO_OVF_MASK;
        } else {
            timer_del(&s->rx_timeout_timer);
            s->rxfifo_tout = false;
            fifo8_push(&s->rx_fifo, data);
        }
    }

    /* A UHCI attached to this UART takes the data from the RX FIFO. */
    notifier_list_notify(&s->dma_notifiers, s);

    esp32_uart_set_rx_timeout(s);
    uart_update_lines(s, true);
    esp32_uart_update_irq(s);
}

static bool uart_rx_format(void *opaque, Esp32UartFrameFmt *fmt)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    uart_format(s, fmt);
    return fmt->bit_q32 != 0;
}

static uint64_t uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    uint64_t r = 0;

    uart_tx_advance(s);
    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            r = 0xEE;
            error_report("esp_uart: read UART FIFO while it is empty");
        } else {
            r = fifo8_pop(&s->rx_fifo);
            uart_update_lines(s, true);
            esp32_uart_update_irq(s);
        }
        break;

    case A_UART_STATUS:
        r = FIELD_DP32(r, UART_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
        r = FIELD_DP32(r, UART_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
        /* TX_STRT while a frame is on the line, TX_IDLE otherwise */
        r = FIELD_DP32(r, UART_STATUS, ST_UTX_OUT, s->tx_busy ? 1 : 0);
        r = FIELD_DP32(r, UART_STATUS, ST_URX_OUT, s->rx.busy ? 1 : 0);
        r = FIELD_DP32(r, UART_STATUS, RXD, s->rx.level);
        r = FIELD_DP32(r, UART_STATUS, CTSN, s->cts_line);
        r = FIELD_DP32(r, UART_STATUS, DSRN, s->dsr_line);
        r = FIELD_DP32(r, UART_STATUS, TXD, s->tx_line);
        r = FIELD_DP32(r, UART_STATUS, RTSN, s->rts_line);
        r = FIELD_DP32(r, UART_STATUS, DTRN, s->dtr_line);
        break;

    case A_UART_LOWPULSE:
    case A_UART_HIGHPULSE:
        r = 337;  /* FIXME: this should depend on the APB frequency */
        break;
    case A_UART_MEM_CONF:
        r = FIELD_DP32(r, UART_MEM_CONF, RX_SIZE, (unsigned char)(UART_FIFO_LENGTH/128));
        r = FIELD_DP32(r, UART_MEM_CONF, TX_SIZE,  (unsigned char)(UART_FIFO_LENGTH/128));
        break;
    case A_UART_MEM_RX_STATUS: {
        uint32_t fifo_size = fifo8_num_used(&s->rx_fifo);
        /* The software only cares about the differene between WR_ADDR and RD_ADDR;
         * to keep things simpler, set RD_ADDR to 0 and WR_ADDR to the number of bytes
         * in the FIFO. 128 is a special case — write and read pointers should be
         * the same in this case.
         */
        r = FIELD_DP32(0, UART_MEM_RX_STATUS, WR_ADDR, (fifo_size == 128) ? 0 : fifo_size);
        }
        break;
    case A_UART_DATE:
        r = 0x15122500;
        break;
    default:
        r = s->reg[addr / 4];
        break;
    }

    return r;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * CONF0: the frame format and baud clock apply from the next bit time on;
 * the FIFO resets hold their FIFO empty while set; the line settings take
 * effect at once.
 */
static void uart_write_conf0(ESP32UARTState *s, uint32_t value)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    uart_tx_freeze(s, now);
    s->reg[R_UART_CONF0] = value;
    if (FIELD_EX32(value, UART_CONF0, RXFIFO_RST)) {
        fifo8_reset(&s->rx_fifo);
    }
    if (FIELD_EX32(value, UART_CONF0, TXFIFO_RST)) {
        fifo8_reset(&s->tx_fifo);
    }
    if (FIELD_EX32(value, UART_CONF0, IRDA_EN)) {
        qemu_log_mask(LOG_UNIMP, "esp32_uart: IrDA mode not implemented\n");
    }
    if (FIELD_EX32(value, UART_CONF0, TXD_BRK)) {
        qemu_log_mask(LOG_UNIMP,
                      "esp32_uart: sending a break (TXD_BRK) not "
                      "implemented\n");
    }
    uart_update_baud(s);
    uart_tx_rerate(s, now);
    uart_update_lines(s, true);
    uart_tx_start(s);
    notifier_list_notify(&s->format_notifiers, s);
}

static void uart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    uart_tx_advance(s);
    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            error_report("esp_uart: write to UART FIFO while it is full");
        } else {
            fifo8_push(&s->tx_fifo, (uint8_t) (value & 0xff));
            uart_tx_start(s);
        }
        break;

    case A_UART_INT_CLR:
        s->reg[R_UART_INT_ST] &= ~((uint32_t) value);
        s->reg[R_UART_INT_RAW] &= ~((uint32_t)value & UART_INT_LATCHED);
        s->reg[addr / 4] = value;
        if (value & R_UART_INT_CLR_RXFIFO_TOUT_MASK) {
            s->rxfifo_tout = false;
        }
        break;

    case A_UART_INT_ENA:
        s->reg[addr / 4] = value;
        break;

    case A_UART_CLKDIV:
        uart_tx_freeze(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        s->reg[addr / 4] = value;
        uart_update_baud(s);
        uart_tx_rerate(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        uart_tx_start(s);
        notifier_list_notify(&s->format_notifiers, s);
        break;

    case A_UART_CONF0:
        uart_write_conf0(s, value);
        break;

    case A_UART_AUTOBAUD:
        /* If autobaud is enabled, pretend that sufficient number of edges on the RXD line
         * have been received instantly. Autobaud is only used in the ROM bootloader,
         * and it doesn't care if the result is ready immediately.
         */
        if (FIELD_EX32(value, UART_AUTOBAUD, EN)) {
            s->reg[R_UART_RXD_CNT] = 0x3FF;
        } else {
            s->reg[R_UART_RXD_CNT] = 0;
        }
        s->reg[addr / 4] = value;
        break;

    case A_UART_INT_RAW:
    case A_UART_INT_ST:
    case A_UART_STATUS:
        /* no-op */
        break;

    case A_UART_CONF1:
        s->reg[addr / 4] = value;
        s->tx_empty_threshold = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TXFIFO_EMPTY_THRD);
        s->rx_full_threshold = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, RXFIFO_FULL_THRD);
        /* On the ESP32, rx_tout_thres is in units of (bit_time * 8).
         * Note this is different on later chips.
         */
        s->rx_tout_thres = 8 * FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TOUT_THRD);
        s->rx_tout_ena = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TOUT_EN) != 0;
        esp32_uart_set_rx_timeout(s);
        uart_update_lines(s, true);
        esp32_uart_update_irq(s);
        break;

    default:
        if (addr > sizeof(s->reg)) {
            error_report("esp_uart: write to addr=0x%x out of bounds\n", (uint32_t) addr);
        } else {
            s->reg[addr / 4] = value;
        }
        break;

    }
    esp32_uart_update_irq(s);
}

/* [spec:nuos:req:emu.esp32.uhci] */
void esp32_uart_add_dma_notifier(ESP32UARTState *s, Notifier *n)
{
    notifier_list_add(&s->dma_notifiers, n);
}

/* [spec:nuos:req:emu.esp32.uhci] */
unsigned esp32_uart_dma_tx_free(ESP32UARTState *s)
{
    return fifo8_num_free(&s->tx_fifo);
}

/* [spec:nuos:req:emu.esp32.uhci] */
void esp32_uart_dma_tx_push(ESP32UARTState *s, uint8_t byte)
{
    if (fifo8_num_free(&s->tx_fifo) == 0) {
        return;
    }
    fifo8_push(&s->tx_fifo, byte);
    uart_tx_start(s);
    esp32_uart_update_irq(s);
}

/* [spec:nuos:req:emu.esp32.uhci] */
unsigned esp32_uart_dma_rx_count(ESP32UARTState *s)
{
    return fifo8_num_used(&s->rx_fifo);
}

/* [spec:nuos:req:emu.esp32.uhci] Take a byte from the RX FIFO for the UHCI. */
uint8_t esp32_uart_dma_rx_pop(ESP32UARTState *s)
{
    uint8_t byte;

    if (fifo8_is_empty(&s->rx_fifo)) {
        return 0;
    }
    byte = fifo8_pop(&s->rx_fifo);
    uart_update_lines(s, true);
    esp32_uart_update_irq(s);
    return byte;
}

static void uart_rx_timeout_timer_cb(void *opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    s->rxfifo_tout = true;
    esp32_uart_update_irq(s);
}

static void uart_rxd_in(void *opaque, int n, int level)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    s->rxd_pin = level != 0;
    uart_rx_input(s, esp32_line_time());
}

static void uart_cts_in(void *opaque, int n, int level)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    s->cts_pin = level != 0;
    uart_update_inputs(s, true);
}

static void uart_dsr_in(void *opaque, int n, int level)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    s->dsr_pin = level != 0;
    uart_update_inputs(s, true);
}

/*
 * The pins' input levels belong to the outside world and are kept; the
 * outputs return to idle: TXD high, RTS and DTR inactive (high).
 */
static void esp32_uart_reset_hold(Object *obj, ResetType type)
{
    ESP32UARTState *s = ESP32_UART(obj);

    memset(s->reg, 0, sizeof(s->reg));
    s->reg[R_UART_RXD_CNT] = 0;
    s->reg[R_UART_INT_ST] = 0;
    s->reg[R_UART_INT_RAW] = 0;
    s->reg[R_UART_INT_ENA] = 0;
    s->reg[R_UART_AUTOBAUD] = 0;
    /* Default baud rate divider after reset */
    s->reg[R_UART_CLKDIV] = FIELD_DP32(0, UART_CLKDIV, CLKDIV, 0x2B6);
    s->reg[R_UART_CONF0] = UART_CONF0_RESET;
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    timer_del(&s->tx_timer);
    s->tx_busy = false;
    s->tx_bit = 0;
    s->tx_q32 = 0;
    s->tx_pos16 = 0;
    s->tx_line = true;
    s->rts_line = true;
    s->dtr_line = true;
    s->cts_line = s->cts_pin;
    s->dsr_line = s->dsr_pin;
    esp32_uart_rx_reset(&s->rx, s->rxd_pin);
    timer_del(&s->rx_timeout_timer);
    s->rxfifo_tout = false;
    uart_update_baud(s);
    s->rx_tout_ena = false;
    s->tx_empty_threshold = 0;
    s->rx_full_threshold = 0;
    s->rx_tout_thres = 0;
}

static void esp32_uart_reset_exit(Object *obj, ResetType type)
{
    ESP32UARTState *s = ESP32_UART(obj);

    uart_update_lines(s, false);
    esp32_uart_update_irq(s);
}

static void esp32_uart_realize(DeviceState *dev, Error **errp)
{
    esp32_line_add_source(&ESP32_UART(dev)->line_source);
}


static void esp32_uart_init(Object *obj)
{
    ESP32UARTState *s = ESP32_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);
    ESP32UARTClass *class = ESP32_UART_GET_CLASS(obj);

    s->uart_ops = (MemoryRegionOps) {
        .read =  class->uart_read,
        .write = class->uart_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    };

    memory_region_init_io(&s->iomem, obj, &s->uart_ops, s,
                          TYPE_ESP32_UART, UART_REG_CNT * sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    fifo8_create(&s->tx_fifo, UART_FIFO_LENGTH);
    fifo8_create(&s->rx_fifo, UART_FIFO_LENGTH);
    notifier_list_init(&s->dma_notifiers);
    notifier_list_init(&s->format_notifiers);
    timer_init_ns(&s->rx_timeout_timer, QEMU_CLOCK_VIRTUAL, uart_rx_timeout_timer_cb, s);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, uart_tx_timer_cb, s);
    esp32_uart_rx_init(&s->rx, uart_rx_format, uart_rx_deliver, s);
    s->apb_clk = qdev_init_clock_in(dev, "apb", uart_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->ref_tick_clk = qdev_init_clock_in(dev, "ref_tick",
                                         uart_clk_update, s,
                                         ClockPreUpdate | ClockUpdate);

    qdev_init_gpio_out_named(dev, &s->txd_out, ESP32_UART_TXD, 1);
    qdev_init_gpio_out_named(dev, &s->rts_out, ESP32_UART_RTS, 1);
    qdev_init_gpio_out_named(dev, &s->dtr_out, ESP32_UART_DTR, 1);
    qdev_init_gpio_in_named(dev, uart_rxd_in, ESP32_UART_RXD, 1);
    qdev_init_gpio_in_named(dev, uart_cts_in, ESP32_UART_CTS, 1);
    qdev_init_gpio_in_named(dev, uart_dsr_in, ESP32_UART_DSR, 1);
    /*
     * The inputs' levels until the matrix drives them: RXD idle high, CTS
     * and DSR at the matrix's default for unassigned inputs, low.
     */
    s->rxd_pin = true;
    s->line_source.notify = uart_line_sync;
}

static const VMStateDescription vmstate_esp32_uart = {
    .name = TYPE_ESP32_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(apb_clk, ESP32UARTState),
        VMSTATE_CLOCK(ref_tick_clk, ESP32UARTState),
        VMSTATE_UINT32_ARRAY(reg, ESP32UARTState, UART_REG_CNT),
        VMSTATE_FIFO8(rx_fifo, ESP32UARTState),
        VMSTATE_FIFO8(tx_fifo, ESP32UARTState),
        VMSTATE_TIMER(rx_timeout_timer, ESP32UARTState),
        VMSTATE_BOOL(rxfifo_tout, ESP32UARTState),
        VMSTATE_UINT32(baud_rate, ESP32UARTState),
        VMSTATE_BOOL(rxd_pin, ESP32UARTState),
        VMSTATE_BOOL(cts_pin, ESP32UARTState),
        VMSTATE_BOOL(dsr_pin, ESP32UARTState),
        VMSTATE_BOOL(tx_line, ESP32UARTState),
        VMSTATE_BOOL(rts_line, ESP32UARTState),
        VMSTATE_BOOL(dtr_line, ESP32UARTState),
        VMSTATE_BOOL(cts_line, ESP32UARTState),
        VMSTATE_BOOL(dsr_line, ESP32UARTState),
        VMSTATE_TIMER(tx_timer, ESP32UARTState),
        VMSTATE_BOOL(tx_busy, ESP32UARTState),
        VMSTATE_UINT16(tx_levels, ESP32UARTState),
        VMSTATE_UINT8(tx_nbits, ESP32UARTState),
        VMSTATE_UINT8(tx_half, ESP32UARTState),
        VMSTATE_UINT8(tx_bit, ESP32UARTState),
        VMSTATE_INT64(tx_start_ns, ESP32UARTState),
        VMSTATE_UINT64(tx_q32, ESP32UARTState),
        VMSTATE_UINT64(tx_pos16, ESP32UARTState),
        VMSTATE_STRUCT(rx, ESP32UARTState, 1, vmstate_esp32_uart_rx,
                       Esp32UartRx),
        VMSTATE_BOOL(rx_tout_ena, ESP32UARTState),
        VMSTATE_UINT32(rx_tout_thres, ESP32UARTState),
        VMSTATE_UINT32(tx_empty_threshold, ESP32UARTState),
        VMSTATE_UINT32(rx_full_threshold, ESP32UARTState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32UARTClass *class = ESP32_UART_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    /* Populate the virtual attributes and methods here (if any) */
    class->uart_write = uart_write;
    class->uart_read = uart_read;

    rc->phases.hold = esp32_uart_reset_hold;
    rc->phases.exit = esp32_uart_reset_exit;
    dc->realize = esp32_uart_realize;
    dc->vmsd = &vmstate_esp32_uart;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_uart_info = {
    .name = TYPE_ESP32_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32UARTState),
    .instance_init = esp32_uart_init,
    .class_init = esp32_uart_class_init,
    .class_size = sizeof(ESP32UARTClass)
};

static void esp32_uart_register_types(void)
{
    type_register_static(&esp32_uart_info);
}

type_init(esp32_uart_register_types)
