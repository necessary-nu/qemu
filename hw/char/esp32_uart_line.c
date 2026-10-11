/*
 * Asynchronous serial frames on Espressif UART lines
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The frame encoding and the line-level receiver that the ESP32 and
 * ESP32-P4 UARTs and the board's serial terminals share; see
 * include/hw/char/esp32_uart_line.h.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/module.h"
#include "hw/char/esp32_uart_line.h"
#include "hw/gpio/esp32_line.h"

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
/* [spec:nuos:req:emu.esp32p4.uart] */
int64_t esp32_uart_half_bits_ns(const Esp32UartFrameFmt *fmt,
                                uint64_t half_bits)
{
    return esp32p4_clk_cycles_ns(esp32p4_clk_period_scale(fmt->bit, 1, 2),
                                 half_bits);
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.uart] */
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

/*
 * The middle of bit `bit` of the frame being received, from its start bit:
 * the first nanosecond at or after it.
 */
static int64_t rx_sample_ns(const Esp32UartRx *rx, unsigned bit)
{
    return rx->edge_ns[0] + esp32_uart_half_bits_ns(&rx->fmt, 2 * bit + 1);
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
 * [spec:nuos:req:emu.esp32p4.uart]
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

    rx->frame_start_ns = rx->edge_ns[0];
    rx->frame_end_ns = resume;
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
    } else if (rx->glitch) {
        rx->glitch(rx->opaque);
    }
    for (unsigned i = 0; i < n_rest; i++) {
        esp32_uart_rx_line(rx, !rx->level, rest[i]);
    }
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.uart] */
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
    if (level || !rx->format(rx->opaque, &rx->fmt) ||
        !esp32_uart_fmt_runs(&rx->fmt)) {
        return;
    }
    rx->busy = true;
    rx->n_edges = 1;
    rx->edge_ns[0] = when_ns;
    /* A receiver told of glitches looks at the start bit's middle first. */
    timer_mod_ns(&rx->timer, rx_sample_ns(rx, rx->glitch ? 0
                                                         : rx_stop_bit(rx)));
    if (rx->start) {
        rx->start(rx->opaque);
    }
}

/* Whether the frame in progress is to be decoded by t. */
static bool rx_due(const Esp32UartRx *rx, int64_t t)
{
    return rx->busy &&
           (t >= rx_sample_ns(rx, rx_stop_bit(rx)) ||
            (rx->glitch && t >= rx_sample_ns(rx, 0) &&
             rx_level_at(rx, rx_sample_ns(rx, 0))));
}

static void rx_sync_sources(void *opaque)
{
    esp32_line_sync();
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.uart]
 * Decode every frame whose stop bit's sample time has passed by now
 * (esp32_line_time()). Every change up to then must be in first: the
 * sources of line changes are brought up to that one moment before a
 * frame is decoded, not to a moment read afresh afterwards, which without
 * icount lies later and would decode a frame whose last bits a source has
 * yet to send. A change after it belongs to a later frame.
 */
void esp32_uart_rx_flush(Esp32UartRx *rx)
{
    int64_t t = esp32_line_time();

    if (rx->busy && (t >= rx_sample_ns(rx, rx_stop_bit(rx)) ||
                     (rx->glitch && t >= rx_sample_ns(rx, 0)))) {
        esp32_line_run_at(t, rx_sync_sources, NULL);
    }
    while (rx_due(rx, t)) {
        rx_finish(rx);
    }
}

static void rx_timer_cb(void *opaque)
{
    Esp32UartRx *rx = opaque;

    esp32_uart_rx_flush(rx);
    if (rx->busy) {
        timer_mod_ns(&rx->timer, rx_sample_ns(rx, rx_stop_bit(rx)));
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
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, Esp32UartRx),
        VMSTATE_BOOL(level, Esp32UartRx),
        VMSTATE_BOOL(busy, Esp32UartRx),
        VMSTATE_BOOL(noise, Esp32UartRx),
        VMSTATE_UINT8(fmt.data_bits, Esp32UartRx),
        VMSTATE_BOOL(fmt.parity_en, Esp32UartRx),
        VMSTATE_BOOL(fmt.parity_odd, Esp32UartRx),
        VMSTATE_UINT8(fmt.stop_half_bits, Esp32UartRx),
        VMSTATE_UINT64(fmt.bit.num_hi, Esp32UartRx),
        VMSTATE_UINT64(fmt.bit.num_lo, Esp32UartRx),
        VMSTATE_UINT64(fmt.bit.den, Esp32UartRx),
        VMSTATE_UINT8(n_edges, Esp32UartRx),
        VMSTATE_INT64_ARRAY(edge_ns, Esp32UartRx, ESP32_UART_RX_EDGES),
        VMSTATE_END_OF_LIST()
    }
};

bool esp32_uart_line_get_format(Object *uart, Esp32UartFrameFmt *fmt)
{
    return ESP32_UART_LINE_IF_GET_CLASS(uart)->get_format(uart, fmt);
}

void esp32_uart_line_add_format_notifier(Object *uart, Notifier *n)
{
    ESP32_UART_LINE_IF_GET_CLASS(uart)->add_format_notifier(uart, n);
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.uart] */
static const TypeInfo esp32_uart_line_if_info = {
    .name = TYPE_ESP32_UART_LINE_IF,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(Esp32UartLineIfClass),
};

static void esp32_uart_line_register_types(void)
{
    type_register_static(&esp32_uart_line_if_info);
}

type_init(esp32_uart_line_register_types)
