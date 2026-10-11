/*
 * Asynchronous serial frames on Espressif UART lines
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_ESP32_UART_LINE_H
#define HW_CHAR_ESP32_UART_LINE_H

#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "migration/vmstate.h"

/*
 * An asynchronous serial frame format: a start bit, data_bits data bits
 * LSB first, an optional parity bit and stop_half_bits / 2 stop bits, each
 * bit exactly @bit long (an ESP32P4ClkPeriod, as the UART's clock and
 * divider make it), stopped when the line can't run.
 */
typedef struct Esp32UartFrameFmt {
    uint8_t data_bits;
    bool parity_en;
    bool parity_odd;
    uint8_t stop_half_bits;
    ESP32P4ClkPeriod bit;
} Esp32UartFrameFmt;

/* Whether the line runs at @fmt's bit time. */
static inline bool esp32_uart_fmt_runs(const Esp32UartFrameFmt *fmt)
{
    return esp32p4_clk_period_running(fmt->bit);
}

/*
 * The first whole ns by which @half_bits half bits of @fmt have elapsed,
 * from the start of a frame (esp32p4_clk_cycles_ns()).
 */
int64_t esp32_uart_half_bits_ns(const Esp32UartFrameFmt *fmt,
                                uint64_t half_bits);

/*
 * The line levels of a frame carrying data, from the start bit: bit i of
 * the result is the level of bit i. Returns the number of bits before the
 * stop bits, which are high.
 */
unsigned esp32_uart_frame_levels(const Esp32UartFrameFmt *fmt, unsigned data,
                                 uint16_t *levels);

/* a * b / 2^shift, without overflow for any 64-bit a and b */
uint64_t esp32_uart_mul_shr(uint64_t a, uint64_t b, unsigned shift);

/*
 * A receiver decoding frames from the time-stamped level changes of a
 * line (see esp32_line_time()), as a UART samples its RXD: a falling edge
 * while idle starts a frame, whose bits are sampled at their middles; the
 * receiver looks for the next start bit from the middle of the first stop
 * bit. A frame is decoded once its stop bit's sample time has passed: at
 * that time by a timer, or earlier if a later change arrives first, so
 * the result does not depend on how promptly the timer runs. format()
 * gives the frame format as a start bit arrives, and false if the
 * receiver can't run; deliver() receives each frame's data and flags.
 *
 * The optional start() is told of each falling edge that starts a frame,
 * at its time, and glitch() of each such edge whose start bit's sample
 * finds the line high again, a glitch rather than a frame, as that sample
 * is taken. While
 * deliver() and glitch() run, frame_start_ns and frame_end_ns hold the
 * time of the frame's falling edge and of its stop bit's sample.
 */
#define ESP32_UART_RX_PARITY_ERR    (1u << 0)
#define ESP32_UART_RX_FRAME_ERR     (1u << 1)
#define ESP32_UART_RX_BREAK         (1u << 2)
/* Level changes kept per frame: enough for any frame, and some noise */
#define ESP32_UART_RX_EDGES         16

typedef bool Esp32UartRxFormat(void *opaque, Esp32UartFrameFmt *fmt);
typedef void Esp32UartRxDeliver(void *opaque, unsigned data, unsigned flags);
typedef void Esp32UartRxEvent(void *opaque);

typedef struct Esp32UartRx {
    QEMUTimer timer;
    Esp32UartRxFormat *format;
    Esp32UartRxDeliver *deliver;
    Esp32UartRxEvent *start;
    Esp32UartRxEvent *glitch;
    void *opaque;

    bool level;
    bool busy;
    /* More changes in the frame than edge_ns holds: it is noise */
    bool noise;
    Esp32UartFrameFmt fmt;
    uint8_t n_edges;
    int64_t edge_ns[ESP32_UART_RX_EDGES];
    int64_t frame_start_ns;
    int64_t frame_end_ns;
} Esp32UartRx;

void esp32_uart_rx_init(Esp32UartRx *rx, Esp32UartRxFormat *format,
                        Esp32UartRxDeliver *deliver, void *opaque);
void esp32_uart_rx_line(Esp32UartRx *rx, bool level, int64_t when_ns);
void esp32_uart_rx_reset(Esp32UartRx *rx, bool level);
/* Decode the frame in progress if its stop bit's sample time has passed. */
void esp32_uart_rx_flush(Esp32UartRx *rx);
extern const VMStateDescription vmstate_esp32_uart_rx;

/*
 * A UART whose lines a board-level device follows (the serial terminal of
 * hw/char/esp32_uart_terminal.h): get_format() gives the frame format and
 * bit time the UART is configured for, as a device on the other end of
 * its lines set up to match it sees them, and false while the UART's baud
 * clock is stopped; add_format_notifier() asks to be told whenever either
 * changes.
 */
#define TYPE_ESP32_UART_LINE_IF "esp32-uart-line-if"
typedef struct Esp32UartLineIfClass Esp32UartLineIfClass;
DECLARE_CLASS_CHECKERS(Esp32UartLineIfClass, ESP32_UART_LINE_IF,
                       TYPE_ESP32_UART_LINE_IF)

struct Esp32UartLineIfClass {
    InterfaceClass parent_class;

    bool (*get_format)(Object *uart, Esp32UartFrameFmt *fmt);
    void (*add_format_notifier)(Object *uart, Notifier *n);
};

bool esp32_uart_line_get_format(Object *uart, Esp32UartFrameFmt *fmt);
void esp32_uart_line_add_format_notifier(Object *uart, Notifier *n);

#endif
