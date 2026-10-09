/*
 * Asynchronous serial lines: edge timing and a host serial adapter
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_UART_LINE_H
#define HW_CHAR_UART_LINE_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "qom/object.h"

/*
 * Edge timing.
 *
 * A UART modelled at line level changes its TX line from QEMU_CLOCK_VIRTUAL
 * timers at the bit boundaries, and a receiver samples the line from its
 * own timers in the middle of each bit. Timers run in deadline order, but
 * may all run late together, so a receiver must not time a frame from the
 * clock reading at the edge: it times it from the edge's deadline. A
 * transmitter changes its line with uart_line_set(), giving the time the
 * change belongs to; a receiver reads that time with uart_line_now(),
 * which outside such a change is the current virtual time.
 */
int64_t uart_line_now(void);
void uart_line_set(qemu_irq irq, int level, int64_t when);

/*
 * A host serial adapter on a UART's lines, for a board to attach a
 * character device to the pins of a UART, as a USB serial adapter or a
 * debug probe is wired to a board's UART header.
 *
 * The adapter runs 8N1 at the fixed "baud" rate, as the host set it; a
 * UART configured differently produces framing errors, which are
 * dropped. Named GPIO input "rx" is the line the adapter receives on (the
 * chip's TX pin): a level of 0 or 1, or a negative level while nothing
 * drives it, which the adapter's pull-up reads as 1. Named GPIO output
 * "tx" is the line the adapter drives (the chip's RX pin), high while
 * idle. A line held low for a whole frame is a break, passed to the
 * character device as one; a break from the character device holds "tx"
 * low for two frames. The adapter has no flow control.
 */
#define TYPE_UART_LINE "uart-line"
OBJECT_DECLARE_SIMPLE_TYPE(UARTLineState, UART_LINE)

#define UART_LINE_RX "rx"
#define UART_LINE_TX "tx"
#define UART_LINE_TX_QUEUE 16

struct UARTLineState {
    SysBusDevice parent_obj;

    CharFrontend chr;
    uint32_t baud;
    qemu_irq tx;

    /* Receiver: the chip's TX line to the character device. */
    QEMUTimer *rx_timer;
    int64_t rx_start;
    int64_t rx_next;
    int32_t rx_bit;
    uint32_t rx_data;
    uint8_t rx_level;
    bool rx_zero;
    bool rx_break;

    /* Transmitter: the character device to the chip's RX line. */
    QEMUTimer *tx_timer;
    Fifo8 tx_queue;
    int64_t tx_start;
    int64_t tx_next;
    uint32_t tx_frame;
    int32_t tx_len;
    int32_t tx_bit;
    uint8_t txd;
    bool tx_break;
};

#endif
