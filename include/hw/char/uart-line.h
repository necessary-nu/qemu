/*
 * A host serial adapter on asynchronous serial lines
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
 * low for two frames, once the bytes before it have gone. The adapter has
 * no flow control. Its transmitter is a line source
 * (hw/core/line-time.h): a receiver's register access brings the "tx"
 * line up to its time first.
 *
 * The host's bytes wait for the line in a queue of UART_LINE_TX_QUEUE
 * (355 ms of the line at 115200 baud), as a host's serial driver buffers
 * what is written to its port, and the character device is read as the
 * queue has room. Without icount the transmitter's timer runs late, and
 * the queue gains room only as it runs: a receiver's access bringing the
 * line up to its time sends what the queue holds, and the host's next
 * bytes come once the main loop reads them. A queue holding less than
 * the line carries over the timer's lateness would leave the line idle
 * until then, which with the timer on time it never is.
 */
#define TYPE_UART_LINE "uart-line"
OBJECT_DECLARE_SIMPLE_TYPE(UARTLineState, UART_LINE)

#define UART_LINE_RX "rx"
#define UART_LINE_TX "tx"
#define UART_LINE_TX_QUEUE 4096

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
    /* A break from the host, after tx_break_after more bytes */
    bool tx_break;
    uint32_t tx_break_after;
    /* The chip's RX line as a source its receiver syncs (line_sync()). */
    Notifier line_source;
};

#endif
