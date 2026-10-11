/*
 * A board-level serial terminal on an Espressif UART's pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_ESP32_UART_TERMINAL_H
#define HW_CHAR_ESP32_UART_TERMINAL_H

#include "hw/core/qdev.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/char/esp32_uart_line.h"

#define TYPE_ESP32_UART_TERMINAL "esp32.uart-terminal"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32UartTerminalState, ESP32_UART_TERMINAL)

/*
 * The terminal's lines: RX (gpio-in) watches the pad carrying the chip's
 * TXD; TX (gpio-out) drives the pad carrying the chip's RXD from outside.
 */
#define ESP32_UART_TERMINAL_RX  "esp32-uart-terminal-rx"
#define ESP32_UART_TERMINAL_TX  "esp32-uart-terminal-tx"

/* Bytes held for a chardev that can't take them yet */
#define ESP32_UART_TERMINAL_BUF 4096
/* Bytes from the chardev waiting to be sent, as a bridge's FIFO holds them */
#define ESP32_UART_TERMINAL_IN_BUF 256

struct Esp32UartTerminalState {
    DeviceState parent_obj;

    CharFrontend chr;
    /* The UART whose frame format and baud rate the terminal is set to */
    Object *uart;

    Esp32UartRx rx;
    Fifo8 out;
    guint out_watch;
    Fifo8 in;

    /*
     * Transmitter: like the UART's, the frame in tx_levels began tx_off
     * half bits after tx_start_ns, with bits exactly tx_bit_time long (the
     * frames of a back-to-back run counted from the first's start), and
     * bit tx_bit is on the line; tx_chained while the next frame is to
     * follow the last without a gap.
     */
    qemu_irq tx_out;
    QEMUTimer tx_timer;
    bool tx_busy;
    uint16_t tx_levels;
    uint8_t tx_nbits;
    uint8_t tx_half;
    uint8_t tx_bit;
    int64_t tx_start_ns;
    uint64_t tx_off;
    ESP32P4ClkPeriod tx_bit_time;
    bool tx_chained;

    Notifier format_notifier;
    Notifier line_source;
};

#endif
