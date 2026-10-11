/*
 * ESP32-P4 UHCI, the UARTs' interface to GDMA
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32P4_UHCI_H
#define HW_DMA_ESP32P4_UHCI_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/char/esp32p4_uart.h"
#include "hw/dma/esp32p4_gdma.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_UHCI "esp32p4-uhci"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4UhciState, ESP32P4_UHCI)

#define ESP32P4_UHCI_BASE       0x500df000
#define ESP32P4_UHCI_MMIO_SIZE  0x1000
#define ESP32P4_UHCI_QUICK_COUNT 7

/*
 * The machine links the UHCI to GDMA-AHB ("gdma"), on which it is
 * peripheral ESP32P4_GDMA_AHB_UHCI0, and to UART0 to UART4 ("uart0" to
 * "uart4"), one of which CONF0.UART_SEL connects. Clock input "clk" is
 * its bus clock, which times its hang timeouts. Sysbus IRQ 0 is
 * UHCI0_INTR.
 */

/* The encoder's bytes for the UART: a separator or escape pair at most. */
#define ESP32P4_UHCI_OUT_QUEUE  4

/* What a UART's notification carries: the UHCI and which UART it is. */
typedef struct ESP32P4UhciLink {
    ESP32P4UhciState *s;
    ESP32P4UartState *uart;
} ESP32P4UhciLink;

struct ESP32P4UhciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4GdmaState *gdma;
    ESP32P4UartState *uart[ESP32P4_UART_COUNT];
    ESP32P4UhciLink link[ESP32P4_UART_COUNT];

    uint32_t conf0;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf1;
    uint32_t escape_conf;
    uint32_t hung_conf;
    uint32_t ack_num;
    uint32_t quick_sent;
    uint32_t q_word[ESP32P4_UHCI_QUICK_COUNT][2];
    uint32_t esc_conf[4];
    uint32_t pkt_thres;
    uint32_t date;

    /* Encoder: bytes for the UART, each with the events it raises. */
    uint16_t out_q[ESP32P4_UHCI_OUT_QUEUE];
    uint8_t out_q_num;
    bool out_in_frame;
    /* SW_START written, for a frame waiting with WAIT_SW_START. */
    bool out_sw_start;
    /* A short packet being sent: its bytes, how far, and its kind. */
    uint8_t quick_buf[8];
    uint8_t quick_len;
    uint8_t quick_pos;
    bool quick_always;
    bool quick_in_frame;

    /* Decoder. */
    bool in_frame;
    bool in_frame_data;
    bool in_esc;
    uint8_t in_esc_char;
    uint32_t in_count;
    /* After a frame's PKT_THRS bytes, the rest up to its separator. */
    bool in_skip;
    /*
     * The UART's idle or a NULL character ended the frame: its EOF goes
     * after the in_eof_left bytes of it still in the UART's RX FIFO.
     */
    bool in_eof_pending;
    uint32_t in_eof_left;

    /* Hang timeouts. */
    QEMUTimer out_hung_timer;
    QEMUTimer in_hung_timer;
    bool out_hung_fired;
    bool in_hung_fired;

    bool in_kick;
    bool kick_again;
};

#endif
