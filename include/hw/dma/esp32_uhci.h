/*
 * ESP32 UHCI (UDMA), the UART DMA engine
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32_UHCI_H
#define HW_DMA_ESP32_UHCI_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/char/esp32_uart.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "system/memory.h"

#define TYPE_ESP32_UHCI "esp32.uhci"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32UhciState, ESP32_UHCI)

#define ESP32_UHCI_UART_COUNT   3
#define ESP32_UHCI_REG_SIZE     0x100
/*
 * Depth of each direction's DMA FIFO. The TRM does not give it; the model
 * takes 32 entries, which only bounds how far the engine runs ahead of
 * the UART.
 */
#define ESP32_UHCI_FIFO_DEPTH   32
/* Short packet registers: Q0..Q6, two words each */
#define ESP32_UHCI_QUICK_COUNT  7

/* The notifier through which an attached UART reports FIFO activity. */
typedef struct Esp32UhciUartLink {
    Notifier notifier;
    Esp32UhciState *uhci;
} Esp32UhciUartLink;

/*
 * A DMA FIFO: entries of a byte and flag bits, each with two words of
 * tag (the out FIFO's EOF entries carry their descriptor and buffer
 * addresses).
 */
typedef struct Esp32UhciFifo {
    uint16_t data[ESP32_UHCI_FIFO_DEPTH];
    uint32_t tag[ESP32_UHCI_FIFO_DEPTH][2];
    uint32_t head;
    uint32_t num;
} Esp32UhciFifo;

struct Esp32UhciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *apb_clk;

    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    ESP32UARTState *uart[ESP32_UHCI_UART_COUNT];
    Esp32UhciUartLink uart_link[ESP32_UHCI_UART_COUNT];

    /* Registers */
    uint32_t conf0;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t dma_out_push;
    uint32_t dma_in_pop;
    uint32_t dma_out_link;
    uint32_t dma_in_link;
    uint32_t conf1;
    uint32_t out_eof_des_addr;
    uint32_t in_suc_eof_des_addr;
    uint32_t in_err_eof_des_addr;
    uint32_t out_eof_bfr_des_addr;
    uint32_t ahb_test;
    uint32_t in_dscr[3];
    uint32_t out_dscr[3];
    uint32_t escape_conf;
    uint32_t hung_conf;
    uint32_t ack_num;
    uint32_t quick_sent;
    uint32_t q_word[ESP32_UHCI_QUICK_COUNT][2];
    uint32_t esc_conf[4];
    uint32_t pkt_thres;
    uint32_t date;

    /*
     * Outlink engine: memory -> encoder -> out FIFO -> UART TX FIFO.
     * out_desc is the descriptor being read, out_pos the next byte of its
     * buffer. out_last is the descriptor last finished, from whose next
     * field OUTLINK_RESTART continues.
     */
    bool out_active;
    bool out_parked;
    bool out_have_desc;
    bool out_in_frame;
    bool out_chain_done;
    uint32_t out_desc_addr;
    uint32_t out_desc_dw0;
    uint32_t out_desc_buf;
    uint32_t out_desc_next;
    uint32_t out_pos;
    uint32_t out_last_addr;
    bool out_have_last;
    uint32_t out_empty_run;
    Esp32UhciFifo out_fifo;
    /* Short packet being sent: its bytes, how far it has got, its kind */
    uint8_t quick_buf[8];
    uint32_t quick_pos;
    uint32_t quick_len;
    bool quick_always;
    bool quick_in_frame;

    /*
     * Inlink engine: UART RX FIFO -> decoder -> in FIFO -> memory.
     * in_desc is the descriptor being filled with in_len bytes.
     */
    bool in_active;
    bool in_parked;
    bool in_have_desc;
    bool in_empty_flagged;
    uint32_t in_desc_addr;
    uint32_t in_desc_dw0;
    uint32_t in_desc_buf;
    uint32_t in_desc_next;
    uint32_t in_len;
    uint32_t in_last_addr;
    bool in_have_last;
    bool in_frame;
    bool in_frame_data;
    bool in_esc;
    uint8_t in_esc_char;
    Esp32UhciFifo in_fifo;

    /* RX_HUNG (sending) and TX_HUNG (receiving) timeouts */
    QEMUTimer out_hung_timer;
    QEMUTimer in_hung_timer;
    bool out_hung_fired;
    bool in_hung_fired;

    bool in_kick;
    bool kick_again;
};

#endif
