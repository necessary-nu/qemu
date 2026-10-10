/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_PL011_H
#define HW_PL011_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_PL011 "pl011"
OBJECT_DECLARE_SIMPLE_TYPE(PL011State, PL011)

/* This shares the same struct (and cast macro) as the base pl011 device */
#define TYPE_PL011_LUMINARY "pl011_luminary"

/* Depth of UART FIFO in bytes, when FIFO mode is enabled (else depth == 1) */
#define PL011_FIFO_DEPTH 16
/* The deeper FIFOs of revision r1p5, selected by property "fifo-depth" */
#define PL011_FIFO_MAX 32

/*
 * Named GPIO output array: the single-transfer DMA requests, TX
 * (UARTTXDMASREQ) and RX (UARTRXDMASREQ). The burst requests and the
 * DMACLR handshake are not modelled.
 */
#define PL011_DMA_REQ "dma-req"
#define PL011_DMA_TX 0
#define PL011_DMA_RX 1

/*
 * Line-level mode (property "line-level"), for boards that route the
 * UART's pins. The UART has no character device; instead it shifts each
 * character out on UARTTXD at the baud rate its divisors set from the
 * "clk" input, and samples UARTRXD in the middle of each bit, with
 * hardware flow control on nUARTRTS and nUARTCTS:
 *
 *   named GPIO output "txd":  UARTTXD, high while idle
 *   named GPIO output "nrts": nUARTRTS, low while asserted
 *   named GPIO input "rxd":   UARTRXD
 *   named GPIO input "ncts":  nUARTCTS, low while asserted
 *
 * The other modem signals, nUARTDTR, nUARTDSR, nUARTDCD, nUARTRI and the
 * OUT1/OUT2 outputs, are not connected: their status flags read 0.
 */
#define PL011_TXD "txd"
#define PL011_NRTS "nrts"
#define PL011_RXD "rxd"
#define PL011_NCTS "ncts"

struct PL011State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t flags;
    uint32_t lcr;
    uint32_t rsr;
    uint32_t cr;
    uint32_t dmacr;
    uint32_t int_enabled;
    uint32_t int_level;
    uint32_t read_fifo[PL011_FIFO_MAX];
    uint32_t ilpr;
    uint32_t ibrd;
    uint32_t fbrd;
    uint32_t ifl;
    int read_pos;
    int read_count;
    int read_trigger;
    CharFrontend chr;
    qemu_irq irq[6];
    qemu_irq dma_req[2];
    Clock *clk;
    bool migrate_clk;
    bool logged_disabled_uart;
    uint32_t fifo_depth;

    /* Line-level mode */
    bool line_level;
    qemu_irq txd_out;
    qemu_irq nrts_out;
    QEMUTimer *tx_timer;
    QEMUTimer *rx_timer;
    QEMUTimer *rt_timer;
    /* IBRD:FBRD as UARTLCR_H writes latch them, 16.6 fixed point */
    uint32_t brd;
    uint8_t tx_fifo[PL011_FIFO_MAX];
    int32_t tx_pos;
    int32_t tx_count;
    /* The frame in the shift register, LSB first, and its timing */
    bool tx_busy;
    uint32_t tx_frame;
    int32_t tx_len;
    int32_t tx_bit;
    uint32_t tx_brd;
    int64_t tx_start;
    int64_t tx_next;
    /* The receiver: bit being sampled (-1 idle) and the frame so far */
    int32_t rx_bit;
    uint32_t rx_data;
    uint32_t rx_brd;
    uint32_t rx_lcr;
    int64_t rx_start;
    int64_t rx_next;
    bool rx_zero;
    bool rx_wait_mark;
    bool rx_overrun;
    /*
     * Across a change of UARTCLK: the UARTCLK cycles since the transmit
     * and receive frames started, and left of the receive timeout, held
     * while UARTCLK is stopped.
     */
    uint64_t tx_held;
    uint64_t rx_held;
    uint64_t rt_held;
    /* Line levels: pins in, the receiver's inputs after loopback, outputs */
    uint8_t rxd_pin;
    uint8_t ncts_pin;
    uint8_t rx_level;
    uint8_t ncts_level;
    uint8_t txd;
    uint8_t nrts;
    const unsigned char *id;
    /*
     * Since some users embed this struct directly, we must
     * ensure that the C struct is at least as big as the Rust one.
     */
    uint8_t padding_for_rust[16];
};

DeviceState *pl011_create(hwaddr addr, qemu_irq irq, Chardev *chr);

#endif
