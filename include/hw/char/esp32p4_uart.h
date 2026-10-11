/*
 * ESP32-P4 UART controllers: UART0 to UART4 and the LP UART
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_ESP32P4_UART_H
#define HW_CHAR_ESP32P4_UART_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/char/esp32_uart_line.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_UART "esp32p4-uart"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4UartState, ESP32P4_UART)

#define ESP32P4_UART0_BASE      0x500ca000
#define ESP32P4_UART_STRIDE     0x1000
#define ESP32P4_LP_UART_BASE    0x50121000
#define ESP32P4_UART_COUNT      5
#define ESP32P4_UART_MMIO_SIZE  0x1000

/* The FIFOs: 128 bytes each way on UART0 to 4, 16 on the LP UART. */
#define ESP32P4_UART_FIFO_LEN       128
#define ESP32P4_LP_UART_FIFO_LEN    16

/* Register words, 0x00 to 0x9c. */
#define ESP32P4_UART_NREGS      (0xa0 / 4)

/*
 * How the machine connects a UART
 * ===============================
 *
 * Property "name": its register block's name in the address map.
 * Property "lp": the LP UART, a cut-down UART with 16-byte FIFOs whose
 * FIFO counts and thresholds sit 3 bits up in their fields (TRM table
 * 45.2-1, lp_uart_reg.h), and without RS485, IrDA or autobaud detection.
 *
 * Clock inputs: "sclk", the UART core clock (HP_SYS_CLKRST's
 * UARTn_SCLK, after its fractional divider; the LP UART's LP_UART_SCLK),
 * from which the baud rate, the glitch filter and autobaud detection are
 * timed; and "sys", the AHB clock of the FIFO RAM port (UARTn_SYS_CLK;
 * the LP UART's bus clock), without which FIFO_REG accesses fail. The
 * registers are mapped behind the bus clock and the APB reset
 * (RST_EN_UARTn_APB; LPPERI's LP_UART reset) with
 * esp32p4_clkrst_map_gated(), which resets the whole UART.
 *
 * Named GPIOs, levels as the matrix carries them (ESP32P4_SIG_UARTn_*):
 *   outputs ESP32P4_UART_TXD, _RTS, _DTR (high while idle);
 *   inputs ESP32P4_UART_RXD, _CTS, _DSR;
 *   input ESP32P4_UART_CORE_RESET: high while RST_EN_UARTn_CORE holds the
 *     UART core (transmitter, receiver, FIFOs) in reset;
 *   input ESP32P4_UART_WAKEUP_EN: the PMU's UART_WAKEUP_EN, high while
 *     the wakeup logic watches the receiver (in sleep, as ESP-IDF sets
 *     the PMU up);
 *   output ESP32P4_UART_WAKEUP: high from a wakeup until UART_WAKEUP_EN
 *     falls, a PMU wakeup source; the LP UART's also goes to LPPERI
 *     ("lp-uart-wakeup").
 * The interrupt is sysbus IRQ 0, the UARTn_INTR / LP_UART_INTR source.
 */
#define ESP32P4_UART_TXD            "txd"
#define ESP32P4_UART_RTS            "rts"
#define ESP32P4_UART_DTR            "dtr"
#define ESP32P4_UART_RXD            "rxd"
#define ESP32P4_UART_CTS            "cts"
#define ESP32P4_UART_DSR            "dsr"
#define ESP32P4_UART_CORE_RESET     "core-reset"
#define ESP32P4_UART_WAKEUP_EN      "wakeup-en"
#define ESP32P4_UART_WAKEUP         "wakeup"

/*
 * The UART's port to UHCI, which moves its data to and from GDMA
 * (hw/dma/esp32p4_uhci.c). A UHCI attaches to each UART it can select;
 * the UART tells it when its TX FIFO has room (tx_room), when bytes arrive
 * in its RX FIFO (rx_data), when the receiver has been idle for
 * RX_IDLE_THRHD bit times after a byte (rx_idle) and when it receives a
 * NULL character (rx_break). The UHCI pushes into the TX FIFO and pops
 * from the RX FIFO as the UART's FIFO RAM port allows.
 */
typedef struct ESP32P4UartDmaOps {
    void (*tx_room)(void *opaque);
    void (*rx_data)(void *opaque);
    void (*rx_idle)(void *opaque);
    void (*rx_break)(void *opaque);
} ESP32P4UartDmaOps;

void esp32p4_uart_dma_attach(ESP32P4UartState *s,
                             const ESP32P4UartDmaOps *ops, void *opaque);
unsigned esp32p4_uart_dma_tx_free(ESP32P4UartState *s);
bool esp32p4_uart_dma_tx_push(ESP32P4UartState *s, uint8_t byte);
unsigned esp32p4_uart_dma_rx_count(ESP32P4UartState *s);
bool esp32p4_uart_dma_rx_pop(ESP32P4UartState *s, uint8_t *byte);

typedef struct ESP32P4UartFifo {
    uint8_t data[ESP32P4_UART_FIFO_LEN];
    /* The RAM offsets read and written next, and the bytes held. */
    uint8_t rd;
    uint8_t wr;
    uint8_t num;
} ESP32P4UartFifo;

/* The transmitter's phases. */
enum {
    ESP32P4_UART_TX_IDLE,
    ESP32P4_UART_TX_DL0,
    ESP32P4_UART_TX_FRAME,
    ESP32P4_UART_TX_DL1,
    ESP32P4_UART_TX_BREAK,
    ESP32P4_UART_TX_GAP,
};

/* Level changes of the longest waveform the transmitter plays. */
#define ESP32P4_UART_TX_EDGES   32

struct ESP32P4UartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq txd_out;
    qemu_irq rts_out;
    qemu_irq dtr_out;
    qemu_irq wakeup_out;
    Clock *sclk;
    Clock *sys_clk;
    bool lp;
    /* The name of its register block, as the address map has it. */
    char *name;
    unsigned fifo_len;

    /* The registers as software wrote them (APB_CLK domain). */
    uint32_t regs[ESP32P4_UART_NREGS];
    /* The _SYNC registers as the core last took them (UART_SCLK domain). */
    uint32_t core[ESP32P4_UART_NREGS];
    uint32_t int_raw;
    QEMUTimer update_timer;
    bool update_pending;

    ESP32P4UartFifo tx_fifo;
    ESP32P4UartFifo rx_fifo;

    /* Inputs: the pins as driven, the wakeup enable and the core reset. */
    bool rxd_pin;
    bool cts_pin;
    bool dsr_pin;
    bool wakeup_en;
    bool core_reset;
    /* The core's side of the lines, after inversion and loopback. */
    bool tx_line;
    bool rts_line;
    bool dtr_line;
    bool cts_line;
    bool dsr_line;
    bool rx_in;

    /*
     * Transmitter: the current phase's waveform, its edges at tx_t16[]
     * sixteenths of a bit from tx_start_ns, the phase lasting tx_end16;
     * bits tx_q32 / 2^32 ns long, or 0 while the transmitter's clock is
     * stopped, the phase having then reached tx_pos16 / 2^16 sixteenths.
     */
    QEMUTimer tx_timer;
    uint8_t tx_phase;
    uint8_t tx_n_edges;
    uint8_t tx_next_edge;
    uint16_t tx_t16[ESP32P4_UART_TX_EDGES];
    uint8_t tx_lvl[ESP32P4_UART_TX_EDGES];
    uint32_t tx_end16;
    int64_t tx_start_ns;
    uint64_t tx_q32;
    uint64_t tx_pos16;
    /* When the transmitter last went idle. */
    int64_t tx_free_ns;
    /* The frame on the line: its byte, bit count, and its stop bits. */
    uint8_t tx_byte;
    uint8_t tx_frame_bits;
    uint8_t tx_data_bits;
    bool tx_parity;
    /* A transfer (frames since the transmitter was last idle) is going. */
    bool tx_in_transfer;
    /* Software flow control: XOFF received, XOFF sent and not yet XON. */
    bool sw_xoff_rcvd;
    bool sw_xoff_sent;
    bool send_xon;
    bool send_xoff;

    /* Receiver. */
    Esp32UartRx rx;
    /* The glitch filter's edge waiting to pass, and when it arrived. */
    QEMUTimer filt_timer;
    bool filt_pending;
    bool filt_level;
    int64_t filt_ns;
    bool filt_out;
    /* The last byte's end, for the RX timeout, idle and AT_CMD timers. */
    QEMUTimer tout_timer;
    QEMUTimer idle_timer;
    int64_t rx_last_end_ns;
    bool rx_had_byte;

    /* IrDA receiver: pulses decoded to NRZ (low for a bit per pulse). */
    QEMUTimer irda_timer;
    bool irda_low;
    int64_t irda_high_ns;
    bool irda_pin;

    /* Autobaud detection, counted in UART core clock cycles. */
    uint32_t rxd_edge_cnt;
    uint32_t lowpulse;
    uint32_t highpulse;
    uint32_t pospulse;
    uint32_t negpulse;
    int64_t ab_last_edge_ns;
    int64_t ab_last_rise_ns;
    int64_t ab_last_fall_ns;
    bool ab_level;
    bool ab_active;

    /* AT_CMD detection. */
    QEMUTimer at_timer;
    uint32_t at_count;
    int64_t at_last_end_ns;
    bool at_seen_byte;

    /* Wakeup: rising edges counted and the last bytes received in sleep. */
    uint32_t wake_edges;
    uint8_t wake_chars[5];
    uint8_t wake_nchars;
    bool wakeup;

    /* RS485: the last frame sent, for comparing its echo. */
    int64_t echo_start_ns;
    int64_t echo_end_ns;
    uint8_t echo_byte;

    const ESP32P4UartDmaOps *dma_ops;
    void *dma_opaque;

    NotifierList format_notifiers;
    Notifier line_source;
};

#endif
