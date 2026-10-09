#pragma once

#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/clock.h"
#include "qemu/notify.h"
#include "migration/vmstate.h"

#define UART_FIFO_LENGTH 128

#define TYPE_ESP32_UART "esp_soc.uart"
#define ESP32_UART_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32UARTClass, obj, TYPE_ESP32_UART)
#define ESP32_UART_CLASS(klass) OBJECT_CLASS_CHECK(ESP32UARTClass, klass, TYPE_ESP32_UART)
#define ESP32_UART(obj) OBJECT_CHECK(ESP32UARTState, (obj), TYPE_ESP32_UART)

REG32(UART_FIFO, 0x0)
REG32(UART_INT_RAW, 0x4)
    FIELD(UART_INT_RAW, RXFIFO_FULL, 0, 1)
    FIELD(UART_INT_RAW, TXFIFO_EMPTY, 1, 1)
    FIELD(UART_INT_RAW, PARITY_ERR, 2, 1)
    FIELD(UART_INT_RAW, FRM_ERR, 3, 1)
    FIELD(UART_INT_RAW, RXFIFO_OVF, 4, 1)
    FIELD(UART_INT_RAW, DSR_CHG, 5, 1)
    FIELD(UART_INT_RAW, CTS_CHG, 6, 1)
    FIELD(UART_INT_RAW, BRK_DET, 7, 1)
    FIELD(UART_INT_RAW, RXFIFO_TOUT, 8, 1)
    FIELD(UART_INT_RAW, TX_DONE, 14, 1)
REG32(UART_INT_ST, 0x8)
REG32(UART_INT_ENA, 0xC)
REG32(UART_INT_CLR, 0x10)
    FIELD(UART_INT_CLR, RXFIFO_TOUT, 8, 1)

REG32(UART_CLKDIV, 0x14)
    FIELD(UART_CLKDIV, CLKDIV, 0, 20)
    FIELD(UART_CLKDIV, CLKDIV_FRAG, 20, 4)

REG32(UART_AUTOBAUD, 0x18)
    FIELD(UART_AUTOBAUD, EN, 0, 1)

REG32(UART_STATUS, 0x1C)
    FIELD(UART_STATUS, RXFIFO_CNT, 0, 8)
    FIELD(UART_STATUS, ST_URX_OUT, 8, 4)
    FIELD(UART_STATUS, DSRN, 13, 1)
    FIELD(UART_STATUS, CTSN, 14, 1)
    FIELD(UART_STATUS, RXD, 15, 1)
    FIELD(UART_STATUS, TXFIFO_CNT, 16, 8)
    FIELD(UART_STATUS, ST_UTX_OUT, 24, 4)
    FIELD(UART_STATUS, DTRN, 29, 1)
    FIELD(UART_STATUS, RTSN, 30, 1)
    FIELD(UART_STATUS, TXD, 31, 1)

REG32(UART_LOWPULSE, 0x28)
REG32(UART_HIGHPULSE, 0x2c)
REG32(UART_RXD_CNT, 0x30)

REG32(UART_CONF0, 0x20)
    FIELD(UART_CONF0, PARITY, 0, 1)
    FIELD(UART_CONF0, PARITY_EN, 1, 1)
    FIELD(UART_CONF0, BIT_NUM, 2, 2)
    FIELD(UART_CONF0, STOP_BIT_NUM, 4, 2)
    FIELD(UART_CONF0, SW_RTS, 6, 1)
    FIELD(UART_CONF0, SW_DTR, 7, 1)
    FIELD(UART_CONF0, TXD_BRK, 8, 1)
    FIELD(UART_CONF0, IRDA_EN, 16, 1)
    FIELD(UART_CONF0, LOOPBACK, 14, 1)
    FIELD(UART_CONF0, TX_FLOW_EN, 15, 1)
    FIELD(UART_CONF0, RXFIFO_RST, 17, 1)
    FIELD(UART_CONF0, TXFIFO_RST, 18, 1)
    FIELD(UART_CONF0, RXD_INV, 19, 1)
    FIELD(UART_CONF0, CTS_INV, 20, 1)
    FIELD(UART_CONF0, DSR_INV, 21, 1)
    FIELD(UART_CONF0, TXD_INV, 22, 1)
    FIELD(UART_CONF0, RTS_INV, 23, 1)
    FIELD(UART_CONF0, DTR_INV, 24, 1)
    FIELD(UART_CONF0, ERR_WR_MASK, 26, 1)
    FIELD(UART_CONF0, TICK_REF_ALWAYS_ON, 27, 1)
/* 8 data bits, 1 stop bit, clocked from APB_CLK */
#define UART_CONF0_RESET 0x0800001c
REG32(UART_CONF1, 0x24)
    FIELD(UART_CONF1, TOUT_EN, 31, 1)
    FIELD(UART_CONF1, TOUT_THRD, 24, 7)
    FIELD(UART_CONF1, RX_FLOW_EN, 23, 1)
    FIELD(UART_CONF1, RX_FLOW_THRHD, 16, 7)
    FIELD(UART_CONF1, TXFIFO_EMPTY_THRD, 8, 7)
    FIELD(UART_CONF1, RXFIFO_FULL_THRD, 0, 7)

REG32(UART_MEM_CONF, 0x58);
    FIELD(UART_MEM_CONF, RX_SIZE, 3, 4);
    FIELD(UART_MEM_CONF, TX_SIZE, 7, 4);
REG32(UART_MEM_RX_STATUS, 0x60);
    FIELD(UART_MEM_RX_STATUS, RD_ADDR, 2, 11);
    FIELD(UART_MEM_RX_STATUS, WR_ADDR, 13, 11);
REG32(UART_DATE, 0x78)

/* Size of the register file */
#define UART_REG_CNT (R_UART_DATE + 1)

/*
 * The UART's lines, by the GPIO matrix's signals (U0TXD_out, U0RXD_in,
 * ...). Outputs are named gpio-outs, inputs named gpio-ins, all one line.
 */
#define ESP32_UART_TXD  "esp32-uart-txd"
#define ESP32_UART_RTS  "esp32-uart-rts"
#define ESP32_UART_DTR  "esp32-uart-dtr"
#define ESP32_UART_RXD  "esp32-uart-rxd"
#define ESP32_UART_CTS  "esp32-uart-cts"
#define ESP32_UART_DSR  "esp32-uart-dsr"

/*
 * An asynchronous serial frame format: a start bit, data_bits data bits
 * LSB first, an optional parity bit and stop_half_bits / 2 stop bits, each
 * bit bit_q32 / 2^32 ns long. bit_q32 is 0 when the line can't run.
 */
typedef struct Esp32UartFrameFmt {
    uint8_t data_bits;
    bool parity_en;
    bool parity_odd;
    uint8_t stop_half_bits;
    uint64_t bit_q32;
} Esp32UartFrameFmt;

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
 */
#define ESP32_UART_RX_PARITY_ERR    (1u << 0)
#define ESP32_UART_RX_FRAME_ERR     (1u << 1)
#define ESP32_UART_RX_BREAK         (1u << 2)
/* Level changes kept per frame: enough for any frame, and some noise */
#define ESP32_UART_RX_EDGES         16

typedef bool Esp32UartRxFormat(void *opaque, Esp32UartFrameFmt *fmt);
typedef void Esp32UartRxDeliver(void *opaque, unsigned data, unsigned flags);

typedef struct Esp32UartRx {
    QEMUTimer timer;
    Esp32UartRxFormat *format;
    Esp32UartRxDeliver *deliver;
    void *opaque;

    bool level;
    bool busy;
    /* More changes in the frame than edge_ns holds: it is noise */
    bool noise;
    Esp32UartFrameFmt fmt;
    uint8_t n_edges;
    int64_t edge_ns[ESP32_UART_RX_EDGES];
} Esp32UartRx;

void esp32_uart_rx_init(Esp32UartRx *rx, Esp32UartRxFormat *format,
                        Esp32UartRxDeliver *deliver, void *opaque);
void esp32_uart_rx_line(Esp32UartRx *rx, bool level, int64_t when_ns);
void esp32_uart_rx_reset(Esp32UartRx *rx, bool level);
extern const VMStateDescription vmstate_esp32_uart_rx;

typedef struct ESPUARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer rx_timeout_timer;
    bool rxfifo_tout;
    unsigned baud_rate;

    /*
     * The baud clock is APB_CLK or REF_TICK, chosen by
     * CONF0.TICK_REF_ALWAYS_ON. The SoC stops both while DPORT gates the
     * UART or holds it in reset.
     */
    Clock *apb_clk;
    Clock *ref_tick_clk;

    /* Line levels at the pins, as driven and as seen */
    qemu_irq txd_out;
    qemu_irq rts_out;
    qemu_irq dtr_out;
    bool rxd_pin;
    bool cts_pin;
    bool dsr_pin;
    /* The UART core's side of the lines, after inversion and loopback */
    bool tx_line;
    bool rts_line;
    bool dtr_line;
    bool cts_line;
    bool dsr_line;

    /*
     * Transmitter: the frame in tx_levels (tx_nbits bits before the stop
     * bits, tx_half half bits in all) began at tx_start_ns, with bits
     * tx_q32 / 2^32 ns long; bit tx_bit is on the line. tx_q32 is 0 while
     * the baud clock is stopped, and the frame has then reached
     * tx_pos16 / 2^16 bits.
     */
    QEMUTimer tx_timer;
    bool tx_busy;
    uint16_t tx_levels;
    uint8_t tx_nbits;
    uint8_t tx_half;
    uint8_t tx_bit;
    int64_t tx_start_ns;
    uint64_t tx_q32;
    uint64_t tx_pos16;

    Esp32UartRx rx;

    Fifo8 rx_fifo;
    Fifo8 tx_fifo;

    uint32_t reg[UART_REG_CNT];
    MemoryRegionOps uart_ops;

    /* Protected: fields can be modified by the child class  */
    bool rx_tout_ena;
    /* Threshold, in bits, before triggering an RX timeout interrupt  */
    uint32_t rx_tout_thres;
    /* Threshold, in bytes, for a full RX FIFO and an empty TX FIFO respectively */
    uint32_t tx_empty_threshold;
    uint32_t rx_full_threshold;

    /*
     * UART DMA (UHCI) attachment: notified, with the UART as data, when a
     * byte leaves the TX FIFO or arrives in the RX FIFO, so an attached
     * UHCI can refill or drain the FIFOs.
     */
    NotifierList dma_notifiers;
    /* Told when the frame format or the baud rate changes */
    NotifierList format_notifiers;
    Notifier line_source;
} ESP32UARTState;

typedef struct ESPUARTClass {
    SysBusDeviceClass parent_class;

    /* Virtual attributes/methods */
    void (*uart_write)(void *opaque, hwaddr addr, uint64_t value, unsigned int size);
    uint64_t (*uart_read)(void *opaque, hwaddr addr, unsigned int size);
} ESP32UARTClass;


/**
 * @brief Enable the RX timeout according to the protected members of ESP32UARTState, as such, their
 * values must be set and valid when calling this function.
 */
void esp32_uart_set_rx_timeout(ESP32UARTState *s);


/**
 * @brief Check the current state of the UART FIFOs and trigger an interrupt if enabled and if any reached
 * the configured threshold.
 */
void esp32_uart_update_irq(ESP32UARTState *s);

/* UART DMA (UHCI) access to the UART's FIFOs, as the hardware's DMA port. */
void esp32_uart_add_dma_notifier(ESP32UARTState *s, Notifier *n);
unsigned esp32_uart_dma_tx_free(ESP32UARTState *s);
void esp32_uart_dma_tx_push(ESP32UARTState *s, uint8_t byte);
unsigned esp32_uart_dma_rx_count(ESP32UARTState *s);
uint8_t esp32_uart_dma_rx_pop(ESP32UARTState *s);

/*
 * The frame format and bit time the UART is configured for, as a device
 * on the other end of its lines set up to match it sees them; false while
 * the UART's baud clock is stopped.
 */
bool esp32_uart_line_format(ESP32UARTState *s, Esp32UartFrameFmt *fmt);
void esp32_uart_add_format_notifier(ESP32UARTState *s, Notifier *n);
