/*
 * ESP32-P4 UART controllers: UART0 to UART4 and the LP UART
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM chapter 45 (UART Controller), with ESP-IDF's
 * rev3 uart_reg.h and lp_uart_reg.h for the register layout, access types
 * and reset values (the ESP32-C6's layout), and its uart_ll.h for how
 * software drives the block.
 *
 * The UART works at the line level, like hw/char/esp32_uart.c: its TXD,
 * RTS and DTR outputs and RXD, CTS and DSR inputs are GPIO matrix signals.
 * The transmitter plays each frame onto TXD edge by edge at the baud rate
 * the UART core clock and CLKDIV give, and the receiver decodes frames
 * from the time-stamped edges of RXD (hw/char/esp32_uart_line.c), so that
 * whatever is on the other end (the board's serial terminal, another UART
 * through the matrix, the UART itself in loopback) sees the bits at their
 * times.
 *
 * The registers are in the APB_CLK domain; the core (transmitter,
 * receiver, flow control, autobaud, AT_CMD and wakeup detection) works
 * from the _SYNC registers as REG_UPDATE last carried them into the
 * UART_SCLK domain (TRM 45.5.1), which takes three core clock cycles and
 * waits while that clock is stopped or the core held in reset.
 *
 * Behaviour the TRM leaves open follows the ESP32 TRM, whose UART this
 * one descends from: the FSM_STATUS state numbers (ESP32 TRM register
 * UART_STATUS_REG, with ESP-IDF's TX_WAIT_SEND, 0xF, for a transmitter
 * stopped by XOFF with data waiting); and TX_IDLE_NUM as the idle time the
 * transmitter keeps after a transfer, once the TX FIFO has emptied, before
 * it starts the next (ESP-IDF's uart_set_tx_idle_num()). Fractional baud
 * dividers spread their extra core clock cycles over the bits (TRM figure
 * 45.4-1); the model gives every bit the average length.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/esp32p4_uart.h"
#include "hw/gpio/esp32_line.h"
#include "migration/vmstate.h"

#define A_FIFO              0x00
#define A_INT_RAW           0x04
#define A_INT_ST            0x08
#define A_INT_ENA           0x0c
#define A_INT_CLR           0x10
#define A_CLKDIV            0x14
#define A_RX_FILT           0x18
#define A_STATUS            0x1c
#define A_CONF0             0x20
#define A_CONF1             0x24
#define A_HWFC_CONF         0x2c
#define A_SLEEP_CONF0       0x30
#define A_SLEEP_CONF1       0x34
#define A_SLEEP_CONF2       0x38
#define A_SWFC_CONF0        0x3c
#define A_SWFC_CONF1        0x40
#define A_TXBRK_CONF        0x44
#define A_IDLE_CONF         0x48
#define A_RS485_CONF        0x4c
#define A_AT_CMD_PRECNT     0x50
#define A_AT_CMD_POSTCNT    0x54
#define A_AT_CMD_GAPTOUT    0x58
#define A_AT_CMD_CHAR       0x5c
#define A_MEM_CONF          0x60
#define A_TOUT_CONF         0x64
#define A_MEM_TX_STATUS     0x68
#define A_MEM_RX_STATUS     0x6c
#define A_FSM_STATUS        0x70
#define A_POSPULSE          0x74
#define A_NEGPULSE          0x78
#define A_LOWPULSE          0x7c
#define A_HIGHPULSE         0x80
#define A_RXD_CNT           0x84
#define A_CLK_CONF          0x88
#define A_DATE              0x8c
#define A_AFIFO_STATUS      0x90
#define A_REG_UPDATE        0x98
#define A_ID                0x9c

#define R(a)                ((a) / 4)

/* Interrupts, INT_RAW, INT_ST, INT_ENA and INT_CLR. */
#define INT_RXFIFO_FULL     BIT(0)
#define INT_TXFIFO_EMPTY    BIT(1)
#define INT_PARITY_ERR      BIT(2)
#define INT_FRM_ERR         BIT(3)
#define INT_RXFIFO_OVF      BIT(4)
#define INT_DSR_CHG         BIT(5)
#define INT_CTS_CHG         BIT(6)
#define INT_BRK_DET         BIT(7)
#define INT_RXFIFO_TOUT     BIT(8)
#define INT_SW_XON          BIT(9)
#define INT_SW_XOFF         BIT(10)
#define INT_GLITCH_DET      BIT(11)
#define INT_TX_BRK_DONE     BIT(12)
#define INT_TX_BRK_IDLE     BIT(13)
#define INT_TX_DONE         BIT(14)
#define INT_RS485_PARITY    BIT(15)
#define INT_RS485_FRM       BIT(16)
#define INT_RS485_CLASH     BIT(17)
#define INT_AT_CMD_DET      BIT(18)
#define INT_WAKEUP          BIT(19)
#define INT_MASK            0x000fffffu
#define LP_INT_MASK         0x000c7fffu

/* CLKDIV_SYNC */
#define CLKDIV_INT(v)       ((v) & 0xfff)
#define CLKDIV_FRAG(v)      (((v) >> 20) & 0xf)

/* RX_FILT */
#define RX_FILT_WIDTH(v)    ((v) & 0xff)
#define RX_FILT_EN          BIT(8)

/* CONF0_SYNC */
#define CONF0_PARITY        BIT(0)
#define CONF0_PARITY_EN     BIT(1)
#define CONF0_BIT_NUM(v)    (((v) >> 2) & 3)
#define CONF0_STOP_NUM(v)   (((v) >> 4) & 3)
#define CONF0_TXD_BRK       BIT(6)
#define CONF0_IRDA_DPLX     BIT(7)
#define CONF0_IRDA_TX_EN    BIT(8)
#define CONF0_IRDA_WCTL     BIT(9)
#define CONF0_IRDA_TX_INV   BIT(10)
#define CONF0_IRDA_RX_INV   BIT(11)
#define CONF0_LOOPBACK      BIT(12)
#define CONF0_TX_FLOW_EN    BIT(13)
#define CONF0_IRDA_EN       BIT(14)
#define CONF0_RXD_INV       BIT(15)
#define CONF0_TXD_INV       BIT(16)
#define CONF0_DIS_RX_OVF    BIT(17)
#define CONF0_ERR_WR_MASK   BIT(18)
#define CONF0_AUTOBAUD_EN   BIT(19)
#define CONF0_SW_RTS        BIT(21)
#define CONF0_RXFIFO_RST    BIT(22)
#define CONF0_TXFIFO_RST    BIT(23)

/* CONF1 */
#define CONF1_CTS_INV       BIT(16)
#define CONF1_DSR_INV       BIT(17)
#define CONF1_RTS_INV       BIT(18)
#define CONF1_DTR_INV       BIT(19)
#define CONF1_SW_DTR        BIT(20)

/* HWFC_CONF_SYNC */
#define HWFC_RX_FLOW_EN     BIT(8)

/* SLEEP_CONF2 */
#define SLEEP2_ACTIVE_THRESHOLD(v)  ((v) & 0x3ff)
#define SLEEP2_WK_CHAR_NUM(v)       (((v) >> 18) & 7)
#define SLEEP2_WK_CHAR_MASK(v)      (((v) >> 21) & 0x1f)
#define SLEEP2_WK_MODE(v)           (((v) >> 26) & 3)

/* SWFC_CONF0_SYNC */
#define SWFC_XON_CHAR(v)    ((v) & 0xff)
#define SWFC_XOFF_CHAR(v)   (((v) >> 8) & 0xff)
#define SWFC_STILL_SEND     BIT(16)
#define SWFC_EN             BIT(17)
#define SWFC_XONOFF_DEL     BIT(18)
#define SWFC_FORCE_XON      BIT(19)
#define SWFC_FORCE_XOFF     BIT(20)
#define SWFC_SEND_XON       BIT(21)
#define SWFC_SEND_XOFF      BIT(22)

/* IDLE_CONF_SYNC */
#define IDLE_RX_THRHD(v)    ((v) & 0x3ff)
#define IDLE_TX_NUM(v)      (((v) >> 10) & 0x3ff)

/* RS485_CONF_SYNC */
#define RS485_EN            BIT(0)
#define RS485_DL0_EN        BIT(1)
#define RS485_DL1_EN        BIT(2)
#define RS485_TX_RX_EN      BIT(3)
#define RS485_RXBY_TX_EN    BIT(4)
#define RS485_RX_DLY        BIT(5)
#define RS485_TX_DLY(v)     (((v) >> 6) & 0xf)

/* AT_CMD_CHAR_SYNC */
#define AT_CHAR(v)          ((v) & 0xff)
#define AT_CHAR_NUM(v)      (((v) >> 8) & 0xff)

/* MEM_CONF */
#define MEM_FORCE_PD        BIT(25)
#define MEM_FORCE_PU        BIT(26)

/* TOUT_CONF_SYNC */
#define TOUT_EN             BIT(0)
#define TOUT_FLOW_DIS       BIT(1)
#define TOUT_THRHD(v)       (((v) >> 2) & 0x3ff)

/* CLK_CONF */
#define CLK_TX_SCLK_EN      BIT(24)
#define CLK_RX_SCLK_EN      BIT(25)
#define CLK_TX_RST_CORE     BIT(26)
#define CLK_RX_RST_CORE     BIT(27)

/* The TX FSM states of FSM_STATUS.ST_UTX_OUT and ST_URX_OUT. */
#define FSM_IDLE            0
#define FSM_START           1
#define FSM_DATA0           2
#define FSM_PARITY          10
#define FSM_STOP1           11
#define FSM_STOP2           12
#define FSM_DL0             13
#define FSM_DL1             14
#define FSM_TX_WAIT_SEND    0xf

/*
 * Rising RXD edges past ACTIVE_THRESHOLD that wake the chip in wakeup
 * mode 0: six on UART0 to 4 and three on the LP UART, as ESP-IDF's
 * uart_ll_set_wakeup_edge_thrd() has it for this chip (the TRM gives
 * three for both).
 */
#define WAKE_EDGE_OFFSET    6
#define LP_WAKE_EDGE_OFFSET 3

/* Core clock cycles REG_UPDATE takes to carry the _SYNC registers over. */
#define UPDATE_SYNC_CYCLES  3

#define AB_MAX              0xfff
#define EDGE_CNT_MAX        0x3ff

typedef struct UartRegInfo {
    uint32_t reset;
    uint32_t rw;
    uint32_t lp_rw;
    bool sync;
} UartRegInfo;

/* The registers software can write, their reset values and masks. */
static const UartRegInfo uart_regs[ESP32P4_UART_NREGS] = {
    [R(A_INT_ENA)]       = { 0x00000000, INT_MASK, LP_INT_MASK, false },
    [R(A_CLKDIV)]        = { 0x000002b6, 0x00f00fff, 0x00f00fff, true },
    [R(A_RX_FILT)]       = { 0x00000008, 0x000001ff, 0x000001ff, false },
    [R(A_CONF0)]         = { 0x0000001c, 0x00ffffff, 0x00f7b07f, true },
    [R(A_CONF1)]         = { 0x00006060, 0x003fffff, 0x003ff8f8, false },
    [R(A_HWFC_CONF)]     = { 0x00000000, 0x000001ff, 0x000001f8, true },
    [R(A_SLEEP_CONF0)]   = { 0x00000000, 0xffffffff, 0xffffffff, false },
    [R(A_SLEEP_CONF1)]   = { 0x00000000, 0x000000ff, 0x000000ff, false },
    [R(A_SLEEP_CONF2)]   = { 0x001404f0, 0x0fffffff, 0x0fffe3ff, false },
    [R(A_SWFC_CONF0)]    = { 0x00001311, 0x007fffff, 0x007fffff, true },
    [R(A_SWFC_CONF1)]    = { 0x0000e000, 0x0000ffff, 0x0000f8f8, false },
    [R(A_TXBRK_CONF)]    = { 0x0000000a, 0x000000ff, 0x000000ff, true },
    [R(A_IDLE_CONF)]     = { 0x00040100, 0x000fffff, 0x000fffff, true },
    [R(A_RS485_CONF)]    = { 0x00000000, 0x000003ff, 0x00000006, true },
    [R(A_AT_CMD_PRECNT)] = { 0x00000901, 0x0000ffff, 0x0000ffff, true },
    [R(A_AT_CMD_POSTCNT)] = { 0x00000901, 0x0000ffff, 0x0000ffff, true },
    [R(A_AT_CMD_GAPTOUT)] = { 0x0000000b, 0x0000ffff, 0x0000ffff, true },
    [R(A_AT_CMD_CHAR)]   = { 0x0000032b, 0x0000ffff, 0x0000ffff, true },
    [R(A_MEM_CONF)]      = { 0x00000000, 0x06000000, 0x06000000, false },
    [R(A_TOUT_CONF)]     = { 0x00000028, 0x00000fff, 0x00000fff, true },
    [R(A_CLK_CONF)]      = { 0x03000000, 0x0f000000, 0x0f000000, false },
    [R(A_DATE)]          = { 0x02305050, 0xffffffff, 0xffffffff, false },
    [R(A_ID)]            = { 0x00000500, 0xffffffff, 0xffffffff, false },
};

/* The LP UART's SLEEP_CONF2 default: RX_WAKE_UP_THRHD 1 in bits 17:13. */
#define LP_SLEEP_CONF2_RESET    0x001420f0
/* The LP UART's SWFC_CONF1 default: XOFF_THRESHOLD 12 in bits 15:11. */
#define LP_SWFC_CONF1_RESET     0x00006000

static void uart_tx_try_start(ESP32P4UartState *s, int64_t t);
static void uart_tx_advance(ESP32P4UartState *s);
static void uart_update_irq(ESP32P4UartState *s);
static void uart_update_lines(ESP32P4UartState *s, int64_t t);
static void uart_rx_input(ESP32P4UartState *s, int64_t t);
static void uart_tout_fire(ESP32P4UartState *s);
static void uart_idle_fire(ESP32P4UartState *s);
static void uart_at_fire(ESP32P4UartState *s);

static int64_t now_ns(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint32_t reg_reset(ESP32P4UartState *s, unsigned r)
{
    if (s->lp && r == R(A_SLEEP_CONF2)) {
        return LP_SLEEP_CONF2_RESET;
    }
    if (s->lp && r == R(A_SWFC_CONF1)) {
        return LP_SWFC_CONF1_RESET;
    }
    return uart_regs[r].reset;
}

/*
 * A FIFO count or threshold field: on the LP UART the five-bit values sit
 * three bits up in the eight-bit fields of UART0 to 4.
 */
static unsigned field8(ESP32P4UartState *s, uint32_t v, unsigned shift)
{
    return ((v >> shift) & 0xff) >> (s->lp ? 3 : 0);
}

static uint32_t put8(ESP32P4UartState *s, unsigned v, unsigned shift)
{
    return ((v << (s->lp ? 3 : 0)) & 0xff) << shift;
}

static uint32_t cconf0(ESP32P4UartState *s)
{
    return s->core[R(A_CONF0)];
}

static bool rs485(ESP32P4UartState *s)
{
    return !s->lp && (s->core[R(A_RS485_CONF)] & RS485_EN);
}

static bool irda(ESP32P4UartState *s)
{
    return !s->lp && (cconf0(s) & CONF0_IRDA_EN);
}

/* ---- FIFOs ---- */

static void fifo_reset(ESP32P4UartFifo *f)
{
    f->rd = 0;
    f->wr = 0;
    f->num = 0;
}

static bool fifo_push(ESP32P4UartState *s, ESP32P4UartFifo *f, uint8_t b)
{
    if (f->num >= s->fifo_len) {
        return false;
    }
    f->data[f->wr] = b;
    f->wr = (f->wr + 1) % s->fifo_len;
    f->num++;
    return true;
}

static uint8_t fifo_pop(ESP32P4UartState *s, ESP32P4UartFifo *f)
{
    uint8_t b = f->data[f->rd];

    f->rd = (f->rd + 1) % s->fifo_len;
    f->num--;
    return b;
}

/* ---- Clocks and timing ---- */

static bool core_running(ESP32P4UartState *s)
{
    return clock_is_enabled(s->sclk) && !s->core_reset;
}

static bool tx_running(ESP32P4UartState *s)
{
    uint32_t c = s->regs[R(A_CLK_CONF)];

    return core_running(s) && (c & CLK_TX_SCLK_EN) && !(c & CLK_TX_RST_CORE);
}

static bool rx_running(ESP32P4UartState *s)
{
    uint32_t c = s->regs[R(A_CLK_CONF)];

    return core_running(s) && (c & CLK_RX_SCLK_EN) && !(c & CLK_RX_RST_CORE);
}

static bool fifo_port_running(ESP32P4UartState *s)
{
    return clock_is_enabled(s->sys_clk);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The bit time in 2^-32 ns: (CLKDIV + CLKDIV_FRAG / 16) periods of the
 * UART core clock at its current frequency (TRM 45.4.3.1); 0 while the
 * core cannot run or the divider is 0.
 */
static uint64_t uart_bit_q32(ESP32P4UartState *s)
{
    uint32_t div = s->core[R(A_CLKDIV)];
    uint64_t div16 = ((uint64_t)CLKDIV_INT(div) << 4) | CLKDIV_FRAG(div);

    if (!core_running(s) || div16 == 0) {
        return 0;
    }
    return esp32_uart_mul_shr(clock_get(s->sclk), div16, 4);
}

static uint64_t tx_bit_q32(ESP32P4UartState *s)
{
    return tx_running(s) ? uart_bit_q32(s) : 0;
}

/* `bits` bit times, in ns, at the current baud rate; 0 if stopped. */
static int64_t bits_ns(ESP32P4UartState *s, uint64_t bits)
{
    return esp32_uart_mul_shr(bits, uart_bit_q32(s), 32);
}

/* Whole UART core clock cycles in `ns`. */
static uint64_t sclk_cycles(ESP32P4UartState *s, int64_t ns)
{
    if (ns <= 0) {
        return 0;
    }
    return muldiv64(ns, clock_get_hz(s->sclk), NANOSECONDS_PER_SECOND);
}

static void uart_format(ESP32P4UartState *s, Esp32UartFrameFmt *fmt)
{
    static const uint8_t stop_half_bits[] = { 2, 2, 3, 4 };
    uint32_t c0 = cconf0(s);

    fmt->data_bits = 5 + CONF0_BIT_NUM(c0);
    fmt->parity_en = c0 & CONF0_PARITY_EN;
    fmt->parity_odd = c0 & CONF0_PARITY;
    fmt->stop_half_bits = stop_half_bits[CONF0_STOP_NUM(c0)];
    fmt->bit_q32 = uart_bit_q32(s);
}

static void uart_format_changed(ESP32P4UartState *s)
{
    notifier_list_notify(&s->format_notifiers, s);
}

/* ---- Interrupts ---- */

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * INT_RAW bits latch until cleared (R/WTC/SS). The FIFO threshold ones
 * are set for as long as their condition holds, so clearing one while it
 * holds sets it again: TXFIFO_EMPTY while the TX FIFO holds fewer bytes
 * than TXFIFO_EMPTY_THRHD, RXFIFO_FULL while the RX FIFO holds
 * RXFIFO_FULL_THRHD or more (TRM 45.4.2).
 */
static void uart_update_irq(ESP32P4UartState *s)
{
    uint32_t conf1 = s->regs[R(A_CONF1)];
    uint32_t mask = s->lp ? LP_INT_MASK : INT_MASK;

    if (s->tx_fifo.num < field8(s, conf1, 8)) {
        s->int_raw |= INT_TXFIFO_EMPTY;
    }
    if (s->rx_fifo.num >= field8(s, conf1, 0)) {
        s->int_raw |= INT_RXFIFO_FULL;
    }
    s->int_raw &= mask;
    qemu_set_irq(s->irq, !!(s->int_raw & s->regs[R(A_INT_ENA)]));
}

static void uart_raise(ESP32P4UartState *s, uint32_t bits)
{
    s->int_raw |= bits;
    uart_update_irq(s);
}

/* ---- Wakeup ---- */

static void uart_wake(ESP32P4UartState *s)
{
    if (!s->light_sleep || s->wakeup) {
        return;
    }
    s->wakeup = true;
    qemu_set_irq(s->wakeup_out, 1);
    uart_raise(s, INT_WAKEUP);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The wakeup conditions of TRM 45.4.8, while the chip is in light sleep:
 * mode 0, enough rising edges on RXD; mode 1, more bytes in the RX FIFO
 * than RX_WAKE_UP_THRHD; mode 2 (a start bit) is checked as one arrives
 * and mode 3 (a character sequence) as each byte does.
 */
static void uart_wake_check(ESP32P4UartState *s)
{
    uint32_t c2 = s->regs[R(A_SLEEP_CONF2)];

    if (!s->light_sleep || s->wakeup) {
        return;
    }
    switch (SLEEP2_WK_MODE(c2)) {
    case 0:
        if (s->wake_edges >= SLEEP2_ACTIVE_THRESHOLD(c2) +
            (s->lp ? LP_WAKE_EDGE_OFFSET : WAKE_EDGE_OFFSET)) {
            uart_wake(s);
        }
        break;
    case 1:
        if (s->rx_fifo.num > field8(s, c2 >> 10, 0)) {
            uart_wake(s);
        }
        break;
    default:
        break;
    }
}

/*
 * Mode 3: the last WK_CHAR_NUM bytes received are WK_CHARn to WK_CHAR4,
 * n = 5 - WK_CHAR_NUM, a character whose WK_CHAR_MASK bit is set matching
 * any byte (TRM table 45.4-1).
 */
static void uart_wake_char(ESP32P4UartState *s, uint8_t b)
{
    uint32_t c2 = s->regs[R(A_SLEEP_CONF2)];
    unsigned num = SLEEP2_WK_CHAR_NUM(c2), mask = SLEEP2_WK_CHAR_MASK(c2);
    uint8_t want[5];

    if (!s->light_sleep || s->wakeup || SLEEP2_WK_MODE(c2) != 3) {
        return;
    }
    memmove(s->wake_chars, s->wake_chars + 1, 4);
    s->wake_chars[4] = b;
    if (s->wake_nchars < 5) {
        s->wake_nchars++;
    }
    if (num == 0 || num > 5 || s->wake_nchars < num) {
        return;
    }
    want[0] = s->regs[R(A_SLEEP_CONF1)] & 0xff;
    want[1] = s->regs[R(A_SLEEP_CONF0)] & 0xff;
    want[2] = (s->regs[R(A_SLEEP_CONF0)] >> 8) & 0xff;
    want[3] = (s->regs[R(A_SLEEP_CONF0)] >> 16) & 0xff;
    want[4] = (s->regs[R(A_SLEEP_CONF0)] >> 24) & 0xff;
    for (unsigned i = 5 - num; i < 5; i++) {
        if (!(mask & BIT(i)) && s->wake_chars[i] != want[i]) {
            return;
        }
    }
    uart_wake(s);
}

/* ---- Lines and flow control ---- */

static bool tx_idle_level(ESP32P4UartState *s)
{
    return !irda(s);
}

/* The TXD pin's level for the core's TX line level. */
static bool txd_pin_level(ESP32P4UartState *s, bool line)
{
    uint32_t c0 = cconf0(s);
    bool v = line ^ !!(c0 & CONF0_TXD_INV);

    if (irda(s) && (c0 & CONF0_IRDA_TX_INV)) {
        v = !v;
    }
    return v;
}

/* The transmitter drives DE, on DTR in RS485 mode, through its frames. */
static bool tx_driving(ESP32P4UartState *s)
{
    return s->tx_phase == ESP32P4_UART_TX_DL0 ||
           s->tx_phase == ESP32P4_UART_TX_FRAME ||
           s->tx_phase == ESP32P4_UART_TX_DL1 ||
           s->tx_phase == ESP32P4_UART_TX_BREAK;
}

static bool rts_blocking(ESP32P4UartState *s)
{
    return (s->core[R(A_HWFC_CONF)] & HWFC_RX_FLOW_EN) && s->rts_line;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * CTS and DSR as the core sees them (TRM figure 45.4-8): the pins
 * inverted by CTS_INV and DSR_INV, or in loopback the UART's own RTS and
 * DTR. A change raises CTS_CHG or DSR_CHG, and CTS going active (low)
 * lets a flow-controlled transmitter go on.
 */
static void uart_update_inputs(ESP32P4UartState *s, int64_t t, bool raise)
{
    uint32_t conf1 = s->regs[R(A_CONF1)];
    bool lb = cconf0(s) & CONF0_LOOPBACK;
    bool cts = lb ? s->rts_line : s->cts_pin ^ !!(conf1 & CONF1_CTS_INV);
    bool dsr = lb ? s->dtr_line : s->dsr_pin ^ !!(conf1 & CONF1_DSR_INV);

    if (cts != s->cts_line) {
        s->cts_line = cts;
        if (raise) {
            s->int_raw |= INT_CTS_CHG;
        }
        if (!cts) {
            uart_tx_try_start(s, t);
        }
    }
    if (dsr != s->dsr_line) {
        s->dsr_line = dsr;
        if (raise) {
            s->int_raw |= INT_DSR_CHG;
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Drive RTS and DTR (TRM 45.4.9.1). RTS is active low: with RX_FLOW_EN
 * the receiver drives it, inactive while the RX FIFO holds more than
 * RX_FLOW_THRHD bytes; otherwise SW_RTS sets it active. DTR follows
 * SW_DTR, set active low, except in RS485 mode, where it is the
 * transceiver's driver enable, high while the transmitter sends. The pins
 * carry them inverted by RTS_INV and DTR_INV.
 */
static void uart_update_lines(ESP32P4UartState *s, int64_t t)
{
    uint32_t c0 = cconf0(s), conf1 = s->regs[R(A_CONF1)];
    bool was_blocking = rts_blocking(s);

    if (s->core[R(A_HWFC_CONF)] & HWFC_RX_FLOW_EN) {
        s->rts_line = s->rx_fifo.num >
                      field8(s, s->core[R(A_HWFC_CONF)], 0);
    } else {
        s->rts_line = !(c0 & CONF0_SW_RTS);
    }
    if (rs485(s)) {
        s->dtr_line = tx_driving(s);
    } else {
        s->dtr_line = !(conf1 & CONF1_SW_DTR);
    }
    esp32_line_set_at(s->rts_out, s->rts_line ^ !!(conf1 & CONF1_RTS_INV), t);
    esp32_line_set_at(s->dtr_out, s->dtr_line ^ !!(conf1 & CONF1_DTR_INV), t);
    uart_update_inputs(s, t, true);
    if (was_blocking && !rts_blocking(s) && s->rx_had_byte &&
        (s->core[R(A_TOUT_CONF)] & TOUT_FLOW_DIS)) {
        /* The idle count restarts once flow control lets data in again. */
        s->rx_last_end_ns = t;
        timer_mod_ns(&s->tout_timer, t + bits_ns(s,
                     TOUT_THRHD(s->core[R(A_TOUT_CONF)])));
    }
}

/*
 * Software flow control (TRM 45.4.9.2): with SW_FLOW_CON_EN the UART sends
 * XOFF once its RX FIFO holds more than XOFF_THRESHOLD bytes and XON once
 * it holds fewer than XON_THRESHOLD again.
 */
static void uart_sw_flow_check(ESP32P4UartState *s, int64_t t)
{
    uint32_t c1 = s->regs[R(A_SWFC_CONF1)];

    if (!(s->core[R(A_SWFC_CONF0)] & SWFC_EN)) {
        return;
    }
    if (!s->sw_xoff_sent && s->rx_fifo.num > field8(s, c1, 8)) {
        s->sw_xoff_sent = true;
        s->send_xoff = true;
        uart_tx_try_start(s, t);
    } else if (s->sw_xoff_sent && s->rx_fifo.num < field8(s, c1, 0)) {
        s->sw_xoff_sent = false;
        s->send_xon = true;
        uart_tx_try_start(s, t);
    }
}

/* The RX FIFO changed: RTS, XON/XOFF, wakeup and the interrupts follow. */
static void uart_rx_fifo_changed(ESP32P4UartState *s, int64_t t)
{
    uart_update_lines(s, t);
    uart_sw_flow_check(s, t);
    uart_wake_check(s);
    uart_update_irq(s);
}

/* ---- Transmitter ---- */

static int64_t tx_time16(ESP32P4UartState *s, uint64_t t16)
{
    return s->tx_start_ns + esp32_uart_mul_shr(t16, s->tx_q32, 36);
}

/* Put the transmitter's level on TXD at t, and on the receiver in loopback. */
static void uart_set_tx_line(ESP32P4UartState *s, bool level, int64_t t)
{
    uint32_t c0 = cconf0(s);

    s->tx_line = level;
    esp32_line_set_at(s->txd_out, txd_pin_level(s, level), t);
    if (irda(s) ? (c0 & CONF0_IRDA_DPLX) : (c0 & CONF0_LOOPBACK)) {
        uart_rx_input(s, t);
    }
}

static void tx_wave_add(ESP32P4UartState *s, uint16_t t16, bool level)
{
    unsigned n = s->tx_n_edges;
    bool prev = n ? s->tx_lvl[n - 1] : tx_idle_level(s);

    if (level != prev && n < ESP32P4_UART_TX_EDGES) {
        s->tx_t16[n] = t16;
        s->tx_lvl[n] = level;
        s->tx_n_edges = n + 1;
    }
}

/*
 * Bit `i` of a waveform, at sixteenths 16i to 16i + 16: NRZ, or in IrDA
 * mode a high pulse in the 9th to 11th sixteenths for a 0 and nothing for
 * a 1 (TRM 45.4.7).
 */
static void tx_wave_bit(ESP32P4UartState *s, unsigned i, bool v)
{
    if (!irda(s)) {
        tx_wave_add(s, i * 16, v);
    } else if (!v) {
        tx_wave_add(s, i * 16 + 8, true);
        tx_wave_add(s, i * 16 + 11, false);
    }
}

static void tx_begin(ESP32P4UartState *s, uint8_t phase, uint32_t end16,
                     int64_t t)
{
    s->tx_phase = phase;
    s->tx_n_edges = 0;
    s->tx_next_edge = 0;
    s->tx_end16 = end16;
    s->tx_start_ns = t;
    s->tx_q32 = tx_bit_q32(s);
    s->tx_pos16 = 0;
}

/* [spec:nuos:req:emu.esp32p4.uart] Start the frame of tx_byte at t. */
static void tx_begin_frame(ESP32P4UartState *s, int64_t t)
{
    Esp32UartFrameFmt fmt;
    uint16_t levels;
    unsigned nbits;

    uart_format(s, &fmt);
    nbits = esp32_uart_frame_levels(&fmt, s->tx_byte, &levels);
    tx_begin(s, ESP32P4_UART_TX_FRAME, nbits * 16 + fmt.stop_half_bits * 8,
             t);
    for (unsigned i = 0; i < nbits; i++) {
        tx_wave_bit(s, i, (levels >> i) & 1);
    }
    if (!irda(s)) {
        tx_wave_add(s, nbits * 16, true);
    }
    s->tx_frame_bits = nbits;
    s->tx_data_bits = fmt.data_bits;
    s->tx_parity = fmt.parity_en;
    s->echo_start_ns = t;
    s->echo_end_ns = tx_time16(s, s->tx_end16);
    s->echo_byte = s->tx_byte;
}

/*
 * TX_BRK_NUM bit times of 0 (TRM 45.4.4). In IrDA mode a 0 is a pulse a
 * bit, which the model does not send for a break: the line keeps idle.
 */
static void tx_begin_break(ESP32P4UartState *s, int64_t t)
{
    unsigned n = s->core[R(A_TXBRK_CONF)] & 0xff;

    tx_begin(s, ESP32P4_UART_TX_BREAK, n * 16, t);
    if (irda(s)) {
        qemu_log_mask(LOG_UNIMP, "%s: a break (TXD_BRK) in IrDA mode is "
                      "not modelled\n", object_get_canonical_path(OBJECT(s)));
        return;
    }
    tx_wave_add(s, 0, false);
    tx_wave_add(s, n * 16, true);
}

/* The transfer is over: the line idles TX_IDLE_NUM bit times. */
static void tx_begin_gap(ESP32P4UartState *s, int64_t t)
{
    unsigned n = IDLE_TX_NUM(s->core[R(A_IDLE_CONF)]);

    if (n) {
        tx_begin(s, ESP32P4_UART_TX_GAP, n * 16, t);
        return;
    }
    s->tx_phase = ESP32P4_UART_TX_IDLE;
    s->tx_free_ns = t;
    s->tx_in_transfer = false;
    s->int_raw |= INT_TX_BRK_IDLE;
}

/* Whether flow control lets the transmitter send data from its FIFO. */
static bool tx_data_allowed(ESP32P4UartState *s)
{
    uint32_t c0 = cconf0(s), sw = s->core[R(A_SWFC_CONF0)];

    if ((c0 & CONF0_TX_FLOW_EN) && s->cts_line) {
        return false;
    }
    if (sw & SWFC_EN) {
        if (sw & SWFC_FORCE_XOFF) {
            return false;
        }
        if (!(sw & SWFC_FORCE_XON) && s->sw_xoff_rcvd) {
            return false;
        }
    }
    return true;
}

/* Whether the line is the transmitter's to take. */
static bool tx_line_free(ESP32P4UartState *s)
{
    uint32_t r = s->core[R(A_RS485_CONF)];

    if (rs485(s) && !(r & RS485_RXBY_TX_EN) && s->rx.busy) {
        return false;
    }
    if (irda(s) && !(cconf0(s) & CONF0_IRDA_TX_EN)) {
        return false;
    }
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Start the next frame at t if the transmitter is idle: an XOFF or XON
 * that flow control asked for (SEND_XOFF, SEND_XON) goes first, sent even
 * after an XOFF received with XON_XOFF_STILL_SEND; then the TX FIFO's next
 * byte, if flow control allows. With DL0_EN a bit of turnaround delay
 * comes first (TRM 45.4.6.2; the LP UART, which has no RS485 mode, has
 * DL0_EN and DL1_EN too). Returns whether a frame started.
 */
static bool tx_start_next(ESP32P4UartState *s, int64_t t)
{
    uint32_t sw = s->core[R(A_SWFC_CONF0)];
    bool still = !(sw & SWFC_EN) || !s->sw_xoff_rcvd ||
                 (sw & SWFC_STILL_SEND);
    bool cts_ok = !(cconf0(s) & CONF0_TX_FLOW_EN) || !s->cts_line;
    bool popped = false;

    if (s->tx_phase != ESP32P4_UART_TX_IDLE || !tx_running(s) ||
        !tx_line_free(s)) {
        return false;
    }
    /* An event stamped before the line went idle starts nothing earlier. */
    t = MAX(t, s->tx_free_ns);
    if (s->send_xoff && still && cts_ok) {
        s->send_xoff = false;
        s->regs[R(A_SWFC_CONF0)] &= ~SWFC_SEND_XOFF;
        s->tx_byte = SWFC_XOFF_CHAR(sw);
    } else if (s->send_xon && still && cts_ok) {
        s->send_xon = false;
        s->regs[R(A_SWFC_CONF0)] &= ~SWFC_SEND_XON;
        s->tx_byte = SWFC_XON_CHAR(sw);
    } else if (s->tx_fifo.num && tx_data_allowed(s)) {
        s->tx_byte = fifo_pop(s, &s->tx_fifo);
        popped = true;
    } else {
        return false;
    }
    s->tx_in_transfer = true;
    if (s->core[R(A_RS485_CONF)] & RS485_DL0_EN) {
        tx_begin(s, ESP32P4_UART_TX_DL0, 16, t);
    } else {
        tx_begin_frame(s, t);
    }
    if (rs485(s)) {
        uart_update_lines(s, t);
    }
    /* A UHCI feeding this UART may refill the slot just freed. */
    if (popped && s->dma_ops) {
        s->dma_ops->tx_room(s->dma_opaque);
    }
    return true;
}

static void uart_tx_schedule(ESP32P4UartState *s)
{
    if (s->tx_phase == ESP32P4_UART_TX_IDLE || !s->tx_q32) {
        timer_del(&s->tx_timer);
        return;
    }
    timer_mod_ns(&s->tx_timer, s->tx_next_edge < s->tx_n_edges ?
                 tx_time16(s, s->tx_t16[s->tx_next_edge]) :
                 tx_time16(s, s->tx_end16));
}

static void uart_tx_try_start(ESP32P4UartState *s, int64_t t)
{
    if (tx_start_next(s, t)) {
        uart_tx_schedule(s);
        uart_update_irq(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * A frame (and its DL1 delay) has ended at t. The next byte follows at
 * once; with nothing left to send the transfer is done (TX_DONE), and
 * the transmitter sends TX_BRK_NUM bits of break if TXD_BRK is set
 * (TX_BRK_DONE), then idles TX_IDLE_NUM bit times (TX_BRK_IDLE_DONE).
 */
static void tx_frame_done(ESP32P4UartState *s, int64_t t)
{
    s->tx_phase = ESP32P4_UART_TX_IDLE;
    s->tx_free_ns = t;
    if (tx_start_next(s, t)) {
        return;
    }
    if (s->tx_fifo.num || s->send_xon || s->send_xoff) {
        /* Held by flow control: the transfer goes on once it is let go. */
        if (rs485(s)) {
            uart_update_lines(s, t);
        }
        return;
    }
    s->int_raw |= INT_TX_DONE;
    if (cconf0(s) & CONF0_TXD_BRK) {
        tx_begin_break(s, t);
    } else {
        tx_begin_gap(s, t);
    }
    if (rs485(s)) {
        uart_update_lines(s, t);
    }
}

static void tx_phase_end(ESP32P4UartState *s, int64_t t)
{
    switch (s->tx_phase) {
    case ESP32P4_UART_TX_DL0:
        tx_begin_frame(s, t);
        break;
    case ESP32P4_UART_TX_FRAME:
        if (s->core[R(A_RS485_CONF)] & RS485_DL1_EN) {
            tx_begin(s, ESP32P4_UART_TX_DL1, 16, t);
        } else {
            tx_frame_done(s, t);
        }
        break;
    case ESP32P4_UART_TX_DL1:
        tx_frame_done(s, t);
        break;
    case ESP32P4_UART_TX_BREAK:
        s->int_raw |= INT_TX_BRK_DONE;
        tx_begin_gap(s, t);
        if (rs485(s)) {
            uart_update_lines(s, t);
        }
        break;
    case ESP32P4_UART_TX_GAP:
        s->tx_phase = ESP32P4_UART_TX_IDLE;
        s->tx_free_ns = t;
        s->tx_in_transfer = false;
        s->int_raw |= INT_TX_BRK_IDLE;
        tx_start_next(s, t);
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Bring the transmitter up to the present: put every edge whose time has
 * passed on TXD, stamped with its own time, and move from phase to phase
 * at the times they end. Register accesses and pad samples call this
 * first, so what software sees does not depend on how promptly the timer
 * callback runs.
 */
static void uart_tx_advance(ESP32P4UartState *s)
{
    int64_t now = now_ns();

    while (s->tx_phase != ESP32P4_UART_TX_IDLE && s->tx_q32) {
        int64_t t;

        if (s->tx_next_edge < s->tx_n_edges) {
            t = tx_time16(s, s->tx_t16[s->tx_next_edge]);
            if (t > now) {
                break;
            }
            uart_set_tx_line(s, s->tx_lvl[s->tx_next_edge], t);
            s->tx_next_edge++;
            continue;
        }
        t = tx_time16(s, s->tx_end16);
        if (t > now) {
            break;
        }
        tx_phase_end(s, t);
    }
    uart_tx_schedule(s);
    uart_update_irq(s);
}

/* How far the current phase has got, in 2^-16 bits. */
static uint64_t tx_pos16_at(ESP32P4UartState *s, int64_t t)
{
    uint64_t lo, hi;

    if (!s->tx_q32) {
        return s->tx_pos16;
    }
    if (t <= s->tx_start_ns) {
        return 0;
    }
    hi = (uint64_t)(t - s->tx_start_ns);
    lo = hi << 48;
    hi >>= 16;
    divu128(&lo, &hi, s->tx_q32);
    return hi ? UINT64_MAX : lo;
}

/* Note how far the phase has got, before the bit time changes. */
static void uart_tx_freeze(ESP32P4UartState *s, int64_t t)
{
    if (s->tx_phase != ESP32P4_UART_TX_IDLE && s->tx_q32) {
        s->tx_pos16 = tx_pos16_at(s, t);
    }
}

/* Continue the phase in flight at the current bit time. */
static void uart_tx_rerate(ESP32P4UartState *s, int64_t t)
{
    uint64_t q32 = tx_bit_q32(s);

    if (s->tx_phase != ESP32P4_UART_TX_IDLE && q32 && !s->tx_q32) {
        s->tx_start_ns = t - esp32_uart_mul_shr(s->tx_pos16, q32, 48);
    } else if (s->tx_phase != ESP32P4_UART_TX_IDLE && q32) {
        s->tx_start_ns = t - esp32_uart_mul_shr(tx_pos16_at(s, t), q32, 48);
    }
    s->tx_q32 = q32;
    uart_tx_schedule(s);
}

/* Return the transmitter to idle, the line idle, driving it if `drive`. */
static void uart_tx_reset(ESP32P4UartState *s, int64_t t, bool drive)
{
    timer_del(&s->tx_timer);
    s->tx_phase = ESP32P4_UART_TX_IDLE;
    s->tx_free_ns = t;
    s->tx_in_transfer = false;
    s->tx_n_edges = 0;
    s->tx_next_edge = 0;
    s->tx_q32 = 0;
    s->tx_pos16 = 0;
    s->send_xon = false;
    s->send_xoff = false;
    s->sw_xoff_sent = false;
    s->sw_xoff_rcvd = false;
    s->echo_start_ns = 0;
    s->echo_end_ns = 0;
    if (!drive) {
        s->tx_line = tx_idle_level(s);
    } else if (s->tx_line != tx_idle_level(s)) {
        uart_set_tx_line(s, tx_idle_level(s), t);
    }
}

static void uart_tx_timer_cb(void *opaque)
{
    uart_tx_advance(ESP32P4_UART(opaque));
}

/* ---- Receiver ---- */

/* [spec:nuos:req:emu.esp32p4.uart] Autobaud detection (TRM 45.4.3.2). */
static void uart_autobaud_reset(ESP32P4UartState *s)
{
    s->rxd_edge_cnt = 0;
    s->lowpulse = AB_MAX;
    s->highpulse = AB_MAX;
    s->pospulse = AB_MAX;
    s->negpulse = AB_MAX;
    s->ab_last_edge_ns = -1;
    s->ab_last_rise_ns = -1;
    s->ab_last_fall_ns = -1;
}

static uint32_t ab_count(ESP32P4UartState *s, int64_t ns)
{
    uint64_t c = sclk_cycles(s, ns);

    return c ? MIN(c - 1, AB_MAX) : 0;
}

/*
 * Each edge of RXD past the glitch filter counts in RXD_EDGE_CNT; the
 * pulse it ends, low or high, and its distance from the last edge of the
 * same direction, in UART core clock cycles less one (so that
 * (LOWPULSE + HIGHPULSE + 2) / 2 is the bit time, TRM 45.4.3.2), keep
 * their minima.
 */
static void uart_autobaud_edge(ESP32P4UartState *s, bool level, int64_t t)
{
    if (!s->ab_active || !rx_running(s)) {
        return;
    }
    s->rxd_edge_cnt = MIN(s->rxd_edge_cnt + 1, EDGE_CNT_MAX);
    if (s->ab_last_edge_ns >= 0) {
        uint32_t w = ab_count(s, t - s->ab_last_edge_ns);

        if (level) {
            s->lowpulse = MIN(s->lowpulse, w);
        } else {
            s->highpulse = MIN(s->highpulse, w);
        }
    }
    if (level) {
        if (s->ab_last_rise_ns >= 0) {
            s->pospulse = MIN(s->pospulse,
                              ab_count(s, t - s->ab_last_rise_ns));
        }
        s->ab_last_rise_ns = t;
    } else {
        if (s->ab_last_fall_ns >= 0) {
            s->negpulse = MIN(s->negpulse,
                              ab_count(s, t - s->ab_last_fall_ns));
        }
        s->ab_last_fall_ns = t;
    }
    s->ab_last_edge_ns = t;
}

/* The receiver's input past the glitch filter. */
static void uart_rx_filtered(ESP32P4UartState *s, bool level, int64_t t)
{
    if (level == s->filt_out) {
        return;
    }
    s->filt_out = level;
    uart_autobaud_edge(s, level, t);
    if (level && s->light_sleep) {
        s->wake_edges++;
        uart_wake_check(s);
    }
    esp32_uart_rx_line(&s->rx, level, t);
}

static int64_t filt_width_ns(ESP32P4UartState *s)
{
    uint32_t f = s->regs[R(A_RX_FILT)];
    uint64_t hz = clock_get_hz(s->sclk);

    if (!(f & RX_FILT_EN) || !RX_FILT_WIDTH(f) || !hz) {
        return 0;
    }
    return muldiv64(RX_FILT_WIDTH(f), NANOSECONDS_PER_SECOND, hz);
}

static void uart_filt_commit(ESP32P4UartState *s)
{
    if (s->filt_pending) {
        int64_t t = s->filt_ns + filt_width_ns(s);

        s->filt_pending = false;
        timer_del(&s->filt_timer);
        uart_rx_filtered(s, s->filt_level, t);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The RX glitch filter (RX_FILT): with GLITCH_FILT_EN a level reaches the
 * receiver once it has held for GLITCH_FILT UART core clock cycles, so a
 * shorter pulse never does.
 */
static void uart_rx_chain(ESP32P4UartState *s, bool level, int64_t t)
{
    int64_t w = filt_width_ns(s);

    if (s->filt_pending && t >= s->filt_ns + w) {
        uart_filt_commit(s);
    }
    if (!w) {
        uart_rx_filtered(s, level, t);
        return;
    }
    if (s->filt_pending) {
        if (level != s->filt_level) {
            /* The pending change did not last: a glitch. */
            s->filt_pending = false;
            timer_del(&s->filt_timer);
        }
        return;
    }
    if (level == s->filt_out) {
        return;
    }
    s->filt_pending = true;
    s->filt_level = level;
    s->filt_ns = t;
    timer_mod_ns(&s->filt_timer, t + w);
}

static void uart_filt_timer_cb(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (s->filt_pending && now_ns() >= s->filt_ns + filt_width_ns(s)) {
        uart_filt_commit(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The IrDA decoder (TRM 45.4.7): a pulse marks a 0 bit, so each rising
 * edge of the decoder's input holds the receiver's NRZ input low for a
 * bit time (and an eighth, so that pulses a bit apart join up).
 */
static void uart_irda_in(ESP32P4UartState *s, bool level, int64_t t)
{
    if (level == s->irda_pin) {
        return;
    }
    s->irda_pin = level;
    if (!level) {
        return;
    }
    if (!s->irda_low) {
        s->irda_low = true;
        uart_rx_chain(s, false, t);
    }
    s->irda_high_ns = t + bits_ns(s, 1) + bits_ns(s, 1) / 8;
    timer_mod_ns(&s->irda_timer, s->irda_high_ns);
}

static void uart_irda_release(ESP32P4UartState *s)
{
    if (s->irda_low && now_ns() >= s->irda_high_ns) {
        s->irda_low = false;
        timer_del(&s->irda_timer);
        uart_rx_chain(s, true, s->irda_high_ns);
    }
}

static void uart_irda_timer_cb(void *opaque)
{
    uart_irda_release(ESP32P4_UART(opaque));
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The level the receiver sees: RXD inverted by RXD_INV, or TXD looped
 * back inside the UART; in IrDA mode the decoded pulses of the IrDA
 * transmitter looped back with IRDA_DPLX, or of RXD inverted by
 * IRDA_RX_INV, and nothing while IRDA_TX_EN has the half-duplex
 * transceiver sending.
 */
static bool uart_irda_input(ESP32P4UartState *s)
{
    uint32_t c0 = cconf0(s);

    if (c0 & CONF0_IRDA_DPLX) {
        return s->tx_line;
    }
    if (!(c0 & CONF0_IRDA_TX_EN)) {
        return s->rxd_pin ^ !!(c0 & CONF0_IRDA_RX_INV);
    }
    return false;
}

static void uart_rx_input(ESP32P4UartState *s, int64_t t)
{
    uint32_t c0 = cconf0(s);

    if (irda(s)) {
        uart_irda_in(s, uart_irda_input(s), t);
        if (!s->irda_low) {
            /* No pulse: the decoded line idles high. */
            uart_rx_chain(s, true, t);
        }
        return;
    }
    if (s->irda_low) {
        s->irda_low = false;
        timer_del(&s->irda_timer);
    }
    s->rx_in = (c0 & CONF0_LOOPBACK) ? s->tx_line :
               s->rxd_pin ^ !!(c0 & CONF0_RXD_INV);
    uart_rx_chain(s, s->rx_in, t);
}

static bool uart_rx_format(void *opaque, Esp32UartFrameFmt *fmt)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    uint32_t r = s->core[R(A_RS485_CONF)];

    if (!rx_running(s)) {
        return false;
    }
    /* In RS485 mode the receiver is off while the transmitter sends. */
    if (rs485(s) && !(r & RS485_TX_RX_EN) && tx_driving(s)) {
        return false;
    }
    uart_format(s, fmt);
    return fmt->bit_q32 != 0;
}

/* Whether `timer` was due by `t`: the bus was idle long enough. */
static bool uart_timer_due(QEMUTimer *timer, int64_t t)
{
    int64_t expire = timer_expire_time_ns(timer);

    timer_del(timer);
    return expire >= 0 && expire <= t;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * A start bit: the bus is busy, so the RX timeout, the UHCI idle timer
 * and an AT_CMD sequence's post-idle wait end, each taking effect first if
 * its time had come by the start bit's; it wakes the chip in wakeup mode
 * 2.
 */
static void uart_rx_start(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    int64_t t = s->rx.edge_ns[0];

    if (uart_timer_due(&s->tout_timer, t)) {
        uart_tout_fire(s);
    }
    if (uart_timer_due(&s->idle_timer, t)) {
        uart_idle_fire(s);
    }
    if (uart_timer_due(&s->at_timer, t)) {
        uart_at_fire(s);
    }
    if (SLEEP2_WK_MODE(s->regs[R(A_SLEEP_CONF2)]) == 2) {
        uart_wake(s);
    }
}

static void uart_rx_glitch(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    uart_raise(s, INT_GLITCH_DET);
    uart_tx_try_start(s, s->rx.frame_start_ns + bits_ns(s, 1) / 2);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * AT_CMD detection (TRM 45.4.5): AT_CMD_CHAR at least CHAR_NUM times,
 * each within RX_GAP_TOUT bit times of the one before, the first after
 * PRE_IDLE_NUM bit times of quiet, raise AT_CMD_CHAR_DET once
 * POST_IDLE_NUM bit times pass without another byte.
 */
static void uart_at_byte(ESP32P4UartState *s, uint8_t b, int64_t start,
                         int64_t end)
{
    uint32_t chr = s->core[R(A_AT_CMD_CHAR)];
    int64_t gap = start - s->at_last_end_ns;
    bool quiet = !s->at_seen_byte ||
                 gap >= bits_ns(s, s->core[R(A_AT_CMD_PRECNT)] & 0xffff);

    if (b != AT_CHAR(chr)) {
        s->at_count = 0;
    } else if (s->at_count &&
               gap < bits_ns(s, s->core[R(A_AT_CMD_GAPTOUT)] & 0xffff)) {
        s->at_count++;
    } else {
        s->at_count = quiet ? 1 : 0;
    }
    s->at_seen_byte = true;
    s->at_last_end_ns = end;
    if (s->at_count && s->at_count >= AT_CHAR_NUM(chr)) {
        timer_mod_ns(&s->at_timer, end + bits_ns(s,
                     s->core[R(A_AT_CMD_POSTCNT)] & 0xffff));
    }
}

static void uart_at_fire(ESP32P4UartState *s)
{
    if (s->at_count) {
        s->at_count = 0;
        uart_raise(s, INT_AT_CMD_DET);
    }
}

static void uart_at_timer_cb(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (!s->rx.busy) {
        uart_at_fire(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * A frame has arrived. Parity and framing errors and breaks raise their
 * interrupts, or in RS485 mode, for the echo of a frame the transmitter
 * sent, the RS485 ones, with RS485_CLASH if the echo differs; with
 * ERR_WR_MASK a frame in error is not stored. With SW_FLOW_CON_EN, XON and
 * XOFF raise SW_XON and SW_XOFF, let the transmitter go or stop it, and
 * with XONOFF_DEL are not stored. A full RX FIFO drops the byte and
 * raises RXFIFO_OVF, unless DIS_RX_DAT_OVF.
 */
static void uart_rx_deliver(void *opaque, unsigned data, unsigned flags)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    uint32_t c0 = cconf0(s), sw = s->core[R(A_SWFC_CONF0)];
    int64_t start = s->rx.frame_start_ns;
    int64_t done = s->rx.frame_end_ns;
    int64_t end = done + bits_ns(s, 1) / 2;
    bool bad = flags & (ESP32_UART_RX_PARITY_ERR | ESP32_UART_RX_FRAME_ERR);
    bool store = true;
    uint32_t idle;

    if (rs485(s) && start >= s->echo_start_ns && start < s->echo_end_ns) {
        if (flags & ESP32_UART_RX_PARITY_ERR) {
            s->int_raw |= INT_RS485_PARITY;
        }
        if (flags & ESP32_UART_RX_FRAME_ERR) {
            s->int_raw |= INT_RS485_FRM;
        }
        if (bad || data != s->echo_byte) {
            s->int_raw |= INT_RS485_CLASH;
        }
    } else {
        if (flags & ESP32_UART_RX_PARITY_ERR) {
            s->int_raw |= INT_PARITY_ERR;
        }
        if (flags & ESP32_UART_RX_FRAME_ERR) {
            s->int_raw |= INT_FRM_ERR;
        }
    }
    if (flags & ESP32_UART_RX_BREAK) {
        s->int_raw |= INT_BRK_DET;
    }
    if (bad && (c0 & CONF0_ERR_WR_MASK)) {
        store = false;
    }
    if ((sw & SWFC_EN) && !bad) {
        if (data == SWFC_XOFF_CHAR(sw)) {
            s->int_raw |= INT_SW_XOFF;
            s->sw_xoff_rcvd = true;
            store = store && !(sw & SWFC_XONOFF_DEL);
        } else if (data == SWFC_XON_CHAR(sw)) {
            s->int_raw |= INT_SW_XON;
            s->sw_xoff_rcvd = false;
            store = store && !(sw & SWFC_XONOFF_DEL);
        }
    }
    if (store && !(c0 & CONF0_RXFIFO_RST)) {
        if (!fifo_push(s, &s->rx_fifo, data)) {
            if (!(c0 & CONF0_DIS_RX_OVF)) {
                s->int_raw |= INT_RXFIFO_OVF;
            }
        }
    }
    uart_wake_char(s, data);
    uart_at_byte(s, data, start, end);

    s->rx_last_end_ns = end;
    s->rx_had_byte = true;
    if (s->core[R(A_TOUT_CONF)] & TOUT_EN) {
        timer_mod_ns(&s->tout_timer, end + bits_ns(s,
                     TOUT_THRHD(s->core[R(A_TOUT_CONF)])));
    }
    idle = IDLE_RX_THRHD(s->core[R(A_IDLE_CONF)]);
    timer_mod_ns(&s->idle_timer, end + bits_ns(s, idle));

    uart_rx_fifo_changed(s, done);
    if (s->dma_ops) {
        s->dma_ops->rx_data(s->dma_opaque);
        if (flags & ESP32_UART_RX_BREAK) {
            s->dma_ops->rx_break(s->dma_opaque);
        }
    }
    /* A transmitter waiting for the line or for XON may go on. */
    uart_tx_try_start(s, done);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * RXFIFO_TOUT: the bus idle for RX_TOUT_THRHD bit times after a byte, with
 * bytes in the RX FIFO. With RX_TOUT_FLOW_DIS the idle count does not run
 * while RTS holds the sender off.
 */
static void uart_tout_fire(ESP32P4UartState *s)
{
    if (!(s->core[R(A_TOUT_CONF)] & TOUT_EN)) {
        return;
    }
    if ((s->core[R(A_TOUT_CONF)] & TOUT_FLOW_DIS) && rts_blocking(s)) {
        return;
    }
    if (s->rx_fifo.num) {
        uart_raise(s, INT_RXFIFO_TOUT);
    }
}

static void uart_tout_timer_cb(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (!s->rx.busy) {
        uart_tout_fire(s);
    }
}

static void uart_idle_fire(ESP32P4UartState *s)
{
    if (s->dma_ops) {
        s->dma_ops->rx_idle(s->dma_opaque);
    }
}

static void uart_idle_timer_cb(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (!s->rx.busy) {
        uart_idle_fire(s);
    }
}

static void uart_rx_reset(ESP32P4UartState *s)
{
    esp32_uart_rx_reset(&s->rx, s->filt_out);
    timer_del(&s->tout_timer);
    timer_del(&s->idle_timer);
    timer_del(&s->at_timer);
    s->at_count = 0;
    s->at_seen_byte = false;
    s->rx_had_byte = false;
}

/* ---- The register update and the core's resets ---- */

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Carry the _SYNC registers into the core (REG_UPDATE). The frame format
 * applies from the next frame and the bit time from the next instant;
 * TXFIFO_RST and RXFIFO_RST hold their FIFO empty while set; a rising
 * AUTOBAUD_EN starts autobaud detection afresh; FORCE_XON lets a
 * transmitter stopped by XOFF go on; SEND_XON and SEND_XOFF queue their
 * character.
 */
static void uart_sync(ESP32P4UartState *s)
{
    int64_t t = now_ns();
    uint32_t old_c0 = cconf0(s);
    uint32_t c0, sw;

    uart_tx_advance(s);
    uart_tx_freeze(s, t);
    for (unsigned r = 0; r < ESP32P4_UART_NREGS; r++) {
        if (uart_regs[r].sync) {
            s->core[r] = s->regs[r];
        }
    }
    c0 = cconf0(s);
    sw = s->core[R(A_SWFC_CONF0)];
    if (!s->lp && ((c0 ^ old_c0) & CONF0_IRDA_EN)) {
        /*
         * Into or out of IrDA mode the receiver starts afresh from the
         * line as it now decodes: idle high, with no pulse made of the
         * level the pin is at.
         */
        s->irda_low = false;
        timer_del(&s->irda_timer);
        s->irda_pin = irda(s) && uart_irda_input(s);
        s->filt_pending = false;
        timer_del(&s->filt_timer);
        s->filt_out = irda(s) || ((c0 & CONF0_LOOPBACK) ? s->tx_line :
                                  s->rxd_pin ^ !!(c0 & CONF0_RXD_INV));
        uart_rx_reset(s);
    }
    if (c0 & CONF0_TXFIFO_RST) {
        fifo_reset(&s->tx_fifo);
    }
    if (c0 & CONF0_RXFIFO_RST) {
        fifo_reset(&s->rx_fifo);
    }
    if (!s->lp && (c0 & CONF0_AUTOBAUD_EN) && !(old_c0 & CONF0_AUTOBAUD_EN)) {
        uart_autobaud_reset(s);
    }
    s->ab_active = !s->lp && (c0 & CONF0_AUTOBAUD_EN);
    if ((sw & SWFC_EN) && (sw & SWFC_FORCE_XON)) {
        s->sw_xoff_rcvd = false;
    }
    if (sw & SWFC_SEND_XON) {
        s->send_xon = true;
    }
    if (sw & SWFC_SEND_XOFF) {
        s->send_xoff = true;
    }
    if (!s->lp && (s->core[R(A_RS485_CONF)] & (RS485_RX_DLY | (0xf << 6)))) {
        qemu_log_mask(LOG_UNIMP, "%s: RS485_RX_DLY_NUM and RS485_TX_DLY_NUM "
                      "(internal data signal delays) are not modelled\n",
                      object_get_canonical_path(OBJECT(s)));
    }
    if (irda(s) && (c0 & CONF0_IRDA_WCTL)) {
        qemu_log_mask(LOG_UNIMP, "%s: IRDA_WCTL (the IrDA transmitter's "
                      "11th bit) is not modelled\n",
                      object_get_canonical_path(OBJECT(s)));
    }
    uart_tx_rerate(s, t);
    if (s->tx_phase == ESP32P4_UART_TX_IDLE &&
        s->tx_line != tx_idle_level(s)) {
        uart_set_tx_line(s, tx_idle_level(s), t);
    } else {
        /* TXD_INV or IRDA_TX_INV may have changed. */
        esp32_line_set_at(s->txd_out, txd_pin_level(s, s->tx_line), t);
    }
    uart_update_lines(s, t);
    uart_rx_input(s, t);
    uart_rx_fifo_changed(s, t);
    uart_tx_try_start(s, t);
    uart_format_changed(s);
    if (s->dma_ops) {
        s->dma_ops->tx_room(s->dma_opaque);
    }
}

static void uart_update_schedule(ESP32P4UartState *s)
{
    uint64_t hz = clock_get_hz(s->sclk);

    if (!s->update_pending || !core_running(s) || !hz) {
        timer_del(&s->update_timer);
        return;
    }
    if (!timer_pending(&s->update_timer)) {
        timer_mod_ns(&s->update_timer, now_ns() +
                     muldiv64(UPDATE_SYNC_CYCLES, NANOSECONDS_PER_SECOND, hz)
                     + 1);
    }
}

static void uart_update_timer_cb(void *opaque)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (!s->update_pending || !core_running(s)) {
        return;
    }
    s->update_pending = false;
    uart_sync(s);
}

/*
 * The core's _SYNC copies and state as a core reset leaves them; the TX
 * line is driven idle if `drive`.
 */
static void uart_core_reset_state(ESP32P4UartState *s, int64_t t, bool drive)
{
    for (unsigned r = 0; r < ESP32P4_UART_NREGS; r++) {
        if (uart_regs[r].sync) {
            s->core[r] = reg_reset(s, r);
        }
    }
    fifo_reset(&s->tx_fifo);
    fifo_reset(&s->rx_fifo);
    uart_tx_reset(s, t, drive);
    uart_rx_reset(s);
    uart_autobaud_reset(s);
    s->ab_active = false;
    s->wake_edges = 0;
    s->wake_nchars = 0;
}

/* [spec:nuos:req:emu.esp32p4.uart] HP_SYS_CLKRST_RST_EN_UARTn_CORE. */
static void uart_core_reset_in(void *opaque, int n, int level)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    int64_t t = now_ns();

    if (s->core_reset == !!level) {
        return;
    }
    uart_tx_advance(s);
    s->core_reset = level;
    if (level) {
        uart_core_reset_state(s, t, true);
    }
    uart_tx_rerate(s, t);
    uart_update_lines(s, t);
    uart_update_schedule(s);
    uart_rx_fifo_changed(s, t);
    uart_format_changed(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * CLK_CONF: TX_SCLK_EN and RX_SCLK_EN gate the transmitter's and the
 * receiver's clocks; TX_RST_CORE and RX_RST_CORE hold the transmitter
 * with its FIFO, and the receiver with its FIFO, in reset while set.
 */
static void uart_write_clk_conf(ESP32P4UartState *s, uint32_t v)
{
    int64_t t = now_ns();
    uint32_t old = s->regs[R(A_CLK_CONF)];

    uart_tx_advance(s);
    uart_tx_freeze(s, t);
    s->regs[R(A_CLK_CONF)] = v;
    if (v & CLK_TX_RST_CORE) {
        fifo_reset(&s->tx_fifo);
        uart_tx_reset(s, t, true);
    }
    if (v & CLK_RX_RST_CORE) {
        fifo_reset(&s->rx_fifo);
        uart_rx_reset(s);
    } else if (!(v & CLK_RX_SCLK_EN) && (old & CLK_RX_SCLK_EN)) {
        uart_rx_reset(s);
    }
    uart_tx_rerate(s, t);
    uart_rx_fifo_changed(s, t);
    uart_tx_try_start(s, t);
    uart_format_changed(s);
}

/* ---- Registers ---- */

static uint32_t uart_status(ESP32P4UartState *s)
{
    uint32_t v = put8(s, s->rx_fifo.num, 0) | put8(s, s->tx_fifo.num, 16);

    v |= s->dsr_line ? BIT(13) : 0;
    v |= s->cts_line ? BIT(14) : 0;
    v |= s->filt_out ? BIT(15) : 0;
    v |= s->dtr_line ? BIT(29) : 0;
    v |= s->rts_line ? BIT(30) : 0;
    v |= s->tx_line ? BIT(31) : 0;
    return v;
}

/* The FIFO RAM offsets: TX from 0, RX from fifo_len (TRM 45.4.2). */
static uint32_t uart_mem_status(ESP32P4UartState *s, bool rx)
{
    unsigned sh = s->lp ? 3 : 0;

    if (rx) {
        return ((s->fifo_len + s->rx_fifo.rd) << sh) |
               ((s->fifo_len + s->rx_fifo.wr) << (9 + sh));
    }
    return (s->tx_fifo.wr << sh) | (s->tx_fifo.rd << (9 + sh));
}

/* The FSM state for bit `b` of a frame of `data` bits, parity `par`. */
static unsigned fsm_bit_state(unsigned b, unsigned data, bool par)
{
    if (b == 0) {
        return FSM_START;
    }
    if (b <= data) {
        return FSM_DATA0 + b - 1;
    }
    if (par && b == data + 1) {
        return FSM_PARITY;
    }
    return b == data + 1 + par ? FSM_STOP1 : FSM_STOP2;
}

/* [spec:nuos:req:emu.esp32p4.uart] FSM_STATUS (see the head comment). */
static uint32_t uart_fsm_status(ESP32P4UartState *s)
{
    unsigned tx = FSM_IDLE, rx = FSM_IDLE;
    int64_t t = now_ns();

    switch (s->tx_phase) {
    case ESP32P4_UART_TX_DL0:
        tx = FSM_DL0;
        break;
    case ESP32P4_UART_TX_DL1:
        tx = FSM_DL1;
        break;
    case ESP32P4_UART_TX_FRAME:
        tx = fsm_bit_state(tx_pos16_at(s, t) >> 16, s->tx_data_bits,
                           s->tx_parity);
        break;
    case ESP32P4_UART_TX_BREAK:
        tx = FSM_START;
        break;
    default:
        if (s->tx_fifo.num && !tx_data_allowed(s) &&
            (s->core[R(A_SWFC_CONF0)] & SWFC_EN)) {
            tx = FSM_TX_WAIT_SEND;
        }
        break;
    }
    if (s->rx.busy && s->rx.fmt.bit_q32) {
        uint64_t lo, hi;

        hi = (uint64_t)MAX(t - s->rx.edge_ns[0], 0);
        lo = hi << 32;
        hi >>= 32;
        divu128(&lo, &hi, s->rx.fmt.bit_q32);
        rx = fsm_bit_state(MIN(lo, 15), s->rx.fmt.data_bits,
                           s->rx.fmt.parity_en);
    }
    return rx | (tx << 4);
}

static bool uart_reg_exists(ESP32P4UartState *s, hwaddr addr)
{
    if (addr >= 0xa0 || addr == 0x28 || addr == 0x94) {
        return false;
    }
    if (s->lp && addr >= A_POSPULSE && addr <= A_RXD_CNT) {
        return false;
    }
    return true;
}

static uint64_t uart_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    uint32_t v = 0;

    uart_tx_advance(s);
    uart_filt_timer_cb(s);
    esp32_uart_rx_flush(&s->rx);
    if (!uart_reg_exists(s, addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", object_get_canonical_path(OBJECT(s)),
                      addr);
        return 0;
    }
    switch (addr) {
    case A_FIFO:
        if (!fifo_port_running(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: FIFO read while the FIFO "
                          "port's clock (UART_SYS_CLK) is stopped\n",
                          object_get_canonical_path(OBJECT(s)));
        } else if (!s->rx_fifo.num) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read of the empty RX FIFO\n",
                          object_get_canonical_path(OBJECT(s)));
        } else {
            v = fifo_pop(s, &s->rx_fifo);
            uart_rx_fifo_changed(s, now_ns());
        }
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->regs[R(A_INT_ENA)];
        break;
    case A_INT_CLR:
        v = 0;
        break;
    case A_STATUS:
        v = uart_status(s);
        break;
    case A_SWFC_CONF0:
        v = s->regs[R(addr)] & ~(SWFC_SEND_XON | SWFC_SEND_XOFF);
        v |= (s->send_xon || (s->regs[R(addr)] & SWFC_SEND_XON)) ?
             SWFC_SEND_XON : 0;
        v |= (s->send_xoff || (s->regs[R(addr)] & SWFC_SEND_XOFF)) ?
             SWFC_SEND_XOFF : 0;
        break;
    case A_MEM_TX_STATUS:
        v = uart_mem_status(s, false);
        break;
    case A_MEM_RX_STATUS:
        v = uart_mem_status(s, true);
        break;
    case A_FSM_STATUS:
        v = uart_fsm_status(s);
        break;
    case A_POSPULSE:
        v = s->pospulse;
        break;
    case A_NEGPULSE:
        v = s->negpulse;
        break;
    case A_LOWPULSE:
        v = s->lowpulse;
        break;
    case A_HIGHPULSE:
        v = s->highpulse;
        break;
    case A_RXD_CNT:
        v = s->rxd_edge_cnt;
        break;
    case A_AFIFO_STATUS:
        /* Bytes cross the APB-to-core FIFOs at once: always empty. */
        v = 0xa;
        break;
    case A_REG_UPDATE:
        v = s->update_pending;
        break;
    default:
        v = s->regs[R(addr)];
        break;
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Registers in the APB_CLK domain take effect as written; the _SYNC ones
 * wait for REG_UPDATE.
 */
static void uart_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    uint32_t v = value;
    int64_t t = now_ns();
    unsigned r = R(addr);

    uart_tx_advance(s);
    uart_filt_timer_cb(s);
    esp32_uart_rx_flush(&s->rx);
    if (!uart_reg_exists(s, addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%"
                      HWADDR_PRIx "\n", object_get_canonical_path(OBJECT(s)),
                      addr);
        return;
    }
    switch (addr) {
    case A_FIFO:
        if (!fifo_port_running(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: FIFO write while the FIFO "
                          "port's clock (UART_SYS_CLK) is stopped\n",
                          object_get_canonical_path(OBJECT(s)));
        } else if (cconf0(s) & CONF0_TXFIFO_RST) {
            /* The FIFO is held in reset. */
        } else if (!fifo_push(s, &s->tx_fifo, v & 0xff)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to the full TX FIFO\n",
                          object_get_canonical_path(OBJECT(s)));
        } else {
            uart_tx_try_start(s, t);
        }
        break;
    case A_INT_RAW:
        s->int_raw &= ~v;
        break;
    case A_INT_CLR:
        s->int_raw &= ~v;
        break;
    case A_REG_UPDATE:
        if (v & 1) {
            s->update_pending = true;
            uart_update_schedule(s);
        }
        break;
    case A_CLK_CONF:
        uart_write_clk_conf(s, v & (s->lp ? uart_regs[r].lp_rw
                                          : uart_regs[r].rw));
        break;
    case A_MEM_CONF:
        s->regs[r] = v & uart_regs[r].rw;
        if ((v & MEM_FORCE_PD) && !(v & MEM_FORCE_PU)) {
            qemu_log_mask(LOG_UNIMP, "%s: forcing the FIFO RAM off "
                          "(MEM_FORCE_PD) is not modelled\n",
                          object_get_canonical_path(OBJECT(s)));
        }
        break;
    case A_INT_ST:
    case A_STATUS:
    case A_MEM_TX_STATUS:
    case A_MEM_RX_STATUS:
    case A_FSM_STATUS:
    case A_POSPULSE:
    case A_NEGPULSE:
    case A_LOWPULSE:
    case A_HIGHPULSE:
    case A_RXD_CNT:
    case A_AFIFO_STATUS:
        break;
    default:
        s->regs[r] = v & (s->lp ? uart_regs[r].lp_rw : uart_regs[r].rw);
        if (addr == A_CONF1 || addr == A_SWFC_CONF1 ||
            addr == A_SLEEP_CONF2) {
            uart_update_lines(s, t);
            uart_sw_flow_check(s, t);
            uart_wake_check(s);
        }
        break;
    }
    uart_update_irq(s);
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ---- UHCI port ---- */

void esp32p4_uart_dma_attach(ESP32P4UartState *s,
                             const ESP32P4UartDmaOps *ops, void *opaque)
{
    s->dma_ops = ops;
    s->dma_opaque = opaque;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
unsigned esp32p4_uart_dma_tx_free(ESP32P4UartState *s)
{
    if (!fifo_port_running(s) || (cconf0(s) & CONF0_TXFIFO_RST)) {
        return 0;
    }
    return s->fifo_len - s->tx_fifo.num;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
bool esp32p4_uart_dma_tx_push(ESP32P4UartState *s, uint8_t byte)
{
    if (!esp32p4_uart_dma_tx_free(s)) {
        return false;
    }
    fifo_push(s, &s->tx_fifo, byte);
    uart_tx_try_start(s, now_ns());
    uart_update_irq(s);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
unsigned esp32p4_uart_dma_rx_count(ESP32P4UartState *s)
{
    return fifo_port_running(s) ? s->rx_fifo.num : 0;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
bool esp32p4_uart_dma_rx_pop(ESP32P4UartState *s, uint8_t *byte)
{
    if (!esp32p4_uart_dma_rx_count(s)) {
        return false;
    }
    *byte = fifo_pop(s, &s->rx_fifo);
    uart_rx_fifo_changed(s, now_ns());
    return true;
}

/* ---- Inputs, clocks and the line interface ---- */

static void uart_rxd_in(void *opaque, int n, int level)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    s->rxd_pin = level > 0;
    uart_rx_input(s, esp32_line_time());
}

static void uart_cts_in(void *opaque, int n, int level)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    s->cts_pin = level > 0;
    uart_update_inputs(s, esp32_line_time(), true);
    uart_update_irq(s);
}

static void uart_dsr_in(void *opaque, int n, int level)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    s->dsr_pin = level > 0;
    uart_update_inputs(s, esp32_line_time(), true);
    uart_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.uart] Light sleep from the PMU. */
static void uart_light_sleep_in(void *opaque, int n, int level)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (s->light_sleep == !!level) {
        return;
    }
    s->light_sleep = level;
    s->wake_edges = 0;
    s->wake_nchars = 0;
    if (!level && s->wakeup) {
        s->wakeup = false;
        qemu_set_irq(s->wakeup_out, 0);
    }
    uart_wake_check(s);
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static void uart_sclk_update(void *opaque, ClockEvent event)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);
    int64_t t = now_ns();

    if (event == ClockPreUpdate) {
        uart_tx_advance(s);
        uart_tx_freeze(s, t);
        return;
    }
    uart_tx_rerate(s, t);
    uart_update_schedule(s);
    uart_tx_try_start(s, t);
    uart_format_changed(s);
}

static void uart_sys_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4UartState *s = ESP32P4_UART(opaque);

    if (s->dma_ops && fifo_port_running(s)) {
        s->dma_ops->tx_room(s->dma_opaque);
        s->dma_ops->rx_data(s->dma_opaque);
    }
}

static void uart_line_sync(Notifier *n, void *data)
{
    ESP32P4UartState *s = container_of(n, ESP32P4UartState, line_source);

    uart_tx_advance(s);
    uart_filt_timer_cb(s);
    uart_irda_release(s);
}

/* [spec:nuos:req:emu.esp32p4.uart] The format a matching terminal uses. */
static bool uart_line_get_format(Object *obj, Esp32UartFrameFmt *fmt)
{
    uart_format(ESP32P4_UART(obj), fmt);
    return fmt->bit_q32 != 0;
}

static void uart_line_add_format_notifier(Object *obj, Notifier *n)
{
    notifier_list_add(&ESP32P4_UART(obj)->format_notifiers, n);
}

/* ---- Reset, realize and state ---- */

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * The APB reset returns every register, the core and both FIFOs to their
 * reset state. The pins' input levels belong to the outside world and are
 * kept; the outputs return to idle: TXD high, RTS and DTR inactive.
 */
static void uart_reset_hold(Object *obj, ResetType type)
{
    ESP32P4UartState *s = ESP32P4_UART(obj);

    for (unsigned r = 0; r < ESP32P4_UART_NREGS; r++) {
        s->regs[r] = reg_reset(s, r);
    }
    s->int_raw = 0;
    s->update_pending = false;
    timer_del(&s->update_timer);
    timer_del(&s->tx_timer);
    s->tx_phase = ESP32P4_UART_TX_IDLE;
    uart_core_reset_state(s, now_ns(), false);
    s->tx_line = true;
    s->rts_line = true;
    s->dtr_line = true;
    s->filt_pending = false;
    timer_del(&s->filt_timer);
    s->irda_low = false;
    s->irda_pin = false;
    timer_del(&s->irda_timer);
    s->rx_in = s->rxd_pin;
    s->filt_out = s->rxd_pin;
    esp32_uart_rx_reset(&s->rx, s->filt_out);
    s->cts_line = s->cts_pin;
    s->dsr_line = s->dsr_pin;
    s->wakeup = false;
    s->rx_last_end_ns = 0;
}

static void uart_reset_exit(Object *obj, ResetType type)
{
    ESP32P4UartState *s = ESP32P4_UART(obj);

    qemu_set_irq(s->wakeup_out, 0);
    qemu_set_irq(s->txd_out, txd_pin_level(s, s->tx_line));
    uart_update_lines(s, now_ns());
    s->int_raw &= ~(INT_CTS_CHG | INT_DSR_CHG);
    uart_update_irq(s);
    uart_format_changed(s);
}

static void uart_realize(DeviceState *dev, Error **errp)
{
    ESP32P4UartState *s = ESP32P4_UART(dev);

    s->fifo_len = s->lp ? ESP32P4_LP_UART_FIFO_LEN : ESP32P4_UART_FIFO_LEN;
    memory_region_init_io(&s->iomem, OBJECT(s), &uart_ops, s,
                          s->name ? s->name : TYPE_ESP32P4_UART,
                          ESP32P4_UART_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    esp32_line_add_source(&s->line_source);
}

static void uart_init(Object *obj)
{
    ESP32P4UartState *s = ESP32P4_UART(obj);
    DeviceState *dev = DEVICE(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->sclk = qdev_init_clock_in(dev, "sclk", uart_sclk_update, s,
                                 ClockPreUpdate | ClockUpdate);
    s->sys_clk = qdev_init_clock_in(dev, "sys", uart_sys_clk_update, s,
                                    ClockUpdate);
    qdev_init_gpio_out_named(dev, &s->txd_out, ESP32P4_UART_TXD, 1);
    qdev_init_gpio_out_named(dev, &s->rts_out, ESP32P4_UART_RTS, 1);
    qdev_init_gpio_out_named(dev, &s->dtr_out, ESP32P4_UART_DTR, 1);
    qdev_init_gpio_out_named(dev, &s->wakeup_out, ESP32P4_UART_WAKEUP, 1);
    qdev_init_gpio_in_named(dev, uart_rxd_in, ESP32P4_UART_RXD, 1);
    qdev_init_gpio_in_named(dev, uart_cts_in, ESP32P4_UART_CTS, 1);
    qdev_init_gpio_in_named(dev, uart_dsr_in, ESP32P4_UART_DSR, 1);
    qdev_init_gpio_in_named(dev, uart_core_reset_in, ESP32P4_UART_CORE_RESET,
                            1);
    qdev_init_gpio_in_named(dev, uart_light_sleep_in,
                            ESP32P4_UART_LIGHT_SLEEP, 1);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, uart_tx_timer_cb, s);
    timer_init_ns(&s->update_timer, QEMU_CLOCK_VIRTUAL, uart_update_timer_cb,
                  s);
    timer_init_ns(&s->filt_timer, QEMU_CLOCK_VIRTUAL, uart_filt_timer_cb, s);
    timer_init_ns(&s->tout_timer, QEMU_CLOCK_VIRTUAL, uart_tout_timer_cb, s);
    timer_init_ns(&s->idle_timer, QEMU_CLOCK_VIRTUAL, uart_idle_timer_cb, s);
    timer_init_ns(&s->irda_timer, QEMU_CLOCK_VIRTUAL, uart_irda_timer_cb, s);
    timer_init_ns(&s->at_timer, QEMU_CLOCK_VIRTUAL, uart_at_timer_cb, s);
    esp32_uart_rx_init(&s->rx, uart_rx_format, uart_rx_deliver, s);
    s->rx.start = uart_rx_start;
    s->rx.glitch = uart_rx_glitch;
    notifier_list_init(&s->format_notifiers);
    s->line_source.notify = uart_line_sync;
    /*
     * The inputs until the matrix drives them, at the matrix's defaults
     * for unrouted inputs (TRM table 10.12-1): RXD, CTS and DSR low.
     */
    s->rxd_pin = false;
    s->cts_pin = false;
    s->dsr_pin = false;
}

static const VMStateDescription vmstate_esp32p4_uart = {
    .name = TYPE_ESP32P4_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(sclk, ESP32P4UartState),
        VMSTATE_CLOCK(sys_clk, ESP32P4UartState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4UartState, ESP32P4_UART_NREGS),
        VMSTATE_UINT32_ARRAY(core, ESP32P4UartState, ESP32P4_UART_NREGS),
        VMSTATE_UINT32(int_raw, ESP32P4UartState),
        VMSTATE_TIMER(update_timer, ESP32P4UartState),
        VMSTATE_BOOL(update_pending, ESP32P4UartState),
        VMSTATE_UINT8_ARRAY(tx_fifo.data, ESP32P4UartState,
                            ESP32P4_UART_FIFO_LEN),
        VMSTATE_UINT8(tx_fifo.rd, ESP32P4UartState),
        VMSTATE_UINT8(tx_fifo.wr, ESP32P4UartState),
        VMSTATE_UINT8(tx_fifo.num, ESP32P4UartState),
        VMSTATE_UINT8_ARRAY(rx_fifo.data, ESP32P4UartState,
                            ESP32P4_UART_FIFO_LEN),
        VMSTATE_UINT8(rx_fifo.rd, ESP32P4UartState),
        VMSTATE_UINT8(rx_fifo.wr, ESP32P4UartState),
        VMSTATE_UINT8(rx_fifo.num, ESP32P4UartState),
        VMSTATE_BOOL(rxd_pin, ESP32P4UartState),
        VMSTATE_BOOL(cts_pin, ESP32P4UartState),
        VMSTATE_BOOL(dsr_pin, ESP32P4UartState),
        VMSTATE_BOOL(light_sleep, ESP32P4UartState),
        VMSTATE_BOOL(core_reset, ESP32P4UartState),
        VMSTATE_BOOL(tx_line, ESP32P4UartState),
        VMSTATE_BOOL(rts_line, ESP32P4UartState),
        VMSTATE_BOOL(dtr_line, ESP32P4UartState),
        VMSTATE_BOOL(cts_line, ESP32P4UartState),
        VMSTATE_BOOL(dsr_line, ESP32P4UartState),
        VMSTATE_BOOL(rx_in, ESP32P4UartState),
        VMSTATE_TIMER(tx_timer, ESP32P4UartState),
        VMSTATE_UINT8(tx_phase, ESP32P4UartState),
        VMSTATE_UINT8(tx_n_edges, ESP32P4UartState),
        VMSTATE_UINT8(tx_next_edge, ESP32P4UartState),
        VMSTATE_UINT16_ARRAY(tx_t16, ESP32P4UartState, ESP32P4_UART_TX_EDGES),
        VMSTATE_UINT8_ARRAY(tx_lvl, ESP32P4UartState, ESP32P4_UART_TX_EDGES),
        VMSTATE_UINT32(tx_end16, ESP32P4UartState),
        VMSTATE_INT64(tx_start_ns, ESP32P4UartState),
        VMSTATE_UINT64(tx_q32, ESP32P4UartState),
        VMSTATE_UINT64(tx_pos16, ESP32P4UartState),
        VMSTATE_INT64(tx_free_ns, ESP32P4UartState),
        VMSTATE_UINT8(tx_byte, ESP32P4UartState),
        VMSTATE_UINT8(tx_frame_bits, ESP32P4UartState),
        VMSTATE_UINT8(tx_data_bits, ESP32P4UartState),
        VMSTATE_BOOL(tx_parity, ESP32P4UartState),
        VMSTATE_BOOL(tx_in_transfer, ESP32P4UartState),
        VMSTATE_BOOL(sw_xoff_rcvd, ESP32P4UartState),
        VMSTATE_BOOL(sw_xoff_sent, ESP32P4UartState),
        VMSTATE_BOOL(send_xon, ESP32P4UartState),
        VMSTATE_BOOL(send_xoff, ESP32P4UartState),
        VMSTATE_STRUCT(rx, ESP32P4UartState, 1, vmstate_esp32_uart_rx,
                       Esp32UartRx),
        VMSTATE_TIMER(filt_timer, ESP32P4UartState),
        VMSTATE_BOOL(filt_pending, ESP32P4UartState),
        VMSTATE_BOOL(filt_level, ESP32P4UartState),
        VMSTATE_INT64(filt_ns, ESP32P4UartState),
        VMSTATE_BOOL(filt_out, ESP32P4UartState),
        VMSTATE_TIMER(tout_timer, ESP32P4UartState),
        VMSTATE_TIMER(idle_timer, ESP32P4UartState),
        VMSTATE_INT64(rx_last_end_ns, ESP32P4UartState),
        VMSTATE_BOOL(rx_had_byte, ESP32P4UartState),
        VMSTATE_TIMER(irda_timer, ESP32P4UartState),
        VMSTATE_BOOL(irda_low, ESP32P4UartState),
        VMSTATE_INT64(irda_high_ns, ESP32P4UartState),
        VMSTATE_BOOL(irda_pin, ESP32P4UartState),
        VMSTATE_UINT32(rxd_edge_cnt, ESP32P4UartState),
        VMSTATE_UINT32(lowpulse, ESP32P4UartState),
        VMSTATE_UINT32(highpulse, ESP32P4UartState),
        VMSTATE_UINT32(pospulse, ESP32P4UartState),
        VMSTATE_UINT32(negpulse, ESP32P4UartState),
        VMSTATE_INT64(ab_last_edge_ns, ESP32P4UartState),
        VMSTATE_INT64(ab_last_rise_ns, ESP32P4UartState),
        VMSTATE_INT64(ab_last_fall_ns, ESP32P4UartState),
        VMSTATE_BOOL(ab_active, ESP32P4UartState),
        VMSTATE_TIMER(at_timer, ESP32P4UartState),
        VMSTATE_UINT32(at_count, ESP32P4UartState),
        VMSTATE_INT64(at_last_end_ns, ESP32P4UartState),
        VMSTATE_BOOL(at_seen_byte, ESP32P4UartState),
        VMSTATE_UINT32(wake_edges, ESP32P4UartState),
        VMSTATE_UINT8_ARRAY(wake_chars, ESP32P4UartState, 5),
        VMSTATE_UINT8(wake_nchars, ESP32P4UartState),
        VMSTATE_BOOL(wakeup, ESP32P4UartState),
        VMSTATE_INT64(echo_start_ns, ESP32P4UartState),
        VMSTATE_INT64(echo_end_ns, ESP32P4UartState),
        VMSTATE_UINT8(echo_byte, ESP32P4UartState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property uart_properties[] = {
    DEFINE_PROP_BOOL("lp", ESP32P4UartState, lp, false),
    DEFINE_PROP_STRING("name", ESP32P4UartState, name),
};

static void uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32UartLineIfClass *lc = ESP32_UART_LINE_IF_CLASS(klass);

    dc->realize = uart_realize;
    dc->vmsd = &vmstate_esp32p4_uart;
    device_class_set_props(dc, uart_properties);
    rc->phases.hold = uart_reset_hold;
    rc->phases.exit = uart_reset_exit;
    lc->get_format = uart_line_get_format;
    lc->add_format_notifier = uart_line_add_format_notifier;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static const TypeInfo uart_info = {
    .name = TYPE_ESP32P4_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4UartState),
    .instance_init = uart_init,
    .class_init = uart_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_ESP32_UART_LINE_IF },
        { }
    },
};

static void uart_register_types(void)
{
    type_register_static(&uart_info);
}

type_init(uart_register_types)
