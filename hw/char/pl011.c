/*
 * Arm PrimeCell PL011 UART
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/*
 * QEMU interface:
 *  + sysbus MMIO region 0: device registers
 *  + sysbus IRQ 0: UARTINTR (combined interrupt line)
 *  + sysbus IRQ 1: UARTRXINTR (receive FIFO interrupt line)
 *  + sysbus IRQ 2: UARTTXINTR (transmit FIFO interrupt line)
 *  + sysbus IRQ 3: UARTRTINTR (receive timeout interrupt line)
 *  + sysbus IRQ 4: UARTMSINTR (momem status interrupt line)
 *  + sysbus IRQ 5: UARTEINTR (error interrupt line)
 *  + named GPIO outputs "dma-req": UARTTXDMASREQ, UARTRXDMASREQ
 *  + in line-level mode, named GPIO outputs "txd" and "nrts" and inputs
 *    "rxd" and "ncts": the UART's pins (see pl011.h)
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/char/pl011.h"
#include "hw/char/uart-line.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

DeviceState *pl011_create(hwaddr addr, qemu_irq irq, Chardev *chr)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_new("pl011");
    s = SYS_BUS_DEVICE(dev);
    qdev_prop_set_chr(dev, "chardev", chr);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, addr);
    sysbus_connect_irq(s, 0, irq);

    return dev;
}

/* Flag Register, UARTFR */
#define PL011_FLAG_RI   0x100
#define PL011_FLAG_TXFE 0x80
#define PL011_FLAG_RXFF 0x40
#define PL011_FLAG_TXFF 0x20
#define PL011_FLAG_RXFE 0x10
#define PL011_FLAG_BUSY 0x08
#define PL011_FLAG_DCD  0x04
#define PL011_FLAG_DSR  0x02
#define PL011_FLAG_CTS  0x01

/* Data Register, UARTDR, and the receive FIFO's error bits */
#define DR_FE   (1 << 8)
#define DR_PE   (1 << 9)
#define DR_BE   (1 << 10)
#define DR_OE   (1 << 11)

/* Interrupt status bits in UARTRIS, UARTMIS, UARTIMSC */
#define INT_OE (1 << 10)
#define INT_BE (1 << 9)
#define INT_PE (1 << 8)
#define INT_FE (1 << 7)
#define INT_RT (1 << 6)
#define INT_TX (1 << 5)
#define INT_RX (1 << 4)
#define INT_DSR (1 << 3)
#define INT_DCD (1 << 2)
#define INT_CTS (1 << 1)
#define INT_RI (1 << 0)
#define INT_E (INT_OE | INT_BE | INT_PE | INT_FE)
#define INT_MS (INT_RI | INT_DSR | INT_DCD | INT_CTS)

/* DMA Control Register, UARTDMACR */
#define DMACR_RXDMAE    (1 << 0)
#define DMACR_TXDMAE    (1 << 1)
#define DMACR_DMAONERR  (1 << 2)

/* Line Control Register, UARTLCR_H */
#define LCR_SPS     (1 << 7)
#define LCR_FEN     (1 << 4)
#define LCR_STP2    (1 << 3)
#define LCR_EPS     (1 << 2)
#define LCR_PEN     (1 << 1)
#define LCR_BRK     (1 << 0)

/* Control Register, UARTCR */
#define CR_CTSEN    (1 << 15)
#define CR_RTSEN    (1 << 14)
#define CR_OUT2     (1 << 13)
#define CR_OUT1     (1 << 12)
#define CR_RTS      (1 << 11)
#define CR_DTR      (1 << 10)
#define CR_RXE      (1 << 9)
#define CR_TXE      (1 << 8)
#define CR_LBE      (1 << 7)
#define CR_UARTEN   (1 << 0)

/* Integer Baud Rate Divider, UARTIBRD */
#define IBRD_MASK 0xffff

/* Fractional Baud Rate Divider, UARTFBRD */
#define FBRD_MASK 0x3f

static const unsigned char pl011_id_arm[8] =
  { 0x11, 0x10, 0x14, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };
static const unsigned char pl011_id_luminary[8] =
  { 0x11, 0x00, 0x18, 0x01, 0x0d, 0xf0, 0x05, 0xb1 };

static const char *pl011_regname(hwaddr offset)
{
    static const char *const rname[] = {
        [0] = "DR", [1] = "RSR", [6] = "FR", [8] = "ILPR", [9] = "IBRD",
        [10] = "FBRD", [11] = "LCRH", [12] = "CR", [13] = "IFLS", [14] = "IMSC",
        [15] = "RIS", [16] = "MIS", [17] = "ICR", [18] = "DMACR",
    };
    unsigned idx = offset >> 2;

    if (idx < ARRAY_SIZE(rname) && rname[idx]) {
        return rname[idx];
    }
    if (idx >= 0x3f8 && idx <= 0x400) {
        return "ID";
    }
    return "UNKN";
}

/* Which bits in the interrupt status matter for each outbound IRQ line ? */
static const uint32_t irqmask[] = {
    INT_E | INT_MS | INT_RT | INT_TX | INT_RX, /* combined IRQ */
    INT_RX,
    INT_TX,
    INT_RT,
    INT_MS,
    INT_E,
};

static inline unsigned pl011_get_fifo_depth(PL011State *s);

/*
 * The single-transfer DMA requests. The transmit request asks for a
 * character while the transmit FIFO has room; outside line-level mode,
 * characters are transmitted as soon as they are written, so it always
 * has. The receive request follows the receive FIFO. DMAONERR masks the
 * receive request while an error interrupt is pending.
 */
static void pl011_update_dma(PL011State *s)
{
    bool en = s->cr & CR_UARTEN;
    bool rx_err = (s->dmacr & DMACR_DMAONERR) && (s->int_level & INT_E);
    bool tx_room = !s->line_level ||
                   s->tx_count < pl011_get_fifo_depth(s);

    qemu_set_irq(s->dma_req[PL011_DMA_TX],
                 en && (s->dmacr & DMACR_TXDMAE) && tx_room);
    qemu_set_irq(s->dma_req[PL011_DMA_RX],
                 en && (s->dmacr & DMACR_RXDMAE) && s->read_count > 0 &&
                 !rx_err);
}

static void pl011_update(PL011State *s)
{
    uint32_t flags;
    int i;

    flags = s->int_level & s->int_enabled;
    trace_pl011_irq_state(flags != 0);
    for (i = 0; i < ARRAY_SIZE(s->irq); i++) {
        qemu_set_irq(s->irq[i], (flags & irqmask[i]) != 0);
    }
    pl011_update_dma(s);
}

static bool pl011_loopback_enabled(PL011State *s)
{
    return !!(s->cr & CR_LBE);
}

static bool pl011_is_fifo_enabled(PL011State *s)
{
    return (s->lcr & LCR_FEN) != 0;
}

static inline unsigned pl011_get_fifo_depth(PL011State *s)
{
    /* Note: FIFO depth is expected to be power-of-2 */
    return pl011_is_fifo_enabled(s) ? s->fifo_depth : 1;
}

static inline void pl011_reset_rx_fifo(PL011State *s)
{
    s->read_count = 0;
    s->read_pos = 0;

    /* Reset FIFO flags */
    s->flags &= ~PL011_FLAG_RXFF;
    s->flags |= PL011_FLAG_RXFE;
}

static inline void pl011_reset_tx_fifo(PL011State *s)
{
    /* Reset FIFO flags */
    s->flags &= ~PL011_FLAG_TXFF;
    s->flags |= PL011_FLAG_TXFE;
}

static void pl011_fifo_rx_put(void *opaque, uint32_t value)
{
    PL011State *s = (PL011State *)opaque;
    int slot;
    unsigned pipe_depth;

    pipe_depth = pl011_get_fifo_depth(s);
    slot = (s->read_pos + s->read_count) & (pipe_depth - 1);
    s->read_fifo[slot] = value;
    s->read_count++;
    s->flags &= ~PL011_FLAG_RXFE;
    trace_pl011_fifo_rx_put(value, s->read_count, pipe_depth);
    if (s->read_count == pipe_depth) {
        trace_pl011_fifo_rx_full();
        s->flags |= PL011_FLAG_RXFF;
    }
    if (s->read_count == s->read_trigger) {
        s->int_level |= INT_RX;
    }
    pl011_update(s);
}

static void pl011_loopback_tx(PL011State *s, uint32_t value)
{
    if (!pl011_loopback_enabled(s)) {
        return;
    }

    /*
     * Caveat:
     *
     * In real hardware, TX loopback happens at the serial-bit level
     * and then reassembled by the RX logics back into bytes and placed
     * into the RX fifo. That is, loopback happens after TX fifo.
     *
     * Because the real hardware TX fifo is time-drained at the frame
     * rate governed by the configured serial format, some loopback
     * bytes in TX fifo may still be able to get into the RX fifo
     * that could be full at times while being drained at software
     * pace.
     *
     * In such scenario, the RX draining pace is the major factor
     * deciding which loopback bytes get into the RX fifo, unless
     * hardware flow-control is enabled.
     *
     * For simplicity, the above described is not emulated.
     */
    pl011_fifo_rx_put(s, value);
}

/*
 * Line-level mode.
 *
 * The transmitter takes characters from the transmit FIFO into its shift
 * register and drives each frame on UARTTXD from a timer at the frame's
 * bit boundaries, timed from the divisor latched by the last UARTLCR_H
 * write: one bit is 16 Baud16 periods of BRD/64 UARTCLK cycles. The
 * receiver starts a frame at a falling edge on its input, checks the
 * start bit half a bit later and samples each following bit one bit
 * period apart, once in the middle of the bit rather than as a majority
 * of three. UARTEN, TXE and RXE gate the start of a frame; a frame in
 * progress completes, as on hardware. The frames follow UARTCLK: a change
 * of its rate takes effect from the change, and while it is stopped the
 * frames in progress hold where they are and no frame starts.
 */

/* Time of `ticks` Baud16 periods at divisor `brd`. */
static int64_t pl011_baud16_ns(PL011State *s, uint32_t brd, unsigned ticks)
{
    return (uint64_t)ticks * brd * NANOSECONDS_PER_SECOND /
           (64 * (uint64_t)clock_get_hz(s->clk));
}

/* A divisor below 1.0 (IBRD 0) does not run the baud rate generator. */
static bool pl011_baud_runs(PL011State *s, uint32_t brd)
{
    return brd >= 64 && clock_get_hz(s->clk) != 0;
}

static unsigned pl011_word_bits(uint32_t lcr)
{
    return 5 + extract32(lcr, 5, 2);
}

/* The parity bit of `data`, with UARTLCR_H's EPS and SPS. */
static unsigned pl011_parity(uint32_t lcr, uint32_t data)
{
    if (lcr & LCR_SPS) {
        return !(lcr & LCR_EPS);
    }
    return (ctpop32(data) & 1) ^ !(lcr & LCR_EPS);
}

/*
 * FIFO trigger levels, from UARTIFLS: 1/8, 1/4, 1/2, 3/4 or 7/8 full.
 * In character mode the receive level is one character and the transmit
 * level is an empty holding register.
 */
static unsigned pl011_ifls_level(PL011State *s, unsigned sel)
{
    static const uint8_t eighths[] = { 1, 2, 4, 6, 7 };

    if (sel >= ARRAY_SIZE(eighths)) {
        sel = 2;
    }
    return s->fifo_depth * eighths[sel] / 8;
}

static unsigned pl011_rx_level(PL011State *s)
{
    if (!pl011_is_fifo_enabled(s)) {
        return 1;
    }
    return pl011_ifls_level(s, extract32(s->ifl, 3, 3));
}

static unsigned pl011_tx_level(PL011State *s)
{
    if (!pl011_is_fifo_enabled(s)) {
        return 0;
    }
    return pl011_ifls_level(s, extract32(s->ifl, 0, 3));
}

static void pl011_line_rx_changed(PL011State *s, int64_t when);
static void pl011_line_cts_changed(PL011State *s, int64_t when);

static void pl011_line_set_txd(PL011State *s, int level, int64_t when)
{
    if (s->txd == level) {
        return;
    }
    s->txd = level;
    uart_line_set(s->txd_out, level, when);
    if (pl011_loopback_enabled(s)) {
        pl011_line_rx_changed(s, when);
    }
}

/*
 * nUARTRTS: with RTSEn, asserted until the receive FIFO fills to its
 * trigger level; otherwise the inverse of UARTCR.RTS.
 */
static void pl011_line_update_rts(PL011State *s, int64_t when)
{
    int level;

    if (s->cr & CR_RTSEN) {
        level = s->read_count >= pl011_rx_level(s);
    } else {
        level = !(s->cr & CR_RTS);
    }
    if (s->nrts == level) {
        return;
    }
    s->nrts = level;
    uart_line_set(s->nrts_out, level, when);
    if (pl011_loopback_enabled(s)) {
        pl011_line_cts_changed(s, when);
    }
}

/* Schedule the transmit timer for the next change of UARTTXD. */
static void pl011_line_tx_schedule(PL011State *s)
{
    int bit = s->tx_bit + 1;

    while (bit < s->tx_len &&
           ((s->tx_frame >> bit) & 1) == ((s->tx_frame >> s->tx_bit) & 1)) {
        bit++;
    }
    s->tx_bit = bit - 1;
    s->tx_next = s->tx_start + pl011_baud16_ns(s, s->tx_brd, 16 * bit);
    timer_mod(s->tx_timer, s->tx_next);
}

/*
 * Move the next character into the shift register and start its frame,
 * if the transmitter may: it is idle, enabled, not sending a break, and
 * with CTSEn, nUARTCTS is asserted.
 */
/* [spec:nuos:req:emu.uart] */
static void pl011_line_tx_start(PL011State *s, int64_t now)
{
    uint32_t data, frame;
    unsigned bits, len, level;

    if (s->tx_busy || s->tx_count == 0 || (s->lcr & LCR_BRK) ||
        (s->cr & (CR_UARTEN | CR_TXE)) != (CR_UARTEN | CR_TXE) ||
        ((s->cr & CR_CTSEN) && s->ncts_level)) {
        return;
    }
    if (!clock_get_hz(s->clk)) {
        /* UARTCLK is stopped: the frame waits for it to run. */
        return;
    }
    if (!pl011_baud_runs(s, s->brd)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl011: transmitting with a zero baud rate divisor\n");
        return;
    }

    data = s->tx_fifo[s->tx_pos];
    s->tx_pos = (s->tx_pos + 1) % PL011_FIFO_MAX;
    level = pl011_tx_level(s);
    if (s->tx_count > level && s->tx_count - 1 <= level) {
        s->int_level |= INT_TX;
    }
    s->tx_count--;

    bits = pl011_word_bits(s->lcr);
    data &= MAKE_64BIT_MASK(0, bits);
    /* Start bit 0, data LSB first, parity, then one or two stop bits. */
    frame = data << 1;
    len = 1 + bits;
    if (s->lcr & LCR_PEN) {
        frame |= pl011_parity(s->lcr, data) << len;
        len++;
    }
    if (s->lcr & LCR_STP2) {
        frame |= 3u << len;
        len += 2;
    } else {
        frame |= 1u << len;
        len++;
    }

    s->tx_busy = true;
    s->tx_frame = frame;
    s->tx_len = len;
    s->tx_bit = 0;
    s->tx_brd = s->brd;
    s->tx_start = now;
    pl011_line_set_txd(s, 0, now);
    pl011_line_tx_schedule(s);
    pl011_update(s);
}

static void pl011_line_tx_tick(void *opaque)
{
    PL011State *s = opaque;
    int64_t now = s->tx_next;

    s->tx_bit++;
    if (s->tx_bit < s->tx_len) {
        pl011_line_set_txd(s, (s->tx_frame >> s->tx_bit) & 1, now);
        pl011_line_tx_schedule(s);
        return;
    }
    s->tx_busy = false;
    /* A break begins once the frame in progress has completed. */
    pl011_line_set_txd(s, !(s->lcr & LCR_BRK), now);
    pl011_line_tx_start(s, now);
}

static void pl011_line_write_txdata(PL011State *s, uint8_t data)
{
    if (s->tx_count >= pl011_get_fifo_depth(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl011: data written to a full transmit FIFO\n");
        return;
    }
    s->tx_fifo[(s->tx_pos + s->tx_count) % PL011_FIFO_MAX] = data;
    s->tx_count++;
    if (s->tx_count > pl011_tx_level(s)) {
        s->int_level &= ~INT_TX;
    }
    pl011_line_tx_start(s, uart_line_now());
    pl011_update(s);
}

/* The receiver's input: UARTRXD, or UARTTXD in loopback. */
static void pl011_line_rx_changed(PL011State *s, int64_t when)
{
    int level = pl011_loopback_enabled(s) ? s->txd : s->rxd_pin;

    if (level == s->rx_level) {
        return;
    }
    s->rx_level = level;
    if (level) {
        s->rx_wait_mark = false;
        return;
    }
    /* A UART that its SoC holds in reset takes no input. */
    if (s->rx_bit >= 0 || s->rx_wait_mark ||
        (s->cr & (CR_UARTEN | CR_RXE)) != (CR_UARTEN | CR_RXE) ||
        !pl011_baud_runs(s, s->brd) || device_is_in_reset(DEVICE(s))) {
        return;
    }
    s->rx_bit = 0;
    s->rx_data = 0;
    s->rx_zero = true;
    s->rx_brd = s->brd;
    s->rx_lcr = s->lcr;
    s->rx_start = when;
    s->rx_next = when + pl011_baud16_ns(s, s->rx_brd, 8);
    timer_mod(s->rx_timer, s->rx_next);
}

/*
 * The modem status input nUARTCTS, or nUARTRTS in loopback. A change of
 * the CTS flag raises the CTS modem status interrupt.
 */
static void pl011_line_cts_changed(PL011State *s, int64_t when)
{
    int level = pl011_loopback_enabled(s) ? s->nrts : s->ncts_pin;

    if (level == s->ncts_level) {
        return;
    }
    s->ncts_level = level;
    s->int_level |= INT_CTS;
    pl011_update(s);
    pl011_line_tx_start(s, when);
}

/*
 * A received character with its error bits. The overrun flag goes with
 * the first character that fits after an overrun.
 */
static void pl011_line_rx_put(PL011State *s, uint32_t value, int64_t when)
{
    unsigned depth = pl011_get_fifo_depth(s);
    unsigned level = pl011_rx_level(s);

    if (s->read_count >= depth) {
        s->rx_overrun = true;
        s->rsr |= DR_OE >> 8;
        s->int_level |= INT_OE;
        pl011_update(s);
        return;
    }
    if (s->rx_overrun) {
        value |= DR_OE;
        s->rx_overrun = false;
    }
    s->read_fifo[(s->read_pos + s->read_count) & (depth - 1)] = value;
    s->read_count++;
    s->flags &= ~PL011_FLAG_RXFE;
    if (s->read_count == depth) {
        s->flags |= PL011_FLAG_RXFF;
    }
    if (s->read_count == level) {
        s->int_level |= INT_RX;
    }
    s->int_level |= (value & DR_FE ? INT_FE : 0) |
                    (value & DR_PE ? INT_PE : 0) |
                    (value & DR_BE ? INT_BE : 0);
    /* The receive timeout: 32 bit periods without another character. */
    timer_mod(s->rt_timer, when + pl011_baud16_ns(s, s->rx_brd, 32 * 16));
    pl011_line_update_rts(s, when);
    pl011_update(s);
}

/* [spec:nuos:req:emu.uart] */
static void pl011_line_rx_sample(void *opaque)
{
    PL011State *s = opaque;
    int64_t now = s->rx_next;
    unsigned bits = pl011_word_bits(s->rx_lcr);
    unsigned parity = (s->rx_lcr & LCR_PEN) ? 1 : 0;
    int level = s->rx_level;
    uint32_t value;

    if (s->rx_bit == 0) {
        if (level) {
            /* The input went high again: not a valid start bit. */
            s->rx_bit = -1;
            return;
        }
    } else if (s->rx_bit <= bits + parity) {
        s->rx_data |= level << (s->rx_bit - 1);
        s->rx_zero &= !level;
    } else {
        /* The (first) stop bit. */
        s->rx_bit = -1;
        if (!level && s->rx_zero) {
            /*
             * A break: the input held low for a whole frame. One zero
             * character is received, and the receiver waits for the
             * input to return high before it detects a start bit.
             */
            value = DR_BE;
            s->rx_wait_mark = true;
        } else {
            value = extract32(s->rx_data, 0, bits);
            if (!level) {
                value |= DR_FE;
            }
            if (parity && extract32(s->rx_data, bits, 1) !=
                pl011_parity(s->rx_lcr, value & 0xff)) {
                value |= DR_PE;
            }
        }
        pl011_line_rx_put(s, value, now);
        return;
    }
    s->rx_bit++;
    s->rx_next = s->rx_start + pl011_baud16_ns(s, s->rx_brd,
                                               8 + 16 * s->rx_bit);
    timer_mod(s->rx_timer, s->rx_next);
}

static void pl011_line_rt_tick(void *opaque)
{
    PL011State *s = opaque;

    if (s->read_count > 0) {
        s->int_level |= INT_RT;
        pl011_update(s);
    }
}

static uint32_t pl011_line_read_rxdata(PL011State *s)
{
    unsigned depth = pl011_get_fifo_depth(s);
    uint32_t c = s->read_fifo[s->read_pos];

    if (s->read_count > 0) {
        if (s->read_count == pl011_rx_level(s)) {
            s->int_level &= ~INT_RX;
        }
        s->read_count--;
        s->read_pos = (s->read_pos + 1) & (depth - 1);
    }
    s->flags &= ~PL011_FLAG_RXFF;
    if (s->read_count == 0) {
        s->flags |= PL011_FLAG_RXFE;
        s->int_level &= ~INT_RT;
        timer_del(s->rt_timer);
    }
    s->rsr = (c >> 8) & 0xf;
    pl011_line_update_rts(s, uart_line_now());
    pl011_update(s);
    return c;
}

static uint32_t pl011_line_flags(PL011State *s)
{
    unsigned depth = pl011_get_fifo_depth(s);
    uint32_t r = s->flags & (PL011_FLAG_RXFF | PL011_FLAG_RXFE);

    if (s->tx_count == 0) {
        r |= PL011_FLAG_TXFE;
    }
    if (s->tx_count >= depth) {
        r |= PL011_FLAG_TXFF;
    }
    if (s->tx_count || s->tx_busy) {
        r |= PL011_FLAG_BUSY;
    }
    if (!s->ncts_level) {
        r |= PL011_FLAG_CTS;
    }
    return r;
}

static void pl011_line_write_lcr(PL011State *s, uint32_t old)
{
    int64_t now = uart_line_now();

    /* UARTLCR_H writes load the divisors into the baud rate generator. */
    s->brd = (s->ibrd << 6) | s->fbrd;
    if ((old ^ s->lcr) & LCR_FEN) {
        s->tx_count = 0;
        s->tx_pos = 0;
    }
    if ((old ^ s->lcr) & LCR_BRK) {
        if (!s->tx_busy) {
            pl011_line_set_txd(s, !(s->lcr & LCR_BRK), now);
        }
    }
    pl011_line_update_rts(s, now);
    pl011_line_tx_start(s, now);
}

static void pl011_line_write_cr(PL011State *s, uint32_t old)
{
    int64_t now = uart_line_now();

    if ((old ^ s->cr) & CR_LBE) {
        pl011_line_rx_changed(s, now);
    }
    pl011_line_update_rts(s, now);
    pl011_line_cts_changed(s, now);
    pl011_line_tx_start(s, now);
}

static void pl011_line_rxd_in(void *opaque, int n, int level)
{
    PL011State *s = opaque;

    s->rxd_pin = level != 0;
    pl011_line_rx_changed(s, uart_line_now());
}

static void pl011_line_ncts_in(void *opaque, int n, int level)
{
    PL011State *s = opaque;

    s->ncts_pin = level != 0;
    pl011_line_cts_changed(s, uart_line_now());
}

static void pl011_line_reset(PL011State *s)
{
    timer_del(s->tx_timer);
    timer_del(s->rx_timer);
    timer_del(s->rt_timer);
    s->brd = 0;
    s->tx_pos = 0;
    s->tx_count = 0;
    s->tx_busy = false;
    s->tx_len = 0;
    s->tx_bit = 0;
    s->rx_bit = -1;
    s->rx_wait_mark = false;
    s->rx_overrun = false;
    s->tx_held = 0;
    s->rx_held = 0;
    s->rt_held = 0;
    s->rx_level = s->rxd_pin;
    s->ncts_level = s->ncts_pin;
    s->txd = 1;
    s->nrts = 1;
    qemu_set_irq(s->txd_out, 1);
    qemu_set_irq(s->nrts_out, 1);
}

static void pl011_write_txdata(PL011State *s, uint8_t data)
{
    if (!(s->cr & CR_UARTEN)) {
        /*
         * Only log this message once, not every time the guest outputs:
         * otherwise we would flood the logs with this message, making
         * harder to debug guests. (Some very popular guests like Linux
         * don't actively enable the UART.)
         */
        if (!s->logged_disabled_uart) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "PL011 data written to disabled UART\n");
            s->logged_disabled_uart = true;
        }
    }
    if (!(s->cr & CR_TXE)) {
        /*
         * We don't bother with the only-log-once machinery for this check
         * because TXE is enabled by default from PL011 reset, so there
         * isn't likely to be existing in-the-wild guest code that trips
         * over this one.
         */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "PL011 data written to disabled TX UART\n");
    }
    if (s->line_level) {
        pl011_line_write_txdata(s, data);
        return;
    }

    /*
     * XXX this blocks entire thread. Rewrite to use
     * qemu_chr_fe_write and background I/O callbacks
     */
    qemu_chr_fe_write_all(&s->chr, &data, 1);
    pl011_loopback_tx(s, data);
    s->int_level |= INT_TX;
    pl011_update(s);
}

static uint32_t pl011_read_rxdata(PL011State *s)
{
    uint32_t c;
    unsigned fifo_depth = pl011_get_fifo_depth(s);

    s->flags &= ~PL011_FLAG_RXFF;
    c = s->read_fifo[s->read_pos];
    if (s->read_count > 0) {
        s->read_count--;
        s->read_pos = (s->read_pos + 1) & (fifo_depth - 1);
    }
    if (s->read_count == 0) {
        s->flags |= PL011_FLAG_RXFE;
    }
    if (s->read_count == s->read_trigger - 1) {
        s->int_level &= ~INT_RX;
    }
    trace_pl011_read_fifo(s->read_count, fifo_depth);
    s->rsr = c >> 8;
    pl011_update(s);
    qemu_chr_fe_accept_input(&s->chr);
    return c;
}

static uint64_t pl011_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    PL011State *s = (PL011State *)opaque;
    uint64_t r;

    switch (offset >> 2) {
    case 0: /* UARTDR */
        r = s->line_level ? pl011_line_read_rxdata(s) : pl011_read_rxdata(s);
        break;
    case 1: /* UARTRSR */
        r = s->rsr;
        break;
    case 6: /* UARTFR */
        r = s->line_level ? pl011_line_flags(s) : s->flags;
        break;
    case 8: /* UARTILPR */
        r = s->ilpr;
        break;
    case 9: /* UARTIBRD */
        r = s->ibrd;
        break;
    case 10: /* UARTFBRD */
        r = s->fbrd;
        break;
    case 11: /* UARTLCR_H */
        r = s->lcr;
        break;
    case 12: /* UARTCR */
        r = s->cr;
        break;
    case 13: /* UARTIFLS */
        r = s->ifl;
        break;
    case 14: /* UARTIMSC */
        r = s->int_enabled;
        break;
    case 15: /* UARTRIS */
        r = s->int_level;
        break;
    case 16: /* UARTMIS */
        r = s->int_level & s->int_enabled;
        break;
    case 18: /* UARTDMACR */
        r = s->dmacr;
        break;
    case 0x3f8 ... 0x400:
        r = s->id[(offset - 0xfe0) >> 2];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl011_read: Bad offset 0x%x\n", (int)offset);
        r = 0;
        break;
    }

    trace_pl011_read(offset, r, pl011_regname(offset));
    return r;
}

static void pl011_set_read_trigger(PL011State *s)
{
#if 0
    /* The docs say the RX interrupt is triggered when the FIFO exceeds
       the threshold.  However linux only reads the FIFO in response to an
       interrupt.  Triggering the interrupt when the FIFO is non-empty seems
       to make things work.  */
    if (s->lcr & LCR_FEN)
        s->read_trigger = (s->ifl >> 1) & 0x1c;
    else
#endif
        s->read_trigger = 1;
}

static unsigned int pl011_get_baudrate(const PL011State *s)
{
    uint64_t clk;

    if (s->ibrd == 0) {
        return 0;
    }

    clk = clock_get_hz(s->clk);
    return (clk / ((s->ibrd << 6) + s->fbrd)) << 2;
}

static void pl011_trace_baudrate_change(const PL011State *s)
{
    trace_pl011_baudrate_change(pl011_get_baudrate(s),
                                clock_get_hz(s->clk),
                                s->ibrd, s->fbrd);
}

static void pl011_loopback_mdmctrl(PL011State *s)
{
    uint32_t cr, fr, il;

    if (!pl011_loopback_enabled(s)) {
        return;
    }

    /*
     * Loopback software-driven modem control outputs to modem status inputs:
     *   FR.RI  <= CR.Out2
     *   FR.DCD <= CR.Out1
     *   FR.CTS <= CR.RTS
     *   FR.DSR <= CR.DTR
     *
     * The loopback happens immediately even if this call is triggered
     * by setting only CR.LBE.
     *
     * CTS/RTS updates due to enabled hardware flow controls are not
     * dealt with here.
     */
    cr = s->cr;
    fr = s->flags & ~(PL011_FLAG_RI | PL011_FLAG_DCD |
                      PL011_FLAG_DSR | PL011_FLAG_CTS);
    fr |= (cr & CR_OUT2) ? PL011_FLAG_RI  : 0;
    fr |= (cr & CR_OUT1) ? PL011_FLAG_DCD : 0;
    fr |= (cr & CR_RTS)  ? PL011_FLAG_CTS : 0;
    fr |= (cr & CR_DTR)  ? PL011_FLAG_DSR : 0;

    /* Change interrupts based on updated FR */
    il = s->int_level & ~(INT_DSR | INT_DCD | INT_CTS | INT_RI);
    il |= (fr & PL011_FLAG_DSR) ? INT_DSR : 0;
    il |= (fr & PL011_FLAG_DCD) ? INT_DCD : 0;
    il |= (fr & PL011_FLAG_CTS) ? INT_CTS : 0;
    il |= (fr & PL011_FLAG_RI)  ? INT_RI  : 0;

    s->flags = fr;
    s->int_level = il;
    pl011_update(s);
}

static void pl011_loopback_break(PL011State *s, int brk_enable)
{
    if (brk_enable) {
        pl011_loopback_tx(s, DR_BE);
    }
}

static inline void pl011_set_break(PL011State *s, int brk_enable)
{
    if (s->line_level) {
        return;
    }
    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK, &brk_enable);
}

static void pl011_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    PL011State *s = (PL011State *)opaque;
    unsigned char ch;
    uint32_t old;

    trace_pl011_write(offset, value, pl011_regname(offset));

    switch (offset >> 2) {
    case 0: /* UARTDR */
        ch = value;
        pl011_write_txdata(s, ch);
        break;
    case 1: /* UARTRSR/UARTECR */
        s->rsr = 0;
        break;
    case 6: /* UARTFR */
        /* Writes to Flag register are ignored.  */
        break;
    case 8: /* UARTILPR */
        s->ilpr = value;
        break;
    case 9: /* UARTIBRD */
        s->ibrd = value & IBRD_MASK;
        pl011_trace_baudrate_change(s);
        break;
    case 10: /* UARTFBRD */
        s->fbrd = value & FBRD_MASK;
        pl011_trace_baudrate_change(s);
        break;
    case 11: /* UARTLCR_H */
        old = s->lcr;
        /* Reset the FIFO state on FIFO enable or disable */
        if ((s->lcr ^ value) & LCR_FEN) {
            pl011_reset_rx_fifo(s);
            pl011_reset_tx_fifo(s);
        }
        if (((s->lcr ^ value) & LCR_BRK) && !s->line_level) {
            bool break_enable = value & LCR_BRK;
            pl011_set_break(s, break_enable);
            pl011_loopback_break(s, break_enable);
        }
        s->lcr = value;
        pl011_set_read_trigger(s);
        if (s->line_level) {
            pl011_line_write_lcr(s, old);
        }
        pl011_update_dma(s);
        break;
    case 12: /* UARTCR */
        /* ??? Need to implement the enable bit.  */
        if ((s->cr ^ value) & CR_UARTEN) {
            /* Re-arm the log warning when the guest toggles UARTEN */
            s->logged_disabled_uart = false;
        }
        old = s->cr;
        s->cr = value;
        if (s->line_level) {
            pl011_line_write_cr(s, old);
        } else {
            pl011_loopback_mdmctrl(s);
        }
        pl011_update_dma(s);
        break;
    case 13: /* UARTIFS */
        s->ifl = value;
        pl011_set_read_trigger(s);
        if (s->line_level) {
            pl011_line_update_rts(s, uart_line_now());
        }
        break;
    case 14: /* UARTIMSC */
        s->int_enabled = value;
        pl011_update(s);
        break;
    case 17: /* UARTICR */
        s->int_level &= ~value;
        pl011_update(s);
        break;
    case 18: /* UARTDMACR */
        s->dmacr = value;
        if ((value & (DMACR_RXDMAE | DMACR_TXDMAE)) &&
            !s->dma_req[PL011_DMA_TX] && !s->dma_req[PL011_DMA_RX]) {
            qemu_log_mask(LOG_UNIMP, "pl011: DMA not connected\n");
        }
        pl011_update_dma(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl011_write: Bad offset 0x%x\n", (int)offset);
    }
}

static int pl011_can_receive(void *opaque)
{
    PL011State *s = (PL011State *)opaque;
    unsigned fifo_depth = pl011_get_fifo_depth(s);
    unsigned fifo_available = fifo_depth - s->read_count;

    /*
     * In theory we should check the UART and RX enable bits here and
     * return 0 if they are not set (so the guest can't receive data
     * until you have enabled the UART). In practice we suspect there
     * is at least some guest code out there which has been tested only
     * on QEMU and which never bothers to enable the UART because we
     * historically never enforced that. So we effectively keep the
     * UART continuously enabled regardless of the enable bits.
     */

    /* A UART that its SoC holds in reset takes no input. */
    if (device_is_in_reset(DEVICE(s))) {
        return 0;
    }
    trace_pl011_can_receive(s->lcr, s->read_count, fifo_depth, fifo_available);
    return fifo_available;
}

static void pl011_receive(void *opaque, const uint8_t *buf, int size)
{
    trace_pl011_receive(size);
    /*
     * In loopback mode, the RX input signal is internally disconnected
     * from the entire receiving logics; thus, all inputs are ignored,
     * and BREAK detection on RX input signal is also not performed.
     */
    if (pl011_loopback_enabled(opaque)) {
        return;
    }

    for (int i = 0; i < size; i++) {
        pl011_fifo_rx_put(opaque, buf[i]);
    }
}

static void pl011_event(void *opaque, QEMUChrEvent event)
{
    if (event == CHR_EVENT_BREAK && !pl011_loopback_enabled(opaque) &&
        !device_is_in_reset(DEVICE(opaque))) {
        pl011_fifo_rx_put(opaque, DR_BE);
    }
}

/*
 * In line-level mode, carry the frames in progress across a change of
 * UARTCLK: before it, note the cycles they have run and the receive
 * timeout has left; after it, re-time them at the new rate, or hold them
 * while the clock is stopped.
 */
static void pl011_line_clock_update(PL011State *s, ClockEvent event)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        if (!clock_get(s->clk)) {
            return;
        }
        if (s->tx_busy) {
            s->tx_held = clock_ns_to_ticks(s->clk, MAX(now - s->tx_start, 0));
            timer_del(s->tx_timer);
        }
        if (s->rx_bit >= 0) {
            s->rx_held = clock_ns_to_ticks(s->clk, MAX(now - s->rx_start, 0));
            timer_del(s->rx_timer);
        }
        if (timer_pending(s->rt_timer)) {
            s->rt_held = clock_ns_to_ticks(s->clk,
                MAX(timer_expire_time_ns(s->rt_timer) - now, 0));
            timer_del(s->rt_timer);
        } else {
            s->rt_held = 0;
        }
        return;
    }
    if (!clock_get(s->clk)) {
        return;
    }
    if (s->tx_busy) {
        s->tx_start = now - clock_ticks_to_ns(s->clk, s->tx_held);
        s->tx_next = s->tx_start +
                     pl011_baud16_ns(s, s->tx_brd, 16 * (s->tx_bit + 1));
        timer_mod(s->tx_timer, MAX(s->tx_next, now));
    }
    if (s->rx_bit >= 0) {
        s->rx_start = now - clock_ticks_to_ns(s->clk, s->rx_held);
        s->rx_next = s->rx_start +
                     pl011_baud16_ns(s, s->rx_brd, 8 + 16 * s->rx_bit);
        timer_mod(s->rx_timer, MAX(s->rx_next, now));
    }
    if (s->rt_held) {
        timer_mod(s->rt_timer, now + clock_ticks_to_ns(s->clk, s->rt_held));
        s->rt_held = 0;
    }
    pl011_line_tx_start(s, now);
}

static void pl011_clock_update(void *opaque, ClockEvent event)
{
    PL011State *s = PL011(opaque);

    if (s->line_level) {
        pl011_line_clock_update(s, event);
    }
    if (event == ClockUpdate) {
        pl011_trace_baudrate_change(s);
    }
}

static const MemoryRegionOps pl011_ops = {
    .read = pl011_read,
    .write = pl011_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static bool pl011_clock_needed(void *opaque)
{
    PL011State *s = PL011(opaque);

    return s->migrate_clk;
}

static const VMStateDescription vmstate_pl011_clock = {
    .name = "pl011/clock",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = pl011_clock_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, PL011State),
        VMSTATE_END_OF_LIST()
    }
};

static int pl011_post_load(void *opaque, int version_id)
{
    PL011State* s = opaque;

    /* Sanity-check input state */
    if (s->read_pos >= s->fifo_depth || s->read_count > s->fifo_depth) {
        return -1;
    }
    if (s->line_level &&
        (s->tx_pos >= PL011_FIFO_MAX || s->tx_count > s->fifo_depth)) {
        return -1;
    }

    if (!pl011_is_fifo_enabled(s) && s->read_count > 0 && s->read_pos > 0) {
        /*
         * Older versions of PL011 didn't ensure that the single
         * character in the FIFO in FIFO-disabled mode is in
         * element 0 of the array; convert to follow the current
         * code's assumptions.
         */
        s->read_fifo[0] = s->read_fifo[s->read_pos];
        s->read_pos = 0;
    }

    s->ibrd &= IBRD_MASK;
    s->fbrd &= FBRD_MASK;

    return 0;
}

static bool pl011_line_needed(void *opaque)
{
    PL011State *s = PL011(opaque);

    return s->line_level || s->fifo_depth > PL011_FIFO_DEPTH;
}

static const VMStateDescription vmstate_pl011_line = {
    .name = "pl011/line",
    .version_id = 2,
    .minimum_version_id = 1,
    .needed = pl011_line_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_SUB_ARRAY(read_fifo, PL011State, PL011_FIFO_DEPTH,
                                 PL011_FIFO_MAX - PL011_FIFO_DEPTH),
        VMSTATE_TIMER_PTR(tx_timer, PL011State),
        VMSTATE_TIMER_PTR(rx_timer, PL011State),
        VMSTATE_TIMER_PTR(rt_timer, PL011State),
        VMSTATE_UINT32(brd, PL011State),
        VMSTATE_UINT8_ARRAY(tx_fifo, PL011State, PL011_FIFO_MAX),
        VMSTATE_INT32(tx_pos, PL011State),
        VMSTATE_INT32(tx_count, PL011State),
        VMSTATE_BOOL(tx_busy, PL011State),
        VMSTATE_UINT32(tx_frame, PL011State),
        VMSTATE_INT32(tx_len, PL011State),
        VMSTATE_INT32(tx_bit, PL011State),
        VMSTATE_UINT32(tx_brd, PL011State),
        VMSTATE_INT64(tx_start, PL011State),
        VMSTATE_INT64(tx_next, PL011State),
        VMSTATE_INT32(rx_bit, PL011State),
        VMSTATE_UINT32(rx_data, PL011State),
        VMSTATE_UINT32(rx_brd, PL011State),
        VMSTATE_UINT32(rx_lcr, PL011State),
        VMSTATE_INT64(rx_start, PL011State),
        VMSTATE_INT64(rx_next, PL011State),
        VMSTATE_BOOL(rx_zero, PL011State),
        VMSTATE_BOOL(rx_wait_mark, PL011State),
        VMSTATE_BOOL(rx_overrun, PL011State),
        VMSTATE_UINT8(rxd_pin, PL011State),
        VMSTATE_UINT8(ncts_pin, PL011State),
        VMSTATE_UINT8(rx_level, PL011State),
        VMSTATE_UINT8(ncts_level, PL011State),
        VMSTATE_UINT8(txd, PL011State),
        VMSTATE_UINT8(nrts, PL011State),
        VMSTATE_UINT64_V(tx_held, PL011State, 2),
        VMSTATE_UINT64_V(rx_held, PL011State, 2),
        VMSTATE_UINT64_V(rt_held, PL011State, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pl011 = {
    .name = "pl011",
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = pl011_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UNUSED(sizeof(uint32_t)),
        VMSTATE_UINT32(flags, PL011State),
        VMSTATE_UINT32(lcr, PL011State),
        VMSTATE_UINT32(rsr, PL011State),
        VMSTATE_UINT32(cr, PL011State),
        VMSTATE_UINT32(dmacr, PL011State),
        VMSTATE_UINT32(int_enabled, PL011State),
        VMSTATE_UINT32(int_level, PL011State),
        VMSTATE_UINT32_SUB_ARRAY(read_fifo, PL011State, 0, PL011_FIFO_DEPTH),
        VMSTATE_UINT32(ilpr, PL011State),
        VMSTATE_UINT32(ibrd, PL011State),
        VMSTATE_UINT32(fbrd, PL011State),
        VMSTATE_UINT32(ifl, PL011State),
        VMSTATE_INT32(read_pos, PL011State),
        VMSTATE_INT32(read_count, PL011State),
        VMSTATE_INT32(read_trigger, PL011State),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_pl011_clock,
        &vmstate_pl011_line,
        NULL
    }
};

static const Property pl011_properties[] = {
    DEFINE_PROP_CHR("chardev", PL011State, chr),
    DEFINE_PROP_BOOL("migrate-clk", PL011State, migrate_clk, true),
    /* 16 for r1p4 and earlier, 32 for r1p5 */
    DEFINE_PROP_UINT32("fifo-depth", PL011State, fifo_depth,
                       PL011_FIFO_DEPTH),
    DEFINE_PROP_BOOL("line-level", PL011State, line_level, false),
};

static void pl011_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    PL011State *s = PL011(obj);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(s), &pl011_ops, s, "pl011", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (i = 0; i < ARRAY_SIZE(s->irq); i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), s->dma_req, PL011_DMA_REQ,
                             ARRAY_SIZE(s->dma_req));

    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", pl011_clock_update, s,
                                ClockPreUpdate | ClockUpdate);

    s->id = pl011_id_arm;

    qdev_init_gpio_out_named(DEVICE(obj), &s->txd_out, PL011_TXD, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->nrts_out, PL011_NRTS, 1);
    qdev_init_gpio_in_named(DEVICE(obj), pl011_line_rxd_in, PL011_RXD, 1);
    qdev_init_gpio_in_named(DEVICE(obj), pl011_line_ncts_in, PL011_NCTS, 1);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pl011_line_tx_tick, s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pl011_line_rx_sample, s);
    s->rt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pl011_line_rt_tick, s);
    /* Undriven inputs idle high: marking, and CTS deasserted. */
    s->rxd_pin = 1;
    s->ncts_pin = 1;
    s->rx_level = 1;
    s->ncts_level = 1;
    s->txd = 1;
    s->nrts = 1;
}

static void pl011_finalize(Object *obj)
{
    PL011State *s = PL011(obj);

    timer_free(s->tx_timer);
    timer_free(s->rx_timer);
    timer_free(s->rt_timer);
}

static int pl011_be_change(void *opaque);

static inline void pl011_set_handlers(PL011State *s)
{
    qemu_chr_fe_set_handlers(&s->chr, pl011_can_receive, pl011_receive,
                             pl011_event, pl011_be_change, s, NULL, true);
}

static int pl011_be_change(void *opaque)
{
    PL011State *s = opaque;

    pl011_set_handlers(s);
    pl011_set_break(s, s->lcr & LCR_BRK);

    return 0;
}

static void pl011_realize(DeviceState *dev, Error **errp)
{
    PL011State *s = PL011(dev);

    if (s->fifo_depth != PL011_FIFO_DEPTH && s->fifo_depth != PL011_FIFO_MAX) {
        error_setg(errp, "fifo-depth must be %d or %d", PL011_FIFO_DEPTH,
                   PL011_FIFO_MAX);
        return;
    }
    if (s->line_level) {
        if (qemu_chr_fe_backend_connected(&s->chr)) {
            error_setg(errp, "a line-level PL011 has no chardev; attach "
                       "one to its pins");
        }
        return;
    }
    pl011_set_handlers(s);
}

static void pl011_reset(DeviceState *dev)
{
    PL011State *s = PL011(dev);

    s->lcr = 0;
    s->rsr = 0;
    s->dmacr = 0;
    s->int_enabled = 0;
    s->int_level = 0;
    s->ilpr = 0;
    s->ibrd = 0;
    s->fbrd = 0;
    s->read_trigger = 1;
    s->ifl = 0x12;
    s->cr = 0x300;
    s->flags = 0;
    s->logged_disabled_uart = false;
    pl011_reset_rx_fifo(s);
    pl011_reset_tx_fifo(s);
    if (s->line_level) {
        pl011_line_reset(s);
    }
    pl011_update(s);
}

static void pl011_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pl011_realize;
    device_class_set_legacy_reset(dc, pl011_reset);
    dc->vmsd = &vmstate_pl011;
    device_class_set_props(dc, pl011_properties);
}

static const TypeInfo pl011_arm_info = {
    .name          = TYPE_PL011,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PL011State),
    .instance_init = pl011_init,
    .instance_finalize = pl011_finalize,
    .class_init    = pl011_class_init,
};

static void pl011_luminary_init(Object *obj)
{
    PL011State *s = PL011(obj);

    s->id = pl011_id_luminary;
}

static const TypeInfo pl011_luminary_info = {
    .name          = TYPE_PL011_LUMINARY,
    .parent        = TYPE_PL011,
    .instance_init = pl011_luminary_init,
};

static void pl011_register_types(void)
{
    type_register_static(&pl011_arm_info);
    type_register_static(&pl011_luminary_info);
}

type_init(pl011_register_types)
