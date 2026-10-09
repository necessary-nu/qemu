/*
 * Arm PrimeCell PL022 Synchronous Serial Port
 *
 * Copyright (c) 2007 CodeSourcery.
 * Written by Paul Brook
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/pl022.h"
#include "hw/ssi/ssi.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define PL022_CR0_DSS(cr0)  ((cr0) & 0xf)
#define PL022_CR0_FRF(cr0)  (((cr0) >> 4) & 3)
#define PL022_CR0_SPO       0x40
#define PL022_CR0_SPH       0x80
#define PL022_CR0_SCR(cr0)  (((cr0) >> 8) & 0xff)
#define PL022_CR0_MASK      0xffff

#define PL022_FRF_MOTOROLA  0
#define PL022_FRF_TI        1
#define PL022_FRF_MICROWIRE 2

#define PL022_CR1_LBM 0x01
#define PL022_CR1_SSE 0x02
#define PL022_CR1_MS  0x04
#define PL022_CR1_SOD 0x08
#define PL022_CR1_MASK 0x0f

#define PL022_SR_TFE  0x01
#define PL022_SR_TNF  0x02
#define PL022_SR_RNE  0x04
#define PL022_SR_RFF  0x08
#define PL022_SR_BSY  0x10

#define PL022_CPSR_MASK 0xfe

#define PL022_INT_ROR 0x01
#define PL022_INT_RT  0x02
#define PL022_INT_RX  0x04
#define PL022_INT_TX  0x08
#define PL022_INT_MASK 0x0f

#define PL022_DMACR_RXDMAE 0x01
#define PL022_DMACR_TXDMAE 0x02
#define PL022_DMACR_MASK   0x03

/* The TX and RX interrupts trigger at half full. */
#define PL022_FIFO_WATERMARK (PL022_FIFO_DEPTH / 2)

/*
 * The receive timeout: the receive FIFO holds data and the port has been
 * idle, with no frame received or read, for 32 bit periods.
 */
#define PL022_RT_BITS 32

/* Microwire frames start with an 8-bit control word. */
#define PL022_MW_CTRL_BITS 8

static const unsigned char pl022_id[8] =
  { 0x22, 0x10, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };

/* SSPPeriphID2 holds the revision in bits 7:4. */
#define PL022_ID_REVISION 2

static bool pl022_enabled(PL022State *s)
{
    return s->cr1 & PL022_CR1_SSE;
}

static bool pl022_slave(PL022State *s)
{
    return s->cr1 & PL022_CR1_MS;
}

static unsigned pl022_frf(PL022State *s)
{
    return PL022_CR0_FRF(s->cr0);
}

static unsigned pl022_data_bits(PL022State *s)
{
    return PL022_CR0_DSS(s->cr0) + 1;
}

/*
 * SSPCLK cycles per bit: SSPCLK / (CPSDVSR * (1 + SCR)). CPSDVSR must be
 * an even value from 2; a smaller one, which leaves the prescaler's
 * behaviour undefined, is taken as 2.
 */
static uint64_t pl022_bit_ticks(PL022State *s)
{
    return MAX(s->cpsr, 2) * (1 + PL022_CR0_SCR(s->cr0));
}

/*
 * The serial clock periods one frame occupies. Motorola SPI frames have
 * a half period of select setup before the first edge and a half period
 * of hold after the last; TI frames start with a one-period frame pulse;
 * Microwire frames are the control word, a one-period turnaround and the
 * response.
 */
static unsigned pl022_frame_bits(PL022State *s)
{
    switch (pl022_frf(s)) {
    case PL022_FRF_MICROWIRE:
        return PL022_MW_CTRL_BITS + 1 + pl022_data_bits(s);
    default:
        return pl022_data_bits(s) + 1;
    }
}

/* Frames are timed only with SSPCLK connected and running. */
static bool pl022_timed(PL022State *s)
{
    return clock_get(s->clk) != 0;
}

static int64_t pl022_ns(PL022State *s, uint64_t bits)
{
    return clock_ticks_to_ns(s->clk, pl022_bit_ticks(s) * bits);
}

/* Drive the interface signals, skipping lines whose level is unchanged. */
static void pl022_set_pin(PL022State *s, int n, int level)
{
    if (s->ssp_level[n] != level) {
        s->ssp_level[n] = level;
        qemu_set_irq(s->ssp_out[n], level);
    }
}

/*
 * The idle and frame-level states of the interface. A master drives its
 * clock and frame select (nSSPCTLOE low); the clock idles at SPO in
 * Motorola format and low otherwise. The frame select is active low for
 * Motorola SPI and Microwire, held low for the frame on the wire, and is
 * low between frames in TI format, where each frame starts with a high
 * pulse. The data output is enabled (nSSPOE low) while a master frame is
 * on the wire, and in slave mode while the slave is enabled, selected
 * and not barred from driving it by SOD.
 */
static void pl022_drive_pins(PL022State *s)
{
    bool master = !pl022_slave(s);
    bool ti = pl022_frf(s) == PL022_FRF_TI;
    bool tx_oe;

    if (master) {
        tx_oe = s->busy;
    } else {
        tx_oe = pl022_enabled(s) && !(s->cr1 & PL022_CR1_SOD) &&
                (ti || !s->fssin);
    }
    pl022_set_pin(s, PL022_NSSPCTLOE, !master);
    pl022_set_pin(s, PL022_SSPCLKOUT,
                  pl022_frf(s) == PL022_FRF_MOTOROLA &&
                  (s->cr0 & PL022_CR0_SPO));
    pl022_set_pin(s, PL022_SSPFSSOUT, ti ? 0 : !(master && s->busy));
    pl022_set_pin(s, PL022_SSPTXD, 0);
    pl022_set_pin(s, PL022_NSSPOE, !tx_oe);
}

static void pl022_update(PL022State *s)
{
    bool en = pl022_enabled(s);

    s->sr = 0;
    if (s->tx_fifo_len == 0) {
        s->sr |= PL022_SR_TFE;
    }
    if (s->tx_fifo_len != PL022_FIFO_DEPTH) {
        s->sr |= PL022_SR_TNF;
    }
    if (s->rx_fifo_len != 0) {
        s->sr |= PL022_SR_RNE;
    }
    if (s->rx_fifo_len == PL022_FIFO_DEPTH) {
        s->sr |= PL022_SR_RFF;
    }
    if (s->tx_fifo_len || s->busy) {
        s->sr |= PL022_SR_BSY;
    }

    /* The FIFO interrupts are not qualified by SSE. */
    s->is &= PL022_INT_ROR | PL022_INT_RT;
    if (s->rx_fifo_len >= PL022_FIFO_WATERMARK) {
        s->is |= PL022_INT_RX;
    }
    if (s->tx_fifo_len <= PL022_FIFO_WATERMARK) {
        s->is |= PL022_INT_TX;
    }
    qemu_set_irq(s->irq, (s->is & s->im) != 0);

    /* Requests drop while the port is disabled or the request disabled. */
    qemu_set_irq(s->dma_req[PL022_DMA_TX],
                 en && (s->dmacr & PL022_DMACR_TXDMAE) &&
                 s->tx_fifo_len < PL022_FIFO_DEPTH);
    qemu_set_irq(s->dma_req[PL022_DMA_RX],
                 en && (s->dmacr & PL022_DMACR_RXDMAE) &&
                 s->rx_fifo_len > 0);

    pl022_drive_pins(s);
}

/* Restart the receive timeout after receive FIFO activity. */
static void pl022_rt_restart(PL022State *s)
{
    if (s->rx_fifo_len && pl022_timed(s)) {
        timer_mod(s->rt_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  pl022_ns(s, PL022_RT_BITS));
    } else {
        timer_del(s->rt_timer);
    }
}

static void pl022_rt_expired(void *opaque)
{
    PL022State *s = opaque;

    if (s->rx_fifo_len) {
        s->is |= PL022_INT_RT;
        pl022_update(s);
    }
}

/*
 * A received frame enters the receive FIFO. A full FIFO overruns: the
 * frame is lost and ROR raised. New data ends a receive timeout.
 */
static void pl022_rx_push(PL022State *s, uint32_t val)
{
    if (s->rx_fifo_len == PL022_FIFO_DEPTH) {
        s->is |= PL022_INT_ROR;
    } else {
        s->rx_fifo[s->rx_fifo_head] = val & s->bitmask;
        s->rx_fifo_head = (s->rx_fifo_head + 1) % PL022_FIFO_DEPTH;
        s->rx_fifo_len++;
    }
    s->is &= ~PL022_INT_RT;
    pl022_rt_restart(s);
}

static uint16_t pl022_tx_pop(PL022State *s)
{
    int i = (s->tx_fifo_head - s->tx_fifo_len) & (PL022_FIFO_DEPTH - 1);

    s->tx_fifo_len--;
    return s->tx_fifo[i];
}

/*
 * The data exchange of one master frame over the SSI bus. Loopback mode
 * connects the transmit shifter to the receive shifter instead. A
 * Microwire frame sends the 8-bit control word while the receive line is
 * not sampled, then shifts in the slave's response; in loopback the
 * response phase sees the transmit line, which is low after the control
 * word.
 */
static uint32_t pl022_exchange(PL022State *s, uint32_t tx)
{
    bool lbm = s->cr1 & PL022_CR1_LBM;

    if (pl022_frf(s) == PL022_FRF_MICROWIRE) {
        if (lbm) {
            return 0;
        }
        ssi_transfer(s->ssi, tx & 0xff);
        return ssi_transfer(s->ssi, 0);
    }
    if (lbm) {
        return tx;
    }
    return ssi_transfer(s->ssi, tx & s->bitmask);
}

static void pl022_start_frame(PL022State *s)
{
    s->shift = pl022_tx_pop(s);
    s->busy = true;
    if (pl022_frf(s) == PL022_FRF_TI) {
        /* The one-period frame pulse ahead of the data. */
        pl022_set_pin(s, PL022_SSPFSSOUT, 1);
    }
    pl022_drive_pins(s);
}

static void pl022_end_frame(PL022State *s)
{
    pl022_rx_push(s, pl022_exchange(s, s->shift));
    s->busy = false;
    /*
     * In Motorola format with SPH=0 the slave select pulses high between
     * back-to-back frames; with SPH=1, and in Microwire, it stays low
     * until the transmit FIFO runs dry.
     */
    if (pl022_frf(s) == PL022_FRF_MOTOROLA && !(s->cr0 & PL022_CR0_SPH)) {
        pl022_drive_pins(s);
    }
}

/*
 * A master transmits while it is enabled and its transmit FIFO holds
 * data, the next frame starting at virtual time `start`. Without SSPCLK,
 * frames complete at once, and the transmitter stalls while the receive
 * FIFO is full rather than lose data.
 */
static void pl022_xfer_at(PL022State *s, int64_t start)
{
    bool timed = pl022_timed(s);

    while (pl022_enabled(s) && !pl022_slave(s) && !s->busy &&
           s->tx_fifo_len &&
           (timed || s->rx_fifo_len < PL022_FIFO_DEPTH)) {
        pl022_start_frame(s);
        if (timed) {
            s->frame_end = start + pl022_ns(s, pl022_frame_bits(s));
            timer_mod(s->frame_timer, s->frame_end);
            break;
        }
        pl022_end_frame(s);
    }
    pl022_update(s);
}

static void pl022_xfer(PL022State *s)
{
    pl022_xfer_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/*
 * Back-to-back frames follow on from the end of the last one, however
 * late its timer runs, so the line rate holds in virtual time. A timer
 * that runs late can leave several frames due at once; they complete
 * together only while the receive FIFO has room for them. Once it is
 * full the next frame starts no earlier than now, giving whatever drains
 * the FIFO (the CPU, the DMA) the frame's time to do so, as it would have
 * had on hardware, before the FIFO can overrun.
 */
static void pl022_frame_done(void *opaque)
{
    PL022State *s = opaque;
    int64_t start;

    if (!s->busy) {
        pl022_xfer(s);
        return;
    }
    pl022_end_frame(s);
    start = s->frame_end;
    if (s->rx_fifo_len == PL022_FIFO_DEPTH) {
        start = MAX(start, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    pl022_xfer_at(s, start);
}

/*
 * A frame clocked by an external master, for a PL022 in slave mode. The
 * slave takes part while enabled and selected by SSPFSSIN low; TI frames
 * are delimited by frame pulses rather than a select, so a TI slave takes
 * every frame. It shifts out the head of its transmit FIFO (zeros when
 * the FIFO is empty) unless SOD bars it from driving the line.
 */
static uint32_t pl022_slave_frame(PL022State *s, uint32_t rx)
{
    bool lbm = s->cr1 & PL022_CR1_LBM;
    uint32_t tx = 0;

    if (!pl022_enabled(s) || !pl022_slave(s) ||
        (pl022_frf(s) != PL022_FRF_TI && s->fssin)) {
        return 0;
    }
    if (s->tx_fifo_len) {
        tx = pl022_tx_pop(s) & s->bitmask;
    }
    pl022_rx_push(s, lbm ? tx : rx);
    pl022_update(s);
    return (s->cr1 & PL022_CR1_SOD) ? 0 : tx;
}

static void pl022_set_fssin(void *opaque, int n, int level)
{
    PL022State *s = opaque;

    s->fssin = level != 0;
    pl022_update(s);
}

static uint64_t pl022_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    PL022State *s = (PL022State *)opaque;
    int val;

    if (offset >= 0xfe0 && offset < 0x1000) {
        val = pl022_id[(offset - 0xfe0) >> 2];
        if ((offset - 0xfe0) >> 2 == PL022_ID_REVISION) {
            val |= (s->revision & 0xf) << 4;
        }
        return val;
    }
    switch (offset) {
    case 0x00: /* CR0 */
        return s->cr0;
    case 0x04: /* CR1 */
        return s->cr1;
    case 0x08: /* DR */
        if (s->rx_fifo_len) {
            val = s->rx_fifo[(s->rx_fifo_head - s->rx_fifo_len) &
                             (PL022_FIFO_DEPTH - 1)];
            s->rx_fifo_len--;
            if (!s->rx_fifo_len) {
                s->is &= ~PL022_INT_RT;
            }
            pl022_rt_restart(s);
            pl022_xfer(s);
        } else {
            val = 0;
        }
        return val;
    case 0x0c: /* SR */
        return s->sr;
    case 0x10: /* CPSR */
        return s->cpsr;
    case 0x14: /* IMSC */
        return s->im;
    case 0x18: /* RIS */
        return s->is;
    case 0x1c: /* MIS */
        return s->im & s->is;
    case 0x20: /* ICR */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl022_read: read of write-only SSPICR\n");
        return 0;
    case 0x24: /* DMACR */
        return s->dmacr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl022_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void pl022_write_cr1(PL022State *s, uint32_t value)
{
    value &= PL022_CR1_MASK;
    if (pl022_enabled(s) && ((value ^ s->cr1) & PL022_CR1_MS)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl022: MS can only change while SSE is clear\n");
        value = (value & ~PL022_CR1_MS) | (s->cr1 & PL022_CR1_MS);
    }
    if ((value & PL022_CR1_SSE) && !pl022_enabled(s) && s->cpsr < 2 &&
        pl022_timed(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl022: enabled with CPSDVSR %u; it must be at "
                      "least 2\n", s->cpsr);
    }
    s->cr1 = value;
    if (!pl022_enabled(s) && s->busy) {
        /* Disabling the port abandons the frame on the wire. */
        s->busy = false;
        timer_del(s->frame_timer);
    }
    pl022_xfer(s);
}

static void pl022_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    PL022State *s = (PL022State *)opaque;

    switch (offset) {
    case 0x00: /* CR0 */
        s->cr0 = value & PL022_CR0_MASK;
        if (PL022_CR0_DSS(s->cr0) < 3) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "pl022: reserved data size %u\n",
                          (unsigned)PL022_CR0_DSS(s->cr0));
        }
        if (pl022_frf(s) == 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl022: reserved frame format\n");
        }
        s->bitmask = (1 << pl022_data_bits(s)) - 1;
        pl022_update(s);
        break;
    case 0x04: /* CR1 */
        pl022_write_cr1(s, value);
        break;
    case 0x08: /* DR */
        /* Writes to a full transmit FIFO are lost. */
        if (s->tx_fifo_len < PL022_FIFO_DEPTH) {
            s->tx_fifo[s->tx_fifo_head] = value & 0xffff;
            s->tx_fifo_head = (s->tx_fifo_head + 1) % PL022_FIFO_DEPTH;
            s->tx_fifo_len++;
            pl022_xfer(s);
        }
        break;
    case 0x10: /* CPSR */
        s->cpsr = value & PL022_CPSR_MASK;
        break;
    case 0x14: /* IMSC */
        s->im = value & PL022_INT_MASK;
        pl022_update(s);
        break;
    case 0x20: /* ICR */
        /*
         * write-1-to-clear: bit 0 clears ROR, bit 1 clears RT;
         * RX and TX interrupts cannot be cleared this way.
         */
        value &= PL022_INT_ROR | PL022_INT_RT;
        s->is &= ~value;
        pl022_update(s);
        break;
    case 0x24: /* DMACR */
        s->dmacr = value & PL022_DMACR_MASK;
        if (s->dmacr && !s->dma_req[PL022_DMA_TX] &&
            !s->dma_req[PL022_DMA_RX]) {
            qemu_log_mask(LOG_UNIMP, "pl022: DMA not connected\n");
        }
        pl022_update(s);
        break;
    case 0x0c: /* SR */
    case 0x18: /* RIS */
    case 0x1c: /* MIS */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl022_write: write to read-only offset %x\n",
                      (int)offset);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl022_write: Bad offset %x\n", (int)offset);
    }
}

static void pl022_reset_hold(Object *obj, ResetType type)
{
    PL022State *s = PL022(obj);
    int i;

    timer_del(s->frame_timer);
    timer_del(s->rt_timer);
    s->cr0 = 0;
    s->cr1 = 0;
    s->bitmask = (1 << pl022_data_bits(s)) - 1;
    s->cpsr = 0;
    s->dmacr = 0;
    s->rx_fifo_len = 0;
    s->tx_fifo_len = 0;
    s->rx_fifo_head = 0;
    s->tx_fifo_head = 0;
    s->busy = false;
    s->shift = 0;
    s->im = 0;
    s->is = PL022_INT_TX;
    s->sr = PL022_SR_TFE | PL022_SR_TNF;
    for (i = 0; i < PL022_SSP_OUTS; i++) {
        s->ssp_level[i] = -1;
    }
}

static void pl022_reset_exit(Object *obj, ResetType type)
{
    pl022_update(PL022(obj));
}

static const MemoryRegionOps pl022_ops = {
    .read = pl022_read,
    .write = pl022_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int pl022_post_load(void *opaque, int version_id)
{
    PL022State *s = opaque;
    int i;

    if (s->tx_fifo_head < 0 ||
        s->tx_fifo_head >= ARRAY_SIZE(s->tx_fifo) ||
        s->rx_fifo_head < 0 ||
        s->rx_fifo_head >= ARRAY_SIZE(s->rx_fifo) ||
        s->tx_fifo_len < 0 || s->tx_fifo_len > PL022_FIFO_DEPTH ||
        s->rx_fifo_len < 0 || s->rx_fifo_len > PL022_FIFO_DEPTH) {
        return -1;
    }
    for (i = 0; i < PL022_SSP_OUTS; i++) {
        s->ssp_level[i] = -1;
    }
    return 0;
}

static const VMStateDescription vmstate_pl022 = {
    .name = "pl022_ssp",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = pl022_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr0, PL022State),
        VMSTATE_UINT32(cr1, PL022State),
        VMSTATE_UINT32(bitmask, PL022State),
        VMSTATE_UINT32(sr, PL022State),
        VMSTATE_UINT32(cpsr, PL022State),
        VMSTATE_UINT32(is, PL022State),
        VMSTATE_UINT32(im, PL022State),
        VMSTATE_INT32(tx_fifo_head, PL022State),
        VMSTATE_INT32(rx_fifo_head, PL022State),
        VMSTATE_INT32(tx_fifo_len, PL022State),
        VMSTATE_INT32(rx_fifo_len, PL022State),
        VMSTATE_UINT16(tx_fifo[0], PL022State),
        VMSTATE_UINT16(rx_fifo[0], PL022State),
        VMSTATE_UINT16(tx_fifo[1], PL022State),
        VMSTATE_UINT16(rx_fifo[1], PL022State),
        VMSTATE_UINT16(tx_fifo[2], PL022State),
        VMSTATE_UINT16(rx_fifo[2], PL022State),
        VMSTATE_UINT16(tx_fifo[3], PL022State),
        VMSTATE_UINT16(rx_fifo[3], PL022State),
        VMSTATE_UINT16(tx_fifo[4], PL022State),
        VMSTATE_UINT16(rx_fifo[4], PL022State),
        VMSTATE_UINT16(tx_fifo[5], PL022State),
        VMSTATE_UINT16(rx_fifo[5], PL022State),
        VMSTATE_UINT16(tx_fifo[6], PL022State),
        VMSTATE_UINT16(rx_fifo[6], PL022State),
        VMSTATE_UINT16(tx_fifo[7], PL022State),
        VMSTATE_UINT16(rx_fifo[7], PL022State),
        VMSTATE_UINT32_V(dmacr, PL022State, 2),
        VMSTATE_BOOL_V(busy, PL022State, 2),
        VMSTATE_UINT16_V(shift, PL022State, 2),
        VMSTATE_BOOL_V(fssin, PL022State, 2),
        VMSTATE_INT64_V(frame_end, PL022State, 2),
        VMSTATE_TIMER_PTR_V(frame_timer, PL022State, 2),
        VMSTATE_TIMER_PTR_V(rt_timer, PL022State, 2),
        VMSTATE_CLOCK_V(clk, PL022State, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void pl022_init(Object *obj)
{
    PL022State *s = PL022(obj);
    DeviceState *dev = DEVICE(obj);

    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pl022_frame_done, s);
    s->rt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pl022_rt_expired, s);
    qdev_init_gpio_out_named(dev, s->dma_req, PL022_DMA_REQ,
                             ARRAY_SIZE(s->dma_req));
    qdev_init_gpio_out_named(dev, s->ssp_out, PL022_SSP_OUT,
                             ARRAY_SIZE(s->ssp_out));
    qdev_init_gpio_in_named(dev, pl022_set_fssin, PL022_SSPFSSIN, 1);
}

static void pl022_finalize(Object *obj)
{
    PL022State *s = PL022(obj);

    timer_free(s->frame_timer);
    timer_free(s->rt_timer);
}

static void pl022_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    PL022State *s = PL022(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &pl022_ops, s, "pl022", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->ssi = ssi_create_bus(dev, s->bus_name ? s->bus_name : "ssi");
}

static const Property pl022_properties[] = {
    /* The name of the SSI bus, for boards with several PL022s. */
    DEFINE_PROP_STRING("bus-name", PL022State, bus_name),
    /* The revision SSPPeriphID2 reports: 3 for r1p4. */
    DEFINE_PROP_UINT8("revision", PL022State, revision, 0),
};

static void pl022_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = pl022_reset_hold;
    rc->phases.exit = pl022_reset_exit;
    dc->vmsd = &vmstate_pl022;
    dc->realize = pl022_realize;
    device_class_set_props(dc, pl022_properties);
}

/*
 * The slave side of a PL022 as a device on another master's SSI bus.
 * Selection is the PL022's own SSPFSSIN, so the device has no SSI chip
 * select of its own.
 */
static uint32_t pl022_target_transfer_raw(SSIPeripheral *dev, uint32_t val)
{
    PL022TargetState *t = PL022_TARGET(dev);

    return pl022_slave_frame(t->controller, val);
}

static void pl022_target_realize(SSIPeripheral *dev, Error **errp)
{
    PL022TargetState *t = PL022_TARGET(dev);

    if (!t->controller) {
        error_setg(errp, "pl022-target: 'controller' link not set");
    }
}

static const Property pl022_target_properties[] = {
    DEFINE_PROP_LINK("controller", PL022TargetState, controller, TYPE_PL022,
                     PL022State *),
};

static void pl022_target_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = pl022_target_realize;
    k->transfer_raw = pl022_target_transfer_raw;
    k->cs_polarity = SSI_CS_NONE;
    dc->desc = "PL022 in slave mode, as an SSI peripheral";
    device_class_set_props(dc, pl022_target_properties);
}

static const TypeInfo pl022_types[] = {
    {
        .name          = TYPE_PL022,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(PL022State),
        .instance_init = pl022_init,
        .instance_finalize = pl022_finalize,
        .class_init    = pl022_class_init,
    },
    {
        .name          = TYPE_PL022_TARGET,
        .parent        = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(PL022TargetState),
        .class_init    = pl022_target_class_init,
    },
};

DEFINE_TYPES(pl022_types)
