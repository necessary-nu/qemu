/*
 * Synopsys DWC2 OTG controller: host mode
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The root port is the port of a QEMU USB bus, so QEMU's USB devices
 * (-device usb-kbd, usb-storage, usb-hub, ...) plug into it. With the port
 * powered (HPRT0.PrtPwr) a device's attach sets PrtConnDet; a port reset
 * (PrtRst) resets the device and, as it ends, enables the port at the
 * speed both sides support (high speed through the UTMI+ PHY unless
 * HCFG.FSLSSupp or the full-speed PHY is selected), shown in PrtSpd. The
 * enabled port sends a (micro)frame's SOF every HFIR.FrInt PHY clocks,
 * counted in HFNUM, until it is suspended (PrtSusp) or disabled.
 *
 * Channels run one transaction at a time on the bus, each taking its
 * packet's time at the port's bit rate: in each (micro)frame the periodic
 * channels due in it first (HCCHAR.OddFrm, or the frame list and
 * SCHED_INFO in scatter/gather mode), then the non-periodic ones in turn.
 * The transfer moves through the channel registers as the databook
 * describes, in the mode the core is in:
 *
 *   - Slave mode: transactions follow the request queues. Enabling an IN
 *     channel queues an IN request; the last word of an OUT packet pushed
 *     into the non-periodic or periodic transmit FIFO queues it; a halt
 *     (HCCHAR.ChDis) queues a halt request. IN data goes to the receive
 *     FIFO with its status entry; the IN transfer completed, data toggle
 *     error and channel halted entries raise their interrupt as software
 *     pops them. A NAK consumes the request; an IN channel stays enabled
 *     until software halts it, an OUT channel disables itself when its
 *     transfer completes.
 *   - Buffer DMA: the channel moves its transfer between memory at HCDMA
 *     and the bus, retrying NAKed control and bulk transactions, retrying
 *     a transaction error up to three times, and halts (ChHltd) at the
 *     end of the transfer or on a STALL, babble, toggle error, third
 *     error or a NAKed periodic transaction.
 *   - Scatter/gather DMA (HCFG.DescDMA): the channel works through its
 *     list of transfer descriptors (qTDs) at HCDMA, writing each back with
 *     its remaining bytes and status, raising XferCompl at an IOC
 *     descriptor or a short IN packet, halting at the end-of-list one,
 *     raising BNA at an inactive one and DescLstRoll as it wraps.
 *     Periodic channels run in the frames the periodic frame list at
 *     HFLBADDR gives them while HCFG.PerSchedEna is set; an isochronous
 *     channel's list is indexed by the (micro)frame number, one
 *     descriptor a (micro)frame, an inactive one skipping it, and runs
 *     until a descriptor with EOL or software halts it.
 *
 * QEMU's devices have no data toggles of their own, so the controller
 * keeps each device endpoint's toggle as the device would: an OUT packet
 * with the wrong PID is acknowledged and dropped, and IN data carries the
 * device's PID, a mismatch with HCTSIZ.PID being a toggle error.
 *
 * Where hardware retries a NAKed transaction at once, this model retries
 * it in the next (micro)frame, which the device cannot tell apart and
 * spares the host's CPU. Split transactions need a high-speed hub's
 * transaction translator, which QEMU's hubs do not have: they get no
 * reply (logged as unimplemented). An isochronous transaction is one
 * packet a (micro)frame: high-bandwidth endpoints' several (HCCHAR.MC) are
 * not modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "dwc2-otg-internal.h"

/* Bits on the wire around a packet's data: token, sync, PID, CRC, EOP. */
#define XACT_OVERHEAD_BYTES     13
#define HS_XACT_OVERHEAD_BYTES  40
#define FS_FRAME_NS             SCALE_MS
#define HS_FRAME_NS             (125 * SCALE_US)

#define QTD_NBYTES_MASK         0x1ffff
#define QTD_ISOC                0
#define QTD_STS_SUCCESS         0
#define QTD_STS_PKTERR          1

enum {
    XACT_ACK,
    XACT_NAK,
    XACT_STALL,
    XACT_ERR,
    XACT_BABBLE,
    XACT_TOGGLE_ERR,
};

static USBDevice *host_root(DWC2OTGState *s)
{
    USBDevice *dev = s->port.dev;

    return dev && dev->attached ? dev : NULL;
}

static bool host_port_powered(DWC2OTGState *s)
{
    return s->host_mode && (s->hprt & HPRT0_PWR) && s->phy_connected &&
           dwc2_otg_phy_period(s);
}

static bool host_running(DWC2OTGState *s)
{
    return s->port_connected && (s->hprt & HPRT0_ENA) &&
           !(s->hprt & (HPRT0_SUSP | HPRT0_RST)) && dwc2_otg_phy_period(s);
}

static bool chan_is_in(DWC2OTGHostChan *c)
{
    return c->hcchar & HCCHAR_EPDIR;
}

static int chan_type(DWC2OTGHostChan *c)
{
    return (c->hcchar & HCCHAR_EPTYPE_MASK) >> HCCHAR_EPTYPE_SHIFT;
}

static bool chan_periodic(DWC2OTGHostChan *c)
{
    int t = chan_type(c);

    return t == USB_ENDPOINT_XFER_INT || t == USB_ENDPOINT_XFER_ISOC;
}

static uint32_t chan_mps(DWC2OTGHostChan *c)
{
    return c->hcchar & HCCHAR_MPS_MASK;
}

static uint32_t chan_xfersize(DWC2OTGHostChan *c)
{
    return c->hctsiz & TSIZ_XFERSIZE_MASK;
}

static uint32_t chan_pktcnt(DWC2OTGHostChan *c)
{
    return (c->hctsiz & TSIZ_PKTCNT_MASK) >> TSIZ_PKTCNT_SHIFT;
}

static void chan_set_xfer(DWC2OTGHostChan *c, uint32_t size, uint32_t pkts)
{
    c->hctsiz = (c->hctsiz & ~(TSIZ_XFERSIZE_MASK | TSIZ_PKTCNT_MASK)) |
                (size & TSIZ_XFERSIZE_MASK) |
                ((pkts << TSIZ_PKTCNT_SHIFT) & TSIZ_PKTCNT_MASK);
}

static int chan_pid(DWC2OTGHostChan *c)
{
    return (c->hctsiz & TSIZ_SC_MC_PID_MASK) >> TSIZ_SC_MC_PID_SHIFT;
}

static void chan_set_pid(DWC2OTGHostChan *c, int pid)
{
    c->hctsiz = (c->hctsiz & ~TSIZ_SC_MC_PID_MASK) |
                (pid << TSIZ_SC_MC_PID_SHIFT);
}

/* The next PID after a successful packet: DATA0 and DATA1 alternate. */
static void chan_toggle(DWC2OTGHostChan *c)
{
    int pid = chan_pid(c);

    if (pid == TSIZ_SC_MC_PID_SETUP || pid == TSIZ_SC_MC_PID_DATA0) {
        chan_set_pid(c, TSIZ_SC_MC_PID_DATA1);
    } else if (pid == TSIZ_SC_MC_PID_DATA1) {
        chan_set_pid(c, TSIZ_SC_MC_PID_DATA0);
    }
}

static bool host_ddma(DWC2OTGState *s)
{
    return dwc2_otg_dma_enabled(s) && (s->hcfg & HCFG_DESCDMA);
}

uint32_t dwc2_otg_haint(DWC2OTGState *s)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < s->num_chans; i++) {
        if (s->hc[i].hcint & s->hc[i].hcintmsk) {
            v |= BIT(i);
        }
    }
    return v;
}

/* Bus timing */

/* The bus's bit rate below high speed. */
static uint32_t host_bit_rate(DWC2OTGState *s)
{
    return s->port_speed == DWC2_SPEED_LOW ? 1500000 : 12000000;
}

/*
 * A transaction of @len bytes on the bus, at the first nanosecond its
 * bits have taken.
 */
static int64_t host_xact_ns(DWC2OTGState *s, uint32_t len)
{
    if (s->port_speed == DWC2_SPEED_HIGH) {
        /* 480 Mb/s: 60 bytes a microsecond. */
        return DIV_ROUND_UP((int64_t)(len + HS_XACT_OVERHEAD_BYTES) *
                            SCALE_US, 60);
    }
    return muldiv64_round_up((uint64_t)(len + XACT_OVERHEAD_BYTES) * 8,
                             NANOSECONDS_PER_SECOND, host_bit_rate(s));
}

/* One (micro)frame: HFIR.FrInt PHY clocks. */
static int64_t host_frame_len_ns(DWC2OTGState *s)
{
    int64_t ns = dwc2_otg_phy_ticks_ns(s, s->hfir & HFIR_FRINT_MASK);

    if (ns <= 0) {
        ns = s->port_speed == DWC2_SPEED_HIGH ? HS_FRAME_NS : FS_FRAME_NS;
    }
    return ns;
}

static void host_bus_kick(DWC2OTGState *s, int64_t delay)
{
    int64_t when = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay;

    if (s->xact_busy || !host_running(s)) {
        return;
    }
    if (!timer_pending(s->bus_timer) ||
        timer_expire_time_ns(s->bus_timer) > when) {
        timer_mod(s->bus_timer, when);
    }
}

void dwc2_otg_host_kick(DWC2OTGState *s)
{
    host_bus_kick(s, 0);
}

static void host_frames_update(DWC2OTGState *s)
{
    if (host_running(s)) {
        if (!timer_pending(s->frame_timer)) {
            /* The first SOF goes out as the port starts running. */
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

            s->frame_ns = host_frame_len_ns(s);
            s->frame_start_ns = now - s->frame_ns;
            s->frame_origin_ns = now;
            s->frame_ticks = 0;
            s->frame_period = dwc2_otg_phy_period(s);
            timer_mod(s->frame_timer, now);
        }
    } else {
        timer_del(s->frame_timer);
        timer_del(s->bus_timer);
    }
}

/*
 * HFNUM: the frame number, and the PHY clocks left in the frame, of those
 * counted from the frames' origin.
 */
uint32_t dwc2_otg_hfnum_read(DWC2OTGState *s)
{
    uint64_t left = 0, done;

    if (timer_pending(s->frame_timer) && s->frame_period &&
        s->frame_period == dwc2_otg_phy_period(s)) {
        done = dwc2_otg_phy_ticks(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                     s->frame_origin_ns);
        left = s->frame_ticks > done ? s->frame_ticks - done : 0;
    }
    return s->frnum | (MIN(left, 0xffff) << HFNUM_FRREM_SHIFT);
}

void dwc2_otg_host_frame_reset(DWC2OTGState *s)
{
    s->frnum = HFNUM_MAX_FRNUM;
}

/* Port */

static uint8_t host_detect_speed(DWC2OTGState *s, USBDevice *dev)
{
    if (!(dev->speedmask & (USB_SPEED_MASK_FULL | USB_SPEED_MASK_HIGH))) {
        return DWC2_SPEED_LOW;
    }
    return DWC2_SPEED_FULL;
}

/* The speed a port reset settles on: the fastest both sides support. */
static uint8_t host_reset_speed(DWC2OTGState *s, USBDevice *dev)
{
    s->port.speedmask = USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL;
    if (s->has_utmi && !(s->gusbcfg & GUSBCFG_PHYSEL) &&
        !(s->hcfg & HCFG_FSLSSUPP)) {
        s->port.speedmask |= USB_SPEED_MASK_HIGH;
    }
    usb_pick_speed(&s->port);
    switch (dev->speed) {
    case USB_SPEED_HIGH:
        return DWC2_SPEED_HIGH;
    case USB_SPEED_LOW:
        return DWC2_SPEED_LOW;
    default:
        return DWC2_SPEED_FULL;
    }
}

static void host_cancel(DWC2OTGState *s)
{
    if (s->xact_async) {
        usb_cancel_packet(&s->packet);
        s->xact_async = false;
    }
    s->xact_busy = false;
    timer_del(s->bus_timer);
}

/*
 * A device attached to (or detached from) a powered port: PrtConnDet on
 * attach; on detach GINTSTS.DisconnInt, and the port disabled.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void host_update_port_dev(DWC2OTGState *s, USBDevice *dev)
{
    bool connected = host_port_powered(s) && dev;

    if (connected && !s->port_connected) {
        s->port_connected = true;
        s->port_speed = host_detect_speed(s, dev);
        s->hprt |= HPRT0_CONNDET;
        if (s->gusbcfg & (GUSBCFG_HNPCAP | GUSBCFG_SRPCAP)) {
            s->gotgint |= GOTGINT_DBNCE_DONE;
        }
    } else if (!connected && s->port_connected) {
        s->port_connected = false;
        if (s->hprt & HPRT0_ENA) {
            s->hprt &= ~HPRT0_ENA;
            s->hprt |= HPRT0_ENACHG;
        }
        if (s->host_mode) {
            s->gintsts |= GINTSTS_DISCONNINT;
        }
        host_cancel(s);
    }
    host_frames_update(s);
    dwc2_otg_update_irq(s);
}

void dwc2_otg_host_update_port(DWC2OTGState *s)
{
    host_update_port_dev(s, host_root(s));
}

void dwc2_otg_host_phy_changed(DWC2OTGState *s)
{
    dwc2_otg_host_update_port(s);
    host_bus_kick(s, 0);
}

uint32_t dwc2_otg_hprt_read(DWC2OTGState *s)
{
    uint32_t v = s->hprt & ~(HPRT0_CONNSTS | HPRT0_LNSTS_MASK |
                             HPRT0_SPD_MASK);

    if (s->port_connected) {
        v |= HPRT0_CONNSTS;
        /* Line state: J (D+ high at full speed, D- at low), SE0 at high. */
        if (s->port_speed == DWC2_SPEED_LOW) {
            v |= 2 << HPRT0_LNSTS_SHIFT;
        } else if (s->port_speed == DWC2_SPEED_FULL) {
            v |= 1 << HPRT0_LNSTS_SHIFT;
        }
        v |= s->port_speed << HPRT0_SPD_SHIFT;
    }
    return v;
}

static void host_reset_toggles(DWC2OTGState *s)
{
    memset(s->dev_toggle, 0, sizeof(s->dev_toggle));
    s->ctrl_stall = false;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_hprt_write(DWC2OTGState *s, uint32_t val)
{
    uint32_t old = s->hprt;
    USBDevice *dev = host_root(s);

    s->hprt &= ~(val & (HPRT0_CONNDET | HPRT0_ENACHG | HPRT0_OVRCURRCHG));
    if ((val & HPRT0_ENA) && (s->hprt & HPRT0_ENA)) {
        /* Software disables the port: PrtEnChng flags any change. */
        s->hprt &= ~HPRT0_ENA;
        s->hprt |= HPRT0_ENACHG;
        host_cancel(s);
    }
    s->hprt = (s->hprt & ~(HPRT0_PWR | HPRT0_TSTCTL_MASK)) |
              (val & (HPRT0_PWR | HPRT0_TSTCTL_MASK));
    if (val & HPRT0_TSTCTL_MASK) {
        qemu_log_mask(LOG_UNIMP, "%s: host port test modes are not "
                      "modelled\n", s->name);
    }
    if ((val & HPRT0_RST) && !(old & HPRT0_RST)) {
        /* The reset disables the port and resets the device. */
        s->hprt |= HPRT0_RST;
        s->hprt &= ~(HPRT0_ENA | HPRT0_SUSP);
        host_cancel(s);
        host_reset_toggles(s);
        if (dev && s->port_connected) {
            usb_device_reset(dev);
        }
    } else if (!(val & HPRT0_RST) && (old & HPRT0_RST)) {
        s->hprt &= ~HPRT0_RST;
        if (s->port_connected && dev) {
            s->port_speed = host_reset_speed(s, dev);
            s->hprt |= HPRT0_ENA | HPRT0_ENACHG;
            s->frnum = HFNUM_MAX_FRNUM;
            if (!s->hfir_written) {
                /*
                 * Unprogrammed, the (micro)frame interval follows the PHY
                 * clock: 125 us at high speed, 1 ms otherwise.
                 */
                uint64_t ticks = dwc2_otg_phy_ticks(s,
                    s->port_speed == DWC2_SPEED_HIGH ? HS_FRAME_NS
                                                     : FS_FRAME_NS);

                s->hfir = (s->hfir & ~HFIR_FRINT_MASK) |
                          MIN(ticks, HFIR_FRINT_MASK);
            }
        }
    }
    if ((val & HPRT0_SUSP) && (s->hprt & HPRT0_ENA)) {
        s->hprt |= HPRT0_SUSP;
        host_cancel(s);
    }
    if (val & HPRT0_RES) {
        s->hprt |= HPRT0_RES;
    } else if (old & HPRT0_RES) {
        /* The end of resume signalling ends the suspend. */
        s->hprt &= ~(HPRT0_RES | HPRT0_SUSP);
    }
    qemu_set_irq(s->suspended_out, !!(s->hprt & HPRT0_SUSP));
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_DRVVBUS],
                 s->host_mode && (s->hprt & HPRT0_PWR));
    dwc2_otg_host_update_port(s);
}

/* Request queues (slave mode) */

static DWC2OTGReqQueue *chan_queue(DWC2OTGState *s, DWC2OTGHostChan *c)
{
    return &s->reqq[chan_periodic(c) ? 1 : 0];
}

static uint8_t reqq_depth(DWC2OTGState *s, int q)
{
    return q ? s->ptxq_depth : s->nptxq_depth;
}

static bool reqq_push(DWC2OTGState *s, int ch, int type)
{
    DWC2OTGReqQueue *q = chan_queue(s, &s->hc[ch]);
    int qi = q == &s->reqq[1];

    if (q->n >= reqq_depth(s, qi)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: request for channel %d with "
                      "the %s request queue full\n", s->name, ch,
                      qi ? "periodic" : "non-periodic");
        return false;
    }
    q->chan[q->n] = ch;
    q->type[q->n] = type;
    q->n++;
    return true;
}

static void reqq_pop(DWC2OTGReqQueue *q, int i)
{
    q->n--;
    memmove(q->chan + i, q->chan + i + 1, q->n - i);
    memmove(q->type + i, q->type + i + 1, q->n - i);
}

/* Drop a channel's queued requests, and its packets at the FIFO's head. */
static void reqq_flush_chan(DWC2OTGState *s, int ch)
{
    int qi, i, fifo = chan_periodic(&s->hc[ch]) ? DWC2_PTXF : DWC2_NPTXF;

    for (qi = 0; qi < 2; qi++) {
        DWC2OTGReqQueue *q = &s->reqq[qi];

        for (i = 0; i < q->n;) {
            if (q->chan[i] == ch) {
                reqq_pop(q, i);
            } else {
                i++;
            }
        }
    }
    while (s->tx[fifo].npkts && s->tx[fifo].pkt_chan[0] == ch) {
        dwc2_otg_tx_pop(s, fifo, NULL);
    }
    s->hc[ch].in_reqs = 0;
    s->hc[ch].pushed = 0;
}

uint32_t dwc2_otg_txsts(DWC2OTGState *s, int fifo)
{
    DWC2OTGReqQueue *q = &s->reqq[fifo];
    uint32_t space = MIN(dwc2_otg_tx_free(s, fifo), 0xffff);
    uint32_t qspace = reqq_depth(s, fifo) - MIN(q->n, reqq_depth(s, fifo));
    uint32_t top = 0;

    if (!s->host_mode) {
        return space | (qspace << 16);
    }
    if (q->n) {
        top = (q->chan[0] << 3) | (q->type[0] << 1);
    }
    if (fifo == DWC2_NPTXF) {
        return space | (MIN(qspace, 0xff) << 16) | ((top & 0x7f) << 24);
    }
    return space | (MIN(qspace, 0xff) << 16) | ((top & 0xff) << 24);
}

/*
 * The application pushed a word for OUT channel @ch: the word that
 * completes a packet (MIN(XferSize left, MPS) bytes) queues the packet's
 * request, and XferSize counts down by what the FIFO took.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_host_tx_push(DWC2OTGState *s, int ch, uint32_t word)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    int fifo = chan_periodic(c) ? DWC2_PTXF : DWC2_NPTXF;
    uint32_t len = MIN(chan_xfersize(c), chan_mps(c));

    if (dwc2_otg_dma_enabled(s) || chan_is_in(c) || !len) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: push to the FIFO of channel %d, "
                      "which has no OUT data to send\n", s->name, ch);
        return;
    }
    if (dwc2_otg_tx_push_packet(s, fifo, ch, &c->pushed, len, word)) {
        chan_set_xfer(c, chan_xfersize(c) - len, chan_pktcnt(c));
        reqq_push(s, ch, DWC2_REQ_INOUT);
        host_bus_kick(s, 0);
    }
}

/* Channel ends */

/* [spec:nuos:req:emu.esp32p4.usb] */
static void chan_disable(DWC2OTGState *s, int ch, uint32_t intr)
{
    DWC2OTGHostChan *c = &s->hc[ch];

    c->hcchar &= ~(HCCHAR_CHENA | HCCHAR_CHDIS);
    c->hcint |= intr;
    c->halt_req = false;
    c->qtd_open = false;
    c->nak_wait = false;
    dwc2_otg_update_irq(s);
}

/*
 * The core halted a channel: ChHltd with the reason. In slave mode an IN
 * channel's halt goes through the receive FIFO, and ChHltd follows when
 * software pops its status.
 */
static void chan_halt(DWC2OTGState *s, int ch, uint32_t intr)
{
    DWC2OTGHostChan *c = &s->hc[ch];

    if (!dwc2_otg_dma_enabled(s) && chan_is_in(c)) {
        reqq_flush_chan(s, ch);
        c->hcchar &= ~(HCCHAR_CHENA | HCCHAR_CHDIS);
        c->hcint |= intr;
        c->halt_req = false;
        if (dwc2_otg_rx_free(s) >= 1) {
            dwc2_otg_rx_push(s, (ch << GRXSTS_HCHNUM_SHIFT) |
                             (GRXSTS_PKTSTS_HCHHALTED << GRXSTS_PKTSTS_SHIFT),
                             NULL, 0);
        } else {
            c->hcint |= HCINTMSK_CHHLTD;
        }
        dwc2_otg_update_irq(s);
        return;
    }
    if (!dwc2_otg_dma_enabled(s)) {
        reqq_flush_chan(s, ch);
    }
    chan_disable(s, ch, intr | HCINTMSK_CHHLTD);
}

void dwc2_otg_host_rx_popped(DWC2OTGState *s, uint32_t status)
{
    int ch = (status & GRXSTS_HCHNUM_MASK) >> GRXSTS_HCHNUM_SHIFT;
    int pktsts = (status & GRXSTS_PKTSTS_MASK) >> GRXSTS_PKTSTS_SHIFT;

    if (ch >= s->num_chans) {
        return;
    }
    switch (pktsts) {
    case GRXSTS_PKTSTS_HCHIN_XFER_COMP:
        s->hc[ch].hcint |= HCINTMSK_XFERCOMPL;
        break;
    case GRXSTS_PKTSTS_DATATOGGLEERR:
        s->hc[ch].hcint |= HCINTMSK_DATATGLERR;
        break;
    case GRXSTS_PKTSTS_HCHHALTED:
        s->hc[ch].hcint |= HCINTMSK_CHHLTD;
        break;
    default:
        return;
    }
    host_bus_kick(s, 0);
}

/*
 * HCCHAR: setting ChEna starts the channel (in slave mode, each write of
 * it on an IN channel queues an IN request); setting ChDis with it asks
 * the channel to halt.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_hcchar_write(DWC2OTGState *s, int ch, uint32_t val)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    bool was = c->hcchar & HCCHAR_CHENA;
    bool dma = dwc2_otg_dma_enabled(s);

    c->hcchar = (c->hcchar & (HCCHAR_CHENA | HCCHAR_CHDIS)) |
                (val & ~(HCCHAR_CHENA | HCCHAR_CHDIS));
    if (val & HCCHAR_CHDIS) {
        if (!was) {
            /* Nothing to halt. */
            return;
        }
        c->hcchar |= HCCHAR_CHDIS;
        c->halt_req = true;
        if (!dma) {
            /* The halt request takes its turn in the request queue. */
            reqq_push(s, ch, DWC2_REQ_HALT);
            host_bus_kick(s, 0);
        } else if (!s->xact_busy || s->xact_chan != ch) {
            /* Otherwise it halts once its transaction ends. */
            chan_halt(s, ch, 0);
        }
        return;
    }
    if (!(val & HCCHAR_CHENA)) {
        return;
    }
    c->hcchar |= HCCHAR_CHENA;
    if (!was) {
        c->err_cnt = 0;
        c->frame_done = false;
        c->nak_wait = false;
        c->halt_req = false;
        c->qtd_open = false;
        c->pushed = 0;
        if (c->hcsplt & HCSPLT_SPLTENA) {
            qemu_log_mask(LOG_UNIMP, "%s: split transactions need a "
                          "high-speed hub's transaction translator, which "
                          "QEMU's hubs lack: channel %d gets no reply\n",
                          s->name, ch);
        }
    }
    if (!dma) {
        if (chan_is_in(c)) {
            if (reqq_push(s, ch, DWC2_REQ_INOUT)) {
                c->in_reqs++;
            }
        } else if (!was && !chan_xfersize(c)) {
            /* A zero-length OUT needs no data in the FIFO. */
            reqq_push(s, ch, DWC2_REQ_ZLP);
        }
    }
    host_bus_kick(s, 0);
}

/* Transactions */

/* Which token this channel's next transaction sends. */
static int chan_token(DWC2OTGHostChan *c, bool setup_qtd)
{
    if (chan_type(c) == USB_ENDPOINT_XFER_CONTROL &&
        (chan_pid(c) == TSIZ_SC_MC_PID_SETUP || setup_qtd)) {
        return DWC2_TOKEN_SETUP;
    }
    return chan_is_in(c) ? DWC2_TOKEN_IN : DWC2_TOKEN_OUT;
}

/*
 * The device endpoint toggle state that standard requests reset: all of a
 * device's non-control endpoints on SET_CONFIGURATION and SET_INTERFACE,
 * one endpoint on CLEAR_FEATURE(ENDPOINT_HALT), all of the new address's
 * on SET_ADDRESS.
 */
static void host_setup_toggles(DWC2OTGState *s, uint8_t addr,
                               const uint8_t *setup)
{
    uint8_t type = setup[0], req = setup[1];
    uint16_t value = setup[2] | (setup[3] << 8);
    uint16_t index = setup[4] | (setup[5] << 8);

    /* Both directions of EP0 continue with DATA1 after a SETUP. */
    s->dev_toggle[addr] |= 3u;
    if ((type & 0x60) != USB_TYPE_STANDARD) {
        return;
    }
    if ((type == (USB_DIR_OUT | USB_RECIP_DEVICE) &&
         req == USB_REQ_SET_CONFIGURATION) ||
        (type == (USB_DIR_OUT | USB_RECIP_INTERFACE) &&
         req == USB_REQ_SET_INTERFACE)) {
        s->dev_toggle[addr] &= 3u;
    } else if (type == (USB_DIR_OUT | USB_RECIP_ENDPOINT) &&
               req == USB_REQ_CLEAR_FEATURE && value == 0) {
        int ep = index & 0xf;

        if (ep) {
            s->dev_toggle[addr] &= ~(1u << (ep * 2 +
                                            !(index & USB_DIR_IN)));
        }
    } else if (type == (USB_DIR_OUT | USB_RECIP_DEVICE) &&
               req == USB_REQ_SET_ADDRESS && value < 128 && value != addr) {
        s->dev_toggle[value] = 0;
    }
}

static int host_status_result(int status)
{
    switch (status) {
    case USB_RET_SUCCESS:
        return XACT_ACK;
    case USB_RET_NAK:
        return XACT_NAK;
    case USB_RET_STALL:
        return XACT_STALL;
    case USB_RET_BABBLE:
        return XACT_BABBLE;
    default:
        return XACT_ERR;
    }
}

static void host_xact_end(DWC2OTGState *s, int result, uint32_t actual,
                          int data_pid);

/*
 * Send one transaction of channel @ch to the addressed device: a USB
 * packet to a QEMU device, which answers it now or later (async).
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void host_issue(DWC2OTGState *s, int ch, int token,
                       const uint8_t *data, uint32_t len)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint8_t addr = (c->hcchar & HCCHAR_DEVADDR_MASK) >> HCCHAR_DEVADDR_SHIFT;
    uint8_t ep = (c->hcchar & HCCHAR_EPNUM_MASK) >> HCCHAR_EPNUM_SHIFT;
    int pid = token == DWC2_TOKEN_SETUP ? USB_TOKEN_SETUP :
              token == DWC2_TOKEN_IN ? USB_TOKEN_IN : USB_TOKEN_OUT;
    USBDevice *dev = NULL;
    USBEndpoint *uep;
    uint32_t tbit = 1u << (ep * 2 + (token != DWC2_TOKEN_IN));

    s->xact_busy = true;
    s->xact_chan = ch;
    s->xact_token = token;
    s->xact_len = len;
    if (token != DWC2_TOKEN_IN && len) {
        memcpy(s->xfer_buf, data, len);
    }
    if (c->hcsplt & HCSPLT_SPLTENA) {
        host_xact_end(s, XACT_ERR, 0, 0);
        return;
    }
    if (s->port_connected) {
        dev = usb_find_device(&s->port, addr);
    }
    if (!dev) {
        host_xact_end(s, XACT_ERR, 0, 0);
        return;
    }
    if (token == DWC2_TOKEN_SETUP) {
        s->ctrl_stall = false;
        if (len != 8) {
            host_xact_end(s, XACT_ERR, 0, 0);
            return;
        }
    } else if (ep == 0 && s->ctrl_stall && s->ctrl_stall_addr == addr) {
        host_xact_end(s, XACT_STALL, 0, 0);
        return;
    }
    if (token == DWC2_TOKEN_OUT &&
        chan_type(c) != USB_ENDPOINT_XFER_ISOC &&
        !!(s->dev_toggle[addr] & tbit) !=
        (chan_pid(c) == TSIZ_SC_MC_PID_DATA1)) {
        /* The device acknowledges a retransmission and drops it. */
        host_xact_end(s, XACT_ACK, len, 0);
        return;
    }
    uep = usb_ep_get(dev, pid == USB_TOKEN_SETUP ? USB_TOKEN_OUT : pid, ep);
    usb_packet_setup(&s->packet, pid, uep, 0, 0, false, false);
    usb_packet_addbuf(&s->packet, s->xfer_buf, token == DWC2_TOKEN_IN ?
                      MIN(chan_mps(c), sizeof(s->xfer_buf)) : len);
    usb_handle_packet(dev, &s->packet);
    if (s->packet.status == USB_RET_ASYNC) {
        s->xact_async = true;
        return;
    }
    host_xact_end(s, host_status_result(s->packet.status),
                  s->packet.actual_length,
                  !!(s->dev_toggle[addr] & tbit));
}

/*
 * The device answered: update the device's toggle as it would, then let
 * the bus carry the packet for its time before the channel sees the end.
 */
static void host_xact_end(DWC2OTGState *s, int result, uint32_t actual,
                          int data_pid)
{
    DWC2OTGHostChan *c = &s->hc[s->xact_chan];
    uint8_t addr = (c->hcchar & HCCHAR_DEVADDR_MASK) >> HCCHAR_DEVADDR_SHIFT;
    uint8_t ep = (c->hcchar & HCCHAR_EPNUM_MASK) >> HCCHAR_EPNUM_SHIFT;
    int token = s->xact_token;
    uint32_t tbit = 1u << (ep * 2 + (token != DWC2_TOKEN_IN));
    int64_t ns;

    if (result == XACT_ACK && token == DWC2_TOKEN_IN &&
        actual > chan_mps(c)) {
        result = XACT_BABBLE;
    }
    if (result == XACT_ACK && chan_type(c) != USB_ENDPOINT_XFER_ISOC) {
        if (token == DWC2_TOKEN_SETUP) {
            host_setup_toggles(s, addr, s->xfer_buf);
        } else if (token == DWC2_TOKEN_IN) {
            int want = chan_pid(c) == TSIZ_SC_MC_PID_DATA1;

            if (data_pid != want) {
                result = XACT_TOGGLE_ERR;
            }
            s->dev_toggle[addr] ^= tbit;
        } else if (!!(s->dev_toggle[addr] & tbit) ==
                   (chan_pid(c) == TSIZ_SC_MC_PID_DATA1)) {
            s->dev_toggle[addr] ^= tbit;
        }
    }
    if (result == XACT_STALL && ep == 0 && token != DWC2_TOKEN_SETUP) {
        s->ctrl_stall = true;
        s->ctrl_stall_addr = addr;
    }
    s->xact_result = result;
    s->xact_actual = token == DWC2_TOKEN_IN ? actual : s->xact_len;
    ns = host_xact_ns(s, token == DWC2_TOKEN_IN ? actual : s->xact_len);
    timer_mod(s->bus_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

/* Slave mode and buffer DMA results */

/* An IN or OUT transfer of a slave or buffer DMA channel completed. */
static void chan_xfer_done(DWC2OTGState *s, int ch)
{
    DWC2OTGHostChan *c = &s->hc[ch];

    if (dwc2_otg_dma_enabled(s)) {
        chan_disable(s, ch, HCINTMSK_XFERCOMPL | HCINTMSK_CHHLTD);
    } else if (chan_is_in(c)) {
        /* The status entry raises XferCompl as software pops it. */
        reqq_flush_chan(s, ch);
        dwc2_otg_rx_push(s, (ch << GRXSTS_HCHNUM_SHIFT) |
                         (GRXSTS_PKTSTS_HCHIN_XFER_COMP <<
                          GRXSTS_PKTSTS_SHIFT), NULL, 0);
    } else {
        c->hcchar &= ~HCCHAR_CHENA;
        chan_disable(s, ch, HCINTMSK_XFERCOMPL);
    }
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void chan_result(DWC2OTGState *s, int ch, int result, uint32_t actual)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    bool dma = dwc2_otg_dma_enabled(s);
    bool iso = chan_type(c) == USB_ENDPOINT_XFER_ISOC;
    uint32_t mps = chan_mps(c);

    if (chan_periodic(c)) {
        c->frame_done = true;
    }
    switch (result) {
    case XACT_ACK:
        c->err_cnt = 0;
        if (!iso) {
            c->hcint |= HCINTMSK_ACK;
        }
        if (s->xact_token == DWC2_TOKEN_IN) {
            uint32_t size = chan_xfersize(c), pkts = chan_pktcnt(c);

            if (dma) {
                if (!dwc2_otg_dma_write(s, c->hcdma, s->xfer_buf, actual)) {
                    chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
                    return;
                }
                c->hcdma += actual;
            } else {
                dwc2_otg_rx_push(s, (ch << GRXSTS_HCHNUM_SHIFT) |
                                 (actual << GRXSTS_BYTECNT_SHIFT) |
                                 ((chan_pid(c) & 3) << GRXSTS_DPID_SHIFT) |
                                 (GRXSTS_PKTSTS_HCHIN <<
                                  GRXSTS_PKTSTS_SHIFT),
                                 s->xfer_buf, actual);
                if (c->in_reqs) {
                    c->in_reqs--;
                }
            }
            size -= MIN(actual, size);
            pkts -= pkts ? 1 : 0;
            chan_set_xfer(c, size, pkts);
            chan_toggle(c);
            if (!pkts || actual < mps) {
                chan_xfer_done(s, ch);
            }
        } else {
            uint32_t size = chan_xfersize(c), pkts = chan_pktcnt(c);

            if (dma) {
                c->hcdma += s->xact_len;
                size -= MIN(s->xact_len, size);
            }
            pkts -= pkts ? 1 : 0;
            chan_set_xfer(c, size, pkts);
            chan_toggle(c);
            if (!pkts) {
                chan_xfer_done(s, ch);
            }
        }
        break;
    case XACT_NAK:
        c->err_cnt = 0;
        c->hcint |= HCINTMSK_NAK;
        if (!dma) {
            if (chan_is_in(c) && c->in_reqs) {
                c->in_reqs--;
            }
        } else if (chan_periodic(c)) {
            chan_halt(s, ch, 0);
        } else {
            c->nak_wait = true;
        }
        break;
    case XACT_STALL:
        c->hcint |= HCINTMSK_STALL;
        if (dma) {
            chan_halt(s, ch, 0);
        } else if (chan_is_in(c) && c->in_reqs) {
            c->in_reqs--;
        }
        break;
    case XACT_BABBLE:
        c->hcint |= HCINTMSK_BBLERR;
        if (dma) {
            chan_halt(s, ch, 0);
        } else if (c->in_reqs) {
            c->in_reqs--;
        }
        break;
    case XACT_TOGGLE_ERR:
        if (dma) {
            c->hcint |= HCINTMSK_DATATGLERR;
            chan_halt(s, ch, 0);
        } else {
            if (c->in_reqs) {
                c->in_reqs--;
            }
            dwc2_otg_rx_push(s, (ch << GRXSTS_HCHNUM_SHIFT) |
                             (GRXSTS_PKTSTS_DATATOGGLEERR <<
                              GRXSTS_PKTSTS_SHIFT), NULL, 0);
        }
        break;
    default:
        c->hcint |= HCINTMSK_XACTERR;
        if (dma) {
            if (++c->err_cnt >= 3 || iso) {
                chan_halt(s, ch, 0);
            } else {
                c->nak_wait = true;
            }
        } else if (chan_is_in(c) && c->in_reqs) {
            c->in_reqs--;
        }
        break;
    }
    dwc2_otg_update_irq(s);
}

/* Scatter/gather DMA */

static int qtd_count(DWC2OTGHostChan *c)
{
    return ((c->hctsiz & TSIZ_NTD_MASK) >> TSIZ_NTD_SHIFT) + 1;
}

static uint32_t qtd_ctd_mask(DWC2OTGHostChan *c)
{
    return qtd_count(c) > 64 ? HCDMA_CTD_MASK_256 : HCDMA_CTD_MASK;
}

static uint32_t qtd_addr(DWC2OTGHostChan *c, int idx)
{
    uint32_t base = qtd_count(c) > 64 ? HCDMA_BASE_MASK_256 : HCDMA_BASE_MASK;

    return (c->hcdma & base) + idx * 8;
}

static int qtd_index(DWC2OTGHostChan *c)
{
    return (c->hcdma & qtd_ctd_mask(c)) >> HCDMA_CTD_SHIFT;
}

static void qtd_set_index(DWC2OTGHostChan *c, int idx)
{
    c->hcdma = (c->hcdma & ~qtd_ctd_mask(c)) | (idx << HCDMA_CTD_SHIFT);
}

/*
 * Fetch the channel's current descriptor; an inactive one ends the
 * channel with BNA.
 */
static bool qtd_fetch(DWC2OTGState *s, int ch)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint32_t q[2];

    if (c->qtd_open) {
        return true;
    }
    if (!dwc2_otg_dma_read(s, qtd_addr(c, qtd_index(c)), q, sizeof(q))) {
        chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
        return false;
    }
    c->qtd_status = le32_to_cpu(q[0]);
    c->qtd_buf = le32_to_cpu(q[1]);
    if (!(c->qtd_status & HOST_DMA_A)) {
        chan_disable(s, ch, HCINTMSK_BNA);
        return false;
    }
    c->qtd_open = true;
    c->qtd_done = 0;
    c->hcdmab = c->qtd_buf;
    return true;
}

/* Write the descriptor back: bytes left, status, no longer active. */
static bool qtd_retire(DWC2OTGState *s, int ch, int status)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint32_t nbytes = c->qtd_status & QTD_NBYTES_MASK;
    uint32_t q = c->qtd_status & ~(HOST_DMA_A | HOST_DMA_STS_MASK |
                                   QTD_NBYTES_MASK);

    q |= (nbytes - MIN(c->qtd_done, nbytes)) |
         (status << HOST_DMA_STS_SHIFT);
    q = cpu_to_le32(q);
    c->qtd_open = false;
    if (!dwc2_otg_dma_write(s, qtd_addr(c, qtd_index(c)), &q, 4)) {
        chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
        return false;
    }
    return true;
}

static void qtd_advance(DWC2OTGState *s, int ch)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    int next = qtd_index(c) + 1;

    if (next >= qtd_count(c)) {
        next = 0;
        c->hcint |= HCINT_DESC_LST_ROLL;
    }
    qtd_set_index(c, next);
}

/*
 * An isochronous channel's descriptor for this (micro)frame: the list is
 * indexed by the (micro)frame number. A descriptor not active skips the
 * frame. Returns the bytes to move, or -1 for none.
 */
static int iso_qtd_fetch(DWC2OTGState *s, int ch)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint32_t q[2];

    qtd_set_index(c, s->frnum % qtd_count(c));
    if (!dwc2_otg_dma_read(s, qtd_addr(c, qtd_index(c)), q, sizeof(q))) {
        chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
        return -1;
    }
    c->qtd_status = le32_to_cpu(q[0]);
    c->qtd_buf = le32_to_cpu(q[1]);
    if (!(c->qtd_status & HOST_DMA_A)) {
        c->frame_done = true;
        return -1;
    }
    c->qtd_open = true;
    c->qtd_done = 0;
    c->hcdmab = c->qtd_buf;
    return c->qtd_status & HOST_DMA_ISOC_NBYTES_MASK;
}

/*
 * The end of an isochronous transaction: the descriptor written back with
 * the bytes not received (IN) and its status, IOC raising XferCompl. EOL
 * halts the channel after it (ESP-IDF's host stack ends each transfer so);
 * without one the channel runs on until software halts it.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void iso_ddma_result(DWC2OTGState *s, int ch, int result,
                            uint32_t actual)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint32_t nbytes = c->qtd_status & HOST_DMA_ISOC_NBYTES_MASK;
    bool ok = result == XACT_ACK && actual <= nbytes;
    uint32_t q;

    c->frame_done = true;
    c->qtd_open = false;
    if (ok && s->xact_token == DWC2_TOKEN_IN &&
        !dwc2_otg_dma_write(s, c->qtd_buf, s->xfer_buf, actual)) {
        chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
        return;
    }
    q = c->qtd_status & ~(HOST_DMA_A | HOST_DMA_STS_MASK |
                          HOST_DMA_ISOC_NBYTES_MASK);
    if (s->xact_token == DWC2_TOKEN_IN) {
        q |= nbytes - (ok ? actual : 0);
    }
    /* A packet larger than the descriptor's buffer is an error. */
    q |= (ok ? QTD_STS_SUCCESS : QTD_STS_PKTERR) << HOST_DMA_STS_SHIFT;
    q = cpu_to_le32(q);
    if (!dwc2_otg_dma_write(s, qtd_addr(c, qtd_index(c)), &q, 4)) {
        chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
        return;
    }
    if (c->qtd_status & HOST_DMA_IOC) {
        c->hcint |= HCINTMSK_XFERCOMPL;
    }
    if (c->qtd_status & HOST_DMA_EOL) {
        chan_disable(s, ch, HCINTMSK_CHHLTD);
        return;
    }
    dwc2_otg_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void ddma_result(DWC2OTGState *s, int ch, int result, uint32_t actual)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint32_t nbytes = c->qtd_status & QTD_NBYTES_MASK;
    uint32_t mps = chan_mps(c);
    bool done = false, shrt = false;

    if (chan_periodic(c)) {
        c->frame_done = true;
    }
    switch (result) {
    case XACT_ACK:
        c->err_cnt = 0;
        if (s->xact_token == DWC2_TOKEN_IN) {
            if (!dwc2_otg_dma_write(s, c->qtd_buf + c->qtd_done, s->xfer_buf,
                                    actual)) {
                chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
                return;
            }
            c->qtd_done += actual;
            shrt = actual < mps;
            done = shrt || c->qtd_done >= nbytes;
        } else {
            c->qtd_done += s->xact_len;
            done = c->qtd_done >= nbytes;
        }
        c->hcdmab = c->qtd_buf + c->qtd_done;
        chan_toggle(c);
        break;
    case XACT_NAK:
        c->err_cnt = 0;
        c->nak_wait = true;
        return;
    case XACT_STALL:
        if (qtd_retire(s, ch, QTD_STS_PKTERR)) {
            chan_disable(s, ch, HCINTMSK_STALL | HCINTMSK_CHHLTD);
        }
        return;
    case XACT_BABBLE:
        if (qtd_retire(s, ch, QTD_STS_PKTERR)) {
            chan_disable(s, ch, HCINTMSK_BBLERR | HCINTMSK_CHHLTD);
        }
        return;
    case XACT_TOGGLE_ERR:
        /* The data is dropped and the transaction retried. */
        c->nak_wait = true;
        return;
    default:
        if (++c->err_cnt >= 3) {
            if (qtd_retire(s, ch, QTD_STS_PKTERR)) {
                chan_disable(s, ch, HCINTMSK_XACTERR | HCINT_XCS_XACT_ERR |
                             HCINTMSK_CHHLTD);
            }
        } else {
            c->nak_wait = true;
        }
        return;
    }
    if (!done) {
        return;
    }
    if (!qtd_retire(s, ch, QTD_STS_SUCCESS)) {
        return;
    }
    if (c->qtd_status & HOST_DMA_EOL) {
        chan_disable(s, ch, HCINTMSK_CHHLTD |
                     ((c->qtd_status & HOST_DMA_IOC) ? HCINTMSK_XFERCOMPL
                                                     : 0));
        qtd_advance(s, ch);
        return;
    }
    if ((c->qtd_status & HOST_DMA_IOC) || shrt) {
        c->hcint |= HCINTMSK_XFERCOMPL;
    }
    qtd_advance(s, ch);
    dwc2_otg_update_irq(s);
}

/* Scheduling */

/* Whether periodic channel @c may send in this (micro)frame. */
static bool chan_due(DWC2OTGState *s, DWC2OTGHostChan *c)
{
    if (c->frame_done) {
        return false;
    }
    if (host_ddma(s)) {
        uint32_t len = 8 << ((s->hcfg & HCFG_FRLISTEN_MASK) >>
                             HCFG_FRLISTEN_SHIFT);
        uint32_t frame = s->port_speed == DWC2_SPEED_HIGH ? s->frnum >> 3
                                                          : s->frnum;
        uint32_t entry;

        if (!(s->hcfg & HCFG_PERSCHEDENA)) {
            return false;
        }
        if (!dwc2_otg_dma_read(s, s->hflbaddr + (frame % len) * 4, &entry,
                               4)) {
            return false;
        }
        if (!(le32_to_cpu(entry) & BIT(c - s->hc))) {
            return false;
        }
        if (s->port_speed == DWC2_SPEED_HIGH &&
            !(c->hctsiz & BIT(s->frnum & 7))) {
            return false;
        }
        return true;
    }
    return !!(c->hcchar & HCCHAR_ODDFRM) == (s->frnum & 1);
}

/*
 * Start the next transaction of channel @ch if it has one; false if it
 * has nothing to send now.
 */
static bool chan_start(DWC2OTGState *s, int ch)
{
    DWC2OTGHostChan *c = &s->hc[ch];
    uint8_t buf[1024];
    uint32_t len, mps = MIN(chan_mps(c), sizeof(buf));
    int token;

    if (!(c->hcchar & HCCHAR_CHENA)) {
        return false;
    }
    if (dwc2_otg_dma_enabled(s) && c->halt_req) {
        chan_halt(s, ch, 0);
        return false;
    }
    if (c->nak_wait) {
        return false;
    }
    if (chan_periodic(c) && !chan_due(s, c)) {
        return false;
    }
    if (host_ddma(s)) {
        if (chan_type(c) == USB_ENDPOINT_XFER_ISOC) {
            int n = iso_qtd_fetch(s, ch);

            if (n < 0) {
                return false;
            }
            if (chan_is_in(c)) {
                host_issue(s, ch, DWC2_TOKEN_IN, NULL, 0);
                return true;
            }
            n = MIN(n, sizeof(buf));
            if (!dwc2_otg_dma_read(s, c->qtd_buf, buf, n)) {
                chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
                return false;
            }
            host_issue(s, ch, DWC2_TOKEN_OUT, buf, n);
            return true;
        }
        if (!qtd_fetch(s, ch)) {
            return false;
        }
        token = chan_token(c, (c->qtd_status & HOST_DMA_SUP) &&
                              c->qtd_done == 0 && !chan_is_in(c));
        if (token == DWC2_TOKEN_SETUP) {
            chan_set_pid(c, TSIZ_SC_MC_PID_SETUP);
        }
        if (token == DWC2_TOKEN_IN) {
            host_issue(s, ch, token, NULL, 0);
            return true;
        }
        len = MIN((c->qtd_status & QTD_NBYTES_MASK) - c->qtd_done, mps);
        if (!dwc2_otg_dma_read(s, c->qtd_buf + c->qtd_done, buf, len)) {
            chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
            return false;
        }
        host_issue(s, ch, token, buf, len);
        return true;
    }
    token = chan_token(c, false);
    if (dwc2_otg_dma_enabled(s)) {
        if (!chan_pktcnt(c)) {
            chan_xfer_done(s, ch);
            return false;
        }
        if (token == DWC2_TOKEN_IN) {
            host_issue(s, ch, token, NULL, 0);
            return true;
        }
        len = MIN(chan_xfersize(c), mps);
        if (!dwc2_otg_dma_read(s, c->hcdma, buf, len)) {
            chan_disable(s, ch, HCINTMSK_AHBERR | HCINTMSK_CHHLTD);
            return false;
        }
        host_issue(s, ch, token, buf, len);
        return true;
    }
    return false;
}

/*
 * Slave mode: the request at the head of queue @qi, if its channel can go:
 * an IN needs room in the receive FIFO for a packet and its status, an
 * OUT its packet at the head of the transmit FIFO.
 */
static bool reqq_start(DWC2OTGState *s, int qi)
{
    DWC2OTGReqQueue *q = &s->reqq[qi];
    int fifo = qi ? DWC2_PTXF : DWC2_NPTXF;
    uint8_t buf[1024];
    int ch, type;
    DWC2OTGHostChan *c;

    if (!q->n) {
        return false;
    }
    ch = q->chan[0];
    type = q->type[0];
    c = &s->hc[ch];
    if (type == DWC2_REQ_HALT) {
        reqq_pop(q, 0);
        chan_halt(s, ch, 0);
        return false;
    }
    if (!(c->hcchar & HCCHAR_CHENA)) {
        reqq_pop(q, 0);
        return false;
    }
    if (qi && !chan_due(s, c)) {
        return false;
    }
    if (chan_is_in(c)) {
        if (dwc2_otg_rx_free(s) < 2 + DIV_ROUND_UP(chan_mps(c), 4)) {
            return false;
        }
        reqq_pop(q, 0);
        host_issue(s, ch, DWC2_TOKEN_IN, NULL, 0);
        return true;
    }
    reqq_pop(q, 0);
    if (type == DWC2_REQ_ZLP) {
        host_issue(s, ch, chan_token(c, false), NULL, 0);
        return true;
    }
    if (!s->tx[fifo].npkts || s->tx[fifo].pkt_chan[0] != ch) {
        return false;
    }
    {
        uint32_t len = s->tx[fifo].pkt_len[0];

        dwc2_otg_tx_pop(s, fifo, buf);
        host_issue(s, ch, chan_token(c, false), buf, len);
    }
    return true;
}

/* Run the bus: finish the transaction on it, then start the next. */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void host_bus(void *opaque)
{
    DWC2OTGState *s = opaque;
    int i, ch;

    if (s->xact_busy) {
        if (s->xact_async) {
            return;
        }
        s->xact_busy = false;
        ch = s->xact_chan;
        if (host_ddma(s) && chan_type(&s->hc[ch]) == USB_ENDPOINT_XFER_ISOC) {
            iso_ddma_result(s, ch, s->xact_result, s->xact_actual);
        } else if (host_ddma(s)) {
            ddma_result(s, ch, s->xact_result, s->xact_actual);
        } else {
            chan_result(s, ch, s->xact_result, s->xact_actual);
        }
        if (s->hc[ch].halt_req && (s->hc[ch].hcchar & HCCHAR_CHENA) &&
            dwc2_otg_dma_enabled(s)) {
            chan_halt(s, ch, 0);
        }
    }
    if (!host_running(s)) {
        return;
    }
    if (!dwc2_otg_dma_enabled(s)) {
        if (reqq_start(s, 1) || reqq_start(s, 0)) {
            return;
        }
        if (s->reqq[0].n || s->reqq[1].n) {
            /* A halt or a dropped request may have unblocked the head. */
            if (s->reqq[0].n && s->reqq[0].type[0] == DWC2_REQ_HALT) {
                host_bus_kick(s, 0);
            }
        }
        return;
    }
    for (ch = 0; ch < s->num_chans; ch++) {
        if (chan_periodic(&s->hc[ch]) && chan_start(s, ch)) {
            return;
        }
    }
    for (i = 0; i < s->num_chans; i++) {
        ch = (s->np_next + i) % s->num_chans;
        if (!chan_periodic(&s->hc[ch]) && chan_start(s, ch)) {
            s->np_next = (ch + 1) % s->num_chans;
            return;
        }
    }
}

/*
 * Begin the next (micro)frame where the last ended. The frames end at the
 * first nanosecond their PHY clocks have elapsed, counted from the first
 * at the same PHY clock, so that they keep to the clock however many
 * there are.
 */
static void host_frame_next(DWC2OTGState *s)
{
    uint64_t period = dwc2_otg_phy_period(s);
    uint32_t len = s->hfir & HFIR_FRINT_MASK;

    s->frame_start_ns += s->frame_ns;
    if (period != s->frame_period) {
        s->frame_period = period;
        s->frame_origin_ns = s->frame_start_ns;
        s->frame_ticks = 0;
    }
    if (!period || !len) {
        s->frame_ns = host_frame_len_ns(s);
        s->frame_period = 0;
        return;
    }
    s->frame_ticks += len;
    s->frame_ns = s->frame_origin_ns +
                  dwc2_otg_phy_ticks_ns(s, s->frame_ticks) - s->frame_start_ns;
}

/* A (micro)frame boundary: the SOF, and a new round of transactions. */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void host_frame(void *opaque)
{
    DWC2OTGState *s = opaque;
    int i;

    if (!host_running(s)) {
        return;
    }
    host_frame_next(s);
    timer_mod(s->frame_timer, s->frame_start_ns + s->frame_ns);
    s->frnum = (s->frnum + 1) & HFNUM_MAX_FRNUM;
    s->gintsts |= GINTSTS_SOF;
    for (i = 0; i < s->num_chans; i++) {
        DWC2OTGHostChan *c = &s->hc[i];

        if (chan_periodic(c) && !c->frame_done &&
            (c->hcchar & HCCHAR_CHENA) && !host_ddma(s) &&
            dwc2_otg_dma_enabled(s) &&
            !!(c->hcchar & HCCHAR_ODDFRM) != (s->frnum & 1)) {
            /* Its frame passed without its transaction: overrun. */
            c->hcint |= HCINTMSK_FRMOVRUN;
            chan_halt(s, i, 0);
        }
        c->frame_done = false;
        c->nak_wait = false;
    }
    dwc2_otg_update_irq(s);
    host_bus_kick(s, 0);
}

/* QEMU USB port */

static void host_attach(USBPort *port)
{
    DWC2OTGState *s = port->opaque;

    dwc2_otg_host_update_port(s);
}

static void host_detach(USBPort *port)
{
    DWC2OTGState *s = port->opaque;

    host_cancel(s);
    host_reset_toggles(s);
    /* The device is still on the port while it detaches. */
    host_update_port_dev(s, NULL);
}

static void host_child_detach(USBPort *port, USBDevice *child)
{
    DWC2OTGState *s = port->opaque;

    if (s->xact_async && s->packet.ep && s->packet.ep->dev == child) {
        usb_cancel_packet(&s->packet);
        s->xact_async = false;
        host_xact_end(s, XACT_ERR, 0, 0);
    }
}

/* A device's remote wakeup resumes the suspended port. */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void host_wakeup(USBPort *port)
{
    DWC2OTGState *s = port->opaque;

    if (s->host_mode && (s->hprt & HPRT0_SUSP)) {
        s->hprt |= HPRT0_RES;
        s->gintsts |= GINTSTS_WKUPINT;
        dwc2_otg_update_irq(s);
    }
}

static void host_complete(USBPort *port, USBPacket *p)
{
    DWC2OTGState *s = port->opaque;
    DWC2OTGHostChan *c;
    uint8_t addr, ep;
    uint32_t tbit;

    if (p != &s->packet || !s->xact_async) {
        return;
    }
    s->xact_async = false;
    if (p->status == USB_RET_REMOVE_FROM_QUEUE) {
        return;
    }
    c = &s->hc[s->xact_chan];
    addr = (c->hcchar & HCCHAR_DEVADDR_MASK) >> HCCHAR_DEVADDR_SHIFT;
    ep = (c->hcchar & HCCHAR_EPNUM_MASK) >> HCCHAR_EPNUM_SHIFT;
    tbit = 1u << (ep * 2 + (s->xact_token != DWC2_TOKEN_IN));
    host_xact_end(s, host_status_result(p->status), p->actual_length,
                  !!(s->dev_toggle[addr] & tbit));
}

static USBPortOps dwc2_otg_port_ops = {
    .attach = host_attach,
    .detach = host_detach,
    .child_detach = host_child_detach,
    .wakeup = host_wakeup,
    .complete = host_complete,
};

static USBBusOps dwc2_otg_bus_ops = {
};

void dwc2_otg_host_mode_changed(DWC2OTGState *s)
{
    int i;

    host_cancel(s);
    s->hprt = 0;
    for (i = 0; i < s->num_chans; i++) {
        s->hc[i].hcchar &= ~(HCCHAR_CHENA | HCCHAR_CHDIS);
    }
    memset(s->reqq, 0, sizeof(s->reqq));
    dwc2_otg_host_update_port(s);
}

void dwc2_otg_host_reset(DWC2OTGState *s)
{
    host_cancel(s);
    timer_del(s->frame_timer);
    s->port_connected = false;
    s->port_speed = DWC2_SPEED_FULL;
    s->frnum = HFNUM_MAX_FRNUM;
    s->frame_ns = 0;
    s->frame_start_ns = 0;
    s->frame_origin_ns = 0;
    s->frame_ticks = 0;
    s->frame_period = 0;
    s->np_next = 0;
    host_reset_toggles(s);
    qemu_set_irq(s->suspended_out, 0);
}

void dwc2_otg_host_pre_save(DWC2OTGState *s)
{
    /* A QEMU device's packet in flight cannot be migrated: redo it. */
    if (s->xact_async) {
        usb_cancel_packet(&s->packet);
        s->xact_async = false;
        s->xact_busy = false;
        s->hc[s->xact_chan].qtd_open = false;
        host_bus_kick(s, 0);
    }
}

void dwc2_otg_host_init(DWC2OTGState *s)
{
    DeviceState *dev = DEVICE(s);
    const char *base = strrchr(s->name, '.');
    int speeds = USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL;

    if (s->has_utmi) {
        speeds |= USB_SPEED_MASK_HIGH;
    }
    /*
     * The bus is named after the controller ("usb-otg-hs.0" for
     * "esp32p4.usb-otg-hs"), as a bus of a device with that id is, so
     * that -device ...,bus= can find it.
     */
    if (!dev->id) {
        dev->id = g_strdup(base ? base + 1 : s->name);
        usb_bus_new(&s->bus, sizeof(s->bus), &dwc2_otg_bus_ops, dev);
        g_free(dev->id);
        dev->id = NULL;
    } else {
        usb_bus_new(&s->bus, sizeof(s->bus), &dwc2_otg_bus_ops, dev);
    }
    usb_register_port(&s->bus, &s->port, s, 0, &dwc2_otg_port_ops, speeds);
    usb_packet_init(&s->packet);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, host_frame, s);
    s->bus_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, host_bus, s);
}

const VMStateDescription vmstate_dwc2_otg_hc = {
    .name = TYPE_DWC2_OTG "/hc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(hcchar, DWC2OTGHostChan),
        VMSTATE_UINT32(hcsplt, DWC2OTGHostChan),
        VMSTATE_UINT32(hcint, DWC2OTGHostChan),
        VMSTATE_UINT32(hcintmsk, DWC2OTGHostChan),
        VMSTATE_UINT32(hctsiz, DWC2OTGHostChan),
        VMSTATE_UINT32(hcdma, DWC2OTGHostChan),
        VMSTATE_UINT32(hcdmab, DWC2OTGHostChan),
        VMSTATE_UINT8(err_cnt, DWC2OTGHostChan),
        VMSTATE_BOOL(frame_done, DWC2OTGHostChan),
        VMSTATE_BOOL(nak_wait, DWC2OTGHostChan),
        VMSTATE_BOOL(halt_req, DWC2OTGHostChan),
        VMSTATE_UINT8(in_reqs, DWC2OTGHostChan),
        VMSTATE_UINT16(pushed, DWC2OTGHostChan),
        VMSTATE_BOOL(qtd_open, DWC2OTGHostChan),
        VMSTATE_UINT32(qtd_status, DWC2OTGHostChan),
        VMSTATE_UINT32(qtd_buf, DWC2OTGHostChan),
        VMSTATE_UINT32(qtd_done, DWC2OTGHostChan),
        VMSTATE_END_OF_LIST()
    },
};
