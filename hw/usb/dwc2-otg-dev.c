/*
 * Synopsys DWC2 OTG controller: device mode
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * In device mode the core is the device on its port, and one of the hosts
 * of usb-devport.h sits at the other end of the cable. A host sees the
 * device while the core is in device mode with a valid B-session and
 * DCTL.SftDiscon clear (or while the PHY's pull overrides pull D+ up), and
 * answers each of its transactions as the core's endpoint state machines
 * decide:
 *
 *   - A bus reset raises GINTSTS.USBRst and deactivates every endpoint but
 *     EP0; its end raises EnumDone with the speed in DSTS.EnumSpd. SOFs
 *     raise GINTSTS.SOF and set DSTS.SOFFN, and EOPF at DCFG.PerFrInt of
 *     the frame; 3 ms without bus activity raise ErlySusp and USBSusp,
 *     activity after it WkUpInt.
 *   - A SETUP addressed to the device is always taken by control OUT
 *     endpoint 0, clearing EP0's STALL and NAKing its data stage until
 *     software clears the NAK; the first data or status token after it
 *     ends the setup phase (DOEPINT.SETUP).
 *   - IN and OUT tokens are answered by the endpoint: no reply unless the
 *     endpoint is active, STALL, NAK while it is disabled, NAKed (or the
 *     global IN or OUT NAK is in effect) or has no data or room, and
 *     otherwise data with the endpoint's data toggle; an OUT packet with
 *     the wrong toggle is acknowledged and dropped. Each packet counts
 *     down DxEPTSIZ, and the last one, or a short OUT packet, completes
 *     the transfer (XferCompl), disabling the endpoint and, for OUT, NAKing
 *     it.
 *   - In slave mode received packets and their status entries go to the
 *     receive FIFO (SETUP data, OUT data, and the setup done, OUT done and
 *     global OUT NAK entries, which raise their interrupt as software pops
 *     them) and IN packets come from each endpoint's dedicated transmit
 *     FIFO, a packet once its bytes are pushed, with TxFEmp as the FIFO
 *     empties. In buffer DMA mode the core moves the data between the
 *     bus and memory at DxEPDMA itself, and a SETUP is written to control
 *     OUT endpoint 0's buffer once software enables it, closing the buffer
 *     (STUPPktRcvd, the endpoint disabled) as cores from 3.00a do.
 *
 * The hosts are full-speed hosts, so a high-speed core enumerates at full
 * speed (DSTS.EnumSpd). Not modelled, and logged as unimplemented:
 * scatter/gather DMA in device mode (DCFG.DescDMA), the frame parity of
 * isochronous endpoints, and the incomplete isochronous interrupts.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "dwc2-otg-internal.h"

#define DEV_SUSPEND_NS          (3 * SCALE_MS)
#define FS_FRAME_NS             SCALE_MS

#define DIEPCTL0_MPS_MASK       0x3
#define DXEPCTL_DPID_BIT        BIT(16)
#define DOEPTSIZ0_SUPCNT_SHIFT  29

static uint32_t ep_mps(DWC2OTGState *s, DWC2OTGDevEp *e, int ep)
{
    static const uint16_t ep0_mps[4] = { 64, 32, 16, 8 };

    if (ep == 0) {
        return ep0_mps[s->in[0].ctl & DIEPCTL0_MPS_MASK];
    }
    return e->ctl & DXEPCTL_MPS_MASK;
}

static int ep_type(DWC2OTGDevEp *e)
{
    return (e->ctl & DXEPCTL_EPTYPE_MASK) >> 18;
}

static uint32_t ep_xfersize(DWC2OTGDevEp *e, int ep)
{
    return e->tsiz & (ep ? DXEPTSIZ_XFERSIZE_MASK : 0x7f);
}

static uint32_t ep_pktcnt(DWC2OTGDevEp *e, int ep, bool in)
{
    if (ep == 0) {
        return (e->tsiz >> DXEPTSIZ_PKTCNT_SHIFT) & (in ? 3 : 1);
    }
    return (e->tsiz & DXEPTSIZ_PKTCNT_MASK) >> DXEPTSIZ_PKTCNT_SHIFT;
}

static void ep_set_xfer(DWC2OTGDevEp *e, int ep, bool in, uint32_t size,
                        uint32_t pkts)
{
    uint32_t smask = ep ? DXEPTSIZ_XFERSIZE_MASK : 0x7f;
    uint32_t pmask = ep ? DXEPTSIZ_PKTCNT_MASK
                        : ((in ? 3u : 1u) << DXEPTSIZ_PKTCNT_SHIFT);

    e->tsiz = (e->tsiz & ~(smask | pmask)) | (size & smask) |
              ((pkts << DXEPTSIZ_PKTCNT_SHIFT) & pmask);
}

/* The data toggle the endpoint sends or expects next. */
static int ep_toggle(DWC2OTGState *s, DWC2OTGDevEp *e, int ep, bool in)
{
    if (ep == 0) {
        return !!(s->dev_ep_toggle & BIT(in ? 16 : 0));
    }
    return !!(e->ctl & DXEPCTL_DPID_BIT);
}

static void ep_flip(DWC2OTGState *s, DWC2OTGDevEp *e, int ep, bool in)
{
    if (ep == 0) {
        s->dev_ep_toggle ^= BIT(in ? 16 : 0);
    } else {
        e->ctl ^= DXEPCTL_DPID_BIT;
    }
}

static uint8_t dev_enum_speed(DWC2OTGState *s)
{
    uint32_t spd = s->dcfg & DCFG_DEVSPD_MASK;

    if (spd == DCFG_DEVSPD_LS) {
        return DWC2_SPEED_LOW;
    }
    return (s->gusbcfg & GUSBCFG_PHYSEL) ? DWC2_SPEED_FULL48
                                         : DWC2_SPEED_FULL;
}

/* Interrupts */

uint32_t dwc2_otg_diepint_read(DWC2OTGState *s, int ep)
{
    DWC2OTGDevEp *e = &s->in[ep];
    int fifo = (e->ctl & DXEPCTL_TXFNUM_MASK) >> DXEPCTL_TXFNUM_SHIFT;
    uint32_t v = e->intr & ~DXEPINT_TXFEMP;
    uint32_t depth, used;

    if (fifo < s->num_in_fifos) {
        depth = dwc2_otg_tx_depth(s, fifo);
        used = s->tx[fifo].used;
        if ((s->gahbcfg & GAHBCFG_NP_TXF_EMP_LVL) ? !used
                                                  : used * 2 <= depth) {
            v |= DXEPINT_TXFEMP;
        }
    }
    return v;
}

static uint32_t dev_in_pending(DWC2OTGState *s, int ep, uint32_t mask)
{
    uint32_t v = dwc2_otg_diepint_read(s, ep);
    uint32_t m = mask & ~DXEPINT_TXFEMP;

    if (s->diepempmsk & BIT(ep)) {
        m |= DXEPINT_TXFEMP;
    }
    return v & m;
}

uint32_t dwc2_otg_daint(DWC2OTGState *s)
{
    uint32_t v = 0;
    int ep;

    for (ep = 0; ep < s->num_eps; ep++) {
        if (dev_in_pending(s, ep, s->diepmsk)) {
            v |= BIT(ep);
        }
        if (s->out[ep].intr & s->doepmsk) {
            v |= BIT(16 + ep);
        }
    }
    return v;
}

uint32_t dwc2_otg_deachint(DWC2OTGState *s)
{
    uint32_t v = 0;
    int ep;

    for (ep = 0; ep < s->num_eps; ep++) {
        if (dev_in_pending(s, ep, s->diepeachmsk[ep])) {
            v |= BIT(ep);
        }
        if (s->out[ep].intr & s->doepeachmsk[ep]) {
            v |= BIT(16 + ep);
        }
    }
    return v;
}

/* Bus state */

static void dev_activity(DWC2OTGState *s)
{
    if (s->dev_suspended) {
        s->dev_suspended = false;
        s->gintsts |= GINTSTS_WKUPINT;
        qemu_set_irq(s->suspended_out, 0);
    }
    timer_mod(s->idle_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DEV_SUSPEND_NS);
}

/* 3 ms of an idle bus: the device suspends. */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void dev_idle(void *opaque)
{
    DWC2OTGState *s = opaque;

    if (!s->dev_present || s->dev_suspended || s->dev_bus_reset) {
        return;
    }
    s->dev_suspended = true;
    s->gintsts |= GINTSTS_ERLYSUSP | GINTSTS_USBSUSP;
    qemu_set_irq(s->suspended_out, 1);
    dwc2_otg_update_irq(s);
}

/* The end of the periodic frame, DCFG.PerFrInt into it. */
static void dev_eopf(void *opaque)
{
    DWC2OTGState *s = opaque;

    if (s->dev_present) {
        s->gintsts |= GINTSTS_EOPF;
        dwc2_otg_update_irq(s);
    }
}

/*
 * Whether a host sees the device: D+ pulled up by the core in device mode
 * with a B-session and no soft disconnect, or by the PHY's override.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_dev_update_present(DWC2OTGState *s)
{
    bool pullup = !s->host_mode && !(s->dctl & DCTL_SFTDISCON) &&
                  s->bsesvld;
    bool present;

    qemu_set_irq(s->dp_pullup_out, pullup);
    if (s->pull_override) {
        pullup = s->pull_dp;
    }
    present = pullup && s->phy_connected;
    if (present == s->dev_present) {
        return;
    }
    s->dev_present = present;
    if (!present) {
        timer_del(s->idle_timer);
        timer_del(s->eopf_timer);
        s->dev_suspended = false;
        s->dev_bus_reset = false;
        qemu_set_irq(s->suspended_out, 0);
    }
    usb_devport_changed(&s->devport);
}

void dwc2_otg_dev_kick(DWC2OTGState *s)
{
    usb_devport_kick(&s->devport);
}

/* SETUP delivery and the setup phase */

static void dev_setup_reset(DWC2OTGState *s)
{
    s->setup_pending = false;
    s->setup_bufs = 0;
}

/*
 * Buffer DMA: control OUT endpoint 0, enabled, takes the oldest SETUP the
 * core holds into its buffer, which closes it: the endpoint disables
 * itself with STUPPktRcvd.
 */
static void dev_deliver_setup(DWC2OTGState *s)
{
    DWC2OTGDevEp *e = &s->out[0];
    uint32_t supcnt;

    if (!s->setup_bufs || !dwc2_otg_dma_enabled(s) ||
        !(e->ctl & DXEPCTL_EPENA)) {
        return;
    }
    if (!dwc2_otg_dma_write(s, e->dma, s->setup_buf[0], 8)) {
        e->intr |= DXEPINT_AHBERR;
        e->ctl &= ~DXEPCTL_EPENA;
        dwc2_otg_update_irq(s);
        return;
    }
    e->dma += 8;
    supcnt = (e->tsiz >> DOEPTSIZ0_SUPCNT_SHIFT) & 3;
    e->tsiz = (e->tsiz & ~DOEPTSIZ0_SUPCNT_MASK) |
              ((supcnt ? supcnt - 1 : 0) << DOEPTSIZ0_SUPCNT_SHIFT);
    ep_set_xfer(e, 0, false, ep_xfersize(e, 0) - MIN(8, ep_xfersize(e, 0)),
                0);
    e->ctl &= ~DXEPCTL_EPENA;
    e->ctl |= DXEPCTL_NAKSTS;
    e->intr |= DXEPINT_SETUP_RCVD;
    s->setup_bufs--;
    memmove(s->setup_buf[0], s->setup_buf[1], s->setup_bufs * 8);
    if (!s->setup_bufs && !s->setup_pending) {
        /* The setup phase already ended: the SETUP is ready to decode. */
        e->intr |= DXEPINT_SETUP;
    }
    dwc2_otg_update_irq(s);
}

/*
 * The first data or status token after a SETUP ends the setup phase: a
 * status entry in slave mode, which raises DOEPINT0.SETUP as software pops
 * it, or DOEPINT0.SETUP once the SETUP is in memory in DMA mode.
 */
static void dev_setup_done(DWC2OTGState *s)
{
    if (!s->setup_pending) {
        return;
    }
    s->setup_pending = false;
    if (dwc2_otg_dma_enabled(s)) {
        if (!s->setup_bufs) {
            s->out[0].intr |= DXEPINT_SETUP;
        }
    } else if (dwc2_otg_rx_free(s) >= 1) {
        dwc2_otg_rx_push(s, (GRXSTS_PKTSTS_SETUPDONE << GRXSTS_PKTSTS_SHIFT),
                         NULL, 0);
    } else {
        s->out[0].intr |= DXEPINT_SETUP;
    }
    dwc2_otg_update_irq(s);
}

void dwc2_otg_dev_rx_popped(DWC2OTGState *s, uint32_t status)
{
    int ep = (status & GRXSTS_EPNUM_MASK) >> GRXSTS_EPNUM_SHIFT;
    int pktsts = (status & GRXSTS_PKTSTS_MASK) >> GRXSTS_PKTSTS_SHIFT;

    if (ep >= s->num_eps) {
        return;
    }
    switch (pktsts) {
    case GRXSTS_PKTSTS_GLOBALOUTNAK:
        s->gintsts |= GINTSTS_GOUTNAKEFF;
        break;
    case GRXSTS_PKTSTS_OUTDONE:
        s->out[ep].intr |= DXEPINT_XFERCOMPL;
        break;
    case GRXSTS_PKTSTS_SETUPDONE:
        s->out[0].intr |= DXEPINT_SETUP;
        break;
    default:
        return;
    }
    dwc2_otg_update_irq(s);
}

/* The host's side of the cable (usb-devport.h) */

static bool dev_addressed(DWC2OTGState *s, uint8_t addr)
{
    return addr == s->dev_addr;
}

static uint8_t dcfg_addr(DWC2OTGState *s)
{
    return (s->dcfg & DCFG_DEVADDR_MASK) >> DCFG_DEVADDR_SHIFT;
}

/*
 * Software writes SET_ADDRESS's address to DCFG.DevAddr before the
 * request's status stage, which the host still sends to the old address:
 * the device takes the new one as the control transfer ends.
 */
void dwc2_otg_dcfg_written(DWC2OTGState *s)
{
    if (!s->ctrl_active) {
        s->dev_addr = dcfg_addr(s);
    }
}

static void dev_ctrl_end(DWC2OTGState *s)
{
    s->ctrl_active = false;
    s->dev_addr = dcfg_addr(s);
}

static bool devport_present(void *opaque)
{
    DWC2OTGState *s = opaque;

    return s->dev_present;
}

static bool devport_suspended(void *opaque)
{
    DWC2OTGState *s = opaque;

    return s->dev_suspended;
}

static void devport_host_attached(void *opaque, bool attached)
{
    DWC2OTGState *s = opaque;

    s->dev_host_attached = attached;
    qemu_set_irq(s->host_attached_out, attached);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void devport_bus_reset(void *opaque, bool asserted)
{
    DWC2OTGState *s = opaque;
    int ep;

    if (!s->dev_present) {
        return;
    }
    if (asserted) {
        if (s->dev_suspended) {
            s->gintsts |= GINTSTS_RESETDET;
        }
        s->dev_suspended = false;
        qemu_set_irq(s->suspended_out, 0);
        s->dev_bus_reset = true;
        s->gintsts |= GINTSTS_USBRST;
        for (ep = 1; ep < s->num_eps; ep++) {
            s->in[ep].ctl &= ~DXEPCTL_USBACTEP;
            s->out[ep].ctl &= ~DXEPCTL_USBACTEP;
        }
        s->dev_ep_toggle = 0;
        dev_setup_reset(s);
        dev_ctrl_end(s);
        timer_del(s->idle_timer);
    } else {
        s->dev_bus_reset = false;
        s->gintsts |= GINTSTS_ENUMDONE;
        dev_activity(s);
    }
    dwc2_otg_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void devport_sof(void *opaque, uint16_t frame)
{
    DWC2OTGState *s = opaque;
    static const uint8_t perfrint[4] = { 80, 85, 90, 95 };
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!s->dev_present || s->dev_bus_reset) {
        return;
    }
    dev_activity(s);
    if (!dwc2_otg_phy_hz(s)) {
        return;
    }
    s->dev_soffn = frame & 0x7ff;
    s->gintsts |= GINTSTS_SOF;
    timer_mod(s->eopf_timer, now + FS_FRAME_NS *
              perfrint[(s->dcfg & DCFG_PERFRINT_MASK) >> DCFG_PERFRINT_SHIFT] /
              100);
    dwc2_otg_update_irq(s);
}

/* Common checks of a transaction: no reply unless the core can answer. */
static bool dev_answers(DWC2OTGState *s, uint8_t addr)
{
    if (!s->dev_present || s->dev_bus_reset) {
        return false;
    }
    dev_activity(s);
    return dwc2_otg_phy_hz(s) && dev_addressed(s, addr);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static USBDevPortHandshake devport_setup(void *opaque, uint8_t addr,
                                         const uint8_t *data)
{
    DWC2OTGState *s = opaque;
    DWC2OTGDevEp *e = &s->out[0];
    uint32_t supcnt;

    if (!dev_answers(s, addr)) {
        return USB_DEVPORT_NORESP;
    }
    if (dwc2_otg_dma_enabled(s) && (s->dcfg & DCFG_DESCDMA_EN)) {
        qemu_log_mask(LOG_UNIMP, "%s: scatter/gather DMA in device mode is "
                      "not modelled\n", s->name);
        return USB_DEVPORT_NORESP;
    }
    if (!dwc2_otg_dma_enabled(s) && dwc2_otg_rx_free(s) < 3) {
        /* No room for the SETUP: the host sees no handshake. */
        return USB_DEVPORT_NORESP;
    }
    /* SETUP clears EP0's STALL and NAKs its data stage. */
    s->in[0].ctl = (s->in[0].ctl & ~DXEPCTL_STALL) | DXEPCTL_NAKSTS;
    e->ctl = (e->ctl & ~DXEPCTL_STALL) | DXEPCTL_NAKSTS;
    s->dev_ep_toggle |= BIT(0) | BIT(16);
    s->ctrl_in = data[0] & USB_DIR_IN;
    s->ctrl_active = true;
    supcnt = (e->tsiz >> DOEPTSIZ0_SUPCNT_SHIFT) & 3;
    if (s->setup_pending && !supcnt) {
        e->intr |= DXEPINT_BACK2BACKSETUP;
    }
    s->setup_pending = true;
    if (dwc2_otg_dma_enabled(s)) {
        if (s->setup_bufs == DWC2_OTG_SETUP_BUFS) {
            /* The oldest of more than three back-to-back SETUPs is lost. */
            s->setup_bufs--;
            memmove(s->setup_buf[0], s->setup_buf[1], s->setup_bufs * 8);
        }
        memcpy(s->setup_buf[s->setup_bufs++], data, 8);
        dev_deliver_setup(s);
    } else {
        e->tsiz = (e->tsiz & ~DOEPTSIZ0_SUPCNT_MASK) |
                  ((supcnt ? supcnt - 1 : 0) << DOEPTSIZ0_SUPCNT_SHIFT);
        dwc2_otg_rx_push(s, (8 << GRXSTS_BYTECNT_SHIFT) |
                         (GRXSTS_PKTSTS_SETUPRX << GRXSTS_PKTSTS_SHIFT),
                         data, 8);
    }
    dwc2_otg_update_irq(s);
    return USB_DEVPORT_ACK;
}

static void ep_in_done(DWC2OTGState *s, DWC2OTGDevEp *e)
{
    e->ctl &= ~DXEPCTL_EPENA;
    e->intr |= DXEPINT_XFERCOMPL;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static USBDevPortHandshake devport_in(void *opaque, uint8_t addr,
                                      uint8_t ep, uint8_t *buf, uint16_t max,
                                      uint16_t *len, int *pid)
{
    DWC2OTGState *s = opaque;
    DWC2OTGDevEp *e;
    uint8_t data[1024];
    uint32_t mps, size, pkts, n = 0;
    int fifo, type;
    bool iso;

    *len = 0;
    *pid = 0;
    if (!dev_answers(s, addr) || ep >= s->num_eps) {
        return USB_DEVPORT_NORESP;
    }
    e = &s->in[ep];
    type = ep_type(e);
    iso = ep && type == USB_ENDPOINT_XFER_ISOC;
    if (ep && !(e->ctl & DXEPCTL_USBACTEP)) {
        return USB_DEVPORT_NORESP;
    }
    if (ep == 0) {
        dev_setup_done(s);
    }
    if ((e->ctl & DXEPCTL_STALL) && !iso) {
        return USB_DEVPORT_STALL;
    }
    if (!(e->ctl & DXEPCTL_EPENA) || (e->ctl & DXEPCTL_NAKSTS) ||
        ((s->dctl & DCTL_GNPINNAKSTS) && type != USB_ENDPOINT_XFER_INT &&
         !iso)) {
        e->intr |= DXEPINT_NAKINTRPT;
        dwc2_otg_update_irq(s);
        return iso ? USB_DEVPORT_NORESP : USB_DEVPORT_NAK;
    }
    if (dwc2_otg_dma_enabled(s) && (s->dcfg & DCFG_DESCDMA_EN)) {
        qemu_log_mask(LOG_UNIMP, "%s: scatter/gather DMA in device mode is "
                      "not modelled\n", s->name);
        return USB_DEVPORT_NAK;
    }
    mps = MIN(ep_mps(s, e, ep), sizeof(data));
    size = ep_xfersize(e, ep);
    pkts = ep_pktcnt(e, ep, true);
    if (!pkts) {
        /* Enabled with nothing to send: the transfer is over. */
        ep_in_done(s, e);
        dwc2_otg_update_irq(s);
        return USB_DEVPORT_NAK;
    }
    if (dwc2_otg_dma_enabled(s)) {
        n = MIN(size, mps);
        if (!dwc2_otg_dma_read(s, e->dma, data, n)) {
            e->intr |= DXEPINT_AHBERR;
            e->ctl &= ~DXEPCTL_EPENA;
            dwc2_otg_update_irq(s);
            return USB_DEVPORT_NAK;
        }
        e->dma += n;
        size -= n;
    } else {
        fifo = (e->ctl & DXEPCTL_TXFNUM_MASK) >> DXEPCTL_TXFNUM_SHIFT;
        if (fifo >= s->num_in_fifos) {
            return USB_DEVPORT_NAK;
        }
        if (s->tx[fifo].npkts) {
            n = s->tx[fifo].pkt_len[0];
            dwc2_otg_tx_pop(s, fifo, data);
        } else if (size) {
            /* Its data is not in the FIFO yet. */
            if (type != USB_ENDPOINT_XFER_INT && !iso) {
                e->intr |= DXEPINT_INTKNTXFEMP;
            }
            e->intr |= DXEPINT_NAKINTRPT;
            dwc2_otg_update_irq(s);
            return iso ? USB_DEVPORT_NORESP : USB_DEVPORT_NAK;
        }
    }
    n = MIN(n, max);
    if (buf) {
        memcpy(buf, data, n);
    }
    *len = n;
    *pid = iso ? 0 : ep_toggle(s, e, ep, true);
    if (!iso) {
        ep_flip(s, e, ep, true);
    }
    pkts--;
    ep_set_xfer(e, ep, true, size, pkts);
    if (!pkts) {
        ep_in_done(s, e);
    }
    if (ep == 0 && !n && !s->ctrl_in && s->ctrl_active) {
        dev_ctrl_end(s);
    }
    dwc2_otg_update_irq(s);
    return USB_DEVPORT_ACK;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static USBDevPortHandshake devport_out(void *opaque, uint8_t addr,
                                       uint8_t ep, int pid,
                                       const uint8_t *data, uint16_t len)
{
    DWC2OTGState *s = opaque;
    DWC2OTGDevEp *e;
    uint32_t mps, size, pkts;
    bool iso, dma = dwc2_otg_dma_enabled(s);
    int type;

    if (!dev_answers(s, addr) || ep >= s->num_eps) {
        return USB_DEVPORT_NORESP;
    }
    e = &s->out[ep];
    type = ep_type(e);
    iso = ep && type == USB_ENDPOINT_XFER_ISOC;
    if (ep && !(e->ctl & DXEPCTL_USBACTEP)) {
        return USB_DEVPORT_NORESP;
    }
    if (ep == 0) {
        dev_setup_done(s);
        if (s->ctrl_in && len && (s->dcfg & DCFG_NZ_STS_OUT_HSHK)) {
            /* A status stage with data. */
            return USB_DEVPORT_STALL;
        }
    }
    if ((e->ctl & DXEPCTL_STALL) && !iso) {
        return USB_DEVPORT_STALL;
    }
    mps = ep_mps(s, e, ep);
    if (len > mps) {
        e->intr |= DXEPINT_BBLEERRINTRPT;
        dwc2_otg_update_irq(s);
        return USB_DEVPORT_NORESP;
    }
    if (!(e->ctl & DXEPCTL_EPENA) || (e->ctl & DXEPCTL_NAKSTS) ||
        (s->dctl & DCTL_GOUTNAKSTS)) {
        if (ep == 0 && !(e->ctl & DXEPCTL_EPENA)) {
            e->intr |= DXEPINT_OUTTKNEPDIS;
        }
        if (iso) {
            s->gintsts |= GINTSTS_ISOUTDROP;
        }
        e->intr |= DXEPINT_NAKINTRPT;
        dwc2_otg_update_irq(s);
        return iso ? USB_DEVPORT_NORESP : USB_DEVPORT_NAK;
    }
    if (dma && (s->dcfg & DCFG_DESCDMA_EN)) {
        qemu_log_mask(LOG_UNIMP, "%s: scatter/gather DMA in device mode is "
                      "not modelled\n", s->name);
        return USB_DEVPORT_NAK;
    }
    if (!iso && pid != ep_toggle(s, e, ep, false)) {
        /* A retransmission of data already taken. */
        return USB_DEVPORT_ACK;
    }
    if (!dma && dwc2_otg_rx_free(s) < 2 + DIV_ROUND_UP(len, 4)) {
        if (iso) {
            s->gintsts |= GINTSTS_ISOUTDROP;
            dwc2_otg_update_irq(s);
            return USB_DEVPORT_NORESP;
        }
        return USB_DEVPORT_NAK;
    }
    size = ep_xfersize(e, ep);
    pkts = ep_pktcnt(e, ep, false);
    if (dma) {
        if (!dwc2_otg_dma_write(s, e->dma, data, MIN(len, size))) {
            e->intr |= DXEPINT_AHBERR;
            e->ctl &= ~DXEPCTL_EPENA;
            dwc2_otg_update_irq(s);
            return USB_DEVPORT_NAK;
        }
        e->dma += MIN(len, size);
    } else {
        dwc2_otg_rx_push(s, (ep << GRXSTS_EPNUM_SHIFT) |
                         (len << GRXSTS_BYTECNT_SHIFT) |
                         ((pid ? 2 : 0) << GRXSTS_DPID_SHIFT) |
                         (GRXSTS_PKTSTS_OUTRX << GRXSTS_PKTSTS_SHIFT) |
                         ((s->dev_soffn & 0xf) << GRXSTS_FN_SHIFT_DEV),
                         data, len);
    }
    if (!iso) {
        ep_flip(s, e, ep, false);
    }
    if (ep == 0 && !len && s->ctrl_in && s->ctrl_active) {
        dev_ctrl_end(s);
    }
    size -= MIN(len, size);
    pkts -= pkts ? 1 : 0;
    ep_set_xfer(e, ep, false, size, pkts);
    if (!pkts || len < mps) {
        /* The transfer is complete: disabled and NAKing. */
        e->ctl &= ~DXEPCTL_EPENA;
        e->ctl |= DXEPCTL_NAKSTS;
        if (dma) {
            e->intr |= DXEPINT_XFERCOMPL;
        } else {
            dwc2_otg_rx_push(s, (ep << GRXSTS_EPNUM_SHIFT) |
                             (GRXSTS_PKTSTS_OUTDONE << GRXSTS_PKTSTS_SHIFT),
                             NULL, 0);
        }
    }
    dwc2_otg_update_irq(s);
    return USB_DEVPORT_ACK;
}

static const USBDevPortOps dwc2_otg_devport_ops = {
    .present = devport_present,
    .suspended = devport_suspended,
    .host_attached = devport_host_attached,
    .bus_reset = devport_bus_reset,
    .sof = devport_sof,
    .setup = devport_setup,
    .in = devport_in,
    .out = devport_out,
};

/* Registers */

/*
 * Slave IN: the application pushed a word for IN endpoint @ep into its
 * transmit FIFO; a packet is MIN(XferSize, MPS) bytes, and XferSize
 * counts down as the FIFO takes each.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_dev_tx_push(DWC2OTGState *s, int ep, uint32_t word)
{
    DWC2OTGDevEp *e = &s->in[ep];
    int fifo = (e->ctl & DXEPCTL_TXFNUM_MASK) >> DXEPCTL_TXFNUM_SHIFT;
    uint32_t len = MIN(ep_xfersize(e, ep), ep_mps(s, e, ep));

    if (fifo >= s->num_in_fifos) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: IN endpoint %d uses transmit "
                      "FIFO %d, which the core does not have\n", s->name, ep,
                      fifo);
        return;
    }
    if (!len) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: push to the FIFO of IN endpoint "
                      "%d, which has no data left to send\n", s->name, ep);
        return;
    }
    if (dwc2_otg_tx_push_packet(s, fifo, ep, &e->pushed, len, word)) {
        ep_set_xfer(e, ep, true, ep_xfersize(e, ep) - len,
                    ep_pktcnt(e, ep, true));
        usb_devport_kick(&s->devport);
    }
}

uint32_t dwc2_otg_dtxfsts(DWC2OTGState *s, int ep)
{
    return ep < s->num_in_fifos ? MIN(dwc2_otg_tx_free(s, ep), 0xffff) : 0;
}

uint32_t dwc2_otg_dsts_read(DWC2OTGState *s)
{
    uint32_t v = dev_enum_speed(s) << DSTS_ENUMSPD_SHIFT;

    if (s->dev_suspended) {
        v |= DSTS_SUSPSTS;
    }
    v |= (uint32_t)s->dev_soffn << DSTS_SOFFN_SHIFT;
    if (s->dev_present && !s->dev_bus_reset) {
        /* An idle full-speed bus is J: D+ high. */
        v |= dev_enum_speed(s) == DWC2_SPEED_LOW ? BIT(22) : BIT(23);
    }
    return v;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_dctl_write(DWC2OTGState *s, uint32_t val)
{
    uint32_t keep = s->dctl & (DCTL_GNPINNAKSTS | DCTL_GOUTNAKSTS);

    s->dctl = keep | (val & 0x0003e873);
    if (val & DCTL_TSTCTL_MASK) {
        qemu_log_mask(LOG_UNIMP, "%s: device test modes are not modelled\n",
                      s->name);
    }
    if (val & DCTL_SGNPINNAK) {
        s->dctl |= DCTL_GNPINNAKSTS;
        s->gintsts |= GINTSTS_GINNAKEFF;
    }
    if (val & DCTL_CGNPINNAK) {
        s->dctl &= ~DCTL_GNPINNAKSTS;
        s->gintsts &= ~GINTSTS_GINNAKEFF;
    }
    if (val & DCTL_SGOUTNAK) {
        s->dctl |= DCTL_GOUTNAKSTS;
        if (dwc2_otg_dma_enabled(s) || dwc2_otg_rx_free(s) < 1) {
            s->gintsts |= GINTSTS_GOUTNAKEFF;
        } else {
            dwc2_otg_rx_push(s, GRXSTS_PKTSTS_GLOBALOUTNAK <<
                             GRXSTS_PKTSTS_SHIFT, NULL, 0);
        }
    }
    if (val & DCTL_CGOUTNAK) {
        s->dctl &= ~DCTL_GOUTNAKSTS;
        s->gintsts &= ~GINTSTS_GOUTNAKEFF;
    }
    if ((val & DCTL_RMTWKUPSIG) && s->dev_suspended) {
        /* Remote wakeup: the host resumes the bus. */
        s->dev_suspended = false;
        qemu_set_irq(s->suspended_out, 0);
        dev_activity(s);
    }
    dwc2_otg_dev_update_present(s);
    usb_devport_kick(&s->devport);
    dwc2_otg_update_irq(s);
}

/* The control bits common to DIEPCTLn and DOEPCTLn. */
static void ep_ctl_write(DWC2OTGState *s, DWC2OTGDevEp *e, int ep, bool in,
                         uint32_t val)
{
    uint32_t rw;

    if (ep == 0) {
        rw = in ? (DIEPCTL0_MPS_MASK | DXEPCTL_TXFNUM_MASK) : DXEPCTL_SNP;
    } else {
        rw = DXEPCTL_MPS_MASK | DXEPCTL_USBACTEP | DXEPCTL_EPTYPE_MASK |
             DXEPCTL_STALL | (in ? DXEPCTL_TXFNUM_MASK : DXEPCTL_SNP);
    }
    e->ctl = (e->ctl & ~rw) | (val & rw);
    if (ep == 0 && (val & DXEPCTL_STALL)) {
        /* EP0's STALL is set by software and cleared by a SETUP. */
        e->ctl |= DXEPCTL_STALL;
    }
    if (val & DXEPCTL_SNAK) {
        e->ctl |= DXEPCTL_NAKSTS;
        if (in) {
            e->intr |= DXEPINT_INEPNAKEFF;
        }
    }
    if (val & DXEPCTL_CNAK) {
        e->ctl &= ~DXEPCTL_NAKSTS;
    }
    if (ep && (val & DXEPCTL_SETD0PID)) {
        e->ctl &= ~DXEPCTL_DPID_BIT;
    }
    if (ep && (val & DXEPCTL_SETD1PID)) {
        e->ctl |= DXEPCTL_DPID_BIT;
    }
    if ((val & DXEPCTL_EPDIS) && (e->ctl & DXEPCTL_EPENA) &&
        (in || ep != 0)) {
        e->ctl &= ~DXEPCTL_EPENA;
        e->intr |= DXEPINT_EPDISBLD;
    } else if ((val & DXEPCTL_EPENA) && !(e->ctl & DXEPCTL_EPENA)) {
        e->ctl |= DXEPCTL_EPENA;
        e->pushed = 0;
        if (s->dcfg & DCFG_DESCDMA_EN && dwc2_otg_dma_enabled(s)) {
            qemu_log_mask(LOG_UNIMP, "%s: scatter/gather DMA in device mode "
                          "is not modelled\n", s->name);
        }
        if (ep && ep_type(e) == USB_ENDPOINT_XFER_ISOC) {
            qemu_log_mask(LOG_UNIMP, "%s: the frame parity of isochronous "
                          "endpoints is not modelled\n", s->name);
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_diepctl_write(DWC2OTGState *s, int ep, uint32_t val)
{
    ep_ctl_write(s, &s->in[ep], ep, true, val);
    usb_devport_kick(&s->devport);
    dwc2_otg_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_doepctl_write(DWC2OTGState *s, int ep, uint32_t val)
{
    ep_ctl_write(s, &s->out[ep], ep, false, val);
    if (ep == 0) {
        dev_deliver_setup(s);
    }
    usb_devport_kick(&s->devport);
    dwc2_otg_update_irq(s);
}

void dwc2_otg_dev_mode_changed(DWC2OTGState *s)
{
    dev_setup_reset(s);
    dwc2_otg_dev_update_present(s);
}

void dwc2_otg_dev_reset(DWC2OTGState *s)
{
    timer_del(s->idle_timer);
    timer_del(s->eopf_timer);
    s->dev_present = false;
    s->dev_suspended = false;
    s->dev_bus_reset = false;
    s->dev_soffn = 0;
    s->dev_ep_toggle = 0;
    s->ctrl_in = false;
    s->ctrl_active = false;
    s->dev_addr = 0;
    dev_setup_reset(s);
    qemu_set_irq(s->suspended_out, 0);
    usb_devport_changed(&s->devport);
}

void dwc2_otg_dev_init(DWC2OTGState *s)
{
    usb_devport_init(&s->devport, s->name, &dwc2_otg_devport_ops, s);
    s->idle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dev_idle, s);
    s->eopf_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dev_eopf, s);
}

const VMStateDescription vmstate_dwc2_otg_ep = {
    .name = TYPE_DWC2_OTG "/ep",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctl, DWC2OTGDevEp),
        VMSTATE_UINT32(intr, DWC2OTGDevEp),
        VMSTATE_UINT32(tsiz, DWC2OTGDevEp),
        VMSTATE_UINT32(dma, DWC2OTGDevEp),
        VMSTATE_UINT32(dmab, DWC2OTGDevEp),
        VMSTATE_UINT16(pushed, DWC2OTGDevEp),
        VMSTATE_BOOL(desc_open, DWC2OTGDevEp),
        VMSTATE_UINT32(desc_status, DWC2OTGDevEp),
        VMSTATE_UINT32(desc_buf, DWC2OTGDevEp),
        VMSTATE_UINT32(desc_done, DWC2OTGDevEp),
        VMSTATE_UINT32(desc_addr, DWC2OTGDevEp),
        VMSTATE_END_OF_LIST()
    },
};
