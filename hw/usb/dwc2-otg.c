/*
 * Synopsys DesignWare Hi-Speed USB 2.0 On-The-Go controller (DWC2)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The DWC2 ("DWC_otg") core as SoCs integrate it: a dual-role USB 2.0
 * controller, host or device as its ID input or software decides, with a
 * UTMI+ high-speed PHY interface and/or a dedicated full-speed PHY, and an
 * internal DMA engine. The configuration (GSNPSID, GHWCFG1 to 4 and the
 * FIFO RAM's size) is a property of the instance and decides the number
 * of host channels and device endpoints, the PHYs, the DMA modes and the
 * request queue depths, as the core's synthesis parameters do. Register
 * behaviour follows Synopsys' DWC_otg databook as the Linux, TinyUSB and
 * ESP-IDF drivers document it (drivers/usb/dwc2/hw.h, which
 * hw/usb/dwc2-regs.h imports, gives the layout).
 *
 * This file has the registers, the interrupt hierarchy, the FIFO RAM and
 * the OTG logic; dwc2-otg-host.c has the host (the root port, frames and
 * channels) and dwc2-otg-dev.c the device (endpoints and the host at the
 * other end of the cable). Modelled:
 *
 *   - The CSRs with their reset values, access types and side effects,
 *     the core soft reset (GRSTCTL.CSftRst, with CSftRstDone from 4.20a,
 *     completing only while the PHY clock runs, as on silicon), the FIFO
 *     flushes and the frame counter reset; registers of the other mode
 *     are ignored and raise GINTSTS.ModeMis.
 *   - The current mode: ForceHstMode/ForceDevMode take effect 25 ms after
 *     they are written, as the databook gives the application to wait;
 *     otherwise the ID input decides (A-device: host).
 *   - The OTG signals: the session-valid inputs and their GOTGCTL
 *     overrides, B-session end (GOTGINT.SesEndDet), the connector ID
 *     (GOTGCTL.ConIDSts, GINTSTS.ConIDStsChng), and the pull-down,
 *     ID pull-up and VBUS drive outputs to the PHY.
 *   - The FIFO RAM, dynamically sized: the shared receive FIFO with its
 *     status entries (GRXSTSR/GRXSTSP), the host's non-periodic and
 *     periodic transmit FIFOs with their request queues, the device's
 *     dedicated transmit FIFOs, the push/pop windows and the direct
 *     access window, in slave mode; in DMA modes the DMA engine moves the
 *     data between memory and the bus itself.
 *   - Clock gating: PCGCCTL.StopPclk stops the PHY clock, GateHclk the
 *     register interface (only PCGCCTL answers).
 *
 * Not modelled, and logged as unimplemented: SRP and HNP signalling
 * (GOTGCTL.SesReq, HNPReq), ADP (GADPCTL), the ULPI vendor control
 * interface (GPVNDCTL), partial power-down (GPWRDN), test modes, and
 * DTHRCTL's thresholding, which only changes the core's internal
 * buffering.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "dwc2-otg-internal.h"

#define DWC2_MODE_SWITCH_NS     (25 * SCALE_MS)

/* GINTSTS bits that are write-1-to-clear. */
#define GINTSTS_W1C             0xf0f2fc0a
/* GINTSTS bits the device part keeps in s->gintsts. */
#define GINTSTS_DEV_LEVELS      (GINTSTS_GINNAKEFF | GINTSTS_GOUTNAKEFF)
#define GINTMSK_WRITABLE        0xf7fefcfe
#define GOTGCTL_WRITABLE        0x00109efe
#define GOTGCTL_VBVALOEN        BIT(2)
#define GOTGCTL_VBVALOVAL       BIT(3)
#define GOTGCTL_AVALOEN         BIT(4)
#define GOTGCTL_AVALOVAL        BIT(5)
#define GOTGCTL_BVALOEN         BIT(6)
#define GOTGCTL_BVALOVAL        BIT(7)
#define GOTGCTL_EHEN            BIT(12)
#define GOTGCTL_CURMOD          BIT(21)
#define GOTGINT_W1C             0x000e0304
#define GAHBCFG_WRITABLE        0x01e001bf
#define GUSBCFG_WRITABLE        0xf7feffbf
#define GUSBCFG_PRESERVED       (GUSBCFG_PHY_LP_CLK_SEL | GUSBCFG_DDRSEL | \
                                 GUSBCFG_PHYSEL | GUSBCFG_FSINTF | \
                                 GUSBCFG_ULPI_UTMI_SEL | GUSBCFG_PHYIF16 | \
                                 GUSBCFG_TXENDDELAY | \
                                 GUSBCFG_TERMSELDLPULSE | \
                                 GUSBCFG_ULPI_CLK_SUSP_M | \
                                 GUSBCFG_ULPI_AUTO_RES | GUSBCFG_ULPI_FS_LS)
#define GRSTCTL_SELF_CLEARING   (GRSTCTL_HSFTRST | GRSTCTL_FRMCNTRRST | \
                                 GRSTCTL_RXFFLSH | GRSTCTL_TXFFLSH)
#define GPVNDCTL_NEWREGREQ      BIT(25)
#define GPWRDN_WRITABLE         0x0005007b
#define GPWRDN_W1C              0x00828000
#define GPWRDN_IDDIG            BIT(21)
#define GPWRDN_BSESSVLD         BIT(22)
#define GADPCTL_WRITABLE        0x1ffffff
#define GADPCTL_ENABLES         (BIT(17) | BIT(18) | BIT(20))
#define HCFG_WRITABLE           0x8780ff87
#define HCFG_PRESERVED          (HCFG_FSLSPCLKSEL_MASK | HCFG_FSLSSUPP)
#define DCFG_WRITABLE           0xff80ffff
#define DIEPMSK_WRITABLE        0x237f
#define DOEPMSK_WRITABLE        0x737f
#define DTHRCTL_WRITABLE        0x0bff1fff
#define PCGCCTL_WRITABLE        (PCGCTL_STOPPCLK | PCGCTL_GATEHCLK | \
                                 PCGCTL_PWRCLMP | PCGCTL_RSTPDWNMODULE | \
                                 PCGCTL_RESETAFTSUSP)

/* Register ranges. */
#define A_HPTXFSIZ              0x100
#define A_DIEPTXF1              0x104
#define A_HOST_BASE             0x400
#define A_CHAN_BASE             0x500
#define A_CHAN_END              0x700
#define A_DEV_BASE              0x800
#define A_DIEPEACHMSK0          0x840
#define A_DOEPEACHMSK0          0x880
#define A_IN_EP_BASE            0x900
#define A_OUT_EP_BASE           0xb00
#define A_DEV_END               0xd00
#define A_PCGCCTL               0xe00

bool dwc2_otg_dma_enabled(DWC2OTGState *s)
{
    return s->has_dma && (s->gahbcfg & GAHBCFG_DMA_EN);
}

bool dwc2_otg_dma_read(DWC2OTGState *s, uint32_t addr, void *buf,
                       uint32_t len)
{
    return !len || address_space_read(s->dma_as, addr, s->dma_attrs, buf,
                                       len) == MEMTX_OK;
}

bool dwc2_otg_dma_write(DWC2OTGState *s, uint32_t addr, const void *buf,
                        uint32_t len)
{
    return !len || address_space_write(s->dma_as, addr, s->dma_attrs, buf,
                                       len) == MEMTX_OK;
}

void dwc2_otg_set_dma(DWC2OTGState *s, AddressSpace *as, MemTxAttrs attrs)
{
    s->dma_as = as;
    s->dma_attrs = attrs;
}

/*
 * The PHY clock the core runs from: the dedicated full-speed PHY's, which
 * a host talking to a low-speed device divides by 8 (HCFG.FSLSPclkSel at
 * 6 MHz), or the UTMI+ PHY's, which runs a 16-bit interface at half the
 * rate.
 */
uint64_t dwc2_otg_phy_hz(DWC2OTGState *s)
{
    uint64_t hz;

    if (s->pcgcctl & PCGCTL_STOPPCLK) {
        return 0;
    }
    if (s->gusbcfg & GUSBCFG_PHYSEL) {
        hz = clock_get_hz(s->fsphy_clk);
        if (s->host_mode && (s->hcfg & HCFG_FSLSPCLKSEL_MASK) ==
                            HCFG_FSLSPCLKSEL_6_MHZ) {
            hz /= 8;
        }
        return hz;
    }
    hz = clock_get_hz(s->utmi_clk);
    return (s->gusbcfg & GUSBCFG_PHYIF16) ? hz / 2 : hz;
}

int64_t dwc2_otg_phy_ticks_ns(DWC2OTGState *s, uint64_t ticks)
{
    uint64_t hz = dwc2_otg_phy_hz(s);

    return hz ? muldiv64(ticks, NANOSECONDS_PER_SECOND, hz) : 0;
}

/* FIFO RAM */

static uint32_t dwc2_ram_index(DWC2OTGState *s, uint32_t start, uint32_t i)
{
    return (start + i) % s->ram_words;
}

static uint32_t dwc2_rx_depth(DWC2OTGState *s)
{
    return MIN(s->grxfsiz & 0xffff, s->ram_words);
}

uint32_t dwc2_otg_rx_free(DWC2OTGState *s)
{
    uint32_t depth = dwc2_rx_depth(s);

    return depth > s->rx_used ? depth - s->rx_used : 0;
}

static void dwc2_rx_put(DWC2OTGState *s, uint32_t word)
{
    uint32_t depth = dwc2_rx_depth(s);

    if (!depth || s->rx_used >= depth) {
        return;
    }
    s->ram[dwc2_ram_index(s, 0, s->rx_wr)] = word;
    s->rx_wr = (s->rx_wr + 1) % depth;
    s->rx_used++;
}

/*
 * Push a status entry and its data into the receive FIFO; the caller has
 * checked there is room for 1 + ceil(len / 4) words.
 */
void dwc2_otg_rx_push(DWC2OTGState *s, uint32_t status, const uint8_t *data,
                      uint32_t len)
{
    uint32_t i;

    dwc2_rx_put(s, status);
    for (i = 0; i < len; i += 4) {
        uint8_t w[4] = { 0 };

        memcpy(w, data + i, MIN(4, len - i));
        dwc2_rx_put(s, ldl_le_p(w));
    }
    dwc2_otg_update_irq(s);
}

static uint32_t dwc2_rx_pop(DWC2OTGState *s, bool peek)
{
    uint32_t depth = dwc2_rx_depth(s), word;

    if (!s->rx_used || !depth) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of the empty receive FIFO\n",
                      s->name);
        return 0;
    }
    word = s->ram[dwc2_ram_index(s, 0, s->rx_rd % depth)];
    if (!peek) {
        s->rx_rd = (s->rx_rd + 1) % depth;
        s->rx_used--;
        /* Room in the FIFO may let a waiting transaction go. */
        if (s->host_mode) {
            dwc2_otg_host_kick(s);
        } else {
            dwc2_otg_dev_kick(s);
        }
    }
    return word;
}

void dwc2_otg_rx_flush(DWC2OTGState *s)
{
    s->rx_rd = s->rx_wr = s->rx_used = 0;
}

/*
 * A transmit FIFO's place in the RAM: host mode has the non-periodic FIFO
 * (GNPTXFSIZ) and the periodic one (HPTXFSIZ); device mode a dedicated
 * FIFO per IN endpoint, FIFO 0 in GNPTXFSIZ (DIEPTXF0) and the others in
 * DIEPTXFn.
 */
uint32_t dwc2_otg_tx_start(DWC2OTGState *s, int fifo)
{
    uint32_t reg;

    if (fifo == 0) {
        reg = s->gnptxfsiz;
    } else if (s->host_mode) {
        reg = s->hptxfsiz;
    } else {
        reg = s->dieptxf[fifo];
    }
    return reg & FIFOSIZE_STARTADDR_MASK;
}

uint32_t dwc2_otg_tx_depth(DWC2OTGState *s, int fifo)
{
    uint32_t reg;

    if (fifo == 0) {
        reg = s->gnptxfsiz;
    } else if (s->host_mode) {
        reg = s->hptxfsiz;
    } else {
        reg = s->dieptxf[fifo];
    }
    return MIN(FIFOSIZE_DEPTH_GET(reg), s->ram_words);
}

uint32_t dwc2_otg_tx_free(DWC2OTGState *s, int fifo)
{
    uint32_t depth = dwc2_otg_tx_depth(s, fifo);
    DWC2OTGTxFifo *f = &s->tx[fifo];

    if (f->npkts >= DWC2_OTG_TX_PACKETS) {
        return 0;
    }
    return depth > f->used ? depth - f->used : 0;
}

/* Push a word the application wrote; false if the FIFO is full. */
static bool dwc2_tx_put(DWC2OTGState *s, int fifo, uint32_t word)
{
    uint32_t depth = dwc2_otg_tx_depth(s, fifo);
    DWC2OTGTxFifo *f = &s->tx[fifo];

    if (!depth || f->used >= depth) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to the full transmit FIFO "
                      "%d\n", s->name, fifo);
        return false;
    }
    s->ram[dwc2_ram_index(s, dwc2_otg_tx_start(s, fifo), f->wr % depth)] =
        word;
    f->wr = (f->wr + 1) % depth;
    f->used++;
    return true;
}

/* Pop the oldest packet's data from a transmit FIFO into @buf. */
void dwc2_otg_tx_pop(DWC2OTGState *s, int fifo, uint8_t *buf)
{
    uint32_t depth = dwc2_otg_tx_depth(s, fifo);
    DWC2OTGTxFifo *f = &s->tx[fifo];
    uint32_t len, words, i;

    if (!f->npkts) {
        return;
    }
    len = f->pkt_len[0];
    words = DIV_ROUND_UP(len, 4);
    for (i = 0; i < words && f->used && depth; i++) {
        uint32_t w = s->ram[dwc2_ram_index(s, dwc2_otg_tx_start(s, fifo),
                                           f->rd % depth)];

        if (buf) {
            uint8_t b[4];

            stl_le_p(b, w);
            memcpy(buf + i * 4, b, MIN(4, len - i * 4));
        }
        f->rd = (f->rd + 1) % depth;
        f->used--;
    }
    f->npkts--;
    memmove(f->pkt_len, f->pkt_len + 1, f->npkts * sizeof(f->pkt_len[0]));
    memmove(f->pkt_chan, f->pkt_chan + 1, f->npkts);
}

void dwc2_otg_tx_flush(DWC2OTGState *s, int fifo)
{
    DWC2OTGTxFifo *f = &s->tx[fifo];

    f->rd = f->wr = f->used = 0;
    f->npkts = 0;
}

/*
 * The application pushed a word into the window of endpoint or channel
 * @n, which completes a packet once it holds the packet's bytes.
 */
static void dwc2_fifo_push(DWC2OTGState *s, int n, uint32_t word)
{
    if (s->host_mode) {
        if (n < s->num_chans) {
            dwc2_otg_host_tx_push(s, n, word);
            return;
        }
    } else if (n < s->num_eps) {
        dwc2_otg_dev_tx_push(s, n, word);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: push to the FIFO window of %s %d, "
                  "which the core does not have\n", s->name,
                  s->host_mode ? "channel" : "endpoint", n);
}

bool dwc2_otg_tx_push_packet(DWC2OTGState *s, int fifo, int chan,
                             uint16_t *pushed, uint32_t len, uint32_t word)
{
    DWC2OTGTxFifo *f = &s->tx[fifo];

    if (f->npkts >= DWC2_OTG_TX_PACKETS || !dwc2_tx_put(s, fifo, word)) {
        return false;
    }
    *pushed = MIN(*pushed + 4, len);
    if (*pushed < len) {
        return false;
    }
    f->pkt_len[f->npkts] = len;
    f->pkt_chan[f->npkts] = chan;
    f->npkts++;
    *pushed = 0;
    return true;
}

/* Interrupts */

static uint32_t dwc2_gintsts(DWC2OTGState *s)
{
    uint32_t v = s->gintsts & (GINTSTS_W1C | GINTSTS_DEV_LEVELS);

    if (s->gotgint) {
        v |= GINTSTS_OTGINT;
    }
    if (s->rx_used) {
        v |= GINTSTS_RXFLVL;
    }
    if (s->host_mode) {
        uint32_t lvl_np = s->gahbcfg & GAHBCFG_NP_TXF_EMP_LVL;
        uint32_t lvl_p = s->gahbcfg & GAHBCFG_P_TXF_EMP_LVL;
        uint32_t d;

        v &= ~GINTSTS_DEV_LEVELS;
        v |= GINTSTS_CURMODE_HOST;
        d = dwc2_otg_tx_depth(s, DWC2_NPTXF);
        if (lvl_np ? !s->tx[DWC2_NPTXF].used
                   : s->tx[DWC2_NPTXF].used * 2 <= d) {
            v |= GINTSTS_NPTXFEMP;
        }
        d = dwc2_otg_tx_depth(s, DWC2_PTXF);
        if (lvl_p ? !s->tx[DWC2_PTXF].used
                  : s->tx[DWC2_PTXF].used * 2 <= d) {
            v |= GINTSTS_PTXFEMP;
        }
        if (s->hprt & (HPRT0_CONNDET | HPRT0_ENACHG | HPRT0_OVRCURRCHG)) {
            v |= GINTSTS_PRTINT;
        }
        if (dwc2_otg_haint(s) & s->haintmsk) {
            v |= GINTSTS_HCHINT;
        }
    } else {
        uint32_t daint = dwc2_otg_daint(s) & s->daintmsk;

        v |= GINTSTS_NPTXFEMP | GINTSTS_PTXFEMP;
        if (daint & 0xffff) {
            v |= GINTSTS_IEPINT;
        }
        if (daint >> 16) {
            v |= GINTSTS_OEPINT;
        }
    }
    return v;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
void dwc2_otg_update_irq(DWC2OTGState *s)
{
    bool en = s->gahbcfg & GAHBCFG_GLBL_INTR_EN;

    qemu_set_irq(s->irq, en && (dwc2_gintsts(s) & s->gintmsk));
    if (s->multiproc) {
        qemu_set_irq(s->ep_irq,
                     en && !s->host_mode &&
                     (dwc2_otg_deachint(s) & s->deachintmsk));
    }
}

/* OTG */

static bool dwc2_bvalid(DWC2OTGState *s)
{
    if (s->gotgctl & GOTGCTL_BVALOEN) {
        return s->gotgctl & GOTGCTL_BVALOVAL;
    }
    return s->sense[DWC2_OTG_SENSE_BVALID];
}

static bool dwc2_avalid(DWC2OTGState *s)
{
    if (s->gotgctl & GOTGCTL_AVALOEN) {
        return s->gotgctl & GOTGCTL_AVALOVAL;
    }
    return s->sense[DWC2_OTG_SENSE_AVALID];
}

/* The mode the force bits or, without them, the ID input select. */
static bool dwc2_wanted_host(DWC2OTGState *s)
{
    if (s->gusbcfg & GUSBCFG_FORCEHOSTMODE) {
        return true;
    }
    if (s->gusbcfg & GUSBCFG_FORCEDEVMODE) {
        return false;
    }
    return !s->sense[DWC2_OTG_SENSE_IDDIG];
}

static void dwc2_update_otg_outputs(DWC2OTGState *s)
{
    bool host = s->host_mode;

    /*
     * The host's 15 kOhm pull-downs, and VBUS as HPRT0.PrtPwr drives it;
     * the ID pull-up samples the ID pin of an OTG-capable core.
     */
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_IDPULLUP], 1);
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_DPPULLDOWN], host);
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_DMPULLDOWN], host);
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_DRVVBUS],
                 host && (s->hprt & HPRT0_PWR));
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_CHRGVBUS], 0);
    qemu_set_irq(s->otg_ctl[DWC2_OTG_CTL_DISCHRGVBUS], 0);
}

static void dwc2_set_mode(DWC2OTGState *s, bool host)
{
    if (host == s->host_mode) {
        return;
    }
    s->host_mode = host;
    if (host) {
        dwc2_otg_dev_mode_changed(s);
    }
    dwc2_otg_host_mode_changed(s);
    if (!host) {
        dwc2_otg_dev_mode_changed(s);
    }
    dwc2_update_otg_outputs(s);
}

static void dwc2_mode_timer(void *opaque)
{
    DWC2OTGState *s = opaque;

    dwc2_set_mode(s, dwc2_wanted_host(s));
    dwc2_otg_dev_update_present(s);
    dwc2_otg_host_update_port(s);
    dwc2_otg_update_irq(s);
}

/*
 * The session-valid comparators: a B-device that loses its session (VBUS
 * falls) flags GOTGINT.SesEndDet.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void dwc2_update_session(DWC2OTGState *s)
{
    bool b = dwc2_bvalid(s), a = dwc2_avalid(s);

    if (s->bsesvld && !b && !s->host_mode) {
        s->gotgint |= GOTGINT_SES_END_DET;
    }
    s->bsesvld = b;
    s->asesvld = a;
    dwc2_otg_dev_update_present(s);
    dwc2_otg_update_irq(s);
}

static void dwc2_sense_in(void *opaque, int n, int level)
{
    DWC2OTGState *s = opaque;
    bool v = level;

    if (s->sense[n] == v) {
        return;
    }
    s->sense[n] = v;
    if (n == DWC2_OTG_SENSE_IDDIG) {
        s->gintsts |= GINTSTS_CONIDSTSCHNG;
        if (!(s->gusbcfg & (GUSBCFG_FORCEHOSTMODE | GUSBCFG_FORCEDEVMODE))) {
            dwc2_set_mode(s, dwc2_wanted_host(s));
        }
    }
    dwc2_update_session(s);
    dwc2_otg_host_update_port(s);
}

static void dwc2_phy_connected_in(void *opaque, int n, int level)
{
    DWC2OTGState *s = opaque;

    s->phy_connected = level;
    dwc2_otg_dev_update_present(s);
    dwc2_otg_host_update_port(s);
    dwc2_otg_update_irq(s);
}

static void dwc2_pull_override_in(void *opaque, int n, int level)
{
    DWC2OTGState *s = opaque;

    if (n == 0) {
        s->pull_override = level;
    } else {
        s->pull_dp = level;
    }
    dwc2_otg_dev_update_present(s);
}

/* Reset */

static void dwc2_reset_regs(DWC2OTGState *s)
{
    int i;

    s->gotgctl = 0;
    s->gotgint = 0;
    s->gahbcfg = 0;
    /* USBTrdTim 5; PHYSel reads 1 on a core with only the full-speed PHY. */
    s->gusbcfg = 5 << GUSBCFG_USBTRDTIM_SHIFT;
    if (!s->has_utmi) {
        s->gusbcfg |= GUSBCFG_PHYSEL;
    }
    s->grstctl = GRSTCTL_AHBIDLE;
    s->gintsts = 0;
    s->gintmsk = 0;
    /*
     * The FIFO sizes reset to the largest depths, each FIFO starting where
     * the last ends, as the core's parameters set them; this model takes
     * each largest depth as the whole RAM.
     */
    s->grxfsiz = s->ram_words;
    s->gnptxfsiz = (s->ram_words << FIFOSIZE_DEPTH_SHIFT) | s->ram_words;
    s->hptxfsiz = (s->ram_words << FIFOSIZE_DEPTH_SHIFT) |
                  (2 * s->ram_words);
    for (i = 1; i < DWC2_OTG_TX_FIFOS; i++) {
        s->dieptxf[i] = (s->ram_words << FIFOSIZE_DEPTH_SHIFT) |
                        ((i + 1) * s->ram_words);
    }
    s->gpvndctl = 0;
    s->ggpio = 0;
    s->guid = 0;
    s->gpwrdn = 0x10;
    s->gdfifocfg = (extract32(s->hwcfg3, 16, 16) << 16) |
                   extract32(s->hwcfg3, 16, 16);
    s->gadpctl = 0;
    s->hcfg = 0x200;
    s->hfir = 60000;
    s->hfir_written = false;
    s->haintmsk = 0;
    s->hflbaddr = 0;
    s->hprt = 0;
    memset(s->hc, 0, sizeof(s->hc));
    s->dcfg = 0x08000000;
    s->dctl = DCTL_SFTDISCON;
    s->diepmsk = 0;
    s->doepmsk = 0;
    s->daintmsk = 0;
    s->dvbusdis = 0x17d7;
    s->dvbuspulse = 0x5b8;
    s->dthrctl = 0x08100020;
    s->diepempmsk = 0;
    s->deachintmsk = 0;
    memset(s->diepeachmsk, 0, sizeof(s->diepeachmsk));
    memset(s->doepeachmsk, 0, sizeof(s->doepeachmsk));
    memset(s->in, 0, sizeof(s->in));
    memset(s->out, 0, sizeof(s->out));
    /* EP0 is always active. */
    s->in[0].ctl = DXEPCTL_USBACTEP;
    s->out[0].ctl = DXEPCTL_USBACTEP;
    s->pcgcctl = 0;
}

static void dwc2_reset_fifos(DWC2OTGState *s)
{
    int i;

    dwc2_otg_rx_flush(s);
    for (i = 0; i < DWC2_OTG_TX_FIFOS; i++) {
        dwc2_otg_tx_flush(s, i);
    }
    memset(s->reqq, 0, sizeof(s->reqq));
}

/*
 * GRSTCTL.CSftRst: every state machine, FIFO and register resets except
 * the PHY and clock selections and the few registers the databook keeps
 * (GGPIO, GPWRDN, GADPCTL, DCFG.DevSpd, DCTL.SftDiscon), and the mode
 * returns to what the ID input selects.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void dwc2_soft_reset(DWC2OTGState *s)
{
    uint32_t gusbcfg = s->gusbcfg & GUSBCFG_PRESERVED;
    uint32_t hcfg = s->hcfg & HCFG_PRESERVED;
    uint32_t pcgcctl = s->pcgcctl;
    uint32_t ggpio = s->ggpio, gpwrdn = s->gpwrdn, gadpctl = s->gadpctl;
    uint32_t devspd = s->dcfg & DCFG_DEVSPD_MASK;
    uint32_t sftdiscon = s->dctl & DCTL_SFTDISCON;
    uint32_t grstctl = s->grstctl;

    timer_del(s->mode_timer);
    dwc2_reset_regs(s);
    s->gusbcfg = (s->gusbcfg & ~GUSBCFG_PRESERVED) | gusbcfg;
    if (!s->has_utmi) {
        s->gusbcfg |= GUSBCFG_PHYSEL;
    }
    s->hcfg = (s->hcfg & ~HCFG_PRESERVED) | hcfg;
    s->pcgcctl = pcgcctl;
    s->ggpio = ggpio;
    s->gpwrdn = gpwrdn;
    s->gadpctl = gadpctl;
    s->dcfg = (s->dcfg & ~DCFG_DEVSPD_MASK) | devspd;
    s->dctl = (s->dctl & ~DCTL_SFTDISCON) | sftdiscon;
    dwc2_reset_fifos(s);
    dwc2_otg_host_reset(s);
    dwc2_otg_dev_reset(s);
    s->host_mode = dwc2_wanted_host(s);
    if (s->snpsid >= DWC2_SNPSID_4_20A) {
        /* From 4.20a the bit stays set until software clears it. */
        s->grstctl = (grstctl & GRSTCTL_CSFTRST) | GRSTCTL_CSFTRST_DONE |
                     GRSTCTL_AHBIDLE;
    }
    dwc2_update_session(s);
    dwc2_otg_dev_update_present(s);
    dwc2_otg_host_update_port(s);
    dwc2_update_otg_outputs(s);
    dwc2_otg_update_irq(s);
}

/* A soft reset completes once the PHY clock runs. */
static void dwc2_try_soft_reset(DWC2OTGState *s)
{
    if ((s->grstctl & GRSTCTL_CSFTRST) &&
        !(s->grstctl & GRSTCTL_CSFTRST_DONE) && dwc2_otg_phy_hz(s)) {
        dwc2_soft_reset(s);
    }
}

static void dwc2_phy_clk_changed(void *opaque, ClockEvent event)
{
    DWC2OTGState *s = opaque;

    dwc2_try_soft_reset(s);
    dwc2_otg_host_phy_changed(s);
    dwc2_otg_dev_update_present(s);
    dwc2_otg_update_irq(s);
}

/* Registers */

static void dwc2_grstctl_write(DWC2OTGState *s, uint32_t val)
{
    uint32_t txfnum = val & GRSTCTL_TXFNUM_MASK;
    int i;

    if ((val & GRSTCTL_CSFTRST_DONE) && s->snpsid >= DWC2_SNPSID_4_20A) {
        s->grstctl &= ~GRSTCTL_CSFTRST_DONE;
    }
    s->grstctl = (s->grstctl & ~(GRSTCTL_TXFNUM_MASK | GRSTCTL_CSFTRST)) |
                 txfnum | (val & GRSTCTL_CSFTRST) | GRSTCTL_AHBIDLE;
    if (val & GRSTCTL_CSFTRST) {
        if (s->snpsid >= DWC2_SNPSID_4_20A) {
            s->grstctl &= ~GRSTCTL_CSFTRST_DONE;
        }
        if (!dwc2_otg_phy_hz(s)) {
            /* Waits for the PHY clock. */
            return;
        }
        dwc2_soft_reset(s);
        if (s->snpsid < DWC2_SNPSID_4_20A) {
            s->grstctl &= ~GRSTCTL_CSFTRST;
        }
        return;
    }
    if (val & GRSTCTL_RXFFLSH) {
        dwc2_otg_rx_flush(s);
    }
    if (val & GRSTCTL_TXFFLSH) {
        int n = txfnum >> GRSTCTL_TXFNUM_SHIFT;

        if (n == 0x10) {
            for (i = 0; i < DWC2_OTG_TX_FIFOS; i++) {
                dwc2_otg_tx_flush(s, i);
            }
        } else if (n < DWC2_OTG_TX_FIFOS) {
            dwc2_otg_tx_flush(s, n);
        }
    }
    if ((val & GRSTCTL_FRMCNTRRST) && s->host_mode) {
        dwc2_otg_host_frame_reset(s);
    }
    dwc2_otg_update_irq(s);
}

static void dwc2_gusbcfg_write(DWC2OTGState *s, uint32_t val)
{
    uint32_t mask = GUSBCFG_WRITABLE;
    uint32_t mode = GUSBCFG_FORCEHOSTMODE | GUSBCFG_FORCEDEVMODE;
    uint32_t otgmode = extract32(s->hwcfg2, 0, 3);

    if (!s->has_utmi || !s->has_fsphy ||
        extract32(s->hwcfg2, 8, 2) != GHWCFG2_FS_PHY_TYPE_DEDICATED) {
        mask &= ~GUSBCFG_PHYSEL;
    }
    if (otgmode != GHWCFG2_OP_MODE_HNP_SRP_CAPABLE) {
        mask &= ~GUSBCFG_HNPCAP;
        if (otgmode != GHWCFG2_OP_MODE_SRP_ONLY_CAPABLE &&
            otgmode != GHWCFG2_OP_MODE_SRP_CAPABLE_DEVICE &&
            otgmode != GHWCFG2_OP_MODE_SRP_CAPABLE_HOST) {
            mask &= ~GUSBCFG_SRPCAP;
        }
    }
    if ((val & mode) == mode) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: GUSBCFG forces both host and "
                      "device mode\n", s->name);
    }
    if ((val ^ s->gusbcfg) & mode) {
        timer_mod(s->mode_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  DWC2_MODE_SWITCH_NS);
    }
    s->gusbcfg = (s->gusbcfg & ~mask) | (val & mask);
    dwc2_try_soft_reset(s);
    dwc2_otg_host_phy_changed(s);
}

/* Whether @addr is a register of the mode the core is not in. */
static bool dwc2_mode_mismatch(DWC2OTGState *s, hwaddr addr)
{
    if (addr >= A_HOST_BASE && addr < A_DEV_BASE) {
        return !s->host_mode;
    }
    if (addr >= A_DEV_BASE && addr < A_DEV_END) {
        return s->host_mode;
    }
    return false;
}

static uint32_t dwc2_chan_read(DWC2OTGState *s, int ch, int reg)
{
    DWC2OTGHostChan *c = &s->hc[ch];

    switch (reg) {
    case 0x00:
        return c->hcchar;
    case 0x04:
        return c->hcsplt;
    case 0x08:
        return c->hcint;
    case 0x0c:
        return c->hcintmsk;
    case 0x10:
        return c->hctsiz;
    case 0x14:
        return c->hcdma;
    case 0x1c:
        return c->hcdmab;
    default:
        return 0;
    }
}

static void dwc2_chan_write(DWC2OTGState *s, int ch, int reg, uint32_t val)
{
    DWC2OTGHostChan *c = &s->hc[ch];

    switch (reg) {
    case 0x00:
        dwc2_otg_hcchar_write(s, ch, val);
        return;
    case 0x04:
        c->hcsplt = val & 0x8001ffff;
        return;
    case 0x08:
        c->hcint &= ~(val & HCINT_ALL);
        break;
    case 0x0c:
        c->hcintmsk = val & HCINTMSK_WRITABLE;
        break;
    case 0x10:
        c->hctsiz = val;
        return;
    case 0x14:
        c->hcdma = val;
        return;
    default:
        return;
    }
    dwc2_otg_update_irq(s);
}

static uint32_t dwc2_ep_read(DWC2OTGState *s, bool in, int ep, int reg)
{
    DWC2OTGDevEp *e = in ? &s->in[ep] : &s->out[ep];

    switch (reg) {
    case 0x00:
        /* Control OUT endpoint 0's packet size is IN endpoint 0's. */
        return (!in && ep == 0) ? e->ctl | (s->in[0].ctl & 3) : e->ctl;
    case 0x08:
        return in ? dwc2_otg_diepint_read(s, ep) : e->intr;
    case 0x10:
        return e->tsiz;
    case 0x14:
        return e->dma;
    case 0x18:
        return in ? dwc2_otg_dtxfsts(s, ep) : 0;
    case 0x1c:
        return e->dmab;
    default:
        return 0;
    }
}

static void dwc2_ep_write(DWC2OTGState *s, bool in, int ep, int reg,
                          uint32_t val)
{
    DWC2OTGDevEp *e = in ? &s->in[ep] : &s->out[ep];

    switch (reg) {
    case 0x00:
        if (in) {
            dwc2_otg_diepctl_write(s, ep, val);
        } else {
            dwc2_otg_doepctl_write(s, ep, val);
        }
        return;
    case 0x08:
        e->intr &= ~(val & ~DXEPINT_TXFEMP);
        break;
    case 0x10:
        if (ep == 0) {
            e->tsiz = val & (in ? 0x0018007f : 0x6008007f);
        } else {
            e->tsiz = val & (in ? 0x7fffffff : 0x1fffffff);
        }
        return;
    case 0x14:
        e->dma = val;
        return;
    default:
        return;
    }
    dwc2_otg_update_irq(s);
}

static uint32_t dwc2_ghwcfg2(DWC2OTGState *s)
{
    return s->hwcfg2;
}

static uint32_t dwc2_reg_read(DWC2OTGState *s, hwaddr addr)
{
    uint32_t v;

    switch (addr) {
    case GOTGCTL:
        v = s->gotgctl & GOTGCTL_WRITABLE;
        if (s->sense[DWC2_OTG_SENSE_IDDIG]) {
            v |= GOTGCTL_CONID_B;
        }
        if (s->asesvld) {
            v |= GOTGCTL_ASESVLD;
        }
        if (s->bsesvld) {
            v |= GOTGCTL_BSESVLD;
        }
        if (s->host_mode) {
            v |= GOTGCTL_CURMOD;
        }
        return v;
    case GOTGINT:
        return s->gotgint;
    case GAHBCFG:
        return s->gahbcfg;
    case GUSBCFG:
        return s->gusbcfg;
    case GRSTCTL:
        return s->grstctl;
    case GINTSTS:
        return dwc2_gintsts(s);
    case GINTMSK:
        return s->gintmsk;
    case GRXSTSR:
    case GRXSTSP:
        if (!s->rx_used) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: GRXSTS%c read with the "
                          "receive FIFO empty\n", s->name,
                          addr == GRXSTSR ? 'R' : 'P');
            return 0;
        }
        v = dwc2_rx_pop(s, addr == GRXSTSR);
        if (addr == GRXSTSP) {
            if (s->host_mode) {
                dwc2_otg_host_rx_popped(s, v);
            } else {
                dwc2_otg_dev_rx_popped(s, v);
            }
            dwc2_otg_update_irq(s);
        }
        return v;
    case GRXFSIZ:
        return s->grxfsiz;
    case GNPTXFSIZ:
        return s->gnptxfsiz;
    case GNPTXSTS:
        return dwc2_otg_txsts(s, DWC2_NPTXF);
    case 0x34:
        return s->has_utmi ? s->gpvndctl : 0;
    case 0x38:
        return s->ggpio & 0xffff0000;
    case 0x3c:
        return s->guid;
    case GSNPSID:
        return s->snpsid;
    case GHWCFG1:
        return s->hwcfg1;
    case GHWCFG2:
        return dwc2_ghwcfg2(s);
    case GHWCFG3:
        return s->hwcfg3;
    case GHWCFG4:
        return s->hwcfg4;
    case 0x58:
        v = s->gpwrdn & (GPWRDN_WRITABLE | GPWRDN_W1C);
        if (s->sense[DWC2_OTG_SENSE_IDDIG]) {
            v |= GPWRDN_IDDIG;
        }
        if (s->bsesvld) {
            v |= GPWRDN_BSESSVLD;
        }
        return v;
    case GDFIFOCFG:
        return s->gdfifocfg;
    case 0x60:
        return s->gadpctl;
    case A_HPTXFSIZ:
        return s->hptxfsiz;
    case HCFG:
        return s->hcfg;
    case HFIR:
        return s->hfir;
    case HFNUM:
        return dwc2_otg_hfnum_read(s);
    case 0x410:
        return dwc2_otg_txsts(s, DWC2_PTXF);
    case HAINT:
        return dwc2_otg_haint(s);
    case HAINTMSK:
        return s->haintmsk;
    case 0x41c:
        return s->hflbaddr;
    case HPRT0:
        return dwc2_otg_hprt_read(s);
    case DCFG:
        return s->dcfg;
    case DCTL:
        return s->dctl;
    case DSTS:
        return dwc2_otg_dsts_read(s);
    case DIEPMSK:
        return s->diepmsk;
    case DOEPMSK:
        return s->doepmsk;
    case DAINT:
        return dwc2_otg_daint(s);
    case DAINTMSK:
        return s->daintmsk;
    case 0x828:
        return s->dvbusdis;
    case 0x82c:
        return s->dvbuspulse;
    case 0x830:
        return s->dthrctl;
    case DIEPEMPMSK:
        return s->diepempmsk;
    case 0x838:
        return s->multiproc ? dwc2_otg_deachint(s) : 0;
    case 0x83c:
        return s->multiproc ? s->deachintmsk : 0;
    case A_PCGCCTL:
        return s->pcgcctl;
    }
    if (addr > A_HPTXFSIZ && addr < A_HPTXFSIZ + 4 * DWC2_OTG_TX_FIFOS) {
        int n = (addr - A_HPTXFSIZ) / 4;

        return n < s->num_in_fifos ? s->dieptxf[n] : 0;
    }
    if (addr >= A_CHAN_BASE && addr < A_CHAN_END) {
        int ch = (addr - A_CHAN_BASE) / 0x20;

        return ch < s->num_chans ? dwc2_chan_read(s, ch, addr & 0x1f) : 0;
    }
    if (s->multiproc && addr >= A_DIEPEACHMSK0 &&
        addr < A_DIEPEACHMSK0 + 4 * DWC2_OTG_MAX_EPS) {
        return s->diepeachmsk[(addr - A_DIEPEACHMSK0) / 4];
    }
    if (s->multiproc && addr >= A_DOEPEACHMSK0 &&
        addr < A_DOEPEACHMSK0 + 4 * DWC2_OTG_MAX_EPS) {
        return s->doepeachmsk[(addr - A_DOEPEACHMSK0) / 4];
    }
    if (addr >= A_IN_EP_BASE && addr < A_DEV_END) {
        bool in = addr < A_OUT_EP_BASE;
        int ep = ((addr - A_IN_EP_BASE) & 0x1ff) / 0x20;

        return ep < s->num_eps ? dwc2_ep_read(s, in, ep, addr & 0x1f) : 0;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                  HWADDR_PRIx "\n", s->name, addr);
    return 0;
}

static void dwc2_reg_write(DWC2OTGState *s, hwaddr addr, uint32_t val)
{
    switch (addr) {
    case GOTGCTL:
        if (val & (GOTGCTL_SESREQ | GOTGCTL_HNPREQ)) {
            qemu_log_mask(LOG_UNIMP, "%s: SRP and HNP signalling "
                          "(GOTGCTL.SesReq, HNPReq) are not modelled\n",
                          s->name);
        }
        s->gotgctl = val & GOTGCTL_WRITABLE;
        dwc2_update_session(s);
        dwc2_otg_host_update_port(s);
        return;
    case GOTGINT:
        s->gotgint &= ~(val & GOTGINT_W1C);
        break;
    case GAHBCFG:
        s->gahbcfg = val & GAHBCFG_WRITABLE;
        if (!s->has_dma) {
            s->gahbcfg &= ~GAHBCFG_DMA_EN;
        }
        dwc2_otg_host_kick(s);
        dwc2_otg_dev_kick(s);
        break;
    case GUSBCFG:
        dwc2_gusbcfg_write(s, val);
        break;
    case GRSTCTL:
        dwc2_grstctl_write(s, val);
        return;
    case GINTSTS:
        s->gintsts &= ~(val & GINTSTS_W1C);
        break;
    case GINTMSK:
        s->gintmsk = val & GINTMSK_WRITABLE;
        break;
    case GRXSTSR:
    case GRXSTSP:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only GRXSTS%c\n",
                      s->name, addr == GRXSTSR ? 'R' : 'P');
        return;
    case GRXFSIZ:
        s->grxfsiz = val & 0xffff;
        break;
    case GNPTXFSIZ:
        s->gnptxfsiz = val;
        break;
    case 0x34:
        if (!s->has_utmi) {
            return;
        }
        s->gpvndctl = val & ~(BIT(26) | BIT(27));
        if (val & GPVNDCTL_NEWREGREQ) {
            qemu_log_mask(LOG_UNIMP, "%s: the ULPI vendor control interface "
                          "is not modelled: the core has no ULPI PHY\n",
                          s->name);
        }
        return;
    case 0x38:
        s->ggpio = val & 0xffff0000;
        return;
    case 0x3c:
        s->guid = val;
        return;
    case 0x58:
        s->gpwrdn = (s->gpwrdn & ~(val & GPWRDN_W1C) & ~GPWRDN_WRITABLE) |
                    (val & GPWRDN_WRITABLE);
        if (val & 0x23) {
            qemu_log_mask(LOG_UNIMP, "%s: partial power-down (GPWRDN) is "
                          "not modelled\n", s->name);
        }
        return;
    case GDFIFOCFG:
        s->gdfifocfg = val;
        return;
    case 0x60:
        s->gadpctl = val & GADPCTL_WRITABLE;
        if (val & GADPCTL_ENABLES) {
            qemu_log_mask(LOG_UNIMP, "%s: attach detection (GADPCTL) is not "
                          "modelled\n", s->name);
        }
        return;
    case A_HPTXFSIZ:
        s->hptxfsiz = val;
        return;
    case HCFG:
        if (!s->has_desc_dma) {
            val &= ~HCFG_DESCDMA;
        }
        s->hcfg = val & HCFG_WRITABLE;
        dwc2_otg_host_phy_changed(s);
        return;
    case HFIR:
        s->hfir = val & 0x1ffff;
        s->hfir_written = true;
        return;
    case HAINTMSK:
        s->haintmsk = val & MAKE_64BIT_MASK(0, s->num_chans);
        break;
    case 0x41c:
        s->hflbaddr = val;
        return;
    case HPRT0:
        dwc2_otg_hprt_write(s, val);
        return;
    case DCFG:
        if (!s->has_desc_dma) {
            val &= ~DCFG_DESCDMA_EN;
        }
        s->dcfg = val & DCFG_WRITABLE;
        dwc2_otg_dcfg_written(s);
        return;
    case DCTL:
        dwc2_otg_dctl_write(s, val);
        return;
    case DIEPMSK:
        s->diepmsk = val & DIEPMSK_WRITABLE;
        break;
    case DOEPMSK:
        s->doepmsk = val & DOEPMSK_WRITABLE;
        break;
    case DAINTMSK:
        s->daintmsk = val;
        break;
    case 0x828:
        s->dvbusdis = val & 0xffff;
        return;
    case 0x82c:
        s->dvbuspulse = val & 0xfff;
        return;
    case 0x830:
        s->dthrctl = val & DTHRCTL_WRITABLE;
        return;
    case DIEPEMPMSK:
        s->diepempmsk = val & 0xffff;
        break;
    case 0x83c:
        if (s->multiproc) {
            s->deachintmsk = val;
        }
        break;
    case A_PCGCCTL:
        s->pcgcctl = val & PCGCCTL_WRITABLE;
        dwc2_phy_clk_changed(s, ClockUpdate);
        return;
    case GSNPSID:
    case GHWCFG1:
    case GHWCFG2:
    case GHWCFG3:
    case GHWCFG4:
    case GNPTXSTS:
    case HFNUM:
    case 0x410:
    case HAINT:
    case DSTS:
    case DAINT:
    case 0x838:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", s->name, addr);
        return;
    default:
        if (addr > A_HPTXFSIZ && addr < A_HPTXFSIZ + 4 * DWC2_OTG_TX_FIFOS) {
            int n = (addr - A_HPTXFSIZ) / 4;

            if (n < s->num_in_fifos) {
                s->dieptxf[n] = val;
            }
            return;
        }
        if (addr >= A_CHAN_BASE && addr < A_CHAN_END) {
            int ch = (addr - A_CHAN_BASE) / 0x20;

            if (ch < s->num_chans) {
                dwc2_chan_write(s, ch, addr & 0x1f, val);
            }
            return;
        }
        if (s->multiproc && addr >= A_DIEPEACHMSK0 &&
            addr < A_DIEPEACHMSK0 + 4 * DWC2_OTG_MAX_EPS) {
            s->diepeachmsk[(addr - A_DIEPEACHMSK0) / 4] =
                val & DIEPMSK_WRITABLE;
            break;
        }
        if (s->multiproc && addr >= A_DOEPEACHMSK0 &&
            addr < A_DOEPEACHMSK0 + 4 * DWC2_OTG_MAX_EPS) {
            s->doepeachmsk[(addr - A_DOEPEACHMSK0) / 4] =
                val & DOEPMSK_WRITABLE;
            break;
        }
        if (addr >= A_IN_EP_BASE && addr < A_DEV_END) {
            bool in = addr < A_OUT_EP_BASE;
            int ep = ((addr - A_IN_EP_BASE) & 0x1ff) / 0x20;

            if (ep < s->num_eps) {
                dwc2_ep_write(s, in, ep, addr & 0x1f, val);
            }
            return;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx " (value 0x%" PRIx32 ")\n", s->name, addr,
                      val);
        return;
    }
    dwc2_otg_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static uint64_t dwc2_csr_read(void *opaque, hwaddr addr, unsigned size)
{
    DWC2OTGState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    if ((s->pcgcctl & PCGCTL_GATEHCLK) && reg != A_PCGCCTL) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%" HWADDR_PRIx
                      " with the AHB clock gated (PCGCCTL.GateHclk)\n",
                      s->name, addr);
        return 0;
    }
    if (dwc2_mode_mismatch(s, reg)) {
        s->gintsts |= GINTSTS_MODEMIS;
        dwc2_otg_update_irq(s);
        return 0;
    }
    v = dwc2_reg_read(s, reg);
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void dwc2_csr_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned size)
{
    DWC2OTGState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t val = data;

    if ((s->pcgcctl & PCGCTL_GATEHCLK) && reg != A_PCGCCTL) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " with the AHB clock gated (PCGCCTL.GateHclk)\n",
                      s->name, addr);
        return;
    }
    if (dwc2_mode_mismatch(s, reg)) {
        s->gintsts |= GINTSTS_MODEMIS;
        dwc2_otg_update_irq(s);
        return;
    }
    if (size < 4) {
        /*
         * The registers are 32 bits wide and drivers access them whole;
         * a narrower write updates its bytes of the register.
         */
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %u-byte write at 0x%"
                      HWADDR_PRIx "\n", s->name, size, addr);
        if (reg == GRXSTSR || reg == GRXSTSP) {
            return;
        }
        val = deposit32(dwc2_reg_read(s, reg), (addr & 3) * 8, size * 8,
                        val);
    }
    dwc2_reg_write(s, reg, val);
}

static const MemoryRegionOps dwc2_csr_ops = {
    .read = dwc2_csr_read,
    .write = dwc2_csr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * The FIFO windows: reading any push/pop window pops the receive FIFO,
 * writing window n pushes into the transmit FIFO of endpoint or channel
 * n; the direct access window reads and writes the RAM.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static uint64_t dwc2_fifo_read(void *opaque, hwaddr addr, unsigned size)
{
    DWC2OTGState *s = opaque;
    uint32_t v;

    if (s->pcgcctl & PCGCTL_GATEHCLK) {
        return 0;
    }
    if (addr >= DWC2_OTG_DFIFO_OFFSET) {
        uint32_t i = (addr - DWC2_OTG_DFIFO_OFFSET) / 4;

        if (i >= s->ram_words) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: direct FIFO access beyond "
                          "the RAM at 0x%" HWADDR_PRIx "\n", s->name, addr);
            return 0;
        }
        return extract32(s->ram[i], (addr & 3) * 8, size * 8);
    }
    if (addr >= DWC2_OTG_MAX_EPS * 0x1000) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved FIFO offset 0x%"
                      HWADDR_PRIx "\n", s->name, addr);
        return 0;
    }
    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %u-byte read of a FIFO window\n",
                      s->name, size);
    }
    v = dwc2_rx_pop(s, false);
    dwc2_otg_update_irq(s);
    return v;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void dwc2_fifo_write(void *opaque, hwaddr addr, uint64_t data,
                            unsigned size)
{
    DWC2OTGState *s = opaque;

    if (s->pcgcctl & PCGCTL_GATEHCLK) {
        return;
    }
    if (addr >= DWC2_OTG_DFIFO_OFFSET) {
        uint32_t i = (addr - DWC2_OTG_DFIFO_OFFSET) / 4;

        if (i >= s->ram_words) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: direct FIFO access beyond "
                          "the RAM at 0x%" HWADDR_PRIx "\n", s->name, addr);
            return;
        }
        s->ram[i] = deposit32(s->ram[i], (addr & 3) * 8, size * 8, data);
        return;
    }
    if (addr >= DWC2_OTG_MAX_EPS * 0x1000) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved FIFO offset 0x%"
                      HWADDR_PRIx "\n", s->name, addr);
        return;
    }
    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %u-byte write to a FIFO window\n",
                      s->name, size);
    }
    dwc2_fifo_push(s, addr >> 12, data);
    dwc2_otg_update_irq(s);
}

static const MemoryRegionOps dwc2_fifo_ops = {
    .read = dwc2_fifo_read,
    .write = dwc2_fifo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* QOM */

static void dwc2_otg_reset_hold(Object *obj, ResetType type)
{
    DWC2OTGState *s = DWC2_OTG(obj);

    timer_del(s->mode_timer);
    dwc2_reset_regs(s);
    dwc2_reset_fifos(s);
    dwc2_otg_host_reset(s);
    dwc2_otg_dev_reset(s);
    s->host_mode = dwc2_wanted_host(s);
    s->bsesvld = dwc2_bvalid(s);
    s->asesvld = dwc2_avalid(s);
}

static void dwc2_otg_reset_exit(Object *obj, ResetType type)
{
    DWC2OTGState *s = DWC2_OTG(obj);

    s->dev_host_attached = usb_devport_host_attached(&s->devport);
    qemu_set_irq(s->host_attached_out, s->dev_host_attached);
    dwc2_otg_dev_update_present(s);
    dwc2_otg_host_update_port(s);
    dwc2_update_otg_outputs(s);
    dwc2_otg_update_irq(s);
}

static void dwc2_otg_init(Object *obj)
{
    DWC2OTGState *s = DWC2_OTG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->csr_mr, obj, &dwc2_csr_ops, s, "dwc2-otg.csr",
                          DWC2_OTG_CSR_SIZE);
    memory_region_init_io(&s->fifo_mr, obj, &dwc2_fifo_ops, s,
                          "dwc2-otg.fifo", DWC2_OTG_FIFO_SIZE);
    sysbus_init_mmio(sbd, &s->csr_mr);
    sysbus_init_mmio(sbd, &s->fifo_mr);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->ep_irq);
    qdev_init_gpio_in_named(dev, dwc2_sense_in, DWC2_OTG_SENSE,
                            DWC2_OTG_SENSE_NUM);
    qdev_init_gpio_in_named(dev, dwc2_phy_connected_in,
                            DWC2_OTG_PHY_CONNECTED, 1);
    qdev_init_gpio_in_named(dev, dwc2_pull_override_in,
                            DWC2_OTG_PULL_OVERRIDE, 2);
    qdev_init_gpio_out_named(dev, s->otg_ctl, DWC2_OTG_CTL, DWC2_OTG_CTL_NUM);
    qdev_init_gpio_out_named(dev, &s->suspended_out, DWC2_OTG_SUSPENDED, 1);
    qdev_init_gpio_out_named(dev, &s->host_attached_out,
                             DWC2_OTG_HOST_ATTACHED, 1);
    qdev_init_gpio_out_named(dev, &s->dp_pullup_out, DWC2_OTG_DP_PULLUP, 1);
    s->utmi_clk = qdev_init_clock_in(dev, "utmi", dwc2_phy_clk_changed, s,
                                     ClockUpdate);
    s->fsphy_clk = qdev_init_clock_in(dev, "fsphy", dwc2_phy_clk_changed, s,
                                      ClockUpdate);
    s->dma_as = &address_space_memory;
    s->dma_attrs = MEMTXATTRS_UNSPECIFIED;
    /* Unconnected inputs: a B-device, the PHY on the cable. */
    s->sense[DWC2_OTG_SENSE_IDDIG] = true;
    s->sense[DWC2_OTG_SENSE_SESSEND] = true;
    s->phy_connected = true;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void dwc2_otg_realize(DeviceState *dev, Error **errp)
{
    DWC2OTGState *s = DWC2_OTG(dev);
    static const uint8_t qdepth[4] = { 2, 4, 8, 8 };

    if (!s->name) {
        s->name = g_strdup(TYPE_DWC2_OTG);
    }
    if (!s->ram_words || s->ram_words > DWC2_OTG_MAX_RAM_WORDS) {
        error_setg(errp, "%s: ram-words must be 1 to %u", s->name,
                   DWC2_OTG_MAX_RAM_WORDS);
        return;
    }
    s->num_chans = extract32(s->hwcfg2, 14, 4) + 1;
    s->num_eps = extract32(s->hwcfg2, 10, 4) + 1;
    s->num_in_fifos = extract32(s->hwcfg4, 26, 4) + 1;
    s->nptxq_depth = qdepth[extract32(s->hwcfg2, 22, 2)];
    s->ptxq_depth = qdepth[extract32(s->hwcfg2, 24, 2)];
    s->has_utmi = extract32(s->hwcfg2, 6, 2) !=
                  GHWCFG2_HS_PHY_TYPE_NOT_SUPPORTED;
    s->has_fsphy = extract32(s->hwcfg2, 8, 2) !=
                   GHWCFG2_FS_PHY_TYPE_NOT_SUPPORTED;
    s->has_dma = extract32(s->hwcfg2, 3, 2) == GHWCFG2_INT_DMA_ARCH;
    s->has_desc_dma = s->has_dma && (s->hwcfg4 & GHWCFG4_DESC_DMA);
    s->multiproc = s->hwcfg2 & GHWCFG2_MULTI_PROC_INT;
    s->mode_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dwc2_mode_timer, s);
    dwc2_otg_host_init(s);
    dwc2_otg_dev_init(s);
    usb_devport_realize(&s->devport, errp);
}

static int dwc2_otg_pre_save(void *opaque)
{
    DWC2OTGState *s = opaque;

    dwc2_otg_host_pre_save(s);
    return 0;
}

static const VMStateDescription vmstate_dwc2_otg_txfifo = {
    .name = TYPE_DWC2_OTG "/txfifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(rd, DWC2OTGTxFifo),
        VMSTATE_UINT16(wr, DWC2OTGTxFifo),
        VMSTATE_UINT16(used, DWC2OTGTxFifo),
        VMSTATE_UINT8(npkts, DWC2OTGTxFifo),
        VMSTATE_UINT16_ARRAY(pkt_len, DWC2OTGTxFifo, DWC2_OTG_TX_PACKETS),
        VMSTATE_UINT8_ARRAY(pkt_chan, DWC2OTGTxFifo, DWC2_OTG_TX_PACKETS),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_dwc2_otg_reqq = {
    .name = TYPE_DWC2_OTG "/reqq",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(n, DWC2OTGReqQueue),
        VMSTATE_UINT8_ARRAY(chan, DWC2OTGReqQueue, DWC2_OTG_REQ_DEPTH),
        VMSTATE_UINT8_ARRAY(type, DWC2OTGReqQueue, DWC2_OTG_REQ_DEPTH),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_dwc2_otg = {
    .name = TYPE_DWC2_OTG,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = dwc2_otg_pre_save,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(gotgctl, DWC2OTGState),
        VMSTATE_UINT32(gotgint, DWC2OTGState),
        VMSTATE_UINT32(gahbcfg, DWC2OTGState),
        VMSTATE_UINT32(gusbcfg, DWC2OTGState),
        VMSTATE_UINT32(grstctl, DWC2OTGState),
        VMSTATE_UINT32(gintsts, DWC2OTGState),
        VMSTATE_UINT32(gintmsk, DWC2OTGState),
        VMSTATE_UINT32(grxfsiz, DWC2OTGState),
        VMSTATE_UINT32(gnptxfsiz, DWC2OTGState),
        VMSTATE_UINT32(gpvndctl, DWC2OTGState),
        VMSTATE_UINT32(ggpio, DWC2OTGState),
        VMSTATE_UINT32(guid, DWC2OTGState),
        VMSTATE_UINT32(gpwrdn, DWC2OTGState),
        VMSTATE_UINT32(gdfifocfg, DWC2OTGState),
        VMSTATE_UINT32(gadpctl, DWC2OTGState),
        VMSTATE_UINT32(hptxfsiz, DWC2OTGState),
        VMSTATE_UINT32_ARRAY(dieptxf, DWC2OTGState, DWC2_OTG_TX_FIFOS),
        VMSTATE_UINT32(hcfg, DWC2OTGState),
        VMSTATE_UINT32(hfir, DWC2OTGState),
        VMSTATE_BOOL(hfir_written, DWC2OTGState),
        VMSTATE_UINT32(haintmsk, DWC2OTGState),
        VMSTATE_UINT32(hflbaddr, DWC2OTGState),
        VMSTATE_UINT32(hprt, DWC2OTGState),
        VMSTATE_STRUCT_ARRAY(hc, DWC2OTGState, DWC2_OTG_MAX_CHANNELS, 1,
                             vmstate_dwc2_otg_hc, DWC2OTGHostChan),
        VMSTATE_UINT32(dcfg, DWC2OTGState),
        VMSTATE_UINT32(dctl, DWC2OTGState),
        VMSTATE_UINT32(diepmsk, DWC2OTGState),
        VMSTATE_UINT32(doepmsk, DWC2OTGState),
        VMSTATE_UINT32(daintmsk, DWC2OTGState),
        VMSTATE_UINT32(dvbusdis, DWC2OTGState),
        VMSTATE_UINT32(dvbuspulse, DWC2OTGState),
        VMSTATE_UINT32(dthrctl, DWC2OTGState),
        VMSTATE_UINT32(diepempmsk, DWC2OTGState),
        VMSTATE_UINT32(deachintmsk, DWC2OTGState),
        VMSTATE_UINT32_ARRAY(diepeachmsk, DWC2OTGState, DWC2_OTG_MAX_EPS),
        VMSTATE_UINT32_ARRAY(doepeachmsk, DWC2OTGState, DWC2_OTG_MAX_EPS),
        VMSTATE_STRUCT_ARRAY(in, DWC2OTGState, DWC2_OTG_MAX_EPS, 1,
                             vmstate_dwc2_otg_ep, DWC2OTGDevEp),
        VMSTATE_STRUCT_ARRAY(out, DWC2OTGState, DWC2_OTG_MAX_EPS, 1,
                             vmstate_dwc2_otg_ep, DWC2OTGDevEp),
        VMSTATE_UINT32(pcgcctl, DWC2OTGState),
        VMSTATE_UINT32_ARRAY(ram, DWC2OTGState, DWC2_OTG_MAX_RAM_WORDS),
        VMSTATE_UINT16(rx_rd, DWC2OTGState),
        VMSTATE_UINT16(rx_wr, DWC2OTGState),
        VMSTATE_UINT16(rx_used, DWC2OTGState),
        VMSTATE_STRUCT_ARRAY(tx, DWC2OTGState, DWC2_OTG_TX_FIFOS, 1,
                             vmstate_dwc2_otg_txfifo, DWC2OTGTxFifo),
        VMSTATE_STRUCT_ARRAY(reqq, DWC2OTGState, 2, 1,
                             vmstate_dwc2_otg_reqq, DWC2OTGReqQueue),
        VMSTATE_BOOL_ARRAY(sense, DWC2OTGState, DWC2_OTG_SENSE_NUM),
        VMSTATE_BOOL(phy_connected, DWC2OTGState),
        VMSTATE_BOOL(pull_override, DWC2OTGState),
        VMSTATE_BOOL(pull_dp, DWC2OTGState),
        VMSTATE_BOOL(host_mode, DWC2OTGState),
        VMSTATE_TIMER_PTR(mode_timer, DWC2OTGState),
        VMSTATE_BOOL(bsesvld, DWC2OTGState),
        VMSTATE_BOOL(asesvld, DWC2OTGState),
        VMSTATE_BOOL(port_connected, DWC2OTGState),
        VMSTATE_UINT8(port_speed, DWC2OTGState),
        VMSTATE_TIMER_PTR(frame_timer, DWC2OTGState),
        VMSTATE_TIMER_PTR(bus_timer, DWC2OTGState),
        VMSTATE_INT64(frame_start_ns, DWC2OTGState),
        VMSTATE_INT64(frame_ns, DWC2OTGState),
        VMSTATE_UINT16(frnum, DWC2OTGState),
        VMSTATE_BOOL(xact_busy, DWC2OTGState),
        VMSTATE_UINT8(xact_chan, DWC2OTGState),
        VMSTATE_UINT8(xact_token, DWC2OTGState),
        VMSTATE_UINT16(xact_len, DWC2OTGState),
        VMSTATE_UINT8(xact_result, DWC2OTGState),
        VMSTATE_UINT16(xact_actual, DWC2OTGState),
        VMSTATE_UINT8(np_next, DWC2OTGState),
        VMSTATE_UINT32_ARRAY(dev_toggle, DWC2OTGState, 128),
        VMSTATE_BOOL(ctrl_stall, DWC2OTGState),
        VMSTATE_UINT8(ctrl_stall_addr, DWC2OTGState),
        VMSTATE_USB_DEVPORT(devport, DWC2OTGState),
        VMSTATE_BOOL(dev_present, DWC2OTGState),
        VMSTATE_BOOL(dev_suspended, DWC2OTGState),
        VMSTATE_BOOL(dev_bus_reset, DWC2OTGState),
        VMSTATE_BOOL(dev_host_attached, DWC2OTGState),
        VMSTATE_TIMER_PTR(idle_timer, DWC2OTGState),
        VMSTATE_TIMER_PTR(eopf_timer, DWC2OTGState),
        VMSTATE_UINT16(dev_soffn, DWC2OTGState),
        VMSTATE_BOOL(setup_pending, DWC2OTGState),
        VMSTATE_BOOL(ctrl_in, DWC2OTGState),
        VMSTATE_BOOL(ctrl_active, DWC2OTGState),
        VMSTATE_UINT8(dev_addr, DWC2OTGState),
        VMSTATE_UINT8(setup_bufs, DWC2OTGState),
        VMSTATE_UINT8_2DARRAY(setup_buf, DWC2OTGState, DWC2_OTG_SETUP_BUFS,
                              8),
        VMSTATE_UINT32(dev_ep_toggle, DWC2OTGState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property dwc2_otg_properties[] = {
    DEFINE_PROP_STRING("name", DWC2OTGState, name),
    DEFINE_PROP_UINT32("snpsid", DWC2OTGState, snpsid, 0x4f54400a),
    DEFINE_PROP_UINT32("hwcfg1", DWC2OTGState, hwcfg1, 0),
    DEFINE_PROP_UINT32("hwcfg2", DWC2OTGState, hwcfg2, 0x224dd930),
    DEFINE_PROP_UINT32("hwcfg3", DWC2OTGState, hwcfg3, 0x00c804b5),
    DEFINE_PROP_UINT32("hwcfg4", DWC2OTGState, hwcfg4, 0xd3f0a030),
    DEFINE_PROP_UINT32("ram-words", DWC2OTGState, ram_words, 256),
    DEFINE_PROP_CHR("cdc-chardev", DWC2OTGState, devport.cdc_chr),
    DEFINE_PROP_CHR("token-chardev", DWC2OTGState, devport.token_chr),
};

static void dwc2_otg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Synopsys DesignWare USB 2.0 OTG controller";
    dc->realize = dwc2_otg_realize;
    dc->vmsd = &vmstate_dwc2_otg;
    rc->phases.hold = dwc2_otg_reset_hold;
    rc->phases.exit = dwc2_otg_reset_exit;
    device_class_set_props(dc, dwc2_otg_properties);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static const TypeInfo dwc2_otg_types[] = {
    {
        .name          = TYPE_DWC2_OTG,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(DWC2OTGState),
        .instance_init = dwc2_otg_init,
        .class_init    = dwc2_otg_class_init,
    },
};

DEFINE_TYPES(dwc2_otg_types)
