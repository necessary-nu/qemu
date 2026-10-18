/*
 * Synopsys DesignWare Hi-Speed USB 2.0 On-The-Go controller (DWC2), host
 * and device modes
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_DWC2_OTG_H
#define HW_USB_DWC2_OTG_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/usb/usb.h"
#include "hw/usb/usb-devport.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_DWC2_OTG "dwc2-otg"
OBJECT_DECLARE_TYPE(DWC2OTGState, DWC2OTGClass, DWC2_OTG)

struct DWC2OTGClass {
    SysBusDeviceClass parent_class;

    /*
     * The whole cycles of @clk, one of the core's PHY clocks, in @ns from
     * one of its edges, and the first ns by which @n of them have elapsed:
     * by default at the Clock's period; a machine that knows the clock's
     * period more exactly than a Clock holds it counts at that. And of
     * its edges: those after @from by @t, and the first ns by which @n of
     * them have come after @from; by default as if it had one at @from.
     */
    uint64_t (*clock_cycles)(DWC2OTGState *s, Clock *clk, int64_t ns);
    int64_t (*clock_cycles_ns)(DWC2OTGState *s, Clock *clk, uint64_t n);
    uint64_t (*clock_count)(DWC2OTGState *s, Clock *clk, int64_t from,
                            int64_t t);
    int64_t (*clock_after)(DWC2OTGState *s, Clock *clk, int64_t from,
                           uint64_t n);
};

/*
 * The core's 256 KiB: its control and status registers (CSRs) in the
 * first 4 KiB, MMIO region DWC2_OTG_MMIO_CSR; the FIFOs above them,
 * MMIO region DWC2_OTG_MMIO_FIFO: a 4 KiB push/pop window for each
 * endpoint or channel from 0x1000, and the FIFO RAM itself for direct
 * access at 0x20000. Reading the FIFO windows and GRXSTSP pops the
 * receive FIFO.
 */
#define DWC2_OTG_MMIO_CSR       0
#define DWC2_OTG_MMIO_FIFO      1
#define DWC2_OTG_CSR_SIZE       0x1000
#define DWC2_OTG_FIFO_BASE      0x1000
#define DWC2_OTG_FIFO_SIZE      0x3f000
#define DWC2_OTG_DFIFO_OFFSET   0x1f000
#define DWC2_OTG_SIZE           0x40000
/* GRXSTSP, the receive status register a read pops. */
#define DWC2_OTG_GRXSTSP        0x20

#define DWC2_OTG_MAX_CHANNELS   16
#define DWC2_OTG_MAX_EPS        16
#define DWC2_OTG_MAX_RAM_WORDS  4096
/* Transmit FIFOs: host mode uses 0 (non-periodic) and 1 (periodic). */
#define DWC2_OTG_TX_FIFOS       16
#define DWC2_OTG_TX_PACKETS     32
#define DWC2_OTG_REQ_DEPTH      8
#define DWC2_OTG_SETUP_BUFS     3

/*
 * Named GPIO inputs:
 *
 * "otg-sense" (DWC2_OTG_SENSE_*): the UTMI+ OTG signals from the PHY,
 *   iddig (1: mini-B, the device role), avalid, bvalid, vbusvalid and
 *   sessend. Unconnected, they read as a B-device with no session.
 * "phy-connected": the PHY drives the cable (its pads are enabled and
 *   routed to it). Unconnected, it does.
 * "pull-override": [0] the PHY's D+/D- pulls are overridden, [1] the
 *   override's D+ pull-up: with the override on, it rather than the
 *   core's device-mode pull-up decides whether a host sees a device.
 *
 * Named GPIO outputs:
 *
 * "otg-ctl" (DWC2_OTG_CTL_*): the UTMI+ OTG controls to the PHY:
 *   idpullup, dppulldown, dmpulldown, drvvbus, chrgvbus, dischrgvbus.
 * "suspended": high while the core sees the bus suspended (device mode)
 *   or has suspended its port (host mode).
 * "host-attached": high while a host is at the other end of the device
 *   port's cable (usb-devport.h), which supplies VBUS to the device.
 * "dp-pullup": the core's D+ pull-up, on in device mode with a B-session
 *   and DCTL.SftDiscon clear, for a PHY whose pads show it.
 *
 * Sysbus IRQs: 0 the core's interrupt, 1 the endpoint interrupt of a core
 * with multi-processor interrupts (GHWCFG2.MultiProcIntrpt).
 *
 * Clock inputs: "utmi", the UTMI+ PHY's clock at 60 MHz (the core runs
 * the 16-bit interface, GUSBCFG.PHYIf, at half of it); "fsphy", the
 * dedicated full-speed PHY's 48 MHz. GUSBCFG.PHYSel picks one; the core's
 * bus and frame timing follows it, and it does nothing on the USB while
 * its PHY's clock is stopped.
 */
#define DWC2_OTG_SENSE          "otg-sense"
#define DWC2_OTG_PHY_CONNECTED  "phy-connected"
#define DWC2_OTG_PULL_OVERRIDE  "pull-override"
#define DWC2_OTG_CTL            "otg-ctl"
#define DWC2_OTG_SUSPENDED      "suspended"
#define DWC2_OTG_HOST_ATTACHED  "host-attached"
#define DWC2_OTG_DP_PULLUP      "dp-pullup"

enum {
    DWC2_OTG_SENSE_IDDIG,
    DWC2_OTG_SENSE_AVALID,
    DWC2_OTG_SENSE_BVALID,
    DWC2_OTG_SENSE_VBUSVALID,
    DWC2_OTG_SENSE_SESSEND,
    DWC2_OTG_SENSE_NUM,
};

enum {
    DWC2_OTG_CTL_IDPULLUP,
    DWC2_OTG_CTL_DPPULLDOWN,
    DWC2_OTG_CTL_DMPULLDOWN,
    DWC2_OTG_CTL_DRVVBUS,
    DWC2_OTG_CTL_CHRGVBUS,
    DWC2_OTG_CTL_DISCHRGVBUS,
    DWC2_OTG_CTL_NUM,
};

typedef struct DWC2OTGHostChan {
    uint32_t hcchar;
    uint32_t hcsplt;
    uint32_t hcint;
    uint32_t hcintmsk;
    uint32_t hctsiz;
    uint32_t hcdma;
    uint32_t hcdmab;
    /* Consecutive transaction errors of the transfer. */
    uint8_t err_cnt;
    /* A periodic channel has had its transaction in this (micro)frame. */
    bool frame_done;
    /* NAKed: the channel waits for the next (micro)frame. */
    bool nak_wait;
    /* Slave mode: the application asked the channel to halt. */
    bool halt_req;
    /* Slave mode: IN requests queued for the channel. */
    uint8_t in_reqs;
    /* Slave mode: bytes of the packet being pushed into the TX FIFO. */
    uint16_t pushed;
    /* Scatter/gather: the descriptor in progress has been fetched. */
    bool qtd_open;
    uint32_t qtd_status;
    uint32_t qtd_buf;
    uint32_t qtd_done;
} DWC2OTGHostChan;

typedef struct DWC2OTGDevEp {
    uint32_t ctl;
    uint32_t intr;
    uint32_t tsiz;
    uint32_t dma;
    uint32_t dmab;
    /* Slave IN: bytes of the packet being pushed into the TX FIFO. */
    uint16_t pushed;
    /* Scatter/gather: the descriptor in progress has been fetched. */
    bool desc_open;
    uint32_t desc_status;
    uint32_t desc_buf;
    uint32_t desc_done;
    uint32_t desc_addr;
} DWC2OTGDevEp;

/* A transmit FIFO's state: a ring in the FIFO RAM and its packets. */
typedef struct DWC2OTGTxFifo {
    uint16_t rd;
    uint16_t wr;
    uint16_t used;
    uint8_t npkts;
    uint16_t pkt_len[DWC2_OTG_TX_PACKETS];
    uint8_t pkt_chan[DWC2_OTG_TX_PACKETS];
} DWC2OTGTxFifo;

/* A host request queue's entries: the channel and what it asks for. */
typedef struct DWC2OTGReqQueue {
    uint8_t n;
    uint8_t chan[DWC2_OTG_REQ_DEPTH];
    uint8_t type[DWC2_OTG_REQ_DEPTH];
} DWC2OTGReqQueue;

struct DWC2OTGState {
    SysBusDevice parent_obj;

    MemoryRegion csr_mr;
    MemoryRegion fifo_mr;
    qemu_irq irq;
    qemu_irq ep_irq;
    qemu_irq otg_ctl[DWC2_OTG_CTL_NUM];
    qemu_irq suspended_out;
    qemu_irq host_attached_out;
    qemu_irq dp_pullup_out;
    Clock *utmi_clk;
    Clock *fsphy_clk;

    /* The DMA engine's view of memory and the attributes of its accesses. */
    AddressSpace *dma_as;
    MemTxAttrs dma_attrs;
    /*
     * The integration's bus, which may hold the DMA engine's accesses
     * (dwc2_otg_set_dma_wait()), and the timer that goes on with what
     * waited once dwc2_otg_dma_ready() says it takes them.
     */
    bool (*dma_wait)(void *opaque, uint32_t addr, uint32_t len, bool write);
    void *dma_wait_opaque;
    QEMUTimer *dma_retry_timer;

    /* Configuration: the core's identification and hardware parameters. */
    uint32_t snpsid;
    uint32_t hwcfg1;
    uint32_t hwcfg2;
    uint32_t hwcfg3;
    uint32_t hwcfg4;
    /* The FIFO RAM, in words, the transmit FIFOs' largest depths too. */
    uint32_t ram_words;
    char *name;

    /* Derived from the configuration. */
    uint8_t num_chans;
    uint8_t num_eps;
    uint8_t num_in_fifos;
    uint8_t nptxq_depth;
    uint8_t ptxq_depth;
    bool has_utmi;
    bool has_fsphy;
    bool has_dma;
    bool has_desc_dma;
    bool multiproc;

    /* Registers, as stored; read-only and computed fields on read. */
    uint32_t gotgctl;
    uint32_t gotgint;
    uint32_t gahbcfg;
    uint32_t gusbcfg;
    uint32_t grstctl;
    uint32_t gintsts;
    uint32_t gintmsk;
    uint32_t grxfsiz;
    uint32_t gnptxfsiz;
    uint32_t gpvndctl;
    uint32_t ggpio;
    uint32_t guid;
    uint32_t gpwrdn;
    uint32_t gdfifocfg;
    uint32_t gadpctl;
    uint32_t hptxfsiz;
    uint32_t dieptxf[DWC2_OTG_TX_FIFOS];
    uint32_t hcfg;
    uint32_t hfir;
    /* Software has programmed HFIR since the last reset. */
    bool hfir_written;
    uint32_t haintmsk;
    uint32_t hflbaddr;
    uint32_t hprt;
    DWC2OTGHostChan hc[DWC2_OTG_MAX_CHANNELS];
    uint32_t dcfg;
    uint32_t dctl;
    uint32_t diepmsk;
    uint32_t doepmsk;
    uint32_t daintmsk;
    uint32_t dvbusdis;
    uint32_t dvbuspulse;
    uint32_t dthrctl;
    uint32_t diepempmsk;
    uint32_t deachintmsk;
    uint32_t diepeachmsk[DWC2_OTG_MAX_EPS];
    uint32_t doepeachmsk[DWC2_OTG_MAX_EPS];
    DWC2OTGDevEp in[DWC2_OTG_MAX_EPS];
    DWC2OTGDevEp out[DWC2_OTG_MAX_EPS];
    uint32_t pcgcctl;

    /* The FIFO RAM, and the receive and transmit FIFOs in it. */
    uint32_t ram[DWC2_OTG_MAX_RAM_WORDS];
    uint16_t rx_rd;
    uint16_t rx_wr;
    uint16_t rx_used;
    DWC2OTGTxFifo tx[DWC2_OTG_TX_FIFOS];
    /* Host request queues, non-periodic and periodic. */
    DWC2OTGReqQueue reqq[2];

    /* OTG inputs, and the mode the core is in. */
    bool sense[DWC2_OTG_SENSE_NUM];
    bool phy_connected;
    bool pull_override;
    bool pull_dp;
    bool host_mode;
    QEMUTimer *mode_timer;
    int64_t mode_due;
    bool bsesvld;
    bool asesvld;

    /* Host port and frames */
    USBBus bus;
    USBPort port;
    USBPacket packet;
    uint8_t xfer_buf[1024];
    bool port_connected;
    uint8_t port_speed;
    QEMUTimer *frame_timer;
    QEMUTimer *bus_timer;
    int64_t bus_due;
    int64_t frame_start_ns;
    int64_t frame_ns;
    /*
     * The frames counted from frame_origin_ns at the PHY clock of
     * frame_period (its Clock's period, as dwc2_otg_phy_period() gives it),
     * the current one ending frame_ticks PHY clocks after it.
     */
    int64_t frame_origin_ns;
    uint64_t frame_ticks;
    uint64_t frame_period;
    uint16_t frnum;
    /* The transaction in flight while a QEMU device completes it. */
    bool xact_async;
    bool xact_busy;
    uint8_t xact_chan;
    uint8_t xact_token;
    uint16_t xact_len;
    /* The device's answer, which the channel sees as the packet ends. */
    uint8_t xact_result;
    uint16_t xact_actual;
    /* The next channel the non-periodic schedule serves. */
    uint8_t np_next;
    /* Data toggles of the devices on the port: bit 2n IN, 2n+1 OUT of EP n. */
    uint32_t dev_toggle[128];
    /* A device stalled a SETUP's request: its next data or status. */
    bool ctrl_stall;
    uint8_t ctrl_stall_addr;

    /* Device controller */
    USBDevPort devport;
    bool dev_present;
    bool dev_suspended;
    bool dev_bus_reset;
    bool dev_host_attached;
    QEMUTimer *idle_timer;
    QEMUTimer *eopf_timer;
    int64_t idle_due;
    int64_t eopf_due;
    uint16_t dev_soffn;
    /* A SETUP's phase has not yet ended with a data or status token. */
    bool setup_pending;
    /* The setup stage's direction (IN data stage), for the status stage. */
    bool ctrl_in;
    /* A control transfer runs from its SETUP to its status stage. */
    bool ctrl_active;
    /* Scatter/gather: StsPhseRcvd has flagged this control transfer's. */
    bool sts_phase;
    /*
     * The address the device answers: DCFG.DevAddr, taken once the
     * control transfer in progress (SET_ADDRESS's) has ended.
     */
    uint8_t dev_addr;
    /* SETUPs waiting for control OUT endpoint 0 in DMA mode. */
    uint8_t setup_bufs;
    uint8_t setup_buf[DWC2_OTG_SETUP_BUFS][8];
    /* Bit n: OUT endpoint n's data toggle; bit 16 + n: IN endpoint n's. */
    uint32_t dev_ep_toggle;
};

/*
 * The DMA engine's AddressSpace and the attributes it gives every access;
 * by default, system memory without attributes. Call before realize.
 */
void dwc2_otg_set_dma(DWC2OTGState *s, AddressSpace *as, MemTxAttrs attrs);

/*
 * The integration's bus may hold the DMA engine's accesses: @fn(@opaque,
 * addr, len, write) says whether one waits, as one a memory controller's
 * held port takes does. The engine then makes none of the step it is in:
 * a host channel's transaction neither starts nor ends, the bus waiting
 * with it; a device endpoint NAKs the host's token, as with its FIFO
 * empty or full; a SETUP waits in the core. It goes on once the
 * integration calls dwc2_otg_dma_ready(). Call before realize.
 */
void dwc2_otg_set_dma_wait(DWC2OTGState *s,
                           bool (*fn)(void *opaque, uint32_t addr,
                                      uint32_t len, bool write),
                           void *opaque);
void dwc2_otg_dma_ready(DWC2OTGState *s);

#endif
