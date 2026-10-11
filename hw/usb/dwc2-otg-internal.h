/*
 * Synopsys DWC2 OTG controller: what the core, host and device parts share
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_DWC2_OTG_INTERNAL_H
#define HW_USB_DWC2_OTG_INTERNAL_H

#include "hw/usb/dwc2-otg.h"
#include "hw/usb/dwc2-regs.h"

/* GSNPSID of the first core whose soft reset needs CSftRstDone. */
#define DWC2_SNPSID_4_20A           0x4f54420a

#define GRSTCTL_CSFTRST_DONE        BIT(29)

/* Fields dwc2-regs.h lacks or defines otherwise. */
#define GRXSTS_FN_SHIFT_DEV         21
#define GRXSTS_FN_MASK_DEV          (0xf << 21)
#define HCINT_ALL                   0x3fff
#define HCINTMSK_WRITABLE           0x2fff
#define HCINT_XCS_XACT_ERR          BIT(12)
#define HCINT_DESC_LST_ROLL         BIT(13)
#define DXEPINT_BNA                 BIT(9)
/*
 * HCDMA in scatter/gather mode: the list's base and the current
 * descriptor, 6 bits of it for a list of up to 64 descriptors on a 512-byte
 * boundary, 8 bits for a high-speed isochronous list of up to 256 on a
 * 2 KiB one.
 */
#define HCDMA_CTD_SHIFT             3
#define HCDMA_CTD_MASK              (0x3f << 3)
#define HCDMA_BASE_MASK             0xfffffe00
#define HCDMA_CTD_MASK_256          (0xff << 3)
#define HCDMA_BASE_MASK_256         0xfffff800

/* Request queue entry types (GNPTXSTS.NPTxQTop's token). */
enum {
    DWC2_REQ_INOUT = 0,
    DWC2_REQ_ZLP = 1,
    DWC2_REQ_HALT = 3,
};

/* The host's tokens for one transaction. */
enum {
    DWC2_TOKEN_SETUP,
    DWC2_TOKEN_IN,
    DWC2_TOKEN_OUT,
};

/* HPRT0.PrtSpd and DSTS.EnumSpd. */
#define DWC2_SPEED_HIGH             0
#define DWC2_SPEED_FULL             1
#define DWC2_SPEED_LOW              2
#define DWC2_SPEED_FULL48           3

/* The transmit FIFOs host mode uses. */
#define DWC2_NPTXF                  0
#define DWC2_PTXF                   1

/* Core */
void dwc2_otg_update_irq(DWC2OTGState *s);
/*
 * The PHY clock the core runs at: its period in Clock units, as its Clock
 * has it, 0 while it is stopped; the first ns by which @ticks of it have
 * elapsed from one of its edges, 0 while stopped; and its whole cycles in
 * @ns from one, at the period the class counts it at
 * (DWC2OTGClass.clock_cycles).
 */
uint64_t dwc2_otg_phy_period(DWC2OTGState *s);
int64_t dwc2_otg_phy_ticks_ns(DWC2OTGState *s, uint64_t ticks);
uint64_t dwc2_otg_phy_ticks(DWC2OTGState *s, int64_t ns);
bool dwc2_otg_dma_enabled(DWC2OTGState *s);
bool dwc2_otg_dma_read(DWC2OTGState *s, uint32_t addr, void *buf,
                       uint32_t len);
bool dwc2_otg_dma_write(DWC2OTGState *s, uint32_t addr, const void *buf,
                        uint32_t len);

/* FIFOs */
uint32_t dwc2_otg_rx_free(DWC2OTGState *s);
void dwc2_otg_rx_push(DWC2OTGState *s, uint32_t status, const uint8_t *data,
                      uint32_t len);
uint32_t dwc2_otg_tx_start(DWC2OTGState *s, int fifo);
uint32_t dwc2_otg_tx_depth(DWC2OTGState *s, int fifo);
uint32_t dwc2_otg_tx_free(DWC2OTGState *s, int fifo);
void dwc2_otg_tx_pop(DWC2OTGState *s, int fifo, uint8_t *buf);
void dwc2_otg_tx_flush(DWC2OTGState *s, int fifo);
void dwc2_otg_rx_flush(DWC2OTGState *s);
/*
 * Push @word into transmit FIFO @fifo for a packet @len bytes long on
 * channel or endpoint @chan; *@pushed counts the packet's bytes so far.
 * True when the word completes the packet.
 */
bool dwc2_otg_tx_push_packet(DWC2OTGState *s, int fifo, int chan,
                             uint16_t *pushed, uint32_t len, uint32_t word);

/* Host */
void dwc2_otg_host_init(DWC2OTGState *s);
void dwc2_otg_host_reset(DWC2OTGState *s);
void dwc2_otg_host_mode_changed(DWC2OTGState *s);
void dwc2_otg_host_update_port(DWC2OTGState *s);
void dwc2_otg_host_phy_changed(DWC2OTGState *s);
uint32_t dwc2_otg_hprt_read(DWC2OTGState *s);
void dwc2_otg_hprt_write(DWC2OTGState *s, uint32_t val);
uint32_t dwc2_otg_hfnum_read(DWC2OTGState *s);
void dwc2_otg_hcchar_write(DWC2OTGState *s, int ch, uint32_t val);
void dwc2_otg_host_tx_push(DWC2OTGState *s, int ch, uint32_t word);
uint32_t dwc2_otg_txsts(DWC2OTGState *s, int fifo);
void dwc2_otg_host_rx_popped(DWC2OTGState *s, uint32_t status);
void dwc2_otg_host_kick(DWC2OTGState *s);
void dwc2_otg_host_frame_reset(DWC2OTGState *s);
uint32_t dwc2_otg_haint(DWC2OTGState *s);
void dwc2_otg_host_pre_save(DWC2OTGState *s);
extern const VMStateDescription vmstate_dwc2_otg_hc;

/* Device */
void dwc2_otg_dev_init(DWC2OTGState *s);
void dwc2_otg_dev_reset(DWC2OTGState *s);
void dwc2_otg_dev_mode_changed(DWC2OTGState *s);
void dwc2_otg_dev_update_present(DWC2OTGState *s);
void dwc2_otg_dctl_write(DWC2OTGState *s, uint32_t val);
void dwc2_otg_dcfg_written(DWC2OTGState *s);
uint32_t dwc2_otg_dsts_read(DWC2OTGState *s);
void dwc2_otg_diepctl_write(DWC2OTGState *s, int ep, uint32_t val);
void dwc2_otg_doepctl_write(DWC2OTGState *s, int ep, uint32_t val);
uint32_t dwc2_otg_diepint_read(DWC2OTGState *s, int ep);
uint32_t dwc2_otg_dtxfsts(DWC2OTGState *s, int ep);
void dwc2_otg_dev_tx_push(DWC2OTGState *s, int ep, uint32_t word);
void dwc2_otg_dev_rx_popped(DWC2OTGState *s, uint32_t status);
uint32_t dwc2_otg_daint(DWC2OTGState *s);
uint32_t dwc2_otg_deachint(DWC2OTGState *s);
void dwc2_otg_dev_kick(DWC2OTGState *s);
extern const VMStateDescription vmstate_dwc2_otg_ep;

#endif
