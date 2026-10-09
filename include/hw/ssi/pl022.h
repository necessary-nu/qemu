/*
 * ARM PrimeCell PL022 Synchronous Serial Port
 *
 * Copyright (c) 2007 CodeSourcery.
 * Written by Paul Brook
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

/*
 * This is a model of the Arm PrimeCell PL022 synchronous serial port.
 * The PL022 TRM is:
 * https://developer.arm.com/documentation/ddi0194/latest
 *
 * QEMU interface:
 * + sysbus IRQ: SSPINTR combined interrupt line
 * + sysbus MMIO region 0: MemoryRegion for the device's registers
 * + SSI bus "ssi" (or the "bus-name" property): the serial interface in
 *   master mode. Each frame is one ssi_transfer() of the frame's data.
 * + Clock input "clk": SSPCLK. Connected and running, it times frames
 *   from SSPCPSR and SSPCR0.SCR in virtual time, so BSY, FIFO levels and
 *   the receive timeout follow the line rate and an unread receive FIFO
 *   overruns. Left unconnected, frames complete as soon as they are
 *   written and the transmitter stalls rather than overrun the receive
 *   FIFO, which boards that model no SSPCLK rely on.
 * + Named GPIO output array "dma-req": SSPTXDMASREQ and SSPRXDMASREQ.
 * + Named GPIO output array "ssp-out": the serial interface signals
 *   SSPCLKOUT, SSPFSSOUT, SSPTXD, nSSPCTLOE and nSSPOE at frame level
 *   (see below).
 * + Named GPIO input "sspfssin": SSPFSSIN, the frame/slave select input in
 *   slave mode. Unconnected, it is low: the slave is selected.
 *
 * Slave mode is reached through a TYPE_PL022_TARGET device on some
 * master's SSI bus, linked to the PL022 by its "controller" property.
 */

#ifndef HW_SSI_PL022_H
#define HW_SSI_PL022_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_PL022 "pl022"
OBJECT_DECLARE_SIMPLE_TYPE(PL022State, PL022)

#define TYPE_PL022_TARGET "pl022-target"
OBJECT_DECLARE_SIMPLE_TYPE(PL022TargetState, PL022_TARGET)

#define PL022_FIFO_DEPTH 8

/* Named GPIO output array: the single-transfer DMA requests. */
#define PL022_DMA_REQ "dma-req"
#define PL022_DMA_TX 0
#define PL022_DMA_RX 1

/*
 * Named GPIO output array: the serial interface. Levels are those of the
 * PL022's signals, so the output enables are active low. The frame
 * select, the output enables and the idle clock level are driven as the
 * TRM describes; the bit-by-bit SSPCLKOUT and SSPTXD waveform within a
 * frame is not (SSPTXD stays low), as the data travels on the SSI bus.
 */
#define PL022_SSP_OUT "ssp-out"
enum {
    PL022_SSPCLKOUT,
    PL022_SSPFSSOUT,
    PL022_SSPTXD,
    PL022_NSSPCTLOE,
    PL022_NSSPOE,
    PL022_SSP_OUTS,
};

/* Named GPIO input: SSPFSSIN. */
#define PL022_SSPFSSIN "sspfssin"

struct PL022State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t cr0;
    uint32_t cr1;
    uint32_t bitmask;
    uint32_t sr;
    uint32_t cpsr;
    /* Raw interrupt status: ROR and RT latch, RX and TX follow the FIFOs */
    uint32_t is;
    uint32_t im;
    uint32_t dmacr;
    /* The FIFO head points to the next empty entry.  */
    int tx_fifo_head;
    int rx_fifo_head;
    int tx_fifo_len;
    int rx_fifo_len;
    uint16_t tx_fifo[PL022_FIFO_DEPTH];
    uint16_t rx_fifo[PL022_FIFO_DEPTH];
    /* A master frame is on the wire, its transmit word in the shifter. */
    bool busy;
    uint16_t shift;
    /* When the frame on the wire ends, in QEMU_CLOCK_VIRTUAL ns. */
    int64_t frame_end;
    bool fssin;

    QEMUTimer *frame_timer;
    QEMUTimer *rt_timer;
    Clock *clk;
    qemu_irq irq;
    qemu_irq dma_req[2];
    qemu_irq ssp_out[PL022_SSP_OUTS];
    /* Levels last driven on ssp_out, -1 before the first. */
    int8_t ssp_level[PL022_SSP_OUTS];
    char *bus_name;
    uint8_t revision;
    SSIBus *ssi;
};

struct PL022TargetState {
    SSIPeripheral parent_obj;

    PL022State *controller;
};

#endif
