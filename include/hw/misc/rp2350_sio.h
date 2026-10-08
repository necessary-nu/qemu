/*
 * RP2350 single-cycle IO block (SIO)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_SIO_H
#define HW_MISC_RP2350_SIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_SIO "rp2350-sio"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350SIOState, RP2350_SIO)

#define RP2350_SIO_CORES 2
#define RP2350_SIO_FIFO_DEPTH 4
#define RP2350_SIO_VIEW_SIZE 0x1000

/* Banks: SIO keeps separate FIFOs, doorbells and spinlocks per state. */
enum {
    RP2350_SIO_SECURE,
    RP2350_SIO_NONSECURE,
    RP2350_SIO_BANKS,
};

typedef struct RP2350SIOView {
    RP2350SIOState *sio;
    int core;
    /* -1 selects the bank by the access's security attribute. */
    int bank;
} RP2350SIOView;

typedef struct RP2350SIOBank {
    /* fifo[c] is written by core c and read by the other core. */
    uint32_t fifo[RP2350_SIO_CORES][RP2350_SIO_FIFO_DEPTH];
    uint8_t fifo_head[RP2350_SIO_CORES];
    uint8_t fifo_count[RP2350_SIO_CORES];
    /* Sticky read-on-empty and write-on-full flags, per accessing core. */
    bool roe[RP2350_SIO_CORES];
    bool wof[RP2350_SIO_CORES];
    /* doorbell[c] holds the flags pending for core c. */
    uint8_t doorbell[RP2350_SIO_CORES];
    uint32_t spinlocks;
} RP2350SIOBank;

struct RP2350SIOState {
    SysBusDevice parent_obj;

    /*
     * Per core, the window at SIO_BASE and the Secure-only Non-secure
     * mirror at SIO_NONSEC_BASE. SIO is core-local, so each core's views
     * are mapped into that core's address space.
     */
    MemoryRegion view[RP2350_SIO_CORES][2];
    RP2350SIOView view_opaque[RP2350_SIO_CORES][2];

    RP2350SIOBank bank[RP2350_SIO_BANKS];
    uint32_t gpio_out[2];
    uint32_t gpio_oe[2];

    /* Core-local interrupts, per bank and core. */
    qemu_irq irq_fifo[RP2350_SIO_BANKS][RP2350_SIO_CORES];
    qemu_irq irq_bell[RP2350_SIO_BANKS][RP2350_SIO_CORES];
};

/*
 * The view a core sees at SIO_BASE (mirror false) or SIO_NONSEC_BASE
 * (mirror true).
 */
MemoryRegion *rp2350_sio_view(RP2350SIOState *s, int core, bool mirror);

#endif
