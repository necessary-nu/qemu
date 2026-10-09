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
#include "target/arm/cpu-qom.h"

#define TYPE_RP2350_SIO "rp2350-sio"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350SIOState, RP2350_SIO)

#define RP2350_SIO_CORES 2
#define RP2350_SIO_FIFO_DEPTH 4
#define RP2350_SIO_VIEW_SIZE 0x1000
#define RP2350_SIO_GPIO_BITS 64

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

/*
 * Called when the emulated core 1 launch handshake completes, with the
 * vector table, stack pointer and entry point core 0 supplied.
 */
typedef void RP2350SIOCore1Launch(void *opaque, uint32_t vtor, uint32_t sp,
                                  uint32_t entry);

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
    RP2350SIOView gpioc_opaque[RP2350_SIO_CORES];
    uint32_t gpio_out[2];
    uint32_t gpio_oe[2];
    /* GPIO_IN as the IO muxing delivers it, through named input "gpio-in". */
    uint32_t gpio_in[2];
    /*
     * GPIO_OUT and GPIO_OE drive the IO muxing's SIO function through
     * named outputs "gpio-out" and "gpio-oe", one line per bit in the
     * 64-bit SIO layout; *_sent are the levels last driven.
     */
    qemu_irq gpio_out_line[RP2350_SIO_GPIO_BITS];
    qemu_irq gpio_oe_line[RP2350_SIO_GPIO_BITS];
    uint32_t gpio_out_sent[2];
    uint32_t gpio_oe_sent[2];

    /*
     * With core1-launch set, SIO plays core 1's side of the boot ROM
     * launch handshake on the Secure FIFOs until core 1 is launched.
     */
    bool core1_launch;
    uint32_t c1_state;
    uint32_t c1_send;
    uint32_t c1_next;
    uint32_t c1_vtor;
    uint32_t c1_sp;
    RP2350SIOCore1Launch *c1_launch_fn;
    void *c1_launch_opaque;

    /* Core-local interrupts, per bank and core. */
    qemu_irq irq_fifo[RP2350_SIO_BANKS][RP2350_SIO_CORES];
    qemu_irq irq_bell[RP2350_SIO_BANKS][RP2350_SIO_CORES];
};

/*
 * The view a core sees at SIO_BASE (mirror false) or SIO_NONSEC_BASE
 * (mirror true).
 */
MemoryRegion *rp2350_sio_view(RP2350SIOState *s, int core, bool mirror);

void rp2350_sio_set_core1_launch(RP2350SIOState *s, RP2350SIOCore1Launch *fn,
                                 void *opaque);

/* Make the GPIO coprocessor (GPIOC) coprocessor 0 of `cpu`, core `core`. */
void rp2350_sio_attach_gpioc(RP2350SIOState *s, int core, ARMCPU *cpu);

#endif
