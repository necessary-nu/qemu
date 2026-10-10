/*
 * RP2350 high-speed serial transmit (HSTX)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_HSTX_H
#define HW_MISC_RP2350_HSTX_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_HSTX "rp2350-hstx"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350HSTXState, RP2350_HSTX)

/*
 * MMIO regions: 0 is HSTX_CTRL (APB: plain, the atomic aliases, then the
 * same again with narrow-write replication off at address bit 14), 1 is
 * HSTX_FIFO (the AHB FIFO window).
 */
#define RP2350_HSTX_CTRL_SIZE 0x8000
#define RP2350_HSTX_FIFO_SIZE 0x100000

/* Output bits, on GPIOs 12-19. */
#define RP2350_HSTX_BITS 8
#define RP2350_HSTX_FIFO_DEPTH 8
#define RP2350_HSTX_PIOS 3
#define RP2350_HSTX_LANES 3

/*
 * Named GPIOs:
 *
 *   "out"     output[8]: the level of output bit n, at coarse resolution
 *                        (see below)
 *   "oe"      output[8]: its output enable; HSTX is output-only, so 1
 *   "dreq"    output[1]: DREQ_HSTX, high while the FIFO has room
 *   "pio-out" input[24]: for coupled mode, PIO p's outputs for GPIOs
 *                        12-19 on lines 8p to 8p+7
 *
 * clk_hstx is the "clk" clock input.
 */
#define RP2350_HSTX_OUT "out"
#define RP2350_HSTX_OE "oe"
#define RP2350_HSTX_DREQ "dreq"
#define RP2350_HSTX_PIO_OUT "pio-out"

/*
 * Observability. The outputs change up to twice per clk_hstx cycle, far
 * faster than anything outside the chip model could follow, so the
 * device keeps exact records of what it emitted and drives the "out"
 * lines only at coarse instants: whenever its state is synchronised (a
 * register access, a DREQ edge, a coupled-mode PIO input change), every
 * "pin-refresh-ns" while it is shifting, and in the cycle in which it
 * stops shifting. At each instant a line
 * shows the level of the last half-cycle emitted, or while the shift
 * register is stopped, the crossbar applied to its current contents.
 *
 * Exact records, as read-only QOM properties (each brings the device up
 * to date with the current virtual time first):
 *
 *   "capture"       uint32 list: the last RP2350_HSTX_CAPTURE words
 *                   loaded into the output shift register, oldest first
 *                   (with the command expander's TMDS encoder these are
 *                   the packed 3 x 10-bit TMDS symbols)
 *   "capture-words" uint64: words loaded since reset
 *   "pin-history"   uint64 list[8]: for output bit n, its levels in the
 *                   last 64 half-cycles emitted, oldest in bit 0 and the
 *                   most recent in bit 63
 *   "half-cycles"   uint64: half-cycles emitted since reset (two per
 *                   cycle in which the shift register shifts)
 */
#define RP2350_HSTX_CAPTURE 16

/* The state that clocking the block evolves. */
typedef struct RP2350HSTXCore {
    /* clk_hstx cycles run, counted from the device's time base. */
    uint64_t cycle;

    uint32_t fifo[RP2350_HSTX_FIFO_DEPTH];
    uint32_t fifo_head;
    uint32_t fifo_level;

    /* Output shift register, and shifts left before it is empty. */
    uint32_t sr;
    uint32_t sr_left;
    /* Clock generator count, in half clk_hstx cycles. */
    uint32_t clk_count;

    /*
     * Command expander: a data command is in progress (else the next
     * FIFO word is a command), its opcode and the words it has yet to
     * output (exp_infinite: count 0, never ends); the expansion shift
     * register, whether it holds data and how many more outputs it
     * gives before it refills.
     */
    bool exp_cmd;
    bool exp_infinite;
    bool exp_valid;
    uint32_t exp_op;
    uint32_t exp_count;
    uint32_t exp_sr;
    uint32_t exp_left;
    /* TMDS running disparity per lane. */
    int32_t disp[RP2350_HSTX_LANES];
} RP2350HSTXCore;

/* What the device has emitted, for the observability properties. */
typedef struct RP2350HSTXRec {
    uint32_t capture[RP2350_HSTX_CAPTURE];
    uint64_t words;
    uint64_t half_cycles;
    uint64_t history[RP2350_HSTX_BITS];
    /* Output levels in the last half-cycle emitted, bit n for output n. */
    uint8_t pins;
} RP2350HSTXRec;

struct RP2350HSTXState {
    SysBusDevice parent_obj;

    MemoryRegion ctrl_iomem;
    MemoryRegion fifo_iomem;
    Clock *clk;
    QEMUTimer *timer;
    qemu_irq out[RP2350_HSTX_BITS];
    qemu_irq oe[RP2350_HSTX_BITS];
    qemu_irq dreq;

    uint32_t pin_refresh_ns;

    /* clk_hstx cycle count at virtual time base_ns. */
    int64_t base_ns;
    uint64_t base_cycle;

    uint32_t csr;
    uint32_t bit[RP2350_HSTX_BITS];
    uint32_t expand_shift;
    uint32_t expand_tmds;
    /* STAT.WOF */
    bool wof;

    RP2350HSTXCore core;
    RP2350HSTXRec rec;

    /* Coupled-mode PIO outputs for GPIOs 12-19, per PIO. */
    uint8_t pio_out[RP2350_HSTX_PIOS];

    /* Levels last driven on the GPIO and DREQ lines. */
    uint8_t out_level[RP2350_HSTX_BITS];
    uint8_t oe_level[RP2350_HSTX_BITS];
    uint8_t dreq_level;
    /*
     * Set while the block is run forward: FIFO writes the DMA makes from
     * the DREQ change of a pop see the block as of that pop, and do not
     * run it further.
     */
    bool syncing;
};

#endif
