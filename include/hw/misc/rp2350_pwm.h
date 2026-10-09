/*
 * RP2350 pulse width modulation (PWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_PWM_H
#define HW_MISC_RP2350_PWM_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_PWM "rp2350-pwm"
typedef struct RP2350PWMState RP2350PWMState;
DECLARE_INSTANCE_CHECKER(RP2350PWMState, RP2350_PWM, TYPE_RP2350_PWM)

#define RP2350_PWM_SLICES 12

/* Sysbus IRQs: PWM_IRQ_WRAP_0 and PWM_IRQ_WRAP_1. */
#define RP2350_PWM_IRQS 2

/*
 * Named GPIO arrays, indexed by RP2350_GPIO_PWM(slice, channel):
 *
 *   "out"  output: the level the slice drives on channel A or B
 *   "oe"   output: its output enable; B is an input outside free-running
 *          mode
 *
 * and, indexed by slice:
 *
 *   "b-in"      input:  the level of the slice's B pin (gated and edge
 *               counting modes)
 *   "dreq-wrap" output: DREQ_PWM_WRAP<slice>. Each counter wrap pulses
 *               the line (1 then 0), one pulse per DMA transfer credit.
 *               Pulses are generated only while the line is connected.
 */
#define RP2350_PWM_OUT "out"
#define RP2350_PWM_OE "oe"
#define RP2350_PWM_B_IN "b-in"
#define RP2350_PWM_DREQ "dreq-wrap"

typedef struct RP2350PWMSlice {
    RP2350PWMState *pwm;
    int index;
    QEMUTimer *timer;

    /* CSR: EN, PH_CORRECT, A_INV, B_INV, DIVMODE, PH_RET, PH_ADV. */
    uint32_t csr;
    uint32_t div;
    /* Software's copies of CC and TOP, and the copies latched at wrap. */
    uint32_t cc;
    uint32_t top;
    uint32_t cc_latched;
    uint32_t top_latched;

    uint32_t ctr;
    /* Phase-correct mode: counting down from TOP. */
    bool down;
    /* Fractional divider accumulator, in sixteenths of an event. */
    uint32_t acc;

    /* clk_sys cycles up to which the slice has been run. */
    uint64_t sync_cycle;

    /*
     * The B pin as it arrives (b_in) and as the slice's input
     * synchroniser last sampled it (b_seen); b_in changed in cycle
     * b_change, and the slice sees the new level a cycle later.
     */
    bool b_in;
    bool b_seen;
    uint64_t b_change;

    /* Wraps not yet signalled on the DREQ line. */
    uint32_t dreq_pending;

    /* When the timer last fired to update the outputs. */
    int64_t last_out_ns;
    int64_t out_due_ns;

    /* Levels last driven on the GPIO lines, per channel. */
    uint8_t out_level[2];
    uint8_t oe_level[2];
} RP2350PWMSlice;

struct RP2350PWMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;
    qemu_irq irq[RP2350_PWM_IRQS];
    qemu_irq out[RP2350_PWM_SLICES * 2];
    qemu_irq oe[RP2350_PWM_SLICES * 2];
    qemu_irq dreq[RP2350_PWM_SLICES];

    /*
     * Output edges closer together than this are not each delivered;
     * the outputs are refreshed at most this often and show the
     * slice's true level at each refresh.
     */
    uint32_t min_edge_ns;

    /* clk_sys cycle count at virtual time base_ns. */
    int64_t base_ns;
    uint64_t base_cycle;

    RP2350PWMSlice slice[RP2350_PWM_SLICES];
    uint32_t intr;
    uint32_t inte[RP2350_PWM_IRQS];
    uint32_t intf[RP2350_PWM_IRQS];
};

#endif
