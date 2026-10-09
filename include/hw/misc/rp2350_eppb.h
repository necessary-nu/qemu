/*
 * RP2350 Cortex-M33 extended private peripheral bus registers (EPPB)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_EPPB_H
#define HW_MISC_RP2350_EPPB_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_EPPB "rp2350-eppb"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350EPPBState, RP2350_EPPB)

#define RP2350_EPPB_NUM_IRQS 52

/*
 * One core's EPPB. Each core has its own instance, mapped in that core's
 * private peripheral bus.
 *
 * The block sits on the core's interrupt inputs: its unnamed GPIO input n
 * is system IRQ n as seen by this core, which it passes on unchanged to
 * GPIO output "irq" n (the core's NVIC input) while also feeding the NMI
 * mask. Named GPIO input "rcp-nmi" is the redundancy coprocessor's NMI
 * request; GPIO output "nmi" drives the core's NMI input.
 */
struct RP2350EPPBState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *wicen_timer;
    qemu_irq irq_out[RP2350_EPPB_NUM_IRQS];
    qemu_irq nmi;

    uint64_t nmi_mask;
    uint32_t sleepctrl;

    /* Current levels of the input lines. */
    uint64_t irq_level;
    bool rcp_nmi;
};

#endif
