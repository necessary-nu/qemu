/*
 * ESP32-P4 LP interrupt controller (LP_INTR)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_ESP32P4_LP_INTR_H
#define HW_INTC_ESP32P4_LP_INTR_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_INTR "esp32p4-lp-intr"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPIntrState, ESP32P4_LP_INTR)

#define ESP32P4_LP_INTR_BASE    0x5012c000
#define ESP32P4_LP_INTR_SIZE    0x1000

/* The LP interrupt sources, interrupt matrix sources 0 to 20. */
#define ESP32P4_LP_INTR_SOURCES 21

/* The LP core's interrupt IDs, lines of ESP32P4_LP_INTR_CPU_IRQ. */
#define ESP32P4_LP_INTR_CPU_IRQS 32

/*
 * Inputs and outputs:
 *
 *   - Named GPIO input ESP32P4_LP_INTR_SOURCE: line n the level of LP
 *     interrupt source n, numbered as the interrupt matrix's (n < 21), from
 *     the matrix's fan-out of every source's level. Line 18, LP_SW_INTR,
 *     is the block's own and is not taken in.
 *   - Named GPIO input ESP32P4_LP_INTR_HP: HP_INTR, HP_SYSTEM's group of the
 *     HP interrupt sources.
 *   - Named GPIO input ESP32P4_LP_INTR_MASK: high while the PMU's sleep
 *     process masks the LP core's interrupts (PMU_LP_CPU_SLP_BYPASS_INTR_EN).
 *   - Named GPIO output ESP32P4_LP_INTR_CPU_IRQ: line n the level of the LP
 *     core's interrupt ID n, to its ESP32P4_CPU_LP_IRQ input.
 *   - Sysbus IRQ 0: LP_SW_INTR, interrupt matrix source 18.
 */
#define ESP32P4_LP_INTR_SOURCE  "source"
#define ESP32P4_LP_INTR_HP      "hp-intr"
#define ESP32P4_LP_INTR_MASK    "mask"
#define ESP32P4_LP_INTR_CPU_IRQ "cpu-irq"

struct ESP32P4LPIntrState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq sw_irq;
    qemu_irq cpu_irq[ESP32P4_LP_INTR_CPU_IRQS];

    /* The sources' levels, bit n source n, and HP_INTR's. */
    uint32_t sources;
    bool hp_intr;
    bool masked;
    /* LP_SW_INT_RAW and _ENA, bit 31, and DATE's CLK_EN. */
    uint32_t sw_raw;
    uint32_t sw_ena;
    uint32_t clk_en;
    /* The levels last driven on the core's lines. */
    uint32_t cpu_level;
};

#endif
