/*
 * ESP32-P4 debug assistant (ASSIST_DEBUG, the CPU bus monitor)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ASSIST_DEBUG_H
#define HW_MISC_ESP32P4_ASSIST_DEBUG_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ASSIST_DEBUG "esp32p4-assist-debug"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4AssistDebugState, ESP32P4_ASSIST_DEBUG)

#define ESP32P4_ASSIST_DEBUG_BASE   0x3ff06000
#define ESP32P4_ASSIST_DEBUG_SIZE   0x1000
#define ESP32P4_ASSIST_DEBUG_CORES  2

/*
 * Named GPIO input ESP32P4_ASSIST_DEBUG_CORE_RESET: per HP core, high
 * while the core is held in reset. As it rises the core stops sending its
 * PC: PDEBUGPC and PDEBUGSP keep the core's PC and SP at the reset, and
 * RCD_EN clears.
 *
 * Sysbus IRQ 0 is ASSIST_DEBUG_INTR, source 127 of the interrupt matrix.
 * Links "cpu0" and "cpu1" are the HP cores.
 */
#define ESP32P4_ASSIST_DEBUG_CORE_RESET "core-reset"

struct ESP32P4CPU;

typedef struct ESP32P4AssistDebugCore {
    struct ESP32P4CPU *cpu;
    /* INTR_ENA (the monitors' enables) and INTR_RLS (the interrupts'). */
    uint32_t ena;
    uint32_t rls;
    /* INTR_RAW: the regions' latched bits, and SP's place, a level. */
    uint32_t raw_region;
    uint32_t raw_sp;
    /* The interrupts latched from the raw bits RLS lets through. */
    uint32_t st;
    /* AREA_DRAM0_0_MIN to AREA_PIF_1_MAX: MIN, MAX of each region. */
    uint32_t area[8];
    uint32_t area_pc;
    uint32_t area_sp;
    uint32_t sp_min;
    uint32_t sp_max;
    uint32_t sp_pc;
    uint32_t rcd_en;
    uint32_t pdebug_pc;
    uint32_t pdebug_sp;
    bool in_reset;
} ESP32P4AssistDebugCore;

struct ESP32P4AssistDebugState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    /*
     * Drives the interrupt once the reporting core's block of translated
     * code ends, for a report made in its middle (see
     * esp32p4_cpu_mid_block()).
     */
    QEMUTimer *irq_timer;
    ESP32P4AssistDebugCore core[ESP32P4_ASSIST_DEBUG_CORES];
    uint32_t limit_cycle[2];
    uint32_t clock_gate;
    uint32_t date;
};

#endif
