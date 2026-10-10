/*
 * ESP32-P4 HP system registers (HP_SYSTEM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_HP_SYS_H
#define HW_MISC_ESP32P4_HP_SYS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_HP_SYS "esp32p4-hp-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4HPSysState, ESP32P4_HP_SYS)

#define ESP32P4_HP_SYS_BASE 0x500e5000
#define ESP32P4_HP_SYS_SIZE 0x1000

/* The software interrupts CPU_INT_FROM_CPU_0 to _3. */
#define ESP32P4_HP_SYS_CPU_INTRS 4

/*
 * Named GPIO output ESP32P4_HP_SYS_CPU_INTR: CPU_INTR_FROM_CPU_0 to _3,
 * interrupt sources ESP32P4_INTR_FROM_CPU_INTR0 to _3 of the interrupt
 * matrix, each high while its register's bit is set.
 */
#define ESP32P4_HP_SYS_CPU_INTR "cpu-intr-from-cpu"

struct ESP32P4HPSysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t cpu_int_from_cpu[ESP32P4_HP_SYS_CPU_INTRS];
    qemu_irq cpu_intr[ESP32P4_HP_SYS_CPU_INTRS];
};

#endif
