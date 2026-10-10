/*
 * ESP32-P4 HP system clock and reset control (HP_SYS_CLKRST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_HP_SYS_CLKRST_H
#define HW_MISC_ESP32P4_HP_SYS_CLKRST_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_HP_SYS_CLKRST "esp32p4-hp-sys-clkrst"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4HPSysClkRstState, ESP32P4_HP_SYS_CLKRST)

#define ESP32P4_HP_SYS_CLKRST_SIZE 0x1000
#define ESP32P4_HP_SYS_CLKRST_CORES 2

/*
 * Named GPIO outputs, one line per HP core: its reset, high while held,
 * and its CPU clock enable.
 */
#define ESP32P4_HP_SYS_CLKRST_CORE_RESET "core-reset"
#define ESP32P4_HP_SYS_CLKRST_CORE_CLOCK "core-clock"

struct ESP32P4HPSysClkRstState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t soc_clk_ctrl0;
    uint32_t hp_rst_en0;
    qemu_irq core_reset[ESP32P4_HP_SYS_CLKRST_CORES];
    qemu_irq core_clock[ESP32P4_HP_SYS_CLKRST_CORES];
};

#endif
