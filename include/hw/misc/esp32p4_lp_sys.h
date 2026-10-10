/*
 * ESP32-P4 LP system registers (LP_SYS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LP_SYS_H
#define HW_MISC_ESP32P4_LP_SYS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_SYS "esp32p4-lp-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPSysState, ESP32P4_LP_SYS)

#define ESP32P4_LP_SYS_SIZE 0x1000

struct ESP32P4LPSysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t boot_addr_hp_core1;
};

#endif
