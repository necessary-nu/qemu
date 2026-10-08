/*
 * RP2350 subsystem resets (RESETS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_RESETS_H
#define HW_MISC_RP2350_RESETS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_RESETS "rp2350-resets"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350ResetsState, RP2350_RESETS)

#define RP2350_RESETS_ALL 0x1fffffff

struct RP2350ResetsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t reset;
    uint32_t wdsel;
};

#endif
