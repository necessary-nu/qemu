/*
 * RP2350 boot RAM (BOOTRAM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_BOOTRAM_H
#define HW_MISC_RP2350_BOOTRAM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_BOOTRAM "rp2350-bootram"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350BootRAMState, RP2350_BOOTRAM)

#define RP2350_BOOTRAM_SIZE 1024
#define RP2350_BOOTRAM_LOCKS 8

struct RP2350BootRAMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint8_t ram[RP2350_BOOTRAM_SIZE];
    uint32_t write_once[2];
    /* BOOTLOCK_STAT: a set bit is an unclaimed lock. */
    uint32_t lock_stat;
};

#endif
