/*
 * Raspberry Pi RP2350 SoC (Arm Cortex-M33 configuration)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_RP2350_SOC_H
#define HW_ARM_RP2350_SOC_H

#include "hw/core/sysbus.h"
#include "hw/arm/armv7m.h"
#include "hw/core/clock.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"

#define TYPE_RP2350_SOC "rp2350-soc"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350State, RP2350_SOC)

#define RP2350_NUM_CORES 2
#define RP2350_NUM_IRQS 52

#define RP2350_ROM_BASE 0x00000000
#define RP2350_ROM_SIZE (32 * KiB)

/*
 * The XIP address space is four 64 MiB windows onto the same QSPI devices.
 * The cached window ends where the 16 KiB XIP cache-as-SRAM begins.
 */
#define RP2350_XIP_BASE 0x10000000
#define RP2350_XIP_NOCACHE_NOALLOC_BASE 0x14000000
#define RP2350_XIP_MAINTENANCE_BASE 0x18000000
#define RP2350_XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE 0x1c000000
#define RP2350_XIP_WINDOW_SIZE (64 * MiB)
#define RP2350_XIP_SRAM_BASE 0x13ffc000

/* Flash sits on QSPI chip select 0, which decodes 16 MiB. */
#define RP2350_FLASH_MAX_SIZE (16 * MiB)

#define RP2350_SRAM_BASE 0x20000000
#define RP2350_SRAM_SIZE (520 * KiB)

#define RP2350_EPPB_BASE 0xe0080000
#define RP2350_EPPB_SIZE 0x1000

#define RP2350_SYSCLK_HZ 150000000
#define RP2350_REFCLK_HZ 1000000

struct RP2350State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m[RP2350_NUM_CORES];
    UnimplementedDeviceState eppb[RP2350_NUM_CORES];

    MemoryRegion rom;
    MemoryRegion flash;
    MemoryRegion flash_nocache_alias;
    MemoryRegion flash_notranslate_alias;
    MemoryRegion sram;

    MemoryRegion *board_memory;
    /*
     * Each armv7m container takes its memory link as a subregion, and a
     * region can have only one container, so every core sees the board
     * memory through its own alias.
     */
    MemoryRegion core_memory[RP2350_NUM_CORES];

    uint32_t flash_size;
    uint32_t init_svtor;

    Clock *sysclk;
    Clock *refclk;
};

#endif
