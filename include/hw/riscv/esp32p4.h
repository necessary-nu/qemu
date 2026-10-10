/*
 * Espressif ESP32-P4 machine
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_RISCV_ESP32P4_H
#define HW_RISCV_ESP32P4_H

#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/cpu/cluster.h"
#include "hw/ssi/ssi.h"
#include "hw/misc/esp32p4_hp_sys_clkrst.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "system/memory.h"
#include "target/riscv/esp32p4_cpu.h"

#define TYPE_ESP32P4_MACHINE MACHINE_TYPE_NAME("esp32p4")
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4MachineState, ESP32P4_MACHINE)

#define ESP32P4_HP_CORES 2

/* The internal memories (TRM table 9.3-1). */
#define ESP32P4_TCM_BASE            0x30100000
#define ESP32P4_TCM_SIZE            (8 * KiB)
#define ESP32P4_HP_ROM_BASE         0x4fc00000
#define ESP32P4_HP_ROM_DIRECT_BASE  0x8fc00000
#define ESP32P4_HP_ROM_SIZE         (128 * KiB)
#define ESP32P4_L2MEM_BASE          0x4ff00000
#define ESP32P4_L2MEM_DIRECT_BASE   0x8ff00000
#define ESP32P4_L2MEM_SIZE          (768 * KiB)
#define ESP32P4_LP_ROM_BASE         0x50100000
#define ESP32P4_LP_ROM_SIZE         (16 * KiB)
#define ESP32P4_LP_SRAM_BASE        0x50108000
#define ESP32P4_LP_SRAM_SIZE        (32 * KiB)

/* The external memory windows, which the cache's MMU maps. */
#define ESP32P4_FLASH_WINDOW_BASE        0x40000000
#define ESP32P4_FLASH_WINDOW_DIRECT_BASE 0x80000000
#define ESP32P4_PSRAM_WINDOW_BASE        0x48000000
#define ESP32P4_PSRAM_WINDOW_DIRECT_BASE 0x88000000
#define ESP32P4_EXT_WINDOW_SIZE          (64 * MiB)

/* In-package PSRAM: the NRW32X's 32 MiB (the NRW16X has 16 MiB). */
#define ESP32P4_PSRAM_SIZE          (32 * MiB)

/* The off-package flash sizes ESP-IDF supports. */
#define ESP32P4_FLASH_MIN_SIZE      (1 * MiB)
#define ESP32P4_FLASH_MAX_SIZE      (128 * MiB)

#define ESP32P4_HP_SYS_CLKRST_BASE  0x500e6000
#define ESP32P4_LP_SYS_BASE         0x50110000

/* Where each core starts from reset. */
#define ESP32P4_HP_RESET_VECTOR     ESP32P4_HP_ROM_BASE
#define ESP32P4_LP_RESET_VECTOR     ESP32P4_LP_ROM_BASE

#define ESP32P4_ROM_FILE            "esp32p4-rev3-rom.bin"

struct ESP32P4MachineState {
    MachineState parent_obj;

    CPUClusterState hp_cluster;
    CPUClusterState lp_cluster;
    ESP32P4CPU hp_cpu[ESP32P4_HP_CORES];
    ESP32P4CPU lp_cpu;

    MemoryRegion tcm;
    MemoryRegion hp_rom;
    MemoryRegion hp_rom_direct;
    MemoryRegion l2mem;
    MemoryRegion l2mem_direct;
    MemoryRegion lp_sram;
    MemoryRegion psram;

    ESP32P4HPSysClkRstState hp_sys_clkrst;
    ESP32P4LPSysState lp_sys;

    SSIBus *flash_bus;
    uint64_t flash_size;
};

#endif
