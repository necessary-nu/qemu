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
#include "hw/core/split-irq.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/intc/esp32p4_clic.h"
#include "hw/intc/esp32p4_clint.h"
#include "hw/intc/esp32p4_intmatrix.h"
#include "hw/ssi/ssi.h"
#include "hw/misc/esp32p4_ana.h"
#include "hw/misc/esp32p4_cache.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/misc/esp32p4_hp_sys.h"
#include "hw/misc/esp32p4_i2c_ana_mst.h"
#include "hw/misc/esp32p4_icm.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "hw/misc/esp32p4_pms.h"
#include "hw/net/esp32p4_emac.h"
#include "hw/net/ip101_phy.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "hw/ssi/esp32p4_mspi.h"
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

#define ESP32P4_CACHE_BASE          0x3ff10000
#define ESP32P4_FLASH_MSPI_BASE     0x5008c000
#define ESP32P4_PSRAM_MSPI_BASE     0x5008e000
#define ESP32P4_HP_SYS_CLKRST_BASE  0x500e6000
#define ESP32P4_LP_SYS_BASE         0x50110000
#define ESP32P4_LP_CLKRST_BASE      0x50111000
#define ESP32P4_LPPERI_BASE         0x50120000
#define ESP32P4_I2C_ANA_MST_BASE    0x50124000
#define ESP32P4_EFUSE_BASE          0x5012d000

/* Where each core starts from reset. */
#define ESP32P4_HP_RESET_VECTOR     ESP32P4_HP_ROM_BASE
#define ESP32P4_LP_RESET_VECTOR     ESP32P4_LP_ROM_BASE

#define ESP32P4_ROM_FILE            "esp32p4-rev3-rom.bin"

/* The board pad the IP101GRI's RESET_N is on */
#define ESP32P4_BOARD_PHY_RESET_PAD 51

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
    /* The HP cores' address space, and the bus in it. */
    MemoryRegion hp_mem;
    MemoryRegion hp_bus;

    /* The HP cores' resets, to each core and its CLIC. */
    SplitIRQ core_reset_split[ESP32P4_HP_CORES];
    ESP32P4ClintState clint;
    ESP32P4ClicState clic;
    ESP32P4IntMatrixState intmatrix;
    ESP32P4HPSysState hp_sys;

    ESP32P4ClkRstState clkrst;
    ESP32P4AnaState ana;
    ESP32P4I2CAnaMstState i2c_ana_mst;
    ESP32P4LPSysState lp_sys;
    /* The HP cores' reset vector inputs from LP_CLKRST_*_STAT_VECTOR_SEL. */
    qemu_irq *core_vector;
    /* The entry point of the -kernel ELF, which core 0 starts at; or 0. */
    uint64_t kernel_entry;
    ESP32P4EfuseState efuse;
    ESP32P4GpioState gpio;
    /* The board's strapping of GPIO34 to GPIO38, in GPIO_STRAP's layout */
    uint32_t strap_mode;
    /* The access permission managers, every master's way onto the bus. */
    ESP32P4PmsState pms;
    ESP32P4IcmState icm;

    ESP32P4MSPIState flash_mspi;
    ESP32P4MSPIState psram_mspi;
    /* HP_SYSTEM_CRYPTO_CTRL's enables, to the flash's and PSRAM's MSPI. */
    SplitIRQ crypto_split[ESP32P4_HP_SYS_CRYPTO_CTRL_LINES];
    MemoryRegion flash_window_alias;
    MemoryRegion psram_window_alias;
    ESP32P4CacheState cache;
    SSIBus *flash_bus;
    uint64_t flash_size;

    /* The EMAC and the board's IP101GRI PHY, and the EMAC's PPS output */
    ESP32P4EmacState emac;
    IP101PhyState phy;
    SplitIRQ pps_split;
};

#endif
