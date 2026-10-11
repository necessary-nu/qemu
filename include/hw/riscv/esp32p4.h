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
#include "hw/audio/esp32p4_i2s.h"
#include "hw/core/boards.h"
#include "hw/char/esp32_uart_terminal.h"
#include "hw/char/esp32p4_uart.h"
#include "hw/char/esp32p4_usb_serial_jtag.h"
#include "hw/cpu/cluster.h"
#include "hw/core/or-irq.h"
#include "hw/core/split-irq.h"
#include "hw/dma/esp32p4_bitscrambler.h"
#include "hw/dma/esp32p4_dw_gdma.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/dma/esp32p4_uhci.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/i2c/esp32p4_i2c.h"
#include "hw/i3c/esp32p4_i3c.h"
#include "hw/intc/esp32p4_clic.h"
#include "hw/intc/esp32p4_clint.h"
#include "hw/intc/esp32p4_intmatrix.h"
#include "hw/intc/esp32p4_lp_intr.h"
#include "hw/ssi/ssi.h"
#include "hw/misc/esp32p4_aes.h"
#include "hw/misc/esp32p4_ana.h"
#include "hw/misc/esp32p4_assist_debug.h"
#include "hw/misc/esp32p4_axi_perf_mon.h"
#include "hw/misc/esp32p4_cache.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/misc/esp32p4_ds.h"
#include "hw/misc/esp32p4_ecc.h"
#include "hw/misc/esp32p4_ecdsa.h"
#include "hw/misc/esp32p4_etm.h"
#include "hw/misc/esp32p4_hmac.h"
#include "hw/misc/esp32p4_hp_sys.h"
#include "hw/misc/esp32p4_huk.h"
#include "hw/misc/esp32p4_i2c_ana_mst.h"
#include "hw/misc/esp32p4_icm.h"
#include "hw/misc/esp32p4_keymng.h"
#include "hw/misc/esp32p4_lp_ana.h"
#include "hw/misc/esp32p4_lp_mailbox.h"
#include "hw/misc/esp32p4_ledc.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "hw/misc/esp32p4_parlio.h"
#include "hw/misc/esp32p4_pcnt.h"
#include "hw/misc/esp32_mcpwm.h"
#include "hw/misc/esp32p4_pms.h"
#include "hw/misc/esp32p4_pmu.h"
#include "hw/misc/esp32p4_rmt.h"
#include "hw/misc/esp32p4_rsa.h"
#include "hw/misc/esp32p4_sha.h"
#include "hw/misc/esp32p4_trace.h"
#include "hw/misc/esp32p4_trng.h"
#include "hw/net/esp32p4_emac.h"
#include "hw/net/esp32p4_twai.h"
#include "hw/net/ip101_phy.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "hw/sd/esp32p4_sdmmc.h"
#include "hw/timer/esp32p4_lp_timer.h"
#include "hw/timer/esp32p4_lp_wdt.h"
#include "hw/timer/esp32p4_systimer.h"
#include "hw/timer/esp32p4_timg.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "hw/ssi/esp32p4_spi.h"
#include "hw/usb/dwc2-otg.h"
#include "hw/usb/esp32p4_usb.h"
#include "system/memory.h"
#include "target/riscv/esp32p4_cpu.h"

#define TYPE_ESP32P4_MACHINE MACHINE_TYPE_NAME("esp32p4")
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4MachineState, ESP32P4_MACHINE)

#define ESP32P4_HP_CORES 2

/* The board's serial ports: UART0 to UART4, then the LP UART. */
#define ESP32P4_SERIAL_PORTS (ESP32P4_UART_COUNT + 1)

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
#define ESP32P4_MCPWM0_BASE         0x500c0000
#define ESP32P4_MCPWM_STRIDE        0x1000
#define ESP32P4_MCPWM_NUM           2

/*
 * Where each core starts from reset: the LP core 0x80 into the vector
 * table at its boot address, LP_SYSTEM_LP_CORE_BOOT_ADDR, whose reset value
 * is the LP ROM's.
 */
#define ESP32P4_HP_RESET_VECTOR     ESP32P4_HP_ROM_BASE
#define ESP32P4_LP_BOOT_OFFSET      0x80
#define ESP32P4_LP_RESET_VECTOR     (ESP32P4_LP_ROM_BASE + \
                                     ESP32P4_LP_BOOT_OFFSET)

#define ESP32P4_ROM_FILE            "esp32p4-rev3-rom.bin"

/* The board pad the IP101GRI's RESET_N is on */
#define ESP32P4_BOARD_PHY_RESET_PAD 51

/* The crypto block resets that hold more than their own block. */
#define ESP32P4_CRYPTO_RST_SPLITS   4

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
    /*
     * The LP ROM, and whether it holds an image. The machine ships none:
     * without the lp-rom property's image, dumped by the user from their
     * own chip, the LP ROM is absent and the LP core starts past it.
     */
    char *lp_rom_file;
    MemoryRegion lp_rom;
    bool lp_rom_loaded;
    /*
     * With an image, the LP ROM is on the LP core's own buses: the LP
     * core's address space is the bus with the LP ROM over it, and the bus
     * itself answers the LP ROM's addresses with an error.
     */
    MemoryRegion lp_mem;
    MemoryRegion lp_bus;
    MemoryRegion lp_rom_hole;
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
    /* The LP core's interrupt controller. */
    ESP32P4LPIntrState lp_intr;

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
    ESP32P4I2CState i2c[2];
    ESP32P4I2CState lp_i2c;
    ESP32P4I3CMstState i3c_mst;
    ESP32P4I3CSlvState i3c_slv;
    ESP32P4SdmmcState sdmmc;
    /* GDMA-AHB, GDMA-AXI. */
    ESP32P4GdmaState gdma[2];
    ESP32P4DwGdmaState vdma;
    /* GP-SPI2, GP-SPI3. */
    ESP32P4SpiState spi[2];
    ESP32P4BitScramblerState bitscrambler;
    ESP32P4ParlioState parlio;
    ESP32P4TwaiState twai;
    /* The QEMU CAN buses on TWAI0 to TWAI2's pads, or NULL. */
    CanBusState *canbus[ESP32P4_TWAI_NUM];
    ESP32P4RmtState rmt;
    ESP32P4PcntState pcnt;
    ESP32P4LedcState ledc;
    Esp32McpwmState mcpwm[ESP32P4_MCPWM_NUM];

    ESP32P4SysTimerState systimer;
    ESP32P4TimgState timg[2];
    ESP32P4LPWdtState lp_wdt;
    ESP32P4LPTimerState lp_timer;
    /* Each HP core's stall, to the system timer and the RTC timer. */
    SplitIRQ core_stall_split[ESP32P4_HP_CORES];
    /* The HP core reset, to the machine and the RTC timer. */
    SplitIRQ hp_core_reset_split;
    ESP32P4EtmState etm;

    /* Low-power management: the PMU, LP_ANALOG_PERI and the LP mailbox. */
    ESP32P4PMUState pmu;
    ESP32P4LPAnaState lp_ana;
    ESP32P4LPMailboxState lp_mailbox;
    /* The PMU's power of PD_TOP and of the HP memory, to the machine. */
    qemu_irq hp_top_power;
    qemu_irq hp_mem_power;
    /* Whether the HP memories have lost their contents since PD_TOP rose. */
    bool hp_mem_lost;
    /* The -kernel ELF's segments outside LP SRAM, for its reload. */
    GArray *kernel_segments;

    ESP32P4MSPIState flash_mspi;
    ESP32P4MSPIState psram_mspi;
    /* HP_SYSTEM_CRYPTO_CTRL's enables, to the flash's and PSRAM's MSPI. */
    SplitIRQ crypto_split[ESP32P4_HP_SYS_CRYPTO_CTRL_LINES];
    MemoryRegion flash_window_alias;
    MemoryRegion psram_window_alias;
    ESP32P4CacheState cache;
    /* The USB 2.0 OTG controllers, high and full speed, and their PHYs. */
    DWC2OTGState usb_hs;
    DWC2OTGState usb_fs;
    ESP32P4UsbUtmiState usb_utmi;
    ESP32P4UsbWrapState usb_wrap;
    SSIBus *flash_bus;
    uint64_t flash_size;

    /* The EMAC and the board's IP101GRI PHY, and the EMAC's PPS output */
    ESP32P4EmacState emac;
    IP101PhyState phy;
    SplitIRQ pps_split;

    ESP32P4UartState uart[ESP32P4_UART_COUNT];
    ESP32P4UartState lp_uart;
    SplitIRQ uart_wakeup_en_split;
    SplitIRQ lp_uart_wakeup_split;
    Esp32UartTerminalState terminal[ESP32P4_SERIAL_PORTS];
    ESP32P4UhciState uhci;
    ESP32P4UsjState usj;

    ESP32P4AssistDebugState assist_debug;
    ESP32P4AxiPerfMonState axi_perf_mon;
    ESP32P4TraceState trace[ESP32P4_HP_CORES];
    /* GPIO16 to GPIO54's holds: LP_SYSTEM's OR HP_SYSTEM's. */
    OrIRQState pad_hold_or[ESP32P4_GPIO_PADS - ESP32P4_HP_SYS_HOLD_FIRST];

    /* I2S0 to I2S2, then the LP I2S */
    ESP32P4I2sState i2s[ESP32P4_I2S_COUNT];

    /* The cryptography blocks. */
    ESP32P4AesState aes;
    ESP32P4ShaDevState sha;
    ESP32P4RsaState rsa;
    ESP32P4EccState ecc;
    ESP32P4HmacState hmac;
    ESP32P4DsState ds;
    ESP32P4EcdsaState ecdsa;
    ESP32P4KeyMngState keymng;
    ESP32P4HukState huk;
    ESP32P4TrngState trng;
    /* RST_EN_DS, _ECDSA, _KM and _HMAC, to the blocks each holds. */
    SplitIRQ crypto_rst_split[ESP32P4_CRYPTO_RST_SPLITS];
};

/*
 * How long LP_SYSTEM_SYS_SW_RST holds the HP core in reset; the TRM gives
 * no length, and this is the shortest of the RTC watchdog's.
 */
#define ESP32P4_SW_SYS_RESET_NS 100

#endif
