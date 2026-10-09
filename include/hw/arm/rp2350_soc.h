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
#include "hw/char/pl011.h"
#include "hw/core/clock.h"
#include "hw/misc/rp2350_bootram.h"
#include "hw/misc/rp2350_clocks.h"
#include "hw/misc/rp2350_dcp.h"
#include "hw/misc/rp2350_rcp.h"
#include "hw/misc/rp2350_resets.h"
#include "hw/misc/rp2350_sio.h"
#include "hw/misc/rp2350_xip.h"
#include "hw/misc/unimp.h"
#include "hw/timer/rp2350_timer.h"
#include "qom/object.h"

#define TYPE_RP2350_SOC "rp2350-soc"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350State, RP2350_SOC)

#define RP2350_NUM_CORES 2
#define RP2350_NUM_IRQS 52
#define RP2350_NUM_UARTS 2
#define RP2350_NUM_TIMERS 2
#define RP2350_MPU_REGIONS 8

/*
 * IRQ numbers, from the pico-sdk intctrl.h. Peripheral models connect to
 * the SoC's unnamed GPIO input of the same number.
 */
#define RP2350_TIMER0_IRQ_0 0
#define RP2350_TIMER1_IRQ_0 4
#define RP2350_DMA_IRQ_0 10
#define RP2350_USBCTRL_IRQ 14
#define RP2350_IO_IRQ_BANK0 21
#define RP2350_SIO_IRQ_FIFO 25
#define RP2350_SIO_IRQ_BELL 26
#define RP2350_SIO_IRQ_FIFO_NS 27
#define RP2350_SIO_IRQ_BELL_NS 28
#define RP2350_SIO_IRQ_MTIMECMP 29
#define RP2350_CLOCKS_IRQ 30
#define RP2350_UART0_IRQ 33
#define RP2350_UART1_IRQ 34
#define RP2350_SPARE_IRQ_5 51

#define RP2350_ROM_BASE 0x00000000
#define RP2350_ROM_SIZE (32 * KiB)

/*
 * The XIP address space is four 64 MiB windows onto the same QSPI devices:
 * cached, uncached, cache maintenance, and uncached-untranslated.
 */
#define RP2350_XIP_BASE 0x10000000
#define RP2350_XIP_NOCACHE_NOALLOC_BASE 0x14000000
#define RP2350_XIP_MAINTENANCE_BASE 0x18000000
#define RP2350_XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE 0x1c000000
#define RP2350_XIP_WINDOW_SIZE (64 * MiB)

#define RP2350_XIP_CTRL_BASE 0x400c8000
#define RP2350_XIP_QMI_BASE 0x400d0000
#define RP2350_XIP_AUX_BASE 0x50500000

/*
 * Flash sits on QSPI chip select 0 and PSRAM, if fitted, on chip select 1.
 * Each chip select decodes 16 MiB.
 */
#define RP2350_FLASH_MIN_SIZE (1 * MiB)
#define RP2350_FLASH_MAX_SIZE (16 * MiB)

#define RP2350_SRAM_BASE 0x20000000
#define RP2350_SRAM_SIZE (520 * KiB)

#define RP2350_EPPB_BASE 0xe0080000
#define RP2350_EPPB_SIZE 0x1000

#define RP2350_CLOCKS_BASE 0x40010000
#define RP2350_RESETS_BASE 0x40020000
#define RP2350_XOSC_BASE 0x40048000
#define RP2350_UART0_BASE 0x40070000
#define RP2350_TIMER0_BASE 0x400b0000
#define RP2350_TIMER1_BASE 0x400b8000
#define RP2350_UART1_BASE 0x40078000
#define RP2350_PLL_SYS_BASE 0x40050000
#define RP2350_PLL_USB_BASE 0x40058000
#define RP2350_TICKS_BASE 0x40108000
#define RP2350_BOOTRAM_BASE 0x400e0000
#define RP2350_SIO_BASE 0xd0000000
#define RP2350_SIO_NONSEC_BASE 0xd0020000

#define RP2350_SYSCLK_HZ 150000000
#define RP2350_REFCLK_HZ 1000000

struct RP2350State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m[RP2350_NUM_CORES];
    UnimplementedDeviceState eppb[RP2350_NUM_CORES];
    RP2350ResetsState resets;
    RP2350SIOState sio;
    RP2350RCPState rcp;
    RP2350BootRAMState bootram;
    RP2350DCPState dcp;
    RP2350XIPState xip;
    RP2350ClkRegsState clocks;
    RP2350ClkRegsState xosc;
    RP2350ClkRegsState pll_sys;
    RP2350ClkRegsState pll_usb;
    RP2350ClkRegsState ticks;
    RP2350TimerState timer[RP2350_NUM_TIMERS];
    PL011State uart[RP2350_NUM_UARTS];
    /* The UARTs' register windows plus their atomic aliases. */
    MemoryRegion uart_alias[RP2350_NUM_UARTS];
    /* Core 0's SIO views as seen from system memory (debug, qtest). */
    MemoryRegion sio_sysmem[2];

    MemoryRegion rom;
    MemoryRegion sram;

    MemoryRegion *board_memory;
    /*
     * Each armv7m container takes its memory link as a subregion, and a
     * region can have only one container, so every core sees the board
     * memory through its own alias.
     */
    MemoryRegion core_memory[RP2350_NUM_CORES];

    uint32_t flash_size;
    uint32_t psram_size;
    uint32_t init_svtor;
    bool core1_launch;

    Clock *sysclk;
    Clock *refclk;
};

/*
 * Leave `core` with coprocessor access as the boot ROM hands it to user
 * code. Called after the core is reset when no boot ROM runs.
 */
void rp2350_soc_boot_rom_handoff(RP2350State *s, int core);

#endif
