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
#include "hw/dma/rp2350_dma.h"
#include "hw/gpio/rp2350_gpio.h"
#include "hw/misc/rp2350_accessctrl.h"
#include "hw/misc/rp2350_bootram.h"
#include "hw/misc/rp2350_busctrl.h"
#include "hw/misc/rp2350_clocks.h"
#include "hw/misc/rp2350_coresight.h"
#include "hw/misc/rp2350_coresight_trace.h"
#include "hw/misc/rp2350_dcp.h"
#include "hw/misc/rp2350_eppb.h"
#include "hw/misc/rp2350_hstx.h"
#include "hw/misc/rp2350_psm.h"
#include "hw/misc/rp2350_pwm.h"
#include "hw/misc/rp2350_otp.h"
#include "hw/misc/rp2350_rcp.h"
#include "hw/misc/rp2350_resets.h"
#include "hw/misc/rp2350_rosc.h"
#include "hw/misc/rp2350_sha256.h"
#include "hw/misc/rp2350_sio.h"
#include "hw/misc/rp2350_sysregs.h"
#include "hw/misc/rp2350_trng.h"
#include "hw/misc/rp2350_xip.h"
#include "hw/misc/unimp.h"
#include "hw/timer/rp2350_timer.h"
#include "hw/watchdog/rp2350_watchdog.h"
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
#define RP2350_PWM_IRQ_WRAP_0 8
#define RP2350_PWM_IRQ_WRAP_1 9
#define RP2350_DMA_IRQ_0 10
#define RP2350_USBCTRL_IRQ 14
#define RP2350_IO_IRQ_BANK0 21
#define RP2350_IO_IRQ_BANK0_NS 22
#define RP2350_IO_IRQ_QSPI 23
#define RP2350_IO_IRQ_QSPI_NS 24
#define RP2350_SIO_IRQ_FIFO 25
#define RP2350_SIO_IRQ_BELL 26
#define RP2350_SIO_IRQ_FIFO_NS 27
#define RP2350_SIO_IRQ_BELL_NS 28
#define RP2350_SIO_IRQ_MTIMECMP 29
#define RP2350_CLOCKS_IRQ 30
#define RP2350_UART0_IRQ 33
#define RP2350_UART1_IRQ 34
#define RP2350_OTP_IRQ 38
#define RP2350_TRNG_IRQ 39
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

#define RP2350_SYSINFO_BASE 0x40000000
#define RP2350_SYSCFG_BASE 0x40008000
#define RP2350_CLOCKS_BASE 0x40010000
#define RP2350_PSM_BASE 0x40018000
#define RP2350_RESETS_BASE 0x40020000
#define RP2350_IO_BANK0_BASE 0x40028000
#define RP2350_IO_QSPI_BASE 0x40030000
#define RP2350_PADS_BANK0_BASE 0x40038000
#define RP2350_PADS_QSPI_BASE 0x40040000
#define RP2350_XOSC_BASE 0x40048000
#define RP2350_BUSCTRL_BASE 0x40068000
#define RP2350_UART0_BASE 0x40070000
#define RP2350_PWM_BASE 0x400a8000
#define RP2350_TIMER0_BASE 0x400b0000
#define RP2350_TIMER1_BASE 0x400b8000
#define RP2350_UART1_BASE 0x40078000
#define RP2350_PLL_SYS_BASE 0x40050000
#define RP2350_PLL_USB_BASE 0x40058000
#define RP2350_TICKS_BASE 0x40108000
#define RP2350_CORESIGHT_PERIPH_BASE 0x40140000
#define RP2350_CORESIGHT_TRACE_BASE 0x50700000
#define RP2350_WATCHDOG_BASE 0x400d8000
#define RP2350_BOOTRAM_BASE 0x400e0000
#define RP2350_DFT_BASE 0x40150000
#define RP2350_GLITCH_DETECTOR_BASE 0x40158000
#define RP2350_TBMAN_BASE 0x40160000
#define RP2350_ROSC_BASE 0x400e8000
#define RP2350_TRNG_BASE 0x400f0000
#define RP2350_SHA256_BASE 0x400f8000
#define RP2350_HSTX_CTRL_BASE 0x400c0000
#define RP2350_HSTX_FIFO_BASE 0x50600000
#define RP2350_SIO_BASE 0xd0000000
#define RP2350_SIO_NONSEC_BASE 0xd0020000

#define RP2350_SYSCLK_HZ 150000000
#define RP2350_REFCLK_HZ 1000000
/*
 * clk_ref, which the TICKS generators divide. Clock frequencies are not
 * otherwise modelled: this is both the ring oscillator's nominal rate,
 * which clk_ref runs from at reset, and a Pico 2's 12 MHz crystal, which
 * pico-sdk switches it to.
 */
#define RP2350_CLK_REF_HZ 12000000

struct RP2350State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m[RP2350_NUM_CORES];
    RP2350EPPBState eppb[RP2350_NUM_CORES];
    RP2350AccessCtrlState accessctrl;
    Notifier accessctrl_notifier;
    RP2350ResetsState resets;
    RP2350PSMState psm;
    RP2350WatchdogState watchdog;
    RP2350SIOState sio;
    RP2350GPIOState gpio;
    RP2350RCPState rcp;
    RP2350BootRAMState bootram;
    RP2350BusCtrlState busctrl;
    RP2350DCPState dcp;
    RP2350XIPState xip;
    RP2350ClkRegsState clocks;
    RP2350ClkRegsState xosc;
    RP2350ClkRegsState pll_sys;
    RP2350ClkRegsState pll_usb;
    RP2350ClkRegsState ticks;
    RP2350SysInfoState sysinfo;
    RP2350SysCfgState syscfg;
    RP2350TBManState tbman;
    RP2350GlitchDetectorState glitch_detector;
    RP2350DFTState dft;
    RP2350ROSCState rosc;
    RP2350TRNGState trng;
    RP2350SHA256State sha256;
    RP2350PWMState pwm;
    RP2350HSTXState hstx;
    RP2350TimerState timer[RP2350_NUM_TIMERS];
    PL011State uart[RP2350_NUM_UARTS];
    RP2350CoreSightState coresight;
    RP2350CoreSightTraceState coresight_trace;
    RP2350OTPState otp;
    RP2350DMAState dma;
    /* The UARTs' register windows plus their atomic aliases. */
    MemoryRegion uart_alias[RP2350_NUM_UARTS];
    /* Core 0's SIO views as seen from system memory (debug, qtest). */
    MemoryRegion sio_sysmem[2];
    /* Core 0's EPPB as seen from system memory (debug, qtest). */
    MemoryRegion eppb_sysmem;

    MemoryRegion rom;
    MemoryRegion sram;

    MemoryRegion *board_memory;

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

/*
 * The input for IRQ `n` of core `core`. Every interrupt source reaches a
 * core through this, never straight to its NVIC, so that the core's NMI
 * mask sees it.
 */
qemu_irq rp2350_soc_core_irq(RP2350State *s, int core, int n);

#endif
