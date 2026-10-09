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
#include "hw/adc/rp2350_adc.h"
#include "hw/char/pl011.h"
#include "hw/core/clock.h"
#include "hw/dma/rp2350_dma.h"
#include "hw/gpio/rp2350_gpio.h"
#include "hw/i2c/rp2350_i2c.h"
#include "hw/misc/rp2350_accessctrl.h"
#include "hw/misc/rp2350_bootram.h"
#include "hw/misc/rp2350_busctrl.h"
#include "hw/misc/rp2350_clocks.h"
#include "hw/misc/rp2350_coresight.h"
#include "hw/misc/rp2350_coresight_trace.h"
#include "hw/misc/rp2350_dcp.h"
#include "hw/misc/rp2350_eppb.h"
#include "hw/misc/rp2350_m33_debug.h"
#include "hw/misc/rp2350_hstx.h"
#include "hw/misc/rp2350_powman.h"
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
#include "hw/ssi/pl022.h"
#include "hw/timer/rp2350_timer.h"
#include "hw/watchdog/rp2350_watchdog.h"
#include "qom/object.h"

#define TYPE_RP2350_SOC "rp2350-soc"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350State, RP2350_SOC)

#define RP2350_NUM_CORES 2
#define RP2350_NUM_IRQS 52
#define RP2350_NUM_UARTS 2
#define RP2350_NUM_SPIS 2
#define RP2350_NUM_TIMERS 2
#define RP2350_NUM_I2C 2
#define RP2350_MPU_REGIONS 8
/* CPUID: Arm Cortex-M33 r1p0, where QEMU's cortex-m33 is r0p3. */
#define RP2350_M33_CPUID 0x411fd210

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
#define RP2350_SPI0_IRQ 31
#define RP2350_SPI1_IRQ 32
#define RP2350_UART0_IRQ 33
#define RP2350_UART1_IRQ 34
#define RP2350_ADC_IRQ_FIFO 35
#define RP2350_I2C0_IRQ 36
#define RP2350_I2C1_IRQ 37
#define RP2350_OTP_IRQ 38
#define RP2350_TRNG_IRQ 39
#define RP2350_POWMAN_POW_IRQ 44
#define RP2350_POWMAN_TIMER_IRQ 45
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
/*
 * The SRAM power domains: SRAM0 is banks 0-3, the lower half of the
 * striped window; SRAM1 is banks 4-7 and the SRAM8/9 scratch banks.
 */
#define RP2350_SRAM0_DOMAIN_SIZE (256 * KiB)
#define RP2350_SRAM1_DOMAIN_SIZE (RP2350_SRAM_SIZE - RP2350_SRAM0_DOMAIN_SIZE)

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
#define RP2350_SPI0_BASE 0x40080000
#define RP2350_SPI1_BASE 0x40088000
#define RP2350_I2C0_BASE 0x40090000
#define RP2350_I2C1_BASE 0x40098000
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
#define RP2350_POWMAN_BASE 0x40100000
#define RP2350_USB_DPRAM_BASE 0x50100000
#define RP2350_USB_DPRAM_SIZE (4 * KiB)
#define RP2350_SIO_BASE 0xd0000000
#define RP2350_SIO_NONSEC_BASE 0xd0020000

#define RP2350_SYSCLK_HZ 150000000
/*
 * SYST_CALIB as the RP2350 hardwires it on both cores: TENMS is 100,000
 * (datasheet 8.5.1, "Tick generators"), with NOREF and SKEW clear. The
 * SysTick reference clock is the core's TICKS generator (PROC0 or PROC1),
 * not a fixed clock.
 */
#define RP2350_SYST_CALIB 100000
/*
 * clk_ref, which the TICKS generators divide. Clock frequencies are not
 * otherwise modelled: this is both the ring oscillator's nominal rate,
 * which clk_ref runs from at reset, and a Pico 2's 12 MHz crystal, which
 * pico-sdk switches it to.
 */
#define RP2350_CLK_REF_HZ 12000000
/*
 * clk_peri, the UARTs' UARTCLK, at the clk_sys frequency pico-sdk runs it
 * from. Its divider and enable are not modelled.
 */
#define RP2350_CLK_PERI_HZ 150000000
/* clk_adc, as pico-sdk sets it up from PLL_USB. */
#define RP2350_CLK_ADC_HZ 48000000

/* Device models and bus windows per RESETS subsystem. */
#define RP2350_RESET_MAX_DEVICES 4
#define RP2350_RESET_MAX_WINDOWS 2

/* What a subsystem's bus window answers while the subsystem is in reset. */
typedef struct RP2350ResetGate {
    MemoryRegion mr;
    const char *name;
} RP2350ResetGate;

struct RP2350State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m[RP2350_NUM_CORES];
    RP2350EPPBState eppb[RP2350_NUM_CORES];
    RP2350M33DebugState m33_debug[RP2350_NUM_CORES];
    RP2350AccessCtrlState accessctrl;
    Notifier accessctrl_notifier;
    RP2350ResetsState resets;
    RP2350PSMState psm;
    RP2350WatchdogState watchdog;
    RP2350PowmanState powman;
    RP2350SIOState sio;
    RP2350GPIOState gpio;
    RP2350RCPState rcp;
    RP2350BootRAMState bootram;
    RP2350BusCtrlState busctrl;
    RP2350DCPState dcp;
    RP2350XIPState xip;
    RP2350ClkRegsState clocks;
    RP2350XOSCState xosc;
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
    RP2350ADCState adc;
    RP2350HSTXState hstx;
    RP2350TimerState timer[RP2350_NUM_TIMERS];
    PL011State uart[RP2350_NUM_UARTS];
    PL022State spi[RP2350_NUM_SPIS];
    /* Wires the chip selects of the SSI devices on the SPI buses. */
    Notifier spi_cs_notifier;
    DesignWareI2CState i2c[RP2350_NUM_I2C];
    RP2350CoreSightState coresight;
    RP2350CoreSightTraceState coresight_trace;
    RP2350OTPState otp;
    RP2350DMAState dma;
    /* The UARTs' register windows plus their atomic aliases. */
    MemoryRegion uart_alias[RP2350_NUM_UARTS];
    /* The SPI controllers' register windows plus their atomic aliases. */
    MemoryRegion spi_alias[RP2350_NUM_SPIS];
    /* The I2C controllers' register windows plus their atomic aliases. */
    MemoryRegion i2c_alias[RP2350_NUM_I2C];
    /* Core 0's SIO views as seen from system memory (debug, qtest). */
    MemoryRegion sio_sysmem[2];
    /* Core 0's EPPB as seen from system memory (debug, qtest). */
    MemoryRegion eppb_sysmem;

    /* The device models of each RESETS subsystem, and its bus gates. */
    DeviceState *reset_dev[RP2350_NUM_RESETS][RP2350_RESET_MAX_DEVICES];
    RP2350ResetGate reset_gate[RP2350_NUM_RESETS][RP2350_RESET_MAX_WINDOWS];

    MemoryRegion rom;
    MemoryRegion sram;
    /* Over SRAM0 and SRAM1 while their power domain is down. */
    MemoryRegion sram_off[2];
    /* SRAM, for clearing a domain that powers down. */
    AddressSpace sram_as;
    MemoryRegion usb_dpram;

    MemoryRegion *board_memory;

    /*
     * DORMANT: the wake events' levels (the GPIO banks' dormant_wake
     * interrupt and the AON alarm), whether clk_sys is stopped, and which
     * cores were halted because it stopped.
     */
    bool dormant_wake[2];
    bool clk_sys_stopped;
    bool core_clock_halted[RP2350_NUM_CORES];

    uint32_t flash_size;
    uint32_t psram_size;
    uint32_t init_svtor;
    bool core1_launch;

    /*
     * Cores whose AIRCR.SYSRESETREQ is waiting for its warm reset, which
     * runs in a bottom half once every vCPU has stopped.
     */
    uint32_t sysresetreq_pending;
    QEMUBH *sysresetreq_bh;

    Clock *sysclk;
    /* Each core's SysTick reference: its TICKS PROC0/PROC1 generator. */
    Clock *refclk[RP2350_NUM_CORES];
    Clock *periclk;
    Clock *adcclk;
};

/*
 * Leave `core` with coprocessor access as the boot ROM hands it to user
 * code. Called after the core is reset when no boot ROM runs.
 */
void rp2350_soc_boot_rom_handoff(RP2350State *s, int core);

/*
 * Make `dev` part of RESETS subsystem `reset` (an RP2350_RESET_* number):
 * while the subsystem is in reset, `dev` is held in its Resettable reset,
 * entering it when the subsystem's reset is asserted and leaving it when
 * RESET_DONE sets. A device model must not operate while held: it starts
 * timers and takes external input only once out of reset. Bus accesses to
 * the subsystem's registers are answered by the SoC while it is in reset,
 * whether or not it has a device model. Call before the SoC is realized
 * or from its realize.
 */
void rp2350_soc_attach_reset(RP2350State *s, int reset, DeviceState *dev);

/*
 * The input for IRQ `n` of core `core`. Every interrupt source reaches a
 * core through this, never straight to its NVIC, so that the core's NMI
 * mask sees it.
 */
qemu_irq rp2350_soc_core_irq(RP2350State *s, int core, int n);

/*
 * Reset the machine as the chip-level reset the power manager has
 * pending: a cold reset of `type` becomes that reset's type (see
 * rp2350_powman_next_reset_type()). Processors stay off while the
 * switched core is unpowered; otherwise, with no ROM executing, core 0
 * starts as the boot ROM would hand it over.
 */
void rp2350_soc_system_reset(RP2350State *s, ResetType type);

#endif
