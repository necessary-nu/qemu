/*
 * ESP32-P4 RTC watchdog and super watchdog (LP_WDT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_ESP32P4_LP_WDT_H
#define HW_TIMER_ESP32P4_LP_WDT_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_WDT "esp32p4-lp-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPWdtState, ESP32P4_LP_WDT)

#define ESP32P4_LP_WDT_BASE     0x50116000
#define ESP32P4_LP_WDT_SIZE     0x1000
#define ESP32P4_LP_WDT_STAGES   4

/*
 * Inputs and outputs:
 *
 *   - Clock inputs "slow", LP_DYN_SLOW_CLK, which the RTC watchdog counts,
 *     and "swd", the RC_SLOW oscillator, which the super watchdog counts.
 *   - Links "clkrst", the reset and clock control, which carries out the
 *     watchdogs' resets and records their causes, and "efuse", whose
 *     EFUSE_WDT_DELAY_SEL scales the RTC watchdog's stage 0 and whose
 *     EFUSE_DIS_SWD disables the super watchdog.
 *   - Sysbus IRQ 0: LP_WDT_INTR, the RTC watchdog's and the super
 *     watchdog's interrupts.
 *   - Named GPIO input "spi-boot": high when the chip booted in SPI boot
 *     mode, in which the RTC watchdog's flash boot protection runs.
 *   - Named GPIO input "sleep": high while the chip sleeps, in which the
 *     RTC watchdog pauses if RTC_WDT_PAUSE_IN_SLP is set; for the PMU.
 *   - Named GPIO input "swd-reset-en": high while LP_SYSTEM's FIB control
 *     lets the super watchdog reset the system (LP_FIB_SEL bit 2 clear and
 *     DIG_FIB bit 2 set, or LP_FIB_SEL bit 2 set and the analog state
 *     ANA_FIB bit 2 set, as from reset); for LP_SYS. High, as the analog
 *     state has it, until LP_SYS drives it.
 *   - Named GPIO output "cpu-reset": per HP core, a pulse of the RTC
 *     watchdog's CPU reset of it, to the reset and clock control's
 *     "rwdt-core-reset".
 */
#define ESP32P4_LP_WDT_SPI_BOOT  "spi-boot"
#define ESP32P4_LP_WDT_SLEEP     "sleep"
#define ESP32P4_LP_WDT_SWD_RESET_EN "swd-reset-en"
#define ESP32P4_LP_WDT_CPU_RESET "cpu-reset"

typedef struct ESP32P4LPWdtCounter {
    QEMUTimer timer;
    /* The count at @sync_ns, whether it counts since then, and when due. */
    uint64_t count;
    int64_t sync_ns;
    bool running;
    int64_t due_ns;
} ESP32P4LPWdtCounter;

struct ESP32P4LPWdtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ESP32P4ClkRstState *clkrst;
    ESP32P4EfuseState *efuse;

    Clock *slow;
    Clock *swd_clk;
    qemu_irq irq;
    qemu_irq cpu_reset[ESP32P4_CLKRST_HP_CORES];

    uint32_t config0;
    uint32_t hold[ESP32P4_LP_WDT_STAGES];
    uint32_t wkey;
    uint32_t swd_config;
    uint32_t swd_wkey;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t date;

    /* The RTC watchdog's stage, its count in it, and the super watchdog's. */
    uint32_t stage;
    ESP32P4LPWdtCounter rwdt;
    ESP32P4LPWdtCounter swd;
    /* SWD_RESET_FLAG, which a system reset the super watchdog caused sets. */
    bool swd_reset_flag;

    bool spi_boot;
    bool sleep;
    bool swd_reset_en;
};

/*
 * A write of SWD_RST_FLAG_CLR, which needs no key: the boot ROM's main()
 * makes it on HP core 0 as it boots after any reset (all but a light
 * sleep's wakeup), and the machine makes it for a program -kernel starts
 * in the ROM's place.
 */
void esp32p4_lp_wdt_swd_rst_flag_clr(ESP32P4LPWdtState *s);

#endif
