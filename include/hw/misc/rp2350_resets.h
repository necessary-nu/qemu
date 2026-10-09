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
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_RESETS "rp2350-resets"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350ResetsState, RP2350_RESETS)

/* The subsystems RESETS controls, by bit number (pico-sdk resets.h). */
enum {
    RP2350_RESET_ADC,
    RP2350_RESET_BUSCTRL,
    RP2350_RESET_DMA,
    RP2350_RESET_HSTX,
    RP2350_RESET_I2C0,
    RP2350_RESET_I2C1,
    RP2350_RESET_IO_BANK0,
    RP2350_RESET_IO_QSPI,
    RP2350_RESET_JTAG,
    RP2350_RESET_PADS_BANK0,
    RP2350_RESET_PADS_QSPI,
    RP2350_RESET_PIO0,
    RP2350_RESET_PIO1,
    RP2350_RESET_PIO2,
    RP2350_RESET_PLL_SYS,
    RP2350_RESET_PLL_USB,
    RP2350_RESET_PWM,
    RP2350_RESET_SHA256,
    RP2350_RESET_SPI0,
    RP2350_RESET_SPI1,
    RP2350_RESET_SYSCFG,
    RP2350_RESET_SYSINFO,
    RP2350_RESET_TBMAN,
    RP2350_RESET_TIMER0,
    RP2350_RESET_TIMER1,
    RP2350_RESET_TRNG,
    RP2350_RESET_UART0,
    RP2350_RESET_UART1,
    RP2350_RESET_USBCTRL,
    RP2350_NUM_RESETS,
};

#define RP2350_RESETS_ALL 0x1fffffff

/*
 * Time from a subsystem's reset being deasserted to the subsystem leaving
 * reset and its RESET_DONE bit setting: the reset is released through
 * synchronisers in the subsystem's clock domain, a few cycles of clk_ref.
 */
#define RP2350_RESETS_RELEASE_NS 250

/*
 * Called with `hold` true when the subsystems in `blocks` enter reset, and
 * false when they leave it. A subsystem is in reset from its reset being
 * asserted until its RESET_DONE bit sets.
 */
typedef void RP2350ResetsHoldFn(void *opaque, uint32_t blocks, bool hold);

struct RP2350ResetsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *release_timer;

    /* Subsystems the boot ROM takes out of reset (no ROM executing). */
    uint32_t rom_unreset;

    uint32_t reset;
    uint32_t wdsel;
    /* Subsystems in reset: RESET_DONE is its complement. */
    uint32_t held;
    /* When each released subsystem leaves reset; -1 when none is due. */
    int64_t release_ns[RP2350_NUM_RESETS];

    /* The subsystems currently held through hold_fn; not migrated. */
    uint32_t asserted;
    RP2350ResetsHoldFn *hold_fn;
    void *hold_opaque;
};

void rp2350_resets_set_hold_fn(RP2350ResetsState *s, RP2350ResetsHoldFn *fn,
                               void *opaque);

/*
 * Pulse the reset of the subsystems in `blocks`, as a watchdog reset does
 * for those RESETS.WDSEL selects: each restarts from its reset state, and
 * RESETS.RESET keeps its value.
 */
void rp2350_resets_pulse(RP2350ResetsState *s, uint32_t blocks);

/*
 * Reset the subsystems the boot ROM uses on its way to a flash image, and
 * leave them out of reset, as the ROM does before entering the image.
 */
void rp2350_resets_boot_rom_handoff(RP2350ResetsState *s);

#endif
