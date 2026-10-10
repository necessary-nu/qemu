/*
 * RP2350 watchdog (WATCHDOG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_WATCHDOG_RP2350_WATCHDOG_H
#define HW_WATCHDOG_RP2350_WATCHDOG_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_WATCHDOG "rp2350-watchdog"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350WatchdogState, RP2350_WATCHDOG)

#define RP2350_WATCHDOG_SCRATCH 8

#define RP2350_WATCHDOG_CHIP_RESET "chip-reset"

/* REASON bits: why the watchdog last reset the chip. */
#define RP2350_WATCHDOG_REASON_TIMER (1u << 0)
#define RP2350_WATCHDOG_REASON_FORCE (1u << 1)

/*
 * The watchdog's reset request is its single GPIO output, pulsed when the
 * counter expires or TRIGGER is written; the PSM runs the reset sequence.
 * Its named GPIO input "chip-reset", pulsed by the power manager, resets
 * the watchdog, scratch registers included, for a chip-level reset that
 * is not a system reset.
 */
struct RP2350WatchdogState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *expiry;
    qemu_irq reset_req;

    /* The TICKS block's WATCHDOG tick, which clocks the counter. */
    Clock *tick;

    /* CTRL's ENABLE and PAUSE_* bits. */
    uint32_t ctrl;
    /* The counter as of virtual time sync_ns; whether it has counted since. */
    uint32_t time;
    int64_t sync_ns;
    bool running;
    uint32_t reason;
    uint32_t scratch[RP2350_WATCHDOG_SCRATCH];
};

#endif
