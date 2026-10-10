/*
 * RP2350 system timer (TIMER0, TIMER1)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_RP2350_TIMER_H
#define HW_TIMER_RP2350_TIMER_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_TIMER "rp2350-timer"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350TimerState, RP2350_TIMER)

#define RP2350_TIMER_ALARMS 4

struct RP2350TimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *alarm_timer[RP2350_TIMER_ALARMS];
    /* Virtual time at which each armed alarm's target is reached, or -1. */
    int64_t alarm_due_ns[RP2350_TIMER_ALARMS];
    qemu_irq irq[RP2350_TIMER_ALARMS];

    /* Its TICKS generator's tick, and clk_sys. */
    Clock *tick;
    Clock *clk_sys;

    /* The counter as of virtual time sync_ns, and whether it was running. */
    uint64_t count;
    int64_t sync_ns;
    bool running;

    uint32_t timelw;
    uint32_t latched_hi;
    uint32_t alarm[RP2350_TIMER_ALARMS];
    uint32_t armed;
    uint32_t dbgpause;
    uint32_t pause;
    uint32_t locked;
    uint32_t source;
    uint32_t intr;
    uint32_t inte;
    uint32_t intf;
};

#endif
