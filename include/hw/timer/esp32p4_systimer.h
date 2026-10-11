/*
 * ESP32-P4 system timer (SYSTIMER)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_ESP32P4_SYSTIMER_H
#define HW_TIMER_ESP32P4_SYSTIMER_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_SYSTIMER "esp32p4-systimer"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4SysTimerState, ESP32P4_SYSTIMER)

#define ESP32P4_SYSTIMER_BASE   0x500e2000
#define ESP32P4_SYSTIMER_SIZE   0x1000
#define ESP32P4_SYSTIMER_UNITS  2
#define ESP32P4_SYSTIMER_COMPS  3
#define ESP32P4_SYSTIMER_CORES  2

/*
 * Inputs and outputs:
 *
 *   - Clock input "clk": the function clock, SYSTIMER_CLK, which the
 *     2.5 divider turns into CNT_CLK.
 *   - Named GPIO input "core-stall": per HP core, high while it is
 *     stalled.
 *   - Sysbus IRQs 0 to 2: SYSTIMER_TARGET0..2_INT.
 *   - Named GPIO output "etm-event": per comparator, a pulse of its ETM
 *     event SYSTIMER_EVT_CNT_CMPx as it alarms while SYSTIMER_ETM_EN is
 *     set. The system timer takes no ETM tasks.
 */
#define ESP32P4_SYSTIMER_CORE_STALL "core-stall"
#define ESP32P4_SYSTIMER_ETM_EVENT  "etm-event"

typedef struct ESP32P4SysTimerUnit {
    /* The count at CNT_CLK edge @sync, and while it runs since then. */
    uint64_t value;
    uint64_t sync;
    bool running;
    uint64_t load;
    /* VALUE_HI:VALUE_LO, as the last update latched them. */
    uint64_t latched;
    bool valid;
} ESP32P4SysTimerUnit;

typedef struct ESP32P4SysTimerComp {
    ESP32P4SysTimerState *s;
    /* TARGETx_HI:TARGETx_LO and TARGETx_CONF as written. */
    uint64_t target;
    uint32_t conf;
    /* The period and alarm value COMPx_LOAD took (REAL_TARGET). */
    uint32_t period;
    uint64_t real;
    /* Comparing for an alarm: target mode alarms once. */
    bool armed;
    /* When the alarm @timer runs falls due. */
    int64_t due_ns;
    QEMUTimer timer;
} ESP32P4SysTimerComp;

struct ESP32P4SysTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;
    qemu_irq irq[ESP32P4_SYSTIMER_COMPS];
    qemu_irq etm_event[ESP32P4_SYSTIMER_COMPS];

    uint32_t conf;
    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t date;
    ESP32P4SysTimerUnit unit[ESP32P4_SYSTIMER_UNITS];
    ESP32P4SysTimerComp comp[ESP32P4_SYSTIMER_COMPS];
    bool stall[ESP32P4_SYSTIMER_CORES];

    /*
     * CNT_CLK: @clk_edges edges up to the origin of @clk_count, an edge of
     * the function clock from which it has run at its current rate.
     */
    uint64_t clk_edges;
    ESP32P4ClkCount clk_count;
};

#endif
