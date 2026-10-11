/*
 * ESP32-P4 RTC timer (LP_TIMER)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_ESP32P4_LP_TIMER_H
#define HW_TIMER_ESP32P4_LP_TIMER_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_TIMER "esp32p4-lp-timer"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPTimerState, ESP32P4_LP_TIMER)

#define ESP32P4_LP_TIMER_BASE   0x50112000
#define ESP32P4_LP_TIMER_SIZE   0x1000
#define ESP32P4_LP_TIMER_TARGETS 2

/* The ETM events, lines of "etm-event". */
enum {
    ESP32P4_LP_TIMER_EVT_CMP,
    ESP32P4_LP_TIMER_EVT_OVF,
    ESP32P4_LP_TIMER_EVTS,
};

/*
 * Inputs and outputs:
 *
 *   - Clock input "slow": LP_DYN_SLOW_CLK, which the counter counts.
 *   - Clock output "etm-tick": the counter's clock, whose every edge is
 *     the ETM event RTC_EVT_TICK.
 *   - Link "clkrst": the reset and clock control, which tells a system
 *     reset, through which the timer runs on, from a chip reset.
 *   - Sysbus IRQs 0 and 1: LP_TIMER_REG_0_INTR (target 0 and overflow, the
 *     INT_* registers) and LP_TIMER_REG_1_INTR (target 1 and overflow, the
 *     LP_INT_* registers), sources of the interrupt matrix.
 *   - Named GPIO output "lp-core-int": LP_TIMER_REG_1_INTR again, the LP
 *     core's timer interrupt, for the LP core's interrupt controller.
 *   - Named GPIO outputs "soc-wakeup" and "lp-wakeup": a pulse as the
 *     counter reaches target 0 and target 1, the PMU's wakeup of the chip
 *     and of the LP core.
 *   - Named GPIO output "etm-event": per ESP32P4_LP_TIMER_EVT_*, a pulse of
 *     RTC_EVT_CMP (target 0 reached) and RTC_EVT_OVF (the counter
 *     overflowing).
 *   - Named GPIO inputs whose events the timer can log: "hp-sys-reset"
 *     (high while the HP system is held in reset), "core-stall" (per HP
 *     core, high while it is stalled), "stall-sel"
 *     (LP_SYSTEM_SYSTIMER_STALL_SEL: which core's stall counts) and
 *     "xtal-off" (high while the PMU has the crystal powered down).
 */
#define ESP32P4_LP_TIMER_LP_CORE_INT  "lp-core-int"
#define ESP32P4_LP_TIMER_SOC_WAKEUP   "soc-wakeup"
#define ESP32P4_LP_TIMER_LP_WAKEUP    "lp-wakeup"
#define ESP32P4_LP_TIMER_ETM_EVENT    "etm-event"
#define ESP32P4_LP_TIMER_HP_SYS_RESET "hp-sys-reset"
#define ESP32P4_LP_TIMER_CORE_STALL   "core-stall"
#define ESP32P4_LP_TIMER_STALL_SEL    "stall-sel"
#define ESP32P4_LP_TIMER_XTAL_OFF     "xtal-off"

typedef struct ESP32P4LPTimerAlarm {
    ESP32P4LPTimerState *s;
    QEMUTimer timer;
    int64_t due_ns;
} ESP32P4LPTimerAlarm;

struct ESP32P4LPTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ESP32P4ClkRstState *clkrst;
    Clock *slow;
    Clock *etm_tick;
    qemu_irq irq[2];
    qemu_irq lp_core_int;
    qemu_irq soc_wakeup;
    qemu_irq lp_wakeup;
    qemu_irq etm_event[ESP32P4_LP_TIMER_EVTS];

    uint64_t target[ESP32P4_LP_TIMER_TARGETS];
    bool armed[ESP32P4_LP_TIMER_TARGETS];
    /* LP_TIMER_UPDATE_REG's enables of event logging. */
    uint32_t update;
    uint64_t buf[2];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t lp_int_raw;
    uint32_t lp_int_ena;
    uint32_t date;

    /* The 48-bit count at @sync_ns, a clock edge. */
    uint64_t count;
    int64_t sync_ns;
    /* Targets 0 and 1, and the overflow. */
    ESP32P4LPTimerAlarm alarm[ESP32P4_LP_TIMER_TARGETS + 1];

    bool hp_sys_reset;
    bool stall[ESP32P4_CLKRST_HP_CORES];
    bool stall_sel;
    bool xtal_off;
    /* In a system reset, which leaves the timer as it is. */
    bool keep;
};

#endif
