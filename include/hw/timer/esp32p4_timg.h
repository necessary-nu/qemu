/*
 * ESP32-P4 timer group (TIMG): two general-purpose timers, the MWDT and
 * the RTC calibration unit
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_ESP32P4_TIMG_H
#define HW_TIMER_ESP32P4_TIMG_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_TIMG "esp32p4-timg"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4TimgState, ESP32P4_TIMG)

#define ESP32P4_TIMG0_BASE      0x500c2000
#define ESP32P4_TIMG1_BASE      0x500c3000
#define ESP32P4_TIMG_SIZE       0x1000
#define ESP32P4_TIMG_TIMERS     2
#define ESP32P4_TIMG_WDT_STAGES 4

/* The ETM tasks of each timer, lines timer * ESP32P4_TIMG_TASKS + task. */
enum {
    ESP32P4_TIMG_TASK_CNT_START,
    ESP32P4_TIMG_TASK_ALARM_START,
    ESP32P4_TIMG_TASK_CNT_STOP,
    ESP32P4_TIMG_TASK_CNT_RELOAD,
    ESP32P4_TIMG_TASK_CNT_CAP,
    ESP32P4_TIMG_TASKS,
};

/*
 * Inputs and outputs:
 *
 *   - Clock inputs "t0" and "t1", the timers' function clocks
 *     (TIMERGRPn_Tx_CLK); "wdt", the MWDT's (TIMERGRPn_WDT_CLK); "xtal",
 *     XTAL_CLK, the calibration unit's reference; and "cal", the clock it
 *     measures (TIMERGRP0_TGRT_CLK, TIMG0_CALI_CLK; timer group 1 has
 *     none).
 *   - Link "clkrst": the reset and clock control, which carries out the
 *     MWDT's HP core reset.
 *   - Property "group": the timer group, 0 or 1.
 *   - Sysbus IRQs 0 to 2: TGn_T0_INT, TGn_T1_INT and TGn_WDT_INT.
 *   - Named GPIO input "spi-boot": high when the chip booted in SPI boot
 *     mode, in which MWDT0's flash boot protection runs.
 *   - Named GPIO input "etm-task": the timers' ETM tasks, ESP32P4_TIMG_TASKS
 *     per timer (TGn_TASK_CNT_START_TIMERx, ...), each acting on a rising
 *     edge while TIMG_ETM_EN is set.
 *   - Named GPIO output "etm-event": per timer, a pulse of
 *     TGn_EVT_CNT_CMP_TIMERx as it alarms while TIMG_ETM_EN is set.
 *   - Named GPIO output "cpu-reset": per HP core, a pulse of the MWDT's CPU
 *     reset of it, to the reset and clock control's "mwdt-core-reset".
 */
#define ESP32P4_TIMG_SPI_BOOT   "spi-boot"
#define ESP32P4_TIMG_ETM_TASK   "etm-task"
#define ESP32P4_TIMG_ETM_EVENT  "etm-event"
#define ESP32P4_TIMG_CPU_RESET  "cpu-reset"

typedef struct ESP32P4TimgTimer {
    ESP32P4TimgState *s;
    Clock *clk;
    QEMUTimer timer;

    /* TIMG_TxCONFIG_REG as it reads. */
    uint32_t config;
    uint64_t alarm;
    uint64_t load;
    uint64_t latched;

    /*
     * The count as of function clock cycle @synced, counted from when the
     * timer last started or its clock changed (@edges), @pre function
     * clock cycles into the current cycle of TB_CLK, and whether it counts
     * since then. While it is stopped, @edges' origin is the time of its
     * last change, from which a start counts.
     */
    uint64_t count;
    uint32_t pre;
    ESP32P4ClkCount edges;
    uint64_t synced;
    bool running;
    /* When the scheduled alarm falls due. */
    int64_t due_ns;
} ESP32P4TimgTimer;

typedef struct ESP32P4TimgWdt {
    Clock *clk;
    QEMUTimer timer;

    /* TIMG_WDTCONFIG0..5_REG and TIMG_WDTWPROTECT_REG as they read. */
    uint32_t config0;
    uint32_t config1;
    uint32_t hold[ESP32P4_TIMG_WDT_STAGES];
    uint32_t wkey;
    /* The configuration TIMG_WDT_CONF_UPDATE_EN last put in effect. */
    uint32_t act_config0;
    uint32_t act_prescale;
    uint32_t act_hold[ESP32P4_TIMG_WDT_STAGES];

    /*
     * The current stage and its count as of function clock cycle @synced,
     * counted from when the MWDT last started or its clock changed
     * (@edges), @pre function clock cycles into the current cycle of the
     * MWDT's clock.
     */
    uint32_t stage;
    uint32_t count;
    uint32_t pre;
    ESP32P4ClkCount edges;
    uint64_t synced;
    bool running;
    /* When the current stage times out. */
    int64_t due_ns;
} ESP32P4TimgWdt;

typedef struct ESP32P4TimgCal {
    QEMUTimer timer;
    QEMUTimer timeout_timer;

    /* TIMG_RTCCALICFG_REG's and RTCCALICFG2's R/W fields as stored. */
    uint32_t cfg;
    uint32_t cfg2;
    bool rdy;
    bool vld;
    bool timeout;
    uint32_t value;

    /*
     * The measurement under way: one-shot or cycling, whether the measured
     * clock's first edge has come, the cycles to count, the measured
     * clock's cycles counted since it did, up to the measured clock's
     * cycle @first (of the timer group's @cal_edges; the first edge's,
     * before it comes), XTAL_CLK's edges by that first edge (of the timer
     * group's @xtal_base on), and the time of its next event; and for its
     * timeout, XTAL_CLK's cycles since it began, up to XTAL_CLK's cycle
     * @xtal_first (of @xtal_edges).
     */
    bool active;
    bool oneshot;
    bool counting;
    uint32_t target;
    uint64_t cycles;
    uint64_t xtal_at_first;
    uint64_t first;
    int64_t event_ns;
    uint64_t since_start;
    uint64_t xtal_first;
    /* A cycling measurement waits after a timeout before starting again. */
    bool restart_pending;
} ESP32P4TimgCal;

struct ESP32P4TimgState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ESP32P4ClkRstState *clkrst;
    uint32_t group;

    Clock *xtal;
    Clock *cal_clk;
    /*
     * XTAL_CLK's and the measured clock's cycles since their last change,
     * and XTAL_CLK's before it, since the timer group's reset.
     */
    ESP32P4ClkCount xtal_edges;
    ESP32P4ClkCount cal_edges;
    uint64_t xtal_base;
    qemu_irq irq[ESP32P4_TIMG_TIMERS + 1];
    qemu_irq etm_event[ESP32P4_TIMG_TIMERS];
    qemu_irq cpu_reset[ESP32P4_CLKRST_HP_CORES];

    ESP32P4TimgTimer t[ESP32P4_TIMG_TIMERS];
    ESP32P4TimgWdt wdt;
    ESP32P4TimgCal cal;

    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t date;
    uint32_t regclk;
    bool spi_boot;
    /* Counts the block's resets, for actions that reset it. */
    uint32_t reset_count;
};

#endif
