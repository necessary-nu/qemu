/*
 * ESP32 ULP coprocessor: the ULP FSM running from RTC slow memory
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_ULP_H
#define HW_MISC_ESP32_ULP_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "system/memory.h"

#define TYPE_ESP32_ULP "misc.esp32.ulp"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32UlpState, ESP32_ULP)

/*
 * Lines:
 * - ESP32_ULP_TIMER_EN_IN (gpio-in): RTC_CNTL_ULP_CP_SLP_TIMER_EN.
 * - ESP32_ULP_START_IN (gpio-in): SENS's software start; its rising edge
 *   starts the ULP.
 * - ESP32_ULP_WAKEUP (gpio-out): pulsed by the WAKE instruction, to
 *   RTC_CNTL's ULP wakeup request;
 * - ESP32_ULP_INT (gpio-out): pulsed by the WAKE instruction, to RTC_CNTL's
 *   ULP_CP interrupt.
 */
#define ESP32_ULP_TIMER_EN_IN   "esp32-ulp-timer-en"
#define ESP32_ULP_START_IN      "esp32-ulp-start"
#define ESP32_ULP_WAKEUP        "esp32-ulp-wakeup"
#define ESP32_ULP_INT           "esp32-ulp-int"

/*
 * Clock inputs, from RTC_CNTL:
 * - ESP32_ULP_FAST_CLK: RTC_FAST_CLK, which times the instructions.
 * - ESP32_ULP_SLOW_CLK: RTC_SLOW_CLK, which times the ULP timer and the
 *   FSM's power-up and power-down.
 */
#define ESP32_ULP_FAST_CLK      "fast-clk"
#define ESP32_ULP_SLOW_CLK      "slow-clk"

/* The FSM's phases (TRM 1.5) */
typedef enum Esp32UlpPhase {
    /* Halted, with its timer stopped */
    ESP32_ULP_OFF,
    /* Halted, the ULP timer counting the sleep period */
    ESP32_ULP_SLEEP,
    /* Powering up, then waiting ULPCP_TOUCH_START_WAIT */
    ESP32_ULP_WAKING,
    ESP32_ULP_RUNNING,
    /* Executed HALT, taking its RTC_FAST_CLK cycles */
    ESP32_ULP_HALTING,
    /* Powering down after HALT */
    ESP32_ULP_POWER_DOWN,
} Esp32UlpPhase;

#define ESP32_ULP_REGS 4

typedef struct Esp32RtcCntlState Esp32RtcCntlState;
typedef struct Esp32SensState Esp32SensState;

struct Esp32UlpState {
    SysBusDevice parent_obj;

    Esp32RtcCntlState *rtc_cntl;
    Esp32SensState *sens;
    /* The RTC peripherals' registers, for REG_RD and REG_WR */
    MemoryRegion *rtc_regs;
    AddressSpace rtc_as;

    qemu_irq wakeup;
    qemu_irq irq;
    QEMUTimer timer;
    Clock *fast_clk;
    Clock *slow_clk;

    uint32_t phase;
    bool timer_en;
    bool start_level;

    /* The programmer's model */
    uint16_t r[ESP32_ULP_REGS];
    uint8_t stage_cnt;
    bool zero;
    bool overflow;
    uint32_t pc;
    /* The SENS_ULP_CP_SLEEP_CYCn the SLEEP instruction selected */
    uint32_t sleep_sel;

    /*
     * Execution timing (RUNNING, HALTING): the next instruction starts
     * run_cycles cycles of RTC_FAST_CLK, at its current rate, after
     * run_base_ns.
     */
    int64_t run_base_ns;
    uint64_t run_cycles;
    /*
     * The RTC_SLOW_CLK-timed phases (SLEEP, WAKING, POWER_DOWN) end
     * slow_left cycles of RTC_SLOW_CLK, at its current rate, after
     * slow_base_ns.
     */
    int64_t slow_base_ns;
    uint64_t slow_left;
};

#endif
