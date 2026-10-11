/*
 * ESP32-P4 capacitive touch sensor (LP_TOUCH)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_TOUCH_H
#define HW_MISC_ESP32P4_TOUCH_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkwait.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_TOUCH "esp32p4-touch"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4TouchState, ESP32P4_TOUCH)

#define ESP32P4_TOUCH_BASE      0x50128000
#define ESP32P4_TOUCH_SIZE      0x1000

/*
 * The registers have slots for channels 0 to 14; touch pads T1 to T14,
 * GPIO2 to GPIO15, are channels 1 to 14. 15 names no channel.
 */
#define ESP32P4_TOUCH_SLOTS     15
#define ESP32P4_TOUCH_FIRST     1
#define ESP32P4_TOUCH_NULL      15
/* The frequency modes of frequency hopping, and the proximity channels. */
#define ESP32P4_TOUCH_FREQS     3
#define ESP32P4_TOUCH_PROX      3

/*
 * Inputs and outputs
 * ==================
 *
 *   - Clock inputs: "fast", LP_DYN_FAST_CLK, which runs the touch FSM
 *     and its counter; "slow", LP_DYN_SLOW_CLK, which the touch timer
 *     counts.
 *   - Named GPIO inputs: ESP32P4_TOUCH_RESET, LPPERI_RST_EN_LP_TOUCH,
 *     which resets the FSM and its data; ESP32P4_TOUCH_SLEEP, high while
 *     the PMU has the chip in LP_SLEEP, when only the sleep channel is
 *     measured; ESP32P4_TOUCH_TIMER_UPDATE, the PMU's pulse as software
 *     writes PMU_TOUCH_PWR_CNTL.
 *   - Named GPIO output ESP32P4_TOUCH_WAKEUP: the PMU's wakeup source 11,
 *     high while a channel whose output TOUCH_OUTEN enables, or the sleep
 *     channel, is touched.
 *   - Sysbus IRQ 0: LP_TOUCH_INTR (interrupt matrix source 14).
 *   - Links: "lp-ana", LP_ANALOG_PERI, whose TOUCH_* registers configure
 *     the sensor; "pmu", whose PMU_TOUCH_PWR_CNTL configures its timer.
 *
 * Each pad's capacitance, in femtofarads, is a property, "ch1-ff" to
 * "ch14-ff" (GPIO2 to GPIO15), 10 pF by default, settable with -global
 * before the machine starts and with qom-set while it runs, a finger
 * adding to it:
 *
 *   -global esp32p4-touch.ch3-ff=12500
 */
#define ESP32P4_TOUCH_RESET         "reset"
#define ESP32P4_TOUCH_SLEEP         "sleep"
#define ESP32P4_TOUCH_TIMER_UPDATE  "timer-update"
#define ESP32P4_TOUCH_WAKEUP        "wakeup"

typedef struct ESP32P4LPAnaState ESP32P4LPAnaState;
typedef struct ESP32P4PMUState ESP32P4PMUState;

/*
 * A channel's data, per frequency mode: the last count, the smoothed
 * count and the benchmark (both with 8 fractional bits), and the
 * proximity mode's sum and samples; and its detection: whether it is
 * touched, its debounce and negative noise counts.
 */
typedef struct ESP32P4TouchChan {
    uint32_t raw[ESP32P4_TOUCH_FREQS];
    uint32_t smooth[ESP32P4_TOUCH_FREQS];
    uint32_t bench[ESP32P4_TOUCH_FREQS];
    bool bench_valid[ESP32P4_TOUCH_FREQS];
    uint32_t prox_sum[ESP32P4_TOUCH_FREQS];
    uint32_t prox_cnt;
    uint8_t nn_cnt;
    uint8_t debounce;
    bool active;
} ESP32P4TouchChan;

/* The touch timer's phases (TRM 63.4.2.2). */
typedef enum ESP32P4TouchTimerPhase {
    ESP32P4_TOUCH_TIMER_OFF,
    ESP32P4_TOUCH_TIMER_WAIT,
    ESP32P4_TOUCH_TIMER_MEAS,
    ESP32P4_TOUCH_TIMER_SLEEP,
} ESP32P4TouchTimerPhase;

typedef enum ESP32P4TouchFsmPhase {
    ESP32P4_TOUCH_FSM_IDLE,
    ESP32P4_TOUCH_FSM_XPD,
    ESP32P4_TOUCH_FSM_MEAS,
} ESP32P4TouchFsmPhase;

struct ESP32P4TouchState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq wakeup;
    Clock *fast;
    Clock *slow;

    ESP32P4LPAnaState *lp_ana;
    ESP32P4PMUState *pmu;

    /* RTC_TOUCH's own state. */
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t date;
    bool meas_done;
    uint8_t scan_curr;
    uint16_t tmp_active;
    uint16_t tmp_inactive;
    uint32_t cur_para;

    ESP32P4TouchChan chan[ESP32P4_TOUCH_SLOTS];
    /* The sleep channel's own data set. */
    ESP32P4TouchChan slp;

    /*
     * The FSM: the channel it measures, whether as the sleep channel, as
     * the scan's or software's, and whether the touch timer started it,
     * the frequency mode and modes to
     * measure, and the measurement's counter: the cycles counted up to
     * the origin of meas_edges, from which LP_DYN_FAST_CLK's cycles are
     * counted since, and when the pulses it waits for will have come.
     */
    uint8_t fsm_phase;
    uint8_t cur_ch;
    bool cur_slp;
    bool cur_scan;
    bool by_timer;
    uint8_t cur_freq;
    uint8_t nfreq;
    uint8_t scan_last;
    ESP32P4ClkWait xpd_wait;
    QEMUTimer meas_timer;
    ESP32P4ClkCount meas_edges;
    uint64_t meas_count;
    int64_t meas_end_ns;
    /* When the timer is due: the end, or the timeout. */
    int64_t meas_due_ns;

    /* The touch timer. */
    uint8_t timer_phase;
    ESP32P4ClkWait timer_wait;
    uint32_t pwr_cntl;

    bool sleeping;
    bool in_reset;
    bool wakeup_level;
    /* A timed event is being taken: a change it makes catches up none. */
    bool in_event;

    uint32_t cap_ff[ESP32P4_TOUCH_SLOTS];
};

/*
 * The LP_ANALOG_PERI TOUCH_* register at @offset has been written: it
 * was @old and is now @val, and @wt holds the write-trigger bits written.
 */
void esp32p4_touch_ana_write(ESP32P4TouchState *s, hwaddr offset,
                             uint32_t old, uint32_t val, uint32_t wt);
/* LP_ANALOG_PERI has been reset, and the TOUCH_* registers with it. */
void esp32p4_touch_ana_reset(ESP32P4TouchState *s);
/*
 * A TOUCH_* register is about to be written: the sensor first takes the
 * events that fell due before the write, under the configuration they
 * fell due under.
 */
void esp32p4_touch_sync(ESP32P4TouchState *s);

#endif
