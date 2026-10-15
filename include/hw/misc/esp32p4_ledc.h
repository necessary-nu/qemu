/*
 * ESP32-P4 LED PWM controller (LEDC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LEDC_H
#define HW_MISC_ESP32P4_LEDC_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LEDC "esp32p4-ledc"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LedcState, ESP32P4_LEDC)

#define ESP32P4_LEDC_BASE           0x500d3000
#define ESP32P4_LEDC_SIZE           0x1000
#define ESP32P4_LEDC_CHANNELS       8
#define ESP32P4_LEDC_TIMERS         4
/* Each channel's duty cycle ranges in the gamma RAM */
#define ESP32P4_LEDC_RANGES         16
#define ESP32P4_LEDC_ETM_EVENTS     24
#define ESP32P4_LEDC_ETM_TASKS      68

/*
 * Signals and hooks:
 *
 *   "sig-out"   (gpio-out, ESP32P4_LEDC_CHANNELS lines): sig_outn, GPIO
 *               matrix output signal ledc_ls_sig_out_pad_outn, each change
 *               stamped with its time (hw/gpio/esp32_line.h).
 *   "etm-event" (gpio-out, ESP32P4_LEDC_ETM_EVENTS lines): the events in
 *               the order of their ETM IDs from LEDC_EVT_DUTY_CHNG_END_CH0
 *               (the channels' DUTY_CHNG_END and OVF_CNT_PLS, the timers'
 *               TIME_OVF and CMP), which is also LEDC_EVT_TASK_EN0's order
 *               of their enables; each pulsed, stamped with its time, as
 *               it happens while enabled.
 *   "etm-task"  (gpio-in, ESP32P4_LEDC_ETM_TASKS lines): the tasks in the
 *               order of their ETM IDs from LEDC_TASK_TIMER0_RES_UPDATE,
 *               each acting as it rises while LEDC_EVT_TASK_EN1/2 or EN0
 *               enables it.
 *   Clock "clk": LEDC_CLK, HP_SYS_CLKRST's "ledc" (its source mux and
 *               gate), which the timers divide. The machine maps the
 *               registers behind LEDC_APB_CLK and the block's reset.
 *   Link "gpio" and property "sig-base": the GPIO matrix, and the matrix
 *               output signal of sig_out0 (the others following it), for
 *               the channels to see whether their outputs reach a pad
 *               something watches (esp32p4_gpio_sig_out_watched()).
 *               Without the link every output is taken to reach one.
 *   sysbus IRQ 0: LEDC_INTR.
 */
#define ESP32P4_LEDC_OUT            "sig-out"
#define ESP32P4_LEDC_ETM_EVENT      "etm-event"
#define ESP32P4_LEDC_ETM_TASK       "etm-task"

typedef struct ESP32P4LedcTimer {
    uint32_t conf;          /* TIMERx_CONF, PARA_UP set while pending */
    uint32_t cmp;           /* TIMERx_CMP */
    uint32_t cnt_cap;       /* TIMERx_CNT_CAP */
    uint32_t div;           /* the divider in use, A.B as A * 256 + B */
    uint32_t res;           /* the duty resolution in use */
    bool pend_div;          /* a divider waiting for the next overflow */
    bool pend_res;          /* a resolution waiting for the next overflow */
    uint64_t pend_cycle;    /* the cycle whose start they wait for */
    bool etm_paused;        /* paused by the PAUSE and RESUME tasks */
    /*
     * The counter: count anchor_cnt of cycle anchor_cycle at anchor_ns,
     * moving one count on each of the edges q of its divided clock, exact
     * period and phase (stopped while it stands still), after edge
     * q_start, LEDC_CLK's last edge at or before the divider last
     * started, whose edges q_clk were then.
     */
    int64_t anchor_ns;
    uint64_t anchor_cycle;
    uint32_t anchor_cnt;
    ESP32P4ClkEdges q;
    int64_t q_start;
    ESP32P4ClkEdges q_clk;
    /* The first cycle whose overflow, and CMP match, are not yet seen */
    uint64_t ovf_next;
    uint64_t cmp_next;
} ESP32P4LedcTimer;

typedef struct ESP32P4LedcChannel {
    uint32_t conf0;         /* CHn_CONF0, PARA_UP set while pending */
    uint32_t hpoint;
    uint32_t duty;
    uint32_t conf1;         /* DUTY_START, cleared as it is taken */
    uint32_t gamma_conf;    /* ENTRY_NUM; PAUSE shows a paused fade */

    /* What the last update (PARA_UP) took in */
    bool active;
    uint32_t timer;
    bool sig_out_en;
    bool ovf_cnt_en;
    uint32_t act_hpoint;
    uint64_t k_dither;      /* the cycle DUTY[3:0]'s pattern counts from */

    /* An update waiting for the start of cycle load_cycle */
    bool load_pending;
    bool load_fade;
    uint64_t load_cycle;

    /*
     * The duty: during cycle k_anchor it is d_anchor; with a fade running
     * that is range `range`'s state, `steps` fades done and `count`
     * overflows since the last, the range's parameters `param` (as the
     * gamma RAM held them as it began) and entry_num ranges in all.
     */
    uint64_t k_anchor;
    uint32_t d_anchor;
    bool fading;
    bool paused;
    bool end_due;           /* DUTY_CHNG_END is still to come */
    uint32_t range;
    uint32_t steps;
    uint32_t count;
    uint32_t param;
    uint32_t entry_num;
    /* Fade pause, resume or restart waiting for the start of op_cycle */
    uint32_t op_pending;
    uint64_t op_cycle;

    /* Overflows counted towards OVF_NUM + 1 */
    uint32_t ovf_cnt;

    /*
     * The generator: its level at the start of cycles up to k_base is
     * base_level; gen_level is its level at eval_ns, out_level the output
     * as last driven.
     */
    uint64_t k_base;
    bool base_level;
    bool gen_level;
    int64_t eval_ns;
    bool out_level;
} ESP32P4LedcChannel;

struct ESP32P4LedcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;
    QEMUTimer timer;
    qemu_irq irq;
    qemu_irq out[ESP32P4_LEDC_CHANNELS];
    qemu_irq etm_event[ESP32P4_LEDC_ETM_EVENTS];
    Notifier line_source;
    ESP32P4GpioState *gpio;
    uint32_t sig_base;
    Notifier route_notifier;
    /*
     * Each channel's output reaches a pad something watches, so that its
     * edges are timed: one something but an input taking its wave watches
     * (out_watched), or one such an input takes while the channel does not
     * describe its edges (out_taken); its wave (hw/gpio/esp32_line.h).
     */
    bool out_seen[ESP32P4_LEDC_CHANNELS];
    bool out_watched[ESP32P4_LEDC_CHANNELS];
    bool out_taken[ESP32P4_LEDC_CHANNELS];
    Esp32LineWave wave[ESP32P4_LEDC_CHANNELS];

    ESP32P4LedcTimer t[ESP32P4_LEDC_TIMERS];
    ESP32P4LedcChannel ch[ESP32P4_LEDC_CHANNELS];
    uint32_t gamma[ESP32P4_LEDC_CHANNELS][ESP32P4_LEDC_RANGES];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t evt_task_en[3];
    uint32_t conf;
    uint32_t date;

    /* The latest moment the block has acted at */
    int64_t seen_ns;
    bool irq_level;
    /* Edges have been lost to a host too slow for them (logged once) */
    bool lag_logged;
    /* How far behind the timer's catch-ups end (esp32_line_caught_up()) */
    Esp32LineLag lag;
};

#endif
