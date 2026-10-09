/*
 * ESP32 motor control PWM (MCPWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_MCPWM_H
#define HW_MISC_ESP32_MCPWM_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32_MCPWM "esp32.mcpwm"
typedef struct Esp32McpwmState Esp32McpwmState;
DECLARE_INSTANCE_CHECKER(Esp32McpwmState, ESP32_MCPWM, TYPE_ESP32_MCPWM)

#define ESP32_MCPWM_TIMERS  3
#define ESP32_MCPWM_OPS     3
#define ESP32_MCPWM_CAPS    3
#define ESP32_MCPWM_FAULTS  3
#define ESP32_MCPWM_SYNCS   3
/* PWM0A, PWM0B, PWM1A, PWM1B, PWM2A, PWM2B */
#define ESP32_MCPWM_OUTS    (ESP32_MCPWM_OPS * 2)

/*
 * Signals, connected through the GPIO matrix by the SoC:
 *
 *   "pwm-out"  (gpio-out, ESP32_MCPWM_OUTS lines): PWMnA on 2n, PWMnB on
 *              2n + 1
 *   "sync-in"  (gpio-in, ESP32_MCPWM_SYNCS lines): SYNC0..SYNC2
 *   "fault-in" (gpio-in, ESP32_MCPWM_FAULTS lines): FAULT0..FAULT2
 *   "cap-in"   (gpio-in, ESP32_MCPWM_CAPS lines): CAP0..CAP2
 *
 * The block's interrupt is its sysbus IRQ 0. Its clocks are "f160m"
 * (PLL_F160M_CLK, from which PWM_clk is divided) and "apb" (APB_CLK, which
 * drives the capture timer).
 */
#define ESP32_MCPWM_OUT         "pwm-out"
#define ESP32_MCPWM_SYNC_IN     "sync-in"
#define ESP32_MCPWM_FAULT_IN    "fault-in"
#define ESP32_MCPWM_CAP_IN      "cap-in"

typedef struct Esp32McpwmTimer {
    uint32_t cfg0;          /* TIMERn_CFG0: prescale, period shadow, method */
    uint32_t cfg1;          /* TIMERn_CFG1: start/stop, mode */
    uint32_t sync;          /* TIMERn_SYNC */
    uint32_t period;        /* the active period */
    uint32_t prescale;      /* the active prescale, latched at start */
    uint32_t value;
    uint32_t pre_cnt;       /* PWM_clk cycles since the last PT_clk tick */
    bool down;
    bool running;
    bool period_pending;    /* the period shadow awaits transfer */
} Esp32McpwmTimer;

/*
 * A dead-time delay element: a rising (RED) or falling (FED) edge on its
 * input reaches its output `due`, unless the input changes back first.
 */
typedef struct Esp32McpwmDelay {
    uint64_t due;
    bool in;
    bool out;
} Esp32McpwmDelay;

/* A PWM operator: generators, dead time, carrier and fault handler */
typedef struct Esp32McpwmOp {
    uint32_t stmp_cfg;
    uint32_t tstmp[2];      /* time stamp A and B shadows */
    uint32_t cmp[2];        /* active A and B */
    uint32_t gen_cfg0;
    uint32_t force;
    uint32_t gen[2];        /* GENn_A and GENn_B shadows */
    uint32_t gen_act[2];    /* active GENn_A and GENn_B */
    uint32_t dt_cfg;
    uint32_t dt_fed;        /* FED and RED shadows */
    uint32_t dt_red;
    uint32_t fed_act;
    uint32_t red_act;
    uint32_t carrier;
    uint32_t fh_cfg0;
    uint32_t fh_cfg1;
    uint32_t cntu_act;      /* active continuous force modes, A [1:0] B [3:2] */
    uint32_t pending;       /* shadows awaiting transfer, OP_PEND_* */
    bool gen_out[2];
    Esp32McpwmDelay red;
    Esp32McpwmDelay fed;
    bool car_in[2];
    uint64_t car_rise[2];   /* when the carrier's input last rose */
    bool cbc_on;
    bool ost_on;
} Esp32McpwmOp;

/*
 * Everything that evolves with PWM_clk: the registers the timers and
 * operators use and their state as of PWM_clk cycle `now`. It is plain
 * data so that the model can run a copy forward to look ahead.
 */
typedef struct Esp32McpwmCore {
    uint64_t now;
    Esp32McpwmTimer timer[ESP32_MCPWM_TIMERS];
    Esp32McpwmOp op[ESP32_MCPWM_OPS];
    uint32_t synci_cfg;
    uint32_t timersel;
    uint32_t fault_detect;
    uint32_t update_cfg;
    uint32_t cap_timer_cfg;
    uint32_t int_ena;
    uint32_t int_raw;
    bool fault_in[ESP32_MCPWM_FAULTS];
    bool fault_ev[ESP32_MCPWM_FAULTS];
    /* A PWM timer's sync_out reached the capture timer, last at cap_sync */
    bool cap_sync_hit;
    uint64_t cap_sync;
} Esp32McpwmCore;

struct Esp32McpwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *f160m;
    Clock *apb;
    QEMUTimer *timer;
    qemu_irq irq;
    qemu_irq out[ESP32_MCPWM_OUTS];

    Esp32McpwmCore core;
    uint32_t clk_cfg;
    uint32_t clk_reg;
    uint32_t version;

    /* PWM_clk cycle core.now corresponds to virtual time base_ns at base */
    int64_t base_ns;
    uint64_t base_cycle;

    /* GPIO SYNCn levels as received, before inversion */
    bool sync_in[ESP32_MCPWM_SYNCS];

    /* Capture: the timer is cap_anchor at cap_anchor_ns, counting APB_CLK */
    uint32_t cap_phase;
    uint32_t cap_anchor;
    int64_t cap_anchor_ns;
    uint32_t cap_cfg[ESP32_MCPWM_CAPS];
    uint32_t cap_val[ESP32_MCPWM_CAPS];
    uint32_t cap_status;
    bool cap_in[ESP32_MCPWM_CAPS];      /* CAPn as received */
    bool cap_x[ESP32_MCPWM_CAPS];       /* after the input inversion */
    bool cap_pre_out[ESP32_MCPWM_CAPS]; /* after the prescaler */
    uint32_t cap_pre_cnt[ESP32_MCPWM_CAPS];

    /* Output lines as last driven, and edge coalescing */
    uint8_t out_level[ESP32_MCPWM_OUTS];
    uint8_t irq_level;
    int64_t last_out_ns;
    uint32_t min_edge_ns;
    /* When the timer is due, and whether its callback is running */
    int64_t due_ns;
    bool in_timer_cb;
    /* The latest moment the block has acted at */
    int64_t seen_ns;
    bool updating;
    bool update_pending;
};

#endif
