/*
 * ESP32 LED PWM controller (LEDC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "qemu/notify.h"

#define TYPE_ESP32_LEDC "misc.esp32.ledc"
#define ESP32_LEDC(obj) OBJECT_CHECK(Esp32LEDCState, (obj), TYPE_ESP32_LEDC)

/* Timers and channels: the high-speed ones first, then the low-speed */
#define ESP32_LEDC_TIMER_CNT 8
#define ESP32_LEDC_CHANNEL_CNT 16
#define ESP32_LEDC_HS_CNT 8

/*
 * The channels' PWM outputs, a named gpio-out of ESP32_LEDC_CHANNEL_CNT
 * lines: LEDC_HS_SIG_OUT0-7, then LEDC_LS_SIG_OUT0-7.
 */
#define ESP32_LEDC_OUT "esp32-ledc-out"

typedef struct Esp32LedcTimer {
    uint32_t conf;
    /* The divider (10.8 fixed point) and duty resolution in effect */
    uint32_t div;
    uint32_t res;
    /*
     * A new divider and resolution wait for the overflow that ends cycle
     * pend_cycle - 1.
     */
    bool pending;
    uint64_t pend_cycle;
    /*
     * The counter: at anchor_ns it stood at anchor_cnt in cycle
     * anchor_cycle (cycles count overflows), and it ticks every q / 2^32 ns;
     * q is 0 while it stands still.
     */
    int64_t anchor_ns;
    uint32_t anchor_cnt;
    uint64_t anchor_cycle;
    uint64_t q;
    /* The first cycle whose overflow has not raised its interrupt yet */
    uint64_t ovf_next;
    QEMUTimer timer;
} Esp32LedcTimer;

typedef struct Esp32LedcChannel {
    uint32_t conf0;
    uint32_t hpoint;
    uint32_t duty;
    uint32_t conf1;

    /*
     * HPOINT and DUTY (and the fade settings, with load_fade) are taken in
     * at the start of cycle load_cycle of the channel's timer.
     */
    bool load_pending;
    bool load_fade;
    uint64_t load_cycle;

    /*
     * The generator, since it last took its settings in: act_hpoint and
     * act_duty (21.4 fixed point, the duty of cycle k_fade), stepping by
     * fade_scale every fade_cycle cycles, fade_num times, while fading.
     * Its level at the start of cycle k_base was base_level.
     */
    bool active;
    uint32_t act_hpoint;
    uint32_t act_duty;
    bool fade;
    bool fade_inc;
    uint32_t fade_scale;
    uint32_t fade_cycle;
    uint32_t fade_num;
    uint64_t k_fade;
    uint64_t k_base;
    bool base_level;
    /* DUTY_CHNG_END is still to come at the fade's end */
    bool fade_end_due;

    /* The generator's level as evaluated at eval_ns */
    bool gen_level;
    int64_t eval_ns;
    /* The output's level, and when it last changed */
    bool out_level;
    int64_t emit_ns;
    QEMUTimer timer;
} Esp32LedcChannel;

typedef struct Esp32LEDCState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[ESP32_LEDC_CHANNEL_CNT];

    /* APB_CLK, REF_TICK and RC_FAST_CLK; the SoC stops them all when gated */
    Clock *apb_clk;
    Clock *ref_tick_clk;
    Clock *rc_fast_clk;

    Esp32LedcTimer timer[ESP32_LEDC_TIMER_CNT];
    Esp32LedcChannel ch[ESP32_LEDC_CHANNEL_CNT];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf;
    uint32_t date;

    Notifier line_source;
} Esp32LEDCState;
