/*
 * ESP32 LED PWM controller (LEDC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "LED PWM Controller (LEDC)", and ESP-IDF's
 * soc/ledc_reg.h, soc/ledc_struct.h and hal/ledc_ll.h.
 *
 * Eight timers (four high-speed, four low-speed) count up from 0 to
 * 2^DUTY_RES - 1 and overflow, each tick taking LEDC_CLK_DIV = A + B/256
 * periods of its clock (B ticks in 256 take A + 1 periods, spread evenly).
 * Sixteen channels each compare a timer's count with HPOINT, where the
 * output goes high, and HPOINT + DUTY[24:4], where it goes low; DUTY[3:0]
 * makes that many cycles in 16 one tick longer. A fade steps DUTY by
 * DUTY_SCALE every DUTY_CYCLE cycles, DUTY_NUM times.
 *
 * Everything is computed from the virtual time: the counters lazily, from
 * when their settings last changed, and the outputs as functions of the
 * counters. A channel's output changes at its edges' exact times, stamped
 * on the GPIO matrix signal (see esp32_line_set_at()), but no sooner than
 * LEDC_EDGE_MIN_NS after its previous change: edges closer together than
 * that (a fast PWM, or a short pulse) are coalesced into the level at the
 * end of the interval, so something watching the pad's edges sees at most
 * one change per interval. Whenever software samples the pads the outputs
 * are brought to their exact level at that moment, so sampling a pad
 * measures the duty cycle faithfully at any frequency.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32_ledc.h"
#include "hw/gpio/esp32_gpio.h"
#include "migration/vmstate.h"

#define LEDC_CH_STRIDE          0x14
REG32(LEDC_CH_CONF0, 0x00)
    FIELD(LEDC_CH_CONF0, TIMER_SEL, 0, 2)
    FIELD(LEDC_CH_CONF0, SIG_OUT_EN, 2, 1)
    FIELD(LEDC_CH_CONF0, IDLE_LV, 3, 1)
    FIELD(LEDC_CH_CONF0, PARA_UP, 4, 1)
    FIELD(LEDC_CH_CONF0, CLK_EN, 31, 1)
REG32(LEDC_CH_HPOINT, 0x04)
REG32(LEDC_CH_DUTY, 0x08)
REG32(LEDC_CH_CONF1, 0x0c)
    FIELD(LEDC_CH_CONF1, DUTY_SCALE, 0, 10)
    FIELD(LEDC_CH_CONF1, DUTY_CYCLE, 10, 10)
    FIELD(LEDC_CH_CONF1, DUTY_NUM, 20, 10)
    FIELD(LEDC_CH_CONF1, DUTY_INC, 30, 1)
    FIELD(LEDC_CH_CONF1, DUTY_START, 31, 1)
REG32(LEDC_CH_DUTY_R, 0x10)
REG32(LEDC_TIMER_CONF, 0x140)
    FIELD(LEDC_TIMER_CONF, DUTY_RES, 0, 5)
    FIELD(LEDC_TIMER_CONF, DIV_NUM, 5, 18)
    FIELD(LEDC_TIMER_CONF, PAUSE, 23, 1)
    FIELD(LEDC_TIMER_CONF, RST, 24, 1)
    FIELD(LEDC_TIMER_CONF, TICK_SEL, 25, 1)
    FIELD(LEDC_TIMER_CONF, PARA_UP, 26, 1)
REG32(LEDC_TIMER_VALUE, 0x144)
REG32(LEDC_INT_RAW, 0x180)
REG32(LEDC_INT_ST, 0x184)
REG32(LEDC_INT_ENA, 0x188)
REG32(LEDC_INT_CLR, 0x18c)
REG32(LEDC_CONF, 0x190)
    FIELD(LEDC_CONF, APB_CLK_SEL, 0, 1)
REG32(LEDC_DATE, 0x1fc)

#define LEDC_TIMER_STRIDE       8
#define LEDC_REGS_SIZE          0x200
#define LEDC_HS_CONF0_WRITABLE  0xf
#define LEDC_LS_CONF0_WRITABLE  0x1f
#define LEDC_HPOINT_MASK        0xfffff
#define LEDC_DUTY_MASK          0x1ffffff
#define LEDC_DUTY_INT_MAX       0x1fffff
#define LEDC_HS_TIMER_WRITABLE  0x3ffffff
#define LEDC_LS_TIMER_WRITABLE  0x7ffffff
#define LEDC_INT_MASK           0xffffff
#define LEDC_CONF1_RESET        R_LEDC_CH_CONF1_DUTY_INC_MASK
#define LEDC_DATE_RESET         0x16031700
#define LEDC_MAX_RES            20
/* LEDC_CLK_DIV below 1 (A = 0) stops the counter */
#define LEDC_MIN_DIV            256
#define LEDC_TIMER_HS_CNT       4
/* INT_RAW: the timers' overflows, then the channels' fade ends */
#define LEDC_INT_DUTY_CHNG_END(n) (1u << (ESP32_LEDC_TIMER_CNT + (n)))

/* Output changes closer together than this are coalesced */
#define LEDC_EDGE_MIN_NS        10000
/*
 * An output this far behind the present skips to the level shortly before
 * it rather than replaying every coalesced change.
 */
#define LEDC_CATCHUP_NS         (100 * LEDC_EDGE_MIN_NS)
/*
 * Cycles looked ahead for a channel's next edge: two show a steady output's
 * pattern, seventeen a dithered one's (DUTY[3:0] repeats every sixteen),
 * and a fade is looked at a stretch at a time.
 */
#define LEDC_LOOK_STEADY        2
#define LEDC_LOOK_DITHER        17
#define LEDC_LOOK_FADE          64

static void ledc_update_irq(Esp32LEDCState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

/* a * b / 2^32, saturating */
static uint64_t mul_shr32(uint64_t a, uint64_t b)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, a, b);
    if (hi >> 32) {
        return UINT64_MAX;
    }
    return (lo >> 32) | (hi << 32);
}

/* Ticks done d ns after the anchor: the largest m with m q / 2^32 <= d */
static uint64_t ledc_ticks(uint64_t d, uint64_t q)
{
    uint64_t hi = (d + 1) >> 32, lo = (d + 1) << 32;

    if (lo == 0) {
        hi--;
    }
    lo--;
    divu128(&lo, &hi, q);
    return hi ? UINT64_MAX : lo;
}

static bool timer_is_ls(unsigned i)
{
    return i >= LEDC_TIMER_HS_CNT;
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * A high-speed timer counts APB_CLK or REF_TICK, a low-speed one REF_TICK
 * or SLOW_CLK, which LEDC_APB_CLK_SEL makes APB_CLK or RC_FAST_CLK.
 */
static Clock *timer_clock(Esp32LEDCState *s, unsigned i)
{
    if (!FIELD_EX32(s->timer[i].conf, LEDC_TIMER_CONF, TICK_SEL)) {
        return s->ref_tick_clk;
    }
    if (!timer_is_ls(i) || FIELD_EX32(s->conf, LEDC_CONF, APB_CLK_SEL)) {
        return s->apb_clk;
    }
    return s->rc_fast_clk;
}

/* The counter's tick period in 2^-32 ns; 0 while it stands still. */
static uint64_t timer_q(Esp32LEDCState *s, unsigned i)
{
    Esp32LedcTimer *t = &s->timer[i];
    Clock *clk = timer_clock(s, i);

    if (FIELD_EX32(t->conf, LEDC_TIMER_CONF, RST) ||
        FIELD_EX32(t->conf, LEDC_TIMER_CONF, PAUSE) ||
        !clock_is_enabled(clk) || t->div < LEDC_MIN_DIV) {
        return 0;
    }
    return mul_shr32(clock_get(clk), (uint64_t)t->div << 24);
}

static uint64_t timer_period(Esp32LedcTimer *t)
{
    return 1ULL << t->res;
}

/* Where the counter stands at x: the count and the cycle. */
static void timer_pos(Esp32LedcTimer *t, int64_t x, uint64_t *k, uint64_t *v)
{
    uint64_t total;

    if (!t->q || x <= t->anchor_ns) {
        *k = t->anchor_cycle;
        *v = t->anchor_cnt;
        return;
    }
    total = t->anchor_cnt + ledc_ticks(x - t->anchor_ns, t->q);
    *k = t->anchor_cycle + (total >> t->res);
    *v = total & (timer_period(t) - 1);
}

/*
 * When the counter reaches v in cycle k: INT64_MIN if that is before the
 * anchor, INT64_MAX if the counter stands still.
 */
static int64_t timer_time(Esp32LedcTimer *t, uint64_t k, uint64_t v)
{
    uint64_t m, ns;

    if (k < t->anchor_cycle || (k == t->anchor_cycle && v < t->anchor_cnt)) {
        return INT64_MIN;
    }
    if (!t->q) {
        return INT64_MAX;
    }
    m = ((k - t->anchor_cycle) << t->res) + v - t->anchor_cnt;
    ns = mul_shr32(m, t->q);
    if (ns >= (uint64_t)(INT64_MAX - t->anchor_ns)) {
        return INT64_MAX;
    }
    return t->anchor_ns + ns;
}

/* Restart the counter's arithmetic from where it stands at now. */
static void timer_reanchor(Esp32LedcTimer *t, int64_t now)
{
    uint64_t k, v;

    timer_pos(t, now, &k, &v);
    t->anchor_ns = now;
    t->anchor_cycle = k;
    t->anchor_cnt = v;
}

static unsigned chan_timer_idx(Esp32LEDCState *s, unsigned n)
{
    return (n >= ESP32_LEDC_HS_CNT ? LEDC_TIMER_HS_CNT : 0) +
           FIELD_EX32(s->ch[n].conf0, LEDC_CH_CONF0, TIMER_SEL);
}

static Esp32LedcTimer *chan_timer(Esp32LEDCState *s, unsigned n)
{
    return &s->timer[chan_timer_idx(s, n)];
}

/* The duty, 21.4 fixed point, in cycle k: a fade's steps saturate. */
static uint32_t chan_duty_at(Esp32LedcChannel *ch, uint64_t k)
{
    uint32_t d = ch->act_duty;

    if (ch->fade && k > ch->k_fade) {
        uint64_t steps = MIN((k - ch->k_fade) / ch->fade_cycle, ch->fade_num);
        uint64_t delta = steps * ch->fade_scale;
        uint64_t ip = d >> 4;

        if (ch->fade_inc) {
            ip = MIN(ip + delta, LEDC_DUTY_INT_MAX);
        } else {
            ip = ip > delta ? ip - delta : 0;
        }
        d = (ip << 4) | (d & 0xf);
    }
    return d;
}

/*
 * The low point of cycle k: HPOINT + DUTY[24:4], one later in DUTY[3:0] of
 * every sixteen cycles, spread evenly.
 */
static uint64_t chan_lpoint(Esp32LedcChannel *ch, uint64_t k)
{
    uint32_t d = chan_duty_at(ch, k);
    uint32_t frac = d & 0xf;
    uint64_t j = (k - ch->k_base) & 0xf;
    unsigned longer = (((j + 1) * frac) >> 4) - ((j * frac) >> 4);

    return (uint64_t)ch->act_hpoint + (d >> 4) + longer;
}

/*
 * The level at the start of cycle k: a cycle that reached its low point
 * ended low; one that went high at HPOINT and never reached its low point
 * ended high. With HPOINT beyond the count the level never changes.
 */
static bool chan_start_level(Esp32LedcChannel *ch, Esp32LedcTimer *t,
                             uint64_t k)
{
    if (k <= ch->k_base || ch->act_hpoint >= timer_period(t)) {
        return ch->base_level;
    }
    return chan_lpoint(ch, k - 1) >= timer_period(t);
}

/* The generator's level with its timer at count v of cycle k. */
static bool chan_level_in(Esp32LedcChannel *ch, Esp32LedcTimer *t,
                          uint64_t k, uint64_t v)
{
    if (!ch->active || ch->act_hpoint >= timer_period(t)) {
        return ch->base_level;
    }
    if (v < ch->act_hpoint) {
        return chan_start_level(ch, t, k);
    }
    return v < chan_lpoint(ch, k);
}

static bool chan_level_at(Esp32LEDCState *s, unsigned n, int64_t x)
{
    Esp32LedcTimer *t = chan_timer(s, n);
    uint64_t k, v;

    timer_pos(t, x, &k, &v);
    return chan_level_in(&s->ch[n], t, k, v);
}

/*
 * The time after x at which the generator's level next changes, or at
 * which to look again during a long fade; INT64_MAX if neither. Cycles
 * from which the generator's settings change (a load, the timer's own
 * update) are not looked into.
 */
static int64_t chan_next_edge(Esp32LEDCState *s, unsigned n, int64_t x)
{
    Esp32LedcChannel *ch = &s->ch[n];
    Esp32LedcTimer *t = chan_timer(s, n);
    uint64_t period = timer_period(t), hp = ch->act_hpoint;
    uint64_t k, v, stop = UINT64_MAX, look;
    bool fading;

    if (!ch->active || !t->q || hp >= period) {
        return INT64_MAX;
    }
    timer_pos(t, x, &k, &v);
    if (ch->load_pending) {
        stop = ch->load_cycle;
    }
    if (t->pending) {
        stop = MIN(stop, t->pend_cycle);
    }
    fading = ch->fade && (k < ch->k_fade ||
                          (k - ch->k_fade) / ch->fade_cycle < ch->fade_num);
    if (fading) {
        look = LEDC_LOOK_FADE;
    } else if (chan_duty_at(ch, k) & 0xf) {
        look = LEDC_LOOK_DITHER;
    } else {
        look = LEDC_LOOK_STEADY;
    }
    for (uint64_t kk = k; kk < k + look && kk < stop; kk++) {
        uint64_t lp = chan_lpoint(ch, kk);
        bool high = hp < lp;
        int64_t tm;

        if (chan_start_level(ch, t, kk) != high) {
            tm = timer_time(t, kk, hp);
            if (tm > x) {
                return tm;
            }
        }
        if (high && lp < period) {
            tm = timer_time(t, kk, lp);
            if (tm > x) {
                return tm;
            }
        }
    }
    if (fading && k + look < stop) {
        return timer_time(t, k + look, 0);
    }
    return INT64_MAX;
}

/* An event's time, with one already due counted as due at the anchor. */
static int64_t due_time(Esp32LedcTimer *t, int64_t tm)
{
    return tm == INT64_MIN ? t->anchor_ns : tm;
}

static int64_t chan_load_time(Esp32LEDCState *s, unsigned n)
{
    Esp32LedcChannel *ch = &s->ch[n];
    Esp32LedcTimer *t = chan_timer(s, n);

    if (!ch->load_pending) {
        return INT64_MAX;
    }
    return due_time(t, timer_time(t, ch->load_cycle, 0));
}

static int64_t chan_fade_end_time(Esp32LEDCState *s, unsigned n)
{
    Esp32LedcChannel *ch = &s->ch[n];
    Esp32LedcTimer *t = chan_timer(s, n);

    if (!ch->fade_end_due) {
        return INT64_MAX;
    }
    return due_time(t, timer_time(t, ch->k_fade +
                                  (uint64_t)ch->fade_num * ch->fade_cycle, 0));
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Drive the channel's output signal: the generator's level while
 * SIG_OUT_EN is set, IDLE_LV otherwise.
 */
static void chan_drive(Esp32LEDCState *s, unsigned n, int64_t when)
{
    Esp32LedcChannel *ch = &s->ch[n];
    bool level = FIELD_EX32(ch->conf0, LEDC_CH_CONF0, SIG_OUT_EN) ?
                 ch->gen_level : FIELD_EX32(ch->conf0, LEDC_CH_CONF0, IDLE_LV);

    if (level != ch->out_level) {
        ch->out_level = level;
        ch->emit_ns = when;
        esp32_line_set_at(s->out[n], level, when);
    }
}

static void chan_fade_ended(Esp32LEDCState *s, unsigned n)
{
    s->ch[n].fade_end_due = false;
    s->int_raw |= LEDC_INT_DUTY_CHNG_END(n);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Take HPOINT and DUTY in at the start of cycle k, at time when, and with
 * load_fade the fade settings too; DUTY_START and PARA_UP clear. A fade
 * of no steps has ended at once.
 */
static void chan_load(Esp32LEDCState *s, unsigned n, int64_t when, uint64_t k)
{
    Esp32LedcChannel *ch = &s->ch[n];
    uint32_t c1 = ch->conf1;

    ch->act_hpoint = ch->hpoint & LEDC_HPOINT_MASK;
    ch->act_duty = ch->duty & LEDC_DUTY_MASK;
    ch->k_base = k;
    ch->k_fade = k;
    ch->base_level = ch->gen_level;
    ch->active = true;
    ch->fade = false;
    ch->fade_end_due = false;
    if (ch->load_fade) {
        ch->fade = true;
        ch->fade_inc = FIELD_EX32(c1, LEDC_CH_CONF1, DUTY_INC);
        ch->fade_scale = FIELD_EX32(c1, LEDC_CH_CONF1, DUTY_SCALE);
        ch->fade_cycle = MAX(FIELD_EX32(c1, LEDC_CH_CONF1, DUTY_CYCLE), 1);
        ch->fade_num = FIELD_EX32(c1, LEDC_CH_CONF1, DUTY_NUM);
        if (ch->fade_num == 0) {
            chan_fade_ended(s, n);
        } else {
            ch->fade_end_due = true;
        }
    }
    ch->load_pending = false;
    ch->conf1 = FIELD_DP32(ch->conf1, LEDC_CH_CONF1, DUTY_START, 0);
    ch->conf0 = FIELD_DP32(ch->conf0, LEDC_CH_CONF0, PARA_UP, 0);
    /* Edges at the cycle's very start belong to the new settings */
    ch->eval_ns = when - 1;
}

/*
 * Carry the generator over a break in its timer's counting, at cycle
 * k_old of the timer it was on and from cycle k_new of the timer it is on
 * now: the fade's progress is kept, and the level at the break starts the
 * new stretch.
 */
static void chan_rebase(Esp32LEDCState *s, unsigned n, uint64_t k_old,
                        uint64_t k_new, int64_t when)
{
    Esp32LedcChannel *ch = &s->ch[n];

    if (ch->active && ch->fade) {
        uint64_t steps = 0, phase = 0;

        if (k_old > ch->k_fade) {
            steps = MIN((k_old - ch->k_fade) / ch->fade_cycle, ch->fade_num);
            phase = k_old - ch->k_fade - steps * ch->fade_cycle;
        }
        ch->act_duty = chan_duty_at(ch, k_old);
        ch->fade_num -= steps;
        ch->k_fade = k_new - MIN(phase, k_new);
        if (ch->fade_num == 0) {
            ch->fade = false;
            if (ch->fade_end_due) {
                chan_fade_ended(s, n);
            }
        }
    }
    ch->k_base = k_new;
    ch->base_level = ch->gen_level;
    ch->eval_ns = when - 1;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Run a channel up to limit (and including it with inclusive): its loads,
 * its fade's end and its output changes, coalesced as the header comment
 * describes.
 */
static void chan_eval(Esp32LEDCState *s, unsigned n, int64_t limit,
                      bool inclusive)
{
    Esp32LedcChannel *ch = &s->ch[n];

    for (;;) {
        int64_t e_load = chan_load_time(s, n);
        int64_t e_fend = chan_fade_end_time(s, n);
        int64_t e_struct = MIN(e_load, e_fend);
        int64_t e = chan_next_edge(s, n, ch->eval_ns);
        int64_t next;

        if (e != INT64_MAX && e < e_struct && e < limit - LEDC_CATCHUP_NS) {
            int64_t x = MIN(limit - LEDC_CATCHUP_NS, e_struct - 1);

            ch->eval_ns = x;
            ch->gen_level = chan_level_at(s, n, x);
            chan_drive(s, n, x);
            continue;
        }
        if (e != INT64_MAX) {
            e = MAX(e, ch->emit_ns + LEDC_EDGE_MIN_NS);
        }
        next = MIN(e, e_struct);
        if (next == INT64_MAX || next > limit ||
            (next == limit && !inclusive)) {
            break;
        }
        if (next == e_fend) {
            chan_fade_ended(s, n);
        } else if (next == e_load) {
            uint64_t k = s->ch[n].load_cycle;

            chan_load(s, n, e_load, k);
        } else {
            ch->eval_ns = e;
            ch->gen_level = chan_level_at(s, n, e);
            chan_drive(s, n, e);
        }
    }
}

/* Bring a channel's output to its exact level at now. */
static void chan_sample(Esp32LEDCState *s, unsigned n, int64_t now)
{
    Esp32LedcChannel *ch = &s->ch[n];

    if (now <= ch->eval_ns) {
        return;
    }
    ch->eval_ns = now;
    ch->gen_level = chan_level_at(s, n, now);
    chan_drive(s, n, now);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A timer's overflow interrupt, raised as its count reaches
 * 2^DUTY_RES - 1, for every overflow up to x.
 */
static void timer_ovf_check(Esp32LEDCState *s, unsigned i, int64_t x)
{
    Esp32LedcTimer *t = &s->timer[i];
    uint64_t k, v;
    bool top;

    timer_pos(t, x, &k, &v);
    top = v == timer_period(t) - 1;
    if (k > t->ovf_next || (k == t->ovf_next && top)) {
        s->int_raw |= 1u << i;
        t->ovf_next = top ? k + 1 : k;
    }
}

/* The divider and resolution conf asks for, the resolution capped at 20. */
static void timer_take_conf(Esp32LedcTimer *t)
{
    t->div = FIELD_EX32(t->conf, LEDC_TIMER_CONF, DIV_NUM);
    t->res = MIN(FIELD_EX32(t->conf, LEDC_TIMER_CONF, DUTY_RES),
                 LEDC_MAX_RES);
    t->pending = false;
    t->conf = FIELD_DP32(t->conf, LEDC_TIMER_CONF, PARA_UP, 0);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Bring the whole controller up to now: the timers' deferred updates in
 * time order, each with its channels run up to it and carried over it,
 * then every channel, the overflow interrupts, and with sample the
 * outputs' exact levels.
 */
static void ledc_advance(Esp32LEDCState *s, int64_t now, bool sample)
{
    for (;;) {
        int best = -1;
        int64_t tb = INT64_MAX;
        Esp32LedcTimer *t;

        for (unsigned i = 0; i < ESP32_LEDC_TIMER_CNT; i++) {
            t = &s->timer[i];
            if (t->pending && t->q) {
                int64_t tm = due_time(t, timer_time(t, t->pend_cycle, 0));

                if (tm < tb) {
                    tb = tm;
                    best = i;
                }
            }
        }
        if (best < 0 || tb > now) {
            break;
        }
        t = &s->timer[best];
        for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
            if (chan_timer_idx(s, n) == best) {
                chan_eval(s, n, tb, false);
            }
        }
        timer_ovf_check(s, best, tb);
        t->anchor_ns = tb;
        t->anchor_cnt = 0;
        t->anchor_cycle = t->pend_cycle;
        t->ovf_next = MAX(t->ovf_next, t->pend_cycle);
        timer_take_conf(t);
        t->q = timer_q(s, best);
        for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
            if (chan_timer_idx(s, n) == best) {
                chan_rebase(s, n, t->anchor_cycle, t->anchor_cycle, tb);
            }
        }
    }
    for (unsigned i = 0; i < ESP32_LEDC_TIMER_CNT; i++) {
        timer_ovf_check(s, i, now);
    }
    for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
        chan_eval(s, n, now, true);
        if (sample) {
            chan_sample(s, n, now);
        }
    }
}

/* Arm the QEMU timers for the next events and update the interrupt. */
static void ledc_schedule(Esp32LEDCState *s)
{
    for (unsigned i = 0; i < ESP32_LEDC_TIMER_CNT; i++) {
        Esp32LedcTimer *t = &s->timer[i];
        int64_t next = INT64_MAX;

        if (t->q && t->pending) {
            next = due_time(t, timer_time(t, t->pend_cycle, 0));
        }
        if (t->q && (s->int_ena & ~s->int_raw & (1u << i))) {
            next = MIN(next, due_time(t, timer_time(t, t->ovf_next,
                                                    timer_period(t) - 1)));
        }
        if (next == INT64_MAX) {
            timer_del(&t->timer);
        } else {
            timer_mod_ns(&t->timer, next);
        }
    }
    for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
        Esp32LedcChannel *ch = &s->ch[n];
        int64_t e = chan_next_edge(s, n, ch->eval_ns);
        int64_t next;

        if (e != INT64_MAX) {
            e = MAX(e, ch->emit_ns + LEDC_EDGE_MIN_NS);
        }
        next = MIN(e, MIN(chan_load_time(s, n), chan_fade_end_time(s, n)));
        if (next == INT64_MAX) {
            timer_del(&ch->timer);
        } else {
            timer_mod_ns(&ch->timer, next);
        }
    }
    ledc_update_irq(s);
}

static void ledc_timer_cb(void *opaque)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);

    ledc_advance(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), false);
    ledc_schedule(s);
}

static void ledc_line_sync(Notifier *n, void *data)
{
    Esp32LEDCState *s = container_of(n, Esp32LEDCState, line_source);

    ledc_advance(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), true);
    ledc_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * The counters keep their counts across a change of clock; the time of
 * each later tick follows the new frequency.
 */
static void ledc_clk_update(void *opaque, ClockEvent event)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        ledc_advance(s, now, false);
        return;
    }
    for (unsigned i = 0; i < ESP32_LEDC_TIMER_CNT; i++) {
        timer_reanchor(&s->timer[i], now);
        s->timer[i].q = timer_q(s, i);
    }
    ledc_schedule(s);
}

/*
 * Ask for HPOINT and DUTY (with fade, the fade too) to be taken in at the
 * start of the timer's next cycle; at once if the timer is not counting.
 */
static void chan_request_load(Esp32LEDCState *s, unsigned n, bool fade,
                              int64_t now)
{
    Esp32LedcChannel *ch = &s->ch[n];
    Esp32LedcTimer *t = chan_timer(s, n);
    uint64_t k, v;

    timer_pos(t, now, &k, &v);
    ch->load_fade = fade;
    if (t->q) {
        ch->load_pending = true;
        ch->load_cycle = k + 1;
    } else {
        chan_load(s, n, now, k);
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A timer's configuration. RST holds the counter at 0, and the cycle after
 * it starts afresh with the divider and resolution written; PAUSE stops
 * it where it is. Otherwise a high-speed timer takes a new divider or
 * resolution at its next overflow, and a low-speed one only once
 * PARA_UP asks it to, also at the next overflow.
 */
static void ledc_timer_write(Esp32LEDCState *s, unsigned i, uint32_t v,
                             int64_t now)
{
    Esp32LedcTimer *t = &s->timer[i];
    bool was_rst = FIELD_EX32(t->conf, LEDC_TIMER_CONF, RST);
    uint64_t k, cnt;

    timer_reanchor(t, now);
    timer_pos(t, now, &k, &cnt);
    t->conf = v & (timer_is_ls(i) ? LEDC_LS_TIMER_WRITABLE :
                                    LEDC_HS_TIMER_WRITABLE);
    if (FIELD_EX32(v, LEDC_TIMER_CONF, DUTY_RES) > LEDC_MAX_RES) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ledc: timer %u duty resolution %u is over 20 "
                      "bits\n", i,
                      (unsigned)FIELD_EX32(v, LEDC_TIMER_CONF, DUTY_RES));
    }
    if (FIELD_EX32(v, LEDC_TIMER_CONF, DIV_NUM) < LEDC_MIN_DIV) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ledc: timer %u clock divider 0x%x is below 1; "
                      "the timer stops\n", i,
                      (unsigned)FIELD_EX32(v, LEDC_TIMER_CONF, DIV_NUM));
    }
    if (FIELD_EX32(t->conf, LEDC_TIMER_CONF, RST)) {
        if (!was_rst) {
            t->anchor_cycle = k + 1;
            t->ovf_next = MAX(t->ovf_next, t->anchor_cycle);
        }
        t->anchor_cnt = 0;
        timer_take_conf(t);
        for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
            if (chan_timer_idx(s, n) == i) {
                chan_rebase(s, n, k, t->anchor_cycle, now);
            }
        }
    } else if (timer_is_ls(i)) {
        if (FIELD_EX32(t->conf, LEDC_TIMER_CONF, PARA_UP) && !t->pending) {
            t->pending = true;
            t->pend_cycle = k + 1;
        }
    } else if (FIELD_EX32(t->conf, LEDC_TIMER_CONF, DIV_NUM) != t->div ||
               FIELD_EX32(t->conf, LEDC_TIMER_CONF, DUTY_RES) != t->res) {
        if (!t->pending) {
            t->pending = true;
            t->pend_cycle = k + 1;
        }
    }
    t->q = timer_q(s, i);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A channel's CONF0. SIG_OUT_EN and IDLE_LV act at once; a new TIMER_SEL
 * moves the generator to that timer's count; PARA_UP has a low-speed
 * channel take in its HPOINT and DUTY, and with DUTY_START its fade, at
 * the timer's next cycle.
 */
static void ledc_conf0_write(Esp32LEDCState *s, unsigned n, uint32_t v,
                             int64_t now)
{
    Esp32LedcChannel *ch = &s->ch[n];
    bool ls = n >= ESP32_LEDC_HS_CNT;
    uint32_t mask = ls ? LEDC_LS_CONF0_WRITABLE : LEDC_HS_CONF0_WRITABLE;
    unsigned old_timer = chan_timer_idx(s, n);
    uint64_t k_old, k_new, cnt;

    if (n == 0) {
        mask |= R_LEDC_CH_CONF0_CLK_EN_MASK;
    }
    timer_pos(&s->timer[old_timer], now, &k_old, &cnt);
    ch->conf0 = (ch->conf0 & R_LEDC_CH_CONF0_PARA_UP_MASK) | (v & mask);
    if (chan_timer_idx(s, n) != old_timer) {
        timer_pos(chan_timer(s, n), now, &k_new, &cnt);
        chan_rebase(s, n, k_old, k_new, now);
        if (ch->load_pending) {
            ch->load_cycle = k_new + 1;
        }
    }
    if (ls && FIELD_EX32(v, LEDC_CH_CONF0, PARA_UP)) {
        chan_request_load(s, n, FIELD_EX32(ch->conf1, LEDC_CH_CONF1,
                                           DUTY_START), now);
        if (ch->load_pending) {
            ch->conf0 |= R_LEDC_CH_CONF0_PARA_UP_MASK;
        }
    }
    chan_drive(s, n, now);
}

static uint64_t esp32_ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t r = 0;

    ledc_advance(s, now, false);
    if (addr < A_LEDC_TIMER_CONF) {
        unsigned n = addr / LEDC_CH_STRIDE;
        Esp32LedcChannel *ch = &s->ch[n];
        uint64_t k, v;

        switch (addr % LEDC_CH_STRIDE) {
        case A_LEDC_CH_CONF0:
            r = ch->conf0;
            break;
        case A_LEDC_CH_HPOINT:
            r = ch->hpoint;
            break;
        case A_LEDC_CH_DUTY:
            r = ch->duty;
            break;
        case A_LEDC_CH_CONF1:
            r = ch->conf1;
            break;
        case A_LEDC_CH_DUTY_R:
            timer_pos(chan_timer(s, n), now, &k, &v);
            r = ch->active ? chan_duty_at(ch, k) : 0;
            break;
        }
    } else if (addr < A_LEDC_INT_RAW) {
        unsigned i = (addr - A_LEDC_TIMER_CONF) / LEDC_TIMER_STRIDE;
        uint64_t k, v;

        if ((addr - A_LEDC_TIMER_CONF) % LEDC_TIMER_STRIDE == 0) {
            r = s->timer[i].conf;
        } else {
            timer_pos(&s->timer[i], now, &k, &v);
            r = v;
        }
    } else {
        switch (addr) {
        case A_LEDC_INT_RAW:
            r = s->int_raw;
            break;
        case A_LEDC_INT_ST:
            r = s->int_raw & s->int_ena;
            break;
        case A_LEDC_INT_ENA:
            r = s->int_ena;
            break;
        case A_LEDC_INT_CLR:
            r = 0;
            break;
        case A_LEDC_CONF:
            r = s->conf;
            break;
        case A_LEDC_DATE:
            r = s->date;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ledc: read from reserved offset 0x%"
                          HWADDR_PRIx "\n", addr);
            break;
        }
    }
    ledc_schedule(s);
    return r;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static void esp32_ledc_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = value;

    ledc_advance(s, now, false);
    if (addr < A_LEDC_TIMER_CONF) {
        unsigned n = addr / LEDC_CH_STRIDE;
        Esp32LedcChannel *ch = &s->ch[n];

        switch (addr % LEDC_CH_STRIDE) {
        case A_LEDC_CH_CONF0:
            ledc_conf0_write(s, n, v, now);
            break;
        case A_LEDC_CH_HPOINT:
            ch->hpoint = v & LEDC_HPOINT_MASK;
            break;
        case A_LEDC_CH_DUTY:
            ch->duty = v & LEDC_DUTY_MASK;
            break;
        case A_LEDC_CH_CONF1:
            ch->conf1 = v;
            /* A low-speed channel waits for PARA_UP */
            if (FIELD_EX32(v, LEDC_CH_CONF1, DUTY_START) &&
                n < ESP32_LEDC_HS_CNT) {
                chan_request_load(s, n, true, now);
            }
            break;
        case A_LEDC_CH_DUTY_R:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ledc: write to read-only DUTY_R of channel "
                          "%u\n", n);
            break;
        }
    } else if (addr < A_LEDC_INT_RAW) {
        unsigned i = (addr - A_LEDC_TIMER_CONF) / LEDC_TIMER_STRIDE;

        if ((addr - A_LEDC_TIMER_CONF) % LEDC_TIMER_STRIDE == 0) {
            ledc_timer_write(s, i, v, now);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ledc: write to read-only VALUE of timer "
                          "%u\n", i);
        }
    } else {
        switch (addr) {
        case A_LEDC_INT_ENA:
            s->int_ena = v & LEDC_INT_MASK;
            break;
        case A_LEDC_INT_CLR:
            s->int_raw &= ~v;
            break;
        case A_LEDC_CONF:
            for (unsigned i = LEDC_TIMER_HS_CNT; i < ESP32_LEDC_TIMER_CNT;
                 i++) {
                timer_reanchor(&s->timer[i], now);
            }
            s->conf = v & R_LEDC_CONF_APB_CLK_SEL_MASK;
            for (unsigned i = LEDC_TIMER_HS_CNT; i < ESP32_LEDC_TIMER_CNT;
                 i++) {
                s->timer[i].q = timer_q(s, i);
            }
            break;
        case A_LEDC_DATE:
            s->date = v;
            break;
        case A_LEDC_INT_RAW:
        case A_LEDC_INT_ST:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ledc: write to read-only register 0x%"
                          HWADDR_PRIx "\n", addr);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ledc: write to reserved offset 0x%"
                          HWADDR_PRIx "\n", addr);
            break;
        }
    }
    ledc_advance(s, now, false);
    ledc_schedule(s);
}

static const MemoryRegionOps esp32_ledc_ops = {
    .read = esp32_ledc_read,
    .write = esp32_ledc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * DPORT's reset bit returns the timers and channels to their reset state:
 * every timer held in reset, every output off at its idle level, low.
 */
static void esp32_ledc_reset_hold(Object *obj, ResetType type)
{
    Esp32LEDCState *s = ESP32_LEDC(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (unsigned i = 0; i < ESP32_LEDC_TIMER_CNT; i++) {
        Esp32LedcTimer *t = &s->timer[i];

        timer_del(&t->timer);
        t->conf = R_LEDC_TIMER_CONF_RST_MASK;
        t->div = 0;
        t->res = 0;
        t->pending = false;
        t->pend_cycle = 0;
        t->anchor_ns = now;
        t->anchor_cnt = 0;
        t->anchor_cycle = 0;
        t->q = 0;
        t->ovf_next = 0;
    }
    for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
        Esp32LedcChannel *ch = &s->ch[n];

        timer_del(&ch->timer);
        ch->conf0 = 0;
        ch->hpoint = 0;
        ch->duty = 0;
        ch->conf1 = LEDC_CONF1_RESET;
        ch->load_pending = false;
        ch->load_fade = false;
        ch->load_cycle = 0;
        ch->active = false;
        ch->act_hpoint = 0;
        ch->act_duty = 0;
        ch->fade = false;
        ch->fade_inc = false;
        ch->fade_scale = 0;
        ch->fade_cycle = 1;
        ch->fade_num = 0;
        ch->k_fade = 0;
        ch->k_base = 0;
        ch->base_level = false;
        ch->fade_end_due = false;
        ch->gen_level = false;
        ch->eval_ns = now;
        ch->emit_ns = now;
    }
    s->int_raw = 0;
    s->int_ena = 0;
    s->conf = 0;
    s->date = LEDC_DATE_RESET;
}

static void esp32_ledc_reset_exit(Object *obj, ResetType type)
{
    Esp32LEDCState *s = ESP32_LEDC(obj);

    for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
        s->ch[n].out_level = false;
        qemu_set_irq(s->out[n], 0);
    }
    ledc_update_irq(s);
}

static void esp32_ledc_init(Object *obj)
{
    Esp32LEDCState *s = ESP32_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_ledc_ops, s,
                          TYPE_ESP32_LEDC, LEDC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, s->out, ESP32_LEDC_OUT,
                             ESP32_LEDC_CHANNEL_CNT);
    s->apb_clk = qdev_init_clock_in(dev, "apb", ledc_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->ref_tick_clk = qdev_init_clock_in(dev, "ref_tick", ledc_clk_update, s,
                                         ClockPreUpdate | ClockUpdate);
    s->rc_fast_clk = qdev_init_clock_in(dev, "rc_fast", ledc_clk_update, s,
                                        ClockPreUpdate | ClockUpdate);
    for (unsigned i = 0; i < ESP32_LEDC_TIMER_CNT; i++) {
        timer_init_ns(&s->timer[i].timer, QEMU_CLOCK_VIRTUAL, ledc_timer_cb,
                      s);
    }
    for (unsigned n = 0; n < ESP32_LEDC_CHANNEL_CNT; n++) {
        timer_init_ns(&s->ch[n].timer, QEMU_CLOCK_VIRTUAL, ledc_timer_cb, s);
    }
    s->line_source.notify = ledc_line_sync;
}

static void esp32_ledc_realize(DeviceState *dev, Error **errp)
{
    esp32_line_add_source(&ESP32_LEDC(dev)->line_source);
}

static const VMStateDescription vmstate_esp32_ledc_timer = {
    .name = "esp32-ledc-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf, Esp32LedcTimer),
        VMSTATE_UINT32(div, Esp32LedcTimer),
        VMSTATE_UINT32(res, Esp32LedcTimer),
        VMSTATE_BOOL(pending, Esp32LedcTimer),
        VMSTATE_UINT64(pend_cycle, Esp32LedcTimer),
        VMSTATE_INT64(anchor_ns, Esp32LedcTimer),
        VMSTATE_UINT32(anchor_cnt, Esp32LedcTimer),
        VMSTATE_UINT64(anchor_cycle, Esp32LedcTimer),
        VMSTATE_UINT64(q, Esp32LedcTimer),
        VMSTATE_UINT64(ovf_next, Esp32LedcTimer),
        VMSTATE_TIMER(timer, Esp32LedcTimer),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_ledc_channel = {
    .name = "esp32-ledc-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, Esp32LedcChannel),
        VMSTATE_UINT32(hpoint, Esp32LedcChannel),
        VMSTATE_UINT32(duty, Esp32LedcChannel),
        VMSTATE_UINT32(conf1, Esp32LedcChannel),
        VMSTATE_BOOL(load_pending, Esp32LedcChannel),
        VMSTATE_BOOL(load_fade, Esp32LedcChannel),
        VMSTATE_UINT64(load_cycle, Esp32LedcChannel),
        VMSTATE_BOOL(active, Esp32LedcChannel),
        VMSTATE_UINT32(act_hpoint, Esp32LedcChannel),
        VMSTATE_UINT32(act_duty, Esp32LedcChannel),
        VMSTATE_BOOL(fade, Esp32LedcChannel),
        VMSTATE_BOOL(fade_inc, Esp32LedcChannel),
        VMSTATE_UINT32(fade_scale, Esp32LedcChannel),
        VMSTATE_UINT32(fade_cycle, Esp32LedcChannel),
        VMSTATE_UINT32(fade_num, Esp32LedcChannel),
        VMSTATE_UINT64(k_fade, Esp32LedcChannel),
        VMSTATE_UINT64(k_base, Esp32LedcChannel),
        VMSTATE_BOOL(base_level, Esp32LedcChannel),
        VMSTATE_BOOL(fade_end_due, Esp32LedcChannel),
        VMSTATE_BOOL(gen_level, Esp32LedcChannel),
        VMSTATE_INT64(eval_ns, Esp32LedcChannel),
        VMSTATE_BOOL(out_level, Esp32LedcChannel),
        VMSTATE_INT64(emit_ns, Esp32LedcChannel),
        VMSTATE_TIMER(timer, Esp32LedcChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_ledc = {
    .name = TYPE_ESP32_LEDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(apb_clk, Esp32LEDCState),
        VMSTATE_CLOCK(ref_tick_clk, Esp32LEDCState),
        VMSTATE_CLOCK(rc_fast_clk, Esp32LEDCState),
        VMSTATE_STRUCT_ARRAY(timer, Esp32LEDCState, ESP32_LEDC_TIMER_CNT, 1,
                             vmstate_esp32_ledc_timer, Esp32LedcTimer),
        VMSTATE_STRUCT_ARRAY(ch, Esp32LEDCState, ESP32_LEDC_CHANNEL_CNT, 1,
                             vmstate_esp32_ledc_channel, Esp32LedcChannel),
        VMSTATE_UINT32(int_raw, Esp32LEDCState),
        VMSTATE_UINT32(int_ena, Esp32LEDCState),
        VMSTATE_UINT32(conf, Esp32LEDCState),
        VMSTATE_UINT32(date, Esp32LEDCState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_ledc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_ledc_reset_hold;
    rc->phases.exit = esp32_ledc_reset_exit;
    dc->realize = esp32_ledc_realize;
    dc->vmsd = &vmstate_esp32_ledc;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_ledc_info = {
    .name = TYPE_ESP32_LEDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32LEDCState),
    .instance_init = esp32_ledc_init,
    .class_init = esp32_ledc_class_init
};

static void esp32_ledc_register_types(void)
{
    type_register_static(&esp32_ledc_info);
}

type_init(esp32_ledc_register_types)
