/*
 * ESP32-P4 LED PWM controller (LEDC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 58 (LED PWM Controller), and ESP-IDF's v3 ledc_reg.h and ledc_struct.h,
 * which give the registers' layout, access and reset values; its hal
 * ledc_ll.h, esp_driver_ledc's ledc.c and ledc_etm.c, and the types the
 * ETM tasks are documented with (hal/ledc_types.h) show how software
 * drives the block.
 *
 * Timers
 * ------
 *
 * Four timers count LEDC_CLK (HP_SYS_CLKRST's "ledc", its source chosen
 * there; LEDC_APB_CLK_SEL in LEDC_CONF, which the TRM's register
 * description leaves out and ESP-IDF never writes, is held as written and
 * selects nothing) through a fractional divider: each count takes
 * LEDC_CLK_DIV = A + B/256 cycles (B counts in 256 take A + 1, spread
 * evenly, so that every count's time is exact on average and within one
 * cycle of LEDC_CLK). A divider below 1 (A = 0) stops the timer. A timer
 * counts from 0 to 2^DUTY_RES - 1, its top, and overflows back to 0, a
 * new cycle; DUTY_RES is 20 at most. The counter stands still while
 * PAUSE is set, while the ETM has paused it (a pulse of either the PAUSE
 * or the RESUME task pauses a running timer and resumes a paused one, TRM
 * 58.4.4, software using them in pairs, ESP-IDF's ledc_types.h; PAUSE is
 * not changed), or while RST holds it at 0.
 *
 * A new divider and resolution, written with PARA_UP (or a resolution
 * with the RES_UPDATE task), take effect at the next overflow, PARA_UP
 * reading 1 until then; written while RST holds the timer, they take
 * effect at once. A timer reset (RST, or the RST task) sets the count to
 * 0 without being an overflow: what waits for the next overflow goes on
 * waiting for the count to reach its top (ESP-IDF's note on the P4's
 * LEDC_TIMER_ETM_TASK_RST), unless the reset comes as the count stands at
 * its top, an overflow that has happened.
 *
 * TIMERx_OVF_INT and the TIME_OVF event come as the count reaches its
 * top (TRM 58.5: "reached its maximum counter value"), the CMP event as it
 * reaches TIMERx_CMP; the CAP task copies the count into TIMERx_CNT_CAP.
 *
 * Channels
 * --------
 *
 * A channel's output is high from the count HPOINT and low from LPOINT =
 * HPOINT + DUTY[24:4] in each cycle of its timer; LPOINT at or past the
 * top keeps it high, HPOINT past the top never sets it (a cycle starts at
 * the level the previous one ended with), and at HPOINT = LPOINT it stays
 * low. DUTY[3:0] makes that many cycles in every sixteen, spread evenly
 * and counted from the update, one count longer. SIG_OUT_EN off, the
 * output is IDLE_LV.
 *
 * PARA_UP takes in HPOINT, DUTY, SIG_OUT_EN, TIMER_SEL and OVF_CNT_EN
 * (TRM 58.4.2, register 58.1) and, with DUTY_START, the fade, all at the
 * next overflow of the channel's timer (at once while that timer stands
 * still), PARA_UP reading 1 and DUTY_START staying set until then.
 * IDLE_LV and OVF_NUM act as written.
 *
 * Fades (TRM 58.4.3): with DUTY_START the update starts a fade through
 * the GAMMA_ENTRY_NUM ranges of the channel's gamma RAM (0x400 + 64 n,
 * sixteen words of DUTY_INC [0], DUTY_CYCLE [10:1], SCALE [20:11] and
 * DUTY_NUM [30:21]). In each range, DUTY_NUM times, after DUTY_CYCLE
 * overflows (a DUTY_CYCLE of 0 counts as 1) DUTY[24:4] steps by SCALE up
 * or down, saturating at 0 and at its top; the next range starts as the
 * last step of one is taken, and a range of no steps is passed at once.
 * When the last range ends, DUTY_CHNG_END_INT and the DUTY_CHNG_END event
 * come at that overflow, and the duty holds. A range's words are read as
 * the range begins (DUTY_SCALE_UPDATE re-reads the current range's
 * SCALE). ESP-IDF's ledc_set_duty is a fade of one step of SCALE 0.
 *
 * GAMMA_PAUSE (and the GAMMA_PAUSE task) suspends the fade at the next
 * overflow, which it does not count; GAMMA_PAUSE reads 1 until
 * GAMMA_RESUME (or the task) resumes the fade at the next overflow, which
 * counts, from where it stood (TRM 58.4.3.3). The GAMMA_RESTART task
 * starts the fade afresh from range 0 at the next overflow, from DUTY,
 * the last configured duty (ledc_types.h). Each of these acts at once
 * while the channel's timer stands still.
 *
 * Each channel counts its timer's overflows while OVF_CNT_EN is in effect:
 * as the count reaches OVF_NUM + 1, OVF_CNT_CHn_INT and the OVF_CNT_PLS
 * event come and it starts again from 0 (if OVF_NUM is lowered below the
 * count, at the next overflow). OVF_CNT_RESET and the OVF_CNT_RST task
 * set it to 0. The SIG_OUT_DIS task clears SIG_OUT_EN, in effect at once.
 *
 * Timing
 * ------
 *
 * Nothing is ticked. The timers' positions are computed from when they
 * last changed, a count every LEDC_CLK's exact period times the divider
 * (esp32p4_clk_period.h), reached at the first nanosecond its exact time
 * has come; the duty of any cycle, fade or not, in closed form from the
 * fade's last anchor; the outputs as functions of both. Each output
 * changes at every one of its edges' exact times, however close together,
 * stamped on the line (hw/gpio/esp32_line.h), so that a capture, a pulse
 * counter or an ETM event that an edge reaches sees each; the host's work
 * grows with the edges' rate while something watches them (below).
 * Whenever software samples the pads the outputs are brought to their
 * exact levels at that moment. Updates, fade
 * ends, fade pauses and the timers' own updates are handled in time order at
 * their overflows; a QEMU timer fires for them, for the edges, and for
 * the overflows, matches and counts that an enabled interrupt or ETM
 * event waits for; the rest are accounted when software looks. An ETM
 * event is pulsed stamped with its time, in time order with the edges,
 * and a task acts at the time its pulse is stamped with. On the host's
 * clock an output the host held up catches up edge by edge, each at its
 * own time; only a host slower than the edges it is to drive, losing
 * ground on the controller as its timer's catch-ups each end further
 * behind than the last, or an edge more than ESP32_LINE_MAX_LAG_NS (1 s)
 * behind the present as the output comes to it (esp32_line_pwm_skips(),
 * include/hw/gpio/esp32_line.h), makes it skip to its level at the
 * moment it is being brought to, stamped with the moment that level
 * began, the edges in between lost, so that no edge carries a time other
 * than its own; under icount none is. An output that reaches no pad
 * something watches (an input a modelled block takes from it, an
 * interrupt type, an ETM channel, a line the board listens to:
 * esp32p4_gpio_sig_out_watched()) is not timed edge by edge:
 * nothing sees its edges, and it takes its level of the moment whenever
 * the controller is brought up to date, so that it costs the host nothing
 * however fast. Nor is one only a pulse counter takes in bulk, through
 * the channel's wave (hw/gpio/esp32_line.h): outside a fade the channel
 * describes its edges in closed form, cycle by cycle of its timer, the
 * cycles' edges repeating every sixteen (DUTY[3:0]'s dither); every
 * change to the channel, its timer or LEDC_CLK is announced at its
 * moment to the counters taking it. During a fade it drives its edges.
 *
 * Not modelled: the light-sleep output (TRM 58.2), which needs the PMU;
 * LEDC_CLK_EN and GAMMA_RAM_CLK_EN, clock gating that software cannot
 * see, are held as written. DATE resets to 0x2404020, which v3's
 * ledc_struct.h gives (its ledc_reg.h and the TRM have v1's 0x2303070).
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_ledc.h"
#include "migration/vmstate.h"

#define CH_STRIDE           0x14
#define CH_CONF0            0x00
#define CH_HPOINT           0x04
#define CH_DUTY             0x08
#define CH_CONF1            0x0c
#define CH_DUTY_R           0x10
#define A_TIMER_BASE        0x0a0
#define TIMER_STRIDE        0x08
#define A_INT_RAW           0x0c0
#define A_INT_ST            0x0c4
#define A_INT_ENA           0x0c8
#define A_INT_CLR           0x0cc
#define A_GAMMA_CONF        0x100
#define A_EVT_TASK_EN       0x120
#define A_TIMER_CMP         0x140
#define A_TIMER_CNT_CAP     0x150
#define A_CONF              0x170
#define A_DATE              0x174
#define A_GAMMA_RAM         0x400
#define GAMMA_RAM_SIZE      (4 * ESP32P4_LEDC_CHANNELS * ESP32P4_LEDC_RANGES)

#define CONF0_TIMER_SEL(v)  ((v) & 3)
#define CONF0_SIG_OUT_EN    (1u << 2)
#define CONF0_IDLE_LV       (1u << 3)
#define CONF0_PARA_UP       (1u << 4)
#define CONF0_OVF_NUM(v)    (((v) >> 5) & 0x3ff)
#define CONF0_OVF_CNT_EN    (1u << 15)
#define CONF0_OVF_CNT_RESET (1u << 16)
#define CONF0_MASK          0xffefu
#define HPOINT_MASK         0xfffffu
#define DUTY_MASK           0x1ffffffu
#define DUTY_INT_MAX        0x1fffffu
#define CONF1_DUTY_START    (1u << 31)

#define TCONF_DUTY_RES(v)   ((v) & 0x1f)
#define TCONF_CLK_DIV(v)    (((v) >> 5) & 0x3ffff)
#define TCONF_PAUSE         (1u << 23)
#define TCONF_RST           (1u << 24)
#define TCONF_PARA_UP       (1u << 26)
#define TCONF_MASK          0x1ffffffu
#define TCONF_RESET         TCONF_RST
#define TIMER_CNT_MASK      0xfffffu
#define MAX_RES             20
/* A divider below 1, A = 0, stops the timer */
#define MIN_DIV             256

#define GAMMA_ENTRY_NUM(v)  ((v) & 0x1f)
#define GAMMA_PAUSE         (1u << 5)
#define GAMMA_RESUME        (1u << 6)

#define RANGE_MASK          0x7fffffffu
#define RANGE_INC(p)        ((p) & 1)
#define RANGE_CYCLE(p)      MAX(((p) >> 1) & 0x3ff, 1)
#define RANGE_SCALE(p)      (((p) >> 11) & 0x3ff)
#define RANGE_NUM(p)        (((p) >> 21) & 0x3ff)

#define INT_TIMER_OVF(x)    (1u << (x))
#define INT_DUTY_CHNG_END(n) (1u << (4 + (n)))
#define INT_OVF_CNT(n)      (1u << (12 + (n)))
#define INT_MASK            0xfffffu
#define CONF_MASK           0x800003ffu
#define DATE_MASK           0x0fffffffu
#define DATE_RESET          0x02404020u

/* The ETM events, as lines of "etm-event" and bits of EVT_TASK_EN0 */
#define EVT_DUTY_CHNG_END(n) (n)
#define EVT_OVF_CNT_PLS(n)  (8 + (n))
#define EVT_TIME_OVF(x)     (16 + (x))
#define EVT_TIMER_CMP(x)    (20 + (x))

/* The ETM tasks, as lines of "etm-task" */
enum {
    TASK_RES_UPDATE,
    TASK_DUTY_SCALE_UPDATE,
    TASK_TIMER_CAP,
    TASK_SIG_OUT_DIS,
    TASK_OVF_CNT_RST,
    TASK_TIMER_RST,
    TASK_TIMER_RESUME,
    TASK_TIMER_PAUSE,
    TASK_GAMMA_RESTART,
    TASK_GAMMA_PAUSE,
    TASK_GAMMA_RESUME,
};

/*
 * Each kind of task: its first line, how many (a timer's or a channel's),
 * and its enable, bit `bit` + index of EVT_TASK_EN`reg`.
 */
static const struct {
    uint8_t first;
    uint8_t count;
    uint8_t reg;
    uint8_t bit;
} ledc_tasks[] = {
    [TASK_RES_UPDATE]        = { 0, 4, 1, 0 },
    [TASK_DUTY_SCALE_UPDATE] = { 4, 8, 0, 24 },
    [TASK_TIMER_CAP]         = { 12, 4, 1, 4 },
    [TASK_SIG_OUT_DIS]       = { 16, 8, 1, 8 },
    [TASK_OVF_CNT_RST]       = { 24, 8, 1, 16 },
    [TASK_TIMER_RST]         = { 32, 4, 1, 24 },
    [TASK_TIMER_RESUME]      = { 36, 4, 1, 28 },
    [TASK_TIMER_PAUSE]       = { 40, 4, 1, 28 },
    [TASK_GAMMA_RESTART]     = { 44, 8, 2, 0 },
    [TASK_GAMMA_PAUSE]       = { 52, 8, 2, 8 },
    [TASK_GAMMA_RESUME]      = { 60, 8, 2, 16 },
};

/* Fade operations waiting for an overflow */
#define OP_PAUSE            (1u << 0)
#define OP_RESUME           (1u << 1)
#define OP_RESTART          (1u << 2)

/* How far back a skip first looks for the edge its level began at */
#define SKIP_WINDOW_NS      (100 * 1000)
/*
 * Cycles looked ahead for a channel's next edge: two show a steady output's
 * pattern, seventeen a dithered one's (DUTY[3:0] repeats every sixteen),
 * and a fade is looked at a stretch at a time.
 */
#define LOOK_STEADY         2
#define LOOK_DITHER         17
#define LOOK_FADE           64

/*
 * Structural events, in the order they are handled at one overflow: the
 * timer's own update, the end of a fade, the channel's update (which may
 * start a fade), then a pause, resume or restart of the fade there is.
 */
enum {
    EV_TIMER_UPDATE,
    EV_FADE_END,
    EV_LOAD,
    EV_FADE_OP,
};

/* Timer arithmetic */

static uint64_t timer_period(const ESP32P4LedcTimer *t)
{
    return 1ull << t->res;
}

static uint32_t timer_top(const ESP32P4LedcTimer *t)
{
    return timer_period(t) - 1;
}

/*
 * The count's period: LEDC_CLK's exact period times the divider, 8
 * fraction bits; stopped while the counter stands still.
 */
static ESP32P4ClkPeriod timer_q(ESP32P4LedcState *s, int x)
{
    ESP32P4LedcTimer *t = &s->t[x];

    if ((t->conf & (TCONF_RST | TCONF_PAUSE)) || t->etm_paused ||
        !clock_is_enabled(s->clk) || t->div < MIN_DIV) {
        return esp32p4_clk_period_stopped();
    }
    return esp32p4_clk_period_scale(esp32p4_clk_exact(s->clk), t->div, 256);
}

static bool timer_runs(const ESP32P4LedcTimer *t)
{
    return esp32p4_clk_edges_running(&t->q);
}

/*
 * Take the count's period after a change at the anchor: the divider
 * counts LEDC_CLK's edges after the anchor, its first count a period
 * after LEDC_CLK's last edge at or before it; as they were if neither the
 * period nor LEDC_CLK's edges have changed.
 */
static void timer_set_q(ESP32P4LedcState *s, int x)
{
    ESP32P4LedcTimer *t = &s->t[x];
    ESP32P4ClkPeriod q = timer_q(s, x);
    ESP32P4ClkEdges clk = esp32p4_clk_edges_or_at(s->clk, t->anchor_ns);
    bool at;

    if (esp32p4_clk_period_eq(q, t->q.period) &&
        esp32p4_clk_edges_eq(&clk, &t->q_clk)) {
        return;
    }
    t->q_clk = clk;
    t->q = esp32p4_clk_edges_restart(q, &clk, t->anchor_ns);
    t->q_start = 0;
    if (timer_runs(t)) {
        t->q_start = esp32p4_clk_edge_index(&t->q, t->anchor_ns, &at);
    }
}

/* The edge of the divided clock the counter last counted by time @x. */
static int64_t timer_edge(const ESP32P4LedcTimer *t, int64_t x)
{
    bool at;

    return MAX(esp32p4_clk_edge_index(&t->q, x, &at), t->q_start);
}

/*
 * Where the counter stands at time x: the cycle and the count, a count
 * reached at the first nanosecond its exact time has come.
 */
static void timer_pos(const ESP32P4LedcTimer *t, int64_t x, uint64_t *k,
                      uint64_t *v)
{
    uint64_t total;

    if (!timer_runs(t) || x <= t->anchor_ns) {
        *k = t->anchor_cycle;
        *v = t->anchor_cnt;
        return;
    }
    total = t->anchor_cnt + (timer_edge(t, x) - timer_edge(t, t->anchor_ns));
    *k = t->anchor_cycle + (total >> t->res);
    *v = total & (timer_period(t) - 1);
}

/*
 * When the counter reaches count v of cycle k: INT64_MIN if that is
 * before the anchor, INT64_MAX if the counter stands still short of it.
 */
static int64_t timer_time(const ESP32P4LedcTimer *t, uint64_t k, uint64_t v)
{
    uint64_t m;
    int64_t ns;

    if (k < t->anchor_cycle || (k == t->anchor_cycle && v < t->anchor_cnt)) {
        return INT64_MIN;
    }
    m = ((k - t->anchor_cycle) << t->res) + v - t->anchor_cnt;
    if (m == 0) {
        return t->anchor_ns;
    }
    if (!timer_runs(t)) {
        return INT64_MAX;
    }
    if (m > INT64_MAX - timer_edge(t, t->anchor_ns)) {
        return INT64_MAX;
    }
    ns = esp32p4_clk_edge_ns(&t->q, timer_edge(t, t->anchor_ns) + m);
    return MAX(ns, t->anchor_ns);
}

/* An event's time, one already due counting as due at the anchor. */
static int64_t due_time(const ESP32P4LedcTimer *t, int64_t tm)
{
    return tm == INT64_MIN ? t->anchor_ns : tm;
}

/* Restart the counter's arithmetic from where it stands at x. */
static void timer_reanchor(ESP32P4LedcTimer *t, int64_t x)
{
    uint64_t k, v;

    timer_pos(t, x, &k, &v);
    t->anchor_ns = x;
    t->anchor_cycle = k;
    t->anchor_cnt = v;
}

/* The last cycle whose count reached v by time x, or -1. */
static int64_t timer_last_reached(const ESP32P4LedcTimer *t, int64_t x,
                                  uint64_t v)
{
    uint64_t k, cnt;

    timer_pos(t, x, &k, &cnt);
    return cnt >= v ? (int64_t)k : (int64_t)k - 1;
}

static void ledc_update_irq(ESP32P4LedcState *s)
{
    bool level = (s->int_raw & s->int_ena) != 0;

    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->irq, level);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * [spec:nuos:req:emu.esp32p4.etm]
 * Pulse ETM event n, stamped with time `when`, if it is enabled. A task
 * this answers in this block acts before this returns.
 */
static void ledc_etm_event(ESP32P4LedcState *s, int n, int64_t when)
{
    if (s->evt_task_en[0] & (1u << n)) {
        s->seen_ns = MAX(s->seen_ns, when);
        esp32_line_set_at(s->etm_event[n], 1, when);
        esp32_line_set_at(s->etm_event[n], 0, when);
    }
}

/* Whether an enabled interrupt or ETM event waits for timer x's overflows */
static bool timer_ovf_watched(ESP32P4LedcState *s, int x)
{
    return (s->evt_task_en[0] & (1u << EVT_TIME_OVF(x))) ||
           (s->int_ena & ~s->int_raw & INT_TIMER_OVF(x));
}

/* Channels */

static ESP32P4LedcTimer *chan_timer(ESP32P4LedcState *s, int n)
{
    return &s->t[s->ch[n].timer];
}

/* Whether channel n counts overflows, and its count's limit. */
static bool chan_counting(ESP32P4LedcState *s, int n)
{
    return s->ch[n].active && s->ch[n].ovf_cnt_en;
}

static uint32_t chan_ovf_limit(ESP32P4LedcState *s, int n)
{
    return CONF0_OVF_NUM(s->ch[n].conf0) + 1;
}

/* Overflows from the timer's next until channel n's next count pulse. */
static uint32_t chan_ovf_left(ESP32P4LedcState *s, int n)
{
    uint32_t lim = chan_ovf_limit(s, n);

    return lim - MIN(s->ch[n].ovf_cnt, lim - 1);
}

static bool chan_ovf_watched(ESP32P4LedcState *s, int n)
{
    return chan_counting(s, n) &&
           ((s->evt_task_en[0] & (1u << EVT_OVF_CNT_PLS(n))) ||
            (s->int_ena & ~s->int_raw & INT_OVF_CNT(n)));
}

/* A duty stepped by `steps` fades of range parameters p */
static uint32_t duty_step(uint32_t d, uint64_t steps, uint32_t p)
{
    uint64_t delta = steps * RANGE_SCALE(p);
    uint64_t ip = d >> 4;

    if (RANGE_INC(p)) {
        ip = MIN(ip + delta, DUTY_INT_MAX);
    } else {
        ip = ip > delta ? ip - delta : 0;
    }
    return (ip << 4) | (d & 0xf);
}

/* A fade's state during a cycle, and when it ended, if it has. */
typedef struct LedcFade {
    uint32_t range;
    uint32_t steps;
    uint32_t count;
    uint32_t param;
    uint32_t duty;
    bool done;
    uint64_t k_done;
} LedcFade;

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Channel n's fade after m more overflows from its anchor (m UINT64_MAX:
 * to its end). A range's parameters are read from the gamma RAM as it
 * begins; writes to the RAM first bring the anchor up to date, so that
 * the ranges a fade has begun are those it read.
 */
static LedcFade fade_run(ESP32P4LedcState *s, int n, uint64_t m)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    LedcFade f = {
        .range = ch->range, .steps = ch->steps, .count = ch->count,
        .param = ch->param, .duty = ch->d_anchor,
    };
    uint64_t k = ch->k_anchor;

    while (f.range < ch->entry_num) {
        uint32_t num = RANGE_NUM(f.param);
        uint64_t cyc = RANGE_CYCLE(f.param);
        uint64_t need, adv;

        if (f.steps >= num) {
            f.range++;
            f.steps = 0;
            f.count = 0;
            if (f.range < ch->entry_num) {
                f.param = s->gamma[n][f.range];
            }
            continue;
        }
        need = (num - f.steps) * cyc - f.count;
        if (m >= need) {
            f.duty = duty_step(f.duty, num - f.steps, f.param);
            f.steps = num;
            if (m != UINT64_MAX) {
                m -= need;
            }
            k += need;
            continue;
        }
        adv = f.count + m;
        f.duty = duty_step(f.duty, adv / cyc, f.param);
        f.steps += adv / cyc;
        f.count = adv % cyc;
        return f;
    }
    f.done = true;
    f.k_done = k;
    return f;
}

/* The duty, 21.4 fixed point, during cycle k of the channel's timer. */
static uint32_t chan_duty_at(ESP32P4LedcState *s, int n, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    if (!ch->fading || ch->paused || k <= ch->k_anchor) {
        return ch->d_anchor;
    }
    return fade_run(s, n, k - ch->k_anchor).duty;
}

/* Bring the fade's anchor to cycle k, k_anchor <= k. */
static void fade_rebase(ESP32P4LedcState *s, int n, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    LedcFade f;

    if (!ch->fading || ch->paused || k <= ch->k_anchor) {
        return;
    }
    f = fade_run(s, n, k - ch->k_anchor);
    ch->range = f.range;
    ch->steps = f.steps;
    ch->count = f.count;
    ch->param = f.param;
    ch->d_anchor = f.duty;
    ch->k_anchor = k;
}

/* Start a fade from duty d, its ranges counted from cycle k. */
static void fade_start(ESP32P4LedcState *s, int n, uint64_t k, uint32_t d)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    uint32_t num = GAMMA_ENTRY_NUM(ch->gamma_conf);

    if (num > ESP32P4_LEDC_RANGES) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: channel %d has %u "
                      "fade ranges, the gamma RAM %d\n", n, num,
                      ESP32P4_LEDC_RANGES);
        num = ESP32P4_LEDC_RANGES;
    }
    ch->entry_num = num;
    ch->k_anchor = k;
    ch->d_anchor = d;
    ch->range = 0;
    ch->steps = 0;
    ch->count = 0;
    ch->param = s->gamma[n][0];
    ch->fading = true;
    ch->paused = false;
    ch->end_due = true;
    ch->gamma_conf &= ~GAMMA_PAUSE;
}

/*
 * The low point of cycle k: HPOINT + DUTY[24:4], one count later in
 * DUTY[3:0] of every sixteen cycles, spread evenly.
 */
static uint64_t chan_lpoint(ESP32P4LedcState *s, int n, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    uint32_t d = chan_duty_at(s, n, k);
    uint32_t frac = d & 0xf;
    uint64_t j = (k - ch->k_dither) & 0xf;
    unsigned longer = (((j + 1) * frac) >> 4) - ((j * frac) >> 4);

    return (uint64_t)ch->act_hpoint + (d >> 4) + longer;
}

/*
 * The level at the start of cycle k: a cycle that reached its low point
 * ended low; one that went high at HPOINT and never reached its low point
 * ended high. With HPOINT beyond the count the level never changes.
 */
static bool chan_start_level(ESP32P4LedcState *s, int n, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    ESP32P4LedcTimer *t = chan_timer(s, n);

    if (k <= ch->k_base || ch->act_hpoint >= timer_period(t)) {
        return ch->base_level;
    }
    return chan_lpoint(s, n, k - 1) >= timer_period(t);
}

/* The generator's level with its timer at count v of cycle k. */
static bool chan_level_in(ESP32P4LedcState *s, int n, uint64_t k, uint64_t v)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    if (!ch->active || ch->act_hpoint >= timer_period(chan_timer(s, n))) {
        return ch->base_level;
    }
    if (v < ch->act_hpoint) {
        return chan_start_level(s, n, k);
    }
    return v < chan_lpoint(s, n, k);
}

static bool chan_level_at(ESP32P4LedcState *s, int n, int64_t x)
{
    uint64_t k, v;

    timer_pos(chan_timer(s, n), x, &k, &v);
    return chan_level_in(s, n, k, v);
}

/*
 * The time after x at which the generator's level next changes, or at
 * which to look again during a long fade; INT64_MAX if neither. Cycles
 * from which the generator's settings change (an update, a fade
 * operation, the timer's own update) are not looked into.
 */
static int64_t chan_next_edge(ESP32P4LedcState *s, int n, int64_t x)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    ESP32P4LedcTimer *t = chan_timer(s, n);
    uint64_t period = timer_period(t), hp = ch->act_hpoint;
    uint64_t k, v, stop = UINT64_MAX, look;
    bool fading;

    if (!ch->active || !timer_runs(t) || hp >= period) {
        return INT64_MAX;
    }
    timer_pos(t, x, &k, &v);
    if (ch->load_pending) {
        stop = ch->load_cycle;
    }
    if (ch->op_pending) {
        stop = MIN(stop, ch->op_cycle);
    }
    if (t->pend_div || t->pend_res) {
        stop = MIN(stop, t->pend_cycle);
    }
    fading = ch->fading && !ch->paused;
    if (fading) {
        look = LOOK_FADE;
    } else if (chan_duty_at(s, n, k) & 0xf) {
        look = LOOK_DITHER;
    } else {
        look = LOOK_STEADY;
    }
    for (uint64_t kk = k; kk < k + look && kk < stop; kk++) {
        uint64_t lp = chan_lpoint(s, n, kk);
        bool high = hp < lp;
        int64_t tm;

        if (chan_start_level(s, n, kk) != high) {
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

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Drive the channel's output signal, stamped with time `when`: the
 * generator's level while SIG_OUT_EN is in effect, IDLE_LV otherwise.
 */
static void chan_drive(ESP32P4LedcState *s, int n, int64_t when)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    bool level = ch->sig_out_en ? ch->gen_level :
                 !!(ch->conf0 & CONF0_IDLE_LV);

    s->seen_ns = MAX(s->seen_ns, when);
    if (level != ch->out_level) {
        ch->out_level = level;
        esp32_line_set_at(s->out[n], level, when);
    }
}

/* Bring a channel's output to its exact level at x. */
static void chan_sample(ESP32P4LedcState *s, int n, int64_t x)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    if (x <= ch->eval_ns || !ch->sig_out_en) {
        return;
    }
    ch->eval_ns = x;
    ch->gen_level = chan_level_at(s, n, x);
    chan_drive(s, n, x);
}

/* Waves (hw/gpio/esp32_line.h) */

/*
 * The counts at which channel n's generator changes level in cycle k of
 * its timer, into c, in order, and how many (as chan_next_edge() finds
 * them).
 */
static unsigned chan_cycle_edges(ESP32P4LedcState *s, int n, uint64_t k,
                                 uint64_t *c)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    uint64_t period = timer_period(chan_timer(s, n)), hp = ch->act_hpoint;
    uint64_t lp;
    unsigned m = 0;
    bool high;

    if (!ch->active || hp >= period) {
        return 0;
    }
    lp = chan_lpoint(s, n, k);
    high = hp < lp;
    if (chan_start_level(s, n, k) != high) {
        c[m++] = hp;
    }
    if (high && lp < period) {
        c[m++] = lp;
    }
    return m;
}

/*
 * The edges of the cycles after the generator's base cycle, which repeat
 * every sixteen (outside a fade): per[j] for the cycles k with
 * (k - k_dither) % 16 == j, and their sum.
 */
static unsigned chan_cycle_table(ESP32P4LedcState *s, int n, unsigned *per)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    unsigned sum = 0;
    uint64_t c[2];

    for (unsigned i = 0; i < 16; i++) {
        uint64_t k = ch->k_base + 1 + i;

        per[(k - ch->k_dither) & 15] = chan_cycle_edges(s, n, k, c);
        sum += per[(k - ch->k_dither) & 15];
    }
    return sum;
}

/*
 * The edges of channel n's generator from its base cycle up to count v of
 * cycle k, inclusive.
 */
static uint64_t chan_edges_to(ESP32P4LedcState *s, int n, uint64_t k,
                              uint64_t v)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    unsigned per[16], sum, m;
    uint64_t c[2], total = 0, full, j0;

    if (k < ch->k_base) {
        return 0;
    }
    m = chan_cycle_edges(s, n, ch->k_base, c);
    if (k == ch->k_base) {
        return (m > 0 && c[0] <= v) + (m > 1 && c[1] <= v);
    }
    total = m;
    full = k - ch->k_base - 1;
    if (full) {
        sum = chan_cycle_table(s, n, per);
        j0 = (ch->k_base + 1 - ch->k_dither) & 15;
        total += full / 16 * sum;
        for (uint64_t i = 0; i < full % 16; i++) {
            total += per[(j0 + i) & 15];
        }
    }
    m = chan_cycle_edges(s, n, k, c);
    return total + (m > 0 && c[0] <= v) + (m > 1 && c[1] <= v);
}

static ESP32P4LedcState *wave_ledc(Esp32LineWave *w)
{
    return w->opaque;
}

/* Outside a fade under way, the channel's edges are worked out below. */
static bool ledc_wave_describes(Esp32LineWave *w)
{
    ESP32P4LedcChannel *ch = &wave_ledc(w)->ch[w->index];

    return !ch->sig_out_en || !ch->active || !ch->fading || ch->paused;
}

static bool ledc_wave_level(Esp32LineWave *w, int64_t t)
{
    ESP32P4LedcState *s = wave_ledc(w);
    ESP32P4LedcChannel *ch = &s->ch[w->index];

    if (!ch->sig_out_en) {
        return ch->conf0 & CONF0_IDLE_LV;
    }
    return chan_level_at(s, w->index, t);
}

/* The channel's edges by time x, from its generator's base cycle. */
static uint64_t ledc_wave_edges_by(ESP32P4LedcState *s, int n, int64_t x)
{
    uint64_t k, v;

    if (!s->ch[n].sig_out_en) {
        return 0;
    }
    timer_pos(chan_timer(s, n), x, &k, &v);
    return chan_edges_to(s, n, k, v);
}

static uint64_t ledc_wave_edges(Esp32LineWave *w, int64_t a, int64_t b)
{
    ESP32P4LedcState *s = wave_ledc(w);

    if (b <= a) {
        return 0;
    }
    return ledc_wave_edges_by(s, w->index, b) -
           ledc_wave_edges_by(s, w->index, a);
}

/* The time of the generator's m-th edge (m >= 1) from its base cycle. */
static int64_t chan_edge_time(ESP32P4LedcState *s, int n, uint64_t m)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    ESP32P4LedcTimer *t = chan_timer(s, n);
    unsigned per[16], sum, e0;
    uint64_t c[2], k, j0, q, r, i;

    e0 = chan_cycle_edges(s, n, ch->k_base, c);
    if (m <= e0) {
        return due_time(t, timer_time(t, ch->k_base, c[m - 1]));
    }
    m -= e0;
    sum = chan_cycle_table(s, n, per);
    if (!sum) {
        return INT64_MAX;
    }
    q = (m - 1) / sum;
    r = m - q * sum;
    j0 = (ch->k_base + 1 - ch->k_dither) & 15;
    for (i = 0; r > per[(j0 + i) & 15]; i++) {
        r -= per[(j0 + i) & 15];
    }
    if (q > (UINT64_MAX - ch->k_base) / 32) {
        return INT64_MAX;
    }
    k = ch->k_base + 1 + q * 16 + i;
    chan_cycle_edges(s, n, k, c);
    return due_time(t, timer_time(t, k, c[r - 1]));
}

static int64_t ledc_wave_edge(Esp32LineWave *w, int64_t a, uint64_t j)
{
    ESP32P4LedcState *s = wave_ledc(w);

    if (!s->ch[w->index].sig_out_en) {
        return INT64_MAX;
    }
    return chan_edge_time(s, w->index, ledc_wave_edges_by(s, w->index, a) +
                          j);
}

/*
 * The shortest level the output holds: high for DUTY[24:4] counts or one
 * more, low for the rest of the cycle, each count at least the divider's
 * whole part of LEDC_CLK's cycles.
 */
static int64_t ledc_wave_min_hold(Esp32LineWave *w)
{
    ESP32P4LedcState *s = wave_ledc(w);
    ESP32P4LedcChannel *ch = &s->ch[w->index];
    ESP32P4LedcTimer *t = chan_timer(s, w->index);
    uint64_t d = ch->d_anchor, period = timer_period(t);
    uint64_t counts = MIN(d >> 4, period - (d >> 4) - ((d & 0xf) != 0));
    int64_t ns;

    if (!ch->sig_out_en || !ch->active || (d >> 4) >= period || !counts) {
        return 0;
    }
    ns = esp32p4_clk_cycles_ns_floor(esp32p4_clk_exact(s->clk),
                                     counts * (t->div >> 8));
    return ns == INT64_MAX ? 0 : MAX(ns - 1, 0);
}

/* The next update the channel or its timer has waiting. */
static int64_t ledc_wave_until(Esp32LineWave *w)
{
    ESP32P4LedcState *s = wave_ledc(w);
    ESP32P4LedcChannel *ch = &s->ch[w->index];
    ESP32P4LedcTimer *t = chan_timer(s, w->index);
    int64_t until = INT64_MAX;

    if (ch->load_pending) {
        until = MIN(until, due_time(t, timer_time(t, ch->load_cycle, 0)));
    }
    if (ch->op_pending) {
        until = MIN(until, due_time(t, timer_time(t, ch->op_cycle, 0)));
    }
    if ((t->pend_div || t->pend_res) && timer_runs(t)) {
        until = MIN(until, due_time(t, timer_time(t, t->pend_cycle, 0)));
    }
    return until;
}

static const Esp32LineWaveOps ledc_wave_ops = {
    .describes = ledc_wave_describes,
    .level = ledc_wave_level,
    .edges = ledc_wave_edges,
    .edge = ledc_wave_edge,
    .min_hold = ledc_wave_min_hold,
    .until = ledc_wave_until,
};

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Whether channel n's edges are timed one by one: something watches its
 * pad, or a counter takes its wave while it does not describe its edges.
 * A channel that comes to be has its level of the moment at `now`.
 */
static void chan_update_seen(ESP32P4LedcState *s, int n, int64_t now)
{
    bool seen = s->out_watched[n] ||
                (s->out_taken[n] && !ledc_wave_describes(&s->wave[n]));

    if (seen && !s->out_seen[n]) {
        chan_sample(s, n, now);
    }
    s->out_seen[n] = seen;
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * The channels may change at `when`: the counters taking their waves take
 * their edges before it as they were.
 */
static void ledc_waves_changing(ESP32P4LedcState *s, int64_t when)
{
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        if (esp32_line_wave_taken(&s->wave[n])) {
            esp32_line_wave_changing(&s->wave[n], when);
        }
    }
}

/* They have: the counters go on from their new edges. */
static void ledc_waves_changed(ESP32P4LedcState *s, int64_t when)
{
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        if (s->out_taken[n]) {
            chan_update_seen(s, n, when);
        }
        if (esp32_line_wave_taken(&s->wave[n])) {
            esp32_line_wave_changed(&s->wave[n], when);
        }
    }
}

/*
 * The time of the last edge of channel n at or before x, after `from`,
 * from which its level has not changed: INT64_MIN if none. Only the last
 * stretch before x is walked edge by edge, as long as it takes to find
 * one.
 */
static int64_t chan_last_edge(ESP32P4LedcState *s, int n, int64_t from,
                              int64_t x)
{
    for (int64_t w = SKIP_WINDOW_NS;; w *= 2) {
        int64_t start = x - from > w ? x - w : from;
        int64_t last = INT64_MIN;

        for (int64_t e = chan_next_edge(s, n, start); e <= x;
             e = chan_next_edge(s, n, e)) {
            last = e;
        }
        if (last != INT64_MIN || start == from) {
            return last;
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Run a channel's output up to limit (and including it with inclusive),
 * each edge at its own time. On the host's clock an edge the host is too
 * slow for (esp32_line_pwm_skips()) makes the output skip to its level at
 * limit, stamped with the edge that began that level, the edges in
 * between lost: a host that slow cannot follow them, and a skip to any
 * earlier moment would leave the output behind still, skipping again at
 * every edge.
 */
static void chan_eval(ESP32P4LedcState *s, int n, int64_t limit,
                      bool inclusive)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    int64_t last = inclusive ? limit : limit - 1;

    /*
     * An output showing IDLE_LV follows no edges; the generator's level is
     * worked out afresh as SIG_OUT_EN comes back into effect.
     */
    if (!ch->sig_out_en) {
        ch->eval_ns = MAX(ch->eval_ns, last);
        return;
    }
    /*
     * An output that reaches no pad something watches has no edge anyone
     * sees: it takes its level of the moment, as a sample does.
     */
    if (!s->out_seen[n]) {
        chan_sample(s, n, last);
        return;
    }
    for (;;) {
        int64_t e = chan_next_edge(s, n, ch->eval_ns);

        if (e == INT64_MAX || e > last) {
            break;
        }
        if (esp32_line_pwm_skips(&s->lag, e)) {
            int64_t x = last;
            bool level = chan_level_at(s, n, x);
            int64_t began = level == ch->out_level ? x :
                            chan_last_edge(s, n, ch->eval_ns, x);

            if (!s->lag_logged) {
                s->lag_logged = true;
                qemu_log_mask(LOG_UNIMP, "esp32p4_ledc: the host is too "
                              "slow for channel %d's edges: they skip to the "
                              "present\n", n);
            }
            ch->eval_ns = x;
            ch->gen_level = level;
            chan_drive(s, n, began == INT64_MIN ? x : began);
            continue;
        }
        ch->eval_ns = e;
        ch->gen_level = chan_level_at(s, n, e);
        chan_drive(s, n, e);
    }
}

/*
 * The generator goes on from cycle k of its timer at time `when`, from
 * `level`, the level it had: its settings have changed, the counter has
 * jumped (an update of the timer's resolution, a reset) or the channel has
 * moved to another timer. Its output is driven at its level of the moment.
 */
static void chan_rebase_level(ESP32P4LedcState *s, int n, uint64_t k,
                              int64_t when, bool level)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    ch->k_base = k;
    ch->base_level = level;
    ch->gen_level = chan_level_at(s, n, when);
    ch->eval_ns = when;
    chan_drive(s, n, when);
}

/* The level channel n's generator ended cycle k - 1 with. */
static bool chan_end_level(ESP32P4LedcState *s, int n, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    return ch->active ? chan_start_level(s, n, k) : ch->base_level;
}

/* Overflows and matches */

/*
 * The next overflow of timer x that an ETM event waits for (TIME_OVF, or
 * a channel's OVF_CNT_PLS): its cycle, UINT64_MAX if none.
 */
static uint64_t timer_ovf_evt_cycle(ESP32P4LedcState *s, int x)
{
    ESP32P4LedcTimer *t = &s->t[x];
    uint64_t k = UINT64_MAX;

    if (s->evt_task_en[0] & (1u << EVT_TIME_OVF(x))) {
        k = t->ovf_next;
    }
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        if (s->ch[n].timer == x && chan_counting(s, n) &&
            (s->evt_task_en[0] & (1u << EVT_OVF_CNT_PLS(n)))) {
            k = MIN(k, t->ovf_next + chan_ovf_left(s, n) - 1);
        }
    }
    return k;
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Account timer x's overflows up to that of cycle k: TIMERx_OVF_INT and
 * the channels' overflow counts, OVF_CNT_CHn_INT for each count that
 * reaches its limit, and the ETM events of the overflow of cycle k
 * itself, at its time. Earlier ones have none: the caller goes from one
 * overflow with events to the next.
 */
static void timer_account(ESP32P4LedcState *s, int x, uint64_t k)
{
    ESP32P4LedcTimer *t = &s->t[x];
    uint64_t n_ovf = k - t->ovf_next + 1;
    uint32_t evts = 1u << EVT_TIME_OVF(x);

    s->int_raw |= INT_TIMER_OVF(x);
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        ESP32P4LedcChannel *ch = &s->ch[n];
        uint32_t lim = chan_ovf_limit(s, n);
        uint64_t total;

        if (ch->timer != x || !chan_counting(s, n)) {
            continue;
        }
        total = MIN(ch->ovf_cnt, lim - 1) + n_ovf;
        if (total >= lim) {
            s->int_raw |= INT_OVF_CNT(n);
            if (total % lim == 0) {
                evts |= 1u << EVT_OVF_CNT_PLS(n);
            }
        }
        ch->ovf_cnt = total % lim;
    }
    t->ovf_next = k + 1;
    for (int e = 0; evts; e++) {
        if (evts & (1u << e)) {
            evts &= ~(1u << e);
            ledc_etm_event(s, e, due_time(t, timer_time(t, k, timer_top(t))));
        }
    }
}

/*
 * The next overflow or CMP match an ETM event waits for, of any timer: its
 * time (INT64_MAX if none), timer, whether it is a match, and for an
 * overflow its cycle.
 */
static int64_t ledc_next_etm(ESP32P4LedcState *s, int *bx, bool *cmp,
                             uint64_t *bk)
{
    int64_t best = INT64_MAX;

    for (int x = 0; x < ESP32P4_LEDC_TIMERS; x++) {
        ESP32P4LedcTimer *t = &s->t[x];
        uint64_t k = timer_ovf_evt_cycle(s, x);
        int64_t tm;

        if (k != UINT64_MAX) {
            tm = due_time(t, timer_time(t, k, timer_top(t)));
            if (tm < best) {
                best = tm;
                *bx = x;
                *bk = k;
                *cmp = false;
            }
        }
        if ((s->evt_task_en[0] & (1u << EVT_TIMER_CMP(x))) &&
            t->cmp < timer_period(t)) {
            tm = due_time(t, timer_time(t, t->cmp_next, t->cmp));
            if (tm < best) {
                best = tm;
                *bx = x;
                *cmp = true;
            }
        }
    }
    return best;
}

/* [spec:nuos:req:emu.esp32p4.pwm] */
static void ledc_etm_step(ESP32P4LedcState *s, int x, bool cmp, uint64_t k,
                          int64_t when)
{
    if (cmp) {
        s->t[x].cmp_next++;
        ledc_etm_event(s, EVT_TIMER_CMP(x), when);
    } else {
        timer_account(s, x, k);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Account the timers' overflows and CMP matches up to time limit that no
 * ETM event waits for: TIMERx_OVF_INT and the channels' overflow counts.
 */
static void ledc_catch_up(ESP32P4LedcState *s, int64_t limit)
{
    for (int x = 0; x < ESP32P4_LEDC_TIMERS; x++) {
        ESP32P4LedcTimer *t = &s->t[x];
        int64_t last = timer_last_reached(t, limit, timer_top(t));

        if (last >= (int64_t)t->ovf_next) {
            timer_account(s, x, last);
        }
        if (t->cmp < timer_period(t)) {
            last = timer_last_reached(t, limit, t->cmp);
            t->cmp_next = MAX(t->cmp_next, (uint64_t)(last + 1));
        }
    }
}

/* Structural events */

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Channel n's update takes in, at time `when`, at the start of cycle k of
 * its timer: HPOINT, DUTY, SIG_OUT_EN, TIMER_SEL and OVF_CNT_EN, and with
 * DUTY_START the fade; PARA_UP and DUTY_START clear.
 */
static void chan_load(ESP32P4LedcState *s, int n, int64_t when, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    uint32_t sel = CONF0_TIMER_SEL(ch->conf0);
    bool level = chan_end_level(s, n, k);

    if (sel != ch->timer) {
        uint64_t v;

        timer_pos(&s->t[sel], when, &k, &v);
        ch->timer = sel;
    }
    ch->act_hpoint = ch->hpoint & HPOINT_MASK;
    ch->sig_out_en = !!(ch->conf0 & CONF0_SIG_OUT_EN);
    ch->ovf_cnt_en = !!(ch->conf0 & CONF0_OVF_CNT_EN);
    ch->k_dither = k;
    if (ch->load_fade) {
        fade_start(s, n, k, ch->duty & DUTY_MASK);
    } else {
        ch->fading = false;
        ch->paused = false;
        ch->end_due = false;
        ch->k_anchor = k;
        ch->d_anchor = ch->duty & DUTY_MASK;
    }
    ch->active = true;
    ch->load_pending = false;
    ch->conf0 &= ~CONF0_PARA_UP;
    ch->conf1 &= ~CONF1_DUTY_START;
    chan_rebase_level(s, n, k, when, level);
}

/*
 * Ask for channel n's settings, with fade the fade too, at the next
 * overflow of its timer: at once while it stands still.
 */
static void chan_request_load(ESP32P4LedcState *s, int n, bool fade,
                              int64_t now)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    ESP32P4LedcTimer *t;
    uint64_t k, v;

    if (!ch->active) {
        ch->timer = CONF0_TIMER_SEL(ch->conf0);
    }
    t = chan_timer(s, n);
    timer_pos(t, now, &k, &v);
    ch->load_fade = fade;
    ch->conf0 |= CONF0_PARA_UP;
    if (timer_runs(t)) {
        ch->load_pending = true;
        ch->load_cycle = k + 1;
    } else {
        chan_load(s, n, now, k);
    }
}

/* The fade ends as cycle k of the channel's timer starts, at `when`. */
static void fade_end(ESP32P4LedcState *s, int n, int64_t when, uint64_t k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    fade_rebase(s, n, k);
    ch->fading = false;
    ch->end_due = false;
    s->int_raw |= INT_DUTY_CHNG_END(n);
    ledc_etm_event(s, EVT_DUTY_CHNG_END(n), when);
}

/* The cycle of the channel's timer at which its fade ends, if it will. */
static bool fade_end_cycle(ESP32P4LedcState *s, int n, uint64_t *k)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    if (!ch->fading || ch->paused || !ch->end_due) {
        return false;
    }
    *k = fade_run(s, n, UINT64_MAX).k_done;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Fade operations ops of channel n at the overflow that starts cycle k: a
 * pause does not count that overflow, a resume does, and a restart starts
 * the fade afresh from DUTY. With at_once, the timer standing still in
 * cycle k, a pause or resume acts from cycle k on, and a restart from k.
 */
static void fade_ops(ESP32P4LedcState *s, int n, uint32_t ops, uint64_t k,
                     bool at_once)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    uint64_t kf = at_once ? k : k - 1;

    if (ops & OP_RESTART) {
        fade_start(s, n, k, ch->duty & DUTY_MASK);
        return;
    }
    if ((ops & OP_PAUSE) && ch->fading && !ch->paused) {
        fade_rebase(s, n, MAX(kf, ch->k_anchor));
        ch->paused = true;
        ch->gamma_conf |= GAMMA_PAUSE;
    }
    if ((ops & OP_RESUME) && ch->paused) {
        ch->k_anchor = MAX(kf, ch->k_anchor);
        ch->paused = false;
    }
}

static void chan_request_ops(ESP32P4LedcState *s, int n, uint32_t ops,
                             int64_t now)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    ESP32P4LedcTimer *t = chan_timer(s, n);
    uint64_t k, v;

    /* GAMMA_PAUSE reads 1 from a pause until a resume (TRM 58.4.3.3). */
    if (ops & OP_PAUSE) {
        ch->gamma_conf |= GAMMA_PAUSE;
    }
    if (ops & OP_RESUME) {
        ch->gamma_conf &= ~GAMMA_PAUSE;
    }
    timer_pos(t, now, &k, &v);
    if (!timer_runs(t)) {
        fade_ops(s, n, ops, k, true);
        return;
    }
    if (ops & OP_RESTART) {
        ch->op_pending = OP_RESTART;
    } else {
        ch->op_pending |= ops;
    }
    ch->op_cycle = k + 1;
}

/* The timer's waiting divider and resolution, as cycle k starts. */
static void timer_take_update(ESP32P4LedcState *s, int x, int64_t when,
                              uint64_t k)
{
    ESP32P4LedcTimer *t = &s->t[x];
    bool level[ESP32P4_LEDC_CHANNELS];

    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        level[n] = chan_end_level(s, n, k);
    }
    t->anchor_ns = when;
    t->anchor_cycle = k;
    t->anchor_cnt = 0;
    t->ovf_next = MAX(t->ovf_next, k);
    t->cmp_next = MAX(t->cmp_next, k);
    if (t->pend_div) {
        t->div = TCONF_CLK_DIV(t->conf);
    }
    if (t->pend_res) {
        t->res = MIN(TCONF_DUTY_RES(t->conf), MAX_RES);
    }
    t->pend_div = false;
    t->pend_res = false;
    t->conf &= ~TCONF_PARA_UP;
    timer_set_q(s, x);
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        if (s->ch[n].timer == x) {
            chan_rebase_level(s, n, k, when, level[n]);
        }
    }
}

/*
 * The counter has been set to its anchor's count rather than counted to
 * it: an overflow or CMP match comes as the count arrives at the top or
 * TIMERx_CMP by counting, not as it is put there.
 */
static void timer_landed(ESP32P4LedcTimer *t)
{
    if (t->anchor_cnt >= timer_top(t)) {
        t->ovf_next = MAX(t->ovf_next, t->anchor_cycle + 1);
    }
    if (t->anchor_cnt >= t->cmp) {
        t->cmp_next = MAX(t->cmp_next, t->anchor_cycle + 1);
    }
}

/*
 * The counter set to 0 at time x, not an overflow: the cycle goes on,
 * unless the count stood at its top, its overflow seen.
 */
static void timer_reset_count(ESP32P4LedcState *s, int x, int64_t when)
{
    ESP32P4LedcTimer *t = &s->t[x];
    bool level[ESP32P4_LEDC_CHANNELS];
    uint64_t k, v;

    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        level[n] = s->ch[n].timer == x && chan_level_at(s, n, when);
    }
    timer_pos(t, when, &k, &v);
    if (t->ovf_next > k) {
        k++;
    }
    t->anchor_ns = when;
    t->anchor_cycle = k;
    t->anchor_cnt = 0;
    t->cmp_next = k;
    timer_landed(t);
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        if (s->ch[n].timer == x) {
            chan_rebase_level(s, n, k, when, level[n]);
        }
    }
}

/*
 * The earliest structural event due, of the kind first in EV_* order at
 * one moment: its time, kind and timer or channel, INT64_MAX if none.
 */
static int64_t ledc_next_event(ESP32P4LedcState *s, int *kind, int *idx)
{
    int64_t best = INT64_MAX;

    for (int x = 0; x < ESP32P4_LEDC_TIMERS; x++) {
        ESP32P4LedcTimer *t = &s->t[x];

        if ((t->pend_div || t->pend_res) && timer_runs(t)) {
            int64_t tm = due_time(t, timer_time(t, t->pend_cycle, 0));

            if (tm < best) {
                best = tm;
                *kind = EV_TIMER_UPDATE;
                *idx = x;
            }
        }
    }
    for (int k = EV_FADE_END; k <= EV_FADE_OP; k++) {
        for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
            ESP32P4LedcChannel *ch = &s->ch[n];
            ESP32P4LedcTimer *t = chan_timer(s, n);
            uint64_t cyc;
            int64_t tm;

            if (k == EV_FADE_END) {
                if (!fade_end_cycle(s, n, &cyc)) {
                    continue;
                }
            } else if (k == EV_FADE_OP) {
                if (!ch->op_pending) {
                    continue;
                }
                cyc = ch->op_cycle;
            } else {
                if (!ch->load_pending) {
                    continue;
                }
                cyc = ch->load_cycle;
            }
            tm = due_time(t, timer_time(t, cyc, 0));
            if (tm < best) {
                best = tm;
                *kind = k;
                *idx = n;
            }
        }
    }
    return best;
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Bring the whole controller up to time now: its structural events in
 * time order, each after the overflows and edges before it, then the
 * overflows and edges up to now.
 */
static void ledc_advance(ESP32P4LedcState *s, int64_t now)
{
    for (;;) {
        int kind = 0, idx = 0, x = 0;
        bool cmp = false;
        uint64_t k = 0;
        int64_t te = ledc_next_event(s, &kind, &idx);
        int64_t tv = ledc_next_etm(s, &x, &cmp, &k);

        if (MIN(te, tv) > now) {
            break;
        }
        for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
            chan_eval(s, n, MIN(te, tv), false);
        }
        if (tv <= te) {
            /* A task the event causes can change what comes next. */
            ledc_etm_step(s, x, cmp, k, tv);
            continue;
        }
        ledc_catch_up(s, te);
        ledc_waves_changing(s, te);
        switch (kind) {
        case EV_TIMER_UPDATE:
            timer_take_update(s, idx, te, s->t[idx].pend_cycle);
            break;
        case EV_FADE_END: {
            uint64_t k;

            fade_end_cycle(s, idx, &k);
            fade_end(s, idx, te, k);
            break;
        }
        case EV_FADE_OP: {
            ESP32P4LedcChannel *ch = &s->ch[idx];
            uint32_t ops = ch->op_pending;

            ch->op_pending = 0;
            fade_ops(s, idx, ops, ch->op_cycle, false);
            break;
        }
        case EV_LOAD:
            chan_load(s, idx, te, s->ch[idx].load_cycle);
            break;
        }
        ledc_waves_changed(s, te);
    }
    ledc_catch_up(s, now);
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        chan_eval(s, n, now, true);
    }
    s->seen_ns = MAX(s->seen_ns, now);
}

/* The next time anything visible happens, for the QEMU timer. */
static int64_t ledc_next_wake(ESP32P4LedcState *s)
{
    int kind, idx;
    int64_t next = ledc_next_event(s, &kind, &idx);

    for (int x = 0; x < ESP32P4_LEDC_TIMERS; x++) {
        ESP32P4LedcTimer *t = &s->t[x];

        if (!timer_runs(t)) {
            continue;
        }
        if (timer_ovf_watched(s, x)) {
            next = MIN(next, due_time(t, timer_time(t, t->ovf_next,
                                                    timer_top(t))));
        }
        if ((s->evt_task_en[0] & (1u << EVT_TIMER_CMP(x))) &&
            t->cmp < timer_period(t)) {
            next = MIN(next, due_time(t, timer_time(t, t->cmp_next, t->cmp)));
        }
    }
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        ESP32P4LedcChannel *ch = &s->ch[n];
        ESP32P4LedcTimer *t = chan_timer(s, n);
        int64_t e;

        if (timer_runs(t) && chan_ovf_watched(s, n)) {
            uint64_t k = t->ovf_next + chan_ovf_left(s, n) - 1;

            next = MIN(next, due_time(t, timer_time(t, k, timer_top(t))));
        }
        e = ch->sig_out_en && s->out_seen[n] ?
            chan_next_edge(s, n, ch->eval_ns) : INT64_MAX;
        next = MIN(next, e);
    }
    return next;
}

static void ledc_schedule(ESP32P4LedcState *s)
{
    int64_t next = ledc_next_wake(s);

    if (next == INT64_MAX) {
        timer_del(&s->timer);
    } else {
        timer_mod_ns(&s->timer, MAX(next, s->seen_ns));
    }
    ledc_update_irq(s);
}

/*
 * The present: the time of the line change being propagated, if this
 * block acts on one (an ETM task acts at its event's time, even one this
 * block's own event causes while it runs ahead to a later time). Time
 * never goes back from a moment the block has already acted at: an edge
 * driven, an event pulsed, or a time it has been brought up to.
 */
static int64_t ledc_now(ESP32P4LedcState *s)
{
    return MAX(s->seen_ns, esp32_line_time());
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * The timer's catch-up: everything due up to the present as it runs, and
 * how far behind the present that leaves the controller, which tells
 * whether the host is losing ground on it (esp32_line_caught_up()).
 */
static void ledc_timer_cb(void *opaque)
{
    ESP32P4LedcState *s = opaque;
    int64_t now = ledc_now(s);

    esp32_line_catching_up(&s->lag);
    ledc_advance(s, now);
    esp32_line_caught_up(&s->lag, now);
    ledc_schedule(s);
}

/* Software sampling the pads: the outputs at their levels of the moment. */
static void ledc_line_sync(Notifier *notifier, void *data)
{
    ESP32P4LedcState *s = container_of(notifier, ESP32P4LedcState,
                                       line_source);
    int64_t now = ledc_now(s);

    ledc_advance(s, now);
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        chan_sample(s, n, now);
    }
    ledc_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Which channels' outputs reach a pad, through the GPIO matrix or an IO
 * MUX function, with the pad's output enabled, whose level something but
 * software sampling the pads sees (esp32p4_gpio_sig_out_watched()): only
 * theirs are timed edge by edge. The others are brought to their level of
 * the moment whenever the controller is, as when software samples the
 * pads, so that an output nobody watches costs the host nothing however
 * fast it runs. A pad taking one up gets its level at that moment, the
 * matrix bringing the line sources up to the present before a write.
 */
static void ledc_routes_changed(ESP32P4LedcState *s)
{
    int64_t now = ledc_now(s);

    ledc_advance(s, now);
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        s->out_watched[n] = !s->gpio ||
            esp32p4_gpio_sig_out_watched(s->gpio, s->sig_base + n);
        s->out_taken[n] = s->gpio &&
            esp32p4_gpio_sig_out_taken(s->gpio, s->sig_base + n);
        chan_update_seen(s, n, now);
    }
    ledc_schedule(s);
}

static void ledc_route_notify(Notifier *notifier, void *data)
{
    ledc_routes_changed(container_of(notifier, ESP32P4LedcState,
                                     route_notifier));
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * LEDC_CLK changing: the counters keep their counts, and each later count
 * follows the new frequency.
 */
static void ledc_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4LedcState *s = opaque;
    int64_t now = ledc_now(s);

    if (event == ClockPreUpdate) {
        ledc_advance(s, now);
        ledc_waves_changing(s, now);
        return;
    }
    for (int x = 0; x < ESP32P4_LEDC_TIMERS; x++) {
        timer_reanchor(&s->t[x], now);
        timer_set_q(s, x);
    }
    ledc_waves_changed(s, now);
    ledc_schedule(s);
}

/* Registers */

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * TIMERx_CONF. RST holds the count at 0 and takes a new divider and
 * resolution at once; otherwise PARA_UP has them wait for the next
 * overflow. PAUSE stops the count where it stands.
 */
static void ledc_timer_write(ESP32P4LedcState *s, int x, uint32_t v,
                             int64_t now)
{
    ESP32P4LedcTimer *t = &s->t[x];
    bool was_rst = t->conf & TCONF_RST;
    uint64_t k, cnt;

    if (TCONF_DUTY_RES(v) > MAX_RES) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: timer %d duty "
                      "resolution %u is over 20 bits\n", x,
                      TCONF_DUTY_RES(v));
    }
    if (TCONF_CLK_DIV(v) < MIN_DIV) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: timer %d clock divider "
                      "0x%x is below 1; the timer stops\n", x,
                      TCONF_CLK_DIV(v));
    }
    timer_reanchor(t, now);
    t->conf = (t->conf & TCONF_PARA_UP) | (v & TCONF_MASK);
    if (t->conf & TCONF_RST) {
        if (!was_rst) {
            timer_reset_count(s, x, now);
        }
        t->pend_div = true;
        t->pend_res = true;
        t->q = esp32p4_clk_edges_stopped();
        timer_take_update(s, x, now, t->anchor_cycle);
        /* Held at count 0 of its cycle, which has had no overflow. */
        t->ovf_next = t->anchor_cycle;
        t->cmp_next = t->anchor_cycle;
        timer_landed(t);
        return;
    }
    if (v & TCONF_PARA_UP) {
        timer_pos(t, now, &k, &cnt);
        if (!t->pend_div && !t->pend_res) {
            t->pend_cycle = k + 1;
        }
        t->pend_div = true;
        t->pend_res = true;
        t->conf |= TCONF_PARA_UP;
    }
    timer_set_q(s, x);
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * CHn_CONF0: IDLE_LV and OVF_NUM act at once, OVF_CNT_RESET clears the
 * overflow count, PARA_UP asks for the update at the next overflow.
 */
static void ledc_conf0_write(ESP32P4LedcState *s, int n, uint32_t v,
                             int64_t now)
{
    ESP32P4LedcChannel *ch = &s->ch[n];

    ch->conf0 = (ch->conf0 & CONF0_PARA_UP) | (v & CONF0_MASK);
    if (!ch->active && !ch->load_pending) {
        ch->timer = CONF0_TIMER_SEL(ch->conf0);
    }
    if (v & CONF0_OVF_CNT_RESET) {
        ch->ovf_cnt = 0;
    }
    if (v & CONF0_PARA_UP) {
        chan_request_load(s, n, !!(ch->conf1 & CONF1_DUTY_START), now);
    }
    chan_drive(s, n, now);
}

/* CHn_GAMMA_CONF: ENTRY_NUM, and PAUSE and RESUME for the next overflow. */
static void ledc_gamma_conf_write(ESP32P4LedcState *s, int n, uint32_t v,
                                  int64_t now)
{
    ESP32P4LedcChannel *ch = &s->ch[n];
    uint32_t ops = 0;

    ch->gamma_conf = (ch->gamma_conf & GAMMA_PAUSE) |
                     GAMMA_ENTRY_NUM(v);
    if (v & GAMMA_PAUSE) {
        ops |= OP_PAUSE;
    }
    if (v & GAMMA_RESUME) {
        ops |= OP_RESUME;
    }
    if (ops) {
        chan_request_ops(s, n, ops, now);
    }
}

/* The current cycle of channel n's timer. */
static uint64_t chan_cycle(ESP32P4LedcState *s, int n, int64_t now)
{
    uint64_t k, v;

    timer_pos(chan_timer(s, n), now, &k, &v);
    return k;
}

/* [spec:nuos:req:emu.esp32p4.pwm] */
static uint64_t esp32p4_ledc_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LedcState *s = opaque;
    int64_t now = ledc_now(s);
    uint64_t r = 0;

    ledc_advance(s, now);
    if (addr < A_TIMER_BASE) {
        int n = addr / CH_STRIDE;
        ESP32P4LedcChannel *ch = &s->ch[n];

        switch (addr % CH_STRIDE) {
        case CH_CONF0:
            r = ch->conf0;
            break;
        case CH_HPOINT:
            r = ch->hpoint;
            break;
        case CH_DUTY:
            r = ch->duty;
            break;
        case CH_CONF1:
            r = ch->conf1;
            break;
        case CH_DUTY_R:
            r = ch->active ? chan_duty_at(s, n, chan_cycle(s, n, now)) : 0;
            break;
        }
    } else if (addr < A_INT_RAW) {
        int x = (addr - A_TIMER_BASE) / TIMER_STRIDE;
        uint64_t k, v;

        if ((addr - A_TIMER_BASE) % TIMER_STRIDE == 0) {
            r = s->t[x].conf;
        } else {
            timer_pos(&s->t[x], now, &k, &v);
            r = v;
        }
    } else if (addr >= A_GAMMA_CONF &&
               addr < A_GAMMA_CONF + 4 * ESP32P4_LEDC_CHANNELS) {
        r = s->ch[(addr - A_GAMMA_CONF) / 4].gamma_conf;
    } else if (addr >= A_EVT_TASK_EN && addr < A_EVT_TASK_EN + 12) {
        r = s->evt_task_en[(addr - A_EVT_TASK_EN) / 4];
    } else if (addr >= A_TIMER_CMP && addr < A_TIMER_CMP + 16) {
        r = s->t[(addr - A_TIMER_CMP) / 4].cmp;
    } else if (addr >= A_TIMER_CNT_CAP && addr < A_TIMER_CNT_CAP + 16) {
        r = s->t[(addr - A_TIMER_CNT_CAP) / 4].cnt_cap;
    } else if (addr >= A_GAMMA_RAM && addr < A_GAMMA_RAM + GAMMA_RAM_SIZE) {
        unsigned w = (addr - A_GAMMA_RAM) / 4;

        r = s->gamma[w / ESP32P4_LEDC_RANGES][w % ESP32P4_LEDC_RANGES];
    } else {
        switch (addr) {
        case A_INT_RAW:
            r = s->int_raw;
            break;
        case A_INT_ST:
            r = s->int_raw & s->int_ena;
            break;
        case A_INT_ENA:
            r = s->int_ena;
            break;
        case A_INT_CLR:
            break;
        case A_CONF:
            r = s->conf;
            break;
        case A_DATE:
            r = s->date;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: read from reserved "
                          "offset 0x%" HWADDR_PRIx "\n", addr);
            break;
        }
    }
    ledc_schedule(s);
    return r;
}

/* [spec:nuos:req:emu.esp32p4.pwm] */
static void esp32p4_ledc_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    ESP32P4LedcState *s = opaque;
    int64_t now = ledc_now(s);
    uint32_t v = value;

    ledc_advance(s, now);
    ledc_waves_changing(s, now);
    if (addr < A_TIMER_BASE) {
        int n = addr / CH_STRIDE;
        ESP32P4LedcChannel *ch = &s->ch[n];

        switch (addr % CH_STRIDE) {
        case CH_CONF0:
            ledc_conf0_write(s, n, v, now);
            break;
        case CH_HPOINT:
            ch->hpoint = v & HPOINT_MASK;
            break;
        case CH_DUTY:
            ch->duty = v & DUTY_MASK;
            break;
        case CH_CONF1:
            ch->conf1 = v & CONF1_DUTY_START;
            break;
        case CH_DUTY_R:
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: write to read-only "
                          "DUTY_R of channel %d\n", n);
            break;
        }
    } else if (addr < A_INT_RAW) {
        int x = (addr - A_TIMER_BASE) / TIMER_STRIDE;

        if ((addr - A_TIMER_BASE) % TIMER_STRIDE == 0) {
            ledc_timer_write(s, x, v, now);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: write to read-only "
                          "VALUE of timer %d\n", x);
        }
    } else if (addr >= A_GAMMA_CONF &&
               addr < A_GAMMA_CONF + 4 * ESP32P4_LEDC_CHANNELS) {
        ledc_gamma_conf_write(s, (addr - A_GAMMA_CONF) / 4, v, now);
    } else if (addr >= A_EVT_TASK_EN && addr < A_EVT_TASK_EN + 12) {
        s->evt_task_en[(addr - A_EVT_TASK_EN) / 4] =
            v & (addr == A_EVT_TASK_EN + 8 ? 0xffffffu : 0xffffffffu);
    } else if (addr >= A_TIMER_CMP && addr < A_TIMER_CMP + 16) {
        int x = (addr - A_TIMER_CMP) / 4;
        ESP32P4LedcTimer *t = &s->t[x];
        uint64_t k, cnt;

        t->cmp = v & TIMER_CNT_MASK;
        timer_pos(t, now, &k, &cnt);
        t->cmp_next = cnt >= t->cmp ? k + 1 : k;
    } else if (addr >= A_TIMER_CNT_CAP && addr < A_TIMER_CNT_CAP + 16) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: write to read-only "
                      "CNT_CAP of timer %d\n",
                      (int)(addr - A_TIMER_CNT_CAP) / 4);
    } else if (addr >= A_GAMMA_RAM && addr < A_GAMMA_RAM + GAMMA_RAM_SIZE) {
        unsigned w = (addr - A_GAMMA_RAM) / 4;
        int n = w / ESP32P4_LEDC_RANGES;

        fade_rebase(s, n, MAX(chan_cycle(s, n, now), s->ch[n].k_anchor));
        s->gamma[n][w % ESP32P4_LEDC_RANGES] = v & RANGE_MASK;
    } else {
        switch (addr) {
        case A_INT_ENA:
            s->int_ena = v & INT_MASK;
            break;
        case A_INT_RAW:
        case A_INT_CLR:
            s->int_raw &= ~v;
            break;
        case A_CONF:
            s->conf = v & CONF_MASK;
            break;
        case A_DATE:
            s->date = v & DATE_MASK;
            break;
        case A_INT_ST:
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: write to read-only "
                          "INT_ST\n");
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4-ledc: write to reserved "
                          "offset 0x%" HWADDR_PRIx "\n", addr);
            break;
        }
    }
    ledc_advance(s, now);
    ledc_waves_changed(s, now);
    ledc_schedule(s);
}

static const MemoryRegionOps esp32p4_ledc_ops = {
    .read = esp32p4_ledc_read,
    .write = esp32p4_ledc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * [spec:nuos:req:emu.esp32p4.etm]
 * An ETM task, acted on as its line rises while its enable is set, at the
 * time of the event that generated it.
 */
static void esp32p4_ledc_etm_task(void *opaque, int line, int level)
{
    ESP32P4LedcState *s = opaque;
    int64_t now;
    int kind, i;

    if (!level) {
        return;
    }
    for (kind = ARRAY_SIZE(ledc_tasks) - 1; ledc_tasks[kind].first > line;
         kind--) {
    }
    i = line - ledc_tasks[kind].first;
    if (!(s->evt_task_en[ledc_tasks[kind].reg] &
          (1u << (ledc_tasks[kind].bit + i)))) {
        return;
    }
    now = ledc_now(s);
    ledc_advance(s, now);
    ledc_waves_changing(s, now);
    switch (kind) {
    case TASK_RES_UPDATE: {
        ESP32P4LedcTimer *t = &s->t[i];
        uint64_t k, v;

        timer_pos(t, now, &k, &v);
        if (!t->pend_div && !t->pend_res) {
            t->pend_cycle = k + 1;
        }
        t->pend_res = true;
        break;
    }
    case TASK_DUTY_SCALE_UPDATE: {
        ESP32P4LedcChannel *ch = &s->ch[i];

        fade_rebase(s, i, MAX(chan_cycle(s, i, now), ch->k_anchor));
        if (ch->fading && ch->range < ch->entry_num) {
            uint32_t ram = s->gamma[i][ch->range];

            ch->param = (ch->param & ~(0x3ffu << 11)) | (ram & (0x3ffu << 11));
        }
        break;
    }
    case TASK_TIMER_CAP: {
        uint64_t k, v;

        timer_pos(&s->t[i], now, &k, &v);
        s->t[i].cnt_cap = v;
        break;
    }
    case TASK_SIG_OUT_DIS:
        s->ch[i].conf0 &= ~CONF0_SIG_OUT_EN;
        s->ch[i].sig_out_en = false;
        chan_drive(s, i, now);
        break;
    case TASK_OVF_CNT_RST:
        s->ch[i].ovf_cnt = 0;
        break;
    case TASK_TIMER_RST:
        timer_reanchor(&s->t[i], now);
        timer_reset_count(s, i, now);
        break;
    case TASK_TIMER_RESUME:
    case TASK_TIMER_PAUSE:
        timer_reanchor(&s->t[i], now);
        s->t[i].etm_paused = !s->t[i].etm_paused;
        timer_set_q(s, i);
        break;
    case TASK_GAMMA_RESTART:
        chan_request_ops(s, i, OP_RESTART, now);
        break;
    case TASK_GAMMA_PAUSE:
        chan_request_ops(s, i, OP_PAUSE, now);
        break;
    case TASK_GAMMA_RESUME:
        chan_request_ops(s, i, OP_RESUME, now);
        break;
    }
    ledc_advance(s, now);
    ledc_waves_changed(s, now);
    ledc_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * RST_EN_LEDC: every timer held in reset, every channel off at its idle
 * level, low. The gamma RAM, a memory, keeps its contents.
 */
static void esp32p4_ledc_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LedcState *s = ESP32P4_LEDC(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    ledc_waves_changing(s, now);
    timer_del(&s->timer);
    s->seen_ns = now;
    for (int x = 0; x < ESP32P4_LEDC_TIMERS; x++) {
        ESP32P4LedcTimer *t = &s->t[x];

        memset(t, 0, sizeof(*t));
        t->conf = TCONF_RESET;
        t->anchor_ns = now;
        timer_landed(t);
    }
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        ESP32P4LedcChannel *ch = &s->ch[n];

        memset(ch, 0, sizeof(*ch));
        ch->eval_ns = now;
    }
    s->int_raw = 0;
    s->int_ena = 0;
    memset(s->evt_task_en, 0, sizeof(s->evt_task_en));
    s->conf = 0;
    s->date = DATE_RESET;
    ledc_waves_changed(s, now);
}

static void esp32p4_ledc_reset_exit(Object *obj, ResetType type)
{
    ESP32P4LedcState *s = ESP32P4_LEDC(obj);

    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        qemu_set_irq(s->out[n], 0);
    }
    s->irq_level = false;
    qemu_set_irq(s->irq, 0);
    ledc_routes_changed(s);
}

static void esp32p4_ledc_init(Object *obj)
{
    ESP32P4LedcState *s = ESP32P4_LEDC(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_ledc_ops, s,
                          TYPE_ESP32P4_LEDC, ESP32P4_LEDC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->out, ESP32P4_LEDC_OUT,
                             ESP32P4_LEDC_CHANNELS);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_LEDC_ETM_EVENT,
                             ESP32P4_LEDC_ETM_EVENTS);
    qdev_init_gpio_in_named(dev, esp32p4_ledc_etm_task, ESP32P4_LEDC_ETM_TASK,
                            ESP32P4_LEDC_ETM_TASKS);
    s->clk = qdev_init_clock_in(dev, "clk", ledc_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, ledc_timer_cb, s);
    s->line_source.notify = ledc_line_sync;
    s->route_notifier.notify = ledc_route_notify;
    for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        esp32_line_wave_init(&s->wave[n], &ledc_wave_ops, s, n);
    }
}

static void esp32p4_ledc_realize(DeviceState *dev, Error **errp)
{
    ESP32P4LedcState *s = ESP32P4_LEDC(dev);

    esp32_line_add_source(&s->line_source);
    if (s->gpio) {
        esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
        for (int n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
            esp32p4_gpio_set_wave(s->gpio, s->sig_base + n, &s->wave[n]);
        }
    }
}

static const Property esp32p4_ledc_properties[] = {
    DEFINE_PROP_LINK("gpio", ESP32P4LedcState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
    DEFINE_PROP_UINT32("sig-base", ESP32P4LedcState, sig_base, 0),
};

static const VMStateDescription vmstate_esp32p4_ledc_timer = {
    .name = TYPE_ESP32P4_LEDC "-timer",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf, ESP32P4LedcTimer),
        VMSTATE_UINT32(cmp, ESP32P4LedcTimer),
        VMSTATE_UINT32(cnt_cap, ESP32P4LedcTimer),
        VMSTATE_UINT32(div, ESP32P4LedcTimer),
        VMSTATE_UINT32(res, ESP32P4LedcTimer),
        VMSTATE_BOOL(pend_div, ESP32P4LedcTimer),
        VMSTATE_BOOL(pend_res, ESP32P4LedcTimer),
        VMSTATE_UINT64(pend_cycle, ESP32P4LedcTimer),
        VMSTATE_BOOL(etm_paused, ESP32P4LedcTimer),
        VMSTATE_INT64(anchor_ns, ESP32P4LedcTimer),
        VMSTATE_UINT64(anchor_cycle, ESP32P4LedcTimer),
        VMSTATE_UINT32(anchor_cnt, ESP32P4LedcTimer),
        VMSTATE_ESP32P4_CLK_EDGES(q, ESP32P4LedcTimer),
        VMSTATE_INT64(q_start, ESP32P4LedcTimer),
        VMSTATE_ESP32P4_CLK_EDGES(q_clk, ESP32P4LedcTimer),
        VMSTATE_UINT64(ovf_next, ESP32P4LedcTimer),
        VMSTATE_UINT64(cmp_next, ESP32P4LedcTimer),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_ledc_channel = {
    .name = TYPE_ESP32P4_LEDC "-channel",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, ESP32P4LedcChannel),
        VMSTATE_UINT32(hpoint, ESP32P4LedcChannel),
        VMSTATE_UINT32(duty, ESP32P4LedcChannel),
        VMSTATE_UINT32(conf1, ESP32P4LedcChannel),
        VMSTATE_UINT32(gamma_conf, ESP32P4LedcChannel),
        VMSTATE_BOOL(active, ESP32P4LedcChannel),
        VMSTATE_UINT32(timer, ESP32P4LedcChannel),
        VMSTATE_BOOL(sig_out_en, ESP32P4LedcChannel),
        VMSTATE_BOOL(ovf_cnt_en, ESP32P4LedcChannel),
        VMSTATE_UINT32(act_hpoint, ESP32P4LedcChannel),
        VMSTATE_UINT64(k_dither, ESP32P4LedcChannel),
        VMSTATE_BOOL(load_pending, ESP32P4LedcChannel),
        VMSTATE_BOOL(load_fade, ESP32P4LedcChannel),
        VMSTATE_UINT64(load_cycle, ESP32P4LedcChannel),
        VMSTATE_UINT64(k_anchor, ESP32P4LedcChannel),
        VMSTATE_UINT32(d_anchor, ESP32P4LedcChannel),
        VMSTATE_BOOL(fading, ESP32P4LedcChannel),
        VMSTATE_BOOL(paused, ESP32P4LedcChannel),
        VMSTATE_BOOL(end_due, ESP32P4LedcChannel),
        VMSTATE_UINT32(range, ESP32P4LedcChannel),
        VMSTATE_UINT32(steps, ESP32P4LedcChannel),
        VMSTATE_UINT32(count, ESP32P4LedcChannel),
        VMSTATE_UINT32(param, ESP32P4LedcChannel),
        VMSTATE_UINT32(entry_num, ESP32P4LedcChannel),
        VMSTATE_UINT32(op_pending, ESP32P4LedcChannel),
        VMSTATE_UINT64(op_cycle, ESP32P4LedcChannel),
        VMSTATE_UINT32(ovf_cnt, ESP32P4LedcChannel),
        VMSTATE_UINT64(k_base, ESP32P4LedcChannel),
        VMSTATE_BOOL(base_level, ESP32P4LedcChannel),
        VMSTATE_BOOL(gen_level, ESP32P4LedcChannel),
        VMSTATE_INT64(eval_ns, ESP32P4LedcChannel),
        VMSTATE_BOOL(out_level, ESP32P4LedcChannel),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_ledc = {
    .name = TYPE_ESP32P4_LEDC,
    .version_id = 5,
    .minimum_version_id = 5,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4LedcState),
        VMSTATE_TIMER(timer, ESP32P4LedcState),
        VMSTATE_STRUCT_ARRAY(t, ESP32P4LedcState, ESP32P4_LEDC_TIMERS, 2,
                             vmstate_esp32p4_ledc_timer, ESP32P4LedcTimer),
        VMSTATE_STRUCT_ARRAY(ch, ESP32P4LedcState, ESP32P4_LEDC_CHANNELS, 1,
                             vmstate_esp32p4_ledc_channel,
                             ESP32P4LedcChannel),
        VMSTATE_UINT32_2DARRAY(gamma, ESP32P4LedcState, ESP32P4_LEDC_CHANNELS,
                               ESP32P4_LEDC_RANGES),
        VMSTATE_UINT32(int_raw, ESP32P4LedcState),
        VMSTATE_UINT32(int_ena, ESP32P4LedcState),
        VMSTATE_UINT32_ARRAY(evt_task_en, ESP32P4LedcState, 3),
        VMSTATE_UINT32(conf, ESP32P4LedcState),
        VMSTATE_UINT32(date, ESP32P4LedcState),
        VMSTATE_INT64(seen_ns, ESP32P4LedcState),
        VMSTATE_BOOL(irq_level, ESP32P4LedcState),
        VMSTATE_BOOL_ARRAY(out_seen, ESP32P4LedcState,
                           ESP32P4_LEDC_CHANNELS),
        VMSTATE_BOOL_ARRAY(out_watched, ESP32P4LedcState,
                           ESP32P4_LEDC_CHANNELS),
        VMSTATE_BOOL_ARRAY(out_taken, ESP32P4LedcState,
                           ESP32P4_LEDC_CHANNELS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_ledc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_ledc_realize;
    device_class_set_props(dc, esp32p4_ledc_properties);
    dc->vmsd = &vmstate_esp32p4_ledc;
    rc->phases.hold = esp32p4_ledc_reset_hold;
    rc->phases.exit = esp32p4_ledc_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.pwm] */
static const TypeInfo esp32p4_ledc_info = {
    .name          = TYPE_ESP32P4_LEDC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4LedcState),
    .instance_init = esp32p4_ledc_init,
    .class_init    = esp32p4_ledc_class_init,
};

static void esp32p4_ledc_register_types(void)
{
    type_register_static(&esp32p4_ledc_info);
}

type_init(esp32p4_ledc_register_types)
