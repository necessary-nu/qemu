/*
 * RP2350 pulse width modulation (PWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "PWM", and the pico-sdk pwm.h register
 * descriptions.
 *
 * Twelve slices, each a 16-bit counter behind a fractional clock divider.
 * The divider's input events are clk_sys cycles (free-running), clk_sys
 * cycles while the B pin is high (level-gated), or rising or falling
 * edges of the B pin. The divider is first-order sigma-delta: every
 * event adds 16 to an accumulator, and each time the accumulator reaches
 * DIV (in sixteenths) it emits a count enable and drops by DIV. The
 * counter counts 0..TOP and wraps to 0, or in phase-correct mode counts
 * 0..TOP, holds TOP for one more count while turning round, counts back
 * down to 0 and holds 0 for one more count, which is the wrap. TOP and CC
 * are double-buffered: the counter uses copies latched at each wrap, and
 * while a slice is disabled the copies follow software's writes. A
 * channel's output is high while the counter is below its compare value,
 * so CC = 0 is 0% and CC > TOP is 100% duty.
 *
 * Counters are not ticked: each slice holds its state as of a clk_sys
 * cycle and is run forward in closed form to the current cycle whenever
 * it is observed or changed, and a per-slice timer fires only at the
 * moments something becomes visible outside the block: a wrap whose
 * interrupt is enabled (and not yet raised), an output edge, a B pin edge
 * passing the input synchroniser, or a DREQ. Output edges and DREQs
 * closer together than the min-edge-ns property are coalesced: the timer
 * then fires every min-edge-ns, the outputs show the slice's true level
 * at each firing, and the DREQ line is pulsed once per wrap since the
 * last firing (at most RP2350_PWM_DREQ_BURST times). Wrap interrupts are
 * never coalesced.
 *
 * clk_sys is the "clk" clock input.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_pwm.h"
#include "migration/vmstate.h"

#define SLICE_STRIDE            0x14
#define A_CSR                   0x00
#define A_DIV                   0x04
#define A_CTR                   0x08
#define A_CC                    0x0c
#define A_TOP                   0x10
#define A_EN                    0xf0
#define A_INTR                  0xf4
#define A_IRQ0_INTE             0xf8
#define A_IRQ0_INTF             0xfc
#define A_IRQ0_INTS             0x100
#define A_IRQ1_INTE             0x104
#define A_IRQ1_INTF             0x108
#define A_IRQ1_INTS             0x10c

#define CSR_EN                  (1u << 0)
#define CSR_PH_CORRECT          (1u << 1)
#define CSR_A_INV               (1u << 2)
#define CSR_B_INV               (1u << 3)
#define CSR_DIVMODE_SHIFT       4
#define CSR_DIVMODE             (3u << CSR_DIVMODE_SHIFT)
#define CSR_PH_RET              (1u << 6)
#define CSR_PH_ADV              (1u << 7)
#define CSR_RW                  0x3fu
#define CSR_PHASE               (CSR_PH_RET | CSR_PH_ADV)

enum {
    DIVMODE_DIV,
    DIVMODE_LEVEL,
    DIVMODE_RISE,
    DIVMODE_FALL,
};

#define DIV_MASK                0xfffu
#define DIV_RESET               0x10u
#define DIV_INT(v)              (((v) >> 4) & 0xff)
#define DIV_FRAC(v)             ((v) & 0xf)
#define CTR_MASK                0xffffu
#define TOP_RESET               0xffffu
#define SLICES_MASK             ((1u << RP2350_PWM_SLICES) - 1)

/* The most DREQ pulses one coalesced timer firing delivers. */
#define RP2350_PWM_DREQ_BURST   64

#define NO_EVENT                UINT64_MAX

/*
 * A stretch of the counter's path in which it counts steadily one way:
 * `len` counts starting at tick offset `off` from value `ctr`.
 */
typedef struct PWMSeg {
    uint64_t off;
    uint32_t ctr;
    uint32_t len;
    bool down;
} PWMSeg;

#define PWM_MAX_SEGS 3

static unsigned pwm_divmode(const RP2350PWMSlice *sl)
{
    return (sl->csr & CSR_DIVMODE) >> CSR_DIVMODE_SHIFT;
}

static bool pwm_ph_correct(const RP2350PWMSlice *sl)
{
    return sl->csr & CSR_PH_CORRECT;
}

/* The divisor in sixteenths; DIV_INT 0 divides by 256. */
static uint32_t pwm_div16(const RP2350PWMSlice *sl)
{
    uint32_t i = DIV_INT(sl->div);

    return (i ? i : 256) * 16 + DIV_FRAC(sl->div);
}

static uint32_t pwm_chan_cc(uint32_t cc, int ch)
{
    return ch ? cc >> 16 : cc & 0xffff;
}

static uint64_t pwm_cycle_at(RP2350PWMState *s, int64_t ns)
{
    if (ns <= s->base_ns) {
        return s->base_cycle;
    }
    return s->base_cycle + clock_ns_to_ticks(s->clk, ns - s->base_ns);
}

/* The first virtual time at which clk_sys has reached `cycle`. */
static int64_t pwm_ns_at(RP2350PWMState *s, uint64_t cycle)
{
    uint64_t d, ns;

    if (cycle <= s->base_cycle) {
        return s->base_ns;
    }
    d = cycle - s->base_cycle;
    ns = clock_ticks_to_ns(s->clk, d);
    while (clock_ns_to_ticks(s->clk, ns) < d) {
        ns++;
    }
    return s->base_ns + MIN(ns, (uint64_t)INT64_MAX / 2);
}

/*
 * The counter's path from (ctr, down) up to the wrap, with latched wrap
 * value `top`, starting at tick offset `off`. *wrap is the offset of the
 * count that wraps. A counter above TOP counts on through 0xffff and 0
 * before it can reach TOP: the wrap is taken only on reaching TOP.
 */
static int pwm_segments(uint32_t ctr, bool down, uint32_t top, bool ph,
                        uint64_t off, PWMSeg *seg, uint64_t *wrap)
{
    int n = 0;

    if (ph && down) {
        seg[n++] = (PWMSeg){ off, ctr, ctr + 1, true };
        *wrap = off + ctr + 1;
        return n;
    }
    if (ctr > top) {
        seg[n++] = (PWMSeg){ off, ctr, 0x10000 - ctr, false };
        off += 0x10000 - ctr;
        ctr = 0;
    }
    seg[n++] = (PWMSeg){ off, ctr, top - ctr + 1, false };
    off += top - ctr + 1;
    if (ph) {
        /* The turn at TOP repeats TOP, counting down. */
        seg[n++] = (PWMSeg){ off, top, top + 1, true };
        off += top + 1;
    }
    *wrap = off;
    return n;
}

static void pwm_seg_pos(const PWMSeg *seg, int n, uint64_t k, uint32_t *ctr,
                        bool *down)
{
    int i = n - 1;
    uint32_t d;

    while (i > 0 && seg[i].off > k) {
        i--;
    }
    d = k - seg[i].off;
    *ctr = (seg[i].down ? seg[i].ctr - d : seg[i].ctr + d) & CTR_MASK;
    *down = seg[i].down;
}

/*
 * The first tick offset, after the start of seg[0], at which
 * `ctr < cc` changes along the path, or NO_EVENT.
 */
static uint64_t pwm_seg_change(const PWMSeg *seg, int n, uint32_t cc)
{
    bool level = seg[0].ctr < cc;
    int i;

    for (i = 0; i < n; i++) {
        const PWMSeg *g = &seg[i];

        if (i > 0 && (g->ctr < cc) != level) {
            return g->off;
        }
        if (!g->down) {
            uint32_t last = g->ctr + g->len - 1;

            if (g->ctr < cc && cc <= last) {
                return g->off + (cc - g->ctr);
            }
        } else {
            uint32_t last = g->ctr - (g->len - 1);

            if (cc >= 1 && cc <= g->ctr && cc - 1 >= last) {
                return g->off + (g->ctr - (cc - 1));
            }
        }
    }
    return NO_EVENT;
}

static void pwm_latch(RP2350PWMSlice *sl)
{
    sl->top_latched = sl->top;
    sl->cc_latched = sl->cc;
}

/* Count `ticks` times; returns the number of wraps. */
static uint64_t pwm_count(RP2350PWMSlice *sl, uint64_t ticks)
{
    PWMSeg seg[PWM_MAX_SEGS];
    bool ph = pwm_ph_correct(sl);
    uint64_t wrap, wraps = 0;
    int n;

    if (!ticks) {
        return 0;
    }
    n = pwm_segments(sl->ctr, sl->down, sl->top_latched, ph, 0, seg, &wrap);
    if (ticks >= wrap) {
        uint64_t period;

        ticks -= wrap;
        pwm_latch(sl);
        wraps = 1;
        /* Later periods all run with the values just latched. */
        period = (uint64_t)(sl->top_latched + 1) << ph;
        wraps += ticks / period;
        ticks %= period;
        n = pwm_segments(0, false, sl->top_latched, ph, 0, seg, &wrap);
    }
    pwm_seg_pos(seg, n, ticks, &sl->ctr, &sl->down);
    return wraps;
}

static void pwm_ticks(RP2350PWMState *s, int i, uint64_t ticks)
{
    RP2350PWMSlice *sl = &s->slice[i];
    uint64_t wraps = pwm_count(sl, ticks);

    if (wraps) {
        s->intr |= 1u << i;
        sl->dreq_pending = MIN(sl->dreq_pending + MIN(wraps,
                                                      RP2350_PWM_DREQ_BURST),
                               RP2350_PWM_DREQ_BURST);
    }
}

/* Pass `events` divider input events through the fractional divider. */
static void pwm_events(RP2350PWMState *s, int i, uint64_t events)
{
    RP2350PWMSlice *sl = &s->slice[i];
    uint32_t div16 = pwm_div16(sl);
    uint64_t total, pulses;

    if (!events) {
        return;
    }
    total = sl->acc + events * 16;
    pulses = total / div16;
    sl->acc = total % div16;
    /* PH_RET deletes the next count enable. */
    if (pulses && (sl->csr & CSR_PH_RET)) {
        pulses--;
        sl->csr &= ~CSR_PH_RET;
    }
    pwm_ticks(s, i, pulses);
}

/*
 * Whether the divider's output has cycles without a count enable, into
 * which PH_ADV can insert one: never at full speed, since the counter
 * cannot count twice in a cycle.
 */
static bool pwm_has_gap(const RP2350PWMSlice *sl)
{
    switch (pwm_divmode(sl)) {
    case DIVMODE_DIV:
        return pwm_div16(sl) > 16;
    case DIVMODE_LEVEL:
        return !sl->b_seen || pwm_div16(sl) > 16;
    default:
        return true;
    }
}

static bool pwm_running(RP2350PWMState *s, const RP2350PWMSlice *sl)
{
    return (sl->csr & CSR_EN) && clock_is_enabled(s->clk);
}

/*
 * PH_ADV inserts a count enable into the next gap; at most a few cycles
 * away, so it is taken at once.
 */
static void pwm_try_advance(RP2350PWMState *s, int i)
{
    RP2350PWMSlice *sl = &s->slice[i];

    if ((sl->csr & CSR_PH_ADV) && pwm_running(s, sl) && pwm_has_gap(sl)) {
        sl->csr &= ~CSR_PH_ADV;
        pwm_ticks(s, i, 1);
    }
}

/*
 * Run slice i up to clk_sys cycle `cycle`. A B pin change is seen by the
 * slice in the cycle after it arrives; two changes in one cycle cancel,
 * as the synchroniser never samples the level between them.
 */
/* [spec:nuos:req:emu.pwm] */
static void pwm_sync(RP2350PWMState *s, int i, uint64_t cycle)
{
    RP2350PWMSlice *sl = &s->slice[i];

    while (sl->sync_cycle < cycle) {
        bool en = sl->csr & CSR_EN;
        unsigned mode = pwm_divmode(sl);
        uint64_t end = cycle;
        bool edge = false;

        if (sl->b_in != sl->b_seen && sl->b_change + 1 <= cycle) {
            end = MAX(sl->b_change + 1, sl->sync_cycle);
            edge = true;
        }
        if (en && (mode == DIVMODE_DIV ||
                   (mode == DIVMODE_LEVEL && sl->b_seen))) {
            pwm_events(s, i, end - sl->sync_cycle);
        }
        sl->sync_cycle = end;
        if (edge) {
            sl->b_seen = sl->b_in;
            if (en && ((mode == DIVMODE_RISE && sl->b_seen) ||
                       (mode == DIVMODE_FALL && !sl->b_seen))) {
                pwm_events(s, i, 1);
            }
        }
    }
    pwm_try_advance(s, i);
}

static uint64_t pwm_now_cycle(RP2350PWMState *s)
{
    return pwm_cycle_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void pwm_sync_all(RP2350PWMState *s)
{
    uint64_t cycle = pwm_now_cycle(s);
    int i;

    for (i = 0; i < RP2350_PWM_SLICES; i++) {
        pwm_sync(s, i, cycle);
    }
}

static void pwm_update_irq(RP2350PWMState *s)
{
    int j;

    for (j = 0; j < RP2350_PWM_IRQS; j++) {
        qemu_set_irq(s->irq[j], !!((s->intr & s->inte[j]) | s->intf[j]));
    }
}

/* Drive slice i's outputs and DREQ from its state. */
static void pwm_drive(RP2350PWMState *s, int i)
{
    RP2350PWMSlice *sl = &s->slice[i];
    int ch;

    for (ch = 0; ch < 2; ch++) {
        bool inv = sl->csr & (ch ? CSR_B_INV : CSR_A_INV);
        uint8_t level = (sl->ctr < pwm_chan_cc(sl->cc_latched, ch)) ^ inv;
        /* Outside free-running mode the B pin is the slice's input. */
        uint8_t oe = !ch || pwm_divmode(sl) == DIVMODE_DIV;

        /*
         * Record the level before driving it: driving a pin can feed
         * back into this device through the B input.
         */
        if (sl->oe_level[ch] != oe) {
            sl->oe_level[ch] = oe;
            qemu_set_irq(s->oe[i * 2 + ch], oe);
        }
        if (sl->out_level[ch] != level) {
            sl->out_level[ch] = level;
            qemu_set_irq(s->out[i * 2 + ch], level);
        }
    }

    if (!s->dreq[i]) {
        sl->dreq_pending = 0;
    }
    while (sl->dreq_pending) {
        sl->dreq_pending--;
        qemu_irq_pulse(s->dreq[i]);
    }
}

/* The clk_sys cycle in which the k-th count (k >= 1) from now happens. */
static uint64_t pwm_tick_cycle(const RP2350PWMSlice *sl, uint64_t k)
{
    uint64_t pulses = k + !!(sl->csr & CSR_PH_RET);
    uint64_t need = pulses * pwm_div16(sl) - sl->acc;

    return sl->sync_cycle + DIV_ROUND_UP(need, 16);
}

/* The count offset at which channel ch's output next changes. */
static uint64_t pwm_next_change(const RP2350PWMSlice *sl, int ch)
{
    PWMSeg seg[PWM_MAX_SEGS];
    bool ph = pwm_ph_correct(sl);
    uint32_t cc_now = pwm_chan_cc(sl->cc_latched, ch);
    uint32_t cc_next = pwm_chan_cc(sl->cc, ch);
    uint64_t wrap, wrap2, k;
    int n;

    n = pwm_segments(sl->ctr, sl->down, sl->top_latched, ph, 0, seg, &wrap);
    k = pwm_seg_change(seg, n, cc_now);
    if (k != NO_EVENT) {
        return k;
    }
    if ((sl->ctr < cc_now) != (0 < cc_next)) {
        return wrap;
    }
    /* After the wrap every period is alike: one period settles it. */
    n = pwm_segments(0, false, sl->top, ph, wrap, seg, &wrap2);
    return pwm_seg_change(seg, n, cc_next);
}

static void pwm_schedule(RP2350PWMState *s, int i)
{
    RP2350PWMSlice *sl = &s->slice[i];
    unsigned mode = pwm_divmode(sl);
    int64_t due = INT64_MAX;

    sl->out_due_ns = INT64_MAX;
    if (!pwm_running(s, sl)) {
        timer_del(sl->timer);
        return;
    }

    if (mode != DIVMODE_DIV && sl->b_in != sl->b_seen) {
        due = pwm_ns_at(s, sl->b_change + 1);
    }

    if (mode == DIVMODE_DIV || (mode == DIVMODE_LEVEL && sl->b_seen)) {
        PWMSeg seg[PWM_MAX_SEGS];
        bool irq_wrap = !(s->intr & (1u << i)) &&
                        ((s->inte[0] | s->inte[1]) & (1u << i));
        uint64_t wrap, k = NO_EVENT;
        int64_t floor = sl->last_out_ns + s->min_edge_ns;
        int ch;

        pwm_segments(sl->ctr, sl->down, sl->top_latched, pwm_ph_correct(sl),
                     0, seg, &wrap);
        if (irq_wrap) {
            due = MIN(due, pwm_ns_at(s, pwm_tick_cycle(sl, wrap)));
        } else if (s->dreq[i]) {
            k = wrap;
        }
        for (ch = 0; ch < (mode == DIVMODE_DIV ? 2 : 1); ch++) {
            k = MIN(k, pwm_next_change(sl, ch));
        }
        if (k != NO_EVENT) {
            sl->out_due_ns = MAX(pwm_ns_at(s, pwm_tick_cycle(sl, k)), floor);
            due = MIN(due, sl->out_due_ns);
        }
    }

    if (due == INT64_MAX) {
        timer_del(sl->timer);
    } else {
        timer_mod(sl->timer, due);
    }
}

static void pwm_update(RP2350PWMState *s, int i)
{
    pwm_drive(s, i);
    pwm_update_irq(s);
    pwm_schedule(s, i);
}

static void pwm_update_all(RP2350PWMState *s)
{
    int i;

    for (i = 0; i < RP2350_PWM_SLICES; i++) {
        pwm_update(s, i);
    }
}

static void pwm_timer_cb(void *opaque)
{
    RP2350PWMSlice *sl = opaque;
    RP2350PWMState *s = sl->pwm;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    pwm_sync(s, sl->index, pwm_cycle_at(s, now));
    if (sl->out_due_ns <= now) {
        sl->last_out_ns = now;
    }
    pwm_update(s, sl->index);
}

static void pwm_set_csr(RP2350PWMState *s, int i, uint32_t csr)
{
    RP2350PWMSlice *sl = &s->slice[i];
    uint32_t old = sl->csr;

    if ((csr & CSR_EN) && !(old & CSR_EN)) {
        /*
         * The divider starts afresh, so slices enabled together run in
         * lockstep.
         */
        sl->acc = 0;
    }
    if (!(csr & CSR_EN)) {
        pwm_latch(sl);
    }
    if (!(csr & CSR_PH_CORRECT)) {
        sl->down = false;
    }
    if (((csr ^ old) & CSR_DIVMODE) &&
        (csr & CSR_DIVMODE) >> CSR_DIVMODE_SHIFT == DIVMODE_DIV) {
        sl->b_seen = sl->b_in;
    }
    sl->csr = csr;
    pwm_try_advance(s, i);
}

static void pwm_b_in(void *opaque, int n, int level)
{
    RP2350PWMState *s = opaque;
    RP2350PWMSlice *sl = &s->slice[n];
    bool b = level != 0;
    uint64_t cycle;

    if (b == sl->b_in) {
        return;
    }
    if (pwm_divmode(sl) == DIVMODE_DIV) {
        /* B is not an input: nothing waits on the synchroniser. */
        sl->b_in = sl->b_seen = b;
        return;
    }
    cycle = pwm_now_cycle(s);
    pwm_sync(s, n, cycle);
    sl->b_in = b;
    sl->b_change = cycle;
    pwm_update(s, n);
}

/* [spec:nuos:req:emu.pwm] */
static uint64_t rp2350_pwm_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350PWMState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v = 0;
    int i;

    if (reg < A_EN) {
        RP2350PWMSlice *sl;

        i = reg / SLICE_STRIDE;
        sl = &s->slice[i];
        pwm_sync(s, i, pwm_now_cycle(s));
        switch (reg % SLICE_STRIDE) {
        case A_CSR:
            v = sl->csr;
            break;
        case A_DIV:
            v = sl->div;
            break;
        case A_CTR:
            v = sl->ctr;
            break;
        case A_CC:
            v = sl->cc;
            break;
        case A_TOP:
            v = sl->top;
            break;
        }
        pwm_update(s, i);
        return v;
    }

    pwm_sync_all(s);
    switch (reg) {
    case A_EN:
        for (i = 0; i < RP2350_PWM_SLICES; i++) {
            v |= (s->slice[i].csr & CSR_EN) << i;
        }
        break;
    case A_INTR:
        v = s->intr;
        break;
    case A_IRQ0_INTE:
    case A_IRQ1_INTE:
        v = s->inte[reg == A_IRQ1_INTE];
        break;
    case A_IRQ0_INTF:
    case A_IRQ1_INTF:
        v = s->intf[reg == A_IRQ1_INTF];
        break;
    case A_IRQ0_INTS:
    case A_IRQ1_INTS:
        i = reg == A_IRQ1_INTS;
        v = (s->intr & s->inte[i]) | s->intf[i];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of bad offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    pwm_update_all(s);
    return v;
}

/* [spec:nuos:req:emu.pwm] */
static void rp2350_pwm_write(void *opaque, hwaddr addr, uint64_t value64,
                             unsigned size)
{
    RP2350PWMState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t value = value64;
    int i;

    if (reg < A_EN) {
        RP2350PWMSlice *sl;
        uint32_t v;

        i = reg / SLICE_STRIDE;
        sl = &s->slice[i];
        pwm_sync(s, i, pwm_now_cycle(s));
        switch (reg % SLICE_STRIDE) {
        case A_CSR:
            /*
             * Writing 1 to PH_ADV or PH_RET requests a one-count
             * adjustment; the bit reads 1 until the slice has made it.
             */
            v = rp2350_atomic_apply(addr, sl->csr, value);
            pwm_set_csr(s, i, (v & (CSR_RW | CSR_PHASE)) |
                              (sl->csr & CSR_PHASE));
            break;
        case A_DIV:
            sl->div = rp2350_atomic_apply(addr, sl->div, value) & DIV_MASK;
            if (!DIV_INT(sl->div) && DIV_FRAC(sl->div)) {
                qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pwm: slice %d: "
                              "DIV_FRAC set with DIV_INT 0\n", i);
            }
            sl->acc = MIN(sl->acc, pwm_div16(sl) - 1);
            pwm_try_advance(s, i);
            break;
        case A_CTR:
            sl->ctr = rp2350_atomic_apply(addr, sl->ctr, value) & CTR_MASK;
            break;
        case A_CC:
            sl->cc = rp2350_atomic_apply(addr, sl->cc, value);
            if (!(sl->csr & CSR_EN)) {
                pwm_latch(sl);
            }
            break;
        case A_TOP:
            sl->top = rp2350_atomic_apply(addr, sl->top, value) & CTR_MASK;
            if (!(sl->csr & CSR_EN)) {
                pwm_latch(sl);
            }
            break;
        }
        pwm_update(s, i);
        return;
    }

    pwm_sync_all(s);
    switch (reg) {
    case A_EN: {
        uint32_t en = 0;

        for (i = 0; i < RP2350_PWM_SLICES; i++) {
            en |= (s->slice[i].csr & CSR_EN) << i;
        }
        en = rp2350_atomic_apply(addr, en, value);
        for (i = 0; i < RP2350_PWM_SLICES; i++) {
            pwm_set_csr(s, i, (s->slice[i].csr & ~CSR_EN) |
                              ((en >> i) & CSR_EN));
        }
        break;
    }
    case A_INTR:
        /*
         * Writing 1 clears a flag. The atomic aliases act per written
         * bit: a SET alias write writes 1s, so it clears; CLR and XOR
         * alias writes never write a 1 to a set bit.
         */
        switch (addr / RP2350_ATOMIC_ALIAS_SIZE) {
        case 0:
        case 2:
            s->intr &= ~value;
            break;
        default:
            break;
        }
        break;
    case A_IRQ0_INTE:
    case A_IRQ1_INTE:
        i = reg == A_IRQ1_INTE;
        s->inte[i] = rp2350_atomic_apply(addr, s->inte[i], value) &
                     SLICES_MASK;
        break;
    case A_IRQ0_INTF:
    case A_IRQ1_INTF:
        i = reg == A_IRQ1_INTF;
        s->intf[i] = rp2350_atomic_apply(addr, s->intf[i], value) &
                     SLICES_MASK;
        break;
    case A_IRQ0_INTS:
    case A_IRQ1_INTS:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to bad offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    pwm_update_all(s);
}

static const MemoryRegionOps rp2350_pwm_ops = {
    .read = rp2350_pwm_read,
    .write = rp2350_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * A change of clk_sys frequency: run every slice to the change at the old
 * rate, then count from there at the new one.
 */
static void rp2350_pwm_clk_update(void *opaque, ClockEvent event)
{
    RP2350PWMState *s = opaque;

    if (event == ClockPreUpdate) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t cycle = pwm_cycle_at(s, now);
        int i;

        for (i = 0; i < RP2350_PWM_SLICES; i++) {
            pwm_sync(s, i, cycle);
        }
        s->base_ns = now;
        s->base_cycle = cycle;
    } else {
        pwm_update_all(s);
    }
}

static void rp2350_pwm_hold_reset(Object *obj, ResetType type)
{
    RP2350PWMState *s = RP2350_PWM(obj);
    int i;

    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_cycle = 0;
    s->intr = 0;
    memset(s->inte, 0, sizeof(s->inte));
    memset(s->intf, 0, sizeof(s->intf));

    for (i = 0; i < RP2350_PWM_SLICES; i++) {
        RP2350PWMSlice *sl = &s->slice[i];

        timer_del(sl->timer);
        sl->csr = 0;
        sl->div = DIV_RESET;
        sl->cc = 0;
        sl->top = TOP_RESET;
        pwm_latch(sl);
        sl->ctr = 0;
        sl->down = false;
        sl->acc = 0;
        sl->sync_cycle = 0;
        /* The B pin's level is outside the block; it is not reset. */
        sl->b_seen = sl->b_in;
        sl->b_change = 0;
        sl->dreq_pending = 0;
        sl->last_out_ns = INT64_MIN / 2;
        sl->out_due_ns = INT64_MAX;
        /* Force every line to be driven afresh. */
        memset(sl->out_level, 0xff, sizeof(sl->out_level));
        memset(sl->oe_level, 0xff, sizeof(sl->oe_level));
    }
    for (i = 0; i < RP2350_PWM_SLICES; i++) {
        pwm_drive(s, i);
    }
    pwm_update_irq(s);
}

static void rp2350_pwm_init(Object *obj)
{
    RP2350PWMState *s = RP2350_PWM(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &rp2350_pwm_ops, s, TYPE_RP2350_PWM,
                          RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (i = 0; i < RP2350_PWM_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_out_named(dev, s->out, RP2350_PWM_OUT,
                             RP2350_PWM_SLICES * 2);
    qdev_init_gpio_out_named(dev, s->oe, RP2350_PWM_OE,
                             RP2350_PWM_SLICES * 2);
    qdev_init_gpio_out_named(dev, s->dreq, RP2350_PWM_DREQ,
                             RP2350_PWM_SLICES);
    qdev_init_gpio_in_named(dev, pwm_b_in, RP2350_PWM_B_IN,
                            RP2350_PWM_SLICES);
    s->clk = qdev_init_clock_in(dev, "clk", rp2350_pwm_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    for (i = 0; i < RP2350_PWM_SLICES; i++) {
        s->slice[i].pwm = s;
        s->slice[i].index = i;
    }
}

static void rp2350_pwm_realize(DeviceState *dev, Error **errp)
{
    RP2350PWMState *s = RP2350_PWM(dev);
    int i;

    for (i = 0; i < RP2350_PWM_SLICES; i++) {
        s->slice[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pwm_timer_cb,
                                         &s->slice[i]);
    }
}

static const Property rp2350_pwm_properties[] = {
    DEFINE_PROP_UINT32("min-edge-ns", RP2350PWMState, min_edge_ns, 1000),
};

static const VMStateDescription vmstate_rp2350_pwm_slice = {
    .name = TYPE_RP2350_PWM "-slice",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, RP2350PWMSlice),
        VMSTATE_UINT32(csr, RP2350PWMSlice),
        VMSTATE_UINT32(div, RP2350PWMSlice),
        VMSTATE_UINT32(cc, RP2350PWMSlice),
        VMSTATE_UINT32(top, RP2350PWMSlice),
        VMSTATE_UINT32(cc_latched, RP2350PWMSlice),
        VMSTATE_UINT32(top_latched, RP2350PWMSlice),
        VMSTATE_UINT32(ctr, RP2350PWMSlice),
        VMSTATE_BOOL(down, RP2350PWMSlice),
        VMSTATE_UINT32(acc, RP2350PWMSlice),
        VMSTATE_UINT64(sync_cycle, RP2350PWMSlice),
        VMSTATE_BOOL(b_in, RP2350PWMSlice),
        VMSTATE_BOOL(b_seen, RP2350PWMSlice),
        VMSTATE_UINT64(b_change, RP2350PWMSlice),
        VMSTATE_UINT32(dreq_pending, RP2350PWMSlice),
        VMSTATE_INT64(last_out_ns, RP2350PWMSlice),
        VMSTATE_INT64(out_due_ns, RP2350PWMSlice),
        VMSTATE_UINT8_ARRAY(out_level, RP2350PWMSlice, 2),
        VMSTATE_UINT8_ARRAY(oe_level, RP2350PWMSlice, 2),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_pwm = {
    .name = TYPE_RP2350_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, RP2350PWMState),
        VMSTATE_INT64(base_ns, RP2350PWMState),
        VMSTATE_UINT64(base_cycle, RP2350PWMState),
        VMSTATE_STRUCT_ARRAY(slice, RP2350PWMState, RP2350_PWM_SLICES, 1,
                             vmstate_rp2350_pwm_slice, RP2350PWMSlice),
        VMSTATE_UINT32(intr, RP2350PWMState),
        VMSTATE_UINT32_ARRAY(inte, RP2350PWMState, RP2350_PWM_IRQS),
        VMSTATE_UINT32_ARRAY(intf, RP2350PWMState, RP2350_PWM_IRQS),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_pwm_realize;
    rc->phases.hold = rp2350_pwm_hold_reset;
    dc->vmsd = &vmstate_rp2350_pwm;
    device_class_set_props(dc, rp2350_pwm_properties);
}

/* [spec:nuos:req:emu.pwm] */
static const TypeInfo rp2350_pwm_info = {
    .name          = TYPE_RP2350_PWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350PWMState),
    .instance_init = rp2350_pwm_init,
    .class_init    = rp2350_pwm_class_init,
};

static void rp2350_pwm_register_types(void)
{
    type_register_static(&rp2350_pwm_info);
}

type_init(rp2350_pwm_register_types)
