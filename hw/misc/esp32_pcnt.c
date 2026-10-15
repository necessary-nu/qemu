/*
 * ESP32 pulse count controller (PCNT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Pulse Count Controller (PCNT)", with the field
 * layout and reset values of ESP-IDF's soc/esp32 pcnt_reg.h and
 * pcnt_struct.h.
 *
 * Eight units each keep a 16-bit signed counter driven by two channels.
 * A channel counts the edges of its signal input, up, down or not at all
 * per edge direction (POS_MODE, NEG_MODE), with the action kept, inverted
 * or inhibited by the level of its control input (HCTRL_MODE,
 * LCTRL_MODE). All four inputs of a unit pass through the unit's glitch
 * filter, clocked by APB_CLK.
 *
 * Five comparators watch each counter as it counts: the two thresholds
 * and zero match on equality, the high limit on count >= H_LIM and the
 * low limit on count <= L_LIM, and a limit match also returns the counter
 * to 0. Comparators are evaluated on a step of the counter only, not when
 * software resets it or changes a comparator's value. A step that matches
 * any enabled comparator raises the unit's interrupt and latches which
 * comparators matched in the unit's STATUS register, replacing the
 * previous latch.
 *
 * Times are those of the signal lines (hw/gpio/esp32_line.h): an input
 * changes at the time its change is stamped with, and a register access,
 * the filter's timer or a change of APB_CLK first brings every source of
 * line changes up to the moment it acts at, so that a source sending late
 * (the host running behind virtual time without icount) neither shortens
 * a pulse in the filter's eyes nor adds a count that a later change would
 * have filtered.
 *
 * A signal input reached through the GPIO matrix by a periodic output
 * that describes its edges (a PWM output, hw/gpio/esp32_line.h) takes
 * them from the output's wave rather than its line, which the output then
 * need not drive edge by edge: the edges since the input was last brought
 * up to a moment are worked out when it is next, in time order with the
 * other inputs' changes, each at its own time. While every pulse of the
 * wave outlasts the glitch filter, the count they leave, and the first of
 * them at which a watch point matches, follow in closed form
 * (hw/misc/esp32_pcnt_bulk.h): the counter steps through matches one at a
 * time, and between them takes any number of edges at once; a watch
 * point's interrupt, enabled, has the timer set for its edge. Otherwise
 * the wave's edges are taken one by one, as the line's would be.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32_pcnt.h"
#include "hw/misc/esp32_pcnt_bulk.h"
#include "migration/vmstate.h"

#define A_UNIT_CONF0(u)         (0x00 + 0x0c * (u))
#define A_UNIT_CNT(u)           (0x60 + 0x04 * (u))
#define A_INT_RAW               0x80
#define A_INT_ST                0x84
#define A_INT_ENA               0x88
#define A_INT_CLR               0x8c
#define A_UNIT_STATUS(u)        (0x90 + 0x04 * (u))
#define A_CTRL                  0xb0
#define A_DATE                  0xfc

#define CONF0_FILTER_THRES      0x3ffu
#define CONF0_FILTER_EN         (1u << 10)
#define CONF0_THR_ZERO_EN       (1u << 11)
#define CONF0_THR_H_LIM_EN      (1u << 12)
#define CONF0_THR_L_LIM_EN      (1u << 13)
#define CONF0_THR_THRES0_EN     (1u << 14)
#define CONF0_THR_THRES1_EN     (1u << 15)
/* Channel ch's NEG_MODE, POS_MODE, HCTRL_MODE and LCTRL_MODE, 2 bits each */
#define CONF0_CH_SHIFT(ch)      (16 + 8 * (ch))
#define CONF0_RESET             0x00003c10u

#define STATUS_ZERO_MODE        0x3u
#define STATUS_THRES1_LAT       (1u << 2)
#define STATUS_THRES0_LAT       (1u << 3)
#define STATUS_L_LIM_LAT        (1u << 4)
#define STATUS_H_LIM_LAT        (1u << 5)
#define STATUS_ZERO_LAT         (1u << 6)

/* ZERO_MODE: how the counter last stood relative to zero */
#define ZERO_MODE_POS_ZERO      0
#define ZERO_MODE_NEG_ZERO      1
#define ZERO_MODE_NEG           2
#define ZERO_MODE_POS           3

#define CTRL_CNT_RST(u)         (1u << (2 * (u)))
#define CTRL_CNT_PAUSE(u)       (1u << (2 * (u) + 1))
#define CTRL_CLK_EN             (1u << 16)
#define CTRL_MASK               0x1ffffu
#define CTRL_RESET              0x5555u

#define INT_MASK                0xffu
#define DATE_RESET              0x14122600u

/* Inputs within a unit */
#define IN_SIG(ch)              (ch)
#define IN_CTRL(ch)             (2 + (ch))

/* Edge modes and control modes */
#define EDGE_INC                1
#define EDGE_DEC                2
#define CTRL_KEEP               0
#define CTRL_INVERT             1

static void pcnt_update_irq(Esp32PcntState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

/* STATUS's zero mode after a step of the counter from old to cnt. */
static uint32_t pcnt_zero_mode(int16_t old, int16_t cnt)
{
    if (cnt == 0) {
        return old > 0 ? ZERO_MODE_POS_ZERO : ZERO_MODE_NEG_ZERO;
    }
    return cnt < 0 ? ZERO_MODE_NEG : ZERO_MODE_POS;
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Step unit u's counter by delta and run its comparators on the new value.
 */
static void pcnt_step(Esp32PcntState *s, unsigned u, int delta)
{
    Esp32PcntUnit *un = &s->unit[u];
    int16_t old = un->cnt;
    int16_t cnt = (int16_t)(uint16_t)(old + delta);
    int16_t thres0 = (int16_t)extract32(un->conf1, 0, 16);
    int16_t thres1 = (int16_t)extract32(un->conf1, 16, 16);
    int16_t h_lim = (int16_t)extract32(un->conf2, 0, 16);
    int16_t l_lim = (int16_t)extract32(un->conf2, 16, 16);
    uint32_t mode;
    uint32_t lat = 0;

    un->cnt = cnt;
    mode = pcnt_zero_mode(old, cnt);

    if ((un->conf0 & CONF0_THR_THRES1_EN) && cnt == thres1) {
        lat |= STATUS_THRES1_LAT;
    }
    if ((un->conf0 & CONF0_THR_THRES0_EN) && cnt == thres0) {
        lat |= STATUS_THRES0_LAT;
    }
    if ((un->conf0 & CONF0_THR_L_LIM_EN) && cnt <= l_lim) {
        lat |= STATUS_L_LIM_LAT;
    }
    if ((un->conf0 & CONF0_THR_H_LIM_EN) && cnt >= h_lim) {
        lat |= STATUS_H_LIM_LAT;
    }
    if ((un->conf0 & CONF0_THR_ZERO_EN) && cnt == 0) {
        lat |= STATUS_ZERO_LAT;
    }

    /* A limit returns the counter to zero from the side it was reached */
    if (lat & STATUS_H_LIM_LAT) {
        un->cnt = 0;
        mode = ZERO_MODE_POS_ZERO;
    } else if (lat & STATUS_L_LIM_LAT) {
        un->cnt = 0;
        mode = ZERO_MODE_NEG_ZERO;
    }

    if (lat) {
        un->status = lat | mode;
        s->int_raw |= 1u << u;
        pcnt_update_irq(s);
    } else {
        un->status = (un->status & ~STATUS_ZERO_MODE) | mode;
    }
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * The step a filtered edge on unit u's input k takes the counter, 0 for
 * none. Only signal inputs count; a control input's level is consulted
 * when its channel's signal moves.
 */
static int pcnt_delta(Esp32PcntState *s, unsigned u, unsigned k, bool rising)
{
    uint32_t conf0 = s->unit[u].conf0;
    unsigned ch, edge, lvl;
    int delta;

    if (k >= IN_CTRL(0)) {
        return 0;
    }
    if (s->ctrl & (CTRL_CNT_RST(u) | CTRL_CNT_PAUSE(u))) {
        return 0;
    }
    ch = k;
    edge = extract32(conf0, CONF0_CH_SHIFT(ch) + (rising ? 2 : 0), 2);
    if (s->filt[u * ESP32_PCNT_UNIT_INPUTS + IN_CTRL(ch)]) {
        lvl = extract32(conf0, CONF0_CH_SHIFT(ch) + 4, 2);
    } else {
        lvl = extract32(conf0, CONF0_CH_SHIFT(ch) + 6, 2);
    }

    switch (edge) {
    case EDGE_INC:
        delta = 1;
        break;
    case EDGE_DEC:
        delta = -1;
        break;
    default:
        return 0;
    }
    switch (lvl) {
    case CTRL_KEEP:
        break;
    case CTRL_INVERT:
        delta = -delta;
        break;
    default:
        return 0;
    }
    return delta;
}

/* [spec:nuos:req:emu.esp32.pcnt] A filtered edge on input k. */
static void pcnt_edge(Esp32PcntState *s, unsigned u, unsigned k, bool rising)
{
    int delta = pcnt_delta(s, u, k, rising);

    if (delta) {
        pcnt_step(s, u, delta);
    }
}

/* How long a change takes past input i's glitch filter. */
static int64_t pcnt_filter_delay(Esp32PcntState *s, unsigned i)
{
    uint32_t conf0 = s->unit[i / ESP32_PCNT_UNIT_INPUTS].conf0;

    if (!(conf0 & CONF0_FILTER_EN)) {
        return 0;
    }
    return clock_ticks_to_ns(s->apb_clk, conf0 & CONF0_FILTER_THRES);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * The glitch filter: with FILTER_EN, a level change on an input reaches
 * the counter once it has held for FILTER_THRES APB_CLK cycles; a pulse
 * shorter than that is lost. Whether a change got through is decided from
 * the virtual time stamps of the wire's changes, not from when the
 * filter's timer happens to run, so a late timer cannot lose an edge or
 * reorder edges between inputs. Returns whether input i has a change
 * waiting, and when it matures.
 */
static bool pcnt_pending(Esp32PcntState *s, unsigned i, int64_t *due)
{
    uint32_t conf0 = s->unit[i / ESP32_PCNT_UNIT_INPUTS].conf0;
    uint32_t thres = conf0 & CONF0_FILTER_THRES;

    if (!s->clk_running || s->raw[i] == s->filt[i]) {
        return false;
    }
    *due = s->raw_since_ns[i];
    if (conf0 & CONF0_FILTER_EN) {
        *due += clock_ticks_to_ns(s->apb_clk, thres);
    }
    return true;
}

/* Inputs taking a wave in bulk */

#define UI                  ESP32_PCNT_UNIT_INPUTS

static bool pcnt_is_sig(unsigned i)
{
    return i % UI == IN_SIG(0) || i % UI == IN_SIG(1);
}

static int64_t pcnt_feed_due(void *opaque, int64_t e)
{
    Esp32PcntBulk *b = opaque;

    return e + pcnt_filter_delay(b->s, b->input);
}

/* Input i's state, taking its wave, as a feed (esp32_pcnt_bulk.h). */
static Esp32PcntFeed pcnt_feed(Esp32PcntState *s, unsigned i)
{
    uint32_t conf0 = s->unit[i / UI].conf0;
    Esp32PcntFeed f = {
        .w = s->bulk[i].wave,
        .inv = s->bulk[i].inv,
        .pos = s->bulk_ns[i],
        .raw = s->raw[i],
        .filt = s->filt[i],
        .raw_since = s->raw_since_ns[i],
        .filter = (conf0 & CONF0_FILTER_EN) != 0,
        .dmax = pcnt_filter_delay(s, i),
        .due = pcnt_feed_due,
        .opaque = &s->bulk[i],
    };

    return f;
}

static void pcnt_feed_store(Esp32PcntState *s, unsigned i,
                            const Esp32PcntFeed *f)
{
    s->bulk_ns[i] = f->pos;
    s->raw[i] = f->raw;
    s->filt[i] = f->filt;
    s->raw_since_ns[i] = f->raw_since;
}

/*
 * The moment input i takes its wave up to, for a catch-up to t: no later
 * than the description holds; its output announces the change there.
 */
static int64_t pcnt_bulk_limit(Esp32PcntState *s, unsigned i, int64_t t)
{
    Esp32LineWave *w = s->bulk[i].wave;
    int64_t until = w->ops->until(w);

    return until == INT64_MAX ? t : MIN(t, until - 1);
}

/* Unit u's watch points, as the counter's next steps meet them. */
static unsigned pcnt_cmps(Esp32PcntState *s, unsigned u, Esp32PcntCmp *c)
{
    Esp32PcntUnit *un = &s->unit[u];
    unsigned n = 0;

    if (un->conf0 & CONF0_THR_THRES1_EN) {
        c[n++] = (Esp32PcntCmp) { ESP32_PCNT_CMP_EQ,
                                  (int16_t)extract32(un->conf1, 16, 16) };
    }
    if (un->conf0 & CONF0_THR_THRES0_EN) {
        c[n++] = (Esp32PcntCmp) { ESP32_PCNT_CMP_EQ,
                                  (int16_t)extract32(un->conf1, 0, 16) };
    }
    if (un->conf0 & CONF0_THR_L_LIM_EN) {
        c[n++] = (Esp32PcntCmp) { ESP32_PCNT_CMP_LE,
                                  (int16_t)extract32(un->conf2, 16, 16) };
    }
    if (un->conf0 & CONF0_THR_H_LIM_EN) {
        c[n++] = (Esp32PcntCmp) { ESP32_PCNT_CMP_GE,
                                  (int16_t)extract32(un->conf2, 0, 16) };
    }
    if (un->conf0 & CONF0_THR_ZERO_EN) {
        c[n++] = (Esp32PcntCmp) { ESP32_PCNT_CMP_EQ, 0 };
    }
    return n;
}

/*
 * The signal input of unit u whose filtered edges the unit takes in
 * closed form, into *f: one taking its wave, every pulse passing the
 * filter, a change waiting in the filter due before the next edge; -1 if
 * neither is. The other's, as any other input's, come one by one.
 */
static int pcnt_bulk_fast(Esp32PcntState *s, unsigned u, Esp32PcntFeed *f)
{
    if (!s->clk_running) {
        return -1;
    }
    for (unsigned ch = 0; ch < 2; ch++) {
        unsigned i = u * UI + IN_SIG(ch);

        if (!s->bulk[i].on) {
            continue;
        }
        *f = pcnt_feed(s, i);
        if (esp32_pcnt_feed_passes(f) && esp32_pcnt_feed_aligned(f)) {
            return i;
        }
    }
    return -1;
}

/* The steps the next filtered edge of f, and the one after, take. */
static void pcnt_bulk_steps(Esp32PcntState *s, unsigned i,
                            const Esp32PcntFeed *f, int *dn, int *da)
{
    /* The next filtered edge takes the level to !filt. */
    bool rising = !f->filt;

    *dn = pcnt_delta(s, i / UI, i % UI, rising);
    *da = pcnt_delta(s, i / UI, i % UI, !rising);
}

/* Take k filtered edges at once, none at a watch point. */
static void pcnt_bulk_count(Esp32PcntState *s, unsigned u, int dn, int da,
                            uint64_t k)
{
    Esp32PcntUnit *un = &s->unit[u];
    int16_t last, before;

    if (esp32_pcnt_count_edges(un->cnt, dn, da, k, &last, &before)) {
        un->cnt = last;
        un->status = (un->status & ~STATUS_ZERO_MODE) |
                     pcnt_zero_mode(before, last);
    }
}

/*
 * The time of the first filtered edge of the fast input i (f) at which a
 * watch point matches, its index into *m; INT64_MAX if none by limit.
 */
static int64_t pcnt_bulk_match(Esp32PcntState *s, unsigned i,
                               const Esp32PcntFeed *f, int64_t limit,
                               uint64_t *m)
{
    Esp32PcntCmp cmp[ESP32_PCNT_CMPS];
    unsigned n = pcnt_cmps(s, i / UI, cmp);
    int dn, da;
    int64_t t;

    pcnt_bulk_steps(s, i, f, &dn, &da);
    *m = esp32_pcnt_first_match(s->unit[i / UI].cnt, dn, da, cmp, n);
    if (*m == UINT64_MAX) {
        return INT64_MAX;
    }
    t = esp32_pcnt_feed_time(f, *m);
    return t > limit ? INT64_MAX : t;
}

/* Bring the fast input i (f) to x, taking its filtered edges by then. */
static void pcnt_bulk_advance(Esp32PcntState *s, unsigned i,
                              Esp32PcntFeed *f, int64_t x)
{
    int dn, da;

    if (x <= f->pos) {
        return;
    }
    pcnt_bulk_steps(s, i, f, &dn, &da);
    pcnt_bulk_count(s, i / UI, dn, da, esp32_pcnt_feed_count(f, x));
    esp32_pcnt_feed_advance(f, x);
    pcnt_feed_store(s, i, f);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Bring unit u up to time t: every change of its inputs that has matured
 * by then passed through the filter, oldest first, and its inputs taking
 * a wave given the wave's edges by then, in time order with the rest. At
 * one moment, changes maturing go before an edge reaching an input, and
 * an input with a lower number before one with a higher, as an input's
 * line would have them.
 */
static void pcnt_sync_unit(Esp32PcntState *s, unsigned u, int64_t t)
{
    for (;;) {
        Esp32PcntFeed fa;
        int a = pcnt_bulk_fast(s, u, &fa);
        int mat = -1, edge_in = -1;
        int64_t tm = INT64_MAX, te = INT64_MAX, ta = INT64_MAX, y, due;
        int64_t la = t;
        uint64_t ma = 0;

        for (unsigned k = 0; k < UI; k++) {
            unsigned i = u * UI + k;

            if ((int)i == a) {
                continue;
            }
            if (pcnt_pending(s, i, &due) && due < tm) {
                tm = due;
                mat = i;
            }
            if (s->bulk[i].on && s->clk_running) {
                Esp32PcntFeed f = pcnt_feed(s, i);
                int64_t e = esp32_pcnt_feed_next_raw(&f);

                if (e <= pcnt_bulk_limit(s, i, t) && e < te) {
                    te = e;
                    edge_in = i;
                }
            }
        }
        if (a >= 0) {
            la = pcnt_bulk_limit(s, a, t);
            ta = pcnt_bulk_match(s, a, &fa, la, &ma);
        }
        y = MIN(MIN(tm, te), ta);
        if (y > t) {
            if (a >= 0) {
                pcnt_bulk_advance(s, a, &fa, la);
            }
            break;
        }
        if (a >= 0) {
            if (ta == y && (tm > y || a < mat)) {
                /* The edges before the watch point's, then its own */
                pcnt_bulk_advance(s, a, &fa, ta - 1);
                fa = pcnt_feed(s, a);
                esp32_pcnt_feed_advance(&fa, ta);
                pcnt_feed_store(s, a, &fa);
                pcnt_edge(s, u, a % UI, s->filt[a]);
                continue;
            }
            pcnt_bulk_advance(s, a, &fa, tm == y && mat < a ? y - 1 : y);
        }
        if (tm <= te) {
            s->filt[mat] = s->raw[mat];
            pcnt_edge(s, u, mat % UI, s->filt[mat]);
        } else {
            Esp32PcntFeed f = pcnt_feed(s, edge_in);

            esp32_pcnt_feed_raw_edge(&f, te);
            pcnt_feed_store(s, edge_in, &f);
        }
    }
    for (unsigned ch = 0; ch < 2; ch++) {
        unsigned i = u * UI + IN_SIG(ch);
        int64_t lim;

        if (!s->bulk[i].on) {
            continue;
        }
        lim = pcnt_bulk_limit(s, i, t);
        if (!s->clk_running && lim > s->bulk_ns[i]) {
            /* The filter's flops hold: only the level moves. */
            Esp32LineWave *w = s->bulk[i].wave;
            uint64_t nr = w->ops->edges(w, s->bulk_ns[i], lim);

            if (nr) {
                s->raw_since_ns[i] = w->ops->edge(w, s->bulk_ns[i], nr);
            }
            s->raw[i] = w->ops->level(w, lim) ^ s->bulk[i].inv;
        }
        s->bulk_ns[i] = MAX(s->bulk_ns[i], lim);
    }
}

/* Bring every unit up to time t (pcnt_sync_unit()). */
static void pcnt_sync(Esp32PcntState *s, int64_t t)
{
    for (unsigned u = 0; u < ESP32_PCNT_UNIT_COUNT; u++) {
        pcnt_sync_unit(s, u, t);
    }
}

/*
 * Arm the timer for the next change to mature, and for the next watch
 * point an input taking its wave reaches in a unit whose interrupt is
 * enabled and not yet raised; the rest is taken when software looks.
 */
static void pcnt_schedule(Esp32PcntState *s)
{
    int64_t next = INT64_MAX, due;

    for (unsigned u = 0; u < ESP32_PCNT_UNIT_COUNT; u++) {
        bool armed = s->int_ena & ~s->int_raw & (1u << u);
        Esp32PcntFeed fa;
        int a = pcnt_bulk_fast(s, u, &fa);

        for (unsigned k = 0; k < UI; k++) {
            unsigned i = u * UI + k;

            if ((int)i == a) {
                continue;
            }
            if (s->bulk[i].on) {
                if (armed && pcnt_pending(s, i, &due)) {
                    next = MIN(next, due);
                }
                if (armed && s->clk_running) {
                    Esp32PcntFeed f = pcnt_feed(s, i);

                    next = MIN(next, esp32_pcnt_feed_next_raw(&f));
                }
                continue;
            }
            if (pcnt_pending(s, i, &due)) {
                next = MIN(next, due);
            }
        }
        if (a >= 0 && armed) {
            uint64_t m;

            next = MIN(next, pcnt_bulk_match(s, a, &fa,
                                             pcnt_bulk_limit(s, a, INT64_MAX),
                                             &m));
        }
    }
    if (next == INT64_MAX) {
        timer_del(&s->filter_timer);
    } else {
        timer_mod_ns(&s->filter_timer, next);
    }
}

static void pcnt_sync_sources(void *opaque)
{
    esp32_line_sync();
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Bring every source of line changes, then the filters, up to now
 * (esp32_line_time()), and return that moment: a change a source has yet
 * to send can still cut short a pulse waiting in a filter.
 */
static int64_t pcnt_catch_up(Esp32PcntState *s)
{
    int64_t t = esp32_line_time();

    esp32_line_run_at(t, pcnt_sync_sources, NULL);
    pcnt_sync(s, t);
    return t;
}

static void pcnt_filter_cb(void *opaque)
{
    pcnt_catch_up(opaque);
    pcnt_schedule(opaque);
}

/* Input n changes to level at t, as its line or a wave says. */
static void pcnt_input_change(Esp32PcntState *s, int n, bool level,
                              int64_t t)
{
    /* A change already held long enough happened before this one */
    pcnt_sync(s, t);
    s->raw[n] = level;
    s->raw_since_ns[n] = t;
    pcnt_sync(s, t);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * An input changes at its stamp's time. A signal input taking a wave
 * ignores its line, which the output drives only at its level of the
 * moment as software samples the pads, keeping its level for when it
 * follows the line again.
 */
static void pcnt_set_input(void *opaque, int n, int level)
{
    Esp32PcntState *s = opaque;
    int64_t t = esp32_line_time();

    level = !!level;
    s->line[n] = level;
    if (s->bulk[n].on || s->raw[n] == level) {
        return;
    }
    s->line_ns[n] = t;
    pcnt_input_change(s, n, level, t);
    pcnt_schedule(s);
}

/*
 * Signal input i, taking its wave, goes on from the moment `from`: the
 * wave's edges after it, and its level there, a change at `from` if the
 * input had another (an output whose settings changed there can change
 * its level at once).
 */
static void pcnt_bulk_join(Esp32PcntState *s, unsigned i, int64_t from)
{
    Esp32PcntBulk *b = &s->bulk[i];
    bool level;

    from = MAX(from, s->bulk_ns[i]);
    s->bulk_ns[i] = from;
    level = b->wave->ops->level(b->wave, from) ^ b->inv;
    if (level != s->raw[i]) {
        pcnt_input_change(s, i, level, from);
    }
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Whether signal input i takes its wave, from the moment `from` on: as
 * its output describes its edges. One that does takes the wave's level
 * there if its line had another; one that stops follows its line from
 * then, taking the line's level.
 */
static void pcnt_bulk_switch(Esp32PcntState *s, unsigned i, int64_t from)
{
    Esp32PcntBulk *b = &s->bulk[i];
    bool on = b->wave && b->wave->ops->describes(b->wave);

    if (on == b->on) {
        return;
    }
    if (!on) {
        b->on = false;
        from = MAX(from, s->bulk_ns[i]);
        if (s->line[i] != s->raw[i]) {
            s->line_ns[i] = from;
            pcnt_input_change(s, i, s->line[i], from);
        }
        return;
    }
    from = MAX(from, s->line_ns[i]);
    pcnt_sync(s, from);
    b->on = true;
    s->bulk_ns[i] = from;
    pcnt_bulk_join(s, i, from);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * A signal input's output announcing a change at c->when: before it, the
 * unit takes its edges until then; after, the input goes on from its new
 * description, its level at c->when as the change left it, or follows its
 * line if it describes its edges no more.
 */
static void pcnt_wave_changed(Notifier *n, void *data)
{
    Esp32PcntBulk *b = container_of(n, Esp32PcntBulk, taker);
    Esp32PcntState *s = b->s;
    Esp32LineWaveChange *c = data;

    if (!c->done) {
        if (b->on) {
            pcnt_sync(s, c->when - 1);
        }
        return;
    }
    if (b->on) {
        pcnt_bulk_switch(s, b->input, c->when);
        if (b->on) {
            pcnt_bulk_join(s, b->input, c->when);
        }
    } else {
        pcnt_bulk_switch(s, b->input, c->when);
    }
    pcnt_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * [spec:nuos:req:emu.esp32.gpio]
 * The matrix's routing changed: each signal input takes the wave it now
 * connects it to (esp32_sig_in_wave()), from the present on, having taken
 * the old one's edges up to it.
 */
static void pcnt_routes_changed(Notifier *notifier, void *data)
{
    Esp32PcntState *s = container_of(notifier, Esp32PcntState,
                                     route_notifier);
    int64_t now = esp32_line_time();

    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        Esp32PcntBulk *b = &s->bulk[i];
        Esp32LineWave *w;
        bool inv = false;

        if (!pcnt_is_sig(i)) {
            continue;
        }
        w = esp32_sig_in_wave(OBJECT(s->gpio),
                              ESP32_PCNT_SIG(i / UI) + i % UI, &inv);
        if (w == b->wave && inv == b->inv) {
            continue;
        }
        pcnt_sync(s, now);
        if (b->wave) {
            b->wave = NULL;
            pcnt_bulk_switch(s, i, now);
            esp32_line_wave_drop(&b->taker);
        }
        b->wave = w;
        b->inv = inv;
        if (w) {
            esp32_line_wave_take(w, &b->taker);
            pcnt_bulk_switch(s, i, now);
        }
    }
    pcnt_schedule(s);
}

static uint64_t pcnt_reg_read(Esp32PcntState *s, hwaddr addr)
{
    pcnt_catch_up(s);

    if (addr < A_UNIT_CNT(0)) {
        Esp32PcntUnit *un = &s->unit[addr / 0x0c];

        switch (addr % 0x0c) {
        case 0x0:
            return un->conf0;
        case 0x4:
            return un->conf1;
        default:
            return un->conf2;
        }
    }
    if (addr < A_INT_RAW) {
        return (uint16_t)s->unit[(addr - A_UNIT_CNT(0)) / 4].cnt;
    }
    if (addr >= A_UNIT_STATUS(0) && addr < A_CTRL) {
        return s->unit[(addr - A_UNIT_STATUS(0)) / 4].status;
    }

    switch (addr) {
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_CTRL:
        return s->ctrl;
    case A_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pcnt: read of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void pcnt_reg_write(Esp32PcntState *s, hwaddr addr, uint64_t value)
{
    /* Edges that matured before the write are counted under the old state */
    pcnt_catch_up(s);
    if (addr < A_UNIT_CNT(0)) {
        unsigned u = addr / 0x0c;
        Esp32PcntUnit *un = &s->unit[u];

        switch (addr % 0x0c) {
        case 0x0:
            un->conf0 = value;
            /* A new filter setting applies to changes already waiting */
            pcnt_sync(s, esp32_line_time());
            break;
        case 0x4:
            un->conf1 = value;
            break;
        default:
            un->conf2 = value;
            break;
        }
        return;
    }

    switch (addr) {
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        pcnt_update_irq(s);
        return;
    case A_INT_CLR:
        s->int_raw &= ~(value & INT_MASK);
        pcnt_update_irq(s);
        return;
    case A_CTRL:
        /* A unit's counter is held at zero while its CNT_RST is set */
        s->ctrl = value & CTRL_MASK;
        for (unsigned u = 0; u < ESP32_PCNT_UNIT_COUNT; u++) {
            if (s->ctrl & CTRL_CNT_RST(u)) {
                s->unit[u].cnt = 0;
                s->unit[u].status &= ~STATUS_ZERO_MODE;
            }
        }
        return;
    case A_DATE:
        s->date = value;
        return;
    default:
        break;
    }

    if (addr < A_INT_RAW || addr == A_INT_RAW || addr == A_INT_ST ||
        (addr >= A_UNIT_STATUS(0) && addr < A_CTRL)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pcnt: write to read-only register 0x%"
                      HWADDR_PRIx "\n", addr);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pcnt: write to reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
    }
}

/* A register access under way, which acts at one moment throughout. */
typedef struct PcntAccess {
    Esp32PcntState *s;
    hwaddr addr;
    uint64_t value;
} PcntAccess;

static void pcnt_access_read(void *opaque)
{
    PcntAccess *a = opaque;

    a->value = pcnt_reg_read(a->s, a->addr);
    pcnt_schedule(a->s);
}

static void pcnt_access_write(void *opaque)
{
    PcntAccess *a = opaque;

    pcnt_reg_write(a->s, a->addr, a->value);
    pcnt_schedule(a->s);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * A register access acts at the moment it began (see esp32_line.h): the
 * count it reads is the count at that moment, however long the access
 * takes without icount.
 */
static uint64_t esp32_pcnt_read(void *opaque, hwaddr addr, unsigned size)
{
    PcntAccess a = { .s = opaque, .addr = addr };

    esp32_line_run_at(esp32_line_time(), pcnt_access_read, &a);
    return a.value;
}

static void esp32_pcnt_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    PcntAccess a = { .s = opaque, .addr = addr, .value = value };

    esp32_line_run_at(esp32_line_time(), pcnt_access_write, &a);
}

static const MemoryRegionOps esp32_pcnt_ops = {
    .read = esp32_pcnt_read,
    .write = esp32_pcnt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Reset leaves the counters held by CNT_RST. The input wires are not part
 * of the block: their levels survive, and the filters start out agreeing
 * with them, so a reset does not count an edge.
 */
static void esp32_pcnt_reset_hold(Object *obj, ResetType type)
{
    Esp32PcntState *s = ESP32_PCNT(obj);

    for (unsigned u = 0; u < ESP32_PCNT_UNIT_COUNT; u++) {
        s->unit[u] = (Esp32PcntUnit) { .conf0 = CONF0_RESET };
    }
    s->int_raw = 0;
    s->int_ena = 0;
    s->ctrl = CTRL_RESET;
    s->date = DATE_RESET;
    timer_del(&s->filter_timer);
    s->clk_running = clock_is_enabled(s->apb_clk);
    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        Esp32PcntBulk *b = &s->bulk[i];

        if (b->on) {
            /* A wave's level, as its line's survives */
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

            s->raw[i] = b->wave->ops->level(b->wave, now) ^ b->inv;
            s->bulk_ns[i] = now;
        }
        s->filt[i] = s->raw[i];
    }
    pcnt_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * APB_CLK is about to change: the line sources are brought up to now and
 * the filters judge the changes matured by then at its old period
 * (pcnt_catch_up()). As it changes, at the moment that has come to, the
 * sources are brought up to that and the filters count its new period.
 * With the clock stopped the filter's flops hold their state; a change
 * waiting at the wire is filtered afresh once the clock restarts.
 */
static void esp32_pcnt_clk_update(void *opaque, ClockEvent event)
{
    Esp32PcntState *s = opaque;
    bool running = clock_is_enabled(s->apb_clk);
    int64_t now;

    if (event == ClockPreUpdate) {
        pcnt_catch_up(s);
        pcnt_schedule(s);
        return;
    }
    now = esp32_line_time();
    if (running && !s->clk_running) {
        for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
            s->raw_since_ns[i] = now;
        }
    }
    s->clk_running = running;
    esp32_line_run_at(now, pcnt_sync_sources, NULL);
    pcnt_sync(s, now);
    pcnt_schedule(s);
}

static void esp32_pcnt_init(Object *obj)
{
    Esp32PcntState *s = ESP32_PCNT(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_pcnt_ops, s,
                          TYPE_ESP32_PCNT, ESP32_PCNT_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), pcnt_set_input, ESP32_PCNT_INPUT,
                            ESP32_PCNT_INPUT_COUNT);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", esp32_pcnt_clk_update,
                                    s, ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->filter_timer, QEMU_CLOCK_VIRTUAL, pcnt_filter_cb, s);
    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        s->bulk[i].s = s;
        s->bulk[i].input = i;
        s->bulk[i].taker.notify = pcnt_wave_changed;
    }
    s->route_notifier.notify = pcnt_routes_changed;
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * The signal inputs take waves in bulk through the matrix.
 */
static void esp32_pcnt_realize(DeviceState *dev, Error **errp)
{
    Esp32PcntState *s = ESP32_PCNT(dev);

    if (!s->gpio) {
        return;
    }
    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        if (pcnt_is_sig(i)) {
            esp32_sig_set_bulk_in(OBJECT(s->gpio),
                                  ESP32_PCNT_SIG(i / UI) + i % UI, true);
        }
    }
    esp32_sig_add_route_notifier(OBJECT(s->gpio), &s->route_notifier);
}

static int esp32_pcnt_pre_save(void *opaque)
{
    Esp32PcntState *s = opaque;

    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        s->bulk_on[i] = s->bulk[i].on;
    }
    return 0;
}

/*
 * The waves the inputs take are found again from the matrix's routing as
 * it was migrated; the moments they were brought to came with the state.
 */
static int esp32_pcnt_post_load(void *opaque, int version_id)
{
    Esp32PcntState *s = opaque;

    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        Esp32PcntBulk *b = &s->bulk[i];
        bool inv = false;
        Esp32LineWave *w;

        if (!s->gpio || !pcnt_is_sig(i)) {
            continue;
        }
        w = esp32_sig_in_wave(OBJECT(s->gpio),
                              ESP32_PCNT_SIG(i / UI) + i % UI, &inv);
        if (w != b->wave) {
            if (b->wave) {
                esp32_line_wave_drop(&b->taker);
            }
            if (w) {
                esp32_line_wave_take(w, &b->taker);
            }
        }
        b->wave = w;
        b->inv = inv;
        b->on = w && s->bulk_on[i];
    }
    return 0;
}

static const Property esp32_pcnt_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32PcntState, gpio, TYPE_DEVICE,
                     DeviceState *),
};

static const VMStateDescription vmstate_esp32_pcnt_unit = {
    .name = "esp32-pcnt-unit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, Esp32PcntUnit),
        VMSTATE_UINT32(conf1, Esp32PcntUnit),
        VMSTATE_UINT32(conf2, Esp32PcntUnit),
        VMSTATE_INT16(cnt, Esp32PcntUnit),
        VMSTATE_UINT32(status, Esp32PcntUnit),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_pcnt = {
    .name = TYPE_ESP32_PCNT,
    .version_id = 2,
    .minimum_version_id = 2,
    .pre_save = esp32_pcnt_pre_save,
    .post_load = esp32_pcnt_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(unit, Esp32PcntState, ESP32_PCNT_UNIT_COUNT, 1,
                             vmstate_esp32_pcnt_unit, Esp32PcntUnit),
        VMSTATE_UINT32(int_raw, Esp32PcntState),
        VMSTATE_UINT32(int_ena, Esp32PcntState),
        VMSTATE_UINT32(ctrl, Esp32PcntState),
        VMSTATE_UINT32(date, Esp32PcntState),
        VMSTATE_UINT8_ARRAY(raw, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_UINT8_ARRAY(filt, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_INT64_ARRAY(raw_since_ns, Esp32PcntState,
                            ESP32_PCNT_INPUT_COUNT),
        VMSTATE_TIMER(filter_timer, Esp32PcntState),
        VMSTATE_BOOL(clk_running, Esp32PcntState),
        VMSTATE_CLOCK(apb_clk, Esp32PcntState),
        VMSTATE_UINT8_ARRAY(line, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_INT64_ARRAY(line_ns, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_INT64_ARRAY(bulk_ns, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_BOOL_ARRAY(bulk_on, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_pcnt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_pcnt_reset_hold;
    dc->realize = esp32_pcnt_realize;
    dc->vmsd = &vmstate_esp32_pcnt;
    device_class_set_props(dc, esp32_pcnt_properties);
}

/* [spec:nuos:req:emu.esp32.pcnt] */
static const TypeInfo esp32_pcnt_info = {
    .name = TYPE_ESP32_PCNT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32PcntState),
    .instance_init = esp32_pcnt_init,
    .class_init = esp32_pcnt_class_init,
};

static void esp32_pcnt_register_types(void)
{
    type_register_static(&esp32_pcnt_info);
}

type_init(esp32_pcnt_register_types)
