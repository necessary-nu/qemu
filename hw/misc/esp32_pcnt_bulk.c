/*
 * Espressif pulse counters taking a periodic output's edges in bulk
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/misc/esp32_pcnt_bulk.h. The ESP32's and the ESP32-P4's
 * PCNT models share this: an input's filtered edges from a wave, and the
 * arithmetic of a counter stepped by them.
 */

#include "qemu/osdep.h"
#include "hw/misc/esp32_pcnt_bulk.h"

/* When a change at e reaches the counter past the filter. */
int64_t esp32_pcnt_feed_due(const Esp32PcntFeed *f, int64_t e)
{
    if (!f->filter || e == INT64_MAX) {
        return e;
    }
    return f->due(f->opaque, e);
}

/* The time of the wave's next edge after pos; INT64_MAX if none. */
int64_t esp32_pcnt_feed_next_raw(const Esp32PcntFeed *f)
{
    return f->w->ops->edge(f->w, f->pos, 1);
}

/*
 * Whether the filter passes every pulse of the wave, each lasting at least
 * as long as any change takes to reach the counter: a change is then due
 * no later than the edge after it, which the counter takes only after
 * what is due by then, and every pulse, and every gap between two, is
 * seen at as many of the filter's samples as it asks for.
 */
bool esp32_pcnt_feed_passes(const Esp32PcntFeed *f)
{
    if (!f->filter) {
        return true;
    }
    return f->w->ops->min_hold(f->w) >= f->dmax;
}

/*
 * Whether a change waiting in the filter at pos, if any, reaches the
 * counter no later than the wave's next edge: from then on every edge's
 * change does (esp32_pcnt_feed_passes()).
 */
bool esp32_pcnt_feed_aligned(const Esp32PcntFeed *f)
{
    if (f->raw == f->filt) {
        return true;
    }
    return esp32_pcnt_feed_due(f, f->raw_since) <=
           esp32_pcnt_feed_next_raw(f);
}

/*
 * How many filtered edges come by x, aligned and passing: a change waiting
 * at pos, if due by then, which can be at pos itself (one that changed the
 * input's level there, unfiltered), and the wave's edges in (pos, x].
 */
uint64_t esp32_pcnt_feed_count(const Esp32PcntFeed *f, int64_t x)
{
    Esp32LineWave *w = f->w;
    uint64_t n = 0, j;

    if (x < f->pos) {
        return 0;
    }
    if (f->raw != f->filt) {
        if (esp32_pcnt_feed_due(f, f->raw_since) > x) {
            return 0;
        }
        n = 1;
    }
    if (!f->filter) {
        return n + w->ops->edges(w, f->pos, x);
    }
    /* Every edge by x - dmax is due by x; of those after, the first few. */
    j = x - f->dmax > f->pos ? w->ops->edges(w, f->pos, x - f->dmax) : 0;
    for (;;) {
        int64_t e = w->ops->edge(w, f->pos, j + 1);

        if (e > x || esp32_pcnt_feed_due(f, e) > x) {
            break;
        }
        j++;
    }
    return n + j;
}

/* The time of the m-th filtered edge after pos (m >= 1). */
int64_t esp32_pcnt_feed_time(const Esp32PcntFeed *f, uint64_t m)
{
    Esp32LineWave *w = f->w;

    if (f->raw != f->filt) {
        if (m == 1) {
            return esp32_pcnt_feed_due(f, f->raw_since);
        }
        m--;
    }
    return esp32_pcnt_feed_due(f, w->ops->edge(w, f->pos, m));
}

/*
 * Bring the feed to x, at or after pos, aligned and passing: its level,
 * when it last changed, and its level past the filter.
 */
void esp32_pcnt_feed_advance(Esp32PcntFeed *f, int64_t x)
{
    Esp32LineWave *w = f->w;
    uint64_t nf, nr;

    if (x < f->pos) {
        return;
    }
    nf = esp32_pcnt_feed_count(f, x);
    nr = w->ops->edges(w, f->pos, x);
    if (nr) {
        f->raw_since = w->ops->edge(w, f->pos, nr);
    }
    f->raw = w->ops->level(w, x) ^ f->inv;
    f->filt ^= nf & 1;
    f->pos = x;
}

/*
 * The first step s >= 1 at which a count walking from c by d (+1 or -1)
 * a step, wrapping at 16 bits, is in [lo, hi]: the walk, in the order of
 * the 16-bit values from -32768 up, enters the interval at lo going up and
 * at hi going down, unless its first step is already in it.
 */
static uint64_t steps_into(int16_t c, int d, int16_t lo, int16_t hi)
{
    uint16_t o0 = (uint16_t)c ^ 0x8000;
    uint16_t ol = (uint16_t)lo ^ 0x8000;
    uint16_t oh = (uint16_t)hi ^ 0x8000;
    uint16_t o1 = o0 + d;
    uint16_t dist;

    if (o1 >= ol && o1 <= oh) {
        return 1;
    }
    dist = d > 0 ? (uint16_t)(ol - o0) : (uint16_t)(o0 - oh);
    return dist ? dist : 65536;
}

static bool cmp_match(const Esp32PcntCmp *cmp, unsigned n, int16_t v)
{
    for (unsigned i = 0; i < n; i++) {
        switch (cmp[i].kind) {
        case ESP32_PCNT_CMP_EQ:
            if (v == cmp[i].v) {
                return true;
            }
            break;
        case ESP32_PCNT_CMP_GE:
            if (v >= cmp[i].v) {
                return true;
            }
            break;
        default:
            if (v <= cmp[i].v) {
                return true;
            }
            break;
        }
    }
    return false;
}

/*
 * Of the filtered edges stepping a counter at c by dn and da alternately,
 * the first (1 for the next) whose step leaves a count a watch point
 * matches; UINT64_MAX if none does.
 */
uint64_t esp32_pcnt_first_match(int16_t c, int dn, int da,
                                const Esp32PcntCmp *cmp, unsigned n)
{
    uint64_t s = UINT64_MAX;
    int d;

    if (!dn && !da) {
        return UINT64_MAX;
    }
    if (dn && da && dn != da) {
        /* Back and forth between c + dn and c */
        if (cmp_match(cmp, n, (int16_t)(uint16_t)(c + dn))) {
            return 1;
        }
        return cmp_match(cmp, n, c) ? 2 : UINT64_MAX;
    }
    d = dn ? dn : da;
    for (unsigned i = 0; i < n; i++) {
        int16_t lo = cmp[i].v, hi = cmp[i].v;

        if (cmp[i].kind == ESP32_PCNT_CMP_GE) {
            hi = INT16_MAX;
        } else if (cmp[i].kind == ESP32_PCNT_CMP_LE) {
            lo = INT16_MIN;
        }
        s = MIN(s, steps_into(c, d, lo, hi));
    }
    if (s == UINT64_MAX) {
        return s;
    }
    if (!da) {
        return 2 * s - 1;
    }
    if (!dn) {
        return 2 * s;
    }
    return s;
}

/*
 * The counter at c after k filtered edges stepping it by dn and da
 * alternately, none of them matching a watch point: whether any stepped
 * it, and if so the count after the last step (*last) and before it
 * (*before), from which its zero mode follows.
 */
bool esp32_pcnt_count_edges(int16_t c, int dn, int da, uint64_t k,
                            int16_t *last, int16_t *before)
{
    uint64_t steps;
    int d;

    if (dn && da && dn != da) {
        if (!k) {
            return false;
        }
        *last = k & 1 ? (int16_t)(uint16_t)(c + dn) : c;
        *before = k & 1 ? c : (int16_t)(uint16_t)(c + dn);
        return true;
    }
    if (dn && da) {
        steps = k;
    } else if (dn) {
        steps = (k + 1) / 2;
    } else if (da) {
        steps = k / 2;
    } else {
        steps = 0;
    }
    if (!steps) {
        return false;
    }
    d = dn ? dn : da;
    *last = (int16_t)(uint16_t)(c + (uint16_t)(steps * d));
    *before = (int16_t)(uint16_t)(*last - d);
    return true;
}
