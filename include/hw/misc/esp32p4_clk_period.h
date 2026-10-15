/*
 * ESP32-P4 clock tree: exact clock periods and edges
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_CLK_PERIOD_H
#define HW_MISC_ESP32P4_CLK_PERIOD_H

#include "hw/core/clock.h"
#include "migration/vmstate.h"
#include "qemu/host-utils.h"
#include "qom/object.h"

/*
 * A Clock holds its period as a whole number of 2^-32 ns units. A clock
 * divided from one whose period is not a whole number of units would
 * inherit that rounding, multiplied by every divisor on its way: the
 * 480 MHz SPLL_CLK is 8947848533 1/3 units, and PLL_F160M_CLK, three of
 * its periods, would come out at 26843545599 units rather than its exact
 * 26843545600 (6.25 ns). The clock tree therefore carries each clock's
 * period as an exact fraction of units, from the exact frequencies of its
 * sources through every multiplier and divider, and rounds it only for
 * the Clock it drives: a clock whose period is a whole number of units
 * gets exactly that, and any other is less than a unit short of it,
 * however many dividers it has passed through.
 *
 * The rounding is down, to the largest whole number of units not above
 * the period. A block counts a clock's cycles as the whole number of
 * periods gone by, so that over an interval of exactly n cycles it counts
 * n, and takes the n-th cycle to come at the first nanosecond its count
 * reaches n, which is the cycle's exact time when that is a whole
 * nanosecond; with the period rounded up it would count n - 1 and find
 * the cycle a nanosecond late.
 *
 * The tree's denominators are bounded by its dividers' (the 2^23 of the
 * audio PLL's fraction, the 2^18 of an I2S divider's, the 2^8 of each
 * fractional divider, the 2^32 of a crystal's frequency), so they fit in
 * 64 bits, and the numerators in 128. A period too long for a Clock
 * saturates at UINT64_MAX units, too slow to run.
 */
typedef struct ESP32P4ClkPeriod {
    /* (num_hi:num_lo) / den units, in lowest terms; 0 / 1 when stopped. */
    uint64_t num_hi;
    uint64_t num_lo;
    uint64_t den;
} ESP32P4ClkPeriod;

static inline ESP32P4ClkPeriod esp32p4_clk_period_stopped(void)
{
    return (ESP32P4ClkPeriod) { 0, 0, 1 };
}

static inline bool esp32p4_clk_period_running(ESP32P4ClkPeriod p)
{
    return p.num_hi || p.num_lo;
}

static inline bool esp32p4_clk_period_eq(ESP32P4ClkPeriod a,
                                         ESP32P4ClkPeriod b)
{
    return a.num_hi == b.num_hi && a.num_lo == b.num_lo && a.den == b.den;
}

extern const VMStateDescription vmstate_esp32p4_clk_period;

#define VMSTATE_ESP32P4_CLK_PERIOD(field, state) \
    VMSTATE_STRUCT(field, state, 0, vmstate_esp32p4_clk_period, \
                   ESP32P4ClkPeriod)

/* A whole number of units. */
static inline ESP32P4ClkPeriod esp32p4_clk_period_exact(uint64_t period)
{
    return (ESP32P4ClkPeriod) { 0, period, 1 };
}

/* The largest whole number of units not above @p. */
static inline uint64_t esp32p4_clk_period_floor(ESP32P4ClkPeriod p)
{
    uint64_t lo = p.num_lo, hi = p.num_hi;

    divu128(&lo, &hi, p.den);
    return lo;
}

static inline uint64_t esp32p4_clk_gcd(uint64_t a, uint64_t b)
{
    while (b) {
        uint64_t r = a % b;

        a = b;
        b = r;
    }
    return a;
}

/* @p * @a / @b; stopped if @p is stopped or @a or @b is 0. */
static inline ESP32P4ClkPeriod esp32p4_clk_period_scale(ESP32P4ClkPeriod p,
                                                        uint64_t a,
                                                        uint64_t b)
{
    uint64_t num_lo = p.num_lo, num_hi = p.num_hi, den = p.den;
    uint64_t a0, b0, g, lo, hi, l0, h0, l1, h1, w1, w2, den_lo, den_hi;

    if (!esp32p4_clk_period_running(p) || !a || !b) {
        return esp32p4_clk_period_stopped();
    }
    g = esp32p4_clk_gcd(a, b);
    a0 = a /= g;
    b0 = b /= g;
    /* num / den and a / b are in lowest terms: cancel across them. */
    g = esp32p4_clk_gcd(a, den);
    a /= g;
    den /= g;
    lo = num_lo;
    hi = num_hi;
    g = esp32p4_clk_gcd(divu128(&lo, &hi, b), b);
    b /= g;
    divu128(&num_lo, &num_hi, g);
    mulu64(&den_lo, &den_hi, den, b);
    if (den_hi) {
        /* Beyond the tree's dividers: carry on from the whole units. */
        return esp32p4_clk_period_scale(
            esp32p4_clk_period_exact(esp32p4_clk_period_floor(p)), a0, b0);
    }
    /* num * a, the 192-bit w2:w1:l0, below den * 2^64 for a Clock period. */
    mulu64(&l0, &h0, num_lo, a);
    mulu64(&l1, &h1, num_hi, a);
    w1 = l1 + h0;
    w2 = h1 + (w1 < l1);
    if (w2 || w1 >= den_lo) {
        return (ESP32P4ClkPeriod) { 0, UINT64_MAX, 1 };
    }
    return (ESP32P4ClkPeriod) { w1, l0, den_lo };
}

/* The period of a clock whose frequency is the fraction @num / @den Hz. */
static inline ESP32P4ClkPeriod esp32p4_clk_period_of_hz(uint64_t num,
                                                        uint64_t den)
{
    return esp32p4_clk_period_scale(
        esp32p4_clk_period_exact(CLOCK_PERIOD_1SEC), den, num);
}

/*
 * The Clock period of @p: rounded down, as above; 0 only for a stopped
 * clock, at least one unit for any other.
 */
static inline uint64_t esp32p4_clk_period_clock(ESP32P4ClkPeriod p)
{
    if (!esp32p4_clk_period_running(p)) {
        return 0;
    }
    return MAX(esp32p4_clk_period_floor(p), 1);
}

/*
 * Counting cycles exactly
 * =======================
 *
 * A clock's cycle n, counted from one of its edges, ends exactly n periods
 * after that edge. A block that counts a clock's cycles, or times what it
 * does in them, takes the clock's exact period (esp32p4_clk_exact()) and
 * converts with these, never with the Clock's rounded period: QEMU's
 * clock_ticks_to_ns() rounds n periods of less than the exact length down,
 * so that cycle n can come out a nanosecond early, and the error grows
 * with n; a time kept by adding such conversions together loses up to a
 * nanosecond at each.
 *
 * esp32p4_clk_cycles_ns() is the first whole nanosecond by which n cycles
 * have elapsed: n periods rounded up, the cycle's exact time when that is
 * a whole nanosecond. esp32p4_clk_ns_cycles() is the number of whole
 * cycles that have elapsed by a time: the time over the period, rounded
 * down. They agree: the count reaches n at esp32p4_clk_cycles_ns(p, n),
 * and not a nanosecond before.
 */

/*
 * A period is num / den Clock units, num up to 128 bits and den up to 64;
 * converting between cycles and nanoseconds multiplies a 64-bit count by
 * one of them and divides by the other, through 192-bit intermediates.
 */

/* @a * (@b_hi:@b_lo), as the 192-bit r[2]:r[1]:r[0]. */
static inline void esp32p4_mul_64_128(uint64_t r[3], uint64_t a,
                                      uint64_t b_hi, uint64_t b_lo)
{
    uint64_t l0, h0, l1, h1;

    mulu64(&l0, &h0, a, b_lo);
    mulu64(&l1, &h1, a, b_hi);
    r[0] = l0;
    r[1] = h0 + l1;
    r[2] = h1 + (r[1] < l1);
}

static inline int esp32p4_cmp_192(const uint64_t a[3], const uint64_t b[3])
{
    int i;

    for (i = 2; i >= 0; i--) {
        if (a[i] != b[i]) {
            return a[i] < b[i] ? -1 : 1;
        }
    }
    return 0;
}

/*
 * The 192-bit @y over (@d_hi:@d_lo), which is not 0, rounded down; or
 * UINT64_MAX if that does not fit 64 bits. The remainder, below the
 * divisor, goes to @r[1]:@r[0] if @r is not NULL (and is not given when
 * the quotient does not fit).
 */
static inline uint64_t esp32p4_divrem_192_128(const uint64_t y[3],
                                              uint64_t d_hi, uint64_t d_lo,
                                              uint64_t *r)
{
    uint64_t u[3], p[3], n1, n0, lo, hi, w0, w1, w2;
    uint64_t q;
    int s;

    if (!d_hi) {
        if (y[2] || y[1] >= d_lo) {
            return UINT64_MAX;
        }
        lo = y[0];
        hi = y[1];
        w0 = divu128(&lo, &hi, d_lo);
        if (r) {
            r[0] = w0;
            r[1] = 0;
        }
        return lo;
    }
    /* The quotient fits 64 bits if y < d * 2^64. */
    if (y[2] > d_hi || (y[2] == d_hi && y[1] >= d_lo)) {
        return UINT64_MAX;
    }
    /*
     * One step of long division by a two-word divisor (Knuth, TAOCP vol. 2,
     * 4.3.1, algorithm D): with the divisor shifted to have its top bit set,
     * the top two words of the dividend over its top word overestimate the
     * quotient by at most 2.
     */
    s = clz64(d_hi);
    n1 = s ? d_hi << s | d_lo >> (64 - s) : d_hi;
    n0 = d_lo << s;
    u[2] = s ? y[2] << s | y[1] >> (64 - s) : y[2];
    u[1] = s ? y[1] << s | y[0] >> (64 - s) : y[1];
    u[0] = y[0] << s;
    if (u[2] >= n1) {
        q = UINT64_MAX;
    } else {
        lo = u[1];
        hi = u[2];
        divu128(&lo, &hi, n1);
        q = lo;
    }
    for (;;) {
        esp32p4_mul_64_128(p, q, n1, n0);
        if (esp32p4_cmp_192(p, u) <= 0) {
            break;
        }
        q--;
    }
    if (r) {
        /* u - p, below the shifted divisor: two words, shifted back. */
        w0 = u[0] - p[0];
        w1 = u[1] - p[1] - (u[0] < p[0]);
        w2 = u[2] - p[2] - (u[1] < p[1] || (u[1] == p[1] && u[0] < p[0]));
        r[0] = s ? w0 >> s | w1 << (64 - s) : w0;
        r[1] = s ? w1 >> s | w2 << (64 - s) : w1;
    }
    return q;
}

/*
 * The 192-bit @y over (@d_hi:@d_lo), which is not 0, rounded down; or
 * UINT64_MAX if that does not fit 64 bits. *@rem is set if the division
 * leaves a remainder.
 */
static inline uint64_t esp32p4_div_192_128(const uint64_t y[3],
                                           uint64_t d_hi, uint64_t d_lo,
                                           bool *rem)
{
    uint64_t r[2];
    uint64_t q = esp32p4_divrem_192_128(y, d_hi, d_lo, r);

    *rem = q == UINT64_MAX || r[0] || r[1];
    return q;
}

/* The 192-bit @y over @d, not 0: the quotient in @q, and the remainder. */
static inline uint64_t esp32p4_divrem_192_64(const uint64_t y[3], uint64_t d,
                                             uint64_t q[3])
{
    uint64_t r = 0;
    int i;

    for (i = 2; i >= 0; i--) {
        uint64_t lo = y[i], hi = r;

        r = divu128(&lo, &hi, d);
        q[i] = lo;
    }
    return r;
}

/* The 192-bit @y modulo (@n_hi:@n_lo), not 0, in @r[1]:@r[0]. */
static inline void esp32p4_mod_192_128(const uint64_t y[3], uint64_t n_hi,
                                       uint64_t n_lo, uint64_t r[2])
{
    uint64_t q[3], t[3];

    if (!n_hi) {
        r[0] = esp32p4_divrem_192_64(y, n_lo, q);
        r[1] = 0;
        return;
    }
    /*
     * With the divisor at least 2^64, the top two words over it, and then
     * what remains with the low word, each have a quotient of 64 bits.
     */
    t[0] = y[1];
    t[1] = y[2];
    t[2] = 0;
    esp32p4_divrem_192_128(t, n_hi, n_lo, r);
    t[0] = y[0];
    t[1] = r[0];
    t[2] = r[1];
    esp32p4_divrem_192_128(t, n_hi, n_lo, r);
}

/* @y += (@hi:@lo); false if that overflows 192 bits. */
static inline bool esp32p4_add_192_128(uint64_t y[3], uint64_t hi,
                                       uint64_t lo)
{
    uint64_t c;

    y[0] += lo;
    c = y[0] < lo;
    y[1] += c;
    c = y[1] < c;
    y[1] += hi;
    c += y[1] < hi;
    y[2] += c;
    return y[2] >= c;
}

/* Whether @y < (@hi:@lo). */
static inline bool esp32p4_lt_192_128(const uint64_t y[3], uint64_t hi,
                                      uint64_t lo)
{
    return !y[2] && (y[1] < hi || (y[1] == hi && y[0] < lo));
}

/* @y -= (@hi:@lo), which is not more than @y. */
static inline void esp32p4_sub_192_128(uint64_t y[3], uint64_t hi,
                                       uint64_t lo)
{
    uint64_t b;

    b = y[0] < lo;
    y[0] -= lo;
    if (y[1] < b) {
        y[1] -= b;
        b = 1;
    } else {
        y[1] -= b;
        b = 0;
    }
    b += y[1] < hi;
    y[1] -= hi;
    y[2] -= b;
}

/* n * num / (den * 2^32) ns, rounded down; *@rem if not a whole ns. */
static inline uint64_t esp32p4_clk_cycles_div(ESP32P4ClkPeriod p,
                                              uint64_t n, bool *rem)
{
    uint64_t y[3];

    esp32p4_mul_64_128(y, n, p.num_hi, p.num_lo);
    return esp32p4_div_192_128(y, p.den >> 32, p.den << 32, rem);
}

/*
 * The first whole ns by which @n cycles of a clock of period @p have
 * elapsed from one of its edges: 0 for none; INT64_MAX if the clock is
 * stopped, or if that is no earlier.
 */
static inline int64_t esp32p4_clk_cycles_ns(ESP32P4ClkPeriod p, uint64_t n)
{
    uint64_t q;
    bool rem;

    if (!n) {
        return 0;
    }
    if (!esp32p4_clk_period_running(p)) {
        return INT64_MAX;
    }
    q = esp32p4_clk_cycles_div(p, n, &rem);
    if (q >= INT64_MAX) {
        return INT64_MAX;
    }
    return q + rem;
}

/*
 * The whole ns in @n cycles of period @p, rounded down: the last whole ns
 * before the end of cycle @n, or at it, as INT64_MAX as above.
 */
static inline int64_t esp32p4_clk_cycles_ns_floor(ESP32P4ClkPeriod p,
                                                  uint64_t n)
{
    uint64_t q;
    bool rem;

    if (!n) {
        return 0;
    }
    if (!esp32p4_clk_period_running(p)) {
        return INT64_MAX;
    }
    q = esp32p4_clk_cycles_div(p, n, &rem);
    return MIN(q, INT64_MAX);
}

/*
 * The whole cycles of a clock of period @p that have elapsed @ns after one
 * of its edges: 0 for a stopped clock or @ns <= 0; saturating at
 * UINT64_MAX.
 */
static inline uint64_t esp32p4_clk_ns_cycles(ESP32P4ClkPeriod p, int64_t ns)
{
    uint64_t y[3], lo, hi;
    bool rem;

    if (ns <= 0 || !esp32p4_clk_period_running(p)) {
        return 0;
    }
    mulu64(&lo, &hi, ns, p.den);
    y[2] = hi >> 32;
    y[1] = hi << 32 | lo >> 32;
    y[0] = lo << 32;
    return esp32p4_div_192_128(y, p.num_hi, p.num_lo, &rem);
}

/*
 * The whole cycles of a clock of period @p that have elapsed
 * @units_hi:@units_lo Clock units (2^-32 ns) after one of its edges: 0 for
 * a stopped clock; saturating at UINT64_MAX.
 */
static inline uint64_t esp32p4_clk_units_cycles(ESP32P4ClkPeriod p,
                                                uint64_t units_hi,
                                                uint64_t units_lo)
{
    uint64_t y[3];
    bool rem;

    if (!esp32p4_clk_period_running(p)) {
        return 0;
    }
    esp32p4_mul_64_128(y, p.den, units_hi, units_lo);
    return esp32p4_div_192_128(y, p.num_hi, p.num_lo, &rem);
}

/* esp32p4_clk_ns_cycles(), counting a cycle begun as a whole one. */
static inline uint64_t esp32p4_clk_ns_cycles_up(ESP32P4ClkPeriod p,
                                                int64_t ns)
{
    uint64_t y[3], lo, hi, q;
    bool rem;

    if (ns <= 0 || !esp32p4_clk_period_running(p)) {
        return 0;
    }
    mulu64(&lo, &hi, ns, p.den);
    y[2] = hi >> 32;
    y[1] = hi << 32 | lo >> 32;
    y[0] = lo << 32;
    q = esp32p4_div_192_128(y, p.num_hi, p.num_lo, &rem);
    return q == UINT64_MAX ? q : q + rem;
}

/* esp32p4_clk_units_cycles(), counting a cycle begun as a whole one. */
static inline uint64_t esp32p4_clk_units_cycles_up(ESP32P4ClkPeriod p,
                                                   uint64_t units_hi,
                                                   uint64_t units_lo)
{
    uint64_t y[3], q;
    bool rem;

    if (!esp32p4_clk_period_running(p)) {
        return 0;
    }
    esp32p4_mul_64_128(y, p.den, units_hi, units_lo);
    q = esp32p4_div_192_128(y, p.num_hi, p.num_lo, &rem);
    return q == UINT64_MAX ? q : q + rem;
}

/* (@a_hi:@a_lo) * (@b_hi:@b_lo), as the 256-bit r[3]:r[2]:r[1]:r[0]. */
static inline void esp32p4_mul_128_128(uint64_t r[4], uint64_t a_hi,
                                       uint64_t a_lo, uint64_t b_hi,
                                       uint64_t b_lo)
{
    uint64_t lo[3], hi[3], c;

    esp32p4_mul_64_128(lo, a_lo, b_hi, b_lo);
    esp32p4_mul_64_128(hi, a_hi, b_hi, b_lo);
    r[0] = lo[0];
    r[1] = lo[1] + hi[0];
    c = r[1] < hi[0];
    r[2] = lo[2] + c;
    c = r[2] < c;
    r[2] += hi[1];
    c += r[2] < hi[1];
    r[3] = hi[2] + c;
}

/*
 * Whether @m cycles of a clock of period @p last less than (-1), exactly
 * as long as (0), or longer than (1) @n cycles of a clock of period @q: a
 * stopped clock's cycles take no time.
 */
static inline int esp32p4_clk_cycles_cmp(ESP32P4ClkPeriod p, uint64_t m,
                                         ESP32P4ClkPeriod q, uint64_t n)
{
    uint64_t a_lo, a_hi, b_lo, b_hi, x[4], y[4];
    int i;

    /* m * p.num / p.den against n * q.num / q.den, both times both dens. */
    mulu64(&a_lo, &a_hi, m, q.den);
    mulu64(&b_lo, &b_hi, n, p.den);
    esp32p4_mul_128_128(x, a_hi, a_lo, p.num_hi, p.num_lo);
    esp32p4_mul_128_128(y, b_hi, b_lo, q.num_hi, q.num_lo);
    for (i = 3; i >= 0; i--) {
        if (x[i] != y[i]) {
            return x[i] < y[i] ? -1 : 1;
        }
    }
    return 0;
}

/* Whether a period @p is a whole number of ns, every edge on a whole ns. */
static inline bool esp32p4_clk_period_whole_ns(ESP32P4ClkPeriod p)
{
    return !p.num_hi && p.den == 1 && !(p.num_lo & 0xffffffff);
}

/*
 * Clocks' edges
 * =============
 *
 * A clock's rising edges are at one phase of its period, which the clock
 * tree carries with the period: at the virtual times (phase + k * num) /
 * den Clock units after time 0, for every integer k, the period num / den
 * units and 0 <= phase < num, so that every edge is exact to the
 * period's own fraction of a unit. Where a clock begins, its first edge
 * is there; a clock divided from another, or a PLL locked to its
 * reference, takes its edges from that clock's; a mux or a gate passes
 * on its input's. Two clocks of one frequency need not have one phase,
 * and a block that switches between them sees the edges move.
 */
typedef struct ESP32P4ClkEdges {
    ESP32P4ClkPeriod period;
    uint64_t phase_hi;
    uint64_t phase_lo;
} ESP32P4ClkEdges;

extern const VMStateDescription vmstate_esp32p4_clk_edges;

#define VMSTATE_ESP32P4_CLK_EDGES(field, state) \
    VMSTATE_STRUCT(field, state, 0, vmstate_esp32p4_clk_edges, \
                   ESP32P4ClkEdges)

static inline ESP32P4ClkEdges esp32p4_clk_edges_stopped(void)
{
    return (ESP32P4ClkEdges) { esp32p4_clk_period_stopped(), 0, 0 };
}

static inline bool esp32p4_clk_edges_running(const ESP32P4ClkEdges *e)
{
    return esp32p4_clk_period_running(e->period);
}

static inline bool esp32p4_clk_edges_eq(const ESP32P4ClkEdges *a,
                                        const ESP32P4ClkEdges *b)
{
    return esp32p4_clk_period_eq(a->period, b->period) &&
           a->phase_hi == b->phase_hi && a->phase_lo == b->phase_lo;
}

/* @t ns, at least 0, as the 192-bit @y, t * 2^32 * @den. */
static inline void esp32p4_clk_scaled(uint64_t y[3], int64_t t, uint64_t den)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, t > 0 ? t : 0, den);
    y[0] = lo << 32;
    y[1] = hi << 32 | lo >> 32;
    y[2] = hi >> 32;
}

/* The edges of a clock of period @p with an edge at @t ns. */
static inline ESP32P4ClkEdges esp32p4_clk_edges_at(ESP32P4ClkPeriod p,
                                                   int64_t t)
{
    ESP32P4ClkEdges e = { p, 0, 0 };
    uint64_t y[3], r[2];

    if (!esp32p4_clk_period_running(p)) {
        return esp32p4_clk_edges_stopped();
    }
    esp32p4_clk_scaled(y, t, p.den);
    esp32p4_mod_192_128(y, p.num_hi, p.num_lo, r);
    e.phase_hi = r[1];
    e.phase_lo = r[0];
    return e;
}

/*
 * The number k of the last edge at or before @t ns, (phase + k * num) /
 * den units: -1 for the one before time 0 when no later one is; saturating
 * at INT64_MAX. *@at is set if that edge is at @t exactly. The clock must
 * run.
 */
static inline int64_t esp32p4_clk_edge_index(const ESP32P4ClkEdges *e,
                                             int64_t t, bool *at)
{
    uint64_t y[3], r[2], q;

    esp32p4_clk_scaled(y, t, e->period.den);
    if (esp32p4_lt_192_128(y, e->phase_hi, e->phase_lo)) {
        *at = false;
        return -1;
    }
    esp32p4_sub_192_128(y, e->phase_hi, e->phase_lo);
    q = esp32p4_divrem_192_128(y, e->period.num_hi, e->period.num_lo, r);
    if (q >= INT64_MAX) {
        *at = false;
        return INT64_MAX;
    }
    *at = !r[0] && !r[1];
    return q;
}

/*
 * The first whole ns at or after edge @k, k >= 0, of a running clock:
 * INT64_MAX if that is no earlier.
 */
static inline int64_t esp32p4_clk_edge_ns(const ESP32P4ClkEdges *e,
                                          uint64_t k)
{
    uint64_t y[3], q;
    bool rem;

    esp32p4_mul_64_128(y, k, e->period.num_hi, e->period.num_lo);
    if (!esp32p4_add_192_128(y, e->phase_hi, e->phase_lo)) {
        return INT64_MAX;
    }
    q = esp32p4_div_192_128(y, e->period.den >> 32, e->period.den << 32,
                            &rem);
    if (q >= INT64_MAX) {
        return INT64_MAX;
    }
    return q + rem;
}

/*
 * The time from @t ns to the first edge at or after it of a running clock,
 * in Clock units, rounded up: 0 for an edge at @t; saturating at
 * UINT64_MAX.
 */
static inline uint64_t esp32p4_clk_edge_wait(const ESP32P4ClkEdges *e,
                                             int64_t t)
{
    uint64_t y[3], a[3], q;
    int64_t k;
    bool at, rem;

    k = esp32p4_clk_edge_index(e, t, &at);
    if (at || k == INT64_MAX) {
        return 0;
    }
    /* Edge k + 1, less t: (phase + (k + 1) * num - t * 2^32 * den) / den. */
    esp32p4_mul_64_128(a, k + 1, e->period.num_hi, e->period.num_lo);
    if (!esp32p4_add_192_128(a, e->phase_hi, e->phase_lo)) {
        return UINT64_MAX;
    }
    esp32p4_clk_scaled(y, t, e->period.den);
    esp32p4_sub_192_128(a, y[1], y[0]);
    a[2] -= y[2];
    q = esp32p4_div_192_128(a, 0, e->period.den, &rem);
    return q == UINT64_MAX ? UINT64_MAX : q + rem;
}

/*
 * The time to @t ns from the last edge at or before it of a running clock,
 * in Clock units, rounded down: 0 for an edge at @t; saturating at
 * UINT64_MAX.
 */
static inline uint64_t esp32p4_clk_edge_since(const ESP32P4ClkEdges *e,
                                              int64_t t)
{
    uint64_t y[3], a[3];
    int64_t k;
    bool at, rem;

    k = esp32p4_clk_edge_index(e, t, &at);
    if (at || k == INT64_MAX) {
        return 0;
    }
    /* t, less edge k: (t * 2^32 * den - phase - k * num) / den. */
    esp32p4_clk_scaled(y, t, e->period.den);
    if (k < 0) {
        esp32p4_add_192_128(y, e->period.num_hi, e->period.num_lo);
        esp32p4_sub_192_128(y, e->phase_hi, e->phase_lo);
    } else {
        esp32p4_mul_64_128(a, k, e->period.num_hi, e->period.num_lo);
        esp32p4_add_192_128(a, e->phase_hi, e->phase_lo);
        esp32p4_sub_192_128(y, a[1], a[0]);
        y[2] -= a[2];
    }
    return esp32p4_div_192_128(y, 0, e->period.den, &rem);
}

/*
 * The edges of a clock of period @p through edge @k, at least 0, of the
 * running @in; to within 1 / p.den of a unit. Out of range, from an edge
 * at @t ns.
 */
static inline ESP32P4ClkEdges esp32p4_clk_edges_through(
    ESP32P4ClkPeriod p, const ESP32P4ClkEdges *in, int64_t k, int64_t t)
{
    ESP32P4ClkEdges e = { p, 0, 0 };
    uint64_t a[3], q[3], v[3], r[2], rr, lo, hi;

    if (k == INT64_MAX) {
        return esp32p4_clk_edges_at(p, t);
    }
    /* The edge: a / in.den units, a = phase + k * num. */
    esp32p4_mul_64_128(a, k, in->period.num_hi, in->period.num_lo);
    /* a = Q * in.den + R; at p.den, Q * p.den + R * p.den / in.den. */
    if (!esp32p4_add_192_128(a, in->phase_hi, in->phase_lo)) {
        return esp32p4_clk_edges_at(p, t);
    }
    rr = esp32p4_divrem_192_64(a, in->period.den, q);
    if (q[2]) {
        return esp32p4_clk_edges_at(p, t);
    }
    esp32p4_mul_64_128(v, p.den, q[1], q[0]);
    mulu64(&lo, &hi, rr, p.den);
    divu128(&lo, &hi, in->period.den);
    v[0] += lo;
    if (v[0] < lo && !++v[1]) {
        v[2]++;
    }
    esp32p4_mod_192_128(v, p.num_hi, p.num_lo, r);
    e.phase_hi = r[1];
    e.phase_lo = r[0];
    return e;
}

/*
 * The edges of a clock of period @p whose edges are some of those of @in,
 * its first one at the first edge of @in at or after @t ns: as a PLL
 * locking to @in at @t has them, or a block that starts dividing @in at
 * its next edge. With @in stopped, from an edge at @t.
 */
static inline ESP32P4ClkEdges esp32p4_clk_edges_from(ESP32P4ClkPeriod p,
                                                     const ESP32P4ClkEdges *in,
                                                     int64_t t)
{
    int64_t k;
    bool at;

    if (!esp32p4_clk_period_running(p)) {
        return esp32p4_clk_edges_stopped();
    }
    if (!esp32p4_clk_edges_running(in)) {
        return esp32p4_clk_edges_at(p, t);
    }
    k = esp32p4_clk_edge_index(in, t, &at);
    return esp32p4_clk_edges_through(p, in, k == INT64_MAX ? k : k + !at, t);
}

/*
 * The edges of a divider of @in to period @p restarted at @t ns: counting
 * @in's edges after @t, its first edge is at the one that completes its
 * period, the period after @in's last edge at or before @t (or its first,
 * when it has none by then). With @in stopped, from an edge at @t.
 */
static inline ESP32P4ClkEdges esp32p4_clk_edges_restart(
    ESP32P4ClkPeriod p, const ESP32P4ClkEdges *in, int64_t t)
{
    int64_t k;
    bool at;

    if (!esp32p4_clk_period_running(p)) {
        return esp32p4_clk_edges_stopped();
    }
    if (!esp32p4_clk_edges_running(in)) {
        return esp32p4_clk_edges_at(p, t);
    }
    k = esp32p4_clk_edge_index(in, t, &at);
    return esp32p4_clk_edges_through(p, in, MAX(k, 0), t);
}

/*
 * A Clock carries only the rounded period. The device that drives a clock
 * knows its edges exactly: the reset and clock control for the clock
 * tree's outputs, the analog block for its PLLs, and a block that drives
 * a clock onto a pad (an I2S controller's MCLK_out) or makes one (the USB
 * PHY's PLL) for that clock. Each implements the TYPE_ESP32P4_CLK_SOURCE
 * interface, and esp32p4_clk_exact_edges() asks it for the clock at the
 * source of any Clock connected to its output, through every clock input
 * that follows it.
 */
#define TYPE_ESP32P4_CLK_SOURCE "esp32p4-clk-source"

typedef struct ESP32P4ClkSourceClass ESP32P4ClkSourceClass;
DECLARE_CLASS_CHECKERS(ESP32P4ClkSourceClass, ESP32P4_CLK_SOURCE,
                       TYPE_ESP32P4_CLK_SOURCE)

struct ESP32P4ClkSourceClass {
    InterfaceClass parent_class;

    /*
     * The edges, exact period and phase, of @out, a clock output of @obj,
     * as @out last took them: false if @obj does not drive @out.
     */
    bool (*exact_edges)(Object *obj, Clock *out, ESP32P4ClkEdges *e);
    /*
     * Optional: make the changes of @obj's outputs that fell due by @now
     * (a PLL locking), each at its own time, which a QEMU timer running
     * late has not made yet. The reset and clock control calls it on the
     * sources of its inputs before an access to it or to a block behind
     * its gates, so that the access finds the clocks as they are then.
     */
    void (*sync)(Object *obj, int64_t now);
};

/*
 * The edges of the clock on @clk: as the device driving the clock at its
 * source gives them, while their period rounds to @clk's period, and
 * true; otherwise @clk's period, taken as exact, at phase 0, and false. A
 * Clock's callbacks see the source's new edges from ClockUpdate on, and
 * @clk's own old period in ClockPreUpdate, as the source has already
 * moved on: a block that counts a clock across its changes keeps the
 * edges it took at the last ClockUpdate (an ESP32P4ClkCount). A change of
 * the edges that leaves the Clock's period as it was, a change of phase
 * or of the period by less than a Clock unit, runs the callbacks all the
 * same (esp32p4_clk_update()).
 */
static inline bool esp32p4_clk_exact_edges(Clock *clk, ESP32P4ClkEdges *e)
{
    uint64_t period = clock_get(clk);
    Clock *root = clk;
    Object *owner;

    *e = (ESP32P4ClkEdges) { esp32p4_clk_period_exact(period), 0, 0 };
    if (!period) {
        *e = esp32p4_clk_edges_stopped();
        return true;
    }
    /*
     * A clock input has its source's edges, unless clock_set_mul_div()
     * scales it on the way, which the source cannot know.
     */
    while (root->source) {
        root = root->source;
        if (root->multiplier != 1 || root->divider != 1) {
            return false;
        }
    }
    owner = OBJECT(root)->parent;
    if (owner && object_dynamic_cast(owner, TYPE_ESP32P4_CLK_SOURCE)) {
        ESP32P4ClkSourceClass *k = ESP32P4_CLK_SOURCE_GET_CLASS(owner);
        ESP32P4ClkEdges x;

        if (k->exact_edges(owner, root, &x) &&
            esp32p4_clk_period_clock(x.period) == period) {
            *e = x;
            return true;
        }
    }
    return false;
}

/*
 * The device driving the clock at a Clock's source, @owner, and its
 * ESP32P4ClkSourceClass sync, NULL if it has none, as
 * esp32p4_clk_find_source() last found them for the source at @root.
 */
typedef struct ESP32P4ClkSourceSync {
    Clock *root;
    Object *owner;
    void (*sync)(Object *obj, int64_t now);
} ESP32P4ClkSourceSync;

/*
 * Find the device driving the clock on @clk at its source, and its sync,
 * which makes the changes the device has due by a time, into @found. What
 * @found holds stands while @clk's source stays the same: a caller
 * syncing before every access to a block would otherwise spend more on
 * finding them, a QOM interface cast each, than on the access.
 */
static inline void esp32p4_clk_find_source(Clock *clk,
                                           ESP32P4ClkSourceSync *found)
{
    Clock *root = clk;
    Object *owner;

    while (root->source) {
        root = root->source;
    }
    if (root == found->root) {
        return;
    }
    owner = OBJECT(root)->parent;
    found->root = root;
    found->owner = owner;
    found->sync = NULL;
    if (owner && object_dynamic_cast(owner, TYPE_ESP32P4_CLK_SOURCE)) {
        found->sync = ESP32P4_CLK_SOURCE_GET_CLASS(owner)->sync;
    }
}

/* The exact period of the clock on @clk (esp32p4_clk_exact_edges()). */
static inline ESP32P4ClkPeriod esp32p4_clk_exact(Clock *clk)
{
    ESP32P4ClkEdges e;

    esp32p4_clk_exact_edges(clk, &e);
    return e.period;
}

/*
 * The edges of the clock on @clk; where its source does not give them, a
 * clock of its period with an edge at @t ns, as a block that sees the
 * clock change at @t takes it.
 */
static inline ESP32P4ClkEdges esp32p4_clk_edges_or_at(Clock *clk, int64_t t)
{
    ESP32P4ClkEdges e;

    if (!esp32p4_clk_exact_edges(clk, &e)) {
        e = esp32p4_clk_edges_at(e.period, t);
    }
    return e;
}

/*
 * Drive @clk, an output of a TYPE_ESP32P4_CLK_SOURCE, whose edges have
 * changed from @old to @new: its Clock to @new's period rounded, which
 * runs the callbacks of the clocks connected to it if that changes, and
 * when it does not, while @new is not @old, the same callbacks all the
 * same (clock_propagate_retime()), so that the blocks timed from the
 * clock take its new edges.
 */
static inline void esp32p4_clk_update(Clock *clk, const ESP32P4ClkEdges *old,
                                      const ESP32P4ClkEdges *new)
{
    uint64_t period = esp32p4_clk_period_clock(new->period);

    if (clock_get(clk) != period) {
        clock_update(clk, period);
    } else if (!esp32p4_clk_edges_eq(old, new)) {
        clock_propagate_retime(clk);
    }
}

/*
 * A clock's cycles counted from @origin_ns: the edges after it, cycle n
 * ending at the n-th, edge number base + n of @edges, base the last one
 * at or before the origin. A block that counts a clock across its
 * changes starts one at each change (esp32p4_clk_count_start() in its
 * ClockUpdate callback), its cycles beginning afresh at the first edge of
 * the clock after the change, and keeps the cycles counted before.
 */
typedef struct ESP32P4ClkCount {
    ESP32P4ClkEdges edges;
    int64_t origin_ns;
    int64_t base;
} ESP32P4ClkCount;

extern const VMStateDescription vmstate_esp32p4_clk_count;

#define VMSTATE_ESP32P4_CLK_COUNT(field, state) \
    VMSTATE_STRUCT(field, state, 0, vmstate_esp32p4_clk_count, \
                   ESP32P4ClkCount)

/* Count the cycles of @c's clock from @t ns on, at its edges. */
static inline void esp32p4_clk_count_rebase(ESP32P4ClkCount *c, int64_t t)
{
    bool at;

    c->origin_ns = t;
    c->base = esp32p4_clk_edges_running(&c->edges) ?
              esp32p4_clk_edge_index(&c->edges, t, &at) : 0;
}

/* Count @clk's cycles, at its current edges, from @now on. */
static inline void esp32p4_clk_count_start(ESP32P4ClkCount *c, Clock *clk,
                                           int64_t now)
{
    c->edges = esp32p4_clk_edges_or_at(clk, now);
    esp32p4_clk_count_rebase(c, now);
}

static inline bool esp32p4_clk_count_running(const ESP32P4ClkCount *c)
{
    return esp32p4_clk_edges_running(&c->edges);
}

/* The cycles completed by @t: 0 before the origin and while stopped. */
static inline uint64_t esp32p4_clk_count_at(const ESP32P4ClkCount *c,
                                            int64_t t)
{
    int64_t k;
    bool at;

    if (t <= c->origin_ns || !esp32p4_clk_count_running(c)) {
        return 0;
    }
    k = esp32p4_clk_edge_index(&c->edges, t, &at);
    return k > c->base ? k - c->base : 0;
}

/*
 * The first whole ns by which cycle @n has completed: the origin for 0;
 * INT64_MAX while stopped, or if that is no earlier.
 */
static inline int64_t esp32p4_clk_count_time(const ESP32P4ClkCount *c,
                                             uint64_t n)
{
    if (!n) {
        return c->origin_ns;
    }
    if (!esp32p4_clk_count_running(c) || n > INT64_MAX - c->base) {
        return INT64_MAX;
    }
    return MAX(esp32p4_clk_edge_ns(&c->edges, c->base + n), c->origin_ns);
}

/*
 * The first whole ns by which @n cycles of a clock with edges @e have
 * completed from @t ns on: its n-th edge after @t; @t for none, INT64_MAX
 * while stopped.
 */
static inline int64_t esp32p4_clk_edges_after(const ESP32P4ClkEdges *e,
                                              int64_t t, uint64_t n)
{
    ESP32P4ClkCount c = { .edges = *e };

    esp32p4_clk_count_rebase(&c, t);
    return esp32p4_clk_count_time(&c, n);
}

/*
 * The cycles of a clock with edges @e completed from @from ns to @t ns:
 * its edges after @from and at or before @t; 0 while stopped.
 */
static inline uint64_t esp32p4_clk_edges_between(const ESP32P4ClkEdges *e,
                                                 int64_t from, int64_t t)
{
    ESP32P4ClkCount c = { .edges = *e };

    esp32p4_clk_count_rebase(&c, from);
    return esp32p4_clk_count_at(&c, t);
}

/*
 * The cycles @x has completed by the exact time cycle @k of @c completes,
 * which need not be a whole ns: 0 before @x's origin and while either is
 * stopped; saturating at UINT64_MAX. Of two clocks on one count, as a
 * calibration counts one clock's edges between two of another's.
 */
static inline uint64_t esp32p4_clk_count_at_cycle(const ESP32P4ClkCount *x,
                                                  const ESP32P4ClkCount *c,
                                                  uint64_t k)
{
    const ESP32P4ClkEdges *ec = &c->edges, *ex = &x->edges;
    uint64_t a[3], q[3], v[3], rr, lo, hi, kc, idx;
    bool rem;

    if (!esp32p4_clk_count_running(c) || !esp32p4_clk_count_running(x) ||
        k > INT64_MAX - c->base) {
        return 0;
    }
    kc = c->base + k;
    /* The edge: a / ec.den units, a = phase + kc * num. */
    esp32p4_mul_64_128(a, kc, ec->period.num_hi, ec->period.num_lo);
    if (!esp32p4_add_192_128(a, ec->phase_hi, ec->phase_lo)) {
        return UINT64_MAX;
    }
    /* At @x's denominator, rounded down: Q * ex.den + R * ex.den / ec.den */
    rr = esp32p4_divrem_192_64(a, ec->period.den, q);
    if (q[2]) {
        return UINT64_MAX;
    }
    esp32p4_mul_64_128(v, ex->period.den, q[1], q[0]);
    mulu64(&lo, &hi, rr, ex->period.den);
    divu128(&lo, &hi, ec->period.den);
    v[0] += lo;
    if (v[0] < lo && !++v[1]) {
        v[2]++;
    }
    /* Less @x's phase, over its period: the last edge of @x by then. */
    if (esp32p4_lt_192_128(v, ex->phase_hi, ex->phase_lo)) {
        return 0;
    }
    esp32p4_sub_192_128(v, ex->phase_hi, ex->phase_lo);
    idx = esp32p4_div_192_128(v, ex->period.num_hi, ex->period.num_lo, &rem);
    if (idx == UINT64_MAX) {
        return UINT64_MAX;
    }
    return (int64_t)idx > x->base ? idx - x->base : 0;
}

#endif
