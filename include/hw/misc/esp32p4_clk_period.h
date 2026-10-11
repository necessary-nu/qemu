/*
 * ESP32-P4 clock tree: exact clock periods
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
 * UINT64_MAX if that does not fit 64 bits. *@rem is set if the division
 * leaves a remainder.
 */
static inline uint64_t esp32p4_div_192_128(const uint64_t y[3],
                                           uint64_t d_hi, uint64_t d_lo,
                                           bool *rem)
{
    uint64_t u[3], p[3], n1, n0, lo, hi;
    uint64_t q;
    int s;

    *rem = true;
    if (!d_hi) {
        if (y[2] || y[1] >= d_lo) {
            return UINT64_MAX;
        }
        lo = y[0];
        hi = y[1];
        *rem = divu128(&lo, &hi, d_lo) != 0;
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
    *rem = esp32p4_cmp_192(p, u) != 0;
    return q;
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

/* Whether a period @p is a whole number of ns, every edge on a whole ns. */
static inline bool esp32p4_clk_period_whole_ns(ESP32P4ClkPeriod p)
{
    return !p.num_hi && p.den == 1 && !(p.num_lo & 0xffffffff);
}

/*
 * Clocks' exact periods
 * =====================
 *
 * A Clock carries only the rounded period. The device that drives a clock
 * knows it exactly: the reset and clock control for the clock tree's
 * outputs, the analog block for its PLLs, and a block that drives a clock
 * onto a pad (an I2S controller's MCLK_out) for that clock. Each
 * implements the TYPE_ESP32P4_CLK_SOURCE interface, and
 * esp32p4_clk_exact() asks it for the clock at the source of any Clock
 * connected to its output, through every clock input that follows it.
 */
#define TYPE_ESP32P4_CLK_SOURCE "esp32p4-clk-source"

typedef struct ESP32P4ClkSourceClass ESP32P4ClkSourceClass;
DECLARE_CLASS_CHECKERS(ESP32P4ClkSourceClass, ESP32P4_CLK_SOURCE,
                       TYPE_ESP32P4_CLK_SOURCE)

struct ESP32P4ClkSourceClass {
    InterfaceClass parent_class;

    /*
     * The exact period of @out, a clock output of @obj, as @out last took
     * it: false if @obj does not drive @out.
     */
    bool (*exact_period)(Object *obj, Clock *out, ESP32P4ClkPeriod *p);
};

/*
 * The exact period of the clock on @clk: as the device driving the clock
 * at its source gives it, while that rounds to @clk's period, and
 * otherwise @clk's period, taken as exact. A Clock's callbacks see the
 * source's new period from ClockUpdate on, and @clk's own old one in
 * ClockPreUpdate, as the source has already moved on: a block that counts
 * a clock across its changes keeps the period it took at the last
 * ClockUpdate (an ESP32P4ClkCount). A change of the exact period that
 * leaves the Clock's rounding as it was, less than a Clock unit, runs no
 * callbacks and is seen from the block's next look.
 */
static inline ESP32P4ClkPeriod esp32p4_clk_exact(Clock *clk)
{
    uint64_t period = clock_get(clk);
    Clock *root = clk;
    Object *owner;

    /*
     * A clock input has its source's period, unless clock_set_mul_div()
     * scales it on the way, which the source cannot know.
     */
    while (root->source) {
        root = root->source;
        if (root->multiplier != 1 || root->divider != 1) {
            return esp32p4_clk_period_exact(period);
        }
    }
    owner = OBJECT(root)->parent;
    if (period && owner &&
        object_dynamic_cast(owner, TYPE_ESP32P4_CLK_SOURCE)) {
        ESP32P4ClkSourceClass *k = ESP32P4_CLK_SOURCE_GET_CLASS(owner);
        ESP32P4ClkPeriod p;

        if (k->exact_period(owner, root, &p) &&
            esp32p4_clk_period_clock(p) == period) {
            return p;
        }
    }
    return esp32p4_clk_period_exact(period);
}

/*
 * A clock's cycles counted from one of its edges, at @origin_ns: cycle n
 * ends at origin_ns + n * period exactly, in the whole ns
 * esp32p4_clk_count_time() gives. A block that counts a clock across its
 * changes starts one at each change (esp32p4_clk_count_start() in its
 * ClockUpdate callback), its cycles beginning afresh from the change, and
 * keeps the cycles counted before.
 */
typedef struct ESP32P4ClkCount {
    ESP32P4ClkPeriod period;
    int64_t origin_ns;
} ESP32P4ClkCount;

extern const VMStateDescription vmstate_esp32p4_clk_count;

#define VMSTATE_ESP32P4_CLK_COUNT(field, state) \
    VMSTATE_STRUCT(field, state, 0, vmstate_esp32p4_clk_count, \
                   ESP32P4ClkCount)

/* Count @clk's cycles, at its current period, from an edge at @now. */
static inline void esp32p4_clk_count_start(ESP32P4ClkCount *c, Clock *clk,
                                           int64_t now)
{
    c->period = esp32p4_clk_exact(clk);
    c->origin_ns = now;
}

static inline bool esp32p4_clk_count_running(const ESP32P4ClkCount *c)
{
    return esp32p4_clk_period_running(c->period);
}

/* The cycles completed by @t: 0 before the origin and while stopped. */
static inline uint64_t esp32p4_clk_count_at(const ESP32P4ClkCount *c,
                                            int64_t t)
{
    if (t <= c->origin_ns) {
        return 0;
    }
    return esp32p4_clk_ns_cycles(c->period, t - c->origin_ns);
}

/*
 * The first whole ns by which cycle @n has completed: the origin for 0;
 * INT64_MAX while stopped, or if that is no earlier.
 */
static inline int64_t esp32p4_clk_count_time(const ESP32P4ClkCount *c,
                                             uint64_t n)
{
    int64_t ns = esp32p4_clk_cycles_ns(c->period, n);

    if (ns > INT64_MAX - c->origin_ns) {
        return INT64_MAX;
    }
    return c->origin_ns + ns;
}

#endif
