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
#include "qemu/host-utils.h"

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

#endif
