/*
 * RP2350 double-precision coprocessor (DCP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Double-precision coprocessor (DCP)", the
 * instruction encodings in pico-sdk's hardware/dcp_instr.inc.S, and the
 * canned sequences in hardware/dcp_canned.inc.S and pico_double's
 * double_fma_dcp.S, which fix the fixed-point formats the CPU side of each
 * sequence reads and writes.
 *
 * The datasheet gives the DCP's registers and an outline of each
 * instruction but not its internal datapath. The model below is a datapath
 * that gives every canned sequence the result the hardware gives: IEEE 754
 * round-to-nearest-even results, with denormal operands read as zero and
 * results below the normal range flushed to zero (the hardware's results in
 * pico-sdk's test/pico_float_test/m33.c show both). Instructions are only
 * meant to be used within the canned sequences; outside them the state is
 * undefined on hardware too.
 *
 * Internal formats:
 *  - A mantissa register holds a value as an unsigned fixed-point number
 *    with 62 fraction bits (Q62): an unpacked operand is 01 followed by its
 *    fraction field, with the low-order bits clear.
 *  - An exponent register is a 14-bit two's complement exponent biased by
 *    1023 for both precisions, so (xm, xe) is xm * 2^(xe - 1023 - 62).
 *  - A flag register holds sign, zero, infinity and NaN (bits 3..0).
 *  - Status bits 5:0 hold the alignment shift and bits 7:6 how |X|
 *    compares with |Y|, both set by ADD0; bit 8 is the engaged flag.
 *
 * The mantissa datapath operations (ADD1, SUB1 and the direct writes that
 * deliver a product, quotient or root) leave X alone when an operand is
 * infinite or NaN; the packed-result reads then resolve the special cases
 * for their operation from the flags, taking a NaN's payload from the
 * mantissa it was unpacked into.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/misc/rp2350_dcp.h"
#include "migration/vmstate.h"
#include "target/arm/cpu.h"

#define STATUS_SHIFT_MASK 0x3f
#define STATUS_CMP_SHIFT  6
#define STATUS_ENGAGED    (1u << 8)
#define STATUS_MASK       0x1ff
#define EXP_MASK          0x3fff
#define FLAG_MASK         0xf

/* Flag register bits */
#define F_NAN  (1u << 0)
#define F_INF  (1u << 1)
#define F_ZERO (1u << 2)
#define F_SIGN (1u << 3)

/* Status bits 7:6: how the magnitude of X compares with that of Y */
enum {
    CMP_EQ = 0,
    CMP_GT = 1,
    CMP_LT = 2,
    CMP_UN = 3,
};

#define DBL_BIAS   1023
#define FLT_REBIAS (1023 - 127)
#define Q          62

/* WXIC, WXUC, WXDC and WXFC put 2^52 + 2^32 in Y. */
#define CONV_OFFSET ((1ull << 52) + (1ull << 32))

/* RXVD's VERSION field; the datasheet does not give its value. */
#define DCP_VERSION 0

static bool is_cdp(uint32_t insn)
{
    return (insn & 0x0f000010) == 0x0e000000;
}

static bool is_mrc(uint32_t insn)
{
    return (insn & 0x0f100010) == 0x0e100010;
}

static bool is_mcrr(uint32_t insn)
{
    return (insn & 0x0ff00000) == 0x0c400000;
}

static bool is_mrrc(uint32_t insn)
{
    return (insn & 0x0ff00000) == 0x0c500000;
}

/* The "2" forms: the reads' P variants, which leave the engaged flag. */
static bool is_two(uint32_t insn)
{
    return extract32(insn, 28, 1);
}

static int32_t exp_get(uint32_t e)
{
    return sextract32(e, 0, 14);
}

static uint32_t exp_put(int32_t e)
{
    return e & EXP_MASK;
}

/* Flags of an operand that is infinite or NaN, so has no finite value. */
static bool nonfinite(uint32_t f)
{
    return f & (F_NAN | F_INF);
}

/* An operand's mantissa as the adder sees it: zero for a zero. */
static uint64_t adder_m(uint64_t m, uint32_t f)
{
    return (f & F_ZERO) ? 0 : m;
}

static int cmp_get(RP2350DCPInstance *d)
{
    return extract32(d->status, STATUS_CMP_SHIFT, 2);
}

/*
 * Unpack an IEEE double into a mantissa, exponent and flags. A denormal
 * reads as zero of the same sign.
 */
static void unpack_double(uint64_t v, uint64_t *m, uint32_t *e, uint32_t *f)
{
    uint32_t exp = extract64(v, 52, 11);
    uint64_t frac = extract64(v, 0, 52);

    *m = (1ull << Q) | (frac << 10);
    *e = exp;
    *f = (v >> 63) ? F_SIGN : 0;
    if (exp == 0) {
        *f |= F_ZERO;
    } else if (exp == 0x7ff) {
        *f |= frac ? F_NAN : F_INF;
    }
}

static void unpack_float(uint32_t v, uint64_t *m, uint32_t *e, uint32_t *f)
{
    uint32_t exp = extract32(v, 23, 8);
    uint32_t frac = extract32(v, 0, 23);

    *m = (1ull << Q) | ((uint64_t)frac << 39);
    *e = exp_put(exp + FLT_REBIAS);
    *f = (v >> 31) ? F_SIGN : 0;
    if (exp == 0) {
        *f |= F_ZERO;
    } else if (exp == 0xff) {
        *f |= frac ? F_NAN : F_INF;
    }
}

/* Y := 2^52 + 2^32, the offset the integer conversions add or subtract. */
static void set_y_offset(RP2350DCPInstance *d)
{
    d->ym = CONV_OFFSET << 10;
    d->ye = DBL_BIAS + 52;
    d->yf = 0;
}

/* X := an integer plus 2^52 + 2^32, exactly. */
static void set_x_int(RP2350DCPInstance *d, int64_t v)
{
    d->xm = (uint64_t)(CONV_OFFSET + v) << 10;
    d->xe = DBL_BIAS + 52;
    d->xf = 0;
}

/*
 * ADD0: compare |X| with |Y| (zero < finite < infinity; NaN unordered) and
 * record the shift that aligns the smaller with the larger.
 */
static void dcp_add0(RP2350DCPInstance *d)
{
    int32_t de = exp_get(d->xe) - exp_get(d->ye);
    uint32_t shift = MIN(de < 0 ? -de : de, STATUS_SHIFT_MASK);
    int rx, ry, cmp;

    if ((d->xf | d->yf) & F_NAN) {
        cmp = CMP_UN;
    } else {
        rx = (d->xf & F_INF) ? 2 : (d->xf & F_ZERO) ? 0 : 1;
        ry = (d->yf & F_INF) ? 2 : (d->yf & F_ZERO) ? 0 : 1;
        if (rx != ry) {
            cmp = rx > ry ? CMP_GT : CMP_LT;
        } else if (rx != 1 || (de == 0 && d->xm == d->ym)) {
            cmp = CMP_EQ;
        } else if (de != 0) {
            cmp = de > 0 ? CMP_GT : CMP_LT;
        } else {
            cmp = d->xm > d->ym ? CMP_GT : CMP_LT;
        }
    }
    d->status = deposit32(d->status, 0, 8, shift | cmp << STATUS_CMP_SHIFT);
}

/* Shift right, ORing the bits shifted out into bit 0. */
static uint64_t shift_jam(uint64_t m, uint32_t s)
{
    if (s == 0) {
        return m;
    }
    if (s >= 64) {
        return m != 0;
    }
    return (m >> s) | ((m & MAKE_64BIT_MASK(0, s)) != 0);
}

/*
 * ADD1 and SUB1: add or subtract the aligned smaller operand to or from
 * the larger, as magnitudes. The result's sign is that of the larger
 * operand, which the result reads work out from the flags and ADD0's
 * comparison; its exponent is the larger operand's.
 */
static void dcp_addsub(RP2350DCPInstance *d, bool sub)
{
    bool x_big = cmp_get(d) != CMP_LT;
    uint64_t xm = adder_m(d->xm, d->xf), ym = adder_m(d->ym, d->yf);
    uint64_t big = x_big ? xm : ym, small = x_big ? ym : xm;
    bool subtract = ((d->xf ^ d->yf) & F_SIGN) ? !sub : sub;

    if (nonfinite(d->xf) || nonfinite(d->yf)) {
        return;
    }
    small = shift_jam(small, d->status & STATUS_SHIFT_MASK);
    d->xm = subtract ? big - small : big + small;
    d->xe = x_big ? d->xe : d->ye;
}

/*
 * SQR0: make the exponent even, shifting the mantissa up a place when it
 * was odd so that it lies in [1, 4), then halve the exponent.
 */
static void dcp_sqr0(RP2350DCPInstance *d)
{
    int32_t e = exp_get(d->xe) - DBL_BIAS;

    if (e & 1) {
        d->xm <<= 1;
        e--;
    }
    d->xe = exp_put((e >> 1) + DBL_BIAS);
}

/* Move X's leading one to bit 62, keeping a sticky bit on a right shift. */
static void dcp_normalise(RP2350DCPInstance *d)
{
    int32_t e = exp_get(d->xe);
    int lz;

    if (d->xm == 0) {
        return;
    }
    if (d->xm >> 63) {
        d->xm = shift_jam(d->xm, 1);
        e++;
    } else {
        lz = clz64(d->xm) - 1;
        d->xm <<= lz;
        e -= lz;
    }
    d->xe = exp_put(e);
}

enum {
    ROUND_NEAREST_EVEN,
    ROUND_DOWN,
    ROUND_UP,
};

/* Round X's normalised mantissa to `bits` significant bits. */
static void dcp_round(RP2350DCPInstance *d, int bits, int mode)
{
    int drop = 63 - bits;
    uint64_t lsb = 1ull << drop, rem = d->xm & (lsb - 1), half = lsb >> 1;
    bool up;

    if (d->xm == 0) {
        return;
    }
    d->xm -= rem;
    switch (mode) {
    case ROUND_NEAREST_EVEN:
        up = rem > half || (rem == half && (d->xm & lsb));
        break;
    case ROUND_UP:
        up = rem != 0;
        break;
    default:
        up = false;
        break;
    }
    if (up) {
        d->xm += lsb;
        if (d->xm >> 63) {
            d->xm >>= 1;
            d->xe = exp_put(exp_get(d->xe) + 1);
        }
    }
}

/*
 * NTDC rounds the integer conversion so that X's own contribution is
 * truncated towards zero: up when X was subtracted from the larger offset
 * in Y, down otherwise.
 */
static int ntdc_mode(RP2350DCPInstance *d)
{
    bool subtract = (d->xf ^ d->yf) & F_SIGN;

    return subtract && cmp_get(d) == CMP_LT ? ROUND_UP : ROUND_DOWN;
}

/*
 * The direct mantissa writes that deliver a CPU-computed product,
 * quotient or root, and set X's exponent from X's and Y's. They leave X
 * alone when an operand has no finite value; a square root has only X as
 * its operand, Y being whatever an earlier sequence left there.
 */
static void dcp_write_result(RP2350DCPInstance *d, uint64_t m, int32_t e,
                             bool binary)
{
    if (nonfinite(d->xf) || (binary && nonfinite(d->yf))) {
        return;
    }
    d->xm = m;
    d->xe = exp_put(e);
}

/* The exponent of a product written as Q60 (the top of a Q62 x Q62). */
static int32_t mul_exp(RP2350DCPInstance *d)
{
    return exp_get(d->xe) + exp_get(d->ye) - DBL_BIAS + 2;
}

static int32_t div_exp(RP2350DCPInstance *d, int q)
{
    return exp_get(d->xe) - exp_get(d->ye) + DBL_BIAS + Q - q;
}

static int32_t sqrt_exp(RP2350DCPInstance *d, int q)
{
    return exp_get(d->xe) + Q - q;
}

/*
 * RYMR's reciprocal approximation: 1/y for the midpoint of the 1/256-wide
 * interval holding Y's mantissa, as a Q31 fraction.
 */
static uint32_t recip_approx(uint64_t ym)
{
    uint32_t i = extract64(ym, Q - 8, 8);

    return (1ull << 40) / (513 + 2 * i);
}

static uint64_t isqrt64(uint64_t v)
{
    uint64_t r = 0, bit = 1ull << 62;

    while (bit > v) {
        bit >>= 2;
    }
    while (bit) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

/*
 * RXMQ's reciprocal square root approximation: 1/sqrt(x) for the midpoint
 * of the 1/64-wide interval holding X's mantissa in [1, 4), as a Q31
 * fraction below 1: 2^31 / sqrt((2i + 1) / 128) = 2^4 sqrt(2^61 / (2i + 1)).
 */
static uint32_t rsqrt_approx(uint64_t xm)
{
    uint32_t i = extract64(xm, 56, 8);

    return isqrt64((1ull << 61) / (2 * i + 1)) << 4;
}

/* What a packed-result read produces before packing. */
typedef enum DCPClass {
    RES_FINITE,
    RES_ZERO,
    RES_INF,
    RES_NAN,      /* NaN with payload `m` */
    RES_DEFAULT_NAN,
} DCPClass;

typedef struct DCPResult {
    DCPClass cls;
    bool sign;
    uint64_t m;
    int32_t e;
} DCPResult;

enum {
    OP_ADD,
    OP_SUB,
    OP_MUL,
    OP_DIV,
    OP_SQRT,
    OP_GENERAL,
};

static DCPResult res(DCPClass cls, bool sign, uint64_t m, int32_t e)
{
    return (DCPResult) { .cls = cls, .sign = sign, .m = m, .e = e };
}

/* Resolve the special cases of operation `op` and give its result. */
static DCPResult dcp_result(RP2350DCPInstance *d, int op)
{
    bool sx = d->xf & F_SIGN, sy = d->yf & F_SIGN;
    uint32_t xf = d->xf, yf = d->yf;
    int32_t e = exp_get(d->xe);

    if (xf & F_NAN) {
        return res(RES_NAN, sx, d->xm, 0);
    }
    switch (op) {
    case OP_ADD:
    case OP_SUB:
        sy ^= op == OP_SUB;
        if (yf & F_NAN) {
            return res(RES_NAN, d->yf & F_SIGN, d->ym, 0);
        }
        if (xf & F_INF) {
            if ((yf & F_INF) && sx != sy) {
                return res(RES_DEFAULT_NAN, true, 0, 0);
            }
            return res(RES_INF, sx, 0, 0);
        }
        if (yf & F_INF) {
            return res(RES_INF, sy, 0, 0);
        }
        if (d->xm == 0) {
            return res(RES_ZERO, sx && sy, 0, 0);
        }
        return res(RES_FINITE, cmp_get(d) == CMP_LT ? sy : sx, d->xm, e);
    case OP_MUL:
        if (yf & F_NAN) {
            return res(RES_NAN, sy, d->ym, 0);
        }
        if (((xf | yf) & F_INF) && ((xf | yf) & F_ZERO)) {
            return res(RES_DEFAULT_NAN, true, 0, 0);
        }
        if ((xf | yf) & F_INF) {
            return res(RES_INF, sx ^ sy, 0, 0);
        }
        if ((xf | yf) & F_ZERO) {
            return res(RES_ZERO, sx ^ sy, 0, 0);
        }
        return res(RES_FINITE, sx ^ sy, d->xm, e);
    case OP_DIV:
        if (yf & F_NAN) {
            return res(RES_NAN, sy, d->ym, 0);
        }
        if ((xf & yf & F_INF) || (xf & yf & F_ZERO)) {
            return res(RES_DEFAULT_NAN, true, 0, 0);
        }
        if ((xf & F_INF) || (yf & F_ZERO)) {
            return res(RES_INF, sx ^ sy, 0, 0);
        }
        if ((yf & F_INF) || (xf & F_ZERO)) {
            return res(RES_ZERO, sx ^ sy, 0, 0);
        }
        return res(RES_FINITE, sx ^ sy, d->xm, e);
    case OP_SQRT:
        if (xf & F_ZERO) {
            return res(RES_ZERO, sx, 0, 0);
        }
        if (sx) {
            return res(RES_DEFAULT_NAN, true, 0, 0);
        }
        if (xf & F_INF) {
            return res(RES_INF, false, 0, 0);
        }
        return res(RES_FINITE, false, d->xm, e);
    default:
        if (xf & F_INF) {
            return res(RES_INF, sx, 0, 0);
        }
        if (xf & F_ZERO) {
            return res(RES_ZERO, sx, 0, 0);
        }
        return res(RES_FINITE, sx, d->xm, e);
    }
}

/*
 * Pack a result as a double. A finite result's mantissa has been
 * normalised and rounded; an exponent beyond the normal range gives
 * infinity, and one below it zero.
 */
static uint64_t pack_double(DCPResult r)
{
    uint64_t sign = (uint64_t)r.sign << 63;

    if (r.cls == RES_FINITE && r.m == 0) {
        r.cls = RES_ZERO;
    }
    if (r.cls == RES_FINITE && r.e >= 0x7ff) {
        r.cls = RES_INF;
    }
    if (r.cls == RES_FINITE && r.e <= 0) {
        r.cls = RES_ZERO;
    }
    switch (r.cls) {
    case RES_ZERO:
        return sign;
    case RES_INF:
        return sign | 0x7ff0000000000000ull;
    case RES_NAN:
        return sign | 0x7ff8000000000000ull | extract64(r.m, 10, 52);
    case RES_DEFAULT_NAN:
        return sign | 0x7ff8000000000000ull;
    default:
        return sign | (uint64_t)r.e << 52 | extract64(r.m, 10, 52);
    }
}

static uint32_t pack_float(DCPResult r)
{
    uint32_t sign = (uint32_t)r.sign << 31;
    int32_t e = r.e - FLT_REBIAS;

    if (r.cls == RES_FINITE && r.m == 0) {
        r.cls = RES_ZERO;
    }
    if (r.cls == RES_FINITE && e >= 0xff) {
        r.cls = RES_INF;
    }
    if (r.cls == RES_FINITE && e <= 0) {
        r.cls = RES_ZERO;
    }
    switch (r.cls) {
    case RES_ZERO:
        return sign;
    case RES_INF:
        return sign | 0x7f800000;
    case RES_NAN:
        return sign | 0x7fc00000 | extract64(r.m, 39, 23);
    case RES_DEFAULT_NAN:
        return sign | 0x7fc00000;
    default:
        return sign | e << 23 | extract64(r.m, 39, 23);
    }
}

/*
 * RDIC and RDUC: X holds the conversion operand plus 2^52 + 2^32, rounded
 * to an integer by NTDC or NRDC; take the offset off and saturate.
 */
static uint32_t read_int(RP2350DCPInstance *d, bool is_signed)
{
    int64_t lo = is_signed ? INT32_MIN : 0;
    int64_t hi = is_signed ? INT32_MAX : UINT32_MAX;
    bool neg = (cmp_get(d) == CMP_LT ? d->yf : d->xf) & F_SIGN;
    int32_t e = exp_get(d->xe) - DBL_BIAS;
    int64_t v;

    if (d->xf & F_NAN) {
        return 0;
    }
    if (d->xf & F_INF) {
        return (d->xf & F_SIGN) ? lo : hi;
    }
    if (d->xm == 0 || e < 0) {
        v = 0;
    } else if (e > 60) {
        return neg ? lo : hi;
    } else {
        v = d->xm >> (Q - e);
    }
    v = (neg ? -v : v) - (int64_t)CONV_OFFSET;
    return MIN(MAX(v, lo), hi);
}

static void set_engaged(RP2350DCPInstance *d)
{
    d->status |= STATUS_ENGAGED;
}

/* A final read clears the engaged flag; its P form leaves it. */
static void end_sequence(RP2350DCPInstance *d, uint32_t insn)
{
    if (!is_two(insn)) {
        d->status &= ~STATUS_ENGAGED;
    }
}

/* A mid-sequence read sets the engaged flag; its P form leaves it. */
static void mid_sequence(RP2350DCPInstance *d, uint32_t insn)
{
    if (!is_two(insn)) {
        set_engaged(d);
    }
}

/*
 * WEFD and REFD move the exponents, flags and status as a pair of words:
 * low word xe | status << 14 | xf << 28, high word ye | yf << 28. The flags
 * in the top nibbles are where double_fma_dcp.S tests for zero, infinity
 * and NaN operands (0x70000000).
 */
static uint64_t pack_efd(RP2350DCPInstance *d)
{
    uint32_t lo = d->xe | d->status << 14 | d->xf << 28;
    uint32_t hi = d->ye | d->yf << 28;

    return ((uint64_t)hi << 32) | lo;
}

static void unpack_efd(RP2350DCPInstance *d, uint32_t lo, uint32_t hi)
{
    d->xe = lo & EXP_MASK;
    d->status = (lo >> 14) & STATUS_MASK;
    d->xf = (lo >> 28) & FLAG_MASK;
    d->ye = hi & EXP_MASK;
    d->yf = (hi >> 28) & FLAG_MASK;
}

/*
 * RCMP: the processed comparison of X with Y for APSR.NZCV. N is the
 * engaged flag (so PCMP tests it), Z is X == Y, C is X >= Y and V is
 * "unordered", with C clear when unordered.
 */
static uint32_t read_cmp(RP2350DCPInstance *d)
{
    bool sx = d->xf & F_SIGN, sy = d->yf & F_SIGN;
    bool zx = d->xf & F_ZERO, zy = d->yf & F_ZERO;
    int cmp = cmp_get(d);
    bool eq, ge;

    if (cmp == CMP_UN) {
        eq = false;
        ge = false;
    } else if (zx && zy) {
        eq = true;
        ge = true;
    } else if (sx != sy) {
        eq = false;
        ge = !sx;
    } else {
        eq = cmp == CMP_EQ;
        ge = eq || ((cmp == CMP_GT) != sx);
    }
    return !!(d->status & STATUS_ENGAGED) << 31 | eq << 30 | ge << 29 |
           (cmp == CMP_UN) << 28;
}

static bool dcp_cdp(RP2350DCPInstance *d, uint32_t insn)
{
    uint32_t opc1 = extract32(insn, 20, 4), crm = extract32(insn, 0, 4);
    uint32_t opc2 = extract32(insn, 5, 3);

    if (is_two(insn) || extract32(insn, 12, 8)) {
        return false;
    }
    switch (opc1 << 8 | crm << 4 | opc2) {
    case 0x000: /* INIT: zero all registers, starting a canned sequence */
        memset(d, 0, sizeof(*d));
        break;
    case 0x010: /* ADD0 */
        dcp_add0(d);
        break;
    case 0x110: /* ADD1 */
        dcp_addsub(d, false);
        break;
    case 0x111: /* SUB1 */
        dcp_addsub(d, true);
        break;
    case 0x210: /* SQR0 */
        dcp_sqr0(d);
        break;
    case 0x820: /* NORM */
        dcp_normalise(d);
        break;
    case 0x821: /* NRDF */
        dcp_normalise(d);
        dcp_round(d, 24, ROUND_NEAREST_EVEN);
        break;
    case 0x801: /* NRDD */
        dcp_normalise(d);
        dcp_round(d, 53, ROUND_NEAREST_EVEN);
        break;
    case 0x802: /* NTDC */
        dcp_normalise(d);
        dcp_round(d, 53, ntdc_mode(d));
        break;
    case 0x803: /* NRDC */
        dcp_normalise(d);
        dcp_round(d, 53, ROUND_NEAREST_EVEN);
        break;
    default:
        return false;
    }
    set_engaged(d);
    return true;
}

static bool dcp_mcrr(RP2350DCPInstance *d, uint32_t insn, uint32_t lo,
                     uint32_t hi)
{
    uint32_t opc1 = extract32(insn, 4, 4), crm = extract32(insn, 0, 4);
    uint64_t v = ((uint64_t)hi << 32) | lo;

    if (is_two(insn)) {
        return false;
    }
    switch (opc1 << 4 | crm) {
    case 0x00: /* WXMD */
        d->xm = v;
        break;
    case 0x01: /* WYMD */
        d->ym = v;
        break;
    case 0x02: /* WEFD, which restores the engaged flag too */
        unpack_efd(d, lo, hi);
        return true;
    case 0x10: /* WXUP */
        unpack_double(v, &d->xm, &d->xe, &d->xf);
        break;
    case 0x11: /* WYUP */
        unpack_double(v, &d->ym, &d->ye, &d->yf);
        break;
    case 0x12: /* WXYU */
        unpack_float(lo, &d->xm, &d->xe, &d->xf);
        unpack_float(hi, &d->ym, &d->ye, &d->yf);
        break;
    case 0x20: /* WXMS: the low half of a product, as a sticky bit */
        if (!nonfinite(d->xf) && !nonfinite(d->yf)) {
            d->xm = v != 0;
        }
        break;
    case 0x30: /* WXMO: the high half of a product, Q60 */
        dcp_write_result(d, v | (d->xm & 1), mul_exp(d), true);
        break;
    case 0x40: /* WXDD: a double quotient */
        dcp_write_result(d, v, div_exp(d, 57), true);
        break;
    case 0x50: /* WXDQ: a double square root */
        dcp_write_result(d, v, sqrt_exp(d, 60), false);
        break;
    case 0x60: /* WXUC */
        set_x_int(d, lo);
        set_y_offset(d);
        break;
    case 0x70: /* WXIC */
        set_x_int(d, (int32_t)lo);
        set_y_offset(d);
        break;
    case 0x80: /* WXDC */
        unpack_double(v, &d->xm, &d->xe, &d->xf);
        set_y_offset(d);
        break;
    case 0x92: /* WXFC */
        unpack_float(lo, &d->xm, &d->xe, &d->xf);
        set_y_offset(d);
        break;
    case 0xa0: /* WXFM: a float product, Q60 */
        dcp_write_result(d, v, mul_exp(d), true);
        break;
    case 0xb0: /* WXFD: a float quotient */
        dcp_write_result(d, v, div_exp(d, 61), true);
        break;
    case 0xc0: /* WXFQ: a float square root */
        dcp_write_result(d, v, sqrt_exp(d, 61), false);
        break;
    default:
        return false;
    }
    set_engaged(d);
    return true;
}

/* The operations of RDFA, RDFS, RDFM, RDFD, RDFQ and RDFG, by opc2 */
static const int float_ops[] = {
    OP_ADD, OP_SUB, OP_MUL, OP_DIV, OP_SQRT, OP_GENERAL,
};

static bool dcp_mrc(RP2350DCPInstance *d, uint32_t insn, uint64_t *result)
{
    uint32_t opc1 = extract32(insn, 21, 3), crn = extract32(insn, 16, 4);
    uint32_t crm = extract32(insn, 0, 4), opc2 = extract32(insn, 5, 3);

    if (opc1 != 0 || crn != 0) {
        return false;
    }
    switch (crm << 4 | opc2) {
    case 0x00: /* RXVD */
        *result = d->xf << 28 | DCP_VERSION;
        break;
    case 0x01: /* RCMP */
        *result = read_cmp(d);
        break;
    case 0x20 ... 0x25: /* RDFA, RDFS, RDFM, RDFD, RDFQ, RDFG */
        *result = pack_float(dcp_result(d, float_ops[opc2]));
        break;
    case 0x30: /* RDIC */
        *result = read_int(d, true);
        break;
    case 0x31: /* RDUC */
        *result = read_int(d, false);
        break;
    default:
        return false;
    }
    end_sequence(d, insn);
    return true;
}

static bool dcp_mrrc(RP2350DCPInstance *d, uint32_t insn, uint64_t *result)
{
    uint32_t opc1 = extract32(insn, 4, 4), crm = extract32(insn, 0, 4);

    switch (crm) {
    case 8: /* RXMD */
        if (opc1) {
            return false;
        }
        *result = d->xm;
        return true;
    case 9: /* RYMD, the last read of a save: engaged = 0 */
        if (opc1) {
            return false;
        }
        *result = d->ym;
        end_sequence(d, insn);
        return true;
    case 10: /* REFD */
        if (opc1) {
            return false;
        }
        *result = pack_efd(d);
        return true;
    case 4: /* RXMS: xm as Q(62 - s) */
        *result = d->xm >> opc1;
        mid_sequence(d, insn);
        return true;
    case 5: /* RYMS: ym as Q(62 - s) */
        *result = d->ym >> opc1;
        mid_sequence(d, insn);
        return true;
    case 1:
        switch (opc1) {
        case 1: /* RXYH: low word xm hi, high word ym hi */
            *result = (d->ym & 0xffffffff00000000ull) | d->xm >> 32;
            break;
        case 2: /* RYMR: low word 1/y approximation, high word ym Q31 */
            *result = (d->ym >> 31) << 32 | recip_approx(d->ym);
            break;
        case 4: /* RXMQ: low word 1/sqrt(x) approximation, high word xm hi */
            *result = (d->xm & 0xffffffff00000000ull) | rsqrt_approx(d->xm);
            break;
        default:
            return false;
        }
        mid_sequence(d, insn);
        return true;
    case 0:
        switch (opc1) {
        case 1: /* RDDA */
            *result = pack_double(dcp_result(d, OP_ADD));
            break;
        case 3: /* RDDS */
            *result = pack_double(dcp_result(d, OP_SUB));
            break;
        case 5: /* RDDM */
            *result = pack_double(dcp_result(d, OP_MUL));
            break;
        case 7: /* RDDD */
            *result = pack_double(dcp_result(d, OP_DIV));
            break;
        case 9: /* RDDQ */
            *result = pack_double(dcp_result(d, OP_SQRT));
            break;
        case 11: /* RDDG */
            *result = pack_double(dcp_result(d, OP_GENERAL));
            break;
        default:
            return false;
        }
        end_sequence(d, insn);
        return true;
    }
    return false;
}

/* [spec:nuos:req:emu.dcp-state] */
/* [spec:nuos:req:emu.dcp-arith] */
static ARMMCoprocResult rp2350_dcp_op(void *opaque, ARMCPU *cpu,
                                      uint32_t insn, uint32_t rt,
                                      uint32_t rt2, bool secure,
                                      uint64_t *result)
{
    RP2350DCPCore *c = opaque;
    int cp = extract32(insn, 8, 4);
    RP2350DCPInstance *d;
    bool ok = false;

    /* cp4 is this state's instance; cp5 is the Non-secure one, for Secure. */
    if (cp == 5 && !secure) {
        return ARM_M_COPROC_UNDEF;
    }
    d = &c->inst[cp == 5 ? 1 : !secure];

    if (is_cdp(insn)) {
        ok = dcp_cdp(d, insn);
    } else if (is_mcrr(insn)) {
        ok = dcp_mcrr(d, insn, rt, rt2);
    } else if (is_mrrc(insn)) {
        ok = dcp_mrrc(d, insn, result);
    } else if (is_mrc(insn)) {
        ok = dcp_mrc(d, insn, result);
    }
    if (ok) {
        return ARM_M_COPROC_OK;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dcp: core %d: instruction "
                  "0x%08x is not a DCP instruction\n", c->core, insn);
    return ARM_M_COPROC_UNDEF;
}

void rp2350_dcp_attach(RP2350DCPState *s, int core, ARMCPU *cpu)
{
    arm_m_set_coprocessor(cpu, 4, rp2350_dcp_op, &s->core[core]);
    arm_m_set_coprocessor(cpu, 5, rp2350_dcp_op, &s->core[core]);
}

void rp2350_dcp_reset_core(RP2350DCPState *s, int core)
{
    memset(s->core[core].inst, 0, sizeof(s->core[core].inst));
}

static void rp2350_dcp_hold_reset(Object *obj, ResetType type)
{
    RP2350DCPState *s = RP2350_DCP(obj);
    int i;

    for (i = 0; i < RP2350_DCP_CORES; i++) {
        rp2350_dcp_reset_core(s, i);
    }
}

static void rp2350_dcp_init(Object *obj)
{
    RP2350DCPState *s = RP2350_DCP(obj);
    int i;

    for (i = 0; i < RP2350_DCP_CORES; i++) {
        s->core[i].dcp = s;
        s->core[i].core = i;
    }
}

static const VMStateDescription vmstate_rp2350_dcp_instance = {
    .name = "rp2350-dcp-instance",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(xm, RP2350DCPInstance),
        VMSTATE_UINT64(ym, RP2350DCPInstance),
        VMSTATE_UINT32(xe, RP2350DCPInstance),
        VMSTATE_UINT32(ye, RP2350DCPInstance),
        VMSTATE_UINT32(xf, RP2350DCPInstance),
        VMSTATE_UINT32(yf, RP2350DCPInstance),
        VMSTATE_UINT32(status, RP2350DCPInstance),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_dcp_core = {
    .name = "rp2350-dcp-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(inst, RP2350DCPCore, 2, 1,
                             vmstate_rp2350_dcp_instance, RP2350DCPInstance),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_dcp = {
    .name = TYPE_RP2350_DCP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, RP2350DCPState, RP2350_DCP_CORES, 1,
                             vmstate_rp2350_dcp_core, RP2350DCPCore),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_dcp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_dcp_hold_reset;
    dc->vmsd = &vmstate_rp2350_dcp;
}

static const TypeInfo rp2350_dcp_info = {
    .name          = TYPE_RP2350_DCP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350DCPState),
    .instance_init = rp2350_dcp_init,
    .class_init    = rp2350_dcp_class_init,
};

static void rp2350_dcp_register_types(void)
{
    type_register_static(&rp2350_dcp_info);
}
type_init(rp2350_dcp_register_types)
