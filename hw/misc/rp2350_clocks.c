/*
 * RP2350 clock generation: CLOCKS, XOSC, PLL_SYS/PLL_USB and TICKS
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Clocks", "Crystal oscillator (XOSC)",
 * "PLL", "Tick generators" and "DORMANT state".
 *
 * Every clock is a QEMU Clock whose frequency follows the configuration:
 *
 *  - XOSC outputs its crystal (the xtal-hz property) once it is enabled
 *    and STABLE, STARTUP.DELAY * 256 crystal periods after it starts, and
 *    nothing while it is disabled or DORMANT. CTRL's ENABLE and FREQ_RANGE
 *    take only their valid codes; another value keeps the setting in force
 *    and sets BADWRITE. COUNT counts down at its frequency while it runs.
 *  - Each PLL's output, while it is locked and its post dividers are
 *    powered, is (FREF / REFDIV) * FBDIV / (POSTDIV1 * POSTDIV2), FREF
 *    being the XOSC; BYPASS passes FREF through. The VCO locks
 *    PLL_LOCK_REF_CYCLES cycles of FREF / REFDIV after it is powered with
 *    a valid configuration and a running reference; the datasheet gives
 *    no lock time, and this is of the order of an integer-N PLL's. Losing
 *    lock sets CS.LOCK_N and INTR.LOCK_N_STICKY.
 *  - Each CLOCKS generator runs from the source its glitchless and
 *    auxiliary muxes select, divided by DIV (integer, and fractional on
 *    the generators with a FRAC field; an integer part of 0 divides by
 *    its maximum plus one), and gated by ENABLE and KILL where it has
 *    them. A generator whose source is stopped is stopped. A glitchless
 *    mux completes a switch (SELECTED shows the source) only once the new
 *    source is running; until then its output is stopped. Enabling and
 *    switching take effect at once rather than after two source cycles.
 *  - Each TICKS generator ticks every CYCLES clk_ref cycles while it is
 *    enabled.
 *
 * The frequency counter (FC0) waits FC0_DELAY clk_ref cycles, then for
 * the clock under test to run (WAITING), then counts its edges for the
 * test interval it derives from FC0_REF_KHZ and FC0_INTERVAL, timed by
 * clk_ref as it actually runs, so a wrong FC0_REF_KHZ scales the result
 * as on hardware. The edges follow every change of the clock's rate; one
 * that stops during the count sets DIED. The count's timing is taken
 * from clk_ref's rate when each phase starts. The clk_sys resuscitation
 * circuit switches clk_sys to clk_ref once clk_sys has been stopped for
 * CLK_SYS_RESUS_CTRL.TIMEOUT clk_ref cycles, raising CLOCKS_IRQ. The
 * per-destination clock gates (WAKE_EN and SLEEP_EN) are not modelled:
 * every destination's clock is enabled, and ENABLED reports WAKE_EN. The
 * DFTCLK test muxes are not modelled.
 *
 * DORMANT. Writing the DORMANT keyword to XOSC.DORMANT stops the crystal
 * oscillator until a wake event: the GPIO banks' dormant_wake interrupt
 * or the AON timer alarm, both arriving on the "dormant-wake" input. The
 * oscillator then restarts and its output stays stopped until it is
 * stable again. The PLLs are not halted by DORMANT, but their reference
 * stops with the XOSC, so they lose lock and the clocks running from
 * them stop too.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_clocks.h"
#include "migration/vmstate.h"

/* a * num / den in clock period units, saturating at "too slow to run". */
static uint64_t period_scale(uint64_t a, uint64_t num, uint64_t den)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, a, num);
    divu128(&lo, &hi, den);
    return hi ? UINT64_MAX : lo;
}

static uint64_t rp2350_clkregs_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350ClkRegsState *s = opaque;
    RP2350ClkRegsClass *c = RP2350_CLKREGS_GET_CLASS(s);
    unsigned reg = rp2350_atomic_reg(addr) / 4;

    if (reg >= c->nregs) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of bad offset 0x%"
                      HWADDR_PRIx "\n", object_get_typename(OBJECT(s)), addr);
        return 0;
    }
    return c->read ? c->read(s, reg) : s->regs[reg];
}

static void rp2350_clkregs_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    RP2350ClkRegsState *s = opaque;
    RP2350ClkRegsClass *c = RP2350_CLKREGS_GET_CLASS(s);
    unsigned reg = rp2350_atomic_reg(addr) / 4;
    uint32_t mask, v;

    if (reg >= c->nregs) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to bad offset 0x%"
                      HWADDR_PRIx "\n", object_get_typename(OBJECT(s)), addr);
        return;
    }
    if (c->write && c->write(s, reg, addr, value)) {
        return;
    }
    mask = c->wmask ? c->wmask[reg] : UINT32_MAX;
    v = rp2350_atomic_apply(addr, s->regs[reg], value);
    s->regs[reg] = (s->regs[reg] & ~mask) | (v & mask);
    if (c->written) {
        c->written(s, reg);
    }
}

static const MemoryRegionOps rp2350_clkregs_ops = {
    .read = rp2350_clkregs_read,
    .write = rp2350_clkregs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_clkregs_hold_reset(Object *obj, ResetType type)
{
    RP2350ClkRegsState *s = RP2350_CLKREGS(obj);
    RP2350ClkRegsClass *c = RP2350_CLKREGS_GET_CLASS(s);

    memset(s->regs, 0, sizeof(s->regs));
    if (c->reset) {
        memcpy(s->regs, c->reset, c->nregs * sizeof(uint32_t));
    }
    if (c->reset_hold) {
        c->reset_hold(s);
    }
}

static void rp2350_clkregs_init(Object *obj)
{
    RP2350ClkRegsState *s = RP2350_CLKREGS(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_clkregs_ops, s,
                          object_get_typename(obj), RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void rp2350_clkregs_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_clkregs_hold_reset;
}

/* CLOCKS */

#define CLK_STRIDE         3 /* CTRL, DIV, SELECTED */
#define R_CLK_CTRL(n)      ((n) * CLK_STRIDE)
#define R_CLK_DIV(n)       ((n) * CLK_STRIDE + 1)
#define R_CLK_SELECTED(n)  ((n) * CLK_STRIDE + 2)
#define R_DFTCLK_XOSC_CTRL (0x78 / 4)
#define R_DFTCLK_LPOSC_CTRL (0x80 / 4)
#define R_RESUS_CTRL       (0x84 / 4)
#define R_RESUS_STATUS     (0x88 / 4)
#define R_FC0_REF_KHZ      (0x8c / 4)
#define R_FC0_MIN_KHZ      (0x90 / 4)
#define R_FC0_MAX_KHZ      (0x94 / 4)
#define R_FC0_DELAY        (0x98 / 4)
#define R_FC0_INTERVAL     (0x9c / 4)
#define R_FC0_SRC          (0xa0 / 4)
#define R_FC0_STATUS       (0xa4 / 4)
#define R_FC0_RESULT       (0xa8 / 4)
#define R_WAKE_EN0         (0xac / 4)
#define R_WAKE_EN1         (0xb0 / 4)
#define R_SLEEP_EN0        (0xb4 / 4)
#define R_SLEEP_EN1        (0xb8 / 4)
#define R_ENABLED0         (0xbc / 4)
#define R_ENABLED1         (0xc0 / 4)
#define R_INTR             (0xc4 / 4)
#define R_INTE             (0xc8 / 4)
#define R_INTF             (0xcc / 4)
#define R_INTS             (0xd0 / 4)
#define CLOCKS_NREGS       (R_INTS + 1)

#define CTRL_ENABLED       (1u << 28)
#define CTRL_ENABLE        (1u << 11)
#define CTRL_KILL          (1u << 10)
#define CTRL_AUXSRC_SHIFT  5

#define RESUS_CTRL_CLEAR   (1u << 16)
#define RESUS_CTRL_FRCE    (1u << 12)
#define RESUS_CTRL_ENABLE  (1u << 8)
#define RESUS_CTRL_TIMEOUT 0xffu

#define FC0_STATUS_PASS    (1u << 0)
#define FC0_STATUS_DONE    (1u << 4)
#define FC0_STATUS_RUNNING (1u << 8)
#define FC0_STATUS_WAITING (1u << 12)
#define FC0_STATUS_FAIL    (1u << 16)
#define FC0_STATUS_SLOW    (1u << 20)
#define FC0_STATUS_FAST    (1u << 24)
#define FC0_STATUS_DIED    (1u << 28)

/* The frequency counter's progress. */
enum {
    FC0_IDLE,
    FC0_DELAY,
    FC0_WAITING,
    FC0_COUNTING,
};

const char *const rp2350_clock_names[RP2350_NUM_CLOCKS] = {
    [RP2350_CLK_GPOUT0] = "clk-gpout0",
    [RP2350_CLK_GPOUT1] = "clk-gpout1",
    [RP2350_CLK_GPOUT2] = "clk-gpout2",
    [RP2350_CLK_GPOUT3] = "clk-gpout3",
    [RP2350_CLK_REF] = "clk-ref",
    [RP2350_CLK_SYS] = "clk-sys",
    [RP2350_CLK_PERI] = "clk-peri",
    [RP2350_CLK_HSTX] = "clk-hstx",
    [RP2350_CLK_USB] = "clk-usb",
    [RP2350_CLK_ADC] = "clk-adc",
};

const char *const rp2350_clock_source_names[RP2350_NUM_CLKSRC] = {
    [RP2350_CLKSRC_ROSC] = "rosc",
    [RP2350_CLKSRC_ROSC_PH] = "rosc-ph",
    [RP2350_CLKSRC_XOSC] = "xosc",
    [RP2350_CLKSRC_LPOSC] = "lposc",
    [RP2350_CLKSRC_PLL_SYS] = "pll-sys",
    [RP2350_CLKSRC_PLL_USB] = "pll-usb",
    [RP2350_CLKSRC_GPIN0] = "gpin0",
    [RP2350_CLKSRC_GPIN1] = "gpin1",
    [RP2350_CLKSRC_PLL_USB_REF] = "pll-usb-ref",
    [RP2350_CLKSRC_OTP] = "otp",
};

/*
 * A mux input: a clock source (RP2350_CLKSRC_*), a generator's output
 * (SRC_CLK + RP2350_CLK_*), or nothing.
 */
#define SRC_NONE           (-1)
#define SRC_CLK            RP2350_NUM_CLKSRC
#define SRC_GEN(n)         (SRC_CLK + (n))

/* The auxiliary mux inputs of each generator, by AUXSRC value. */
static const int8_t gpout_aux[2][16] = {
    {
        RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1,
        RP2350_CLKSRC_PLL_USB, RP2350_CLKSRC_PLL_USB_REF,
        RP2350_CLKSRC_ROSC, RP2350_CLKSRC_XOSC, RP2350_CLKSRC_LPOSC,
        SRC_GEN(RP2350_CLK_SYS), SRC_GEN(RP2350_CLK_USB),
        SRC_GEN(RP2350_CLK_ADC), SRC_GEN(RP2350_CLK_REF),
        SRC_GEN(RP2350_CLK_PERI), SRC_GEN(RP2350_CLK_HSTX),
        RP2350_CLKSRC_OTP, SRC_NONE,
    },
    /* clk_gpout2 and clk_gpout3 take the ROSC's phase-shifted output. */
    {
        RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1,
        RP2350_CLKSRC_PLL_USB, RP2350_CLKSRC_PLL_USB_REF,
        RP2350_CLKSRC_ROSC_PH, RP2350_CLKSRC_XOSC, RP2350_CLKSRC_LPOSC,
        SRC_GEN(RP2350_CLK_SYS), SRC_GEN(RP2350_CLK_USB),
        SRC_GEN(RP2350_CLK_ADC), SRC_GEN(RP2350_CLK_REF),
        SRC_GEN(RP2350_CLK_PERI), SRC_GEN(RP2350_CLK_HSTX),
        RP2350_CLKSRC_OTP, SRC_NONE,
    },
};

static const int8_t ref_aux[4] = {
    RP2350_CLKSRC_PLL_USB, RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1,
    RP2350_CLKSRC_PLL_USB_REF,
};

/* clk_ref's glitchless mux; input 1 is its auxiliary mux. */
static const int8_t ref_glitchless[4] = {
    RP2350_CLKSRC_ROSC_PH, SRC_NONE, RP2350_CLKSRC_XOSC, RP2350_CLKSRC_LPOSC,
};

static const int8_t sys_aux[8] = {
    RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_PLL_USB, RP2350_CLKSRC_ROSC,
    RP2350_CLKSRC_XOSC, RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1,
    SRC_NONE, SRC_NONE,
};

static const int8_t peri_aux[8] = {
    SRC_GEN(RP2350_CLK_SYS), RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_PLL_USB,
    RP2350_CLKSRC_ROSC_PH, RP2350_CLKSRC_XOSC, RP2350_CLKSRC_GPIN0,
    RP2350_CLKSRC_GPIN1, SRC_NONE,
};

static const int8_t hstx_aux[8] = {
    SRC_GEN(RP2350_CLK_SYS), RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_PLL_USB,
    RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1, SRC_NONE, SRC_NONE, SRC_NONE,
};

/* clk_usb and clk_adc. */
static const int8_t usb_aux[8] = {
    RP2350_CLKSRC_PLL_USB, RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_ROSC_PH,
    RP2350_CLKSRC_XOSC, RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1,
    SRC_NONE, SRC_NONE,
};

/* The clocks FC0_SRC selects for the frequency counter. */
static const int8_t fc0_src[0x11] = {
    SRC_NONE, RP2350_CLKSRC_PLL_SYS, RP2350_CLKSRC_PLL_USB,
    RP2350_CLKSRC_ROSC, RP2350_CLKSRC_ROSC_PH, RP2350_CLKSRC_XOSC,
    RP2350_CLKSRC_GPIN0, RP2350_CLKSRC_GPIN1, SRC_GEN(RP2350_CLK_REF),
    SRC_GEN(RP2350_CLK_SYS), SRC_GEN(RP2350_CLK_PERI),
    SRC_GEN(RP2350_CLK_USB), SRC_GEN(RP2350_CLK_ADC),
    SRC_GEN(RP2350_CLK_HSTX), RP2350_CLKSRC_LPOSC, RP2350_CLKSRC_OTP,
    /* pll_usb_clksrc_primary_dft, a test path, is not modelled. */
    SRC_NONE,
};

/*
 * Each generator's divider: the width of DIV.INT (bits from 16) and
 * whether it has DIV.FRAC; its writable CTRL bits; whether it has ENABLE
 * and KILL (every generator but clk_ref and clk_sys).
 */
static const struct {
    uint8_t int_bits;
    bool frac;
    uint32_t ctrl_wmask;
} clk_info[RP2350_NUM_CLOCKS] = {
    [RP2350_CLK_GPOUT0] = { 16, true, 0x00131de0 },
    [RP2350_CLK_GPOUT1] = { 16, true, 0x00131de0 },
    [RP2350_CLK_GPOUT2] = { 16, true, 0x00131de0 },
    [RP2350_CLK_GPOUT3] = { 16, true, 0x00131de0 },
    [RP2350_CLK_REF] = { 8, false, 0x00000063 },
    [RP2350_CLK_SYS] = { 16, true, 0x000000e1 },
    [RP2350_CLK_PERI] = { 2, false, 0x00000ce0 },
    [RP2350_CLK_HSTX] = { 2, false, 0x00130ce0 },
    [RP2350_CLK_USB] = { 4, false, 0x00130ce0 },
    [RP2350_CLK_ADC] = { 4, false, 0x00130ce0 },
};

static bool clk_has_enable(int n)
{
    return n != RP2350_CLK_REF && n != RP2350_CLK_SYS;
}

static uint32_t clocks_reset[CLOCKS_NREGS];
static uint32_t clocks_wmask[CLOCKS_NREGS];

static uint32_t clk_aux(RP2350ClocksState *s, int n)
{
    return s->parent_obj.regs[R_CLK_CTRL(n)] >> CTRL_AUXSRC_SHIFT;
}

/* The input generator n's muxes select. */
static int clk_source(RP2350ClocksState *s, int n)
{
    uint32_t ctrl = s->parent_obj.regs[R_CLK_CTRL(n)];

    switch (n) {
    case RP2350_CLK_REF:
        if ((ctrl & 3) == 1) {
            return ref_aux[clk_aux(s, n) & 3];
        }
        return ref_glitchless[ctrl & 3];
    case RP2350_CLK_SYS:
        if (ctrl & 1) {
            return sys_aux[clk_aux(s, n) & 7];
        }
        return SRC_GEN(RP2350_CLK_REF);
    case RP2350_CLK_PERI:
        return peri_aux[clk_aux(s, n) & 7];
    case RP2350_CLK_HSTX:
        return hstx_aux[clk_aux(s, n) & 7];
    case RP2350_CLK_USB:
    case RP2350_CLK_ADC:
        return usb_aux[clk_aux(s, n) & 7];
    default:
        return gpout_aux[n >= RP2350_CLK_GPOUT2][clk_aux(s, n) & 0xf];
    }
}

static uint64_t source_period(RP2350ClocksState *s, int src)
{
    if (src == SRC_NONE) {
        return 0;
    }
    if (src >= SRC_CLK) {
        return clock_get(s->clk[src - SRC_CLK]);
    }
    return clock_get(s->src[src]);
}

/* The divisor of generator n in 16.16 fixed point. */
static uint64_t clk_divisor(RP2350ClocksState *s, int n)
{
    uint32_t div = s->parent_obj.regs[R_CLK_DIV(n)];
    uint64_t integer = extract32(div, 16, clk_info[n].int_bits);

    if (!integer) {
        integer = 1ull << clk_info[n].int_bits;
    }
    return (integer << 16) | (clk_info[n].frac ? div & 0xffff : 0);
}

/*
 * Generator n's output from its source's period: divided, while it runs
 * (the generators with an enable only once ENABLED).
 */
static uint64_t clk_period(RP2350ClocksState *s, int n, uint64_t src)
{
    if (!src) {
        return 0;
    }
    if (clk_has_enable(n) && !(s->enabled & BIT(n))) {
        return 0;
    }
    return period_scale(src, clk_divisor(s, n), 1u << 16);
}

/*
 * ENABLED follows ENABLE while the generator's source runs, since the
 * enable is synchronised to the source clock; KILL stops it at once.
 */
static void clk_update_enabled(RP2350ClocksState *s, int n, uint64_t src)
{
    uint32_t ctrl = s->parent_obj.regs[R_CLK_CTRL(n)];

    if (ctrl & CTRL_KILL) {
        s->enabled &= ~BIT(n);
    } else if (src) {
        s->enabled = deposit32(s->enabled, n, 1, !!(ctrl & CTRL_ENABLE));
    }
}

static void clocks_update_irq(RP2350ClocksState *s)
{
    uint32_t *regs = s->parent_obj.regs;

    qemu_set_irq(s->irq, (s->resussed | regs[R_INTF]) & regs[R_INTE] & 1);
}

static void fc0_source_changed(RP2350ClocksState *s);
static void fc0_cb(void *opaque);

/*
 * Arm or disarm the resuscitation timer: it runs while resus is enabled
 * and clk_sys, as its source and divider make it, is stopped, and only
 * while clk_ref runs to time it.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void clocks_resus_arm(RP2350ClocksState *s, uint64_t sys)
{
    uint32_t ctrl = s->parent_obj.regs[R_RESUS_CTRL];
    uint64_t ref = clock_get(s->clk[RP2350_CLK_REF]);
    int64_t when;

    if (sys || s->resussed || !(ctrl & RESUS_CTRL_ENABLE) ||
        (ctrl & RESUS_CTRL_CLEAR) || !ref) {
        timer_del(s->resus_timer);
        return;
    }
    if (timer_pending(s->resus_timer)) {
        return;
    }
    when = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
           clock_ticks_to_ns(s->clk[RP2350_CLK_REF],
                             (ctrl & RESUS_CTRL_TIMEOUT) + 1);
    timer_mod(s->resus_timer, when);
}

/*
 * Recompute every generator, in dependency order: clk_ref, then clk_sys
 * (which may run from clk_ref), then the clocks that may run from
 * clk_sys, then the GPOUTs, which may run from any of them.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void clocks_update(RP2350ClocksState *s)
{
    static const int order[RP2350_NUM_CLOCKS] = {
        RP2350_CLK_REF, RP2350_CLK_SYS, RP2350_CLK_PERI, RP2350_CLK_HSTX,
        RP2350_CLK_USB, RP2350_CLK_ADC, RP2350_CLK_GPOUT0,
        RP2350_CLK_GPOUT1, RP2350_CLK_GPOUT2, RP2350_CLK_GPOUT3,
    };
    int i;

    for (i = 0; i < RP2350_NUM_CLOCKS; i++) {
        int n = order[i];
        uint64_t src = source_period(s, clk_source(s, n));
        uint64_t period;

        clk_update_enabled(s, n, src);
        period = clk_period(s, n, src);
        if (n == RP2350_CLK_SYS) {
            clocks_resus_arm(s, period);
            if (s->resussed) {
                period = clock_get(s->clk[RP2350_CLK_REF]);
            }
        }
        clock_update(s->clk[n], period);
    }
    fc0_source_changed(s);
    if (s->fc0_stalled && clock_get(s->clk[RP2350_CLK_REF])) {
        /* The phase clk_ref's stop held up ends now. */
        s->fc0_end_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        fc0_cb(s);
    }
}

static void clocks_resus_cb(void *opaque)
{
    RP2350ClocksState *s = opaque;

    s->resussed = true;
    clocks_update(s);
    clocks_update_irq(s);
}

static void clocks_src_changed(void *opaque, ClockEvent event)
{
    clocks_update(opaque);
}

/* The period of FC0's clock under test, 0 if it is stopped or NULL. */
static uint64_t fc0_test_period(RP2350ClocksState *s)
{
    uint32_t sel = s->parent_obj.regs[R_FC0_SRC];

    if (sel >= ARRAY_SIZE(fc0_src)) {
        return 0;
    }
    return source_period(s, fc0_src[sel]);
}

/* clk_ref cycles in the test interval: 2^INTERVAL / 1024 ms. */
static uint64_t fc0_interval_cycles(RP2350ClocksState *s)
{
    uint32_t *regs = s->parent_obj.regs;

    return ((uint64_t)regs[R_FC0_REF_KHZ] << regs[R_FC0_INTERVAL]) >> 10;
}

/*
 * Arm the counter's timer for when `cycles` clk_ref cycles from virtual
 * time `from` have passed.
 */
static void fc0_wait_ref(RP2350ClocksState *s, int64_t from, uint64_t cycles)
{
    Clock *ref = s->clk[RP2350_CLK_REF];
    uint64_t ns = clock_ticks_to_ns(ref, cycles);

    while (clock_is_enabled(ref) && clock_ns_to_ticks(ref, ns) < cycles) {
        ns++;
    }
    s->fc0_end_ns = from + ns;
    timer_mod(s->fc0_timer, s->fc0_end_ns);
}

/*
 * Count the test clock's edges up to virtual time `t`, no later than the
 * end of the interval, at its rate since the last time, in 32.32 fixed
 * point.
 */
static void fc0_accumulate(RP2350ClocksState *s, int64_t t)
{
    uint64_t lo = 0, hi;

    t = MIN(t, s->fc0_end_ns);
    if (t > s->fc0_last_ns && s->fc0_last_period) {
        hi = t - s->fc0_last_ns;
        divu128(&lo, &hi, s->fc0_last_period);
        s->fc0_edges += lo;
    }
    s->fc0_last_ns = MAX(t, s->fc0_last_ns);
}

/* The test interval starts at `t`, once the test clock is running. */
static void fc0_start_count(RP2350ClocksState *s, int64_t t)
{
    s->fc0_phase = FC0_COUNTING;
    s->fc0_edges = 0;
    s->fc0_last_period = fc0_test_period(s);
    s->fc0_last_ns = t;
    s->parent_obj.regs[R_FC0_STATUS] = FC0_STATUS_RUNNING;
    fc0_wait_ref(s, t, fc0_interval_cycles(s));
}

/*
 * Follow the test clock: a count waiting for it starts once it runs, and
 * a count in progress tallies its edges at each change of rate, noting
 * if it dies.
 */
static void fc0_source_changed(RP2350ClocksState *s)
{
    uint64_t period = fc0_test_period(s);

    switch (s->fc0_phase) {
    case FC0_WAITING:
        if (period) {
            fc0_start_count(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        break;
    case FC0_COUNTING:
        fc0_accumulate(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        if (s->fc0_last_period && !period) {
            s->fc0_died = true;
        }
        s->fc0_last_period = period;
        break;
    }
}

/* [spec:nuos:req:emu.clock-tree] */
static void fc0_finish(RP2350ClocksState *s)
{
    uint32_t *regs = s->parent_obj.regs;
    uint64_t n = fc0_interval_cycles(s);
    uint64_t edges, result = 0;
    uint32_t khz, st = FC0_STATUS_DONE;

    fc0_accumulate(s, s->fc0_end_ns);
    s->fc0_phase = FC0_IDLE;
    edges = s->fc0_edges >> 32;
    if (n) {
        /* The counter scales the edges by its idea of the interval. */
        result = muldiv64(edges, regs[R_FC0_REF_KHZ] * 32ull, n);
    }
    result = MIN(result, 0x3fffffffull);
    khz = result >> 5;
    if (khz < regs[R_FC0_MIN_KHZ]) {
        st |= FC0_STATUS_FAIL | FC0_STATUS_SLOW;
    } else if (khz > regs[R_FC0_MAX_KHZ]) {
        st |= FC0_STATUS_FAIL | FC0_STATUS_FAST;
    } else {
        st |= FC0_STATUS_PASS;
    }
    if (s->fc0_died) {
        st |= FC0_STATUS_DIED;
    }
    regs[R_FC0_RESULT] = result;
    regs[R_FC0_STATUS] = st;
}

/*
 * The end of the delay or of the test interval, counted in clk_ref
 * cycles and taken at the virtual time it falls due, however late the
 * timer runs: with clk_ref stopped, nothing ends until it runs again.
 */
static void fc0_cb(void *opaque)
{
    RP2350ClocksState *s = opaque;

    if (!clock_is_enabled(s->clk[RP2350_CLK_REF])) {
        s->fc0_stalled = true;
        return;
    }
    s->fc0_stalled = false;
    switch (s->fc0_phase) {
    case FC0_DELAY:
        /* A NULL source is counted at once: it has no clock to wait for. */
        if (!s->parent_obj.regs[R_FC0_SRC] || fc0_test_period(s)) {
            fc0_start_count(s, s->fc0_end_ns);
        } else {
            s->fc0_phase = FC0_WAITING;
            s->parent_obj.regs[R_FC0_STATUS] = FC0_STATUS_RUNNING |
                                                FC0_STATUS_WAITING;
        }
        break;
    case FC0_COUNTING:
        fc0_finish(s);
        break;
    }
}

/*
 * Writing FC0_SRC starts a count, even of no clock: FC0_DELAY clk_ref
 * cycles for the mux to settle, then, once the clock under test runs,
 * the test interval.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void fc0_start(RP2350ClocksState *s)
{
    uint32_t *regs = s->parent_obj.regs;

    if (regs[R_FC0_SRC] == 0x10 || regs[R_FC0_SRC] >= ARRAY_SIZE(fc0_src)) {
        qemu_log_mask(LOG_UNIMP, "rp2350-clocks: FC0_SRC 0x%x is not "
                      "modelled\n", regs[R_FC0_SRC]);
    }
    s->fc0_phase = FC0_DELAY;
    s->fc0_stalled = false;
    s->fc0_died = false;
    regs[R_FC0_STATUS] = FC0_STATUS_RUNNING;
    regs[R_FC0_RESULT] = 0;
    fc0_wait_ref(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), regs[R_FC0_DELAY]);
}

/* [spec:nuos:req:emu.clocks] */
/* [spec:nuos:req:emu.clock-tree] */
static uint32_t clocks_read(RP2350ClkRegsState *rs, unsigned reg)
{
    RP2350ClocksState *s = RP2350_CLOCKS(rs);
    uint32_t *regs = rs->regs;

    if (reg < RP2350_NUM_CLOCKS * CLK_STRIDE) {
        int n = reg / CLK_STRIDE;
        uint32_t ctrl = regs[R_CLK_CTRL(n)];

        switch (reg % CLK_STRIDE) {
        case 0:
            if (clk_has_enable(n) && (s->enabled & BIT(n))) {
                ctrl |= CTRL_ENABLED;
            }
            return ctrl;
        case 2:
            /*
             * clk_ref and clk_sys have glitchless muxes, whose SELECTED is
             * a one-hot of the source once the switch to it completes,
             * which takes the new source running. The others' read 1.
             */
            if (n == RP2350_CLK_REF || n == RP2350_CLK_SYS) {
                if (!source_period(s, clk_source(s, n))) {
                    return 0;
                }
                return 1u << (ctrl & (n == RP2350_CLK_REF ? 3 : 1));
            }
            return 1;
        }
        return regs[reg];
    }
    switch (reg) {
    case R_RESUS_STATUS:
    case R_INTR:
        return s->resussed;
    case R_INTS:
        return (s->resussed | regs[R_INTF]) & regs[R_INTE] & 1;
    case R_ENABLED0:
        return regs[R_WAKE_EN0];
    case R_ENABLED1:
        return regs[R_WAKE_EN1];
    }
    return regs[reg];
}

static void clocks_written(RP2350ClkRegsState *rs, unsigned reg)
{
    RP2350ClocksState *s = RP2350_CLOCKS(rs);
    uint32_t *regs = rs->regs;

    if (reg < RP2350_NUM_CLOCKS * CLK_STRIDE) {
        clocks_update(s);
        return;
    }
    switch (reg) {
    case R_DFTCLK_XOSC_CTRL ... R_DFTCLK_LPOSC_CTRL:
        if (regs[reg]) {
            qemu_log_mask(LOG_UNIMP, "rp2350-clocks: the DFTCLK test muxes "
                          "are not modelled\n");
        }
        break;
    case R_RESUS_CTRL:
        if (regs[reg] & RESUS_CTRL_CLEAR) {
            s->resussed = false;
        } else if (regs[reg] & RESUS_CTRL_FRCE) {
            s->resussed = true;
        }
        clocks_update(s);
        clocks_update_irq(s);
        break;
    case R_FC0_SRC:
        fc0_start(s);
        break;
    case R_WAKE_EN0 ... R_SLEEP_EN1:
        if (regs[reg] != UINT32_MAX) {
            qemu_log_mask(LOG_UNIMP, "rp2350-clocks: per-destination clock "
                          "gating is not modelled\n");
        }
        break;
    case R_INTE:
    case R_INTF:
        clocks_update_irq(s);
        break;
    }
}

/* [spec:nuos:req:emu.clock-tree] */
void rp2350_clocks_boot_rom_handoff(RP2350ClocksState *s)
{
    /* The ROM writes 0x54040000; only DIV.INT's eight bits take it. */
    s->parent_obj.regs[R_CLK_DIV(RP2350_CLK_REF)] =
        0x54040000 & clocks_wmask[R_CLK_DIV(RP2350_CLK_REF)];
    clocks_update(s);
}

static void clocks_reset_hold(RP2350ClkRegsState *rs)
{
    RP2350ClocksState *s = RP2350_CLOCKS(rs);

    timer_del(s->resus_timer);
    timer_del(s->fc0_timer);
    s->enabled = 0;
    s->resussed = false;
    s->fc0_phase = FC0_IDLE;
    s->fc0_stalled = false;
    s->fc0_died = false;
    s->fc0_end_ns = 0;
    s->fc0_last_period = 0;
    s->fc0_last_ns = 0;
    s->fc0_edges = 0;
}

/* A reset switches clk_ref and clk_sys back to the ROSC. */
static void rp2350_clocks_exit_reset(Object *obj, ResetType type)
{
    RP2350ClocksState *s = RP2350_CLOCKS(obj);

    clocks_update(s);
    clocks_update_irq(s);
}

static void rp2350_clocks_init(Object *obj)
{
    RP2350ClocksState *s = RP2350_CLOCKS(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    for (i = 0; i < RP2350_NUM_CLKSRC; i++) {
        s->src[i] = qdev_init_clock_in(dev, rp2350_clock_source_names[i],
                                       clocks_src_changed, s, ClockUpdate);
    }
    for (i = 0; i < RP2350_NUM_CLOCKS; i++) {
        s->clk[i] = qdev_init_clock_out(dev, rp2350_clock_names[i]);
    }
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->resus_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clocks_resus_cb, s);
    s->fc0_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fc0_cb, s);
}

static const VMStateDescription vmstate_rp2350_clocks = {
    .name = TYPE_RP2350_CLOCKS,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(parent_obj.regs, RP2350ClocksState,
                             RP2350_CLKREGS_MAX),
        VMSTATE_ARRAY_CLOCK(src, RP2350ClocksState, RP2350_NUM_CLKSRC),
        VMSTATE_ARRAY_CLOCK(clk, RP2350ClocksState, RP2350_NUM_CLOCKS),
        VMSTATE_UINT32(enabled, RP2350ClocksState),
        VMSTATE_BOOL(resussed, RP2350ClocksState),
        VMSTATE_TIMER_PTR(resus_timer, RP2350ClocksState),
        VMSTATE_TIMER_PTR(fc0_timer, RP2350ClocksState),
        VMSTATE_UINT32(fc0_phase, RP2350ClocksState),
        VMSTATE_BOOL(fc0_stalled, RP2350ClocksState),
        VMSTATE_BOOL(fc0_died, RP2350ClocksState),
        VMSTATE_INT64(fc0_end_ns, RP2350ClocksState),
        VMSTATE_UINT64(fc0_last_period, RP2350ClocksState),
        VMSTATE_INT64(fc0_last_ns, RP2350ClocksState),
        VMSTATE_UINT64(fc0_edges, RP2350ClocksState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_clocks_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);
    int i;

    for (i = 0; i < RP2350_NUM_CLOCKS; i++) {
        clocks_reset[R_CLK_DIV(i)] = 0x00010000;
        clocks_wmask[R_CLK_CTRL(i)] = clk_info[i].ctrl_wmask;
        clocks_wmask[R_CLK_DIV(i)] =
            MAKE_64BIT_MASK(16, clk_info[i].int_bits) |
            (clk_info[i].frac ? 0xffff : 0);
    }
    clocks_reset[R_CLK_CTRL(RP2350_CLK_SYS)] = 0x41;
    clocks_reset[R_RESUS_CTRL] = 0xff;
    clocks_reset[R_FC0_MAX_KHZ] = 0x1ffffff;
    clocks_reset[R_FC0_DELAY] = 1;
    clocks_reset[R_FC0_INTERVAL] = 8;
    for (i = R_WAKE_EN0; i <= R_SLEEP_EN1; i++) {
        clocks_reset[i] = UINT32_MAX;
        clocks_wmask[i] = UINT32_MAX;
    }
    for (i = R_DFTCLK_XOSC_CTRL; i <= R_DFTCLK_LPOSC_CTRL; i++) {
        clocks_wmask[i] = 0x3;
    }
    clocks_wmask[R_RESUS_CTRL] = 0x000111ff;
    clocks_wmask[R_FC0_REF_KHZ] = 0x000fffff;
    clocks_wmask[R_FC0_MIN_KHZ] = 0x01ffffff;
    clocks_wmask[R_FC0_MAX_KHZ] = 0x01ffffff;
    clocks_wmask[R_FC0_DELAY] = 0x7;
    clocks_wmask[R_FC0_INTERVAL] = 0xf;
    clocks_wmask[R_FC0_SRC] = 0xff;
    clocks_wmask[R_INTE] = 0x1;
    clocks_wmask[R_INTF] = 0x1;

    c->nregs = CLOCKS_NREGS;
    c->reset = clocks_reset;
    c->wmask = clocks_wmask;
    c->read = clocks_read;
    c->written = clocks_written;
    c->reset_hold = clocks_reset_hold;
    rc->phases.exit = rp2350_clocks_exit_reset;
    dc->vmsd = &vmstate_rp2350_clocks;
}

/* XOSC */

#define R_XOSC_CTRL        0
#define R_XOSC_STATUS      1
#define R_XOSC_DORMANT     2
#define R_XOSC_STARTUP     3
#define R_XOSC_COUNT       4
#define XOSC_NREGS         5

#define XOSC_CTRL_ENABLE_SHIFT 12
#define XOSC_CTRL_ENABLE_MASK  0xfff
#define XOSC_ENABLE            0xfab
#define XOSC_DISABLE           0xd1e
#define XOSC_RANGE_MASK        0xfff
#define XOSC_RANGE_1_15MHZ     0xaa0
#define XOSC_RANGE_40_100MHZ   0xaa3
#define XOSC_STATUS_ENABLED    (1u << 12)
#define XOSC_STATUS_BADWRITE   (1u << 24)
#define XOSC_STATUS_STABLE     (1u << 31)
#define XOSC_STARTUP_DELAY     0x3fff
#define XOSC_STARTUP_X4        (1u << 20)
#define XOSC_DORMANT_DORMANT   0x636f6d61u
#define XOSC_DORMANT_WAKE      0x77616b65u
#define XOSC_COUNT_MASK        0xffffu

static const uint32_t xosc_reset[XOSC_NREGS] = {
    /* On power-up DORMANT is initialised to WAKE. */
    [R_XOSC_DORMANT] = XOSC_DORMANT_WAKE,
};

static const uint32_t xosc_wmask[XOSC_NREGS] = {
    [R_XOSC_CTRL] = 0x00ffffff,
    [R_XOSC_STARTUP] = 0x00103fff,
};

/* STARTUP.DELAY counts 256 crystal periods, four times over with X4. */
static int64_t xosc_startup_ns(RP2350XOSCState *x)
{
    uint32_t startup = x->parent_obj.regs[R_XOSC_STARTUP];
    uint64_t cycles = (uint64_t)(startup & XOSC_STARTUP_DELAY) * 256;

    if (startup & XOSC_STARTUP_X4) {
        cycles *= 4;
    }
    return muldiv64(cycles, NANOSECONDS_PER_SECOND, x->xtal_hz);
}

static bool xosc_stable(RP2350XOSCState *x)
{
    return x->enabled && !x->dormant &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= x->stable_ns;
}

/* The oscillator's output runs once it is stable and not gated. */
static bool xosc_running(RP2350XOSCState *x)
{
    return xosc_stable(x) && !x->gated;
}

static uint32_t xosc_count_now(RP2350XOSCState *x)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ticks;

    if (!clock_is_enabled(x->out) || !x->count || now <= x->count_ns) {
        return x->count;
    }
    ticks = clock_ns_to_ticks(x->out, now - x->count_ns);
    return ticks >= x->count ? 0 : x->count - ticks;
}

/*
 * Drive the output from the oscillator's state, bringing COUNT up to
 * date at the old rate first.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void xosc_update_out(RP2350XOSCState *x)
{
    x->count = xosc_count_now(x);
    x->count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    clock_update_hz(x->out, xosc_running(x) ? x->xtal_hz : 0);
}

/* The crystal starts oscillating; it is STABLE once the delay expires. */
static void xosc_start(RP2350XOSCState *x)
{
    x->stable_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + xosc_startup_ns(x);
    timer_mod(x->startup_timer, x->stable_ns);
}

static void xosc_set_gated(RP2350XOSCState *x, bool gated)
{
    x->gated = gated;
}

/*
 * A wake event restarts the oscillator, and its output is ungated once
 * it is stable. A disabled oscillator has nothing to restart.
 */
/* [spec:nuos:req:emu.clocks] */
static void xosc_wake(RP2350XOSCState *x)
{
    x->dormant = false;
    x->parent_obj.regs[R_XOSC_DORMANT] = XOSC_DORMANT_WAKE;
    if (!x->enabled) {
        xosc_set_gated(x, false);
        return;
    }
    xosc_start(x);
}

static void xosc_startup_cb(void *opaque)
{
    RP2350XOSCState *x = opaque;

    if (!x->dormant && x->gated) {
        xosc_set_gated(x, false);
    }
    xosc_update_out(x);
}

/* [spec:nuos:req:emu.clocks] */
static void xosc_enter_dormant(RP2350XOSCState *x)
{
    if (x->dormant) {
        return;
    }
    timer_del(x->startup_timer);
    x->dormant = true;
    x->parent_obj.regs[R_XOSC_DORMANT] = XOSC_DORMANT_DORMANT;
    xosc_set_gated(x, true);
    xosc_update_out(x);
    /* A wake event already asserted restarts the oscillator at once. */
    if (x->wake) {
        xosc_wake(x);
    }
}

static void xosc_set_wake(void *opaque, int n, int level)
{
    RP2350XOSCState *x = opaque;

    x->wake = level;
    if (level && x->dormant) {
        xosc_wake(x);
    }
}

/* [spec:nuos:req:emu.clocks] */
static uint32_t xosc_read(RP2350ClkRegsState *s, unsigned reg)
{
    RP2350XOSCState *x = RP2350_XOSC(s);
    uint32_t st = 0;

    switch (reg) {
    case R_XOSC_STATUS:
        if (x->enabled) {
            st |= XOSC_STATUS_ENABLED;
        }
        if (xosc_stable(x)) {
            st |= XOSC_STATUS_STABLE;
        }
        if (x->badwrite) {
            st |= XOSC_STATUS_BADWRITE;
        }
        return st | (x->freq_range & 0x3);
    case R_XOSC_COUNT:
        /* Reads return 1 while the count runs and 0 once it is done. */
        return xosc_count_now(x) != 0;
    }
    return s->regs[reg];
}

/* [spec:nuos:req:emu.clocks] */
static bool xosc_write(RP2350ClkRegsState *s, unsigned reg, hwaddr addr,
                       uint32_t value)
{
    RP2350XOSCState *x = RP2350_XOSC(s);

    switch (reg) {
    case R_XOSC_STATUS:
        /* BADWRITE is write-1-to-clear through any alias. */
        if (value & XOSC_STATUS_BADWRITE) {
            x->badwrite = false;
        }
        return true;
    case R_XOSC_DORMANT:
        value = rp2350_atomic_apply(addr, s->regs[R_XOSC_DORMANT], value);
        if (value == XOSC_DORMANT_DORMANT) {
            xosc_enter_dormant(x);
        } else if (value != XOSC_DORMANT_WAKE) {
            /* An invalid write selects WAKE, which the running XOSC is. */
            x->badwrite = true;
        }
        return true;
    case R_XOSC_COUNT:
        /*
         * A down counter at the XOSC frequency, stopping at zero; a
         * non-zero count below 4 counts 4.
         */
        value = rp2350_atomic_apply(addr, xosc_count_now(x), value);
        x->count = value & XOSC_COUNT_MASK;
        if (x->count && x->count < 4) {
            x->count = 4;
        }
        x->count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        return true;
    }
    return false;
}

/*
 * CTRL is stored as written; its ENABLE and FREQ_RANGE codes take effect
 * only when valid.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void xosc_written(RP2350ClkRegsState *s, unsigned reg)
{
    RP2350XOSCState *x = RP2350_XOSC(s);
    uint32_t ctrl = s->regs[R_XOSC_CTRL];
    uint32_t enable = (ctrl >> XOSC_CTRL_ENABLE_SHIFT) & XOSC_CTRL_ENABLE_MASK;
    uint32_t range = ctrl & XOSC_RANGE_MASK;
    bool enabled = x->enabled;

    if (reg != R_XOSC_CTRL) {
        return;
    }
    if (range >= XOSC_RANGE_1_15MHZ && range <= XOSC_RANGE_40_100MHZ) {
        x->freq_range = range - XOSC_RANGE_1_15MHZ;
    } else {
        x->badwrite = true;
    }
    if (enable == XOSC_ENABLE || enable == XOSC_DISABLE) {
        enabled = enable == XOSC_ENABLE;
    } else {
        x->badwrite = true;
    }
    if (enabled == x->enabled) {
        return;
    }
    x->enabled = enabled;
    if (enabled && !x->dormant) {
        xosc_start(x);
    } else {
        timer_del(x->startup_timer);
    }
    xosc_update_out(x);
}

static void xosc_reset_hold(RP2350ClkRegsState *s)
{
    RP2350XOSCState *x = RP2350_XOSC(s);

    timer_del(x->startup_timer);
    x->enabled = false;
    x->freq_range = 0;
    x->badwrite = false;
    x->dormant = false;
    x->gated = false;
    x->stable_ns = 0;
    x->count = 0;
    x->count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void rp2350_xosc_exit_reset(Object *obj, ResetType type)
{
    RP2350XOSCState *x = RP2350_XOSC(obj);

    clock_update(x->out, 0);
}

static void rp2350_xosc_init(Object *obj)
{
    RP2350XOSCState *x = RP2350_XOSC(obj);
    DeviceState *dev = DEVICE(obj);

    x->startup_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xosc_startup_cb, x);
    x->out = qdev_init_clock_out(dev, RP2350_XOSC_CLK);
    qdev_init_gpio_in_named(dev, xosc_set_wake, RP2350_OSC_DORMANT_WAKE, 1);
}

static void rp2350_xosc_realize(DeviceState *dev, Error **errp)
{
    RP2350XOSCState *x = RP2350_XOSC(dev);

    if (x->xtal_hz < 1000000 || x->xtal_hz > 50000000) {
        error_setg(errp, "xtal-hz must be from 1 MHz to 50 MHz");
    }
}

static const VMStateDescription vmstate_rp2350_xosc = {
    .name = TYPE_RP2350_XOSC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(parent_obj.regs, RP2350XOSCState,
                             RP2350_CLKREGS_MAX),
        VMSTATE_TIMER_PTR(startup_timer, RP2350XOSCState),
        VMSTATE_CLOCK(out, RP2350XOSCState),
        VMSTATE_BOOL(enabled, RP2350XOSCState),
        VMSTATE_UINT32(freq_range, RP2350XOSCState),
        VMSTATE_BOOL(badwrite, RP2350XOSCState),
        VMSTATE_BOOL(dormant, RP2350XOSCState),
        VMSTATE_BOOL(gated, RP2350XOSCState),
        VMSTATE_BOOL(wake, RP2350XOSCState),
        VMSTATE_INT64(stable_ns, RP2350XOSCState),
        VMSTATE_UINT32(count, RP2350XOSCState),
        VMSTATE_INT64(count_ns, RP2350XOSCState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_xosc_properties[] = {
    DEFINE_PROP_UINT32("xtal-hz", RP2350XOSCState, xtal_hz, 12000000),
};

static void rp2350_xosc_class_init(ObjectClass *klass, const void *data)
{
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    c->nregs = XOSC_NREGS;
    c->reset = xosc_reset;
    c->wmask = xosc_wmask;
    c->read = xosc_read;
    c->write = xosc_write;
    c->written = xosc_written;
    c->reset_hold = xosc_reset_hold;
    rc->phases.exit = rp2350_xosc_exit_reset;
    dc->realize = rp2350_xosc_realize;
    dc->vmsd = &vmstate_rp2350_xosc;
    device_class_set_props(dc, rp2350_xosc_properties);
}

/* PLL */

#define R_PLL_CS           0
#define R_PLL_PWR          1
#define R_PLL_FBDIV_INT    2
#define R_PLL_PRIM         3
#define R_PLL_INTR         4
#define R_PLL_INTE         5
#define R_PLL_INTF         6
#define R_PLL_INTS         7
#define PLL_NREGS          8

#define PLL_CS_REFDIV_MASK 0x3f
#define PLL_CS_BYPASS      (1u << 8)
#define PLL_CS_LOCK_N      (1u << 30)
#define PLL_CS_LOCK        (1u << 31)
#define PLL_PWR_PD         (1u << 0)
#define PLL_PWR_POSTDIVPD  (1u << 3)
#define PLL_PWR_VCOPD      (1u << 5)
#define PLL_INTR_LOCK_N    (1u << 0)

/* Cycles of FREF / REFDIV the VCO takes to lock. */
#define PLL_LOCK_REF_CYCLES 400

static const uint32_t pll_reset[PLL_NREGS] = {
    [R_PLL_CS] = 0x00000001,
    [R_PLL_PWR] = 0x0000002d,
    [R_PLL_PRIM] = 0x00077000,
};

static const uint32_t pll_wmask[PLL_NREGS] = {
    [R_PLL_CS] = 0x0000013f,
    [R_PLL_PWR] = 0x0000002d,
    [R_PLL_FBDIV_INT] = 0x00000fff,
    [R_PLL_PRIM] = 0x00077000,
    [R_PLL_INTE] = 0x1,
    [R_PLL_INTF] = 0x1,
};

static void pll_update_irq(RP2350PLLState *p)
{
    uint32_t *regs = p->parent_obj.regs;

    qemu_set_irq(p->irq, (regs[R_PLL_INTR] | regs[R_PLL_INTF]) &
                 regs[R_PLL_INTE] & 1);
}

/*
 * Whether the VCO can lock: powered, with a running reference and
 * dividers in range (REFDIV is undefined at 0).
 */
static bool pll_vco_valid(RP2350PLLState *p)
{
    uint32_t *regs = p->parent_obj.regs;
    uint32_t fbdiv = regs[R_PLL_FBDIV_INT];

    return !(regs[R_PLL_PWR] & (PLL_PWR_PD | PLL_PWR_VCOPD)) &&
           clock_is_enabled(p->ref) &&
           (regs[R_PLL_CS] & PLL_CS_REFDIV_MASK) &&
           fbdiv >= 16 && fbdiv <= 320;
}

/* [spec:nuos:req:emu.clock-tree] */
static void pll_update_out(RP2350PLLState *p)
{
    uint32_t *regs = p->parent_obj.regs;
    uint32_t refdiv = regs[R_PLL_CS] & PLL_CS_REFDIV_MASK;
    uint32_t pd1 = extract32(regs[R_PLL_PRIM], 16, 3);
    uint32_t pd2 = extract32(regs[R_PLL_PRIM], 12, 3);
    uint64_t period = 0;

    if (device_is_in_reset(DEVICE(p))) {
        period = 0;
    } else if (regs[R_PLL_CS] & PLL_CS_BYPASS) {
        period = clock_get(p->ref);
    } else if (p->locked && !(regs[R_PLL_PWR] & PLL_PWR_POSTDIVPD) &&
               pd1 && pd2) {
        period = period_scale(clock_get(p->ref), refdiv * pd1 * pd2,
                              regs[R_PLL_FBDIV_INT]);
    }
    clock_update(p->out, period);
}

static void pll_lose_lock(RP2350PLLState *p)
{
    uint32_t *regs = p->parent_obj.regs;

    timer_del(p->lock_timer);
    p->locking = false;
    if (p->locked) {
        p->locked = false;
        regs[R_PLL_CS] |= PLL_CS_LOCK_N;
        regs[R_PLL_INTR] |= PLL_INTR_LOCK_N;
        pll_update_irq(p);
    }
}

/*
 * Follow a change of power, dividers or reference: a VCO that can no
 * longer run loses lock, and one that can starts acquiring it. A change
 * of REFDIV disturbs a locked VCO, which must lock again; FBDIV can be
 * changed while locked.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void pll_update(RP2350PLLState *p, bool relock)
{
    uint32_t *regs = p->parent_obj.regs;

    if (!pll_vco_valid(p) || relock) {
        pll_lose_lock(p);
    }
    if (pll_vco_valid(p) && !p->locked && !p->locking) {
        uint32_t refdiv = regs[R_PLL_CS] & PLL_CS_REFDIV_MASK;
        uint64_t vco_hz = muldiv64(clock_get_hz(p->ref), regs[R_PLL_FBDIV_INT],
                                   refdiv);

        if (vco_hz < 750000000 || vco_hz > 1600000000) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: VCO at %" PRIu64 " Hz is "
                          "outside 750-1600 MHz\n",
                          object_get_canonical_path_component(OBJECT(p)),
                          vco_hz);
        }
        p->locking = true;
        p->lock_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     clock_ticks_to_ns(p->ref, PLL_LOCK_REF_CYCLES * refdiv);
        timer_mod(p->lock_timer, p->lock_ns);
    }
    pll_update_out(p);
}

static void pll_lock_cb(void *opaque)
{
    RP2350PLLState *p = opaque;

    p->locking = false;
    p->locked = pll_vco_valid(p);
    pll_update_out(p);
}

static void pll_ref_changed(void *opaque, ClockEvent event)
{
    RP2350PLLState *p = opaque;

    if (!device_is_in_reset(DEVICE(p))) {
        pll_update(p, false);
    }
}

/* [spec:nuos:req:emu.clocks] */
static uint32_t pll_read(RP2350ClkRegsState *s, unsigned reg)
{
    RP2350PLLState *p = RP2350_PLL(s);
    uint32_t *regs = s->regs;

    switch (reg) {
    case R_PLL_CS:
        return regs[R_PLL_CS] | (p->locked ? PLL_CS_LOCK : 0);
    case R_PLL_INTS:
        return (regs[R_PLL_INTR] | regs[R_PLL_INTF]) & regs[R_PLL_INTE];
    }
    return regs[reg];
}

/* [spec:nuos:req:emu.clocks] */
static bool pll_write(RP2350ClkRegsState *s, unsigned reg, hwaddr addr,
                      uint32_t value)
{
    RP2350PLLState *p = RP2350_PLL(s);
    uint32_t *regs = s->regs;
    uint32_t old = regs[reg];
    uint32_t v;

    switch (reg) {
    case R_PLL_CS:
        /* LOCK_N is write-1-to-clear through any alias. */
        if (value & PLL_CS_LOCK_N) {
            regs[R_PLL_CS] &= ~PLL_CS_LOCK_N;
        }
        v = rp2350_atomic_apply(addr, old & pll_wmask[reg], value);
        regs[R_PLL_CS] = (regs[R_PLL_CS] & ~pll_wmask[reg]) |
                         (v & pll_wmask[reg]);
        pll_update(p, (old ^ regs[R_PLL_CS]) & PLL_CS_REFDIV_MASK);
        return true;
    case R_PLL_INTR:
        if (value & PLL_INTR_LOCK_N) {
            regs[R_PLL_INTR] = 0;
            pll_update_irq(p);
        }
        return true;
    }
    return false;
}

static void pll_written(RP2350ClkRegsState *s, unsigned reg)
{
    RP2350PLLState *p = RP2350_PLL(s);

    switch (reg) {
    case R_PLL_PWR:
    case R_PLL_FBDIV_INT:
    case R_PLL_PRIM:
        pll_update(p, false);
        break;
    case R_PLL_INTE:
    case R_PLL_INTF:
        pll_update_irq(p);
        break;
    }
}

/* A PLL held in reset is powered down, and its output is stopped. */
static void pll_reset_hold(RP2350ClkRegsState *s)
{
    RP2350PLLState *p = RP2350_PLL(s);

    timer_del(p->lock_timer);
    p->locked = false;
    p->locking = false;
    p->lock_ns = 0;
    qemu_set_irq(p->irq, 0);
    clock_update(p->out, 0);
}

static void rp2350_pll_exit_reset(Object *obj, ResetType type)
{
    pll_update(RP2350_PLL(obj), false);
}

static void rp2350_pll_init(Object *obj)
{
    RP2350PLLState *p = RP2350_PLL(obj);
    DeviceState *dev = DEVICE(obj);

    p->ref = qdev_init_clock_in(dev, RP2350_PLL_REF, pll_ref_changed, p,
                                ClockUpdate);
    p->out = qdev_init_clock_out(dev, RP2350_PLL_OUT);
    p->lock_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pll_lock_cb, p);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &p->irq);
}

static const VMStateDescription vmstate_rp2350_pll = {
    .name = TYPE_RP2350_PLL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(parent_obj.regs, RP2350PLLState,
                             RP2350_CLKREGS_MAX),
        VMSTATE_CLOCK(ref, RP2350PLLState),
        VMSTATE_CLOCK(out, RP2350PLLState),
        VMSTATE_TIMER_PTR(lock_timer, RP2350PLLState),
        VMSTATE_BOOL(locked, RP2350PLLState),
        VMSTATE_BOOL(locking, RP2350PLLState),
        VMSTATE_INT64(lock_ns, RP2350PLLState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_pll_class_init(ObjectClass *klass, const void *data)
{
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    c->nregs = PLL_NREGS;
    c->reset = pll_reset;
    c->wmask = pll_wmask;
    c->read = pll_read;
    c->write = pll_write;
    c->written = pll_written;
    c->reset_hold = pll_reset_hold;
    rc->phases.exit = rp2350_pll_exit_reset;
    dc->vmsd = &vmstate_rp2350_pll;
}

/* TICKS */

#define TICK_STRIDE        3 /* CTRL, CYCLES, COUNT */
#define TICKS_NREGS        (RP2350_NUM_TICKS * TICK_STRIDE)
#define TICK_CTRL_ENABLE   (1u << 0)
#define TICK_CTRL_RUNNING  (1u << 1)

const char *const rp2350_tick_names[RP2350_NUM_TICKS] = {
    [RP2350_TICK_PROC0] = "proc0",
    [RP2350_TICK_PROC1] = "proc1",
    [RP2350_TICK_TIMER0] = "timer0",
    [RP2350_TICK_TIMER1] = "timer1",
    [RP2350_TICK_WATCHDOG] = "watchdog",
    [RP2350_TICK_RISCV] = "riscv",
};

static uint32_t ticks_wmask[TICKS_NREGS];

static bool tick_running(RP2350TicksState *t, int n)
{
    return (t->parent_obj.regs[n * TICK_STRIDE] & TICK_CTRL_ENABLE) &&
           clock_is_enabled(t->ref);
}

/*
 * Drive tick n: one tick every CYCLES clk_ref cycles while it runs. A
 * generator with CYCLES 0 never completes a tick.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void tick_update(RP2350TicksState *t, int n)
{
    uint32_t cycles = t->parent_obj.regs[n * TICK_STRIDE + 1];
    uint64_t ref = clock_get(t->ref);
    uint64_t period = 0;

    if (tick_running(t, n) && cycles) {
        uint64_t hz = CLOCK_PERIOD_TO_HZ(ref);

        /*
         * clk_ref's period is truncated to 2^-32 ns; for a clk_ref of a
         * whole number of Hz, as from a crystal, take the tick's period
         * from the frequency, so that a 1 us tick is exactly 1 us.
         */
        if (hz && CLOCK_PERIOD_FROM_HZ(hz) == ref) {
            period = period_scale(CLOCK_PERIOD_1SEC, cycles, hz);
        } else {
            period = ref * cycles;
        }
    }
    if (period != clock_get(t->tick[n])) {
        t->phase_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    clock_update(t->tick[n], period);
}

/* [spec:nuos:req:emu.clocks] */
static uint32_t ticks_read(RP2350ClkRegsState *s, unsigned reg)
{
    RP2350TicksState *t = RP2350_TICKS(s);
    int n = reg / TICK_STRIDE;
    uint32_t v = s->regs[reg];

    switch (reg % TICK_STRIDE) {
    case 0:
        return tick_running(t, n) ? v | TICK_CTRL_RUNNING : v;
    case 2:
        /* The clk_ref cycles left before the next tick. */
        if (clock_is_enabled(t->tick[n])) {
            uint32_t cycles = s->regs[reg - 1];
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

            return cycles - clock_ns_to_ticks(t->ref, now - t->phase_ns[n]) %
                            cycles;
        }
        return 0;
    }
    return v;
}

static void ticks_written(RP2350ClkRegsState *s, unsigned reg)
{
    tick_update(RP2350_TICKS(s), reg / TICK_STRIDE);
}

static void ticks_ref_changed(void *opaque, ClockEvent event)
{
    RP2350TicksState *t = opaque;
    int i;

    for (i = 0; i < RP2350_NUM_TICKS; i++) {
        tick_update(t, i);
    }
}

/* A reset stops every generator. */
static void rp2350_ticks_exit_reset(Object *obj, ResetType type)
{
    ticks_ref_changed(obj, ClockUpdate);
}

static void rp2350_ticks_init(Object *obj)
{
    RP2350TicksState *t = RP2350_TICKS(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    t->ref = qdev_init_clock_in(dev, RP2350_TICKS_REF, ticks_ref_changed, t,
                                ClockUpdate);
    for (i = 0; i < RP2350_NUM_TICKS; i++) {
        t->tick[i] = qdev_init_clock_out(dev, rp2350_tick_names[i]);
    }
}

static const VMStateDescription vmstate_rp2350_ticks = {
    .name = TYPE_RP2350_TICKS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(parent_obj.regs, RP2350TicksState,
                             RP2350_CLKREGS_MAX),
        VMSTATE_CLOCK(ref, RP2350TicksState),
        VMSTATE_ARRAY_CLOCK(tick, RP2350TicksState, RP2350_NUM_TICKS),
        VMSTATE_INT64_ARRAY(phase_ns, RP2350TicksState, RP2350_NUM_TICKS),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_ticks_class_init(ObjectClass *klass, const void *data)
{
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    int i;

    for (i = 0; i < RP2350_NUM_TICKS; i++) {
        ticks_wmask[i * TICK_STRIDE] = TICK_CTRL_ENABLE;
        ticks_wmask[i * TICK_STRIDE + 1] = 0x1ff;
    }
    c->nregs = TICKS_NREGS;
    c->wmask = ticks_wmask;
    c->read = ticks_read;
    c->written = ticks_written;
    rc->phases.exit = rp2350_ticks_exit_reset;
    dc->vmsd = &vmstate_rp2350_ticks;
}

/* [spec:nuos:req:emu.clock-tree] */
static const TypeInfo rp2350_clocks_types[] = {
    {
        .name          = TYPE_RP2350_CLKREGS,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RP2350ClkRegsState),
        .instance_init = rp2350_clkregs_init,
        .class_size    = sizeof(RP2350ClkRegsClass),
        .class_init    = rp2350_clkregs_class_init,
        .abstract      = true,
    },
    {
        .name          = TYPE_RP2350_CLOCKS,
        .parent        = TYPE_RP2350_CLKREGS,
        .instance_size = sizeof(RP2350ClocksState),
        .instance_init = rp2350_clocks_init,
        .class_init    = rp2350_clocks_class_init,
    },
    {
        .name          = TYPE_RP2350_XOSC,
        .parent        = TYPE_RP2350_CLKREGS,
        .instance_size = sizeof(RP2350XOSCState),
        .instance_init = rp2350_xosc_init,
        .class_init    = rp2350_xosc_class_init,
    },
    {
        .name          = TYPE_RP2350_PLL,
        .parent        = TYPE_RP2350_CLKREGS,
        .instance_size = sizeof(RP2350PLLState),
        .instance_init = rp2350_pll_init,
        .class_init    = rp2350_pll_class_init,
    },
    {
        .name          = TYPE_RP2350_TICKS,
        .parent        = TYPE_RP2350_CLKREGS,
        .instance_size = sizeof(RP2350TicksState),
        .instance_init = rp2350_ticks_init,
        .class_init    = rp2350_ticks_class_init,
    },
};

DEFINE_TYPES(rp2350_clocks_types)
