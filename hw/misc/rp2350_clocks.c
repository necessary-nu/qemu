/*
 * RP2350 clock generation: CLOCKS, XOSC, PLL_SYS/PLL_USB and TICKS
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Clocks", "Crystal oscillator (XOSC)",
 * "PLL", "Tick generators" and "DORMANT state". PLLs lock and clock
 * muxes switch as soon as they are configured. The crystal oscillator
 * becomes stable STARTUP.DELAY * 256 crystal periods after it starts.
 *
 * DORMANT. Writing the DORMANT keyword to XOSC.DORMANT stops the crystal
 * oscillator until a wake event: the GPIO banks' dormant_wake interrupt
 * or the AON timer alarm, both arriving on the "dormant-wake" input. The
 * oscillator then restarts and its output stays gated until it is stable
 * again. From entry until then the "dormant" output is high, and the SoC
 * stops every clock running from the oscillator: the processors, the
 * TICKS generators and the blocks counting them. The PLLs are not halted
 * by DORMANT, but their reference stops with the XOSC, so clocks running
 * from a PLL stop too.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_clocks.h"
#include "migration/vmstate.h"

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

static const VMStateDescription vmstate_rp2350_clkregs = {
    .name = TYPE_RP2350_CLKREGS,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RP2350ClkRegsState, RP2350_CLKREGS_MAX),
        VMSTATE_BOOL(ref_stopped, RP2350ClkRegsState),
        VMSTATE_BOOL(sys_stopped, RP2350ClkRegsState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_clkregs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_clkregs_hold_reset;
    dc->vmsd = &vmstate_rp2350_clkregs;
}

/* CLOCKS */

#define CLK_COUNT          10
#define CLK_REF            4
#define CLK_SYS            5
#define CLK_STRIDE         3 /* CTRL, DIV, SELECTED */
#define R_CLK_CTRL(n)      ((n) * CLK_STRIDE)
#define R_CLK_DIV(n)       ((n) * CLK_STRIDE + 1)
#define R_CLK_SELECTED(n)  ((n) * CLK_STRIDE + 2)
#define R_RESUS_STATUS     (0x88 / 4)
#define R_FC0_STATUS       (0xa4 / 4)
#define R_FC0_RESULT       (0xa8 / 4)
#define R_ENABLED0         (0xbc / 4)
#define R_ENABLED1         (0xc0 / 4)
#define R_INTR             (0xc4 / 4)
#define R_INTS             (0xd0 / 4)
#define CLOCKS_NREGS       (R_INTS + 1)

#define CLK_REF_SRC_ROSC   0
#define CLK_REF_SRC_AUX    1
#define CLK_REF_SRC_XOSC   2
#define CLK_REF_AUX_GPIN0  1
#define CLK_REF_AUX_GPIN1  2
#define CLK_SYS_AUX_PLL_SYS 0
#define CLK_SYS_AUX_PLL_USB 1
#define CLK_SYS_AUX_ROSC   2
#define CLK_SYS_AUX_XOSC   3

#define FC0_STATUS_PASS    (1u << 0)
#define FC0_STATUS_DONE    (1u << 4)

static uint32_t clocks_reset[CLOCKS_NREGS];
static uint32_t clocks_wmask[CLOCKS_NREGS];

/* [spec:nuos:req:emu.clocks] */
static uint32_t clocks_read(RP2350ClkRegsState *s, unsigned reg)
{
    if (reg < CLK_COUNT * CLK_STRIDE && reg % CLK_STRIDE == 2) {
        unsigned clk = reg / CLK_STRIDE;

        /*
         * clk_ref and clk_sys have glitchless muxes whose SELECTED is a
         * one-hot of the current source; the other clocks' read 1.
         */
        if (clk == CLK_REF) {
            return 1u << (s->regs[R_CLK_CTRL(CLK_REF)] & 0x3);
        }
        if (clk == CLK_SYS) {
            return 1u << (s->regs[R_CLK_CTRL(CLK_SYS)] & 0x1);
        }
        return 1;
    }
    switch (reg) {
    case R_FC0_STATUS:
        /* The frequency counter finishes at once and always passes. */
        return FC0_STATUS_DONE | FC0_STATUS_PASS;
    case R_ENABLED0:
    case R_ENABLED1:
        return UINT32_MAX;
    }
    return s->regs[reg];
}

/* [spec:nuos:req:emu.clocks] */
RP2350ClockRoot rp2350_clocks_root(RP2350ClkRegsState *clocks, bool sys)
{
    uint32_t ref = clocks->regs[R_CLK_CTRL(CLK_REF)];
    uint32_t sysc = clocks->regs[R_CLK_CTRL(CLK_SYS)];

    if (sys && (sysc & 0x1)) {
        switch ((sysc >> 5) & 0x7) {
        case CLK_SYS_AUX_PLL_SYS:
        case CLK_SYS_AUX_PLL_USB:
            /* The PLLs' reference is the XOSC. */
        case CLK_SYS_AUX_XOSC:
            return RP2350_ROOT_XOSC;
        case CLK_SYS_AUX_ROSC:
            return RP2350_ROOT_ROSC;
        default:
            /* GPIN0 and GPIN1 are external clocks. */
            return RP2350_ROOT_OTHER;
        }
    }
    switch (ref & 0x3) {
    case CLK_REF_SRC_ROSC:
        return RP2350_ROOT_ROSC;
    case CLK_REF_SRC_XOSC:
        return RP2350_ROOT_XOSC;
    case CLK_REF_SRC_AUX:
        switch ((ref >> 5) & 0x3) {
        case CLK_REF_AUX_GPIN0:
        case CLK_REF_AUX_GPIN1:
            return RP2350_ROOT_OTHER;
        default:
            /* PLL_USB, and its primary reference, the XOSC. */
            return RP2350_ROOT_XOSC;
        }
    default:
        /* The LPOSC runs through DORMANT, for the AON timer. */
        return RP2350_ROOT_OTHER;
    }
}

void rp2350_clocks_set_notify(RP2350ClkRegsState *clocks,
                              RP2350TickNotify *fn, void *opaque)
{
    clocks->clocks_notify = fn;
    clocks->clocks_opaque = opaque;
}

static void clocks_written(RP2350ClkRegsState *s, unsigned reg)
{
    if ((reg == R_CLK_CTRL(CLK_REF) || reg == R_CLK_CTRL(CLK_SYS)) &&
        s->clocks_notify) {
        s->clocks_notify(s->clocks_opaque);
    }
}

/* A reset switches clk_ref and clk_sys back to the ROSC. */
static void rp2350_clocks_exit_reset(Object *obj, ResetType type)
{
    RP2350ClkRegsState *s = RP2350_CLKREGS(obj);

    if (s->clocks_notify) {
        s->clocks_notify(s->clocks_opaque);
    }
}

static void rp2350_clocks_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);
    int i;

    for (i = 0; i < CLOCKS_NREGS; i++) {
        clocks_wmask[i] = UINT32_MAX;
    }
    for (i = 0; i < CLK_COUNT; i++) {
        clocks_reset[R_CLK_DIV(i)] = 0x00010000;
        clocks_wmask[R_CLK_SELECTED(i)] = 0;
    }
    clocks_reset[R_CLK_CTRL(CLK_SYS)] = 0x41;
    clocks_wmask[R_RESUS_STATUS] = 0;
    clocks_wmask[R_FC0_STATUS] = 0;
    clocks_wmask[R_FC0_RESULT] = 0;
    clocks_wmask[R_ENABLED0] = 0;
    clocks_wmask[R_ENABLED1] = 0;
    clocks_wmask[R_INTR] = 0;
    clocks_wmask[R_INTS] = 0;

    c->nregs = CLOCKS_NREGS;
    c->reset = clocks_reset;
    c->wmask = clocks_wmask;
    c->read = clocks_read;
    c->written = clocks_written;
    rc->phases.exit = rp2350_clocks_exit_reset;
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
#define XOSC_STATUS_ENABLED    (1u << 12)
#define XOSC_STATUS_BADWRITE   (1u << 24)
#define XOSC_STATUS_STABLE     (1u << 31)
#define XOSC_STARTUP_DELAY     0x3fff
#define XOSC_STARTUP_X4        (1u << 20)
#define XOSC_DORMANT_DORMANT   0x636f6d61u
#define XOSC_DORMANT_WAKE      0x77616b65u

static const uint32_t xosc_reset[XOSC_NREGS] = {
    /* On power-up DORMANT is initialised to WAKE. */
    [R_XOSC_DORMANT] = XOSC_DORMANT_WAKE,
};

static const uint32_t xosc_wmask[XOSC_NREGS] = {
    [R_XOSC_CTRL] = 0x00ffffff,
    [R_XOSC_STARTUP] = 0x00103fff,
    [R_XOSC_COUNT] = 0x0000ffff,
};

static bool xosc_ctrl_enabled(RP2350ClkRegsState *s)
{
    return ((s->regs[R_XOSC_CTRL] >> XOSC_CTRL_ENABLE_SHIFT) &
            XOSC_CTRL_ENABLE_MASK) == XOSC_ENABLE;
}

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

/* The crystal starts oscillating; it is STABLE once the delay expires. */
static void xosc_start(RP2350XOSCState *x)
{
    x->stable_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + xosc_startup_ns(x);
}

static bool xosc_stable(RP2350XOSCState *x)
{
    return x->enabled && !x->dormant &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= x->stable_ns;
}

bool rp2350_xosc_dormant(RP2350XOSCState *x)
{
    return x->gated;
}

static void xosc_set_gated(RP2350XOSCState *x, bool gated)
{
    x->gated = gated;
    qemu_set_irq(x->dormant_irq, gated);
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
    timer_mod(x->startup_timer, x->stable_ns);
}

static void xosc_startup_cb(void *opaque)
{
    RP2350XOSCState *x = opaque;

    if (!x->dormant) {
        xosc_set_gated(x, false);
    }
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
        return st | (s->regs[R_XOSC_CTRL] & 0x3);
    case R_XOSC_COUNT:
        /* The count-down timer expires at once. */
        return 0;
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
    }
    return false;
}

static void xosc_written(RP2350ClkRegsState *s, unsigned reg)
{
    RP2350XOSCState *x = RP2350_XOSC(s);
    bool enabled = xosc_ctrl_enabled(s);

    if (reg != R_XOSC_CTRL || enabled == x->enabled) {
        return;
    }
    x->enabled = enabled;
    if (enabled && !x->dormant) {
        xosc_start(x);
    }
}

static void xosc_reset_hold(RP2350ClkRegsState *s)
{
    RP2350XOSCState *x = RP2350_XOSC(s);

    timer_del(x->startup_timer);
    x->enabled = false;
    x->badwrite = false;
    x->dormant = false;
    x->gated = false;
    x->stable_ns = 0;
}

static void rp2350_xosc_exit_reset(Object *obj, ResetType type)
{
    RP2350XOSCState *x = RP2350_XOSC(obj);

    qemu_set_irq(x->dormant_irq, x->gated);
}

static void rp2350_xosc_init(Object *obj)
{
    RP2350XOSCState *x = RP2350_XOSC(obj);
    DeviceState *dev = DEVICE(obj);

    x->startup_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xosc_startup_cb, x);
    qdev_init_gpio_in_named(dev, xosc_set_wake, RP2350_OSC_DORMANT_WAKE, 1);
    qdev_init_gpio_out_named(dev, &x->dormant_irq, RP2350_OSC_DORMANT, 1);
}

static const VMStateDescription vmstate_rp2350_xosc = {
    .name = TYPE_RP2350_XOSC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(parent_obj.regs, RP2350XOSCState,
                             RP2350_CLKREGS_MAX),
        VMSTATE_TIMER_PTR(startup_timer, RP2350XOSCState),
        VMSTATE_BOOL(enabled, RP2350XOSCState),
        VMSTATE_BOOL(badwrite, RP2350XOSCState),
        VMSTATE_BOOL(dormant, RP2350XOSCState),
        VMSTATE_BOOL(gated, RP2350XOSCState),
        VMSTATE_BOOL(wake, RP2350XOSCState),
        VMSTATE_INT64(stable_ns, RP2350XOSCState),
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
#define PLL_CS_LOCK        (1u << 31)
#define PLL_PWR_PD         (1u << 0)
#define PLL_PWR_VCOPD      (1u << 5)

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

/* [spec:nuos:req:emu.clocks] */
static uint32_t pll_read(RP2350ClkRegsState *s, unsigned reg)
{
    uint32_t cs = s->regs[R_PLL_CS];
    uint32_t pwr = s->regs[R_PLL_PWR];
    uint32_t fbdiv = s->regs[R_PLL_FBDIV_INT];

    if (reg == R_PLL_CS) {
        /*
         * The PLL locks as soon as it is powered with a valid feedback
         * divider (16..320) and reference divider.
         */
        if (!(pwr & (PLL_PWR_PD | PLL_PWR_VCOPD)) &&
            fbdiv >= 16 && fbdiv <= 320 && (cs & PLL_CS_REFDIV_MASK)) {
            cs |= PLL_CS_LOCK;
        }
        return cs;
    }
    return s->regs[reg];
}

static void rp2350_pll_class_init(ObjectClass *klass, const void *data)
{
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);

    c->nregs = PLL_NREGS;
    c->reset = pll_reset;
    c->wmask = pll_wmask;
    c->read = pll_read;
}

/* TICKS */

#define TICK_STRIDE        3 /* CTRL, CYCLES, COUNT */
#define TICKS_NREGS        (RP2350_NUM_TICKS * TICK_STRIDE)
#define TICK_CTRL_ENABLE   (1u << 0)
#define TICK_CTRL_RUNNING  (1u << 1)

static uint32_t ticks_wmask[TICKS_NREGS];

/* [spec:nuos:req:emu.clocks] */
static uint32_t ticks_read(RP2350ClkRegsState *s, unsigned reg)
{
    uint32_t v = s->regs[reg];

    switch (reg % TICK_STRIDE) {
    case 0:
        return rp2350_ticks_running(s, reg / TICK_STRIDE) ?
               v | TICK_CTRL_RUNNING : v;
    case 2:
        return 0;
    }
    return v;
}

bool rp2350_ticks_running(RP2350ClkRegsState *ticks, int tick)
{
    return !ticks->ref_stopped &&
           (ticks->regs[tick * TICK_STRIDE] & TICK_CTRL_ENABLE);
}

bool rp2350_ticks_sys_running(RP2350ClkRegsState *ticks)
{
    return !ticks->sys_stopped;
}

static void ticks_notify_all(RP2350ClkRegsState *s)
{
    int i;

    for (i = 0; i < RP2350_NUM_TICKS; i++) {
        if (s->tick_notify[i]) {
            s->tick_notify[i](s->tick_opaque[i]);
        }
    }
}

/* [spec:nuos:req:emu.clocks] */
void rp2350_ticks_set_clocks(RP2350ClkRegsState *ticks, bool ref_running,
                             bool sys_running)
{
    if (ticks->ref_stopped == !ref_running &&
        ticks->sys_stopped == !sys_running) {
        return;
    }
    ticks->ref_stopped = !ref_running;
    ticks->sys_stopped = !sys_running;
    ticks_notify_all(ticks);
}

uint32_t rp2350_ticks_cycles(RP2350ClkRegsState *ticks, int tick)
{
    return ticks->regs[tick * TICK_STRIDE + 1];
}

void rp2350_ticks_set_notify(RP2350ClkRegsState *ticks, int tick,
                             RP2350TickNotify *fn, void *opaque)
{
    ticks->tick_notify[tick] = fn;
    ticks->tick_opaque[tick] = opaque;
}

static void ticks_written(RP2350ClkRegsState *s, unsigned reg)
{
    unsigned tick = reg / TICK_STRIDE;

    /* CTRL starts or stops the generator; CYCLES changes its rate. */
    if (reg % TICK_STRIDE != 2 && s->tick_notify[tick]) {
        s->tick_notify[tick](s->tick_opaque[tick]);
    }
}

/* A reset stops every generator, which their consumers must see. */
static void rp2350_ticks_exit_reset(Object *obj, ResetType type)
{
    ticks_notify_all(RP2350_CLKREGS(obj));
}

static void rp2350_ticks_class_init(ObjectClass *klass, const void *data)
{
    RP2350ClkRegsClass *c = RP2350_CLKREGS_CLASS(klass);
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
}

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
        .class_init    = rp2350_pll_class_init,
    },
    {
        .name          = TYPE_RP2350_TICKS,
        .parent        = TYPE_RP2350_CLKREGS,
        .class_init    = rp2350_ticks_class_init,
    },
};

DEFINE_TYPES(rp2350_clocks_types)
