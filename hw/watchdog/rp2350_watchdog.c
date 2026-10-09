/*
 * RP2350 watchdog (WATCHDOG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Watchdog". The counter in CTRL.TIME
 * counts down once per tick of the TICKS block's WATCHDOG generator, which
 * divides clk_ref by its CYCLES setting. When the counter reaches zero, or
 * TRIGGER is written, the watchdog records why in REASON and requests a
 * reset, which the PSM carries out (see rp2350_psm.c). The watchdog itself
 * is reset only by a chip-level reset (a QEMU system reset, or the power
 * manager's chip-reset input for a glitch detector trigger), so its
 * scratch registers and REASON survive the resets it requests.
 *
 * Firing also clears ENABLE: the block is not reset by its own reset, so
 * an enabled, expired counter would otherwise reset the chip again at the
 * next tick, before any software could run. The PAUSE_DBG0, PAUSE_DBG1
 * and PAUSE_JTAG bits are stored only: QEMU stops the virtual clock, and
 * so the counter, whenever the debugger halts the machine.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/watchdog/rp2350_watchdog.h"
#include "migration/vmstate.h"

#define A_CTRL       0x00
#define A_LOAD       0x04
#define A_REASON     0x08
#define A_SCRATCH0   0x0c
#define A_SCRATCH7   0x28

#define CTRL_TRIGGER (1u << 31)
#define CTRL_ENABLE  (1u << 30)
#define CTRL_PAUSE   0x07000000u /* PAUSE_DBG1, PAUSE_DBG0, PAUSE_JTAG */
#define CTRL_TIME    0x00ffffffu

static bool wd_should_run(RP2350WatchdogState *s)
{
    return (s->ctrl & CTRL_ENABLE) && s->cycles &&
           rp2350_ticks_running(s->ticks, RP2350_TICK_WATCHDOG);
}

/* Virtual time taken by n ticks at the current rate, rounded up. */
static int64_t wd_ticks_ns(RP2350WatchdogState *s, uint64_t n)
{
    return muldiv64_round_up(n * s->cycles, NANOSECONDS_PER_SECOND,
                             s->ref_hz);
}

/* Whole ticks since sync_ns. */
static uint64_t wd_elapsed(RP2350WatchdogState *s, int64_t now)
{
    if (!s->running || now <= s->sync_ns) {
        return 0;
    }
    return muldiv64(now - s->sync_ns, s->ref_hz, NANOSECONDS_PER_SECOND) /
           s->cycles;
}

static uint32_t wd_time(RP2350WatchdogState *s, int64_t now)
{
    uint64_t n = wd_elapsed(s, now);

    return n >= s->time ? 0 : s->time - n;
}

/*
 * Fold elapsed ticks into the counter, keeping sync_ns on a tick boundary
 * so that partial ticks are not lost. Called before anything that changes
 * the count, the rate or whether the counter runs.
 */
static void wd_sync(RP2350WatchdogState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t n = wd_elapsed(s, now);

    if (!s->running) {
        s->sync_ns = now;
        return;
    }
    s->time = n >= s->time ? 0 : s->time - n;
    s->sync_ns += wd_ticks_ns(s, n);
}

/*
 * Re-evaluate whether the counter runs and schedule its expiry: on the
 * tick that takes it to zero, or, if it is already zero, the next tick.
 */
static void wd_restart(RP2350WatchdogState *s)
{
    s->running = wd_should_run(s);
    if (!s->running) {
        timer_del(s->expiry);
        return;
    }
    timer_mod(s->expiry, s->sync_ns + wd_ticks_ns(s, s->time ? s->time : 1));
}

/* [spec:nuos:req:emu.watchdog] */
static void wd_fire(RP2350WatchdogState *s, uint32_t reason)
{
    wd_sync(s);
    s->reason = reason;
    s->ctrl &= ~CTRL_ENABLE;
    wd_restart(s);
    qemu_irq_pulse(s->reset_req);
}

static void wd_expired(void *opaque)
{
    RP2350WatchdogState *s = opaque;

    if (s->running) {
        wd_fire(s, RP2350_WATCHDOG_REASON_TIMER);
    }
}

/* The WATCHDOG tick generator started, stopped or changed rate. */
static void wd_tick_changed(void *opaque)
{
    RP2350WatchdogState *s = opaque;

    wd_sync(s);
    s->cycles = rp2350_ticks_cycles(s->ticks, RP2350_TICK_WATCHDOG);
    wd_restart(s);
}

static uint32_t wd_ctrl(RP2350WatchdogState *s)
{
    return s->ctrl | wd_time(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* [spec:nuos:req:emu.watchdog] */
static uint64_t rp2350_watchdog_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350WatchdogState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);

    switch (reg) {
    case A_CTRL:
        return wd_ctrl(s);
    case A_LOAD:
        /* Write-only: the loaded value is visible in CTRL.TIME. */
        return 0;
    case A_REASON:
        return s->reason;
    case A_SCRATCH0 ... A_SCRATCH7:
        return s->scratch[(reg - A_SCRATCH0) / 4];
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-watchdog: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void rp2350_watchdog_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    RP2350WatchdogState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v;

    switch (reg) {
    case A_CTRL:
        v = rp2350_atomic_apply(addr, wd_ctrl(s), value);
        wd_sync(s);
        s->ctrl = v & (CTRL_ENABLE | CTRL_PAUSE);
        wd_restart(s);
        /* TRIGGER self-clears: it reads as 0. */
        if (v & CTRL_TRIGGER) {
            wd_fire(s, RP2350_WATCHDOG_REASON_FORCE);
        }
        break;
    case A_LOAD:
        wd_sync(s);
        s->time = rp2350_atomic_apply(addr, 0, value) & CTRL_TIME;
        wd_restart(s);
        break;
    case A_REASON:
        break;
    case A_SCRATCH0 ... A_SCRATCH7:
        v = (reg - A_SCRATCH0) / 4;
        s->scratch[v] = rp2350_atomic_apply(addr, s->scratch[v], value);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-watchdog: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_watchdog_ops = {
    .read = rp2350_watchdog_read,
    .write = rp2350_watchdog_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_watchdog_hold_reset(Object *obj, ResetType type)
{
    RP2350WatchdogState *s = RP2350_WATCHDOG(obj);

    timer_del(s->expiry);
    s->ctrl = CTRL_PAUSE;
    s->time = 0;
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->running = false;
    s->reason = 0;
    memset(s->scratch, 0, sizeof(s->scratch));
}

static void rp2350_watchdog_exit_reset(Object *obj, ResetType type)
{
    RP2350WatchdogState *s = RP2350_WATCHDOG(obj);

    s->cycles = rp2350_ticks_cycles(s->ticks, RP2350_TICK_WATCHDOG);
    wd_restart(s);
}

/*
 * A chip-level reset that the power manager carries out without a system
 * reset.
 */
/* [spec:nuos:req:emu.watchdog] */
static void rp2350_watchdog_chip_reset(void *opaque, int n, int level)
{
    if (level) {
        device_cold_reset(DEVICE(opaque));
    }
}

static void rp2350_watchdog_realize(DeviceState *dev, Error **errp)
{
    RP2350WatchdogState *s = RP2350_WATCHDOG(dev);

    if (!s->ticks) {
        error_setg(errp, "ticks property was not set");
        return;
    }
    if (!s->ref_hz) {
        error_setg(errp, "ref-hz property must be set");
        return;
    }
    s->expiry = timer_new_ns(QEMU_CLOCK_VIRTUAL, wd_expired, s);
    rp2350_ticks_set_notify(s->ticks, RP2350_TICK_WATCHDOG, wd_tick_changed,
                            s);
}

static void rp2350_watchdog_init(Object *obj)
{
    RP2350WatchdogState *s = RP2350_WATCHDOG(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_watchdog_ops, s,
                          TYPE_RP2350_WATCHDOG, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out(DEVICE(obj), &s->reset_req, 1);
    qdev_init_gpio_in_named(DEVICE(obj), rp2350_watchdog_chip_reset,
                            RP2350_WATCHDOG_CHIP_RESET, 1);
}

static const VMStateDescription vmstate_rp2350_watchdog = {
    .name = TYPE_RP2350_WATCHDOG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(expiry, RP2350WatchdogState),
        VMSTATE_UINT32(ctrl, RP2350WatchdogState),
        VMSTATE_UINT32(time, RP2350WatchdogState),
        VMSTATE_INT64(sync_ns, RP2350WatchdogState),
        VMSTATE_BOOL(running, RP2350WatchdogState),
        VMSTATE_UINT32(cycles, RP2350WatchdogState),
        VMSTATE_UINT32(reason, RP2350WatchdogState),
        VMSTATE_UINT32_ARRAY(scratch, RP2350WatchdogState,
                             RP2350_WATCHDOG_SCRATCH),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_watchdog_properties[] = {
    DEFINE_PROP_LINK("ticks", RP2350WatchdogState, ticks, TYPE_RP2350_TICKS,
                     RP2350ClkRegsState *),
    DEFINE_PROP_UINT32("ref-hz", RP2350WatchdogState, ref_hz, 0),
};

/* [spec:nuos:req:emu.watchdog] */
static void rp2350_watchdog_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_watchdog_realize;
    dc->vmsd = &vmstate_rp2350_watchdog;
    rc->phases.hold = rp2350_watchdog_hold_reset;
    rc->phases.exit = rp2350_watchdog_exit_reset;
    device_class_set_props(dc, rp2350_watchdog_properties);
}

static const TypeInfo rp2350_watchdog_info = {
    .name          = TYPE_RP2350_WATCHDOG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350WatchdogState),
    .instance_init = rp2350_watchdog_init,
    .class_init    = rp2350_watchdog_class_init,
};

static void rp2350_watchdog_register_types(void)
{
    type_register_static(&rp2350_watchdog_info);
}
type_init(rp2350_watchdog_register_types)
