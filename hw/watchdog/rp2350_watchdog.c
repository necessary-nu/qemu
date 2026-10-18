/*
 * RP2350 watchdog (WATCHDOG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Watchdog". The counter in CTRL.TIME
 * counts down once per tick of the TICKS block's WATCHDOG generator, which
 * divides clk_ref by its CYCLES setting (the "tick" clock), at the tick's
 * current rate, a change of the tick counting from the change's time
 * (line_time()). When the counter reaches zero, or
 * TRIGGER is written, the watchdog records why in REASON and requests a
 * reset, which the PSM carries out (see rp2350_psm.c) unless POWMAN.WDSEL
 * makes it a chip-level reset (see rp2350_powman.c). The watchdog itself
 * is reset only by a chip-level reset (a QEMU system reset, or the power
 * manager's chip-reset input for a glitch detector trigger or a
 * watchdog RESET_PSM), so its scratch registers and REASON survive the
 * PSM and RESETS resets it requests, and are lost, as on hardware, when
 * it requests a chip-level reset.
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
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/line-time.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/watchdog/rp2350_watchdog.h"
#include "migration/vmstate.h"
#include "system/cpus.h"

#define A_CTRL       0x00
#define A_LOAD       0x04
#define A_REASON     0x08
#define A_SCRATCH0   0x0c
#define A_SCRATCH7   0x28

#define CTRL_TRIGGER (1u << 31)
#define CTRL_ENABLE  (1u << 30)
#define CTRL_PAUSE   0x07000000u /* PAUSE_DBG1, PAUSE_DBG0, PAUSE_JTAG */
#define CTRL_TIME    0x00ffffffu

/* [spec:nuos:req:emu.clock-tree] */
static bool wd_should_run(RP2350WatchdogState *s)
{
    return (s->ctrl & CTRL_ENABLE) && clock_is_enabled(s->tick);
}

/* Virtual time taken by n ticks at the current rate, rounded up. */
static int64_t wd_ticks_ns(RP2350WatchdogState *s, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(s->tick, n);

    while (clock_ns_to_ticks(s->tick, ns) < n) {
        ns++;
    }
    return MIN(ns, (uint64_t)INT64_MAX / 2);
}

/* Whole ticks since sync_ns. */
static uint64_t wd_elapsed(RP2350WatchdogState *s, int64_t now)
{
    if (!s->running || now <= s->sync_ns) {
        return 0;
    }
    return clock_ns_to_ticks(s->tick, now - s->sync_ns);
}

static uint32_t wd_time(RP2350WatchdogState *s, int64_t now)
{
    uint64_t n = wd_elapsed(s, now);

    return n >= s->time ? 0 : s->time - n;
}

/*
 * Fold elapsed ticks into the counter, keeping sync_ns on a tick boundary
 * so that partial ticks are not lost. Called before anything that changes
 * the count, the rate or whether the counter runs, at the time of the
 * access or change at hand; never before sync_ns, which the counter has
 * already counted up to.
 */
static void wd_sync(RP2350WatchdogState *s)
{
    int64_t now = MAX(line_time(), s->sync_ns);
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

/*
 * The WATCHDOG tick started, stopped or changed rate, at the change's
 * time however late the event making it ran: count up to the change at
 * the old rate, and the next tick afresh at the new one.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void wd_tick_changed(void *opaque, ClockEvent event)
{
    RP2350WatchdogState *s = opaque;

    if (event == ClockPreUpdate) {
        wd_sync(s);
        return;
    }
    s->sync_ns = MAX(line_time(), s->sync_ns);
    wd_restart(s);
}

/*
 * [spec:nuos:req:emu.watchdog]
 * The count has reached zero by the access being made: the watchdog fires
 * first, as its timer, which without icount can run well after, would
 * have. A LOAD written after that moment comes too late to keep the
 * reset off, as on the chip, and CTRL and REASON read as the expiry left
 * them. Only a timer that has come fires.
 *
 * A reset the expiry asks of the processor making the access comes before
 * the access, which the chip never makes: the PSM's reset of the
 * processor, queued as work for it, or a chip-level reset, which stops
 * it. The access is then held (cpu_hold_access()): the processor stops
 * at the instruction, which has not executed, for the reset. True if it
 * is.
 *
 * First the clock tree comes up to the access (clock_sync()), so that
 * the count follows a change of its tick that has fallen due from the
 * change's time.
 */
static bool wd_catch_up(RP2350WatchdogState *s, MemTxAttrs attrs)
{
    CPUState *cs = current_cpu;

    clock_sync(s->tick);
    if (!timer_pending(s->expiry) ||
        timer_expire_time_ns(s->expiry) >
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) {
        return false;
    }
    timer_del(s->expiry);
    wd_expired(s);
    if (!cs || !qemu_cpu_is_self(cs) || attrs.debug || attrs.unspecified ||
        (cpu_work_list_empty(cs) && !cs->stop)) {
        return false;
    }
    cpu_hold_access(cs);
    return true;
}

static uint32_t wd_ctrl(RP2350WatchdogState *s)
{
    return s->ctrl | wd_time(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* [spec:nuos:req:emu.watchdog] */
static MemTxResult rp2350_watchdog_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    RP2350WatchdogState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);

    if (wd_catch_up(s, attrs)) {
        return MEMTX_ERROR;
    }
    switch (reg) {
    case A_CTRL:
        *data = wd_ctrl(s);
        return MEMTX_OK;
    case A_LOAD:
        /* Write-only: the loaded value is visible in CTRL.TIME. */
        *data = 0;
        return MEMTX_OK;
    case A_REASON:
        *data = s->reason;
        return MEMTX_OK;
    case A_SCRATCH0 ... A_SCRATCH7:
        *data = s->scratch[(reg - A_SCRATCH0) / 4];
        return MEMTX_OK;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-watchdog: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    *data = 0;
    return MEMTX_OK;
}

static MemTxResult rp2350_watchdog_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    RP2350WatchdogState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v;

    if (wd_catch_up(s, attrs)) {
        return MEMTX_ERROR;
    }
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
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_watchdog_ops = {
    .read_with_attrs = rp2350_watchdog_read,
    .write_with_attrs = rp2350_watchdog_write,
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
    wd_restart(RP2350_WATCHDOG(obj));
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

    if (!clock_has_source(s->tick)) {
        error_setg(errp, "the tick clock must be connected");
        return;
    }
    s->expiry = timer_new_ns(QEMU_CLOCK_VIRTUAL, wd_expired, s);
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
    s->tick = qdev_init_clock_in(DEVICE(obj), "tick", wd_tick_changed, s,
                                 ClockPreUpdate | ClockUpdate);
}

static const VMStateDescription vmstate_rp2350_watchdog = {
    .name = TYPE_RP2350_WATCHDOG,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(tick, RP2350WatchdogState),
        VMSTATE_TIMER_PTR(expiry, RP2350WatchdogState),
        VMSTATE_UINT32(ctrl, RP2350WatchdogState),
        VMSTATE_UINT32(time, RP2350WatchdogState),
        VMSTATE_INT64(sync_ns, RP2350WatchdogState),
        VMSTATE_BOOL(running, RP2350WatchdogState),
        VMSTATE_UINT32(reason, RP2350WatchdogState),
        VMSTATE_UINT32_ARRAY(scratch, RP2350WatchdogState,
                             RP2350_WATCHDOG_SCRATCH),
        VMSTATE_END_OF_LIST()
    },
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
