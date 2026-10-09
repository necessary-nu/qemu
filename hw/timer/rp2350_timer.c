/*
 * RP2350 system timer (TIMER0, TIMER1)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "System timers". The 64-bit counter
 * follows QEMU's virtual clock, counting microseconds from its TICKS
 * generator (or clk_sys cycles when SOURCE selects it), and stops while
 * paused or while that tick generator is disabled. Debug pause is not
 * modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/timer/rp2350_timer.h"
#include "migration/vmstate.h"

#define A_TIMEHW    0x00
#define A_TIMELW    0x04
#define A_TIMEHR    0x08
#define A_TIMELR    0x0c
#define A_ALARM0    0x10
#define A_ALARM3    0x1c
#define A_ARMED     0x20
#define A_TIMERAWH  0x24
#define A_TIMERAWL  0x28
#define A_DBGPAUSE  0x2c
#define A_PAUSE     0x30
#define A_LOCKED    0x34
#define A_SOURCE    0x38
#define A_INTR      0x3c
#define A_INTE      0x40
#define A_INTF      0x44
#define A_INTS      0x48

#define ALARM_MASK  0xf
#define TICK_HZ     1000000

static uint64_t timer_hz(RP2350TimerState *s)
{
    return s->source ? s->sysclk_hz : TICK_HZ;
}

/* The counter stops while the timer is held in reset. */
/* [spec:nuos:req:emu.resets] */
static bool timer_should_run(RP2350TimerState *s)
{
    if (s->pause || device_is_in_reset(DEVICE(s))) {
        return false;
    }
    return s->source || rp2350_ticks_running(s->ticks, s->tick);
}

static uint64_t timer_count(RP2350TimerState *s, int64_t now)
{
    if (!s->running) {
        return s->count;
    }
    return s->count + muldiv64(now - s->sync_ns, timer_hz(s),
                               NANOSECONDS_PER_SECOND);
}

static void timer_update_irqs(RP2350TimerState *s)
{
    uint32_t ints = (s->intr | s->intf) & s->inte;
    int i;

    for (i = 0; i < RP2350_TIMER_ALARMS; i++) {
        qemu_set_irq(s->irq[i], (ints >> i) & 1);
    }
}

static void timer_fire(RP2350TimerState *s, int n)
{
    s->armed &= ~(1u << n);
    s->intr |= 1u << n;
    timer_update_irqs(s);
}

/*
 * Schedule each armed alarm for when the counter's low word next matches
 * it. The deadline is rounded up, so the counter has reached the target
 * by the time the callback runs, however late that is.
 */
static void timer_schedule(RP2350TimerState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t lo = timer_count(s, now);
    uint64_t hz = timer_hz(s);
    int i;

    for (i = 0; i < RP2350_TIMER_ALARMS; i++) {
        uint32_t delta;

        if (!(s->armed & (1u << i)) || !s->running) {
            timer_del(s->alarm_timer[i]);
            s->alarm_due_ns[i] = -1;
            continue;
        }
        delta = s->alarm[i] - lo;
        if (delta == 0) {
            timer_del(s->alarm_timer[i]);
            s->alarm_due_ns[i] = -1;
            timer_fire(s, i);
            continue;
        }
        s->alarm_due_ns[i] = now + (delta * NANOSECONDS_PER_SECOND + hz - 1) /
                                   hz;
        timer_mod(s->alarm_timer[i], s->alarm_due_ns[i]);
    }
}

/*
 * Fold elapsed time into the counter and re-evaluate whether it runs.
 * Called before anything that changes the count, rate or run state.
 */
static void timer_sync(RP2350TimerState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->count = timer_count(s, now);
    s->sync_ns = now;
}

static void timer_restart(RP2350TimerState *s)
{
    s->running = timer_should_run(s);
    timer_schedule(s);
}

static void timer_tick_changed(void *opaque)
{
    RP2350TimerState *s = opaque;

    timer_sync(s);
    timer_restart(s);
}

/* Fire every armed alarm whose target the counter has reached. */
static void timer_alarm_cb(void *opaque)
{
    RP2350TimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    for (i = 0; i < RP2350_TIMER_ALARMS; i++) {
        if ((s->armed & (1u << i)) && s->alarm_due_ns[i] >= 0 &&
            s->alarm_due_ns[i] <= now) {
            s->alarm_due_ns[i] = -1;
            timer_fire(s, i);
        }
    }
}

/* [spec:nuos:req:emu.timer] */
static uint64_t rp2350_timer_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350TimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    hwaddr reg = rp2350_atomic_reg(addr);
    uint64_t count;

    switch (reg) {
    case A_TIMEHR:
        return s->latched_hi;
    case A_TIMELR:
        count = timer_count(s, now);
        s->latched_hi = count >> 32;
        return (uint32_t)count;
    case A_ALARM0 ... A_ALARM3:
        return s->alarm[(reg - A_ALARM0) / 4];
    case A_ARMED:
        return s->armed;
    case A_TIMERAWH:
        return timer_count(s, now) >> 32;
    case A_TIMERAWL:
        return (uint32_t)timer_count(s, now);
    case A_DBGPAUSE:
        return s->dbgpause;
    case A_PAUSE:
        return s->pause;
    case A_LOCKED:
        return s->locked;
    case A_SOURCE:
        return s->source;
    case A_INTR:
        return s->intr;
    case A_INTE:
        return s->inte;
    case A_INTF:
        return s->intf;
    case A_INTS:
        return (s->intr | s->intf) & s->inte;
    case A_TIMEHW:
    case A_TIMELW:
        return 0;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-timer: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void rp2350_timer_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    RP2350TimerState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    int n;

    switch (reg) {
    case A_TIMEHW:
        /* LOCKED makes the counter itself read-only. */
        if (!s->locked) {
            timer_sync(s);
            s->count = ((uint64_t)value << 32) | s->timelw;
            timer_schedule(s);
        }
        break;
    case A_TIMELW:
        if (!s->locked) {
            s->timelw = value;
        }
        break;
    case A_ALARM0 ... A_ALARM3:
        n = (reg - A_ALARM0) / 4;
        s->alarm[n] = rp2350_atomic_apply(addr, s->alarm[n], value);
        s->armed |= 1u << n;
        timer_schedule(s);
        break;
    case A_ARMED:
        /* Write 1 to disarm. */
        s->armed &= ~(value & ALARM_MASK);
        timer_schedule(s);
        break;
    case A_DBGPAUSE:
        s->dbgpause = rp2350_atomic_apply(addr, s->dbgpause, value) & 0x6;
        break;
    case A_PAUSE:
        timer_sync(s);
        s->pause = rp2350_atomic_apply(addr, s->pause, value) & 1;
        timer_restart(s);
        break;
    case A_LOCKED:
        /* Sticky until reset. */
        s->locked |= rp2350_atomic_apply(addr, s->locked, value) & 1;
        break;
    case A_SOURCE:
        timer_sync(s);
        s->source = rp2350_atomic_apply(addr, s->source, value) & 1;
        timer_restart(s);
        break;
    case A_INTR:
        /* Write 1 to clear. */
        s->intr &= ~(value & ALARM_MASK);
        timer_update_irqs(s);
        break;
    case A_INTE:
        s->inte = rp2350_atomic_apply(addr, s->inte, value) & ALARM_MASK;
        timer_update_irqs(s);
        break;
    case A_INTF:
        s->intf = rp2350_atomic_apply(addr, s->intf, value) & ALARM_MASK;
        timer_update_irqs(s);
        break;
    case A_TIMEHR:
    case A_TIMELR:
    case A_TIMERAWH:
    case A_TIMERAWL:
    case A_INTS:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-timer: write to bad offset 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_timer_ops = {
    .read = rp2350_timer_read,
    .write = rp2350_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_timer_hold_reset(Object *obj, ResetType type)
{
    RP2350TimerState *s = RP2350_TIMER(obj);
    int i;

    for (i = 0; i < RP2350_TIMER_ALARMS; i++) {
        timer_del(s->alarm_timer[i]);
        s->alarm[i] = 0;
        s->alarm_due_ns[i] = -1;
    }
    s->count = 0;
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->running = false;
    s->timelw = 0;
    s->latched_hi = 0;
    s->armed = 0;
    s->dbgpause = 0x7;
    s->pause = 0;
    s->locked = 0;
    s->source = 0;
    s->intr = 0;
    s->inte = 0;
    s->intf = 0;
}

/* The counter starts from zero when the timer leaves reset. */
static void rp2350_timer_exit_reset(Object *obj, ResetType type)
{
    RP2350TimerState *s = RP2350_TIMER(obj);

    timer_sync(s);
    timer_restart(s);
    timer_update_irqs(s);
}

static void rp2350_timer_realize(DeviceState *dev, Error **errp)
{
    RP2350TimerState *s = RP2350_TIMER(dev);
    int i;

    if (!s->ticks) {
        error_setg(errp, "ticks property was not set");
        return;
    }
    if (s->tick >= RP2350_NUM_TICKS) {
        error_setg(errp, "tick must be a TICKS generator index");
        return;
    }
    for (i = 0; i < RP2350_TIMER_ALARMS; i++) {
        s->alarm_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_alarm_cb,
                                         s);
    }
    rp2350_ticks_set_notify(s->ticks, s->tick, timer_tick_changed, s);
}

static void rp2350_timer_init(Object *obj)
{
    RP2350TimerState *s = RP2350_TIMER(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &rp2350_timer_ops, s,
                          TYPE_RP2350_TIMER, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (i = 0; i < RP2350_TIMER_ALARMS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
}

static const VMStateDescription vmstate_rp2350_timer = {
    .name = TYPE_RP2350_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR_ARRAY(alarm_timer, RP2350TimerState,
                                RP2350_TIMER_ALARMS),
        VMSTATE_INT64_ARRAY(alarm_due_ns, RP2350TimerState,
                            RP2350_TIMER_ALARMS),
        VMSTATE_UINT64(count, RP2350TimerState),
        VMSTATE_INT64(sync_ns, RP2350TimerState),
        VMSTATE_BOOL(running, RP2350TimerState),
        VMSTATE_UINT32(timelw, RP2350TimerState),
        VMSTATE_UINT32(latched_hi, RP2350TimerState),
        VMSTATE_UINT32_ARRAY(alarm, RP2350TimerState, RP2350_TIMER_ALARMS),
        VMSTATE_UINT32(armed, RP2350TimerState),
        VMSTATE_UINT32(dbgpause, RP2350TimerState),
        VMSTATE_UINT32(pause, RP2350TimerState),
        VMSTATE_UINT32(locked, RP2350TimerState),
        VMSTATE_UINT32(source, RP2350TimerState),
        VMSTATE_UINT32(intr, RP2350TimerState),
        VMSTATE_UINT32(inte, RP2350TimerState),
        VMSTATE_UINT32(intf, RP2350TimerState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_timer_properties[] = {
    DEFINE_PROP_LINK("ticks", RP2350TimerState, ticks, TYPE_RP2350_TICKS,
                     RP2350ClkRegsState *),
    DEFINE_PROP_UINT32("tick", RP2350TimerState, tick, RP2350_NUM_TICKS),
    DEFINE_PROP_UINT32("sysclk-hz", RP2350TimerState, sysclk_hz, 150000000),
};

static void rp2350_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_timer_realize;
    dc->vmsd = &vmstate_rp2350_timer;
    rc->phases.hold = rp2350_timer_hold_reset;
    rc->phases.exit = rp2350_timer_exit_reset;
    device_class_set_props(dc, rp2350_timer_properties);
}

static const TypeInfo rp2350_timer_info = {
    .name          = TYPE_RP2350_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350TimerState),
    .instance_init = rp2350_timer_init,
    .class_init    = rp2350_timer_class_init,
};

static void rp2350_timer_register_types(void)
{
    type_register_static(&rp2350_timer_info);
}
type_init(rp2350_timer_register_types)
