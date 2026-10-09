/*
 * RP2350 subsystem resets (RESETS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Subsystem resets".
 *
 * Each subsystem's reset is held asserted while its RESET bit is set, and
 * every subsystem starts in reset. Deasserting a reset releases the
 * subsystem RP2350_RESETS_RELEASE_NS later, when its RESET_DONE bit sets.
 * What a subsystem does while it is in reset is up to the owner of the
 * hold callback, which is told when subsystems enter and leave reset.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_resets.h"
#include "migration/vmstate.h"

#define A_RESET      0x0
#define A_WDSEL      0x4
#define A_RESET_DONE 0x8

/* Bring the subsystems' hold state in line with `held`. */
static void resets_sync(RP2350ResetsState *s)
{
    uint32_t enter = s->held & ~s->asserted;
    uint32_t leave = s->asserted & ~s->held;

    s->asserted = s->held;
    if (!s->hold_fn) {
        return;
    }
    if (leave) {
        s->hold_fn(s->hold_opaque, leave, false);
    }
    if (enter) {
        s->hold_fn(s->hold_opaque, enter, true);
    }
}

static void resets_schedule(RP2350ResetsState *s)
{
    int64_t due = INT64_MAX;
    int i;

    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        if (s->release_ns[i] >= 0) {
            due = MIN(due, s->release_ns[i]);
        }
    }
    if (due == INT64_MAX) {
        timer_del(s->release_timer);
    } else {
        timer_mod(s->release_timer, due);
    }
}

/*
 * Apply a new RESET value: asserting a reset puts the subsystem into reset
 * at once; deasserting one starts its release, unless it is already
 * leaving reset.
 */
/* [spec:nuos:req:emu.resets] */
static void resets_update(RP2350ResetsState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        if (s->reset & BIT(i)) {
            s->held |= BIT(i);
            s->release_ns[i] = -1;
        } else if ((s->held & BIT(i)) && s->release_ns[i] < 0) {
            s->release_ns[i] = now + RP2350_RESETS_RELEASE_NS;
        }
    }
    resets_sync(s);
    resets_schedule(s);
}

/* [spec:nuos:req:emu.resets] */
static void resets_release_cb(void *opaque)
{
    RP2350ResetsState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        if (s->release_ns[i] >= 0 && s->release_ns[i] <= now) {
            s->release_ns[i] = -1;
            s->held &= ~BIT(i);
        }
    }
    resets_sync(s);
    resets_schedule(s);
}

/* [spec:nuos:req:emu.resets] */
void rp2350_resets_pulse(RP2350ResetsState *s, uint32_t blocks)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    blocks &= RP2350_RESETS_ALL & ~s->reset;
    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        if (blocks & BIT(i)) {
            s->release_ns[i] = now + RP2350_RESETS_RELEASE_NS;
        }
    }
    /* A subsystem already in reset is restarted by leaving and entering. */
    s->held &= ~blocks;
    resets_sync(s);
    s->held |= blocks;
    resets_sync(s);
    resets_schedule(s);
}

/* The boot ROM waits for RESET_DONE, so these leave reset at once. */
void rp2350_resets_boot_rom_handoff(RP2350ResetsState *s)
{
    uint32_t blocks = s->rom_unreset;
    int i;

    s->held |= blocks;
    resets_sync(s);
    s->reset &= ~blocks;
    s->held &= ~blocks;
    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        if (blocks & BIT(i)) {
            s->release_ns[i] = -1;
        }
    }
    resets_sync(s);
    resets_schedule(s);
}

void rp2350_resets_set_hold_fn(RP2350ResetsState *s, RP2350ResetsHoldFn *fn,
                               void *opaque)
{
    s->hold_fn = fn;
    s->hold_opaque = opaque;
}

static uint64_t rp2350_resets_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350ResetsState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_RESET:
        return s->reset;
    case A_WDSEL:
        return s->wdsel;
    case A_RESET_DONE:
        /* [spec:nuos:req:emu.resets] */
        return ~s->held & RP2350_RESETS_ALL;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-resets: read of bad offset 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void rp2350_resets_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    RP2350ResetsState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_RESET:
        s->reset = rp2350_atomic_apply(addr, s->reset, value) &
                   RP2350_RESETS_ALL;
        resets_update(s);
        break;
    case A_WDSEL:
        s->wdsel = rp2350_atomic_apply(addr, s->wdsel, value) &
                   RP2350_RESETS_ALL;
        break;
    case A_RESET_DONE:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-resets: write to bad offset 0x%" HWADDR_PRIx
                      "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_resets_ops = {
    .read = rp2350_resets_read,
    .write = rp2350_resets_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * Every subsystem enters reset, apart from those the boot ROM would have
 * taken out of reset by the time the image runs. `asserted` is left alone:
 * it tracks the subsystems' own reset state, which the exit phase brings
 * in line once every device has been reset.
 */
/* [spec:nuos:req:emu.resets] */
static void rp2350_resets_hold_reset(Object *obj, ResetType type)
{
    RP2350ResetsState *s = RP2350_RESETS(obj);
    int i;

    timer_del(s->release_timer);
    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        s->release_ns[i] = -1;
    }
    s->reset = RP2350_RESETS_ALL & ~s->rom_unreset;
    s->held = s->reset;
    s->wdsel = 0;
}

static void rp2350_resets_exit_reset(Object *obj, ResetType type)
{
    resets_sync(RP2350_RESETS(obj));
}

static void rp2350_resets_init(Object *obj)
{
    RP2350ResetsState *s = RP2350_RESETS(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_resets_ops, s,
                          TYPE_RP2350_RESETS, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->release_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, resets_release_cb, s);
}

static int rp2350_resets_post_load(void *opaque, int version_id)
{
    resets_sync(opaque);
    return 0;
}

static const VMStateDescription vmstate_rp2350_resets = {
    .name = TYPE_RP2350_RESETS,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = rp2350_resets_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reset, RP2350ResetsState),
        VMSTATE_UINT32(wdsel, RP2350ResetsState),
        VMSTATE_UINT32(held, RP2350ResetsState),
        VMSTATE_INT64_ARRAY(release_ns, RP2350ResetsState, RP2350_NUM_RESETS),
        VMSTATE_TIMER_PTR(release_timer, RP2350ResetsState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_resets_properties[] = {
    DEFINE_PROP_UINT32("rom-unreset", RP2350ResetsState, rom_unreset, 0),
};

static void rp2350_resets_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_resets_hold_reset;
    rc->phases.exit = rp2350_resets_exit_reset;
    dc->vmsd = &vmstate_rp2350_resets;
    device_class_set_props(dc, rp2350_resets_properties);
}

static const TypeInfo rp2350_resets_info = {
    .name          = TYPE_RP2350_RESETS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350ResetsState),
    .instance_init = rp2350_resets_init,
    .class_init    = rp2350_resets_class_init,
};

static void rp2350_resets_register_types(void)
{
    type_register_static(&rp2350_resets_info);
}
type_init(rp2350_resets_register_types)
