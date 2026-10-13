/*
 * Nsiway NS4150 (NS4150B) filterless class D mono audio amplifier
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: Nsiway's NS4150 datasheet, "CTRL terminal function" (H open,
 * L shutdown) and its electrical characteristics: start-up time from
 * power-down release tST 30 ms typical. Its power-down setting time
 * (tSD, 80 ms minimum) is taken as how long CTRL must stay low for a
 * clean shutdown, not as a delay of the output: the speaker goes quiet
 * as CTRL falls.
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/audio/ns4150.h"
#include "migration/vmstate.h"

#define NS4150_STARTUP_NS   (30 * 1000 * 1000)

static void ns4150_set_on(NS4150State *s, bool on)
{
    if (s->on != on) {
        s->on = on;
        qemu_set_irq(s->on_out, on);
    }
}

static void ns4150_started(void *opaque)
{
    NS4150State *s = opaque;

    ns4150_set_on(s, s->ctrl);
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static void ns4150_ctrl(void *opaque, int n, int level)
{
    NS4150State *s = opaque;

    if (s->ctrl == !!level) {
        return;
    }
    s->ctrl = level;
    if (level) {
        timer_mod_ns(&s->startup, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     s->startup_ns);
    } else {
        timer_del(&s->startup);
        ns4150_set_on(s, false);
    }
}

static void ns4150_reset_hold(Object *obj, ResetType type)
{
    NS4150State *s = NS4150(obj);

    timer_del(&s->startup);
    s->ctrl = false;
    s->on = false;
}

static void ns4150_reset_exit(Object *obj, ResetType type)
{
    NS4150State *s = NS4150(obj);

    qemu_set_irq(s->on_out, 0);
}

static void ns4150_init(Object *obj)
{
    NS4150State *s = NS4150(obj);
    DeviceState *dev = DEVICE(obj);

    qdev_init_gpio_in_named(dev, ns4150_ctrl, NS4150_CTRL, 1);
    qdev_init_gpio_out_named(dev, &s->on_out, NS4150_ON, 1);
    timer_init_ns(&s->startup, QEMU_CLOCK_VIRTUAL, ns4150_started, s);
}

static const VMStateDescription vmstate_ns4150 = {
    .name = TYPE_NS4150,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(startup, NS4150State),
        VMSTATE_BOOL(ctrl, NS4150State),
        VMSTATE_BOOL(on, NS4150State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ns4150_properties[] = {
    DEFINE_PROP_UINT64("startup-ns", NS4150State, startup_ns,
                       NS4150_STARTUP_NS),
};

static void ns4150_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NS4150 class D mono audio amplifier";
    dc->vmsd = &vmstate_ns4150;
    rc->phases.hold = ns4150_reset_hold;
    rc->phases.exit = ns4150_reset_exit;
    device_class_set_props(dc, ns4150_properties);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static const TypeInfo ns4150_types[] = {
    {
        .name = TYPE_NS4150,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NS4150State),
        .instance_init = ns4150_init,
        .class_init = ns4150_class_init,
    },
};

DEFINE_TYPES(ns4150_types)
