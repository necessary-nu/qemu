/*
 * Parallel LCD panels on an LCD controller's port: the bus and the panels
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/display/lcd_panel.h.
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/lcd_panel.h"
#include "qemu/module.h"

LCDBus *lcd_bus_create(DeviceState *parent, const char *name)
{
    return LCD_BUS(qbus_new(TYPE_LCD_BUS, parent, name));
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
void lcd_bus_foreach(LCDBus *bus, void (*fn)(LCDPanel *p, void *opaque),
                     void *opaque)
{
    BusChild *kid;

    QTAILQ_FOREACH(kid, &BUS(bus)->children, sibling) {
        fn(LCD_PANEL(kid->child), opaque);
    }
}

static const Property lcd_panel_properties[] = {
    DEFINE_PROP_INT32("cs-pad", LCDPanel, cs_pad, -1),
};

static void lcd_panel_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->bus_type = TYPE_LCD_BUS;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    device_class_set_props(dc, lcd_panel_properties);
}

static const TypeInfo lcd_panel_types[] = {
    {
        .name = TYPE_LCD_BUS,
        .parent = TYPE_BUS,
        .instance_size = sizeof(LCDBus),
    },
    {
        .name = TYPE_LCD_PANEL,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(LCDPanel),
        .class_size = sizeof(LCDPanelClass),
        .class_init = lcd_panel_class_init,
        .abstract = true,
    },
};

DEFINE_TYPES(lcd_panel_types)
