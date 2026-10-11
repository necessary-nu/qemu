/*
 * ESP32-P4 clock tree: exact clock periods and cycle counts
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/misc/esp32p4_clk_period.h: the clock source interface,
 * and the migration state of a cycle count.
 */

#include "qemu/osdep.h"
#include "hw/misc/esp32p4_clk_period.h"

const VMStateDescription vmstate_esp32p4_clk_period = {
    .name = "esp32p4-clk-period",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(num_hi, ESP32P4ClkPeriod),
        VMSTATE_UINT64(num_lo, ESP32P4ClkPeriod),
        VMSTATE_UINT64(den, ESP32P4ClkPeriod),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_esp32p4_clk_count = {
    .name = "esp32p4-clk-count",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(period, ESP32P4ClkCount, 0,
                       vmstate_esp32p4_clk_period, ESP32P4ClkPeriod),
        VMSTATE_INT64(origin_ns, ESP32P4ClkCount),
        VMSTATE_END_OF_LIST()
    }
};

/* [spec:nuos:req:emu.esp32p4.clocks] */
static const TypeInfo esp32p4_clk_source_info = {
    .name = TYPE_ESP32P4_CLK_SOURCE,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(ESP32P4ClkSourceClass),
};

static void esp32p4_clk_period_register_types(void)
{
    type_register_static(&esp32p4_clk_source_info);
}

type_init(esp32p4_clk_period_register_types)
