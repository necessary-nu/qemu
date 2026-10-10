/*
 * Timing of the signal lines between Espressif peripherals and pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/gpio/esp32_line.h.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/gpio/esp32_line.h"

/*
 * The time of the line change being propagated, while esp32_line_set_at()
 * propagates one; INT64_MIN otherwise. Changes nest: a receiver can answer
 * a stamped change with a change of its own.
 */
static int64_t line_stamp = INT64_MIN;
static NotifierList line_sources = NOTIFIER_LIST_INITIALIZER(line_sources);
static bool line_syncing;

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
int64_t esp32_line_time(void)
{
    if (line_stamp != INT64_MIN) {
        return line_stamp;
    }
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
void esp32_line_set_at(qemu_irq line, int level, int64_t when_ns)
{
    int64_t saved = line_stamp;

    line_stamp = when_ns;
    qemu_set_irq(line, level);
    line_stamp = saved;
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
void esp32_line_add_source(Notifier *n)
{
    notifier_list_add(&line_sources, n);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Bring every registered source's outputs up to the present. A source
 * syncing may change lines whose receivers sample the pads again; that
 * inner sample sees the sources as they are.
 */
void esp32_line_sync(void)
{
    if (line_syncing) {
        return;
    }
    line_syncing = true;
    notifier_list_notify(&line_sources, NULL);
    line_syncing = false;
}
