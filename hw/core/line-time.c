/*
 * Timing of level changes on signal lines
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/core/line-time.h.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/core/line-time.h"

/*
 * The time of the change being propagated, while line_set_at() or
 * line_run_at() propagates one; INT64_MIN otherwise. Changes nest: a
 * receiver can answer a stamped change with a change of its own.
 */
static int64_t line_stamp = INT64_MIN;

int64_t line_time(void)
{
    if (line_stamp != INT64_MIN) {
        return line_stamp;
    }
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

void line_set_at(qemu_irq line, int level, int64_t when_ns)
{
    int64_t outer = line_stamp;

    line_stamp = when_ns;
    qemu_set_irq(line, level);
    line_stamp = outer;
}

void line_run_at(int64_t when_ns, void (*fn)(void *opaque), void *opaque)
{
    int64_t outer = line_stamp;

    line_stamp = when_ns;
    fn(opaque);
    line_stamp = outer;
}
