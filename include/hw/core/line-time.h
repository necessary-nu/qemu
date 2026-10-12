/*
 * Timing of level changes on signal lines
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CORE_LINE_TIME_H
#define HW_CORE_LINE_TIME_H

#include "hw/core/irq.h"

/*
 * Levels move between devices as qemu_irq level changes, which carry no
 * time. A device whose change belongs to a virtual time other than the
 * present, such as a transmitter putting out a bit on its own bit grid,
 * or a timer callback that runs after its deadline, sets the line with
 * line_set_at(). Everything that change reaches synchronously, through
 * pin muxing, pads and board wiring, sees its time through line_time(),
 * which is the present when no stamped change is being propagated: a
 * receiver times what it takes by the stamp, not by when its handler
 * happens to run.
 *
 * A device catching up on its own changes, running late what fell due
 * earlier, runs each with line_run_at(), so that every line it changes,
 * and everything those reach, sees the change's time.
 *
 * A receiver can only take a change at or after its own state's time: one
 * that has run on to a later moment than a stamp takes the change there.
 * Devices that catch up from a timer therefore run only to the timer's
 * deadline, so that, the timers running in deadline order, their changes
 * reach each other in the order of their times.
 */
int64_t line_time(void);
void line_set_at(qemu_irq line, int level, int64_t when_ns);
void line_run_at(int64_t when_ns, void (*fn)(void *opaque), void *opaque);

#endif
