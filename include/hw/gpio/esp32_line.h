/*
 * Timing of the signal lines between Espressif peripherals and pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_ESP32_LINE_H
#define HW_GPIO_ESP32_LINE_H

#include "hw/core/irq.h"
#include "hw/core/line-time.h"
#include "qemu/notify.h"

/*
 * Line timing.
 *
 * Levels move between peripherals, the matrix, the pads and the board as
 * qemu_irq level changes, which carry no time. A source whose change
 * belongs to a virtual time other than the present, such as a transmitter
 * catching up on bits whose time has passed, or a timer callback that runs
 * late, sets the line with esp32_line_set_at(). Everything that change
 * reaches synchronously sees its time through esp32_line_time(), which is
 * the present when no stamped change is being propagated. The matrix is
 * combinational, so a change keeps its time from source to receiver: a
 * UART receiver times its bits by these stamps, not by when its handlers
 * happen to run.
 *
 * A model that catches up on its own changes, running late what fell due
 * earlier (a timer whose QEMU timer runs late, a filter whose output
 * changed in the past), runs each with esp32_line_run_at(), so that every
 * line it changes and everything those reach sees the change's time, as
 * if each line were set with esp32_line_set_at().
 *
 * A source whose output follows a function of time faster than it is
 * worth timing edge by edge (a PWM output, a clock) registers with
 * esp32_line_add_source(); esp32_line_sync() asks every such source to
 * bring its outputs up to the present, and the matrix calls it whenever
 * software samples the pads, so a sampled pad reads its level at the
 * virtual time of the sample.
 *
 * The ESP32 and the ESP32-P4 GPIO matrices both time their lines this
 * way, so that a peripheral model serves either chip's matrix unchanged.
 * A change that reaches a matrix while it resolves one at another time is
 * held back with its own time (see hw/core/line-time.h).
 */
int64_t esp32_line_time(void);
void esp32_line_set_at(qemu_irq line, int level, int64_t when_ns);
void esp32_line_run_at(int64_t when_ns, void (*fn)(void *opaque),
                       void *opaque);
void esp32_line_add_source(Notifier *n);
void esp32_line_sync(void);

#endif
