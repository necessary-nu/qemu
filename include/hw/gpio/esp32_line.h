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
 */
int64_t esp32_line_time(void);
void esp32_line_set_at(qemu_irq line, int level, int64_t when_ns);
void esp32_line_run_at(int64_t when_ns, void (*fn)(void *opaque),
                       void *opaque);
void esp32_line_add_source(Notifier *n);
void esp32_line_sync(void);

/*
 * Changes that reach a device while it resolves another.
 *
 * A device that resolves its lines in rounds (a GPIO matrix) is re-entered
 * when a block a round reaches answers by driving a line back. An answer
 * at the round's time belongs with it, as another round at that time. An
 * answer at another time, such as the edges of a source that was behind
 * and catches up as it takes the round's change, would take the round's
 * time if the device took it into its state at once: the device holds it
 * back with its own time (esp32_line_defer(), from the input's handler),
 * and once the round has settled resolves the held-back changes, the
 * earliest time first, each time's in a round of its own: for each time
 * esp32_line_deferred_next() gives, it replays them with
 * esp32_line_deferred_replay() and resolves the result at that time.
 */
typedef void Esp32LineInput(void *opaque, int n, int level);

typedef struct Esp32LineDeferred {
    /* Esp32LineChange, in order of time, then of arrival, from head on */
    GArray *changes;
    guint head;
} Esp32LineDeferred;

void esp32_line_deferred_init(Esp32LineDeferred *d);
void esp32_line_deferred_finalize(Esp32LineDeferred *d);
void esp32_line_defer(Esp32LineDeferred *d, Esp32LineInput *fn,
                      void *opaque, int n, int level);
bool esp32_line_deferred_next(Esp32LineDeferred *d, int64_t *when_ns);
void esp32_line_deferred_replay(Esp32LineDeferred *d, int64_t when_ns);

#endif
