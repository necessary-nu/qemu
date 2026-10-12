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
#include "migration/vmstate.h"

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

/*
 * Changes that reach a device while it resolves another.
 *
 * A device that resolves its lines in rounds (pin muxing, a GPIO matrix)
 * is re-entered when a block a round reaches answers by driving a line
 * back. An answer at the round's time belongs with it, as another round
 * at that time. An answer at another time, such as the edges of a source
 * that was behind and catches up as it takes the round's change, would
 * take the round's time if the device took it into its state at once:
 * the device holds it back with its own time (line_defer(), from the
 * input's handler), and once the round has settled resolves the held-back
 * changes, the earliest time first, each time's in a round of its own:
 * for each time line_deferred_next() gives, it replays them with
 * line_deferred_replay() and resolves the result at that time.
 */
typedef void LineInput(void *opaque, int n, int level);

typedef struct LineDeferred {
    /* LineChange, in order of time, then of arrival, from head on */
    GArray *changes;
    guint head;
} LineDeferred;

void line_deferred_init(LineDeferred *d);
void line_deferred_finalize(LineDeferred *d);
void line_defer(LineDeferred *d, LineInput *fn, void *opaque, int n,
                int level, int64_t when_ns);
bool line_deferred_next(LineDeferred *d, int64_t *when_ns);
void line_deferred_replay(LineDeferred *d, int64_t when_ns);

/*
 * An input taken at the receiver's own times.
 *
 * A device that acts on an input at times of its own, such as the ends of
 * the steps its timer marks, takes the input's level as it was then. Its
 * timer can fire after a change at a later time has come in: under
 * icount an access comes up to an instruction after a deadline whose
 * timer has yet to fire. The device keeps the input's latest changes
 * with their times (line_history_set(), from the input's handler, at
 * line_time()), and reads its level at a time with line_history_at().
 */
#define LINE_HISTORY 4

typedef struct LineHistory {
    /* The level now, and before the changes kept */
    bool level;
    bool before;
    /* The changes kept, oldest first: their times and the levels set */
    uint32_t n;
    int64_t when[LINE_HISTORY];
    bool set[LINE_HISTORY];
} LineHistory;

void line_history_reset(LineHistory *h, bool level);
bool line_history_set(LineHistory *h, bool level, int64_t when_ns);
bool line_history_at(const LineHistory *h, int64_t when_ns);

/* The changes kept, for a device's VMState; `level` goes on its own. */
extern const VMStateDescription vmstate_line_history;

#endif
