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
#include "qemu/notify.h"
#include "qemu/queue.h"
#include "qemu/timer.h"

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
 * The time of the event at hand, for a receiver that must know when an
 * event fell due however late it came, as a core's sleep ends at the time
 * of the interrupt that woke it: a stamped change's time; otherwise, in
 * the callback of a QEMU_CLOCK_VIRTUAL timer, the timer's deadline, the
 * time of the event a device set it for, which without icount the
 * callback can run well after; otherwise the present.
 */
int64_t line_event_time(void);

/*
 * Sources a receiver brings up to its time.
 *
 * A transmitter times its bits on QEMU timers of its own, which without
 * icount run late, so a receiver in another device would see a line as
 * the host had got to it. A source registers with line_add_source(); its
 * notifier puts out every change of its outputs due by line_time(), each
 * at its own time (line_set_at()), and nothing else: not what it takes
 * in, as a receiver it is brought up to time by its own accesses. A
 * receiver's register access calls line_sync() before it takes its own
 * events due, so that what it shows is what its lines brought by then.
 * line_sync() fixes the moment, line_time() as it is called, for every
 * source and returns it: the receiver takes its own events up to that
 * same moment, not to one read afresh afterwards, which without icount
 * lies later and would see samples taken ahead of bits a source has yet
 * to put out.
 */
void line_add_source(Notifier *n);
void line_remove_source(Notifier *n);
int64_t line_sync(void);

/*
 * Events of several devices in one order.
 *
 * A device catching up at a register access runs its own events due by
 * then, each at its time and in order. Where another device's events act
 * on it, as one watchdog's reset resets a block whose own watchdog would
 * fire a little after, two such events due close together must run in
 * the order of their times across the devices, as their QEMU timers would
 * have run them: each device's stream of events is a LineDueSource, which
 * says when its next event falls due (INT64_MAX for none) and runs it, at
 * its own time, and the devices' sources join one LineDueOrder.
 * line_due_order_run() runs every source's events due by a moment, the
 * earliest first; a device calls it before its own catch-up. It runs
 * nothing while a system reset is pending, which undoes what would come
 * after its request, and called again from inside an event it leaves the
 * order to the run under way.
 *
 * A source whose events each have a QEMU timer at their time can arm it
 * with line_due_order_mod(), which tells the order when the event falls
 * due, and say so (`tells`). While all its sources tell it, the order
 * knows when the earliest of their events can fall due, and a run to a
 * moment before then asks none of them: line_due_order_catch_up(), which
 * runs the events due by line_time(), costs a comparison while nothing
 * has fallen due, and while nothing is pending does not read the clock.
 * Devices that bring the order up to time at each register access, as
 * those timing from a clock its sources change do, cost nothing more
 * while it has nothing to run.
 */
typedef struct LineDueSource LineDueSource;

struct LineDueSource {
    int64_t (*next_due)(LineDueSource *src);
    void (*run)(LineDueSource *src);
    /* The source arms its events' timers with line_due_order_mod(). */
    bool tells;
    QLIST_ENTRY(LineDueSource) next;
};

typedef struct LineDueOrder {
    QLIST_HEAD(, LineDueSource) sources;
    bool running;
    /*
     * While every source tells it of its events (untold 0), none falls
     * due before next: INT64_MAX while none is pending, INT64_MIN while
     * that is to be found.
     */
    unsigned untold;
    int64_t next;
} LineDueOrder;

void line_due_order_init(LineDueOrder *o);
void line_due_order_add(LineDueOrder *o, LineDueSource *src);
void line_due_order_run(LineDueOrder *o, int64_t now);
void line_due_order_catch_up(LineDueOrder *o);
void line_due_order_mod(LineDueOrder *o, QEMUTimer *t, int64_t when);

/*
 * A telling source whose event can fall due earlier otherwise than by a
 * timer it arms, as one that counts only while another block is in some
 * state, has the order (if any) ask its sources afresh.
 */
void line_due_order_changed(LineDueOrder *o);

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
