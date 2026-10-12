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
#include "migration/vmstate.h"

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

/* A change held back, to be replayed at its time: fn(opaque, n, level). */
typedef struct LineChange {
    int64_t when_ns;
    LineInput *fn;
    void *opaque;
    int n;
    int level;
} LineChange;

void line_deferred_init(LineDeferred *d)
{
    d->changes = g_array_new(false, false, sizeof(LineChange));
    d->head = 0;
}

void line_deferred_finalize(LineDeferred *d)
{
    g_array_free(d->changes, true);
    d->changes = NULL;
}

/*
 * Hold back a change at when_ns: after every change held back at that
 * time or earlier. A source catching up holds its changes back in the
 * order of their times, so that they go at the end.
 */
void line_defer(LineDeferred *d, LineInput *fn, void *opaque, int n,
                int level, int64_t when_ns)
{
    LineChange c = {
        .when_ns = when_ns, .fn = fn, .opaque = opaque,
        .n = n, .level = level,
    };
    guint lo = d->head, hi = d->changes->len;

    while (lo < hi) {
        guint mid = lo + (hi - lo) / 2;

        if (g_array_index(d->changes, LineChange, mid).when_ns <= when_ns) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    g_array_insert_val(d->changes, lo, c);
}

/* The earliest time a change is held back at; false when none is. */
bool line_deferred_next(LineDeferred *d, int64_t *when_ns)
{
    if (d->head == d->changes->len) {
        g_array_set_size(d->changes, 0);
        d->head = 0;
        return false;
    }
    if (d->head > d->changes->len / 2) {
        g_array_remove_range(d->changes, 0, d->head);
        d->head = 0;
    }
    *when_ns = g_array_index(d->changes, LineChange, d->head).when_ns;
    return true;
}

/*
 * Replay the changes held back at when_ns, in the order they came: each
 * reaches its handler again, which, the device resolving a round at
 * when_ns, takes it. They are taken out first, as a replay can hold back
 * more.
 */
void line_deferred_replay(LineDeferred *d, int64_t when_ns)
{
    guint start = d->head, end = start;
    g_autofree LineChange *batch = NULL;

    while (end < d->changes->len &&
           g_array_index(d->changes, LineChange, end).when_ns == when_ns) {
        end++;
    }
    if (end == start) {
        return;
    }
    batch = g_memdup2(&g_array_index(d->changes, LineChange, start),
                      (end - start) * sizeof(LineChange));
    d->head = end;
    for (guint i = 0; i < end - start; i++) {
        batch[i].fn(batch[i].opaque, batch[i].n, batch[i].level);
    }
}

/* Forget the changes kept: the input is at `level` as far back as asked. */
void line_history_reset(LineHistory *h, bool level)
{
    h->level = level;
    h->before = level;
    h->n = 0;
}

/*
 * The input changes to `level` at when_ns; returns whether it changed.
 * The changes are kept in order of time, a change held back from an
 * earlier round coming in after a later one; the oldest goes once
 * LINE_HISTORY are kept.
 */
bool line_history_set(LineHistory *h, bool level, int64_t when_ns)
{
    uint32_t i;

    if (h->level == level) {
        return false;
    }
    h->level = level;
    if (h->n == LINE_HISTORY) {
        h->before = h->set[0];
        memmove(&h->when[0], &h->when[1],
                (LINE_HISTORY - 1) * sizeof(h->when[0]));
        memmove(&h->set[0], &h->set[1],
                (LINE_HISTORY - 1) * sizeof(h->set[0]));
        h->n--;
    }
    for (i = h->n; i > 0 && h->when[i - 1] > when_ns; i--) {
    }
    memmove(&h->when[i + 1], &h->when[i], (h->n - i) * sizeof(h->when[0]));
    memmove(&h->set[i + 1], &h->set[i], (h->n - i) * sizeof(h->set[0]));
    h->when[i] = when_ns;
    h->set[i] = level;
    h->n++;
    return true;
}

/* The input's level at when_ns: a change at when_ns has taken effect. */
bool line_history_at(const LineHistory *h, int64_t when_ns)
{
    int i;

    for (i = h->n - 1; i >= 0; i--) {
        if (h->when[i] <= when_ns) {
            return h->set[i];
        }
    }
    return h->before;
}

static int line_history_post_load(void *opaque, int version_id)
{
    LineHistory *h = opaque;

    if (h->n > LINE_HISTORY) {
        return -EINVAL;
    }
    return 0;
}

const VMStateDescription vmstate_line_history = {
    .name = "line-history",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = line_history_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(before, LineHistory),
        VMSTATE_UINT32(n, LineHistory),
        VMSTATE_INT64_ARRAY(when, LineHistory, LINE_HISTORY),
        VMSTATE_BOOL_ARRAY(set, LineHistory, LINE_HISTORY),
        VMSTATE_END_OF_LIST()
    },
};
