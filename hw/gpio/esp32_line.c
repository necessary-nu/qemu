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
void esp32_line_run_at(int64_t when_ns, void (*fn)(void *opaque),
                       void *opaque)
{
    int64_t saved = line_stamp;

    line_stamp = when_ns;
    fn(opaque);
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

/* A change held back, to be replayed at its time: fn(opaque, n, level). */
typedef struct Esp32LineChange {
    int64_t when_ns;
    Esp32LineInput *fn;
    void *opaque;
    int n;
    int level;
} Esp32LineChange;

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
void esp32_line_deferred_init(Esp32LineDeferred *d)
{
    d->changes = g_array_new(false, false, sizeof(Esp32LineChange));
    d->head = 0;
}

void esp32_line_deferred_finalize(Esp32LineDeferred *d)
{
    g_array_free(d->changes, true);
    d->changes = NULL;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Hold back a change at esp32_line_time(): after every change held back
 * at that time or earlier. A source catching up holds its changes back in
 * the order of their times, so that they go at the end.
 */
void esp32_line_defer(Esp32LineDeferred *d, Esp32LineInput *fn,
                      void *opaque, int n, int level)
{
    Esp32LineChange c = {
        .when_ns = esp32_line_time(), .fn = fn, .opaque = opaque,
        .n = n, .level = level,
    };
    guint lo = d->head, hi = d->changes->len;

    while (lo < hi) {
        guint mid = lo + (hi - lo) / 2;

        if (g_array_index(d->changes, Esp32LineChange, mid).when_ns <=
            c.when_ns) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    g_array_insert_val(d->changes, lo, c);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The earliest time a change is held back at; false when none is.
 */
bool esp32_line_deferred_next(Esp32LineDeferred *d, int64_t *when_ns)
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
    *when_ns = g_array_index(d->changes, Esp32LineChange, d->head).when_ns;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Replay the changes held back at when_ns, in the order they came: each
 * reaches its handler again, which, the device resolving a round at
 * when_ns, takes it. They are taken out first, as a replay can hold back
 * more.
 */
void esp32_line_deferred_replay(Esp32LineDeferred *d, int64_t when_ns)
{
    guint start = d->head, end = start;
    g_autofree Esp32LineChange *batch = NULL;

    while (end < d->changes->len &&
           g_array_index(d->changes, Esp32LineChange, end).when_ns ==
           when_ns) {
        end++;
    }
    if (end == start) {
        return;
    }
    batch = g_memdup2(&g_array_index(d->changes, Esp32LineChange, start),
                      (end - start) * sizeof(Esp32LineChange));
    d->head = end;
    for (guint i = 0; i < end - start; i++) {
        batch[i].fn(batch[i].opaque, batch[i].n, batch[i].level);
    }
}
