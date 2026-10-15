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
#include "exec/icount.h"
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

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Whether a stream's event due at @due_ns is more than
 * ESP32_LINE_MAX_LAG_NS behind the present: never under icount, where
 * virtual time waits for every event.
 */
bool esp32_line_lags(int64_t due_ns)
{
    return !icount_enabled() &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - due_ns >
           ESP32_LINE_MAX_LAG_NS;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * An output's own QEMU timer begins a catch-up, at the present, from
 * where its last one left it.
 */
void esp32_line_catching_up(Esp32LineLag *lag)
{
    lag->catching_up = true;
    lag->began_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The catch-up has brought the output to @reached_ns, the present as it
 * began: how far behind the present that leaves the output, and whether
 * that is beyond the slack and further behind than the last one left it,
 * one more catch-up of a run losing ground. Nothing under icount, where
 * nothing falls behind.
 */
void esp32_line_caught_up(Esp32LineLag *lag, int64_t reached_ns)
{
    int64_t behind;

    lag->catching_up = false;
    if (icount_enabled()) {
        return;
    }
    behind = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - reached_ns;
    if (behind > ESP32_LINE_SLACK_NS && behind > lag->behind_ns) {
        lag->losing = MIN(lag->losing + 1, ESP32_LINE_LOSING_RUN);
    } else {
        lag->losing = 0;
    }
    lag->behind_ns = behind;
    lag->reached_ns = reached_ns;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Whether a PWM output skips its edge due at @due_ns: the edge lags
 * (esp32_line_lags()), or it is beyond the slack while the host is losing
 * ground on the output: its last catch-up ended beyond the slack and
 * further behind than the one before, and either the one before that
 * did too, or the catch-up taking the edge has spent longer than the
 * stretch it has covered since the last one, falling further behind
 * still.
 */
bool esp32_line_pwm_skips(const Esp32LineLag *lag, int64_t due_ns)
{
    int64_t now, behind;

    if (icount_enabled()) {
        return false;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    behind = now - due_ns;
    if (behind > ESP32_LINE_MAX_LAG_NS) {
        return true;
    }
    if (behind <= ESP32_LINE_SLACK_NS || !lag->losing) {
        return false;
    }
    return lag->losing >= ESP32_LINE_LOSING_RUN ||
           (lag->catching_up &&
            now - lag->began_ns > due_ns - lag->reached_ns);
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
void esp32_line_wave_init(Esp32LineWave *w, const Esp32LineWaveOps *ops,
                          void *opaque, unsigned index)
{
    w->ops = ops;
    w->opaque = opaque;
    w->index = index;
    notifier_list_init(&w->takers);
}

bool esp32_line_wave_taken(Esp32LineWave *w)
{
    return !QLIST_EMPTY(&w->takers.notifiers);
}

void esp32_line_wave_take(Esp32LineWave *w, Notifier *n)
{
    notifier_list_add(&w->takers, n);
}

void esp32_line_wave_drop(Notifier *n)
{
    notifier_remove(n);
}

static void wave_announce(Esp32LineWave *w, int64_t when, bool done)
{
    Esp32LineWaveChange c = { .wave = w, .when = when, .done = done };

    notifier_list_notify(&w->takers, &c);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The output is about to change at @when: its takers take its edges
 * before then as its description has them.
 */
void esp32_line_wave_changing(Esp32LineWave *w, int64_t when)
{
    wave_announce(w, when, false);
}

/* The output has changed at @when: its takers go on from its new one. */
void esp32_line_wave_changed(Esp32LineWave *w, int64_t when)
{
    wave_announce(w, when, true);
}

/* [spec:nuos:req:emu.esp32.gpio] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
bool esp32_sig_out_watched(Object *matrix, unsigned sig)
{
    return ESP32_SIG_ROUTES_GET_CLASS(matrix)->sig_out_watched(matrix, sig);
}

void esp32_sig_add_route_notifier(Object *matrix, Notifier *n)
{
    ESP32_SIG_ROUTES_GET_CLASS(matrix)->add_route_notifier(matrix, n);
}

void esp32_sig_set_wave(Object *matrix, unsigned sig, Esp32LineWave *w)
{
    ESP32_SIG_ROUTES_GET_CLASS(matrix)->set_wave(matrix, sig, w);
}

void esp32_sig_set_bulk_in(Object *matrix, unsigned sig, bool on)
{
    ESP32_SIG_ROUTES_GET_CLASS(matrix)->set_bulk_in(matrix, sig, on);
}

Esp32LineWave *esp32_sig_in_wave(Object *matrix, unsigned sig,
                                 bool *inverted)
{
    return ESP32_SIG_ROUTES_GET_CLASS(matrix)->sig_in_wave(matrix, sig,
                                                           inverted);
}

bool esp32_sig_out_taken(Object *matrix, unsigned sig)
{
    return ESP32_SIG_ROUTES_GET_CLASS(matrix)->sig_out_taken(matrix, sig);
}

static const TypeInfo esp32_sig_routes_info = {
    .name = TYPE_ESP32_SIG_ROUTES,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(Esp32SigRoutesClass),
};

static void esp32_line_register_types(void)
{
    type_register_static(&esp32_sig_routes_info);
}

type_init(esp32_line_register_types)
