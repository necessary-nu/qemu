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
#include "qom/object.h"

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

/*
 * Streams on the host's clock.
 *
 * A stream (a unit's clock edges, a sensor's or a panel's lines, an audio
 * port's frames) takes each of its events at the event's own time, however
 * late the QEMU timer that runs it: under icount virtual time waits for
 * the stream, which never falls behind. On the host's clock QEMU's main
 * loop runs late, by however long the host holds it up, and a stream
 * catches up: it takes the events that fell due at their due times, in
 * order, with what feeds or takes its data (DMA, a peer on the pads)
 * brought up to each event's time first, so that a frame or transfer in
 * progress completes as it would have.
 *
 * Only a stream more than ESP32_LINE_MAX_LAG_NS behind the present, as it
 * comes to take an event (esp32_line_lags()), slips: it goes on from the
 * present, as the stream says (logged). The bound is far beyond the host's
 * pauses: a scheduler holds a thread up for a quantum of about 10 ms, a
 * host loaded several times over its cores for tens of milliseconds at a
 * time. A stream a second behind is one whose events cost the host more
 * than their own time, which it can never catch up with; slipping then
 * keeps the catch-up from holding QEMU's main loop, and the cores waiting
 * for it, ever longer.
 *
 * A PWM output (LEDC's channels, MCPWM's generators), whose edges follow
 * a counter software reads, keeps its timing: it goes on from the present
 * by skipping to its level at the moment it is being brought to, stamped
 * with the moment that level began, the edges in between lost.
 *
 * It does so as soon as the host is losing ground on it, not only a
 * second behind (esp32_line_pwm_skips()). Its own QEMU timer's catch-ups,
 * each doing the work due by the present as the timer runs, bring it up
 * to that moment (esp32_line_catching_up(), esp32_line_caught_up(), with
 * an Esp32LineLag per output): a host keeping up ends each within a
 * scheduler's quantum of the present, ESP32_LINE_SLACK_NS, or nearer than
 * it ended the last. One losing ground on the output ends a catch-up
 * beyond the slack and further behind than the last, and the next falls
 * further behind still, taking longer than the stretch it covers; the
 * output skips an edge beyond the slack as soon as the catch-up taking it
 * shows that, or after ESP32_LINE_LOSING_RUN catch-ups running ended so,
 * as the main loop's work as a whole outruns the host (two outputs, each
 * cheaper than its own edges, together dearer). Edges that cost the host
 * more than their own time would otherwise hold the main loop, and the
 * cores waiting on it, for up to a second at a time before the bound. An
 * output only briefly behind, after a pause of the host, ends its first
 * catch-up further behind than the one before the pause, but covers the
 * stretch the pause left faster than it goes by, and ends each catch-up
 * after nearer the present: it keeps the bound alone, and every edge.
 *
 * The streams keep the bound alone. A stream that slips moves its data
 * later (a frame restarts, a transfer resumes, a port's timing restarts)
 * where a skip only drops edges of a waveform that keeps its phase, so
 * that a slip on a false alarm costs more; and several streams take one
 * event a timer callback, in time order with every other timer, so that
 * how far behind each callback ends counts the main loop's other work
 * too, which, after a pause of the host, falls behind with the stream
 * across several callbacks before all of it catches up.
 */
#define ESP32_LINE_MAX_LAG_NS   NANOSECONDS_PER_SECOND
#define ESP32_LINE_SLACK_NS     (10 * SCALE_MS)
#define ESP32_LINE_LOSING_RUN   2

bool esp32_line_lags(int64_t due_ns);

/*
 * An output's catch-ups by its own timer: the moment the last brought it
 * to, how far behind the present it then was, how many running each
 * ended beyond the slack and further behind than the one before, and,
 * while one is under way, when it began. Host timing, not the chip's:
 * left out of migration, and as it is across a reset.
 */
typedef struct Esp32LineLag {
    int64_t reached_ns;
    int64_t behind_ns;
    uint32_t losing;
    bool catching_up;
    int64_t began_ns;
} Esp32LineLag;

void esp32_line_catching_up(Esp32LineLag *lag);
void esp32_line_caught_up(Esp32LineLag *lag, int64_t reached_ns);
bool esp32_line_pwm_skips(const Esp32LineLag *lag, int64_t due_ns);

/*
 * The routing of a GPIO matrix, for a block both chips have whose outputs
 * cost it work edge by edge: whether output signal sig reaches a pad
 * whose level anything but software sampling the pads sees (an input
 * signal a modelled block takes from it, its interrupt type, a line of it
 * the board listens to, ...), and a notifier the matrix calls whenever
 * that may have changed. The ESP32's and the ESP32-P4's matrices
 * implement it; a block holds a link to its machine's.
 */
#define TYPE_ESP32_SIG_ROUTES "esp32-sig-routes"

typedef struct Esp32SigRoutesClass {
    InterfaceClass parent_class;

    bool (*sig_out_watched)(Object *matrix, unsigned sig);
    void (*add_route_notifier)(Object *matrix, Notifier *n);
} Esp32SigRoutesClass;

DECLARE_CLASS_CHECKERS(Esp32SigRoutesClass, ESP32_SIG_ROUTES,
                       TYPE_ESP32_SIG_ROUTES)

bool esp32_sig_out_watched(Object *matrix, unsigned sig);
void esp32_sig_add_route_notifier(Object *matrix, Notifier *n);

#endif
