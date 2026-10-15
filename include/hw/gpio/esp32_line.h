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
 * Periodic outputs taken in bulk.
 *
 * A PWM output (LEDC's channels, MCPWM's generators) whose edges only a
 * block counting them takes (a PCNT signal input) need not be put on its
 * line edge by edge, an event of the host's each: the counter can take the
 * edges in bulk, worked out at its own accesses, and time only what it
 * does with them (a threshold's interrupt). Such an output describes its
 * edges as functions of time, an Esp32LineWave: its level at a moment, how
 * many edges it has between two moments and when its j-th edge after a
 * moment comes, each exactly as the output would drive it. A description
 * holds for a stretch: until the output's settings change, which the
 * output announces at the moment t they change, before and after it
 * changes (esp32_line_wave_changing(), esp32_line_wave_changed()); its
 * edges before t are the old description's, its level at t the new one's
 * (which can differ from the old at once), and its edges after t the new
 * one's. A block taking the output (esp32_line_wave_take()) first takes
 * the old description's edges up to t, then, the output changed, its
 * level at t and the new one's edges from there. A description may end
 * of itself, at an update the output has waiting (until), which it
 * announces as it comes.
 *
 * An output describes its edges only in stretches it can work out in
 * closed form (a steady PWM, not a fade): otherwise (describes false) it
 * drives every edge on its line, as an output something else watches
 * does, and a taker follows the line.
 *
 * The GPIO matrix connects an output's wave to the input signals that
 * take it in bulk (Esp32SigRoutesClass below): only through a plain path,
 * a pad the output alone drives, push-pull, with no filter on its way in,
 * that nothing else on the pad watches. A wave reaching only such inputs
 * makes the output's pad unwatched by them (esp32_sig_out_watched()),
 * and the output, describing its edges, drives its line only at its
 * level of the moment as software samples the pads (esp32_line_sync()).
 */
typedef struct Esp32LineWave Esp32LineWave;

typedef struct Esp32LineWaveOps {
    /* Whether the output describes its edges from now on, as below. */
    bool (*describes)(Esp32LineWave *w);
    /* Its level at time t. */
    bool (*level)(Esp32LineWave *w, int64_t t);
    /* How many edges it has at times in (a, b]. */
    uint64_t (*edges)(Esp32LineWave *w, int64_t a, int64_t b);
    /* The time of its j-th edge after a (j >= 1); INT64_MAX if none. */
    int64_t (*edge)(Esp32LineWave *w, int64_t a, uint64_t j);
    /*
     * A bound on its pulses: no level it takes lasts less than this many
     * ns (0 if it may), for a filter that passes or removes them all.
     */
    int64_t (*min_hold)(Esp32LineWave *w);
    /* When the description ends of itself; INT64_MAX if it does not. */
    int64_t (*until)(Esp32LineWave *w);
} Esp32LineWaveOps;

struct Esp32LineWave {
    const Esp32LineWaveOps *ops;
    /* The output's block and its index there, for the ops */
    void *opaque;
    unsigned index;
    /* The takers' notifiers, each given an Esp32LineWaveChange */
    NotifierList takers;
};

typedef struct Esp32LineWaveChange {
    Esp32LineWave *wave;
    int64_t when;
    /* false: the output is about to change; true: it has */
    bool done;
} Esp32LineWaveChange;

void esp32_line_wave_init(Esp32LineWave *w, const Esp32LineWaveOps *ops,
                          void *opaque, unsigned index);
bool esp32_line_wave_taken(Esp32LineWave *w);
void esp32_line_wave_take(Esp32LineWave *w, Notifier *n);
void esp32_line_wave_drop(Notifier *n);
void esp32_line_wave_changing(Esp32LineWave *w, int64_t when);
void esp32_line_wave_changed(Esp32LineWave *w, int64_t when);

/*
 * The routing of a GPIO matrix, for a block both chips have whose outputs
 * cost it work edge by edge: whether output signal sig reaches a pad
 * whose level anything but software sampling the pads sees (an input
 * signal a modelled block takes from it, its interrupt type, a line of it
 * the board listens to, ...), and a notifier the matrix calls whenever
 * that may have changed. The ESP32's and the ESP32-P4's matrices
 * implement it; a block holds a link to its machine's.
 *
 * For outputs taken in bulk (above): an output registers its wave for its
 * signal (set_wave), and a block taking waves in bulk on an input signal
 * says so (set_bulk_in); the input's wave is the one a plain path
 * connects it to (sig_in_wave, NULL if none), and sig_out_taken tells an
 * output whether such an input takes it. sig_out_watched leaves such
 * inputs out for an output with a wave.
 */
#define TYPE_ESP32_SIG_ROUTES "esp32-sig-routes"

typedef struct Esp32SigRoutesClass {
    InterfaceClass parent_class;

    bool (*sig_out_watched)(Object *matrix, unsigned sig);
    void (*add_route_notifier)(Object *matrix, Notifier *n);
    void (*set_wave)(Object *matrix, unsigned sig, Esp32LineWave *w);
    void (*set_bulk_in)(Object *matrix, unsigned sig, bool on);
    Esp32LineWave *(*sig_in_wave)(Object *matrix, unsigned sig,
                                  bool *inverted);
    bool (*sig_out_taken)(Object *matrix, unsigned sig);
} Esp32SigRoutesClass;

DECLARE_CLASS_CHECKERS(Esp32SigRoutesClass, ESP32_SIG_ROUTES,
                       TYPE_ESP32_SIG_ROUTES)

bool esp32_sig_out_watched(Object *matrix, unsigned sig);
void esp32_sig_add_route_notifier(Object *matrix, Notifier *n);
void esp32_sig_set_wave(Object *matrix, unsigned sig, Esp32LineWave *w);
void esp32_sig_set_bulk_in(Object *matrix, unsigned sig, bool on);
Esp32LineWave *esp32_sig_in_wave(Object *matrix, unsigned sig,
                                 bool *inverted);
bool esp32_sig_out_taken(Object *matrix, unsigned sig);

#endif
