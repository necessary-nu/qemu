/*
 * ESP32-P4 waits counted in cycles of a block's clock
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_CLKWAIT_H
#define HW_MISC_ESP32P4_CLKWAIT_H

#include "hw/core/clock.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"

/*
 * A wait of a number of cycles of one clock, as a block's counter
 * counts them: until cycle @end of @count, the clock's cycles counted
 * exactly from an edge at its origin, the wait's start. Its timer runs to
 * the first nanosecond the count reaches @end, at least a nanosecond
 * after the wait began (@from_ns). While the clock is stopped the count
 * holds and the timer is not armed. A block whose clock is about to
 * change (ClockPreUpdate) folds the cycles counted so far into the wait;
 * once it has changed (ClockUpdate), it reschedules: the wait goes on at
 * the new rate from the moment of the change. A wait started at the end
 * of the one before (@ended_ns), as a periodic timer starts its next
 * period, goes on counting from that wait's exact end rather than the
 * nanosecond it was seen at, so that a train of waits keeps to the
 * clock's edges.
 */
typedef struct ESP32P4ClkWait {
    QEMUTimer timer;
    /* The clock counted; set at the block's init, not migrated. */
    Clock *clk;
    ESP32P4ClkCount count;
    uint64_t end;
    int64_t from_ns;
    int64_t ended_ns;
    bool active;
} ESP32P4ClkWait;

void esp32p4_clkwait_init(ESP32P4ClkWait *w, Clock *clk, QEMUTimerCB *cb,
                          void *opaque);

/*
 * Wait @cycles counted from @from_ns: the present, or the end of a wait
 * that has just ended, which a late timer must not stretch.
 */
void esp32p4_clkwait_start(ESP32P4ClkWait *w, int64_t from_ns,
                           uint64_t cycles);
void esp32p4_clkwait_stop(ESP32P4ClkWait *w);

/* Whether the wait runs, and when it ends at the clock's current rate. */
bool esp32p4_clkwait_active(ESP32P4ClkWait *w);
int64_t esp32p4_clkwait_end(ESP32P4ClkWait *w);

/*
 * Before the clock changes, at @when, count the cycles made; after it has
 * changed, rearm the timer at the clock's new rate.
 */
void esp32p4_clkwait_fold(ESP32P4ClkWait *w, int64_t when);
void esp32p4_clkwait_resched(ESP32P4ClkWait *w);

/*
 * The timer has fired: end the wait, and return its end, the virtual
 * time it was due, which a timer running late has passed.
 */
int64_t esp32p4_clkwait_expire(ESP32P4ClkWait *w);

extern const VMStateDescription vmstate_esp32p4_clkwait;

#define VMSTATE_ESP32P4_CLKWAIT(_f, _s) \
    VMSTATE_STRUCT(_f, _s, 1, vmstate_esp32p4_clkwait, ESP32P4ClkWait)
#define VMSTATE_ESP32P4_CLKWAIT_ARRAY(_f, _s, _n) \
    VMSTATE_STRUCT_ARRAY(_f, _s, _n, 1, vmstate_esp32p4_clkwait, \
                         ESP32P4ClkWait)

#endif
