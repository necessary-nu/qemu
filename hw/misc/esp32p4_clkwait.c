/*
 * ESP32-P4 waits counted in cycles of a block's clock
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/esp32p4_clkwait.h"

void esp32p4_clkwait_init(ESP32P4ClkWait *w, Clock *clk, QEMUTimerCB *cb,
                          void *opaque)
{
    w->clk = clk;
    w->active = false;
    timer_init_ns(&w->timer, QEMU_CLOCK_VIRTUAL, cb, opaque);
}

int64_t esp32p4_clkwait_end(ESP32P4ClkWait *w)
{
    if (!w->active || !clock_is_enabled(w->clk)) {
        return INT64_MAX;
    }
    return w->base_ns + MAX(clock_ticks_to_ns(w->clk, w->left), 1);
}

void esp32p4_clkwait_resched(ESP32P4ClkWait *w)
{
    int64_t end = esp32p4_clkwait_end(w);

    if (end == INT64_MAX) {
        timer_del(&w->timer);
    } else {
        timer_mod(&w->timer, end);
    }
}

void esp32p4_clkwait_start(ESP32P4ClkWait *w, int64_t from_ns,
                           uint64_t cycles)
{
    w->base_ns = from_ns;
    w->left = cycles;
    w->active = true;
    esp32p4_clkwait_resched(w);
}

void esp32p4_clkwait_stop(ESP32P4ClkWait *w)
{
    w->active = false;
    timer_del(&w->timer);
}

bool esp32p4_clkwait_active(ESP32P4ClkWait *w)
{
    return w->active;
}

void esp32p4_clkwait_fold(ESP32P4ClkWait *w, int64_t when)
{
    uint64_t done;

    if (!w->active) {
        return;
    }
    if (clock_is_enabled(w->clk) && when > w->base_ns) {
        done = clock_ns_to_ticks(w->clk, when - w->base_ns);
        w->left -= MIN(done, w->left);
    }
    w->base_ns = MAX(when, w->base_ns);
}

int64_t esp32p4_clkwait_expire(ESP32P4ClkWait *w)
{
    int64_t end = esp32p4_clkwait_end(w);

    w->active = false;
    if (end == INT64_MAX) {
        end = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    return end;
}

const VMStateDescription vmstate_esp32p4_clkwait = {
    .name = "esp32p4-clkwait",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4ClkWait),
        VMSTATE_INT64(base_ns, ESP32P4ClkWait),
        VMSTATE_UINT64(left, ESP32P4ClkWait),
        VMSTATE_BOOL(active, ESP32P4ClkWait),
        VMSTATE_END_OF_LIST()
    }
};
