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
    w->ended_ns = INT64_MIN;
    timer_init_ns(&w->timer, QEMU_CLOCK_VIRTUAL, cb, opaque);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
int64_t esp32p4_clkwait_end(ESP32P4ClkWait *w)
{
    if (!w->active || !esp32p4_clk_count_running(&w->count)) {
        return INT64_MAX;
    }
    return MAX(esp32p4_clk_count_time(&w->count, w->end), w->from_ns + 1);
}

void esp32p4_clkwait_resched(ESP32P4ClkWait *w)
{
    int64_t end;

    if (w->active) {
        w->count.period = esp32p4_clk_exact(w->clk);
    }
    end = esp32p4_clkwait_end(w);
    if (end == INT64_MAX) {
        timer_del(&w->timer);
    } else {
        timer_mod(&w->timer, end);
    }
}

/* [spec:nuos:req:emu.esp32p4.analog] */
void esp32p4_clkwait_start(ESP32P4ClkWait *w, int64_t from_ns,
                           uint64_t cycles)
{
    ESP32P4ClkPeriod p = esp32p4_clk_exact(w->clk);

    if (!w->active && from_ns == w->ended_ns &&
        esp32p4_clk_period_eq(p, w->count.period) &&
        cycles <= UINT64_MAX - w->end) {
        w->end += cycles;
    } else {
        w->count.period = p;
        w->count.origin_ns = from_ns;
        w->end = cycles;
    }
    w->from_ns = from_ns;
    w->active = true;
    esp32p4_clkwait_resched(w);
}

void esp32p4_clkwait_stop(ESP32P4ClkWait *w)
{
    w->active = false;
    w->ended_ns = INT64_MIN;
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
    done = MIN(esp32p4_clk_count_at(&w->count, when), w->end);
    w->end -= done;
    w->count.origin_ns = MAX(when, w->count.origin_ns);
    w->from_ns = MAX(when, w->from_ns);
}

int64_t esp32p4_clkwait_expire(ESP32P4ClkWait *w)
{
    int64_t end = esp32p4_clkwait_end(w);

    w->active = false;
    if (end == INT64_MAX) {
        w->ended_ns = INT64_MIN;
        return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    w->ended_ns = end;
    return end;
}

const VMStateDescription vmstate_esp32p4_clkwait = {
    .name = "esp32p4-clkwait",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4ClkWait),
        VMSTATE_ESP32P4_CLK_COUNT(count, ESP32P4ClkWait),
        VMSTATE_UINT64(end, ESP32P4ClkWait),
        VMSTATE_INT64(from_ns, ESP32P4ClkWait),
        VMSTATE_INT64(ended_ns, ESP32P4ClkWait),
        VMSTATE_BOOL(active, ESP32P4ClkWait),
        VMSTATE_END_OF_LIST()
    }
};
