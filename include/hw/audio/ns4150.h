/*
 * Nsiway NS4150 (NS4150B) filterless class D mono audio amplifier
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_NS4150_H
#define HW_AUDIO_NS4150_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_NS4150 "ns4150"
OBJECT_DECLARE_SIMPLE_TYPE(NS4150State, NS4150)

/*
 * The amplifier's CTRL pin, input "ctrl": high, it leaves power-down and
 * drives the speaker once its start-up time (tST, 30 ms typical,
 * property "startup-ns") is over; low, it shuts down at once. Output
 * "on": high while it drives the speaker, for the board's audio path.
 * Its gain, which the board's input resistors set with its undocumented
 * internal feedback, is not modelled: the speaker carries its input as a
 * fraction of full scale.
 */
#define NS4150_CTRL     "ctrl"
#define NS4150_ON       "on"

struct NS4150State {
    SysBusDevice parent_obj;

    qemu_irq on_out;
    QEMUTimer startup;
    uint64_t startup_ns;
    bool ctrl;
    bool on;
};

#endif
