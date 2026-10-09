/*
 * QEMU ARM CPU -- interface for an M-profile system's global exclusive
 * monitor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "target/arm/tcg/excl-monitor.h"

static const TypeInfo excl_monitor_types[] = {
    {
        .name       = TYPE_ARM_EXCL_MONITOR_INTERFACE,
        .parent     = TYPE_INTERFACE,
        .class_size = sizeof(ARMExclMonitorClass),
    }
};

DEFINE_TYPES(excl_monitor_types)
