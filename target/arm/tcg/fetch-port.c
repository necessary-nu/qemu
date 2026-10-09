/*
 * QEMU ARM CPU -- interface for an M-profile instruction fetch port decode
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "target/arm/tcg/fetch-port.h"

static const TypeInfo fetch_port_types[] = {
    {
        .name       = TYPE_ARM_FETCH_PORT_INTERFACE,
        .parent     = TYPE_INTERFACE,
        .class_size = sizeof(ARMFetchPortClass),
    }
};

DEFINE_TYPES(fetch_port_types)
