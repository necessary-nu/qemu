/*
 * QEMU ARM CPU -- interface for an M-profile instruction fetch port decode
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * An M-profile core can have separate bus ports for instruction fetch and
 * for loads and stores, and the SoC bus fabric need not route both to
 * every target: a fetch from a target with no instruction path fails on
 * the bus, after it has passed the MPU, and the core takes a BusFault
 * with IBUSERR. QEMU gives the CPU a single address space for both kinds
 * of access, so the SoC describes which addresses its instruction port
 * reaches through this QOM interface, connected to the CPU with a link
 * property in the same way as the IDAU.
 */

#ifndef TARGET_ARM_FETCH_PORT_H
#define TARGET_ARM_FETCH_PORT_H

#include "qom/object.h"

#define TYPE_ARM_FETCH_PORT_INTERFACE "arm-fetch-port-interface"
#define ARM_FETCH_PORT(obj) \
    INTERFACE_CHECK(ARMFetchPort, (obj), TYPE_ARM_FETCH_PORT_INTERFACE)
typedef struct ARMFetchPortClass ARMFetchPortClass;
DECLARE_CLASS_CHECKERS(ARMFetchPortClass, ARM_FETCH_PORT,
                       TYPE_ARM_FETCH_PORT_INTERFACE)

typedef struct ARMFetchPort ARMFetchPort;

struct ARMFetchPortClass {
    InterfaceClass parent;

    /*
     * Return true if an instruction fetch from the specified address
     * reaches a target, false if the bus fabric rejects it. The caller
     * sets *base to 0 and *limit to 0xffffffff; an implementation narrows
     * them to the inclusive range around the address over which its
     * answer holds.
     */
    bool (*fetchable)(ARMFetchPort *fp, uint32_t address,
                      uint32_t *base, uint32_t *limit);
};

#endif
