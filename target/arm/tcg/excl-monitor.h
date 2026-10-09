/*
 * QEMU ARM CPU -- interface for an M-profile system's global exclusive
 * monitor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * An M-profile core has a local exclusive monitor of its own. Exclusive
 * accesses that the core marks as external (to Shareable memory, or to
 * any memory on a Cortex-M33 with ACTLR.EXTEXCLALL set) also go to the
 * system's global exclusive monitor, which decides which addresses
 * support exclusives at all, the reservation granule, and which writes
 * by other bus managers break a reservation. Those rules belong to the
 * SoC, so the SoC provides them through this QOM interface, connected to
 * the CPU with a link property in the same way as the IDAU.
 *
 * Every call is made after the core has passed the access through its
 * MPU and SAU, with @addr the physical address. @cs identifies the core,
 * @size is log2 of the access size in bytes, and @tag identifies the
 * Security and privilege state of the access, which must match between
 * an exclusive read and the exclusive write that completes it.
 */

#ifndef TARGET_ARM_EXCL_MONITOR_H
#define TARGET_ARM_EXCL_MONITOR_H

#include "qom/object.h"

#define TYPE_ARM_EXCL_MONITOR_INTERFACE "arm-excl-monitor-interface"
#define ARM_EXCL_MONITOR(obj) \
    INTERFACE_CHECK(ARMExclMonitor, (obj), TYPE_ARM_EXCL_MONITOR_INTERFACE)
typedef struct ARMExclMonitorClass ARMExclMonitorClass;
DECLARE_CLASS_CHECKERS(ARMExclMonitorClass, ARM_EXCL_MONITOR,
                       TYPE_ARM_EXCL_MONITOR_INTERFACE)

typedef struct ARMExclMonitor ARMExclMonitor;

struct ARMExclMonitorClass {
    InterfaceClass parent;

    /*
     * Return true if the monitor supports exclusive accesses at @addr.
     * Elsewhere every external exclusive reports failure to the core,
     * and the core reports exclusive reads and writes there through
     * clear().
     */
    bool (*supported)(ARMExclMonitor *m, uint32_t addr);

    /*
     * An exclusive read by @cs at a supported address, which takes a
     * reservation. If @host is not NULL it points at the bytes at @addr
     * in host memory and the monitor reads *val from it as it takes the
     * reservation; otherwise the access went through I/O, the core has
     * already made it, and *val holds the value it read.
     */
    void (*load)(ARMExclMonitor *m, CPUState *cs, uint32_t addr,
                 unsigned size, unsigned tag, const void *host,
                 uint32_t *val);

    /*
     * An exclusive write by @cs at a supported address. Return true if the
     * monitor lets it complete. If @host is not NULL the monitor makes the
     * write to it, atomically with deciding; otherwise the core makes the
     * write after a true return, with every other core stopped.
     */
    bool (*store)(ARMExclMonitor *m, CPUState *cs, uint32_t addr,
                  unsigned size, unsigned tag, void *host, uint32_t val);

    /*
     * An external exclusive read or write by @cs at an address the
     * monitor does not support.
     */
    void (*clear)(ARMExclMonitor *m, CPUState *cs);

    /*
     * A write by @cs at a supported address that the monitor sees as a
     * normal (non-exclusive) write: an exclusive write the core did not
     * mark as external.
     */
    void (*write)(ARMExclMonitor *m, CPUState *cs, uint32_t addr,
                  unsigned size);
};

#endif
