/*
 * RP2350 redundancy coprocessor (RCP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_RCP_H
#define HW_MISC_RP2350_RCP_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

#define TYPE_RP2350_RCP "rp2350-rcp"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350RCPState, RP2350_RCP)

#define RP2350_RCP_CORES 2

typedef struct RP2350RCPCore {
    RP2350RCPState *rcp;
    int core;

    uint64_t salt;
    bool salt_valid;
    uint8_t count;
    /* Set by an RCP fault on either core; further instructions stall. */
    bool faulted;
    /* The delay/random-byte PRNG state, seeded from salt bits 63:40. */
    uint32_t prng;
} RP2350RCPCore;

struct RP2350RCPState {
    SysBusDevice parent_obj;

    RP2350RCPCore core[RP2350_RCP_CORES];
    /* Each core's NMI input. */
    qemu_irq nmi[RP2350_RCP_CORES];

    /*
     * Start with both salts valid and random, as the boot ROM leaves them
     * for user code (for directly loaded images, when no ROM runs).
     */
    bool boot_rom_handoff;
};

/* Make the RCP coprocessor 7 of `cpu`, which is RP2350 core `core`. */
void rp2350_rcp_attach(RP2350RCPState *s, int core, ARMCPU *cpu);

/*
 * Reset core `core`'s RCP, as that core's own reset does, leaving it as
 * the boot ROM hands it over when boot-rom-handoff is set.
 */
void rp2350_rcp_reset_core(RP2350RCPState *s, int core);

#endif
