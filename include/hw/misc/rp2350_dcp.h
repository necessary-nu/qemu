/*
 * RP2350 double-precision coprocessor (DCP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_DCP_H
#define HW_MISC_RP2350_DCP_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

#define TYPE_RP2350_DCP "rp2350-dcp"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350DCPState, RP2350_DCP)

#define RP2350_DCP_CORES 2

/* One DCP instance: each core has a Secure and a Non-secure one. */
typedef struct RP2350DCPInstance {
    uint64_t xm;
    uint64_t ym;
    uint32_t xe;
    uint32_t ye;
    uint32_t xf;
    uint32_t yf;
    /* Bits 5:0 alignment shift, 7:6 comparison, 8 engaged. */
    uint32_t status;
} RP2350DCPInstance;

typedef struct RP2350DCPCore {
    RP2350DCPState *dcp;
    int core;
    RP2350DCPInstance inst[2];
} RP2350DCPCore;

struct RP2350DCPState {
    SysBusDevice parent_obj;

    RP2350DCPCore core[RP2350_DCP_CORES];
};

/* Make the DCP coprocessors 4 and 5 of `cpu`, which is RP2350 core `core`. */
void rp2350_dcp_attach(RP2350DCPState *s, int core, ARMCPU *cpu);

#endif
