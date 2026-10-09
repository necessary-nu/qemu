/*
 * RP2350 Cortex-M33 processor debug and trace components
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_M33_DEBUG_H
#define HW_MISC_RP2350_M33_DEBUG_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_RP2350_M33_DEBUG "rp2350-m33-debug"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350M33DebugState, RP2350_M33_DEBUG)

/*
 * The device's MMIO regions, in sysbus_mmio order, and the PPB address
 * each belongs at in its core's view of the bus.
 */
typedef enum RP2350M33DebugRegion {
    RP2350_M33_DEBUG_ITM,       /* 0xe0000000 */
    RP2350_M33_DEBUG_DWT,       /* 0xe0001000 */
    RP2350_M33_DEBUG_FPB,       /* 0xe0002000 */
    RP2350_M33_DEBUG_SCS_ID,    /* 0xe000efbc, over the SCS */
    RP2350_M33_DEBUG_SCS_ID_NS, /* 0xe002efbc, over the SCS NS alias */
    RP2350_M33_DEBUG_ETM,       /* 0xe0041000 */
    RP2350_M33_DEBUG_CTI,       /* 0xe0042000 */
    RP2350_M33_DEBUG_ROM,       /* 0xe00ff000 */
    RP2350_M33_DEBUG_NUM_REGIONS
} RP2350M33DebugRegion;

extern const hwaddr rp2350_m33_debug_base[RP2350_M33_DEBUG_NUM_REGIONS];

/* Register state slots per component; enough for the largest (the ETM). */
#define RP2350_M33_DEBUG_MAX_REGS 48

typedef struct RP2350M33DebugComponent {
    RP2350M33DebugState *s;
    const struct RP2350M33DebugInfo *info;
    MemoryRegion mr;
    uint32_t regs[RP2350_M33_DEBUG_MAX_REGS];
} RP2350M33DebugComponent;

/*
 * One core's debug components inside the processor: the ITM, DWT, FPB,
 * ETM and CTI, the identification block of the SCS, and the PPB ROM
 * table that lists them. Clock input "cpuclk" is the core's clock, which
 * the DWT cycle counter counts.
 */
struct RP2350M33DebugState {
    SysBusDevice parent_obj;

    RP2350M33DebugComponent comp[RP2350_M33_DEBUG_NUM_REGIONS];
    Clock *cpuclk;

    /* DWT_CYCCNT as it stood at dwt_cyccnt_ns, when it last changed rate. */
    uint32_t dwt_cyccnt;
    int64_t dwt_cyccnt_ns;
    /* CTI trigger outputs already reported as having no modelled effect. */
    uint32_t cti_trigout_logged;
};

#endif
