/*
 * RP2350 CoreSight debug and trace components (self-hosted debug window)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_CORESIGHT_H
#define HW_MISC_RP2350_CORESIGHT_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_RP2350_CORESIGHT "rp2350-coresight"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350CoreSightState, RP2350_CORESIGHT)

#define RP2350_CORESIGHT_SIZE 0x10000
#define RP2350_CORESIGHT_CORES 2
#define RP2350_CTI_TRIGGERS 5
#define RP2350_CTI_CHANNELS 4

/*
 * Bus masters that reach the window, each through its own view: the
 * system bus (DMA and anything else on the AHB fabric) and the two cores.
 * A core may not use its own AHB-AP.
 */
enum {
    RP2350_CORESIGHT_SYSTEM,
    RP2350_CORESIGHT_CORE0,
    RP2350_CORESIGHT_CORE1,
    RP2350_CORESIGHT_VIEWS,
};

typedef struct RP2350CoreSightView {
    RP2350CoreSightState *cs;
    int requester;
} RP2350CoreSightView;

/*
 * The registers a SoC-600 MEM-AP duplicates between its two logical APs.
 * The read-only registers are shared.
 */
typedef struct RP2350MemAPRegs {
    uint32_t csw;
    uint32_t tar;
    uint32_t trr;
    uint32_t itctrl;
    uint32_t claim;
} RP2350MemAPRegs;

struct RP2350CoreSightState {
    SysBusDevice parent_obj;

    MemoryRegion view[RP2350_CORESIGHT_VIEWS];
    RP2350CoreSightView view_opaque[RP2350_CORESIGHT_VIEWS];

    /* What each core's AHB-AP masters: that core's own bus view. */
    MemoryRegion *core_memory[RP2350_CORESIGHT_CORES];
    AddressSpace ap_as[RP2350_CORESIGHT_CORES];

    /* The timestamp generator's counter clock. */
    Clock *clk;

    /* The OTP CRIT1 debug-disable flags, as hardware latches them. */
    bool debug_disable;
    bool secure_debug_disable;

    RP2350MemAPRegs ahbap[RP2350_CORESIGHT_CORES][2];
    RP2350MemAPRegs apbap[2];

    uint32_t tsgen_cntcr;
    uint32_t tsgen_cntcvl;
    uint32_t tsgen_cntfid0;
    uint32_t tsgen_itctrl;
    /* The count: tsgen_count at tsgen_epoch_ns, counting on while EN. */
    uint64_t tsgen_count;
    int64_t tsgen_epoch_ns;

    uint32_t funnel_ctrl;
    uint32_t funnel_prio;
    uint32_t funnel_itctrl;
    uint32_t funnel_claim;

    uint32_t tpiu_cspsr;
    uint32_t tpiu_tcvr;
    uint32_t tpiu_tcmr;
    uint32_t tpiu_ctpmr;
    uint32_t tpiu_tprcr;
    uint32_t tpiu_ffcr;
    uint32_t tpiu_fscr;
    uint32_t tpiu_extctlout;
    uint32_t tpiu_itctrl;
    uint32_t tpiu_claim;

    uint32_t cti_ctrl;
    uint32_t cti_app;
    uint32_t cti_inen[RP2350_CTI_TRIGGERS];
    uint32_t cti_outen[RP2350_CTI_TRIGGERS];
    uint32_t cti_gate;
    uint32_t cti_itctrl;
    uint32_t cti_claim;
    /* Trigger outputs last reported high, so each rising edge logs once. */
    uint32_t cti_trigout;
};

/* The view of the window that bus master `requester` uses. */
MemoryRegion *rp2350_coresight_view(RP2350CoreSightState *s, int requester);

#endif
