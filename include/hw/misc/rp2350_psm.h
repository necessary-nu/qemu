/*
 * RP2350 power-on state machine (PSM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_PSM_H
#define HW_MISC_RP2350_PSM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_PSM "rp2350-psm"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350PSMState, RP2350_PSM)

/*
 * The PSM's stages, one bit each in FRCE_ON, FRCE_OFF, WDSEL and DONE, in
 * the order the sequence releases them. PROC0 and PROC1 are released
 * together at the end.
 */
enum {
    RP2350_PSM_PROC_COLD,
    RP2350_PSM_OTP,
    RP2350_PSM_ROSC,
    RP2350_PSM_XOSC,
    RP2350_PSM_RESETS,
    RP2350_PSM_CLOCKS,
    RP2350_PSM_PSM_READY,
    RP2350_PSM_BUSFABRIC,
    RP2350_PSM_ROM,
    RP2350_PSM_BOOTRAM,
    RP2350_PSM_SRAM0,
    RP2350_PSM_SRAM9 = RP2350_PSM_SRAM0 + 9,
    RP2350_PSM_XIP,
    RP2350_PSM_SIO,
    RP2350_PSM_ACCESSCTRL,
    RP2350_PSM_PROC0,
    RP2350_PSM_PROC1,
    RP2350_PSM_STAGES,
};

#define RP2350_PSM_ALL ((1u << RP2350_PSM_STAGES) - 1)

/*
 * Carry out a (partial) reset sequence. Every stage in `reset` is reset;
 * those also in `held` stay in reset (FRCE_OFF), the others are released
 * and start again. `watchdog` is set when the watchdog requested the
 * sequence, which also resets the subsystems RESETS.WDSEL selects. Called
 * from the main loop with every vCPU paused.
 */
typedef void RP2350PSMResetFn(void *opaque, uint32_t reset, uint32_t held,
                              bool watchdog);

/*
 * Named GPIO inputs: "watchdog", the watchdog's reset request, which runs
 * the sequence WDSEL selects; and "powman-reset", the power manager's
 * chip-level reset of the PSM, which restores its registers and runs the
 * full sequence.
 */
struct RP2350PSMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUBH *bh;

    uint32_t frce_on;
    uint32_t frce_off;
    uint32_t wdsel;

    /* Stages to reset when the bottom half runs the sequence. */
    uint32_t pending;
    bool pending_watchdog;

    RP2350PSMResetFn *reset_fn;
    void *reset_opaque;
};

void rp2350_psm_set_reset_fn(RP2350PSMState *s, RP2350PSMResetFn *fn,
                             void *opaque);

#endif
