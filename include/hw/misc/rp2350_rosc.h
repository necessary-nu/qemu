/*
 * RP2350 ring oscillator (ROSC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_ROSC_H
#define HW_MISC_RP2350_ROSC_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/rp2350_clocks.h"
#include "qom/object.h"

#define TYPE_RP2350_ROSC "rp2350-rosc"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350ROSCState, RP2350_ROSC)

struct RP2350ROSCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *startup_timer;
    /* The divider's outputs, rosc_clksrc and rosc_clksrc_ph. */
    Clock *out;
    Clock *out_ph;

    /* Registers as last written (raw, including invalid codes). */
    uint32_t ctrl;
    uint32_t freqa;
    uint32_t freqb;
    uint32_t random;
    uint32_t dormant;
    uint32_t div;
    uint32_t phase;
    bool badwrite;

    /*
     * The drive-strength and randomise controls in force: a FREQA/FREQB
     * write with a bad password sets them all to 0 rather than applying
     * the written fields.
     */
    uint32_t freqa_applied;
    uint32_t freqb_applied;

    /* Virtual time from which an enabled oscillator reads as STABLE. */
    int64_t stable_ns;

    /* Stopped by DORMANT, waiting for a wake event. */
    bool dormant_stopped;
    /* Output gated: from DORMANT entry until stable after the wake. */
    bool gated;
    /* The level of the dormant-wake input. */
    bool wake;

    /* COUNT as of virtual time count_ns; it counts down from there. */
    uint32_t count;
    int64_t count_ns;

    /* Host entropy not yet handed out through RANDOMBIT, and the last bit. */
    uint32_t entropy;
    uint32_t entropy_bits;
    uint32_t randombit;
};

/* Named clock outputs of ROSC: rosc_clksrc and rosc_clksrc_ph. */
#define RP2350_ROSC_CLK "clk"
#define RP2350_ROSC_CLK_PH "clk-ph"

/*
 * Configure the ROSC as the boot ROM's early boot path leaves it: DIV
 * lowered from 8 to 2, quadrupling the boot clk_sys.
 */
void rp2350_rosc_boot_rom_handoff(RP2350ROSCState *s);

#endif
