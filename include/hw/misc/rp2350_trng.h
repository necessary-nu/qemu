/*
 * RP2350 true random number generator (TRNG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_TRNG_H
#define HW_MISC_RP2350_TRNG_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_TRNG "rp2350-trng"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350TRNGState, RP2350_TRNG)

#define RP2350_TRNG_EHR_WORDS 6

struct RP2350TRNGState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;

    /* rng_clk (clk_sys), which times SAMPLE_CNT1. */
    Clock *clk;

    uint32_t imr;
    uint32_t isr;
    uint32_t config;
    uint32_t rnd_source_enable;
    uint32_t sample_cnt1;
    uint32_t debug_control;
    uint32_t debug_en;

    /* The Entropy Holding Register, and whether it holds a result. */
    bool ehr_valid;
    uint32_t ehr[RP2350_TRNG_EHR_WORDS];

    /*
     * A collection in progress (paused while RND_SRC_EN is clear). Each
     * collection is drawn in full when it starts: it ends after
     * samples_left more ROSC samples, counted from virtual time sync_ns,
     * in the event `pending_event` -- a completed EHR (pending_ehr) or an
     * entropy-check failure, after which collection starts afresh.
     */
    bool collecting;
    uint32_t pending_event;
    uint32_t pending_ehr[RP2350_TRNG_EHR_WORDS];
    uint64_t samples_left;
    int64_t sync_ns;

    /* Host entropy not yet used as raw ROSC samples. */
    uint32_t entropy;
    uint32_t entropy_bits;
};

#endif
