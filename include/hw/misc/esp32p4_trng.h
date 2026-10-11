/*
 * ESP32-P4 random number generator (LP_TRNG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_TRNG_H
#define HW_MISC_ESP32P4_TRNG_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_TRNG "esp32p4-trng"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4TrngState, ESP32P4_TRNG)

#define ESP32P4_TRNG_BASE   0x50126000
#define ESP32P4_TRNG_SIZE   0x1000
/* RNG_DATA_REG, which a read moves on. */
#define ESP32P4_TRNG_DATA   0x04

/*
 * The clock input "clk" is LPPERI's RNG clock (LPPERI_CK_EN_RNG), which
 * its registers are behind and which paces its sampling; LPPERI_RST_EN_
 * LP_RNG resets it.
 */
struct ESP32P4TrngState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;

    uint32_t cfg;
    uint32_t data;
    uint32_t rstn;
    uint32_t date;
    /* SAMPLE_CNT's count, and since when it has counted at the clock. */
    uint32_t samples;
    int64_t sample_ns;
};

/*
 * The next random number, as RNG_DATA_REG gives it: the generator's
 * output, which only changes while it runs (RNG_CLK_EN set, RNG_RSTN
 * set, its clock running).
 */
uint32_t esp32p4_trng_read(ESP32P4TrngState *s);

#endif
