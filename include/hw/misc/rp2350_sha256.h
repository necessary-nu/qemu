/*
 * RP2350 SHA-256 accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_SHA256_H
#define HW_MISC_RP2350_SHA256_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_SHA256 "rp2350-sha256"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350SHA256State, RP2350_SHA256)

/*
 * The block's DMA request, a named qdev GPIO output with one line. It is
 * high while the block will accept another DMA transfer into WDATA: from
 * START (or the end of a block's digest) until it has received one
 * block's worth of transfers of the size CSR.DMA_SIZE selects.
 */
#define RP2350_SHA256_DREQ "dreq"

/*
 * The block's register window: plain, its atomic aliases, then the same
 * again with narrow-write replication off (address bit 14).
 */
#define RP2350_SHA256_REGION_SIZE 0x8000

#define RP2350_SHA256_BLOCK_WORDS 16
#define RP2350_SHA256_SUM_WORDS 8

struct RP2350SHA256State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *timer;
    Clock *clk;
    qemu_irq dreq;

    bool bswap;
    uint32_t dma_size;
    bool err_not_rdy;

    /* SUM_VLD; WDATA_RDY is the inverse of `busy`. */
    bool sum_vld;
    /* A full block is being digested; the timer ends it. */
    bool busy;

    uint32_t sum[RP2350_SHA256_SUM_WORDS];
    /* Message words of the current block, as committed to the core. */
    uint32_t block[RP2350_SHA256_BLOCK_WORDS];
    uint32_t block_words;

    /* The bus interface's 32-bit shift register and its fill in bytes. */
    uint32_t shift;
    uint32_t shift_bytes;

    /* DMA transfers the block will still request for the current block. */
    uint32_t dreq_credits;
};

#endif
