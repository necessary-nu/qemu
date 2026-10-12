/*
 * Storage cells left part way by a program, write or erase cut short
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_BLOCK_POWER_CUT_H
#define HW_BLOCK_POWER_CUT_H

#include "qemu/host-utils.h"

/*
 * A cycle that changes storage cells (bits) and is cut short leaves each
 * of them done or not: a cell is done once the fraction f of the cycle's
 * time that has passed exceeds h / 2^32, where h is murmur3's 32-bit
 * finalizer of the cell's bit address, byte address * 8 + bit (an
 * address of more than 32 bits is folded first, its high half XORed into
 * its low). The rule is monotonic in f, so suspending, resuming and
 * cutting a cycle in any order leaves what one cut at its total elapsed
 * time does. A cycle that runs its whole time applies in full.
 */

static inline uint32_t power_cut_cell_hash(uint64_t bit)
{
    uint32_t h = (uint32_t)bit ^ (uint32_t)(bit >> 32);

    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}

/* The bound below which a cell's hash is done, @done ns into @total. */
static inline uint32_t power_cut_threshold(int64_t done, int64_t total)
{
    uint64_t lo, hi;

    if (done <= 0) {
        return 0;
    }
    if (done >= total) {
        return UINT32_MAX;
    }
    lo = (uint64_t)done << 32;
    hi = (uint64_t)done >> 32;
    divu128(&lo, &hi, total);
    return lo;
}

/* The cells of the byte at bit address @bit0 done below @threshold. */
static inline uint8_t power_cut_done_mask(uint64_t bit0, uint32_t threshold)
{
    uint8_t mask = 0;

    for (int b = 0; b < 8; b++) {
        if (power_cut_cell_hash(bit0 + b) < threshold) {
            mask |= 1 << b;
        }
    }
    return mask;
}

#endif
