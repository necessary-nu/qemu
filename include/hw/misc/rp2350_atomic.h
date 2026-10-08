/*
 * RP2350 atomic register aliases
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_ATOMIC_H
#define HW_MISC_RP2350_ATOMIC_H

#include "exec/hwaddr.h"

/*
 * Every APB peripheral register is also reachable through three 4 KiB
 * aliases above its block: +0x1000 XORs, +0x2000 sets and +0x3000 clears
 * the written bits. Reads through an alias read the register itself.
 */
#define RP2350_ATOMIC_ALIAS_SIZE 0x1000
#define RP2350_ATOMIC_REGION_SIZE (4 * RP2350_ATOMIC_ALIAS_SIZE)

static inline hwaddr rp2350_atomic_reg(hwaddr addr)
{
    return addr & (RP2350_ATOMIC_ALIAS_SIZE - 1);
}

static inline uint32_t rp2350_atomic_apply(hwaddr addr, uint32_t old,
                                           uint32_t value)
{
    switch (addr / RP2350_ATOMIC_ALIAS_SIZE) {
    case 1:
        return old ^ value;
    case 2:
        return old | value;
    case 3:
        return old & ~value;
    default:
        return value;
    }
}

#endif
