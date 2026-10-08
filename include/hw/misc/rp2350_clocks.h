/*
 * RP2350 clock generation: CLOCKS, XOSC, PLL_SYS/PLL_USB and TICKS
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_CLOCKS_H
#define HW_MISC_RP2350_CLOCKS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

/*
 * The four blocks share a register-file base: configuration registers are
 * stored as written (through the atomic aliases), and each block derives
 * the status bits guest code waits on from that configuration. Clock
 * frequencies themselves are not modelled.
 */
#define TYPE_RP2350_CLKREGS "rp2350-clkregs"
OBJECT_DECLARE_TYPE(RP2350ClkRegsState, RP2350ClkRegsClass, RP2350_CLKREGS)

#define TYPE_RP2350_CLOCKS "rp2350-clocks"
#define TYPE_RP2350_XOSC "rp2350-xosc"
#define TYPE_RP2350_PLL "rp2350-pll"
#define TYPE_RP2350_TICKS "rp2350-ticks"

#define RP2350_CLKREGS_MAX 64

struct RP2350ClkRegsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[RP2350_CLKREGS_MAX];
};

struct RP2350ClkRegsClass {
    SysBusDeviceClass parent_class;

    unsigned nregs;
    /* Reset values, indexed by register; unlisted registers reset to 0. */
    const uint32_t *reset;
    /* Writable bits per register; NULL writes all bits of every register. */
    const uint32_t *wmask;
    /* The value a read returns, given the stored register. */
    uint32_t (*read)(RP2350ClkRegsState *s, unsigned reg);
};

/* The TICKS generators, in register order. */
enum {
    RP2350_TICK_PROC0,
    RP2350_TICK_PROC1,
    RP2350_TICK_TIMER0,
    RP2350_TICK_TIMER1,
    RP2350_TICK_WATCHDOG,
    RP2350_TICK_RISCV,
    RP2350_NUM_TICKS,
};

/* Whether TICKS generator `tick` is enabled, and so producing ticks. */
bool rp2350_ticks_running(RP2350ClkRegsState *ticks, int tick);

#endif
