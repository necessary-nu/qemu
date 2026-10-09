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
#include "qemu/timer.h"
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
OBJECT_DECLARE_SIMPLE_TYPE(RP2350XOSCState, RP2350_XOSC)
#define TYPE_RP2350_PLL "rp2350-pll"
#define TYPE_RP2350_TICKS "rp2350-ticks"

#define RP2350_CLKREGS_MAX 64

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

typedef void RP2350TickNotify(void *opaque);

struct RP2350ClkRegsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[RP2350_CLKREGS_MAX];

    /* TICKS only: see rp2350_ticks_set_notify(). */
    RP2350TickNotify *tick_notify[RP2350_NUM_TICKS];
    void *tick_opaque[RP2350_NUM_TICKS];
    /* TICKS only: see rp2350_ticks_set_clocks(). */
    bool ref_stopped;
    bool sys_stopped;

    /* CLOCKS only: see rp2350_clocks_set_notify(). */
    RP2350TickNotify *clocks_notify;
    void *clocks_opaque;
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
    /* Called after a register is written. */
    void (*written)(RP2350ClkRegsState *s, unsigned reg);
    /*
     * Handles a write itself, given the bus address (for its atomic
     * alias) and data, returning true; false stores it as usual.
     */
    bool (*write)(RP2350ClkRegsState *s, unsigned reg, hwaddr addr,
                  uint32_t value);
    /* Resets state beyond the register file. */
    void (*reset_hold)(RP2350ClkRegsState *s);
};

/*
 * Named GPIO input of XOSC and ROSC: a DORMANT wake event while high (the
 * GPIO banks' dormant_wake interrupt or the AON timer alarm).
 */
#define RP2350_OSC_DORMANT_WAKE "dormant-wake"

/*
 * Named GPIO output of XOSC and ROSC: high from entry to DORMANT until
 * the woken oscillator is stable again, while its output is gated and
 * every clock running from it is stopped.
 */
#define RP2350_OSC_DORMANT "dormant"

struct RP2350XOSCState {
    RP2350ClkRegsState parent_obj;

    QEMUTimer *startup_timer;
    qemu_irq dormant_irq;
    uint32_t xtal_hz;

    /* CTRL.ENABLE as last applied, to see the oscillator start. */
    bool enabled;
    bool badwrite;
    /* Stopped by DORMANT, waiting for a wake event. */
    bool dormant;
    /* Output gated: from DORMANT entry until stable after the wake. */
    bool gated;
    /* The level of the dormant-wake input. */
    bool wake;
    /* Virtual time from which an enabled, running oscillator is STABLE. */
    int64_t stable_ns;
};

/* Whether the XOSC's output is gated by DORMANT. */
bool rp2350_xosc_dormant(RP2350XOSCState *s);

/* The oscillators a clock can run from, for DORMANT. */
typedef enum RP2350ClockRoot {
    RP2350_ROOT_ROSC,
    RP2350_ROOT_XOSC,
    /* The LPOSC or a GPIO clock input: not stopped by DORMANT. */
    RP2350_ROOT_OTHER,
} RP2350ClockRoot;

/*
 * The oscillator clk_ref or clk_sys (CLOCKS' clock indices 4 and 5) runs
 * from through the glitchless and auxiliary muxes, with the PLLs counting
 * as their XOSC reference.
 */
RP2350ClockRoot rp2350_clocks_root(RP2350ClkRegsState *clocks, bool sys);

/*
 * Have `fn` called whenever the clk_ref or clk_sys source selection is
 * written or CLOCKS is reset.
 */
void rp2350_clocks_set_notify(RP2350ClkRegsState *clocks,
                              RP2350TickNotify *fn, void *opaque);

/*
 * Whether TICKS generator `tick` is producing ticks: it is enabled and
 * clk_ref, which it divides, is running.
 */
bool rp2350_ticks_running(RP2350ClkRegsState *ticks, int tick);

/*
 * Whether clk_sys is running, for blocks such as TIMER that count it
 * instead of their tick.
 */
bool rp2350_ticks_sys_running(RP2350ClkRegsState *ticks);

/*
 * Start or stop clk_ref and clk_sys (DORMANT), telling every generator's
 * consumer when either changes.
 */
void rp2350_ticks_set_clocks(RP2350ClkRegsState *ticks, bool ref_running,
                             bool sys_running);

/* TICKS generator `tick`'s CYCLES: clk_ref cycles per tick. */
uint32_t rp2350_ticks_cycles(RP2350ClkRegsState *ticks, int tick);

/*
 * Have `fn` called whenever TICKS generator `tick` is enabled or disabled,
 * its CYCLES is written, clk_ref or clk_sys starts or stops, or the TICKS
 * block is reset.
 */
void rp2350_ticks_set_notify(RP2350ClkRegsState *ticks, int tick,
                             RP2350TickNotify *fn, void *opaque);

#endif
