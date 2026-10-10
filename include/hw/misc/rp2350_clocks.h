/*
 * RP2350 clock generation: CLOCKS, XOSC, PLL_SYS/PLL_USB and TICKS
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_CLOCKS_H
#define HW_MISC_RP2350_CLOCKS_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

/*
 * The four blocks share a register-file base: configuration registers are
 * stored as written (through the atomic aliases), and each block derives
 * its status bits and its output clocks from that configuration.
 */
#define TYPE_RP2350_CLKREGS "rp2350-clkregs"
OBJECT_DECLARE_TYPE(RP2350ClkRegsState, RP2350ClkRegsClass, RP2350_CLKREGS)

#define TYPE_RP2350_CLOCKS "rp2350-clocks"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350ClocksState, RP2350_CLOCKS)
#define TYPE_RP2350_XOSC "rp2350-xosc"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350XOSCState, RP2350_XOSC)
#define TYPE_RP2350_PLL "rp2350-pll"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350PLLState, RP2350_PLL)
#define TYPE_RP2350_TICKS "rp2350-ticks"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350TicksState, RP2350_TICKS)

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
 * The clock generators of CLOCKS, in register order. Each is a clock
 * output of the CLOCKS device named by rp2350_clock_names[].
 */
enum {
    RP2350_CLK_GPOUT0,
    RP2350_CLK_GPOUT1,
    RP2350_CLK_GPOUT2,
    RP2350_CLK_GPOUT3,
    RP2350_CLK_REF,
    RP2350_CLK_SYS,
    RP2350_CLK_PERI,
    RP2350_CLK_HSTX,
    RP2350_CLK_USB,
    RP2350_CLK_ADC,
    RP2350_NUM_CLOCKS,
};

/*
 * The clock sources CLOCKS selects from, each a clock input of the CLOCKS
 * device named by rp2350_clock_source_names[]: the ring oscillator's two
 * outputs, the crystal oscillator, the LPOSC, the PLLs, the GPIN0/1 clock
 * inputs from GPIOs, the PLL_USB reference as the clock muxes see it
 * (pll_usb_primary_ref_opcg) and the OTP's clock for the frequency
 * counter. An unconnected input is a stopped clock.
 */
enum {
    RP2350_CLKSRC_ROSC,
    RP2350_CLKSRC_ROSC_PH,
    RP2350_CLKSRC_XOSC,
    RP2350_CLKSRC_LPOSC,
    RP2350_CLKSRC_PLL_SYS,
    RP2350_CLKSRC_PLL_USB,
    RP2350_CLKSRC_GPIN0,
    RP2350_CLKSRC_GPIN1,
    RP2350_CLKSRC_PLL_USB_REF,
    RP2350_CLKSRC_OTP,
    RP2350_NUM_CLKSRC,
};

extern const char *const rp2350_clock_names[RP2350_NUM_CLOCKS];
extern const char *const rp2350_clock_source_names[RP2350_NUM_CLKSRC];

struct RP2350ClocksState {
    RP2350ClkRegsState parent_obj;

    Clock *src[RP2350_NUM_CLKSRC];
    Clock *clk[RP2350_NUM_CLOCKS];
    qemu_irq irq;

    /* ENABLED of the generators that have an enable, by generator bit. */
    uint32_t enabled;

    /*
     * The clk_sys resuscitation circuit: whether it has switched clk_sys
     * to clk_ref, and the timer that fires once clk_sys has been stopped
     * for CLK_SYS_RESUS_CTRL.TIMEOUT clk_ref cycles.
     */
    bool resussed;
    QEMUTimer *resus_timer;

    /*
     * The frequency counter: the phase of its count, when that phase ends
     * in virtual time, whether it has reached the end but waits for a
     * stopped clk_ref, and whether the clock under test stopped during
     * the count. Over the count, the clock's edges so far (32.32 fixed
     * point) and its period since fc0_last_ns.
     */
    QEMUTimer *fc0_timer;
    uint32_t fc0_phase;
    bool fc0_stalled;
    bool fc0_died;
    int64_t fc0_end_ns;
    uint64_t fc0_edges;
    uint64_t fc0_last_period;
    int64_t fc0_last_ns;
};

/*
 * Configure CLOCKS as the boot ROM's early boot path leaves it: clk_ref
 * divided by 4, keeping it near its nominal frequency while the ROM runs
 * the ROSC four times faster.
 */
void rp2350_clocks_boot_rom_handoff(RP2350ClocksState *s);

/* Named clock output of XOSC: its output, xosc_clksrc. */
#define RP2350_XOSC_CLK "clk"

/*
 * Named GPIO input of XOSC and ROSC: a DORMANT wake event while high (the
 * GPIO banks' dormant_wake interrupt or the AON timer alarm).
 */
#define RP2350_OSC_DORMANT_WAKE "dormant-wake"

struct RP2350XOSCState {
    RP2350ClkRegsState parent_obj;

    QEMUTimer *startup_timer;
    Clock *out;
    uint32_t xtal_hz;

    /* CTRL's ENABLE and FREQ_RANGE (0-3) codes in force. */
    bool enabled;
    uint32_t freq_range;
    bool badwrite;
    /* Stopped by DORMANT, waiting for a wake event. */
    bool dormant;
    /* Output gated: from DORMANT entry until stable after the wake. */
    bool gated;
    /* The level of the dormant-wake input. */
    bool wake;
    /* Virtual time from which an enabled, running oscillator is STABLE. */
    int64_t stable_ns;

    /* COUNT as of virtual time count_ns; it counts down from there. */
    uint32_t count;
    int64_t count_ns;
};

/*
 * Named clock input and output of a PLL: its reference (FREF, the
 * crystal oscillator's XIN) and its output (FOUTPOSTDIV).
 */
#define RP2350_PLL_REF "ref"
#define RP2350_PLL_OUT "out"

struct RP2350PLLState {
    RP2350ClkRegsState parent_obj;

    Clock *ref;
    Clock *out;
    qemu_irq irq;
    QEMUTimer *lock_timer;

    /* Whether the VCO is locked, or is acquiring lock until lock_ns. */
    bool locked;
    bool locking;
    int64_t lock_ns;
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

/*
 * Named clock input of TICKS, clk_ref; its outputs are the generators'
 * ticks, named by rp2350_tick_names[]. A tick clock runs at clk_ref
 * divided by the generator's CYCLES while the generator is enabled, and
 * is stopped otherwise.
 */
#define RP2350_TICKS_REF "clk-ref"

extern const char *const rp2350_tick_names[RP2350_NUM_TICKS];

struct RP2350TicksState {
    RP2350ClkRegsState parent_obj;

    Clock *ref;
    Clock *tick[RP2350_NUM_TICKS];
    /*
     * The virtual time from which each running generator has counted its
     * current CYCLES down without a change of rate.
     */
    int64_t phase_ns[RP2350_NUM_TICKS];
};

#endif
