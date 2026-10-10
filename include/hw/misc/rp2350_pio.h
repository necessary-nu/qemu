/*
 * RP2350 programmable I/O (PIO0, PIO1 and PIO2)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The three PIO blocks are one device: they run off the same clk_sys and
 * reach into each other through the PREV/NEXT IRQ flags and the CTRL
 * NEXTPREV operations, so they are stepped in lockstep.
 *
 * Inputs and outputs:
 *
 *  - Sysbus MMIO regions 0-2: PIO0-PIO2's register blocks, each with its
 *    atomic XOR/SET/CLR aliases.
 *  - Sysbus IRQs 0-5: PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0, ... PIO2_IRQ_1.
 *  - Named GPIO output arrays "out" and "oe", indexed by
 *    block * RP2350_PIO_GPIOS + GPIO: the level and output enable each
 *    block drives on its GPIO function signal for that bank 0 GPIO.
 *  - Named GPIO input array "in", indexed by bank 0 GPIO: the level of the
 *    pin. PIO sees every pin whether its FUNCSEL selects PIO or not, and
 *    all three blocks see the same levels.
 *  - Named GPIO output array "dreq", indexed by DREQ number (block * 8 +
 *    state machine for TX, + 4 for RX): high while the TX FIFO has room or
 *    the RX FIFO holds data.
 *  - Named GPIO output array "hstx", indexed by block * 8 + n: the level
 *    the block drives for GPIO 12 + n, which HSTX's coupled mode takes.
 *    These are timed to the cycle whether or not a pin selects PIO.
 *  - Clock input "clk": clk_sys.
 *  - Link "gpio" (optional): the IO bank. Output changes on GPIOs no pin
 *    selects for PIO are invisible outside the block, so the model does
 *    not time them to the cycle; without the link every output is timed.
 */

#ifndef HW_MISC_RP2350_PIO_H
#define HW_MISC_RP2350_PIO_H

#include "hw/core/sysbus.h"
#include "hw/gpio/rp2350_gpio.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_PIO "rp2350-pio"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350PIOState, RP2350_PIO)

#define RP2350_PIO0_BASE 0x50200000
#define RP2350_PIO_STRIDE 0x100000

#define RP2350_PIO_BLOCKS 3
#define RP2350_PIO_SMS 4
#define RP2350_PIO_IMEM 32
#define RP2350_PIO_FIFO_DEPTH 4
#define RP2350_PIO_IRQS_PER_BLOCK 2
#define RP2350_PIO_IRQS (RP2350_PIO_BLOCKS * RP2350_PIO_IRQS_PER_BLOCK)
#define RP2350_PIO_DREQS (RP2350_PIO_BLOCKS * 2 * RP2350_PIO_SMS)
/* Bank 0 GPIOs a block can reach: 32 at a time, from GPIOBASE. */
#define RP2350_PIO_GPIOS 48

#define RP2350_PIO_OUT "out"
#define RP2350_PIO_OE "oe"
#define RP2350_PIO_IN "in"
#define RP2350_PIO_DREQ "dreq"
#define RP2350_PIO_HSTX "hstx"

/* The GPIOs HSTX can take from PIO in coupled mode. */
#define RP2350_PIO_HSTX_FIRST 12
#define RP2350_PIO_HSTX_BITS 8

/* Input pin history entries; see RP2350PIOCore. */
#define RP2350_PIO_IN_HIST 4
/* Pin changes from outside waiting to be applied; see RP2350PIOState. */
#define RP2350_PIO_IN_QUEUE 8

typedef struct RP2350PIOSM {
    /* Configuration registers. EXEC_STALLED is derived, not stored. */
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t pinctrl;

    uint32_t x;
    uint32_t y;
    uint32_t isr;
    uint32_t osr;
    uint8_t pc;
    /* Input and output shift counters, 0-32. */
    uint8_t isr_count;
    uint8_t osr_count;
    /* Delay cycles left before the next instruction. */
    uint8_t delay;
    /* The instruction latch (OUT/MOV EXEC, or a stalled SMx_INSTR). */
    uint16_t latch;
    /* An SMx_INSTR write that has not yet executed. */
    uint16_t forced;
    uint16_t flags;

    /*
     * The FIFO storage: TX in entries 0-3 and RX in 4-7, or all eight for
     * a joined FIFO. Each FIFO is a ring from its head.
     */
    uint32_t fifo[2 * RP2350_PIO_FIFO_DEPTH];
    uint8_t tx_head;
    uint8_t tx_level;
    uint8_t rx_head;
    uint8_t rx_level;

    /*
     * The clock divider: a first-order sigma-delta accumulator in 1/256
     * cycle units as of the start of clk_sys cycle div_cycle, and the next
     * cycle in which it gives the state machine a clock enable.
     */
    uint32_t div_acc;
    uint64_t div_cycle;
    uint64_t next_en;

    /* The OUT/SET pin write OUT_STICKY keeps asserting. */
    uint32_t sticky_mask;
    uint32_t sticky_data;

    /*
     * A one-instruction JMP X--/Y-- loop run in closed form: from the
     * divider state loop_acc at loop_start, with loop_count more taken
     * iterations of 1 + loop_delay enabled cycles each to come.
     */
    uint64_t loop_start;
    uint32_t loop_acc;
    uint32_t loop_count;
    uint8_t loop_delay;
} RP2350PIOSM;

typedef struct RP2350PIOBlock {
    uint16_t imem[RP2350_PIO_IMEM];
    RP2350PIOSM sm[RP2350_PIO_SMS];
    uint32_t sm_enable;
    uint32_t fdebug;
    uint32_t irq;
    uint32_t sync_bypass;
    uint32_t gpiobase;
    uint32_t inte[RP2350_PIO_IRQS_PER_BLOCK];
    uint32_t intf[RP2350_PIO_IRQS_PER_BLOCK];
    /* The block's pin output levels and enables, in its 32-pin space. */
    uint32_t pad_out;
    uint32_t pad_oe;
    /* A cycle in which sticky writes must be resolved again, or never. */
    uint64_t resolve_at;
    /* ACCESSCTRL grants Non-secure access to the block. */
    bool ns;
    /* RESETS holds the block in reset. */
    bool held;
} RP2350PIOBlock;

/*
 * Everything the state machines' execution depends on and changes. The
 * model runs a copy of it ahead of virtual time to find when the blocks
 * next do something visible outside, so it holds no pointers.
 */
typedef struct RP2350PIOCore {
    RP2350PIOBlock blk[RP2350_PIO_BLOCKS];
    /* The next clk_sys cycle to run. */
    uint64_t cycle;
    /*
     * State machines (bit block * 4 + sm) stalled with nothing changed
     * since that could release them; cleared by every change one state
     * machine can see of another's.
     */
    uint32_t idle;
    /* State machines with an SMx_INSTR write pending or latched. */
    uint32_t xmask;
    /*
     * Pin levels (bank 0 GPIOs) and the cycle from which each applies,
     * oldest first; enough to look two cycles back through the input
     * synchronisers.
     */
    uint64_t in_val[RP2350_PIO_IN_HIST];
    uint64_t in_cycle[RP2350_PIO_IN_HIST];
    uint32_t in_n;
    /* The first cycle in which the newest pin levels are synchronised. */
    uint64_t in_settle;
    /* ACCESSCTRL GPIO_NSMASK1:0 (SIO layout). */
    uint64_t gpio_nsmask;
} RP2350PIOCore;

/* One block's register window. */
typedef struct RP2350PIOWindow {
    RP2350PIOState *s;
    int block;
    MemoryRegion iomem;
} RP2350PIOWindow;

struct RP2350PIOState {
    SysBusDevice parent_obj;

    RP2350PIOWindow win[RP2350_PIO_BLOCKS];
    Clock *clk;
    RP2350GPIOState *gpio;
    QEMUTimer *timer;

    qemu_irq irq[RP2350_PIO_IRQS];
    qemu_irq out[RP2350_PIO_BLOCKS * RP2350_PIO_GPIOS];
    qemu_irq oe[RP2350_PIO_BLOCKS * RP2350_PIO_GPIOS];
    qemu_irq dreq[RP2350_PIO_DREQS];
    qemu_irq hstx[RP2350_PIO_BLOCKS * RP2350_PIO_HSTX_BITS];

    RP2350PIOCore core;

    /* The run ahead, valid until anything outside disturbs the blocks. */
    RP2350PIOCore spec;
    bool spec_valid;
    /* How many cycles the next run ahead may cover. */
    uint64_t horizon;
    /* Set while the engine runs or drives its outputs. */
    bool busy;

    /* clk_sys cycle count at virtual time base_ns. */
    int64_t base_ns;
    uint64_t base_cycle;

    /* The latest pin levels delivered on "in". */
    uint64_t in_level;
    /*
     * Pin changes from outside the blocks and the cycles they came in,
     * oldest first. They arrive while the IO bank propagates, when the
     * blocks cannot run (their pin changes would not reach the bank in
     * time), so they wait here for in_timer, due at once, or the next
     * catch-up. Each carries only the pins that changed (inq_mask): the
     * blocks' own pins may change before it applies.
     */
    uint64_t inq_vec[RP2350_PIO_IN_QUEUE];
    uint64_t inq_mask[RP2350_PIO_IN_QUEUE];
    uint64_t inq_cycle[RP2350_PIO_IN_QUEUE];
    uint32_t inq_n;
    QEMUTimer *in_timer;

    /* Levels last driven on the outputs, GPIO-numbered for the pins. */
    uint64_t drv_out[RP2350_PIO_BLOCKS];
    uint64_t drv_oe[RP2350_PIO_BLOCKS];
    uint32_t drv_irq;
    uint32_t drv_dreq;
};

/*
 * Hold block `n` in reset (`hold` true) or release it, as its RESETS
 * subsystem reset does. A held block is reset and inert: it runs nothing,
 * drives its IRQs and DREQs low, and is cut off from the other blocks.
 * Its registers are not reachable; the SoC gates its bus window.
 */
void rp2350_pio_hold_block(RP2350PIOState *s, int n, bool hold);

/*
 * The blocks' Non-secure accessibility (bit n for PIOn) and ACCESSCTRL
 * GPIO_NSMASK1:0 in the SIO layout. A Non-secure block reads Secure pins
 * as 0, and blocks of different accessibility do not see each other's
 * IRQ flags or CTRL operations.
 */
void rp2350_pio_set_security(RP2350PIOState *s, uint32_t ns_blocks,
                             uint64_t gpio_nsmask);

#endif
