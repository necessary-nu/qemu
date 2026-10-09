/*
 * RP2350 programmable I/O (PIO0, PIO1 and PIO2)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "PIO", and the pico-sdk pio.h register
 * descriptions.
 *
 * Each block has 32 words of write-only instruction memory and four state
 * machines, each with X, Y, ISR, OSR, shift counters, a 4+4 word FIFO pair
 * (joinable, or open to random access with FJOIN_RX_PUT/GET), a 16.8
 * fractional clock divider and pin mapping. The model runs the state
 * machines cycle by cycle in clk_sys cycles:
 *
 *  - The clock divider is first-order sigma-delta: every clk_sys cycle
 *    adds 256 to an accumulator, and the cycle in which it reaches the
 *    divisor (in 1/256ths) gives a clock enable and subtracts the divisor.
 *    A CLKDIV_RESTART leaves it one cycle short of an enable, so a state
 *    machine whose divider restarts with its enable runs in that cycle.
 *  - In each enabled cycle a state machine counts down a delay, or
 *    executes one instruction, which completes or stalls. Side-set happens
 *    in the first cycle of an instruction only.
 *  - State machines see the IRQ flags and pins as they were at the start
 *    of the cycle. IRQ flag sets and clears take effect at its end (a set
 *    wins over a clear), so another state machine sees them the next
 *    cycle; pin writes are resolved at its end, the highest-numbered state
 *    machine winning and side-set winning within one, and reach the pads
 *    the next cycle.
 *  - Pins come back through a two-flop synchroniser: a state machine sees
 *    a pin level two cycles after it reaches the pad, or in the same cycle
 *    for pins in INPUT_SYNC_BYPASS.
 *  - Registers are accessed between cycles. An instruction written to
 *    SMx_INSTR executes in the next cycle, which the write runs at once.
 *
 * Time. The blocks hold their state as of a clk_sys cycle and are run
 * forward to the current virtual time before every register access and
 * pin input change. Cycles in which no state machine has anything to do
 * are skipped: a state machine is only run in cycles in which its divider
 * gives an enable, and one that stalled with nothing changed since that
 * could release it is not run at all. To time the outputs (DREQs,
 * interrupts and pins that a GPIO's FUNCSEL routes to PIO), the model runs
 * a copy of the blocks ahead of virtual time until one changes, and fires
 * a timer at that cycle; if nothing disturbed the blocks meanwhile, the
 * copy becomes the state, so the cycles are run once. While no output
 * changes, the run ahead is limited to a horizon that starts short after
 * every disturbance and grows while there are none. Output changes on
 * GPIOs that no pin routes to PIO are not timed: they are delivered
 * whenever the blocks next catch up.
 *
 * Not modelled: the RESETS hold of a block (it runs while held), and pad
 * input and output delays beyond the synchronisers.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_pio.h"
#include "migration/vmstate.h"

#define A_CTRL                  0x000
#define A_FSTAT                 0x004
#define A_FDEBUG                0x008
#define A_FLEVEL                0x00c
#define A_TXF0                  0x010
#define A_RXF0                  0x020
#define A_IRQ                   0x030
#define A_IRQ_FORCE             0x034
#define A_INPUT_SYNC_BYPASS     0x038
#define A_DBG_PADOUT            0x03c
#define A_DBG_PADOE             0x040
#define A_DBG_CFGINFO           0x044
#define A_INSTR_MEM0            0x048
#define A_SM0_CLKDIV            0x0c8
#define SM_STRIDE               0x18
#define A_RXF0_PUTGET0          0x128
#define A_GPIOBASE              0x168
#define A_INTR                  0x16c
#define A_IRQ0_INTE             0x170
#define A_IRQ1_INTS             0x184

/* Per state machine, from SMn_CLKDIV. */
#define SM_CLKDIV               0x00
#define SM_EXECCTRL             0x04
#define SM_SHIFTCTRL            0x08
#define SM_ADDR                 0x0c
#define SM_INSTR                0x10
#define SM_PINCTRL              0x14

#define CTRL_SM_ENABLE(v)       ((v) & 0xf)
#define CTRL_SM_RESTART(v)      (((v) >> 4) & 0xf)
#define CTRL_CLKDIV_RESTART(v)  (((v) >> 8) & 0xf)
#define CTRL_PREV_MASK(v)       (((v) >> 16) & 0xf)
#define CTRL_NEXT_MASK(v)       (((v) >> 20) & 0xf)
#define CTRL_NEXTPREV_ENABLE    (1u << 24)
#define CTRL_NEXTPREV_DISABLE   (1u << 25)
#define CTRL_NEXTPREV_CLKDIV    (1u << 26)

#define FDEBUG_RXSTALL(i)       (1u << (i))
#define FDEBUG_RXUNDER(i)       (1u << (8 + (i)))
#define FDEBUG_TXOVER(i)        (1u << (16 + (i)))
#define FDEBUG_TXSTALL(i)       (1u << (24 + (i)))
#define FDEBUG_MASK             0x0f0f0f0fu

#define CLKDIV_MASK             0xffffff00u
#define CLKDIV_RESET            0x00010000u

#define EXECCTRL_EXEC_STALLED   (1u << 31)
#define EXECCTRL_SIDE_EN        (1u << 30)
#define EXECCTRL_SIDE_PINDIR    (1u << 29)
#define EXECCTRL_JMP_PIN(v)     (((v) >> 24) & 0x1f)
#define EXECCTRL_OUT_EN_SEL(v)  (((v) >> 19) & 0x1f)
#define EXECCTRL_INLINE_OUT_EN  (1u << 18)
#define EXECCTRL_OUT_STICKY     (1u << 17)
#define EXECCTRL_WRAP_TOP(v)    (((v) >> 12) & 0x1f)
#define EXECCTRL_WRAP_BOTTOM(v) (((v) >> 7) & 0x1f)
#define EXECCTRL_STATUS_SEL(v)  (((v) >> 5) & 3)
#define EXECCTRL_STATUS_N(v)    ((v) & 0x1f)
#define EXECCTRL_RW             0x7fffffffu
#define EXECCTRL_RESET          0x0001f000u

#define SHIFTCTRL_FJOIN_RX      (1u << 31)
#define SHIFTCTRL_FJOIN_TX      (1u << 30)
#define SHIFTCTRL_PULL_THRESH(v) (((v) >> 25) & 0x1f)
#define SHIFTCTRL_PUSH_THRESH(v) (((v) >> 20) & 0x1f)
#define SHIFTCTRL_OUT_SHIFTDIR  (1u << 19)
#define SHIFTCTRL_IN_SHIFTDIR   (1u << 18)
#define SHIFTCTRL_AUTOPULL      (1u << 17)
#define SHIFTCTRL_AUTOPUSH      (1u << 16)
#define SHIFTCTRL_FJOIN_RX_PUT  (1u << 15)
#define SHIFTCTRL_FJOIN_RX_GET  (1u << 14)
#define SHIFTCTRL_IN_COUNT(v)   ((v) & 0x1f)
#define SHIFTCTRL_RW            0xffffc01fu
#define SHIFTCTRL_RESET         0x000c0000u
#define SHIFTCTRL_FIFO_MODE     (SHIFTCTRL_FJOIN_RX | SHIFTCTRL_FJOIN_TX | \
                                 SHIFTCTRL_FJOIN_RX_PUT | \
                                 SHIFTCTRL_FJOIN_RX_GET)
#define SHIFTCTRL_PUTGET        (SHIFTCTRL_FJOIN_RX_PUT | \
                                 SHIFTCTRL_FJOIN_RX_GET)

#define PINCTRL_SIDESET_COUNT(v) ((v) >> 29)
#define PINCTRL_SET_COUNT(v)    (((v) >> 26) & 7)
#define PINCTRL_OUT_COUNT(v)    (((v) >> 20) & 0x3f)
#define PINCTRL_IN_BASE(v)      (((v) >> 15) & 0x1f)
#define PINCTRL_SIDESET_BASE(v) (((v) >> 10) & 0x1f)
#define PINCTRL_SET_BASE(v)     (((v) >> 5) & 0x1f)
#define PINCTRL_OUT_BASE(v)     ((v) & 0x1f)
#define PINCTRL_RESET           0x14000000u

#define GPIOBASE_MASK           0x10u

/* VERSION 1, IMEM_SIZE 32, SM_COUNT 4, FIFO_DEPTH 4. */
#define DBG_CFGINFO_VALUE       0x10200404u

#define INTR_MASK               0xffffu

/* RP2350PIOSM.flags */
#define SMF_STALLED             (1u << 0)   /* the instruction stalled */
#define SMF_IRQ_WAIT            (1u << 1)   /* IRQ WAIT has set its flag */
#define SMF_LATCH               (1u << 2)   /* the latch holds an instruction */
#define SMF_LATCH_FORCED        (1u << 3)   /* ... a stalled SMx_INSTR */
#define SMF_FORCED              (1u << 4)   /* an SMx_INSTR write waits */
#define SMF_STICKY              (1u << 5)   /* a sticky pin write is held */
#define SMF_STICKY_DIR          (1u << 6)   /* ... to the pin directions */

/* Instruction classes (bits 15:13) and fields. */
enum {
    OP_JMP,
    OP_WAIT,
    OP_IN,
    OP_OUT,
    OP_PUSH_PULL,
    OP_MOV,
    OP_IRQ,
    OP_SET,
};

#define INSTR_OP(i)             ((i) >> 13)
#define INSTR_DS(i)             (((i) >> 8) & 0x1f)
#define INSTR_ARG1(i)           (((i) >> 5) & 7)
#define INSTR_ARG2(i)           ((i) & 0x1f)

#define NO_CYCLE                UINT64_MAX

/*
 * The run-ahead horizon in clk_sys cycles: from MIN after a disturbance,
 * doubling each time a run ahead is committed undisturbed, up to MAX.
 * MAX also bounds how stale an untimed output can be.
 */
#define HORIZON_MIN             64
#define HORIZON_MAX             (1u << 17)

#define GPIO_MASK               MAKE_64BIT_MASK(0, RP2350_PIO_GPIOS)

/* Pin writes collected from one cycle. */
typedef struct PIOPinWrite {
    uint32_t mask;
    uint32_t data;
    bool dir;
} PIOPinWrite;

enum {
    CH_OUT,         /* OUT, SET and MOV to PINS or PINDIRS */
    CH_SIDE,        /* side-set */
    CH_COUNT,
};

typedef struct PIOCycle {
    uint8_t irq_set[RP2350_PIO_BLOCKS];
    uint8_t irq_clr[RP2350_PIO_BLOCKS];
    /* Bit sm * CH_COUNT + channel for each write made. */
    uint32_t wvalid[RP2350_PIO_BLOCKS];
    PIOPinWrite w[RP2350_PIO_BLOCKS][RP2350_PIO_SMS][CH_COUNT];
    /* A FIFO level changed. */
    bool fifo;
} PIOCycle;

typedef enum PIOExecKind {
    EXEC_MEM,       /* from instruction memory */
    EXEC_LATCH,     /* run by OUT/MOV EXEC */
    EXEC_FORCED,    /* written to SMx_INSTR */
} PIOExecKind;

static inline uint32_t mask_n(unsigned n)
{
    return n >= 32 ? UINT32_MAX : (1u << n) - 1;
}

/* A 5-bit count field in which 0 means 32. */
static inline unsigned count32(unsigned field)
{
    return field ? field : 32;
}

static int blk_prev(int b)
{
    return (b + RP2350_PIO_BLOCKS - 1) % RP2350_PIO_BLOCKS;
}

static int blk_next(int b)
{
    return (b + 1) % RP2350_PIO_BLOCKS;
}

/*
 * Blocks of different Non-secure accessibility are disconnected: status
 * from the other reads 0 and controls to it are ignored.
 */
static bool blk_linked(const RP2350PIOCore *core, int b, int other)
{
    return core->blk[b].ns == core->blk[other].ns &&
           !core->blk[b].held && !core->blk[other].held;
}

/* FIFOs */

static unsigned tx_cap(const RP2350PIOSM *sm)
{
    if (sm->shiftctrl & SHIFTCTRL_FJOIN_RX) {
        return 0;
    }
    return sm->shiftctrl & SHIFTCTRL_FJOIN_TX ? 2 * RP2350_PIO_FIFO_DEPTH
                                              : RP2350_PIO_FIFO_DEPTH;
}

static unsigned rx_cap(const RP2350PIOSM *sm)
{
    if (sm->shiftctrl & (SHIFTCTRL_FJOIN_TX | SHIFTCTRL_PUTGET)) {
        return 0;
    }
    return sm->shiftctrl & SHIFTCTRL_FJOIN_RX ? 2 * RP2350_PIO_FIFO_DEPTH
                                              : RP2350_PIO_FIFO_DEPTH;
}

static unsigned rx_base(const RP2350PIOSM *sm)
{
    return sm->shiftctrl & SHIFTCTRL_FJOIN_RX ? 0 : RP2350_PIO_FIFO_DEPTH;
}

static bool tx_full(const RP2350PIOSM *sm)
{
    return sm->tx_level >= tx_cap(sm);
}

static bool rx_full(const RP2350PIOSM *sm)
{
    return sm->rx_level >= rx_cap(sm);
}

static void tx_push(RP2350PIOSM *sm, uint32_t v)
{
    unsigned cap = tx_cap(sm);

    sm->fifo[(sm->tx_head + sm->tx_level) % cap] = v;
    sm->tx_level++;
}

static uint32_t tx_pop(RP2350PIOSM *sm)
{
    uint32_t v = sm->fifo[sm->tx_head];

    sm->tx_head = (sm->tx_head + 1) % tx_cap(sm);
    sm->tx_level--;
    return v;
}

static void rx_push(RP2350PIOSM *sm, uint32_t v)
{
    unsigned cap = rx_cap(sm);

    sm->fifo[rx_base(sm) + (sm->rx_head + sm->rx_level) % cap] = v;
    sm->rx_level++;
}

/* The RX FIFO's head entry, or the word a read of an empty FIFO returns. */
static uint32_t rx_peek(const RP2350PIOSM *sm)
{
    return sm->fifo[rx_base(sm) + sm->rx_head];
}

static uint32_t rx_pop(RP2350PIOSM *sm)
{
    uint32_t v = rx_peek(sm);

    sm->rx_head = (sm->rx_head + 1) % rx_cap(sm);
    sm->rx_level--;
    return v;
}

static void fifo_flush(RP2350PIOSM *sm)
{
    sm->tx_head = sm->tx_level = 0;
    sm->rx_head = sm->rx_level = 0;
}

/* Clock dividers */

/* The divisor in 1/256ths of a cycle; INT 0 divides by 65536. */
static uint32_t sm_div(const RP2350PIOSM *sm)
{
    uint32_t i = sm->clkdiv >> 16;

    return (i ? i : 65536) * 256 + ((sm->clkdiv >> 8) & 0xff);
}

static void div_advance(RP2350PIOSM *sm, uint64_t to)
{
    uint64_t d = sm_div(sm);

    if (to > sm->div_cycle) {
        sm->div_acc = (sm->div_acc + 256 * ((to - sm->div_cycle) % d)) % d;
        sm->div_cycle = to;
    }
}

/* The first cycle from `from` in which the divider gives an enable. */
static uint64_t div_next(RP2350PIOSM *sm, uint64_t from)
{
    uint32_t d = sm_div(sm);

    div_advance(sm, from);
    from = MAX(from, sm->div_cycle);
    return from + DIV_ROUND_UP(d - sm->div_acc, 256) - 1;
}

static void div_restart(RP2350PIOSM *sm, uint64_t cycle)
{
    sm->div_cycle = cycle;
    sm->div_acc = sm_div(sm) - 256;
    sm->next_en = cycle;
}

/* Pins */

static uint64_t core_in_at(const RP2350PIOCore *core, uint64_t t)
{
    int k;

    for (k = core->in_n - 1; k > 0; k--) {
        if (core->in_cycle[k] <= t) {
            break;
        }
    }
    return core->in_val[k];
}

/*
 * Record the pin levels `vec` from cycle `t` on, keeping only the history
 * still to be seen through the synchronisers.
 */
static void core_input(RP2350PIOCore *core, uint64_t vec, uint64_t t)
{
    int keep = 0, k;

    if (core->in_cycle[core->in_n - 1] >= t) {
        core->in_val[core->in_n - 1] = vec;
    } else {
        /* The newest entry from at least two cycles before t is enough. */
        for (k = core->in_n - 1; k >= 0; k--) {
            if (core->in_cycle[k] + 2 <= t) {
                keep = k;
                break;
            }
        }
        if (keep) {
            memmove(&core->in_val[0], &core->in_val[keep],
                    (core->in_n - keep) * sizeof(core->in_val[0]));
            memmove(&core->in_cycle[0], &core->in_cycle[keep],
                    (core->in_n - keep) * sizeof(core->in_cycle[0]));
            core->in_n -= keep;
        }
        assert(core->in_n < RP2350_PIO_IN_HIST);
        core->in_val[core->in_n] = vec;
        core->in_cycle[core->in_n] = t;
        core->in_n++;
    }
    core->in_settle = t + 2;
    core->seq++;
}

/* A block's view of the pins at cycle c, in its 32-pin space. */
static uint32_t blk_pins(const RP2350PIOCore *core, int b, uint64_t c)
{
    const RP2350PIOBlock *blk = &core->blk[b];
    uint64_t allowed = blk->ns ? core->gpio_nsmask : UINT64_MAX;
    uint32_t raw = (core_in_at(core, c) & allowed) >> blk->gpiobase;
    uint32_t syn = (core_in_at(core, c >= 2 ? c - 2 : 0) & allowed) >>
                   blk->gpiobase;

    return (syn & ~blk->sync_bypass) | (raw & blk->sync_bypass);
}

/* The IN-mapped pins: rotated to IN_BASE and masked to IN_COUNT. */
static uint32_t sm_in_pins(const RP2350PIOCore *core, int b,
                           const RP2350PIOSM *sm, uint64_t c)
{
    return ror32(blk_pins(core, b, c), PINCTRL_IN_BASE(sm->pinctrl)) &
           mask_n(count32(SHIFTCTRL_IN_COUNT(sm->shiftctrl)));
}

static void pin_write(PIOCycle *cx, int b, int i, int ch, bool dir,
                      unsigned base, unsigned count, uint32_t data)
{
    PIOPinWrite *w = &cx->w[b][i][ch];
    uint32_t m = mask_n(count);

    w->dir = dir;
    w->mask = rol32(m, base);
    w->data = rol32(data & m, base);
    cx->wvalid[b] |= 1u << (i * CH_COUNT + ch);
}

/* An OUT, SET or MOV write to the pins, which OUT_STICKY then holds. */
static void out_write(PIOCycle *cx, int b, int i, RP2350PIOSM *sm,
                      bool dir, unsigned base, unsigned count, uint32_t data)
{
    PIOPinWrite *w = &cx->w[b][i][CH_OUT];

    pin_write(cx, b, i, CH_OUT, dir, base, count, data);
    if (sm->execctrl & EXECCTRL_OUT_STICKY) {
        sm->sticky_mask = w->mask;
        sm->sticky_data = w->data;
        sm->flags |= SMF_STICKY;
        if (dir) {
            sm->flags |= SMF_STICKY_DIR;
        } else {
            sm->flags &= ~SMF_STICKY_DIR;
        }
    }
}

static void apply_write(uint32_t *out, uint32_t *oe, uint32_t mask,
                        uint32_t data, bool dir)
{
    uint32_t *r = dir ? oe : out;

    *r = (*r & ~mask) | (data & mask);
}

/* IRQ flags */

/*
 * Decode an IRQ index field (IRQ, WAIT IRQ): bits 4:3 select this block,
 * the previous (PREV) or next (NEXT) block, or this block with the state
 * machine number added to the low two bits (REL).
 */
static void irq_target(int b, int i, unsigned field, int *tb, unsigned *flag)
{
    unsigned n = field & 7;

    switch ((field >> 3) & 3) {
    case 1:
        *tb = blk_prev(b);
        break;
    case 2:
        *tb = b;
        n = (n & 4) | ((n + i) & 3);
        break;
    case 3:
        *tb = blk_next(b);
        break;
    default:
        *tb = b;
        break;
    }
    *flag = n;
}

static bool sm_status(const RP2350PIOCore *core, int b, const RP2350PIOSM *sm)
{
    unsigned n = EXECCTRL_STATUS_N(sm->execctrl);
    int tb;

    switch (EXECCTRL_STATUS_SEL(sm->execctrl)) {
    case 0:
        return sm->tx_level < n;
    case 1:
        return sm->rx_level < n;
    case 2:
        switch (n >> 3) {
        case 0:
            tb = b;
            break;
        case 1:
            tb = blk_prev(b);
            break;
        case 2:
            tb = blk_next(b);
            break;
        default:
            return false;
        }
        return blk_linked(core, b, tb) && (core->blk[tb].irq >> (n & 7)) & 1;
    default:
        return false;
    }
}

/* Execution */

static unsigned pull_thresh(const RP2350PIOSM *sm)
{
    return count32(SHIFTCTRL_PULL_THRESH(sm->shiftctrl));
}

static unsigned push_thresh(const RP2350PIOSM *sm)
{
    return count32(SHIFTCTRL_PUSH_THRESH(sm->shiftctrl));
}

/* Refill the OSR from the TX FIFO if it has reached the pull threshold. */
static bool autopull(RP2350PIOSM *sm, PIOCycle *cx)
{
    if ((sm->shiftctrl & SHIFTCTRL_AUTOPULL) &&
        sm->osr_count >= pull_thresh(sm) && sm->tx_level) {
        sm->osr = tx_pop(sm);
        sm->osr_count = 0;
        cx->fifo = true;
        return true;
    }
    return false;
}

static uint32_t mov_source(const RP2350PIOCore *core, int b,
                           const RP2350PIOSM *sm, unsigned src, uint64_t c)
{
    switch (src) {
    case 0:
        return sm_in_pins(core, b, sm, c);
    case 1:
        return sm->x;
    case 2:
        return sm->y;
    case 5:
        return sm_status(core, b, sm) ? UINT32_MAX : 0;
    case 6:
        return sm->isr;
    case 7:
        return sm->osr;
    default:
        return 0;
    }
}

/* Shift `n` bits of `data` into the ISR. */
static void isr_shift(RP2350PIOSM *sm, uint32_t data, unsigned n)
{
    data &= mask_n(n);
    if (n >= 32) {
        sm->isr = data;
    } else if (sm->shiftctrl & SHIFTCTRL_IN_SHIFTDIR) {
        sm->isr = (sm->isr >> n) | (data << (32 - n));
    } else {
        sm->isr = (sm->isr << n) | data;
    }
    sm->isr_count = MIN(32, sm->isr_count + n);
}

/* Shift `n` bits out of the OSR. */
static uint32_t osr_shift(RP2350PIOSM *sm, unsigned n)
{
    uint32_t data;

    if (n >= 32) {
        data = sm->osr;
        sm->osr = 0;
    } else if (sm->shiftctrl & SHIFTCTRL_OUT_SHIFTDIR) {
        data = sm->osr & mask_n(n);
        sm->osr >>= n;
    } else {
        data = sm->osr >> (32 - n);
        sm->osr <<= n;
    }
    sm->osr_count = MIN(32, sm->osr_count + n);
    return data;
}

/*
 * Execute `instr` on state machine i of block b in cycle c. Returns
 * whether the state machine changed state other than stalling again.
 */
/* [spec:nuos:req:emu.pio] */
static bool sm_exec(RP2350PIOCore *core, int b, int i, uint16_t instr,
                    PIOExecKind kind, uint64_t c, PIOCycle *cx)
{
    RP2350PIOBlock *blk = &core->blk[b];
    RP2350PIOSM *sm = &blk->sm[i];
    uint32_t ec = sm->execctrl, pinctrl = sm->pinctrl;
    unsigned ss_count = MIN(PINCTRL_SIDESET_COUNT(pinctrl), 5);
    unsigned ds = INSTR_DS(instr);
    unsigned delay = ds & mask_n(5 - ss_count);
    unsigned arg1 = INSTR_ARG1(instr), arg2 = INSTR_ARG2(instr);
    bool first = !(sm->flags & SMF_STALLED);
    bool done = true, is_out = false, exec = false, progress = false;
    int new_pc = -1;
    uint32_t v = 0;
    unsigned n;
    int tb;
    unsigned flag;

    if (first && ss_count) {
        uint32_t ss = ds >> (5 - ss_count);
        unsigned bits = ss_count;
        bool go = true;

        if (ec & EXECCTRL_SIDE_EN) {
            bits--;
            go = (ss >> bits) & 1;
        }
        if (go && bits) {
            pin_write(cx, b, i, CH_SIDE, ec & EXECCTRL_SIDE_PINDIR,
                      PINCTRL_SIDESET_BASE(pinctrl), bits, ss);
        }
    }

    switch (INSTR_OP(instr)) {
    case OP_JMP: {
        bool take;

        switch (arg1) {
        case 0:
            take = true;
            break;
        case 1:
            take = !sm->x;
            break;
        case 2:
            take = sm->x;
            sm->x--;
            break;
        case 3:
            take = !sm->y;
            break;
        case 4:
            take = sm->y;
            sm->y--;
            break;
        case 5:
            take = sm->x != sm->y;
            break;
        case 6:
            take = (blk_pins(core, b, c) >> EXECCTRL_JMP_PIN(ec)) & 1;
            break;
        default:
            take = sm->osr_count < pull_thresh(sm);
            break;
        }
        if (take) {
            new_pc = arg2;
        }
        break;
    }

    case OP_WAIT: {
        bool pol = arg1 & 4;
        bool level;

        switch (arg1 & 3) {
        case 0:
            level = (blk_pins(core, b, c) >> arg2) & 1;
            break;
        case 1:
            level = (sm_in_pins(core, b, sm, c) >> arg2) & 1;
            break;
        case 2:
            irq_target(b, i, arg2, &tb, &flag);
            if (!blk_linked(core, b, tb)) {
                level = false;
                break;
            }
            level = (core->blk[tb].irq >> flag) & 1;
            if (pol && level) {
                cx->irq_clr[tb] |= 1u << flag;
            }
            break;
        default:
            level = (blk_pins(core, b, c) >>
                     ((EXECCTRL_JMP_PIN(ec) + (arg2 & 3)) & 31)) & 1;
            break;
        }
        done = level == pol;
        break;
    }

    case OP_IN: {
        bool push;

        n = count32(arg2);
        v = mov_source(core, b, sm, arg1 == 0 ? 0 : arg1, c);
        if (arg1 == 4 || arg1 == 5) {
            v = 0;
        }
        push = (sm->shiftctrl & SHIFTCTRL_AUTOPUSH) &&
               MIN(32, sm->isr_count + n) >= push_thresh(sm);
        if (push && rx_full(sm)) {
            blk->fdebug |= FDEBUG_RXSTALL(i);
            done = false;
            break;
        }
        isr_shift(sm, v, n);
        if (push) {
            rx_push(sm, sm->isr);
            sm->isr = 0;
            sm->isr_count = 0;
            cx->fifo = true;
        }
        break;
    }

    case OP_OUT:
        is_out = true;
        n = count32(arg2);
        if ((sm->shiftctrl & SHIFTCTRL_AUTOPULL) &&
            sm->osr_count >= pull_thresh(sm)) {
            /* An empty OSR cannot be refilled and shifted in one cycle. */
            if (!autopull(sm, cx)) {
                blk->fdebug |= FDEBUG_TXSTALL(i);
            } else {
                progress = true;
            }
            done = false;
            break;
        }
        v = osr_shift(sm, n);
        switch (arg1) {
        case 0:
        case 4:
            if ((ec & EXECCTRL_INLINE_OUT_EN) &&
                !((v >> EXECCTRL_OUT_EN_SEL(ec)) & 1)) {
                /* With OUT_STICKY, a disabled write releases the pins. */
                sm->flags &= ~SMF_STICKY;
                break;
            }
            out_write(cx, b, i, sm, arg1 == 4, PINCTRL_OUT_BASE(pinctrl),
                      MIN(PINCTRL_OUT_COUNT(pinctrl), 32), v);
            break;
        case 1:
            sm->x = v;
            break;
        case 2:
            sm->y = v;
            break;
        case 3:
            break;
        case 5:
            new_pc = v & 31;
            break;
        case 6:
            sm->isr = v;
            sm->isr_count = n;
            break;
        default:
            exec = true;
            break;
        }
        autopull(sm, cx);
        break;

    case OP_PUSH_PULL:
        if ((instr & 0x1f) == 0 && !(instr & 0x80)) {
            /* PUSH */
            if ((instr & 0x40) && sm->isr_count < push_thresh(sm)) {
                break;
            }
            if (rx_full(sm)) {
                blk->fdebug |= FDEBUG_RXSTALL(i);
                if (instr & 0x20) {
                    done = false;
                    break;
                }
            } else {
                rx_push(sm, sm->isr);
                cx->fifo = true;
            }
            sm->isr = 0;
            sm->isr_count = 0;
        } else if ((instr & 0x1f) == 0) {
            /* PULL */
            if ((instr & 0x40) && sm->osr_count < pull_thresh(sm)) {
                break;
            }
            /* With autopull, PULL on a full OSR does nothing. */
            if ((sm->shiftctrl & SHIFTCTRL_AUTOPULL) && !sm->osr_count) {
                break;
            }
            if (sm->tx_level) {
                sm->osr = tx_pop(sm);
                cx->fifo = true;
            } else if (instr & 0x20) {
                blk->fdebug |= FDEBUG_TXSTALL(i);
                done = false;
                break;
            } else {
                sm->osr = sm->x;
            }
            sm->osr_count = 0;
        } else if ((instr & 0x74) == 0x10) {
            /* MOV RXFIFO[], ISR and MOV OSR, RXFIFO[] */
            unsigned idx = (instr & 8 ? instr : sm->y) & 3;

            if (instr & 0x80) {
                sm->osr = sm->fifo[RP2350_PIO_FIFO_DEPTH + idx];
                sm->osr_count = 0;
            } else {
                sm->fifo[RP2350_PIO_FIFO_DEPTH + idx] = sm->isr;
            }
        }
        break;

    case OP_MOV: {
        unsigned src = arg2 & 7, op = (arg2 >> 3) & 3;

        v = mov_source(core, b, sm, src, c);
        if (op == 1) {
            v = ~v;
        } else if (op == 2) {
            v = revbit32(v);
        }
        switch (arg1) {
        case 0:
        case 3:
            out_write(cx, b, i, sm, arg1 == 3, PINCTRL_OUT_BASE(pinctrl),
                      MIN(PINCTRL_OUT_COUNT(pinctrl), 32), v);
            break;
        case 1:
            sm->x = v;
            break;
        case 2:
            sm->y = v;
            break;
        case 4:
            exec = true;
            break;
        case 5:
            new_pc = v & 31;
            break;
        case 6:
            sm->isr = v;
            sm->isr_count = 0;
            break;
        default:
            sm->osr = v;
            sm->osr_count = 0;
            break;
        }
        break;
    }

    case OP_IRQ:
        irq_target(b, i, instr & 0x1f, &tb, &flag);
        if (!blk_linked(core, b, tb)) {
            break;
        }
        if (instr & 0x40) {
            cx->irq_clr[tb] |= 1u << flag;
        } else if (!(instr & 0x20)) {
            cx->irq_set[tb] |= 1u << flag;
        } else if (!(sm->flags & SMF_IRQ_WAIT)) {
            cx->irq_set[tb] |= 1u << flag;
            sm->flags |= SMF_IRQ_WAIT;
            progress = true;
            done = false;
        } else {
            done = !((core->blk[tb].irq >> flag) & 1);
        }
        break;

    default:
        switch (arg1) {
        case 0:
        case 4:
            out_write(cx, b, i, sm, arg1 == 4, PINCTRL_SET_BASE(pinctrl),
                      MIN(PINCTRL_SET_COUNT(pinctrl), 5), arg2);
            break;
        case 1:
            sm->x = arg2;
            break;
        case 2:
            sm->y = arg2;
            break;
        default:
            break;
        }
        break;
    }

    if (!done) {
        sm->flags |= SMF_STALLED;
        if (kind == EXEC_FORCED) {
            /* A stalled SMx_INSTR is latched until it completes. */
            sm->latch = instr;
            sm->flags |= SMF_LATCH | SMF_LATCH_FORCED;
        }
    } else {
        sm->flags &= ~(SMF_STALLED | SMF_IRQ_WAIT);
        if (kind != EXEC_MEM) {
            sm->flags &= ~(SMF_LATCH | SMF_LATCH_FORCED);
        }
        if (new_pc >= 0) {
            sm->pc = new_pc;
        } else if (kind == EXEC_MEM) {
            sm->pc = sm->pc == EXECCTRL_WRAP_TOP(ec) ? EXECCTRL_WRAP_BOTTOM(ec)
                                                     : (sm->pc + 1) & 31;
        }
        /*
         * Delays on SMx_INSTR writes and on OUT/MOV EXEC are ignored; the
         * executee's own delay applies.
         */
        sm->delay = kind == EXEC_FORCED || exec ? 0 : delay;
        if (exec) {
            sm->latch = v;
            sm->flags |= SMF_LATCH;
        }
        progress = true;
    }

    /* On cycles without an OUT, an exhausted OSR refills when it can. */
    if (!is_out && autopull(sm, cx)) {
        progress = true;
    }
    return progress || first;
}

/* Note a stalled state machine that nothing has released. */
static void sm_note_stall(RP2350PIOCore *core, RP2350PIOSM *sm, uint64_t c,
                          bool progress)
{
    if ((sm->flags & SMF_STALLED) && !progress && c >= core->in_settle) {
        sm->idle_seq = core->seq;
    } else {
        sm->idle_seq = core->seq - 1;
    }
}

static bool sm_idle(const RP2350PIOCore *core, const RP2350PIOSM *sm)
{
    return (sm->flags & SMF_STALLED) && sm->idle_seq == core->seq;
}

/* The next cycle in which state machine i of block b must run. */
static uint64_t sm_due(RP2350PIOCore *core, int b, int i)
{
    RP2350PIOBlock *blk = &core->blk[b];
    RP2350PIOSM *sm = &blk->sm[i];

    if (sm->flags & SMF_FORCED) {
        return core->cycle;
    }
    if (sm_idle(core, sm)) {
        return NO_CYCLE;
    }
    if (sm->flags & SMF_LATCH_FORCED) {
        return core->cycle;
    }
    if (!(blk->sm_enable & (1u << i))) {
        return NO_CYCLE;
    }
    if (sm->next_en < core->cycle) {
        sm->next_en = div_next(sm, core->cycle);
    }
    return sm->next_en;
}

static uint64_t core_next(RP2350PIOCore *core)
{
    uint64_t next = NO_CYCLE;
    int b, i;

    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        next = MIN(next, core->blk[b].resolve_at);
        for (i = 0; i < RP2350_PIO_SMS; i++) {
            next = MIN(next, sm_due(core, b, i));
        }
    }
    return next;
}

/* Run state machine i of block b in cycle c, if it has work in it. */
static void sm_cycle(RP2350PIOCore *core, int b, int i, uint64_t c,
                     PIOCycle *cx)
{
    RP2350PIOBlock *blk = &core->blk[b];
    RP2350PIOSM *sm = &blk->sm[i];
    bool enabled = (blk->sm_enable & (1u << i)) && sm->next_en == c;
    bool progress;

    if (enabled) {
        sm->next_en = div_next(sm, c + 1);
    }

    if (sm->flags & SMF_FORCED) {
        /* An SMx_INSTR write replaces whatever was latched. */
        sm->flags &= ~(SMF_FORCED | SMF_STALLED | SMF_IRQ_WAIT |
                       SMF_LATCH | SMF_LATCH_FORCED);
        progress = sm_exec(core, b, i, sm->forced, EXEC_FORCED, c, cx);
    } else if (sm_idle(core, sm)) {
        return;
    } else if (sm->flags & SMF_LATCH_FORCED) {
        progress = sm_exec(core, b, i, sm->latch, EXEC_FORCED, c, cx);
    } else if (!enabled) {
        return;
    } else if (sm->delay) {
        sm->delay--;
        autopull(sm, cx);
        return;
    } else if (sm->flags & SMF_LATCH) {
        progress = sm_exec(core, b, i, sm->latch, EXEC_LATCH, c, cx);
    } else {
        progress = sm_exec(core, b, i, blk->imem[sm->pc], EXEC_MEM, c, cx);
    }
    sm_note_stall(core, sm, c, progress);
}

typedef struct PIOChanges {
    bool pins[RP2350_PIO_BLOCKS];
    bool outputs;
} PIOChanges;

/* Apply the end of cycle c: IRQ flags and pin writes. */
static void core_commit(RP2350PIOCore *core, uint64_t c, PIOCycle *cx,
                        PIOChanges *ch)
{
    int b, i;

    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        RP2350PIOBlock *blk = &core->blk[b];
        uint32_t irq = (blk->irq & ~cx->irq_clr[b]) | cx->irq_set[b];
        bool sticky = false;
        uint32_t out, oe;

        if (irq != blk->irq) {
            blk->irq = irq;
            core->seq++;
            ch->outputs = true;
        }

        if (!cx->wvalid[b] && blk->resolve_at != c) {
            continue;
        }
        blk->resolve_at = NO_CYCLE;
        out = blk->pad_out;
        oe = blk->pad_oe;
        for (i = 0; i < RP2350_PIO_SMS; i++) {
            RP2350PIOSM *sm = &blk->sm[i];
            PIOPinWrite *w = cx->w[b][i];
            bool held = (sm->execctrl & EXECCTRL_OUT_STICKY) &&
                        (sm->flags & SMF_STICKY);

            if (cx->wvalid[b] & (1u << (i * CH_COUNT + CH_OUT))) {
                apply_write(&out, &oe, w[CH_OUT].mask, w[CH_OUT].data,
                            w[CH_OUT].dir);
            } else if (held) {
                apply_write(&out, &oe, sm->sticky_mask, sm->sticky_data,
                            sm->flags & SMF_STICKY_DIR);
            }
            if (cx->wvalid[b] & (1u << (i * CH_COUNT + CH_SIDE))) {
                apply_write(&out, &oe, w[CH_SIDE].mask, w[CH_SIDE].data,
                            w[CH_SIDE].dir);
            }
            sticky |= held;
        }
        /*
         * A sticky write overridden this cycle asserts itself again in
         * the next.
         */
        if (sticky && cx->wvalid[b]) {
            blk->resolve_at = c + 1;
        }
        if (out != blk->pad_out || oe != blk->pad_oe) {
            blk->pad_out = out;
            blk->pad_oe = oe;
            core->seq++;
            ch->pins[b] = true;
        }
    }
    if (cx->fifo) {
        ch->outputs = true;
    }
}

/* The block's raw interrupt flags (INTR). */
static uint32_t blk_intr(const RP2350PIOBlock *blk)
{
    uint32_t v = blk->irq << 8;
    int i;

    for (i = 0; i < RP2350_PIO_SMS; i++) {
        const RP2350PIOSM *sm = &blk->sm[i];

        if (sm->rx_level) {
            v |= 1u << i;
        }
        if (!tx_full(sm)) {
            v |= 1u << (4 + i);
        }
    }
    return v;
}

static uint32_t blk_ints(const RP2350PIOBlock *blk, int n)
{
    return ((blk_intr(blk) & blk->inte[n]) | blk->intf[n]) & INTR_MASK;
}

/* The IRQ lines (bit 2b + n) and DREQs (DREQ numbering) of the blocks. */
static void core_lines(const RP2350PIOCore *core, uint32_t *irq,
                       uint32_t *dreq)
{
    int b, i;

    *irq = 0;
    *dreq = 0;
    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        const RP2350PIOBlock *blk = &core->blk[b];

        if (blk->held) {
            continue;
        }
        for (i = 0; i < RP2350_PIO_IRQS_PER_BLOCK; i++) {
            if (blk_ints(blk, i)) {
                *irq |= 1u << (b * RP2350_PIO_IRQS_PER_BLOCK + i);
            }
        }
        for (i = 0; i < RP2350_PIO_SMS; i++) {
            const RP2350PIOSM *sm = &blk->sm[i];

            if (!tx_full(sm)) {
                *dreq |= 1u << (b * 8 + i);
            }
            if (sm->rx_level) {
                *dreq |= 1u << (b * 8 + 4 + i);
            }
        }
    }
}

/* The pins a block drives, numbered by GPIO. */
static uint64_t blk_gpio(const RP2350PIOBlock *blk, uint32_t v)
{
    return ((uint64_t)v << blk->gpiobase) & GPIO_MASK;
}

/* Drive block b's pin outputs that differ from what was last driven. */
static void drive_pins(RP2350PIOState *s, int b)
{
    RP2350PIOBlock *blk = &s->core.blk[b];
    uint64_t out = blk_gpio(blk, blk->pad_out);
    uint64_t oe = blk_gpio(blk, blk->pad_oe);
    uint64_t d_out = out ^ s->drv_out[b], d_oe = oe ^ s->drv_oe[b];

    /*
     * Record the levels before driving them: driving a pin feeds back
     * into this device through its inputs.
     */
    s->drv_out[b] = out;
    s->drv_oe[b] = oe;
    while (d_oe) {
        int p = ctz64(d_oe);

        d_oe &= d_oe - 1;
        qemu_set_irq(s->oe[b * RP2350_PIO_GPIOS + p], (oe >> p) & 1);
    }
    while (d_out) {
        int p = ctz64(d_out);

        d_out &= d_out - 1;
        qemu_set_irq(s->out[b * RP2350_PIO_GPIOS + p], (out >> p) & 1);
        if (p >= RP2350_PIO_HSTX_FIRST &&
            p < RP2350_PIO_HSTX_FIRST + RP2350_PIO_HSTX_BITS) {
            qemu_set_irq(s->hstx[b * RP2350_PIO_HSTX_BITS + p -
                                 RP2350_PIO_HSTX_FIRST], (out >> p) & 1);
        }
    }
}

static void drive_lines(RP2350PIOState *s)
{
    uint32_t irq, dreq, d;

    core_lines(&s->core, &irq, &dreq);
    d = irq ^ s->drv_irq;
    s->drv_irq = irq;
    while (d) {
        int n = ctz32(d);

        d &= d - 1;
        qemu_set_irq(s->irq[n], (irq >> n) & 1);
    }
    d = dreq ^ s->drv_dreq;
    s->drv_dreq = dreq;
    while (d) {
        int n = ctz32(d);

        d &= d - 1;
        qemu_set_irq(s->dreq[n], (dreq >> n) & 1);
    }
}

static void drive_all(RP2350PIOState *s)
{
    int b;

    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        drive_pins(s, b);
    }
    drive_lines(s);
}

/*
 * The GPIOs on which a block's outputs are seen outside it: those a pin
 * routes to the block, and those HSTX's coupled mode can take.
 */
static uint64_t blk_routed(RP2350PIOState *s, int b)
{
    uint64_t hstx = MAKE_64BIT_MASK(RP2350_PIO_HSTX_FIRST,
                                    RP2350_PIO_HSTX_BITS);

    if (!s->gpio) {
        return GPIO_MASK;
    }
    return rp2350_gpio_port_selected(s->gpio, RP2350_GPIO_PORT_PIO0 + b) |
           hstx;
}

/*
 * Run `core` up to (not including) cycle `target`. The live state drives
 * pin changes as they happen, so that they come back through the pads
 * stamped with the right cycle. A run ahead (`spec`) drives nothing and
 * stops after the first cycle that changes an output visible outside the
 * blocks; it returns whether it did.
 */
static bool core_run(RP2350PIOState *s, RP2350PIOCore *core, uint64_t target,
                     bool spec)
{
    uint64_t routed[RP2350_PIO_BLOCKS];
    int b, i;

    if (spec) {
        for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
            routed[b] = blk_routed(s, b);
        }
    }

    while (core->cycle < target) {
        uint64_t c = core_next(core);
        PIOChanges ch = {};
        PIOCycle cx;

        if (c >= target) {
            core->cycle = target;
            break;
        }
        memset(cx.irq_set, 0, sizeof(cx.irq_set));
        memset(cx.irq_clr, 0, sizeof(cx.irq_clr));
        memset(cx.wvalid, 0, sizeof(cx.wvalid));
        cx.fifo = false;
        for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
            for (i = 0; i < RP2350_PIO_SMS; i++) {
                sm_cycle(core, b, i, c, &cx);
            }
        }
        core_commit(core, c, &cx, &ch);
        core->cycle = c + 1;

        if (!spec) {
            for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
                if (ch.pins[b]) {
                    drive_pins(s, b);
                }
            }
            continue;
        }
        for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
            const RP2350PIOBlock *blk = &core->blk[b];

            if (ch.pins[b] &&
                (((blk_gpio(blk, blk->pad_out) ^ s->drv_out[b]) |
                  (blk_gpio(blk, blk->pad_oe) ^ s->drv_oe[b])) & routed[b])) {
                return true;
            }
        }
        if (ch.outputs) {
            uint32_t irq, dreq;

            core_lines(core, &irq, &dreq);
            if (irq != s->drv_irq || dreq != s->drv_dreq) {
                return true;
            }
        }
    }
    return false;
}

/* Time */

static uint64_t pio_cycle_at(RP2350PIOState *s, int64_t ns)
{
    if (ns <= s->base_ns || !clock_is_enabled(s->clk)) {
        return s->base_cycle;
    }
    return s->base_cycle + clock_ns_to_ticks(s->clk, ns - s->base_ns);
}

/* The first virtual time at which clk_sys has reached `cycle`. */
static int64_t pio_ns_at(RP2350PIOState *s, uint64_t cycle)
{
    uint64_t d, ns;

    if (cycle <= s->base_cycle) {
        return s->base_ns;
    }
    d = cycle - s->base_cycle;
    ns = clock_ticks_to_ns(s->clk, d);
    while (clock_ns_to_ticks(s->clk, ns) < d) {
        ns++;
    }
    return s->base_ns + MIN(ns, (uint64_t)INT64_MAX / 2);
}

static uint64_t pio_now(RP2350PIOState *s)
{
    return pio_cycle_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/*
 * Bring the blocks up to the current virtual time: commit the run ahead
 * if it has come due, or else discard it and run the blocks themselves.
 */
static void pio_sync(RP2350PIOState *s)
{
    uint64_t now = pio_now(s);

    s->busy = true;
    if (s->spec_valid && s->spec.cycle <= now) {
        s->core = s->spec;
        drive_all(s);
    }
    s->spec_valid = false;
    core_run(s, &s->core, now, false);
    drive_all(s);
    s->busy = false;
}

/* Run ahead to find when the blocks next change an output. */
static void pio_schedule(RP2350PIOState *s)
{
    bool event;

    if (!clock_is_enabled(s->clk)) {
        s->spec_valid = false;
        timer_del(s->timer);
        return;
    }
    s->spec = s->core;
    event = core_run(s, &s->spec, s->core.cycle + s->horizon, true);
    if (!event && core_next(&s->spec) == NO_CYCLE) {
        /* Nothing will happen until something disturbs the blocks. */
        s->spec_valid = false;
        timer_del(s->timer);
        return;
    }
    s->spec_valid = true;
    timer_mod(s->timer, pio_ns_at(s, s->spec.cycle));
}

static void pio_timer_cb(void *opaque)
{
    RP2350PIOState *s = opaque;

    s->horizon = MIN(s->horizon * 2, HORIZON_MAX);
    pio_sync(s);
    pio_schedule(s);
}

/* Something outside is about to change the blocks: catch them up. */
static void pio_disturb(RP2350PIOState *s)
{
    s->horizon = HORIZON_MIN;
    pio_sync(s);
}

/* The blocks were changed from outside: drive and re-time the outputs. */
static void pio_settle(RP2350PIOState *s)
{
    s->busy = true;
    drive_all(s);
    s->busy = false;
    pio_schedule(s);
}

/* Run the cycle in which an SMx_INSTR write executes. */
static void pio_run_cycle(RP2350PIOState *s)
{
    s->busy = true;
    core_run(s, &s->core, s->core.cycle + 1, false);
    s->busy = false;
}

/* [spec:nuos:req:emu.pio] */
static void pio_in(void *opaque, int n, int level)
{
    RP2350PIOState *s = opaque;
    uint64_t vec = deposit64(s->in_level, n, 1, level != 0);

    if (vec == s->in_level) {
        return;
    }
    s->in_level = vec;
    if (s->busy) {
        core_input(&s->core, vec, s->core.cycle);
        return;
    }
    pio_disturb(s);
    core_input(&s->core, vec, s->core.cycle);
    pio_schedule(s);
}

/* State machine control */

static void sm_restart(RP2350PIOSM *sm)
{
    sm->isr_count = 0;
    sm->osr_count = 32;
    sm->isr = 0;
    sm->delay = 0;
    sm->flags &= ~(SMF_IRQ_WAIT | SMF_LATCH | SMF_LATCH_FORCED |
                   SMF_STALLED | SMF_STICKY | SMF_STICKY_DIR);
}

/* Apply CTRL's per-state-machine operations to block b. */
static void blk_ctrl(RP2350PIOCore *core, int b, uint32_t enable,
                     uint32_t restart, uint32_t clkdiv_restart)
{
    RP2350PIOBlock *blk = &core->blk[b];
    int i;

    for (i = 0; i < RP2350_PIO_SMS; i++) {
        RP2350PIOSM *sm = &blk->sm[i];

        if (restart & (1u << i)) {
            sm_restart(sm);
        }
        if (clkdiv_restart & (1u << i)) {
            div_restart(sm, core->cycle);
        }
        if ((enable & ~blk->sm_enable & (1u << i)) &&
            !(clkdiv_restart & (1u << i))) {
            sm->next_en = div_next(sm, core->cycle);
        }
    }
    blk->sm_enable = enable;
}

static void pio_write_ctrl(RP2350PIOCore *core, int b, uint32_t v)
{
    static const int neighbour[2] = { -1, 1 };
    int k;

    for (k = 0; k < 2; k++) {
        int nb = (b + RP2350_PIO_BLOCKS + neighbour[k]) % RP2350_PIO_BLOCKS;
        uint32_t m = k ? CTRL_NEXT_MASK(v) : CTRL_PREV_MASK(v);
        uint32_t en = core->blk[nb].sm_enable;

        if (!m || !blk_linked(core, b, nb)) {
            continue;
        }
        if (v & CTRL_NEXTPREV_DISABLE) {
            en &= ~m;
        } else if (v & CTRL_NEXTPREV_ENABLE) {
            en |= m;
        }
        blk_ctrl(core, nb, en, 0, v & CTRL_NEXTPREV_CLKDIV ? m : 0);
    }
    blk_ctrl(core, b, CTRL_SM_ENABLE(v), CTRL_SM_RESTART(v),
             CTRL_CLKDIV_RESTART(v));
    core->seq++;
}

static void pio_write_shiftctrl(RP2350PIOSM *sm, uint32_t v)
{
    v &= SHIFTCTRL_RW;
    /* Random-access RX modes take the FIFO storage from the joins. */
    if (v & SHIFTCTRL_PUTGET) {
        v &= ~(SHIFTCTRL_FJOIN_RX | SHIFTCTRL_FJOIN_TX);
    }
    if ((v ^ sm->shiftctrl) & SHIFTCTRL_FIFO_MODE) {
        fifo_flush(sm);
    }
    sm->shiftctrl = v;
}

static void pio_check_instr(int b, const char *where, uint16_t instr)
{
    unsigned arg1 = INSTR_ARG1(instr), arg2 = INSTR_ARG2(instr);
    bool bad = false;

    switch (INSTR_OP(instr)) {
    case OP_WAIT:
        bad = (arg1 & 3) == 3 && arg2 > 3;
        break;
    case OP_IN:
        bad = arg1 == 4 || arg1 == 5;
        break;
    case OP_PUSH_PULL:
        if (instr & 0x1f) {
            bad = (instr & 0x74) != 0x10 || (!(instr & 8) && (instr & 3));
        }
        break;
    case OP_MOV:
        bad = ((arg2 >> 3) & 3) == 3 || (arg2 & 7) == 4;
        break;
    case OP_IRQ:
        bad = instr & 0x80;
        break;
    case OP_SET:
        bad = arg1 != 0 && arg1 != 1 && arg1 != 2 && arg1 != 4;
        break;
    default:
        break;
    }
    if (bad) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pio%d: %s: reserved "
                      "instruction encoding 0x%04x\n", b, where, instr);
    }
}

/* Registers */

/* A register's value, without the side effects of reading it. */
static uint32_t pio_peek(RP2350PIOState *s, int b, hwaddr reg)
{
    RP2350PIOBlock *blk = &s->core.blk[b];
    uint32_t v = 0;
    int i;

    switch (reg) {
    case A_CTRL:
        return blk->sm_enable;
    case A_FSTAT:
        for (i = 0; i < RP2350_PIO_SMS; i++) {
            RP2350PIOSM *sm = &blk->sm[i];

            v |= (rx_full(sm) << i) | ((sm->rx_level == 0) << (8 + i)) |
                 (tx_full(sm) << (16 + i)) |
                 ((sm->tx_level == 0) << (24 + i));
        }
        return v;
    case A_FDEBUG:
        return blk->fdebug;
    case A_FLEVEL:
        for (i = 0; i < RP2350_PIO_SMS; i++) {
            v |= ((blk->sm[i].tx_level & 0xf) << (8 * i)) |
                 ((blk->sm[i].rx_level & 0xf) << (8 * i + 4));
        }
        return v;
    case A_IRQ:
        return blk->irq;
    case A_INPUT_SYNC_BYPASS:
        return blk->sync_bypass;
    case A_DBG_PADOUT:
        return blk->pad_out;
    case A_DBG_PADOE:
        return blk->pad_oe;
    case A_DBG_CFGINFO:
        return DBG_CFGINFO_VALUE;
    case A_GPIOBASE:
        return blk->gpiobase;
    case A_INTR:
        return blk_intr(blk) & INTR_MASK;
    case A_IRQ0_INTE ... A_IRQ1_INTS:
        i = (reg - A_IRQ0_INTE) / 12;
        switch ((reg - A_IRQ0_INTE) % 12) {
        case 0:
            return blk->inte[i];
        case 4:
            return blk->intf[i];
        default:
            return blk_ints(blk, i);
        }
    case A_SM0_CLKDIV ... A_RXF0_PUTGET0 - 4: {
        RP2350PIOSM *sm = &blk->sm[(reg - A_SM0_CLKDIV) / SM_STRIDE];

        switch ((reg - A_SM0_CLKDIV) % SM_STRIDE) {
        case SM_CLKDIV:
            return sm->clkdiv;
        case SM_EXECCTRL:
            return sm->execctrl |
                   (sm->flags & SMF_LATCH_FORCED ? EXECCTRL_EXEC_STALLED : 0);
        case SM_SHIFTCTRL:
            return sm->shiftctrl;
        case SM_ADDR:
            return sm->pc;
        case SM_INSTR:
            return blk->imem[sm->pc];
        default:
            return sm->pinctrl;
        }
    }
    case A_RXF0_PUTGET0 ... A_GPIOBASE - 4: {
        RP2350PIOSM *sm = &blk->sm[(reg - A_RXF0_PUTGET0) / 16];
        uint32_t mode = sm->shiftctrl & SHIFTCTRL_PUTGET;

        if (mode == SHIFTCTRL_FJOIN_RX_PUT || mode == SHIFTCTRL_FJOIN_RX_GET) {
            return sm->fifo[RP2350_PIO_FIFO_DEPTH +
                            ((reg - A_RXF0_PUTGET0) / 4) % 4];
        }
        return 0;
    }
    default:
        return 0;
    }
}

static bool pio_reg_valid(hwaddr reg)
{
    return reg <= A_IRQ1_INTS;
}

/* [spec:nuos:req:emu.pio] */
static uint32_t pio_read_reg(RP2350PIOState *s, int b, hwaddr reg)
{
    RP2350PIOBlock *blk = &s->core.blk[b];

    if (reg >= A_RXF0 && reg < A_IRQ) {
        int i = (reg - A_RXF0) / 4;
        RP2350PIOSM *sm = &blk->sm[i];

        if (!sm->rx_level) {
            blk->fdebug |= FDEBUG_RXUNDER(i);
            return rx_cap(sm) ? rx_peek(sm) : 0;
        }
        s->core.seq++;
        return rx_pop(sm);
    }
    if (reg >= A_RXF0_PUTGET0 && reg < A_GPIOBASE) {
        RP2350PIOSM *sm = &blk->sm[(reg - A_RXF0_PUTGET0) / 16];
        uint32_t mode = sm->shiftctrl & SHIFTCTRL_PUTGET;

        if (mode != SHIFTCTRL_FJOIN_RX_PUT && mode != SHIFTCTRL_FJOIN_RX_GET) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pio%d: RXF%d_PUTGET read "
                          "without FJOIN_RX_PUT xor FJOIN_RX_GET\n", b,
                          (int)(reg - A_RXF0_PUTGET0) / 16);
        }
    }
    if (!pio_reg_valid(reg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pio%d: read of bad offset "
                      "0x%" HWADDR_PRIx "\n", b, reg);
        return 0;
    }
    return pio_peek(s, b, reg);
}

/*
 * Write `value` (already combined with the register by an atomic alias)
 * to a register. `raw` and `alias` are the written data and alias, which
 * write-1-to-clear registers act on: a normal or SET alias write clears
 * the bits written as 1; XOR and CLR alias writes leave the flags alone.
 */
/* [spec:nuos:req:emu.pio] */
static void pio_write_reg(RP2350PIOState *s, int b, hwaddr reg,
                          uint32_t value, uint32_t raw, int alias)
{
    RP2350PIOCore *core = &s->core;
    RP2350PIOBlock *blk = &core->blk[b];
    bool w1c = alias == 0 || alias == 2;
    int i;

    switch (reg) {
    case A_CTRL:
        pio_write_ctrl(core, b, value);
        return;
    case A_FDEBUG:
        if (w1c) {
            blk->fdebug &= ~raw;
        }
        return;
    case A_TXF0 ... A_RXF0 - 4: {
        RP2350PIOSM *sm = &blk->sm[(reg - A_TXF0) / 4];

        i = (reg - A_TXF0) / 4;
        if (tx_full(sm)) {
            blk->fdebug |= FDEBUG_TXOVER(i);
        } else {
            tx_push(sm, value);
            core->seq++;
        }
        return;
    }
    case A_IRQ:
        if (w1c) {
            blk->irq &= ~raw;
            core->seq++;
        }
        return;
    case A_IRQ_FORCE:
        blk->irq |= value & 0xff;
        core->seq++;
        return;
    case A_INPUT_SYNC_BYPASS:
        blk->sync_bypass = value;
        core->seq++;
        return;
    case A_INSTR_MEM0 ... A_SM0_CLKDIV - 4:
        blk->imem[(reg - A_INSTR_MEM0) / 4] = value;
        pio_check_instr(b, "INSTR_MEM", value);
        core->seq++;
        return;
    case A_SM0_CLKDIV ... A_RXF0_PUTGET0 - 4: {
        RP2350PIOSM *sm;

        i = (reg - A_SM0_CLKDIV) / SM_STRIDE;
        sm = &blk->sm[i];
        switch ((reg - A_SM0_CLKDIV) % SM_STRIDE) {
        case SM_CLKDIV:
            div_advance(sm, core->cycle);
            sm->clkdiv = value & CLKDIV_MASK;
            if (!(sm->clkdiv >> 16) && (sm->clkdiv & 0xff00)) {
                qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pio%d: SM%d_CLKDIV "
                              "FRAC set with INT 0\n", b, i);
            }
            sm->div_acc = MIN(sm->div_acc, sm_div(sm) - 1);
            sm->next_en = div_next(sm, core->cycle);
            break;
        case SM_EXECCTRL:
            sm->execctrl = value & EXECCTRL_RW;
            break;
        case SM_SHIFTCTRL:
            pio_write_shiftctrl(sm, value);
            break;
        case SM_ADDR:
            return;
        case SM_INSTR:
            sm->forced = value;
            sm->flags |= SMF_FORCED;
            pio_check_instr(b, "SM_INSTR", value);
            core->seq++;
            pio_run_cycle(s);
            return;
        default:
            sm->pinctrl = value;
            break;
        }
        core->seq++;
        return;
    }
    case A_RXF0_PUTGET0 ... A_GPIOBASE - 4: {
        RP2350PIOSM *sm;
        uint32_t mode;

        i = (reg - A_RXF0_PUTGET0) / 16;
        sm = &blk->sm[i];
        mode = sm->shiftctrl & SHIFTCTRL_PUTGET;
        if (mode != SHIFTCTRL_FJOIN_RX_PUT && mode != SHIFTCTRL_FJOIN_RX_GET) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pio%d: RXF%d_PUTGET write "
                          "without FJOIN_RX_PUT xor FJOIN_RX_GET\n", b, i);
            return;
        }
        sm->fifo[RP2350_PIO_FIFO_DEPTH + ((reg - A_RXF0_PUTGET0) / 4) % 4] =
            value;
        core->seq++;
        return;
    }
    case A_GPIOBASE:
        blk->gpiobase = value & GPIOBASE_MASK;
        core->seq++;
        return;
    case A_IRQ0_INTE:
    case A_IRQ0_INTE + 12:
        blk->inte[reg != A_IRQ0_INTE] = value & INTR_MASK;
        return;
    case A_IRQ0_INTE + 4:
    case A_IRQ0_INTE + 16:
        blk->intf[reg != A_IRQ0_INTE + 4] = value & INTR_MASK;
        return;
    case A_FSTAT:
    case A_FLEVEL:
    case A_RXF0 ... A_IRQ - 4:
    case A_DBG_PADOUT:
    case A_DBG_PADOE:
    case A_DBG_CFGINFO:
    case A_INTR:
    case A_IRQ0_INTE + 8:
    case A_IRQ1_INTS:
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pio%d: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", b, reg);
        return;
    }
}

static uint64_t rp2350_pio_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350PIOWindow *w = opaque;
    RP2350PIOState *s = w->s;
    uint32_t v;

    pio_disturb(s);
    v = pio_read_reg(s, w->block, rp2350_atomic_reg(addr) & ~3);
    pio_settle(s);
    return extract32(v, (addr & 3) * 8, size * 8);
}

/*
 * A narrow write is replicated across the 32-bit bus and writes the whole
 * register: a byte written to TXF fills all four bytes of the FIFO entry.
 */
static void rp2350_pio_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    RP2350PIOWindow *w = opaque;
    RP2350PIOState *s = w->s;
    hwaddr reg = rp2350_atomic_reg(addr) & ~3;
    int alias = addr / RP2350_ATOMIC_ALIAS_SIZE;
    uint32_t raw = value, v;

    if (size == 1) {
        raw = (value & 0xff) * 0x01010101u;
    } else if (size == 2) {
        raw = (value & 0xffff) * 0x00010001u;
    }
    pio_disturb(s);
    v = alias ? rp2350_atomic_apply(addr, pio_peek(s, w->block, reg), raw)
              : raw;
    pio_write_reg(s, w->block, reg, v, raw, alias);
    pio_settle(s);
}

static const MemoryRegionOps rp2350_pio_ops = {
    .read = rp2350_pio_read,
    .write = rp2350_pio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * A change of clk_sys frequency: run the blocks to the change at the old
 * rate, then count from there at the new one.
 */
static void rp2350_pio_clk_update(void *opaque, ClockEvent event)
{
    RP2350PIOState *s = opaque;

    if (event == ClockPreUpdate) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t cycle = pio_cycle_at(s, now);

        pio_disturb(s);
        s->base_ns = now;
        s->base_cycle = cycle;
    } else {
        pio_schedule(s);
    }
}

static void blk_reset(RP2350PIOCore *core, int b)
{
    RP2350PIOBlock *blk = &core->blk[b];
    bool ns = blk->ns, held = blk->held;
    int i;

    memset(blk, 0, sizeof(*blk));
    blk->ns = ns;
    blk->held = held;
    blk->resolve_at = NO_CYCLE;
    for (i = 0; i < RP2350_PIO_SMS; i++) {
        RP2350PIOSM *sm = &blk->sm[i];

        sm->clkdiv = CLKDIV_RESET;
        sm->execctrl = EXECCTRL_RESET;
        sm->shiftctrl = SHIFTCTRL_RESET;
        sm->pinctrl = PINCTRL_RESET;
        sm->osr_count = 32;
        div_restart(sm, core->cycle);
    }
    core->seq++;
}

/* [spec:nuos:req:emu.pio] */
void rp2350_pio_hold_block(RP2350PIOState *s, int n, bool hold)
{
    pio_disturb(s);
    if (hold) {
        blk_reset(&s->core, n);
    }
    s->core.blk[n].held = hold;
    s->core.seq++;
    pio_settle(s);
}

/* [spec:nuos:req:emu.pio] */
void rp2350_pio_set_security(RP2350PIOState *s, uint32_t ns_blocks,
                             uint64_t gpio_nsmask)
{
    int b;

    pio_disturb(s);
    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        s->core.blk[b].ns = (ns_blocks >> b) & 1;
    }
    s->core.gpio_nsmask = gpio_nsmask;
    s->core.seq++;
    pio_settle(s);
}

static void rp2350_pio_reset_hold(Object *obj, ResetType type)
{
    RP2350PIOState *s = RP2350_PIO(obj);
    RP2350PIOCore *core = &s->core;
    uint32_t irq, dreq;
    int b;

    timer_del(s->timer);
    s->spec_valid = false;
    s->horizon = HORIZON_MIN;
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_cycle = 0;

    core->cycle = 0;
    /* The pin levels are outside the blocks; they are not reset. */
    core->in_val[0] = s->in_level;
    core->in_cycle[0] = 0;
    core->in_n = 1;
    core->in_settle = 0;
    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        blk_reset(core, b);
    }

    /* Drive every line afresh: mark each as last driven to the opposite. */
    core_lines(core, &irq, &dreq);
    s->drv_irq = ~irq & MAKE_64BIT_MASK(0, RP2350_PIO_IRQS);
    s->drv_dreq = ~dreq & MAKE_64BIT_MASK(0, RP2350_PIO_DREQS);
    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        s->drv_out[b] = GPIO_MASK;
        s->drv_oe[b] = GPIO_MASK;
    }
    pio_settle(s);
}

static void rp2350_pio_init(Object *obj)
{
    RP2350PIOState *s = RP2350_PIO(obj);
    DeviceState *dev = DEVICE(obj);
    int b;

    for (b = 0; b < RP2350_PIO_BLOCKS; b++) {
        g_autofree char *name = g_strdup_printf("rp2350-pio%d", b);

        s->win[b].s = s;
        s->win[b].block = b;
        memory_region_init_io(&s->win[b].iomem, obj, &rp2350_pio_ops,
                              &s->win[b], name, RP2350_ATOMIC_REGION_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->win[b].iomem);
    }
    for (b = 0; b < RP2350_PIO_IRQS; b++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[b]);
    }
    qdev_init_gpio_out_named(dev, s->out, RP2350_PIO_OUT,
                             RP2350_PIO_BLOCKS * RP2350_PIO_GPIOS);
    qdev_init_gpio_out_named(dev, s->oe, RP2350_PIO_OE,
                             RP2350_PIO_BLOCKS * RP2350_PIO_GPIOS);
    qdev_init_gpio_out_named(dev, s->dreq, RP2350_PIO_DREQ, RP2350_PIO_DREQS);
    qdev_init_gpio_out_named(dev, s->hstx, RP2350_PIO_HSTX,
                             RP2350_PIO_BLOCKS * RP2350_PIO_HSTX_BITS);
    qdev_init_gpio_in_named(dev, pio_in, RP2350_PIO_IN, RP2350_PIO_GPIOS);
    s->clk = qdev_init_clock_in(dev, "clk", rp2350_pio_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
}

static void rp2350_pio_realize(DeviceState *dev, Error **errp)
{
    RP2350PIOState *s = RP2350_PIO(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pio_timer_cb, s);
}

static const Property rp2350_pio_properties[] = {
    DEFINE_PROP_LINK("gpio", RP2350PIOState, gpio, TYPE_RP2350_GPIO,
                     RP2350GPIOState *),
};

static const VMStateDescription vmstate_rp2350_pio_sm = {
    .name = TYPE_RP2350_PIO "-sm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(clkdiv, RP2350PIOSM),
        VMSTATE_UINT32(execctrl, RP2350PIOSM),
        VMSTATE_UINT32(shiftctrl, RP2350PIOSM),
        VMSTATE_UINT32(pinctrl, RP2350PIOSM),
        VMSTATE_UINT32(x, RP2350PIOSM),
        VMSTATE_UINT32(y, RP2350PIOSM),
        VMSTATE_UINT32(isr, RP2350PIOSM),
        VMSTATE_UINT32(osr, RP2350PIOSM),
        VMSTATE_UINT8(pc, RP2350PIOSM),
        VMSTATE_UINT8(isr_count, RP2350PIOSM),
        VMSTATE_UINT8(osr_count, RP2350PIOSM),
        VMSTATE_UINT8(delay, RP2350PIOSM),
        VMSTATE_UINT16(latch, RP2350PIOSM),
        VMSTATE_UINT16(forced, RP2350PIOSM),
        VMSTATE_UINT8(flags, RP2350PIOSM),
        VMSTATE_UINT32_ARRAY(fifo, RP2350PIOSM, 2 * RP2350_PIO_FIFO_DEPTH),
        VMSTATE_UINT8(tx_head, RP2350PIOSM),
        VMSTATE_UINT8(tx_level, RP2350PIOSM),
        VMSTATE_UINT8(rx_head, RP2350PIOSM),
        VMSTATE_UINT8(rx_level, RP2350PIOSM),
        VMSTATE_UINT32(div_acc, RP2350PIOSM),
        VMSTATE_UINT64(div_cycle, RP2350PIOSM),
        VMSTATE_UINT64(next_en, RP2350PIOSM),
        VMSTATE_UINT32(sticky_mask, RP2350PIOSM),
        VMSTATE_UINT32(sticky_data, RP2350PIOSM),
        VMSTATE_UINT64(idle_seq, RP2350PIOSM),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_pio_block = {
    .name = TYPE_RP2350_PIO "-block",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(imem, RP2350PIOBlock, RP2350_PIO_IMEM),
        VMSTATE_STRUCT_ARRAY(sm, RP2350PIOBlock, RP2350_PIO_SMS, 1,
                             vmstate_rp2350_pio_sm, RP2350PIOSM),
        VMSTATE_UINT32(sm_enable, RP2350PIOBlock),
        VMSTATE_UINT32(fdebug, RP2350PIOBlock),
        VMSTATE_UINT32(irq, RP2350PIOBlock),
        VMSTATE_UINT32(sync_bypass, RP2350PIOBlock),
        VMSTATE_UINT32(gpiobase, RP2350PIOBlock),
        VMSTATE_UINT32_ARRAY(inte, RP2350PIOBlock, RP2350_PIO_IRQS_PER_BLOCK),
        VMSTATE_UINT32_ARRAY(intf, RP2350PIOBlock, RP2350_PIO_IRQS_PER_BLOCK),
        VMSTATE_UINT32(pad_out, RP2350PIOBlock),
        VMSTATE_UINT32(pad_oe, RP2350PIOBlock),
        VMSTATE_UINT64(resolve_at, RP2350PIOBlock),
        VMSTATE_BOOL(ns, RP2350PIOBlock),
        VMSTATE_BOOL(held, RP2350PIOBlock),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_pio_core = {
    .name = TYPE_RP2350_PIO "-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(blk, RP2350PIOCore, RP2350_PIO_BLOCKS, 1,
                             vmstate_rp2350_pio_block, RP2350PIOBlock),
        VMSTATE_UINT64(cycle, RP2350PIOCore),
        VMSTATE_UINT64(seq, RP2350PIOCore),
        VMSTATE_UINT64_ARRAY(in_val, RP2350PIOCore, RP2350_PIO_IN_HIST),
        VMSTATE_UINT64_ARRAY(in_cycle, RP2350PIOCore, RP2350_PIO_IN_HIST),
        VMSTATE_UINT32(in_n, RP2350PIOCore),
        VMSTATE_UINT64(in_settle, RP2350PIOCore),
        VMSTATE_UINT64(gpio_nsmask, RP2350PIOCore),
        VMSTATE_END_OF_LIST()
    },
};

static int rp2350_pio_post_load(void *opaque, int version_id)
{
    RP2350PIOState *s = opaque;

    s->spec_valid = false;
    s->busy = false;
    return 0;
}

static const VMStateDescription vmstate_rp2350_pio = {
    .name = TYPE_RP2350_PIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rp2350_pio_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, RP2350PIOState),
        VMSTATE_TIMER_PTR(timer, RP2350PIOState),
        VMSTATE_STRUCT(core, RP2350PIOState, 1, vmstate_rp2350_pio_core,
                       RP2350PIOCore),
        VMSTATE_UINT64(horizon, RP2350PIOState),
        VMSTATE_INT64(base_ns, RP2350PIOState),
        VMSTATE_UINT64(base_cycle, RP2350PIOState),
        VMSTATE_UINT64(in_level, RP2350PIOState),
        VMSTATE_UINT64_ARRAY(drv_out, RP2350PIOState, RP2350_PIO_BLOCKS),
        VMSTATE_UINT64_ARRAY(drv_oe, RP2350PIOState, RP2350_PIO_BLOCKS),
        VMSTATE_UINT32(drv_irq, RP2350PIOState),
        VMSTATE_UINT32(drv_dreq, RP2350PIOState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_pio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_pio_realize;
    rc->phases.hold = rp2350_pio_reset_hold;
    dc->vmsd = &vmstate_rp2350_pio;
    device_class_set_props(dc, rp2350_pio_properties);
}

/* [spec:nuos:req:emu.pio] */
static const TypeInfo rp2350_pio_info = {
    .name          = TYPE_RP2350_PIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350PIOState),
    .instance_init = rp2350_pio_init,
    .class_init    = rp2350_pio_class_init,
};

static void rp2350_pio_register_types(void)
{
    type_register_static(&rp2350_pio_info);
}

type_init(rp2350_pio_register_types)
