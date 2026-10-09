/*
 * RP2350 high-speed serial transmit (HSTX)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "HSTX", and the pico-sdk hstx_ctrl.h and
 * hstx_fifo.h register descriptions.
 *
 * An 8-entry FIFO, written from clk_sys, feeds the clk_hstx domain. Each
 * clk_hstx cycle the output shift register, if it has shifts left,
 * presents its bits through the crossbar (two half-cycle values per
 * output, SEL_P then SEL_N, or the clock generator), right-rotates by
 * SHIFT and counts one shift; when it has none left it refills from the
 * FIFO, or with CSR.EXPAND_EN from the command expander, in time to show
 * the new word in the next cycle. The expander pops at most one FIFO word
 * a cycle, and cannot output in a cycle in which it pops a command; it
 * pops a data word in the cycle its output is needed, or ahead of time
 * when its expansion shift register is empty, and fetches the next
 * command as soon as the current one has output its last word.
 *
 * The block is not ticked. It holds its state as of a clk_hstx cycle and
 * is run forward to the current cycle whenever it is observed or written,
 * at the cycle in which a pop makes room in a full FIFO (raising
 * DREQ_HSTX), and every pin-refresh-ns while it is shifting, to drive its
 * GPIO outputs at that coarse resolution. Cycles in which nothing but the
 * output shift register changes are run in bulk, and an infinite
 * RAW_REPEAT or TMDS_REPEAT command, whose output is eventually periodic,
 * is run forward by whole periods.
 *
 * Choices where the datasheet is silent or unclear:
 *
 *  - The clock generator counts half-cycles from CSR.CLKPHASE; in each
 *    half-cycle the clock is low if the count, modulo 2 * CLKDIV, is
 *    below CLKDIV, and high otherwise. This follows the CSR.CLKPHASE field
 *    description (CLKDIV 2, CLKPHASE 1: first rising edge half a cycle
 *    after the first data, first falling edge one and a half cycles
 *    after). The single-data-rate examples in the clock generator section
 *    disagree with it by half a cycle for CLKDIV 1.
 *  - Clearing CSR.EN also clears the TMDS encoders' running disparity and
 *    the command expander, which next expects a command.
 *  - Reserved command expander opcodes (4-14) are logged and act as NOP.
 *  - The outputs always drive their output enables.
 *  - A narrow write to the FIFO pushes one word: the bus replicates the
 *    narrow value across the word.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/qapi-builtin-visit.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_hstx.h"
#include "migration/vmstate.h"

#define A_CSR                   0x00
#define A_BIT0                  0x04
#define A_BIT7                  0x20
#define A_EXPAND_SHIFT          0x24
#define A_EXPAND_TMDS           0x28

#define A_STAT                  0x00
#define A_FIFO                  0x04

#define CSR_EN                  (1u << 0)
#define CSR_EXPAND_EN           (1u << 1)
#define CSR_COUPLED_MODE        (1u << 4)
#define CSR_MASK                0xff1f1f73u
#define CSR_RESET               0x10050600u

#define BITN_CLK                (1u << 17)
#define BITN_INV                (1u << 16)
#define BITN_MASK               0x00031f1fu

#define EXPAND_SHIFT_MASK       0x1f1f1f1fu
#define EXPAND_SHIFT_RESET      0x01000100u
#define EXPAND_TMDS_MASK        0x00ffffffu

#define STAT_LEVEL_MASK         0xffu
#define STAT_FULL               (1u << 8)
#define STAT_EMPTY              (1u << 9)
#define STAT_WOF                (1u << 10)

/* Address bit 14 selects zeroed, rather than replicated, narrow lanes. */
#define NO_REPLICATE            0x4000

enum {
    OP_RAW = 0x0,
    OP_RAW_REPEAT = 0x1,
    OP_TMDS = 0x2,
    OP_TMDS_REPEAT = 0x3,
    OP_NOP = 0xf,
};

/*
 * Words an infinite repeat is run for normally after being run forward by
 * whole periods: enough to refill the capture and pin history records.
 */
#define FF_MARGIN               (RP2350_HSTX_CAPTURE + 64)
/* The longest period, in words, sought in an infinite repeat. */
#define FF_MAX_PERIOD           4096
/* The furthest ahead, in cycles, the next FIFO pop is looked for. */
#define POP_LOOKAHEAD           (1u << 20)

/* Register fields. */

static unsigned csr_shift(RP2350HSTXState *s)
{
    return extract32(s->csr, 8, 5);
}

static unsigned csr_n_shifts(RP2350HSTXState *s)
{
    unsigned n = extract32(s->csr, 16, 5);

    return n ? n : 32;
}

static unsigned csr_clkphase(RP2350HSTXState *s)
{
    return extract32(s->csr, 24, 4);
}

/* The generated clock's period in half-cycles; CLKDIV 0 is 16 cycles. */
static unsigned clk_period(RP2350HSTXState *s)
{
    unsigned d = extract32(s->csr, 28, 4);

    return (d ? d : 16) * 2;
}

static bool hstx_en(RP2350HSTXState *s)
{
    return s->csr & CSR_EN;
}

static bool hstx_expand_en(RP2350HSTXState *s)
{
    return s->csr & CSR_EXPAND_EN;
}

static bool op_tmds(uint32_t op)
{
    return op == OP_TMDS || op == OP_TMDS_REPEAT;
}

static bool op_repeat(uint32_t op)
{
    return op == OP_RAW_REPEAT || op == OP_TMDS_REPEAT;
}

/* EXPAND_SHIFT's fields for the op's bank: RAW_ or ENC_. */
static unsigned exp_shift(RP2350HSTXState *s, uint32_t op)
{
    return extract32(s->expand_shift, op_tmds(op) ? 16 : 0, 5);
}

static unsigned exp_n_shifts(RP2350HSTXState *s, uint32_t op)
{
    unsigned n = extract32(s->expand_shift, op_tmds(op) ? 24 : 8, 5);

    return n ? n : 32;
}

/* TMDS encoding, as in chapter 3 of the DVI 1.0 specification. */

static uint32_t tmds_encode(uint8_t d, int32_t *cnt)
{
    int n1 = ctpop8(d);
    bool xnor = n1 > 4 || (n1 == 4 && !(d & 1));
    uint32_t qm = d & 1;
    int n1q, n0q, i;
    bool qm8;

    for (i = 1; i < 8; i++) {
        uint32_t b = ((qm >> (i - 1)) ^ (d >> i) ^ xnor) & 1;

        qm |= b << i;
    }
    qm8 = !xnor;
    n1q = ctpop8(qm);
    n0q = 8 - n1q;

    if (*cnt == 0 || n1q == n0q) {
        *cnt += qm8 ? n1q - n0q : n0q - n1q;
        return (!qm8 << 9) | (qm8 << 8) | (qm8 ? qm : ~qm & 0xff);
    }
    if ((*cnt > 0 && n1q > n0q) || (*cnt < 0 && n0q > n1q)) {
        *cnt += 2 * qm8 + n0q - n1q;
        return (1u << 9) | (qm8 << 8) | (~qm & 0xff);
    }
    *cnt += -2 * !qm8 + n1q - n0q;
    return (qm8 << 8) | qm;
}

/*
 * The expander's output for expansion shift register contents `x`: x
 * itself for raw commands, or three TMDS symbols, lane 0 in bits 9:0.
 * Each lane encodes x rotated right by Ln_ROT, keeping Ln_NBITS + 1 bits
 * down from bit 7 and the rest zero.
 */
static uint32_t hstx_expand_word(RP2350HSTXState *s, uint32_t op, uint32_t x,
                                 int32_t *disp)
{
    uint32_t w = 0;
    int l;

    if (!op_tmds(op)) {
        return x;
    }
    for (l = 0; l < RP2350_HSTX_LANES; l++) {
        uint32_t f = extract32(s->expand_tmds, l * 8, 8);
        unsigned nbits = extract32(f, 5, 3) + 1;
        uint8_t d = ror32(x, f & 0x1f) & (0xff << (8 - nbits));

        w |= tmds_encode(d, &disp[l]) << (l * 10);
    }
    return w;
}

/* The FIFO. */

static bool fifo_pop(RP2350HSTXCore *c, uint32_t *w)
{
    if (!c->fifo_level) {
        return false;
    }
    *w = c->fifo[c->fifo_head];
    c->fifo_head = (c->fifo_head + 1) % RP2350_HSTX_FIFO_DEPTH;
    c->fifo_level--;
    return true;
}

/* The command expander. */

static void hstx_command(RP2350HSTXState *s, RP2350HSTXCore *c, uint32_t w,
                         RP2350HSTXRec *rec)
{
    uint32_t op = extract32(w, 12, 4);
    uint32_t count = extract32(w, 0, 12);

    switch (op) {
    case OP_RAW:
    case OP_RAW_REPEAT:
    case OP_TMDS:
    case OP_TMDS_REPEAT:
        c->exp_cmd = true;
        c->exp_op = op;
        c->exp_infinite = count == 0;
        c->exp_count = count;
        c->exp_valid = false;
        break;
    case OP_NOP:
        break;
    default:
        if (rec) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-hstx: reserved command "
                          "expander opcode 0x%x\n", op);
        }
        break;
    }
}

static void exp_load(RP2350HSTXState *s, RP2350HSTXCore *c, uint32_t w)
{
    c->exp_sr = w;
    c->exp_valid = true;
    c->exp_left = exp_n_shifts(s, c->exp_op);
}

/* The expander's next output word, from its loaded expansion register. */
static uint32_t exp_output(RP2350HSTXState *s, RP2350HSTXCore *c)
{
    uint32_t op = c->exp_op;
    uint32_t w = hstx_expand_word(s, op, c->exp_sr, c->disp);

    c->exp_sr = ror32(c->exp_sr, exp_shift(s, op));
    if (!op_repeat(op) && !--c->exp_left) {
        c->exp_valid = false;
    }
    if (!c->exp_infinite && !--c->exp_count) {
        c->exp_cmd = false;
        c->exp_valid = false;
    }
    return w;
}

/*
 * The output shift register needs a word this cycle. Returns whether one
 * is supplied in *w; *popped says whether the FIFO was popped.
 */
static bool hstx_supply(RP2350HSTXState *s, RP2350HSTXCore *c, uint32_t *w,
                        bool *popped, RP2350HSTXRec *rec)
{
    uint32_t v;

    if (!hstx_expand_en(s)) {
        *popped = fifo_pop(c, w);
        return *popped;
    }
    if (!c->exp_cmd) {
        /* A command: no output in the cycle it is popped. */
        if (fifo_pop(c, &v)) {
            *popped = true;
            hstx_command(s, c, v, rec);
        }
        return false;
    }
    if (!c->exp_valid) {
        if (!fifo_pop(c, &v)) {
            return false;
        }
        *popped = true;
        exp_load(s, c, v);
    }
    *w = exp_output(s, c);
    return true;
}

/* A cycle in which the FIFO is not otherwise popped: the expander's fetch. */
static bool hstx_prefetch(RP2350HSTXState *s, RP2350HSTXCore *c,
                          RP2350HSTXRec *rec)
{
    uint32_t v;

    if (!hstx_expand_en(s) || (c->exp_cmd && c->exp_valid) ||
        !fifo_pop(c, &v)) {
        return false;
    }
    if (c->exp_cmd) {
        exp_load(s, c, v);
    } else {
        hstx_command(s, c, v, rec);
    }
    return true;
}

static bool hstx_wants_pop(RP2350HSTXState *s, RP2350HSTXCore *c)
{
    return hstx_expand_en(s) && c->fifo_level &&
           !(c->exp_cmd && c->exp_valid);
}

/* Nothing can happen until the FIFO is written or the block reconfigured. */
static bool hstx_idle(RP2350HSTXState *s, RP2350HSTXCore *c)
{
    if (!hstx_en(s)) {
        return true;
    }
    if (c->sr_left || c->fifo_level) {
        return false;
    }
    return !(hstx_expand_en(s) && c->exp_cmd && c->exp_valid);
}

/*
 * At a word boundary of an infinite repeat: from here on the block pops
 * nothing, and outputs a word every N_SHIFTS cycles for ever.
 */
static bool hstx_steady(RP2350HSTXState *s, RP2350HSTXCore *c)
{
    return hstx_en(s) && hstx_expand_en(s) && c->exp_cmd && c->exp_valid &&
           c->exp_infinite && op_repeat(c->exp_op) && !c->sr_left;
}

/* The output shift register and the crossbar. */

static bool clk_level(RP2350HSTXState *s, uint32_t count)
{
    unsigned period = clk_period(s);

    return count % period >= period / 2;
}

static bool sr_bit(RP2350HSTXState *s, uint32_t sr, unsigned sel)
{
    if (sel >= 24 && (s->csr & CSR_COUPLED_MODE)) {
        unsigned pio = extract32(s->csr, 5, 2);

        return pio < RP2350_HSTX_PIOS && (s->pio_out[pio] >> (sel - 24)) & 1;
    }
    return (sr >> sel) & 1;
}

/* Output bit n in the half-cycle selecting `sel`, at clock count `count`. */
static bool out_bit(RP2350HSTXState *s, int n, uint32_t sr, unsigned sel,
                    uint32_t count)
{
    uint32_t cfg = s->bit[n];
    bool v = cfg & BITN_CLK ? clk_level(s, count) : sr_bit(s, sr, sel);

    return v ^ !!(cfg & BITN_INV);
}

/* Record the outputs of one shifting cycle. */
static void hstx_emit(RP2350HSTXState *s, RP2350HSTXCore *c,
                      RP2350HSTXRec *rec)
{
    uint8_t pins = 0;
    int n;

    if (!rec) {
        return;
    }
    for (n = 0; n < RP2350_HSTX_BITS; n++) {
        uint64_t p = out_bit(s, n, c->sr, extract32(s->bit[n], 0, 5),
                             c->clk_count);
        uint64_t q = out_bit(s, n, c->sr, extract32(s->bit[n], 8, 5),
                             c->clk_count + 1);

        rec->history[n] = (rec->history[n] >> 2) | (p << 62) | (q << 63);
        pins |= q << n;
    }
    rec->pins = pins;
    rec->half_cycles += 2;
}

static void hstx_shift(RP2350HSTXState *s, RP2350HSTXCore *c, uint64_t k)
{
    c->sr = ror32(c->sr, (k * csr_shift(s)) % 32);
    c->sr_left -= k;
    c->clk_count = (c->clk_count + 2 * k) % clk_period(s);
}

/* k cycles in which the output shift register only shifts. */
static void hstx_shift_n(RP2350HSTXState *s, RP2350HSTXCore *c, uint64_t k,
                         RP2350HSTXRec *rec)
{
    /* Only the last 32 cycles show in the 64 half-cycle history. */
    if (k > 32) {
        hstx_shift(s, c, k - 32);
        if (rec) {
            rec->half_cycles += 2 * (k - 32);
        }
        k = 32;
    }
    while (k--) {
        hstx_emit(s, c, rec);
        hstx_shift(s, c, 1);
    }
}

/* Run one clk_hstx cycle; returns whether the FIFO was popped. */
static bool hstx_cycle(RP2350HSTXState *s, RP2350HSTXCore *c,
                       RP2350HSTXRec *rec)
{
    bool popped = false;
    uint32_t w;

    if (!c->sr_left && hstx_supply(s, c, &w, &popped, rec)) {
        c->sr = w;
        c->sr_left = csr_n_shifts(s);
        if (rec) {
            rec->capture[rec->words % RP2350_HSTX_CAPTURE] = w;
            rec->words++;
        }
    }
    if (c->sr_left) {
        hstx_emit(s, c, rec);
        hstx_shift(s, c, 1);
    }
    if (!popped) {
        popped = hstx_prefetch(s, c, rec);
    }
    c->cycle++;
    return popped;
}

/* The state an infinite repeat's output evolves, word by word. */
typedef struct HSTXRepeatState {
    uint32_t exp_sr;
    uint32_t clk_count;
    int32_t disp[RP2350_HSTX_LANES];
} HSTXRepeatState;

static void repeat_step(RP2350HSTXState *s, uint32_t op, HSTXRepeatState *r)
{
    hstx_expand_word(s, op, r->exp_sr, r->disp);
    r->exp_sr = ror32(r->exp_sr, exp_shift(s, op));
    r->clk_count = (r->clk_count + 2 * csr_n_shifts(s)) % clk_period(s);
}

static bool repeat_eq(const HSTXRepeatState *a, const HSTXRepeatState *b)
{
    return a->exp_sr == b->exp_sr && a->clk_count == b->clk_count &&
           !memcmp(a->disp, b->disp, sizeof(a->disp));
}

/*
 * Run a steady infinite repeat forward by whole periods of its output,
 * found by Brent's cycle detection, leaving at least FF_MARGIN words to
 * run normally before `end`. Returns whether it moved.
 */
static bool hstx_fast_forward(RP2350HSTXState *s, RP2350HSTXCore *c,
                              uint64_t end, RP2350HSTXRec *rec)
{
    unsigned n = csr_n_shifts(s);
    uint64_t words = (end - c->cycle) / n;
    uint32_t op = c->exp_op;
    HSTXRepeatState x0 = {
        .exp_sr = c->exp_sr,
        .clk_count = c->clk_count % clk_period(s),
    };
    HSTXRepeatState t, h;
    uint64_t power = 1, lam = 1, mu = 0, i, k;

    if (words < 2 * FF_MARGIN) {
        return false;
    }
    memcpy(x0.disp, c->disp, sizeof(x0.disp));
    t = x0;
    h = x0;
    repeat_step(s, op, &h);
    while (!repeat_eq(&t, &h)) {
        if (power == lam) {
            t = h;
            power *= 2;
            lam = 0;
        }
        repeat_step(s, op, &h);
        if (++lam > FF_MAX_PERIOD) {
            return false;
        }
    }
    t = x0;
    h = x0;
    for (i = 0; i < lam; i++) {
        repeat_step(s, op, &h);
    }
    while (!repeat_eq(&t, &h)) {
        repeat_step(s, op, &t);
        repeat_step(s, op, &h);
        mu++;
    }
    if (words < mu + lam + FF_MARGIN) {
        return false;
    }
    /* Into the periodic part, normally. */
    for (i = 0; i < mu * n; i++) {
        hstx_cycle(s, c, rec);
    }
    k = (words - mu - FF_MARGIN) / lam * lam;
    c->cycle += k * n;
    c->clk_count %= clk_period(s);
    if (rec) {
        rec->words += k;
        rec->half_cycles += 2 * k * n;
    }
    return k != 0;
}

/*
 * Run the block up to clk_hstx cycle `end`. With stop_on_pop, stop after
 * the first cycle that pops the FIFO, and return whether there was one.
 * `rec` is NULL for a look-ahead that leaves no trace.
 */
static bool hstx_run(RP2350HSTXState *s, RP2350HSTXCore *c, uint64_t end,
                     bool stop_on_pop, RP2350HSTXRec *rec)
{
    bool ff_tried = false;

    while (c->cycle < end) {
        if (hstx_idle(s, c)) {
            c->cycle = end;
            break;
        }
        if (hstx_steady(s, c)) {
            if (stop_on_pop) {
                c->cycle = end;
                break;
            }
            if (!ff_tried) {
                ff_tried = true;
                if (hstx_fast_forward(s, c, end, rec)) {
                    continue;
                }
            }
        }
        if (hstx_cycle(s, c, rec) && stop_on_pop) {
            return true;
        }
        if (c->sr_left > 1 && !hstx_wants_pop(s, c)) {
            uint64_t k = MIN(c->sr_left - 1, end - c->cycle);

            hstx_shift_n(s, c, k, rec);
            c->cycle += k;
        }
    }
    return false;
}

/* Time. */

static uint64_t hstx_cycle_at(RP2350HSTXState *s, int64_t ns)
{
    if (ns <= s->base_ns) {
        return s->base_cycle;
    }
    return s->base_cycle + clock_ns_to_ticks(s->clk, ns - s->base_ns);
}

/* The first virtual time at which clk_hstx has reached `cycle`. */
static int64_t hstx_ns_at(RP2350HSTXState *s, uint64_t cycle)
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

static int64_t hstx_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* Bring the block up to the current clk_hstx cycle. */
/* [spec:nuos:req:emu.hstx] */
static void hstx_sync(RP2350HSTXState *s)
{
    uint64_t now = hstx_cycle_at(s, hstx_now());

    if (s->core.cycle < now) {
        hstx_run(s, &s->core, now, false, &s->rec);
    }
}

/* The outputs while the shift register is not shifting. */
static uint8_t hstx_static_pins(RP2350HSTXState *s)
{
    RP2350HSTXCore *c = &s->core;
    uint8_t pins = 0;
    int n;

    for (n = 0; n < RP2350_HSTX_BITS; n++) {
        pins |= out_bit(s, n, c->sr, extract32(s->bit[n], 0, 5),
                        c->clk_count) << n;
    }
    return pins;
}

static void hstx_drive(RP2350HSTXState *s)
{
    uint8_t pins = hstx_idle(s, &s->core) ? hstx_static_pins(s) : s->rec.pins;
    uint8_t dreq = s->core.fifo_level < RP2350_HSTX_FIFO_DEPTH;
    int n;

    for (n = 0; n < RP2350_HSTX_BITS; n++) {
        uint8_t level = (pins >> n) & 1;

        if (s->oe_level[n] != 1) {
            s->oe_level[n] = 1;
            qemu_set_irq(s->oe[n], 1);
        }
        if (s->out_level[n] != level) {
            s->out_level[n] = level;
            qemu_set_irq(s->out[n], level);
        }
    }
    if (s->dreq_level != dreq) {
        s->dreq_level = dreq;
        qemu_set_irq(s->dreq, dreq);
    }
}

/*
 * Wake at the cycle in which a pop makes room in a full FIFO, and every
 * pin-refresh-ns while shifting.
 */
static void hstx_schedule(RP2350HSTXState *s)
{
    int64_t now = hstx_now();
    int64_t due = INT64_MAX;

    if (!clock_is_enabled(s->clk) || hstx_idle(s, &s->core)) {
        timer_del(s->timer);
        return;
    }
    if (s->core.fifo_level == RP2350_HSTX_FIFO_DEPTH) {
        RP2350HSTXCore c = s->core;
        uint64_t limit = c.cycle + POP_LOOKAHEAD;

        if (hstx_run(s, &c, limit, true, NULL)) {
            due = hstx_ns_at(s, c.cycle);
        } else if (!hstx_idle(s, &c) && !hstx_steady(s, &c)) {
            /* Still going: look again from there. */
            due = hstx_ns_at(s, limit);
        }
    }
    if (s->pin_refresh_ns) {
        due = MIN(due, now + s->pin_refresh_ns);
    }
    if (due == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, MAX(due, now));
    }
}

static void hstx_update(RP2350HSTXState *s)
{
    hstx_drive(s);
    hstx_schedule(s);
}

static void hstx_timer_cb(void *opaque)
{
    RP2350HSTXState *s = opaque;

    hstx_sync(s);
    hstx_update(s);
}

/* What clearing CSR.EN does, and holds while it is clear. */
static void hstx_halt(RP2350HSTXState *s)
{
    RP2350HSTXCore *c = &s->core;

    c->sr = 0;
    c->sr_left = 0;
    c->clk_count = csr_clkphase(s);
    c->exp_cmd = false;
    c->exp_valid = false;
    memset(c->disp, 0, sizeof(c->disp));
}

/* Registers. */

/* The value a write puts on the bus, given its size and address. */
static uint32_t bus_value(hwaddr addr, uint64_t value, unsigned size,
                          bool replicate)
{
    switch (size) {
    case 1:
        return replicate ? (value & 0xff) * 0x01010101u
                         : (value & 0xff) << ((addr & 3) * 8);
    case 2:
        return replicate ? (value & 0xffff) * 0x00010001u
                         : (value & 0xffff) << ((addr & 2) * 8);
    default:
        return value;
    }
}

/* [spec:nuos:req:emu.hstx] */
static uint64_t rp2350_hstx_ctrl_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    RP2350HSTXState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr) & ~3;
    uint32_t v = 0;

    switch (reg) {
    case A_CSR:
        v = s->csr;
        break;
    case A_BIT0 ... A_BIT7:
        v = s->bit[(reg - A_BIT0) / 4];
        break;
    case A_EXPAND_SHIFT:
        v = s->expand_shift;
        break;
    case A_EXPAND_TMDS:
        v = s->expand_tmds;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-hstx: read of bad CTRL "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    return v >> ((addr & 3) * 8);
}

/* [spec:nuos:req:emu.hstx] */
static void rp2350_hstx_ctrl_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size)
{
    RP2350HSTXState *s = opaque;
    hwaddr window = addr & ~(hwaddr)NO_REPLICATE;
    hwaddr reg = rp2350_atomic_reg(window) & ~3;
    uint32_t v = bus_value(addr, value, size, !(addr & NO_REPLICATE));

    hstx_sync(s);
    switch (reg) {
    case A_CSR: {
        bool was_en = hstx_en(s);

        s->csr = rp2350_atomic_apply(window, s->csr, v) & CSR_MASK;
        if (!hstx_en(s)) {
            /* Halted, holding the clock generator at CLKPHASE. */
            hstx_halt(s);
        } else if (!was_en) {
            s->core.clk_count = csr_clkphase(s);
        }
        break;
    }
    case A_BIT0 ... A_BIT7: {
        int n = (reg - A_BIT0) / 4;

        s->bit[n] = rp2350_atomic_apply(window, s->bit[n], v) & BITN_MASK;
        break;
    }
    case A_EXPAND_SHIFT:
        s->expand_shift = rp2350_atomic_apply(window, s->expand_shift, v) &
                          EXPAND_SHIFT_MASK;
        break;
    case A_EXPAND_TMDS:
        s->expand_tmds = rp2350_atomic_apply(window, s->expand_tmds, v) &
                         EXPAND_TMDS_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-hstx: write to bad CTRL "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    hstx_update(s);
}

static const MemoryRegionOps rp2350_hstx_ctrl_ops = {
    .read = rp2350_hstx_ctrl_read,
    .write = rp2350_hstx_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.hstx] */
static uint64_t rp2350_hstx_fifo_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    RP2350HSTXState *s = opaque;
    uint32_t v = 0;

    switch (addr & ~3) {
    case A_STAT:
        hstx_sync(s);
        v = s->core.fifo_level | (s->wof ? STAT_WOF : 0);
        if (!s->core.fifo_level) {
            v |= STAT_EMPTY;
        }
        if (s->core.fifo_level == RP2350_HSTX_FIFO_DEPTH) {
            v |= STAT_FULL;
        }
        hstx_update(s);
        break;
    case A_FIFO:
        /* Write-only. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-hstx: read of bad FIFO "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    return v >> ((addr & 3) * 8);
}

/* [spec:nuos:req:emu.hstx] */
static void rp2350_hstx_fifo_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size)
{
    RP2350HSTXState *s = opaque;
    RP2350HSTXCore *c = &s->core;
    uint32_t v = bus_value(addr, value, size, true);

    switch (addr & ~3) {
    case A_STAT:
        if (v & STAT_WOF) {
            s->wof = false;
        }
        break;
    case A_FIFO:
        hstx_sync(s);
        if (c->fifo_level == RP2350_HSTX_FIFO_DEPTH) {
            /* Dropped. */
            s->wof = true;
        } else {
            c->fifo[(c->fifo_head + c->fifo_level) %
                    RP2350_HSTX_FIFO_DEPTH] = v;
            c->fifo_level++;
        }
        hstx_update(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-hstx: write to bad FIFO "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_hstx_fifo_ops = {
    .read = rp2350_hstx_fifo_read,
    .write = rp2350_hstx_fifo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* Coupled-mode PIO outputs. */
static void hstx_pio_in(void *opaque, int n, int level)
{
    RP2350HSTXState *s = opaque;
    int pio = n / RP2350_HSTX_BITS;
    uint8_t bit = 1u << (n % RP2350_HSTX_BITS);
    uint8_t v = level ? s->pio_out[pio] | bit : s->pio_out[pio] & ~bit;

    if (v == s->pio_out[pio]) {
        return;
    }
    hstx_sync(s);
    s->pio_out[pio] = v;
    hstx_update(s);
}

/*
 * A change of clk_hstx frequency: run the block to the change at the old
 * rate, then count from there at the new one.
 */
static void rp2350_hstx_clk_update(void *opaque, ClockEvent event)
{
    RP2350HSTXState *s = opaque;

    if (event == ClockPreUpdate) {
        hstx_sync(s);
        s->base_ns = hstx_now();
        s->base_cycle = s->core.cycle;
    } else {
        hstx_update(s);
    }
}

/* Observability properties. */

static void hstx_get_capture(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    RP2350HSTXState *s = RP2350_HSTX(obj);
    uint32List *list = NULL;
    uint64_t n, i;

    hstx_sync(s);
    hstx_update(s);
    n = MIN(s->rec.words, RP2350_HSTX_CAPTURE);
    for (i = 0; i < n; i++) {
        /* Prepending, newest first, leaves the list oldest first. */
        uint64_t w = s->rec.words - 1 - i;

        QAPI_LIST_PREPEND(list, s->rec.capture[w % RP2350_HSTX_CAPTURE]);
    }
    visit_type_uint32List(v, name, &list, errp);
    qapi_free_uint32List(list);
}

static void hstx_get_history(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    RP2350HSTXState *s = RP2350_HSTX(obj);
    uint64List *list = NULL;
    int n;

    hstx_sync(s);
    hstx_update(s);
    for (n = RP2350_HSTX_BITS - 1; n >= 0; n--) {
        QAPI_LIST_PREPEND(list, s->rec.history[n]);
    }
    visit_type_uint64List(v, name, &list, errp);
    qapi_free_uint64List(list);
}

static void hstx_get_count(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    RP2350HSTXState *s = RP2350_HSTX(obj);
    uint64_t val;

    hstx_sync(s);
    hstx_update(s);
    val = opaque ? s->rec.half_cycles : s->rec.words;
    visit_type_uint64(v, name, &val, errp);
}

static void rp2350_hstx_hold_reset(Object *obj, ResetType type)
{
    RP2350HSTXState *s = RP2350_HSTX(obj);

    timer_del(s->timer);
    s->base_ns = hstx_now();
    s->base_cycle = 0;
    s->csr = CSR_RESET;
    memset(s->bit, 0, sizeof(s->bit));
    s->expand_shift = EXPAND_SHIFT_RESET;
    s->expand_tmds = 0;
    s->wof = false;
    memset(&s->core, 0, sizeof(s->core));
    hstx_halt(s);
    memset(&s->rec, 0, sizeof(s->rec));
    /* The PIO outputs are outside the block; they are not reset. */
    memset(s->out_level, 0xff, sizeof(s->out_level));
    memset(s->oe_level, 0xff, sizeof(s->oe_level));
    s->dreq_level = 0xff;
    hstx_drive(s);
}

static void rp2350_hstx_init(Object *obj)
{
    RP2350HSTXState *s = RP2350_HSTX(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->ctrl_iomem, obj, &rp2350_hstx_ctrl_ops, s,
                          "rp2350-hstx-ctrl", RP2350_HSTX_CTRL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->ctrl_iomem);
    memory_region_init_io(&s->fifo_iomem, obj, &rp2350_hstx_fifo_ops, s,
                          "rp2350-hstx-fifo", RP2350_HSTX_FIFO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->fifo_iomem);
    qdev_init_gpio_out_named(dev, s->out, RP2350_HSTX_OUT, RP2350_HSTX_BITS);
    qdev_init_gpio_out_named(dev, s->oe, RP2350_HSTX_OE, RP2350_HSTX_BITS);
    qdev_init_gpio_out_named(dev, &s->dreq, RP2350_HSTX_DREQ, 1);
    qdev_init_gpio_in_named(dev, hstx_pio_in, RP2350_HSTX_PIO_OUT,
                            RP2350_HSTX_PIOS * RP2350_HSTX_BITS);
    s->clk = qdev_init_clock_in(dev, "clk", rp2350_hstx_clk_update, s,
                                ClockPreUpdate | ClockUpdate);

    object_property_add(obj, "capture", "uint32List", hstx_get_capture,
                        NULL, NULL, NULL);
    object_property_add(obj, "capture-words", "uint64", hstx_get_count,
                        NULL, NULL, NULL);
    object_property_add(obj, "half-cycles", "uint64", hstx_get_count,
                        NULL, NULL, (void *)1);
    object_property_add(obj, "pin-history", "uint64List", hstx_get_history,
                        NULL, NULL, NULL);
}

static void rp2350_hstx_realize(DeviceState *dev, Error **errp)
{
    RP2350HSTXState *s = RP2350_HSTX(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hstx_timer_cb, s);
}

static const Property rp2350_hstx_properties[] = {
    DEFINE_PROP_UINT32("pin-refresh-ns", RP2350HSTXState, pin_refresh_ns,
                       10000),
};

static const VMStateDescription vmstate_rp2350_hstx_core = {
    .name = TYPE_RP2350_HSTX "-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(cycle, RP2350HSTXCore),
        VMSTATE_UINT32_ARRAY(fifo, RP2350HSTXCore, RP2350_HSTX_FIFO_DEPTH),
        VMSTATE_UINT32(fifo_head, RP2350HSTXCore),
        VMSTATE_UINT32(fifo_level, RP2350HSTXCore),
        VMSTATE_UINT32(sr, RP2350HSTXCore),
        VMSTATE_UINT32(sr_left, RP2350HSTXCore),
        VMSTATE_UINT32(clk_count, RP2350HSTXCore),
        VMSTATE_BOOL(exp_cmd, RP2350HSTXCore),
        VMSTATE_BOOL(exp_infinite, RP2350HSTXCore),
        VMSTATE_BOOL(exp_valid, RP2350HSTXCore),
        VMSTATE_UINT32(exp_op, RP2350HSTXCore),
        VMSTATE_UINT32(exp_count, RP2350HSTXCore),
        VMSTATE_UINT32(exp_sr, RP2350HSTXCore),
        VMSTATE_UINT32(exp_left, RP2350HSTXCore),
        VMSTATE_INT32_ARRAY(disp, RP2350HSTXCore, RP2350_HSTX_LANES),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_hstx_rec = {
    .name = TYPE_RP2350_HSTX "-rec",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(capture, RP2350HSTXRec, RP2350_HSTX_CAPTURE),
        VMSTATE_UINT64(words, RP2350HSTXRec),
        VMSTATE_UINT64(half_cycles, RP2350HSTXRec),
        VMSTATE_UINT64_ARRAY(history, RP2350HSTXRec, RP2350_HSTX_BITS),
        VMSTATE_UINT8(pins, RP2350HSTXRec),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_hstx = {
    .name = TYPE_RP2350_HSTX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, RP2350HSTXState),
        VMSTATE_TIMER_PTR(timer, RP2350HSTXState),
        VMSTATE_INT64(base_ns, RP2350HSTXState),
        VMSTATE_UINT64(base_cycle, RP2350HSTXState),
        VMSTATE_UINT32(csr, RP2350HSTXState),
        VMSTATE_UINT32_ARRAY(bit, RP2350HSTXState, RP2350_HSTX_BITS),
        VMSTATE_UINT32(expand_shift, RP2350HSTXState),
        VMSTATE_UINT32(expand_tmds, RP2350HSTXState),
        VMSTATE_BOOL(wof, RP2350HSTXState),
        VMSTATE_STRUCT(core, RP2350HSTXState, 1, vmstate_rp2350_hstx_core,
                       RP2350HSTXCore),
        VMSTATE_STRUCT(rec, RP2350HSTXState, 1, vmstate_rp2350_hstx_rec,
                       RP2350HSTXRec),
        VMSTATE_UINT8_ARRAY(pio_out, RP2350HSTXState, RP2350_HSTX_PIOS),
        VMSTATE_UINT8_ARRAY(out_level, RP2350HSTXState, RP2350_HSTX_BITS),
        VMSTATE_UINT8_ARRAY(oe_level, RP2350HSTXState, RP2350_HSTX_BITS),
        VMSTATE_UINT8(dreq_level, RP2350HSTXState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_hstx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_hstx_realize;
    rc->phases.hold = rp2350_hstx_hold_reset;
    dc->vmsd = &vmstate_rp2350_hstx;
    device_class_set_props(dc, rp2350_hstx_properties);
}

/* [spec:nuos:req:emu.hstx] */
static const TypeInfo rp2350_hstx_info = {
    .name          = TYPE_RP2350_HSTX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350HSTXState),
    .instance_init = rp2350_hstx_init,
    .class_init    = rp2350_hstx_class_init,
};

static void rp2350_hstx_register_types(void)
{
    type_register_static(&rp2350_hstx_info);
}

type_init(rp2350_hstx_register_types)
