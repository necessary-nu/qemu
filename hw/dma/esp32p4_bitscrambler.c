/*
 * ESP32-P4 BitScrambler
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 62 (BitScrambler) and register 21.50 (HP_SYSTEM_BITSCRAMBLER_PERI_SEL),
 * with the layout and reset values of ESP-IDF's bitscrambler_reg.h for
 * v3, its bitscrambler_ll driver and bitscrambler and loopback drivers,
 * and its assembler, tools/bsasm.py, whose output the instruction memory
 * holds.
 *
 * Two cores, TX and RX, each a small processor in the DMA stream of the
 * peripheral HP_SYSTEM_BITSCRAMBLER_PERI_SEL attaches it to: the TX core
 * between the GDMA channel that reads memory for the peripheral and the
 * peripheral, the RX core between the peripheral and the channel that
 * writes memory for it (TRM 62.1). A core is in the stream while its ENA
 * is set and its selection names a peripheral; it interposes on that
 * path of the controller serving the peripheral (GDMA-AXI for LCD_CAM,
 * GPSPI2, GPSPI3, PARLIO, AES and SHA; GDMA-AHB for ADC, I2S0 to 2,
 * I3C_MST, UHCI0 and RMT), as include/hw/dma/esp32p4_gdma.h describes.
 * In loopback (BITSCRAMBLER_LOOP_MODE) the TX core's output goes to the
 * RX channel of the peripheral it is attached to, that peripheral's own
 * DMA paths are cut off, and the RX core is out of the stream.
 *
 * A core executes one 257-bit instruction a cycle of its clock
 * (BITSCRAMBLER_TX/RX_SYS_CLK), from instruction memory (eight, written
 * nine words each through INST_CFG0/1, the 257th bit in the ninth word's
 * bit 0). An instruction routes 32 output bits, each through a 128-to-1
 * multiplexer (CTL_MUX_SEL_n, TRM table 62.4-3), from: the 64-bit input
 * register; with CTL_CTR_SRC_SEL, counters A and B in place of its high
 * half; with CTL_LUT_SRC_SEL, the LUT's output in place of its top 8, 16
 * or 32 bits; comparisons of counter B with the previous output; constant
 * 0 and 1; and the previous output. CTL_MUX_REL offsets the selections of
 * the input register's (or counters') bits by counter A (table 62.4-2).
 * The LUT (2048 bytes, 8, 16 or 32 bits wide as LUT_MODE sets) is read
 * at bits 16 up of the previous output, 9, 10 or 11 of them for widths
 * 32, 16 and 8. CTL_WR_OUT writes the low 0, 8, 16 or 32 output bits to
 * the output FIFO; CTL_READ_IN shifts 0, 8, 16 or 32 bits from the input
 * FIFO into the input register's top, the rest moving down. The opcode
 * (table 62.4-4) is LOOP, ADD, IF, IFN, LDCTD or LDCTI. An instruction
 * waits, whole, while its read or its write cannot proceed.
 *
 * Fetch mode. With FETCH_MODE clear the core first reads 64 bits into the
 * input register (two cycles of 32) and an instruction's read happens at
 * its end, for the instructions after it. With FETCH_MODE set nothing is
 * prefetched and an instruction's read happens before its outputs are
 * routed, so it sees the bits it reads: the hardware reads, at the end of
 * each cycle, what the next instruction reads, and so does the model,
 * or, when the input has not got the bytes yet, just before that
 * instruction. ESP-IDF's tests of loops that read a byte and use it in
 * the same instruction (loop.bsasm) need this.
 *
 * The end of a stream. When the core reads the input frame's last byte
 * it counts, from the next operation on, the bytes it reads (EOF_MODE
 * clear) or writes (EOF_MODE set); reads past the frame's end return
 * zeros with RD_DUMMY set, and wait for more input without it. Once the
 * count reaches TAILING_BITS / 8 bytes the core ends the stream: the last
 * byte it has written carries the frame's end downstream, and it stops.
 * In fetch mode 1 a read that reaches the count ends the stream before
 * its instruction routes and writes anything: it is the read ahead at
 * the end of the cycle before, whose last byte written carries the
 * frame's end. These are the semantics that reproduce, byte for byte,
 * what ESP-IDF's BitScrambler and PARLIO tests expect of the hardware
 * (eof_upstream, eof_downstream, lut32, loop, trivial, the PARLIO TX
 * programs); the TRM gives EOF_MODE's two values the other way round, as
 * does its table 62.4-11, against ESP-IDF's driver
 * (BITSCRAMBLER_EOF_MODE_READ is 0) and the register header.
 *
 * States: halted (HALT set, or a halt complete: IN_IDLE), running
 * (IN_RUN), paused (PAUSE: IN_PAUSE), and waiting (IN_WAIT): halted with
 * HALT_MODE clear while its output FIFO still holds bytes for downstream,
 * or after a stream's end until they are taken. Clearing HALT starts the
 * program from its first instruction with the registers and counters
 * zero; halting empties the FIFOs once the output is written back (at
 * once with HALT_MODE set), and FIFO_RST empties them at any time.
 * The core takes input only while running or paused.
 *
 * Each cycle happens at its own time, however late the QEMU timer that
 * runs it: a late cycle runs as of its time (esp32_line_run_at()), with
 * the channel's bus rounds due before it run first, and the next follows
 * a cycle after it. The peripheral's DMA path and register accesses
 * first take the cycles due before them (the interposer's sync op,
 * include/hw/dma/esp32p4_gdma.h).
 *
 * STATE's FIFO bit: for the TX core it reads 1 while the output FIFO
 * holds bytes for the peripheral (ESP-IDF's bitscrambler_ll_is_fifo_ready
 * reads it so; its reset value 0 agrees), for the RX core while the input
 * FIFO is full. EOF_GET_CNT is the count of bytes toward TAILING_BITS at
 * the stream's end; EOF_OVERLOAD is set when a core reads the end of a
 * second frame while counting (with RD_DUMMY clear).
 *
 * Not modelled: the FIFOs' depth, which the TRM does not give, is taken
 * as 16 bytes each way; the instruction and LUT memories are RAMs and
 * keep their contents through resets (the register header's "reset
 * values" of INST_CFG1 and LUT_CFG1, 4, 12, 20 and 28, are the
 * registers' offsets, not contents); BITSCRAMBLER_CLK_EN and
 * HP_SYSTEM_PERI_MEM_CLK_FORCE_ON's memory clock forcing are kept as
 * written and gate nothing. The per-core resets (RST_EN_BITSCRAMBLER_TX
 * and _RX) reset that core's registers and state.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32p4_bitscrambler.h"
#include "hw/gpio/esp32_line.h"
#include "migration/vmstate.h"

#define TX                  ESP32P4_BS_TX
#define RX                  ESP32P4_BS_RX
#define OUT                 ESP32P4_GDMA_OUT
#define IN                  ESP32P4_GDMA_IN

#define A_INST_CFG0(c)      (0x00 + 8 * (c))
#define A_INST_CFG1(c)      (0x04 + 8 * (c))
#define A_LUT_CFG0(c)       (0x10 + 8 * (c))
#define A_LUT_CFG1(c)       (0x14 + 8 * (c))
#define A_TAILING_BITS(c)   (0x20 + 4 * (c))
#define A_CTRL(c)           (0x28 + 4 * (c))
#define A_STATE(c)          (0x30 + 4 * (c))
#define A_SYS               0xf8
#define A_VERSION           0xfc

#define INST_IDX(v)         extract32((v), 0, 3)
#define INST_POS(v)         extract32((v), 3, 4)
#define INST_CFG0_MASK      0x7f
#define LUT_IDX(v)          extract32((v), 0, 11)
#define LUT_MODE(v)         extract32((v), 11, 2)
#define LUT_CFG0_MASK       0x1fff
#define TAILING_MASK        0xffff

#define CTRL_ENA            BIT(0)
#define CTRL_PAUSE          BIT(1)
#define CTRL_HALT           BIT(2)
#define CTRL_EOF_MODE       BIT(3)
#define CTRL_COND_MODE      BIT(4)
#define CTRL_FETCH_MODE     BIT(5)
#define CTRL_HALT_MODE      BIT(6)
#define CTRL_RD_DUMMY       BIT(7)
#define CTRL_FIFO_RST       BIT(8)
#define CTRL_MASK           0xff

#define STATE_IN_IDLE       BIT(0)
#define STATE_IN_RUN        BIT(1)
#define STATE_IN_WAIT       BIT(2)
#define STATE_IN_PAUSE      BIT(3)
#define STATE_FIFO          BIT(4)
#define STATE_EOF_CNT_SHIFT 16
#define STATE_EOF_CNT_MAX   0x3fff
#define STATE_EOF_OVERLOAD  BIT(30)
#define STATE_EOF_TRACE_CLR BIT(31)

#define SYS_LOOP_MODE       BIT(0)
#define SYS_CLK_EN          BIT(31)
#define SYS_MASK            (SYS_LOOP_MODE | SYS_CLK_EN)

#define VERSION_RESET       0x2303240
#define VERSION_MASK        0x0fffffff
#define PERI_SEL_RESET      0xff

/* The instruction's fields (TRM table 62.4-1). */
#define I_MUX_SEL(n)        (7 * (n))
#define I_OPCODE            224
#define I_READ_IN           250
#define I_WR_OUT            252
#define I_MUX_REL           254
#define I_CTR_SRC_SEL       255
#define I_LUT_SRC_SEL       256

/* The opcode's fields (TRM table 62.4-4). */
#define OP_LOOP             BIT(25)
#define OP_C(op)            extract32((op), 24, 1)
#define OP_H(op)            extract32((op), 23, 1)
#define OP_L(op)            extract32((op), 22, 1)
#define OP_TGT(op)          extract32((op), 21, 3)
#define OP_KIND(op)         extract32((op), 16, 5)
#define OP_END_VAL(op)      extract32((op), 5, 16)
#define OP_LOOP_ADD(op)     sextract32((op), 0, 5)
#define OP_IMM(op)          extract32((op), 0, 16)
#define OP_SRC(op)          extract32((op), 0, 7)
enum { OP_ADD, OP_IF, OP_IFN, OP_LDCTD, OP_LDCTI };

/* The FIFO entries' marks. */
#define F_EOF               BIT(0)
#define F_ERR               BIT(1)
#define F_NODATA            BIT(2)

enum { BS_IDLE, BS_RUN, BS_WAIT, BS_END };

static const char *const bs_name[2] = { "TX", "RX" };

/* Where each PERI_SEL value's peripheral is served: GDMA-AXI or -AHB. */
static const struct {
    bool axi;
    uint8_t periph;
} bs_periph[ESP32P4_BS_PERIPHS] = {
    { true, ESP32P4_GDMA_AXI_LCD_CAM },
    { true, ESP32P4_GDMA_AXI_SPI2 },
    { true, ESP32P4_GDMA_AXI_SPI3 },
    { true, ESP32P4_GDMA_AXI_PARLIO },
    { true, ESP32P4_GDMA_AXI_AES },
    { true, ESP32P4_GDMA_AXI_SHA },
    { false, ESP32P4_GDMA_AHB_ADC },
    { false, ESP32P4_GDMA_AHB_I2S0 },
    { false, ESP32P4_GDMA_AHB_I2S1 },
    { false, ESP32P4_GDMA_AHB_I2S2 },
    { false, ESP32P4_GDMA_AHB_I3C },
    { false, ESP32P4_GDMA_AHB_UHCI0 },
    { false, ESP32P4_GDMA_AHB_RMT },
};

/* FIFOs */

static void fifo_reset(ESP32P4BsFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static uint32_t fifo_room(ESP32P4BsFifo *f)
{
    return ESP32P4_BS_FIFO - f->num;
}

static void fifo_put(ESP32P4BsFifo *f, uint8_t byte, uint8_t flags)
{
    assert(f->num < ESP32P4_BS_FIFO);
    f->data[(f->head + f->num) % ESP32P4_BS_FIFO] = byte;
    f->flags[(f->head + f->num) % ESP32P4_BS_FIFO] = flags;
    f->num++;
}

static uint8_t fifo_peek_flags(ESP32P4BsFifo *f, uint32_t i)
{
    return f->flags[(f->head + i) % ESP32P4_BS_FIFO];
}

static void fifo_take(ESP32P4BsFifo *f, uint8_t *byte, uint8_t *flags)
{
    assert(f->num > 0);
    *byte = f->data[f->head];
    *flags = f->flags[f->head];
    f->head = (f->head + 1) % ESP32P4_BS_FIFO;
    f->num--;
}

/* The core's state */

static bool core_loop(ESP32P4BsCore *c)
{
    return c->dir == TX && (c->s->sys & SYS_LOOP_MODE);
}

/* Whether the core executes: enabled, running, not paused nor held. */
static bool core_active(ESP32P4BsCore *c)
{
    return (c->ctrl & CTRL_ENA) && c->state == BS_RUN &&
           !(c->ctrl & CTRL_PAUSE) && !c->reset_held &&
           clock_is_enabled(c->clk);
}

/* Whether the core takes input: running or paused. */
static bool core_takes_input(ESP32P4BsCore *c)
{
    return (c->ctrl & CTRL_ENA) && c->state == BS_RUN && !c->reset_held;
}

/* When the cycle @timer runs falls due. */
static int64_t core_due_ns(ESP32P4BsCore *c)
{
    return esp32p4_clk_count_time(&c->edges, c->due_cycle);
}

/* Arm the core's timer for its step at clock cycle @cycle. */
static void core_arm(ESP32P4BsCore *c, uint64_t cycle)
{
    c->due_cycle = cycle;
    timer_mod(c->timer, core_due_ns(c));
}

/*
 * Let the core run from the clock's next edge after the change that lets
 * it, at esp32_line_time().
 */
static void core_kick(ESP32P4BsCore *c)
{
    if (core_active(c) && !timer_pending(c->timer)) {
        core_arm(c, esp32p4_clk_count_at(&c->edges, esp32_line_time()) + 1);
    }
}

/* The output FIFO went from empty: the TX core's peripheral may pull. */
static void core_tell_periph_data(ESP32P4BsCore *c)
{
    if (c->gdma && c->dir == TX && !core_loop(c)) {
        esp32p4_gdma_periph_ready(c->gdma, c->periph, OUT);
    }
}

/* Room appeared in a full input FIFO: the RX core's peripheral may push. */
static void core_tell_periph_room(ESP32P4BsCore *c)
{
    if (c->gdma && c->dir == RX) {
        esp32p4_gdma_periph_ready(c->gdma, c->periph, IN);
    }
}

static void core_flush(ESP32P4BsCore *c)
{
    bool was_full = c->fifo_in.num == ESP32P4_BS_FIFO;

    fifo_reset(&c->fifo_in);
    fifo_reset(&c->fifo_out);
    if (was_full) {
        core_tell_periph_room(c);
    }
}

/* Halted: idle, the FIFOs emptied. */
static void core_halted(ESP32P4BsCore *c)
{
    c->state = BS_IDLE;
    timer_del(c->timer);
    core_flush(c);
}

/* The output written back: a waiting or ended core goes idle. */
static void core_check_drained(ESP32P4BsCore *c)
{
    if (c->fifo_out.num) {
        return;
    }
    if (c->state == BS_WAIT) {
        core_halted(c);
    }
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void core_start(ESP32P4BsCore *c)
{
    c->state = BS_RUN;
    c->ip = 0;
    c->fetched = false;
    c->in = 0;
    c->last = 0;
    c->ctr[0] = c->ctr[1] = 0;
    c->eof_seen = false;
    c->eof_err = false;
    c->eof_count = 0;
    c->prefetch = (c->ctrl & CTRL_FETCH_MODE) ? 0 : 8;
    core_kick(c);
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void core_halt(ESP32P4BsCore *c)
{
    if (c->state == BS_IDLE) {
        return;
    }
    if (!(c->ctrl & CTRL_HALT_MODE) && c->fifo_out.num) {
        c->state = BS_WAIT;
        timer_del(c->timer);
        return;
    }
    core_halted(c);
}

static void core_reset(ESP32P4BsCore *c)
{
    timer_del(c->timer);
    c->inst_cfg0 = 0;
    c->lut_cfg0 = 0;
    c->tailing_bits = 0;
    c->ctrl = CTRL_HALT;
    c->eof_overload = false;
    c->eof_get_cnt = 0;
    c->state = BS_IDLE;
    c->prefetch = 0;
    c->ip = 0;
    c->fetched = false;
    c->in = 0;
    c->last = 0;
    c->ctr[0] = c->ctr[1] = 0;
    c->eof_seen = false;
    c->eof_err = false;
    c->eof_count = 0;
    fifo_reset(&c->fifo_in);
    fifo_reset(&c->fifo_out);
}

/* The paths */

static ESP32P4GdmaState *bs_gdma(ESP32P4BitScramblerState *s, bool axi)
{
    return ESP32P4_GDMA(s->gdma_obj[axi ? 1 : 0]);
}

static const ESP32P4GdmaInterposerOps bs_path_ops;
static const ESP32P4GdmaInterposerOps bs_loop_rx_ops;

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * Put each core in the stream of the peripheral its selection names,
 * while it is enabled; in loopback the TX core also holds the RX path.
 */
static void bs_update_paths(ESP32P4BitScramblerState *s)
{
    for (int i = 0; i < 2; i++) {
        ESP32P4BsCore *c = &s->core[i];
        unsigned sel = extract32(s->peri_sel, i == TX ? 4 : 0, 4);
        ESP32P4GdmaState *gdma = NULL;
        unsigned periph = 0;
        bool loop_rx = false;
        int dir = i == TX ? OUT : IN;

        if ((c->ctrl & CTRL_ENA) && sel < ESP32P4_BS_PERIPHS &&
            !(i == RX && (s->sys & SYS_LOOP_MODE))) {
            gdma = bs_gdma(s, bs_periph[sel].axi);
            periph = bs_periph[sel].periph;
            loop_rx = core_loop(c);
        }
        if (gdma == c->gdma && periph == c->periph &&
            loop_rx == c->loop_rx) {
            continue;
        }
        if (c->gdma) {
            esp32p4_gdma_interpose(c->gdma, dir, c->periph, NULL, NULL);
            if (c->loop_rx) {
                esp32p4_gdma_interpose(c->gdma, IN, c->periph, NULL, NULL);
            }
            /* The peripheral takes its path back. */
            esp32p4_gdma_periph_ready(c->gdma, c->periph, dir);
        }
        c->gdma = gdma;
        c->periph = periph;
        c->loop_rx = loop_rx;
        if (gdma) {
            esp32p4_gdma_interpose(gdma, dir, periph, &bs_path_ops, c);
            if (loop_rx) {
                esp32p4_gdma_interpose(gdma, IN, periph, &bs_loop_rx_ops, c);
            }
        }
        core_kick(c);
    }
}

/* Moving bytes in and out */

/*
 * A frame's end that came without a byte: it goes with the last byte in
 * the input FIFO, or, when that has been read, as a mark of its own.
 */
static void core_mark_end(ESP32P4BsCore *c, uint8_t mark)
{
    ESP32P4BsFifo *f = &c->fifo_in;

    if (f->num && !(fifo_peek_flags(f, f->num - 1) & F_EOF)) {
        f->flags[(f->head + f->num - 1) % ESP32P4_BS_FIFO] |= mark;
    } else if (fifo_room(f)) {
        fifo_put(f, 0, mark | F_NODATA);
    }
}

/* The TX core takes what the channel has, while it takes input. */
static void core_fill(ESP32P4BsCore *c)
{
    uint8_t buf[ESP32P4_BS_FIFO];
    size_t n;
    bool eof;

    if (c->dir != TX || !c->gdma || !core_takes_input(c)) {
        return;
    }
    while (fifo_room(&c->fifo_in)) {
        n = esp32p4_gdma_chan_pull(c->gdma, c->periph, buf,
                                   fifo_room(&c->fifo_in), &eof);
        for (size_t i = 0; i < n; i++) {
            fifo_put(&c->fifo_in, buf[i], eof && i == n - 1 ? F_EOF : 0);
        }
        if (eof && !n) {
            core_mark_end(c, F_EOF);
        }
        if (!n && !eof) {
            break;
        }
    }
}

/*
 * Give the output FIFO's bytes to the RX channel: the RX core's always,
 * the TX core's in loopback. A frame's end goes with its last byte.
 */
static void core_drain(ESP32P4BsCore *c)
{
    ESP32P4BsFifo *f = &c->fifo_out;
    uint8_t buf[ESP32P4_BS_FIFO], byte, mark;
    uint32_t n, entries, taken;
    unsigned flags;

    if (!c->gdma || (c->dir == TX && !core_loop(c))) {
        return;
    }
    while (f->num) {
        /* The entries up to and including the next frame's end. */
        n = 0;
        flags = 0;
        for (entries = 0; entries < f->num;) {
            mark = fifo_peek_flags(f, entries);
            if (!(mark & F_NODATA)) {
                buf[n++] = f->data[(f->head + entries) % ESP32P4_BS_FIFO];
            }
            entries++;
            if (mark & F_EOF) {
                flags = mark & F_ERR ? ESP32P4_GDMA_ERR_EOF
                                     : ESP32P4_GDMA_EOF;
                break;
            }
        }
        if (!esp32p4_gdma_chan_rx_room(c->gdma, c->periph)) {
            break;
        }
        taken = esp32p4_gdma_chan_push(c->gdma, c->periph, buf, n, flags);
        if (taken < n) {
            /* The rest, and the frame's end, wait for room. */
            for (uint32_t k = 0; k < taken; k++) {
                fifo_take(f, &byte, &mark);
            }
            break;
        }
        for (uint32_t k = 0; k < entries; k++) {
            fifo_take(f, &byte, &mark);
        }
    }
    core_check_drained(c);
}

/* Reading and writing */

/* Whether @n bytes can be read now. */
static bool core_can_read(ESP32P4BsCore *c, uint32_t n)
{
    bool dummy = c->ctrl & CTRL_RD_DUMMY;
    uint32_t have = 0;

    if (c->eof_seen && dummy) {
        return true;
    }
    for (uint32_t i = 0; i < c->fifo_in.num && have < n; i++) {
        uint8_t mark = fifo_peek_flags(&c->fifo_in, i);

        if (!(mark & F_NODATA)) {
            have++;
        }
        if ((mark & F_EOF) && dummy) {
            return true;
        }
    }
    return have >= n;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void core_read(ESP32P4BsCore *c, uint32_t n)
{
    bool counting = c->eof_seen;
    uint8_t byte, mark;

    for (uint32_t i = 0; i < n; i++) {
        byte = 0;
        while (!(c->eof_seen && (c->ctrl & CTRL_RD_DUMMY))) {
            fifo_take(&c->fifo_in, &byte, &mark);
            if (mark & F_EOF) {
                if (c->eof_seen) {
                    c->eof_overload = true;
                }
                c->eof_seen = true;
                c->eof_err = mark & F_ERR;
            }
            if (!(mark & F_NODATA)) {
                break;
            }
            byte = 0;
        }
        c->in = (c->in >> 8) | ((uint64_t)byte << 56);
    }
    if (counting && !(c->ctrl & CTRL_EOF_MODE)) {
        c->eof_count += n;
    }
}

static void core_write(ESP32P4BsCore *c, uint32_t out, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        fifo_put(&c->fifo_out, out >> (8 * i), 0);
    }
    if (c->eof_seen && (c->ctrl & CTRL_EOF_MODE)) {
        c->eof_count += n;
    }
}

static bool core_stream_done(ESP32P4BsCore *c)
{
    return c->eof_seen &&
           (uint64_t)c->eof_count * 8 >= (c->tailing_bits & TAILING_MASK);
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * The stream's end: the last byte written carries the frame's end on,
 * and the core stops.
 */
static void core_end(ESP32P4BsCore *c)
{
    ESP32P4BsFifo *f = &c->fifo_out;
    uint8_t mark = F_EOF | (c->eof_err ? F_ERR : 0);

    if (f->num) {
        f->flags[(f->head + f->num - 1) % ESP32P4_BS_FIFO] |= mark;
    } else {
        fifo_put(f, 0, mark | F_NODATA);
    }
    c->eof_get_cnt = MIN(c->eof_count, STATE_EOF_CNT_MAX);
    c->state = BS_END;
    timer_del(c->timer);
}

/* Executing */

static uint32_t inst_field(ESP32P4BsCore *c, unsigned ip, unsigned lo,
                           unsigned n)
{
    const uint32_t *w = c->inst[ip];
    uint64_t v = w[lo / 32];

    if (lo / 32 + 1 < ESP32P4_BS_INST_WORDS) {
        v |= (uint64_t)w[lo / 32 + 1] << 32;
    }
    return extract64(v, lo % 32, n);
}

static uint32_t inst_bits(ESP32P4BsCore *c, unsigned lo, unsigned n)
{
    return inst_field(c, c->ip, lo, n);
}

static unsigned lut_width(ESP32P4BsCore *c)
{
    switch (LUT_MODE(c->lut_cfg0)) {
    case 0:
        return 8;
    case 1:
        return 16;
    default:
        return 32;
    }
}

static uint32_t lut_read(ESP32P4BsCore *c, uint32_t idx, unsigned width)
{
    uint32_t v = 0;

    for (unsigned b = 0; b < width / 8; b++) {
        v |= (uint32_t)c->lut[(idx * (width / 8) + b) %
                              ESP32P4_BS_LUT_BYTES] << (8 * b);
    }
    return v;
}

typedef struct BsRoute {
    bool rel;
    bool ctrsel;
    bool lutsel;
    unsigned lutw;
    uint32_t lutv;
} BsRoute;

/* [spec:nuos:req:emu.esp32p4.parlio] */
static bool route_bit(ESP32P4BsCore *c, const BsRoute *r, unsigned sel)
{
    uint8_t b, o;

    if (r->rel && sel < 64) {
        unsigned a = c->ctr[0];

        if (!r->ctrsel) {
            sel = (sel + a) & 63;
        } else if (sel < 32) {
            sel = (sel + a) & 31;
        } else {
            sel = ((sel + a) & 31) + 32;
        }
    }
    if (sel < 32) {
        return extract64(c->in, sel, 1);
    }
    if (sel < 64) {
        if (r->lutsel && sel >= 64 - r->lutw) {
            return extract32(r->lutv, sel - (64 - r->lutw), 1);
        }
        if (r->ctrsel) {
            return extract32(c->ctr[sel < 48 ? 0 : 1], (sel - 32) % 16, 1);
        }
        return extract64(c->in, sel, 1);
    }
    if (sel < 88) {
        /* Counter B's bytes against the previous output's bytes. */
        unsigned k = sel - 64;

        b = c->ctr[1] >> (8 * (k / 12));
        o = c->last >> (8 * ((k / 3) % 4));
        return k % 3 == 0 ? b <= o : k % 3 == 1 ? b > o : b == o;
    }
    if (sel < 94) {
        /* Counter B against the previous output's halves. */
        unsigned k = sel - 88;
        uint16_t h = c->last >> (16 * (k / 3));

        return k % 3 == 0 ? c->ctr[1] <= h :
               k % 3 == 1 ? c->ctr[1] > h : c->ctr[1] == h;
    }
    if (sel < 96) {
        return sel == 95;
    }
    return extract32(c->last, sel - 96, 1);
}

static void ctr_write(ESP32P4BsCore *c, unsigned n, bool h, bool l,
                      uint16_t v)
{
    uint16_t mask = (h ? 0xff00 : 0) | (l ? 0x00ff : 0);

    c->ctr[n] = (c->ctr[n] & ~mask) | (v & mask);
}

static const uint8_t bs_rw_bytes[4] = { 0, 1, 2, 4 };

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * One cycle: the prefetch's read, or an instruction. False if it waits.
 */
static bool core_step(ESP32P4BsCore *c)
{
    bool fetch = c->ctrl & CTRL_FETCH_MODE;
    uint32_t rd, wr, op, out = 0;
    unsigned nip;
    BsRoute r;

    if (c->prefetch) {
        rd = MIN(4, c->prefetch);
        if (!core_can_read(c, rd)) {
            return false;
        }
        core_read(c, rd);
        c->prefetch -= rd;
        if (core_stream_done(c)) {
            core_end(c);
        }
        return true;
    }

    rd = bs_rw_bytes[inst_bits(c, I_READ_IN, 2)];
    wr = bs_rw_bytes[inst_bits(c, I_WR_OUT, 2)];
    if (fetch && c->fetched) {
        rd = 0;
    }
    if ((rd && !core_can_read(c, rd)) || fifo_room(&c->fifo_out) < wr) {
        return false;
    }
    if (fetch && rd) {
        core_read(c, rd);
        if (core_stream_done(c)) {
            core_end(c);
            return true;
        }
    }
    r.rel = inst_bits(c, I_MUX_REL, 1);
    r.ctrsel = inst_bits(c, I_CTR_SRC_SEL, 1);
    r.lutsel = inst_bits(c, I_LUT_SRC_SEL, 1);
    r.lutw = lut_width(c);
    r.lutv = lut_read(c, extract32(c->last, 16,
                                   r.lutw == 32 ? 9 : r.lutw == 16 ? 10 : 11),
                      r.lutw);
    for (unsigned n = 0; n < 32; n++) {
        out |= (uint32_t)route_bit(c, &r, inst_bits(c, I_MUX_SEL(n), 7)) << n;
    }

    op = inst_bits(c, I_OPCODE, 26);
    nip = c->ip + 1;
    if (op & OP_LOOP) {
        unsigned n = OP_C(op);
        bool cond = (c->ctrl & CTRL_COND_MODE)
                    ? c->ctr[n] != OP_END_VAL(op)
                    : c->ctr[n] < OP_END_VAL(op);

        if (cond) {
            nip = OP_TGT(op);
            c->ctr[n] += OP_LOOP_ADD(op);
        } else {
            c->ctr[n] = 0;
        }
    } else {
        switch (OP_KIND(op)) {
        case OP_ADD:
            ctr_write(c, OP_C(op), OP_H(op), OP_L(op),
                      c->ctr[OP_C(op)] + OP_IMM(op));
            break;
        case OP_IF:
        case OP_IFN:
            if (route_bit(c, &r, OP_SRC(op)) == (OP_KIND(op) == OP_IF)) {
                nip = OP_TGT(op);
            }
            break;
        case OP_LDCTD:
            ctr_write(c, OP_C(op), OP_H(op), OP_L(op), OP_IMM(op));
            break;
        case OP_LDCTI:
            ctr_write(c, OP_C(op), OP_H(op), OP_L(op), out >> 16);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s core: instruction %u has "
                          "no opcode 0x%07x\n", TYPE_ESP32P4_BITSCRAMBLER,
                          bs_name[c->dir], c->ip, op);
            break;
        }
    }

    c->last = out;
    c->ip = nip % ESP32P4_BS_INSTS;
    c->fetched = false;
    if (wr) {
        core_write(c, out, wr);
        if (core_stream_done(c)) {
            core_end(c);
            return true;
        }
    }
    if (!fetch && rd) {
        core_read(c, rd);
    } else if (fetch) {
        /* The read ahead, for the next instruction, if it can be had. */
        rd = bs_rw_bytes[inst_bits(c, I_READ_IN, 2)];
        if (rd && core_can_read(c, rd)) {
            core_read(c, rd);
            c->fetched = true;
        }
    }
    if (core_stream_done(c)) {
        core_end(c);
    }
    return true;
}

/*
 * Move what can move between the core and its path, and tell the
 * peripheral of output that appeared in an empty output FIFO or of room
 * that appeared in a full input FIFO since @out_had and @in_full.
 */
static void core_exchange(ESP32P4BsCore *c, bool out_had, bool in_full)
{
    core_fill(c);
    core_drain(c);
    if (!out_had && c->fifo_out.num) {
        core_tell_periph_data(c);
    }
    if (in_full && c->fifo_in.num < ESP32P4_BS_FIFO) {
        core_tell_periph_room(c);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * A cycle, at esp32_line_time().
 */
static void core_run(void *opaque)
{
    ESP32P4BsCore *c = opaque;
    bool out_had = c->fifo_out.num;
    bool in_full = c->fifo_in.num == ESP32P4_BS_FIFO;
    bool progress;

    core_fill(c);
    progress = core_active(c) && core_step(c);
    core_exchange(c, out_had, in_full);
    if (progress && core_active(c)) {
        core_arm(c, c->due_cycle + 1);
    }
}

/* The cycle falls due: run it at its time, which a late timer has passed. */
static void core_timer_cb(void *opaque)
{
    ESP32P4BsCore *c = opaque;

    esp32_line_run_at(core_due_ns(c), core_run, c);
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * Run the cycles that fell due by @when, in order, each at its own time
 * after the channel's rounds due by then.
 */
static void core_catch_up(ESP32P4BsCore *c, int64_t when)
{
    int64_t due;

    while (timer_pending(c->timer) && (due = core_due_ns(c)) <= when) {
        timer_del(c->timer);
        if (c->gdma) {
            esp32p4_gdma_chan_sync(c->gdma, due);
        }
        esp32_line_run_at(due, core_run, c);
    }
}

static void bs_catch_up(ESP32P4BitScramblerState *s, int64_t when)
{
    for (int i = 0; i < 2; i++) {
        core_catch_up(&s->core[i], when);
    }
}

/*
 * The clock changes: the cycles due before it run at the old rate; its
 * cycles are counted afresh from the change, a step under way taking the
 * first edge at the new rate.
 */
static void core_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4BsCore *c = opaque;
    int64_t now = esp32_line_time();
    bool pending;

    if (event == ClockPreUpdate) {
        core_catch_up(c, now);
        return;
    }
    pending = timer_pending(c->timer);
    esp32p4_clk_count_start(&c->edges, c->clk, now);
    if (!clock_is_enabled(c->clk)) {
        timer_del(c->timer);
    } else if (pending) {
        core_arm(c, 1);
    } else {
        core_kick(c);
    }
}

/* The interposer's ops: the peripheral's side of the path */

/* [spec:nuos:req:emu.esp32p4.parlio] */
static size_t bs_path_pull(void *opaque, uint8_t *buf, size_t len, bool *eof)
{
    ESP32P4BsCore *c = opaque;
    ESP32P4BsFifo *f = &c->fifo_out;
    uint8_t byte, mark;
    size_t n = 0;

    *eof = false;
    if (core_loop(c)) {
        return 0;
    }
    while (f->num && n < len) {
        fifo_take(f, &byte, &mark);
        if (!(mark & F_NODATA)) {
            buf[n++] = byte;
        }
        if (mark & F_EOF) {
            *eof = true;
            break;
        }
    }
    core_check_drained(c);
    core_kick(c);
    return n;
}

static size_t bs_path_tx_pending(void *opaque)
{
    ESP32P4BsCore *c = opaque;

    return core_loop(c) ? 0 : c->fifo_out.num;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static size_t bs_path_push(void *opaque, const uint8_t *buf, size_t len,
                           unsigned flags)
{
    ESP32P4BsCore *c = opaque;
    uint8_t mark = (flags & ESP32P4_GDMA_EOF ? F_EOF : 0) |
                   (flags & ESP32P4_GDMA_ERR_EOF ? F_EOF | F_ERR : 0);
    size_t n;

    if (!core_takes_input(c)) {
        return 0;
    }
    if (!len) {
        if (mark) {
            core_mark_end(c, mark);
        }
        core_kick(c);
        return 0;
    }
    n = MIN(len, fifo_room(&c->fifo_in));
    for (size_t i = 0; i < n; i++) {
        fifo_put(&c->fifo_in, buf[i], i == len - 1 ? mark : 0);
    }
    core_kick(c);
    return n;
}

static size_t bs_path_rx_room(void *opaque)
{
    ESP32P4BsCore *c = opaque;

    return core_takes_input(c) ? fifo_room(&c->fifo_in) : 0;
}

static void bs_path_chan_ready(void *opaque, int dir)
{
    ESP32P4BsCore *c = opaque;

    core_exchange(c, c->fifo_out.num, false);
    core_kick(c);
}

static void bs_path_sync(void *opaque, int64_t when)
{
    core_catch_up(opaque, when);
}

static const ESP32P4GdmaInterposerOps bs_path_ops = {
    .pull = bs_path_pull,
    .tx_pending = bs_path_tx_pending,
    .push = bs_path_push,
    .rx_room = bs_path_rx_room,
    .chan_ready = bs_path_chan_ready,
    .sync = bs_path_sync,
};

/* In loopback the peripheral's RX path is cut off. */
static size_t bs_loop_rx_push(void *opaque, const uint8_t *buf, size_t len,
                              unsigned flags)
{
    return 0;
}

static size_t bs_loop_rx_room(void *opaque)
{
    return 0;
}

static const ESP32P4GdmaInterposerOps bs_loop_rx_ops = {
    .pull = bs_path_pull,
    .tx_pending = bs_path_tx_pending,
    .push = bs_loop_rx_push,
    .rx_room = bs_loop_rx_room,
    .chan_ready = bs_path_chan_ready,
    .sync = bs_path_sync,
};

/* Registers */

static uint32_t core_state_reg(ESP32P4BsCore *c)
{
    uint32_t v = 0;

    switch (c->state) {
    case BS_IDLE:
        v |= STATE_IN_IDLE;
        break;
    case BS_RUN:
        v |= (c->ctrl & CTRL_PAUSE) ? STATE_IN_PAUSE : STATE_IN_RUN;
        break;
    case BS_WAIT:
        v |= STATE_IN_WAIT;
        break;
    case BS_END:
        v |= c->fifo_out.num ? STATE_IN_WAIT : STATE_IN_IDLE;
        break;
    }
    if (c->dir == TX ? c->fifo_out.num != 0
                     : c->fifo_in.num == ESP32P4_BS_FIFO) {
        v |= STATE_FIFO;
    }
    v |= c->eof_get_cnt << STATE_EOF_CNT_SHIFT;
    if (c->eof_overload) {
        v |= STATE_EOF_OVERLOAD;
    }
    return v;
}

static uint32_t *inst_word(ESP32P4BsCore *c)
{
    unsigned pos = INST_POS(c->inst_cfg0);

    if (pos >= ESP32P4_BS_INST_WORDS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s INST_POS %u is past the "
                      "instruction's nine words\n", TYPE_ESP32P4_BITSCRAMBLER,
                      bs_name[c->dir], pos);
        return NULL;
    }
    return &c->inst[INST_IDX(c->inst_cfg0)][pos];
}

static uint64_t bs_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4BitScramblerState *s = opaque;
    ESP32P4BsCore *c;
    uint32_t *w;

    bs_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    switch (addr) {
    case A_INST_CFG0(TX):
    case A_INST_CFG0(RX):
        return s->core[addr == A_INST_CFG0(RX)].inst_cfg0;
    case A_INST_CFG1(TX):
    case A_INST_CFG1(RX):
        w = inst_word(&s->core[addr == A_INST_CFG1(RX)]);
        return w ? *w : 0;
    case A_LUT_CFG0(TX):
    case A_LUT_CFG0(RX):
        return s->core[addr == A_LUT_CFG0(RX)].lut_cfg0;
    case A_LUT_CFG1(TX):
    case A_LUT_CFG1(RX):
        c = &s->core[addr == A_LUT_CFG1(RX)];
        return lut_read(c, LUT_IDX(c->lut_cfg0), lut_width(c));
    case A_TAILING_BITS(TX):
    case A_TAILING_BITS(RX):
        return s->core[addr == A_TAILING_BITS(RX)].tailing_bits;
    case A_CTRL(TX):
    case A_CTRL(RX):
        return s->core[addr == A_CTRL(RX)].ctrl;
    case A_STATE(TX):
    case A_STATE(RX):
        return core_state_reg(&s->core[addr == A_STATE(RX)]);
    case A_SYS:
        return s->sys;
    case A_VERSION:
        return s->version;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_BITSCRAMBLER, addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void ctrl_write(ESP32P4BsCore *c, uint32_t v)
{
    uint32_t old = c->ctrl;

    c->ctrl = v & CTRL_MASK;
    if (v & CTRL_FIFO_RST) {
        core_flush(c);
        core_check_drained(c);
    }
    if ((c->ctrl & CTRL_HALT) && c->state != BS_IDLE) {
        core_halt(c);
    } else if ((old & CTRL_HALT) && !(c->ctrl & CTRL_HALT) &&
               c->state == BS_IDLE) {
        core_start(c);
    }
    if ((old ^ c->ctrl) & CTRL_ENA) {
        bs_update_paths(c->s);
    }
    core_exchange(c, c->fifo_out.num, false);
    core_kick(c);
}

static void bs_write(void *opaque, hwaddr addr, uint64_t value,
                     unsigned size)
{
    ESP32P4BitScramblerState *s = opaque;
    uint32_t v = value;
    ESP32P4BsCore *c;
    uint32_t *w;

    bs_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    switch (addr) {
    case A_INST_CFG0(TX):
    case A_INST_CFG0(RX):
        s->core[addr == A_INST_CFG0(RX)].inst_cfg0 = v & INST_CFG0_MASK;
        break;
    case A_INST_CFG1(TX):
    case A_INST_CFG1(RX):
        c = &s->core[addr == A_INST_CFG1(RX)];
        w = inst_word(c);
        if (w) {
            /* The ninth word holds the 257th bit only. */
            *w = INST_POS(c->inst_cfg0) == ESP32P4_BS_INST_WORDS - 1
                 ? v & 1 : v;
        }
        break;
    case A_LUT_CFG0(TX):
    case A_LUT_CFG0(RX):
        c = &s->core[addr == A_LUT_CFG0(RX)];
        c->lut_cfg0 = v & LUT_CFG0_MASK;
        if (LUT_MODE(c->lut_cfg0) == 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s LUT_MODE 3 is reserved; "
                          "taken as 32 bits\n", TYPE_ESP32P4_BITSCRAMBLER,
                          bs_name[c->dir]);
        }
        break;
    case A_LUT_CFG1(TX):
    case A_LUT_CFG1(RX): {
        unsigned width;
        uint32_t idx;

        c = &s->core[addr == A_LUT_CFG1(RX)];
        width = lut_width(c) / 8;
        idx = LUT_IDX(c->lut_cfg0);
        for (unsigned b = 0; b < width; b++) {
            c->lut[(idx * width + b) % ESP32P4_BS_LUT_BYTES] = v >> (8 * b);
        }
        break;
    }
    case A_TAILING_BITS(TX):
    case A_TAILING_BITS(RX):
        s->core[addr == A_TAILING_BITS(RX)].tailing_bits = v & TAILING_MASK;
        break;
    case A_CTRL(TX):
    case A_CTRL(RX):
        ctrl_write(&s->core[addr == A_CTRL(RX)], v);
        break;
    case A_STATE(TX):
    case A_STATE(RX):
        c = &s->core[addr == A_STATE(RX)];
        if (v & STATE_EOF_TRACE_CLR) {
            c->eof_overload = false;
            c->eof_get_cnt = 0;
        }
        break;
    case A_SYS: {
        bool loop_changed = (s->sys ^ v) & SYS_LOOP_MODE;

        s->sys = v & SYS_MASK;
        if (loop_changed) {
            bs_update_paths(s);
            for (int i = 0; i < 2; i++) {
                core_exchange(&s->core[i], s->core[i].fifo_out.num, false);
                core_kick(&s->core[i]);
            }
        }
        break;
    }
    case A_VERSION:
        s->version = v & VERSION_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_BITSCRAMBLER, addr);
        break;
    }
}

static const MemoryRegionOps bs_ops = {
    .read = bs_read,
    .write = bs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Inputs */

static void bs_peri_sel(void *opaque, int n, int level)
{
    ESP32P4BitScramblerState *s = opaque;

    s->peri_sel = deposit32(s->peri_sel, n, 1, level);
    bs_update_paths(s);
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void bs_core_reset_in(void *opaque, int n, int level)
{
    ESP32P4BitScramblerState *s = opaque;
    ESP32P4BsCore *c = &s->core[n];

    if (level == c->reset_held) {
        return;
    }
    core_catch_up(c, esp32_line_time());
    c->reset_held = level;
    core_reset(c);
    bs_update_paths(s);
}

/* Device */

static void bs_reset_hold(Object *obj, ResetType type)
{
    ESP32P4BitScramblerState *s = ESP32P4_BITSCRAMBLER(obj);

    for (int i = 0; i < 2; i++) {
        core_reset(&s->core[i]);
    }
    s->sys = 0;
    s->version = VERSION_RESET;
}

static void bs_reset_exit(Object *obj, ResetType type)
{
    bs_update_paths(ESP32P4_BITSCRAMBLER(obj));
}

static void bs_init(Object *obj)
{
    ESP32P4BitScramblerState *s = ESP32P4_BITSCRAMBLER(obj);
    static const char *const clk_names[2] = { "tx-clk", "rx-clk" };

    memory_region_init_io(&s->iomem, obj, &bs_ops, s, "esp32p4.bitscrambler",
                          ESP32P4_BITSCRAMBLER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(DEVICE(obj), bs_peri_sel, ESP32P4_BS_PERI_SEL, 8);
    qdev_init_gpio_in_named(DEVICE(obj), bs_core_reset_in,
                            ESP32P4_BS_CORE_RESET, 2);
    s->peri_sel = PERI_SEL_RESET;
    for (int i = 0; i < 2; i++) {
        ESP32P4BsCore *c = &s->core[i];

        c->s = s;
        c->dir = i;
        c->clk = qdev_init_clock_in(DEVICE(obj), clk_names[i],
                                    core_clk_update, c,
                                    ClockPreUpdate | ClockUpdate);
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, core_timer_cb, c);
    }
}

static void bs_realize(DeviceState *dev, Error **errp)
{
    ESP32P4BitScramblerState *s = ESP32P4_BITSCRAMBLER(dev);

    if (!s->gdma_obj[0] || !s->gdma_obj[1]) {
        error_setg(errp, "%s: the 'ahb-gdma' and 'axi-gdma' links must be "
                   "set", TYPE_ESP32P4_BITSCRAMBLER);
    }
}

static void bs_finalize(Object *obj)
{
    ESP32P4BitScramblerState *s = ESP32P4_BITSCRAMBLER(obj);

    for (int i = 0; i < 2; i++) {
        timer_free(s->core[i].timer);
    }
}

/*
 * The paths are wiring the migration does not carry: drop the record of
 * them and put the cores back on the paths their registers name.
 */
static int bs_post_load(void *opaque, int version_id)
{
    ESP32P4BitScramblerState *s = opaque;

    for (int i = 0; i < 2; i++) {
        s->core[i].gdma = NULL;
        s->core[i].loop_rx = false;
    }
    bs_update_paths(s);
    return 0;
}

static const Property bs_properties[] = {
    DEFINE_PROP_LINK("ahb-gdma", ESP32P4BitScramblerState, gdma_obj[0],
                     TYPE_ESP32P4_GDMA, Object *),
    DEFINE_PROP_LINK("axi-gdma", ESP32P4BitScramblerState, gdma_obj[1],
                     TYPE_ESP32P4_GDMA, Object *),
};

static const VMStateDescription vmstate_bs_fifo = {
    .name = "esp32p4-bitscrambler-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, ESP32P4BsFifo, ESP32P4_BS_FIFO),
        VMSTATE_UINT8_ARRAY(flags, ESP32P4BsFifo, ESP32P4_BS_FIFO),
        VMSTATE_UINT32(head, ESP32P4BsFifo),
        VMSTATE_UINT32(num, ESP32P4BsFifo),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_bs_core = {
    .name = "esp32p4-bitscrambler-core",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(inst_cfg0, ESP32P4BsCore),
        VMSTATE_UINT32(lut_cfg0, ESP32P4BsCore),
        VMSTATE_UINT32(tailing_bits, ESP32P4BsCore),
        VMSTATE_UINT32(ctrl, ESP32P4BsCore),
        VMSTATE_BOOL(eof_overload, ESP32P4BsCore),
        VMSTATE_UINT32(eof_get_cnt, ESP32P4BsCore),
        VMSTATE_UINT32_2DARRAY(inst, ESP32P4BsCore, ESP32P4_BS_INSTS,
                               ESP32P4_BS_INST_WORDS),
        VMSTATE_UINT8_ARRAY(lut, ESP32P4BsCore, ESP32P4_BS_LUT_BYTES),
        VMSTATE_UINT8(state, ESP32P4BsCore),
        VMSTATE_BOOL(reset_held, ESP32P4BsCore),
        VMSTATE_UINT8(prefetch, ESP32P4BsCore),
        VMSTATE_UINT8(ip, ESP32P4BsCore),
        VMSTATE_BOOL(fetched, ESP32P4BsCore),
        VMSTATE_UINT64(in, ESP32P4BsCore),
        VMSTATE_UINT32(last, ESP32P4BsCore),
        VMSTATE_UINT16_ARRAY(ctr, ESP32P4BsCore, 2),
        VMSTATE_BOOL(eof_seen, ESP32P4BsCore),
        VMSTATE_BOOL(eof_err, ESP32P4BsCore),
        VMSTATE_UINT32(eof_count, ESP32P4BsCore),
        VMSTATE_STRUCT(fifo_in, ESP32P4BsCore, 1, vmstate_bs_fifo,
                       ESP32P4BsFifo),
        VMSTATE_STRUCT(fifo_out, ESP32P4BsCore, 1, vmstate_bs_fifo,
                       ESP32P4BsFifo),
        VMSTATE_CLOCK(clk, ESP32P4BsCore),
        VMSTATE_ESP32P4_CLK_COUNT(edges, ESP32P4BsCore),
        VMSTATE_UINT64(due_cycle, ESP32P4BsCore),
        VMSTATE_TIMER_PTR(timer, ESP32P4BsCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_bs = {
    .name = TYPE_ESP32P4_BITSCRAMBLER,
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = bs_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, ESP32P4BitScramblerState, 2, 1,
                             vmstate_bs_core, ESP32P4BsCore),
        VMSTATE_UINT32(sys, ESP32P4BitScramblerState),
        VMSTATE_UINT32(version, ESP32P4BitScramblerState),
        VMSTATE_UINT32(peri_sel, ESP32P4BitScramblerState),
        VMSTATE_END_OF_LIST()
    },
};

static void bs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = bs_realize;
    dc->vmsd = &vmstate_bs;
    device_class_set_props(dc, bs_properties);
    rc->phases.hold = bs_reset_hold;
    rc->phases.exit = bs_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static const TypeInfo bs_types[] = {
    {
        .name           = TYPE_ESP32P4_BITSCRAMBLER,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(ESP32P4BitScramblerState),
        .instance_init  = bs_init,
        .instance_finalize = bs_finalize,
        .class_init     = bs_class_init,
    },
};

DEFINE_TYPES(bs_types)
