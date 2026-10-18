/*
 * ESP32 RMT, the remote control (pulse train) peripheral
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Remote Control Peripheral (RMT)", with the field
 * layout and reset values of ESP-IDF's soc/esp32 rmt_reg.h and
 * rmt_struct.h, and the use ESP-IDF's RMT driver and hal/rmt_ll.h make of
 * them on the ESP32.
 *
 * Eight channels share 512 words of RAM, eight 64-word blocks. Channel n
 * owns block n and, with MEM_SIZE > 1, the blocks after it, wrapping at
 * the end of the RAM. Each word holds two 16-bit entries, low half first:
 * a level (bit 15) and a period in channel clock ticks (bits 14:0); a
 * period of 0 marks the end. Each channel's base clock is APB_CLK or
 * REF_TICK (REF_ALWAYS_ON), divided by DIV_CNT (0 meaning 256) into the
 * channel clock. In continuous mode (TX_CONTI_MODE) the transmitter
 * starts the data again after an end marker, one channel clock tick at
 * the idle level between passes (TRM 30.2.2: "there will be an idle level
 * lasting one clk_div cycle between N and N+1 transmissions"), so that
 * entries N ticks long repeat every N + 1.
 *
 * Timing runs on QEMU_CLOCK_VIRTUAL; there are no per-tick events. The
 * channels run on whenever the block is brought up to a moment (a register
 * or RAM access, an input change, software sampling the pads, a clock
 * change, the block's timer): each runs through every event due since,
 * each at its own virtual time, so that what software reads is exact at
 * the moment it reads it. The block's one timer is set only for the
 * moments something outside the block sees, and only when they change
 * (rmt_schedule()): an edge of an output that reaches a watched pad (one
 * whose level something but software sampling the pads sees:
 * esp32_sig_out_watched()), and the raising of an interrupt INT_ENA lets
 * out while INT_RAW does not already hold it, found by running a copy of
 * the channel ahead. An output nothing watches has no edge anyone sees:
 * it is driven with its level of the moment whenever the block is brought
 * up to date, its carrier worked out in closed form and the passes of a
 * continuous transmission taken whole once they repeat, so that it costs
 * the host nothing however fast it switches. A register access that finds
 * the timer's moment unchanged leaves the timer alone: setting it again
 * would stop the CPU's run under icount for nothing.
 *
 * Times are those of the signal lines (hw/gpio/esp32_line.h): an output
 * edge is stamped with its time on the GPIO matrix signal, and an input
 * changes at the time its change is stamped with, so a channel receiving
 * another block's output through the matrix, or another channel's,
 * measures the transmitted durations exactly. The block is a line source:
 * a block that samples the pads brings the transmitters up to the moment
 * it samples at, and a register access, the block's timer or a clock
 * change, while a receiver is enabled, first brings every other source up
 * to the moment it acts at, so that an edge sent late (the host running
 * behind virtual time without icount) still reaches the receiver before
 * it records the time after it.
 *
 * Behaviour the TRM leaves open, as modelled:
 * - TX_START starts the transmitter when written as 1 while it is idle,
 *   and reads as 1 while it is transmitting. ESP-IDF sets it with
 *   read-modify-writes and never clears it, which only works if the bit
 *   does not stay set once a transmission has ended. REF_CNT_RST likewise
 *   acts on a write of 1 and reads as 0. MEM_RD_RST, MEM_WR_RST and
 *   APB_MEM_RST hold their pointer at the start of the channel's memory
 *   while set.
 * - The transmitter starts on the channel clock tick after TX_START. It
 *   counts the words it fetches; each time the count reaches TX_LIM it
 *   raises TX_THR_EVENT and starts again from 0. At an end marker it
 *   raises TX_END and returns its read pointer to the start of the
 *   channel's memory; in continuous mode it raises TX_END on every pass.
 *   Reaching the end of its memory without MEM_TX_WRAP_EN sets MEM_EMPTY,
 *   raises ERR and stops it without TX_END.
 * - The idle output is IDLE_OUT_LV with IDLE_OUT_EN set, else the level
 *   of the last end marker sent (0 out of reset).
 * - Carrier: an entry whose level equals CARRIER_OUT_LV is sent as the
 *   carrier, high for CARRIER_HIGH and low for CARRIER_LOW cycles of the
 *   base clock (0 meaning 65536), as ESP-IDF programs it. The carrier
 *   starts high where a carried stretch of entries starts and runs on
 *   across consecutive carried entries.
 * - The receiver samples its (filtered) input on channel clock ticks: an
 *   input change becomes an edge on the next tick, and a pulse that
 *   spans no tick is not seen. The first edge after RX_EN starts a frame;
 *   from then on the receiver writes an entry for each level when it
 *   ends. With no edge for more than IDLE_THRES ticks it writes an end
 *   marker with the current level, raises RX_END and waits for the next
 *   frame. A level that lasts longer than an entry can hold (32767
 *   ticks) without going idle is written as entries of 32767 ticks.
 * - The filter takes an input change once the input has held the new
 *   level for FILTER_THRES APB_CLK cycles, always APB_CLK as ESP-IDF
 *   states for the ESP32, so it delays edges by that much and removes
 *   shorter pulses.
 * - The receiver writes only while MEM_OWNER gives the memory to it;
 *   otherwise it sets MEM_OWNER_ERR and raises ERR, and drops the entry.
 *   With its memory full it sets MEM_FULL, and raises ERR for each entry
 *   it then drops. The transmitter does not check MEM_OWNER: ESP-IDF
 *   transmits with it at its reset value, which gives the memory to the
 *   receiver.
 * - The ESP32's receiver has no carrier demodulation (ESP-IDF's
 *   RMT_LL_SUPPORT_RX_DEMODULATION is absent for it), so a carrier is
 *   received as its pulses.
 * - MEM_PD powers the RAM down: its contents are lost, the CPU reads 0
 *   and its writes are dropped, the transmitter reads end markers and the
 *   receiver's writes are dropped.
 * - APB_CONF.FIFO_MASK (the TRM's MEM_ACCESS_EN) selects between direct
 *   access to the RAM at 0x800 and the per-channel FIFO registers at
 *   0x00-0x1c; the other path reads 0 and drops writes. The FIFO uses one
 *   pointer per channel for reads and writes, reported in APB_MEM_ADDR,
 *   and sets APB_MEM_RD_ERR or APB_MEM_WR_ERR at the end of the channel's
 *   memory.
 * - Changing a channel's divider, its clock source or a clock frequency
 *   while it runs restarts its divider count from that moment.
 * - CLK_EN, described as the register clock's gate, is kept as a plain
 *   bit: ESP-IDF programs the block on the ESP32 with it at its reset
 *   value, 0.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32_rmt.h"
#include "migration/vmstate.h"

#define A_DATA(n)               (0x00 + 4 * (n))
#define A_CONF0(n)              (0x20 + 8 * (n))
#define A_CONF1(n)              (0x24 + 8 * (n))
#define A_STATUS(n)             (0x60 + 4 * (n))
#define A_APB_MEM_ADDR(n)       (0x80 + 4 * (n))
#define A_INT_RAW               0xa0
#define A_INT_ST                0xa4
#define A_INT_ENA               0xa8
#define A_INT_CLR               0xac
#define A_CARRIER_DUTY(n)       (0xb0 + 4 * (n))
#define A_TX_LIM(n)             (0xd0 + 4 * (n))
#define A_APB_CONF              0xf0
#define A_DATE                  0xfc

/* CHnCONF0 */
#define CONF0_DIV_CNT_MASK      0xffu
#define CONF0_IDLE_THRES_SHIFT  8
#define CONF0_IDLE_THRES_MASK   0xffffu
#define CONF0_MEM_SIZE_SHIFT    24
#define CONF0_MEM_SIZE_MASK     0xfu
#define CONF0_CARRIER_EN        (1u << 28)
#define CONF0_CARRIER_OUT_LV    (1u << 29)
/* MEM_PD and CLK_EN exist only in CH0CONF0 */
#define CONF0_MEM_PD            (1u << 30)
#define CONF0_CLK_EN            (1u << 31)
#define CONF0_RESET             0x31100002u

/* CHnCONF1 */
#define CONF1_TX_START          (1u << 0)
#define CONF1_RX_EN             (1u << 1)
#define CONF1_MEM_WR_RST        (1u << 2)
#define CONF1_MEM_RD_RST        (1u << 3)
#define CONF1_APB_MEM_RST       (1u << 4)
#define CONF1_MEM_OWNER         (1u << 5)
#define CONF1_TX_CONTI_MODE     (1u << 6)
#define CONF1_RX_FILTER_EN      (1u << 7)
#define CONF1_FILTER_THRES_SHIFT 8
#define CONF1_FILTER_THRES_MASK 0xffu
#define CONF1_REF_CNT_RST       (1u << 16)
#define CONF1_REF_ALWAYS_ON     (1u << 17)
#define CONF1_IDLE_OUT_LV       (1u << 18)
#define CONF1_IDLE_OUT_EN       (1u << 19)
#define CONF1_MASK              0x000fffffu
#define CONF1_RESET             0x00000f20u

/* CHnSTATUS */
#define STATUS_WADDR_SHIFT      0
#define STATUS_RADDR_SHIFT      12
#define STATUS_ADDR_MASK        0x3ffu
#define STATUS_STATE_SHIFT      24
#define STATUS_MEM_OWNER_ERR    (1u << 27)
#define STATUS_MEM_FULL         (1u << 28)
#define STATUS_MEM_EMPTY        (1u << 29)
#define STATUS_APB_MEM_WR_ERR   (1u << 30)
#define STATUS_APB_MEM_RD_ERR   (1u << 31)
#define STATE_IDLE              0
#define STATE_SEND              1
#define STATE_RECEIVE           3

/* INT_RAW, INT_ST, INT_ENA and INT_CLR */
#define INT_TX_END(n)           (1u << (3 * (n)))
#define INT_RX_END(n)           (1u << (3 * (n) + 1))
#define INT_ERR(n)              (1u << (3 * (n) + 2))
#define INT_TX_THR_EVENT(n)     (1u << (24 + (n)))

#define CARRIER_DUTY_RESET      0x00400040u
#define TX_LIM_MASK             0x1ffu
#define TX_LIM_RESET            0x80u

/* APB_CONF */
#define APB_CONF_FIFO_MASK      (1u << 0)
#define APB_CONF_MEM_TX_WRAP_EN (1u << 1)
#define APB_CONF_MASK           0x3u

#define DATE_RESET              0x16022600u

#define ENTRY_LEVEL             (1u << 15)
#define ENTRY_PERIOD_MASK       0x7fffu

#define NEVER                   INT64_MAX

/* The interrupts channel n's transmitter and its receiver raise */
#define INT_TX_BITS(n)          (INT_TX_END(n) | INT_ERR(n) | \
                                 INT_TX_THR_EVENT(n))
#define INT_RX_BITS(n)          (INT_RX_END(n) | INT_ERR(n))

/*
 * A receiver's look-ahead with no further input goes through its pending
 * filter and sampler decisions, at most two full entries (an idle
 * threshold of up to 65535 ticks against entries of 32767) and the idle
 * end.
 */
#define RX_AHEAD_STEPS          8

static inline bool ch_conf1(Esp32RmtChannel *c, uint32_t bit)
{
    return (c->conf1 & bit) != 0;
}

static uint32_t ch_div(Esp32RmtChannel *c)
{
    uint32_t div = c->conf0 & CONF0_DIV_CNT_MASK;

    return div ? div : 256;
}

static uint32_t ch_mem_words(Esp32RmtChannel *c)
{
    return ((c->conf0 >> CONF0_MEM_SIZE_SHIFT) & CONF0_MEM_SIZE_MASK) *
           ESP32_RMT_BLOCK_WORDS;
}

static uint32_t ch_idle_thres(Esp32RmtChannel *c)
{
    return (c->conf0 >> CONF0_IDLE_THRES_SHIFT) & CONF0_IDLE_THRES_MASK;
}

static uint32_t ch_filter_thres(Esp32RmtChannel *c)
{
    return (c->conf1 >> CONF1_FILTER_THRES_SHIFT) & CONF1_FILTER_THRES_MASK;
}

static bool rmt_mem_pd(Esp32RmtState *s)
{
    return (s->ch[0].conf0 & CONF0_MEM_PD) != 0;
}

/* Index into the RAM of word off of channel n's memory */
static unsigned ram_index(unsigned n, uint32_t off)
{
    return (n * ESP32_RMT_BLOCK_WORDS + off) % ESP32_RMT_RAM_WORDS;
}

/* Period of channel c's base clock, in 2^-32 ns; 0 while it is stopped */
static uint64_t base_period(Esp32RmtState *s, Esp32RmtChannel *c)
{
    if (ch_conf1(c, CONF1_REF_ALWAYS_ON)) {
        return clock_get(s->apb_clk);
    }
    return clock_get(s->ref_tick_clk);
}

/* The base clock cycle channel c is in at time t */
static uint64_t cycle_at(Esp32RmtState *s, Esp32RmtChannel *c, int64_t t)
{
    uint64_t period = base_period(s, c);
    uint64_t lo, hi, d;

    if (!period || t <= c->anchor_ns) {
        return c->anchor_c;
    }
    d = t - c->anchor_ns;
    lo = d << 32;
    hi = d >> 32;
    divu128(&lo, &hi, period);
    return c->anchor_c + lo;
}

/* The first time at which channel c has reached cycle cyc */
static int64_t cycle_time(Esp32RmtState *s, Esp32RmtChannel *c, uint64_t cyc)
{
    uint64_t period = base_period(s, c);
    uint64_t lo, hi, ns;

    if (cyc <= c->anchor_c) {
        return c->anchor_ns;
    }
    if (!period) {
        return NEVER;
    }
    mulu64(&lo, &hi, cyc - c->anchor_c, period);
    if (hi >> 31) {
        return NEVER;
    }
    ns = (hi << 32) | (lo >> 32);
    if (lo & 0xffffffffu) {
        ns++;
    }
    if (ns > (uint64_t)(NEVER - c->anchor_ns)) {
        return NEVER;
    }
    return c->anchor_ns + ns;
}

/* Restart channel c's cycle count from time now, at the cycle it is in. */
static void reanchor(Esp32RmtState *s, Esp32RmtChannel *c, int64_t now)
{
    c->anchor_c = cycle_at(s, c, now);
    c->anchor_ns = now;
}

/* The first channel clock tick of channel c at or after cycle cyc */
static uint64_t tick_ceil(Esp32RmtChannel *c, uint64_t cyc)
{
    uint32_t div = ch_div(c);

    return DIV_ROUND_UP(cyc, div) * div;
}

/*
 * Raise interrupts. A look-ahead (s->ahead), running a copy of a channel
 * to find when it first raises one that is seen, only notes them.
 */
static void raise_int(Esp32RmtState *s, uint32_t bits)
{
    if (s->ahead) {
        s->ahead_raised |= bits;
    } else {
        s->int_raw |= bits;
    }
}

/* The interrupt line, driven only as its level changes */
static void update_irq(Esp32RmtState *s)
{
    bool level = (s->int_raw & s->int_ena) != 0;

    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->irq, level);
    }
}

/*
 * Drive channel n's output to level from time t, stamped with t. An input
 * of this block that changes in response does not run the channels again.
 */
static void set_out(Esp32RmtState *s, unsigned n, uint8_t level, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];
    bool was_running = s->running;

    if (c->out_level == level) {
        return;
    }
    c->out_level = level;
    s->running = true;
    esp32_line_set_at(s->sig_out[n], level, t);
    s->running = was_running;
}

/* [spec:nuos:req:emu.esp32.rmt] The level channel c's output is to have */
static uint8_t tx_out_level(Esp32RmtChannel *c)
{
    if (c->tx_state == ESP32_RMT_TX_ENTRY) {
        return c->tx_carrier ? c->tx_carrier_high : c->tx_level;
    }
    if (ch_conf1(c, CONF1_IDLE_OUT_EN)) {
        return ch_conf1(c, CONF1_IDLE_OUT_LV);
    }
    return c->tx_end_level;
}

static void tx_update_out(Esp32RmtState *s, unsigned n, int64_t t)
{
    set_out(s, n, tx_out_level(&s->ch[n]), t);
}

/*
 * Fetch the word at the read pointer. Past the end of the channel's
 * memory the transmitter stops with MEM_EMPTY and ERR.
 */
static bool tx_fetch(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n)
{
    uint32_t lim = c->tx_lim & TX_LIM_MASK;

    if (c->tx_raddr >= ch_mem_words(c)) {
        c->mem_empty = true;
        c->tx_state = ESP32_RMT_TX_IDLE;
        c->tx_carrier = false;
        raise_int(s, INT_ERR(n));
        return false;
    }
    c->tx_word = rmt_mem_pd(s) ? 0 : s->ram[ram_index(n, c->tx_raddr)];
    c->tx_half = 0;
    if (++c->tx_thr_count == lim) {
        c->tx_thr_count = 0;
        raise_int(s, INT_TX_THR_EVENT(n));
    }
    return true;
}

static uint32_t carrier_cycles(Esp32RmtChannel *c, bool high)
{
    uint32_t v = high ? c->carrier_duty >> 16 : c->carrier_duty & 0xffff;

    return v ? v : 0x10000;
}

static void tx_set_event(Esp32RmtChannel *c)
{
    c->tx_event_c = c->tx_entry_end_c;
    if (c->tx_carrier && c->tx_carrier_end_c < c->tx_event_c) {
        c->tx_event_c = c->tx_carrier_end_c;
    }
}

/*
 * The carrier's toggles before cycle lim, taken at once: high for
 * CARRIER_HIGH and low for CARRIER_LOW cycles from where it started, it
 * keeps its phase however many periods pass. Returns the cycle of the
 * last toggle taken; the carrier is to have one due before lim.
 */
static uint64_t tx_carrier_skip(Esp32RmtChannel *c, uint64_t lim)
{
    uint64_t e0 = c->tx_carrier_end_c;
    uint64_t d1 = carrier_cycles(c, !c->tx_carrier_high);
    uint64_t p = d1 + carrier_cycles(c, c->tx_carrier_high);
    uint64_t span = lim - 1 - e0;
    uint64_t j = span / p;

    /* Toggles fall at e0 + j * p (to the other level) and + d1 (back). */
    if (span % p < d1) {
        c->tx_carrier_high = !c->tx_carrier_high;
        c->tx_carrier_end_c = e0 + j * p + d1;
        return e0 + j * p;
    }
    c->tx_carrier_end_c = e0 + (j + 1) * p;
    return e0 + j * p + d1;
}

/* An end marker with the given level, reached at cycle cyc. */
static void tx_end(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                   uint64_t cyc, uint8_t level)
{
    c->tx_end_level = level;
    c->tx_carrier = false;
    c->tx_raddr = 0;
    c->tx_half = 0;
    raise_int(s, INT_TX_END(n));
    if (ch_conf1(c, CONF1_TX_CONTI_MODE)) {
        /*
         * One channel clock tick at the idle level between passes, as the
         * TRM gives it (30.2.2), before the next pass's first entry.
         */
        c->tx_state = ESP32_RMT_TX_STARTING;
        c->tx_event_c = cyc + ch_div(c);
        c->tx_thr_count = 0;
    } else {
        c->tx_state = ESP32_RMT_TX_IDLE;
    }
}

/* Start sending the current entry at cycle cyc. */
static void tx_begin_entry(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                           uint64_t cyc)
{
    uint32_t e = c->tx_half ? c->tx_word >> 16 : c->tx_word & 0xffff;
    uint32_t period = e & ENTRY_PERIOD_MASK;
    uint8_t level = (e & ENTRY_LEVEL) != 0;
    bool carried;

    if (!period) {
        tx_end(s, c, n, cyc, level);
        return;
    }
    c->tx_state = ESP32_RMT_TX_ENTRY;
    c->tx_level = level;
    c->tx_entry_end_c = cyc + (uint64_t)period * ch_div(c);
    carried = (c->conf0 & CONF0_CARRIER_EN) &&
              level == ((c->conf0 & CONF0_CARRIER_OUT_LV) != 0);
    if (carried && !c->tx_carrier) {
        c->tx_carrier = true;
        c->tx_carrier_high = true;
        c->tx_carrier_end_c = cyc + carrier_cycles(c, true);
    } else if (!carried) {
        c->tx_carrier = false;
    }
    tx_set_event(c);
}

/* Process the transmitter's event at cycle cyc. */
static void tx_step(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                    uint64_t cyc)
{
    if (c->tx_state == ESP32_RMT_TX_STARTING) {
        if (tx_fetch(s, c, n)) {
            tx_begin_entry(s, c, n, cyc);
        }
        return;
    }
    if (c->tx_carrier && c->tx_carrier_end_c == cyc) {
        c->tx_carrier_high = !c->tx_carrier_high;
        c->tx_carrier_end_c = cyc + carrier_cycles(c, c->tx_carrier_high);
    }
    if (c->tx_entry_end_c != cyc) {
        tx_set_event(c);
        return;
    }
    if (c->tx_half == 0) {
        c->tx_half = 1;
    } else {
        c->tx_raddr++;
        if (ch_conf1(c, CONF1_MEM_RD_RST)) {
            c->tx_raddr = 0;
        } else if (c->tx_raddr >= ch_mem_words(c) &&
                   (s->apb_conf & APB_CONF_MEM_TX_WRAP_EN)) {
            c->tx_raddr = 0;
        }
        if (!tx_fetch(s, c, n)) {
            return;
        }
    }
    tx_begin_entry(s, c, n, cyc);
}

/*
 * A continuous transmission about to start a pass at cycle cyc. Each pass
 * starts afresh, the carrier off and the threshold count at 0, from the
 * read pointer: with the memory, the settings and the read pointer where
 * the last pass started, the pass is the last one again, as long and
 * raising the same interrupts. The passes up to cycle now_c are then
 * passed whole (the channel's pass tracking is dropped whenever the block
 * changes other than by running on). Returns the cycle at which the pass
 * to run starts, which the caller runs with, as the event's.
 */
static uint64_t tx_pass_skip(Esp32RmtChannel *c, uint64_t cyc, uint64_t now_c)
{
    if (c->pass_valid && c->pass_raddr == c->tx_raddr && cyc > c->pass_c) {
        uint64_t len = cyc - c->pass_c;

        if (now_c - cyc >= len) {
            cyc += (now_c - cyc) / len * len;
            c->tx_event_c = cyc;
        }
    }
    c->pass_valid = true;
    c->pass_c = cyc;
    c->pass_raddr = c->tx_raddr;
    return cyc;
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * Run channel n's transmitter up to time t. The edges of an output that
 * reaches a watched pad are driven one by one, each stamped with its time.
 * One that reaches none is driven once, with the level it has at t,
 * stamped with the moment that level began; meanwhile its carrier's
 * toggles are taken entry by entry in closed form, and a continuous
 * transmission whose passes repeat skips whole passes.
 */
static void tx_advance(Esp32RmtState *s, unsigned n, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];
    bool watched = s->out_watched & BIT(n);
    uint8_t level = c->out_level;
    uint64_t level_c = 0, now_c = 0;
    bool have_now = false;

    while (c->tx_state != ESP32_RMT_TX_IDLE) {
        uint64_t cyc = c->tx_event_c;
        int64_t at = cycle_time(s, c, cyc);

        if (at > t) {
            break;
        }
        if (watched) {
            tx_step(s, c, n, cyc);
            tx_update_out(s, n, at);
            continue;
        }
        if (!have_now) {
            now_c = cycle_at(s, c, t);
            have_now = true;
        }
        if (c->tx_state == ESP32_RMT_TX_ENTRY && c->tx_carrier &&
            c->tx_carrier_end_c < c->tx_entry_end_c) {
            cyc = tx_carrier_skip(c, MIN(c->tx_entry_end_c, now_c + 1));
            tx_set_event(c);
        } else {
            if (c->tx_state == ESP32_RMT_TX_STARTING &&
                ch_conf1(c, CONF1_TX_CONTI_MODE)) {
                uint64_t from = c->pass_c;
                uint64_t start = tx_pass_skip(c, cyc, now_c);

                /* The idle level, begun in the last pass, moves with it */
                if (level_c >= from) {
                    level_c += start - cyc;
                }
                cyc = start;
            }
            tx_step(s, c, n, cyc);
        }
        if (tx_out_level(c) != level) {
            level = tx_out_level(c);
            level_c = cyc;
        }
    }
    if (!watched && level != c->out_level) {
        set_out(s, n, level, cycle_time(s, c, level_c));
    }
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void tx_start(Esp32RmtState *s, unsigned n, int64_t now)
{
    Esp32RmtChannel *c = &s->ch[n];

    if (c->tx_state != ESP32_RMT_TX_IDLE) {
        return;
    }
    if (ch_conf1(c, CONF1_RX_EN)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: channel %u transmits "
                      "with its receiver enabled\n", n);
    }
    if (!ch_mem_words(c)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: channel %u transmits "
                      "with MEM_SIZE 0\n", n);
    }
    c->mem_empty = false;
    c->tx_thr_count = 0;
    c->tx_half = 0;
    c->tx_carrier = false;
    c->tx_state = ESP32_RMT_TX_STARTING;
    c->tx_event_c = tick_ceil(c, cycle_at(s, c, now) + 1);
    c->pass_valid = false;
}

/*
 * Write an entry at the receiver's write pointer. An end marker in the
 * low half of a word moves the pointer past the word, so that the pointer
 * counts the words holding the frame.
 */
static void rx_write(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                     uint32_t entry)
{
    uint32_t *w;

    if (!ch_conf1(c, CONF1_MEM_OWNER)) {
        c->owner_err = true;
        raise_int(s, INT_ERR(n));
        return;
    }
    if (c->rx_waddr >= ch_mem_words(c)) {
        c->mem_full = true;
        raise_int(s, INT_ERR(n));
        return;
    }
    w = &s->ram[ram_index(n, c->rx_waddr)];
    if (!rmt_mem_pd(s) && !s->ahead) {
        if (c->rx_half) {
            *w = (*w & 0xffff) | (entry << 16);
        } else {
            *w = (*w & 0xffff0000u) | entry;
        }
    }
    if (c->rx_half || !(entry & ENTRY_PERIOD_MASK)) {
        c->rx_half = 0;
        c->rx_waddr++;
        if (c->rx_waddr >= ch_mem_words(c)) {
            c->mem_full = true;
        }
    } else {
        c->rx_half = 1;
    }
    if (ch_conf1(c, CONF1_MEM_WR_RST)) {
        c->rx_waddr = 0;
        c->rx_half = 0;
    }
}

/*
 * The receiver's next own event: the cycle at which the frame goes idle,
 * or at which the current level fills an entry.
 */
static uint64_t rx_record_event(Esp32RmtChannel *c, bool *idle)
{
    uint64_t div = ch_div(c);
    uint64_t idle_c = c->rx_edge_c + (ch_idle_thres(c) + 1) * div;
    uint64_t full_c = c->rx_edge_c + ENTRY_PERIOD_MASK * div;

    *idle = idle_c <= full_c;
    return *idle ? idle_c : full_c;
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void rx_record(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                      uint64_t cyc)
{
    bool idle;

    rx_record_event(c, &idle);
    if (idle) {
        rx_write(s, c, n, c->rx_level ? ENTRY_LEVEL : 0);
        c->rx_frame = false;
        raise_int(s, INT_RX_END(n));
    } else {
        rx_write(s, c, n,
                 (c->rx_level ? ENTRY_LEVEL : 0) | ENTRY_PERIOD_MASK);
        c->rx_edge_c = cyc;
    }
}

/* A sampled input edge to level at channel clock tick cyc */
static void rx_edge(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                    uint64_t cyc, uint8_t level)
{
    uint64_t ticks;

    c->in_samp = level;
    if (!c->rx_frame) {
        c->rx_frame = true;
        c->rx_level = level;
        c->rx_edge_c = cyc;
        return;
    }
    ticks = (cyc - c->rx_edge_c) / ch_div(c);
    if (ticks) {
        rx_write(s, c, n, (c->rx_level ? ENTRY_LEVEL : 0) |
                          MIN(ticks, ENTRY_PERIOD_MASK));
        c->rx_edge_c = cyc;
    }
    c->rx_level = level;
}

/* The filter's output changed to level at time t: sample it. */
static void rx_filtered(Esp32RmtState *s, Esp32RmtChannel *c, int64_t t,
                        uint8_t level)
{
    uint64_t cyc = cycle_at(s, c, t);

    c->in_filt = level;
    if (cycle_time(s, c, cyc) < t) {
        cyc++;
    }
    cyc = tick_ceil(c, cyc);
    if (c->samp_pending) {
        /* Back to the sampled level before the tick: the pulse is lost */
        c->samp_pending = false;
    } else if (level != c->in_samp) {
        c->samp_pending = true;
        c->samp_c = cyc;
    }
}

/* The raw input changed to level at time t. */
static void rx_raw(Esp32RmtState *s, unsigned n, int64_t t, uint8_t level)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint32_t thres = ch_filter_thres(c);

    c->in_raw = level;
    if (!ch_conf1(c, CONF1_RX_FILTER_EN) || !thres) {
        if (level != c->in_filt) {
            rx_filtered(s, c, t, level);
        }
        return;
    }
    if (c->filt_pending) {
        /* The pulse was shorter than FILTER_THRES */
        c->filt_pending = false;
    } else if (level != c->in_filt) {
        uint64_t ns = clock_is_enabled(s->apb_clk) ?
                      clock_ticks_to_ns(s->apb_clk, thres) : 0;

        c->filt_pending = true;
        c->filt_ns = ns ? t + ns : NEVER;
    }
}

/*
 * Process the receiver's first event due by time t, a filter decision, a
 * sampled edge or a recorded entry, and return its time; NEVER if none is
 * due.
 */
static int64_t rx_step(Esp32RmtState *s, Esp32RmtChannel *c, unsigned n,
                       int64_t t)
{
    int64_t filt_t = c->filt_pending ? c->filt_ns : NEVER;
    int64_t samp_t = c->samp_pending ? cycle_time(s, c, c->samp_c) : NEVER;
    int64_t rec_t = NEVER;
    uint64_t rec_c = 0;
    bool idle;

    if (c->rx_frame) {
        rec_c = rx_record_event(c, &idle);
        rec_t = cycle_time(s, c, rec_c);
    }
    if (filt_t <= samp_t && filt_t <= rec_t && filt_t <= t &&
        filt_t != NEVER) {
        c->filt_pending = false;
        rx_filtered(s, c, filt_t, !c->in_filt);
        return filt_t;
    }
    if (samp_t <= rec_t && samp_t <= t && samp_t != NEVER) {
        c->samp_pending = false;
        rx_edge(s, c, n, c->samp_c, !c->in_samp);
        return samp_t;
    }
    if (rec_t <= t && rec_t != NEVER) {
        rx_record(s, c, n, rec_c);
        return rec_t;
    }
    return NEVER;
}

/* Run channel n's receiver up to time t. */
static void rx_advance(Esp32RmtState *s, unsigned n, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];

    if (!ch_conf1(c, CONF1_RX_EN)) {
        return;
    }
    while (rx_step(s, c, n, t) != NEVER) {
        continue;
    }
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * The time at which channel n's transmitter, running on as it is with the
 * memory as it is, first raises one of the interrupts seen; NEVER if it
 * does not. The carrier, which raises nothing, is left out. If it ever
 * reaches an end marker, the end of its memory or its threshold, it does
 * within MAX(its memory, TX_LIM) + 1 fetches, so the run looks no further.
 */
static int64_t tx_seen_at(Esp32RmtState *s, unsigned n, uint32_t seen)
{
    Esp32RmtChannel c = s->ch[n];
    uint32_t steps = 3 * (MAX(ch_mem_words(&c), c.tx_lim & TX_LIM_MASK) + 2);
    int64_t found = NEVER;

    c.conf0 &= ~CONF0_CARRIER_EN;
    c.tx_carrier = false;
    if (c.tx_state == ESP32_RMT_TX_ENTRY) {
        tx_set_event(&c);
    }
    s->ahead = true;
    s->ahead_raised = 0;
    while (c.tx_state != ESP32_RMT_TX_IDLE && steps--) {
        uint64_t cyc = c.tx_event_c;
        int64_t at = cycle_time(s, &c, cyc);

        if (at == NEVER) {
            break;
        }
        tx_step(s, &c, n, cyc);
        if (s->ahead_raised & seen) {
            found = at;
            break;
        }
    }
    s->ahead = false;
    return found;
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * The time at which channel n's receiver, with its input as it is, first
 * raises one of the interrupts seen; NEVER if it does not.
 */
static int64_t rx_seen_at(Esp32RmtState *s, unsigned n, uint32_t seen)
{
    Esp32RmtChannel c = s->ch[n];
    int64_t found = NEVER;

    s->ahead = true;
    s->ahead_raised = 0;
    for (unsigned i = 0; i < RX_AHEAD_STEPS; i++) {
        int64_t at = rx_step(s, &c, n, NEVER);

        if (at == NEVER) {
            break;
        }
        if (s->ahead_raised & seen) {
            found = at;
            break;
        }
    }
    s->ahead = false;
    return found;
}

/*
 * Bring every channel up to time now. Transmitters first, so that a
 * receiver sees the edges they sent before its own later events.
 */
static void rmt_sync(Esp32RmtState *s, int64_t now)
{
    if (s->running) {
        return;
    }
    s->running = true;
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        tx_advance(s, n, now);
    }
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        rx_advance(s, n, now);
    }
    s->running = false;
}

/*
 * The block has changed other than by running on: the moments it is seen
 * at are to be found again, and continuous passes compared afresh.
 */
static void rmt_changed(Esp32RmtState *s)
{
    s->sched_stale = true;
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        s->ch[n].pass_valid = false;
    }
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * Drive the interrupt line, and set the timer for the first moment
 * something outside the block sees: the next event of a transmitter whose
 * output reaches a watched pad, and the first raising of an interrupt that
 * INT_ENA lets out and INT_RAW does not hold. Running on alone does not
 * change that moment, as the channels evolve alone: it is found again only
 * when the block has changed otherwise (sched_stale), and the timer is set
 * only when it moves.
 */
static void rmt_schedule(Esp32RmtState *s)
{
    int64_t next = NEVER;
    uint32_t seen;

    update_irq(s);
    if (!s->sched_stale) {
        return;
    }
    s->sched_stale = false;
    seen = s->int_ena & ~s->int_raw;
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        Esp32RmtChannel *c = &s->ch[n];

        if (c->tx_state != ESP32_RMT_TX_IDLE) {
            if (s->out_watched & BIT(n)) {
                next = MIN(next, cycle_time(s, c, c->tx_event_c));
            } else if (seen & INT_TX_BITS(n)) {
                next = MIN(next, tx_seen_at(s, n, seen & INT_TX_BITS(n)));
            }
        }
        if (ch_conf1(c, CONF1_RX_EN) && (seen & INT_RX_BITS(n))) {
            next = MIN(next, rx_seen_at(s, n, seen & INT_RX_BITS(n)));
        }
    }
    if (next == NEVER) {
        if (s->due_ns != NEVER) {
            timer_del(&s->timer);
            s->due_ns = NEVER;
        }
    } else if (next != s->due_ns) {
        timer_mod_ns(&s->timer, next);
        s->due_ns = next;
    }
}

static bool rmt_receiving(Esp32RmtState *s)
{
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        if (ch_conf1(&s->ch[n], CONF1_RX_EN)) {
            return true;
        }
    }
    return false;
}

static void rmt_sync_sources(void *opaque)
{
    esp32_line_sync();
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * Bring the block up to now (esp32_line_time()), and return that moment:
 * with a receiver enabled, every source of line changes first, this
 * block's transmitters among them, so that no input edge older than now
 * is still to come when the receivers run up to now.
 */
static int64_t rmt_catch_up(Esp32RmtState *s)
{
    int64_t t = esp32_line_time();

    if (rmt_receiving(s)) {
        esp32_line_run_at(t, rmt_sync_sources, NULL);
    }
    rmt_sync(s, t);
    return t;
}

static void rmt_timer_cb(void *opaque)
{
    Esp32RmtState *s = opaque;

    s->due_ns = NEVER;
    s->sched_stale = true;
    rmt_catch_up(s);
    rmt_schedule(s);
}

/*
 * The transmitters as a line source, for blocks that sample the pads. Only
 * the transmitters: a receiver run up to the moment of the sample could
 * find a line idle whose edges another source, not yet brought up to it,
 * is still to send.
 */
static void rmt_line_sync(Notifier *notifier, void *data)
{
    Esp32RmtState *s = container_of(notifier, Esp32RmtState, line_source);
    int64_t t = esp32_line_time();

    if (s->running) {
        return;
    }
    s->running = true;
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        tx_advance(s, n, t);
    }
    s->running = false;
    rmt_schedule(s);
}

/* [spec:nuos:req:emu.esp32.rmt] An input changes at its stamp's time. */
static void esp32_rmt_sig_in(void *opaque, int n, int level)
{
    Esp32RmtState *s = opaque;
    Esp32RmtChannel *c = &s->ch[n];
    bool outer = !s->running;
    int64_t t;

    level = level != 0;
    if (level == c->in_raw) {
        return;
    }
    if (!ch_conf1(c, CONF1_RX_EN)) {
        c->in_raw = c->in_filt = c->in_samp = level;
        return;
    }
    s->sched_stale = true;
    t = esp32_line_time();
    if (outer) {
        s->running = true;
        for (unsigned i = 0; i < ESP32_RMT_CHANNELS; i++) {
            tx_advance(s, i, t);
        }
    }
    rx_advance(s, n, t);
    rx_raw(s, n, t, level);
    if (outer) {
        s->running = false;
        rmt_schedule(s);
    }
}

static uint32_t ch_status(Esp32RmtState *s, unsigned n)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint32_t base = n * ESP32_RMT_BLOCK_WORDS;
    uint32_t state = STATE_IDLE;
    uint32_t v;

    if (c->tx_state != ESP32_RMT_TX_IDLE) {
        state = STATE_SEND;
    } else if (c->rx_frame) {
        state = STATE_RECEIVE;
    }
    v = ((base + c->rx_waddr) & STATUS_ADDR_MASK) << STATUS_WADDR_SHIFT;
    v |= ((base + c->tx_raddr) & STATUS_ADDR_MASK) << STATUS_RADDR_SHIFT;
    v |= state << STATUS_STATE_SHIFT;
    v |= c->owner_err ? STATUS_MEM_OWNER_ERR : 0;
    v |= c->mem_full ? STATUS_MEM_FULL : 0;
    v |= c->mem_empty ? STATUS_MEM_EMPTY : 0;
    v |= c->apb_wr_err ? STATUS_APB_MEM_WR_ERR : 0;
    v |= c->apb_rd_err ? STATUS_APB_MEM_RD_ERR : 0;
    return v;
}

/* [spec:nuos:req:emu.esp32.rmt] */
static uint32_t fifo_read(Esp32RmtState *s, unsigned n)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint32_t v;

    if (s->apb_conf & APB_CONF_FIFO_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: CH%uDATA read with "
                      "FIFO_MASK set\n", n);
        return 0;
    }
    if (c->apb_ptr >= ch_mem_words(c)) {
        c->apb_rd_err = true;
        return 0;
    }
    v = rmt_mem_pd(s) ? 0 : s->ram[ram_index(n, c->apb_ptr)];
    if (!ch_conf1(c, CONF1_APB_MEM_RST)) {
        c->apb_ptr++;
    }
    return v;
}

static void fifo_write(Esp32RmtState *s, unsigned n, uint32_t v)
{
    Esp32RmtChannel *c = &s->ch[n];

    if (s->apb_conf & APB_CONF_FIFO_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: CH%uDATA written with "
                      "FIFO_MASK set\n", n);
        return;
    }
    if (c->apb_ptr >= ch_mem_words(c)) {
        c->apb_wr_err = true;
        return;
    }
    if (!rmt_mem_pd(s)) {
        s->ram[ram_index(n, c->apb_ptr)] = v;
    }
    if (!ch_conf1(c, CONF1_APB_MEM_RST)) {
        c->apb_ptr++;
    }
}

static uint64_t rmt_reg_read(Esp32RmtState *s, hwaddr addr)
{
    unsigned n;

    rmt_catch_up(s);
    rmt_schedule(s);
    switch (addr) {
    case A_DATA(0) ... A_DATA(7):
        n = addr / 4;
        return fifo_read(s, n);
    case A_CONF0(0) ... A_CONF1(7):
        n = (addr - A_CONF0(0)) / 8;
        if (addr & 4) {
            return s->ch[n].conf1 |
                   (s->ch[n].tx_state != ESP32_RMT_TX_IDLE ? CONF1_TX_START
                                                           : 0);
        }
        return s->ch[n].conf0;
    case A_STATUS(0) ... A_STATUS(7):
        return ch_status(s, (addr - A_STATUS(0)) / 4);
    case A_APB_MEM_ADDR(0) ... A_APB_MEM_ADDR(7):
        n = (addr - A_APB_MEM_ADDR(0)) / 4;
        return (n * ESP32_RMT_BLOCK_WORDS + s->ch[n].apb_ptr) &
               STATUS_ADDR_MASK;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_CARRIER_DUTY(0) ... A_CARRIER_DUTY(7):
        return s->ch[(addr - A_CARRIER_DUTY(0)) / 4].carrier_duty;
    case A_TX_LIM(0) ... A_TX_LIM(7):
        return s->ch[(addr - A_TX_LIM(0)) / 4].tx_lim;
    case A_APB_CONF:
        return s->apb_conf;
    case A_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: read of reserved offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void conf0_write(Esp32RmtState *s, unsigned n, uint32_t v, int64_t now)
{
    Esp32RmtChannel *c = &s->ch[n];

    if (n != 0) {
        v &= ~(CONF0_MEM_PD | CONF0_CLK_EN);
    }
    if ((v ^ c->conf0) & CONF0_DIV_CNT_MASK) {
        reanchor(s, c, now);
    }
    if (n == 0 && (v & CONF0_MEM_PD) && !(c->conf0 & CONF0_MEM_PD)) {
        memset(s->ram, 0, sizeof(s->ram));
    }
    c->conf0 = v;
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void conf1_write(Esp32RmtState *s, unsigned n, uint32_t v, int64_t now)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint32_t old = c->conf1;

    v &= CONF1_MASK;
    if ((v ^ old) & CONF1_REF_ALWAYS_ON) {
        reanchor(s, c, now);
    }
    c->conf1 = v & ~(CONF1_TX_START | CONF1_REF_CNT_RST);

    if (v & CONF1_REF_CNT_RST) {
        /* The divider restarts: a channel clock tick now */
        reanchor(s, c, now);
        c->anchor_c = tick_ceil(c, c->anchor_c);
    }
    if (v & CONF1_MEM_RD_RST) {
        c->tx_raddr = 0;
        c->mem_empty = false;
        if (c->tx_state == ESP32_RMT_TX_IDLE) {
            c->tx_half = 0;
        }
    }
    if (v & CONF1_MEM_WR_RST) {
        c->rx_waddr = 0;
        c->rx_half = 0;
        c->mem_full = false;
        c->owner_err = false;
    }
    if (v & CONF1_APB_MEM_RST) {
        c->apb_ptr = 0;
        c->apb_rd_err = false;
        c->apb_wr_err = false;
    }
    if ((v ^ old) & CONF1_RX_EN) {
        /* Enabling or disabling the receiver drops any frame under way */
        c->rx_frame = false;
        c->filt_pending = false;
        c->samp_pending = false;
        c->in_filt = c->in_samp = c->in_raw;
    } else if ((old & CONF1_RX_FILTER_EN) && !(v & CONF1_RX_FILTER_EN) &&
               c->filt_pending) {
        c->filt_pending = false;
        rx_filtered(s, c, now, c->in_raw);
    }
    if (v & CONF1_TX_START) {
        tx_start(s, n, now);
    }
    if (c->tx_state != ESP32_RMT_TX_ENTRY) {
        tx_update_out(s, n, now);
    }
}

static void rmt_reg_write(Esp32RmtState *s, hwaddr addr, uint64_t value)
{
    int64_t now = esp32_line_time();
    uint32_t v = value;
    unsigned n;

    rmt_catch_up(s);
    switch (addr) {
    case A_DATA(0) ... A_DATA(7):
        fifo_write(s, addr / 4, v);
        break;
    case A_CONF0(0) ... A_CONF1(7):
        n = (addr - A_CONF0(0)) / 8;
        if (addr & 4) {
            conf1_write(s, n, v, now);
        } else {
            conf0_write(s, n, v, now);
        }
        break;
    case A_STATUS(0) ... A_STATUS(7):
    case A_APB_MEM_ADDR(0) ... A_APB_MEM_ADDR(7):
    case A_INT_RAW:
    case A_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: write to read-only "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    case A_INT_ENA:
        s->int_ena = v;
        break;
    case A_INT_CLR:
        s->int_raw &= ~v;
        break;
    case A_CARRIER_DUTY(0) ... A_CARRIER_DUTY(7):
        s->ch[(addr - A_CARRIER_DUTY(0)) / 4].carrier_duty = v;
        break;
    case A_TX_LIM(0) ... A_TX_LIM(7):
        s->ch[(addr - A_TX_LIM(0)) / 4].tx_lim = v & TX_LIM_MASK;
        break;
    case A_APB_CONF:
        s->apb_conf = v & APB_CONF_MASK;
        break;
    case A_DATE:
        s->date = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: write to reserved offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    rmt_changed(s);
    rmt_schedule(s);
}

/* A register or RAM access under way, which acts at one moment throughout. */
typedef struct RmtAccess {
    Esp32RmtState *s;
    hwaddr addr;
    uint64_t value;
    unsigned size;
} RmtAccess;

static void rmt_access_read(void *opaque)
{
    RmtAccess *a = opaque;

    a->value = rmt_reg_read(a->s, a->addr);
}

static void rmt_access_write(void *opaque)
{
    RmtAccess *a = opaque;

    rmt_reg_write(a->s, a->addr, a->value);
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * A register or RAM access acts at the moment it began (see
 * esp32_line.h): what it reads, the transmission it starts and the line
 * changes it makes belong to that moment, however long the access takes
 * without icount.
 */
static uint64_t esp32_rmt_read(void *opaque, hwaddr addr, unsigned size)
{
    RmtAccess a = { .s = opaque, .addr = addr, .size = size };

    esp32_line_run_at(esp32_line_time(), rmt_access_read, &a);
    return a.value;
}

static void esp32_rmt_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    RmtAccess a = { .s = opaque, .addr = addr, .value = value, .size = size };

    esp32_line_run_at(esp32_line_time(), rmt_access_write, &a);
}

static const MemoryRegionOps esp32_rmt_ops = {
    .read = esp32_rmt_read,
    .write = esp32_rmt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static bool ram_accessible(Esp32RmtState *s, hwaddr addr, bool write)
{
    if (!(s->apb_conf & APB_CONF_FIFO_MASK)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: RAM %s at 0x%" HWADDR_PRIx
                      " with FIFO_MASK clear\n", write ? "write" : "read",
                      addr);
        return false;
    }
    if (rmt_mem_pd(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_rmt: RAM %s at 0x%" HWADDR_PRIx
                      " while it is powered down (MEM_PD)\n",
                      write ? "write" : "read", addr);
        return false;
    }
    return true;
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void rmt_ram_access_read(void *opaque)
{
    RmtAccess *a = opaque;
    Esp32RmtState *s = a->s;

    rmt_catch_up(s);
    rmt_schedule(s);
    a->value = ram_accessible(s, a->addr, false) ?
               extract32(s->ram[a->addr / 4], (a->addr & 3) * 8,
                         a->size * 8) : 0;
}

static void rmt_ram_access_write(void *opaque)
{
    RmtAccess *a = opaque;
    Esp32RmtState *s = a->s;

    rmt_catch_up(s);
    if (ram_accessible(s, a->addr, true)) {
        s->ram[a->addr / 4] = deposit32(s->ram[a->addr / 4],
                                        (a->addr & 3) * 8, a->size * 8,
                                        a->value);
        rmt_changed(s);
    }
    rmt_schedule(s);
}

static uint64_t esp32_rmt_ram_read(void *opaque, hwaddr addr, unsigned size)
{
    RmtAccess a = { .s = opaque, .addr = addr, .size = size };

    esp32_line_run_at(esp32_line_time(), rmt_ram_access_read, &a);
    return a.value;
}

static void esp32_rmt_ram_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    RmtAccess a = { .s = opaque, .addr = addr, .value = value, .size = size };

    esp32_line_run_at(esp32_line_time(), rmt_ram_access_write, &a);
}

static const MemoryRegionOps esp32_rmt_ram_ops = {
    .read = esp32_rmt_ram_read,
    .write = esp32_rmt_ram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * A clock is about to change: bring the line sources and the channels up
 * to now with the old period (rmt_catch_up()), and restart their cycle
 * counts there.
 */
static void esp32_rmt_clk_update(void *opaque, ClockEvent event)
{
    Esp32RmtState *s = opaque;
    int64_t now;

    if (event == ClockPreUpdate) {
        now = rmt_catch_up(s);
        for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
            reanchor(s, &s->ch[n], now);
        }
    } else {
        rmt_changed(s);
        rmt_schedule(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32.rmt]
 * Which channels' outputs reach a pad whose level something but software
 * sampling the pads sees (esp32_sig_out_watched()): only their edges are
 * timed one by one. The others are driven with their level of the moment
 * whenever the block is brought up to date. The transmitters are first
 * brought up to now as they were, so that an output a watcher takes up
 * has its level of the moment and its next edge timed from there.
 */
static void rmt_routes_run(void *opaque)
{
    Esp32RmtState *s = opaque;
    int64_t t = esp32_line_time();
    uint8_t watched = 0;

    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        if (!s->gpio ||
            esp32_sig_out_watched(OBJECT(s->gpio), ESP32_SIG_RMT_OUT0 + n)) {
            watched |= BIT(n);
        }
    }
    if (watched == s->out_watched) {
        return;
    }
    if (!s->running) {
        s->running = true;
        for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
            tx_advance(s, n, t);
        }
        s->running = false;
    }
    s->out_watched = watched;
    rmt_changed(s);
    if (!s->running) {
        rmt_schedule(s);
    }
}

static void rmt_route_notify(Notifier *notifier, void *data)
{
    esp32_line_run_at(esp32_line_time(), rmt_routes_run,
                      container_of(notifier, Esp32RmtState, route_notifier));
}

static void esp32_rmt_reset_hold(Object *obj, ResetType type)
{
    Esp32RmtState *s = ESP32_RMT(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timer_del(&s->timer);
    s->due_ns = NEVER;
    s->sched_stale = true;
    s->irq_level = false;
    s->int_raw = 0;
    s->int_ena = 0;
    s->apb_conf = 0;
    s->date = DATE_RESET;
    s->running = false;
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        Esp32RmtChannel *c = &s->ch[n];
        uint8_t in = c->in_raw;

        memset(c, 0, sizeof(*c));
        c->conf0 = n == 0 ? CONF0_RESET : CONF0_RESET & ~CONF0_CLK_EN;
        c->conf1 = CONF1_RESET;
        c->carrier_duty = CARRIER_DUTY_RESET;
        c->tx_lim = TX_LIM_RESET;
        c->anchor_ns = now;
        c->tx_state = ESP32_RMT_TX_IDLE;
        c->in_raw = c->in_filt = c->in_samp = in;
    }
}

static void esp32_rmt_reset_exit(Object *obj, ResetType type)
{
    Esp32RmtState *s = ESP32_RMT(obj);

    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        qemu_set_irq(s->sig_out[n], 0);
    }
    qemu_set_irq(s->irq, 0);
    esp32_line_run_at(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), rmt_routes_run,
                      s);
}

static void esp32_rmt_init(Object *obj)
{
    Esp32RmtState *s = ESP32_RMT(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init(&s->container, obj, TYPE_ESP32_RMT, ESP32_RMT_SIZE);
    memory_region_init_io(&s->iomem, obj, &esp32_rmt_ops, s,
                          TYPE_ESP32_RMT ".regs", ESP32_RMT_REG_SIZE);
    memory_region_add_subregion(&s->container, 0, &s->iomem);
    memory_region_init_io(&s->ram_iomem, obj, &esp32_rmt_ram_ops, s,
                          TYPE_ESP32_RMT ".ram", ESP32_RMT_RAM_WORDS * 4);
    memory_region_add_subregion(&s->container, ESP32_RMT_RAM_OFFSET,
                                &s->ram_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->container);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->sig_out, ESP32_RMT_SIG_OUT,
                             ESP32_RMT_CHANNELS);
    qdev_init_gpio_in_named(dev, esp32_rmt_sig_in, ESP32_RMT_SIG_IN,
                            ESP32_RMT_CHANNELS);
    s->apb_clk = qdev_init_clock_in(dev, "apb", esp32_rmt_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->ref_tick_clk = qdev_init_clock_in(dev, "ref_tick",
                                         esp32_rmt_clk_update, s,
                                         ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, rmt_timer_cb, s);
    s->line_source.notify = rmt_line_sync;
    s->route_notifier.notify = rmt_route_notify;
    s->out_watched = MAKE_64BIT_MASK(0, ESP32_RMT_CHANNELS);
    s->due_ns = NEVER;
}

static void esp32_rmt_realize(DeviceState *dev, Error **errp)
{
    Esp32RmtState *s = ESP32_RMT(dev);

    if (s->gpio && !object_dynamic_cast(OBJECT(s->gpio),
                                        TYPE_ESP32_SIG_ROUTES)) {
        error_setg(errp, "%s: \"gpio\" is not a GPIO matrix",
                   object_get_typename(OBJECT(dev)));
        return;
    }
    esp32_line_add_source(&s->line_source);
    if (s->gpio) {
        esp32_sig_add_route_notifier(OBJECT(s->gpio), &s->route_notifier);
    }
}

static const Property esp32_rmt_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32RmtState, gpio, TYPE_DEVICE,
                     DeviceState *),
};

/*
 * The timer's moment is the one it is set for; the schedule is found
 * again, and continuous passes compared afresh, as the block next runs.
 */
static int esp32_rmt_post_load(void *opaque, int version_id)
{
    Esp32RmtState *s = opaque;

    s->due_ns = timer_pending(&s->timer) ? timer_expire_time_ns(&s->timer)
                                         : NEVER;
    rmt_changed(s);
    return 0;
}

static const VMStateDescription vmstate_esp32_rmt_channel = {
    .name = "esp32-rmt-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, Esp32RmtChannel),
        VMSTATE_UINT32(conf1, Esp32RmtChannel),
        VMSTATE_UINT32(carrier_duty, Esp32RmtChannel),
        VMSTATE_UINT32(tx_lim, Esp32RmtChannel),
        VMSTATE_INT64(anchor_ns, Esp32RmtChannel),
        VMSTATE_UINT64(anchor_c, Esp32RmtChannel),
        VMSTATE_UINT8(tx_state, Esp32RmtChannel),
        VMSTATE_UINT32(tx_raddr, Esp32RmtChannel),
        VMSTATE_UINT8(tx_half, Esp32RmtChannel),
        VMSTATE_UINT32(tx_word, Esp32RmtChannel),
        VMSTATE_UINT64(tx_event_c, Esp32RmtChannel),
        VMSTATE_UINT64(tx_entry_end_c, Esp32RmtChannel),
        VMSTATE_UINT8(tx_level, Esp32RmtChannel),
        VMSTATE_BOOL(tx_carrier, Esp32RmtChannel),
        VMSTATE_BOOL(tx_carrier_high, Esp32RmtChannel),
        VMSTATE_UINT64(tx_carrier_end_c, Esp32RmtChannel),
        VMSTATE_UINT32(tx_thr_count, Esp32RmtChannel),
        VMSTATE_UINT8(tx_end_level, Esp32RmtChannel),
        VMSTATE_BOOL(mem_empty, Esp32RmtChannel),
        VMSTATE_UINT8(out_level, Esp32RmtChannel),
        VMSTATE_UINT8(in_raw, Esp32RmtChannel),
        VMSTATE_UINT8(in_filt, Esp32RmtChannel),
        VMSTATE_BOOL(filt_pending, Esp32RmtChannel),
        VMSTATE_INT64(filt_ns, Esp32RmtChannel),
        VMSTATE_UINT8(in_samp, Esp32RmtChannel),
        VMSTATE_BOOL(samp_pending, Esp32RmtChannel),
        VMSTATE_UINT64(samp_c, Esp32RmtChannel),
        VMSTATE_BOOL(rx_frame, Esp32RmtChannel),
        VMSTATE_UINT8(rx_level, Esp32RmtChannel),
        VMSTATE_UINT64(rx_edge_c, Esp32RmtChannel),
        VMSTATE_UINT32(rx_waddr, Esp32RmtChannel),
        VMSTATE_UINT8(rx_half, Esp32RmtChannel),
        VMSTATE_BOOL(mem_full, Esp32RmtChannel),
        VMSTATE_BOOL(owner_err, Esp32RmtChannel),
        VMSTATE_UINT32(apb_ptr, Esp32RmtChannel),
        VMSTATE_BOOL(apb_rd_err, Esp32RmtChannel),
        VMSTATE_BOOL(apb_wr_err, Esp32RmtChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_rmt = {
    .name = TYPE_ESP32_RMT,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = esp32_rmt_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ram, Esp32RmtState, ESP32_RMT_RAM_WORDS),
        VMSTATE_UINT32(int_raw, Esp32RmtState),
        VMSTATE_UINT32(int_ena, Esp32RmtState),
        VMSTATE_UINT32(apb_conf, Esp32RmtState),
        VMSTATE_UINT32(date, Esp32RmtState),
        VMSTATE_STRUCT_ARRAY(ch, Esp32RmtState, ESP32_RMT_CHANNELS, 1,
                             vmstate_esp32_rmt_channel, Esp32RmtChannel),
        VMSTATE_TIMER(timer, Esp32RmtState),
        VMSTATE_CLOCK(apb_clk, Esp32RmtState),
        VMSTATE_CLOCK(ref_tick_clk, Esp32RmtState),
        VMSTATE_UINT8(out_watched, Esp32RmtState),
        VMSTATE_BOOL(irq_level, Esp32RmtState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_rmt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_rmt_reset_hold;
    rc->phases.exit = esp32_rmt_reset_exit;
    dc->realize = esp32_rmt_realize;
    dc->vmsd = &vmstate_esp32_rmt;
    device_class_set_props(dc, esp32_rmt_properties);
}

/* [spec:nuos:req:emu.esp32.rmt] */
static const TypeInfo esp32_rmt_info = {
    .name = TYPE_ESP32_RMT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RmtState),
    .instance_init = esp32_rmt_init,
    .class_init = esp32_rmt_class_init,
};

static void esp32_rmt_register_types(void)
{
    type_register_static(&esp32_rmt_info);
}

type_init(esp32_rmt_register_types)
