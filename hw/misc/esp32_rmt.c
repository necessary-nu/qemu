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
 * channel clock.
 *
 * Timing runs on QEMU_CLOCK_VIRTUAL with one timer for the block, set for
 * the next output edge, filter decision, sampled input edge or idle end;
 * there are no per-tick events. When the timer runs late, or a register
 * access finds events due, every due event is processed in order, each at
 * its own virtual time, so signal edges at rates far above the timer's
 * resolution are delivered as a batch with their exact times. An output
 * edge passes through the GPIO matrix at once; an RMT input that changes
 * in response takes the time of the edge that caused it, so a channel
 * receiving another's output through the matrix measures the transmitted
 * durations exactly. Every edge is still delivered: the cost of a
 * transmission is proportional to its number of edges, carrier edges
 * included.
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
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
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

/* Period of channel n's base clock, in 2^-32 ns; 0 while it is stopped */
static uint64_t base_period(Esp32RmtState *s, unsigned n)
{
    if (ch_conf1(&s->ch[n], CONF1_REF_ALWAYS_ON)) {
        return clock_get(s->apb_clk);
    }
    return clock_get(s->ref_tick_clk);
}

/* The base clock cycle channel n is in at time t */
static uint64_t cycle_at(Esp32RmtState *s, unsigned n, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint64_t period = base_period(s, n);
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

/* The first time at which channel n has reached cycle cyc */
static int64_t cycle_time(Esp32RmtState *s, unsigned n, uint64_t cyc)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint64_t period = base_period(s, n);
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

/* Restart channel n's cycle count from time now, at the cycle it is in. */
static void reanchor(Esp32RmtState *s, unsigned n, int64_t now)
{
    Esp32RmtChannel *c = &s->ch[n];

    c->anchor_c = cycle_at(s, n, now);
    c->anchor_ns = now;
}

/* The first channel clock tick of channel n at or after cycle cyc */
static uint64_t tick_ceil(Esp32RmtState *s, unsigned n, uint64_t cyc)
{
    uint32_t div = ch_div(&s->ch[n]);

    return DIV_ROUND_UP(cyc, div) * div;
}

static void raise_int(Esp32RmtState *s, uint32_t bits)
{
    s->int_raw |= bits;
}

static void update_irq(Esp32RmtState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

/* Drive channel n's output to level from time t. */
static void set_out(Esp32RmtState *s, unsigned n, uint8_t level, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];
    bool was_emitting = s->emitting;
    int64_t was_ns = s->emit_ns;

    if (c->out_level == level) {
        return;
    }
    c->out_level = level;
    s->emitting = true;
    s->emit_ns = t;
    qemu_set_irq(s->sig_out[n], level);
    s->emitting = was_emitting;
    s->emit_ns = was_ns;
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void tx_update_out(Esp32RmtState *s, unsigned n, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint8_t level;

    if (c->tx_state == ESP32_RMT_TX_ENTRY) {
        level = c->tx_carrier ? c->tx_carrier_high : c->tx_level;
    } else if (ch_conf1(c, CONF1_IDLE_OUT_EN)) {
        level = ch_conf1(c, CONF1_IDLE_OUT_LV);
    } else {
        level = c->tx_end_level;
    }
    set_out(s, n, level, t);
}

/*
 * Fetch the word at the read pointer. Past the end of the channel's
 * memory the transmitter stops with MEM_EMPTY and ERR.
 */
static bool tx_fetch(Esp32RmtState *s, unsigned n)
{
    Esp32RmtChannel *c = &s->ch[n];
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

/* An end marker with the given level, reached at cycle cyc. */
static void tx_end(Esp32RmtState *s, unsigned n, uint64_t cyc, uint8_t level)
{
    Esp32RmtChannel *c = &s->ch[n];

    c->tx_end_level = level;
    c->tx_carrier = false;
    c->tx_raddr = 0;
    c->tx_half = 0;
    raise_int(s, INT_TX_END(n));
    if (ch_conf1(c, CONF1_TX_CONTI_MODE)) {
        /* One channel clock tick at the idle level between passes */
        c->tx_state = ESP32_RMT_TX_STARTING;
        c->tx_event_c = cyc + ch_div(c);
        c->tx_thr_count = 0;
    } else {
        c->tx_state = ESP32_RMT_TX_IDLE;
    }
}

/* Start sending the current entry at cycle cyc. */
static void tx_begin_entry(Esp32RmtState *s, unsigned n, uint64_t cyc)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint32_t e = c->tx_half ? c->tx_word >> 16 : c->tx_word & 0xffff;
    uint32_t period = e & ENTRY_PERIOD_MASK;
    uint8_t level = (e & ENTRY_LEVEL) != 0;
    bool carried;

    if (!period) {
        tx_end(s, n, cyc, level);
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
static void tx_step(Esp32RmtState *s, unsigned n, uint64_t cyc)
{
    Esp32RmtChannel *c = &s->ch[n];

    if (c->tx_state == ESP32_RMT_TX_STARTING) {
        if (tx_fetch(s, n)) {
            tx_begin_entry(s, n, cyc);
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
        if (!tx_fetch(s, n)) {
            return;
        }
    }
    tx_begin_entry(s, n, cyc);
}

/* Run channel n's transmitter up to time t. */
static void tx_advance(Esp32RmtState *s, unsigned n, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];

    while (c->tx_state != ESP32_RMT_TX_IDLE) {
        uint64_t cyc = c->tx_event_c;
        int64_t at = cycle_time(s, n, cyc);

        if (at > t) {
            break;
        }
        tx_step(s, n, cyc);
        tx_update_out(s, n, at);
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
    c->tx_event_c = tick_ceil(s, n, cycle_at(s, n, now) + 1);
}

/*
 * Write an entry at the receiver's write pointer. An end marker in the
 * low half of a word moves the pointer past the word, so that the pointer
 * counts the words holding the frame.
 */
static void rx_write(Esp32RmtState *s, unsigned n, uint32_t entry)
{
    Esp32RmtChannel *c = &s->ch[n];
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
    if (!rmt_mem_pd(s)) {
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
static void rx_record(Esp32RmtState *s, unsigned n, uint64_t cyc)
{
    Esp32RmtChannel *c = &s->ch[n];
    bool idle;

    rx_record_event(c, &idle);
    if (idle) {
        rx_write(s, n, c->rx_level ? ENTRY_LEVEL : 0);
        c->rx_frame = false;
        raise_int(s, INT_RX_END(n));
    } else {
        rx_write(s, n, (c->rx_level ? ENTRY_LEVEL : 0) | ENTRY_PERIOD_MASK);
        c->rx_edge_c = cyc;
    }
}

/* A sampled input edge to level at channel clock tick cyc */
static void rx_edge(Esp32RmtState *s, unsigned n, uint64_t cyc, uint8_t level)
{
    Esp32RmtChannel *c = &s->ch[n];
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
        rx_write(s, n, (c->rx_level ? ENTRY_LEVEL : 0) |
                       MIN(ticks, ENTRY_PERIOD_MASK));
        c->rx_edge_c = cyc;
    }
    c->rx_level = level;
}

/* The filter's output changed to level at time t: sample it. */
static void rx_filtered(Esp32RmtState *s, unsigned n, int64_t t,
                        uint8_t level)
{
    Esp32RmtChannel *c = &s->ch[n];
    uint64_t cyc = cycle_at(s, n, t);

    c->in_filt = level;
    if (cycle_time(s, n, cyc) < t) {
        cyc++;
    }
    cyc = tick_ceil(s, n, cyc);
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
            rx_filtered(s, n, t, level);
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

/* Run channel n's receiver up to time t. */
static void rx_advance(Esp32RmtState *s, unsigned n, int64_t t)
{
    Esp32RmtChannel *c = &s->ch[n];

    if (!ch_conf1(c, CONF1_RX_EN)) {
        return;
    }
    for (;;) {
        int64_t filt_t = c->filt_pending ? c->filt_ns : NEVER;
        int64_t samp_t = c->samp_pending ? cycle_time(s, n, c->samp_c)
                                         : NEVER;
        int64_t rec_t = NEVER;
        uint64_t rec_c = 0;
        bool idle;

        if (c->rx_frame) {
            rec_c = rx_record_event(c, &idle);
            rec_t = cycle_time(s, n, rec_c);
        }
        if (filt_t <= samp_t && filt_t <= rec_t && filt_t <= t &&
            filt_t != NEVER) {
            c->filt_pending = false;
            rx_filtered(s, n, filt_t, !c->in_filt);
        } else if (samp_t <= rec_t && samp_t <= t && samp_t != NEVER) {
            c->samp_pending = false;
            rx_edge(s, n, c->samp_c, !c->in_samp);
        } else if (rec_t <= t && rec_t != NEVER) {
            rx_record(s, n, rec_c);
        } else {
            break;
        }
    }
}

static int64_t rx_next_event(Esp32RmtState *s, unsigned n)
{
    Esp32RmtChannel *c = &s->ch[n];
    int64_t t = NEVER;
    bool idle;

    if (!ch_conf1(c, CONF1_RX_EN)) {
        return NEVER;
    }
    if (c->filt_pending) {
        t = MIN(t, c->filt_ns);
    }
    if (c->samp_pending) {
        t = MIN(t, cycle_time(s, n, c->samp_c));
    }
    if (c->rx_frame) {
        t = MIN(t, cycle_time(s, n, rx_record_event(c, &idle)));
    }
    return t;
}

/*
 * Bring every channel up to time now. Transmitters first, so that a
 * receiver sees the edges they sent before its own later events.
 */
static void rmt_sync(Esp32RmtState *s, int64_t now)
{
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        tx_advance(s, n, now);
    }
    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        rx_advance(s, n, now);
    }
}

static void rmt_schedule(Esp32RmtState *s)
{
    int64_t next = NEVER;

    for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
        Esp32RmtChannel *c = &s->ch[n];

        if (c->tx_state != ESP32_RMT_TX_IDLE) {
            next = MIN(next, cycle_time(s, n, c->tx_event_c));
        }
        next = MIN(next, rx_next_event(s, n));
    }
    if (next == NEVER) {
        timer_del(&s->timer);
    } else {
        timer_mod_ns(&s->timer, next);
    }
    update_irq(s);
}

static void rmt_timer_cb(void *opaque)
{
    Esp32RmtState *s = opaque;

    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    rmt_schedule(s);
}

/* [spec:nuos:req:emu.esp32.rmt] */
static void esp32_rmt_sig_in(void *opaque, int n, int level)
{
    Esp32RmtState *s = opaque;
    Esp32RmtChannel *c = &s->ch[n];
    int64_t t;

    level = level != 0;
    if (level == c->in_raw) {
        return;
    }
    if (!ch_conf1(c, CONF1_RX_EN)) {
        c->in_raw = c->in_filt = c->in_samp = level;
        return;
    }
    if (s->emitting) {
        t = s->emit_ns;
    } else {
        t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        for (unsigned i = 0; i < ESP32_RMT_CHANNELS; i++) {
            tx_advance(s, i, t);
        }
    }
    rx_advance(s, n, t);
    rx_raw(s, n, t, level);
    if (!s->emitting) {
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

static uint64_t esp32_rmt_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32RmtState *s = opaque;
    unsigned n;

    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
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
        reanchor(s, n, now);
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
        reanchor(s, n, now);
    }
    c->conf1 = v & ~(CONF1_TX_START | CONF1_REF_CNT_RST);

    if (v & CONF1_REF_CNT_RST) {
        /* The divider restarts: a channel clock tick now */
        reanchor(s, n, now);
        c->anchor_c = tick_ceil(s, n, c->anchor_c);
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
        rx_filtered(s, n, now, c->in_raw);
    }
    if (v & CONF1_TX_START) {
        tx_start(s, n, now);
    }
    if (c->tx_state != ESP32_RMT_TX_ENTRY) {
        tx_update_out(s, n, now);
    }
}

static void esp32_rmt_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Esp32RmtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = value;
    unsigned n;

    rmt_sync(s, now);
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
    rmt_schedule(s);
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
static uint64_t esp32_rmt_ram_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32RmtState *s = opaque;
    unsigned shift = (addr & 3) * 8;

    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    rmt_schedule(s);
    if (!ram_accessible(s, addr, false)) {
        return 0;
    }
    return extract32(s->ram[addr / 4], shift, size * 8);
}

static void esp32_rmt_ram_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    Esp32RmtState *s = opaque;
    unsigned shift = (addr & 3) * 8;

    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    rmt_schedule(s);
    if (!ram_accessible(s, addr, true)) {
        return;
    }
    s->ram[addr / 4] = deposit32(s->ram[addr / 4], shift, size * 8, value);
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
 * A clock is about to change: bring the channels up to now with the old
 * period, and restart their cycle counts there.
 */
static void esp32_rmt_clk_update(void *opaque, ClockEvent event)
{
    Esp32RmtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        rmt_sync(s, now);
        for (unsigned n = 0; n < ESP32_RMT_CHANNELS; n++) {
            reanchor(s, n, now);
        }
    } else {
        rmt_schedule(s);
    }
}

static void esp32_rmt_reset_hold(Object *obj, ResetType type)
{
    Esp32RmtState *s = ESP32_RMT(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timer_del(&s->timer);
    s->int_raw = 0;
    s->int_ena = 0;
    s->apb_conf = 0;
    s->date = DATE_RESET;
    s->emitting = false;
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
    update_irq(s);
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
    .version_id = 1,
    .minimum_version_id = 1,
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
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_rmt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_rmt_reset_hold;
    rc->phases.exit = esp32_rmt_reset_exit;
    dc->vmsd = &vmstate_esp32_rmt;
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
