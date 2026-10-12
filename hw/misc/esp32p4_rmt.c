/*
 * ESP32-P4 RMT, the remote control (pulse train) peripheral
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM (v3.x silicon), "Remote Control Peripheral
 * (RMT)", with the field layout, access types and reset values of
 * ESP-IDF's v3 rmt_reg.h and rmt_struct.h, and the use ESP-IDF's RMT
 * driver (esp_driver_rmt) and hal/rmt_ll.h make of them on the P4. The
 * timing model follows the ESP32's RMT (hw/misc/esp32_rmt.c).
 *
 * Channels 0 to 3 transmit and channels 4 to 7 (RX channels 0 to 3)
 * receive. They share 384 words of RAM, eight 48-word blocks at offset
 * 0x800: channel n owns block n and, with MEM_SIZE > 1, the blocks after
 * it, up to the end of the RAM, which does not wrap. Each word holds two
 * 16-bit entries, low half first: a level (bit 15) and a period in
 * channel clock ticks (bits 14:0); a period of 0 is an end marker. Every
 * channel divides rmt_sclk, which HP_SYS_CLKRST's RMT_CLK_SRC_SEL and
 * fractional divider make (the TRM's 60.3.3; RMT_SYS_CONF's SCLK_* fields
 * are kept but select nothing on the P4), by its DIV_CNT (0 meaning 256)
 * into its channel clock.
 *
 * Timing runs on QEMU_CLOCK_VIRTUAL with one timer for the block, set for
 * the next output edge, filter decision, sampled input edge, demodulator
 * decision or idle end; there are no per-tick events. Every due event is
 * processed in order, each at its own virtual time, and an output edge is
 * stamped with its time on the GPIO matrix signal (hw/gpio/esp32_line.h),
 * so that a receiver measures it exactly whenever the model gets to run.
 * The transmitters' edges are delivered in time order across channels,
 * and an RMT input that changes in response takes the time of the edge
 * that caused it. The block is a line source: whenever software samples
 * the pads or reads a block that counts edges, the outputs are brought up
 * to the present first.
 *
 * Behaviour the TRM leaves open, as modelled:
 * - The fields of TRM table 60.3-1 take effect when the channel's
 *   CONF_UPDATE is written; until then the registers read back what was
 *   written. TX_SIM_EN takes effect on any TX channel's CONF_UPDATE, the
 *   RX channels' REF_CNT_RST on their own. TX_STOP stops the transmitter
 *   at that CONF_UPDATE and then clears.
 * - TX_START starts the transmitter when it is idle, from the start of its
 *   memory, on the channel clock tick after the write. The transmitter
 *   counts the words it fetches; each time the count reaches TX_LIM it
 *   raises TX_THR_EVENT and starts again from 0. At an end marker it
 *   raises TX_END and, unless in continuous mode, stops with its read
 *   pointer back at the start of its memory. Reaching the end of its
 *   memory without MEM_TX_WRAP_EN (or continuous mode, which also starts
 *   over there) sets MEM_EMPTY, raises ERR and stops it without TX_END.
 * - Continuous mode starts the next pass at the end marker without a
 *   gap, as the TRM's constraint (60.3) on the entry before an end marker
 *   implies; a pass with no entry before its end marker takes a channel
 *   clock tick. With TX_LOOP_CNT_EN each end marker counts a loop; at
 *   TX_LOOP_NUM loops TX_LOOP is raised, the count starts again from 0
 *   (ESP-IDF continues a loop transaction in batches without resetting
 *   it), and with LOOP_STOP_EN the transmitter stops there.
 * - Simultaneous transmission: with TX_SIM_EN in effect, TX_START of a
 *   channel in TX_SIM_CHn waits until every such channel has had one, and
 *   they then start together, each on its next channel clock tick.
 * - The idle output is IDLE_OUT_LV with IDLE_OUT_EN set, else the level
 *   of the last end marker sent (0 out of reset).
 * - Carrier: a level equal to CARRIER_OUT_LV is sent as the carrier, high
 *   for CARRIER_HIGH and low for CARRIER_LOW rmt_sclk cycles (0 meaning
 *   65536) as ESP-IDF programs it, against the TRM's "+ 1"; only while
 *   sending entries with CARRIER_EFF_EN, also while idle without it. The
 *   carrier starts high where a carried stretch starts and runs on across
 *   it.
 * - The receiver's pipeline: the filter (RX_FILTER_EN) takes an input
 *   change once the input has held the new level for RX_FILTER_THRES
 *   rmt_sclk cycles, removing shorter pulses; the sampler sees a change on
 *   the next channel clock tick (a pulse that spans no tick is lost); the
 *   demodulator (CARRIER_EN), on channel clock ticks, takes the level
 *   CARRIER_OUT_LV at once and the other level only once it has lasted
 *   longer than CARRIER_LOW_THRES + 1 ticks (CARRIER_HIGH_THRES + 1 for a
 *   low carrier), the carrier's gap as ESP-IDF programs it, so the gaps
 *   of a carrier are bridged and the end of a carried stretch is seen that
 *   many ticks late, the TRM placing the demodulator before the edge
 *   detector; then the recorder. The first edge after RX_EN starts a
 *   frame; from then on an entry is written for each level when it ends.
 *   With no edge for more than IDLE_THRES ticks the receiver writes an
 *   end marker with the current level, raises RX_END and waits for the
 *   next frame. A level lasting longer than an entry holds (32767 ticks)
 *   is written as entries of 32767 ticks.
 * - The write pointer counts the words holding the frame: an end marker
 *   in the low half moves it past the word. RX_THR_EVENT is raised each
 *   time RX_LIM more words have been filled by entries, not counting the
 *   word holding the end marker (ESP-IDF's rmt_rx.c notes the count is
 *   one short of the pulses). MEM_WR_RST restarts that count.
 * - The receiver writes only while MEM_OWNER gives the memory to it;
 *   otherwise it sets MEM_OWNER_ERR, raises ERR and drops the entry. With
 *   its memory full and no MEM_RX_WRAP_EN it sets MEM_FULL and raises ERR
 *   for each entry it drops. MEM_OWNER is kept as written: the TRM does
 *   not say what clears it.
 * - The APB FIFO: writes to a TX channel's DATA register write its
 *   memory, reads of an RX channel's read it, each through its own
 *   pointer (APB_MEM_WADDR, APB_MEM_RADDR), which wraps with the channel's
 *   wrap mode and otherwise sets APB_MEM_WR_ERR or APB_MEM_RD_ERR at the
 *   end of the channel's memory. RMT_SYS_CONF.APB_FIFO_MASK selects
 *   between that and direct access to the RAM at 0x800; the other path
 *   reads 0 and drops writes.
 * - DMA (channels 3 and 7, GDMA-AHB's PERI_SEL 10): with TX channel 3's
 *   DMA_ACCESS_EN, GDMA's data is written through its APB write pointer
 *   as long as the words not yet sent leave room, so its memory is a
 *   FIFO in wrap mode and the transmitter reads what is there (stale words
 *   if DMA has not kept up, as on the hardware). DMA_ACCESS_FAIL for
 *   channel 3 is never raised: data is only taken into free words. With
 *   RX channel 7's DMA_ACCESS_EN, each word the receiver fills is given
 *   to GDMA through the APB read pointer, the word holding an end marker
 *   as 2 bytes (end marker in the low half) or 4, closing the frame
 *   (in_suc_eof); a word overwritten before GDMA took it raises
 *   DMA_ACCESS_FAIL for channel 7. APB FIFO access to a channel in DMA
 *   mode, and direct access to channel 3's memory, are logged and have no
 *   effect, the TRM calling their result unpredictable.
 * - MEM_FORCE_PD powers the RAM down: its contents are lost, the CPU reads
 *   0 and its writes are dropped, the transmitter reads end markers and the
 *   receiver's writes are dropped. MEM_FORCE_PU and MEM_CLK_FORCE_ON only
 *   matter to the PMU's light sleep, which is not modelled.
 * - Changing a channel's divider through CONF_UPDATE or REF_CNT_RST
 *   restarts its divider count: a channel clock tick at that moment.
 * - The STATE fields read 0 while idle, 1 while transmitting and 3 while
 *   receiving a frame, as on the ESP32.
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
#include "hw/misc/esp32p4_rmt.h"
#include "migration/vmstate.h"

#define NTX                     ESP32P4_RMT_TX_CHANNELS
#define NRX                     ESP32P4_RMT_RX_CHANNELS
#define BLOCK                   ESP32P4_RMT_BLOCK_WORDS
#define RAM_WORDS               ESP32P4_RMT_RAM_WORDS
#define DMA_TX                  3
#define DMA_RX                  3

#define A_TX_DATA(n)            (0x00 + 4 * (n))
#define A_RX_DATA(m)            (0x10 + 4 * (m))
#define A_TX_CONF0(n)           (0x20 + 4 * (n))
#define A_RX_CONF0(m)           (0x30 + 8 * (m))
#define A_RX_CONF1(m)           (0x34 + 8 * (m))
#define A_TX_STATUS(n)          (0x50 + 4 * (n))
#define A_RX_STATUS(m)          (0x60 + 4 * (m))
#define A_INT_RAW               0x70
#define A_INT_ST                0x74
#define A_INT_ENA               0x78
#define A_INT_CLR               0x7c
#define A_TX_CARRIER_DUTY(n)    (0x80 + 4 * (n))
#define A_RX_CARRIER_RM(m)      (0x90 + 4 * (m))
#define A_TX_LIM(n)             (0xa0 + 4 * (n))
#define A_RX_LIM(m)             (0xb0 + 4 * (m))
#define A_SYS_CONF              0xc0
#define A_TX_SIM                0xc4
#define A_REF_CNT_RST           0xc8
#define A_DATE                  0xcc

/* RMT_CHnCONF0 (TX) */
#define TX_START                (1u << 0)
#define TX_MEM_RD_RST           (1u << 1)
#define TX_APB_MEM_RST          (1u << 2)
#define TX_CONTI_MODE           (1u << 3)
#define TX_MEM_WRAP_EN          (1u << 4)
#define TX_IDLE_OUT_LV          (1u << 5)
#define TX_IDLE_OUT_EN          (1u << 6)
#define TX_STOP                 (1u << 7)
#define TX_DIV_SHIFT            8
#define TX_MEM_SIZE_SHIFT       16
#define TX_CARRIER_EFF_EN       (1u << 20)
#define TX_CARRIER_EN           (1u << 21)
#define TX_CARRIER_OUT_LV       (1u << 22)
#define TX_AFIFO_RST            (1u << 23)
#define TX_CONF_UPDATE          (1u << 24)
#define TX_DMA_ACCESS_EN        (1u << 25)
#define TX_CONF0_RW             0x007ffff8u
#define TX_CONF0_LATCHED        0x0070ffe8u
#define TX_CONF0_RESET          0x00710200u

/* RMT_CHmCONF0 (RX) */
#define RX_DIV_SHIFT            0
#define RX_IDLE_THRES_SHIFT     8
#define RX_IDLE_THRES_MASK      0x7fffu
#define RX_DMA_ACCESS_EN        (1u << 23)
#define RX_MEM_SIZE_SHIFT       24
#define RX_CARRIER_EN           (1u << 28)
#define RX_CARRIER_OUT_LV       (1u << 29)
#define RX_CONF0_RW             0x3f7fffffu
#define RX_CONF0_LATCHED        0x307fffffu
#define RX_CONF0_RESET          0x317fff02u

/* RMT_CHmCONF1 (RX) */
#define RX_EN                   (1u << 0)
#define RX_MEM_WR_RST           (1u << 1)
#define RX_APB_MEM_RST          (1u << 2)
#define RX_MEM_OWNER            (1u << 3)
#define RX_FILTER_EN            (1u << 4)
#define RX_FILTER_THRES_SHIFT   5
#define RX_FILTER_THRES_MASK    0xffu
#define RX_MEM_WRAP_EN          (1u << 13)
#define RX_AFIFO_RST            (1u << 14)
#define RX_CONF_UPDATE          (1u << 15)
#define RX_CONF1_RW             0x00003ff9u
#define RX_CONF1_LATCHED        0x00001fe1u
#define RX_CONF1_RESET          0x000001e8u

/* RMT_CHnSTATUS and RMT_CHmSTATUS */
#define ST_ADDR_MASK            0x3ffu
#define ST_APB_ADDR_SHIFT       11
#define ST_STATE_SHIFT          22
#define ST_TX_MEM_EMPTY         (1u << 25)
#define ST_TX_APB_MEM_WR_ERR    (1u << 26)
#define ST_RX_MEM_OWNER_ERR     (1u << 25)
#define ST_RX_MEM_FULL          (1u << 26)
#define ST_RX_APB_MEM_RD_ERR    (1u << 27)
#define STATE_IDLE              0
#define STATE_SEND              1
#define STATE_RECEIVE           3
/* The pointers' reset values (rmt_reg.h): 0 for TX, 192 for RX. */
#define TX_ADDR_RESET           0
#define RX_ADDR_RESET           192

/* RMT_INT_* */
#define INT_TX_END(n)           (1u << (n))
#define INT_TX_ERR(n)           (1u << (4 + (n)))
#define INT_TX_THR(n)           (1u << (8 + (n)))
#define INT_TX_LOOP(n)          (1u << (12 + (n)))
#define INT_RX_END(m)           (1u << (16 + (m)))
#define INT_RX_ERR(m)           (1u << (20 + (m)))
#define INT_RX_THR(m)           (1u << (24 + (m)))
#define INT_CH3_DMA_FAIL        (1u << 28)
#define INT_CH7_DMA_FAIL        (1u << 29)
#define INT_MASK                0x3fffffffu

#define CARRIER_DUTY_RESET      0x00400040u

/* RMT_CHn_TX_LIM */
#define TXL_LIM_MASK            0x1ffu
#define TXL_LOOP_NUM_SHIFT      9
#define TXL_LOOP_NUM_MASK       0x3ffu
#define TXL_LOOP_CNT_EN         (1u << 19)
#define TXL_LOOP_COUNT_RESET    (1u << 20)
#define TXL_LOOP_STOP_EN        (1u << 21)
#define TXL_RW                  0x002fffffu
#define TXL_LATCHED             0x000fffffu
#define TXL_RESET               0x00000080u

#define RXL_MASK                0x1ffu
#define RXL_RESET               0x80u

/* RMT_SYS_CONF */
#define SYS_APB_FIFO_MASK       (1u << 0)
#define SYS_MEM_FORCE_PD        (1u << 2)
#define SYS_CONF_RW             0x87ffffffu
#define SYS_CONF_RESET          0x05000010u

#define TX_SIM_CH_MASK          0xfu
#define TX_SIM_EN               (1u << 4)
#define TX_SIM_RW               0x1fu

#define DATE_MASK               0x0fffffffu
#define DATE_RESET              0x02201111u

#define ENTRY_LEVEL             (1u << 15)
#define ENTRY_PERIOD_MASK       0x7fffu

#define NEVER                   INT64_MAX

enum {
    TX_IDLE,
    /* Waiting for the channel clock tick on which a pass starts */
    TX_STARTING,
    TX_ENTRY,
};

/* Clocks and cycles */

/* The rmt_sclk cycle a channel anchored at (a_ns, a_c) is in at time t */
static uint64_t cyc_at(ESP32P4RmtState *s, int64_t a_ns, uint64_t a_c,
                       int64_t t)
{
    if (t <= a_ns) {
        return a_c;
    }
    return a_c + esp32p4_clk_edges_between(&s->sclk_edges, a_ns, t);
}

/* The first time at which such a channel has reached cycle cyc */
static int64_t cyc_time(ESP32P4RmtState *s, int64_t a_ns, uint64_t a_c,
                        uint64_t cyc)
{
    int64_t ns;

    if (cyc <= a_c) {
        return a_ns;
    }
    ns = esp32p4_clk_edges_after(&s->sclk_edges, a_ns, cyc - a_c);
    return MIN(ns, NEVER);
}

/* The first channel clock tick at or after cycle cyc */
static uint64_t tick_ceil(uint64_t cyc, uint32_t div, uint32_t phase)
{
    if (cyc <= phase) {
        return phase;
    }
    return phase + DIV_ROUND_UP(cyc - phase, div) * div;
}

static uint32_t div_of(uint32_t field)
{
    field &= 0xff;
    return field ? field : 256;
}

static void raise_int(ESP32P4RmtState *s, uint32_t bits)
{
    s->int_raw |= bits;
}

static void update_irq(ESP32P4RmtState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static bool ram_pd(ESP32P4RmtState *s)
{
    return (s->sys_conf & SYS_MEM_FORCE_PD) != 0;
}

/* The RAM range of channel ch (0 to 7), [base, end), in words */
static uint32_t mem_base(unsigned ch)
{
    return ch * BLOCK;
}

static uint32_t mem_end(unsigned ch, uint32_t mem_size)
{
    return MIN(mem_base(ch) + mem_size * BLOCK, RAM_WORDS);
}

/* Transmitter */

static uint32_t tx_div(ESP32P4RmtTx *c)
{
    return div_of(c->act_conf0 >> TX_DIV_SHIFT);
}

static uint32_t tx_end(ESP32P4RmtTx *c, unsigned n)
{
    return mem_end(n, (c->conf0 >> TX_MEM_SIZE_SHIFT) & 0xf);
}

static uint64_t tx_cyc_at(ESP32P4RmtState *s, unsigned n, int64_t t)
{
    return cyc_at(s, s->tx[n].anchor_ns, s->tx[n].anchor_c, t);
}

static int64_t tx_cyc_time(ESP32P4RmtState *s, unsigned n, uint64_t cyc)
{
    return cyc_time(s, s->tx[n].anchor_ns, s->tx[n].anchor_c, cyc);
}

/* Drive TX channel n's output to level from time t. */
static void set_out(ESP32P4RmtState *s, unsigned n, uint8_t level, int64_t t)
{
    ESP32P4RmtTx *c = &s->tx[n];

    if (c->out_level == level) {
        return;
    }
    c->out_level = level;
    esp32_line_set_at(s->sig_out[n], level, t);
}

/* The output before carrier modulation */
static uint8_t tx_unmod(ESP32P4RmtTx *c)
{
    if (c->state == TX_ENTRY) {
        return c->level;
    }
    if (c->act_conf0 & TX_IDLE_OUT_EN) {
        return (c->act_conf0 & TX_IDLE_OUT_LV) != 0;
    }
    return c->end_level;
}

static bool tx_carried(ESP32P4RmtTx *c)
{
    return (c->act_conf0 & TX_CARRIER_EN) &&
           tx_unmod(c) == ((c->act_conf0 & TX_CARRIER_OUT_LV) != 0) &&
           (c->state == TX_ENTRY || !(c->act_conf0 & TX_CARRIER_EFF_EN));
}

static uint32_t carrier_cycles(ESP32P4RmtTx *c, bool high)
{
    uint32_t v = high ? c->act_carrier_duty >> 16
                      : c->act_carrier_duty & 0xffff;

    return v ? v : 0x10000;
}

/* Start, keep or stop the carrier as the output's level asks, at cyc. */
static void tx_carrier_eval(ESP32P4RmtTx *c, uint64_t cyc)
{
    if (!tx_carried(c)) {
        c->carrier = false;
    } else if (!c->carrier) {
        c->carrier = true;
        c->carrier_high = true;
        c->carrier_end_c = cyc + carrier_cycles(c, true);
    }
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void tx_update_out(ESP32P4RmtState *s, unsigned n, int64_t t)
{
    ESP32P4RmtTx *c = &s->tx[n];

    set_out(s, n, c->carrier ? c->carrier_high : tx_unmod(c), t);
}

/* The next event of TX channel n, in its cycles, if it has one */
static bool tx_event(ESP32P4RmtTx *c, uint64_t *cyc)
{
    switch (c->state) {
    case TX_STARTING:
        *cyc = c->event_c;
        return true;
    case TX_ENTRY:
        *cyc = c->entry_end_c;
        if (c->carrier && c->carrier_end_c < *cyc) {
            *cyc = c->carrier_end_c;
        }
        return true;
    default:
        if (c->carrier) {
            *cyc = c->carrier_end_c;
            return true;
        }
        return false;
    }
}

static void rx_dma_push(ESP32P4RmtState *s);
static void tx_dma_pull(ESP32P4RmtState *s);

static void tx_stop(ESP32P4RmtState *s, unsigned n, uint64_t cyc)
{
    ESP32P4RmtTx *c = &s->tx[n];

    c->state = TX_IDLE;
    tx_carrier_eval(c, cyc);
}

/*
 * Fetch the word at the read pointer, which then names the next. At the
 * end of the channel's memory the pointer starts over in wrap and
 * continuous modes; otherwise the transmitter stops with MEM_EMPTY and
 * ERR.
 */
static bool tx_fetch(ESP32P4RmtState *s, unsigned n, uint64_t cyc)
{
    ESP32P4RmtTx *c = &s->tx[n];

    if (c->raddr >= tx_end(c, n)) {
        if (!(c->conf0 & TX_MEM_WRAP_EN) &&
            !(c->act_conf0 & TX_CONTI_MODE)) {
            c->mem_empty = true;
            raise_int(s, INT_TX_ERR(n));
            tx_stop(s, n, cyc);
            return false;
        }
        c->raddr = mem_base(n);
    }
    c->word = ram_pd(s) ? 0 : s->ram[c->raddr];
    c->raddr++;
    c->half = 0;
    if (++c->thr_count == (c->act_tx_lim & TXL_LIM_MASK)) {
        c->thr_count = 0;
        raise_int(s, INT_TX_THR(n));
    }
    if (n == DMA_TX && (c->conf0 & TX_DMA_ACCESS_EN)) {
        if (c->dma_fill) {
            c->dma_fill--;
        }
        tx_dma_pull(s);
    }
    return true;
}

static void tx_begin_entry(ESP32P4RmtState *s, unsigned n, uint64_t cyc);

/* Start a pass from the start of the channel's memory at cycle cyc. */
static void tx_pass(ESP32P4RmtState *s, unsigned n, uint64_t cyc)
{
    ESP32P4RmtTx *c = &s->tx[n];

    c->raddr = mem_base(n);
    c->half = 0;
    c->pass_sent = false;
    if (tx_fetch(s, n, cyc)) {
        tx_begin_entry(s, n, cyc);
    }
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void tx_end_marker(ESP32P4RmtState *s, unsigned n, uint64_t cyc,
                          uint8_t level)
{
    ESP32P4RmtTx *c = &s->tx[n];
    bool sent = c->pass_sent;

    c->end_level = level;
    c->raddr = mem_base(n);
    c->half = 0;
    raise_int(s, INT_TX_END(n));
    if (!(c->act_conf0 & TX_CONTI_MODE)) {
        tx_stop(s, n, cyc);
        return;
    }
    if (c->act_tx_lim & TXL_LOOP_CNT_EN) {
        uint32_t num = (c->act_tx_lim >> TXL_LOOP_NUM_SHIFT) &
                       TXL_LOOP_NUM_MASK;

        if (++c->loop_count == num) {
            c->loop_count = 0;
            raise_int(s, INT_TX_LOOP(n));
            if (c->tx_lim & TXL_LOOP_STOP_EN) {
                tx_stop(s, n, cyc);
                return;
            }
        }
    }
    c->thr_count = 0;
    if (sent) {
        tx_pass(s, n, cyc);
    } else {
        /* A pass of nothing but an end marker takes a tick. */
        c->state = TX_STARTING;
        c->event_c = cyc + tx_div(c);
        tx_carrier_eval(c, cyc);
    }
}

/* Start sending the current entry at cycle cyc. */
static void tx_begin_entry(ESP32P4RmtState *s, unsigned n, uint64_t cyc)
{
    ESP32P4RmtTx *c = &s->tx[n];
    uint32_t e = c->half ? c->word >> 16 : c->word & 0xffff;
    uint32_t period = e & ENTRY_PERIOD_MASK;
    uint8_t level = (e & ENTRY_LEVEL) != 0;

    if (!period) {
        tx_end_marker(s, n, cyc, level);
        return;
    }
    c->state = TX_ENTRY;
    c->pass_sent = true;
    c->level = level;
    c->entry_end_c = cyc + (uint64_t)period * tx_div(c);
    tx_carrier_eval(c, cyc);
}

/* Process TX channel n's event at cycle cyc. */
static void tx_step(ESP32P4RmtState *s, unsigned n, uint64_t cyc)
{
    ESP32P4RmtTx *c = &s->tx[n];

    if (c->state == TX_STARTING) {
        tx_pass(s, n, cyc);
        return;
    }
    if (c->carrier && c->carrier_end_c == cyc) {
        c->carrier_high = !c->carrier_high;
        c->carrier_end_c = cyc + carrier_cycles(c, c->carrier_high);
    }
    if (c->state != TX_ENTRY || c->entry_end_c != cyc) {
        return;
    }
    if (c->half == 0) {
        c->half = 1;
    } else if (!tx_fetch(s, n, cyc)) {
        return;
    }
    tx_begin_entry(s, n, cyc);
}

/*
 * Run the transmitters up to time t, their events across channels in
 * time order, so that their edges reach the GPIO matrix in order.
 */
static void tx_advance(ESP32P4RmtState *s, int64_t t)
{
    for (;;) {
        int best = -1;
        int64_t best_t = NEVER;
        uint64_t best_c = 0;

        for (unsigned n = 0; n < NTX; n++) {
            uint64_t cyc;
            int64_t at;

            if (!tx_event(&s->tx[n], &cyc)) {
                continue;
            }
            at = tx_cyc_time(s, n, cyc);
            if (at <= t && at < best_t) {
                best = n;
                best_t = at;
                best_c = cyc;
            }
        }
        if (best < 0) {
            return;
        }
        tx_step(s, best, best_c);
        tx_update_out(s, best, best_t);
    }
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void tx_start(ESP32P4RmtState *s, unsigned n, int64_t now)
{
    ESP32P4RmtTx *c = &s->tx[n];
    uint32_t div = tx_div(c);

    if (!((c->conf0 >> TX_MEM_SIZE_SHIFT) & 0xf)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TX channel %u transmits with "
                      "MEM_SIZE 0\n", TYPE_ESP32P4_RMT, n);
    }
    c->mem_empty = false;
    c->thr_count = 0;
    c->state = TX_STARTING;
    c->event_c = tick_ceil(tx_cyc_at(s, n, now) + 1, div, c->div_phase % div);
}

/* TX_START written for channel n */
static void tx_start_req(ESP32P4RmtState *s, unsigned n, int64_t now)
{
    uint32_t group = s->tx_sim & TX_SIM_CH_MASK;

    if (s->tx[n].state != TX_IDLE) {
        return;
    }
    if (!s->act_tx_sim_en || !(group & BIT(n))) {
        tx_start(s, n, now);
        return;
    }
    s->tx_armed |= BIT(n);
    if ((s->tx_armed & group) != group) {
        return;
    }
    for (unsigned i = 0; i < NTX; i++) {
        if (group & BIT(i) && s->tx[i].state == TX_IDLE) {
            tx_start(s, i, now);
        }
    }
    s->tx_armed &= ~group;
}

/* Restart channel n's divider count: a channel clock tick at time now. */
static void tx_div_reset(ESP32P4RmtState *s, unsigned n, int64_t now)
{
    ESP32P4RmtTx *c = &s->tx[n];

    c->div_phase = tx_cyc_at(s, n, now) % tx_div(c);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void tx_conf_update(ESP32P4RmtState *s, unsigned n, int64_t now)
{
    ESP32P4RmtTx *c = &s->tx[n];
    uint32_t old = c->act_conf0;

    c->act_conf0 = c->conf0 & TX_CONF0_LATCHED;
    c->act_carrier_duty = c->carrier_duty;
    c->act_tx_lim = c->tx_lim & TXL_LATCHED;
    s->act_tx_sim_en = (s->tx_sim & TX_SIM_EN) != 0;
    if ((old ^ c->act_conf0) & (0xffu << TX_DIV_SHIFT)) {
        tx_div_reset(s, n, now);
    }
    if (c->act_conf0 & TX_STOP) {
        c->conf0 &= ~TX_STOP;
        c->act_conf0 &= ~TX_STOP;
        s->tx_armed &= ~BIT(n);
        tx_stop(s, n, tx_cyc_at(s, n, now));
    }
    if ((old ^ c->act_conf0) & (TX_CARRIER_EN | TX_CARRIER_OUT_LV |
                                TX_CARRIER_EFF_EN | TX_IDLE_OUT_EN |
                                TX_IDLE_OUT_LV)) {
        /* A carrier whose setting changes starts afresh. */
        c->carrier = false;
    }
    tx_carrier_eval(c, tx_cyc_at(s, n, now));
    tx_update_out(s, n, now);
}

/* Receiver */

static uint32_t rx_div(ESP32P4RmtRx *c)
{
    return div_of(c->act_conf0 >> RX_DIV_SHIFT);
}

static uint32_t rx_base(unsigned m)
{
    return mem_base(NTX + m);
}

static uint32_t rx_end(ESP32P4RmtRx *c, unsigned m)
{
    return mem_end(NTX + m, (c->conf0 >> RX_MEM_SIZE_SHIFT) & 0xf);
}

static bool rx_enabled(ESP32P4RmtRx *c)
{
    return (c->act_conf1 & RX_EN) != 0;
}

static bool rx_dma(ESP32P4RmtState *s, unsigned m)
{
    return m == DMA_RX && (s->rx[m].conf0 & RX_DMA_ACCESS_EN);
}

static uint64_t rx_cyc_at(ESP32P4RmtState *s, unsigned m, int64_t t)
{
    return cyc_at(s, s->rx[m].anchor_ns, s->rx[m].anchor_c, t);
}

static int64_t rx_cyc_time(ESP32P4RmtState *s, unsigned m, uint64_t cyc)
{
    return cyc_time(s, s->rx[m].anchor_ns, s->rx[m].anchor_c, cyc);
}

static void rx_div_reset(ESP32P4RmtState *s, unsigned m, int64_t now)
{
    ESP32P4RmtRx *c = &s->rx[m];

    c->div_phase = rx_cyc_at(s, m, now) % rx_div(c);
}

/* Advance a channel's APB pointer past word a, wrapping if wrap. */
static uint32_t ptr_next(uint32_t a, uint32_t base, uint32_t end, bool wrap)
{
    a++;
    if (a >= end && wrap) {
        a = base;
    }
    return a;
}

/*
 * Write an entry at the receiver's write pointer. A word is complete when
 * its high half is written or it holds an end marker, which moves the
 * pointer past it.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void rx_write(ESP32P4RmtState *s, unsigned m, uint32_t entry)
{
    ESP32P4RmtRx *c = &s->rx[m];
    bool wrap = (c->conf1 & RX_MEM_WRAP_EN) != 0;
    bool marker = !(entry & ENTRY_PERIOD_MASK);
    uint32_t end = rx_end(c, m);
    uint32_t *w;

    if (!(c->conf1 & RX_MEM_OWNER)) {
        c->owner_err = true;
        raise_int(s, INT_RX_ERR(m));
        return;
    }
    if (c->waddr >= end) {
        c->mem_full = true;
        raise_int(s, INT_RX_ERR(m));
        return;
    }
    w = &s->ram[c->waddr];
    if (!c->half) {
        s->eof_bytes[c->waddr] = 0;
    }
    if (!ram_pd(s)) {
        if (c->half) {
            *w = (*w & 0xffff) | (entry << 16);
        } else {
            *w = (*w & 0xffff0000u) | entry;
        }
    }
    if (!c->half && !marker) {
        c->half = 1;
        return;
    }
    if (marker && rx_dma(s, m)) {
        s->eof_bytes[c->waddr] = c->half ? 4 : 2;
    }
    c->half = 0;
    if (!marker && ++c->thr_count == (c->act_rx_lim & RXL_MASK)) {
        c->thr_count = 0;
        raise_int(s, INT_RX_THR(m));
    }
    c->waddr = ptr_next(c->waddr, rx_base(m), end, wrap);
    if (rx_dma(s, m)) {
        if (c->dma_unread < end - rx_base(m)) {
            c->dma_unread++;
        } else {
            /* The word GDMA was to take next has been overwritten. */
            raise_int(s, INT_CH7_DMA_FAIL);
        }
        rx_dma_push(s);
    }
}

/*
 * The receiver's next own event: the cycle at which the frame goes idle,
 * or at which the current level fills an entry.
 */
static uint64_t rx_record_event(ESP32P4RmtRx *c, bool *idle)
{
    uint64_t div = rx_div(c);
    uint64_t thres = (c->act_conf0 >> RX_IDLE_THRES_SHIFT) &
                     RX_IDLE_THRES_MASK;
    uint64_t idle_c = c->edge_c + (thres + 1) * div;
    uint64_t full_c = c->edge_c + ENTRY_PERIOD_MASK * div;

    *idle = idle_c <= full_c;
    return *idle ? idle_c : full_c;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void rx_record(ESP32P4RmtState *s, unsigned m, uint64_t cyc)
{
    ESP32P4RmtRx *c = &s->rx[m];
    bool idle;

    rx_record_event(c, &idle);
    if (idle) {
        c->frame = false;
        rx_write(s, m, c->level ? ENTRY_LEVEL : 0);
        raise_int(s, INT_RX_END(m));
    } else {
        rx_write(s, m, (c->level ? ENTRY_LEVEL : 0) | ENTRY_PERIOD_MASK);
        c->edge_c = cyc;
    }
}

/* An edge of the demodulated input to level at channel clock tick cyc */
static void rx_edge(ESP32P4RmtState *s, unsigned m, uint64_t cyc,
                    uint8_t level)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint64_t ticks;

    c->demod = level;
    if (!c->frame) {
        c->frame = true;
        c->level = level;
        c->edge_c = cyc;
        return;
    }
    ticks = (cyc - c->edge_c) / rx_div(c);
    if (ticks) {
        rx_write(s, m, (c->level ? ENTRY_LEVEL : 0) |
                       MIN(ticks, ENTRY_PERIOD_MASK));
        c->edge_c = cyc;
    }
    c->level = level;
}

/*
 * [spec:nuos:req:emu.esp32p4.rmt-pcnt]
 * The sampled input changed to level at tick cyc: demodulate. The carrier
 * level is taken at once; the other level once it has lasted longer than
 * the gap threshold, a sample back at the carrier level on the tick the
 * threshold ends still bridging the gap.
 */
static void rx_sampled(ESP32P4RmtState *s, unsigned m, uint64_t cyc,
                       uint8_t level)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint8_t carrier = (c->act_conf0 & RX_CARRIER_OUT_LV) != 0;
    uint32_t thres;

    c->in_samp = level;
    if (!(c->act_conf0 & RX_CARRIER_EN)) {
        rx_edge(s, m, cyc, level);
        return;
    }
    if (level == carrier) {
        c->demod_pending = false;
        if (c->demod != level) {
            rx_edge(s, m, cyc, level);
        }
        return;
    }
    if (c->demod == level) {
        return;
    }
    thres = carrier ? c->act_carrier_rm & 0xffff : c->act_carrier_rm >> 16;
    c->demod_pending = true;
    c->demod_c = cyc + ((uint64_t)thres + 1) * rx_div(c);
}

/* The filter's output changed to level at time t: sample it. */
static void rx_filtered(ESP32P4RmtState *s, unsigned m, int64_t t,
                        uint8_t level)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint32_t div = rx_div(c);
    uint64_t cyc = rx_cyc_at(s, m, t);

    c->in_filt = level;
    if (rx_cyc_time(s, m, cyc) < t) {
        cyc++;
    }
    cyc = tick_ceil(cyc, div, c->div_phase % div);
    if (c->samp_pending) {
        /* Back to the sampled level before the tick: the pulse is lost */
        c->samp_pending = false;
    } else if (level != c->in_samp) {
        c->samp_pending = true;
        c->samp_c = cyc;
    }
}

/* The raw input changed to level at time t. */
static void rx_raw(ESP32P4RmtState *s, unsigned m, int64_t t, uint8_t level)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint32_t thres = (c->act_conf1 >> RX_FILTER_THRES_SHIFT) &
                     RX_FILTER_THRES_MASK;

    c->in_raw = level;
    if (!(c->conf1 & RX_FILTER_EN) || !thres) {
        if (level != c->in_filt) {
            rx_filtered(s, m, t, level);
        }
        return;
    }
    if (c->filt_pending) {
        /* The pulse was shorter than RX_FILTER_THRES */
        c->filt_pending = false;
    } else if (level != c->in_filt) {
        c->filt_pending = true;
        c->filt_ns = esp32p4_clk_edges_running(&s->sclk_edges) ?
                     MIN(esp32p4_clk_edges_after(&s->sclk_edges, t, thres),
                         NEVER) : NEVER;
    }
}

/* Run RX channel m up to time t. */
static void rx_advance(ESP32P4RmtState *s, unsigned m, int64_t t)
{
    ESP32P4RmtRx *c = &s->rx[m];

    if (!rx_enabled(c)) {
        return;
    }
    for (;;) {
        int64_t filt_t = c->filt_pending ? c->filt_ns : NEVER;
        int64_t samp_t = c->samp_pending ? rx_cyc_time(s, m, c->samp_c)
                                         : NEVER;
        int64_t demod_t = c->demod_pending ? rx_cyc_time(s, m, c->demod_c)
                                           : NEVER;
        int64_t rec_t = NEVER;
        uint64_t rec_c = 0;
        bool idle;

        if (c->frame) {
            rec_c = rx_record_event(c, &idle);
            rec_t = rx_cyc_time(s, m, rec_c);
        }
        if (filt_t != NEVER && filt_t <= t && filt_t <= samp_t &&
            filt_t <= demod_t && filt_t <= rec_t) {
            c->filt_pending = false;
            rx_filtered(s, m, filt_t, !c->in_filt);
        } else if (samp_t != NEVER && samp_t <= t && samp_t <= demod_t &&
                   samp_t <= rec_t) {
            c->samp_pending = false;
            rx_sampled(s, m, c->samp_c, !c->in_samp);
        } else if (demod_t != NEVER && demod_t <= t && demod_t <= rec_t) {
            c->demod_pending = false;
            rx_edge(s, m, c->demod_c, c->in_samp);
        } else if (rec_t != NEVER && rec_t <= t) {
            rx_record(s, m, rec_c);
        } else {
            break;
        }
    }
}

static int64_t rx_next_event(ESP32P4RmtState *s, unsigned m)
{
    ESP32P4RmtRx *c = &s->rx[m];
    int64_t t = NEVER;
    bool idle;

    if (!rx_enabled(c)) {
        return NEVER;
    }
    if (c->filt_pending) {
        t = MIN(t, c->filt_ns);
    }
    if (c->samp_pending) {
        t = MIN(t, rx_cyc_time(s, m, c->samp_c));
    }
    if (c->demod_pending) {
        t = MIN(t, rx_cyc_time(s, m, c->demod_c));
    }
    if (c->frame) {
        t = MIN(t, rx_cyc_time(s, m, rx_record_event(c, &idle)));
    }
    return t;
}

/* Drop any frame under way and start the pipeline at the input's level. */
static void rx_restart(ESP32P4RmtRx *c)
{
    c->frame = false;
    c->filt_pending = false;
    c->samp_pending = false;
    c->demod_pending = false;
    c->in_filt = c->in_samp = c->demod = c->in_raw;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void rx_conf_update(ESP32P4RmtState *s, unsigned m, int64_t now)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint32_t old0 = c->act_conf0, old1 = c->act_conf1;

    c->act_conf0 = c->conf0 & RX_CONF0_LATCHED;
    c->act_conf1 = c->conf1 & RX_CONF1_LATCHED;
    c->act_carrier_rm = c->carrier_rm;
    c->act_rx_lim = c->rx_lim & RXL_MASK;
    if ((old0 ^ c->act_conf0) & 0xffu || c->div_rst_pending) {
        c->div_rst_pending = false;
        rx_div_reset(s, m, now);
    }
    if ((old1 ^ c->act_conf1) & RX_EN) {
        rx_restart(c);
    } else if ((old0 ^ c->act_conf0) & (RX_CARRIER_EN | RX_CARRIER_OUT_LV)) {
        c->demod_pending = false;
        if (c->demod != c->in_samp && c->frame) {
            rx_edge(s, m, rx_cyc_at(s, m, now), c->in_samp);
        }
        c->demod = c->in_samp;
    }
}

/* DMA */

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static bool tx_apb_write(ESP32P4RmtState *s, unsigned n, uint32_t v)
{
    ESP32P4RmtTx *c = &s->tx[n];
    uint32_t end = tx_end(c, n);

    if (c->apb_waddr >= end) {
        c->apb_wr_err = true;
        return false;
    }
    if (!ram_pd(s)) {
        s->ram[c->apb_waddr] = v;
    }
    c->apb_waddr = ptr_next(c->apb_waddr, mem_base(n), end,
                            c->conf0 & TX_MEM_WRAP_EN);
    return true;
}

/*
 * TX channel 3 in DMA mode: take GDMA's bytes into the words the
 * transmitter has finished with, a word at a time.
 */
static void tx_dma_pull(ESP32P4RmtState *s)
{
    ESP32P4RmtTx *c = &s->tx[DMA_TX];
    uint32_t words = tx_end(c, DMA_TX) - MIN(mem_base(DMA_TX),
                                             tx_end(c, DMA_TX));
    uint8_t buf[4 * RAM_WORDS];

    if (!s->gdma || !(c->conf0 & TX_DMA_ACCESS_EN)) {
        return;
    }
    while (c->dma_fill < words) {
        size_t want = 4 * (words - c->dma_fill) - c->dma_bytes;
        bool eof;
        size_t got = esp32p4_gdma_pull(s->gdma, ESP32P4_GDMA_AHB_RMT, buf,
                                       want, &eof);

        for (size_t i = 0; i < got; i++) {
            c->dma_word |= (uint32_t)buf[i] << (8 * c->dma_bytes);
            if (++c->dma_bytes == 4 || (eof && i == got - 1)) {
                if (tx_apb_write(s, DMA_TX, c->dma_word)) {
                    c->dma_fill++;
                }
                c->dma_word = 0;
                c->dma_bytes = 0;
            }
        }
        if (!got) {
            break;
        }
    }
}

/*
 * RX channel 7 in DMA mode: give GDMA the words the receiver has filled,
 * the word holding an end marker closing the frame.
 */
static void rx_dma_push(ESP32P4RmtState *s)
{
    ESP32P4RmtRx *c = &s->rx[DMA_RX];
    uint32_t base = rx_base(DMA_RX), end = rx_end(c, DMA_RX);
    bool wrap = (c->conf1 & RX_MEM_WRAP_EN) != 0;

    if (!s->gdma || !rx_dma(s, DMA_RX)) {
        return;
    }
    while (c->dma_unread) {
        uint32_t a = c->apb_raddr;
        uint32_t len, w;
        uint8_t buf[4];

        if (a >= end) {
            c->apb_rd_err = true;
            c->dma_unread = 0;
            return;
        }
        len = s->eof_bytes[a] ? s->eof_bytes[a] : 4;
        if (esp32p4_gdma_rx_room(s->gdma, ESP32P4_GDMA_AHB_RMT) < len) {
            return;
        }
        w = ram_pd(s) ? 0 : s->ram[a];
        for (unsigned i = 0; i < 4; i++) {
            buf[i] = w >> (8 * i);
        }
        esp32p4_gdma_push(s->gdma, ESP32P4_GDMA_AHB_RMT, buf, len,
                          s->eof_bytes[a] ? ESP32P4_GDMA_EOF : 0);
        s->eof_bytes[a] = 0;
        c->apb_raddr = ptr_next(a, base, end, wrap);
        c->dma_unread--;
    }
}

/* Synchronisation */

/*
 * Bring every channel up to time now: the transmitters first, so that a
 * receiver sees the edges they sent before its own later events.
 */
static void rmt_sync(ESP32P4RmtState *s, int64_t now)
{
    if (s->syncing) {
        return;
    }
    s->syncing = true;
    tx_advance(s, now);
    for (unsigned m = 0; m < NRX; m++) {
        rx_advance(s, m, now);
    }
    s->syncing = false;
}

static void rmt_schedule(ESP32P4RmtState *s)
{
    int64_t next = NEVER;

    for (unsigned n = 0; n < NTX; n++) {
        uint64_t cyc;

        if (tx_event(&s->tx[n], &cyc)) {
            next = MIN(next, tx_cyc_time(s, n, cyc));
        }
    }
    for (unsigned m = 0; m < NRX; m++) {
        next = MIN(next, rx_next_event(s, m));
    }
    if (next == NEVER) {
        timer_del(&s->timer);
    } else {
        timer_mod_ns(&s->timer, next);
    }
    update_irq(s);
}

static bool rmt_receiving(ESP32P4RmtState *s)
{
    for (unsigned m = 0; m < NRX; m++) {
        if (rx_enabled(&s->rx[m])) {
            return true;
        }
    }
    return false;
}

/*
 * Bring the other line sources up to the present, so that no input edge
 * older than now is still to come, then this block.
 */
static void rmt_catch_up(ESP32P4RmtState *s)
{
    if (rmt_receiving(s)) {
        esp32_line_sync();
    }
    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void rmt_timer_cb(void *opaque)
{
    ESP32P4RmtState *s = opaque;

    rmt_catch_up(s);
    rmt_schedule(s);
}

static void rmt_line_sync(Notifier *n, void *data)
{
    ESP32P4RmtState *s = container_of(n, ESP32P4RmtState, line_source);

    if (s->syncing) {
        return;
    }
    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    rmt_schedule(s);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void esp32p4_rmt_sig_in(void *opaque, int m, int level)
{
    ESP32P4RmtState *s = opaque;
    ESP32P4RmtRx *c = &s->rx[m];
    bool outer = !s->syncing;
    int64_t t;

    level = level != 0;
    if (level == c->in_raw) {
        return;
    }
    if (!rx_enabled(c)) {
        c->in_raw = c->in_filt = c->in_samp = c->demod = level;
        return;
    }
    t = esp32_line_time();
    if (outer) {
        s->syncing = true;
        tx_advance(s, t);
    }
    if (level != c->in_raw) {
        /*
         * A tick at t samples the new level, so the receiver's own events
         * at t come after it, but for a filter maturing at t: a pulse of
         * exactly RX_FILTER_THRES cycles passes.
         */
        rx_advance(s, m, t - 1);
        if (c->filt_pending && c->filt_ns == t) {
            c->filt_pending = false;
            rx_filtered(s, m, t, !c->in_filt);
        }
        rx_raw(s, m, t, level);
    }
    if (outer) {
        s->syncing = false;
        rmt_schedule(s);
    }
}

static void tx_dma_ready(void *opaque)
{
    ESP32P4RmtState *s = opaque;

    rmt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    tx_dma_pull(s);
    rmt_schedule(s);
}

static void rx_dma_ready(void *opaque)
{
    ESP32P4RmtState *s = opaque;

    rx_dma_push(s);
    update_irq(s);
}

static const ESP32P4GdmaPeriphOps esp32p4_rmt_dma_ops = {
    .tx_ready = tx_dma_ready,
    .rx_ready = rx_dma_ready,
};

/* Registers */

static uint32_t tx_status(ESP32P4RmtState *s, unsigned n)
{
    ESP32P4RmtTx *c = &s->tx[n];
    uint32_t v;

    v = c->raddr & ST_ADDR_MASK;
    v |= (c->apb_waddr & ST_ADDR_MASK) << ST_APB_ADDR_SHIFT;
    v |= (c->state != TX_IDLE ? STATE_SEND : STATE_IDLE) << ST_STATE_SHIFT;
    v |= c->mem_empty ? ST_TX_MEM_EMPTY : 0;
    v |= c->apb_wr_err ? ST_TX_APB_MEM_WR_ERR : 0;
    return v;
}

static uint32_t rx_status(ESP32P4RmtState *s, unsigned m)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint32_t v;

    v = c->waddr & ST_ADDR_MASK;
    v |= (c->apb_raddr & ST_ADDR_MASK) << ST_APB_ADDR_SHIFT;
    v |= (c->frame ? STATE_RECEIVE : STATE_IDLE) << ST_STATE_SHIFT;
    v |= c->owner_err ? ST_RX_MEM_OWNER_ERR : 0;
    v |= c->mem_full ? ST_RX_MEM_FULL : 0;
    v |= c->apb_rd_err ? ST_RX_APB_MEM_RD_ERR : 0;
    return v;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void fifo_write(ESP32P4RmtState *s, unsigned n, uint32_t v)
{
    if (s->sys_conf & SYS_APB_FIFO_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CH%uDATA written with "
                      "APB_FIFO_MASK set\n", TYPE_ESP32P4_RMT, n);
        return;
    }
    if (n == DMA_TX && (s->tx[n].conf0 & TX_DMA_ACCESS_EN)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CH3DATA written in DMA mode\n",
                      TYPE_ESP32P4_RMT);
        return;
    }
    tx_apb_write(s, n, v);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static uint32_t fifo_read(ESP32P4RmtState *s, unsigned m)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint32_t end = rx_end(c, m);
    uint32_t v;

    if (s->sys_conf & SYS_APB_FIFO_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CH%uDATA read with "
                      "APB_FIFO_MASK set\n", TYPE_ESP32P4_RMT, NTX + m);
        return 0;
    }
    if (rx_dma(s, m)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CH7DATA read in DMA mode\n",
                      TYPE_ESP32P4_RMT);
        return 0;
    }
    if (c->apb_raddr >= end) {
        c->apb_rd_err = true;
        return 0;
    }
    v = ram_pd(s) ? 0 : s->ram[c->apb_raddr];
    c->apb_raddr = ptr_next(c->apb_raddr, rx_base(m), end,
                            c->conf1 & RX_MEM_WRAP_EN);
    return v;
}

static uint64_t esp32p4_rmt_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4RmtState *s = opaque;
    uint64_t v;

    rmt_catch_up(s);
    switch (addr) {
    case A_TX_DATA(0) ... A_TX_DATA(NTX - 1):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of TX channel %u's FIFO\n",
                      TYPE_ESP32P4_RMT, (unsigned)(addr / 4));
        v = 0;
        break;
    case A_RX_DATA(0) ... A_RX_DATA(NRX - 1):
        v = fifo_read(s, (addr - A_RX_DATA(0)) / 4);
        break;
    case A_TX_CONF0(0) ... A_TX_CONF0(NTX - 1):
        v = s->tx[(addr - A_TX_CONF0(0)) / 4].conf0;
        break;
    case A_RX_CONF0(0) ... A_RX_CONF1(NRX - 1):
        if (addr & 4) {
            v = s->rx[(addr - A_RX_CONF0(0)) / 8].conf1;
        } else {
            v = s->rx[(addr - A_RX_CONF0(0)) / 8].conf0;
        }
        break;
    case A_TX_STATUS(0) ... A_TX_STATUS(NTX - 1):
        v = tx_status(s, (addr - A_TX_STATUS(0)) / 4);
        break;
    case A_RX_STATUS(0) ... A_RX_STATUS(NRX - 1):
        v = rx_status(s, (addr - A_RX_STATUS(0)) / 4);
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_TX_CARRIER_DUTY(0) ... A_TX_CARRIER_DUTY(NTX - 1):
        v = s->tx[(addr - A_TX_CARRIER_DUTY(0)) / 4].carrier_duty;
        break;
    case A_RX_CARRIER_RM(0) ... A_RX_CARRIER_RM(NRX - 1):
        v = s->rx[(addr - A_RX_CARRIER_RM(0)) / 4].carrier_rm;
        break;
    case A_TX_LIM(0) ... A_TX_LIM(NTX - 1):
        v = s->tx[(addr - A_TX_LIM(0)) / 4].tx_lim;
        break;
    case A_RX_LIM(0) ... A_RX_LIM(NRX - 1):
        v = s->rx[(addr - A_RX_LIM(0)) / 4].rx_lim;
        break;
    case A_SYS_CONF:
        v = s->sys_conf;
        break;
    case A_TX_SIM:
        v = s->tx_sim;
        break;
    case A_INT_CLR:
    case A_REF_CNT_RST:
        v = 0;
        break;
    case A_DATE:
        v = s->date;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RMT, addr);
        v = 0;
        break;
    }
    rmt_schedule(s);
    return v;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void tx_conf0_write(ESP32P4RmtState *s, unsigned n, uint32_t v,
                           int64_t now)
{
    ESP32P4RmtTx *c = &s->tx[n];
    uint32_t old = c->conf0;

    c->conf0 = v & (TX_CONF0_RW | (n == DMA_TX ? TX_DMA_ACCESS_EN : 0));
    if (v & TX_MEM_RD_RST) {
        c->raddr = mem_base(n);
        c->mem_empty = false;
        c->thr_count = 0;
    }
    if (v & TX_APB_MEM_RST) {
        c->apb_waddr = mem_base(n);
        c->apb_wr_err = false;
        c->dma_fill = 0;
        c->dma_word = 0;
        c->dma_bytes = 0;
    }
    if (v & TX_CONF_UPDATE) {
        tx_conf_update(s, n, now);
    }
    if (v & TX_START) {
        tx_start_req(s, n, now);
    }
    if (n == DMA_TX && (c->conf0 & ~old & TX_DMA_ACCESS_EN)) {
        tx_dma_pull(s);
    }
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void rx_conf1_write(ESP32P4RmtState *s, unsigned m, uint32_t v,
                           int64_t now)
{
    ESP32P4RmtRx *c = &s->rx[m];
    uint32_t old = c->conf1;

    c->conf1 = v & RX_CONF1_RW;
    if (v & RX_MEM_WR_RST) {
        c->waddr = rx_base(m);
        c->half = 0;
        c->mem_full = false;
        c->owner_err = false;
        c->thr_count = 0;
    }
    if (v & RX_APB_MEM_RST) {
        c->apb_raddr = rx_base(m);
        c->apb_rd_err = false;
        c->dma_unread = 0;
        for (uint32_t a = rx_base(m); a < rx_end(c, m); a++) {
            s->eof_bytes[a] = 0;
        }
    }
    if ((old & RX_FILTER_EN) && !(c->conf1 & RX_FILTER_EN) &&
        c->filt_pending) {
        c->filt_pending = false;
        rx_filtered(s, m, now, c->in_raw);
    }
    if (v & RX_CONF_UPDATE) {
        rx_conf_update(s, m, now);
    }
}

static void sys_conf_write(ESP32P4RmtState *s, uint32_t v)
{
    v &= SYS_CONF_RW;
    if (v & ~s->sys_conf & SYS_MEM_FORCE_PD) {
        memset(s->ram, 0, sizeof(s->ram));
        memset(s->eof_bytes, 0, sizeof(s->eof_bytes));
    }
    s->sys_conf = v;
}

static void esp32p4_rmt_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4RmtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = value;
    unsigned n;

    rmt_catch_up(s);
    s->syncing = true;
    switch (addr) {
    case A_TX_DATA(0) ... A_TX_DATA(NTX - 1):
        fifo_write(s, addr / 4, v);
        break;
    case A_RX_DATA(0) ... A_RX_DATA(NRX - 1):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to RX channel %u's "
                      "FIFO\n", TYPE_ESP32P4_RMT,
                      (unsigned)((addr - A_RX_DATA(0)) / 4));
        break;
    case A_TX_CONF0(0) ... A_TX_CONF0(NTX - 1):
        tx_conf0_write(s, (addr - A_TX_CONF0(0)) / 4, v, now);
        break;
    case A_RX_CONF0(0) ... A_RX_CONF1(NRX - 1):
        n = (addr - A_RX_CONF0(0)) / 8;
        if (addr & 4) {
            rx_conf1_write(s, n, v, now);
        } else {
            s->rx[n].conf0 = v & (RX_CONF0_RW |
                                  (n == DMA_RX ? RX_DMA_ACCESS_EN : 0));
            rx_dma_push(s);
        }
        break;
    case A_TX_STATUS(0) ... A_RX_STATUS(NRX - 1):
    case A_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RMT, addr);
        break;
    case A_INT_RAW:
        s->int_raw &= ~v;
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_MASK;
        break;
    case A_INT_CLR:
        s->int_raw &= ~v;
        break;
    case A_TX_CARRIER_DUTY(0) ... A_TX_CARRIER_DUTY(NTX - 1):
        s->tx[(addr - A_TX_CARRIER_DUTY(0)) / 4].carrier_duty = v;
        break;
    case A_RX_CARRIER_RM(0) ... A_RX_CARRIER_RM(NRX - 1):
        s->rx[(addr - A_RX_CARRIER_RM(0)) / 4].carrier_rm = v;
        break;
    case A_TX_LIM(0) ... A_TX_LIM(NTX - 1):
        n = (addr - A_TX_LIM(0)) / 4;
        s->tx[n].tx_lim = v & TXL_RW;
        if (v & TXL_LOOP_COUNT_RESET) {
            s->tx[n].loop_count = 0;
        }
        break;
    case A_RX_LIM(0) ... A_RX_LIM(NRX - 1):
        s->rx[(addr - A_RX_LIM(0)) / 4].rx_lim = v & RXL_MASK;
        break;
    case A_SYS_CONF:
        sys_conf_write(s, v);
        break;
    case A_TX_SIM:
        s->tx_sim = v & TX_SIM_RW;
        s->tx_armed &= s->tx_sim;
        break;
    case A_REF_CNT_RST:
        for (n = 0; n < NTX; n++) {
            if (v & BIT(n)) {
                tx_div_reset(s, n, now);
            }
        }
        for (n = 0; n < NRX; n++) {
            if (v & BIT(NTX + n)) {
                s->rx[n].div_rst_pending = true;
            }
        }
        break;
    case A_DATE:
        s->date = v & DATE_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RMT, addr);
        break;
    }
    s->syncing = false;
    /* Edges the write caused at once are processed with everything due. */
    rmt_sync(s, now);
    rmt_schedule(s);
}

static const MemoryRegionOps esp32p4_rmt_ops = {
    .read = esp32p4_rmt_read,
    .write = esp32p4_rmt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Direct access to the RAM, at 0x800 */
static bool ram_accessible(ESP32P4RmtState *s, hwaddr addr, bool write)
{
    ESP32P4RmtTx *c3 = &s->tx[DMA_TX];
    uint32_t word = addr / 4;

    if (word >= RAM_WORDS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RMT,
                      write ? "write" : "read",
                      addr + ESP32P4_RMT_RAM_OFFSET);
        return false;
    }
    if (!(s->sys_conf & SYS_APB_FIFO_MASK)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: RAM %s at 0x%" HWADDR_PRIx
                      " with APB_FIFO_MASK clear\n", TYPE_ESP32P4_RMT,
                      write ? "write" : "read", addr);
        return false;
    }
    if (ram_pd(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: RAM %s at 0x%" HWADDR_PRIx
                      " while it is powered down (MEM_FORCE_PD)\n",
                      TYPE_ESP32P4_RMT, write ? "write" : "read", addr);
        return false;
    }
    if ((c3->conf0 & TX_DMA_ACCESS_EN) && word >= mem_base(DMA_TX) &&
        word < tx_end(c3, DMA_TX)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: RAM %s at 0x%" HWADDR_PRIx
                      " in channel 3's memory in DMA mode\n",
                      TYPE_ESP32P4_RMT, write ? "write" : "read", addr);
        return false;
    }
    return true;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static uint64_t esp32p4_rmt_ram_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    ESP32P4RmtState *s = opaque;
    uint64_t v = 0;

    rmt_catch_up(s);
    if (ram_accessible(s, addr, false)) {
        v = extract32(s->ram[addr / 4], (addr & 3) * 8, size * 8);
    }
    rmt_schedule(s);
    return v;
}

static void esp32p4_rmt_ram_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    ESP32P4RmtState *s = opaque;

    rmt_catch_up(s);
    if (ram_accessible(s, addr, true)) {
        s->ram[addr / 4] = deposit32(s->ram[addr / 4], (addr & 3) * 8,
                                     size * 8, value);
    }
    rmt_schedule(s);
}

static const MemoryRegionOps esp32p4_rmt_ram_ops = {
    .read = esp32p4_rmt_ram_read,
    .write = esp32p4_rmt_ram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * rmt_sclk is about to change: bring the channels up to now with the old
 * period, and restart their cycle counts there, at its new exact period.
 */
static void esp32p4_rmt_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4RmtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        rmt_sync(s, now);
        for (unsigned n = 0; n < NTX; n++) {
            s->tx[n].anchor_c = tx_cyc_at(s, n, now);
            s->tx[n].anchor_ns = now;
        }
        for (unsigned m = 0; m < NRX; m++) {
            s->rx[m].anchor_c = rx_cyc_at(s, m, now);
            s->rx[m].anchor_ns = now;
        }
    } else {
        s->sclk_edges = esp32p4_clk_edges_or_at(s->sclk, now);
        rmt_schedule(s);
    }
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void esp32p4_rmt_reset_hold(Object *obj, ResetType type)
{
    ESP32P4RmtState *s = ESP32P4_RMT(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timer_del(&s->timer);
    s->sclk_edges = esp32p4_clk_edges_or_at(s->sclk, now);
    s->int_raw = 0;
    s->int_ena = 0;
    s->sys_conf = SYS_CONF_RESET;
    s->tx_sim = 0;
    s->act_tx_sim_en = false;
    s->tx_armed = 0;
    s->date = DATE_RESET;
    s->syncing = false;
    memset(s->eof_bytes, 0, sizeof(s->eof_bytes));
    for (unsigned n = 0; n < NTX; n++) {
        ESP32P4RmtTx *c = &s->tx[n];

        memset(c, 0, sizeof(*c));
        c->conf0 = TX_CONF0_RESET;
        c->act_conf0 = TX_CONF0_RESET & TX_CONF0_LATCHED;
        c->carrier_duty = c->act_carrier_duty = CARRIER_DUTY_RESET;
        c->tx_lim = TXL_RESET;
        c->act_tx_lim = TXL_RESET & TXL_LATCHED;
        c->anchor_ns = now;
        c->state = TX_IDLE;
        c->raddr = c->apb_waddr = TX_ADDR_RESET;
    }
    for (unsigned m = 0; m < NRX; m++) {
        ESP32P4RmtRx *c = &s->rx[m];
        uint8_t in = c->in_raw;

        memset(c, 0, sizeof(*c));
        c->conf0 = RX_CONF0_RESET;
        c->act_conf0 = RX_CONF0_RESET & RX_CONF0_LATCHED;
        c->conf1 = RX_CONF1_RESET;
        c->act_conf1 = RX_CONF1_RESET & RX_CONF1_LATCHED;
        c->rx_lim = c->act_rx_lim = RXL_RESET;
        c->anchor_ns = now;
        c->waddr = c->apb_raddr = RX_ADDR_RESET;
        c->in_raw = c->in_filt = c->in_samp = c->demod = in;
    }
}

static void esp32p4_rmt_reset_exit(Object *obj, ResetType type)
{
    ESP32P4RmtState *s = ESP32P4_RMT(obj);

    for (unsigned n = 0; n < NTX; n++) {
        qemu_set_irq(s->sig_out[n], 0);
    }
    update_irq(s);
}

static void esp32p4_rmt_init(Object *obj)
{
    ESP32P4RmtState *s = ESP32P4_RMT(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init(&s->container, obj, TYPE_ESP32P4_RMT,
                       ESP32P4_RMT_SIZE);
    memory_region_init_io(&s->iomem, obj, &esp32p4_rmt_ops, s,
                          TYPE_ESP32P4_RMT ".regs", ESP32P4_RMT_REG_SIZE);
    memory_region_add_subregion(&s->container, 0, &s->iomem);
    memory_region_init_io(&s->ram_iomem, obj, &esp32p4_rmt_ram_ops, s,
                          TYPE_ESP32P4_RMT ".ram",
                          ESP32P4_RMT_SIZE - ESP32P4_RMT_RAM_OFFSET);
    memory_region_add_subregion(&s->container, ESP32P4_RMT_RAM_OFFSET,
                                &s->ram_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->container);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->sig_out, ESP32P4_RMT_SIG_OUT, NTX);
    qdev_init_gpio_in_named(dev, esp32p4_rmt_sig_in, ESP32P4_RMT_SIG_IN,
                            NRX);
    s->sclk = qdev_init_clock_in(dev, "sclk", esp32p4_rmt_clk_update, s,
                                 ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, rmt_timer_cb, s);
    s->line_source.notify = rmt_line_sync;
}

static void esp32p4_rmt_realize(DeviceState *dev, Error **errp)
{
    ESP32P4RmtState *s = ESP32P4_RMT(dev);

    if (s->gdma) {
        esp32p4_gdma_attach(s->gdma, ESP32P4_GDMA_AHB_RMT,
                            &esp32p4_rmt_dma_ops, s);
    }
    esp32_line_add_source(&s->line_source);
}

static const Property esp32p4_rmt_properties[] = {
    DEFINE_PROP_LINK("gdma", ESP32P4RmtState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
};

static const VMStateDescription vmstate_esp32p4_rmt_tx = {
    .name = "esp32p4-rmt-tx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, ESP32P4RmtTx),
        VMSTATE_UINT32(carrier_duty, ESP32P4RmtTx),
        VMSTATE_UINT32(tx_lim, ESP32P4RmtTx),
        VMSTATE_UINT32(act_conf0, ESP32P4RmtTx),
        VMSTATE_UINT32(act_carrier_duty, ESP32P4RmtTx),
        VMSTATE_UINT32(act_tx_lim, ESP32P4RmtTx),
        VMSTATE_INT64(anchor_ns, ESP32P4RmtTx),
        VMSTATE_UINT64(anchor_c, ESP32P4RmtTx),
        VMSTATE_UINT32(div_phase, ESP32P4RmtTx),
        VMSTATE_UINT8(state, ESP32P4RmtTx),
        VMSTATE_UINT32(raddr, ESP32P4RmtTx),
        VMSTATE_UINT8(half, ESP32P4RmtTx),
        VMSTATE_UINT32(word, ESP32P4RmtTx),
        VMSTATE_UINT64(event_c, ESP32P4RmtTx),
        VMSTATE_UINT64(entry_end_c, ESP32P4RmtTx),
        VMSTATE_UINT8(level, ESP32P4RmtTx),
        VMSTATE_BOOL(pass_sent, ESP32P4RmtTx),
        VMSTATE_BOOL(carrier, ESP32P4RmtTx),
        VMSTATE_BOOL(carrier_high, ESP32P4RmtTx),
        VMSTATE_UINT64(carrier_end_c, ESP32P4RmtTx),
        VMSTATE_UINT32(thr_count, ESP32P4RmtTx),
        VMSTATE_UINT32(loop_count, ESP32P4RmtTx),
        VMSTATE_UINT8(end_level, ESP32P4RmtTx),
        VMSTATE_BOOL(mem_empty, ESP32P4RmtTx),
        VMSTATE_UINT8(out_level, ESP32P4RmtTx),
        VMSTATE_UINT32(apb_waddr, ESP32P4RmtTx),
        VMSTATE_BOOL(apb_wr_err, ESP32P4RmtTx),
        VMSTATE_UINT32(dma_fill, ESP32P4RmtTx),
        VMSTATE_UINT32(dma_word, ESP32P4RmtTx),
        VMSTATE_UINT8(dma_bytes, ESP32P4RmtTx),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_rmt_rx = {
    .name = "esp32p4-rmt-rx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, ESP32P4RmtRx),
        VMSTATE_UINT32(conf1, ESP32P4RmtRx),
        VMSTATE_UINT32(carrier_rm, ESP32P4RmtRx),
        VMSTATE_UINT32(rx_lim, ESP32P4RmtRx),
        VMSTATE_UINT32(act_conf0, ESP32P4RmtRx),
        VMSTATE_UINT32(act_conf1, ESP32P4RmtRx),
        VMSTATE_UINT32(act_carrier_rm, ESP32P4RmtRx),
        VMSTATE_UINT32(act_rx_lim, ESP32P4RmtRx),
        VMSTATE_BOOL(div_rst_pending, ESP32P4RmtRx),
        VMSTATE_INT64(anchor_ns, ESP32P4RmtRx),
        VMSTATE_UINT64(anchor_c, ESP32P4RmtRx),
        VMSTATE_UINT32(div_phase, ESP32P4RmtRx),
        VMSTATE_UINT8(in_raw, ESP32P4RmtRx),
        VMSTATE_UINT8(in_filt, ESP32P4RmtRx),
        VMSTATE_BOOL(filt_pending, ESP32P4RmtRx),
        VMSTATE_INT64(filt_ns, ESP32P4RmtRx),
        VMSTATE_UINT8(in_samp, ESP32P4RmtRx),
        VMSTATE_BOOL(samp_pending, ESP32P4RmtRx),
        VMSTATE_UINT64(samp_c, ESP32P4RmtRx),
        VMSTATE_UINT8(demod, ESP32P4RmtRx),
        VMSTATE_BOOL(demod_pending, ESP32P4RmtRx),
        VMSTATE_UINT64(demod_c, ESP32P4RmtRx),
        VMSTATE_BOOL(frame, ESP32P4RmtRx),
        VMSTATE_UINT8(level, ESP32P4RmtRx),
        VMSTATE_UINT64(edge_c, ESP32P4RmtRx),
        VMSTATE_UINT32(waddr, ESP32P4RmtRx),
        VMSTATE_UINT8(half, ESP32P4RmtRx),
        VMSTATE_BOOL(mem_full, ESP32P4RmtRx),
        VMSTATE_BOOL(owner_err, ESP32P4RmtRx),
        VMSTATE_UINT32(thr_count, ESP32P4RmtRx),
        VMSTATE_UINT32(apb_raddr, ESP32P4RmtRx),
        VMSTATE_BOOL(apb_rd_err, ESP32P4RmtRx),
        VMSTATE_UINT32(dma_unread, ESP32P4RmtRx),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_rmt = {
    .name = TYPE_ESP32P4_RMT,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ram, ESP32P4RmtState, RAM_WORDS),
        VMSTATE_UINT8_ARRAY(eof_bytes, ESP32P4RmtState, RAM_WORDS),
        VMSTATE_UINT32(int_raw, ESP32P4RmtState),
        VMSTATE_UINT32(int_ena, ESP32P4RmtState),
        VMSTATE_UINT32(sys_conf, ESP32P4RmtState),
        VMSTATE_UINT32(tx_sim, ESP32P4RmtState),
        VMSTATE_BOOL(act_tx_sim_en, ESP32P4RmtState),
        VMSTATE_UINT8(tx_armed, ESP32P4RmtState),
        VMSTATE_UINT32(date, ESP32P4RmtState),
        VMSTATE_STRUCT_ARRAY(tx, ESP32P4RmtState, NTX, 1,
                             vmstate_esp32p4_rmt_tx, ESP32P4RmtTx),
        VMSTATE_STRUCT_ARRAY(rx, ESP32P4RmtState, NRX, 1,
                             vmstate_esp32p4_rmt_rx, ESP32P4RmtRx),
        VMSTATE_TIMER(timer, ESP32P4RmtState),
        VMSTATE_CLOCK(sclk, ESP32P4RmtState),
        VMSTATE_ESP32P4_CLK_EDGES(sclk_edges, ESP32P4RmtState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32p4_rmt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_rmt_reset_hold;
    rc->phases.exit = esp32p4_rmt_reset_exit;
    dc->realize = esp32p4_rmt_realize;
    dc->vmsd = &vmstate_esp32p4_rmt;
    device_class_set_props(dc, esp32p4_rmt_properties);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static const TypeInfo esp32p4_rmt_info = {
    .name = TYPE_ESP32P4_RMT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4RmtState),
    .instance_init = esp32p4_rmt_init,
    .class_init = esp32p4_rmt_class_init,
};

static void esp32p4_rmt_register_types(void)
{
    type_register_static(&esp32p4_rmt_info);
}

type_init(esp32p4_rmt_register_types)
