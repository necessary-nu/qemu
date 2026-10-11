/*
 * ESP32-P4 parallel IO controller (PARLIO)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 61 (Parallel IO Controller), with the layout, access types and reset
 * values of ESP-IDF's parl_io_reg.h for v3, its parlio_ll driver and its
 * esp_driver_parlio TX and RX drivers.
 *
 * Two units on a bus of up to 16 lines, each in its own core clock
 * domain: TX takes bytes from GDMA-AXI (PERI_SEL 3) and shifts them out
 * on TXD, RX samples RXD and gives the bytes to GDMA-AXI; both reach the
 * pads through the GPIO matrix's PARLIO signals.
 *
 * Clocks (TRM 61.5.1). A unit's core clock, PARLIO_TX_CLK or _RX_CLK, is
 * HP_SYS_CLKRST's mux of XTAL, RC_FAST and PLL_F160M, divided, or the
 * pad's clock (source 3), which reaches the unit through the matrix as
 * the signal PAD_PARLIO_TX_CLK / _RX_CLK; the model takes that signal's
 * edges, divides them by the integer part of the divider and asks
 * HP_SYS_CLKRST at each edge whether the pad is the source
 * (esp32p4_clkrst_pad_source()). *_CLK_I_INV inverts the clock in: TX
 * shifts and RX samples on the rising edge of the clock after it. The
 * unit puts its clock out as PARLIO_TX_CLK / _RX_CLK, inverted by
 * *_CLK_O_INV. TX_GATING_EN lets TX's clock out toggle only while CS is
 * active (TX_VALID_OUTPUT_EN set) or TXD[15] is high (clear); with
 * RX_GATING_EN RX's toggles only while RX_START is set. At an active edge
 * a unit samples its inputs before its clock out changes, and changes
 * its data after it, as flip-flops do: a receiver clocked by the same
 * edge sees the data from before it. Every edge is stamped with its own
 * virtual time (hw/gpio/esp32_line.h), so transfers between units, or
 * with a device on the pads, are exact under icount and host time alike.
 *
 * TX (TRM 61.5.6 to 61.5.11). With TX_START set the next active edge
 * starts a frame: CS (PARLIO_TX_CS, active low) falls, and
 * TX_CS_START_DELAY edges later the unit shifts out TX_BUS_WID_SEL's
 * width of bits an edge, a 16-bit cycle a byte pair, low byte on
 * TXD[7:0]; at widths 1, 2 and 4 a byte goes out from its low bits up, or
 * from its high bits down with TX_DATA_ORDER_INV (61.5.13). The frame
 * ends when the bit count equals TX_BITLEN (TX_EOF_GEN_SEL clear: the
 * hardware compares for equality, so a length the count never meets
 * never ends a frame, which ESP-IDF's loop transmission relies on) or
 * when the byte that ended GDMA's frame has gone (set). The edge after
 * the last bit puts TX_IDLE_VALUE back on TXD[15:0]; CS rises
 * TX_CS_STOP_DELAY edges later, when TX_EOF_INT is raised and the unit
 * waits for TX_START to be cleared. A data cycle that finds the FIFO
 * empty raises TX_FIFO_REMPTY_INT and is taken at a later edge, TXD
 * holding meanwhile. Lines above the width are 0 while data goes out.
 * TX_READY is set while the FIFO holds data.
 *
 * RX (TRM 61.5.4, 61.5.5, 61.5.7). With RX_START set the unit samples
 * RXD at each active edge. Level enable mode: a sample is data while
 * RXD[RX_EXT_EN_SEL] (inverted by RX_EXT_EN_INV) is high. Pulse enable
 * mode: a rising edge of the enable opens a frame and the next one closes
 * it (sub-modes 0 to 3), each pulse's own sample data or not as the
 * sub-mode says; sub-modes 4 and 5 close it by length. Software enable
 * mode: every sample is data while RX_SW_EN is set. Data is packed into
 * bytes as TX unpacks them. A frame ends, and its last byte carries
 * GDMA's SUC EOF, when the bit count equals RX_BITLEN (RX_EOF_GEN_SEL
 * clear) or when the enable ends it (set: level mode as it falls, pulse
 * mode at the closing pulse); a partial last byte is padded with zeros.
 * The next frame follows without software. The receive timeout counts
 * PARLIO_SYS_CLK cycles from the last data of a frame; at
 * RX_TIMEOUT_THRES it ends the frame with GDMA's ERR EOF. Bytes the FIFO
 * cannot take raise RX_FIFO_WOVF_INT and are lost. The RX configuration
 * registers (RX_MODE_CFG, RX_DATA_CFG, RX_GENRL_CFG) take effect at
 * RX_REG_UPDATE (TRM 61.7.1, step 8), which clears itself at once.
 *
 * The units' asynchronous FIFOs move bytes to and from GDMA-AXI while
 * PARLIO_SYS_CLK runs; *_FIFO_SRST empties a FIFO and holds it empty.
 * RST_EN_PARLIO_TX and _RX reset a unit's core clock domain (its frame,
 * counters and outputs), RST_EN_PARLIO the whole block. HP_PARLIO_TX_INTR
 * carries TX_FIFO_REMPTY and TX_EOF; HP_PARLIO_RX_INTR, RX_FIFO_WOVF.
 * There are no ETM events or tasks.
 *
 * On the host's clock, rather than icount, the units' internal clocks
 * slip behind virtual time when their timers run late, so that the DMA,
 * which times itself from the present, keeps up (see uclk_timer_cb()).
 *
 * Not modelled: the FIFOs' depth, which the TRM does not give, is taken
 * as 16 bytes; a fractional divider on a pad clock divides by its
 * integer part (logged); a unit's clock toggles on its output only while
 * the unit is busy (TX with a frame to send, RX with RX_START set), so a
 * device on the pads clocked by an idle unit sees no edges, and RX in
 * software enable mode clocked by the TX unit's idle clock samples
 * nothing between TX's frames; TX_CNT and RX_CNT, the TRM's "cycle
 * number of reading" each FIFO, count the bytes read from it (by the TX
 * core, by GDMA), wrapping at their widths; PARL_IO_CLK_EN is kept as
 * written.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_parlio.h"
#include "migration/vmstate.h"

#define TX                      ESP32P4_PARLIO_TX
#define RX                      ESP32P4_PARLIO_RX
#define PERIPH                  ESP32P4_GDMA_AXI_PARLIO

#define A_RX_MODE_CFG           0x00
#define A_RX_DATA_CFG           0x04
#define A_RX_GENRL_CFG          0x08
#define A_RX_START_CFG          0x0c
#define A_TX_DATA_CFG           0x10
#define A_TX_START_CFG          0x14
#define A_TX_GENRL_CFG          0x18
#define A_FIFO_CFG              0x1c
#define A_REG_UPDATE            0x20
#define A_ST                    0x24
#define A_INT_ENA               0x28
#define A_INT_RAW               0x2c
#define A_INT_ST                0x30
#define A_INT_CLR               0x34
#define A_RX_ST0                0x38
#define A_RX_ST1                0x3c
#define A_TX_ST0                0x40
#define A_RX_CLK_CFG            0x44
#define A_TX_CLK_CFG            0x48
#define A_TX_CS_CFG             0x4c
#define A_CLK                   0x120
#define A_VERSION               0x3fc

#define RX_EXT_EN_SEL(v)        extract32((v), 21, 4)
#define RX_SW_EN                BIT(25)
#define RX_EXT_EN_INV           BIT(26)
#define RX_PULSE_SUBMODE(v)     extract32((v), 27, 3)
#define RX_SMP_MODE(v)          extract32((v), 30, 2)
#define RX_MODE_MASK            0xffe00000u
#define RX_MODE_RESET           (7u << 21)

#define DATA_BITLEN(v)          extract32((v), 9, 19)
#define DATA_ORDER_INV          BIT(28)
#define DATA_BUS_WID_SEL(v)     extract32((v), 29, 3)
#define DATA_CFG_MASK           0xfffffe00u
#define DATA_CFG_RESET          (3u << 29)

#define RX_GATING_EN            BIT(12)
#define RX_TIMEOUT_THRES(v)     extract32((v), 13, 16)
#define RX_TIMEOUT_EN           BIT(29)
#define RX_EOF_GEN_SEL          BIT(30)
#define RX_GENRL_MASK           0x7ffff000u
#define RX_GENRL_RESET          ((0xfffu << 13) | RX_TIMEOUT_EN)

#define START                   BIT(31)

#define TX_EOF_GEN_SEL          BIT(13)
#define TX_IDLE_VALUE(v)        extract32((v), 14, 16)
#define TX_GATING_EN            BIT(30)
#define TX_VALID_OUTPUT_EN      BIT(31)
#define TX_GENRL_MASK           0xffffe000u

#define TX_FIFO_SRST            BIT(30)
#define RX_FIFO_SRST            BIT(31)
#define FIFO_CFG_MASK           (TX_FIFO_SRST | RX_FIFO_SRST)

#define RX_REG_UPDATE           BIT(31)
#define TX_READY                BIT(31)

#define INT_TX_FIFO_REMPTY      BIT(0)
#define INT_RX_FIFO_WOVF        BIT(1)
#define INT_TX_EOF              BIT(2)
#define INT_MASK                0x7u
#define INT_TX                  (INT_TX_FIFO_REMPTY | INT_TX_EOF)
#define INT_RX                  INT_RX_FIFO_WOVF

#define CLK_I_INV               BIT(30)
#define CLK_O_INV               BIT(31)
#define CLK_CFG_MASK            (CLK_I_INV | CLK_O_INV)

#define TX_CS_STOP_DELAY(v)     extract32((v), 0, 16)
#define TX_CS_START_DELAY(v)    extract32((v), 16, 16)

#define PARLIO_CLK_EN           BIT(31)
#define VERSION_RESET           0x2409230
#define VERSION_MASK            0x0fffffffu

#define BITCNT_MASK             0x7ffffu

/* How late an internal edge may be taken before the clock slips. */
#define PARLIO_SLIP_NS          2000

/* The FIFO entries' marks. */
#define F_EOF                   BIT(0)
#define F_ERR                   BIT(1)
#define F_NODATA                BIT(2)

enum { SMP_LEVEL, SMP_PULSE, SMP_SW };
enum { TXS_IDLE, TXS_START, TXS_DATA, TXS_STOP, TXS_DONE };

static const char *const unit_name[2] = { "TX", "RX" };

/* FIFOs */

static void fifo_reset(ESP32P4ParlioFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static uint32_t fifo_room(ESP32P4ParlioFifo *f)
{
    return ESP32P4_PARLIO_FIFO - f->num;
}

static void fifo_put(ESP32P4ParlioFifo *f, uint8_t byte, uint8_t flags)
{
    uint32_t i = (f->head + f->num) % ESP32P4_PARLIO_FIFO;

    assert(f->num < ESP32P4_PARLIO_FIFO);
    f->data[i] = byte;
    f->flags[i] = flags;
    f->num++;
}

static uint8_t fifo_flags(ESP32P4ParlioFifo *f, uint32_t i)
{
    return f->flags[(f->head + i) % ESP32P4_PARLIO_FIFO];
}

static void fifo_take(ESP32P4ParlioFifo *f, uint8_t *byte, uint8_t *flags)
{
    assert(f->num > 0);
    *byte = f->data[f->head];
    *flags = f->flags[f->head];
    f->head = (f->head + 1) % ESP32P4_PARLIO_FIFO;
    f->num--;
}

/* Configuration */

/* The width TX_/RX_BUS_WID_SEL selects, or 0 for the invalid values. */
static unsigned bus_width(uint32_t data_cfg)
{
    unsigned sel = DATA_BUS_WID_SEL(data_cfg);

    return sel <= 4 ? 1u << sel : 0;
}

static uint32_t clk_cfg(ESP32P4ParlioState *s, int unit)
{
    return unit == TX ? s->tx_clk_cfg : s->rx_clk_cfg;
}

static void parlio_update_irq(ESP32P4ParlioState *s)
{
    uint32_t st = s->int_raw & s->int_ena;

    qemu_set_irq(s->irq[RX], !!(st & INT_RX));
    qemu_set_irq(s->irq[TX], !!(st & INT_TX));
}

static void parlio_raise(ESP32P4ParlioState *s, uint32_t bits)
{
    s->int_raw |= bits;
    parlio_update_irq(s);
}

static bool sys_running(ESP32P4ParlioState *s)
{
    return clock_is_enabled(s->sys_clk);
}

/* Whether a unit has edges to take from its internal clock. */
static bool unit_busy(ESP32P4ParlioState *s, int unit)
{
    if (unit == TX) {
        return !s->tx_reset_held && (s->tx_start_cfg & START) &&
               s->tx_state != TXS_DONE && bus_width(s->tx_data_cfg);
    }
    return !s->rx_reset_held && (s->rx_start_cfg & START) &&
           bus_width(s->rx_data_act);
}

/* Whether TX's clock out may toggle, with CS and TXD as given. */
static bool tx_gate(ESP32P4ParlioState *s, bool cs, uint16_t out)
{
    if (!(s->tx_genrl_cfg & TX_GATING_EN)) {
        return true;
    }
    return s->tx_genrl_cfg & TX_VALID_OUTPUT_EN ? cs : out >> 15;
}

/* Whether a unit's clock out may toggle now. */
static bool unit_gate(ESP32P4ParlioState *s, int unit)
{
    if (unit == TX) {
        return tx_gate(s, s->tx_cs_active, s->tx_out);
    }
    return !(s->rx_genrl_act & RX_GATING_EN) || (s->rx_start_cfg & START);
}

/* Outputs */

static void tx_drive(ESP32P4ParlioState *s, uint16_t out, int64_t when)
{
    uint16_t changed = out ^ s->tx_out;

    s->tx_out = out;
    for (int i = 0; i < ESP32P4_PARLIO_LINES; i++) {
        if (changed & BIT(i)) {
            esp32_line_set_at(s->txd[i], (out >> i) & 1, when);
        }
    }
}

static void tx_drive_cs(ESP32P4ParlioState *s, bool active, int64_t when)
{
    if (active != s->tx_cs_active) {
        s->tx_cs_active = active;
        esp32_line_set_at(s->tx_cs, !active, when);
    }
}

static void unit_drive_clk(ESP32P4ParlioState *s, int unit, bool gate,
                           int64_t when)
{
    ESP32P4ParlioClk *u = &s->uclk[unit];
    bool clk = u->in_level ^ !!(clk_cfg(s, unit) & CLK_I_INV);
    bool out = (clk && gate) ^ !!(clk_cfg(s, unit) & CLK_O_INV);

    if (out != u->out_level) {
        u->out_level = out;
        esp32_line_set_at(unit == TX ? s->tx_clk_out : s->rx_clk_out, out,
                          when);
    }
}

static void unit_redrive_clk(ESP32P4ParlioState *s, int unit, int64_t when)
{
    unit_drive_clk(s, unit, unit_gate(s, unit), when);
}

/* The TX unit */

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void tx_fill(ESP32P4ParlioState *s)
{
    uint8_t buf[ESP32P4_PARLIO_FIFO];
    size_t n;
    bool eof;

    if (!s->gdma || !sys_running(s) || (s->fifo_cfg & TX_FIFO_SRST)) {
        return;
    }
    while (fifo_room(&s->tx_fifo)) {
        n = esp32p4_gdma_pull(s->gdma, PERIPH, buf, fifo_room(&s->tx_fifo),
                              &eof);
        for (size_t i = 0; i < n; i++) {
            fifo_put(&s->tx_fifo, buf[i], eof && i == n - 1 ? F_EOF : 0);
        }
        if (eof && !n) {
            fifo_put(&s->tx_fifo, 0, F_EOF | F_NODATA);
        }
        if (!n && !eof) {
            break;
        }
    }
}

static void tx_ready_cb(void *opaque)
{
    tx_fill(opaque);
}

/*
 * The bytes the FIFO holds for the frame, and in *ends whether GDMA's
 * end of the frame is among them (TX_EOF_GEN_SEL set).
 */
static uint32_t tx_frame_bytes(ESP32P4ParlioState *s, bool *ends)
{
    bool dma_eof = s->tx_genrl_cfg & TX_EOF_GEN_SEL;
    uint32_t n = 0;

    *ends = false;
    for (uint32_t i = 0; i < s->tx_fifo.num; i++) {
        uint8_t mark = fifo_flags(&s->tx_fifo, i);

        n += !(mark & F_NODATA);
        if (dma_eof && (mark & F_EOF)) {
            *ends = true;
            break;
        }
    }
    return n;
}

/* Take the next byte, past marks that carry nothing for the frame. */
static void tx_load(ESP32P4ParlioState *s)
{
    uint8_t byte, mark;

    while (fifo_flags(&s->tx_fifo, 0) & F_NODATA) {
        fifo_take(&s->tx_fifo, &byte, &mark);
    }
    fifo_take(&s->tx_fifo, &s->tx_byte, &s->tx_byte_flags);
    s->tx_byte_bits = 8;
    s->tx_cycles++;
}

/*
 * Whether GDMA's frame ended with the byte just taken, or with the mark
 * after it, which goes with it.
 */
static bool tx_frame_ended(ESP32P4ParlioState *s)
{
    uint8_t byte, mark;

    if (s->tx_byte_flags & F_EOF) {
        return true;
    }
    if (s->tx_fifo.num &&
        (fifo_flags(&s->tx_fifo, 0) & (F_NODATA | F_EOF)) ==
        (F_NODATA | F_EOF)) {
        fifo_take(&s->tx_fifo, &byte, &mark);
        return true;
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * One data cycle: the next width of bits in *out, or false if the FIFO
 * has not got them; *ended, GDMA's frame has ended (TX_EOF_GEN_SEL set)
 * with this cycle's bits or, when there are none, before them.
 */
static bool tx_shift(ESP32P4ParlioState *s, unsigned width, uint16_t *out,
                     bool *ended)
{
    bool dma_eof = s->tx_genrl_cfg & TX_EOF_GEN_SEL;
    bool inv = s->tx_data_cfg & DATA_ORDER_INV;
    uint32_t need = width == 16 ? 2 : 1, avail;
    uint8_t byte, mark;
    bool ends;

    *ended = false;
    *out = 0;
    if (!s->tx_byte_bits) {
        avail = tx_frame_bytes(s, &ends);
        if (!avail && ends) {
            /* The frame's end came as a mark after its last byte. */
            fifo_take(&s->tx_fifo, &byte, &mark);
            *ended = true;
            return false;
        }
        if (avail < need && !ends) {
            return false;
        }
        need = MIN(need, avail);
        for (uint32_t i = 0; i < need; i++) {
            tx_load(s);
            if (width >= 8) {
                *out |= s->tx_byte << (8 * i);
                s->tx_byte_bits = 0;
            }
        }
        if (width >= 8) {
            *ended = dma_eof && tx_frame_ended(s);
            return true;
        }
    }
    *out = inv ? extract32(s->tx_byte, s->tx_byte_bits - width, width)
               : extract32(s->tx_byte, 8 - s->tx_byte_bits, width);
    s->tx_byte_bits -= width;
    *ended = dma_eof && !s->tx_byte_bits && tx_frame_ended(s);
    return true;
}

static void tx_end_frame(ESP32P4ParlioState *s)
{
    s->tx_state = TXS_DONE;
    parlio_raise(s, INT_TX_EOF);
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * An active edge of the TX core clock: the frame's next step, with the
 * new data and CS in *out and *cs.
 */
static void tx_edge(ESP32P4ParlioState *s, uint16_t *out, bool *cs)
{
    unsigned width = bus_width(s->tx_data_cfg);
    uint16_t idle = TX_IDLE_VALUE(s->tx_genrl_cfg);
    uint16_t bits;
    bool ended;

    *out = s->tx_out;
    *cs = s->tx_cs_active;
    if (!unit_busy(s, TX)) {
        return;
    }
    if (s->tx_state == TXS_IDLE) {
        s->tx_state = TXS_START;
        s->tx_delay = TX_CS_START_DELAY(s->tx_cs_cfg);
        s->tx_all_sent = false;
        s->tx_bitcnt = 0;
        s->tx_byte_bits = 0;
        *cs = true;
    }
    if (s->tx_state == TXS_START) {
        if (s->tx_delay) {
            s->tx_delay--;
            return;
        }
        s->tx_state = TXS_DATA;
    }
    if (s->tx_state == TXS_DATA && !s->tx_all_sent) {
        if (!tx_shift(s, width, &bits, &ended)) {
            if (!ended) {
                parlio_raise(s, INT_TX_FIFO_REMPTY);
                tx_fill(s);
                return;
            }
            s->tx_all_sent = true;
        } else {
            *out = bits;
            s->tx_bitcnt = (s->tx_bitcnt + width) & BITCNT_MASK;
            s->tx_all_sent = s->tx_genrl_cfg & TX_EOF_GEN_SEL
                             ? ended
                             : s->tx_bitcnt == DATA_BITLEN(s->tx_data_cfg);
            tx_fill(s);
            return;
        }
    }
    if (s->tx_state == TXS_DATA) {
        *out = idle;
        s->tx_state = TXS_STOP;
        s->tx_delay = TX_CS_STOP_DELAY(s->tx_cs_cfg);
    }
    if (s->tx_state == TXS_STOP) {
        if (s->tx_delay) {
            s->tx_delay--;
            return;
        }
        *cs = false;
        tx_end_frame(s);
    }
}

/* Abandon any frame: TXD idle, CS inactive. */
static void tx_stop(ESP32P4ParlioState *s, int64_t when)
{
    s->tx_state = TXS_IDLE;
    s->tx_byte_bits = 0;
    tx_drive_cs(s, false, when);
    tx_drive(s, TX_IDLE_VALUE(s->tx_genrl_cfg), when);
}

/* The RX unit */

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void rx_drain(ESP32P4ParlioState *s)
{
    ESP32P4ParlioFifo *f = &s->rx_fifo;
    uint8_t buf[ESP32P4_PARLIO_FIFO], byte, mark;
    uint32_t n, entries;
    unsigned flags;
    size_t taken;

    if (!s->gdma || !sys_running(s)) {
        return;
    }
    while (f->num) {
        n = 0;
        flags = 0;
        for (entries = 0; entries < f->num;) {
            mark = fifo_flags(f, entries);
            if (!(mark & F_NODATA)) {
                buf[n++] = f->data[(f->head + entries) % ESP32P4_PARLIO_FIFO];
            }
            entries++;
            if (mark & F_EOF) {
                flags = mark & F_ERR ? ESP32P4_GDMA_ERR_EOF
                                     : ESP32P4_GDMA_EOF;
                break;
            }
        }
        if (!esp32p4_gdma_rx_room(s->gdma, PERIPH)) {
            break;
        }
        taken = esp32p4_gdma_push(s->gdma, PERIPH, buf, n, flags);
        s->rx_rd_bitcnt = (s->rx_rd_bitcnt + 8 * taken) & BITCNT_MASK;
        s->rx_cycles += taken;
        if (taken < n) {
            for (size_t k = 0; k < taken; k++) {
                fifo_take(f, &byte, &mark);
            }
            break;
        }
        for (uint32_t k = 0; k < entries; k++) {
            fifo_take(f, &byte, &mark);
        }
        if (flags) {
            s->rx_rd_bitcnt = 0;
        }
    }
}

static void rx_ready_cb(void *opaque)
{
    rx_drain(opaque);
}

static void rx_put(ESP32P4ParlioState *s, uint8_t byte, uint8_t flags)
{
    if (s->fifo_cfg & RX_FIFO_SRST) {
        return;
    }
    if (!fifo_room(&s->rx_fifo)) {
        parlio_raise(s, INT_RX_FIFO_WOVF);
        return;
    }
    fifo_put(&s->rx_fifo, byte, flags);
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * The frame's end: its last byte carries SUC EOF (or ERR EOF), a partial
 * one padded; or a mark carries it if the last byte has gone on.
 */
static void rx_end_frame(ESP32P4ParlioState *s, bool err)
{
    ESP32P4ParlioFifo *f = &s->rx_fifo;
    uint8_t mark = F_EOF | (err ? F_ERR : 0);

    if (s->rx_byte_bits) {
        rx_put(s, s->rx_byte, mark);
    } else if (f->num && !(s->fifo_cfg & RX_FIFO_SRST) &&
               !(fifo_flags(f, f->num - 1) & F_EOF)) {
        f->flags[(f->head + f->num - 1) % ESP32P4_PARLIO_FIFO] |= mark;
    } else {
        rx_put(s, 0, mark | F_NODATA);
    }
    s->rx_byte = 0;
    s->rx_byte_bits = 0;
    s->rx_bitcnt = 0;
    s->rx_in_frame = false;
    s->rx_pulse_open = false;
    timer_del(s->rx_timeout_timer);
    rx_drain(s);
}

/*
 * The receive timeout counts from the frame's last data, or, on the
 * host's clock, from when the data's edge was taken late or a unit's
 * clock last slipped, whichever is latest.
 */
static int64_t rx_timeout_at(ESP32P4ParlioState *s)
{
    return MAX(s->rx_timeout_from, s->slip_ns) +
           clock_ticks_to_ns(s->sys_clk, RX_TIMEOUT_THRES(s->rx_genrl_act));
}

static void rx_arm_timeout(ESP32P4ParlioState *s)
{
    if (!(s->rx_genrl_act & RX_TIMEOUT_EN) || !sys_running(s) ||
        !s->rx_in_frame) {
        timer_del(s->rx_timeout_timer);
        return;
    }
    s->rx_timeout_from = MAX(s->rx_last_data_ns,
                             qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    timer_mod(s->rx_timeout_timer, rx_timeout_at(s));
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void rx_timeout_cb(void *opaque)
{
    ESP32P4ParlioState *s = opaque;

    if (!s->rx_in_frame || !(s->rx_genrl_act & RX_TIMEOUT_EN)) {
        return;
    }
    if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < rx_timeout_at(s)) {
        timer_mod(s->rx_timeout_timer, rx_timeout_at(s));
        return;
    }
    rx_end_frame(s, true);
}

static void rx_data(ESP32P4ParlioState *s, uint16_t sample, unsigned width,
                    int64_t when)
{
    bool inv = s->rx_data_act & DATA_ORDER_INV;

    if (width == 16) {
        rx_put(s, sample, 0);
        rx_put(s, sample >> 8, 0);
    } else if (width == 8) {
        rx_put(s, sample, 0);
    } else {
        uint8_t chunk = sample & MAKE_64BIT_MASK(0, width);

        s->rx_byte |= inv ? chunk << (8 - width - s->rx_byte_bits)
                          : chunk << s->rx_byte_bits;
        s->rx_byte_bits += width;
        if (s->rx_byte_bits == 8) {
            rx_put(s, s->rx_byte, 0);
            s->rx_byte = 0;
            s->rx_byte_bits = 0;
        }
    }
    s->rx_bitcnt = (s->rx_bitcnt + width) & BITCNT_MASK;
    s->rx_in_frame = true;
    s->rx_last_data_ns = when;
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * An active edge of the RX core clock: RXD sampled.
 */
static void rx_edge(ESP32P4ParlioState *s, int64_t when)
{
    uint32_t mode = s->rx_mode_act;
    unsigned width = bus_width(s->rx_data_act);
    bool eof_by_en = s->rx_genrl_act & RX_EOF_GEN_SEL;
    uint16_t sample = s->rxd;
    bool en, rise, valid = false, end_after = false, by_len = true;
    unsigned sub;

    if (!unit_busy(s, RX)) {
        return;
    }
    en = ((sample >> RX_EXT_EN_SEL(mode)) & 1) ^ !!(mode & RX_EXT_EN_INV);
    rise = en && !s->rx_prev_en;
    s->rx_prev_en = en;

    switch (RX_SMP_MODE(mode)) {
    case SMP_LEVEL:
        valid = en;
        if (eof_by_en) {
            by_len = false;
            if (!en && s->rx_in_frame) {
                rx_end_frame(s, false);
            }
        }
        break;
    case SMP_PULSE:
        sub = RX_PULSE_SUBMODE(mode);
        if (sub > 5) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: RX pulse sub-mode %u is "
                          "invalid\n", TYPE_ESP32P4_PARLIO, sub);
            return;
        }
        by_len = sub >= 4 || !eof_by_en;
        if (!s->rx_pulse_open) {
            if (rise) {
                s->rx_pulse_open = true;
                valid = sub == 0 || sub == 1 || sub == 4;
            }
        } else if (sub < 4 && rise) {
            valid = sub == 0 || sub == 2;
            s->rx_pulse_open = false;
            end_after = eof_by_en;
        } else {
            valid = true;
        }
        break;
    case SMP_SW:
        valid = mode & RX_SW_EN;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: RX sampling mode 3 is "
                      "invalid\n", TYPE_ESP32P4_PARLIO);
        return;
    }

    if (valid) {
        rx_data(s, sample, width, when);
        if (by_len && s->rx_bitcnt == DATA_BITLEN(s->rx_data_act)) {
            end_after = true;
        }
    }
    if (end_after) {
        rx_end_frame(s, false);
    } else if (valid) {
        rx_drain(s);
        rx_arm_timeout(s);
    }
}

/* The units' core clocks */

/* [spec:nuos:req:emu.esp32p4.parlio] */
static bool uclk_pad(ESP32P4ParlioClk *u, ESP32P4ClkDiv *div)
{
    return u->s->clkrst &&
           esp32p4_clkrst_pad_source(u->s->clkrst,
                                     u->unit == TX ? ESP32P4_CLK_PARLIO_TX
                                                   : ESP32P4_CLK_PARLIO_RX,
                                     div);
}

/* The time of internal edge @n, a rising edge for even @n. */
static int64_t uclk_edge_ns(ESP32P4ParlioClk *u, uint64_t n)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, u->period, n + 1);
    return u->base_ns + (int64_t)((hi << 31) | (lo >> 33));
}

/* The number of the first internal edge after @t. */
static uint64_t uclk_edge_after(ESP32P4ParlioClk *u, int64_t t)
{
    uint64_t d, lo, hi, n;

    if (t < u->base_ns) {
        return 0;
    }
    d = t - u->base_ns;
    lo = d << 33;
    hi = d >> 31;
    divu128(&lo, &hi, u->period);
    n = lo ? lo - 1 : 0;
    while (uclk_edge_ns(u, n) <= t) {
        n++;
    }
    return n;
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * The divided input clock changed: an active edge takes the unit's step
 * before the clock goes out, and TX's outputs follow it.
 */
static void uclk_in_edge(ESP32P4ParlioClk *u, bool level, int64_t when)
{
    ESP32P4ParlioState *s = u->s;
    uint16_t out = s->tx_out;
    bool cs = s->tx_cs_active;
    bool active, gate;

    if (level == u->in_level) {
        return;
    }
    u->in_level = level;
    active = level ^ !!(clk_cfg(s, u->unit) & CLK_I_INV);
    if (active && u->unit == TX) {
        tx_edge(s, &out, &cs);
    } else if (active) {
        rx_edge(s, when);
    }
    gate = u->unit == TX ? tx_gate(s, cs, out) : unit_gate(s, RX);
    unit_drive_clk(s, u->unit, gate, when);
    if (u->unit == TX) {
        tx_drive_cs(s, cs, when);
        tx_drive(s, out, when);
    }
}

/*
 * Time the internal clock's next edge while the unit is busy, and until
 * the clock in is low after it, where an idle clock rests. Edges are
 * taken one by one, each at its own time, however late the timer runs; a
 * clock that was resting picks up at the first edge after the present.
 */
static void uclk_schedule(ESP32P4ParlioClk *u)
{
    uint64_t n;

    if (!u->period || (!unit_busy(u->s, u->unit) && !u->in_level)) {
        timer_del(u->timer);
        u->running = false;
        return;
    }
    if (timer_pending(u->timer)) {
        return;
    }
    if (!u->running) {
        n = uclk_edge_after(u, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        if (n > u->next) {
            u->next = n;
            /* The clock in as it is before that edge, a rising one if even. */
            u->in_level = n & 1;
        }
        u->running = true;
    }
    timer_mod(u->timer, uclk_edge_ns(u, u->next));
}

/* Whether TX's next data cycle has its bits, or GDMA's frame has ended. */
static bool tx_can_shift(ESP32P4ParlioState *s)
{
    unsigned width = bus_width(s->tx_data_cfg);
    uint32_t avail;
    bool ends;

    if (s->tx_state != TXS_DATA || s->tx_all_sent || s->tx_byte_bits) {
        return true;
    }
    avail = tx_frame_bytes(s, &ends);
    return ends || avail >= (width == 16 ? 2u : 1u);
}

/*
 * Whether the unit's next active edge waits for the DMA: TX's FIFO has
 * not got its next cycle's bits, or RX's FIFO is full.
 */
static bool unit_waits_for_dma(ESP32P4ParlioState *s, int unit)
{
    if (unit == TX) {
        return unit_busy(s, TX) && !tx_can_shift(s);
    }
    return unit_busy(s, RX) && !fifo_room(&s->rx_fifo);
}

/* Move the internal clock's edges later, the next one to @when. */
static void uclk_slip(ESP32P4ParlioClk *u, int64_t when)
{
    int64_t delta = when - uclk_edge_ns(u, u->next);

    if (delta > 0) {
        u->base_ns += delta;
        u->s->slip_ns = MAX(u->s->slip_ns,
                            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.parlio]
 * An internal edge, at its own time. Under icount the timer runs at that
 * time; on the host's clock it may run behind it, and the GDMA channels,
 * which time their transfers from the present, then fall behind the edges
 * the unit catches up on: an active edge whose DMA has not kept up, and
 * the edges after one taken more than PARLIO_SLIP_NS late, slip to the
 * present instead, so that what goes over the bus stays whole.
 */
static void uclk_timer_cb(void *opaque)
{
    ESP32P4ParlioClk *u = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t n = u->next;
    int64_t when = uclk_edge_ns(u, n);
    bool level = !(n & 1);
    bool active = level ^ !!(clk_cfg(u->s, u->unit) & CLK_I_INV);

    if (when < now && active && unit_waits_for_dma(u->s, u->unit)) {
        uclk_slip(u, now + clock_ticks_to_ns(u->clk, 1) / 2);
        timer_mod(u->timer, uclk_edge_ns(u, n));
        return;
    }
    u->next++;
    uclk_in_edge(u, level, when);
    if (now - when > PARLIO_SLIP_NS) {
        uclk_slip(u, now + clock_ticks_to_ns(u->clk, 1) / 2);
    }
    uclk_schedule(u);
}

static void uclk_update(void *opaque, ClockEvent event)
{
    ESP32P4ParlioClk *u = opaque;

    timer_del(u->timer);
    u->running = false;
    u->period = clock_get(u->clk);
    u->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    u->next = 0;
    if (u->period) {
        /* A started clock starts low. */
        u->in_level = false;
        unit_redrive_clk(u->s, u->unit, u->base_ns);
    }
    uclk_schedule(u);
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void uclk_pad_in(ESP32P4ParlioClk *u, bool level)
{
    bool rise = level && !u->pad_level;
    bool fall = !level && u->pad_level;
    ESP32P4ClkDiv div;

    u->pad_level = level;
    if ((!rise && !fall) || !uclk_pad(u, &div)) {
        return;
    }
    if (div.numerator && div.denominator && !u->pad_frac_logged) {
        u->pad_frac_logged = true;
        qemu_log_mask(LOG_UNIMP, "%s: %s: a fractional divider of the pad "
                      "clock divides by its integer part\n",
                      TYPE_ESP32P4_PARLIO, unit_name[u->unit]);
    }
    if (div.integer <= 1) {
        uclk_in_edge(u, level, esp32_line_time());
        return;
    }
    if (!rise) {
        return;
    }
    if (u->pad_count == 0) {
        uclk_in_edge(u, true, esp32_line_time());
    } else if (u->pad_count == div.integer / 2) {
        uclk_in_edge(u, false, esp32_line_time());
    }
    u->pad_count = (u->pad_count + 1) % div.integer;
}

/* Inputs */

static void parlio_rxd_in(void *opaque, int n, int level)
{
    ESP32P4ParlioState *s = opaque;

    s->rxd = deposit32(s->rxd, n, 1, !!level);
}

static void parlio_tx_clk_in(void *opaque, int n, int level)
{
    ESP32P4ParlioState *s = opaque;

    uclk_pad_in(&s->uclk[TX], level);
}

static void parlio_rx_clk_in(void *opaque, int n, int level)
{
    ESP32P4ParlioState *s = opaque;

    uclk_pad_in(&s->uclk[RX], level);
}

static void tx_core_reset(ESP32P4ParlioState *s, int64_t when)
{
    s->tx_bitcnt = 0;
    s->tx_cycles = 0;
    s->tx_all_sent = false;
    s->tx_delay = 0;
    s->uclk[TX].pad_count = 0;
    tx_stop(s, when);
    unit_redrive_clk(s, TX, when);
}

static void rx_core_reset(ESP32P4ParlioState *s)
{
    s->rx_prev_en = false;
    s->rx_in_frame = false;
    s->rx_pulse_open = false;
    s->rx_bitcnt = 0;
    s->rx_byte = 0;
    s->rx_byte_bits = 0;
    s->uclk[RX].pad_count = 0;
    timer_del(s->rx_timeout_timer);
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void parlio_core_reset_in(void *opaque, int n, int level)
{
    ESP32P4ParlioState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (n == TX) {
        s->tx_reset_held = level;
        tx_core_reset(s, now);
    } else {
        s->rx_reset_held = level;
        rx_core_reset(s);
    }
    uclk_schedule(&s->uclk[n]);
}

static void parlio_sys_update(void *opaque, ClockEvent event)
{
    ESP32P4ParlioState *s = opaque;

    if (sys_running(s)) {
        tx_fill(s);
        rx_drain(s);
        rx_arm_timeout(s);
    } else {
        timer_del(s->rx_timeout_timer);
    }
}

/* Registers */

static uint64_t parlio_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4ParlioState *s = opaque;

    switch (addr) {
    case A_RX_MODE_CFG:
        return s->rx_mode_cfg;
    case A_RX_DATA_CFG:
        return s->rx_data_cfg;
    case A_RX_GENRL_CFG:
        return s->rx_genrl_cfg;
    case A_RX_START_CFG:
        return s->rx_start_cfg;
    case A_TX_DATA_CFG:
        return s->tx_data_cfg;
    case A_TX_START_CFG:
        return s->tx_start_cfg;
    case A_TX_GENRL_CFG:
        return s->tx_genrl_cfg;
    case A_FIFO_CFG:
        return s->fifo_cfg;
    case A_REG_UPDATE:
    case A_INT_CLR:
        return 0;
    case A_ST:
        return s->tx_fifo.num ? TX_READY : 0;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_RX_ST0:
        return (s->rx_cycles & 0x1f) << 8 | s->rx_bitcnt << 13;
    case A_RX_ST1:
        return s->rx_rd_bitcnt << 13;
    case A_TX_ST0:
        return (s->tx_cycles & 0x7f) << 6 | s->tx_bitcnt << 13;
    case A_RX_CLK_CFG:
        return s->rx_clk_cfg;
    case A_TX_CLK_CFG:
        return s->tx_clk_cfg;
    case A_TX_CS_CFG:
        return s->tx_cs_cfg;
    case A_CLK:
        return s->clk;
    case A_VERSION:
        return s->version;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PARLIO, addr);
        return 0;
    }
}

static void check_width(uint32_t data_cfg, const char *unit)
{
    if (!bus_width(data_cfg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s_BUS_WID_SEL %u is invalid\n",
                      TYPE_ESP32P4_PARLIO, unit, DATA_BUS_WID_SEL(data_cfg));
    }
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void parlio_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    ESP32P4ParlioState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = value, old;

    switch (addr) {
    case A_RX_MODE_CFG:
        s->rx_mode_cfg = v & RX_MODE_MASK;
        break;
    case A_RX_DATA_CFG:
        s->rx_data_cfg = v & DATA_CFG_MASK;
        check_width(s->rx_data_cfg, "RX");
        break;
    case A_RX_GENRL_CFG:
        s->rx_genrl_cfg = v & RX_GENRL_MASK;
        break;
    case A_RX_START_CFG:
        old = s->rx_start_cfg;
        s->rx_start_cfg = v & START;
        if (!(old & START) && (v & START)) {
            s->rx_in_frame = false;
            s->rx_pulse_open = false;
            s->rx_bitcnt = 0;
            s->rx_byte = 0;
            s->rx_byte_bits = 0;
        }
        if (!(v & START)) {
            timer_del(s->rx_timeout_timer);
        }
        unit_redrive_clk(s, RX, now);
        uclk_schedule(&s->uclk[RX]);
        break;
    case A_TX_DATA_CFG:
        s->tx_data_cfg = v & DATA_CFG_MASK;
        check_width(s->tx_data_cfg, "TX");
        break;
    case A_TX_START_CFG:
        old = s->tx_start_cfg;
        s->tx_start_cfg = v & START;
        if (!(v & START) && !s->tx_reset_held) {
            if (s->tx_state != TXS_IDLE && s->tx_state != TXS_DONE) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: TX_START cleared in a "
                              "frame\n", TYPE_ESP32P4_PARLIO);
            }
            tx_stop(s, now);
        } else if (!(old & START) && s->tx_state == TXS_DONE) {
            s->tx_state = TXS_IDLE;
        }
        uclk_schedule(&s->uclk[TX]);
        break;
    case A_TX_GENRL_CFG:
        s->tx_genrl_cfg = v & TX_GENRL_MASK;
        if (s->tx_state != TXS_DATA && !s->tx_reset_held) {
            tx_drive(s, TX_IDLE_VALUE(s->tx_genrl_cfg), now);
        }
        unit_redrive_clk(s, TX, now);
        break;
    case A_FIFO_CFG:
        s->fifo_cfg = v & FIFO_CFG_MASK;
        if (s->fifo_cfg & TX_FIFO_SRST) {
            fifo_reset(&s->tx_fifo);
        } else {
            tx_fill(s);
        }
        if (s->fifo_cfg & RX_FIFO_SRST) {
            fifo_reset(&s->rx_fifo);
        }
        break;
    case A_REG_UPDATE:
        if (v & RX_REG_UPDATE) {
            s->rx_mode_act = s->rx_mode_cfg;
            s->rx_data_act = s->rx_data_cfg;
            s->rx_genrl_act = s->rx_genrl_cfg;
            rx_arm_timeout(s);
            uclk_schedule(&s->uclk[RX]);
        }
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_MASK;
        parlio_update_irq(s);
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_MASK);
        parlio_update_irq(s);
        break;
    case A_RX_CLK_CFG:
        s->rx_clk_cfg = v & CLK_CFG_MASK;
        unit_redrive_clk(s, RX, now);
        break;
    case A_TX_CLK_CFG:
        s->tx_clk_cfg = v & CLK_CFG_MASK;
        unit_redrive_clk(s, TX, now);
        break;
    case A_TX_CS_CFG:
        s->tx_cs_cfg = v;
        break;
    case A_CLK:
        s->clk = v & PARLIO_CLK_EN;
        break;
    case A_VERSION:
        s->version = v & VERSION_MASK;
        break;
    case A_ST:
    case A_INT_ST:
    case A_RX_ST0:
    case A_RX_ST1:
    case A_TX_ST0:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PARLIO, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PARLIO, addr);
        break;
    }
}

static const MemoryRegionOps parlio_ops = {
    .read = parlio_read,
    .write = parlio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const ESP32P4GdmaPeriphOps parlio_dma_ops = {
    .tx_ready = tx_ready_cb,
    .rx_ready = rx_ready_cb,
};

/* Device */

static void parlio_reset_hold(Object *obj, ResetType type)
{
    ESP32P4ParlioState *s = ESP32P4_PARLIO(obj);

    s->rx_mode_cfg = RX_MODE_RESET;
    s->rx_data_cfg = DATA_CFG_RESET;
    s->rx_genrl_cfg = RX_GENRL_RESET;
    s->rx_start_cfg = 0;
    s->tx_data_cfg = DATA_CFG_RESET;
    s->tx_start_cfg = 0;
    s->tx_genrl_cfg = 0;
    s->fifo_cfg = 0;
    s->int_ena = 0;
    s->int_raw = 0;
    s->rx_clk_cfg = 0;
    s->tx_clk_cfg = 0;
    s->tx_cs_cfg = 0;
    s->clk = 0;
    s->version = VERSION_RESET;
    s->rx_mode_act = RX_MODE_RESET;
    s->rx_data_act = DATA_CFG_RESET;
    s->rx_genrl_act = RX_GENRL_RESET;
    s->rx_rd_bitcnt = 0;
    s->rx_cycles = 0;
    fifo_reset(&s->tx_fifo);
    fifo_reset(&s->rx_fifo);
    for (int i = 0; i < 2; i++) {
        timer_del(s->uclk[i].timer);
        s->uclk[i].running = false;
        s->uclk[i].in_level = false;
    }
    rx_core_reset(s);
}

static void parlio_reset_exit(Object *obj, ResetType type)
{
    ESP32P4ParlioState *s = ESP32P4_PARLIO(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Drive every output to its reset level. */
    s->tx_out = 0;
    s->tx_cs_active = false;
    for (int i = 0; i < ESP32P4_PARLIO_LINES; i++) {
        qemu_set_irq(s->txd[i], 0);
    }
    qemu_set_irq(s->tx_cs, 1);
    for (int i = 0; i < 2; i++) {
        s->uclk[i].out_level = false;
        qemu_set_irq(i == TX ? s->tx_clk_out : s->rx_clk_out, 0);
    }
    tx_core_reset(s, now);
    parlio_update_irq(s);
}

static void parlio_init(Object *obj)
{
    ESP32P4ParlioState *s = ESP32P4_PARLIO(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &parlio_ops, s, "esp32p4.parlio",
                          ESP32P4_PARLIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[RX]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[TX]);
    qdev_init_gpio_out_named(dev, s->txd, ESP32P4_PARLIO_TXD,
                             ESP32P4_PARLIO_LINES);
    qdev_init_gpio_out_named(dev, &s->tx_clk_out, ESP32P4_PARLIO_TX_CLK_OUT,
                             1);
    qdev_init_gpio_out_named(dev, &s->tx_cs, ESP32P4_PARLIO_TX_CS, 1);
    qdev_init_gpio_out_named(dev, &s->rx_clk_out, ESP32P4_PARLIO_RX_CLK_OUT,
                             1);
    qdev_init_gpio_in_named(dev, parlio_rxd_in, ESP32P4_PARLIO_RXD,
                            ESP32P4_PARLIO_LINES);
    qdev_init_gpio_in_named(dev, parlio_tx_clk_in, ESP32P4_PARLIO_TX_CLK_IN,
                            1);
    qdev_init_gpio_in_named(dev, parlio_rx_clk_in, ESP32P4_PARLIO_RX_CLK_IN,
                            1);
    qdev_init_gpio_in_named(dev, parlio_core_reset_in,
                            ESP32P4_PARLIO_CORE_RESET, 2);
    s->sys_clk = qdev_init_clock_in(dev, "sys", parlio_sys_update, s,
                                    ClockUpdate);
    s->rx_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rx_timeout_cb, s);
    for (int i = 0; i < 2; i++) {
        ESP32P4ParlioClk *u = &s->uclk[i];

        u->s = s;
        u->unit = i;
        u->clk = qdev_init_clock_in(dev, i == TX ? "tx" : "rx", uclk_update,
                                    u, ClockUpdate);
        u->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uclk_timer_cb, u);
    }
}

static void parlio_realize(DeviceState *dev, Error **errp)
{
    ESP32P4ParlioState *s = ESP32P4_PARLIO(dev);

    if (!s->gdma) {
        error_setg(errp, "%s: the 'gdma' link must be set",
                   TYPE_ESP32P4_PARLIO);
        return;
    }
    esp32p4_gdma_attach(s->gdma, PERIPH, &parlio_dma_ops, s);
}

static void parlio_finalize(Object *obj)
{
    ESP32P4ParlioState *s = ESP32P4_PARLIO(obj);

    timer_free(s->rx_timeout_timer);
    for (int i = 0; i < 2; i++) {
        timer_free(s->uclk[i].timer);
    }
}

static const Property parlio_properties[] = {
    DEFINE_PROP_LINK("gdma", ESP32P4ParlioState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
    DEFINE_PROP_LINK("clkrst", ESP32P4ParlioState, clkrst,
                     TYPE_ESP32P4_CLKRST, ESP32P4ClkRstState *),
};

static const VMStateDescription vmstate_parlio_fifo = {
    .name = "esp32p4-parlio-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, ESP32P4ParlioFifo, ESP32P4_PARLIO_FIFO),
        VMSTATE_UINT8_ARRAY(flags, ESP32P4ParlioFifo, ESP32P4_PARLIO_FIFO),
        VMSTATE_UINT32(head, ESP32P4ParlioFifo),
        VMSTATE_UINT32(num, ESP32P4ParlioFifo),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_parlio_clk = {
    .name = "esp32p4-parlio-clk",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4ParlioClk),
        VMSTATE_TIMER_PTR(timer, ESP32P4ParlioClk),
        VMSTATE_INT64(base_ns, ESP32P4ParlioClk),
        VMSTATE_UINT64(period, ESP32P4ParlioClk),
        VMSTATE_UINT64(next, ESP32P4ParlioClk),
        VMSTATE_BOOL(running, ESP32P4ParlioClk),
        VMSTATE_BOOL(in_level, ESP32P4ParlioClk),
        VMSTATE_BOOL(pad_level, ESP32P4ParlioClk),
        VMSTATE_UINT32(pad_count, ESP32P4ParlioClk),
        VMSTATE_BOOL(out_level, ESP32P4ParlioClk),
        VMSTATE_BOOL(pad_frac_logged, ESP32P4ParlioClk),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_parlio = {
    .name = TYPE_ESP32P4_PARLIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(sys_clk, ESP32P4ParlioState),
        VMSTATE_TIMER_PTR(rx_timeout_timer, ESP32P4ParlioState),
        VMSTATE_STRUCT_ARRAY(uclk, ESP32P4ParlioState, 2, 1,
                             vmstate_parlio_clk, ESP32P4ParlioClk),
        VMSTATE_UINT32(rx_mode_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_data_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_genrl_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_start_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_data_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_start_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_genrl_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(fifo_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(int_ena, ESP32P4ParlioState),
        VMSTATE_UINT32(int_raw, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_clk_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_clk_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_cs_cfg, ESP32P4ParlioState),
        VMSTATE_UINT32(clk, ESP32P4ParlioState),
        VMSTATE_UINT32(version, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_mode_act, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_data_act, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_genrl_act, ESP32P4ParlioState),
        VMSTATE_BOOL(tx_reset_held, ESP32P4ParlioState),
        VMSTATE_UINT8(tx_state, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_delay, ESP32P4ParlioState),
        VMSTATE_BOOL(tx_all_sent, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_bitcnt, ESP32P4ParlioState),
        VMSTATE_UINT32(tx_cycles, ESP32P4ParlioState),
        VMSTATE_UINT8(tx_byte, ESP32P4ParlioState),
        VMSTATE_UINT8(tx_byte_flags, ESP32P4ParlioState),
        VMSTATE_UINT8(tx_byte_bits, ESP32P4ParlioState),
        VMSTATE_UINT16(tx_out, ESP32P4ParlioState),
        VMSTATE_BOOL(tx_cs_active, ESP32P4ParlioState),
        VMSTATE_STRUCT(tx_fifo, ESP32P4ParlioState, 1, vmstate_parlio_fifo,
                       ESP32P4ParlioFifo),
        VMSTATE_BOOL(rx_reset_held, ESP32P4ParlioState),
        VMSTATE_UINT16(rxd, ESP32P4ParlioState),
        VMSTATE_BOOL(rx_prev_en, ESP32P4ParlioState),
        VMSTATE_BOOL(rx_in_frame, ESP32P4ParlioState),
        VMSTATE_BOOL(rx_pulse_open, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_bitcnt, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_rd_bitcnt, ESP32P4ParlioState),
        VMSTATE_UINT32(rx_cycles, ESP32P4ParlioState),
        VMSTATE_UINT8(rx_byte, ESP32P4ParlioState),
        VMSTATE_UINT8(rx_byte_bits, ESP32P4ParlioState),
        VMSTATE_INT64(rx_last_data_ns, ESP32P4ParlioState),
        VMSTATE_INT64(rx_timeout_from, ESP32P4ParlioState),
        VMSTATE_INT64(slip_ns, ESP32P4ParlioState),
        VMSTATE_STRUCT(rx_fifo, ESP32P4ParlioState, 1, vmstate_parlio_fifo,
                       ESP32P4ParlioFifo),
        VMSTATE_END_OF_LIST()
    },
};

static void parlio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = parlio_realize;
    dc->vmsd = &vmstate_parlio;
    device_class_set_props(dc, parlio_properties);
    rc->phases.hold = parlio_reset_hold;
    rc->phases.exit = parlio_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static const TypeInfo parlio_types[] = {
    {
        .name           = TYPE_ESP32P4_PARLIO,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(ESP32P4ParlioState),
        .instance_init  = parlio_init,
        .instance_finalize = parlio_finalize,
        .class_init     = parlio_class_init,
    },
};

DEFINE_TYPES(parlio_types)
