/*
 * ESP32-P4 RMT, the remote control (pulse train) peripheral
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_RMT_H
#define HW_MISC_ESP32P4_RMT_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/dma/esp32p4_gdma.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_RMT "esp32p4-rmt"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4RmtState, ESP32P4_RMT)

#define ESP32P4_RMT_BASE        0x500a2000
#define ESP32P4_RMT_SIZE        0x1000

/* Channels 0 to 3 transmit, channels 4 to 7 receive. */
#define ESP32P4_RMT_TX_CHANNELS 4
#define ESP32P4_RMT_RX_CHANNELS 4

/*
 * The RAM: 384 words, eight 48-word blocks, at offset 0x800. Channel n
 * (0 to 7, RX channel m being channel 4 + m) owns block n and, with
 * MEM_SIZE > 1, the blocks after it.
 */
#define ESP32P4_RMT_RAM_WORDS   384
#define ESP32P4_RMT_BLOCK_WORDS 48
#define ESP32P4_RMT_RAM_OFFSET  0x800
#define ESP32P4_RMT_REG_SIZE    0x800

/*
 * Inputs and outputs:
 *
 *   - Clock input "sclk": rmt_sclk, HP_SYS_CLKRST's RMT_CLK (source,
 *     fractional divider and gate), which every channel divides by its
 *     DIV_CNT and whose cycles time the carrier and the RX filter.
 *   - Named GPIO output ESP32P4_RMT_SIG_OUT, 4 lines: rmt_sig_pad_out0..3,
 *     the TX channels' outputs (GPIO matrix output signals 246 to 249).
 *   - Named GPIO input ESP32P4_RMT_SIG_IN, 4 lines: rmt_sig_pad_in0..3,
 *     the RX channels' inputs (GPIO matrix input signals 246 to 249).
 *   - Sysbus IRQ 0: RMT_INTR (interrupt matrix source 43).
 *   - Link "gdma": GDMA-AHB, which serves TX channel 3 and RX channel 7
 *     (PERI_SEL 10) when their DMA_ACCESS_EN is set.
 *
 * The RMT has no ETM events or tasks on the P4.
 */
#define ESP32P4_RMT_SIG_OUT     "sig-out"
#define ESP32P4_RMT_SIG_IN      "sig-in"

typedef struct ESP32P4RmtTx {
    /* Registers as written. */
    uint32_t conf0;
    uint32_t carrier_duty;
    uint32_t tx_lim;
    /* The fields CONF_UPDATE latches, as last latched. */
    uint32_t act_conf0;
    uint32_t act_carrier_duty;
    uint32_t act_tx_lim;

    /*
     * Times as counts of rmt_sclk cycles: cycle(t) = anchor_c +
     * (t - anchor_ns) / period. The channel clock ticks on the cycles
     * that are div_phase modulo the divider.
     */
    int64_t anchor_ns;
    uint64_t anchor_c;
    uint32_t div_phase;

    uint8_t state;
    /* Transmitter's read pointer, an absolute RAM word address. */
    uint32_t raddr;
    uint8_t half;
    uint32_t word;
    uint64_t event_c;
    uint64_t entry_end_c;
    uint8_t level;
    /* An entry has been sent since the pass started. */
    bool pass_sent;
    bool carrier;
    bool carrier_high;
    uint64_t carrier_end_c;
    uint32_t thr_count;
    uint32_t loop_count;
    uint8_t end_level;
    bool mem_empty;
    uint8_t out_level;

    /* The APB FIFO's (and DMA's) write pointer, absolute. */
    uint32_t apb_waddr;
    bool apb_wr_err;
    /* DMA: words written ahead of the transmitter, and a partial word. */
    uint32_t dma_fill;
    uint32_t dma_word;
    uint8_t dma_bytes;
} ESP32P4RmtTx;

typedef struct ESP32P4RmtRx {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t carrier_rm;
    uint32_t rx_lim;
    uint32_t act_conf0;
    uint32_t act_conf1;
    uint32_t act_carrier_rm;
    uint32_t act_rx_lim;
    /* REF_CNT_RST written, waiting for CONF_UPDATE. */
    bool div_rst_pending;

    int64_t anchor_ns;
    uint64_t anchor_c;
    uint32_t div_phase;

    /*
     * The input pipeline: raw input, the glitch filter (a change is taken
     * once it has held for RX_FILTER_THRES rmt_sclk cycles), the sampler
     * (a change is seen on the next channel clock tick), the demodulator,
     * then the recorder.
     */
    uint8_t in_raw;
    uint8_t in_filt;
    bool filt_pending;
    int64_t filt_ns;
    uint8_t in_samp;
    bool samp_pending;
    uint64_t samp_c;
    uint8_t demod;
    bool demod_pending;
    uint64_t demod_c;

    bool frame;
    uint8_t level;
    uint64_t edge_c;
    /* Receiver's write pointer, absolute. */
    uint32_t waddr;
    uint8_t half;
    bool mem_full;
    bool owner_err;
    uint32_t thr_count;

    /* The APB FIFO's (and DMA's) read pointer, absolute. */
    uint32_t apb_raddr;
    bool apb_rd_err;
    /* DMA: words written by the receiver and not yet given to GDMA. */
    uint32_t dma_unread;
} ESP32P4RmtRx;

struct ESP32P4RmtState {
    SysBusDevice parent_obj;

    MemoryRegion container;
    MemoryRegion iomem;
    MemoryRegion ram_iomem;
    qemu_irq irq;
    qemu_irq sig_out[ESP32P4_RMT_TX_CHANNELS];
    Clock *sclk;
    QEMUTimer timer;
    Notifier line_source;
    ESP32P4GdmaState *gdma;

    uint32_t ram[ESP32P4_RMT_RAM_WORDS];
    /*
     * Per RAM word, for RX channel 7's DMA: 0, or the bytes (2 or 4) of a
     * word holding an end marker that GDMA is to take as the frame's end.
     */
    uint8_t eof_bytes[ESP32P4_RMT_RAM_WORDS];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t sys_conf;
    uint32_t tx_sim;
    bool act_tx_sim_en;
    uint8_t tx_armed;
    uint32_t date;
    ESP32P4RmtTx tx[ESP32P4_RMT_TX_CHANNELS];
    ESP32P4RmtRx rx[ESP32P4_RMT_RX_CHANNELS];

    /* Bringing the channels up to date; inputs arriving meanwhile wait. */
    bool syncing;
};

#endif
