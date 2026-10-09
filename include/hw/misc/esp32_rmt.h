/*
 * ESP32 RMT, the remote control (pulse train) peripheral
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_RMT_H
#define HW_MISC_ESP32_RMT_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"

#define TYPE_ESP32_RMT "esp32.rmt"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32RmtState, ESP32_RMT)

#define ESP32_RMT_CHANNELS      8
/* RMT RAM: 512 words, eight 64-word blocks, at offset 0x800 */
#define ESP32_RMT_RAM_WORDS     512
#define ESP32_RMT_BLOCK_WORDS   64
#define ESP32_RMT_RAM_OFFSET    0x800
#define ESP32_RMT_REG_SIZE      0x800
#define ESP32_RMT_SIZE          0x1000

/*
 * Signal lines, connected by the SoC to the GPIO matrix:
 * - ESP32_RMT_SIG_OUT (gpio-out): RMT_SIG_OUTn, channel n's output.
 * - ESP32_RMT_SIG_IN (gpio-in): RMT_SIG_INn, channel n's input.
 */
#define ESP32_RMT_SIG_OUT       "esp32-rmt-sig-out"
#define ESP32_RMT_SIG_IN        "esp32-rmt-sig-in"

/* GPIO matrix signal indices of channel 0; channel n is at + n */
#define ESP32_SIG_RMT_IN0       83
#define ESP32_SIG_RMT_OUT0      87

typedef enum Esp32RmtTxState {
    ESP32_RMT_TX_IDLE,
    /* Waiting for the channel clock tick on which the first entry starts */
    ESP32_RMT_TX_STARTING,
    ESP32_RMT_TX_ENTRY,
} Esp32RmtTxState;

/*
 * One channel. Times are kept as counts of the channel's base clock
 * (APB_CLK or REF_TICK), "cycles": cycle(t) = anchor_c + (t - anchor_ns) /
 * base period. The channel clock ticks on the cycles that are multiples of
 * the divider.
 */
typedef struct Esp32RmtChannel {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t carrier_duty;
    uint32_t tx_lim;

    int64_t anchor_ns;
    uint64_t anchor_c;

    /* Transmitter */
    uint8_t tx_state;
    uint32_t tx_raddr;
    uint8_t tx_half;
    uint32_t tx_word;
    uint64_t tx_event_c;
    uint64_t tx_entry_end_c;
    uint8_t tx_level;
    bool tx_carrier;
    bool tx_carrier_high;
    uint64_t tx_carrier_end_c;
    uint32_t tx_thr_count;
    uint8_t tx_end_level;
    bool mem_empty;

    /* Output as driven onto RMT_SIG_OUTn */
    uint8_t out_level;

    /*
     * Receiver input pipeline: raw input, then the glitch filter (a
     * change is taken once it has held for FILTER_THRES APB_CLK cycles),
     * then the sampler (a change is seen on the next channel clock tick),
     * then the recorder.
     */
    uint8_t in_raw;
    uint8_t in_filt;
    bool filt_pending;
    int64_t filt_ns;
    uint8_t in_samp;
    bool samp_pending;
    uint64_t samp_c;

    bool rx_frame;
    uint8_t rx_level;
    uint64_t rx_edge_c;
    uint32_t rx_waddr;
    uint8_t rx_half;
    bool mem_full;
    bool owner_err;

    /* APB FIFO access pointer, relative to the channel's memory */
    uint32_t apb_ptr;
    bool apb_rd_err;
    bool apb_wr_err;
} Esp32RmtChannel;

struct Esp32RmtState {
    SysBusDevice parent_obj;

    MemoryRegion container;
    MemoryRegion iomem;
    MemoryRegion ram_iomem;
    qemu_irq irq;
    qemu_irq sig_out[ESP32_RMT_CHANNELS];
    Clock *apb_clk;
    Clock *ref_tick_clk;
    QEMUTimer timer;

    uint32_t ram[ESP32_RMT_RAM_WORDS];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t apb_conf;
    uint32_t date;
    Esp32RmtChannel ch[ESP32_RMT_CHANNELS];

    /*
     * While the model drives an output change that happened at a past
     * virtual time, the time of that change; inputs that change in
     * response take it as theirs.
     */
    bool emitting;
    int64_t emit_ns;
};

#endif
