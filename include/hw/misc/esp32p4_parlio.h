/*
 * ESP32-P4 parallel IO controller (PARLIO)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_PARLIO_H
#define HW_MISC_ESP32P4_PARLIO_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/core/clock.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_PARLIO "esp32p4-parlio"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ParlioState, ESP32P4_PARLIO)

#define ESP32P4_PARLIO_BASE     0x500cf000
#define ESP32P4_PARLIO_SIZE     0x1000

#define ESP32P4_PARLIO_LINES    16
/* The units' asynchronous FIFOs, in bytes. */
#define ESP32P4_PARLIO_FIFO     16

/*
 * Named GPIOs, the units' pad signals, which the machine connects to the
 * GPIO matrix's signals of the same names (TRM table 10.12-1):
 *
 *   ESP32P4_PARLIO_TXD, "txd" (output, 16 lines): PARLIO_TX_DATA0..15.
 *   ESP32P4_PARLIO_TX_CLK_OUT, "tx-clk-out" (output): PARLIO_TX_CLK.
 *   ESP32P4_PARLIO_TX_CS, "tx-cs" (output): PARLIO_TX_CS, active low.
 *   ESP32P4_PARLIO_RX_CLK_OUT, "rx-clk-out" (output): PARLIO_RX_CLK.
 *   ESP32P4_PARLIO_RXD, "rxd" (input, 16 lines): PARLIO_RX_DATA0..15.
 *   ESP32P4_PARLIO_TX_CLK_IN, "tx-clk-in" (input): PAD_PARLIO_TX_CLK,
 *     PARLIO_TX_CLK's pad source.
 *   ESP32P4_PARLIO_RX_CLK_IN, "rx-clk-in" (input): PAD_PARLIO_RX_CLK.
 *   ESP32P4_PARLIO_CORE_RESET, "core-reset" (input, 2 lines):
 *     RST_EN_PARLIO_TX and RST_EN_PARLIO_RX, each high while that unit's
 *     core clock domain is held in reset.
 *
 * Clock inputs: "tx" and "rx", PARLIO_TX_CLK and PARLIO_RX_CLK from their
 * internal sources; "sys", PARLIO_SYS_CLK, the GDMA interface's. Links:
 * "gdma", GDMA-AXI, which serves PARLIO (peripheral 3), and "clkrst",
 * asked whether a unit's clock comes from its pad
 * (esp32p4_clkrst_pad_source()); "gpio", the GPIO matrix
 * (TYPE_ESP32_SIG_ROUTES), asked which of the units' outputs reach a pad
 * something watches, whose edges are then timed one by one (without it,
 * all are). Sysbus IRQs: 0, HP_PARLIO_RX_INTR; 1, HP_PARLIO_TX_INTR.
 */
#define ESP32P4_PARLIO_TXD          "txd"
#define ESP32P4_PARLIO_TX_CLK_OUT   "tx-clk-out"
#define ESP32P4_PARLIO_TX_CS        "tx-cs"
#define ESP32P4_PARLIO_RX_CLK_OUT   "rx-clk-out"
#define ESP32P4_PARLIO_RXD          "rxd"
#define ESP32P4_PARLIO_TX_CLK_IN    "tx-clk-in"
#define ESP32P4_PARLIO_RX_CLK_IN    "rx-clk-in"
#define ESP32P4_PARLIO_CORE_RESET   "core-reset"

#define ESP32P4_PARLIO_TX           0
#define ESP32P4_PARLIO_RX           1

/* A unit's FIFO: bytes, or marks, each with what it carries. */
typedef struct ESP32P4ParlioFifo {
    uint8_t data[ESP32P4_PARLIO_FIFO];
    uint8_t flags[ESP32P4_PARLIO_FIFO];
    uint32_t head;
    uint32_t num;
} ESP32P4ParlioFifo;

/*
 * A unit's core clock: its internal source's edges, timed from base_ns
 * on the clock's own edges, or its pad's, divided; and the clock it puts
 * out.
 */
typedef struct ESP32P4ParlioClk {
    struct ESP32P4ParlioState *s;
    int unit;
    Clock *clk;
    QEMUTimer *timer;
    int64_t base_ns;        /* when the internal clock was last timed */
    ESP32P4ClkPeriod half;  /* ... half its period then, exactly */
    ESP32P4ClkEdges half_e; /* its half periods, on its edges */
    int64_t k0;             /* the half period internal edge 0 ends */
    uint64_t next;          /* the next internal edge's number */
    uint64_t wake;          /* the first edge the timer must take at its time */
    bool running;           /* edges being taken, one after the other */
    bool in_level;          /* the divided input clock */
    bool pad_level;         /* the pad clock as last seen */
    uint32_t pad_count;     /* the pad clock's rising edges, divided */
    bool out_level;         /* the level put out */
    bool line_level;        /* ... as driven on its line */
    bool watched;           /* its line reaches a pad something watches */
    bool pad_frac_logged;
} ESP32P4ParlioClk;

struct ESP32P4ParlioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[2];
    qemu_irq txd[ESP32P4_PARLIO_LINES];
    qemu_irq tx_clk_out;
    qemu_irq tx_cs;
    qemu_irq rx_clk_out;
    Clock *sys_clk;
    QEMUTimer *rx_timeout_timer;
    ESP32P4ParlioClk uclk[2];

    /* Registers. */
    uint32_t rx_mode_cfg;
    uint32_t rx_data_cfg;
    uint32_t rx_genrl_cfg;
    uint32_t rx_start_cfg;
    uint32_t tx_data_cfg;
    uint32_t tx_start_cfg;
    uint32_t tx_genrl_cfg;
    uint32_t fifo_cfg;
    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t rx_clk_cfg;
    uint32_t tx_clk_cfg;
    uint32_t tx_cs_cfg;
    uint32_t clk;
    uint32_t version;
    /* The RX configuration in effect, as RX_REG_UPDATE last took it. */
    uint32_t rx_mode_act;
    uint32_t rx_data_act;
    uint32_t rx_genrl_act;

    /* The TX unit. */
    bool tx_reset_held;
    uint8_t tx_state;
    uint32_t tx_delay;          /* edges left of a CS delay */
    bool tx_all_sent;           /* the frame's last bit has gone */
    uint32_t tx_bitcnt;
    uint32_t tx_cycles;
    uint8_t tx_byte;            /* the byte being shifted out */
    uint8_t tx_byte_flags;
    uint8_t tx_byte_bits;       /* ... its bits still to go */
    uint16_t tx_out;            /* TXD as put out */
    uint16_t txd_line;          /* ... as driven on the lines */
    uint16_t txd_watched;       /* the lines that reach a watched pad */
    bool tx_cs_active;
    bool cs_line;               /* CS's line level */
    bool cs_watched;
    ESP32P4ParlioFifo tx_fifo;

    /* The RX unit. */
    bool rx_reset_held;
    uint16_t rxd;               /* RXD's levels */
    bool rx_prev_en;            /* the enable at the previous sample */
    bool rx_in_frame;           /* a frame has had data, not yet ended */
    bool rx_pulse_open;         /* between a start pulse and its end */
    uint32_t rx_bitcnt;
    uint32_t rx_rd_bitcnt;
    uint32_t rx_cycles;
    uint8_t rx_byte;
    uint8_t rx_byte_bits;
    int64_t rx_last_data_ns;
    int64_t rx_timeout_from;
    bool rx_timeout_on;         /* the receive timeout counting */
    bool lag_logged;
    ESP32P4ParlioFifo rx_fifo;

    ESP32P4GdmaState *gdma;
    ESP32P4ClkRstState *clkrst;
    DeviceState *gpio;
    Notifier line_source;
    Notifier route_notifier;
    bool in_catch_up;
    bool routes_stale;
};

#endif
