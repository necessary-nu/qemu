/*
 * ESP32-P4 general-purpose SPI controllers, GP-SPI2 and GP-SPI3
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_ESP32P4_SPI_H
#define HW_SSI_ESP32P4_SPI_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/ssi/ssi.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_SPI "esp32p4-spi"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4SpiState, ESP32P4_SPI)

#define ESP32P4_SPI2_BASE       0x500d0000
#define ESP32P4_SPI3_BASE       0x500d1000
#define ESP32P4_SPI_REGS_SIZE   0x1000

/*
 * The controller's bus signals (TRM table 46.5-2), the index of each in
 * the named GPIO groups below. The data lanes come first, lane n carrying
 * bit n of a multi-line transfer: D (MOSI, SIO0), Q (MISO, SIO1), WP
 * (SIO2), HD (SIO3) and, on GP-SPI2 only, IO4 to IO7; then the clock, the
 * chip selects (CS0 to CS5 on GP-SPI2, CS0 to CS2 on GP-SPI3) and GP-SPI2's
 * data strobe.
 */
enum {
    ESP32P4_SPI_LINE_D,
    ESP32P4_SPI_LINE_Q,
    ESP32P4_SPI_LINE_WP,
    ESP32P4_SPI_LINE_HD,
    ESP32P4_SPI_LINE_IO4,
    ESP32P4_SPI_LINE_IO5,
    ESP32P4_SPI_LINE_IO6,
    ESP32P4_SPI_LINE_IO7,
    ESP32P4_SPI_LINE_CK,
    ESP32P4_SPI_LINE_CS0,
    ESP32P4_SPI_LINE_CS5 = ESP32P4_SPI_LINE_CS0 + 5,
    ESP32P4_SPI_LINE_DQS,
    ESP32P4_SPI_LINES,
};
#define ESP32P4_SPI_LANES       8
#define ESP32P4_SPI_CS_MAX      6

/*
 * How the controller connects to the pads
 * =======================================
 *
 *   ESP32P4_SPI_SIG_OUT, "sig-out" (output, ESP32P4_SPI_LINES lines): the
 *     level the controller drives on each signal, for the GPIO matrix's
 *     "sig-out" line of the signal's number;
 *   ESP32P4_SPI_SIG_OE, "sig-oe" (output): each signal's output enable,
 *     for the matrix's "sig-oe";
 *   ESP32P4_SPI_SIG_IN, "sig-in" (input): the level the matrix or the IO
 *     MUX delivers on each input signal (the data lanes, the clock and
 *     CS0, which a slave takes).
 *
 * esp32p4_spi_line_signal() gives the matrix signal number of each line
 * (TRM table 10.12-1), or -1 where the controller has no such signal;
 * esp32p4_spi_line_has_input() says which lines are also inputs.
 *
 * The board's SPI devices are QEMU SSI peripherals on the controller's
 * bus, named "spi2" or "spi3":
 *
 *   -device w25q32,bus=spi2,cs=0
 *
 * A device with "cs" n is wired to the pads that carry the controller's
 * chip select n and its data lines, wherever software routes them: it is
 * selected while the pad that carries CSn is at its active level (low,
 * or high for a device with an active-high select, which software then
 * drives with SPI_MASTER_CS_POL), it hears the data lanes the controller
 * drives onto pads, and the controller hears it on a data input whose pad
 * no signal of the chip drives. A chip select or data lane routed to no
 * pad reaches no device.
 *
 * The two controllers reach each other through the pads as well: one in
 * slave mode is clocked by the other when its clock input comes, through
 * the matrix or the IO MUX, from the master's clock output, and each sees
 * on its data inputs whichever data outputs the routing connects to them.
 * The machine links each to the other through the "peer" property.
 *
 * Other links and clocks: "gdma" (GDMA-AXI, which serves both through
 * ESP32P4_GDMA_AXI_SPI2 and _SPI3), "gpio" (the matrix, for the routing),
 * the clock inputs "mst" (clk_spi_mst, the master's module clock) and
 * "sys" (the AHB side the controller's data path runs on), and the
 * property "host", 2 or 3. The interrupt is sysbus IRQ 0, SPI2_INTR or
 * SPI3_INTR.
 */
#define ESP32P4_SPI_SIG_OUT     "sig-out"
#define ESP32P4_SPI_SIG_OE      "sig-oe"
#define ESP32P4_SPI_SIG_IN      "sig-in"

int esp32p4_spi_line_signal(unsigned host, unsigned line);
bool esp32p4_spi_line_has_input(unsigned host, unsigned line);

/* The controller's asynchronous FIFOs towards GDMA (TRM figure 46.5-2). */
#define ESP32P4_SPI_AFIFO       64

typedef struct ESP32P4SpiFifo {
    uint8_t data[ESP32P4_SPI_AFIFO];
    uint8_t flags[ESP32P4_SPI_AFIFO];
    uint32_t head;
    uint32_t num;
} ESP32P4SpiFifo;

/* A state of a master transaction (TRM figure 46.5-5). */
typedef struct ESP32P4SpiPhase {
    uint8_t kind;
    /* The data lanes of a CMD, ADDR or data state: 1, 2, 4 or 8. */
    uint8_t lanes;
    /* The bits a cycle of the state carries for a QEMU SSI device. */
    uint8_t ssi_lanes;
    /* The bits of a CMD, ADDR or data state. */
    uint32_t bits;
    /* The SPI_CLK cycles of a CMD, ADDR, DUMMY or data state. */
    uint32_t beats;
    /* The state's length in half SPI_CLK cycles. */
    uint32_t halves;
} ESP32P4SpiPhase;

#define ESP32P4_SPI_PHASES      8

/* A cycle of a master transaction, generated ahead of its time. */
typedef struct ESP32P4SpiBeat {
    uint8_t phase;
    /* The master's output levels, bit n for data lane n. */
    uint8_t out;
    /* The bits the cycle carries for QEMU SSI devices, and their answer. */
    uint8_t ssi_bits;
    uint8_t ssi_out;
    uint8_t ssi_in;
    /* The index in the SSI bit stream of the cycle's first bit. */
    uint32_t ssi_pos;
} ESP32P4SpiBeat;

#define ESP32P4_SPI_RING        16

/* The source of one of a controller's data inputs in a transfer. */
typedef struct ESP32P4SpiLaneSrc {
    uint8_t kind;
    uint8_t lane;
    bool inv;
} ESP32P4SpiLaneSrc;

/* The registers the master copies into its clock domain (SPI_UPDATE). */
enum {
    ESP32P4_SPI_M_ADDR,
    ESP32P4_SPI_M_CTRL,
    ESP32P4_SPI_M_CLOCK,
    ESP32P4_SPI_M_USER,
    ESP32P4_SPI_M_USER1,
    ESP32P4_SPI_M_USER2,
    ESP32P4_SPI_M_MS_DLEN,
    ESP32P4_SPI_M_MISC,
    ESP32P4_SPI_M_DIN_MODE,
    ESP32P4_SPI_M_DIN_NUM,
    ESP32P4_SPI_M_DOUT_MODE,
    ESP32P4_SPI_M_CONF_BITLEN,
    ESP32P4_SPI_M_REGS,
};

struct ESP32P4SpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq sig_out[ESP32P4_SPI_LINES];
    qemu_irq sig_oe[ESP32P4_SPI_LINES];
    Clock *mst_clk;
    Clock *sys_clk;
    QEMUTimer *timer;
    QEMUTimer *update_timer;
    SSIBus *bus;
    Notifier route_notifier;

    uint8_t host;
    ESP32P4GdmaState *gdma;
    ESP32P4GpioState *gpio;
    ESP32P4SpiState *peer;

    /* Registers, as the bus sees them. */
    uint32_t cmd;
    uint32_t addr;
    uint32_t ctrl;
    uint32_t clock;
    uint32_t user;
    uint32_t user1;
    uint32_t user2;
    uint32_t ms_dlen;
    uint32_t misc;
    uint32_t din_mode;
    uint32_t din_num;
    uint32_t dout_mode;
    uint32_t dma_conf;
    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t w[16];
    uint32_t slave;
    uint32_t slave1;
    uint32_t clk_gate;
    uint32_t date;

    /* The master's copy of its configuration, as SPI_UPDATE last took it. */
    uint32_t mreg[ESP32P4_SPI_M_REGS];
    bool update_pending;

    /* Levels the controller drives and receives. */
    uint8_t out_level[ESP32P4_SPI_LINES];
    uint8_t oe_level[ESP32P4_SPI_LINES];
    uint8_t in_level[ESP32P4_SPI_LINES];
    bool lines_driven;

    /* The data path's FIFOs: DMA TX data, and received data for DMA. */
    ESP32P4SpiFifo txf;
    ESP32P4SpiFifo rxf;
    /* The last byte the master sent from the DMA. */
    uint8_t tx_last;
    /* Slave segmented transfer: received data dropped until reset. */
    bool rx_drop;
    /* Bits pushed to the RX DMA since its last end of frame. */
    uint32_t rx_bits;

    /*
     * The master: a transaction (or configurable segmented transfer) in
     * progress, its configuration, its states and where it is in them.
     */
    bool busy;
    bool waiting;           /* SPI_USR set, waiting for TX data or a clock */
    bool stalled;           /* waiting for the DMA */
    bool sct;               /* a configurable segmented transfer */
    bool abort;             /* an error ends the transfer at DONE */
    bool entered;           /* the current state's lines are set */
    bool running;
    bool cs_on;             /* from PREP to the end of DONE */
    bool cs_held;           /* SPI_CS_KEEP_ACTIVE ended the last transfer */
    uint32_t x[ESP32P4_SPI_M_REGS];
    ESP32P4SpiPhase ph[ESP32P4_SPI_PHASES];
    uint8_t nph;
    uint8_t cur;            /* the state being consumed */
    uint32_t cur_beat;
    uint64_t half_units;    /* half an SPI_CLK cycle, 2^-32 ns */
    int64_t t0;             /* when half cycle 0 began */
    uint64_t h;             /* half cycles from t0 to the next step */
    /* The generator, ahead of the consumer. */
    uint8_t gen;
    uint32_t gen_beat;
    uint8_t gen_byte;
    uint32_t gen_n;
    ESP32P4SpiBeat ring[ESP32P4_SPI_RING];
    uint32_t ring_head;
    uint32_t ring_num;
    /* The SSI byte stream: bits generated, sent, and the answers. */
    uint32_t ssi_gen_bits;
    uint32_t ssi_sent_bits;
    uint32_t ssi_acc;
    uint8_t ssi_resp[4];
    /* Received data: the byte being assembled and its index. */
    uint8_t din_byte;
    uint32_t din_n;
    bool rx_had_data;
    bool tx_err_raised;
    /* The input sources, as last resolved, and when. */
    ESP32P4SpiLaneSrc src[ESP32P4_SPI_LANES];
    bool src_valid;
    uint8_t ssi_out_lanes;
    bool ssi_selected;
    /* Configurable segmented transfer: the CONF state's progress. */
    uint32_t conf_bitmap;
    uint8_t conf_word;
    uint8_t conf_byte;
    uint8_t conf_bit;
    uint32_t conf_val;

    /* The slave. */
    bool s_cs;              /* CS0 is active */
    uint8_t s_state;
    uint32_t s_bits;        /* bits of the current state */
    uint32_t s_data_bits;   /* data bits of the transaction */
    uint8_t s_cmd;
    uint8_t s_addr;
    uint8_t s_rx;           /* the byte being received */
    uint8_t s_tx;           /* the byte being sent */
    uint32_t s_n;           /* the data byte index */
    uint8_t s_lanes;        /* data lanes of the current state */
    uint8_t s_addr_lanes;
    uint8_t s_data_lanes;
    uint8_t s_kind;         /* the half-duplex command's kind */
    bool s_err;
};

#endif
