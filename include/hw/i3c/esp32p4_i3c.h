/*
 * ESP32-P4 I3C master and slave controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I3C_ESP32P4_I3C_H
#define HW_I3C_ESP32P4_I3C_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/i3c/i3c.h"
#include "qemu/fifo32.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_I3C_MST "esp32p4-i3c-mst"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4I3CMstState, ESP32P4_I3C_MST)

#define TYPE_ESP32P4_I3C_SLV "esp32p4-i3c-slv"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4I3CSlvState, ESP32P4_I3C_SLV)

#define TYPE_ESP32P4_I3C_SLV_TARGET "esp32p4-i3c-slv-target"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4I3CSlvTarget, ESP32P4_I3C_SLV_TARGET)

/* The master's I3C bus, which takes the targets' in-band interrupts. */
#define TYPE_ESP32P4_I3C_BUS "esp32p4-i3c-bus"

#define ESP32P4_I3C_BLOCK_SIZE  0x1000
#define ESP32P4_I3C_MST_NREGS   (0xc0 / 4)
#define ESP32P4_I3C_DEVS        12
#define ESP32P4_I3C_CMD_WORDS   16
#define ESP32P4_I3C_RESP_DEPTH  8
#define ESP32P4_I3C_DATA_WORDS  32
#define ESP32P4_I3C_IBI_STATUS_DEPTH 8
#define ESP32P4_I3C_IBI_DATA_WORDS 8
#define ESP32P4_I3C_SLV_FIFO    16

/*
 * Connections
 * ===========
 *
 * Both controllers' lines are GPIO matrix signals (I3C_MST_SCL/SDA,
 * I3C_SLV_SCL/SDA), each an output with its enable ("scl-out", "scl-oe",
 * "sda-out", "sda-oe") and an input ("scl-in", "sda-in"); the master
 * also drives I3C_MST_SCL_PULLUP_EN and _SDA_PULLUP_EN ("scl-pullup",
 * "sda-pullup").
 *
 * The master's QEMU I3C bus ("bus-name", default "i3c") is the board's
 * wires on the pads its SCL and SDA are routed to, with i3c-target
 * devices on it and i2c-slave devices on its legacy I2C bus
 * ("<bus-name>-legacy-i2c"). The master addresses them when both of its
 * lines, pulled low for a START, read back low through the pads. It
 * works a byte at a time: START, repeated START and STOP appear on the
 * lines, and between them SCL and SDA are held low while each header and
 * byte takes the time of its bits at the timing registers' open-drain,
 * push-pull or I2C counts.
 *
 * The slave controller's target logic is an I3C target on the master's
 * bus ("i3c-bus" link); it takes part in a transfer only once its own
 * inputs have seen the START, so only when its pads are wired to the
 * master's.
 *
 * With DMA_TX_EN and DMA_RX_EN the master's TX and RX data buffers are
 * GDMA-AHB's ("gdma" link): it fills the TX buffer from the channel that
 * selects the I3C master, a word as the channel has four bytes for it or
 * the frame ends, and empties the RX buffer into the channel that
 * selects it, ending the frame (suc_eof) where a read command ends.
 *
 * Clock inputs: the master's "clk", I3C_MST_CLK, which times the bus;
 * the slave's "clk", its APB clock. While a clock is stopped, its
 * controller does nothing.
 *
 * Sysbus IRQ 0: I3C_MST_INTR or I3C_SLV_INTR.
 */
#define ESP32P4_I3C_SCL_OUT     "scl-out"
#define ESP32P4_I3C_SCL_OE      "scl-oe"
#define ESP32P4_I3C_SDA_OUT     "sda-out"
#define ESP32P4_I3C_SDA_OE      "sda-oe"
#define ESP32P4_I3C_SCL_IN      "scl-in"
#define ESP32P4_I3C_SDA_IN      "sda-in"
#define ESP32P4_I3C_SCL_PULLUP  "scl-pullup"
#define ESP32P4_I3C_SDA_PULLUP  "sda-pullup"

/* An IBI the master has taken, waiting to be reported. */
typedef struct ESP32P4I3CIbi {
    bool valid;
    bool acked;
    bool notify;
    uint8_t id;
    uint8_t len;
    uint8_t data[ESP32P4_I3C_IBI_DATA_WORDS * 4];
    /* A rejected IBI's target, to send DISEC; 0 for none. */
    uint8_t disec_addr;
} ESP32P4I3CIbi;

struct ESP32P4I3CMstState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq scl_out;
    qemu_irq scl_oe;
    qemu_irq sda_out;
    qemu_irq sda_oe;
    qemu_irq scl_pullup;
    qemu_irq sda_pullup;
    Clock *clk;
    I3CBus *bus;
    char *bus_name;
    ESP32P4GdmaState *gdma;
    QEMUTimer timer;

    uint32_t regs[ESP32P4_I3C_MST_NREGS];
    uint32_t dat[ESP32P4_I3C_DEVS];
    uint32_t dct[ESP32P4_I3C_DEVS][4];
    uint32_t int_raw;
    bool irq_level;

    Fifo32 cmd;
    Fifo32 resp;
    Fifo32 tx;
    Fifo32 rx;
    Fifo32 ibi_status;
    Fifo32 ibi_data;
    uint8_t present_dct;
    uint8_t present_dat;

    /* Bytes received but not yet a whole word of the RX buffer. */
    uint32_t rx_word;
    uint8_t rx_bytes;
    /* What is left of the TX word being sent. */
    uint32_t tx_word;
    uint8_t tx_bytes;
    /*
     * DMA: the TX word GDMA is filling; a read in progress, which keeps
     * its last word back for the frame's end to go with it; and a read
     * whose end GDMA has still to be told of.
     */
    uint32_t dma_tx_word;
    uint8_t dma_tx_bytes;
    bool dma_rx_hold;
    bool dma_rx_eof;

    /* The lines: the levels read back, and what the master drives. */
    bool scl_in;
    bool sda_in;
    bool scl_drv;
    bool sda_drv;
    bool connected;

    /*
     * The command being run and where it has got to: its words, the
     * phase the timer ends next, the bytes left and done, its error, the
     * device it addresses, the address retries made and what follows a
     * repeated START. The phases end phase_cyc cycles of I3C_MST_CLK, at
     * its exact period phase_period, after phase_origin, the current one
     * at phase_due; phase_left holds the cycles of I3C_MST_CLK a phase
     * has left while the clock is stopped.
     */
    bool busy;
    bool in_txn;
    uint32_t cmd_l;
    uint32_t cmd_h;
    uint32_t phase;
    int64_t phase_due;
    ESP32P4ClkPeriod phase_period;
    uint64_t phase_left;
    int64_t phase_origin;
    uint64_t phase_cyc;
    uint32_t left;
    uint32_t done;
    uint8_t err;
    uint8_t dev;
    uint8_t retries;
    uint8_t rs_next;
    uint16_t byte_cnt;
    int64_t free_at;
    bool dev_active;
    bool dev_i2c;

    /* IBIs: the one being taken from a target, and those to report. */
    ESP32P4I3CIbi ibi_in;
    ESP32P4I3CIbi ibi_pend;
};

struct ESP32P4I3CSlvTarget {
    I3CTarget parent_obj;

    ESP32P4I3CSlvState *slv;
};

struct ESP32P4I3CSlvState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq scl_out;
    qemu_irq scl_oe;
    qemu_irq sda_out;
    qemu_irq sda_oe;
    Clock *clk;
    I3CBus *i3c_bus;
    ESP32P4I3CSlvTarget target;

    uint32_t config;
    uint32_t status;
    uint32_t ctrl;
    uint32_t intset;
    uint32_t datactrl;
    uint32_t partno;
    uint32_t idext;
    uint32_t vendorid;
    bool irq_level;

    /* The FIFOs: bytes, and for TX whether each ends a message. */
    Fifo8 tx;
    Fifo8 tx_end;
    Fifo8 rx;
    bool errwarn;

    bool scl_in;
    bool sda_in;
    bool sda_drv;
    bool ibi_enabled;
    bool ibi_waiting;
    uint8_t last_da;
};

#endif
