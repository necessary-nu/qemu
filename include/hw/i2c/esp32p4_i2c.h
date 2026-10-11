/*
 * ESP32-P4 I2C controllers: I2C0, I2C1 and the LP I2C
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_ESP32P4_I2C_H
#define HW_I2C_ESP32P4_I2C_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "qom/object.h"

/*
 * The HP controllers (I2C0, I2C1), masters or slaves, and the LP one, a
 * master only with half the RAM: the same device, two classes.
 */
#define TYPE_ESP32P4_I2C "esp32p4-i2c"
#define TYPE_ESP32P4_LP_I2C "esp32p4-lp-i2c"
OBJECT_DECLARE_TYPE(ESP32P4I2CState, ESP32P4I2CClass, ESP32P4_I2C)

/* The registers and the TX and RX RAM; the block decodes 4 KiB. */
#define ESP32P4_I2C_REGS_SIZE   0x200
#define ESP32P4_I2C_BLOCK_SIZE  0x1000
#define ESP32P4_I2C_NREGS       (0x100 / 4)
#define ESP32P4_I2C_RAM_MAX     32
#define ESP32P4_I2C_CMDS        8

/*
 * Connections
 * ===========
 *
 * The lines are the controller's GPIO matrix signals (I2CEXTn_SCL and
 * _SDA, or the LP matrix's LP_I2C_SCL and _SDA), each an output with its
 * enable and an input:
 *
 *   "scl-out", "scl-oe", "sda-out", "sda-oe" (outputs): open drain, as
 *     I2C_SCL_FORCE_OUT and I2C_SDA_FORCE_OUT clear have them, drives the
 *     signal low with its enable to pull the line low and disables the
 *     output to release it; with FORCE_OUT set the enable stays on and
 *     the signal carries the level, which the pad's open-drain
 *     configuration turns into the same thing.
 *   "scl-in", "sda-in" (inputs): the lines as the matrix delivers them.
 *
 * The controller works at the level of the lines: as a master it times
 * every SCL edge from its timing registers at its function clock, waits
 * for SCL to rise when something else holds it low, samples SDA and
 * loses arbitration to another driver; as a slave it follows the edges a
 * master puts on its inputs and drives SDA and stretches SCL in reply.
 * Two controllers routed to the same pads talk to each other through
 * them.
 *
 * Its QEMU I2C bus ("bus-name") is the board's wires on the pads its
 * SCL and SDA are routed to, with whatever i2c-slave devices are put on
 * it. As a master it addresses them when both of its lines, pulled low,
 * read back low: its outputs reach pads whose inputs come back to it.
 * A device's acknowledge and data bits read as the level on SDA, ANDed
 * with what the pad shows; the devices answer a byte at a time.
 *
 * Clock input "sclk": I2C_SCLK, the function clock from HP_SYS_CLKRST
 * after its divider, or for the LP I2C the source LPPERI selects, which
 * LP_I2C_CLK_CONF divides further. While it is stopped the controller
 * does nothing.
 *
 * Sysbus IRQ 0: I2Cn_INTR (or LP_I2C_INTR), INT_STATUS != 0.
 */
#define ESP32P4_I2C_SCL_OUT "scl-out"
#define ESP32P4_I2C_SCL_OE  "scl-oe"
#define ESP32P4_I2C_SDA_OUT "sda-out"
#define ESP32P4_I2C_SDA_OE  "sda-oe"
#define ESP32P4_I2C_SCL_IN  "scl-in"
#define ESP32P4_I2C_SDA_IN  "sda-in"

struct ESP32P4I2CClass {
    SysBusDeviceClass parent_class;

    /* The LP controller: master only, 16-byte RAM, LP_I2C_CLK_CONF. */
    bool lp;
};

/* An input line and its filter (I2C_SCL_FILTER_*, I2C_SDA_FILTER_*). */
typedef struct ESP32P4I2CLine {
    /* The level at the input and the level past the filter. */
    bool raw;
    bool filt;
    /* When the filtered level last changed. */
    int64_t edge_ns;
    /* A change waiting to pass the filter: when it does. */
    int64_t pend_ns;
} ESP32P4I2CLine;

struct ESP32P4I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq scl_out;
    qemu_irq scl_oe;
    qemu_irq sda_out;
    qemu_irq sda_oe;
    Clock *sclk;
    I2CBus *bus;
    char *bus_name;

    QEMUTimer fsm_timer;
    QEMUTimer slv_timer;
    QEMUTimer to_timer;
    QEMUTimer flt_timer[2];

    /*
     * Registers as software sees them, and the copies in the I2C_SCLK
     * domain that I2C_CONF_UPGATE updates (TRM table 47.4-1), which the
     * controller works from.
     */
    uint32_t regs[ESP32P4_I2C_NREGS];
    uint32_t act[ESP32P4_I2C_NREGS];
    bool upgate_pending;

    /* TX and RX RAM and their FIFO pointers. */
    uint8_t txram[ESP32P4_I2C_RAM_MAX];
    uint8_t rxram[ESP32P4_I2C_RAM_MAX];
    uint8_t tx_rd;
    uint8_t tx_wr;
    uint8_t tx_cnt;
    uint8_t rx_rd;
    uint8_t rx_wr;
    uint8_t rx_cnt;

    /* Interrupts raised, and the status they make. */
    uint32_t int_raw;
    bool irq_level;

    /* SR fields the controller keeps. */
    bool resp_rec;
    bool slave_rw;
    bool arb_lost;
    bool slave_addressed;
    uint8_t stretch_cause;
    uint8_t main_state;
    uint8_t scl_state;
    uint8_t slave_rw_point;

    /* The lines: inputs, and what the controller drives (1 releases). */
    ESP32P4I2CLine line[2];
    bool scl_drv;
    bool sda_drv;
    /* A START seen on the lines and no STOP since. */
    bool bus_active;

    /*
     * The master's next step: due at step_mark, timed at step_period,
     * the I2C_SCLK period when it was scheduled, or with the clock
     * stopped (step_period 0) step_left cycles after it restarts; a clock
     * change rescales what is left. step_ns is the time of the step
     * being run.
     */
    uint32_t step;
    uint64_t step_left;
    int64_t step_mark;
    uint64_t step_period;
    int64_t step_ns;

    /* Master. */
    bool mst_busy;
    bool mst_suspended;
    bool mst_in_txn;
    uint8_t cmd_idx;
    uint16_t byte_left;
    uint8_t bit;
    uint8_t cur_byte;
    bool reading;
    bool bit_out;
    bool mst_ack;
    bool addr_next;
    bool nack_stop;
    bool sample_at_fall;
    bool chk_sda;
    bool chk_scl;
    bool connected_sda;
    bool connected;
    bool dev_active;
    bool dev_read;
    bool dev_nack;
    uint8_t dev_addr;
    uint8_t dev_byte;
    int64_t t_rel;
    int64_t t_fall_due;
    uint8_t rst_pulses;
    bool start_pending;

    /* Slave. */
    uint8_t slv_state;
    uint8_t slv_bits;
    uint8_t slv_byte;
    uint8_t slv_next;
    bool slv_ack;
    bool slv_matched;
    bool slv_10bit;
    bool slv_first_data;
    bool slv_armed;
    bool slv_master_nack;
    /* A stretch to begin at the next SCL fall, and one in progress. */
    uint8_t slv_stretch_req;
    bool slv_stretch;
    bool slv_stretch_tx;
    /* Line changes the slave has scheduled: when, and to what. */
    int64_t slv_sda_due;
    bool slv_sda_lvl;
    int64_t slv_scl_due;
    bool slv_scl_lvl;

    /*
     * The timeouts: whether the SCL timeout's condition holds and since
     * when, when the master began waiting on the lines, and the
     * deadlines armed (INT64_MAX for none).
     */
    bool to_cond;
    int64_t to_since;
    int64_t wait_since;
    int64_t to_scl_due;
    int64_t to_st_due;
    int64_t to_main_due;
};

#endif
