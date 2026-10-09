/*
 * DesignWare I2C Module.
 *
 * Copyright 2021 Google LLC
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Synopsys DW_apb_i2c: an I2C controller (master) and target (slave) with
 * 16-entry transmit and receive FIFOs, DMA handshaking and a combined
 * interrupt. Controller transfers take the time the SCL count registers
 * give them at the frequency of the "clk" input (ic_clk).
 *
 * The device's QEMU I2C bus ("bus-name", default "i2c-bus") is the bus
 * its SCL and SDA are wired to. Its target logic is an I2C slave on that
 * bus (child "target"), answering to IC_SAR while target mode is enabled.
 * Further TYPE_DESIGNWARE_I2C_TARGET devices with their "controller" link
 * set put the same target logic on other buses.
 *
 * Inputs and outputs:
 *
 *  - Sysbus IRQ 0: the combined interrupt (ic_intr).
 *  - Named GPIO outputs "dma-tx-req" and "dma-rx-req": the DMA handshake
 *    requests, high while the FIFO level meets IC_DMA_TDLR/IC_DMA_RDLR
 *    with IC_DMA_CR enabling them.
 *  - Named GPIO inputs "scl-in" and "sda-in": the levels of the SCL and
 *    SDA wires (1 when left unconnected). A controller starts a transfer
 *    only on an idle bus, with both high, and waits while another device
 *    holds SCL low.
 *  - Named GPIO output "scl-oe": high while the device holds SCL low
 *    (ic_clk_oe), as a controller waiting for a command or a target
 *    waiting for software.
 *  - Clock input "clk": ic_clk.
 */
#ifndef DESIGNWARE_I2C_H
#define DESIGNWARE_I2C_H

#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/core/clock.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define DESIGNWARE_I2C_RX_FIFO_SIZE 16
#define DESIGNWARE_I2C_TX_FIFO_SIZE 16

#define DESIGNWARE_I2C_DMA_TX_REQ "dma-tx-req"
#define DESIGNWARE_I2C_DMA_RX_REQ "dma-rx-req"
#define DESIGNWARE_I2C_SCL_IN "scl-in"
#define DESIGNWARE_I2C_SDA_IN "sda-in"
#define DESIGNWARE_I2C_SCL_OE "scl-oe"

#define TYPE_DESIGNWARE_I2C "designware-i2c"
OBJECT_DECLARE_TYPE(DesignWareI2CState, DesignWareI2CClass, DESIGNWARE_I2C)

#define TYPE_DESIGNWARE_I2C_TARGET "designware-i2c-target"
OBJECT_DECLARE_SIMPLE_TYPE(DesignWareI2CTarget, DESIGNWARE_I2C_TARGET)

/* The target logic of a controller, as a slave on one I2C bus. */
struct DesignWareI2CTarget {
    I2CSlave parent_obj;

    DesignWareI2CState *controller;
};

/*
 * The configuration a DW_apb_i2c instance was synthesised with: register
 * reset values and the read-only parameter registers. Subclasses for a
 * particular chip set these in their class_init.
 */
struct DesignWareI2CClass {
    SysBusDeviceClass parent_class;

    uint32_t con_reset;
    uint32_t tar_reset;
    /* Writable IC_TAR bits (IC_10BITADDR_MASTER at bit 12 or not). */
    uint32_t tar_mask;
    uint32_t sar_reset;
    uint32_t ss_scl_hcnt_reset;
    uint32_t ss_scl_lcnt_reset;
    uint32_t fs_scl_hcnt_reset;
    uint32_t fs_scl_lcnt_reset;
    uint32_t fs_spklen_reset;
    /* IC_MAX_SPEED_MODE: 2 fast mode (plus), 3 high speed. */
    uint32_t max_speed;
    uint32_t comp_param_1;
    uint32_t comp_version;
    bool has_smbus_intr_mask;
};

/* A step of a controller transfer on the wire. */
typedef enum DesignWareI2CStep {
    DW_I2C_STEP_STOP,
    DW_I2C_STEP_SBYTE,
    DW_I2C_STEP_ADDR,
    DW_I2C_STEP_ADDR2,
    DW_I2C_STEP_ADDR_R,
    DW_I2C_STEP_DATA,
} DesignWareI2CStep;

#define DW_I2C_MAX_STEPS 8

struct DesignWareI2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    char *bus_name;
    I2CBus *bus;
    DesignWareI2CTarget target;
    qemu_irq irq;
    qemu_irq dma_tx_req;
    qemu_irq dma_rx_req;
    qemu_irq scl_oe;
    Clock *clk;
    QEMUTimer *timer;

    /* Registers. RAW_INTR_STAT's TX_EMPTY and RX_FULL are derived. */
    uint32_t con;
    uint32_t tar;
    uint32_t sar;
    uint32_t ss_scl_hcnt;
    uint32_t ss_scl_lcnt;
    uint32_t fs_scl_hcnt;
    uint32_t fs_scl_lcnt;
    uint32_t intr_mask;
    uint32_t raw_intr;
    uint32_t rx_tl;
    uint32_t tx_tl;
    uint32_t enable;
    uint32_t sda_hold;
    uint32_t tx_abrt_source;
    uint32_t slv_data_nack_only;
    uint32_t dma_cr;
    uint32_t dma_tdlr;
    uint32_t dma_rdlr;
    uint32_t sda_setup;
    uint32_t ack_general_call;
    /* ENABLE_STATUS SLV_DISABLED_WHILE_BUSY and SLV_RX_DATA_LOST. */
    uint32_t enable_status_slv;
    uint32_t fs_spklen;
    uint32_t smbus_intr_mask;

    /* TX FIFO: IC_DATA_CMD writes; RX FIFO: data, FIRST_DATA_BYTE. */
    uint32_t tx_fifo[DESIGNWARE_I2C_TX_FIFO_SIZE];
    uint8_t tx_head;
    uint8_t tx_num;
    uint32_t rx_fifo[DESIGNWARE_I2C_RX_FIFO_SIZE];
    uint8_t rx_head;
    uint8_t rx_num;

    /* The SCL and SDA wires and whether this device holds SCL low. */
    bool scl_in;
    bool sda_in;
    bool scl_hold;
    /* ic_en, the enable status, as last seen by dw_i2c_update(). */
    bool ic_en;

    /*
     * Controller. A command popped from the TX FIFO becomes a plan of
     * steps on the wire, each completed by the timer.
     */
    bool mst_owned;
    bool mst_xfer_open;
    bool mst_read;
    bool mst_read_done;
    bool mst_first_data;
    bool mst_shifting;
    bool mst_stepping;
    bool mst_polling;
    bool mst_user_abort;
    bool mst_rx_stall;
    bool mst_kicking;
    bool mst_kick_again;
    uint8_t mst_addr;
    uint32_t mst_cmd;
    uint8_t plan[DW_I2C_MAX_STEPS];
    uint8_t plan_len;
    uint8_t plan_pos;
    /* Whether the planned command has left the TX FIFO. */
    bool plan_popped;

    /* Target. */
    bool slv_active;
    bool slv_read;
    bool slv_gc;
    bool slv_gc_match;
    bool slv_first;
    bool slv_data;
    bool slv_hold;
    /* 10-bit addressing: 0 none, 1 awaiting the second byte, 2 matched. */
    uint8_t slv_10bit;
    I2CBus *slv_bus;
};

#endif /* DESIGNWARE_I2C_H */
