/*
 * ESP32 Ethernet MAC (EMAC), a Synopsys DesignWare GMAC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ESP32_EMAC_H
#define HW_NET_ESP32_EMAC_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/net/ip101_phy.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "system/memory.h"

#define TYPE_ESP32_EMAC "esp32.emac"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32EmacState, ESP32_EMAC)

/* DMA registers at 0x0000, the ESP32 clock wrapper at 0x0800, MAC 0x1000 */
#define ESP32_EMAC_REG_SIZE     0x2000
#define ESP32_EMAC_ADDR_COUNT   8
#define ESP32_EMAC_RWUFFR_COUNT 8
/* The MTL receive FIFO */
#define ESP32_EMAC_RX_FIFO_SIZE 2048
#define ESP32_EMAC_RX_FIFO_MAX_FRAMES (ESP32_EMAC_RX_FIFO_SIZE / 64)
/* Longest frame the MAC handles with its watchdog and jabber disabled */
#define ESP32_EMAC_MAX_FRAME    16383
/* Room for the gathered frame plus an inserted address, padding and FCS */
#define ESP32_EMAC_TX_BUF_SIZE  (ESP32_EMAC_MAX_FRAME + 64)

struct Esp32EmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *apb_clk;
    NICState *nic;
    NICConf conf;
    IP101PhyState *phy;

    MemoryRegion *dma_mr;
    AddressSpace dma_as;

    /* DMA */
    uint32_t bus_mode;
    uint32_t rx_base;
    uint32_t tx_base;
    uint32_t status;
    uint32_t op_mode;
    uint32_t int_en;
    uint32_t missed_frames;
    uint32_t overflow_frames;
    bool missed_overflow;
    bool fifo_overflow;
    uint32_t rx_wdt;
    uint32_t tx_desc;
    uint32_t rx_desc;
    uint32_t tx_buf;
    uint32_t rx_buf;
    uint8_t tx_state;
    uint8_t rx_state;
    /* An engine that took a bus error stays off the bus until reset */
    bool tx_fatal;
    bool rx_fatal;

    /* MAC */
    uint32_t config;
    uint32_t frame_filter;
    uint32_t mii_addr;
    uint32_t mii_data;
    uint32_t flow_ctrl;
    uint32_t rwuffr[ESP32_EMAC_RWUFFR_COUNT];
    uint32_t rwuffr_ptr;
    uint32_t pmt_csr;
    uint32_t lpi_csr;
    uint32_t lpi_timers;
    uint32_t int_mask;
    uint32_t addr_high[ESP32_EMAC_ADDR_COUNT];
    uint32_t addr_low[ESP32_EMAC_ADDR_COUNT];
    uint32_t wdog_to;

    /* Clock wrapper */
    uint32_t ex_clkout_conf;
    uint32_t ex_oscclk_conf;
    uint32_t ex_clk_ctrl;
    uint32_t ex_phyinf_conf;
    uint32_t ex_pd_sel;
    uint32_t ex_date;

    /* A software reset waits for the MII clocks */
    bool sw_reset_pending;

    /* MDIO: a management frame on the wire */
    QEMUTimer mdio_timer;

    /*
     * Transmit: the frame the DMA has gathered so far from its
     * descriptors, the first descriptor's control words, and the
     * descriptor holding its last segment. A complete frame waits in the
     * MTL FIFO (tx_ready) until the MAC can send it; then, rewritten in
     * place as the wire's tx_wire_len bytes, it occupies the wire
     * (tx_timer) until it reaches the receiver and the DMA closes its
     * descriptor.
     */
    uint8_t tx_frame[ESP32_EMAC_TX_BUF_SIZE];
    uint32_t tx_len;
    uint32_t tx_wire_len;
    bool tx_gathering;
    bool tx_ready;
    bool tx_on_wire;
    bool tx_jabber;
    uint32_t tx_tdes0;
    uint32_t tx_tdes1;
    uint32_t tx_last_desc;
    uint32_t tx_status;
    QEMUTimer tx_timer;
    /* A pause frame the MAC owes the wire, and the one on it */
    bool pause_pending;
    bool pause_on_wire;
    /* The transmitter is paused by a received pause frame */
    QEMUTimer pause_timer;
    bool paused;

    /* Receive: frames in the MTL FIFO with their status words */
    uint8_t rx_fifo[ESP32_EMAC_RX_FIFO_SIZE];
    uint32_t rx_fifo_used;
    uint32_t rx_fifo_count;
    uint16_t rx_fifo_len[ESP32_EMAC_RX_FIFO_MAX_FRAMES];
    uint32_t rx_fifo_rdes0[ESP32_EMAC_RX_FIFO_MAX_FRAMES];
    uint32_t rx_fifo_rdes4[ESP32_EMAC_RX_FIFO_MAX_FRAMES];
    /* RIWT: the receive interrupt held back by RDES1.DIC */
    QEMUTimer rx_wdt_timer;
};

#endif
