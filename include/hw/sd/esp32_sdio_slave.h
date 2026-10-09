/*
 * ESP32 SDIO slave: HINF, SLC and SLCHOST, and the SDIO card they present
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_ESP32_SDIO_SLAVE_H
#define HW_SD_ESP32_SDIO_SLAVE_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/sd/sd.h"
#include "system/memory.h"

#define TYPE_ESP32_SDIO_SLAVE "esp32.sdio_slave"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32SdioSlaveState, ESP32_SDIO_SLAVE)

/* The card the slave presents on an SD bus, a child of the slave */
#define TYPE_ESP32_SDIO_CARD "esp32-sdio-card"

/* The three register blocks, in the order of the slave's MMIO regions */
enum {
    ESP32_SDIO_MMIO_SLC,
    ESP32_SDIO_MMIO_SLCHOST,
    ESP32_SDIO_MMIO_HINF,
};

/* The interrupt sources, in the order of the slave's IRQ outputs */
enum {
    ESP32_SDIO_IRQ_SLC0,
    ESP32_SDIO_IRQ_SLC1,
    ESP32_SDIO_IRQ_COUNT,
};

#define ESP32_SDIO_BLOCK_SIZE   0x1000
#define ESP32_SDIO_SLC_WORDS    (0x200 / 4)
#define ESP32_SDIO_HOST_WORDS   (0x200 / 4)
#define ESP32_SDIO_HINF_WORDS   (0x100 / 4)

/*
 * Depth of the sending DMA's FIFO, which the DMA fills from the
 * descriptor chain ahead of the host's reads. The TRM does not give it;
 * it only bounds how far the DMA runs ahead of the host.
 */
#define ESP32_SDIO_FIFO_DEPTH   32

/* SLC0's sending (slave to host, the SLC's "RX") link */
typedef struct Esp32SdioSendLink {
    bool active;
    bool parked;
    bool have_desc;
    bool have_last;
    /* The next descriptor loaded starts a packet */
    bool new_packet;
    /* The FIFO holds the end of a packet, at its tail */
    bool eof_in_fifo;
    uint32_t desc_addr;
    uint32_t dw0;
    uint32_t buf;
    uint32_t next;
    /* Bytes of the descriptor's buffer moved into the FIFO */
    uint32_t pos;
    uint32_t last_addr;
    uint32_t eof_desc;
    uint32_t eof_buf;
    uint8_t fifo[ESP32_SDIO_FIFO_DEPTH];
    uint32_t head;
    uint32_t num;
} Esp32SdioSendLink;

/* SLC0's receiving (host to slave, the SLC's "TX") link */
typedef struct Esp32SdioRecvLink {
    bool active;
    bool parked;
    bool have_desc;
    bool have_last;
    uint32_t desc_addr;
    uint32_t dw0;
    uint32_t buf;
    uint32_t next;
    /* Bytes stored into the descriptor's buffer */
    uint32_t len;
    uint32_t last_addr;
} Esp32SdioRecvLink;

struct Esp32SdioSlaveState {
    SysBusDevice parent_obj;

    MemoryRegion slc_mr;
    MemoryRegion host_mr;
    MemoryRegion hinf_mr;
    qemu_irq irq[ESP32_SDIO_IRQ_COUNT];
    Clock *apb_clk;
    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    /* The card, once plugged into a host's bus */
    DeviceState *card;

    /* Register files; behavioural registers keep their state elsewhere */
    uint32_t slc[ESP32_SDIO_SLC_WORDS];
    uint32_t host[ESP32_SDIO_HOST_WORDS];
    uint32_t hinf[ESP32_SDIO_HINF_WORDS];

    uint32_t slc0_token[2];
    uint32_t slc1_token[2];
    uint32_t slc0_len;
    /* SLCHOST_PKT_LEN as the host last read it */
    uint32_t pkt_len_latch;

    Esp32SdioSendLink send;
    Esp32SdioRecvLink recv;
    bool undoc_logged;

    /* The card's SDIO protocol state and the host-written CCCR and FBRs */
    uint8_t card_state;
    uint16_t rca;
    uint8_t ioe;
    uint8_t ien;
    uint8_t bus_if;
    uint8_t cap_wr;
    uint8_t speed_wr;
    uint16_t blksize[3];
    /* The CMD53 whose data is on the bus */
    bool xfer_active;
    bool xfer_write;
    bool xfer_inc;
    bool xfer_endless;
    uint8_t xfer_fn;
    uint32_t xfer_addr;
    uint32_t xfer_left;
    /* The card's interrupt (DAT1) as the host last saw it */
    bool card_irq;
};

/* Plug the slave's card into an SD host's bus. */
void esp32_sdio_slave_attach(Esp32SdioSlaveState *s, SDBus *bus);

#endif
