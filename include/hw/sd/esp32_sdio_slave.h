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
#include "hw/gpio/esp32_gpio.h"
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
 * The slave's SD lines, CLK, CMD and DAT0 to DAT3, as bits of
 * Esp32SdioLines, and its pads: function 4 of slot 1's IO_MUX pads (TRM
 * table 6.10-1; ESP-IDF's sdio_slave_pins.h), the pads a board wires the
 * slave's host to. Slot 0's, function 0 of GPIO6 to GPIO11, are the
 * ESP32-WROOM-32E's flash lines.
 */
#define ESP32_SDIO_LINE_CLK     0x001
#define ESP32_SDIO_LINE_CMD     0x002
#define ESP32_SDIO_LINE_DAT(n)  (0x004 << (n))
#define ESP32_SDIO_LINES        6
#define ESP32_SDIO_FUNC         4
#define ESP32_SDIO_SLOT1_PADS   { 14, 15, 2, 4, 12, 13 }

/*
 * The slave's outputs on its slot 1 pads, a line per SD line (CLK's is
 * never driven), for the GPIO block's ESP32_GPIO_IOMUX_OUT and
 * ESP32_GPIO_IOMUX_OE: the levels its lines hold between transfers.
 */
#define ESP32_SDIO_PAD_OUT      "pad-out"
#define ESP32_SDIO_PAD_OE       "pad-oe"

/*
 * The lines between the slave and the host the board wires to slot 1's
 * pads (esp32_sdio_slave_lines()): to_slave, where the host's output
 * reaches the slave's input, with what the slave's input reads where it
 * does not (slave_level); to_host, where the slave's output reaches the
 * pad, which is the host's input; host_level, the pads' levels.
 */
typedef struct Esp32SdioLines {
    uint16_t to_slave;
    uint16_t to_host;
    uint16_t slave_level;
    uint16_t host_level;
} Esp32SdioLines;

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
    /* The GPIO matrix and IO_MUX between the slave and its pads */
    Esp32GpioState *gpio;
    qemu_irq pad_out[ESP32_SDIO_LINES];
    qemu_irq pad_oe[ESP32_SDIO_LINES];
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
    bool xfer_block;
    /* A block of this write failed its CRC check: the rest is dropped */
    bool xfer_discard;
    uint8_t xfer_fn;
    uint32_t xfer_addr;
    uint32_t xfer_left;
    /* The card's interrupt (DAT1) as the host last saw it */
    bool card_irq;
    /* SLC_SDIO_CRC_ST0's per-line counts of write blocks with bad CRCs */
    uint8_t dat_crc_err[4];
};

/* Plug the slave's card into an SD host's bus. */
void esp32_sdio_slave_attach(Esp32SdioSlaveState *s, SDBus *bus);

/* The lines between the slave and its host on slot 1's pads, as they stand */
void esp32_sdio_slave_lines(Esp32SdioSlaveState *s, Esp32SdioLines *l);

#endif
