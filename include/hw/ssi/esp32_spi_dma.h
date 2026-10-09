/*
 * ESP32 SPI DMA: the linked-list DMA engine of SPI1, SPI2 and SPI3
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/clock.h"

typedef struct Esp32DportState Esp32DportState;

#define TYPE_ESP32_SPI_DMA "ssi.esp32.spi_dma"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32SpiDmaState, ESP32_SPI_DMA)

/* The DMA registers sit at this offset in their SPI controller's block. */
#define ESP32_SPI_DMA_REG_OFFSET    0x100
#define ESP32_SPI_DMA_REG_SIZE      0x50

/*
 * One direction of the engine: the outlink (memory to MOSI) or the inlink
 * (MISO to memory), walking a chain of lldesc_t descriptors.
 */
typedef struct Esp32SpiDmaLink {
    /* DMA_STATUS TX_EN / RX_EN: the link is started and has a descriptor */
    bool active;
    /* The descriptor in use, its first word, its buffer and next pointer */
    uint32_t dscr;
    uint32_t dw0;
    uint32_t buf;
    uint32_t next;
    /* Bytes of the current descriptor's buffer already moved */
    uint32_t pos;
} Esp32SpiDmaLink;

struct Esp32SpiDmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    Esp32DportState *dport;
    /* The SPI controller this engine belongs to: 1, 2 or 3 */
    uint8_t host;

    uint32_t conf;
    uint32_t out_link;
    uint32_t in_link;
    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t in_err_eof_des_addr;
    uint32_t in_suc_eof_des_addr;
    uint32_t inlink_dscr;
    uint32_t inlink_dscr_bf0;
    uint32_t inlink_dscr_bf1;
    uint32_t out_eof_bfr_des_addr;
    uint32_t out_eof_des_addr;
    uint32_t outlink_dscr;
    uint32_t outlink_dscr_bf0;
    uint32_t outlink_dscr_bf1;

    Esp32SpiDmaLink out;
    Esp32SpiDmaLink in;
};

/* Register offsets, relative to ESP32_SPI_DMA_REG_OFFSET */
REG32(SPI_DMA_CONF, 0x00)
    FIELD(SPI_DMA_CONF, IN_RST, 2, 1)
    FIELD(SPI_DMA_CONF, OUT_RST, 3, 1)
    FIELD(SPI_DMA_CONF, AHBM_FIFO_RST, 4, 1)
    FIELD(SPI_DMA_CONF, AHBM_RST, 5, 1)
    FIELD(SPI_DMA_CONF, IN_LOOP_TEST, 6, 1)
    FIELD(SPI_DMA_CONF, OUT_LOOP_TEST, 7, 1)
    FIELD(SPI_DMA_CONF, OUT_AUTO_WRBACK, 8, 1)
    FIELD(SPI_DMA_CONF, OUT_EOF_MODE, 9, 1)
    FIELD(SPI_DMA_CONF, OUTDSCR_BURST_EN, 10, 1)
    FIELD(SPI_DMA_CONF, INDSCR_BURST_EN, 11, 1)
    FIELD(SPI_DMA_CONF, OUT_DATA_BURST_EN, 12, 1)
    FIELD(SPI_DMA_CONF, DMA_RX_STOP, 14, 1)
    FIELD(SPI_DMA_CONF, DMA_TX_STOP, 15, 1)
    FIELD(SPI_DMA_CONF, DMA_CONTINUE, 16, 1)
REG32(SPI_DMA_OUT_LINK, 0x04)
    FIELD(SPI_DMA_OUT_LINK, ADDR, 0, 20)
    FIELD(SPI_DMA_OUT_LINK, STOP, 28, 1)
    FIELD(SPI_DMA_OUT_LINK, START, 29, 1)
    FIELD(SPI_DMA_OUT_LINK, RESTART, 30, 1)
REG32(SPI_DMA_IN_LINK, 0x08)
    FIELD(SPI_DMA_IN_LINK, ADDR, 0, 20)
    FIELD(SPI_DMA_IN_LINK, AUTO_RET, 20, 1)
    FIELD(SPI_DMA_IN_LINK, STOP, 28, 1)
    FIELD(SPI_DMA_IN_LINK, START, 29, 1)
    FIELD(SPI_DMA_IN_LINK, RESTART, 30, 1)
REG32(SPI_DMA_STATUS, 0x0c)
    FIELD(SPI_DMA_STATUS, RX_EN, 0, 1)
    FIELD(SPI_DMA_STATUS, TX_EN, 1, 1)
REG32(SPI_DMA_INT_ENA, 0x10)
REG32(SPI_DMA_INT_RAW, 0x14)
REG32(SPI_DMA_INT_ST, 0x18)
REG32(SPI_DMA_INT_CLR, 0x1c)
    FIELD(SPI_DMA_INT, INLINK_DSCR_EMPTY, 0, 1)
    FIELD(SPI_DMA_INT, OUTLINK_DSCR_ERROR, 1, 1)
    FIELD(SPI_DMA_INT, INLINK_DSCR_ERROR, 2, 1)
    FIELD(SPI_DMA_INT, IN_DONE, 3, 1)
    FIELD(SPI_DMA_INT, IN_ERR_EOF, 4, 1)
    FIELD(SPI_DMA_INT, IN_SUC_EOF, 5, 1)
    FIELD(SPI_DMA_INT, OUT_DONE, 6, 1)
    FIELD(SPI_DMA_INT, OUT_EOF, 7, 1)
    FIELD(SPI_DMA_INT, OUT_TOTAL_EOF, 8, 1)
REG32(SPI_IN_ERR_EOF_DES_ADDR, 0x20)
REG32(SPI_IN_SUC_EOF_DES_ADDR, 0x24)
REG32(SPI_INLINK_DSCR, 0x28)
REG32(SPI_INLINK_DSCR_BF0, 0x2c)
REG32(SPI_INLINK_DSCR_BF1, 0x30)
REG32(SPI_OUT_EOF_BFR_DES_ADDR, 0x34)
REG32(SPI_OUT_EOF_DES_ADDR, 0x38)
REG32(SPI_OUTLINK_DSCR, 0x3c)
REG32(SPI_OUTLINK_DSCR_BF0, 0x40)
REG32(SPI_OUTLINK_DSCR_BF1, 0x44)
REG32(SPI_DMA_RSTATUS, 0x48)
    FIELD(SPI_DMA_RSTATUS, DES_ADDRESS, 0, 20)
    FIELD(SPI_DMA_RSTATUS, FIFO_FULL, 30, 1)
    FIELD(SPI_DMA_RSTATUS, FIFO_EMPTY, 31, 1)
REG32(SPI_DMA_TSTATUS, 0x4c)

/*
 * The SPI controller's side of the data path. A data phase takes its MOSI
 * bytes from the outlink while esp32_spi_dma_tx_enabled(), and gives its
 * MISO bytes to the inlink while esp32_spi_dma_rx_enabled(), in place of
 * the W0..W15 buffer. esp32_spi_dma_trans_done() ends the transaction.
 */
bool esp32_spi_dma_tx_enabled(Esp32SpiDmaState *s);
bool esp32_spi_dma_rx_enabled(Esp32SpiDmaState *s);
uint8_t esp32_spi_dma_pop(Esp32SpiDmaState *s);
void esp32_spi_dma_push(Esp32SpiDmaState *s, uint8_t byte);
void esp32_spi_dma_trans_done(Esp32SpiDmaState *s);
