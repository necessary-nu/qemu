/*
 * ESP32 SPI controller
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * [spec:nuos:req:emu.esp32.gpio]
 * SPI0 (the cache's) and SPI1 share one set of lines, the GPIO matrix
 * signals SPICLK, SPIQ, SPID, SPIHD, SPIWP and SPICS0 to SPICS2 (TRM
 * table 6.9-1), which reach their pads through IO_MUX function 1 of GPIO6
 * to GPIO11 or through the matrix. The chip on CS0, the module's flash,
 * sits on those pads, and the machine says how they join the two
 * (Esp32SpiFlashLines). A transaction on CS0 then crosses the lines as
 * the hardware sends it: the command on IO0 (SPID); the address on IO0,
 * on IO0 and IO1 with FREAD_DIO (a read) or FWRITE_DIO (a write), on IO0
 * to IO3 with FREAD_QIO or FWRITE_QIO; read data on IO1 (SPIQ), on IO0
 * and IO1 with FREAD_DUAL or FREAD_DIO, on all four with FREAD_QUAD or
 * FREAD_QIO; written data likewise with FWRITE_*; in full duplex, out on
 * IO0 and in on IO1. On two lines a byte's bits 7, 5, 3 and 1 are IO1's,
 * on four bits 7 and 3 IO3's, 6 and 2 IO2's, 5 and 1 IO1's. The flash
 * takes part only selected (its CS#: the controller's where SPICS0
 * reaches it, else its pad's level, which can hold it selected across
 * transactions) and clocked (SPICLK reaching it, HOLD# not holding it);
 * it takes, on each line, the controller's bits where they reach it and
 * its pin's level elsewhere, and answers only a command that reached it
 * as sent; the controller takes the flash's bits on the lines they reach
 * it on and, elsewhere or with the flash silent, what its inputs read.
 * The model's flash takes commands as bytes, the dedicated read always
 * as READ (03h) whatever the mode, so a read's dual or quad lines are
 * the byte's bits, not the clocks. Not modelled: the dummy and mode
 * cycles, which carry nothing the model's flash takes, and the timing.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/system.h"
#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/misc/esp32_flash_enc.h"
#include "system/block-backend.h"



enum {
    CMD_RES = 0xab,
    CMD_DP = 0xb9,
    CMD_CE = 0x60,
    CMD_BE = 0xD8,
    CMD_SE = 0x20,
    CMD_PP = 0x02,
    CMD_WRSR = 0x1,
    CMD_RDSR = 0x5,
    CMD_RDID = 0x9f,
    CMD_WRDI = 0x4,
    CMD_WREN = 0x6,
    CMD_READ = 0x03,
};


#define ESP32_SPI_REG_SIZE    0x1000

static void esp32_spi_do_command(Esp32SpiState* state, uint32_t cmd_reg);

/* A cache block, which a cache miss reads from the flash (TRM 3.3.2) */
#define ESP32_SPI_CACHE_BLOCK   32

static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_SPI_ADDR:
        r = s->addr_reg;
        break;
    case A_SPI_CTRL:
        r = s->ctrl_reg;
        break;
    case A_SPI_STATUS:
        r = s->status_reg;
        break;
    case A_SPI_CTRL1:
        r = s->ctrl1_reg;
        break;
    case A_SPI_CTRL2:
        r = s->ctrl2_reg;
        break;
    case A_SPI_USER:
        r = s->user_reg;
        break;
    case A_SPI_USER1:
        r = s->user1_reg;
        break;
    case A_SPI_USER2:
        r = s->user2_reg;
        break;
    case A_SPI_MOSI_DLEN:
        r = s->mosi_dlen_reg;
        break;
    case A_SPI_MISO_DLEN:
        r = s->miso_dlen_reg;
        break;
    case A_SPI_PIN:
        r = s->pin_reg;
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)];
        break;
    case A_SPI_EXT2:
        r = 0;
        break;
    case A_SPI_SLAVE:
        r = BIT(R_SPI_SLAVE_TRANS_DONE_SHIFT) | BIT(R_SPI_SLAVE_TRANS_INTEN_SHIFT);
        break;
    }
    return r;
}

static void esp32_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    switch (addr) {
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)] = value;
        break;
    case A_SPI_ADDR:
        s->addr_reg = value;
        break;
    case A_SPI_CTRL:
        s->ctrl_reg = value;
        break;
    case A_SPI_STATUS:
        s->status_reg = value;
        break;
    case A_SPI_CTRL1:
        s->ctrl1_reg = value;
        break;
    case A_SPI_CTRL2:
        s->ctrl2_reg = value;
        break;
    case A_SPI_USER:
        s->user_reg = value;
        break;
    case A_SPI_USER1:
        s->user1_reg = value;
        break;
    case A_SPI_USER2:
        s->user2_reg = value;
        break;
    case A_SPI_MOSI_DLEN:
        s->mosi_dlen_reg = value;
        break;
    case A_SPI_MISO_DLEN:
        s->miso_dlen_reg = value;
        break;
    case A_SPI_PIN:
        s->pin_reg = value;
        break;
    case A_SPI_CMD:
        esp32_spi_do_command(s, value);
        break;
    }
}

typedef struct Esp32SpiTransaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t* data;
    /* Size of the buffer data points to, in bytes */
    int data_size;
    /* MOSI and MISO shift together (SPI_DOUTDIN), not one after the other */
    bool full_duplex;
    /* A user-defined command, whose data phases can use the DMA engine */
    bool usr;
    /* The lines the address, written data and read data cross on */
    unsigned addr_w;
    unsigned out_w;
    unsigned in_w;
} Esp32SpiTransaction;

/* A transaction's view of the lines to the flash on CS0 */
typedef struct Esp32SpiLink {
    /* The transaction is on CS0, across the machine's lines */
    bool active;
    /* The flash is selected and clocked: it takes the bits */
    bool flash_on;
    /* ... and it answers: the command reached it as sent */
    bool flash_drives;
    Esp32SpiFlashLines l;
} Esp32SpiLink;

/* The bits of a byte line IOn carries over @width lines, in or out */
static uint8_t esp32_spi_line_bits(unsigned width, bool in, unsigned n)
{
    switch (width) {
    case 4:
        return 0x11 << n;
    case 2:
        return n < 2 ? 0x55 << n : 0;
    default:
        return n == (in ? 1 : 0) ? 0xff : 0;
    }
}

/*
 * Byte @b as its receiver takes it over @width lines: on the lines in
 * @reach as sent, inverted on those in @inv, elsewhere at @level.
 */
static uint8_t esp32_spi_line_byte(uint8_t b, unsigned width, bool in,
                                   uint8_t reach, uint8_t inv, uint8_t level)
{
    for (unsigned n = 0; n < 4; n++) {
        uint8_t bits = esp32_spi_line_bits(width, in, n);

        if (!(reach & BIT(n))) {
            b = (b & ~bits) | ((level & BIT(n)) ? bits : 0);
        } else if (inv & BIT(n)) {
            b ^= bits;
        }
    }
    return b;
}

/* The flash's CS# with the controller's SPICS0 at @ctrl_cs */
static bool esp32_spi_flash_cs(const Esp32SpiFlashLines *l, bool ctrl_cs)
{
    return l->cs ? ctrl_cs ^ l->cs_inv : l->cs_level;
}

/*
 * The lines a read's address and data, and a write's, cross on, as CTRL's
 * FREAD_* and USER's FWRITE_* say.
 */
static unsigned esp32_spi_fread_addr_w(uint32_t ctrl)
{
    return FIELD_EX32(ctrl, SPI_CTRL, FREAD_QIO) ? 4 :
           FIELD_EX32(ctrl, SPI_CTRL, FREAD_DIO) ? 2 : 1;
}

static unsigned esp32_spi_fread_data_w(uint32_t ctrl)
{
    return FIELD_EX32(ctrl, SPI_CTRL, FREAD_QIO) ||
           FIELD_EX32(ctrl, SPI_CTRL, FREAD_QUAD) ? 4 :
           FIELD_EX32(ctrl, SPI_CTRL, FREAD_DIO) ||
           FIELD_EX32(ctrl, SPI_CTRL, FREAD_DUAL) ? 2 : 1;
}

static unsigned esp32_spi_fwrite_addr_w(uint32_t user)
{
    return FIELD_EX32(user, SPI_USER, FWRITE_QIO) ? 4 :
           FIELD_EX32(user, SPI_USER, FWRITE_DIO) ? 2 : 1;
}

static unsigned esp32_spi_fwrite_data_w(uint32_t user)
{
    return FIELD_EX32(user, SPI_USER, FWRITE_QIO) ||
           FIELD_EX32(user, SPI_USER, FWRITE_QUAD) ? 4 :
           FIELD_EX32(user, SPI_USER, FWRITE_DIO) ||
           FIELD_EX32(user, SPI_USER, FWRITE_DUAL) ? 2 : 1;
}

/* The read command the read mode in CTRL sends */
static uint8_t esp32_spi_fread_cmd(uint32_t ctrl)
{
    if (FIELD_EX32(ctrl, SPI_CTRL, FREAD_QIO)) {
        return 0xeb;
    }
    if (FIELD_EX32(ctrl, SPI_CTRL, FREAD_DIO)) {
        return 0xbb;
    }
    if (FIELD_EX32(ctrl, SPI_CTRL, FREAD_QUAD)) {
        return 0x6b;
    }
    if (FIELD_EX32(ctrl, SPI_CTRL, FREAD_DUAL)) {
        return 0x3b;
    }
    return FIELD_EX32(ctrl, SPI_CTRL, FASTRD_MODE) ? 0x0b : 0x03;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A transaction begins: on CS0, it crosses the machine's lines to the
 * flash, which takes part selected and clocked, and gets its WP# as the
 * lines hold it.
 */
static void esp32_spi_link_begin(Esp32SpiState *s, Esp32SpiLink *k)
{
    memset(k, 0, sizeof(*k));
    if (!s->flash_lines) {
        return;
    }
    s->flash_lines(s->flash_lines_opaque, &k->l);
    qemu_set_irq(s->flash_wp, k->l.wp_level);
    k->active = !FIELD_EX32(s->pin_reg, SPI_PIN, CS0_DIS);
    k->flash_on = k->active && !esp32_spi_flash_cs(&k->l, 0) && k->l.clk &&
                  !k->l.hold;
    k->flash_drives = k->flash_on;
    if (k->active && !k->flash_on) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: the flash on CS0 is not "
                      "%s, as its pads are set up: it takes nothing\n",
                      k->l.hold ? "free of HOLD#" :
                      !k->l.clk ? "clocked" : "selected");
    }
}

/*
 * One byte across the bus: @out sent on @out_w lines (0: the controller
 * sends nothing the flash takes), the byte read back on @in_w lines (0:
 * none). Without the flash's lines, the bus is wires.
 */
static uint8_t esp32_spi_xfer(Esp32SpiState *s, Esp32SpiLink *k, uint8_t out,
                              unsigned out_w, unsigned in_w)
{
    const Esp32SpiFlashLines *l = &k->l;
    uint8_t f = 0;

    if (!k->active) {
        return ssi_transfer(s->spi, out);
    }
    if (out_w) {
        out = esp32_spi_line_byte(out, out_w, false, l->to_flash,
                                  l->flash_inv, l->flash_level);
    }
    if (k->flash_on) {
        f = ssi_transfer(s->spi, out);
    }
    if (!in_w) {
        return out;
    }
    if (!k->flash_drives) {
        return esp32_spi_line_byte(0, in_w, true, 0, 0, l->ctrl_level);
    }
    return esp32_spi_line_byte(f, in_w, true, l->to_ctrl, l->ctrl_inv,
                               l->ctrl_level);
}

/*
 * The command phase, on IO0: the flash answers the transaction only if
 * the command reaches it as sent.
 */
static void esp32_spi_cmd_phase(Esp32SpiState *s, Esp32SpiLink *k,
                                const uint8_t *buf, int n)
{
    for (int i = 0; i < n; i++) {
        if (esp32_spi_xfer(s, k, buf[i], 1, 0) != buf[i] && k->active) {
            if (k->flash_drives) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: the flash on CS0 "
                              "takes command 0x%02x as 0x%02x, as its pads "
                              "are set up\n", buf[i],
                              esp32_spi_line_byte(buf[i], 1, false,
                                                  k->l.to_flash,
                                                  k->l.flash_inv,
                                                  k->l.flash_level));
            }
            k->flash_drives = false;
        }
    }
}

static void esp32_spi_out_bytes(Esp32SpiState *s, Esp32SpiLink *k,
                                const uint8_t *buf, int n, unsigned width)
{
    for (int i = 0; i < n; i++) {
        esp32_spi_xfer(s, k, buf[i], width, 0);
    }
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * The data phase of a transaction. MOSI bytes come from the outlink while
 * the DMA engine's TX is enabled, else from the CPU buffer; MISO bytes go
 * to the inlink while its RX is enabled, else to the CPU buffer. In full
 * duplex both shift together; in half duplex the MOSI phase precedes the
 * MISO phase.
 */
static void esp32_spi_data_phase(Esp32SpiState *s, Esp32SpiLink *k,
                                 Esp32SpiTransaction *t)
{
    uint8_t *buf = (uint8_t *)t->data;
    bool tx_dma = t->usr && s->dma && t->data_tx_bytes > 0 &&
                  esp32_spi_dma_tx_enabled(s->dma);
    bool rx_dma = t->usr && s->dma && t->data_rx_bytes > 0 &&
                  esp32_spi_dma_rx_enabled(s->dma);
    int bytes = t->full_duplex ? MAX(t->data_tx_bytes, t->data_rx_bytes)
                               : t->data_tx_bytes + t->data_rx_bytes;

    if ((!tx_dma && t->data_tx_bytes > t->data_size) ||
        (!rx_dma && t->data_rx_bytes > t->data_size)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi: data phase longer than the %d-byte buffer\n",
                      t->data_size);
    }
    for (int i = 0; i < bytes; ++i) {
        int tx_i = i;
        int rx_i = t->full_duplex ? i : i - t->data_tx_bytes;
        uint8_t out = 0;
        uint8_t in;

        if (tx_i < t->data_tx_bytes) {
            if (tx_dma) {
                out = esp32_spi_dma_pop(s->dma);
            } else if (tx_i < t->data_size) {
                out = buf[tx_i];
            }
        }
        if (t->full_duplex) {
            in = esp32_spi_xfer(s, k, out, 1, 1);
        } else if (tx_i < t->data_tx_bytes) {
            in = esp32_spi_xfer(s, k, out, t->out_w, 0);
        } else {
            in = esp32_spi_xfer(s, k, 0, 0, t->in_w);
        }
        if (rx_i >= 0 && rx_i < t->data_rx_bytes) {
            if (rx_dma) {
                esp32_spi_dma_push(s->dma, in);
            } else if (rx_i < t->data_size) {
                buf[rx_i] = in;
            }
        }
    }
}

/*
 * The chip selects, CS0 as it reaches the flash where the machine gives
 * its lines.
 */
static void esp32_spi_cs_set(Esp32SpiState *s, Esp32SpiLink *k, int value)
{
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        int level = ((s->pin_reg & (1 << i)) == 0) ? value : 1;

        if (i == 0 && s->flash_lines) {
            level = esp32_spi_flash_cs(&k->l, level);
        }
        qemu_set_irq(s->cs_gpio[i], level);
    }
}

static void esp32_spi_transaction(Esp32SpiState *s, Esp32SpiTransaction *t)
{
    Esp32SpiLink k;

    esp32_spi_link_begin(s, &k);
    esp32_spi_cs_set(s, &k, 0);
    esp32_spi_cmd_phase(s, &k, (uint8_t *)&t->cmd, t->cmd_bytes);
    esp32_spi_out_bytes(s, &k, (uint8_t *)&t->addr, t->addr_bytes,
                        t->addr_w);
    esp32_spi_data_phase(s, &k, t);
    esp32_spi_cs_set(s, &k, 1);
    if (t->usr && s->dma) {
        esp32_spi_dma_trans_done(s->dma);
    }
}

/* Convert one of the hardware "bitlen" registers to a byte count */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8; /* bitlen registers hold number of bits, minus one */
}

static void maybe_encrypt_data(Esp32SpiState *s)
{
    Esp32FlashEncryptionState* flash_enc = esp32_flash_encryption_find();
    if (esp32_flash_encryption_enabled(flash_enc)) {
        esp32_flash_encryption_get_result(flash_enc, &s->data_reg[0], 8);
    }
}

static void esp32_spi_do_command(Esp32SpiState* s, uint32_t cmd_reg)
{
    Esp32SpiTransaction t = {
        .cmd_bytes = 1,
        .data_size = sizeof(s->data_reg),
        .addr_w = 1,
        .out_w = 1,
        .in_w = 1,
    };
    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t.cmd = CMD_READ;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        t.data = &s->data_reg[0];
        t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        t.addr_w = esp32_spi_fread_addr_w(s->ctrl_reg);
        t.in_w = esp32_spi_fread_data_w(s->ctrl_reg);
        break;

    case R_SPI_CMD_WREN_MASK:
        t.cmd = CMD_WREN;
        break;

    case R_SPI_CMD_WRDI_MASK:
        t.cmd = CMD_WRDI;
        break;

    case R_SPI_CMD_RDID_MASK:
        t.cmd = CMD_RDID;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_RDSR_MASK:
        t.cmd = CMD_RDSR;
        t.data = &s->status_reg;
        t.data_size = sizeof(s->status_reg);
        t.data_rx_bytes = 1;
        break;

    case R_SPI_CMD_WRSR_MASK:
        t.cmd = CMD_WRSR;
        t.data = &s->status_reg;
        t.data_size = sizeof(s->status_reg);
        t.data_tx_bytes = 1;
        break;

    case R_SPI_CMD_PP_MASK:
        maybe_encrypt_data(s);
        t.cmd = CMD_PP;
        t.data = &s->data_reg[0];
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> 8;
        t.data = &s->data_reg[0];
        t.data_tx_bytes = s->addr_reg >> 24;
        break;

    case R_SPI_CMD_SE_MASK:
        t.cmd = CMD_SE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_BE_MASK:
        t.cmd = CMD_BE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_CE_MASK:
        t.cmd = CMD_CE;
        break;

    case R_SPI_CMD_DP_MASK:
        t.cmd = CMD_DP;
        break;

    case R_SPI_CMD_RES_MASK:
        t.cmd = CMD_RES;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_USR_MASK:
        maybe_encrypt_data(s);
        t.usr = true;
        t.full_duplex = FIELD_EX32(s->user_reg, SPI_USER, DOUTDIN);
        if (FIELD_EX32(s->user_reg, SPI_USER, COMMAND) || FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN)) {
            t.cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
            t.cmd_bytes = bitlen_to_bytes(FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN));
        } else {
            t.cmd_bytes = 0;
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {
            t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
            t.addr = bswap32(s->addr_reg);
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MOSI)) {
            t.data = &s->data_reg[0];
            t.data_tx_bytes = bitlen_to_bytes(s->mosi_dlen_reg);
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MISO)) {
            t.data = &s->data_reg[0];
            t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        }
        if (!t.full_duplex) {
            t.addr_w = FIELD_EX32(s->user_reg, SPI_USER, MISO) ?
                       esp32_spi_fread_addr_w(s->ctrl_reg) :
                       esp32_spi_fwrite_addr_w(s->user_reg);
            t.out_w = esp32_spi_fwrite_data_w(s->user_reg);
            t.in_w = esp32_spi_fread_data_w(s->ctrl_reg);
        }
        break;
    default:
        return;
    }
    esp32_spi_transaction(s, &t);
}


/* [spec:nuos:req:emu.esp32.gpio] */
void esp32_spi_set_flash_lines(Esp32SpiState *s, Esp32SpiFlashLinesFunc *fn,
                               void *opaque)
{
    s->flash_lines = fn;
    s->flash_lines_opaque = opaque;
    esp32_spi_flash_lines_changed(s);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Between transactions the flash's CS# and WP# follow the lines: the
 * controller's idle SPICS0 where it reaches the flash, else the pad's
 * level, which can hold the flash selected.
 */
void esp32_spi_flash_lines_changed(Esp32SpiState *s)
{
    Esp32SpiFlashLines l;

    if (!s->flash_lines) {
        return;
    }
    s->flash_lines(s->flash_lines_opaque, &l);
    qemu_set_irq(s->flash_wp, l.wp_level);
    qemu_set_irq(s->cs_gpio[0], esp32_spi_flash_cs(&l, 1));
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A cache fill: the cache reads each 32-byte block (TRM 3.3.2) with one
 * transaction on SPI0, in the read mode SPI0's CTRL sets (FREAD_*, with
 * its read command: 03h, 0Bh, 3Bh, BBh, 6Bh or EBh). With every line that
 * mode uses reaching the flash and back, the flash's array comes as it is.
 * Otherwise, block by block: the flash answers only selected, clocked and
 * given the command as sent, at the address as it takes it off its
 * address lines (wrapping at its size); the cache takes each data line's
 * bits from the flash where they reach it, else what SPI0's input reads.
 */
void esp32_spi_flash_cache_read(Esp32SpiState *s, BlockBackend *blk,
                                uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t ctrl = s->ctrl_reg;
    unsigned aw = esp32_spi_fread_addr_w(ctrl);
    unsigned dw = esp32_spi_fread_data_w(ctrl);
    uint8_t cmd = esp32_spi_fread_cmd(ctrl);
    uint8_t out_mask = MAKE_64BIT_MASK(0, aw);
    uint8_t in_mask = dw == 1 ? BIT(1) : MAKE_64BIT_MASK(0, dw);
    int64_t size = blk_getlength(blk);
    Esp32SpiFlashLines l;
    bool on, drives;

    if (!s->flash_lines || size <= 0) {
        blk_pread(blk, addr, len, buf, 0);
        return;
    }
    s->flash_lines(s->flash_lines_opaque, &l);
    on = !esp32_spi_flash_cs(&l, 0) && l.clk && !l.hold;
    if (on && (l.to_flash & out_mask) == out_mask &&
        !(l.flash_inv & out_mask) && (l.to_ctrl & in_mask) == in_mask &&
        !(l.ctrl_inv & in_mask)) {
        blk_pread(blk, addr, len, buf, 0);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: cache fill at flash address "
                  "0x%06x with the flash's pads not set up for SPI0's read "
                  "mode\n", addr);
    drives = on && esp32_spi_line_byte(cmd, 1, false, l.to_flash, l.flash_inv,
                                       l.flash_level) == cmd;
    if (drives && (l.to_flash & out_mask) == out_mask &&
        !(l.flash_inv & out_mask)) {
        /* Every block from its own address: only the data lines differ */
        blk_pread(blk, addr, len, buf, 0);
        for (uint32_t i = 0; i < len; i++) {
            buf[i] = esp32_spi_line_byte(buf[i], dw, true, l.to_ctrl,
                                         l.ctrl_inv, l.ctrl_level);
        }
        return;
    }
    for (uint32_t off = 0; off < len; off += ESP32_SPI_CACHE_BLOCK) {
        uint32_t n = MIN(ESP32_SPI_CACHE_BLOCK, len - off);
        uint8_t block[ESP32_SPI_CACHE_BLOCK] = { 0 };
        uint32_t a = addr + off, fa = 0;

        if (drives) {
            for (int b = 2; b >= 0; b--) {
                fa = (fa << 8) | esp32_spi_line_byte(a >> (8 * b), aw, false,
                                                     l.to_flash, l.flash_inv,
                                                     l.flash_level);
            }
            fa %= size;
            if (fa + n <= size) {
                blk_pread(blk, fa, n, block, 0);
            } else {
                blk_pread(blk, fa, size - fa, block, 0);
                blk_pread(blk, 0, n - (size - fa), block + (size - fa), 0);
            }
        }
        for (uint32_t i = 0; i < n; i++) {
            buf[off + i] = drives ?
                esp32_spi_line_byte(block[i], dw, true, l.to_ctrl, l.ctrl_inv,
                                    l.ctrl_level) :
                esp32_spi_line_byte(0, dw, true, 0, 0, l.ctrl_level);
        }
    }
}

static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset_hold(Object *obj, ResetType type)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, SPI_USER2, COMMAND_BITLEN, 4);
    s->user2_reg = FIELD_DP32(s->user2_reg, SPI_USER2, COMMAND_VALUE, 0);
    s->status_reg = 0;
}

static void esp32_spi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_spi_init(Object *obj)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s,
                          TYPE_ESP32_SPI, ESP32_SPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI_CS_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), &s->flash_wp, ESP32_SPI_FLASH_WP, 1);
}

static const Property esp32_spi_properties[] = {
    DEFINE_PROP_LINK("dma", Esp32SpiState, dma, TYPE_ESP32_SPI_DMA,
                     Esp32SpiDmaState *),
};

static void esp32_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_spi_reset_hold;
    dc->realize = esp32_spi_realize;
    device_class_set_props(dc, esp32_spi_properties);
}

static const TypeInfo esp32_spi_info = {
    .name = TYPE_ESP32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SpiState),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init
};

static void esp32_spi_register_types(void)
{
    type_register_static(&esp32_spi_info);
}

type_init(esp32_spi_register_types)
