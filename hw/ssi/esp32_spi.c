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
 * (Esp32SpiFlashLines). A transaction on CS0 crosses the lines clock by
 * clock, as the hardware sends it and as the flash takes it:
 *
 * - the controller's phases (TRM 20.3.2, 20.5): the command on IO0 (SPID);
 *   the address on IO0, on IO0 and IO1 with FREAD_DIO (a read) or
 *   FWRITE_DIO (a write), on IO0 to IO3 with FREAD_QIO or FWRITE_QIO; the
 *   dummy clocks, its data lines released; read data on IO1 (SPIQ), on
 *   IO0 and IO1 with FREAD_DUAL or FREAD_DIO, on all four with FREAD_QUAD
 *   or FREAD_QIO; written data likewise with FWRITE_*; in full duplex, out
 *   on IO0 and in on IO1. On n lines a clock carries n bits, the highest
 *   line the most significant. Outside a phase that uses them, SPIWP
 *   drives CTRL's SPI_WP and SPIHD drives high;
 * - the flash takes, on each line, the controller's level where its output
 *   reaches the pin and is on, else the pin's level; it takes part only
 *   selected (its CS#: the controller's where SPICS0 reaches it, else its
 *   pad's level, which can hold it selected across transactions), clocked
 *   (SPICLK reaching it), powered, and on each clock only with HOLD# (IO3,
 *   while its QE is clear) high. It groups the clocks it takes into its
 *   bytes as its datasheet lays out the command it received, on the lines
 *   it uses for each (the GD25Q32C's: BBh's and 92h's address and mode on
 *   two lines, EBh's, E7h's and 94h's on four, then their dummy clocks;
 *   0Bh's, 3Bh's and 6Bh's eight dummy clocks; 32h's data on four lines;
 *   everything else on IO0 in and IO1 out), and starts with the address
 *   where its continuous read mode keeps the command;
 * - the controller takes, on each line it reads, the flash's level where
 *   the flash drives it and its output reaches the controller's input,
 *   else what the input reads. An input through the GPIO matrix is two
 *   APB_CLK cycles late (TRM 20.4.2): the controller samples what the
 *   flash drove floor(2 * f_SPI / f_APB) SPI clocks earlier, one at
 *   APB_CLK / 2 and two at APB_CLK, which ESP-IDF's extra dummy cycles
 *   (g_rom_spiflash_dummy_len_plus, 1 at 40 MHz and 2 at 80 MHz) make up
 *   for. A dummy phase that does not match the flash's, or an input
 *   delay it does not make up for, shifts what the controller reads by
 *   the difference.
 *
 * The dedicated commands (SPI_CMD's READ to PER) send the flash's
 * commands: READ the one the read mode in CTRL gives (EBh with FREAD_QIO,
 * BBh with FREAD_DIO, 6Bh with FREAD_QUAD, 3Bh with FREAD_DUAL, 0Bh with
 * FASTRD_MODE, else 03h), its address from SPI_ADDR's top USR_ADDR_BITLEN
 * + 1 bits, then USR_DUMMY_CYCLELEN + 1 dummy clocks with USR_DUMMY, then
 * MISO_DLEN + 1 bits into W0; PP, SE and BE their 24-bit address from
 * SPI_ADDR[23:0], PP the number of bytes in SPI_ADDR[31:24] from W0; RDSR
 * a byte into SPI_RD_STATUS, WRSR one from it (two with WRSR_2B, S7-S0
 * first); RDID three bytes into W0; RES, with RESANDRES, three dummy
 * bytes and the device ID into SPI_RD_STATUS. They have no dummy phase:
 * ESP-IDF's ROM code reads the status with a user command where it needs
 * dummy clocks. USR sends what USER, USER1 and USER2 configure.
 *
 * Timing: a transaction takes its SPI clocks, at APB_CLK divided as
 * SPI_CLOCK gives (TRM 20.4), with CS_SETUP's SETUP_TIME + 1.5 clocks
 * before and CS_HOLD's HOLD_TIME after, in virtual time. SPI_CMD's bit
 * reads set, and SPI_EXT2's state machine busy, until it ends; its bits
 * cross the lines then, and SLAVE's TRANS_DONE sets, interrupting with
 * TRANS_INTEN. A cache fill on SPI0 that comes while SPI1's transaction
 * runs comes after it, as their arbitration has it.
 *
 * Not modelled: the bit and byte orders (RD/WR_BIT_ORDER, RD/WR_BYTE_ORDER,
 * which ESP-IDF leaves clear), MISO_DELAY_MODE and MISO_DELAY_NUM, the
 * dummy phase with the clock stopped (USR_DUMMY_IDLE), the prep/hold
 * states, the CRC, slave mode, and the cache's 4-byte addresses
 * (CACHE_USR_CMD_4BYTE) and suspend before a read (CACHE_FLASH_PES_EN);
 * each is logged when software sets it.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/system.h"
#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/block/flash.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/misc/esp32_flash_enc.h"
#include "migration/vmstate.h"

#define ESP32_SPI_REG_SIZE    0x1000
#define ESP32_SPI_BUF_BYTES   (ESP32_SPI_BUF_WORDS * 4)

/* SLAVE's TRANS_CNT, SLV_LAST_STATE and SLV_LAST_COMMAND */
#define ESP32_SPI_SLAVE_RO    0x07fe0000

/* The flash's commands the dedicated SPI_CMD bits send */
enum {
    FLASH_READ = 0x03,
    FLASH_PP = 0x02,
    FLASH_WRSR = 0x01,
    FLASH_RDSR = 0x05,
    FLASH_WRDI = 0x04,
    FLASH_WREN = 0x06,
    FLASH_SE = 0x20,
    FLASH_PES = 0x75,
    FLASH_PER = 0x7a,
    FLASH_RDID = 0x9f,
    FLASH_HPM = 0xa3,
    FLASH_RES = 0xab,
    FLASH_DP = 0xb9,
    FLASH_CE = 0x60,
    FLASH_BE = 0xd8,
};

/* Where a data phase's bytes come from or go */
typedef enum {
    DATA_NONE,
    DATA_BUF,       /* W0..W15 from a byte offset, wrapping at 64 bytes */
    DATA_STATUS,    /* SPI_RD_STATUS, byte 0 first */
} Esp32SpiData;

/*
 * A transaction as the controller clocks it. The address is left-aligned
 * in addr: its first bit is bit 63.
 */
typedef struct Esp32SpiTransaction {
    uint32_t cmd;
    unsigned cmd_bits;
    uint64_t addr;
    unsigned addr_bits;
    unsigned addr_w;
    unsigned dummy;
    unsigned out_bits;
    unsigned out_w;
    unsigned in_bits;
    unsigned in_w;
    bool full_duplex;
    /* A user-defined command, whose data phases can use the DMA engine */
    bool usr;
    Esp32SpiData out_to;
    unsigned out_off;
    Esp32SpiData in_to;
    unsigned in_off;
} Esp32SpiTransaction;

/* What a data phase moves, as it moves it */
typedef struct Esp32SpiStream {
    Esp32SpiState *s;
    const Esp32SpiTransaction *t;
    bool tx_dma;
    bool rx_dma;
    /* A cache fill's block, in place of the controller's buffers */
    uint8_t *block;
    unsigned out_pos;
    uint8_t out_byte;
    unsigned in_pos;
    uint8_t in_byte;
    unsigned in_got;
} Esp32SpiStream;

static void esp32_spi_done(void *opaque);

/* The lines a read's address and data, and a write's, cross on */
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
    return FIELD_EX32(ctrl, SPI_CTRL, FASTRD_MODE) ? 0x0b : FLASH_READ;
}

static uint8_t esp32_spi_buf_get(Esp32SpiState *s, unsigned i)
{
    i %= ESP32_SPI_BUF_BYTES;
    return s->data_reg[i / 4] >> (8 * (i % 4));
}

static void esp32_spi_buf_put(Esp32SpiState *s, unsigned i, uint8_t v)
{
    i %= ESP32_SPI_BUF_BYTES;
    s->data_reg[i / 4] = deposit32(s->data_reg[i / 4], 8 * (i % 4), 8, v);
}

/* Configure t from the controller's registers for command @cmd_reg */
static bool esp32_spi_decode(Esp32SpiState *s, uint32_t cmd_reg,
                             Esp32SpiTransaction *t)
{
    unsigned addr_bits = FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN) + 1;
    uint64_t addr_hi = (uint64_t)s->addr_reg << 32 | s->slv_wr_status_reg;
    uint64_t addr24 = (uint64_t)(s->addr_reg & 0xffffff) << 40;
    unsigned dummy = FIELD_EX32(s->user_reg, SPI_USER, DUMMY) ?
                     FIELD_EX32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN) + 1 :
                     0;

    memset(t, 0, sizeof(*t));
    t->cmd_bits = 8;
    t->addr_w = t->out_w = t->in_w = 1;

    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t->cmd = esp32_spi_fread_cmd(s->ctrl_reg);
        t->addr = addr_hi;
        t->addr_bits = addr_bits;
        t->addr_w = esp32_spi_fread_addr_w(s->ctrl_reg);
        t->dummy = dummy;
        t->in_bits = FIELD_EX32(s->miso_dlen_reg, SPI_MISO_DLEN, DBITLEN) + 1;
        t->in_w = esp32_spi_fread_data_w(s->ctrl_reg);
        t->in_to = DATA_BUF;
        break;
    case R_SPI_CMD_WREN_MASK:
        t->cmd = FLASH_WREN;
        break;
    case R_SPI_CMD_WRDI_MASK:
        t->cmd = FLASH_WRDI;
        break;
    case R_SPI_CMD_RDID_MASK:
        t->cmd = FLASH_RDID;
        t->in_bits = 24;
        t->in_to = DATA_BUF;
        break;
    case R_SPI_CMD_RDSR_MASK:
        t->cmd = FLASH_RDSR;
        t->in_bits = 8;
        t->in_to = DATA_STATUS;
        break;
    case R_SPI_CMD_WRSR_MASK:
        t->cmd = FLASH_WRSR;
        t->out_bits = FIELD_EX32(s->ctrl_reg, SPI_CTRL, WRSR_2B) ? 16 : 8;
        t->out_to = DATA_STATUS;
        break;
    case R_SPI_CMD_PP_MASK:
        t->cmd = FLASH_PP;
        t->addr = addr24;
        t->addr_bits = 24;
        t->out_bits = (s->addr_reg >> 24) * 8;
        t->out_to = DATA_BUF;
        break;
    case R_SPI_CMD_SE_MASK:
    case R_SPI_CMD_BE_MASK:
        t->cmd = cmd_reg == R_SPI_CMD_SE_MASK ? FLASH_SE : FLASH_BE;
        t->addr = addr24;
        t->addr_bits = 24;
        break;
    case R_SPI_CMD_CE_MASK:
        t->cmd = FLASH_CE;
        break;
    case R_SPI_CMD_DP_MASK:
        t->cmd = FLASH_DP;
        break;
    case R_SPI_CMD_RES_MASK:
        t->cmd = FLASH_RES;
        if (FIELD_EX32(s->ctrl_reg, SPI_CTRL, RESANDRES)) {
            t->addr_bits = 24;
            t->in_bits = 8;
            t->in_to = DATA_STATUS;
        }
        break;
    case R_SPI_CMD_HPM_MASK:
        t->cmd = FLASH_HPM;
        t->addr_bits = 24;
        break;
    case R_SPI_CMD_PES_MASK:
        t->cmd = FLASH_PES;
        break;
    case R_SPI_CMD_PER_MASK:
        t->cmd = FLASH_PER;
        break;
    case R_SPI_CMD_USR_MASK:
        t->usr = true;
        t->full_duplex = FIELD_EX32(s->user_reg, SPI_USER, DOUTDIN);
        t->cmd_bits = 0;
        if (FIELD_EX32(s->user_reg, SPI_USER, COMMAND)) {
            t->cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
            t->cmd_bits =
                FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN) + 1;
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {
            t->addr = addr_hi;
            t->addr_bits = addr_bits;
        }
        t->dummy = t->full_duplex ? 0 : dummy;
        if (FIELD_EX32(s->user_reg, SPI_USER, MOSI)) {
            t->out_bits =
                FIELD_EX32(s->mosi_dlen_reg, SPI_MOSI_DLEN, DBITLEN) + 1;
            t->out_to = DATA_BUF;
            t->out_off = FIELD_EX32(s->user_reg, SPI_USER, MOSI_HIGHPART) ?
                         32 : 0;
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MISO)) {
            t->in_bits =
                FIELD_EX32(s->miso_dlen_reg, SPI_MISO_DLEN, DBITLEN) + 1;
            t->in_to = DATA_BUF;
            t->in_off = FIELD_EX32(s->user_reg, SPI_USER, MISO_HIGHPART) ?
                        32 : 0;
        }
        if (!t->full_duplex) {
            t->addr_w = FIELD_EX32(s->user_reg, SPI_USER, MISO) ?
                        esp32_spi_fread_addr_w(s->ctrl_reg) :
                        esp32_spi_fwrite_addr_w(s->user_reg);
            t->out_w = esp32_spi_fwrite_data_w(s->user_reg);
            t->in_w = esp32_spi_fread_data_w(s->ctrl_reg);
        }
        break;
    default:
        return false;
    }
    if (t->addr_bits > 64) {
        t->addr_bits = 64;
    }
    return true;
}

/* The SPI clock's APB_CLK cycles */
static uint64_t esp32_spi_clock_div(uint32_t clock_reg)
{
    if (FIELD_EX32(clock_reg, SPI_CLOCK, CLK_EQU_SYSCLK)) {
        return 1;
    }
    return (uint64_t)(FIELD_EX32(clock_reg, SPI_CLOCK, CLKCNT_N) + 1) *
           (FIELD_EX32(clock_reg, SPI_CLOCK, CLKDIV_PRE) + 1);
}

static unsigned esp32_spi_clocks(unsigned bits, unsigned w)
{
    return DIV_ROUND_UP(bits, w);
}

/* The SPI clocks of the transaction's phases */
static uint64_t esp32_spi_xfer_clocks(const Esp32SpiTransaction *t)
{
    uint64_t n = t->cmd_bits + esp32_spi_clocks(t->addr_bits, t->addr_w) +
                 t->dummy;

    if (t->full_duplex) {
        return n + MAX(t->out_bits, t->in_bits);
    }
    return n + esp32_spi_clocks(t->out_bits, t->out_w) +
           esp32_spi_clocks(t->in_bits, t->in_w);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * The time @t takes on controller @s, in ns, with CS_SETUP's and CS_HOLD's
 * clocks around it; INT64_MAX with no APB_CLK.
 */
static int64_t esp32_spi_xfer_ns(Esp32SpiState *s,
                                 const Esp32SpiTransaction *t)
{
    uint64_t div = esp32_spi_clock_div(s->clock_reg);
    /* Half SPI clocks */
    uint64_t halves = 2 * esp32_spi_xfer_clocks(t);

    if (!clock_get(s->apb_clk)) {
        return INT64_MAX;
    }
    if (FIELD_EX32(s->user_reg, SPI_USER, CS_SETUP)) {
        halves += 2 * FIELD_EX32(s->ctrl2_reg, SPI_CTRL2, SETUP_TIME) + 3;
    }
    if (FIELD_EX32(s->user_reg, SPI_USER, CS_HOLD)) {
        halves += 2 * FIELD_EX32(s->ctrl2_reg, SPI_CTRL2, HOLD_TIME);
    }
    return clock_ticks_to_ns(s->apb_clk, DIV_ROUND_UP(halves * div, 2));
}

/* Data phases */

static void esp32_spi_stream_init(Esp32SpiStream *d, Esp32SpiState *s,
                                  const Esp32SpiTransaction *t,
                                  uint8_t *block)
{
    memset(d, 0, sizeof(*d));
    d->s = s;
    d->t = t;
    d->block = block;
    d->tx_dma = t->usr && s->dma && t->out_bits &&
                esp32_spi_dma_tx_enabled(s->dma);
    d->rx_dma = t->usr && s->dma && t->in_bits &&
                esp32_spi_dma_rx_enabled(s->dma);
}

/* [spec:nuos:req:emu.esp32.spi-dma] The next MOSI byte */
static uint8_t esp32_spi_out_next(Esp32SpiStream *d)
{
    Esp32SpiState *s = d->s;
    unsigned i = d->out_pos++;

    if (d->tx_dma) {
        return esp32_spi_dma_pop(s->dma);
    }
    switch (d->t->out_to) {
    case DATA_BUF:
        if (i >= ESP32_SPI_BUF_BYTES - d->t->out_off) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: MOSI data past the "
                          "end of the buffer\n");
        }
        return esp32_spi_buf_get(s, d->t->out_off + i);
    case DATA_STATUS:
        return s->status_reg >> (8 * (i % 4));
    default:
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.spi-dma] A MISO byte, the last one maybe short */
static void esp32_spi_in_put(Esp32SpiStream *d, uint8_t v)
{
    Esp32SpiState *s = d->s;
    unsigned i = d->in_pos++;

    if (d->block) {
        if (i < ESP32_SPI_CACHE_BLOCK) {
            d->block[i] = v;
        }
        return;
    }
    if (d->rx_dma) {
        esp32_spi_dma_push(s->dma, v);
        return;
    }
    switch (d->t->in_to) {
    case DATA_BUF:
        if (i >= ESP32_SPI_BUF_BYTES - d->t->in_off) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: MISO data past the "
                          "end of the buffer wraps to its start\n");
        }
        esp32_spi_buf_put(s, d->t->in_off + i, v);
        break;
    case DATA_STATUS:
        s->status_reg = deposit32(s->status_reg, 8 * (i % 4), 8, v);
        break;
    default:
        break;
    }
}

/* Take @n bits of @v (the low ones) in, MSB first */
static void esp32_spi_in_bits(Esp32SpiStream *d, unsigned v, unsigned n)
{
    for (unsigned i = n; i-- > 0;) {
        d->in_byte = d->in_byte << 1 | ((v >> i) & 1);
        if (++d->in_got == 8) {
            esp32_spi_in_put(d, d->in_byte);
            d->in_byte = 0;
            d->in_got = 0;
        }
    }
}

static void esp32_spi_in_flush(Esp32SpiStream *d)
{
    if (d->in_got) {
        esp32_spi_in_put(d, d->in_byte << (8 - d->in_got));
        d->in_got = 0;
    }
}

/* Chip selects */

/* The flash's CS# with the controller's SPICS0 at @ctrl_cs */
static bool esp32_spi_flash_cs(const Esp32SpiFlashLines *l, bool ctrl_cs)
{
    return l->cs ? ctrl_cs ^ l->cs_inv : l->cs_level;
}

/*
 * The chip selects of controller @s, on the lines of @bus, CS0 as it
 * reaches the flash where the machine gives its lines (@l).
 */
static void esp32_spi_cs_set(Esp32SpiState *s, Esp32SpiState *bus,
                             const Esp32SpiFlashLines *l, int value)
{
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        int level = ((s->pin_reg & (1 << i)) == 0) ? value : 1;

        if (i == 0 && l) {
            level = esp32_spi_flash_cs(l, level);
        }
        qemu_set_irq(bus->cs_gpio[i], level);
    }
}

/*
 * The wires path: SPI2 and SPI3, and SPI0/SPI1 on CS1 or with no flash,
 * whose chips take bytes: the command, the address, the data, a byte at a
 * time; the dummy clocks carry nothing to them.
 */
static void esp32_spi_wires(Esp32SpiState *s, const Esp32SpiTransaction *t)
{
    Esp32SpiStream d;
    unsigned n;

    esp32_spi_stream_init(&d, s, t, NULL);
    esp32_spi_cs_set(s, s, NULL, 0);
    for (unsigned i = 0; i < DIV_ROUND_UP(t->cmd_bits, 8); i++) {
        ssi_transfer(s->spi, t->cmd >> (8 * i));
    }
    for (unsigned i = 0; i < DIV_ROUND_UP(t->addr_bits, 8); i++) {
        ssi_transfer(s->spi, t->addr >> (56 - 8 * i));
    }
    if (t->full_duplex) {
        n = DIV_ROUND_UP(MAX(t->out_bits, t->in_bits), 8);
        for (unsigned i = 0; i < n; i++) {
            uint8_t out = i * 8 < t->out_bits ? esp32_spi_out_next(&d) : 0;
            uint8_t in = ssi_transfer(s->spi, out);

            if (i * 8 < t->in_bits) {
                esp32_spi_in_put(&d, in);
            }
        }
    } else {
        for (unsigned i = 0; i < DIV_ROUND_UP(t->out_bits, 8); i++) {
            ssi_transfer(s->spi, esp32_spi_out_next(&d));
        }
        for (unsigned i = 0; i < DIV_ROUND_UP(t->in_bits, 8); i++) {
            esp32_spi_in_put(&d, ssi_transfer(s->spi, 0));
        }
    }
    if (!FIELD_EX32(s->pin_reg, SPI_PIN, CS_KEEP_ACTIVE)) {
        esp32_spi_cs_set(s, s, NULL, 1);
    }
}

/*
 * The flash's view of a command it received: its address, mode and dummy
 * bytes after the command and the lines they come on, and the lines its
 * data comes or goes on (GD25Q32C datasheet, table 2 and figures 7 to 19,
 * 29, 30; the parts of other sizes the machine takes lay them out alike).
 * The rest go on IO0 in and IO1 out.
 */
typedef struct Esp32SpiFlashCmd {
    uint8_t cmd;
    uint8_t pre_bytes;
    uint8_t pre_w;
    uint8_t data_w;
} Esp32SpiFlashCmd;

static const Esp32SpiFlashCmd esp32_spi_flash_cmds[] = {
    { 0x3b, 4, 1, 2 },
    { 0x6b, 4, 1, 4 },
    { 0xbb, 4, 2, 2 },
    { 0x92, 4, 2, 2 },
    { 0xeb, 6, 4, 4 },
    { 0xe7, 5, 4, 4 },
    { 0x94, 6, 4, 4 },
    { 0x32, 3, 1, 4 },
    { 0x77, 3, 4, 4 },
};

static Esp32SpiFlashCmd esp32_spi_flash_cmd(uint8_t cmd)
{
    for (unsigned i = 0; i < ARRAY_SIZE(esp32_spi_flash_cmds); i++) {
        if (esp32_spi_flash_cmds[i].cmd == cmd) {
            return esp32_spi_flash_cmds[i];
        }
    }
    return (Esp32SpiFlashCmd) { cmd, 0, 1, 1 };
}

/* The lines n bits on @w lines go on: IO1 for one line in, IO0 out */
static uint8_t esp32_spi_lines(unsigned w, bool in)
{
    return w == 1 ? (in ? 0x2 : 0x1) : MAKE_64BIT_MASK(0, w);
}

/* The @w bits of a clock as the lines of @w carry them (1: on @line) */
static uint8_t esp32_spi_spread(unsigned bits, unsigned w, unsigned line)
{
    return w == 1 ? (bits & 1) << line : bits;
}

static unsigned esp32_spi_gather(uint8_t lines, unsigned w, unsigned line)
{
    return w == 1 ? (lines >> line) & 1 : lines & MAKE_64BIT_MASK(0, w);
}

/* The bits a clock carries from a left-aligned value: @k-th of @w each */
static unsigned esp32_spi_left_bits(uint64_t v, unsigned nbits, unsigned k,
                                    unsigned w)
{
    unsigned r = 0;

    for (unsigned j = 0; j < w; j++) {
        unsigned idx = k * w + j;

        r = r << 1 | (idx < nbits ? (v >> (63 - idx)) & 1 : 0);
    }
    return r;
}

/* The flash's side of a transaction, a clock at a time */
typedef struct Esp32SpiFlashSide {
    DeviceState *flash;
    SSIBus *bus;
    bool cont;
    Esp32SpiFlashCmd fc;
    unsigned byte;
    unsigned bit;
    unsigned w;
    bool drives;
    uint8_t in;
    uint8_t out;
} Esp32SpiFlashSide;

/* The flash's next byte begins: its lines, and whether it drives them */
static void esp32_spi_flash_byte(Esp32SpiFlashSide *f)
{
    unsigned j = f->byte - (f->cont ? 0 : 1);

    f->bit = 0;
    f->in = 0;
    if (f->byte == 0 && !f->cont) {
        f->w = 1;
        f->drives = false;
        return;
    }
    f->w = j < f->fc.pre_bytes ? f->fc.pre_w : f->fc.data_w;
    f->drives = m25p80_drives_output(f->flash);
    if (f->drives) {
        f->out = ssi_transfer(f->bus, 0xff);
    }
}

/*
 * One clock the flash takes, @fin on its pins: returns the lines it drives
 * in *oe and their levels.
 */
static uint8_t esp32_spi_flash_clock(Esp32SpiFlashSide *f, uint8_t fin,
                                     uint8_t *oe)
{
    uint8_t v = 0;
    unsigned shift;

    *oe = 0;
    if (f->bit == 0) {
        esp32_spi_flash_byte(f);
    }
    shift = 8 - (f->bit + 1) * f->w;
    if (f->drives) {
        *oe = esp32_spi_lines(f->w, true);
        v = esp32_spi_spread((f->out >> shift) & MAKE_64BIT_MASK(0, f->w),
                             f->w, 1);
    } else {
        f->in |= esp32_spi_gather(fin, f->w, 0) << shift;
    }
    if (++f->bit * f->w == 8) {
        if (!f->drives) {
            ssi_transfer(f->bus, f->in);
            if (f->byte == 0 && !f->cont) {
                f->fc = esp32_spi_flash_cmd(f->in);
            }
        }
        f->byte++;
        f->bit = 0;
    }
    return v;
}

/* A byte the flash had not finished when CS# rose */
static void esp32_spi_flash_end(Esp32SpiFlashSide *f)
{
    if (f->bit && !f->drives) {
        ssi_transfer(f->bus, f->in | MAKE_64BIT_MASK(0, 8 - f->bit * f->w));
    }
}

/* The controller's drive on clock @c of @t: its lines' levels and enables */
static uint8_t esp32_spi_ctrl_clock(Esp32SpiState *s,
                                    const Esp32SpiTransaction *t,
                                    Esp32SpiStream *d, uint64_t c,
                                    uint8_t *oe, uint8_t *in_lines,
                                    unsigned *in_w)
{
    /* SPIWP and SPIHD, where a phase does not use them */
    uint8_t idle = (FIELD_EX32(s->ctrl_reg, SPI_CTRL, WP) ? 0x4 : 0) | 0x8;
    uint64_t a = esp32_spi_clocks(t->addr_bits, t->addr_w);
    uint64_t o = esp32_spi_clocks(t->out_bits, t->out_w);
    unsigned w, k;
    uint8_t v;

    *in_lines = 0;
    *in_w = 0;
    if (c < t->cmd_bits) {
        unsigned bytes = DIV_ROUND_UP(t->cmd_bits, 8);
        unsigned b = c / 8, i = 7 - c % 8;

        *oe = 0xd;
        return ((b < bytes ? t->cmd >> (8 * b + i) : 0) & 1) | idle;
    }
    c -= t->cmd_bits;
    if (c < a) {
        w = t->addr_w;
        *oe = esp32_spi_lines(w, false) | (w <= 2 ? 0xc : 0);
        return esp32_spi_spread(esp32_spi_left_bits(t->addr, t->addr_bits,
                                                    c, w), w, 0) |
               (w <= 2 ? idle : 0);
    }
    c -= a;
    if (c < t->dummy) {
        w = MAX(t->addr_w, MAX(t->in_w, t->out_w));
        *oe = w <= 2 ? 0xc : 0;
        return w <= 2 ? idle : 0;
    }
    c -= t->dummy;
    if (t->full_duplex) {
        v = idle;
        *oe = 0xc;
        if (c < t->out_bits) {
            if (c % 8 == 0) {
                d->out_byte = esp32_spi_out_next(d);
            }
            v |= (d->out_byte >> (7 - c % 8)) & 1;
            *oe |= 0x1;
        }
        if (c < t->in_bits) {
            *in_lines = 0x2;
            *in_w = 1;
        }
        return v;
    }
    if (c < o) {
        w = t->out_w;
        k = (c * w) % 8;
        if (k == 0) {
            d->out_byte = esp32_spi_out_next(d);
        }
        *oe = esp32_spi_lines(w, false) | (w <= 2 ? 0xc : 0);
        return esp32_spi_spread((d->out_byte >> (8 - k - w)) &
                                MAKE_64BIT_MASK(0, w), w, 0) |
               (w <= 2 ? idle : 0);
    }
    w = t->in_w;
    *in_lines = esp32_spi_lines(w, true);
    *in_w = w;
    *oe = w <= 2 ? 0xc : 0;
    return w <= 2 ? idle : 0;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.flash-chip]
 * Controller @s's transaction @t on CS0, across the lines to the flash on
 * @s->bus's SSI bus, a clock at a time (see the top of the file). The data
 * goes to the controller's buffers, or to @block for a cache fill.
 */
static void esp32_spi_flash_xfer(Esp32SpiState *s,
                                 const Esp32SpiTransaction *t,
                                 uint8_t *block)
{
    Esp32SpiState *bus = s->bus;
    uint64_t n = esp32_spi_xfer_clocks(t);
    uint64_t div = esp32_spi_clock_div(s->clock_reg);
    Esp32SpiFlashLines l;
    Esp32SpiFlashSide f = { .flash = s->flash, .bus = bus->spi };
    Esp32SpiStream d;
    uint8_t hist_oe[4] = { 0 }, hist_v[4] = { 0 };
    unsigned delay[4];
    bool on, hold_en;

    esp32_spi_stream_init(&d, s, t, block);
    s->flash_lines(s->flash_lines_opaque, &l);
    for (unsigned i = 0; i < 4; i++) {
        /* Two APB_CLK cycles through the matrix, in whole SPI clocks */
        delay[i] = (l.in_matrix & BIT(i)) ? 2 / div : 0;
    }
    /* WP# as the command finds it: SPIWP's idle level where it reaches */
    qemu_set_irq(bus->flash_wp,
                 (l.to_flash & 0x4) ?
                 FIELD_EX32(s->ctrl_reg, SPI_CTRL, WP) ^
                 !!(l.flash_inv & 0x4) : !!(l.flash_level & 0x4));
    on = !FIELD_EX32(s->pin_reg, SPI_PIN, CS0_DIS) && l.powered &&
         !esp32_spi_flash_cs(&l, 0) && l.clk;
    hold_en = on && m25p80_hold_enabled(s->flash);
    f.cont = on && m25p80_continuous_cmd(s->flash);
    if (f.cont) {
        f.fc = esp32_spi_flash_cmd(m25p80_continuous_cmd(s->flash));
    }
    if (FIELD_EX32(s->pin_reg, SPI_PIN, CS0_DIS)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: transaction with CS0 "
                      "disabled\n");
    } else if (!on) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: the flash on CS0 is not "
                      "%s: it takes nothing\n",
                      !l.powered ? "powered" :
                      !l.clk ? "clocked, as its pads are set up" :
                      "selected, as its pads are set up");
    }
    esp32_spi_cs_set(s, bus, &l, 0);
    for (uint64_t c = 0; c < n; c++) {
        uint8_t oe, in_lines, foe = 0, fv = 0, cv, fin;
        unsigned in_w;

        cv = esp32_spi_ctrl_clock(s, t, &d, c, &oe, &in_lines, &in_w);
        fin = (((cv ^ l.flash_inv) & oe & l.to_flash) |
               (l.flash_level & ~(oe & l.to_flash))) & 0xf;
        if (on && !(hold_en && !(fin & 0x8))) {
            fv = esp32_spi_flash_clock(&f, fin, &foe);
        }
        hist_oe[c % 4] = foe;
        hist_v[c % 4] = fv;
        if (in_w) {
            uint8_t got = 0;

            for (unsigned i = 0; i < 4; i++) {
                uint64_t src = c - delay[i];
                bool driven = (l.to_ctrl & BIT(i)) && c >= delay[i] &&
                              (hist_oe[src % 4] & BIT(i));
                bool bit = driven ?
                           !!(hist_v[src % 4] & BIT(i)) ^
                           !!(l.ctrl_inv & BIT(i)) :
                           !!(l.ctrl_level & BIT(i));

                got |= bit << i;
            }
            esp32_spi_in_bits(&d, esp32_spi_gather(got & in_lines, in_w, 1),
                              in_w);
        }
    }
    esp32_spi_in_flush(&d);
    if (on) {
        esp32_spi_flash_end(&f);
    }
    if (!FIELD_EX32(s->pin_reg, SPI_PIN, CS_KEEP_ACTIVE)) {
        esp32_spi_cs_set(s, bus, &l, 1);
    }
}

static void maybe_encrypt_data(Esp32SpiState *s)
{
    Esp32FlashEncryptionState *flash_enc = esp32_flash_encryption_find();

    if (esp32_flash_encryption_enabled(flash_enc)) {
        esp32_flash_encryption_get_result(flash_enc, &s->data_reg[0], 8);
    }
}

/* The transaction in flight crosses the lines */
static void esp32_spi_exchange(Esp32SpiState *s)
{
    Esp32SpiTransaction t;

    if (s->exchanged || !s->cmd_reg) {
        return;
    }
    s->exchanged = true;
    if (!esp32_spi_decode(s, s->cmd_reg, &t)) {
        return;
    }
    if (s->cmd_reg == R_SPI_CMD_PP_MASK || s->cmd_reg == R_SPI_CMD_USR_MASK) {
        maybe_encrypt_data(s);
    }
    if (s->flash && !FIELD_EX32(s->pin_reg, SPI_PIN, CS0_DIS)) {
        esp32_spi_flash_xfer(s, &t, NULL);
        if (s->changed) {
            s->changed(s->changed_opaque);
        }
    } else {
        esp32_spi_wires(s, &t);
    }
    if (t.usr && s->dma) {
        esp32_spi_dma_trans_done(s->dma);
    }
}

static void esp32_spi_update_irq(Esp32SpiState *s)
{
    qemu_set_irq(s->irq, FIELD_EX32(s->slave_reg, SPI_SLAVE, TRANS_DONE) &&
                         FIELD_EX32(s->slave_reg, SPI_SLAVE, TRANS_INTEN));
}

/* The transaction ends: its command bit clears, TRANS_DONE sets */
static void esp32_spi_done(void *opaque)
{
    Esp32SpiState *s = opaque;

    if (!s->cmd_reg) {
        return;
    }
    esp32_spi_exchange(s);
    s->cmd_reg = 0;
    s->exchanged = false;
    timer_del(s->done_timer);
    s->slave_reg = FIELD_DP32(s->slave_reg, SPI_SLAVE, TRANS_DONE, 1);
    esp32_spi_update_irq(s);
}

/* Bring a transaction whose end has passed to its end */
static void esp32_spi_catch_up(Esp32SpiState *s)
{
    if (s->cmd_reg && s->done_at != INT64_MAX &&
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->done_at) {
        esp32_spi_done(s);
    }
}

static void esp32_spi_log_unimp(Esp32SpiState *s)
{
    if (s->ctrl_reg & (R_SPI_CTRL_RD_BIT_ORDER_MASK |
                       R_SPI_CTRL_WR_BIT_ORDER_MASK) ||
        s->user_reg & (R_SPI_USER_RD_BYTE_ORDER_MASK |
                       R_SPI_USER_WR_BYTE_ORDER_MASK |
                       R_SPI_USER_DUMMY_IDLE_MASK)) {
        qemu_log_mask(LOG_UNIMP, "esp32_spi: bit and byte orders and the "
                      "idle dummy clock are not modelled\n");
    }
    if (s->ctrl2_reg & (R_SPI_CTRL2_MISO_DELAY_MODE_MASK |
                        R_SPI_CTRL2_MISO_DELAY_NUM_MASK)) {
        qemu_log_mask(LOG_UNIMP, "esp32_spi: MISO delays are not "
                      "modelled\n");
    }
    if (FIELD_EX32(s->slave_reg, SPI_SLAVE, SLAVE_MODE)) {
        qemu_log_mask(LOG_UNIMP, "esp32_spi: slave mode is not modelled\n");
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32.clock-gating]
 * SPI_CMD: start the command, which takes the time its clocks take.
 */
static void esp32_spi_start(Esp32SpiState *s, uint32_t cmd_reg)
{
    Esp32SpiTransaction t;
    int64_t ns;

    esp32_spi_catch_up(s);
    if (s->cmd_reg) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: SPI_CMD written while a "
                      "transaction runs; ignored\n");
        return;
    }
    if (!cmd_reg) {
        return;
    }
    if (!esp32_spi_decode(s, cmd_reg, &t)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: SPI_CMD 0x%08" PRIx32
                      " is not one command\n", cmd_reg);
        return;
    }
    esp32_spi_log_unimp(s);
    ns = esp32_spi_xfer_ns(s, &t);
    s->cmd_reg = cmd_reg;
    s->exchanged = false;
    if (ns == INT64_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_spi: transaction with no "
                      "APB_CLK; it does not end\n");
        s->done_at = INT64_MAX;
        return;
    }
    s->done_at = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns;
    timer_mod(s->done_timer, s->done_at);
}

static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    uint64_t r = 0;

    esp32_spi_catch_up(s);
    switch (addr) {
    case A_SPI_CMD:
        r = s->cmd_reg;
        break;
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
    case A_SPI_CLOCK:
        r = s->clock_reg;
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
    case A_SPI_SLV_WR_STATUS:
        r = s->slv_wr_status_reg;
        break;
    case A_SPI_PIN:
        r = s->pin_reg;
        break;
    case A_SPI_SLAVE:
        r = s->slave_reg;
        break;
    case A_SPI_CACHE_FCTRL:
        r = s->cache_fctrl_reg;
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)];
        break;
    case A_SPI_EXT2:
        /* The state machine: idle, or (the encoding unpublished) busy */
        r = s->cmd_reg ? 1 : 0;
        break;
    }
    return r;
}

/* SPI0's registers that set up the cache's reads */
static bool esp32_spi_cache_setup(hwaddr addr)
{
    switch (addr) {
    case A_SPI_CTRL:
    case A_SPI_CLOCK:
    case A_SPI_USER:
    case A_SPI_USER1:
    case A_SPI_USER2:
    case A_SPI_PIN:
    case A_SPI_CACHE_FCTRL:
        return true;
    }
    return false;
}

static void esp32_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);

    esp32_spi_catch_up(s);
    if (s->changed && s->bus != s && esp32_spi_cache_setup(addr)) {
        s->changed(s->changed_opaque);
    }
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
    case A_SPI_CLOCK:
        s->clock_reg = value & (R_SPI_CLOCK_CLK_EQU_SYSCLK_MASK |
                                R_SPI_CLOCK_CLKDIV_PRE_MASK |
                                R_SPI_CLOCK_CLKCNT_N_MASK |
                                R_SPI_CLOCK_CLKCNT_H_MASK |
                                R_SPI_CLOCK_CLKCNT_L_MASK);
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
    case A_SPI_SLV_WR_STATUS:
        s->slv_wr_status_reg = value;
        break;
    case A_SPI_PIN:
        s->pin_reg = value;
        if (s->flash) {
            esp32_spi_flash_lines_changed(s);
        }
        break;
    case A_SPI_SLAVE:
        s->slave_reg = (s->slave_reg & ESP32_SPI_SLAVE_RO) |
                       (value & ~ESP32_SPI_SLAVE_RO);
        esp32_spi_update_irq(s);
        break;
    case A_SPI_CACHE_FCTRL:
        s->cache_fctrl_reg = value & 0xf;
        if (value & (R_SPI_CACHE_FCTRL_CACHE_USR_CMD_4BYTE_MASK |
                     R_SPI_CACHE_FCTRL_CACHE_FLASH_PES_EN_MASK)) {
            qemu_log_mask(LOG_UNIMP, "esp32_spi: the cache's 4-byte "
                          "addresses and suspend are not modelled\n");
        }
        break;
    case A_SPI_CMD:
        esp32_spi_start(s, value);
        break;
    }
}

/* [spec:nuos:req:emu.esp32.gpio] */
void esp32_spi_set_flash(Esp32SpiState *s, Esp32SpiState *bus,
                         DeviceState *flash, Esp32SpiFlashLinesFunc *fn,
                         void *opaque)
{
    s->bus = bus;
    s->flash = flash;
    s->flash_lines = fn;
    s->flash_lines_opaque = opaque;
    esp32_spi_flash_lines_changed(s);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Between transactions the flash's CS# and WP# follow the lines: the
 * controller's idle SPICS0 and SPIWP where they reach the flash, else the
 * pads' levels, which can hold the flash selected.
 */
void esp32_spi_flash_lines_changed(Esp32SpiState *s)
{
    Esp32SpiFlashLines l;
    Esp32SpiState *bus = s->bus;

    if (!s->flash_lines || bus != s) {
        return;
    }
    s->flash_lines(s->flash_lines_opaque, &l);
    qemu_set_irq(bus->flash_wp,
                 (l.to_flash & 0x4) ?
                 FIELD_EX32(s->ctrl_reg, SPI_CTRL, WP) ^
                 !!(l.flash_inv & 0x4) : !!(l.flash_level & 0x4));
    qemu_set_irq(bus->cs_gpio[0],
                 FIELD_EX32(s->pin_reg, SPI_PIN, CS0_DIS) ? 1 :
                 esp32_spi_flash_cs(&l, 1));
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32.clock-gating]
 * A cache fill: the cache reads a 32-byte block (TRM 3.3.4) with one
 * transaction on SPI0, as SPI0's registers set it up (ESP-IDF's
 * spi_cache_mode_switch() and bootloader_flash_dummy_config()): the
 * command USER2 gives with CACHE_FLASH_USR_CMD, else the one the read mode
 * in CTRL gives; the flash address in USR_ADDR_BITLEN + 1 bits, as its 24
 * bits then zeros (a read's mode bits); USR_DUMMY_CYCLELEN + 1 dummy
 * clocks with USR_DUMMY; 256 bits on the read mode's data lines. SPI1's
 * transaction in flight crosses the lines first, the cache's coming after
 * it ends.
 */
static void esp32_spi_cache_xfer(Esp32SpiState *s, uint32_t addr,
                                 Esp32SpiTransaction *t)
{
    *t = (Esp32SpiTransaction) {
        .cmd_bits = 8,
        .addr = (uint64_t)(addr & 0xffffff) << 40,
        .addr_bits = FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN) + 1,
        .addr_w = esp32_spi_fread_addr_w(s->ctrl_reg),
        .dummy = FIELD_EX32(s->user_reg, SPI_USER, DUMMY) ?
                 FIELD_EX32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN) + 1 : 0,
        .in_bits = ESP32_SPI_CACHE_BLOCK * 8,
        .in_w = esp32_spi_fread_data_w(s->ctrl_reg),
        .out_w = 1,
    };
    if (FIELD_EX32(s->cache_fctrl_reg, SPI_CACHE_FCTRL, CACHE_FLASH_USR_CMD)) {
        t->cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
        t->cmd_bits = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN) + 1;
    } else {
        t->cmd = esp32_spi_fread_cmd(s->ctrl_reg);
    }
}

/* [spec:nuos:req:emu.esp32.clock-gating] */
int64_t esp32_spi_flash_cache_ns(Esp32SpiState *s)
{
    Esp32SpiTransaction t;

    esp32_spi_cache_xfer(s, 0, &t);
    return esp32_spi_xfer_ns(s, &t);
}

int64_t esp32_spi_flash_cache_read(Esp32SpiState *s, uint32_t addr,
                                   uint8_t *buf)
{
    Esp32SpiTransaction t;
    int64_t wait = 0, now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t ns;

    esp32_spi_cache_xfer(s, addr, &t);
    if (s->bus && s->bus->cmd_reg) {
        esp32_spi_exchange(s->bus);
        if (s->bus->done_at != INT64_MAX && s->bus->done_at > now) {
            wait = s->bus->done_at - now;
        }
    }
    memset(buf, 0, ESP32_SPI_CACHE_BLOCK);
    if (s->flash && s->flash_lines) {
        esp32_spi_flash_xfer(s, &t, buf);
    }
    ns = esp32_spi_xfer_ns(s, &t);
    return ns == INT64_MAX ? ns : wait + ns;
}

static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Reset values from ESP-IDF's soc/esp32/register/soc/spi_reg.h */
static void esp32_spi_reset_hold(Object *obj, ResetType type)
{
    Esp32SpiState *s = ESP32_SPI(obj);

    timer_del(s->done_timer);
    s->cmd_reg = 0;
    s->exchanged = false;
    s->done_at = 0;
    s->addr_reg = 0;
    s->ctrl_reg = R_SPI_CTRL_WP_MASK | R_SPI_CTRL_RESANDRES_MASK |
                  R_SPI_CTRL_FASTRD_MODE_MASK | BIT(10);
    s->ctrl1_reg = 0x5fff0000;
    s->ctrl2_reg = FIELD_DP32(0, SPI_CTRL2, SETUP_TIME, 1);
    s->ctrl2_reg = FIELD_DP32(s->ctrl2_reg, SPI_CTRL2, HOLD_TIME, 1);
    s->clock_reg = R_SPI_CLOCK_CLK_EQU_SYSCLK_MASK |
                   FIELD_DP32(0, SPI_CLOCK, CLKCNT_N, 3) |
                   FIELD_DP32(0, SPI_CLOCK, CLKCNT_H, 1) |
                   FIELD_DP32(0, SPI_CLOCK, CLKCNT_L, 3);
    s->status_reg = 0;
    s->user_reg = R_SPI_USER_COMMAND_MASK | R_SPI_USER_CK_I_EDGE_MASK;
    s->user1_reg = FIELD_DP32(0, SPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, SPI_USER2, COMMAND_BITLEN, 7);
    s->mosi_dlen_reg = 0;
    s->miso_dlen_reg = 0;
    s->slv_wr_status_reg = 0;
    s->pin_reg = 0x6;
    s->slave_reg = R_SPI_SLAVE_TRANS_INTEN_MASK;
    s->cache_fctrl_reg = 0;
    memset(s->data_reg, 0, sizeof(s->data_reg));
    esp32_spi_update_irq(s);
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
    s->bus = s;
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI_CS_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), &s->flash_wp, ESP32_SPI_FLASH_WP, 1);
    s->apb_clk = qdev_init_clock_in(DEVICE(s), "apb", NULL, NULL, 0);
    s->done_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32_spi_done, s);
}

static const Property esp32_spi_properties[] = {
    DEFINE_PROP_LINK("dma", Esp32SpiState, dma, TYPE_ESP32_SPI_DMA,
                     Esp32SpiDmaState *),
};

static const VMStateDescription vmstate_esp32_spi = {
    .name = TYPE_ESP32_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(apb_clk, Esp32SpiState),
        VMSTATE_TIMER_PTR(done_timer, Esp32SpiState),
        VMSTATE_UINT32(cmd_reg, Esp32SpiState),
        VMSTATE_INT64(done_at, Esp32SpiState),
        VMSTATE_BOOL(exchanged, Esp32SpiState),
        VMSTATE_UINT32(addr_reg, Esp32SpiState),
        VMSTATE_UINT32(ctrl_reg, Esp32SpiState),
        VMSTATE_UINT32(status_reg, Esp32SpiState),
        VMSTATE_UINT32(ctrl1_reg, Esp32SpiState),
        VMSTATE_UINT32(ctrl2_reg, Esp32SpiState),
        VMSTATE_UINT32(clock_reg, Esp32SpiState),
        VMSTATE_UINT32(user_reg, Esp32SpiState),
        VMSTATE_UINT32(user1_reg, Esp32SpiState),
        VMSTATE_UINT32(user2_reg, Esp32SpiState),
        VMSTATE_UINT32(mosi_dlen_reg, Esp32SpiState),
        VMSTATE_UINT32(miso_dlen_reg, Esp32SpiState),
        VMSTATE_UINT32(slv_wr_status_reg, Esp32SpiState),
        VMSTATE_UINT32(pin_reg, Esp32SpiState),
        VMSTATE_UINT32(slave_reg, Esp32SpiState),
        VMSTATE_UINT32(cache_fctrl_reg, Esp32SpiState),
        VMSTATE_UINT32_ARRAY(data_reg, Esp32SpiState, ESP32_SPI_BUF_WORDS),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_spi_reset_hold;
    dc->realize = esp32_spi_realize;
    dc->vmsd = &vmstate_esp32_spi;
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
