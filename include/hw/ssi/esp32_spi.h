#pragma once

#include "hw/core/registerfields.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi_dma.h"

#define TYPE_ESP32_SPI "ssi.esp32.spi"
#define ESP32_SPI(obj) OBJECT_CHECK(Esp32SpiState, (obj), TYPE_ESP32_SPI)

#define ESP32_SPI_CS_COUNT      3
#define ESP32_SPI_BUF_WORDS     16

/*
 * The lines between SPI0 and SPI1, which share them, and the chip on
 * their CS0, the module's flash, as the pads join them (the machine says,
 * esp32_spi_set_flash_lines()): IOn is bit n of the masks, IO0 SPID (the
 * flash's DI), IO1 SPIQ (DO), IO2 SPIWP (WP#), IO3 SPIHD (HOLD#).
 *
 * - clk: SPICLK reaches the flash's CLK;
 * - cs: SPICS0 reaches the flash's CS#, inverted with cs_inv; where it
 *   does not, the flash's CS# is at cs_level;
 * - hold: the flash's HOLD# holds it, paused, its outputs off;
 * - wp_level: the flash's WP# while the controller is idle;
 * - to_flash: the IOn lines on which the controller's output reaches the
 *   flash, inverted on flash_inv's; elsewhere the flash's pin is at
 *   flash_level;
 * - to_ctrl: the IOn lines on which the flash's output reaches the
 *   controller's input, inverted on ctrl_inv's; ctrl_level, what each
 *   input reads while the flash does not drive it.
 */
typedef struct Esp32SpiFlashLines {
    bool clk;
    bool cs;
    bool cs_inv;
    bool cs_level;
    bool hold;
    bool wp_level;
    uint8_t to_flash;
    uint8_t flash_inv;
    uint8_t flash_level;
    uint8_t to_ctrl;
    uint8_t ctrl_inv;
    uint8_t ctrl_level;
} Esp32SpiFlashLines;

typedef void Esp32SpiFlashLinesFunc(void *opaque, Esp32SpiFlashLines *l);

/* The flash's WP#, which the controller drives at each transaction */
#define ESP32_SPI_FLASH_WP      "flash-wp"

typedef struct Esp32SpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs_gpio[ESP32_SPI_CS_COUNT];
    int num_cs;
    SSIBus *spi;
    /* The DMA engine of SPI1..SPI3; NULL for SPI0 */
    Esp32SpiDmaState *dma;
    /* SPI0 and SPI1: the lines to the flash on CS0; NULL, wires */
    Esp32SpiFlashLinesFunc *flash_lines;
    void *flash_lines_opaque;
    qemu_irq flash_wp;

    uint32_t addr_reg;
    uint32_t ctrl_reg;
    uint32_t status_reg;
    uint32_t ctrl1_reg;
    uint32_t ctrl2_reg;
    uint32_t user_reg;
    uint32_t user1_reg;
    uint32_t user2_reg;
    uint32_t mosi_dlen_reg;
    uint32_t miso_dlen_reg;
    uint32_t pin_reg;
    uint32_t data_reg[ESP32_SPI_BUF_WORDS];
} Esp32SpiState;


REG32(SPI_CMD, 0x00)
    FIELD(SPI_CMD, READ, 31, 1)
    FIELD(SPI_CMD, WREN, 30, 1)
    FIELD(SPI_CMD, WRDI, 29, 1)
    FIELD(SPI_CMD, RDID, 28, 1)
    FIELD(SPI_CMD, RDSR, 27, 1)
    FIELD(SPI_CMD, WRSR, 26, 1)
    FIELD(SPI_CMD, PP, 25, 1)
    FIELD(SPI_CMD, SE, 24, 1)
    FIELD(SPI_CMD, BE, 23, 1)
    FIELD(SPI_CMD, CE, 22, 1)
    FIELD(SPI_CMD, DP, 21, 1)
    FIELD(SPI_CMD, RES, 20, 1)
    FIELD(SPI_CMD, HPM, 19, 1)
    FIELD(SPI_CMD, USR, 18, 1)
    FIELD(SPI_CMD, PES, 17, 1)
    FIELD(SPI_CMD, PER, 16, 1)

REG32(SPI_ADDR, 0x04)
REG32(SPI_CTRL, 0x08)
    FIELD(SPI_CTRL, FASTRD_MODE, 13, 1)
    FIELD(SPI_CTRL, FREAD_DUAL, 14, 1)
    FIELD(SPI_CTRL, FREAD_QUAD, 20, 1)
    FIELD(SPI_CTRL, WP, 21, 1)
    FIELD(SPI_CTRL, FREAD_DIO, 23, 1)
    FIELD(SPI_CTRL, FREAD_QIO, 24, 1)
REG32(SPI_STATUS, 0x10)
    FIELD(SPI_STATUS, STATUS, 0, 16)

REG32(SPI_CTRL1, 0x0c)
REG32(SPI_CTRL2, 0x14)
REG32(SPI_USER, 0x1C)
    FIELD(SPI_USER, COMMAND, 31, 1)
    FIELD(SPI_USER, ADDR, 30, 1)
    FIELD(SPI_USER, DUMMY, 29, 1)
    FIELD(SPI_USER, MISO, 28, 1)
    FIELD(SPI_USER, MOSI, 27, 1)
    FIELD(SPI_USER, FWRITE_DUAL, 12, 1)
    FIELD(SPI_USER, FWRITE_QUAD, 13, 1)
    FIELD(SPI_USER, FWRITE_DIO, 14, 1)
    FIELD(SPI_USER, FWRITE_QIO, 15, 1)
    FIELD(SPI_USER, SIO, 16, 1)
    FIELD(SPI_USER, DOUTDIN, 0, 1)

REG32(SPI_USER1, 0x20)
    FIELD(SPI_USER1, ADDR_BITLEN, 26, 6)
    FIELD(SPI_USER1, DUMMY_CYCLELEN, 0, 8)

REG32(SPI_USER2, 0x24)
    FIELD(SPI_USER2, COMMAND_BITLEN, 28, 4)
    FIELD(SPI_USER2, COMMAND_VALUE, 0, 16)

REG32(SPI_MOSI_DLEN, 0x28)
REG32(SPI_MISO_DLEN, 0x2c)
REG32(SPI_PIN, 0x34)
    FIELD(SPI_PIN, CS0_DIS, 0, 1)
REG32(SPI_SLAVE, 0x38)
    FIELD(SPI_SLAVE, TRANS_DONE, 4, 1)
    FIELD(SPI_SLAVE, TRANS_INTEN, 9, 1)
    FIELD(SPI_SLAVE, TRANS_CNT, 22, 4)
    FIELD(SPI_SLAVE, SLAVE_MODE, 30, 1)

REG32(SPI_W0, 0x80)
REG32(SPI_EXT0, 0xF0)
REG32(SPI_EXT1, 0xF4)
REG32(SPI_EXT2, 0xF8)
REG32(SPI_EXT3, 0xFC)

/*
 * The machine joins SPI0 or SPI1 to the flash on CS0 as @fn says; it
 * calls esp32_spi_flash_lines_changed() when what @fn would say changes.
 */
void esp32_spi_set_flash_lines(Esp32SpiState *s, Esp32SpiFlashLinesFunc *fn,
                               void *opaque);
void esp32_spi_flash_lines_changed(Esp32SpiState *s);

/*
 * A cache fill through SPI0 @s: @len bytes at flash address @addr of the
 * flash's array @blk, as SPI0's read mode and the lines bring them.
 */
void esp32_spi_flash_cache_read(Esp32SpiState *s, BlockBackend *blk,
                                uint32_t addr, uint8_t *buf, uint32_t len);
