#pragma once

#include "qemu/timer.h"
#include "hw/core/clock.h"
#include "hw/core/registerfields.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi_dma.h"

#define TYPE_ESP32_SPI "ssi.esp32.spi"
#define ESP32_SPI(obj) OBJECT_CHECK(Esp32SpiState, (obj), TYPE_ESP32_SPI)

#define ESP32_SPI_CS_COUNT      3
#define ESP32_SPI_BUF_WORDS     16

/* A cache block, which a cache miss reads from the flash (TRM 3.3.4) */
#define ESP32_SPI_CACHE_BLOCK   32

/*
 * The lines between SPI0 and SPI1, which share them, and the chip on
 * their CS0, the module's flash, as the pads join them (the machine says,
 * esp32_spi_set_flash()): IOn is bit n of the masks, IO0 SPID (the
 * flash's DI), IO1 SPIQ (DO), IO2 SPIWP (WP#), IO3 SPIHD (HOLD#).
 *
 * - clk: SPICLK reaches the flash's CLK;
 * - cs: SPICS0 reaches the flash's CS#, inverted with cs_inv; where it
 *   does not, the flash's CS# is at cs_level;
 * - powered: the flash's supply is in its operating range; out of it the
 *   flash takes and drives nothing;
 * - to_flash: the IOn lines on which the controller's output reaches the
 *   flash, inverted on flash_inv's; while the controller does not drive a
 *   line (it does not reach, or its output is off), the flash's pin is at
 *   flash_level;
 * - to_ctrl: the IOn lines on which the flash's output reaches the
 *   controller's input, inverted on ctrl_inv's, and in_matrix those of
 *   them that go through the GPIO matrix; ctrl_level, what each input
 *   reads while the flash does not drive it.
 */
typedef struct Esp32SpiFlashLines {
    bool clk;
    bool cs;
    bool cs_inv;
    bool cs_level;
    bool powered;
    uint8_t to_flash;
    uint8_t flash_inv;
    uint8_t flash_level;
    uint8_t to_ctrl;
    uint8_t ctrl_inv;
    uint8_t ctrl_level;
    uint8_t in_matrix;
} Esp32SpiFlashLines;

typedef void Esp32SpiFlashLinesFunc(void *opaque, Esp32SpiFlashLines *l);

/* What a cache fill reads of the flash may have changed */
typedef void Esp32SpiChangedFunc(void *opaque);

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
    Clock *apb_clk;
    /* A transaction's end */
    QEMUTimer *done_timer;

    /*
     * SPI0 and SPI1: the flash on CS0, the controller whose SSI bus and
     * CS0 line it is on (SPI1: the two share the lines), and how the pads
     * join them; NULL, wires.
     */
    DeviceState *flash;
    struct Esp32SpiState *bus;
    Esp32SpiFlashLinesFunc *flash_lines;
    void *flash_lines_opaque;
    qemu_irq flash_wp;
    /*
     * Called after a transaction on CS0, and when SPI0's set-up for the
     * cache changes
     */
    Esp32SpiChangedFunc *changed;
    void *changed_opaque;

    /* The command in flight (SPI_CMD's bits) and when it ends */
    uint32_t cmd_reg;
    int64_t done_at;
    /* Its bits have been exchanged with the chips already */
    bool exchanged;

    uint32_t addr_reg;
    uint32_t ctrl_reg;
    uint32_t status_reg;
    uint32_t ctrl1_reg;
    uint32_t ctrl2_reg;
    uint32_t clock_reg;
    uint32_t user_reg;
    uint32_t user1_reg;
    uint32_t user2_reg;
    uint32_t mosi_dlen_reg;
    uint32_t miso_dlen_reg;
    uint32_t slv_wr_status_reg;
    uint32_t pin_reg;
    uint32_t slave_reg;
    uint32_t cache_fctrl_reg;
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
    FIELD(SPI_CTRL, RESANDRES, 15, 1)
    FIELD(SPI_CTRL, FREAD_QUAD, 20, 1)
    FIELD(SPI_CTRL, WP, 21, 1)
    FIELD(SPI_CTRL, WRSR_2B, 22, 1)
    FIELD(SPI_CTRL, FREAD_DIO, 23, 1)
    FIELD(SPI_CTRL, FREAD_QIO, 24, 1)
    FIELD(SPI_CTRL, RD_BIT_ORDER, 25, 1)
    FIELD(SPI_CTRL, WR_BIT_ORDER, 26, 1)
REG32(SPI_CTRL1, 0x0c)
REG32(SPI_STATUS, 0x10)
    FIELD(SPI_STATUS, STATUS, 0, 16)
REG32(SPI_CTRL2, 0x14)
    FIELD(SPI_CTRL2, SETUP_TIME, 0, 4)
    FIELD(SPI_CTRL2, HOLD_TIME, 4, 4)
    FIELD(SPI_CTRL2, MISO_DELAY_MODE, 16, 2)
    FIELD(SPI_CTRL2, MISO_DELAY_NUM, 18, 3)
REG32(SPI_CLOCK, 0x18)
    FIELD(SPI_CLOCK, CLKCNT_L, 0, 6)
    FIELD(SPI_CLOCK, CLKCNT_H, 6, 6)
    FIELD(SPI_CLOCK, CLKCNT_N, 12, 6)
    FIELD(SPI_CLOCK, CLKDIV_PRE, 18, 13)
    FIELD(SPI_CLOCK, CLK_EQU_SYSCLK, 31, 1)
REG32(SPI_USER, 0x1C)
    FIELD(SPI_USER, COMMAND, 31, 1)
    FIELD(SPI_USER, ADDR, 30, 1)
    FIELD(SPI_USER, DUMMY, 29, 1)
    FIELD(SPI_USER, MISO, 28, 1)
    FIELD(SPI_USER, MOSI, 27, 1)
    FIELD(SPI_USER, DUMMY_IDLE, 26, 1)
    FIELD(SPI_USER, MOSI_HIGHPART, 25, 1)
    FIELD(SPI_USER, MISO_HIGHPART, 24, 1)
    FIELD(SPI_USER, SIO, 16, 1)
    FIELD(SPI_USER, FWRITE_QIO, 15, 1)
    FIELD(SPI_USER, FWRITE_DIO, 14, 1)
    FIELD(SPI_USER, FWRITE_QUAD, 13, 1)
    FIELD(SPI_USER, FWRITE_DUAL, 12, 1)
    FIELD(SPI_USER, WR_BYTE_ORDER, 11, 1)
    FIELD(SPI_USER, RD_BYTE_ORDER, 10, 1)
    FIELD(SPI_USER, CK_OUT_EDGE, 7, 1)
    FIELD(SPI_USER, CK_I_EDGE, 6, 1)
    FIELD(SPI_USER, CS_SETUP, 5, 1)
    FIELD(SPI_USER, CS_HOLD, 4, 1)
    FIELD(SPI_USER, DOUTDIN, 0, 1)

REG32(SPI_USER1, 0x20)
    FIELD(SPI_USER1, ADDR_BITLEN, 26, 6)
    FIELD(SPI_USER1, DUMMY_CYCLELEN, 0, 8)

REG32(SPI_USER2, 0x24)
    FIELD(SPI_USER2, COMMAND_BITLEN, 28, 4)
    FIELD(SPI_USER2, COMMAND_VALUE, 0, 16)

REG32(SPI_MOSI_DLEN, 0x28)
    FIELD(SPI_MOSI_DLEN, DBITLEN, 0, 24)
REG32(SPI_MISO_DLEN, 0x2c)
    FIELD(SPI_MISO_DLEN, DBITLEN, 0, 24)
REG32(SPI_SLV_WR_STATUS, 0x30)
REG32(SPI_PIN, 0x34)
    FIELD(SPI_PIN, CS0_DIS, 0, 1)
    FIELD(SPI_PIN, CS_KEEP_ACTIVE, 30, 1)
REG32(SPI_SLAVE, 0x38)
    FIELD(SPI_SLAVE, TRANS_DONE, 4, 1)
    FIELD(SPI_SLAVE, TRANS_INTEN, 9, 1)
    FIELD(SPI_SLAVE, TRANS_CNT, 23, 4)
    FIELD(SPI_SLAVE, SLAVE_MODE, 30, 1)
REG32(SPI_CACHE_FCTRL, 0x50)
    FIELD(SPI_CACHE_FCTRL, CACHE_REQ_EN, 0, 1)
    FIELD(SPI_CACHE_FCTRL, CACHE_USR_CMD_4BYTE, 1, 1)
    FIELD(SPI_CACHE_FCTRL, CACHE_FLASH_USR_CMD, 2, 1)
    FIELD(SPI_CACHE_FCTRL, CACHE_FLASH_PES_EN, 3, 1)

REG32(SPI_W0, 0x80)
REG32(SPI_EXT0, 0xF0)
REG32(SPI_EXT1, 0xF4)
REG32(SPI_EXT2, 0xF8)
    FIELD(SPI_EXT2, ST, 0, 3)
REG32(SPI_EXT3, 0xFC)

/*
 * The machine puts @flash on CS0 of SPI0 or SPI1 @s, on the SSI bus and
 * CS0 line of @bus (SPI1), joined as @fn says; it calls
 * esp32_spi_flash_lines_changed() when what @fn would say changes.
 */
void esp32_spi_set_flash(Esp32SpiState *s, Esp32SpiState *bus,
                         DeviceState *flash, Esp32SpiFlashLinesFunc *fn,
                         void *opaque);
void esp32_spi_flash_lines_changed(Esp32SpiState *s);

/*
 * A cache fill through SPI0 @s: the 32-byte block at flash address @addr,
 * read as SPI0's read mode and the lines bring it. Returns the time the
 * transaction takes, in ns.
 */
int64_t esp32_spi_flash_cache_read(Esp32SpiState *s, uint32_t addr,
                                   uint8_t *buf);

/* The time such a fill takes, as SPI0 is set up now, in ns */
int64_t esp32_spi_flash_cache_ns(Esp32SpiState *s);
