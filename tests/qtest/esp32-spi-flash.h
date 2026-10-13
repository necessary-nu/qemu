/*
 * SPI0 and SPI1 of the ESP32, the controllers on the module's flash, for
 * the qtests
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A transaction takes its SPI clocks in virtual time: SPI_CMD's bit stays
 * set until it ends, and its data is in the buffer only then, so a qtest
 * steps the clock until the bit clears. The flash takes the time its
 * datasheet gives to program, erase and write its status registers,
 * reading busy meanwhile.
 */

#ifndef ESP32_SPI_FLASH_H
#define ESP32_SPI_FLASH_H

#define SF_SPI0                 0x3ff43000
#define SF_SPI1                 0x3ff42000
#define SF_CMD                  0x00
#define SF_ADDR                 0x04
#define SF_CTRL                 0x08
#define SF_RD_STATUS            0x10
#define SF_CTRL2                0x14
#define SF_CLOCK                0x18
#define SF_USER                 0x1c
#define SF_USER1                0x20
#define SF_USER2                0x24
#define SF_MOSI_DLEN            0x28
#define SF_MISO_DLEN            0x2c
#define SF_PIN                  0x34
#define SF_SLAVE                0x38
#define SF_CACHE_FCTRL          0x50
#define SF_W0                   0x80
#define SF_EXT2                 0xf8

#define SF_CMD_READ             (1u << 31)
#define SF_CMD_WREN             (1u << 30)
#define SF_CMD_WRDI             (1u << 29)
#define SF_CMD_RDID             (1u << 28)
#define SF_CMD_RDSR             (1u << 27)
#define SF_CMD_WRSR             (1u << 26)
#define SF_CMD_PP               (1u << 25)
#define SF_CMD_SE               (1u << 24)
#define SF_CMD_BE               (1u << 23)
#define SF_CMD_CE               (1u << 22)
#define SF_CMD_DP               (1u << 21)
#define SF_CMD_RES              (1u << 20)
#define SF_CMD_HPM              (1u << 19)
#define SF_CMD_USR              (1u << 18)

#define SF_CTRL_FASTRD          (1u << 13)
#define SF_CTRL_FREAD_DUAL      (1u << 14)
#define SF_CTRL_RESANDRES       (1u << 15)
#define SF_CTRL_FREAD_QUAD      (1u << 20)
#define SF_CTRL_WP              (1u << 21)
#define SF_CTRL_WRSR_2B         (1u << 22)
#define SF_CTRL_FREAD_DIO       (1u << 23)
#define SF_CTRL_FREAD_QIO       (1u << 24)

#define SF_USER_COMMAND         (1u << 31)
#define SF_USER_ADDR            (1u << 30)
#define SF_USER_DUMMY           (1u << 29)
#define SF_USER_MISO            (1u << 28)
#define SF_USER_MOSI            (1u << 27)
#define SF_USER_CS_SETUP        (1u << 5)
#define SF_USER_CS_HOLD         (1u << 4)
#define SF_USER1_VAL(addr_bits, dummy) \
    ((uint32_t)((addr_bits) - 1) << 26 | ((dummy) - 1))

#define SF_SR1_WIP              0x01
#define SF_SR1_WEL              0x02

/* Step the clock until the controller's transaction ends */
static inline void sf_wait(QTestState *qts, uint32_t spi)
{
    for (int i = 0; i < 1000000 && qtest_readl(qts, spi + SF_CMD); i++) {
        qtest_clock_step(qts, 50);
    }
    g_assert_cmphex(qtest_readl(qts, spi + SF_CMD), ==, 0);
}

/* A command on SPI1, run to its end */
static inline void sf_cmd(QTestState *qts, uint32_t cmd)
{
    qtest_writel(qts, SF_SPI1 + SF_CMD, cmd);
    sf_wait(qts, SF_SPI1);
}

static inline uint8_t sf_rdsr(QTestState *qts)
{
    qtest_writel(qts, SF_SPI1 + SF_RD_STATUS, 0);
    sf_cmd(qts, SF_CMD_RDSR);
    return qtest_readl(qts, SF_SPI1 + SF_RD_STATUS);
}

/* Step the clock until the flash's cycle ends (WIP clears) */
static inline void sf_wait_idle(QTestState *qts)
{
    for (int i = 0; i < 100000 && (sf_rdsr(qts) & SF_SR1_WIP); i++) {
        qtest_clock_step(qts, 100000);
    }
    g_assert_cmphex(sf_rdsr(qts) & SF_SR1_WIP, ==, 0);
}

/*
 * A user command on SPI1, as ESP-IDF's bootloader sends it: the command
 * byte, @mosi_bits of @mosi, @miso_bits back from W0; CTRL cleared to the
 * one-line mode first, and restored after.
 */
static inline uint32_t sf_usr(QTestState *qts, uint8_t cmd, uint32_t mosi,
                              unsigned mosi_bits, unsigned miso_bits)
{
    uint32_t ctrl = qtest_readl(qts, SF_SPI1 + SF_CTRL);
    uint32_t user = SF_USER_COMMAND;
    uint32_t r;

    qtest_writel(qts, SF_SPI1 + SF_CTRL, SF_CTRL_WP);
    if (mosi_bits) {
        user |= SF_USER_MOSI;
        qtest_writel(qts, SF_SPI1 + SF_MOSI_DLEN, mosi_bits - 1);
        qtest_writel(qts, SF_SPI1 + SF_W0, mosi);
    }
    if (miso_bits) {
        user |= SF_USER_MISO;
        qtest_writel(qts, SF_SPI1 + SF_MISO_DLEN, miso_bits - 1);
    }
    qtest_writel(qts, SF_SPI1 + SF_USER, user);
    qtest_writel(qts, SF_SPI1 + SF_USER2, (7u << 28) | cmd);
    sf_cmd(qts, SF_CMD_USR);
    r = qtest_readl(qts, SF_SPI1 + SF_W0);
    qtest_writel(qts, SF_SPI1 + SF_CTRL, ctrl);
    return miso_bits < 32 ? r & ((1u << miso_bits) - 1) : r;
}

/*
 * SPI1's read mode, configured as ESP-IDF's esp_rom_spiflash_read() does,
 * with @plus extra dummy clocks (its g_rom_spiflash_dummy_len_plus).
 */
enum { SF_SLOW, SF_FAST, SF_DOUT, SF_DIO, SF_QOUT, SF_QIO };

static inline void sf_read_mode(QTestState *qts, int mode, unsigned plus)
{
    static const uint32_t ctrl[] = {
        [SF_SLOW] = 0,
        [SF_FAST] = SF_CTRL_FASTRD,
        [SF_DOUT] = SF_CTRL_FASTRD | SF_CTRL_FREAD_DUAL,
        [SF_DIO] = SF_CTRL_FASTRD | SF_CTRL_FREAD_DIO,
        [SF_QOUT] = SF_CTRL_FASTRD | SF_CTRL_FREAD_QUAD,
        [SF_QIO] = SF_CTRL_FASTRD | SF_CTRL_FREAD_QIO,
    };
    uint32_t user = SF_USER_COMMAND | SF_USER_ADDR | SF_USER_MISO;
    uint32_t user1;

    switch (mode) {
    case SF_QIO:
        user |= SF_USER_DUMMY;
        user1 = SF_USER1_VAL(32, 4 + plus);
        break;
    case SF_DIO:
        if (plus) {
            user |= SF_USER_DUMMY;
        }
        user1 = SF_USER1_VAL(32, plus ? plus : 1);
        break;
    case SF_SLOW:
        if (plus) {
            user |= SF_USER_DUMMY;
        }
        user1 = SF_USER1_VAL(24, plus ? plus : 1);
        break;
    default:
        user |= SF_USER_DUMMY;
        user1 = SF_USER1_VAL(24, 8 + plus);
        break;
    }
    qtest_writel(qts, SF_SPI1 + SF_CTRL,
                 SF_CTRL_WP | SF_CTRL_RESANDRES | ctrl[mode]);
    qtest_writel(qts, SF_SPI1 + SF_USER, user);
    qtest_writel(qts, SF_SPI1 + SF_USER1, user1);
}

/* SPI1's READ of the four bytes at @a, as esp_rom_spiflash_read_data() */
static inline uint32_t sf_read_word(QTestState *qts, uint32_t a)
{
    qtest_writel(qts, SF_SPI1 + SF_MISO_DLEN, 31);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, a << 8);
    sf_cmd(qts, SF_CMD_READ);
    return qtest_readl(qts, SF_SPI1 + SF_W0);
}

/*
 * SPI0 for the cache as the ROM leaves it, in DIO mode (its own setup:
 * BBh, a 24-bit address and four dummy clocks, at APB_CLK / 2).
 */
static inline void sf_cache_rom_dio(QTestState *qts)
{
    qtest_writel(qts, SF_SPI0 + SF_CLOCK, 0x1001);
    qtest_writel(qts, SF_SPI0 + SF_CTRL, SF_CTRL_WP | SF_CTRL_FASTRD |
                 SF_CTRL_FREAD_DIO);
    qtest_writel(qts, SF_SPI0 + SF_USER, SF_USER_ADDR | SF_USER_DUMMY |
                 SF_USER_MISO);
    qtest_writel(qts, SF_SPI0 + SF_USER1, SF_USER1_VAL(24, 4));
    qtest_writel(qts, SF_SPI0 + SF_USER2, (7u << 28) | 0xbb);
    qtest_writel(qts, SF_SPI0 + SF_CACHE_FCTRL, 1);
}

#endif
