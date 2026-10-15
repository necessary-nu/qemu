/*
 * QTest testcase for the ESP32-P4 flash MSPI's SPI0 reads, as its
 * registers put them on the board's GD25Q256E
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The flash is a raw image of the 32 MiB part, erased but for a pattern in
 * its first page at LOW. qtest's reads of the direct window are SPI0's
 * reads, one transaction each. Out of reset, no pull holds the flash's
 * data lines: a line nothing drives reads 0.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define FLASH_SIZE          (32 * 1024 * 1024)
#define LOW                 0x10000u
#define SPI0_FLASH          0x5008c000u
#define FLASH_DIRECT        0x80000000u
#define PAGE                0x10000u

#define CTRL                0x008
#define USER                0x018
#define USER1               0x01c
#define USER2               0x020
#define RD_STATUS           0x02c
#define CACHE_FCTRL         0x03c
#define MMU_ITEM_CONTENT    0x37c
#define MMU_ITEM_INDEX      0x380

#define CTRL_DIO            0x00ac200cu
#define USER_USR_DUMMY      (1u << 29)
#define FCTRL_AXI_REQ_EN    (1u << 0)
#define FCTRL_ADDR_4BYTE    (1u << 1)
#define FCTRL_USR_CMD       (1u << 2)
#define FLASH_VALID         (1u << 12)

static uint8_t pat(uint32_t a)
{
    return (a * 7 + (a >> 8) * 13) ^ 0x5a;
}

static uint32_t pat_word(uint32_t a)
{
    return pat(a) | pat(a + 1) << 8 | pat(a + 2) << 16 |
           (uint32_t)pat(a + 3) << 24;
}

static char *flash_image(void)
{
    g_autofree uint8_t *img = g_malloc(FLASH_SIZE);
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("esp32p4-spi0-XXXXXX.bin", &path, &err);

    g_assert_no_error(err);
    memset(img, 0xff, FLASH_SIZE);
    for (uint32_t i = 0; i < PAGE; i++) {
        img[LOW + i] = pat(LOW + i);
    }
    g_assert_cmpint(write(fd, img, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);
    return path;
}

/*
 * The DIO set-up: BBh or BCh from USER2, `addr_bits`, `dummy` cycles (or
 * no dummy phase).
 */
static void spi0_dio(QTestState *qts, uint8_t cmd, int addr_bits, int dummy,
                     bool four)
{
    qtest_writel(qts, SPI0_FLASH + CTRL, CTRL_DIO);
    qtest_writel(qts, SPI0_FLASH + USER, (dummy ? USER_USR_DUMMY : 0) | 0xc0);
    qtest_writel(qts, SPI0_FLASH + USER1, (uint32_t)(addr_bits - 1) << 26 |
                 (uint32_t)(dummy ? dummy - 1 : 0));
    qtest_writel(qts, SPI0_FLASH + USER2, (7u << 28) | cmd);
    qtest_writel(qts, SPI0_FLASH + RD_STATUS, 0);
    qtest_writel(qts, SPI0_FLASH + CACHE_FCTRL, FCTRL_AXI_REQ_EN |
                 FCTRL_USR_CMD | (four ? FCTRL_ADDR_4BYTE : 0));
}

static QTestState *start(char **image, char **log)
{
    QTestState *qts;
    int fd = g_file_open_tmp("esp32p4-spi0-XXXXXX.log", log, NULL);

    close(fd);
    *image = flash_image();
    qts = qtest_initf("-M esp32p4,flash-size=32M,strap-mode=0 "
                      "-drive if=mtd,format=raw,file=%s "
                      "-d guest_errors -D %s", *image, *log);
    qtest_writel(qts, SPI0_FLASH + MMU_ITEM_INDEX, 1);
    qtest_writel(qts, SPI0_FLASH + MMU_ITEM_CONTENT,
                 FLASH_VALID | (LOW / PAGE));
    return qts;
}

static void finish(QTestState *qts, char *image, char *log)
{
    qtest_quit(qts);
    unlink(image);
    unlink(log);
    g_free(image);
    g_free(log);
}

/*
 * [spec:nuos:req:emu.esp32p4.memory/test]
 * The ROM's DIO set-up reads the flash: BBh, 28 address bits (the address
 * and WB_MODE's M7-M4) and 2 dummy cycles; its 32-bit form, BCh with
 * CACHE_USR_ADDR_4BYTE and 4 dummy cycles, too. Out of reset SPI0 sends
 * FAST_READ (0Bh) without its dummy phase and reads a byte late.
 */
static void test_dio(void)
{
    char *image, *log;
    QTestState *qts = start(&image, &log);

    qtest_writel(qts, SPI0_FLASH + CACHE_FCTRL, FCTRL_AXI_REQ_EN);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x40), ==,
                    pat_word(LOW + 0x40) << 8);
    spi0_dio(qts, 0xbb, 28, 2, false);
    for (uint32_t off = 0; off < 0x100; off += 4) {
        g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + off), ==,
                        pat_word(LOW + off));
    }
    spi0_dio(qts, 0xbc, 32, 4, true);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x80), ==,
                    pat_word(LOW + 0x80));
    finish(qts, image, log);
}

/*
 * [spec:nuos:req:emu.esp32p4.memory/test]
 * BCh with a 40-bit address phase and no dummy phase, M7-M0 meant to go
 * in the address phase: the 8 bits past the 32-bit address are undefined,
 * sent as the mode bits that put the flash in continuous read mode
 * (logged). The next read's command goes to the flash as the
 * top of its address, in the erased upper half, and its address's top
 * byte as M7-M0, which ends the mode.
 */
static void test_long_address(void)
{
    char *image, *log, *text;
    QTestState *qts = start(&image, &log);

    spi0_dio(qts, 0xbc, 40, 0, true);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x10), ==,
                    pat_word(LOW + 0x10));
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x14), ==,
                    0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x18), ==,
                    pat_word(LOW + 0x18));
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x1c), ==,
                    0xffffffff);
    spi0_dio(qts, 0xbc, 32, 4, true);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x14), ==,
                    pat_word(LOW + 0x14));
    qtest_quit(qts);
    g_assert_true(g_file_get_contents(log, &text, NULL, NULL));
    g_assert_nonnull(strstr(text, "SPI0's address phase is 40 bits, longer "
                            "than the 32-bit address; the 8 bits after the "
                            "address are undefined"));
    g_free(text);
    unlink(image);
    unlink(log);
    g_free(image);
    g_free(log);
}

/*
 * [spec:nuos:req:emu.esp32p4.memory/test]
 * A dummy phase a cycle short samples the flash's last mode-bit clock,
 * which nothing drives, and reads the data two bits late; a cycle long
 * reads it two bits early.
 */
static void test_dummy(void)
{
    char *image, *log;
    QTestState *qts = start(&image, &log);
    uint32_t a = LOW + 0x20, late = 0, early = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t prev = i ? pat(a + i - 1) : 0;

        late |= (uint32_t)(uint8_t)(prev << 6 | pat(a + i) >> 2) << (8 * i);
        early |= (uint32_t)(uint8_t)(pat(a + i) << 2 | pat(a + i + 1) >> 6) <<
                 (8 * i);
    }
    spi0_dio(qts, 0xbb, 28, 1, false);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x20), ==, late);
    spi0_dio(qts, 0xbb, 28, 3, false);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0x20), ==, early);
    finish(qts, image, log);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32p4/spi0-reads/dio", test_dio);
    qtest_add_func("esp32p4/spi0-reads/long-address", test_long_address);
    qtest_add_func("esp32p4/spi0-reads/dummy", test_dummy);
    return g_test_run();
}
