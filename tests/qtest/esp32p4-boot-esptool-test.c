/*
 * QTest testcase for the ESP32-P4 boot ROM's download mode as esptool
 * drives it on serial port 0: the board check it makes of the chip, and
 * the security download mode
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "esp32p4-boot-utils.h"

#define EFUSE_RD_MAC_SYS_2              0x5012d04c
#define ROM_ECO_VERSION                 0x4fc00014

/*
 * The board check for erratum ROM-764 and the chip revision, as esptool
 * makes it in download mode ("chip-id", "read-mem", "write-mem"): the
 * eFuses' wafer version, as esptool reads it, is v3.0; the ROM's ECO
 * version (0x4FC00014) is 5, v3.0's ROM; a word written at 0x4FF00000,
 * in the L2 cache's 128 KiB of L2MEM, does not stick, while one at
 * 0x4FF20000, just past them, does.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_rom764_board_check(void)
{
    int fd;
    QTestState *qts = qtest_init_with_serial(
        "-M esp32p4,strap-mode=0x04 -accel tcg -icount shift=0,sleep=off",
        &fd);
    uint32_t sys2;

    esp_sync(fd);
    sys2 = esp_read_reg(fd, EFUSE_RD_MAC_SYS_2);
    g_assert_cmpuint((((sys2 >> 23) & 1) << 2 | ((sys2 >> 4) & 3)) * 100 +
                     (sys2 & 0xf), ==, 300);
    g_assert_cmpuint(esp_read_reg(fd, ROM_ECO_VERSION), ==, 5);
    esp_write_reg(fd, 0x4ff00000, 0x12345678);
    g_assert_cmphex(esp_read_reg(fd, 0x4ff00000), ==, 0);
    esp_write_reg(fd, 0x4ff20000, 0x12345678);
    g_assert_cmphex(esp_read_reg(fd, 0x4ff20000), ==, 0x12345678);
    close(fd);
    qtest_quit(qts);
}

/*
 * The security download mode (EFUSE_ENABLE_SECURITY_DOWNLOAD). In joint
 * download boot the ROM, past its anti-glitch checks of the random number
 * generator (which reset the chip when they fail in this mode), takes
 * the commands that work on the flash and the security information
 * (SPI_ATTACH, FLASH_BEGIN, FLASH_DATA, GET_SECURITY_INFO) and refuses
 * those on memory and registers (MEM_BEGIN, READ_REG: status 1, error
 * 5, the response keeping the last command taken's opcode). It refuses
 * the SPI download boot mode outright. The flash is powered by the board,
 * whose power the model does not switch: on v3.1 silicon with its flash
 * on the chip's switchable supply the ROM's refusal of register writes
 * keeps the flash commands from working (Espressif's erratum ROM-770).
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_secure_download(void)
{
    g_autofree char *efuse = write_efuse(EFUSE_DATA(3),
                                         ENABLE_SECURITY_DOWNLOAD);
    g_autofree char *args = g_strdup_printf(
        "-M esp32p4,flash-size=4M,strap-mode=0x04 -accel tcg "
        "-icount shift=0,sleep=off -drive file=%s,if=none,format=raw,"
        "id=efuse,snapshot=on -global driver=nvram.esp32p4.efuse,"
        "property=drive,value=efuse", efuse);
    uint8_t begin[20], block[16 + 1024], word[4];
    uint32_t checksum = 0xef;
    EspReply r;
    int fd;
    QTestState *qts = qtest_init_with_serial(args, &fd);
    Boot b;

    esp_sync(fd);

    /* GET_SECURITY_INFO: flags, SECURE_DOWNLOAD_ENABLE (bit 2), chip 18. */
    r = esp_command(fd, 0x14, NULL, 0, 0);
    g_assert_cmpint(r.cmd, ==, 0x14);
    g_assert_cmpint(r.status, ==, 0);
    g_assert_cmpint(r.len, ==, 20);
    g_assert_cmphex(ldl_le_p(r.data) & (1u << 2), ==, 1u << 2);
    g_assert_cmpuint(ldl_le_p(r.data + 12), ==, 18);

    /* READ_REG and MEM_BEGIN are refused. */
    stl_le_p(word, GPIO_STRAP);
    r = esp_command(fd, 0x0a, word, sizeof(word), 0);
    g_assert_cmpint(r.status, ==, 1);
    g_assert_cmpint(r.error, ==, 5);
    stl_le_p(begin, 64);
    stl_le_p(begin + 4, 1);
    stl_le_p(begin + 8, 64);
    stl_le_p(begin + 12, LOAD_ADDR);
    r = esp_command(fd, 0x05, begin, 16, 0);
    g_assert_cmpint(r.status, ==, 1);
    g_assert_cmpint(r.error, ==, 5);

    /* SPI_ATTACH, then a block written to the flash at 1 MiB. */
    memset(begin, 0, sizeof(begin));
    r = esp_command(fd, 0x0d, begin, 8, 0);
    g_assert_cmpint(r.cmd, ==, 0x0d);
    g_assert_cmpint(r.status, ==, 0);
    stl_le_p(begin, 1024);
    stl_le_p(begin + 4, 1);
    stl_le_p(begin + 8, 1024);
    stl_le_p(begin + 12, 0x100000);
    stl_le_p(begin + 16, 0);
    r = esp_command(fd, 0x02, begin, 20, 0);
    g_assert_cmpint(r.cmd, ==, 0x02);
    g_assert_cmpint(r.status, ==, 0);
    memset(block, 0, 16);
    stl_le_p(block, 1024);
    for (int i = 16; i < sizeof(block); i++) {
        block[i] = i * 7;
        checksum ^= block[i];
    }
    r = esp_command(fd, 0x03, block, sizeof(block), checksum);
    g_assert_cmpint(r.cmd, ==, 0x03);
    g_assert_cmpint(r.status, ==, 0);
    close(fd);
    qtest_quit(qts);

    /* Strapped for SPI download boot: refused, and the chip reset. */
    b = boot(0x00, NULL, efuse, NULL);
    await_output(&b, "boot:0x1 (SPI_DOWNLOAD_BOOT)");
    await_output(&b, "Invalid mode for secure boot download");
    boot_end(&b);
    unlink(efuse);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/boot/rom764-board-check", test_rom764_board_check);
    qtest_add_func("/esp32p4/boot/secure-download", test_secure_download);
    return g_test_run();
}
