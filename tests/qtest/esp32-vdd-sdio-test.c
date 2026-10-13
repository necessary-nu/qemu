/*
 * QTest testcase for the ESP32's VDD_SDIO, the ESP32-WROOM-32E's flash
 * supply
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * VDD_SDIO comes from the flash regulator: as RTC_CNTL_SDIO_CONF forces
 * it, else as the eFuses force it, else at the voltage the MTDI strap
 * gives it at reset (low: VDD3P3_RTC's 3.3 V; high: 1.8 V). The module's
 * flash is a 3.3 V part: at 1.8 V it does not answer, and SPI1 reads the
 * pull-ups; with VDD_SDIO off the pads' inputs read 0. Back in range it
 * starts as at power-up and takes no command for 1.8 ms (tVSL); a cycle
 * the supply cut short is left partly done.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "esp32-flash-pins.h"
#include "esp32-spi-flash.h"

#define RTC_CNTL_SDIO_CONF      0x3ff48074
#define SDIO_CONF_RESET         0x02a00000
#define SDIO_PD_EN              (1u << 21)
#define SDIO_FORCE              (1u << 22)
#define SDIO_TIEH               (1u << 23)
#define XPD_SDIO_REG            (1u << 31)

#define GPIO_STRAP              (FP_GPIO + 0x38)
#define STRAP_MTDI              (1u << 5)

/* eFuse BLK0 word 4's flash regulator bits */
#define EFUSE_XPD_SDIO_REG      (1u << 14)
#define EFUSE_SDIO_TIEH         (1u << 15)
#define EFUSE_SDIO_FORCE        (1u << 16)
#define EFUSE_REGS_SIZE         (31 * 4)

#define FLASH_ID                0x1640c8
#define FLASH_SIZE              (4 * 1024 * 1024)
#define T_VSL_NS                (1800 * 1000)

static char *flash_path;

static QTestState *start(const char *extra)
{
    QTestState *qts = qtest_initf("-M esp32 -nic none "
                                  "-drive file=%s,if=mtd,format=raw,"
                                  "snapshot=on %s", flash_path, extra);

    flash_pins_rom(qts);
    sf_read_mode(qts, SF_SLOW, 0);
    return qts;
}

static uint32_t rdid(QTestState *qts)
{
    return sf_usr(qts, 0x9f, 0, 0, 24);
}

static void vdd_sdio(QTestState *qts, uint32_t conf)
{
    qtest_writel(qts, RTC_CNTL_SDIO_CONF, conf);
}

/* An eFuse image with BLK0 word 4 @word4, as the eFuse drive holds it */
static char *efuse_image(uint32_t word4)
{
    uint8_t img[EFUSE_REGS_SIZE] = { 0 };
    char *path;
    int fd;

    stl_le_p(img + 4 * 4, word4);
    fd = g_file_open_tmp("esp32-vdd-sdio-efuse-XXXXXX", &path, NULL);
    g_assert(fd >= 0);
    g_assert_cmpint(write(fd, img, sizeof(img)), ==, sizeof(img));
    close(fd);
    return path;
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * From reset, MTDI pulled down: 3.3 V and the flash answers.
 */
static void test_default(void)
{
    QTestState *qts = start("");

    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP) & STRAP_MTDI, ==, 0);
    g_assert_cmphex(qtest_readl(qts, RTC_CNTL_SDIO_CONF), ==,
                    SDIO_CONF_RESET);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * [spec:nuos:req:emu.flash-power-cut/test]
 * RTC_CNTL_SDIO_CONF forcing 1.8 V: the flash does not answer and SPI1
 * reads the pull-ups; off, the pads read 0. Back at 3.3 V the flash is
 * deaf for tVSL, then answers, having lost its write enable latch.
 */
static void test_force(void)
{
    QTestState *qts = start("");

    sf_cmd(qts, SF_CMD_WREN);
    g_assert_cmphex(sf_rdsr(qts), ==, SF_SR1_WEL);
    vdd_sdio(qts, SDIO_FORCE | XPD_SDIO_REG);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    g_assert_cmphex(sf_read_word(qts, 0x100), ==, 0xffffffff);
    vdd_sdio(qts, SDIO_FORCE);
    g_assert_cmphex(rdid(qts), ==, 0);
    vdd_sdio(qts, SDIO_FORCE | XPD_SDIO_REG | SDIO_TIEH);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    qtest_clock_step(qts, T_VSL_NS);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    g_assert_cmphex(sf_rdsr(qts), ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * [spec:nuos:req:emu.flash-power-cut/test]
 * A sector erase the supply cuts short leaves the sector partly erased.
 */
static void test_power_cut(void)
{
    QTestState *qts = start("");
    const uint32_t a = 0x20000;
    uint32_t v;

    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_W0, 0);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, a | (4u << 24));
    sf_cmd(qts, SF_CMD_PP);
    sf_wait_idle(qts);
    g_assert_cmphex(sf_read_word(qts, a), ==, 0);

    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, a);
    sf_cmd(qts, SF_CMD_SE);
    qtest_clock_step(qts, 25 * 1000 * 1000);
    vdd_sdio(qts, SDIO_FORCE | XPD_SDIO_REG);
    vdd_sdio(qts, SDIO_FORCE | XPD_SDIO_REG | SDIO_TIEH);
    qtest_clock_step(qts, T_VSL_NS);
    g_assert_cmphex(sf_rdsr(qts), ==, 0);
    v = sf_read_word(qts, a);
    g_assert_cmphex(v, !=, 0);
    g_assert_cmphex(v, !=, 0xffffffff);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * MTDI strapped high, by the board's tie or by an SD card's pull-up on
 * GPIO12 against MTDI's pull-down, gives 1.8 V from reset: the flash
 * does not answer, until software forces 3.3 V.
 */
static void test_strap(void)
{
    QTestState *qts = start("-global driver=esp32.gpio,property=strap_mode,"
                            "value=0x32");

    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP) & STRAP_MTDI, ==,
                    STRAP_MTDI);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    vdd_sdio(qts, SDIO_FORCE | XPD_SDIO_REG | SDIO_TIEH);
    qtest_clock_step(qts, T_VSL_NS);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_quit(qts);

    qts = start("-machine sd-pull-up=10000");
    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP) & STRAP_MTDI, ==,
                    STRAP_MTDI);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rtc/test]
 * The eFuses' SDIO_FORCE overrides the strap: with SDIO_TIEH and
 * XPD_SDIO_REG, 3.3 V whatever MTDI says (espefuse's set_flash_voltage
 * 3.3V); without XPD_SDIO_REG, off.
 */
static void test_efuse(void)
{
    char *fuses = efuse_image(EFUSE_SDIO_FORCE | EFUSE_SDIO_TIEH |
                              EFUSE_XPD_SDIO_REG);
    g_autofree char *args = g_strdup_printf(
        "-drive file=%s,if=none,format=raw,id=efuse,snapshot=on "
        "-global driver=nvram.esp32.efuse,property=drive,value=efuse "
        "-global driver=esp32.gpio,property=strap_mode,value=0x32", fuses);
    QTestState *qts = start(args);

    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_quit(qts);
    unlink(fuses);
    g_free(fuses);

    fuses = efuse_image(EFUSE_SDIO_FORCE | EFUSE_SDIO_TIEH);
    g_free(args);
    args = g_strdup_printf(
        "-drive file=%s,if=none,format=raw,id=efuse,snapshot=on "
        "-global driver=nvram.esp32.efuse,property=drive,value=efuse",
        fuses);
    qts = start(args);
    g_assert_cmphex(rdid(qts), ==, 0);
    qtest_quit(qts);
    unlink(fuses);
    g_free(fuses);
}

int main(int argc, char **argv)
{
    g_autofree uint8_t *data = g_malloc(FLASH_SIZE);
    int fd, ret;

    memset(data, 0x5a, FLASH_SIZE);
    fd = g_file_open_tmp("esp32-vdd-sdio-XXXXXX", &flash_path, NULL);
    g_assert(fd >= 0);
    g_assert_cmpint(write(fd, data, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);

    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/vdd-sdio/default", test_default);
    qtest_add_func("/esp32/vdd-sdio/force", test_force);
    qtest_add_func("/esp32/vdd-sdio/power-cut", test_power_cut);
    qtest_add_func("/esp32/vdd-sdio/strap", test_strap);
    qtest_add_func("/esp32/vdd-sdio/efuse", test_efuse);
    ret = g_test_run();
    unlink(flash_path);
    g_free(flash_path);
    return ret;
}
