/*
 * QTest testcase for the ESP32 board's chips across ESP32 resets
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define DPORT_PERIP_RST_EN      0x3ff000c4
#define DPORT_WIFI_CLK_EN       0x3ff000cc
#define DPORT_CORE_RST_EN       0x3ff000d0
#define PERIP_I2C0              (1u << 7)
#define WIFI_CLK_SDIO_HOST      (1u << 13)
#define CORE_RST_SDIO_HOST      (1u << 6)

#define RTC_CNTL                0x3ff48000
#define RTC_CNTL_OPTIONS0       (RTC_CNTL + 0x00)
#define RTC_CNTL_RESET_STATE    (RTC_CNTL + 0x34)
#define RTC_CNTL_WDTCONFIG0     (RTC_CNTL + 0x8c)
#define RTC_CNTL_WDTCONFIG1     (RTC_CNTL + 0x90)
#define RTC_CNTL_WDTWPROTECT    (RTC_CNTL + 0xa4)
#define SW_SYS_RST              (1u << 31)
#define WDT_WKEY                0x50d83aa1
#define WDT_EN                  (1u << 31)
#define WDT_STG0(a)             ((a) << 28)
#define WDT_STG_RTC_RESET       4
#define WDT_FLASHBOOT_MOD_EN    (1u << 10)

#define POWERON_RESET           1
#define SW_RESET                3
#define RTCWDT_RTC_RESET        16

#define TIMG0_WDTCONFIG0        0x3ff5f048
#define TIMG0_WDTWPROTECT       0x3ff5f064

#define GPIO_BASE               0x3ff44000
#define GPIO_PIN(n)             (GPIO_BASE + 0x88 + 4 * (n))
#define GPIO_FUNC_IN(s)         (GPIO_BASE + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)        (GPIO_BASE + 0x530 + 4 * (n))
#define PIN_PAD_DRIVER          (1u << 2)
#define IN_SIG_IN_SEL           (1u << 7)
#define IO_MUX_GPIO21           0x3ff4907c
#define IO_MUX_GPIO22           0x3ff49080
#define FUN_WPU                 (1u << 8)
#define FUN_IE                  (1u << 9)
#define MCU_SEL_GPIO            (2u << 12)
#define SIG_I2C0_SCL            29
#define SIG_I2C0_SDA            30

#define I2C0_BASE               0x3ff53000
#define I2C_SCL_LOW_PERIOD      (I2C0_BASE + 0x00)
#define I2C_CTR                 (I2C0_BASE + 0x04)
#define I2C_FIFO_CONF           (I2C0_BASE + 0x18)
#define I2C_FIFO_DATA           (I2C0_BASE + 0x1c)
#define I2C_INT_RAW             (I2C0_BASE + 0x20)
#define I2C_INT_CLR             (I2C0_BASE + 0x24)
#define I2C_CMD(n)              (I2C0_BASE + 0x58 + 4 * (n))
#define CTR_MS_MODE             (1u << 4)
#define CTR_TRANS_START         (1u << 5)
#define FIFO_RX_RST             (1u << 12)
#define FIFO_TX_RST             (1u << 13)
#define I2C_ACK_ERR             (1u << 10)
#define I2C_TRANS_COMPLETE      (1u << 7)
#define CMD_RSTART              (0u << 11)
#define CMD_WRITE               (1u << 11)
#define CMD_READ                (2u << 11)
#define CMD_STOP                (3u << 11)
#define CMD_ACK_CHECK           (1u << 8)
#define CMD_ACK_VAL             (1u << 10)

#define TMP105_ADDR             0x48
#define TMP105_T_HIGH           3
#define TMP105_80C              0x5000
#define TMP105_60C              0x3c00

#define SPI1_BASE               0x3ff42000
#define SPI_CMD                 (SPI1_BASE + 0x00)
#define SPI_RD_STATUS           (SPI1_BASE + 0x10)
#define SPI_CMD_WREN            (1u << 30)
#define SPI_CMD_RDSR            (1u << 27)
#define SPI_CMD_WRSR            (1u << 26)
#define FLASH_SR_WEL            (1u << 1)
#define FLASH_SR_BP0            (1u << 2)
#define FLASH_SR_BP1            (1u << 3)

#define SDMMC_BASE              0x3ff68000
#define SDMMC_CTRL              (SDMMC_BASE + 0x00)
#define SDMMC_BLKSIZ            (SDMMC_BASE + 0x1c)
#define SDMMC_CMDARG            (SDMMC_BASE + 0x28)
#define SDMMC_CMD               (SDMMC_BASE + 0x2c)
#define SDMMC_RESP0             (SDMMC_BASE + 0x30)
#define SDMMC_RINTSTS           (SDMMC_BASE + 0x44)
#define SDMMC_CMD_START         (1u << 31)
#define SDMMC_CMD_USE_HOLD_REG  (1u << 29)
#define SDMMC_CMD_RESP_EXPECT   (1u << 6)
#define SDMMC_CMD_RESP_LONG     (1u << 7)
#define SDMMC_INT_RTO           (1u << 8)
#define SDMMC_INT_CMD_DONE      (1u << 2)
/* The card's R1 status: CURRENT_STATE, bits 12:9; 3 is stand-by */
#define SD_R1_STATE(r)          (((r) >> 9) & 0xf)
#define SD_STATE_STBY           3

static char *make_image(size_t size)
{
    char *path;
    int fd = g_file_open_tmp("esp32-board-reset-XXXXXX", &path, NULL);

    g_assert(fd >= 0);
    g_assert(ftruncate(fd, size) == 0);
    close(fd);
    return path;
}

/*
 * The ESP32 with a 4 MiB SPI flash and an SD card. The flash and SD card
 * images are deleted as soon as QEMU has them open.
 */
static QTestState *start(void)
{
    char *flash = make_image(4 * 1024 * 1024);
    char *sd = make_image(16 * 1024 * 1024);
    QTestState *qts = qtest_initf(
        "-M esp32 -nic none "
        "-drive file=%s,if=mtd,format=raw "
        "-drive file=%s,if=sd,format=raw", flash, sd);

    unlink(flash);
    unlink(sd);
    g_free(flash);
    g_free(sd);
    return qts;
}

/* Stop both flash-boot watchdogs, which would reset the chip. */
static void stop_watchdogs(QTestState *qts)
{
    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, RTC_CNTL_WDTCONFIG0,
                 qtest_readl(qts, RTC_CNTL_WDTCONFIG0) &
                 ~(WDT_EN | WDT_FLASHBOOT_MOD_EN));
    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, 0);
    qtest_writel(qts, TIMG0_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, TIMG0_WDTCONFIG0, 0);
    qtest_writel(qts, TIMG0_WDTWPROTECT, 0);
}

static void reset_cause(QTestState *qts, uint32_t cause)
{
    g_assert_cmphex(qtest_readl(qts, RTC_CNTL_RESET_STATE) & 0x3f, ==, cause);
}

/* A software system reset, as RTC_CNTL_SW_SYS_RST makes it. */
static void sw_reset(QTestState *qts)
{
    qtest_writel(qts, RTC_CNTL_OPTIONS0,
                 qtest_readl(qts, RTC_CNTL_OPTIONS0) | SW_SYS_RST);
    reset_cause(qts, SW_RESET);
    stop_watchdogs(qts);
}

/* An RTC watchdog RTC reset, which resets the RTC domain too. */
static void rtc_reset(QTestState *qts)
{
    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, RTC_CNTL_WDTCONFIG1, 1500);
    qtest_writel(qts, RTC_CNTL_WDTCONFIG0,
                 WDT_EN | WDT_STG0(WDT_STG_RTC_RESET));
    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, 0);
    qtest_clock_step(qts, 11 * 1000 * 1000);
    reset_cause(qts, RTCWDT_RTC_RESET);
    stop_watchdogs(qts);
}

/* QEMU's system reset: the board's power cycle. */
static void power_cycle(QTestState *qts)
{
    qtest_system_reset(qts);
    reset_cause(qts, POWERON_RESET);
    stop_watchdogs(qts);
}

static void i2c0_setup(QTestState *qts)
{
    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_I2C0);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) & ~PERIP_I2C0);
    qtest_writel(qts, IO_MUX_GPIO21, MCU_SEL_GPIO | FUN_IE | FUN_WPU);
    qtest_writel(qts, IO_MUX_GPIO22, MCU_SEL_GPIO | FUN_IE | FUN_WPU);
    qtest_writel(qts, GPIO_PIN(21), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_PIN(22), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_FUNC_OUT(21), SIG_I2C0_SDA);
    qtest_writel(qts, GPIO_FUNC_OUT(22), SIG_I2C0_SCL);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SDA), IN_SIG_IN_SEL | 21);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SCL), IN_SIG_IN_SEL | 22);
}

static void i2c_run(QTestState *qts, const uint32_t *cmds, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        qtest_writel(qts, I2C_CMD(i), cmds[i]);
    }
    qtest_writel(qts, I2C_CTR, CTR_MS_MODE | CTR_TRANS_START);
    g_assert_cmphex(qtest_readl(qts, I2C_INT_RAW) &
                    (I2C_ACK_ERR | I2C_TRANS_COMPLETE), ==,
                    I2C_TRANS_COMPLETE);
}

static void i2c_fifo_reset(QTestState *qts)
{
    qtest_writel(qts, I2C_FIFO_CONF, FIFO_RX_RST | FIFO_TX_RST);
    qtest_writel(qts, I2C_FIFO_CONF, 0);
    qtest_writel(qts, I2C_INT_CLR, 0x1fff);
}

static void tmp105_set_t_high(QTestState *qts, uint16_t value)
{
    static const uint32_t cmds[] = {
        CMD_RSTART, CMD_WRITE | CMD_ACK_CHECK | 4, CMD_STOP,
    };

    i2c0_setup(qts);
    i2c_fifo_reset(qts);
    qtest_writel(qts, I2C_FIFO_DATA, TMP105_ADDR << 1);
    qtest_writel(qts, I2C_FIFO_DATA, TMP105_T_HIGH);
    qtest_writel(qts, I2C_FIFO_DATA, value >> 8);
    qtest_writel(qts, I2C_FIFO_DATA, value & 0xff);
    i2c_run(qts, cmds, ARRAY_SIZE(cmds));
}

static uint16_t tmp105_t_high(QTestState *qts)
{
    static const uint32_t cmds[] = {
        CMD_RSTART,
        CMD_WRITE | CMD_ACK_CHECK | 2,
        CMD_RSTART,
        CMD_WRITE | CMD_ACK_CHECK | 1,
        CMD_READ | 1,
        CMD_READ | CMD_ACK_VAL | 1,
        CMD_STOP,
    };
    uint16_t value;

    i2c0_setup(qts);
    i2c_fifo_reset(qts);
    qtest_writel(qts, I2C_FIFO_DATA, TMP105_ADDR << 1);
    qtest_writel(qts, I2C_FIFO_DATA, TMP105_T_HIGH);
    qtest_writel(qts, I2C_FIFO_DATA, (TMP105_ADDR << 1) | 1);
    i2c_run(qts, cmds, ARRAY_SIZE(cmds));
    value = qtest_readl(qts, I2C_FIFO_DATA) << 8;
    value |= qtest_readl(qts, I2C_FIFO_DATA);
    return value;
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_tmp105(void)
{
    QTestState *qts = start();

    stop_watchdogs(qts);
    g_assert_cmphex(tmp105_t_high(qts), ==, TMP105_80C);
    tmp105_set_t_high(qts, TMP105_60C);

    sw_reset(qts);
    g_assert_cmphex(tmp105_t_high(qts), ==, TMP105_60C);
    rtc_reset(qts);
    g_assert_cmphex(tmp105_t_high(qts), ==, TMP105_60C);

    power_cycle(qts);
    g_assert_cmphex(tmp105_t_high(qts), ==, TMP105_80C);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_tmp105_dport_reset(void)
{
    QTestState *qts = start();

    stop_watchdogs(qts);
    tmp105_set_t_high(qts, TMP105_60C);
    qtest_writel(qts, I2C_SCL_LOW_PERIOD, 100);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) | PERIP_I2C0);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) & ~PERIP_I2C0);
    g_assert_cmphex(qtest_readl(qts, I2C_SCL_LOW_PERIOD), ==, 0);
    g_assert_cmphex(tmp105_t_high(qts), ==, TMP105_60C);
    qtest_quit(qts);
}

static void spi1_cmd(QTestState *qts, uint32_t cmd)
{
    qtest_writel(qts, SPI_CMD, cmd);
}

static uint32_t flash_status(QTestState *qts)
{
    spi1_cmd(qts, SPI_CMD_RDSR);
    return qtest_readl(qts, SPI_RD_STATUS) & 0xff;
}

static void flash_set_status(QTestState *qts)
{
    spi1_cmd(qts, SPI_CMD_WREN);
    qtest_writel(qts, SPI_RD_STATUS, FLASH_SR_BP1 | FLASH_SR_BP0);
    spi1_cmd(qts, SPI_CMD_WRSR);
    spi1_cmd(qts, SPI_CMD_WREN);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_flash(void)
{
    const uint32_t set = FLASH_SR_BP1 | FLASH_SR_BP0 | FLASH_SR_WEL;
    QTestState *qts = start();

    stop_watchdogs(qts);
    g_assert_cmphex(flash_status(qts), ==, 0);
    flash_set_status(qts);
    g_assert_cmphex(flash_status(qts), ==, set);

    sw_reset(qts);
    g_assert_cmphex(flash_status(qts), ==, set);
    rtc_reset(qts);
    g_assert_cmphex(flash_status(qts), ==, set);

    power_cycle(qts);
    g_assert_cmphex(flash_status(qts), ==, 0);
    qtest_quit(qts);
}

static void sdmmc_setup(QTestState *qts)
{
    qtest_writel(qts, DPORT_WIFI_CLK_EN,
                 qtest_readl(qts, DPORT_WIFI_CLK_EN) | WIFI_CLK_SDIO_HOST);
    qtest_writel(qts, DPORT_CORE_RST_EN,
                 qtest_readl(qts, DPORT_CORE_RST_EN) & ~CORE_RST_SDIO_HOST);
}

/*
 * Send a command to the card; returns false on a response timeout, which
 * is how a card that does not answer a command in its state shows.
 */
static bool sd_cmd(QTestState *qts, uint32_t index, uint32_t arg,
                   uint32_t flags, uint32_t *resp)
{
    uint32_t rintsts;

    qtest_writel(qts, SDMMC_RINTSTS, 0xffffffff);
    qtest_writel(qts, SDMMC_CMDARG, arg);
    qtest_writel(qts, SDMMC_CMD,
                 SDMMC_CMD_START | SDMMC_CMD_USE_HOLD_REG | flags | index);
    rintsts = qtest_readl(qts, SDMMC_RINTSTS);
    g_assert_true(rintsts & SDMMC_INT_CMD_DONE);
    if (resp) {
        *resp = qtest_readl(qts, SDMMC_RESP0);
    }
    return !(rintsts & SDMMC_INT_RTO);
}

/* Identify the card, which leaves it in stand-by with an RCA; returns it. */
static uint32_t sd_identify(QTestState *qts)
{
    uint32_t resp;

    sd_cmd(qts, 0, 0, 0, NULL);
    g_assert_true(sd_cmd(qts, 8, 0x1aa, SDMMC_CMD_RESP_EXPECT, &resp));
    g_assert_cmphex(resp & 0xfff, ==, 0x1aa);
    g_assert_true(sd_cmd(qts, 55, 0, SDMMC_CMD_RESP_EXPECT, NULL));
    g_assert_true(sd_cmd(qts, 41, 0x40ff8000, SDMMC_CMD_RESP_EXPECT, &resp));
    g_assert_true(resp & (1u << 31));
    g_assert_true(sd_cmd(qts, 2, 0,
                         SDMMC_CMD_RESP_EXPECT | SDMMC_CMD_RESP_LONG, NULL));
    g_assert_true(sd_cmd(qts, 3, 0, SDMMC_CMD_RESP_EXPECT, &resp));
    return resp >> 16;
}

/* Whether the card answers CMD13 for `rca`, in stand-by. */
static bool sd_in_standby(QTestState *qts, uint32_t rca)
{
    uint32_t resp;

    sdmmc_setup(qts);
    if (!sd_cmd(qts, 13, rca << 16, SDMMC_CMD_RESP_EXPECT, &resp)) {
        return false;
    }
    g_assert_cmpuint(SD_R1_STATE(resp), ==, SD_STATE_STBY);
    return true;
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_sd_card(void)
{
    QTestState *qts = start();
    uint32_t rca;

    stop_watchdogs(qts);
    sdmmc_setup(qts);
    qtest_writel(qts, SDMMC_BLKSIZ, 64);
    rca = sd_identify(qts);
    g_assert_cmphex(rca, !=, 0);
    g_assert_true(sd_in_standby(qts, rca));

    /* The controller resets; the card stays identified */
    sw_reset(qts);
    sdmmc_setup(qts);
    g_assert_cmphex(qtest_readl(qts, SDMMC_BLKSIZ), ==, 512);
    g_assert_true(sd_in_standby(qts, rca));
    rtc_reset(qts);
    g_assert_true(sd_in_standby(qts, rca));

    /* After a power cycle the card is idle again and has no RCA */
    power_cycle(qts);
    g_assert_false(sd_in_standby(qts, rca));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32/board-reset/tmp105", test_tmp105);
    qtest_add_func("esp32/board-reset/tmp105-dport-reset",
                   test_tmp105_dport_reset);
    qtest_add_func("esp32/board-reset/flash", test_flash);
    qtest_add_func("esp32/board-reset/sd-card", test_sd_card);
    return g_test_run();
}
