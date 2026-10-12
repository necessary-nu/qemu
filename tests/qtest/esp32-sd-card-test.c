/*
 * QTest testcase for SD cards that take time on the ESP32 machine: the
 * board's card in the SD/MMC host's slot 1, from -drive if=sd, its
 * registers and busy
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Card 1's clock is the CIU's 80 MHz divided by 4, 20 MHz; the card a raw
 * image of 16 MiB, SDSC, Class 4: its access and programming times are
 * both its TAAC, 70 us.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_WIFI_CLK_EN       0x3ff000cc
#define DPORT_CORE_RST_EN       0x3ff000d0
#define WIFI_CLK_SDIO_HOST      (1u << 13)
#define CORE_RST_SDIO_HOST      (1u << 6)
#define RTC_CNTL                0x3ff48000
#define RTC_CNTL_WDTCONFIG0     (RTC_CNTL + 0x8c)
#define RTC_CNTL_WDTWPROTECT    (RTC_CNTL + 0xa4)
#define WDT_WKEY                0x50d83aa1
#define WDT_EN                  (1u << 31)
#define WDT_FLASHBOOT_MOD_EN    (1u << 10)
#define TIMG0_WDTCONFIG0        0x3ff5f048
#define TIMG0_WDTWPROTECT       0x3ff5f064

#define SD                      0x3ff68000
#define SD_CTRL                 (SD + 0x000)
#define SD_CLKDIV               (SD + 0x008)
#define SD_CLKSRC               (SD + 0x00c)
#define SD_CLKENA               (SD + 0x010)
#define SD_CTYPE                (SD + 0x018)
#define SD_BLKSIZ               (SD + 0x01c)
#define SD_BYTCNT               (SD + 0x020)
#define SD_CMDARG               (SD + 0x028)
#define SD_CMD                  (SD + 0x02c)
#define SD_RESP(n)              (SD + 0x030 + 4 * (n))
#define SD_RINTSTS              (SD + 0x044)
#define SD_STATUS               (SD + 0x048)
#define SD_FIFOTH               (SD + 0x04c)
#define SD_BUFFIFO              (SD + 0x200)

#define CMD_RESP                (1u << 6)
#define CMD_LONG                (1u << 7)
#define CMD_CRC                 (1u << 8)
#define CMD_DATA                (1u << 9)
#define CMD_WRITE               (1u << 10)
#define CMD_WAIT                (1u << 13)
#define CMD_INIT                (1u << 15)
#define CMD_CARD1               (1u << 16)
#define CMD_UPDATE_CLK          (1u << 21)
#define CMD_HOLD                (1u << 29)
#define CMD_START               (1u << 31)
#define R1                      (CMD_RESP | CMD_CRC)
#define R2                      (CMD_RESP | CMD_LONG | CMD_CRC)
#define R3                      CMD_RESP

#define INT_CMDD                (1u << 2)
#define INT_DTO                 (1u << 3)
#define INT_RTO                 (1u << 8)
#define STATUS_DATA_BUSY        (1u << 9)

#define R1_STATE(r)             (((r) >> 9) & 0xf)
#define STATE_TRAN              4
#define STATE_PRG               7

#define TAAC_NS                 70000
#define IMAGE_SIZE              (16 * 1024 * 1024)

static char *make_image(void)
{
    g_autofree uint8_t *data = g_malloc(IMAGE_SIZE);
    char *path;
    int fd;

    fd = g_file_open_tmp("esp32-sd-card-XXXXXX.img", &path, NULL);
    g_assert(fd >= 0);
    for (int i = 0; i < IMAGE_SIZE; i++) {
        data[i] = (uint8_t)(i / 512 * 3 + i);
    }
    g_assert_cmpint(write(fd, data, IMAGE_SIZE), ==, IMAGE_SIZE);
    close(fd);
    return path;
}

static uint8_t new_byte(int i)
{
    return (uint8_t)(0x5a ^ (i * 11));
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

static void sdmmc_setup(QTestState *qts)
{
    stop_watchdogs(qts);
    qtest_writel(qts, DPORT_WIFI_CLK_EN,
                 qtest_readl(qts, DPORT_WIFI_CLK_EN) | WIFI_CLK_SDIO_HOST);
    qtest_writel(qts, DPORT_CORE_RST_EN,
                 qtest_readl(qts, DPORT_CORE_RST_EN) & ~CORE_RST_SDIO_HOST);
    qtest_writel(qts, SD_CLKDIV, 2 << 8);
    qtest_writel(qts, SD_CLKSRC, 1 << 2);
    qtest_writel(qts, SD_CLKENA, 1 << 1);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_CARD1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_FIFOTH, (7u << 16) | 8);
}

static uint32_t wait_int(QTestState *qts, uint32_t mask)
{
    for (int i = 0; i < 100000; i++) {
        uint32_t st = qtest_readl(qts, SD_RINTSTS);

        if (st & mask) {
            qtest_writel(qts, SD_RINTSTS, st);
            return st;
        }
        qtest_clock_step(qts, 1000);
    }
    g_assert_not_reached();
}

static void cmd_start(QTestState *qts, uint32_t flags, unsigned index,
                      uint32_t arg)
{
    qtest_writel(qts, SD_CMDARG, arg);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_CARD1 | flags |
                 index);
}

static uint32_t cmd(QTestState *qts, uint32_t flags, unsigned index,
                    uint32_t arg)
{
    cmd_start(qts, flags, index, arg);
    return wait_int(qts, INT_CMDD);
}

/* Identify and select the card, its CSD's TAAC in @taac: its RCA. */
static uint32_t card_init(QTestState *qts, uint32_t *taac)
{
    uint32_t rca;

    cmd(qts, CMD_INIT, 0, 0);
    g_assert_cmphex(cmd(qts, R1 | CMD_WAIT, 8, 0x1aa) & INT_RTO, ==, 0);
    cmd(qts, R1 | CMD_WAIT, 55, 0);
    cmd(qts, R3 | CMD_WAIT, 41, 0x40ff8000);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)) >> 31, ==, 1);
    cmd(qts, R2 | CMD_WAIT, 2, 0);
    cmd(qts, R1 | CMD_WAIT, 3, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    cmd(qts, R2 | CMD_WAIT, 9, rca << 16);
    /* CSD bits 119:112, in RESP3 bits 23:16 */
    *taac = (qtest_readl(qts, SD_RESP(3)) >> 16) & 0xff;
    g_assert_cmphex(cmd(qts, R1 | CMD_WAIT, 7, rca << 16) & INT_RTO, ==, 0);
    return rca;
}

static void write_block(QTestState *qts, uint32_t block)
{
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA | CMD_WRITE, 24, block * 512);
    qtest_clock_step(qts, 1000);
    for (int i = 0; i < 128; i++) {
        uint32_t w = new_byte(4 * i) | new_byte(4 * i + 1) << 8 |
                     new_byte(4 * i + 2) << 16 |
                     (uint32_t)new_byte(4 * i + 3) << 24;

        qtest_writel(qts, SD_BUFFIFO, w);
    }
    wait_int(qts, INT_DTO);
}

static bool busy(QTestState *qts)
{
    return qtest_readl(qts, SD_STATUS) & STATUS_DATA_BUSY;
}

static uint32_t status_now(QTestState *qts, uint32_t rca)
{
    g_assert_cmphex(cmd(qts, R1, 13, rca << 16) & INT_RTO, ==, 0);
    return qtest_readl(qts, SD_RESP(0));
}

/*
 * -drive if=sd puts the card in slot 1, taking time: its TAAC 70 us, a
 * write busy that long, the card in the programming state meanwhile.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_drive(void)
{
    g_autofree char *img = make_image();
    QTestState *qts = qtest_initf("-M esp32 -nic none "
                                  "-drive file=%s,if=sd,format=raw", img);
    uint32_t rca, taac;

    sdmmc_setup(qts);
    rca = card_init(qts, &taac);
    g_assert_cmphex(taac, ==, 0x74);
    write_block(qts, 3);
    g_assert_true(busy(qts));
    g_assert_cmpint(R1_STATE(status_now(qts, rca)), ==, STATE_PRG);
    qtest_clock_step(qts, TAAC_NS);
    g_assert_false(busy(qts));
    g_assert_cmpint(R1_STATE(status_now(qts, rca)), ==, STATE_TRAN);
    qtest_quit(qts);
    unlink(img);
}

/*
 * The card's properties, from -global: Class 2 makes its TAAC 200 us;
 * with timing off, it is QEMU's card as it always was, upstream's TAAC
 * and no busy.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_properties(void)
{
    g_autofree char *img = make_image();
    QTestState *qts;
    uint32_t taac;

    qts = qtest_initf("-M esp32 -nic none -drive file=%s,if=sd,format=raw "
                      "-global sd-card.speed-class=2", img);
    sdmmc_setup(qts);
    card_init(qts, &taac);
    g_assert_cmphex(taac, ==, 0x2d);
    write_block(qts, 4);
    qtest_clock_step(qts, TAAC_NS);
    g_assert_true(busy(qts));
    qtest_quit(qts);

    qts = qtest_initf("-M esp32 -nic none -drive file=%s,if=sd,format=raw "
                      "-global sd-card.timing=off", img);
    sdmmc_setup(qts);
    card_init(qts, &taac);
    g_assert_cmphex(taac, ==, 0x26);
    write_block(qts, 4);
    g_assert_false(busy(qts));
    qtest_quit(qts);
    unlink(img);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/sd-card/drive", test_drive);
    qtest_add_func("/esp32/sd-card/properties", test_properties);
    return g_test_run();
}
