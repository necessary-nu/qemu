/*
 * QTest testcase for the ESP32 SD/MMC host's pins: the card reached only
 * through pads set up as the hardware needs them
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The board's card is in slot 1, on its IO_MUX pads (function 3, HS2_*):
 * CLK GPIO14, CMD GPIO15, DAT0 GPIO2, DAT1 GPIO4, DAT2 GPIO12, DAT3
 * GPIO13; their pulls are RTCIO's RUE and RDE (errata [GPIO-3.6]). Set up
 * as ESP-IDF sets them up (esp32-sd-pins.h), the card answers; set up
 * otherwise, each as the board would have it: pads off their function
 * reach no card and read 1 (RTO, a read's DAT line 1s, DCRC), inputs
 * disabled read 0 (responses of 0s, RE; DAT0 busy; data 0s, EBE), a DAT
 * line off its function makes the card discard a written block, a
 * released line reads as its pulls hold it. Slot 0's pads are the
 * module's flash: no card can be there.
 *
 * Card 1's clock is the CIU's 80 MHz divided by 4, 20 MHz; the card a raw
 * image of 1 MiB, standard capacity, byte addressed.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "esp32-sd-pins.h"

#define DPORT_WIFI_CLK_EN       0x3ff000cc
#define DPORT_CORE_RST_EN       0x3ff000d0
#define WIFI_CLK_SDIO_HOST      (1u << 13)
#define CORE_RST_SDIO_HOST      (1u << 6)
#define RTC_CNTL_WDTCONFIG0     0x3ff4808c
#define RTC_CNTL_WDTWPROTECT    0x3ff480a4
#define WDT_WKEY                0x50d83aa1
#define TIMG0_WDTCONFIG0        0x3ff5f048
#define TIMG0_WDTWPROTECT       0x3ff5f064

#define GPIO_IN                 (SDP_GPIO + 0x03c)

#define SD                      0x3ff68000
#define SD_CTRL                 (SD + 0x000)
#define SD_CLKDIV               (SD + 0x008)
#define SD_CLKSRC               (SD + 0x00c)
#define SD_CLKENA               (SD + 0x010)
#define SD_TMOUT                (SD + 0x014)
#define SD_CTYPE                (SD + 0x018)
#define SD_BLKSIZ               (SD + 0x01c)
#define SD_BYTCNT               (SD + 0x020)
#define SD_CMDARG               (SD + 0x028)
#define SD_CMD                  (SD + 0x02c)
#define SD_RESP(n)              (SD + 0x030 + 4 * (n))
#define SD_RINTSTS              (SD + 0x044)
#define SD_STATUS               (SD + 0x048)
#define SD_CDETECT              (SD + 0x050)
#define SD_WRTPRT               (SD + 0x054)
#define SD_BUFFIFO              (SD + 0x200)

#define CTRL_RESETS             7u

#define CMD_RESP                (1u << 6)
#define CMD_LONG                (1u << 7)
#define CMD_CRC                 (1u << 8)
#define CMD_DATA                (1u << 9)
#define CMD_WRITE               (1u << 10)
#define CMD_WAIT                (1u << 13)
#define CMD_STOP                (1u << 14)
#define CMD_INIT                (1u << 15)
#define CMD_CARD(n)             ((n) << 16)
#define CMD_UPDATE_CLK          (1u << 21)
#define CMD_HOLD                (1u << 29)
#define CMD_START               (1u << 31)
#define R1                      (CMD_RESP | CMD_CRC)
#define R2                      (CMD_RESP | CMD_LONG | CMD_CRC)
#define R3                      CMD_RESP

#define INT_RE                  (1u << 1)
#define INT_CMDD                (1u << 2)
#define INT_DTO                 (1u << 3)
#define INT_RCRC                (1u << 6)
#define INT_DCRC                (1u << 7)
#define INT_RTO                 (1u << 8)
#define INT_DRTO                (1u << 9)
#define INT_EBE                 (1u << 15)
#define INT_SDIO(n)             (1u << (16 + (n)))
#define INT_ERRORS              (INT_RE | INT_RCRC | INT_DCRC | INT_RTO | \
                                 INT_DRTO | INT_EBE)

#define STATUS_BUSY             (1u << 9)

#define R1_STATE(r)             (((r) >> 9) & 0xf)
#define STATE_STBY              3
#define STATE_TRAN              4
#define STATE_RCV               6
#define STATE_PRG               7

#define MS                      (1000 * 1000)
#define IMAGE_SIZE              (1 * 1024 * 1024)

static char *make_image(uint8_t seed)
{
    g_autofree uint8_t *data = g_malloc(IMAGE_SIZE);
    char *path;
    int fd;

    fd = g_file_open_tmp("esp32-sd-pins-XXXXXX.img", &path, NULL);
    g_assert(fd >= 0);
    for (int i = 0; i < IMAGE_SIZE; i++) {
        data[i] = (uint8_t)(i / 512 * 7 + i + seed);
    }
    g_assert_cmpint(write(fd, data, IMAGE_SIZE), ==, IMAGE_SIZE);
    close(fd);
    return path;
}

static uint8_t image_byte(uint8_t seed, uint32_t offset)
{
    return (uint8_t)(offset / 512 * 7 + offset + seed);
}

/*
 * The machine with @img in slot 1 (-drive if=sd), @machine its further
 * options; the flash-boot watchdogs stopped, the SD/MMC host clocked and
 * out of reset.
 */
static QTestState *start(const char *machine, const char *img,
                         const char *extra)
{
    QTestState *qts = qtest_initf("-M esp32%s -nic none "
                                  "-drive file=%s,if=sd,format=raw %s",
                                  machine, img, extra);

    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, RTC_CNTL_WDTCONFIG0, 0);
    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, 0);
    qtest_writel(qts, TIMG0_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, TIMG0_WDTCONFIG0, 0);
    qtest_writel(qts, TIMG0_WDTWPROTECT, 0);
    qtest_writel(qts, DPORT_WIFI_CLK_EN,
                 qtest_readl(qts, DPORT_WIFI_CLK_EN) | WIFI_CLK_SDIO_HOST);
    qtest_writel(qts, DPORT_CORE_RST_EN,
                 qtest_readl(qts, DPORT_CORE_RST_EN) & ~CORE_RST_SDIO_HOST);
    return qts;
}

/* Card @card's clock: the CIU's 80 MHz divided by 4, 20 MHz. */
static void card_clock(QTestState *qts, unsigned card)
{
    qtest_writel(qts, SD_CLKDIV, 2u << (8 * card));
    qtest_writel(qts, SD_CLKSRC, card << (2 * card));
    qtest_writel(qts, SD_CLKENA, 1u << card);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_HOLD |
                 CMD_CARD(card));
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
}

/* Run until a RINTSTS bit in @mask is set: the bits, cleared. */
static uint32_t wait_int(QTestState *qts, uint32_t mask)
{
    for (int64_t t = 0; t <= 100 * MS; t += 1000) {
        uint32_t st = qtest_readl(qts, SD_RINTSTS);

        if (st & mask) {
            qtest_writel(qts, SD_RINTSTS, st);
            return st & ~(INT_SDIO(0) | INT_SDIO(1));
        }
        qtest_clock_step(qts, 1000);
    }
    g_assert_not_reached();
}

static uint32_t cmd(QTestState *qts, unsigned card, uint32_t flags,
                    unsigned index, uint32_t arg)
{
    qtest_writel(qts, SD_CMDARG, arg);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_CARD(card) |
                 flags | index);
    return wait_int(qts, INT_CMDD);
}

static void cmd_ok(QTestState *qts, uint32_t flags, unsigned index,
                   uint32_t arg)
{
    g_assert_cmphex(cmd(qts, 1, flags, index, arg) & INT_ERRORS, ==, 0);
}

/* CMD13: the card's state, as its R1 gives it. */
static unsigned card_state(QTestState *qts, uint32_t rca)
{
    cmd_ok(qts, R1, 13, rca << 16);
    return R1_STATE(qtest_readl(qts, SD_RESP(0)));
}

/* Identify and select card 1, its bus 4 bits wide: its RCA */
static uint32_t card_init(QTestState *qts)
{
    uint32_t ocr = 0, rca;

    card_clock(qts, 1);
    cmd(qts, 1, CMD_INIT, 0, 0);
    cmd_ok(qts, R1 | CMD_WAIT, 8, 0x1aa);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0x1aa);
    for (int tries = 0; tries < 20 && !(ocr & (1u << 31)); tries++) {
        cmd_ok(qts, R1 | CMD_WAIT, 55, 0);
        cmd(qts, 1, R3 | CMD_WAIT, 41, 0x40ff8000);
        ocr = qtest_readl(qts, SD_RESP(0));
    }
    g_assert_cmphex(ocr & (1u << 31), !=, 0);
    cmd_ok(qts, R2 | CMD_WAIT, 2, 0);
    cmd_ok(qts, R1 | CMD_WAIT, 3, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    cmd_ok(qts, R1 | CMD_WAIT, 7, rca << 16);
    cmd_ok(qts, R1 | CMD_WAIT, 55, rca << 16);
    cmd_ok(qts, R1 | CMD_WAIT, 6, 2);
    qtest_writel(qts, SD_CTYPE, 1u << 1);
    return rca;
}

/* The FIFO emptied, the data path reset, RINTSTS clear */
static void data_reset(QTestState *qts)
{
    qtest_writel(qts, SD_CTRL, CTRL_RESETS);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
}

/* CMD17 of the block at @addr through the FIFO: RINTSTS to its end. */
static uint32_t read_block(QTestState *qts, uint32_t addr, uint8_t *buf)
{
    uint32_t all = 0;

    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 512);
    qtest_writel(qts, SD_CMDARG, addr);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | CMD_CARD(1) | 17);
    while (!(all & (INT_DTO | INT_DRTO))) {
        all |= wait_int(qts, INT_DTO | INT_DRTO | INT_CMDD);
    }
    for (int i = 0; i < 512; i += 4) {
        uint32_t w = qtest_readl(qts, SD_BUFFIFO);

        memcpy(&buf[i], &w, 4);
    }
    data_reset(qts);
    return all;
}

/*
 * CMD24 (or, with @blocks 2, CMD25) of @blocks blocks of @fill at @addr
 * through the FIFO: RINTSTS to the data's end.
 */
static uint32_t write_blocks(QTestState *qts, uint32_t addr, unsigned blocks,
                             uint32_t fill)
{
    uint32_t all;

    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 512 * blocks);
    qtest_writel(qts, SD_CMDARG, addr);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | CMD_DATA |
                 CMD_WRITE | CMD_CARD(1) | (blocks > 1 ? 25 : 24));
    all = wait_int(qts, INT_CMDD);
    for (unsigned i = 0; i < 128 * blocks && !(all & INT_DTO); i++) {
        while ((qtest_readl(qts, SD_STATUS) >> 17 & 0x1fff) >= 256 &&
               !(qtest_readl(qts, SD_RINTSTS) & INT_DTO)) {
            qtest_clock_step(qts, 1000);
        }
        if (qtest_readl(qts, SD_RINTSTS) & INT_DTO) {
            break;
        }
        qtest_writel(qts, SD_BUFFIFO, fill);
    }
    all |= wait_int(qts, INT_DTO);
    data_reset(qts);
    return all;
}

static uint32_t le32(const uint8_t *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static bool gpio_in(QTestState *qts, unsigned pad)
{
    return qtest_readl(qts, GPIO_IN) >> pad & 1;
}

static bool busy(QTestState *qts)
{
    return qtest_readl(qts, SD_STATUS) & STATUS_BUSY;
}

static void mux_bits(QTestState *qts, unsigned pin, uint32_t set,
                     uint32_t clear)
{
    uint32_t addr = sd_pin_mux_addr(pin);

    qtest_writel(qts, addr, (qtest_readl(qts, addr) & ~clear) | set);
}

/*
 * From reset the pads are on function 0 (MTMS, MTDO, GPIO2, GPIO4, MTDI,
 * MTCK): no command reaches the card, the host's CMD and DAT inputs read
 * 1 (I1 inputs off their function): RTO, not busy; the card detect,
 * write protect and interrupt inputs unrouted read 0: a card, writable,
 * an SDIO interrupt asserted on both slots. Set up as ESP-IDF sets them
 * up, the card, untouched, answers from its idle state, and a block
 * reads back.
 */
/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_reset(void)
{
    g_autofree char *img = make_image(0x11);
    QTestState *qts = start("", img, "-global sd-card.timing=off");
    uint8_t buf[512];
    uint32_t rca;

    card_clock(qts, 1);
    g_assert_false(busy(qts));
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) &
                    (INT_SDIO(0) | INT_SDIO(1)), ==,
                    INT_SDIO(0) | INT_SDIO(1));
    g_assert_cmphex(qtest_readl(qts, SD_CDETECT) & 3, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_WRTPRT) & 3, ==, 0);
    cmd(qts, 1, CMD_INIT, 0, 0);
    g_assert_cmphex(cmd(qts, 1, R1 | CMD_WAIT, 8, 0x1aa), ==,
                    INT_CMDD | INT_RTO);
    g_assert_cmphex(cmd(qts, 1, R1 | CMD_WAIT, 55, 0), ==,
                    INT_CMDD | INT_RTO);

    sd_pins(qts);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) &
                    (INT_SDIO(0) | INT_SDIO(1)), ==, INT_SDIO(0));
    g_assert_cmphex(qtest_readl(qts, SD_WRTPRT) & 3, ==, 0);
    rca = card_init(qts);
    g_assert_cmphex(read_block(qts, 5 * 512, buf) & INT_ERRORS, ==, 0);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, image_byte(0x11, 5 * 512 + i));
    }
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);

    /* The card detect input from a pad pulled up: no card */
    qtest_writel(qts, SDP_FUNC_IN(SDP_SIG_CARD_DETECT_N),
                 SDP_IN_SIG_IN_SEL | SDP_IN_CONST_HIGH);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CDETECT) & 3, ==, 2);
    qtest_quit(qts);
    unlink(img);
}

/*
 * CMD set up otherwise: its input disabled reads 0, every response 0s
 * (RE), the card taking the command; on the GPIO function no command
 * reaches the card and its input reads 1 (RTO); its output enable from
 * GPIO_ENABLE (OEN_SEL) clear reaches no card (RTO), set drives the pad
 * against the card's response (RTO); handed to the RTC IO MUX, its
 * input buffer off there, the host reads 0 (RE); open drain, the pad's
 * pull-up carries the host's highs, without it they never reach the
 * card and the floating pad reads 0 (RE).
 */
/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_cmd(void)
{
    g_autofree char *img = make_image(0x22);
    QTestState *qts = start("", img, "-global sd-card.timing=off");
    uint32_t rca;

    sd_pins(qts);
    rca = card_init(qts);

    /* CMD's input disabled: CMD7 deselects the card all the same */
    mux_bits(qts, SD_PIN_CMD, 0, SDP_FUN_IE);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RE);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0);
    g_assert_cmphex(cmd(qts, 1, R1, 7, 0), ==, INT_CMDD | INT_RE);
    mux_bits(qts, SD_PIN_CMD, SDP_FUN_IE, 0);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_STBY);
    cmd_ok(qts, R1, 7, rca << 16);

    /* CMD on the GPIO function: CMD7 never reaches it */
    mux_bits(qts, SD_PIN_CMD, SDP_MCU_SEL(SDP_FUNC_GPIO), SDP_MCU_SEL_MASK);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    g_assert_cmphex(cmd(qts, 1, R1, 7, 0), ==, INT_CMDD | INT_RTO);
    sd_pin_iomux(qts, SD_PIN_CMD);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);

    /* CMD's enable from GPIO_ENABLE: clear, then set */
    qtest_writel(qts, SDP_FUNC_OUT(15), SDP_SIG_GPIO_OUT | SDP_OUT_OEN_SEL);
    qtest_writel(qts, SDP_ENABLE_W1TC, 1u << 15);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    qtest_writel(qts, SDP_ENABLE_W1TS, 1u << 15);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    qtest_writel(qts, SDP_ENABLE_W1TC, 1u << 15);
    qtest_writel(qts, SDP_FUNC_OUT(15), SDP_SIG_GPIO_OUT);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);

    /* CMD handed to the RTC IO MUX */
    sd_pin_rtc_bits(qts, SD_PIN_CMD, SDP_RTC_MUX_SEL, 0);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RE);
    sd_pin_rtc_bits(qts, SD_PIN_CMD, 0, SDP_RTC_MUX_SEL);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);

    /* CMD open drain: with its pull-up, then without */
    qtest_writel(qts, SDP_PIN(15), SDP_PIN_PAD_DRIVER);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);
    sd_pin_rtc_bits(qts, SD_PIN_CMD, 0, SDP_RTC_RUE);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RE);
    sd_pin_rtc_bits(qts, SD_PIN_CMD, SDP_RTC_RUE, 0);
    qtest_writel(qts, SDP_PIN(15), 0);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);
    qtest_quit(qts);
    unlink(img);
}

/*
 * The DAT lines at 4 bits: DAT2 on the GPIO function reads 1 at the
 * host, a read block's CRC failing on it (DCRC), and reaches no card,
 * which finds a written block's CRC wrong and discards it (DCRC), in a
 * multiple block write the rest too (EBE, no CRC status) until CMD12;
 * DAT1's input disabled reads 0, a read block's end bit failing (EBE),
 * its writes reaching the card; DAT0's input disabled holds the card busy
 * and makes a written block's CRC status 000 (DCRC), the card taking it;
 * DAT0 on the GPIO function reads 1, hiding a read block's start bit
 * (DRTO), the card having sent it.
 */
/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_data_lines(void)
{
    g_autofree char *img = make_image(0x33);
    QTestState *qts = start("", img, "-global sd-card.timing=off");
    uint32_t st, rca;
    uint8_t buf[512];

    sd_pins(qts);
    rca = card_init(qts);

    /* DAT2 on the GPIO function */
    mux_bits(qts, SD_PIN_D2, SDP_MCU_SEL(SDP_FUNC_GPIO), SDP_MCU_SEL_MASK);
    st = read_block(qts, 3 * 512, buf);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, image_byte(0x33, 3 * 512 + i) | 0x44);
    }
    st = write_blocks(qts, 3 * 512, 1, 0xa5a5a5a5);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);
    st = write_blocks(qts, 6 * 512, 2, 0x5a5a5a5a);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC | INT_EBE);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_RCV);
    cmd_ok(qts, R1 | CMD_STOP, 12, 0);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);
    sd_pin_iomux(qts, SD_PIN_D2);
    for (unsigned b = 3; b <= 7; b += 3) {
        g_assert_cmphex(read_block(qts, b * 512, buf) & INT_ERRORS, ==, 0);
        g_assert_cmphex(buf[100], ==, image_byte(0x33, b * 512 + 100));
    }

    /* DAT1's input disabled */
    mux_bits(qts, SD_PIN_D1, 0, SDP_FUN_IE);
    st = read_block(qts, 3 * 512, buf);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_EBE);
    g_assert_cmphex(buf[9], ==, image_byte(0x33, 3 * 512 + 9) & ~0x22);
    g_assert_cmphex(write_blocks(qts, 3 * 512, 1, 0x12345678) & INT_ERRORS,
                    ==, 0);
    mux_bits(qts, SD_PIN_D1, SDP_FUN_IE, 0);
    read_block(qts, 3 * 512, buf);
    g_assert_cmphex(le32(buf), ==, 0x12345678);

    /* DAT0's input disabled */
    mux_bits(qts, SD_PIN_D0, 0, SDP_FUN_IE);
    g_assert_true(busy(qts));
    st = write_blocks(qts, 4 * 512, 1, 0x87654321);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC);
    mux_bits(qts, SD_PIN_D0, SDP_FUN_IE, 0);
    g_assert_false(busy(qts));
    g_assert_cmphex(read_block(qts, 4 * 512, buf) & INT_ERRORS, ==, 0);
    g_assert_cmphex(le32(buf), ==, 0x87654321);

    /* DAT0 on the GPIO function */
    mux_bits(qts, SD_PIN_D0, SDP_MCU_SEL(SDP_FUNC_GPIO), SDP_MCU_SEL_MASK);
    qtest_writel(qts, SD_TMOUT, (1000u << 8) | 0x40);
    st = read_block(qts, 4 * 512, buf);
    g_assert_cmphex(st & (INT_DRTO | INT_DTO), ==, INT_DRTO | INT_DTO);
    sd_pin_iomux(qts, SD_PIN_D0);
    qtest_writel(qts, SD_TMOUT, 0xffffff40);
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);
    g_assert_cmphex(read_block(qts, 5 * 512, buf) & INT_ERRORS, ==, 0);
    g_assert_cmphex(buf[3], ==, image_byte(0x33, 5 * 512 + 3));
    qtest_quit(qts);
    unlink(img);
}

/*
 * DAT0: a write's busy, the card programming, reaches the host through
 * DAT0's pad, and only through it: on the GPIO function DAT0 reads 1, the
 * card still programming. A released DAT0 reads as its pulls hold it: the
 * pad's 45 kOhm pull-up (RTCIO's RUE) high, none floating low and busy,
 * the pull-up against the pad's pull-down (RDE) between VIL and VIH, low
 * and busy; the IO_MUX's FUN_WPD does nothing on this RTC pad. A
 * WAIT_PRVDATA command is not taken while DAT0 reads low, and goes out
 * once it reads high.
 */
/* [spec:nuos:req:emu.esp32.gpio/test] */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_dat0(void)
{
    g_autofree char *img = make_image(0x44);
    QTestState *qts = start("", img, "");
    uint32_t rca;

    sd_pins(qts);
    rca = card_init(qts);
    g_assert_false(busy(qts));
    g_assert_cmphex(write_blocks(qts, 4 * 512, 1, 0x0badf00d) & INT_ERRORS,
                    ==, 0);
    g_assert_true(busy(qts));
    mux_bits(qts, SD_PIN_D0, SDP_MCU_SEL(SDP_FUNC_GPIO), SDP_MCU_SEL_MASK);
    g_assert_false(busy(qts));
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_PRG);
    sd_pin_iomux(qts, SD_PIN_D0);
    g_assert_true(busy(qts));
    qtest_clock_step(qts, MS);
    g_assert_false(busy(qts));
    g_assert_cmpuint(card_state(qts, rca), ==, STATE_TRAN);

    mux_bits(qts, SD_PIN_D0, SDP_FUN_WPD, 0);
    g_assert_false(busy(qts));
    mux_bits(qts, SD_PIN_D0, 0, SDP_FUN_WPD);
    sd_pin_rtc_bits(qts, SD_PIN_D0, SDP_RTC_RDE, 0);
    g_assert_true(busy(qts));
    sd_pin_rtc_bits(qts, SD_PIN_D0, 0, SDP_RTC_RUE | SDP_RTC_RDE);
    g_assert_true(busy(qts));

    qtest_writel(qts, SD_CMDARG, rca << 16);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_CARD(1) | 13);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, CMD_START);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_CMDD, ==, 0);
    sd_pin_rtc_bits(qts, SD_PIN_D0, SDP_RTC_RUE, 0);
    g_assert_cmphex(wait_int(qts, INT_CMDD), ==, INT_CMDD);
    g_assert_cmpuint(R1_STATE(qtest_readl(qts, SD_RESP(0))), ==, STATE_TRAN);
    qtest_quit(qts);
    unlink(img);
}

/*
 * The board's pull-ups (-M esp32,sd-pull-up=10000): GPIO15, 2, 4, 12 and
 * 13 read high, against the pads' 45 kOhm pull-downs too, CLK's GPIO14
 * not pulled; with them, ESP-IDF's set-up without its internal pull-ups
 * reaches the card. On the bare module (no sd-pull-up) the same set-up
 * leaves DAT0 floating, read low: the card busy, CMD8 waiting for it
 * never taken.
 */
/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_board_pulls(void)
{
    g_autofree char *img = make_image(0x55);
    QTestState *qts = start(",sd-pull-up=10000", img,
                            "-global sd-card.timing=off");
    uint8_t buf[512];

    for (unsigned pin = 0; pin < SD_PINS; pin++) {
        mux_bits(qts, pin, SDP_MCU_SEL(SDP_FUNC_GPIO) | SDP_FUN_IE,
                 SDP_MCU_SEL_MASK);
        sd_pin_rtc_bits(qts, pin, SDP_RTC_RDE, SDP_RTC_RUE);
        g_assert_cmpint(gpio_in(qts, sd_pin[pin].pad), ==,
                        pin != SD_PIN_CLK);
    }
    for (unsigned pin = 0; pin < SD_PINS; pin++) {
        sd_pin_iomux(qts, pin);
    }
    sd_side_band(qts);
    card_init(qts);
    g_assert_cmphex(read_block(qts, 2 * 512, buf) & INT_ERRORS, ==, 0);
    g_assert_cmphex(buf[0], ==, image_byte(0x55, 2 * 512));
    qtest_quit(qts);

    qts = start("", img, "-global sd-card.timing=off");
    for (unsigned pin = 0; pin < SD_PINS; pin++) {
        sd_pin_iomux(qts, pin);
    }
    sd_side_band(qts);
    card_clock(qts, 1);
    g_assert_true(busy(qts));
    cmd(qts, 1, CMD_INIT, 0, 0);
    qtest_writel(qts, SD_CMDARG, 0x1aa);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_CARD(1) | 8);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, CMD_START);
    qtest_quit(qts);
    unlink(img);
}

/*
 * Slot 0's CLK, CMD and DAT0 to DAT3 are the module's flash lines: its bus
 * takes no card (device_add finds it full), and a command to it, on its
 * pads at reset or on function 3 as ESP-IDF would set them, goes
 * unanswered (RTO), GPIO11's pull-up holding CMD high. -device sd-card
 * reaches slot 1's bus.
 */
/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_slot0(void)
{
    static const uint8_t slot0_mux[] = { 0x60, 0x5c, 0x64, 0x68, 0x54, 0x58 };
    g_autofree char *img = make_image(0x66);
    g_autofree char *extra = g_strdup_printf(
        "-drive if=none,id=sd1,format=raw,file=%s,file.locking=off "
        "-device sd-card,drive=sd1,bus=sd-bus1 -global sd-card.timing=off",
        img);
    QTestState *qts = qtest_initf("-M esp32 -nic none %s", extra);
    uint8_t buf[512];
    QDict *rsp;

    qtest_writel(qts, RTC_CNTL_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, RTC_CNTL_WDTCONFIG0, 0);
    qtest_writel(qts, TIMG0_WDTWPROTECT, WDT_WKEY);
    qtest_writel(qts, TIMG0_WDTCONFIG0, 0);
    qtest_writel(qts, DPORT_WIFI_CLK_EN,
                 qtest_readl(qts, DPORT_WIFI_CLK_EN) | WIFI_CLK_SDIO_HOST);
    qtest_writel(qts, DPORT_CORE_RST_EN,
                 qtest_readl(qts, DPORT_CORE_RST_EN) & ~CORE_RST_SDIO_HOST);

    rsp = qtest_qmp(qts, "{ 'execute': 'device_add', 'arguments': "
                    "{ 'driver': 'sd-card', 'bus': 'sd-bus0' } }");
    g_assert_true(qdict_haskey(rsp, "error"));
    g_assert_nonnull(strstr(qdict_get_str(qdict_get_qdict(rsp, "error"),
                                          "desc"), "full"));
    qobject_unref(rsp);

    card_clock(qts, 0);
    cmd(qts, 0, CMD_INIT, 0, 0);
    g_assert_cmphex(cmd(qts, 0, R1, 8, 0x1aa), ==, INT_CMDD | INT_RTO);
    for (unsigned i = 0; i < ARRAY_SIZE(slot0_mux); i++) {
        uint32_t addr = SDP_IO_MUX + slot0_mux[i];

        qtest_writel(qts, addr, (qtest_readl(qts, addr) & ~SDP_MCU_SEL_MASK) |
                     SDP_MCU_SEL(SDP_FUNC_SD) | SDP_FUN_IE);
    }
    g_assert_cmphex(cmd(qts, 0, R1, 8, 0x1aa), ==, INT_CMDD | INT_RTO);

    sd_pins(qts);
    card_init(qts);
    g_assert_cmphex(read_block(qts, 9 * 512, buf) & INT_ERRORS, ==, 0);
    g_assert_cmphex(buf[77], ==, image_byte(0x66, 9 * 512 + 77));
    qtest_quit(qts);
    unlink(img);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/sd-pins/reset", test_reset);
    qtest_add_func("/esp32/sd-pins/cmd", test_cmd);
    qtest_add_func("/esp32/sd-pins/data-lines", test_data_lines);
    qtest_add_func("/esp32/sd-pins/dat0", test_dat0);
    qtest_add_func("/esp32/sd-pins/board-pulls", test_board_pulls);
    qtest_add_func("/esp32/sd-pins/slot0", test_slot0);
    return g_test_run();
}
