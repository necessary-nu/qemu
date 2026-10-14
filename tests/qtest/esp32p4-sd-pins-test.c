/*
 * QTest testcase for the ESP32-P4 SD/MMC host's pins: the cards reached
 * only through pads set up as the hardware needs them
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Slot 0 is the board's microSD socket on its IO MUX pins (CLK GPIO43,
 * CMD GPIO44, DAT0 to DAT3 GPIO39 to GPIO42), with the board's 51 kOhm
 * pull-ups to VO4 on CMD and DAT; slot 1's card stands where the
 * ESP32-C6 is, its lines through the GPIO matrix on GPIO18, GPIO19 and
 * GPIO14 to GPIO17. Set up as ESP-IDF sets them up (esp32p4-sd-pins.h),
 * the cards answer; set up otherwise, each as the board would have it:
 * pads off their function reach no card (RTO), inputs disabled read 0
 * (responses of 0s, RE; DAT0 busy; data 0s, EBE), a DAT line off its
 * function a CRC error both ways, the card discarding a block it found
 * wrong, and slot 1's signals unrouted reaching nothing.
 *
 * The controller runs at 40 MHz (25 ns) from PLL_F160M_CLK, the cards at
 * 400 kHz for identification and 10 MHz for data; the cards are raw
 * images of 1 MiB, standard capacity, byte addressed, that take no time.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32p4-sd-pins.h"

#define SD                  0x50083000
#define SD_CTRL             (SD + 0x000)
#define SD_CLKDIV           (SD + 0x008)
#define SD_CLKSRC           (SD + 0x00c)
#define SD_CLKENA           (SD + 0x010)
#define SD_TMOUT            (SD + 0x014)
#define SD_CTYPE            (SD + 0x018)
#define SD_BLKSIZ           (SD + 0x01c)
#define SD_BYTCNT           (SD + 0x020)
#define SD_CMDARG           (SD + 0x028)
#define SD_CMD              (SD + 0x02c)
#define SD_RESP(n)          (SD + 0x030 + 4 * (n))
#define SD_RINTSTS          (SD + 0x044)
#define SD_STATUS           (SD + 0x048)
#define SD_BUFFIFO          (SD + 0x200)

#define CTRL_RESETS         7u

#define CMD_RESP            (1u << 6)
#define CMD_LONG            (1u << 7)
#define CMD_CRC             (1u << 8)
#define CMD_DATA            (1u << 9)
#define CMD_WRITE           (1u << 10)
#define CMD_WAIT            (1u << 13)
#define CMD_STOP            (1u << 14)
#define CMD_INIT            (1u << 15)
#define CMD_CARD(n)         ((n) << 16)
#define CMD_UPDATE_CLK      (1u << 21)
#define CMD_HOLD            (1u << 29)
#define CMD_START           (1u << 31)
#define R1                  (CMD_RESP | CMD_CRC)
#define R2                  (CMD_RESP | CMD_LONG | CMD_CRC)
#define R3                  CMD_RESP

#define INT_RE              (1u << 1)
#define INT_CMDD            (1u << 2)
#define INT_DTO             (1u << 3)
#define INT_RCRC            (1u << 6)
#define INT_DCRC            (1u << 7)
#define INT_RTO             (1u << 8)
#define INT_DRTO            (1u << 9)
#define INT_EBE             (1u << 15)
#define INT_ERRORS          (INT_RE | INT_RCRC | INT_DCRC | INT_RTO | \
                             INT_DRTO | INT_EBE)

#define STATUS_DATA3        (1u << 8)
#define STATUS_BUSY         (1u << 9)

#define R1_STATE(r)         (((r) >> 9) & 0xf)
#define STATE_STBY          3
#define STATE_TRAN          4
#define STATE_RCV           6

/* HP_SYS_CLKRST: SDMMC_SYS_CLK, the SDIO_LS clocks from PLL_F160M_CLK */
#define SOC_CLK_CTRL1       0x500e6018
#define SDMMC_SYS_CLK_EN    (1u << 14)
#define PERI_CLK_CTRL01     0x500e6034
#define PERI_CLK_CTRL02     0x500e6038

/* The PMU's VO4: XPD, FORCE_TIEH_SEL (TIEH_SEL 0) and TIEH (bypass) */
#define PMU_EXT_LDO_VO4     0x501151d8
#define VO4_ON_3V3          ((1u << 7) | (1u << 8) | (1u << 14))

#define GPIO_IN             (SDP_GPIO + 0x03c)
#define GPIO_IN1            (SDP_GPIO + 0x040)
#define IN_INV              (1u << 6)

#define CCLK_NS             25
#define MS                  (1000 * 1000)
#define IMAGE_SIZE          (1 * 1024 * 1024)

static char *make_image(uint8_t seed)
{
    g_autofree uint8_t *data = g_malloc(IMAGE_SIZE);
    char *path;
    int fd;

    fd = g_file_open_tmp("esp32p4-sd-pins-XXXXXX.img", &path, NULL);
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

/* A card in @slot; no flash boot watchdogs, the cards untimed. */
static QTestState *start(unsigned slot, const char *img)
{
    return qtest_initf("-M esp32p4,strap-mode=0 "
                       "-drive if=sd,index=%u,format=raw,file=%s "
                       "-global sd-card.timing=off", slot, img);
}

/* SDMMC_SYS_CLK, the SDIO_LS clocks at 40 MHz, slot 0's pads at 3.3 V. */
static void clocks_on(QTestState *qts)
{
    uint32_t edges = (3u << 17) | (3u << 9) | (1u << 13) | (7u << 27);

    qtest_writel(qts, PMU_EXT_LDO_VO4,
                 qtest_readl(qts, PMU_EXT_LDO_VO4) | VO4_ON_3V3);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | SDMMC_SYS_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | (1u << 24));
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_writel(qts, PERI_CLK_CTRL02, edges | (1u << 8));
}

/* Card @card's clock: cclk_in divided by 2 * @div. */
static void card_clock(QTestState *qts, unsigned card, unsigned div)
{
    qtest_writel(qts, SD_CLKDIV, div << (8 * card));
    qtest_writel(qts, SD_CLKSRC, card << (2 * card));
    qtest_writel(qts, SD_CLKENA, 1u << card);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_WAIT |
                 CMD_HOLD | CMD_CARD(card));
    qtest_clock_step(qts, 2 * CCLK_NS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
}

/* Run until a RINTSTS bit in @mask is set: the bits, cleared. */
static uint32_t wait_int(QTestState *qts, uint32_t mask, int64_t max_ns)
{
    for (int64_t t = 0; t <= max_ns; t += 1000) {
        uint32_t st = qtest_readl(qts, SD_RINTSTS);

        if (st & mask) {
            qtest_writel(qts, SD_RINTSTS, st);
            return st;
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
    return wait_int(qts, INT_CMDD, 100 * MS);
}

static void cmd_ok(QTestState *qts, unsigned card, uint32_t flags,
                   unsigned index, uint32_t arg)
{
    g_assert_cmphex(cmd(qts, card, flags, index, arg) & INT_ERRORS, ==, 0);
}

/* CMD13: the card's state, as its R1 gives it. */
static unsigned card_state(QTestState *qts, unsigned card, uint32_t rca)
{
    cmd_ok(qts, card, R1, 13, rca << 16);
    return R1_STATE(qtest_readl(qts, SD_RESP(0)));
}

/* Identify and select card @card, its bus 4 bits wide at 10 MHz: its RCA */
static uint32_t card_init(QTestState *qts, unsigned card)
{
    uint32_t ocr = 0, rca;

    card_clock(qts, card, 50);
    cmd(qts, card, CMD_INIT, 0, 0);
    cmd_ok(qts, card, R1 | CMD_WAIT, 8, 0x1aa);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0x1aa);
    for (int tries = 0; tries < 20 && !(ocr & (1u << 31)); tries++) {
        cmd_ok(qts, card, R1 | CMD_WAIT, 55, 0);
        cmd(qts, card, R3 | CMD_WAIT, 41, 0x40ff8000);
        ocr = qtest_readl(qts, SD_RESP(0));
    }
    g_assert_cmphex(ocr & (1u << 31), !=, 0);
    cmd_ok(qts, card, R2 | CMD_WAIT, 2, 0);
    cmd_ok(qts, card, R1 | CMD_WAIT, 3, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    cmd_ok(qts, card, R1 | CMD_WAIT, 7, rca << 16);
    cmd_ok(qts, card, R1 | CMD_WAIT, 55, rca << 16);
    cmd_ok(qts, card, R1 | CMD_WAIT, 6, 2);
    qtest_writel(qts, SD_CTYPE, 1u << card);
    card_clock(qts, card, 2);
    return rca;
}

/* CMD17 of the block at @addr through the FIFO: RINTSTS to its end. */
static uint32_t read_block(QTestState *qts, unsigned card, uint32_t addr,
                           uint8_t *buf)
{
    uint32_t all = 0;

    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 512);
    qtest_writel(qts, SD_CMDARG, addr);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | CMD_DATA |
                 CMD_CARD(card) | 17);
    while (!(all & (INT_DTO | INT_DRTO))) {
        all |= wait_int(qts, INT_DTO | INT_DRTO | INT_CMDD, 100 * MS);
    }
    for (int i = 0; i < 512; i += 4) {
        uint32_t w = qtest_readl(qts, SD_BUFFIFO);

        memcpy(&buf[i], &w, 4);
    }
    qtest_writel(qts, SD_CTRL, CTRL_RESETS);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    return all;
}

/*
 * CMD24 (or, with @blocks 2, CMD25) of @blocks blocks of @fill at @addr
 * through the FIFO: RINTSTS to the data's end.
 */
static uint32_t write_blocks(QTestState *qts, unsigned card, uint32_t addr,
                             unsigned blocks, uint32_t fill)
{
    uint32_t all;

    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 512 * blocks);
    qtest_writel(qts, SD_CMDARG, addr);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | CMD_DATA |
                 CMD_WRITE | CMD_CARD(card) | (blocks > 1 ? 25 : 24));
    all = wait_int(qts, INT_CMDD, 100 * MS);
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
    all |= wait_int(qts, INT_DTO, 100 * MS);
    qtest_writel(qts, SD_CTRL, CTRL_RESETS);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    return all;
}

static uint32_t io_mux(QTestState *qts, unsigned pad)
{
    return qtest_readl(qts, SDP_IO_MUX(pad));
}

static uint32_t le32(const uint8_t *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static bool gpio_in(QTestState *qts, unsigned pad)
{
    return qtest_readl(qts, pad < 32 ? GPIO_IN : GPIO_IN1) >> (pad % 32) & 1;
}

/*
 * Slot 0 from reset: its pads are on function 0 (SD1_*) with their
 * inputs disabled. Commands reach the card, but its responses read 0s
 * (RE) and DAT0 reads low, busy: a command waiting for DAT0 is not taken
 * until DAT0's input is enabled. Set up as ESP-IDF sets it up, the card
 * answers what it took meanwhile. On the GPIO function the pads reach no
 * card (RTO), their SD1 inputs reading 1, the card's state unchanged.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_slot0_setup(void)
{
    g_autofree char *img = make_image(0x11);
    QTestState *qts = start(0, img);
    uint32_t st, rca;
    uint8_t buf[512];

    clocks_on(qts);
    g_assert_cmphex(io_mux(qts, 44) & (SDP_MCU_SEL(7) | SDP_FUN_IE), ==, 0);
    card_clock(qts, 0, 50);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==,
                    STATUS_BUSY);
    cmd(qts, 0, CMD_INIT, 0, 0);
    g_assert_cmphex(cmd(qts, 0, R1, 8, 0x1aa), ==, INT_CMDD | INT_RE);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0);
    /* Waiting for DAT0, the command is not taken... */
    qtest_writel(qts, SD_CMDARG, 0x1aa);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 | 8);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, CMD_START);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    /* ...until DAT0's input is enabled; CMD's still reads 0s. */
    sd_pin_iomux(qts, 39);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==, 0);
    g_assert_cmphex(wait_int(qts, INT_CMDD, MS), ==, INT_CMDD | INT_RE);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);

    /* As ESP-IDF sets them up: the card took CMD8, and answers. */
    sd_pins(qts, 0);
    cmd_ok(qts, 0, R1 | CMD_WAIT, 8, 0x1aa);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0x1aa);
    rca = card_init(qts, 0);
    st = read_block(qts, 0, 5 * 512, buf);
    g_assert_cmphex(st & INT_ERRORS, ==, 0);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, image_byte(0x11, 5 * 512 + i));
    }

    /* The GPIO function: no card; the SD1 inputs read 1 (I1). */
    for (unsigned i = 0; i < SD_PINS; i++) {
        unsigned pad = sd_pin_pad[0][i];

        qtest_writel(qts, SDP_IO_MUX(pad),
                     (io_mux(qts, pad) & ~SDP_MCU_SEL(7)) | SDP_MCU_SEL(1));
    }
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & (STATUS_BUSY |
                                                   STATUS_DATA3), ==,
                    STATUS_DATA3);
    g_assert_cmphex(cmd(qts, 0, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    g_assert_cmphex(cmd(qts, 0, R1, 7, 0), ==, INT_CMDD | INT_RTO);
    qtest_writel(qts, SD_TMOUT, (1000u << 8) | 0x40);
    st = read_block(qts, 0, 0, buf);
    g_assert_cmphex(st & (INT_RTO | INT_DRTO), ==, INT_RTO | INT_DRTO);
    /* Back on function 0: still selected, CMD7 never having reached it. */
    sd_pins(qts, 0);
    g_assert_cmpuint(card_state(qts, 0, rca), ==, STATE_TRAN);
    qtest_quit(qts);
    unlink(img);
}

/*
 * Slot 0's DAT lines at 4 bits: DAT2 on the GPIO function reads 1 at the
 * host, the block's CRC failing on it (DCRC), and reaches no card, which
 * finds a written block's CRC wrong and discards it (DCRC, the negative
 * CRC status), and in a multiple block write the rest (EBE, no CRC
 * status), until CMD12; DAT1's input disabled reads 0, the block's end
 * bit failing (EBE), its writes reaching the card; DAT0's disabled holds
 * the card busy and makes a written block's CRC status 000 (DCRC), the
 * card taking it. The pad's own pull-down against the board's 51 kOhm
 * pull-up leaves a released DAT0 between VIL and VIH: it reads low, busy.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_slot0_data_lines(void)
{
    g_autofree char *img = make_image(0x22);
    QTestState *qts = start(0, img);
    uint32_t st, rca;
    uint8_t buf[512];

    clocks_on(qts);
    sd_pins(qts, 0);
    rca = card_init(qts, 0);

    /* DAT2 on the GPIO function */
    qtest_writel(qts, SDP_IO_MUX(41), SDP_MCU_SEL(1) | SDP_FUN_IE);
    st = read_block(qts, 0, 3 * 512, buf);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, image_byte(0x22, 3 * 512 + i) | 0x44);
    }
    st = write_blocks(qts, 0, 3 * 512, 1, 0xa5a5a5a5);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC);
    g_assert_cmpuint(card_state(qts, 0, rca), ==, STATE_TRAN);
    st = write_blocks(qts, 0, 6 * 512, 2, 0x5a5a5a5a);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC | INT_EBE);
    g_assert_cmpuint(card_state(qts, 0, rca), ==, STATE_RCV);
    cmd_ok(qts, 0, R1 | CMD_STOP, 12, 0);
    g_assert_cmpuint(card_state(qts, 0, rca), ==, STATE_TRAN);
    sd_pin_iomux(qts, 41);
    for (unsigned b = 3; b <= 7; b += 3) {
        g_assert_cmphex(read_block(qts, 0, b * 512, buf) & INT_ERRORS, ==,
                        0);
        g_assert_cmphex(buf[100], ==, image_byte(0x22, b * 512 + 100));
    }

    /* DAT1's input disabled */
    qtest_writel(qts, SDP_IO_MUX(40), io_mux(qts, 40) & ~SDP_FUN_IE);
    st = read_block(qts, 0, 3 * 512, buf);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_EBE);
    g_assert_cmphex(buf[9], ==, image_byte(0x22, 3 * 512 + 9) & ~0x22);
    g_assert_cmphex(write_blocks(qts, 0, 3 * 512, 1, 0x12345678) &
                    INT_ERRORS, ==, 0);
    sd_pin_iomux(qts, 40);
    read_block(qts, 0, 3 * 512, buf);
    g_assert_cmphex(le32(buf), ==, 0x12345678);

    /* DAT0's input disabled */
    qtest_writel(qts, SDP_IO_MUX(39), io_mux(qts, 39) & ~SDP_FUN_IE);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==,
                    STATUS_BUSY);
    st = write_blocks(qts, 0, 4 * 512, 1, 0x87654321);
    g_assert_cmphex(st & INT_ERRORS, ==, INT_DCRC);
    sd_pin_iomux(qts, 39);
    g_assert_cmphex(read_block(qts, 0, 4 * 512, buf) & INT_ERRORS, ==, 0);
    g_assert_cmphex(le32(buf), ==, 0x87654321);

    /* DAT0's own pull-down against the board's pull-up */
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==, 0);
    qtest_writel(qts, SDP_IO_MUX(39), io_mux(qts, 39) | SDP_FUN_WPD);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==,
                    STATUS_BUSY);
    qtest_writel(qts, SDP_IO_MUX(39), io_mux(qts, 39) & ~SDP_FUN_WPD);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==, 0);
    qtest_quit(qts);
    unlink(img);
}

/*
 * Slot 1, a card where the ESP32-C6 is: its signals unrouted reach no card
 * and read their default, 1 (RTO, not busy); routed through the matrix as
 * ESP-IDF routes them, it answers. CMD's output enable from GPIO_ENABLE
 * (OEN_SEL) clear reaches no card, set drives the pad against the card's
 * response (RTO); CMD's input inverted reads the pulled-up pad as 0s (RE);
 * DAT0's input left to its default hides a block's start bit (DRTO).
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_slot1(void)
{
    g_autofree char *img = make_image(0x33);
    QTestState *qts = start(1, img);
    uint32_t st, rca;
    uint8_t buf[512];

    clocks_on(qts);
    card_clock(qts, 1, 50);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==, 0);
    cmd(qts, 1, CMD_INIT, 0, 0);
    g_assert_cmphex(cmd(qts, 1, R1, 8, 0x1aa), ==, INT_CMDD | INT_RTO);

    sd_pins(qts, 1);
    rca = card_init(qts, 1);
    st = read_block(qts, 1, 9 * 512, buf);
    g_assert_cmphex(st & INT_ERRORS, ==, 0);
    g_assert_cmphex(buf[77], ==, image_byte(0x33, 9 * 512 + 77));

    /* CMD's enable from GPIO_ENABLE: clear, then set */
    qtest_writel(qts, SDP_FUNC_OUT(19), 1 | SDP_OUT_OEN_SEL);
    qtest_writel(qts, SDP_ENABLE_W1TC, 1u << 19);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    qtest_writel(qts, SDP_ENABLE_W1TS, 1u << 19);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    qtest_writel(qts, SDP_FUNC_OUT(19), 1);
    g_assert_cmpuint(card_state(qts, 1, rca), ==, STATE_TRAN);

    /* CMD's input inverted */
    qtest_writel(qts, SDP_FUNC_IN(1), SDP_IN_SIG_IN_SEL | IN_INV | 19);
    g_assert_cmphex(cmd(qts, 1, R1, 13, rca << 16), ==, INT_CMDD | INT_RE);
    qtest_writel(qts, SDP_FUNC_IN(1), SDP_IN_SIG_IN_SEL | 19);

    /* DAT0's input bypassing the matrix: its default, 1 */
    qtest_writel(qts, SDP_FUNC_IN(2), 0);
    qtest_writel(qts, SD_TMOUT, (1000u << 8) | 0x40);
    st = read_block(qts, 1, 9 * 512, buf);
    g_assert_cmphex(st & (INT_DRTO | INT_DTO), ==, INT_DRTO | INT_DTO);
    qtest_writel(qts, SDP_FUNC_IN(2), SDP_IN_SIG_IN_SEL | 14);
    g_assert_cmphex(read_block(qts, 1, 10 * 512, buf) & INT_ERRORS, ==, 0);
    qtest_quit(qts);
    unlink(img);
}

/*
 * The SD lines' pads as GPIO_IN reads them: slot 0's CMD and DAT pulled
 * up to VO4, low with VO4 off; CLK with no pull; GPIO45, the supply
 * switch's gate, pulled down by R27 against its own pull-up; slot 1's
 * pads, the ESP32-C6's, pulled up by the board, with a card there or
 * none.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_board_pulls(void)
{
    g_autofree char *img = make_image(0x44);
    QTestState *qts = start(1, img);

    for (unsigned i = 0; i < SD_PINS; i++) {
        unsigned pad = sd_pin_pad[0][i];

        sd_pin_iomux(qts, pad);
        g_assert_false(gpio_in(qts, pad));
    }
    clocks_on(qts);
    for (unsigned i = 0; i < SD_PINS; i++) {
        unsigned pad = sd_pin_pad[0][i];

        g_assert_cmpint(gpio_in(qts, pad), ==, i != SD_PIN_CLK);
    }
    qtest_writel(qts, SDP_IO_MUX(45), SDP_MCU_SEL(1) | SDP_FUN_IE |
                 SDP_FUN_WPU);
    g_assert_false(gpio_in(qts, 45));
    for (unsigned i = 0; i < SD_PINS; i++) {
        unsigned pad = sd_pin_pad[1][i];

        qtest_writel(qts, SDP_IO_MUX(pad), SDP_MCU_SEL(1) | SDP_FUN_IE);
        g_assert_true(gpio_in(qts, pad));
    }
    qtest_quit(qts);

    qts = start(0, img);
    for (unsigned i = 0; i < SD_PINS; i++) {
        unsigned pad = sd_pin_pad[1][i];

        qtest_writel(qts, SDP_IO_MUX(pad), SDP_MCU_SEL(1) | SDP_FUN_IE);
        g_assert_true(gpio_in(qts, pad));
    }
    qtest_quit(qts);
    unlink(img);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/sd-pins/slot0-setup", test_slot0_setup);
    qtest_add_func("/esp32p4/sd-pins/slot0-data-lines",
                   test_slot0_data_lines);
    qtest_add_func("/esp32p4/sd-pins/slot1", test_slot1);
    qtest_add_func("/esp32p4/sd-pins/board-pulls", test_board_pulls);
    return g_test_run();
}
