/*
 * QTest testcase for SD cards that take time, on the ESP32-P4 machine's
 * SD/MMC host: access times, busy, the commands a busy card refuses, the
 * registers that state the times, and the specification's maximums
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The controller is clocked as ESP-IDF clocks it: SDIO_LS at 40 MHz
 * (25 ns), card 0's clock 400 kHz (2.5 us) for identification and 10 MHz
 * (100 ns) for data, 4 bits wide. The SDSC card is a raw image of 1 MiB:
 * Class 4, so its access and programming times are both its TAAC, 70 us,
 * its AU 512 KiB. The SDHC card is 4 GiB of zeros that keeps no writes.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

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
#define SD_FIFOTH           (SD + 0x04c)
#define SD_CARDTHRCTL       (SD + 0x100)
#define SD_BUFFIFO          (SD + 0x200)

#define CTRL_FIFO_RESET     (1u << 1)

#define CMD_RESP            (1u << 6)
#define CMD_LONG            (1u << 7)
#define CMD_CRC             (1u << 8)
#define CMD_DATA            (1u << 9)
#define CMD_WRITE           (1u << 10)
#define CMD_AUTO_STOP       (1u << 12)
#define CMD_WAIT            (1u << 13)
#define CMD_INIT            (1u << 15)
#define CMD_UPDATE_CLK      (1u << 21)
#define CMD_HOLD            (1u << 29)
#define CMD_START           (1u << 31)
#define R1                  (CMD_RESP | CMD_CRC)
#define R2                  (CMD_RESP | CMD_LONG | CMD_CRC)
#define R3                  CMD_RESP

#define INT_CMDD            (1u << 2)
#define INT_DTO             (1u << 3)
#define INT_RTO             (1u << 8)
#define INT_DRTO            (1u << 9)
#define INT_BCI             (1u << 13)
#define INT_ACD             (1u << 14)

#define STATUS_DATA_BUSY    (1u << 9)

#define CARDTHRCTL_BUSY_CLR (1u << 1)

/* R1's card status */
#define R1_STATE(r)         (((r) >> 9) & 0xf)
#define R1_READY            (1u << 8)
#define R1_ILLEGAL          (1u << 22)
#define STATE_STBY          3
#define STATE_TRAN          4
#define STATE_DATA          5
#define STATE_PRG           7

/* SDMMC's clock */
#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SDMMC_SYS_CLK_EN    (1u << 14)
#define PERI_CLK_CTRL01     (HP_CLKRST + 0x34)
#define PERI_CLK_CTRL02     (HP_CLKRST + 0x38)

#define CCLK_NS             25
#define ID_CLK_NS           2500
#define CLK_NS              100

/* The card's times: CIU sync, a command, and one with its response */
#define TAKE_NS             (2 * CCLK_NS)
#define CMD_NS              (48 * CLK_NS)
#define CMD_RESP_NS         (98 * CLK_NS)
/*
 * A block read (start bit, data, CRC, end bit), and one written (Nwr
 * before it, its CRC status after)
 */
#define RBLOCK_NS           ((1 + 1024 + 16 + 1) * CLK_NS)
#define WBLOCK_NS           ((2 + 1 + 1024 + 16 + 1 + 8) * CLK_NS)
/* The SDSC card's TAAC, its access and its programming time */
#define TAAC_NS             70000

#define IMAGE_SIZE          (1 * 1024 * 1024)

static char *make_image(uint8_t seed)
{
    g_autofree uint8_t *data = g_malloc(IMAGE_SIZE);
    char *path;
    int fd;

    fd = g_file_open_tmp("esp32p4-sd-card-XXXXXX.img", &path, NULL);
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

/* The new contents a test writes to a block */
static uint8_t new_byte(int i)
{
    return (uint8_t)(0xa5 ^ (i * 13));
}

static QTestState *start(const char *img, const char *opts)
{
    return qtest_initf("-M esp32p4,strap-mode=0 "
                       "-drive if=sd,index=0,format=raw,file=%s %s",
                       img, opts);
}

/* SDMMC_SYS_CLK, and the SDIO_LS clocks at 40 MHz. */
static void clocks_on(QTestState *qts)
{
    uint32_t edges = (3u << 17) | (3u << 9) | (1u << 13) | (7u << 27);

    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | SDMMC_SYS_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | (1u << 24));
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_writel(qts, PERI_CLK_CTRL02, edges | (1u << 8));
}

static void card_clock(QTestState *qts, unsigned div)
{
    qtest_writel(qts, SD_CLKDIV, div);
    qtest_writel(qts, SD_CLKSRC, 0);
    qtest_writel(qts, SD_CLKENA, 1);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_WAIT |
                 CMD_HOLD);
    qtest_clock_step(qts, TAKE_NS);
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

/* Start a command, no waiting. */
static void cmd_start(QTestState *qts, uint32_t flags, unsigned index,
                      uint32_t arg)
{
    qtest_writel(qts, SD_CMDARG, arg);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | flags | index);
}

/* A command and its end: RINTSTS, cleared. */
static uint32_t cmd(QTestState *qts, uint32_t flags, unsigned index,
                    uint32_t arg)
{
    cmd_start(qts, flags, index, arg);
    return wait_int(qts, INT_CMDD, 2000LL * 1000 * 1000);
}

/* CMD13 now, without waiting for the card: its R1 */
static uint32_t status_now(QTestState *qts, uint32_t rca)
{
    uint32_t st = cmd(qts, R1, 13, rca << 16);

    g_assert_cmphex(st & INT_RTO, ==, 0);
    return qtest_readl(qts, SD_RESP(0));
}

/* Identify the card, its CSD in @csd, select it: its RCA. */
static uint32_t card_init(QTestState *qts, uint32_t *csd)
{
    uint32_t st, rca;
    int tries;

    card_clock(qts, 50);
    cmd(qts, CMD_INIT, 0, 0);
    st = cmd(qts, R1 | CMD_WAIT, 8, 0x1aa);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    for (tries = 0; tries < 20; tries++) {
        cmd(qts, R1 | CMD_WAIT, 55, 0);
        cmd(qts, R3 | CMD_WAIT, 41, 0x40ff8000);
        if (qtest_readl(qts, SD_RESP(0)) & 0x80000000) {
            break;
        }
        qtest_clock_step(qts, 1000 * 1000);
    }
    g_assert_cmpint(tries, <, 20);
    cmd(qts, R2 | CMD_WAIT, 2, 0);
    cmd(qts, R1 | CMD_WAIT, 3, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    g_assert_cmphex(rca, !=, 0);
    st = cmd(qts, R2 | CMD_WAIT, 9, rca << 16);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    if (csd) {
        for (int i = 0; i < 4; i++) {
            csd[i] = qtest_readl(qts, SD_RESP(i));
        }
    }
    st = cmd(qts, R1 | CMD_WAIT, 7, rca << 16);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    cmd(qts, R1 | CMD_WAIT, 55, rca << 16);
    cmd(qts, R1 | CMD_WAIT, 6, 2);
    qtest_writel(qts, SD_CTYPE, 1);
    card_clock(qts, 2);
    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_FIFOTH, (7u << 16) | 8);
    return rca;
}

/* CSD bits [hi:lo] from RESP0 to RESP3 (bit 0 the CRC's end bit) */
static uint32_t csd_bits(const uint32_t *csd, int hi, int lo)
{
    uint32_t v = 0;

    for (int b = hi; b >= lo; b--) {
        v = v << 1 | ((csd[b / 32] >> (b % 32)) & 1);
    }
    return v;
}

/* Read @len bytes of a register (ACMD @index) through the FIFO. */
static void read_reg(QTestState *qts, uint32_t rca, unsigned index,
                     uint8_t *buf, unsigned len)
{
    uint32_t st;

    cmd(qts, R1 | CMD_WAIT, 55, rca << 16);
    qtest_writel(qts, SD_BLKSIZ, len);
    qtest_writel(qts, SD_BYTCNT, len);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA, index, 0);
    st = wait_int(qts, INT_DTO, 1000LL * 1000 * 1000);
    g_assert_cmphex(st & INT_DRTO, ==, 0);
    for (unsigned i = 0; i < len; i += 4) {
        uint32_t w = qtest_readl(qts, SD_BUFFIFO);

        memcpy(&buf[i], &w, 4);
    }
    qtest_writel(qts, SD_BLKSIZ, 512);
}

/* Write one block through the FIFO, CMD24 started at once, no waiting. */
static void write_start(QTestState *qts, uint32_t block, uint32_t flags)
{
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd_start(qts, R1 | CMD_DATA | CMD_WRITE | flags, 24, block * 512);
    qtest_clock_step(qts, TAKE_NS);
    for (int i = 0; i < 128; i++) {
        uint32_t w = new_byte(4 * i) | new_byte(4 * i + 1) << 8 |
                     new_byte(4 * i + 2) << 16 |
                     (uint32_t)new_byte(4 * i + 3) << 24;

        qtest_writel(qts, SD_BUFFIFO, w);
    }
}

/* Read one block through the FIFO into @buf. */
static void read_block(QTestState *qts, uint32_t block, uint8_t *buf)
{
    uint32_t st;

    qtest_writel(qts, SD_BYTCNT, 512);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA, 17, block * 512);
    st = wait_int(qts, INT_DTO, 3000LL * 1000 * 1000);
    g_assert_cmphex(st & INT_DRTO, ==, 0);
    for (int i = 0; i < 128; i++) {
        uint32_t w = qtest_readl(qts, SD_BUFFIFO);

        memcpy(&buf[4 * i], &w, 4);
    }
}

static bool busy(QTestState *qts)
{
    return qtest_readl(qts, SD_STATUS) & STATUS_DATA_BUSY;
}

/*
 * The registers of a 1 MiB SDSC card state its times: CSD TAAC 70 us,
 * NSAC 0, R2W_FACTOR 1; SD Status Class 4, Pm infinite, AU 512 KiB, the
 * erase fields 128 AUs in 1 s from 1 s, no UHS grade; and SCR's erased
 * state, 1s.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_registers_sdsc(void)
{
    g_autofree char *img = make_image(0x11);
    QTestState *qts = start(img, "");
    uint32_t csd[4], rca;
    uint8_t ssr[64], scr[8];

    clocks_on(qts);
    rca = card_init(qts, csd);
    g_assert_cmphex(csd_bits(csd, 127, 126), ==, 0);
    g_assert_cmphex(csd_bits(csd, 119, 112), ==, 0x74);
    g_assert_cmphex(csd_bits(csd, 111, 104), ==, 0);
    g_assert_cmphex(csd_bits(csd, 28, 26), ==, 0);
    read_reg(qts, rca, 13, ssr, 64);
    g_assert_cmphex(ssr[8], ==, 2);
    g_assert_cmphex(ssr[9], ==, 0xff);
    g_assert_cmphex(ssr[10] >> 4, ==, 6);
    g_assert_cmphex(ssr[11] << 8 | ssr[12], ==, 128);
    g_assert_cmphex(ssr[13] >> 2, ==, 1);
    g_assert_cmphex(ssr[13] & 3, ==, 1);
    g_assert_cmphex(ssr[14], ==, 0);
    read_reg(qts, rca, 51, scr, 8);
    g_assert_cmphex(scr[1] & 0x80, ==, 0x80);
    qtest_quit(qts);
    unlink(img);
}

/*
 * A 4 GiB SDHC card: CSD 2.0, its fixed TAAC 1 ms; Class 10, AU 4 MiB,
 * Pm 0, no UHS speed grade. Properties set the registers: Class 6 with a
 * 2 MiB AU, which is its UHS AU too with U1.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_registers_sdhc(void)
{
    QTestState *qts;
    uint32_t csd[4], rca;
    uint8_t ssr[64];

    qts = qtest_init("-M esp32p4,strap-mode=0 -drive if=none,id=sd0,"
                     "driver=null-co,read-zeroes=on,size=4G "
                     "-device sd-card,drive=sd0,bus=sd-bus0");
    clocks_on(qts);
    rca = card_init(qts, csd);
    g_assert_cmphex(csd_bits(csd, 127, 126), ==, 1);
    g_assert_cmphex(csd_bits(csd, 119, 112), ==, 0x0e);
    g_assert_cmphex(csd_bits(csd, 28, 26), ==, 2);
    read_reg(qts, rca, 13, ssr, 64);
    g_assert_cmphex(ssr[8], ==, 4);
    g_assert_cmphex(ssr[9], ==, 0);
    g_assert_cmphex(ssr[10] >> 4, ==, 9);
    g_assert_cmphex(ssr[14], ==, 0);
    qtest_quit(qts);

    qts = qtest_init("-M esp32p4,strap-mode=0 -drive if=none,id=sd0,"
                     "driver=null-co,read-zeroes=on,size=4G "
                     "-device sd-card,drive=sd0,bus=sd-bus0,speed-class=6,"
                     "au-size=2M,uhs-speed-grade=1");
    clocks_on(qts);
    rca = card_init(qts, NULL);
    read_reg(qts, rca, 13, ssr, 64);
    g_assert_cmphex(ssr[8], ==, 3);
    g_assert_cmphex(ssr[9], ==, 0xff);
    g_assert_cmphex(ssr[10] >> 4, ==, 8);
    g_assert_cmphex(ssr[14], ==, 0x18);
    qtest_quit(qts);
}

/*
 * A block read: its start bit comes TAAC after CMD17's end bit, the next
 * of a CMD18 TAAC after the last's end bit. A data timeout shorter than
 * that ends the read with a data read timeout.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_read_access(void)
{
    g_autofree char *img = make_image(0x22);
    QTestState *qts = start(img, "");
    int64_t dto;
    uint32_t st;
    uint8_t buf[512];

    clocks_on(qts);
    card_init(qts, NULL);

    qtest_writel(qts, SD_BYTCNT, 512);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA, 17, 3 * 512);
    dto = TAKE_NS + CMD_NS + TAAC_NS + RBLOCK_NS;
    qtest_clock_step(qts, dto - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    for (int i = 0; i < 128; i++) {
        uint32_t w = qtest_readl(qts, SD_BUFFIFO);

        memcpy(&buf[4 * i], &w, 4);
    }
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, image_byte(0x22, 3 * 512 + i));
    }
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    /* Two blocks: TAAC again between them, then the auto stop. */
    qtest_writel(qts, SD_BYTCNT, 1024);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA | CMD_AUTO_STOP, 18, 0);
    dto = TAKE_NS + CMD_NS + 2 * (TAAC_NS + RBLOCK_NS);
    qtest_clock_step(qts, dto - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    wait_int(qts, INT_ACD, 1000 * 1000);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    /* TMOUT's data timeout, 600 clocks of 100 ns: 60 us, short of 70. */
    qtest_writel(qts, SD_TMOUT, (600u << 8) | 0x40);
    qtest_writel(qts, SD_BYTCNT, 512);
    st = cmd(qts, R1 | CMD_WAIT | CMD_DATA, 17, 512);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    st = wait_int(qts, INT_DRTO | INT_DTO, 1000 * 1000);
    g_assert_cmphex(st & INT_DRTO, ==, INT_DRTO);
    qtest_quit(qts);
    unlink(img);
}

/*
 * A write holds DAT0 low for the block's programming, TAAC times
 * R2W_FACTOR after its CRC status: STATUS.DATA_BUSY, the card in the
 * programming state and not READY_FOR_DATA; it refuses a read (no
 * response, ILLEGAL_COMMAND after) and a write, and the write's data is
 * not stored; a command waiting for the previous data waits for the busy
 * to end; the busy clear interrupt comes as it ends.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_write_busy(void)
{
    g_autofree char *img = make_image(0x33);
    QTestState *qts = start(img, "");
    uint32_t rca, st, r1;
    uint8_t buf[512];

    clocks_on(qts);
    rca = card_init(qts, NULL);
    qtest_writel(qts, SD_CARDTHRCTL, CARDTHRCTL_BUSY_CLR);

    write_start(qts, 5, CMD_WAIT);
    /* The response, then the block: DTO as the CRC status is in. */
    qtest_clock_step(qts, CMD_RESP_NS + WBLOCK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    g_assert_false(busy(qts));
    qtest_clock_step(qts, 1);
    st = qtest_readl(qts, SD_RINTSTS);
    g_assert_cmphex(st & (INT_DTO | INT_BCI), ==, INT_DTO);
    g_assert_true(busy(qts));
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    /* CMD13 at once: programming, not ready for data. */
    r1 = status_now(qts, rca);
    g_assert_cmpint(R1_STATE(r1), ==, STATE_PRG);
    g_assert_cmphex(r1 & R1_READY, ==, 0);
    /* A read and a write are refused: no response. */
    qtest_writel(qts, SD_BYTCNT, 512);
    st = cmd(qts, R1 | CMD_DATA, 17, 0);
    g_assert_cmphex(st & INT_RTO, ==, INT_RTO);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    r1 = status_now(qts, rca);
    g_assert_cmphex(r1 & R1_ILLEGAL, ==, R1_ILLEGAL);
    g_assert_cmpint(R1_STATE(r1), ==, STATE_PRG);
    write_start(qts, 6, 0);
    st = wait_int(qts, INT_CMDD, 1000 * 1000);
    g_assert_cmphex(st & INT_RTO, ==, INT_RTO);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    g_assert_true(busy(qts));

    /*
     * CMD13 waiting for the previous data: taken as the busy ends, with
     * the busy clear interrupt.
     */
    cmd_start(qts, R1 | CMD_WAIT, 13, rca << 16);
    while (busy(qts)) {
        g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, CMD_START);
        g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_BCI, ==, 0);
        qtest_clock_step(qts, 100);
    }
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_BCI, ==, INT_BCI);
    st = wait_int(qts, INT_CMDD, 1000 * 1000);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    r1 = qtest_readl(qts, SD_RESP(0));
    g_assert_cmpint(R1_STATE(r1), ==, STATE_TRAN);
    g_assert_cmphex(r1 & R1_READY, ==, R1_READY);

    /* Block 5 holds the new data, block 6, refused, its old. */
    read_block(qts, 5, buf);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, new_byte(i));
    }
    read_block(qts, 6, buf);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(buf[i], ==, image_byte(0x33, 6 * 512 + i));
    }
    qtest_quit(qts);
    unlink(img);
}

/*
 * The busy's exact length: STATUS.DATA_BUSY from the CRC status of the
 * block to TAAC later; and for a multiple block write, after each block,
 * the controller sending the next as the card takes it.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_busy_length(void)
{
    g_autofree char *img = make_image(0x44);
    QTestState *qts = start(img, "");
    int64_t end;
    uint8_t buf[512];

    clocks_on(qts);
    card_init(qts, NULL);

    write_start(qts, 7, CMD_WAIT);
    end = CMD_RESP_NS + WBLOCK_NS + TAAC_NS;
    qtest_clock_step(qts, end - 1);
    g_assert_true(busy(qts));
    qtest_clock_step(qts, 1);
    g_assert_false(busy(qts));
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    /* Three blocks, the auto stop after: each block's busy in turn. */
    qtest_writel(qts, SD_BYTCNT, 3 * 512);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA | CMD_WRITE | CMD_AUTO_STOP,
              25, 10 * 512);
    qtest_clock_step(qts, TAKE_NS);
    for (int b = 0; b < 3; b++) {
        for (int i = 0; i < 128; i++) {
            qtest_writel(qts, SD_BUFFIFO, 0x01010101u * (b + 1));
        }
    }
    end = CMD_RESP_NS + 3 * (WBLOCK_NS + TAAC_NS);
    qtest_clock_step(qts, end - 1);
    g_assert_true(busy(qts));
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    qtest_clock_step(qts, 1);
    g_assert_false(busy(qts));
    wait_int(qts, INT_ACD, 1000 * 1000);
    for (int b = 0; b < 3; b++) {
        read_block(qts, 10 + b, buf);
        g_assert_cmphex(buf[0], ==, b + 1);
        g_assert_cmphex(buf[511], ==, b + 1);
    }
    qtest_quit(qts);
    unlink(img);
}

/*
 * CMD7 to another card moves a card that is programming to the
 * disconnect state, DAT0 released, its programming going on; done, it is
 * in standby. Reselected while programming, it is busy again.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_deselect(void)
{
    g_autofree char *img = make_image(0x55);
    QTestState *qts = start(img, "");
    uint32_t rca, r1;
    uint8_t buf[512];

    clocks_on(qts);
    rca = card_init(qts, NULL);
    write_start(qts, 8, CMD_WAIT);
    wait_int(qts, INT_DTO, 1000 * 1000);
    g_assert_true(busy(qts));
    cmd(qts, R1, 7, 0);
    g_assert_false(busy(qts));
    qtest_clock_step(qts, TAAC_NS);
    r1 = status_now(qts, rca);
    g_assert_cmpint(R1_STATE(r1), ==, STATE_STBY);
    cmd(qts, R1 | CMD_WAIT, 7, rca << 16);
    read_block(qts, 8, buf);
    g_assert_cmphex(buf[100], ==, new_byte(100));

    write_start(qts, 9, CMD_WAIT);
    wait_int(qts, INT_DTO, 1000 * 1000);
    cmd(qts, R1, 7, 0);
    g_assert_false(busy(qts));
    cmd(qts, R1, 7, rca << 16);
    g_assert_true(busy(qts));
    r1 = status_now(qts, rca);
    g_assert_cmpint(R1_STATE(r1), ==, STATE_PRG);
    qtest_quit(qts);
    unlink(img);
}

/*
 * An erase of two blocks, in the first AU: busy for ERASE_TIMEOUT /
 * ERASE_SIZE, 1/128 s, after CMD38's response; the blocks then read 1s.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_erase(void)
{
    g_autofree char *img = make_image(0x66);
    QTestState *qts = start(img, "");
    int64_t end = 1000000000LL / 128;
    uint8_t buf[512];

    clocks_on(qts);
    card_init(qts, NULL);
    cmd(qts, R1 | CMD_WAIT, 32, 2 * 512);
    cmd(qts, R1 | CMD_WAIT, 33, 3 * 512);
    cmd_start(qts, R1 | CMD_WAIT, 38, 0);
    qtest_clock_step(qts, TAKE_NS + CMD_RESP_NS + end - 1);
    g_assert_true(busy(qts));
    qtest_clock_step(qts, 1);
    g_assert_false(busy(qts));
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    read_block(qts, 2, buf);
    g_assert_cmphex(buf[0], ==, 0xff);
    read_block(qts, 3, buf);
    g_assert_cmphex(buf[511], ==, 0xff);
    read_block(qts, 4, buf);
    g_assert_cmphex(buf[0], ==, image_byte(0x66, 4 * 512));
    qtest_quit(qts);
    unlink(img);
}

/*
 * max-timing: the specification's maximums. For the SDSC card, 100 times
 * its typical times, under the 100 ms and 250 ms limits: 7 ms each; its
 * erase's timeout, 1 s at least, plus 250 ms for the range's end inside
 * an AU. For an SDHC card, 100 ms and 250 ms; ESP-IDF's data timeout,
 * 100 ms of card clocks, still lets a multiple block read through, each
 * block's start bit coming as the count runs out.
 */
/* [spec:nuos:req:emu.sd-card-timing/test] */
static void test_max_timing(void)
{
    g_autofree char *img = make_image(0x77);
    QTestState *qts = start(img, "-global sd-card.max-timing=on");
    uint32_t st;
    int64_t t;

    clocks_on(qts);
    card_init(qts, NULL);
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA, 17, 0);
    t = TAKE_NS + CMD_NS + 100 * TAAC_NS + RBLOCK_NS;
    qtest_clock_step(qts, t - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    write_start(qts, 1, CMD_WAIT);
    t = CMD_RESP_NS + WBLOCK_NS + 100 * TAAC_NS;
    qtest_clock_step(qts, t - 1);
    g_assert_true(busy(qts));
    qtest_clock_step(qts, 1);
    g_assert_false(busy(qts));
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    cmd(qts, R1 | CMD_WAIT, 32, 0);
    cmd(qts, R1 | CMD_WAIT, 33, 512);
    cmd_start(qts, R1 | CMD_WAIT, 38, 0);
    t = TAKE_NS + CMD_RESP_NS + 1000000000LL + 1000000000LL / 128 +
        250000000LL;
    qtest_clock_step(qts, t - 1);
    g_assert_true(busy(qts));
    qtest_clock_step(qts, 1);
    g_assert_false(busy(qts));
    qtest_quit(qts);
    unlink(img);

    qts = qtest_init("-M esp32p4,strap-mode=0 -drive if=none,id=sd0,"
                     "driver=null-co,read-zeroes=on,size=4G "
                     "-device sd-card,drive=sd0,bus=sd-bus0,max-timing=on");
    clocks_on(qts);
    card_init(qts, NULL);
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA, 17, 0);
    t = TAKE_NS + CMD_NS + 100000000LL + RBLOCK_NS;
    qtest_clock_step(qts, t - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    qtest_writel(qts, SD_TMOUT, (100000000 / CLK_NS) << 8 | 0xff);
    qtest_writel(qts, SD_BYTCNT, 1024);
    cmd_start(qts, R1 | CMD_WAIT | CMD_DATA | CMD_AUTO_STOP, 18, 0);
    st = wait_int(qts, INT_ACD | INT_DRTO, 1000LL * 1000 * 1000);
    g_assert_cmphex(st & (INT_DTO | INT_DRTO | INT_ACD), ==,
                    INT_DTO | INT_ACD);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    write_start(qts, 1, CMD_WAIT);
    t = CMD_RESP_NS + WBLOCK_NS + 250000000LL;
    qtest_clock_step(qts, t - 1);
    g_assert_true(busy(qts));
    qtest_clock_step(qts, 1);
    g_assert_false(busy(qts));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/sd-card/registers-sdsc", test_registers_sdsc);
    qtest_add_func("/esp32p4/sd-card/registers-sdhc", test_registers_sdhc);
    qtest_add_func("/esp32p4/sd-card/read-access", test_read_access);
    qtest_add_func("/esp32p4/sd-card/write-busy", test_write_busy);
    qtest_add_func("/esp32p4/sd-card/busy-length", test_busy_length);
    qtest_add_func("/esp32p4/sd-card/deselect", test_deselect);
    qtest_add_func("/esp32p4/sd-card/erase", test_erase);
    qtest_add_func("/esp32p4/sd-card/max-timing", test_max_timing);
    return g_test_run();
}
