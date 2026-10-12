/*
 * QTest testcase for UHS-I on the ESP32-P4 board's microSD slot
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The SD/MMC host's slot 0, whose pads the PMU's regulator VO4 supplies,
 * with a QEMU SD card: S18R and S18A in ACMD41, the CMD11 voltage switch
 * and its timing, CMD6's UHS-I functions, CMD19, VO4's voltage as the
 * pads' and what a card and pads at different voltages make of each
 * other, and the sample phase at SDR50 and DDR50.
 *
 * The controller's CIU runs at 40 MHz (25 ns) from PLL_F160M_CLK for
 * identification, the card at 400 kHz (2.5 us); SDR50 and DDR50 take the
 * SDIO PLL's 200 MHz divided by 2 and 4. The cards are sparse raw images
 * of 4 GiB (high capacity, which UHS-I needs) or 1 MiB (standard
 * capacity), their first blocks patterned.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

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
#define SD_UHS              (SD + 0x074)
#define SD_BUFFIFO          (SD + 0x200)
#define SD_DLL_CLK_CONF     (SD + 0x808)
#define DLL_SAM_PHASE(k)    ((k) << 15)

#define CTRL_RESETS         7u

#define CMD_RESP            (1u << 6)
#define CMD_LONG            (1u << 7)
#define CMD_CRC             (1u << 8)
#define CMD_DATA            (1u << 9)
#define CMD_WAIT            (1u << 13)
#define CMD_INIT            (1u << 15)
#define CMD_CARD(n)         ((n) << 16)
#define CMD_UPDATE_CLK      (1u << 21)
#define CMD_VOLT_SWITCH     (1u << 28)
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
#define INT_HTO             (1u << 10)
#define INT_EBE             (1u << 15)
#define INT_ERRORS          (INT_RE | INT_RCRC | INT_DCRC | INT_RTO | \
                             INT_DRTO | INT_EBE)

#define STATUS_DATA3        (1u << 8)
#define STATUS_BUSY         (1u << 9)

#define OCR_BUSY            (1u << 31)
#define OCR_CCS             (1u << 30)
#define OCR_S18             (1u << 24)
#define ACMD41_ARG          0x40ff8000u

#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SDMMC_SYS_CLK_EN    (1u << 14)
#define PERI_CLK_CTRL01     (HP_CLKRST + 0x34)
#define SDIO_HS_MODE        (1u << 22)
#define SDIO_LS_SRC_SDIO    (1u << 23)
#define SDIO_LS_EN          (1u << 24)
#define PERI_CLK_CTRL02     (HP_CLKRST + 0x38)
#define EDGE_UPDATE         (1u << 8)
#define EDGE_L(v)           ((v) << 9)
#define EDGE_H(v)           ((v) << 13)
#define EDGE_N(v)           ((v) << 17)
#define DRV_SEL(v)          ((v) << 23)
#define SAM_SEL(v)          ((v) << 25)
#define SLF_DRV_SAM_EN      (7u << 27)
#define LP_HP_CLK_CTRL      0x50111040
#define SDIO_PLL_CLK_EN     (7u << 21)

/* The PMU: the SDIO PLL's power and the regulator VO4 */
#define PMU                 0x50115000
#define PMU_IMM_HP_CK_POWER (PMU + 0xcc)
#define TIE_HIGH_SDIOPLL    ((1u << 20) | (1u << 26) | (1u << 30))
#define PMU_RF_PWC          (PMU + 0x15c)
#define SDIO_PLL_XPD        (1u << 25)
#define PMU_VO4             (PMU + 0x1d8)
#define PMU_VO4_ANA         (PMU + 0x1dc)
#define LDO_RESET           0x40200000u
#define LDO_FORCE_TIEH_SEL  (1u << 7)
#define LDO_XPD             (1u << 8)
#define LDO_TIEH_SEL(v)     ((v) << 9)
#define LDO_TIEH            (1u << 14)
#define LDO_ANA(dref, mul)  (((uint32_t)(dref) << 28) | ((mul) << 23))

#define PMU_PATH            "/machine/pmu"
#define SDMMC_PATH          "/machine/sdmmc"

#define CCLK_NS             25
#define ID_CLK_NS           2500
#define CMD_NS              (2 * CCLK_NS + (48 + 2 + 48) * ID_CLK_NS)
#define MS                  (1000 * 1000)

#define SDHC_SIZE           (4ull << 30)
#define SDSC_SIZE           (1 << 20)

/* The SD Physical Layer Simplified Specification 9.10's Table 4-3 */
static const uint8_t tuning_block[64] = {
    0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
    0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
    0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
    0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
    0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
    0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
    0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
    0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

static uint8_t pattern_byte(uint32_t offset)
{
    return (uint8_t)(offset * 13 + offset / 512 + 1);
}

/* A sparse image of @size bytes, its first 16 blocks patterned. */
static char *make_image(uint64_t size)
{
    uint8_t data[16 * 512];
    char *path;
    int fd;

    fd = g_file_open_tmp("esp32p4-sd-uhs-XXXXXX.img", &path, NULL);
    g_assert(fd >= 0);
    for (int i = 0; i < sizeof(data); i++) {
        data[i] = pattern_byte(i);
    }
    g_assert_cmpint(write(fd, data, sizeof(data)), ==, sizeof(data));
    g_assert_cmpint(ftruncate(fd, size), ==, 0);
    close(fd);
    return path;
}

static QTestState *start(const char *img0, const char *img1)
{
    g_autofree char *d0 = img0 ?
        g_strdup_printf("-drive if=sd,index=0,format=raw,file=%s", img0) :
        g_strdup("");
    g_autofree char *d1 = img1 ?
        g_strdup_printf("-drive if=sd,index=1,format=raw,file=%s", img1) :
        g_strdup("");

    /* No flash boot watchdogs: the tests run for longer than they allow. */
    return qtest_initf("-M esp32p4,strap-mode=0 %s %s", d0, d1);
}

static uint64_t qom_uint(QTestState *qts, const char *path, const char *prop)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': %s } }", path, prop);
    uint64_t v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_uint(r, "return");
    qobject_unref(r);
    return v;
}

/* VO4: off, bypassed to VDD_LDO (3.3 V), or LDO mode at DREF/MUL. */
static void vo4_off(QTestState *qts)
{
    qtest_writel(qts, PMU_VO4, LDO_RESET);
}

static void vo4_3v3(QTestState *qts)
{
    qtest_writel(qts, PMU_VO4,
                 LDO_RESET | LDO_FORCE_TIEH_SEL | LDO_XPD | LDO_TIEH);
}

static void vo4_ldo(QTestState *qts, unsigned dref, unsigned mul)
{
    qtest_writel(qts, PMU_VO4_ANA, LDO_ANA(dref, mul));
    qtest_writel(qts, PMU_VO4, LDO_RESET | LDO_FORCE_TIEH_SEL | LDO_XPD);
}

/* SDMMC_SYS_CLK and the SDIO_LS clocks at 40 MHz, slot 0's pads at 3.3 V. */
static void clocks_on(QTestState *qts)
{
    uint32_t edges = EDGE_N(3) | EDGE_L(3) | EDGE_H(1) | SLF_DRV_SAM_EN;

    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | SDMMC_SYS_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | SDIO_LS_EN);
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_writel(qts, PERI_CLK_CTRL02, edges | EDGE_UPDATE);
    vo4_3v3(qts);
}

/* A clock update for card @card, the clock enabled or not. */
static void clock_update(QTestState *qts, unsigned card, bool on,
                         uint32_t flags)
{
    qtest_writel(qts, SD_CLKENA, on ? 1u << card : 0);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_WAIT |
                 CMD_HOLD | CMD_CARD(card) | flags);
    qtest_clock_step(qts, 2 * CCLK_NS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
}

static void card_clock(QTestState *qts, unsigned card, unsigned div)
{
    qtest_writel(qts, SD_CLKDIV, div << (8 * card));
    qtest_writel(qts, SD_CLKSRC, card << (2 * card));
    clock_update(qts, card, true, 0);
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

/*
 * A read of @len bytes after command @index, through the FIFO: RINTSTS
 * gathered to the data's end, the bytes in @buf.
 */
static uint32_t read_data(QTestState *qts, unsigned card, uint32_t flags,
                          unsigned index, uint32_t arg, uint32_t len,
                          uint8_t *buf)
{
    uint32_t all = 0;

    qtest_writel(qts, SD_BLKSIZ, len);
    qtest_writel(qts, SD_BYTCNT, len);
    qtest_writel(qts, SD_CMDARG, arg);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | CMD_CARD(card) | flags | index);
    while (!(all & (INT_DTO | INT_DRTO))) {
        all |= wait_int(qts, INT_DTO | INT_DRTO | INT_CMDD, 200 * MS);
    }
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = qtest_readl(qts, SD_BUFFIFO);

        memcpy(&buf[i], &w, MIN(4, len - i));
    }
    /* The receive data requests the FIFO's reads left */
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    return all;
}

/* CMD0, CMD8 and ACMD41 until the card is ready: its OCR. */
static uint32_t identify(QTestState *qts, unsigned card, bool s18r)
{
    uint32_t ocr = 0;

    card_clock(qts, card, 50);
    cmd(qts, card, CMD_INIT, 0, 0);
    cmd_ok(qts, card, R1 | CMD_WAIT, 8, 0x1aa);
    for (int tries = 0; tries < 20 && !(ocr & OCR_BUSY); tries++) {
        cmd_ok(qts, card, R1 | CMD_WAIT, 55, 0);
        cmd(qts, card, R3 | CMD_WAIT, 41, ACMD41_ARG | (s18r ? OCR_S18 : 0));
        ocr = qtest_readl(qts, SD_RESP(0));
    }
    g_assert_cmphex(ocr & OCR_BUSY, ==, OCR_BUSY);
    return ocr;
}

/*
 * The voltage switch as ESP-IDF makes it: CMD11, the clock stopped with
 * VOLT_SWITCH, VO4 to 1.8 V, the clock started @gap_ns later.
 */
static void volt_switch(QTestState *qts, int64_t gap_ns)
{
    uint32_t st = cmd(qts, 0, R1 | CMD_VOLT_SWITCH, 11, 0);

    g_assert_cmphex(st, ==, INT_CMDD | INT_HTO);
    clock_update(qts, 0, false, CMD_VOLT_SWITCH);
    vo4_ldo(qts, 8, 4);
    qtest_clock_step(qts, gap_ns - 2 * CCLK_NS);
    clock_update(qts, 0, true, CMD_VOLT_SWITCH);
}

/* CMD2, CMD3, CMD7: the RCA of the card, selected. */
static uint32_t select_card(QTestState *qts)
{
    uint32_t rca;

    cmd_ok(qts, 0, R2 | CMD_WAIT, 2, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(3)) >> 24, ==, 0xaa);
    cmd_ok(qts, 0, R1 | CMD_WAIT, 3, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    cmd_ok(qts, 0, R1 | CMD_WAIT, 7, rca << 16);
    return rca;
}

/* A card at 1.8 V in the transfer state, its bus 4 bits wide. */
static uint32_t uhs_card(QTestState *qts)
{
    uint32_t rca;

    clocks_on(qts);
    g_assert_cmphex(identify(qts, 0, true) & OCR_S18, ==, OCR_S18);
    volt_switch(qts, 10 * MS);
    wait_int(qts, INT_HTO, 2 * MS);
    rca = select_card(qts);
    cmd_ok(qts, 0, R1 | CMD_WAIT, 55, rca << 16);
    cmd_ok(qts, 0, R1 | CMD_WAIT, 6, 2);
    qtest_writel(qts, SD_CTYPE, 1);
    return rca;
}

/* CMD6's status for @arg: the 64 bytes in @st, RINTSTS returned. */
static uint32_t cmd6(QTestState *qts, uint32_t arg, uint8_t *st)
{
    return read_data(qts, 0, 0, 6, arg, 64, st);
}

static uint16_t be16(const uint8_t *p)
{
    return (p[0] << 8) | p[1];
}

/* Group @g's function as CMD6's status gives it */
static unsigned cmd6_sel(const uint8_t *st, unsigned g)
{
    return (st[16 - (g - 1) / 2] >> (((g - 1) % 2) * 4)) & 0xf;
}

static uint16_t cmd6_support(const uint8_t *st, unsigned g)
{
    return be16(&st[14 - 2 * g]);
}

/*
 * S18A: only on S18R, only by a UHS-I card of high capacity in the board's
 * socket.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_s18a(void)
{
    g_autofree char *sdhc = make_image(SDHC_SIZE);
    g_autofree char *sdsc = make_image(SDSC_SIZE);
    QTestState *qts = start(sdhc, NULL);
    uint32_t ocr, st;

    clocks_on(qts);
    /* No S18R: no S18A, and CMD11 is illegal (no response). */
    ocr = identify(qts, 0, false);
    g_assert_cmphex(ocr & (OCR_CCS | OCR_S18), ==, OCR_CCS);
    st = cmd(qts, 0, R1 | CMD_VOLT_SWITCH, 11, 0);
    g_assert_cmphex(st, ==, INT_CMDD | INT_RTO);
    /* S18R: S18A. */
    ocr = identify(qts, 0, true);
    g_assert_cmphex(ocr & (OCR_CCS | OCR_S18), ==, OCR_CCS | OCR_S18);
    /* CMD11 outside the ready state is illegal. */
    cmd(qts, 0, CMD_INIT, 0, 0);
    st = cmd(qts, 0, R1 | CMD_VOLT_SWITCH, 11, 0);
    g_assert_cmphex(st, ==, INT_CMDD | INT_RTO);
    qtest_quit(qts);

    /* A standard capacity card, and a card in slot 1: never S18A. */
    qts = start(sdsc, sdhc);
    clocks_on(qts);
    ocr = identify(qts, 0, true);
    g_assert_cmphex(ocr & (OCR_CCS | OCR_S18), ==, 0);
    st = cmd(qts, 0, R1 | CMD_VOLT_SWITCH, 11, 0);
    g_assert_cmphex(st, ==, INT_CMDD | INT_RTO);
    ocr = identify(qts, 1, true);
    g_assert_cmphex(ocr & (OCR_CCS | OCR_S18), ==, OCR_CCS);
    qtest_quit(qts);
    unlink(sdhc);
    unlink(sdsc);
}

/*
 * The voltage switch: CMD11's R1 with the voltage switch interrupt, the
 * card holding DAT[3:0] low and answering nothing; the clock stopped for
 * 5 ms; DAT[3:0] released 1 ms after the clock is back, the second
 * interrupt with command done; the card then at 1.8 V, through CMD0.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_switch(void)
{
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);
    uint32_t st, ocr;

    clocks_on(qts);
    identify(qts, 0, true);
    qtest_writel(qts, SD_CMDARG, 0);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | CMD_VOLT_SWITCH |
                 11);
    qtest_clock_step(qts, CMD_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_CMDD | INT_HTO);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    /* R1: the card in the ready state (1). */
    g_assert_cmphex((qtest_readl(qts, SD_RESP(0)) >> 9) & 0xf, ==, 1);
    /* DAT[3:0] low: busy, and no DAT3. */
    st = qtest_readl(qts, SD_STATUS);
    g_assert_cmphex(st & (STATUS_BUSY | STATUS_DATA3), ==, STATUS_BUSY);
    /* A command now finds no answer. */
    g_assert_cmphex(cmd(qts, 0, R1, 13, 0), ==, INT_CMDD | INT_RTO);

    /* The clock stopped, VO4 to 1.8 V, the clock back 5 ms later. */
    clock_update(qts, 0, false, CMD_VOLT_SWITCH);
    vo4_ldo(qts, 8, 4);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 1800);
    qtest_clock_step(qts, 5 * MS - 2 * CCLK_NS);
    clock_update(qts, 0, true, CMD_VOLT_SWITCH);
    /* DAT[3:0] released 1 ms after the clock is back. */
    qtest_clock_step(qts, 1 * MS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==,
                    STATUS_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_CMDD | INT_HTO);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    st = qtest_readl(qts, SD_STATUS);
    g_assert_cmphex(st & (STATUS_BUSY | STATUS_DATA3), ==, STATUS_DATA3);
    /* At 1.8 V the card answers. */
    select_card(qts);

    /* CMD0 keeps it at 1.8 V: no S18A, CMD11 illegal, CMD2 answered. */
    cmd(qts, 0, CMD_INIT, 0, 0);
    ocr = identify(qts, 0, true);
    g_assert_cmphex(ocr & (OCR_CCS | OCR_S18), ==, OCR_CCS);
    st = cmd(qts, 0, R1 | CMD_VOLT_SWITCH, 11, 0);
    g_assert_cmphex(st, ==, INT_CMDD | INT_RTO);
    select_card(qts);
    qtest_quit(qts);
    unlink(img);
}

/*
 * A clock back sooner than 5 ms: the switch fails, the card holding
 * DAT[3:0] low and answering nothing until a power cycle, which brings it
 * back at 3.3 V.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_switch_fails(void)
{
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);

    clocks_on(qts);
    identify(qts, 0, true);
    volt_switch(qts, 5 * MS - 1);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==,
                    STATUS_BUSY);
    g_assert_cmphex(cmd(qts, 0, R2, 2, 0), ==, INT_CMDD | INT_RTO);
    g_assert_cmphex(cmd(qts, 0, CMD_INIT, 0, 0) & INT_ERRORS, ==, 0);
    g_assert_cmphex(cmd(qts, 0, R1, 8, 0x1aa), ==, INT_CMDD | INT_RTO);

    /* A power cycle: VO4 off again, the card at 3.3 V offering S18A. */
    qtest_system_reset(qts);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 0);
    clocks_on(qts);
    g_assert_cmphex(identify(qts, 0, true) & OCR_S18, ==, OCR_S18);
    qtest_quit(qts);
    unlink(img);
}

/*
 * CMD6's function groups at 1.8 V: the UHS-I bus speed modes, driver
 * strengths and power limits offered; a switch taking effect after its
 * status; a selection not offered voiding the switch. At 3.3 V, Default
 * and High Speed alone.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_cmd6(void)
{
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);
    uint8_t st[64];

    uhs_card(qts);
    g_assert_cmphex(cmd6(qts, 0x00fffff0, st) & INT_ERRORS, ==, 0);
    g_assert_cmphex(cmd6_support(st, 1), ==, 0x801f);
    g_assert_cmphex(cmd6_support(st, 2), ==, 0x8043);
    g_assert_cmphex(cmd6_support(st, 3), ==, 0x800f);
    g_assert_cmphex(cmd6_support(st, 4), ==, 0x800f);
    g_assert_cmphex(cmd6_support(st, 5), ==, 0x8001);
    g_assert_cmphex(cmd6_support(st, 6), ==, 0x8001);
    for (unsigned g = 1; g <= 6; g++) {
        g_assert_cmpuint(cmd6_sel(st, g), ==, 0);
    }
    /* Version 1, its busy status all ready; SDR12's 100 mA. */
    g_assert_cmphex(st[17], ==, 1);
    g_assert_cmphex(be16(&st[18]) | be16(&st[28]), ==, 0);
    g_assert_cmpuint(be16(&st[0]), ==, 100);

    /* Mode 0 checks SDR50: 400 mA, nothing switched. */
    cmd6(qts, 0x00fffff2, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 2);
    g_assert_cmpuint(be16(&st[0]), ==, 400);
    cmd6(qts, 0x00ffffff, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 0);

    /* Mode 1: SDR50, within the 0.72 W power limit's 200 mA. */
    g_assert_cmphex(cmd6(qts, 0x80fffff2, st) & INT_ERRORS, ==, 0);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 2);
    g_assert_cmpuint(be16(&st[0]), ==, 200);
    cmd6(qts, 0x00ffffff, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 2);

    /* Function 5 of group 1 is not offered: nothing switches. */
    cmd6(qts, 0x80fff1f5, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 0xf);
    g_assert_cmpuint(cmd6_sel(st, 3), ==, 0);
    g_assert_cmpuint(be16(&st[0]), ==, 0);
    cmd6(qts, 0x00ffffff, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 2);
    g_assert_cmpuint(cmd6_sel(st, 3), ==, 0);

    /* Driver strength type A and the 1.44 W power limit (400 mA). */
    cmd6(qts, 0x80ff11ff, st);
    g_assert_cmpuint(cmd6_sel(st, 3), ==, 1);
    g_assert_cmpuint(cmd6_sel(st, 4), ==, 1);
    g_assert_cmpuint(be16(&st[0]), ==, 400);
    qtest_quit(qts);

    /* At 3.3 V: Default and High Speed only. */
    qts = start(img, NULL);
    clocks_on(qts);
    identify(qts, 0, true);
    select_card(qts);
    cmd6(qts, 0x00fffff0, st);
    g_assert_cmphex(cmd6_support(st, 1), ==, 0x8003);
    g_assert_cmphex(cmd6_support(st, 3), ==, 0x8001);
    g_assert_cmphex(cmd6_support(st, 4), ==, 0x8001);
    cmd6(qts, 0x80fffff2, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 0xf);
    cmd6(qts, 0x80fffff1, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 1);
    g_assert_cmpuint(be16(&st[0]), ==, 200);
    qtest_quit(qts);
    unlink(img);
}

/* CMD19: the tuning block at 1.8 V, illegal at 3.3 V. */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_cmd19(void)
{
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);
    uint8_t blk[64];
    uint32_t st;

    clocks_on(qts);
    identify(qts, 0, true);
    select_card(qts);
    /* 3.3 V: no response, and no data within the data timeout. */
    qtest_writel(qts, SD_TMOUT, (1000u << 8) | 0x40);
    st = read_data(qts, 0, 0, 19, 0, 64, blk);
    g_assert_cmphex(st & (INT_RTO | INT_DRTO), ==, INT_RTO | INT_DRTO);
    qtest_writel(qts, SD_CTRL, CTRL_RESETS);
    qtest_clock_step(qts, 1000);
    qtest_quit(qts);

    qts = start(img, NULL);
    uhs_card(qts);
    st = read_data(qts, 0, 0, 19, 0, 64, blk);
    g_assert_cmphex(st & INT_ERRORS, ==, 0);
    g_assert_cmpmem(blk, 64, tuning_block, 64);
    qtest_quit(qts);
    unlink(img);
}

/*
 * VO4's voltage, which slot 0's pads signal at: off, bypassed to VDD_LDO,
 * in LDO mode as DREF and MUL give it, the mode by TIEH_SEL's mux, the
 * SD/MMC host's VOLT_REG among its inputs. Slot 1's pads are at 3.3 V.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_ldo(void)
{
    QTestState *qts = start(NULL, NULL);

    g_assert_cmpuint(qom_uint(qts, PMU_PATH, "vo4-mv"), ==, 0);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 0);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad1-mv"), ==, 3300);
    clocks_on(qts);
    g_assert_cmpuint(qom_uint(qts, PMU_PATH, "vo4-mv"), ==, 3300);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 3300);
    /* LDO mode: 0.9 V x 2.0, and ESP-IDF's 0.8 V x 2.25. */
    vo4_ldo(qts, 8, 4);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 1800);
    vo4_ldo(qts, 6, 5);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 1800);
    /* 1.2 V x 2.0 */
    vo4_ldo(qts, 11, 4);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 2400);
    /* Beyond VDD_LDO: VDD_LDO. */
    vo4_ldo(qts, 15, 7);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 3300);
    vo4_ldo(qts, 8, 4);
    /* TIEH_SEL 2: bypass. */
    qtest_writel(qts, PMU_VO4, LDO_RESET | LDO_FORCE_TIEH_SEL | LDO_XPD |
                 LDO_TIEH_SEL(2));
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 3300);
    /* TIEH_SEL 1: the host's card 0 VOLT_REG, LDO mode while it is set. */
    qtest_writel(qts, PMU_VO4, LDO_RESET | LDO_FORCE_TIEH_SEL | LDO_XPD |
                 LDO_TIEH_SEL(1));
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 3300);
    qtest_writel(qts, SD_UHS, 1);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 1800);
    qtest_writel(qts, SD_UHS, 2);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 3300);
    /* TIEH_SEL 3: card 1's. */
    qtest_writel(qts, PMU_VO4, LDO_RESET | LDO_FORCE_TIEH_SEL | LDO_XPD |
                 LDO_TIEH_SEL(3));
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 1800);
    /* FORCE_TIEH_SEL clear: TIEH decides, whatever TIEH_SEL says. */
    qtest_writel(qts, PMU_VO4, LDO_RESET | LDO_XPD | LDO_TIEH_SEL(2));
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 1800);
    qtest_writel(qts, PMU_VO4, LDO_RESET | LDO_XPD | LDO_TIEH);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 3300);
    vo4_off(qts);
    g_assert_cmpuint(qom_uint(qts, PMU_PATH, "vo4-mv"), ==, 0);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad0-mv"), ==, 0);
    g_assert_cmpuint(qom_uint(qts, SDMMC_PATH, "pad1-mv"), ==, 3300);
    qtest_quit(qts);
}

/*
 * A card and pads at different voltages: a 3.3 V card does not see pads
 * at 1.8 V (no response); pads at 3.3 V read a 1.8 V card's highs as
 * lows (responses all 0, a response error; data all 0, an end bit error);
 * unpowered pads read every line low.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_mismatch(void)
{
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);
    uint8_t blk[512];
    uint32_t st, rca;

    clocks_on(qts);
    vo4_ldo(qts, 8, 4);
    card_clock(qts, 0, 50);
    cmd(qts, 0, CMD_INIT, 0, 0);
    g_assert_cmphex(cmd(qts, 0, R1, 8, 0x1aa), ==, INT_CMDD | INT_RTO);
    vo4_3v3(qts);
    cmd_ok(qts, 0, R1, 8, 0x1aa);
    qtest_quit(qts);

    qts = start(img, NULL);
    rca = uhs_card(qts);
    vo4_3v3(qts);
    st = cmd(qts, 0, R1 | CMD_CRC, 13, rca << 16);
    g_assert_cmphex(st & (INT_RE | INT_RCRC | INT_RTO), ==, INT_RE);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0);
    /* The card takes the read and sends block 1: it reads all 0. */
    st = read_data(qts, 0, 0, 17, 1, 512, blk);
    g_assert_cmphex(st & (INT_EBE | INT_DCRC | INT_DTO), ==,
                    INT_EBE | INT_DTO);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(blk[i], ==, 0);
    }
    /* Back at 1.8 V, it answers, still selected (transfer, 4). */
    vo4_ldo(qts, 8, 4);
    cmd_ok(qts, 0, R1, 13, rca << 16);
    g_assert_cmphex((qtest_readl(qts, SD_RESP(0)) >> 9) & 0xf, ==, 4);
    st = read_data(qts, 0, 0, 17, 1, 512, blk);
    g_assert_cmphex(st & INT_ERRORS, ==, 0);
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(blk[i], ==, pattern_byte(512 + i));
    }
    /* Unpowered pads: every line low, DAT0 busy, no response. */
    vo4_off(qts);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & STATUS_BUSY, ==,
                    STATUS_BUSY);
    g_assert_cmphex(cmd(qts, 0, R1, 13, rca << 16), ==, INT_CMDD | INT_RTO);
    qtest_quit(qts);
    unlink(img);
}

/* The SDIO PLL powered and SDIO_LS from it, divided by @div, at @phase. */
static void sdio_pll_clock(QTestState *qts, unsigned div, unsigned phase)
{
    uint32_t edges = EDGE_N(div - 1) | EDGE_L(div - 1) |
                     EDGE_H(div / 2 - 1) | DRV_SEL(1) | SAM_SEL(phase) |
                     SLF_DRV_SAM_EN;

    clock_update(qts, 0, false, 0);
    qtest_writel(qts, PMU_RF_PWC, qtest_readl(qts, PMU_RF_PWC) |
                 SDIO_PLL_XPD);
    qtest_writel(qts, PMU_IMM_HP_CK_POWER, TIE_HIGH_SDIOPLL);
    qtest_writel(qts, LP_HP_CLK_CTRL, qtest_readl(qts, LP_HP_CLK_CTRL) |
                 SDIO_PLL_CLK_EN);
    qtest_clock_step(qts, 20 * 1000);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | SDIO_LS_EN | SDIO_LS_SRC_SDIO);
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_writel(qts, PERI_CLK_CTRL02, edges | EDGE_UPDATE);
    qtest_writel(qts, SD_CLKDIV, 0);
    clock_update(qts, 0, true, 0);
    qtest_writel(qts, SD_TMOUT, (100000u << 8) | 0xff);
}

/* The sample phase, as ESP-IDF's tuning sets it (no edge update). */
static void sample_phase(QTestState *qts, unsigned div, unsigned phase)
{
    qtest_writel(qts, PERI_CLK_CTRL02, EDGE_N(div - 1) | EDGE_L(div - 1) |
                 EDGE_H(div / 2 - 1) | DRV_SEL(1) | SAM_SEL(phase) |
                 SLF_DRV_SAM_EN);
}

/*
 * The sample phase at SDR50 (100 MHz, 2.5 ns a phase) and DDR50 (50 MHz,
 * 5 ns), with the board's 4.0 ns round trip: at SDR50 phases 1 and 2
 * sample in the card's output window, phases 0 and 3 miss it, the
 * response failing its CRC check and the block its CRC; at DDR50 only
 * phase 1 catches both CMD and DAT. DDR on one side only is a CRC error.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_tuning(void)
{
    static const uint32_t sdr50[4] = {
        INT_RCRC | INT_DCRC, 0, 0, INT_RCRC | INT_DCRC,
    };
    static const uint32_t ddr50[4] = {
        INT_DCRC, 0, INT_RCRC | INT_DCRC, INT_RCRC,
    };
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);
    uint8_t blk[64], st[64];
    uint32_t rint;

    uhs_card(qts);
    cmd6(qts, 0x80fffff2, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 2);
    sdio_pll_clock(qts, 2, 0);
    for (unsigned phase = 0; phase < 4; phase++) {
        sample_phase(qts, 2, phase);
        rint = read_data(qts, 0, 0, 19, 0, 64, blk);
        g_assert_cmphex(rint & INT_ERRORS, ==, sdr50[phase]);
        g_assert_cmpint(!memcmp(blk, tuning_block, 64), ==, !sdr50[phase]);
    }
    qtest_quit(qts);

    qts = start(img, NULL);
    uhs_card(qts);
    /* The switch's status comes in SDR; the card is then in DDR50. */
    g_assert_cmphex(cmd6(qts, 0x80fffff4, st) & INT_ERRORS, ==, 0);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 4);
    g_assert_cmphex(cmd6(qts, 0x00ffffff, st) & INT_DCRC, ==, INT_DCRC);
    qtest_writel(qts, SD_UHS, 1u << 16);
    g_assert_cmphex(cmd6(qts, 0x00ffffff, st) & INT_ERRORS, ==, 0);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 4);
    sdio_pll_clock(qts, 4, 0);
    for (unsigned phase = 0; phase < 4; phase++) {
        sample_phase(qts, 4, phase);
        rint = read_data(qts, 0, 0, 19, 0, 64, blk);
        g_assert_cmphex(rint & INT_ERRORS, ==, ddr50[phase]);
        g_assert_cmpint(!memcmp(blk, tuning_block, 64), ==,
                        !(ddr50[phase] & INT_DCRC));
    }
    /* At phase 1, the host in SDR: a CRC error. */
    sample_phase(qts, 4, 1);
    qtest_writel(qts, SD_UHS, 0);
    rint = read_data(qts, 0, 0, 19, 0, 64, blk);
    g_assert_cmphex(rint & INT_ERRORS, ==, INT_DCRC);
    qtest_quit(qts);
    unlink(img);
}

/*
 * SDR104: the SDIO PLL's 200 MHz undivided in high-speed mode, the sample
 * phase DLL_CCLK_IN_SAM_PHASE's 64ths of 5 ns. The card's window of
 * 0.6 UI from 2.5 ns, through the 4.0 ns round trip, takes phases 20 to
 * 57: ESP-IDF's phases 3 to 7 (24 to 56), not 0 to 2.
 */
/* [spec:nuos:req:emu.esp32p4.sd-uhs/test] */
static void test_sdr104(void)
{
    g_autofree char *img = make_image(SDHC_SIZE);
    QTestState *qts = start(img, NULL);
    uint8_t blk[64], st[64];
    uint32_t rint;

    uhs_card(qts);
    cmd6(qts, 0x80fffff3, st);
    g_assert_cmpuint(cmd6_sel(st, 1), ==, 3);
    sdio_pll_clock(qts, 2, 0);
    clock_update(qts, 0, false, 0);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | SDIO_LS_EN | SDIO_LS_SRC_SDIO |
                 SDIO_HS_MODE);
    clock_update(qts, 0, true, 0);
    for (unsigned k = 0; k < 64; k++) {
        qtest_writel(qts, SD_DLL_CLK_CONF, DLL_SAM_PHASE(k));
        rint = read_data(qts, 0, 0, 19, 0, 64, blk);
        g_assert_cmphex(rint & INT_ERRORS, ==,
                        k >= 20 && k <= 57 ? 0 : INT_RCRC | INT_DCRC);
    }
    qtest_quit(qts);
    unlink(img);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/sd-uhs/s18a", test_s18a);
    qtest_add_func("/esp32p4/sd-uhs/switch", test_switch);
    qtest_add_func("/esp32p4/sd-uhs/switch-fails", test_switch_fails);
    qtest_add_func("/esp32p4/sd-uhs/cmd6", test_cmd6);
    qtest_add_func("/esp32p4/sd-uhs/cmd19", test_cmd19);
    qtest_add_func("/esp32p4/sd-uhs/ldo", test_ldo);
    qtest_add_func("/esp32p4/sd-uhs/mismatch", test_mismatch);
    qtest_add_func("/esp32p4/sd-uhs/tuning", test_tuning);
    qtest_add_func("/esp32p4/sd-uhs/sdr104", test_sdr104);
    return g_test_run();
}
