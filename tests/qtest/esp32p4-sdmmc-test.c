/*
 * QTest testcase for the ESP32-P4 SD/MMC host controller (SDHOST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The controller is clocked as ESP-IDF clocks it: SDMMC_SYS_CLK on, the
 * SDIO_LS clocks from PLL_F160M_CLK divided by 4 (40 MHz, 25 ns), and
 * card 0's clock divided by 2 * 50 to 400 kHz (2.5 us) for identification
 * or by 2 to 10 MHz for data. Slot 0's pads are on the PMU's regulator
 * VO4, off from reset, which the tests power at 3.3 V (bypass), as
 * ESP-IDF's on-chip LDO power control does; both slots' pins are set up
 * as ESP-IDF's SD/MMC host driver sets them up (esp32p4-sd-pins.h). The
 * cards are raw images of 1 MiB, standard capacity, byte addressed.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32p4-sd-pins.h"

#define SD                  0x50083000
#define SD_CTRL             (SD + 0x000)
#define SD_PWREN            (SD + 0x004)
#define SD_CLKDIV           (SD + 0x008)
#define SD_CLKSRC           (SD + 0x00c)
#define SD_CLKENA           (SD + 0x010)
#define SD_TMOUT            (SD + 0x014)
#define SD_CTYPE            (SD + 0x018)
#define SD_BLKSIZ           (SD + 0x01c)
#define SD_BYTCNT           (SD + 0x020)
#define SD_INTMASK          (SD + 0x024)
#define SD_CMDARG           (SD + 0x028)
#define SD_CMD              (SD + 0x02c)
#define SD_RESP(n)          (SD + 0x030 + 4 * (n))
#define SD_MINTSTS          (SD + 0x040)
#define SD_RINTSTS          (SD + 0x044)
#define SD_STATUS           (SD + 0x048)
#define SD_FIFOTH           (SD + 0x04c)
#define SD_CDETECT          (SD + 0x050)
#define SD_WRTPRT           (SD + 0x054)
#define SD_TCBCNT           (SD + 0x05c)
#define SD_TBBCNT           (SD + 0x060)
#define SD_DEBNCE           (SD + 0x064)
#define SD_USRID            (SD + 0x068)
#define SD_VERID            (SD + 0x06c)
#define SD_HCON             (SD + 0x070)
#define SD_UHS              (SD + 0x074)
#define SD_RST_N            (SD + 0x078)
#define SD_BMOD             (SD + 0x080)
#define SD_PLDMND           (SD + 0x084)
#define SD_DBADDR           (SD + 0x088)
#define SD_IDSTS            (SD + 0x08c)
#define SD_IDINTEN          (SD + 0x090)
#define SD_DSCADDR          (SD + 0x094)
#define SD_BUFADDR          (SD + 0x098)
#define SD_CARDTHRCTL       (SD + 0x100)
#define SD_EMMCDDR          (SD + 0x10c)
#define SD_ENSHIFT          (SD + 0x110)
#define SD_BUFFIFO          (SD + 0x200)
#define SD_CLK_EDGE_SEL     (SD + 0x800)
#define SD_RAW_INTS         (SD + 0x804)
#define SD_DLL_CLK_CONF     (SD + 0x808)
#define SD_DLL_CONF         (SD + 0x80c)

#define CTRL_CONTROLLER_RESET (1u << 0)
#define CTRL_FIFO_RESET     (1u << 1)
#define CTRL_DMA_RESET      (1u << 2)
#define CTRL_INT_ENABLE     (1u << 4)
#define CTRL_DMA_ENABLE     (1u << 5)
#define CTRL_USE_IDMAC      (1u << 25)

#define CMD_RESP            (1u << 6)
#define CMD_LONG            (1u << 7)
#define CMD_CRC             (1u << 8)
#define CMD_DATA            (1u << 9)
#define CMD_WRITE           (1u << 10)
#define CMD_AUTO_STOP       (1u << 12)
#define CMD_WAIT            (1u << 13)
#define CMD_STOP_ABORT      (1u << 14)
#define CMD_INIT            (1u << 15)
#define CMD_CARD(n)         ((n) << 16)
#define CMD_UPDATE_CLK      (1u << 21)
#define CMD_VOLT_SWITCH     (1u << 28)
#define CMD_HOLD            (1u << 29)
#define CMD_START           (1u << 31)
#define R1                  (CMD_RESP | CMD_CRC)
#define R2                  (CMD_RESP | CMD_LONG | CMD_CRC)
#define R3                  CMD_RESP

#define INT_CD              (1u << 0)
#define INT_CMDD            (1u << 2)
#define INT_DTO             (1u << 3)
#define INT_TXDR            (1u << 4)
#define INT_RXDR            (1u << 5)
#define INT_RCRC            (1u << 6)
#define INT_RTO             (1u << 8)
#define INT_DRTO            (1u << 9)
#define INT_FRUN            (1u << 11)
#define INT_HLE             (1u << 12)
#define INT_ACD             (1u << 14)

#define BMOD_SWR            (1u << 0)
#define BMOD_FB             (1u << 1)
#define BMOD_DE             (1u << 7)

#define IDSTS_TI            (1u << 0)
#define IDSTS_RI            (1u << 1)
#define IDSTS_FBE           (1u << 2)
#define IDSTS_DU            (1u << 4)
#define IDSTS_NIS           (1u << 8)
#define IDSTS_AIS           (1u << 9)
#define IDSTS_FBE_CODE(v)   (((v) >> 10) & 7)
#define IDSTS_FSM(v)        (((v) >> 13) & 0xf)

#define DES0_DIC            (1u << 1)
#define DES0_LD             (1u << 2)
#define DES0_FD             (1u << 3)
#define DES0_CH             (1u << 4)
#define DES0_OWN            (1u << 31)

/* SDMMC's clock and reset */
#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SDMMC_SYS_CLK_EN    (1u << 14)
#define PERI_CLK_CTRL01     (HP_CLKRST + 0x34)
#define PERI_CLK_CTRL02     (HP_CLKRST + 0x38)
#define LP_CLKRST           0x50111000
#define SDMMC_EMAC_RST_CTRL (LP_CLKRST + 0x4c)
#define RST_EN_SDMMC        (1u << 28)

/* The DMA APM and the AXI ICM */
#define DMA_PMS             0x500a6000
#define DMA_SDMMC_R         (DMA_PMS + 0x1bc)
#define DMA_SDMMC_W         (DMA_PMS + 0x1c0)
#define ICM                 0x500a4000
#define ICM_INT_RAW         (ICM + 0x0c)
#define ICM_CPU_ADDR        (ICM + 0x40)
#define ICM_CPU_INFO        (ICM + 0x44)
#define ICM_INT_CPU         0x4
#define ICM_WR              0x100
#define ICM_ID_SDMMC        6

/* The interrupt matrix's status of core 0, SDIO_HOST_INTR (source 23) */
#define INTMTX_STATUS0      (0x500d6000 + 0x200)
#define SRC_SDIO_HOST       23

/* L2MEM past the L2 cache, for descriptors and buffers */
#define DESC                0x4ff40000
#define BUF                 0x4ff41000

#define CCLK_NS             25
#define ID_CLK_NS           2500
#define DATA_CLK_NS         100

#define IMAGE_SIZE          (1 * 1024 * 1024)

#define SDMMC_PATH          "/machine/sdmmc"

static char *make_image(uint8_t seed)
{
    g_autofree uint8_t *data = g_malloc(IMAGE_SIZE);
    char *path;
    int fd;

    fd = g_file_open_tmp("esp32p4-sdmmc-XXXXXX.img", &path, NULL);
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

static QTestState *start_opts(const char *img0, const char *img1,
                              const char *opts)
{
    g_autofree char *d0 = img0 ?
        g_strdup_printf("-drive if=sd,index=0,format=raw,file=%s", img0) :
        g_strdup("");
    g_autofree char *d1 = img1 ?
        g_strdup_printf("-drive if=sd,index=1,format=raw,file=%s", img1) :
        g_strdup("");

    return qtest_initf("-M esp32p4 %s %s %s", d0, d1, opts);
}

static QTestState *start(const char *img0, const char *img1)
{
    return start_opts(img0, img1, "");
}

/* The PMU's VO4: XPD, FORCE_TIEH_SEL (TIEH_SEL 0) and TIEH (bypass) */
#define PMU_EXT_LDO_VO4     0x501151d8
#define VO4_ON_3V3          ((1u << 7) | (1u << 8) | (1u << 14))

/*
 * SDMMC_SYS_CLK, the SDIO_LS clocks at 40 MHz, slot 0's pads powered, and
 * both slots' pins.
 */
static void clocks_on(QTestState *qts)
{
    qtest_writel(qts, PMU_EXT_LDO_VO4,
                 qtest_readl(qts, PMU_EXT_LDO_VO4) | VO4_ON_3V3);
    uint32_t edges = (3u << 17) | (3u << 9) | (1u << 13) | (7u << 27);

    sd_pins(qts, 0);
    sd_pins(qts, 1);

    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | SDMMC_SYS_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | (1u << 24));
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_writel(qts, PERI_CLK_CTRL02, edges | (1u << 8));
}

/* Hand the CIU card @card's clock divided by 2 * @div, and wait. */
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

/* Send a command and wait for its command done: RINTSTS, cleared. */
static uint32_t cmd(QTestState *qts, unsigned card, uint32_t flags,
                    unsigned index, uint32_t arg)
{
    qtest_writel(qts, SD_CMDARG, arg);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_CARD(card) |
                 flags | index);
    return wait_int(qts, INT_CMDD, 100 * 1000 * 1000);
}

/* Identify and select card @card: its RCA. */
static uint32_t card_init(QTestState *qts, unsigned card)
{
    uint32_t st, rca;
    int tries;

    card_clock(qts, card, 50);
    st = cmd(qts, card, CMD_INIT, 0, 0);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    st = cmd(qts, card, R1 | CMD_WAIT, 8, 0x1aa);
    g_assert_cmphex(st & (INT_RTO | INT_RCRC), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0x1aa);
    for (tries = 0; tries < 20; tries++) {
        st = cmd(qts, card, R1 | CMD_WAIT, 55, 0);
        g_assert_cmphex(st & INT_RTO, ==, 0);
        st = cmd(qts, card, R3 | CMD_WAIT, 41, 0x40ff8000);
        g_assert_cmphex(st & (INT_RTO | INT_RCRC), ==, 0);
        if (qtest_readl(qts, SD_RESP(0)) & 0x80000000) {
            break;
        }
        qtest_clock_step(qts, 1000 * 1000);
    }
    g_assert_cmpint(tries, <, 20);
    /* R2: the CID, its manufacturer ID QEMU's 0xAA. */
    st = cmd(qts, card, R2 | CMD_WAIT, 2, 0);
    g_assert_cmphex(st & (INT_RTO | INT_RCRC), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(3)) >> 24, ==, 0xaa);
    st = cmd(qts, card, R1 | CMD_WAIT, 3, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    g_assert_cmphex(rca, !=, 0);
    st = cmd(qts, card, R1 | CMD_WAIT, 7, rca << 16);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    return rca;
}

/* Four-bit mode, on the card and on the bus, and the data clock. */
static void card_4bit(QTestState *qts, unsigned card, uint32_t rca)
{
    uint32_t st;

    st = cmd(qts, card, R1 | CMD_WAIT, 55, rca << 16);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    st = cmd(qts, card, R1 | CMD_WAIT, 6, 2);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    qtest_writel(qts, SD_CTYPE, 1u << card);
    card_clock(qts, card, 2);
}

static void desc_write(QTestState *qts, uint32_t addr, uint32_t des0,
                       uint32_t size, uint32_t buf, uint32_t next)
{
    qtest_writel(qts, addr, des0);
    qtest_writel(qts, addr + 4, size);
    qtest_writel(qts, addr + 8, buf);
    qtest_writel(qts, addr + 12, next);
}

static void idmac_on(QTestState *qts, uint32_t list)
{
    qtest_writel(qts, SD_CTRL, CTRL_DMA_ENABLE | CTRL_USE_IDMAC |
                 CTRL_INT_ENABLE);
    qtest_writel(qts, SD_BMOD, BMOD_SWR);
    qtest_writel(qts, SD_IDINTEN, 0x337);
    qtest_writel(qts, SD_DBADDR, list);
    qtest_writel(qts, SD_BMOD, BMOD_DE | BMOD_FB);
    qtest_writel(qts, SD_PLDMND, 1);
}

static bool source_level(QTestState *qts, int src)
{
    return qtest_readl(qts, INTMTX_STATUS0 + 4 * (src / 32)) &
           (1u << (src % 32));
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_reset_values(void)
{
    QTestState *qts = start(NULL, NULL);

    g_assert_cmphex(qtest_readl(qts, SD_CTRL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_TMOUT), ==, 0xffffff40);
    g_assert_cmphex(qtest_readl(qts, SD_BLKSIZ), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, SD_BYTCNT), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, SD_CMD), ==, CMD_HOLD);
    /* No cclk_in: the CIU's state holds its reset value. */
    g_assert_cmphex(qtest_readl(qts, SD_STATUS), ==, 0x716);
    g_assert_cmphex(qtest_readl(qts, SD_VERID), ==, 0x5432270a);
    g_assert_cmphex(qtest_readl(qts, SD_HCON), ==, 0x03444cc3);
    g_assert_cmphex(qtest_readl(qts, SD_RST_N), ==, 1);
    g_assert_cmphex(qtest_readl(qts, SD_CLK_EDGE_SEL), ==, 0x00820200);
    /* Card detect and write protect inputs, as the matrix defaults them. */
    g_assert_cmphex(qtest_readl(qts, SD_CDETECT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_WRTPRT), ==, 0);
    /* The P4 has no PWREN. */
    g_assert_cmphex(qtest_readl(qts, SD_PWREN), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_gating(void)
{
    QTestState *qts = start(NULL, NULL);

    /* SDMMC_SYS_CLK is off out of reset: writes are dropped. */
    qtest_writel(qts, SD_USRID, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SD_USRID), ==, 0);
    clocks_on(qts);
    qtest_writel(qts, SD_USRID, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SD_USRID), ==, 0x12345678);
    /* LP_CLKRST's SDMMC reset resets the block and holds it. */
    qtest_writel(qts, SDMMC_EMAC_RST_CTRL, RST_EN_SDMMC);
    g_assert_cmphex(qtest_readl(qts, SD_USRID), ==, 0);
    qtest_writel(qts, SD_USRID, 1);
    g_assert_cmphex(qtest_readl(qts, SD_USRID), ==, 0);
    qtest_writel(qts, SDMMC_EMAC_RST_CTRL, 0);
    qtest_writel(qts, SD_USRID, 1);
    g_assert_cmphex(qtest_readl(qts, SD_USRID), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_register_masks(void)
{
    QTestState *qts = start(NULL, NULL);
    static const struct {
        uint32_t addr, mask;
    } regs[] = {
        { SD_CLKDIV, 0xffffffff },
        { SD_CLKSRC, 0xf },
        { SD_CLKENA, 0x00030003 },
        { SD_TMOUT, 0xffffffff },
        { SD_CTYPE, 0x00030003 },
        { SD_BLKSIZ, 0xffff },
        { SD_BYTCNT, 0xffffffff },
        { SD_INTMASK, 0x3ffff },
        { SD_CMDARG, 0xffffffff },
        { SD_FIFOTH, 0x77ff0fff },
        { SD_DEBNCE, 0xffffff },
        { SD_USRID, 0xffffffff },
        { SD_UHS, 0x00030003 },
        { SD_RST_N, 0x3 },
        { SD_DBADDR, 0xfffffffc },
        { SD_IDINTEN, 0x337 },
        { SD_CARDTHRCTL, 0xffff0007 },
        { SD_EMMCDDR, 0x80000003 },
        { SD_ENSHIFT, 0xf },
        { SD_CLK_EDGE_SEL, 0x01ffffff },
        { SD_DLL_CLK_CONF, 0x001fffff },
    };

    clocks_on(qts);
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        qtest_writel(qts, regs[i].addr, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, regs[i].addr), ==, regs[i].mask);
        qtest_writel(qts, regs[i].addr, 0);
        g_assert_cmphex(qtest_readl(qts, regs[i].addr), ==, 0);
    }
    /* Byte writes change their byte alone. */
    qtest_writel(qts, SD_USRID, 0x11223344);
    qtest_writeb(qts, SD_USRID + 2, 0xaa);
    g_assert_cmphex(qtest_readl(qts, SD_USRID), ==, 0x11aa3344);
    g_assert_cmphex(qtest_readb(qts, SD_USRID + 3), ==, 0x11);
    /* CMD keeps no boot bits on the P4. */
    qtest_writel(qts, SD_CMD, 0x7fffffff);
    g_assert_cmphex(qtest_readl(qts, SD_CMD), ==, 0x30ffffff);
    /* BMOD: no descriptor skip length; PBL mirrors FIFOTH's MSIZE. */
    qtest_writel(qts, SD_FIFOTH, 5u << 28);
    qtest_writel(qts, SD_BMOD, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SD_BMOD), ==,
                    BMOD_DE | BMOD_FB | (5u << 8));
    /* Read-only registers keep their values. */
    qtest_writel(qts, SD_VERID, 0);
    g_assert_cmphex(qtest_readl(qts, SD_VERID), ==, 0x5432270a);
    /* DLL_CONF: DLL_CAL_STOP; DLL_CAL_END reads 0. */
    qtest_writel(qts, SD_DLL_CONF, 3);
    g_assert_cmphex(qtest_readl(qts, SD_DLL_CONF), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_ctrl_resets(void)
{
    QTestState *qts = start(NULL, NULL);
    uint32_t edges = (3u << 17) | (3u << 9) | (1u << 13) | (7u << 27);

    clocks_on(qts);
    /* Each bit reads set until two bus and two cclk_in cycles pass. */
    qtest_writel(qts, SD_CTRL, CTRL_CONTROLLER_RESET | CTRL_FIFO_RESET |
                 CTRL_DMA_RESET);
    g_assert_cmphex(qtest_readl(qts, SD_CTRL), ==, 7);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CTRL), ==, 0);

    /* Without cclk_in the controller reset never completes... */
    qtest_writel(qts, PERI_CLK_CTRL02, edges & ~(7u << 27));
    qtest_writel(qts, SD_CTRL, CTRL_CONTROLLER_RESET);
    qtest_clock_step(qts, 100 * 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CTRL), ==, CTRL_CONTROLLER_RESET);
    /* ...but the DMA reset, on the bus clock alone, does. */
    qtest_writel(qts, SD_CTRL, CTRL_DMA_RESET);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CTRL) & CTRL_DMA_RESET, ==, 0);
    /* Once cclk_in runs, it completes. */
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CTRL), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_clock_update_and_hle(void)
{
    QTestState *qts = start(NULL, NULL);
    uint32_t edges = (3u << 17) | (3u << 9) | (1u << 13) | (7u << 27);

    clocks_on(qts);
    /* A clock update: START_CMD clears after two cclk_in cycles, no CMDD. */
    qtest_writel(qts, SD_CLKENA, 1);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_WAIT);
    qtest_clock_step(qts, 2 * CCLK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, CMD_START);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);

    /* With SDIO_LS_DRV_CLK stopped, the CIU takes no command. */
    qtest_writel(qts, PERI_CLK_CTRL02, edges & ~(1u << 28));
    qtest_writel(qts, SD_CMDARG, 0x1111);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK);
    qtest_clock_step(qts, 100 * 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, CMD_START);
    /* While START_CMD is set the command registers are locked: HLE. */
    qtest_writel(qts, SD_CMDARG, 0x2222);
    qtest_writel(qts, SD_BYTCNT, 0x400);
    qtest_writel(qts, SD_CLKDIV, 9);
    g_assert_cmphex(qtest_readl(qts, SD_CMDARG), ==, 0x1111);
    g_assert_cmphex(qtest_readl(qts, SD_BYTCNT), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, SD_CLKDIV), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_HLE);
    /* Interrupt status: RINTSTS clears by writing 1. */
    qtest_writel(qts, SD_INTMASK, INT_HLE);
    g_assert_cmphex(qtest_readl(qts, SD_MINTSTS), ==, INT_HLE);
    qtest_writel(qts, SD_RINTSTS, INT_HLE);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_clock_step(qts, 2 * CCLK_NS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_empty_slot(void)
{
    QTestState *qts = start(NULL, NULL);
    uint32_t st;

    clocks_on(qts);
    card_clock(qts, 0, 50);
    /* No card: a response timeout after the command and 64 clocks. */
    qtest_writel(qts, SD_CMDARG, 0x1aa);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | 8);
    qtest_clock_step(qts, 2 * CCLK_NS + (48 + 64) * ID_CLK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    /* On the bus: STATUS shows the response wait and its card's DAT3. */
    st = qtest_readl(qts, SD_STATUS);
    g_assert_cmphex((st >> 4) & 0xf, ==, 11);
    g_assert_cmphex(st & (1u << 8), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_RTO | INT_CMDD);
    st = qtest_readl(qts, SD_STATUS);
    g_assert_cmphex((st >> 4) & 0xf, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_card_init_and_fifo(void)
{
    g_autofree char *img = make_image(0x11);
    /* The controller's own cycles, with a card that takes no time. */
    QTestState *qts = start_opts(img, NULL, "-global sd-card.timing=off");
    uint32_t st, rca, word;
    uint8_t got[512];
    int64_t t;

    clocks_on(qts);
    qtest_writel(qts, SD_CMDARG, 0x1aa);
    card_clock(qts, 0, 50);
    /* CMD8's command, the card's turnaround and R7, in card clocks. */
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_INIT | 0);
    wait_int(qts, INT_CMDD, 1000 * 1000);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | 8);
    qtest_clock_step(qts, 2 * CCLK_NS + (48 + 2 + 48) * ID_CLK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_CMDD);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)), ==, 0x1aa);
    g_assert_cmphex((qtest_readl(qts, SD_STATUS) >> 11) & 0x3f, ==, 8);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    /* ACMD41 with a CRC check: R3 has none, so its check fails. */
    cmd(qts, 0, R1 | CMD_WAIT, 55, 0);
    st = cmd(qts, 0, R1 | CMD_WAIT, 41, 0x40ff8000);
    g_assert_cmphex(st & INT_RCRC, ==, INT_RCRC);
    g_assert_cmphex((qtest_readl(qts, SD_STATUS) >> 11) & 0x3f, ==, 0x3f);

    rca = card_init(qts, 0);
    /*
     * The controller's voltage switch: CMD11, which a QEMU card, not
     * UHS-I, does not answer; without the card holding its lines low
     * there is no voltage switch interrupt.
     */
    st = cmd(qts, 0, R1 | CMD_WAIT | CMD_VOLT_SWITCH, 11, 0);
    g_assert_cmphex(st, ==, INT_RTO | INT_CMDD);
    card_4bit(qts, 0, rca);

    /* A block read through the FIFO, at 4 bits and 10 MHz. */
    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 512);
    qtest_writel(qts, SD_FIFOTH, (7u << 16) | 8);
    qtest_writel(qts, SD_CMDARG, 3 * 512);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | 17);
    t = 2 * CCLK_NS + 98 * DATA_CLK_NS;
    qtest_clock_step(qts, t);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_CMDD);
    g_assert_cmphex(qtest_readl(qts, SD_STATUS) & (1u << 10), ==, 1u << 10);
    /* Access time, start bit, 1024 clocks of data, CRC and end bit. */
    qtest_clock_step(qts, (2 + 1 + 1024 + 16 + 1) * DATA_CLK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    st = qtest_readl(qts, SD_RINTSTS);
    g_assert_cmphex(st & (INT_DTO | INT_RXDR), ==, INT_DTO | INT_RXDR);
    g_assert_cmphex(qtest_readl(qts, SD_TCBCNT), ==, 512);
    g_assert_cmphex((qtest_readl(qts, SD_STATUS) >> 17) & 0x1fff, ==, 128);
    for (int i = 0; i < 128; i++) {
        word = qtest_readl(qts, SD_BUFFIFO);
        memcpy(&got[4 * i], &word, 4);
    }
    for (int i = 0; i < 512; i++) {
        g_assert_cmphex(got[i], ==, image_byte(0x11, 3 * 512 + i));
    }
    g_assert_cmphex(qtest_readl(qts, SD_TBBCNT), ==, 512);

    /*
     * The bus width sets the data's clocks: 512 in 8-bit mode, 256 more
     * halved in DDR mode. (A QEMU card takes any width.)
     */
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    qtest_writel(qts, SD_CTYPE, 1u << 16);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | 17);
    qtest_clock_step(qts, t + (2 + 1 + 512 + 16 + 1) * DATA_CLK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    qtest_writel(qts, SD_UHS, 1u << 16);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | 17);
    qtest_clock_step(qts, t + (2 + 1 + 256 + 16 + 1) * DATA_CLK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    qtest_writel(qts, SD_UHS, 0);
    qtest_writel(qts, SD_CTYPE, 1);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    /* Reading it empty is an underrun. */
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    qtest_readl(qts, SD_BUFFIFO);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_FRUN);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);

    /* A block write through the FIFO: TXDR asks for the data. */
    qtest_writel(qts, SD_CMDARG, 5 * 512);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | CMD_WRITE | 24);
    qtest_clock_step(qts, 2 * CCLK_NS);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_TXDR, ==, INT_TXDR);
    for (int i = 0; i < 128; i++) {
        qtest_writel(qts, SD_BUFFIFO, 0xa5a5a5a5 ^ i);
    }
    st = wait_int(qts, INT_DTO, 10 * 1000 * 1000);
    g_assert_cmphex(st & (INT_CMDD | INT_DTO), ==, INT_CMDD | INT_DTO);
    g_assert_cmphex(qtest_readl(qts, SD_TCBCNT), ==, 512);
    g_assert_cmphex(qtest_readl(qts, SD_TBBCNT), ==, 512);

    /* Read it back. */
    qtest_writel(qts, SD_CMDARG, 5 * 512);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_WAIT | R1 |
                 CMD_DATA | 17);
    wait_int(qts, INT_DTO, 10 * 1000 * 1000);
    for (int i = 0; i < 128; i++) {
        g_assert_cmphex(qtest_readl(qts, SD_BUFFIFO), ==, 0xa5a5a5a5 ^ i);
    }
    qtest_quit(qts);
    unlink(img);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_idmac(void)
{
    g_autofree char *img = make_image(0x22);
    QTestState *qts = start(img, NULL);
    uint32_t st, rca, ids;

    clocks_on(qts);
    rca = card_init(qts, 0);
    card_4bit(qts, 0, rca);
    (void)rca;

    /* Two blocks with CMD18 and auto stop, into two chained buffers. */
    desc_write(qts, DESC, DES0_OWN | DES0_FD | DES0_CH, 512, BUF,
               DESC + 64);
    desc_write(qts, DESC + 64, DES0_OWN | DES0_CH | DES0_LD, 512,
               BUF + 0x400, 0);
    idmac_on(qts, DESC);
    g_assert_cmphex(qtest_readl(qts, SD_DSCADDR), ==, DESC);
    g_assert_cmphex(IDSTS_FSM(qtest_readl(qts, SD_IDSTS)), !=, 0);
    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 1024);
    cmd(qts, 0, R1 | CMD_WAIT | CMD_DATA | CMD_AUTO_STOP, 18, 7 * 512);
    st = wait_int(qts, INT_ACD, 10 * 1000 * 1000);
    g_assert_cmphex(st & INT_DTO, ==, INT_DTO);
    ids = qtest_readl(qts, SD_IDSTS);
    g_assert_cmphex(ids & (IDSTS_RI | IDSTS_NIS), ==, IDSTS_RI | IDSTS_NIS);
    g_assert_cmphex(IDSTS_FSM(ids), ==, 0);
    /* The auto stop's R1 is in RESP1: the card was sending data. */
    g_assert_cmphex((qtest_readl(qts, SD_RESP(1)) >> 9) & 0xf, ==, 5);
    g_assert_cmphex((qtest_readl(qts, SD_STATUS) >> 11) & 0x3f, ==, 12);
    /* Both descriptors are the host's again. */
    g_assert_cmphex(qtest_readl(qts, DESC) & DES0_OWN, ==, 0);
    g_assert_cmphex(qtest_readl(qts, DESC + 64) & DES0_OWN, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_DSCADDR), ==, DESC + 64);
    for (int i = 0; i < 1024; i += 37) {
        uint32_t addr = (i < 512 ? BUF : BUF + 0x400) + i % 512;

        g_assert_cmphex(qtest_readb(qts, addr), ==,
                        image_byte(0x22, 7 * 512 + i));
    }
    qtest_writel(qts, SD_IDSTS, 0x337);
    g_assert_cmphex(qtest_readl(qts, SD_IDSTS), ==, 0);

    /* CMD25 writes the buffers to blocks 20 and 21; DIC holds off TI. */
    desc_write(qts, DESC, DES0_OWN | DES0_FD | DES0_CH | DES0_DIC, 512, BUF,
               DESC + 64);
    desc_write(qts, DESC + 64, DES0_OWN | DES0_CH | DES0_LD, 512,
               BUF + 0x400, 0);
    qtest_writel(qts, SD_DBADDR, DESC);
    qtest_writel(qts, SD_PLDMND, 1);
    qtest_writel(qts, SD_BYTCNT, 1024);
    cmd(qts, 0, R1 | CMD_WAIT | CMD_DATA | CMD_WRITE | CMD_AUTO_STOP, 25,
        20 * 512);
    st = wait_int(qts, INT_ACD, 10 * 1000 * 1000);
    g_assert_cmphex(st & INT_DTO, ==, INT_DTO);
    g_assert_cmphex(qtest_readl(qts, SD_IDSTS) & (IDSTS_TI | IDSTS_RI), ==,
                    IDSTS_TI);
    g_assert_cmphex(qtest_readl(qts, SD_TBBCNT), ==, 1024);

    /* And read them back through the FIFO. */
    qtest_writel(qts, SD_CTRL, CTRL_INT_ENABLE);
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd(qts, 0, R1 | CMD_WAIT | CMD_DATA, 17, 21 * 512);
    wait_int(qts, INT_DTO, 10 * 1000 * 1000);
    for (int i = 0; i < 128; i++) {
        g_assert_cmphex(qtest_readl(qts, SD_BUFFIFO), ==,
                        qtest_readl(qts, BUF + 0x400 + 4 * i));
    }
    qtest_quit(qts);
    unlink(img);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_idmac_suspend(void)
{
    g_autofree char *img = make_image(0x33);
    QTestState *qts = start(img, NULL);
    uint32_t rca, ids, st;

    clocks_on(qts);
    rca = card_init(qts, 0);
    card_4bit(qts, 0, rca);

    /* The second descriptor is still the host's: DU, and a suspend. */
    desc_write(qts, DESC, DES0_OWN | DES0_FD | DES0_CH, 512, BUF,
               DESC + 64);
    desc_write(qts, DESC + 64, DES0_CH | DES0_LD, 512, BUF + 0x400, 0);
    idmac_on(qts, DESC);
    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 1024);
    cmd(qts, 0, R1 | CMD_WAIT | CMD_DATA | CMD_AUTO_STOP, 18, 0);
    qtest_clock_step(qts, 1000 * 1000);
    ids = qtest_readl(qts, SD_IDSTS);
    g_assert_cmphex(ids & (IDSTS_DU | IDSTS_AIS | IDSTS_RI), ==,
                    IDSTS_DU | IDSTS_AIS | IDSTS_RI);
    g_assert_cmphex(IDSTS_FSM(ids), ==, 1);
    g_assert_cmphex(qtest_readl(qts, SD_DSCADDR), ==, DESC + 64);
    /* The second block has come in, and waits in the FIFO. */
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_DTO, ==, INT_DTO);
    g_assert_cmphex((qtest_readl(qts, SD_STATUS) >> 17) & 0x1fff, ==, 128);
    /* Hand it over and resume. */
    qtest_writel(qts, DESC + 64, DES0_OWN | DES0_CH | DES0_LD);
    qtest_writel(qts, SD_PLDMND, 1);
    st = wait_int(qts, INT_ACD, 10 * 1000 * 1000);
    (void)st;
    g_assert_cmphex(qtest_readl(qts, DESC + 64) & DES0_OWN, ==, 0);
    g_assert_cmphex(qtest_readb(qts, BUF + 0x400 + 10), ==,
                    image_byte(0x33, 512 + 10));
    qtest_quit(qts);
    unlink(img);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_idmac_refused(void)
{
    g_autofree char *img = make_image(0x44);
    QTestState *qts = start(img, NULL);
    uint32_t rca, ids;

    clocks_on(qts);
    rca = card_init(qts, 0);
    card_4bit(qts, 0, rca);

    /* The DMA APM lets SDMMC read memory but not write it. */
    qtest_writel(qts, DMA_SDMMC_W, 0);
    desc_write(qts, DESC, DES0_OWN | DES0_FD | DES0_CH | DES0_LD, 512, BUF,
               0);
    qtest_writel(qts, BUF, 0x5a5a5a5a);
    idmac_on(qts, DESC);
    qtest_writel(qts, SD_BLKSIZ, 512);
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd(qts, 0, R1 | CMD_WAIT | CMD_DATA, 17, 0);
    wait_int(qts, INT_DTO, 10 * 1000 * 1000);
    ids = qtest_readl(qts, SD_IDSTS);
    g_assert_cmphex(ids & (IDSTS_FBE | IDSTS_AIS | IDSTS_RI), ==,
                    IDSTS_FBE | IDSTS_AIS);
    g_assert_cmphex(IDSTS_FBE_CODE(ids), ==, 2);
    g_assert_cmphex(qtest_readl(qts, BUF), ==, 0x5a5a5a5a);
    /* The AXI ICM records the refusal, by SDMMC's master ID. */
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW) & ICM_INT_CPU, ==,
                    ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==,
                    ICM_WR | ICM_ID_SDMMC);
    /* Stopped until a DMA reset. */
    qtest_writel(qts, DMA_SDMMC_W, 0xffffffff);
    qtest_writel(qts, SD_PLDMND, 1);
    g_assert_cmphex(qtest_readl(qts, BUF), ==, 0x5a5a5a5a);
    qtest_writel(qts, SD_CTRL, CTRL_DMA_ENABLE | CTRL_USE_IDMAC |
                 CTRL_DMA_RESET | CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(IDSTS_FSM(qtest_readl(qts, SD_IDSTS)), ==, 0);
    qtest_quit(qts);
    unlink(img);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_card_detect(void)
{
    QTestState *qts = start(NULL, NULL);

    clocks_on(qts);
    /* The filter counts SDMMC_SYS_CLK: SYS_CLK, 20 MHz out of reset. */
    qtest_writel(qts, SD_DEBNCE, 1000);
    qtest_set_irq_in(qts, SDMMC_PATH, "card-detect-n", 1, 1);
    qtest_clock_step(qts, 1000 * 50 - 1);
    g_assert_cmphex(qtest_readl(qts, SD_CDETECT), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SD_CDETECT), ==, 2);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, INT_CD);
    qtest_writel(qts, SD_RINTSTS, INT_CD);
    /* A bounce shorter than the filter changes nothing. */
    qtest_set_irq_in(qts, SDMMC_PATH, "card-detect-n", 1, 0);
    qtest_clock_step(qts, 1000);
    qtest_set_irq_in(qts, SDMMC_PATH, "card-detect-n", 1, 1);
    qtest_clock_step(qts, 100 * 1000);
    g_assert_cmphex(qtest_readl(qts, SD_CDETECT), ==, 2);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    /* Write protect, straight from its input. */
    qtest_set_irq_in(qts, SDMMC_PATH, "card-write-prt", 0, 1);
    g_assert_cmphex(qtest_readl(qts, SD_WRTPRT), ==, 1);
    /* An SDIO interrupt request, low on card-int-n, while it lasts. */
    qtest_set_irq_in(qts, SDMMC_PATH, "card-int-n", 0, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 1u << 16);
    qtest_writel(qts, SD_RINTSTS, 1u << 16);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 1u << 16);
    qtest_set_irq_in(qts, SDMMC_PATH, "card-int-n", 0, 1);
    qtest_writel(qts, SD_RINTSTS, 1u << 16);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_interrupt(void)
{
    QTestState *qts = start(NULL, NULL);

    clocks_on(qts);
    card_clock(qts, 0, 50);
    qtest_writel(qts, SD_INTMASK, INT_CMDD);
    cmd(qts, 0, R1, 8, 0);
    g_assert_false(source_level(qts, SRC_SDIO_HOST));
    /* RINTSTS & INTMASK, gated by CTRL.INT_ENABLE */
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | R1 | 8);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, SD_MINTSTS), ==, INT_CMDD);
    g_assert_false(source_level(qts, SRC_SDIO_HOST));
    qtest_writel(qts, SD_CTRL, CTRL_INT_ENABLE);
    g_assert_true(source_level(qts, SRC_SDIO_HOST));
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    g_assert_false(source_level(qts, SRC_SDIO_HOST));
    /* The IDMAC's summaries, with their enables */
    qtest_writel(qts, SD_CTRL, CTRL_INT_ENABLE | CTRL_DMA_ENABLE |
                 CTRL_USE_IDMAC);
    qtest_writel(qts, SD_IDINTEN, IDSTS_DU);
    qtest_writel(qts, DESC, 0);
    qtest_writel(qts, SD_DBADDR, DESC);
    qtest_writel(qts, SD_BMOD, BMOD_DE);
    qtest_writel(qts, SD_PLDMND, 1);
    g_assert_cmphex(qtest_readl(qts, SD_IDSTS) & 0x3ff, ==,
                    IDSTS_DU | IDSTS_AIS);
    g_assert_false(source_level(qts, SRC_SDIO_HOST));
    qtest_writel(qts, SD_IDINTEN, IDSTS_DU | IDSTS_AIS);
    g_assert_true(source_level(qts, SRC_SDIO_HOST));
    qtest_writel(qts, SD_IDSTS, IDSTS_AIS);
    g_assert_false(source_level(qts, SRC_SDIO_HOST));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc/test] */
static void test_two_slots(void)
{
    g_autofree char *img0 = make_image(0x55);
    g_autofree char *img1 = make_image(0x66);
    QTestState *qts = start(img0, img1);
    uint32_t rca, word;

    clocks_on(qts);
    rca = card_init(qts, 1);
    (void)rca;
    /* Slot 1's card on its own divider; slot 0's card is untouched. */
    g_assert_cmphex(qtest_readl(qts, SD_CLKSRC), ==, 1u << 2);
    qtest_writel(qts, SD_BYTCNT, 512);
    cmd(qts, 1, R1 | CMD_WAIT | CMD_DATA, 17, 512);
    wait_int(qts, INT_DTO, 100 * 1000 * 1000);
    word = qtest_readl(qts, SD_BUFFIFO);
    g_assert_cmphex(word & 0xff, ==, image_byte(0x66, 512));
    /* Slot 0's card is still idle: CMD3 there before CMD2 goes unheard. */
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_clock_step(qts, 1000);
    card_clock(qts, 0, 50);
    g_assert_cmphex(cmd(qts, 0, R1 | CMD_WAIT, 3, 0) & INT_RTO, ==, INT_RTO);
    qtest_quit(qts);
    unlink(img0);
    unlink(img1);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/sdmmc/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/sdmmc/gating", test_gating);
    qtest_add_func("/esp32p4/sdmmc/register-masks", test_register_masks);
    qtest_add_func("/esp32p4/sdmmc/ctrl-resets", test_ctrl_resets);
    qtest_add_func("/esp32p4/sdmmc/clock-update-hle",
                   test_clock_update_and_hle);
    qtest_add_func("/esp32p4/sdmmc/empty-slot", test_empty_slot);
    qtest_add_func("/esp32p4/sdmmc/card-init-fifo", test_card_init_and_fifo);
    qtest_add_func("/esp32p4/sdmmc/idmac", test_idmac);
    qtest_add_func("/esp32p4/sdmmc/idmac-suspend", test_idmac_suspend);
    qtest_add_func("/esp32p4/sdmmc/idmac-refused", test_idmac_refused);
    qtest_add_func("/esp32p4/sdmmc/card-detect", test_card_detect);
    qtest_add_func("/esp32p4/sdmmc/interrupt", test_interrupt);
    qtest_add_func("/esp32p4/sdmmc/two-slots", test_two_slots);
    return g_test_run();
}
