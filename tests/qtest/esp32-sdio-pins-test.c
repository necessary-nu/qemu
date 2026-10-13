/*
 * QTest testcase for the ESP32 SDIO slave's pins: its host reaches it only
 * through pads set up as the hardware needs them
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The machine's sdio-host option wires an SD host, an SDHCI at 0x22000000,
 * to the slave's slot 1 pads (function 4, SD_*): CLK GPIO14, CMD GPIO15,
 * DAT0 GPIO2, DAT1 GPIO4, DAT2 GPIO12, DAT3 GPIO13; their pulls are
 * RTCIO's RUE and RDE (errata [GPIO-3.6]) and the board's ("sd-pull-up").
 * Set up as ESP-IDF's sdio_slave driver sets them up (esp32-sdio-pins.h),
 * the slave answers; set up otherwise, each as the board would have it:
 * a command whose CLK or CMD does not reach the slave goes unanswered (a
 * command timeout where CMD is held high, CRC and end bit errors where it
 * is held low), as does one whose response does not reach the host,
 * though the slave took it; a read whose DAT0 does not follow the slave
 * times out, one with another DAT line not following fails its CRC; a
 * write block with a DAT line not reaching the slave fails the slave's CRC
 * check, which it counts and discards the block; the slave's interrupt
 * reaches the host only on DAT1's pad; floating lines read low.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "esp32-sdio-pins.h"

#define SLC                     0x3ff58000
#define SLC_CONF0               (SLC + 0x00)
#define SLC_0INT_RAW            (SLC + 0x04)
#define SLC_0TX_LINK            (SLC + 0x40)
#define SLC_INTVEC_TOHOST       (SLC + 0x4c)
#define SLC_0TOKEN1             (SLC + 0x54)
#define SLC_SDIO_CRC_ST0        (SLC + 0x11c)
#define SLC_SDIO_CRC_ST1        (SLC + 0x120)
#define SLCINT_TX_DONE          (1u << 14)
#define LINK_START              (1u << 29)
#define TOKEN_INC_MORE          (1u << 14)
#define CRC_ST1_ERR_CNT_CLR     (1u << 31)

#define SLCHOST                 0x3ff55000
#define HOST_CONF_W0            0x6c
#define HOST_FUNC1_INT_ENA      0xdc
#define HOST_INT_CLR            0xd4

#define HINF_CFG_DATA1          0x3ff4b004
#define CFG1_IOREADY1           (1u << 1)

#define DESC_OWNER              (1u << 31)
#define DESC_EOF                (1u << 30)

/* The SDHCI wired to the slave's pads */
#define HOST                    0x22000000
#define SDHC_BLKSIZE            0x04
#define SDHC_BLKCNT             0x06
#define SDHC_ARGUMENT           0x08
#define SDHC_TRNMOD             0x0c
#define SDHC_CMDREG             0x0e
#define SDHC_RSPREG0            0x10
#define SDHC_BDATA              0x20
#define SDHC_PRNSTS             0x24
#define SDHC_HOSTCTL            0x28
#define SDHC_CLKCON             0x2c
#define SDHC_SWRST              0x2f
#define SDHC_NORINTSTS          0x30
#define SDHC_ERRINTSTS          0x32
#define SDHC_NORINTSTSEN        0x34
#define SDHC_ERRINTSTSEN        0x36
#define TRNS_READ               0x10
#define CMD_RSP48               0x02
#define CMD_CRC_CHECK           0x08
#define CMD_INDEX_CHECK         0x10
#define CMD_DATA                0x20
#define R5                      (CMD_RSP48 | CMD_CRC_CHECK | CMD_INDEX_CHECK)
#define CTRL_4BITBUS            0x02
#define RESET_CMD               0x02
#define RESET_DATA              0x04
#define NIS_CMDCMP              0x0001
#define NIS_TRSCMP              0x0002
#define NIS_WBUFRDY             0x0010
#define NIS_RBUFRDY             0x0020
#define NIS_CARDINT             0x0100
#define NIS_ERR                 0x8000
#define EIS_CMDTIMEOUT          0x0001
#define EIS_CMDCRC              0x0002
#define EIS_CMDEND              0x0004
#define EIS_CMDIDX              0x0008
#define EIS_DATATIMEOUT         0x0010
#define EIS_DATACRC             0x0020
#define EIS_DATAEND             0x0040
#define PRNSTS_DAT(n)           (1u << (20 + (n)))
#define PRNSTS_CMD              (1u << 24)

#define R5_FLAGS(r)             (((r) >> 8) & 0xff)
#define R5_DATA(r)              ((r) & 0xff)
#define IO_WRITE                (1u << 31)
#define IO_RAW                  (1u << 27)
#define IO_OP_INC               (1u << 26)
#define CCCR_IOE                0x02
#define CCCR_IEN                0x04
#define CCCR_BUS_IF             0x07
#define DATA_END                0x1f800

/* DMA-capable SRAM for descriptors and buffers */
#define RAM                     0x3ffc0000

#define BOARD_PULL_UP           "sd-pull-up=10000"

static QTestState *start(const char *board)
{
    QTestState *qts = qtest_initf("-M esp32,sdio-host=on%s%s -nic none",
                                  *board ? "," : "", board);

    qtest_writeb(qts, HOST + SDHC_SWRST, 1);
    qtest_writew(qts, HOST + SDHC_CLKCON, 0x0005);
    qtest_writew(qts, HOST + SDHC_NORINTSTSEN, 0xffff);
    qtest_writew(qts, HOST + SDHC_ERRINTSTSEN, 0xffff);
    return qts;
}

static uint32_t mux_addr(unsigned pin)
{
    return sd_pin_mux_addr(pin);
}

static void mux_bits(QTestState *qts, unsigned pin, uint32_t set,
                     uint32_t clear)
{
    uint32_t a = mux_addr(pin);

    qtest_writel(qts, a, (qtest_readl(qts, a) & ~clear) | set);
}

static void func_sel(QTestState *qts, unsigned pin, unsigned func)
{
    mux_bits(qts, pin, SDP_MCU_SEL(func), SDP_MCU_SEL_MASK);
}

/*
 * Issue a command; returns the error interrupts it raised, 0 with its
 * response in *resp.
 */
static uint16_t cmd(QTestState *qts, uint8_t idx, uint32_t arg,
                    uint16_t flags, uint32_t *resp)
{
    uint16_t err;

    qtest_writew(qts, HOST + SDHC_NORINTSTS, 0xffff);
    qtest_writew(qts, HOST + SDHC_ERRINTSTS, 0xffff);
    qtest_writel(qts, HOST + SDHC_ARGUMENT, arg);
    qtest_writew(qts, HOST + SDHC_CMDREG, (idx << 8) | flags);
    g_assert_true(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CMDCMP);
    err = qtest_readw(qts, HOST + SDHC_ERRINTSTS);
    if (!err && resp) {
        *resp = qtest_readl(qts, HOST + SDHC_RSPREG0);
    }
    return err;
}

static uint32_t must_cmd(QTestState *qts, uint8_t idx, uint32_t arg,
                         uint16_t flags)
{
    uint32_t r = 0;

    g_assert_cmphex(cmd(qts, idx, arg, flags, &r), ==, 0);
    return r;
}

/* Bring the card up: CMD5, CMD3 and CMD7, and enable function 1. */
static void card_init(QTestState *qts)
{
    g_assert_cmphex(must_cmd(qts, 5, 0, CMD_RSP48), ==, 0xa0ff8000);
    must_cmd(qts, 5, 0x00200000, CMD_RSP48);
    g_assert_cmphex(must_cmd(qts, 3, 0, CMD_RSP48) >> 16, ==, 1);
    must_cmd(qts, 7, 1u << 16, CMD_RSP48);
    must_cmd(qts, 52, IO_WRITE | (CCCR_IOE << 9) | 0x02, R5);
}

static uint8_t cmd52_read(QTestState *qts, unsigned fn, uint32_t addr)
{
    uint32_t r = must_cmd(qts, 52, (fn << 28) | (addr << 9), R5);

    g_assert_cmphex(R5_FLAGS(r) & 0xcb, ==, 0);
    return R5_DATA(r);
}

static uint16_t cmd52_write(QTestState *qts, unsigned fn, uint32_t addr,
                            uint8_t v)
{
    return cmd(qts, 52, IO_WRITE | IO_RAW | (fn << 28) | (addr << 9) | v, R5,
               NULL);
}

/* Both ends of the bus 4 bits wide */
static void bus_4bit(QTestState *qts)
{
    must_cmd(qts, 52, IO_WRITE | (CCCR_BUS_IF << 9) | 0x02, R5);
    qtest_writeb(qts, HOST + SDHC_HOSTCTL, CTRL_4BITBUS);
}

/* The host's DAT line back from an error: Software Reset for DAT Line */
static void reset_dat(QTestState *qts)
{
    qtest_writeb(qts, HOST + SDHC_SWRST, RESET_DATA);
}

/*
 * A byte mode CMD53 of len bytes (a multiple of 4), with programmed I/O;
 * returns the data errors the host saw.
 */
static uint16_t cmd53(QTestState *qts, bool write, uint32_t addr,
                      uint8_t *buf, uint32_t len)
{
    uint16_t nis, err;

    qtest_writew(qts, HOST + SDHC_BLKSIZE, len);
    qtest_writew(qts, HOST + SDHC_BLKCNT, 1);
    qtest_writew(qts, HOST + SDHC_TRNMOD, write ? 0 : TRNS_READ);
    /* A read's data comes with the command, and with it its errors. */
    g_assert_cmphex(cmd(qts, 53, (write ? IO_WRITE : 0) | (1u << 28) |
                        IO_OP_INC | (addr << 9) | len, R5 | CMD_DATA, NULL) &
                    (EIS_CMDTIMEOUT | EIS_CMDCRC | EIS_CMDEND | EIS_CMDIDX),
                    ==, 0);
    for (uint32_t i = 0; i < len; i += 4) {
        if (write) {
            qtest_writel(qts, HOST + SDHC_BDATA, ldl_le_p(buf + i));
        } else {
            nis = qtest_readw(qts, HOST + SDHC_NORINTSTS);
            if (!(nis & NIS_RBUFRDY)) {
                break;
            }
            stl_le_p(buf + i, qtest_readl(qts, HOST + SDHC_BDATA));
        }
    }
    nis = qtest_readw(qts, HOST + SDHC_NORINTSTS);
    err = qtest_readw(qts, HOST + SDHC_ERRINTSTS);
    if (err) {
        g_assert_true(nis & NIS_ERR);
        g_assert_false(nis & NIS_TRSCMP);
        reset_dat(qts);
    } else {
        g_assert_true(nis & NIS_TRSCMP);
    }
    return err;
}

static uint32_t prnsts(QTestState *qts)
{
    return qtest_readl(qts, HOST + SDHC_PRNSTS);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * From reset slot 1's pads are on function 0 (MTMS, MTDO, GPIO2, ...):
 * no command reaches the slave, and the host, its CMD pulled up, sees
 * none answered. Set up as ESP-IDF sets them up, the slave answers. The
 * machine takes no SD card on the pads its host has.
 */
static void test_reset_and_setup(void)
{
    QTestState *qts = start(BOARD_PULL_UP);
    QDict *rsp;

    g_assert_cmphex(cmd(qts, 5, 0, CMD_RSP48, NULL), ==, EIS_CMDTIMEOUT);
    g_assert_cmphex(prnsts(qts) & (PRNSTS_CMD | 0xf00000), ==,
                    PRNSTS_CMD | 0xf00000);
    sdio_pins(qts, false);
    card_init(qts);
    g_assert_cmphex(cmd52_read(qts, 0, CCCR_IOE), ==, 0x02);

    rsp = qtest_qmp(qts, "{ 'execute': 'device_add', 'arguments': "
                    "{ 'driver': 'sd-card', 'bus': 'sd-bus1' } }");
    g_assert_true(qdict_haskey(rsp, "error"));
    g_assert_nonnull(strstr(qdict_get_str(qdict_get_qdict(rsp, "error"),
                                          "desc"), "full"));
    qobject_unref(rsp);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * The bare module has no pull-ups: with ESP-IDF's internal ones
 * (SDIO_SLAVE_FLAG_INTERNAL_PULLUP) the slave works; without them CMD
 * keeps the pull-up GPIO15 has from reset but DAT0 to DAT3 float and read
 * low, DAT1 among them, which the host takes for the slave's interrupt.
 */
static void test_pulls(void)
{
    QTestState *qts = start("");

    sdio_pins(qts, true);
    card_init(qts);
    g_assert_cmphex(prnsts(qts) & 0xf00000, ==, 0xf00000);
    g_assert_false(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CARDINT);
    qtest_quit(qts);

    qts = start("");
    sdio_pins(qts, false);
    card_init(qts);
    g_assert_cmphex(prnsts(qts) & (PRNSTS_CMD | 0xf00000), ==, PRNSTS_CMD);
    g_assert_true(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CARDINT);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * CLK reaches the slave only on function 4 with its input enabled: on the
 * GPIO function, with FUN_IE clear, or handed to the RTC IO MUX with its
 * input off, the slave sees no clock and answers nothing.
 */
static void test_clk(void)
{
    QTestState *qts = start(BOARD_PULL_UP);

    sdio_pins(qts, false);
    card_init(qts);

    func_sel(qts, SD_PIN_CLK, SDP_FUNC_GPIO);
    g_assert_cmphex(cmd(qts, 52, 0, R5, NULL), ==, EIS_CMDTIMEOUT);
    func_sel(qts, SD_PIN_CLK, SDP_FUNC_SDIO);
    cmd52_read(qts, 0, CCCR_IOE);

    mux_bits(qts, SD_PIN_CLK, 0, SDP_FUN_IE);
    g_assert_cmphex(cmd(qts, 52, 0, R5, NULL), ==, EIS_CMDTIMEOUT);
    mux_bits(qts, SD_PIN_CLK, SDP_FUN_IE, 0);
    cmd52_read(qts, 0, CCCR_IOE);

    sd_pin_rtc_bits(qts, SD_PIN_CLK, SDP_RTC_MUX_SEL, 0);
    g_assert_cmphex(cmd(qts, 52, 0, R5, NULL), ==, EIS_CMDTIMEOUT);
    sd_pin_rtc_bits(qts, SD_PIN_CLK, 0, SDP_RTC_MUX_SEL);
    cmd52_read(qts, 0, CCCR_IOE);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * CMD: off function 4 the slave's input reads 1 and its output does not
 * reach the pad; with its input disabled it reads 0. Either way the host,
 * its CMD pulled up, times out. With the output enable taken from
 * GPIO_ENABLE (OEN_SEL) and that clear, or the pad handed to the RTC IO
 * MUX with its input on, the slave hears the command and acts on it, but
 * its response does not reach the host; with GPIO_ENABLE set the pad
 * drives the slave's idle level against the host's command, which never
 * reaches the slave. Open drain, the response's highs pass only as the
 * pulls hold them: with the board's pull-ups the slave answers, with none
 * the host reads a response of 0s, its CRC, end bit and index wrong.
 */
static void test_cmd(void)
{
    QTestState *qts = start(BOARD_PULL_UP);
    uint32_t out = SDP_FUNC_OUT(sd_pin[SD_PIN_CMD].pad);

    sdio_pins(qts, false);
    card_init(qts);

    func_sel(qts, SD_PIN_CMD, SDP_FUNC_GPIO);
    g_assert_cmphex(cmd(qts, 52, 0, R5, NULL), ==, EIS_CMDTIMEOUT);
    func_sel(qts, SD_PIN_CMD, SDP_FUNC_SDIO);

    mux_bits(qts, SD_PIN_CMD, 0, SDP_FUN_IE);
    g_assert_cmphex(cmd(qts, 52, 0, R5, NULL), ==, EIS_CMDTIMEOUT);
    mux_bits(qts, SD_PIN_CMD, SDP_FUN_IE, 0);

    /* The slave takes a write to IEN; only its response is lost. */
    qtest_writel(qts, out, qtest_readl(qts, out) | SDP_OUT_OEN_SEL);
    g_assert_cmphex(cmd52_write(qts, 0, CCCR_IEN, 0x03), ==, EIS_CMDTIMEOUT);
    qtest_writel(qts, SDP_ENABLE_W1TS, 1u << sd_pin[SD_PIN_CMD].pad);
    g_assert_cmphex(cmd52_write(qts, 0, CCCR_IEN, 0x00), ==, EIS_CMDTIMEOUT);
    qtest_writel(qts, SDP_ENABLE_W1TC, 1u << sd_pin[SD_PIN_CMD].pad);
    qtest_writel(qts, out, qtest_readl(qts, out) & ~SDP_OUT_OEN_SEL);
    g_assert_cmphex(cmd52_read(qts, 0, CCCR_IEN), ==, 0x03);

    sd_pin_rtc_bits(qts, SD_PIN_CMD, SDP_RTC_MUX_SEL | (1u << 13), 0);
    g_assert_cmphex(cmd52_write(qts, 0, CCCR_IEN, 0x01), ==, EIS_CMDTIMEOUT);
    sd_pin_rtc_bits(qts, SD_PIN_CMD, 0, SDP_RTC_MUX_SEL | (1u << 13));
    g_assert_cmphex(cmd52_read(qts, 0, CCCR_IEN), ==, 0x01);

    qtest_writel(qts, SDP_PIN(sd_pin[SD_PIN_CMD].pad), SDP_PIN_PAD_DRIVER);
    g_assert_cmphex(cmd52_read(qts, 0, CCCR_IEN), ==, 0x01);
    qtest_quit(qts);

    qts = start("");
    sdio_pins(qts, true);
    card_init(qts);
    qtest_writel(qts, SDP_PIN(sd_pin[SD_PIN_CMD].pad), SDP_PIN_PAD_DRIVER);
    sd_pin_rtc_bits(qts, SD_PIN_CMD, 0, SDP_RTC_RUE);
    g_assert_cmphex(cmd(qts, 52, 0, R5, NULL), ==,
                    EIS_CMDCRC | EIS_CMDEND | EIS_CMDIDX);
    /* Without a response check, only the end bit is wrong. */
    g_assert_cmphex(cmd(qts, 52, 0, CMD_RSP48, NULL), ==, EIS_CMDEND);
    sd_pin_rtc_bits(qts, SD_PIN_CMD, SDP_RTC_RUE, 0);
    cmd52_read(qts, 0, CCCR_IEN);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * A CMD53 read takes the slave's DAT lines only where they follow it: off
 * function 4, DAT0 shows the host no start bit (a data timeout); DAT2, in
 * a 4-bit read, its bits read 1 as the pull-up holds them (a CRC error);
 * pulled down, 0 (a CRC and an end bit error). Each stops the host's
 * transfer until its DAT line is reset.
 */
static void test_dat_read(void)
{
    QTestState *qts = start(BOARD_PULL_UP);
    uint8_t buf[4];

    sdio_pins(qts, false);
    card_init(qts);
    qtest_writel(qts, SLCHOST + HOST_CONF_W0, 0x12345678);

    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==, 0);
    g_assert_cmphex(ldl_le_p(buf), ==, 0x12345678);

    func_sel(qts, SD_PIN_D0, SDP_FUNC_GPIO);
    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==,
                    EIS_DATATIMEOUT);
    func_sel(qts, SD_PIN_D0, SDP_FUNC_SDIO);
    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==, 0);

    bus_4bit(qts);
    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==, 0);
    g_assert_cmphex(ldl_le_p(buf), ==, 0x12345678);
    func_sel(qts, SD_PIN_D2, SDP_FUNC_GPIO);
    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==,
                    EIS_DATACRC);
    qtest_set_irq_in(qts, "/machine/soc/gpio", "esp32-gpio-board-pull-up",
                     sd_pin[SD_PIN_D2].pad, 0);
    sd_pin_rtc_bits(qts, SD_PIN_D2, SDP_RTC_RDE, 0);
    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==,
                    EIS_DATACRC | EIS_DATAEND);
    func_sel(qts, SD_PIN_D2, SDP_FUNC_SDIO);
    g_assert_cmphex(cmd53(qts, false, HOST_CONF_W0, buf, 4), ==, 0);
    g_assert_cmphex(ldl_le_p(buf), ==, 0x12345678);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * A 4-bit CMD53 write block with DAT3 off function 4 reaches the slave
 * with DAT3's bits read 1 (SD_DATA3 is an I1 input on GPIO13): its CRC
 * check fails; the slave counts it on DAT3, discards the block and sends
 * a negative CRC status, a CRC error at the host. ERR_CNT_CLR clears the
 * counts. With DAT0 off function the host's start bit never reaches the
 * slave, which sends no CRC status: the host times out.
 */
static void test_dat_write(void)
{
    QTestState *qts = start(BOARD_PULL_UP);
    const uint32_t d0 = RAM, buf0 = RAM + 0x100;
    uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 }, got[8];

    sdio_pins(qts, false);
    card_init(qts);
    bus_4bit(qts);
    qtest_writel(qts, HINF_CFG_DATA1, CFG1_IOREADY1);
    qtest_writel(qts, d0, DESC_OWNER | 64);
    qtest_writel(qts, d0 + 4, buf0);
    qtest_writel(qts, d0 + 8, 0);
    qtest_writel(qts, SLC_CONF0, 0);
    qtest_writel(qts, SLC_0TX_LINK, LINK_START | (d0 & 0xfffff));
    qtest_writel(qts, SLC_0TOKEN1, TOKEN_INC_MORE | 1);

    func_sel(qts, SD_PIN_D3, SDP_FUNC_GPIO);
    g_assert_cmphex(cmd53(qts, true, DATA_END - 8, data, 8), ==,
                    EIS_DATACRC);
    g_assert_cmphex(qtest_readl(qts, SLC_SDIO_CRC_ST0), ==, 0x01000000);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_TX_DONE, ==, 0);

    func_sel(qts, SD_PIN_D0, SDP_FUNC_GPIO);
    func_sel(qts, SD_PIN_D3, SDP_FUNC_SDIO);
    g_assert_cmphex(cmd53(qts, true, DATA_END - 8, data, 8), ==,
                    EIS_DATATIMEOUT);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_TX_DONE, ==, 0);
    func_sel(qts, SD_PIN_D0, SDP_FUNC_SDIO);
    /* The slave still waits for the block it never took. */
    must_cmd(qts, 52, IO_WRITE | (0x06 << 9) | 0x01, R5);

    qtest_writel(qts, SLC_SDIO_CRC_ST1, CRC_ST1_ERR_CNT_CLR);
    g_assert_cmphex(qtest_readl(qts, SLC_SDIO_CRC_ST0), ==, 0);
    qtest_writel(qts, SLC_SDIO_CRC_ST1, 0);
    g_assert_cmphex(cmd53(qts, true, DATA_END - 8, data, 8), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_TX_DONE, ==,
                    SLCINT_TX_DONE);
    qtest_memread(qts, buf0, got, 8);
    g_assert_cmpmem(got, 8, data, 8);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * [spec:nuos:req:emu.esp32.gpio/test]
 * The slave interrupts its host by holding DAT1 low, on its pad: with
 * DAT1 off function 4 the host, its DAT1 pulled up, sees no interrupt;
 * back on it, the interrupt.
 */
static void test_dat1_interrupt(void)
{
    QTestState *qts = start(BOARD_PULL_UP);

    sdio_pins(qts, false);
    card_init(qts);
    qtest_writel(qts, SLCHOST + HOST_FUNC1_INT_ENA, 0x01);
    must_cmd(qts, 52, IO_WRITE | (CCCR_IEN << 9) | 0x03, R5);

    func_sel(qts, SD_PIN_D1, SDP_FUNC_GPIO);
    qtest_writel(qts, SLC_INTVEC_TOHOST, 0x01);
    g_assert_false(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CARDINT);
    g_assert_cmphex(prnsts(qts) & PRNSTS_DAT(1), ==, PRNSTS_DAT(1));

    func_sel(qts, SD_PIN_D1, SDP_FUNC_SDIO);
    g_assert_true(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CARDINT);
    g_assert_cmphex(prnsts(qts) & PRNSTS_DAT(1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SDP_GPIO + 0x3c) &
                    (1u << sd_pin[SD_PIN_D1].pad), ==, 0);

    cmd52_write(qts, 1, HOST_INT_CLR, 0x01);
    g_assert_false(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CARDINT);
    g_assert_cmphex(prnsts(qts) & PRNSTS_DAT(1), ==, PRNSTS_DAT(1));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave/test]
 * The board has the slot 1 pads wired to the SDIO host or to an SD card,
 * not both: the machine refuses -drive if=sd with sdio-host=on.
 */
static void test_no_sd_card(void)
{
    const char *qemu = g_getenv("QTEST_QEMU_BINARY");
    g_autofree char *err = NULL;
    const char *argv[] = {
        qemu, "-M", "esp32,sdio-host=on", "-nic", "none", "-display", "none",
        "-S", "-drive", "if=sd,driver=null-co,size=1048576", NULL,
    };
    int status;

    g_assert_true(g_spawn_sync(NULL, (char **)argv, NULL,
                               G_SPAWN_STDOUT_TO_DEV_NULL, NULL, NULL, NULL,
                               &err, &status, NULL));
    g_assert_cmpint(status, !=, 0);
    g_assert_nonnull(strstr(err, "sdio-host=on"));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/sdio-pins/reset-and-setup", test_reset_and_setup);
    qtest_add_func("/esp32/sdio-pins/pulls", test_pulls);
    qtest_add_func("/esp32/sdio-pins/clk", test_clk);
    qtest_add_func("/esp32/sdio-pins/cmd", test_cmd);
    qtest_add_func("/esp32/sdio-pins/dat-read", test_dat_read);
    qtest_add_func("/esp32/sdio-pins/dat-write", test_dat_write);
    qtest_add_func("/esp32/sdio-pins/dat1-interrupt", test_dat1_interrupt);
    qtest_add_func("/esp32/sdio-pins/no-sd-card", test_no_sd_card);
    return g_test_run();
}
