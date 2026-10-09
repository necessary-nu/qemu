/*
 * QTest testcase for the ESP32 SDIO slave, driven from an SD host
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The machine's sdio-host option puts an SDHCI at 0x22000000 with the
 * slave's card on its bus; the test is the host's driver, issuing CMD52
 * and CMD53 through it with programmed I/O, and the slave's software,
 * through the slave's registers.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define DPORT_CORE_RST_EN       0x3ff000d0
#define CORE_RST_SDIO           (1u << 5)

#define SLC                     0x3ff58000
#define SLC_CONF0               (SLC + 0x00)
#define SLC_0INT_RAW            (SLC + 0x04)
#define SLC_0INT_ST             (SLC + 0x08)
#define SLC_0INT_ENA            (SLC + 0x0c)
#define SLC_0INT_CLR            (SLC + 0x10)
#define SLC_RX_STATUS           (SLC + 0x24)
#define SLC_0RX_LINK            (SLC + 0x3c)
#define SLC_0TX_LINK            (SLC + 0x40)
#define SLC_INTVEC_TOHOST       (SLC + 0x4c)
#define SLC_0TOKEN1             (SLC + 0x54)
#define SLC_CONF1               (SLC + 0x60)
#define SLC_0TO_EOF_DES_ADDR    (SLC + 0x78)
#define SLC_0TX_EOF_DES_ADDR    (SLC + 0x7c)
#define SLC_0TXLINK_DSCR        (SLC + 0x9c)
#define SLC_0RXLINK_DSCR        (SLC + 0xa8)
#define SLC_0_LEN_CONF          (SLC + 0xe4)
#define SLC_0_LENGTH            (SLC + 0xe8)

#define CONF0_RX_RST            (1u << 1)
#define CONF0_RX_AUTO_WRBACK    (1u << 6)
#define SLCINT_RX_START         (1u << 8)
#define SLCINT_TX_START         (1u << 9)
#define SLCINT_RX_UDF           (1u << 10)
#define SLCINT_TX_OVF           (1u << 11)
#define SLCINT_TX_DONE          (1u << 14)
#define SLCINT_TX_SUC_EOF       (1u << 15)
#define SLCINT_RX_DONE          (1u << 16)
#define SLCINT_RX_EOF           (1u << 17)
#define SLCINT_RX_DSCR_ERR      (1u << 20)
#define LINK_START              (1u << 29)
#define LINK_RESTART            (1u << 30)
#define LINK_PARK               (1u << 31)
#define TOKEN_WR                (1u << 12)
#define TOKEN_INC_MORE          (1u << 14)
#define LEN_WR                  (1u << 20)

#define SLCHOST                 0x3ff55000
#define HOST_PKT_LEN            0x60
#define HOST_TOKEN_RDATA        0x44
#define HOST_INT_RAW            0x50
#define HOST_INT_ST             0x58
#define HOST_CONF_W0            0x6c
#define HOST_CONF_W7            0x8c
#define HOST_INT_CLR            0xd4
#define HOST_FUNC1_INT_ENA      0xdc
#define HOSTINT_RX_UDF          (1u << 16)
#define HOSTINT_TX_OVF          (1u << 17)
#define HOSTINT_RX_NEW_PACKET   (1u << 23)

#define HINF_CFG_DATA0          0x3ff4b000
#define HINF_CFG_DATA1          0x3ff4b004
#define HINF_CIS_CONF0          0x3ff4b020
#define CFG1_IOREADY1           (1u << 1)
#define CFG1_HIGHSPEED_ENABLE   (1u << 2)
#define CFG1_HIGHSPEED_MODE     (1u << 3)
#define CFG1_IOENABLE1          (1u << 11)
#define CFG1_SDIO_VER_232       (0x232u << 16)

#define DESC_OWNER              (1u << 31)
#define DESC_EOF                (1u << 30)

/* The SDHCI standing for the slave's host */
#define HOST                    0x22000000
#define SDHC_BLKSIZE            0x04
#define SDHC_BLKCNT             0x06
#define SDHC_ARGUMENT           0x08
#define SDHC_TRNMOD             0x0c
#define SDHC_CMDREG             0x0e
#define SDHC_RSPREG0            0x10
#define SDHC_BDATA              0x20
#define SDHC_PRNSTS             0x24
#define SDHC_CLKCON             0x2c
#define SDHC_SWRST              0x2f
#define SDHC_NORINTSTS          0x30
#define SDHC_ERRINTSTS          0x32
#define SDHC_NORINTSTSEN        0x34
#define SDHC_ERRINTSTSEN        0x36
#define SDHC_NORINTSIGEN        0x38
#define TRNS_BLK_CNT_EN         0x02
#define TRNS_READ               0x10
#define TRNS_MULTI              0x20
#define CMD_RSP48               0x02
#define CMD_DATA                0x20
#define NIS_CMDCMP              0x0001
#define NIS_TRSCMP              0x0002
#define NIS_CARDINT             0x0100
#define EIS_CMDTIMEOUT          0x0001
#define PRNSTS_DAT1             (1u << 21)

#define R5_FLAGS(r)             (((r) >> 8) & 0xff)
#define R5_DATA(r)              ((r) & 0xff)
#define IO_WRITE                (1u << 31)
#define IO_BLOCK                (1u << 27)
#define IO_RAW                  (1u << 27)
#define IO_OP_INC               (1u << 26)
#define DATA_END                0x1f800

/* DMA-capable SRAM for descriptors and buffers */
#define RAM                     0x3ffc0000

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32,sdio-host=on -nic none");

    qtest_writeb(qts, HOST + SDHC_SWRST, 1);
    qtest_writew(qts, HOST + SDHC_CLKCON, 0x0005);
    qtest_writew(qts, HOST + SDHC_NORINTSTSEN, 0xffff);
    qtest_writew(qts, HOST + SDHC_ERRINTSTSEN, 0xffff);
    return qts;
}

/* Issue a command; false if the card does not answer. */
static bool cmd(QTestState *qts, uint8_t idx, uint32_t arg, uint16_t flags,
                uint32_t *resp)
{
    qtest_writew(qts, HOST + SDHC_NORINTSTS, 0xffff);
    qtest_writew(qts, HOST + SDHC_ERRINTSTS, 0xffff);
    qtest_writel(qts, HOST + SDHC_ARGUMENT, arg);
    qtest_writew(qts, HOST + SDHC_CMDREG, (idx << 8) | flags);
    g_assert_true(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CMDCMP);
    if (qtest_readw(qts, HOST + SDHC_ERRINTSTS) & EIS_CMDTIMEOUT) {
        return false;
    }
    *resp = qtest_readl(qts, HOST + SDHC_RSPREG0);
    return true;
}

static uint32_t must_cmd(QTestState *qts, uint8_t idx, uint32_t arg,
                         uint16_t flags)
{
    uint32_t r = 0;

    g_assert_true(cmd(qts, idx, arg, flags, &r));
    return r;
}

/* Bring the card up: CMD5, CMD3 and CMD7. */
static void card_init(QTestState *qts)
{
    uint32_t r;

    r = must_cmd(qts, 5, 0, CMD_RSP48);
    g_assert_cmphex(r, ==, 0xa0ff8000);
    must_cmd(qts, 5, 0x00200000, CMD_RSP48);
    r = must_cmd(qts, 3, 0, CMD_RSP48);
    g_assert_cmphex(r >> 16, ==, 1);
    must_cmd(qts, 7, 1u << 16, CMD_RSP48);
}

static uint8_t cmd52_read(QTestState *qts, unsigned fn, uint32_t addr)
{
    uint32_t r = must_cmd(qts, 52, (fn << 28) | (addr << 9), CMD_RSP48);

    g_assert_cmphex(R5_FLAGS(r) & 0xcb, ==, 0);
    return R5_DATA(r);
}

static uint8_t cmd52_write(QTestState *qts, unsigned fn, uint32_t addr,
                           uint8_t v)
{
    uint32_t r = must_cmd(qts, 52, IO_WRITE | IO_RAW | (fn << 28) |
                          (addr << 9) | v, CMD_RSP48);

    g_assert_cmphex(R5_FLAGS(r) & 0xcb, ==, 0);
    return R5_DATA(r);
}

/*
 * CMD53 with programmed I/O: len bytes, a multiple of 4, in byte mode, or
 * in blocks of 512 if block.
 */
static void cmd53(QTestState *qts, bool write, unsigned fn, uint32_t addr,
                  uint8_t *buf, uint32_t len, bool block)
{
    uint32_t count = block ? len / 512 : len % 512;
    uint32_t r;

    if (block) {
        qtest_writew(qts, HOST + SDHC_BLKSIZE, 512);
        qtest_writew(qts, HOST + SDHC_BLKCNT, len / 512);
        qtest_writew(qts, HOST + SDHC_TRNMOD, TRNS_BLK_CNT_EN | TRNS_MULTI |
                     (write ? 0 : TRNS_READ));
    } else {
        qtest_writew(qts, HOST + SDHC_BLKSIZE, len);
        qtest_writew(qts, HOST + SDHC_BLKCNT, 1);
        qtest_writew(qts, HOST + SDHC_TRNMOD, write ? 0 : TRNS_READ);
    }
    r = must_cmd(qts, 53, (write ? IO_WRITE : 0) | (fn << 28) |
                 (block ? IO_BLOCK : 0) | IO_OP_INC | (addr << 9) | count,
                 CMD_RSP48 | CMD_DATA);
    g_assert_cmphex(R5_FLAGS(r) & 0xcb, ==, 0);
    for (uint32_t i = 0; i < len; i += 4) {
        if (write) {
            qtest_writel(qts, HOST + SDHC_BDATA, ldl_le_p(buf + i));
        } else {
            stl_le_p(buf + i, qtest_readl(qts, HOST + SDHC_BDATA));
        }
    }
    g_assert_true(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_TRSCMP);
}

/* Read a 32-bit SLCHOST register through function 1. */
static uint32_t host_reg(QTestState *qts, uint32_t off)
{
    uint8_t b[4];

    cmd53(qts, false, 1, off, b, 4, false);
    return ldl_le_p(b);
}

static void put_desc(QTestState *qts, uint32_t at, uint32_t dw0,
                     uint32_t buf, uint32_t next)
{
    qtest_writel(qts, at, dw0);
    qtest_writel(qts, at + 4, buf);
    qtest_writel(qts, at + 8, next);
}

/* [spec:nuos:req:emu.esp32.sdio-slave/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, SLC_CONF0), ==, 0x00004030);
    g_assert_cmphex(qtest_readl(qts, SLC_CONF1), ==, 0x00000078);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SLC_RX_STATUS), ==, 0x00020002);
    g_assert_cmphex(qtest_readl(qts, SLC_0RX_LINK), ==, 0);
    g_assert_cmphex(qtest_readl(qts, HINF_CFG_DATA0), ==, 0x22226666);
    g_assert_cmphex(qtest_readl(qts, HINF_CFG_DATA1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SLCHOST + HOST_INT_RAW), ==, 0);

    /* Access types: RO status, WO clear and vector, write masks */
    qtest_writel(qts, SLC_0INT_RAW, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==, 0);
    qtest_writel(qts, SLC_0INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_ENA), ==, 0x07ffffff);
    qtest_writel(qts, SLC_CONF1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SLC_CONF1), ==, 0x007f007f);
    qtest_writel(qts, SLC_INTVEC_TOHOST, 0x01);
    g_assert_cmphex(qtest_readl(qts, SLC_INTVEC_TOHOST), ==, 0);
    qtest_writel(qts, SLCHOST + HOST_CONF_W0, 0x12345678);
    g_assert_cmphex(qtest_readb(qts, SLCHOST + HOST_CONF_W0 + 2), ==, 0x34);
    qtest_writeb(qts, SLCHOST + HOST_CONF_W0 + 1, 0xab);
    g_assert_cmphex(qtest_readl(qts, SLCHOST + HOST_CONF_W0), ==,
                    0x1234ab78);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.sdio-slave/test] */
static void test_cccr(void)
{
    QTestState *qts = start();
    uint32_t r;

    card_init(qts);

    /* CCCR: the slave software sets the SDIO version it reports. */
    g_assert_cmphex(cmd52_read(qts, 0, 0x00), ==, 0x00);
    qtest_writel(qts, HINF_CFG_DATA1, CFG1_SDIO_VER_232 |
                 CFG1_HIGHSPEED_ENABLE);
    g_assert_cmphex(cmd52_read(qts, 0, 0x00), ==, 0x32);
    g_assert_cmphex(cmd52_read(qts, 0, 0x01), ==, 0x02);
    g_assert_cmphex(cmd52_read(qts, 0, 0x08), ==, 0x13);
    g_assert_cmphex(cmd52_read(qts, 0, 0x13), ==, 0x01);
    g_assert_cmphex(cmd52_write(qts, 0, 0x13, 0x02), ==, 0x03);
    g_assert_cmphex(qtest_readl(qts, HINF_CFG_DATA1) & CFG1_HIGHSPEED_MODE,
                    ==, CFG1_HIGHSPEED_MODE);

    /* The common CIS: pointer, manufacturer ID, function ID */
    g_assert_cmphex(cmd52_read(qts, 0, 0x09), ==, 0x00);
    g_assert_cmphex(cmd52_read(qts, 0, 0x0a), ==, 0x10);
    g_assert_cmphex(cmd52_read(qts, 0, 0x0b), ==, 0x00);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1000), ==, 0x20);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1001), ==, 4);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1002), ==, 0x66);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1003), ==, 0x66);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1004), ==, 0x22);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1006), ==, 0x21);
    /* HINF_CIS_CONF's bytes follow function 0's extension tuple. */
    qtest_writel(qts, HINF_CIS_CONF0, 0x00cd0180);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1010), ==, 0x80);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1012), ==, 0xcd);
    g_assert_cmphex(cmd52_read(qts, 0, 0x1030), ==, 0xff);
    /* Function 1's FBR points at its CIS, which has its IDs. */
    g_assert_cmphex(cmd52_read(qts, 0, 0x10a), ==, 0x20);
    g_assert_cmphex(cmd52_read(qts, 0, 0x2004), ==, 0x22);
    g_assert_cmphex(cmd52_read(qts, 0, 0x20a), ==, 0x30);
    g_assert_cmphex(cmd52_read(qts, 0, 0x3004), ==, 0x33);

    /* Function 1 is ready once enabled and the slave says it is. */
    g_assert_cmphex(cmd52_write(qts, 0, 0x02, 0x02), ==, 0x02);
    g_assert_cmphex(cmd52_read(qts, 0, 0x03), ==, 0x00);
    g_assert_cmphex(qtest_readl(qts, HINF_CFG_DATA1) & CFG1_IOENABLE1, ==,
                    CFG1_IOENABLE1);
    qtest_writel(qts, HINF_CFG_DATA1, CFG1_SDIO_VER_232 | CFG1_IOREADY1 |
                 CFG1_HIGHSPEED_ENABLE);
    g_assert_cmphex(cmd52_read(qts, 0, 0x03), ==, 0x02);

    /* Block sizes: function 0's in the CCCR, function 1's in its FBR */
    cmd52_write(qts, 0, 0x110, 0x00);
    g_assert_cmphex(cmd52_write(qts, 0, 0x111, 0x02), ==, 0x02);
    g_assert_cmphex(cmd52_read(qts, 0, 0x110), ==, 0x00);
    cmd52_write(qts, 0, 0x10, 0x40);
    g_assert_cmphex(cmd52_read(qts, 0, 0x10), ==, 0x40);

    /* A function the card does not have */
    r = must_cmd(qts, 52, (3u << 28), CMD_RSP48);
    g_assert_cmphex(R5_FLAGS(r) & 0x02, ==, 0x02);

    /* Held in reset, the slave does not answer. */
    qtest_writel(qts, DPORT_CORE_RST_EN, CORE_RST_SDIO);
    g_assert_false(cmd(qts, 52, 0, CMD_RSP48, &r));
    qtest_writel(qts, DPORT_CORE_RST_EN, 0);
    /* Its reset took the card back to its initialization. */
    g_assert_false(cmd(qts, 52, 0, CMD_RSP48, &r));
    card_init(qts);
    g_assert_cmphex(cmd52_read(qts, 0, 0x02), ==, 0x00);

    /* The I/O reset (RES) likewise. */
    cmd52_write(qts, 0, 0x02, 0x02);
    must_cmd(qts, 52, IO_WRITE | (0x06 << 9) | 0x08, CMD_RSP48);
    g_assert_false(cmd(qts, 52, 0, CMD_RSP48, &r));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.sdio-slave/test] */
static void test_registers_and_interrupts(void)
{
    QTestState *qts = start();
    uint32_t r;

    card_init(qts);
    cmd52_write(qts, 0, 0x02, 0x02);

    /* The CONF_W registers are shared both ways. */
    cmd52_write(qts, 1, HOST_CONF_W0 + 3, 0x5a);
    g_assert_cmphex(qtest_readl(qts, SLCHOST + HOST_CONF_W0), ==,
                    0x5a000000);
    qtest_writel(qts, SLCHOST + HOST_CONF_W0 + 4, 0xc0de0042);
    g_assert_cmphex(host_reg(qts, HOST_CONF_W0 + 4), ==, 0xc0de0042);

    /* Slave to host: the vector raises the TOHOST bits. */
    qtest_irq_intercept_out_named(qts, "/machine/sdio-sdhci", "sysbus-irq");
    qtest_writew(qts, HOST + SDHC_NORINTSIGEN, NIS_CARDINT);
    qtest_writel(qts, SLCHOST + HOST_FUNC1_INT_ENA, 0x08);
    qtest_writel(qts, SLC_INTVEC_TOHOST, 0x0c);
    g_assert_cmphex(host_reg(qts, HOST_INT_RAW), ==, 0x0c);
    g_assert_cmphex(host_reg(qts, HOST_INT_ST), ==, 0x08);
    g_assert_cmphex(cmd52_read(qts, 0, 0x05), ==, 0x02);
    /* The card interrupts the host only as IENM and IEN1 allow. */
    g_assert_false(qtest_get_irq(qts, 0));
    cmd52_write(qts, 0, 0x04, 0x03);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_true(qtest_readw(qts, HOST + SDHC_NORINTSTS) & NIS_CARDINT);
    g_assert_cmphex(qtest_readl(qts, HOST + SDHC_PRNSTS) & PRNSTS_DAT1, ==,
                    0);
    /* The host clears it through SLCHOST. */
    cmd52_write(qts, 1, HOST_INT_CLR, 0x08);
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmphex(host_reg(qts, HOST_INT_RAW), ==, 0x04);
    qtest_writel(qts, SLCHOST + HOST_INT_CLR, 0x04);
    g_assert_cmphex(qtest_readl(qts, SLCHOST + HOST_INT_RAW), ==, 0);

    /* Host to slave: CONF_W7's byte 0 is the vector to SLC0. */
    qtest_writel(qts, SLC_0INT_ENA, 0x20);
    cmd52_write(qts, 1, HOST_CONF_W7, 0x21);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==, 0x21);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_ST), ==, 0x20);
    g_assert_cmphex(cmd52_read(qts, 1, HOST_CONF_W7), ==, 0);
    qtest_writel(qts, SLC_0INT_CLR, 0x21);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==, 0);

    /* The receive buffer count, which the host reads from TOKEN_RDATA */
    qtest_writel(qts, SLC_0TOKEN1, TOKEN_WR | 5);
    qtest_writel(qts, SLC_0TOKEN1, TOKEN_INC_MORE | 3);
    g_assert_cmphex(qtest_readl(qts, SLC_0TOKEN1), ==, 8u << 16);
    g_assert_cmphex(host_reg(qts, HOST_TOKEN_RDATA), ==, 8u << 16);

    /* A read-only register ignores the host's writes. */
    r = must_cmd(qts, 52, IO_WRITE | IO_RAW | (1u << 28) |
                 (HOST_TOKEN_RDATA + 2) << 9 | 0x77, CMD_RSP48);
    g_assert_cmphex(R5_DATA(r), ==, 8);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.sdio-slave/test] */
static void test_send_packet(void)
{
    QTestState *qts = start();
    const uint32_t d0 = RAM, d1 = RAM + 0x10, buf0 = RAM + 0x100;
    const uint32_t buf1 = RAM + 0x400;
    uint8_t pattern[600], got[600];
    uint32_t len = 524;

    for (int i = 0; i < sizeof(pattern); i++) {
        pattern[i] = i * 7 + 1;
    }
    qtest_memwrite(qts, buf0, pattern, 300);
    qtest_memwrite(qts, buf1, pattern + 300, 300);
    card_init(qts);
    cmd52_write(qts, 0, 0x02, 0x02);
    cmd52_write(qts, 0, 0x110, 0x00);
    cmd52_write(qts, 0, 0x111, 0x02);

    /* The slave queues a 524-byte packet in two buffers. */
    put_desc(qts, d0, DESC_OWNER | (300 << 12) | 300, buf0, d1);
    put_desc(qts, d1, DESC_OWNER | DESC_EOF | (224 << 12) | 300, buf1, 0);
    qtest_writel(qts, SLC_CONF0, CONF0_RX_AUTO_WRBACK | CONF0_RX_RST);
    qtest_writel(qts, SLC_CONF0, CONF0_RX_AUTO_WRBACK);
    qtest_writel(qts, SLC_0_LEN_CONF, LEN_WR | len);
    qtest_writel(qts, SLC_0RX_LINK, LINK_START | (d0 & 0xfffff));
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==, SLCINT_RX_START);
    g_assert_cmphex(qtest_readl(qts, SLC_0RXLINK_DSCR), ==, d0);
    g_assert_cmphex(qtest_readl(qts, SLC_RX_STATUS) & 1, ==, 1);

    /* The host learns of it and reads its length. */
    g_assert_cmphex(host_reg(qts, HOST_INT_RAW), ==, HOSTINT_RX_NEW_PACKET);
    g_assert_cmphex(qtest_readl(qts, SLC_0_LENGTH), ==, len);
    g_assert_cmphex(qtest_readl(qts, SLCHOST + HOST_PKT_LEN), ==, 0);
    g_assert_cmphex(host_reg(qts, HOST_PKT_LEN), ==,
                    len | (((len & 0x3ff) + (len >> 10)) << 20));
    g_assert_cmphex(qtest_readl(qts, SLCHOST + HOST_PKT_LEN) & 0xfffff, ==,
                    len);

    /* One block of 512, then 12 bytes, ending at 0x1f800 */
    cmd53(qts, false, 1, DATA_END - len, got, 512, true);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_RX_EOF, ==, 0);
    g_assert_cmphex(qtest_readl(qts, d0), ==, (300 << 12) | 300);
    cmd53(qts, false, 1, DATA_END - 12, got + 512, 12, false);
    g_assert_cmpmem(got, len, pattern, len);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==,
                    SLCINT_RX_START | SLCINT_RX_DONE | SLCINT_RX_EOF);
    g_assert_cmphex(qtest_readl(qts, SLC_0TO_EOF_DES_ADDR), ==, d1);
    g_assert_cmphex(qtest_readl(qts, d1), ==,
                    DESC_EOF | (224 << 12) | 300);
    /* The list has ended. */
    g_assert_cmphex(qtest_readl(qts, SLC_0RX_LINK) & LINK_PARK, ==,
                    LINK_PARK);

    /* Reading on finds nothing: the FIFO underflows. */
    cmd53(qts, false, 1, DATA_END - 4, got, 4, false);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_RX_UDF, ==,
                    SLCINT_RX_UDF);
    g_assert_cmphex(host_reg(qts, HOST_INT_RAW) & HOSTINT_RX_UDF, ==,
                    HOSTINT_RX_UDF);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.sdio-slave/test] */
static void test_send_errors(void)
{
    QTestState *qts = start();
    const uint32_t d0 = RAM, d1 = RAM + 0x10;

    /* A descriptor whose buffer is not DMA RAM */
    put_desc(qts, d0, DESC_OWNER | DESC_EOF | (4 << 12) | 4, 0x3ff40000, 0);
    qtest_writel(qts, SLC_0RX_LINK, LINK_START | (d0 & 0xfffff));
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_RX_DSCR_ERR, ==,
                    SLCINT_RX_DSCR_ERR);

    /*
     * ESP-IDF's way to raise RX_DONE: a one-byte packet the host never
     * reads, from a buffer that need not be aligned
     */
    qtest_writel(qts, SLC_0INT_CLR, 0xffffffff);
    qtest_writel(qts, SLC_CONF0, CONF0_RX_RST);
    qtest_writel(qts, SLC_CONF0, 0);
    put_desc(qts, d1, DESC_OWNER | DESC_EOF | (1 << 12) | 1, 0x3ffbbbbb, 0);
    qtest_writel(qts, SLC_0RX_LINK, LINK_START | (d1 & 0xfffff));
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==,
                    SLCINT_RX_START | SLCINT_RX_DONE);
    /* Without RX_AUTO_WRBACK, the descriptor stays the DMA's. */
    g_assert_cmphex(qtest_readl(qts, d1) & DESC_OWNER, ==, DESC_OWNER);
    qtest_writel(qts, SLC_CONF0, CONF0_RX_RST);
    g_assert_cmphex(qtest_readl(qts, SLC_RX_STATUS) & 2, ==, 2);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.sdio-slave/test] */
static void test_receive_packet(void)
{
    QTestState *qts = start();
    const uint32_t d0 = RAM, d1 = RAM + 0x10, d2 = RAM + 0x20;
    const uint32_t buf0 = RAM + 0x100, buf1 = RAM + 0x200;
    const uint32_t buf2 = RAM + 0x300;
    uint8_t data[1024], got[64];

    for (int i = 0; i < sizeof(data); i++) {
        data[i] = 0xa5 ^ i;
    }
    card_init(qts);
    cmd52_write(qts, 0, 0x02, 0x02);

    /* The slave lends two 64-byte buffers and counts them for the host. */
    put_desc(qts, d0, DESC_OWNER | 64, buf0, d1);
    put_desc(qts, d1, DESC_OWNER | 64, buf1, 0);
    qtest_writel(qts, SLC_CONF0, 0);
    qtest_writel(qts, SLC_0TX_LINK, LINK_START | (d0 & 0xfffff));
    qtest_writel(qts, SLC_0TOKEN1, TOKEN_INC_MORE | 2);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==, SLCINT_TX_START);
    g_assert_cmphex(host_reg(qts, HOST_TOKEN_RDATA) >> 16, ==, 2);

    /* A 100-byte packet, ending at 0x1f800 */
    cmd53(qts, true, 1, DATA_END - 100, data, 100, false);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==,
                    SLCINT_TX_START | SLCINT_TX_DONE | SLCINT_TX_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, d0), ==, (64 << 12) | 64);
    g_assert_cmphex(qtest_readl(qts, d1), ==, DESC_EOF | (36 << 12) | 64);
    g_assert_cmphex(qtest_readl(qts, SLC_0TX_EOF_DES_ADDR), ==, d1);
    g_assert_cmphex(qtest_readl(qts, SLC_0TXLINK_DSCR), ==, d1);
    qtest_memread(qts, buf0, got, 64);
    g_assert_cmpmem(got, 64, data, 64);
    qtest_memread(qts, buf1, got, 36);
    g_assert_cmpmem(got, 36, data + 64, 36);

    /* No buffers left: the next packet overflows. */
    cmd53(qts, true, 1, DATA_END - 8, data, 8, false);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW) & SLCINT_TX_OVF, ==,
                    SLCINT_TX_OVF);
    g_assert_cmphex(host_reg(qts, HOST_INT_RAW) & HOSTINT_TX_OVF, ==,
                    HOSTINT_TX_OVF);

    /*
     * The slave links on another buffer and restarts the link; the host
     * sends 6 bytes, padded to 8: the padding past 0x1f800 is dropped.
     */
    qtest_writel(qts, SLC_0INT_CLR, 0xffffffff);
    put_desc(qts, d2, DESC_OWNER | 64, buf2, 0);
    qtest_writel(qts, d1 + 8, d2);
    qtest_writel(qts, SLC_0TX_LINK, LINK_RESTART);
    cmd53(qts, true, 1, DATA_END - 6, data + 200, 8, false);
    g_assert_cmphex(qtest_readl(qts, d2), ==, DESC_EOF | (6 << 12) | 64);
    qtest_memread(qts, buf2, got, 6);
    g_assert_cmpmem(got, 6, data + 200, 6);
    g_assert_cmphex(qtest_readl(qts, SLC_0INT_RAW), ==,
                    SLCINT_TX_DONE | SLCINT_TX_SUC_EOF);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/sdio/reset-values", test_reset_values);
    qtest_add_func("/esp32/sdio/cccr", test_cccr);
    qtest_add_func("/esp32/sdio/registers-and-interrupts",
                   test_registers_and_interrupts);
    qtest_add_func("/esp32/sdio/send-packet", test_send_packet);
    qtest_add_func("/esp32/sdio/send-errors", test_send_errors);
    qtest_add_func("/esp32/sdio/receive-packet", test_receive_packet);
    return g_test_run();
}
