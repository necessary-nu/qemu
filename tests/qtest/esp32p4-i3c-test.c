/*
 * QTest testcase for the ESP32-P4 I3C master and slave controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The master's QEMU I3C bus carries a mock I3C target with static address
 * 0x30 and a second without one, for ENTDAA, and its legacy I2C bus a
 * TMP105 at 0x48. The master is on GPIO20 (SCL) and GPIO21 (SDA), the
 * slave controller on GPIO22 and GPIO23, which the board wires to them.
 * I3C_MST_CLK runs from the 40 MHz crystal: 25 ns a cycle.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define MST                 0x500da000
#define SLV                 0x500db000

#define DEVICE_CTRL         0x00
#define COMMAND_BUF_PORT    0x08
#define RESPONSE_BUF_PORT   0x0c
#define RX_DATA_PORT        0x10
#define TX_DATA_PORT        0x14
#define IBI_STATUS_BUF      0x18
#define BUFFER_THLD_CTRL    0x1c
#define DATA_BUFFER_THLD_CTRL 0x20
#define IBI_NOTIFY_CTRL     0x24
#define IBI_SIR_REQ_PAYLOAD 0x28
#define IBI_SIR_REQ_REJECT  0x2c
#define INT_CLR             0x30
#define INT_RAW             0x34
#define INT_ST              0x38
#define INT_ST_ENA          0x3c
#define IBI_DATA_BUF        0x40
#define RESET_CTRL          0x44
#define BUFFER_STATUS_LEVEL 0x48
#define DATA_BUFFER_STATUS_LEVEL 0x4c
#define PRESENT_STATE0      0x50
#define PRESENT_STATE1      0x54
#define DEVICE_TABLE        0x58
#define TIME_OUT_VALUE      0x5c
#define OD_TIME             0x60
#define PP_TIME             0x64
#define FM_TIME             0x68
#define FMP_TIME            0x6c
#define SDA_HOLD_TIME       0x78
#define SCL_START_HOLD      0x7c
#define BUS_FREE_TIME       0x90
#define VER_ID              0xa0
#define RND_ECO_HIGH        0xb8
#define DAT(n)              (0xc0 + 4 * (n))
#define DCT(n, w)           (0x100 + 16 * (n) + 4 * (w))

#define CTRL_BA_INCLUDE     (1u << 1)
#define CTRL_DMA_RX_EN      (1u << 6)
#define CTRL_DMA_TX_EN      (1u << 7)
#define CTRL_TRANS_START    (1u << 2)
#define CTRL_AUTO_DIS_IBI   (1u << 5)
#define CTRL_SCL_OE_FORCE   (1u << 12)

#define INT_RESP_READY      (1u << 4)
#define INT_NXT_CMD_REQ_ERR (1u << 5)
#define INT_TRANSFER_ERR    (1u << 6)
#define INT_TRANSFER_COMPLETE (1u << 7)
#define INT_COMMAND_DONE    (1u << 8)
#define INT_IBI_HANDLE_DONE (1u << 13)
#define INT_IBI_DETECT      (1u << 14)

#define RST_CMD_BUF         (1u << 1)
#define RST_RESP_BUF        (1u << 2)
#define RST_TX_BUF          (1u << 3)
#define RST_RX_BUF          (1u << 4)

/* Command descriptors */
#define CMDL(attr, tid, ccc, cp, dev, mode, rnw, roc, toc) \
    ((attr) | ((tid) << 3) | ((ccc) << 7) | ((cp) << 15) | ((dev) << 16) | \
     ((mode) << 26) | ((rnw) << 29) | ((roc) << 30) | ((uint32_t)(toc) << 31))
#define ADDR_CMDL(ccc, dev, cnt, roc, toc) \
    (2 | ((ccc) << 7) | ((dev) << 16) | ((cnt) << 26) | ((roc) << 30) | \
     ((uint32_t)(toc) << 31))
#define IMM_CMDL(ccc, cp, dev, bc, roc, toc) \
    (1 | ((ccc) << 7) | ((cp) << 15) | ((dev) << 16) | ((bc) << 23) | \
     ((roc) << 30) | ((uint32_t)(toc) << 31))
#define DL(n)               ((uint32_t)(n) << 16)

#define RESP_DL(r)          ((r) & 0xffff)
#define RESP_TID(r)         (((r) >> 24) & 0xf)
#define RESP_ERR(r)         ((r) >> 28)

#define DAT_I3C_STATIC(sa, da) ((sa) | ((da) << 16))
#define DAT_DYNAMIC(da, par)   (((da) << 16) | ((par) << 23))
#define DAT_I2C(sa)         ((sa) | (1u << 31))

#define CCC_ENEC_DIRECT     0x80
#define CCC_ENTDAA          0x07
#define CCC_SETDASA         0x87
#define CCC_GETPID          0x8d
#define CCC_GETSTATUS       0x90

/* The slave */
#define S_CONFIG            0x04
#define S_STATUS            0x08
#define S_CTRL              0x0c
#define S_INTSET            0x10
#define S_INTCLR            0x14
#define S_INTMASKED         0x18
#define S_DATACTRL          0x2c
#define S_WDATAB            0x30
#define S_WDATABE           0x34
#define S_RDARAB            0x40
#define S_RDATAH            0x48
#define S_CAPABILITIES2     0x5c
#define S_CAPABILITIES      0x60
#define S_IDPARTNO          0x6c
#define S_IDEXT             0x70
#define S_VENDORID          0x74
#define ST_STNOTSTOP        (1u << 0)
#define ST_START            (1u << 8)
#define ST_MATCHED          (1u << 9)
#define ST_STOP             (1u << 10)
#define ST_RXPEND           (1u << 11)
#define ST_TXNOTFULL        (1u << 12)
#define ST_DACHG            (1u << 13)

/* GPIO matrix and IO MUX */
#define GPIO_PIN(n)         (0x500e0074 + 4 * (n))
#define GPIO_FUNC_IN(s)     (0x500e0158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (0x500e0558 + 4 * (n))
#define PIN_PAD_DRIVER      (1u << 2)
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define PAD_PULLUP          ((1u << 12) | (1u << 9) | (1u << 8) | (2u << 10))
#define SIG_MST_SCL         134
#define SIG_MST_SDA         135
#define SIG_SLV_SCL         136
#define SIG_SLV_SDA         137

#define SOC_CLK_CTRL2       0x500e601c
#define PERI_CLK_CTRL119    0x500e608c
#define HP_RST_EN1          0x500e60c4
#define I3C_MST_APB_CLK_EN  (1u << 17)
#define I3C_SLV_APB_CLK_EN  (1u << 18)
#define I3C_MST_CLK_EN      (1u << 18)
#define RST_EN_I3CMST       (1u << 19)
#define RST_EN_I3CSLV       (1u << 20)

#define INTMTX_STATUS(w)    (0x500d6200 + 4 * (w))
/* I3C_MST_INTR 101 and I3C_SLV_INTR 102, word 3 */
#define SRC_I3C_MST         (1u << 5)
#define SRC_I3C_SLV         (1u << 6)

/* GDMA-AHB channel 0, and descriptors in L2MEM past the L2 cache */
#define AHB                 0x50085000
#define AHB_IN_INT(ch)      (AHB + 0x10 * (ch))
#define AHB_OUT_INT(ch)     (AHB + 0x30 + 0x10 * (ch))
#define AHB_IN(ch)          (AHB + 0x70 + 0xc0 * (ch))
#define AHB_OUT(ch)         (AHB + 0xd0 + 0xc0 * (ch))
#define AHB_IN_LINK_ADDR(ch)  (AHB + 0x3ac + 4 * (ch))
#define AHB_OUT_LINK_ADDR(ch) (AHB + 0x3b8 + 4 * (ch))
#define DMA_CONF0           0x00
#define DMA_LINK            0x10
#define DMA_PERI_SEL        0x30
#define DMA_RAW             0x0
#define DMA_CLR             0xc
#define IN_SUC_EOF          (1u << 1)
#define OUT_EOF             (1u << 1)
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define OUT_AUTO_WRBACK     (1u << 2)
#define OUT_EOF_MODE        (1u << 3)
#define DW0(size, len)      ((size) | (len) << 12)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)
#define PERI_I3C            0
#define MEM                 0x4ff40000

#define MOCK_SA             0x30
#define MOCK_DA             0x08
#define DAA_DA              0x09
#define SLV_SA              0x25
#define SLV_DA              0x0a
#define TMP105_ADDR         0x48
#define IBI_MAGIC           0xa5

#define CYCLE_NS            25

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M "
                      "-device mock-i3c-target,id=mock,bus=i3c,"
                      "static-address=0x30,pid=0xffeeddccbbaa,bcr=0x06,"
                      "dcr=0x44,ibi-magic-num=0xa5 "
                      "-device mock-i3c-target,id=daa,bus=i3c,"
                      "pid=0x0123456789ab,bcr=0x01,dcr=0x22 "
                      "-device tmp105,id=tmp,bus=i3c-legacy-i2c,address=0x48 "
                      "-global esp32p4-gpio.pad-wires=20:22,21:23");
}

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t off)
{
    return qtest_readl(qts, base + off);
}

static void wr(QTestState *qts, uint32_t base, uint32_t off, uint32_t v)
{
    qtest_writel(qts, base + off, v);
}

static void pad(QTestState *qts, int n, int out_sig, int in_sig, bool od)
{
    qtest_writel(qts, IO_MUX_GPIO(n), PAD_PULLUP);
    qtest_writel(qts, GPIO_PIN(n), od ? PIN_PAD_DRIVER : 0);
    qtest_writel(qts, GPIO_FUNC_OUT(n), out_sig);
    qtest_writel(qts, GPIO_FUNC_IN(in_sig), IN_SIG_IN_SEL | n);
}

static void setup(QTestState *qts)
{
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 I3C_MST_APB_CLK_EN | I3C_SLV_APB_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL119, I3C_MST_CLK_EN);
    qtest_writel(qts, HP_RST_EN1, qtest_readl(qts, HP_RST_EN1) &
                 ~(RST_EN_I3CMST | RST_EN_I3CSLV));
    pad(qts, 20, SIG_MST_SCL, SIG_MST_SCL, false);
    pad(qts, 21, SIG_MST_SDA, SIG_MST_SDA, true);
    pad(qts, 22, SIG_SLV_SCL, SIG_SLV_SCL, true);
    pad(qts, 23, SIG_SLV_SDA, SIG_SLV_SDA, true);
}

static void run(QTestState *qts, const uint32_t *cmds, int n)
{
    int i;

    wr(qts, MST, INT_CLR, 0xffff);
    for (i = 0; i < n; i++) {
        wr(qts, MST, COMMAND_BUF_PORT, cmds[i]);
    }
    wr(qts, MST, DEVICE_CTRL, rd(qts, MST, DEVICE_CTRL) | CTRL_TRANS_START);
    wr(qts, MST, DEVICE_CTRL, rd(qts, MST, DEVICE_CTRL) & ~CTRL_TRANS_START);
}

static void wait_complete(QTestState *qts)
{
    int i;

    for (i = 0; i < 10000; i++) {
        if (rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE) {
            return;
        }
        qtest_clock_step(qts, 1000);
    }
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE, !=, 0);
}

/* SETDASA: the mock target at 0x30 takes 0x08. */
static void setdasa(QTestState *qts)
{
    uint32_t cmds[] = { ADDR_CMDL(CCC_SETDASA, 0, 1, 1, 1), 0 };
    uint32_t r;

    wr(qts, MST, DAT(0), DAT_I3C_STATIC(MOCK_SA, MOCK_DA));
    run(qts, cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_DL(r), ==, 0);
}

/* Every register at its reset value */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    setup(qts);
    g_assert_cmphex(rd(qts, MST, DEVICE_CTRL), ==, 0x1020);
    g_assert_cmphex(rd(qts, MST, BUFFER_THLD_CTRL), ==, 0x41041);
    g_assert_cmphex(rd(qts, MST, DATA_BUFFER_THLD_CTRL), ==, 0x9);
    /* Command buffer empty: CMD_BUF_EMPTY_THLD; TX empty: TX_THLD */
    g_assert_cmphex(rd(qts, MST, INT_RAW), ==, 0x9);
    g_assert_cmphex(rd(qts, MST, BUFFER_STATUS_LEVEL), ==, 16);
    g_assert_cmphex(rd(qts, MST, DATA_BUFFER_STATUS_LEVEL), ==, 32);
    g_assert_cmphex(rd(qts, MST, PRESENT_STATE0) & 0xf, ==, 0xb);
    g_assert_cmphex(rd(qts, MST, TIME_OUT_VALUE), ==, 0x410410);
    g_assert_cmphex(rd(qts, MST, OD_TIME), ==, 0x00050019);
    g_assert_cmphex(rd(qts, MST, PP_TIME), ==, 0x00050005);
    g_assert_cmphex(rd(qts, MST, FM_TIME), ==, 0x004b00a3);
    g_assert_cmphex(rd(qts, MST, FMP_TIME), ==, 0x0021003f);
    g_assert_cmphex(rd(qts, MST, SDA_HOLD_TIME), ==, 1);
    g_assert_cmphex(rd(qts, MST, SCL_START_HOLD), ==, 8);
    g_assert_cmphex(rd(qts, MST, BUS_FREE_TIME), ==, 5);
    g_assert_cmphex(rd(qts, MST, VER_ID), ==, 0x20230504);
    g_assert_cmphex(rd(qts, MST, RND_ECO_HIGH), ==, 0xffff);

    g_assert_cmphex(rd(qts, SLV, S_CONFIG), ==, 0x002f0001);
    g_assert_cmphex(rd(qts, SLV, S_STATUS), ==, ST_TXNOTFULL);
    g_assert_cmphex(rd(qts, SLV, S_DATACTRL), ==, 0x800000b0);
    g_assert_cmphex(rd(qts, SLV, S_CAPABILITIES), ==, 0x7c13fc1c);
    g_assert_cmphex(rd(qts, SLV, S_CAPABILITIES2), ==, 0x100);
    g_assert_cmphex(rd(qts, SLV, S_VENDORID), ==, 0x5550);
    qtest_quit(qts);
}

/*
 * SETDASA, then a private write and read back of the mock target's
 * buffer at its dynamic address.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_private(void)
{
    QTestState *qts = start();
    uint32_t wr_cmds[] = { CMDL(0, 3, 0, 0, 0, 0, 0, 1, 1), DL(6) };
    uint32_t rd_cmds[] = { CMDL(0, 4, 0, 0, 0, 0, 1, 1, 1), DL(6) };
    uint32_t r;

    setup(qts);
    setdasa(qts);
    wr(qts, MST, TX_DATA_PORT, 0x44332211);
    wr(qts, MST, TX_DATA_PORT, 0x6655);
    run(qts, wr_cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_TID(r), ==, 3);
    g_assert_cmphex(RESP_DL(r), ==, 0);
    g_assert_cmpuint(rd(qts, MST, PRESENT_STATE1), ==, 6);

    run(qts, rd_cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_DL(r), ==, 6);
    g_assert_cmphex(rd(qts, MST, DATA_BUFFER_STATUS_LEVEL) >> 16, ==, 2);
    g_assert_cmphex(rd(qts, MST, RX_DATA_PORT), ==, 0x44332211);
    g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xffff, ==, 0x6655);

    /*
     * Chained by repeated STARTs (TOC clear): a write, a read that goes
     * on where the write left off, with no STOP between, and a direct
     * CCC (GETPID), which starts with the broadcast address again.
     */
    {
        uint32_t chain[] = { CMDL(0, 1, 0, 0, 0, 0, 0, 1, 0), DL(2),
                             CMDL(0, 2, 0, 0, 0, 0, 1, 1, 0), DL(2),
                             CMDL(0, 3, CCC_GETPID, 1, 0, 0, 1, 1, 1),
                             DL(6) };

        wr(qts, MST, TX_DATA_PORT, 0x7c5b);
        run(qts, chain, 6);
        wait_complete(qts);
        g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_ERR, ==, 0);
        r = rd(qts, MST, RESPONSE_BUF_PORT);
        g_assert_cmphex(RESP_TID(r), ==, 1);
        g_assert_cmphex(RESP_ERR(r), ==, 0);
        r = rd(qts, MST, RESPONSE_BUF_PORT);
        g_assert_cmphex(RESP_TID(r), ==, 2);
        g_assert_cmphex(RESP_DL(r), ==, 2);
        r = rd(qts, MST, RESPONSE_BUF_PORT);
        g_assert_cmphex(RESP_TID(r), ==, 3);
        g_assert_cmphex(RESP_DL(r), ==, 6);
        g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xffff, ==, 0x4433);
        /* The mock's PID, 0xaabbccddeeff, as it sends it */
        g_assert_cmphex(rd(qts, MST, RX_DATA_PORT), ==, 0xddccbbaa);
        g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xffff, ==, 0xffee);
    }
    qtest_quit(qts);
}

/* ENTDAA finds the target without a static address and records it. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_entdaa(void)
{
    QTestState *qts = start();
    /* 0x09 has three bits set: odd parity is 0 */
    uint32_t cmds[] = { ADDR_CMDL(CCC_ENTDAA, 1, 1, 0, 1), 0 };
    uint32_t again[] = { ADDR_CMDL(CCC_ENTDAA, 2, 1, 0, 1), 0 };
    uint32_t r;

    setup(qts);
    setdasa(qts);
    wr(qts, MST, DAT(1), DAT_DYNAMIC(DAA_DA, 0));
    run(qts, cmds, 2);
    wait_complete(qts);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_ERR, ==, 0);
    /* No ROC, no error: no response */
    g_assert_cmphex(rd(qts, MST, BUFFER_STATUS_LEVEL) & 0xf00, ==, 0);
    /*
     * QEMU's targets keep their "pid" property byte-reversed, sending its
     * low byte first: this one's provisioned ID is 0xab8967452301.
     */
    g_assert_cmphex(rd(qts, MST, DCT(0, 0)), ==, 0xab896745);
    g_assert_cmphex(rd(qts, MST, DCT(0, 1)), ==, 0x2301);
    g_assert_cmphex(rd(qts, MST, DCT(0, 2)), ==, 0x2201);
    g_assert_cmphex(rd(qts, MST, DCT(0, 3)), ==, DAA_DA << 1);
    g_assert_cmphex((rd(qts, MST, DEVICE_TABLE) >> 8) & 0xf, ==, 1);

    /*
     * The slave controller, wired to the master's pads, has no address
     * either: it is next, its provisioned ID VENDORID << 33.
     */
    wr(qts, MST, DAT(2), DAT_DYNAMIC(SLV_DA, 1));
    run(qts, again, 2);
    wait_complete(qts);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_ERR, ==, 0);
    g_assert_cmphex(rd(qts, MST, DCT(1, 0)), ==, 0xaaa00000);
    g_assert_cmphex(rd(qts, MST, DCT(1, 1)), ==, 0);
    g_assert_cmphex(rd(qts, MST, DCT(1, 3)), ==, (SLV_DA << 1) | 1);
    g_assert_cmphex(rd(qts, SLV, S_STATUS) & ST_DACHG, ==, ST_DACHG);

    /* Everyone has an address now: nobody answers 7E/R. */
    run(qts, again, 2);
    wait_complete(qts);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_ERR, ==,
                    INT_TRANSFER_ERR);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 5);
    g_assert_cmphex(RESP_DL(r), ==, 1);

    /* The new address works: GETPID, a direct CCC read. */
    {
        uint32_t getpid[] = { CMDL(0, 1, CCC_GETPID, 1, 1, 0, 1, 1, 1),
                              DL(6) };

        run(qts, getpid, 2);
        wait_complete(qts);
        r = rd(qts, MST, RESPONSE_BUF_PORT);
        g_assert_cmphex(RESP_ERR(r), ==, 0);
        g_assert_cmphex(RESP_DL(r), ==, 6);
        g_assert_cmphex(rd(qts, MST, RX_DATA_PORT), ==, 0x456789ab);
        g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xffff, ==, 0x0123);
    }
    qtest_quit(qts);
}

/* A private write to an address nobody has: error 5, then a STOP. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_nack(void)
{
    QTestState *qts = start();
    uint32_t cmds[] = { CMDL(0, 7, 0, 0, 0, 0, 0, 0, 0), DL(1),
                        CMDL(0, 8, 0, 0, 0, 0, 0, 1, 1), DL(1) };
    uint32_t r;

    setup(qts);
    wr(qts, MST, DAT(0), DAT_DYNAMIC(0x11, 0));
    wr(qts, MST, TX_DATA_PORT, 0x5a);
    run(qts, cmds, 4);
    wait_complete(qts);
    g_assert_cmphex(rd(qts, MST, INT_RAW) &
                    (INT_TRANSFER_ERR | INT_COMMAND_DONE), ==,
                    INT_TRANSFER_ERR | INT_COMMAND_DONE);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 5);
    g_assert_cmphex(RESP_TID(r), ==, 7);
    g_assert_cmphex(RESP_DL(r), ==, 1);
    /* The second command was discarded. */
    g_assert_cmphex(rd(qts, MST, BUFFER_STATUS_LEVEL) & 0xf1f, ==, 16);
    qtest_quit(qts);
}

/* The TMP105 on the legacy I2C bus: write the pointer, then read. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_legacy_i2c(void)
{
    QTestState *qts = start();
    uint32_t cmds[] = { CMDL(0, 0, 0, 0, 2, 0, 0, 0, 0), DL(1),
                        CMDL(0, 1, 0, 0, 2, 0, 1, 1, 1), DL(2) };
    QDict *rsp;
    uint32_t r;

    setup(qts);
    rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                    "{ 'path': '/machine/peripheral/tmp', "
                    "'property': 'temperature', 'value': 33000 } }");
    qobject_unref(rsp);
    wr(qts, MST, DAT(2), DAT_I2C(TMP105_ADDR));
    wr(qts, MST, TX_DATA_PORT, 0);
    run(qts, cmds, 4);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_DL(r), ==, 2);
    g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xffff, ==, 33);

    /* An I2C device that is not there */
    wr(qts, MST, DAT(2), DAT_I2C(0x4f));
    wr(qts, MST, TX_DATA_PORT, 0);
    run(qts, cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 5);
    qtest_quit(qts);
}

/*
 * The master in I2C mode and the general call. With only an ES8311 on
 * the legacy bus, which answers its own address only, a general call is
 * NACKed (error 5), and afterwards the codec reads its chip ID, an absent
 * address is NACKed and so is the next general call. With a TMP105 there
 * too, which takes general calls, a general call and then, after
 * repeated STARTs (TOC clear), a register read from the codec reach the
 * codec alone: a repeated START to another address leaves the general
 * call.
 */
#define ES8311_ADDR         0x18
#define ES8311_CHIP_ID1     0xfd

/* Empty the TX and RX buffers, of what a NACKed command left there. */
static void reset_data_bufs(QTestState *qts)
{
    wr(qts, MST, RESET_CTRL, RST_TX_BUF | RST_RX_BUF);
}

/* A command to legacy device @dev: write the byte @b, or read one. */
static uint32_t legacy_xfer(QTestState *qts, int dev, bool rnw, uint8_t b,
                            bool toc, uint32_t *cmd)
{
    if (!rnw) {
        wr(qts, MST, TX_DATA_PORT, b);
    }
    cmd[0] = CMDL(0, dev, 0, 0, dev, 0, rnw, 1, toc);
    cmd[1] = DL(1);
    return 2;
}

static uint32_t legacy_run(QTestState *qts, const uint32_t *cmds, int n,
                           int responses)
{
    uint32_t err = 0;
    int i;

    run(qts, cmds, n);
    wait_complete(qts);
    for (i = 0; i < responses; i++) {
        err = (err << 4) | RESP_ERR(rd(qts, MST, RESPONSE_BUF_PORT));
    }
    qtest_clock_step(qts, 100 * 1000);
    return err;
}

static uint8_t legacy_chip_id(QTestState *qts)
{
    uint32_t cmds[4];

    reset_data_bufs(qts);
    legacy_xfer(qts, 1, false, ES8311_CHIP_ID1, false, cmds);
    legacy_xfer(qts, 1, true, 0, true, cmds + 2);
    g_assert_cmphex(legacy_run(qts, cmds, 4, 2), ==, 0);
    return rd(qts, MST, RX_DATA_PORT) & 0xff;
}

/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_legacy_general_call(void)
{
    QTestState *qts;
    uint32_t cmds[6];
    int i;

    qts = qtest_init("-M esp32p4,flash-size=16M "
                     "-device es8311,bus=i3c-legacy-i2c "
                     "-global esp32p4-gpio.pad-wires=20:22,21:23");
    setup(qts);
    wr(qts, MST, DAT(0), DAT_I2C(0));
    wr(qts, MST, DAT(1), DAT_I2C(ES8311_ADDR));
    wr(qts, MST, DAT(2), DAT_I2C(0x33));
    for (i = 0; i < 2; i++) {
        reset_data_bufs(qts);
        legacy_xfer(qts, 0, false, 0x06, true, cmds);
        g_assert_cmphex(legacy_run(qts, cmds, 2, 1), ==, 5);
        g_assert_cmphex(legacy_chip_id(qts), ==, 0x83);
    }
    reset_data_bufs(qts);
    legacy_xfer(qts, 0, false, 0x06, true, cmds);
    g_assert_cmphex(legacy_run(qts, cmds, 2, 1), ==, 5);
    reset_data_bufs(qts);
    legacy_xfer(qts, 2, false, 0x06, true, cmds);
    g_assert_cmphex(legacy_run(qts, cmds, 2, 1), ==, 5);
    g_assert_cmphex(legacy_chip_id(qts), ==, 0x83);
    qtest_quit(qts);

    qts = qtest_init("-M esp32p4,flash-size=16M "
                     "-device es8311,bus=i3c-legacy-i2c "
                     "-device tmp105,bus=i3c-legacy-i2c,address=0x48 "
                     "-global esp32p4-gpio.pad-wires=20:22,21:23");
    setup(qts);
    wr(qts, MST, DAT(0), DAT_I2C(0));
    wr(qts, MST, DAT(1), DAT_I2C(ES8311_ADDR));
    legacy_xfer(qts, 0, false, 0x00, false, cmds);
    legacy_xfer(qts, 1, false, ES8311_CHIP_ID1, false, cmds + 2);
    legacy_xfer(qts, 1, true, 0, true, cmds + 4);
    g_assert_cmphex(legacy_run(qts, cmds, 6, 3), ==, 0);
    g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xff, ==, 0x83);
    qtest_quit(qts);
}

/*
 * Timing: START, the header open drain, two bytes push-pull, the STOP,
 * at the reset values' counts (OD 26 + 6, PP 6 + 6, START_HOLD 9, STOP
 * PP_LOW 6 + STOP_SETUP 9) of I3C_MST_CLK.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_timing(void)
{
    QTestState *qts = start();
    uint32_t cmds[] = { CMDL(0, 0, 0, 0, 0, 0, 0, 1, 1), DL(2) };
    uint64_t cycles = 9 + 9 * 32 + 2 * 9 * 12 + 6 + 9;

    setup(qts);
    setdasa(qts);
    qtest_clock_step(qts, 10000);
    wr(qts, MST, TX_DATA_PORT, 0xbeef);
    run(qts, cmds, 2);
    qtest_clock_step(qts, cycles * CYCLE_NS - 1);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE, ==, 0);
    g_assert_cmphex(rd(qts, MST, PRESENT_STATE0) & 4, ==, 4);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE, ==,
                    INT_TRANSFER_COMPLETE);

    /* A slower push-pull clock: PP 20 + 20 */
    wr(qts, MST, PP_TIME, (19 << 16) | 19);
    rd(qts, MST, RESPONSE_BUF_PORT);
    qtest_clock_step(qts, 10000);
    wr(qts, MST, TX_DATA_PORT, 0xbeef);
    run(qts, cmds, 2);
    cycles = 9 + 9 * 32 + 2 * 9 * 40 + 20 + 9;
    qtest_clock_step(qts, cycles * CYCLE_NS - 1);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE, ==,
                    INT_TRANSFER_COMPLETE);

    /* I3C_MST_CLK stopped: nothing moves. */
    qtest_clock_step(qts, 10000);
    wr(qts, MST, TX_DATA_PORT, 0xbeef);
    qtest_writel(qts, PERI_CLK_CTRL119, 0);
    run(qts, cmds, 2);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_TRANSFER_COMPLETE, ==, 0);
    qtest_writel(qts, PERI_CLK_CTRL119, I3C_MST_CLK_EN);
    wait_complete(qts);
    qtest_quit(qts);
}

/*
 * An IBI from the mock target, which it raises 1 ms after it is sent its
 * magic byte once ENEC has enabled its events: taken with IBI_STATUS
 * holding its address, then refused with IBI_SIR_REQ_REJECT, which
 * disables it with DISEC. (Direct ENEC: the mock takes the broadcast
 * ENEC's CCC code, 0, for no CCC at all.)
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_ibi(void)
{
    QTestState *qts = start();
    uint32_t enec[] = { CMDL(0, 0, CCC_ENEC_DIRECT, 1, 0, 0, 0, 1, 1),
                        DL(1) };
    uint32_t magic[] = { CMDL(0, 0, 0, 0, 0, 0, 0, 1, 1), DL(1) };
    unsigned bit = ((MOCK_DA >> 5) + (MOCK_DA & 0x1f)) % 32;
    uint32_t st;

    setup(qts);
    setdasa(qts);
    wr(qts, MST, TX_DATA_PORT, 0x01);
    run(qts, enec, 2);
    wait_complete(qts);
    rd(qts, MST, RESPONSE_BUF_PORT);
    wr(qts, MST, TX_DATA_PORT, IBI_MAGIC);
    run(qts, magic, 2);
    wait_complete(qts);
    rd(qts, MST, RESPONSE_BUF_PORT);
    wr(qts, MST, INT_ST_ENA, INT_IBI_HANDLE_DONE);
    qtest_clock_step(qts, 1100000);
    g_assert_cmphex(rd(qts, MST, INT_RAW) &
                    (INT_IBI_DETECT | INT_IBI_HANDLE_DONE), ==,
                    INT_IBI_DETECT | INT_IBI_HANDLE_DONE);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(3)) & SRC_I3C_MST, ==,
                    SRC_I3C_MST);
    g_assert_cmphex((rd(qts, MST, BUFFER_STATUS_LEVEL) >> 24) & 0xf, ==, 1);
    st = rd(qts, MST, IBI_STATUS_BUF);
    g_assert_cmphex((st >> 8) & 0xff, ==, (MOCK_DA << 1) | 1);
    g_assert_cmphex(st & 0xff, ==, 0);
    g_assert_cmphex(st >> 28, ==, 0);

    /* Refused, and reported as such; DISEC turns the target's IBIs off. */
    wr(qts, MST, IBI_SIR_REQ_REJECT, 1u << bit);
    wr(qts, MST, IBI_NOTIFY_CTRL, 1u << 2);
    wr(qts, MST, INT_CLR, 0xffff);
    wr(qts, MST, TX_DATA_PORT, IBI_MAGIC);
    run(qts, magic, 2);
    wait_complete(qts);
    rd(qts, MST, RESPONSE_BUF_PORT);
    qtest_clock_step(qts, 1100000);
    st = rd(qts, MST, IBI_STATUS_BUF);
    g_assert_cmphex((st >> 8) & 0xff, ==, (MOCK_DA << 1) | 1);
    g_assert_cmphex(st >> 28, ==, 1);
    wr(qts, MST, IBI_SIR_REQ_REJECT, 0);
    wr(qts, MST, INT_CLR, 0xffff);
    wr(qts, MST, TX_DATA_PORT, IBI_MAGIC);
    run(qts, magic, 2);
    wait_complete(qts);
    qtest_clock_step(qts, 1100000);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_IBI_DETECT, ==, 0);
    qtest_quit(qts);
}

/*
 * The slave controller on pads wired to the master's: SETDASA gives it
 * a dynamic address (DACHG), a private write fills its RX FIFO, a read
 * takes its TX FIFO up to the byte marked as the end, and its IBI, with
 * its mandatory data byte, reaches the master.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_slave(void)
{
    QTestState *qts = start();
    uint32_t dasa[] = { ADDR_CMDL(CCC_SETDASA, 3, 1, 1, 1), 0 };
    uint32_t wr_cmds[] = { CMDL(0, 0, 0, 0, 3, 0, 0, 1, 1), DL(3) };
    uint32_t rd_cmds[] = { CMDL(0, 0, 0, 0, 3, 0, 1, 1, 1), DL(8) };
    unsigned bit = ((SLV_DA >> 5) + (SLV_DA & 0x1f)) % 32;
    uint32_t r, st;

    setup(qts);
    wr(qts, SLV, S_CONFIG, 0x002f0001 | (SLV_SA << 25));
    wr(qts, SLV, S_IDEXT, (0x06 << 16) | (0x55 << 8));
    wr(qts, MST, DAT(3), DAT_I3C_STATIC(SLV_SA, SLV_DA));
    run(qts, dasa, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(rd(qts, SLV, S_STATUS) & (ST_DACHG | ST_START | ST_STOP),
                    ==, ST_DACHG | ST_START | ST_STOP);
    wr(qts, SLV, S_STATUS, 0xffffffff);

    wr(qts, SLV, S_INTSET, ST_STOP);
    /* RXPEND once the RX FIFO is not empty (RXTRIG 0) */
    wr(qts, SLV, S_DATACTRL, (1u << 3) | (3u << 4));
    wr(qts, MST, TX_DATA_PORT, 0x00c0ffee);
    run(qts, wr_cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(rd(qts, SLV, S_STATUS) & (ST_MATCHED | ST_RXPEND), ==,
                    ST_MATCHED | ST_RXPEND);
    g_assert_cmphex(rd(qts, SLV, S_INTMASKED), ==, ST_STOP);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(3)) & SRC_I3C_SLV, ==,
                    SRC_I3C_SLV);
    g_assert_cmphex((rd(qts, SLV, S_DATACTRL) >> 24) & 0x1f, ==, 3);
    g_assert_cmphex(rd(qts, SLV, S_RDATAH), ==, 0xffee);
    g_assert_cmphex(rd(qts, SLV, S_RDARAB), ==, 0xc0);
    wr(qts, SLV, S_STATUS, ST_STOP);
    g_assert_cmphex(rd(qts, SLV, S_INTMASKED), ==, 0);

    /* The slave sends three bytes, the last marked as the end. */
    wr(qts, SLV, S_WDATAB, 0x11);
    wr(qts, SLV, S_WDATAB, 0x22);
    wr(qts, SLV, S_WDATABE, 0x33);
    run(qts, rd_cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_DL(r), ==, 3);
    g_assert_cmphex(rd(qts, MST, RX_DATA_PORT) & 0xffffff, ==, 0x332211);

    /* An empty TX FIFO: the slave refuses the read. */
    run(qts, rd_cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 5);

    /* The slave's IBI, with its data byte */
    wr(qts, MST, IBI_SIR_REQ_PAYLOAD, 1u << bit);
    wr(qts, MST, INT_CLR, 0xffff);
    wr(qts, SLV, S_CTRL, (0x5d << 8) | 1);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, SLV, S_CTRL) & 3, ==, 0);
    g_assert_cmphex(rd(qts, MST, INT_RAW) & INT_IBI_HANDLE_DONE, ==,
                    INT_IBI_HANDLE_DONE);
    st = rd(qts, MST, IBI_STATUS_BUF);
    g_assert_cmphex((st >> 8) & 0xff, ==, (SLV_DA << 1) | 1);
    g_assert_cmphex(st & 0xff, ==, 1);
    g_assert_cmphex(rd(qts, MST, IBI_DATA_BUF) & 0xff, ==, 0x5d);
    qtest_quit(qts);
}

/*
 * With DMA_TX_EN and DMA_RX_EN the data goes through GDMA-AHB: a write
 * takes its bytes from a TX descriptor, a read leaves them in an RX
 * descriptor, which the read's end closes with suc_eof.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_dma(void)
{
    QTestState *qts = start();
    uint32_t wr_cmds[] = { CMDL(0, 0, 0, 0, 0, 0, 0, 1, 1), DL(8) };
    uint32_t rd_cmds[] = { CMDL(0, 0, 0, 0, 0, 0, 1, 1, 1), DL(8) };
    uint8_t data[8] = { 0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04 };
    uint8_t got[8];
    uint32_t r;

    setup(qts);
    setdasa(qts);
    qtest_memwrite(qts, MEM + 0x1000, data, 8);
    qtest_writel(qts, MEM, DW0(8, 8) | SUC_EOF | OWNER);
    qtest_writel(qts, MEM + 4, MEM + 0x1000);
    qtest_writel(qts, MEM + 8, 0);
    qtest_writel(qts, MEM + 0x10, DW0(8, 0) | OWNER);
    qtest_writel(qts, MEM + 0x14, MEM + 0x2000);
    qtest_writel(qts, MEM + 0x18, 0);
    qtest_writel(qts, AHB_OUT(0) + DMA_PERI_SEL, PERI_I3C);
    qtest_writel(qts, AHB_IN(0) + DMA_PERI_SEL, PERI_I3C);
    qtest_writel(qts, AHB_OUT(0) + DMA_CONF0, OUT_EOF_MODE |
                 OUT_AUTO_WRBACK);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(0), MEM);
    qtest_writel(qts, AHB_IN_LINK_ADDR(0), MEM + 0x10);
    qtest_writel(qts, AHB_OUT(0) + DMA_LINK, OUT_LINK_START);
    qtest_writel(qts, AHB_IN(0) + DMA_LINK, IN_LINK_START);
    wr(qts, MST, DEVICE_CTRL, rd(qts, MST, DEVICE_CTRL) | CTRL_DMA_TX_EN |
       CTRL_DMA_RX_EN);

    run(qts, wr_cmds, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_DL(r), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_INT(0) + DMA_RAW) & OUT_EOF, ==,
                    OUT_EOF);

    run(qts, rd_cmds, 2);
    wait_complete(qts);
    qtest_clock_step(qts, 10000);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    g_assert_cmphex(RESP_ERR(r), ==, 0);
    g_assert_cmphex(RESP_DL(r), ==, 8);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(0) + DMA_RAW) & IN_SUC_EOF,
                    ==, IN_SUC_EOF);
    qtest_memread(qts, MEM + 0x2000, got, 8);
    g_assert_cmpmem(got, 8, data, 8);
    /* The RX descriptor came back with the length and suc_eof. */
    g_assert_cmphex(qtest_readl(qts, MEM + 0x10), ==, DW0(8, 8) | SUC_EOF);
    /* Nothing went through the CPU's ports. */
    g_assert_cmphex(rd(qts, MST, DATA_BUFFER_STATUS_LEVEL), ==, 32);
    qtest_quit(qts);
}

/* Not wired to the master's pads, the slave hears nothing. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_slave_unwired(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M");
    uint32_t dasa[] = { ADDR_CMDL(CCC_SETDASA, 3, 1, 1, 1), 0 };
    uint32_t r;

    setup(qts);
    wr(qts, SLV, S_CONFIG, 0x002f0001 | (SLV_SA << 25));
    wr(qts, MST, DAT(3), DAT_I3C_STATIC(SLV_SA, SLV_DA));
    run(qts, dasa, 2);
    wait_complete(qts);
    r = rd(qts, MST, RESPONSE_BUF_PORT);
    /* Nothing on the bus at all: the broadcast address is NACKed. */
    g_assert_cmphex(RESP_ERR(r), ==, 4);
    g_assert_cmphex(rd(qts, SLV, S_STATUS) & ST_DACHG, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/i3c/reset-values", test_reset_values);
    qtest_add_func("esp32p4/i3c/private", test_private);
    qtest_add_func("esp32p4/i3c/entdaa", test_entdaa);
    qtest_add_func("esp32p4/i3c/nack", test_nack);
    qtest_add_func("esp32p4/i3c/legacy-i2c", test_legacy_i2c);
    qtest_add_func("esp32p4/i3c/legacy-general-call",
                   test_legacy_general_call);
    qtest_add_func("esp32p4/i3c/timing", test_timing);
    qtest_add_func("esp32p4/i3c/ibi", test_ibi);
    qtest_add_func("esp32p4/i3c/slave", test_slave);
    qtest_add_func("esp32p4/i3c/slave-unwired", test_slave_unwired);
    qtest_add_func("esp32p4/i3c/dma", test_dma);
    return g_test_run();
}
