/*
 * QTest testcase for the ESP32 Ethernet MAC and its IP101 PHY
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/sockets.h"
#include "libqtest.h"

#define DPORT_WIFI_CLK_EN   0x3ff000cc
#define DPORT_CORE_RST_EN   0x3ff000d0
#define WIFI_CLK_EMAC       (1u << 14)
#define CORE_RST_EMAC       (1u << 7)

#define EMAC                0x3ff69000
#define EMAC_APB            0x60029000

#define BUS_MODE            0x0000
#define TX_POLL             0x0004
#define RX_POLL             0x0008
#define RX_BASE             0x000c
#define TX_BASE             0x0010
#define STATUS              0x0014
#define OP_MODE             0x0018
#define INT_EN              0x001c
#define MISSED              0x0020
#define TX_CUR_DESC         0x0048
#define RX_CUR_DESC         0x004c
#define EX_CLKOUT_CONF      0x0800
#define EX_OSCCLK_CONF      0x0804
#define EX_CLK_CTRL         0x0808
#define EX_PHYINF_CONF      0x080c
#define EX_DATE             0x08fc
#define CONFIG              0x1000
#define FRAME_FILTER        0x1004
#define MII_ADDR            0x1010
#define MII_DATA            0x1014
#define DEBUG               0x1024
#define LPI_TIMERS          0x1034
#define ADDR0_HIGH          0x1040
#define ADDR0_LOW           0x1044
#define ADDR1_HIGH          0x1048

#define BUS_MODE_SWR        (1u << 0)
#define BUS_MODE_ATDS       (1u << 7)

#define ST_TI               (1u << 0)
#define ST_TU               (1u << 2)
#define ST_RI               (1u << 6)
#define ST_RU               (1u << 7)
#define ST_ETI              (1u << 10)
#define ST_FBI              (1u << 13)
#define ST_NIS              (1u << 16)
#define ST_RS(v)            (((v) >> 17) & 7)
#define ST_TS(v)            (((v) >> 20) & 7)
#define ST_EB(v)            (((v) >> 23) & 7)

#define IE_TIE              (1u << 0)
#define IE_RIE              (1u << 6)
#define IE_NIE              (1u << 16)

#define OP_SR               (1u << 1)
#define OP_ST               (1u << 13)

#define CFG_RE              (1u << 2)
#define CFG_TE              (1u << 3)
#define CFG_DM              (1u << 11)
#define CFG_LM              (1u << 12)
#define CFG_FES             (1u << 14)
#define CFG_PS              (1u << 15)

#define FF_PR               (1u << 0)

#define MII_GB              (1u << 0)
#define MII_GW              (1u << 1)
#define MII_REG(r)          ((r) << 6)
#define MII_PHY(p)          ((p) << 11)

#define EX_CLK_EXT_EN       (1u << 0)
#define EX_PHY_RMII         (4u << 13)

#define TDES0_OWN           (1u << 31)
#define TDES0_IC            (1u << 30)
#define TDES0_LS            (1u << 29)
#define TDES0_FS            (1u << 28)
#define TDES0_TCH           (1u << 20)
#define TDES0_STATUS        0x0003ffffu

#define RDES0_OWN           (1u << 31)
#define RDES0_FL(v)         (((v) >> 16) & 0x3fff)
#define RDES0_ES            (1u << 15)
#define RDES0_FS            (1u << 9)
#define RDES0_LS            (1u << 8)
#define RDES0_FT            (1u << 5)
#define RDES1_RCH           (1u << 14)
#define RDES4_UDP           1
#define RDES4_IPPE          (1u << 4)
#define RDES4_IPV4          (1u << 6)

#define CFG_IPC             (1u << 10)
#define OP_DT               (1u << 26)
#define TDES0_CIC_FULL      (3u << 22)

/* DMA-capable SRAM for descriptors and buffers */
#define RAM                 0x3ffc0000
#define TX_DESC             (RAM + 0x000)
#define RX_DESC             (RAM + 0x100)
#define TX_BUF              (RAM + 0x1000)
#define RX_BUF              (RAM + 0x2000)
#define RX_BUF_SIZE         1536

#define US                  1000
#define MS                  (1000 * US)

/* The Ethernet FCS: CRC-32, reflected, as the wire carries it LSB first */
static uint32_t eth_fcs(const uint8_t *p, size_t len)
{
    uint32_t crc = 0xffffffff;

    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }
    return ~crc;
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, EMAC + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, EMAC + off, v);
}

/* Give the MAC its RMII reference clock and finish a software reset. */
static void emac_setup(QTestState *qts)
{
    wr(qts, EX_PHYINF_CONF, EX_PHY_RMII);
    wr(qts, EX_CLK_CTRL, EX_CLK_EXT_EN);
    wr(qts, BUS_MODE, BUS_MODE_SWR);
    g_assert_cmphex(rd(qts, BUS_MODE) & BUS_MODE_SWR, ==, 0);
    wr(qts, BUS_MODE, BUS_MODE_ATDS | (1u << 8));
}

static uint16_t mdio(QTestState *qts, unsigned phy, unsigned reg,
                     bool write, uint16_t val)
{
    if (write) {
        wr(qts, MII_DATA, val);
    }
    wr(qts, MII_ADDR, MII_PHY(phy) | MII_REG(reg) | (write ? MII_GW : 0) |
       MII_GB);
    g_assert_true(rd(qts, MII_ADDR) & MII_GB);
    qtest_clock_step(qts, 200 * US);
    g_assert_false(rd(qts, MII_ADDR) & MII_GB);
    return rd(qts, MII_DATA);
}

static void put_tx_desc(QTestState *qts, uint32_t at, uint32_t tdes0,
                        uint32_t len, uint32_t buf, uint32_t next)
{
    qtest_writel(qts, at + 4, len);
    qtest_writel(qts, at + 8, buf);
    qtest_writel(qts, at + 12, next);
    qtest_writel(qts, at, tdes0);
}

static void put_rx_desc(QTestState *qts, uint32_t at, uint32_t buf,
                        uint32_t next)
{
    qtest_writel(qts, at + 4, RDES1_RCH | RX_BUF_SIZE);
    qtest_writel(qts, at + 8, buf);
    qtest_writel(qts, at + 12, next);
    qtest_writel(qts, at + 16, 0);
    qtest_writel(qts, at, RDES0_OWN);
}

static void make_frame(uint8_t *f, size_t len, const uint8_t *da)
{
    static const uint8_t sa[6] = { 0x02, 0, 0, 0, 0, 0x01 };

    memcpy(f, da, 6);
    memcpy(f + 6, sa, 6);
    f[12] = 0x88;
    f[13] = 0xb5;
    for (size_t i = 14; i < len; i++) {
        f[i] = i * 7;
    }
}

static const uint8_t station[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
static const uint8_t other[6] = { 0x02, 0x99, 0x99, 0x99, 0x99, 0x99 };

static void set_station(QTestState *qts)
{
    wr(qts, ADDR0_HIGH, station[4] | (station[5] << 8));
    wr(qts, ADDR0_LOW, ldl_le_p(station));
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_reset_values(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    g_assert_cmphex(rd(qts, BUS_MODE), ==, 0x00020100);
    g_assert_cmphex(rd(qts, STATUS), ==, 0);
    g_assert_cmphex(rd(qts, ADDR0_HIGH), ==, 0x8000ffff);
    g_assert_cmphex(rd(qts, ADDR0_LOW), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, ADDR1_HIGH), ==, 0x0000ffff);
    g_assert_cmphex(rd(qts, EX_CLKOUT_CONF), ==, 0x24);
    g_assert_cmphex(rd(qts, EX_OSCCLK_CONF), ==, 0x1253);
    g_assert_cmphex(rd(qts, EX_DATE), ==, 0x16042200);
    g_assert_cmphex(rd(qts, LPI_TIMERS), ==, 0x03e80000);
    /* The APB alias reaches the same registers. */
    g_assert_cmphex(qtest_readl(qts, EMAC_APB + EX_DATE), ==, 0x16042200);

    /* Address 0's enable bit is read-only; base addresses are aligned. */
    wr(qts, ADDR0_HIGH, 0x1234);
    g_assert_cmphex(rd(qts, ADDR0_HIGH), ==, 0x80001234);
    wr(qts, TX_BASE, RAM + 3);
    g_assert_cmphex(rd(qts, TX_BASE), ==, RAM);
    g_assert_cmphex(rd(qts, TX_CUR_DESC), ==, RAM);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_gate_and_sw_reset(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    uint32_t clk = qtest_readl(qts, DPORT_WIFI_CLK_EN);

    /* Held in reset by DPORT, the EMAC drops writes. */
    qtest_writel(qts, DPORT_CORE_RST_EN, CORE_RST_EMAC);
    wr(qts, FRAME_FILTER, FF_PR);
    g_assert_cmphex(rd(qts, FRAME_FILTER), ==, 0);
    qtest_writel(qts, DPORT_CORE_RST_EN, 0);
    qtest_writel(qts, DPORT_WIFI_CLK_EN, clk & ~WIFI_CLK_EMAC);
    wr(qts, FRAME_FILTER, FF_PR);
    g_assert_cmphex(rd(qts, FRAME_FILTER), ==, 0);
    qtest_writel(qts, DPORT_WIFI_CLK_EN, clk | WIFI_CLK_EMAC);
    wr(qts, FRAME_FILTER, FF_PR);
    g_assert_cmphex(rd(qts, FRAME_FILTER), ==, FF_PR);

    /* A software reset cannot finish without the MII clocks. */
    wr(qts, BUS_MODE, BUS_MODE_SWR);
    g_assert_cmphex(rd(qts, BUS_MODE), ==, 0x00020101);
    g_assert_cmphex(rd(qts, FRAME_FILTER), ==, 0);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(rd(qts, BUS_MODE) & BUS_MODE_SWR, ==, BUS_MODE_SWR);
    wr(qts, EX_PHYINF_CONF, EX_PHY_RMII);
    g_assert_cmphex(rd(qts, BUS_MODE) & BUS_MODE_SWR, ==, BUS_MODE_SWR);
    wr(qts, EX_CLK_CTRL, EX_CLK_EXT_EN);
    g_assert_cmphex(rd(qts, BUS_MODE), ==, 0x00020100);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_mdio_phy(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    emac_setup(qts);
    /* The IP101GR at address 1; nothing answers at address 2. */
    g_assert_cmphex(mdio(qts, 1, 2, false, 0), ==, 0x0243);
    g_assert_cmphex(mdio(qts, 1, 3, false, 0), ==, 0x0c54);
    g_assert_cmphex(mdio(qts, 2, 2, false, 0), ==, 0xffff);
    g_assert_cmphex(mdio(qts, 1, 0, false, 0), ==, 0x3100);
    /* No cable: no link, no negotiation. */
    g_assert_cmphex(mdio(qts, 1, 1, false, 0) & 0x24, ==, 0);
    g_assert_cmphex(mdio(qts, 1, 30, false, 0) & 0x100, ==, 0);

    /* The management frame is 64 MDC cycles of APB_CLK / 42. */
    wr(qts, MII_ADDR, MII_PHY(1) | MII_REG(2) | MII_GB);
    qtest_clock_step(qts, 10 * US);
    g_assert_true(rd(qts, MII_ADDR) & MII_GB);
    qtest_clock_step(qts, 200 * US);
    g_assert_false(rd(qts, MII_ADDR) & MII_GB);

    /* Writes reach the PHY; a BMCR reset restores its defaults. */
    mdio(qts, 1, 4, true, 0x0061);
    g_assert_cmphex(mdio(qts, 1, 4, false, 0), ==, 0x0061);
    mdio(qts, 1, 0, true, 0x8000);
    g_assert_cmphex(mdio(qts, 1, 0, false, 0), ==, 0x3100);
    g_assert_cmphex(mdio(qts, 1, 4, false, 0), ==, 0x05e1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_link_up(void)
{
    int fd[2];
    QTestState *qts;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32 -nic socket,fd=%d", fd[1]);
    emac_setup(qts);
    /* BMSR's link bit is latched low: the first read may show it down. */
    mdio(qts, 1, 1, false, 0);
    g_assert_cmphex(mdio(qts, 1, 1, false, 0) & 0x24, ==, 0x24);
    /* Negotiated 100BASE-TX full duplex, partner acknowledged. */
    g_assert_cmphex(mdio(qts, 1, 30, false, 0) & 0x107, ==, 0x106);
    g_assert_cmphex(mdio(qts, 1, 5, false, 0) & 0x4100, ==, 0x4100);
    qtest_quit(qts);
    close(fd[0]);
}

static void start_mac(QTestState *qts, uint32_t extra_cfg)
{
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | extra_cfg);
    wr(qts, OP_MODE, OP_ST | OP_SR);
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | CFG_RE | extra_cfg);
}

static size_t sock_recv_frame(int fd, uint8_t *buf, size_t max)
{
    uint32_t len_be;
    size_t len;

    g_assert_cmpint(recv(fd, &len_be, 4, MSG_WAITALL), ==, 4);
    len = be32_to_cpu(len_be);
    g_assert_cmpuint(len, <=, max);
    g_assert_cmpint(recv(fd, buf, len, MSG_WAITALL), ==, len);
    return len;
}

static void sock_send_frame(int fd, const uint8_t *buf, size_t len)
{
    uint32_t len_be = cpu_to_be32(len);

    g_assert_cmpint(send(fd, &len_be, 4, 0), ==, 4);
    g_assert_cmpint(send(fd, buf, len, 0), ==, len);
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_tx(void)
{
    int fd[2];
    QTestState *qts;
    uint8_t frame[100], got[2048];
    size_t n;
    uint32_t tdes0;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32 -nic socket,fd=%d", fd[1]);
    qtest_irq_intercept_out_named(qts, "/machine/soc/emac", "sysbus-irq");
    emac_setup(qts);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, RX_BASE, RX_DESC);
    wr(qts, INT_EN, IE_NIE | IE_TIE);

    /* A short frame across two chained descriptors. */
    make_frame(frame, 42, other);
    qtest_memwrite(qts, TX_BUF, frame, 20);
    qtest_memwrite(qts, TX_BUF + 0x100, frame + 20, 22);
    put_tx_desc(qts, TX_DESC, TDES0_OWN | TDES0_FS | TDES0_TCH, 20, TX_BUF,
                TX_DESC + 32);
    put_tx_desc(qts, TX_DESC + 32,
                TDES0_OWN | TDES0_LS | TDES0_IC | TDES0_TCH, 22,
                TX_BUF + 0x100, TX_DESC + 64);
    put_tx_desc(qts, TX_DESC + 64, TDES0_TCH, 0, 0, TX_DESC);
    start_mac(qts, 0);

    /* The frame is on the wire for 84 byte times at 100 Mbit/s. */
    g_assert_cmphex(rd(qts, DEBUG) & (3u << 17), ==, 3u << 17);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(rd(qts, DEBUG) & (3u << 17), ==, 0);
    g_assert_true(qtest_get_irq(qts, 0));

    n = sock_recv_frame(fd[0], got, sizeof(got));
    g_assert_cmpuint(n, ==, 60);
    g_assert_cmpmem(got, 42, frame, 42);
    for (size_t i = 42; i < 60; i++) {
        g_assert_cmpuint(got[i], ==, 0);
    }

    tdes0 = qtest_readl(qts, TX_DESC);
    g_assert_cmphex(tdes0 & TDES0_OWN, ==, 0);
    tdes0 = qtest_readl(qts, TX_DESC + 32);
    g_assert_cmphex(tdes0 & (TDES0_OWN | TDES0_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, STATUS) & (ST_TI | ST_TU | ST_ETI | ST_NIS), ==,
                    ST_TI | ST_TU | ST_ETI | ST_NIS);
    /* Suspended at the host-owned third descriptor. */
    g_assert_cmpuint(ST_TS(rd(qts, STATUS)), ==, 6);
    g_assert_cmphex(rd(qts, TX_CUR_DESC), ==, TX_DESC + 64);
    wr(qts, STATUS, 0x1ffff);
    g_assert_false(qtest_get_irq(qts, 0));

    /* A poll demand resumes at the descriptor the host hands back. */
    qtest_memwrite(qts, TX_BUF, frame, 42);
    put_tx_desc(qts, TX_DESC + 64,
                TDES0_OWN | TDES0_FS | TDES0_LS | TDES0_IC | TDES0_TCH, 42,
                TX_BUF, TX_DESC);
    wr(qts, TX_POLL, 0);
    qtest_clock_step(qts, 10 * US);
    n = sock_recv_frame(fd[0], got, sizeof(got));
    g_assert_cmpuint(n, ==, 60);
    g_assert_true(rd(qts, STATUS) & ST_TI);

    qtest_quit(qts);
    close(fd[0]);
}

static void rx_ring(QTestState *qts, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        put_rx_desc(qts, RX_DESC + 32 * i, RX_BUF + RX_BUF_SIZE * i,
                    RX_DESC + 32 * ((i + 1) % n));
    }
}

static bool wait_rx(QTestState *qts, uint32_t desc)
{
    for (int i = 0; i < 100; i++) {
        if (!(qtest_readl(qts, desc) & RDES0_OWN)) {
            return true;
        }
        qtest_clock_step(qts, 100 * US);
    }
    return false;
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_rx(void)
{
    int fd[2];
    QTestState *qts;
    uint8_t frame[200], got[200];
    uint32_t rdes0;
    uint32_t fcs;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32 -nic socket,fd=%d", fd[1]);
    qtest_irq_intercept_out_named(qts, "/machine/soc/emac", "sysbus-irq");
    emac_setup(qts);
    set_station(qts);
    rx_ring(qts, 2);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, RX_BASE, RX_DESC);
    wr(qts, INT_EN, IE_NIE | IE_RIE);
    start_mac(qts, 0);

    /* Not for us: the perfect filter drops it. */
    make_frame(frame, 100, other);
    sock_send_frame(fd[0], frame, 100);
    g_assert_false(wait_rx(qts, RX_DESC));

    /* For us: the frame and its FCS land in the first descriptor. */
    make_frame(frame, 100, station);
    sock_send_frame(fd[0], frame, 100);
    g_assert_true(wait_rx(qts, RX_DESC));
    rdes0 = qtest_readl(qts, RX_DESC);
    g_assert_cmpuint(RDES0_FL(rdes0), ==, 104);
    g_assert_cmphex(rdes0 & (RDES0_FS | RDES0_LS | RDES0_ES | RDES0_FT), ==,
                    RDES0_FS | RDES0_LS | RDES0_FT);
    qtest_memread(qts, RX_BUF, got, 104);
    g_assert_cmpmem(got, 100, frame, 100);
    fcs = eth_fcs(frame, 100);
    g_assert_cmphex(ldl_le_p(got + 100), ==, fcs);
    g_assert_true(rd(qts, STATUS) & ST_RI);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, RX_CUR_DESC), ==, RX_DESC + 32);

    /* A short frame arrives padded to the minimum size. */
    make_frame(frame, 30, station);
    sock_send_frame(fd[0], frame, 30);
    g_assert_true(wait_rx(qts, RX_DESC + 32));
    g_assert_cmpuint(RDES0_FL(qtest_readl(qts, RX_DESC + 32)), ==, 64);

    /* Both descriptors are the host's: the next frame is missed. */
    wr(qts, STATUS, 0x1ffff);
    sock_send_frame(fd[0], frame, 30);
    for (int i = 0; i < 20 && !(rd(qts, STATUS) & ST_RU); i++) {
        qtest_clock_step(qts, 100 * US);
    }
    g_assert_true(rd(qts, STATUS) & ST_RU);
    g_assert_cmpuint(ST_RS(rd(qts, STATUS)), ==, 4);
    g_assert_cmphex(rd(qts, MISSED), ==, 1);
    g_assert_cmphex(rd(qts, MISSED), ==, 0);

    /* Promiscuous mode passes anything once a descriptor is back. */
    wr(qts, FRAME_FILTER, FF_PR);
    put_rx_desc(qts, RX_DESC, RX_BUF, RX_DESC + 32);
    wr(qts, RX_POLL, 0);
    make_frame(frame, 80, other);
    sock_send_frame(fd[0], frame, 80);
    g_assert_true(wait_rx(qts, RX_DESC));
    g_assert_cmpuint(RDES0_FL(qtest_readl(qts, RX_DESC)), ==, 84);

    qtest_quit(qts);
    close(fd[0]);
}

static uint16_t csum_fold(uint32_t sum)
{
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}

static uint32_t csum_add(const uint8_t *p, size_t len)
{
    uint32_t sum = 0;

    for (size_t i = 0; i < len; i += 2) {
        sum += (p[i] << 8) | (i + 1 < len ? p[i + 1] : 0);
    }
    return sum;
}

/* An IPv4 UDP frame of 62 bytes; with sums, both checksums filled in */
static void make_udp(uint8_t *f, const uint8_t *da, bool sums)
{
    uint8_t *ip = f + 14, *udp = ip + 20;
    uint32_t sum;

    make_frame(f, 62, da);
    memset(ip, 0, 28);
    f[12] = 0x08;
    f[13] = 0x00;
    ip[0] = 0x45;
    stw_be_p(ip + 2, 48);
    ip[8] = 64;
    ip[9] = 17;
    stl_be_p(ip + 12, 0x0a000001);
    stl_be_p(ip + 16, 0x0a000002);
    stw_be_p(udp, 1234);
    stw_be_p(udp + 2, 5678);
    stw_be_p(udp + 4, 28);
    stw_be_p(ip + 10, 0);
    stw_be_p(udp + 6, 0);
    if (sums) {
        stw_be_p(ip + 10, csum_fold(csum_add(ip, 20)));
        sum = csum_add(ip + 12, 8) + 17 + 28 + csum_add(udp, 28);
        stw_be_p(udp + 6, csum_fold(sum));
    }
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_checksum_offload(void)
{
    int fd[2];
    QTestState *qts;
    uint8_t frame[64], want[64], got[2048];
    uint32_t rdes0;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32 -nic socket,fd=%d", fd[1]);
    emac_setup(qts);
    set_station(qts);
    rx_ring(qts, 2);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, RX_BASE, RX_DESC);
    start_mac(qts, CFG_IPC);

    /* TDES0.CIC = 3: the MAC fills in the IPv4 and UDP checksums. */
    make_udp(frame, other, false);
    make_udp(want, other, true);
    qtest_memwrite(qts, TX_BUF, frame, 62);
    put_tx_desc(qts, TX_DESC, TDES0_OWN | TDES0_FS | TDES0_LS | TDES0_TCH |
                TDES0_CIC_FULL, 62, TX_BUF, TX_DESC + 0x80);
    wr(qts, TX_POLL, 0);
    qtest_clock_step(qts, 20 * US);
    g_assert_cmpuint(sock_recv_frame(fd[0], got, sizeof(got)), ==, 62);
    g_assert_cmpmem(got, 62, want, 62);
    g_assert_cmphex(qtest_readl(qts, TX_DESC) & TDES0_STATUS, ==, 0);

    /* A good datagram: RDES4 reports IPv4 UDP and no errors. */
    make_udp(frame, station, true);
    sock_send_frame(fd[0], frame, 62);
    g_assert_true(wait_rx(qts, RX_DESC));
    rdes0 = qtest_readl(qts, RX_DESC);
    g_assert_cmphex(rdes0 & (RDES0_ES | 1), ==, 1);
    g_assert_cmphex(qtest_readl(qts, RX_DESC + 16), ==,
                    RDES4_IPV4 | RDES4_UDP);

    /* A bad UDP checksum: dropped while DT is clear... */
    frame[14 + 20 + 6] ^= 0x55;
    sock_send_frame(fd[0], frame, 62);
    g_assert_false(wait_rx(qts, RX_DESC + 32));

    /* ...and delivered with IPPE and ES once DT is set. */
    wr(qts, OP_MODE, OP_ST | OP_SR | OP_DT);
    sock_send_frame(fd[0], frame, 62);
    g_assert_true(wait_rx(qts, RX_DESC + 32));
    g_assert_true(qtest_readl(qts, RX_DESC + 32) & RDES0_ES);
    g_assert_cmphex(qtest_readl(qts, RX_DESC + 48), ==,
                    RDES4_IPV4 | RDES4_UDP | RDES4_IPPE);

    qtest_quit(qts);
    close(fd[0]);
}

/* [spec:nuos:req:emu.esp32.emac/test] */
static void test_loopback_and_bus_error(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    uint8_t frame[64], got[64];

    emac_setup(qts);
    set_station(qts);
    rx_ring(qts, 2);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, RX_BASE, RX_DESC);

    /* MAC loopback: the transmitted frame comes back with its FCS. */
    make_frame(frame, 60, station);
    qtest_memwrite(qts, TX_BUF, frame, 60);
    put_tx_desc(qts, TX_DESC,
                TDES0_OWN | TDES0_FS | TDES0_LS | TDES0_IC | TDES0_TCH, 60,
                TX_BUF, TX_DESC + 0x80);
    start_mac(qts, CFG_LM);
    g_assert_true(wait_rx(qts, RX_DESC));
    g_assert_cmpuint(RDES0_FL(qtest_readl(qts, RX_DESC)), ==, 64);
    qtest_memread(qts, RX_BUF, got, 60);
    g_assert_cmpmem(got, 60, frame, 60);

    /* A descriptor outside internal SRAM is a fatal bus error. */
    qtest_clock_step(qts, 100 * US);
    wr(qts, OP_MODE, 0);
    wr(qts, TX_BASE, 0x3ff00000);
    wr(qts, OP_MODE, OP_ST);
    g_assert_true(rd(qts, STATUS) & ST_FBI);
    g_assert_cmpuint(ST_EB(rd(qts, STATUS)), ==, 7);
    g_assert_cmpuint(ST_TS(rd(qts, STATUS)), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32-emac/reset-values", test_reset_values);
    qtest_add_func("/esp32-emac/gate-and-sw-reset", test_gate_and_sw_reset);
    qtest_add_func("/esp32-emac/mdio-phy", test_mdio_phy);
    qtest_add_func("/esp32-emac/link-up", test_link_up);
    qtest_add_func("/esp32-emac/tx", test_tx);
    qtest_add_func("/esp32-emac/rx", test_rx);
    qtest_add_func("/esp32-emac/checksum-offload", test_checksum_offload);
    qtest_add_func("/esp32-emac/loopback-and-bus-error",
                   test_loopback_and_bus_error);
    return g_test_run();
}
