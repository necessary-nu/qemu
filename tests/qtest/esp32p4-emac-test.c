/*
 * QTest testcase for the ESP32-P4 Ethernet MAC and the board's IP101 PHY
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The EMAC is set up as ESP-IDF's esp_eth sets it up on the Waveshare
 * ESP32-P4-WIFI6-POE-ETH: its bus clock, RMII with the PHY's 50 MHz
 * reference in on GPIO50, the RMII data on the IO MUX's EMAC functions of
 * GPIO49, 34 and 35 (transmit) and 28 to 30 (receive), MDC and MDIO
 * through the GPIO matrix on GPIO31 and GPIO52. Descriptors and buffers
 * are in L2MEM past the part the L2 cache occupies.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/sockets.h"
#include "libqtest.h"

#define EMAC                0x50098000

/* MAC */
#define CONFIG              0x0000
#define FRAME_FILTER        0x0004
#define MII_ADDR            0x0010
#define MII_DATA            0x0014
#define FLOW_CTRL           0x0018
#define VLAN_TAG            0x001c
#define PMT_CSR             0x002c
#define LPI_TIMERS          0x0034
#define INTS                0x0038
#define INT_MASK            0x003c
#define ADDR_HIGH(n)        (0x0040 + 8 * (n))
#define ADDR_LOW(n)         (0x0044 + 8 * (n))
#define TX_VLAN             0x0584
#define TS_CTRL             0x0700
#define TS_SSINC            0x0704
#define TS_SEC              0x0708
#define TS_SUBSEC           0x070c
#define TS_SEC_UPD          0x0710
#define TS_SUBSEC_UPD       0x0714
#define TS_ADDEND           0x0718
#define TS_TARGET_SEC       0x071c
#define TS_TARGET_SUBSEC    0x0720
#define TS_SEC_HI           0x0724
#define TS_STATUS           0x0728
#define PPS_CTRL            0x072c
#define HW_FEATURE          0x1058
#define CH1_BUS_MODE        0x1100
/* DMA */
#define BUS_MODE            0x1000
#define TX_POLL             0x1004
#define RX_POLL             0x1008
#define RX_BASE             0x100c
#define TX_BASE             0x1010
#define STATUS              0x1014
#define OP_MODE             0x1018
#define INT_EN              0x101c
#define AHB_STATUS          0x102c
#define TX_CUR_DESC         0x1048

#define BUS_MODE_SWR        (1u << 0)
#define BUS_MODE_ATDS       (1u << 7)
#define BUS_MODE_PBL(n)     ((n) << 8)

#define ST_TI               (1u << 0)
#define ST_TU               (1u << 2)
#define ST_RI               (1u << 6)
#define ST_FBI              (1u << 13)
#define ST_NIS              (1u << 16)
#define ST_TS(v)            (((v) >> 20) & 7)
#define ST_EB(v)            (((v) >> 23) & 7)
#define ST_TTI              (1u << 29)

#define IE_TIE              (1u << 0)
#define IE_RIE              (1u << 6)
#define IE_NIE              (1u << 16)

#define OP_SR               (1u << 1)
#define OP_ST               (1u << 13)

#define CFG_RE              (1u << 2)
#define CFG_TE              (1u << 3)
#define CFG_DM              (1u << 11)
#define CFG_FES             (1u << 14)
#define CFG_PS              (1u << 15)
#define CFG_CST             (1u << 25)

#define FF_VTFE             (1u << 16)
#define FC_TFE              (1u << 1)
#define FC_DZPQ             (1u << 7)
#define FC_PT(q)            ((q) << 16)
#define VLAN_TAG_ETV        (1u << 16)
#define PMT_PWRDWN          (1u << 0)
#define PMT_MGKPKTEN        (1u << 1)
#define PMT_MGKPRCVD        (1u << 5)

#define MII_GB              (1u << 0)
#define MII_GW              (1u << 1)
#define MII_CR(c)           ((c) << 2)
#define MII_REG(r)          ((r) << 6)
#define MII_PHY(p)          ((p) << 11)

#define TSC_TSENA           (1u << 0)
#define TSC_TSCFUPDT        (1u << 1)
#define TSC_TSINIT          (1u << 2)
#define TSC_TSUPDT          (1u << 3)
#define TSC_TSTRIG          (1u << 4)
#define TSC_TSADDREG        (1u << 5)
#define TSC_TSCTRLSSR       (1u << 9)
#define TSC_TSVER2ENA       (1u << 10)
#define TSC_TSIPENA         (1u << 11)
#define TSC_TSIPV4ENA       (1u << 13)
#define TSC_SNAPTYPSEL(n)   ((n) << 16)
#define TSS_TSSOVF          (1u << 0)
#define TSS_TSTARGT         (1u << 1)
#define TSS_TSTRGTERR       (1u << 3)
#define INTS_TS             (1u << 9)
#define SUBSEC_ADDSUB       (1u << 31)

#define TDES0_OWN           (1u << 31)
#define TDES0_IC            (1u << 30)
#define TDES0_LS            (1u << 29)
#define TDES0_FS            (1u << 28)
#define TDES0_TTSE          (1u << 25)
#define TDES0_TCH           (1u << 20)
#define TDES0_VLIC(v)       ((v) << 18)
#define TDES0_TTSS          (1u << 17)
#define TDES0_STATUS        0x0003ffffu

#define RDES0_OWN           (1u << 31)
#define RDES0_FL(v)         (((v) >> 16) & 0x3fff)
#define RDES0_ES            (1u << 15)
#define RDES0_VLAN          (1u << 10)
#define RDES0_FS            (1u << 9)
#define RDES0_LS            (1u << 8)
#define RDES0_TSA           (1u << 7)
#define RDES0_ESA           (1u << 0)
#define RDES1_RCH           (1u << 14)
#define RDES4_PTP_MSG(v)    (((v) >> 8) & 0xf)
#define RDES4_PTP_ETH       (1u << 12)
#define RDES4_PTP_V2        (1u << 13)

/* HP_SYS_CLKRST, LP_CLKRST and HP_SYSTEM */
#define SOC_CLK_CTRL1       0x500e6018
#define EMAC_SYS_CLK_EN     (1u << 13)
#define PERI_CLK_CTRL00     0x500e6030
#define PERI_CLK_CTRL01     0x500e6034
#define LP_RST_CTRL         0x5011104c
#define RST_EN_EMAC         (1u << 30)
#define GMAC_CTRL0          0x500e514c
#define GMAC_CTRL1          0x500e5150
#define GMAC_CTRL2          0x500e5154
#define GMAC_PTP_PPS        (1u << 0)
#define GMAC_SBD_FLOWCTRL   (1u << 1)
#define GMAC_PHY_RMII       (4u << 2)
#define GMAC_RST_TX_N       (1u << 6)
#define GMAC_RST_RX_N       (1u << 7)

/* The GPIO matrix and IO MUX */
#define GPIO                0x500e0000
#define GPIO_ENABLE1_W1TS   (GPIO + 0x030)
#define GPIO_OUT1_W1TC      (GPIO + 0x018)
#define GPIO_OUT1_W1TS      (GPIO + 0x014)
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IN_CONST_LOW        62
#define SIG_GPIO_OUT        0x100
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define FUN_DRV2            (2u << 10)
#define MCU_SEL(f)          ((f) << 12)

#define SIG_MII_MDI         107
#define SIG_MII_MDC         108
#define SIG_MII_MDO         109

/* The DMA APM: the GMAC's read permissions for the 32 regions */
#define DMA_PMS_GMAC_R      0x500a61b4

/* The interrupt matrix's status, word 2: ETH_MAC_INTR, source 92 */
#define INTMTX_STATUS2      0x500d6208
#define SRC_ETH_MAC_BIT     (1u << (92 - 64))
#define SRC_PMT_BIT         (1u << (91 - 64))

/* DMA-reachable L2MEM, past the L2 cache's 128 KiB */
#define RAM                 0x4ff40000
#define TX_DESC             (RAM + 0x000)
#define RX_DESC             (RAM + 0x100)
#define TX_BUF              (RAM + 0x1000)
#define RX_BUF              (RAM + 0x2000)
#define RX_BUF_SIZE         1536

#define PHY_ADDR            1

#define US                  1000
#define MS                  (1000 * US)

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, EMAC + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, EMAC + off, v);
}

static void set_bits(QTestState *qts, uint32_t addr, uint32_t clear,
                     uint32_t set)
{
    qtest_writel(qts, addr, (qtest_readl(qts, addr) & ~clear) | set);
}

/*
 * ESP-IDF's emac_ll_clock_enable_rmii_input(): RMII, the reference from
 * PAD_EMAC_TXRX_CLK for EMAC_RMII_CLK and, divided by 2, for EMAC_RX_CLK
 * and EMAC_TX_CLK; with the IO MUX taking the reference from GPIO50.
 */
static void rmii_clocks(QTestState *qts, bool ref_pad)
{
    qtest_writel(qts, GMAC_CTRL0, GMAC_PHY_RMII);
    if (ref_pad) {
        qtest_writel(qts, IO_MUX_GPIO(50), MCU_SEL(3) | FUN_IE | FUN_DRV2);
    }
    set_bits(qts, PERI_CLK_CTRL00, (3u << 25) | (1u << 28),
             (1u << 27) | (1u << 29));
    set_bits(qts, PERI_CLK_CTRL01, 0xff | (1u << 8) | (0xffu << 10),
             1 | (1u << 9) | (1u << 10));
}

/* The bus clock and a software reset, as emac_hal_reset() does. */
static void emac_setup(QTestState *qts)
{
    set_bits(qts, SOC_CLK_CTRL1, 0, EMAC_SYS_CLK_EN);
    rmii_clocks(qts, true);
    wr(qts, BUS_MODE, BUS_MODE_SWR);
    g_assert_cmphex(rd(qts, BUS_MODE) & BUS_MODE_SWR, ==, 0);
    wr(qts, BUS_MODE, BUS_MODE_ATDS | BUS_MODE_PBL(8));
}

/* emac_esp_gpio_init_smi(): MDC on GPIO31, MDIO on GPIO52 */
static void smi_pins(QTestState *qts)
{
    qtest_writel(qts, IO_MUX_GPIO(31), MCU_SEL(1) | FUN_DRV2);
    qtest_writel(qts, GPIO_FUNC_OUT(31), SIG_MII_MDC);
    qtest_writel(qts, IO_MUX_GPIO(52), MCU_SEL(1) | FUN_IE | FUN_DRV2);
    qtest_writel(qts, GPIO_FUNC_OUT(52), SIG_MII_MDO);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_MII_MDI), IN_SIG_IN_SEL | 52);
}

/* The RMII data on the IO MUX's EMAC functions */
static void rmii_pins(QTestState *qts)
{
    static const int tx[] = { 49, 34, 35 };
    static const int rx[] = { 28, 29, 30 };

    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, IO_MUX_GPIO(tx[i]), MCU_SEL(3) | FUN_DRV2);
        qtest_writel(qts, IO_MUX_GPIO(rx[i]), MCU_SEL(3) | FUN_IE | FUN_DRV2);
        qtest_writel(qts, GPIO_FUNC_IN(178 + i), 0);
    }
}

static uint16_t mdio(QTestState *qts, unsigned phy, unsigned reg,
                     bool write, uint16_t val)
{
    if (write) {
        wr(qts, MII_DATA, val);
    }
    wr(qts, MII_ADDR, MII_PHY(phy) | MII_REG(reg) | MII_CR(3) |
       (write ? MII_GW : 0) | MII_GB);
    g_assert_true(rd(qts, MII_ADDR) & MII_GB);
    qtest_clock_step(qts, 1 * MS);
    g_assert_false(rd(qts, MII_ADDR) & MII_GB);
    return rd(qts, MII_DATA);
}

static void put_tx_desc(QTestState *qts, uint32_t at, uint32_t tdes0,
                        uint32_t len, uint32_t buf, uint32_t next)
{
    qtest_writel(qts, at + 4, len);
    qtest_writel(qts, at + 8, buf);
    qtest_writel(qts, at + 12, next);
    qtest_writel(qts, at + 24, 0);
    qtest_writel(qts, at + 28, 0);
    qtest_writel(qts, at, tdes0);
}

static void put_rx_desc(QTestState *qts, uint32_t at, uint32_t buf,
                        uint32_t next)
{
    qtest_writel(qts, at + 4, RDES1_RCH | RX_BUF_SIZE);
    qtest_writel(qts, at + 8, buf);
    qtest_writel(qts, at + 12, next);
    for (int i = 16; i < 32; i += 4) {
        qtest_writel(qts, at + i, 0);
    }
    qtest_writel(qts, at, RDES0_OWN);
}

static const uint8_t station[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };

static void make_frame(uint8_t *f, size_t len, const uint8_t *da,
                       uint16_t type)
{
    static const uint8_t sa[6] = { 0x02, 0, 0, 0, 0, 0x01 };

    memcpy(f, da, 6);
    memcpy(f + 6, sa, 6);
    stw_be_p(f + 12, type);
    for (size_t i = 14; i < len; i++) {
        f[i] = i * 7;
    }
}

static void set_station(QTestState *qts)
{
    wr(qts, ADDR_HIGH(0), station[4] | (station[5] << 8));
    wr(qts, ADDR_LOW(0), ldl_le_p(station));
}

/* Send a frame to the EMAC over the socket backend's stream protocol. */
static void sock_send(int fd, const uint8_t *f, size_t len)
{
    uint32_t be = cpu_to_be32(len);

    g_assert_cmpint(send(fd, &be, 4, 0), ==, 4);
    g_assert_cmpint(send(fd, f, len, 0), ==, len);
}

/* Receive a frame the EMAC sent, or return 0 if there is none. */
static size_t sock_recv(int fd, uint8_t *f, size_t max)
{
    uint32_t be;
    size_t len;

    if (recv(fd, &be, 4, MSG_DONTWAIT) != 4) {
        return 0;
    }
    len = be32_to_cpu(be);
    g_assert_cmpint(len, <=, max);
    g_assert_cmpint(recv(fd, f, len, MSG_WAITALL), ==, len);
    return len;
}

/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_reset_values(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -nic none");

    /* SW_RST reads 1 until the MII clocks let the reset finish. */
    g_assert_cmphex(rd(qts, BUS_MODE), ==, 0x00020101);
    g_assert_cmphex(rd(qts, CONFIG), ==, CFG_PS);
    g_assert_cmphex(rd(qts, FRAME_FILTER), ==, 0x00010000);
    g_assert_cmphex(rd(qts, ADDR_HIGH(0)), ==, 0x8000ffff);
    g_assert_cmphex(rd(qts, ADDR_LOW(0)), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, ADDR_HIGH(8)), ==, 0x0000ffff);
    g_assert_cmphex(rd(qts, ADDR_LOW(8)), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, LPI_TIMERS), ==, 0x03e80000);
    g_assert_cmphex(rd(qts, TS_CTRL), ==, TSC_TSIPV4ENA);
    g_assert_cmphex(rd(qts, AHB_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, STATUS), ==, 0);
    /* Registers the TRM does not document read as zero. */
    g_assert_cmphex(rd(qts, HW_FEATURE), ==, 0);
    g_assert_cmphex(rd(qts, CH1_BUS_MODE), ==, 0);
    /* No clock reaches the transmit or receive domains. */
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0), ==, 0);

    /* With EMAC_SYS_CLK gated, writes do not land. */
    wr(qts, VLAN_TAG, 0x1234);
    g_assert_cmphex(rd(qts, VLAN_TAG), ==, 0);
    set_bits(qts, SOC_CLK_CTRL1, 0, EMAC_SYS_CLK_EN);
    wr(qts, VLAN_TAG, 0xffffffff);
    g_assert_cmphex(rd(qts, VLAN_TAG), ==, 0x0007ffff);
    wr(qts, TX_VLAN, 0xffffffff);
    g_assert_cmphex(rd(qts, TX_VLAN), ==, 0x000fffff);
    wr(qts, INT_MASK, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_MASK), ==, 0x608);
    wr(qts, ADDR_HIGH(8), 0xffffffff);
    g_assert_cmphex(rd(qts, ADDR_HIGH(8)), ==, 0xff00ffff);
    qtest_quit(qts);
}

/*
 * The software reset finishes only once the MII clocks reach the MAC:
 * in RMII mode the PHY's reference, through GPIO50's IO MUX, then the
 * clock tree. LP_CLKRST_RST_EN_EMAC holds the whole block in reset.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_reset_and_clocks(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -nic none");

    set_bits(qts, SOC_CLK_CTRL1, 0, EMAC_SYS_CLK_EN);
    rmii_clocks(qts, false);
    wr(qts, BUS_MODE, BUS_MODE_SWR);
    g_assert_cmphex(rd(qts, BUS_MODE) & BUS_MODE_SWR, ==, BUS_MODE_SWR);
    /* A pending reset ignores other writes. */
    wr(qts, BUS_MODE, BUS_MODE_ATDS);
    g_assert_cmphex(rd(qts, BUS_MODE), ==, 0x00020101);
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0) &
                    (GMAC_RST_TX_N | GMAC_RST_RX_N), ==, 0);

    /* The reference clock comes in once GPIO50 takes EMAC_RMII_CLK. */
    qtest_writel(qts, IO_MUX_GPIO(50), MCU_SEL(3) | FUN_IE | FUN_DRV2);
    g_assert_cmphex(rd(qts, BUS_MODE) & BUS_MODE_SWR, ==, 0);
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0), ==,
                    GMAC_PHY_RMII | GMAC_RST_TX_N | GMAC_RST_RX_N);

    /* Losing the pad stops the clocks the MAC's domains run on. */
    qtest_writel(qts, IO_MUX_GPIO(50), MCU_SEL(1) | FUN_IE | FUN_DRV2);
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0) &
                    (GMAC_RST_TX_N | GMAC_RST_RX_N), ==, 0);
    qtest_writel(qts, IO_MUX_GPIO(50), MCU_SEL(3) | FUN_IE | FUN_DRV2);

    wr(qts, CONFIG, CFG_PS | CFG_DM);
    set_bits(qts, LP_RST_CTRL, 0, RST_EN_EMAC);
    g_assert_cmphex(rd(qts, CONFIG), ==, CFG_PS);
    set_bits(qts, LP_RST_CTRL, RST_EN_EMAC, 0);
    /* Out of reset with its clocks running, the core finishes at once. */
    g_assert_cmphex(rd(qts, BUS_MODE), ==, 0x00020100);
    qtest_quit(qts);
}

/*
 * The IP101 at MDIO address 1, reached through the GPIO matrix: its ID,
 * no answer at another address, and none where the pins are not routed
 * or the PHY is held in reset on GPIO51.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_mdio(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -nic none");

    emac_setup(qts);
    /* MDIO unrouted: the MAC's MDI input sees the matrix's constant 0. */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_MII_MDI), IN_SIG_IN_SEL | IN_CONST_LOW);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 2, false, 0), ==, 0x0000);

    smi_pins(qts);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 2, false, 0), ==, 0x0243);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 3, false, 0), ==, 0x0c54);
    /* Nothing answers at address 0: MDIO's pull-up reads all ones. */
    g_assert_cmphex(mdio(qts, 0, 2, false, 0), ==, 0xffff);

    /* A write reaches the PHY: auto-negotiation advertisement. */
    mdio(qts, PHY_ADDR, 4, true, 0x0061);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 4, false, 0), ==, 0x0061);

    /* MDC on the wrong pad: the frame never reaches the PHY. */
    qtest_writel(qts, GPIO_FUNC_OUT(31), SIG_GPIO_OUT);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 2, false, 0), ==, 0xffff);
    qtest_writel(qts, GPIO_FUNC_OUT(31), SIG_MII_MDC);

    /* RESET_N low on GPIO51 holds the PHY, which loses its settings. */
    qtest_writel(qts, IO_MUX_GPIO(51), MCU_SEL(1) | FUN_DRV2);
    qtest_writel(qts, GPIO_FUNC_OUT(51), SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_OUT1_W1TC, 1u << (51 - 32));
    qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << (51 - 32));
    g_assert_cmphex(mdio(qts, PHY_ADDR, 2, false, 0), ==, 0xffff);
    qtest_writel(qts, GPIO_OUT1_W1TS, 1u << (51 - 32));
    g_assert_cmphex(mdio(qts, PHY_ADDR, 2, false, 0), ==, 0x0243);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 4, false, 0), ==, 0x05e1);
    qtest_quit(qts);
}

/*
 * A frame out through the PHY to the backend and one in from it, with
 * the descriptors handed back and ETH_MAC_INTR raised in the interrupt
 * matrix; and none while a data pin is not routed, or at a line rate
 * that is not the RMII clocks'.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_tx_rx(void)
{
    uint8_t frame[100], got[2048];
    QTestState *qts;
    uint32_t rdes0;
    int fd[2];

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32p4 -nic socket,fd=%d,model=esp32p4-emac", fd[1]);
    emac_setup(qts);
    smi_pins(qts);
    rmii_pins(qts);
    /* The link, latched low until read once */
    mdio(qts, PHY_ADDR, 1, false, 0);
    g_assert_cmphex(mdio(qts, PHY_ADDR, 1, false, 0) & 0x0024, ==, 0x0024);

    set_station(qts);
    put_rx_desc(qts, RX_DESC, RX_BUF, RX_DESC + 32);
    put_rx_desc(qts, RX_DESC + 32, RX_BUF + RX_BUF_SIZE, RX_DESC);
    wr(qts, RX_BASE, RX_DESC);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, INT_EN, IE_NIE | IE_RIE | IE_TIE);
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | CFG_RE);
    wr(qts, OP_MODE, OP_ST | OP_SR);

    make_frame(frame, sizeof(frame), station, 0x88b5);
    qtest_memwrite(qts, TX_BUF, frame, sizeof(frame));
    put_tx_desc(qts, TX_DESC, TDES0_OWN | TDES0_IC | TDES0_FS | TDES0_LS |
                TDES0_TCH, sizeof(frame), TX_BUF, TX_DESC + 32);
    put_tx_desc(qts, TX_DESC + 32, TDES0_TCH, 0, 0, TX_DESC);
    wr(qts, TX_POLL, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(qtest_readl(qts, TX_DESC) &
                    (TDES0_OWN | TDES0_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, STATUS) & (ST_TI | ST_TU), ==, ST_TI | ST_TU);
    g_assert_cmpint(sock_recv(fd[0], got, sizeof(got)), ==, sizeof(frame));
    g_assert_cmpmem(got, sizeof(frame), frame, sizeof(frame));
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_ETH_MAC_BIT, ==,
                    SRC_ETH_MAC_BIT);
    wr(qts, STATUS, 0x1ffff);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_ETH_MAC_BIT, ==,
                    0);

    sock_send(fd[0], frame, sizeof(frame));
    qtest_clock_step(qts, 1 * MS);
    rdes0 = qtest_readl(qts, RX_DESC);
    g_assert_cmphex(rdes0 & (RDES0_OWN | RDES0_FS | RDES0_LS | RDES0_ES), ==,
                    RDES0_FS | RDES0_LS);
    g_assert_cmpint(RDES0_FL(rdes0), ==, sizeof(frame) + 4);
    qtest_memread(qts, RX_BUF, got, sizeof(frame));
    g_assert_cmpmem(got, sizeof(frame), frame, sizeof(frame));
    g_assert_cmphex(rd(qts, STATUS) & ST_RI, ==, ST_RI);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_ETH_MAC_BIT, ==,
                    SRC_ETH_MAC_BIT);
    wr(qts, STATUS, 0x1ffff);

    /* TXD1 off its pad: the frame never reaches the PHY. */
    qtest_writel(qts, IO_MUX_GPIO(35), MCU_SEL(1) | FUN_DRV2);
    put_tx_desc(qts, TX_DESC + 32, TDES0_OWN | TDES0_FS | TDES0_LS |
                TDES0_TCH, sizeof(frame), TX_BUF, TX_DESC);
    put_tx_desc(qts, TX_DESC, TDES0_TCH, 0, 0, TX_DESC + 32);
    wr(qts, TX_POLL, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(qtest_readl(qts, TX_DESC + 32) & TDES0_OWN, ==, 0);
    g_assert_cmpint(sock_recv(fd[0], got, sizeof(got)), ==, 0);
    qtest_writel(qts, IO_MUX_GPIO(35), MCU_SEL(3) | FUN_DRV2);

    /* 10 Mbit/s on 25 MHz RMII clocks: nothing usable crosses. */
    wr(qts, CONFIG, CFG_PS | CFG_DM | CFG_TE | CFG_RE);
    sock_send(fd[0], frame, sizeof(frame));
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(qtest_readl(qts, RX_DESC + 32) & RDES0_OWN, ==,
                    RDES0_OWN);

    qtest_quit(qts);
    close(fd[0]);
}

/*
 * The system time: counting the 40 MHz crystal in coarse and fine
 * update, set, adjusted, read through HP_SYSTEM_GMAC_CTRL1/2, its target
 * time interrupt and a target already passed, the seconds overflow, and
 * the PPS output in HP_SYSTEM_GMAC_CTRL0.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_ptp_time(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -nic none");
    uint32_t ns;

    emac_setup(qts);
    /* EMAC_PTP_REF_CLK from the crystal, 25 ns a cycle */
    set_bits(qts, PERI_CLK_CTRL01, 1u << 18, 1u << 19);
    wr(qts, TS_SSINC, 25);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR);
    wr(qts, TS_SEC_UPD, 5);
    wr(qts, TS_SUBSEC_UPD, 0);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSINIT);
    g_assert_cmphex(rd(qts, TS_CTRL) & TSC_TSINIT, ==, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpuint(rd(qts, TS_SEC), ==, 5);
    ns = rd(qts, TS_SUBSEC);
    g_assert_cmpuint(ns, >=, 1000000);
    g_assert_cmpuint(ns, <=, 1000025);
    g_assert_cmpuint(qtest_readl(qts, GMAC_CTRL2), ==, 5);
    g_assert_cmpuint(qtest_readl(qts, GMAC_CTRL1), >=, ns);

    /* Fine update: half of each cycle's 50 ns, 25 ns a cycle again. */
    wr(qts, TS_SSINC, 50);
    wr(qts, TS_ADDEND, 0x80000000);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT |
       TSC_TSADDREG);
    g_assert_cmphex(rd(qts, TS_CTRL) & TSC_TSADDREG, ==, 0);
    ns = rd(qts, TS_SUBSEC);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmpuint(rd(qts, TS_SUBSEC) - ns, >=, 2000000 - 50);
    g_assert_cmpuint(rd(qts, TS_SUBSEC) - ns, <=, 2000000 + 50);

    /* Subtract 1.5 s. */
    wr(qts, TS_SEC_UPD, 1);
    wr(qts, TS_SUBSEC_UPD, SUBSEC_ADDSUB | 500000000);
    ns = rd(qts, TS_SUBSEC);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT | TSC_TSUPDT);
    g_assert_cmpuint(rd(qts, TS_SEC), ==, 3);
    g_assert_cmpuint(rd(qts, TS_SUBSEC), >=, ns + 500000000);

    /* The target time interrupt, 2 ms on */
    wr(qts, TS_TARGET_SEC, 3);
    wr(qts, TS_TARGET_SUBSEC, rd(qts, TS_SUBSEC) + 2000000);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT | TSC_TSTRIG);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, INTS), ==, 0);
    g_assert_cmphex(rd(qts, TS_CTRL) & TSC_TSTRIG, ==, TSC_TSTRIG);
    qtest_clock_step(qts, 1 * MS + 100);
    g_assert_cmphex(rd(qts, TS_CTRL) & TSC_TSTRIG, ==, 0);
    g_assert_cmphex(rd(qts, INTS), ==, INTS_TS);
    g_assert_cmphex(rd(qts, STATUS) & ST_TTI, ==, ST_TTI);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_ETH_MAC_BIT, ==,
                    SRC_ETH_MAC_BIT);
    g_assert_cmphex(rd(qts, TS_STATUS), ==, TSS_TSTARGT);
    g_assert_cmphex(rd(qts, TS_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, INTS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_ETH_MAC_BIT, ==,
                    0);

    /* A target already passed */
    wr(qts, TS_TARGET_SEC, 2);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT | TSC_TSTRIG);
    g_assert_cmphex(rd(qts, TS_STATUS), ==, TSS_TSTRGTERR | TSS_TSTARGT);

    /* The seconds wrap into the high word. */
    wr(qts, TS_SEC_UPD, 0xffffffff);
    wr(qts, TS_SUBSEC_UPD, 999999000);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT | TSC_TSINIT);
    g_assert_cmphex(rd(qts, TS_SEC_HI), ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, TS_SEC), ==, 0);
    g_assert_cmphex(rd(qts, TS_SEC_HI), ==, 1);
    g_assert_cmphex(rd(qts, TS_STATUS), ==, TSS_TSSOVF);

    /* PPS at 2 Hz: high in the first quarter second, low in the next. */
    wr(qts, PPS_CTRL, 1);
    wr(qts, TS_SEC_UPD, 7);
    wr(qts, TS_SUBSEC_UPD, 100000000);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT | TSC_TSINIT);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0) & GMAC_PTP_PPS, ==,
                    GMAC_PTP_PPS);
    qtest_clock_step(qts, 200 * MS);
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0) & GMAC_PTP_PPS, ==, 0);
    qtest_clock_step(qts, 250 * MS);
    g_assert_cmphex(qtest_readl(qts, GMAC_CTRL0) & GMAC_PTP_PPS, ==,
                    GMAC_PTP_PPS);

    /* With the PTP clock stopped the time stands, and TSINIT waits. */
    set_bits(qts, PERI_CLK_CTRL01, 1u << 19, 0);
    ns = rd(qts, TS_SUBSEC);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpuint(rd(qts, TS_SUBSEC), ==, ns);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSCFUPDT | TSC_TSINIT);
    g_assert_cmphex(rd(qts, TS_CTRL) & TSC_TSINIT, ==, TSC_TSINIT);
    set_bits(qts, PERI_CLK_CTRL01, 0, 1u << 19);
    g_assert_cmphex(rd(qts, TS_CTRL) & TSC_TSINIT, ==, 0);
    qtest_quit(qts);
}

/*
 * Timestamps: a PTP Sync over Ethernet received with its RDES4 message
 * status and RDES6/7 timestamp, a frame sent with TDES0.TTSE, and VLAN
 * tag insertion with the tag control register.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_ptp_frames(void)
{
    uint8_t frame[100], got[2048];
    QTestState *qts;
    uint32_t rdes0, rdes4, sec, ns;
    size_t len;
    int fd[2];

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32p4 -nic socket,fd=%d,model=esp32p4-emac", fd[1]);
    emac_setup(qts);
    rmii_pins(qts);
    set_station(qts);
    set_bits(qts, PERI_CLK_CTRL01, 1u << 18, 1u << 19);
    /* emac_hal_ptp_start(): PTP over Ethernet, v2, Sync and the like */
    wr(qts, TS_SSINC, 25);
    wr(qts, TS_SEC_UPD, 100);
    wr(qts, TS_SUBSEC_UPD, 0);
    wr(qts, TS_CTRL, TSC_TSENA | TSC_TSCTRLSSR | TSC_TSIPENA |
       TSC_TSVER2ENA | TSC_SNAPTYPSEL(1) | TSC_TSINIT);

    put_rx_desc(qts, RX_DESC, RX_BUF, RX_DESC + 32);
    put_rx_desc(qts, RX_DESC + 32, RX_BUF + RX_BUF_SIZE, RX_DESC);
    wr(qts, RX_BASE, RX_DESC);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | CFG_RE);
    wr(qts, OP_MODE, OP_ST | OP_SR);
    qtest_clock_step(qts, 1 * MS);

    make_frame(frame, sizeof(frame), station, 0x88f7);
    frame[14] = 0x00;   /* Sync */
    frame[15] = 0x02;   /* PTPv2 */
    sock_send(fd[0], frame, sizeof(frame));
    qtest_clock_step(qts, 1 * MS);
    rdes0 = qtest_readl(qts, RX_DESC);
    g_assert_cmphex(rdes0 & (RDES0_OWN | RDES0_LS | RDES0_TSA | RDES0_ESA),
                    ==, RDES0_LS | RDES0_TSA | RDES0_ESA);
    rdes4 = qtest_readl(qts, RX_DESC + 16);
    g_assert_cmphex(RDES4_PTP_MSG(rdes4), ==, 1);
    g_assert_cmphex(rdes4 & (RDES4_PTP_ETH | RDES4_PTP_V2), ==,
                    RDES4_PTP_ETH | RDES4_PTP_V2);
    g_assert_cmpuint(qtest_readl(qts, RX_DESC + 28), ==, 100);
    ns = qtest_readl(qts, RX_DESC + 24);
    g_assert_cmpuint(ns, >=, 1000000);
    g_assert_cmpuint(ns, <=, 2000000);

    /* Not a PTP message: no snapshot. */
    make_frame(frame, sizeof(frame), station, 0x88b5);
    sock_send(fd[0], frame, sizeof(frame));
    qtest_clock_step(qts, 1 * MS);
    rdes0 = qtest_readl(qts, RX_DESC + 32);
    g_assert_cmphex(rdes0 & (RDES0_OWN | RDES0_LS | RDES0_TSA), ==,
                    RDES0_LS);

    /* Transmit with a timestamp and an inserted VLAN tag */
    wr(qts, TX_VLAN, 0x0123);
    qtest_memwrite(qts, TX_BUF, frame, sizeof(frame));
    put_tx_desc(qts, TX_DESC, TDES0_OWN | TDES0_FS | TDES0_LS | TDES0_TCH |
                TDES0_TTSE | TDES0_VLIC(2), sizeof(frame), TX_BUF,
                TX_DESC + 32);
    put_tx_desc(qts, TX_DESC + 32, TDES0_TCH, 0, 0, TX_DESC);
    wr(qts, TX_POLL, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(qtest_readl(qts, TX_DESC) & (TDES0_OWN | TDES0_TTSS), ==,
                    TDES0_TTSS);
    sec = qtest_readl(qts, TX_DESC + 28);
    ns = qtest_readl(qts, TX_DESC + 24);
    g_assert_cmpuint(sec, ==, 100);
    g_assert_cmpuint(ns, >=, 3000000);
    g_assert_cmpuint(ns, <=, 4000000);
    len = sock_recv(fd[0], got, sizeof(got));
    g_assert_cmpint(len, ==, sizeof(frame) + 4);
    g_assert_cmpmem(got, 12, frame, 12);
    g_assert_cmphex(lduw_be_p(got + 12), ==, 0x8100);
    g_assert_cmphex(lduw_be_p(got + 14), ==, 0x0123);
    g_assert_cmpmem(got + 16, sizeof(frame) - 12, frame + 12,
                    sizeof(frame) - 12);

    qtest_quit(qts);
    close(fd[0]);
}

/*
 * A descriptor the DMA APM refuses the GMAC: a fatal bus error on the
 * transmit descriptor read, the transmit process stopped.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_apm_refused(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -nic none");
    uint32_t st;

    emac_setup(qts);
    put_tx_desc(qts, TX_DESC, TDES0_OWN | TDES0_FS | TDES0_LS | TDES0_TCH,
                64, TX_BUF, TX_DESC);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE);
    qtest_writel(qts, DMA_PMS_GMAC_R, 0);
    wr(qts, OP_MODE, OP_ST);
    st = rd(qts, STATUS);
    g_assert_cmphex(st & ST_FBI, ==, ST_FBI);
    g_assert_cmpuint(ST_EB(st), ==, 7);
    g_assert_cmpuint(ST_TS(st), ==, 0);
    /* The descriptor is still the DMA's: nothing reached memory. */
    g_assert_cmphex(qtest_readl(qts, TX_DESC) & TDES0_OWN, ==, TDES0_OWN);
    /* Off the bus until a software reset */
    qtest_writel(qts, DMA_PMS_GMAC_R, 0xffffffff);
    wr(qts, TX_POLL, 0);
    g_assert_cmphex(qtest_readl(qts, TX_DESC) & TDES0_OWN, ==, TDES0_OWN);
    qtest_quit(qts);
}

/* Receive the next frame into @rx's buffer; its RDES0. */
static uint32_t rx_one(QTestState *qts, int fd, uint32_t rx,
                       const uint8_t *f, size_t len)
{
    put_rx_desc(qts, rx, RX_BUF, rx);
    wr(qts, RX_BASE, rx);
    wr(qts, OP_MODE, OP_ST | OP_SR);
    sock_send(fd, f, len);
    qtest_clock_step(qts, 1 * MS);
    wr(qts, OP_MODE, OP_ST);
    return qtest_readl(qts, rx);
}

/*
 * The receive VLAN tag filter and FCS stripping for type frames; pause
 * frames on HP_SYSTEM_GMAC_CTRL0.SBD_FLOWCTRL, repeated while it stays up
 * and ended with a zero-quanta one; and a magic packet waking the MAC
 * from power-down with PMT_INTR.
 */
/* [spec:nuos:req:emu.esp32p4.emac/test] */
static void test_vlan_flow_pmt(void)
{
    uint8_t frame[128], got[2048];
    QTestState *qts;
    uint32_t rdes0;
    int fd[2], pauses;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    qts = qtest_initf("-M esp32p4 -nic socket,fd=%d,model=esp32p4-emac", fd[1]);
    emac_setup(qts);
    rmii_pins(qts);
    set_station(qts);
    wr(qts, TX_BASE, TX_DESC);
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | CFG_RE);

    /* VLAN 5's frames pass the filter, VLAN 6's do not. */
    wr(qts, VLAN_TAG, VLAN_TAG_ETV | 5);
    make_frame(frame, 100, station, 0x8100);
    stw_be_p(frame + 14, 0x2005);
    stw_be_p(frame + 16, 0x88b5);
    rdes0 = rx_one(qts, fd[0], RX_DESC, frame, 100);
    g_assert_cmphex(rdes0 & (RDES0_OWN | RDES0_LS | RDES0_VLAN), ==,
                    RDES0_LS | RDES0_VLAN);
    stw_be_p(frame + 14, 0x2006);
    rdes0 = rx_one(qts, fd[0], RX_DESC, frame, 100);
    g_assert_cmphex(rdes0 & RDES0_OWN, ==, RDES0_OWN);
    wr(qts, FRAME_FILTER, 0);
    rdes0 = rx_one(qts, fd[0], RX_DESC, frame, 100);
    g_assert_cmphex(rdes0 & (RDES0_OWN | RDES0_VLAN), ==, RDES0_VLAN);

    /* EMACCONFIG.CST strips a type frame's FCS. */
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | CFG_RE | CFG_CST);
    make_frame(frame, 100, station, 0x88b5);
    rdes0 = rx_one(qts, fd[0], RX_DESC, frame, 100);
    g_assert_cmpint(RDES0_FL(rdes0), ==, 100);
    wr(qts, CONFIG, CFG_PS | CFG_FES | CFG_DM | CFG_TE | CFG_RE);

    /* Sideband flow control: pause frames while it is up, then a zero. */
    wr(qts, FLOW_CTRL, FC_PT(0x40) | FC_TFE);
    qtest_writel(qts, GMAC_CTRL0, GMAC_PHY_RMII | GMAC_SBD_FLOWCTRL);
    qtest_clock_step(qts, 1 * MS);
    for (pauses = 0; sock_recv(fd[0], got, sizeof(got)) == 60; pauses++) {
        g_assert_cmphex(lduw_be_p(got + 12), ==, 0x8808);
        g_assert_cmphex(lduw_be_p(got + 14), ==, 1);
        g_assert_cmphex(lduw_be_p(got + 16), ==, 0x40);
        g_assert_cmpmem(got + 6, 6, station, 6);
    }
    /* 0x40 - 4 slot times apart, 307.2 us at 100 Mbit/s: 4 in 1 ms */
    g_assert_cmpint(pauses, ==, 4);
    qtest_writel(qts, GMAC_CTRL0, GMAC_PHY_RMII);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpint(sock_recv(fd[0], got, sizeof(got)), ==, 60);
    g_assert_cmphex(lduw_be_p(got + 16), ==, 0);
    g_assert_cmpint(sock_recv(fd[0], got, sizeof(got)), ==, 0);
    /* DZPQ: no zero-quanta frame at the end */
    wr(qts, FLOW_CTRL, FC_PT(0x40) | FC_TFE | FC_DZPQ);
    qtest_writel(qts, GMAC_CTRL0, GMAC_PHY_RMII | GMAC_SBD_FLOWCTRL);
    qtest_clock_step(qts, 10 * US);
    qtest_writel(qts, GMAC_CTRL0, GMAC_PHY_RMII);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpint(sock_recv(fd[0], got, sizeof(got)), ==, 60);
    g_assert_cmphex(lduw_be_p(got + 16), ==, 0x40);
    g_assert_cmpint(sock_recv(fd[0], got, sizeof(got)), ==, 0);

    /* A magic packet wakes the MAC: PMT_INTR, not maskable. */
    wr(qts, PMT_CSR, PMT_MGKPKTEN | PMT_PWRDWN);
    memset(frame, 0, sizeof(frame));
    make_frame(frame, 14, station, 0x0842);
    memset(frame + 14, 0xff, 6);
    for (int i = 0; i < 16; i++) {
        memcpy(frame + 20 + 6 * i, station, 6);
    }
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_PMT_BIT, ==, 0);
    sock_send(fd[0], frame, 14 + 6 + 16 * 6);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_PMT_BIT, ==,
                    SRC_PMT_BIT);
    g_assert_cmphex(rd(qts, PMT_CSR) & (PMT_MGKPRCVD | PMT_PWRDWN), ==,
                    PMT_MGKPRCVD);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS2) & SRC_PMT_BIT, ==, 0);

    qtest_quit(qts);
    close(fd[0]);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4-emac/reset-values", test_reset_values);
    qtest_add_func("/esp32p4-emac/reset-and-clocks", test_reset_and_clocks);
    qtest_add_func("/esp32p4-emac/mdio", test_mdio);
    qtest_add_func("/esp32p4-emac/tx-rx", test_tx_rx);
    qtest_add_func("/esp32p4-emac/ptp-time", test_ptp_time);
    qtest_add_func("/esp32p4-emac/ptp-frames", test_ptp_frames);
    qtest_add_func("/esp32p4-emac/apm-refused", test_apm_refused);
    qtest_add_func("/esp32p4-emac/vlan-flow-pmt", test_vlan_flow_pmt);

    return g_test_run();
}
