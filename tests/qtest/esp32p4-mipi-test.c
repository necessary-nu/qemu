/*
 * QTest testcase for the ESP32-P4 MIPI DSI and MIPI CSI blocks
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * DSI: the host's and bridge's registers, the D-PHY's test codes and its
 * PLL's lock, commands to a panel (esp32p4-dsi-panel) and its answers,
 * their LP timing, errors, and video mode end to end, VDMA feeding the
 * bridge from L2MEM as ESP-IDF's driver sets it up, the panel's console
 * read back with screendump. CSI: the host's and bridge's registers and
 * test codes, packets crafted through the host's "packet" hook with ECC,
 * checksum, data type and frame errors, CSI_CM's conversions read from the
 * bridge's port, and capture from an esp32p4-image-source through VDMA,
 * each frame done at its frame end packet's time.
 * Expected values are worked out here from the TRM, the DSI and CSI-2
 * packet formats and the sensor's documented timing.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "qemu/bswap.h"

#define NS_PER_US           1000ull

/* Reset and clock control. */
#define CLKRST              0x500e6000
#define SOC_CLK_CTRL0       (CLKRST + 0x14)
#define SOC_CLK_CTRL1       (CLKRST + 0x18)
#define PERI_CLK_CTRL02     (CLKRST + 0x38)
#define PERI_CLK_CTRL03     (CLKRST + 0x3c)
#define HP_RST_EN0          (CLKRST + 0xc0)
#define GDMA_CPU_CLK_EN     (1u << 13)
#define GDMA_SYS_CLK_EN     (1u << 5)
#define CSI_BRG_SYS_CLK_EN  (1u << 10)
#define CSI_HOST_SYS_CLK_EN (1u << 11)
#define DSI_SYS_CLK_EN      (1u << 12)

/* The interrupt matrix's source status. */
#define INTMTX_STATUS(n)    (0x500d6000 + 0x200 + 4 * ((n) / 32))
#define SRC_CSI_BRIDGE      85
#define SRC_DSI_BRIDGE      86
#define SRC_CSI             87
#define SRC_DSI             88
#define SRC_DW_GDMA         24

/* The DSI host. */
#define DSI                 0x500a0000
#define D_VERSION           (DSI + 0x000)
#define D_PWR_UP            (DSI + 0x004)
#define D_CLKMGR_CFG        (DSI + 0x008)
#define D_DPI_VCID          (DSI + 0x00c)
#define D_DPI_COLOR_CODING  (DSI + 0x010)
#define D_DPI_CFG_POL       (DSI + 0x014)
#define D_PCKHDL_CFG        (DSI + 0x02c)
#define D_GEN_VCID          (DSI + 0x030)
#define D_MODE_CFG          (DSI + 0x034)
#define D_VID_MODE_CFG      (DSI + 0x038)
#define D_VID_PKT_SIZE      (DSI + 0x03c)
#define D_VID_NUM_CHUNKS    (DSI + 0x040)
#define D_VID_NULL_SIZE     (DSI + 0x044)
#define D_VID_HSA_TIME      (DSI + 0x048)
#define D_VID_HBP_TIME      (DSI + 0x04c)
#define D_VID_HLINE_TIME    (DSI + 0x050)
#define D_VID_VSA_LINES     (DSI + 0x054)
#define D_VID_VBP_LINES     (DSI + 0x058)
#define D_VID_VFP_LINES     (DSI + 0x05c)
#define D_VID_VACTIVE_LINES (DSI + 0x060)
#define D_CMD_MODE_CFG      (DSI + 0x068)
#define D_GEN_HDR           (DSI + 0x06c)
#define D_GEN_PLD_DATA      (DSI + 0x070)
#define D_CMD_PKT_STATUS    (DSI + 0x074)
#define D_TO_CNT_CFG        (DSI + 0x078)
#define D_LPCLK_CTRL        (DSI + 0x094)
#define D_PHY_TMR_LPCLK_CFG (DSI + 0x098)
#define D_PHY_TMR_CFG       (DSI + 0x09c)
#define D_PHY_RSTZ          (DSI + 0x0a0)
#define D_PHY_IF_CFG        (DSI + 0x0a4)
#define D_PHY_TX_TRIGGERS   (DSI + 0x0ac)
#define D_PHY_STATUS        (DSI + 0x0b0)
#define D_PHY_TST_CTRL0     (DSI + 0x0b4)
#define D_PHY_TST_CTRL1     (DSI + 0x0b8)
#define D_INT_ST0           (DSI + 0x0bc)
#define D_INT_ST1           (DSI + 0x0c0)
#define D_INT_MSK0          (DSI + 0x0c4)
#define D_INT_MSK1          (DSI + 0x0c8)
#define D_INT_FORCE1        (DSI + 0x0dc)
#define D_VID_SHADOW_CTRL   (DSI + 0x100)
#define D_VID_MODE_CFG_ACT  (DSI + 0x138)
#define D_VID_PKT_SIZE_ACT  (DSI + 0x13c)

/* The DSI bridge. */
#define BRG                 0x500a0800
#define B_EN                (BRG + 0x04)
#define B_DMA_REQ_CFG       (BRG + 0x08)
#define B_RAW_NUM_CFG       (BRG + 0x0c)
#define B_FIFO_FLOW_STATUS  (BRG + 0x14)
#define B_PIXEL_TYPE        (BRG + 0x18)
#define B_DPI_RSV_DPI_DATA  (BRG + 0x28)
#define B_DPI_V_CFG0        (BRG + 0x30)
#define B_DPI_V_CFG1        (BRG + 0x34)
#define B_DPI_H_CFG0        (BRG + 0x38)
#define B_DPI_H_CFG1        (BRG + 0x3c)
#define B_DPI_MISC_CONFIG   (BRG + 0x40)
#define B_DPI_CONFIG_UPDATE (BRG + 0x44)
#define B_INT_ENA           (BRG + 0x50)
#define B_INT_CLR           (BRG + 0x54)
#define B_INT_RAW           (BRG + 0x58)
#define B_INT_ST            (BRG + 0x5c)
#define B_HOST_CTRL         (BRG + 0x80)
#define B_DMA_FLOW_CTRL     (BRG + 0x88)
#define B_RAW_BUF_ALMOST_EMPTY (BRG + 0x8c)
#define B_YUV_CFG           (BRG + 0x90)
#define B_VER_DATE          (BRG + 0x100)
#define DSI_MEM             0x50105000

/* VDMA. */
#define VDMA                0x50081000
#define V_CFG0              (VDMA + 0x10)
#define V_CHEN0             (VDMA + 0x18)
#define V_CH(n)             (VDMA + 0x100 * (n))
#define V_SAR0              0x00
#define V_DAR0              0x08
#define V_BLOCK_TS0         0x10
#define V_CTL0              0x18
#define V_CTL1              0x1c
#define V_CH_CFG0           0x20
#define V_CH_CFG1           0x24
#define V_LLP0              0x28
#define V_INTST_ENA0        0x80
#define V_INTSTATUS         0x88
#define V_INTSIG_ENA0       0x90
#define V_INTCLEAR0         0x98
#define DMA_TFR_DONE        (1u << 1)

/* The CSI host and bridge. */
#define CSI                 0x5009f000
#define C_VERSION           (CSI + 0x000)
#define C_N_LANES           (CSI + 0x004)
#define C_CSI2_RESETN       (CSI + 0x008)
#define C_INT_ST_MAIN       (CSI + 0x00c)
#define C_PHY_SHUTDOWNZ     (CSI + 0x040)
#define C_DPHY_RSTZ         (CSI + 0x044)
#define C_PHY_RX            (CSI + 0x048)
#define C_PHY_STOPSTATE     (CSI + 0x04c)
#define C_PHY_TEST_CTRL0    (CSI + 0x050)
#define C_PHY_TEST_CTRL1    (CSI + 0x054)
#define C_VC_EXTENSION      (CSI + 0x0c8)
#define C_ST_PHY_FATAL      (CSI + 0x0e0)
#define C_MSK_PHY_FATAL     (CSI + 0x0e4)
#define C_ST_PKT_FATAL      (CSI + 0x0f0)
#define C_MSK_PKT_FATAL     (CSI + 0x0f4)
#define C_FORCE_PKT_FATAL   (CSI + 0x0f8)
#define C_ST_PHY            (CSI + 0x110)
#define C_ST_BNDRY          (CSI + 0x280)
#define C_ST_SEQ            (CSI + 0x290)
#define C_ST_CRC_FRAME      (CSI + 0x2a0)
#define C_ST_PLD_CRC        (CSI + 0x2b0)
#define C_MSK_PLD_CRC       (CSI + 0x2b4)
#define C_ST_DATA_ID        (CSI + 0x2c0)
#define C_ST_ECC_CORRECTED  (CSI + 0x2d0)
#define C_SCRAMBLING_SEED1  (CSI + 0x304)
#define C_SCRAMBLING_SEED2  (CSI + 0x308)

#define CB                  0x5009f800
#define CB_CSI_EN           (CB + 0x04)
#define CB_DMA_REQ_CFG      (CB + 0x08)
#define CB_BUF_FLOW_CTL     (CB + 0x0c)
#define CB_DATA_TYPE_CFG    (CB + 0x10)
#define CB_FRAME_CFG        (CB + 0x14)
#define CB_ENDIAN_MODE      (CB + 0x18)
#define CB_INT_RAW          (CB + 0x1c)
#define CB_INT_CLR          (CB + 0x20)
#define CB_INT_ST           (CB + 0x24)
#define CB_INT_ENA          (CB + 0x28)
#define CB_DMABLK_SIZE      (CB + 0x30)
#define CB_HOST_CTRL        (CB + 0x40)
#define CB_HOST_CM_CTRL     (CB + 0x48)
#define CSI_MEM             0x50104000

#define MEM                 0x4ff40000

/* ---- Common ---- */

/* Virtual time, as the last step left it. */
static int64_t vt;

static int64_t step(QTestState *qts, int64_t ns)
{
    vt = qtest_clock_step(qts, ns);
    return vt;
}

static int64_t vnow(void)
{
    return vt;
}

static QTestState *machine(const char *extra)
{
    QTestState *qts = qtest_initf("-M esp32p4,flash-size=16M,strap-mode=0 %s",
                                  extra ? extra : "");

    vt = step(qts, 1);

    qtest_writel(qts, SOC_CLK_CTRL0,
                 qtest_readl(qts, SOC_CLK_CTRL0) | GDMA_CPU_CLK_EN);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | GDMA_SYS_CLK_EN |
                 CSI_BRG_SYS_CLK_EN | CSI_HOST_SYS_CLK_EN | DSI_SYS_CLK_EN);
    return qts;
}

static bool intr(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src)) >> (src % 32)) & 1;
}

/* ---- The DSI D-PHY ---- */

/* Write test code @code's data @data as ESP-IDF's HAL does. */
static void dsi_phy_write(QTestState *qts, uint8_t code, uint8_t data)
{
    qtest_writel(qts, D_PHY_TST_CTRL0, 0);
    qtest_writel(qts, D_PHY_TST_CTRL1, (1u << 16) | code);
    qtest_writel(qts, D_PHY_TST_CTRL0, 2);
    qtest_writel(qts, D_PHY_TST_CTRL0, 0);
    qtest_writel(qts, D_PHY_TST_CTRL1, data);
    qtest_writel(qts, D_PHY_TST_CTRL0, 2);
    qtest_writel(qts, D_PHY_TST_CTRL0, 0);
}

/* TESTDOUT for @code with TESTDIN @din. */
static uint8_t dsi_phy_dout(QTestState *qts, uint8_t code, uint8_t din)
{
    qtest_writel(qts, D_PHY_TST_CTRL0, 2);
    qtest_writel(qts, D_PHY_TST_CTRL1, (1u << 16) | code);
    qtest_writel(qts, D_PHY_TST_CTRL0, 0);
    qtest_writel(qts, D_PHY_TST_CTRL1, din);
    return qtest_readl(qts, D_PHY_TST_CTRL1) >> 8;
}

/*
 * The DSI PHY's clocks: configuration from PLL_F20M, PLL reference from
 * XTAL (40 MHz) divided by @refdiv.
 */
static void dsi_clocks(QTestState *qts, uint32_t refdiv)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL03);

    qtest_writel(qts, PERI_CLK_CTRL02,
                 qtest_readl(qts, PERI_CLK_CTRL02) & ~(3u << 30));
    v &= ~(0xffu << 19 | 7u << 16);
    v |= 3u | (refdiv - 1) << 19;
    qtest_writel(qts, PERI_CLK_CTRL03, v);
}

/*
 * Bring the PHY up at @mbps (a 40 MHz reference: N 2, M mbps / 20), its
 * hsfreqrange @range, as ESP-IDF's HAL does, and wait for its lanes.
 */
static void dsi_phy_up(QTestState *qts, uint32_t mbps, uint8_t range,
                       uint32_t lanes)
{
    uint32_t m = mbps / 20;

    dsi_clocks(qts, 1);
    qtest_writel(qts, D_PHY_IF_CFG, lanes - 1);
    qtest_writel(qts, D_PWR_UP, 1);
    qtest_writel(qts, D_PHY_RSTZ, 1);
    qtest_writel(qts, D_PHY_RSTZ, 1 | 2);
    qtest_writel(qts, D_PHY_RSTZ, 1 | 2 | 4 | 8);
    dsi_phy_write(qts, 0x44, range << 1);
    dsi_phy_write(qts, 0x22, mbps >= 1000 ? 0x88 : 0x80);
    dsi_phy_write(qts, 0x19, 0x30);
    dsi_phy_write(qts, 0x17, 2 - 1);
    dsi_phy_write(qts, 0x18, (m - 1) & 0x1f);
    dsi_phy_write(qts, 0x18, 0x80 | (((m - 1) >> 5) & 0x0f));
    /* Lock in 2000 cycles of the 20 MHz configuration clock, LP-11 1000 on. */
    step(qts, 150 * NS_PER_US + 1000);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 0x95, ==,
                    lanes > 1 ? 0x95 : 0x15);
}

/* ---- DSI tests ---- */

static void test_dsi_registers(void)
{
    QTestState *qts = machine(NULL);

    g_assert_cmphex(qtest_readl(qts, D_VERSION), ==, 0x3134312a);
    g_assert_cmphex(qtest_readl(qts, D_MODE_CFG), ==, 1);
    g_assert_cmphex(qtest_readl(qts, D_PHY_IF_CFG), ==, 1);
    g_assert_cmphex(qtest_readl(qts, D_CMD_PKT_STATUS), ==, 0x50015);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS), ==, 0x140);
    g_assert_cmphex(qtest_readl(qts, D_PHY_TST_CTRL0), ==, 1);
    g_assert_cmphex(qtest_readl(qts, B_DMA_REQ_CFG), ==, 0x80);
    g_assert_cmphex(qtest_readl(qts, B_RAW_NUM_CFG), ==, 0x38400);
    g_assert_cmphex(qtest_readl(qts, B_DPI_V_CFG0), ==, 0x01e0020d);
    g_assert_cmphex(qtest_readl(qts, B_DPI_H_CFG1), ==, 0x00600030);
    g_assert_cmphex(qtest_readl(qts, B_DPI_MISC_CONFIG), ==, 0x19d0);
    g_assert_cmphex(qtest_readl(qts, B_HOST_CTRL), ==, 1);
    g_assert_cmphex(qtest_readl(qts, B_DMA_FLOW_CTRL), ==, 0x11);
    g_assert_cmphex(qtest_readl(qts, B_RAW_BUF_ALMOST_EMPTY), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, B_VER_DATE), ==, 0x20250109);

    /* Fields hold what they hold; read-only registers keep theirs. */
    qtest_writel(qts, D_VID_PKT_SIZE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, D_VID_PKT_SIZE), ==, 0x3fff);
    qtest_writel(qts, D_VERSION, 0);
    g_assert_cmphex(qtest_readl(qts, D_VERSION), ==, 0x3134312a);

    /* Without shadowing the _ACT registers follow, VID_MODE_CFG packed. */
    g_assert_cmphex(qtest_readl(qts, D_VID_PKT_SIZE_ACT), ==, 0x3fff);
    qtest_writel(qts, D_VID_MODE_CFG, 0x3f02 | 1u << 15);
    g_assert_cmphex(qtest_readl(qts, D_VID_MODE_CFG_ACT), ==,
                    2 | 0x3f << 2 | 1u << 9);
    /* With shadowing, on request (at once outside video mode). */
    qtest_writel(qts, D_VID_SHADOW_CTRL, 1);
    qtest_writel(qts, D_VID_PKT_SIZE, 100);
    g_assert_cmphex(qtest_readl(qts, D_VID_PKT_SIZE_ACT), ==, 0x3fff);
    qtest_writel(qts, D_VID_SHADOW_CTRL, 1 | 1u << 8);
    g_assert_cmphex(qtest_readl(qts, D_VID_PKT_SIZE_ACT), ==, 100);
    g_assert_cmphex(qtest_readl(qts, D_VID_SHADOW_CTRL), ==, 1);

    /* INT_ST clears as it is read; INT_FORCE holds its bits there. */
    qtest_writel(qts, D_INT_FORCE1, 1u << 8);
    g_assert_false(intr(qts, SRC_DSI));
    qtest_writel(qts, D_INT_MSK1, 1u << 8);
    g_assert_true(intr(qts, SRC_DSI));
    g_assert_cmphex(qtest_readl(qts, D_INT_ST1), ==, 1u << 8);
    qtest_writel(qts, D_INT_FORCE1, 0);
    g_assert_false(intr(qts, SRC_DSI));
    g_assert_cmphex(qtest_readl(qts, D_INT_ST1), ==, 0);

    /* The generic FIFOs: a write to a full one is lost. */
    for (int i = 0; i < 128; i++) {
        qtest_writel(qts, D_GEN_PLD_DATA, i);
    }
    g_assert_cmphex(qtest_readl(qts, D_CMD_PKT_STATUS) & 0xc, ==, 8);
    qtest_writel(qts, D_GEN_PLD_DATA, 0xdead);
    g_assert_cmphex(qtest_readl(qts, D_INT_ST1), ==, 1u << 9);
    g_assert_cmphex(qtest_readl(qts, D_GEN_PLD_DATA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, D_INT_ST1), ==, 1u << 11);

    /* The bridge's interrupts: raw, enable, status, clear. */
    qtest_writel(qts, B_INT_ENA, 2);
    g_assert_cmphex(qtest_readl(qts, B_INT_ST), ==, 0);
    g_assert_false(intr(qts, SRC_DSI_BRIDGE));
    qtest_quit(qts);
}

static void test_dsi_phy(void)
{
    QTestState *qts = machine("-device esp32p4-dsi-panel,width=16,height=8");

    /* TESTDOUT loops each code's data back, by set where it has two. */
    dsi_phy_write(qts, 0x44, 0x06);
    g_assert_cmphex(dsi_phy_dout(qts, 0x44, 0), ==, 0x06);
    dsi_phy_write(qts, 0x18, 0x09);
    dsi_phy_write(qts, 0x18, 0x83);
    g_assert_cmphex(dsi_phy_dout(qts, 0x18, 0x00), ==, 0x09);
    g_assert_cmphex(dsi_phy_dout(qts, 0x18, 0x80), ==, 0x03);
    g_assert_cmphex(dsi_phy_dout(qts, 0x17, 0), ==, 0x01);
    dsi_phy_write(qts, 0x22, 0x88);
    g_assert_cmphex(dsi_phy_dout(qts, 0x22, 0x80), ==, 0x08);
    g_assert_cmphex(dsi_phy_dout(qts, 0x22, 0x00), ==, 0x00);
    /* TESTCLR takes them back to their defaults. */
    qtest_writel(qts, D_PHY_TST_CTRL0, 1);
    qtest_writel(qts, D_PHY_TST_CTRL0, 0);
    g_assert_cmphex(dsi_phy_dout(qts, 0x18, 0x00), ==, 0x05);
    g_assert_cmphex(dsi_phy_dout(qts, 0x44, 0), ==, 0x00);
    g_assert_cmphex(dsi_phy_dout(qts, 0x10, 0), ==, 0x01);

    /* The PLL locks 2000 configuration clocks (20 MHz) after RSTZ. */
    dsi_clocks(qts, 1);
    dsi_phy_write(qts, 0x44, 0x03 << 1);
    dsi_phy_write(qts, 0x19, 0x30);
    dsi_phy_write(qts, 0x17, 1);
    dsi_phy_write(qts, 0x18, 9);
    dsi_phy_write(qts, 0x18, 0x80);
    qtest_writel(qts, D_PHY_RSTZ, 1 | 2 | 4);
    step(qts, 99 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 1, ==, 0);
    step(qts, 1 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 0x95, ==, 1);
    step(qts, 50 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 0x95, ==, 0x95);
    /* A divider change locks again; an odd M does not lock. */
    dsi_phy_write(qts, 0x18, 8);
    step(qts, 200 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 1, ==, 0);
    dsi_phy_write(qts, 0x18, 9);
    step(qts, 200 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 1, ==, 1);
    /* Shut down, its status is back to its reset value. */
    qtest_writel(qts, D_PHY_RSTZ, 0);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS), ==, 0x140);
    qtest_quit(qts);
}

/* Queue a DCS write of @cmd and its @n parameters, as ESP-IDF's HAL does. */
static void dcs_write(QTestState *qts, uint8_t cmd, const uint8_t *p,
                      uint32_t n)
{
    uint32_t w = cmd, size = n + 1;

    for (uint32_t i = 0; i < MIN(n, 3); i++) {
        w |= (uint32_t)p[i] << (8 * (i + 1));
    }
    if (size > 2) {
        qtest_writel(qts, D_GEN_PLD_DATA, w);
        for (uint32_t i = 3; i < n; i += 4) {
            uint32_t v = 0;

            for (uint32_t k = 0; k < 4 && i + k < n; k++) {
                v |= (uint32_t)p[i + k] << (8 * k);
            }
            qtest_writel(qts, D_GEN_PLD_DATA, v);
        }
        qtest_writel(qts, D_GEN_HDR, 0x39 | size << 8);
    } else if (size == 2) {
        qtest_writel(qts, D_GEN_HDR, 0x15 | (w & 0xffff) << 8);
    } else {
        qtest_writel(qts, D_GEN_HDR, 0x05 | (w & 0xffff) << 8);
    }
}

/* Wait for the queued packets to go, and their answers to come in. */
static void wait_cmd_empty(QTestState *qts)
{
    for (int i = 0; i < 10000; i++) {
        if ((qtest_readl(qts, D_CMD_PKT_STATUS) & 0x10041) == 0x10001 &&
            (qtest_readl(qts, D_PHY_STATUS) & 0x10)) {
            return;
        }
        step(qts, NS_PER_US);
    }
    g_assert_not_reached();
}

/* A DCS read of @cmd into @out, as ESP-IDF's HAL does; false on timeout. */
static bool dcs_read(QTestState *qts, uint8_t cmd, uint8_t *out, uint32_t n)
{
    uint32_t got = 0;

    qtest_writel(qts, D_GEN_HDR, 0x37 | n << 8);
    qtest_writel(qts, D_MODE_CFG, 1);
    qtest_writel(qts, D_PCKHDL_CFG, qtest_readl(qts, D_PCKHDL_CFG) | 4);
    qtest_writel(qts, D_GEN_VCID, 0);
    qtest_writel(qts, D_GEN_HDR, 0x06 | cmd << 8);
    for (int i = 0; i < 20000; i++) {
        if (!(qtest_readl(qts, D_CMD_PKT_STATUS) & (1u << 6))) {
            break;
        }
        if (qtest_readl(qts, D_INT_ST1) & 2) {
            return false;
        }
        step(qts, NS_PER_US);
    }
    while (!(qtest_readl(qts, D_CMD_PKT_STATUS) & (1u << 4))) {
        uint32_t w = qtest_readl(qts, D_GEN_PLD_DATA);

        for (int k = 0; k < 4 && got < n; k++) {
            out[got++] = w >> (8 * k);
        }
    }
    return got == n;
}

/* The host as ESP-IDF's esp_lcd_new_dsi_bus() and DBI IO set it up. */
static void dsi_bus(QTestState *qts, uint32_t mbps, uint8_t range,
                    uint32_t esc_div)
{
    dsi_phy_up(qts, mbps, range, 2);
    qtest_writel(qts, D_MODE_CFG, 1);
    qtest_writel(qts, D_LPCLK_CTRL, 3);
    qtest_writel(qts, D_PHY_TMR_CFG, 50u << 16 | 104);
    qtest_writel(qts, D_PHY_TMR_LPCLK_CFG, 46u << 16 | 128);
    qtest_writel(qts, D_PCKHDL_CFG, 1u << 4 | 1u << 3 | 1u << 0);
    qtest_writel(qts, D_CLKMGR_CFG, 10u << 8 | esc_div);
    qtest_writel(qts, D_TO_CNT_CFG, 0x7fff);
    qtest_writel(qts, D_CMD_MODE_CFG, 0x010f7f02);
}

static void test_dsi_commands(void)
{
    QTestState *qts = machine("-device esp32p4-dsi-panel,width=16,height=8,"
                              "id1=0x83,id2=0x11,id3=0x6c");
    static const uint8_t vend[5] = { 1, 2, 3, 4, 5 };
    uint8_t b[8];
    uint64_t t0;

    /* 200 Mbit/s: a 25 MHz byte clock, TX_ESC_CLK 12.5 MHz (80 ns). */
    dsi_bus(qts, 200, 0x03, 2);

    /*
     * A DCS short write in LP: escape entry, the LPDT command, 4 bytes at
     * 16 escape clocks and the exit, 22 + 64 = 86 escape clocks (6880
     * ns, counted on the byte clock's edges), the command FIFO empty from
     * its start, lane 0 out of its stop state until it ends.
     */
    t0 = vnow();
    dcs_write(qts, 0x36, (const uint8_t[]){ 0x40 }, 1);
    g_assert_cmphex(qtest_readl(qts, D_CMD_PKT_STATUS) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, D_CMD_PKT_STATUS) & (1u << 16), ==, 0);
    step(qts, 6880 - 40);
    g_assert_cmphex(qtest_readl(qts, D_PHY_STATUS) & 0x10, ==, 0);
    while (!(qtest_readl(qts, D_PHY_STATUS) & 0x10)) {
        step(qts, 1);
    }
    g_assert_cmpuint(vnow() - t0, >, 6880 - 40);
    g_assert_cmpuint(vnow() - t0, <=, 6880);

    /* The panel keeps a manufacturer command's parameters to read back. */
    dcs_write(qts, 0xb2, vend, 5);
    wait_cmd_empty(qts);
    g_assert_true(dcs_read(qts, 0xb2, b, 5));
    g_assert_cmpmem(b, 5, vend, 5);
    g_assert_true(dcs_read(qts, 0xda, b, 1));
    g_assert_cmphex(b[0], ==, 0x83);
    g_assert_true(dcs_read(qts, 0x04, b, 3));
    g_assert_cmphex(b[0] << 16 | b[1] << 8 | b[2], ==, 0x83116c);
    /* get_power_mode: in sleep, display on (EK79007-like); then out. */
    g_assert_true(dcs_read(qts, 0x0a, b, 1));
    g_assert_cmphex(b[0], ==, 0x0c);
    dcs_write(qts, 0x11, NULL, 0);
    wait_cmd_empty(qts);
    g_assert_true(dcs_read(qts, 0x0a, b, 1));
    g_assert_cmphex(b[0], ==, 0x9c);
    g_assert_true(dcs_read(qts, 0x0b, b, 1));
    g_assert_cmphex(b[0], ==, 0x40);

    /*
     * With BTA on, ACK_RQST_EN asks every write for an acknowledge: a data
     * type the panel does not take comes back in its error report.
     */
    qtest_writel(qts, D_GEN_HDR, 0x0a);
    wait_cmd_empty(qts);
    g_assert_cmphex(qtest_readl(qts, D_INT_ST0), ==, 1u << 11);
    dcs_write(qts, 0x29, NULL, 0);
    wait_cmd_empty(qts);
    g_assert_cmphex(qtest_readl(qts, D_INT_ST0), ==, 0);
    /* Another virtual channel: VC ID invalid. */
    qtest_writel(qts, D_GEN_HDR, 0x05 | 1u << 6 | 0x29 << 8);
    wait_cmd_empty(qts);
    g_assert_cmphex(qtest_readl(qts, D_INT_ST0), ==, 1u << 12);
    /* A long write with its payload short: GEN_PLD_SEND_ERR. */
    qtest_writel(qts, D_GEN_HDR, 0x39 | 8u << 8);
    wait_cmd_empty(qts);
    g_assert_cmphex(qtest_readl(qts, D_INT_ST1) & (1u << 10), ==, 1u << 10);
    qtest_quit(qts);
}

static void test_dsi_no_panel(void)
{
    QTestState *qts = machine(NULL);
    uint8_t b[1];
    uint64_t t0;

    dsi_bus(qts, 200, 0x03, 2);
    /*
     * No panel answers the turnaround: TO_LP_RX after LPRX_TO_CNT (0x7fff)
     * x TO_CLK_DIVISION (10) byte clocks, 13.1 ms.
     */
    t0 = vnow();
    g_assert_false(dcs_read(qts, 0x0a, b, 1));
    g_assert_cmpuint(vnow() - t0, >=,
                     0x7fffull * 10 * 40);
    qtest_quit(qts);
}

/* ---- DSI video ---- */

#define PW  16
#define PH  8

static uint8_t px_r(int x, int y)
{
    return x * 16 + y;
}

static uint8_t px_g(int x, int y)
{
    return y * 32 + 7;
}

static uint8_t px_b(int x, int y)
{
    return (x ^ y) * 9;
}

/* A screendump of the panel's console: its pixels, 0xRRGGBB. */
static uint32_t *screendump(QTestState *qts, int *w, int *h)
{
    g_autofree char *path = g_strdup_printf("%s/esp32p4-mipi-%d.ppm",
                                            g_get_tmp_dir(), getpid());
    g_autofree char *data = NULL;
    QDict *r;
    gsize len;
    uint32_t *px;
    char *p;
    int maxval;

    r = qtest_qmp(qts, "{'execute': 'screendump', 'arguments': "
                  "{'filename': %s}}", path);
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    g_assert(g_file_get_contents(path, &data, &len, NULL));
    unlink(path);
    g_assert(sscanf(data, "P6 %d %d %d", w, h, &maxval) == 3);
    p = data;
    for (int k = 0; k < 4; k++) {
        p = strpbrk(p, "\n ") + 1;
    }
    px = g_new(uint32_t, *w * *h);
    for (int i = 0; i < *w * *h; i++) {
        px[i] = (uint8_t)p[3 * i] << 16 | (uint8_t)p[3 * i + 1] << 8 |
                (uint8_t)p[3 * i + 2];
    }
    return px;
}

/*
 * The DPI clock from PLL_F240M over @div; the bridge and host for a PW x
 * PH RGB888 panel, as ESP-IDF's esp_lcd_new_panel_dpi() and panel init do.
 * Horizontal: HSYNC 2, HBP 2, HFP 4 (HTOTAL 24); vertical: VSYNC 1, VBP 1,
 * VFP 2 (VTOTAL 12).
 */
static void dsi_dpi(QTestState *qts, uint32_t div, uint32_t in_type,
                    uint32_t bpp, uint32_t coding, uint32_t dpi_type)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL03);
    double ratio = 200.0 / 8 / (240.0 / div);

    v &= ~(0xffu << 8 | 3u << 5);
    v |= 1u << 5 | 1u << 7 | (div - 1) << 8;
    qtest_writel(qts, PERI_CLK_CTRL03, v);

    qtest_writel(qts, D_DPI_VCID, 0);
    qtest_writel(qts, D_DPI_COLOR_CODING, coding);
    qtest_writel(qts, D_DPI_CFG_POL, 0);
    qtest_writel(qts, D_VID_MODE_CFG, 0x3f00 | 1u << 15);
    qtest_writel(qts, D_VID_PKT_SIZE, PW);
    qtest_writel(qts, D_VID_NUM_CHUNKS, 0);
    qtest_writel(qts, D_VID_NULL_SIZE, 0);
    qtest_writel(qts, D_VID_HSA_TIME, (uint32_t)(2 * ratio + 0.5));
    qtest_writel(qts, D_VID_HBP_TIME, (uint32_t)(2 * ratio + 0.5));
    qtest_writel(qts, D_VID_HLINE_TIME, (uint32_t)(24 * ratio + 0.5));
    qtest_writel(qts, D_VID_VSA_LINES, 1);
    qtest_writel(qts, D_VID_VBP_LINES, 1);
    qtest_writel(qts, D_VID_VACTIVE_LINES, PH);
    qtest_writel(qts, D_VID_VFP_LINES, 2);
    /* Short switch times, for the small panel's short lines. */
    qtest_writel(qts, D_PHY_TMR_CFG, 4u << 16 | 8);

    qtest_writel(qts, B_EN, 2);
    qtest_writel(qts, B_EN, 0);
    qtest_writel(qts, B_PIXEL_TYPE, in_type | (in_type >= 8 ? 1u << 6 : 0) |
                 dpi_type << 7);
    qtest_writel(qts, B_DPI_H_CFG0, PW << 16 | 24);
    qtest_writel(qts, B_DPI_H_CFG1, 2u << 16 | 2);
    qtest_writel(qts, B_DPI_V_CFG0, PH << 16 | 12);
    qtest_writel(qts, B_DPI_V_CFG1, 1u << 16 | 1);
    qtest_writel(qts, B_RAW_NUM_CFG, (PW * PH * bpp / 64) | 1u << 31);
    qtest_writel(qts, B_DPI_MISC_CONFIG, 12u << 4);
    qtest_writel(qts, B_DMA_FLOW_CTRL, 1u << 4);
    qtest_writel(qts, B_DMA_REQ_CFG, 256);
    qtest_writel(qts, B_RAW_BUF_ALMOST_EMPTY, 1024 - 256);
    qtest_writel(qts, B_EN, 1);
    qtest_writel(qts, B_DPI_CONFIG_UPDATE, 1);
}

/* VDMA channel 1 feeding the bridge from @fb, @bytes, as ESP-IDF does. */
static void dsi_dma(QTestState *qts, uint32_t fb, uint32_t bytes)
{
    uint32_t lli = MEM + 0x8000;
    uint32_t w[16] = { 0 };

    qtest_writel(qts, V_CFG0, 3);
    w[0] = fb;
    w[2] = DSI_MEM;
    w[4] = bytes / 8 - 1;
    /*
     * Memory on master 2, the bridge on master 1, fixed; 64 bits; bursts
     * of 512 and 256.
     */
    w[8] = 1u << 0 | 1u << 6 | 3u << 8 | 3u << 11 | 8u << 14 | 7u << 18;
    w[9] = 1u << 30 | 1u << 31;
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, lli + 4 * i, w[i]);
    }
    qtest_writel(qts, V_CH(1) + V_CH_CFG0, 3 | 3 << 2);
    qtest_writel(qts, V_CH(1) + V_CH_CFG1, 1 | 1u << 17);
    qtest_writel(qts, V_CH(1) + V_LLP0, lli | 1);
    qtest_writel(qts, V_CH(1) + V_INTST_ENA0, 0xffffffff);
    qtest_writel(qts, V_CHEN0, 1 | 1u << 8);
}

static void dsi_video_up(QTestState *qts)
{
    qtest_writel(qts, D_MODE_CFG, 0);
    qtest_writel(qts, B_DPI_MISC_CONFIG,
                 qtest_readl(qts, B_DPI_MISC_CONFIG) | 1);
    qtest_writel(qts, B_DPI_CONFIG_UPDATE, 1);
    qtest_writel(qts, B_INT_ENA, 3);
}

static void test_dsi_video(void)
{
    QTestState *qts = machine("-device esp32p4-dsi-panel,width=16,height=8");
    uint8_t fb[PW * PH * 3];
    g_autofree uint32_t *px = NULL;
    uint64_t t0, t1;
    int w, h;

    for (int y = 0; y < PH; y++) {
        for (int x = 0; x < PW; x++) {
            fb[3 * (y * PW + x)] = px_b(x, y);
            fb[3 * (y * PW + x) + 1] = px_g(x, y);
            fb[3 * (y * PW + x) + 2] = px_r(x, y);
        }
    }
    qtest_memwrite(qts, MEM, fb, sizeof(fb));
    dsi_bus(qts, 200, 0x03, 2);
    dcs_write(qts, 0x11, NULL, 0);
    wait_cmd_empty(qts);
    /* DPI at 240 MHz / 24, 10 MHz: a frame of 24 x 12 clocks, 28.8 us. */
    dsi_dpi(qts, 24, 0, 24, 5, 0);
    dsi_dma(qts, MEM, sizeof(fb));
    dsi_video_up(qts);

    /* VSYNC rises at each frame's start, 28.8 us apart. */
    g_assert_true(qtest_readl(qts, B_INT_RAW) & 2);
    qtest_writel(qts, B_INT_CLR, 2);
    t0 = vnow();
    while (!(qtest_readl(qts, B_INT_RAW) & 2)) {
        step(qts, 100);
    }
    t1 = vnow();
    g_assert_cmpuint(t1 - t0, <=, 28800 + 100);
    g_assert_cmpuint(t1 - t0, >=, 28800 - 100);
    g_assert_true(intr(qts, SRC_DSI_BRIDGE));
    /* The frame went with no underrun; VDMA's block is done. */
    g_assert_cmphex(qtest_readl(qts, B_INT_RAW) & 1, ==, 0);
    g_assert_true(qtest_readl(qts, V_CH(1) + V_INTSTATUS) & DMA_TFR_DONE);

    px = screendump(qts, &w, &h);
    g_assert_cmpint(w, ==, PW);
    g_assert_cmpint(h, ==, PH);
    for (int y = 0; y < PH; y++) {
        for (int x = 0; x < PW; x++) {
            g_assert_cmphex(px[y * PW + x], ==,
                            px_r(x, y) << 16 | px_g(x, y) << 8 | px_b(x, y));
        }
    }

    /*
     * VDMA not restarted: the next frame underruns, its lines the
     * DPI_RSV_DPI_DATA colour, UNDERRUN_INT for lines below the count.
     */
    qtest_writel(qts, B_DPI_RSV_DPI_DATA, 0x00ff00);
    qtest_writel(qts, B_DPI_CONFIG_UPDATE, 1);
    step(qts, 60 * NS_PER_US);
    g_assert_cmphex(qtest_readl(qts, B_INT_RAW) & 1, ==, 1);
    g_free(px);
    px = screendump(qts, &w, &h);
    g_assert_cmphex(px[0], ==, 0x00ff00);
    g_assert_cmphex(px[PW * PH - 1], ==, 0x00ff00);
    qtest_quit(qts);
}

/* The bridge's RGB565 input to an RGB565 link, and YUV422 to RGB888. */
static void test_dsi_formats(void)
{
    QTestState *qts = machine("-device esp32p4-dsi-panel,width=16,height=8");
    uint16_t fb[PW * PH];
    uint8_t yuv[PW * PH * 2];
    g_autofree uint32_t *px = NULL;
    int w, h;

    for (int i = 0; i < PW * PH; i++) {
        fb[i] = cpu_to_le16(((i * 3) & 31) << 11 | ((i * 5) & 63) << 5 |
                            (i & 31));
    }
    qtest_memwrite(qts, MEM, fb, sizeof(fb));
    dsi_bus(qts, 200, 0x03, 2);
    dcs_write(qts, 0x11, NULL, 0);
    wait_cmd_empty(qts);
    dsi_dpi(qts, 24, 2, 16, 0, 2);
    dsi_dma(qts, MEM, sizeof(fb));
    dsi_video_up(qts);
    /* The first frame shown; VDMA not restarted, the next would underrun. */
    step(qts, 30 * NS_PER_US);
    px = screendump(qts, &w, &h);
    for (int i = 0; i < PW * PH; i++) {
        uint32_t r = (i * 3) & 31, g = (i * 5) & 63, b = i & 31;
        uint32_t want = (r << 3 | r >> 2) << 16 | (g << 2 | g >> 4) << 8 |
                        (b << 3 | b >> 2);

        g_assert_cmphex(px[i], ==, want);
    }
    qtest_quit(qts);

    /* YUV422 (U Y0 V Y1, BT.601 limited range) in, RGB888 out. */
    qts = machine("-device esp32p4-dsi-panel,width=16,height=8");
    for (int i = 0; i < PW * PH / 2; i++) {
        yuv[4 * i] = 128;
        yuv[4 * i + 1] = 16;
        yuv[4 * i + 2] = 128;
        yuv[4 * i + 3] = 235;
    }
    qtest_memwrite(qts, MEM, yuv, sizeof(yuv));
    dsi_bus(qts, 200, 0x03, 2);
    dcs_write(qts, 0x11, NULL, 0);
    wait_cmd_empty(qts);
    dsi_dpi(qts, 24, 9, 16, 5, 0);
    qtest_writel(qts, B_YUV_CFG, 1u << 1);
    dsi_dma(qts, MEM, sizeof(yuv));
    dsi_video_up(qts);
    /* The first frame shown; VDMA not restarted, the next would underrun. */
    step(qts, 30 * NS_PER_US);
    g_free(px);
    px = screendump(qts, &w, &h);
    /*
     * TRM 39.5.1.1's BT.601 coefficients over 256, the fraction dropped:
     * Y 16 is black; Y 235 is (298 x 235 + 409 x 128 - 56906) >> 8 = 255,
     * G 255, B (298 x 235 + 516 x 128 - 70836) >> 8 = 254.
     */
    g_assert_cmphex(px[0], ==, 0x000000);
    g_assert_cmphex(px[1], ==, 0xfffffe);
    qtest_quit(qts);
}

/* The video pattern generator: vertical colour bars, the host's timing. */
static void test_dsi_vpg(void)
{
    QTestState *qts = machine("-device esp32p4-dsi-panel,width=16,height=8");
    static const uint32_t bars[8] = {
        0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
        0xff00ff, 0xff0000, 0x0000ff, 0x000000,
    };
    g_autofree uint32_t *px = NULL;
    int w, h;

    dsi_bus(qts, 200, 0x03, 2);
    dcs_write(qts, 0x11, NULL, 0);
    wait_cmd_empty(qts);
    qtest_writel(qts, D_PHY_TMR_CFG, 4u << 16 | 8);
    qtest_writel(qts, D_DPI_COLOR_CODING, 5);
    qtest_writel(qts, D_VID_PKT_SIZE, PW);
    qtest_writel(qts, D_VID_HSA_TIME, 5);
    qtest_writel(qts, D_VID_HBP_TIME, 5);
    qtest_writel(qts, D_VID_HLINE_TIME, 60);
    qtest_writel(qts, D_VID_VSA_LINES, 1);
    qtest_writel(qts, D_VID_VBP_LINES, 1);
    qtest_writel(qts, D_VID_VACTIVE_LINES, PH);
    qtest_writel(qts, D_VID_VFP_LINES, 2);
    qtest_writel(qts, D_VID_MODE_CFG, 1u << 16);
    qtest_writel(qts, D_MODE_CFG, 0);
    /* 12 lines of 60 byte clocks (40 ns): 28.8 us a frame. */
    step(qts, 40 * NS_PER_US);
    px = screendump(qts, &w, &h);
    for (int y = 0; y < PH; y++) {
        for (int x = 0; x < PW; x++) {
            g_assert_cmphex(px[y * PW + x], ==, bars[x / 2]);
        }
    }
    qtest_quit(qts);
}

/* ---- The CSI host ---- */

static void csi_phy_write(QTestState *qts, uint8_t code, uint8_t data)
{
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, (1u << 16) | code);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 2);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, data);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 2);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
}

/* The host and its PHY as ESP-IDF's mipi_csi_hal_init() sets them. */
static void csi_up(QTestState *qts, uint8_t range, uint32_t lanes)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL03);

    qtest_writel(qts, PERI_CLK_CTRL03, (v & ~(3u << 2)) | 1u << 4);
    qtest_writel(qts, C_PHY_SHUTDOWNZ, 0);
    qtest_writel(qts, C_DPHY_RSTZ, 0);
    qtest_writel(qts, C_CSI2_RESETN, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, 1u << 16);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 1);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    csi_phy_write(qts, 0x44, range << 1);
    qtest_writel(qts, C_PHY_SHUTDOWNZ, 1);
    qtest_writel(qts, C_DPHY_RSTZ, 1);
    qtest_writel(qts, C_CSI2_RESETN, 1);
    qtest_writel(qts, C_N_LANES, lanes - 1);
    qtest_writel(qts, C_VC_EXTENSION, 1);
    /* Up 1000 cycles of the 20 MHz configuration clock later. */
    step(qts, 50 * NS_PER_US);
}

static uint8_t ecc(uint32_t h)
{
    static const uint32_t parity[6] = {
        0xf12cb7, 0xf2555b, 0x749a6d, 0xb8e38e, 0xdf03f0, 0xeffc00,
    };
    uint8_t e = 0;

    for (int i = 0; i < 6; i++) {
        e |= (__builtin_popcount(h & parity[i]) & 1) << i;
    }
    return e;
}

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xffff;

    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
        }
    }
    return crc;
}

/*
 * Send a packet through the host's hook: @hdr_flip and @crc_flip flip bits
 * of the header and checksum; @cut drops trailing bytes.
 */
static void csi_send(QTestState *qts, uint8_t vc, uint8_t dt, uint16_t wc,
                     const uint8_t *pld, uint32_t hdr_flip,
                     uint16_t crc_flip, uint32_t cut)
{
    uint32_t h = vc << 6 | dt | (uint32_t)wc << 8;
    uint32_t n = 4 + (pld ? wc + 2 : 0);
    g_autofree uint8_t *b = g_malloc(n);
    g_autofree char *hex = g_malloc(2 * n + 1);
    QDict *r;

    b[0] = h ^ hdr_flip;
    b[1] = (h ^ hdr_flip) >> 8;
    b[2] = (h ^ hdr_flip) >> 16;
    b[3] = ecc(h);
    if (pld) {
        uint16_t c = crc16(pld, wc) ^ crc_flip;

        memcpy(b + 4, pld, wc);
        b[4 + wc] = c;
        b[5 + wc] = c >> 8;
    }
    n -= cut;
    for (uint32_t i = 0; i < n; i++) {
        sprintf(hex + 2 * i, "%02x", b[i]);
    }
    hex[2 * n] = 0;
    r = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': {'path': "
                  "'/machine/csi-host', 'property': 'packet', 'value': %s}}",
                  hex);
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
}

static void test_csi_registers(void)
{
    QTestState *qts = machine(NULL);

    g_assert_cmphex(qtest_readl(qts, C_VERSION), ==, 0x3135302a);
    g_assert_cmphex(qtest_readl(qts, C_N_LANES), ==, 1);
    g_assert_cmphex(qtest_readl(qts, C_PHY_TEST_CTRL0), ==, 1);
    g_assert_cmphex(qtest_readl(qts, C_SCRAMBLING_SEED1), ==, 0x1008);
    g_assert_cmphex(qtest_readl(qts, C_SCRAMBLING_SEED2), ==, 0x1188);
    g_assert_cmphex(qtest_readl(qts, C_PHY_RX), ==, 1u << 16);
    g_assert_cmphex(qtest_readl(qts, C_PHY_STOPSTATE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CB_DMA_REQ_CFG), ==, 0x20080);
    g_assert_cmphex(qtest_readl(qts, CB_BUF_FLOW_CTL), ==, 0x7f8);
    g_assert_cmphex(qtest_readl(qts, CB_DATA_TYPE_CFG), ==, 0x2f18);
    g_assert_cmphex(qtest_readl(qts, CB_FRAME_CFG), ==, 0x011e01e0);
    g_assert_cmphex(qtest_readl(qts, CB_DMABLK_SIZE), ==, 0x1fff);
    g_assert_cmphex(qtest_readl(qts, CB_HOST_CTRL), ==, 3);
    g_assert_cmphex(qtest_readl(qts, CB_HOST_CM_CTRL), ==, 0x803);

    /* Force: the status reads set; the mask lets it out; read clears. */
    qtest_writel(qts, C_FORCE_PKT_FATAL, 2);
    g_assert_false(intr(qts, SRC_CSI));
    qtest_writel(qts, C_MSK_PKT_FATAL, 3);
    g_assert_true(intr(qts, SRC_CSI));
    g_assert_cmphex(qtest_readl(qts, C_INT_ST_MAIN), ==, 2);
    g_assert_cmphex(qtest_readl(qts, C_ST_PKT_FATAL), ==, 2);
    qtest_writel(qts, C_FORCE_PKT_FATAL, 0);
    g_assert_false(intr(qts, SRC_CSI));
    g_assert_cmphex(qtest_readl(qts, C_ST_PKT_FATAL), ==, 0);

    /* Test code 0x44's loopback, cleared by TESTCLR. */
    csi_phy_write(qts, 0x44, 0x0a);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 2);
    qtest_writel(qts, C_PHY_TEST_CTRL1, 1u << 16 | 0x44);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 0);
    qtest_writel(qts, C_PHY_TEST_CTRL1, 0);
    g_assert_cmphex(qtest_readl(qts, C_PHY_TEST_CTRL1) >> 8, ==, 0x0a);
    qtest_writel(qts, C_PHY_TEST_CTRL0, 1);
    g_assert_cmphex(qtest_readl(qts, C_PHY_TEST_CTRL1) >> 8, ==, 0);
    qtest_quit(qts);
}

static void test_csi_packets(void)
{
    QTestState *qts = machine(NULL);
    uint8_t line[12];

    for (int i = 0; i < 12; i++) {
        line[i] = i * 7 + 1;
    }
    csi_up(qts, 0x00, 2);
    qtest_writel(qts, C_MSK_PLD_CRC, 0xffff);

    /* A frame: FS 1, a line, FE 1: no errors. */
    csi_send(qts, 0, 0x00, 1, NULL, 0, 0, 0);
    csi_send(qts, 0, 0x2a, 12, line, 0, 0, 0);
    csi_send(qts, 0, 0x01, 1, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_INT_ST_MAIN), ==, 0);

    /* A single header bit wrong is corrected; two are not. */
    csi_send(qts, 1, 0x00, 2, NULL, 0, 0, 0);
    csi_send(qts, 1, 0x2a, 12, line, 1u << 13, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_ECC_CORRECTED), ==, 2);
    csi_send(qts, 1, 0x2a, 12, line, 3u << 13, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_PKT_FATAL), ==, 1);
    /* A bad checksum on VC 1, and at its frame's end. */
    csi_send(qts, 1, 0x2a, 12, line, 0, 0x100, 0);
    g_assert_true(intr(qts, SRC_CSI));
    g_assert_cmphex(qtest_readl(qts, C_INT_ST_MAIN), ==, 1u << 5);
    g_assert_cmphex(qtest_readl(qts, C_ST_PLD_CRC), ==, 2);
    g_assert_false(intr(qts, SRC_CSI));
    csi_send(qts, 1, 0x01, 2, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_CRC_FRAME), ==, 2);
    /* A payload cut short. */
    csi_send(qts, 0, 0x2a, 12, line, 0, 0, 3);
    g_assert_cmphex(qtest_readl(qts, C_ST_PKT_FATAL), ==, 2);
    /* A reserved data type. */
    csi_send(qts, 2, 0x26, 12, line, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_DATA_ID), ==, 4);
    /* FE without FS; FS numbers not in sequence; FS twice. */
    csi_send(qts, 3, 0x01, 5, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_BNDRY), ==, 8);
    csi_send(qts, 0, 0x00, 3, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_SEQ), ==, 1);
    csi_send(qts, 0, 0x00, 4, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_BNDRY), ==, 1);
    g_assert_cmphex(qtest_readl(qts, C_ST_SEQ), ==, 0);
    /* With CSI2_RESETN clear nothing is received. */
    qtest_writel(qts, C_CSI2_RESETN, 0);
    csi_send(qts, 0, 0x26, 12, line, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, C_ST_DATA_ID), ==, 0);
    qtest_quit(qts);
}

/* ---- CSI bridge and CM, read from the bridge's port ---- */

static void cb_read(QTestState *qts, uint8_t *out, uint32_t n)
{
    for (uint32_t i = 0; i < n; i += 4) {
        uint32_t w = qtest_readl(qts, CSI_MEM);

        for (int k = 0; k < 4 && i + k < n; k++) {
            out[i + k] = w >> (8 * k);
        }
    }
}

static void test_csi_bridge(void)
{
    QTestState *qts = machine(NULL);
    uint8_t rgb[24], out[24];

    for (int i = 0; i < 24; i++) {
        rgb[i] = i * 11 + 3;
    }
    csi_up(qts, 0x00, 2);
    qtest_writel(qts, CB_DATA_TYPE_CFG, 0x2f12);
    qtest_writel(qts, CB_INT_ENA, 0x3f);
    qtest_writel(qts, CB_CSI_EN, 1);

    /* Bypass: the payload as it came, in 64-bit words. */
    csi_send(qts, 0, 0x00, 1, NULL, 0, 0, 0);
    csi_send(qts, 0, 0x24, 24, rgb, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, ==, 3);
    csi_send(qts, 0, 0x01, 1, NULL, 0, 0, 0);
    cb_read(qts, out, 24);
    g_assert_cmpmem(out, 24, rgb, 24);

    /* RGB888 (RGB order: B, G, R bytes) to RGB565. */
    qtest_writel(qts, CB_HOST_CM_CTRL, 1 | 0u << 2 | 1u << 9);
    csi_send(qts, 0, 0x00, 2, NULL, 0, 0, 0);
    csi_send(qts, 0, 0x24, 24, rgb, 0, 0, 0);
    csi_send(qts, 0, 0x01, 2, NULL, 0, 0, 0);
    cb_read(qts, out, 16);
    for (int i = 0; i < 8; i++) {
        uint32_t b = rgb[3 * i], g = rgb[3 * i + 1], r = rgb[3 * i + 2];
        uint16_t v = (r >> 3) << 11 | (g >> 2) << 5 | b >> 3;

        g_assert_cmphex(out[2 * i] | out[2 * i + 1] << 8, ==, v);
    }

    /* YUV422 (YUYV in) to YUV420: U on the first row, V on the next. */
    qtest_writel(qts, CB_HOST_CM_CTRL, 1 | 2u << 2 | 1u << 7 | 3u << 9);
    csi_send(qts, 0, 0x00, 3, NULL, 0, 0, 0);
    csi_send(qts, 0, 0x1e, 8, (const uint8_t[]){ 1, 2, 3, 4, 5, 6, 7, 8 },
             0, 0, 0);
    csi_send(qts, 0, 0x1e, 8, (const uint8_t[]){ 1, 2, 3, 4, 5, 6, 7, 8 },
             0, 0, 0);
    csi_send(qts, 0, 0x01, 3, NULL, 0, 0, 0);
    cb_read(qts, out, 16);
    g_assert_cmpmem(out, 12, ((const uint8_t[]){ 2, 1, 3, 6, 5, 7,
                                                 4, 1, 3, 8, 5, 7 }), 12);

    /* Data types outside MIN..MAX are left out; the swap bits apply. */
    qtest_writel(qts, CB_HOST_CM_CTRL, 1 | 2 | 1u << 13);
    csi_send(qts, 0, 0x00, 4, NULL, 0, 0, 0);
    csi_send(qts, 0, 0x30, 8, rgb, 0, 0, 0);
    csi_send(qts, 0, 0x2a, 8, rgb, 0, 0, 0);
    csi_send(qts, 0, 0x01, 4, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, CB_BUF_FLOW_CTL) >> 16, ==, 1);
    cb_read(qts, out, 8);
    g_assert_cmpmem(out, 8, ((const uint8_t[]){ rgb[1], rgb[0], rgb[3],
                                                rgb[2], rgb[5], rgb[4],
                                                rgb[7], rgb[6] }), 8);

    /* VADR_NUM checked: two lines against 1. */
    qtest_writel(qts, CB_HOST_CM_CTRL, 3);
    qtest_writel(qts, CB_FRAME_CFG, 1u << 25 | 1u << 24 | 1);
    csi_send(qts, 0, 0x00, 5, NULL, 0, 0, 0);
    csi_send(qts, 0, 0x2a, 8, rgb, 0, 0, 0);
    csi_send(qts, 0, 0x2a, 8, rgb, 0, 0, 0);
    csi_send(qts, 0, 0x01, 5, NULL, 0, 0, 0);
    g_assert_cmphex(qtest_readl(qts, CB_INT_RAW), ==, 2);
    g_assert_true(intr(qts, SRC_CSI_BRIDGE));
    qtest_writel(qts, CB_INT_CLR, 2);
    g_assert_false(intr(qts, SRC_CSI_BRIDGE));
    qtest_quit(qts);
}

/* ---- CSI capture from the sensor through VDMA ---- */

#define CW  32
#define CH  8

/* The sensor's "bars" pattern, RAW8 RGGB, frame @f (image source header). */
static uint8_t bars_raw8(uint32_t f, uint32_t x, uint32_t y)
{
    static const uint8_t rgb[8][3] = {
        { 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 0, 255, 0 },
        { 255, 0, 255 }, { 255, 0, 0 }, { 0, 0, 255 }, { 0, 0, 0 },
    };
    uint32_t step = MAX(CW / 64, 1), bar = ((x + f * step) % CW) * 8 / CW;
    int c = (y & 1) ? ((x & 1) ? 2 : 1) : ((x & 1) ? 1 : 0);
    uint32_t cell = 2;

    /* The frame number in the top left: 16 squares, bit 15 first. */
    if (y < cell && x < MIN(16 * cell, CW)) {
        return (f >> (15 - x / cell)) & 1 ? 255 : 0;
    }
    return rgb[bar][c];
}

/* Step until VDMA channel 2's block is done, for up to @us; when it was. */
static int64_t csi_dma_wait(QTestState *qts, int us)
{
    for (int i = 0; i < us; i++) {
        step(qts, NS_PER_US);
        if (qtest_readl(qts, V_CH(2) + V_INTSTATUS) & DMA_TFR_DONE) {
            return vnow();
        }
    }
    g_assert_not_reached();
}

/* VDMA channel 2 from the bridge's port to MEM, a frame of CW x CH. */
static void csi_dma_start(QTestState *qts)
{
    qtest_writel(qts, V_CH(2) + V_INTCLEAR0, 0xffffffff);
    qtest_writel(qts, V_CH(2) + V_SAR0, CSI_MEM);
    qtest_writel(qts, V_CH(2) + V_DAR0, MEM);
    qtest_writel(qts, V_CH(2) + V_BLOCK_TS0, CW * CH / 8 - 1);
    qtest_writel(qts, V_CH(2) + V_CTL0, 1u << 2 | 1u << 4 | 3u << 8 |
                 3u << 11 | 8u << 14 | 8u << 18);
    qtest_writel(qts, V_CH(2) + V_CTL1, 0);
    qtest_writel(qts, V_CHEN0, 2 | 2u << 8);
}

/* The bars frame a capture in MEM shows, or -1. */
static int csi_frame_of(QTestState *qts)
{
    uint8_t got[CW * CH];

    qtest_memread(qts, MEM, got, sizeof(got));
    for (int f = 0; f < 4; f++) {
        bool ok = true;

        for (int y = 0; y < CH && ok; y++) {
            for (int x = 0; x < CW && ok; x++) {
                ok = got[y * CW + x] == bars_raw8(f, x, y);
            }
        }
        if (ok) {
            return f;
        }
    }
    return -1;
}

/*
 * Capture from the sensor, 32 x 8 RAW8 at 1000 frames a second, its frame
 * number in 2-pixel squares, its stream starting at power-on: frames of 11 lines (a VSYNC line, a back
 * porch line, 8 of pixels, a front porch line) of 48 pixel clocks (32 and
 * a horizontal blanking of 16) at 528 kHz, the lanes at 80 Mbit/s, the
 * lowest rate the line's burst fits. The bridge, enabled within frame 0,
 * takes frame 1. Its frame end packet arrives after the last line's long
 * packet, which starts with line 9 (818.18 us into the frame): 38 bytes
 * over 2 lanes (T_LPX, HS-PREPARE, HS-ZERO and the sync, 200 ns + 19 UI;
 * 19 bytes of 8 UI; HS-TRAIL, 60 ns + 4 UI; HS-EXIT, 100 ns: 2547.5 ns),
 * then the frame end packet's 4 bytes (437.5 ns + 16 UI = 637.5 ns), at
 * 1821.37 us. Then the bridge, the flow controller, asks for the frame's
 * last 32 words one at a time, the last marked, and VDMA's block is done
 * when they have moved. The next frame's is a frame period later.
 */
static void test_csi_capture(void)
{
    QTestState *qts = machine("-object esp32p4-image-source,id=cam0,"
                              "format=raw8,width=32,height=8,fps=1000,"
                              "cell=2 -M csi-camera=cam0");
    int64_t t1, t2;

    csi_up(qts, 0x00, 2);
    qtest_writel(qts, CB_DATA_TYPE_CFG, 0x2f12);
    qtest_writel(qts, CB_DMA_REQ_CFG, 0x20000 | 512);
    qtest_writel(qts, CB_FRAME_CFG, 1u << 24 | CW << 12 | CH);

    /*
     * VDMA channel 2, as esp_cam_ctlr_csi does: the bridge's port to
     * memory, the bridge the flow controller, 64-bit items in bursts of
     * 512.
     */
    qtest_writel(qts, V_CFG0, 3);
    qtest_writel(qts, V_CH(2) + V_CH_CFG0, 0);
    qtest_writel(qts, V_CH(2) + V_CH_CFG1, 4 | 1u << 4 | 1u << 7 | 1u << 17);
    qtest_writel(qts, V_CH(2) + V_INTST_ENA0, 0xffffffff);
    csi_dma_start(qts);
    g_assert_cmpint(vnow(), <, 1000 * NS_PER_US);
    qtest_writel(qts, CB_CSI_EN, 1);

    t1 = csi_dma_wait(qts, 2200);
    g_assert_cmpint(t1, >, 1821370);
    g_assert_cmpint(t1, <=, 1821370 + 10 * NS_PER_US);
    g_assert_cmpint(csi_frame_of(qts), ==, 1);

    csi_dma_start(qts);
    t2 = csi_dma_wait(qts, 1100);
    g_assert_cmpint(t2 - t1, >=, 1000 * NS_PER_US - NS_PER_US);
    g_assert_cmpint(t2 - t1, <=, 1000 * NS_PER_US + NS_PER_US);
    g_assert_cmpint(csi_frame_of(qts), ==, 2);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/mipi/dsi/registers", test_dsi_registers);
    qtest_add_func("esp32p4/mipi/dsi/phy", test_dsi_phy);
    qtest_add_func("esp32p4/mipi/dsi/commands", test_dsi_commands);
    qtest_add_func("esp32p4/mipi/dsi/no-panel", test_dsi_no_panel);
    qtest_add_func("esp32p4/mipi/dsi/video", test_dsi_video);
    qtest_add_func("esp32p4/mipi/dsi/formats", test_dsi_formats);
    qtest_add_func("esp32p4/mipi/dsi/vpg", test_dsi_vpg);
    qtest_add_func("esp32p4/mipi/csi/registers", test_csi_registers);
    qtest_add_func("esp32p4/mipi/csi/packets", test_csi_packets);
    qtest_add_func("esp32p4/mipi/csi/bridge", test_csi_bridge);
    qtest_add_func("esp32p4/mipi/csi/capture", test_csi_capture);
    return g_test_run();
}
