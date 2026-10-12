/*
 * QTest testcase for the ESP32-P4 USB 2.0 OTG controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The two DWC2 cores' registers, their clocks and resets, the USB_WRAP
 * and UTMI+ PHY registers, and host mode against QEMU's usb-kbd: port
 * connection, reset and enable, frames, and control transfers through a
 * channel in buffer DMA and scatter/gather DMA modes. qtest's accesses
 * reach memory directly, past the caches, as the cores' DMA does.
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include <sys/un.h>
#include "libqtest.h"

#define OTG_HS          0x50000000
#define OTG_FS          0x50040000
#define USB_WRAP        0x50080000
#define USB_UTMI        0x5009c000

#define GOTGCTL         0x000
#define GOTGINT         0x004
#define GAHBCFG         0x008
#define GUSBCFG         0x00c
#define GRSTCTL         0x010
#define GINTSTS         0x014
#define GINTMSK         0x018
#define GRXFSIZ         0x024
#define GNPTXFSIZ       0x028
#define GNPTXSTS        0x02c
#define GSNPSID         0x040
#define GHWCFG1         0x044
#define GHWCFG2         0x048
#define GHWCFG3         0x04c
#define GHWCFG4         0x050
#define HPTXFSIZ        0x100
#define HCFG            0x400
#define HFIR            0x404
#define HFNUM           0x408
#define HAINT           0x414
#define HAINTMSK        0x418
#define HPRT            0x440
#define HCCHAR(n)       (0x500 + 0x20 * (n))
#define HCINT(n)        (0x508 + 0x20 * (n))
#define HCINTMSK(n)     (0x50c + 0x20 * (n))
#define HCTSIZ(n)       (0x510 + 0x20 * (n))
#define HCDMA(n)        (0x514 + 0x20 * (n))
#define GRXSTSP         0x020
#define DCFG            0x800
#define DCTL            0x804
#define DSTS            0x808
#define DIEPCTL(n)      (0x900 + 0x20 * (n))
#define DIEPINT(n)      (0x908 + 0x20 * (n))
#define DIEPTSIZ(n)     (0x910 + 0x20 * (n))
#define DTXFSTS(n)      (0x918 + 0x20 * (n))
#define DOEPCTL(n)      (0xb00 + 0x20 * (n))
#define DOEPINT(n)      (0xb08 + 0x20 * (n))
#define DOEPTSIZ(n)     (0xb10 + 0x20 * (n))
#define PCGCCTL         0xe00
#define FIFO(n)         (0x1000 + 0x1000 * (n))

#define GOTGCTL_CONID_B         (1u << 16)
#define GOTGCTL_CURMOD          (1u << 21)
#define GAHBCFG_GLBL_INTR_EN    (1u << 0)
#define GAHBCFG_DMA_EN          (1u << 5)
#define GUSBCFG_PHYSEL          (1u << 6)
#define GUSBCFG_PHYIF16         (1u << 3)
#define GUSBCFG_FORCEHOSTMODE   (1u << 29)
#define GUSBCFG_FORCEDEVMODE    (1u << 30)
#define GRSTCTL_CSFTRST         (1u << 0)
#define GRSTCTL_CSFTRST_DONE    (1u << 29)
#define GRSTCTL_AHBIDLE         (1u << 31)
#define GINTSTS_CURMOD_HOST     (1u << 0)
#define GINTSTS_MODEMIS         (1u << 1)
#define GINTSTS_SOF             (1u << 3)
#define GINTSTS_NPTXFEMP        (1u << 5)
#define GINTSTS_PRTINT          (1u << 24)
#define GINTSTS_HCHINT          (1u << 25)
#define GINTSTS_PTXFEMP         (1u << 26)
#define GINTSTS_CONIDSTSCHNG    (1u << 28)
#define GINTSTS_DISCONNINT      (1u << 29)
#define HCFG_FSLSPCLKSEL_48     1
#define HCFG_DESCDMA            (1u << 23)
#define HPRT_CONNSTS            (1u << 0)
#define HPRT_CONNDET            (1u << 1)
#define HPRT_ENA                (1u << 2)
#define HPRT_ENACHG             (1u << 3)
#define HPRT_RST                (1u << 8)
#define HPRT_PWR                (1u << 12)
#define HPRT_SPD(v)             (((v) >> 17) & 3)
#define HCCHAR_MPS(n)           (n)
#define HCCHAR_EPDIR_IN         (1u << 15)
#define HCCHAR_DEVADDR(a)       ((a) << 22)
#define HCCHAR_CHDIS            (1u << 30)
#define HCCHAR_CHENA            (1u << 31)
#define HCINT_XFERCOMPL         (1u << 0)
#define HCINT_CHHLTD            (1u << 1)
#define HCINT_ACK               (1u << 5)
#define HCINT_BNA               (1u << 11)
#define TSIZ(size, pkts, pid)   ((size) | ((pkts) << 19) | ((pid) << 29))
#define PID_DATA0               0
#define PID_DATA1               2
#define PID_SETUP               3
#define DCTL_SFTDISCON          (1u << 1)
#define GOTGCTL_BSESVLD         (1u << 19)
#define GINTSTS_RXFLVL          (1u << 4)
#define GINTSTS_ERLYSUSP        (1u << 10)
#define GINTSTS_USBSUSP         (1u << 11)
#define GINTSTS_USBRST          (1u << 12)
#define GINTSTS_ENUMDONE        (1u << 13)
#define GINTSTS_WKUPINT         (1u << 31)
#define HPRT_RES                (1u << 6)
#define HPRT_SUSP               (1u << 7)
#define HCINT_AHBERR            (1u << 2)
#define GRXSTS_BCNT(v)          (((v) >> 4) & 0x7ff)
#define GRXSTS_PKTSTS(v)        (((v) >> 17) & 0xf)
#define DSTS_SUSPSTS            (1u << 0)
#define DSTS_ENUMSPD(v)         (((v) >> 1) & 3)
#define DSTS_SOFFN(v)           (((v) >> 8) & 0x3fff)
#define DEPCTL_NAKSTS           (1u << 17)
#define DEPCTL_STALL            (1u << 21)
#define DEPCTL_CNAK             (1u << 26)
#define DEPCTL_EPENA            (1u << 31)
#define DEPINT_XFERCOMPL        (1u << 0)
#define DEPINT_SETUP            (1u << 3)
#define DEPINT_OUTTKNEPDIS      (1u << 4)
#define DOEPTSIZ_SUPCNT(v)      (((v) >> 29) & 3)
#define PCGCCTL_GATEHCLK        (1u << 1)

/* Scatter/gather qTD status quadlet. */
#define QTD_ACTIVE              (1u << 31)
#define QTD_STS(v)              (((v) >> 28) & 3)
#define QTD_EOL                 (1u << 26)
#define QTD_IOC                 (1u << 25)
#define QTD_SUP                 (1u << 24)
#define QTD_NBYTES(v)           ((v) & 0x1ffff)

#define HP_SYS_CLKRST_SOC_CLK_CTRL1     0x500e6018
#define USB_OTG11_SYS_CLK_EN            (1u << 15)
#define LP_CLKRST_HP_USB_CLKRST_CTRL0   0x50111044
#define USB_OTG11_48M_CLK_EN            (1u << 4)
#define PLL_F48M_DIV_MASK               (0xffu << 6)
#define PLL_F48M_DIV(n)                 (((n) - 1u) << 6)
#define LP_CLKRST_HP_USB_CLKRST_CTRL1   0x50111048
#define RST_EN_USB_OTG11                (1u << 3)
#define RST_EN_USB_OTG20                (1u << 2)

#define HP_SYS_USBOTG20_CTRL            0x500e515c
#define PHY_PLL_FORCE_EN                (1u << 19)
#define PHY_PLL_EN                      (1u << 20)
#define HP_SYS_USB_OTGHS_PHY_ST         0x500e51e8
#define LP_SYS_USB_CTRL                 0x50110100
#define LP_SYS_HP_USB_OTGHS_PHY_CTRL    0x50110204
#define OTG_CONF_PAD_PULL_OVERRIDE      (1u << 12)
#define OTG_CONF_DP_PULLUP              (1u << 13)
#define OTG_CONF_DP_PULLDOWN            (1u << 14)
#define OTG_CONF_DM_PULLDOWN            (1u << 16)
#define OTG_CONF_USB_PAD_ENABLE         (1u << 18)
#define GPIO_FUNC_IN(sig)               (0x500e0158 + 4 * (sig))
#define GPIO_IN_SEL                     (1u << 7)
#define GPIO_CONST_ONE                  0x3f

#define INTMTX_STATUS(src)  (0x500d6200 + 4 * ((src) / 32))
#define DMA_PMS_USB_W       0x500a61b0
#define ICM_INT_RAW         0x500a400c
#define ICM_CPU_ADDR        0x500a4040
#define ICM_CPU_INFO        0x500a4044
#define ICM_INT_CPU         0x4
#define ICM_WR              0x100
#define ICM_ID_USB_OTG_HS   7
#define SRC_USB_OTG         93
#define SRC_USB_OTG11       103

/* DMA buffers in L2MEM, past the part the L2 cache takes from reset. */
#define BUF_SETUP       0x4ff40000
#define BUF_DATA        0x4ff40100
#define QTD_LIST        0x4ff40400

#define HS_FRAME_NS     (125 * 1000)

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t off)
{
    return qtest_readl(qts, base + off);
}

static void wr(QTestState *qts, uint32_t base, uint32_t off, uint32_t v)
{
    qtest_writel(qts, base + off, v);
}

static void fs_clock_on(QTestState *qts)
{
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1,
                 qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1) |
                 USB_OTG11_SYS_CLK_EN);
}

static bool irq_source(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src)) >> (src % 32)) & 1;
}

/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_reset_values(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* OTG_HS: a 4.30a core, in device mode (its ID floats high). */
    g_assert_cmphex(rd(qts, OTG_HS, GSNPSID), ==, 0x4f54430a);
    g_assert_cmphex(rd(qts, OTG_HS, GHWCFG1), ==, 0);
    g_assert_cmphex(rd(qts, OTG_HS, GHWCFG2), ==, 0x215fffd0);
    g_assert_cmphex(rd(qts, OTG_HS, GHWCFG3), ==, 0x03805eb5);
    g_assert_cmphex(rd(qts, OTG_HS, GHWCFG4), ==, 0xdff1a030);
    g_assert_cmphex(rd(qts, OTG_HS, GUSBCFG), ==, 0x1400);
    g_assert_cmphex(rd(qts, OTG_HS, GRSTCTL), ==, GRSTCTL_AHBIDLE);
    g_assert_cmphex(rd(qts, OTG_HS, GAHBCFG), ==, 0);
    g_assert_cmphex(rd(qts, OTG_HS, GINTMSK), ==, 0);
    g_assert_cmphex(rd(qts, OTG_HS, GRXFSIZ), ==, 1024);
    g_assert_cmphex(rd(qts, OTG_HS, GOTGCTL), ==, GOTGCTL_CONID_B);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_CURMOD_HOST, ==, 0);
    g_assert_cmphex(rd(qts, OTG_HS, DCTL), ==, DCTL_SFTDISCON);
    g_assert_cmphex(rd(qts, OTG_HS, DCFG), ==, 0x08000000);
    g_assert_cmphex(rd(qts, OTG_HS, PCGCCTL), ==, 0);

    /* OTG_FS: a 4.00a core with only the full-speed PHY. */
    g_assert_cmphex(rd(qts, OTG_FS, GSNPSID), ==, 0x4f54400a);
    g_assert_cmphex(rd(qts, OTG_FS, GHWCFG2), ==, 0x224dd930);
    g_assert_cmphex(rd(qts, OTG_FS, GHWCFG3), ==, 0x00c804b5);
    g_assert_cmphex(rd(qts, OTG_FS, GHWCFG4), ==, 0xd3f0a030);
    g_assert_cmphex(rd(qts, OTG_FS, GUSBCFG), ==, 0x1440);
    g_assert_cmphex(rd(qts, OTG_FS, GRXFSIZ), ==, 256);

    /* USB_WRAP, the UTMI+ PHY, and their bits of HP_SYSTEM and LP_SYS. */
    g_assert_cmphex(qtest_readl(qts, USB_WRAP), ==, 0x00100000);
    g_assert_cmphex(qtest_readl(qts, USB_WRAP + 0x3fc), ==, 0x23030504);
    g_assert_cmphex(qtest_readl(qts, USB_UTMI + 0x00), ==, 0x80);
    g_assert_cmphex(qtest_readl(qts, USB_UTMI + 0x04), ==, 0xf2);
    g_assert_cmphex(qtest_readl(qts, USB_UTMI + 0x08), ==, 0x47);
    g_assert_cmphex(qtest_readl(qts, USB_UTMI + 0x0c), ==, 0x05);
    g_assert_cmphex(qtest_readl(qts, USB_UTMI + 0x14), ==, 0xc1);
    g_assert_cmphex(qtest_readl(qts, USB_UTMI + 0x18), ==, 0x40);
    g_assert_cmphex(qtest_readl(qts, HP_SYS_USBOTG20_CTRL), ==, 0x00822640);
    /* No VBUS: session end. */
    g_assert_cmphex(qtest_readl(qts, HP_SYS_USB_OTGHS_PHY_ST), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_USB_CTRL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_HP_USB_OTGHS_PHY_CTRL), ==, 7);

    qtest_quit(qts);
}

/*
 * Registers of the other mode are ignored and raise ModeMis; the
 * read-only and write-1-to-clear fields hold.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_register_access(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    wr(qts, OTG_HS, HCFG, 1);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_MODEMIS, ==,
                    GINTSTS_MODEMIS);
    wr(qts, OTG_HS, GINTSTS, GINTSTS_MODEMIS);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_MODEMIS, ==, 0);
    g_assert_cmphex(rd(qts, OTG_HS, HPRT), ==, 0);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_MODEMIS, ==,
                    GINTSTS_MODEMIS);

    wr(qts, OTG_HS, GSNPSID, 0);
    g_assert_cmphex(rd(qts, OTG_HS, GSNPSID), ==, 0x4f54430a);
    /* PHYSel is read-only on each core: one of them has no UTMI+ PHY. */
    wr(qts, OTG_HS, GUSBCFG, 0x1400 | GUSBCFG_PHYSEL | GUSBCFG_PHYIF16);
    g_assert_cmphex(rd(qts, OTG_HS, GUSBCFG), ==, 0x1400 | GUSBCFG_PHYIF16);
    fs_clock_on(qts);
    wr(qts, OTG_FS, GUSBCFG, 0x1400);
    g_assert_cmphex(rd(qts, OTG_FS, GUSBCFG), ==, 0x1440);

    /* With the AHB clock gated only PCGCCTL answers. */
    wr(qts, OTG_HS, PCGCCTL, PCGCCTL_GATEHCLK);
    g_assert_cmphex(rd(qts, OTG_HS, GSNPSID), ==, 0);
    wr(qts, OTG_HS, GAHBCFG, GAHBCFG_DMA_EN);
    wr(qts, OTG_HS, PCGCCTL, 0);
    g_assert_cmphex(rd(qts, OTG_HS, GAHBCFG), ==, 0);
    qtest_quit(qts);
}

/*
 * OTG_FS's registers are behind USB_OTG11_SYS_CLK, off from reset, and
 * RST_EN_USB_OTG11; USB_WRAP shares them.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_clock_and_reset(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    wr(qts, OTG_FS, GAHBCFG, GAHBCFG_DMA_EN);
    qtest_writel(qts, USB_WRAP, 1u << 18);
    g_assert_cmphex(rd(qts, OTG_FS, GAHBCFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, USB_WRAP), ==, 0x00100000);

    fs_clock_on(qts);
    wr(qts, OTG_FS, GAHBCFG, GAHBCFG_DMA_EN);
    qtest_writel(qts, USB_WRAP, 1u << 18);
    g_assert_cmphex(rd(qts, OTG_FS, GAHBCFG), ==, GAHBCFG_DMA_EN);
    g_assert_cmphex(qtest_readl(qts, USB_WRAP), ==, 1u << 18);

    /* The reset returns both to their reset state and holds them there. */
    qtest_writel(qts, LP_CLKRST_HP_USB_CLKRST_CTRL1,
                 qtest_readl(qts, LP_CLKRST_HP_USB_CLKRST_CTRL1) |
                 RST_EN_USB_OTG11);
    g_assert_cmphex(rd(qts, OTG_FS, GAHBCFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, USB_WRAP), ==, 0x00100000);
    wr(qts, OTG_FS, GAHBCFG, GAHBCFG_DMA_EN);
    g_assert_cmphex(rd(qts, OTG_FS, GAHBCFG), ==, 0);
    qtest_writel(qts, LP_CLKRST_HP_USB_CLKRST_CTRL1,
                 qtest_readl(qts, LP_CLKRST_HP_USB_CLKRST_CTRL1) &
                 ~RST_EN_USB_OTG11);
    wr(qts, OTG_FS, GAHBCFG, GAHBCFG_DMA_EN);
    g_assert_cmphex(rd(qts, OTG_FS, GAHBCFG), ==, GAHBCFG_DMA_EN);
    qtest_quit(qts);
}

/*
 * The core soft reset: from 4.20a CSftRstDone flags its end and CSftRst
 * stays set until cleared; before, CSftRst clears itself. Either waits
 * for the PHY clock.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_soft_reset(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    wr(qts, OTG_HS, GINTMSK, 0xfc00);
    wr(qts, OTG_HS, GUSBCFG, 0x2400 | GUSBCFG_PHYIF16);
    wr(qts, OTG_HS, GRSTCTL, GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_HS, GRSTCTL), ==,
                    GRSTCTL_AHBIDLE | GRSTCTL_CSFTRST_DONE |
                    GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_HS, GINTMSK), ==, 0);
    /* PHYIf survives; the turnaround time does not. */
    g_assert_cmphex(rd(qts, OTG_HS, GUSBCFG), ==, 0x1400 | GUSBCFG_PHYIF16);
    wr(qts, OTG_HS, GRSTCTL, GRSTCTL_CSFTRST_DONE);
    g_assert_cmphex(rd(qts, OTG_HS, GRSTCTL), ==, GRSTCTL_AHBIDLE);

    /* HP_SYSTEM forcing the UTMI+ PHY's PLL off stops its clock. */
    qtest_writel(qts, HP_SYS_USBOTG20_CTRL,
                 qtest_readl(qts, HP_SYS_USBOTG20_CTRL) | PHY_PLL_FORCE_EN);
    wr(qts, OTG_HS, GINTMSK, 0xfc00);
    wr(qts, OTG_HS, GRSTCTL, GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_HS, GRSTCTL), ==,
                    GRSTCTL_AHBIDLE | GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_HS, GINTMSK), ==, 0xfc00);
    qtest_writel(qts, HP_SYS_USBOTG20_CTRL,
                 qtest_readl(qts, HP_SYS_USBOTG20_CTRL) | PHY_PLL_EN);
    g_assert_cmphex(rd(qts, OTG_HS, GRSTCTL), ==,
                    GRSTCTL_AHBIDLE | GRSTCTL_CSFTRST_DONE |
                    GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_HS, GINTMSK), ==, 0);

    /* OTG_FS without its 48 MHz PHY clock: the reset waits for it. */
    fs_clock_on(qts);
    qtest_writel(qts, LP_CLKRST_HP_USB_CLKRST_CTRL0,
                 qtest_readl(qts, LP_CLKRST_HP_USB_CLKRST_CTRL0) &
                 ~USB_OTG11_48M_CLK_EN);
    wr(qts, OTG_FS, GINTMSK, 0xfc00);
    wr(qts, OTG_FS, GRSTCTL, GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_FS, GRSTCTL), ==,
                    GRSTCTL_AHBIDLE | GRSTCTL_CSFTRST);
    g_assert_cmphex(rd(qts, OTG_FS, GINTMSK), ==, 0xfc00);
    qtest_writel(qts, LP_CLKRST_HP_USB_CLKRST_CTRL0,
                 qtest_readl(qts, LP_CLKRST_HP_USB_CLKRST_CTRL0) |
                 USB_OTG11_48M_CLK_EN);
    g_assert_cmphex(rd(qts, OTG_FS, GRSTCTL), ==, GRSTCTL_AHBIDLE);
    g_assert_cmphex(rd(qts, OTG_FS, GINTMSK), ==, 0);
    qtest_quit(qts);
}

/* ForceHstMode takes effect 25 ms after it is written. */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_force_mode(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    wr(qts, OTG_HS, GUSBCFG, 0x1400 | GUSBCFG_FORCEHOSTMODE);
    qtest_clock_step(qts, 24 * 1000 * 1000);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_CURMOD_HOST, ==, 0);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_CURMOD_HOST, ==,
                    GINTSTS_CURMOD_HOST);
    g_assert_cmphex(rd(qts, OTG_HS, GOTGCTL) & GOTGCTL_CURMOD, ==,
                    GOTGCTL_CURMOD);
    /* An idle host's transmit FIFOs are empty. */
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) &
                    (GINTSTS_NPTXFEMP | GINTSTS_PTXFEMP), ==,
                    GINTSTS_NPTXFEMP | GINTSTS_PTXFEMP);
    qtest_quit(qts);
}

/*
 * Bring @base's port up with a device on it: host mode, port power,
 * connection, a 10 ms reset and the port enabled. Returns HPRT.
 */
static uint32_t port_up(QTestState *qts, uint32_t base)
{
    uint32_t hprt;

    wr(qts, base, GUSBCFG, rd(qts, base, GUSBCFG) | GUSBCFG_FORCEHOSTMODE);
    qtest_clock_step(qts, 25 * 1000 * 1000);
    g_assert_cmphex(rd(qts, base, GINTSTS) & GINTSTS_CURMOD_HOST, ==,
                    GINTSTS_CURMOD_HOST);
    g_assert_cmphex(rd(qts, base, HPRT), ==, 0);
    wr(qts, base, HPRT, HPRT_PWR);
    hprt = rd(qts, base, HPRT);
    g_assert_cmphex(hprt & (HPRT_CONNSTS | HPRT_CONNDET | HPRT_ENA), ==,
                    HPRT_CONNSTS | HPRT_CONNDET);
    g_assert_cmphex(rd(qts, base, GINTSTS) & GINTSTS_PRTINT, ==,
                    GINTSTS_PRTINT);
    wr(qts, base, HPRT, HPRT_PWR | HPRT_CONNDET | HPRT_RST);
    g_assert_cmphex(rd(qts, base, GINTSTS) & GINTSTS_PRTINT, ==, 0);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    wr(qts, base, HPRT, HPRT_PWR);
    hprt = rd(qts, base, HPRT);
    g_assert_cmphex(hprt & (HPRT_CONNSTS | HPRT_ENA | HPRT_ENACHG), ==,
                    HPRT_CONNSTS | HPRT_ENA | HPRT_ENACHG);
    wr(qts, base, HPRT, HPRT_PWR | HPRT_ENACHG);
    return hprt;
}

/* Run channel 0 and wait for it to halt; returns HCINT. */
static uint32_t run_chan(QTestState *qts, uint32_t base, uint32_t hcchar)
{
    uint32_t hcint;
    int i;

    wr(qts, base, HCINT(0), 0x3fff);
    wr(qts, base, HCCHAR(0), hcchar | HCCHAR_CHENA);
    for (i = 0; i < 100; i++) {
        qtest_clock_step(qts, HS_FRAME_NS);
        if (!(rd(qts, base, HCCHAR(0)) & HCCHAR_CHENA)) {
            break;
        }
    }
    hcint = rd(qts, base, HCINT(0));
    wr(qts, base, HCINT(0), hcint);
    return hcint;
}

static const uint8_t get_device_desc[8] = {
    0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 18, 0,
};

/*
 * OTG_HS as a host in buffer DMA mode: usb-kbd connects, the reset
 * enables the port at high speed, SOFs count microframes, and channel 0
 * reads the device descriptor with a control transfer.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_hs_host_buffer_dma(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -device "
                                 "usb-kbd,bus=usb-otg-hs.0,port=1,id=kbd");
    uint32_t hprt, hcint, frnum, hcchar;
    uint8_t desc[18];

    hprt = port_up(qts, OTG_HS);
    g_assert_cmpint(HPRT_SPD(hprt), ==, 0);

    wr(qts, OTG_HS, GAHBCFG, GAHBCFG_DMA_EN | GAHBCFG_GLBL_INTR_EN);
    /* The (micro)frame interval takes effect at the next SOF. */
    wr(qts, OTG_HS, HFIR, 7500);
    qtest_clock_step(qts, 8 * HS_FRAME_NS);
    frnum = rd(qts, OTG_HS, HFNUM) & 0x3fff;
    qtest_clock_step(qts, 8 * HS_FRAME_NS);
    g_assert_cmpint((rd(qts, OTG_HS, HFNUM) - frnum) & 0x3fff, ==, 8);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_SOF, ==, GINTSTS_SOF);

    wr(qts, OTG_HS, GINTMSK, GINTSTS_HCHINT);
    wr(qts, OTG_HS, HAINTMSK, 1);
    wr(qts, OTG_HS, HCINTMSK(0), HCINT_XFERCOMPL | HCINT_CHHLTD);

    /* SETUP */
    qtest_memwrite(qts, BUF_SETUP, get_device_desc, 8);
    hcchar = HCCHAR_MPS(64) | HCCHAR_DEVADDR(0);
    wr(qts, OTG_HS, HCTSIZ(0), TSIZ(8, 1, PID_SETUP));
    wr(qts, OTG_HS, HCDMA(0), BUF_SETUP);
    wr(qts, OTG_HS, HCINT(0), 0x3fff);
    wr(qts, OTG_HS, HCCHAR(0), hcchar | HCCHAR_CHENA);
    g_assert_false(irq_source(qts, SRC_USB_OTG));
    qtest_clock_step(qts, HS_FRAME_NS);
    g_assert_true(irq_source(qts, SRC_USB_OTG));
    g_assert_cmphex(rd(qts, OTG_HS, HAINT), ==, 1);
    hcint = rd(qts, OTG_HS, HCINT(0));
    wr(qts, OTG_HS, HCINT(0), hcint);
    g_assert_cmphex(hcint, ==, HCINT_XFERCOMPL | HCINT_CHHLTD | HCINT_ACK);
    g_assert_false(irq_source(qts, SRC_USB_OTG));
    g_assert_cmphex(rd(qts, OTG_HS, HCTSIZ(0)), ==, TSIZ(0, 0, PID_DATA1));
    g_assert_cmphex(rd(qts, OTG_HS, HCDMA(0)), ==, BUF_SETUP + 8);

    /* IN data: 18 bytes of 64 asked, a short packet ends it. */
    wr(qts, OTG_HS, HCTSIZ(0), TSIZ(64, 1, PID_DATA1));
    wr(qts, OTG_HS, HCDMA(0), BUF_DATA);
    hcint = run_chan(qts, OTG_HS, hcchar | HCCHAR_EPDIR_IN);
    g_assert_cmphex(hcint, ==, HCINT_XFERCOMPL | HCINT_CHHLTD | HCINT_ACK);
    g_assert_cmphex(rd(qts, OTG_HS, HCTSIZ(0)), ==,
                    TSIZ(64 - 18, 0, PID_DATA0));
    qtest_memread(qts, BUF_DATA, desc, sizeof(desc));
    g_assert_cmphex(desc[0], ==, 18);
    g_assert_cmphex(desc[1], ==, 1);
    g_assert_cmphex(desc[8] | (desc[9] << 8), ==, 0x0627);
    g_assert_cmphex(desc[10] | (desc[11] << 8), ==, 0x0001);

    /* Status: a zero-length OUT. */
    wr(qts, OTG_HS, HCTSIZ(0), TSIZ(0, 1, PID_DATA1));
    hcint = run_chan(qts, OTG_HS, hcchar);
    g_assert_cmphex(hcint, ==, HCINT_XFERCOMPL | HCINT_CHHLTD | HCINT_ACK);

    /* Suspended, the port sends no SOFs; resume signalling ends it. */
    wr(qts, OTG_HS, HPRT, HPRT_PWR | HPRT_SUSP);
    frnum = rd(qts, OTG_HS, HFNUM) & 0x3fff;
    qtest_clock_step(qts, 8 * HS_FRAME_NS);
    g_assert_cmpint(rd(qts, OTG_HS, HFNUM) & 0x3fff, ==, frnum);
    wr(qts, OTG_HS, HPRT, HPRT_PWR | HPRT_RES);
    qtest_clock_step(qts, 20 * 1000 * 1000);
    g_assert_cmphex(rd(qts, OTG_HS, HPRT) & (HPRT_SUSP | HPRT_RES), ==,
                    HPRT_SUSP | HPRT_RES);
    wr(qts, OTG_HS, HPRT, HPRT_PWR);
    g_assert_cmphex(rd(qts, OTG_HS, HPRT) & (HPRT_SUSP | HPRT_ENA), ==,
                    HPRT_ENA);
    /* An SOF as the port runs again, then one each microframe. */
    qtest_clock_step(qts, 8 * HS_FRAME_NS);
    g_assert_cmpint((rd(qts, OTG_HS, HFNUM) - frnum) & 0x3fff, ==, 9);

    /*
     * The DMA APM refuses the core's writes once HP_DMA_PMS gives OTG_HS
     * no region: the IN data never lands, the channel halts with AHBErr,
     * and the AXI ICM records the AHB master's ID and the address.
     */
    qtest_writel(qts, DMA_PMS_USB_W, 0);
    qtest_memset(qts, BUF_DATA, 0x5a, 18);
    qtest_memwrite(qts, BUF_SETUP, get_device_desc, 8);
    wr(qts, OTG_HS, HCTSIZ(0), TSIZ(8, 1, PID_SETUP));
    wr(qts, OTG_HS, HCDMA(0), BUF_SETUP);
    hcint = run_chan(qts, OTG_HS, hcchar);
    g_assert_cmphex(hcint, ==, HCINT_XFERCOMPL | HCINT_CHHLTD | HCINT_ACK);
    wr(qts, OTG_HS, HCTSIZ(0), TSIZ(64, 1, PID_DATA1));
    wr(qts, OTG_HS, HCDMA(0), BUF_DATA);
    hcint = run_chan(qts, OTG_HS, hcchar | HCCHAR_EPDIR_IN);
    g_assert_cmphex(hcint & (HCINT_AHBERR | HCINT_CHHLTD), ==,
                    HCINT_AHBERR | HCINT_CHHLTD);
    g_assert_cmphex(qtest_readb(qts, BUF_DATA), ==, 0x5a);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW) & ICM_INT_CPU, ==,
                    ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, BUF_DATA);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==,
                    ICM_WR | ICM_ID_USB_OTG_HS);

    /* Unplugging the device disables the port. */
    qtest_qmp_device_del(qts, "kbd");
    hprt = rd(qts, OTG_HS, HPRT);
    g_assert_cmphex(hprt & (HPRT_CONNSTS | HPRT_CONNDET | HPRT_ENA |
                            HPRT_ENACHG), ==, HPRT_ENACHG);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_DISCONNINT, ==,
                    GINTSTS_DISCONNINT);
    qtest_quit(qts);
}

/*
 * OTG_FS as a host in scatter/gather mode with a full-speed usb-kbd:
 * each stage of a control transfer is one activation of channel 0 on a
 * one-descriptor list, which the core writes back.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_fs_host_scatter_gather(void)
{
    QTestState *qts = qtest_init("-M esp32p4 -device "
                                 "usb-kbd,bus=usb-otg-fs.0,port=1,"
                                 "usb_version=1");
    uint32_t hprt, hcint, qtd, hcchar;
    uint8_t desc[18];

    fs_clock_on(qts);
    /* The full-speed PHY joins the cable once its pads are enabled. */
    qtest_writel(qts, USB_WRAP, 1u << 18);
    hprt = port_up(qts, OTG_FS);
    g_assert_cmpint(HPRT_SPD(hprt), ==, 1);
    wr(qts, OTG_FS, GAHBCFG, GAHBCFG_DMA_EN);
    wr(qts, OTG_FS, HCFG, HCFG_DESCDMA | HCFG_FSLSPCLKSEL_48);
    wr(qts, OTG_FS, HFIR, 48000);

    /* SETUP: one qTD with SUP, IOC and EOL. */
    qtest_memwrite(qts, BUF_SETUP, get_device_desc, 8);
    qtest_writel(qts, QTD_LIST, QTD_ACTIVE | QTD_SUP | QTD_IOC | QTD_EOL | 8);
    qtest_writel(qts, QTD_LIST + 4, BUF_SETUP);
    hcchar = HCCHAR_MPS(8) | HCCHAR_DEVADDR(0);
    wr(qts, OTG_FS, HCTSIZ(0), PID_DATA0 << 29);
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    hcint = run_chan(qts, OTG_FS, hcchar);
    g_assert_cmphex(hcint & ~0x2000, ==, HCINT_XFERCOMPL | HCINT_CHHLTD);
    qtd = qtest_readl(qts, QTD_LIST);
    g_assert_cmphex(qtd & QTD_ACTIVE, ==, 0);
    g_assert_cmpint(QTD_STS(qtd), ==, 0);
    g_assert_cmpint(QTD_NBYTES(qtd), ==, 0);

    /* IN: 24 bytes asked, 18 received in three packets of up to 8. */
    qtest_writel(qts, QTD_LIST, QTD_ACTIVE | QTD_IOC | QTD_EOL | 24);
    qtest_writel(qts, QTD_LIST + 4, BUF_DATA);
    wr(qts, OTG_FS, HCTSIZ(0), PID_DATA1 << 29);
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    hcint = run_chan(qts, OTG_FS, hcchar | HCCHAR_EPDIR_IN);
    g_assert_cmphex(hcint & ~0x2000, ==, HCINT_XFERCOMPL | HCINT_CHHLTD);
    qtd = qtest_readl(qts, QTD_LIST);
    g_assert_cmphex(qtd & QTD_ACTIVE, ==, 0);
    g_assert_cmpint(QTD_NBYTES(qtd), ==, 24 - 18);
    qtest_memread(qts, BUF_DATA, desc, sizeof(desc));
    g_assert_cmphex(desc[0], ==, 18);
    g_assert_cmphex(desc[7], ==, 8);
    g_assert_cmphex(desc[8] | (desc[9] << 8), ==, 0x0627);

    /* An inactive descriptor: BNA, and no ChHltd. */
    qtest_writel(qts, QTD_LIST, 0);
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    hcint = run_chan(qts, OTG_FS, hcchar);
    g_assert_cmphex(hcint, ==, HCINT_BNA);
    g_assert_cmphex(rd(qts, OTG_FS, HCCHAR(0)) & HCCHAR_CHENA, ==, 0);
    qtest_quit(qts);
}

/*
 * OTG_FS's frames at a PHY clock of no whole number of Hz: PLL_F48M_CLK
 * from SPLL_CLK's 480 MHz by 11, 43 7/11 MHz. Unprogrammed, HFIR's frame
 * interval is the PHY clocks in 1 ms, 43636, and each frame lasts
 * 43636 * 11/480 us, 999991 2/3 ns. The frames count the PHY clock's
 * edges, at multiples of 275/12 ns (SPLL_CLK runs from XTAL_CLK's edge at
 * time 0, and the divider, set then, from its edge there), from the last
 * at or before the port starts running: frame 1000 begins 1000 frames
 * after that edge, at the first ns at or after. Counted at the clock's
 * frequency in whole Hz, from the start, the frames would run late.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_fs_host_frames_exact(void)
{
    /* Strapped for download boot, where no watchdog resets the chip. */
    QTestState *qts = qtest_init("-M esp32p4,strap-mode=0x04 -device "
                                 "usb-kbd,bus=usb-otg-fs.0,port=1,"
                                 "usb_version=1");
    uint32_t ctrl0 = qtest_readl(qts, LP_CLKRST_HP_USB_CLKRST_CTRL0);
    int64_t t0, t, end;

    fs_clock_on(qts);
    qtest_writel(qts, LP_CLKRST_HP_USB_CLKRST_CTRL0,
                 (ctrl0 & ~PLL_F48M_DIV_MASK) | PLL_F48M_DIV(11));
    qtest_writel(qts, USB_WRAP, 1u << 18);
    port_up(qts, OTG_FS);
    /* The port runs, and frame 0 begins, from port_up()'s last writes. */
    t0 = qtest_clock_step(qts, 1) - 1;
    g_assert_cmpuint(rd(qts, OTG_FS, HFIR) & 0xffff, ==, 43636);
    g_assert_cmpuint(rd(qts, OTG_FS, HFNUM) & 0x3fff, ==, 0);
    end = ((t0 * 12 / 275 + 43636 * 1000ll) * 275 + 11) / 12;
    t = qtest_clock_step(qts, end - 1 - (t0 + 1));
    g_assert_cmpint(t, ==, end - 1);
    g_assert_cmpuint(rd(qts, OTG_FS, HFNUM) & 0x3fff, ==, 999);
    qtest_clock_step(qts, 1);
    g_assert_cmpuint(rd(qts, OTG_FS, HFNUM) & 0x3fff, ==, 1000);
    qtest_quit(qts);
}

#define HFLBADDR                0x41c
#define HCFG_PERSCHEDENA        (1u << 26)
#define HCCHAR_EPNUM(n)         ((n) << 11)
#define HCCHAR_MC(n)            ((n) << 20)
#define HCCHAR_EPTYPE_ISOC      (1u << 18)
#define TSIZ_NTD(n)             ((n) << 8)
#define FRAME_LIST              0x4ff40600
#define ISO_PACKET              192

/*
 * A control transfer without a data stage on OTG_FS's channel 0 in
 * scatter/gather mode: the SETUP, then a zero-length IN status stage.
 */
static void sg_control_nodata(QTestState *qts, const uint8_t *setup)
{
    uint32_t hcchar = HCCHAR_MPS(8) | HCCHAR_DEVADDR(0);
    uint32_t hcint;

    qtest_memwrite(qts, BUF_SETUP, setup, 8);
    qtest_writel(qts, QTD_LIST, QTD_ACTIVE | QTD_SUP | QTD_IOC | QTD_EOL | 8);
    qtest_writel(qts, QTD_LIST + 4, BUF_SETUP);
    wr(qts, OTG_FS, HCTSIZ(0), PID_DATA0 << 29);
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    hcint = run_chan(qts, OTG_FS, hcchar);
    g_assert_cmphex(hcint & ~0x2000, ==, HCINT_XFERCOMPL | HCINT_CHHLTD);

    qtest_writel(qts, QTD_LIST, QTD_ACTIVE | QTD_IOC | QTD_EOL);
    qtest_writel(qts, QTD_LIST + 4, BUF_DATA);
    wr(qts, OTG_FS, HCTSIZ(0), PID_DATA1 << 29);
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    hcint = run_chan(qts, OTG_FS, hcchar | HCCHAR_EPDIR_IN);
    g_assert_cmphex(hcint & ~0x2000, ==, HCINT_XFERCOMPL | HCINT_CHHLTD);
    g_assert_cmpint(QTD_STS(qtest_readl(qts, QTD_LIST)), ==, 0);
}

/*
 * OTG_FS as a host in scatter/gather mode streaming to a usb-audio's
 * isochronous OUT endpoint: the periodic frame list schedules channel 0 in
 * every frame, the descriptor list is indexed by the frame number, each
 * active descriptor sends one packet and is written back, inactive ones
 * skip their frame, and the channel runs until it is disabled.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_fs_host_iso_scatter_gather(void)
{
    static const uint8_t set_config[8] = { 0x00, 0x09, 1, 0, 0, 0, 0, 0 };
    static const uint8_t set_alt[8] = { 0x01, 0x0b, 1, 0, 1, 0, 0, 0 };
    QTestState *qts = qtest_init("-M esp32p4 -audiodev none,id=snd "
                                 "-device usb-audio,audiodev=snd,"
                                 "bus=usb-otg-fs.0,port=1");
    uint8_t pcm[ISO_PACKET];
    uint32_t hcint, frame, qtd;
    int i;

    fs_clock_on(qts);
    qtest_writel(qts, USB_WRAP, 1u << 18);
    g_assert_cmpint(HPRT_SPD(port_up(qts, OTG_FS)), ==, 1);
    wr(qts, OTG_FS, GAHBCFG, GAHBCFG_DMA_EN);
    wr(qts, OTG_FS, HCFG, HCFG_DESCDMA | HCFG_FSLSPCLKSEL_48);
    wr(qts, OTG_FS, HFIR, 48000);
    sg_control_nodata(qts, set_config);
    sg_control_nodata(qts, set_alt);

    /* Eight frame-list entries, all naming channel 0. */
    for (i = 0; i < 8; i++) {
        qtest_writel(qts, FRAME_LIST + 4 * i, 1u << 0);
    }
    wr(qts, OTG_FS, HFLBADDR, FRAME_LIST);
    wr(qts, OTG_FS, HCFG, HCFG_DESCDMA | HCFG_FSLSPCLKSEL_48 |
       HCFG_PERSCHEDENA);

    /* Eight descriptors, the even ones active with a packet each. */
    memset(pcm, 0x5a, sizeof(pcm));
    qtest_memwrite(qts, BUF_DATA, pcm, sizeof(pcm));
    for (i = 0; i < 8; i++) {
        qtest_writel(qts, QTD_LIST + 8 * i,
                     i & 1 ? 0 : QTD_ACTIVE | QTD_IOC | ISO_PACKET);
        qtest_writel(qts, QTD_LIST + 8 * i + 4, BUF_DATA);
    }
    wr(qts, OTG_FS, HCINT(0), 0x3fff);
    wr(qts, OTG_FS, HCTSIZ(0), TSIZ_NTD(7));
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    wr(qts, OTG_FS, HCCHAR(0), HCCHAR_CHENA | HCCHAR_EPTYPE_ISOC |
       HCCHAR_MC(1) | HCCHAR_EPNUM(1) | HCCHAR_MPS(ISO_PACKET));
    for (i = 0; i < 10; i++) {
        qtest_clock_step(qts, 1000 * 1000);
    }
    frame = rd(qts, OTG_FS, HFNUM) & 0xffff;
    g_assert_cmphex(rd(qts, OTG_FS, HCCHAR(0)) & HCCHAR_CHENA, ==,
                    HCCHAR_CHENA);
    hcint = rd(qts, OTG_FS, HCINT(0));
    g_assert_cmphex(hcint & ~0x2000, ==, HCINT_XFERCOMPL);
    for (i = 0; i < 8; i++) {
        qtd = qtest_readl(qts, QTD_LIST + 8 * i);
        if (i & 1) {
            g_assert_cmphex(qtd, ==, 0);
        } else {
            g_assert_cmphex(qtd & QTD_ACTIVE, ==, 0);
            g_assert_cmpint(QTD_STS(qtd), ==, 0);
            g_assert_cmphex(qtd & QTD_IOC, ==, QTD_IOC);
        }
    }
    /* The current descriptor follows the frame number. */
    g_assert_cmpint((rd(qts, OTG_FS, HCDMA(0)) >> 3) & 0x3f, <=, 7);
    g_assert_cmpint(frame, >, 8);

    /* Disabling the channel halts it. */
    wr(qts, OTG_FS, HCINT(0), hcint);
    wr(qts, OTG_FS, HCCHAR(0), rd(qts, OTG_FS, HCCHAR(0)) | HCCHAR_CHDIS);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, OTG_FS, HCCHAR(0)) & HCCHAR_CHENA, ==, 0);
    g_assert_cmphex(rd(qts, OTG_FS, HCINT(0)), ==, HCINT_CHHLTD);

    /*
     * As ESP-IDF schedules a transfer: packets in the frames after the
     * next, the last with EOL, which halts the channel after it.
     */
    frame = rd(qts, OTG_FS, HFNUM) & 0xffff;
    for (i = 0; i < 8; i++) {
        qtest_writel(qts, QTD_LIST + 8 * i, 0);
    }
    for (i = 2; i < 5; i++) {
        qtest_writel(qts, QTD_LIST + 8 * ((frame + i) % 8),
                     QTD_ACTIVE | (i == 4 ? QTD_IOC | QTD_EOL : 0) |
                     ISO_PACKET);
    }
    wr(qts, OTG_FS, HCINT(0), 0x3fff);
    wr(qts, OTG_FS, HCDMA(0), QTD_LIST);
    wr(qts, OTG_FS, HCCHAR(0), HCCHAR_CHENA | HCCHAR_EPTYPE_ISOC |
       HCCHAR_MC(1) | HCCHAR_EPNUM(1) | HCCHAR_MPS(ISO_PACKET));
    for (i = 0; i < 6; i++) {
        qtest_clock_step(qts, 1000 * 1000);
    }
    g_assert_cmphex(rd(qts, OTG_FS, HCCHAR(0)) & HCCHAR_CHENA, ==, 0);
    g_assert_cmphex(rd(qts, OTG_FS, HCINT(0)) & ~0x2000, ==,
                    HCINT_XFERCOMPL | HCINT_CHHLTD);
    for (i = 2; i < 5; i++) {
        qtd = qtest_readl(qts, QTD_LIST + 8 * ((frame + i) % 8));
        g_assert_cmphex(qtd & QTD_ACTIVE, ==, 0);
        g_assert_cmpint(QTD_STS(qtd), ==, 0);
    }
    qtest_quit(qts);
}

/* The token port: see hw/usb/usb-devport.h for the protocol. */

static int tok_connect(const char *path)
{
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    int fd, i;

    g_assert_cmpuint(strlen(path), <, sizeof(sa.sun_path));
    strcpy(sa.sun_path, path);
    for (i = 0; i < 100; i++) {
        fd = socket(AF_UNIX, SOCK_STREAM, 0);
        g_assert_cmpint(fd, >=, 0);
        if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
            return fd;
        }
        close(fd);
        g_usleep(10000);
    }
    g_assert_not_reached();
}

static void tok_recv(int fd, uint8_t *buf, size_t len)
{
    while (len) {
        ssize_t n = recv(fd, buf, len, 0);

        g_assert_cmpint(n, >, 0);
        buf += n;
        len -= n;
    }
}

/*
 * One transaction: returns the handshake; for an acknowledged IN, the
 * data in @buf, its length in *@len and its PID in *@pid.
 */
static char tok_xact(int fd, char op, uint8_t addr, uint8_t ep, uint8_t pid,
                     uint16_t len, const uint8_t *data, uint8_t *buf,
                     uint16_t *got, uint8_t *got_pid)
{
    uint8_t hdr[8] = { op, addr, ep, pid, len, len >> 8, 0, 0 };
    uint8_t reply[4];
    uint16_t n;

    g_assert_cmpint(send(fd, hdr, 8, 0), ==, 8);
    if ((op == 'S' || op == 'O') && len) {
        g_assert_cmpint(send(fd, data, len, 0), ==, len);
    }
    tok_recv(fd, reply, 4);
    n = reply[2] | (reply[3] << 8);
    if (got_pid) {
        *got_pid = reply[1];
    }
    if (got) {
        *got = n;
    }
    if (n) {
        tok_recv(fd, buf, n);
    }
    return reply[0];
}

static uint8_t tok_port(int fd)
{
    uint8_t flags;

    g_assert_cmpint(tok_xact(fd, 'P', 0, 0, 0, 0, NULL, NULL, NULL, &flags),
                    ==, 'A');
    return flags;
}

/*
 * Which full-speed PHY OTG_FS has: by default the one on GPIO26/27, USB
 * Serial/JTAG the other; LP_SYS_USB_CTRL's SW_HW_USB_PHY_SEL hands the
 * choice from the eFuse to SW_USB_PHY_SEL.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_fs_phy_select(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_irq_intercept_out_named(qts, "/machine/usb-wrap", "otg-phy");
    qtest_writel(qts, LP_SYS_USB_CTRL, 3);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_USB_CTRL), ==, 3);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, LP_SYS_USB_CTRL, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, LP_SYS_USB_CTRL, 2);
    g_assert_true(qtest_get_irq(qts, 0));
    /* USBOTG20_WAKEUP_CLR reads as 0. */
    qtest_writel(qts, LP_SYS_USB_CTRL, 0x1f);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_USB_CTRL), ==, 0x1b);
    qtest_quit(qts);
}

/*
 * OTG_FS in device mode: a host sees it only once USB_WRAP enables the
 * pads, and the pads' pull override decides over the core's pull-up.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_fs_device_pads(void)
{
    g_autofree char *dir = g_dir_make_tmp("usbXXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/tok", dir);
    g_autofree char *args = g_strdup_printf(
        "-M esp32p4 -chardev socket,id=tok,path=%s,server=on,wait=off "
        "-global esp32p4-usb-otg-fs.token-chardev=tok", path);
    QTestState *qts = qtest_init(args);
    int fd = tok_connect(path);

    /* The PHY's D+ pull-up shows on its pad, GPIO27. */
    qtest_irq_intercept_out_named(qts, "/machine/gpio", "pad");
    fs_clock_on(qts);
    /* A B-device with a session, as ESP-IDF routes the OTG signals. */
    qtest_writel(qts, GPIO_FUNC_IN(110), GPIO_IN_SEL | GPIO_CONST_ONE);
    qtest_writel(qts, GPIO_FUNC_IN(112), GPIO_IN_SEL | GPIO_CONST_ONE);
    qtest_writel(qts, GPIO_FUNC_IN(113), GPIO_IN_SEL | GPIO_CONST_ONE);
    g_assert_cmphex(rd(qts, OTG_FS, GINTSTS) & GINTSTS_CURMOD_HOST, ==, 0);
    g_assert_cmphex(rd(qts, OTG_FS, GOTGCTL) &
                    (GOTGCTL_CONID_B | GOTGCTL_BSESVLD), ==,
                    GOTGCTL_CONID_B | GOTGCTL_BSESVLD);
    wr(qts, OTG_FS, DCTL, 0);
    g_assert_cmpint(tok_port(fd), ==, 0);
    g_assert_false(qtest_get_irq(qts, 27));
    qtest_writel(qts, USB_WRAP, OTG_CONF_USB_PAD_ENABLE);
    g_assert_cmpint(tok_port(fd), ==, 1);
    g_assert_true(qtest_get_irq(qts, 27));
    /* TinyUSB's dcd_disconnect(): the override pulls both lines down. */
    qtest_writel(qts, USB_WRAP, OTG_CONF_USB_PAD_ENABLE |
                 OTG_CONF_PAD_PULL_OVERRIDE | OTG_CONF_DP_PULLDOWN |
                 OTG_CONF_DM_PULLDOWN);
    g_assert_cmpint(tok_port(fd), ==, 0);
    g_assert_false(qtest_get_irq(qts, 27));
    wr(qts, OTG_FS, DCTL, DCTL_SFTDISCON);
    qtest_writel(qts, USB_WRAP, OTG_CONF_USB_PAD_ENABLE |
                 OTG_CONF_PAD_PULL_OVERRIDE | OTG_CONF_DP_PULLUP);
    g_assert_cmpint(tok_port(fd), ==, 1);
    close(fd);
    qtest_quit(qts);
    unlink(path);
    rmdir(dir);
}

/*
 * OTG_HS in device mode, slave mode, driven by a host on its token port:
 * the pull-up, the bus reset and enumeration, a SETUP into the receive
 * FIFO and the setup phase's end, an IN data stage from the transmit
 * FIFO with DATA1, a status stage, SET_ADDRESS taking effect after its
 * status stage, STALL, and suspend and resume.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_hs_device_token(void)
{
    g_autofree char *dir = g_dir_make_tmp("usbXXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/tok", dir);
    g_autofree char *args = g_strdup_printf(
        "-M esp32p4 -chardev socket,id=tok,path=%s,server=on,wait=off "
        "-global esp32p4-usb-otg-hs.token-chardev=tok", path);
    static const uint8_t setup_get[8] = { 0x80, 6, 0, 1, 0, 0, 18, 0 };
    static const uint8_t setup_addr[8] = { 0x00, 5, 5, 0, 0, 0, 0, 0 };
    static const uint8_t desc[20] = { 18, 1, 0, 2, 0, 0, 0, 64, 0xfe, 0xca,
                                      1, 0, 0, 1, 0, 0, 0, 1 };
    QTestState *qts = qtest_init(args);
    uint8_t buf[64], pid;
    uint32_t st, w0, w1;
    uint16_t n;
    int fd = tok_connect(path), i;

    /* The host on the cable supplies VBUS: a B-session. */
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(rd(qts, OTG_HS, GOTGCTL) & GOTGCTL_BSESVLD, ==,
                    GOTGCTL_BSESVLD);
    g_assert_cmpint(tok_port(fd), ==, 0);
    wr(qts, OTG_HS, DCTL, 0);
    g_assert_cmpint(tok_port(fd), ==, 1);

    g_assert_cmpint(tok_xact(fd, 'R', 0, 0, 0, 0, NULL, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) &
                    (GINTSTS_USBRST | GINTSTS_ENUMDONE), ==,
                    GINTSTS_USBRST | GINTSTS_ENUMDONE);
    wr(qts, OTG_HS, GINTSTS, GINTSTS_USBRST | GINTSTS_ENUMDONE);
    /* The host is a full-speed one. */
    g_assert_cmpint(DSTS_ENUMSPD(rd(qts, OTG_HS, DSTS)), ==, 1);
    wr(qts, OTG_HS, GRXFSIZ, 64);
    wr(qts, OTG_HS, GNPTXFSIZ, (32u << 16) | 64);
    wr(qts, OTG_HS, DOEPTSIZ(0), 3u << 29);

    /* A SETUP: always taken, into the receive FIFO with its status. */
    g_assert_cmpint(tok_xact(fd, 'S', 0, 0, 0, 8, setup_get, NULL, NULL,
                             NULL), ==, 'A');
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_RXFLVL, ==,
                    GINTSTS_RXFLVL);
    st = rd(qts, OTG_HS, GRXSTSP);
    g_assert_cmpint(GRXSTS_PKTSTS(st), ==, 6);
    g_assert_cmpint(GRXSTS_BCNT(st), ==, 8);
    w0 = rd(qts, OTG_HS, FIFO(0));
    w1 = rd(qts, OTG_HS, FIFO(0));
    g_assert_cmphex(w0, ==, 0x01000680);
    g_assert_cmphex(w1, ==, 0x00120000);
    g_assert_cmpint(DOEPTSIZ_SUPCNT(rd(qts, OTG_HS, DOEPTSIZ(0))), ==, 2);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_RXFLVL, ==, 0);

    /*
     * The data stage's first IN ends the setup phase, NAKed until the
     * application has the data in the FIFO.
     */
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'N');
    st = rd(qts, OTG_HS, GRXSTSP);
    g_assert_cmpint(GRXSTS_PKTSTS(st), ==, 4);
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_SETUP, ==,
                    DEPINT_SETUP);
    wr(qts, OTG_HS, DOEPINT(0), DEPINT_SETUP);
    wr(qts, OTG_HS, DIEPTSIZ(0), 18 | (1u << 19));
    wr(qts, OTG_HS, DIEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    for (i = 0; i < 18; i += 4) {
        wr(qts, OTG_HS, FIFO(0), desc[i] | (desc[i + 1] << 8) |
                                 (desc[i + 2] << 16) |
                                 ((uint32_t)desc[i + 3] << 24));
    }
    g_assert_cmpint(rd(qts, OTG_HS, DTXFSTS(0)), ==, 32 - 5);
    g_assert_cmphex(rd(qts, OTG_HS, DIEPTSIZ(0)), ==, 1u << 19);
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, &n, &pid),
                    ==, 'A');
    g_assert_cmpint(n, ==, 18);
    g_assert_cmpint(pid, ==, 1);
    g_assert_cmpmem(buf, 18, desc, 18);
    g_assert_cmphex(rd(qts, OTG_HS, DIEPINT(0)) & DEPINT_XFERCOMPL, ==,
                    DEPINT_XFERCOMPL);
    wr(qts, OTG_HS, DIEPINT(0), DEPINT_XFERCOMPL);
    g_assert_cmphex(rd(qts, OTG_HS, DIEPCTL(0)) & DEPCTL_EPENA, ==, 0);

    /* The status stage: NAKed, and flagged, until EP0 OUT is enabled. */
    g_assert_cmpint(tok_xact(fd, 'O', 0, 0, 1, 0, NULL, NULL, NULL, NULL),
                    ==, 'N');
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_OUTTKNEPDIS, ==,
                    DEPINT_OUTTKNEPDIS);
    wr(qts, OTG_HS, DOEPINT(0), 0xffff);
    wr(qts, OTG_HS, DOEPTSIZ(0), (3u << 29) | (1u << 19));
    wr(qts, OTG_HS, DOEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'O', 0, 0, 1, 0, NULL, NULL, NULL, NULL),
                    ==, 'A');
    st = rd(qts, OTG_HS, GRXSTSP);
    g_assert_cmpint(GRXSTS_PKTSTS(st), ==, 2);
    g_assert_cmpint(GRXSTS_BCNT(st), ==, 0);
    st = rd(qts, OTG_HS, GRXSTSP);
    g_assert_cmpint(GRXSTS_PKTSTS(st), ==, 3);
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_XFERCOMPL, ==,
                    DEPINT_XFERCOMPL);
    g_assert_cmphex(rd(qts, OTG_HS, DOEPCTL(0)) &
                    (DEPCTL_EPENA | DEPCTL_NAKSTS), ==, DEPCTL_NAKSTS);
    wr(qts, OTG_HS, DOEPINT(0), 0xffff);

    /*
     * SET_ADDRESS: software writes DCFG.DevAddr before the status stage,
     * which the host still sends to address 0.
     */
    g_assert_cmpint(tok_xact(fd, 'S', 0, 0, 0, 8, setup_addr, NULL, NULL,
                             NULL), ==, 'A');
    rd(qts, OTG_HS, GRXSTSP);
    rd(qts, OTG_HS, FIFO(0));
    rd(qts, OTG_HS, FIFO(0));
    wr(qts, OTG_HS, DCFG, rd(qts, OTG_HS, DCFG) | (5u << 4));
    wr(qts, OTG_HS, DIEPTSIZ(0), 1u << 19);
    wr(qts, OTG_HS, DIEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, &n, &pid),
                    ==, 'A');
    g_assert_cmpint(n, ==, 0);
    g_assert_cmpint(pid, ==, 1);
    rd(qts, OTG_HS, GRXSTSP);
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'T');
    g_assert_cmpint(tok_xact(fd, 'I', 5, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'N');

    /* A stalled EP0 until the next SETUP clears it. */
    wr(qts, OTG_HS, DIEPCTL(0), DEPCTL_STALL);
    g_assert_cmpint(tok_xact(fd, 'I', 5, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'S');
    g_assert_cmpint(tok_xact(fd, 'S', 5, 0, 0, 8, setup_get, NULL, NULL,
                             NULL), ==, 'A');
    g_assert_cmphex(rd(qts, OTG_HS, DIEPCTL(0)) & DEPCTL_STALL, ==, 0);

    /* 3 ms of an idle bus suspends the device; an SOF resumes it. */
    qtest_clock_step(qts, 4 * 1000 * 1000);
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) &
                    (GINTSTS_ERLYSUSP | GINTSTS_USBSUSP), ==,
                    GINTSTS_ERLYSUSP | GINTSTS_USBSUSP);
    g_assert_cmphex(rd(qts, OTG_HS, DSTS) & DSTS_SUSPSTS, ==, DSTS_SUSPSTS);
    g_assert_cmpint(tok_port(fd), ==, 3);
    g_assert_cmpint(tok_xact(fd, 'F', 0, 0, 0, 0x123, NULL, NULL, NULL,
                             NULL), ==, 'A');
    g_assert_cmphex(rd(qts, OTG_HS, GINTSTS) & GINTSTS_WKUPINT, ==,
                    GINTSTS_WKUPINT);
    g_assert_cmphex(rd(qts, OTG_HS, DSTS) & DSTS_SUSPSTS, ==, 0);
    g_assert_cmpint(DSTS_SOFFN(rd(qts, OTG_HS, DSTS)), ==, 0x123);
    g_assert_cmpint(tok_port(fd), ==, 1);

    /* A soft disconnect takes the pull-up away. */
    wr(qts, OTG_HS, DCTL, DCTL_SFTDISCON);
    g_assert_cmpint(tok_port(fd), ==, 0);

    close(fd);
    qtest_quit(qts);
    unlink(path);
    rmdir(dir);
}

/* Device scatter/gather descriptor quadlet. */
#define DESC_BS(v)              (((v) >> 30) & 3)
#define DESC_BS_HOST_READY      0u
#define DESC_BS_HOST_BUSY       (3u << 30)
#define DESC_BS_DMA_DONE        2
#define DESC_L                  (1u << 27)
#define DESC_SP                 (1u << 26)
#define DESC_IOC                (1u << 25)
#define DESC_SR                 (1u << 24)
#define DESC_NBYTES(v)          ((v) & 0xffff)
#define DCFG_DESCDMA            (1u << 23)
#define DEPINT_BNA              (1u << 9)
#define DEPINT_STSPHSERCVD      (1u << 5)
#define DEPINT_STUPPKTRCVD      (1u << 15)
#define DIEPDMA(n)              (0x914 + 0x20 * (n))
#define DOEPDMA(n)              (0xb14 + 0x20 * (n))
#define DESC_OUT0               0x4ff40400
#define DESC_IN0                0x4ff40500

static void arm_desc(QTestState *qts, uint32_t desc, uint32_t status,
                     uint32_t buf)
{
    qtest_writel(qts, desc, status);
    qtest_writel(qts, desc + 4, buf);
}

/*
 * OTG_HS in device mode with scatter/gather DMA: SETUPs into control OUT
 * endpoint 0's descriptor (SR), IN data from a descriptor, short OUT
 * packets closing theirs (SP), BNA for a descriptor not ready, and a
 * control write's status stage (StsPhseRcvd).
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
static void test_hs_device_scatter_gather(void)
{
    g_autofree char *dir = g_dir_make_tmp("usbXXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/tok", dir);
    g_autofree char *args = g_strdup_printf(
        "-M esp32p4 -chardev socket,id=tok,path=%s,server=on,wait=off "
        "-global esp32p4-usb-otg-hs.token-chardev=tok", path);
    static const uint8_t setup_get[8] = { 0x80, 6, 0, 1, 0, 0, 18, 0 };
    static const uint8_t setup_write[8] = { 0x21, 0x20, 0, 0, 0, 0, 7, 0 };
    static const uint8_t coding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0, 8 };
    QTestState *qts = qtest_init(args);
    uint8_t buf[64], pid;
    uint32_t st;
    uint16_t n;
    int fd = tok_connect(path);

    qtest_clock_step(qts, 1000);
    wr(qts, OTG_HS, GAHBCFG, GAHBCFG_DMA_EN);
    wr(qts, OTG_HS, DCFG, rd(qts, OTG_HS, DCFG) | DCFG_DESCDMA);
    wr(qts, OTG_HS, DCTL, 0);
    g_assert_cmpint(tok_xact(fd, 'R', 0, 0, 0, 0, NULL, NULL, NULL, NULL),
                    ==, 'A');

    /* A SETUP lands in control OUT endpoint 0's descriptor. */
    arm_desc(qts, DESC_OUT0, DESC_BS_HOST_READY | DESC_L | DESC_IOC | 8,
             BUF_SETUP);
    wr(qts, OTG_HS, DOEPDMA(0), DESC_OUT0);
    wr(qts, OTG_HS, DOEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'S', 0, 0, 0, 8, setup_get, NULL, NULL,
                             NULL), ==, 'A');
    st = qtest_readl(qts, DESC_OUT0);
    g_assert_cmpint(DESC_BS(st), ==, DESC_BS_DMA_DONE);
    g_assert_cmphex(st & DESC_SR, ==, DESC_SR);
    g_assert_cmpint(DESC_NBYTES(st), ==, 0);
    qtest_memread(qts, BUF_SETUP, buf, 8);
    g_assert_cmpmem(buf, 8, setup_get, 8);
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_STUPPKTRCVD, ==,
                    DEPINT_STUPPKTRCVD);
    g_assert_cmphex(rd(qts, OTG_HS, DOEPCTL(0)) & DEPCTL_EPENA, ==, 0);

    /* The data stage: NAKed until a descriptor is ready, then its data. */
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'N');
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_SETUP, ==,
                    DEPINT_SETUP);
    wr(qts, OTG_HS, DOEPINT(0), 0xffff);
    qtest_memset(qts, BUF_DATA, 0xa5, 18);
    arm_desc(qts, DESC_IN0, DESC_BS_HOST_READY | DESC_L | DESC_IOC | 18,
             BUF_DATA);
    wr(qts, OTG_HS, DIEPDMA(0), DESC_IN0);
    wr(qts, OTG_HS, DIEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, &n, &pid),
                    ==, 'A');
    g_assert_cmpint(n, ==, 18);
    g_assert_cmpint(pid, ==, 1);
    g_assert_cmphex(buf[17], ==, 0xa5);
    g_assert_cmpint(DESC_BS(qtest_readl(qts, DESC_IN0)), ==,
                    DESC_BS_DMA_DONE);
    g_assert_cmphex(rd(qts, OTG_HS, DIEPINT(0)) & DEPINT_XFERCOMPL, ==,
                    DEPINT_XFERCOMPL);
    wr(qts, OTG_HS, DIEPINT(0), 0xffff);

    /* The status stage's zero-length OUT: a short packet, SP. */
    arm_desc(qts, DESC_OUT0, DESC_BS_HOST_READY | DESC_L | DESC_IOC | 8,
             BUF_SETUP);
    wr(qts, OTG_HS, DOEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'O', 0, 0, 1, 0, NULL, NULL, NULL, NULL),
                    ==, 'A');
    st = qtest_readl(qts, DESC_OUT0);
    g_assert_cmphex(st & (DESC_SP | DESC_SR), ==, DESC_SP);
    g_assert_cmpint(DESC_NBYTES(st), ==, 8);
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_XFERCOMPL, ==,
                    DEPINT_XFERCOMPL);
    wr(qts, OTG_HS, DOEPINT(0), 0xffff);

    /*
     * A control write: its data into a descriptor, then the status
     * stage's first IN raises StsPhseRcvd.
     */
    arm_desc(qts, DESC_OUT0, DESC_BS_HOST_READY | DESC_L | DESC_IOC | 8,
             BUF_SETUP);
    wr(qts, OTG_HS, DOEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'S', 0, 0, 0, 8, setup_write, NULL, NULL,
                             NULL), ==, 'A');
    arm_desc(qts, DESC_OUT0, DESC_BS_HOST_READY | DESC_L | DESC_IOC | 64,
             BUF_DATA);
    wr(qts, OTG_HS, DOEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'O', 0, 0, 1, 7, coding, NULL, NULL, NULL),
                    ==, 'A');
    st = qtest_readl(qts, DESC_OUT0);
    g_assert_cmpint(DESC_NBYTES(st), ==, 64 - 7);
    qtest_memread(qts, BUF_DATA, buf, 7);
    g_assert_cmpmem(buf, 7, coding, 7);
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'N');
    g_assert_cmphex(rd(qts, OTG_HS, DOEPINT(0)) & DEPINT_STSPHSERCVD, ==,
                    DEPINT_STSPHSERCVD);

    /* A descriptor the core may not use: BNA, the endpoint disabled. */
    arm_desc(qts, DESC_IN0, DESC_BS_HOST_BUSY | DESC_L | DESC_IOC, BUF_DATA);
    wr(qts, OTG_HS, DIEPCTL(0), DEPCTL_CNAK | DEPCTL_EPENA);
    g_assert_cmpint(tok_xact(fd, 'I', 0, 0, 0, 64, NULL, buf, NULL, NULL),
                    ==, 'N');
    g_assert_cmphex(rd(qts, OTG_HS, DIEPINT(0)) & DEPINT_BNA, ==,
                    DEPINT_BNA);
    g_assert_cmphex(rd(qts, OTG_HS, DIEPCTL(0)) & DEPCTL_EPENA, ==, 0);

    close(fd);
    qtest_quit(qts);
    unlink(path);
    rmdir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/usb/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/usb/register-access", test_register_access);
    qtest_add_func("/esp32p4/usb/clock-and-reset", test_clock_and_reset);
    qtest_add_func("/esp32p4/usb/soft-reset", test_soft_reset);
    qtest_add_func("/esp32p4/usb/force-mode", test_force_mode);
    qtest_add_func("/esp32p4/usb/hs-host-buffer-dma",
                   test_hs_host_buffer_dma);
    qtest_add_func("/esp32p4/usb/fs-host-frames-exact",
                   test_fs_host_frames_exact);
    qtest_add_func("/esp32p4/usb/fs-host-scatter-gather",
                   test_fs_host_scatter_gather);
    if (qtest_has_device("usb-audio")) {
        qtest_add_func("/esp32p4/usb/fs-host-iso-scatter-gather",
                       test_fs_host_iso_scatter_gather);
    }
    qtest_add_func("/esp32p4/usb/hs-device-token", test_hs_device_token);
    qtest_add_func("/esp32p4/usb/hs-device-scatter-gather",
                   test_hs_device_scatter_gather);
    qtest_add_func("/esp32p4/usb/fs-phy-select", test_fs_phy_select);
    qtest_add_func("/esp32p4/usb/fs-device-pads", test_fs_device_pads);
    return g_test_run();
}
