/*
 * QTest testcase for the ESP32-P4 HP and LP system registers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * HP_SYSTEM and LP_SYSTEM: their registers' reset values and access types,
 * and what they drive: the HP2LP interrupt groups (seen in their status
 * registers and on the "hp-intr" and "hp-wakeup" outputs), the pads'
 * holds (seen on the GPIO block's "pad" outputs), the memories'
 * initialisation, the FIB control of the brownout detector and the super
 * watchdog's reset, the LP
 * interrupts (in the interrupt matrix's status of source 19) and the
 * software system reset, an HP core reset with cause 0x03 that LP_SYSTEM
 * itself lives through.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define HP_SYS              0x500e5000
#define HP_SYS_PATH         "/machine/hp-sys"
#define HP_VER_DATE         (HP_SYS + 0x000)
#define HP_CLK_EN           (HP_SYS + 0x004)
#define HP_RESERVED         (HP_SYS + 0x008)
#define HP_CPU_INT_FROM_CPU(n) (HP_SYS + 0x010 + 4 * (n))
#define HP_CACHE_CLK_CONFIG (HP_SYS + 0x020)
#define HP_CACHE_RESET_CONFIG (HP_SYS + 0x024)
#define HP_TCM_RAM_WRR_CONFIG (HP_SYS + 0x034)
#define HP_CPU_CORESTALLED_ST (HP_SYS + 0x064)
#define HP_GPIO_O_HOLD_CTRL0 (HP_SYS + 0x074)
#define HP_GPIO_O_HOLD_CTRL1 (HP_SYS + 0x078)
#define HP_L2_MEM_RDN_ECO_HIGH (HP_SYS + 0x0e0)
#define HP_TCM_INT_RAW      (HP_SYS + 0x0fc)
#define HP_TCM_INT_ENA      (HP_SYS + 0x104)
#define HP_CPU_WAITI_CONF   (HP_SYS + 0x118)
#define HP_CORE_AHB_TIMEOUT (HP_SYS + 0x120)
#define HP_ICM_CPU_H2X_CFG  (HP_SYS + 0x138)
#define HP_BITSCRAMBLER_PERI_SEL (HP_SYS + 0x140)
#define HP_SYS_USBOTG20_CTRL (HP_SYS + 0x15c)
#define HP_L2_MEM_REFRESH   (HP_SYS + 0x164)
#define HP_TCM_INIT         (HP_SYS + 0x168)
#define HP_AHB2AXI_RAW      (HP_SYS + 0x188)
#define HP_AHB2AXI_ST       (HP_SYS + 0x18c)
#define HP_AHB2AXI_ENA      (HP_SYS + 0x190)
#define HP_CORE_ERR_RESP_DIS (HP_SYS + 0x1a4)
#define HP_CORE_TIMEOUT_INT_ENA (HP_SYS + 0x1b0)
#define HP_RSA_PD_CTRL      (HP_SYS + 0x1d0)
#define HP_RNG_CFG          (HP_SYS + 0x1d8)
#define HP_CPU_WAKEUP_EVENT (HP_SYS + 0x1ec)
#define HP_HP2LP_INTR_EN(n) (HP_SYS + 0x1f0 + 4 * (n))
#define HP_HP2LP_INTR_ST(n) (HP_SYS + 0x200 + 4 * (n))
#define HP_HP2LP_WAKEUP_EN(n) (HP_SYS + 0x210 + 4 * (n))

#define LP_SYS              0x50110000
#define LP_SYS_PATH         "/machine/lp-sys"
#define LP_VER_DATE         (LP_SYS + 0x000)
#define LP_SYS_CTRL         (LP_SYS + 0x008)
#define LP_CLK_CTRL         (LP_SYS + 0x00c)
#define LP_RST_CTRL         (LP_SYS + 0x010)
#define LP_CORE_BOOT_ADDR   (LP_SYS + 0x018)
#define LP_STORE(n)         (LP_SYS + 0x02c + 4 * (n))
#define LP_ANA_XPD_PAD_GROUP (LP_SYS + 0x10c)
#define LP_BACKUP_DMA_CFG0  (LP_SYS + 0x154)
#define LP_INT_RAW          (LP_SYS + 0x170)
#define LP_INT_ST           (LP_SYS + 0x174)
#define LP_INT_ENA          (LP_SYS + 0x178)
#define LP_INT_CLR          (LP_SYS + 0x17c)
#define LP_HP_MEM_AUX_CTRL  (LP_SYS + 0x180)
#define LP_CPU_DBG_PC       (LP_SYS + 0x190)
#define LP_CPU_EXC_PC       (LP_SYS + 0x194)
#define LP_HP_POR_RST_BYPASS_CTRL (LP_SYS + 0x1a0)
#define LP_RNG_DATA         (LP_SYS + 0x1a4)
#define LP_CORE_AHB_TIMEOUT (LP_SYS + 0x1b0)
#define LP_CORE_ERR_RESP_DIS (LP_SYS + 0x1bc)
#define LP_RNG_CFG          (LP_SYS + 0x1c0)
#define LP_PAD_RTC_HOLD_CTRL0 (LP_SYS + 0x1c4)
#define LP_DISCHARGE        (LP_SYS + 0x200)

#define LP_ANA_PATH         "/machine/lp-ana"

/* ANA_FIB reads 0x7B: the analog state enables the BOD, not the SWD. */
#define SYS_CTRL_RESET      0x1ffec7f8u
#define SYS_SW_RST          (1u << 1)
#define DIG_FIB(b)          (1u << (3 + (b)))
#define LP_FIB_SEL(b)       (1u << (21 + (b)))
#define FIB_BOD             1
#define FIB_SWD             2
#define ETM_WAKEUP_FLAG_CLR (1u << 29)
#define ETM_WAKEUP_FLAG     (1u << 30)
#define LP_INT_ETM_TASK_ULP (1u << 5)
#define LP_INT_SLOW_CLK_TICK (1u << 6)

#define LP_CLKRST           0x50111000
#define RESET_CAUSE         (LP_CLKRST + 0x10)
#define CAUSE_CLEAR_ALL     0xfc000000u
#define CORE0_CAUSE(v)      (((v) >> 7) & 0x3f)
#define CORE1_CAUSE(v)      (((v) >> 14) & 0x3f)
#define LP_CAUSE(v)         ((v) & 0x3f)
#define CAUSE_SW_SYS        0x03

#define INTMTX_STATUS(w)    (0x500d6200 + 4 * (w))
#define SRC_FROM_CPU0       79
#define SRC_LP_SYSREG       19

#define GPIO_PATH           "/machine/gpio"
#define GPIO                0x500e0000
#define GPIO_OUT_W1TS       (GPIO + 0x008)
#define GPIO_OUT_W1TC       (GPIO + 0x00c)
#define GPIO_OUT1_W1TS      (GPIO + 0x014)
#define GPIO_OUT1_W1TC      (GPIO + 0x018)
#define GPIO_ENABLE_W1TS    (GPIO + 0x024)
#define GPIO_ENABLE1_W1TS   (GPIO + 0x030)
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define SIG_GPIO_OUT        0x100
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define MCU_SEL_GPIO        (1u << 12)
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((d) << 10)

#define LP_ROM_BASE         0x50100000u

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

typedef struct {
    uint32_t addr;
    uint32_t reset;
} RegReset;

/* A sample of HP_SYSTEM's registers, at their reset values. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_hp_reset_values(void)
{
    static const RegReset regs[] = {
        { HP_VER_DATE, 0x20250307 },
        { HP_CLK_EN, 0 },
        { HP_CACHE_CLK_CONFIG, 0x33 },
        { HP_TCM_RAM_WRR_CONFIG, 0x826ed93f },
        { HP_L2_MEM_RDN_ECO_HIGH, 0xffffffff },
        { HP_CPU_WAITI_CONF, 0x1 },
        { HP_CORE_AHB_TIMEOUT, 0x1ffff },
        { HP_ICM_CPU_H2X_CFG, 0x3 },
        { HP_BITSCRAMBLER_PERI_SEL, 0xff },
        { HP_SYS_USBOTG20_CTRL, 0x822640 },
        { HP_L2_MEM_REFRESH, 0x40 },
        { HP_TCM_INIT, 0x2 },
        { HP_CORE_ERR_RESP_DIS, 0 },
        { HP_RSA_PD_CTRL, 0x2 },
        { HP_CPU_WAKEUP_EVENT, 0 },
        { HP_HP2LP_INTR_EN(0), 0xffffffff },
        { HP_HP2LP_INTR_EN(1), 0xffffffff },
        { HP_HP2LP_INTR_EN(2), 0xffffffff },
        { HP_HP2LP_INTR_EN(3), 0x3fff },
        { HP_HP2LP_INTR_ST(1), 0 },
        { HP_HP2LP_WAKEUP_EN(0), 0 },
        { HP_HP2LP_WAKEUP_EN(3), 0 },
    };
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(qtest_readl(qts, regs[i].addr), ==, regs[i].reset);
    }
    qtest_quit(qts);
}

/*
 * Writes change only the read-write bits; read-only registers keep what
 * the hardware drives; reserved offsets read 0.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_hp_access(void)
{
    QTestState *qts = start();

    qtest_writel(qts, HP_CLK_EN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_CLK_EN), ==, 1);
    qtest_writel(qts, HP_CACHE_RESET_CONFIG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_CACHE_RESET_CONFIG), ==, 0x32);
    qtest_writel(qts, HP_CACHE_RESET_CONFIG, 0);
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_GPIO_O_HOLD_CTRL1), ==, 0x1ff);
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL1, 0);
    qtest_writel(qts, HP_CORE_ERR_RESP_DIS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_CORE_ERR_RESP_DIS), ==, 0x7);
    qtest_writel(qts, HP_CORE_TIMEOUT_INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_CORE_TIMEOUT_INT_ENA), ==, 0x3f);
    qtest_writel(qts, HP_RNG_CFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_RNG_CFG), ==, 0x00ff0001);
    qtest_writel(qts, HP_HP2LP_WAKEUP_EN(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_WAKEUP_EN(3)), ==, 0x3fff);

    /* Byte writes change their byte alone. */
    qtest_writeb(qts, HP_BITSCRAMBLER_PERI_SEL, 0x5a);
    g_assert_cmphex(qtest_readl(qts, HP_BITSCRAMBLER_PERI_SEL), ==, 0x5a);
    qtest_writeb(qts, HP_TCM_RAM_WRR_CONFIG + 3, 0);
    g_assert_cmphex(qtest_readl(qts, HP_TCM_RAM_WRR_CONFIG), ==, 0x006ed93f);

    /* Read-only and reserved */
    qtest_writel(qts, HP_CPU_CORESTALLED_ST, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_CPU_CORESTALLED_ST), ==, 0);
    qtest_writel(qts, HP_HP2LP_INTR_ST(0), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_INTR_ST(0)), ==, 0);
    qtest_writel(qts, HP_RESERVED, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_RESERVED), ==, 0);
    qtest_quit(qts);
}

/*
 * The interrupt registers' layout: TCM's and the AHB-to-AXI bridge's
 * enables are bit 31; their raw bits only hardware sets.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_hp_interrupt_regs(void)
{
    QTestState *qts = start();

    qtest_writel(qts, HP_TCM_INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_TCM_INT_ENA), ==, 0x80000000);
    qtest_writel(qts, HP_AHB2AXI_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_AHB2AXI_ENA), ==, 0x80000000);
    qtest_writel(qts, HP_TCM_INT_RAW, 0xffffffff);
    qtest_writel(qts, HP_AHB2AXI_RAW, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_TCM_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, HP_AHB2AXI_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, HP_AHB2AXI_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(3)) & (1u << 14), ==, 0);
    qtest_quit(qts);
}

/*
 * L2MEM's refresh and the SPM's initialisation complete at once: a unit's
 * done bit sets as its enable does, with the count reset clear, and
 * clears when the count is reset.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_hp_memory_init(void)
{
    QTestState *qts = start();

    /* REFRESH_CNT_RESET (bit 6) is set from reset: nothing is done. */
    qtest_writel(qts, HP_L2_MEM_REFRESH, 0x40 | 0x05);
    g_assert_cmphex(qtest_readl(qts, HP_L2_MEM_REFRESH), ==, 0x45);
    qtest_writel(qts, HP_L2_MEM_REFRESH, 0x05);
    g_assert_cmphex(qtest_readl(qts, HP_L2_MEM_REFRESH), ==,
                    0x05 | (0x05 << 7));
    qtest_writel(qts, HP_L2_MEM_REFRESH, 0x3f);
    g_assert_cmphex(qtest_readl(qts, HP_L2_MEM_REFRESH), ==,
                    0x3f | (0x3f << 7));
    qtest_writel(qts, HP_L2_MEM_REFRESH, 0x40);
    g_assert_cmphex(qtest_readl(qts, HP_L2_MEM_REFRESH), ==, 0x40);

    /* TCM_INIT: EN (bit 0), CNT_RESET (bit 1), DONE (bit 2). */
    qtest_writel(qts, HP_TCM_INIT, 0x3);
    g_assert_cmphex(qtest_readl(qts, HP_TCM_INIT), ==, 0x3);
    qtest_writel(qts, HP_TCM_INIT, 0x1);
    g_assert_cmphex(qtest_readl(qts, HP_TCM_INIT), ==, 0x5);
    qtest_writel(qts, HP_TCM_INIT, 0xff);
    g_assert_cmphex(qtest_readl(qts, HP_TCM_INIT), ==, 0x3);
    qtest_quit(qts);
}

/*
 * The HP2LP groups: source 21 + k is bit k of the groups, so
 * CPU_INTR_FROM_CPU_0 (source 79) is bit 26 of group 1. The status shows
 * the enabled sources; HP_INTR (line 0) and the HP wakeup (line 1) are
 * high while one is.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_hp2lp(void)
{
    QTestState *qts = start();
    const uint32_t bit = 1u << (SRC_FROM_CPU0 - 21 - 32);

    qtest_irq_intercept_out_named(qts, HP_SYS_PATH, "hp-intr");
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_writel(qts, HP_CPU_INT_FROM_CPU(0), 1);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_INTR_ST(1)), ==, bit);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_INTR_ST(0)), ==, 0);
    g_assert_true(qtest_get_irq(qts, 0));

    /* Disabled, the source is masked from the status and from HP_INTR. */
    qtest_writel(qts, HP_HP2LP_INTR_EN(1), ~bit);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_INTR_ST(1)), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, HP_HP2LP_INTR_EN(1), 0xffffffff);
    g_assert_true(qtest_get_irq(qts, 0));

    /* The source falls. */
    qtest_writel(qts, HP_CPU_INT_FROM_CPU(0), 0);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_INTR_ST(1)), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Software interrupt 3, source 82, is bit 29. */
    qtest_writel(qts, HP_CPU_INT_FROM_CPU(3), 1);
    g_assert_cmphex(qtest_readl(qts, HP_HP2LP_INTR_ST(1)), ==, bit << 3);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, HP_CPU_INT_FROM_CPU(3), 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);

    /* The wakeup enables select the same sources for the PMU. */
    qts = start();
    qtest_irq_intercept_out_named(qts, HP_SYS_PATH, "hp-wakeup");
    qtest_writel(qts, HP_CPU_INT_FROM_CPU(0), 1);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, HP_HP2LP_WAKEUP_EN(1), bit);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, HP_CPU_INT_FROM_CPU(0), 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* Make pad n a plain GPIO driving its output. */
static void gpio_drive(QTestState *qts, int n)
{
    qtest_writel(qts, IO_MUX_GPIO(n), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    qtest_writel(qts, GPIO_FUNC_OUT(n), SIG_GPIO_OUT);
    if (n < 32) {
        qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << n);
    } else {
        qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << (n - 32));
    }
}

static void gpio_set(QTestState *qts, int n, bool level)
{
    if (n < 32) {
        qtest_writel(qts, level ? GPIO_OUT_W1TS : GPIO_OUT_W1TC, 1u << n);
    } else {
        qtest_writel(qts, level ? GPIO_OUT1_W1TS : GPIO_OUT1_W1TC,
                     1u << (n - 32));
    }
}

/*
 * The pads' holds: HP_SYSTEM_GPIO_O_HOLD_CTRL0 bit n holds GPIO(16 + n),
 * CTRL1 bit n GPIO(48 + n); LP_SYSTEM_PAD_RTC_HOLD_CTRL0 bit n holds GPIO
 * n; a pad held by either keeps its level.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_pad_hold(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");

    gpio_drive(qts, 16);
    gpio_set(qts, 16, true);
    g_assert_true(qtest_get_irq(qts, 16));
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL0, 1u << 0);
    gpio_set(qts, 16, false);
    g_assert_true(qtest_get_irq(qts, 16));
    qtest_writel(qts, LP_PAD_RTC_HOLD_CTRL0, 1u << 16);
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL0, 0);
    g_assert_true(qtest_get_irq(qts, 16));
    qtest_writel(qts, LP_PAD_RTC_HOLD_CTRL0, 0);
    g_assert_false(qtest_get_irq(qts, 16));

    gpio_drive(qts, 50);
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL1, 1u << 2);
    gpio_set(qts, 50, true);
    g_assert_false(qtest_get_irq(qts, 50));
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL1, 0);
    g_assert_true(qtest_get_irq(qts, 50));

    /* GPIO2 has only LP_SYSTEM's hold. */
    gpio_drive(qts, 2);
    qtest_writel(qts, LP_PAD_RTC_HOLD_CTRL0, 1u << 2);
    gpio_set(qts, 2, true);
    g_assert_false(qtest_get_irq(qts, 2));
    qtest_writel(qts, HP_GPIO_O_HOLD_CTRL0, 0xffffffff);
    qtest_writel(qts, LP_PAD_RTC_HOLD_CTRL0, 0);
    g_assert_true(qtest_get_irq(qts, 2));
    qtest_quit(qts);
}

/* A sample of LP_SYSTEM's registers, at their reset values. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_lp_reset_values(void)
{
    static const RegReset regs[] = {
        { LP_VER_DATE, 0x20250214 },
        { LP_SYS_CTRL, SYS_CTRL_RESET },
        { LP_CLK_CTRL, 0x4001 },
        { LP_RST_CTRL, 0x3 },
        { LP_CORE_BOOT_ADDR, LP_ROM_BASE },
        { LP_STORE(0), 0 },
        { LP_STORE(15), 0 },
        { LP_ANA_XPD_PAD_GROUP, 0xff },
        { LP_BACKUP_DMA_CFG0, 0x1906414a },
        { LP_INT_RAW, 0 },
        { LP_HP_MEM_AUX_CTRL, 0x2070 },
        { LP_HP_POR_RST_BYPASS_CTRL, 0xff00ff00 },
        { LP_CORE_AHB_TIMEOUT, 0x7fffff },
        { LP_CORE_ERR_RESP_DIS, 0 },
        { LP_RNG_CFG, 0x3 },
        { LP_DISCHARGE, 0x4 },
    };
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(qtest_readl(qts, regs[i].addr), ==, regs[i].reset);
    }

    /* The LP core sits at its reset vector, having taken no trap. */
    g_assert_cmphex(qtest_readl(qts, LP_CPU_DBG_PC), ==, LP_ROM_BASE);
    g_assert_cmphex(qtest_readl(qts, LP_CPU_EXC_PC), ==, 0);

    /* LP_CORE_BOOT_ADDR is read-write; ANA_FIB is read-only. */
    qtest_writel(qts, LP_CORE_BOOT_ADDR, 0x50108000);
    g_assert_cmphex(qtest_readl(qts, LP_CORE_BOOT_ADDR), ==, 0x50108000);
    qtest_writel(qts, LP_SYS_CTRL, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_CTRL), ==, 0x7b << 14);
    qtest_writel(qts, LP_CORE_ERR_RESP_DIS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_CORE_ERR_RESP_DIS), ==, 0x7);
    qtest_quit(qts);
}

/*
 * The legacy RNG gives a fresh word on each read while its timer or SAR
 * sampling runs, as from reset, and the last word otherwise.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_lp_rng(void)
{
    QTestState *qts = start();
    uint32_t a, b, c;

    a = qtest_readl(qts, LP_RNG_DATA);
    b = qtest_readl(qts, LP_RNG_DATA);
    c = qtest_readl(qts, LP_RNG_DATA);
    g_assert_false(a == b && b == c);
    qtest_writel(qts, LP_RNG_CFG, 0);
    a = qtest_readl(qts, LP_RNG_DATA);
    g_assert_cmphex(qtest_readl(qts, LP_RNG_DATA), ==, a);
    qtest_writel(qts, LP_RNG_CFG, 1u << 9);
    b = qtest_readl(qts, LP_RNG_DATA);
    c = qtest_readl(qts, LP_RNG_DATA);
    g_assert_false(a == b && b == c);
    qtest_quit(qts);
}

/*
 * The FIB control (TRM 21.2.2.2) of FIB bit @bit's enable, seen on output
 * @out: from reset LP_FIB_SEL picks the analog state ANA_FIB, whose bit
 * enables when set (@analog); picked instead, DIG_FIB's bit enables while
 * set; the other FIB bit's choice changes nothing.
 */
static void lp_fib_out(const char *out, int bit, int other, bool analog)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, LP_SYS_PATH, out);
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET);
    g_assert_cmpint(qtest_get_irq(qts, 0), ==, analog);
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET & ~LP_FIB_SEL(bit));
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, LP_SYS_CTRL,
                 SYS_CTRL_RESET & ~(LP_FIB_SEL(bit) | DIG_FIB(bit)));
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET & ~DIG_FIB(bit));
    g_assert_cmpint(qtest_get_irq(qts, 0), ==, analog);
    qtest_writel(qts, LP_SYS_CTRL,
                 SYS_CTRL_RESET & ~(LP_FIB_SEL(other) | DIG_FIB(other)));
    g_assert_cmpint(qtest_get_irq(qts, 0), ==, analog);
    /* ESP-IDF's start-up LP_FIB_SEL, 0xEF, leaves both to the analog. */
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET & ~LP_FIB_SEL(4));
    g_assert_cmpint(qtest_get_irq(qts, 0), ==, analog);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_lp_fib(void)
{
    lp_fib_out("swd-reset-en", FIB_SWD, FIB_BOD, false);
    lp_fib_out("bod-reset-en", FIB_BOD, FIB_SWD, true);
}

/*
 * The FIB control's brownout detector enable reaches the detector: with
 * DIG_FIB's bit picked and clear, VDD_ANA below its 2.4 V threshold makes
 * no brown-out; set again, it makes one, whose mode 1 reset (enabled by
 * the analog default) resets the system with cause 0x0F.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_lp_fib_bod(void)
{
    QTestState *qts = start();
    QDict *r;

    qtest_writel(qts, LP_SYS_CTRL,
                 SYS_CTRL_RESET & ~(LP_FIB_SEL(FIB_BOD) | DIG_FIB(FIB_BOD)));
    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': { 'path': %s, "
                  "'property': 'vdda-mv', 'value': 2300 } }", LP_ANA_PATH);
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0);
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET & ~LP_FIB_SEL(FIB_BOD));
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(LP_CAUSE(qtest_readl(qts, RESET_CAUSE)), ==, 0x0f);
    qtest_quit(qts);
}

/*
 * LP_SYSREG_INTR, source 19: the enabled raw bits, raised by the ETM's
 * ULP task and the slow clock's tick; INT_RAW and INT_CLR clear them.
 * LP_CORE_ETM_WAKEUP_FLAG sets on its input and clears through either of
 * its bits.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_lp_interrupts(void)
{
    QTestState *qts = start();

    qtest_set_irq_in(qts, LP_SYS_PATH, "slow-clk-tick", 0, 1);
    qtest_set_irq_in(qts, LP_SYS_PATH, "slow-clk-tick", 0, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INT_RAW), ==, LP_INT_SLOW_CLK_TICK);
    g_assert_cmphex(qtest_readl(qts, LP_INT_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(0)) & (1u << SRC_LP_SYSREG),
                    ==, 0);
    qtest_writel(qts, LP_INT_ENA, LP_INT_SLOW_CLK_TICK | LP_INT_ETM_TASK_ULP);
    g_assert_cmphex(qtest_readl(qts, LP_INT_ST), ==, LP_INT_SLOW_CLK_TICK);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(0)) & (1u << SRC_LP_SYSREG),
                    ==, 1u << SRC_LP_SYSREG);
    qtest_set_irq_in(qts, LP_SYS_PATH, "etm-task-ulp-int", 0, 1);
    g_assert_cmphex(qtest_readl(qts, LP_INT_ST), ==,
                    LP_INT_SLOW_CLK_TICK | LP_INT_ETM_TASK_ULP);
    qtest_writel(qts, LP_INT_CLR, LP_INT_SLOW_CLK_TICK);
    g_assert_cmphex(qtest_readl(qts, LP_INT_RAW), ==, LP_INT_ETM_TASK_ULP);
    qtest_writel(qts, LP_INT_RAW, LP_INT_ETM_TASK_ULP);
    g_assert_cmphex(qtest_readl(qts, LP_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(0)) & (1u << SRC_LP_SYSREG),
                    ==, 0);

    qtest_set_irq_in(qts, LP_SYS_PATH, "lp-core-etm-wakeup", 0, 1);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_CTRL), ==,
                    SYS_CTRL_RESET | ETM_WAKEUP_FLAG);
    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET | ETM_WAKEUP_FLAG_CLR);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_CTRL), ==, SYS_CTRL_RESET);
    qtest_quit(qts);
}

/*
 * SYS_SW_RST: an HP core reset with cause 0x03 for both HP cores, which
 * resets HP_SYSTEM and leaves LP_SYSTEM and the LP core's cause (cleared
 * here) as they were.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_sw_reset(void)
{
    QTestState *qts = start();
    uint32_t cause;

    qtest_writel(qts, RESET_CAUSE, CAUSE_CLEAR_ALL);
    qtest_writel(qts, LP_STORE(3), 0x5eed1e55);
    qtest_writel(qts, LP_CORE_BOOT_ADDR, 0x50104000);
    qtest_writel(qts, HP_CLK_EN, 1);
    qtest_writel(qts, HP_CORE_ERR_RESP_DIS, 0x5);

    qtest_writel(qts, LP_SYS_CTRL, SYS_CTRL_RESET | SYS_SW_RST);
    qtest_clock_step(qts, 10000);
    cause = qtest_readl(qts, RESET_CAUSE);
    g_assert_cmphex(CORE0_CAUSE(cause), ==, CAUSE_SW_SYS);
    g_assert_cmphex(CORE1_CAUSE(cause), ==, CAUSE_SW_SYS);
    g_assert_cmphex(LP_CAUSE(cause), ==, 0);

    /* SYS_SW_RST is write-only; LP_SYSTEM kept its state. */
    g_assert_cmphex(qtest_readl(qts, LP_SYS_CTRL), ==, SYS_CTRL_RESET);
    g_assert_cmphex(qtest_readl(qts, LP_STORE(3)), ==, 0x5eed1e55);
    g_assert_cmphex(qtest_readl(qts, LP_CORE_BOOT_ADDR), ==, 0x50104000);
    g_assert_cmphex(qtest_readl(qts, HP_CLK_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, HP_CORE_ERR_RESP_DIS), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/sysreg/hp/reset-values", test_hp_reset_values);
    qtest_add_func("esp32p4/sysreg/hp/access", test_hp_access);
    qtest_add_func("esp32p4/sysreg/hp/interrupt-regs",
                   test_hp_interrupt_regs);
    qtest_add_func("esp32p4/sysreg/hp/memory-init", test_hp_memory_init);
    qtest_add_func("esp32p4/sysreg/hp/hp2lp", test_hp2lp);
    qtest_add_func("esp32p4/sysreg/pad-hold", test_pad_hold);
    qtest_add_func("esp32p4/sysreg/lp/reset-values", test_lp_reset_values);
    qtest_add_func("esp32p4/sysreg/lp/rng", test_lp_rng);
    qtest_add_func("esp32p4/sysreg/lp/fib", test_lp_fib);
    qtest_add_func("esp32p4/sysreg/lp/fib-bod", test_lp_fib_bod);
    qtest_add_func("esp32p4/sysreg/lp/interrupts", test_lp_interrupts);
    qtest_add_func("esp32p4/sysreg/lp/sw-reset", test_sw_reset);

    return g_test_run();
}
