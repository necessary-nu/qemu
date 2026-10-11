/*
 * QTest testcase for the ESP32-P4 debug assistant
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The cores do not run under qtest: each sits at its reset vector with SP
 * 0, which the SP monitor compares with its bounds as they are set. The
 * region monitors, which watch the cores' own accesses, are left to the
 * guest tests. Interrupts are seen in the interrupt matrix's status of
 * source 127; core resets come from LP_CLKRST_HPCORE<n>_SW_RESET.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ASSIST              0x3ff06000
#define CORE(n)             (ASSIST + 0x80 * (n))
#define INTR_ENA(n)         (CORE(n) + 0x00)
#define INTR_RAW(n)         (CORE(n) + 0x04)
#define INTR_RLS(n)         (CORE(n) + 0x08)
#define INTR_CLR(n)         (CORE(n) + 0x0c)
#define AREA(n, k)          (CORE(n) + 0x10 + 4 * (k))
#define AREA_PC(n)          (CORE(n) + 0x30)
#define AREA_SP(n)          (CORE(n) + 0x34)
#define SP_MIN(n)           (CORE(n) + 0x38)
#define SP_MAX(n)           (CORE(n) + 0x3c)
#define SP_PC(n)            (CORE(n) + 0x40)
#define RCD_EN(n)           (CORE(n) + 0x44)
#define RCD_PDEBUGPC(n)     (CORE(n) + 0x48)
#define RCD_PDEBUGSP(n)     (CORE(n) + 0x4c)
#define EXC_MON(n, k)       (CORE(n) + 0x50 + 4 * (k))
#define LASTPC(n)           (CORE(n) + 0x70)
#define DEBUG_MODE(n)       (CORE(n) + 0x74)
#define LIMIT_CYCLE(k)      (ASSIST + 0x100 + 4 * (k))
#define CLOCK_GATE          (ASSIST + 0x108)
#define DATE                (ASSIST + 0x3fc)

#define SP_SPILL_MIN        (1u << 8)
#define SP_SPILL_MAX        (1u << 9)
#define RCD_RECORDEN        (1u << 0)
#define RCD_PDEBUGEN        (1u << 1)

#define INTMTX_STATUS3      0x500d620c
#define SRC_ASSIST_BIT      (1u << (127 - 96))

#define LP_CLKRST           0x50111000
#define HPCPU_RESET_CTRL0   (LP_CLKRST + 0x14)
#define HPCORE_SW_RESET(n)  (1u << (13 + 16 * (n)))

#define HP_RESET_VECTOR     0x4fc00000u

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

static bool assist_irq(QTestState *qts)
{
    return qtest_readl(qts, INTMTX_STATUS3) & SRC_ASSIST_BIT;
}

/* Both cores' registers at their reset values. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    for (int n = 0; n < 2; n++) {
        g_assert_cmphex(qtest_readl(qts, INTR_ENA(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, INTR_RAW(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, INTR_RLS(n)), ==, 0);
        for (int k = 0; k < 8; k++) {
            g_assert_cmphex(qtest_readl(qts, AREA(n, k)), ==,
                            k % 2 ? 0 : 0xffffffff);
        }
        g_assert_cmphex(qtest_readl(qts, AREA_PC(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, AREA_SP(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, SP_MIN(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, SP_MAX(n)), ==, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, SP_PC(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, RCD_EN(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGPC(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGSP(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, EXC_MON(n, 0)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, LASTPC(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, DEBUG_MODE(n)), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, LIMIT_CYCLE(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CLOCK_GATE), ==, 1);
    g_assert_cmphex(qtest_readl(qts, DATE), ==, 0x2109130);
    g_assert_false(assist_irq(qts));
    qtest_quit(qts);
}

/* The registers' writable bits, and the read-only ones left alone. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_access(void)
{
    QTestState *qts = start();

    qtest_writel(qts, INTR_RLS(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INTR_RLS(1)), ==, 0x3ff);
    qtest_writel(qts, AREA(1, 2), 0x3ff00010);
    g_assert_cmphex(qtest_readl(qts, AREA(1, 2)), ==, 0x3ff00010);
    qtest_writel(qts, RCD_EN(0), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RCD_EN(0)), ==, 0x3);
    qtest_writel(qts, LIMIT_CYCLE(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LIMIT_CYCLE(1)), ==, 0xfffff);
    qtest_writel(qts, SP_PC(0), 0x1234);
    qtest_writel(qts, AREA_PC(0), 0x1234);
    qtest_writel(qts, LASTPC(0), 0x1234);
    g_assert_cmphex(qtest_readl(qts, SP_PC(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AREA_PC(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LASTPC(0)), ==, 0);
    qtest_quit(qts);
}

/*
 * The SP monitor: SP (0) below SP_MIN raises SP_SPILL_MIN at once, with
 * SP_PC at the core's PC; it is a level that follows the bounds. RLS lets
 * it latch the interrupt, which INTR_CLR clears, and which latches again
 * while SP is still out of bounds.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_sp_monitor(void)
{
    QTestState *qts = start();

    qtest_writel(qts, SP_MIN(0), 0x4ff80000);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(0)), ==, 0);
    qtest_writel(qts, INTR_ENA(0), SP_SPILL_MIN | SP_SPILL_MAX);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(0)), ==, SP_SPILL_MIN);
    g_assert_cmphex(qtest_readl(qts, SP_PC(0)), ==, HP_RESET_VECTOR);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(1)), ==, 0);
    g_assert_false(assist_irq(qts));

    /* Interrupt enabled: latched. */
    qtest_writel(qts, INTR_RLS(0), SP_SPILL_MIN);
    g_assert_true(assist_irq(qts));
    qtest_writel(qts, INTR_CLR(0), SP_SPILL_MIN);
    g_assert_true(assist_irq(qts));
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(0)), ==, SP_SPILL_MIN);

    /* SP back in bounds: the raw bit falls, the latched interrupt holds. */
    qtest_writel(qts, SP_MIN(0), 0);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(0)), ==, 0);
    g_assert_true(assist_irq(qts));
    qtest_writel(qts, INTR_CLR(0), SP_SPILL_MIN);
    g_assert_false(assist_irq(qts));

    /* Monitoring off: no raw bit, whatever the bounds. */
    qtest_writel(qts, INTR_ENA(0), 0);
    qtest_writel(qts, SP_MIN(0), 0x100);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(0)), ==, 0);
    g_assert_false(assist_irq(qts));

    /* Core 1's monitor, its own. */
    qtest_writel(qts, SP_MIN(1), 0x10);
    qtest_writel(qts, INTR_RLS(1), SP_SPILL_MIN);
    qtest_writel(qts, INTR_ENA(1), SP_SPILL_MIN);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(1)), ==, SP_SPILL_MIN);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(0)), ==, 0);
    g_assert_true(assist_irq(qts));
    qtest_writel(qts, INTR_ENA(1), 0);
    qtest_writel(qts, INTR_CLR(1), SP_SPILL_MIN);
    g_assert_false(assist_irq(qts));
    qtest_quit(qts);
}

/*
 * PC logging: with RECORDEN and PDEBUGEN the records follow the core; a
 * reset of the core keeps them, clears RCD_EN, and resets nothing else
 * of theirs.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_pc_logging(void)
{
    QTestState *qts = start();

    qtest_writel(qts, RCD_EN(0), RCD_RECORDEN);
    g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGPC(0)), ==, 0);
    qtest_writel(qts, RCD_EN(0), RCD_RECORDEN | RCD_PDEBUGEN);
    g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGPC(0)), ==, HP_RESET_VECTOR);
    g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGSP(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGPC(1)), ==, 0);

    qtest_writel(qts, SP_MIN(0), 0x100);
    qtest_writel(qts, INTR_ENA(0), SP_SPILL_MIN);
    qtest_writel(qts, HPCPU_RESET_CTRL0,
                 qtest_readl(qts, HPCPU_RESET_CTRL0) | HPCORE_SW_RESET(0));
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, RCD_EN(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RCD_PDEBUGPC(0)), ==, HP_RESET_VECTOR);

    /* A CPU reset leaves the block's registers alone. */
    g_assert_cmphex(qtest_readl(qts, INTR_ENA(0)), ==, SP_SPILL_MIN);
    g_assert_cmphex(qtest_readl(qts, SP_MIN(0)), ==, 0x100);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/assist-debug/reset-values", test_reset_values);
    qtest_add_func("esp32p4/assist-debug/access", test_access);
    qtest_add_func("esp32p4/assist-debug/sp-monitor", test_sp_monitor);
    qtest_add_func("esp32p4/assist-debug/pc-logging", test_pc_logging);

    return g_test_run();
}
