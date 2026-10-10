/*
 * QTest testcase for the ESP32-P4 system timer
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The counters count CNT_CLK, the 40 MHz XTAL_CLK divided by 2.5: an edge
 * after three crystal cycles and another after two more, 16 per
 * microsecond. Virtual time starts at the machine's reset, when UNIT0
 * starts counting. Interrupts are seen in the interrupt matrix's status of
 * sources 53 to 55.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/misc/esp32p4_clkrst.h"

#define SYSTIMER            0x500e2000
#define CONF                0x00
#define UNIT_OP(n)          (0x04 + 4 * (n))
#define UNIT_LOAD_HI(n)     (0x0c + 8 * (n))
#define UNIT_LOAD_LO(n)     (0x10 + 8 * (n))
#define TARGET_HI(x)        (0x1c + 8 * (x))
#define TARGET_LO(x)        (0x20 + 8 * (x))
#define TARGET_CONF(x)      (0x34 + 4 * (x))
#define VALUE_HI(n)         (0x40 + 8 * (n))
#define VALUE_LO(n)         (0x44 + 8 * (n))
#define COMP_LOAD(x)        (0x50 + 4 * (x))
#define UNIT_LOAD(n)        (0x5c + 4 * (n))
#define INT_ENA             0x64
#define INT_RAW             0x68
#define INT_CLR             0x6c
#define INT_ST              0x70
#define REAL_LO(x)          (0x74 + 8 * (x))
#define REAL_HI(x)          (0x78 + 8 * (x))
#define DATE                0xfc

#define CONF_ETM_EN         (1u << 1)
#define CONF_TARGET_EN(x)   (1u << (24 - (x)))
#define CONF_UNIT_EN(n)     (1u << (30 - (n)))
#define CONF_RESET          0x46000000u
#define OP_VALID            (1u << 29)
#define OP_UPDATE           (1u << 30)
#define TCONF_PERIOD_MODE   (1u << 30)
#define TCONF_UNIT1         (1u << 31)

#define CLKRST              "/machine/clkrst"
#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL2       (HP_CLKRST + 0x1c)
#define PERI_CLK_CTRL21     (HP_CLKRST + 0x98)
#define HP_RST_EN1          (HP_CLKRST + 0xc4)
#define SYSTIMER_APB_CLK_EN (1u << 23)
#define SYSTIMER_SRC_RC_FAST (1u << 29)
#define SYSTIMER_CLK_EN     (1u << 30)
#define RST_EN_STIMER       (1u << 5)

#define INTMTX_STATUS1      0x500d6204
#define SRC_TARGET0_BIT     (53 - 32)

/* Nanoseconds per five crystal cycles, two CNT_CLK edges. */
#define NS_PER_2_COUNTS     125

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, SYSTIMER + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, SYSTIMER + off, v);
}

/* Virtual time, which starts at 0 with each test's machine. */
static int64_t now;

static QTestState *start(void)
{
    now = 0;
    return qtest_init("-M esp32p4");
}

static void step(QTestState *qts, int64_t ns)
{
    now = qtest_clock_step(qts, ns);
}

/* Latch and read counter @n. */
static uint64_t unit_value(QTestState *qts, int n)
{
    wr(qts, UNIT_OP(n), OP_UPDATE);
    g_assert_cmphex(rd(qts, UNIT_OP(n)), ==, OP_VALID);
    return ((uint64_t)rd(qts, VALUE_HI(n)) << 32) | rd(qts, VALUE_LO(n));
}

static void load_unit(QTestState *qts, int n, uint64_t v)
{
    wr(qts, UNIT_LOAD_HI(n), v >> 32);
    wr(qts, UNIT_LOAD_LO(n), (uint32_t)v);
    wr(qts, UNIT_LOAD(n), 1);
}

/* Step to the next start of a pair of CNT_CLK cycles. */
static void align(QTestState *qts)
{
    if (now % NS_PER_2_COUNTS) {
        step(qts, NS_PER_2_COUNTS - now % NS_PER_2_COUNTS);
    }
}

static bool target_irq(QTestState *qts, int x)
{
    return (qtest_readl(qts, INTMTX_STATUS1) >> (SRC_TARGET0_BIT + x)) & 1;
}

/* Every register at its reset value. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_reset(void)
{
    QTestState *qts = start();
    uint32_t off;

    g_assert_cmphex(rd(qts, CONF), ==, CONF_RESET);
    g_assert_cmphex(rd(qts, DATE), ==, 0x02201073);
    for (off = 0x04; off < 0x8c; off += 4) {
        g_assert_cmphex(rd(qts, off), ==, 0);
    }
    qtest_quit(qts);
}

/*
 * UNIT0 counts from reset at 16 MHz, its edges 3 then 2 crystal cycles
 * apart; UNIT1 does not count until enabled.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_counting(void)
{
    QTestState *qts = start();

    step(qts, 74);
    g_assert_cmpuint(unit_value(qts, 0), ==, 0);
    step(qts, 1);
    g_assert_cmpuint(unit_value(qts, 0), ==, 1);
    step(qts, 49);
    g_assert_cmpuint(unit_value(qts, 0), ==, 1);
    step(qts, 1);
    g_assert_cmpuint(unit_value(qts, 0), ==, 2);
    step(qts, 1000000 - 125);
    g_assert_cmpuint(unit_value(qts, 0), ==, 16000);
    g_assert_cmpuint(unit_value(qts, 1), ==, 0);

    /* The latch holds until the next update; writing VALID clears it. */
    step(qts, 1000);
    g_assert_cmphex(rd(qts, VALUE_LO(0)), ==, 16000);
    wr(qts, UNIT_OP(0), OP_VALID);
    g_assert_cmphex(rd(qts, UNIT_OP(0)), ==, 0);
    g_assert_cmpuint(unit_value(qts, 0), ==, 16016);

    /* A load takes effect at once; the counter counts on from it. */
    align(qts);
    load_unit(qts, 0, 0xfffffffffff00ull);
    g_assert_cmpuint(unit_value(qts, 0), ==, 0xfffffffffff00ull);
    step(qts, 16 * 125);
    g_assert_cmpuint(unit_value(qts, 0), ==, 0xfffffffffff00ull + 32);
    /* It wraps at 52 bits. */
    step(qts, 128 * 125);
    g_assert_cmpuint(unit_value(qts, 0), ==, 0x20);

    /* Disabled, it holds its count; enabled again, it counts on. */
    wr(qts, CONF, CONF_RESET & ~CONF_UNIT_EN(0));
    step(qts, 10000);
    g_assert_cmpuint(unit_value(qts, 0), ==, 0x20);
    wr(qts, CONF, CONF_RESET | CONF_UNIT_EN(1));
    align(qts);
    load_unit(qts, 0, 0);
    load_unit(qts, 1, 100);
    step(qts, 1000);
    g_assert_cmpuint(unit_value(qts, 0), ==, 16);
    g_assert_cmpuint(unit_value(qts, 1), ==, 116);
    qtest_quit(qts);
}

/* A target-mode alarm at exactly its count, once, and its interrupt. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_target(void)
{
    QTestState *qts = start();

    align(qts);
    load_unit(qts, 0, 0);
    wr(qts, TARGET_HI(0), 0);
    wr(qts, TARGET_LO(0), 1000);
    wr(qts, TARGET_CONF(0), 0);
    wr(qts, COMP_LOAD(0), 1);
    wr(qts, INT_ENA, 1);
    wr(qts, CONF, CONF_RESET | CONF_TARGET_EN(0));
    g_assert_cmphex(rd(qts, REAL_LO(0)), ==, 1000);
    g_assert_cmphex(rd(qts, REAL_HI(0)), ==, 0);

    /* Count 1000 comes 2500 crystal cycles in. */
    step(qts, 62499);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert(!target_irq(qts, 0));
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);
    g_assert_cmphex(rd(qts, INT_ST), ==, 1);
    g_assert(target_irq(qts, 0));

    /* Cleared, it does not alarm again as the count moves past. */
    wr(qts, INT_CLR, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert(!target_irq(qts, 0));
    step(qts, 100000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);

    /* The raw bit also clears when written with 1. */
    wr(qts, COMP_LOAD(0), 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);
    wr(qts, INT_RAW, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);

    /*
     * A target up to 2^51 - 1 behind alarms at once when loaded; one
     * further behind is ahead, reached after the counter wraps.
     */
    load_unit(qts, 0, 1ull << 51);
    wr(qts, TARGET_HI(0), 0);
    wr(qts, TARGET_LO(0), 1);
    wr(qts, COMP_LOAD(0), 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);
    wr(qts, INT_CLR, 1);
    load_unit(qts, 0, (1ull << 51) + 1);
    wr(qts, TARGET_LO(0), 0);
    wr(qts, COMP_LOAD(0), 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    align(qts);
    load_unit(qts, 0, 0xffffffffffff0ull);
    step(qts, 8 * 125 - 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    step(qts, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);
    wr(qts, INT_CLR, 1);

    /* Disabled, the comparator does not alarm; enabled, it compares again. */
    wr(qts, CONF, CONF_RESET);
    load_unit(qts, 0, 0);
    wr(qts, TARGET_LO(0), 10);
    wr(qts, COMP_LOAD(0), 1);
    step(qts, 10000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    wr(qts, CONF, CONF_RESET | CONF_TARGET_EN(0));
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);
    qtest_quit(qts);
}

/* Period mode: an alarm every period from the load. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_period(void)
{
    QTestState *qts = start();
    int i;

    align(qts);
    load_unit(qts, 1, 5000);
    wr(qts, CONF, CONF_RESET | CONF_UNIT_EN(1));
    wr(qts, TARGET_CONF(2), TCONF_UNIT1 | TCONF_PERIOD_MODE | 160);
    wr(qts, COMP_LOAD(2), 1);
    wr(qts, INT_ENA, 4);
    wr(qts, CONF, CONF_RESET | CONF_UNIT_EN(1) | CONF_TARGET_EN(2));
    g_assert_cmphex(rd(qts, REAL_LO(2)), ==, 5160);
    for (i = 1; i <= 3; i++) {
        step(qts, 10000 - 1);
        g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
        step(qts, 1);
        g_assert_cmphex(rd(qts, INT_RAW), ==, 4);
        g_assert(target_irq(qts, 2));
        g_assert_cmphex(rd(qts, REAL_LO(2)), ==, 5000 + 160 * (i + 1));
        wr(qts, INT_CLR, 4);
    }
    /* The period takes effect at the next load. */
    wr(qts, TARGET_CONF(2), TCONF_UNIT1 | TCONF_PERIOD_MODE | 16);
    step(qts, 10000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 4);
    wr(qts, INT_CLR, 4);
    wr(qts, COMP_LOAD(2), 1);
    step(qts, 1000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 4);
    qtest_quit(qts);
}

/*
 * UNIT1 stops while HP core 0 or 1 is stalled (STALL_EN set at reset),
 * UNIT0 does not (clear); the stall comes from the reset and clock
 * control, here the PMU's.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_stall(void)
{
    QTestState *qts = start();

    wr(qts, CONF, CONF_RESET | CONF_UNIT_EN(1));
    align(qts);
    load_unit(qts, 0, 0);
    load_unit(qts, 1, 0);
    step(qts, 1000);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 1);
    step(qts, 1000);
    g_assert_cmpuint(unit_value(qts, 0), ==, 32);
    g_assert_cmpuint(unit_value(qts, 1), ==, 16);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 0);
    step(qts, 1000);
    g_assert_cmpuint(unit_value(qts, 1), ==, 32);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 0, 1);
    step(qts, 1000);
    g_assert_cmpuint(unit_value(qts, 1), ==, 32);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 0, 0);

    /* Without STALL_EN for core 0, UNIT1 counts on through its stall. */
    wr(qts, CONF, (CONF_RESET | CONF_UNIT_EN(1)) & ~(1u << 26));
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 0, 1);
    step(qts, 1000);
    g_assert_cmpuint(unit_value(qts, 1), ==, 48);
    qtest_quit(qts);
}

/*
 * The function clock: gated, nothing counts or alarms; from RC_FAST_CLK
 * (20 MHz) the counters count at 8 MHz. The block's reset and bus clock.
 */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_clocks(void)
{
    QTestState *qts = start();
    uint32_t ctrl21 = qtest_readl(qts, PERI_CLK_CTRL21);

    align(qts);
    load_unit(qts, 0, 0);
    wr(qts, TARGET_LO(0), 20);
    wr(qts, COMP_LOAD(0), 1);
    qtest_writel(qts, PERI_CLK_CTRL21, ctrl21 & ~SYSTIMER_CLK_EN);
    wr(qts, CONF, CONF_RESET | CONF_TARGET_EN(0));
    step(qts, 10000);
    g_assert_cmpuint(unit_value(qts, 0), ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    qtest_writel(qts, PERI_CLK_CTRL21, ctrl21);
    step(qts, 1250);
    g_assert_cmpuint(unit_value(qts, 0), ==, 20);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);

    qtest_writel(qts, PERI_CLK_CTRL21, ctrl21 | SYSTIMER_SRC_RC_FAST);
    load_unit(qts, 0, 0);
    step(qts, 100000);
    g_assert_cmpuint(unit_value(qts, 0), ==, 800);
    qtest_writel(qts, PERI_CLK_CTRL21, ctrl21);

    /* Held in reset, the block's registers hold their reset values. */
    wr(qts, INT_ENA, 7);
    qtest_writel(qts, HP_RST_EN1, RST_EN_STIMER);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0);
    wr(qts, INT_ENA, 7);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0);
    qtest_writel(qts, HP_RST_EN1, 0);
    g_assert_cmphex(rd(qts, CONF), ==, CONF_RESET);
    g_assert_cmpuint(unit_value(qts, 0), <, 16);

    /* With its bus clock gated, writes are lost. */
    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) & ~SYSTIMER_APB_CLK_EN);
    wr(qts, INT_ENA, 7);
    qtest_writel(qts, SOC_CLK_CTRL2,
                 qtest_readl(qts, SOC_CLK_CTRL2) | SYSTIMER_APB_CLK_EN);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0);
    qtest_quit(qts);
}

/* Byte and halfword accesses read and write their bytes of a register. */
/* [spec:nuos:req:emu.esp32p4.timers/test] */
static void test_byte_access(void)
{
    QTestState *qts = start();

    qtest_writeb(qts, SYSTIMER + TARGET_LO(1) + 1, 0x12);
    qtest_writew(qts, SYSTIMER + TARGET_LO(1) + 2, 0x3456);
    g_assert_cmphex(rd(qts, TARGET_LO(1)), ==, 0x34561200);
    g_assert_cmphex(qtest_readb(qts, SYSTIMER + TARGET_LO(1) + 2), ==,
                    0x56);
    /* The update bit's byte triggers it. */
    qtest_writeb(qts, SYSTIMER + UNIT_OP(0) + 3, OP_UPDATE >> 24);
    g_assert_cmphex(rd(qts, UNIT_OP(0)), ==, OP_VALID);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/systimer/reset", test_reset);
    qtest_add_func("/esp32p4/systimer/counting", test_counting);
    qtest_add_func("/esp32p4/systimer/target", test_target);
    qtest_add_func("/esp32p4/systimer/period", test_period);
    qtest_add_func("/esp32p4/systimer/stall", test_stall);
    qtest_add_func("/esp32p4/systimer/clocks", test_clocks);
    qtest_add_func("/esp32p4/systimer/byte-access", test_byte_access);
    return g_test_run();
}
