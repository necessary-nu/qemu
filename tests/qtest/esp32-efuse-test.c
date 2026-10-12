/*
 * QTest testcase for the ESP32 eFuse controller's command timing
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A command takes APB_CLK cycles from EFUSE_CLK and EFUSE_DAC_CONF as
 * hw/nvram/esp32_efuse.c interprets them: an access to the array takes
 * CLK_SEL0 + CLK_SEL1 cycles, a read accesses its 31 words, and a program
 * ramps the programming voltage up and down through 255 DAC steps of
 * DAC_CLK_DIV cycles each and accesses its 992 bits. APB_CLK is the
 * 40 MHz XTAL_CLK out of reset: 25 ns a cycle.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define EFUSE                   0x3ff5a000
#define EFUSE_CLK               (EFUSE + 0x0f8)
#define EFUSE_CONF              (EFUSE + 0x0fc)
#define EFUSE_CMD               (EFUSE + 0x104)
#define EFUSE_INT_RAW           (EFUSE + 0x108)
#define EFUSE_INT_CLR           (EFUSE + 0x114)
#define EFUSE_DAC_CONF          (EFUSE + 0x118)
#define CONF_READ               0x5aa5
#define CONF_PGM                0x5a5a
#define CMD_READ                1
#define CMD_PGM                 2
#define CLK(sel0, sel1)         (((sel1) << 8) | (sel0))

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define PERIP_EFUSE             (1u << 14)

#define APB_NS                  25
/* A read at the reset values, and a read and a program at table 5.3-4's */
#define READ_RESET_NS           (31 * (0x52 + 0x40) * APB_NS)
#define READ_40M_NS             (31 * (160 + 255) * APB_NS)
#define PGM_40M_NS              ((2 * 255 * 80 + 992 * (160 + 255)) * APB_NS)

/* Assert that the command just started ends exactly @ns from now. */
static void assert_runs(QTestState *qts, uint32_t cmd, uint64_t ns)
{
    qtest_clock_step(qts, ns - 1);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CMD), ==, cmd);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, EFUSE_INT_RAW) & cmd, ==, cmd);
    qtest_writel(qts, EFUSE_INT_CLR, cmd);
}

static void command(QTestState *qts, uint32_t conf, uint32_t cmd)
{
    qtest_writel(qts, EFUSE_CONF, conf);
    qtest_writel(qts, EFUSE_CMD, cmd);
}

/*
 * The timing fields' reset values (TRM v5.8, registers 5.21 and 5.28),
 * and the read every reset makes, at them.
 */
static void test_reset(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    g_assert_cmphex(qtest_readl(qts, EFUSE_CLK), ==, CLK(0x52, 0x40));
    g_assert_cmphex(qtest_readl(qts, EFUSE_DAC_CONF), ==, 40);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CONF), ==, 0);
    assert_runs(qts, CMD_READ, READ_RESET_NS);
    qtest_quit(qts);
}

/* A read and a program at the TRM's settings for a 40 MHz APB_CLK */
static void test_table_40m(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    qtest_clock_step(qts, READ_RESET_NS);
    qtest_writel(qts, EFUSE_CLK, CLK(160, 255));
    qtest_writel(qts, EFUSE_DAC_CONF, 80);
    command(qts, CONF_READ, CMD_READ);
    assert_runs(qts, CMD_READ, READ_40M_NS);
    command(qts, CONF_PGM, CMD_PGM);
    assert_runs(qts, CMD_PGM, PGM_40M_NS);
    qtest_quit(qts);
}

/* A read stands still while DPORT gates the controller's clock. */
static void test_gated(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    uint32_t clk_en;

    qtest_clock_step(qts, READ_RESET_NS);
    command(qts, CONF_READ, CMD_READ);
    qtest_clock_step(qts, 50000);
    clk_en = qtest_readl(qts, DPORT_PERIP_CLK_EN);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, clk_en & ~PERIP_EFUSE);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CMD), ==, CMD_READ);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, clk_en);
    assert_runs(qts, CMD_READ, READ_RESET_NS - 50000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32-efuse/reset", test_reset);
    qtest_add_func("esp32-efuse/table-40m", test_table_40m);
    qtest_add_func("esp32-efuse/gated", test_gated);

    return g_test_run();
}
