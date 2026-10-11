/*
 * QTest testcase for the ESP32-P4 LP Mailbox
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Interrupts are seen in the interrupt matrix's status of sources 4
 * (MB_HP_INTR) and 5 (MB_LP_INTR).
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define MB                  0x50118000
#define MESSAGE(n)          (4 * (n))
#define LP_INT_RAW          0x40
#define LP_INT_ST           0x44
#define LP_INT_ENA          0x48
#define LP_INT_CLR          0x4c
#define HP_INT_RAW          0x50
#define HP_INT_ST           0x54
#define HP_INT_ENA          0x58
#define HP_INT_CLR          0x5c
#define REG_CLK_EN          0x60

#define LP_CLKRST_RST_EN    0x5011100c
#define RST_EN_LP_MAILBOX   (1u << 29)
#define INTMTX_STATUS0      0x500d6200
#define SRC_MB_HP           4
#define SRC_MB_LP           5

/* A board strapped for Joint Download Boot: no flash boot protection. */
static QTestState *start(void)
{
    return qtest_init("-M esp32p4,strap-mode=0x04");
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, MB + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, MB + off, v);
}

static bool source(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> src) & 1;
}

/* Every register at its reset value: messages 0 to 7's interrupts on. */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_reset(void)
{
    QTestState *qts = start();
    int n;

    for (n = 0; n < 16; n++) {
        g_assert_cmphex(rd(qts, MESSAGE(n)), ==, 0);
    }
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, LP_INT_ST), ==, 0);
    g_assert_cmphex(rd(qts, LP_INT_ENA), ==, 0xff);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, HP_INT_ST), ==, 0);
    g_assert_cmphex(rd(qts, HP_INT_ENA), ==, 0xff);
    g_assert_cmphex(rd(qts, REG_CLK_EN), ==, 0);
    /* Past the registers, reserved space reads 0. */
    g_assert_cmphex(rd(qts, 0x64), ==, 0);
    qtest_quit(qts);
}

/*
 * A write to message n, whatever its value, raises both MB_LP_n_INT and
 * MB_HP_n_INT; the enabled ones drive MB_LP_INTR and MB_HP_INTR; the raw
 * bits are read-only and clear only through the clear registers.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_messages(void)
{
    QTestState *qts = start();

    wr(qts, MESSAGE(3), 0xdeadbeef);
    g_assert_cmphex(rd(qts, MESSAGE(3)), ==, 0xdeadbeef);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 1u << 3);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, 1u << 3);
    g_assert_cmphex(rd(qts, LP_INT_ST), ==, 1u << 3);
    g_assert_cmphex(rd(qts, HP_INT_ST), ==, 1u << 3);
    g_assert(source(qts, SRC_MB_HP));
    g_assert(source(qts, SRC_MB_LP));

    /* The raw bits are read-only. */
    wr(qts, LP_INT_RAW, 0xffff);
    wr(qts, HP_INT_ST, 0xffff);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 1u << 3);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, 1u << 3);

    /* Clearing one side leaves the other. */
    wr(qts, HP_INT_CLR, 1u << 3);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, 0);
    g_assert(!source(qts, SRC_MB_HP));
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 1u << 3);
    g_assert(source(qts, SRC_MB_LP));
    wr(qts, LP_INT_CLR, 1u << 3);
    g_assert(!source(qts, SRC_MB_LP));
    /* The clear registers read 0. */
    g_assert_cmphex(rd(qts, LP_INT_CLR), ==, 0);

    /* Message 9's interrupts are off from reset: raw, but no interrupt. */
    wr(qts, MESSAGE(9), 0);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 1u << 9);
    g_assert_cmphex(rd(qts, LP_INT_ST), ==, 0);
    g_assert(!source(qts, SRC_MB_LP));
    wr(qts, LP_INT_ENA, 1u << 9);
    g_assert_cmphex(rd(qts, LP_INT_ENA), ==, 1u << 9);
    g_assert(source(qts, SRC_MB_LP));
    g_assert(!source(qts, SRC_MB_HP));
    wr(qts, HP_INT_ENA, 0xffffffff);
    g_assert_cmphex(rd(qts, HP_INT_ENA), ==, 0xffff);
    g_assert(source(qts, SRC_MB_HP));

    /* A byte write is a write: it raises the interrupt and merges. */
    wr(qts, LP_INT_CLR, 0xffff);
    wr(qts, HP_INT_CLR, 0xffff);
    qtest_writeb(qts, MB + MESSAGE(3) + 1, 0x12);
    g_assert_cmphex(rd(qts, MESSAGE(3)), ==, 0xdead12ef);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 1u << 3);

    wr(qts, REG_CLK_EN, 0xffffffff);
    g_assert_cmphex(rd(qts, REG_CLK_EN), ==, 1);
    qtest_quit(qts);
}

/*
 * LP_CLKRST_RST_EN_LP_MAILBOX holds the mailbox in reset: its registers
 * read their reset values and take no writes until released.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_block_reset(void)
{
    QTestState *qts = start();
    uint32_t rst = qtest_readl(qts, LP_CLKRST_RST_EN);

    wr(qts, MESSAGE(0), 0x1234);
    g_assert(source(qts, SRC_MB_HP));
    qtest_writel(qts, LP_CLKRST_RST_EN, rst | RST_EN_LP_MAILBOX);
    g_assert_cmphex(rd(qts, MESSAGE(0)), ==, 0);
    g_assert(!source(qts, SRC_MB_HP));
    wr(qts, MESSAGE(0), 0x5678);
    g_assert_cmphex(rd(qts, MESSAGE(0)), ==, 0);
    qtest_writel(qts, LP_CLKRST_RST_EN, rst);
    g_assert_cmphex(rd(qts, MESSAGE(0)), ==, 0);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, 0);
    wr(qts, MESSAGE(0), 0x5678);
    g_assert_cmphex(rd(qts, MESSAGE(0)), ==, 0x5678);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/lp-mailbox/reset", test_reset);
    qtest_add_func("/esp32p4/lp-mailbox/messages", test_messages);
    qtest_add_func("/esp32p4/lp-mailbox/block-reset", test_block_reset);
    return g_test_run();
}
