/*
 * QTest testcase for the RP2350 BUSCTRL block
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses do not pass through a core's bus port, so they never
 * count; the counting itself is exercised by the busctrl guest test.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define BUSCTRL_BASE     0x40068000
#define BUS_PRIORITY     (BUSCTRL_BASE + 0x00)
#define BUS_PRIORITY_ACK (BUSCTRL_BASE + 0x04)
#define PERFCTR_EN       (BUSCTRL_BASE + 0x08)
#define PERFCTR(n)       (BUSCTRL_BASE + 0x0c + 8 * (n))
#define PERFSEL(n)       (BUSCTRL_BASE + 0x10 + 8 * (n))
#define XOR              0x1000
#define SET              0x2000
#define CLR              0x3000

#define PROC0            (1u << 0)
#define PROC1            (1u << 4)
#define DMA_R            (1u << 8)
#define DMA_W            (1u << 12)

static char *rom_path;

/* A blank boot ROM is enough: qtest never runs the CPU. */
static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* [spec:nuos:req:emu.busctrl/test] */
static void test_reset_state(void)
{
    QTestState *qts = start();
    int n;

    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY_ACK), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PERFCTR_EN), ==, 0);
    for (n = 0; n < 4; n++) {
        g_assert_cmphex(qtest_readl(qts, PERFCTR(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, PERFSEL(n)), ==, 0x1f);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.busctrl/test] */
static void test_bus_priority(void)
{
    QTestState *qts = start();

    /* Only the four manager priority bits are implemented. */
    qtest_writel(qts, BUS_PRIORITY, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY), ==,
                    PROC0 | PROC1 | DMA_R | DMA_W);
    /* Every arbiter has registered the new levels once the write is done. */
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY_ACK), ==, 1);

    qtest_writel(qts, BUS_PRIORITY + CLR, PROC1 | DMA_W);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY), ==, PROC0 | DMA_R);
    qtest_writel(qts, BUS_PRIORITY + XOR, PROC0 | PROC1);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY), ==, PROC1 | DMA_R);
    qtest_writel(qts, BUS_PRIORITY + SET, DMA_W);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY), ==, PROC1 | DMA_R | DMA_W);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY + SET), ==,
                    PROC1 | DMA_R | DMA_W);

    qtest_writel(qts, BUS_PRIORITY_ACK, 0);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY_ACK), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.busctrl/test] */
static void test_perfctr_en_and_sel(void)
{
    QTestState *qts = start();
    int n;

    qtest_writel(qts, PERFCTR_EN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PERFCTR_EN), ==, 1);
    qtest_writel(qts, PERFCTR_EN + CLR, 1);
    g_assert_cmphex(qtest_readl(qts, PERFCTR_EN), ==, 0);
    qtest_writel(qts, PERFCTR_EN + XOR, 1);
    g_assert_cmphex(qtest_readl(qts, PERFCTR_EN), ==, 1);

    for (n = 0; n < 4; n++) {
        /* PERFSEL is seven bits wide. */
        qtest_writel(qts, PERFSEL(n), 0xffffff00 | (0x40 + n));
        g_assert_cmphex(qtest_readl(qts, PERFSEL(n)), ==, 0x40 + n);
        qtest_writel(qts, PERFSEL(n) + CLR, 0x40);
        g_assert_cmphex(qtest_readl(qts, PERFSEL(n)), ==, n);
        qtest_writel(qts, PERFSEL(n) + SET, 0x30);
        g_assert_cmphex(qtest_readl(qts, PERFSEL(n)), ==, 0x30 + n);
        qtest_writel(qts, PERFSEL(n) + XOR, 0x13);
        g_assert_cmphex(qtest_readl(qts, PERFSEL(n)), ==, (0x30 + n) ^ 0x13);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.busctrl/test] */
static void test_perfctr_write_clears(void)
{
    QTestState *qts = start();
    int n;

    /*
     * The counters only count up from zero: no write sets a value, through
     * the plain register or any alias.
     */
    for (n = 0; n < 4; n++) {
        qtest_writel(qts, PERFCTR(n), 0x123456);
        g_assert_cmphex(qtest_readl(qts, PERFCTR(n)), ==, 0);
        qtest_writel(qts, PERFCTR(n) + SET, 0xffffff);
        g_assert_cmphex(qtest_readl(qts, PERFCTR(n)), ==, 0);
        qtest_writel(qts, PERFCTR(n) + XOR, 0xffffff);
        g_assert_cmphex(qtest_readl(qts, PERFCTR(n)), ==, 0);
    }
    qtest_quit(qts);
}

/* A system reset restores every register, including the selections. */
/* [spec:nuos:req:emu.busctrl/test] */
static void test_system_reset(void)
{
    QTestState *qts = start();

    qtest_writel(qts, BUS_PRIORITY, PROC0);
    qtest_writel(qts, PERFCTR_EN, 1);
    qtest_writel(qts, PERFSEL(2), 0x0b);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BUS_PRIORITY_ACK), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PERFCTR_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PERFSEL(2)), ==, 0x1f);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-busctrl-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/busctrl/reset-state", test_reset_state);
    qtest_add_func("/rp2350/busctrl/bus-priority", test_bus_priority);
    qtest_add_func("/rp2350/busctrl/perfctr-en-sel", test_perfctr_en_and_sel);
    qtest_add_func("/rp2350/busctrl/perfctr-clear", test_perfctr_write_clears);
    qtest_add_func("/rp2350/busctrl/system-reset", test_system_reset);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
