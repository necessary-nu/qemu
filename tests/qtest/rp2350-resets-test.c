/*
 * QTest testcase for the RP2350 RESETS block
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RESETS_BASE 0x40020000
#define RESET       (RESETS_BASE + 0x0)
#define WDSEL       (RESETS_BASE + 0x4)
#define RESET_DONE  (RESETS_BASE + 0x8)
#define XOR         0x1000
#define SET         0x2000
#define CLR         0x3000

#define ALL         0x1fffffff
#define UART0       (1u << 26)
#define TIMER0      (1u << 23)
#define IO_BANK0    (1u << 6)

static char *rom_path;

/* A blank boot ROM is enough: qtest never runs the CPU. */
static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_reset_state(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL);
    g_assert_cmphex(qtest_readl(qts, WDSEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_reset_done_tracks_reset(void)
{
    QTestState *qts = start();

    /* The pico-sdk unreset_block_wait() sequence: clear, then poll. */
    qtest_writel(qts, RESET + CLR, UART0 | IO_BANK0);
    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL & ~(UART0 | IO_BANK0));
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, UART0 | IO_BANK0);

    qtest_writel(qts, RESET + SET, UART0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, IO_BANK0);

    qtest_writel(qts, RESET + XOR, TIMER0 | IO_BANK0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, TIMER0);

    qtest_writel(qts, RESET, 0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, ALL);

    /* Writes outside the implemented bits are ignored. */
    qtest_writel(qts, RESET, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL);

    /* RESET_DONE is read-only. */
    qtest_writel(qts, RESET_DONE, ALL);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);

    /* Aliases read the register itself. */
    g_assert_cmphex(qtest_readl(qts, RESET + SET), ==, ALL);
    qtest_quit(qts);
}

static void test_wdsel(void)
{
    QTestState *qts = start();

    qtest_writel(qts, WDSEL, UART0);
    qtest_writel(qts, WDSEL + SET, TIMER0);
    g_assert_cmphex(qtest_readl(qts, WDSEL), ==, UART0 | TIMER0);
    qtest_writel(qts, WDSEL + CLR, UART0);
    g_assert_cmphex(qtest_readl(qts, WDSEL), ==, TIMER0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-resets-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/resets/reset-state", test_reset_state);
    qtest_add_func("/rp2350/resets/reset-done", test_reset_done_tracks_reset);
    qtest_add_func("/rp2350/resets/wdsel", test_wdsel);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
