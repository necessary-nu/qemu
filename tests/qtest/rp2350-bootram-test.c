/*
 * QTest testcase for the RP2350 boot RAM
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define BOOTRAM       0x400e0000
#define WRITE_ONCE0   (BOOTRAM + 0x800)
#define WRITE_ONCE1   (BOOTRAM + 0x804)
#define BOOTLOCK_STAT (BOOTRAM + 0x808)
#define BOOTLOCK(n)   (BOOTRAM + 0x80c + 4 * (n))
#define XOR           0x1000
#define SET           0x2000
#define CLR           0x3000

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* [spec:nuos:req:emu.bootram/test] */
static void test_ram(void)
{
    QTestState *qts = start();

    qtest_writel(qts, BOOTRAM, 0x11223344);
    qtest_writel(qts, BOOTRAM + 0x3fc, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, BOOTRAM), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, BOOTRAM + 0x3fc), ==, 0xcafef00d);
    g_assert_cmphex(qtest_readb(qts, BOOTRAM + 1), ==, 0x33);

    qtest_writeb(qts, BOOTRAM + 2, 0xaa);
    g_assert_cmphex(qtest_readl(qts, BOOTRAM), ==, 0x11aa3344);

    /* The atomic aliases work on the RAM too. */
    qtest_writel(qts, BOOTRAM + SET, 0x0000000f);
    qtest_writel(qts, BOOTRAM + CLR, 0x11000000);
    qtest_writel(qts, BOOTRAM + XOR, 0x00000100);
    g_assert_cmphex(qtest_readl(qts, BOOTRAM), ==, 0x00aa324f);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.bootram/test] */
static void test_write_once(void)
{
    QTestState *qts = start();

    qtest_writel(qts, WRITE_ONCE0, 0x5);
    qtest_writel(qts, WRITE_ONCE0, 0x2);
    g_assert_cmphex(qtest_readl(qts, WRITE_ONCE0), ==, 0x7);
    qtest_writel(qts, WRITE_ONCE0 + CLR, 0x7);
    g_assert_cmphex(qtest_readl(qts, WRITE_ONCE0), ==, 0x7);
    g_assert_cmphex(qtest_readl(qts, WRITE_ONCE1), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.bootram/test] */
static void test_boot_locks(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, BOOTLOCK_STAT), ==, 0xff);
    g_assert_cmphex(qtest_readl(qts, BOOTLOCK(7)), ==, 1u << 7);
    g_assert_cmphex(qtest_readl(qts, BOOTLOCK(7)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BOOTLOCK_STAT), ==, 0x7f);

    qtest_writel(qts, BOOTLOCK(7), 0);
    g_assert_cmphex(qtest_readl(qts, BOOTLOCK_STAT), ==, 0xff);

    qtest_writel(qts, BOOTLOCK_STAT, 0x0f);
    g_assert_cmphex(qtest_readl(qts, BOOTLOCK(5)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BOOTLOCK(2)), ==, 1u << 2);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-bootram-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/bootram/ram", test_ram);
    qtest_add_func("/rp2350/bootram/write-once", test_write_once);
    qtest_add_func("/rp2350/bootram/boot-locks", test_boot_locks);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
