/*
 * QTest testcase for the RP2350 ring oscillator (ROSC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SET         0x2000
#define CLR         0x3000

#define ROSC        0x400e8000
#define CTRL        (ROSC + 0x00)
#define FREQA       (ROSC + 0x04)
#define FREQB       (ROSC + 0x08)
#define RANDOM      (ROSC + 0x0c)
#define DORMANT     (ROSC + 0x10)
#define DIV         (ROSC + 0x14)
#define PHASE       (ROSC + 0x18)
#define STATUS      (ROSC + 0x1c)
#define RANDOMBIT   (ROSC + 0x20)
#define COUNT       (ROSC + 0x24)

#define ENABLE      (0xfabu << 12)
#define DISABLE     (0xd1eu << 12)
#define RANGE_LOW   0xfa4u
#define RANGE_MEDIUM 0xfa5u
#define RANGE_HIGH  0xfa7u
#define PASSWD      (0x9696u << 16)

#define ST_ENABLED      (1u << 12)
#define ST_DIV_RUNNING  (1u << 16)
#define ST_BADWRITE     (1u << 24)
#define ST_STABLE       (1u << 31)

#define US          1000

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s -seed 1", rom_path);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CTRL), ==, ENABLE | 0xaa0);
    g_assert_cmphex(qtest_readl(qts, FREQA), ==, 0x88);
    g_assert_cmphex(qtest_readl(qts, FREQB), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RANDOM), ==, 0x3f04b16d);
    g_assert_cmphex(qtest_readl(qts, DORMANT), ==, 0x77616b65);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0xaa08);
    g_assert_cmphex(qtest_readl(qts, PHASE), ==, 0x8);
    g_assert_cmphex(qtest_readl(qts, STATUS), ==,
                    ST_STABLE | ST_DIV_RUNNING | ST_ENABLED);
    g_assert_cmphex(qtest_readl(qts, COUNT), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_badwrite(void)
{
    QTestState *qts = start();

    /* The valid codes are accepted. */
    qtest_writel(qts, CTRL, ENABLE | RANGE_LOW);
    qtest_writel(qts, FREQA, PASSWD | 0x88);
    qtest_writel(qts, FREQB, PASSWD);
    qtest_writel(qts, DIV, 0xaa04);
    qtest_writel(qts, PHASE, 0xaa8);
    qtest_writel(qts, DORMANT, 0x77616b65);
    g_assert_cmphex(qtest_readl(qts, STATUS) & ST_BADWRITE, ==, 0);

    /* An invalid ENABLE code is flagged but still enables. */
    qtest_writel(qts, CTRL, (0x123u << 12) | RANGE_LOW);
    g_assert_cmphex(qtest_readl(qts, STATUS) & (ST_BADWRITE | ST_ENABLED),
                    ==, ST_BADWRITE | ST_ENABLED);

    /* BADWRITE is write-1-to-clear, also through the CLR alias. */
    qtest_writel(qts, STATUS, 0);
    g_assert_cmphex(qtest_readl(qts, STATUS) & ST_BADWRITE, ==, ST_BADWRITE);
    qtest_writel(qts, STATUS, ST_BADWRITE);
    g_assert_cmphex(qtest_readl(qts, STATUS) & ST_BADWRITE, ==, 0);

    /* So is a FREQ_RANGE code outside the four ranges. */
    qtest_writel(qts, CTRL, ENABLE | 0x123);
    g_assert_cmphex(qtest_readl(qts, STATUS) & ST_BADWRITE, ==, ST_BADWRITE);
    qtest_writel(qts, STATUS + CLR, ST_BADWRITE);
    g_assert_cmphex(qtest_readl(qts, STATUS) & ST_BADWRITE, ==, 0);

    /* FREQA, FREQB, DIV and PHASE need their passwords. */
    static const struct {
        uint32_t addr, value;
    } bad[] = {
        { FREQA, 0x1234 },
        { FREQB, 0x96950000 },
        { DIV, 0xab04 },
        { PHASE, 0xab8 },
        { DORMANT, 0x12345678 },
    };
    for (int i = 0; i < ARRAY_SIZE(bad); i++) {
        qtest_writel(qts, bad[i].addr, bad[i].value);
        g_assert_cmphex(qtest_readl(qts, STATUS) & ST_BADWRITE, ==,
                        ST_BADWRITE);
        qtest_writel(qts, STATUS, ST_BADWRITE);
    }
    /* An invalid DORMANT write selects WAKE. */
    g_assert_cmphex(qtest_readl(qts, DORMANT), ==, 0x77616b65);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_enable(void)
{
    QTestState *qts = start();
    uint32_t bit;

    /* rosc_disable(): write DISABLE, wait for STABLE to clear. */
    qtest_writel(qts, CTRL, DISABLE | RANGE_LOW);
    g_assert_cmphex(qtest_readl(qts, STATUS) &
                    (ST_STABLE | ST_ENABLED | ST_DIV_RUNNING), ==, 0);

    /* A stopped oscillator's RANDOMBIT holds its level. */
    bit = qtest_readl(qts, RANDOMBIT);
    for (int i = 0; i < 32; i++) {
        g_assert_cmphex(qtest_readl(qts, RANDOMBIT), ==, bit);
    }

    /* rosc_restart(): enabled at once, stable after its ~1us start-up. */
    qtest_writel(qts, CTRL, ENABLE);
    g_assert_cmphex(qtest_readl(qts, STATUS) & (ST_STABLE | ST_ENABLED), ==,
                    ST_ENABLED);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, STATUS) & (ST_STABLE | ST_ENABLED), ==,
                    ST_STABLE | ST_ENABLED);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_randombit(void)
{
    QTestState *qts = start();
    unsigned ones = 0;

    for (int i = 0; i < 256; i++) {
        uint32_t bit = qtest_readl(qts, RANDOMBIT);

        g_assert_cmphex(bit & ~1u, ==, 0);
        ones += bit;
    }
    g_assert_cmpuint(ones, >, 64);
    g_assert_cmpuint(ones, <, 192);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_count(void)
{
    QTestState *qts = start();

    /*
     * With no randomisation and default drive the LOW ring runs at
     * 88 MHz: 11 MHz after the reset divisor of 8, 11 counts per us.
     */
    qtest_writel(qts, CTRL, ENABLE | RANGE_LOW);
    qtest_writel(qts, FREQA, PASSWD);
    qtest_writel(qts, COUNT, 1100);
    qtest_clock_step(qts, 50 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), ==, 550);
    /* It stops at zero. */
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), ==, 0);

    /* Halving the divisor doubles the rate. */
    qtest_writel(qts, DIV, 0xaa04);
    qtest_writel(qts, COUNT, 2200);
    qtest_clock_step(qts, 50 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), ==, 1100);

    /* HIGH uses half the stages, so twice the frequency again. */
    qtest_writel(qts, CTRL, ENABLE | RANGE_MEDIUM);
    qtest_writel(qts, CTRL, ENABLE | RANGE_HIGH);
    qtest_writel(qts, COUNT, 4400);
    qtest_clock_step(qts, 25 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), ==, 3300);

    /* Extra drive strength shortens the stage delays: faster. */
    qtest_writel(qts, FREQA, PASSWD | 0x1111);
    qtest_writel(qts, COUNT, 0xffff);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), <, 0xffff - 440);
    /* A bad password zeroes the drive strengths again. */
    qtest_writel(qts, FREQA, 0x1111);
    qtest_writel(qts, STATUS, ST_BADWRITE);
    qtest_writel(qts, COUNT, 4400);
    qtest_clock_step(qts, 25 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), ==, 3300);

    /* A stopped oscillator stops the count. */
    qtest_writel(qts, CTRL, DISABLE | RANGE_HIGH);
    qtest_clock_step(qts, 25 * US);
    g_assert_cmpuint(qtest_readl(qts, COUNT), ==, 3300);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-rosc-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/rosc/reset", test_reset);
    qtest_add_func("/rp2350/rosc/badwrite", test_badwrite);
    qtest_add_func("/rp2350/rosc/enable", test_enable);
    qtest_add_func("/rp2350/rosc/randombit", test_randombit);
    qtest_add_func("/rp2350/rosc/count", test_count);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
