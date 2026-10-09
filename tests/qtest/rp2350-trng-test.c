/*
 * QTest testcase for the RP2350 true random number generator (TRNG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SET                 0x2000

#define TRNG                0x400f0000
#define RNG_IMR             (TRNG + 0x100)
#define RNG_ISR             (TRNG + 0x104)
#define RNG_ICR             (TRNG + 0x108)
#define TRNG_CONFIG         (TRNG + 0x10c)
#define TRNG_VALID          (TRNG + 0x110)
#define EHR_DATA(n)         (TRNG + 0x114 + 4 * (n))
#define RND_SOURCE_ENABLE   (TRNG + 0x12c)
#define SAMPLE_CNT1         (TRNG + 0x130)
#define TRNG_DEBUG_CONTROL  (TRNG + 0x138)
#define TRNG_SW_RESET       (TRNG + 0x140)
#define TRNG_BUSY           (TRNG + 0x1b8)
#define RST_BITS_COUNTER    (TRNG + 0x1bc)

#define EHR_VALID           (1u << 0)
#define BYPASS_ALL          0xeu

#define TRNG_IRQ            39

/*
 * clk_sys is 150 MHz: with the checks bypassed and a sample every cycle,
 * the 192 samples of a collection take 1280 ns.
 */
#define BYPASS_COLLECT_NS   1280

static char *rom_path;

static QTestState *start(const char *seed)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s -seed %s", rom_path,
                                  seed);

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    return qts;
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_reset(void)
{
    QTestState *qts = start("1");

    g_assert_cmphex(qtest_readl(qts, RNG_IMR), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, RNG_ISR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TRNG_CONFIG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RND_SOURCE_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SAMPLE_CNT1), ==, 0xffff);
    g_assert_cmphex(qtest_readl(qts, TRNG_BUSY), ==, 0);
    for (int i = 0; i < 6; i++) {
        g_assert_cmphex(qtest_readl(qts, EHR_DATA(i)), ==, 0);
    }
    /* Nothing is collected until the source is enabled. */
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    qtest_quit(qts);
}

static void read_ehr(QTestState *qts, uint32_t *ehr)
{
    for (int i = 0; i < 6; i++) {
        ehr[i] = qtest_readl(qts, EHR_DATA(i));
    }
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_collect(void)
{
    QTestState *qts = start("1");
    uint32_t ehr[6], again[6];

    /* pico_rand's raw-sample setup. */
    qtest_writel(qts, SAMPLE_CNT1, 0);
    qtest_writel(qts, TRNG_DEBUG_CONTROL, BYPASS_ALL);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_writel(qts, RNG_ICR, 0xf);
    g_assert_cmphex(qtest_readl(qts, TRNG_BUSY), ==, 1);

    /* Collection takes its 192 samples' time. */
    qtest_clock_step(qts, BYPASS_COLLECT_NS - 1);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    g_assert_cmphex(qtest_readl(qts, EHR_DATA(0)), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, TRNG_BUSY), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 1);
    g_assert_cmphex(qtest_readl(qts, RNG_ISR), ==, EHR_VALID);
    /* EHR_VALID is masked at reset. */
    g_assert_false(qtest_get_irq(qts, TRNG_IRQ));

    /* The result holds until EHR_DATA5 is read. */
    qtest_clock_step(qts, 100 * BYPASS_COLLECT_NS);
    read_ehr(qts, ehr);
    g_assert_true(ehr[0] | ehr[1] | ehr[2] | ehr[3] | ehr[4] | ehr[5]);

    /* Reading EHR_DATA5 emptied the EHR and started a new collection. */
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TRNG_BUSY), ==, 1);
    g_assert_cmphex(qtest_readl(qts, EHR_DATA(0)), ==, 0);
    /* The ISR bit stays until cleared through RNG_ICR. */
    g_assert_cmphex(qtest_readl(qts, RNG_ISR), ==, EHR_VALID);
    qtest_writel(qts, RNG_ICR, EHR_VALID);
    g_assert_cmphex(qtest_readl(qts, RNG_ISR), ==, 0);

    qtest_clock_step(qts, BYPASS_COLLECT_NS);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 1);
    read_ehr(qts, again);
    g_assert_false(memcmp(ehr, again, sizeof(ehr)) == 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_interrupt(void)
{
    QTestState *qts = start("2");
    uint32_t ehr[6];

    qtest_writel(qts, SAMPLE_CNT1, 0);
    qtest_writel(qts, TRNG_DEBUG_CONTROL, BYPASS_ALL);
    qtest_writel(qts, RNG_IMR, 0xf & ~EHR_VALID);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_clock_step(qts, BYPASS_COLLECT_NS - 1);
    g_assert_false(qtest_get_irq(qts, TRNG_IRQ));
    qtest_clock_step(qts, 1);
    g_assert_true(qtest_get_irq(qts, TRNG_IRQ));

    /* Masking it drops the line; unmasking raises it again. */
    qtest_writel(qts, RNG_IMR + SET, EHR_VALID);
    g_assert_false(qtest_get_irq(qts, TRNG_IRQ));
    qtest_writel(qts, RNG_IMR, 0xe);
    g_assert_true(qtest_get_irq(qts, TRNG_IRQ));

    /* Reading the data does not clear it; RNG_ICR does. */
    read_ehr(qts, ehr);
    g_assert_true(qtest_get_irq(qts, TRNG_IRQ));
    qtest_writel(qts, RNG_ICR, EHR_VALID);
    g_assert_false(qtest_get_irq(qts, TRNG_IRQ));

    /* The collection EHR_DATA5 started raises it again. */
    qtest_clock_step(qts, BYPASS_COLLECT_NS);
    g_assert_true(qtest_get_irq(qts, TRNG_IRQ));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_sample_period(void)
{
    QTestState *qts = start("3");
    uint64_t ns = 0;

    /*
     * With the von Neumann decorrelator in, each collected bit costs at
     * least two samples: at 100 cycles a sample, at least 256 us.
     */
    qtest_writel(qts, SAMPLE_CNT1, 100);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_clock_step(qts, 256 * 1000 - 1);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    while (!qtest_readl(qts, TRNG_VALID)) {
        qtest_clock_step(qts, 10 * 1000);
        ns += 10 * 1000;
        g_assert_cmpuint(ns, <, 10 * 1000 * 1000);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_pause_and_reset(void)
{
    QTestState *qts = start("4");

    qtest_writel(qts, SAMPLE_CNT1, 0);
    qtest_writel(qts, TRNG_DEBUG_CONTROL, BYPASS_ALL);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_clock_step(qts, BYPASS_COLLECT_NS / 2);

    /* Disabling the source pauses collection where it is. */
    qtest_writel(qts, RND_SOURCE_ENABLE, 0);
    g_assert_cmphex(qtest_readl(qts, TRNG_BUSY), ==, 0);
    qtest_clock_step(qts, 10 * BYPASS_COLLECT_NS);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_clock_step(qts, BYPASS_COLLECT_NS / 2);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 1);

    /* RST_BITS_COUNTER is ignored while the source is enabled... */
    qtest_writel(qts, RST_BITS_COUNTER, 1);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 1);
    /* ...and otherwise discards the result. */
    qtest_writel(qts, RND_SOURCE_ENABLE, 0);
    qtest_writel(qts, RST_BITS_COUNTER, 1);
    g_assert_cmphex(qtest_readl(qts, TRNG_VALID), ==, 0);
    g_assert_cmphex(qtest_readl(qts, EHR_DATA(5)), ==, 0);

    /* A software reset returns every register to its reset value. */
    qtest_writel(qts, RNG_IMR, 0);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_writel(qts, TRNG_SW_RESET, 1);
    g_assert_cmphex(qtest_readl(qts, RNG_IMR), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, RND_SOURCE_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SAMPLE_CNT1), ==, 0xffff);
    g_assert_cmphex(qtest_readl(qts, TRNG_DEBUG_CONTROL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RNG_ISR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TRNG_BUSY), ==, 0);
    qtest_quit(qts);
}

static void collect_once(const char *seed, uint32_t *ehr)
{
    QTestState *qts = start(seed);

    qtest_writel(qts, SAMPLE_CNT1, 0);
    qtest_writel(qts, TRNG_DEBUG_CONTROL, BYPASS_ALL);
    qtest_writel(qts, RND_SOURCE_ENABLE, 1);
    qtest_clock_step(qts, BYPASS_COLLECT_NS);
    read_ehr(qts, ehr);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.rosc-trng/test] */
static void test_seed(void)
{
    uint32_t a[6], b[6], c[6];

    /* The entropy comes from the host's random source, so -seed fixes it. */
    collect_once("42", a);
    collect_once("42", b);
    collect_once("43", c);
    g_assert_true(memcmp(a, b, sizeof(a)) == 0);
    g_assert_false(memcmp(a, c, sizeof(a)) == 0);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-trng-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/trng/reset", test_reset);
    qtest_add_func("/rp2350/trng/collect", test_collect);
    qtest_add_func("/rp2350/trng/interrupt", test_interrupt);
    qtest_add_func("/rp2350/trng/sample-period", test_sample_period);
    qtest_add_func("/rp2350/trng/pause-and-reset", test_pause_and_reset);
    qtest_add_func("/rp2350/trng/seed", test_seed);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
