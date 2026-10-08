/*
 * QTest testcase for the RP2350 SIO block
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses go through system memory, which holds core 0's view of
 * SIO, so these tests act as core 0 and observe core 1 through its
 * interrupts.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SIO         0xd0000000
#define SIO_NS      0xd0020000
#define CPUID       0x000
#define GPIO_IN     0x004
#define GPIO_OUT    0x010
#define GPIO_OUT_SET 0x018
#define GPIO_OUT_CLR 0x020
#define GPIO_OUT_XOR 0x028
#define GPIO_OE     0x030
#define GPIO_OE_SET 0x038
#define FIFO_ST     0x050
#define FIFO_WR     0x054
#define FIFO_RD     0x058
#define SPINLOCK_ST 0x05c
#define SPINLOCK(n) (0x100 + 4 * (n))
#define DOORBELL_OUT_SET 0x180
#define DOORBELL_OUT_CLR 0x184
#define DOORBELL_IN_SET  0x188
#define DOORBELL_IN_CLR  0x18c

#define VLD 0x1
#define RDY 0x2
#define WOF 0x4
#define ROE 0x8

#define IRQ_FIFO    25
#define IRQ_BELL    26
#define IRQ_FIFO_NS 27
#define IRQ_BELL_NS 28

static char *rom_path;

static QTestState *start(int intercept_core)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    if (intercept_core >= 0) {
        g_autofree char *cpu = g_strdup_printf("/machine/soc/armv7m[%d]",
                                               intercept_core);

        qtest_irq_intercept_in(qts, cpu);
    }
    return qts;
}

/* [spec:nuos:req:emu.sio/test] */
static void test_cpuid(void)
{
    QTestState *qts = start(-1);

    g_assert_cmphex(qtest_readl(qts, SIO + CPUID), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sio/test] */
static void test_fifo_status(void)
{
    QTestState *qts = start(0);
    int i;

    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_ST), ==, RDY);
    g_assert_false(qtest_get_irq(qts, IRQ_FIFO));

    for (i = 0; i < 4; i++) {
        qtest_writel(qts, SIO + FIFO_WR, 0x100 + i);
    }
    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_ST), ==, 0);
    g_assert_false(qtest_get_irq(qts, IRQ_FIFO));

    qtest_writel(qts, SIO + FIFO_WR, 0xdead);
    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_ST), ==, WOF);
    g_assert_true(qtest_get_irq(qts, IRQ_FIFO));

    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_RD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_ST), ==, WOF | ROE);

    qtest_writel(qts, SIO + FIFO_ST, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_ST), ==, 0);
    g_assert_false(qtest_get_irq(qts, IRQ_FIFO));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sio/test] */
/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_fifo_irq_is_core_local(void)
{
    QTestState *qts = start(1);

    /* Core 0 writing raises core 1's FIFO interrupt (VLD), only there. */
    qtest_writel(qts, SIO + FIFO_WR, 42);
    g_assert_true(qtest_get_irq(qts, IRQ_FIFO));
    g_assert_false(qtest_get_irq(qts, IRQ_FIFO_NS));
    qtest_quit(qts);

    qts = start(0);
    qtest_writel(qts, SIO + FIFO_WR, 42);
    g_assert_false(qtest_get_irq(qts, IRQ_FIFO));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sio/test] */
static void test_spinlocks(void)
{
    QTestState *qts = start(-1);

    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK(5)), ==, 1u << 5);
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK(5)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK(31)), ==, 1u << 31);
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK_ST), ==,
                    (1u << 5) | (1u << 31));

    qtest_writel(qts, SIO + SPINLOCK(5), 0);
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK_ST), ==, 1u << 31);
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK(5)), ==, 1u << 5);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sio/test] */
/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_doorbells(void)
{
    QTestState *qts = start(0);

    /* Ringing our own doorbell. */
    qtest_writel(qts, SIO + DOORBELL_IN_SET, 0x103);
    g_assert_true(qtest_get_irq(qts, IRQ_BELL));
    g_assert_cmphex(qtest_readl(qts, SIO + DOORBELL_IN_CLR), ==, 0x03);
    qtest_writel(qts, SIO + DOORBELL_IN_CLR, 0x01);
    g_assert_true(qtest_get_irq(qts, IRQ_BELL));
    qtest_writel(qts, SIO + DOORBELL_IN_CLR, 0x02);
    g_assert_false(qtest_get_irq(qts, IRQ_BELL));

    /* Ringing core 1 does not interrupt core 0. */
    qtest_writel(qts, SIO + DOORBELL_OUT_SET, 0x80);
    g_assert_false(qtest_get_irq(qts, IRQ_BELL));
    g_assert_cmphex(qtest_readl(qts, SIO + DOORBELL_OUT_CLR), ==, 0x80);
    qtest_quit(qts);

    qts = start(1);
    qtest_writel(qts, SIO + DOORBELL_OUT_SET, 0x80);
    g_assert_true(qtest_get_irq(qts, IRQ_BELL));
    qtest_writel(qts, SIO + DOORBELL_OUT_CLR, 0x80);
    g_assert_false(qtest_get_irq(qts, IRQ_BELL));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sio/test] */
static void test_nonsecure_bank(void)
{
    QTestState *qts = start(0);

    /* Spinlocks, FIFOs and doorbells are separate per bank. */
    g_assert_cmphex(qtest_readl(qts, SIO + SPINLOCK(3)), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, SIO_NS + SPINLOCK(3)), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, SIO_NS + SPINLOCK_ST), ==, 1u << 3);

    qtest_writel(qts, SIO_NS + FIFO_WR, 1);
    g_assert_cmphex(qtest_readl(qts, SIO_NS + FIFO_ST), ==, RDY);
    g_assert_cmphex(qtest_readl(qts, SIO + FIFO_ST), ==, RDY);

    qtest_writel(qts, SIO_NS + DOORBELL_IN_SET, 1);
    g_assert_true(qtest_get_irq(qts, IRQ_BELL_NS));
    g_assert_false(qtest_get_irq(qts, IRQ_BELL));
    g_assert_cmphex(qtest_readl(qts, SIO + DOORBELL_IN_SET), ==, 0);
    qtest_quit(qts);
}

static void test_gpio(void)
{
    QTestState *qts = start(-1);

    qtest_writel(qts, SIO + GPIO_OUT, 0x0f);
    qtest_writel(qts, SIO + GPIO_OUT_SET, 0x30);
    qtest_writel(qts, SIO + GPIO_OUT_CLR, 0x01);
    qtest_writel(qts, SIO + GPIO_OUT_XOR, 0x03);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_OUT), ==, 0x3d);

    qtest_writel(qts, SIO + GPIO_OE_SET, 0x0c);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_OE), ==, 0x0c);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_IN), ==, 0x0c);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-sio-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/sio/cpuid", test_cpuid);
    qtest_add_func("/rp2350/sio/fifo-status", test_fifo_status);
    qtest_add_func("/rp2350/sio/fifo-irq", test_fifo_irq_is_core_local);
    qtest_add_func("/rp2350/sio/spinlocks", test_spinlocks);
    qtest_add_func("/rp2350/sio/doorbells", test_doorbells);
    qtest_add_func("/rp2350/sio/nonsecure-bank", test_nonsecure_bank);
    qtest_add_func("/rp2350/sio/gpio", test_gpio);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
