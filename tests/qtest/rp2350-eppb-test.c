/*
 * QTest testcase for the RP2350 Cortex-M33 EPPB registers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses go through system memory, which holds core 0's EPPB, so
 * register tests act on core 0. The NMI tests watch a core's EPPB "nmi"
 * output, which drives that core's NMI input.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define EPPB        0xe0080000
#define NMI_MASK0   (EPPB + 0x0)
#define NMI_MASK1   (EPPB + 0x4)
#define SLEEPCTRL   (EPPB + 0x8)
#define XOR         0x1000
#define SET         0x2000
#define CLR         0x3000

#define LIGHT_SLEEP 0x1
#define WICENREQ    0x2
#define WICENACK    0x4

#define TIMER0      0x400b0000
#define TIMER_INTE  0x40
#define TIMER_INTF  0x44

#define TIMER0_IRQ_0 0
#define UART0_IRQ    33
#define SPARE_IRQ_5  51

#define US          1000

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

static const char *eppb_path(int core)
{
    return core ? "/machine/soc/eppb[1]" : "/machine/soc/eppb[0]";
}

static QTestState *start_watching_nmi(int core)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, eppb_path(core), "nmi");
    return qts;
}

static void set_irq(QTestState *qts, int core, int n, int level)
{
    qtest_set_irq_in(qts, eppb_path(core), NULL, n, level);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, NMI_MASK0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, WICENREQ);

    /* The interrupt controller acknowledges WICENREQ's reset value. */
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, WICENREQ | WICENACK);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_nmi_mask_registers(void)
{
    QTestState *qts = start();

    qtest_writel(qts, NMI_MASK0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK0), ==, 0xffffffff);
    /* NMI_MASK1 covers IRQs 32 to 51 only. */
    qtest_writel(qts, NMI_MASK1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK1), ==, 0x000fffff);

    /* The EPPB has the RP2350 atomic aliases. */
    qtest_writel(qts, NMI_MASK0 + CLR, 0xffff0000);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK0), ==, 0x0000ffff);
    qtest_writel(qts, NMI_MASK0 + XOR, 0x000000ff);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK0), ==, 0x0000ff00);
    qtest_writel(qts, NMI_MASK1 + CLR, 0xffffffff);
    qtest_writel(qts, NMI_MASK1 + SET, 0xfff00001);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK1), ==, 0x00000001);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK1 + XOR), ==, 0x00000001);

    /* Unimplemented offsets read as zero and ignore writes. */
    qtest_writel(qts, EPPB + 0xc, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, EPPB + 0xc), ==, 0);

    /* A warm reset clears the masks. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NMI_MASK1), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_sleepctrl(void)
{
    QTestState *qts = start();

    qtest_clock_step(qts, 1 * US);

    /* WICENACK is read-only and follows WICENREQ after a delay. */
    qtest_writel(qts, SLEEPCTRL, LIGHT_SLEEP);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, LIGHT_SLEEP | WICENACK);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, LIGHT_SLEEP);

    qtest_writel(qts, SLEEPCTRL + SET, WICENREQ | WICENACK);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, LIGHT_SLEEP | WICENREQ);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==,
                    LIGHT_SLEEP | WICENREQ | WICENACK);

    qtest_writel(qts, SLEEPCTRL + CLR, LIGHT_SLEEP);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, WICENREQ | WICENACK);
    qtest_writel(qts, SLEEPCTRL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==,
                    LIGHT_SLEEP | WICENREQ | WICENACK);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, SLEEPCTRL), ==, WICENREQ);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_nmi_mask(void)
{
    QTestState *qts = start_watching_nmi(0);

    /* An unmasked IRQ does not raise NMI. */
    set_irq(qts, 0, TIMER0_IRQ_0, 1);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Masking an asserted IRQ raises NMI; unmasking drops it. */
    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, NMI_MASK0, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* NMI follows a masked IRQ's level. */
    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    set_irq(qts, 0, TIMER0_IRQ_0, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    set_irq(qts, 0, TIMER0_IRQ_0, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    set_irq(qts, 0, TIMER0_IRQ_0, 0);

    /* NMI_MASK1 bit n masks IRQ 32 + n. */
    qtest_writel(qts, NMI_MASK1, 1u << (UART0_IRQ - 32));
    set_irq(qts, 0, SPARE_IRQ_5, 1);
    g_assert_false(qtest_get_irq(qts, 0));
    set_irq(qts, 0, UART0_IRQ, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, NMI_MASK1 + SET, 1u << (SPARE_IRQ_5 - 32));
    set_irq(qts, 0, UART0_IRQ, 0);
    g_assert_true(qtest_get_irq(qts, 0));
    set_irq(qts, 0, SPARE_IRQ_5, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_nmi_with_rcp_fault(void)
{
    QTestState *qts = start_watching_nmi(0);

    /* The RCP's NMI and masked IRQs combine: either one holds NMI. */
    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    qtest_set_irq_in(qts, eppb_path(0), "rcp-nmi", 0, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    set_irq(qts, 0, TIMER0_IRQ_0, 1);
    set_irq(qts, 0, TIMER0_IRQ_0, 0);
    g_assert_true(qtest_get_irq(qts, 0));

    /* The NMI mask does not gate the RCP's NMI. */
    qtest_writel(qts, NMI_MASK0, 0);
    g_assert_true(qtest_get_irq(qts, 0));

    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    set_irq(qts, 0, TIMER0_IRQ_0, 1);
    qtest_set_irq_in(qts, eppb_path(0), "rcp-nmi", 0, 0);
    g_assert_true(qtest_get_irq(qts, 0));
    set_irq(qts, 0, TIMER0_IRQ_0, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_nmi_is_core_local(void)
{
    QTestState *qts;

    /* Core 0's mask routes the timer IRQ to core 0's NMI... */
    qts = start_watching_nmi(0);
    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    qtest_writel(qts, TIMER0 + TIMER_INTE, 1);
    qtest_writel(qts, TIMER0 + TIMER_INTF, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, TIMER0 + TIMER_INTF, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);

    /* ...and not to core 1's, whose own mask is clear. */
    qts = start_watching_nmi(1);
    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    qtest_writel(qts, TIMER0 + TIMER_INTE, 1);
    qtest_writel(qts, TIMER0 + TIMER_INTF, 1);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.eppb/test] */
static void test_masked_irq_still_reaches_nvic(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    qtest_writel(qts, NMI_MASK0, 1u << TIMER0_IRQ_0);
    qtest_writel(qts, TIMER0 + TIMER_INTE, 1);
    qtest_writel(qts, TIMER0 + TIMER_INTF, 1);
    g_assert_true(qtest_get_irq(qts, TIMER0_IRQ_0));
    qtest_writel(qts, TIMER0 + TIMER_INTF, 0);
    g_assert_false(qtest_get_irq(qts, TIMER0_IRQ_0));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-eppb-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/eppb/reset-values", test_reset_values);
    qtest_add_func("/rp2350/eppb/nmi-mask-registers",
                   test_nmi_mask_registers);
    qtest_add_func("/rp2350/eppb/sleepctrl", test_sleepctrl);
    qtest_add_func("/rp2350/eppb/nmi-mask", test_nmi_mask);
    qtest_add_func("/rp2350/eppb/nmi-with-rcp-fault", test_nmi_with_rcp_fault);
    qtest_add_func("/rp2350/eppb/nmi-is-core-local", test_nmi_is_core_local);
    qtest_add_func("/rp2350/eppb/masked-irq-still-reaches-nvic",
                   test_masked_irq_still_reaches_nvic);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
