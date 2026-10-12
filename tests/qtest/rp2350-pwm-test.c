/*
 * QTest testcase for the RP2350 pulse width modulation (PWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * clk_sys is 150 MHz, so DIV_INT 150 makes a slice count once per
 * microsecond, on the microsecond. Pins are driven from outside through
 * the GPIO block's "pad-in" lines.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define ALIAS_XOR       0x1000
#define ALIAS_SET       0x2000
#define ALIAS_CLR       0x3000

#define PWM             0x400a8000
#define CSR(n)          (PWM + 0x14 * (n) + 0x00)
#define DIV(n)          (PWM + 0x14 * (n) + 0x04)
#define CTR(n)          (PWM + 0x14 * (n) + 0x08)
#define CC(n)           (PWM + 0x14 * (n) + 0x0c)
#define TOP(n)          (PWM + 0x14 * (n) + 0x10)
#define EN              (PWM + 0xf0)
#define INTR            (PWM + 0xf4)
#define IRQ0_INTE       (PWM + 0xf8)
#define IRQ0_INTF       (PWM + 0xfc)
#define IRQ0_INTS       (PWM + 0x100)
#define IRQ1_INTE       (PWM + 0x104)
#define IRQ1_INTF       (PWM + 0x108)
#define IRQ1_INTS       (PWM + 0x10c)

#define CSR_EN          (1u << 0)
#define CSR_PH_CORRECT  (1u << 1)
#define CSR_A_INV       (1u << 2)
#define CSR_B_INV       (1u << 3)
#define CSR_DIVMODE(m)  ((m) << 4)
#define CSR_PH_RET      (1u << 6)
#define CSR_PH_ADV      (1u << 7)
#define DIVMODE_LEVEL   1
#define DIVMODE_RISE    2
#define DIVMODE_FALL    3

#define DIV_INT(i)      ((i) << 4)
#define DIV_1US         DIV_INT(150)

#define IO_BANK0        0x40028000
#define PADS_BANK0      0x40038000
#define STATUS(p)       (IO_BANK0 + 8 * (p))
#define CTRL(p)         (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define STATUS_OUTTOPAD (1u << 9)
#define STATUS_OETOPAD  (1u << 13)
#define PAD_IE          (1u << 6)
#define FUNC_PWM        4
#define SIO_GPIO_IN     0xd0000004

#define PWM_IRQ_WRAP_0  8
#define PWM_IRQ_WRAP_1  9

#define US              1000

static char *rom_path;

/*
 * clk_sys's edges fall at whole multiples of its period from time 0; at
 * 150 MHz, every 20 ns is one. Step to the next, so that the tests' writes,
 * whole microseconds apart, land on edges and the counts they time start
 * with a whole cycle.
 */
static void align_to_clk_sys(QTestState *qts)
{
    int64_t now = qtest_clock_step(qts, 1);

    if (now % 20) {
        qtest_clock_step(qts, 20 - now % 20);
    }
}

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    align_to_clk_sys(qts);
    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    return qts;
}

/* As start(), with the pads' levels intercepted rather than the CPU's IRQs */
static QTestState *start_pads(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    align_to_clk_sys(qts);
    qtest_irq_intercept_out_named(qts, "/machine/soc/gpio", "pad-out");
    return qts;
}

static void step(QTestState *qts, int64_t ns)
{
    qtest_clock_step(qts, ns);
}

/* Route pin p to the PWM with its input buffer on. */
static void pin_pwm(QTestState *qts, int p)
{
    qtest_writel(qts, PAD(p), PAD_IE);
    qtest_writel(qts, CTRL(p), FUNC_PWM);
}

static bool pin_in(QTestState *qts, int p)
{
    return (qtest_readl(qts, SIO_GPIO_IN) >> p) & 1;
}

static void drive(QTestState *qts, int p, int level)
{
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", p, level);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int n;

    for (n = 0; n < 12; n++) {
        g_assert_cmphex(qtest_readl(qts, CSR(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, DIV(n)), ==, 0x10);
        g_assert_cmphex(qtest_readl(qts, CTR(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, CC(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, TOP(n)), ==, 0xffff);
    }
    g_assert_cmphex(qtest_readl(qts, EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQ0_INTE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQ0_INTF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQ0_INTS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQ1_INTE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQ1_INTF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQ1_INTS), ==, 0);

    /* Reserved bits read zero. */
    qtest_writel(qts, CSR(3), 0xffffff3e);
    g_assert_cmphex(qtest_readl(qts, CSR(3)), ==, 0x3e);
    qtest_writel(qts, DIV(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DIV(3)), ==, 0xfff);
    qtest_writel(qts, CTR(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CTR(3)), ==, 0xffff);
    qtest_writel(qts, TOP(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TOP(3)), ==, 0xffff);
    qtest_writel(qts, CC(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CC(3)), ==, 0xffffffff);
    qtest_writel(qts, IRQ0_INTE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, IRQ0_INTE), ==, 0xfff);
    qtest_writel(qts, IRQ0_INTE, 0);

    /* Atomic aliases. */
    qtest_writel(qts, CC(3), 0x12345678);
    qtest_writel(qts, CC(3) + ALIAS_XOR, 0x0000ffff);
    g_assert_cmphex(qtest_readl(qts, CC(3)), ==, 0x1234a987);
    qtest_writel(qts, CC(3) + ALIAS_CLR, 0xffff0000);
    g_assert_cmphex(qtest_readl(qts, CC(3)), ==, 0x0000a987);
    qtest_writel(qts, CC(3) + ALIAS_SET, 0x00010000);
    g_assert_cmphex(qtest_readl(qts, CC(3) + ALIAS_SET), ==, 0x0001a987);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_counting(void)
{
    QTestState *qts = start();

    /* Full speed: one count per clk_sys cycle. */
    qtest_writel(qts, CSR(0), CSR_EN);
    step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(0)), ==, 1500);
    qtest_writel(qts, CSR(0), 0);
    step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(0)), ==, 1500);

    /* Divided to once per microsecond; CTR is writable while running. */
    qtest_writel(qts, CTR(0), 0);
    qtest_writel(qts, DIV(0), DIV_1US);
    qtest_writel(qts, CSR(0), CSR_EN);
    step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(0)), ==, 10);
    qtest_writel(qts, CTR(0), 0xfff0);
    step(qts, 20 * US);
    /* Wrapped at TOP 0xffff. */
    g_assert_cmpuint(qtest_readl(qts, CTR(0)), ==, 4);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1);

    /* DIV_INT 0 divides by 256. */
    qtest_writel(qts, CSR(1), 0);
    qtest_writel(qts, DIV(1), 0);
    qtest_writel(qts, CSR(1), CSR_EN);
    /* 10 counts take 2560 cycles: 17066.67 ns. */
    step(qts, 17066);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 9);
    step(qts, 1);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 10);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_fractional_divider(void)
{
    QTestState *qts = start();

    /* 2.5: two counts every five cycles, spaced 2 then 3 cycles. */
    qtest_writel(qts, DIV(2), DIV_INT(2) | 8);
    qtest_writel(qts, CSR(2), CSR_EN);
    step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(2)), ==, 600);

    /* 150.5 for 100 counts takes 15050 cycles. */
    qtest_writel(qts, CSR(2), 0);
    qtest_writel(qts, CTR(2), 0);
    qtest_writel(qts, DIV(2), DIV_INT(150) | 8);
    qtest_writel(qts, CSR(2), CSR_EN);
    step(qts, 100 * US + 333);
    g_assert_cmpuint(qtest_readl(qts, CTR(2)), ==, 99);
    step(qts, 1);
    g_assert_cmpuint(qtest_readl(qts, CTR(2)), ==, 100);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_wrap_interrupt(void)
{
    QTestState *qts = start();

    qtest_writel(qts, TOP(4), 9);
    qtest_writel(qts, DIV(4), DIV_1US);
    qtest_writel(qts, IRQ0_INTE, 1u << 4);
    qtest_writel(qts, CSR(4), CSR_EN);
    step(qts, 9 * US + 999);
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    step(qts, 1);
    g_assert_true(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_1));
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1u << 4);
    g_assert_cmphex(qtest_readl(qts, IRQ0_INTS), ==, 1u << 4);
    g_assert_cmphex(qtest_readl(qts, IRQ1_INTS), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, CTR(4)), ==, 0);

    /* The CLR and XOR aliases do not clear; a write of 1 does. */
    qtest_writel(qts, INTR + ALIAS_CLR, 1u << 4);
    qtest_writel(qts, INTR + ALIAS_XOR, 1u << 4);
    g_assert_true(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    qtest_writel(qts, INTR, 1u << 4);
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);

    /* The second interrupt line. */
    qtest_writel(qts, IRQ0_INTE, 0);
    qtest_writel(qts, IRQ1_INTE, 1u << 4);
    step(qts, 10 * US - 1);
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_1));
    step(qts, 1);
    g_assert_true(qtest_get_irq(qts, PWM_IRQ_WRAP_1));
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    g_assert_cmphex(qtest_readl(qts, IRQ1_INTS), ==, 1u << 4);
    qtest_writel(qts, INTR + ALIAS_SET, 1u << 4);
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_1));

    /* Wraps with the interrupt masked still set INTR. */
    qtest_writel(qts, IRQ1_INTE, 0);
    step(qts, 10 * US);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1u << 4);
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_1));

    /* Forcing. */
    qtest_writel(qts, IRQ0_INTF, 1u << 7);
    g_assert_true(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    g_assert_cmphex(qtest_readl(qts, IRQ0_INTS), ==, 1u << 7);
    qtest_writel(qts, IRQ0_INTF, 0);
    g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_double_buffering(void)
{
    QTestState *qts = start();

    qtest_writel(qts, TOP(5), 9);
    qtest_writel(qts, DIV(5), DIV_1US);
    qtest_writel(qts, CSR(5), CSR_EN);
    step(qts, 3 * US);
    /* The new TOP waits for the wrap at the old one. */
    qtest_writel(qts, TOP(5), 4);
    g_assert_cmphex(qtest_readl(qts, TOP(5)), ==, 4);
    step(qts, 6 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(5)), ==, 9);
    step(qts, 1 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(5)), ==, 0);
    step(qts, 4 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(5)), ==, 4);
    step(qts, 1 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(5)), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_phase_correct(void)
{
    static const uint32_t path[] = { 1, 2, 3, 3, 2, 1, 0, 0, 1, 2 };
    QTestState *qts = start();
    int i;

    qtest_writel(qts, TOP(6), 3);
    qtest_writel(qts, DIV(6), DIV_1US);
    qtest_writel(qts, CSR(6), CSR_EN | CSR_PH_CORRECT);
    for (i = 0; i < ARRAY_SIZE(path); i++) {
        step(qts, US);
        g_assert_cmpuint(qtest_readl(qts, CTR(6)), ==, path[i]);
        /* The wrap is the step from 0 to 0. */
        g_assert_cmphex(qtest_readl(qts, INTR), ==, i >= 7 ? 1u << 6 : 0);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_phase_adjust(void)
{
    QTestState *qts = start();

    qtest_writel(qts, DIV(7), DIV_1US);
    qtest_writel(qts, CSR(7), CSR_EN);
    step(qts, 5 * US + 500);
    g_assert_cmpuint(qtest_readl(qts, CTR(7)), ==, 5);

    /* PH_ADV adds a count at once. */
    qtest_writel(qts, CSR(7) + ALIAS_SET, CSR_PH_ADV);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN);
    g_assert_cmpuint(qtest_readl(qts, CTR(7)), ==, 6);

    /* PH_RET swallows the next count, then clears. */
    qtest_writel(qts, CSR(7) + ALIAS_SET, CSR_PH_RET);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN | CSR_PH_RET);
    step(qts, 499);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN | CSR_PH_RET);
    step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN);
    g_assert_cmpuint(qtest_readl(qts, CTR(7)), ==, 6);
    step(qts, US);
    g_assert_cmpuint(qtest_readl(qts, CTR(7)), ==, 7);

    /* At full speed there is no gap to advance into. */
    qtest_writel(qts, DIV(7), DIV_INT(1));
    qtest_writel(qts, CSR(7) + ALIAS_SET, CSR_PH_ADV);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN | CSR_PH_ADV);
    qtest_writel(qts, DIV(7), DIV_INT(1) | 1);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN);

    /* A stopped slice holds the request. */
    qtest_writel(qts, CSR(7), 0);
    qtest_writel(qts, CTR(7), 0);
    qtest_writel(qts, CSR(7) + ALIAS_SET, CSR_PH_ADV);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_PH_ADV);
    g_assert_cmpuint(qtest_readl(qts, CTR(7)), ==, 0);
    qtest_writel(qts, CSR(7) + ALIAS_SET, CSR_EN);
    g_assert_cmphex(qtest_readl(qts, CSR(7)), ==, CSR_EN);
    g_assert_cmpuint(qtest_readl(qts, CTR(7)), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_global_enable(void)
{
    QTestState *qts = start();

    qtest_writel(qts, DIV(8), DIV_INT(3) | 5);
    qtest_writel(qts, DIV(9), DIV_INT(3) | 5);
    qtest_writel(qts, CTR(9), 100);
    step(qts, 123);
    qtest_writel(qts, EN, (1u << 8) | (1u << 9));
    g_assert_cmphex(qtest_readl(qts, CSR(8)), ==, CSR_EN);
    g_assert_cmphex(qtest_readl(qts, CSR(9)), ==, CSR_EN);
    step(qts, 7777);
    /* In lockstep, 100 apart. */
    g_assert_cmpuint(qtest_readl(qts, CTR(9)) - qtest_readl(qts, CTR(8)), ==,
                     100);
    qtest_writel(qts, EN + ALIAS_CLR, 1u << 8);
    g_assert_cmphex(qtest_readl(qts, EN), ==, 1u << 9);
    g_assert_cmphex(qtest_readl(qts, CSR(8)), ==, 0);
    qtest_writel(qts, CSR(9), 0);
    g_assert_cmphex(qtest_readl(qts, EN), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_outputs(void)
{
    QTestState *qts = start();

    /* GPIO 0 and 1 carry slice 0's A and B. */
    pin_pwm(qts, 0);
    pin_pwm(qts, 1);
    g_assert_false(pin_in(qts, 0));
    g_assert_cmphex(qtest_readl(qts, STATUS(0)) & STATUS_OETOPAD, ==,
                    STATUS_OETOPAD);
    g_assert_cmphex(qtest_readl(qts, STATUS(1)) & STATUS_OETOPAD, ==,
                    STATUS_OETOPAD);

    /* A disabled slice takes CC at once. */
    qtest_writel(qts, CC(0), 1);
    g_assert_true(pin_in(qts, 0));
    qtest_writel(qts, CSR(0), CSR_A_INV);
    g_assert_false(pin_in(qts, 0));
    qtest_writel(qts, CSR(0), 0);

    /* A high for 3 of 10 counts; B (CC > TOP) always high. */
    qtest_writel(qts, TOP(0), 9);
    qtest_writel(qts, CC(0), (10u << 16) | 3);
    qtest_writel(qts, DIV(0), DIV_1US);
    qtest_writel(qts, CSR(0), CSR_EN);
    step(qts, 2 * US + 999);
    g_assert_true(pin_in(qts, 0));
    g_assert_true(pin_in(qts, 1));
    step(qts, 1);
    g_assert_false(pin_in(qts, 0));
    g_assert_cmphex(qtest_readl(qts, STATUS(0)) & STATUS_OUTTOPAD, ==, 0);
    step(qts, 7 * US - 1);
    g_assert_false(pin_in(qts, 0));
    step(qts, 1);
    g_assert_true(pin_in(qts, 0));
    g_assert_true(pin_in(qts, 1));

    /* A new CC applies from the next wrap; inversion at once. */
    qtest_writel(qts, CC(0), 5);
    qtest_writel(qts, CSR(0), CSR_EN | CSR_B_INV);
    g_assert_true(pin_in(qts, 0));
    g_assert_false(pin_in(qts, 1));
    step(qts, 3 * US);
    g_assert_false(pin_in(qts, 0));
    g_assert_false(pin_in(qts, 1));
    step(qts, 7 * US);
    /* Now A is 5 of 10, and B 0% inverted to always high. */
    g_assert_true(pin_in(qts, 1));
    g_assert_true(pin_in(qts, 0));
    step(qts, 4 * US);
    g_assert_true(pin_in(qts, 0));
    step(qts, 1 * US);
    g_assert_false(pin_in(qts, 0));
    g_assert_true(pin_in(qts, 1));

    /* Phase-correct: high while below CC on the way up and down. */
    qtest_writel(qts, CSR(0), 0);
    qtest_writel(qts, CTR(0), 0);
    qtest_writel(qts, TOP(0), 4);
    qtest_writel(qts, CC(0), 2);
    qtest_writel(qts, CSR(0), CSR_EN | CSR_PH_CORRECT);
    {
        /* Counts 0 1 2 3 4 4 3 2 1 0, then again. */
        static const bool a[] = { 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0 };
        int i;

        for (i = 0; i < ARRAY_SIZE(a); i++) {
            g_assert_cmpint(pin_in(qts, 0), ==, a[i]);
            step(qts, US);
        }
    }
    qtest_quit(qts);
}

/*
 * Edges every one or two clk_sys cycles, each driven at its own: the pin
 * shows the slice's level at whatever moment it is read.
 */
/* [spec:nuos:req:emu.pwm/test] */
static void test_fast_outputs(void)
{
    QTestState *qts = start();
    int i;

    pin_pwm(qts, 20);
    qtest_writel(qts, TOP(2), 2);
    qtest_writel(qts, CC(2), 1);
    qtest_writel(qts, CSR(2), CSR_EN);
    for (i = 0; i < 20; i++) {
        uint32_t ctr;

        step(qts, 1 * US + 7 * i);
        ctr = qtest_readl(qts, CTR(2));
        g_assert_cmpint(pin_in(qts, 20), ==, ctr < 1);
    }
    qtest_quit(qts);
}

/*
 * Edges a few clk_sys cycles apart, each at its own cycle: slice 0's A on
 * GPIO 0 high for 3 of 9 counts at the full 150 MHz, edges 20 ns and
 * 40 ns apart. The edges' cycles are three apart, so that from the first
 * edge they come at whole nanoseconds: the pad is checked on each side of
 * every edge for ten periods.
 */
/* [spec:nuos:req:emu.pwm/test] */
static void test_close_edges(void)
{
    QTestState *qts = start_pads();
    int64_t t0 = 0, now;
    int i;

    pin_pwm(qts, 0);
    qtest_writel(qts, TOP(0), 8);
    qtest_writel(qts, CC(0), 3);
    qtest_writel(qts, CSR(0), CSR_EN);
    /* The first rising edge after the high from the enable, 60 ns on */
    for (i = 0; i < 100; i++) {
        bool before = qtest_get_irq(qts, 0);

        t0 = qtest_clock_step(qts, 1);
        if (!before && qtest_get_irq(qts, 0)) {
            break;
        }
    }
    g_assert_cmpint(i, <, 100);
    now = t0;
    for (i = 0; i < 10; i++) {
        int64_t t = t0 + 60 * i;

        if (t + 19 > now) {
            qtest_clock_step(qts, t + 19 - now);
        }
        g_assert_true(qtest_get_irq(qts, 0));
        qtest_clock_step(qts, 1);
        g_assert_false(qtest_get_irq(qts, 0));
        qtest_clock_step(qts, 39);
        g_assert_false(qtest_get_irq(qts, 0));
        now = qtest_clock_step(qts, 1);
        g_assert_true(qtest_get_irq(qts, 0));
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pwm/test] */
static void test_input_modes(void)
{
    QTestState *qts = start();
    int i;

    /* Slice 1's B is GPIO 3. */
    pin_pwm(qts, 3);
    drive(qts, 3, 0);

    /* Rising edges, each passed by the divider (DIV 1). */
    qtest_writel(qts, CSR(1), CSR_EN | CSR_DIVMODE(DIVMODE_RISE));
    g_assert_cmphex(qtest_readl(qts, STATUS(3)) & STATUS_OETOPAD, ==, 0);
    for (i = 0; i < 5; i++) {
        drive(qts, 3, 1);
        step(qts, 100);
        drive(qts, 3, 0);
        step(qts, 100);
    }
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 5);

    /* A pulse within one clk_sys cycle is not seen. */
    drive(qts, 3, 1);
    drive(qts, 3, 0);
    step(qts, 100);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 5);

    /* Falling edges, through a divide by 2. */
    qtest_writel(qts, CSR(1), CSR_EN | CSR_DIVMODE(DIVMODE_FALL));
    qtest_writel(qts, DIV(1), DIV_INT(2));
    for (i = 0; i < 6; i++) {
        drive(qts, 3, 1);
        step(qts, 100);
        drive(qts, 3, 0);
        step(qts, 100);
    }
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 8);

    /* Level-gated: counts only while B is high. */
    qtest_writel(qts, CSR(1), 0);
    qtest_writel(qts, CTR(1), 0);
    qtest_writel(qts, DIV(1), DIV_1US);
    qtest_writel(qts, CSR(1), CSR_EN | CSR_DIVMODE(DIVMODE_LEVEL));
    step(qts, 5 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 0);
    drive(qts, 3, 1);
    step(qts, 7 * US + 10);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 7);
    drive(qts, 3, 0);
    step(qts, 20 * US);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 7);
    drive(qts, 3, 1);
    step(qts, 3 * US + 10);
    g_assert_cmpuint(qtest_readl(qts, CTR(1)), ==, 10);

    /* Free-running again: B is an output. */
    qtest_writel(qts, CSR(1), CSR_EN);
    g_assert_cmphex(qtest_readl(qts, STATUS(3)) & STATUS_OETOPAD, ==,
                    STATUS_OETOPAD);

    /* An edge-counted wrap interrupts. */
    qtest_writel(qts, CSR(1), 0);
    qtest_writel(qts, CTR(1), 0);
    qtest_writel(qts, TOP(1), 2);
    qtest_writel(qts, DIV(1), DIV_INT(1));
    qtest_writel(qts, IRQ0_INTE, 1u << 1);
    qtest_writel(qts, CSR(1), CSR_EN | CSR_DIVMODE(DIVMODE_RISE));
    drive(qts, 3, 0);
    for (i = 0; i < 3; i++) {
        g_assert_false(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
        drive(qts, 3, 1);
        step(qts, 100);
        drive(qts, 3, 0);
        step(qts, 100);
    }
    g_assert_true(qtest_get_irq(qts, PWM_IRQ_WRAP_0));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-pwm-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/pwm/reset-values", test_reset_values);
    qtest_add_func("/rp2350/pwm/counting", test_counting);
    qtest_add_func("/rp2350/pwm/fractional-divider", test_fractional_divider);
    qtest_add_func("/rp2350/pwm/wrap-interrupt", test_wrap_interrupt);
    qtest_add_func("/rp2350/pwm/double-buffering", test_double_buffering);
    qtest_add_func("/rp2350/pwm/phase-correct", test_phase_correct);
    qtest_add_func("/rp2350/pwm/phase-adjust", test_phase_adjust);
    qtest_add_func("/rp2350/pwm/global-enable", test_global_enable);
    qtest_add_func("/rp2350/pwm/outputs", test_outputs);
    qtest_add_func("/rp2350/pwm/fast-outputs", test_fast_outputs);
    qtest_add_func("/rp2350/pwm/input-modes", test_input_modes);
    qtest_add_func("/rp2350/pwm/close-edges", test_close_edges);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
