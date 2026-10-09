/*
 * QTest testcase for the ESP32 GPIO matrix, IO_MUX and sigma-delta
 * modulators
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GPIO                0x3ff44000
#define GPIO_OUT            (GPIO + 0x04)
#define GPIO_OUT_W1TS       (GPIO + 0x08)
#define GPIO_OUT_W1TC       (GPIO + 0x0c)
#define GPIO_OUT1           (GPIO + 0x10)
#define GPIO_ENABLE         (GPIO + 0x20)
#define GPIO_ENABLE_W1TS    (GPIO + 0x24)
#define GPIO_ENABLE_W1TC    (GPIO + 0x28)
#define GPIO_ENABLE1        (GPIO + 0x2c)
#define GPIO_ENABLE1_W1TS   (GPIO + 0x30)
#define GPIO_STRAP          (GPIO + 0x38)
#define GPIO_IN             (GPIO + 0x3c)
#define GPIO_IN1            (GPIO + 0x40)
#define GPIO_STATUS         (GPIO + 0x44)
#define GPIO_STATUS_W1TS    (GPIO + 0x48)
#define GPIO_STATUS_W1TC    (GPIO + 0x4c)
#define GPIO_STATUS1        (GPIO + 0x50)
#define GPIO_STATUS1_W1TC   (GPIO + 0x58)
#define GPIO_ACPU_INT       (GPIO + 0x60)
#define GPIO_ACPU_NMI_INT   (GPIO + 0x64)
#define GPIO_PCPU_INT       (GPIO + 0x68)
#define GPIO_PCPU_NMI_INT   (GPIO + 0x6c)
#define GPIO_PCPU_INT1      (GPIO + 0x7c)
#define GPIO_PIN(n)         (GPIO + 0x88 + 4 * (n))
#define GPIO_FUNC_IN(s)     (GPIO + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x530 + 4 * (n))
#define GPIO_SIGMADELTA(n)  (GPIO + 0xf00 + 4 * (n))
#define GPIO_SIGMADELTA_CG  (GPIO + 0xf20)
#define GPIO_SD_VERSION     (GPIO + 0xf28)

#define PIN_PAD_DRIVER      (1u << 2)
#define PIN_INT_TYPE(t)     ((t) << 7)
#define PIN_INT_ENA(e)      ((e) << 13)
#define INT_ENA_APP         1
#define INT_ENA_APP_NMI     2
#define INT_ENA_PRO         4
#define INT_ENA_PRO_NMI     8

#define IN_SIG_IN_SEL       (1u << 7)
#define IN_INV_SEL          (1u << 6)
#define OUT_INV_SEL         (1u << 9)
#define OEN_SEL             (1u << 10)
#define OEN_INV_SEL         (1u << 11)
#define SIG_GPIO_OUT        0x100

#define IO_MUX              0x3ff49000
#define IO_MUX_GPIO0        (IO_MUX + 0x44)
#define IO_MUX_GPIO1        (IO_MUX + 0x88)
#define IO_MUX_GPIO2        (IO_MUX + 0x40)
#define IO_MUX_GPIO3        (IO_MUX + 0x84)
#define IO_MUX_GPIO16       (IO_MUX + 0x4c)
#define IO_MUX_GPIO18       (IO_MUX + 0x70)
#define IO_MUX_GPIO19       (IO_MUX + 0x74)
#define IO_MUX_GPIO25       (IO_MUX + 0x24)
#define IO_MUX_GPIO34       (IO_MUX + 0x14)
#define IO_MUX_GPIO36       (IO_MUX + 0x04)
#define IO_MUX_GPIO39       (IO_MUX + 0x10)
#define FUN_WPD             (1u << 7)
#define FUN_WPU             (1u << 8)
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((d) << 10)
#define MCU_SEL(f)          ((f) << 12)
#define PIN_FUNC_GPIO       2

#define SIG_U0RXD           14
#define SIG_HSPICS0         11
#define SIG_I2CEXT0_SCL     29
#define SIG_LEDC_HS0        71
#define SIG_RMT_IN0         83
#define SIG_GPIO_SD0        100

#define GPIO_PATH           "/machine/soc/gpio"
#define PAD_IN              "esp32-gpio-pad-in"
#define PAD_RELEASE         "esp32-gpio-pad-release"
#define PAD_OUT             "esp32-gpio-pad"
#define SIG_OUT             "esp32-gpio-sig-out"
#define SIG_IN              "esp32-gpio-sig-in"

/* The block's sysbus IRQs */
#define IRQ_PRO             0
#define IRQ_PRO_NMI         1
#define IRQ_APP             2
#define IRQ_APP_NMI         3

static QTestState *start(void)
{
    return qtest_init("-M esp32 -nic none");
}

static bool in_bit(QTestState *qts, unsigned n)
{
    if (n < 32) {
        return (qtest_readl(qts, GPIO_IN) >> n) & 1;
    }
    return (qtest_readl(qts, GPIO_IN1) >> (n - 32)) & 1;
}

static void drive(QTestState *qts, unsigned n, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, n, level);
}

static void release(QTestState *qts, unsigned n)
{
    qtest_set_irq_in(qts, GPIO_PATH, PAD_RELEASE, n, 1);
}

/* GPIO18 as a GPIO-matrix pad with its input enabled */
static void gpio18_matrix(QTestState *qts)
{
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x12);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_PIN(5)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(SIG_U0RXD)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_OUT(5)), ==, SIG_GPIO_OUT);

    /* The "Reset" column of TRM table 6.10-1, with FUN_DRV = 2 */
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO0), ==,
                    FUN_DRV(2) | FUN_IE | FUN_WPU);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO2), ==,
                    FUN_DRV(2) | FUN_IE | FUN_WPD);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO16), ==, FUN_DRV(2) | FUN_IE);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO25), ==, FUN_DRV(2));
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO36), ==, 0);

    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA(3)), ==, 0xff00);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA_CG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_SD_VERSION), ==, 0x1506190);

    /*
     * Pads at reset: GPIO0 and U0RXD pulled up, GPIO2 pulled down, GPIO16
     * floating, U0TXD driven high by the idle UART.
     */
    g_assert_true(in_bit(qts, 0));
    g_assert_true(in_bit(qts, 1));
    g_assert_false(in_bit(qts, 2));
    g_assert_true(in_bit(qts, 3));
    g_assert_false(in_bit(qts, 16));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_strap(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none -global "
                                 "driver=esp32.gpio,property=strap_mode,"
                                 "value=0x0f");

    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x0f);
    qtest_writel(qts, GPIO_STRAP, 0x12);
    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x0f);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_output_readback(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, PAD_OUT);
    gpio18_matrix(qts);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 18);
    /* Not enabled: the pad floats */
    g_assert_false(in_bit(qts, 18));
    g_assert_false(qtest_get_irq(qts, 18));

    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_ENABLE), ==, 1u << 18);
    g_assert_true(in_bit(qts, 18));
    g_assert_true(qtest_get_irq(qts, 18));

    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    g_assert_false(in_bit(qts, 18));
    g_assert_false(qtest_get_irq(qts, 18));

    /* Input disabled: GPIO_IN reads 0 while the pad is high */
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 18);
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL(PIN_FUNC_GPIO));
    g_assert_true(qtest_get_irq(qts, 18));
    g_assert_false(in_bit(qts, 18));

    /* OUT_INV_SEL inverts the GPIO_OUT bit */
    gpio18_matrix(qts);
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_GPIO_OUT | OUT_INV_SEL);
    g_assert_false(in_bit(qts, 18));

    /*
     * Another IO_MUX function takes the pad from the matrix: VSPICLK,
     * low while SPI3 does not drive it.
     */
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_GPIO_OUT);
    g_assert_true(in_bit(qts, 18));
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL(1) | FUN_IE);
    g_assert_false(in_bit(qts, 18));

    /* GPIO32-39 through OUT1 / ENABLE1 */
    qtest_writel(qts, IO_MUX + 0x1c, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, GPIO_OUT1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT1), ==, 0xff);
    qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << 0);
    g_assert_true(in_bit(qts, 32));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_pads(void)
{
    QTestState *qts = start();

    /* An outside driver wins over pulls and the chip's own driver */
    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPU);
    g_assert_true(in_bit(qts, 19));
    drive(qts, 19, 0);
    g_assert_false(in_bit(qts, 19));
    release(qts, 19);
    g_assert_true(in_bit(qts, 19));
    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPD);
    g_assert_false(in_bit(qts, 19));

    /* Open drain: a high output releases the pad to its pull-up */
    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPU);
    qtest_writel(qts, GPIO_PIN(19), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 19);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 19);
    g_assert_true(in_bit(qts, 19));
    drive(qts, 19, 0);
    g_assert_false(in_bit(qts, 19));
    release(qts, 19);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 19);
    g_assert_false(in_bit(qts, 19));

    /* GPIO34-39 are input-only, without pulls or drive strength */
    qtest_writel(qts, IO_MUX_GPIO34, 0x7fff);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO34), ==,
                    0x7fff & ~(FUN_WPU | FUN_WPD | FUN_DRV(3)));
    qtest_writel(qts, IO_MUX_GPIO34, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, GPIO_ENABLE1, 0xff);
    qtest_writel(qts, GPIO_OUT1, 0xff);
    g_assert_false(in_bit(qts, 34));
    drive(qts, 34, 1);
    g_assert_true(in_bit(qts, 34));

    /* Pads that do not exist read 0 */
    drive(qts, 20, 1);
    g_assert_false(in_bit(qts, 20));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_input_routing(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, SIG_IN);
    /* A reset drives every input signal, so the intercept sees them all */
    qtest_system_reset(qts);

    /* Bypassing the matrix with no IO_MUX source: the signal's default */
    g_assert_false(qtest_get_irq(qts, SIG_RMT_IN0));
    g_assert_true(qtest_get_irq(qts, SIG_I2CEXT0_SCL));

    /* Through the matrix from GPIO18 */
    gpio18_matrix(qts);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN0), IN_SIG_IN_SEL | 18);
    drive(qts, 18, 1);
    g_assert_true(qtest_get_irq(qts, SIG_RMT_IN0));
    drive(qts, 18, 0);
    g_assert_false(qtest_get_irq(qts, SIG_RMT_IN0));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN0),
                 IN_SIG_IN_SEL | IN_INV_SEL | 18);
    g_assert_true(qtest_get_irq(qts, SIG_RMT_IN0));

    /* The pad's input buffer must be on */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN0), IN_SIG_IN_SEL | 18);
    drive(qts, 18, 1);
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL(PIN_FUNC_GPIO));
    g_assert_false(qtest_get_irq(qts, SIG_RMT_IN0));

    /* The constant inputs */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN0), IN_SIG_IN_SEL | 0x38);
    g_assert_true(qtest_get_irq(qts, SIG_RMT_IN0));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN0), IN_SIG_IN_SEL | 0x30);
    g_assert_false(qtest_get_irq(qts, SIG_RMT_IN0));

    /*
     * U0RXD bypasses the matrix by default and comes from U0RXD (GPIO3) in
     * its IO_MUX function 0, pulled up.
     */
    g_assert_true(qtest_get_irq(qts, SIG_U0RXD));
    drive(qts, 3, 0);
    g_assert_false(qtest_get_irq(qts, SIG_U0RXD));
    /* With GPIO3 in its GPIO function, U0RXD has no source */
    qtest_writel(qts, IO_MUX_GPIO3, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    drive(qts, 3, 1);
    g_assert_false(qtest_get_irq(qts, SIG_U0RXD));
    /* Routed through the matrix from GPIO3 instead */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_U0RXD), IN_SIG_IN_SEL | 3);
    g_assert_true(qtest_get_irq(qts, SIG_U0RXD));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_output_routing(void)
{
    QTestState *qts = start();

    gpio18_matrix(qts);
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_LEDC_HS0);
    /* The peripheral's own output enable (1'd1) drives the pad */
    g_assert_false(in_bit(qts, 18));
    qtest_set_irq_in(qts, GPIO_PATH, SIG_OUT, SIG_LEDC_HS0, 1);
    g_assert_true(in_bit(qts, 18));
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_LEDC_HS0 | OUT_INV_SEL);
    g_assert_false(in_bit(qts, 18));

    /* OEN_SEL takes the enable from GPIO_ENABLE, OEN_INV_SEL inverts it */
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPU);
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_LEDC_HS0 | OEN_SEL);
    g_assert_true(in_bit(qts, 18));
    qtest_set_irq_in(qts, GPIO_PATH, SIG_OUT, SIG_LEDC_HS0, 0);
    g_assert_true(in_bit(qts, 18));
    qtest_writel(qts, GPIO_FUNC_OUT(18),
                 SIG_LEDC_HS0 | OEN_SEL | OEN_INV_SEL);
    g_assert_false(in_bit(qts, 18));

    /* One signal can drive several pads */
    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(19), SIG_LEDC_HS0);
    qtest_set_irq_in(qts, GPIO_PATH, SIG_OUT, SIG_LEDC_HS0, 1);
    g_assert_true(in_bit(qts, 18));
    g_assert_true(in_bit(qts, 19));

    /* SPI2's chip select, idle high, reaches a pad through the matrix */
    qtest_writel(qts, GPIO_FUNC_OUT(19), SIG_HSPICS0);
    g_assert_true(in_bit(qts, 19));

    /* Signals 224-228 route one pad to another */
    qtest_writel(qts, GPIO_FUNC_IN(224), IN_SIG_IN_SEL | 21);
    qtest_writel(qts, IO_MUX + 0x7c, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(19), 224);
    drive(qts, 21, 0);
    g_assert_false(in_bit(qts, 19));
    drive(qts, 21, 1);
    g_assert_true(in_bit(qts, 19));

    /* U0TXD in its IO_MUX function bypasses the matrix */
    g_assert_true(in_bit(qts, 1));
    qtest_set_irq_in(qts, GPIO_PATH, SIG_OUT, 14, 0);
    g_assert_false(in_bit(qts, 1));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_interrupts(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "sysbus-irq");
    gpio18_matrix(qts);
    drive(qts, 18, 0);

    /* Rising edge to the PRO CPU */
    qtest_writel(qts, GPIO_PIN(18), PIN_INT_TYPE(1) |
                 PIN_INT_ENA(INT_ENA_PRO));
    drive(qts, 18, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_PCPU_INT), ==, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_ACPU_INT), ==, 0);
    g_assert_true(qtest_get_irq(qts, IRQ_PRO));
    g_assert_false(qtest_get_irq(qts, IRQ_APP));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(qts, IRQ_PRO));
    /* A falling edge does not latch */
    drive(qts, 18, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);

    /* Falling edge to the APP CPU */
    qtest_writel(qts, GPIO_PIN(18), PIN_INT_TYPE(2) |
                 PIN_INT_ENA(INT_ENA_APP));
    drive(qts, 18, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    drive(qts, 18, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_ACPU_INT), ==, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_PCPU_INT), ==, 0);
    g_assert_true(qtest_get_irq(qts, IRQ_APP));
    g_assert_false(qtest_get_irq(qts, IRQ_PRO));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);
    g_assert_false(qtest_get_irq(qts, IRQ_APP));

    /* Any edge, to both NMIs */
    qtest_writel(qts, GPIO_PIN(18), PIN_INT_TYPE(3) |
                 PIN_INT_ENA(INT_ENA_PRO_NMI | INT_ENA_APP_NMI));
    drive(qts, 18, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_PCPU_NMI_INT), ==, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_ACPU_NMI_INT), ==, 1u << 18);
    g_assert_true(qtest_get_irq(qts, IRQ_PRO_NMI));
    g_assert_true(qtest_get_irq(qts, IRQ_APP_NMI));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);
    drive(qts, 18, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 18);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);

    /* High level: the status sets again while the level holds */
    qtest_writel(qts, GPIO_PIN(18), PIN_INT_TYPE(5) |
                 PIN_INT_ENA(INT_ENA_PRO));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    drive(qts, 18, 1);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 18);
    g_assert_true(qtest_get_irq(qts, IRQ_PRO));
    drive(qts, 18, 0);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(qts, IRQ_PRO));

    /* Low level */
    qtest_writel(qts, GPIO_PIN(18), PIN_INT_TYPE(4) |
                 PIN_INT_ENA(INT_ENA_PRO));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 18);
    drive(qts, 18, 1);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);

    /* Disabled interrupts latch nothing; W1TS sets the status by hand */
    qtest_writel(qts, GPIO_PIN(18), PIN_INT_ENA(INT_ENA_PRO));
    drive(qts, 18, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    qtest_writel(qts, GPIO_STATUS_W1TS, 1u << 18);
    g_assert_true(qtest_get_irq(qts, IRQ_PRO));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 18);

    /* GPIO39 latches into STATUS1 and the INT1 views */
    qtest_writel(qts, IO_MUX_GPIO39, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, GPIO_PIN(39), PIN_INT_TYPE(1) |
                 PIN_INT_ENA(INT_ENA_PRO));
    drive(qts, 39, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS1), ==, 1u << 7);
    g_assert_cmphex(qtest_readl(qts, GPIO_PCPU_INT1), ==, 1u << 7);
    g_assert_true(qtest_get_irq(qts, IRQ_PRO));
    qtest_writel(qts, GPIO_STATUS1_W1TC, 1u << 7);
    g_assert_false(qtest_get_irq(qts, IRQ_PRO));

    qtest_quit(qts);
}

/*
 * Fraction of GPIO18 samples, every step_ns, that read high, in 1/1000.
 * The modulator's output holds its last bit until its next tick, so the
 * sampling starts a few ticks after the settings change.
 */
static unsigned sample_density(QTestState *qts, unsigned samples,
                               unsigned step_ns)
{
    unsigned high = 0;

    qtest_clock_step(qts, 10000);
    for (unsigned i = 0; i < samples; i++) {
        qtest_clock_step(qts, step_ns);
        high += in_bit(qts, 18);
    }
    return high * 1000 / samples;
}

/* [spec:nuos:req:emu.esp32.gpio/test] */
static void test_sigma_delta(void)
{
    QTestState *qts = start();
    unsigned d;

    gpio18_matrix(qts);
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_GPIO_SD0);

    /*
     * APB_CLK is 40 MHz out of reset; PRESCALE 99 ticks the modulator at
     * 400 kHz. Duty -64 gives a density of 64 / 256.
     */
    qtest_writel(qts, GPIO_SIGMADELTA(0), (99 << 8) | 0xc0);
    d = sample_density(qts, 4000, 1100);
    g_assert_cmpuint(d, >=, 230);
    g_assert_cmpuint(d, <=, 270);

    /* Duty 64: 192 / 256 */
    qtest_writel(qts, GPIO_SIGMADELTA(0), (99 << 8) | 0x40);
    d = sample_density(qts, 4000, 1100);
    g_assert_cmpuint(d, >=, 730);
    g_assert_cmpuint(d, <=, 770);

    /* The extremes: -128 never pulses, 127 pulses 255 ticks in 256 */
    qtest_writel(qts, GPIO_SIGMADELTA(0), (99 << 8) | 0x80);
    g_assert_cmpuint(sample_density(qts, 500, 1100), ==, 0);
    qtest_writel(qts, GPIO_SIGMADELTA(0), (99 << 8) | 0x7f);
    g_assert_cmpuint(sample_density(qts, 500, 1100), >=, 980);

    /*
     * Channel 7 drives its own signal. At 4 MHz its 50% density is the
     * pattern 0101..., which a 1337 ns step does not alias with.
     */
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_GPIO_SD0 + 7);
    qtest_writel(qts, GPIO_SIGMADELTA(7), (9 << 8) | 0x00);
    d = sample_density(qts, 4000, 1337);
    g_assert_cmpuint(d, >=, 470);
    g_assert_cmpuint(d, <=, 530);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32-gpio/reset", test_reset);
    qtest_add_func("esp32-gpio/strap", test_strap);
    qtest_add_func("esp32-gpio/output-readback", test_output_readback);
    qtest_add_func("esp32-gpio/pads", test_pads);
    qtest_add_func("esp32-gpio/input-routing", test_input_routing);
    qtest_add_func("esp32-gpio/output-routing", test_output_routing);
    qtest_add_func("esp32-gpio/interrupts", test_interrupts);
    qtest_add_func("esp32-gpio/sigma-delta", test_sigma_delta);

    return g_test_run();
}
