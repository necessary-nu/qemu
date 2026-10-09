/*
 * QTest testcase for the RP2350 IO muxing and pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Pins are driven from outside through the GPIO block's "pad-in" lines;
 * qtest accesses act as core 0 and are Secure.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GPIO_PATH   "/machine/soc/gpio"

#define IO_BANK0    0x40028000
#define IO_QSPI     0x40030000
#define PADS_BANK0  0x40038000
#define PADS_QSPI   0x40040000
#define ALIAS_XOR   0x1000
#define ALIAS_SET   0x2000
#define ALIAS_CLR   0x3000

#define STATUS(p)   (IO_BANK0 + 8 * (p))
#define CTRL(p)     (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)      (PADS_BANK0 + 4 + 4 * (p))
#define PADS_SWCLK  (PADS_BANK0 + 0xc4)
#define PADS_SWD    (PADS_BANK0 + 0xc8)

#define IRQSUMMARY_PROC0_SECURE0 (IO_BANK0 + 0x200)
#define IRQSUMMARY_PROC0_NONSECURE0 (IO_BANK0 + 0x208)
#define IRQSUMMARY_PROC1_SECURE0 (IO_BANK0 + 0x210)
#define INTR(w)       (IO_BANK0 + 0x230 + 4 * (w))
#define PROC0_INTE(w) (IO_BANK0 + 0x248 + 4 * (w))
#define PROC0_INTF(w) (IO_BANK0 + 0x260 + 4 * (w))
#define PROC0_INTS(w) (IO_BANK0 + 0x278 + 4 * (w))
#define PROC1_INTE(w) (IO_BANK0 + 0x290 + 4 * (w))
#define PROC1_INTS(w) (IO_BANK0 + 0x2c0 + 4 * (w))
#define DORMANT_WAKE_INTE(w) (IO_BANK0 + 0x2d8 + 4 * (w))
#define DORMANT_WAKE_INTS(w) (IO_BANK0 + 0x308 + 4 * (w))

#define QSPI_CTRL(q)     (IO_QSPI + 8 * (q) + 4)
#define QSPI_SUMMARY_PROC0_SECURE (IO_QSPI + 0x200)
#define QSPI_INTR        (IO_QSPI + 0x218)
#define QSPI_PROC0_INTE  (IO_QSPI + 0x21c)
#define QSPI_PROC0_INTS  (IO_QSPI + 0x224)
#define QSPI_PIN_SS      3
#define QSPI_PIN_SD2     6

#define STATUS_OUTTOPAD  (1u << 9)
#define STATUS_OETOPAD   (1u << 13)
#define STATUS_INFROMPAD (1u << 17)
#define STATUS_IRQTOPROC (1u << 26)

#define OUTOVER(v)  ((v) << 12)
#define OEOVER(v)   ((v) << 14)
#define INOVER(v)   ((v) << 16)
#define IRQOVER(v)  ((v) << 28)
#define OVER_INVERT 1
#define OVER_LOW    2
#define OVER_HIGH   3

#define FUNC_SPI    1
#define FUNC_UART   2
#define FUNC_SIO    5
#define FUNC_NULL   0x1f

#define PAD_PDE     (1u << 2)
#define PAD_PUE     (1u << 3)
#define PAD_IE      (1u << 6)
#define PAD_OD      (1u << 7)
#define PAD_ISO     (1u << 8)
#define PAD_DEFAULT 0x116

#define SIO_GPIO_IN     0xd0000004
#define SIO_GPIO_HI_IN  0xd0000008
#define SIO_GPIO_OUT    0xd0000010
#define SIO_GPIO_OE     0xd0000030

#define LEVEL_LOW   0x1
#define LEVEL_HIGH  0x2
#define EDGE_LOW    0x4
#define EDGE_HIGH   0x8

#define IO_IRQ_BANK0    21
#define IO_IRQ_BANK0_NS 22
#define IO_IRQ_QSPI     23

/* The UART signal numbers on the GPIO block's uart ports. */
#define UART_RX     1

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

static QTestState *start_on_core(int core)
{
    QTestState *qts = start();
    g_autofree char *cpu = g_strdup_printf("/machine/soc/armv7m[%d]", core);

    qtest_irq_intercept_in(qts, cpu);
    return qts;
}

/* Drive pin p from outside: 0, 1, or -1 to release it. */
static void drive(QTestState *qts, int p, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", p, level);
}

/* Give pin p an enabled, unisolated pad with the given pulls. */
static void pad_open(QTestState *qts, int p, uint32_t pulls)
{
    qtest_writel(qts, PAD(p), PAD_IE | pulls);
}

static bool gpio_in(QTestState *qts, int p)
{
    return (qtest_readl(qts, SIO_GPIO_IN) >> p) & 1;
}

static uint32_t int_field(QTestState *qts, uint64_t reg, int p)
{
    return (qtest_readl(qts, reg) >> (4 * (p % 8))) & 0xf;
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int p;

    for (p = 0; p < 48; p++) {
        g_assert_cmphex(qtest_readl(qts, STATUS(p)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, CTRL(p)), ==, FUNC_NULL);
        g_assert_cmphex(qtest_readl(qts, PAD(p)), ==, PAD_DEFAULT);
    }
    for (p = 0; p < 8; p++) {
        g_assert_cmphex(qtest_readl(qts, QSPI_CTRL(p)), ==, FUNC_NULL);
    }
    g_assert_cmphex(qtest_readl(qts, PADS_BANK0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PADS_SWCLK), ==, 0x5a);
    g_assert_cmphex(qtest_readl(qts, PADS_SWD), ==, 0x5a);
    /* SCLK, SD0 and SD1 pull down; SD2, SD3 and SS pull up. */
    g_assert_cmphex(qtest_readl(qts, PADS_QSPI + 0x04), ==, 0x156);
    g_assert_cmphex(qtest_readl(qts, PADS_QSPI + 0x08), ==, 0x156);
    g_assert_cmphex(qtest_readl(qts, PADS_QSPI + 0x0c), ==, 0x156);
    g_assert_cmphex(qtest_readl(qts, PADS_QSPI + 0x10), ==, 0x15a);
    g_assert_cmphex(qtest_readl(qts, PADS_QSPI + 0x14), ==, 0x15a);
    g_assert_cmphex(qtest_readl(qts, PADS_QSPI + 0x18), ==, 0x15a);
    g_assert_cmphex(qtest_readl(qts, PROC0_INTE(0)), ==, 0);

    /* Writable bits only; STATUS is read-only. */
    qtest_writel(qts, CTRL(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CTRL(3)), ==, 0x3003f01f);
    qtest_writel(qts, STATUS(3), 0xffffffff);
    qtest_writel(qts, PAD(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PAD(3)), ==, 0x1ff);
    qtest_writel(qts, PADS_BANK0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PADS_BANK0), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_atomic_aliases(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CTRL(7) + ALIAS_CLR, 0x1f);
    qtest_writel(qts, CTRL(7) + ALIAS_SET, FUNC_SIO | OUTOVER(OVER_HIGH));
    g_assert_cmphex(qtest_readl(qts, CTRL(7)), ==,
                    FUNC_SIO | OUTOVER(OVER_HIGH));
    qtest_writel(qts, CTRL(7) + ALIAS_XOR, OUTOVER(OVER_HIGH));
    g_assert_cmphex(qtest_readl(qts, CTRL(7) + ALIAS_XOR), ==, FUNC_SIO);
    qtest_writel(qts, PAD(7) + ALIAS_CLR, PAD_ISO);
    g_assert_cmphex(qtest_readl(qts, PAD(7)), ==, PAD_DEFAULT & ~PAD_ISO);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_input_path(void)
{
    QTestState *qts = start();
    int p = 5;

    drive(qts, p, 1);
    /* Input disabled at reset, and held disabled by the isolation latch. */
    g_assert_false(gpio_in(qts, p));
    qtest_writel(qts, PAD(p) + ALIAS_SET, PAD_IE);
    g_assert_false(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)), ==, 0);
    qtest_writel(qts, PAD(p) + ALIAS_CLR, PAD_ISO);
    g_assert_true(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)), ==,
                    STATUS_INFROMPAD | STATUS_IRQTOPROC);

    /* SIO reads the input whatever FUNCSEL selects, after INOVER. */
    qtest_writel(qts, CTRL(p), FUNC_NULL | INOVER(OVER_INVERT));
    g_assert_false(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)) & STATUS_INFROMPAD, ==,
                    STATUS_INFROMPAD);
    qtest_writel(qts, CTRL(p), FUNC_NULL | INOVER(OVER_HIGH));
    drive(qts, p, 0);
    g_assert_true(gpio_in(qts, p));
    qtest_writel(qts, CTRL(p), FUNC_NULL);
    g_assert_false(gpio_in(qts, p));

    /* Undriven, the pin follows its pulls; with both it keeps its level. */
    drive(qts, p, 1);
    drive(qts, p, -1);
    g_assert_false(gpio_in(qts, p));
    pad_open(qts, p, PAD_PUE);
    g_assert_true(gpio_in(qts, p));
    pad_open(qts, p, PAD_PUE | PAD_PDE);
    g_assert_true(gpio_in(qts, p));
    drive(qts, p, 0);
    drive(qts, p, -1);
    g_assert_false(gpio_in(qts, p));

    /* Disabling the input buffer reads 0. */
    drive(qts, p, 1);
    qtest_writel(qts, PAD(p), 0);
    g_assert_false(gpio_in(qts, p));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_unbonded_pins(void)
{
    QTestState *qts = start();

    /* The RP2350A bonds out GPIOs 0-29; GPIO 35 has no pin to drive. */
    pad_open(qts, 35, 0);
    drive(qts, 35, 1);
    g_assert_cmphex(qtest_readl(qts, SIO_GPIO_HI_IN) & 0xffff, ==, 0);
    pad_open(qts, 35, PAD_PUE);
    g_assert_cmphex(qtest_readl(qts, SIO_GPIO_HI_IN) & 0xffff, ==, 1u << 3);

    pad_open(qts, 29, 0);
    drive(qts, 29, 1);
    g_assert_true(gpio_in(qts, 29));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_output_path(void)
{
    QTestState *qts = start();
    int p = 9;

    qtest_writel(qts, SIO_GPIO_OUT, 1u << p);
    qtest_writel(qts, SIO_GPIO_OE, 1u << p);
    pad_open(qts, p, PAD_PDE);
    /* SIO drives the pin only when selected. */
    g_assert_cmphex(qtest_readl(qts, STATUS(p)), ==, 0);
    g_assert_false(gpio_in(qts, p));

    qtest_writel(qts, CTRL(p), FUNC_SIO);
    g_assert_cmphex(qtest_readl(qts, STATUS(p)), ==,
                    STATUS_OUTTOPAD | STATUS_OETOPAD | STATUS_INFROMPAD |
                    STATUS_IRQTOPROC);
    g_assert_true(gpio_in(qts, p));
    /* The chip's driver wins over an external one. */
    drive(qts, p, 0);
    g_assert_true(gpio_in(qts, p));

    /* Output overrides. */
    qtest_writel(qts, CTRL(p), FUNC_SIO | OUTOVER(OVER_INVERT));
    g_assert_false(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)) & STATUS_OUTTOPAD, ==, 0);
    qtest_writel(qts, CTRL(p), FUNC_SIO | OUTOVER(OVER_LOW));
    g_assert_false(gpio_in(qts, p));
    qtest_writel(qts, CTRL(p), FUNC_SIO | OUTOVER(OVER_HIGH));
    g_assert_true(gpio_in(qts, p));

    /* Output-enable overrides: disabled, the external drive shows. */
    qtest_writel(qts, CTRL(p), FUNC_SIO | OEOVER(OVER_LOW));
    g_assert_false(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)) & STATUS_OETOPAD, ==, 0);
    qtest_writel(qts, CTRL(p), FUNC_SIO | OEOVER(OVER_INVERT));
    g_assert_false(gpio_in(qts, p));
    qtest_writel(qts, SIO_GPIO_OE, 0);
    g_assert_true(gpio_in(qts, p));
    qtest_writel(qts, CTRL(p), FUNC_SIO | OEOVER(OVER_HIGH));
    g_assert_true(gpio_in(qts, p));

    /* The pad's output disable beats any output enable. */
    qtest_writel(qts, PAD(p), PAD_IE | PAD_OD);
    g_assert_false(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)) & STATUS_OETOPAD, ==,
                    STATUS_OETOPAD);

    /* A null function drives nothing even with overrides off. */
    qtest_writel(qts, PAD(p), PAD_IE | PAD_PUE);
    qtest_writel(qts, CTRL(p), FUNC_NULL);
    drive(qts, p, -1);
    qtest_writel(qts, SIO_GPIO_OUT, 0);
    qtest_writel(qts, SIO_GPIO_OE, 1u << p);
    g_assert_true(gpio_in(qts, p));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_isolation(void)
{
    QTestState *qts = start();
    int p = 11;

    qtest_writel(qts, CTRL(p), FUNC_SIO);
    qtest_writel(qts, SIO_GPIO_OE, 1u << p);
    qtest_writel(qts, SIO_GPIO_OUT, 1u << p);
    pad_open(qts, p, PAD_PDE);
    g_assert_true(gpio_in(qts, p));

    /* Isolated, the pad holds the controls it had; input still flows. */
    qtest_writel(qts, PAD(p) + ALIAS_SET, PAD_ISO);
    qtest_writel(qts, SIO_GPIO_OUT, 0);
    g_assert_true(gpio_in(qts, p));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)) & STATUS_OUTTOPAD, ==, 0);
    qtest_writel(qts, SIO_GPIO_OE, 0);
    qtest_writel(qts, PAD(p) + ALIAS_CLR, PAD_IE);
    g_assert_true(gpio_in(qts, p));

    /* Releasing isolation applies the current controls. */
    qtest_writel(qts, PAD(p) + ALIAS_SET, PAD_IE);
    qtest_writel(qts, PAD(p) + ALIAS_CLR, PAD_ISO);
    g_assert_false(gpio_in(qts, p));

    /* Input from the pad is not isolated. */
    qtest_writel(qts, PAD(p) + ALIAS_SET, PAD_ISO);
    drive(qts, p, 1);
    g_assert_true(gpio_in(qts, p));

    /* A system reset puts the latches back to the reset controls. */
    qtest_system_reset(qts);
    g_assert_false(gpio_in(qts, p));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
static void test_peripheral_routing(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "uart0-in");

    /* UART0 RX is function 2 on GPIO 1 and GPIO 13. */
    pad_open(qts, 1, 0);
    pad_open(qts, 13, 0);
    drive(qts, 1, 1);
    drive(qts, 13, 0);
    g_assert_false(qtest_get_irq(qts, UART_RX));
    qtest_writel(qts, CTRL(1), FUNC_UART);
    g_assert_true(qtest_get_irq(qts, UART_RX));
    qtest_writel(qts, CTRL(1), FUNC_SPI);
    g_assert_false(qtest_get_irq(qts, UART_RX));

    /* Two pins selecting one input give their OR. */
    qtest_writel(qts, CTRL(1), FUNC_UART);
    qtest_writel(qts, CTRL(13), FUNC_UART);
    g_assert_true(qtest_get_irq(qts, UART_RX));
    drive(qts, 1, 0);
    g_assert_false(qtest_get_irq(qts, UART_RX));
    drive(qts, 13, 1);
    g_assert_true(qtest_get_irq(qts, UART_RX));
    qtest_writel(qts, CTRL(13), FUNC_UART | INOVER(OVER_INVERT));
    g_assert_false(qtest_get_irq(qts, UART_RX));

    /*
     * UART0 TX on GPIO 0: QEMU's PL011 has no line-level TX, so the pin
     * shows the idle line, driven high.
     */
    pad_open(qts, 0, PAD_PDE);
    qtest_writel(qts, CTRL(0), FUNC_UART);
    g_assert_cmphex(qtest_readl(qts, STATUS(0)) &
                    (STATUS_OUTTOPAD | STATUS_OETOPAD), ==,
                    STATUS_OUTTOPAD | STATUS_OETOPAD);
    g_assert_true(gpio_in(qts, 0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_edge_interrupts(void)
{
    QTestState *qts = start_on_core(0);
    int p = 4;

    pad_open(qts, p, 0);
    drive(qts, p, 0);
    qtest_writel(qts, INTR(0), 0xffffffff);
    g_assert_cmphex(int_field(qts, INTR(0), p), ==, LEVEL_LOW);

    drive(qts, p, 1);
    g_assert_cmphex(int_field(qts, INTR(0), p), ==, LEVEL_HIGH | EDGE_HIGH);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));

    qtest_writel(qts, PROC0_INTE(0), EDGE_HIGH << (4 * p));
    g_assert_cmphex(int_field(qts, PROC0_INTS(0), p), ==, EDGE_HIGH);
    g_assert_cmphex(qtest_readl(qts, IRQSUMMARY_PROC0_SECURE0), ==, 1u << p);
    g_assert_cmphex(qtest_readl(qts, IRQSUMMARY_PROC0_NONSECURE0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IRQSUMMARY_PROC1_SECURE0), ==, 0);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0_NS));

    /* Edges stay latched until written 1; CLR and XOR aliases don't. */
    drive(qts, p, 0);
    g_assert_cmphex(int_field(qts, INTR(0), p), ==,
                    LEVEL_LOW | EDGE_LOW | EDGE_HIGH);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_writel(qts, INTR(0) + ALIAS_CLR, EDGE_HIGH << (4 * p));
    qtest_writel(qts, INTR(0) + ALIAS_XOR, EDGE_HIGH << (4 * p));
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_writel(qts, INTR(0) + ALIAS_SET, EDGE_HIGH << (4 * p));
    g_assert_cmphex(int_field(qts, INTR(0), p), ==, LEVEL_LOW | EDGE_LOW);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));
    /* Level bits are read-only. */
    qtest_writel(qts, INTR(0), 0xffffffff);
    g_assert_cmphex(int_field(qts, INTR(0), p), ==, LEVEL_LOW);

    /* IRQOVER acts on the interrupt, after the pad. */
    qtest_writel(qts, CTRL(p), FUNC_NULL | IRQOVER(OVER_INVERT));
    g_assert_cmphex(int_field(qts, INTR(0), p), ==, LEVEL_HIGH | EDGE_HIGH);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    g_assert_cmphex(qtest_readl(qts, STATUS(p)), ==, STATUS_IRQTOPROC);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_level_interrupts(void)
{
    QTestState *qts = start_on_core(0);
    int p = 20;

    pad_open(qts, p, 0);
    drive(qts, p, 0);
    qtest_writel(qts, PROC0_INTE(2), LEVEL_HIGH << (4 * (p % 8)));
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));
    drive(qts, p, 1);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    /* Not latched: writing INTR does not clear it, the level does. */
    qtest_writel(qts, INTR(2), 0xffffffff);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    drive(qts, p, 0);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));

    /* Forcing raises the interrupt where it is enabled. */
    qtest_writel(qts, PROC0_INTE(2), EDGE_LOW << (4 * (p % 8)));
    qtest_writel(qts, INTR(2), 0xffffffff);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_writel(qts, PROC0_INTF(2), EDGE_LOW << (4 * (p % 8)));
    g_assert_cmphex(int_field(qts, PROC0_INTS(2), p), ==, EDGE_LOW);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_writel(qts, PROC0_INTF(2), 0);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_interrupts_are_core_local(void)
{
    QTestState *qts = start_on_core(1);
    int p = 2;

    pad_open(qts, p, 0);
    drive(qts, p, 1);

    /* Core 0's enable does not reach core 1. */
    qtest_writel(qts, PROC0_INTE(0), LEVEL_HIGH << (4 * p));
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));
    g_assert_cmphex(int_field(qts, PROC1_INTS(0), p), ==, 0);

    qtest_writel(qts, PROC1_INTE(0), LEVEL_HIGH << (4 * p));
    g_assert_cmphex(int_field(qts, PROC1_INTS(0), p), ==, LEVEL_HIGH);
    g_assert_cmphex(qtest_readl(qts, IRQSUMMARY_PROC1_SECURE0), ==, 1u << p);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_writel(qts, PROC0_INTE(0), 0);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_writel(qts, PROC1_INTE(0), 0);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));

    /* Dormant wake has its own enables and status, and no core IRQ. */
    qtest_writel(qts, DORMANT_WAKE_INTE(0), LEVEL_HIGH << (4 * p));
    g_assert_cmphex(int_field(qts, DORMANT_WAKE_INTS(0), p), ==, LEVEL_HIGH);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.gpio/test] */
/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_qspi_bank(void)
{
    QTestState *qts = start_on_core(0);
    int sd2 = 48 + QSPI_PIN_SD2;

    /* SD2 has its input enabled and pulls up at reset. */
    qtest_writel(qts, PADS_QSPI + 0x10 + ALIAS_CLR, PAD_ISO);
    g_assert_cmphex(qtest_readl(qts, SIO_GPIO_HI_IN) >> 30 & 1, ==, 1);
    qtest_writel(qts, QSPI_PROC0_INTE, LEVEL_HIGH << (4 * QSPI_PIN_SD2));
    g_assert_cmphex(qtest_readl(qts, QSPI_PROC0_INTS), ==,
                    LEVEL_HIGH << (4 * QSPI_PIN_SD2));
    g_assert_cmphex(qtest_readl(qts, QSPI_SUMMARY_PROC0_SECURE), ==,
                    1u << QSPI_PIN_SD2);
    g_assert_true(qtest_get_irq(qts, IO_IRQ_QSPI));
    g_assert_false(qtest_get_irq(qts, IO_IRQ_BANK0));

    qtest_writel(qts, QSPI_INTR, 0xffffffff);
    drive(qts, sd2, 0);
    g_assert_false(qtest_get_irq(qts, IO_IRQ_QSPI));
    g_assert_cmphex(qtest_readl(qts, QSPI_INTR) >> (4 * QSPI_PIN_SD2) & 0xf,
                    ==, LEVEL_LOW | EDGE_LOW);

    /* SIO drives SD2 as GPIO 62 (SIO_HI bit 30). */
    drive(qts, sd2, -1);
    qtest_writel(qts, QSPI_CTRL(QSPI_PIN_SD2), FUNC_SIO);
    qtest_writel(qts, SIO_GPIO_OE + 4, 1u << 30);
    g_assert_cmphex(qtest_readl(qts, SIO_GPIO_HI_IN) >> 30 & 1, ==, 0);
    qtest_writel(qts, SIO_GPIO_OUT + 4, 1u << 30);
    g_assert_cmphex(qtest_readl(qts, SIO_GPIO_HI_IN) >> 30 & 1, ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-gpio-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/gpio/reset-values", test_reset_values);
    qtest_add_func("/rp2350/gpio/atomic-aliases", test_atomic_aliases);
    qtest_add_func("/rp2350/gpio/input-path", test_input_path);
    qtest_add_func("/rp2350/gpio/unbonded-pins", test_unbonded_pins);
    qtest_add_func("/rp2350/gpio/output-path", test_output_path);
    qtest_add_func("/rp2350/gpio/isolation", test_isolation);
    qtest_add_func("/rp2350/gpio/peripheral-routing",
                   test_peripheral_routing);
    qtest_add_func("/rp2350/gpio/edge-interrupts", test_edge_interrupts);
    qtest_add_func("/rp2350/gpio/level-interrupts", test_level_interrupts);
    qtest_add_func("/rp2350/gpio/core-local", test_interrupts_are_core_local);
    qtest_add_func("/rp2350/gpio/qspi-bank", test_qspi_bank);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
