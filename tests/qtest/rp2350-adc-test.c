/*
 * QTest testcase for the RP2350 ADC and temperature sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * clk_adc is 48 MHz: a conversion is 96 cycles, 2 us, and 6 cycles are
 * exactly 125 ns. Every test starts at virtual time 0, on a cycle edge.
 * Input voltages are set with qom-set on the ADC's "ain<n>-uv",
 * "temperature" and "conversion-errors" properties.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define ALIAS_XOR       0x1000
#define ALIAS_SET       0x2000
#define ALIAS_CLR       0x3000

#define ADC             0x400a0000
#define CS              (ADC + 0x00)
#define RESULT          (ADC + 0x04)
#define FCS             (ADC + 0x08)
#define FIFO            (ADC + 0x0c)
#define DIV             (ADC + 0x10)
#define INTR            (ADC + 0x14)
#define INTE            (ADC + 0x18)
#define INTF            (ADC + 0x1c)
#define INTS            (ADC + 0x20)

#define CS_EN           (1u << 0)
#define CS_TS_EN        (1u << 1)
#define CS_START_ONCE   (1u << 2)
#define CS_START_MANY   (1u << 3)
#define CS_READY        (1u << 8)
#define CS_ERR          (1u << 9)
#define CS_ERR_STICKY   (1u << 10)
#define CS_AINSEL(n)    ((n) << 12)
#define CS_RROBIN(m)    ((m) << 16)

#define FCS_EN          (1u << 0)
#define FCS_SHIFT       (1u << 1)
#define FCS_ERR         (1u << 2)
#define FCS_DREQ_EN     (1u << 3)
#define FCS_EMPTY       (1u << 8)
#define FCS_FULL        (1u << 9)
#define FCS_UNDER       (1u << 10)
#define FCS_OVER        (1u << 11)
#define FCS_LEVEL(v)    (((v) >> 16) & 0xf)
#define FCS_THRESH(n)   ((n) << 24)

#define FIFO_ERR        (1u << 15)

#define IO_BANK0        0x40028000
#define PADS_BANK0      0x40038000
#define CTRL(p)         (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define PAD_IE          (1u << 6)
#define PAD_OD          (1u << 7)
#define FUNC_SIO        5
#define SIO_GPIO_OUT_SET 0xd0000018
#define SIO_GPIO_OUT_CLR 0xd0000020
#define SIO_GPIO_OE_SET 0xd0000038

#define ADC_IRQ_FIFO    35

#define DMA             0x50000000
#define DMA_READ_ADDR   (DMA + 0x00)
#define DMA_WRITE_ADDR  (DMA + 0x04)
#define DMA_TRANS_COUNT (DMA + 0x08)
#define DMA_CTRL_TRIG   (DMA + 0x0c)
#define DMA_EN          (1u << 0)
#define DMA_INCR_WRITE  (1u << 6)
#define DMA_TREQ(n)     ((n) << 17)
#define DMA_BUSY        (1u << 26)
#define DREQ_ADC        48
#define SRAM_BUF        0x20001000

#define CONV_NS         2000
/* clk_adc cycles to ns, for whole multiples of 6 cycles. */
#define CYC_NS(c)       ((c) * 125 / 6)

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    return qts;
}

static void set_prop(QTestState *qts, const char *prop, int64_t value)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                           "{ 'path': '/machine/soc/adc', 'property': %s, "
                           "'value': %" PRId64 " } }", prop, value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

/* The code of `uv` microvolts against the 3.3 V reference. */
static uint32_t code_of(int64_t uv)
{
    return uv * 4096 / 3300000;
}

static uint32_t convert_once(QTestState *qts, uint32_t cs)
{
    qtest_writel(qts, CS, cs | CS_EN | CS_START_ONCE);
    g_assert_false(qtest_readl(qts, CS) & CS_READY);
    qtest_clock_step(qts, CONV_NS);
    g_assert_true(qtest_readl(qts, CS) & CS_READY);
    return qtest_readl(qts, RESULT);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, FCS), ==, FCS_EMPTY);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, 0);

    /* Reserved, read-only and self-clearing bits read zero. */
    qtest_writel(qts, CS, 0xfffff8f2);
    g_assert_cmphex(qtest_readl(qts, CS), ==, 0x01fff002);
    qtest_writel(qts, CS, 0);
    qtest_writel(qts, FCS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FCS), ==, 0x0f00010f);
    qtest_writel(qts, DIV, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0x00ffffff);
    qtest_writel(qts, INTE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INTE), ==, 1);
    qtest_writel(qts, INTF, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INTF), ==, 1);
    qtest_writel(qts, RESULT, 0xfff);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, 0);

    /* Atomic aliases. */
    qtest_writel(qts, DIV, 0x123456);
    qtest_writel(qts, DIV + ALIAS_XOR, 0x0000ff);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0x1234a9);
    qtest_writel(qts, DIV + ALIAS_CLR, 0xff0000);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0x0034a9);
    qtest_writel(qts, DIV + ALIAS_SET, 0x010000);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0x0134a9);

    /* A narrow write is replicated across the register. */
    qtest_writeb(qts, DIV + 1, 0x12);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0x121212);
    qtest_writew(qts, DIV, 0x0034);
    g_assert_cmphex(qtest_readl(qts, DIV), ==, 0x340034);
    g_assert_cmphex(qtest_readb(qts, DIV + 2), ==, 0x34);
    qtest_writel(qts, DIV, 0);

    qtest_writel(qts, CS + ALIAS_SET, CS_AINSEL(3));
    qtest_writel(qts, CS + ALIAS_CLR, CS_AINSEL(1));
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_AINSEL(2));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_one_shot(void)
{
    QTestState *qts = start();

    /* Disabled, the ADC is not ready and ignores START_ONCE. */
    qtest_writel(qts, CS, CS_START_ONCE);
    g_assert_cmphex(qtest_readl(qts, CS), ==, 0);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, CS), ==, 0);

    qtest_writel(qts, CS, CS_EN);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN | CS_READY);

    set_prop(qts, "ain0-uv", 1650000);
    set_prop(qts, "ain1-uv", 1000000);
    set_prop(qts, "ain3-uv", 3300000);

    /* 96 cycles of clk_adc. */
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN);
    qtest_clock_step(qts, CONV_NS - 1);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN | CS_READY);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, 2048);

    g_assert_cmphex(convert_once(qts, CS_AINSEL(1)), ==, code_of(1000000));
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, 0);
    /* At or above the reference the code saturates. */
    g_assert_cmphex(convert_once(qts, CS_AINSEL(3)), ==, 0xfff);
    set_prop(qts, "ain3-uv", 5000000);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(3)), ==, 0xfff);
    set_prop(qts, "ain3-uv", -100000);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(3)), ==, 0);

    /* The input is sampled when the conversion starts. */
    qtest_writel(qts, CS, CS_EN | CS_AINSEL(0) | CS_START_ONCE);
    set_prop(qts, "ain0-uv", 0);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, 2048);

    /* START_ONCE during a conversion does not restart it. */
    set_prop(qts, "ain0-uv", 1000000);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS / 2);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS / 2);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN | CS_READY);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, code_of(1000000));

    /* Clearing EN abandons a conversion. */
    set_prop(qts, "ain0-uv", 2000000);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS / 2);
    qtest_writel(qts, CS, 0);
    qtest_clock_step(qts, CONV_NS);
    qtest_writel(qts, CS, CS_EN);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN | CS_READY);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, code_of(1000000));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_pins(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CS, CS_EN);
    set_prop(qts, "ain2-uv", 1000000);

    /* Driven from outside, the pin is at 0 V or IOVDD. */
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", 28, 1);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, 0xfff);
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", 28, 0);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, 0);
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", 28, -1);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, code_of(1000000));

    /* So it is when the chip drives it, until OD disables the driver. */
    qtest_writel(qts, PAD(28), 0);
    qtest_writel(qts, CTRL(28), FUNC_SIO);
    qtest_writel(qts, SIO_GPIO_OE_SET, 1u << 28);
    qtest_writel(qts, SIO_GPIO_OUT_SET, 1u << 28);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, 0xfff);
    qtest_writel(qts, SIO_GPIO_OUT_CLR, 1u << 28);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, 0);
    qtest_writel(qts, PAD(28), PAD_OD);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, code_of(1000000));

    /* The digital input buffer does not change the reading. */
    qtest_writel(qts, PAD(28), PAD_OD | PAD_IE);
    g_assert_cmphex(convert_once(qts, CS_AINSEL(2)), ==, code_of(1000000));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_temperature(void)
{
    QTestState *qts = start();

    /* Without its bias the sensor reads nothing. */
    g_assert_cmphex(convert_once(qts, CS_AINSEL(4)), ==, 0);

    /* Vbe = 0.706 V at 27 C, -1.721 mV per degree. */
    g_assert_cmphex(convert_once(qts, CS_TS_EN | CS_AINSEL(4)), ==,
                    code_of(706000));
    g_assert_cmphex(convert_once(qts, CS_TS_EN | CS_AINSEL(4)), ==, 876);
    set_prop(qts, "temperature", 50000);
    g_assert_cmphex(convert_once(qts, CS_TS_EN | CS_AINSEL(4)), ==,
                    code_of(706000 - 23 * 1721));
    set_prop(qts, "temperature", -10500);
    g_assert_cmphex(convert_once(qts, CS_TS_EN | CS_AINSEL(4)), ==,
                    code_of(706000 + 37500 * 1721 / 1000));
    /* The datasheet's example: 891 is 20.1 C. */
    set_prop(qts, "temperature", 20100);
    g_assert_cmphex(convert_once(qts, CS_TS_EN | CS_AINSEL(4)), ==, 891);

    /* Channels past the temperature sensor are not bonded out. */
    g_assert_cmphex(convert_once(qts, CS_TS_EN | CS_AINSEL(5)), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_fifo_irq(void)
{
    QTestState *qts = start();
    uint32_t fcs;
    int i;

    set_prop(qts, "ain1-uv", 1000000);
    qtest_writel(qts, FCS, FCS_EN | FCS_THRESH(2));
    qtest_writel(qts, INTE, 1);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    g_assert_false(qtest_get_irq(qts, ADC_IRQ_FIFO));

    /* Free-running with DIV 0: back to back, one sample per 2 us. */
    qtest_writel(qts, CS, CS_EN | CS_AINSEL(1) | CS_START_MANY);
    g_assert_cmphex(qtest_readl(qts, CS) & CS_READY, ==, 0);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 1);
    g_assert_false(qtest_get_irq(qts, ADC_IRQ_FIFO));
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 2);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, 1);
    g_assert_true(qtest_get_irq(qts, ADC_IRQ_FIFO));

    /* Draining below THRESH clears the interrupt. */
    g_assert_cmphex(qtest_readl(qts, FIFO), ==, code_of(1000000));
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    g_assert_false(qtest_get_irq(qts, ADC_IRQ_FIFO));

    /* INTF forces it. */
    qtest_writel(qts, INTE, 0);
    qtest_writel(qts, INTF, 1);
    g_assert_true(qtest_get_irq(qts, ADC_IRQ_FIFO));
    qtest_writel(qts, INTF, 0);
    g_assert_false(qtest_get_irq(qts, ADC_IRQ_FIFO));

    /* Eight samples fill the FIFO; more are lost, and set OVER. */
    qtest_clock_step(qts, 9 * CONV_NS);
    fcs = qtest_readl(qts, FCS);
    g_assert_cmphex(FCS_LEVEL(fcs), ==, 8);
    g_assert_cmphex(fcs & (FCS_FULL | FCS_OVER | FCS_EMPTY), ==,
                    FCS_FULL | FCS_OVER);

    /* Stop: the conversion in progress completes, then READY. */
    qtest_writel(qts, CS + ALIAS_CLR, CS_START_MANY);
    g_assert_cmphex(qtest_readl(qts, CS) & CS_READY, ==, 0);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, CS) & CS_READY, ==, CS_READY);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, code_of(1000000));

    for (i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, FIFO), ==, code_of(1000000));
    }
    fcs = qtest_readl(qts, FCS);
    g_assert_cmphex(fcs & (FCS_EMPTY | FCS_UNDER | FCS_OVER), ==,
                    FCS_EMPTY | FCS_OVER);
    qtest_readl(qts, FIFO);
    g_assert_cmphex(qtest_readl(qts, FCS) & FCS_UNDER, ==, FCS_UNDER);

    /* OVER and UNDER are write-1-to-clear. */
    qtest_writel(qts, FCS + ALIAS_XOR, FCS_OVER);
    g_assert_cmphex(qtest_readl(qts, FCS) & FCS_OVER, ==, FCS_OVER);
    qtest_writel(qts, FCS + ALIAS_SET, FCS_OVER);
    g_assert_cmphex(qtest_readl(qts, FCS) & FCS_OVER, ==, 0);
    qtest_writel(qts, FCS, FCS_EN | FCS_THRESH(2) | FCS_UNDER);
    g_assert_cmphex(qtest_readl(qts, FCS), ==,
                    FCS_EN | FCS_THRESH(2) | FCS_EMPTY);

    /* A one-shot conversion fills the FIFO too. */
    qtest_writel(qts, CS, CS_EN | CS_AINSEL(1) | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_shift_and_errors(void)
{
    QTestState *qts = start();
    uint32_t code = code_of(2500000);

    set_prop(qts, "ain0-uv", 2500000);
    qtest_writel(qts, FCS, FCS_EN | FCS_SHIFT);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, code);
    g_assert_cmphex(qtest_readl(qts, FIFO), ==, code >> 4);

    /* A byte read of FIFO, as DMA to a byte buffer makes, pops a sample. */
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readb(qts, FIFO), ==, code >> 4);
    g_assert_cmphex(qtest_readl(qts, FCS) & FCS_EMPTY, ==, FCS_EMPTY);

    /* A conversion that fails to converge sets ERR and ERR_STICKY. */
    set_prop(qts, "conversion-errors", 1);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, CS), ==,
                    CS_EN | CS_READY | CS_ERR | CS_ERR_STICKY);
    /* Without FCS.ERR, the FIFO does not carry the flag. */
    g_assert_cmphex(qtest_readl(qts, FIFO), ==, code >> 4);

    qtest_writel(qts, FCS, FCS_EN | FCS_ERR);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, FIFO), ==, code | FIFO_ERR);

    set_prop(qts, "conversion-errors", 0);
    qtest_writel(qts, CS, CS_EN | CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN | CS_READY |
                    CS_ERR_STICKY);
    g_assert_cmphex(qtest_readl(qts, FIFO), ==, code);
    qtest_writel(qts, CS, CS_EN | CS_ERR_STICKY);
    g_assert_cmphex(qtest_readl(qts, CS), ==, CS_EN | CS_READY);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_round_robin(void)
{
    static const uint32_t order[] = { 0, 1, 2, 1, 2, 1, 2 };
    QTestState *qts = start();
    uint32_t uv[3] = { 500000, 1500000, 2500000 };
    int i;

    for (i = 0; i < 3; i++) {
        g_autofree char *name = g_strdup_printf("ain%d-uv", i);

        set_prop(qts, name, uv[i]);
    }
    qtest_writel(qts, FCS, FCS_EN);

    /* The datasheet's example: AINSEL 0, RROBIN 0x06. */
    qtest_writel(qts, CS, CS_EN | CS_AINSEL(0) | CS_RROBIN(6) |
                 CS_START_MANY);
    qtest_clock_step(qts, CONV_NS / 2);
    g_assert_cmphex(qtest_readl(qts, CS) & 0xf000, ==, CS_AINSEL(0));
    qtest_clock_step(qts, CONV_NS / 2);
    g_assert_cmphex(qtest_readl(qts, CS) & 0xf000, ==, CS_AINSEL(1));
    qtest_clock_step(qts, 6 * CONV_NS);
    qtest_writel(qts, CS + ALIAS_CLR, CS_START_MANY);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 8);
    for (i = 0; i < ARRAY_SIZE(order); i++) {
        g_assert_cmphex(qtest_readl(qts, FIFO), ==, code_of(uv[order[i]]));
    }

    /* One-shot conversions step round too. */
    qtest_readl(qts, FIFO);
    qtest_writel(qts, CS, CS_EN | CS_AINSEL(2) | CS_RROBIN(5));
    qtest_writel(qts, CS + ALIAS_SET, CS_START_ONCE);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, CS) & 0xf000, ==, CS_AINSEL(0));
    g_assert_cmphex(qtest_readl(qts, RESULT), ==, code_of(uv[2]));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.adc/test] */
static void test_pacing(void)
{
    QTestState *qts = start();
    uint32_t level;

    qtest_writel(qts, FCS, FCS_EN);

    /*
     * DIV.INT 479: one conversion per 480 cycles (10 us), the first as
     * START_MANY is set, with the ADC ready in between.
     */
    qtest_writel(qts, DIV, 479 << 8);
    qtest_writel(qts, CS, CS_EN | CS_START_MANY);
    qtest_clock_step(qts, CONV_NS);
    g_assert_cmphex(qtest_readl(qts, CS) & CS_READY, ==, CS_READY);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 1);
    qtest_clock_step(qts, 10000 - CONV_NS - 1);
    g_assert_cmphex(qtest_readl(qts, CS) & CS_READY, ==, CS_READY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, CS) & CS_READY, ==, 0);
    qtest_clock_step(qts, CONV_NS - 1);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 2);
    qtest_writel(qts, CS, CS_EN);
    qtest_quit(qts);

    /*
     * DIV 99 + 128/256: ticks at floor(100.5 k) = 0, 100, 201, 301, 402,
     * 502, 603, 703, 804, 904, 1005, ...; each conversion ends 96 cycles
     * after its tick.
     */
    qts = start();
    qtest_writel(qts, FCS, FCS_EN);
    qtest_writel(qts, DIV, (99 << 8) | 128);
    qtest_writel(qts, CS, CS_EN | CS_START_MANY);
    qtest_clock_step(qts, CYC_NS(600));
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 6);
    while (!(qtest_readl(qts, FCS) & FCS_EMPTY)) {
        qtest_readl(qts, FIFO);
    }
    qtest_clock_step(qts, CYC_NS(1098 - 600));
    level = FCS_LEVEL(qtest_readl(qts, FCS));
    g_assert_cmphex(level, ==, 4);
    qtest_clock_step(qts, CYC_NS(6));
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 5);
    qtest_quit(qts);
}


/*
 * Free-running at the full 500 kS/s, alternating between channels 0 and
 * the temperature sensor, the DMA carries byte samples to memory on
 * DREQ_ADC without losing one.
 */
/* [spec:nuos:req:emu.adc/test] */
static void test_dma(void)
{
    QTestState *qts = start();
    uint8_t buf[32];
    uint8_t ch0 = code_of(3000000) >> 4, temp = code_of(706000) >> 4;
    int i;

    set_prop(qts, "ain0-uv", 3000000);
    qtest_memset(qts, SRAM_BUF, 0xa5, sizeof(buf));
    qtest_writel(qts, FCS, FCS_EN | FCS_SHIFT | FCS_DREQ_EN | FCS_THRESH(1));
    qtest_writel(qts, DMA_READ_ADDR, FIFO);
    qtest_writel(qts, DMA_WRITE_ADDR, SRAM_BUF);
    qtest_writel(qts, DMA_TRANS_COUNT, 30);
    qtest_writel(qts, DMA_CTRL_TRIG, DMA_EN | DMA_INCR_WRITE |
                 DMA_TREQ(DREQ_ADC));
    qtest_writel(qts, CS, CS_EN | CS_TS_EN | CS_RROBIN(0x11) |
                 CS_START_MANY);

    /* The DREQ follows the FIFO: one sample, one transfer. */
    qtest_clock_step(qts, CONV_NS + 500);
    g_assert_cmphex(qtest_readl(qts, DMA_TRANS_COUNT), ==, 29);
    g_assert_cmphex(qtest_readl(qts, FCS) & FCS_EMPTY, ==, FCS_EMPTY);

    qtest_clock_step(qts, 30 * CONV_NS);
    g_assert_cmphex(qtest_readl(qts, DMA_TRANS_COUNT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DMA_CTRL_TRIG) & DMA_BUSY, ==, 0);
    qtest_memread(qts, SRAM_BUF, buf, sizeof(buf));
    for (i = 0; i < 30; i++) {
        g_assert_cmphex(buf[i], ==, i % 2 ? temp : ch0);
    }
    g_assert_cmphex(buf[30], ==, 0xa5);
    g_assert_cmphex(qtest_readl(qts, FCS) & FCS_OVER, ==, 0);

    /* Without the DMA reading, the DREQ does not touch the FIFO. */
    qtest_writel(qts, FCS + ALIAS_CLR, FCS_DREQ_EN);
    qtest_clock_step(qts, 8 * CONV_NS);
    g_assert_cmphex(FCS_LEVEL(qtest_readl(qts, FCS)), ==, 8);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-adc-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/adc/reset-values", test_reset_values);
    qtest_add_func("/rp2350/adc/one-shot", test_one_shot);
    qtest_add_func("/rp2350/adc/pins", test_pins);
    qtest_add_func("/rp2350/adc/temperature", test_temperature);
    qtest_add_func("/rp2350/adc/fifo-irq", test_fifo_irq);
    qtest_add_func("/rp2350/adc/shift-and-errors", test_shift_and_errors);
    qtest_add_func("/rp2350/adc/round-robin", test_round_robin);
    qtest_add_func("/rp2350/adc/pacing", test_pacing);
    qtest_add_func("/rp2350/adc/dma", test_dma);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
