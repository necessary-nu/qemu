/*
 * QTest testcase for the ESP32-P4 analog voltage comparators
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The comparators' pads, GPIO51 to GPIO54, take their voltages from the
 * ADC's properties (ADC2 channels 2 to 5) or from a digital driver; their
 * crossings land in GPIO_EXT's interrupt registers.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define LP_SYS              0x50110000u
#define PAD_COMP(n)         (LP_SYS + 0x148 + 4 * (n))
#define DREF(d)             (d)
#define MODE_EXT            (1u << 3)
#define XPD                 (1u << 4)

#define GPIO                0x500e0000u
#define GPIO_INT_RAW        (GPIO + 0x700)
#define GPIO_INT_ST         (GPIO + 0x704)
#define GPIO_INT_ENA        (GPIO + 0x708)
#define GPIO_INT_CLR        (GPIO + 0x70c)
#define ZERO_DET(n)         (GPIO + 0x710 + 4 * (n))
/* GPIO_COMPn_NEG_INT, _POS_INT and _ALL_INT. */
#define NEG(n)              (1u << (3 * (n)))
#define POS(n)              (1u << (3 * (n) + 1))
#define ALL(n)              (1u << (3 * (n) + 2))
#define GPIO_PATH           "/machine/gpio"
#define ADC_PATH            "/machine/adc"

#define INTMTX_STATUS2      0x500d6208u
#define SRC_PAD_COMP_BIT    (1u << (78 - 64))

static QTestState *qts;

static uint32_t rd(uint32_t a)
{
    return qtest_readl(qts, a);
}

static void wr(uint32_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static void set_mv(const char *prop, uint32_t mv)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': %s, 'property': %s, 'value': %u } }",
                         ADC_PATH, prop, mv);

    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
}

/* Past the debounce of the last crossing. */
static void settle(void)
{
    qtest_clock_step(qts, 1000);
}

static void start(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    /* ESP-IDF's debounce, which the reset value's 2^32 cycles defeat. */
    wr(ZERO_DET(0), 1);
    wr(ZERO_DET(1), 1);
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

/*
 * Comparator 0 against its internal reference, tenths of VDD_IO_6: the
 * source above it raises POS and ALL, below it NEG; powered down, it is
 * low; the interrupt reaches the interrupt matrix's GPIO_PAD_COMP_INTR.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_internal_reference(void)
{
    start();
    set_mv("adc2-ch3-mv", 1500);
    wr(GPIO_INT_ENA, POS(0) | NEG(0));
    wr(PAD_COMP(0), DREF(3));
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, 0);
    wr(PAD_COMP(0), DREF(3) | XPD);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, POS(0) | ALL(0));
    g_assert_cmphex(rd(GPIO_INT_ST), ==, POS(0));
    g_assert_true(rd(INTMTX_STATUS2) & SRC_PAD_COMP_BIT);
    wr(GPIO_INT_CLR, 0x3f);
    settle();
    /* 0.5 VDD_IO_6, 1650 mV: above the source. */
    wr(PAD_COMP(0), DREF(5) | XPD);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, NEG(0) | ALL(0));
    wr(GPIO_INT_CLR, 0x3f);
    /* The source set from QEMU crosses it. */
    settle();
    set_mv("adc2-ch3-mv", 1700);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, POS(0) | ALL(0));
    wr(GPIO_INT_CLR, 0x3f);
    settle();
    set_mv("adc2-ch3-mv", 1649);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, NEG(0) | ALL(0));
    wr(GPIO_INT_CLR, 0x3f);
    /* Powered down while high: COMP_OUT falls. */
    settle();
    set_mv("adc2-ch3-mv", 3000);
    wr(GPIO_INT_CLR, 0x3f);
    settle();
    wr(PAD_COMP(0), DREF(5));
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, NEG(0) | ALL(0));
    stop();
}

/* Comparator 1 against its external reference on GPIO53. */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_external_reference(void)
{
    start();
    set_mv("adc2-ch4-mv", 1000);
    set_mv("adc2-ch5-mv", 800);
    wr(PAD_COMP(1), MODE_EXT | XPD);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, 0);
    set_mv("adc2-ch5-mv", 1200);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, POS(1) | ALL(1));
    wr(GPIO_INT_CLR, 0x3f);
    settle();
    set_mv("adc2-ch4-mv", 1300);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, NEG(1) | ALL(1));
    stop();
}

/*
 * A source pad driven digitally, here from outside the chip, is at the
 * supply or ground; GPIO_ZERO_DET1's debounce holds back an edge too soon
 * after the last.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_digital_source_and_debounce(void)
{
    start();
    set_mv("adc2-ch5-mv", 2000);
    wr(PAD_COMP(1), DREF(5) | XPD);
    wr(GPIO_INT_CLR, 0x3f);
    settle();
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 54, 0);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, NEG(1) | ALL(1));
    wr(GPIO_INT_CLR, 0x3f);
    settle();
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 54, 1);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, POS(1) | ALL(1));
    wr(GPIO_INT_CLR, 0x3f);
    /* Released, the pad is back at its analog voltage: still high. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-release", 54, 1);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, 0);
    /* 1000 cycles of the IO MUX clock between interrupts. */
    wr(ZERO_DET(1), 1000);
    qtest_clock_step(qts, 1000000);
    set_mv("adc2-ch5-mv", 1000);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, NEG(1) | ALL(1));
    wr(GPIO_INT_CLR, 0x3f);
    set_mv("adc2-ch5-mv", 2000);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, 0);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/ana-cmpr/internal_reference",
                   test_internal_reference);
    qtest_add_func("esp32p4/ana-cmpr/external_reference",
                   test_external_reference);
    qtest_add_func("esp32p4/ana-cmpr/digital_source_and_debounce",
                   test_digital_source_and_debounce);
    return g_test_run();
}
