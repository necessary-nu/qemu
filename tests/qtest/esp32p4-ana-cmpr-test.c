/*
 * QTest testcase for the ESP32-P4 analog voltage comparators
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The comparators' pads, GPIO51 to GPIO54, take their voltages from the
 * ADC's properties (ADC2 channels 2 to 5) or from a digital driver; their
 * crossings land in GPIO_EXT's interrupt registers and come to the ETM as
 * events, each checked as ESP-IDF's driver programs and reads it.
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
#define GPIO_OUT1_W1TS      (GPIO + 0x014)
#define GPIO_OUT1_W1TC      (GPIO + 0x018)
#define GPIO_ENABLE1_W1TS   (GPIO + 0x030)
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define GPIO_INT_RAW        (GPIO + 0x700)
#define GPIO_INT_ST         (GPIO + 0x704)
#define GPIO_INT_ENA        (GPIO + 0x708)
#define GPIO_INT_CLR        (GPIO + 0x70c)
#define ZERO_DET(n)         (GPIO + 0x710 + 4 * (n))
#define SIG_GPIO_OUT        0x100
#define IO_MUX_GPIO(n)      (0x500e1004u + 4 * (n))
#define FUN_DRV2            (2u << 10)
#define MCU_SEL_GPIO        (1u << 12)
#define GPIO_PATH           "/machine/gpio"
#define ADC_PATH            "/machine/adc"

/*
 * ESP-IDF's hal/esp32p4 ana_cmpr_ll.h for unit n: a positive crossing,
 * the source rising above the reference, is bit 3n of GPIO_INT_*
 * (ANALOG_CMPR_LL_POS_CROSS_INTR_MASK) and the ETM event
 * ANALOG_CMPR_LL_ETM_SOURCE(n, 0, ANA_CMPR_EVENT_POS_CROSS),
 * GPIO_EVT_ZERO_DET_POSn; a negative crossing bit 3n + 1 and
 * GPIO_EVT_ZERO_DET_NEGn. Bit 3n + 2, either crossing, the driver only
 * masks off and clears (ANALOG_CMPR_LL_ALL_INTR_MASK). The TRM's names
 * for bits 3n and 3n + 1 are GPIO_COMPn_NEG_INT and _POS_INT, the other
 * way round.
 */
#define POS(n)              (1u << (3 * (n) + 0))
#define NEG(n)              (1u << (3 * (n) + 1))
#define ALL(n)              (1u << (3 * (n) + 2))
#define ALL_INTR_MASK(n)    (0x7u << (3 * (n)))
#define GPIO_EVT_ZERO_DET_POS0  25
#define EVENT_POS_CROSS     0
#define EVENT_NEG_CROSS     1
#define ETM_SOURCE(n, type) (GPIO_EVT_ZERO_DET_POS0 + 2 * (n) + (type))

/* SOC_ETM's EVT_ST0 bit for an event ID. */
#define EVT(id)             (1u << ((id) - 1))
#define EVT_POS(n)          EVT(ETM_SOURCE(n, EVENT_POS_CROSS))
#define EVT_NEG(n)          EVT(ETM_SOURCE(n, EVENT_NEG_CROSS))
#define ZERO_DET_EVTS       (EVT_POS(0) | EVT_NEG(0) | EVT_POS(1) | \
                             EVT_NEG(1))

#define ETM                 0x500d5000u
#define ETM_EVT_ST0         (ETM + 0x1a8)
#define ETM_EVT_ST0_CLR     (ETM + 0x1ac)
#define SOC_CLK_CTRL1       0x500e6018u
#define SOC_CLK_CTRL3       0x500e6020u
#define ETM_SYS_CLK_EN      (1u << 24)
#define ETM_APB_CLK_EN      (1u << 2)

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
    /* The ETM's clocks, so that it receives the comparators' events. */
    wr(SOC_CLK_CTRL1, rd(SOC_CLK_CTRL1) | ETM_SYS_CLK_EN);
    wr(SOC_CLK_CTRL3, rd(SOC_CLK_CTRL3) | ETM_APB_CLK_EN);
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

/*
 * What the last crossing latched: GPIO_INT_RAW and the comparators' ETM
 * events received, both cleared after.
 */
static void expect_crossing(uint32_t raw, uint32_t evts)
{
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, raw);
    g_assert_cmphex(rd(ETM_EVT_ST0) & ZERO_DET_EVTS, ==, evts);
    wr(GPIO_INT_CLR, ALL_INTR_MASK(0) | ALL_INTR_MASK(1));
    wr(ETM_EVT_ST0_CLR, ZERO_DET_EVTS);
}

/*
 * Comparator 0 against its internal reference, tenths of VDD_IO_6: the
 * source crossing above it is a positive crossing, below it a negative
 * one; powered down, the comparator's output is low; the interrupt
 * reaches the interrupt matrix's GPIO_PAD_COMP_INTR.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_internal_reference(void)
{
    start();
    set_mv("adc2-ch3-mv", 1500);
    /* ana_cmpr_enable() with ANA_CMPR_CROSS_ANY. */
    wr(GPIO_INT_ENA, POS(0) | NEG(0));
    wr(PAD_COMP(0), DREF(3));
    expect_crossing(0, 0);
    wr(PAD_COMP(0), DREF(3) | XPD);
    g_assert_cmphex(rd(GPIO_INT_ST), ==, POS(0));
    g_assert_true(rd(INTMTX_STATUS2) & SRC_PAD_COMP_BIT);
    expect_crossing(POS(0) | ALL(0), EVT_POS(0));
    settle();
    /* 0.5 VDD_IO_6, 1650 mV: above the source. */
    wr(PAD_COMP(0), DREF(5) | XPD);
    g_assert_cmphex(rd(GPIO_INT_ST), ==, NEG(0));
    expect_crossing(NEG(0) | ALL(0), EVT_NEG(0));
    /* The source set from QEMU crosses it. */
    settle();
    set_mv("adc2-ch3-mv", 1700);
    expect_crossing(POS(0) | ALL(0), EVT_POS(0));
    settle();
    set_mv("adc2-ch3-mv", 1649);
    expect_crossing(NEG(0) | ALL(0), EVT_NEG(0));
    /* Powered down while high: COMP_OUT falls. */
    settle();
    set_mv("adc2-ch3-mv", 3000);
    expect_crossing(POS(0) | ALL(0), EVT_POS(0));
    settle();
    wr(PAD_COMP(0), DREF(5));
    expect_crossing(NEG(0) | ALL(0), EVT_NEG(0));
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
    expect_crossing(0, 0);
    set_mv("adc2-ch5-mv", 1200);
    expect_crossing(POS(1) | ALL(1), EVT_POS(1));
    settle();
    set_mv("adc2-ch4-mv", 1300);
    expect_crossing(NEG(1) | ALL(1), EVT_NEG(1));
    stop();
}

/*
 * A source pad driven digitally, here from outside the chip, is at the
 * supply or ground; GPIO_ZERO_DET1's debounce holds back an edge too soon
 * after the last, interrupt and event alike.
 */
/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_digital_source_and_debounce(void)
{
    start();
    set_mv("adc2-ch5-mv", 2000);
    wr(PAD_COMP(1), DREF(5) | XPD);
    expect_crossing(POS(1) | ALL(1), EVT_POS(1));
    settle();
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 54, 0);
    expect_crossing(NEG(1) | ALL(1), EVT_NEG(1));
    settle();
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 54, 1);
    expect_crossing(POS(1) | ALL(1), EVT_POS(1));
    /* Released, the pad is back at its analog voltage: still high. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-release", 54, 1);
    expect_crossing(0, 0);
    /* 1000 cycles of the IO MUX clock between interrupts. */
    wr(ZERO_DET(1), 1000);
    qtest_clock_step(qts, 1000000);
    set_mv("adc2-ch5-mv", 1000);
    expect_crossing(NEG(1) | ALL(1), EVT_NEG(1));
    set_mv("adc2-ch5-mv", 2000);
    expect_crossing(0, 0);
    stop();
}

/* GPIO54, comparator 1's source pad, driven by the chip's GPIO output. */
static void src_level(int level)
{
    wr(level ? GPIO_OUT1_W1TS : GPIO_OUT1_W1TC, 1u << (54 - 32));
}

/*
 * The driver's interrupt handler: GPIO_INT_ST under unit 1's mask,
 * cleared; ANA_CMPR_CROSS_POS for POS(1), _NEG for NEG(1).
 */
static uint32_t isr_status(void)
{
    uint32_t st = rd(GPIO_INT_ST) & ALL_INTR_MASK(1);

    wr(GPIO_INT_CLR, st);
    return st;
}

/*
 * ESP-IDF's test "ana_cmpr edge-specific interrupt reports correct cross
 * direction", which runs on ESP32-P4 hardware, on unit 1 as it is there:
 * its source GPIO54 a GPIO output, set to @init_level, its internal
 * reference at 0.5 VDD_IO_6 and the interrupt armed for @cross_type only
 * (POS(1) or NEG(1)). The transition of that type is reported; the other
 * is not, though GPIO_INT_RAW latches it in its own bit.
 */
static void cross_type_case(uint32_t cross_type, int init_level)
{
    uint32_t other = cross_type == POS(1) ? NEG(1) : POS(1);

    /* ana_cmpr_new_unit(): the unit's interrupts off and cleared. */
    wr(GPIO_INT_ENA, rd(GPIO_INT_ENA) & ~ALL_INTR_MASK(1));
    wr(GPIO_INT_CLR, ALL_INTR_MASK(1));
    /* test_init_src_chan_gpio() */
    wr(IO_MUX_GPIO(54), MCU_SEL_GPIO | FUN_DRV2);
    wr(GPIO_FUNC_OUT(54), SIG_GPIO_OUT);
    src_level(init_level);
    wr(GPIO_ENABLE1_W1TS, 1u << (54 - 32));
    /* ana_cmpr_set_internal_reference(), ana_cmpr_set_debounce(). */
    wr(PAD_COMP(1), DREF(5));
    wr(ZERO_DET(1), 800);
    /* ana_cmpr_enable() */
    wr(GPIO_INT_ENA, rd(GPIO_INT_ENA) | cross_type);
    wr(PAD_COMP(1), DREF(5) | XPD);
    qtest_clock_step(qts, 1000000);
    /*
     * Powering up with the source high is a positive crossing, not armed
     * when the source starts high.
     */
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, init_level ? POS(1) | ALL(1) : 0);
    g_assert_cmphex(isr_status(), ==, 0);
    wr(GPIO_INT_CLR, ALL_INTR_MASK(1));

    src_level(!init_level);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, cross_type | ALL(1));
    g_assert_cmphex(isr_status(), ==, cross_type);
    wr(GPIO_INT_CLR, ALL_INTR_MASK(1));

    src_level(init_level);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(rd(GPIO_INT_RAW), ==, other | ALL(1));
    g_assert_cmphex(isr_status(), ==, 0);
    wr(GPIO_INT_CLR, ALL_INTR_MASK(1));

    /* ana_cmpr_disable() */
    wr(GPIO_INT_ENA, rd(GPIO_INT_ENA) & ~cross_type);
    wr(PAD_COMP(1), DREF(5));
}

/* [spec:nuos:req:emu.esp32p4.analog/test] */
static void test_cross_type(void)
{
    start();
    cross_type_case(POS(1), 0);
    cross_type_case(NEG(1), 1);
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
    qtest_add_func("esp32p4/ana-cmpr/cross_type", test_cross_type);
    return g_test_run();
}
