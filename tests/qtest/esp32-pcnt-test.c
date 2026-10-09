/*
 * QTest testcase for the ESP32 pulse count controller (PCNT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN  0x3ff000c0
#define DPORT_PERIP_RST_EN  0x3ff000c4
#define PERIP_PCNT          (1u << 10)
#define PERIP_CLK_EN_RESET  0xf9c1e06fu

#define PCNT                0x3ff57000
#define PCNT_CONF0(u)       (PCNT + 0x00 + 0x0c * (u))
#define PCNT_CONF1(u)       (PCNT + 0x04 + 0x0c * (u))
#define PCNT_CONF2(u)       (PCNT + 0x08 + 0x0c * (u))
#define PCNT_CNT(u)         (PCNT + 0x60 + 4 * (u))
#define PCNT_INT_RAW        (PCNT + 0x80)
#define PCNT_INT_ST         (PCNT + 0x84)
#define PCNT_INT_ENA        (PCNT + 0x88)
#define PCNT_INT_CLR        (PCNT + 0x8c)
#define PCNT_STATUS(u)      (PCNT + 0x90 + 4 * (u))
#define PCNT_CTRL           (PCNT + 0xb0)
#define PCNT_DATE           (PCNT + 0xfc)
/* The same block in the APB window */
#define PCNT_APB_CONF0      0x60017000

#define FILTER_THRES(n)     (n)
#define FILTER_EN           (1u << 10)
#define THR_ZERO_EN         (1u << 11)
#define THR_H_LIM_EN        (1u << 12)
#define THR_L_LIM_EN        (1u << 13)
#define THR_THRES0_EN       (1u << 14)
#define THR_THRES1_EN       (1u << 15)
#define CH_NEG(ch, m)       ((uint32_t)(m) << (16 + 8 * (ch)))
#define CH_POS(ch, m)       ((uint32_t)(m) << (18 + 8 * (ch)))
#define CH_HCTRL(ch, m)     ((uint32_t)(m) << (20 + 8 * (ch)))
#define CH_LCTRL(ch, m)     ((uint32_t)(m) << (22 + 8 * (ch)))
#define INC                 1
#define DEC                 2
#define INVERT              1
#define INHIBIT             2

#define ST_THRES1           (1u << 2)
#define ST_THRES0           (1u << 3)
#define ST_L_LIM            (1u << 4)
#define ST_H_LIM            (1u << 5)
#define ST_ZERO             (1u << 6)
#define ZM_POS_ZERO         0
#define ZM_NEG_ZERO         1
#define ZM_NEG              2
#define ZM_POS              3

#define CNT_RST(u)          (1u << (2 * (u)))
#define CNT_PAUSE(u)        (1u << (2 * (u) + 1))

#define GPIO                0x3ff44000
#define GPIO_FUNC_IN(s)     (GPIO + 0x130 + 4 * (s))
#define IN_SIG_IN_SEL       (1u << 7)
#define IN_CONST_LOW        0x30
#define IN_CONST_HIGH       0x38
#define IO_MUX              0x3ff49000
#define IO_MUX_GPIO18       (IO_MUX + 0x70)
#define IO_MUX_GPIO19       (IO_MUX + 0x74)
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (2u << 12)

/* Unit u's signal and control inputs in the GPIO matrix */
#define SIG_PCNT(u)         ((u) < 5 ? 39 + 4 * (u) : 71 + 4 * ((u) - 5))
#define SIG_CH0(u)          (SIG_PCNT(u) + 0)
#define SIG_CH1(u)          (SIG_PCNT(u) + 1)
#define CTRL_CH0(u)         (SIG_PCNT(u) + 2)
#define CTRL_CH1(u)         (SIG_PCNT(u) + 3)

#define GPIO_PATH           "/machine/soc/gpio"
#define PCNT_PATH           "/machine/soc/pcnt"
#define PAD_IN              "esp32-gpio-pad-in"

#define PAD_SIG             18
#define PAD_CTRL            19

/* APB_CLK after reset: 40 MHz from the crystal */
#define APB_NS              25

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    qtest_writel(qts, DPORT_PERIP_CLK_EN, PERIP_CLK_EN_RESET | PERIP_PCNT);
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL_GPIO | FUN_IE);
    return qts;
}

static void drive(QTestState *qts, unsigned pad, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, pad, level);
}

static void pulses(QTestState *qts, unsigned pad, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        drive(qts, pad, 1);
        drive(qts, pad, 0);
    }
}

static int16_t count(QTestState *qts, unsigned u)
{
    return (int16_t)qtest_readl(qts, PCNT_CNT(u));
}

/*
 * Unit u counting pad PAD_SIG on channel 0, its control input held at
 * ctrl_level (or taken from PAD_CTRL if ctrl_level < 0), filter off,
 * counter released.
 */
static void setup_unit(QTestState *qts, unsigned u, uint32_t conf0,
                       int ctrl_level)
{
    drive(qts, PAD_SIG, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_CH0(u)), IN_SIG_IN_SEL | PAD_SIG);
    if (ctrl_level < 0) {
        qtest_writel(qts, GPIO_FUNC_IN(CTRL_CH0(u)), IN_SIG_IN_SEL | PAD_CTRL);
    } else {
        qtest_writel(qts, GPIO_FUNC_IN(CTRL_CH0(u)), IN_SIG_IN_SEL |
                     (ctrl_level ? IN_CONST_HIGH : IN_CONST_LOW));
    }
    qtest_writel(qts, PCNT_CONF0(u), conf0);
    qtest_writel(qts, PCNT_CTRL, qtest_readl(qts, PCNT_CTRL) & ~CNT_RST(u));
}

/* [spec:nuos:req:emu.esp32.pcnt/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    for (unsigned u = 0; u < 8; u++) {
        g_assert_cmphex(qtest_readl(qts, PCNT_CONF0(u)), ==, 0x3c10);
        g_assert_cmphex(qtest_readl(qts, PCNT_CONF1(u)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, PCNT_CONF2(u)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, PCNT_CNT(u)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(u)), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_ENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_CTRL), ==, 0x5555);
    g_assert_cmphex(qtest_readl(qts, PCNT_DATE), ==, 0x14122600);

    /* Read-only registers keep their values */
    qtest_writel(qts, PCNT_CNT(0), 0x1234);
    qtest_writel(qts, PCNT_STATUS(0), 0x7f);
    qtest_writel(qts, PCNT_INT_RAW, 0xff);
    g_assert_cmphex(qtest_readl(qts, PCNT_CNT(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);

    /* The APB window reaches the same registers */
    qtest_writel(qts, PCNT_CONF1(0), 0x00050003);
    g_assert_cmphex(qtest_readl(qts, PCNT_APB_CONF0 + 4), ==, 0x00050003);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.pcnt/test] */
static void test_edges(void)
{
    QTestState *qts = start();

    /* Rising edges count up, falling edges are ignored */
    setup_unit(qts, 0, CH_POS(0, INC), 0);
    pulses(qts, PAD_SIG, 5);
    g_assert_cmpint(count(qts, 0), ==, 5);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(0)), ==, ZM_POS);

    /* Falling edges count down, through zero to negative */
    qtest_writel(qts, PCNT_CONF0(0), CH_NEG(0, DEC));
    pulses(qts, PAD_SIG, 7);
    g_assert_cmpint(count(qts, 0), ==, -2);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(0)), ==, ZM_NEG);
    /* The counter field reads as 16 bits */
    g_assert_cmphex(qtest_readl(qts, PCNT_CNT(0)), ==, 0xfffe);

    /* Both edges */
    qtest_writel(qts, PCNT_CONF0(0), CH_POS(0, INC) | CH_NEG(0, INC));
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 0), ==, 2);

    /* Modes 0 and 3 do nothing */
    qtest_writel(qts, PCNT_CONF0(0), CH_POS(0, 3) | CH_NEG(0, 0));
    pulses(qts, PAD_SIG, 3);
    g_assert_cmpint(count(qts, 0), ==, 2);

    /* Control input: high inverts, low inhibits */
    qtest_writel(qts, GPIO_FUNC_IN(CTRL_CH0(0)), IN_SIG_IN_SEL | PAD_CTRL);
    qtest_writel(qts, PCNT_CONF0(0), CH_POS(0, INC) | CH_HCTRL(0, INVERT) |
                 CH_LCTRL(0, INHIBIT));
    drive(qts, PAD_CTRL, 1);
    pulses(qts, PAD_SIG, 4);
    g_assert_cmpint(count(qts, 0), ==, -2);
    drive(qts, PAD_CTRL, 0);
    pulses(qts, PAD_SIG, 4);
    g_assert_cmpint(count(qts, 0), ==, -2);
    /* Control edges themselves do not count */
    drive(qts, PAD_CTRL, 1);
    drive(qts, PAD_CTRL, 0);
    g_assert_cmpint(count(qts, 0), ==, -2);
    /* Inhibit value 3 behaves as 2 */
    qtest_writel(qts, PCNT_CONF0(0), CH_POS(0, DEC) | CH_LCTRL(0, 3));
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 0), ==, -2);

    /* Channel 1 counts into the same counter as channel 0 */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_CH1(0)), IN_SIG_IN_SEL | PAD_SIG);
    qtest_writel(qts, GPIO_FUNC_IN(CTRL_CH1(0)), IN_SIG_IN_SEL |
                 IN_CONST_LOW);
    qtest_writel(qts, PCNT_CONF0(0), CH_POS(0, INC) | CH_LCTRL(0, 0) |
                 CH_POS(1, INC));
    pulses(qts, PAD_SIG, 3);
    g_assert_cmpint(count(qts, 0), ==, 4);

    /* With the limit comparators off, counting down from zero goes negative */
    qtest_writel(qts, PCNT_CTRL, 0x5555);
    setup_unit(qts, 7, CH_POS(0, DEC), 0);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmpint(count(qts, 7), ==, -1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.pcnt/test] */
static void test_pause_reset(void)
{
    QTestState *qts = start();

    /* Held in reset after reset */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_CH0(3)), IN_SIG_IN_SEL | PAD_SIG);
    qtest_writel(qts, PCNT_CONF0(3), CH_POS(0, INC));
    pulses(qts, PAD_SIG, 3);
    g_assert_cmpint(count(qts, 3), ==, 0);

    qtest_writel(qts, PCNT_CTRL, 0x5555 & ~CNT_RST(3));
    pulses(qts, PAD_SIG, 3);
    g_assert_cmpint(count(qts, 3), ==, 3);

    qtest_writel(qts, PCNT_CTRL, (0x5555 & ~CNT_RST(3)) | CNT_PAUSE(3));
    pulses(qts, PAD_SIG, 3);
    g_assert_cmpint(count(qts, 3), ==, 3);

    qtest_writel(qts, PCNT_CTRL, 0x5555 & ~CNT_RST(3));
    pulses(qts, PAD_SIG, 1);
    g_assert_cmpint(count(qts, 3), ==, 4);

    /* Reset clears and holds; release counts from zero */
    qtest_writel(qts, PCNT_CTRL, 0x5555);
    g_assert_cmpint(count(qts, 3), ==, 0);
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 3), ==, 0);
    qtest_writel(qts, PCNT_CTRL, 0x5555 & ~CNT_RST(3));
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 3), ==, 2);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.pcnt/test] */
static void test_watchpoints(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, PCNT_PATH, "sysbus-irq");
    qtest_writel(qts, PCNT_CONF1(2), (uint32_t)(uint16_t)-2 << 16 | 3);
    qtest_writel(qts, PCNT_CONF2(2), (uint32_t)(uint16_t)-4 << 16 | 5);
    setup_unit(qts, 2, CH_POS(0, INC) | THR_THRES0_EN | THR_THRES1_EN |
               THR_H_LIM_EN | THR_L_LIM_EN | THR_ZERO_EN, -1);
    drive(qts, PAD_CTRL, 0);
    qtest_writel(qts, PCNT_INT_ENA, 1u << 2);

    pulses(qts, PAD_SIG, 2);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* thres0 = 3 */
    pulses(qts, PAD_SIG, 1);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_ST), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==, ST_THRES0 | ZM_POS);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    /* The latch holds until the next event */
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==, ST_THRES0 | ZM_POS);

    /* h_lim = 5: the counter returns to zero */
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==,
                    ST_H_LIM | ZM_POS_ZERO);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);

    /* Counting down: zero is not crossed yet, thres1 = -2, l_lim = -4 */
    qtest_writel(qts, PCNT_CONF0(2), CH_POS(0, INC) | CH_LCTRL(0, INVERT) |
                 THR_THRES0_EN | THR_THRES1_EN | THR_H_LIM_EN |
                 THR_L_LIM_EN | THR_ZERO_EN);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmpint(count(qts, 2), ==, -1);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==, ST_THRES1 | ZM_NEG);
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==,
                    ST_L_LIM | ZM_NEG_ZERO);
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);

    /* Back up to zero from below: the zero event and -0 */
    pulses(qts, PAD_SIG, 1);
    drive(qts, PAD_CTRL, 1);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmpint(count(qts, 2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==,
                    ST_ZERO | ZM_NEG_ZERO);
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);
    /* and from above: +0 */
    pulses(qts, PAD_SIG, 1);
    drive(qts, PAD_CTRL, 0);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmphex(qtest_readl(qts, PCNT_STATUS(2)), ==,
                    ST_ZERO | ZM_POS_ZERO);
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);

    /* A disabled comparator raises nothing; INT_ENA masks the line only */
    qtest_writel(qts, PCNT_INT_ENA, 0);
    qtest_writel(qts, PCNT_CONF0(2), CH_POS(0, INC) | THR_THRES0_EN);
    drive(qts, PAD_CTRL, 1);
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 2), ==, 2);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_ST), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, PCNT_INT_ENA, 1u << 2);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, PCNT_INT_CLR, 1u << 2);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Changing a threshold onto the current count is not a match */
    qtest_writel(qts, PCNT_CONF1(2), 3);
    qtest_writel(qts, PCNT_CONF1(2), 3);
    g_assert_cmphex(qtest_readl(qts, PCNT_INT_RAW), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.pcnt/test] */
static void test_filter(void)
{
    QTestState *qts = start();

    /* 10 APB_CLK cycles: 250 ns */
    setup_unit(qts, 1, CH_POS(0, INC) | FILTER_EN | FILTER_THRES(10), 0);

    /* A 200 ns pulse is lost */
    drive(qts, PAD_SIG, 1);
    qtest_clock_step(qts, 8 * APB_NS);
    drive(qts, PAD_SIG, 0);
    qtest_clock_step(qts, 100 * APB_NS);
    g_assert_cmpint(count(qts, 1), ==, 0);

    /* A 300 ns pulse counts once its edge has held for 250 ns */
    drive(qts, PAD_SIG, 1);
    qtest_clock_step(qts, 9 * APB_NS);
    g_assert_cmpint(count(qts, 1), ==, 0);
    qtest_clock_step(qts, 3 * APB_NS);
    g_assert_cmpint(count(qts, 1), ==, 1);
    drive(qts, PAD_SIG, 0);
    qtest_clock_step(qts, 100 * APB_NS);
    g_assert_cmpint(count(qts, 1), ==, 1);

    /* A short low glitch inside a high pulse is not two edges */
    qtest_writel(qts, PCNT_CONF0(1), CH_POS(0, INC) | CH_NEG(0, INC) |
                 FILTER_EN | FILTER_THRES(10));
    drive(qts, PAD_SIG, 1);
    qtest_clock_step(qts, 20 * APB_NS);
    drive(qts, PAD_SIG, 0);
    qtest_clock_step(qts, 2 * APB_NS);
    drive(qts, PAD_SIG, 1);
    qtest_clock_step(qts, 20 * APB_NS);
    g_assert_cmpint(count(qts, 1), ==, 2);
    drive(qts, PAD_SIG, 0);
    qtest_clock_step(qts, 20 * APB_NS);
    g_assert_cmpint(count(qts, 1), ==, 3);

    /* Turning the filter off passes a held-back change at once */
    drive(qts, PAD_SIG, 1);
    g_assert_cmpint(count(qts, 1), ==, 3);
    qtest_writel(qts, PCNT_CONF0(1), CH_POS(0, INC) | CH_NEG(0, INC));
    g_assert_cmpint(count(qts, 1), ==, 4);

    /* Filter disabled: every edge counts at once */
    drive(qts, PAD_SIG, 0);
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 1), ==, 9);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.pcnt/test] */
static void test_gated(void)
{
    QTestState *qts = start();

    setup_unit(qts, 0, CH_POS(0, INC), 0);
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 0), ==, 2);

    /* Clock gated: registers ignore writes, the counter stops */
    qtest_writel(qts, DPORT_PERIP_CLK_EN, PERIP_CLK_EN_RESET);
    qtest_writel(qts, PCNT_CONF1(0), 0x1234);
    g_assert_cmphex(qtest_readl(qts, PCNT_CONF1(0)), ==, 0);
    pulses(qts, PAD_SIG, 2);
    g_assert_cmpint(count(qts, 0), ==, 2);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, PERIP_CLK_EN_RESET | PERIP_PCNT);
    pulses(qts, PAD_SIG, 1);
    g_assert_cmpint(count(qts, 0), ==, 3);

    /* DPORT reset: back to reset values, counters held */
    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_PCNT);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    g_assert_cmpint(count(qts, 0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PCNT_CONF0(0)), ==, 0x3c10);
    g_assert_cmphex(qtest_readl(qts, PCNT_CTRL), ==, 0x5555);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32-pcnt/reset", test_reset);
    qtest_add_func("esp32-pcnt/edges", test_edges);
    qtest_add_func("esp32-pcnt/pause-reset", test_pause_reset);
    qtest_add_func("esp32-pcnt/watchpoints", test_watchpoints);
    qtest_add_func("esp32-pcnt/filter", test_filter);
    qtest_add_func("esp32-pcnt/gated", test_gated);

    return g_test_run();
}
