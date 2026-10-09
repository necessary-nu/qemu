/*
 * QTest testcase for the ESP32 motor control PWM (MCPWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN  0x3ff000c0
#define DPORT_PERIP_RST_EN  0x3ff000c4
#define PERIP_PWM0          (1u << 17)
#define PERIP_PWM1          (1u << 20)

#define PWM0                0x3ff5e000
#define PWM1                0x3ff6c000
#define CLK_CFG             0x000
#define TIMER_CFG0(i)       (0x004 + 0x10 * (i))
#define TIMER_CFG1(i)       (0x008 + 0x10 * (i))
#define TIMER_SYNC(i)       (0x00c + 0x10 * (i))
#define TIMER_STATUS(i)     (0x010 + 0x10 * (i))
#define SYNCI_CFG           0x034
#define TIMERSEL            0x038
#define OP(k, r)            (0x03c + 0x38 * (k) + (r))
#define STMP_CFG            0x00
#define TSTMP_A             0x04
#define TSTMP_B             0x08
#define GEN_CFG0            0x0c
#define GEN_FORCE           0x10
#define GEN_A               0x14
#define GEN_B               0x18
#define DT_CFG              0x1c
#define DT_FED              0x20
#define DT_RED              0x24
#define CARRIER             0x28
#define FH_CFG0             0x2c
#define FH_CFG1             0x30
#define FH_STATUS           0x34
#define FAULT_DETECT        0x0e4
#define CAP_TIMER_CFG       0x0e8
#define CAP_TIMER_PHASE     0x0ec
#define CAP_CH_CFG(n)       (0x0f0 + 4 * (n))
#define CAP_CH(n)           (0x0fc + 4 * (n))
#define CAP_STATUS          0x108
#define UPDATE_CFG          0x10c
#define INT_ENA             0x110
#define INT_RAW             0x114
#define INT_ST              0x118
#define INT_CLR             0x11c
#define VERSION             0x124

#define CFG0(pre, period, up)   ((pre) | ((period) << 8) | ((up) << 24))
#define CFG1(start, mod)        ((start) | ((mod) << 3))
#define START_STOP_TEZ      0
#define START_RUN           2
#define START_ONCE_TEZ      3
#define MOD_UP              1
#define MOD_DOWN            2
#define MOD_UPDOWN          3
#define SYNCI_EN            (1u << 0)
#define SYNC_SW             (1u << 1)
#define SYNCO_TEZ           (1u << 2)
#define PHASE(p)            ((p) << 4)

/* Generator actions: event field offsets, add 12 for counting down */
#define ACT(field, act)     ((act) << (field))
#define UTEZ                0
#define UTEP                2
#define UTEA                4
#define UTEB                6
#define UT0                 8
#define DTEA                16
#define LOW                 1
#define HIGH                2
#define TOGGLE              3

#define INT_TIMER_STOP(i)   (1u << (i))
#define INT_TEZ(i)          (1u << (3 + (i)))
#define INT_TEP(i)          (1u << (6 + (i)))
#define INT_FAULT(n)        (1u << (9 + (n)))
#define INT_FAULT_CLR(n)    (1u << (12 + (n)))
#define INT_TEA(k)          (1u << (15 + (k)))
#define INT_OST(k)          (1u << (24 + (k)))
#define INT_CBC(k)          (1u << (21 + (k)))
#define INT_CAP(n)          (1u << (27 + (n)))

#define GPIO                0x3ff44000
#define GPIO_IN             (GPIO + 0x3c)
#define GPIO_FUNC_IN(s)     (GPIO + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x530 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO18       0x3ff49070
#define IO_MUX_GPIO19       0x3ff49074
#define IO_MUX_GPIO21       0x3ff4907c
#define MCU_SEL_GPIO        (2u << 12)
#define FUN_IE              (1u << 9)
#define SIG_PWM0_OUT0A      32
#define SIG_PWM0_OUT0B      33
#define SIG_PWM0_F0         34
#define SIG_PWM0_CAP0       109

#define TIMG0_WDTCONFIG0    0x3ff5f048
#define RTC_WDTCONFIG0      0x3ff4808c
#define RTC_WDTWPROTECT     0x3ff480a4
#define RTC_WDT_WKEY        0x50d83aa1

#define GPIO_PATH           "/machine/soc/gpio"
#define PAD_IN              "esp32-gpio-pad-in"
#define PAD_OUT             "esp32-gpio-pad"

#define US                  1000

/* PT_clk at 1 MHz: PWM_clk 160 MHz divided by 160 */
#define PRE_1MHZ            159

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) |
                 PERIP_PWM0 | PERIP_PWM1);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) &
                 ~(PERIP_PWM0 | PERIP_PWM1));
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, PWM0 + reg);
}

static void wr(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, PWM0 + reg, v);
}

/* A pad as a GPIO matrix pad with its input enabled */
static void pad_matrix(QTestState *qts, uint32_t iomux)
{
    qtest_writel(qts, iomux, MCU_SEL_GPIO | FUN_IE);
}

static bool pad_in(QTestState *qts, unsigned n)
{
    return (qtest_readl(qts, GPIO_IN) >> n) & 1;
}

/* Timer 0 counting up at 1 MHz with period P, from 0 */
static void timer0_up(QTestState *qts, uint32_t period)
{
    wr(qts, TIMER_CFG0(0), CFG0(PRE_1MHZ, period, 0));
    wr(qts, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    for (int i = 0; i < 3; i++) {
        g_assert_cmphex(rd(qts, TIMER_CFG0(i)), ==, 0xff00);
        g_assert_cmphex(rd(qts, TIMER_CFG1(i)), ==, 0);
        g_assert_cmphex(rd(qts, TIMER_STATUS(i)), ==, 0);
    }
    for (int k = 0; k < 3; k++) {
        g_assert_cmphex(rd(qts, OP(k, GEN_FORCE)), ==, 0x20);
        g_assert_cmphex(rd(qts, OP(k, DT_CFG)), ==, 0x18000);
        g_assert_cmphex(rd(qts, OP(k, GEN_A)), ==, 0);
        g_assert_cmphex(rd(qts, OP(k, FH_STATUS)), ==, 0);
    }
    g_assert_cmphex(rd(qts, UPDATE_CFG), ==, 0x55);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, VERSION), ==, 0x2107230);
    g_assert_cmphex(qtest_readl(qts, PWM1 + UPDATE_CFG), ==, 0x55);

    /* Field widths */
    wr(qts, CLK_CFG, 0xffffffff);
    g_assert_cmphex(rd(qts, CLK_CFG), ==, 0xff);
    wr(qts, TIMER_CFG0(1), 0xffffffff);
    g_assert_cmphex(rd(qts, TIMER_CFG0(1)), ==, 0x03ffffff);
    wr(qts, TIMER_SYNC(1), 0xfffffffd);
    g_assert_cmphex(rd(qts, TIMER_SYNC(1)), ==, 0x1ffffd);
    wr(qts, OP(1, GEN_A), 0xffffffff);
    g_assert_cmphex(rd(qts, OP(1, GEN_A)), ==, 0xffffff);
    wr(qts, OP(1, CARRIER), 0xffffffff);
    g_assert_cmphex(rd(qts, OP(1, CARRIER)), ==, 0x3fff);
    wr(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0x3fffffff);
    /* STATUS is read-only */
    wr(qts, TIMER_STATUS(0), 0x1234);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 0);

    /* Held in reset by DPORT, the block keeps its reset values */
    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_PWM0);
    g_assert_cmphex(rd(qts, CLK_CFG), ==, 0);
    wr(qts, CLK_CFG, 0x12);
    g_assert_cmphex(rd(qts, CLK_CFG), ==, 0);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    wr(qts, CLK_CFG, 0x12);
    g_assert_cmphex(rd(qts, CLK_CFG), ==, 0x12);

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_counting(void)
{
    QTestState *qts = start();

    timer0_up(qts, 999);
    qtest_clock_step(qts, 250 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 250);
    qtest_clock_step(qts, 1000 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 250);

    /* Freeze holds the count */
    wr(qts, TIMER_CFG1(0), CFG1(START_RUN, 0));
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 250);

    /* Down: 250 down to 0, then from the period */
    wr(qts, TIMER_CFG1(0), CFG1(START_RUN, MOD_DOWN));
    qtest_clock_step(qts, 50 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, (1u << 16) | 200);
    qtest_clock_step(qts, 201 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, (1u << 16) | 999);

    /* Up-down with period 100: 0..99 up, 100..1 down */
    wr(qts, TIMER_CFG0(1), CFG0(PRE_1MHZ, 100, 0));
    wr(qts, TIMER_CFG1(1), CFG1(START_RUN, MOD_UPDOWN));
    qtest_clock_step(qts, 99 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 99);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, (1u << 16) | 100);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 0);
    /* Many periods later the counter is where it should be */
    qtest_clock_step(qts, 2000 * 200 * US + 30 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 30);

    /* The group prescaler halves PWM_clk */
    wr(qts, TIMER_CFG1(1), CFG1(START_STOP_TEZ, MOD_UPDOWN));
    qtest_clock_step(qts, 200 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TIMER_STOP(1), ==,
                    INT_TIMER_STOP(1));
    wr(qts, CLK_CFG, 1);
    wr(qts, TIMER_CFG1(1), CFG1(START_RUN, MOD_UPDOWN));
    qtest_clock_step(qts, 40 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 20);

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_interrupts(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, "/machine/soc/mcpwm0", "sysbus-irq");
    wr(qts, INT_ENA, INT_TEZ(0));
    timer0_up(qts, 99);
    /* TEP at 99 us, TEZ at 100 us */
    qtest_clock_step(qts, 99 * US);
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TEP(0));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, INT_ST), ==, INT_TEZ(0));
    wr(qts, INT_CLR, INT_TEZ(0) | INT_TEP(0));
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 99 * US);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 0));

    /* TEA of operator 0 */
    wr(qts, INT_CLR, 0xffffffff);
    wr(qts, OP(0, TSTMP_A), 40);
    wr(qts, INT_ENA, INT_TEA(0));
    qtest_clock_step(qts, 39 * US);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 0));

    /* A one-shot start stops at the next TEZ with a stop interrupt */
    wr(qts, INT_CLR, 0xffffffff);
    wr(qts, INT_ENA, INT_TIMER_STOP(0));
    wr(qts, TIMER_CFG1(0), CFG1(START_ONCE_TEZ, MOD_UP));
    qtest_clock_step(qts, 59 * US);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, TIMER_CFG1(0)), ==,
                    CFG1(START_STOP_TEZ, MOD_UP));
    qtest_clock_step(qts, 50 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 0);

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_shadow(void)
{
    QTestState *qts = start();

    /* Period taken at TEZ; the first write, immediate, sets it at once */
    wr(qts, TIMER_CFG0(0), CFG0(PRE_1MHZ, 99, 0));
    wr(qts, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    qtest_clock_step(qts, 10 * US);
    wr(qts, TIMER_CFG0(0), CFG0(PRE_1MHZ, 199, 1));
    qtest_clock_step(qts, 90 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 0);
    qtest_clock_step(qts, 150 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, 150);

    /* Compare A taken at TEP, SHDW_FULL until then */
    wr(qts, OP(0, STMP_CFG), 2);
    wr(qts, OP(0, TSTMP_A), 77);
    g_assert_cmphex(rd(qts, OP(0, STMP_CFG)), ==, 2 | (1u << 8));
    g_assert_cmphex(rd(qts, OP(0, TSTMP_A)), ==, 77);
    wr(qts, INT_CLR, 0xffffffff);
    qtest_clock_step(qts, 49 * US);
    g_assert_cmphex(rd(qts, OP(0, STMP_CFG)), ==, 2);
    qtest_clock_step(qts, 77 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TEA(0), ==, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TEA(0), ==, INT_TEA(0));

    /* With updates disabled, only a forced update transfers */
    wr(qts, UPDATE_CFG, 0x54);
    wr(qts, OP(0, STMP_CFG), 0);
    wr(qts, OP(0, TSTMP_A), 5);
    g_assert_cmphex(rd(qts, OP(0, STMP_CFG)), ==, 1u << 8);
    wr(qts, UPDATE_CFG, 0x54 | 2);
    g_assert_cmphex(rd(qts, OP(0, STMP_CFG)), ==, 0);

    qtest_quit(qts);
}

/* Pad 18 shows PWM0A, pad 21 PWM0B; the pad outputs are intercepted. */
static QTestState *start_pads(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, PAD_OUT);
    pad_matrix(qts, IO_MUX_GPIO18);
    pad_matrix(qts, IO_MUX_GPIO21);
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_PWM0_OUT0A);
    qtest_writel(qts, GPIO_FUNC_OUT(21), SIG_PWM0_OUT0B);
    return qts;
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_generator(void)
{
    QTestState *qts = start_pads();

    /* PWM0A: high at TEZ, low at TEA = 25 of a 100 us period */
    wr(qts, OP(0, TSTMP_A), 25);
    wr(qts, OP(0, GEN_A), ACT(UTEZ, HIGH) | ACT(UTEA, LOW));
    /* PWM0B: toggled at every TEP */
    wr(qts, OP(0, GEN_B), ACT(UTEP, TOGGLE));
    timer0_up(qts, 99);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 99 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    g_assert_true(qtest_get_irq(qts, 21));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    g_assert_true(pad_in(qts, 18));
    qtest_clock_step(qts, 24 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 1 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 74 * US);
    g_assert_false(qtest_get_irq(qts, 21));

    /* Long after, still in step: 1000 periods on, 10 us into one */
    qtest_clock_step(qts, 1000 * 100 * US + 11 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    g_assert_false(qtest_get_irq(qts, 21));

    /* A non-continuous force lasts until the next action */
    wr(qts, OP(0, GEN_FORCE), (1u << 10) | (LOW << 11));
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 89 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 18));

    /* A continuous force holds the output */
    wr(qts, OP(0, GEN_FORCE), (HIGH << 6) | (LOW << 8));
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 300 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    g_assert_false(qtest_get_irq(qts, 21));
    wr(qts, OP(0, GEN_FORCE), 0);

    /* Count-down: DTEA sets PWM0A low, UTEA would never happen */
    wr(qts, OP(0, GEN_A), ACT(DTEA, LOW) | ACT(UTEZ + 12, HIGH));
    wr(qts, TIMER_CFG1(0), CFG1(START_RUN, MOD_DOWN));
    qtest_clock_step(qts, 200 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TEA(0), ==, INT_TEA(0));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_deadtime(void)
{
    QTestState *qts = start_pads();

    /*
     * Active high complementary: PWM0A is the generator's A with its
     * rising edge delayed by RED, PWM0B its inverse with the falling
     * edge delayed by FED. Dead times count PWM_clk, here 1 MHz.
     */
    wr(qts, CLK_CFG, 159);
    wr(qts, OP(0, TSTMP_A), 50);
    wr(qts, OP(0, GEN_A), ACT(UTEZ, HIGH) | ACT(UTEA, LOW));
    wr(qts, OP(0, DT_RED), 4);
    wr(qts, OP(0, DT_FED), 9);
    wr(qts, OP(0, DT_CFG), 1u << 14);
    wr(qts, TIMER_CFG0(0), CFG0(0, 99, 0));
    wr(qts, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    /* At reset of the path PWM0B is the inverse of a low A */
    g_assert_true(qtest_get_irq(qts, 21));

    /* TEZ at 100 us: A rises 5 us later, B falls at once */
    qtest_clock_step(qts, 100 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    g_assert_false(qtest_get_irq(qts, 21));
    qtest_clock_step(qts, 4 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    /* TEA at 150 us: A falls at once, B rises 10 us later */
    qtest_clock_step(qts, 45 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    g_assert_false(qtest_get_irq(qts, 21));
    qtest_clock_step(qts, 9 * US);
    g_assert_false(qtest_get_irq(qts, 21));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 21));

    /* Swapped outputs */
    wr(qts, OP(0, DT_CFG), (1u << 14) | (1u << 9) | (1u << 10));
    g_assert_true(qtest_get_irq(qts, 18));
    g_assert_false(qtest_get_irq(qts, 21));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_carrier(void)
{
    QTestState *qts = start_pads();

    /*
     * PWM_clk at 1 MHz, PC_clk at 500 kHz: carrier period 16 us, the
     * first pulse two periods (32 us), then 3/8 duty: high 6 us of 16.
     */
    wr(qts, CLK_CFG, 159);
    wr(qts, OP(0, CARRIER), 1 | (1 << 1) | (3 << 5) | (1 << 8));
    wr(qts, OP(0, GEN_FORCE), 1u << 10 | (HIGH << 11));
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 31 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 6 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 10 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 5 * US);
    g_assert_true(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 1 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    /* A low input stops the carrier */
    wr(qts, OP(0, GEN_FORCE), (1u << 10) | (LOW << 11));
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 100 * US);
    g_assert_false(qtest_get_irq(qts, 18));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_fault(void)
{
    QTestState *qts = start_pads();

    /* FAULT0 from pad 19, active high */
    pad_matrix(qts, IO_MUX_GPIO19);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM0_F0), IN_SIG_IN_SEL | 19);
    wr(qts, FAULT_DETECT, 1 | (1 << 3));
    g_assert_cmphex(rd(qts, FAULT_DETECT), ==, 0x9);

    /* PWM0A held high by its generator; one-shot trips force it low */
    wr(qts, OP(0, GEN_FORCE), (HIGH << 6));
    wr(qts, OP(0, FH_CFG0), (1u << 7) | (LOW << 12) | (LOW << 14) |
       (HIGH << 20) | (HIGH << 22));
    g_assert_true(qtest_get_irq(qts, 18));
    g_assert_false(qtest_get_irq(qts, 21));

    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 1);
    g_assert_cmphex(rd(qts, FAULT_DETECT), ==, 0x9 | (1u << 6));
    g_assert_false(qtest_get_irq(qts, 18));
    g_assert_true(qtest_get_irq(qts, 21));
    g_assert_cmphex(rd(qts, OP(0, FH_STATUS)), ==, 2);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_FAULT(0) | INT_OST(0));

    /* The trip outlasts the fault until cleared */
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==,
                    INT_FAULT(0) | INT_OST(0) | INT_FAULT_CLR(0));
    g_assert_false(qtest_get_irq(qts, 18));
    wr(qts, OP(0, FH_CFG1), 1);
    g_assert_cmphex(rd(qts, OP(0, FH_STATUS)), ==, 0);
    g_assert_true(qtest_get_irq(qts, 18));

    /* Cycle-by-cycle: ends at the TEZ after the fault goes */
    wr(qts, INT_CLR, 0xffffffff);
    wr(qts, OP(0, FH_CFG0), (1u << 3) | (LOW << 8) | (LOW << 10));
    wr(qts, OP(0, FH_CFG1), 1 << 1);
    timer0_up(qts, 99);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 1);
    g_assert_false(qtest_get_irq(qts, 18));
    g_assert_cmphex(rd(qts, OP(0, FH_STATUS)), ==, 1);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_CBC(0), ==, INT_CBC(0));
    qtest_clock_step(qts, 150 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
    qtest_clock_step(qts, 49 * US);
    g_assert_false(qtest_get_irq(qts, 18));
    qtest_clock_step(qts, 1 * US);
    g_assert_true(qtest_get_irq(qts, 18));

    /* Software one-shot trip */
    wr(qts, OP(0, FH_CFG0), (1u << 4) | (LOW << 12) | (LOW << 14));
    wr(qts, OP(0, FH_CFG1), (1 << 1) | (1 << 4));
    g_assert_false(qtest_get_irq(qts, 18));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_capture(void)
{
    QTestState *qts = start();
    uint32_t t0, t1;

    /* CAP0 from pad 19 */
    pad_matrix(qts, IO_MUX_GPIO19);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM0_CAP0), IN_SIG_IN_SEL | 19);
    wr(qts, CAP_TIMER_CFG, 1);
    wr(qts, CAP_CH_CFG(0), 1 | (1 << 2));

    qtest_clock_step(qts, 10 * US);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 1);
    t0 = rd(qts, CAP_CH(0));
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_CAP(0));
    g_assert_cmphex(rd(qts, CAP_STATUS), ==, 0);
    qtest_clock_step(qts, 10 * US);
    /* A falling edge is not selected */
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
    g_assert_cmphex(rd(qts, CAP_CH(0)), ==, t0);
    qtest_clock_step(qts, 15 * US);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 1);
    t1 = rd(qts, CAP_CH(0));
    /* 25 us of APB_CLK, which is the 40 MHz XTAL_CLK after reset */
    g_assert_cmpuint(t1 - t0, ==, 25 * 40);

    /* Both edges, inverted: a falling pad is a rising capture edge */
    wr(qts, CAP_CH_CFG(0), 1 | (3 << 1) | (1u << 11));
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
    g_assert_cmphex(rd(qts, CAP_STATUS), ==, 0);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 1);
    g_assert_cmphex(rd(qts, CAP_STATUS), ==, 1);

    /*
     * Prescale 2: the prescaled signal toggles every second input rising
     * edge, so it rises on the 2nd, 6th, ... input rising edges.
     */
    wr(qts, CAP_CH_CFG(0), 0);
    wr(qts, CAP_CH_CFG(0), 1 | (1 << 2) | (2 << 3));
    wr(qts, INT_CLR, 0xffffffff);
    for (int i = 1; i <= 8; i++) {
        qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 0);
        qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, 1);
        g_assert_cmphex(rd(qts, INT_RAW), ==,
                        (i == 2 || i == 6) ? INT_CAP(0) : 0);
        wr(qts, INT_CLR, 0xffffffff);
    }

    /* Software capture and capture timer sync */
    wr(qts, CAP_TIMER_PHASE, 1000);
    wr(qts, CAP_TIMER_CFG, 1 | 2 | (1 << 5));
    qtest_clock_step(qts, 1 * US);
    wr(qts, CAP_CH_CFG(1), 1u << 12);
    g_assert_cmpuint(rd(qts, CAP_CH(1)), ==, 1040);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_CAP(1));

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_sync(void)
{
    QTestState *qts = start();

    /* Timer 1 reloads with phase 30 at timer 0's TEZ */
    wr(qts, TIMER_SYNC(0), SYNCO_TEZ);
    wr(qts, SYNCI_CFG, 1 << 3);
    wr(qts, TIMER_SYNC(1), SYNCI_EN | PHASE(30));
    wr(qts, TIMER_CFG0(1), CFG0(PRE_1MHZ, 999, 0));
    wr(qts, TIMER_CFG1(1), CFG1(START_RUN, MOD_UP));
    qtest_clock_step(qts, 50 * US);
    timer0_up(qts, 99);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 30);
    qtest_clock_step(qts, 60 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 90);
    qtest_clock_step(qts, 40 * US + 1000 * 100 * US);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 30);

    /* A software sync of timer 1 */
    wr(qts, TIMER_SYNC(1), SYNCI_EN | PHASE(500) | SYNC_SW);
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, 500);

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.mcpwm/test] */
static void test_long_run(void)
{
    QTestState *qts = start();
    const uint64_t period[3] = { 1000, 1001, 1003 };
    uint64_t cycles;

    /* The flash boot watchdogs (TIMG0 and RTC) would reset the chip. */
    qtest_writel(qts, TIMG0_WDTCONFIG0, 0);
    qtest_writel(qts, RTC_WDTWPROTECT, RTC_WDT_WKEY);
    qtest_writel(qts, RTC_WDTCONFIG0, 0);
    qtest_writel(qts, RTC_WDTWPROTECT, 0);

    /*
     * Three timers at 160 MHz with unrelated periods, the third synced
     * to the first's TEZ with phase 7. Ten seconds are 1.6e9 PWM_clk
     * cycles, which the model must cover without stepping through each
     * period.
     */
    wr(qts, TIMER_SYNC(0), SYNCO_TEZ);
    wr(qts, SYNCI_CFG, 1 << 6);
    wr(qts, TIMER_SYNC(2), SYNCI_EN | PHASE(7));
    wr(qts, TIMERSEL, 0 | (1 << 2) | (2 << 4));
    for (int i = 0; i < 3; i++) {
        wr(qts, TIMER_CFG0(i), CFG0(0, period[i], 0));
        wr(qts, OP(i, TSTMP_A), 300);
    }
    for (int i = 0; i < 3; i++) {
        wr(qts, TIMER_CFG1(i), CFG1(START_RUN, MOD_UP));
    }
    qtest_clock_step(qts, 10ULL * 1000 * 1000 * 1000);
    cycles = 1600000000;
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, cycles % (period[0] + 1));
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, cycles % (period[1] + 1));
    g_assert_cmphex(rd(qts, TIMER_STATUS(2)), ==,
                    7 + cycles % (period[0] + 1));

    /* Now generating edges on all six outputs, for 10 ms */
    for (int i = 0; i < 3; i++) {
        wr(qts, OP(i, GEN_A), ACT(UTEZ, HIGH) | ACT(UTEA, LOW));
        wr(qts, OP(i, GEN_B), ACT(UTEP, TOGGLE));
    }
    qtest_clock_step(qts, 10 * 1000 * US);
    cycles += 1600000;
    g_assert_cmphex(rd(qts, TIMER_STATUS(0)), ==, cycles % (period[0] + 1));
    g_assert_cmphex(rd(qts, TIMER_STATUS(1)), ==, cycles % (period[1] + 1));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32-mcpwm/reset", test_reset);
    qtest_add_func("esp32-mcpwm/counting", test_counting);
    qtest_add_func("esp32-mcpwm/interrupts", test_interrupts);
    qtest_add_func("esp32-mcpwm/shadow", test_shadow);
    qtest_add_func("esp32-mcpwm/generator", test_generator);
    qtest_add_func("esp32-mcpwm/deadtime", test_deadtime);
    qtest_add_func("esp32-mcpwm/carrier", test_carrier);
    qtest_add_func("esp32-mcpwm/fault", test_fault);
    qtest_add_func("esp32-mcpwm/capture", test_capture);
    qtest_add_func("esp32-mcpwm/sync", test_sync);
    qtest_add_func("esp32-mcpwm/long-run", test_long_run);

    return g_test_run();
}
