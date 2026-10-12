/*
 * QTest testcase for the ESP32-P4 LED PWM controller and MCPWM0/1
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The blocks run from XTAL_CLK (40 MHz, 25 ns a cycle), whose period QEMU
 * holds exactly, so that every edge and count below is exact to the
 * nanosecond. The board is strapped for Joint Download Boot, so that the
 * flash boot watchdogs do not run.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SOC_CLK_CTRL2       (HP_CLKRST + 0x1c)
#define SOC_CLK_CTRL3       (HP_CLKRST + 0x20)
#define PERI_CLK_CTRL20     (HP_CLKRST + 0x94)
#define PERI_CLK_CTRL22     (HP_CLKRST + 0x9c)
#define HP_RST_EN1          (HP_CLKRST + 0xc4)
#define MCPWM0_APB_CLK_EN   (1u << 27)
#define MCPWM1_APB_CLK_EN   (1u << 28)
#define LEDC_APB_CLK_EN     (1u << 0)
#define ETM_APB_CLK_EN      (1u << 2)
#define ETM_SYS_CLK_EN      (1u << 24)
#define MCPWM0_CLK_EN       (1u << 2)
#define MCPWM0_DIV(d)       ((uint32_t)(d) << 3)
#define PCNT_APB_CLK_EN     (1u << 30)
#define MCPWM1_CLK_EN       (1u << 13)
#define MCPWM_CLK_MASK      0x003fffffu
#define LEDC_CLK_EN         (1u << 2)
#define RST_EN_PWM0         (1u << 24)
#define RST_EN_LEDC         (1u << 29)

#define LEDC                0x500d3000
#define CH_CONF0(n)         (0x00 + 0x14 * (n))
#define CH_HPOINT(n)        (0x04 + 0x14 * (n))
#define CH_DUTY(n)          (0x08 + 0x14 * (n))
#define CH_CONF1(n)         (0x0c + 0x14 * (n))
#define CH_DUTY_R(n)        (0x10 + 0x14 * (n))
#define TIMER_CONF(x)       (0xa0 + 8 * (x))
#define TIMER_VALUE(x)      (0xa4 + 8 * (x))
#define L_INT_RAW           0xc0
#define L_INT_ST            0xc4
#define L_INT_ENA           0xc8
#define L_INT_CLR           0xcc
#define GAMMA_CONF(n)       (0x100 + 4 * (n))
#define EVT_TASK_EN(r)      (0x120 + 4 * (r))
#define TIMER_CMP(x)        (0x140 + 4 * (x))
#define TIMER_CNT_CAP(x)    (0x150 + 4 * (x))
#define L_CONF              0x170
#define L_DATE              0x174
#define GAMMA_RAM(n, r)     (0x400 + 0x40 * (n) + 4 * (r))

#define SIG_OUT_EN          (1u << 2)
#define IDLE_LV             (1u << 3)
#define PARA_UP             (1u << 4)
#define OVF_NUM(v)          ((uint32_t)(v) << 5)
#define OVF_CNT_EN          (1u << 15)
#define OVF_CNT_RESET       (1u << 16)
#define DUTY_START          (1u << 31)
#define TCONF(res, div)     ((res) | ((uint32_t)(div) << 5))
#define T_PAUSE             (1u << 23)
#define T_RST               (1u << 24)
#define T_PARA_UP           (1u << 26)
#define GAMMA_PAUSE         (1u << 5)
#define GAMMA_RESUME        (1u << 6)
#define RANGE(inc, cyc, scale, num) \
    ((inc) | ((cyc) << 1) | ((scale) << 11) | ((uint32_t)(num) << 21))
#define L_INT_TIMER_OVF(x)  (1u << (x))
#define L_INT_DUTY_END(n)   (1u << (4 + (n)))
#define L_INT_OVF_CNT(n)    (1u << (12 + (n)))

#define PWM(i)              (0x500c0000 + 0x1000 * (i))
#define CLK_CFG             0x000
#define TIMER_CFG0(i)       (0x004 + 0x10 * (i))
#define TIMER_CFG1(i)       (0x008 + 0x10 * (i))
#define TIMER_SYNC(i)       (0x00c + 0x10 * (i))
#define TIMER_STATUS(i)     (0x010 + 0x10 * (i))
#define SYNCI_CFG           0x034
#define TIMERSEL            0x038
#define OP(k, r)            (0x03c + 0x38 * (k) + (r))
#define TSTMP_A             0x04
#define GEN_FORCE           0x10
#define GEN_A               0x14
#define GEN_B               0x18
#define DT_CFG              0x1c
#define DT_FED              0x20
#define DT_RED              0x24
#define FH_CFG0             0x2c
#define FH_CFG1             0x30
#define FH_STATUS           0x34
#define FAULT_DETECT        0x0e4
#define CAP_TIMER_CFG       0x0e8
#define CAP_CH_CFG(n)       (0x0f0 + 4 * (n))
#define CAP_CH(n)           (0x0fc + 4 * (n))
#define CAP_STATUS          0x108
#define UPDATE_CFG          0x10c
#define P_INT_ENA           0x110
#define P_INT_RAW           0x114
#define P_INT_CLR           0x11c
#define EVT_EN              0x120
#define TASK_EN             0x124
#define EVT_EN2             0x128
#define OP_TSTMP_E(k, e)    (0x12c + 8 * (k) + 4 * (e))
#define P_CLK               0x144
#define P_VERSION           0x148

#define CFG0(pre, period, up)   ((pre) | ((period) << 8) | ((up) << 24))
#define CFG1(start, mod)        ((start) | ((mod) << 3))
#define START_RUN           2
#define MOD_UP              1
#define SYNCI_EN            (1u << 0)
#define PHASE(p)            ((p) << 4)
#define ACT(field, act)     ((act) << (field))
#define UTEZ                0
#define UTEA                4
#define LOW                 1
#define HIGH                2
#define P_INT_STOP(i)       (1u << (i))
#define P_INT_FAULT(n)      (1u << (9 + (n)))
#define P_INT_OST(k)        (1u << (24 + (k)))
#define P_INT_CAP(n)        (1u << (27 + (n)))

#define ETM                 0x500d5000
#define ETM_CH_ENA_SET      (ETM + 0x004)
#define ETM_CH_EVT_ID(n)    (ETM + 0x018 + 8 * (n))
#define ETM_CH_TASK_ID(n)   (ETM + 0x01c + 8 * (n))
#define ID_LEDC_EVT_OVF_CNT_PLS(n)  (37 + (n))
#define ID_LEDC_EVT_TIME_OVF(x)     (45 + (x))
#define ID_LEDC_TASK_TIMER_CAP(x)   (37 + (x))
#define ID_LEDC_TASK_TIMER_PAUSE(x) (65 + (x))
#define ID_LEDC_TASK_TIMER_RESUME(x) (61 + (x))
#define ID_PWM0_EVT_TEZ(i)          (63 + (i))
#define ID_PWM0_EVT_TEE1(k)         (90 + (k))
#define ID_PWM0_TASK_CAP(n)         (132 + (n))
#define ID_PWM0_TASK_GEN_STOP       119
#define ID_PWM1_TASK_TIMER_SYN(i)   (142 + (i))

#define GPIO                0x500e0000
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define IO_MUX_PAD          ((1u << 12) | (1u << 9) | (2u << 10))
#define SIG_LEDC(n)         (126 + (n))
#define SIG_PWM0_OUT(k, b)  (89 + 2 * (k) + (b))
#define SIG_PWM0_F(n)       (92 + (n))
#define SIG_PWM1_CAP(n)     (104 + (n))
#define SIG_PCNT_CH0(u)     (141 + (u))
#define GPIO_PATH           "/machine/gpio"

#define PCNT                0x500c9000
#define PCNT_CONF0(u)       (PCNT + 0x0c * (u))
#define PCNT_CNT(u)         (PCNT + 0x30 + 4 * (u))
#define PCNT_CTRL           (PCNT + 0x60)
#define PCNT_POS_INC(ch)    (1u << (18 + 8 * (ch)))
#define PCNT_CNT_RST(u)     (1u << (2 * (u)))

#define INTMTX_STATUS1      0x500d6204
#define SRC_PWM0            38
#define SRC_LEDC            52

#define US                  1000ll

static int64_t now;

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4,strap-mode=0x04");
    uint32_t v;

    /*
     * On an edge of XTAL_CLK, which runs from time 0: the clocks below,
     * and a timer started now, count from it.
     */
    now = qtest_clock_step(qts, 25);
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 MCPWM0_APB_CLK_EN | MCPWM1_APB_CLK_EN);
    qtest_writel(qts, SOC_CLK_CTRL3, qtest_readl(qts, SOC_CLK_CTRL3) |
                 LEDC_APB_CLK_EN | ETM_APB_CLK_EN);
    qtest_writel(qts, SOC_CLK_CTRL1, qtest_readl(qts, SOC_CLK_CTRL1) |
                 ETM_SYS_CLK_EN);
    /* LEDC_CLK and MCPWMn_CLK from XTAL_CLK, undivided */
    qtest_writel(qts, PERI_CLK_CTRL22, LEDC_CLK_EN);
    v = qtest_readl(qts, PERI_CLK_CTRL20) & ~MCPWM_CLK_MASK;
    qtest_writel(qts, PERI_CLK_CTRL20, v | MCPWM0_CLK_EN | MCPWM1_CLK_EN);
    return qts;
}

static void step_to(QTestState *qts, int64_t t)
{
    g_assert_cmpint(t, >=, now);
    now = qtest_clock_step(qts, t - now);
}

static uint32_t lrd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, LEDC + reg);
}

static void lwr(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, LEDC + reg, v);
}

static uint32_t prd(QTestState *qts, int i, uint32_t reg)
{
    return qtest_readl(qts, PWM(i) + reg);
}

static void pwr(QTestState *qts, int i, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, PWM(i) + reg, v);
}

/* Pad @n a GPIO matrix pad, with input enabled, showing output @sig. */
static void pad_out(QTestState *qts, int n, int sig)
{
    qtest_writel(qts, IO_MUX_GPIO(n), IO_MUX_PAD);
    qtest_writel(qts, GPIO_FUNC_OUT(n), sig);
}

static bool source(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS1) >> (src - 32)) & 1;
}

static void etm_connect(QTestState *qts, int ch, int evt, int task)
{
    qtest_writel(qts, ETM_CH_EVT_ID(ch), evt);
    qtest_writel(qts, ETM_CH_TASK_ID(ch), task);
    qtest_writel(qts, ETM_CH_ENA_SET, 1u << ch);
}

/*
 * LEDC timer @x running from now with resolution @res and a whole divider
 * @div: a count every div * 25 ns. As ledc_timer_config: the settings
 * taken while the timer is held in reset, then the reset released.
 */
static void ledc_timer(QTestState *qts, int x, int res, int div)
{
    lwr(qts, TIMER_CONF(x), TCONF(res, div * 256) | T_RST);
    lwr(qts, TIMER_CONF(x), TCONF(res, div * 256));
}

/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_ledc_reset(void)
{
    QTestState *qts = start();

    for (int n = 0; n < 8; n++) {
        g_assert_cmphex(lrd(qts, CH_CONF0(n)), ==, 0);
        g_assert_cmphex(lrd(qts, CH_DUTY(n)), ==, 0);
        g_assert_cmphex(lrd(qts, CH_DUTY_R(n)), ==, 0);
        g_assert_cmphex(lrd(qts, GAMMA_CONF(n)), ==, 0);
    }
    for (int x = 0; x < 4; x++) {
        g_assert_cmphex(lrd(qts, TIMER_CONF(x)), ==, T_RST);
        g_assert_cmphex(lrd(qts, TIMER_VALUE(x)), ==, 0);
        g_assert_cmphex(lrd(qts, TIMER_CMP(x)), ==, 0);
        g_assert_cmphex(lrd(qts, TIMER_CNT_CAP(x)), ==, 0);
    }
    g_assert_cmphex(lrd(qts, L_INT_RAW), ==, 0);
    g_assert_cmphex(lrd(qts, L_INT_ENA), ==, 0);
    g_assert_cmphex(lrd(qts, EVT_TASK_EN(0)), ==, 0);
    g_assert_cmphex(lrd(qts, L_CONF), ==, 0);
    g_assert_cmphex(lrd(qts, L_DATE), ==, 0x2404020);

    /* Field widths; the timer, held in reset, takes its settings at once */
    lwr(qts, CH_HPOINT(1), 0xffffffff);
    g_assert_cmphex(lrd(qts, CH_HPOINT(1)), ==, 0xfffff);
    lwr(qts, CH_DUTY(1), 0xffffffff);
    g_assert_cmphex(lrd(qts, CH_DUTY(1)), ==, 0x1ffffff);
    lwr(qts, TIMER_CONF(1), 0xffffffff);
    g_assert_cmphex(lrd(qts, TIMER_CONF(1)), ==, 0x1ffffff);
    lwr(qts, EVT_TASK_EN(2), 0xffffffff);
    g_assert_cmphex(lrd(qts, EVT_TASK_EN(2)), ==, 0xffffff);
    lwr(qts, L_INT_ENA, 0xffffffff);
    g_assert_cmphex(lrd(qts, L_INT_ENA), ==, 0xfffff);
    lwr(qts, L_CONF, 0xffffffff);
    g_assert_cmphex(lrd(qts, L_CONF), ==, 0x800003ff);
    lwr(qts, GAMMA_RAM(7, 15), 0xffffffff);
    g_assert_cmphex(lrd(qts, GAMMA_RAM(7, 15)), ==, 0x7fffffff);
    lwr(qts, GAMMA_CONF(2), 0xffffffff);
    g_assert_cmphex(lrd(qts, GAMMA_CONF(2)), ==, 0x1f);

    /* Held in reset by HP_SYS_CLKRST, the block takes no writes */
    qtest_writel(qts, HP_RST_EN1, RST_EN_LEDC);
    lwr(qts, CH_HPOINT(3), 0x1234);
    qtest_writel(qts, HP_RST_EN1, 0);
    g_assert_cmphex(lrd(qts, CH_HPOINT(3)), ==, 0);
    g_assert_cmphex(lrd(qts, CH_HPOINT(1)), ==, 0);
    lwr(qts, CH_HPOINT(3), 0x1234);
    g_assert_cmphex(lrd(qts, CH_HPOINT(3)), ==, 0x1234);

    qtest_quit(qts);
}

/*
 * Channel 0 on timer 0, 8 bits at 100 ns a count, a 25.6 us cycle:
 * HPOINT 32 and DUTY 64 give a high from 3.2 us to 9.6 us of each cycle,
 * from the cycle after the update was asked for.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_ledc_pwm(void)
{
    QTestState *qts = start();
    int64_t t0, c;

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    pad_out(qts, 20, SIG_LEDC(0));
    ledc_timer(qts, 0, 8, 4);
    t0 = now;
    lwr(qts, CH_HPOINT(0), 32);
    lwr(qts, CH_DUTY(0), 64 << 4);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | PARA_UP);
    g_assert_cmphex(lrd(qts, CH_CONF0(0)), ==, SIG_OUT_EN | PARA_UP);

    step_to(qts, t0 + 12 * US);
    g_assert_cmpuint(lrd(qts, TIMER_VALUE(0)), ==, 120);
    g_assert_false(qtest_get_irq(qts, 20));
    /* The update at the overflow */
    c = t0 + 25600;
    step_to(qts, c);
    g_assert_cmphex(lrd(qts, CH_CONF0(0)), ==, SIG_OUT_EN);
    g_assert_cmphex(lrd(qts, CH_DUTY_R(0)), ==, 64 << 4);
    step_to(qts, c + 3200 - 1);
    g_assert_false(qtest_get_irq(qts, 20));
    step_to(qts, c + 3200);
    g_assert_true(qtest_get_irq(qts, 20));
    step_to(qts, c + 9600 - 1);
    g_assert_true(qtest_get_irq(qts, 20));
    step_to(qts, c + 9600);
    g_assert_false(qtest_get_irq(qts, 20));
    /* A thousand cycles on, in step */
    c += 1000 * 25600;
    step_to(qts, c + 3200 - 1);
    g_assert_false(qtest_get_irq(qts, 20));
    step_to(qts, c + 3200);
    g_assert_true(qtest_get_irq(qts, 20));
    g_assert_true((qtest_readl(qts, GPIO_IN) >> 20) & 1);

    /*
     * DUTY[3:0] = 8: every other cycle, counted from the update, one
     * count longer.
     */
    lwr(qts, CH_DUTY(0), (64 << 4) | 8);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | PARA_UP);
    c += 25600;
    for (int j = 0; j < 4; j++) {
        int64_t low = c + 25600 * j + 9600 + (j & 1 ? 100 : 0);

        step_to(qts, low - 1);
        g_assert_true(qtest_get_irq(qts, 20));
        step_to(qts, low);
        g_assert_false(qtest_get_irq(qts, 20));
    }

    /* Output off: IDLE_LV at once, SIG_OUT_EN at the next overflow */
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | IDLE_LV);
    lwr(qts, CH_CONF0(0), IDLE_LV | PARA_UP);
    c += 4 * 25600;
    step_to(qts, c - 1);
    g_assert_false(qtest_get_irq(qts, 20));
    step_to(qts, c);
    g_assert_true(qtest_get_irq(qts, 20));
    step_to(qts, c + 50 * US);
    g_assert_true(qtest_get_irq(qts, 20));

    /* Paused, the counter holds */
    lwr(qts, TIMER_CONF(0), TCONF(8, 4 * 256) | T_PAUSE);
    c = lrd(qts, TIMER_VALUE(0));
    step_to(qts, now + 100 * US);
    g_assert_cmpuint(lrd(qts, TIMER_VALUE(0)), ==, c);

    qtest_quit(qts);
}

/* The duty, DUTY_R[24:4], in the middle of cycle @k after @c0. */
static uint32_t duty_in(QTestState *qts, int64_t c0, int k)
{
    step_to(qts, c0 + 25600ll * k + 12800);
    return lrd(qts, CH_DUTY_R(0)) >> 4;
}

/*
 * Fades through the gamma RAM: two ranges, up 4 every 2 cycles 3 times,
 * then down 2 every cycle twice; the end interrupt as the last range
 * ends; then a fade paused and resumed.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_ledc_fade(void)
{
    static const uint32_t want[] = { 64, 64, 68, 68, 72, 72, 76, 74, 72, 72 };
    QTestState *qts = start();
    int64_t c0;

    ledc_timer(qts, 0, 8, 4);
    lwr(qts, GAMMA_RAM(0, 0), RANGE(1, 2, 4, 3));
    lwr(qts, GAMMA_RAM(0, 1), RANGE(0, 1, 2, 2));
    lwr(qts, GAMMA_CONF(0), 2);
    lwr(qts, CH_HPOINT(0), 0);
    lwr(qts, CH_DUTY(0), 64 << 4);
    lwr(qts, L_INT_ENA, L_INT_DUTY_END(0));
    lwr(qts, CH_CONF1(0), DUTY_START);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | PARA_UP);
    c0 = now + 25600;
    for (int k = 0; k < ARRAY_SIZE(want); k++) {
        g_assert_cmpuint(duty_in(qts, c0, k), ==, want[k]);
        g_assert_cmpint(source(qts, SRC_LEDC), ==, k >= 8);
    }
    lwr(qts, L_INT_CLR, 0xffffffff);
    g_assert_false(source(qts, SRC_LEDC));
    g_assert_cmphex(lrd(qts, CH_CONF1(0)), ==, 0);
    lwr(qts, L_INT_ENA, 0);

    /* One step a cycle, paused in cycle 3 and resumed in cycle 7 */
    lwr(qts, GAMMA_RAM(0, 0), RANGE(1, 1, 1, 20));
    lwr(qts, GAMMA_CONF(0), 1);
    lwr(qts, CH_DUTY(0), 10 << 4);
    lwr(qts, CH_CONF1(0), DUTY_START);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | PARA_UP);
    c0 += 10 * 25600;
    g_assert_cmpuint(duty_in(qts, c0, 3), ==, 13);
    lwr(qts, GAMMA_CONF(0), 1 | GAMMA_PAUSE);
    g_assert_cmphex(lrd(qts, GAMMA_CONF(0)), ==, 1 | GAMMA_PAUSE);
    for (int k = 4; k < 8; k++) {
        g_assert_cmpuint(duty_in(qts, c0, k), ==, 13);
    }
    lwr(qts, GAMMA_CONF(0), 1 | GAMMA_RESUME);
    g_assert_cmphex(lrd(qts, GAMMA_CONF(0)), ==, 1);
    g_assert_cmpuint(duty_in(qts, c0, 8), ==, 14);
    g_assert_cmpuint(duty_in(qts, c0, 9), ==, 15);

    qtest_quit(qts);
}

/*
 * The timer's overflow interrupt as the count reaches its top; a
 * channel's overflow count interrupt every OVF_NUM + 1 overflows from
 * its update.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_ledc_overflow(void)
{
    QTestState *qts = start();
    int64_t t0;

    ledc_timer(qts, 0, 8, 4);
    t0 = now;
    lwr(qts, L_INT_ENA, L_INT_TIMER_OVF(0) | L_INT_OVF_CNT(0));
    lwr(qts, CH_CONF0(0), OVF_NUM(2) | OVF_CNT_EN | PARA_UP);
    step_to(qts, t0 + 25500 - 1);
    g_assert_cmphex(lrd(qts, L_INT_RAW), ==, 0);
    g_assert_false(source(qts, SRC_LEDC));
    step_to(qts, t0 + 25500);
    g_assert_cmphex(lrd(qts, L_INT_RAW), ==, L_INT_TIMER_OVF(0));
    g_assert_true(source(qts, SRC_LEDC));
    g_assert_cmphex(lrd(qts, L_INT_ST), ==, L_INT_TIMER_OVF(0));
    lwr(qts, L_INT_CLR, L_INT_TIMER_OVF(0));
    lwr(qts, L_INT_ENA, L_INT_OVF_CNT(0));

    /* Counting from the update at 25.6 us: cycles 1, 2 and 3 */
    step_to(qts, t0 + 3 * 25600 + 25500 - 1);
    g_assert_false(source(qts, SRC_LEDC));
    step_to(qts, t0 + 3 * 25600 + 25500);
    g_assert_true(source(qts, SRC_LEDC));
    lwr(qts, L_INT_CLR, 0xffffffff);
    step_to(qts, t0 + 6 * 25600 + 25500 - 1);
    g_assert_false(source(qts, SRC_LEDC));
    step_to(qts, t0 + 6 * 25600 + 25500);
    g_assert_true(source(qts, SRC_LEDC));
    lwr(qts, L_INT_CLR, 0xffffffff);
    /* Cycle 7's overflow counted, then the count reset: cycles 8 to 10 */
    step_to(qts, t0 + 8 * 25600);
    lwr(qts, CH_CONF0(0), OVF_NUM(2) | OVF_CNT_EN | OVF_CNT_RESET);
    step_to(qts, t0 + 9 * 25600 + 25500);
    g_assert_false(source(qts, SRC_LEDC));
    step_to(qts, t0 + 10 * 25600 + 25500 - 1);
    g_assert_false(source(qts, SRC_LEDC));
    step_to(qts, t0 + 10 * 25600 + 25500);
    g_assert_true(source(qts, SRC_LEDC));

    qtest_quit(qts);
}

/*
 * ETM: channel 0's overflow count pauses its timer, so that exactly
 * three pulses come out (ESP-IDF's "ledc etm event and task" test); the
 * timer's overflow event captures timer 1's count.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_ledc_etm(void)
{
    QTestState *qts = start();
    int64_t t0;

    ledc_timer(qts, 1, 10, 1);
    ledc_timer(qts, 0, 8, 4);
    t0 = now;
    lwr(qts, CH_HPOINT(0), 32);
    lwr(qts, CH_DUTY(0), 64 << 4);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | OVF_NUM(2) | OVF_CNT_EN | PARA_UP);
    lwr(qts, EVT_TASK_EN(0), (1u << 8) | (1u << 16));
    lwr(qts, EVT_TASK_EN(1), (1u << 28) | (1u << 5));
    etm_connect(qts, 0, ID_LEDC_EVT_OVF_CNT_PLS(0),
                ID_LEDC_TASK_TIMER_PAUSE(0));
    etm_connect(qts, 1, ID_LEDC_EVT_TIME_OVF(0), ID_LEDC_TASK_TIMER_CAP(1));

    /* Timer 1 counts every 25 ns: at 25.5 us it has counted 1020 */
    step_to(qts, t0 + 25500);
    g_assert_cmpuint(lrd(qts, TIMER_CNT_CAP(1)), ==, 1020);
    step_to(qts, t0 + 2 * 25600 + 25500);
    g_assert_cmpuint(lrd(qts, TIMER_CNT_CAP(1)), ==, (2 * 1024 + 1020) % 1024);
    /* Cycles 1 to 3 counted: paused at cycle 3's top */
    step_to(qts, t0 + 3 * 25600 + 25500);
    step_to(qts, now + 200 * US);
    g_assert_cmpuint(lrd(qts, TIMER_VALUE(0)), ==, 255);
    g_assert_cmphex(lrd(qts, TIMER_CONF(0)) & T_PAUSE, ==, 0);

    /* The other task of the pair resumes it */
    etm_connect(qts, 2, ID_LEDC_EVT_TIME_OVF(1),
                ID_LEDC_TASK_TIMER_RESUME(0));
    lwr(qts, EVT_TASK_EN(0), (1u << 8) | (1u << 17));
    step_to(qts, now + 26 * US);
    g_assert_cmpuint(lrd(qts, TIMER_VALUE(0)), !=, 255);

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_reset(void)
{
    QTestState *qts = start();

    for (int i = 0; i < 2; i++) {
        g_assert_cmphex(prd(qts, i, P_VERSION), ==, 0x2212290);
        g_assert_cmphex(prd(qts, i, P_CLK), ==, 0);
        g_assert_cmphex(prd(qts, i, EVT_EN), ==, 0);
        g_assert_cmphex(prd(qts, i, TASK_EN), ==, 0);
        g_assert_cmphex(prd(qts, i, EVT_EN2), ==, 0);
        g_assert_cmphex(prd(qts, i, TIMER_CFG0(0)), ==, 0xff00);
        g_assert_cmphex(prd(qts, i, UPDATE_CFG), ==, 0x55);
        g_assert_cmphex(prd(qts, i, OP(0, DT_CFG)), ==, 0x18000);
    }
    pwr(qts, 0, EVT_EN, 0xffffffff);
    g_assert_cmphex(prd(qts, 0, EVT_EN), ==, 0x3fffffff);
    pwr(qts, 0, TASK_EN, 0xffffffff);
    g_assert_cmphex(prd(qts, 0, TASK_EN), ==, 0x3fffff);
    pwr(qts, 0, EVT_EN2, 0xffffffff);
    g_assert_cmphex(prd(qts, 0, EVT_EN2), ==, 0x3f);
    pwr(qts, 0, OP_TSTMP_E(2, 1), 0xffffffff);
    g_assert_cmphex(prd(qts, 0, OP_TSTMP_E(2, 1)), ==, 0xffff);
    pwr(qts, 0, P_CLK, 0xffffffff);
    g_assert_cmphex(prd(qts, 0, P_CLK), ==, 1);

    /* Held in reset by HP_SYS_CLKRST, the block takes no writes */
    qtest_writel(qts, HP_RST_EN1, RST_EN_PWM0);
    pwr(qts, 0, CLK_CFG, 7);
    qtest_writel(qts, HP_RST_EN1, 0);
    g_assert_cmphex(prd(qts, 0, CLK_CFG), ==, 0);
    g_assert_cmphex(prd(qts, 0, EVT_EN), ==, 0);

    qtest_quit(qts);
}

/*
 * PWM_clk is MCPWMn_CLK, here XTAL_CLK divided by HP_SYS_CLKRST's 4, by
 * CLK_PRESCALE + 1; the capture timer counts MCPWMn_CLK itself.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_clock(void)
{
    QTestState *qts = start();
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL20) & ~MCPWM_CLK_MASK;
    uint32_t c0;
    int64_t t0;

    qtest_writel(qts, PERI_CLK_CTRL20, v | MCPWM0_CLK_EN | MCPWM0_DIV(3));
    /* 10 MHz, by 2: 5 MHz; the timer by 5: 1 MHz */
    pwr(qts, 0, CLK_CFG, 1);
    pwr(qts, 0, TIMER_CFG0(0), CFG0(4, 999, 0));
    pwr(qts, 0, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    pwr(qts, 0, CAP_TIMER_CFG, 1);
    t0 = now;
    pwr(qts, 0, CAP_CH_CFG(0), 1u << 12);
    c0 = prd(qts, 0, CAP_CH(0));
    step_to(qts, t0 + 250 * US);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 250);
    pwr(qts, 0, CAP_CH_CFG(0), 1u << 12);
    g_assert_cmpuint(prd(qts, 0, CAP_CH(0)) - c0, ==, 2500);

    /* With its clock stopped the block stands still */
    qtest_writel(qts, PERI_CLK_CTRL20, v);
    step_to(qts, now + 100 * US);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 250);

    qtest_quit(qts);
}

/*
 * Complementary outputs with dead time on pads 21 and 22, PWM_clk at
 * 1 MHz: PWM0A's rising edge delayed by RED + 1, PWM0B the inverse with
 * its rising edge delayed by FED + 1; a fault from pad 23 trips them.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_deadtime(void)
{
    QTestState *qts = start();
    int64_t t0;

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    pad_out(qts, 21, SIG_PWM0_OUT(0, 0));
    pad_out(qts, 22, SIG_PWM0_OUT(0, 1));
    pwr(qts, 0, CLK_CFG, 39);
    pwr(qts, 0, OP(0, TSTMP_A), 50);
    pwr(qts, 0, OP(0, GEN_A), ACT(UTEZ, HIGH) | ACT(UTEA, LOW));
    pwr(qts, 0, OP(0, DT_RED), 4);
    pwr(qts, 0, OP(0, DT_FED), 9);
    pwr(qts, 0, OP(0, DT_CFG), 1u << 14);
    pwr(qts, 0, TIMER_CFG0(0), CFG0(0, 99, 0));
    pwr(qts, 0, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    t0 = now;
    g_assert_false(qtest_get_irq(qts, 21));
    g_assert_true(qtest_get_irq(qts, 22));

    step_to(qts, t0 + 100 * US);
    g_assert_false(qtest_get_irq(qts, 21));
    g_assert_false(qtest_get_irq(qts, 22));
    step_to(qts, t0 + 105 * US - 1);
    g_assert_false(qtest_get_irq(qts, 21));
    step_to(qts, t0 + 105 * US);
    g_assert_true(qtest_get_irq(qts, 21));
    step_to(qts, t0 + 150 * US);
    g_assert_false(qtest_get_irq(qts, 21));
    g_assert_false(qtest_get_irq(qts, 22));
    step_to(qts, t0 + 160 * US - 1);
    g_assert_false(qtest_get_irq(qts, 22));
    step_to(qts, t0 + 160 * US);
    g_assert_true(qtest_get_irq(qts, 22));

    /* FAULT0 from pad 23, active high: a one-shot trip holds A low */
    qtest_writel(qts, IO_MUX_GPIO(23), IO_MUX_PAD);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 23, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM0_F(0)), IN_SIG_IN_SEL | 23);
    pwr(qts, 0, FAULT_DETECT, 1 | (1 << 3));
    pwr(qts, 0, OP(0, FH_CFG0), (1u << 7) | (LOW << 12) | (LOW << 14));
    step_to(qts, t0 + 220 * US);
    g_assert_true(qtest_get_irq(qts, 21));
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 23, 1);
    g_assert_false(qtest_get_irq(qts, 21));
    g_assert_cmphex(prd(qts, 0, OP(0, FH_STATUS)), ==, 2);
    g_assert_cmphex(prd(qts, 0, P_INT_RAW) & (P_INT_FAULT(0) | P_INT_OST(0)),
                    ==, P_INT_FAULT(0) | P_INT_OST(0));
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 23, 0);
    step_to(qts, t0 + 320 * US);
    g_assert_false(qtest_get_irq(qts, 21));
    pwr(qts, 0, OP(0, FH_CFG1), 1);
    step_to(qts, t0 + 410 * US);
    g_assert_true(qtest_get_irq(qts, 21));

    qtest_quit(qts);
}

/*
 * MCPWM1's capture channel 0 times LEDC channel 0 through pad 20, at
 * 40 MHz: a 6.4 us pulse of 25.6 us.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_capture_ledc(void)
{
    QTestState *qts = start();
    uint32_t r1, f1, r2;
    int64_t c;

    pad_out(qts, 20, SIG_LEDC(0));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM1_CAP(0)), IN_SIG_IN_SEL | 20);
    pwr(qts, 1, CAP_TIMER_CFG, 1);
    pwr(qts, 1, CAP_CH_CFG(0), 1 | (3 << 1));
    ledc_timer(qts, 0, 8, 4);
    c = now + 25600;
    lwr(qts, CH_HPOINT(0), 32);
    lwr(qts, CH_DUTY(0), 64 << 4);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | PARA_UP);

    step_to(qts, c + 3200);
    r1 = prd(qts, 1, CAP_CH(0));
    g_assert_cmphex(prd(qts, 1, CAP_STATUS), ==, 0);
    step_to(qts, c + 9600);
    f1 = prd(qts, 1, CAP_CH(0));
    g_assert_cmphex(prd(qts, 1, CAP_STATUS), ==, 1);
    step_to(qts, c + 25600 + 3200);
    r2 = prd(qts, 1, CAP_CH(0));
    g_assert_cmpuint(f1 - r1, ==, 256);
    g_assert_cmpuint(r2 - r1, ==, 1024);
    g_assert_cmphex(prd(qts, 1, P_INT_RAW), ==, P_INT_CAP(0));

    qtest_quit(qts);
}

/*
 * ETM: MCPWM0 timer 0's TEZ syncs MCPWM1's timer 0 to its phase; the
 * event comparator E1 of operator 0 has MCPWM0 capture its own timer;
 * and LEDC timer 0's overflow stops MCPWM0's timers (GEN_STOP).
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_etm(void)
{
    QTestState *qts = start();
    uint32_t c0;
    int64_t t0;

    /* 40 MHz / 40: 1 us counts for both blocks' timers */
    for (int i = 0; i < 2; i++) {
        pwr(qts, i, CLK_CFG, 39);
        pwr(qts, i, TIMER_CFG0(0), CFG0(0, i ? 999 : 99, 0));
    }
    pwr(qts, 1, TIMER_SYNC(0), SYNCI_EN | PHASE(500));
    pwr(qts, 0, OP_TSTMP_E(0, 0), 30);
    pwr(qts, 0, CAP_TIMER_CFG, 1);
    pwr(qts, 0, EVT_EN, 1u << 3);
    pwr(qts, 0, EVT_EN2, 1u << 0);
    pwr(qts, 0, TASK_EN, (1u << 19) | (1u << 6));
    pwr(qts, 1, TASK_EN, 1u << 7);
    etm_connect(qts, 0, ID_PWM0_EVT_TEZ(0), ID_PWM1_TASK_TIMER_SYN(0));
    etm_connect(qts, 1, ID_PWM0_EVT_TEE1(0), ID_PWM0_TASK_CAP(0));
    pwr(qts, 1, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    pwr(qts, 0, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    t0 = now;
    pwr(qts, 0, CAP_CH_CFG(1), 1u << 12);
    c0 = prd(qts, 0, CAP_CH(1));

    /* TEZ at 100 us: MCPWM1's timer 0 at 500, then on */
    step_to(qts, t0 + 99 * US);
    g_assert_cmpuint(prd(qts, 1, TIMER_STATUS(0)), ==, 99);
    step_to(qts, t0 + 100 * US);
    g_assert_cmpuint(prd(qts, 1, TIMER_STATUS(0)), ==, 500);
    step_to(qts, t0 + 120 * US);
    g_assert_cmpuint(prd(qts, 1, TIMER_STATUS(0)), ==, 520);
    /* E1 at count 30: captured at 30 us, 130 us, ... */
    step_to(qts, t0 + 135 * US);
    g_assert_cmpuint(prd(qts, 0, CAP_CH(0)) - c0, ==, 130 * 40);
    g_assert_cmphex(prd(qts, 0, P_INT_RAW) & P_INT_CAP(0), ==, P_INT_CAP(0));

    /* GEN_STOP from LEDC timer 0's overflow at 25.5 us from its start */
    pwr(qts, 0, P_INT_CLR, 0xffffffff);
    lwr(qts, EVT_TASK_EN(0), 1u << 16);
    etm_connect(qts, 2, ID_LEDC_EVT_TIME_OVF(0), ID_PWM0_TASK_GEN_STOP);
    ledc_timer(qts, 0, 8, 4);
    step_to(qts, now + 25500);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 60);
    g_assert_cmphex(prd(qts, 0, P_INT_RAW) & P_INT_STOP(0), ==,
                    P_INT_STOP(0));
    step_to(qts, now + 50 * US);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 60);

    qtest_quit(qts);
}

/*
 * PWM_clk at 40 MHz, XTAL_CLK undivided: every edge comes at its own
 * cycle however close it is to others. In a 1 us period, A is high from
 * TEZ for 4 cycles and rises 2 cycles late (RED 1); B, its inverse, rises
 * 3 cycles late (FED 2): B falls at TEZ, A rises 50 ns later and falls
 * at 100 ns, and B rises at 175 ns. The pads show each edge at its
 * nanosecond; MCPWM1's capture channels, counting XTAL_CLK, time them;
 * PCNT unit 0 counts every rising edge of A.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_close_edges(void)
{
    QTestState *qts = start();
    uint32_t c0, c1, c2;
    int64_t t0, t;

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    pad_out(qts, 21, SIG_PWM0_OUT(0, 0));
    pad_out(qts, 22, SIG_PWM0_OUT(0, 1));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM1_CAP(0)), IN_SIG_IN_SEL | 21);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM1_CAP(1)), IN_SIG_IN_SEL | 21);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM1_CAP(2)), IN_SIG_IN_SEL | 22);
    pwr(qts, 1, CAP_TIMER_CFG, 1);
    pwr(qts, 1, CAP_CH_CFG(0), 1 | (1 << 2));
    pwr(qts, 1, CAP_CH_CFG(1), 1 | (1 << 1));
    pwr(qts, 1, CAP_CH_CFG(2), 1 | (1 << 2));
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 PCNT_APB_CLK_EN);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PCNT_CH0(0)), IN_SIG_IN_SEL | 21);
    /* Rising edges counted up, unfiltered */
    qtest_writel(qts, PCNT_CONF0(0), PCNT_POS_INC(0));
    qtest_writel(qts, PCNT_CTRL, PCNT_CNT_RST(0));
    qtest_writel(qts, PCNT_CTRL, 0);

    pwr(qts, 0, CLK_CFG, 0);
    pwr(qts, 0, OP(0, TSTMP_A), 4);
    pwr(qts, 0, OP(0, GEN_A), ACT(UTEZ, HIGH) | ACT(UTEA, LOW));
    pwr(qts, 0, OP(0, DT_RED), 1);
    pwr(qts, 0, OP(0, DT_FED), 2);
    pwr(qts, 0, OP(0, DT_CFG), 1u << 14);
    pwr(qts, 0, TIMER_CFG0(0), CFG0(0, 39, 0));
    pwr(qts, 0, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    t0 = now;

    for (int m = 1; m <= 3; m++) {
        t = t0 + m * US;
        step_to(qts, t - 1);
        g_assert_true(qtest_get_irq(qts, 22));
        g_assert_false(qtest_get_irq(qts, 21));
        step_to(qts, t);
        g_assert_false(qtest_get_irq(qts, 22));
        step_to(qts, t + 49);
        g_assert_false(qtest_get_irq(qts, 21));
        step_to(qts, t + 50);
        g_assert_true(qtest_get_irq(qts, 21));
        step_to(qts, t + 99);
        g_assert_true(qtest_get_irq(qts, 21));
        step_to(qts, t + 100);
        g_assert_false(qtest_get_irq(qts, 21));
        step_to(qts, t + 174);
        g_assert_false(qtest_get_irq(qts, 22));
        step_to(qts, t + 175);
        g_assert_true(qtest_get_irq(qts, 22));
    }
    c0 = prd(qts, 1, CAP_CH(0));
    c1 = prd(qts, 1, CAP_CH(1));
    c2 = prd(qts, 1, CAP_CH(2));
    g_assert_cmpuint(c1 - c0, ==, 2);
    g_assert_cmpuint(c2 - c1, ==, 3);

    /* A thousand periods on: a thousand and three rising edges of A */
    step_to(qts, t0 + 1003 * US + 500);
    g_assert_cmpuint(qtest_readl(qts, PCNT_CNT(0)), ==, 1003);
    g_assert_cmpuint(prd(qts, 1, CAP_CH(1)) - prd(qts, 1, CAP_CH(0)), ==, 2);

    qtest_quit(qts);
}

/*
 * A change of PWM_clk's rate keeps the PWM_clk cycle in progress: the
 * divider keeps the MCPWM0_CLK cycles it has counted into it, and counts
 * the rest at the new rate. MCPWM0_CLK at 10 MHz (XTAL_CLK by 4), by
 * CLK_PRESCALE + 1 = 10: PWM_clk 1 MHz, timer 0 counting it.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_mcpwm_rate_change(void)
{
    QTestState *qts = start();
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL20) & ~MCPWM_CLK_MASK;
    int64_t t0, tc;

    qtest_writel(qts, PERI_CLK_CTRL20, v | MCPWM0_CLK_EN | MCPWM0_DIV(3));
    pwr(qts, 0, CLK_CFG, 9);
    pwr(qts, 0, TIMER_CFG0(0), CFG0(0, 999, 0));
    pwr(qts, 0, TIMER_CFG1(0), CFG1(START_RUN, MOD_UP));
    t0 = now;
    step_to(qts, t0 + 13 * US - 1);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 12);
    step_to(qts, t0 + 13 * US);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 13);

    /*
     * 250 ns into PWM_clk cycle 14, two and a half MCPWM0_CLK cycles,
     * MCPWM0_CLK doubles to 20 MHz: the two whole cycles stay counted,
     * the eight left take 400 ns, and the clock's own cycles start afresh
     * from the change, as every block counting it takes them.
     */
    tc = t0 + 13 * US + 250;
    step_to(qts, tc);
    qtest_writel(qts, PERI_CLK_CTRL20, v | MCPWM0_CLK_EN | MCPWM0_DIV(1));
    step_to(qts, tc + 399);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 13);
    step_to(qts, tc + 400);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 14);
    step_to(qts, tc + 900);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 15);

    /*
     * 175 ns into cycle 18 (at tc + 1900), three and a half 50 ns cycles,
     * CLK_PRESCALE becomes 4: MCPWM0_CLK runs on as it was, and the cycle
     * ends at its fifth, 250 ns in; from there every 250 ns.
     */
    step_to(qts, tc + 2075);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 17);
    pwr(qts, 0, CLK_CFG, 4);
    step_to(qts, tc + 2149);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 17);
    step_to(qts, tc + 2150);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 18);
    step_to(qts, tc + 2400);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 19);

    /*
     * Three cycles into the next (175 ns after tc + 2650), CLK_PRESCALE
     * becomes 1, a division by 2 that those three cycles already exceed:
     * the cycle ends at the next MCPWM0_CLK cycle, then every 100 ns.
     */
    step_to(qts, tc + 2825);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 20);
    pwr(qts, 0, CLK_CFG, 1);
    step_to(qts, tc + 2849);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 20);
    step_to(qts, tc + 2850);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 21);
    step_to(qts, tc + 2950);
    g_assert_cmpuint(prd(qts, 0, TIMER_STATUS(0)), ==, 22);

    qtest_quit(qts);
}

/*
 * LEDC channel 0 from XTAL_CLK undivided, a count every 25 ns, at a 4-bit
 * resolution: a 400 ns period, high from HPOINT 2 for DUTY 3 counts, so
 * that it rises 50 ns into each period and falls 75 ns later. The pad
 * shows each edge at its nanosecond; MCPWM1's capture channels, counting
 * XTAL_CLK too, time the pulse; PCNT unit 0 counts every rising edge.
 */
/* [spec:nuos:req:emu.esp32p4.pwm/test] */
static void test_ledc_close_edges(void)
{
    QTestState *qts = start();
    int64_t c;

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    pad_out(qts, 20, SIG_LEDC(0));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM1_CAP(0)), IN_SIG_IN_SEL | 20);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PWM1_CAP(1)), IN_SIG_IN_SEL | 20);
    pwr(qts, 1, CAP_TIMER_CFG, 1);
    pwr(qts, 1, CAP_CH_CFG(0), 1 | (1 << 2));
    pwr(qts, 1, CAP_CH_CFG(1), 1 | (1 << 1));
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 PCNT_APB_CLK_EN);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_PCNT_CH0(0)), IN_SIG_IN_SEL | 20);
    /* Rising edges counted up, unfiltered */
    qtest_writel(qts, PCNT_CONF0(0), PCNT_POS_INC(0));
    qtest_writel(qts, PCNT_CTRL, PCNT_CNT_RST(0));
    qtest_writel(qts, PCNT_CTRL, 0);

    ledc_timer(qts, 0, 4, 1);
    /* The update at the first overflow */
    c = now + 400;
    lwr(qts, CH_HPOINT(0), 2);
    lwr(qts, CH_DUTY(0), 3 << 4);
    lwr(qts, CH_CONF0(0), SIG_OUT_EN | PARA_UP);
    for (int m = 0; m < 3; m++) {
        int64_t t = c + 400 * m;

        step_to(qts, t + 49);
        g_assert_false(qtest_get_irq(qts, 20));
        step_to(qts, t + 50);
        g_assert_true(qtest_get_irq(qts, 20));
        step_to(qts, t + 124);
        g_assert_true(qtest_get_irq(qts, 20));
        step_to(qts, t + 125);
        g_assert_false(qtest_get_irq(qts, 20));
    }
    g_assert_cmpuint(prd(qts, 1, CAP_CH(1)) - prd(qts, 1, CAP_CH(0)), ==, 3);

    /* Into the 1004th period: a rising edge in each */
    step_to(qts, c + 1003 * 400 + 200);
    g_assert_cmpuint(qtest_readl(qts, PCNT_CNT(0)), ==, 1004);
    g_assert_cmpuint(prd(qts, 1, CAP_CH(1)) - prd(qts, 1, CAP_CH(0)), ==, 3);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4-pwm/ledc/reset", test_ledc_reset);
    qtest_add_func("esp32p4-pwm/ledc/pwm", test_ledc_pwm);
    qtest_add_func("esp32p4-pwm/ledc/fade", test_ledc_fade);
    qtest_add_func("esp32p4-pwm/ledc/overflow", test_ledc_overflow);
    qtest_add_func("esp32p4-pwm/ledc/etm", test_ledc_etm);
    qtest_add_func("esp32p4-pwm/ledc/close-edges", test_ledc_close_edges);
    qtest_add_func("esp32p4-pwm/mcpwm/reset", test_mcpwm_reset);
    qtest_add_func("esp32p4-pwm/mcpwm/clock", test_mcpwm_clock);
    qtest_add_func("esp32p4-pwm/mcpwm/deadtime", test_mcpwm_deadtime);
    qtest_add_func("esp32p4-pwm/mcpwm/capture-ledc", test_mcpwm_capture_ledc);
    qtest_add_func("esp32p4-pwm/mcpwm/etm", test_mcpwm_etm);
    qtest_add_func("esp32p4-pwm/mcpwm/close-edges", test_mcpwm_close_edges);
    qtest_add_func("esp32p4-pwm/mcpwm/rate-change", test_mcpwm_rate_change);

    return g_test_run();
}
