/*
 * QTest testcase for the ESP32-P4 event task matrix (SOC_ETM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Events come from the qtest-driven "event" lines of the ETM, or from the
 * blocks the machine connects; tasks are seen in TASK_STn and in what
 * they do, most often GPIO_EXT's toggle, set and clear of GPIO_OUT, a
 * toggle telling each generated task apart.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define ETM                 0x500d5000
#define CH_ENA_AD0          0x000
#define CH_ENA_AD0_SET      0x004
#define CH_ENA_AD0_CLR      0x008
#define CH_ENA_AD1          0x00c
#define CH_ENA_AD1_SET      0x010
#define CH_ENA_AD1_CLR      0x014
#define CH_EVT_ID(n)        (0x018 + 8 * (n))
#define CH_TASK_ID(n)       (0x01c + 8 * (n))
#define EVT_ST(k)           (0x1a8 + 8 * (k))
#define EVT_ST_CLR(k)       (0x1ac + 8 * (k))
#define TASK_ST(k)          (0x1e8 + 8 * (k))
#define TASK_ST_CLR(k)      (0x1ec + 8 * (k))
#define CLK_EN              0x220
#define DATE                0x224

#define ETM_PATH            "/machine/etm"
#define GPIO_PATH           "/machine/gpio"

/* Event and task IDs, soc_etm_source.h's. */
#define GPIO_EVT_CH_RISE(c)     (1 + (c))
#define GPIO_EVT_CH_FALL(c)     (9 + (c))
#define GPIO_EVT_CH_ANY(c)      (17 + (c))
#define GPIO_EVT_ZERO_DET_POS0  25
#define GPIO_EVT_ZERO_DET_NEG0  26
#define TG0_EVT_CMP_T0          53
#define SYSTIMER_EVT_CMP(x)     (57 + (x))
#define RTC_EVT_TICK            164
#define RTC_EVT_CMP             166
#define GPIO_TASK_SET(c)        (1 + (c))
#define GPIO_TASK_CLEAR(c)      (9 + (c))
#define GPIO_TASK_TOGGLE(c)     (17 + (c))
#define TG0_TASK_CNT_CAP_T1     102
#define ULP_TASK_INT_CPU        180

#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SOC_CLK_CTRL3       (HP_CLKRST + 0x20)
#define HP_RST_EN1          (HP_CLKRST + 0xc4)
#define ETM_SYS_CLK_EN      (1u << 24)
#define ETM_APB_CLK_EN      (1u << 2)
#define RST_EN_ETM          (1u << 31)

#define GPIO                0x500e0000
#define GPIO_OUT            (GPIO + 0x004)
#define GPIO_OUT_W1TC       (GPIO + 0x00c)
#define GPIO_INT_RAW        (GPIO + 0x700)
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define GPIO_ETM_EVENT(c)   (GPIO + 0xf60 + 4 * (c))
#define GPIO_ETM_TASK(n)    (GPIO + 0xfa0 + 4 * ((n) / 4))
#define ETM_EVENT_EN        (1u << 7)
#define SIG_GPIO_OUT        0x100
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (1u << 12)

#define TIMG0               0x500c2000
#define T_CONFIG(x)         (TIMG0 + 0x24 * (x))
#define T_LO(x)             (TIMG0 + 0x04 + 0x24 * (x))
#define T_ALARMLO(x)        (TIMG0 + 0x10 + 0x24 * (x))
#define T_ALARMHI(x)        (TIMG0 + 0x14 + 0x24 * (x))
#define T_LOADLO(x)         (TIMG0 + 0x18 + 0x24 * (x))
#define T_LOADHI(x)         (TIMG0 + 0x1c + 0x24 * (x))
#define T_LOAD(x)           (TIMG0 + 0x20 + 0x24 * (x))
#define TIMG_REGCLK         (TIMG0 + 0xfc)
#define T_ALARM_EN          (1u << 10)
#define T_DIVIDER(d)        ((uint32_t)(d) << 13)
#define T_INCREASE          (1u << 30)
#define T_EN                (1u << 31)
#define REGCLK_ETM_EN       (1u << 28)

#define SYSTIMER            0x500e2000
#define ST_CONF             (SYSTIMER + 0x00)
#define ST_TARGET_HI(x)     (SYSTIMER + 0x1c + 8 * (x))
#define ST_TARGET_LO(x)     (SYSTIMER + 0x20 + 8 * (x))
#define ST_TARGET_CONF(x)   (SYSTIMER + 0x34 + 4 * (x))
#define ST_COMP_LOAD(x)     (SYSTIMER + 0x50 + 4 * (x))
#define ST_INT_RAW          (SYSTIMER + 0x68)
#define CONF_ETM_EN         (1u << 1)
#define CONF_TARGET_EN(x)   (1u << (24 - (x)))
#define CONF_RESET          0x46000000u

#define LP_SYS_INT_RAW      0x50110170
#define INT_ETM_TASK_ULP    (1u << 5)

#define LP_TIMER            0x50112000
#define LP_TAR_LOW0         (LP_TIMER + 0x00)
#define LP_TAR_HIGH0        (LP_TIMER + 0x04)
#define LP_TAR_EN           (1u << 31)

#define AHB_GDMA            0x50085000
#define AHB_IN_INT_RAW(ch)  (AHB_GDMA + 0x10 * (ch))
#define AHB_IN(ch)          (AHB_GDMA + 0x70 + 0xc0 * (ch))
#define AHB_OUT(ch)         (AHB_GDMA + 0xd0 + 0xc0 * (ch))
#define AHB_IN_LINK_ADDR(ch)  (AHB_GDMA + 0x3ac + 4 * (ch))
#define AHB_OUT_LINK_ADDR(ch) (AHB_GDMA + 0x3b8 + 4 * (ch))
#define AXI_GDMA            0x5008a000
#define AXI_IN(ch)          (AXI_GDMA + 0x68 * (ch))
#define AXI_OUT(ch)         (AXI_GDMA + 0x138 + 0x68 * (ch))
#define G_CONF0             0x00
#define G_PERI_SEL          0x30
#define X_RAW               0x00
#define X_CONF0             0x10
#define X_LINK2             0x24
#define X_PERI_SEL          0x44
#define AHB_MEM_TRANS       (1u << 4)
#define AHB_IN_ETM_EN       (1u << 5)
#define AHB_OUT_ETM_EN      (1u << 6)
#define AXI_MEM_TRANS       (1u << 2)
#define AXI_IN_ETM_EN       (1u << 3)
#define AXI_OUT_ETM_EN      (1u << 4)
#define IN_DONE             (1u << 0)
#define IN_SUC_EOF          (1u << 1)
#define DW0(size, len)      ((size) | (len) << 12)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)
#define AHB_DUMMY           1
#define AXI_DUMMY           6
#define MEM                 0x4ff40000

#define PDMA_AHB_EVT_IN_SUC_EOF(c)    (170 + (c))
#define PDMA_AHB_EVT_OUT_TOTAL_EOF(c) (185 + (c))
#define PDMA_AXI_EVT_IN_SUC_EOF(c)    (197 + (c))
#define PDMA_AXI_EVT_OUT_EOF(c)       (209 + (c))
#define PDMA_AHB_TASK_IN_START(c)     (185 + (c))
#define PDMA_AHB_TASK_OUT_START(c)    (188 + (c))
#define PDMA_AXI_TASK_IN_START(c)     (191 + (c))
#define PDMA_AXI_TASK_OUT_START(c)    (194 + (c))

#define DATE_RESET          0x02303031u
/*
 * Pads the board leaves free: GPIO7 has ESP_I2C's 2.2 kOhm pull-up, which
 * holds it high.
 */
#define TASK_PAD            5
#define EVENT_PAD           6

/*
 * A board strapped for Joint Download Boot (GPIO36 high, GPIO35 low), so
 * that the watchdogs' flash boot protection does not run.
 */
static QTestState *start(void)
{
    return qtest_init("-M esp32p4,strap-mode=0x04");
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, ETM + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, ETM + off, v);
}

static void set_bits(QTestState *qts, uint32_t addr, uint32_t bits, bool on)
{
    uint32_t v = qtest_readl(qts, addr);

    qtest_writel(qts, addr, on ? v | bits : v & ~bits);
}

/* ETM_SYS_CLK, ETM_APB_CLK, out of reset. */
static QTestState *start_running(void)
{
    QTestState *qts = start();

    set_bits(qts, SOC_CLK_CTRL1, ETM_SYS_CLK_EN, true);
    set_bits(qts, SOC_CLK_CTRL3, ETM_APB_CLK_EN, true);
    return qts;
}

static bool evt_st(QTestState *qts, int id)
{
    return (rd(qts, EVT_ST((id - 1) / 32)) >> ((id - 1) % 32)) & 1;
}

static bool task_st(QTestState *qts, int id)
{
    return (rd(qts, TASK_ST((id - 1) / 32)) >> ((id - 1) % 32)) & 1;
}

static void clear_status(QTestState *qts)
{
    for (int k = 0; k < 8; k++) {
        wr(qts, EVT_ST_CLR(k), 0xffffffff);
    }
    for (int k = 0; k < 7; k++) {
        wr(qts, TASK_ST_CLR(k), 0xffffffff);
    }
}

static void map(QTestState *qts, int ch, int evt, int task, bool enable)
{
    wr(qts, CH_EVT_ID(ch), evt);
    wr(qts, CH_TASK_ID(ch), task);
    if (ch < 32) {
        wr(qts, enable ? CH_ENA_AD0_SET : CH_ENA_AD0_CLR, 1u << ch);
    } else {
        wr(qts, enable ? CH_ENA_AD1_SET : CH_ENA_AD1_CLR, 1u << (ch - 32));
    }
}

/* Pulse the ETM's event line @id, as its peripheral would. */
static void pulse(QTestState *qts, int id)
{
    qtest_set_irq_in(qts, ETM_PATH, "event", id, 1);
    qtest_set_irq_in(qts, ETM_PATH, "event", id, 0);
}

/* GPIO_EXT's ETM channel @ch sets, clears or toggles TASK_PAD's GPIO_OUT. */
static void gpio_task_pad(QTestState *qts, int ch)
{
    qtest_writel(qts, GPIO_ETM_TASK(TASK_PAD),
                 (1u | ch << 1) << (TASK_PAD % 4 * 8));
}

static bool task_pad(QTestState *qts)
{
    return (qtest_readl(qts, GPIO_OUT) >> TASK_PAD) & 1;
}

/* The ETM's SYS_CLK cycle, in nanoseconds rounded up. */
static uint64_t etm_cycle_ns(QTestState *qts)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         ETM_PATH "/clk");
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    g_assert_cmpuint(period, !=, 0);
    return (period + 0xffffffffu) >> 32;
}

/* Every register at its reset value; the registers need ETM_APB_CLK. */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    /* Gated, the registers cannot latch a write. */
    wr(qts, CH_EVT_ID(0), 5);
    g_assert_cmphex(rd(qts, CH_EVT_ID(0)), ==, 0);
    set_bits(qts, SOC_CLK_CTRL3, ETM_APB_CLK_EN, true);

    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0);
    g_assert_cmphex(rd(qts, CH_ENA_AD1), ==, 0);
    g_assert_cmphex(rd(qts, CH_ENA_AD0_SET), ==, 0);
    g_assert_cmphex(rd(qts, CH_ENA_AD1_CLR), ==, 0);
    for (int ch = 0; ch < 50; ch++) {
        g_assert_cmphex(rd(qts, CH_EVT_ID(ch)), ==, 0);
        g_assert_cmphex(rd(qts, CH_TASK_ID(ch)), ==, 0);
    }
    for (int k = 0; k < 8; k++) {
        g_assert_cmphex(rd(qts, EVT_ST(k)), ==, 0);
        g_assert_cmphex(rd(qts, EVT_ST_CLR(k)), ==, 0);
    }
    for (int k = 0; k < 7; k++) {
        g_assert_cmphex(rd(qts, TASK_ST(k)), ==, 0);
        g_assert_cmphex(rd(qts, TASK_ST_CLR(k)), ==, 0);
    }
    g_assert_cmphex(rd(qts, CLK_EN), ==, 0);
    g_assert_cmphex(rd(qts, DATE), ==, DATE_RESET);
    g_assert_cmphex(rd(qts, 0x228), ==, 0);
    qtest_quit(qts);
}

/* Field widths and access types. */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_registers(void)
{
    QTestState *qts = start_running();

    wr(qts, CH_EVT_ID(49), 0xffffffff);
    wr(qts, CH_TASK_ID(49), 0xffffff12);
    g_assert_cmphex(rd(qts, CH_EVT_ID(49)), ==, 0xff);
    g_assert_cmphex(rd(qts, CH_TASK_ID(49)), ==, 0x12);

    /* CH_ENA_ADn change only through _SET and _CLR. */
    wr(qts, CH_ENA_AD0_SET, 0xffffffff);
    wr(qts, CH_ENA_AD1_SET, 0xffffffff);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, CH_ENA_AD1), ==, 0x3ffff);
    wr(qts, CH_ENA_AD0, 0);
    wr(qts, CH_ENA_AD1, 0);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0xffffffff);
    wr(qts, CH_ENA_AD0_CLR, 0x0000ff0f);
    wr(qts, CH_ENA_AD1_CLR, 0x20001);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0xffff00f0);
    g_assert_cmphex(rd(qts, CH_ENA_AD1), ==, 0x1fffe);
    g_assert_cmphex(rd(qts, CH_ENA_AD0_SET), ==, 0);
    g_assert_cmphex(rd(qts, CH_ENA_AD1_CLR), ==, 0);
    /* A byte write sets or clears that byte's channels alone. */
    qtest_writeb(qts, ETM + CH_ENA_AD0_SET + 1, 0x01);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0xffff01f0);
    qtest_writeb(qts, ETM + CH_ENA_AD0_CLR + 3, 0x80);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0x7fff01f0);

    /* The status registers are read-only but through their _CLR. */
    wr(qts, EVT_ST(0), 0xffffffff);
    wr(qts, TASK_ST(0), 0xffffffff);
    g_assert_cmphex(rd(qts, EVT_ST(0)), ==, 0);
    g_assert_cmphex(rd(qts, TASK_ST(0)), ==, 0);

    wr(qts, CLK_EN, 0xffffffff);
    g_assert_cmphex(rd(qts, CLK_EN), ==, 1);
    wr(qts, DATE, 0xffffffff);
    g_assert_cmphex(rd(qts, DATE), ==, 0x0fffffff);
    qtest_quit(qts);
}

/* RST_EN_ETM resets the block and holds it: no writes, no events. */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_reset_line(void)
{
    QTestState *qts = start_running();

    gpio_task_pad(qts, 0);
    map(qts, 3, SYSTIMER_EVT_CMP(0), GPIO_TASK_TOGGLE(0), true);
    wr(qts, DATE, 0x1234);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_true(evt_st(qts, SYSTIMER_EVT_CMP(0)));

    set_bits(qts, HP_RST_EN1, RST_EN_ETM, true);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0);
    g_assert_cmphex(rd(qts, CH_EVT_ID(3)), ==, 0);
    g_assert_cmphex(rd(qts, EVT_ST(1)), ==, 0);
    g_assert_cmphex(rd(qts, DATE), ==, DATE_RESET);
    wr(qts, CH_EVT_ID(3), SYSTIMER_EVT_CMP(0));
    wr(qts, CH_ENA_AD0_SET, 1u << 3);
    g_assert_cmphex(rd(qts, CH_EVT_ID(3)), ==, 0);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_cmphex(rd(qts, EVT_ST(1)), ==, 0);

    set_bits(qts, HP_RST_EN1, RST_EN_ETM, false);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 0);
    map(qts, 3, SYSTIMER_EVT_CMP(0), GPIO_TASK_TOGGLE(0), true);
    g_assert_cmphex(rd(qts, CH_ENA_AD0), ==, 1u << 3);
    qtest_quit(qts);
}

/*
 * An event on an enabled channel generates its task at once; a disabled
 * channel, or one whose task ID selects nothing, generates none, though
 * the event's status is set whatever maps it.
 */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_event_to_task(void)
{
    QTestState *qts = start_running();
    uint64_t cycle = etm_cycle_ns(qts);

    gpio_task_pad(qts, 0);
    map(qts, 3, SYSTIMER_EVT_CMP(0), GPIO_TASK_TOGGLE(0), true);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_true(task_pad(qts));
    g_assert_true(evt_st(qts, SYSTIMER_EVT_CMP(0)));
    g_assert_true(task_st(qts, GPIO_TASK_TOGGLE(0)));
    g_assert_cmphex(rd(qts, EVT_ST(1)), ==, 1u << 24);
    g_assert_cmphex(rd(qts, TASK_ST(0)), ==, 1u << 16);

    /* The status bits clear through their _CLR registers, by bit. */
    wr(qts, EVT_ST_CLR(1), ~(1u << 24));
    wr(qts, TASK_ST_CLR(0), ~(1u << 16));
    g_assert_cmphex(rd(qts, EVT_ST(1)), ==, 1u << 24);
    clear_status(qts);
    g_assert_cmphex(rd(qts, EVT_ST(1)), ==, 0);
    g_assert_cmphex(rd(qts, TASK_ST(0)), ==, 0);

    /* The task line is a pulse: the next event toggles again. */
    qtest_clock_step(qts, cycle);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_false(task_pad(qts));

    /* Disabled, the channel maps nothing; the event is still received. */
    clear_status(qts);
    wr(qts, CH_ENA_AD0_CLR, 1u << 3);
    qtest_clock_step(qts, cycle);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_false(task_pad(qts));
    g_assert_true(evt_st(qts, SYSTIMER_EVT_CMP(0)));
    g_assert_false(task_st(qts, GPIO_TASK_TOGGLE(0)));

    /* Configured but never enabled, likewise. */
    map(qts, 40, SYSTIMER_EVT_CMP(1), GPIO_TASK_TOGGLE(0), false);
    pulse(qts, SYSTIMER_EVT_CMP(1));
    g_assert_false(task_pad(qts));
    g_assert_true(evt_st(qts, SYSTIMER_EVT_CMP(1)));
    /* Enabled, channel 40 (CH_ENA_AD1) maps it. */
    wr(qts, CH_ENA_AD1_SET, 1u << 8);
    qtest_clock_step(qts, cycle);
    pulse(qts, SYSTIMER_EVT_CMP(1));
    g_assert_true(task_pad(qts));

    /* Task 0, and IDs past the last task, select none. */
    clear_status(qts);
    map(qts, 40, SYSTIMER_EVT_CMP(1), 0, true);
    map(qts, 41, SYSTIMER_EVT_CMP(1), 208, true);
    map(qts, 42, 0, GPIO_TASK_TOGGLE(0), true);
    qtest_clock_step(qts, cycle);
    pulse(qts, SYSTIMER_EVT_CMP(1));
    g_assert_true(task_pad(qts));
    for (int k = 0; k < 7; k++) {
        g_assert_cmphex(rd(qts, TASK_ST(k)), ==, 0);
    }
    qtest_quit(qts);
}

/*
 * A task several channels generate in one ETM cycle is generated once,
 * and an event pulsed twice in a cycle is received once; a cycle on, each
 * again. Events and tasks with nothing connected are received and
 * generated all the same.
 */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_once_per_cycle(void)
{
    QTestState *qts = start_running();
    uint64_t cycle = etm_cycle_ns(qts);

    gpio_task_pad(qts, 0);
    map(qts, 0, SYSTIMER_EVT_CMP(0), GPIO_TASK_TOGGLE(0), true);
    map(qts, 1, SYSTIMER_EVT_CMP(0), GPIO_TASK_TOGGLE(0), true);
    map(qts, 2, SYSTIMER_EVT_CMP(1), GPIO_TASK_TOGGLE(0), true);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_true(task_pad(qts));
    pulse(qts, SYSTIMER_EVT_CMP(1));
    g_assert_true(task_pad(qts));
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_true(task_pad(qts));

    qtest_clock_step(qts, cycle);
    pulse(qts, SYSTIMER_EVT_CMP(1));
    g_assert_false(task_pad(qts));
    qtest_clock_step(qts, cycle);
    pulse(qts, SYSTIMER_EVT_CMP(0));
    g_assert_true(task_pad(qts));

    /* The LED PWM's event, which nothing generates here, to an I2S task. */
    map(qts, 4, 29, 167, true);
    pulse(qts, 29);
    g_assert_true(evt_st(qts, 29));
    g_assert_true(task_st(qts, 167));
    qtest_quit(qts);
}

/* With ETM_SYS_CLK stopped, events are lost. */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_clock_gated(void)
{
    QTestState *qts = start_running();

    gpio_task_pad(qts, 0);
    map(qts, 0, SYSTIMER_EVT_CMP(2), GPIO_TASK_SET(0), true);
    set_bits(qts, SOC_CLK_CTRL1, ETM_SYS_CLK_EN, false);
    pulse(qts, SYSTIMER_EVT_CMP(2));
    g_assert_false(task_pad(qts));
    g_assert_false(evt_st(qts, SYSTIMER_EVT_CMP(2)));
    set_bits(qts, SOC_CLK_CTRL1, ETM_SYS_CLK_EN, true);
    pulse(qts, SYSTIMER_EVT_CMP(2));
    g_assert_true(task_pad(qts));
    qtest_quit(qts);
}

/*
 * GPIO_EXT's events: a pad's rising, falling and any edge, through its
 * ETM channel, set and clear another pad; and the analog comparator's
 * zero-detect events.
 */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_gpio_events(void)
{
    QTestState *qts = start_running();

    qtest_writel(qts, IO_MUX_GPIO(EVENT_PAD), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(EVENT_PAD), SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_ETM_EVENT(1), ETM_EVENT_EN | EVENT_PAD);
    gpio_task_pad(qts, 2);
    map(qts, 10, GPIO_EVT_CH_RISE(1), GPIO_TASK_SET(2), true);
    map(qts, 11, GPIO_EVT_CH_FALL(1), GPIO_TASK_CLEAR(2), true);

    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", EVENT_PAD, 1);
    g_assert_true(task_pad(qts));
    g_assert_true(evt_st(qts, GPIO_EVT_CH_RISE(1)));
    g_assert_true(evt_st(qts, GPIO_EVT_CH_ANY(1)));
    g_assert_false(evt_st(qts, GPIO_EVT_CH_FALL(1)));
    qtest_clock_step(qts, 1000);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", EVENT_PAD, 0);
    g_assert_false(task_pad(qts));
    g_assert_true(evt_st(qts, GPIO_EVT_CH_FALL(1)));

    /* Event channel off in GPIO_EXT: no event at all. */
    clear_status(qts);
    qtest_writel(qts, GPIO_ETM_EVENT(1), EVENT_PAD);
    qtest_clock_step(qts, 1000);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", EVENT_PAD, 1);
    g_assert_false(task_pad(qts));
    g_assert_cmphex(rd(qts, EVT_ST(0)), ==, 0);

    /* The comparator's rising edge is ZERO_DET_POS0, falling NEG0. */
    map(qts, 12, GPIO_EVT_ZERO_DET_POS0, GPIO_TASK_SET(2), true);
    map(qts, 13, GPIO_EVT_ZERO_DET_NEG0, GPIO_TASK_CLEAR(2), true);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 0, 1);
    g_assert_true(task_pad(qts));
    g_assert_true(evt_st(qts, GPIO_EVT_ZERO_DET_POS0));
    /* GPIO_ZERO_DET0_FILTER_CNT, all ones from reset, mutes the next. */
    qtest_clock_step(qts, 1000);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 0, 0);
    g_assert_true(task_pad(qts));
    g_assert_false(evt_st(qts, GPIO_EVT_ZERO_DET_NEG0));
    qtest_writel(qts, GPIO + 0x710, 0);
    qtest_clock_step(qts, 1000);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 0, 1);
    qtest_clock_step(qts, 1000);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 0, 0);
    g_assert_false(task_pad(qts));
    g_assert_true(evt_st(qts, GPIO_EVT_ZERO_DET_NEG0));
    qtest_quit(qts);
}

/*
 * Timer group 0's T0 alarm, with TIMG_ETM_EN, captures T1 (counting the
 * same clock from the same start) at the alarm's count, and sets LP_SYS's
 * ETM_TASK_ULP_INT through ULP_TASK_INT_CPU; the system timer's and the
 * RTC timer's alarms are received.
 */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_timer_events(void)
{
    QTestState *qts = start_running();

    qtest_writel(qts, TIMG_REGCLK,
                 qtest_readl(qts, TIMG_REGCLK) | REGCLK_ETM_EN);
    map(qts, 0, TG0_EVT_CMP_T0, TG0_TASK_CNT_CAP_T1, true);
    map(qts, 1, TG0_EVT_CMP_T0, ULP_TASK_INT_CPU, true);
    for (int x = 0; x < 2; x++) {
        qtest_writel(qts, T_LOADLO(x), 0);
        qtest_writel(qts, T_LOADHI(x), 0);
        qtest_writel(qts, T_LOAD(x), 1);
        qtest_writel(qts, T_CONFIG(x), T_INCREASE | T_DIVIDER(2));
    }
    qtest_writel(qts, T_ALARMLO(0), 1000);
    qtest_writel(qts, T_ALARMHI(0), 0);
    /* Both timers start in the same instant, from their configuration. */
    qtest_writel(qts, T_CONFIG(1), T_EN | T_INCREASE | T_DIVIDER(2));
    qtest_writel(qts, T_CONFIG(0),
                 T_EN | T_INCREASE | T_DIVIDER(2) | T_ALARM_EN);
    qtest_clock_step(qts, 60 * 1000);
    g_assert_true(evt_st(qts, TG0_EVT_CMP_T0));
    g_assert_cmpuint(qtest_readl(qts, T_LO(1)), ==, 1000);
    g_assert_true(qtest_readl(qts, LP_SYS_INT_RAW) & INT_ETM_TASK_ULP);

    /* The system timer's comparator 0, its target long passed. */
    qtest_writel(qts, ST_CONF, CONF_RESET | CONF_ETM_EN | CONF_TARGET_EN(0));
    qtest_writel(qts, ST_TARGET_HI(0), 0);
    qtest_writel(qts, ST_TARGET_LO(0), 0);
    qtest_writel(qts, ST_COMP_LOAD(0), 1);
    g_assert_true(evt_st(qts, SYSTIMER_EVT_CMP(0)));

    /* The RTC timer's target 0, count 20, at 133.3 us. */
    qtest_writel(qts, LP_TAR_LOW0, 20);
    qtest_writel(qts, LP_TAR_HIGH0, LP_TAR_EN);
    g_assert_false(evt_st(qts, RTC_EVT_CMP));
    qtest_clock_step(qts, 80 * 1000);
    g_assert_true(evt_st(qts, RTC_EVT_CMP));
    qtest_quit(qts);
}

/*
 * RTC_EVT_TICK, every count of the RTC timer (RC_SLOW_CLK, 150 kHz, from
 * virtual time 0): its status sets while nothing maps it, and mapped, each
 * tick toggles a pad, the k-th at k * 6666.7 ns.
 */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_rtc_tick(void)
{
    QTestState *qts = start_running();
    int64_t now, k0;

    /* The first tick is at 6667 ns. */
    now = qtest_clock_step(qts, 6666);
    g_assert_false(evt_st(qts, RTC_EVT_TICK));
    now = qtest_clock_step(qts, 1);
    g_assert_true(evt_st(qts, RTC_EVT_TICK));
    clear_status(qts);
    g_assert_false(evt_st(qts, RTC_EVT_TICK));
    now = qtest_clock_step(qts, 7000);
    g_assert_true(evt_st(qts, RTC_EVT_TICK));

    gpio_task_pad(qts, 0);
    map(qts, 5, RTC_EVT_TICK, GPIO_TASK_TOGGLE(0), true);
    /* Tick k at k * 20000 / 3 ns, rounded up: the next twenty. */
    k0 = now * 3 / 20000 + 1;
    for (int64_t k = k0; k < k0 + 20; k++) {
        int64_t at = (k * 20000 + 2) / 3;

        now = qtest_clock_step(qts, at - 1 - now);
        g_assert_cmpint(task_pad(qts), ==, (k - k0) & 1);
        now = qtest_clock_step(qts, 1);
        g_assert_cmpint(task_pad(qts), ==, !((k - k0) & 1));
    }

    /* Disabled, the pad keeps its level. */
    wr(qts, CH_ENA_AD0_CLR, 1u << 5);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << TASK_PAD);
    qtest_clock_step(qts, 100 * 1000);
    g_assert_false(task_pad(qts));
    qtest_quit(qts);
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, 0);
}

static void fill(QTestState *qts, uint32_t addr, uint32_t len)
{
    g_autofree uint8_t *b = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        b[i] = i * 7 + 3;
    }
    qtest_memwrite(qts, addr, b, len);
}

static void check_copy(QTestState *qts, uint32_t dst, uint32_t src,
                       uint32_t len)
{
    g_autofree uint8_t *a = g_malloc(len);
    g_autofree uint8_t *b = g_malloc(len);

    qtest_memread(qts, src, a, len);
    qtest_memread(qts, dst, b, len);
    g_assert_cmpmem(a, len, b, len);
}

/*
 * GDMA-AHB's channel 1 and GDMA-AXI's channel 2, their *_ETM_EN set, copy
 * memory to memory as an event's ETM tasks start their RX and TX sides;
 * their EOF events come back to the ETM, AHB's IN_SUC_EOF toggling a pad.
 */
/* [spec:nuos:req:emu.esp32p4.etm/test] */
static void test_gdma(void)
{
    QTestState *qts = start_running();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x4000;

    fill(qts, src, 256);
    qtest_memset(qts, dst, 0, 512);
    desc(qts, tx, OWNER | SUC_EOF | DW0(256, 256), src);
    desc(qts, rx, OWNER | DW0(256, 0), dst);
    qtest_writel(qts, AHB_IN(1) + G_PERI_SEL, AHB_DUMMY + 5);
    qtest_writel(qts, AHB_OUT(1) + G_PERI_SEL, AHB_DUMMY + 5);
    qtest_writel(qts, AHB_IN(1) + G_CONF0, AHB_MEM_TRANS | AHB_IN_ETM_EN);
    qtest_writel(qts, AHB_OUT(1) + G_CONF0,
                 qtest_readl(qts, AHB_OUT(1) + G_CONF0) | AHB_OUT_ETM_EN);
    qtest_writel(qts, AHB_IN_LINK_ADDR(1), rx);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(1), tx);

    desc(qts, tx + 0x40, OWNER | SUC_EOF | DW0(256, 256), src);
    desc(qts, rx + 0x40, OWNER | DW0(256, 0), dst + 256);
    qtest_writel(qts, AXI_IN(2) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_OUT(2) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_IN(2) + X_CONF0, AXI_MEM_TRANS | AXI_IN_ETM_EN);
    qtest_writel(qts, AXI_OUT(2) + X_CONF0,
                 qtest_readl(qts, AXI_OUT(2) + X_CONF0) | AXI_OUT_ETM_EN);
    qtest_writel(qts, AXI_IN(2) + X_LINK2, rx + 0x40);
    qtest_writel(qts, AXI_OUT(2) + X_LINK2, tx + 0x40);

    gpio_task_pad(qts, 0);
    map(qts, 0, SYSTIMER_EVT_CMP(0), PDMA_AHB_TASK_IN_START(1), true);
    map(qts, 1, SYSTIMER_EVT_CMP(0), PDMA_AHB_TASK_OUT_START(1), true);
    map(qts, 2, SYSTIMER_EVT_CMP(0), PDMA_AXI_TASK_IN_START(2), true);
    map(qts, 3, SYSTIMER_EVT_CMP(0), PDMA_AXI_TASK_OUT_START(2), true);
    map(qts, 4, PDMA_AHB_EVT_IN_SUC_EOF(1), GPIO_TASK_TOGGLE(0), true);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT_RAW(1)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(2) + X_RAW), ==, 0);

    pulse(qts, SYSTIMER_EVT_CMP(0));
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT_RAW(1)), ==,
                    IN_DONE | IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(2) + X_RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    check_copy(qts, dst, src, 256);
    check_copy(qts, dst + 256, src, 256);
    g_assert_true(task_st(qts, PDMA_AHB_TASK_OUT_START(1)));
    g_assert_true(task_st(qts, PDMA_AXI_TASK_IN_START(2)));
    g_assert_true(evt_st(qts, PDMA_AHB_EVT_IN_SUC_EOF(1)));
    g_assert_true(evt_st(qts, PDMA_AHB_EVT_OUT_TOTAL_EOF(1)));
    g_assert_true(evt_st(qts, PDMA_AXI_EVT_IN_SUC_EOF(2)));
    g_assert_true(evt_st(qts, PDMA_AXI_EVT_OUT_EOF(2)));
    g_assert_false(evt_st(qts, PDMA_AXI_EVT_IN_SUC_EOF(1)));
    g_assert_true(task_pad(qts));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/etm/reset-values", test_reset_values);
    qtest_add_func("esp32p4/etm/registers", test_registers);
    qtest_add_func("esp32p4/etm/reset-line", test_reset_line);
    qtest_add_func("esp32p4/etm/event-to-task", test_event_to_task);
    qtest_add_func("esp32p4/etm/once-per-cycle", test_once_per_cycle);
    qtest_add_func("esp32p4/etm/clock-gated", test_clock_gated);
    qtest_add_func("esp32p4/etm/gpio-events", test_gpio_events);
    qtest_add_func("esp32p4/etm/timer-events", test_timer_events);
    qtest_add_func("esp32p4/etm/rtc-tick", test_rtc_tick);
    qtest_add_func("esp32p4/etm/gdma", test_gdma);
    return g_test_run();
}
