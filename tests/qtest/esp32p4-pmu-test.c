/*
 * QTest testcase for the ESP32-P4 power management unit (PMU) and the
 * brown-out detector
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The RTC timer counts RC_SLOW_CLK's 150 kHz from the machine's reset at
 * virtual time 0; LP_DYN_FAST_CLK is LP_FAST_CLK, from reset the 40 MHz
 * crystal, but in LP_SLEEP, when it is LP_SLOW_CLK.
 * Interrupts are seen in the interrupt matrix's status of sources 6
 * (PMU_INTR), 7 (the PMU's LP interrupt) and 8 (LP_ANAPERI_INTR).
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define PMU                     0x50115000
#define HP_ACTIVE_DIG_POWER     0x000
#define HP_ACTIVE_ICG_HP_FUNC   0x004
#define HP_ACTIVE_HP_CK_POWER   0x014
#define HP_ACTIVE_SYSCLK        0x024
#define HP_MODEM_ICG_HP_FUNC    0x038
#define HP_SLEEP_DIG_POWER      0x068
#define HP_SLEEP_ICG_HP_FUNC    0x06c
#define HP_SLEEP_HP_SYS_CNTL    0x078
#define HP_SLEEP_HP_CK_POWER    0x07c
#define HP_SLEEP_SYSCLK         0x08c
#define HP_SLEEP_XTAL           0x098
#define HP_SLEEP_LP_CK_POWER    0x0ac
#define LP_SLEEP_XTAL           0x0bc
#define LP_SLEEP_LP_DIG_POWER   0x0c0
#define IMM_HP_CK_POWER         0x0cc
#define IMM_SLEEP_SYSCLK        0x0d0
#define IMM_HP_FUNC_ICG         0x0d4
#define IMM_PAD_HOLD_ALL        0x0e4
#define POWER_PD_TOP_CNTL       0x0f4
#define POWER_PD_CNNT_CNTL      0x0f8
#define POWER_PD_HPMEM_CNTL     0x0fc
#define POWER_PD_LPPERI_CNTL    0x110
#define SLP_WAKEUP_CNTL0        0x120
#define SLP_WAKEUP_CNTL1        0x124
#define SLP_WAKEUP_CNTL2        0x128
#define SLP_WAKEUP_CNTL3        0x12c
#define SLP_WAKEUP_CNTL4        0x130
#define SLP_WAKEUP_STATUS0      0x144
#define SLP_WAKEUP_STATUS1      0x148
#define POR_STATUS              0x158
#define HP_INT_RAW              0x164
#define HP_INT_ST               0x168
#define HP_INT_ENA              0x16c
#define HP_INT_CLR              0x170
#define LP_INT_RAW              0x174
#define LP_INT_CLR              0x180
#define LP_CPU_PWR0             0x184
#define LP_CPU_PWR1             0x188
#define LP_CPU_PWR2             0x18c
#define LP_CPU_PWR3             0x190
#define LP_CPU_PWR4             0x194
#define LP_CPU_PWR5             0x198
#define HP_LP_CPU_COMM          0x19c
#define MAIN_STATE              0x1a4
#define CLK_STATE0              0x1ac
#define EXT_LDO_P0_0P2A         0x1c0
#define EXT_WAKEUP_LV           0x1e8
#define EXT_WAKEUP_SEL          0x1ec
#define EXT_WAKEUP_ST           0x1f0
#define EXT_WAKEUP_CNTL         0x1f4
#define CPU_SW_STALL            0x200
#define POWER_PD_HP_CPU_CNTL    0x218
#define DATE                    0x3fc

#define DIG_MEM_PD              (1u << 23)
#define DIG_CPU_PD              (1u << 29)
#define DIG_CNNT_PD             (1u << 30)
#define DIG_TOP_PD              (1u << 31)
#define CNTL_DIG_PAUSE_WDT      (1u << 28)
#define CNTL_DIG_CPU_STALL      (1u << 29)
#define XPD_PLL(v)              ((uint32_t)(v) << 27)
#define SYSCLK_ICG_SYSCLK_EN    (1u << 27)
#define SYSCLK_SLP_SEL          (1u << 28)
#define SYSCLK_ICG_SLP_SEL      (1u << 29)
#define XPD_XTAL                (1u << 31)
#define LP_XPD_RC32K            (1u << 29)
#define LP_XPD_FOSC             (1u << 30)
#define LP_PERI_PD              (1u << 31)
#define TIE_LOW_XPD_APLL        (1u << 13)
#define TIE_HIGH_XPD_APLL       (1u << 29)
#define TIE_LOW_ICG_SLP_SEL     (1u << 29)
#define TIE_HIGH_ICG_SLP_SEL    (1u << 30)
#define UPDATE_ICG              (1u << 31)
#define PAD_HOLD_LP_ST          (1u << 1)
#define PAD_HOLD_HP_ST          (1u << 2)
#define TIE_LOW_LP_PAD_HOLD     (1u << 29)
#define TIE_LOW_HP_PAD_HOLD     (1u << 31)
#define SLEEP_REQ               (1u << 31)
#define SLP_REJECT_EN           (1u << 31)
#define SLP_REJECT_CAUSE_CLR    (1u << 31)
#define LP_CPU_SLP_STALL_EN     (1u << 29)
#define LP_CPU_SLP_RESET_EN     (1u << 30)
#define LP_TRIGGER_HP           (1u << 30)
#define HP_TRIGGER_LP           (1u << 31)
#define EXT_STATUS_CLR          (1u << 30)
#define HP_INT_SW               (1u << 29)
#define HP_INT_REJECT           (1u << 30)
#define HP_INT_WAKEUP           (1u << 31)
#define LP_INT_LP_CPU_REJECT    (1u << 13)
#define LP_INT_LP_CPU_WAKEUP    (1u << 26)
#define LP_INT_S2A_END          (1u << 27)
#define LP_INT_A2S_END          (1u << 28)
#define LP_INT_S2A_START        (1u << 29)
#define LP_INT_A2S_START        (1u << 30)
#define LP_INT_HP_SW_TRIGGER    (1u << 31)
#define LDO_XPD                 (1u << 8)
#define LDO_TARGET1(v)          ((uint32_t)(v) << 15)
#define LDO_TARGET0(v)          ((uint32_t)(v) << 23)
#define LDO_INT_T0_VO3          (1u << 18)
#define LDO_INT_T1_VO3          (1u << 19)

#define WAKE_GPIO               2
#define WAKE_EXT                12
#define WAKE_RTC_TIMER0         13
#define WAKE_LP_CPU             22

#define STATE_ACTIVE            0x01
#define STATE_HP_SLEEP          0x04
#define STATE_LP_SLEEP          0x08

#define LP_TIMER                0x50112000
#define LP_TIMER_TAR_LOW0       (LP_TIMER + 0x00)
#define LP_TIMER_TAR_HIGH0      (LP_TIMER + 0x04)
#define LP_TIMER_UPDATE         (LP_TIMER + 0x10)
#define LP_TIMER_BUF0_LOW       (LP_TIMER + 0x14)
#define LP_TIMER_INT_CLR        (LP_TIMER + 0x34)
#define TAR_EN                  (1u << 31)
#define TIMER_UPDATE            (1u << 28)
#define INT_TARGET              (1u << 31)

#define LP_CLKRST_RESET_CAUSE   0x50111010
#define LPCORE_CAUSE_MASK_PMU   (1u << 25)
#define LP_SRAM                 0x50108000
#define L2MEM_DIRECT            0x8ff40000
#define HP_SYS_CLKRST_SOC_CLK_CTRL1 0x500e6018

#define LP_ANA                  0x50113000
#define BOD_MODE0_CNTL          (LP_ANA + 0x00)
#define BOD_MODE1_CNTL          (LP_ANA + 0x04)
#define VDD_SOURCE_CNTL         (LP_ANA + 0x08)
#define VDDBAT_BOD_CNTL         (LP_ANA + 0x0c)
#define PG_GLITCH_CNTL          (LP_ANA + 0x18)
#define FIB_ENABLE              (LP_ANA + 0x1c)
#define ANA_INT_RAW             (LP_ANA + 0x20)
#define ANA_INT_ST              (LP_ANA + 0x24)
#define ANA_INT_ENA             (LP_ANA + 0x28)
#define ANA_INT_CLR             (LP_ANA + 0x2c)
#define ANA_LP_INT_RAW          (LP_ANA + 0x30)
#define MODE0_INTR_WAIT(v)      ((uint32_t)(v) << 8)
#define MODE0_RESET_WAIT(v)     ((uint32_t)(v) << 18)
#define MODE0_INTR_ENA          (1u << 29)
#define MODE0_RESET_SEL         (1u << 30)
#define MODE0_RESET_ENA         (1u << 31)
#define INT_BOD_MODE0           (1u << 31)
#define INT_VBAT_UNDER          (1u << 30)
#define INT_VBAT_UP             (1u << 29)
#define FILTER_UNDER(v)         ((uint32_t)(v) << 22)
#define FILTER_UP(v)            ((uint32_t)(v) << 12)

#define I2C_ANA_MST             0x50124000
#define ANA_I2C0_CTRL           (I2C_ANA_MST + 0x00)
#define ANA_CONF1               (I2C_ANA_MST + 0x1c)
#define ANA_CONF2               (I2C_ANA_MST + 0x20)
#define REGI2C_WRITE            (1u << 24)
#define REGI2C_BUSY             (1u << 25)
#define REGI2C_BIAS_MST_SEL     (1u << 12)
#define I2C_BIAS                0x6a
#define BIAS_DREF_VBAT          8
#define BIAS_DREF_VDDA          9

#define INTMTX_STATUS0          0x500d6200
#define SRC_PMU_0               6
#define SRC_PMU_1               7
#define SRC_LP_ANAPERI          8

#define CLKRST                  "/machine/clkrst"
#define HP_CPU0                 "/machine/hp-cluster/cpu[0]"
#define LP_CPU                  "/machine/lp-cluster/cpu"
#define PMU_PATH                "/machine/pmu"
#define LP_ANA_PATH             "/machine/lp-ana"
#define GPIO_PATH               "/machine/gpio"

#define US                      1000ll
#define MS                      (1000 * US)

/* A board strapped for Joint Download Boot: no flash boot protection. */
static QTestState *start(const char *extra)
{
    return qtest_initf("-M esp32p4,strap-mode=0x04 %s", extra ? extra : "");
}

/* The virtual time, a nanosecond on: qtest cannot step the clock by 0. */
static int64_t now(QTestState *qts)
{
    return qtest_clock_step(qts, 1);
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, PMU + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, PMU + off, v);
}

static bool source(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> src) & 1;
}

static uint32_t current_state(QTestState *qts)
{
    return rd(qts, MAIN_STATE) >> 25;
}

static uint64_t clk_hz(QTestState *qts, const char *name)
{
    g_autofree char *path = g_strdup_printf(CLKRST "/%s", name);
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         path);
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    return period ? (1000000000ull << 32) / period : 0;
}

static void qom_set_uint(QTestState *qts, const char *path,
                         const char *property, uint64_t value)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': %s, 'property': %s, 'value': %" PRIu64
                         " } }", path, property, value);

    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
}

static uint32_t rtc_count(QTestState *qts)
{
    qtest_writel(qts, LP_TIMER_UPDATE, TIMER_UPDATE);
    return qtest_readl(qts, LP_TIMER_BUF0_LOW);
}

/* Arm RTC timer target 0 @ticks ahead, its alarm cleared. */
static void rtc_alarm(QTestState *qts, uint32_t ticks)
{
    qtest_writel(qts, LP_TIMER_INT_CLR, INT_TARGET);
    qtest_writel(qts, LP_TIMER_TAR_LOW0, rtc_count(qts) + ticks);
    qtest_writel(qts, LP_TIMER_TAR_HIGH0, TAR_EN);
}

/* Write a BIAS register through the analog I2C master. */
static void bias_write(QTestState *qts, uint8_t reg, uint8_t data)
{
    qtest_writel(qts, ANA_CONF2, 0);
    qtest_writel(qts, ANA_CONF1, 0);
    qtest_writel(qts, ANA_CONF2, REGI2C_BIAS_MST_SEL);
    qtest_writel(qts, ANA_I2C0_CTRL, I2C_BIAS | (reg << 8) |
                 ((uint32_t)data << 16) | REGI2C_WRITE);
    while (qtest_readl(qts, ANA_I2C0_CTRL) & REGI2C_BUSY) {
        qtest_clock_step(qts, 10 * US);
    }
}

/*
 * The light-sleep settings ESP-IDF's pmu_sleep_init() gives: in HP_SLEEP
 * the cores stall and the PMU gates every clock of table 15.4-2 and
 * ROOT_CLK; HP_ACTIVE keeps the CPU and system PLLs.
 */
static void light_sleep_config(QTestState *qts)
{
    wr(qts, HP_ACTIVE_HP_CK_POWER, XPD_PLL(3));
    wr(qts, HP_ACTIVE_SYSCLK, SYSCLK_ICG_SYSCLK_EN);
    wr(qts, HP_SLEEP_ICG_HP_FUNC, 0);
    wr(qts, HP_SLEEP_SYSCLK, SYSCLK_SLP_SEL | SYSCLK_ICG_SLP_SEL);
    wr(qts, HP_SLEEP_HP_SYS_CNTL, CNTL_DIG_CPU_STALL | CNTL_DIG_PAUSE_WDT);
}

/* The power-on state of the registers and of what the PMU controls. */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_reset(void)
{
    QTestState *qts = start(NULL);

    g_assert_cmphex(rd(qts, DATE), ==, 0x02501030);
    g_assert_cmphex(rd(qts, MAIN_STATE), ==, 0x02100801);
    g_assert_cmphex(rd(qts, POR_STATUS), ==, 0x80000000);
    g_assert_cmphex(rd(qts, POWER_PD_TOP_CNTL), ==, 0x1c);
    g_assert_cmphex(rd(qts, HP_SLEEP_ICG_HP_FUNC), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, LP_CPU_PWR0), ==, 0x1ff00000);
    g_assert_cmphex(rd(qts, HP_SLEEP_LP_CK_POWER), ==, 0x40000000);
    /*
     * The controls in effect: the crystal and the CPU and system PLLs
     * powered and their gates open, ROOT_CLK's gate open.
     */
    g_assert_cmphex(rd(qts, CLK_STATE0), ==,
                    0x80000000 | (3u << 27) | (3u << 20) | (1u << 19) |
                    (1u << 14) | (1u << 3) | 3);
    g_assert_cmpuint(clk_hz(qts, "osc-xtal"), ==, 40000000);
    g_assert_cmpuint(clk_hz(qts, "lp-dyn-fast"), ==, 40000000);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));

    /* The HP_MODEM state's registers, of a state the P4 lacks, read 0. */
    wr(qts, HP_MODEM_ICG_HP_FUNC, 0xffffffff);
    g_assert_cmphex(rd(qts, HP_MODEM_ICG_HP_FUNC), ==, 0);
    /* Read-only registers and reserved space. */
    wr(qts, SLP_WAKEUP_STATUS0, 0xffffffff);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS0), ==, 0);
    g_assert_cmphex(rd(qts, 0x300), ==, 0);
    qtest_quit(qts);
}

/*
 * A source that PMU_SLEEP_REJECT_ENA enables rejects the sleep, recording
 * it in PMU_REJECT_CAUSE and raising PMU_SOC_SLEEP_REJECT_INT; the chip
 * stays in HP_ACTIVE.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_reject(void)
{
    QTestState *qts = start(NULL);

    wr(qts, HP_INT_ENA, HP_INT_REJECT);
    wr(qts, SLP_WAKEUP_CNTL1, SLP_REJECT_EN | (1u << WAKE_GPIO));
    qtest_set_irq_in(qts, PMU_PATH, "wakeup", WAKE_GPIO, 1);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS1), ==, 1u << WAKE_GPIO);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, HP_INT_REJECT);
    g_assert(source(qts, SRC_PMU_0));
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(current_state(qts), ==, STATE_ACTIVE);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    wr(qts, SLP_WAKEUP_CNTL4, SLP_REJECT_CAUSE_CLR);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS1), ==, 0);
    wr(qts, HP_INT_CLR, HP_INT_REJECT);
    g_assert(!source(qts, SRC_PMU_0));

    /* A source only wake-enabled does not reject: the chip falls asleep. */
    wr(qts, SLP_WAKEUP_CNTL1, 0);
    light_sleep_config(qts);
    wr(qts, SLP_WAKEUP_CNTL2, 1u << WAKE_RTC_TIMER0);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_LP_SLEEP);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, 0);
    qtest_quit(qts);
}

/*
 * A light sleep, woken by the RTC timer: the cores stall and the PMU
 * gates their clocks and ROOT_CLK; LP_DYN_FAST_CLK follows LP_SLOW_CLK in
 * LP_SLEEP; the wakeup restores HP_ACTIVE's settings, records the cause
 * and raises PMU_SOC_WAKEUP_INT, the switches marked by the LP interrupts.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_light_sleep(void)
{
    QTestState *qts = start(NULL);
    int64_t t;

    light_sleep_config(qts);
    wr(qts, HP_INT_ENA, HP_INT_WAKEUP);
    wr(qts, SLP_WAKEUP_CNTL2, 1u << WAKE_RTC_TIMER0);
    rtc_alarm(qts, 150);
    t = now(qts);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, LP_INT_A2S_START);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_LP_SLEEP);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==,
                    LP_INT_A2S_START | LP_INT_A2S_END);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmpuint(clk_hz(qts, "root"), ==, 0);
    g_assert_cmpuint(clk_hz(qts, "core0-cpu"), ==, 0);
    g_assert_cmpuint(clk_hz(qts, "lp-dyn-fast"), ==, 150000);
    g_assert_cmpuint(clk_hz(qts, "osc-xtal"), ==, 40000000);
    /* The CPU and system PLLs are off in HP_SLEEP (XPD_PLL clear). */
    g_assert_cmphex((rd(qts, CLK_STATE0) >> 27) & 0xf, ==, 0);

    /* The alarm comes at 1 ms after it was set; the wakeup takes a while. */
    qtest_clock_step(qts, t + 1 * MS - now(qts));
    g_assert_cmphex(current_state(qts), !=, STATE_ACTIVE);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_ACTIVE);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS0), ==, 1u << WAKE_RTC_TIMER0);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, HP_INT_WAKEUP);
    g_assert(source(qts, SRC_PMU_0));
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==,
                    LP_INT_A2S_START | LP_INT_A2S_END | LP_INT_S2A_START |
                    LP_INT_S2A_END);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    g_assert_cmpuint(clk_hz(qts, "root"), ==, 40000000);
    g_assert_cmpuint(clk_hz(qts, "lp-dyn-fast"), ==, 40000000);
    g_assert_cmphex((rd(qts, CLK_STATE0) >> 27) & 0xf, ==, 3);
    qtest_quit(qts);
}

/*
 * A deep sleep: PD_TOP, PD_HP_CPU, the HP memories and PD_LP_PERI powered
 * down. The HP system is held in reset and loses its state, L2MEM its
 * contents; LP SRAM and the LP system keep theirs. The wakeup is a PMU HP
 * power down reset (0x05) of the HP cores and a PMU LP peripheral power
 * down reset (0x09) of the LP core, with the pads held.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_deep_sleep(void)
{
    QTestState *qts = start(NULL);
    uint32_t soc_reset = qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1);
    uint32_t cause;

    wr(qts, POWER_PD_TOP_CNTL, 0);
    wr(qts, POWER_PD_CNNT_CNTL, 0);
    wr(qts, POWER_PD_HPMEM_CNTL, 0);
    wr(qts, POWER_PD_HP_CPU_CNTL, 0);
    wr(qts, POWER_PD_LPPERI_CNTL, 0);
    wr(qts, HP_SLEEP_DIG_POWER, DIG_TOP_PD | DIG_CPU_PD | DIG_CNNT_PD |
       DIG_MEM_PD);
    wr(qts, LP_SLEEP_LP_DIG_POWER, LP_PERI_PD);
    wr(qts, LP_SLEEP_XTAL, 0);
    wr(qts, SLP_WAKEUP_CNTL2, 1u << WAKE_RTC_TIMER0);
    qtest_writel(qts, LP_SRAM, 0x5a5a1234);
    qtest_writel(qts, L2MEM_DIRECT, 0x12345678);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1, 0);
    qtest_writel(qts, LP_CLKRST_RESET_CAUSE, 0xfc000000);
    rtc_alarm(qts, 300);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    qtest_clock_step(qts, 100 * US);

    g_assert_cmphex(current_state(qts), ==, STATE_LP_SLEEP);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmpuint(clk_hz(qts, "osc-xtal"), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SRAM), ==, 0x5a5a1234);
    g_assert_cmphex(qtest_readl(qts, L2MEM_DIRECT), ==, 0xa5a5a5a5);
    g_assert_cmphex(qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1), ==,
                    soc_reset);

    qtest_clock_step(qts, 2 * MS);
    g_assert_cmphex(current_state(qts), ==, STATE_ACTIVE);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmpuint(clk_hz(qts, "osc-xtal"), ==, 40000000);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS0), ==, 1u << WAKE_RTC_TIMER0);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, HP_INT_WAKEUP);
    cause = qtest_readl(qts, LP_CLKRST_RESET_CAUSE);
    g_assert_cmphex((cause >> 7) & 0x3f, ==, 0x05);
    g_assert_cmphex((cause >> 13) & 1, ==, 1);
    g_assert_cmphex((cause >> 14) & 0x3f, ==, 0x05);
    g_assert_cmphex(cause & 0x3f, ==, 0x09);
    g_assert_cmphex(qtest_readl(qts, LP_SRAM), ==, 0x5a5a1234);
    g_assert_cmphex(qtest_readl(qts, L2MEM_DIRECT), ==, 0xa5a5a5a5);

    /* The PMU held the pads as it powered their domains down. */
    g_assert_cmphex(rd(qts, IMM_PAD_HOLD_ALL) & 7, ==,
                    PAD_HOLD_HP_ST | PAD_HOLD_LP_ST);
    wr(qts, IMM_PAD_HOLD_ALL, TIE_LOW_HP_PAD_HOLD | TIE_LOW_LP_PAD_HOLD);
    g_assert_cmphex(rd(qts, IMM_PAD_HOLD_ALL) & 7, ==, 0);
    qtest_quit(qts);
}

/*
 * The minimum sleep: a wakeup source already active wakes the chip as it
 * falls asleep, but not before HP_MIN_SLP_VAL cycles of LP_DYN_SLOW_CLK.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_min_sleep(void)
{
    QTestState *qts = start(NULL);

    light_sleep_config(qts);
    wr(qts, SLP_WAKEUP_CNTL2, 1u << WAKE_GPIO);
    wr(qts, SLP_WAKEUP_CNTL3, 150 << 8);
    qtest_set_irq_in(qts, PMU_PATH, "wakeup", WAKE_GPIO, 1);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    qtest_clock_step(qts, 500 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_LP_SLEEP);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(current_state(qts), ==, STATE_ACTIVE);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS0), ==, 1u << WAKE_GPIO);
    qtest_quit(qts);
}

/*
 * The EXT wakeup: a pad of GPIO0 to GPIO15 that PMU_EXT_WAKEUP_SEL selects
 * at its PMU_EXT_WAKEUP_LV level, recorded in PMU_EXT_WAKEUP_ST until
 * PMU_EXT_WAKEUP_STATUS_CLR.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_ext_wakeup(void)
{
    QTestState *qts = start(NULL);

    light_sleep_config(qts);
    wr(qts, SLP_WAKEUP_CNTL2, 1u << WAKE_EXT);
    wr(qts, EXT_WAKEUP_SEL, 1u << 5);
    wr(qts, EXT_WAKEUP_LV, 1u << 5);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, 0);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(current_state(qts), ==, STATE_LP_SLEEP);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, 1);
    qtest_clock_step(qts, 200 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_ACTIVE);
    g_assert_cmphex(rd(qts, SLP_WAKEUP_STATUS0), ==, 1u << WAKE_EXT);
    g_assert_cmphex(rd(qts, EXT_WAKEUP_ST), ==, 1u << 5);
    wr(qts, EXT_WAKEUP_CNTL, EXT_STATUS_CLR);
    g_assert_cmphex(rd(qts, EXT_WAKEUP_ST), ==, 0);
    wr(qts, EXT_WAKEUP_CNTL, 0);
    qtest_quit(qts);
}

/*
 * The LP CPU's power: released by an enabled wakeup source (the HP CPU's
 * trigger), recording the cause and raising PMU_LP_CPU_WAKEUP_INT; its
 * sleep request rejected while a reject-enabled source is active, or
 * granted, stalling and resetting it (PMU LP CPU reset, 0x0A, once
 * LPCORE_RESET_CAUSE_PMU_LP_CPU_MASK is cleared). With the LP CPU running
 * the HP system sleeps in HP_SLEEP; its sleep takes the chip on to
 * LP_SLEEP.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_lp_cpu(void)
{
    QTestState *qts = start(NULL);
    uint32_t cause;

    wr(qts, LP_CPU_PWR2, 1u << WAKE_LP_CPU);
    wr(qts, HP_LP_CPU_COMM, HP_TRIGGER_LP);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    g_assert_cmphex(rd(qts, LP_CPU_PWR3), ==, 1u << WAKE_LP_CPU);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==,
                    LP_INT_HP_SW_TRIGGER | LP_INT_LP_CPU_WAKEUP);
    g_assert(source(qts, SRC_PMU_1) == false);

    /* Rejected while the trigger holds and PWR4 enables it. */
    wr(qts, LP_CPU_PWR4, 1u << WAKE_LP_CPU);
    wr(qts, LP_CPU_PWR1, 1u << 31);
    g_assert_cmphex(rd(qts, LP_CPU_PWR5), ==, 1u << WAKE_LP_CPU);
    g_assert_cmphex(rd(qts, LP_INT_RAW) & LP_INT_LP_CPU_REJECT, ==,
                    LP_INT_LP_CPU_REJECT);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));

    /* With the LP CPU running, the HP system sleeps in HP_SLEEP. */
    light_sleep_config(qts);
    wr(qts, SLP_WAKEUP_CNTL2, 1u << WAKE_RTC_TIMER0);
    rtc_alarm(qts, 1500);
    wr(qts, SLP_WAKEUP_CNTL0, SLEEP_REQ);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_HP_SLEEP);
    g_assert_cmpuint(clk_hz(qts, "lp-dyn-fast"), ==, 40000000);

    /* The LP CPU's sleep, stalled and reset, takes it on to LP_SLEEP. */
    wr(qts, LP_INT_CLR, 0xffffe000);
    wr(qts, LP_CPU_PWR4, 0);
    wr(qts, LP_CPU_PWR0, rd(qts, LP_CPU_PWR0) | LP_CPU_SLP_STALL_EN |
       LP_CPU_SLP_RESET_EN);
    cause = qtest_readl(qts, LP_CLKRST_RESET_CAUSE);
    wr(qts, LP_CPU_PWR1, 1u << 31);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    g_assert_cmphex(rd(qts, LP_CPU_PWR0) & 3, ==, 2);
    /* The mask keeps the first run's cause. */
    g_assert_cmphex(qtest_readl(qts, LP_CLKRST_RESET_CAUSE) & 0x3f, ==,
                    cause & 0x3f);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_LP_SLEEP);
    g_assert_cmpuint(clk_hz(qts, "lp-dyn-fast"), ==, 150000);

    /* The HP trigger wakes the LP CPU from LP_SLEEP, back to HP_SLEEP. */
    wr(qts, HP_LP_CPU_COMM, HP_TRIGGER_LP);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(current_state(qts), ==, STATE_HP_SLEEP);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));

    /* Unmasked, the PMU's reset of the LP CPU records 0x0A. */
    qtest_writel(qts, LP_CLKRST_RESET_CAUSE,
                 qtest_readl(qts, LP_CLKRST_RESET_CAUSE) &
                 ~LPCORE_CAUSE_MASK_PMU);
    wr(qts, LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    wr(qts, LP_CPU_PWR1, 1u << 31);
    g_assert_cmphex(qtest_readl(qts, LP_CLKRST_RESET_CAUSE) & 0x3f, ==, 0x0a);

    /* The RTC timer wakes the HP system from LP_SLEEP. */
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(current_state(qts), ==, STATE_ACTIVE);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    qtest_quit(qts);
}

/*
 * The immediate controls: TIE_HIGH and TIE_LOW power a PLL up and down at
 * once; UPDATE takes the current state's clock gates; CPU_SW_STALL's
 * 0x86 stalls a core; the LP oscillators' settings act at once.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_immediate(void)
{
    QTestState *qts = start(NULL);

    wr(qts, IMM_HP_CK_POWER, TIE_HIGH_XPD_APLL);
    g_assert_cmphex((rd(qts, CLK_STATE0) >> 29) & 1, ==, 1);
    qtest_clock_step(qts, 20 * US);
    g_assert_cmpuint(clk_hz(qts, "apll"), !=, 0);
    wr(qts, IMM_HP_CK_POWER, TIE_LOW_XPD_APLL);
    g_assert_cmphex((rd(qts, CLK_STATE0) >> 29) & 1, ==, 0);
    g_assert_cmpuint(clk_hz(qts, "apll"), ==, 0);

    /* The PMU's clock gate of SYSREG_APB_CLK (bit 25) in HP_ACTIVE. */
    wr(qts, HP_ACTIVE_ICG_HP_FUNC, ~(1u << 25));
    g_assert_cmpuint(clk_hz(qts, "sysreg-apb"), !=, 0);
    wr(qts, IMM_HP_FUNC_ICG, UPDATE_ICG);
    g_assert_cmpuint(clk_hz(qts, "sysreg-apb"), !=, 0);
    wr(qts, IMM_SLEEP_SYSCLK, TIE_HIGH_ICG_SLP_SEL);
    g_assert_cmpuint(clk_hz(qts, "sysreg-apb"), ==, 0);
    g_assert_cmpuint(clk_hz(qts, "intrmtx-apb"), !=, 0);
    wr(qts, IMM_SLEEP_SYSCLK, TIE_LOW_ICG_SLP_SEL);
    g_assert_cmpuint(clk_hz(qts, "sysreg-apb"), !=, 0);

    wr(qts, CPU_SW_STALL, 0x86u << 24);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    wr(qts, CPU_SW_STALL, 0xffu << 24);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));

    g_assert_cmpuint(clk_hz(qts, "osc-rc32k"), ==, 0);
    wr(qts, HP_SLEEP_LP_CK_POWER, LP_XPD_FOSC | LP_XPD_RC32K);
    g_assert_cmpuint(clk_hz(qts, "osc-rc32k"), ==, 32768);
    qtest_quit(qts);
}

/*
 * The output regulators' waiting counters: an enabled regulator counts
 * LP_DYN_SLOW_CLK / 16 and raises its TARGET0 and TARGET1 interrupts.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_ldo_counter(void)
{
    QTestState *qts = start(NULL);
    int64_t t = now(qts);

    /* VO3: TARGET0 2 (213 us), TARGET1 4 (427 us). */
    wr(qts, EXT_LDO_P0_0P2A, LDO_TARGET0(2) | LDO_TARGET1(4) | LDO_XPD);
    qtest_clock_step(qts, t + 210 * US - now(qts));
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, LDO_INT_T0_VO3);
    g_assert_cmphex(rd(qts, LP_INT_RAW), ==, LDO_INT_T0_VO3);
    qtest_clock_step(qts, 220 * US);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==,
                    LDO_INT_T0_VO3 | LDO_INT_T1_VO3);
    qtest_quit(qts);
}

/* The LP CPU's software interrupt to the HP CPU: PMU_SW_INT. */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_sw_int(void)
{
    QTestState *qts = start(NULL);

    wr(qts, HP_LP_CPU_COMM, LP_TRIGGER_HP);
    g_assert_cmphex(rd(qts, HP_INT_RAW), ==, HP_INT_SW);
    g_assert(!source(qts, SRC_PMU_0));
    wr(qts, HP_INT_ENA, HP_INT_SW);
    g_assert_cmphex(rd(qts, HP_INT_ST), ==, HP_INT_SW);
    g_assert(source(qts, SRC_PMU_0));
    wr(qts, HP_INT_CLR, HP_INT_SW);
    g_assert(!source(qts, SRC_PMU_0));
    qtest_quit(qts);
}

/*
 * The brown-out detector on VDD_ANA, at 2.5 V and so above the 2.4 V
 * default threshold. Software takes mode 1's reset from the analog
 * default (FIB_ENABLE bit 1) and arms mode 0; raising the threshold to
 * 2.6 V makes a brown-out, whose counter raises BOD_MODE0_INT after
 * INTR_WAIT cycles of LP_DYN_FAST_CLK and, with RESET_ENA, resets the
 * system after RESET_WAIT, with cause 0x0F.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_brownout(void)
{
    QTestState *qts = start("-global esp32p4-lp-ana.vdda-mv=2500");
    int64_t t;
    uint32_t cause;

    qtest_writel(qts, FIB_ENABLE, 0xfffffffd);
    qtest_writel(qts, BOD_MODE1_CNTL, 0);
    qtest_writel(qts, ANA_INT_ENA, INT_BOD_MODE0);
    bias_write(qts, BIAS_DREF_VDDA, (6 << 5) | (6 << 2));
    g_assert_cmphex(qtest_readl(qts, ANA_INT_RAW), ==, 0);
    t = now(qts);
    qtest_writel(qts, BOD_MODE0_CNTL, MODE0_INTR_WAIT(100) |
                 MODE0_RESET_WAIT(1000) | MODE0_INTR_ENA);
    /* 100 cycles of 40 MHz: 2.5 us. */
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(qtest_readl(qts, ANA_INT_RAW), ==, 0);
    qtest_clock_step(qts, t + 2500 + 1 - now(qts));
    g_assert_cmphex(qtest_readl(qts, ANA_INT_RAW), ==, INT_BOD_MODE0);
    g_assert_cmphex(qtest_readl(qts, ANA_INT_ST), ==, INT_BOD_MODE0);
    g_assert_cmphex(qtest_readl(qts, ANA_LP_INT_RAW), ==, INT_BOD_MODE0);
    g_assert(source(qts, SRC_LP_ANAPERI));
    /* Its interrupt holds while the brown-out does. */
    qtest_writel(qts, ANA_INT_CLR, INT_BOD_MODE0);
    g_assert(source(qts, SRC_LP_ANAPERI));
    /* The supply back above the threshold ends it. */
    qom_set_uint(qts, LP_ANA_PATH, "vdda-mv", 3300);
    qtest_writel(qts, ANA_INT_CLR, INT_BOD_MODE0);
    g_assert(!source(qts, SRC_LP_ANAPERI));

    /* Mode 0's system reset. */
    qtest_writel(qts, LP_SRAM, 0xabcd0001);
    qtest_writel(qts, BOD_MODE0_CNTL, MODE0_INTR_WAIT(100) |
                 MODE0_RESET_WAIT(200) | MODE0_INTR_ENA | MODE0_RESET_ENA |
                 MODE0_RESET_SEL);
    qom_set_uint(qts, LP_ANA_PATH, "vdda-mv", 2500);
    qtest_clock_step(qts, 20 * US);
    cause = qtest_readl(qts, LP_CLKRST_RESET_CAUSE);
    g_assert_cmphex((cause >> 7) & 0x3f, ==, 0x0f);
    g_assert_cmphex((cause >> 14) & 0x3f, ==, 0x0f);
    g_assert_cmphex(cause & 0x3f, ==, 0x0f);
    g_assert_cmphex(qtest_readl(qts, LP_SRAM), ==, 0xabcd0001);
    /* The reset took BIAS back to its 2.4 V threshold: no brown-out. */
    g_assert_cmphex(qtest_readl(qts, BOD_MODE0_CNTL), ==, 0x0ffc0100);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0x0f);
    qtest_quit(qts);
}

/*
 * Mode 1, by the analog default: a brown-out resets the system at once.
 * A power glitch resets it while POWER_GLITCH_RESET_ENA is set (0x13).
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_brownout_mode1(void)
{
    QTestState *qts = start(NULL);

    qtest_writel(qts, LP_CLKRST_RESET_CAUSE, 0xfc000000);
    qom_set_uint(qts, LP_ANA_PATH, "vdda-mv", 2300);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0x0f);
    qom_set_uint(qts, LP_ANA_PATH, "vdda-mv", 3300);
    qtest_clock_step(qts, 1 * US);

    qtest_writel(qts, LP_CLKRST_RESET_CAUSE, 0xfc000000);
    qtest_set_irq_in(qts, LP_ANA_PATH, "power-glitch", 0, 1);
    qtest_set_irq_in(qts, LP_ANA_PATH, "power-glitch", 0, 0);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0);
    qtest_writel(qts, PG_GLITCH_CNTL, 1u << 31);
    qtest_set_irq_in(qts, LP_ANA_PATH, "power-glitch", 0, 1);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0x13);
    qtest_quit(qts);
}

/*
 * VDD_BAT's under-voltage filter: the flag sets UNDERVOLTAGE_TARGET cycles
 * into the under-voltage, raising its interrupt, and clears as the count
 * falls back to UPVOLTAGE_TARGET once the supply is good.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_vbat_filter(void)
{
    QTestState *qts = start(NULL);

    /* 80 cycles of 40 MHz under, 2 us; back down to 40, 1 us. */
    qtest_writel(qts, VDDBAT_BOD_CNTL, FILTER_UNDER(80) | FILTER_UP(40));
    qtest_writel(qts, ANA_INT_ENA, INT_VBAT_UNDER | INT_VBAT_UP);
    qom_set_uint(qts, LP_ANA_PATH, "vbat-mv", 2300);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, VDDBAT_BOD_CNTL) & 1, ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(qtest_readl(qts, VDDBAT_BOD_CNTL) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, ANA_INT_RAW), ==, INT_VBAT_UNDER);
    g_assert(source(qts, SRC_LP_ANAPERI));
    qtest_writel(qts, ANA_INT_CLR, INT_VBAT_UNDER);
    qom_set_uint(qts, LP_ANA_PATH, "vbat-mv", 3300);
    qtest_clock_step(qts, 500);
    g_assert_cmphex(qtest_readl(qts, VDDBAT_BOD_CNTL) & 1, ==, 1);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmphex(qtest_readl(qts, VDDBAT_BOD_CNTL) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, ANA_INT_RAW), ==, INT_VBAT_UP);
    /* VDD_BAT's under-voltage was recorded. */
    g_assert_cmphex((qtest_readl(qts, VDD_SOURCE_CNTL) >> 8) & 0xff, ==, 2);
    qtest_quit(qts);
}

/* A glitch on the HP system regulator lasting @ns. */
static void hp_glitch(QTestState *qts, int64_t ns)
{
    qtest_set_irq_in(qts, LP_ANA_PATH, "hp-supply-glitch", 0, 1);
    qtest_clock_step(qts, ns);
    qtest_set_irq_in(qts, LP_ANA_PATH, "hp-supply-glitch", 0, 0);
    qtest_clock_step(qts, 1 * US);
}

/*
 * The power supply detector: a glitch of 50 ns or more on the HP system
 * regulator resets the system (0x1B), only while EFUSE_PVT_GLITCH_EN
 * enables it, which a factory-fresh chip does not.
 */
/* [spec:nuos:req:emu.esp32p4.low-power/test] */
static void test_psdet(void)
{
    QTestState *qts = start(NULL);
    uint32_t img[84] = { 0 };
    g_autofree char *path = NULL;
    g_autofree char *args = NULL;
    GError *err = NULL;
    int fd;

    qtest_writel(qts, LP_CLKRST_RESET_CAUSE, 0xfc000000);
    hp_glitch(qts, 1 * US);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0);
    qtest_quit(qts);

    /* EFUSE_PVT_GLITCH_EN, RD_REPEAT_DATA4 bit 2, in the eFuse file. */
    fd = g_file_open_tmp("esp32p4-pmu-efuse-XXXXXX.bin", &path, &err);
    g_assert_no_error(err);
    img[5] = cpu_to_le32(1u << 2);
    g_assert_cmpint(pwrite(fd, img, sizeof(img), 0), ==, sizeof(img));
    close(fd);
    args = g_strdup_printf("-drive file=%s,if=none,format=raw,id=efuse "
                           "-global driver=nvram.esp32p4.efuse,"
                           "property=drive,value=efuse", path);
    qts = start(args);
    qtest_writel(qts, LP_CLKRST_RESET_CAUSE, 0xfc000000);
    hp_glitch(qts, 40);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0);
    hp_glitch(qts, 50);
    g_assert_cmphex((qtest_readl(qts, LP_CLKRST_RESET_CAUSE) >> 7) & 0x3f,
                    ==, 0x1b);
    qtest_quit(qts);
    unlink(path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/pmu/reset", test_reset);
    qtest_add_func("/esp32p4/pmu/reject", test_reject);
    qtest_add_func("/esp32p4/pmu/light-sleep", test_light_sleep);
    qtest_add_func("/esp32p4/pmu/deep-sleep", test_deep_sleep);
    qtest_add_func("/esp32p4/pmu/min-sleep", test_min_sleep);
    qtest_add_func("/esp32p4/pmu/ext-wakeup", test_ext_wakeup);
    qtest_add_func("/esp32p4/pmu/lp-cpu", test_lp_cpu);
    qtest_add_func("/esp32p4/pmu/immediate", test_immediate);
    qtest_add_func("/esp32p4/pmu/ldo-counter", test_ldo_counter);
    qtest_add_func("/esp32p4/pmu/sw-int", test_sw_int);
    qtest_add_func("/esp32p4/pmu/brownout", test_brownout);
    qtest_add_func("/esp32p4/pmu/brownout-mode1", test_brownout_mode1);
    qtest_add_func("/esp32p4/pmu/vbat-filter", test_vbat_filter);
    qtest_add_func("/esp32p4/pmu/psdet", test_psdet);
    return g_test_run();
}
