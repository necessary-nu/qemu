/*
 * QTest testcase for the ESP32 ULP coprocessor and RTC I2C controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define RTC_CNTL                0x3ff48000
#define STATE0                  (RTC_CNTL + 0x18)
#define TIMER2                  (RTC_CNTL + 0x20)
#define INT_ENA                 (RTC_CNTL + 0x3c)
#define INT_RAW                 (RTC_CNTL + 0x40)
#define INT_CLR                 (RTC_CNTL + 0x48)
#define STORE0                  (RTC_CNTL + 0x4c)
#define CLK_CONF                (RTC_CNTL + 0x70)
#define ULP_CP_SLP_TIMER_EN     (1u << 24)
#define INT_ULP_CP              (1u << 5)
#define FAST_CLK_RTC_SEL_8M     (1u << 29)
#define START_WAIT(v)           ((v) << 15)
#define START_WAIT_MASK         (0x1ffu << 15)

#define RTCIO                   0x3ff48400
#define RTCIO_TOUCH_PAD(n)      (RTCIO + 0x94 + 4 * (n))
#define RTCIO_SAR_I2C_IO        (RTCIO + 0xc4)
#define TOUCH_RUE               (1u << 27)
#define TOUCH_MUX_SEL           (1u << 19)
#define TOUCH_FUN_SEL(f)        ((f) << 17)
#define TOUCH_FUN_IE            (1u << 13)
#define SCL_SEL(v)              ((v) << 28)
#define SDA_SEL(v)              ((v) << 30)

#define SENS                    0x3ff48800
#define READ_CTRL               (SENS + 0x00)
#define SLEEP_CYC(n)            (SENS + 0x18 + 4 * (n))
#define START_FORCE             (SENS + 0x2c)
#define SLAVE_ADDR1             (SENS + 0x3c)
#define SLAVE_ADDR3             (SENS + 0x44)
#define SLAVE_ADDR4             (SENS + 0x48)
#define TSENS_CTRL              (SENS + 0x4c)
#define SAR_I2C_CTRL            (SENS + 0x50)
#define MEAS_START1             (SENS + 0x54)
#define DATA_INV                (1u << 28)
#define FORCE_START_TOP         (1u << 8)
#define START_TOP               (1u << 9)
#define PC_INIT(pc)             ((pc) << 11)
#define START_FORCE_RESET       0xfu
#define I2C_DONE                (1u << 30)
#define I2C_RDATA(v)            (((v) >> 22) & 0xff)
#define TSENS_OUT(v)            (((v) >> 22) & 0xff)
#define TSENS_RDY               (1u << 30)
#define TSENS_POWER_UP          (1u << 24)
#define TSENS_POWER_UP_FORCE    (1u << 25)
#define TSENS_DUMP_OUT          (1u << 26)
#define SAR_I2C_START           (1u << 28)
#define SAR_I2C_START_FORCE     (1u << 29)
#define MEAS1_DONE              (1u << 16)

#define RTC_I2C                 0x3ff48c00
#define RTC_I2C_SCL_LOW         (RTC_I2C + 0x00)
#define RTC_I2C_CTRL            (RTC_I2C + 0x04)
#define RTC_I2C_DEBUG_STATUS    (RTC_I2C + 0x08)
#define RTC_I2C_TIMEOUT         (RTC_I2C + 0x0c)
#define RTC_I2C_INT_RAW         (RTC_I2C + 0x20)
#define RTC_I2C_INT_CLR         (RTC_I2C + 0x24)
#define RTC_I2C_INT_EN          (RTC_I2C + 0x28)
#define RTC_I2C_INT_ST          (RTC_I2C + 0x2c)
#define RTC_I2C_SDA_DUTY        (RTC_I2C + 0x30)
#define RTC_I2C_SCL_HIGH        (RTC_I2C + 0x38)
#define RTC_I2C_SCL_START       (RTC_I2C + 0x40)
#define RTC_I2C_SCL_STOP        (RTC_I2C + 0x44)
#define RTC_I2C_MS_MODE         (1u << 4)
#define RTC_I2C_TX_LSB_FIRST    (1u << 6)
#define I2C_INT_TIME_OUT        (1u << 7)
#define I2C_INT_TRANS_COMPLETE  (1u << 6)
#define I2C_INT_MASTER_COMPLETE (1u << 5)
#define I2C_ACK_VAL             (1u << 0)
#define I2C_TIMED_OUT           (1u << 2)

#define RTC_SLOW_MEM            0x50000000
#define SENS_PATH               "/machine/soc/sens"

/* ULP instructions (ESP-IDF ulp_fsm/include/esp32/ulp.h) */
#define ALU_ADD 0
#define ALU_SUB 1
#define ALU_AND 2
#define ALU_OR  3
#define ALU_MOV 4
#define ALU_LSH 5
#define ALU_RSH 6
#define I_ALUR(sel, rd, rs, rt) ((7u << 28) | ((sel) << 21) | ((rt) << 4) | \
                                 ((rs) << 2) | (rd))
#define I_ALUI(sel, rd, rs, imm) ((7u << 28) | (1u << 25) | ((sel) << 21) | \
                                  (((imm) & 0xffff) << 4) | ((rs) << 2) | \
                                  (rd))
#define I_MOVI(rd, imm)         I_ALUI(ALU_MOV, rd, 0, imm)
#define I_STAGE_INC(n)          ((7u << 28) | (2u << 25) | (0u << 21) | \
                                 ((n) << 4))
#define I_STAGE_DEC(n)          ((7u << 28) | (2u << 25) | (1u << 21) | \
                                 ((n) << 4))
#define I_STAGE_RST()           ((7u << 28) | (2u << 25) | (2u << 21))
#define I_ST(rdata, raddr, off) ((6u << 28) | (4u << 25) | \
                                 (((off) & 0x7ff) << 10) | ((raddr) << 2) | \
                                 (rdata))
#define I_LD(rd, raddr, off)    ((13u << 28) | (((off) & 0x7ff) << 10) | \
                                 ((raddr) << 2) | (rd))
#define I_JUMP(addr)            ((8u << 28) | ((addr) << 2))
#define I_JUMP_EQ(addr)         ((8u << 28) | (1u << 22) | ((addr) << 2))
#define I_JUMP_OV(addr)         ((8u << 28) | (2u << 22) | ((addr) << 2))
#define I_JUMP_R(r)             ((8u << 28) | (1u << 21) | (r))
#define REL(step)               ((((step) < 0) << 24) | \
                                 ((((step) < 0 ? -(step) : (step)) & 0x7f) \
                                  << 17))
#define I_JUMPR_LT(step, thr)   ((8u << 28) | (1u << 25) | REL(step) | (thr))
#define I_JUMPR_GE(step, thr)   ((8u << 28) | (1u << 25) | REL(step) | \
                                 (1u << 16) | (thr))
#define I_JUMPS_LT(step, thr)   ((8u << 28) | (2u << 25) | REL(step) | (thr))
#define I_JUMPS_GE(step, thr)   ((8u << 28) | (2u << 25) | REL(step) | \
                                 (1u << 15) | (thr))
#define I_JUMPS_LE(step, thr)   ((8u << 28) | (2u << 25) | REL(step) | \
                                 (2u << 15) | (thr))
#define I_WAKE()                ((9u << 28) | 1u)
#define I_SLEEP(n)              ((9u << 28) | (1u << 25) | (n))
#define I_HALT()                (11u << 28)
#define I_WAIT(n)               ((4u << 28) | (n))
#define I_ADC(rd, sar, mux)     ((5u << 28) | ((sar) << 6) | ((mux) << 2) | \
                                 (rd))
#define I_TSENS(rd, wait)       ((10u << 28) | ((wait) << 2) | (rd))
#define I_I2C_RD(sub, hi, lo, sel) ((3u << 28) | ((sel) << 22) | \
                                    ((hi) << 19) | ((lo) << 16) | (sub))
#define I_I2C_WR(sub, val, hi, lo, sel) ((3u << 28) | (1u << 27) | \
                                         ((sel) << 22) | ((hi) << 19) | \
                                         ((lo) << 16) | ((val) << 8) | (sub))
#define ULP_REG(a)              (((a) - RTC_CNTL) / 4)
#define I_REG_RD(a, hi, lo)     ((2u << 28) | ((hi) << 23) | ((lo) << 18) | \
                                 ULP_REG(a))
#define I_REG_WR(a, hi, lo, d)  ((1u << 28) | ((hi) << 23) | ((lo) << 18) | \
                                 ((d) << 10) | ULP_REG(a))

/* Words of RTC slow memory the programs keep their results in */
#define DATA                    0x100

#define US                      1000
#define MS                      (1000 * US)
/* RTC_FAST_CLK at 8 MHz */
#define FAST_NS                 125

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    qtest_writel(qts, CLK_CONF, qtest_readl(qts, CLK_CONF) |
                 FAST_CLK_RTC_SEL_8M);
    return qts;
}

static uint32_t rd(QTestState *qts, uint64_t a)
{
    return qtest_readl(qts, a);
}

static void wr(QTestState *qts, uint64_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static uint32_t mem(QTestState *qts, unsigned word)
{
    return rd(qts, RTC_SLOW_MEM + 4 * word);
}

static void load(QTestState *qts, unsigned at, const uint32_t *prog,
                 size_t n)
{
    for (size_t i = 0; i < n; i++) {
        wr(qts, RTC_SLOW_MEM + 4 * (at + i), prog[i]);
    }
}

/* Start the ULP from software at pc; it runs 2 + START_WAIT slow cycles on */
static void run(QTestState *qts, unsigned pc)
{
    wr(qts, START_FORCE, START_FORCE_RESET | PC_INIT(pc) | FORCE_START_TOP);
    wr(qts, START_FORCE, START_FORCE_RESET | PC_INIT(pc) | FORCE_START_TOP |
       START_TOP);
}

static void set_sens_prop(QTestState *qts, const char *prop, int64_t value)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                           "{ 'path': %s, 'property': %s, "
                           "'value': %" PRId64 " } }", SENS_PATH, prop,
                           value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * ALU operations and their flags, LD and ST with offsets, and ST's upper
 * half-word: the PC and the address register's number.
 */
static void test_alu_ld_st(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        /* 0 */ I_MOVI(1, 5),
        /* 1 */ I_MOVI(2, DATA),
        /* 2 */ I_ALUI(ALU_ADD, 1, 1, 3),          /* R1 = 8 */
        /* 3 */ I_ST(1, 2, 0),
        /* 4 */ I_ALUI(ALU_LSH, 3, 1, 4),          /* R3 = 0x80 */
        /* 5 */ I_ST(3, 2, 1),
        /* 6 */ I_MOVI(0, 0xffff),
        /* 7 */ I_ALUI(ALU_ADD, 0, 0, 2),          /* R0 = 1, overflow */
        /* 8 */ I_JUMP_OV(10),
        /* 9 */ I_HALT(),
        /* 10 */ I_ST(0, 2, 2),
        /* 11 */ I_ALUR(ALU_SUB, 0, 0, 0),         /* zero */
        /* 12 */ I_JUMP_EQ(14),
        /* 13 */ I_HALT(),
        /* 14 */ I_LD(3, 2, 1),                    /* R3 = 0x80 */
        /* 15 */ I_ALUI(ALU_RSH, 3, 3, 3),         /* R3 = 0x10 */
        /* 16 */ I_ALUR(ALU_OR, 3, 3, 1),          /* R3 = 0x18 */
        /* 17 */ I_ALUI(ALU_AND, 3, 3, 0x1c),      /* R3 = 0x18 */
        /* 18 */ I_MOVI(1, DATA + 3),
        /* 19 */ I_ST(3, 1, 0),
        /* 20 */ I_HALT(),
    };

    load(qts, 0, prog, ARRAY_SIZE(prog));
    for (unsigned i = 0; i < 4; i++) {
        wr(qts, RTC_SLOW_MEM + 4 * (DATA + i), 0xdeadbeef);
    }
    run(qts, 0);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmphex(mem(qts, DATA), ==, (3u << 21) | (2u << 16) | 8);
    g_assert_cmphex(mem(qts, DATA + 1), ==, (5u << 21) | (2u << 16) | 0x80);
    g_assert_cmphex(mem(qts, DATA + 2), ==, (10u << 21) | (2u << 16) | 1);
    g_assert_cmphex(mem(qts, DATA + 3), ==, (19u << 21) | (1u << 16) | 0x18);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * JUMPR on R0 and JUMPS on the stage count, in loops; and the start time
 * (2 + START_WAIT RTC_SLOW_CLK cycles) and the cycle counts of WAIT and
 * the instructions around it.
 */
static void test_branches_and_timing(void)
{
    QTestState *qts = start();
    const uint32_t loops[] = {
        /* 0 */ I_MOVI(2, DATA),
        /* 1 */ I_MOVI(0, 0),
        /* 2 */ I_ALUI(ALU_ADD, 0, 0, 1),
        /* 3 */ I_JUMPR_LT(-1, 10),                /* until R0 == 10 */
        /* 4 */ I_ST(0, 2, 0),
        /* 5 */ I_STAGE_RST(),
        /* 6 */ I_STAGE_INC(3),
        /* 7 */ I_ALUI(ALU_ADD, 0, 0, 1),
        /* 8 */ I_JUMPS_LT(-2, 30),                /* stage 3, 6, ... 30 */
        /* 9 */ I_STAGE_DEC(25),                   /* stage 5 */
        /* 10 */ I_JUMPS_LE(2, 4),
        /* 11 */ I_JUMPS_GE(2, 5),
        /* 12 */ I_HALT(),
        /* 13 */ I_ST(0, 2, 1),                    /* R0 = 10 + 10 */
        /* 14 */ I_HALT(),
    };
    const uint32_t wait[] = {
        /* 32 */ I_MOVI(2, DATA + 2),
        /* 33 */ I_WAIT(7990),
        /* 34 */ I_ST(2, 2, 0),
        /* 35 */ I_HALT(),
    };
    uint64_t start_ns, st_ns;

    load(qts, 0, loops, ARRAY_SIZE(loops));
    run(qts, 0);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, 10);
    g_assert_cmpuint(mem(qts, DATA + 1) & 0xffff, ==, 20);

    /*
     * Started with START_WAIT 0 the ULP runs 2 RTC_SLOW_CLK cycles (13.3
     * us) later; the ST comes after MOVE (6 cycles) and WAIT (2 + 7990 +
     * 4 cycles): 1 ms of RTC_FAST_CLK.
     */
    wr(qts, TIMER2, rd(qts, TIMER2) & ~START_WAIT_MASK);
    load(qts, 32, wait, ARRAY_SIZE(wait));
    wr(qts, RTC_SLOW_MEM + 4 * (DATA + 2), 0);
    run(qts, 32);
    start_ns = qtest_clock_step(qts, 1) + 2 * (uint64_t)(1000 * MS) / 150000;
    st_ns = start_ns + (6 + 2 + 7990 + 4) * FAST_NS;
    qtest_clock_set(qts, st_ns - 1 * US);
    g_assert_cmphex(mem(qts, DATA + 2), ==, 0);
    qtest_clock_set(qts, st_ns + 1 * US);
    g_assert_cmphex(mem(qts, DATA + 2), ==,
                    (34u << 21) | (2u << 16) | (DATA + 2));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * The ULP timer: with RTC_CNTL_ULP_CP_SLP_TIMER_EN the ULP runs every
 * SENS_ULP_CP_SLEEP_CYC0 (+ the FSM's wakeup and power-down) RTC_SLOW_CLK
 * cycles; SLEEP selects another period; clearing the enable stops it. WAKE
 * raises RTC_CNTL's ULP_CP interrupt.
 */
static void test_timer(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        /* 0 */ I_MOVI(2, DATA),
        /* 1 */ I_LD(0, 2, 0),
        /* 2 */ I_ALUI(ALU_ADD, 0, 0, 1),
        /* 3 */ I_ST(0, 2, 0),
        /* 4 */ I_JUMPR_LT(3, 5),
        /* 5 */ I_SLEEP(1),                        /* from the 5th run */
        /* 6 */ I_WAKE(),
        /* 7 */ I_HALT(),
    };
    /* Periods of 1 ms and 2 ms, less the FSM's 2 + 2 + START_WAIT */
    uint32_t wait = (rd(qts, TIMER2) & START_WAIT_MASK) >> 15;
    uint32_t over = 2 + 2 + wait;
    uint64_t period1 = 150 * (uint64_t)(1000 * MS) / 150000;
    uint32_t n;

    g_assert_cmpuint(wait, ==, 16);
    load(qts, 0, prog, ARRAY_SIZE(prog));
    wr(qts, RTC_SLOW_MEM + 4 * DATA, 0);
    wr(qts, SLEEP_CYC(0), 150 - over);
    wr(qts, SLEEP_CYC(1), 300 - over);
    wr(qts, START_FORCE, START_FORCE_RESET | PC_INIT(0));
    wr(qts, INT_CLR, INT_ULP_CP);

    wr(qts, STATE0, rd(qts, STATE0) | ULP_CP_SLP_TIMER_EN);
    /* The first run comes a whole period after the enable */
    qtest_clock_step(qts, period1 - 50 * US);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, 0);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, 1);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_ULP_CP, ==, 0);

    /* Runs 2 to 5 a period apart; the 5th selects CYC1 and wakes */
    qtest_clock_step(qts, 4 * period1);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, 5);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_ULP_CP, ==, INT_ULP_CP);
    wr(qts, INT_CLR, INT_ULP_CP);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_ULP_CP, ==, 0);

    /* Then every 2 ms: 5 more in 10 ms */
    qtest_clock_step(qts, 10 * period1 + 500 * US);
    n = mem(qts, DATA) & 0xffff;
    g_assert_cmpuint(n, ==, 10);

    wr(qts, STATE0, rd(qts, STATE0) & ~ULP_CP_SLP_TIMER_EN);
    qtest_clock_step(qts, 10 * period1);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, n);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * REG_RD and REG_WR on the RTC peripherals through the ULP's register
 * address: a field write leaving the rest of the register, a read of more
 * than 16 bits returning [low + 15:low], and the ULP clearing its own
 * timer enable as ESP-IDF's I_END() does.
 */
static void test_reg_rd_wr(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        /* 0 */ I_REG_WR(STORE0, 15, 8, 0xa5),
        /* 1 */ I_REG_WR(STORE0, 31, 20, 0x3c),    /* data is 8 bits */
        /* 2 */ I_MOVI(2, DATA),
        /* 3 */ I_REG_RD(SLEEP_CYC(4), 31, 0),
        /* 4 */ I_ST(0, 2, 0),
        /* 5 */ I_REG_RD(RTCIO + 0xc8, 27, 16),    /* RTCIO_DATE */
        /* 6 */ I_ST(0, 2, 1),
        /* 7 */ I_REG_WR(STATE0, 24, 24, 0),
        /* 8 */ I_HALT(),
    };

    load(qts, 0, prog, ARRAY_SIZE(prog));
    wr(qts, STORE0, 0x12345678);
    wr(qts, SLEEP_CYC(4), 0x9abcdef0);
    wr(qts, START_FORCE, START_FORCE_RESET | PC_INIT(0));
    wr(qts, SLEEP_CYC(0), 10);
    wr(qts, STATE0, rd(qts, STATE0) | ULP_CP_SLP_TIMER_EN);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(rd(qts, STORE0), ==, 0x03c4a578);
    g_assert_cmphex(mem(qts, DATA) & 0xffff, ==, 0xdef0);
    g_assert_cmphex(mem(qts, DATA + 1) & 0xffff, ==, 0x160);
    g_assert_cmphex(rd(qts, STATE0) & ULP_CP_SLP_TIMER_EN, ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * The ADC instruction converts the pad it selects through ADC1's RTC
 * controller, which the ULP starts (no MEAS1_START_FORCE) and whose pad it
 * picks (no SAR1_EN_PAD_FORCE); the result also lands in SENS.
 */
static void test_adc(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        I_MOVI(2, DATA),
        I_ADC(1, 0, 7),                            /* ADC1 channel 6 */
        I_ST(1, 2, 0),
        I_HALT(),
    };

    set_sens_prop(qts, "adc1-ch6-mv", 1000);
    wr(qts, READ_CTRL, rd(qts, READ_CTRL) | DATA_INV);
    load(qts, 0, prog, ARRAY_SIZE(prog));
    run(qts, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, 1050);
    g_assert_cmphex(rd(qts, MEAS_START1) & (MEAS1_DONE | 0xffff), ==,
                    MEAS1_DONE | 1050);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * The temperature sensor, read by the ULP's TSENS and by software's
 * TSENS_DUMP_OUT: the die temperature in degrees Fahrenheit.
 */
static void test_tsens(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        I_MOVI(2, DATA),
        I_TSENS(1, 100),
        I_ST(1, 2, 0),
        I_HALT(),
    };

    set_sens_prop(qts, "tsens-temp-mc", 40000);
    load(qts, 0, prog, ARRAY_SIZE(prog));
    run(qts, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpuint(mem(qts, DATA) & 0xffff, ==, 104);
    g_assert_cmpuint(TSENS_OUT(rd(qts, SLAVE_ADDR3)), ==, 104);

    set_sens_prop(qts, "tsens-temp-mc", -10000);
    wr(qts, TSENS_CTRL, rd(qts, TSENS_CTRL) | TSENS_POWER_UP_FORCE |
       TSENS_POWER_UP);
    wr(qts, TSENS_CTRL, rd(qts, TSENS_CTRL) | TSENS_DUMP_OUT);
    g_assert_cmphex(rd(qts, SLAVE_ADDR3) & TSENS_RDY, ==, TSENS_RDY);
    g_assert_cmpuint(TSENS_OUT(rd(qts, SLAVE_ADDR3)), ==, 14);
    qtest_quit(qts);
}

/*
 * Put the RTC I2C controller on TOUCH_PAD0 (GPIO4, SCL) and TOUCH_PAD1
 * (GPIO0, SDA), RTC function 1, inputs enabled and pulled up, at 100 kHz.
 */
static void rtc_i2c_setup(QTestState *qts, bool pull_up)
{
    for (unsigned n = 0; n < 2; n++) {
        wr(qts, RTCIO_TOUCH_PAD(n), TOUCH_MUX_SEL | TOUCH_FUN_SEL(3) |
           TOUCH_FUN_IE | (pull_up ? TOUCH_RUE : 0));
    }
    wr(qts, RTCIO_SAR_I2C_IO, SCL_SEL(0) | SDA_SEL(0));
    wr(qts, RTC_I2C_SCL_LOW, 40);
    wr(qts, RTC_I2C_SCL_HIGH, 40);
    wr(qts, RTC_I2C_SDA_DUTY, 16);
    wr(qts, RTC_I2C_SCL_START, 30);
    wr(qts, RTC_I2C_SCL_STOP, 44);
    wr(qts, RTC_I2C_TIMEOUT, 200);
    wr(qts, RTC_I2C_CTRL, RTC_I2C_MS_MODE);
    /* SENS_I2C_SLAVE_ADDR0 = the board's TMP105, ADDR1 = nobody */
    wr(qts, SLAVE_ADDR1, (0x48 << 11) | 0x50);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * I2C_RD and I2C_WR through the RTC I2C controller to the board's TMP105
 * at 0x48 on the RTC pads: its temperature register, whole and with its
 * bit range masked; a write acknowledged; an absent slave NACKs; the
 * transaction takes its SCL periods; least significant bit first the
 * address changes; without pull-ups the controller times out; and without
 * the pads routed the device does not answer.
 */
static void test_i2c(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        /* 0 */ I_MOVI(2, DATA),
        /* 1 */ I_I2C_RD(0x00, 7, 0, 0),           /* temperature MSB */
        /* 2 */ I_ST(0, 2, 0),
        /*
         * I2C_WR's repeated START makes the TMP105 take the data byte for
         * its pointer: it acknowledges every byte but changes nothing.
         */
        /* 3 */ I_I2C_WR(0x01, 0x5a, 7, 0, 0),
        /* 4 */ I_I2C_RD(0x00, 7, 4, 0),
        /* 5 */ I_ST(0, 2, 1),
        /* 6 */ I_HALT(),
    };
    const uint32_t nack[] = {
        /* 16 */ I_MOVI(2, DATA + 2),
        /* 17 */ I_I2C_RD(0x00, 7, 0, 1),
        /* 18 */ I_ST(0, 2, 0),
        /* 19 */ I_HALT(),
    };
    uint64_t t0;

    rtc_i2c_setup(qts, true);
    load(qts, 0, prog, ARRAY_SIZE(prog));
    load(qts, 16, nack, ARRAY_SIZE(nack));
    run(qts, 0);
    qtest_clock_step(qts, 5 * MS);
    /* 21 C */
    g_assert_cmphex(mem(qts, DATA) & 0xffff, ==, 0x15);
    g_assert_cmphex(mem(qts, DATA + 1) & 0xffff, ==, 0x10);
    g_assert_cmphex(I2C_RDATA(rd(qts, SLAVE_ADDR4)), ==, 0x10);
    g_assert_cmphex(rd(qts, SLAVE_ADDR4) & I2C_DONE, ==, I2C_DONE);
    g_assert_cmphex(rd(qts, RTC_I2C_INT_RAW) &
                    (I2C_INT_MASTER_COMPLETE | I2C_INT_TRANS_COMPLETE), ==,
                    I2C_INT_MASTER_COMPLETE | I2C_INT_TRANS_COMPLETE);
    /* INT_EN and INT_CLR sit one bit above INT_RAW */
    wr(qts, RTC_I2C_INT_EN, I2C_INT_MASTER_COMPLETE << 1);
    g_assert_cmphex(rd(qts, RTC_I2C_INT_ST), ==, I2C_INT_MASTER_COMPLETE);
    wr(qts, RTC_I2C_INT_CLR, 0xff << 1);
    g_assert_cmphex(rd(qts, RTC_I2C_INT_RAW), ==, 0);

    wr(qts, RTC_SLOW_MEM + 4 * (DATA + 2), 0);
    run(qts, 16);
    qtest_clock_step(qts, 5 * MS);
    g_assert_cmphex(mem(qts, DATA + 2) & 0xffff, ==, 0xff);
    g_assert_cmphex(rd(qts, RTC_I2C_DEBUG_STATUS) & I2C_ACK_VAL, ==,
                    I2C_ACK_VAL);
    g_assert_cmphex(rd(qts, RTC_I2C_INT_RAW) & I2C_INT_MASTER_COMPLETE, ==,
                    0);

    /*
     * Software's start through SENS_SAR_I2C_CTRL: I2C_DONE comes after the
     * transaction's 30 + 4 * 9 * 80 + 40 + 30 + 44 cycles at 8 MHz.
     */
    wr(qts, SAR_I2C_CTRL, (7u << 19) | 0x00 | SAR_I2C_START_FORCE);
    t0 = qtest_clock_step(qts, 1);
    wr(qts, SAR_I2C_CTRL, (7u << 19) | 0x00 | SAR_I2C_START_FORCE |
       SAR_I2C_START);
    g_assert_cmphex(rd(qts, SLAVE_ADDR4) & I2C_DONE, ==, 0);
    g_assert_cmphex(I2C_RDATA(rd(qts, SLAVE_ADDR4)), ==, 0x15);
    qtest_clock_set(qts, t0 + (30 + 4 * 9 * 80 + 40 + 30 + 44) * FAST_NS -
                    US);
    g_assert_cmphex(rd(qts, SLAVE_ADDR4) & I2C_DONE, ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, SLAVE_ADDR4) & I2C_DONE, ==, I2C_DONE);

    /* Least significant bit first, the TMP105 sees 0x12 for 0x48 */
    wr(qts, SAR_I2C_CTRL, 0);
    wr(qts, RTC_I2C_CTRL, RTC_I2C_MS_MODE | RTC_I2C_TX_LSB_FIRST);
    wr(qts, SAR_I2C_CTRL, (7u << 19) | SAR_I2C_START_FORCE | SAR_I2C_START);
    g_assert_cmphex(I2C_RDATA(rd(qts, SLAVE_ADDR4)), ==, 0xff);
    wr(qts, RTC_I2C_CTRL, RTC_I2C_MS_MODE);
    qtest_clock_step(qts, 1 * MS);

    /* Without pull-ups SCL stays low: the controller times out */
    rtc_i2c_setup(qts, false);
    wr(qts, RTC_I2C_INT_CLR, 0xff << 1);
    wr(qts, SAR_I2C_CTRL, 0);
    wr(qts, SAR_I2C_CTRL, (7u << 19) | SAR_I2C_START_FORCE | SAR_I2C_START);
    g_assert_cmphex(I2C_RDATA(rd(qts, SLAVE_ADDR4)), ==, 0xff);
    g_assert_cmphex(rd(qts, RTC_I2C_INT_RAW) & I2C_INT_TIME_OUT, ==,
                    I2C_INT_TIME_OUT);
    g_assert_cmphex(rd(qts, RTC_I2C_DEBUG_STATUS) & I2C_TIMED_OUT, ==,
                    I2C_TIMED_OUT);
    qtest_clock_step(qts, 1 * MS);

    /* With the pads in RTC GPIO function the TMP105 is not on the lines */
    rtc_i2c_setup(qts, true);
    for (unsigned n = 0; n < 2; n++) {
        wr(qts, RTCIO_TOUCH_PAD(n), TOUCH_MUX_SEL | TOUCH_FUN_IE | TOUCH_RUE);
    }
    wr(qts, SAR_I2C_CTRL, 0);
    wr(qts, SAR_I2C_CTRL, (7u << 19) | SAR_I2C_START_FORCE | SAR_I2C_START);
    g_assert_cmphex(I2C_RDATA(rd(qts, SLAVE_ADDR4)), ==, 0xff);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.ulp/test]
 * JUMP to the word address in a register, and to an immediate one.
 */
static void test_jump_register(void)
{
    QTestState *qts = start();
    const uint32_t prog[] = {
        /* 0 */ I_MOVI(1, 4),
        /* 1 */ I_JUMP_R(1),
        /* 2 */ I_MOVI(3, 0xbad),
        /* 3 */ I_HALT(),
        /* 4 */ I_MOVI(3, 0x600d),
        /* 5 */ I_MOVI(2, DATA),
        /* 6 */ I_ST(3, 2, 0),
        /* 7 */ I_JUMP(3),
    };

    load(qts, 0, prog, ARRAY_SIZE(prog));
    run(qts, 0);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmphex(mem(qts, DATA) & 0xffff, ==, 0x600d);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32/ulp/alu-ld-st", test_alu_ld_st);
    qtest_add_func("esp32/ulp/branches-and-timing", test_branches_and_timing);
    qtest_add_func("esp32/ulp/jump-register", test_jump_register);
    qtest_add_func("esp32/ulp/timer", test_timer);
    qtest_add_func("esp32/ulp/reg-rd-wr", test_reg_rd_wr);
    qtest_add_func("esp32/ulp/adc", test_adc);
    qtest_add_func("esp32/ulp/tsens", test_tsens);
    qtest_add_func("esp32/ulp/i2c", test_i2c);
    return g_test_run();
}
