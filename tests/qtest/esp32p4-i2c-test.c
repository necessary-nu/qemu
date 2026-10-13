/*
 * QTest testcase for the ESP32-P4 I2C controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * I2C0's QEMU bus carries a TMP105 at 0x48 and an AT24C EEPROM at 0x50,
 * put there with -device as a board would wire them; the controllers
 * reach them only through pads their SCL and SDA are routed to, open
 * drain with the pads' pull-ups. I2C1 in slave mode on the same pads
 * answers I2C0 as a slave. The I2C function clocks run from the 40 MHz
 * crystal undivided: 25 ns a cycle.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define I2C0                0x500c4000
#define I2C1                0x500c5000
#define LP_I2C              0x50122000

#define SCL_LOW_PERIOD      0x00
#define CTR                 0x04
#define SR                  0x08
#define TO                  0x0c
#define SLAVE_ADDR          0x10
#define FIFO_ST             0x14
#define FIFO_CONF           0x18
#define DATA                0x1c
#define INT_RAW             0x20
#define INT_CLR             0x24
#define INT_ENA             0x28
#define INT_STATUS          0x2c
#define SDA_HOLD            0x30
#define SDA_SAMPLE          0x34
#define SCL_HIGH_PERIOD     0x38
#define SCL_START_HOLD      0x40
#define SCL_RSTART_SETUP    0x44
#define SCL_STOP_HOLD       0x48
#define SCL_STOP_SETUP      0x4c
#define FILTER_CFG          0x50
#define CLK_CONF            0x54
#define COMD(n)             (0x58 + 4 * (n))
#define SCL_ST_TIME_OUT     0x78
#define SCL_MAIN_ST_TIME_OUT 0x7c
#define SCL_SP_CONF         0x80
#define SCL_STRETCH_CONF    0x84
#define DATE                0xf8
#define TXRAM(n)            (0x100 + 4 * (n))
#define RXRAM(n)            (0x180 + 4 * (n))

#define CTR_SDA_FORCE_OUT   (1u << 0)
#define CTR_SCL_FORCE_OUT   (1u << 1)
#define CTR_RX_FULL_ACK     (1u << 3)
#define CTR_MS_MODE         (1u << 4)
#define CTR_TRANS_START     (1u << 5)
#define CTR_TX_LSB_FIRST    (1u << 6)
#define CTR_RX_LSB_FIRST    (1u << 7)
#define CTR_ARB_EN          (1u << 9)
#define CTR_FSM_RST         (1u << 10)
#define CTR_CONF_UPGATE     (1u << 11)
#define CTR_SLV_TX_AUTO     (1u << 12)
#define CTR_10BIT_RW_CHECK  (1u << 13)
#define CTR_BROADCAST       (1u << 14)

#define TO_EN               (1u << 5)
#define SLAVE_10BIT_EN      (1u << 31)

#define FIFO_NONFIFO_EN     (1u << 10)
#define FIFO_ADDR_CFG_EN    (1u << 11)
#define FIFO_RX_RST         (1u << 12)
#define FIFO_TX_RST         (1u << 13)
#define FIFO_PRT_EN         (1u << 14)

#define FILTER_SCL_EN       (1u << 8)
#define FILTER_SDA_EN       (1u << 9)

#define SP_RST_SLV_EN       (1u << 0)
#define SP_RST_SLV_NUM(n)   ((n) << 1)

#define STRETCH_EN          (1u << 10)
#define STRETCH_CLR         (1u << 11)
#define STRETCH_ACK_CTL     (1u << 12)
#define STRETCH_ACK_LVL     (1u << 13)

#define SR_RESP_REC         (1u << 0)
#define SR_SLAVE_RW         (1u << 1)
#define SR_ARB_LOST         (1u << 3)
#define SR_BUS_BUSY         (1u << 4)
#define SR_ADDRESSED        (1u << 5)
#define SR_RXFIFO_CNT(v)    (((v) >> 8) & 0x3f)
#define SR_STRETCH_CAUSE(v) (((v) >> 14) & 3)
#define SR_TXFIFO_CNT(v)    (((v) >> 18) & 0x3f)

#define INT_RXFIFO_WM       (1u << 0)
#define INT_TXFIFO_WM       (1u << 1)
#define INT_RXFIFO_OVF      (1u << 2)
#define INT_END_DETECT      (1u << 3)
#define INT_BYTE_DONE       (1u << 4)
#define INT_ARB_LOST        (1u << 5)
#define INT_MST_TXFIFO_UDF  (1u << 6)
#define INT_TRANS_COMPLETE  (1u << 7)
#define INT_TIME_OUT        (1u << 8)
#define INT_TRANS_START     (1u << 9)
#define INT_NACK            (1u << 10)
#define INT_TXFIFO_OVF      (1u << 11)
#define INT_RXFIFO_UDF      (1u << 12)
#define INT_SCL_ST_TO       (1u << 13)
#define INT_SCL_MAIN_ST_TO  (1u << 14)
#define INT_DET_START       (1u << 15)
#define INT_SLAVE_STRETCH   (1u << 16)
#define INT_GENERAL_CALL    (1u << 17)
#define INT_ADDR_UNMATCH    (1u << 18)

#define CMD_WRITE           1
#define CMD_STOP            2
#define CMD_READ            3
#define CMD_END             4
#define CMD_RSTART          6
#define CMD(op, n, chk, exp, ack) \
    ((n) | ((chk) << 8) | ((exp) << 9) | ((ack) << 10) | ((op) << 11))
#define CMD_DONE            (1u << 31)

/* GPIO matrix and IO MUX */
#define GPIO                0x500e0000
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_PIN(n)         (GPIO + 0x074 + 4 * (n))
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define PIN_PAD_DRIVER      (1u << 2)
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1000 + 0x004 + 4 * (n))
#define FUN_WPU             (1u << 8)
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((d) << 10)
#define MCU_SEL_GPIO        (1u << 12)

#define LP_GPIO             0x5012a000
#define LP_GPIO_PIN(n)      (LP_GPIO + 0x034 + 4 * (n))
#define LP_GPIO_FUNC_IN(s)  (LP_GPIO + 0x074 + 4 * (s))
#define LP_GPIO_FUNC_OUT(n) (LP_GPIO + 0x0f4 + 4 * (n))
#define LP_PIN_PAD_DRIVER   (1u << 4)
#define LP_IN_SIG_IN_SEL    (1u << 1)
#define LP_IN_SEL(p)        ((p) << 2)
#define LP_OUT_SEL(s)       ((s) << 3)
#define LP_IOMUX_PAD(n)     (0x5012b000 + 0x008 + 4 * (n))
#define LP_PAD_RUE          (1u << 3)
#define LP_PAD_MUX_SEL      (1u << 4)
#define LP_PAD_FUN_SEL(f)   ((f) << 5)
#define LP_PAD_FUN_IE       (1u << 10)

#define SIG_I2C0_SCL        68
#define SIG_I2C0_SDA        69
#define SIG_I2C1_SCL        70
#define SIG_I2C1_SDA        71
#define SIG_GPIO_OUT        256
#define LP_SIG_I2C_SCL      0
#define LP_SIG_I2C_SDA      1

/*
 * The pads the board wires the bus to, ESP_I2C (GPIO8 SCL, GPIO7 SDA, with
 * R48 and R50, 2.2 kOhm pull-ups, and the ES8311 at 0x18 on them); I2C1's,
 * wired to them where a test says so.
 */
#define PAD_SCL             8
#define PAD_SDA             7
#define PAD_SLV_SCL         10
#define PAD_SLV_SDA         9
#define LP_PAD_SCL          4
#define LP_PAD_SDA          5

/* Clocks and resets */
#define HP_SYS_CLKRST       0x500e6000
#define SOC_CLK_CTRL2       (HP_SYS_CLKRST + 0x1c)
#define PERI_CLK_CTRL10     (HP_SYS_CLKRST + 0x40)
#define HP_RST_EN1          (HP_SYS_CLKRST + 0xc4)
#define I2C0_APB_CLK_EN     (1u << 12)
#define I2C1_APB_CLK_EN     (1u << 13)
#define I2C0_CLK_EN         (1u << 1)
#define I2C1_CLK_EN         (1u << 27)
#define RST_EN_I2C1         (1u << 21)
#define RST_EN_I2C0         (1u << 22)
#define LPPERI_CLK_EN       0x50120000
#define LPPERI_CORE_CLK_SEL 0x50120004
#define CK_EN_LP_I2C        (1u << 28)

/* Interrupt matrix status: I2C0 44, I2C1 45, LP_I2C 11 */
#define INTMTX_STATUS(w)    (0x500d6200 + 4 * (w))
#define SRC_I2C0            (1u << 12)
#define SRC_I2C1            (1u << 13)
#define SRC_LP_I2C          (1u << 11)

#define TMP105_ADDR         0x48
#define EEPROM_ADDR         0x50
#define SLAVE_ADDR7         0x28

#define CYCLE_NS            25

/*
 * The timing every test uses unless it says otherwise, in cycles: a bit
 * is LOW + 1 + WAIT + 1 + HIGH + 1 = 100 cycles, 2.5 us (400 kHz).
 */
#define T_LOW               39
#define T_HIGH              29
#define T_WAIT              29
#define T_HOLD              9
#define T_SAMPLE            34
#define T_SETUP             19
#define BIT_CYCLES          ((T_LOW + 1) + (T_WAIT + 1) + (T_HIGH + 1))
#define START_CYCLES        ((T_SETUP + 1) + (T_SETUP + 1))
#define STOP_CYCLES         ((T_LOW + 1) + (T_SETUP + 1))

static QTestState *start(const char *extra)
{
    return qtest_initf("-M esp32p4,flash-size=16M "
                       "-device tmp105,id=tmp,bus=i2c0,address=0x48 "
                       "-device at24c-eeprom,bus=i2c0,address=0x50,"
                       "rom-size=256 %s", extra ? extra : "");
}

static void qom_set_int(QTestState *qts, const char *path,
                        const char *prop, int64_t value)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                           "{ 'path': %s, 'property': %s, 'value': %"
                           PRId64 " } }", path, prop, value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t off)
{
    return qtest_readl(qts, base + off);
}

static void wr(QTestState *qts, uint32_t base, uint32_t off, uint32_t v)
{
    qtest_writel(qts, base + off, v);
}

/* Clock both HP controllers and take them out of reset. */
static void hp_enable(QTestState *qts)
{
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 I2C0_APB_CLK_EN | I2C1_APB_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL10, qtest_readl(qts, PERI_CLK_CTRL10) |
                 I2C0_CLK_EN | I2C1_CLK_EN);
    qtest_writel(qts, HP_RST_EN1, qtest_readl(qts, HP_RST_EN1) &
                 ~(RST_EN_I2C0 | RST_EN_I2C1));
}

/* An open-drain pad with its input and pull-up enabled. */
static void od_pad(QTestState *qts, int pad)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE | FUN_WPU |
                 FUN_DRV(2));
    qtest_writel(qts, GPIO_PIN(pad), PIN_PAD_DRIVER);
}

/* Route a controller's SCL and SDA, both ways, to two pads. */
static void route(QTestState *qts, int scl_sig, int sda_sig, int scl_pad,
                  int sda_pad)
{
    od_pad(qts, scl_pad);
    od_pad(qts, sda_pad);
    qtest_writel(qts, GPIO_FUNC_OUT(scl_pad), scl_sig);
    qtest_writel(qts, GPIO_FUNC_OUT(sda_pad), sda_sig);
    qtest_writel(qts, GPIO_FUNC_IN(scl_sig), IN_SIG_IN_SEL | scl_pad);
    qtest_writel(qts, GPIO_FUNC_IN(sda_sig), IN_SIG_IN_SEL | sda_pad);
}

static void timing(QTestState *qts, uint32_t base)
{
    wr(qts, base, SCL_LOW_PERIOD, T_LOW);
    wr(qts, base, SCL_HIGH_PERIOD, T_HIGH | (T_WAIT << 9));
    wr(qts, base, SDA_HOLD, T_HOLD);
    wr(qts, base, SDA_SAMPLE, T_SAMPLE);
    wr(qts, base, SCL_START_HOLD, T_SETUP);
    wr(qts, base, SCL_RSTART_SETUP, T_SETUP);
    wr(qts, base, SCL_STOP_HOLD, T_SETUP);
    wr(qts, base, SCL_STOP_SETUP, T_SETUP);
}

/* I2C0 as a master on the bus's pads, its FIFOs empty. */
static void master0(QTestState *qts)
{
    hp_enable(qts);
    route(qts, SIG_I2C0_SCL, SIG_I2C0_SDA, PAD_SCL, PAD_SDA);
    timing(qts, I2C0);
    wr(qts, I2C0, CTR, CTR_MS_MODE | CTR_RX_FULL_ACK);
    wr(qts, I2C0, CTR, CTR_MS_MODE | CTR_RX_FULL_ACK | CTR_CONF_UPGATE);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) | FIFO_RX_RST |
       FIFO_TX_RST);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) &
       ~(FIFO_RX_RST | FIFO_TX_RST));
    wr(qts, I2C0, INT_CLR, 0x7ffff);
}

static void fifo_reset(QTestState *qts, uint32_t base)
{
    wr(qts, base, FIFO_CONF, rd(qts, base, FIFO_CONF) | FIFO_RX_RST |
       FIFO_TX_RST);
    wr(qts, base, FIFO_CONF, rd(qts, base, FIFO_CONF) &
       ~(FIFO_RX_RST | FIFO_TX_RST));
}

/* RSTART, a WRITE of two bytes with ACK checks, STOP */
static void commands_write2(QTestState *qts)
{
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
}

static void trans_start(QTestState *qts, uint32_t base)
{
    wr(qts, base, CTR, rd(qts, base, CTR) | CTR_TRANS_START);
}

/* Step the clock until one of @bits is raised, at most @max_ns. */
static uint64_t wait_int(QTestState *qts, uint32_t base, uint32_t bits,
                         uint64_t max_ns)
{
    uint64_t t;

    for (t = 0; t < max_ns; t += 100) {
        if (rd(qts, base, INT_RAW) & bits) {
            return t;
        }
        qtest_clock_step(qts, 100);
    }
    g_assert_cmphex(rd(qts, base, INT_RAW) & bits, !=, 0);
    return t;
}

/* Write @n bytes to the device at @addr, with a STOP; true if ACKed. */
static bool write_bytes(QTestState *qts, uint8_t addr, const uint8_t *buf,
                        int n)
{
    int i;

    fifo_reset(qts, I2C0);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, addr << 1);
    for (i = 0; i < n; i++) {
        wr(qts, I2C0, DATA, buf[i]);
    }
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, n + 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 10000000);
    return !(rd(qts, I2C0, INT_RAW) & INT_NACK);
}

/* Write @reg, then read @n bytes with a repeated START. */
static void read_reg(QTestState *qts, uint8_t addr, uint8_t reg,
                     uint8_t *buf, int n)
{
    int i;

    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, addr << 1);
    wr(qts, I2C0, DATA, reg);
    wr(qts, I2C0, DATA, (addr << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(4), CMD(CMD_READ, n - 1, 0, 0, 0));
    wr(qts, I2C0, COMD(5), CMD(CMD_READ, 1, 0, 0, 1));
    wr(qts, I2C0, COMD(6), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 10000000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_NACK, ==, 0);
    g_assert_cmpuint(SR_RXFIFO_CNT(rd(qts, I2C0, SR)), ==, n);
    for (i = 0; i < n; i++) {
        buf[i] = rd(qts, I2C0, DATA);
    }
}

static void pad_in(QTestState *qts, int pad, int level)
{
    qtest_set_irq_in(qts, "/machine/gpio", "pad-in", pad, level);
}

static void pad_release(QTestState *qts, int pad)
{
    qtest_set_irq_in(qts, "/machine/gpio", "pad-release", pad, 1);
}

static bool pad_level(QTestState *qts, int pad)
{
    return (qtest_readl(qts, GPIO_IN) >> pad) & 1;
}

static void step_cycles(QTestState *qts, uint64_t cycles)
{
    qtest_clock_step(qts, cycles * CYCLE_NS);
}

/* Every register at its reset value */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_reset_values(void)
{
    QTestState *qts = start(NULL);
    uint32_t base[] = { I2C0, I2C1 };
    int i, n;

    hp_enable(qts);
    for (i = 0; i < 2; i++) {
        uint32_t b = base[i];

        g_assert_cmphex(rd(qts, b, SCL_LOW_PERIOD), ==, 0);
        g_assert_cmphex(rd(qts, b, CTR), ==, 0x208);
        g_assert_cmphex(rd(qts, b, SR), ==, 0xc000);
        g_assert_cmphex(rd(qts, b, TO), ==, 0x10);
        g_assert_cmphex(rd(qts, b, SLAVE_ADDR), ==, 0);
        g_assert_cmphex(rd(qts, b, FIFO_ST), ==, 0);
        g_assert_cmphex(rd(qts, b, FIFO_CONF), ==, 0x408b);
        /* TX FIFO empty, below its watermark of 4 */
        g_assert_cmphex(rd(qts, b, INT_RAW), ==, INT_TXFIFO_WM);
        g_assert_cmphex(rd(qts, b, INT_ENA), ==, 0);
        g_assert_cmphex(rd(qts, b, INT_STATUS), ==, 0);
        g_assert_cmphex(rd(qts, b, SDA_HOLD), ==, 0);
        g_assert_cmphex(rd(qts, b, SDA_SAMPLE), ==, 0);
        g_assert_cmphex(rd(qts, b, SCL_HIGH_PERIOD), ==, 0);
        g_assert_cmphex(rd(qts, b, SCL_START_HOLD), ==, 8);
        g_assert_cmphex(rd(qts, b, SCL_RSTART_SETUP), ==, 8);
        g_assert_cmphex(rd(qts, b, SCL_STOP_HOLD), ==, 8);
        g_assert_cmphex(rd(qts, b, SCL_STOP_SETUP), ==, 8);
        g_assert_cmphex(rd(qts, b, FILTER_CFG), ==, 0x300);
        for (n = 0; n < 8; n++) {
            g_assert_cmphex(rd(qts, b, COMD(n)), ==, 0);
        }
        g_assert_cmphex(rd(qts, b, SCL_ST_TIME_OUT), ==, 0x10);
        g_assert_cmphex(rd(qts, b, SCL_MAIN_ST_TIME_OUT), ==, 0x10);
        g_assert_cmphex(rd(qts, b, SCL_SP_CONF), ==, 0);
        g_assert_cmphex(rd(qts, b, SCL_STRETCH_CONF), ==, 0);
        g_assert_cmphex(rd(qts, b, DATE), ==, 0x02201172);
    }

    /* The LP I2C: half the RAM, no slave registers, its clock divider */
    g_assert_cmphex(rd(qts, LP_I2C, CTR), ==, 0x208);
    g_assert_cmphex(rd(qts, LP_I2C, FIFO_CONF), ==, 0x4046);
    g_assert_cmphex(rd(qts, LP_I2C, CLK_CONF), ==, 0x200000);
    g_assert_cmphex(rd(qts, LP_I2C, DATE), ==, 0x02404120);
    g_assert_cmphex(rd(qts, LP_I2C, INT_RAW), ==, INT_TXFIFO_WM);

    /* Read-only and write-only bits */
    wr(qts, I2C0, CTR, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, CTR), ==, 0x73df);
    wr(qts, I2C0, CTR, 0x208);
    wr(qts, LP_I2C, CTR, 0xffffffff);
    g_assert_cmphex(rd(qts, LP_I2C, CTR), ==, 0x3cc);
    wr(qts, LP_I2C, CTR, 0x208);
    wr(qts, I2C0, COMD(3), 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, COMD(3)), ==, 0x80003fff);
    wr(qts, I2C0, SCL_STRETCH_CONF, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, SCL_STRETCH_CONF), ==, 0x37ff);
    wr(qts, I2C0, DATE, 0x12345678);
    g_assert_cmphex(rd(qts, I2C0, DATE), ==, 0x12345678);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_tmp105(void)
{
    QTestState *qts = start(NULL);
    uint8_t buf[2];
    int i;

    master0(qts);
    qom_set_int(qts, "/machine/peripheral/tmp", "temperature", 25000);
    read_reg(qts, TMP105_ADDR, 0, buf, 2);
    g_assert_cmphex(buf[0], ==, 25);
    g_assert_cmphex(buf[1], ==, 0);
    for (i = 0; i < 7; i++) {
        g_assert_cmphex(rd(qts, I2C0, COMD(i)) & CMD_DONE, ==, CMD_DONE);
    }
    /* The bus is busy until STOP_HOLD has passed after the STOP. */
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_BUS_BUSY, ==, SR_BUS_BUSY);
    step_cycles(qts, T_SETUP + 1);
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_BUS_BUSY, ==, 0);

    /* A new temperature reads back through the same path. */
    qom_set_int(qts, "/machine/peripheral/tmp", "temperature", -10000);
    read_reg(qts, TMP105_ADDR, 0, buf, 2);
    g_assert_cmphex(buf[0], ==, 0xf6);
    qtest_quit(qts);
}

/* An EEPROM write, then the bytes read back across the FIFO's wrap. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_eeprom(void)
{
    QTestState *qts = start(NULL);
    uint8_t data[] = { 0x10, 0xde, 0xad, 0xbe, 0xef };
    uint8_t buf[4];

    master0(qts);
    g_assert_true(write_bytes(qts, EEPROM_ADDR, data, 5));
    step_cycles(qts, 100);
    read_reg(qts, EEPROM_ADDR, 0x10, buf, 4);
    g_assert_cmphex(buf[0], ==, 0xde);
    g_assert_cmphex(buf[1], ==, 0xad);
    g_assert_cmphex(buf[2], ==, 0xbe);
    g_assert_cmphex(buf[3], ==, 0xef);
    /* The FIFO pointers moved through the RAM. */
    g_assert_cmphex(rd(qts, I2C0, FIFO_ST) & 0x3ff, ==, (4 << 5) | 4);
    qtest_quit(qts);
}

/*
 * The transfer takes the time the timing registers give it: a START,
 * nine bits a byte and a STOP, at 25 ns a cycle.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_timing(void)
{
    QTestState *qts = start(NULL);
    /* START, two bytes of 9 bits, STOP up to SDA rising */
    uint64_t cycles = START_CYCLES + 2 * 9 * BIT_CYCLES +
                      (T_LOW + 1) + (T_WAIT + 1);

    master0(qts);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    step_cycles(qts, T_SETUP + 1);
    /* SDA falls first: the START */
    g_assert_false(pad_level(qts, PAD_SDA));
    g_assert_true(pad_level(qts, PAD_SCL));
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & (INT_TRANS_START | INT_DET_START),
                    ==, INT_TRANS_START | INT_DET_START);
    step_cycles(qts, T_SETUP + 1);
    g_assert_false(pad_level(qts, PAD_SCL));
    step_cycles(qts, cycles - START_CYCLES - 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);
    step_cycles(qts, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==,
                    INT_TRANS_COMPLETE);
    g_assert_true(pad_level(qts, PAD_SDA));
    g_assert_true(pad_level(qts, PAD_SCL));

    /* Without CONF_UPGATE a new SCL_LOW_PERIOD does not apply... */
    wr(qts, I2C0, SCL_LOW_PERIOD, T_LOW + 100);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    step_cycles(qts, 100);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    trans_start(qts, I2C0);
    step_cycles(qts, cycles);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==,
                    INT_TRANS_COMPLETE);
    /* ... with it, each bit's low phase is 100 cycles longer. */
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_CONF_UPGATE);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    step_cycles(qts, 100);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    trans_start(qts, I2C0);
    step_cycles(qts, cycles + 18 * 100 + 100 - 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);
    step_cycles(qts, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==,
                    INT_TRANS_COMPLETE);
    qtest_quit(qts);
}

/*
 * The same transfer from I2C_SCLK at XTAL_CLK / (2 + 1/3), 175/3 ns a
 * cycle: its 1910 cycles take 111416 2/3 ns, every step counted at the
 * clock's exact period. The divider, set at time 0, counts from XTAL_CLK's
 * edge there, and the transfer, started at 1 ns, from the clock's edge at
 * 0: it ends at 111416 2/3 ns, and TRANS_COMPLETE comes at the first
 * nanosecond after.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_exact_timing(void)
{
    QTestState *qts = start(NULL);
    uint64_t cycles = START_CYCLES + 2 * 9 * BIT_CYCLES +
                      (T_LOW + 1) + (T_WAIT + 1);
    int64_t t0;

    master0(qts);
    /* Integer part 1 (a divider of 2), numerator 1, denominator 3. */
    qtest_writel(qts, PERI_CLK_CTRL10,
                 (qtest_readl(qts, PERI_CLK_CTRL10) & ~0x3fffffcu) |
                 (1u << 2) | (1u << 10) | (3u << 18));
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    t0 = qtest_clock_step(qts, 1);
    trans_start(qts, I2C0);
    g_assert_cmpint(qtest_clock_step(qts, cycles * 175 / 3 - t0), ==,
                    111416);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==,
                    INT_TRANS_COMPLETE);
    qtest_quit(qts);
}

/* No device at the address: NACK_INT, then a STOP the master makes. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_nack(void)
{
    QTestState *qts = start(NULL);
    uint8_t data[] = { 1, 2 };

    master0(qts);
    g_assert_false(write_bytes(qts, 0x33, data, 2));
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & (INT_NACK | INT_TRANS_COMPLETE),
                    ==, INT_NACK | INT_TRANS_COMPLETE);
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_RESP_REC, ==, SR_RESP_REC);
    /* The RSTART ran; the WRITE and STOP never completed. */
    g_assert_cmphex(rd(qts, I2C0, COMD(0)) & CMD_DONE, ==, CMD_DONE);
    g_assert_cmphex(rd(qts, I2C0, COMD(1)) & CMD_DONE, ==, 0);
    g_assert_cmphex(rd(qts, I2C0, COMD(2)) & CMD_DONE, ==, 0);
    /* The second data byte was never sent. */
    g_assert_cmpuint(SR_TXFIFO_CNT(rd(qts, I2C0, SR)), ==, 2);
    step_cycles(qts, T_SETUP + 1);
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_BUS_BUSY, ==, 0);

    /* Without ack_check_en the master carries on regardless. */
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) | FIFO_TX_RST);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) & ~FIFO_TX_RST);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, 0x33 << 1);
    wr(qts, I2C0, DATA, 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 0, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 1000000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_NACK, ==, 0);
    g_assert_cmphex(rd(qts, I2C0, COMD(2)) & CMD_DONE, ==, CMD_DONE);
    qtest_quit(qts);
}

/*
 * The bus's devices sit on the pads: with SDA's input not coming back
 * from the pad its output drives, the TMP105 does not answer.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_unrouted(void)
{
    QTestState *qts = start(NULL);
    uint8_t reg = 0;

    master0(qts);
    /* SDA's input from another pad, pulled up */
    od_pad(qts, 20);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SDA), IN_SIG_IN_SEL | 20);
    g_assert_false(write_bytes(qts, TMP105_ADDR, &reg, 1));

    /* Routed again, it does. */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SDA), IN_SIG_IN_SEL | PAD_SDA);
    step_cycles(qts, 100);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) | FIFO_TX_RST);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) & ~FIFO_TX_RST);
    g_assert_true(write_bytes(qts, TMP105_ADDR, &reg, 1));

    /*
     * On pads the board has no pull-ups on, without the pads' own SCL
     * stays low: the bus never goes idle.
     */
    step_cycles(qts, 100);
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_SCL), SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_FUNC_OUT(PAD_SDA), SIG_GPIO_OUT);
    route(qts, SIG_I2C0_SCL, SIG_I2C0_SDA, 21, 20);
    qtest_writel(qts, IO_MUX_GPIO(21), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    trans_start(qts, I2C0);
    step_cycles(qts, 10000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) &
                    (INT_TRANS_START | INT_TRANS_COMPLETE), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_BUS_BUSY, ==, SR_BUS_BUSY);
    qtest_quit(qts);
}

/*
 * A device holding SCL low stretches the clock: the master waits for it,
 * and the transfer ends that much later. Held long enough, I2C_TO times
 * out; with TIME_OUT_EN clear, SCL_ST_TO does.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_stretch_timeout(void)
{
    QTestState *qts = start(NULL);
    uint64_t cycles = START_CYCLES + 2 * 9 * BIT_CYCLES +
                      (T_LOW + 1) + (T_WAIT + 1);

    master0(qts);
    wr(qts, I2C0, TO, TO_EN | 14);
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_CONF_UPGATE);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    /* In the low phase of the fourth bit, hold SCL for 1000 cycles. */
    step_cycles(qts, START_CYCLES + 3 * BIT_CYCLES + 20);
    g_assert_false(pad_level(qts, PAD_SCL));
    pad_in(qts, PAD_SCL, 0);
    step_cycles(qts, 1000);
    pad_release(qts, PAD_SCL);
    /*
     * Unstretched, SCL would have risen at T_LOW + 1 and its high period
     * begun T_WAIT + 1 later; it begins as SCL rises instead.
     */
    step_cycles(qts, cycles - (START_CYCLES + 3 * BIT_CYCLES + 20) -
                1000 + (1000 - (T_LOW + 1 - 20) - (T_WAIT + 1)) - 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);
    step_cycles(qts, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==,
                    INT_TRANS_COMPLETE);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TIME_OUT, ==, 0);

    /* Held for 2^14 cycles: TIME_OUT, and the lines are released. */
    step_cycles(qts, 100);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    commands_write2(qts);
    trans_start(qts, I2C0);
    step_cycles(qts, START_CYCLES + 20);
    pad_in(qts, PAD_SCL, 0);
    step_cycles(qts, (1 << 14) - 20 - 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TIME_OUT, ==, 0);
    step_cycles(qts, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TIME_OUT, ==, INT_TIME_OUT);
    pad_release(qts, PAD_SCL);
    step_cycles(qts, 10);
    g_assert_true(pad_level(qts, PAD_SCL));
    g_assert_true(pad_level(qts, PAD_SDA));
    g_assert_cmphex(rd(qts, I2C0, COMD(2)) & CMD_DONE, ==, 0);

    /* TIME_OUT_EN clear: the SCL_FSM gives up after 2^(10 + 1) - 1. */
    wr(qts, I2C0, TO, 14);
    wr(qts, I2C0, SCL_ST_TIME_OUT, 10);
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_CONF_UPGATE);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) | FIFO_TX_RST);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) & ~FIFO_TX_RST);
    step_cycles(qts, 100);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    commands_write2(qts);
    trans_start(qts, I2C0);
    step_cycles(qts, START_CYCLES + 20);
    pad_in(qts, PAD_SCL, 0);
    /* The wait begins WAIT_HIGH + 1 cycles after SCL is released. */
    step_cycles(qts, (T_LOW + 1) - 20 + (T_WAIT + 1) + (1 << 11) - 2);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_SCL_ST_TO, ==, 0);
    step_cycles(qts, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_SCL_ST_TO, ==,
                    INT_SCL_ST_TO);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TIME_OUT, ==, 0);
    pad_release(qts, PAD_SCL);
    qtest_quit(qts);
}

/* Another driver holding SDA low while the master sends a 1 wins. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_arbitration(void)
{
    QTestState *qts = start(NULL);

    master0(qts);
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_ARB_EN | CTR_CONF_UPGATE);
    /* 0x7f << 1: the address byte's first bit is a 1 */
    wr(qts, I2C0, DATA, 0xfe);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    step_cycles(qts, START_CYCLES + 5);
    pad_in(qts, PAD_SDA, 0);
    step_cycles(qts, BIT_CYCLES);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_ARB_LOST, ==, INT_ARB_LOST);
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_ARB_LOST, ==, SR_ARB_LOST);
    pad_release(qts, PAD_SDA);
    step_cycles(qts, 10);
    /* The master let go of both lines. */
    g_assert_true(pad_level(qts, PAD_SCL));
    g_assert_true(pad_level(qts, PAD_SDA));
    g_assert_cmphex(rd(qts, I2C0, COMD(1)) & CMD_DONE, ==, 0);

    /* With arbitration off, the master does not notice. */
    wr(qts, I2C0, CTR, (rd(qts, I2C0, CTR) & ~CTR_ARB_EN) | CTR_CONF_UPGATE);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, 0xfe);
    trans_start(qts, I2C0);
    step_cycles(qts, START_CYCLES + 5);
    pad_in(qts, PAD_SDA, 0);
    step_cycles(qts, 9 * BIT_CYCLES);
    pad_release(qts, PAD_SDA);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 1000000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_ARB_LOST, ==, 0);
    qtest_quit(qts);
}

/* I2C1 as a slave at 0x28 on pads 10 and 9, wired to I2C0's 8 and 7. */
static void slave1(QTestState *qts, uint32_t ctr, uint32_t stretch)
{
    route(qts, SIG_I2C1_SCL, SIG_I2C1_SDA, PAD_SLV_SCL, PAD_SLV_SDA);
    timing(qts, I2C1);
    wr(qts, I2C1, SLAVE_ADDR, SLAVE_ADDR7);
    wr(qts, I2C1, SCL_STRETCH_CONF, stretch);
    wr(qts, I2C1, CTR, ctr);
    wr(qts, I2C1, CTR, ctr | CTR_CONF_UPGATE);
    wr(qts, I2C1, INT_CLR, 0x7ffff);
}

#define WIRES "-global esp32p4-gpio.pad-wires=8:10,7:9"

/* I2C0 writes to I2C1 as a slave, and reads back from it. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_slave(void)
{
    QTestState *qts = start(WIRES);
    uint8_t data[] = { 0x11, 0x22, 0x33 };
    uint8_t buf[2];

    master0(qts);
    slave1(qts, CTR_RX_FULL_ACK | CTR_SLV_TX_AUTO, 0);
    g_assert_true(write_bytes(qts, SLAVE_ADDR7, data, 3));
    g_assert_cmpuint(SR_RXFIFO_CNT(rd(qts, I2C1, SR)), ==, 3);
    g_assert_cmphex(rd(qts, I2C1, DATA), ==, 0x11);
    g_assert_cmphex(rd(qts, I2C1, DATA), ==, 0x22);
    g_assert_cmphex(rd(qts, I2C1, DATA), ==, 0x33);
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) &
                    (INT_TRANS_COMPLETE | INT_DET_START | INT_BYTE_DONE), ==,
                    INT_TRANS_COMPLETE | INT_DET_START | INT_BYTE_DONE);
    g_assert_cmphex(rd(qts, I2C1, SR) & (SR_ADDRESSED | SR_SLAVE_RW), ==,
                    SR_ADDRESSED);
    /* Reading the empty FIFO underflows. */
    rd(qts, I2C1, DATA);
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_RXFIFO_UDF, ==,
                    INT_RXFIFO_UDF);

    /* The slave sends what its TX FIFO holds. */
    wr(qts, I2C1, DATA, 0xa5);
    wr(qts, I2C1, DATA, 0x5a);
    wr(qts, I2C1, INT_CLR, 0x7ffff);
    step_cycles(qts, 100);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, (SLAVE_ADDR7 << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_READ, 1, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_READ, 1, 0, 0, 1));
    wr(qts, I2C0, COMD(4), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 1000000);
    buf[0] = rd(qts, I2C0, DATA);
    buf[1] = rd(qts, I2C0, DATA);
    g_assert_cmphex(buf[0], ==, 0xa5);
    g_assert_cmphex(buf[1], ==, 0x5a);
    /* The master's NACK on the last byte */
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_NACK, ==, INT_NACK);
    g_assert_cmphex(rd(qts, I2C1, SR) & SR_SLAVE_RW, ==, SR_SLAVE_RW);

    /* Another address: the slave does not answer, and says so. */
    wr(qts, I2C1, INT_CLR, 0x7ffff);
    step_cycles(qts, 100);
    g_assert_false(write_bytes(qts, 0x29, data, 1));
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_ADDR_UNMATCH, ==,
                    INT_ADDR_UNMATCH);
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);
    qtest_quit(qts);
}

/*
 * A slave with SCL stretching holds SCL low when addressed for a read
 * until software has filled its TX FIFO, and when its RX FIFO fills.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_slave_stretch(void)
{
    QTestState *qts = start(WIRES);
    uint8_t data[40];
    int i;

    master0(qts);
    slave1(qts, CTR_SLV_TX_AUTO, STRETCH_EN | 20);
    wr(qts, I2C0, DATA, (SLAVE_ADDR7 << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_READ, 1, 0, 0, 1));
    wr(qts, I2C0, COMD(3), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C1, INT_SLAVE_STRETCH, 100000);
    g_assert_cmpuint(SR_STRETCH_CAUSE(rd(qts, I2C1, SR)), ==, 0);
    /* The master waits as long as the slave holds SCL. */
    step_cycles(qts, 4000);
    g_assert_false(pad_level(qts, PAD_SCL));
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);
    wr(qts, I2C1, DATA, 0x3c);
    wr(qts, I2C1, SCL_STRETCH_CONF, STRETCH_EN | 20 | STRETCH_CLR);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 100000);
    g_assert_cmphex(rd(qts, I2C0, DATA), ==, 0x3c);

    /* 33 bytes: the slave stretches as the 32nd fills its FIFO. */
    for (i = 0; i < 33; i++) {
        data[i] = i;
    }
    wr(qts, I2C1, INT_CLR, 0x7ffff);
    step_cycles(qts, 100);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, SLAVE_ADDR7 << 1);
    for (i = 0; i < 31; i++) {
        wr(qts, I2C0, DATA, data[i]);
    }
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 32, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_END, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_END_DETECT, 1000000);
    wr(qts, I2C0, DATA, data[31]);
    wr(qts, I2C0, DATA, data[32]);
    wr(qts, I2C0, COMD(0), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C1, INT_SLAVE_STRETCH, 100000);
    g_assert_cmpuint(SR_STRETCH_CAUSE(rd(qts, I2C1, SR)), ==, 2);
    g_assert_cmpuint(SR_RXFIFO_CNT(rd(qts, I2C1, SR)), ==, 32);
    for (i = 0; i < 32; i++) {
        g_assert_cmphex(rd(qts, I2C1, DATA), ==, i);
    }
    wr(qts, I2C1, SCL_STRETCH_CONF, STRETCH_EN | 20 | STRETCH_CLR);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 100000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_NACK, ==, 0);
    g_assert_cmpuint(SR_RXFIFO_CNT(rd(qts, I2C1, SR)), ==, 1);
    g_assert_cmphex(rd(qts, I2C1, DATA), ==, 32);
    qtest_quit(qts);
}

/*
 * The slave's SDA filter: a pulse on SDA shorter than SDA_FILTER_THRES
 * cycles while SCL is high is no START; a longer one is.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_filter(void)
{
    QTestState *qts = start(NULL);

    hp_enable(qts);
    route(qts, SIG_I2C1_SCL, SIG_I2C1_SDA, PAD_SLV_SCL, PAD_SLV_SDA);
    timing(qts, I2C1);
    wr(qts, I2C1, FILTER_CFG, FILTER_SCL_EN | FILTER_SDA_EN | (10 << 4) | 10);
    wr(qts, I2C1, CTR, CTR_CONF_UPGATE);
    wr(qts, I2C1, INT_CLR, 0x7ffff);
    pad_in(qts, PAD_SLV_SDA, 0);
    qtest_clock_step(qts, 9 * CYCLE_NS);
    pad_release(qts, PAD_SLV_SDA);
    step_cycles(qts, 20);
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_DET_START, ==, 0);
    pad_in(qts, PAD_SLV_SDA, 0);
    step_cycles(qts, 10);
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_DET_START, ==,
                    INT_DET_START);
    g_assert_cmphex(rd(qts, I2C1, SR) & SR_BUS_BUSY, ==, SR_BUS_BUSY);
    pad_release(qts, PAD_SLV_SDA);
    step_cycles(qts, 10);
    g_assert_cmphex(rd(qts, I2C1, SR) & SR_BUS_BUSY, ==, 0);
    qtest_quit(qts);
}

/*
 * FSM reset releases the lines mid-transfer; SCL_RST_SLV_EN then clocks
 * nine pulses out on SCL and clears itself.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_fsm_rst_clear_bus(void)
{
    QTestState *qts = start(NULL);
    int i;

    master0(qts);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    step_cycles(qts, START_CYCLES + 3 * BIT_CYCLES + 5);
    g_assert_false(pad_level(qts, PAD_SCL));
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_FSM_RST);
    step_cycles(qts, 1);
    g_assert_true(pad_level(qts, PAD_SCL));
    g_assert_true(pad_level(qts, PAD_SDA));
    g_assert_cmphex(rd(qts, I2C0, CTR) & CTR_FSM_RST, ==, 0);
    step_cycles(qts, 10 * BIT_CYCLES);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TRANS_COMPLETE, ==, 0);

    wr(qts, I2C0, SCL_SP_CONF, SP_RST_SLV_EN | SP_RST_SLV_NUM(9));
    g_assert_cmphex(rd(qts, I2C0, SCL_SP_CONF), ==,
                    SP_RST_SLV_EN | SP_RST_SLV_NUM(9));
    step_cycles(qts, 10 * BIT_CYCLES);
    /* Not until CONF_UPGATE */
    g_assert_cmphex(rd(qts, I2C0, SCL_SP_CONF) & SP_RST_SLV_EN, ==,
                    SP_RST_SLV_EN);
    wr(qts, I2C0, CTR, rd(qts, I2C0, CTR) | CTR_CONF_UPGATE);
    for (i = 0; i < 9; i++) {
        step_cycles(qts, 1);
        g_assert_false(pad_level(qts, PAD_SCL));
        g_assert_true(pad_level(qts, PAD_SDA));
        step_cycles(qts, T_LOW + 1);
        g_assert_true(pad_level(qts, PAD_SCL));
        step_cycles(qts, (T_WAIT + 1) + (T_HIGH + 1) - 1);
    }
    g_assert_cmphex(rd(qts, I2C0, SCL_SP_CONF) & SP_RST_SLV_EN, ==, 0);
    step_cycles(qts, 1000);
    g_assert_true(pad_level(qts, PAD_SCL));
    qtest_quit(qts);
}

/*
 * A transfer in pieces separated by END: SCL stays low between them, and
 * the master resumes at COMD0 on TRANS_START.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_end(void)
{
    QTestState *qts = start(NULL);

    master0(qts);
    qom_set_int(qts, "/machine/peripheral/tmp", "temperature", 30000);
    wr(qts, I2C0, DATA, TMP105_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_END, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_END_DETECT, 1000000);
    step_cycles(qts, 5000);
    g_assert_false(pad_level(qts, PAD_SCL));
    g_assert_cmphex(rd(qts, I2C0, SR) & SR_BUS_BUSY, ==, SR_BUS_BUSY);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TIME_OUT, ==, 0);

    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, (TMP105_ADDR << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_READ, 1, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_END, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_END_DETECT, 1000000);
    g_assert_cmphex(rd(qts, I2C0, DATA), ==, 30);

    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, COMD(0), CMD(CMD_READ, 1, 0, 0, 1));
    wr(qts, I2C0, COMD(1), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 1000000);
    g_assert_cmphex(rd(qts, I2C0, DATA), ==, 0);
    qtest_quit(qts);
}

/* Non-FIFO mode: TX RAM written, RX RAM read, at their own addresses. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_nonfifo(void)
{
    QTestState *qts = start(NULL);

    master0(qts);
    qom_set_int(qts, "/machine/peripheral/tmp", "temperature", 20000);
    /* In FIFO mode, TX RAM's addresses only read. */
    wr(qts, I2C0, TXRAM(0), 0x77);
    g_assert_cmphex(rd(qts, I2C0, TXRAM(0)), ==, 0);
    wr(qts, I2C0, FIFO_CONF, rd(qts, I2C0, FIFO_CONF) | FIFO_NONFIFO_EN);
    wr(qts, I2C0, TXRAM(0), TMP105_ADDR << 1);
    wr(qts, I2C0, TXRAM(1), 0);
    wr(qts, I2C0, TXRAM(2), (TMP105_ADDR << 1) | 1);
    g_assert_cmphex(rd(qts, I2C0, TXRAM(2)), ==, (TMP105_ADDR << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(4), CMD(CMD_READ, 1, 0, 0, 0));
    wr(qts, I2C0, COMD(5), CMD(CMD_READ, 2, 0, 0, 1));
    wr(qts, I2C0, COMD(6), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 1000000);
    g_assert_cmphex(rd(qts, I2C0, RXRAM(0)), ==, 20);
    g_assert_cmphex(rd(qts, I2C0, RXRAM(1)), ==, 0);
    /* After the master's NACK the sensor lets go of SDA. */
    g_assert_cmphex(rd(qts, I2C0, RXRAM(2)), ==, 0xff);
    g_assert_cmphex(rd(qts, I2C0, FIFO_ST) & 0x7fff, ==,
                    (3 << 10) | (3 << 5));
    qtest_quit(qts);
}

/*
 * A master reading into a full RX FIFO loses the byte (RXFIFO_OVF) and
 * answers it with RX_FULL_ACK_LEVEL.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_rx_overflow(void)
{
    QTestState *qts = start(NULL);
    uint8_t data[17] = { 0 };
    int i;

    master0(qts);
    for (i = 0; i < 16; i++) {
        data[i + 1] = 0x80 + i;
    }
    g_assert_true(write_bytes(qts, EEPROM_ADDR, data, 17));
    step_cycles(qts, 100);
    data[0] = 0x10;
    g_assert_true(write_bytes(qts, EEPROM_ADDR, data, 17));
    step_cycles(qts, 100);
    wr(qts, I2C0, INT_CLR, 0x7ffff);
    wr(qts, I2C0, DATA, EEPROM_ADDR << 1);
    wr(qts, I2C0, DATA, 0);
    wr(qts, I2C0, DATA, (EEPROM_ADDR << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(4), CMD(CMD_READ, 33, 0, 0, 0));
    wr(qts, I2C0, COMD(5), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 10000000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_RXFIFO_OVF, ==,
                    INT_RXFIFO_OVF);
    g_assert_cmpuint(SR_RXFIFO_CNT(rd(qts, I2C0, SR)), ==, 32);
    for (i = 0; i < 32; i++) {
        g_assert_cmphex(rd(qts, I2C0, DATA), ==, 0x80 + (i & 15));
    }
    qtest_quit(qts);
}

/* The interrupt reaches the interrupt matrix as source 44. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_irq(void)
{
    QTestState *qts = start(NULL);
    uint8_t reg = 0;

    master0(qts);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(1)) & SRC_I2C0, ==, 0);
    wr(qts, I2C0, INT_ENA, INT_TRANS_COMPLETE);
    write_bytes(qts, TMP105_ADDR, &reg, 1);
    g_assert_cmphex(rd(qts, I2C0, INT_STATUS), ==, INT_TRANS_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(1)) & SRC_I2C0, ==,
                    SRC_I2C0);
    wr(qts, I2C0, INT_CLR, INT_TRANS_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(1)) & SRC_I2C0, ==, 0);
    /* The TX watermark interrupt follows the FIFO level. */
    wr(qts, I2C0, INT_ENA, INT_TXFIFO_WM);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(1)) & SRC_I2C0, ==,
                    SRC_I2C0);
    wr(qts, I2C0, DATA, 1);
    wr(qts, I2C0, DATA, 2);
    wr(qts, I2C0, DATA, 3);
    wr(qts, I2C0, DATA, 4);
    wr(qts, I2C0, INT_CLR, INT_TXFIFO_WM);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_TXFIFO_WM, ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(1)) & SRC_I2C0, ==, 0);
    qtest_quit(qts);
}

/* With its bus clock gated, the controller's registers ignore writes. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_gating(void)
{
    QTestState *qts = start(NULL);

    hp_enable(qts);
    wr(qts, I2C0, SCL_LOW_PERIOD, 0x55);
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) &
                 ~I2C0_APB_CLK_EN);
    wr(qts, I2C0, SCL_LOW_PERIOD, 0x66);
    g_assert_cmphex(rd(qts, I2C0, SCL_LOW_PERIOD), ==, 0x55);
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 I2C0_APB_CLK_EN);
    /* Its reset puts the registers back. */
    qtest_writel(qts, HP_RST_EN1, qtest_readl(qts, HP_RST_EN1) | RST_EN_I2C0);
    qtest_writel(qts, HP_RST_EN1, qtest_readl(qts, HP_RST_EN1) &
                 ~RST_EN_I2C0);
    g_assert_cmphex(rd(qts, I2C0, SCL_LOW_PERIOD), ==, 0);
    qtest_quit(qts);
}

/* A 10-bit addressed slave, written and then read. */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_slave_10bit(void)
{
    QTestState *qts = start(WIRES);
    uint16_t a10 = 0x2a5;
    uint8_t hi = 0x78 | (a10 >> 8);

    master0(qts);
    slave1(qts, CTR_SLV_TX_AUTO | CTR_10BIT_RW_CHECK, 0);
    wr(qts, I2C1, SLAVE_ADDR, SLAVE_10BIT_EN | ((a10 & 0xff) << 7) | hi);
    wr(qts, I2C1, CTR, rd(qts, I2C1, CTR) | CTR_CONF_UPGATE);
    wr(qts, I2C1, DATA, 0x99);
    wr(qts, I2C0, DATA, hi << 1);
    wr(qts, I2C0, DATA, a10 & 0xff);
    wr(qts, I2C0, DATA, 0x42);
    wr(qts, I2C0, DATA, (hi << 1) | 1);
    wr(qts, I2C0, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(1), CMD(CMD_WRITE, 3, 1, 0, 0));
    wr(qts, I2C0, COMD(2), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, I2C0, COMD(3), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, I2C0, COMD(4), CMD(CMD_READ, 1, 0, 0, 1));
    wr(qts, I2C0, COMD(5), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, I2C0);
    wait_int(qts, I2C0, INT_TRANS_COMPLETE, 1000000);
    g_assert_cmphex(rd(qts, I2C0, INT_RAW) & INT_NACK, ==, 0);
    g_assert_cmphex(rd(qts, I2C1, DATA), ==, 0x42);
    g_assert_cmphex(rd(qts, I2C0, DATA), ==, 0x99);
    qtest_quit(qts);
}

/*
 * The general call reaches a slave with ADDR_BROADCASTING_EN only. (The
 * bus carries no QEMU devices, which take address 0 as a broadcast.)
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_general_call(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M " WIRES);
    uint8_t data = 0x06;

    master0(qts);
    slave1(qts, 0, 0);
    g_assert_false(write_bytes(qts, 0, &data, 1));
    slave1(qts, CTR_BROADCAST, 0);
    step_cycles(qts, 100);
    g_assert_true(write_bytes(qts, 0, &data, 1));
    g_assert_cmphex(rd(qts, I2C1, INT_RAW) & INT_GENERAL_CALL, ==,
                    INT_GENERAL_CALL);
    g_assert_cmphex(rd(qts, I2C1, DATA), ==, 0x06);
    qtest_quit(qts);
}

/*
 * The LP I2C on LP pads 4 and 5, reading the TMP105 on its own bus, at
 * LP_I2C_SCLK through LP_I2C_CLK_CONF's divider.
 */
/* [spec:nuos:req:emu.esp32p4.i2c/test] */
static void test_lp_i2c(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M "
                                 "-device tmp105,id=lptmp,bus=lp-i2c,"
                                 "address=0x49");
    int pads[] = { LP_PAD_SCL, LP_PAD_SDA };
    int i;

    qom_set_int(qts, "/machine/peripheral/lptmp", "temperature", 40000);
    for (i = 0; i < 2; i++) {
        qtest_writel(qts, LP_IOMUX_PAD(pads[i]), LP_PAD_MUX_SEL |
                     LP_PAD_FUN_SEL(1) | LP_PAD_FUN_IE | LP_PAD_RUE | 2);
        qtest_writel(qts, LP_GPIO_PIN(pads[i]), LP_PIN_PAD_DRIVER);
        qtest_writel(qts, LP_GPIO_FUNC_OUT(pads[i]), LP_OUT_SEL(i));
        qtest_writel(qts, LP_GPIO_FUNC_IN(i), LP_IN_SIG_IN_SEL |
                     LP_IN_SEL(pads[i]));
    }
    timing(qts, LP_I2C);
    wr(qts, LP_I2C, CTR, rd(qts, LP_I2C, CTR) | CTR_CONF_UPGATE);
    wr(qts, LP_I2C, INT_CLR, 0xffff);
    wr(qts, LP_I2C, DATA, 0x49 << 1);
    wr(qts, LP_I2C, DATA, 0);
    wr(qts, LP_I2C, DATA, (0x49 << 1) | 1);
    wr(qts, LP_I2C, COMD(0), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, LP_I2C, COMD(1), CMD(CMD_WRITE, 2, 1, 0, 0));
    wr(qts, LP_I2C, COMD(2), CMD(CMD_RSTART, 0, 0, 0, 0));
    wr(qts, LP_I2C, COMD(3), CMD(CMD_WRITE, 1, 1, 0, 0));
    wr(qts, LP_I2C, COMD(4), CMD(CMD_READ, 1, 0, 0, 1));
    wr(qts, LP_I2C, COMD(5), CMD(CMD_STOP, 0, 0, 0, 0));
    trans_start(qts, LP_I2C);
    wait_int(qts, LP_I2C, INT_TRANS_COMPLETE, 10000000);
    g_assert_cmphex(rd(qts, LP_I2C, INT_RAW) & INT_NACK, ==, 0);
    g_assert_cmphex(rd(qts, LP_I2C, DATA), ==, 40);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(0)) & SRC_LP_I2C, ==, 0);
    wr(qts, LP_I2C, INT_ENA, INT_TRANS_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS(0)) & SRC_LP_I2C, ==,
                    SRC_LP_I2C);
    /* Stopping its function clock stops the controller. */
    step_cycles(qts, T_SETUP + 1);
    wr(qts, LP_I2C, INT_CLR, 0xffff);
    wr(qts, LP_I2C, CLK_CONF, 0);
    trans_start(qts, LP_I2C);
    step_cycles(qts, 100000);
    g_assert_cmphex(rd(qts, LP_I2C, INT_RAW) & INT_TRANS_START, ==, 0);
    wr(qts, LP_I2C, CLK_CONF, 0x200000);
    wait_int(qts, LP_I2C, INT_TRANS_COMPLETE, 10000000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/i2c/reset-values", test_reset_values);
    qtest_add_func("esp32p4/i2c/tmp105", test_tmp105);
    qtest_add_func("esp32p4/i2c/eeprom", test_eeprom);
    qtest_add_func("esp32p4/i2c/timing", test_timing);
    qtest_add_func("esp32p4/i2c/exact-timing", test_exact_timing);
    qtest_add_func("esp32p4/i2c/nack", test_nack);
    qtest_add_func("esp32p4/i2c/unrouted", test_unrouted);
    qtest_add_func("esp32p4/i2c/stretch-timeout", test_stretch_timeout);
    qtest_add_func("esp32p4/i2c/arbitration", test_arbitration);
    qtest_add_func("esp32p4/i2c/slave", test_slave);
    qtest_add_func("esp32p4/i2c/slave-stretch", test_slave_stretch);
    qtest_add_func("esp32p4/i2c/slave-10bit", test_slave_10bit);
    qtest_add_func("esp32p4/i2c/general-call", test_general_call);
    qtest_add_func("esp32p4/i2c/filter", test_filter);
    qtest_add_func("esp32p4/i2c/fsm-rst-clear-bus", test_fsm_rst_clear_bus);
    qtest_add_func("esp32p4/i2c/end", test_end);
    qtest_add_func("esp32p4/i2c/nonfifo", test_nonfifo);
    qtest_add_func("esp32p4/i2c/rx-overflow", test_rx_overflow);
    qtest_add_func("esp32p4/i2c/irq", test_irq);
    qtest_add_func("esp32p4/i2c/gating", test_gating);
    qtest_add_func("esp32p4/i2c/lp", test_lp_i2c);
    return g_test_run();
}
