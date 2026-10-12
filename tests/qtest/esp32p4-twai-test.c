/*
 * QTest testcase for the ESP32-P4 TWAI controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The controllers run from the 40 MHz crystal. Most tests put TWAI0 and
 * TWAI1 on one QEMU CAN bus at 1 Mbit/s (a 50 ns quantum, 20 quanta a
 * bit), their twai_tx and twai_rx on pads through the GPIO matrix. Frame
 * times are checked to the bit: frame_bits() codes a frame as CAN 2.0B
 * does, CRC-15 then bit stuffing, independently of the model.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define TWAI(n)             (0x500d7000u + 0x1000u * (n))
#define MODE                0x00
#define CMD                 0x04
#define STATUS              0x08
#define INT                 0x0c
#define INT_ENA             0x10
#define BTR0                0x18
#define BTR1                0x1c
#define ALC                 0x2c
#define ECC                 0x30
#define EWL                 0x34
#define RXERR               0x38
#define TXERR               0x3c
#define DATA(i)             (0x40 + 4 * (i))
#define RMC                 0x74
#define CDR                 0x7c
#define SW_STANDBY          0x80
#define HW_CFG              0x84
#define HW_STANDBY_CNT      0x88
#define IDLE_INTR_CNT       0x8c
#define ECO_CFG             0x90
#define TS_DATA             0x94
#define TS_PRESCALER        0x98
#define TS_CFG              0x9c

#define MODE_RM             0x01
#define MODE_LOM            0x02
#define MODE_STM            0x04
#define MODE_AFM            0x08
#define CMD_TR              0x01
#define CMD_AT              0x02
#define CMD_RRB             0x04
#define CMD_CDO             0x08
#define CMD_SRR             0x10
#define ST_RBS              0x001
#define ST_DOS              0x002
#define ST_TBS              0x004
#define ST_TCS              0x008
#define ST_RS               0x010
#define ST_TS               0x020
#define ST_ES               0x040
#define ST_BS               0x080
#define ST_MS               0x100
#define INT_RI              0x01
#define INT_TI              0x02
#define INT_EI              0x04
#define INT_DOI             0x08
#define INT_TSI             0x10
#define INT_EPI             0x20
#define INT_ALI             0x40
#define INT_BEI             0x80

/* ERR_CODE_CAP values */
#define ECC_ACK_ERR_TX      0xd9    /* other, transmitting, ACK slot */
#define ECC_FORM_ACK_DELIM  0x7b    /* form, receiving, ACK delimiter */
#define ECC_BIT_SOF_TX      0x03    /* bit, transmitting, SOF */

/* 1 Mbit/s: BRP 0, SJW 4, TSEG1 15, TSEG2 4. */
#define BTR0_1M             0xc000
#define BTR1_1M             0x3e
#define BIT_NS              1000

#define CLKRST              0x500e6000u
#define SOC_CLK_CTRL2       (CLKRST + 0x1c)
#define PERI_CLK_CTRL115    (CLKRST + 0x7c)
#define HP_RST_EN1          (CLKRST + 0xc4)
#define TWAI_APB_CLK_EN(n)  (1u << (24 + (n)))
#define TWAI_CLK_EN(n)      (1u << (25 + 2 * (n)))
#define TWAI_SRC_RC_FAST(n) (1u << (24 + 2 * (n)))
#define FOSC_CNTL           0x5011101cu
#define RST_EN_CAN(n)       (1u << (26 + (n)))

#define GPIO                0x500e0000u
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004u + 4 * (n))
#define MCU_SEL_GPIO        (1u << 12)
#define FUN_IE              (1u << 9)
#define FUN_DRV2            (2u << 10)
#define SIG_TX(n)           (80 + 3 * (n))
#define SIG_RX(n)           (80 + 3 * (n))

#define INTMTX_STATUS1      0x500d6204u
#define SRC_TWAI_BIT(n)     (1u << (8 + (n)))

#define TWAI_PATH           "/machine/twai"

static QTestState *qts;
static int64_t now;

static uint32_t rd(int n, uint32_t reg)
{
    return qtest_readl(qts, TWAI(n) + reg);
}

static void wr(int n, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, TWAI(n) + reg, v);
}

static void clock_to(int64_t t)
{
    g_assert_cmpint(t, >=, now);
    if (t > now) {
        now = qtest_clock_set(qts, t);
    }
}

static void step(int64_t ns)
{
    clock_to(now + ns);
}

static void start(const char *extra)
{
    qts = qtest_initf("-M esp32p4,flash-size=16M%s", extra);
    /* Under qtest the virtual clock starts at 0 and moves only on request. */
    now = 0;
}

/* A QEMU CAN bus with TWAI0 and TWAI1 on it. */
static void start_bus(const char *extra)
{
    g_autofree char *args =
        g_strdup_printf("%s,canbus0=can0,canbus1=can0 "
                        "-object can-bus,id=can0", extra);

    start(args);
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

static void clocks_on(void)
{
    qtest_writel(qts, SOC_CLK_CTRL2, qtest_readl(qts, SOC_CLK_CTRL2) |
                 TWAI_APB_CLK_EN(0) | TWAI_APB_CLK_EN(1) | TWAI_APB_CLK_EN(2));
    qtest_writel(qts, PERI_CLK_CTRL115,
                 TWAI_CLK_EN(0) | TWAI_CLK_EN(1) | TWAI_CLK_EN(2));
}

static void pad_gpio(int pad)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE | FUN_DRV2);
}

static void route(int n, int tx, int rx)
{
    pad_gpio(tx);
    qtest_writel(qts, GPIO_FUNC_OUT(tx), SIG_TX(n));
    pad_gpio(rx);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RX(n)), IN_SIG_IN_SEL | rx);
}

/* Configure in reset mode, accepting everything, and leave it. */
static void setup(int n, uint32_t btr0, uint32_t btr1, uint32_t mode)
{
    wr(n, MODE, MODE_RM);
    wr(n, BTR0, btr0);
    wr(n, BTR1, btr1);
    for (int i = 0; i < 4; i++) {
        wr(n, DATA(i), 0);
        wr(n, DATA(4 + i), 0xff);
    }
    wr(n, MODE, mode);
}

/*
 * TWAI0 and TWAI1 on one bus at 1 Mbit/s, past their integration, with
 * @extra machine options.
 */
static void two_nodes_with(const char *extra)
{
    start_bus(extra);
    clocks_on();
    route(0, 20, 21);
    route(1, 22, 23);
    setup(0, BTR0_1M, BTR1_1M, 0);
    setup(1, BTR0_1M, BTR1_1M, 0);
    step(20 * BIT_NS);
}

static void two_nodes(void)
{
    two_nodes_with("");
}

/* Frames, in the transmit buffer's layout */

static int sff(uint8_t *f, uint32_t id, bool rtr, int dlc, const uint8_t *d)
{
    memset(f, 0, 13);
    f[0] = (rtr ? 0x40 : 0) | dlc;
    f[1] = id >> 3;
    f[2] = id << 5;
    if (!rtr) {
        memcpy(f + 3, d, MIN(dlc, 8));
    }
    return 3 + (rtr ? 0 : MIN(dlc, 8));
}

static int eff(uint8_t *f, uint32_t id, bool rtr, int dlc, const uint8_t *d)
{
    memset(f, 0, 13);
    f[0] = 0x80 | (rtr ? 0x40 : 0) | dlc;
    f[1] = id >> 21;
    f[2] = id >> 13;
    f[3] = id >> 5;
    f[4] = id << 3;
    if (!rtr) {
        memcpy(f + 5, d, MIN(dlc, 8));
    }
    return 5 + (rtr ? 0 : MIN(dlc, 8));
}

/*
 * The frame's bits on the bus, SOF to the end of EOF; with @u >= 0, the
 * bus bit that unstuffed bit @u is sent in instead.
 */
static int frame_code(const uint8_t *f, int u)
{
    bool e = f[0] & 0x80, rtr = f[0] & 0x40;
    int dlc = f[0] & 15, nd = rtr ? 0 : MIN(dlc, 8);
    uint32_t id = e ? (f[1] << 21 | f[2] << 13 | f[3] << 5 | f[4] >> 3)
                    : (f[1] << 3 | f[2] >> 5);
    const uint8_t *data = f + (e ? 5 : 3);
    uint8_t raw[160];
    int n = 0, i, run = 0, last = -1, len = 0;
    uint16_t crc = 0;

    raw[n++] = 0;
    for (i = 10; i >= 0; i--) {
        raw[n++] = (id >> (e ? 18 + i : i)) & 1;
    }
    if (e) {
        raw[n++] = 1;
        raw[n++] = 1;
        for (i = 17; i >= 0; i--) {
            raw[n++] = (id >> i) & 1;
        }
        raw[n++] = rtr;
        raw[n++] = 0;
        raw[n++] = 0;
    } else {
        raw[n++] = rtr;
        raw[n++] = 0;
        raw[n++] = 0;
    }
    for (i = 3; i >= 0; i--) {
        raw[n++] = (dlc >> i) & 1;
    }
    for (i = 0; i < 8 * nd; i++) {
        raw[n++] = (data[i / 8] >> (7 - i % 8)) & 1;
    }
    for (i = 0; i < n; i++) {
        bool next = raw[i] ^ ((crc >> 14) & 1);

        crc = (crc << 1) & 0x7fff;
        if (next) {
            crc ^= 0x4599;
        }
    }
    for (i = 14; i >= 0; i--) {
        raw[n++] = (crc >> i) & 1;
    }
    for (i = 0; i < n; i++) {
        if (i == u) {
            return len;
        }
        len++;
        run = raw[i] == last ? run + 1 : 1;
        last = raw[i];
        if (run == 5) {
            len++;
            last = !last;
            run = 1;
        }
    }
    /* CRC delimiter, ACK slot and delimiter, EOF */
    return len + 10;
}

static int frame_bits(const uint8_t *f)
{
    return frame_code(f, -1);
}

/* The ACK slot's bit. */
static int ack_slot(const uint8_t *f)
{
    return frame_bits(f) - 9;
}

static void load(int n, const uint8_t *f)
{
    for (int i = 0; i < 13; i++) {
        wr(n, DATA(i), f[i]);
    }
}

/*
 * The SOF of a frame requested now on an idle bus: the first boundary of
 * the 1 Mbit/s grid after now.
 */
static int64_t next_sof(void)
{
    return (now / BIT_NS + 1) * BIT_NS;
}

static void expect_rx(int n, const uint8_t *f, int len)
{
    for (int i = 0; i < len; i++) {
        g_assert_cmphex(rd(n, DATA(i)), ==, f[i]);
    }
}

/* Registers at reset, and what the APB clock gate does to them */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_reset_values(void)
{
    start("");
    /* TWAIn_APB_CLK is gated out of reset: writes are dropped. */
    wr(0, EWL, 7);
    g_assert_cmphex(rd(0, EWL), ==, 96);
    clocks_on();
    for (int n = 0; n < 3; n++) {
        g_assert_cmphex(rd(n, MODE), ==, MODE_RM);
        g_assert_cmphex(rd(n, CMD), ==, 0);
        g_assert_cmphex(rd(n, STATUS), ==, ST_TBS | ST_TCS);
        g_assert_cmphex(rd(n, INT), ==, 0);
        g_assert_cmphex(rd(n, INT_ENA), ==, 0);
        g_assert_cmphex(rd(n, BTR0), ==, 0);
        g_assert_cmphex(rd(n, BTR1), ==, 0);
        g_assert_cmphex(rd(n, ALC), ==, 0);
        g_assert_cmphex(rd(n, ECC), ==, 0);
        g_assert_cmphex(rd(n, EWL), ==, 96);
        g_assert_cmphex(rd(n, RXERR), ==, 0);
        g_assert_cmphex(rd(n, TXERR), ==, 0);
        for (int i = 0; i < 13; i++) {
            g_assert_cmphex(rd(n, DATA(i)), ==, 0);
        }
        g_assert_cmphex(rd(n, RMC), ==, 0);
        g_assert_cmphex(rd(n, CDR), ==, 0);
        g_assert_cmphex(rd(n, SW_STANDBY), ==, 2);
        g_assert_cmphex(rd(n, HW_CFG), ==, 0);
        g_assert_cmphex(rd(n, HW_STANDBY_CNT), ==, 1);
        g_assert_cmphex(rd(n, IDLE_INTR_CNT), ==, 1);
        g_assert_cmphex(rd(n, ECO_CFG), ==, 2);
        g_assert_cmphex(rd(n, TS_DATA), ==, 0);
        g_assert_cmphex(rd(n, TS_PRESCALER), ==, 31);
        g_assert_cmphex(rd(n, TS_CFG), ==, 0);
    }
    stop();
}

/* Which registers take writes in reset mode, operation mode, or never */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_register_access(void)
{
    start("");
    clocks_on();

    wr(0, INT_ENA, 0x1ff);
    g_assert_cmphex(rd(0, INT_ENA), ==, 0xff);
    wr(0, INT_ENA, 0);
    wr(0, BTR0, 0xffffffff);
    g_assert_cmphex(rd(0, BTR0), ==, 0xffff);
    wr(0, BTR1, 0xffffffff);
    g_assert_cmphex(rd(0, BTR1), ==, 0xff);
    wr(0, EWL, 0x1ff);
    g_assert_cmphex(rd(0, EWL), ==, 0xff);
    wr(0, RXERR, 5);
    wr(0, TXERR, 7);
    g_assert_cmphex(rd(0, RXERR), ==, 5);
    g_assert_cmphex(rd(0, TXERR), ==, 7);
    for (int i = 0; i < 8; i++) {
        wr(0, DATA(i), 0x11 * (i + 1));
    }
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(rd(0, DATA(i)), ==, 0x11 * (i + 1));
    }
    /* DATA_8 to _12 are read-only in reset mode. */
    wr(0, DATA(8), 0x55);
    g_assert_cmphex(rd(0, DATA(8)), ==, 0);
    wr(0, CDR, 0x1ff);
    g_assert_cmphex(rd(0, CDR), ==, 0x1ff);
    wr(0, MODE, MODE_RM | MODE_LOM | MODE_STM | MODE_AFM);
    g_assert_cmphex(rd(0, MODE), ==, 0x0f);
    wr(0, EWL, 96);
    wr(0, RXERR, 0);
    wr(0, TXERR, 0);

    /* Leaving reset mode keeps the mode bits written with it. */
    wr(0, MODE, MODE_STM | MODE_AFM);
    g_assert_cmphex(rd(0, MODE), ==, MODE_STM | MODE_AFM);
    wr(0, MODE, MODE_LOM);
    g_assert_cmphex(rd(0, MODE), ==, MODE_STM | MODE_AFM);
    wr(0, BTR0, 0);
    wr(0, BTR1, 0);
    wr(0, EWL, 3);
    wr(0, TXERR, 9);
    wr(0, RXERR, 9);
    g_assert_cmphex(rd(0, BTR0), ==, 0xffff);
    g_assert_cmphex(rd(0, BTR1), ==, 0xff);
    g_assert_cmphex(rd(0, EWL), ==, 96);
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    g_assert_cmphex(rd(0, RXERR), ==, 0);
    /* CD changes in operation mode, CLOCK_OFF does not. */
    wr(0, CDR, 0x005);
    g_assert_cmphex(rd(0, CDR), ==, 0x105);
    /* The data registers are the receive buffer to read. */
    g_assert_cmphex(rd(0, DATA(0)), ==, 0);
    wr(0, MODE, MODE_RM);
    g_assert_cmphex(rd(0, MODE), ==, MODE_RM | MODE_STM | MODE_AFM);
    g_assert_cmphex(rd(0, DATA(0)), ==, 0x11);

    /* Reserved addresses */
    g_assert_cmphex(rd(0, 0x14), ==, 0);
    g_assert_cmphex(rd(0, 0xa0), ==, 0);
    g_assert_cmphex(rd(0, 0xffc), ==, 0);

    wr(0, SW_STANDBY, 0xffffffff);
    g_assert_cmphex(rd(0, SW_STANDBY), ==, 3);
    wr(0, HW_CFG, 0xffffffff);
    g_assert_cmphex(rd(0, HW_CFG), ==, 1);
    wr(0, HW_STANDBY_CNT, 0x12345678);
    g_assert_cmphex(rd(0, HW_STANDBY_CNT), ==, 0x12345678);
    wr(0, IDLE_INTR_CNT, 0x9abcdef0);
    g_assert_cmphex(rd(0, IDLE_INTR_CNT), ==, 0x9abcdef0);
    wr(0, ECO_CFG, 0xffffffff);
    g_assert_cmphex(rd(0, ECO_CFG), ==, 3);
    wr(0, TS_PRESCALER, 0xffffffff);
    g_assert_cmphex(rd(0, TS_PRESCALER), ==, 0xffff);
    wr(0, TS_CFG, 0xffffffff);
    g_assert_cmphex(rd(0, TS_CFG), ==, 1);
    /* Read-only registers */
    wr(0, STATUS, 0xffffffff);
    wr(0, RMC, 0xffffffff);
    g_assert_cmphex(rd(0, STATUS), ==, ST_TBS | ST_TCS);
    g_assert_cmphex(rd(0, RMC), ==, 0);
    stop();
}

/*
 * Send @f from @tx, received by @rx: the receiver has it at the last but
 * one bit of EOF, the transmitter completes at the end of EOF.
 */
static void exchange(int tx, int rx, const uint8_t *f, int rx_len)
{
    uint8_t msg[13];
    int64_t sof;
    int bits = frame_bits(f);

    load(tx, f);
    wr(tx, CMD, CMD_TR);
    g_assert_cmphex(rd(tx, STATUS) & (ST_TBS | ST_TCS), ==, 0);
    sof = next_sof();
    clock_to(sof + (bits - 1) * BIT_NS - 1);
    g_assert_cmphex(rd(rx, RMC), ==, 0);
    g_assert_cmphex(rd(rx, STATUS) & ST_RS, ==, ST_RS);
    g_assert_cmphex(rd(tx, STATUS) & ST_TS, ==, ST_TS);
    step(1);
    g_assert_cmphex(rd(rx, RMC), ==, 1);
    g_assert_cmphex(rd(rx, STATUS) & ST_RBS, ==, ST_RBS);
    g_assert_cmphex(rd(tx, STATUS) & ST_TBS, ==, 0);
    clock_to(sof + bits * BIT_NS);
    g_assert_cmphex(rd(tx, STATUS) & (ST_TBS | ST_TCS | ST_TS), ==,
                    ST_TBS | ST_TCS);

    /* The receive buffer: RTR after the identifier, bits 5 and 4 clear. */
    memcpy(msg, f, 13);
    msg[0] &= 0xcf;
    if (f[0] & 0x80) {
        msg[4] = (f[4] & 0xf8) | (f[0] & 0x40 ? 0x04 : 0);
    } else {
        msg[2] = (f[2] & 0xe0) | (f[0] & 0x40 ? 0x10 : 0);
    }
    expect_rx(rx, msg, rx_len);
    wr(rx, CMD, CMD_RRB);
    g_assert_cmphex(rd(rx, RMC), ==, 0);
    g_assert_cmphex(rd(rx, STATUS) & ST_RBS, ==, 0);
    step(3 * BIT_NS);
}

/* Standard, extended and remote frames between TWAI0 and TWAI1 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_frames(void)
{
    static const uint8_t d[8] = { 0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03,
                                  0xff };
    uint8_t f[13];
    int len;

    two_nodes();
    len = sff(f, 0x123, false, 8, d);
    exchange(0, 1, f, len);
    len = eff(f, 0x1abcdef0 & 0x1fffffff, false, 3, d);
    exchange(0, 1, f, len);
    len = sff(f, 0x7ff, true, 5, NULL);
    exchange(1, 0, f, len);
    len = eff(f, 0x00000001, true, 8, NULL);
    exchange(1, 0, f, len);
    len = sff(f, 0x000, false, 0, NULL);
    exchange(0, 1, f, len);
    /* A DLC over 8 carries 8 bytes and is received as sent. */
    len = sff(f, 0x555, false, 15, d);
    g_assert_cmpint(len, ==, 11);
    exchange(0, 1, f, len);

    /* Successes leave the counters at 0. */
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    g_assert_cmphex(rd(1, RXERR), ==, 0);
    stop();
}

/* RI, TI and the interrupt matrix's view of the controllers' interrupts */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_interrupts(void)
{
    static const uint8_t d[2] = { 1, 2 };
    uint8_t f[13];
    int bits;
    int64_t sof;

    two_nodes();
    wr(0, INT_ENA, INT_TI);
    wr(1, INT_ENA, INT_RI);
    sff(f, 0x42, false, 2, d);
    bits = frame_bits(f);
    load(0, f);
    wr(0, CMD, CMD_TR);
    sof = next_sof();
    clock_to(sof + bits * BIT_NS);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS1) &
                    (SRC_TWAI_BIT(0) | SRC_TWAI_BIT(1)), ==,
                    SRC_TWAI_BIT(0) | SRC_TWAI_BIT(1));
    /* Reading clears TI; RI stays while a message is pending. */
    g_assert_cmphex(rd(0, INT), ==, INT_TI);
    g_assert_cmphex(rd(0, INT), ==, 0);
    g_assert_cmphex(rd(1, INT), ==, INT_RI);
    g_assert_cmphex(rd(1, INT), ==, INT_RI);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS1) &
                    (SRC_TWAI_BIT(0) | SRC_TWAI_BIT(1)), ==,
                    SRC_TWAI_BIT(1));
    wr(1, CMD, CMD_RRB);
    g_assert_cmphex(rd(1, INT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS1) & SRC_TWAI_BIT(1), ==,
                    0);
    /* An event while its enable is clear latches nothing. */
    wr(1, INT_ENA, 0);
    load(0, f);
    wr(0, INT_ENA, 0);
    wr(0, CMD, CMD_TR);
    step(200 * BIT_NS);
    g_assert_cmphex(rd(0, INT), ==, 0);
    g_assert_cmphex(rd(1, INT), ==, 0);
    /* Enabling RI with a message pending raises it at once. */
    wr(1, INT_ENA, INT_RI);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS1) & SRC_TWAI_BIT(1), ==,
                    SRC_TWAI_BIT(1));
    stop();
}

static void filter(int n, bool single, uint32_t acr, uint32_t amr)
{
    wr(n, MODE, MODE_RM);
    for (int i = 0; i < 4; i++) {
        wr(n, DATA(i), acr >> (24 - 8 * i));
        wr(n, DATA(4 + i), amr >> (24 - 8 * i));
    }
    wr(n, MODE, single ? MODE_AFM : 0);
    step(20 * BIT_NS);
}

/* Send @f from TWAI0 and return whether TWAI1 kept it. */
static bool passes(const uint8_t *f)
{
    bool got;

    load(0, f);
    wr(0, CMD, CMD_TR);
    step((frame_bits(f) + 4) * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, ST_TCS);
    got = rd(1, RMC);
    if (got) {
        wr(1, CMD, CMD_RRB);
    }
    return got;
}

/* The single and dual acceptance filters, standard and extended frames */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_filters(void)
{
    static const uint8_t aa[2] = { 0xaa, 0x55 }, ab[2] = { 0xab, 0x55 };
    static const uint8_t d5a[1] = { 0x5a }, d5b[1] = { 0x5b };
    uint8_t f[13];

    two_nodes();

    /* Single, SFF: ID 0x123, RTR 0, data byte 1 0xAA. */
    filter(1, true, 0x2460aa00, 0x000f00ff);
    sff(f, 0x123, false, 2, aa);
    g_assert_true(passes(f));
    sff(f, 0x123, false, 2, ab);
    g_assert_false(passes(f));
    sff(f, 0x124, false, 2, aa);
    g_assert_false(passes(f));
    sff(f, 0x123, true, 2, NULL);
    g_assert_false(passes(f));
    /* Data bytes the frame does not carry are not compared. */
    sff(f, 0x123, false, 0, NULL);
    g_assert_true(passes(f));
    /* Acknowledged though not kept. */
    g_assert_cmphex(rd(0, TXERR), ==, 0);

    /* Single, EFF: ID 0x1234567 with either RTR. */
    filter(1, true, 0x1234567 << 3, 0x00000007);
    eff(f, 0x1234567, false, 1, aa);
    g_assert_true(passes(f));
    eff(f, 0x1234567, true, 1, NULL);
    g_assert_true(passes(f));
    eff(f, 0x1234566, false, 1, aa);
    g_assert_false(passes(f));
    sff(f, 0x123, false, 1, aa);
    g_assert_false(passes(f));

    /*
     * Dual, SFF: filter 1 ID 0x100 with data byte 1 0x5A, its nibbles in
     * ACR1 and ACR3; filter 2 ID 0x200, to which ACR3's low nibble does
     * not belong.
     */
    filter(1, false, 0x2005400a, 0x00000000);
    sff(f, 0x100, false, 1, d5a);
    g_assert_true(passes(f));
    sff(f, 0x100, false, 1, d5b);
    g_assert_false(passes(f));
    sff(f, 0x100, false, 0, NULL);
    g_assert_true(passes(f));
    sff(f, 0x200, false, 1, d5b);
    g_assert_true(passes(f));
    sff(f, 0x201, false, 1, d5b);
    g_assert_false(passes(f));
    sff(f, 0x200, true, 1, NULL);
    g_assert_false(passes(f));

    /* Dual, EFF: the first 16 identifier bits of either filter. */
    filter(1, false, ((0x1234567 >> 13) << 16) | (0x0abcdef >> 13),
           0x00000000);
    eff(f, 0x1234567, false, 0, NULL);
    g_assert_true(passes(f));
    eff(f, (0x1234567 & ~0x1fffu) | 0x1fff, false, 0, NULL);
    g_assert_true(passes(f));
    eff(f, 0x0abcdef, true, 0, NULL);
    g_assert_true(passes(f));
    eff(f, 0x1234567 ^ (1 << 13), false, 0, NULL);
    g_assert_false(passes(f));
    stop();
}

/*
 * A transmitter nobody acknowledges: an ACK error per attempt, 8 on the
 * TEC while error active, warning at 96 and error passive at 128, then
 * no more; an acknowledgement then succeeds.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_ack_error(void)
{
    static const uint8_t d[1] = { 0x77 };
    uint8_t f[13];
    int bits, a, i;
    int64_t sof;

    two_nodes();
    wr(1, MODE, MODE_RM);
    wr(0, INT_ENA, INT_EI | INT_EPI | INT_BEI | INT_TI);
    sff(f, 0x321, false, 1, d);
    bits = frame_bits(f);
    a = ack_slot(f);
    load(0, f);
    wr(0, CMD, CMD_TR);
    sof = next_sof();
    for (i = 1; i <= 16; i++) {
        clock_to(sof + (a + 1) * BIT_NS - 1);
        g_assert_cmphex(rd(0, TXERR), ==, 8 * (i - 1));
        step(1);
        g_assert_cmphex(rd(0, TXERR), ==, 8 * i);
        g_assert_cmphex(rd(0, ECC), ==, ECC_ACK_ERR_TX);
        g_assert_cmphex(rd(0, STATUS) & ST_ES, ==, 8 * i >= 96 ? ST_ES : 0);
        g_assert_cmphex(rd(0, INT), ==,
                        INT_BEI | (8 * i == 96 ? INT_EI : 0) |
                        (8 * i == 128 ? INT_EPI : 0));
        /* Active flag, delimiter and intermission: the next SOF. */
        sof += (a + 18) * BIT_NS;
    }
    /* Error passive as the frame ends: Suspend Transmission. */
    sof += 8 * BIT_NS;
    /* Error passive: the passive flag finds no dominant bit, TEC stays. */
    for (i = 0; i < 3; i++) {
        clock_to(sof + (a + 1) * BIT_NS);
        g_assert_cmphex(rd(0, TXERR), ==, 128);
        g_assert_cmphex(rd(0, INT), ==, INT_BEI);
        /* Passive: Suspend Transmission adds 8 bits. */
        sof += (a + 26) * BIT_NS;
    }
    g_assert_cmphex(rd(0, STATUS) & ST_TBS, ==, 0);

    /* TWAI1 back: acknowledged, TEC 127, error active again. */
    wr(1, MODE, 0);
    clock_to(sof + bits * BIT_NS + 20 * BIT_NS);
    for (i = 0; i < 4 && !(rd(0, STATUS) & ST_TCS); i++) {
        step(200 * BIT_NS);
    }
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS), ==, ST_TBS | ST_TCS);
    g_assert_cmphex(rd(0, TXERR), ==, 127);
    g_assert_cmphex(rd(0, INT) & (INT_TI | INT_EPI), ==, INT_TI | INT_EPI);
    g_assert_cmphex(rd(1, RMC), >=, 1);
    stop();
}

/* A single-shot transmission is not retried */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_single_shot(void)
{
    uint8_t f[13];
    int a;
    int64_t sof;

    two_nodes();
    wr(1, MODE, MODE_RM);
    wr(0, INT_ENA, INT_TI);
    sff(f, 0x10, false, 0, NULL);
    a = ack_slot(f);
    load(0, f);
    wr(0, CMD, CMD_TR | CMD_AT);
    sof = next_sof();
    clock_to(sof + (a + 18) * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS), ==, ST_TBS);
    g_assert_cmphex(rd(0, INT), ==, INT_TI);
    g_assert_cmphex(rd(0, TXERR), ==, 8);
    step(1000 * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 8);

    /* AT alone cancels a transmission still waiting for the bus. */
    wr(0, MODE, MODE_RM);
    wr(0, MODE, 0);
    load(0, f);
    wr(0, CMD, CMD_TR);
    wr(0, CMD, CMD_AT);
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS), ==, ST_TBS);
    g_assert_cmphex(rd(0, INT), ==, INT_TI);
    step(1000 * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 8);
    stop();
}

/*
 * Self-test mode needs no acknowledgement; self-reception receives the
 * frame sent, on a QEMU CAN bus alone or with twai_tx looped to twai_rx on
 * one pad.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_self_test(void)
{
    static const uint8_t d[3] = { 9, 8, 7 };
    uint8_t f[13];
    int bits, len;
    int64_t sof;

    two_nodes();
    wr(1, MODE, MODE_RM);
    wr(0, MODE, MODE_RM);
    wr(0, MODE, MODE_STM);
    step(20 * BIT_NS);
    len = sff(f, 0x2aa, false, 3, d);
    bits = frame_bits(f);
    load(0, f);
    wr(0, CMD, CMD_SRR);
    sof = next_sof();
    clock_to(sof + bits * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS | ST_RBS), ==,
                    ST_TBS | ST_TCS | ST_RBS);
    g_assert_cmphex(rd(0, RMC), ==, 1);
    expect_rx(0, f, len);
    wr(0, CMD, CMD_RRB);
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    /* TR alone: sent, not received. */
    load(0, f);
    wr(0, CMD, CMD_TR);
    step((bits + 4) * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, ST_TCS);
    g_assert_cmphex(rd(0, RMC), ==, 0);
    stop();

    /* TWAI2, with no CAN bus, looped through GPIO30. */
    start("");
    clocks_on();
    route(2, 30, 30);
    setup(2, BTR0_1M, BTR1_1M, MODE_STM);
    step(20 * BIT_NS);
    load(2, f);
    wr(2, CMD, CMD_SRR);
    step((bits + 4) * BIT_NS);
    g_assert_cmphex(rd(2, STATUS) & ST_TCS, ==, ST_TCS);
    g_assert_cmphex(rd(2, RMC), ==, 1);
    expect_rx(2, f, len);
    wr(2, CMD, CMD_RRB);
    /* Without self-test mode no one acknowledges it. */
    wr(2, MODE, MODE_RM);
    wr(2, MODE, 0);
    step(20 * BIT_NS);
    load(2, f);
    wr(2, CMD, CMD_SRR);
    step((ack_slot(f) + 2) * BIT_NS);
    g_assert_cmphex(rd(2, ECC), ==, ECC_ACK_ERR_TX);
    g_assert_cmphex(rd(2, TXERR), ==, 8);
    g_assert_cmphex(rd(2, RMC), ==, 0);
    stop();
}

/*
 * A listen-only controller acknowledges nothing and signals nothing,
 * its counters frozen. It sees an error active transmitter's flag in the
 * ACK delimiter and drops the frame; an error passive transmitter's flag is
 * recessive, and it receives the frame each time it is sent.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_listen_only(void)
{
    uint8_t f[13];
    int a;

    two_nodes();
    wr(1, MODE, MODE_RM);
    wr(1, MODE, MODE_LOM);
    wr(1, INT_ENA, INT_BEI);
    step(20 * BIT_NS);
    sff(f, 0x99, false, 0, NULL);
    a = ack_slot(f);
    load(0, f);
    wr(0, CMD, CMD_TR);
    clock_to(next_sof() + (a + 2) * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 8);
    g_assert_cmphex(rd(1, ECC), ==, ECC_FORM_ACK_DELIM);
    g_assert_cmphex(rd(1, INT), ==, INT_BEI);
    g_assert_cmphex(rd(1, RXERR), ==, 0);
    g_assert_cmphex(rd(1, RMC), ==, 0);
    /* TR in listen-only mode is ignored. */
    load(1, f);
    wr(1, CMD, CMD_TR);
    g_assert_cmphex(rd(1, STATUS) & ST_TBS, ==, ST_TBS);

    /* TWAI0 error passive. */
    wr(0, MODE, MODE_RM);
    wr(0, TXERR, 200);
    wr(0, MODE, 0);
    step(20 * BIT_NS);
    load(0, f);
    wr(0, CMD, CMD_TR);
    step(3 * (a + 30) * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 200);
    g_assert_cmphex(rd(1, RMC), >=, 2);
    g_assert_cmphex(rd(1, RXERR), ==, 0);
    stop();
}

/*
 * Two transmitters starting together arbitrate bit by bit: the lower
 * identifier wins, the loser records the bit, receives the winner's
 * frame, and sends its own next.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void arbitrate(uint8_t *f0, uint8_t *f1, int alc, int u)
{
    int64_t sof;
    int bits0 = frame_bits(f0), bits1 = frame_bits(f1);
    int lost_bit = frame_code(f1, u);

    load(0, f0);
    load(1, f1);
    wr(0, CMD, CMD_TR);
    wr(1, CMD, CMD_TR);
    sof = next_sof();
    clock_to(sof + (lost_bit + 1) * BIT_NS - 1);
    g_assert_cmphex(rd(1, INT) & INT_ALI, ==, 0);
    g_assert_cmphex(rd(1, STATUS) & (ST_TS | ST_RS), ==, ST_TS);
    step(1);
    g_assert_cmphex(rd(1, INT) & INT_ALI, ==, INT_ALI);
    g_assert_cmphex(rd(1, STATUS) & (ST_TS | ST_RS), ==, ST_RS);
    g_assert_cmphex(rd(1, ALC), ==, alc);
    clock_to(sof + bits0 * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, ST_TCS);
    g_assert_cmphex(rd(1, RMC), ==, 1);
    g_assert_cmphex(rd(1, STATUS) & ST_TBS, ==, 0);
    /* After the intermission, TWAI1's frame. */
    sof += (bits0 + 3) * BIT_NS;
    clock_to(sof + bits1 * BIT_NS);
    g_assert_cmphex(rd(1, STATUS) & ST_TCS, ==, ST_TCS);
    g_assert_cmphex(rd(0, RMC), ==, 1);
    wr(0, CMD, CMD_RRB);
    wr(1, CMD, CMD_RRB);
    step(3 * BIT_NS);
}

/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_arbitration(void)
{
    uint8_t f0[13], f1[13];

    two_nodes();
    wr(1, INT_ENA, INT_ALI);
    /*
     * The loser's ARB_LOST_CAP and the unstuffed bit it lost at, counting
     * SOF as 0. 0x123 and 0x127 differ at ID.20, bit 8 of the identifier.
     */
    sff(f0, 0x123, false, 0, NULL);
    sff(f1, 0x127, false, 0, NULL);
    arbitrate(f0, f1, 8, 9);
    /* A data frame beats an extended frame of the same base ID at SRR. */
    sff(f0, 0x123, false, 0, NULL);
    eff(f1, 0x123 << 18, false, 0, NULL);
    arbitrate(f0, f1, 11, 12);
    /* A remote frame and an extended frame: the extended loses at IDE. */
    sff(f0, 0x123, true, 0, NULL);
    eff(f1, 0x123 << 18, false, 0, NULL);
    arbitrate(f0, f1, 12, 13);
    /* Extended frames differing in ID.0, and in RTR. */
    eff(f0, 0x0000010, false, 0, NULL);
    eff(f1, 0x0000011, false, 0, NULL);
    arbitrate(f0, f1, 30, 31);
    eff(f0, 0x0000010, false, 0, NULL);
    eff(f1, 0x0000010, true, 0, NULL);
    arbitrate(f0, f1, 31, 32);
    stop();
}

/* The capture holds the first loss until read */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_capture_hold(void)
{
    uint8_t f0[13], f1[13];

    two_nodes();
    sff(f0, 0x001, false, 0, NULL);
    sff(f1, 0x400, false, 0, NULL);
    load(0, f0);
    load(1, f1);
    wr(0, CMD, CMD_TR);
    wr(1, CMD, CMD_TR);
    step(200 * BIT_NS);
    wr(0, CMD, CMD_RRB);
    wr(1, CMD, CMD_RRB);
    sff(f1, 0x002, false, 0, NULL);
    load(0, f0);
    load(1, f1);
    wr(0, CMD, CMD_TR);
    wr(1, CMD, CMD_TR);
    step(200 * BIT_NS);
    /* Lost at bit 0 first, then at bit 9; the first is kept. */
    g_assert_cmphex(rd(1, ALC), ==, 0);
    g_assert_cmphex(rd(1, ALC), ==, 0);
    stop();
}

/*
 * The receive FIFO: 64 bytes, a message that does not fit whole is an
 * overrun message, counted and marked missing, DOS and DOI set once.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_overrun(void)
{
    static const uint8_t d[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t f[13];
    int i;

    two_nodes();
    wr(1, INT_ENA, INT_DOI);
    for (i = 0; i < 6; i++) {
        eff(f, 0x100 + i, false, 8, d);
        load(0, f);
        wr(0, CMD, CMD_TR);
        step((frame_bits(f) + 4) * BIT_NS);
        g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, ST_TCS);
        g_assert_cmphex(rd(1, RMC), ==, i + 1);
        g_assert_cmphex(rd(1, STATUS) & ST_DOS, ==, i >= 4 ? ST_DOS : 0);
        g_assert_cmphex(rd(1, INT) & INT_DOI, ==, i == 4 ? INT_DOI : 0);
    }
    for (i = 0; i < 6; i++) {
        g_assert_cmphex(rd(1, STATUS) & ST_MS, ==, i >= 4 ? ST_MS : 0);
        if (i < 4) {
            g_assert_cmphex(rd(1, DATA(1)), ==, (0x100 + i) >> 21);
            g_assert_cmphex(rd(1, DATA(4)), ==, ((0x100 + i) << 3) & 0xff);
        }
        wr(1, CMD, CMD_RRB);
        g_assert_cmphex(rd(1, RMC), ==, 5 - i);
    }
    g_assert_cmphex(rd(1, STATUS) & (ST_RBS | ST_DOS), ==, ST_DOS);
    wr(1, CMD, CMD_CDO);
    g_assert_cmphex(rd(1, STATUS) & ST_DOS, ==, 0);
    stop();
}

/*
 * Bus-off forced by writing 255 to the TEC in reset mode, and recovery:
 * 128 runs of 11 recessive bits counting the TEC down from 127.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_bus_off_recovery(void)
{
    int64_t t0;

    two_nodes();
    wr(0, INT_ENA, INT_EI | INT_EPI);
    wr(0, MODE, MODE_RM);
    wr(0, TXERR, 255);
    g_assert_cmphex(rd(0, STATUS) & (ST_BS | ST_ES), ==, 0);
    wr(0, MODE, 0);
    g_assert_cmphex(rd(0, MODE), ==, MODE_RM);
    g_assert_cmphex(rd(0, STATUS), ==, ST_TBS | ST_TCS | ST_ES | ST_BS);
    g_assert_cmphex(rd(0, TXERR), ==, 127);
    g_assert_cmphex(rd(0, RXERR), ==, 0);
    g_assert_cmphex(rd(0, INT), ==, INT_EI);

    /* Recovery: TEC 95, below the warning limit, after 32 runs. */
    wr(0, MODE, 0);
    t0 = now;
    clock_to(t0 + 32 * 11 * BIT_NS - 1);
    g_assert_cmphex(rd(0, TXERR), ==, 96);
    g_assert_cmphex(rd(0, STATUS) & (ST_BS | ST_ES), ==, ST_BS | ST_ES);
    step(1);
    g_assert_cmphex(rd(0, TXERR), ==, 95);
    g_assert_cmphex(rd(0, STATUS) & (ST_BS | ST_ES), ==, ST_BS);
    g_assert_cmphex(rd(0, INT), ==, INT_EI);
    clock_to(t0 + 127 * 11 * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    g_assert_cmphex(rd(0, STATUS) & ST_BS, ==, ST_BS);
    clock_to(t0 + 128 * 11 * BIT_NS - 1);
    g_assert_cmphex(rd(0, STATUS) & ST_BS, ==, ST_BS);
    step(1);
    g_assert_cmphex(rd(0, STATUS) & (ST_BS | ST_ES), ==, 0);
    g_assert_cmphex(rd(0, INT), ==, INT_EI);

    /* Back on the bus at once. */
    {
        uint8_t f[13];

        sff(f, 0x3, false, 0, NULL);
        g_assert_true(passes(f));
    }

    /* Reset mode freezes recovery. */
    wr(0, MODE, MODE_RM);
    wr(0, TXERR, 255);
    wr(0, MODE, 0);
    wr(0, MODE, 0);
    step(10 * 11 * BIT_NS);
    wr(0, MODE, MODE_RM);
    g_assert_cmphex(rd(0, TXERR), ==, 117);
    step(100 * 11 * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 117);
    g_assert_cmphex(rd(0, STATUS) & ST_BS, ==, ST_BS);
    /* A TEC write ends bus-off as reset mode ends. */
    wr(0, TXERR, 0);
    g_assert_cmphex(rd(0, STATUS) & ST_BS, ==, ST_BS);
    wr(0, MODE, 0);
    g_assert_cmphex(rd(0, STATUS) & ST_BS, ==, 0);
    stop();
}

/*
 * Without a transceiver, twai_rx unrouted, the controller never hears its
 * own SOF: a bit error, then one per bit of its active error flag until
 * error passive; then one per attempt until bus-off.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_no_transceiver(void)
{
    uint8_t f[13];
    int64_t t0;

    start("");
    clocks_on();
    setup(2, BTR0_1M, BTR1_1M, 0);
    step(20 * BIT_NS);
    wr(2, INT_ENA, INT_BEI | INT_EPI | INT_EI);
    sff(f, 0x5, false, 0, NULL);
    load(2, f);
    t0 = now;
    wr(2, CMD, CMD_TR);
    clock_to(t0 + BIT_NS);
    g_assert_cmphex(rd(2, TXERR), ==, 8);
    g_assert_cmphex(rd(2, ECC), ==, ECC_BIT_SOF_TX);
    clock_to(t0 + 16 * BIT_NS - 1);
    g_assert_cmphex(rd(2, TXERR), ==, 120);
    step(1);
    g_assert_cmphex(rd(2, TXERR), ==, 128);
    g_assert_cmphex(rd(2, INT), ==, INT_BEI | INT_EI | INT_EPI);
    /* Passive flag, delimiter, intermission, suspend; 16 more attempts. */
    clock_to(t0 + (16 + 6 + 8 + 3 + 8 + 15 * 26 + 1) * BIT_NS - 1);
    g_assert_cmphex(rd(2, TXERR), ==, 248);
    g_assert_cmphex(rd(2, MODE), ==, 0);
    step(1);
    g_assert_cmphex(rd(2, MODE), ==, MODE_RM);
    g_assert_cmphex(rd(2, TXERR), ==, 127);
    g_assert_cmphex(rd(2, STATUS) & (ST_BS | ST_ES | ST_TBS), ==,
                    ST_BS | ST_ES | ST_TBS);
    stop();
}

/*
 * A receiver within its oscillator tolerance, resynchronising, receives;
 * one at half the bit rate detects errors and destroys the frame.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_bit_rate_mismatch(void)
{
    static const uint8_t d[8] = { 0x00, 0xff, 0x0f, 0xf0, 0x55, 0xaa, 0x01,
                                  0x80 };
    uint8_t f[13];
    int i;

    two_nodes();
    /* 100 kbit/s, 10 us a bit, against 9.9 us (BRP 8, 22 quanta). */
    setup(0, 0x09 | 0xc000, 0x3e, 0);
    setup(1, 0x08 | 0xc000, 0x4f, 0);
    step(200 * BIT_NS);
    eff(f, 0x15555555, false, 8, d);
    load(0, f);
    wr(0, CMD, CMD_TR);
    step((frame_bits(f) + 30) * 10 * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, ST_TCS);
    g_assert_cmphex(rd(1, RMC), ==, 1);
    expect_rx(1, f, 13);
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    g_assert_cmphex(rd(1, RXERR), ==, 0);
    wr(1, CMD, CMD_RRB);

    /* TWAI1 at 50 kbit/s. */
    setup(1, 0x13 | 0xc000, 0x3e, 0);
    wr(1, INT_ENA, INT_BEI);
    step(400 * 10 * BIT_NS);
    load(0, f);
    wr(0, CMD, CMD_TR);
    for (i = 0; i < 3; i++) {
        step(400 * 10 * BIT_NS);
    }
    g_assert_cmphex(rd(1, RMC), ==, 0);
    g_assert_cmphex(rd(1, INT) & INT_BEI, ==, INT_BEI);
    g_assert_cmpuint(rd(1, RXERR), >, 0);
    g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, 0);
    stop();
}

/*
 * A receiver at another bit time samples the bus at the exact times of
 * its own quanta and the transmitter's edges. Both controllers count
 * RC_FAST_CLK trimmed to FOSC_DFREQ 53, a cycle of 120000/2053 ns, which
 * is no whole number of picoseconds: TWAI0 sends 0x653 at 80 cycles a bit
 * (BRP 4, TSEG1 3, TSEG2 2) and TWAI1, at 76 (BRP 1, TSEG1 14, TSEG2 2,
 * SJW 2), resynchronising on its edges, takes every bit right, receives
 * it and acknowledges it. With the bit time and quantum rounded down to
 * picoseconds, a sample falls the wrong side of an edge and TWAI1 finds
 * the CRC wrong.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_bit_rate_exact(void)
{
    uint8_t f[13];

    two_nodes();
    qtest_writel(qts, FOSC_CNTL, 53u << 22);
    qtest_writel(qts, PERI_CLK_CTRL115,
                 TWAI_CLK_EN(0) | TWAI_CLK_EN(1) | TWAI_CLK_EN(2) |
                 TWAI_SRC_RC_FAST(0) | TWAI_SRC_RC_FAST(1));
    setup(0, 0x04, 0x23, 0);
    setup(1, 0x4001, 0x2e, 0);
    step(100 * 5 * BIT_NS);
    sff(f, 0x653, false, 0, NULL);
    load(0, f);
    wr(0, CMD, CMD_TR);
    step((frame_bits(f) + 30) * 5 * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & ST_TCS, ==, ST_TCS);
    g_assert_cmphex(rd(1, RMC), ==, 1);
    expect_rx(1, f, 3);
    g_assert_cmphex(rd(1, RXERR), ==, 0);
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    stop();
}

/*
 * The timestamp counter: TWAIn_CLK / (TS_DIV_NUM + 1), counting from 0 as
 * it is enabled; each frame stamped at its SOF; its overflow interrupt.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_timestamp(void)
{
    uint8_t f[13];
    int64_t t_en, sof;

    /*
     * Strapped for download boot: the flash boot watchdogs, which would
     * reset the chip within seconds, do not run.
     */
    two_nodes_with(",strap-mode=0");
    wr(1, TS_PRESCALER, 39);
    t_en = now;
    wr(1, TS_CFG, 1);
    step(12345);
    sff(f, 0x1, false, 0, NULL);
    load(0, f);
    wr(0, CMD, CMD_TR);
    sof = next_sof();
    step((frame_bits(f) + 4) * BIT_NS);
    g_assert_cmphex(rd(1, RMC), ==, 1);
    g_assert_cmpuint(rd(1, TS_DATA), ==, (sof - t_en) / 1000);
    wr(1, CMD, CMD_RRB);

    /* At 40 MHz the 32-bit count wraps after 107.3741824 s. */
    wr(1, TS_PRESCALER, 0);
    wr(1, TS_CFG, 0);
    t_en = now;
    wr(1, TS_CFG, 1);
    wr(1, INT_ENA, INT_TSI);
    clock_to(t_en + 4294967296LL * 25 - 1);
    g_assert_cmphex(rd(1, INT), ==, 0);
    step(1);
    g_assert_cmphex(rd(1, INT), ==, INT_TSI);
    stop();
}

/* The function clock: stopped, the controller does nothing */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_function_clock(void)
{
    uint8_t f[13];

    two_nodes();
    qtest_writel(qts, PERI_CLK_CTRL115, TWAI_CLK_EN(1));
    sff(f, 0x7, false, 0, NULL);
    load(0, f);
    wr(0, CMD, CMD_TR);
    step(1000 * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS), ==, 0);
    g_assert_cmphex(rd(0, TXERR), ==, 0);
    qtest_writel(qts, PERI_CLK_CTRL115, TWAI_CLK_EN(0) | TWAI_CLK_EN(1));
    step(200 * BIT_NS);
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS), ==, ST_TBS | ST_TCS);
    g_assert_cmphex(rd(1, RMC), ==, 1);
    stop();
}

/* HP_SYS_CLKRST_RST_EN_CANn resets its controller alone */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_block_reset(void)
{
    start("");
    clocks_on();
    wr(0, EWL, 5);
    wr(1, EWL, 6);
    qtest_writel(qts, HP_RST_EN1, RST_EN_CAN(0));
    qtest_writel(qts, HP_RST_EN1, 0);
    g_assert_cmphex(rd(0, EWL), ==, 96);
    g_assert_cmphex(rd(1, EWL), ==, 6);
    stop();
}

/* The standby signal, software and hardware controlled */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_standby(void)
{
    uint8_t f[13];

    two_nodes();
    qtest_irq_intercept_out_named(qts, TWAI_PATH, "standby");
    g_assert_false(qtest_get_irq(qts, 0));
    wr(0, SW_STANDBY, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(0, SW_STANDBY, 3);
    g_assert_false(qtest_get_irq(qts, 0));
    wr(0, SW_STANDBY, 0);
    wr(0, SW_STANDBY, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    /* Bus activity ends it. */
    sff(f, 0x7, false, 0, NULL);
    load(1, f);
    wr(1, CMD, CMD_TR);
    step(2 * BIT_NS);
    g_assert_false(qtest_get_irq(qts, 0));
    step(200 * BIT_NS);

    /* Hardware: 400 cycles of 25 ns of bus idle. */
    wr(0, HW_STANDBY_CNT, 400);
    wr(0, HW_CFG, 1);
    step(10000);
    g_assert_true(qtest_get_irq(qts, 0));
    stop();
}

/*
 * Two transmitters with one identifier and different data: past
 * arbitration the one sending recessive has a bit error and its active
 * flag gives the other one; both count 8 a round until error passive.
 * Then the second's passive flag leaves the first's frame on the bus, and
 * only it counts on, to bus-off.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_collision(void)
{
    static const uint8_t d0[1] = { 0x00 }, d1[1] = { 0x80 };
    uint8_t f0[13], f1[13];

    two_nodes();
    sff(f0, 0x50, false, 1, d0);
    sff(f1, 0x50, false, 1, d1);
    load(0, f0);
    load(1, f1);
    wr(0, CMD, CMD_TR);
    wr(1, CMD, CMD_TR);
    step(40 * BIT_NS);
    g_assert_cmphex(rd(0, TXERR), ==, 8);
    g_assert_cmphex(rd(1, TXERR), ==, 8);
    /* Both in the data field: bit errors, transmitting. */
    g_assert_cmphex(rd(0, ECC), ==, 0x0a);
    g_assert_cmphex(rd(1, ECC), ==, 0x0a);
    step(20000 * BIT_NS);
    g_assert_cmphex(rd(1, STATUS) & ST_BS, ==, ST_BS);
    g_assert_cmphex(rd(1, MODE) & MODE_RM, ==, MODE_RM);
    g_assert_cmphex(rd(0, TXERR), ==, 128);
    g_assert_cmphex(rd(0, STATUS) & (ST_BS | ST_TCS), ==, 0);
    stop();
}

/*
 * A transmitter leaving mid-frame for reset mode leaves the bus
 * recessive; the receiver detects six equal bits, a stuff error.
 */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_abandoned_frame(void)
{
    static const uint8_t d[8] = { 0 };
    uint8_t f[13];
    int64_t sof;

    two_nodes();
    wr(1, INT_ENA, INT_BEI);
    sff(f, 0x7f0, false, 8, d);
    load(0, f);
    wr(0, CMD, CMD_TR);
    sof = next_sof();
    clock_to(sof + 30 * BIT_NS + 500);
    wr(0, MODE, MODE_RM);
    g_assert_cmphex(rd(0, STATUS) & (ST_TBS | ST_TCS | ST_TS), ==, ST_TBS);
    step(10 * BIT_NS);
    g_assert_cmphex(rd(1, INT), ==, INT_BEI);
    g_assert_cmphex(rd(1, ECC) & 0xe0, ==, 0xa0);
    g_assert_cmphex(rd(1, RXERR), ==, 1);
    g_assert_cmphex(rd(1, RMC), ==, 0);
    stop();
}

/* twai_clkout: TWAIn_CLK / (2 * (CD + 1)), off with CLOCK_OFF */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_clkout(void)
{
    uint32_t in;

    start("");
    clocks_on();
    pad_gpio(24);
    qtest_writel(qts, GPIO_FUNC_OUT(24), 82);
    /* CD 3: 40 MHz / 8, high for 100 ns of every 200. */
    wr(0, CDR, 3);
    for (int t = 0; t < 4; t++) {
        clock_to(1000 + 50 * t + 25);
        in = qtest_readl(qts, GPIO + 0x3c);
        g_assert_cmphex((in >> 24) & 1, ==, (t / 2) & 1);
    }
    wr(0, CDR, 0x103);
    step(100);
    g_assert_cmphex((qtest_readl(qts, GPIO + 0x3c) >> 24) & 1, ==, 0);
    step(100);
    g_assert_cmphex((qtest_readl(qts, GPIO + 0x3c) >> 24) & 1, ==, 0);
    stop();
}

/* The idle interrupt's enable is read-only zero */
/* [spec:nuos:req:emu.esp32p4.twai/test] */
static void test_idle_interrupt(void)
{
    two_nodes();
    wr(0, INT_ENA, 0x100);
    g_assert_cmphex(rd(0, INT_ENA), ==, 0);
    wr(0, IDLE_INTR_CNT, 10);
    step(1000 * BIT_NS);
    g_assert_cmphex(rd(0, INT), ==, 0);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/twai/reset_values", test_reset_values);
    qtest_add_func("esp32p4/twai/register_access", test_register_access);
    qtest_add_func("esp32p4/twai/frames", test_frames);
    qtest_add_func("esp32p4/twai/interrupts", test_interrupts);
    qtest_add_func("esp32p4/twai/filters", test_filters);
    qtest_add_func("esp32p4/twai/ack_error", test_ack_error);
    qtest_add_func("esp32p4/twai/single_shot", test_single_shot);
    qtest_add_func("esp32p4/twai/self_test", test_self_test);
    qtest_add_func("esp32p4/twai/listen_only", test_listen_only);
    qtest_add_func("esp32p4/twai/arbitration", test_arbitration);
    qtest_add_func("esp32p4/twai/capture_hold", test_capture_hold);
    qtest_add_func("esp32p4/twai/overrun", test_overrun);
    qtest_add_func("esp32p4/twai/bus_off_recovery", test_bus_off_recovery);
    qtest_add_func("esp32p4/twai/no_transceiver", test_no_transceiver);
    qtest_add_func("esp32p4/twai/bit_rate_mismatch", test_bit_rate_mismatch);
    qtest_add_func("esp32p4/twai/bit_rate_exact", test_bit_rate_exact);
    qtest_add_func("esp32p4/twai/timestamp", test_timestamp);
    qtest_add_func("esp32p4/twai/function_clock", test_function_clock);
    qtest_add_func("esp32p4/twai/block_reset", test_block_reset);
    qtest_add_func("esp32p4/twai/standby", test_standby);
    qtest_add_func("esp32p4/twai/idle_interrupt", test_idle_interrupt);
    qtest_add_func("esp32p4/twai/collision", test_collision);
    qtest_add_func("esp32p4/twai/abandoned_frame", test_abandoned_frame);
    qtest_add_func("esp32p4/twai/clkout", test_clkout);
    return g_test_run();
}
