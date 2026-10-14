/*
 * QTest testcase for the ESP32-P4 RMT
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * rmt_sclk is the 40 MHz crystal (25 ns a cycle). TX channels drive pads
 * through the GPIO matrix, and pads feed the RX channels back, so a
 * transmitted sequence is received and its durations checked to the tick;
 * pad levels are checked at exact virtual times by reading GPIO_IN.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RMT                 0x500a2000u
#define TX_DATA(n)          (RMT + 0x00 + 4 * (n))
#define RX_DATA(m)          (RMT + 0x10 + 4 * (m))
#define TX_CONF0(n)         (RMT + 0x20 + 4 * (n))
#define RX_CONF0(m)         (RMT + 0x30 + 8 * (m))
#define RX_CONF1(m)         (RMT + 0x34 + 8 * (m))
#define TX_STATUS(n)        (RMT + 0x50 + 4 * (n))
#define RX_STATUS(m)        (RMT + 0x60 + 4 * (m))
#define INT_RAW             (RMT + 0x70)
#define INT_ST              (RMT + 0x74)
#define INT_ENA             (RMT + 0x78)
#define INT_CLR             (RMT + 0x7c)
#define TX_CARRIER(n)       (RMT + 0x80 + 4 * (n))
#define RX_CARRIER_RM(m)    (RMT + 0x90 + 4 * (m))
#define TX_LIM(n)           (RMT + 0xa0 + 4 * (n))
#define RX_LIM(m)           (RMT + 0xb0 + 4 * (m))
#define SYS_CONF            (RMT + 0xc0)
#define TX_SIM              (RMT + 0xc4)
#define REF_CNT_RST         (RMT + 0xc8)
#define DATE                (RMT + 0xcc)
#define RAM(w)              (RMT + 0x800 + 4 * (w))
#define TX_BLOCK(n)         (48 * (n))
#define RX_BLOCK(m)         (48 * (4 + (m)))

#define TX_START            (1u << 0)
#define TX_MEM_RD_RST       (1u << 1)
#define TX_APB_MEM_RST      (1u << 2)
#define TX_CONTI            (1u << 3)
#define TX_WRAP             (1u << 4)
#define TX_IDLE_LV          (1u << 5)
#define TX_IDLE_EN          (1u << 6)
#define TX_STOP             (1u << 7)
#define TX_DIV(d)           ((d) << 8)
#define TX_MEM_SIZE(b)      ((b) << 16)
#define TX_CARRIER_EFF      (1u << 20)
#define TX_CARRIER_EN       (1u << 21)
#define TX_CARRIER_LV       (1u << 22)
#define TX_UPDATE           (1u << 24)

#define RX_DIV(d)           (d)
#define RX_IDLE(t)          ((t) << 8)
#define RX_MEM_SIZE(b)      ((b) << 24)
#define RX_CARRIER_EN       (1u << 28)
#define RX_CARRIER_LV       (1u << 29)
#define RX_EN               (1u << 0)
#define RX_MEM_WR_RST       (1u << 1)
#define RX_APB_MEM_RST      (1u << 2)
#define RX_OWNER            (1u << 3)
#define RX_FILTER_EN        (1u << 4)
#define RX_FILTER(t)        ((t) << 5)
#define RX_WRAP             (1u << 13)
#define RX_UPDATE           (1u << 15)

#define INT_TX_END(n)       (1u << (n))
#define INT_TX_ERR(n)       (1u << (4 + (n)))
#define INT_TX_THR(n)       (1u << (8 + (n)))
#define INT_TX_LOOP(n)      (1u << (12 + (n)))
#define INT_RX_END(m)       (1u << (16 + (m)))
#define INT_RX_ERR(m)       (1u << (20 + (m)))
#define INT_RX_THR(m)       (1u << (24 + (m)))

#define LOOP_NUM(k)         ((k) << 9)
#define LOOP_CNT_EN         (1u << 19)
#define LOOP_CNT_RESET      (1u << 20)
#define LOOP_STOP_EN        (1u << 21)

#define FIFO_MASK           (1u << 0)
#define MEM_FORCE_PD        (1u << 2)
#define TX_SIM_EN           (1u << 4)

#define ST_STATE(v)         (((v) >> 22) & 7)
#define ST_TX_EMPTY         (1u << 25)
#define ST_TX_WR_ERR        (1u << 26)
#define ST_RX_OWNER_ERR     (1u << 25)
#define ST_RX_FULL          (1u << 26)
#define ST_RX_RD_ERR        (1u << 27)

#define CLKRST              0x500e6000u
#define SOC_CLK_CTRL2       (CLKRST + 0x1c)
#define PERI_CLK_CTRL22     (CLKRST + 0x9c)
#define HP_RST_EN1          (CLKRST + 0xc4)
#define RMT_SYS_CLK_EN      (1u << 0)
#define RMT_CLK_EN          (1u << 5)
#define RST_EN_RMT          (1u << 23)

#define GPIO                0x500e0000u
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004u + 4 * (n))
#define MCU_SEL_GPIO        (1u << 12)
#define FUN_IE              (1u << 9)
#define FUN_DRV2            (2u << 10)
#define SIG_RMT(n)          (246 + (n))
#define GPIO_STATUS         (GPIO + 0x044)
#define GPIO_STATUS_W1TC    (GPIO + 0x04c)
#define GPIO_PIN(n)         (GPIO + 0x074 + 4 * (n))
#define PIN_INT_RISING      (1u << 7)
#define GPIO_PATH           "/machine/gpio"

#define INTMTX_STATUS1      0x500d6204u
#define SRC_RMT_BIT         (1u << 11)

/* The crystal's cycle */
#define CYC                 25

static QTestState *qts;
static int64_t now;

static uint32_t rd(uint32_t a)
{
    return qtest_readl(qts, a);
}

static void wr(uint32_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
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

static void start(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    now = 0;
}

static void stop(void)
{
    qtest_quit(qts);
    qts = NULL;
}

/* The bus clock and rmt_sclk from the crystal; direct RAM access. */
static void clocks_on(void)
{
    wr(SOC_CLK_CTRL2, rd(SOC_CLK_CTRL2) | RMT_SYS_CLK_EN);
    wr(PERI_CLK_CTRL22, RMT_CLK_EN);
    wr(SYS_CONF, rd(SYS_CONF) | FIFO_MASK);
}

static uint32_t entry(int level, uint32_t period)
{
    return (level ? 0x8000u : 0) | period;
}

static uint32_t word(uint32_t lo, uint32_t hi)
{
    return lo | (hi << 16);
}

static void pad_gpio(int pad)
{
    wr(IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE | FUN_DRV2);
}

/* TX channel n out on pad, which feeds RX channel m. */
static void loop_tx_rx(int n, int m, int pad)
{
    pad_gpio(pad);
    wr(GPIO_FUNC_OUT(pad), SIG_RMT(n));
    if (m >= 0) {
        wr(GPIO_FUNC_IN(SIG_RMT(m)), IN_SIG_IN_SEL | pad);
    }
}

static int pad_level(int pad)
{
    return (rd(GPIO_IN) >> pad) & 1;
}

/* Configure TX channel n and latch it with CONF_UPDATE. */
static void tx_setup(int n, uint32_t conf0)
{
    wr(TX_CONF0(n), conf0 | TX_MEM_RD_RST | TX_APB_MEM_RST);
    wr(TX_CONF0(n), conf0 | TX_UPDATE);
}

/* Configure RX channel m, latch it and enable it, as rmt_ll does. */
static void rx_setup(int m, uint32_t conf0, uint32_t conf1)
{
    wr(RX_CONF0(m), conf0);
    wr(RX_CONF1(m), conf1 | RX_OWNER | RX_MEM_WR_RST | RX_APB_MEM_RST);
    wr(RX_CONF1(m), conf1 | RX_OWNER | RX_EN | RX_UPDATE);
}

/*
 * Restart TX channel n's divider now, so that its channel clock ticks at
 * now + k * div cycles, and start it: the first entry begins a tick later.
 */
static void tx_go(int n, uint32_t conf0)
{
    wr(REF_CNT_RST, 1u << n);
    wr(TX_CONF0(n), conf0 | TX_START);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_reset_values(void)
{
    start();
    clocks_on();
    wr(SYS_CONF, 0x05000010);
    for (int n = 0; n < 4; n++) {
        g_assert_cmphex(rd(TX_CONF0(n)), ==, 0x00710200);
        g_assert_cmphex(rd(TX_STATUS(n)), ==, 0);
        g_assert_cmphex(rd(TX_CARRIER(n)), ==, 0x00400040);
        g_assert_cmphex(rd(TX_LIM(n)), ==, 0x80);
        g_assert_cmphex(rd(RX_CONF0(n)), ==, 0x317fff02);
        g_assert_cmphex(rd(RX_CONF1(n)), ==, 0x1e8);
        g_assert_cmphex(rd(RX_STATUS(n)), ==, 192 | (192 << 11));
        g_assert_cmphex(rd(RX_CARRIER_RM(n)), ==, 0);
        g_assert_cmphex(rd(RX_LIM(n)), ==, 0x80);
    }
    g_assert_cmphex(rd(SYS_CONF), ==, 0x05000010);
    g_assert_cmphex(rd(TX_SIM), ==, 0);
    g_assert_cmphex(rd(DATE), ==, 0x02201111);
    g_assert_cmphex(rd(INT_RAW), ==, 0);

    /*
     * Write-trigger bits read back as 0; DMA_ACCESS_EN is only channel 3's;
     * TX_STOP holds until a CONF_UPDATE, which clears it.
     */
    wr(TX_CONF0(0), 0xffffffff & ~TX_UPDATE);
    g_assert_cmphex(rd(TX_CONF0(0)), ==, 0x007ffff8);
    wr(TX_CONF0(3), 0xffffffff & ~TX_UPDATE);
    g_assert_cmphex(rd(TX_CONF0(3)), ==, 0x027ffff8);
    wr(TX_CONF0(3), 0xffffffff);
    g_assert_cmphex(rd(TX_CONF0(3)), ==, 0x027fff78);
    wr(RX_CONF0(0), 0xffffffff);
    g_assert_cmphex(rd(RX_CONF0(0)), ==, 0x3f7fffff);
    wr(RX_CONF0(3), 0xffffffff);
    g_assert_cmphex(rd(RX_CONF0(3)), ==, 0x3fffffff);
    wr(RX_CONF1(0), 0xffffffff);
    g_assert_cmphex(rd(RX_CONF1(0)), ==, 0x3ff9);
    wr(TX_LIM(0), 0xffffffff);
    g_assert_cmphex(rd(TX_LIM(0)), ==, 0x2fffff);
    wr(DATE, 0xffffffff);
    g_assert_cmphex(rd(DATE), ==, 0x0fffffff);
    wr(SYS_CONF, 0xffffffff);
    g_assert_cmphex(rd(SYS_CONF), ==, 0x87ffffff);
    stop();
}

/*
 * A sequence sent on TX channel 0 at 100 ns a tick, the pad checked at
 * its edges, and received on RX channel 0 to the tick.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_tx_rx_loopback(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN;
    int64_t t0;

    start();
    clocks_on();
    loop_tx_rx(0, 0, 20);
    wr(RAM(TX_BLOCK(0) + 0), word(entry(1, 10), entry(0, 20)));
    wr(RAM(TX_BLOCK(0) + 1), word(entry(1, 30), entry(0, 40)));
    wr(RAM(TX_BLOCK(0) + 2), word(entry(1, 5), entry(0, 0)));
    tx_setup(0, conf);
    rx_setup(0, RX_DIV(4) | RX_IDLE(100) | RX_MEM_SIZE(1), 0);
    step(1000);

    t0 = now;
    tx_go(0, conf);
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 1);
    /* The first entry starts on the tick after TX_START */
    clock_to(t0 + 100 - 1);
    g_assert_cmpint(pad_level(20), ==, 0);
    clock_to(t0 + 100);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 100 + 1000 - 1);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 100 + 1000);
    g_assert_cmpint(pad_level(20), ==, 0);
    clock_to(t0 + 100 + 3000);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 100 + 10000);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 100 + 10500 - 1);
    g_assert_cmpint(pad_level(20), ==, 1);
    g_assert_cmphex(rd(INT_RAW) & INT_TX_END(0), ==, 0);
    clock_to(t0 + 100 + 10500);
    g_assert_cmpint(pad_level(20), ==, 0);
    g_assert_cmphex(rd(INT_RAW) & INT_TX_END(0), ==, INT_TX_END(0));
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 0);
    g_assert_cmphex(rd(TX_STATUS(0)) & 0x3ff, ==, TX_BLOCK(0));

    /* The receiver ends the frame 101 ticks after the last edge. */
    g_assert_cmpuint(ST_STATE(rd(RX_STATUS(0))), ==, 3);
    clock_to(t0 + 100 + 10500 + 10100 - 1);
    g_assert_cmphex(rd(INT_RAW) & INT_RX_END(0), ==, 0);
    clock_to(t0 + 100 + 10500 + 10100);
    g_assert_cmphex(rd(INT_RAW) & INT_RX_END(0), ==, INT_RX_END(0));
    g_assert_cmpuint(ST_STATE(rd(RX_STATUS(0))), ==, 0);
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 0)), ==,
                    word(entry(1, 10), entry(0, 20)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 1)), ==,
                    word(entry(1, 30), entry(0, 40)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 2)), ==,
                    word(entry(1, 5), entry(0, 0)));
    g_assert_cmphex(rd(RX_STATUS(0)) & 0x3ff, ==, RX_BLOCK(0) + 3);

    /* The interrupt reaches the matrix once enabled. */
    g_assert_cmphex(rd(INTMTX_STATUS1) & SRC_RMT_BIT, ==, 0);
    wr(INT_ENA, INT_RX_END(0));
    g_assert_cmphex(rd(INT_ST), ==, INT_RX_END(0));
    g_assert_cmphex(rd(INTMTX_STATUS1) & SRC_RMT_BIT, ==, SRC_RMT_BIT);
    wr(INT_CLR, INT_RX_END(0));
    g_assert_cmphex(rd(INTMTX_STATUS1) & SRC_RMT_BIT, ==, 0);
    /* INT_RAW is write-1-to-clear as well. */
    wr(INT_RAW, INT_TX_END(0));
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    stop();
}

/*
 * Configuration takes effect on CONF_UPDATE: a divider written without it
 * leaves the old one in force.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_conf_update(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN;
    int64_t t0;

    start();
    clocks_on();
    loop_tx_rx(0, -1, 20);
    wr(RAM(TX_BLOCK(0)), word(entry(1, 10), entry(0, 0)));
    tx_setup(0, conf);
    /* DIV 8 written, not latched: the pulse is still 10 x 100 ns. */
    wr(TX_CONF0(0), TX_DIV(8) | TX_MEM_SIZE(1) | TX_IDLE_EN);
    t0 = now;
    wr(REF_CNT_RST, 1);
    wr(TX_CONF0(0), TX_DIV(8) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_START);
    clock_to(t0 + 100);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 1100);
    g_assert_cmpint(pad_level(20), ==, 0);

    /* Latched, it is 10 x 200 ns. */
    step(1000);
    t0 = now;
    wr(TX_CONF0(0), TX_DIV(8) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_UPDATE);
    wr(REF_CNT_RST, 1);
    wr(TX_CONF0(0), TX_DIV(8) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_START);
    clock_to(t0 + 200);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 200 + 2000 - 1);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(t0 + 200 + 2000);
    g_assert_cmpint(pad_level(20), ==, 0);

    /* The idle level follows IDLE_OUT_LV once latched. */
    wr(TX_CONF0(0), TX_DIV(8) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_IDLE_LV);
    step(100);
    g_assert_cmpint(pad_level(20), ==, 0);
    wr(TX_CONF0(0), TX_DIV(8) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_IDLE_LV |
       TX_UPDATE);
    step(100);
    g_assert_cmpint(pad_level(20), ==, 1);

    /* TX_STOP stops a transmission at CONF_UPDATE, and clears itself. */
    wr(INT_CLR, 0xffffffff);
    wr(RAM(TX_BLOCK(0)), word(entry(0, 1000), entry(0, 0)));
    tx_go(0, conf | TX_IDLE_LV);
    step(10000);
    g_assert_cmpint(pad_level(20), ==, 0);
    wr(TX_CONF0(0), conf | TX_IDLE_LV | TX_STOP);
    step(1000);
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 1);
    g_assert_cmphex(rd(TX_CONF0(0)) & TX_STOP, ==, TX_STOP);
    wr(TX_CONF0(0), conf | TX_IDLE_LV | TX_STOP | TX_UPDATE);
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 0);
    g_assert_cmphex(rd(TX_CONF0(0)) & TX_STOP, ==, 0);
    step(100);
    g_assert_cmpint(pad_level(20), ==, 1);
    g_assert_cmphex(rd(INT_RAW) & INT_TX_END(0), ==, 0);
    stop();
}

/*
 * Carrier modulation: a high entry of 80 rmt_sclk cycles sent as a
 * carrier 3 cycles high and 5 low, and demodulated by a receiver bridging
 * gaps of 5 ticks.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_carrier(void)
{
    uint32_t conf = TX_DIV(1) | TX_MEM_SIZE(1) | TX_IDLE_EN |
                    TX_CARRIER_EFF | TX_CARRIER_EN | TX_CARRIER_LV;
    int64_t t0, e;

    start();
    clocks_on();
    loop_tx_rx(0, 0, 20);
    wr(RAM(TX_BLOCK(0) + 0), word(entry(1, 80), entry(0, 40)));
    wr(RAM(TX_BLOCK(0) + 1), word(entry(1, 16), entry(0, 0)));
    wr(TX_CARRIER(0), (3 << 16) | 5);
    tx_setup(0, conf);
    wr(RX_CARRIER_RM(0), (2 << 16) | 4);
    rx_setup(0, RX_DIV(1) | RX_IDLE(200) | RX_MEM_SIZE(1) | RX_CARRIER_EN |
             RX_CARRIER_LV, 0);
    step(1000);
    t0 = now;
    tx_go(0, conf);
    e = t0 + CYC;
    for (int k = 0; k < 10; k++) {
        int64_t p = e + k * 8 * CYC;

        clock_to(p);
        g_assert_cmpint(pad_level(20), ==, 1);
        clock_to(p + 3 * CYC - 1);
        g_assert_cmpint(pad_level(20), ==, 1);
        clock_to(p + 3 * CYC);
        g_assert_cmpint(pad_level(20), ==, 0);
    }
    /* The low entry is plain; the next high one starts a new carrier. */
    clock_to(e + 120 * CYC - 1);
    g_assert_cmpint(pad_level(20), ==, 0);
    clock_to(e + 120 * CYC);
    g_assert_cmpint(pad_level(20), ==, 1);
    clock_to(e + 123 * CYC);
    g_assert_cmpint(pad_level(20), ==, 0);
    clock_to(e + 128 * CYC);
    g_assert_cmpint(pad_level(20), ==, 1);

    step(20000);
    g_assert_cmphex(rd(INT_RAW) & INT_RX_END(0), ==, INT_RX_END(0));
    /* Each carried stretch ends on a carrier gap, bridged to the tick. */
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 0)), ==,
                    word(entry(1, 80), entry(0, 40)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 1)), ==,
                    word(entry(1, 16), entry(0, 0)));

    /* Without demodulation the carrier's pulses are received as such. */
    rx_setup(0, RX_DIV(1) | RX_IDLE(200) | RX_MEM_SIZE(1), 0);
    tx_go(0, conf);
    step(20000);
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 0)), ==,
                    word(entry(1, 3), entry(0, 5)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 9)), ==,
                    word(entry(1, 3), entry(0, 45)));
    stop();
}

/*
 * The RX filter removes pulses shorter than RX_FILTER_THRES rmt_sclk
 * cycles and delays the rest by as much; the sampler loses a pulse that
 * spans no tick.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_rx_filter(void)
{
    start();
    clocks_on();
    pad_gpio(21);
    wr(GPIO_FUNC_IN(SIG_RMT(1)), IN_SIG_IN_SEL | 21);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 0);
    rx_setup(1, RX_DIV(4) | RX_IDLE(50) | RX_MEM_SIZE(1),
             RX_FILTER_EN | RX_FILTER(10));
    step(1000);
    /* A 200 ns glitch (8 cycles) is filtered out. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 1);
    step(200);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 0);
    step(10000);
    g_assert_cmpuint(ST_STATE(rd(RX_STATUS(1))), ==, 0);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    /* A 1.2 us pulse passes, 12 ticks. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 1);
    step(1200);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 0);
    step(10000);
    g_assert_cmphex(rd(INT_RAW), ==, INT_RX_END(1));
    g_assert_cmphex(rd(RAM(RX_BLOCK(1))), ==,
                    word(entry(1, 12), entry(0, 0)));

    /* Without the filter, a 50 ns pulse between ticks is not seen. */
    rx_setup(1, RX_DIV(4) | RX_IDLE(50) | RX_MEM_SIZE(1), 0);
    wr(INT_CLR, 0xffffffff);
    wr(REF_CNT_RST, 1u << 5);
    wr(RX_CONF1(1), RX_OWNER | RX_EN | RX_UPDATE);
    step(110);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 1);
    step(50);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 21, 0);
    step(10000);
    g_assert_cmphex(rd(INT_RAW), ==, 0);
    stop();
}

/*
 * Continuous mode with a loop count: three passes, TX_LOOP, and the
 * transmitter stopping by itself; the passes follow on without a gap.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_loop_count(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_CONTI;

    start();
    clocks_on();
    loop_tx_rx(0, 0, 20);
    wr(RAM(TX_BLOCK(0) + 0), word(entry(1, 3), entry(0, 2)));
    wr(RAM(TX_BLOCK(0) + 1), word(entry(1, 4), entry(0, 0)));
    wr(TX_LIM(0), 0x80 | LOOP_NUM(3) | LOOP_CNT_EN | LOOP_STOP_EN |
       LOOP_CNT_RESET);
    tx_setup(0, conf);
    rx_setup(0, RX_DIV(4) | RX_IDLE(20) | RX_MEM_SIZE(1), 0);
    tx_go(0, conf);
    step(100000);
    g_assert_cmphex(rd(INT_RAW) & (INT_TX_LOOP(0) | INT_TX_END(0)), ==,
                    INT_TX_LOOP(0) | INT_TX_END(0));
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 0);
    /* High 3, low 2, high 4 + 3, low 2, high 4 + 3, low 2, high 4. */
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 0)), ==,
                    word(entry(1, 3), entry(0, 2)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 1)), ==,
                    word(entry(1, 7), entry(0, 2)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 2)), ==,
                    word(entry(1, 7), entry(0, 2)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 3)), ==,
                    word(entry(1, 4), entry(0, 0)));
    stop();
}

/*
 * Two channels in the TX_SIM group start together once both have had
 * TX_START.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_tx_sync(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN;

    start();
    clocks_on();
    loop_tx_rx(0, -1, 20);
    loop_tx_rx(1, -1, 22);
    wr(RAM(TX_BLOCK(0)), word(entry(1, 10), entry(0, 0)));
    wr(RAM(TX_BLOCK(1)), word(entry(1, 20), entry(0, 0)));
    wr(TX_SIM, TX_SIM_EN | 0x3);
    tx_setup(0, conf);
    tx_setup(1, conf);
    wr(REF_CNT_RST, 0x3);
    step(1000);
    wr(TX_CONF0(0), conf | TX_START);
    step(5000);
    g_assert_cmpint(pad_level(20), ==, 0);
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 0);
    wr(TX_CONF0(1), conf | TX_START);
    step(100);
    g_assert_cmpint(pad_level(20), ==, 1);
    g_assert_cmpint(pad_level(22), ==, 1);
    step(1000);
    g_assert_cmpint(pad_level(20), ==, 0);
    g_assert_cmpint(pad_level(22), ==, 1);
    step(1000);
    g_assert_cmpint(pad_level(22), ==, 0);
    stop();
}

/*
 * Wrap mode with TX_LIM: the transmitter raises TX_THR_EVENT every 24
 * words and wraps; refilled half by half, as ESP-IDF's ping-pong does, it
 * sends more than its block holds.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_tx_wrap(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_WRAP;
    int sent = 0, thr = 0;

    start();
    clocks_on();
    loop_tx_rx(0, 0, 20);
    for (int w = 0; w < 48; w++) {
        wr(RAM(TX_BLOCK(0) + w), word(entry(1, 1), entry(0, 1)));
    }
    sent = 48;
    wr(TX_LIM(0), 24);
    tx_setup(0, conf);
    rx_setup(0, RX_DIV(4) | RX_IDLE(20) | RX_MEM_SIZE(1) | 0, RX_WRAP);
    wr(RX_LIM(0), 24);
    wr(RX_CONF1(0), RX_OWNER | RX_EN | RX_WRAP | RX_UPDATE);
    tx_go(0, conf);
    /* Each word takes 200 ns: refill the half just sent at each event. */
    while (!(rd(INT_RAW) & INT_TX_END(0))) {
        g_assert_cmpint(thr, <=, 5);
        step(100);
        if (rd(INT_RAW) & INT_TX_THR(0)) {
            int half = (thr % 2) * 24;

            wr(INT_CLR, INT_TX_THR(0));
            thr++;
            for (int w = 0; w < 24; w++) {
                uint32_t v = sent < 120 ? word(entry(1, 1), entry(0, 1))
                                        : 0;

                wr(RAM(TX_BLOCK(0) + half + w), v);
                sent++;
            }
        }
    }
    /* 120 words sent, the end marker in word 24 on the third lap. */
    g_assert_cmpint(thr, ==, 5);
    g_assert_cmphex(rd(INT_RAW) & INT_TX_ERR(0), ==, 0);
    step(100000);
    g_assert_cmphex(rd(INT_RAW) & INT_RX_THR(0), ==, INT_RX_THR(0));
    /* The frame, wrapped, holds 120 words and ends in word 24. */
    g_assert_cmphex(rd(RX_STATUS(0)) & 0x3ff, ==, RX_BLOCK(0) + 24);
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 23)), ==,
                    word(entry(1, 1), entry(0, 0)));
    stop();
}

/*
 * Errors: a transmission running off its memory without wrap mode
 * (MEM_EMPTY, ERR, no TX_END), a receiver writing past its memory
 * (MEM_FULL, ERR) and without owning it (MEM_OWNER_ERR).
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_errors(void)
{
    uint32_t conf = TX_DIV(1) | TX_MEM_SIZE(1) | TX_IDLE_EN;
    uint32_t conf2 = TX_DIV(1) | TX_MEM_SIZE(2) | TX_IDLE_EN;

    start();
    clocks_on();
    loop_tx_rx(2, -1, 21);
    for (int w = 0; w < 48; w++) {
        wr(RAM(TX_BLOCK(2) + w), word(entry(1, 1), entry(0, 1)));
    }
    tx_setup(2, conf);
    tx_go(2, conf);
    step(10000);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_ERR(2));
    g_assert_cmphex(rd(TX_STATUS(2)) & ST_TX_EMPTY, ==, ST_TX_EMPTY);
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(2))), ==, 0);
    wr(TX_CONF0(2), conf | TX_MEM_RD_RST);
    g_assert_cmphex(rd(TX_STATUS(2)) & ST_TX_EMPTY, ==, 0);

    /* 60 words into a 48-word memory: the rest is dropped. */
    wr(INT_CLR, 0xffffffff);
    loop_tx_rx(0, 2, 20);
    for (int w = 0; w < 60; w++) {
        wr(RAM(TX_BLOCK(0) + w), word(entry(1, 1), entry(0, 1)));
    }
    wr(RAM(TX_BLOCK(0) + 60), word(entry(1, 1), entry(0, 0)));
    tx_setup(0, conf2);
    rx_setup(2, RX_DIV(1) | RX_IDLE(10) | RX_MEM_SIZE(1), 0);
    tx_go(0, conf2);
    step(10000);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_END(0) | INT_RX_ERR(2) |
                    INT_RX_END(2));
    g_assert_cmphex(rd(RX_STATUS(2)) & ST_RX_FULL, ==, ST_RX_FULL);
    g_assert_cmphex(rd(RX_STATUS(2)) & 0x3ff, ==, RX_BLOCK(2) + 48);
    g_assert_cmphex(rd(RAM(RX_BLOCK(2) + 47)), ==,
                    word(entry(1, 1), entry(0, 1)));

    /* Owned by the APB, the receiver writes nothing. */
    wr(INT_CLR, 0xffffffff);
    wr(RX_CONF1(2), RX_MEM_WR_RST);
    g_assert_cmphex(rd(RX_STATUS(2)) & ST_RX_FULL, ==, 0);
    wr(RX_CONF1(2), RX_EN | RX_UPDATE);
    tx_go(0, conf2);
    step(10000);
    g_assert_cmphex(rd(RX_STATUS(2)) & ST_RX_OWNER_ERR, ==,
                    ST_RX_OWNER_ERR);
    g_assert_cmphex(rd(INT_RAW) & INT_RX_ERR(2), ==, INT_RX_ERR(2));
    g_assert_cmphex(rd(RX_STATUS(2)) & 0x3ff, ==, RX_BLOCK(2));
    stop();
}

/*
 * The APB FIFO: words written to a TX channel's DATA register, and read
 * back through an RX channel's, each pointer stopping with an error at
 * the end of the channel's memory.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_fifo(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN;

    start();
    clocks_on();
    wr(SYS_CONF, rd(SYS_CONF) & ~FIFO_MASK);
    loop_tx_rx(1, 3, 20);
    tx_setup(1, conf);
    wr(TX_DATA(1), word(entry(1, 7), entry(0, 9)));
    wr(TX_DATA(1), word(entry(1, 11), entry(0, 0)));
    g_assert_cmphex((rd(TX_STATUS(1)) >> 11) & 0x3ff, ==, TX_BLOCK(1) + 2);
    /* Direct access is off while the FIFO is in use. */
    g_assert_cmphex(rd(RAM(TX_BLOCK(1))), ==, 0);
    rx_setup(3, RX_DIV(4) | RX_IDLE(30) | RX_MEM_SIZE(1), 0);
    tx_go(1, conf);
    step(20000);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_END(1) | INT_RX_END(3));
    g_assert_cmphex(rd(RX_DATA(3)), ==, word(entry(1, 7), entry(0, 9)));
    g_assert_cmphex(rd(RX_DATA(3)), ==, word(entry(1, 11), entry(0, 0)));
    g_assert_cmphex((rd(RX_STATUS(3)) >> 11) & 0x3ff, ==, RX_BLOCK(3) + 2);
    for (int w = 2; w < 48; w++) {
        rd(RX_DATA(3));
    }
    g_assert_cmphex(rd(RX_STATUS(3)) & ST_RX_RD_ERR, ==, 0);
    g_assert_cmphex(rd(RX_DATA(3)), ==, 0);
    g_assert_cmphex(rd(RX_STATUS(3)) & ST_RX_RD_ERR, ==, ST_RX_RD_ERR);
    wr(RX_CONF1(3), RX_OWNER | RX_APB_MEM_RST);
    g_assert_cmphex(rd(RX_STATUS(3)) & ST_RX_RD_ERR, ==, 0);
    g_assert_cmphex(rd(RX_DATA(3)), ==, word(entry(1, 7), entry(0, 9)));
    for (int w = 2; w < 48; w++) {
        wr(TX_DATA(1), 0);
    }
    g_assert_cmphex(rd(TX_STATUS(1)) & ST_TX_WR_ERR, ==, 0);
    wr(TX_DATA(1), 0);
    g_assert_cmphex(rd(TX_STATUS(1)) & ST_TX_WR_ERR, ==, ST_TX_WR_ERR);
    stop();
}

/*
 * A larger memory: MEM_SIZE 2 gives TX channel 0 blocks 0 and 1; the
 * receiver's idle threshold in the same channel ticks.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_mem_size_and_power(void)
{
    uint32_t conf = TX_DIV(2) | TX_MEM_SIZE(2) | TX_IDLE_EN;

    start();
    clocks_on();
    loop_tx_rx(0, 0, 20);
    for (int w = 0; w < 60; w++) {
        wr(RAM(TX_BLOCK(0) + w), word(entry(1, 2), entry(0, 2)));
    }
    wr(RAM(TX_BLOCK(0) + 60), word(entry(1, 2), entry(0, 0)));
    tx_setup(0, conf);
    rx_setup(0, RX_DIV(2) | RX_IDLE(10) | RX_MEM_SIZE(2), 0);
    tx_go(0, conf);
    step(100000);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_END(0) | INT_RX_END(0));
    g_assert_cmphex(rd(RX_STATUS(0)) & 0x3ff, ==, RX_BLOCK(0) + 61);
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 59)), ==,
                    word(entry(1, 2), entry(0, 2)));
    g_assert_cmphex(rd(RAM(RX_BLOCK(0) + 60)), ==,
                    word(entry(1, 2), entry(0, 0)));

    /* Powered down, the RAM loses its contents and reads 0. */
    wr(SYS_CONF, rd(SYS_CONF) | MEM_FORCE_PD);
    g_assert_cmphex(rd(RAM(RX_BLOCK(0))), ==, 0);
    wr(SYS_CONF, rd(SYS_CONF) & ~MEM_FORCE_PD);
    g_assert_cmphex(rd(RAM(RX_BLOCK(0))), ==, 0);
    g_assert_cmphex(rd(RAM(TX_BLOCK(0))), ==, 0);
    stop();
}

/* The level of a carrier of 3 + 5 cycles, started high at t0, at t */
static int carrier_level(int64_t t0, int64_t t)
{
    return (t - t0) / CYC % 8 < 3;
}

static int pad30_rose(void)
{
    return (rd(GPIO_STATUS) >> 30) & 1;
}

/*
 * TX channel 0 idling low on pad 30, which nothing watches, its low level
 * carried at 5 MHz with CARRIER_EFF_EN clear: ten million carrier edges a
 * second, which the block takes in closed form. GPIO_IN reads the exact
 * level a nanosecond either side of edges a second on; a transmission
 * then starts on its tick, its high entry plain, and the carrier starts
 * afresh, high, after it; with pad 30's rising-edge interrupt set, the
 * pad watched, GPIO_STATUS latches the next carrier rise at its
 * nanosecond.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_unwatched_carrier(void)
{
    uint32_t conf = TX_DIV(1) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_CARRIER_EN;
    int64_t t0, s0, e;

    start();
    clocks_on();
    pad_gpio(30);
    wr(GPIO_FUNC_OUT(30), SIG_RMT(0));
    wr(RAM(TX_BLOCK(0)), word(entry(1, 100), entry(0, 0)));
    wr(TX_CARRIER(0), (3 << 16) | 5);
    step(1000);
    t0 = now;
    tx_setup(0, conf);

    for (e = t0 + 1000 * 1000 * 1000 + 3 * CYC;
         e <= t0 + 1000 * 1000 * 1000 + 8 * CYC; e += 5 * CYC) {
        clock_to(e - 1);
        g_assert_cmpint(pad_level(30), ==, carrier_level(t0, e - 1));
        clock_to(e);
        g_assert_cmpint(pad_level(30), ==, carrier_level(t0, e));
        g_assert_cmpint(carrier_level(t0, e), !=, carrier_level(t0, e - 1));
    }

    /* Mid-way through a carrier low phase, the transmission starts */
    s0 = now + 4 * CYC;
    clock_to(s0);
    g_assert_cmpint(pad_level(30), ==, 0);
    tx_go(0, conf);
    clock_to(s0 + CYC - 1);
    g_assert_cmpint(pad_level(30), ==, 0);
    clock_to(s0 + CYC);
    g_assert_cmpint(pad_level(30), ==, 1);
    clock_to(s0 + 101 * CYC + 3 * CYC - 1);
    g_assert_cmpint(pad_level(30), ==, 1);
    clock_to(s0 + 101 * CYC + 3 * CYC);
    g_assert_cmpint(pad_level(30), ==, 0);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_END(0));

    /* Watched: the carrier's second rise after the entry, at its moment */
    wr(GPIO_PIN(30), PIN_INT_RISING);
    e = s0 + 101 * CYC + 16 * CYC;
    clock_to(e - 1);
    wr(GPIO_STATUS_W1TC, 1u << 30);
    g_assert_cmpint(pad30_rose(), ==, 0);
    clock_to(e);
    g_assert_cmpint(pad30_rose(), ==, 1);
    stop();
}

/*
 * A continuous transmission on pad 30, which nothing watches, of passes of
 * three 25 ns entries, high, low, high: some 13 million passes a second,
 * which the block takes whole. GPIO_IN reads each entry's level at its
 * nanosecond a second on, and TX_STATUS the read pointer.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_unwatched_passes(void)
{
    uint32_t conf = TX_DIV(1) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_CONTI;
    int64_t s0, p;

    start();
    clocks_on();
    pad_gpio(30);
    wr(GPIO_FUNC_OUT(30), SIG_RMT(0));
    wr(RAM(TX_BLOCK(0) + 0), word(entry(1, 1), entry(0, 1)));
    wr(RAM(TX_BLOCK(0) + 1), word(entry(1, 1), entry(0, 0)));
    tx_setup(0, conf);
    step(1000);
    s0 = now;
    tx_go(0, conf);
    /* Pass k starts at s0 + CYC + 3 * k * CYC */
    p = s0 + CYC + 3 * CYC * (1000 * 1000 * 1000 / (3 * CYC));
    clock_to(p - 1);
    g_assert_cmpint(pad_level(30), ==, 1);
    clock_to(p);
    g_assert_cmpint(pad_level(30), ==, 1);
    g_assert_cmphex(rd(TX_STATUS(0)) & 0x3ff, ==, TX_BLOCK(0) + 1);
    clock_to(p + CYC - 1);
    g_assert_cmpint(pad_level(30), ==, 1);
    clock_to(p + CYC);
    g_assert_cmpint(pad_level(30), ==, 0);
    clock_to(p + 2 * CYC - 1);
    g_assert_cmpint(pad_level(30), ==, 0);
    clock_to(p + 2 * CYC);
    g_assert_cmpint(pad_level(30), ==, 1);
    g_assert_cmphex(rd(TX_STATUS(0)) & 0x3ff, ==, TX_BLOCK(0) + 2);
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 1);
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_END(0));
    stop();
}

/*
 * A loop transmission of 1000 passes on pad 30, which nothing watches,
 * TX_LOOP enabled: with nothing read meanwhile, TX_LOOP reaches the
 * interrupt line at the 1000th end marker to the nanosecond, and the
 * transmitter stops there. Without LOOP_STOP_EN it runs on, the count
 * starting again: TX_LOOP comes again 1000 passes later.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_loop_moment(void)
{
    uint32_t conf = TX_DIV(4) | TX_MEM_SIZE(1) | TX_IDLE_EN | TX_CONTI;
    int64_t s0, e;

    start();
    clocks_on();
    qtest_irq_intercept_out_named(qts, "/machine/rmt", "sysbus-irq");
    pad_gpio(30);
    wr(GPIO_FUNC_OUT(30), SIG_RMT(0));
    wr(RAM(TX_BLOCK(0) + 0), word(entry(1, 30), entry(0, 20)));
    wr(RAM(TX_BLOCK(0) + 1), word(entry(1, 10), entry(0, 0)));
    wr(TX_LIM(0), 0x80 | LOOP_NUM(1000) | LOOP_CNT_EN | LOOP_STOP_EN |
       LOOP_CNT_RESET);
    wr(TX_LIM(0), 0x80 | LOOP_NUM(1000) | LOOP_CNT_EN | LOOP_STOP_EN);
    tx_setup(0, conf);
    wr(INT_ENA, INT_TX_LOOP(0));
    step(1000);
    s0 = now;
    tx_go(0, conf);
    /* Passes of 60 ticks of 100 ns from s0 + 100 */
    e = s0 + 100 + 1000 * 6000;
    clock_to(e - 1);
    g_assert_false(qtest_get_irq(qts, 0));
    clock_to(e);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(INT_RAW), ==, INT_TX_LOOP(0) | INT_TX_END(0));
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 0);
    g_assert_cmpint(pad_level(30), ==, 0);

    /* Looping on */
    wr(INT_CLR, 0xffffffff);
    wr(TX_LIM(0), 0x80 | LOOP_NUM(1000) | LOOP_CNT_EN);
    tx_setup(0, conf);
    s0 = now;
    tx_go(0, conf);
    e = s0 + 100 + 1000 * 6000;
    clock_to(e - 1);
    g_assert_false(qtest_get_irq(qts, 0));
    clock_to(e);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(INT_CLR, INT_TX_LOOP(0));
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmpuint(ST_STATE(rd(TX_STATUS(0))), ==, 1);
    clock_to(e + 1000 * 6000 - 1);
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmpint(pad_level(30), ==, 1);
    clock_to(e + 1000 * 6000);
    g_assert_true(qtest_get_irq(qts, 0));
    stop();
}

/* Held in reset, the block is back at its reset values. */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt/test] */
static void test_block_reset(void)
{
    start();
    clocks_on();
    wr(TX_CONF0(0), 0x123400);
    wr(INT_ENA, 0xff);
    wr(HP_RST_EN1, RST_EN_RMT);
    wr(HP_RST_EN1, 0);
    g_assert_cmphex(rd(TX_CONF0(0)), ==, 0x00710200);
    g_assert_cmphex(rd(INT_ENA), ==, 0);
    g_assert_cmphex(rd(SYS_CONF), ==, 0x05000010);
    stop();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/rmt/reset_values", test_reset_values);
    qtest_add_func("esp32p4/rmt/tx_rx_loopback", test_tx_rx_loopback);
    qtest_add_func("esp32p4/rmt/conf_update", test_conf_update);
    qtest_add_func("esp32p4/rmt/carrier", test_carrier);
    qtest_add_func("esp32p4/rmt/rx_filter", test_rx_filter);
    qtest_add_func("esp32p4/rmt/loop_count", test_loop_count);
    qtest_add_func("esp32p4/rmt/tx_sync", test_tx_sync);
    qtest_add_func("esp32p4/rmt/tx_wrap", test_tx_wrap);
    qtest_add_func("esp32p4/rmt/errors", test_errors);
    qtest_add_func("esp32p4/rmt/fifo", test_fifo);
    qtest_add_func("esp32p4/rmt/mem_size_and_power",
                   test_mem_size_and_power);
    qtest_add_func("esp32p4/rmt/block_reset", test_block_reset);
    qtest_add_func("esp32p4/rmt/unwatched_carrier", test_unwatched_carrier);
    qtest_add_func("esp32p4/rmt/unwatched_passes", test_unwatched_passes);
    qtest_add_func("esp32p4/rmt/loop_moment", test_loop_moment);
    return g_test_run();
}
