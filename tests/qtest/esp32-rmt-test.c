/*
 * QTest testcase for the ESP32 RMT
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN  0x3ff000c0
#define DPORT_PERIP_RST_EN  0x3ff000c4
#define PERIP_RMT           (1u << 9)

#define RMT                 0x3ff56000
#define DATA(n)             (0x00 + 4 * (n))
#define CONF0(n)            (0x20 + 8 * (n))
#define CONF1(n)            (0x24 + 8 * (n))
#define STATUS(n)           (0x60 + 4 * (n))
#define APB_MEM_ADDR(n)     (0x80 + 4 * (n))
#define INT_RAW             0xa0
#define INT_ST              0xa4
#define INT_ENA             0xa8
#define INT_CLR             0xac
#define CARRIER_DUTY(n)     (0xb0 + 4 * (n))
#define TX_LIM(n)           (0xd0 + 4 * (n))
#define APB_CONF            0xf0
#define DATE                0xfc
#define RAM(ch, i)          (RMT + 0x800 + 4 * ((ch) * 64 + (i)))

#define DIV(d)              (d)
#define IDLE_THRES(t)       ((t) << 8)
#define MEM_SIZE(m)         ((m) << 24)
#define CARRIER_EN          (1u << 28)
#define CARRIER_OUT_LV      (1u << 29)
#define MEM_PD              (1u << 30)

#define TX_START            (1u << 0)
#define RX_EN               (1u << 1)
#define MEM_WR_RST          (1u << 2)
#define MEM_RD_RST          (1u << 3)
#define APB_MEM_RST         (1u << 4)
#define MEM_OWNER_RX        (1u << 5)
#define TX_CONTI_MODE       (1u << 6)
#define RX_FILTER_EN        (1u << 7)
#define FILTER_THRES(t)     ((t) << 8)
#define REF_CNT_RST         (1u << 16)
#define REF_ALWAYS_ON       (1u << 17)
#define IDLE_OUT_LV         (1u << 18)
#define IDLE_OUT_EN         (1u << 19)

#define ST_OWNER_ERR        (1u << 27)
#define ST_MEM_FULL         (1u << 28)
#define ST_MEM_EMPTY        (1u << 29)
#define ST_APB_WR_ERR       (1u << 30)
#define ST_APB_RD_ERR       (1u << 31)
#define ST_STATE(s)         (((s) >> 24) & 7)
#define ST_WADDR(s)         ((s) & 0x3ff)
#define ST_RADDR(s)         (((s) >> 12) & 0x3ff)

#define INT_TX_END(n)       (1u << (3 * (n)))
#define INT_RX_END(n)       (1u << (3 * (n) + 1))
#define INT_ERR(n)          (1u << (3 * (n) + 2))
#define INT_TX_THR(n)       (1u << (24 + (n)))

#define FIFO_MASK           (1u << 0)
#define MEM_TX_WRAP_EN      (1u << 1)

#define GPIO                0x3ff44000
#define GPIO_IN             (GPIO + 0x3c)
#define GPIO_FUNC_IN(s)     (GPIO + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x530 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO18       0x3ff49070
#define IO_MUX_GPIO19       0x3ff49074
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (2u << 12)
#define SIG_RMT_IN(n)       (83 + (n))
#define SIG_RMT_OUT(n)      (87 + (n))

#define GPIO_PATH           "/machine/soc/gpio"
#define PAD_IN              "esp32-gpio-pad-in"
#define PAD_OUT             "esp32-gpio-pad"

/* Entries: level and period in channel clock ticks */
#define E(l, p)             (((l) ? 0x8000u : 0) | (p))
#define W(e0, e1)           ((e0) | ((e1) << 16))

/* APB_CLK runs from the 40 MHz crystal after reset: 25 ns a cycle */
#define APB_NS              25
#define US                  1000

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_RMT);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) & ~PERIP_RMT);
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, RMT + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, RMT + off, v);
}

/*
 * Channel ch on APB_CLK, its divider restarted: its next tick is a full
 * channel clock period away.
 */
static void chan_clock(QTestState *qts, unsigned ch, uint32_t conf0,
                       uint32_t conf1)
{
    wr(qts, CONF0(ch), conf0);
    wr(qts, CONF1(ch), conf1 | REF_ALWAYS_ON | REF_CNT_RST);
}

/* RMT_SIG_OUT0 out on GPIO18; GPIO18 read back as RMT_SIG_IN1 */
static void loopback(QTestState *qts)
{
    qtest_writel(qts, IO_MUX_GPIO18, MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(18), SIG_RMT_OUT(0));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN(1)), IN_SIG_IN_SEL | 18);
}

static bool pad18(QTestState *qts)
{
    return (qtest_readl(qts, GPIO_IN) >> 18) & 1;
}

/* [spec:nuos:req:emu.esp32.rmt/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    for (unsigned n = 0; n < 8; n++) {
        g_assert_cmphex(rd(qts, CONF0(n)), ==, 0x31100002);
        g_assert_cmphex(rd(qts, CONF1(n)), ==, 0x00000f20);
        g_assert_cmphex(rd(qts, STATUS(n)), ==, (n * 64) | (n * 64) << 12);
        g_assert_cmphex(rd(qts, APB_MEM_ADDR(n)), ==, n * 64);
        g_assert_cmphex(rd(qts, CARRIER_DUTY(n)), ==, 0x00400040);
        g_assert_cmphex(rd(qts, TX_LIM(n)), ==, 0x80);
    }
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0);
    g_assert_cmphex(rd(qts, APB_CONF), ==, 0);
    g_assert_cmphex(rd(qts, DATE), ==, 0x16022600);

    /* Access types and field masks */
    wr(qts, CONF0(0), 0xffffffff);
    g_assert_cmphex(rd(qts, CONF0(0)), ==, 0xffffffff);
    wr(qts, CONF0(0), 0x31100002);
    wr(qts, CONF0(1), 0xffffffff);
    g_assert_cmphex(rd(qts, CONF0(1)), ==, 0x3fffffff);
    wr(qts, CONF1(2), 0xffffffff & ~(TX_START | RX_EN));
    g_assert_cmphex(rd(qts, CONF1(2)), ==, 0x000efffe & ~RX_EN);
    wr(qts, TX_LIM(3), 0xffffffff);
    g_assert_cmphex(rd(qts, TX_LIM(3)), ==, 0x1ff);
    wr(qts, APB_CONF, 0xffffffff);
    g_assert_cmphex(rd(qts, APB_CONF), ==, 3);
    wr(qts, INT_RAW, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    wr(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_CLR), ==, 0);
    wr(qts, DATE, 0x12345678);
    g_assert_cmphex(rd(qts, DATE), ==, 0x12345678);

    /* The DPORT reset bit resets the block */
    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_RMT);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    g_assert_cmphex(rd(qts, DATE), ==, 0x16022600);
    g_assert_cmphex(rd(qts, CONF1(2)), ==, 0x00000f20);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rmt/test] */
static void test_gated(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    /* The RMT's clock is off after reset: writes do not land */
    wr(qts, TX_LIM(0), 0x10);
    g_assert_cmphex(rd(qts, TX_LIM(0)), ==, 0x80);
    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_RMT);
    wr(qts, TX_LIM(0), 0x10);
    g_assert_cmphex(rd(qts, TX_LIM(0)), ==, 0x10);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rmt/test] */
static void test_ram_access(void)
{
    QTestState *qts = start();

    /* Direct RAM access needs FIFO_MASK */
    qtest_writel(qts, RAM(0, 0), 0x11223344);
    g_assert_cmphex(qtest_readl(qts, RAM(0, 0)), ==, 0);
    wr(qts, APB_CONF, FIFO_MASK);
    qtest_writel(qts, RAM(0, 0), 0x11223344);
    qtest_writeb(qts, RAM(0, 0) + 1, 0xaa);
    g_assert_cmphex(qtest_readl(qts, RAM(0, 0)), ==, 0x1122aa44);
    g_assert_cmphex(qtest_readw(qts, RAM(0, 0) + 2), ==, 0x1122);
    qtest_writel(qts, RAM(7, 63), 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, RAM(7, 63)), ==, 0xcafef00d);
    /* ... and keeps the FIFO registers out of use */
    wr(qts, DATA(1), 0x5555);
    g_assert_cmphex(qtest_readl(qts, RAM(1, 0)), ==, 0);

    /* Through channel 1's FIFO: one pointer, reads and writes */
    wr(qts, APB_CONF, 0);
    wr(qts, DATA(1), 0xa1);
    wr(qts, DATA(1), 0xa2);
    g_assert_cmphex(rd(qts, APB_MEM_ADDR(1)), ==, 64 + 2);
    wr(qts, CONF1(1), 0xf20 | APB_MEM_RST);
    wr(qts, CONF1(1), 0xf20);
    g_assert_cmphex(rd(qts, APB_MEM_ADDR(1)), ==, 64);
    g_assert_cmphex(rd(qts, DATA(1)), ==, 0xa1);
    g_assert_cmphex(rd(qts, DATA(1)), ==, 0xa2);
    wr(qts, APB_CONF, FIFO_MASK);
    g_assert_cmphex(qtest_readl(qts, RAM(1, 1)), ==, 0xa2);

    /* The FIFO stops at the end of the channel's memory */
    wr(qts, APB_CONF, 0);
    wr(qts, CONF1(1), 0xf20 | APB_MEM_RST);
    wr(qts, CONF1(1), 0xf20);
    for (unsigned i = 0; i < 64; i++) {
        wr(qts, DATA(1), i);
    }
    g_assert_cmphex(rd(qts, STATUS(1)) & ST_APB_WR_ERR, ==, 0);
    wr(qts, DATA(1), 0xbad);
    g_assert_cmphex(rd(qts, STATUS(1)) & ST_APB_WR_ERR, ==, ST_APB_WR_ERR);
    g_assert_cmphex(rd(qts, DATA(1)), ==, 0);
    g_assert_cmphex(rd(qts, STATUS(1)) & ST_APB_RD_ERR, ==, ST_APB_RD_ERR);
    wr(qts, APB_CONF, FIFO_MASK);
    g_assert_cmphex(qtest_readl(qts, RAM(2, 0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RAM(1, 63)), ==, 63);

    /* MEM_PD powers the RAM down and loses its contents */
    wr(qts, CONF0(0), 0x31100002 | MEM_PD);
    g_assert_cmphex(qtest_readl(qts, RAM(1, 63)), ==, 0);
    wr(qts, CONF0(0), 0x31100002);
    g_assert_cmphex(qtest_readl(qts, RAM(1, 63)), ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * Channel 0 sends high 10, low 5, high 3 ticks of 1 us and ends low; its
 * output reaches GPIO18 through the matrix.
 */
static void test_tx_timing(void)
{
    QTestState *qts = start();

    loopback(qts);
    wr(qts, APB_CONF, FIFO_MASK);
    qtest_writel(qts, RAM(0, 0), W(E(1, 10), E(0, 5)));
    qtest_writel(qts, RAM(0, 1), W(E(1, 3), E(0, 0)));
    chan_clock(qts, 0, DIV(40) | MEM_SIZE(1) | IDLE_THRES(0x1000), 0);
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);
    g_assert_cmphex(rd(qts, CONF1(0)) & TX_START, ==, TX_START);
    g_assert_cmpuint(ST_STATE(rd(qts, STATUS(0))), ==, 1);

    /* The first tick comes 1 us after the divider restarted */
    qtest_clock_step(qts, US / 2);
    g_assert_false(pad18(qts));
    qtest_clock_step(qts, US);
    g_assert_true(pad18(qts));
    qtest_clock_step(qts, 9 * US);
    g_assert_true(pad18(qts));
    qtest_clock_step(qts, US);
    g_assert_false(pad18(qts));
    qtest_clock_step(qts, 4 * US);
    g_assert_false(pad18(qts));
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    qtest_clock_step(qts, US);
    g_assert_true(pad18(qts));
    g_assert_cmpuint(ST_RADDR(rd(qts, STATUS(0))), ==, 1);
    qtest_clock_step(qts, 2 * US);
    g_assert_true(pad18(qts));
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    qtest_clock_step(qts, US);
    g_assert_false(pad18(qts));
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0));
    g_assert_cmphex(rd(qts, CONF1(0)) & TX_START, ==, 0);
    g_assert_cmpuint(ST_STATE(rd(qts, STATUS(0))), ==, 0);
    g_assert_cmpuint(ST_RADDR(rd(qts, STATUS(0))), ==, 0);
    wr(qts, INT_CLR, INT_TX_END(0));
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);

    /* The idle level: the end marker's, or IDLE_OUT_LV */
    qtest_writel(qts, RAM(0, 1), W(E(1, 3), E(1, 0)));
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);
    qtest_clock_step(qts, 30 * US);
    g_assert_true(pad18(qts));
    wr(qts, CONF1(0), REF_ALWAYS_ON | IDLE_OUT_EN);
    g_assert_false(pad18(qts));
    wr(qts, CONF1(0), REF_ALWAYS_ON | IDLE_OUT_EN | IDLE_OUT_LV);
    g_assert_true(pad18(qts));
    qtest_quit(qts);
}

/* Drive GPIO19 into RMT_SIG_IN1 */
static void drive19(QTestState *qts, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 19, level);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * Channel 1 receives pulses driven on GPIO19, ends the frame after the
 * idle threshold, and writes its entries and the end marker.
 */
static void test_rx(void)
{
    QTestState *qts = start();
    uint32_t st;

    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN(1)), IN_SIG_IN_SEL | 19);
    drive19(qts, 0);
    wr(qts, APB_CONF, FIFO_MASK);
    chan_clock(qts, 1, DIV(40) | MEM_SIZE(1) | IDLE_THRES(20),
               MEM_OWNER_RX);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | MEM_WR_RST);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN);

    qtest_clock_step(qts, US / 2);
    drive19(qts, 1);
    qtest_clock_step(qts, 7 * US);
    drive19(qts, 0);
    g_assert_cmpuint(ST_STATE(rd(qts, STATUS(1))), ==, 3);
    qtest_clock_step(qts, 5 * US);
    drive19(qts, 1);
    /* A pulse that spans no tick is not seen */
    qtest_clock_step(qts, 100);
    drive19(qts, 0);
    qtest_clock_step(qts, 100);
    drive19(qts, 1);
    qtest_clock_step(qts, 3 * US - 200);
    drive19(qts, 0);
    qtest_clock_step(qts, 20 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RX_END(1));

    g_assert_cmphex(qtest_readl(qts, RAM(1, 0)), ==, W(E(1, 7), E(0, 5)));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 1)), ==, W(E(1, 3), E(0, 0)));
    st = rd(qts, STATUS(1));
    g_assert_cmpuint(ST_WADDR(st), ==, 64 + 2);
    g_assert_cmpuint(ST_STATE(st), ==, 0);
    g_assert_cmphex(st & (ST_OWNER_ERR | ST_MEM_FULL), ==, 0);

    /* With the memory given to software, the receiver reports an error */
    wr(qts, INT_CLR, ~0u);
    wr(qts, CONF1(1), REF_ALWAYS_ON | RX_EN);
    drive19(qts, 1);
    qtest_clock_step(qts, 5 * US);
    drive19(qts, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_ERR(1));
    g_assert_cmphex(rd(qts, STATUS(1)) & ST_OWNER_ERR, ==, ST_OWNER_ERR);
    g_assert_cmpuint(ST_WADDR(rd(qts, STATUS(1))), ==, 64 + 2);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_WR_RST);
    g_assert_cmphex(rd(qts, STATUS(1)), ==, 64 | (64 << 12));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * The filter drops pulses shorter than FILTER_THRES APB cycles and passes
 * longer ones.
 */
static void test_rx_filter(void)
{
    QTestState *qts = start();

    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_RMT_IN(1)), IN_SIG_IN_SEL | 19);
    drive19(qts, 0);
    wr(qts, APB_CONF, FIFO_MASK);
    /* 100 ns ticks; pulses under 80 APB cycles (2 us) are dropped */
    chan_clock(qts, 1, DIV(4) | MEM_SIZE(1) | IDLE_THRES(100),
               MEM_OWNER_RX);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_FILTER_EN |
       FILTER_THRES(80) | RX_EN);

    qtest_clock_step(qts, 50);
    drive19(qts, 1);
    qtest_clock_step(qts, 1500);
    drive19(qts, 0);
    qtest_clock_step(qts, 30 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmpuint(ST_STATE(rd(qts, STATUS(1))), ==, 0);

    drive19(qts, 1);
    qtest_clock_step(qts, 5 * US);
    drive19(qts, 0);
    qtest_clock_step(qts, 30 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RX_END(1));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 0)), ==, W(E(1, 50), E(0, 0)));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * Channel 0's output received by channel 1 through the GPIO matrix: the
 * received entries are the sent ones, with the TX_END and RX_END
 * interrupts on the RMT's interrupt line.
 */
static void test_loopback(void)
{
    QTestState *qts = start();
    static const uint32_t sent[] = {
        W(E(1, 20), E(0, 7)),
        W(E(1, 1), E(0, 1)),
        W(E(1, 300), E(0, 45)),
        W(E(1, 2), E(0, 0)),
    };

    qtest_irq_intercept_out_named(qts, "/machine/soc/rmt", "sysbus-irq");
    loopback(qts);
    wr(qts, APB_CONF, FIFO_MASK);
    for (unsigned i = 0; i < ARRAY_SIZE(sent); i++) {
        qtest_writel(qts, RAM(0, i), sent[i]);
    }
    /* Both channels on 200 ns ticks, their dividers out of phase */
    chan_clock(qts, 1, DIV(8) | MEM_SIZE(1) | IDLE_THRES(310),
               MEM_OWNER_RX);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN);
    qtest_clock_step(qts, 130);
    chan_clock(qts, 0, DIV(8) | MEM_SIZE(1), 0);
    wr(qts, INT_ENA, INT_TX_END(0) | INT_RX_END(1));
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_clock_step(qts, 400 * 200);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0));
    g_assert_true(qtest_get_irq(qts, 0));
    wr(qts, INT_CLR, INT_TX_END(0));
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 300 * 200);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_RX_END(1));
    g_assert_cmphex(rd(qts, INT_ST), ==, INT_RX_END(1));
    g_assert_true(qtest_get_irq(qts, 0));
    for (unsigned i = 0; i < ARRAY_SIZE(sent); i++) {
        g_assert_cmphex(qtest_readl(qts, RAM(1, i)), ==, sent[i]);
    }
    wr(qts, INT_ENA, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * A carrier of 10 + 10 APB cycles on the high level, received in 125 ns
 * ticks as its pulses.
 */
static void test_carrier(void)
{
    QTestState *qts = start();

    loopback(qts);
    wr(qts, APB_CONF, FIFO_MASK);
    qtest_writel(qts, RAM(0, 0), W(E(1, 3), E(0, 2)));
    qtest_writel(qts, RAM(0, 1), W(E(1, 0), 0));
    wr(qts, CARRIER_DUTY(0), (10 << 16) | 10);
    chan_clock(qts, 1, DIV(5) | MEM_SIZE(1) | IDLE_THRES(100),
               MEM_OWNER_RX);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN);
    /* 1 us ticks: high for 3 us is 6 carrier periods of 500 ns */
    chan_clock(qts, 0, DIV(40) | MEM_SIZE(1) | CARRIER_EN | CARRIER_OUT_LV,
               0);
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);
    qtest_clock_step(qts, 30 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0) | INT_RX_END(1));
    for (unsigned i = 0; i < 5; i++) {
        g_assert_cmphex(qtest_readl(qts, RAM(1, i)), ==,
                        W(E(1, 2), E(0, 2)));
    }
    /* The last carrier low runs into the data's low */
    g_assert_cmphex(qtest_readl(qts, RAM(1, 5)), ==, W(E(1, 2), E(0, 18)));

    /*
     * Carried on the low level instead: the high level is sent plain and
     * the carrier's first high phase lengthens it. The line idles low.
     */
    wr(qts, CONF1(0), REF_ALWAYS_ON | IDLE_OUT_EN);
    qtest_clock_step(qts, 20 * US);
    wr(qts, INT_CLR, ~0u);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN | MEM_WR_RST);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN);
    wr(qts, CONF0(0), DIV(40) | MEM_SIZE(1) | CARRIER_EN);
    wr(qts, CONF1(0), REF_ALWAYS_ON | IDLE_OUT_EN | TX_START);
    qtest_clock_step(qts, 30 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0) | INT_RX_END(1));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 0)), ==, W(E(1, 26), E(0, 2)));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 1)), ==, W(E(1, 2), E(0, 2)));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 2)), ==, W(E(1, 2), E(0, 2)));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 3)) & 0xffff, ==, E(1, 2));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 3)) >> 16, ==, E(0, 0));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * Wraparound with TX_LIM: the threshold interrupt comes after every 16
 * words, software replaces the sent words, and the transmitter runs
 * through the end of its memory into them.
 */
static void test_wrap_threshold(void)
{
    QTestState *qts = start();
    int thr = 0;

    wr(qts, APB_CONF, FIFO_MASK | MEM_TX_WRAP_EN);
    for (unsigned i = 0; i < 64; i++) {
        qtest_writel(qts, RAM(0, i), W(E(1, 1), E(0, 1)));
    }
    wr(qts, TX_LIM(0), 16);
    chan_clock(qts, 0, DIV(40) | MEM_SIZE(1), 0);
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);

    /* Each word takes 2 us; the 16th is fetched at 1 + 15 * 2 us */
    qtest_clock_step(qts, 30 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    qtest_clock_step(qts, 2 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_THR(0));
    wr(qts, INT_CLR, INT_TX_THR(0));
    thr++;
    /* Refill the first block with words that end the transmission */
    qtest_writel(qts, RAM(0, 0), W(E(1, 4), E(0, 0)));
    for (unsigned i = 0; i < 4; i++) {
        qtest_clock_step(qts, 32 * US);
        if (rd(qts, INT_RAW) & INT_TX_THR(0)) {
            wr(qts, INT_CLR, INT_TX_THR(0));
            thr++;
        }
    }
    g_assert_cmpint(thr, ==, 4);
    /* 128 us for the 64 words, then 4 us high before the end marker */
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0));
    g_assert_cmphex(rd(qts, STATUS(0)) & (ST_MEM_EMPTY), ==, 0);

    /* Without wraparound, the end of memory is an error */
    wr(qts, INT_CLR, ~0u);
    wr(qts, APB_CONF, FIFO_MASK);
    qtest_writel(qts, RAM(0, 0), W(E(1, 1), E(0, 1)));
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);
    qtest_clock_step(qts, 200 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & ~INT_TX_THR(0), ==, INT_ERR(0));
    g_assert_cmphex(rd(qts, STATUS(0)) & ST_MEM_EMPTY, ==, ST_MEM_EMPTY);
    g_assert_cmpuint(ST_STATE(rd(qts, STATUS(0))), ==, 0);

    /* Two blocks: channel 0 runs on through channel 1's block */
    wr(qts, INT_CLR, ~0u);
    for (unsigned i = 0; i < 10; i++) {
        qtest_writel(qts, RAM(1, i), W(E(1, 1), E(0, 1)));
    }
    qtest_writel(qts, RAM(1, 10), W(E(1, 1), E(0, 0)));
    wr(qts, CONF0(0), DIV(40) | MEM_SIZE(2));
    wr(qts, CONF1(0), REF_ALWAYS_ON | MEM_RD_RST);
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);
    qtest_clock_step(qts, 200 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & ~INT_TX_THR(0), ==, INT_TX_END(0));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * Continuous mode repeats the data with one idle tick between passes,
 * raising TX_END on each, until it is turned off.
 */
static void test_continuous(void)
{
    QTestState *qts = start();

    loopback(qts);
    wr(qts, APB_CONF, FIFO_MASK);
    qtest_writel(qts, RAM(0, 0), W(E(1, 3), E(0, 0)));
    qtest_writel(qts, RAM(1, 0), 0);
    chan_clock(qts, 1, DIV(40) | MEM_SIZE(1) | IDLE_THRES(10),
               MEM_OWNER_RX);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN);
    chan_clock(qts, 0, DIV(40) | MEM_SIZE(1), TX_CONTI_MODE);
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_CONTI_MODE | TX_START);

    qtest_clock_step(qts, 4 * US + US / 2);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0));
    wr(qts, INT_CLR, INT_TX_END(0));
    g_assert_false(pad18(qts));
    qtest_clock_step(qts, US);
    g_assert_true(pad18(qts));
    qtest_clock_step(qts, 3 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0));
    g_assert_cmphex(rd(qts, CONF1(0)) & TX_START, ==, TX_START);

    wr(qts, CONF1(0), REF_ALWAYS_ON);
    qtest_clock_step(qts, 20 * US);
    g_assert_cmphex(rd(qts, CONF1(0)) & TX_START, ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0) | INT_RX_END(1));
    /* Received: high 3, low 1, high 3, low 1, high 3, end */
    g_assert_cmphex(qtest_readl(qts, RAM(1, 0)), ==, W(E(1, 3), E(0, 1)));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 1)), ==, W(E(1, 3), E(0, 1)));
    g_assert_cmphex(qtest_readl(qts, RAM(1, 2)), ==, W(E(1, 3), E(0, 0)));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * A frame longer than the receiver's memory fills it, sets MEM_FULL and
 * raises ERR as well as RX_END.
 */
static void test_rx_full(void)
{
    QTestState *qts = start();

    loopback(qts);
    wr(qts, APB_CONF, FIFO_MASK | MEM_TX_WRAP_EN);
    for (unsigned i = 0; i < 64; i++) {
        qtest_writel(qts, RAM(0, i), W(E(1, 1), E(0, 1)));
    }
    chan_clock(qts, 1, DIV(40) | MEM_SIZE(1) | IDLE_THRES(10),
               MEM_OWNER_RX);
    wr(qts, CONF1(1), REF_ALWAYS_ON | MEM_OWNER_RX | RX_EN);
    chan_clock(qts, 0, DIV(40) | MEM_SIZE(1), 0);
    wr(qts, CONF1(0), REF_ALWAYS_ON | TX_START);

    /* 70 words sent, 64 received */
    qtest_clock_step(qts, 140 * US);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_ERR(1));
    g_assert_cmphex(rd(qts, STATUS(1)) & ST_MEM_FULL, ==, ST_MEM_FULL);
    g_assert_cmpuint(ST_WADDR(rd(qts, STATUS(1))), ==, 128);

    /* End the transmission on its next pass */
    qtest_writel(qts, RAM(0, 0), W(E(0, 0), 0));
    qtest_clock_step(qts, 160 * US);
    g_assert_cmphex(rd(qts, INT_RAW) & ~INT_TX_THR(0), ==,
                    INT_TX_END(0) | INT_RX_END(1) | INT_ERR(1));
    g_assert_cmpuint(ST_WADDR(rd(qts, STATUS(1))), ==, 128);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.rmt/test]
 * REF_TICK as the base clock: 1 MHz, so a divider of 2 makes 2 us ticks.
 */
static void test_ref_tick(void)
{
    QTestState *qts = start();

    loopback(qts);
    wr(qts, APB_CONF, FIFO_MASK);
    qtest_writel(qts, RAM(0, 0), W(E(1, 5), E(0, 0)));
    wr(qts, CONF0(0), DIV(2) | MEM_SIZE(1));
    wr(qts, CONF1(0), REF_CNT_RST);
    wr(qts, CONF1(0), TX_START);
    qtest_clock_step(qts, 2 * US + 100);
    g_assert_true(pad18(qts));
    qtest_clock_step(qts, 9 * US);
    g_assert_true(pad18(qts));
    qtest_clock_step(qts, US);
    g_assert_false(pad18(qts));
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_TX_END(0));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/rmt/reset-values", test_reset_values);
    qtest_add_func("/esp32/rmt/gated", test_gated);
    qtest_add_func("/esp32/rmt/ram-access", test_ram_access);
    qtest_add_func("/esp32/rmt/tx-timing", test_tx_timing);
    qtest_add_func("/esp32/rmt/rx", test_rx);
    qtest_add_func("/esp32/rmt/rx-filter", test_rx_filter);
    qtest_add_func("/esp32/rmt/loopback", test_loopback);
    qtest_add_func("/esp32/rmt/carrier", test_carrier);
    qtest_add_func("/esp32/rmt/wrap-threshold", test_wrap_threshold);
    qtest_add_func("/esp32/rmt/continuous", test_continuous);
    qtest_add_func("/esp32/rmt/rx-full", test_rx_full);
    qtest_add_func("/esp32/rmt/ref-tick", test_ref_tick);
    return g_test_run();
}
