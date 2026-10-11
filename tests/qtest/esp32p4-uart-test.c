/*
 * QTest testcase for the ESP32-P4 UARTs and the board's serial ports
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The UARTs' core clocks are the 40 MHz crystal out of reset (UARTn_SCLK
 * undivided): 25 ns a cycle, so CLKDIV 40 gives bits of 1 us. Frames go
 * between UARTs through the GPIO matrix, through a UART's internal
 * loopback, and to and from the serial ports' chardevs on UART0's pads.
 */

#include "qemu/osdep.h"
#include <poll.h>
#include "libqtest.h"
#include "qobject/qdict.h"

#define UART(n)             (0x500ca000 + 0x1000 * (n))
#define LP_UART             0x50121000
#define UART_PATH(n)        "/machine/uart[" #n "]"
#define LP_UART_PATH        "/machine/lp-uart"

#define FIFO                0x00
#define INT_RAW             0x04
#define INT_ST              0x08
#define INT_ENA             0x0c
#define INT_CLR             0x10
#define CLKDIV              0x14
#define RX_FILT             0x18
#define STATUS              0x1c
#define CONF0               0x20
#define CONF1               0x24
#define HWFC_CONF           0x2c
#define SLEEP_CONF0         0x30
#define SLEEP_CONF1         0x34
#define SLEEP_CONF2         0x38
#define SWFC_CONF0          0x3c
#define SWFC_CONF1          0x40
#define TXBRK_CONF          0x44
#define IDLE_CONF           0x48
#define RS485_CONF          0x4c
#define AT_CMD_PRECNT       0x50
#define AT_CMD_POSTCNT      0x54
#define AT_CMD_GAPTOUT      0x58
#define AT_CMD_CHAR         0x5c
#define MEM_CONF            0x60
#define TOUT_CONF           0x64
#define MEM_TX_STATUS       0x68
#define MEM_RX_STATUS       0x6c
#define FSM_STATUS          0x70
#define POSPULSE            0x74
#define NEGPULSE            0x78
#define LOWPULSE            0x7c
#define HIGHPULSE           0x80
#define RXD_CNT             0x84
#define CLK_CONF            0x88
#define DATE                0x8c
#define AFIFO_STATUS        0x90
#define REG_UPDATE          0x98
#define ID                  0x9c

#define INT_RXFIFO_FULL     (1u << 0)
#define INT_TXFIFO_EMPTY    (1u << 1)
#define INT_PARITY_ERR      (1u << 2)
#define INT_FRM_ERR         (1u << 3)
#define INT_RXFIFO_OVF      (1u << 4)
#define INT_DSR_CHG         (1u << 5)
#define INT_CTS_CHG         (1u << 6)
#define INT_BRK_DET         (1u << 7)
#define INT_RXFIFO_TOUT     (1u << 8)
#define INT_SW_XON          (1u << 9)
#define INT_SW_XOFF         (1u << 10)
#define INT_GLITCH_DET      (1u << 11)
#define INT_TX_BRK_DONE     (1u << 12)
#define INT_TX_BRK_IDLE     (1u << 13)
#define INT_TX_DONE         (1u << 14)
#define INT_RS485_PARITY    (1u << 15)
#define INT_RS485_FRM       (1u << 16)
#define INT_RS485_CLASH     (1u << 17)
#define INT_AT_CMD_DET      (1u << 18)
#define INT_WAKEUP          (1u << 19)

/* CONF0: 8 data bits, 1 stop bit, and the bits the tests use */
#define CONF0_8N1           0x1c
#define CONF0_PARITY_ODD    ((1u << 1) | (1u << 0))
#define CONF0_PARITY_EVEN   (1u << 1)
#define CONF0_TXD_BRK       (1u << 6)
#define CONF0_LOOPBACK      (1u << 12)
#define CONF0_TX_FLOW_EN    (1u << 13)
#define CONF0_ERR_WR_MASK   (1u << 18)
#define CONF0_AUTOBAUD_EN   (1u << 19)
#define CONF0_SW_RTS        (1u << 21)
#define CONF0_RXFIFO_RST    (1u << 22)
#define CONF0_TXFIFO_RST    (1u << 23)

#define STATUS_RXFIFO_CNT(v) ((v) & 0xff)
#define STATUS_TXFIFO_CNT(v) (((v) >> 16) & 0xff)
#define STATUS_DSRN         (1u << 13)
#define STATUS_CTSN         (1u << 14)
#define STATUS_RXD          (1u << 15)
#define STATUS_DTRN         (1u << 29)
#define STATUS_RTSN         (1u << 30)
#define STATUS_TXD          (1u << 31)

#define HWFC_RX_FLOW_EN     (1u << 8)
#define SWFC_EN             (1u << 17)
#define SWFC_XONOFF_DEL     (1u << 18)
#define SWFC_FORCE_XOFF     (1u << 20)
#define SWFC_SEND_XOFF      (1u << 22)
#define RS485_EN            (1u << 0)
#define RS485_DL0_EN        (1u << 1)
#define RS485_DL1_EN        (1u << 2)
#define RS485_TX_RX_EN      (1u << 3)
#define TOUT_EN             (1u << 0)
#define TOUT_THRHD(n)       ((n) << 2)
#define CLK_TX_SCLK_EN      (1u << 24)
#define CLK_RX_SCLK_EN      (1u << 25)
#define CLK_TX_RST_CORE     (1u << 26)
#define CLK_RX_RST_CORE     (1u << 27)

/* Bits of 1 us at the crystal's 40 MHz. */
#define DIV_1US             40
#define BIT_NS              1000
#define FRAME_NS            (10 * BIT_NS)

#define GPIO                0x500e0000
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define FUN_WPU             (1u << 8)
#define FUN_DRV(d)          ((d) << 10)
#define MCU_SEL_GPIO        (1u << 12)
#define SIG_UART1_TXD       13
#define SIG_UART1_RTS       14
#define SIG_UART2_RXD       16
#define SIG_UART2_CTS       17
#define GPIO_PATH           "/machine/gpio"

#define HP_SYS_CLKRST_PERI_CLK_CTRL111 0x500e606c
#define HP_SYS_CLKRST_PERI_CLK_CTRL112 0x500e6070
#define HP_SYS_CLKRST_HP_RST_EN1       0x500e60c4
#define RST_EN_UART1_CORE              (1u << 9)
#define RST_EN_UART1_APB               (1u << 14)
#define UART1_CLK_EN                   (1u << 26)
#define LPPERI_MEM_CTRL                0x50120028
#define LP_UART_WAKEUP_FLAG            (1u << 1)
#define LP_UART_WAKEUP_EN              (1u << 29)

#define PMU_HP_SLEEP_HP_SYS_CNTL       0x50115078
#define PMU_SLP_WAKEUP_CNTL0           0x50115120
#define PMU_SLP_WAKEUP_CNTL2           0x50115128
#define PMU_SLP_WAKEUP_STATUS0         0x50115144
#define PMU_MAIN_STATE                 0x501151a4
#define PMU_CNTL_UART_WAKEUP_EN        (1u << 24)
#define PMU_CNTL_DIG_CPU_STALL         (1u << 29)
#define PMU_SLEEP_REQ                  (1u << 31)
#define PMU_WAKE_UART2                 6
#define PMU_WAKE_LP_UART               10
#define PMU_STATE_ACTIVE               0x01

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M -serial none");
}

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t off)
{
    return qtest_readl(qts, base + off);
}

static void wr(QTestState *qts, uint32_t base, uint32_t off, uint32_t v)
{
    qtest_writel(qts, base + off, v);
}

/* REG_UPDATE: the core takes the _SYNC registers within a few cycles. */
static void update(QTestState *qts, uint32_t base)
{
    wr(qts, base, REG_UPDATE, 1);
    qtest_clock_step(qts, 200);
    g_assert_cmphex(rd(qts, base, REG_UPDATE), ==, 0);
}

/*
 * A UART at 1 Mbaud with `conf0`, no TX idle gap, its interrupts clear
 * and its FIFOs empty.
 */
static void setup(QTestState *qts, uint32_t base, uint32_t conf0)
{
    wr(qts, base, CLKDIV, DIV_1US);
    wr(qts, base, IDLE_CONF, 0x100);
    wr(qts, base, CONF0, conf0 | CONF0_TXFIFO_RST | CONF0_RXFIFO_RST);
    update(qts, base);
    wr(qts, base, CONF0, conf0);
    update(qts, base);
    wr(qts, base, INT_CLR, 0xfffff);
}

static unsigned rx_count(QTestState *qts, uint32_t base)
{
    return STATUS_RXFIFO_CNT(rd(qts, base, STATUS));
}

/* UART1's TXD out on GPIO30 and UART2's RXD in from it, by the matrix. */
static void route_uart1_to_uart2(QTestState *qts)
{
    qtest_writel(qts, IO_MUX_GPIO(30), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    qtest_writel(qts, GPIO_FUNC_OUT(30), SIG_UART1_TXD);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_UART2_RXD), IN_SIG_IN_SEL | 30);
}

/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    static const struct {
        uint32_t off, reset, mask, lp_reset, lp_mask;
    } regs[] = {
        { INT_ENA, 0, 0x000fffff, 0, 0x000c7fff },
        { CLKDIV, 0x2b6, 0x00f00fff, 0x2b6, 0x00f00fff },
        { RX_FILT, 0x8, 0x1ff, 0x8, 0x1ff },
        { CONF0, 0x1c, 0x00ffffff, 0x1c, 0x00f7b07f },
        { CONF1, 0x6060, 0x003fffff, 0x6060, 0x003ff8f8 },
        { HWFC_CONF, 0, 0x1ff, 0, 0x1f8 },
        { SLEEP_CONF0, 0, 0xffffffff, 0, 0xffffffff },
        { SLEEP_CONF1, 0, 0xff, 0, 0xff },
        { SLEEP_CONF2, 0x1404f0, 0x0fffffff, 0x1420f0, 0x0fffe3ff },
        { SWFC_CONF0, 0x1311, 0x007fffff, 0x1311, 0x007fffff },
        { SWFC_CONF1, 0xe000, 0xffff, 0x6000, 0xf8f8 },
        { TXBRK_CONF, 0xa, 0xff, 0xa, 0xff },
        { IDLE_CONF, 0x40100, 0xfffff, 0x40100, 0xfffff },
        { RS485_CONF, 0, 0x3ff, 0, 0x6 },
        { AT_CMD_PRECNT, 0x901, 0xffff, 0x901, 0xffff },
        { AT_CMD_POSTCNT, 0x901, 0xffff, 0x901, 0xffff },
        { AT_CMD_GAPTOUT, 0xb, 0xffff, 0xb, 0xffff },
        { AT_CMD_CHAR, 0x32b, 0xffff, 0x32b, 0xffff },
        { MEM_CONF, 0, 0x06000000, 0, 0x06000000 },
        { TOUT_CONF, 0x28, 0xfff, 0x28, 0xfff },
        { DATE, 0x02305050, 0xffffffff, 0x02305050, 0xffffffff },
        { ID, 0x500, 0xffffffff, 0x500, 0xffffffff },
    };
    int i;

    g_assert_cmphex(rd(qts, UART(1), INT_RAW), ==, INT_TXFIFO_EMPTY);
    for (i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(rd(qts, UART(1), regs[i].off), ==, regs[i].reset);
        g_assert_cmphex(rd(qts, LP_UART, regs[i].off), ==,
                        regs[i].lp_reset);
        wr(qts, UART(1), regs[i].off, 0xffffffff);
        wr(qts, LP_UART, regs[i].off, 0xffffffff);
        if (regs[i].off != MEM_CONF) {
            g_assert_cmphex(rd(qts, UART(1), regs[i].off), ==,
                            regs[i].mask);
            g_assert_cmphex(rd(qts, LP_UART, regs[i].off), ==,
                            regs[i].lp_mask);
        }
        wr(qts, UART(1), regs[i].off, regs[i].reset);
        wr(qts, LP_UART, regs[i].off, regs[i].lp_reset);
    }
    /* CONF1's CTS_INV and DSR_INV changed CTS and DSR as the core sees them. */
    g_assert_cmphex(rd(qts, UART(1), INT_RAW), ==,
                    INT_TXFIFO_EMPTY | INT_CTS_CHG | INT_DSR_CHG);
    g_assert_cmphex(rd(qts, UART(1), CLK_CONF), ==, 0x03000000);
    g_assert_cmphex(rd(qts, UART(1), MEM_TX_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), MEM_RX_STATUS), ==, 0x00010080);
    g_assert_cmphex(rd(qts, LP_UART, MEM_RX_STATUS), ==, 0x00010080);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), AFIFO_STATUS), ==, 0xa);
    g_assert_cmphex(rd(qts, UART(1), LOWPULSE), ==, 0xfff);
    g_assert_cmphex(rd(qts, UART(1), HIGHPULSE), ==, 0xfff);
    g_assert_cmphex(rd(qts, UART(1), POSPULSE), ==, 0xfff);
    g_assert_cmphex(rd(qts, UART(1), NEGPULSE), ==, 0xfff);
    g_assert_cmphex(rd(qts, UART(1), RXD_CNT), ==, 0);
    /*
     * TXD, RTS and DTR idle high; RXD, CTS and DSR are the matrix's
     * defaults for unrouted inputs, low.
     */
    g_assert_cmphex(rd(qts, UART(1), STATUS), ==, 0xe0000000);
    /* The LP UART has no autobaud registers; reserved offsets read 0. */
    g_assert_cmphex(rd(qts, LP_UART, LOWPULSE), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), 0x28), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), 0x94), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), 0xa0), ==, 0);
    /*
     * INT_RAW clears by writing 1, TXFIFO_EMPTY setting itself again while
     * the TX FIFO is empty; INT_CLR reads 0.
     */
    wr(qts, UART(1), INT_RAW, INT_CTS_CHG | INT_TXFIFO_EMPTY);
    g_assert_cmphex(rd(qts, UART(1), INT_RAW), ==,
                    INT_TXFIFO_EMPTY | INT_DSR_CHG);
    wr(qts, UART(1), INT_RAW, INT_DSR_CHG);
    g_assert_cmphex(rd(qts, UART(1), INT_RAW), ==, INT_TXFIFO_EMPTY);
    g_assert_cmphex(rd(qts, UART(1), INT_CLR), ==, 0);
    qtest_quit(qts);
}

/*
 * The _SYNC registers reach the core only through REG_UPDATE, which takes
 * three core clock cycles and waits while that clock is stopped.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_reg_update(void)
{
    QTestState *qts = start();

    wr(qts, UART(1), IDLE_CONF, 0x100);
    update(qts, UART(1));
    /* CLKDIV written but not updated: the old 694 cycles a bit. */
    wr(qts, UART(1), CLKDIV, DIV_1US);
    wr(qts, UART(1), CONF0, CONF0_8N1 | CONF0_LOOPBACK);
    g_assert_cmphex(rd(qts, UART(1), CLKDIV), ==, DIV_1US);
    wr(qts, UART(1), REG_UPDATE, 1);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 1);
    qtest_clock_step(qts, 50);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 1);
    qtest_clock_step(qts, 50);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 0);
    wr(qts, UART(1), FIFO, 0x5a);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 1);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x5a);

    /* With UART1's core clock gated the update waits for it. */
    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL111,
                 qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL111) &
                 ~UART1_CLK_EN);
    wr(qts, UART(1), CLKDIV, 694);
    wr(qts, UART(1), REG_UPDATE, 1);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 1);
    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL111,
                 qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL111) |
                 UART1_CLK_EN);
    qtest_clock_step(qts, 100);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 0);
    qtest_quit(qts);
}

/*
 * The transmitter's timing: bits of (CLKDIV + CLKDIV_FRAG / 16) core
 * clock cycles, the FSM through start, data and stop bits, TX_DONE as the
 * stop bit ends, and the TX FIFO count.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_baud_timing(void)
{
    QTestState *qts = start();

    setup(qts, UART(1), CONF0_8N1);
    wr(qts, UART(1), FIFO, 0x55);
    wr(qts, UART(1), FIFO, 0xaa);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 1);
    qtest_clock_step(qts, BIT_NS / 2);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 1);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_TXD);
    qtest_clock_step(qts, BIT_NS);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 2);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_TXD);
    qtest_clock_step(qts, 8 * BIT_NS);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 11);
    /* The second byte follows the first's stop bit at once. */
    qtest_clock_step(qts, BIT_NS);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 1);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    qtest_clock_step(qts, FRAME_NS - BIT_NS / 2 - 10);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    qtest_clock_step(qts, 20);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 0);
    /* With no TX_IDLE_NUM, TX_BRK_IDLE_DONE follows at once. */
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_BRK_IDLE);

    /* 40 + 8/16 cycles a bit: a frame of 10 bits takes 10125 ns. */
    wr(qts, UART(1), CLKDIV, DIV_1US | (8u << 20));
    update(qts, UART(1));
    wr(qts, UART(1), INT_CLR, INT_TX_DONE);
    wr(qts, UART(1), FIFO, 0x00);
    qtest_clock_step(qts, 10125 - 10);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    qtest_clock_step(qts, 20);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);

    /* TX_IDLE_NUM bits of quiet follow a transfer. */
    wr(qts, UART(1), CLKDIV, DIV_1US);
    wr(qts, UART(1), IDLE_CONF, 5 << 10);
    update(qts, UART(1));
    wr(qts, UART(1), INT_CLR, 0xfffff);
    wr(qts, UART(1), FIFO, 0x00);
    qtest_clock_step(qts, FRAME_NS + 10);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_TX_BRK_IDLE);
    /* A byte written in the gap waits for its end. */
    wr(qts, UART(1), FIFO, 0x00);
    qtest_clock_step(qts, 4 * BIT_NS);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 1);
    qtest_clock_step(qts, BIT_NS);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_BRK_IDLE);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 0);
    qtest_quit(qts);
}

/*
 * The FIFOs and their interrupts, through the internal loopback:
 * RXFIFO_FULL at the threshold, TXFIFO_EMPTY below its threshold, latched
 * until cleared and set again while their condition holds; RXFIFO_TOUT
 * after the bus idles RX_TOUT_THRHD bits; RXFIFO_OVF; the FIFO RAM
 * offsets; the interrupt line.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_fifo_irq(void)
{
    QTestState *qts = start();
    int i;

    qtest_irq_intercept_out_named(qts, UART_PATH(1), "sysbus-irq");
    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK);
    wr(qts, UART(1), CONF1, (2u << 8) | 4);
    wr(qts, UART(1), TOUT_CONF, TOUT_EN | TOUT_THRHD(3));
    update(qts, UART(1));
    wr(qts, UART(1), INT_CLR, 0xfffff);
    wr(qts, UART(1), INT_ENA, INT_RXFIFO_FULL);
    g_assert_false(qtest_get_irq(qts, 0));

    for (i = 0; i < 5; i++) {
        wr(qts, UART(1), FIFO, 0x30 + i);
    }
    g_assert_cmphex(rd(qts, UART(1), MEM_TX_STATUS), ==, 5 | (1 << 9));
    /* TXFIFO_EMPTY latched while the FIFO was empty; four are waiting. */
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TXFIFO_EMPTY);
    wr(qts, UART(1), INT_CLR, INT_TXFIFO_EMPTY);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_TXFIFO_EMPTY);
    qtest_clock_step(qts, 3 * FRAME_NS + BIT_NS);
    /* Three left the TX FIFO: one left, below the threshold of 2. */
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TXFIFO_EMPTY);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 3);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 4);
    g_assert_true(rd(qts, UART(1), INT_ST) & INT_RXFIFO_FULL);
    g_assert_true(qtest_get_irq(qts, 0));
    /* Clearing it while the FIFO is still at the threshold: set again */
    wr(qts, UART(1), INT_CLR, INT_RXFIFO_FULL);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_RXFIFO_FULL);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x30);
    wr(qts, UART(1), INT_CLR, INT_RXFIFO_FULL);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_RXFIFO_FULL);
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, UART(1), MEM_RX_STATUS), ==,
                    (128 + 1) | ((128 + 4) << 9));
    /* The last byte, then 3 bits of quiet: RXFIFO_TOUT */
    qtest_clock_step(qts, FRAME_NS + BIT_NS / 2);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 4);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_RXFIFO_TOUT);
    qtest_clock_step(qts, 3 * BIT_NS);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_RXFIFO_TOUT);
    for (i = 1; i < 5; i++) {
        g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x30 + i);
    }
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 0);

    /* 129 bytes into a 128-byte RX FIFO: RXFIFO_OVF, the last dropped */
    for (i = 0; i < 128; i++) {
        wr(qts, UART(1), FIFO, i);
    }
    qtest_clock_step(qts, 128 * FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 128);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_RXFIFO_OVF);
    wr(qts, UART(1), FIFO, 0xff);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_RXFIFO_OVF);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0);
    qtest_quit(qts);
}

/*
 * Frames between two UARTs through the GPIO matrix: data, parity errors,
 * ERR_WR_MASK, a break, the receiver's FSM.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_uart_to_uart(void)
{
    QTestState *qts = start();

    route_uart1_to_uart2(qts);
    setup(qts, UART(1), CONF0_8N1);
    setup(qts, UART(2), CONF0_8N1);
    wr(qts, UART(1), FIFO, 'P');
    wr(qts, UART(1), FIFO, '4');
    qtest_clock_step(qts, FRAME_NS + BIT_NS / 2);
    /* UART2's receiver is in the second frame's start bit. */
    g_assert_cmphex(rd(qts, UART(2), FSM_STATUS) & 0xf, ==, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(rx_count(qts, UART(2)), ==, 2);
    g_assert_cmphex(rd(qts, UART(2), FIFO), ==, 'P');
    g_assert_cmphex(rd(qts, UART(2), FIFO), ==, '4');
    g_assert_cmphex(rd(qts, UART(2), INT_RAW) &
                    (INT_PARITY_ERR | INT_FRM_ERR), ==, 0);

    /* Odd parity sent, even expected: a parity error, the byte kept */
    setup(qts, UART(1), CONF0_8N1 | CONF0_PARITY_ODD);
    setup(qts, UART(2), CONF0_8N1 | CONF0_PARITY_EVEN);
    wr(qts, UART(1), FIFO, 0x01);
    qtest_clock_step(qts, FRAME_NS + 2 * BIT_NS);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_PARITY_ERR);
    g_assert_cmpuint(rx_count(qts, UART(2)), ==, 1);
    /* With ERR_WR_MASK it is not stored. */
    setup(qts, UART(2), CONF0_8N1 | CONF0_PARITY_EVEN | CONF0_ERR_WR_MASK);
    wr(qts, UART(1), FIFO, 0x01);
    qtest_clock_step(qts, FRAME_NS + 2 * BIT_NS);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_PARITY_ERR);
    g_assert_cmpuint(rx_count(qts, UART(2)), ==, 0);

    /*
     * A byte, then a break of 20 bits with TXD_BRK: TX_DONE, then
     * TX_BRK_DONE; the receiver sees a NULL character with a framing
     * error and BRK_DET.
     */
    setup(qts, UART(1), CONF0_8N1 | CONF0_TXD_BRK);
    setup(qts, UART(2), CONF0_8N1);
    wr(qts, UART(1), TXBRK_CONF, 20);
    update(qts, UART(1));
    wr(qts, UART(1), FIFO, 0x42);
    qtest_clock_step(qts, FRAME_NS + 10);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_TX_BRK_DONE);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_TXD);
    qtest_clock_step(qts, 20 * BIT_NS);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_BRK_DONE);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_BRK_IDLE);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_BRK_DET);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_FRM_ERR);
    g_assert_cmpuint(rx_count(qts, UART(2)), ==, 2);
    g_assert_cmphex(rd(qts, UART(2), FIFO), ==, 0x42);
    g_assert_cmphex(rd(qts, UART(2), FIFO), ==, 0x00);
    qtest_quit(qts);
}

/*
 * Hardware flow control: with TX_FLOW_EN the transmitter waits for CTS,
 * here UART1's own RTS looped back, which RX_FLOW_EN drives inactive once
 * the RX FIFO holds more than RX_FLOW_THRHD bytes; CTS_CHG on each change.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_hw_flow(void)
{
    QTestState *qts = start();
    int i;

    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK | CONF0_TX_FLOW_EN);
    /* RTS inactive (SW_RTS clear) holds the transmitter. */
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_RTSN);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_CTSN);
    wr(qts, UART(1), FIFO, 1);
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 1);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 0);

    /* RX_FLOW_EN: RTS active while the RX FIFO holds 2 or fewer */
    wr(qts, UART(1), HWFC_CONF, HWFC_RX_FLOW_EN | 2);
    update(qts, UART(1));
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_RTSN);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_CTS_CHG);
    for (i = 0; i < 4; i++) {
        wr(qts, UART(1), FIFO, 2 + i);
    }
    qtest_clock_step(qts, 6 * FRAME_NS);
    /* The third byte stored took RTS inactive: the rest wait. */
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 3);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 2);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_RTSN);
    rd(qts, UART(1), FIFO);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 3);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 1);
    qtest_quit(qts);
}

/*
 * Software flow control: an XOFF received stops the transmitter once the
 * byte it is sending is out (FSM TX_WAIT_SEND with data waiting), and
 * XONOFF_DEL keeps it out of the RX FIFO; SEND_XOFF waits too, without
 * XON_XOFF_STILL_SEND; with flow control off it goes, then the data.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_sw_flow(void)
{
    QTestState *qts = start();

    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK);
    wr(qts, UART(1), SWFC_CONF0, 0x1311 | SWFC_EN | SWFC_XONOFF_DEL);
    update(qts, UART(1));
    /* XOFF looped back, then two bytes: they wait. */
    wr(qts, UART(1), FIFO, 0x13);
    wr(qts, UART(1), FIFO, 0x41);
    wr(qts, UART(1), FIFO, 0x42);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_SW_XOFF);
    qtest_clock_step(qts, 3 * FRAME_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 0);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 2);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 0xf);

    wr(qts, UART(1), SWFC_CONF0, 0x1311 | SWFC_EN | SWFC_XONOFF_DEL |
       SWFC_SEND_XOFF);
    update(qts, UART(1));
    g_assert_true(rd(qts, UART(1), SWFC_CONF0) & SWFC_SEND_XOFF);
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_true(rd(qts, UART(1), SWFC_CONF0) & SWFC_SEND_XOFF);
    /* FORCE_XOFF holds the transmitter whatever was received. */
    wr(qts, UART(1), SWFC_CONF0, 0x1311 | SWFC_EN | SWFC_FORCE_XOFF);
    update(qts, UART(1));
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 0xf);
    wr(qts, UART(1), SWFC_CONF0, 0x1311);
    update(qts, UART(1));
    qtest_clock_step(qts, 3 * FRAME_NS + BIT_NS);
    /* The pending XOFF goes out, then the data. */
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 0);
    g_assert_false(rd(qts, UART(1), SWFC_CONF0) & SWFC_SEND_XOFF);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 3);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x13);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x41);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x42);
    qtest_quit(qts);
}

/*
 * Autobaud: UART2 measures UART1's frames, sent at 1 Mbaud, in core clock
 * cycles: 0x55's pulses are one bit, 40 cycles, counted as 39; the
 * distance between rising edges two bits, 79.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_autobaud(void)
{
    QTestState *qts = start();
    int i;

    route_uart1_to_uart2(qts);
    setup(qts, UART(1), CONF0_8N1);
    setup(qts, UART(2), CONF0_8N1 | CONF0_AUTOBAUD_EN);
    wr(qts, UART(2), CLKDIV, 694);
    update(qts, UART(2));
    for (i = 0; i < 4; i++) {
        wr(qts, UART(1), FIFO, 0x55);
    }
    qtest_clock_step(qts, 4 * FRAME_NS + BIT_NS);
    g_assert_cmphex(rd(qts, UART(2), LOWPULSE), ==, 39);
    g_assert_cmphex(rd(qts, UART(2), HIGHPULSE), ==, 39);
    g_assert_cmphex(rd(qts, UART(2), POSPULSE), ==, 79);
    g_assert_cmphex(rd(qts, UART(2), NEGPULSE), ==, 79);
    /* 0x55 8N1 is ten bits of alternating level: 40 edges. */
    g_assert_cmpuint(rd(qts, UART(2), RXD_CNT), ==, 40);
    /* (LOWPULSE + HIGHPULSE + 2) / 2 is the bit time in cycles. */
    wr(qts, UART(2), CONF0, CONF0_8N1);
    update(qts, UART(2));
    g_assert_cmphex(rd(qts, UART(2), LOWPULSE), ==, 39);
    /* Enabling it again starts afresh. */
    wr(qts, UART(2), CONF0, CONF0_8N1 | CONF0_AUTOBAUD_EN);
    update(qts, UART(2));
    g_assert_cmphex(rd(qts, UART(2), LOWPULSE), ==, 0xfff);
    g_assert_cmpuint(rd(qts, UART(2), RXD_CNT), ==, 0);
    qtest_quit(qts);
}

/*
 * A pulse on RXD shorter than half a bit: the receiver finds the line
 * high again at the start bit's middle, a glitch (GLITCH_DET); with the
 * glitch filter set wider than it, it never reaches the receiver.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_glitch(void)
{
    QTestState *qts = start();

    qtest_writel(qts, IO_MUX_GPIO(31), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_UART2_RXD), IN_SIG_IN_SEL | 31);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 31, 1);
    setup(qts, UART(2), CONF0_8N1);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 31, 0);
    qtest_clock_step(qts, 200);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 31, 1);
    qtest_clock_step(qts, BIT_NS);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_GLITCH_DET);
    g_assert_cmpuint(rx_count(qts, UART(2)), ==, 0);

    /* A 10-cycle filter takes out the 200 ns (8 cycle) pulse. */
    wr(qts, UART(2), RX_FILT, (1u << 8) | 10);
    wr(qts, UART(2), INT_CLR, INT_GLITCH_DET);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 31, 0);
    qtest_clock_step(qts, 200);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 31, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_false(rd(qts, UART(2), INT_RAW) & INT_GLITCH_DET);
    g_assert_true(rd(qts, UART(2), STATUS) & STATUS_RXD);
    qtest_quit(qts);
}

/*
 * RS485: DTR is the driver enable, high from the DL0 turnaround bit
 * through the DL1 bit after the stop bit; with RS485TX_RX_EN the receiver
 * hears the echo, and an echo that differs is a clash.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_rs485(void)
{
    QTestState *qts = start();

    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK);
    wr(qts, UART(1), RS485_CONF, RS485_EN | RS485_DL0_EN | RS485_DL1_EN |
       RS485_TX_RX_EN);
    update(qts, UART(1));
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_DTRN);
    wr(qts, UART(1), FIFO, 0x77);
    qtest_clock_step(qts, BIT_NS / 2);
    /* The DL0 bit: DE up, the line still idle */
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_DTRN);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_TXD);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 13);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 14);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_DTRN);
    qtest_clock_step(qts, BIT_NS);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_DTRN);
    g_assert_true(rd(qts, UART(1), INT_RAW) & INT_TX_DONE);
    /* The echo matched: no clash, and it was stored. */
    g_assert_false(rd(qts, UART(1), INT_RAW) & INT_RS485_CLASH);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 1);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x77);
    qtest_quit(qts);
}

#define CONF0_IRDA_DPLX     (1u << 7)
#define CONF0_IRDA_TX_EN    (1u << 8)
#define CONF0_IRDA_EN       (1u << 14)

/*
 * IrDA (TRM 45.4.7): a 0 bit is a high pulse in the 9th to 11th
 * sixteenths of its bit time and a 1 bit no pulse; UART1's frames reach
 * UART2's IrDA receiver through the matrix, and come back to UART1's own
 * receiver with IRDA_DPLX.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_irda(void)
{
    QTestState *qts = start();

    route_uart1_to_uart2(qts);
    setup(qts, UART(1), CONF0_8N1 | CONF0_IRDA_EN | CONF0_IRDA_TX_EN);
    setup(qts, UART(2), CONF0_8N1 | CONF0_IRDA_EN);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_TXD);
    wr(qts, UART(1), FIFO, 0xf0);
    /* The start bit: low, then its pulse */
    qtest_clock_step(qts, 300);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_TXD);
    qtest_clock_step(qts, 300);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_TXD);
    qtest_clock_step(qts, 200);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_TXD);
    /* Data bit 4 is a 1: no pulse */
    qtest_clock_step(qts, 4 * BIT_NS);
    g_assert_false(rd(qts, UART(1), STATUS) & STATUS_TXD);
    qtest_clock_step(qts, 6 * BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(2)), ==, 1);
    g_assert_cmphex(rd(qts, UART(2), FIFO), ==, 0xf0);
    /* Sending, UART1 hears nothing; with IRDA_DPLX it hears itself. */
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 0);
    setup(qts, UART(1), CONF0_8N1 | CONF0_IRDA_EN | CONF0_IRDA_TX_EN |
          CONF0_IRDA_DPLX);
    wr(qts, UART(1), FIFO, 0x5a);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 1);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x5a);
    qtest_quit(qts);
}

/*
 * AT_CMD detection: three '+' within RX_GAP_TOUT bits of each other,
 * after PRE_IDLE_NUM bits of quiet, followed by POST_IDLE_NUM bits of it.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_at_cmd(void)
{
    QTestState *qts = start();
    int i;

    route_uart1_to_uart2(qts);
    setup(qts, UART(1), CONF0_8N1);
    setup(qts, UART(2), CONF0_8N1);
    wr(qts, UART(2), AT_CMD_PRECNT, 20);
    wr(qts, UART(2), AT_CMD_POSTCNT, 20);
    wr(qts, UART(2), AT_CMD_GAPTOUT, 15);
    update(qts, UART(2));
    for (i = 0; i < 3; i++) {
        wr(qts, UART(1), FIFO, '+');
    }
    qtest_clock_step(qts, 3 * FRAME_NS + 19 * BIT_NS);
    g_assert_false(rd(qts, UART(2), INT_RAW) & INT_AT_CMD_DET);
    qtest_clock_step(qts, 2 * BIT_NS);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_AT_CMD_DET);

    /* A byte too soon after the last '+': nothing */
    wr(qts, UART(2), INT_CLR, INT_AT_CMD_DET);
    qtest_clock_step(qts, 30 * BIT_NS);
    for (i = 0; i < 3; i++) {
        wr(qts, UART(1), FIFO, '+');
    }
    wr(qts, UART(1), FIFO, 'x');
    qtest_clock_step(qts, 4 * FRAME_NS + 40 * BIT_NS);
    g_assert_false(rd(qts, UART(2), INT_RAW) & INT_AT_CMD_DET);
    qtest_quit(qts);
}

/*
 * The LP UART: 16-byte FIFOs whose counts and thresholds sit three bits
 * up in their fields; its loopback.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_lp_uart(void)
{
    QTestState *qts = start();
    int i;

    /* LP_UART_SCLK is RC_FAST out of reset; CLKDIV 20 is about 1 Mbaud. */
    wr(qts, LP_UART, CLKDIV, 20);
    wr(qts, LP_UART, IDLE_CONF, 0x100);
    wr(qts, LP_UART, CONF0, CONF0_8N1 | CONF0_LOOPBACK);
    update(qts, LP_UART);
    for (i = 0; i < 17; i++) {
        wr(qts, LP_UART, FIFO, i);
    }
    /* One on the line, 16 waiting, the count three bits up */
    g_assert_cmphex(rd(qts, LP_UART, STATUS) & 0x00ff0000, ==, 16u << 19);
    g_assert_cmphex(rd(qts, LP_UART, MEM_TX_STATUS), ==,
                    (1u << 3) | (1u << 12));
    qtest_clock_step(qts, 20 * 12 * BIT_NS);
    /* 17 bytes into the 16-byte RX FIFO: the last one overflowed. */
    g_assert_cmphex(rd(qts, LP_UART, STATUS) & 0xff, ==, 16u << 3);
    g_assert_true(rd(qts, LP_UART, INT_RAW) & INT_RXFIFO_OVF);
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(rd(qts, LP_UART, FIFO), ==, i);
    }
    qtest_quit(qts);
}

/*
 * The core reset (HP_SYS_CLKRST_RST_EN_UART1_CORE, or CLK_CONF's
 * TX_RST_CORE and RX_RST_CORE) empties the FIFOs and stops the line but
 * keeps the registers; the APB reset returns the registers too.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_resets(void)
{
    QTestState *qts = start();

    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK);
    wr(qts, UART(1), FIFO, 1);
    wr(qts, UART(1), FIFO, 2);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 1);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, RST_EN_UART1_CORE);
    g_assert_cmphex(rd(qts, UART(1), STATUS) & 0x00ff00ff, ==, 0);
    g_assert_true(rd(qts, UART(1), STATUS) & STATUS_TXD);
    /* The register update waits for the core. */
    wr(qts, UART(1), REG_UPDATE, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 1);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, 0);
    qtest_clock_step(qts, 200);
    g_assert_cmphex(rd(qts, UART(1), REG_UPDATE), ==, 0);
    g_assert_cmphex(rd(qts, UART(1), CLKDIV), ==, DIV_1US);
    g_assert_cmphex(rd(qts, UART(1), CONF0), ==, CONF0_8N1 | CONF0_LOOPBACK);

    /* CLK_CONF: the transmitter held in reset drops its FIFO. */
    wr(qts, UART(1), FIFO, 3);
    wr(qts, UART(1), FIFO, 4);
    wr(qts, UART(1), CLK_CONF, CLK_TX_SCLK_EN | CLK_RX_SCLK_EN |
       CLK_TX_RST_CORE);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 0);
    wr(qts, UART(1), CLK_CONF, CLK_TX_SCLK_EN | CLK_RX_SCLK_EN);
    /* With TX_SCLK_EN clear the transmitter does not start. */
    qtest_clock_step(qts, FRAME_NS);
    wr(qts, UART(1), INT_CLR, 0xfffff);
    wr(qts, UART(1), CLK_CONF, CLK_RX_SCLK_EN);
    wr(qts, UART(1), FIFO, 5);
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_cmpuint(STATUS_TXFIFO_CNT(rd(qts, UART(1), STATUS)), ==, 1);
    g_assert_cmphex(rd(qts, UART(1), FSM_STATUS) >> 4, ==, 0);
    wr(qts, UART(1), CLK_CONF, CLK_TX_SCLK_EN | CLK_RX_SCLK_EN);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 1);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 5);

    /* The APB reset */
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, RST_EN_UART1_APB);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, 0);
    g_assert_cmphex(rd(qts, UART(1), CLKDIV), ==, 0x2b6);
    g_assert_cmphex(rd(qts, UART(1), CONF0), ==, CONF0_8N1);
    g_assert_cmphex(rd(qts, UART(1), INT_RAW), ==, INT_TXFIFO_EMPTY);
    qtest_quit(qts);
}

/*
 * Wakeup, armed by the PMU's UART_WAKEUP_EN: mode 2 wakes on a start
 * bit, raising WAKEUP and the wakeup output; mode 0 needs
 * ACTIVE_THRESHOLD + 6 rising edges on RXD.
 * The LP UART's wakeup reaches LPPERI's flag.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_wakeup(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, UART_PATH(2), "wakeup");
    route_uart1_to_uart2(qts);
    setup(qts, UART(1), CONF0_8N1);
    setup(qts, UART(2), CONF0_8N1);
    wr(qts, UART(2), SLEEP_CONF2, 0x001404f0 | (2u << 26));
    /* Awake, a start bit does nothing. */
    wr(qts, UART(1), FIFO, 0x00);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_false(rd(qts, UART(2), INT_RAW) & INT_WAKEUP);
    qtest_set_irq_in(qts, UART_PATH(2), "wakeup-en", 0, 1);
    wr(qts, UART(1), FIFO, 0x00);
    qtest_clock_step(qts, BIT_NS);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_WAKEUP);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_set_irq_in(qts, UART_PATH(2), "wakeup-en", 0, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Mode 0, ACTIVE_THRESHOLD 2: 8 rising edges; 0x55 has 5 a frame */
    qtest_clock_step(qts, 2 * FRAME_NS);
    wr(qts, UART(2), INT_CLR, INT_WAKEUP);
    wr(qts, UART(2), SLEEP_CONF2, 0x00140002);
    qtest_set_irq_in(qts, UART_PATH(2), "wakeup-en", 0, 1);
    wr(qts, UART(1), FIFO, 0x55);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_false(rd(qts, UART(2), INT_RAW) & INT_WAKEUP);
    wr(qts, UART(1), FIFO, 0x55);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_WAKEUP);
    qtest_set_irq_in(qts, UART_PATH(2), "wakeup-en", 0, 0);

    /* The LP UART, in mode 1 with RX_WAKE_UP_THRHD 0: its first byte */
    qtest_writel(qts, LPPERI_MEM_CTRL,
                 qtest_readl(qts, LPPERI_MEM_CTRL) | LP_UART_WAKEUP_EN);
    wr(qts, LP_UART, CLKDIV, 20);
    wr(qts, LP_UART, CONF0, CONF0_8N1 | CONF0_LOOPBACK);
    update(qts, LP_UART);
    wr(qts, LP_UART, SLEEP_CONF2, 0x00140000 | (1u << 26));
    qtest_set_irq_in(qts, LP_UART_PATH, "wakeup-en", 0, 1);
    g_assert_false(qtest_readl(qts, LPPERI_MEM_CTRL) & LP_UART_WAKEUP_FLAG);
    wr(qts, LP_UART, FIFO, 0x31);
    qtest_clock_step(qts, 12 * BIT_NS);
    g_assert_true(rd(qts, LP_UART, INT_RAW) & INT_WAKEUP);
    g_assert_true(qtest_readl(qts, LPPERI_MEM_CTRL) & LP_UART_WAKEUP_FLAG);
    qtest_quit(qts);
}

static uint32_t pmu_state(QTestState *qts)
{
    return qtest_readl(qts, PMU_MAIN_STATE) >> 25;
}

/*
 * Through the PMU, set up as ESP-IDF's pmu_param.c has it: HP_SLEEP's
 * UART_WAKEUP_EN arms UART2 as the chip falls asleep, and a start bit on
 * its RXD wakes the chip, the PMU recording source UART2; awake, with
 * HP_ACTIVE's UART_WAKEUP_EN clear, a start bit wakes nothing. The LP
 * UART's wakeup is the PMU's LP_UART source.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_pmu_wakeup(void)
{
    QTestState *qts = start();

    /* RXD idles high, as a line on a pad would. */
    qtest_set_irq_in(qts, UART_PATH(2), "rxd", 0, 1);
    qtest_set_irq_in(qts, LP_UART_PATH, "rxd", 0, 1);
    setup(qts, UART(2), CONF0_8N1);
    wr(qts, UART(2), SLEEP_CONF2, 0x001404f0 | (2u << 26));
    qtest_writel(qts, PMU_HP_SLEEP_HP_SYS_CNTL,
                 PMU_CNTL_UART_WAKEUP_EN | PMU_CNTL_DIG_CPU_STALL);
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL2, 1u << PMU_WAKE_UART2);
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL0, PMU_SLEEP_REQ);
    qtest_clock_step(qts, 100 * BIT_NS);
    g_assert_cmphex(pmu_state(qts), !=, PMU_STATE_ACTIVE);

    qtest_set_irq_in(qts, UART_PATH(2), "rxd", 0, 0);
    qtest_clock_step(qts, BIT_NS);
    qtest_set_irq_in(qts, UART_PATH(2), "rxd", 0, 1);
    qtest_clock_step(qts, 200 * BIT_NS);
    g_assert_cmphex(pmu_state(qts), ==, PMU_STATE_ACTIVE);
    g_assert_cmphex(qtest_readl(qts, PMU_SLP_WAKEUP_STATUS0), ==,
                    1u << PMU_WAKE_UART2);
    g_assert_true(rd(qts, UART(2), INT_RAW) & INT_WAKEUP);

    wr(qts, UART(2), INT_CLR, INT_WAKEUP);
    qtest_set_irq_in(qts, UART_PATH(2), "rxd", 0, 0);
    qtest_clock_step(qts, BIT_NS);
    qtest_set_irq_in(qts, UART_PATH(2), "rxd", 0, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_false(rd(qts, UART(2), INT_RAW) & INT_WAKEUP);

    /* The LP UART, in mode 1 with RX_WAKE_UP_THRHD 0: its first byte */
    wr(qts, LP_UART, CLKDIV, 20);
    wr(qts, LP_UART, CONF0, CONF0_8N1);
    update(qts, LP_UART);
    wr(qts, LP_UART, SLEEP_CONF2, 0x00140000 | (1u << 26));
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL2, 1u << PMU_WAKE_LP_UART);
    qtest_writel(qts, PMU_SLP_WAKEUP_CNTL0, PMU_SLEEP_REQ);
    qtest_clock_step(qts, 100 * BIT_NS);
    g_assert_cmphex(pmu_state(qts), !=, PMU_STATE_ACTIVE);
    qtest_set_irq_in(qts, LP_UART_PATH, "rxd", 0, 0);
    qtest_clock_step(qts, 9 * BIT_NS);
    qtest_set_irq_in(qts, LP_UART_PATH, "rxd", 0, 1);
    qtest_clock_step(qts, 200 * BIT_NS);
    g_assert_cmphex(pmu_state(qts), ==, PMU_STATE_ACTIVE);
    g_assert_cmphex(qtest_readl(qts, PMU_SLP_WAKEUP_STATUS0), ==,
                    1u << PMU_WAKE_LP_UART);
    qtest_quit(qts);
}

/*
 * Serial port 0 is a terminal on UART0's pads: UART0's frames reach the
 * chardev, and the chardev's bytes reach UART0 once software routes
 * GPIO38's input to it, at the baud rate UART0 is set to (out of reset,
 * the crystal over 694, about 57.6 kbaud).
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_serial_port(void)
{
    int fd;
    QTestState *qts = qtest_init_with_serial("-M esp32p4,flash-size=16M",
                                             &fd);
    const char *msg = "Hello, P4\r\n";
    int64_t frame = 10 * 694 * 25;
    char buf[32];
    size_t got = 0;
    int i;

    wr(qts, UART(0), IDLE_CONF, 0x100);
    update(qts, UART(0));
    for (i = 0; msg[i]; i++) {
        wr(qts, UART(0), FIFO, msg[i]);
    }
    qtest_clock_step(qts, (strlen(msg) + 1) * frame);
    while (got < strlen(msg)) {
        ssize_t n = recv(fd, buf + got, sizeof(buf) - got, 0);

        g_assert_cmpint(n, >, 0);
        got += n;
    }
    g_assert_cmpmem(buf, got, msg, strlen(msg));

    /* The bridge holds GPIO38 high; the pad's input is off from reset. */
    g_assert_false(rd(qts, UART(0), STATUS) & STATUS_RXD);
    qtest_writel(qts, IO_MUX_GPIO(38), FUN_IE | FUN_WPU | FUN_DRV(2));
    g_assert_true(rd(qts, UART(0), STATUS) & STATUS_RXD);
    g_assert_cmpint(send(fd, "ok\n", 3, 0), ==, 3);
    for (i = 0; i < 100 && rx_count(qts, UART(0)) < 3; i++) {
        g_usleep(10000);
        qtest_clock_step(qts, frame);
    }
    g_assert_cmpuint(rx_count(qts, UART(0)), ==, 3);
    g_assert_cmphex(rd(qts, UART(0), FIFO), ==, 'o');
    g_assert_cmphex(rd(qts, UART(0), FIFO), ==, 'k');
    g_assert_cmphex(rd(qts, UART(0), FIFO), ==, '\n');
    close(fd);
    qtest_quit(qts);
}

/*
 * The boot ROM, run from reset with a blank 16 MiB flash, prints its
 * banner on UART0, through GPIO37 to serial port 0, and reports the
 * flash's invalid image header. The log is read while the ROM prints, a
 * character a frame, so the wait is for the whole report.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_rom_banner(void)
{
    g_autofree char *log = NULL;
    g_autofree char *text = NULL;
    QTestState *qts;
    int fd, i;

    fd = g_file_open_tmp("esp32p4-uart-XXXXXX.log", &log, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = qtest_initf("-M esp32p4,flash-size=16M -accel tcg "
                      "-icount shift=0,sleep=off "
                      "-chardev file,id=s0,path=%s -serial chardev:s0", log);
    for (i = 0; i < 600; i++) {
        g_free(text);
        text = NULL;
        g_assert(g_file_get_contents(log, &text, NULL, NULL));
        if (strstr(text, "invalid header: 0xffffffff")) {
            break;
        }
        g_usleep(100000);
    }
    qtest_quit(qts);
    g_assert_nonnull(strstr(text, "ESP-ROM:esp32p4-eco5-20250430"));
    g_assert_nonnull(strstr(text, "rst:0x1 (POWERON)"));
    g_assert_nonnull(strstr(text, "invalid header: 0xffffffff"));
    unlink(log);
}

/* UHCI0 and GDMA-AHB */
#define UHCI                0x500df000
#define UHCI_CONF0          (UHCI + 0x00)
#define UHCI_INT_RAW        (UHCI + 0x04)
#define UHCI_INT_CLR        (UHCI + 0x10)
#define UHCI_CONF1          (UHCI + 0x14)
#define UHCI_ESCAPE_CONF    (UHCI + 0x20)
#define UHCI_HUNG_CONF      (UHCI + 0x24)
#define UHCI_QUICK_SENT     (UHCI + 0x30)
#define UHCI_Q_WORD(n, w)   (UHCI + 0x34 + 8 * (n) + 4 * (w))
#define UHCI_ESC_CONF0      (UHCI + 0x6c)
#define UHCI_PKT_THRES      (UHCI + 0x7c)
#define UHCI_DATE           (UHCI + 0x80)
#define UHCI_SEL(n)         ((n) << 2)
#define UHCI_SEPER_EN       (1u << 5)
#define UHCI_IDLE_EOF_EN    (1u << 8)
#define UHCI_LEN_EOF_EN     (1u << 9)
#define UHCI_BRK_EOF_EN     (1u << 12)
#define UHCI_INT_RX_START   (1u << 0)
#define UHCI_INT_TX_START   (1u << 1)
#define UHCI_INT_TX_HUNG    (1u << 3)
#define UHCI_INT_SEND_S_Q   (1u << 4)

#define AHB                 0x50085000
#define AHB_IN_INT_RAW(ch)  (AHB + 0x10 * (ch))
#define AHB_OUT_INT_RAW(ch) (AHB + 0x30 + 0x10 * (ch))
#define AHB_IN(ch)          (AHB + 0x70 + 0xc0 * (ch))
#define AHB_OUT(ch)         (AHB + 0xd0 + 0xc0 * (ch))
#define AHB_IN_LINK_ADDR(ch)  (AHB + 0x3ac + 4 * (ch))
#define AHB_OUT_LINK_ADDR(ch) (AHB + 0x3b8 + 4 * (ch))
#define AHB_LINK            0x10
#define AHB_PERI_SEL        0x30
#define AHB_IN_LINK_START   (1u << 2)
#define AHB_OUT_LINK_START  (1u << 1)
#define AHB_PERI_UHCI0      2
#define AHB_IN_SUC_EOF      (1u << 1)
#define AHB_OUT_TOTAL_EOF   (1u << 3)
#define DW0(size, len)      ((size) | (len) << 12)
#define DESC_SUC_EOF        (1u << 30)
#define DESC_OWNER          (1u << 31)
#define MEM                 0x4ff40000

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

/* GDMA-AHB channel 0's TX (or RX) side on UHCI0, its list at @list. */
static void ahb_start(QTestState *qts, bool rx, uint32_t list)
{
    if (rx) {
        qtest_writel(qts, AHB_IN(0) + AHB_PERI_SEL, AHB_PERI_UHCI0);
        qtest_writel(qts, AHB_IN_LINK_ADDR(0), list);
        qtest_writel(qts, AHB_IN(0) + AHB_LINK, AHB_IN_LINK_START);
    } else {
        qtest_writel(qts, AHB_OUT(0) + AHB_PERI_SEL, AHB_PERI_UHCI0);
        qtest_writel(qts, AHB_OUT_LINK_ADDR(0), list);
        qtest_writel(qts, AHB_OUT(0) + AHB_LINK, AHB_OUT_LINK_START);
    }
}

/*
 * UHCI sending: GDMA-AHB's TX channel on UHCI0 feeds UART1's TX FIFO as
 * it has room, here looped back into UART1's RX FIFO; with SEPER_EN the
 * encoder frames the data with separators and escapes 0xC0 and 0xDB.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_uhci_tx(void)
{
    QTestState *qts = start();
    static const uint8_t data[] = { 0x01, 0xc0, 0xdb, 0x02 };
    static const uint8_t line[] = {
        0xc0, 0x01, 0xdb, 0xdc, 0xdb, 0xdd, 0x02, 0xc0,
    };
    uint32_t list = MEM, buf = MEM + 0x100;
    int i;

    g_assert_cmphex(qtest_readl(qts, UHCI_CONF0), ==, 0x6fc);
    g_assert_cmphex(qtest_readl(qts, UHCI_ESC_CONF0), ==, 0x00dcdbc0);
    g_assert_cmphex(qtest_readl(qts, UHCI_HUNG_CONF), ==, 0x00810810);
    g_assert_cmphex(qtest_readl(qts, UHCI_PKT_THRES), ==, 0x80);
    g_assert_cmphex(qtest_readl(qts, UHCI_DATE), ==, 0x02201100);

    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK);
    qtest_memwrite(qts, buf, data, sizeof(data));
    desc(qts, list, DESC_OWNER | DESC_SUC_EOF | DW0(4, 4), buf, 0);
    /* Raw, as ESP-IDF sets it up without separators: no escapes either */
    qtest_writel(qts, UHCI_CONF0, UHCI_SEL(1));
    qtest_writel(qts, UHCI_ESCAPE_CONF, 0);
    ahb_start(qts, false, list);
    qtest_clock_step(qts, 5 * FRAME_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 4);
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(rd(qts, UART(1), FIFO), ==, data[i]);
    }
    g_assert_true(qtest_readl(qts, AHB_OUT_INT_RAW(0)) & AHB_OUT_TOTAL_EOF);

    qtest_writel(qts, UHCI_INT_CLR, 0x1ff);
    qtest_writel(qts, UHCI_CONF0, UHCI_SEL(1) | UHCI_SEPER_EN);
    qtest_writel(qts, UHCI_ESCAPE_CONF, 0x33);
    desc(qts, list, DESC_OWNER | DESC_SUC_EOF | DW0(4, 4), buf, 0);
    ahb_start(qts, false, list);
    qtest_clock_step(qts, 9 * FRAME_NS);
    g_assert_true(qtest_readl(qts, UHCI_INT_RAW) & UHCI_INT_RX_START);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, sizeof(line));
    for (i = 0; i < sizeof(line); i++) {
        g_assert_cmphex(rd(qts, UART(1), FIFO), ==, line[i]);
    }

    /* A short packet: Q2's two words, little-endian, as a frame */
    qtest_writel(qts, UHCI_Q_WORD(2, 0), 0x44332211);
    qtest_writel(qts, UHCI_Q_WORD(2, 1), 0x88776655);
    qtest_writel(qts, UHCI_QUICK_SENT, 2 | (1u << 3));
    qtest_clock_step(qts, 11 * FRAME_NS);
    g_assert_true(qtest_readl(qts, UHCI_INT_RAW) & UHCI_INT_SEND_S_Q);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 10);
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0xc0);
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0x11 * (i + 1));
    }
    g_assert_cmphex(rd(qts, UART(1), FIFO), ==, 0xc0);
    qtest_quit(qts);
}

/*
 * UHCI receiving: UART1's RX FIFO into GDMA-AHB's RX channel on UHCI0,
 * through the decoder; a frame ends at its closing separator, when the
 * UART has idled RX_IDLE_THRHD bits (UART_IDLE_EOF_EN), or after PKT_THRS
 * bytes (LEN_EOF_EN); with no RX channel the bytes stay in the FIFO.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_uhci_rx(void)
{
    QTestState *qts = start();
    static const uint8_t line[] = {
        0xc0, 0x01, 0xdb, 0xdc, 0xdb, 0xdd, 0x02, 0xc0,
    };
    static const uint8_t want[] = { 0x01, 0xc0, 0xdb, 0x02 };
    uint32_t list = MEM, buf = MEM + 0x100;
    uint8_t got[16];
    int i;

    setup(qts, UART(1), CONF0_8N1 | CONF0_LOOPBACK);
    qtest_writel(qts, UHCI_CONF0, UHCI_SEL(1) | UHCI_SEPER_EN);
    /* No RX channel: the UART keeps its bytes for software. */
    wr(qts, UART(1), FIFO, 0x55);
    qtest_clock_step(qts, FRAME_NS + BIT_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 1);
    rd(qts, UART(1), FIFO);

    desc(qts, list, DESC_OWNER | DW0(16, 0), buf, 0);
    ahb_start(qts, true, list);
    for (i = 0; i < sizeof(line); i++) {
        wr(qts, UART(1), FIFO, line[i]);
    }
    qtest_clock_step(qts, 9 * FRAME_NS);
    g_assert_cmpuint(rx_count(qts, UART(1)), ==, 0);
    g_assert_true(qtest_readl(qts, UHCI_INT_RAW) & UHCI_INT_TX_START);
    g_assert_true(qtest_readl(qts, AHB_IN_INT_RAW(0)) & AHB_IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, list), ==, DESC_SUC_EOF | DW0(16, 4));
    qtest_memread(qts, buf, got, 4);
    g_assert_cmpmem(got, 4, want, 4);

    /* Raw, ended by 10 bits of idle */
    wr(qts, UART(1), IDLE_CONF, 10);
    update(qts, UART(1));
    qtest_writel(qts, UHCI_CONF0, UHCI_SEL(1) | UHCI_IDLE_EOF_EN);
    qtest_writel(qts, AHB_IN_INT_RAW(0) + 0xc, 0xff);
    desc(qts, list, DESC_OWNER | DW0(16, 0), buf, 0);
    ahb_start(qts, true, list);
    for (i = 0; i < 3; i++) {
        wr(qts, UART(1), FIFO, 0xa0 + i);
    }
    qtest_clock_step(qts, 3 * FRAME_NS + 5 * BIT_NS);
    g_assert_false(qtest_readl(qts, AHB_IN_INT_RAW(0)) & AHB_IN_SUC_EOF);
    qtest_clock_step(qts, 10 * BIT_NS);
    g_assert_true(qtest_readl(qts, AHB_IN_INT_RAW(0)) & AHB_IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, list), ==, DESC_SUC_EOF | DW0(16, 3));

    /* Raw, ended after PKT_THRS bytes */
    qtest_writel(qts, UHCI_CONF0, UHCI_SEL(1) | UHCI_LEN_EOF_EN);
    qtest_writel(qts, UHCI_PKT_THRES, 2);
    qtest_writel(qts, AHB_IN_INT_RAW(0) + 0xc, 0xff);
    desc(qts, list, DESC_OWNER | DW0(16, 0), buf, 0);
    ahb_start(qts, true, list);
    for (i = 0; i < 3; i++) {
        wr(qts, UART(1), FIFO, 0xb0 + i);
    }
    qtest_clock_step(qts, 3 * FRAME_NS + BIT_NS);
    g_assert_cmphex(qtest_readl(qts, list), ==, DESC_SUC_EOF | DW0(16, 2));
    qtest_quit(qts);
}

/* The USB Serial/JTAG controller */
#define USJ                 0x500d2000
#define USJ_EP1             (USJ + 0x00)
#define USJ_EP1_CONF        (USJ + 0x04)
#define USJ_INT_RAW         (USJ + 0x08)
#define USJ_INT_CLR         (USJ + 0x14)
#define USJ_CONF0           (USJ + 0x18)
#define USJ_TEST            (USJ + 0x1c)
#define USJ_JFIFO_ST        (USJ + 0x20)
#define USJ_FRAM_NUM        (USJ + 0x24)
#define USJ_IN_EP1_ST       (USJ + 0x2c)
#define USJ_OUT_EP1_ST      (USJ + 0x3c)
#define USJ_MEM_CONF        (USJ + 0x48)
#define USJ_CHIP_RST        (USJ + 0x4c)
#define USJ_SET_LINE_W0     (USJ + 0x50)
#define USJ_SET_LINE_W1     (USJ + 0x54)
#define USJ_SER_AFIFO       (USJ + 0x64)
#define USJ_BUS_RESET_ST    (USJ + 0x68)
#define USJ_ECO_HIGH_48     (USJ + 0x70)
#define USJ_SRAM_CTRL       (USJ + 0x84)
#define USJ_DATE            (USJ + 0x88)
#define USJ_WR_DONE         (1u << 0)
#define USJ_IN_FREE         (1u << 1)
#define USJ_OUT_AVAIL       (1u << 2)
#define USJ_INT_SOF         (1u << 1)
#define USJ_INT_RECV_PKT    (1u << 2)
#define USJ_INT_IN_EMPTY    (1u << 3)
#define USJ_INT_BUS_RESET   (1u << 9)
#define USJ_INT_RTS_CHG     (1u << 12)
#define USJ_INT_DTR_CHG     (1u << 13)
#define USJ_INT_GET_LINE    (1u << 14)
#define USJ_INT_SET_LINE    (1u << 15)
#define USJ_PAD_ENABLE      (1u << 14)
#define GPIO_IN             (GPIO + 0x03c)
#define MS                  1000000

/*
 * The controller's registers out of reset, and its IN buffer with no
 * host: the 64th byte flushes it, and it stays full; USB_PAD_ENABLE's
 * D+ pull-up on GPIO25.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_usj_registers(void)
{
    QTestState *qts = start();
    int i;

    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, USJ_IN_FREE);
    g_assert_cmphex(qtest_readl(qts, USJ_INT_RAW), ==, USJ_INT_IN_EMPTY);
    g_assert_cmphex(qtest_readl(qts, USJ_CONF0), ==, 0x4200);
    g_assert_cmphex(qtest_readl(qts, USJ_TEST), ==, 0x30);
    g_assert_cmphex(qtest_readl(qts, USJ_JFIFO_ST), ==, 0x44);
    g_assert_cmphex(qtest_readl(qts, USJ_IN_EP1_ST), ==, 1);
    g_assert_cmphex(qtest_readl(qts, USJ_MEM_CONF), ==, 2);
    g_assert_cmphex(qtest_readl(qts, USJ_SER_AFIFO), ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, USJ_BUS_RESET_ST), ==, 1);
    g_assert_cmphex(qtest_readl(qts, USJ_ECO_HIGH_48), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, USJ_SRAM_CTRL), ==, 0x1320);
    g_assert_cmphex(qtest_readl(qts, USJ_DATE), ==, 0x02112010);

    for (i = 0; i < 63; i++) {
        qtest_writel(qts, USJ_EP1, i);
    }
    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, USJ_IN_FREE);
    qtest_writel(qts, USJ_EP1, 63);
    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, USJ_IN_EP1_ST), ==, 1 | (64 << 2));
    qtest_clock_step(qts, 500 * MS);
    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, 0);
    g_assert_false(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_SOF);

    /* The PHY's D+ pull-up holds GPIO25 high, until the pads go. */
    qtest_writel(qts, IO_MUX_GPIO(25), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    g_assert_true(qtest_readl(qts, GPIO_IN) & (1u << 25));
    qtest_writel(qts, USJ_CONF0, 0x4200 & ~USJ_PAD_ENABLE);
    g_assert_false(qtest_readl(qts, GPIO_IN) & (1u << 25));
    qtest_quit(qts);
}

static QTestState *start_usj_args(int *fd, const char *extra)
{
    g_autofree char *dir = g_dir_make_tmp("esp32p4-usj-XXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/sock", dir);
    int srv = qtest_socket_server(path);
    QTestState *qts;

    qts = qtest_initf("-M esp32p4,flash-size=16M%s -serial none "
                      "-chardev socket,id=usj,path=%s "
                      "-global esp32p4-usb-serial-jtag.chardev=usj", extra,
                      path);
    *fd = accept(srv, NULL, NULL);
    g_assert_cmpint(*fd, >=, 0);
    close(srv);
    unlink(path);
    rmdir(dir);
    return qts;
}

static QTestState *start_usj(int *fd)
{
    return start_usj_args(fd, "");
}

/*
 * The host on the chardev: it attaches after 100 ms, resets the bus for
 * 10 ms, sends SOFs a millisecond apart, enumerates and opens the port
 * (SET_LINE_CODING, GET_LINE_CODING, DTR and RTS); then a flushed IN
 * buffer reaches the chardev and the chardev's bytes arrive as an OUT
 * packet; closing the chardev drops DTR and RTS.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_usj_host(void)
{
    int fd;
    QTestState *qts = start_usj(&fd);
    uint32_t frame;
    char buf[8];
    int i;

    g_assert_false(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_BUS_RESET);
    qtest_clock_step(qts, 100 * MS + 1000);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_BUS_RESET);
    g_assert_cmphex(qtest_readl(qts, USJ_BUS_RESET_ST), ==, 0);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(qtest_readl(qts, USJ_BUS_RESET_ST), ==, 1);
    frame = qtest_readl(qts, USJ_FRAM_NUM);
    qtest_clock_step(qts, 3 * MS);
    g_assert_cmpuint(qtest_readl(qts, USJ_FRAM_NUM), ==, frame + 3);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_SOF);
    qtest_clock_step(qts, 20 * MS);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_SET_LINE);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_GET_LINE);
    g_assert_cmpuint(qtest_readl(qts, USJ_SET_LINE_W0), ==, 115200);
    g_assert_cmphex(qtest_readl(qts, USJ_SET_LINE_W1), ==, 8u << 16);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_DTR_CHG);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_RTS_CHG);
    g_assert_cmphex(qtest_readl(qts, USJ_CHIP_RST), ==, 3);

    /* IN: two bytes and WR_DONE */
    qtest_writel(qts, USJ_INT_CLR, USJ_INT_IN_EMPTY);
    qtest_writel(qts, USJ_EP1, 'h');
    qtest_writel(qts, USJ_EP1, 'i');
    qtest_writel(qts, USJ_EP1_CONF, USJ_WR_DONE);
    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, 0);
    qtest_clock_step(qts, 20000);
    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, USJ_IN_FREE);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_IN_EMPTY);
    g_assert_cmpint(recv(fd, buf, 2, MSG_WAITALL), ==, 2);
    g_assert_cmpmem(buf, 2, "hi", 2);

    /* OUT: three bytes from the chardev */
    g_assert_cmpint(send(fd, "abc", 3, 0), ==, 3);
    for (i = 0; i < 100 && !(qtest_readl(qts, USJ_EP1_CONF) &
                             USJ_OUT_AVAIL); i++) {
        g_usleep(10000);
        qtest_clock_step(qts, 100000);
    }
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_RECV_PKT);
    g_assert_cmphex(qtest_readl(qts, USJ_OUT_EP1_ST), ==,
                    (5 << 2) | (3 << 16));
    g_assert_cmphex(qtest_readl(qts, USJ_EP1), ==, 'a');
    g_assert_cmphex(qtest_readl(qts, USJ_EP1), ==, 'b');
    g_assert_cmphex(qtest_readl(qts, USJ_EP1), ==, 'c');
    g_assert_cmphex(qtest_readl(qts, USJ_EP1_CONF), ==, USJ_IN_FREE);

    /* The terminal closes the port: DTR and RTS drop. */
    qtest_writel(qts, USJ_INT_CLR, USJ_INT_DTR_CHG | USJ_INT_RTS_CHG);
    close(fd);
    for (i = 0; i < 100 && !(qtest_readl(qts, USJ_INT_RAW) &
                             USJ_INT_DTR_CHG); i++) {
        g_usleep(10000);
        qtest_clock_step(qts, MS);
    }
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_DTR_CHG);
    g_assert_cmphex(qtest_readl(qts, USJ_CHIP_RST), ==, 0);
    qtest_quit(qts);
}

/* How long esptool waits for SYNC's response before resending it. */
#define SYNC_WAIT_MS    100

/*
 * Read from @fd into @buf, up to @len bytes, until @want shows or @ms
 * milliseconds pass without it.
 */
static size_t read_until(int fd, char *buf, size_t len, const char *want,
                         size_t want_len, int ms)
{
    size_t got = 0;

    for (int i = 0; i < ms / 100; i++) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };

        if (poll(&pfd, 1, 100) > 0) {
            ssize_t n = recv(fd, buf + got, len - got, 0);

            g_assert_cmpint(n, >, 0);
            got += n;
        }
        if (memmem(buf, got, want, want_len) || got == len) {
            break;
        }
    }
    return got;
}

/*
 * The boot ROM in download mode (strap-mode 0x04, GPIO35 low and GPIO36
 * high) through serial port 0: it says it is waiting, then answers
 * esptool's SLIP-framed SYNC command, received on GPIO38 by UART0 with
 * the ROM's interrupt-driven receiver, with SYNC's response. The ROM
 * detects the baud rate from the first SYNC and answers a later one, so
 * SYNC is resent until answered, as esptool resends it.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_rom_download_sync(void)
{
    static const uint8_t sync_cmd[] = {
        0xc0, 0x00, 0x08, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x07, 0x07, 0x12, 0x20,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0xc0,
    };
    static const char sync_resp[] = { (char)0xc0, 0x01, 0x08 };
    static const char waiting[] = "waiting for download";
    char buf[512];
    size_t got = 0;
    int fd, i;
    QTestState *qts = qtest_init_with_serial(
        "-M esp32p4,flash-size=16M,strap-mode=0x04 -accel tcg "
        "-icount shift=0,sleep=off", &fd);

    got = read_until(fd, buf, sizeof(buf) - 1, waiting, strlen(waiting),
                     30000);
    g_assert_nonnull(memmem(buf, got, "DOWNLOAD(USB/UART0/SPI)", 23));
    g_assert_nonnull(memmem(buf, got, waiting, strlen(waiting)));
    for (i = 0; i < 30000 / SYNC_WAIT_MS; i++) {
        g_assert_cmpint(send(fd, sync_cmd, sizeof(sync_cmd), 0), ==,
                        sizeof(sync_cmd));
        got = read_until(fd, buf, sizeof(buf), sync_resp, sizeof(sync_resp),
                         SYNC_WAIT_MS);
        if (memmem(buf, got, sync_resp, sizeof(sync_resp))) {
            break;
        }
    }
    g_assert_nonnull(memmem(buf, got, sync_resp, sizeof(sync_resp)));
    close(fd);
    qtest_quit(qts);
}

/*
 * The boot ROM in download mode answers esptool's SYNC over the USB
 * Serial/JTAG port too, once the host has enumerated it; its banner went
 * to the IN buffer before that, and the bus reset emptied it.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
static void test_rom_download_sync_usj(void)
{
    static const uint8_t sync_cmd[] = {
        0xc0, 0x00, 0x08, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x07, 0x07, 0x12, 0x20,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0xc0,
    };
    static const char sync_resp[] = { (char)0xc0, 0x01, 0x08 };
    char buf[512];
    size_t got = 0;
    int fd, i;
    QTestState *qts = start_usj_args(&fd, ",strap-mode=0x04 -accel tcg "
                                     "-icount shift=0,sleep=off");

    for (i = 0; i < 30000 / SYNC_WAIT_MS; i++) {
        g_assert_cmpint(send(fd, sync_cmd, sizeof(sync_cmd), 0), ==,
                        sizeof(sync_cmd));
        got = read_until(fd, buf, sizeof(buf), sync_resp, sizeof(sync_resp),
                         SYNC_WAIT_MS);
        if (memmem(buf, got, sync_resp, sizeof(sync_resp))) {
            break;
        }
    }
    g_assert_nonnull(memmem(buf, got, sync_resp, sizeof(sync_resp)));
    close(fd);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/uart/reset_values", test_reset_values);
    qtest_add_func("esp32p4/uart/reg_update", test_reg_update);
    qtest_add_func("esp32p4/uart/baud_timing", test_baud_timing);
    qtest_add_func("esp32p4/uart/fifo_irq", test_fifo_irq);
    qtest_add_func("esp32p4/uart/uart_to_uart", test_uart_to_uart);
    qtest_add_func("esp32p4/uart/hw_flow", test_hw_flow);
    qtest_add_func("esp32p4/uart/sw_flow", test_sw_flow);
    qtest_add_func("esp32p4/uart/autobaud", test_autobaud);
    qtest_add_func("esp32p4/uart/glitch", test_glitch);
    qtest_add_func("esp32p4/uart/rs485", test_rs485);
    qtest_add_func("esp32p4/uart/irda", test_irda);
    qtest_add_func("esp32p4/uart/at_cmd", test_at_cmd);
    qtest_add_func("esp32p4/uart/lp_uart", test_lp_uart);
    qtest_add_func("esp32p4/uart/resets", test_resets);
    qtest_add_func("esp32p4/uart/wakeup", test_wakeup);
    qtest_add_func("esp32p4/uart/pmu_wakeup", test_pmu_wakeup);
    qtest_add_func("esp32p4/uart/serial_port", test_serial_port);
    qtest_add_func("esp32p4/uart/uhci_tx", test_uhci_tx);
    qtest_add_func("esp32p4/uart/uhci_rx", test_uhci_rx);
    qtest_add_func("esp32p4/uart/usj_registers", test_usj_registers);
    qtest_add_func("esp32p4/uart/usj_host", test_usj_host);
    qtest_add_func("esp32p4/uart/rom_banner", test_rom_banner);
    qtest_add_func("esp32p4/uart/rom_download_sync", test_rom_download_sync);
    qtest_add_func("esp32p4/uart/rom_download_sync_usj",
                   test_rom_download_sync_usj);

    return g_test_run();
}
