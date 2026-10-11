/*
 * QTest testcase for the ESP32-P4 GPIO matrix, IO MUX and LP IO
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Peripheral signals are driven through the device's "sig-out" and
 * "lp-sig-out" inputs, pads from outside through "pad-in", and what the
 * block drives is watched on its named outputs, one group per QEMU, as
 * qtest intercepts one device's outputs at a time. The interrupts are
 * seen in the interrupt matrix's status registers. The HP IO MUX
 * operating clock is the 40 MHz crystal out of reset: 25 ns a cycle.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define GPIO_PATH           "/machine/gpio"

#define GPIO                0x500e0000
#define GPIO_BT_SELECT      (GPIO + 0x000)
#define GPIO_OUT            (GPIO + 0x004)
#define GPIO_OUT_W1TS       (GPIO + 0x008)
#define GPIO_OUT_W1TC       (GPIO + 0x00c)
#define GPIO_OUT1           (GPIO + 0x010)
#define GPIO_OUT1_W1TS      (GPIO + 0x014)
#define GPIO_OUT1_W1TC      (GPIO + 0x018)
#define GPIO_ENABLE         (GPIO + 0x020)
#define GPIO_ENABLE_W1TS    (GPIO + 0x024)
#define GPIO_ENABLE_W1TC    (GPIO + 0x028)
#define GPIO_ENABLE1        (GPIO + 0x02c)
#define GPIO_ENABLE1_W1TS   (GPIO + 0x030)
#define GPIO_STRAP          (GPIO + 0x038)
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_IN1            (GPIO + 0x040)
#define GPIO_STATUS         (GPIO + 0x044)
#define GPIO_STATUS_W1TS    (GPIO + 0x048)
#define GPIO_STATUS_W1TC    (GPIO + 0x04c)
#define GPIO_STATUS1        (GPIO + 0x050)
#define GPIO_STATUS1_W1TS   (GPIO + 0x054)
#define GPIO_STATUS1_W1TC   (GPIO + 0x058)
#define GPIO_INTR_0         (GPIO + 0x05c)
#define GPIO_INTR1_0        (GPIO + 0x060)
#define GPIO_INTR_1         (GPIO + 0x064)
#define GPIO_STATUS_NEXT    (GPIO + 0x06c)
#define GPIO_PIN(n)         (GPIO + 0x074 + 4 * (n))
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define GPIO_INTR_2         (GPIO + 0x63c)
#define GPIO_INTR_3         (GPIO + 0x644)
#define GPIO_CLOCK_GATE     (GPIO + 0x64c)
#define GPIO_INT_RAW        (GPIO + 0x700)
#define GPIO_INT_ST         (GPIO + 0x704)
#define GPIO_INT_ENA        (GPIO + 0x708)
#define GPIO_INT_CLR        (GPIO + 0x70c)
#define GPIO_ZERO_DET0      (GPIO + 0x710)
#define GPIO_ZERO_DET1      (GPIO + 0x714)
#define GPIO_SEND_SEQ       (GPIO + 0x718)
#define GPIO_BISTIN_SEL     (GPIO + 0x720)
#define GPIO_BIST_CTRL      (GPIO + 0x724)
#define GPIO_DATE           (GPIO + 0x7fc)
#define GPIO_SIGMADELTA(n)  (GPIO + 0xf00 + 4 * (n))
#define GPIO_SIGMADELTA_MISC (GPIO + 0xf24)
#define GPIO_GLITCH(n)      (GPIO + 0xf30 + 4 * (n))
#define GPIO_ETM_EVENT(n)   (GPIO + 0xf60 + 4 * (n))
#define GPIO_ETM_TASK(n)    (GPIO + 0xfa0 + 4 * (n))
#define GPIO_EXT_VERSION    (GPIO + 0xffc)

#define PIN_PAD_DRIVER      (1u << 2)
#define PIN_INT_TYPE(t)     ((t) << 7)
#define PIN_WAKEUP_ENABLE   (1u << 10)
#define PIN_INT_ENA(b)      (1u << (13 + (b)))
#define INT_RISING          1
#define INT_FALLING         2
#define INT_ANY             3
#define INT_LOW             4
#define INT_HIGH            5

#define IN_SIG_IN_SEL       (1u << 7)
#define IN_INV_SEL          (1u << 6)
#define IN_CONST_LOW        62
#define IN_CONST_HIGH       63
#define OUT_INV_SEL         (1u << 9)
#define OEN_SEL             (1u << 10)
#define OEN_INV_SEL         (1u << 11)
#define SIG_GPIO_OUT        0x100

#define SDM_FUNCTION_CLK_EN (1u << 30)
#define GLITCH_EN           1u
#define GLITCH(pad, thres, width) \
    (GLITCH_EN | ((pad) << 1) | ((thres) << 7) | ((width) << 13))
#define ETM_EVENT_EN        (1u << 7)

#define IO_MUX              0x500e1000
#define IO_MUX_GPIO(n)      (IO_MUX + 0x004 + 4 * (n))
#define IO_MUX_DATE         (IO_MUX + 0x104)
#define MCU_OE              (1u << 0)
#define SLP_SEL             (1u << 1)
#define MCU_WPU             (1u << 3)
#define MCU_IE              (1u << 4)
#define FUN_WPD             (1u << 7)
#define FUN_WPU             (1u << 8)
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((d) << 10)
#define MCU_SEL(f)          ((f) << 12)
#define FILTER_EN           (1u << 15)
#define MCU_SEL_GPIO        MCU_SEL(1)

#define LP_GPIO             0x5012a000
#define LP_GPIO_CLK_EN      (LP_GPIO + 0x000)
#define LP_GPIO_VER_DATE    (LP_GPIO + 0x004)
#define LP_GPIO_OUT         (LP_GPIO + 0x008)
#define LP_GPIO_OUT_W1TS    (LP_GPIO + 0x00c)
#define LP_GPIO_OUT_W1TC    (LP_GPIO + 0x010)
#define LP_GPIO_ENABLE      (LP_GPIO + 0x014)
#define LP_GPIO_ENABLE_W1TS (LP_GPIO + 0x018)
#define LP_GPIO_STATUS      (LP_GPIO + 0x020)
#define LP_GPIO_STATUS_W1TC (LP_GPIO + 0x028)
#define LP_GPIO_STATUS_NEXT (LP_GPIO + 0x02c)
#define LP_GPIO_IN          (LP_GPIO + 0x030)
#define LP_GPIO_PIN(n)      (LP_GPIO + 0x034 + 4 * (n))
#define LP_GPIO_FUNC_IN(s)  (LP_GPIO + 0x074 + 4 * (s))
#define LP_GPIO_FUNC_OUT(n) (LP_GPIO + 0x0f4 + 4 * (n))
#define LP_PIN_WAKEUP       (1u << 0)
#define LP_PIN_INT_TYPE(t)  ((t) << 1)
#define LP_PIN_PAD_DRIVER   (1u << 4)
#define LP_PIN_EDGE_CLR     (1u << 5)
#define LP_IN_INV           (1u << 0)
#define LP_IN_SIG_IN_SEL    (1u << 1)
#define LP_IN_SEL(p)        ((p) << 2)
#define LP_OUT_SEL(s)       ((s) << 3)
#define LP_SIG_GPIO_OUT     32

#define LP_IOMUX            0x5012b000
#define LP_IOMUX_CLK_EN     (LP_IOMUX + 0x000)
#define LP_IOMUX_VER_DATE   (LP_IOMUX + 0x004)
#define LP_IOMUX_PAD(n)     (LP_IOMUX + 0x008 + 4 * (n))
#define LP_IOMUX_EXT_WAKEUP (LP_IOMUX + 0x048)
#define LP_IOMUX_PAD_HOLD   (LP_IOMUX + 0x04c)
#define LP_IOMUX_PAD_HYS    (LP_IOMUX + 0x050)
#define LP_PAD_DRV(d)       (d)
#define LP_PAD_RDE          (1u << 2)
#define LP_PAD_RUE          (1u << 3)
#define LP_PAD_MUX_SEL      (1u << 4)
#define LP_PAD_FUN_SEL(f)   ((f) << 5)
#define LP_PAD_FUN_IE       (1u << 10)
#define LP_PAD_FILTER_EN    (1u << 11)
#define LP_PAD_LP_GPIO      (LP_PAD_MUX_SEL | LP_PAD_FUN_SEL(1))

#define INTMTX_STATUS(w)    (0x500d6200 + 4 * (w))
/* GPIO_INTR0 to _3 and GPIO_PAD_COMP: sources 74 to 78, word 2 */
#define SRC_GPIO_BIT(k)     (1u << (10 + (k)))
/* LP_GPIO_INTR: source 10, word 0 */
#define SRC_LP_GPIO_BIT     (1u << 10)

#define HP_SYS_CLKRST_SOC_CLK_CTRL3 0x500e6020
#define HP_SYS_CLKRST_PERI_CLK_CTRL26 0x500e60ac
#define HP_SYS_CLKRST_HP_RST_EN1    0x500e60c4
#define IOMUX_APB_CLK_EN            (1u << 3)
#define RST_EN_IOMUX                (1u << 3)
#define LPPERI_CLK_EN               0x50120000
#define LPPERI_RESET_EN             0x50120008
#define LP_IOMUX_CK_EN              (1u << 20)
#define RST_EN_LP_IOMUX             (1u << 21)

/* Matrix signals */
#define SIG_UART0_TXD       10
#define SIG_I2S0_I_WS       30
#define SIG_SPI2_CK         53
#define SIG_I2C0_SCL        68
#define SIG_SDM0            72
#define SIG_CONST1          213
#define SIG_CORE_GPIO0      214
#define SIG_LOOP            250
#define LP_SIG_UART_TXD     2
#define LP_SIG_SPI_CK       5

#define CYCLE_NS            25

/*
 * A board with no serial port on UART0's pads: a connected one's bridge
 * drives GPIO38 high, which GPIO_STRAP would latch.
 */
static QTestState *start_plain(void)
{
    return qtest_init("-M esp32p4,flash-size=16M -serial none");
}

static void pad_in(QTestState *qts, int n, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", n, level);
}

static void pad_release(QTestState *qts, int n)
{
    qtest_set_irq_in(qts, GPIO_PATH, "pad-release", n, 1);
}

static void sig_out(QTestState *qts, int sig, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, "sig-out", sig, level);
}

static void sig_oe(QTestState *qts, int sig, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, "sig-oe", sig, level);
}

static bool gpio_in(QTestState *qts, int n)
{
    return n < 32 ? (qtest_readl(qts, GPIO_IN) >> n) & 1
                  : (qtest_readl(qts, GPIO_IN1) >> (n - 32)) & 1;
}

static bool lp_in(QTestState *qts, int n)
{
    return (qtest_readl(qts, LP_GPIO_IN) >> n) & 1;
}

/* Make pad n a plain GPIO with its input enabled, driving nothing. */
static void gpio_mode(QTestState *qts, int n, uint32_t extra)
{
    qtest_writel(qts, IO_MUX_GPIO(n), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2) |
                 extra);
    qtest_writel(qts, GPIO_FUNC_OUT(n), SIG_GPIO_OUT);
}

static uint32_t src_status(QTestState *qts, int word)
{
    return qtest_readl(qts, INTMTX_STATUS(word));
}

/* A clock's period, in nanoseconds rounded up */
static uint64_t clock_period_ns(QTestState *qts, const char *path)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         path);
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    g_assert_cmpuint(period, !=, 0);
    return (period + 0xffffffffu) >> 32;
}

/* Every register at its reset value */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_reset_values(void)
{
    QTestState *qts = start_plain();

    g_assert_cmphex(qtest_readl(qts, GPIO_BT_SELECT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_PIN(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_PIN(56)), ==, 0);
    /* Each input signal's matrix source is its default as a constant. */
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(1)), ==, IN_CONST_HIGH);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(10)), ==, IN_CONST_LOW);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(68)), ==, IN_CONST_HIGH);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(255)), ==, IN_CONST_LOW);
    /* Signals with no input have no register. */
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(212)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_OUT(0)), ==, SIG_GPIO_OUT);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_OUT(56)), ==, SIG_GPIO_OUT);
    g_assert_cmphex(qtest_readl(qts, GPIO_CLOCK_GATE), ==, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_ENA), ==, 0xff);
    g_assert_cmphex(qtest_readl(qts, GPIO_ZERO_DET0), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_ZERO_DET1), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_SEND_SEQ), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, GPIO_BISTIN_SEL), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, GPIO_BIST_CTRL), ==, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_DATE), ==, 0x241226);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA(0)), ==, 0xff00);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA(7)), ==, 0xff00);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA_MISC), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_GLITCH(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_EXT_VERSION), ==, 0x2203050);

    /* Table 10.14-1's resets: the JTAG pads, the USB PHY's */
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(0)), ==, FUN_DRV(2));
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(2)), ==,
                    FUN_DRV(2) | FUN_IE | FUN_WPU);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(3)), ==, FUN_DRV(2) | FUN_IE);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(4)), ==, FUN_DRV(2) | FUN_IE);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(5)), ==, FUN_DRV(2));
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(24)), ==, FUN_DRV(3));
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(25)), ==, FUN_DRV(3));
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(56)), ==, FUN_DRV(2));
    g_assert_cmphex(qtest_readl(qts, IO_MUX_DATE), ==, 0x201222);

    g_assert_cmphex(qtest_readl(qts, LP_GPIO_CLK_EN), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_VER_DATE), ==, 0x230323);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_IN(0)), ==, LP_IN_SEL(0x30));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_IN(2)), ==, LP_IN_SEL(0x20));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_IN(4)), ==, LP_IN_SEL(0x30));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_IN(13)), ==,
                    LP_IN_SEL(0x20));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_OUT(0)), ==,
                    LP_OUT_SEL(LP_SIG_GPIO_OUT));
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_CLK_EN), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_VER_DATE), ==, 0x230313);
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_PAD(0)), ==, LP_PAD_DRV(2));
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_PAD(15)), ==, LP_PAD_DRV(2));
    qtest_quit(qts);
}

/* Write masks, set and clear registers, byte lanes, the read-only ones */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_register_access(void)
{
    QTestState *qts = start_plain();

    qtest_writel(qts, GPIO_OUT, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0xffffffff);
    qtest_writel(qts, GPIO_OUT_W1TC, 0x0000ff00);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0xffff00ff);
    qtest_writel(qts, GPIO_OUT_W1TS, 0x00000100);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0xffff01ff);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT_W1TS), ==, 0);
    /* GPIO32 to GPIO56: 25 bits */
    qtest_writel(qts, GPIO_OUT1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT1), ==, 0x1ffffff);
    qtest_writel(qts, GPIO_OUT1_W1TC, 0x1000000);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT1), ==, 0x0ffffff);
    qtest_writel(qts, GPIO_ENABLE1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_ENABLE1), ==, 0x1ffffff);

    /* GPIO_PINn: bits 5 and 6 and 18 up are reserved. */
    qtest_writel(qts, GPIO_PIN(7), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_PIN(7)), ==, 0x3ff9f);
    qtest_writel(qts, GPIO_FUNC_IN(30), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(30)), ==, 0xff);
    qtest_writel(qts, GPIO_FUNC_IN(30), IN_CONST_LOW);
    qtest_writel(qts, GPIO_FUNC_OUT(7), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_OUT(7)), ==, 0xfff);
    qtest_writel(qts, GPIO_FUNC_OUT(7), SIG_GPIO_OUT);
    /* A signal without an input has no register to write. */
    qtest_writel(qts, GPIO_FUNC_IN(0), IN_SIG_IN_SEL);
    g_assert_cmphex(qtest_readl(qts, GPIO_FUNC_IN(0)), ==, 0);

    /* A byte write changes its own byte only. */
    qtest_writeb(qts, GPIO_BT_SELECT + 2, 0x5a);
    g_assert_cmphex(qtest_readl(qts, GPIO_BT_SELECT), ==, 0x005a0000);
    g_assert_cmphex(qtest_readb(qts, GPIO_BT_SELECT + 2), ==, 0x5a);
    qtest_writew(qts, GPIO_SIGMADELTA(1), 0x1234);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA(1)), ==, 0x1234);

    /* Read-only registers keep their values. */
    qtest_writel(qts, GPIO_STRAP, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x08);
    qtest_writel(qts, GPIO_INTR_0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR_0), ==, 0);

    /* GPIO_EXT and the IO MUX */
    qtest_writel(qts, GPIO_GLITCH(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_GLITCH(3)), ==, 0x7ffff);
    qtest_writel(qts, GPIO_GLITCH(3), 0);
    qtest_writel(qts, GPIO_ETM_EVENT(2), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_ETM_EVENT(2)), ==, 0xbf);
    qtest_writel(qts, GPIO_ETM_EVENT(2), 0);
    qtest_writel(qts, GPIO_ETM_TASK(13), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_ETM_TASK(13)), ==, 0x0f0f0f0f);
    qtest_writel(qts, GPIO_ETM_TASK(13), 0);
    qtest_writel(qts, GPIO_SIGMADELTA_MISC, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA_MISC), ==, 0xc0000000);
    qtest_writel(qts, IO_MUX_GPIO(40), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(40)), ==, 0x7ffff);

    /* The LP registers */
    qtest_writel(qts, LP_GPIO_OUT, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0xffff);
    qtest_writel(qts, LP_GPIO_OUT_W1TC, 0x00f0);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0xff0f);
    qtest_writel(qts, LP_GPIO_PIN(3), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_PIN(3)), ==, 0x1f);
    qtest_writel(qts, LP_GPIO_FUNC_IN(5), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_IN(5)), ==, 0xff);
    qtest_writel(qts, LP_GPIO_FUNC_OUT(5), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_FUNC_OUT(5)), ==, 0x1ff);
    qtest_writel(qts, LP_IOMUX_PAD(9), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_PAD(9)), ==, 0xfff);
    qtest_writel(qts, LP_IOMUX_EXT_WAKEUP, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_EXT_WAKEUP), ==, 0x3ff);
    qtest_writel(qts, LP_IOMUX_PAD_HYS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_IOMUX_PAD_HYS), ==, 0xffff);
    qtest_quit(qts);
}

/*
 * GPIO_STRAP latches GPIO38 to GPIO34 at reset: SPI boot by default, the
 * board's strapping from the machine's strap-mode, and a pad driven from
 * outside at its level; a block reset does not latch it again.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_strap(void)
{
    QTestState *qts = start_plain();

    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x08);
    qtest_quit(qts);

    qts = qtest_init("-M esp32p4,flash-size=16M,strap-mode=0x15");
    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x15);
    /* The IO MUX block's reset leaves it. */
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, RST_EN_IOMUX);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x15);
    qtest_quit(qts);

    /* GPIO35 (bit 3) held low and GPIO38 (bit 0) high from outside */
    qts = qtest_initf("-M esp32p4,flash-size=16M "
                      "-global esp32p4-gpio.pad-ext-driven=0x%" PRIx64
                      " -global esp32p4-gpio.pad-ext-level=0x%" PRIx64,
                      (1ULL << 35) | (1ULL << 38), 1ULL << 38);
    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x01);
    qtest_quit(qts);
}

/*
 * A pad's level: the chip's driver with its output enabled, an outside
 * driver over it, the pulls, open drain; and GPIO_IN only with the input
 * enabled.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_pad_levels(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    gpio_mode(qts, 20, 0);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 20);
    g_assert_false(gpio_in(qts, 20));
    g_assert_false(qtest_get_irq(qts, 20));
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 20);
    g_assert_true(gpio_in(qts, 20));
    g_assert_true(qtest_get_irq(qts, 20));
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 20);
    g_assert_false(gpio_in(qts, 20));
    g_assert_false(qtest_get_irq(qts, 20));

    /* Driven from outside, over the chip's own driver */
    pad_in(qts, 20, 1);
    g_assert_true(gpio_in(qts, 20));
    g_assert_true(qtest_get_irq(qts, 20));
    pad_release(qts, 20);
    g_assert_false(gpio_in(qts, 20));

    /* The input disabled reads low, the pad still at its level. */
    pad_in(qts, 20, 1);
    qtest_writel(qts, IO_MUX_GPIO(20), MCU_SEL_GPIO);
    g_assert_false(gpio_in(qts, 20));
    g_assert_true(qtest_get_irq(qts, 20));
    pad_release(qts, 20);

    /* Pulls on an undriven pad */
    qtest_writel(qts, GPIO_ENABLE_W1TC, 1u << 20);
    gpio_mode(qts, 20, FUN_WPU);
    g_assert_true(gpio_in(qts, 20));
    gpio_mode(qts, 20, FUN_WPD);
    g_assert_false(gpio_in(qts, 20));
    gpio_mode(qts, 20, 0);
    g_assert_false(gpio_in(qts, 20));

    /* Open drain: high releases the pad to its pull, low drives it. */
    gpio_mode(qts, 20, FUN_WPU);
    qtest_writel(qts, GPIO_PIN(20), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 20);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 20);
    g_assert_true(gpio_in(qts, 20));
    pad_in(qts, 20, 0);
    g_assert_false(gpio_in(qts, 20));
    pad_release(qts, 20);
    gpio_mode(qts, 20, 0);
    g_assert_false(gpio_in(qts, 20));
    gpio_mode(qts, 20, FUN_WPU);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 20);
    g_assert_false(gpio_in(qts, 20));

    /* A high pad (GPIO40, GPIO_IN1 bit 8) */
    gpio_mode(qts, 40, 0);
    qtest_writel(qts, GPIO_OUT1_W1TS, 1u << 8);
    qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << 8);
    g_assert_true(gpio_in(qts, 40));
    g_assert_true(qtest_get_irq(qts, 40));
    qtest_quit(qts);
}

/* pad-ext-driven and pad-ext-level drive pads, set while QEMU runs too. */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_pad_properties(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M "
                                 "-global esp32p4-gpio.pad-ext-driven=0x30 "
                                 "-global esp32p4-gpio.pad-ext-level=0x10");
    QDict *rsp;

    gpio_mode(qts, 4, 0);
    gpio_mode(qts, 5, FUN_WPU);
    g_assert_true(gpio_in(qts, 4));
    g_assert_false(gpio_in(qts, 5));
    rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                    "{ 'path': '" GPIO_PATH "', 'property': "
                    "'pad-ext-level', 'value': 32 } }");
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    g_assert_false(gpio_in(qts, 4));
    g_assert_true(gpio_in(qts, 5));
    rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                    "{ 'path': '" GPIO_PATH "', 'property': "
                    "'pad-ext-driven', 'value': 0 } }");
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    g_assert_false(gpio_in(qts, 4));
    g_assert_true(gpio_in(qts, 5));
    /* A mask beyond the pads is refused. */
    rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': "
                    "{ 'path': '" GPIO_PATH "', 'property': "
                    "'pad-ext-driven', 'value': %" PRIu64 " } }",
                    1ULL << 55);
    g_assert(qdict_haskey(rsp, "error"));
    qobject_unref(rsp);
    qtest_quit(qts);
}

/*
 * Output routing: a peripheral's output signal to a pad through the
 * matrix, inverted; its own output enable, GPIO_ENABLE in its place,
 * inverted; and the IO MUX's direct path for UART0's TXD on GPIO37,
 * high from reset.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_output_routing(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    /* UART0 TXD idles high on GPIO37, function 0, from reset. */
    g_assert_true(gpio_in(qts, 37) == false);
    qtest_writel(qts, IO_MUX_GPIO(37), FUN_DRV(2) | FUN_IE);
    g_assert_true(gpio_in(qts, 37));
    sig_out(qts, SIG_UART0_TXD, 0);
    g_assert_false(gpio_in(qts, 37));
    g_assert_false(qtest_get_irq(qts, 37));
    sig_out(qts, SIG_UART0_TXD, 1);
    g_assert_true(qtest_get_irq(qts, 37));
    /* In function 1 it is a GPIO, from GPIO_OUT, undriven. */
    qtest_writel(qts, IO_MUX_GPIO(37), MCU_SEL_GPIO | FUN_IE);
    g_assert_false(gpio_in(qts, 37));

    /* Signal 30 (1'd1 enable) through the matrix to GPIO21 */
    gpio_mode(qts, 21, 0);
    qtest_writel(qts, GPIO_FUNC_OUT(21), SIG_I2S0_I_WS);
    g_assert_false(gpio_in(qts, 21));
    sig_out(qts, SIG_I2S0_I_WS, 1);
    g_assert_true(gpio_in(qts, 21));
    g_assert_true(qtest_get_irq(qts, 21));
    qtest_writel(qts, GPIO_FUNC_OUT(21), SIG_I2S0_I_WS | OUT_INV_SEL);
    g_assert_false(gpio_in(qts, 21));
    /* One signal to two pads */
    gpio_mode(qts, 22, 0);
    qtest_writel(qts, GPIO_FUNC_OUT(22), SIG_I2S0_I_WS);
    g_assert_true(gpio_in(qts, 22));

    /* SPI2's clock has an enable of its own. */
    gpio_mode(qts, 23, FUN_WPU);
    qtest_writel(qts, GPIO_FUNC_OUT(23), SIG_SPI2_CK);
    sig_out(qts, SIG_SPI2_CK, 0);
    g_assert_false(gpio_in(qts, 23));
    sig_oe(qts, SIG_SPI2_CK, 0);
    g_assert_true(gpio_in(qts, 23));
    /* OEN_SEL: GPIO_ENABLE decides instead, and OEN_INV_SEL inverts it. */
    qtest_writel(qts, GPIO_FUNC_OUT(23), SIG_SPI2_CK | OEN_SEL);
    g_assert_true(gpio_in(qts, 23));
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 23);
    g_assert_false(gpio_in(qts, 23));
    qtest_writel(qts, GPIO_FUNC_OUT(23), SIG_SPI2_CK | OEN_SEL | OEN_INV_SEL);
    g_assert_true(gpio_in(qts, 23));

    /* The constant 1 output signal */
    gpio_mode(qts, 24, 0);
    qtest_writel(qts, GPIO_FUNC_OUT(24), SIG_CONST1);
    g_assert_true(gpio_in(qts, 24));
    qtest_quit(qts);
}

/*
 * Input routing: a pad to an input signal through the matrix, inverted;
 * the constant sources; the default when bypassing the matrix; and the IO
 * MUX's direct path for UART0's RXD on GPIO38.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_input_routing(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "sig-in");
    gpio_mode(qts, 20, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0_I_WS), IN_SIG_IN_SEL | 20);
    pad_in(qts, 20, 1);
    g_assert_true(qtest_get_irq(qts, SIG_I2S0_I_WS));
    pad_in(qts, 20, 0);
    g_assert_false(qtest_get_irq(qts, SIG_I2S0_I_WS));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0_I_WS),
                 IN_SIG_IN_SEL | IN_INV_SEL | 20);
    g_assert_true(qtest_get_irq(qts, SIG_I2S0_I_WS));
    /* The input disabled, the pad reads low to the matrix. */
    qtest_writel(qts, IO_MUX_GPIO(20), MCU_SEL_GPIO);
    pad_in(qts, 20, 1);
    g_assert_true(qtest_get_irq(qts, SIG_I2S0_I_WS));
    /* Constants */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0_I_WS),
                 IN_SIG_IN_SEL | IN_CONST_LOW);
    g_assert_false(qtest_get_irq(qts, SIG_I2S0_I_WS));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0_I_WS),
                 IN_SIG_IN_SEL | IN_CONST_HIGH);
    g_assert_true(qtest_get_irq(qts, SIG_I2S0_I_WS));
    /* Bypassing the matrix with no IO MUX path: the default, 0 */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0_I_WS), IN_CONST_HIGH);
    g_assert_false(qtest_get_irq(qts, SIG_I2S0_I_WS));
    /* ... and I2C0 SCL's, 1 */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SCL), IN_SIG_IN_SEL | IN_CONST_LOW);
    g_assert_false(qtest_get_irq(qts, SIG_I2C0_SCL));
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SCL), 0);
    g_assert_true(qtest_get_irq(qts, SIG_I2C0_SCL));

    /* UART0 RXD from GPIO38 in function 0 (its reset), input enabled */
    qtest_writel(qts, IO_MUX_GPIO(38), FUN_DRV(2) | FUN_IE);
    pad_in(qts, 38, 1);
    g_assert_true(qtest_get_irq(qts, SIG_UART0_TXD));
    pad_in(qts, 38, 0);
    g_assert_false(qtest_get_irq(qts, SIG_UART0_TXD));
    /* In the GPIO function the direct path is gone. */
    qtest_writel(qts, IO_MUX_GPIO(38), MCU_SEL_GPIO | FUN_IE);
    pad_in(qts, 38, 1);
    g_assert_false(qtest_get_irq(qts, SIG_UART0_TXD));
    qtest_quit(qts);
}

/*
 * Signals 250 to 255 loop back: a pad through input signal 250 to output
 * signal 250 and out on another pad.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_loopback(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    gpio_mode(qts, 20, 0);
    gpio_mode(qts, 30, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_LOOP), IN_SIG_IN_SEL | 20);
    qtest_writel(qts, GPIO_FUNC_OUT(30), SIG_LOOP);
    pad_in(qts, 20, 1);
    g_assert_true(qtest_get_irq(qts, 30));
    g_assert_true(gpio_in(qts, 30));
    pad_in(qts, 20, 0);
    g_assert_false(qtest_get_irq(qts, 30));
    qtest_quit(qts);
}

/*
 * Pad interrupts: rising, falling and any edges, levels that set the
 * status again while they hold, GPIO_STATUS_NEXT, and INT_ENA's bits 13,
 * 14, 16 and 17 to GPIO_INTR0 to _3, sources 74 to 77 of the interrupt
 * matrix, bit 15 to none.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_interrupts(void)
{
    QTestState *qts = start_plain();

    gpio_mode(qts, 6, 0);
    pad_in(qts, 6, 0);
    qtest_writel(qts, GPIO_PIN(6), PIN_INT_TYPE(INT_RISING) | PIN_INT_ENA(0));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    pad_in(qts, 6, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 6);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR_0), ==, 1u << 6);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR_1), ==, 0);
    g_assert_cmphex(src_status(qts, 2), ==, SRC_GPIO_BIT(0));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);
    g_assert_cmphex(src_status(qts, 2), ==, 0);
    pad_in(qts, 6, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);

    qtest_writel(qts, GPIO_PIN(6), PIN_INT_TYPE(INT_FALLING) | PIN_INT_ENA(1));
    pad_in(qts, 6, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    pad_in(qts, 6, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR_1), ==, 1u << 6);
    g_assert_cmphex(src_status(qts, 2), ==, SRC_GPIO_BIT(1));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);

    qtest_writel(qts, GPIO_PIN(6), PIN_INT_TYPE(INT_ANY) | PIN_INT_ENA(3));
    pad_in(qts, 6, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR_2), ==, 1u << 6);
    g_assert_cmphex(src_status(qts, 2), ==, SRC_GPIO_BIT(2));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);
    pad_in(qts, 6, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 6);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);
    /* An edge's condition lasts one cycle: STATUS_NEXT never shows it. */
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS_NEXT), ==, 0);

    /* A high level sets the status again as long as it holds. */
    qtest_writel(qts, GPIO_PIN(6), PIN_INT_TYPE(INT_HIGH) | PIN_INT_ENA(4));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    pad_in(qts, 6, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS_NEXT), ==, 1u << 6);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR_3), ==, 1u << 6);
    g_assert_cmphex(src_status(qts, 2), ==, SRC_GPIO_BIT(3));
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 6);
    pad_in(qts, 6, 0);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    g_assert_cmphex(src_status(qts, 2), ==, 0);

    /* Low level, enabled on bit 15, which feeds none of the four */
    qtest_writel(qts, GPIO_PIN(6), PIN_INT_TYPE(INT_LOW) | PIN_INT_ENA(2));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 6);
    g_assert_cmphex(src_status(qts, 2), ==, 0);
    qtest_writel(qts, GPIO_PIN(6), 0);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 6);

    /* Software sets a status bit; a high pad's in GPIO_STATUS1. */
    qtest_writel(qts, GPIO_PIN(45), PIN_INT_ENA(0));
    qtest_writel(qts, GPIO_STATUS1_W1TS, 1u << (45 - 32));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS1), ==, 1u << 13);
    g_assert_cmphex(qtest_readl(qts, GPIO_INTR1_0), ==, 1u << 13);
    g_assert_cmphex(src_status(qts, 2), ==, SRC_GPIO_BIT(0));
    qtest_writel(qts, GPIO_STATUS1_W1TC, 1u << 13);
    g_assert_cmphex(src_status(qts, 2), ==, 0);

    /* The input disabled, the pad's edges are not seen. */
    qtest_writel(qts, GPIO_PIN(6), PIN_INT_TYPE(INT_ANY) | PIN_INT_ENA(0));
    qtest_writel(qts, IO_MUX_GPIO(6), MCU_SEL_GPIO);
    pad_in(qts, 6, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    qtest_quit(qts);
}

/*
 * The pin filter passes a level held for two IO MUX clock cycles and
 * drops a pulse of one.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_pin_filter(void)
{
    QTestState *qts = start_plain();

    gpio_mode(qts, 20, FILTER_EN);
    qtest_writel(qts, GPIO_PIN(20), PIN_INT_TYPE(INT_ANY));
    pad_in(qts, 20, 1);
    g_assert_false(gpio_in(qts, 20));
    qtest_clock_step(qts, CYCLE_NS);
    g_assert_false(gpio_in(qts, 20));
    qtest_clock_step(qts, 2 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 20);
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 20);

    /* A one-cycle low pulse */
    pad_in(qts, 20, 0);
    qtest_clock_step(qts, CYCLE_NS);
    pad_in(qts, 20, 1);
    qtest_clock_step(qts, 10 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);

    /* Without the filter, at once */
    gpio_mode(qts, 20, 0);
    pad_in(qts, 20, 0);
    g_assert_false(gpio_in(qts, 20));
    qtest_quit(qts);
}

/*
 * A glitch filter with a window of 4 samples and a threshold of 3: a
 * two-cycle pulse does not pass, a level held for three cycles does, after
 * them; it filters the pad's interrupts and its matrix input too.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_glitch_filter(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "sig-in");
    gpio_mode(qts, 20, 0);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0_I_WS), IN_SIG_IN_SEL | 20);
    qtest_writel(qts, GPIO_PIN(20), PIN_INT_TYPE(INT_ANY));
    qtest_writel(qts, GPIO_GLITCH(0), GLITCH(20, 2, 3));

    pad_in(qts, 20, 1);
    qtest_clock_step(qts, 2 * CYCLE_NS);
    pad_in(qts, 20, 0);
    qtest_clock_step(qts, 20 * CYCLE_NS);
    g_assert_false(gpio_in(qts, 20));
    g_assert_false(qtest_get_irq(qts, SIG_I2S0_I_WS));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);

    pad_in(qts, 20, 1);
    qtest_clock_step(qts, 2 * CYCLE_NS);
    g_assert_false(gpio_in(qts, 20));
    qtest_clock_step(qts, 2 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));
    g_assert_true(qtest_get_irq(qts, SIG_I2S0_I_WS));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 20);

    /*
     * A threshold below the window, 2 of 6 samples: one sample low does
     * not pass, two not consecutive do, and the input's level, held,
     * takes the output back.
     */
    qtest_writel(qts, GPIO_STATUS_W1TC, 1u << 20);
    qtest_writel(qts, GPIO_GLITCH(0), GLITCH(20, 1, 5));
    pad_in(qts, 20, 0);
    qtest_clock_step(qts, CYCLE_NS);
    pad_in(qts, 20, 1);
    qtest_clock_step(qts, 10 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 0);
    pad_in(qts, 20, 0);
    qtest_clock_step(qts, CYCLE_NS);
    pad_in(qts, 20, 1);
    qtest_clock_step(qts, CYCLE_NS);
    pad_in(qts, 20, 0);
    qtest_clock_step(qts, CYCLE_NS);
    pad_in(qts, 20, 1);
    g_assert_false(gpio_in(qts, 20));
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS), ==, 1u << 20);
    qtest_clock_step(qts, 2 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));

    /* Disabled, the pad's level passes at once. */
    qtest_writel(qts, GPIO_GLITCH(0), 0);
    g_assert_true(gpio_in(qts, 20));
    qtest_quit(qts);
}

/*
 * A sigma-delta modulator on a pad: SD0_IN -64 with no prescaling gives a
 * pulse every fourth IO MUX clock cycle, a quarter of the time; with
 * PRESCALE 3 every sixteenth cycle; and with its clock disabled it stops.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_sigma_delta(void)
{
    QTestState *qts = start_plain();
    int ones = 0;

    gpio_mode(qts, 21, 0);
    qtest_writel(qts, GPIO_FUNC_OUT(21), SIG_SDM0);
    qtest_writel(qts, GPIO_SIGMADELTA(0), 0xc0);
    /* Its clock is off from reset: no ticks. */
    qtest_clock_step(qts, 100 * CYCLE_NS);
    g_assert_false(gpio_in(qts, 21));

    qtest_writel(qts, GPIO_SIGMADELTA_MISC, SDM_FUNCTION_CLK_EN);
    for (int i = 1; i <= 256; i++) {
        bool v;

        qtest_clock_step(qts, CYCLE_NS);
        v = gpio_in(qts, 21);
        g_assert_cmpint(v, ==, i % 4 == 0);
        ones += v;
    }
    g_assert_cmpint(ones, ==, 64);

    /* PRESCALE 3: a tick every 4 cycles, a pulse every 4 ticks */
    qtest_writel(qts, GPIO_SIGMADELTA(0), (3 << 8) | 0xc0);
    ones = 0;
    for (int i = 1; i <= 256; i++) {
        qtest_clock_step(qts, CYCLE_NS);
        ones += gpio_in(qts, 21);
    }
    g_assert_cmpint(ones, ==, 64);

    /* +127: all but one tick in 256; -128: none */
    qtest_writel(qts, GPIO_SIGMADELTA(0), 0x7f);
    ones = 0;
    for (int i = 0; i < 256; i++) {
        qtest_clock_step(qts, CYCLE_NS);
        ones += gpio_in(qts, 21);
    }
    g_assert_cmpint(ones, ==, 255);
    qtest_writel(qts, GPIO_SIGMADELTA(0), 0x80);
    ones = 0;
    for (int i = 0; i < 256; i++) {
        qtest_clock_step(qts, CYCLE_NS);
        ones += gpio_in(qts, 21);
    }
    g_assert_cmpint(ones, ==, 0);

    /* Its clock disabled, it holds its output. */
    qtest_writel(qts, GPIO_SIGMADELTA(0), 0x00);
    qtest_clock_step(qts, CYCLE_NS);
    qtest_clock_step(qts, CYCLE_NS);
    g_assert_true(gpio_in(qts, 21));
    qtest_writel(qts, GPIO_SIGMADELTA_MISC, 0);
    qtest_clock_step(qts, 7 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 21));
    qtest_quit(qts);
}

/*
 * LP IO: LP_IOMUX_PADn_MUX_SEL hands GPIO5 to the LP IO MUX, after which
 * LP_GPIO_OUT and LP_GPIO_ENABLE drive it and the HP registers do not; its
 * input reaches LP_GPIO_IN and the LP interrupt (source 10); an LP
 * peripheral's signals through the LP matrix; the LP UART's TXD on GPIO14
 * in function 0; and the pad handed back.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_lp_io(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    gpio_mode(qts, 5, 0);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 5);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 5);
    g_assert_true(qtest_get_irq(qts, 5));

    qtest_writel(qts, LP_IOMUX_PAD(5), LP_PAD_LP_GPIO | LP_PAD_FUN_IE);
    g_assert_false(qtest_get_irq(qts, 5));
    qtest_writel(qts, LP_GPIO_ENABLE_W1TS, 1u << 5);
    qtest_writel(qts, LP_GPIO_OUT_W1TS, 1u << 5);
    g_assert_true(qtest_get_irq(qts, 5));
    g_assert_true(lp_in(qts, 5));
    /* GPIO_IN sees the pad too, through the one input buffer. */
    g_assert_true(gpio_in(qts, 5));
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 5);
    g_assert_true(qtest_get_irq(qts, 5));
    qtest_writel(qts, LP_GPIO_OUT_W1TC, 1u << 5);
    g_assert_false(qtest_get_irq(qts, 5));

    /* The LP pulls and open drain */
    qtest_writel(qts, LP_IOMUX_PAD(5), LP_PAD_LP_GPIO | LP_PAD_FUN_IE |
                 LP_PAD_RUE);
    qtest_writel(qts, LP_GPIO_PIN(5), LP_PIN_PAD_DRIVER);
    qtest_writel(qts, LP_GPIO_OUT_W1TS, 1u << 5);
    g_assert_true(lp_in(qts, 5));
    qtest_writel(qts, LP_GPIO_OUT_W1TC, 1u << 5);
    g_assert_false(lp_in(qts, 5));
    qtest_writel(qts, LP_GPIO_PIN(5), 0);

    /* An LP interrupt, rising edge */
    qtest_writel(qts, LP_GPIO_ENABLE, 0);
    qtest_writel(qts, LP_IOMUX_PAD(5), LP_PAD_LP_GPIO | LP_PAD_FUN_IE);
    qtest_writel(qts, LP_GPIO_PIN(5), LP_PIN_INT_TYPE(INT_RISING));
    g_assert_cmphex(src_status(qts, 0) & SRC_LP_GPIO_BIT, ==, 0);
    pad_in(qts, 5, 1);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_STATUS), ==, 1u << 5);
    g_assert_cmphex(src_status(qts, 0) & SRC_LP_GPIO_BIT, ==,
                    SRC_LP_GPIO_BIT);
    qtest_writel(qts, LP_GPIO_STATUS_W1TC, 1u << 5);
    g_assert_cmphex(src_status(qts, 0) & SRC_LP_GPIO_BIT, ==, 0);
    qtest_writel(qts, LP_GPIO_PIN(5), LP_PIN_INT_TYPE(INT_HIGH));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_STATUS_NEXT), ==, 1u << 5);
    qtest_writel(qts, LP_GPIO_PIN(5), 0);
    qtest_writel(qts, LP_GPIO_STATUS_W1TC, 1u << 5);
    pad_release(qts, 5);

    /* LP SPI's clock out through the LP matrix, its own enable on */
    qtest_writel(qts, LP_GPIO_FUNC_OUT(5), LP_OUT_SEL(LP_SIG_SPI_CK));
    qtest_set_irq_in(qts, GPIO_PATH, "lp-sig-out", LP_SIG_SPI_CK, 1);
    g_assert_true(qtest_get_irq(qts, 5));
    qtest_set_irq_in(qts, GPIO_PATH, "lp-sig-oe", LP_SIG_SPI_CK, 0);
    g_assert_false(qtest_get_irq(qts, 5));

    /* The LP UART's TXD, high out of reset, on GPIO14 in function 0 */
    qtest_writel(qts, LP_IOMUX_PAD(14), LP_PAD_MUX_SEL | LP_PAD_FUN_IE);
    g_assert_true(qtest_get_irq(qts, 14));
    qtest_set_irq_in(qts, GPIO_PATH, "lp-sig-out", LP_SIG_UART_TXD, 0);
    g_assert_false(qtest_get_irq(qts, 14));
    g_assert_false(lp_in(qts, 14));

    /* Handed back to the HP IO MUX, GPIO5 is GPIO_OUT's again. */
    qtest_writel(qts, LP_IOMUX_PAD(5), 0);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 5);
    g_assert_true(qtest_get_irq(qts, 5));
    g_assert_true(gpio_in(qts, 5));
    qtest_quit(qts);
}

/* An LP input signal from an LP pad, inverted, and the constants */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_lp_input_routing(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "lp-sig-in");
    qtest_writel(qts, LP_IOMUX_PAD(7), LP_PAD_LP_GPIO | LP_PAD_FUN_IE);
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_SPI_CK),
                 LP_IN_SIG_IN_SEL | LP_IN_SEL(7));
    pad_in(qts, 7, 1);
    g_assert_true(qtest_get_irq(qts, LP_SIG_SPI_CK));
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_SPI_CK),
                 LP_IN_SIG_IN_SEL | LP_IN_INV | LP_IN_SEL(7));
    g_assert_false(qtest_get_irq(qts, LP_SIG_SPI_CK));
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_SPI_CK),
                 LP_IN_SIG_IN_SEL | LP_IN_SEL(0x30));
    g_assert_true(qtest_get_irq(qts, LP_SIG_SPI_CK));
    qtest_writel(qts, LP_GPIO_FUNC_IN(LP_SIG_SPI_CK),
                 LP_IN_SIG_IN_SEL | LP_IN_SEL(0x20));
    g_assert_false(qtest_get_irq(qts, LP_SIG_SPI_CK));

    /* The LP UART's RXD directly from GPIO15 in function 0 */
    qtest_writel(qts, LP_IOMUX_PAD(15), LP_PAD_MUX_SEL | LP_PAD_FUN_IE);
    pad_in(qts, 15, 1);
    g_assert_true(qtest_get_irq(qts, LP_SIG_UART_TXD));
    pad_in(qts, 15, 0);
    g_assert_false(qtest_get_irq(qts, LP_SIG_UART_TXD));
    qtest_quit(qts);
}

/*
 * The LP pin filter, timed by the LP IO MUX's clock: a level reaches
 * LP_GPIO_IN after two of its cycles, a one-cycle pulse does not.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_lp_pin_filter(void)
{
    QTestState *qts = start_plain();
    uint64_t cycle = clock_period_ns(qts, "/machine/clkrst/lp-iomux");

    qtest_writel(qts, LP_IOMUX_PAD(7), LP_PAD_LP_GPIO | LP_PAD_FUN_IE |
                 LP_PAD_FILTER_EN);
    qtest_writel(qts, LP_GPIO_PIN(7), LP_PIN_INT_TYPE(INT_ANY));
    pad_in(qts, 7, 1);
    qtest_clock_step(qts, cycle - 1);
    g_assert_false(lp_in(qts, 7));
    qtest_clock_step(qts, 2 * cycle);
    g_assert_true(lp_in(qts, 7));
    qtest_writel(qts, LP_GPIO_STATUS_W1TC, 1u << 7);
    pad_in(qts, 7, 0);
    qtest_clock_step(qts, cycle);
    pad_in(qts, 7, 1);
    qtest_clock_step(qts, 10 * cycle);
    g_assert_true(lp_in(qts, 7));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_STATUS), ==, 0);
    qtest_quit(qts);
}

/*
 * The LP GPIO wakeup: a high level while it holds, and an edge latched
 * until LP_GPIO_PINn_EDGE_WAKEUP_CLR.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_lp_wakeup(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "lp-wakeup");
    qtest_writel(qts, LP_IOMUX_PAD(3), LP_PAD_LP_GPIO | LP_PAD_FUN_IE);
    qtest_writel(qts, LP_GPIO_PIN(3), LP_PIN_WAKEUP |
                 LP_PIN_INT_TYPE(INT_HIGH));
    g_assert_false(qtest_get_irq(qts, 0));
    pad_in(qts, 3, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    pad_in(qts, 3, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_writel(qts, LP_GPIO_PIN(3), LP_PIN_WAKEUP |
                 LP_PIN_INT_TYPE(INT_FALLING));
    pad_in(qts, 3, 1);
    g_assert_false(qtest_get_irq(qts, 0));
    pad_in(qts, 3, 0);
    g_assert_true(qtest_get_irq(qts, 0));
    pad_in(qts, 3, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, LP_GPIO_PIN(3), LP_PIN_WAKEUP | LP_PIN_EDGE_CLR |
                 LP_PIN_INT_TYPE(INT_FALLING));
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_PIN(3)), ==,
                    LP_PIN_WAKEUP | LP_PIN_INT_TYPE(INT_FALLING));
    qtest_quit(qts);
}

/* The HP GPIO wakeup from light sleep, on a level type */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_hp_wakeup(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "wakeup");
    gpio_mode(qts, 30, 0);
    qtest_writel(qts, GPIO_PIN(30), PIN_WAKEUP_ENABLE |
                 PIN_INT_TYPE(INT_LOW));
    g_assert_true(qtest_get_irq(qts, 0));
    pad_in(qts, 30, 1);
    g_assert_false(qtest_get_irq(qts, 0));
    /* An edge type wakes nothing. */
    qtest_writel(qts, GPIO_PIN(30), PIN_WAKEUP_ENABLE |
                 PIN_INT_TYPE(INT_RISING));
    pad_in(qts, 30, 0);
    pad_in(qts, 30, 1);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/*
 * ETM: events on a pad's edges for an enabled event channel; tasks that
 * set, clear and toggle GPIO_OUT of the pads given to their channel.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_etm(void)
{
    QTestState *qts = start_plain();

    gpio_mode(qts, 9, 0);
    qtest_writel(qts, GPIO_ETM_EVENT(2), ETM_EVENT_EN | 9);
    qtest_irq_intercept_out_named(qts, GPIO_PATH, "etm-event");
    /* The lines pulse; qtest sees the last level, low, after each. */
    pad_in(qts, 9, 1);
    pad_in(qts, 9, 0);
    g_assert_false(qtest_get_irq(qts, 2 * 3 + 0));

    /* Task channel 5 drives GPIO10 and GPIO33. */
    qtest_writel(qts, GPIO_ETM_TASK(2), (5 << 9) | (1 << 8));
    qtest_writel(qts, GPIO_ETM_TASK(8), (5 << 9) | (1 << 8));
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 5 * 3 + 0, 1);
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 5 * 3 + 0, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 1u << 9);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT1), ==, 1u << 1);
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 5 * 3 + 2, 1);
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 5 * 3 + 2, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT1), ==, 0);
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 5 * 3 + 2, 1);
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 5 * 3 + 1, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    /* Another channel's tasks leave them. */
    qtest_set_irq_in(qts, GPIO_PATH, "etm-task", 4 * 3 + 0, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    qtest_quit(qts);
}

/*
 * The analog comparators' edges latch GPIO_INT_RAW and raise
 * GPIO_PAD_COMP_INTR (source 78) while enabled; GPIO_ZERO_DETn_FILTER_CNT
 * hides edges for that many IO MUX clock cycles after an interrupt.
 * Comparator n's rising edge is bit 3n and its falling edge bit 3n + 1,
 * as ESP-IDF's driver reads them (the TRM names them the other way
 * round), and either is bit 3n + 2.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_pad_comp(void)
{
    QTestState *qts = start_plain();

    qtest_writel(qts, GPIO_ZERO_DET0, 0);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 0, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0x5);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_ST), ==, 0x5);
    g_assert_cmphex(src_status(qts, 2), ==, SRC_GPIO_BIT(4));
    qtest_writel(qts, GPIO_INT_CLR, 0x5);
    g_assert_cmphex(src_status(qts, 2), ==, 0);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 0, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0x6);
    qtest_writel(qts, GPIO_INT_ENA, 0xfd);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_ST), ==, 0x4);
    qtest_writel(qts, GPIO_INT_ENA, 0);
    g_assert_cmphex(src_status(qts, 2), ==, 0);
    qtest_writel(qts, GPIO_INT_RAW, 0xff);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0);

    /* Comparator 1, filtered for 100 cycles after each interrupt */
    qtest_writel(qts, GPIO_ZERO_DET1, 100);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 1, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0x28);
    qtest_writel(qts, GPIO_INT_CLR, 0x28);
    qtest_clock_step(qts, 50 * CYCLE_NS);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 1, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0);
    qtest_clock_step(qts, 60 * CYCLE_NS);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 1, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0x28);
    qtest_clock_step(qts, 110 * CYCLE_NS);
    qtest_set_irq_in(qts, GPIO_PATH, "ana-comp", 1, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_INT_RAW), ==, 0x38);
    qtest_quit(qts);
}

/*
 * A held pad keeps its drive, pulls and input enable whatever its
 * registers do: LP_IOMUX_LP_PAD_HOLD for an LP pad, the "pad-hold" hook
 * for any.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_hold(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    gpio_mode(qts, 3, 0);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 3);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << 3);
    qtest_writel(qts, LP_IOMUX_PAD_HOLD, 1u << 3);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 3);
    qtest_writel(qts, IO_MUX_GPIO(3), MCU_SEL_GPIO);
    g_assert_true(qtest_get_irq(qts, 3));
    g_assert_true(gpio_in(qts, 3));
    qtest_writel(qts, LP_IOMUX_PAD_HOLD, 0);
    g_assert_false(qtest_get_irq(qts, 3));

    gpio_mode(qts, 40, 0);
    qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << 8);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-hold", 40, 1);
    qtest_writel(qts, GPIO_OUT1_W1TS, 1u << 8);
    g_assert_false(qtest_get_irq(qts, 40));
    qtest_set_irq_in(qts, GPIO_PATH, "pad-hold", 40, 0);
    g_assert_true(qtest_get_irq(qts, 40));
    qtest_quit(qts);
}

/*
 * In light sleep a pad with SLP_SEL takes its sleep configuration: its
 * output enable MCU_OE, its pull-up MCU_WPU, its input MCU_IE.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_light_sleep(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "pad");
    gpio_mode(qts, 20, SLP_SEL | MCU_WPU);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << 20);
    qtest_writel(qts, GPIO_OUT_W1TC, 1u << 20);
    g_assert_false(qtest_get_irq(qts, 20));
    qtest_set_irq_in(qts, GPIO_PATH, "light-sleep", 0, 1);
    /* Output disabled, pulled up */
    g_assert_true(qtest_get_irq(qts, 20));
    qtest_writel(qts, IO_MUX_GPIO(20), MCU_SEL_GPIO | FUN_IE | SLP_SEL |
                 MCU_WPU | MCU_OE | MCU_IE);
    g_assert_false(qtest_get_irq(qts, 20));
    qtest_set_irq_in(qts, GPIO_PATH, "light-sleep", 0, 0);
    g_assert_false(qtest_get_irq(qts, 20));
    qtest_quit(qts);
}

/*
 * GPIO25, the USB Serial/JTAG PHY's D+, is pulled up by the PHY out of
 * reset; the PHY's "pad-pull-up" hook releases it.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_usb_pull_up(void)
{
    QTestState *qts = start_plain();

    gpio_mode(qts, 25, 0);
    g_assert_true(gpio_in(qts, 25));
    qtest_set_irq_in(qts, GPIO_PATH, "pad-pull-up", 25, 0);
    g_assert_false(gpio_in(qts, 25));
    qtest_quit(qts);
}

/*
 * The IO MUX functions with lines of their own: GPIO2's MTCK, input
 * enabled from reset, to its "iomux-func-in" line; GPIO5's MTDO driven by
 * its "iomux-func-out" and "-oe" lines.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_iomux_direct_lines(void)
{
    QTestState *qts = start_plain();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, "iomux-func-in");
    /* Pulled up from reset: low from outside, then high again */
    pad_in(qts, 2, 0);
    g_assert_false(qtest_get_irq(qts, 2 * 5 + 0));
    pad_in(qts, 2, 1);
    g_assert_true(qtest_get_irq(qts, 2 * 5 + 0));
    qtest_writel(qts, IO_MUX_GPIO(2), MCU_SEL_GPIO | FUN_IE);
    g_assert_false(qtest_get_irq(qts, 2 * 5 + 0));
    pad_release(qts, 2);

    qtest_writel(qts, IO_MUX_GPIO(5), FUN_IE);
    qtest_set_irq_in(qts, GPIO_PATH, "iomux-func-out", 5 * 5 + 0, 1);
    g_assert_false(gpio_in(qts, 5));
    qtest_set_irq_in(qts, GPIO_PATH, "iomux-func-oe", 5 * 5 + 0, 1);
    g_assert_true(gpio_in(qts, 5));
    qtest_quit(qts);
}

/*
 * The registers behind their bus clocks and resets: with IOMUX_APB_CLK
 * gated a write is lost; with the IO MUX held in reset the HP registers
 * are at their reset values and take no writes; the LP IO MUX's reset
 * leaves the HP registers alone.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_clock_and_reset(void)
{
    QTestState *qts = start_plain();
    uint32_t ctrl3 = qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL3);

    g_assert_true(ctrl3 & IOMUX_APB_CLK_EN);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL3, ctrl3 & ~IOMUX_APB_CLK_EN);
    qtest_writel(qts, GPIO_OUT, 0x1234);
    qtest_writel(qts, IO_MUX_GPIO(20), MCU_SEL_GPIO);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL3, ctrl3);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IO_MUX_GPIO(20)), ==, FUN_DRV(2));

    qtest_writel(qts, GPIO_OUT, 0x1234);
    qtest_writel(qts, LP_GPIO_OUT, 0x12);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, RST_EN_IOMUX);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    qtest_writel(qts, GPIO_OUT, 0x5678);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0x12);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN1, 0);
    qtest_writel(qts, GPIO_OUT, 0x5678);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0x5678);

    qtest_writel(qts, LPPERI_RESET_EN, RST_EN_LP_IOMUX);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0);
    qtest_writel(qts, LP_GPIO_OUT, 0x34);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_OUT), ==, 0x5678);
    qtest_writel(qts, LPPERI_RESET_EN, 0);
    qtest_writel(qts, LP_GPIO_OUT, 0x34);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0x34);

    /* The LP blocks behind their clock */
    qtest_writel(qts, LPPERI_CLK_EN,
                 qtest_readl(qts, LPPERI_CLK_EN) & ~LP_IOMUX_CK_EN);
    qtest_writel(qts, LP_GPIO_OUT, 0x56);
    qtest_writel(qts, LPPERI_CLK_EN,
                 qtest_readl(qts, LPPERI_CLK_EN) | LP_IOMUX_CK_EN);
    g_assert_cmphex(qtest_readl(qts, LP_GPIO_OUT), ==, 0x34);
    qtest_quit(qts);
}

/*
 * The filters and modulators run at the IO MUX operating clock: divided by
 * 2 (HP_SYS_CLKRST_IOMUX_CLK_DIV_NUM 1) the pin filter takes four crystal
 * cycles to pass a level.
 */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_iomux_clock(void)
{
    QTestState *qts = start_plain();
    uint32_t ctrl26 = qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL26);

    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL26,
                 (ctrl26 & ~(0xffu << 10)) | (1u << 10));
    gpio_mode(qts, 20, FILTER_EN);
    pad_in(qts, 20, 1);
    qtest_clock_step(qts, 3 * CYCLE_NS);
    g_assert_false(gpio_in(qts, 20));
    qtest_clock_step(qts, 2 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));

    /* Gated (IOMUX_CLK_EN, bit 9), the filter holds its output. */
    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL26,
                 qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL26) & ~(1u << 9));
    pad_in(qts, 20, 0);
    qtest_clock_step(qts, 100 * CYCLE_NS);
    g_assert_true(gpio_in(qts, 20));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/gpio/reset_values", test_reset_values);
    qtest_add_func("esp32p4/gpio/register_access", test_register_access);
    qtest_add_func("esp32p4/gpio/strap", test_strap);
    qtest_add_func("esp32p4/gpio/pad_levels", test_pad_levels);
    qtest_add_func("esp32p4/gpio/pad_properties", test_pad_properties);
    qtest_add_func("esp32p4/gpio/output_routing", test_output_routing);
    qtest_add_func("esp32p4/gpio/input_routing", test_input_routing);
    qtest_add_func("esp32p4/gpio/loopback", test_loopback);
    qtest_add_func("esp32p4/gpio/interrupts", test_interrupts);
    qtest_add_func("esp32p4/gpio/pin_filter", test_pin_filter);
    qtest_add_func("esp32p4/gpio/glitch_filter", test_glitch_filter);
    qtest_add_func("esp32p4/gpio/sigma_delta", test_sigma_delta);
    qtest_add_func("esp32p4/gpio/lp_io", test_lp_io);
    qtest_add_func("esp32p4/gpio/lp_input_routing", test_lp_input_routing);
    qtest_add_func("esp32p4/gpio/lp_pin_filter", test_lp_pin_filter);
    qtest_add_func("esp32p4/gpio/lp_wakeup", test_lp_wakeup);
    qtest_add_func("esp32p4/gpio/hp_wakeup", test_hp_wakeup);
    qtest_add_func("esp32p4/gpio/etm", test_etm);
    qtest_add_func("esp32p4/gpio/pad_comp", test_pad_comp);
    qtest_add_func("esp32p4/gpio/hold", test_hold);
    qtest_add_func("esp32p4/gpio/light_sleep", test_light_sleep);
    qtest_add_func("esp32p4/gpio/usb_pull_up", test_usb_pull_up);
    qtest_add_func("esp32p4/gpio/iomux_direct_lines",
                   test_iomux_direct_lines);
    qtest_add_func("esp32p4/gpio/clock_and_reset", test_clock_and_reset);
    qtest_add_func("esp32p4/gpio/iomux_clock", test_iomux_clock);
    return g_test_run();
}
