/*
 * QTest testcase for the ESP32 peripherals' line-level signals through the
 * GPIO matrix: UART, I2C, LEDC and TWAI
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qobject/qdict.h"
#include "libqtest.h"
#include "qemu/sockets.h"

#define DPORT_PERIP_CLK_EN  0x3ff000c0
#define DPORT_PERIP_RST_EN  0x3ff000c4
#define PERIP_UART0         (1u << 2)
#define PERIP_UART1         (1u << 5)
#define PERIP_I2C0          (1u << 7)
#define PERIP_LEDC          (1u << 11)
#define PERIP_TWAI          (1u << 19)
#define PERIP_UART2         (1u << 23)
#define PERIP_UART_MEM      (1u << 24)

#define GPIO                0x3ff44000
#define GPIO_IN             (GPIO + 0x3c)
#define GPIO_PIN(n)         (GPIO + 0x88 + 4 * (n))
#define GPIO_FUNC_IN(s)     (GPIO + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x530 + 4 * (n))
#define PIN_PAD_DRIVER      (1u << 2)
#define IN_SIG_IN_SEL       (1u << 7)
#define IN_CONST_HIGH       0x38

#define IO_MUX              0x3ff49000
#define IO_MUX_GPIO1        (IO_MUX + 0x88)
#define IO_MUX_GPIO18       (IO_MUX + 0x70)
#define IO_MUX_GPIO19       (IO_MUX + 0x74)
#define IO_MUX_GPIO21       (IO_MUX + 0x7c)
#define IO_MUX_GPIO22       (IO_MUX + 0x80)
#define FUN_WPU             (1u << 8)
#define FUN_IE              (1u << 9)
#define MCU_SEL(f)          ((f) << 12)
#define PIN_FUNC_GPIO       2

#define SIG_U0RXD           14
#define SIG_U0TXD           14
#define SIG_U1RXD           17
#define SIG_U1TXD           17
#define SIG_I2C0_SCL        29
#define SIG_I2C0_SDA        30
#define SIG_LEDC_HS0        71
#define SIG_LEDC_LS0        79
#define SIG_TWAI_RX         94
#define SIG_TWAI_TX         123
#define SIG_TWAI_BUS_OFF    124
#define SIG_TWAI_CLKOUT     125
#define SIG_U2RXD           198

#define UART0               0x3ff40000
#define UART1               0x3ff50000
#define UART2               0x3ff6e000
#define UART_FIFO           0x00
#define UART_INT_RAW        0x04
#define UART_INT_CLR        0x10
#define UART_CLKDIV         0x14
#define UART_STATUS         0x1c
#define UART_CONF0          0x20
#define UART_CONF0_RESET    0x0800001c
#define CONF0_PARITY_ODD    (1u << 0)
#define CONF0_PARITY_EN     (1u << 1)
#define INT_PARITY_ERR      (1u << 2)
#define RXFIFO_CNT(st)      ((st) & 0xff)
/* 115200 baud from the 40 MHz APB_CLK: 347 + 3/16 */
#define CLKDIV_115200       (347 | (3u << 20))
/* Its bit time in ns: (347 + 3/16) * 25 */
#define BIT_NS              8680

#define I2C0                0x3ff53000
#define I2C_CTR             0x04
#define I2C_STATUS          0x08
#define I2C_FIFO_DATA       0x1c
#define I2C_INT_RAW         0x20
#define I2C_INT_CLR         0x24
#define I2C_CMD(n)          (0x58 + 4 * (n))
#define CTR_MS_MODE         (1u << 4)
#define CTR_TRANS_START     (1u << 5)
#define I2C_INT_ACK_ERR     (1u << 10)
#define I2C_INT_TRANS_COMPLETE (1u << 7)
#define CMD_RSTART          (0u << 11)
#define CMD_WRITE           (1u << 11)
#define CMD_READ            (2u << 11)
#define CMD_STOP            (3u << 11)
#define CMD_ACK_CHECK       (1u << 8)
#define CMD_ACK_VAL         (1u << 10)
#define TMP105_ADDR         0x48

#define LEDC                0x3ff59000
#define LEDC_CH(n)          (LEDC + 0x14 * (n))
#define LEDC_CONF0          0x00
#define LEDC_HPOINT         0x04
#define LEDC_DUTY           0x08
#define LEDC_CONF1          0x0c
#define LEDC_DUTY_R         0x10
#define LEDC_TIMER(i)       (LEDC + 0x140 + 8 * (i))
#define LEDC_VALUE          0x04
#define LEDC_INT_RAW        (LEDC + 0x180)
#define LEDC_INT_CLR        (LEDC + 0x18c)
#define LEDC_CONF           (LEDC + 0x190)
#define CH_SIG_OUT_EN       (1u << 2)
#define CH_IDLE_LV          (1u << 3)
#define CH_PARA_UP          (1u << 4)
#define CONF1_SCALE(n)      (n)
#define CONF1_CYCLE(n)      ((n) << 10)
#define CONF1_NUM(n)        ((n) << 20)
#define CONF1_INC           (1u << 30)
#define CONF1_START         (1u << 31)
#define TIMER_RES(n)        (n)
#define TIMER_DIV(n)        ((n) << 5)
#define TIMER_RST           (1u << 24)
#define TIMER_TICK_APB      (1u << 25)
#define TIMER_PARA_UP       (1u << 26)
#define LEDC_INT_HS0_OVF    (1u << 0)
#define LEDC_INT_HSCH0_END  (1u << 8)

#define TWAI                0x3ff6b000
#define TWAI_CDR            (TWAI + 0x7c)
#define CDR_CLOCK_OFF       0x08

#define US                  1000
#define MS                  (1000 * US)

static void enable(QTestState *qts, uint32_t bits)
{
    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | bits);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) & ~bits);
}

static bool pad(QTestState *qts, unsigned n)
{
    return (qtest_readl(qts, GPIO_IN) >> n) & 1;
}

/* Pad n in its GPIO matrix function, input enabled, driven by sig */
static void route_out(QTestState *qts, uint32_t iomux, unsigned n,
                      unsigned sig)
{
    qtest_writel(qts, iomux, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE | FUN_WPU);
    qtest_writel(qts, GPIO_FUNC_OUT(n), sig);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * UART1's TXD reaches its RXD only through a pad the matrix routes both
 * to, and bytes take their frame time at the configured baud rate.
 */
static void test_uart_loopback(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    static const uint8_t data[] = { 0x55, 0xa3, 0x00 };

    enable(qts, PERIP_UART1 | PERIP_UART_MEM);
    qtest_writel(qts, UART1 + UART_CLKDIV, CLKDIV_115200);
    route_out(qts, IO_MUX_GPIO18, 18, SIG_U1TXD);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_U1RXD), IN_SIG_IN_SEL | 18);
    /* The idle line is high */
    g_assert_true(pad(qts, 18));

    for (int i = 0; i < sizeof(data); i++) {
        qtest_writel(qts, UART1 + UART_FIFO, data[i]);
    }
    /* The start bit is on the pad at once */
    g_assert_false(pad(qts, 18));
    /* A byte arrives at its stop bit's middle, 9.5 bits after its start */
    qtest_clock_step(qts, 95 * BIT_NS / 10 - 200);
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)), ==, 0);
    qtest_clock_step(qts, 400);
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)), ==, 1);
    /* Frames follow back to back, 10 bits each */
    qtest_clock_step(qts, 20 * BIT_NS - 400);
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)), ==, 2);
    qtest_clock_step(qts, 400);
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)), ==, 3);
    for (int i = 0; i < sizeof(data); i++) {
        g_assert_cmphex(qtest_readl(qts, UART1 + UART_FIFO), ==, data[i]);
    }

    /* RXD taken from another pad: nothing arrives */
    qtest_writel(qts, IO_MUX_GPIO19, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPU);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_U1RXD), IN_SIG_IN_SEL | 19);
    qtest_writel(qts, UART1 + UART_FIFO, 'x');
    qtest_clock_step(qts, 30 * BIT_NS);
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)), ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * UART1 talks to UART2 through a pad; a parity mismatch between them is
 * a parity error at the receiver.
 */
static void test_uart_parity(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    enable(qts, PERIP_UART1 | PERIP_UART2 | PERIP_UART_MEM);
    qtest_writel(qts, UART1 + UART_CLKDIV, CLKDIV_115200);
    qtest_writel(qts, UART2 + UART_CLKDIV, CLKDIV_115200);
    qtest_writel(qts, UART1 + UART_CONF0, UART_CONF0_RESET | CONF0_PARITY_EN);
    qtest_writel(qts, UART2 + UART_CONF0, UART_CONF0_RESET | CONF0_PARITY_EN);
    route_out(qts, IO_MUX_GPIO18, 18, SIG_U1TXD);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_U2RXD), IN_SIG_IN_SEL | 18);

    qtest_writel(qts, UART1 + UART_FIFO, 0x31);
    qtest_clock_step(qts, 12 * BIT_NS);
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART2 + UART_STATUS)), ==, 1);
    g_assert_cmphex(qtest_readl(qts, UART2 + UART_FIFO), ==, 0x31);
    g_assert_false(qtest_readl(qts, UART2 + UART_INT_RAW) & INT_PARITY_ERR);

    qtest_writel(qts, UART2 + UART_CONF0, UART_CONF0_RESET | CONF0_PARITY_EN |
                 CONF0_PARITY_ODD);
    qtest_writel(qts, UART1 + UART_FIFO, 0x31);
    qtest_clock_step(qts, 12 * BIT_NS);
    g_assert_true(qtest_readl(qts, UART2 + UART_INT_RAW) & INT_PARITY_ERR);
    qtest_writel(qts, UART2 + UART_INT_CLR, INT_PARITY_ERR);
    g_assert_false(qtest_readl(qts, UART2 + UART_INT_RAW) & INT_PARITY_ERR);
    qtest_quit(qts);
}

/* Run the machine until n bytes have come out of the serial socket. */
static void serial_read(QTestState *qts, int sock, uint8_t *buf, size_t n)
{
    size_t got = 0;

    for (int i = 0; i < 2000 && got < n; i++) {
        ssize_t r;

        qtest_clock_step(qts, MS);
        r = recv(sock, buf + got, n - got, MSG_DONTWAIT);
        if (r > 0) {
            got += r;
        }
    }
    g_assert_cmpuint(got, ==, n);
}

static void serial_quiet(QTestState *qts, int sock)
{
    uint8_t b;

    for (int i = 0; i < 20; i++) {
        qtest_clock_step(qts, MS);
        g_usleep(1000);
    }
    g_assert_cmpint(recv(sock, &b, 1, MSG_DONTWAIT), <=, 0);
}

/* Run the machine until UART0's RX FIFO holds n bytes. */
static void uart0_wait_rx(QTestState *qts, unsigned n)
{
    for (int i = 0; i < 2000 &&
         RXFIFO_CNT(qtest_readl(qts, UART0 + UART_STATUS)) < n; i++) {
        g_usleep(1000);
        qtest_clock_step(qts, 100 * US);
    }
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART0 + UART_STATUS)), ==, n);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * The -serial chardev is a terminal on GPIO1 and GPIO3: UART0's bytes
 * reach it while U0TXD is on GPIO1, by the IO_MUX or the matrix, and not
 * otherwise; its bytes reach UART0 while U0RXD comes from GPIO3.
 */
static void test_uart_terminal(void)
{
    int sock;
    QTestState *qts = qtest_init_with_serial("-M esp32 -nic none", &sock);
    uint8_t got;

    qtest_writel(qts, UART0 + UART_FIFO, 'A');
    serial_read(qts, sock, &got, 1);
    g_assert_cmphex(got, ==, 'A');

    /* GPIO1 as a plain GPIO: UART0 transmits to no pad */
    qtest_writel(qts, IO_MUX_GPIO1, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPU);
    qtest_writel(qts, UART0 + UART_FIFO, 'B');
    serial_quiet(qts, sock);

    /* U0TXD back on GPIO1 through the matrix */
    qtest_writel(qts, GPIO_FUNC_OUT(1), SIG_U0TXD);
    qtest_writel(qts, UART0 + UART_FIFO, 'C');
    serial_read(qts, sock, &got, 1);
    g_assert_cmphex(got, ==, 'C');

    /* The terminal's bytes arrive on U0RXD from GPIO3 */
    g_assert_cmpint(send(sock, "x", 1, 0), ==, 1);
    uart0_wait_rx(qts, 1);
    g_assert_cmphex(qtest_readl(qts, UART0 + UART_FIFO), ==, 'x');

    /* U0RXD from a constant instead: the terminal is not heard */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_U0RXD), IN_SIG_IN_SEL | IN_CONST_HIGH);
    g_assert_cmpint(send(sock, "y", 1, 0), ==, 1);
    for (int i = 0; i < 20; i++) {
        g_usleep(1000);
        qtest_clock_step(qts, MS);
    }
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART0 + UART_STATUS)), ==, 0);
    qtest_quit(qts);
}

/* Start an I2C0 transaction and return INT_RAW after it. */
static uint32_t i2c_run(QTestState *qts, const uint32_t *cmds, unsigned n,
                        const uint8_t *tx, unsigned n_tx)
{
    qtest_writel(qts, I2C0 + I2C_INT_CLR, 0x1fff);
    for (unsigned i = 0; i < n; i++) {
        qtest_writel(qts, I2C0 + I2C_CMD(i), cmds[i]);
    }
    for (unsigned i = 0; i < n_tx; i++) {
        qtest_writel(qts, I2C0 + I2C_FIFO_DATA, tx[i]);
    }
    qtest_writel(qts, I2C0 + I2C_CTR, CTR_MS_MODE | CTR_TRANS_START);
    return qtest_readl(qts, I2C0 + I2C_INT_RAW);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * The board's TMP105 on I2C0 answers only while SCL and SDA are routed
 * through pads, open drain with pull-ups, in and out.
 */
static void test_i2c_routing(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    static const uint32_t read_temp[] = {
        CMD_RSTART,
        CMD_WRITE | CMD_ACK_CHECK | 2,
        CMD_RSTART,
        CMD_WRITE | CMD_ACK_CHECK | 1,
        CMD_READ | 1,
        CMD_READ | CMD_ACK_VAL | 1,
        CMD_STOP,
    };
    /* Read T_HIGH, register 3: 80 degrees C out of reset */
    static const uint8_t tx[] = { TMP105_ADDR << 1, 3, (TMP105_ADDR << 1) | 1 };
    static const uint8_t tx_bad[] = { 0x49 << 1, 3, (0x49 << 1) | 1 };
    uint32_t raw;

    enable(qts, PERIP_I2C0);

    /* Unrouted, the address byte is not acknowledged */
    raw = i2c_run(qts, read_temp, ARRAY_SIZE(read_temp), tx, sizeof(tx));
    g_assert_true(raw & I2C_INT_ACK_ERR);
    g_assert_false(raw & I2C_INT_TRANS_COMPLETE);

    qtest_writel(qts, I2C0 + 0x18, (1u << 12) | (1u << 13));
    qtest_writel(qts, I2C0 + 0x18, 0);
    route_out(qts, IO_MUX_GPIO21, 21, SIG_I2C0_SDA);
    route_out(qts, IO_MUX_GPIO22, 22, SIG_I2C0_SCL);
    qtest_writel(qts, GPIO_PIN(21), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_PIN(22), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SDA), IN_SIG_IN_SEL | 21);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SCL), IN_SIG_IN_SEL | 22);
    /* The bus idles high */
    g_assert_true(pad(qts, 21));
    g_assert_true(pad(qts, 22));

    raw = i2c_run(qts, read_temp, ARRAY_SIZE(read_temp), tx, sizeof(tx));
    g_assert_false(raw & I2C_INT_ACK_ERR);
    g_assert_true(raw & I2C_INT_TRANS_COMPLETE);
    g_assert_cmpuint((qtest_readl(qts, I2C0 + I2C_STATUS) >> 8) & 0x3f, ==, 2);
    g_assert_cmphex(qtest_readl(qts, I2C0 + I2C_FIFO_DATA), ==, 0x50);
    g_assert_cmphex(qtest_readl(qts, I2C0 + I2C_FIFO_DATA), ==, 0x00);
    g_assert_true(pad(qts, 21));
    g_assert_true(pad(qts, 22));

    /* No device at 0x49 */
    raw = i2c_run(qts, read_temp, ARRAY_SIZE(read_temp), tx_bad,
                  sizeof(tx_bad));
    g_assert_true(raw & I2C_INT_ACK_ERR);

    /* SDA's input from another pad: the controller is off the bus again */
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2C0_SDA),
                 IN_SIG_IN_SEL | IN_CONST_HIGH);
    raw = i2c_run(qts, read_temp, ARRAY_SIZE(read_temp), tx, sizeof(tx));
    g_assert_true(raw & I2C_INT_ACK_ERR);
    qtest_quit(qts);
}

/*
 * HSTIMER0 from APB_CLK (40 MHz) divided by 40: a 1 us tick, 1024 ticks a
 * cycle with a 10-bit resolution.
 */
static void ledc_hs_timer0(QTestState *qts)
{
    uint32_t conf = TIMER_RES(10) | TIMER_DIV(40 << 8) | TIMER_TICK_APB;

    qtest_writel(qts, LEDC_TIMER(0), conf | TIMER_RST);
    qtest_writel(qts, LEDC_TIMER(0), conf);
}

/*
 * Sample a pad once per timer tick over a whole cycle, checking it is
 * high exactly while the count is in [hp, lp); returns the high samples.
 */
static unsigned ledc_sample_cycle(QTestState *qts, unsigned n, unsigned timer,
                                  unsigned hp, unsigned lp)
{
    unsigned high = 0;

    for (unsigned i = 0; i < 1024; i++) {
        uint32_t v = qtest_readl(qts, LEDC_TIMER(timer) + LEDC_VALUE);
        bool level = pad(qts, n);

        g_assert_cmpint(level, ==, v >= hp && v < lp);
        high += level;
        qtest_clock_step(qts, US);
    }
    return high;
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * A high-speed channel's PWM output reaches a pad with its duty and
 * period: high from HPOINT to HPOINT + DUTY of every cycle, taken in at a
 * cycle start after DUTY_START. A fade steps the duty and ends with
 * DUTY_CHNG_END.
 */
static void test_ledc_hs(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    enable(qts, PERIP_LEDC);
    route_out(qts, IO_MUX_GPIO18, 18, SIG_LEDC_HS0);
    ledc_hs_timer0(qts);
    qtest_writel(qts, LEDC_CH(0) + LEDC_HPOINT, 100);
    qtest_writel(qts, LEDC_CH(0) + LEDC_DUTY, 256 << 4);
    qtest_writel(qts, LEDC_CH(0) + LEDC_CONF0, CH_SIG_OUT_EN);
    qtest_writel(qts, LEDC_CH(0) + LEDC_CONF1, CONF1_START | CONF1_INC |
                 CONF1_NUM(1) | CONF1_CYCLE(1));
    g_assert_true(qtest_readl(qts, LEDC_CH(0) + LEDC_CONF1) & CONF1_START);
    /* Off until the cycle starts */
    g_assert_false(pad(qts, 18));
    qtest_clock_step(qts, 1100 * US);
    g_assert_false(qtest_readl(qts, LEDC_CH(0) + LEDC_CONF1) & CONF1_START);
    g_assert_cmphex(qtest_readl(qts, LEDC_CH(0) + LEDC_DUTY_R), ==, 256 << 4);
    g_assert_true(qtest_readl(qts, LEDC_INT_RAW) & LEDC_INT_HS0_OVF);

    g_assert_cmpuint(ledc_sample_cycle(qts, 18, 0, 100, 356), ==, 256);
    /* The one-step fade has ended */
    g_assert_true(qtest_readl(qts, LEDC_INT_RAW) & LEDC_INT_HSCH0_END);

    /* Fade up by 8 a cycle, 4 times */
    qtest_writel(qts, LEDC_INT_CLR, LEDC_INT_HSCH0_END);
    qtest_writel(qts, LEDC_CH(0) + LEDC_CONF1, CONF1_START | CONF1_INC |
                 CONF1_NUM(4) | CONF1_CYCLE(1) | CONF1_SCALE(8));
    qtest_clock_step(qts, 6 * MS);
    g_assert_true(qtest_readl(qts, LEDC_INT_RAW) & LEDC_INT_HSCH0_END);
    g_assert_cmphex(qtest_readl(qts, LEDC_CH(0) + LEDC_DUTY_R), ==,
                    (256 + 32) << 4);
    g_assert_cmpuint(ledc_sample_cycle(qts, 18, 0, 100, 388), ==, 288);

    /* Output disabled: the idle level */
    qtest_writel(qts, LEDC_CH(0) + LEDC_CONF0, CH_IDLE_LV);
    g_assert_true(pad(qts, 18));
    qtest_writel(qts, LEDC_CH(0) + LEDC_CONF0, 0);
    g_assert_false(pad(qts, 18));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * A low-speed channel takes its settings in only with PARA_UP; full duty
 * holds the output high; a fast PWM still samples at its duty.
 */
static void test_ledc_ls(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    uint32_t conf = TIMER_RES(10) | TIMER_DIV(40 << 8) | TIMER_TICK_APB;
    unsigned high = 0;

    enable(qts, PERIP_LEDC);
    route_out(qts, IO_MUX_GPIO19, 19, SIG_LEDC_LS0);
    /* LSTIMER0 from SLOW_CLK, APB_CLK */
    qtest_writel(qts, LEDC_CONF, 1);
    qtest_writel(qts, LEDC_TIMER(4), conf | TIMER_RST);
    qtest_writel(qts, LEDC_TIMER(4), conf);
    qtest_writel(qts, LEDC_CH(8) + LEDC_HPOINT, 0);
    qtest_writel(qts, LEDC_CH(8) + LEDC_DUTY, 512 << 4);
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF0, CH_SIG_OUT_EN);
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF1, CONF1_START | CONF1_INC |
                 CONF1_NUM(1) | CONF1_CYCLE(1));
    qtest_clock_step(qts, 3 * MS);
    /* Without PARA_UP nothing is taken in */
    g_assert_true(qtest_readl(qts, LEDC_CH(8) + LEDC_CONF1) & CONF1_START);
    g_assert_cmphex(qtest_readl(qts, LEDC_CH(8) + LEDC_DUTY_R), ==, 0);
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF0, CH_SIG_OUT_EN | CH_PARA_UP);
    qtest_clock_step(qts, 1100 * US);
    g_assert_false(qtest_readl(qts, LEDC_CH(8) + LEDC_CONF1) & CONF1_START);
    g_assert_cmpuint(ledc_sample_cycle(qts, 19, 4, 0, 512), ==, 512);

    /* Full duty: never reaches its low point */
    qtest_writel(qts, LEDC_CH(8) + LEDC_DUTY, 1024 << 4);
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF1, CONF1_START | CONF1_INC |
                 CONF1_NUM(1) | CONF1_CYCLE(1));
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF0, CH_SIG_OUT_EN | CH_PARA_UP);
    qtest_clock_step(qts, 2100 * US);
    for (int i = 0; i < 64; i++) {
        g_assert_true(pad(qts, 19));
        qtest_clock_step(qts, 37 * US);
    }

    /*
     * A 2.5 MHz PWM, 16 ticks of 25 ns, a quarter high: far faster than
     * its edges are timed, but a pad sampled every tick reads its duty.
     */
    conf = TIMER_RES(4) | TIMER_DIV(1 << 8) | TIMER_TICK_APB;
    qtest_writel(qts, LEDC_TIMER(4), conf | TIMER_RST);
    qtest_writel(qts, LEDC_TIMER(4), conf);
    qtest_writel(qts, LEDC_CH(8) + LEDC_DUTY, 4 << 4);
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF1, CONF1_START | CONF1_INC |
                 CONF1_NUM(1) | CONF1_CYCLE(1));
    qtest_writel(qts, LEDC_CH(8) + LEDC_CONF0, CH_SIG_OUT_EN | CH_PARA_UP);
    qtest_clock_step(qts, 2 * US);
    for (int i = 0; i < 160; i++) {
        high += pad(qts, 19);
        qtest_clock_step(qts, 25);
    }
    g_assert_cmpuint(high, ==, 40);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * TWAI's TX idles recessive and BUS_OFF low on their pads; CLKOUT is
 * APB_CLK / 2 until CLOCK_OFF. The controller joins the CAN bus backend
 * only while TX goes to a pad and RX comes from one.
 */
static void test_twai_lines(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none "
                                 "-object can-bus,id=canbus0 "
                                 "-global driver=esp32.twai,property=canbus,"
                                 "value=canbus0");
    unsigned high = 0;
    QDict *r;

    enable(qts, PERIP_TWAI);
    route_out(qts, IO_MUX_GPIO18, 18, SIG_TWAI_TX);
    route_out(qts, IO_MUX_GPIO19, 19, SIG_TWAI_BUS_OFF);
    g_assert_true(pad(qts, 18));
    g_assert_false(pad(qts, 19));

    r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': { "
                  "'path': '/machine/soc/twai', 'property': "
                  "'bus-connected' } }");
    g_assert_false(qdict_get_bool(r, "return"));
    qobject_unref(r);
    qtest_writel(qts, IO_MUX_GPIO21, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE |
                 FUN_WPU);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_TWAI_RX), IN_SIG_IN_SEL | 21);
    r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': { "
                  "'path': '/machine/soc/twai', 'property': "
                  "'bus-connected' } }");
    g_assert_true(qdict_get_bool(r, "return"));
    qobject_unref(r);
    qtest_writel(qts, GPIO_FUNC_OUT(18), 0x100);
    r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': { "
                  "'path': '/machine/soc/twai', 'property': "
                  "'bus-connected' } }");
    g_assert_false(qdict_get_bool(r, "return"));
    qobject_unref(r);

    /* CLKOUT at 20 MHz, sampled every half period */
    route_out(qts, IO_MUX_GPIO18, 18, SIG_TWAI_CLKOUT);
    for (int i = 0; i < 16; i++) {
        high += pad(qts, 18);
        qtest_clock_step(qts, 25);
    }
    g_assert_cmpuint(high, ==, 8);
    qtest_writel(qts, TWAI_CDR, CDR_CLOCK_OFF);
    for (int i = 0; i < 16; i++) {
        g_assert_false(pad(qts, 18));
        qtest_clock_step(qts, 25);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/periph-lines/uart-loopback", test_uart_loopback);
    qtest_add_func("/esp32/periph-lines/uart-parity", test_uart_parity);
    qtest_add_func("/esp32/periph-lines/uart-terminal", test_uart_terminal);
    qtest_add_func("/esp32/periph-lines/i2c-routing", test_i2c_routing);
    qtest_add_func("/esp32/periph-lines/ledc-hs", test_ledc_hs);
    qtest_add_func("/esp32/periph-lines/ledc-ls", test_ledc_ls);
    qtest_add_func("/esp32/periph-lines/twai-lines", test_twai_lines);
    return g_test_run();
}
