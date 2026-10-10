/*
 * QTest testcase for the RP2350 UARTs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The UARTs work at line level through the GPIO muxing, and the board's
 * serial port is an adapter on GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX).
 * Under qtest the virtual clock only moves when the test steps it, so
 * bit timing is exact.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define UART0       0x40070000
#define UART1       0x40078000
#define XOR         0x1000
#define SET         0x2000
#define CLR         0x3000

#define UARTDR      0x000
#define UARTRSR     0x004
#define UARTFR      0x018
#define UARTIBRD    0x024
#define UARTFBRD    0x028
#define UARTLCR_H   0x02c
#define UARTCR      0x030
#define UARTIFLS    0x034
#define UARTIMSC    0x038
#define UARTRIS     0x03c
#define UARTMIS     0x040
#define UARTICR     0x044

#define FR_CTS      (1u << 0)
#define FR_BUSY     (1u << 3)
#define FR_RXFE     (1u << 4)
#define FR_TXFF     (1u << 5)
#define FR_TXFE     (1u << 7)
#define LCR_H_BRK   (1u << 0)
#define LCR_H_PEN   (1u << 1)
#define LCR_H_EPS   (1u << 2)
#define LCR_H_FEN   (1u << 4)
#define LCR_H_WLEN8 (3u << 5)
#define CR_UARTEN   (1u << 0)
#define CR_LBE      (1u << 7)
#define CR_TXE      (1u << 8)
#define CR_RXE      (1u << 9)
#define CR_RTSEN    (1u << 14)
#define CR_CTSEN    (1u << 15)
#define INT_CTS     (1u << 1)
#define INT_RX      (1u << 4)
#define INT_TX      (1u << 5)
#define INT_RT      (1u << 6)
#define INT_BE      (1u << 9)
#define DR_PE       (1u << 9)
#define DR_BE       (1u << 10)
#define DR_OE       (1u << 11)

#define UART0_IRQ   33

#define GPIO_PATH   "/machine/soc/gpio"
#define IO_BANK0    0x40028000
#define PADS_BANK0  0x40038000
#define CTRL(p)     (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)      (PADS_BANK0 + 4 + 4 * (p))
#define PAD_PUE     (1u << 3)
#define PAD_IE      (1u << 6)
#define FUNC_UART   2
#define SIO_GPIO_IN 0xd0000004

/*
 * 115200 baud from clk_peri at 150 MHz: IBRD 81, FBRD 24, a divisor of
 * 5208/64, so one bit is 16 * 5208 / 64 clk_peri cycles: 8680 ns.
 */
#define BIT_NS      8680
#define FRAME_NS    (10 * BIT_NS)

static char *rom_path;

/* The machine, with every subsystem out of reset. */
static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

static QTestState *start_with_serial(int *sock)
{
    g_autofree char *args = g_strdup_printf("-M rp2350 -bios %s", rom_path);
    QTestState *qts = qtest_init_with_serial(args, sock);

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

/* uart_init(): divisors, 8N1 with FIFOs, then enable via the SET alias. */
static void uart_init(QTestState *qts, uint32_t uart)
{
    qtest_writel(qts, uart + UARTIBRD, 81);
    qtest_writel(qts, uart + UARTFBRD, 24);
    qtest_writel(qts, uart + UARTLCR_H, LCR_H_WLEN8 | LCR_H_FEN);
    qtest_writel(qts, uart + UARTCR + SET, CR_UARTEN | CR_TXE | CR_RXE);
}

/* gpio_set_function(): an enabled, unisolated pad and the function. */
static void pin_function(QTestState *qts, int p, uint32_t func)
{
    qtest_writel(qts, PAD(p), PAD_IE);
    qtest_writel(qts, CTRL(p), func);
}

static bool pin_level(QTestState *qts, int p)
{
    return (qtest_readl(qts, SIO_GPIO_IN) >> p) & 1;
}

/* Run the clock and the main loop until the receive FIFO has data. */
static bool wait_rx(QTestState *qts, uint32_t uart)
{
    int i;

    for (i = 0; i < 1000; i++) {
        if (!(qtest_readl(qts, uart + UARTFR) & FR_RXFE)) {
            return true;
        }
        qtest_clock_step(qts, FRAME_NS);
        g_usleep(1000);
    }
    return false;
}

static ssize_t sock_poll(int sock, char *buf, size_t len)
{
    return recv(sock, buf, len, MSG_DONTWAIT);
}

/* [spec:nuos:req:emu.uart/test] */
static void test_tx_rx(void)
{
    int sock;
    char buf[8];
    QTestState *qts = start_with_serial(&sock);
    const char *msg = "nuos";
    int i;

    uart_init(qts, UART0);
    pin_function(qts, 0, FUNC_UART);
    pin_function(qts, 1, FUNC_UART);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTCR) &
                    (CR_UARTEN | CR_TXE | CR_RXE), ==,
                    CR_UARTEN | CR_TXE | CR_RXE);

    for (i = 0; msg[i]; i++) {
        qtest_writel(qts, UART0 + UARTDR, msg[i]);
    }
    qtest_clock_step(qts, 4 * FRAME_NS);
    g_assert_cmpint(recv(sock, buf, 4, MSG_WAITALL), ==, 4);
    g_assert_cmpmem(buf, 4, msg, 4);

    g_assert_cmpint(send(sock, "x", 1, 0), ==, 1);
    g_assert_true(wait_rx(qts, UART0));
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTDR), ==, 'x');

    close(sock);
    qtest_quit(qts);
}

/*
 * The console sees UART0 only through its pins: output sent while GPIO 0
 * has another function never reaches it, nor does input while GPIO 1 has.
 */
/* [spec:nuos:req:emu.uart/test] */
/* [spec:nuos:req:emu.gpio/test] */
static void test_console_needs_pins(void)
{
    int sock;
    char buf[8];
    QTestState *qts = start_with_serial(&sock);

    uart_init(qts, UART0);
    qtest_writel(qts, UART0 + UARTDR, 'a');
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_BUSY, ==, 0);
    g_assert_cmpint(sock_poll(sock, buf, sizeof(buf)), <, 0);

    g_assert_cmpint(send(sock, "b", 1, 0), ==, 1);
    for (int i = 0; i < 20; i++) {
        qtest_clock_step(qts, FRAME_NS);
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_RXFE, ==, FR_RXFE);

    pin_function(qts, 0, FUNC_UART);
    qtest_writel(qts, UART0 + UARTDR, 'c');
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpint(recv(sock, buf, 1, MSG_WAITALL), ==, 1);
    g_assert_cmpint(buf[0], ==, 'c');

    pin_function(qts, 1, FUNC_UART);
    g_assert_cmpint(send(sock, "d", 1, 0), ==, 1);
    g_assert_true(wait_rx(qts, UART0));
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTDR), ==, 'd');

    close(sock);
    qtest_quit(qts);
}

/*
 * A character takes a frame time at the programmed rate, shifted out LSB
 * first after a start bit; BUSY covers it and TXFE does not.
 */
/* [spec:nuos:req:emu.uart/test] */
static void test_baud_timing(void)
{
    QTestState *qts = start();
    static const int bits[] = { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 };
    int i;

    uart_init(qts, UART0);
    pin_function(qts, 0, FUNC_UART);
    g_assert_true(pin_level(qts, 0));
    /* No pin selects UART0 CTS, so nUARTCTS is low: CTS asserted. */
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR), ==,
                    FR_TXFE | FR_RXFE | FR_CTS);

    /* 0x55: start 0, data 1,0,1,0,1,0,1,0, stop 1. */
    qtest_writel(qts, UART0 + UARTDR, 0x55);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & (FR_BUSY | FR_TXFE),
                    ==, FR_BUSY | FR_TXFE);
    qtest_clock_step(qts, BIT_NS / 2);
    for (i = 0; i < ARRAY_SIZE(bits); i++) {
        g_assert_cmpint(pin_level(qts, 0), ==, bits[i]);
        qtest_clock_step(qts, BIT_NS);
    }
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_BUSY, ==, 0);

    /* A divisor change takes effect at the next UARTLCR_H write. */
    qtest_writel(qts, UART0 + UARTIBRD, 162);
    qtest_writel(qts, UART0 + UARTFBRD, 48);
    qtest_writel(qts, UART0 + UARTDR, 0);
    qtest_clock_step(qts, FRAME_NS + 10);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_BUSY, ==, 0);
    qtest_writel(qts, UART0 + UARTLCR_H + SET, 0);
    qtest_writel(qts, UART0 + UARTDR, 0);
    qtest_clock_step(qts, FRAME_NS + 10);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_BUSY, ==, FR_BUSY);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_BUSY, ==, 0);
    qtest_quit(qts);
}

/*
 * The 32-entry transmit FIFO fills while the line drains it, and the
 * transmit interrupt asserts as it drains through the trigger level.
 */
/* [spec:nuos:req:emu.uart/test] */
static void test_tx_fifo(void)
{
    QTestState *qts = start();
    int i;

    uart_init(qts, UART0);
    qtest_writel(qts, UART0 + UARTIMSC, INT_TX);
    /* The first character goes straight to the shift register. */
    for (i = 0; i < 33; i++) {
        g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_TXFF, ==, 0);
        qtest_writel(qts, UART0 + UARTDR, i);
    }
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_TXFF, ==, FR_TXFF);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTRIS) & INT_TX, ==, 0);

    /* TXIFLSEL 1/2: the interrupt comes when 16 are left. */
    qtest_clock_step(qts, 15 * FRAME_NS + BIT_NS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTRIS) & INT_TX, ==, 0);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTRIS) & INT_TX, ==, INT_TX);
    qtest_writel(qts, UART0 + UARTDR, 0);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTRIS) & INT_TX, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.uart/test] */
static void test_atomic_aliases(void)
{
    QTestState *qts = start();

    qtest_writel(qts, UART0 + UARTLCR_H, LCR_H_WLEN8);
    /* hw_write_masked() goes through the XOR alias. */
    qtest_writel(qts, UART0 + UARTLCR_H + XOR, LCR_H_FEN);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTLCR_H), ==,
                    LCR_H_WLEN8 | LCR_H_FEN);
    qtest_writel(qts, UART0 + UARTLCR_H + CLR, LCR_H_WLEN8);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTLCR_H), ==, LCR_H_FEN);
    g_assert_cmphex(qtest_readl(qts, UART0 + SET + UARTLCR_H), ==, LCR_H_FEN);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.uart/test] */
static void test_rx_interrupt(void)
{
    int core;

    for (core = 0; core < 2; core++) {
        g_autofree char *cpu = g_strdup_printf("/machine/soc/armv7m[%d]",
                                               core);
        int sock;
        QTestState *qts = start_with_serial(&sock);

        qtest_irq_intercept_in(qts, cpu);
        uart_init(qts, UART0);
        pin_function(qts, 1, FUNC_UART);
        qtest_writel(qts, UART0 + UARTIMSC, INT_RX);
        g_assert_false(qtest_get_irq(qts, UART0_IRQ));

        /* With FIFOs on, RX interrupts on the 1/2 fill level. */
        qtest_writel(qts, UART0 + UARTLCR_H + CLR, LCR_H_FEN);
        g_assert_cmpint(send(sock, "y", 1, 0), ==, 1);
        g_assert_true(wait_rx(qts, UART0));
        g_assert_true(qtest_get_irq(qts, UART0_IRQ));
        g_assert_cmphex(qtest_readl(qts, UART0 + UARTMIS), ==, INT_RX);

        qtest_readl(qts, UART0 + UARTDR);
        g_assert_false(qtest_get_irq(qts, UART0_IRQ));

        close(sock);
        qtest_quit(qts);
    }
}

/*
 * Run the clock for `ns`, with GPIO 0 (UART0 TX) wired to GPIO 5 (UART1
 * RX) outside the chip: the test copies the one pin to the other every
 * microsecond.
 */
static void run_wired(QTestState *qts, int64_t ns)
{
    int last = -1;

    for (; ns > 0; ns -= 1000) {
        int level = pin_level(qts, 0);

        if (level != last) {
            qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, level);
            last = level;
        }
        qtest_clock_step(qts, 1000);
    }
}

/*
 * UART0 TX looped to UART1 RX through their pins: characters arrive
 * intact with matching formats, a parity mismatch is a parity error, and
 * a receive FIFO left full overruns. Data held in the FIFO raises the
 * receive timeout after 32 bit periods.
 */
/* [spec:nuos:req:emu.uart/test] */
/* [spec:nuos:req:emu.gpio/test] */
static void test_pin_loopback(void)
{
    QTestState *qts = start();
    int i;

    uart_init(qts, UART0);
    uart_init(qts, UART1);
    pin_function(qts, 0, FUNC_UART);
    pin_function(qts, 5, FUNC_UART);
    /* The wire idles high, as UART0 TX does. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, 1);
    qtest_writel(qts, UART1 + UARTIMSC, INT_RT);

    qtest_writel(qts, UART0 + UARTDR, 'O');
    qtest_writel(qts, UART0 + UARTDR, 'K');
    run_wired(qts, 2 * FRAME_NS + BIT_NS);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTRIS) & INT_RT, ==, 0);
    run_wired(qts, 32 * BIT_NS);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTRIS) & INT_RT, ==, INT_RT);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTDR), ==, 'O');
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTDR), ==, 'K');
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTRIS) & INT_RT, ==, 0);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTFR) & FR_RXFE, ==, FR_RXFE);

    /* Even parity sent, odd parity expected. */
    qtest_writel(qts, UART0 + UARTLCR_H,
                 LCR_H_WLEN8 | LCR_H_FEN | LCR_H_PEN | LCR_H_EPS);
    qtest_writel(qts, UART1 + UARTLCR_H, LCR_H_WLEN8 | LCR_H_FEN | LCR_H_PEN);
    qtest_writel(qts, UART0 + UARTDR, 0x31);
    run_wired(qts, FRAME_NS + 2 * BIT_NS);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTDR), ==, DR_PE | 0x31);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTRSR), ==, DR_PE >> 8);
    /* Writing UARTECR clears the error status. */
    qtest_writel(qts, UART1 + UARTRSR, 0);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTRSR), ==, 0);

    /* 33 characters into a 32-entry FIFO: the 33rd is lost. */
    qtest_writel(qts, UART1 + UARTLCR_H,
                 LCR_H_WLEN8 | LCR_H_FEN | LCR_H_PEN | LCR_H_EPS);
    for (i = 0; i < 34; i++) {
        qtest_writel(qts, UART0 + UARTDR, i);
        run_wired(qts, 12 * BIT_NS);
    }
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTRSR), ==, DR_OE >> 8);
    for (i = 0; i < 32; i++) {
        g_assert_cmphex(qtest_readl(qts, UART1 + UARTDR), ==, i);
    }
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTFR) & FR_RXFE, ==, FR_RXFE);
    /* The next character carries the overrun. */
    qtest_writel(qts, UART0 + UARTDR, 0x77);
    run_wired(qts, 12 * BIT_NS);
    g_assert_cmphex(qtest_readl(qts, UART1 + UARTDR), ==, DR_OE | 0x77);
    qtest_quit(qts);
}

/*
 * CTS flow control: with CTSEn the transmitter holds characters while
 * nUARTCTS (GPIO 2, UART0 CTS) is high, and a CTS change raises the CTS
 * modem status interrupt.
 */
/* [spec:nuos:req:emu.uart/test] */
static void test_cts_flow(void)
{
    int sock;
    char buf[2];
    QTestState *qts = start_with_serial(&sock);

    uart_init(qts, UART0);
    pin_function(qts, 0, FUNC_UART);
    pin_function(qts, 2, FUNC_UART);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 2, 1);
    qtest_writel(qts, UART0 + UARTICR, 0x7ff);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_CTS, ==, 0);

    qtest_writel(qts, UART0 + UARTCR + SET, CR_CTSEN);
    qtest_writel(qts, UART0 + UARTDR, 'f');
    qtest_clock_step(qts, 10 * FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) &
                    (FR_BUSY | FR_TXFE), ==, FR_BUSY);
    g_assert_true(pin_level(qts, 0));
    g_assert_cmpint(sock_poll(sock, buf, 1), <, 0);

    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 2, 0);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_CTS, ==, FR_CTS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTRIS) & INT_CTS, ==, INT_CTS);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_BUSY, ==, 0);
    g_assert_cmpint(recv(sock, buf, 1, MSG_WAITALL), ==, 1);
    g_assert_cmpint(buf[0], ==, 'f');

    /* Without CTSEn, nUARTCTS only shows in the flags. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 2, 1);
    qtest_writel(qts, UART0 + UARTCR + CLR, CR_CTSEN);
    qtest_writel(qts, UART0 + UARTDR, 'g');
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpint(recv(sock, buf, 1, MSG_WAITALL), ==, 1);
    g_assert_cmpint(buf[0], ==, 'g');

    close(sock);
    qtest_quit(qts);
}

/*
 * RTS flow control: with RTSEn, nUARTRTS (GPIO 3, UART0 RTS) is asserted
 * low until the receive FIFO reaches its trigger level.
 */
/* [spec:nuos:req:emu.uart/test] */
static void test_rts_flow(void)
{
    int sock;
    QTestState *qts = start_with_serial(&sock);

    uart_init(qts, UART0);
    pin_function(qts, 1, FUNC_UART);
    pin_function(qts, 3, FUNC_UART);
    /* RTS deasserted (high) unless software asserts it. */
    g_assert_true(pin_level(qts, 3));
    qtest_writel(qts, UART0 + UARTCR + SET, 1u << 11);
    g_assert_false(pin_level(qts, 3));

    /* RXIFLSEL 1/8: four characters. */
    qtest_writel(qts, UART0 + UARTIFLS, 0);
    qtest_writel(qts, UART0 + UARTCR + SET, CR_RTSEN);
    qtest_writel(qts, UART0 + UARTCR + CLR, 1u << 11);
    g_assert_false(pin_level(qts, 3));
    g_assert_cmpint(send(sock, "1234", 4, 0), ==, 4);
    for (int i = 0; i < 1000 && !pin_level(qts, 3); i++) {
        qtest_clock_step(qts, BIT_NS);
        g_usleep(100);
    }
    g_assert_true(pin_level(qts, 3));
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTDR), ==, '1');
    g_assert_false(pin_level(qts, 3));

    close(sock);
    qtest_quit(qts);
}

/*
 * Break: UARTLCR_H.BRK holds TX low; in loopback the receiver takes a
 * single zero character with the break error and waits for the line to
 * return high.
 */
/* [spec:nuos:req:emu.uart/test] */
static void test_break(void)
{
    QTestState *qts = start();

    uart_init(qts, UART0);
    pin_function(qts, 0, FUNC_UART);
    qtest_writel(qts, UART0 + UARTCR + SET, CR_LBE);
    qtest_writel(qts, UART0 + UARTLCR_H + SET, LCR_H_BRK);
    g_assert_false(pin_level(qts, 0));
    qtest_clock_step(qts, 5 * FRAME_NS);
    qtest_writel(qts, UART0 + UARTLCR_H + CLR, LCR_H_BRK);
    g_assert_true(pin_level(qts, 0));
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTRIS) & INT_BE, ==, INT_BE);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTDR), ==, DR_BE);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTFR) & FR_RXFE, ==, FR_RXFE);

    /* A break waits for the character in progress. */
    qtest_writel(qts, UART0 + UARTDR, 0xff);
    qtest_writel(qts, UART0 + UARTLCR_H + SET, LCR_H_BRK);
    qtest_clock_step(qts, BIT_NS * 3 / 2);
    g_assert_true(pin_level(qts, 0));
    qtest_clock_step(qts, FRAME_NS);
    g_assert_false(pin_level(qts, 0));
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTDR), ==, 0xff);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-uart-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/uart/tx-rx", test_tx_rx);
    qtest_add_func("/rp2350/uart/console-needs-pins", test_console_needs_pins);
    qtest_add_func("/rp2350/uart/baud-timing", test_baud_timing);
    qtest_add_func("/rp2350/uart/tx-fifo", test_tx_fifo);
    qtest_add_func("/rp2350/uart/atomic-aliases", test_atomic_aliases);
    qtest_add_func("/rp2350/uart/rx-interrupt", test_rx_interrupt);
    qtest_add_func("/rp2350/uart/pin-loopback", test_pin_loopback);
    qtest_add_func("/rp2350/uart/cts-flow", test_cts_flow);
    qtest_add_func("/rp2350/uart/rts-flow", test_rts_flow);
    qtest_add_func("/rp2350/uart/break", test_break);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
