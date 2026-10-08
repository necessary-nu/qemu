/*
 * QTest testcase for the RP2350 UARTs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define UART0       0x40070000
#define XOR         0x1000
#define SET         0x2000
#define CLR         0x3000

#define UARTDR      0x000
#define UARTFR      0x018
#define UARTIBRD    0x024
#define UARTFBRD    0x028
#define UARTLCR_H   0x02c
#define UARTCR      0x030
#define UARTIMSC    0x038
#define UARTMIS     0x040
#define UARTICR     0x044

#define FR_RXFE     (1u << 4)
#define LCR_H_FEN   (1u << 4)
#define LCR_H_WLEN8 (3u << 5)
#define CR_UARTEN   (1u << 0)
#define CR_TXE      (1u << 8)
#define CR_RXE      (1u << 9)
#define INT_RX      (1u << 4)

#define UART0_IRQ   33

static char *rom_path;

/* uart_init(): divisors, 8N1 with FIFOs, then enable via the SET alias. */
static void uart_init(QTestState *qts)
{
    qtest_writel(qts, UART0 + UARTIBRD, 81);
    qtest_writel(qts, UART0 + UARTFBRD, 24);
    qtest_writel(qts, UART0 + UARTLCR_H, LCR_H_WLEN8 | LCR_H_FEN);
    qtest_writel(qts, UART0 + UARTCR + SET, CR_UARTEN | CR_TXE | CR_RXE);
}

static bool wait_rx(QTestState *qts)
{
    int i;

    for (i = 0; i < 1000; i++) {
        if (!(qtest_readl(qts, UART0 + UARTFR) & FR_RXFE)) {
            return true;
        }
        g_usleep(1000);
    }
    return false;
}

/* [spec:nuos:req:emu.uart/test] */
static void test_tx_rx(void)
{
    g_autofree char *args = g_strdup_printf("-M rp2350 -bios %s", rom_path);
    int sock;
    char buf[8];
    QTestState *qts = qtest_init_with_serial(args, &sock);
    const char *msg = "nuos";
    int i;

    uart_init(qts);
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTCR) &
                    (CR_UARTEN | CR_TXE | CR_RXE), ==,
                    CR_UARTEN | CR_TXE | CR_RXE);

    for (i = 0; msg[i]; i++) {
        qtest_writel(qts, UART0 + UARTDR, msg[i]);
    }
    g_assert_cmpint(recv(sock, buf, 4, MSG_WAITALL), ==, 4);
    g_assert_cmpmem(buf, 4, msg, 4);

    g_assert_cmpint(send(sock, "x", 1, 0), ==, 1);
    g_assert_true(wait_rx(qts));
    g_assert_cmphex(qtest_readl(qts, UART0 + UARTDR) & 0xff, ==, 'x');

    close(sock);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.uart/test] */
static void test_atomic_aliases(void)
{
    g_autofree char *args = g_strdup_printf("-M rp2350 -bios %s", rom_path);
    QTestState *qts = qtest_init(args);

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
        g_autofree char *args = g_strdup_printf("-M rp2350 -bios %s",
                                                rom_path);
        int sock;
        QTestState *qts = qtest_init_with_serial(args, &sock);

        qtest_irq_intercept_in(qts, cpu);
        uart_init(qts);
        qtest_writel(qts, UART0 + UARTIMSC, INT_RX);
        g_assert_false(qtest_get_irq(qts, UART0_IRQ));

        /* With FIFOs on, RX interrupts on the 1/2 fill level. */
        qtest_writel(qts, UART0 + UARTLCR_H + CLR, LCR_H_FEN);
        g_assert_cmpint(send(sock, "y", 1, 0), ==, 1);
        g_assert_true(wait_rx(qts));
        g_assert_true(qtest_get_irq(qts, UART0_IRQ));
        g_assert_cmphex(qtest_readl(qts, UART0 + UARTMIS), ==, INT_RX);

        qtest_readl(qts, UART0 + UARTDR);
        g_assert_false(qtest_get_irq(qts, UART0_IRQ));

        close(sock);
        qtest_quit(qts);
    }
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
    qtest_add_func("/rp2350/uart/atomic-aliases", test_atomic_aliases);
    qtest_add_func("/rp2350/uart/rx-interrupt", test_rx_interrupt);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
