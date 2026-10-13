/*
 * QTest testcase for the pads the ESP32 machine's serial terminals sit on
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * serial0 is on the ESP32-WROOM-32E's TXD0 and RXD0 (GPIO1, GPIO3);
 * serial1, whose UART's own IO_MUX pads are the module's flash's WP# and
 * HOLD#, on GPIO33 (TX) and GPIO32 (RX) by default; serial2 on UART2's
 * IO_MUX pads, GPIO17 and GPIO16. A terminal receiving does not touch the
 * flash's pads, and the machine refuses a terminal on them.
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include <sys/un.h>
#include "libqtest.h"
#include "esp32-flash-pins.h"
#include "esp32-spi-flash.h"

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define PERIP_UART1             (1u << 5)
#define PERIP_UART_MEM          (1u << 24)

#define GPIO_IN                 (FP_GPIO + 0x3c)
#define GPIO_IN1                (FP_GPIO + 0x40)
#define GPIO_ENABLE1_W1TS       (FP_GPIO + 0x30)
#define IN_SIG_IN_SEL           (1u << 7)
#define SIG_U1RXD               17
#define SIG_U1TXD               17

#define IO_MUX_GPIO32           (FP_IO_MUX + 0x1c)
#define IO_MUX_GPIO33           (FP_IO_MUX + 0x20)
#define FUN_IE                  (1u << 9)
#define MCU_SEL(f)              ((f) << 12)
#define PIN_FUNC_GPIO           2

#define UART1                   0x3ff50000
#define UART_FIFO               0x00
#define UART_CLKDIV             0x14
#define UART_STATUS             0x1c
#define RXFIFO_CNT(st)          ((st) & 0xff)
/* 115200 baud from the 40 MHz APB_CLK the machine starts with */
#define CLKDIV_115200           (347 | (3u << 20))

#define FLASH_SIZE              (4 * 1024 * 1024)
#define MS                      (1000 * 1000)

static char *flash_path;

static uint8_t flash_byte(uint32_t a)
{
    return (uint8_t)(a * 13 + (a >> 8) + 1);
}

static uint32_t flash_word(uint32_t a)
{
    return flash_byte(a) | flash_byte(a + 1) << 8 | flash_byte(a + 2) << 16 |
           (uint32_t)flash_byte(a + 3) << 24;
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * serial1 receiving zeros, without end: its RX pad, GPIO32, goes low with
 * each byte, while the flash's HOLD# (GPIO9) stays high and SPI1 reads the
 * flash throughout.
 */
static void test_flash_untouched(void)
{
    QTestState *qts = qtest_initf("-M esp32 -nic none "
                                  "-drive file=%s,if=mtd,format=raw "
                                  "-serial null -chardev file,id=u1,"
                                  "path=/dev/null,input-path=/dev/zero "
                                  "-serial chardev:u1", flash_path);
    unsigned lows = 0;

    flash_pins_rom(qts);
    sf_read_mode(qts, SF_SLOW, 0);
    for (int i = 0; i < 400; i++) {
        uint32_t a = i * 0x1234 % (FLASH_SIZE - 4);

        qtest_clock_step(qts, 7000);
        lows += !(qtest_readl(qts, GPIO_IN1) & 1);
        g_assert_cmphex(qtest_readl(qts, GPIO_IN) & (1u << 9), ==, 1u << 9);
        g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));
    }
    g_assert_cmpuint(lows, >, 0);
    qtest_quit(qts);
}

/* A listening socket at a new path, for a -chardev socket to connect to */
static int listen_at(char **path)
{
    g_autofree char *dir = g_dir_make_tmp("esp32-terminal-XXXXXX", NULL);
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);

    g_assert(dir && fd >= 0);
    *path = g_strdup_printf("%s/sock", dir);
    g_strlcpy(addr.sun_path, *path, sizeof(addr.sun_path));
    g_assert_cmpint(bind(fd, (struct sockaddr *)&addr, sizeof(addr)), ==, 0);
    g_assert_cmpint(listen(fd, 1), ==, 0);
    return fd;
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * serial1 on GPIO33 and GPIO32: UART1 reaches it routed to them through
 * the GPIO matrix, its bytes out and the terminal's in; on UART1's own
 * IO_MUX pads, the flash's, it does not.
 */
static void test_serial1(void)
{
    char *path;
    int lfd = listen_at(&path);
    QTestState *qts = qtest_initf("-M esp32 -nic none -serial null "
                                  "-chardev socket,id=s1,path=%s "
                                  "-serial chardev:s1", path);
    int sock = accept(lfd, NULL, NULL);
    uint8_t got = 0;

    g_assert(sock >= 0);
    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_UART1 |
                 PERIP_UART_MEM);
    qtest_writel(qts, UART1 + UART_CLKDIV, CLKDIV_115200);

    /* U1TXD on its IO_MUX pad, GPIO10's function 4: not the terminal's */
    qtest_writel(qts, flash_pin_mux_addr(FLASH_PIN_WP), MCU_SEL(4) | FUN_IE);
    qtest_writel(qts, UART1 + UART_FIFO, 'x');
    for (int i = 0; i < 20; i++) {
        qtest_clock_step(qts, MS);
        g_usleep(1000);
    }
    g_assert_cmpint(recv(sock, &got, 1, MSG_DONTWAIT), <=, 0);

    qtest_writel(qts, IO_MUX_GPIO33, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, FP_FUNC_OUT(33), SIG_U1TXD);
    qtest_writel(qts, GPIO_ENABLE1_W1TS, 1u << (33 - 32));
    qtest_writel(qts, IO_MUX_GPIO32, MCU_SEL(PIN_FUNC_GPIO) | FUN_IE);
    qtest_writel(qts, FP_FUNC_IN(SIG_U1RXD), IN_SIG_IN_SEL | 32);
    qtest_writel(qts, UART1 + UART_FIFO, 'A');
    for (int i = 0; i < 2000 && got != 'A'; i++) {
        qtest_clock_step(qts, MS);
        if (recv(sock, &got, 1, MSG_DONTWAIT) <= 0) {
            g_usleep(1000);
        }
    }
    g_assert_cmphex(got, ==, 'A');

    g_assert_cmpint(send(sock, "Z", 1, 0), ==, 1);
    for (int i = 0; i < 2000 &&
         !RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)); i++) {
        g_usleep(1000);
        qtest_clock_step(qts, 100 * 1000);
    }
    g_assert_cmpuint(RXFIFO_CNT(qtest_readl(qts, UART1 + UART_STATUS)), ==, 1);
    g_assert_cmphex(qtest_readl(qts, UART1 + UART_FIFO), ==, 'Z');
    qtest_quit(qts);
    close(sock);
    close(lfd);
    unlink(path);
    g_free(path);
}

/* QEMU's exit status for a command line the machine must refuse */
static int run_qemu(const char *args)
{
    g_autofree char *cmd = g_strdup_printf("%s -display none -serial none "
                                           "-monitor none -nic none %s",
                                           qtest_qemu_binary(NULL), args);
    g_auto(GStrv) argv = NULL;
    GError *err = NULL;
    int status;

    g_assert(g_shell_parse_argv(cmd, NULL, &argv, &err));
    g_assert(g_spawn_sync(NULL, argv, NULL,
                          G_SPAWN_STDOUT_TO_DEV_NULL |
                          G_SPAWN_STDERR_TO_DEV_NULL, NULL, NULL, NULL, NULL,
                          &status, &err));
    g_assert_no_error(err);
    return status;
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * A terminal on the flash's pads, on a pad with no output driver, or on
 * another terminal's pad is refused.
 */
static void test_refused(void)
{
    g_assert_cmpint(run_qemu("-M esp32,uart1-tx=10"), !=, 0);
    g_assert_cmpint(run_qemu("-M esp32,uart1-rx=9"), !=, 0);
    g_assert_cmpint(run_qemu("-M esp32,uart1-tx=35"), !=, 0);
    g_assert_cmpint(run_qemu("-M esp32,uart2-rx=33"), !=, 0);
}

int main(int argc, char **argv)
{
    g_autofree uint8_t *data = g_malloc(FLASH_SIZE);
    int fd, ret;

    for (uint32_t a = 0; a < FLASH_SIZE; a++) {
        data[a] = flash_byte(a);
    }
    fd = g_file_open_tmp("esp32-uart-terminal-pads-XXXXXX", &flash_path,
                         NULL);
    g_assert(fd >= 0);
    g_assert_cmpint(write(fd, data, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);

    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/uart-terminal-pads/flash-untouched",
                   test_flash_untouched);
    qtest_add_func("/esp32/uart-terminal-pads/serial1", test_serial1);
    qtest_add_func("/esp32/uart-terminal-pads/refused", test_refused);
    ret = g_test_run();
    unlink(flash_path);
    g_free(flash_path);
    return ret;
}
