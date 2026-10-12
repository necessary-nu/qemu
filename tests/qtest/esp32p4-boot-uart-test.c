/*
 * QTest testcase for the ESP32-P4 boot ROM's download over serial port 0
 * (UART0): esptool's SYNC, then a program into L2MEM that the ROM runs, in
 * joint download boot and in UART0_BOOT
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "esp32p4-boot-utils.h"

/*
 * Joint download boot (GPIO35 low, GPIO36 high): the ROM waits for esptool
 * on UART0 (and USB Serial/JTAG, the USB OTG and the SPI slave); over
 * UART0 it syncs, takes a program into L2MEM (MEM_BEGIN, MEM_DATA) and
 * runs it (MEM_END).
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_uart_download(void)
{
    int fd;
    QTestState *qts = qtest_init_with_serial(
        "-M esp32p4,strap-mode=0x04 -accel tcg -icount shift=0,sleep=off",
        &fd);

    esp_load_and_run(qts, fd);
    close(fd);
    qtest_quit(qts);
}

/*
 * GPIO38 strapped low as a host does it (TRM 12.2.1): the board's
 * terminal on UART0's pads holds its TX line, U0RXD, low (a break)
 * through the power-on reset, then lets it go idle. The strapping 0x2
 * (GPIO37 high) is the ROM's UART0_BOOT: it waits for a download on UART0
 * alone, measures the baud rate of the host's SYNC, and runs the program
 * it is given.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_uart0_boot(void)
{
    int fd;
    QTestState *qts = qtest_init_with_serial(
        "-M esp32p4,strap-mode=0x02 -accel tcg -icount shift=0,sleep=off "
        "-global driver=esp32.uart-terminal,property=break,value=on", &fd);
    g_autoptr(GString) rx = g_string_new(NULL);
    QDict *rsp;

    g_assert_cmphex(qtest_readl(qts, GPIO_STRAP), ==, 0x02);
    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': {"
                    "'path': '/machine/terminal[0]', 'property': 'break', "
                    "'value': false}}");
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    await_text(fd, rx, "boot:0x2 (UART0_BOOT)");
    await_text(fd, rx, "wait uart0 download");
    esp_load_and_run(qts, fd);
    close(fd);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/boot/uart-download", test_uart_download);
    qtest_add_func("/esp32p4/boot/uart0-boot", test_uart0_boot);
    return g_test_run();
}
