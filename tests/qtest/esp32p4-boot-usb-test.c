/*
 * QTest testcase for the ESP32-P4 boot ROM's download over the USB OTG,
 * and its loader's commands over USB Serial/JTAG as esptool drives them
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/un.h>
#include "libqtest.h"
#include "esp32p4-boot-utils.h"

#define USJ_CHIP_RST        0x500d204c
#define CHIP_RST_DTR        (1u << 1)

/*
 * A unix socket the test listens on, for a chardev QEMU connects as a
 * client: its path in *path.
 */
static int listen_socket(char **path)
{
    g_autofree char *dir = g_dir_make_tmp("esp32p4-boot-XXXXXX", NULL);
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);

    g_assert_nonnull(dir);
    g_assert_cmpint(fd, >=, 0);
    *path = g_strdup_printf("%s/sock", dir);
    g_strlcpy(addr.sun_path, *path, sizeof(addr.sun_path));
    g_assert_cmpint(bind(fd, (struct sockaddr *)&addr, sizeof(addr)), ==, 0);
    g_assert_cmpint(listen(fd, 1), ==, 0);
    return fd;
}

static void unlink_socket(char *path)
{
    g_autofree char *dir = g_path_get_dirname(path);

    unlink(path);
    rmdir(dir);
    g_free(path);
}

/*
 * The download over the USB OTG: the ROM's USB stack on OTG_HS presents a
 * CDC-ACM serial port, which the computer at the other end of the cable
 * (the controller's CDC-ACM bridge, "cdc-chardev") enumerates; esptool's
 * protocol then runs over it. Strapped 0x0 (GPIO38 held low by the
 * board's terminal, which holds a break on U0RXD through the reset), the
 * ROM's USB_BOOT waits on USB alone; in joint download boot it polls the
 * USB OTG with UART0, USB Serial/JTAG and the SPI slave.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_usb_otg_download(void)
{
    static const struct {
        uint32_t strap;
        const char *args;
        const char *banner;
        const char *wait;
    } modes[] = {
        { 0x00, "-global driver=esp32.uart-terminal,property=break,value=on",
          "boot:0x0 (USB_BOOT)", "wait usb download" },
        { 0x04, "", "boot:0x5 (DOWNLOAD(USB/UART0/SPI))",
          "waiting for download" },
    };

    for (int m = 0; m < ARRAY_SIZE(modes); m++) {
        char *path;
        int lfd = listen_socket(&path);
        int fd;
        Boot b;
        QTestState *qts;
        g_autofree char *log = NULL;
        int lfd2 = g_file_open_tmp("esp32p4-boot-XXXXXX.log", &log, NULL);

        close(lfd2);
        qts = qtest_initf("-M esp32p4,strap-mode=0x%x -accel tcg "
                          "-icount shift=0,sleep=off %s "
                          "-chardev file,id=s0,path=%s -serial chardev:s0 "
                          "-chardev socket,id=cdc,path=%s "
                          "-global esp32p4-usb-otg-hs.cdc-chardev=cdc",
                          modes[m].strap, modes[m].args, log, path);
        fd = accept(lfd, NULL, NULL);
        g_assert_cmpint(fd, >=, 0);
        b.qts = qts;
        b.log = g_strdup(log);
        await_output(&b, modes[m].banner);
        await_output(&b, modes[m].wait);
        esp_load_and_run(qts, fd);
        close(fd);
        close(lfd);
        unlink_socket(path);
        boot_end(&b);
    }
}

/*
 * The ROM's loader in joint download boot over USB Serial/JTAG
 * (esp_port_check_loader()), under icount or on the host's clock, where
 * the models' timers run as late as the host makes them: 4 KiB, 1024
 * READ_REG round trips, each response a bulk IN transfer the ROM waits
 * at most 5 ms to send. The host enumerates the port and opens it (DTR)
 * before esptool sends anything, as on a computer.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void run_usj_loader(bool icount)
{
    g_autofree char *dir = g_dir_make_tmp("esp32p4-boot-XXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/sock", dir);
    int srv = qtest_socket_server(path);
    EspPort p = { .rx = g_byte_array_new() };
    int i;

    p.qts = qtest_initf("-M esp32p4,strap-mode=0x04 %s -serial none "
                        "-chardev socket,id=usj,path=%s "
                        "-global esp32p4-usb-serial-jtag.chardev=usj",
                        esp_timing(icount), path);
    p.fd = accept(srv, NULL, NULL);
    g_assert_cmpint(p.fd, >=, 0);
    close(srv);
    unlink(path);
    rmdir(dir);
    for (i = 0; i < 300; i++) {
        if (qtest_readl(p.qts, USJ_CHIP_RST) & CHIP_RST_DTR) {
            break;
        }
        g_usleep(100 * 1000);
    }
    g_assert_cmpint(i, <, 300);
    esp_port_check_loader(&p, 4);
    close(p.fd);
    qtest_quit(p.qts);
    g_byte_array_free(p.rx, true);
}

static void test_usj_loader_icount(void)
{
    run_usj_loader(true);
}

static void test_usj_loader_host_clock(void)
{
    run_usj_loader(false);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/boot/usb-otg-download", test_usb_otg_download);
    qtest_add_func("/esp32p4/boot/usj-loader/icount", test_usj_loader_icount);
    qtest_add_func("/esp32p4/boot/usj-loader/host-clock",
                   test_usj_loader_host_clock);
    return g_test_run();
}
