/*
 * QTest testcase for the ESP32-P4 USB full-speed PHYs' pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * PHY 0 has GPIO24 (D-) and GPIO25 (D+), PHY 1 GPIO26 and GPIO27; USB
 * Serial/JTAG has PHY 0 and OTG_FS PHY 1 unless LP_SYS_USB_CTRL swaps
 * them. While its controller's USB_PAD_ENABLE is set, a PHY has its pads:
 * a GPIO output there drives nothing, and the USB keeps working. Clearing
 * it, as ESP-IDF's gpio_ll_func_sel() does before it selects a function
 * for one of these pads, gives the pads to the GPIO matrix and detaches
 * the device.
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include "libqtest.h"

#define GPIO                0x500e0000u
#define GPIO_OUT_W1TS       (GPIO + 0x008)
#define GPIO_OUT_W1TC       (GPIO + 0x00c)
#define GPIO_ENABLE_W1TS    (GPIO + 0x024)
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define SIG_GPIO_OUT        0x100
#define IO_MUX_GPIO(n)      (0x500e1004u + 4 * (n))
#define FUN_IE              (1u << 9)
#define FUN_DRV(d)          ((uint32_t)(d) << 10)
#define MCU_SEL_GPIO        (1u << 12)

#define USJ                 0x500d2000u
#define USJ_INT_RAW         (USJ + 0x08)
#define USJ_INT_CLR         (USJ + 0x14)
#define USJ_CONF0           (USJ + 0x18)
#define USJ_BUS_RESET_ST    (USJ + 0x68)
#define USJ_INT_SOF         (1u << 1)
#define USJ_PAD_ENABLE      (1u << 14)

#define USB_WRAP            0x50080000u
#define OTG_CONF_PAD_PULL_OVERRIDE (1u << 12)
#define OTG_CONF_DP_PULLUP  (1u << 13)
#define OTG_CONF_USB_PAD_ENABLE (1u << 18)
#define HP_SYS_CLKRST_SOC_CLK_CTRL1 0x500e6018u
#define USB_OTG11_SYS_CLK_EN (1u << 15)
#define LP_SYS_USB_CTRL     0x50110100u
#define SW_HW_USB_PHY_SEL   (1u << 0)
#define SW_USB_PHY_SEL      (1u << 1)

#define MS                  1000000

static bool gpio_in(QTestState *qts, int n)
{
    return (qtest_readl(qts, GPIO_IN) >> n) & 1;
}

/*
 * Pad n a GPIO output at @level, its input on, written straight to the IO
 * MUX: without gpio_ll_func_sel()'s clearing of USB_PAD_ENABLE.
 */
static void drive(QTestState *qts, int n, bool level)
{
    qtest_writel(qts, IO_MUX_GPIO(n), MCU_SEL_GPIO | FUN_IE | FUN_DRV(2));
    qtest_writel(qts, GPIO_FUNC_OUT(n), SIG_GPIO_OUT);
    qtest_writel(qts, level ? GPIO_OUT_W1TS : GPIO_OUT_W1TC, 1u << n);
    qtest_writel(qts, GPIO_ENABLE_W1TS, 1u << n);
}

static void usj_release(QTestState *qts)
{
    qtest_writel(qts, USJ_CONF0,
                 qtest_readl(qts, USJ_CONF0) & ~USJ_PAD_ENABLE);
}

/*
 * Out of reset USB Serial/JTAG's PHY has GPIO24 and GPIO25: GPIO outputs
 * there drive nothing, D- floating low and D+ held up by the PHY. Once
 * USB_PAD_ENABLE is cleared they drive the pads.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_usj_pads(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    drive(qts, 24, true);
    drive(qts, 25, false);
    g_assert_false(gpio_in(qts, 24));
    g_assert_true(gpio_in(qts, 25));
    usj_release(qts);
    g_assert_true(gpio_in(qts, 24));
    g_assert_false(gpio_in(qts, 25));
    /* Given back to the PHY, the outputs are off the pads again. */
    qtest_writel(qts, USJ_CONF0,
                 qtest_readl(qts, USJ_CONF0) | USJ_PAD_ENABLE);
    g_assert_false(gpio_in(qts, 24));
    g_assert_true(gpio_in(qts, 25));
    qtest_quit(qts);
}

/*
 * OTG_FS's PHY 1, GPIO26 and GPIO27, its D+ pulled up by the override:
 * the PHY's while USB_WRAP's USB_PAD_ENABLE is set.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_otg_pads(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1,
                 qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1) |
                 USB_OTG11_SYS_CLK_EN);
    drive(qts, 26, true);
    drive(qts, 27, false);
    g_assert_true(gpio_in(qts, 26));
    g_assert_false(gpio_in(qts, 27));
    qtest_writel(qts, USB_WRAP, OTG_CONF_USB_PAD_ENABLE |
                 OTG_CONF_PAD_PULL_OVERRIDE | OTG_CONF_DP_PULLUP);
    g_assert_false(gpio_in(qts, 26));
    g_assert_true(gpio_in(qts, 27));
    qtest_writel(qts, USB_WRAP, 0);
    g_assert_true(gpio_in(qts, 26));
    g_assert_false(gpio_in(qts, 27));
    qtest_quit(qts);
}

/*
 * With the PHYs swapped (SW_HW_USB_PHY_SEL, SW_USB_PHY_SEL), USB
 * Serial/JTAG's PHY is on GPIO26 and GPIO27, and GPIO24 and GPIO25 are
 * OTG_FS's PHY's, free while USB_WRAP's USB_PAD_ENABLE is clear.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_swapped_phys(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_writel(qts, LP_SYS_USB_CTRL, SW_HW_USB_PHY_SEL | SW_USB_PHY_SEL);
    drive(qts, 24, true);
    drive(qts, 25, false);
    drive(qts, 26, true);
    drive(qts, 27, false);
    g_assert_true(gpio_in(qts, 24));
    g_assert_false(gpio_in(qts, 25));
    g_assert_false(gpio_in(qts, 26));
    g_assert_true(gpio_in(qts, 27));
    usj_release(qts);
    g_assert_true(gpio_in(qts, 26));
    g_assert_false(gpio_in(qts, 27));
    qtest_quit(qts);
}

/*
 * A computer on the cable: USB Serial/JTAG enumerates and gets an SOF
 * every millisecond. A GPIO output enabled on D+ while the PHY has the
 * pads changes nothing; clearing USB_PAD_ENABLE takes its D+ pull-up away,
 * and the host sees the device go.
 */
/* [spec:nuos:req:emu.esp32p4.usb/test] */
/* [spec:nuos:req:emu.esp32p4.gpio/test] */
static void test_usj_host(void)
{
    g_autofree char *dir = g_dir_make_tmp("esp32p4-usbpads-XXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/sock", dir);
    int srv = qtest_socket_server(path);
    QTestState *qts;
    int fd;

    qts = qtest_initf("-M esp32p4 -serial none "
                      "-chardev socket,id=usj,path=%s "
                      "-global esp32p4-usb-serial-jtag.chardev=usj", path);
    fd = accept(srv, NULL, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(srv);
    unlink(path);
    rmdir(dir);

    qtest_clock_step(qts, 200 * MS);
    qtest_writel(qts, USJ_INT_CLR, USJ_INT_SOF);
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_SOF);

    drive(qts, 25, false);
    g_assert_true(gpio_in(qts, 25));
    qtest_writel(qts, USJ_INT_CLR, USJ_INT_SOF);
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_SOF);

    usj_release(qts);
    g_assert_false(gpio_in(qts, 25));
    qtest_writel(qts, USJ_INT_CLR, USJ_INT_SOF);
    qtest_clock_step(qts, 5 * MS);
    g_assert_false(qtest_readl(qts, USJ_INT_RAW) & USJ_INT_SOF);
    g_assert_cmphex(qtest_readl(qts, USJ_BUS_RESET_ST), ==, 1);
    close(fd);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/usb-pads/usj_pads", test_usj_pads);
    qtest_add_func("esp32p4/usb-pads/otg_pads", test_otg_pads);
    qtest_add_func("esp32p4/usb-pads/swapped_phys", test_swapped_phys);
    qtest_add_func("esp32p4/usb-pads/usj_host", test_usj_host);
    return g_test_run();
}
