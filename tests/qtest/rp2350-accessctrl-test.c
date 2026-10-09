/*
 * QTest testcase for the RP2350 bus access control (ACCESSCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses go straight to system memory and count as the debugger.
 * Accesses through core 0's view of the bus, with the debugger's
 * attributes, come from the monitor's "x" command.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ACCESSCTRL      0x40060000
#define LOCK            (ACCESSCTRL + 0x000)
#define FORCE_CORE_NS   (ACCESSCTRL + 0x004)
#define CFGRESET        (ACCESSCTRL + 0x008)
#define GPIO_NSMASK0    (ACCESSCTRL + 0x00c)
#define GPIO_NSMASK1    (ACCESSCTRL + 0x010)
#define REG_ROM         (ACCESSCTRL + 0x014)
#define REG_SRAM0       (ACCESSCTRL + 0x01c)
#define REG_DMA         (ACCESSCTRL + 0x044)
#define REG_UART0       (ACCESSCTRL + 0x0a0)
#define REG_SHA256      (ACCESSCTRL + 0x0b8)
#define REG_TICKS       (ACCESSCTRL + 0x0d4)
#define XOR             0x1000
#define SET             0x2000
#define CLR             0x3000

#define PASSWORD        0xacce0000u
#define LOCK_CORE0      (1u << 0)
#define LOCK_DMA        (1u << 2)
#define LOCK_DEBUG      (1u << 3)
#define PERM_DBG        (1u << 7)

#define TICKS           0x40108000
#define SRAM            0x20000000

#define SIO             0xd0000000
#define SIO_NONSEC      0xd0020000
#define GPIO_OUT        0x010
#define GPIO_HI_OUT     0x014
#define GPIO_OUT_SET    0x018
#define GPIO_OUT_XOR    0x028
#define GPIO_OE         0x030
#define GPIO_HI_OE      0x034

/* Reset values, offset 0x000 to 0x0e8, from the pico-sdk accessctrl.h. */
static const uint32_t reset_values[] = {
    0x04, 0x00, 0x00, 0x00, 0x00,                       /* LOCK..NSMASK1 */
    0xff, 0xff,                                         /* ROM, XIP_MAIN */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,     /* SRAM0-7 */
    0xff, 0xff,                                         /* SRAM8-9 */
    0xfc, 0xfc, 0xfc, 0xfc, 0xfc,                       /* DMA..PIO2 */
    0xb8, 0xb8,                                         /* CORESIGHT */
    0xff,                                               /* SYSINFO */
    0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc,                 /* RESETS..BUSCTRL */
    0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc,           /* ADC..SPI1 */
    0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc,                 /* TIMER0..TBMAN */
    0xb8, 0xb8,                                         /* POWMAN, TRNG */
    0xf8,                                               /* SHA256 */
    0xb8, 0xb8, 0xb8, 0xb8, 0xb8, 0xb8, 0xb8,           /* SYSCFG..TICKS */
    0xb8, 0xb8, 0xb8, 0xb8,                             /* WATCHDOG..QMI */
    0xf8,                                               /* XIP_AUX */
};

static char *rom_path;

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* Whether the debugger can read `addr` through core 0's view. */
static bool debug_readable(QTestState *qts, uint32_t addr)
{
    g_autofree char *out = qtest_hmp(qts, "x/1xw 0x%x", addr);

    return !strstr(out, "Cannot access memory");
}

/* [spec:nuos:req:emu.accessctrl/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int i;

    g_assert_cmpint(ARRAY_SIZE(reset_values), ==, 0xe8 / 4 + 1);
    for (i = 0; i < ARRAY_SIZE(reset_values); i++) {
        g_assert_cmphex(qtest_readl(qts, ACCESSCTRL + 4 * i), ==,
                        reset_values[i]);
        /* The atomic aliases read the register itself. */
        g_assert_cmphex(qtest_readl(qts, ACCESSCTRL + SET + 4 * i), ==,
                        reset_values[i]);
    }
    /* Past XIP_AUX the block reads zero. */
    g_assert_cmphex(qtest_readl(qts, ACCESSCTRL + 0xec), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.accessctrl/test] */
static void test_password(void)
{
    QTestState *qts = start();

    /* Without 0xacce in the upper half, writes are refused. */
    qtest_writel(qts, REG_UART0, 0x000000ff);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0xfc);
    qtest_writel(qts, REG_UART0, 0xacc00003);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0xfc);
    qtest_writel(qts, REG_UART0, PASSWORD | 0xff);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0xff);

    /* Only the eight permission bits are stored. */
    qtest_writel(qts, REG_UART0, PASSWORD | 0xff3c);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0x3c);

    /* The atomic aliases need the password too, and leave it out. */
    qtest_writel(qts, REG_UART0 + SET, 0x00000003);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0x3c);
    qtest_writel(qts, REG_UART0 + SET, PASSWORD | 0x03);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0x3f);
    qtest_writel(qts, REG_UART0 + CLR, PASSWORD | 0x30);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0x0f);
    qtest_writel(qts, REG_UART0 + XOR, PASSWORD | 0x81);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0x8e);

    /* The GPIO masks take no password and hold all their bits. */
    qtest_writel(qts, GPIO_NSMASK0, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, GPIO_NSMASK0), ==, 0xdeadbeef);
    qtest_writel(qts, GPIO_NSMASK1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO_NSMASK1), ==, 0xff00ffff);
    qtest_writel(qts, GPIO_NSMASK1 + CLR, 0x0000ff00);
    g_assert_cmphex(qtest_readl(qts, GPIO_NSMASK1), ==, 0xff0000ff);

    /* FORCE_CORE_NS has only its CORE1 bit. */
    qtest_writel(qts, FORCE_CORE_NS, PASSWORD | 0x3);
    g_assert_cmphex(qtest_readl(qts, FORCE_CORE_NS), ==, 0x2);
    qtest_writel(qts, FORCE_CORE_NS, 0x0);
    g_assert_cmphex(qtest_readl(qts, FORCE_CORE_NS), ==, 0x2);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.accessctrl/test] */
static void test_lock(void)
{
    QTestState *qts = start();

    /* Locking out core 0 leaves the debugger free to write. */
    qtest_writel(qts, LOCK, PASSWORD | LOCK_CORE0);
    g_assert_cmphex(qtest_readl(qts, LOCK), ==, LOCK_DMA | LOCK_CORE0);

    /* LOCK bits are set-only, and the DMA bit is read-only. */
    qtest_writel(qts, LOCK, PASSWORD);
    qtest_writel(qts, LOCK + CLR, PASSWORD | 0xf);
    g_assert_cmphex(qtest_readl(qts, LOCK), ==, LOCK_DMA | LOCK_CORE0);

    qtest_writel(qts, REG_TICKS, PASSWORD | 0xbc);
    g_assert_cmphex(qtest_readl(qts, REG_TICKS), ==, 0xbc);

    /* Once the debugger is locked out, its writes are ignored. */
    qtest_writel(qts, LOCK + SET, PASSWORD | LOCK_DEBUG);
    g_assert_cmphex(qtest_readl(qts, LOCK), ==,
                    LOCK_DMA | LOCK_CORE0 | LOCK_DEBUG);
    qtest_writel(qts, REG_TICKS, PASSWORD | 0xb8);
    g_assert_cmphex(qtest_readl(qts, REG_TICKS), ==, 0xbc);
    qtest_writel(qts, GPIO_NSMASK0, 0x1);
    g_assert_cmphex(qtest_readl(qts, GPIO_NSMASK0), ==, 0);
    qtest_writel(qts, CFGRESET, PASSWORD | 1);
    g_assert_cmphex(qtest_readl(qts, REG_TICKS), ==, 0xbc);

    /* Only a full reset of ACCESSCTRL clears LOCK. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, LOCK), ==, LOCK_DMA);
    g_assert_cmphex(qtest_readl(qts, REG_TICKS), ==, 0xb8);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.accessctrl/test] */
static void test_cfgreset(void)
{
    QTestState *qts = start();
    int i;

    for (i = 0x14; i <= 0xe8; i += 4) {
        qtest_writel(qts, ACCESSCTRL + i, PASSWORD | 0x55);
    }
    qtest_writel(qts, GPIO_NSMASK0, 0x12345678);
    qtest_writel(qts, GPIO_NSMASK1, 0x10000001);
    qtest_writel(qts, FORCE_CORE_NS, PASSWORD | 0x2);
    qtest_writel(qts, LOCK, PASSWORD | LOCK_CORE0);

    /* Without the password CFGRESET does nothing. */
    qtest_writel(qts, CFGRESET, 0x1);
    g_assert_cmphex(qtest_readl(qts, REG_ROM), ==, 0x55);
    /* Writing 0 does nothing either. */
    qtest_writel(qts, CFGRESET, PASSWORD);
    g_assert_cmphex(qtest_readl(qts, REG_ROM), ==, 0x55);

    qtest_writel(qts, CFGRESET, PASSWORD | 0x1);
    g_assert_cmphex(qtest_readl(qts, CFGRESET), ==, 0);
    for (i = 3; i < ARRAY_SIZE(reset_values); i++) {
        g_assert_cmphex(qtest_readl(qts, ACCESSCTRL + 4 * i), ==,
                        reset_values[i]);
    }
    /* LOCK and FORCE_CORE_NS survive it. */
    g_assert_cmphex(qtest_readl(qts, LOCK), ==, LOCK_DMA | LOCK_CORE0);
    g_assert_cmphex(qtest_readl(qts, FORCE_CORE_NS), ==, 0x2);
    qtest_quit(qts);
}

/*
 * The DBG bit decides whether the debugger reaches a block, here a
 * peripheral and one bank of the striped SRAM.
 */
/* [spec:nuos:req:emu.accessctrl/test] */
static void test_debugger(void)
{
    QTestState *qts = start();

    g_assert_true(debug_readable(qts, TICKS));
    qtest_writel(qts, REG_TICKS, PASSWORD | (0xb8 & ~PERM_DBG));
    g_assert_false(debug_readable(qts, TICKS));
    qtest_writel(qts, REG_TICKS, PASSWORD | 0xb8);
    g_assert_true(debug_readable(qts, TICKS));

    /* SRAM0-3 are striped on address bits 3:2; SRAM0 is every 4th word. */
    qtest_writel(qts, SRAM + 0x100, 0x11111111);
    qtest_writel(qts, REG_SRAM0, PASSWORD | (0xff & ~PERM_DBG));
    g_assert_false(debug_readable(qts, SRAM + 0x100));
    g_assert_true(debug_readable(qts, SRAM + 0x104));
    g_assert_true(debug_readable(qts, SRAM + 0x108));
    g_assert_true(debug_readable(qts, SRAM + 0x10c));
    g_assert_false(debug_readable(qts, SRAM + 0x110));
    /* The SRAM4-7 stripe has its own registers. */
    g_assert_true(debug_readable(qts, SRAM + 0x40000));
    qtest_writel(qts, REG_SRAM0, PASSWORD | 0xff);
    g_assert_true(debug_readable(qts, SRAM + 0x100));

    /* Clearing a master other than the debugger leaves it alone. */
    qtest_writel(qts, REG_DMA, PASSWORD | (0xfc & ~(1u << 6)));
    g_assert_true(debug_readable(qts, 0x50000000));
    qtest_quit(qts);
}

/*
 * GPIO_NSMASK0/1 decide which GPIOs the Non-secure SIO sees: the others
 * read as zero and ignore writes.
 */
/* [spec:nuos:req:emu.accessctrl/test] */
static void test_gpio_nsmask(void)
{
    QTestState *qts = start();

    qtest_writel(qts, SIO + GPIO_OUT, 0xffffffff);
    qtest_writel(qts, SIO + GPIO_OE, 0xffffffff);
    qtest_writel(qts, SIO + GPIO_HI_OUT, 0xff00ffff);
    qtest_writel(qts, SIO + GPIO_HI_OE, 0xff00ffff);

    /* At reset every GPIO is Secure-only. */
    g_assert_cmphex(qtest_readl(qts, SIO_NONSEC + GPIO_OUT), ==, 0);
    qtest_writel(qts, SIO_NONSEC + GPIO_OUT, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_OUT), ==, 0xffffffff);

    qtest_writel(qts, GPIO_NSMASK0, 0x0000ff00);
    qtest_writel(qts, GPIO_NSMASK1, 0x10000001);
    g_assert_cmphex(qtest_readl(qts, SIO_NONSEC + GPIO_OUT), ==, 0x0000ff00);
    g_assert_cmphex(qtest_readl(qts, SIO_NONSEC + GPIO_OE), ==, 0x0000ff00);
    g_assert_cmphex(qtest_readl(qts, SIO_NONSEC + GPIO_HI_OUT), ==,
                    0x10000001);

    /* Non-secure writes, plain or atomic, reach only the granted pins. */
    qtest_writel(qts, SIO_NONSEC + GPIO_OUT, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_OUT), ==, 0xffff00ff);
    qtest_writel(qts, SIO_NONSEC + GPIO_OUT_SET, 0x00000f0f);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_OUT), ==, 0xffff0fff);
    qtest_writel(qts, SIO_NONSEC + GPIO_OUT_XOR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_OUT), ==, 0xfffff0ff);
    qtest_writel(qts, SIO_NONSEC + GPIO_HI_OUT, 0);
    g_assert_cmphex(qtest_readl(qts, SIO + GPIO_HI_OUT), ==, 0xef00fffe);
    qtest_quit(qts);
}

/*
 * Narrow reads return their byte lanes of the register. Narrow writes are
 * replicated across the bus: a byte cannot carry the password, and a
 * halfword carries it only by being 0xacce, which then also lands in the
 * permission bits.
 */
/* [spec:nuos:req:emu.accessctrl/test] */
static void test_narrow(void)
{
    QTestState *qts = start();

    qtest_writel(qts, REG_UART0, PASSWORD | 0xa5);
    g_assert_cmphex(qtest_readb(qts, REG_UART0), ==, 0xa5);
    g_assert_cmphex(qtest_readb(qts, REG_UART0 + 1), ==, 0x00);
    g_assert_cmphex(qtest_readw(qts, REG_UART0), ==, 0x00a5);
    g_assert_cmphex(qtest_readw(qts, REG_UART0 + 2), ==, 0x0000);
    g_assert_cmphex(qtest_readb(qts, REG_SHA256), ==, 0xf8);

    qtest_writeb(qts, REG_UART0, 0xfc);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0xa5);
    qtest_writew(qts, REG_UART0 + 2, 0xacce);
    g_assert_cmphex(qtest_readl(qts, REG_UART0), ==, 0xce);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-accessctrl-test-XXXXXX.bin", &rom_path,
                         &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/accessctrl/reset-values", test_reset_values);
    qtest_add_func("/rp2350/accessctrl/password", test_password);
    qtest_add_func("/rp2350/accessctrl/lock", test_lock);
    qtest_add_func("/rp2350/accessctrl/cfgreset", test_cfgreset);
    qtest_add_func("/rp2350/accessctrl/debugger", test_debugger);
    qtest_add_func("/rp2350/accessctrl/gpio-nsmask", test_gpio_nsmask);
    qtest_add_func("/rp2350/accessctrl/narrow", test_narrow);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
