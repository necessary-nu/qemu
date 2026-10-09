/*
 * QTest testcase for the RP2350 RESETS block
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RESETS_BASE 0x40020000
#define RESET       (RESETS_BASE + 0x0)
#define WDSEL       (RESETS_BASE + 0x4)
#define RESET_DONE  (RESETS_BASE + 0x8)
#define XOR         0x1000
#define SET         0x2000
#define CLR         0x3000

#define ALL         0x1fffffff
#define UART0       (1u << 26)
#define TIMER1      (1u << 24)
#define TIMER0      (1u << 23)
#define SPI0        (1u << 18)
#define PADS_QSPI   (1u << 10)
#define IO_QSPI     (1u << 7)
#define IO_BANK0    (1u << 6)

#define RELEASE_NS  250
#define US          1000

#define UART0_BASE  0x40070000
#define UARTIBRD    0x24
#define UARTCR      0x30

#define TIMER0_BASE 0x400b0000
#define ALARM0      0x10
#define ARMED       0x20
#define TIMERAWL    0x28
#define INTR        0x3c
#define INTE        0x40
#define TIMER0_IRQ  0

#define SPI0_BASE   0x40080000

#define TICK_TIMER0_CTRL 0x40108018

#define SRAM_BASE   0x20000000
#define XIP_BASE    0x10000000

static char *rom_path;
static char *image_path;

/* A blank boot ROM is enough: qtest never runs the CPU. */
static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_reset_state(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL);
    g_assert_cmphex(qtest_readl(qts, WDSEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);

    /* Nothing leaves reset on its own. */
    qtest_clock_step(qts, 1000 * US);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_reset_done_tracks_reset(void)
{
    QTestState *qts = start();

    /*
     * The pico-sdk unreset_block_wait() sequence: clear, then poll. The
     * block leaves reset, and RESET_DONE sets, a moment after the clear.
     */
    qtest_writel(qts, RESET + CLR, UART0 | IO_BANK0);
    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL & ~(UART0 | IO_BANK0));
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);
    qtest_clock_step(qts, RELEASE_NS - 1);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, UART0 | IO_BANK0);

    /* Asserting a reset puts the block into reset at once. */
    qtest_writel(qts, RESET + SET, UART0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, IO_BANK0);

    qtest_writel(qts, RESET + XOR, TIMER0 | IO_BANK0);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, TIMER0);

    qtest_writel(qts, RESET, 0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, ALL);

    /* Writes outside the implemented bits are ignored. */
    qtest_writel(qts, RESET, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);

    /* RESET_DONE is read-only. */
    qtest_writel(qts, RESET_DONE, ALL);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);

    /* Aliases read the register itself. */
    g_assert_cmphex(qtest_readl(qts, RESET + SET), ==, ALL);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_reassert_during_release(void)
{
    QTestState *qts = start();

    /* Asserting the reset again before RESET_DONE keeps the block in. */
    qtest_writel(qts, RESET + CLR, UART0);
    qtest_clock_step(qts, RELEASE_NS / 2);
    qtest_writel(qts, RESET + SET, UART0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);

    /* Released again, the delay starts over. */
    qtest_writel(qts, RESET + CLR, UART0);
    qtest_clock_step(qts, RELEASE_NS - 1);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, UART0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_held_registers(void)
{
    QTestState *qts = start();

    /*
     * In reset, a block's registers read as zero and ignore writes: the
     * UART's control register resets to 0x300.
     */
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTCR), ==, 0);
    qtest_writel(qts, UART0_BASE + UARTIBRD, 5);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTIBRD), ==, 0);
    qtest_writeb(qts, UART0_BASE + UARTIBRD, 5);
    g_assert_cmphex(qtest_readb(qts, UART0_BASE + UARTIBRD), ==, 0);
    /* The same holds for blocks without a device model. */
    g_assert_cmphex(qtest_readl(qts, SPI0_BASE), ==, 0);

    /* Still in reset until RESET_DONE sets. */
    qtest_writel(qts, RESET + CLR, UART0);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTCR), ==, 0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTCR), ==, 0x300);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTIBRD), ==, 0);
    qtest_writel(qts, UART0_BASE + UARTIBRD, 5);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTIBRD), ==, 5);

    /* Reset again, the block loses its state. */
    qtest_writel(qts, RESET + SET, UART0);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTIBRD), ==, 0);
    qtest_writel(qts, RESET + CLR, UART0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmphex(qtest_readl(qts, UART0_BASE + UARTIBRD), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.resets/test] */
static void test_timer_stops_in_reset(void)
{
    QTestState *qts = start();
    uint32_t now;

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    qtest_writel(qts, TICK_TIMER0_CTRL, 1);

    /* A timer in reset does not count. */
    qtest_clock_step(qts, 100 * US);
    qtest_writel(qts, RESET + CLR, TIMER0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_BASE + TIMERAWL), ==, 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_BASE + TIMERAWL), ==, 10);

    /* Arm an alarm, then hold the timer in reset past it. */
    now = qtest_readl(qts, TIMER0_BASE + TIMERAWL);
    qtest_writel(qts, TIMER0_BASE + INTE, 1);
    qtest_writel(qts, TIMER0_BASE + ALARM0, now + 20);
    qtest_writel(qts, RESET + SET, TIMER0);
    qtest_clock_step(qts, 100 * US);
    g_assert_false(qtest_get_irq(qts, TIMER0_IRQ));
    g_assert_cmphex(qtest_readl(qts, TIMER0_BASE + TIMERAWL), ==, 0);

    /* Released, it starts again from zero with the alarm disarmed. */
    qtest_writel(qts, RESET + CLR, TIMER0);
    qtest_clock_step(qts, RELEASE_NS);
    g_assert_cmpuint(qtest_readl(qts, TIMER0_BASE + TIMERAWL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TIMER0_BASE + ARMED), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TIMER0_BASE + INTR), ==, 0);
    qtest_clock_step(qts, 100 * US);
    g_assert_false(qtest_get_irq(qts, TIMER0_IRQ));
    g_assert_cmpuint(qtest_readl(qts, TIMER0_BASE + TIMERAWL), ==, 100);
    qtest_quit(qts);
}

/*
 * With no boot ROM executing, the machine starts the image as the ROM
 * leaves it: the ROM's flash boot path has connected the flash, taking the
 * QSPI IO and pads out of reset.
 */
/* [spec:nuos:req:emu.resets/test] */
static void test_direct_boot(void)
{
    QTestState *qts = qtest_initf("-M rp2350,flash-size=4M -kernel %s",
                                  image_path);

    g_assert_cmphex(qtest_readl(qts, RESET), ==, ALL & ~(IO_QSPI | PADS_QSPI));
    g_assert_cmphex(qtest_readl(qts, RESET_DONE), ==, IO_QSPI | PADS_QSPI);
    qtest_quit(qts);
}

static void test_wdsel(void)
{
    QTestState *qts = start();

    qtest_writel(qts, WDSEL, UART0);
    qtest_writel(qts, WDSEL + SET, TIMER0);
    g_assert_cmphex(qtest_readl(qts, WDSEL), ==, UART0 | TIMER0);
    qtest_writel(qts, WDSEL + CLR, UART0);
    g_assert_cmphex(qtest_readl(qts, WDSEL), ==, TIMER0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    const uint32_t image[2] = { SRAM_BASE + 0x1000, XIP_BASE + 0x9 };
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-resets-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    fd = g_file_open_tmp("rp2350-resets-test-XXXXXX.bin", &image_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, image, sizeof(image)), ==, sizeof(image));
    close(fd);

    qtest_add_func("/rp2350/resets/reset-state", test_reset_state);
    qtest_add_func("/rp2350/resets/reset-done", test_reset_done_tracks_reset);
    qtest_add_func("/rp2350/resets/reassert", test_reassert_during_release);
    qtest_add_func("/rp2350/resets/held-registers", test_held_registers);
    qtest_add_func("/rp2350/resets/timer-stops", test_timer_stops_in_reset);
    qtest_add_func("/rp2350/resets/direct-boot", test_direct_boot);
    qtest_add_func("/rp2350/resets/wdsel", test_wdsel);

    ret = g_test_run();
    unlink(rom_path);
    unlink(image_path);
    g_free(rom_path);
    g_free(image_path);
    return ret;
}
