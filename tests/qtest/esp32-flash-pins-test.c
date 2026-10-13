/*
 * QTest testcase for the ESP32-WROOM-32E's flash pins: SPI1 reaches the
 * flash only through pads set up as the hardware needs them
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The module's flash, a GD25Q32 here, is on GPIO6 to GPIO11 (CLK, DO,
 * DI, HOLD#, WP#, CS#). From reset the pads are on the SDIO slave's
 * function 0 and SPI1 does not reach the flash; set up as the ROM sets
 * them up (IO_MUX function 1) or as ESP-IDF's bootloader does (the GPIO
 * matrix), it does (esp32-flash-pins.h). Set up otherwise, each as the
 * board would have it: CLK or CS0 off its function leaves the flash
 * silent and SPI1 reads its inputs' pull-ups; DI off its function gives
 * the flash a command of 1s, which it ignores; DO off its function, its
 * input not routed, reads 0s (the "default value if unassigned" of
 * SPIQ_in); in DIO mode a DO that does not carry SPI1's address bits
 * gives the flash another address; HOLD# held low pauses the flash; WP#
 * held low protects its status register.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32-flash-pins.h"

#define SPI1                    0x3ff42000
#define SPI_CMD                 (SPI1 + 0x00)
#define SPI_ADDR                (SPI1 + 0x04)
#define SPI_CTRL                (SPI1 + 0x08)
#define SPI_RD_STATUS           (SPI1 + 0x10)
#define SPI_USER                (SPI1 + 0x1c)
#define SPI_USER2               (SPI1 + 0x24)
#define SPI_MISO_DLEN           (SPI1 + 0x2c)
#define SPI_W0                  (SPI1 + 0x80)
#define CMD_READ                (1u << 31)
#define CMD_WREN                (1u << 30)
#define CMD_RDID                (1u << 28)
#define CMD_RDSR                (1u << 27)
#define CMD_WRSR                (1u << 26)
#define CTRL_FREAD_DIO          (1u << 23)

#define GPIO_OUT_W1TS           (FP_GPIO + 0x08)
#define GPIO_OUT_W1TC           (FP_GPIO + 0x0c)
#define GPIO_ENABLE_W1TS        (FP_GPIO + 0x24)
#define GPIO_ENABLE_W1TC        (FP_GPIO + 0x28)
#define GPIO_IN                 (FP_GPIO + 0x3c)
#define OUT_INV_SEL             (1u << 9)
#define SIG_GPIO_OUT            256

#define MCU_SEL_MASK            (7u << 12)
#define MCU_SEL(f)              ((f) << 12)
#define FUN_IE                  (1u << 9)

/* The GD25Q32's JEDEC ID, as SPI1 leaves it in W0 */
#define FLASH_ID                0x1640c8
#define FLASH_SIZE              (4 * 1024 * 1024)
#define SR_WEL                  0x02
#define SR_SRWD                 0x80

static char *flash_path;

static uint8_t flash_byte(uint32_t a)
{
    return (uint8_t)(a * 7 + (a >> 9) + 3);
}

static uint32_t flash_word(uint32_t a)
{
    return flash_byte(a) | flash_byte(a + 1) << 8 | flash_byte(a + 2) << 16 |
           (uint32_t)flash_byte(a + 3) << 24;
}

static QTestState *start(void)
{
    return qtest_initf("-M esp32 -nic none "
                       "-drive file=%s,if=mtd,format=raw", flash_path);
}

static uint32_t spi1(QTestState *qts, uint32_t cmd)
{
    qtest_writel(qts, SPI_CMD, cmd);
    return qtest_readl(qts, SPI_W0);
}

static uint32_t rdid(QTestState *qts)
{
    qtest_writel(qts, SPI_W0, 0);
    return spi1(qts, CMD_RDID) & 0xffffff;
}

/* SPI1's READ of four bytes at a, in the read mode CTRL sets */
static uint32_t read_word(QTestState *qts, uint32_t a)
{
    qtest_writel(qts, SPI_ADDR, a | (4u << 24));
    qtest_writel(qts, SPI_MISO_DLEN, 31);
    return spi1(qts, CMD_READ);
}

static void func_sel(QTestState *qts, unsigned pin, unsigned func)
{
    uint32_t a = flash_pin_mux_addr(pin);

    qtest_writel(qts, a, (qtest_readl(qts, a) & ~MCU_SEL_MASK) |
                 MCU_SEL(func));
}

/* The pin's pad as a GPIO driving @level */
static void drive(QTestState *qts, unsigned pin, bool level)
{
    uint32_t bit = 1u << flash_pin[pin].pad;

    qtest_writel(qts, level ? GPIO_OUT_W1TS : GPIO_OUT_W1TC, bit);
    qtest_writel(qts, GPIO_ENABLE_W1TS, bit);
    func_sel(qts, pin, 2);
}

static void release(QTestState *qts, unsigned pin)
{
    qtest_writel(qts, GPIO_ENABLE_W1TC, 1u << flash_pin[pin].pad);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * From reset the flash's pads are on function 0: SPI1 reaches no flash
 * and reads SPIQ_in's default, 0. Set up as the ROM sets them up, or as
 * ESP-IDF's bootloader does through the matrix, it reaches the flash.
 */
static void test_setups(void)
{
    QTestState *qts = start();

    g_assert_cmphex(rdid(qts), ==, 0);
    g_assert_cmphex(read_word(qts, 0x1234), ==, 0);
    flash_pins_rom(qts);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    g_assert_cmphex(read_word(qts, 0x1234), ==, flash_word(0x1234));
    qtest_writel(qts, SPI_CTRL, CTRL_FREAD_DIO);
    g_assert_cmphex(read_word(qts, 0x5678), ==, flash_word(0x5678));
    qtest_quit(qts);

    qts = start();
    flash_pins_idf(qts);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    g_assert_cmphex(read_word(qts, 0x1234), ==, flash_word(0x1234));
    qtest_writel(qts, SPI_CTRL, CTRL_FREAD_DIO);
    g_assert_cmphex(read_word(qts, 0x5678), ==, flash_word(0x5678));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * CLK off its function: the flash has no clock and stays silent, and SPI1
 * reads DO's pull-up, 1s. Through the matrix on the GPIO function SPICLK
 * reaches it again.
 */
static void test_clk(void)
{
    QTestState *qts = start();

    flash_pins_rom(qts);
    func_sel(qts, FLASH_PIN_CLK, 2);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    g_assert_cmphex(read_word(qts, 0x40), ==, 0xffffffff);
    qtest_writel(qts, FP_FUNC_OUT(flash_pin[FLASH_PIN_CLK].pad), 0);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_writel(qts, FP_FUNC_OUT(flash_pin[FLASH_PIN_CLK].pad),
                 SIG_GPIO_OUT);
    func_sel(qts, FLASH_PIN_CLK, 1);
    g_assert_cmphex(read_word(qts, 0x40), ==, flash_word(0x40));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * CS0 off its function: pulled up, the flash is not selected; held low as
 * a GPIO it is selected, through the transaction and after; inverted on
 * its way through the matrix, it is selected only between transactions,
 * which give it no clock: it takes nothing.
 */
static void test_cs(void)
{
    QTestState *qts = start();
    uint32_t pad = flash_pin[FLASH_PIN_CS].pad;

    flash_pins_rom(qts);
    func_sel(qts, FLASH_PIN_CS, 2);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    drive(qts, FLASH_PIN_CS, false);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_writel(qts, GPIO_OUT_W1TS, 1u << pad);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    release(qts, FLASH_PIN_CS);

    qtest_writel(qts, FP_FUNC_OUT(pad), FP_SIG_SPICS0 | OUT_INV_SEL);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    qtest_writel(qts, FP_FUNC_OUT(pad), FP_SIG_SPICS0);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * DO off its function: SPIQ_in, routed from no pad, reads 0s; its input
 * disabled on function 1 likewise. Routed through the matrix from the pad
 * the flash's DO reaches SPI1 again. In DIO mode DO carries half the
 * address too: off its function, its address bits are the pad's pull-up,
 * so the flash reads at another address, ORed with 0xaaaaaa, wrapping at
 * its size; with SPIQ_in unrouted the data's odd bits are 0.
 */
static void test_q(void)
{
    QTestState *qts = start();
    const uint32_t a = 0x10404, far = (a | 0xaaaaaa) % FLASH_SIZE;
    uint32_t pad = flash_pin[FLASH_PIN_Q].pad;

    flash_pins_rom(qts);
    func_sel(qts, FLASH_PIN_Q, 2);
    g_assert_cmphex(rdid(qts), ==, 0);
    g_assert_cmphex(read_word(qts, a), ==, 0);

    qtest_writel(qts, SPI_CTRL, CTRL_FREAD_DIO);
    g_assert_cmphex(read_word(qts, a), ==, flash_word(far) & 0x55555555);
    qtest_writel(qts, FP_FUNC_IN(FP_SIG_SPIQ), FP_IN_SIG_IN_SEL | pad);
    g_assert_cmphex(read_word(qts, a), ==, flash_word(far));
    qtest_writel(qts, FP_FUNC_OUT(pad), FP_SIG_SPIQ);
    g_assert_cmphex(read_word(qts, a), ==, flash_word(a));
    qtest_writel(qts, SPI_CTRL, 0);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);

    qtest_writel(qts, FP_FUNC_IN(FP_SIG_SPIQ), 0);
    func_sel(qts, FLASH_PIN_Q, 1);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_writel(qts, flash_pin_mux_addr(FLASH_PIN_Q),
                 qtest_readl(qts, flash_pin_mux_addr(FLASH_PIN_Q)) & ~FUN_IE);
    g_assert_cmphex(rdid(qts), ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * DI off its function: the flash takes the command as the pad's pull-up
 * holds DI, 1s, and stays silent; SPI1 reads DO's pull-up. Through the
 * matrix SPID reaches it again; inverted on the way, the flash takes RDID
 * (9Fh) as 60h, a chip erase it refuses without a write enable.
 */
static void test_d(void)
{
    QTestState *qts = start();
    uint32_t pad = flash_pin[FLASH_PIN_D].pad;

    flash_pins_rom(qts);
    func_sel(qts, FLASH_PIN_D, 2);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    g_assert_cmphex(read_word(qts, 0x100), ==, 0xffffffff);
    qtest_writel(qts, FP_FUNC_OUT(pad), FP_SIG_SPID);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    qtest_writel(qts, FP_FUNC_OUT(pad), FP_SIG_SPID | OUT_INV_SEL);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    qtest_writel(qts, FP_FUNC_OUT(pad), FP_SIG_SPID);
    g_assert_cmphex(read_word(qts, 0x100), ==, flash_word(0x100));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * [spec:nuos:req:emu.flash-chip/test]
 * HOLD#: the ROM leaves HD on function 0, pulled up. Held low as a GPIO
 * it pauses the flash (its quad enable clear): silent, its DO off.
 */
static void test_hold(void)
{
    QTestState *qts = start();

    flash_pins_rom(qts);
    drive(qts, FLASH_PIN_HD, true);
    g_assert_cmphex(rdid(qts), ==, FLASH_ID);
    drive(qts, FLASH_PIN_HD, false);
    g_assert_cmphex(rdid(qts), ==, 0xffffff);
    g_assert_cmphex(read_word(qts, 0x200), ==, 0xffffffff);
    release(qts, FLASH_PIN_HD);
    g_assert_cmphex(read_word(qts, 0x200), ==, flash_word(0x200));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * [spec:nuos:req:emu.flash-chip/test]
 * WP#: the status register's SRWD set, WP# held low protects the status
 * register from WRSR; pulled up again, WRSR takes.
 */
static void test_wp(void)
{
    QTestState *qts = start();

    flash_pins_rom(qts);
    spi1(qts, CMD_WREN);
    qtest_writel(qts, SPI_RD_STATUS, SR_SRWD);
    spi1(qts, CMD_WRSR);
    spi1(qts, CMD_RDSR);
    g_assert_cmphex(qtest_readl(qts, SPI_RD_STATUS) & 0xff, ==, SR_SRWD);

    drive(qts, FLASH_PIN_WP, false);
    spi1(qts, CMD_WREN);
    qtest_writel(qts, SPI_RD_STATUS, 0);
    spi1(qts, CMD_WRSR);
    spi1(qts, CMD_RDSR);
    g_assert_cmphex(qtest_readl(qts, SPI_RD_STATUS) & 0xff, ==,
                    SR_SRWD | SR_WEL);

    release(qts, FLASH_PIN_WP);
    qtest_writel(qts, SPI_RD_STATUS, 0);
    spi1(qts, CMD_WRSR);
    spi1(qts, CMD_RDSR);
    g_assert_cmphex(qtest_readl(qts, SPI_RD_STATUS) & 0xff, ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * Between transactions SPI1 holds the shared lines as the pads show them:
 * CLK low, CS0 high, DO released to its pull-up, WP high.
 */
static void test_idle_lines(void)
{
    QTestState *qts = start();
    uint32_t in;

    flash_pins_rom(qts);
    in = qtest_readl(qts, GPIO_IN);
    g_assert_cmphex(in & (1u << 6), ==, 0);
    g_assert_cmphex(in & (1u << 7), ==, 1u << 7);
    g_assert_cmphex(in & (1u << 11), ==, 1u << 11);
    func_sel(qts, FLASH_PIN_WP, 1);
    g_assert_cmphex(qtest_readl(qts, GPIO_IN) & (1u << 10), ==, 1u << 10);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_autofree uint8_t *data = g_malloc(FLASH_SIZE);
    int fd, ret;

    for (uint32_t a = 0; a < FLASH_SIZE; a++) {
        data[a] = flash_byte(a);
    }
    fd = g_file_open_tmp("esp32-flash-pins-XXXXXX", &flash_path, NULL);
    g_assert(fd >= 0);
    g_assert_cmpint(write(fd, data, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);

    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/flash-pins/setups", test_setups);
    qtest_add_func("/esp32/flash-pins/clk", test_clk);
    qtest_add_func("/esp32/flash-pins/cs", test_cs);
    qtest_add_func("/esp32/flash-pins/q", test_q);
    qtest_add_func("/esp32/flash-pins/d", test_d);
    qtest_add_func("/esp32/flash-pins/hold", test_hold);
    qtest_add_func("/esp32/flash-pins/wp", test_wp);
    qtest_add_func("/esp32/flash-pins/idle-lines", test_idle_lines);
    ret = g_test_run();
    unlink(flash_path);
    g_free(flash_path);
    return ret;
}
