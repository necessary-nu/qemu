/*
 * QTest testcase for the ESP32's flash protocol: SPI1's and SPI0's
 * commands as they cross the lines, clock by clock, to the
 * ESP32-WROOM-32E's GD25Q32C, and the time they take
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Set up as ESP-IDF sets them up, each read mode reads the flash; with
 * the wrong dummy clocks, or an input through the GPIO matrix without the
 * extra dummy clocks that make up for its delay, the data comes shifted
 * as on the board. The quad reads need the flash's QE, which 31h sets
 * (01h writes S7-S0 only); with QE clear IO3 is HOLD#, with it set a data
 * line. A transaction takes its SPI clocks at APB_CLK divided as
 * SPI_CLOCK gives; the flash's erase and program take its datasheet's
 * typical times, its reads ignored meanwhile. GPIO_SPI_SWAP moves SPI0
 * and SPI1's IO_MUX connections to HSPI's pads, leaving the module's flash
 * to HSPI's.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32-flash-pins.h"
#include "esp32-spi-flash.h"

#define FLASH_ID                0x1640c8
#define FLASH_SIZE              (4 * 1024 * 1024)

#define GPIO_OUT_W1TS           (FP_GPIO + 0x08)
#define GPIO_OUT_W1TC           (FP_GPIO + 0x0c)
#define GPIO_ENABLE_W1TS        (FP_GPIO + 0x24)
#define GPIO_ENABLE_W1TC        (FP_GPIO + 0x28)
#define GPIO_IN                 (FP_GPIO + 0x3c)
#define GPIO_SIGMADELTA_MISC    (0x3ff44f00 + 0x24)
#define GPIO_SPI_SWAP           (1u << 31)

#define MCU_SEL_MASK            (7u << 12)
#define MCU_SEL(f)              ((f) << 12)

#define SR2_QE                  0x02

/* The ESP32's SPI clock at reset: APB_CLK, the 40 MHz crystal's */
#define APB_NS                  25

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

/*
 * The four bytes at a as a controller reads them @n bits late: n 1s (the
 * pull-ups while the flash does not drive yet), then the flash's bits;
 * @n negative, the first -n of its bits missed.
 */
static uint32_t shifted_word(uint32_t a, int n)
{
    uint64_t bits = (uint64_t)flash_byte(a) << 56 |
                    (uint64_t)flash_byte(a + 1) << 48 |
                    (uint64_t)flash_byte(a + 2) << 40 |
                    (uint64_t)flash_byte(a + 3) << 32 |
                    (uint64_t)flash_byte(a + 4) << 24;
    uint32_t v;

    if (n >= 0) {
        bits = bits >> n | ~(~0ull >> n);
    } else {
        bits <<= -n;
    }
    v = bits >> 32;
    return (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) |
           (v << 24);
}

static QTestState *start(void)
{
    return qtest_initf("-M esp32 -nic none "
                       "-drive file=%s,if=mtd,format=raw", flash_path);
}

static void func_sel(QTestState *qts, unsigned pin, unsigned func)
{
    uint32_t a = flash_pin_mux_addr(pin);

    qtest_writel(qts, a, (qtest_readl(qts, a) & ~MCU_SEL_MASK) |
                 MCU_SEL(func));
}

/* All six on function 1, as a quad mode needs HD and WP too */
static void flash_pins_quad(QTestState *qts)
{
    flash_pins_rom(qts);
    func_sel(qts, FLASH_PIN_HD, 1);
    func_sel(qts, FLASH_PIN_WP, 1);
}

static void set_qe(QTestState *qts, bool on)
{
    sf_usr(qts, 0x06, 0, 0, 0);
    sf_usr(qts, 0x31, on ? SR2_QE : 0, 8, 0);
    sf_wait_idle(qts);
    g_assert_cmphex(sf_usr(qts, 0x35, 0, 0, 8) & SR2_QE, ==, on ? SR2_QE : 0);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * [spec:nuos:req:emu.flash-chip/test]
 * Each read mode, configured as ESP-IDF configures it, sends its command
 * (03h, 0Bh, 3Bh, BBh) with its address, mode and dummy clocks, and reads
 * the flash.
 */
static void test_read_modes(void)
{
    QTestState *qts = start();
    static const int modes[] = { SF_SLOW, SF_FAST, SF_DOUT, SF_DIO };

    flash_pins_rom(qts);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, FLASH_ID);
    for (int i = 0; i < ARRAY_SIZE(modes); i++) {
        sf_read_mode(qts, modes[i], 0);
        g_assert_cmphex(sf_read_word(qts, 0x1234 + i), ==,
                        flash_word(0x1234 + i));
    }
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * [spec:nuos:req:emu.flash-chip/test]
 * The wrong dummy clocks shift the data as on the board: 0Bh with none
 * reads the flash's dummy byte's undriven DO first (its pull-up, 1s);
 * with four, half of it; BBh with a 24-bit address and no dummy clocks
 * gives the flash its mode bits from the released lines (1s) and reads
 * four clocks, eight bits, early; with two dummy clocks too many it misses
 * the first four bits.
 */
static void test_dummy_shift(void)
{
    QTestState *qts = start();
    const uint32_t a = 0x2468;

    flash_pins_rom(qts);
    sf_read_mode(qts, SF_FAST, 0);
    qtest_writel(qts, SF_SPI1 + SF_USER, SF_USER_COMMAND | SF_USER_ADDR |
                 SF_USER_MISO);
    g_assert_cmphex(sf_read_word(qts, a), ==, shifted_word(a, 8));
    qtest_writel(qts, SF_SPI1 + SF_USER, SF_USER_COMMAND | SF_USER_ADDR |
                 SF_USER_MISO | SF_USER_DUMMY);
    qtest_writel(qts, SF_SPI1 + SF_USER1, SF_USER1_VAL(24, 4));
    g_assert_cmphex(sf_read_word(qts, a), ==, shifted_word(a, 4));

    sf_read_mode(qts, SF_DIO, 0);
    qtest_writel(qts, SF_SPI1 + SF_USER1, SF_USER1_VAL(24, 1));
    g_assert_cmphex(sf_read_word(qts, a), ==, shifted_word(a, 8));
    qtest_writel(qts, SF_SPI1 + SF_USER, SF_USER_COMMAND | SF_USER_ADDR |
                 SF_USER_MISO | SF_USER_DUMMY);
    qtest_writel(qts, SF_SPI1 + SF_USER1, SF_USER1_VAL(32, 2));
    g_assert_cmphex(sf_read_word(qts, a), ==, shifted_word(a, -4));
    /* The ROM's own DIO setup: 24 bits, four dummy clocks */
    qtest_writel(qts, SF_SPI1 + SF_USER1, SF_USER1_VAL(24, 4));
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * [spec:nuos:req:emu.flash-chip/test]
 * The quad reads need QE: with it clear the flash ignores 6Bh and EBh and
 * SPI1 reads the four lines' pull-ups. 01h writes S7-S0 alone; two bytes
 * of it are not executed. 31h sets QE, and then 6Bh and EBh read the
 * flash.
 */
static void test_quad_needs_qe(void)
{
    QTestState *qts = start();
    const uint32_t a = 0x5678;

    flash_pins_quad(qts);
    sf_read_mode(qts, SF_QOUT, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, 0xffffffff);
    sf_read_mode(qts, SF_QIO, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, 0xffffffff);

    /* A two-byte 01h, as the ROM's WRSR_2B sends it: not executed */
    qtest_writel(qts, SF_SPI1 + SF_CTRL, SF_CTRL_WP | SF_CTRL_WRSR_2B);
    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_RD_STATUS, SR2_QE << 8 | 0x04);
    sf_cmd(qts, SF_CMD_WRSR);
    g_assert_cmphex(sf_rdsr(qts) & 0xfd, ==, 0);
    g_assert_cmphex(sf_usr(qts, 0x35, 0, 0, 8), ==, 0);
    /* One byte: SR1 */
    qtest_writel(qts, SF_SPI1 + SF_CTRL, SF_CTRL_WP);
    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_RD_STATUS, 0x04);
    sf_cmd(qts, SF_CMD_WRSR);
    sf_wait_idle(qts);
    g_assert_cmphex(sf_rdsr(qts), ==, 0x04);
    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_RD_STATUS, 0);
    sf_cmd(qts, SF_CMD_WRSR);
    sf_wait_idle(qts);

    set_qe(qts, true);
    sf_read_mode(qts, SF_QOUT, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));
    sf_read_mode(qts, SF_QIO, 0);
    g_assert_cmphex(sf_read_word(qts, a + 1), ==, flash_word(a + 1));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * [spec:nuos:req:emu.flash-chip/test]
 * IO3 is HOLD# while QE is clear: held low it pauses the flash. With QE
 * set it is a data line, and a low pad does not pause the flash.
 */
static void test_hold_is_io3(void)
{
    QTestState *qts = start();
    uint32_t hd = 1u << flash_pin[FLASH_PIN_HD].pad;

    flash_pins_rom(qts);
    qtest_writel(qts, GPIO_OUT_W1TC, hd);
    qtest_writel(qts, GPIO_ENABLE_W1TS, hd);
    func_sel(qts, FLASH_PIN_HD, 2);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, 0xffffff);
    qtest_writel(qts, GPIO_ENABLE_W1TC, hd);
    set_qe(qts, true);
    qtest_writel(qts, GPIO_ENABLE_W1TS, hd);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, FLASH_ID);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * An input through the GPIO matrix is two APB_CLK cycles late: at
 * APB_CLK / 2 one SPI clock, at APB_CLK two. Without the extra dummy
 * clocks the controller reads the data that many bits late, as on the
 * board; with them it reads the flash. Through IO_MUX it has no delay.
 */
static void test_matrix_delay(void)
{
    QTestState *qts = start();
    const uint32_t a = 0x9abc;

    flash_pins_idf(qts);
    qtest_writel(qts, SF_SPI1 + SF_CLOCK, 0x1001);
    sf_read_mode(qts, SF_SLOW, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, shifted_word(a, 1));
    sf_read_mode(qts, SF_SLOW, 1);
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));
    sf_read_mode(qts, SF_DIO, 1);
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));

    qtest_writel(qts, SF_SPI1 + SF_CLOCK, 1u << 31);
    sf_read_mode(qts, SF_FAST, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, shifted_word(a, 2));
    sf_read_mode(qts, SF_FAST, 2);
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));

    /* At APB_CLK / 4 the delay is under a clock */
    qtest_writel(qts, SF_SPI1 + SF_CLOCK, 0x3043);
    sf_read_mode(qts, SF_SLOW, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));
    qtest_quit(qts);

    qts = start();
    flash_pins_rom(qts);
    qtest_writel(qts, SF_SPI1 + SF_CLOCK, 1u << 31);
    sf_read_mode(qts, SF_FAST, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, flash_word(a));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating/test]
 * A transaction takes its SPI clocks: RDID's 32 at APB_CLK (40 MHz from
 * reset), its bit set until then; at APB_CLK / 4, four times as long, and
 * CS_SETUP and CS_HOLD add SETUP_TIME + 1.5 and HOLD_TIME clocks.
 */
static void test_timing(void)
{
    QTestState *qts = start();

    flash_pins_rom(qts);
    qtest_writel(qts, SF_SPI1 + SF_W0, 0);
    qtest_writel(qts, SF_SPI1 + SF_CMD, SF_CMD_RDID);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, SF_CMD_RDID);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_EXT2) & 7, !=, 0);
    qtest_clock_step(qts, 32 * APB_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, SF_CMD_RDID);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_W0), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_EXT2) & 7, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_W0) & 0xffffff, ==,
                    FLASH_ID);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_SLAVE) & (1u << 4), !=, 0);

    qtest_writel(qts, SF_SPI1 + SF_CLOCK, 0x3043);
    qtest_writel(qts, SF_SPI1 + SF_CMD, SF_CMD_RDID);
    qtest_clock_step(qts, 4 * 32 * APB_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, SF_CMD_RDID);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, 0);

    /* SETUP_TIME 1 and HOLD_TIME 1 from reset: 2.5 + 1 more clocks */
    qtest_writel(qts, SF_SPI1 + SF_USER,
                 qtest_readl(qts, SF_SPI1 + SF_USER) | SF_USER_CS_SETUP |
                 SF_USER_CS_HOLD);
    qtest_writel(qts, SF_SPI1 + SF_CMD, SF_CMD_RDID);
    qtest_clock_step(qts, 4 * APB_NS * 35 + 2 * APB_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, SF_CMD_RDID);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_CMD), ==, 0);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.flash-timing/test]
 * The GD25Q32C's sector erase takes its typical 50 ms, page program
 * min(tPP, tBP1 + (n - 1) tBP2): 37.5 us for four bytes. While busy it
 * ignores reads: SPI1 reads DO's pull-up.
 */
static void test_erase_program_timing(void)
{
    QTestState *qts = start();
    const uint32_t a = 0x30000;

    flash_pins_rom(qts);
    sf_read_mode(qts, SF_SLOW, 0);
    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, a);
    sf_cmd(qts, SF_CMD_SE);
    g_assert_cmphex(sf_rdsr(qts), ==, SF_SR1_WIP);
    g_assert_cmphex(sf_read_word(qts, a), ==, 0xffffffff);
    qtest_clock_step(qts, 49 * 1000 * 1000);
    g_assert_cmphex(sf_rdsr(qts), ==, SF_SR1_WIP);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(sf_rdsr(qts), ==, 0);
    g_assert_cmphex(sf_read_word(qts, a + 4), ==, 0xffffffff);

    sf_cmd(qts, SF_CMD_WREN);
    qtest_writel(qts, SF_SPI1 + SF_W0, 0x12345678);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, a | (4u << 24));
    sf_cmd(qts, SF_CMD_PP);
    g_assert_cmphex(sf_rdsr(qts), ==, SF_SR1_WIP);
    qtest_clock_step(qts, 36000);
    g_assert_cmphex(sf_rdsr(qts), ==, SF_SR1_WIP);
    qtest_clock_step(qts, 2000);
    g_assert_cmphex(sf_rdsr(qts), ==, 0);
    g_assert_cmphex(sf_read_word(qts, a), ==, 0x12345678);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.flash-chip/test]
 * The GD25Q32C's SFDP table (5Ah), its high performance mode (A3h, HPF in
 * S20, which ABh clears) and its 128-bit unique ID's length.
 */
static void test_part(void)
{
    QTestState *qts = start();
    uint32_t user = SF_USER_COMMAND | SF_USER_ADDR | SF_USER_DUMMY |
                    SF_USER_MISO;

    flash_pins_rom(qts);
    qtest_writel(qts, SF_SPI1 + SF_CTRL, SF_CTRL_WP);
    qtest_writel(qts, SF_SPI1 + SF_USER, user);
    qtest_writel(qts, SF_SPI1 + SF_USER1, SF_USER1_VAL(24, 8));
    qtest_writel(qts, SF_SPI1 + SF_USER2, (7u << 28) | 0x5a);
    qtest_writel(qts, SF_SPI1 + SF_MISO_DLEN, 31);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, 0);
    sf_cmd(qts, SF_CMD_USR);
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_W0), ==, 0x50444653);
    qtest_writel(qts, SF_SPI1 + SF_ADDR, 0x60u << 8);
    sf_cmd(qts, SF_CMD_USR);
    /* VCC 3.6 V maximum, 2.7 V minimum */
    g_assert_cmphex(qtest_readl(qts, SF_SPI1 + SF_W0), ==, 0x27003600);

    sf_cmd(qts, SF_CMD_HPM);
    g_assert_cmphex(sf_usr(qts, 0x15, 0, 0, 8), ==, 0x30);
    qtest_writel(qts, SF_SPI1 + SF_CTRL, SF_CTRL_WP);
    sf_cmd(qts, SF_CMD_RES);
    g_assert_cmphex(sf_usr(qts, 0x15, 0, 0, 8), ==, 0x20);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * GPIO_SPI_SWAP, as the ROM sets it to boot from a flash on HSPI's pads,
 * moves SPI0 and SPI1's IO_MUX function-1 connections to GPIO14, 12, 13,
 * 4, 2 and 15: the module's flash, on GPIO6 to GPIO11, is cut off them.
 * SPIQ_in, which no pad's function 1 then carries, reads its default, 0,
 * and taken from MTDI's function 1, driven high from outside, 1s.
 */
static void test_spi_swap(void)
{
    QTestState *qts = start();
    uint32_t mtdi = FP_IO_MUX + 0x34;

    flash_pins_rom(qts);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, FLASH_ID);
    qtest_writel(qts, GPIO_SIGMADELTA_MISC, GPIO_SPI_SWAP);
    g_assert_cmphex(qtest_readl(qts, GPIO_SIGMADELTA_MISC), ==,
                    GPIO_SPI_SWAP);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, 0);
    qtest_writel(qts, mtdi, MCU_SEL(1) | (1u << 9));
    qtest_set_irq_in(qts, "/machine/soc/gpio", "esp32-gpio-pad-in", 12, 1);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, 0xffffff);
    qtest_writel(qts, GPIO_SIGMADELTA_MISC, 0);
    g_assert_cmphex(sf_usr(qts, 0x9f, 0, 0, 24), ==, FLASH_ID);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_autofree uint8_t *data = g_malloc(FLASH_SIZE);
    int fd, ret;

    for (uint32_t a = 0; a < FLASH_SIZE; a++) {
        data[a] = flash_byte(a);
    }
    fd = g_file_open_tmp("esp32-flash-protocol-XXXXXX", &flash_path, NULL);
    g_assert(fd >= 0);
    g_assert_cmpint(write(fd, data, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);

    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/flash-protocol/read-modes", test_read_modes);
    qtest_add_func("/esp32/flash-protocol/dummy-shift", test_dummy_shift);
    qtest_add_func("/esp32/flash-protocol/quad-needs-qe", test_quad_needs_qe);
    qtest_add_func("/esp32/flash-protocol/hold-is-io3", test_hold_is_io3);
    qtest_add_func("/esp32/flash-protocol/matrix-delay", test_matrix_delay);
    qtest_add_func("/esp32/flash-protocol/timing", test_timing);
    qtest_add_func("/esp32/flash-protocol/erase-program-timing",
                   test_erase_program_timing);
    qtest_add_func("/esp32/flash-protocol/part", test_part);
    qtest_add_func("/esp32/flash-protocol/spi-swap", test_spi_swap);
    ret = g_test_run();
    unlink(flash_path);
    g_free(flash_path);
    return ret;
}
