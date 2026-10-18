/*
 * QTest testcase for the ESP32-P4 MSPI controllers' SPI1
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * SPI1's commands to the W25Q128 flash and the PSRAM die, timed on the
 * virtual clock: from reset the flash controller's core clock is the
 * 40 MHz crystal divided by 4 (HP_SYS_CLKRST_PERI_CLK_CTRL00), and SPI1's
 * SPI clock that divided by CLOCK_CLKCNT_N + 1 = 4, 2.5 MHz.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32p4-psram.h"

#define SPI1_FLASH          0x5008d000
#define SPI1_PSRAM          0x5008f000
#define SPI0_FLASH          0x5008c000
#define SPI0_PSRAM          0x5008e000
#define FLASH_DIRECT        0x80000000
#define PSRAM_DIRECT        0x88000000
#define PAGE                0x10000

#define CMD                 0x000
#define ADDR                0x004
#define CTRL                0x008
#define USER                0x018
#define USER1               0x01c
#define USER2               0x020
#define MOSI_DLEN           0x024
#define MISO_DLEN           0x028
#define RD_STATUS           0x02c
#define MISC                0x034
#define CACHE_FCTRL         0x03c
#define W(n)                (0x058 + 4 * (n))
#define INT_ENA             0x0c0
#define INT_CLR             0x0c4
#define INT_RAW             0x0c8
#define INT_ST              0x0cc
#define DDR                 0x0d4
#define SMEM_DDR            0x0d8
#define DDR_EN              (1u << 0)
#define DDR_VAR_DUMMY       (1u << 1)
#define DQS_LOOP            (1u << 21)
#define XTS_PLAIN(n)        (0x300 + 4 * (n))
#define XTS_LINESIZE        0x340
#define XTS_DESTINATION     0x344
#define XTS_PHYSICAL_ADDRESS 0x348
#define XTS_TRIGGER         0x34c
#define XTS_RELEASE         0x350
#define XTS_DESTROY         0x354
#define XTS_STATE           0x358
#define MMU_ITEM_CONTENT    0x37c
#define MMU_ITEM_INDEX      0x380

#define CMD_USR             (1u << 18)
#define CMD_SE              (1u << 24)
#define CMD_PP              (1u << 25)
#define CMD_RDSR            (1u << 27)
#define CMD_RDID            (1u << 28)
#define CMD_WREN            (1u << 30)
#define USR_MOSI            (1u << 27)
#define USR_MISO            (1u << 28)
#define USR_DUMMY           (1u << 29)
#define USR_ADDR            (1u << 30)
#define USR_COMMAND         (1u << 31)
#define CLOSE_AXI_INF_EN    (1u << 31)
#define INT_WPE_END         (1u << 2)
#define INT_SLV_ST_END      (1u << 3)
#define INT_MST_ST_END      (1u << 4)
#define FLASH_VALID         (1u << 12)
#define PSRAM_ACCESS        (1u << 10)
#define PSRAM_VALID         (1u << 11)

#define SPI_CYCLE_NS        400

#define INTMTX_STATUS(w)    (0x500d6200 + 4 * (w))
#define SOURCE_MSPI         84
#define SOURCE_PSRAM_MSPI   109

#define HP_RST_EN0          0x500e60c0
#define RST_MSPI_APB        (1u << 24)

#define HP_SYSTEM_CRYPTO_CTRL   0x500e5070
#define ENABLE_SPI_MANUAL       (1u << 0)

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

static bool source(QTestState *qts, int n)
{
    return qtest_readl(qts, INTMTX_STATUS(n / 32)) & (1u << (n % 32));
}

/* Start an SPI1 command and run the clock until it ends. */
static void spi1_run(QTestState *qts, uint32_t spi1, uint32_t cmd)
{
    int i;

    qtest_writel(qts, spi1 + CMD, cmd);
    for (i = 0; i < 1000 && qtest_readl(qts, spi1 + CMD); i++) {
        qtest_clock_step(qts, 10000);
    }
    g_assert_cmphex(qtest_readl(qts, spi1 + CMD), ==, 0);
}

/*
 * A user-defined command: `cmd_bits` of command, `addr_bits` of address,
 * `dummy` cycles, `out` written from W0 and `in_len` bytes read into it.
 */
static void spi1_user(QTestState *qts, uint32_t spi1, uint32_t cmd,
                      int cmd_bits, uint32_t addr, int addr_bits, int dummy,
                      int out_len, int in_len)
{
    uint32_t user = USR_COMMAND;
    uint32_t user1 = 0;

    if (addr_bits) {
        user |= USR_ADDR;
        user1 |= (uint32_t)(addr_bits - 1) << 26;
    }
    if (dummy) {
        user |= USR_DUMMY;
        user1 |= dummy - 1;
    }
    if (out_len) {
        user |= USR_MOSI;
        qtest_writel(qts, spi1 + MOSI_DLEN, out_len * 8 - 1);
    }
    if (in_len) {
        user |= USR_MISO;
        qtest_writel(qts, spi1 + MISO_DLEN, in_len * 8 - 1);
    }
    qtest_writel(qts, spi1 + USER, user);
    qtest_writel(qts, spi1 + USER1, user1);
    qtest_writel(qts, spi1 + USER2, ((uint32_t)(cmd_bits - 1) << 28) | cmd);
    qtest_writel(qts, spi1 + ADDR, addr);
    spi1_run(qts, spi1, CMD_USR);
}

static void flash_wren(QTestState *qts)
{
    spi1_run(qts, SPI1_FLASH, CMD_WREN);
}

static void flash_erase(QTestState *qts, uint32_t addr)
{
    flash_wren(qts);
    qtest_writel(qts, SPI1_FLASH + USER1, 23u << 26);
    qtest_writel(qts, SPI1_FLASH + ADDR, addr);
    spi1_run(qts, SPI1_FLASH, CMD_SE);
}

static void flash_program(QTestState *qts, uint32_t addr, const uint32_t *w,
                          int words)
{
    flash_wren(qts);
    for (int i = 0; i < words; i++) {
        qtest_writel(qts, SPI1_FLASH + W(i), w[i]);
    }
    qtest_writel(qts, SPI1_FLASH + USER1, 23u << 26);
    qtest_writel(qts, SPI1_FLASH + ADDR, addr | (words * 4u) << 24);
    spi1_run(qts, SPI1_FLASH, CMD_PP);
}

/*
 * SPI0's flash reads set up as the boot ROM leaves them (esp32p4_boot.c):
 * DIO, command BBh from USER2, 28 address bits (the address and the mode
 * bits' upper half, WB_MODE's 0) and 2 dummy cycles, the first driven
 * high. Out of reset SPI0 sends FAST_READ (0Bh) with no dummy phase, which
 * reads the flash a byte late, as the chip would.
 */
static void spi0_rom_read_setup(QTestState *qts)
{
    qtest_writel(qts, SPI0_FLASH + 0x008, 0x00ac200c);
    qtest_writel(qts, SPI0_FLASH + 0x018, 0x200000c0);
    qtest_writel(qts, SPI0_FLASH + 0x01c, 0x6c000041);
    qtest_writel(qts, SPI0_FLASH + 0x020, 0x700000bb);
    qtest_writel(qts, SPI0_FLASH + 0x02c, 0);
    qtest_writel(qts, SPI0_FLASH + CACHE_FCTRL,
                 qtest_readl(qts, SPI0_FLASH + CACHE_FCTRL) | 0x5);
}

static void map_flash(QTestState *qts, uint32_t entry, uint32_t page)
{
    spi0_rom_read_setup(qts);
    qtest_writel(qts, SPI0_FLASH + CACHE_FCTRL,
                 qtest_readl(qts, SPI0_FLASH + CACHE_FCTRL) &
                 ~CLOSE_AXI_INF_EN);
    qtest_writel(qts, SPI0_FLASH + MMU_ITEM_INDEX, entry);
    qtest_writel(qts, SPI0_FLASH + MMU_ITEM_CONTENT, FLASH_VALID | page);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_rdid_timing(void)
{
    QTestState *qts = start();

    /* RDID: 8 command and 24 data cycles. */
    qtest_writel(qts, SPI1_FLASH + INT_CLR, 0x41f);
    qtest_writel(qts, SPI1_FLASH + CMD, CMD_RDID);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, CMD_RDID);
    qtest_clock_step(qts, 32 * SPI_CYCLE_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, CMD_RDID);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + INT_RAW), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + W(0)) & 0xffffff, ==,
                    0x1840ef);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + INT_RAW), ==,
                    INT_SLV_ST_END | INT_MST_ST_END);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_program_erase(void)
{
    QTestState *qts = start();
    static const uint32_t data[4] = {
        0x01234567, 0x89abcdef, 0xfedcba98, 0x76543210,
    };

    flash_wren(qts);
    spi1_run(qts, SPI1_FLASH, CMD_RDSR);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + RD_STATUS) & 0xff, ==, 2);

    /*
     * A program ends after its own cycles and FLASH_WAITI_CTRL's status
     * read (16 cycles), with WPE_END, which the MSPI source carries.
     */
    qtest_writel(qts, SPI1_FLASH + INT_CLR, 0x41f);
    qtest_writel(qts, SPI1_FLASH + INT_ENA, INT_WPE_END);
    g_assert_false(source(qts, SOURCE_MSPI));
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, SPI1_FLASH + W(i), data[i]);
    }
    qtest_writel(qts, SPI1_FLASH + USER1, 23u << 26);
    qtest_writel(qts, SPI1_FLASH + ADDR, 0x30100 | 16u << 24);
    qtest_writel(qts, SPI1_FLASH + CMD, CMD_PP);
    qtest_clock_step(qts, (8 + 24 + 16 * 8 + 16) * SPI_CYCLE_NS - 1);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, CMD_PP);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + INT_ST), ==, INT_WPE_END);
    g_assert_true(source(qts, SOURCE_MSPI));
    qtest_writel(qts, SPI1_FLASH + INT_CLR, INT_WPE_END);
    g_assert_false(source(qts, SOURCE_MSPI));
    qtest_writel(qts, SPI1_FLASH + INT_ENA, 0);

    /* Seen through SPI0's MMU, and by a user-defined read. */
    map_flash(qts, 0, 3);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 0x100 + 4 * i), ==,
                        data[i]);
    }
    spi1_user(qts, SPI1_FLASH, 0x0b, 8, 0x30104, 24, 8, 0, 8);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + W(0)), ==, data[1]);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + W(1)), ==, data[2]);

    flash_erase(qts, 0x30000);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 0x100), ==, 0xffffffff);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_psram(void)
{
    QTestState *qts = start();

    /*
     * The die on chip select 1, in OPI DTR mode, the strobes' inputs on,
     * at its power-up latencies.
     */
    esp32p4_psram_spi1_opi(qts);
    esp32p4_psram_dqs_on(qts);

    /*
     * Mode registers 0 to 3 as ESP-IDF reads them: 16 bits at 0 and 2; MR3
     * with RBXen (row boundary crossing) and 4x refresh.
     */
    spi1_user(qts, SPI1_PSRAM, 0x4040, 16, 0, 32, ESP32P4_PSRAM_PUP_DUMMY, 0,
              2);
    g_assert_cmphex(qtest_readl(qts, SPI1_PSRAM + W(0)) & 0xffff, ==,
                    0x0d08);
    spi1_user(qts, SPI1_PSRAM, 0x4040, 16, 2, 32, ESP32P4_PSRAM_PUP_DUMMY, 0,
              2);
    g_assert_cmphex(qtest_readl(qts, SPI1_PSRAM + W(0)) & 0xffff, ==,
                    0xa0df);

    /* Synchronous write and read, and the same memory through SPI0. */
    qtest_writel(qts, SPI1_PSRAM + W(0), 0xa5a55a5a);
    qtest_writel(qts, SPI1_PSRAM + W(1), 0x0f0ff0f0);
    spi1_user(qts, SPI1_PSRAM, 0x8080, 16, 0x70008, 32,
              ESP32P4_PSRAM_PUP_DUMMY, 8, 0);
    qtest_writel(qts, SPI1_PSRAM + W(0), 0);
    spi1_user(qts, SPI1_PSRAM, 0x0000, 16, 0x7000c, 32,
              ESP32P4_PSRAM_PUP_DUMMY, 0, 4);
    g_assert_cmphex(qtest_readl(qts, SPI1_PSRAM + W(0)), ==, 0x0f0ff0f0);

    esp32p4_psram_spi0_opi(qts, false, ESP32P4_PSRAM_PUP_DUMMY,
                           ESP32P4_PSRAM_PUP_DUMMY);
    qtest_writel(qts, SPI0_PSRAM + MMU_ITEM_INDEX, 1);
    qtest_writel(qts, SPI0_PSRAM + MMU_ITEM_CONTENT,
                 PSRAM_VALID | PSRAM_ACCESS | 7);
    g_assert_cmphex(qtest_readl(qts, PSRAM_DIRECT + PAGE + 8), ==, 0xa5a55a5a);

    /* The PSRAM controller's own interrupt source. */
    qtest_writel(qts, SPI1_PSRAM + INT_ENA, INT_MST_ST_END);
    g_assert_true(source(qts, SOURCE_PSRAM_MSPI));
    g_assert_false(source(qts, SOURCE_MSPI));
    qtest_writel(qts, SPI1_PSRAM + INT_CLR, 0x41f);
    g_assert_false(source(qts, SOURCE_PSRAM_MSPI));
    qtest_quit(qts);
}

/*
 * Manual encryption with no key in the eFuses: the all-zero 256-bit key
 * (TRM table 34.4-1). The ciphertext is espsecure's calculation (as
 * encrypt-flash-data --aes-xts at 0x21040), which OpenSSL refuses with a
 * key whose halves are equal, made with AES-ECB.
 */
static const uint32_t xts_plain[16] = {
    0x583b1e01, 0xccaf9275, 0x402306e9, 0xb4977a5d,
    0x280beed1, 0x9c7f6245, 0x10f3d6b9, 0x84674a2d,
    0xf8dbbea1, 0x6c4f3215, 0xe0c3a689, 0x54371afd,
    0xc8ab8e71, 0x3c1f02e5, 0xb0937659, 0x2407eacd,
};
static const uint32_t xts_cipher[16] = {
    0x0dc6aebd, 0xb1f47cee, 0x26c75ad7, 0x4b05bb52,
    0xf4e4cc1d, 0x2438c612, 0xa76aa948, 0xf4e56434,
    0x0088ae53, 0xa8c199fb, 0x8ecbb1c7, 0xe335bae7,
    0xcfccd998, 0x3fae574f, 0xdc276656, 0xb4b3ceb4,
};

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_xts_manual(void)
{
    QTestState *qts = start();
    uint32_t addr = 0x21040;
    int i;

    flash_erase(qts, 0x21000);
    qtest_writel(qts, HP_SYSTEM_CRYPTO_CTRL, ENABLE_SPI_MANUAL);
    qtest_writel(qts, SPI0_FLASH + XTS_DESTINATION, 0);
    qtest_writel(qts, SPI0_FLASH + XTS_LINESIZE, 2);
    for (i = 0; i < 16; i++) {
        qtest_writel(qts, SPI0_FLASH + XTS_PLAIN(i), xts_plain[i]);
    }
    qtest_writel(qts, SPI0_FLASH + XTS_PHYSICAL_ADDRESS, addr);
    qtest_writel(qts, SPI0_FLASH + XTS_TRIGGER, 1);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + XTS_STATE), ==, 2);
    qtest_writel(qts, SPI0_FLASH + XTS_RELEASE, 1);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + XTS_STATE), ==, 3);

    /* Two programs of 32 bytes: each takes its part of the ciphertext. */
    flash_program(qts, addr, xts_plain, 8);
    flash_program(qts, addr + 32, xts_plain + 8, 8);
    qtest_writel(qts, SPI0_FLASH + XTS_DESTROY, 1);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + XTS_STATE), ==, 0);

    /* Without the released ciphertext a program sends its data. */
    flash_program(qts, addr + 64, xts_plain, 1);
    qtest_writel(qts, HP_SYSTEM_CRYPTO_CTRL, 0);

    map_flash(qts, 0, 2);
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 0x1040 + 4 * i), ==,
                        xts_cipher[i]);
    }
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 0x1080), ==,
                    xts_plain[0]);
    qtest_quit(qts);
}

/*
 * SPI0's MOSI_DLEN and MISO_DLEN, in both controllers: bits 9:0 hold what
 * is written, as the boot ROM's SPI_init() sets them (a read-modify-write
 * of 0xFF into bits 9:0), and the controller's reset clears them.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_spi0_data_lengths(void)
{
    static const uint32_t spi0[] = { SPI0_FLASH, SPI0_PSRAM };
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(spi0); i++) {
        g_assert_cmphex(qtest_readl(qts, spi0[i] + MOSI_DLEN), ==, 0);
        g_assert_cmphex(qtest_readl(qts, spi0[i] + MISO_DLEN), ==, 0);
        qtest_writel(qts, spi0[i] + MISO_DLEN,
                     (qtest_readl(qts, spi0[i] + MISO_DLEN) & ~0x3ffu) | 0xff);
        qtest_writel(qts, spi0[i] + MOSI_DLEN,
                     (qtest_readl(qts, spi0[i] + MOSI_DLEN) & ~0x3ffu) | 0xff);
        g_assert_cmphex(qtest_readl(qts, spi0[i] + MISO_DLEN), ==, 0xff);
        g_assert_cmphex(qtest_readl(qts, spi0[i] + MOSI_DLEN), ==, 0xff);
        qtest_writel(qts, spi0[i] + MOSI_DLEN, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, spi0[i] + MOSI_DLEN), ==, 0x3ff);
    }

    /* The flash controller's register reset (RST_EN_MSPI_APB) clears them. */
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) | RST_MSPI_APB);
    qtest_writel(qts, HP_RST_EN0, qtest_readl(qts, HP_RST_EN0) & ~RST_MSPI_APB);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + MOSI_DLEN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + MISO_DLEN), ==, 0);
    qtest_quit(qts);
}

/*
 * Fields the v3 headers make hardware read-only (HRO) though the IP's
 * other instances use them: SPI0's user command (CMD's USR), in both
 * controllers, and the flash controller's DDR mode (FMEM_DDR_EN,
 * FMEM_VAR_DUMMY, DQS_LOOP, on SPI0 and SPI1): a write leaves them as
 * from reset and starts nothing. The PSRAM controller's DDR fields take
 * what is written.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_hro_fields(void)
{
    static const uint32_t spi0[] = { SPI0_FLASH, SPI0_PSRAM };
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(spi0); i++) {
        uint32_t raw = qtest_readl(qts, spi0[i] + INT_RAW);

        qtest_writel(qts, spi0[i] + CMD, CMD_USR);
        g_assert_cmphex(qtest_readl(qts, spi0[i] + CMD), ==, 0);
        g_assert_cmphex(qtest_readl(qts, spi0[i] + INT_RAW), ==, raw);
    }
    qtest_writel(qts, SPI0_FLASH + DDR, DDR_EN | DDR_VAR_DUMMY | DQS_LOOP);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + DDR), ==, 0x3020);
    qtest_writel(qts, SPI0_FLASH + SMEM_DDR, DDR_EN | DDR_VAR_DUMMY);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + SMEM_DDR), ==, 0x3020);
    qtest_writel(qts, SPI1_FLASH + DDR, DDR_EN | DDR_VAR_DUMMY);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + DDR), ==, 0x20);
    qtest_writel(qts, SPI0_PSRAM + SMEM_DDR, DDR_EN | DDR_VAR_DUMMY);
    g_assert_cmphex(qtest_readl(qts, SPI0_PSRAM + SMEM_DDR), ==,
                    DDR_EN | DDR_VAR_DUMMY);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/spimem/rdid-timing", test_rdid_timing);
    qtest_add_func("/esp32p4/spimem/program-erase", test_program_erase);
    qtest_add_func("/esp32p4/spimem/psram", test_psram);
    qtest_add_func("/esp32p4/spimem/xts-manual", test_xts_manual);
    qtest_add_func("/esp32p4/spimem/spi0-data-lengths",
                   test_spi0_data_lengths);
    qtest_add_func("/esp32p4/spimem/hro-fields", test_hro_fields);
    return g_test_run();
}
