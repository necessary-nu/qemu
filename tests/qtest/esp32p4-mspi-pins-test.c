/*
 * QTest testcase for the ESP32-P4's IOMUX_MSPI_PIN
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The flash and PSRAM pads' registers (ESP-IDF's v3 iomux_mspi_pin_reg.h),
 * their reset with the IO MUX, and what reads see through them: the flash
 * pads' pulls on lines nothing drives, and in a DDR read of the PSRAM the
 * strobes' power, phase and delay lines against the data lines' (see
 * hw/gpio/esp32p4_mspi_iomux.c): each beat valid from 400 ps after its
 * strobe edge to 400 ps before the next, sampled (6 + PHASE) 32nds of the
 * SPI clock after its edge, plus the strobe's delay line less the data
 * line's, 100 ps a step.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32p4-psram.h"
#include "qobject/qdict.h"

#define PINS                0x500e1200u
#define CLK_EN0             (PINS + 0x00)
#define FLASH_PIN(n)        (PINS + 0x04 + 4 * (n))
#define FLASH_CS            0
#define FLASH_Q             1
#define FLASH_WP            2
#define FLASH_HOLD          3
#define FLASH_CK            4
#define FLASH_D             5
#define PSRAM_PIN0(n)       (PINS + 0x1c + 4 * (n))
#define PSRAM_DQS(n)        (PINS + ((n) ? 0x68 : 0x3c))
#define PSRAM_PIN1(n)       (PINS + 0x40 + 4 * (n))
#define PINS_END            (PINS + 0x6c)
#define F_WPU               (1u << 2)
#define F_WPD               (1u << 3)
#define P_DLC(v)            ((uint32_t)(v) << 4)
#define DQS_XPD             (1u << 0)
#define DQS_PHASE(v)        ((uint32_t)(v) << 1)
#define DQS_DELAY_90(v)     ((uint32_t)(v) << 7)
#define DQS_DELAY_270(v)    ((uint32_t)(v) << 17)

#define SPI1_FLASH          0x5008d000u
#define SPI0_PSRAM          0x5008e000u
#define SPI1_PSRAM          0x5008f000u
#define PSRAM_DIRECT        0x88000000u
#define CMD                 0x000
#define ADDR                0x004
#define CLOCK               0x014
#define USER                0x018
#define USER1               0x01c
#define USER2               0x020
#define MOSI_DLEN           0x024
#define MISO_DLEN           0x028
#define MISC                0x034
#define CACHE_FCTRL         0x03c
#define SRAM_CLK            0x050
#define W(n)                (0x058 + 4 * (n))
#define DDR                 0x0d4
#define SPI1_TIMING_CALI    0x180
#define TIMING_CALI_EN      (1u << 1)
#define EXTRA_DUMMY(n)      ((uint32_t)(n) << 2)
#define SMEM_DDR            0x0d8
#define MMU_ITEM_CONTENT    0x37c
#define MMU_ITEM_INDEX      0x380
#define CMD_USR             (1u << 18)
#define USR_MOSI            (1u << 27)
#define USR_MISO            (1u << 28)
#define USR_DUMMY           (1u << 29)
#define USR_ADDR            (1u << 30)
#define USR_COMMAND         (1u << 31)
#define FDIN_QUAD           (1u << 6)
#define CLOSE_AXI_INF_EN    (1u << 31)
#define CLK_EQU_SYSCLK      (1u << 31)
#define DDR_EN              (1u << 0)
#define DDR_DQS_LOOP        (1u << 21)
#define PSRAM_VALID         (1u << 11)
#define PSRAM_ACCESS        (1u << 10)
#define PSRAM_SYNC_READ     0x0000
#define PSRAM_SYNC_WRITE    0x8080
#define PSRAM_REG_WRITE     0xc0c0

#define HP_SYS_CLKRST_HP_RST_EN1    0x500e60c4u
#define RST_EN_IOMUX                (1u << 3)
#define HP_SYS_CLKRST_PERI_CLK_CTRL00 0x500e6030u
#define PSRAM_CLK_SRC(s)            ((uint32_t)(s) << 12)
#define PSRAM_CORE_CLK_EN           (1u << 15)
#define PSRAM_CORE_DIV(d)           ((uint32_t)(d) << 16)

#define TDQSQ_PS            400
#define STEP_PS             100

/*
 * MR0 for 240 MHz: fixed latency, read latency code 6 (FLC 18, up to
 * 250 MHz), the read dummy phase that matches it; the power-up code 2
 * allows 133 MHz at most, faster reads of the array being undefined.
 */
#define MR0_FIXED_LC6       0x38
#define FLC18_RD_DUMMY      (2 * (18 - 1))

static QTestState *qts;
static int rd_dummy;

static uint32_t rd(uint32_t a)
{
    return qtest_readl(qts, a);
}

static void wr(uint32_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static uint64_t clock_period_ps(const char *name)
{
    g_autofree char *path = g_strdup_printf("/machine/clkrst/%s", name);
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         path);
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    /* Clock periods are in 2^-32 ns. */
    return (period * 1000) >> 32;
}

/* Start an SPI1 command and run the clock until it ends. */
static void spi1_run(uint32_t spi1, uint32_t cmd)
{
    wr(spi1 + CMD, cmd);
    for (int i = 0; i < 1000 && rd(spi1 + CMD); i++) {
        qtest_clock_step(qts, 10000);
    }
    g_assert_cmphex(rd(spi1 + CMD), ==, 0);
}

static void spi1_user(uint32_t spi1, uint32_t cmd, int cmd_bits,
                      uint32_t addr, int addr_bits, int dummy, int out_len,
                      int in_len)
{
    uint32_t user = USR_COMMAND, user1 = 0;

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
        wr(spi1 + MOSI_DLEN, out_len * 8 - 1);
    }
    if (in_len) {
        user |= USR_MISO;
        wr(spi1 + MISO_DLEN, in_len * 8 - 1);
    }
    wr(spi1 + USER, user);
    wr(spi1 + USER1, user1);
    wr(spi1 + USER2, ((uint32_t)(cmd_bits - 1) << 28) | cmd);
    wr(spi1 + ADDR, addr);
    spi1_run(spi1, CMD_USR);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_registers(void)
{
    static const struct {
        uint32_t addr;
        uint32_t mask;
    } regs[] = {
        { CLK_EN0, 0x1 },
        { FLASH_PIN(FLASH_CS), 0x3f }, { FLASH_PIN(FLASH_Q), 0x3f },
        { FLASH_PIN(FLASH_WP), 0x3f }, { FLASH_PIN(FLASH_HOLD), 0x3f },
        { FLASH_PIN(FLASH_CK), 0x3f }, { FLASH_PIN(FLASH_D), 0x3f },
        { PSRAM_PIN0(0), 0x3fff }, { PSRAM_PIN0(7), 0x3fff },
        { PSRAM_DQS(0), 0x1fffff },
        { PSRAM_PIN1(0), 0x3fff }, { PSRAM_PIN1(9), 0x3fff },
        { PSRAM_DQS(1), 0x1fffff },
    };

    qts = qtest_init("-M esp32p4,strap-mode=0");
    g_assert_cmphex(rd(CLK_EN0), ==, 1);
    for (uint32_t a = FLASH_PIN(0); a < PINS_END; a += 4) {
        g_assert_cmphex(rd(a), ==, 0);
    }
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        wr(regs[i].addr, 0xffffffff);
        g_assert_cmphex(rd(regs[i].addr), ==, regs[i].mask);
    }
    /* Bytes of a register; and no register past DQS1's. */
    qtest_writeb(qts, PSRAM_DQS(0) + 2, 0);
    g_assert_cmphex(rd(PSRAM_DQS(0)), ==, 0xffff);
    g_assert_cmphex(qtest_readb(qts, PSRAM_DQS(0) + 1), ==, 0xff);
    wr(PINS_END, 0xffffffff);
    g_assert_cmphex(rd(PINS_END), ==, 0);
    g_assert_cmphex(rd(PINS + 0xdfc), ==, 0);

    /* RST_EN_IOMUX resets them with the IO MUX. */
    wr(HP_SYS_CLKRST_HP_RST_EN1, rd(HP_SYS_CLKRST_HP_RST_EN1) |
       RST_EN_IOMUX);
    wr(HP_SYS_CLKRST_HP_RST_EN1, rd(HP_SYS_CLKRST_HP_RST_EN1) &
       ~RST_EN_IOMUX);
    g_assert_cmphex(rd(CLK_EN0), ==, 1);
    g_assert_cmphex(rd(PSRAM_DQS(0)), ==, 0);
    g_assert_cmphex(rd(FLASH_PIN(FLASH_Q)), ==, 0);
    qtest_quit(qts);
}

/*
 * A board with no flash: the data lines float, and read 0, until a pull
 * holds them: Q's pull-up gives 1s in a single-line read; a quad read
 * takes each of IO0 (D) to IO3 (HOLD) as its pull has it.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_flash_pulls(void)
{
    qts = qtest_init("-M esp32p4,strap-mode=0");
    spi1_user(SPI1_FLASH, 0x9f, 8, 0, 0, 0, 0, 3);
    g_assert_cmphex(rd(SPI1_FLASH + W(0)) & 0xffffff, ==, 0);
    wr(FLASH_PIN(FLASH_Q), F_WPU);
    spi1_user(SPI1_FLASH, 0x9f, 8, 0, 0, 0, 0, 3);
    g_assert_cmphex(rd(SPI1_FLASH + W(0)) & 0xffffff, ==, 0xffffff);
    wr(FLASH_PIN(FLASH_Q), F_WPD);
    spi1_user(SPI1_FLASH, 0x9f, 8, 0, 0, 0, 0, 3);
    g_assert_cmphex(rd(SPI1_FLASH + W(0)) & 0xffffff, ==, 0);
    /* IO3..IO0 = HOLD up, WP down, Q floating, D up: 1001 each clock. */
    wr(FLASH_PIN(FLASH_D), F_WPU);
    wr(FLASH_PIN(FLASH_HOLD), F_WPU);
    wr(FLASH_PIN(FLASH_WP), F_WPD);
    wr(FLASH_PIN(FLASH_Q), 0);
    wr(SPI1_FLASH + CACHE_FCTRL, rd(SPI1_FLASH + CACHE_FCTRL) | FDIN_QUAD);
    spi1_user(SPI1_FLASH, 0xeb, 8, 0, 24, 6, 0, 4);
    g_assert_cmphex(rd(SPI1_FLASH + W(0)), ==, 0x99999999);
    qtest_quit(qts);
}

/*
 * The boot ROM on a board with no flash: its SelectSpiFunction() pulls
 * the flash pads up, and its first status read finds the flash busy.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_rom_without_flash(void)
{
    g_autofree char *log = NULL;
    bool found = false;
    int fd = g_file_open_tmp("esp32p4-mspi-pins-XXXXXX.log", &log, NULL);

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = qtest_initf("-M esp32p4 -accel tcg -icount shift=0,sleep=off "
                      "-chardev file,id=s0,path=%s -serial chardev:s0", log);
    for (int i = 0; i < 300 && !found; i++) {
        g_autofree char *text = NULL;

        g_assert(g_file_get_contents(log, &text, NULL, NULL));
        found = strstr(text, "SPI flash busy detected(0xff)") != NULL;
        if (!found) {
            g_usleep(100000);
        }
    }
    g_assert_true(found);
    qtest_quit(qts);
    unlink(log);
}

/*
 * The PSRAM's SPI clock at 240 MHz: SPLL's 480 MHz halved, SPI1 at its
 * core clock.
 */
static uint64_t psram_fast(void)
{
    uint64_t spll = clock_period_ps("spll");

    g_assert_cmpuint(spll, >, 0);
    wr(HP_SYS_CLKRST_PERI_CLK_CTRL00,
       (rd(HP_SYS_CLKRST_PERI_CLK_CTRL00) & ~(0xffu << 16 | 3u << 12)) |
       PSRAM_CLK_SRC(2) | PSRAM_CORE_CLK_EN | PSRAM_CORE_DIV(1));
    wr(SPI1_PSRAM + CLOCK, CLK_EQU_SYSCLK);
    wr(SPI0_PSRAM + SRAM_CLK, CLK_EQU_SYSCLK);
    return clock_period_ps("psram-core");
}

/* Write @words words of @data at PSRAM address @addr through SPI1. */
static void psram_write(uint32_t addr, const uint32_t *data, int words)
{
    for (int i = 0; i < words; i++) {
        wr(SPI1_PSRAM + W(i), data[i]);
    }
    spi1_user(SPI1_PSRAM, PSRAM_SYNC_WRITE, 16, addr, 32,
              ESP32P4_PSRAM_PUP_DUMMY, 4 * words, 0);
}

static void psram_read(uint32_t addr, uint32_t *data, int words)
{
    for (int i = 0; i < words; i++) {
        wr(SPI1_PSRAM + W(i), 0);
    }
    spi1_user(SPI1_PSRAM, PSRAM_SYNC_READ, 16, addr, 32, rd_dummy, 0,
              4 * words);
    for (int i = 0; i < words; i++) {
        data[i] = rd(SPI1_PSRAM + W(i));
    }
}

/* Mode register @ma written with @v, at its write latency of 1. */
static void psram_mr_write(uint32_t ma, uint32_t v)
{
    wr(SPI1_PSRAM + W(0), v);
    spi1_user(SPI1_PSRAM, PSRAM_REG_WRITE, 16, ma, 32, 0, 1, 0);
}

/* Every data line's delay line at @dlc. */
static void data_delay(unsigned dlc)
{
    for (int i = 0; i < 8; i++) {
        wr(PSRAM_PIN0(i), P_DLC(dlc));
        wr(PSRAM_PIN1(2 + i), P_DLC(dlc));
    }
}

/*
 * DDR reads through SPI1 of a die in x8 mode, a byte a beat: with the
 * strobe's input off they capture nothing; on, at phase 67.5 degrees,
 * the data; the data lines delayed 1.5 ns, at 240 MHz, sampled before
 * their beats, each byte reads the one before; the strobe delayed 1.5 ns,
 * after them, each reads the one after; the rising edges' delay alone,
 * the even beats only. DQS_LOOP takes no strobe: the controller samples on
 * its own clock, and the die's data, tDQSCK (4.25 ns) behind its clock,
 * come two beats late at 240 MHz, until an extra dummy clock; in SDR the
 * die takes no command at all.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_psram_strobe(void)
{
    static const uint32_t ref[2] = { 0x44332211, 0x88776655 };
    uint32_t got[2];
    uint64_t t;
    int64_t at;

    qts = qtest_init("-M esp32p4,strap-mode=0");
    esp32p4_psram_spi1_opi(qts);
    rd_dummy = ESP32P4_PSRAM_PUP_DUMMY;
    psram_write(0x100, ref, 2);
    psram_mr_write(0, MR0_FIXED_LC6);
    rd_dummy = FLC18_RD_DUMMY;
    t = psram_fast();
    g_assert_cmpuint(t, >=, 4100);
    g_assert_cmpuint(t, <=, 4200);

    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0);
    g_assert_cmphex(got[1], ==, 0);

    wr(PSRAM_DQS(0), DQS_XPD | DQS_PHASE(0));
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, ref[0]);
    g_assert_cmphex(got[1], ==, ref[1]);

    /* 0.78 ns less 1.5 ns: before the beat's 0.4 ns. */
    at = (int64_t)t * 6 / 32 - 15 * STEP_PS;
    g_assert_cmpint(at, <, TDQSQ_PS);
    data_delay(15);
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0x33221100);
    g_assert_cmphex(got[1], ==, 0x77665544);

    /* 0.78 ns and 1.5 ns: past the beat's end, T/2 - 0.4 ns. */
    at = (int64_t)t * 6 / 32 + 15 * STEP_PS;
    g_assert_cmpint(at, >, (int64_t)t / 2 - TDQSQ_PS);
    data_delay(0);
    wr(PSRAM_DQS(0), DQS_XPD | DQS_DELAY_90(15) | DQS_DELAY_270(15));
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0x55443322);
    g_assert_cmphex(got[1], ==, 0x00887766);

    /* The rising edges', the even beats', alone. */
    wr(PSRAM_DQS(0), DQS_XPD | DQS_DELAY_90(15));
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0x44442222);
    g_assert_cmphex(got[1], ==, 0x88886666);

    /*
     * DQS_LOOP: the controller's own clock, two beats early at 240 MHz,
     * the die's data lagging its clock by tDQSCK; an extra dummy clock
     * (SPI1's TIMING_CALI) puts the samples on the data. In SDR the die
     * takes no command: nothing drives the lines. The strobe's delay
     * lines, left set, change nothing.
     */
    wr(SPI1_PSRAM + DDR, rd(SPI1_PSRAM + DDR) | DDR_DQS_LOOP);
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0x22110000);
    g_assert_cmphex(got[1], ==, 0x66554433);
    wr(SPI1_PSRAM + SPI1_TIMING_CALI, TIMING_CALI_EN | EXTRA_DUMMY(1));
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, ref[0]);
    g_assert_cmphex(got[1], ==, ref[1]);
    wr(SPI1_PSRAM + SPI1_TIMING_CALI, 0);
    wr(SPI1_PSRAM + DDR, rd(SPI1_PSRAM + DDR) & ~(DDR_DQS_LOOP | DDR_EN));
    wr(PSRAM_DQS(0), 0);
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0);
    qtest_quit(qts);
}

/*
 * The die in x16 mode (MR8 bit 6), the ports' data on 16 lines (SPI0's
 * SRAM_CMD SDIN_HEX): two bytes a beat, the even on DQ0 to DQ7 with DQS0,
 * the odd on DQ8 to DQ15 with DQS1. DQS1 off: the odd bytes read 0. DQ8
 * to DQ15 delayed: the odd bytes take the beat before's. In x16 the die
 * counts its address in words: the bytes written at 0x200 in x8 are at
 * word 0x100 for SPI1, and at 0x200 for SPI0, which on 16 lines sends
 * the word address of the window's byte. Through SPI0 and the MMU, with
 * SMEM_DDR, alike; in SDR the die takes no command.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_psram_x16(void)
{
    static const uint32_t ref[2] = { 0x44332211, 0x88776655 };
    uint32_t got[2], mr8 = 1u << 6;

    qts = qtest_init("-M esp32p4,strap-mode=0");
    esp32p4_psram_spi1_opi(qts);
    rd_dummy = ESP32P4_PSRAM_PUP_DUMMY;
    psram_write(0x200, ref, 2);
    psram_mr_write(8, mr8);
    psram_mr_write(0, MR0_FIXED_LC6);
    rd_dummy = FLC18_RD_DUMMY;
    psram_fast();
    /* Both ports' data on 16 lines, which SPI0's SRAM_CMD sets. */
    esp32p4_psram_spi0_opi(qts, true, FLC18_RD_DUMMY,
                           ESP32P4_PSRAM_PUP_DUMMY);
    wr(PSRAM_DQS(0), DQS_XPD);
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0x00330011);
    g_assert_cmphex(got[1], ==, 0x00770055);
    wr(PSRAM_DQS(1), DQS_XPD);
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, ref[0]);
    g_assert_cmphex(got[1], ==, ref[1]);
    for (int i = 0; i < 8; i++) {
        wr(PSRAM_PIN1(2 + i), P_DLC(15));
    }
    psram_read(0x100, got, 2);
    g_assert_cmphex(got[0], ==, 0x22330011);
    g_assert_cmphex(got[1], ==, 0x66774455);

    /* SPI0's DDR read through the same pads; in SDR, nothing. */
    wr(SPI0_PSRAM + MMU_ITEM_INDEX, 0);
    wr(SPI0_PSRAM + MMU_ITEM_CONTENT, PSRAM_VALID | PSRAM_ACCESS | 0);
    g_assert_cmphex(rd(PSRAM_DIRECT + 0x200), ==, 0x22330011);
    wr(SPI0_PSRAM + SMEM_DDR, rd(SPI0_PSRAM + SMEM_DDR) & ~DDR_EN);
    g_assert_cmphex(rd(PSRAM_DIRECT + 0x200), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/mspi-pins/registers", test_registers);
    qtest_add_func("esp32p4/mspi-pins/flash_pulls", test_flash_pulls);
    qtest_add_func("esp32p4/mspi-pins/rom_without_flash",
                   test_rom_without_flash);
    qtest_add_func("esp32p4/mspi-pins/psram_strobe", test_psram_strobe);
    qtest_add_func("esp32p4/mspi-pins/psram_x16", test_psram_x16);
    return g_test_run();
}
