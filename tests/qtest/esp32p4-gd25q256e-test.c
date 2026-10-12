/*
 * QTest testcase for the ESP32-P4 board's GigaDevice GD25Q256E, through
 * the flash MSPI controller's SPI1, and its boot through the ROM
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * From reset the controller's core clock is the 40 MHz crystal divided by
 * 4, its edges every 100 ns from time 0, and SPI1's clock that divided by
 * 4: 400 ns a cycle. A user command without FLASH_PE ends on the core
 * clock edge its phases' cycles take it to, when the flash sees /CS rise
 * and starts a program or erase; a status read samples the flash as it
 * starts. Virtual time moves only as the test steps it, so the busy times
 * are checked to the nanosecond.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "esp32p4-boot-utils.h"

#define SPI1_FLASH          0x5008d000
#define CMD                 0x000
#define ADDR                0x004
#define USER                0x018
#define USER1               0x01c
#define USER2               0x020
#define MOSI_DLEN           0x024
#define MISO_DLEN           0x028
#define W(n)                (0x058 + 4 * (n))
#define CMD_USR             (1u << 18)
#define CMD_RDID            (1u << 28)
#define USR_MOSI            (1u << 27)
#define USR_MISO            (1u << 28)
#define USR_DUMMY           (1u << 29)
#define USR_ADDR            (1u << 30)
#define USR_COMMAND         (1u << 31)

#define SPI_CYCLE_NS        400LL
#define CORE_CYCLE_NS       100LL

#define SR1_WIP             0x01
#define SR1_WEL             0x02
#define SR2_ADS             0x01
#define SR2_QE              0x02
#define SR2_SUS2            0x04
#define SR2_SUS1            0x80
#define SR3_PE              0x04
#define SR3_EE              0x08
#define SR3_ADP             0x10

#define US                  1000LL
#define MS                  1000000LL
#define S                   1000000000LL

/* Typical times, range I. */
#define T_W                 (5 * MS)
#define T_BP1               (40 * US)
#define T_BP2               2500LL
#define T_PP                (250 * US)
#define T_SE                (30 * MS)
#define T_BE32              (120 * MS)
#define T_BE64              (150 * MS)
#define T_CE                (70 * S)
#define T_SUS               (20 * US)
#define T_RS                (100 * US)

#define HIGH                0x1200000u

/* The virtual time, which only steps through step() change. */
static int64_t now_ns;

static void step(QTestState *qts, int64_t ns)
{
    if (ns > 0) {
        now_ns = qtest_clock_step(qts, ns);
    }
}

/* strap-mode=0: no flash-boot watchdogs while the test steps time. */
static QTestState *start(const char *extra)
{
    QTestState *qts = qtest_initf("-M esp32p4,flash-size=32M,strap-mode=0 "
                                  "%s", extra ? extra : "");

    now_ns = qtest_clock_step(qts, CORE_CYCLE_NS);
    step(qts, (CORE_CYCLE_NS - now_ns % CORE_CYCLE_NS) % CORE_CYCLE_NS);
    return qts;
}

/*
 * A user command at the current time: `cmd`, `addr_bits` of `addr`,
 * `dummy` cycles, `out_len` bytes of `out` and `in_len` read into `in`.
 * Steps to its last cycle, when the flash sees /CS rise, and returns that
 * time.
 */
static int64_t user(QTestState *qts, uint8_t cmd, uint32_t addr,
                    int addr_bits, int dummy, const uint8_t *out,
                    int out_len, uint8_t *in, int in_len)
{
    uint32_t u = USR_COMMAND, u1 = 0;
    int64_t cycles = 8 + addr_bits + dummy + 8 * (out_len + in_len);
    int64_t end = (now_ns / CORE_CYCLE_NS) * CORE_CYCLE_NS +
                  cycles * SPI_CYCLE_NS;
    int i;

    if (addr_bits) {
        u |= USR_ADDR;
        u1 |= (uint32_t)(addr_bits - 1) << 26;
    }
    if (dummy) {
        u |= USR_DUMMY;
        u1 |= dummy - 1;
    }
    if (out_len) {
        u |= USR_MOSI;
        qtest_writel(qts, SPI1_FLASH + MOSI_DLEN, out_len * 8 - 1);
        for (i = 0; i < out_len; i += 4) {
            uint8_t w[4] = { 0 };

            memcpy(w, out + i, MIN(4, out_len - i));
            qtest_writel(qts, SPI1_FLASH + W(i / 4), ldl_le_p(w));
        }
    }
    if (in_len) {
        u |= USR_MISO;
        qtest_writel(qts, SPI1_FLASH + MISO_DLEN, in_len * 8 - 1);
    }
    qtest_writel(qts, SPI1_FLASH + USER, u);
    qtest_writel(qts, SPI1_FLASH + USER1, u1);
    qtest_writel(qts, SPI1_FLASH + USER2, (7u << 28) | cmd);
    qtest_writel(qts, SPI1_FLASH + ADDR, addr);
    qtest_writel(qts, SPI1_FLASH + CMD, CMD_USR);
    step(qts, end - 1 - now_ns);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, CMD_USR);
    step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + CMD), ==, 0);
    for (i = 0; i < in_len; i++) {
        in[i] = qtest_readl(qts, SPI1_FLASH + W(i / 4)) >> (8 * (i % 4));
    }
    return end;
}

static int64_t cmd(QTestState *qts, uint8_t op)
{
    return user(qts, op, 0, 0, 0, NULL, 0, NULL, 0);
}

static int64_t cmd_addr(QTestState *qts, uint8_t op, uint32_t addr,
                        int bits)
{
    return user(qts, op, addr, bits, 0, NULL, 0, NULL, 0);
}

static int64_t cmd_data(QTestState *qts, uint8_t op, uint8_t v)
{
    return user(qts, op, 0, 0, 0, &v, 1, NULL, 0);
}

static uint8_t rdsr(QTestState *qts, uint8_t op)
{
    uint8_t v;

    user(qts, op, 0, 0, 0, NULL, 0, &v, 1);
    return v;
}

static uint8_t read_byte(QTestState *qts, uint8_t op, uint32_t addr,
                         int bits)
{
    uint8_t v;

    user(qts, op, addr, bits, op == 0x0c ? 8 : 0, NULL, 0, &v, 1);
    return v;
}

static int64_t program(QTestState *qts, uint8_t op, uint32_t addr,
                       int bits, const uint8_t *data, int len)
{
    cmd(qts, 0x06);
    return user(qts, op, addr, bits, 0, data, len, NULL, 0);
}

static int64_t erase(QTestState *qts, uint8_t op, uint32_t addr, int bits)
{
    cmd(qts, 0x06);
    return cmd_addr(qts, op, addr, bits);
}

/* Step to `t`, which must not have passed. */
static void step_to(QTestState *qts, int64_t t)
{
    g_assert_cmpint(t, >=, now_ns);
    step(qts, t - now_ns);
}

/*
 * A cycle that started at `t0` and takes `t`: WIP still set when a status
 * read starts 1 ns before its end. Steps on past the end.
 */
static void expect_busy_until(QTestState *qts, int64_t t0, int64_t t)
{
    step_to(qts, t0 + t - 1);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, SR1_WIP);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, 0);
}

/* ...and clear when one starts at its end. */
static void expect_done_at(QTestState *qts, int64_t t0, int64_t t)
{
    step_to(qts, t0 + t);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, 0);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
static void test_ids(void)
{
    QTestState *qts = start(NULL);
    uint8_t id[16];

    qtest_writel(qts, SPI1_FLASH + CMD, CMD_RDID);
    step(qts, 40 * SPI_CYCLE_NS);
    g_assert_cmphex(qtest_readl(qts, SPI1_FLASH + W(0)) & 0xffffff, ==,
                    0x1940c8);
    user(qts, 0x90, 0, 24, 0, NULL, 0, id, 2);
    g_assert_cmphex(id[0], ==, 0xc8);
    g_assert_cmphex(id[1], ==, 0x18);
    user(qts, 0xab, 0, 24, 0, NULL, 0, id, 1);
    g_assert_cmphex(id[0], ==, 0x18);
    /* 4Bh: an address and a dummy byte, then 128 bits. */
    user(qts, 0x4b, 0, 24, 8, NULL, 0, id, 16);
    g_assert_cmphex(id[0], ==, 0xc8);
    g_assert_cmphex(id[15], ==, 0xd1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
static void test_status(void)
{
    QTestState *qts = start(NULL);
    int64_t t0;

    /* 00h but DRV0 (S21). */
    g_assert_cmphex(rdsr(qts, 0x05), ==, 0x00);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x00);
    g_assert_cmphex(rdsr(qts, 0x15), ==, 0x20);

    /* 01h writes SR1 only; with a second byte it is not executed. */
    cmd(qts, 0x50);
    user(qts, 0x01, 0, 0, 0, (const uint8_t[]){ 0x04, 0x02 }, 2, NULL, 0);
    g_assert_cmphex(rdsr(qts, 0x05), ==, 0x00);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x00);

    /* QE through 31h, as ESP-IDF sets it: tW, then WEL clear. */
    cmd(qts, 0x06);
    t0 = cmd_data(qts, 0x31, SR2_QE);
    g_assert_cmphex(rdsr(qts, 0x05), ==, SR1_WIP | SR1_WEL);
    expect_busy_until(qts, t0, T_W);
    g_assert_cmphex(rdsr(qts, 0x35), ==, SR2_QE);
    g_assert_cmphex(rdsr(qts, 0x05), ==, 0);

    /* Read-only bits ignore writes: SUS1, SUS2, ADS; PE, EE. */
    cmd(qts, 0x50);
    cmd_data(qts, 0x31, 0x85);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x00);
    cmd(qts, 0x50);
    cmd_data(qts, 0x11, 0xff);
    g_assert_cmphex(rdsr(qts, 0x15), ==, 0xf3);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
static void test_addressing(void)
{
    QTestState *qts = start(NULL);
    static const uint8_t hi[2] = { 0x48, 0x49 };
    static const uint8_t lo[2] = { 0x4c, 0x4f };

    /* The 4-byte-address commands reach the upper 16 MiB in 3-byte mode. */
    erase(qts, 0x21, HIGH, 32);
    step(qts, T_SE);
    program(qts, 0x12, HIGH, 32, hi, 2);
    step(qts, T_PP);
    erase(qts, 0x20, HIGH & 0xffffff, 24);
    step(qts, T_SE);
    program(qts, 0x02, HIGH & 0xffffff, 24, lo, 2);
    step(qts, T_PP);
    g_assert_cmphex(read_byte(qts, 0x13, HIGH, 32), ==, 0x48);
    g_assert_cmphex(read_byte(qts, 0x0c, HIGH + 1, 32), ==, 0x49);
    g_assert_cmphex(read_byte(qts, 0x03, HIGH, 24), ==, 0x4c);

    /* The extended address register's A24 (C5h needs WEL). */
    cmd_data(qts, 0xc5, 1);
    g_assert_cmphex(read_byte(qts, 0x03, HIGH, 24), ==, 0x4c);
    cmd(qts, 0x06);
    cmd_data(qts, 0xc5, 1);
    g_assert_cmphex(rdsr(qts, 0xc8), ==, 1);
    g_assert_cmphex(read_byte(qts, 0x03, HIGH, 24), ==, 0x48);
    cmd(qts, 0x04);

    /* B7h: 4-byte mode (ADS); 03h then takes four address bytes. */
    cmd(qts, 0xb7);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_ADS, ==, SR2_ADS);
    g_assert_cmphex(read_byte(qts, 0x03, HIGH, 32), ==, 0x48);
    g_assert_cmphex(read_byte(qts, 0x03, HIGH & 0xffffff, 32), ==, 0x4c);
    cmd(qts, 0xe9);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_ADS, ==, 0);

    /* ADP: the power-up address mode, after a reset. */
    cmd(qts, 0x06);
    cmd_data(qts, 0x11, 0x20 | SR3_ADP);
    step(qts, T_W);
    qtest_system_reset(qts);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_ADS, ==, SR2_ADS);
    g_assert_cmphex(rdsr(qts, 0xc8), ==, 0);
    g_assert_cmphex(read_byte(qts, 0x03, HIGH, 32), ==, 0x48);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
/* [spec:nuos:req:emu.flash-timing/test] */
static void test_times(void)
{
    QTestState *qts = start(NULL);
    uint8_t data[64];
    int64_t t0;

    memset(data, 0x5a, sizeof(data));
    /* min(tPP, tBP1 + (n - 1) * tBP2): 40 us for a byte, 197.5 for 64. */
    t0 = program(qts, 0x02, 0x1000, 24, data, 1);
    expect_busy_until(qts, t0, T_BP1);
    t0 = program(qts, 0x02, 0x1100, 24, data, 64);
    expect_busy_until(qts, t0, T_BP1 + 63 * T_BP2);
    t0 = program(qts, 0x12, HIGH + 0x100, 32, data, 64);
    expect_done_at(qts, t0, T_BP1 + 63 * T_BP2);

    t0 = erase(qts, 0x20, 0x1000, 24);
    expect_busy_until(qts, t0, T_SE);
    t0 = erase(qts, 0x21, HIGH, 32);
    expect_done_at(qts, t0, T_SE);
    t0 = erase(qts, 0x52, 0x8000, 24);
    expect_busy_until(qts, t0, T_BE32);
    t0 = erase(qts, 0x5c, HIGH, 32);
    expect_done_at(qts, t0, T_BE32);
    t0 = erase(qts, 0xd8, 0x10000, 24);
    expect_busy_until(qts, t0, T_BE64);
    t0 = erase(qts, 0xdc, HIGH, 32);
    expect_done_at(qts, t0, T_BE64);
    cmd(qts, 0x06);
    t0 = cmd(qts, 0xc7);
    expect_busy_until(qts, t0, T_CE);

    /* GD25Q256E: WEL clears as the cycle starts. */
    program(qts, 0x02, 0x2000, 24, data, 1);
    g_assert_cmphex(rdsr(qts, 0x05), ==, SR1_WIP);
    step(qts, T_PP);
    qtest_quit(qts);

    qts = start("-global gd25q256e.max-timing=on");
    t0 = program(qts, 0x02, 0x1000, 24, data, 64);
    expect_busy_until(qts, t0, MIN(2 * MS, 90 * US + 63 * 7 * US));
    t0 = erase(qts, 0x21, HIGH, 32);
    expect_busy_until(qts, t0, 400 * MS);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-timing/test] */
static void test_suspend(void)
{
    QTestState *qts = start(NULL);
    uint8_t data[64];
    int64_t t0, sus, res, ignored, done;

    memset(data, 0x00, sizeof(data));
    t0 = erase(qts, 0x21, HIGH, 32);
    step_to(qts, t0 + 10 * MS);
    sus = cmd(qts, 0x75);
    done = sus + T_SUS - t0;
    /* SUS1 for an erase, at once; WIP clears tSUS after /CS rises. */
    g_assert_cmphex(rdsr(qts, 0x35) & (SR2_SUS1 | SR2_SUS2), ==, SR2_SUS1);
    step_to(qts, sus + T_SUS - 1);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, SR1_WIP);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, 0);

    /* Programs run; erases, 44h and status writes are refused. */
    program(qts, 0x02, 0x3000, 24, data, 4);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, SR1_WIP);
    step(qts, T_PP);
    erase(qts, 0x20, 0x4000, 24);
    g_assert_cmphex(rdsr(qts, 0x05), ==, SR1_WEL);
    cmd_addr(qts, 0x44, 0x1000, 24);
    g_assert_cmphex(rdsr(qts, 0x05), ==, SR1_WEL);
    cmd_data(qts, 0x31, SR2_QE);
    g_assert_cmphex(rdsr(qts, 0x35), ==, SR2_SUS1);
    cmd(qts, 0x04);

    /*
     * Resume; a suspend sooner than tRS (100 us) after it is ignored: one
     * ending at 99.9 us, whose tSUS would have passed when the next ends,
     * leaves WIP set until the next's tSUS has.
     */
    res = cmd(qts, 0x7a);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS1, ==, 0);
    step_to(qts, res + T_RS - 100 - 8 * SPI_CYCLE_NS);
    ignored = cmd(qts, 0x75);
    g_assert_cmpint(ignored, ==, res + T_RS - 100);
    sus = cmd(qts, 0x75);
    step_to(qts, ignored + T_SUS);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, SR1_WIP);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, 0);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS1, ==, SR2_SUS1);
    done += sus + T_SUS - res;
    res = cmd(qts, 0x7a);
    expect_busy_until(qts, res, T_SE - done);

    /* A program suspends with SUS2 and refuses programs meanwhile. */
    t0 = program(qts, 0x02, 0x5000, 24, data, 64);
    sus = cmd(qts, 0x75);
    g_assert_cmphex(rdsr(qts, 0x35) & (SR2_SUS1 | SR2_SUS2), ==, SR2_SUS2);
    step(qts, T_SUS);
    program(qts, 0x02, 0x6000, 24, data, 4);
    g_assert_cmphex(rdsr(qts, 0x05), ==, SR1_WEL);
    cmd(qts, 0x04);
    g_assert_cmphex(read_byte(qts, 0x03, 0x3000, 24), ==, 0x00);
    res = cmd(qts, 0x7a);
    expect_busy_until(qts, res,
                      T_BP1 + 63 * T_BP2 - (sus - t0) - T_SUS);
    g_assert_cmphex(read_byte(qts, 0x03, 0x5000, 24), ==, 0x00);

    /* A reset that cuts an erase short takes tRST_E, 12 ms. */
    erase(qts, 0x20, 0x7000, 24);
    step(qts, MS);
    cmd(qts, 0x66);
    t0 = cmd(qts, 0x99);
    step_to(qts, t0 + 12 * MS - 1);
    g_assert_cmphex(rdsr(qts, 0x15), ==, 0);
    g_assert_cmphex(rdsr(qts, 0x15), ==, 0x20);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
static void test_protection(void)
{
    QTestState *qts = start(NULL);
    uint8_t zero = 0;

    /* BP3-0 = 0001, BP4 = 0: the top 64 KiB. */
    cmd(qts, 0x50);
    cmd_data(qts, 0x01, 0x04);
    erase(qts, 0x21, 0x1ff0000, 32);
    g_assert_cmphex(rdsr(qts, 0x05), ==, 0x04 | SR1_WEL);
    g_assert_cmphex(rdsr(qts, 0x15) & (SR3_PE | SR3_EE), ==, SR3_EE);
    program(qts, 0x12, 0x1ff0000, 32, &zero, 1);
    g_assert_cmphex(rdsr(qts, 0x15) & (SR3_PE | SR3_EE), ==,
                    SR3_PE | SR3_EE);
    /* An accepted erase clears EE, a program PE. */
    erase(qts, 0x21, 0x1fe0000, 32);
    g_assert_cmphex(rdsr(qts, 0x15) & (SR3_PE | SR3_EE), ==, SR3_PE);
    step(qts, T_SE);
    /* BP4 = 1, BP3-0 = 1001: the bottom half; 1010 and up: all. */
    cmd(qts, 0x50);
    cmd_data(qts, 0x01, 0x64);
    program(qts, 0x02, 0xfffff0, 24, &zero, 1);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, 0);
    program(qts, 0x12, 0x1000000, 32, &zero, 1);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, SR1_WIP);
    step(qts, T_PP);
    cmd(qts, 0x50);
    cmd_data(qts, 0x01, 0x28);
    program(qts, 0x12, 0x1000000, 32, &zero, 1);
    g_assert_cmphex(rdsr(qts, 0x05) & SR1_WIP, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
static void test_security_registers(void)
{
    QTestState *qts = start(NULL);
    static const uint8_t data[2] = { 0x12, 0x34 };
    uint8_t got[2];

    /* 2048-byte registers: A11 is 0, the bytes in A10-A0. */
    program(qts, 0x42, 0x37ff, 24, data, 1);
    step(qts, T_PP);
    user(qts, 0x48, 0x37ff, 24, 8, NULL, 0, got, 2);
    g_assert_cmphex(got[0], ==, 0x12);
    g_assert_cmphex(got[1], ==, 0xff);
    erase(qts, 0x44, 0x3000, 24);
    step(qts, T_SE);
    user(qts, 0x48, 0x37ff, 24, 8, NULL, 0, got, 1);
    g_assert_cmphex(got[0], ==, 0xff);
    qtest_quit(qts);
}

/*
 * The boot ROM boots an image from the 32 MiB flash: it reads it through
 * SPI0 with 24-bit addresses, as the chip comes up in 3-byte mode.
 */
/* [spec:nuos:req:emu.esp32p4.flash-chip/test] */
static void test_rom_boot(void)
{
    g_autofree uint8_t *flash = g_malloc(32 * 1024 * 1024);
    g_autofree char *path = NULL;
    QTestState *qts;
    int i;

    memset(flash, 0xff, 32 * 1024 * 1024);
    put_image(flash, BOOTLOADER_OFFSET, prog_magic, 6, 0, false);
    path = write_tmp("esp32p4-gd-XXXXXX.img", flash, 32 * 1024 * 1024);
    qts = qtest_initf("-M esp32p4,flash-size=32M,strap-mode=0x08 "
                      "-accel tcg -icount shift=0,sleep=off "
                      "-drive file=%s,if=mtd,format=raw,snapshot=on", path);
    for (i = 0; i < 300 && qtest_readl(qts, MAGIC_ADDR) != MAGIC; i++) {
        g_usleep(100000);
    }
    g_assert_cmphex(qtest_readl(qts, MAGIC_ADDR), ==, MAGIC);
    qtest_quit(qts);
    unlink(path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/gd25q256e/ids", test_ids);
    qtest_add_func("/esp32p4/gd25q256e/status", test_status);
    qtest_add_func("/esp32p4/gd25q256e/addressing", test_addressing);
    qtest_add_func("/esp32p4/gd25q256e/times", test_times);
    qtest_add_func("/esp32p4/gd25q256e/suspend", test_suspend);
    qtest_add_func("/esp32p4/gd25q256e/protection", test_protection);
    qtest_add_func("/esp32p4/gd25q256e/security-registers",
                   test_security_registers);
    qtest_add_func("/esp32p4/gd25q256e/rom-boot", test_rom_boot);
    return g_test_run();
}
