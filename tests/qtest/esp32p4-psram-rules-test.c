/*
 * QTest testcase for the ESP32-P4's PSRAM die's timing and address rules
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The APS256XXN's CE# low time, tCEM (4 us at standard temperature, its
 * datasheet's table 30): a transaction holding it longer keeps the die
 * from its refresh, which is logged as a guest error. A memory access at
 * an odd address, against 7.1's even start rule, is logged, the die
 * starting at the even address before it.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "esp32p4-psram.h"
#include "qobject/qdict.h"

#define SPI1                ESP32P4_PSRAM_SPI1
#define CMD                 0x000
#define ADDR                0x004
#define CLOCK               0x014
#define USER                0x018
#define USER1               0x01c
#define USER2               0x020
#define MOSI_DLEN           0x024
#define MISO_DLEN           0x028
#define W(n)                (0x058 + 4 * (n))
#define CMD_USR             (1u << 18)
#define USR_MOSI            (1u << 27)
#define USR_MISO            (1u << 28)
#define USR_DUMMY           (1u << 29)
#define USR_ADDR            (1u << 30)
#define USR_COMMAND         (1u << 31)
#define PSRAM_SYNC_READ     0x0000
#define PSRAM_SYNC_WRITE    0x8080

static QTestState *qts;

static uint64_t core_period_ps(void)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/machine/clkrst/psram-core', "
                         "'property': 'qtest-clock-period' } }");
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    return (period * 1000) >> 32;
}

static void spi1_xfer(uint32_t cmd, uint32_t addr, int out_len, int in_len)
{
    uint32_t user = USR_COMMAND | USR_ADDR | USR_DUMMY;

    if (out_len) {
        user |= USR_MOSI;
        qtest_writel(qts, SPI1 + MOSI_DLEN, out_len * 8 - 1);
    }
    if (in_len) {
        user |= USR_MISO;
        qtest_writel(qts, SPI1 + MISO_DLEN, in_len * 8 - 1);
    }
    qtest_writel(qts, SPI1 + USER, user);
    qtest_writel(qts, SPI1 + USER1, 31u << 26 | (ESP32P4_PSRAM_PUP_DUMMY - 1));
    qtest_writel(qts, SPI1 + USER2, 15u << 28 | cmd);
    qtest_writel(qts, SPI1 + ADDR, addr);
    qtest_writel(qts, SPI1 + CMD, CMD_USR);
    for (int i = 0; i < 1000 && qtest_readl(qts, SPI1 + CMD); i++) {
        qtest_clock_step(qts, 10000);
    }
    g_assert_cmphex(qtest_readl(qts, SPI1 + CMD), ==, 0);
}

static int occurrences(const char *log, const char *what)
{
    g_autofree char *text = NULL;
    int n = 0;

    g_assert_true(g_file_get_contents(log, &text, NULL, NULL));
    for (const char *p = strstr(text, what); p; p = strstr(p + 1, what)) {
        n++;
    }
    return n;
}

/*
 * At about 5 MHz: a 4-byte read (9 clocks with the power-up dummy phase)
 * keeps to tCEM, a 64-byte one (39 clocks) does not.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_tcem(void)
{
    g_autofree char *log = NULL;
    int fd = g_file_open_tmp("esp32p4-psram-XXXXXX.log", &log, NULL);
    uint64_t div;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = qtest_initf("-M esp32p4,strap-mode=0 -d guest_errors -D %s", log);
    esp32p4_psram_spi1_opi(qts);
    div = DIV_ROUND_UP(200000, core_period_ps());
    g_assert_cmpuint(div, >=, 2);
    qtest_writel(qts, SPI1 + CLOCK, (uint32_t)(div - 1) << 16 |
                 (uint32_t)(div / 2 - 1) << 8 | (uint32_t)(div - 1));

    spi1_xfer(PSRAM_SYNC_READ, 0x100, 0, 4);
    g_assert_cmpint(occurrences(log, "longer than tCEM"), ==, 0);
    spi1_xfer(PSRAM_SYNC_READ, 0x100, 0, 64);
    g_assert_cmpint(occurrences(log, "longer than tCEM"), ==, 1);
    spi1_xfer(PSRAM_SYNC_WRITE, 0x100, 64, 0);
    g_assert_cmpint(occurrences(log, "longer than tCEM"), ==, 2);
    qtest_quit(qts);
    unlink(log);
}

/*
 * The die starts a memory access at an odd address at the even one before
 * it, and the hazard is logged; a mode register access may start at
 * either.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_odd_address(void)
{
    g_autofree char *log = NULL;
    int fd = g_file_open_tmp("esp32p4-psram-XXXXXX.log", &log, NULL);

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = qtest_initf("-M esp32p4,strap-mode=0 -d guest_errors -D %s", log);
    esp32p4_psram_spi1_opi(qts);
    esp32p4_psram_dqs_on(qts);
    qtest_writel(qts, SPI1 + W(0), 0x44332211);
    qtest_writel(qts, SPI1 + W(1), 0x88776655);
    spi1_xfer(PSRAM_SYNC_WRITE, 0x200, 8, 0);
    g_assert_cmpint(occurrences(log, "at odd PSRAM address"), ==, 0);

    spi1_xfer(PSRAM_SYNC_READ, 0x203, 0, 4);
    g_assert_cmphex(qtest_readl(qts, SPI1 + W(0)), ==, 0x66554433);
    g_assert_cmpint(occurrences(log, "SPI1's transaction at odd PSRAM "
                                "address 0x00000203"), ==, 1);
    qtest_quit(qts);
    unlink(log);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32p4/psram-rules/tcem", test_tcem);
    qtest_add_func("esp32p4/psram-rules/odd-address", test_odd_address);
    return g_test_run();
}
