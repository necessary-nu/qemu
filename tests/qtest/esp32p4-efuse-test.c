/*
 * QTest testcase for the ESP32-P4 eFuse controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "libqtest.h"

#define EFUSE_BASE              0x5012d000

#define PGM_DATA(n)             (0x000 + 4 * (n))
#define PGM_CHECK_VALUE(n)      (0x020 + 4 * (n))
#define RD_WR_DIS               0x02c
#define RD_REPEAT_DATA(n)       (0x030 + 4 * (n))
#define RD_MAC_SYS(n)           (0x044 + 4 * (n))
#define RD_SYS_PART1_DATA(n)    (0x05c + 4 * (n))
#define RD_USR_DATA(n)          (0x07c + 4 * (n))
#define RD_KEY_DATA(k, n)       (0x09c + 0x20 * (k) + 4 * (n))
#define RD_SYS_PART2_DATA(n)    (0x15c + 4 * (n))
#define RD_REPEAT_ERR(n)        (0x17c + 4 * (n))
#define ECDSA                   0x1b0
#define RD_RS_ERR0              0x1c0
#define RD_RS_ERR1              0x1c4
#define CLK                     0x1c8
#define CONF                    0x1cc
#define STATUS                  0x1d0
#define CMD                     0x1d4
#define INT_RAW                 0x1d8
#define INT_ST                  0x1dc
#define INT_ENA                 0x1e0
#define INT_CLR                 0x1e4
#define DAC_CONF                0x1e8
#define RD_TIM_CONF             0x1ec
#define WR_TIM_CONF1            0x1f0
#define WR_TIM_CONF2            0x1f4
#define WR_TIM_CONF0_RS_BYPASS  0x1f8
#define DATE                    0x1fc
#define APB2OTP_WR_DIS          0x800
#define APB2OTP_BLK0_BACKUP(k, n) (0x804 + 0x14 * (k) + 4 * (n))
#define APB2OTP_BLK1_W(n)       (0x854 + 4 * (n))
#define APB2OTP_BLK_W(b, n)     (0x878 + 0x2c * ((b) - 2) + 4 * (n))
#define APB2OTP_EN              0xa08

#define READ_OP_CODE            0x5aa5
#define PGM_OP_CODE             0x5a5a
#define CMD_READ                0x1
#define CMD_PGM                 0x2

#define STATE_RESET             0
#define STATE_IDLE              1
#define STATE_READ_INIT         2
#define STATE_READ_BLK0         3
#define STATE_READ_RS_BLK       5

#define EFUSE_PATH              "/machine/efuse"

/* At 20 MHz, with the reset timing: 15 + 128 * (1 + 2 + 1) + 1 cycles. */
#define CYCLE_NS                50
#define READ_NS                 (528 * CYCLE_NS)

/* BLOCK3 (user data) to program, and its Reed-Solomon parity. */
static const uint32_t usr_data[8] = {
    0x01020304, 0x05060708, 0x090a0b0c, 0x0d0e0f10,
    0x11121314, 0x15161718, 0x191a1b1c, 0x1d1e1f20,
};
static const uint32_t usr_check[3] = { 0xc57e0ad4, 0x5fda3237, 0xb24b85b9 };

static const uint32_t key_data[8] = {
    0x03020100, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
    0x13121110, 0x17161514, 0x1b1a1918, 0x1f1e1d1c,
};
static const uint32_t key_check[3] = { 0x0d474ca0, 0x03b2fc3f, 0x13f4e9da };

/* The default BLOCK1, BLOCK2 and BLOCK10, and their parity. */
static const uint32_t default_blk1[6] = {
    0xc4000001, 0x0000240a, 0x88044930, 0x64a0c664, 0x00980a0c, 0x00000100,
};
/* Its Reed-Solomon parity, as espefuse computes it. */
static const uint32_t default_blk1_check[3] = {
    0x39e3fbc8, 0xbad1df9f, 0x39a0b20e,
};
static const uint32_t default_blk2[8] = {
    0, 0, 0, 0, 0x1a466990, 0xf641866e, 0x641e6b99, 0x0003c140,
};
static const uint32_t default_blk10[8] = {
    0x0120581c, 0x10120103, 0x01201102, 0x00000002, 0, 0, 0, 0,
};

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, EFUSE_BASE + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t value)
{
    qtest_writel(qts, EFUSE_BASE + off, value);
}

static QTestState *start(const char *extra)
{
    return qtest_initf("-M esp32p4 %s", extra ? extra : "");
}

/* A read operation, run to its end. */
static void efuse_read(QTestState *qts)
{
    wr(qts, CONF, READ_OP_CODE);
    wr(qts, CMD, CMD_READ);
    qtest_clock_step(qts, READ_NS);
    g_assert_cmphex(rd(qts, CMD), ==, 0);
}

/* The cycles programming takes with the reset timing. */
static uint64_t pgm_cycles(unsigned ones, unsigned bits)
{
    return 0x2667 + ones * (1 + 0xa0 + 1) + (bits - ones) * 1 + 0x140;
}

/*
 * Program `block` with `data` (`n` words from EFUSE_PGM_DATA0) and
 * `check`, and run the operation to its end.
 */
static void efuse_program(QTestState *qts, unsigned block,
                          const uint32_t *data, unsigned n,
                          const uint32_t *check)
{
    unsigned i;

    for (i = 0; i < 8; i++) {
        wr(qts, PGM_DATA(i), i < n ? data[i] : 0);
    }
    for (i = 0; i < 3; i++) {
        wr(qts, PGM_CHECK_VALUE(i), check ? check[i] : 0);
    }
    wr(qts, CONF, PGM_OP_CODE);
    wr(qts, CMD, (block << 2) | CMD_PGM);
    /* The longest programming, every bit of BLOCK0, takes under 6 ms. */
    qtest_clock_step(qts, 6 * 1000 * 1000);
    g_assert_cmphex(rd(qts, CMD), ==, 0);
    for (i = 0; i < 8; i++) {
        wr(qts, PGM_DATA(i), 0);
    }
    for (i = 0; i < 3; i++) {
        wr(qts, PGM_CHECK_VALUE(i), 0);
    }
}

static void assert_words(QTestState *qts, uint32_t off,
                         const uint32_t *want, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        g_assert_cmphex(rd(qts, off + 4 * i), ==, want[i]);
    }
}

/* EFUSE_RD_RS_ERRn's 4 bits for `block` (1 to 10). */
static uint32_t rs_status(QTestState *qts, unsigned block)
{
    unsigned slot = block - 1;

    return (rd(qts, slot < 8 ? RD_RS_ERR0 : RD_RS_ERR1) >> (4 * (slot % 8)))
           & 0xf;
}

static const struct {
    uint32_t off;
    uint32_t reset;
    uint32_t rw;        /* bits a write sets and clears */
} rw_regs[] = {
    { PGM_DATA(0), 0, 0xffffffff },
    { PGM_DATA(1), 0, 0xffffffff },
    { PGM_DATA(2), 0, 0xffffffff },
    { PGM_DATA(3), 0, 0xffffffff },
    { PGM_DATA(4), 0, 0xffffffff },
    { PGM_DATA(5), 0, 0xffffffff },
    { PGM_DATA(6), 0, 0xffffffff },
    { PGM_DATA(7), 0, 0xffffffff },
    { PGM_CHECK_VALUE(0), 0, 0xffffffff },
    { PGM_CHECK_VALUE(1), 0, 0xffffffff },
    { PGM_CHECK_VALUE(2), 0, 0xffffffff },
    { CLK, 0, 0x00010007 },
    { CONF, 0, 0x0000ffff },
    { INT_ENA, 0, 0x00000003 },
    { DAC_CONF, 0x0001fe17, 0x0003ffff },
    { RD_TIM_CONF, 0x0f010201, 0xffffffff },
    { WR_TIM_CONF1, 0x01266701, 0xffffffff },
    { WR_TIM_CONF2, 0x00a00140, 0xffffffff },
    { WR_TIM_CONF0_RS_BYPASS, 0x00002000, 0x001fefff },
    { DATE, 0x02502190, 0x0fffffff },
    { APB2OTP_EN, 0, 0x00000001 },
};

/* The status registers, which writes leave alone. */
static const struct {
    uint32_t off;
    uint32_t reset;
} ro_regs[] = {
    { RD_REPEAT_ERR(0), 0 }, { RD_REPEAT_ERR(1), 0 }, { RD_REPEAT_ERR(2), 0 },
    { RD_REPEAT_ERR(3), 0 }, { RD_REPEAT_ERR(4), 0 },
    { RD_RS_ERR0, 0 }, { RD_RS_ERR1, 0 },
    { STATUS, STATE_IDLE }, { INT_ST, 0 },
};

/* Every register's reset value and how writes change it. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_registers(void)
{
    QTestState *qts = start(NULL);
    uint32_t off;
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(rw_regs); i++) {
        g_assert_cmphex(rd(qts, rw_regs[i].off), ==, rw_regs[i].reset);
        wr(qts, rw_regs[i].off, 0xffffffff);
        g_assert_cmphex(rd(qts, rw_regs[i].off), ==, rw_regs[i].rw);
        wr(qts, rw_regs[i].off, 0);
        g_assert_cmphex(rd(qts, rw_regs[i].off), ==, 0);
        wr(qts, rw_regs[i].off, rw_regs[i].reset);
    }
    for (i = 0; i < ARRAY_SIZE(ro_regs); i++) {
        g_assert_cmphex(rd(qts, ro_regs[i].off), ==, ro_regs[i].reset);
        wr(qts, ro_regs[i].off, 0xffffffff);
        g_assert_cmphex(rd(qts, ro_regs[i].off), ==, ro_regs[i].reset);
    }

    /* The RD_* registers: the defaults, which writes do not change. */
    for (off = RD_WR_DIS; off < RD_REPEAT_ERR(0); off += 4) {
        uint32_t v = rd(qts, off);

        wr(qts, off, ~v);
        g_assert_cmphex(rd(qts, off), ==, v);
    }

    /* EFUSE_CMD keeps EFUSE_BLK_NUM; the commands need an op code. */
    g_assert_cmphex(rd(qts, CMD), ==, 0);
    wr(qts, CMD, 0x3c);
    g_assert_cmphex(rd(qts, CMD), ==, 0x3c);
    wr(qts, CMD, 0);

    /* EFUSE_INT_CLR and EFUSE_UPDATE read as zero. */
    wr(qts, INT_CLR, 3);
    g_assert_cmphex(rd(qts, INT_CLR), ==, 0);
    wr(qts, WR_TIM_CONF0_RS_BYPASS, 0x00003000);
    g_assert_cmphex(rd(qts, WR_TIM_CONF0_RS_BYPASS), ==, 0x00002000);

    /* EFUSE_ECDSA_REG: the blocks in use follow the configured ones. */
    g_assert_cmphex(rd(qts, ECDSA), ==, 0);
    wr(qts, ECDSA, 0xffff5874);
    g_assert_cmphex(rd(qts, ECDSA), ==, 0x58745874);
    wr(qts, ECDSA, 0);

    /* APB2OTP reads nothing until enabled. */
    for (off = APB2OTP_WR_DIS; off <= APB2OTP_BLK_W(10, 10); off += 4) {
        g_assert_cmphex(rd(qts, off), ==, 0);
    }

    /* Unassigned offsets read as zero. */
    g_assert_cmphex(rd(qts, 0x190), ==, 0);
    g_assert_cmphex(rd(qts, 0x1b4), ==, 0);
    g_assert_cmphex(rd(qts, 0x200), ==, 0);
    g_assert_cmphex(rd(qts, 0xa04), ==, 0);
    g_assert_cmphex(rd(qts, 0xffc), ==, 0);

    /* Byte and halfword accesses reach the bytes they name. */
    wr(qts, PGM_DATA(3), 0x11223344);
    qtest_writeb(qts, EFUSE_BASE + PGM_DATA(3) + 2, 0xab);
    g_assert_cmphex(rd(qts, PGM_DATA(3)), ==, 0x11ab3344);
    qtest_writew(qts, EFUSE_BASE + PGM_DATA(3), 0xcdef);
    g_assert_cmphex(rd(qts, PGM_DATA(3)), ==, 0x11abcdef);
    g_assert_cmphex(qtest_readb(qts, EFUSE_BASE + RD_MAC_SYS(1) + 1), ==,
                    0x24);
    g_assert_cmphex(qtest_readw(qts, EFUSE_BASE + RD_MAC_SYS(0) + 2), ==,
                    0xc400);

    qtest_quit(qts);
}

/* The factory-fresh chip: revision, MAC and the rest of BLOCK1. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_defaults(void)
{
    QTestState *qts = start(NULL);
    uint32_t sys2, major, minor;
    unsigned b, w;

    /*
     * As ESP-IDF's efuse_ll reads the wafer version: v3.0, the revision of
     * the HP ROM the machine ships (ECO5).
     */
    sys2 = rd(qts, RD_MAC_SYS(2));
    major = ((sys2 >> 23) & 1) << 2 | ((sys2 >> 4) & 3);
    minor = sys2 & 0xf;
    g_assert_cmpuint(major * 100 + minor, ==, 300);
    g_assert_cmpuint((sys2 >> 6) & 3, ==, 0);         /* DISABLE_*_MAJOR */
    g_assert_cmpuint((sys2 >> 11) & 3, ==, 1);        /* BLK_VERSION_MAJOR */
    g_assert_cmpuint((sys2 >> 8) & 7, ==, 1);         /* BLK_VERSION_MINOR */
    g_assert_cmpuint((sys2 >> 20) & 7, ==, 0);        /* PKG_VERSION */
    g_assert_cmpuint((sys2 >> 13) & 7, ==, 2);        /* PSRAM_CAP */
    g_assert_cmpuint((sys2 >> 18) & 3, ==, 1);        /* PSRAM_VENDOR */

    /* MAC 24:0a:c4:00:00:01, in ESP-IDF's MAC_FACTORY order. */
    g_assert_cmphex(rd(qts, RD_MAC_SYS(0)), ==, 0xc4000001);
    g_assert_cmphex(rd(qts, RD_MAC_SYS(1)) & 0xffff, ==, 0x240a);

    assert_words(qts, RD_MAC_SYS(0), default_blk1, 6);
    assert_words(qts, RD_SYS_PART1_DATA(0), default_blk2, 8);
    assert_words(qts, RD_SYS_PART2_DATA(0), default_blk10, 8);

    /* Everything else is zero, and every block reads without error. */
    for (w = 0; w < 6; w++) {
        g_assert_cmphex(rd(qts, RD_WR_DIS + 4 * w), ==, 0);
    }
    for (b = 3; b < 10; b++) {
        for (w = 0; w < 8; w++) {
            g_assert_cmphex(rd(qts, RD_USR_DATA(0) + 0x20 * (b - 3) + 4 * w),
                            ==, 0);
        }
    }
    g_assert_cmphex(rd(qts, RD_RS_ERR0), ==, 0);
    g_assert_cmphex(rd(qts, RD_RS_ERR1), ==, 0);

    /* The stored parity is the parity of the data. */
    wr(qts, APB2OTP_EN, 1);
    assert_words(qts, APB2OTP_BLK1_W(0), default_blk1, 6);
    assert_words(qts, APB2OTP_BLK1_W(6), default_blk1_check, 3);

    qtest_quit(qts);
}

/* A read takes its time, and the RD_* registers change only at its end. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_read_timing(void)
{
    QTestState *qts = start(NULL);

    qtest_irq_intercept_out_named(qts, EFUSE_PATH, "sysbus-irq");

    /* Programmed, but not yet read. */
    efuse_program(qts, 3, usr_data, 8, usr_check);
    wr(qts, INT_CLR, 3);
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, 0);

    wr(qts, INT_ENA, 1);
    wr(qts, CONF, READ_OP_CODE);
    wr(qts, CMD, CMD_READ);
    g_assert_cmphex(rd(qts, CMD), ==, CMD_READ);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_READ_INIT);
    qtest_clock_step(qts, 15 * CYCLE_NS);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_READ_BLK0);
    qtest_clock_step(qts, 20 * 4 * CYCLE_NS + CYCLE_NS);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_READ_RS_BLK);

    qtest_clock_step(qts, READ_NS - 16 * CYCLE_NS - 20 * 4 * CYCLE_NS - 1);
    g_assert_cmphex(rd(qts, CMD), ==, CMD_READ);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, 0);

    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, CMD), ==, 0);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_IDLE);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 1);
    g_assert_cmphex(rd(qts, INT_ST), ==, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);

    /* EFUSE_INT_RAW clears by writing it, as EFUSE_INT_CLR does. */
    wr(qts, INT_RAW, 1);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* The read timing takes effect at EFUSE_UPDATE: init 0x40 cycles. */
    wr(qts, RD_TIM_CONF, 0x40010201);
    efuse_read(qts);
    wr(qts, WR_TIM_CONF0_RS_BYPASS, 0x00003000);
    wr(qts, CONF, READ_OP_CODE);
    wr(qts, CMD, CMD_READ);
    qtest_clock_step(qts, (0x40 + 512 + 1) * CYCLE_NS - 1);
    g_assert_cmphex(rd(qts, CMD), ==, CMD_READ);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, CMD), ==, 0);

    qtest_quit(qts);
}

/* Programming: timing, and what a command needs. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_program(void)
{
    QTestState *qts = start(NULL);
    unsigned ones = 0, i;
    uint64_t ns;

    qtest_irq_intercept_out_named(qts, EFUSE_PATH, "sysbus-irq");

    /* A command with the wrong op code does nothing. */
    wr(qts, PGM_DATA(0), 0xffffffff);
    wr(qts, CONF, READ_OP_CODE);
    wr(qts, CMD, (3 << 2) | CMD_PGM);
    g_assert_cmphex(rd(qts, CMD), ==, 3 << 2);
    wr(qts, CONF, 0x1234);
    wr(qts, CMD, CMD_READ);
    g_assert_cmphex(rd(qts, CMD), ==, 0);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, 0);
    wr(qts, INT_CLR, 3);

    /* BLOCK3 takes PWR_ON + a pulse per one + inactive per zero + OFF. */
    for (i = 0; i < 8; i++) {
        wr(qts, PGM_DATA(i), usr_data[i]);
        ones += ctpop32(usr_data[i]);
    }
    for (i = 0; i < 3; i++) {
        wr(qts, PGM_CHECK_VALUE(i), usr_check[i]);
        ones += ctpop32(usr_check[i]);
    }
    ns = pgm_cycles(ones, 352) * CYCLE_NS;
    wr(qts, INT_ENA, 2);
    wr(qts, CONF, PGM_OP_CODE);
    wr(qts, CMD, (3 << 2) | CMD_PGM);
    g_assert_cmphex(rd(qts, CMD), ==, (3 << 2) | CMD_PGM);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, 6);

    /* A second command while one runs is ignored. */
    wr(qts, CONF, READ_OP_CODE);
    wr(qts, CMD, CMD_READ);
    g_assert_cmphex(rd(qts, CMD), ==, (3 << 2) | CMD_PGM);

    qtest_clock_step(qts, ns - 1);
    g_assert_cmphex(rd(qts, CMD), ==, (3 << 2) | CMD_PGM);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, CMD), ==, 0);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 2);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(qts, INT_CLR, 2);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Read back after a read operation, without error. */
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, 0);
    efuse_read(qts);
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);
    g_assert_cmphex(rs_status(qts, 3), ==, 0);

    /* BLOCK0 programs each of its copies; bits only go to one. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0, 0, 0x00000200, 0 },
                  6, NULL);
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0, 0, 0x00000400, 0 },
                  6, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(3)), ==, 0x00000600);
    wr(qts, APB2OTP_EN, 1);
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(i, 3)), ==, 0x00000600);
    }
    for (i = 0; i < 5; i++) {
        g_assert_cmphex(rd(qts, RD_REPEAT_ERR(i)), ==, 0);
    }

    /* The 24 bits of BLOCK0's last word are all there is. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0, 0, 0, 0xffffffff },
                  6, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(4)), ==, 0x00ffffff);

    qtest_quit(qts);
}

/* Reed-Solomon: the parity software gives, and what a read finds. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_rs(void)
{
    QTestState *qts = start(NULL);

    /* Data without its parity: four bytes off the blank codeword. */
    efuse_program(qts, 5, (const uint32_t[]){ 0xdeadbeef }, 1, NULL);
    efuse_read(qts);
    g_assert_cmphex(rs_status(qts, 5), ==, 4);
    g_assert_cmphex(rd(qts, RD_KEY_DATA(1, 0)), ==, 0);

    /* Too far from any codeword: the fail bit, and the data as stored. */
    efuse_program(qts, 6, usr_data, 8, NULL);
    efuse_read(qts);
    g_assert_cmphex(rs_status(qts, 6), ==, 8);
    assert_words(qts, RD_KEY_DATA(2, 0), usr_data, 8);

    /* Two codewords ORed: a coded block takes one programming. */
    efuse_program(qts, 7, (const uint32_t[]){ 0x11111111 }, 1,
                  (const uint32_t[]){ 0x33abb410, 0x3137e0c7, 0x436c8ebc });
    efuse_read(qts);
    g_assert_cmphex(rs_status(qts, 7), ==, 0);
    g_assert_cmphex(rd(qts, RD_KEY_DATA(3, 0)), ==, 0x11111111);
    efuse_program(qts, 7, (const uint32_t[]){ 0, 0x22222222 }, 2,
                  (const uint32_t[]){ 0x8c4fded8, 0x901faf09, 0xee443375 });
    efuse_read(qts);
    g_assert_cmphex(rs_status(qts, 7), ==, 8);

    /* The other blocks' records are untouched. */
    g_assert_cmphex(rs_status(qts, 1), ==, 0);
    g_assert_cmphex(rs_status(qts, 4), ==, 0);
    g_assert_cmphex(rs_status(qts, 10), ==, 0);

    /* BLOCK1 is coded with its two missing words as zero. */
    g_assert_cmphex(rs_status(qts, 1), ==, 0);

    qtest_quit(qts);
}

/* EFUSE_WR_DIS: whole blocks, and BLOCK0's fields bit by bit. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_write_protection(void)
{
    QTestState *qts = start(NULL);

    /* Protect BLOCK3 (bit 22) and KEY_PURPOSE_0 (bit 8). */
    efuse_program(qts, 0, (const uint32_t[]){ BIT(22) | BIT(8) }, 1, NULL);

    /* Until a read loads EFUSE_WR_DIS, it does not protect. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0x01000000 }, 3, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_WR_DIS), ==, BIT(22) | BIT(8));
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(1)) >> 24, ==, 0x1);

    /* Now KEY_PURPOSE_0 stays; KEY_PURPOSE_1 (bit 9) does not. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0x22000000 }, 3, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(1)) >> 24, ==, 0x21);

    /* KEY_PURPOSE_0's fifth bit is protected with it. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0, 0, BIT(27) | BIT(28) },
                  5, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(3)), ==, BIT(28));

    /* BLOCK3 cannot be programmed at all. */
    efuse_program(qts, 3, usr_data, 8, usr_check);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, 0);
    g_assert_cmphex(rs_status(qts, 3), ==, 0);

    /* Unprotected fields and EFUSE_WR_DIS itself still program. */
    efuse_program(qts, 0, (const uint32_t[]){ BIT(0), BIT(9) }, 2, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_WR_DIS), ==, BIT(22) | BIT(8) | BIT(0));
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(0)), ==, BIT(9));

    /* EFUSE_WR_DIS bit 0 protects EFUSE_RD_DIS. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0x7f }, 2, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(0)), ==, BIT(9));

    qtest_quit(qts);
}

/* EFUSE_RD_DIS, and the key purposes. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_read_protection(void)
{
    QTestState *qts = start(NULL);

    /* KEY0 with a key, KEY_PURPOSE_0 HMAC upstream (8), then RD_DIS. */
    efuse_program(qts, 4, key_data, 8, key_check);
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0x08000000 }, 3, NULL);
    efuse_read(qts);
    assert_words(qts, RD_KEY_DATA(0, 0), key_data, 8);
    g_assert_cmphex((rd(qts, RD_REPEAT_DATA(1)) >> 24) & 0xf, ==, 8);

    efuse_program(qts, 0, (const uint32_t[]){ 0, BIT(0) }, 2, NULL);
    assert_words(qts, RD_KEY_DATA(0, 0), key_data, 8);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(0)) & 0x7f, ==, 1);
    for (unsigned i = 0; i < 8; i++) {
        g_assert_cmphex(rd(qts, RD_KEY_DATA(0, i)), ==, 0);
    }
    g_assert_cmphex(rs_status(qts, 4), ==, 0);

    /* Nor through APB2OTP, while the other key blocks still read. */
    wr(qts, APB2OTP_EN, 1);
    for (unsigned i = 0; i < 11; i++) {
        g_assert_cmphex(rd(qts, APB2OTP_BLK_W(4, i)), ==, 0);
    }

    /* BLOCK10's RD_DIS bit hides the calibration it holds. */
    efuse_program(qts, 0, (const uint32_t[]){ 0, BIT(6) }, 2, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_SYS_PART2_DATA(0)), ==, 0);
    g_assert_cmphex(rd(qts, RD_SYS_PART1_DATA(4)), ==, default_blk2[4]);

    /* The 5-bit purposes: KEY_PURPOSE_5 = 18 (ECDSA P-384 high). */
    efuse_program(qts, 0, (const uint32_t[]){ 0, 0, 0, 0x2000, 0, BIT(4) },
                  6, NULL);
    efuse_read(qts);
    g_assert_cmphex((rd(qts, RD_REPEAT_DATA(2)) >> 12) & 0xf, ==, 2);
    g_assert_cmphex((rd(qts, RD_REPEAT_DATA(4)) >> 4) & 1, ==, 1);

    qtest_quit(qts);
}

/* APB2OTP: the array as stored, parity and copies. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_apb2otp(void)
{
    QTestState *qts = start(NULL);
    unsigned i;

    efuse_program(qts, 3, usr_data, 8, usr_check);
    efuse_program(qts, 0, (const uint32_t[]){ BIT(5), 0x80, 0, 0, 0, 1 },
                  6, NULL);
    wr(qts, APB2OTP_EN, 1);
    g_assert_cmphex(rd(qts, APB2OTP_WR_DIS), ==, BIT(5));
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(i, 0)), ==, 0x80);
        g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(i, 4)), ==, 1);
    }
    assert_words(qts, APB2OTP_BLK_W(3, 0), usr_data, 8);
    assert_words(qts, APB2OTP_BLK_W(3, 8), usr_check, 3);
    g_assert_cmphex(rd(qts, APB2OTP_BLK_W(10, 0)), ==, default_blk10[0]);

    /* The RD_* registers still wait for a read. */
    g_assert_cmphex(rd(qts, RD_WR_DIS), ==, 0);
    wr(qts, APB2OTP_EN, 0);
    g_assert_cmphex(rd(qts, APB2OTP_WR_DIS), ==, 0);

    qtest_quit(qts);
}

/* The "reset" input holds the controller; released, it loads the array. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_reset_input(void)
{
    QTestState *qts = start(NULL);

    efuse_program(qts, 3, usr_data, 8, usr_check);
    wr(qts, INT_ENA, 3);
    wr(qts, CONF, PGM_OP_CODE);

    qtest_set_irq_in(qts, EFUSE_PATH, "reset", 0, 1);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_RESET);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0);
    g_assert_cmphex(rd(qts, CONF), ==, 0);
    g_assert_cmphex(rd(qts, RD_MAC_SYS(0)), ==, 0);
    wr(qts, CONF, PGM_OP_CODE);
    g_assert_cmphex(rd(qts, CONF), ==, 0);

    qtest_set_irq_in(qts, EFUSE_PATH, "reset", 0, 0);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_READ_INIT);
    qtest_clock_step(qts, READ_NS - 1);
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_IDLE);
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);
    assert_words(qts, RD_MAC_SYS(0), default_blk1, 6);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, CMD), ==, 0);

    /* A system reset loads it at once, the array kept. */
    qtest_system_reset(qts);
    g_assert_cmpuint(rd(qts, STATUS) & 0xf, ==, STATE_IDLE);
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);

    qtest_quit(qts);
}

static char *make_file(size_t size, const void *contents, size_t len)
{
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("esp32p4-efuse-XXXXXX.bin", &path, &err);

    g_assert_no_error(err);
    g_assert_cmpint(ftruncate(fd, size), ==, 0);
    if (len) {
        g_assert_cmpint(pwrite(fd, contents, len, 0), ==, len);
    }
    close(fd);
    return path;
}

static QTestState *start_with_file(const char *path)
{
    g_autofree char *args = g_strdup_printf(
        "-drive file=%s,if=none,format=raw,id=efuse "
        "-global driver=nvram.esp32p4.efuse,property=drive,value=efuse",
        path);

    return start(args);
}

/* A file keeps what is programmed from one run to the next. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_persistence(void)
{
    g_autofree char *path = make_file(1024, NULL, 0);
    g_autofree char *contents = NULL;
    QTestState *qts;
    gsize len;

    /* A blank file starts as a factory-fresh chip, and is written so. */
    qts = start_with_file(path);
    assert_words(qts, RD_MAC_SYS(0), default_blk1, 6);
    g_assert(g_file_get_contents(path, &contents, &len, NULL));
    g_assert_cmpuint(len, ==, 1024);
    g_assert_cmphex((uint32_t)ldl_le_p(contents + 24), ==, default_blk1[0]);
    g_assert_cmphex((uint32_t)ldl_le_p(contents + 32), ==, default_blk1[2]);

    efuse_program(qts, 3, usr_data, 8, usr_check);
    efuse_program(qts, 0, (const uint32_t[]){ BIT(22) }, 1, NULL);
    /* A coded block's bad parity is kept too. */
    efuse_program(qts, 5, (const uint32_t[]){ 0xdeadbeef }, 1, NULL);
    qtest_quit(qts);

    qts = start_with_file(path);
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);
    g_assert_cmphex(rd(qts, RD_WR_DIS), ==, BIT(22));
    g_assert_cmphex(rs_status(qts, 3), ==, 0);
    g_assert_cmphex(rs_status(qts, 5), ==, 4);
    assert_words(qts, RD_MAC_SYS(0), default_blk1, 6);

    /* Protection persists with the bits. */
    efuse_program(qts, 3, (const uint32_t[]){ 0xffffffff }, 1, NULL);
    efuse_read(qts);
    g_assert_cmphex(rd(qts, RD_USR_DATA(0)), ==, usr_data[0]);
    qtest_quit(qts);

    unlink(path);
}

/*
 * Files laid out as Espressif's QEMU and esp-emu lay them out, data only,
 * and errors recorded in the part of the file past it.
 */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_file_format(void)
{
    uint32_t img[129] = { 0 };
    g_autofree char *small = NULL;
    g_autofree char *tiny = NULL;
    g_autofree char *full = NULL;
    char *contents;
    QTestState *qts;
    gsize len;
    unsigned i;

    /* 336 bytes: BLOCK0 then each block's data, no parity, read clean. */
    img[1] = cpu_to_le32(0x00001000);                 /* DIS_FORCE_DOWNLOAD */
    for (i = 0; i < 6; i++) {
        img[6 + i] = cpu_to_le32(default_blk1[i]);
    }
    for (i = 0; i < 8; i++) {
        img[20 + i] = cpu_to_le32(usr_data[i]);
    }
    small = make_file(336, img, 336);
    qts = start_with_file(small);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(0)), ==, 0x00001000);
    assert_words(qts, RD_MAC_SYS(0), default_blk1, 6);
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);
    g_assert_cmphex(rd(qts, RD_RS_ERR0), ==, 0);
    wr(qts, APB2OTP_EN, 1);
    assert_words(qts, APB2OTP_BLK_W(3, 8), usr_check, 3);

    /* Programming writes the data back, at the size it was. */
    efuse_program(qts, 0, (const uint32_t[]){ BIT(3) }, 1, NULL);
    qtest_quit(qts);
    g_assert(g_file_get_contents(small, &contents, &len, NULL));
    g_assert_cmpuint(len, ==, 336);
    g_assert_cmphex((uint32_t)ldl_le_p(contents), ==, BIT(3));
    g_assert_cmphex((uint32_t)ldl_le_p(contents + 4), ==, 0x00001000);
    g_free(contents);

    /* A blank file shorter than the data starts factory-fresh, and grows. */
    tiny = make_file(100, NULL, 0);
    qts = start_with_file(tiny);
    assert_words(qts, RD_MAC_SYS(0), default_blk1, 6);
    qtest_quit(qts);
    g_assert(g_file_get_contents(tiny, &contents, &len, NULL));
    g_assert_cmpuint(len, ==, 336);
    g_assert_cmphex((uint32_t)ldl_le_p(contents + 24), ==, default_blk1[0]);
    g_free(contents);

    /*
     * 1 KiB with errors: BLOCK0's third copy lacks a bit and has another
     * (words 84 + 5 * 1 + n), and BLOCK3's parity has a bad byte (words
     * 99 + 3 * 2 + n).
     */
    img[84 + 5 + 0] = cpu_to_le32(0x00001001);
    img[99 + 3 * 2 + 1] = cpu_to_le32(0x00ff0000);
    full = make_file(1024, img, sizeof(img));
    qts = start_with_file(full);
    g_assert_cmphex(rd(qts, RD_REPEAT_DATA(0)), ==, 0x00001001);
    g_assert_cmphex(rd(qts, RD_REPEAT_ERR(0)), ==, 0x00001001);
    assert_words(qts, RD_USR_DATA(0), usr_data, 8);
    g_assert_cmphex(rs_status(qts, 3), ==, 1);
    wr(qts, APB2OTP_EN, 1);
    g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(0, 0)), ==, 0x00001000);
    g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(1, 0)), ==, 0x00001000);
    g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(2, 0)), ==, 0x00000001);
    g_assert_cmphex(rd(qts, APB2OTP_BLK0_BACKUP(3, 0)), ==, 0x00001000);
    g_assert_cmphex(rd(qts, APB2OTP_BLK_W(3, 9)), ==, usr_check[1] ^
                    0x00ff0000);
    qtest_quit(qts);

    unlink(small);
    unlink(tiny);
    unlink(full);
}

/* An empty file is refused. */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_file_empty(void)
{
    g_autofree char *path = make_file(0, NULL, 0);
    g_autofree char *cmd = g_strdup_printf(
        "%s -M esp32p4 -display none -serial none -monitor none "
        "-drive file=%s,if=none,format=raw,id=efuse "
        "-global driver=nvram.esp32p4.efuse,property=drive,value=efuse",
        qtest_qemu_binary(NULL), path);
    g_auto(GStrv) argv = NULL;
    g_autofree char *err = NULL;
    GError *gerr = NULL;
    int status;

    g_assert(g_shell_parse_argv(cmd, NULL, &argv, &gerr));
    g_assert(g_spawn_sync(NULL, argv, NULL, G_SPAWN_STDOUT_TO_DEV_NULL, NULL,
                          NULL, NULL, &err, &status, &gerr));
    g_assert_no_error(gerr);
    g_assert_cmpint(status, !=, 0);
    g_assert(strstr(err, "eFuse file is empty"));
    unlink(path);
}

/* The chip revision ESP-IDF's efuse_ll reads from the eFuses. */
static uint32_t chip_revision(QTestState *qts)
{
    uint32_t sys2 = rd(qts, RD_MAC_SYS(2));

    return (((sys2 >> 23) & 1) << 2 | ((sys2 >> 4) & 3)) * 100 +
           (sys2 & 0xf);
}

/*
 * The factory-fresh eFuses present the revision of the HP ROM image that
 * runs, by the ECO version at its offset 0x14: a ROM image (-bios) of
 * ECO6, v3.1's, gives v3.1, ECO7 v3.2, and an ECO the machine does not
 * know v3.0.
 */
/* [spec:nuos:req:emu.esp32p4.efuse/test] */
static void test_rom_revision(void)
{
    static const struct {
        uint32_t eco;
        uint32_t revision;
    } roms[] = { { 5, 300 }, { 6, 301 }, { 7, 302 }, { 9, 300 } };

    for (int i = 0; i < ARRAY_SIZE(roms); i++) {
        g_autofree uint8_t *rom = g_malloc0(128 * 1024);
        g_autofree char *path = NULL;
        GError *err = NULL;
        QTestState *qts;
        int fd = g_file_open_tmp("esp32p4-rom-XXXXXX.bin", &path, &err);

        g_assert_no_error(err);
        stl_le_p(rom + 0x10, 18);       /* the ESP32-P4's chip ID */
        stl_le_p(rom + 0x14, roms[i].eco);
        g_assert_cmpint(write(fd, rom, 128 * 1024), ==, 128 * 1024);
        close(fd);
        qts = qtest_initf("-M esp32p4,strap-mode=0 -bios %s", path);
        g_assert_cmpuint(chip_revision(qts), ==, roms[i].revision);
        g_assert_cmphex(qtest_readl(qts, 0x4fc00014), ==, roms[i].eco);
        qtest_quit(qts);
        unlink(path);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/efuse/registers", test_registers);
    qtest_add_func("/esp32p4/efuse/defaults", test_defaults);
    qtest_add_func("/esp32p4/efuse/rom-revision", test_rom_revision);
    qtest_add_func("/esp32p4/efuse/read-timing", test_read_timing);
    qtest_add_func("/esp32p4/efuse/program", test_program);
    qtest_add_func("/esp32p4/efuse/reed-solomon", test_rs);
    qtest_add_func("/esp32p4/efuse/write-protection", test_write_protection);
    qtest_add_func("/esp32p4/efuse/read-protection", test_read_protection);
    qtest_add_func("/esp32p4/efuse/apb2otp", test_apb2otp);
    qtest_add_func("/esp32p4/efuse/reset-input", test_reset_input);
    qtest_add_func("/esp32p4/efuse/persistence", test_persistence);
    qtest_add_func("/esp32p4/efuse/file-format", test_file_format);
    qtest_add_func("/esp32p4/efuse/file-empty", test_file_empty);

    return g_test_run();
}
