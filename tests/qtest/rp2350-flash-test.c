/*
 * QTest testcase for the RP2350 boards' flash chips, the W25Q32RV and
 * W25Q16JV, through the QMI's direct mode
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Virtual time only moves when the test steps it, so every busy time is
 * checked to the nanosecond: a cycle starts when the test raises /CS.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "libqtest.h"

#define XIP_BASE            0x10000000
#define XIP_UNCACHED        0x14000000

#define QMI_BASE            0x400d0000
#define DIRECT_CSR          (QMI_BASE + 0x00)
#define DIRECT_TX           (QMI_BASE + 0x04)
#define DIRECT_RX           (QMI_BASE + 0x08)

#define CSR_EN              (1u << 0)
#define CSR_ASSERT_CS0N     (1u << 2)
#define CSR_RXEMPTY         (1u << 16)
#define CSR_CLKDIV(d)       ((d) << 22)

#define SR1_BUSY            0x01
#define SR1_WEL             0x02
#define SR2_QE              0x02
#define SR2_SUS             0x80

#define US                  1000LL
#define MS                  1000000LL
#define S                   1000000000LL

#define SECTOR              4096
#define PAGE                256

/* The W25Q32RV's typical times. */
#define RV_TW               (1500 * US)
#define RV_TPP              (250 * US)
#define RV_TSE              (30 * MS)
#define RV_TBE32            (80 * MS)
#define RV_TBE64            (120 * MS)
#define RV_TCE              (6 * S)
#define TSUS                (20 * US)

static QTestState *start(const char *opts)
{
    QTestState *qts = qtest_initf("-M rp2350,%s", opts);

    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    return qts;
}

static void cs_low(QTestState *qts)
{
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN | CSR_ASSERT_CS0N);
}

static void cs_high(QTestState *qts)
{
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
}

static uint8_t xfer(QTestState *qts, uint8_t tx)
{
    qtest_writel(qts, DIRECT_TX, tx);
    g_assert_false(qtest_readl(qts, DIRECT_CSR) & CSR_RXEMPTY);
    return qtest_readl(qts, DIRECT_RX);
}

static void cmd(QTestState *qts, uint8_t op)
{
    cs_low(qts);
    xfer(qts, op);
    cs_high(qts);
}

static void cmd_addr(QTestState *qts, uint8_t op, uint32_t addr)
{
    cs_low(qts);
    xfer(qts, op);
    xfer(qts, addr >> 16);
    xfer(qts, addr >> 8);
    xfer(qts, addr);
    cs_high(qts);
}

static uint8_t rdsr(QTestState *qts, uint8_t op)
{
    uint8_t v;

    cs_low(qts);
    xfer(qts, op);
    v = xfer(qts, 0);
    cs_high(qts);
    return v;
}

static uint8_t sr1(QTestState *qts)
{
    return rdsr(qts, 0x05);
}

static void wrsr(QTestState *qts, bool vol, uint8_t op, uint8_t v)
{
    cmd(qts, vol ? 0x50 : 0x06);
    cs_low(qts);
    xfer(qts, op);
    xfer(qts, v);
    cs_high(qts);
}

static void read_data(QTestState *qts, uint32_t addr, uint8_t *buf,
                      unsigned len)
{
    unsigned i;

    cs_low(qts);
    xfer(qts, 0x03);
    xfer(qts, addr >> 16);
    xfer(qts, addr >> 8);
    xfer(qts, addr);
    for (i = 0; i < len; i++) {
        buf[i] = xfer(qts, 0);
    }
    cs_high(qts);
}

static uint8_t read_byte(QTestState *qts, uint32_t addr)
{
    uint8_t b;

    read_data(qts, addr, &b, 1);
    return b;
}

static void program(QTestState *qts, uint32_t addr, const uint8_t *data,
                    unsigned len)
{
    unsigned i;

    cmd(qts, 0x06);
    cs_low(qts);
    xfer(qts, 0x02);
    xfer(qts, addr >> 16);
    xfer(qts, addr >> 8);
    xfer(qts, addr);
    for (i = 0; i < len; i++) {
        xfer(qts, data[i]);
    }
    cs_high(qts);
}

static void erase(QTestState *qts, uint8_t op, uint32_t addr)
{
    cmd(qts, 0x06);
    cmd_addr(qts, op, addr);
}

/*
 * BUSY stays set until exactly `t` ns after now, the cycle's start: the
 * test steps to 1 ns before, then onto the end.
 */
static void expect_busy_for(QTestState *qts, int64_t t)
{
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, t - 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
}

/* The model's rule for cells a cycle has done (m25p80_chip.c). */
static uint32_t cell_hash(uint32_t h)
{
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}

static uint8_t done_mask(uint32_t addr, int64_t done, int64_t total)
{
    uint32_t thr = ((uint64_t)done << 32) / total;
    uint8_t m = 0;
    int b;

    for (b = 0; b < 8; b++) {
        if (cell_hash(addr * 8 + b) < thr) {
            m |= 1 << b;
        }
    }
    return m;
}

static uint8_t erase_partial(uint32_t addr, uint8_t old, int64_t done,
                             int64_t total)
{
    return old | done_mask(addr, done, total);
}

static uint8_t program_partial(uint32_t addr, uint8_t old, uint8_t data,
                               int64_t done, int64_t total)
{
    return old & ~(done_mask(addr, done, total) & ~data);
}

static uint8_t pattern(uint32_t addr)
{
    return (addr * 7 + (addr >> 8) * 13) ^ 0x5a;
}

/* Program [addr, addr + len) with the pattern, waiting for each page. */
static void fill(QTestState *qts, uint32_t addr, uint32_t len)
{
    uint8_t page[PAGE];
    uint32_t a;
    unsigned i;

    for (a = addr; a < addr + len; a += PAGE) {
        for (i = 0; i < PAGE; i++) {
            page[i] = pattern(a + i);
        }
        program(qts, a, page, PAGE);
        qtest_clock_step(qts, RV_TPP);
        g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    }
}

/* [spec:nuos:req:emu.flash-chip/test] */
static void test_ids(void)
{
    QTestState *qts = start("flash-size=4M");
    uint8_t uid[8];
    int i;

    /* W25Q32RV: EF 40 16, device ID 15h. */
    cs_low(qts);
    xfer(qts, 0x9f);
    g_assert_cmphex(xfer(qts, 0), ==, 0xef);
    g_assert_cmphex(xfer(qts, 0), ==, 0x40);
    g_assert_cmphex(xfer(qts, 0), ==, 0x16);
    cs_high(qts);

    cs_low(qts);
    xfer(qts, 0x90);
    xfer(qts, 0);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0xef);
    g_assert_cmphex(xfer(qts, 0), ==, 0x15);
    g_assert_cmphex(xfer(qts, 0), ==, 0xef);
    cs_high(qts);

    cs_low(qts);
    xfer(qts, 0xab);
    xfer(qts, 0);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0x15);
    g_assert_cmphex(xfer(qts, 0), ==, 0x15);
    cs_high(qts);

    /* 4Bh: four dummy bytes, then the 64-bit unique ID. */
    cs_low(qts);
    xfer(qts, 0x4b);
    for (i = 0; i < 4; i++) {
        xfer(qts, 0);
    }
    for (i = 0; i < 8; i++) {
        uid[i] = xfer(qts, 0);
    }
    cs_high(qts);
    g_assert_cmphex(uid[0], ==, 0xe4);
    g_assert_cmphex(uid[7], ==, 0x2b);

    /* Quad I/O device ID (94h): QE is set on the JQ part. */
    cs_low(qts);
    xfer(qts, 0x94);
    for (i = 0; i < 6; i++) {
        xfer(qts, 0);
    }
    g_assert_cmphex(xfer(qts, 0), ==, 0xef);
    g_assert_cmphex(xfer(qts, 0), ==, 0x15);
    cs_high(qts);
    qtest_quit(qts);

    /* W25Q16JV: EF 40 15, device ID 14h; its own unique ID. */
    qts = start("flash-size=2M");
    cs_low(qts);
    xfer(qts, 0x9f);
    g_assert_cmphex(xfer(qts, 0), ==, 0xef);
    g_assert_cmphex(xfer(qts, 0), ==, 0x40);
    g_assert_cmphex(xfer(qts, 0), ==, 0x15);
    cs_high(qts);
    cs_low(qts);
    xfer(qts, 0xab);
    xfer(qts, 0);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0x14);
    cs_high(qts);
    qtest_quit(qts);

    /* The unique ID is a property. */
    qts = start("flash-size=4M -global w25q32rv.unique-id=0123456789abcdef");
    cs_low(qts);
    xfer(qts, 0x4b);
    for (i = 0; i < 4; i++) {
        xfer(qts, 0);
    }
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(xfer(qts, 0), ==, 0x01 + 0x22 * i);
    }
    cs_high(qts);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-chip/test] */
/* [spec:nuos:req:emu.flash-timing/test] */
static void test_status(void)
{
    QTestState *qts = start("flash-size=4M");

    /* W25Q32RV-JQ: QE and LB0 set, DRV 50 ohm. */
    g_assert_cmphex(rdsr(qts, 0x05), ==, 0x00);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x06);
    g_assert_cmphex(rdsr(qts, 0x15), ==, 0x40);

    cmd(qts, 0x06);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    cmd(qts, 0x04);
    g_assert_cmphex(sr1(qts), ==, 0);

    /* A volatile write takes no time and leaves WEL clear. */
    wrsr(qts, true, 0x01, 0x04);
    g_assert_cmphex(sr1(qts), ==, 0x04);

    /* A non-volatile write takes tW, then clears WEL. */
    wrsr(qts, false, 0x01, 0x08);
    g_assert_cmphex(sr1(qts), ==, 0x04 | SR1_WEL | SR1_BUSY);
    qtest_clock_step(qts, RV_TW - 1);
    g_assert_cmphex(sr1(qts), ==, 0x04 | SR1_WEL | SR1_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(sr1(qts), ==, 0x08);

    /* Read-only bits ignore writes; LB0 stays set. */
    wrsr(qts, true, 0x31, 0x00);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x04);
    wrsr(qts, true, 0x01, 0x03);
    g_assert_cmphex(sr1(qts), ==, 0x00);

    /* A power cycle restores the stored bits. */
    qtest_system_reset(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    g_assert_cmphex(sr1(qts), ==, 0x08);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x06);
    wrsr(qts, false, 0x01, 0x00);
    qtest_clock_step(qts, RV_TW);

    /* A status register write clocked past its data byte: not executed. */
    cmd(qts, 0x06);
    cs_low(qts);
    xfer(qts, 0x31);
    xfer(qts, 0x40);
    xfer(qts, 0x00);
    cs_high(qts);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x06);
    cmd(qts, 0x04);

    /* 01h with two bytes writes SR1 and SR2. */
    cmd(qts, 0x50);
    cs_low(qts);
    xfer(qts, 0x01);
    xfer(qts, 0x24);
    xfer(qts, 0x42);
    cs_high(qts);
    g_assert_cmphex(sr1(qts), ==, 0x24);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x46);
    qtest_quit(qts);

    /* W25Q16JV-IQ: QE fixed at 1, DRV 25%. */
    qts = start("flash-size=2M");
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x02);
    g_assert_cmphex(rdsr(qts, 0x15), ==, 0x60);
    wrsr(qts, true, 0x31, 0x00);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x02);
    wrsr(qts, false, 0x01, 0x04);
    expect_busy_for(qts, 10 * MS);
    g_assert_cmphex(sr1(qts), ==, 0x04);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-timing/test] */
static void test_program_erase_times(void)
{
    QTestState *qts = start("flash-size=4M");
    static const uint8_t data[4] = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t buf[4];

    program(qts, 0x1000, data, 4);
    g_assert_cmphex(sr1(qts), ==, SR1_BUSY | SR1_WEL);

    /*
     * While BUSY, the chip ignores reads and IDs; nothing drives the bus.
     * The array keeps its old contents until the program completes.
     */
    g_assert_cmphex(read_byte(qts, 0x1000), ==, 0);
    cs_low(qts);
    xfer(qts, 0x9f);
    g_assert_cmphex(xfer(qts, 0), ==, 0);
    cs_high(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6));
    g_assert_cmphex(qtest_readl(qts, XIP_UNCACHED + 0x1000), ==, 0);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0x8000), ==, 0);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);

    qtest_clock_step(qts, RV_TPP - 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(sr1(qts), ==, 0);
    read_data(qts, 0x1000, buf, 4);
    g_assert_cmpmem(buf, 4, data, 4);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6));
    g_assert_cmphex(qtest_readl(qts, XIP_UNCACHED + 0x1000), ==, 0x78563412);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0x1000), ==, 0x78563412);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);

    /* Without write enable, nothing happens. */
    cmd_addr(qts, 0x20, 0x1000);
    g_assert_cmphex(sr1(qts), ==, 0);

    erase(qts, 0x20, 0x1000);
    expect_busy_for(qts, RV_TSE);
    g_assert_cmphex(read_byte(qts, 0x1000), ==, 0xff);
    erase(qts, 0x52, 0x8000);
    expect_busy_for(qts, RV_TBE32);
    erase(qts, 0xd8, 0x10000);
    expect_busy_for(qts, RV_TBE64);
    cmd(qts, 0x06);
    cmd(qts, 0xc7);
    expect_busy_for(qts, RV_TCE);
    cmd(qts, 0x06);
    cmd(qts, 0x60);
    expect_busy_for(qts, RV_TCE);

    /* An erase clocked past its address is not executed. */
    cmd(qts, 0x06);
    cs_low(qts);
    xfer(qts, 0x20);
    xfer(qts, 0);
    xfer(qts, 0x10);
    xfer(qts, 0);
    xfer(qts, 0);
    cs_high(qts);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    qtest_quit(qts);

    /* The maximum times, by property. */
    qts = start("flash-size=4M -global w25q32rv.max-timing=on");
    program(qts, 0, data, 4);
    expect_busy_for(qts, 2 * MS);
    erase(qts, 0x20, 0);
    expect_busy_for(qts, 240 * MS);
    erase(qts, 0xd8, 0);
    expect_busy_for(qts, 1200 * MS);
    qtest_quit(qts);

    /* The W25Q16JV's own times. */
    qts = start("flash-size=2M");
    program(qts, 0, data, 4);
    expect_busy_for(qts, 400 * US);
    erase(qts, 0x20, 0);
    expect_busy_for(qts, 45 * MS);
    erase(qts, 0x52, 0);
    expect_busy_for(qts, 120 * MS);
    erase(qts, 0xd8, 0);
    expect_busy_for(qts, 150 * MS);
    cmd(qts, 0x06);
    cmd(qts, 0xc7);
    expect_busy_for(qts, 5 * S);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-timing/test] */
static void test_erase_suspend(void)
{
    QTestState *qts = start("flash-size=4M");
    static const uint8_t data[4] = { 0xa1, 0xb2, 0xc3, 0xd4 };
    const uint32_t a = 0x20000, b = 0x30000;
    const int64_t before = 10 * MS;
    int64_t done = before + TSUS;
    uint8_t buf[4];
    uint32_t i;

    fill(qts, a, SECTOR);
    fill(qts, b, PAGE);

    erase(qts, 0x20, a);
    qtest_clock_step(qts, before);
    cmd(qts, 0x75);
    /* SUS sets at once; BUSY clears tSUS later. */
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, SR2_SUS);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    g_assert_cmphex(read_byte(qts, b), ==, 0);
    qtest_clock_step(qts, TSUS - 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, SR2_SUS);

    /* Other sectors read; the suspended one holds its cells part erased. */
    g_assert_cmphex(read_byte(qts, b + 5), ==, pattern(b + 5));
    for (i = 0; i < SECTOR; i += 97) {
        g_assert_cmphex(read_byte(qts, a + i), ==,
                        erase_partial(a + i, pattern(a + i), done, RV_TSE));
    }
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6));
    g_assert_cmphex(qtest_readb(qts, XIP_UNCACHED + a + 3), ==,
                    erase_partial(a + 3, pattern(a + 3), done, RV_TSE));
    g_assert_cmphex(qtest_readb(qts, XIP_BASE + b), ==, pattern(b));
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);

    /* A program of another sector runs, and returns to the suspend. */
    program(qts, b + PAGE, data, 4);
    expect_busy_for(qts, RV_TPP);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, SR2_SUS);
    read_data(qts, b + PAGE, buf, 4);
    g_assert_cmpmem(buf, 4, data, 4);

    /* Erases, and programs of the suspended sector, are refused. */
    erase(qts, 0x20, b);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    cmd(qts, 0x04);
    program(qts, a, data, 4);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    cmd(qts, 0x04);
    /* So is 01h; 31h is not among the commands the datasheet forbids. */
    wrsr(qts, true, 0x01, 0x04);
    g_assert_cmphex(sr1(qts), ==, 0);

    /* Resume: BUSY at once, SUS clear; the erase runs for its time left. */
    cmd(qts, 0x7a);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, 0);
    /* A suspend sooner than tSUS after the resume is ignored. */
    qtest_clock_step(qts, TSUS - 1);
    cmd(qts, 0x75);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, 0);
    qtest_clock_step(qts, RV_TSE - done - TSUS);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    for (i = 0; i < SECTOR; i += 61) {
        g_assert_cmphex(read_byte(qts, a + i), ==, 0xff);
    }

    /* Two suspends: the erase still takes its whole time in total. */
    erase(qts, 0x20, a);
    qtest_clock_step(qts, 5 * MS);
    cmd(qts, 0x75);
    qtest_clock_step(qts, 50 * US);
    cmd(qts, 0x7a);
    qtest_clock_step(qts, 5 * MS);
    cmd(qts, 0x75);
    qtest_clock_step(qts, 50 * US);
    cmd(qts, 0x7a);
    expect_busy_for(qts, RV_TSE - 10 * MS - 2 * TSUS);

    /* A suspend after the cycle ends, or with none, is ignored. */
    cmd(qts, 0x75);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, 0);

    /* A chip erase cannot be suspended. */
    cmd(qts, 0x06);
    cmd(qts, 0xc7);
    qtest_clock_step(qts, MS);
    cmd(qts, 0x75);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, 0);
    qtest_clock_step(qts, TSUS);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, RV_TCE - MS - TSUS - 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, SR1_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-timing/test] */
static void test_program_suspend(void)
{
    QTestState *qts = start("flash-size=4M");
    uint8_t data[PAGE];
    const uint32_t a = 0x40000;
    const int64_t before = 100 * US;
    int64_t done = before + TSUS;
    unsigned i;

    for (i = 0; i < PAGE; i++) {
        data[i] = pattern(i);
    }
    program(qts, a, data, PAGE);
    qtest_clock_step(qts, before);
    cmd(qts, 0x75);
    qtest_clock_step(qts, TSUS);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, SR2_SUS);
    for (i = 0; i < PAGE; i += 7) {
        g_assert_cmphex(read_byte(qts, a + i), ==,
                        program_partial(a + i, 0xff, data[i], done, RV_TPP));
    }
    /* Programs are refused while a program is suspended; erases run. */
    program(qts, a + 0x1000, data, 4);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    cmd(qts, 0x04);
    erase(qts, 0x20, a + 0x1000);
    expect_busy_for(qts, RV_TSE);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, SR2_SUS);

    cmd(qts, 0x7a);
    expect_busy_for(qts, RV_TPP - done);
    for (i = 0; i < PAGE; i += 5) {
        g_assert_cmphex(read_byte(qts, a + i), ==, data[i]);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-chip/test] */
static void test_protection(void)
{
    QTestState *qts = start("flash-size=4M");
    static const uint8_t zero[1] = { 0 };

    /* BP=001, TB=0: the top 64 KiB block. */
    wrsr(qts, true, 0x01, 0x04);
    erase(qts, 0x20, 0x3f0000);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    cmd(qts, 0x04);
    erase(qts, 0x20, 0x3e0000);
    expect_busy_for(qts, RV_TSE);
    /* CMP=1: all but the top block. */
    wrsr(qts, true, 0x31, 0x42);
    program(qts, 0x3e0000, zero, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    cmd(qts, 0x04);
    program(qts, 0x3f0000, zero, 1);
    expect_busy_for(qts, RV_TPP);
    /* SEC=1, TB=1, BP=010, CMP=0: the bottom 8 KiB. */
    wrsr(qts, true, 0x31, 0x02);
    wrsr(qts, true, 0x01, 0x68);
    program(qts, 0x1fff, zero, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    cmd(qts, 0x04);
    program(qts, 0x2000, zero, 1);
    expect_busy_for(qts, RV_TPP);
    /* A chip erase with anything protected is refused. */
    cmd(qts, 0x06);
    cmd(qts, 0xc7);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    cmd(qts, 0x04);

    /* SRP=1 with /WP low protects the registers only while QE=0. */
    wrsr(qts, true, 0x01, 0x80);
    qtest_set_irq_in(qts, "/machine/soc/flash", "WP#", 0, 0);
    wrsr(qts, true, 0x01, 0x84);
    g_assert_cmphex(sr1(qts), ==, 0x84);
    wrsr(qts, true, 0x31, 0x00);
    wrsr(qts, true, 0x01, 0x80);
    g_assert_cmphex(sr1(qts), ==, 0x84);
    qtest_quit(qts);

    /* W25Q16JV: WPS=1 locks every block until unlocked. */
    qts = start("flash-size=2M");
    wrsr(qts, true, 0x11, 0x64);
    program(qts, 0x10000, zero, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    cs_low(qts);
    xfer(qts, 0x3d);
    xfer(qts, 0x01);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0) & 1, ==, 1);
    cs_high(qts);
    /* WEL is still set from the refused program. */
    cmd_addr(qts, 0x39, 0x10000);
    program(qts, 0x10000, zero, 1);
    expect_busy_for(qts, 400 * US);
    program(qts, 0x20000, zero, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    /* The top and bottom blocks lock by sector. */
    cmd_addr(qts, 0x39, 0x1f1000);
    program(qts, 0x1f1000, zero, 1);
    expect_busy_for(qts, 400 * US);
    program(qts, 0x1f2000, zero, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    cmd(qts, 0x98);
    program(qts, 0x1f2000, zero, 1);
    expect_busy_for(qts, 400 * US);
    cmd(qts, 0x06);
    cmd(qts, 0x7e);
    program(qts, 0x1f2000, zero, 1);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-chip/test] */
static void test_security_registers(void)
{
    QTestState *qts = start("flash-size=4M");
    static const uint8_t data[2] = { 0x3c, 0xc3 };
    unsigned i;

    cmd(qts, 0x06);
    cs_low(qts);
    xfer(qts, 0x42);
    xfer(qts, 0x00);
    xfer(qts, 0x20);
    xfer(qts, 0xff);
    for (i = 0; i < 2; i++) {
        xfer(qts, data[i]);
    }
    cs_high(qts);
    expect_busy_for(qts, RV_TPP);

    /* 48h with a dummy byte; the register wraps at its 256 bytes. */
    cs_low(qts);
    xfer(qts, 0x48);
    xfer(qts, 0x00);
    xfer(qts, 0x20);
    xfer(qts, 0xff);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0x3c);
    g_assert_cmphex(xfer(qts, 0), ==, 0xc3);
    g_assert_cmphex(xfer(qts, 0), ==, 0xff);
    cs_high(qts);

    cmd(qts, 0x06);
    cmd_addr(qts, 0x44, 0x2000);
    expect_busy_for(qts, RV_TSE);

    /* LB2 locks register 2 for good. */
    wrsr(qts, false, 0x31, 0x12);
    qtest_clock_step(qts, RV_TW);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x16);
    wrsr(qts, false, 0x31, 0x02);
    qtest_clock_step(qts, RV_TW);
    g_assert_cmphex(rdsr(qts, 0x35), ==, 0x16);
    cmd(qts, 0x06);
    cmd_addr(qts, 0x44, 0x2000);
    g_assert_cmphex(sr1(qts) & SR1_BUSY, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.flash-chip/test] */
static void test_modes(void)
{
    QTestState *qts = start("flash-size=4M");
    static const uint8_t data[4] = { 0x11, 0x22, 0x33, 0x44 };
    int i;

    program(qts, 0x5000, data, 4);
    qtest_clock_step(qts, RV_TPP);

    /* EBh with M = 20h: the next transaction starts with the address. */
    cs_low(qts);
    xfer(qts, 0xeb);
    xfer(qts, 0x00);
    xfer(qts, 0x50);
    xfer(qts, 0x00);
    xfer(qts, 0x20);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0x11);
    cs_high(qts);
    cs_low(qts);
    xfer(qts, 0x00);
    xfer(qts, 0x50);
    xfer(qts, 0x01);
    xfer(qts, 0xff);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0x22);
    cs_high(qts);
    /* M = FFh above ended it. */
    g_assert_cmphex(read_byte(qts, 0x5002), ==, 0x33);

    /* Burst with wrap: 8-byte sections for EBh. */
    cs_low(qts);
    xfer(qts, 0x77);
    xfer(qts, 0);
    xfer(qts, 0);
    xfer(qts, 0);
    xfer(qts, 0x00);
    cs_high(qts);
    cs_low(qts);
    xfer(qts, 0xeb);
    xfer(qts, 0x00);
    xfer(qts, 0x50);
    xfer(qts, 0x02);
    xfer(qts, 0xf0);
    xfer(qts, 0);
    xfer(qts, 0);
    for (i = 2; i < 8; i++) {
        xfer(qts, 0);
    }
    g_assert_cmphex(xfer(qts, 0), ==, 0x11);
    cs_high(qts);

    /* QPI: 38h enters it; 03h is SPI only; FFh leaves it. */
    cmd(qts, 0x38);
    g_assert_cmphex(read_byte(qts, 0x5000), ==, 0);
    cs_low(qts);
    xfer(qts, 0x9f);
    g_assert_cmphex(xfer(qts, 0), ==, 0xef);
    cs_high(qts);
    /* QPI 0Bh: six dummy clocks, three bytes at four lines. */
    cs_low(qts);
    xfer(qts, 0x0b);
    xfer(qts, 0x00);
    xfer(qts, 0x50);
    xfer(qts, 0x01);
    xfer(qts, 0);
    xfer(qts, 0);
    xfer(qts, 0);
    g_assert_cmphex(xfer(qts, 0), ==, 0x22);
    cs_high(qts);
    cmd(qts, 0xff);
    g_assert_cmphex(read_byte(qts, 0x5000), ==, 0x11);

    /* Power-down: only ABh, and nothing for tRES1 after it. */
    cmd(qts, 0xb9);
    qtest_clock_step(qts, 3 * US);
    g_assert_cmphex(sr1(qts), ==, 0);
    g_assert_cmphex(read_byte(qts, 0x5000), ==, 0);
    cmd(qts, 0xab);
    qtest_clock_step(qts, 3 * US - 1);
    g_assert_cmphex(read_byte(qts, 0x5000), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(read_byte(qts, 0x5000), ==, 0x11);

    /* Software reset: tRST with nothing answered, not even 05h. */
    cmd(qts, 0x06);
    cmd(qts, 0x66);
    cmd(qts, 0x99);
    g_assert_cmphex(sr1(qts), ==, 0);
    qtest_clock_step(qts, 30 * US);
    cmd(qts, 0x06);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    /* Anything between 66h and 99h disarms it. */
    cmd(qts, 0x66);
    cmd(qts, 0x05);
    cmd(qts, 0x99);
    g_assert_cmphex(sr1(qts), ==, SR1_WEL);
    qtest_quit(qts);
}

/* The array as XIP reads it, outside direct mode. */
static void xip_read(QTestState *qts, uint32_t addr, uint8_t *buf,
                     size_t len)
{
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6));
    qtest_memread(qts, XIP_UNCACHED + addr, buf, len);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
}

/*
 * `len` bytes at `addr` as an erase from `old` cut `done` ns into `total`
 * leaves them: neither the old contents nor erased, and exactly the rule's.
 */
static void expect_erase_cut(const uint8_t *got, uint32_t addr, size_t len,
                             int64_t done, int64_t total)
{
    size_t i, same = 0, erased = 0;

    for (i = 0; i < len; i++) {
        uint8_t old = pattern(addr + i);

        g_assert_cmphex(got[i], ==, erase_partial(addr + i, old, done,
                                                  total));
        same += got[i] == old;
        erased += got[i] == 0xff;
    }
    g_assert_cmpuint(same, <, len);
    g_assert_cmpuint(erased, <, len);
}

static void expect_program_cut(const uint8_t *got, uint32_t addr,
                               const uint8_t *data, size_t len,
                               int64_t done, int64_t total)
{
    size_t i, same = 0, programmed = 0;

    for (i = 0; i < len; i++) {
        g_assert_cmphex(got[i], ==, program_partial(addr + i, 0xff, data[i],
                                                    done, total));
        same += got[i] == 0xff;
        programmed += got[i] == data[i];
    }
    g_assert_cmpuint(same, <, len);
    g_assert_cmpuint(programmed, <, len);
}

/* [spec:nuos:req:emu.flash-power-cut/test] */
static void test_power_cut_reset(void)
{
    QTestState *qts = start("flash-size=4M");
    const uint32_t a = 0x50000, p = 0x61000, b = 0x70000, c = 0x80000;
    uint8_t data[PAGE], got[SECTOR];
    unsigned i;

    for (i = 0; i < PAGE; i++) {
        data[i] = pattern(p + i) ^ 0x3c;
    }
    fill(qts, a, SECTOR);
    fill(qts, b, SECTOR);
    fill(qts, c, SECTOR);
    erase(qts, 0x20, p);
    qtest_clock_step(qts, RV_TSE);

    /* An erase cut 10 ms into its 30 ms by a machine reset. */
    erase(qts, 0x20, a);
    qtest_clock_step(qts, 10 * MS);
    qtest_system_reset(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    g_assert_cmphex(sr1(qts), ==, 0);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, 0);
    xip_read(qts, a, got, SECTOR);
    expect_erase_cut(got, a, SECTOR, 10 * MS, RV_TSE);

    /* A page program cut 100 us into its 250 us. */
    program(qts, p, data, PAGE);
    qtest_clock_step(qts, 100 * US);
    qtest_system_reset(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    xip_read(qts, p, got, PAGE);
    expect_program_cut(got, p, data, PAGE, 100 * US, RV_TPP);

    /* A suspended erase keeps what it had done when it suspended. */
    erase(qts, 0x20, b);
    qtest_clock_step(qts, 5 * MS);
    cmd(qts, 0x75);
    qtest_clock_step(qts, 2 * MS);
    qtest_system_reset(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    g_assert_cmphex(rdsr(qts, 0x35) & SR2_SUS, ==, 0);
    xip_read(qts, b, got, SECTOR);
    expect_erase_cut(got, b, SECTOR, 5 * MS + TSUS, RV_TSE);

    /* A software reset cuts an erase the same way. */
    erase(qts, 0x20, c);
    qtest_clock_step(qts, 4 * MS);
    cmd(qts, 0x66);
    cmd(qts, 0x99);
    qtest_clock_step(qts, 30 * US);
    xip_read(qts, c, got, SECTOR);
    expect_erase_cut(got, c, SECTOR, 4 * MS, RV_TSE);

    /* Erasing the part-erased sector again: the rule is monotonic. */
    erase(qts, 0x20, a);
    qtest_clock_step(qts, 4 * MS);
    qtest_system_reset(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    xip_read(qts, a, got, SECTOR);
    expect_erase_cut(got, a, SECTOR, 10 * MS, RV_TSE);
    qtest_quit(qts);
}

/*
 * A cycle cut by QEMU's shutdown, and one cut by a machine reset, as the
 * drive holds them afterwards.
 */
/* [spec:nuos:req:emu.flash-power-cut/test] */
static void test_power_cut_drive(void)
{
    const uint32_t a = 0x50000, p = 0x61000;
    g_autofree uint8_t *image = g_malloc(4 * MiB);
    g_autofree char *path = NULL;
    g_autofree char *opts = NULL;
    g_autofree gchar *after = NULL;
    uint8_t data[PAGE];
    gsize len;
    QTestState *qts;
    GError *err = NULL;
    int fd;
    unsigned i;

    for (i = 0; i < 4 * MiB; i++) {
        image[i] = pattern(i);
    }
    for (i = 0; i < SECTOR; i++) {
        image[p - (p % SECTOR) + i] = 0xff;
    }
    for (i = 0; i < PAGE; i++) {
        data[i] = pattern(p + i) ^ 0x3c;
    }
    fd = g_file_open_tmp("rp2350-flash-XXXXXX.img", &path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, image, 4 * MiB), ==, 4 * MiB);
    close(fd);

    opts = g_strdup_printf("flash-size=4M -drive file=%s,if=mtd,format=raw",
                           path);
    qts = start(opts);
    erase(qts, 0x20, a);
    qtest_clock_step(qts, 7 * MS);
    qtest_quit(qts);
    g_assert(g_file_get_contents(path, &after, &len, NULL));
    g_assert_cmpuint(len, ==, 4 * MiB);
    expect_erase_cut((uint8_t *)after + a, a, SECTOR, 7 * MS, RV_TSE);
    g_assert_cmpmem(after, a, image, a);
    g_assert_cmpmem(after + a + SECTOR, 4 * MiB - a - SECTOR,
                    image + a + SECTOR, 4 * MiB - a - SECTOR);
    g_free(after);

    qts = start(opts);
    program(qts, p, data, PAGE);
    qtest_clock_step(qts, 50 * US);
    qtest_system_reset(qts);
    qtest_quit(qts);
    g_assert(g_file_get_contents(path, &after, &len, NULL));
    expect_program_cut((uint8_t *)after + p, p, data, PAGE, 50 * US,
                       RV_TPP);
    unlink(path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/rp2350-flash/ids", test_ids);
    qtest_add_func("/rp2350-flash/status", test_status);
    qtest_add_func("/rp2350-flash/program-erase-times",
                   test_program_erase_times);
    qtest_add_func("/rp2350-flash/erase-suspend", test_erase_suspend);
    qtest_add_func("/rp2350-flash/program-suspend", test_program_suspend);
    qtest_add_func("/rp2350-flash/protection", test_protection);
    qtest_add_func("/rp2350-flash/security-registers",
                   test_security_registers);
    qtest_add_func("/rp2350-flash/modes", test_modes);
    qtest_add_func("/rp2350-flash/power-cut-reset", test_power_cut_reset);
    qtest_add_func("/rp2350-flash/power-cut-drive", test_power_cut_drive);
    return g_test_run();
}
