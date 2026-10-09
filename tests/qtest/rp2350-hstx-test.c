/*
 * QTest testcase for the RP2350 high-speed serial transmit (HSTX)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * clk_hstx is clk_sys, 150 MHz: three cycles take 20 ns. The exact
 * output streams are read back from the device's "capture" and
 * "pin-history" properties.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-resets.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"
#include "qobject/qnum.h"

#define ALIAS_XOR       0x1000
#define ALIAS_SET       0x2000
#define ALIAS_CLR       0x3000
#define NO_REPLICATE    0x4000

#define HSTX_CTRL       0x400c0000
#define CSR             (HSTX_CTRL + 0x00)
#define BITN(n)         (HSTX_CTRL + 0x04 + 4 * (n))
#define EXPAND_SHIFT    (HSTX_CTRL + 0x24)
#define EXPAND_TMDS     (HSTX_CTRL + 0x28)

#define HSTX_FIFO       0x50600000
#define STAT            (HSTX_FIFO + 0x00)
#define FIFO            (HSTX_FIFO + 0x04)

#define CSR_EN          (1u << 0)
#define CSR_EXPAND_EN   (1u << 1)
#define CSR_SHIFT(n)    ((n) << 8)
#define CSR_N_SHIFTS(n) ((n) << 16)
#define CSR_CLKPHASE(n) ((n) << 24)
#define CSR_CLKDIV(n)   ((uint32_t)(n) << 28)

#define BITN_SEL_P(n)   (n)
#define BITN_SEL_N(n)   ((n) << 8)
#define BITN_INV        (1u << 16)
#define BITN_CLK        (1u << 17)

#define STAT_FULL       (1u << 8)
#define STAT_EMPTY      (1u << 9)
#define STAT_WOF        (1u << 10)

#define CMD_RAW         0x0000
#define CMD_RAW_REPEAT  0x1000
#define CMD_TMDS        0x2000
#define CMD_TMDS_REPEAT 0x3000
#define CMD_NOP         0xf000

#define IO_BANK0        0x40028000
#define STATUS(p)       (IO_BANK0 + 8 * (p))
#define CTRL(p)         (IO_BANK0 + 8 * (p) + 4)
#define STATUS_OUTTOPAD (1u << 9)
#define STATUS_OETOPAD  (1u << 13)
#define FUNC_HSTX       0
#define PADS_BANK0      0x40038000
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define PAD_IE          (1u << 6)

#define HSTX_PATH       "/machine/soc/hstx"

static char *rom_path;
/* Virtual time, in ns. */
static int64_t vnow;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    vnow = RP2350_RESETS_RELEASE_NS;
    return qts;
}

static void step(QTestState *qts, int64_t ns)
{
    vnow = qtest_clock_step(qts, ns);
}

/* Run to the end of the n-th clk_hstx cycle from now. */
static void cycles(QTestState *qts, uint64_t n)
{
    uint64_t target = vnow * 3 / 20 + n;

    step(qts, (target * 20 + 2) / 3 - vnow);
}

static QObject *qom_get(QTestState *qts, const char *prop, QDict **resp)
{
    *resp = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                      "{'path': %s, 'property': %s}}", HSTX_PATH, prop);
    g_assert(qdict_haskey(*resp, "return"));
    return qdict_get(*resp, "return");
}

static uint64_t get_u64(QTestState *qts, const char *prop)
{
    QDict *resp;
    uint64_t v = qnum_get_uint(qobject_to(QNum, qom_get(qts, prop, &resp)));

    qobject_unref(resp);
    return v;
}

/* A list property into out[]; returns its length. */
static int get_list(QTestState *qts, const char *prop, uint64_t *out, int max)
{
    QDict *resp;
    QList *list = qobject_to(QList, qom_get(qts, prop, &resp));
    QListEntry *e;
    int n = 0;

    QLIST_FOREACH_ENTRY(list, e) {
        g_assert_cmpint(n, <, max);
        out[n++] = qnum_get_uint(qobject_to(QNum, qlist_entry_obj(e)));
    }
    qobject_unref(resp);
    return n;
}

static uint64_t history(QTestState *qts, int bit)
{
    uint64_t h[8];

    g_assert_cmpint(get_list(qts, "pin-history", h, 8), ==, 8);
    return h[bit];
}

/* TMDS encoding, as in chapter 3 of the DVI 1.0 specification. */
static uint32_t ref_tmds(uint8_t d, int *cnt)
{
    int n1 = __builtin_popcount(d), n1q, n0q, i;
    int xnor = n1 > 4 || (n1 == 4 && !(d & 1));
    unsigned qm = d & 1, qm8 = !xnor;

    for (i = 1; i < 8; i++) {
        unsigned b = (qm >> (i - 1)) ^ (d >> i);

        qm |= ((xnor ? ~b : b) & 1) << i;
    }
    n1q = __builtin_popcount(qm);
    n0q = 8 - n1q;
    if (*cnt == 0 || n1q == n0q) {
        *cnt += qm8 ? n1q - n0q : n0q - n1q;
        return (!qm8 << 9) | (qm8 << 8) | (qm8 ? qm : ~qm & 0xff);
    }
    if ((*cnt > 0 && n1q > n0q) || (*cnt < 0 && n0q > n1q)) {
        *cnt += 2 * qm8 + n0q - n1q;
        return 0x200 | (qm8 << 8) | (~qm & 0xff);
    }
    *cnt += -2 * !qm8 + n1q - n0q;
    return (qm8 << 8) | qm;
}

/* Lanes from bytes 0, 1 and 2 of x (EXPAND_TMDS 0xf0e8e0 below). */
static uint32_t ref_pixel(uint32_t x, int *cnt)
{
    return ref_tmds(x, &cnt[0]) | ref_tmds(x >> 8, &cnt[1]) << 10 |
           ref_tmds(x >> 16, &cnt[2]) << 20;
}

#define TMDS_RGB888     0x00f0e8e0u

/* [spec:nuos:req:emu.hstx/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int n;

    g_assert_cmphex(qtest_readl(qts, CSR), ==, 0x10050600);
    for (n = 0; n < 8; n++) {
        g_assert_cmphex(qtest_readl(qts, BITN(n)), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, EXPAND_SHIFT), ==, 0x01000100);
    g_assert_cmphex(qtest_readl(qts, EXPAND_TMDS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_EMPTY);

    /* Reserved bits read zero. */
    qtest_writel(qts, CSR, 0xfffffffe);
    g_assert_cmphex(qtest_readl(qts, CSR), ==, 0xff1f1f72);
    qtest_writel(qts, BITN(5), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, BITN(5)), ==, 0x00031f1f);
    qtest_writel(qts, EXPAND_SHIFT, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, EXPAND_SHIFT), ==, 0x1f1f1f1f);
    qtest_writel(qts, EXPAND_TMDS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, EXPAND_TMDS), ==, 0x00ffffff);

    /* Atomic aliases. */
    qtest_writel(qts, BITN(2), 0x00001f00);
    qtest_writel(qts, BITN(2) + ALIAS_XOR, 0x00011f1f);
    g_assert_cmphex(qtest_readl(qts, BITN(2)), ==, 0x0001001f);
    qtest_writel(qts, BITN(2) + ALIAS_SET, 0x00020000);
    g_assert_cmphex(qtest_readl(qts, BITN(2)), ==, 0x0003001f);
    qtest_writel(qts, BITN(2) + ALIAS_CLR, 0x00010001);
    g_assert_cmphex(qtest_readl(qts, BITN(2) + ALIAS_CLR), ==, 0x0002001e);

    /* Narrow writes are replicated, or zero-extended at bit 14. */
    qtest_writeb(qts, BITN(3), 0x05);
    g_assert_cmphex(qtest_readl(qts, BITN(3)), ==, 0x00010505);
    qtest_writeb(qts, BITN(3) + NO_REPLICATE + 1, 0x07);
    g_assert_cmphex(qtest_readl(qts, BITN(3)), ==, 0x00000700);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.hstx/test] */
static void test_fifo(void)
{
    QTestState *qts = start();
    int n;

    qtest_irq_intercept_out_named(qts, HSTX_PATH, "dreq");
    /* Not enabled: the FIFO fills and holds. */
    for (n = 0; n < 7; n++) {
        qtest_writel(qts, FIFO, n);
        g_assert_cmphex(qtest_readl(qts, STAT), ==, n + 1);
    }
    qtest_writel(qts, FIFO, 7);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FULL | 8);
    g_assert_false(qtest_get_irq(qts, 0));
    cycles(qts, 100);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FULL | 8);

    /* Overflow drops the word and sets WOF, which is write-1-to-clear. */
    qtest_writel(qts, FIFO, 0xbad);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_WOF | STAT_FULL | 8);
    qtest_writel(qts, STAT, 0x3ff);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_WOF | STAT_FULL | 8);
    qtest_writel(qts, STAT, STAT_WOF);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FULL | 8);

    /*
     * Enabled, bypassing the expander, with one shift per word: one pop
     * per cycle, the first in the first cycle. DREQ rises when the first
     * pop makes room.
     */
    qtest_writel(qts, CSR, CSR_EN | CSR_N_SHIFTS(1) | CSR_SHIFT(0));
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FULL | 8);
    cycles(qts, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 7);
    cycles(qts, 3);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 4);
    cycles(qts, 10);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_EMPTY);
    g_assert_cmpuint(get_u64(qts, "capture-words"), ==, 8);

    /* N_SHIFTS 0 is 32 shifts: a pop every 32 cycles. */
    qtest_writel(qts, CSR, 0);
    qtest_writel(qts, CSR, CSR_EN | CSR_N_SHIFTS(0) | CSR_SHIFT(1));
    for (n = 0; n < 3; n++) {
        qtest_writel(qts, FIFO, n);
    }
    cycles(qts, 1);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 2);
    cycles(qts, 31);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 2);
    cycles(qts, 1);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 1);

    /* Clearing EN stops popping and keeps the FIFO's contents. */
    qtest_writel(qts, CSR, CSR_N_SHIFTS(0) | CSR_SHIFT(1));
    cycles(qts, 100);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 1);

    /* Narrow FIFO writes push one replicated word each. */
    qtest_writel(qts, CSR, CSR_EN | CSR_N_SHIFTS(1));
    cycles(qts, 10);
    qtest_writeb(qts, FIFO, 0xa5);
    qtest_writew(qts, FIFO + 2, 0x1234);
    cycles(qts, 10);
    {
        uint64_t cap[16];
        int len = get_list(qts, "capture", cap, 16);

        g_assert_cmpint(len, >=, 2);
        g_assert_cmphex(cap[len - 2], ==, 0xa5a5a5a5);
        g_assert_cmphex(cap[len - 1], ==, 0x12341234);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.hstx/test] */
static void test_raw_shift(void)
{
    const uint32_t word = 0xc0ffee35;
    QTestState *qts = start();
    uint64_t lsb = 0, msb = 0, ddr = 0, clk = 0;
    int i;

    /* One bit per cycle, LSB-first on bit 0, inverted on bit 1. */
    qtest_writel(qts, BITN(0), BITN_SEL_P(0) | BITN_SEL_N(0));
    qtest_writel(qts, BITN(1), BITN_SEL_P(0) | BITN_SEL_N(0) | BITN_INV);
    /* Bit 31 of the register, as it rotates right: bits 31, 0, 1, ... */
    qtest_writel(qts, BITN(2), BITN_SEL_P(31) | BITN_SEL_N(31));
    /* The clock: period 2 cycles, phase half a cycle. */
    qtest_writel(qts, BITN(3), BITN_CLK);
    qtest_writel(qts, CSR, CSR_EN | CSR_N_SHIFTS(0) | CSR_SHIFT(1) |
                 CSR_CLKDIV(2) | CSR_CLKPHASE(1));
    qtest_writel(qts, FIFO, word);
    cycles(qts, 40);
    g_assert_cmpuint(get_u64(qts, "half-cycles"), ==, 64);

    for (i = 0; i < 32; i++) {
        uint64_t b = (word >> i) & 1;
        uint64_t r = (word >> ((31 + i) % 32)) & 1;

        lsb |= (b | b << 1) << (2 * i);
        msb |= (r | r << 1) << (2 * i);
    }
    /* Clock counts 1, 2, 3, 0, ... half-cycles: high at 2 and 3. */
    for (i = 0; i < 64; i++) {
        clk |= (uint64_t)(((1 + i) % 4) >= 2) << i;
    }
    g_assert_cmphex(history(qts, 0), ==, lsb);
    g_assert_cmphex(history(qts, 1), ==, ~lsb);
    g_assert_cmphex(history(qts, 2), ==, msb);
    g_assert_cmphex(history(qts, 3), ==, clk);

    /*
     * Two bits per cycle (DDR), 16 shifts by 2: the word in order. The
     * clock generator restarts at CLKPHASE when EN is cleared.
     */
    qtest_writel(qts, CSR, 0);
    qtest_writel(qts, BITN(0), BITN_SEL_P(0) | BITN_SEL_N(1));
    qtest_writel(qts, CSR, CSR_EN | CSR_N_SHIFTS(16) | CSR_SHIFT(2) |
                 CSR_CLKDIV(1));
    qtest_writel(qts, FIFO, word);
    qtest_writel(qts, FIFO, ~word);
    cycles(qts, 40);
    g_assert_cmpuint(get_u64(qts, "half-cycles"), ==, 128);
    for (i = 0; i < 32; i++) {
        ddr |= (uint64_t)((word >> i) & 1) << i;
        ddr |= (uint64_t)((~word >> i) & 1) << (32 + i);
    }
    g_assert_cmphex(history(qts, 0), ==, ddr);
    /* CLKDIV 1, phase 0: low then high in every cycle. */
    g_assert_cmphex(history(qts, 3), ==, 0xaaaaaaaaaaaaaaaaull);

    /* The pins: GPIO 12 is output bit 0, GPIO 15 bit 3. */
    qtest_writel(qts, PAD(12), PAD_IE);
    qtest_writel(qts, PAD(15), PAD_IE);
    qtest_writel(qts, CTRL(12), FUNC_HSTX);
    qtest_writel(qts, CTRL(15), FUNC_HSTX);
    g_assert_cmphex(qtest_readl(qts, STATUS(12)) & STATUS_OETOPAD, ==,
                    STATUS_OETOPAD);
    /* Stopped, the register holds ~word rotated by 32: bit 0 is 0. */
    g_assert_cmphex(qtest_readl(qts, STATUS(12)) & STATUS_OUTTOPAD, ==, 0);
    qtest_writel(qts, BITN(0), BITN_SEL_P(0) | BITN_SEL_N(1) | BITN_INV);
    g_assert_cmphex(qtest_readl(qts, STATUS(12)) & STATUS_OUTTOPAD, ==,
                    STATUS_OUTTOPAD);
    /* Disabled, the clock output holds its CLKPHASE level. */
    qtest_writel(qts, CSR, CSR_CLKDIV(1) | CSR_CLKPHASE(1));
    g_assert_cmphex(qtest_readl(qts, STATUS(15)) & STATUS_OUTTOPAD, ==,
                    STATUS_OUTTOPAD);
    qtest_writel(qts, CSR, CSR_CLKDIV(1) | CSR_CLKPHASE(0));
    g_assert_cmphex(qtest_readl(qts, STATUS(15)) & STATUS_OUTTOPAD, ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.hstx/test] */
static void test_expander_raw(void)
{
    QTestState *qts = start();
    uint64_t cap[16];
    int len;

    /* RAW: two outputs per FIFO word, the second rotated by 16. */
    qtest_writel(qts, EXPAND_SHIFT, (2 << 8) | 16);
    qtest_writel(qts, CSR, CSR_EN | CSR_EXPAND_EN | CSR_N_SHIFTS(2) |
                 CSR_SHIFT(0));
    qtest_writel(qts, FIFO, CMD_NOP | 0x123);
    qtest_writel(qts, FIFO, CMD_RAW | 3);
    qtest_writel(qts, FIFO, 0xaaaabbbb);
    qtest_writel(qts, FIFO, 0xccccdddd);
    /* RAW_REPEAT: one word, output three times, rotating by RAW_SHIFT. */
    qtest_writel(qts, FIFO, CMD_RAW_REPEAT | 3);
    qtest_writel(qts, FIFO, 0x354);
    /* A reserved opcode is ignored. */
    qtest_writel(qts, FIFO, 0x7000);
    qtest_writel(qts, FIFO, CMD_RAW | 1);
    cycles(qts, 4);
    qtest_writel(qts, FIFO, 0x0ab);
    cycles(qts, 100);
    len = get_list(qts, "capture", cap, 16);
    g_assert_cmpint(len, ==, 7);
    g_assert_cmphex(cap[0], ==, 0xaaaabbbb);
    g_assert_cmphex(cap[1], ==, 0xbbbbaaaa);
    g_assert_cmphex(cap[2], ==, 0xccccdddd);
    g_assert_cmphex(cap[3], ==, 0x354);
    g_assert_cmphex(cap[4], ==, 0x03540000);
    g_assert_cmphex(cap[5], ==, 0x354);
    g_assert_cmphex(cap[6], ==, 0x0ab);
    /* The unused half of 0xccccdddd was dropped with the command. */
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_EMPTY);
    /* Output was continuous through the two-shift words: 14 cycles. */
    g_assert_cmpuint(get_u64(qts, "half-cycles"), ==, 28);

    /*
     * Timing: the command takes a cycle, its data word is popped in the
     * next and output at once.
     */
    qtest_writel(qts, CSR, 0);
    qtest_writel(qts, EXPAND_SHIFT, 0x01000100);
    qtest_writel(qts, CSR, CSR_EN | CSR_EXPAND_EN | CSR_N_SHIFTS(5));
    qtest_writel(qts, FIFO, CMD_RAW | 2);
    qtest_writel(qts, FIFO, 1);
    qtest_writel(qts, FIFO, 2);
    cycles(qts, 1);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 2);
    g_assert_cmpuint(get_u64(qts, "capture-words"), ==, 7);
    cycles(qts, 1);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 1);
    g_assert_cmpuint(get_u64(qts, "capture-words"), ==, 8);
    /* The next data word is fetched ahead, while the first shifts. */
    cycles(qts, 1);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_EMPTY);
    g_assert_cmpuint(get_u64(qts, "capture-words"), ==, 8);
    cycles(qts, 4);
    g_assert_cmpuint(get_u64(qts, "capture-words"), ==, 9);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.hstx/test] */
static void test_tmds(void)
{
    static const uint32_t px[] = {
        0x000000, 0x000000, 0x000000, 0xffffff, 0x123456, 0x80ff01,
        0x10f0aa, 0x555555, 0x0f0f0f,
    };
    QTestState *qts = start();
    uint64_t cap[16];
    int cnt[3] = { 0, 0, 0 };
    int len, i;

    /* RGB888 pixels, one per FIFO word, to three lanes of 10 bits. */
    qtest_writel(qts, EXPAND_TMDS, TMDS_RGB888);
    qtest_writel(qts, EXPAND_SHIFT, 0x01000100);
    qtest_writel(qts, CSR, CSR_EN | CSR_EXPAND_EN | CSR_N_SHIFTS(5) |
                 CSR_SHIFT(2));
    qtest_writel(qts, FIFO, CMD_TMDS | ARRAY_SIZE(px));
    for (i = 0; i < ARRAY_SIZE(px); i++) {
        qtest_writel(qts, FIFO, px[i]);
        cycles(qts, 5);
    }
    cycles(qts, 20);
    len = get_list(qts, "capture", cap, 16);
    g_assert_cmpint(len, ==, ARRAY_SIZE(px));
    /*
     * Known symbols: black from zero disparity encodes as 0x100, then
     * alternates with 0x3ff to keep DC balance.
     */
    g_assert_cmphex(cap[0], ==, 0x100 | 0x100 << 10 | 0x100 << 20);
    g_assert_cmphex(cap[1], ==, 0x3ff | 0x3ff << 10 | 0x3ff << 20);
    g_assert_cmphex(cap[2], ==, 0x100 | 0x100 << 10 | 0x100 << 20);
    for (i = 0; i < ARRAY_SIZE(px); i++) {
        g_assert_cmphex(cap[i], ==, ref_pixel(px[i], cnt));
    }

    /*
     * Each lane's symbol goes out LSB-first, two bits per cycle: bit 0
     * of the crossbar on lane 0, bit 2 on lane 1, bit 4 on lane 2.
     */
    qtest_writel(qts, CSR, 0);
    qtest_writel(qts, BITN(0), BITN_SEL_P(0) | BITN_SEL_N(1));
    qtest_writel(qts, BITN(2), BITN_SEL_P(10) | BITN_SEL_N(11));
    qtest_writel(qts, BITN(4), BITN_SEL_P(20) | BITN_SEL_N(21));
    qtest_writel(qts, BITN(6), BITN_CLK);
    qtest_writel(qts, CSR, CSR_EN | CSR_EXPAND_EN | CSR_N_SHIFTS(5) |
                 CSR_SHIFT(2) | CSR_CLKDIV(5));
    memset(cnt, 0, sizeof(cnt));
    qtest_writel(qts, FIFO, CMD_TMDS | 3);
    qtest_writel(qts, FIFO, 0x123456);
    qtest_writel(qts, FIFO, 0xabcdef);
    qtest_writel(qts, FIFO, 0x000000);
    cycles(qts, 100);
    {
        uint64_t sym[3] = { 0, 0, 0 };

        /* Only the last 64 half-cycles: the last three symbols' tails. */
        for (i = 0; i < 3; i++) {
            uint32_t w = ref_pixel(i == 0 ? 0x123456 :
                                   i == 1 ? 0xabcdef : 0, cnt);
            int l;

            for (l = 0; l < 3; l++) {
                sym[l] |= (uint64_t)((w >> (10 * l)) & 0x3ff) << (10 * i);
            }
        }
        for (i = 0; i < 3; i++) {
            g_assert_cmphex(history(qts, 2 * i) >> 34, ==, sym[i]);
        }
        /* The TMDS clock: 5 low half-cycles, 5 high, phase-locked. */
        g_assert_cmphex(history(qts, 6) >> 34, ==, 0x3e0f83e0ull);
    }
    qtest_quit(qts);
}

/*
 * Infinite repeats run for many words; the device's fast path must give
 * exactly the stream a word-by-word run gives.
 */
/* [spec:nuos:req:emu.hstx/test] */
static void test_tmds_repeat(void)
{
    const uint32_t x = 0x00c3a512;
    QTestState *qts;
    uint64_t cap[16], words, w;
    int cnt[3] = { 0, 0, 0 };
    uint32_t sr = x, ref[16];
    int len, i;

    /* No periodic refresh: each step below is run in one go. */
    qts = qtest_initf("-M rp2350 -bios %s "
                      "-global rp2350-hstx.pin-refresh-ns=0", rom_path);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    vnow = RP2350_RESETS_RELEASE_NS;

    /* Rotate the expansion register by 8 each output: four pixels. */
    qtest_writel(qts, EXPAND_TMDS, TMDS_RGB888);
    qtest_writel(qts, EXPAND_SHIFT, 8 << 16);
    qtest_writel(qts, CSR, CSR_EN | CSR_EXPAND_EN | CSR_N_SHIFTS(5) |
                 CSR_SHIFT(2) | CSR_CLKDIV(3));
    qtest_writel(qts, FIFO, CMD_TMDS_REPEAT | 0);
    qtest_writel(qts, FIFO, x);
    step(qts, 1000000);
    words = get_u64(qts, "capture-words");
    /* 150000 cycles, less the two popping the command and data. */
    g_assert_cmpuint(words, >=, 29990);
    g_assert_cmpuint(words, <=, 30000);
    /* The last word may not have finished shifting. */
    g_assert_cmpuint(get_u64(qts, "half-cycles"), <=, words * 10);
    g_assert_cmpuint(get_u64(qts, "half-cycles"), >, (words - 1) * 10);
    for (w = 0; w < words; w++) {
        ref[w % 16] = ref_pixel(sr, cnt);
        sr = (sr >> 8) | (sr << 24);
    }
    len = get_list(qts, "capture", cap, 16);
    g_assert_cmpint(len, ==, 16);
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(cap[i], ==, ref[(words - 16 + i) % 16]);
    }
    /* The FIFO is never popped again. */
    qtest_writel(qts, FIFO, CMD_RAW | 1);
    step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, 1);

    /*
     * Restarted, the expander takes the waiting word as a command: one
     * control symbol, then another for ever.
     */
    qtest_writel(qts, CSR, 0);
    qtest_writel(qts, CSR, CSR_EN | CSR_EXPAND_EN | CSR_N_SHIFTS(5));
    qtest_writel(qts, FIFO, 0x354);
    qtest_writel(qts, FIFO, CMD_RAW_REPEAT | 0);
    qtest_writel(qts, FIFO, 0x2ab);
    words = get_u64(qts, "capture-words");
    step(qts, 10000000);
    g_assert_cmpuint(get_u64(qts, "capture-words") - words, >=, 299990);
    len = get_list(qts, "capture", cap, 16);
    g_assert_cmpint(len, ==, 16);
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(cap[i], ==, 0x2ab);
    }
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_EMPTY);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-hstx-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/hstx/reset-values", test_reset_values);
    qtest_add_func("/rp2350/hstx/fifo", test_fifo);
    qtest_add_func("/rp2350/hstx/raw-shift", test_raw_shift);
    qtest_add_func("/rp2350/hstx/expander-raw", test_expander_raw);
    qtest_add_func("/rp2350/hstx/tmds", test_tmds);
    qtest_add_func("/rp2350/hstx/tmds-repeat", test_tmds_repeat);
    ret = g_test_run();

    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
