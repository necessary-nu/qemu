/*
 * QTest testcase for the ESP32-P4 HP cores' CLIC and CLINT
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest's accesses come from outside the cores, so they see core 0's
 * view: its own CLIC and CLINT at the base of each window and core 1's
 * 64 KiB above.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CLINT           0x20000000
#define CLINT_OTHER     0x20010000
#define CLIC            0x20800000
#define CLIC_OTHER      0x20810000
#define CLIC_U          0x20b00000
#define CLIC_U_OTHER    0x20b10000

#define MSIP            0x0000
#define MTIMECMPLO      0x4000
#define MTIMECMPHI      0x4004
#define MTIMELOADLO     0x4008
#define MTIMELOADHI     0x400c
#define MTIMECTL        0x4010
#define MTIMELO         0xbff8
#define MTIMEHI         0xbffc

#define MCLICCFG        0x0
#define CLICINFO        0x4
#define MINTTHRESH      0x8
#define CLICINT(id)     (0x1000 + 4 * (id))

#define HP_SYS_CLKRST_SOC_CLK_CTRL0 0x500e6014
#define HP_SYS_CLKRST_HP_RST_EN0    0x500e60c0
#define CORE0_CLIC_CLK_EN           (1u << 0)
#define RST_EN_CORE1_GLOBAL         (1u << 8)

/* CPU_CLK, which the CLINT's counter counts, from reset: 40 MHz. */
#define CPU_CLK_NS      25

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

/* Release core 1 from reset, so that its CLIC takes writes. */
static void release_core1(QTestState *qts)
{
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0) &
                 ~RST_EN_CORE1_GLOBAL);
}

/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clic_reset(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINFO), ==, 0x00600030);
    g_assert_cmphex(qtest_readl(qts, CLIC + MINTTHRESH), ==, 0);
    for (int id = 0; id < 48; id++) {
        bool exists = id == 3 || id == 7 || id >= 16;

        /* MODE machine, CTL's unimplemented bits reading one. */
        g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(id)), ==,
                        exists ? 0x1fc00000 : 0);
        g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + CLICINT(id)), ==,
                        exists ? 0x1fc00000 : 0);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clic_registers(void)
{
    QTestState *qts = start();

    /* mcliccfg: MNLBITS and UNLBITS clamp to 8; NMBITS has one bit. */
    qtest_writel(qts, CLIC + MCLICCFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0x08000018);
    qtest_writel(qts, CLIC + MCLICCFG, 0x03000013);
    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0x03000013);
    /* clicinfo is read-only. */
    qtest_writel(qts, CLIC + CLICINFO, 0);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINFO), ==, 0x00600030);
    /* mintthresh: TH in bits 31:24. */
    qtest_writel(qts, CLIC + MINTTHRESH, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLIC + MINTTHRESH), ==, 0xff000000);
    qtest_writel(qts, CLIC + MINTTHRESH, 0);

    /*
     * Interrupt 16: IE bit 0, ATTR SHV, TRIG and MODE, CTL's upper three
     * bits; with NMBITS set MODE's upper bit gives the mode.
     */
    qtest_writel(qts, CLIC + CLICINT(16), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(16)), ==, 0xffc70101);
    /* Level-triggered now, its pending bit follows its line. */
    qtest_writel(qts, CLIC + CLICINT(16), 0x00000001);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(16)), ==, 0x1f000000);
    qtest_writel(qts, CLIC + CLICINT(16), 0x00410000);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(16)), ==, 0x1f010000);
    /* A user-mode interrupt's MODE reads 0, and 3 with NMBITS clear. */
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(16) + 2), ==, 0x01);
    qtest_writel(qts, CLIC + MCLICCFG, 0);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(16) + 2), ==, 0xc1);

    /* A level-triggered interrupt's pending bit ignores writes. */
    qtest_writeb(qts, CLIC + CLICINT(17) + 2, 0x00);
    qtest_writeb(qts, CLIC + CLICINT(17), 1);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(17)), ==, 0);
    /* An edge-triggered one's is written. */
    qtest_writeb(qts, CLIC + CLICINT(17) + 2, 0x02);
    qtest_writeb(qts, CLIC + CLICINT(17), 1);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(17)), ==, 1);
    qtest_writeb(qts, CLIC + CLICINT(17), 0);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(17)), ==, 0);

    /* Interrupts that do not exist read zero and ignore writes. */
    qtest_writel(qts, CLIC + CLICINT(5), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(5)), ==, 0);
    qtest_quit(qts);
}

/* Byte, halfword and word accesses each reach their own registers. */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clic_byte_access(void)
{
    QTestState *qts = start();

    qtest_writeb(qts, CLIC + CLICINT(20) + 3, 0xa0);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(20)), ==, 0xbfc00000);
    qtest_writeb(qts, CLIC + CLICINT(20) + 1, 1);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(20)), ==, 0xbfc00100);
    qtest_writew(qts, CLIC + CLICINT(20) + 2, 0x60c2);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(20)), ==, 0x7fc20100);
    g_assert_cmphex(qtest_readw(qts, CLIC + CLICINT(20)), ==, 0x0100);
    g_assert_cmphex(qtest_readw(qts, CLIC + CLICINT(20) + 2), ==, 0x7fc2);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(20) + 3), ==, 0x7f);
    qtest_writeb(qts, CLIC + MCLICCFG, 0x13);
    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0x13);
    qtest_writeb(qts, CLIC + MCLICCFG + 3, 0x02);
    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0x02000013);
    g_assert_cmphex(qtest_readb(qts, CLIC + MINTTHRESH + 3), ==, 0);
    qtest_writeb(qts, CLIC + MINTTHRESH + 3, 0x5f);
    g_assert_cmphex(qtest_readl(qts, CLIC + MINTTHRESH), ==, 0x5f000000);
    qtest_quit(qts);
}

/*
 * The other core's CLIC, 64 KiB up: core 1's, held in reset with core 1
 * from chip reset, takes writes once core 1 is released.
 */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clic_other_core(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CLIC_OTHER + CLICINT(30), 0xe0000100);
    g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + CLICINT(30)), ==,
                    0x1fc00000);
    release_core1(qts);
    qtest_writel(qts, CLIC_OTHER + CLICINT(30), 0xe0000100);
    g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + CLICINT(30)), ==,
                    0xffc00100);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(30)), ==, 0x1fc00000);
    qtest_writel(qts, CLIC_OTHER + MCLICCFG, 0x3);
    g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + MCLICCFG), ==, 0x3);
    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0);
    qtest_writel(qts, CLIC_OTHER + MINTTHRESH, 0x7f000000);
    g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + MINTTHRESH), ==,
                    0x7f000000);
    g_assert_cmphex(qtest_readl(qts, CLIC + MINTTHRESH), ==, 0);

    /* Holding core 1 in reset again resets its CLIC. */
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0) |
                 RST_EN_CORE1_GLOBAL);
    g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + CLICINT(30)), ==,
                    0x1fc00000);
    g_assert_cmphex(qtest_readl(qts, CLIC_OTHER + MCLICCFG), ==, 0);
    qtest_quit(qts);
}

/*
 * The user-mode view: ucliccfg's UNLBITS and the registers of user-mode
 * interrupts only, which it cannot make machine-mode ones.
 */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clic_user_view(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CLIC + MCLICCFG, 0x13);
    g_assert_cmphex(qtest_readl(qts, CLIC_U + MCLICCFG), ==, 0);
    qtest_writel(qts, CLIC_U + MCLICCFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLIC_U + MCLICCFG), ==, 0x08000000);
    g_assert_cmphex(qtest_readl(qts, CLIC + MCLICCFG), ==, 0x08000013);

    /* Interrupt 18 is machine mode: hidden; 19 user mode: visible. */
    qtest_writel(qts, CLIC + CLICINT(18), 0x20c20100);
    qtest_writel(qts, CLIC + CLICINT(19), 0x40020100);
    g_assert_cmphex(qtest_readl(qts, CLIC_U + CLICINT(18)), ==, 0);
    qtest_writel(qts, CLIC_U + CLICINT(18), 0);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(18)), ==, 0x3fc20100);
    g_assert_cmphex(qtest_readl(qts, CLIC_U + CLICINT(19)), ==, 0x5f020100);
    qtest_writel(qts, CLIC_U + CLICINT(19), 0xe0c20001);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(19)), ==, 0xff020001);
    /* The other core's user-mode view. */
    g_assert_cmphex(qtest_readl(qts, CLIC_U_OTHER + MCLICCFG), ==, 0);
    qtest_quit(qts);
}

/* While core 0's CLIC bus clock is gated its registers drop writes. */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clic_gated(void)
{
    QTestState *qts = start();
    uint32_t ctrl0 = qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0);

    qtest_writel(qts, CLIC + CLICINT(16), 0x60000100);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0, ctrl0 & ~CORE0_CLIC_CLK_EN);
    qtest_writel(qts, CLIC + CLICINT(16), 0);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(16)), ==, 0x7fc00100);
    qtest_writel(qts, CLINT + MSIP, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT + MSIP), ==, 0);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0, ctrl0);
    qtest_writel(qts, CLIC + CLICINT(16), 0);
    g_assert_cmphex(qtest_readl(qts, CLIC + CLICINT(16)), ==, 0x1fc00000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clint_reset(void)
{
    QTestState *qts = start();

    for (int i = 0; i < 2; i++) {
        uint32_t base = i ? CLINT_OTHER : CLINT;

        g_assert_cmphex(qtest_readl(qts, base + MSIP), ==, 0);
        g_assert_cmphex(qtest_readl(qts, base + MTIMECMPLO), ==, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, base + MTIMECMPHI), ==, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, base + MTIMELOADLO), ==, 0);
        g_assert_cmphex(qtest_readl(qts, base + MTIMELOADHI), ==, 0);
    }
    /* MTIME_EN is core 0's only, set from reset. */
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECTL), ==, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMECTL), ==, 0);
    qtest_quit(qts);
}

static uint64_t read_mtime(QTestState *qts, uint32_t base)
{
    uint32_t hi = qtest_readl(qts, base + MTIMEHI);
    uint32_t lo = qtest_readl(qts, base + MTIMELO);

    return ((uint64_t)hi << 32) | lo;
}

/*
 * The counter counts CPU_CLK, under core 0's MTIME_EN; core 0 loads it,
 * and both cores see the same count.
 */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clint_counter(void)
{
    QTestState *qts = start();
    uint64_t t0, t1;

    t0 = read_mtime(qts, CLINT);
    qtest_clock_step(qts, 1000 * CPU_CLK_NS);
    t1 = read_mtime(qts, CLINT);
    g_assert_cmpuint(t1 - t0, ==, 1000);
    g_assert_cmpuint(read_mtime(qts, CLINT_OTHER), ==, t1);

    /* Stopped through MTIME_EN; core 1's bit has no effect. */
    qtest_writel(qts, CLINT + MTIMECTL, 0);
    t0 = read_mtime(qts, CLINT);
    qtest_clock_step(qts, 1000 * CPU_CLK_NS);
    g_assert_cmpuint(read_mtime(qts, CLINT), ==, t0);
    qtest_writel(qts, CLINT_OTHER + MTIMECTL, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMECTL), ==, 0);
    qtest_clock_step(qts, 1000 * CPU_CLK_NS);
    g_assert_cmpuint(read_mtime(qts, CLINT), ==, t0);
    qtest_writel(qts, CLINT + MTIMECTL, 1);

    /* Loading: core 0's load registers set the count, core 1's do not. */
    qtest_writel(qts, CLINT + MTIMELOADHI, 0x12345678);
    qtest_writel(qts, CLINT + MTIMELOADLO, 0x9abcdef0);
    g_assert_cmphex(read_mtime(qts, CLINT), ==, 0x123456789abcdef0ull);
    qtest_clock_step(qts, 10 * CPU_CLK_NS);
    g_assert_cmphex(read_mtime(qts, CLINT), ==, 0x123456789abcdefaull);
    qtest_writel(qts, CLINT_OTHER + MTIMELOADLO, 0x55);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMELOADLO), ==, 0x55);
    g_assert_cmphex(read_mtime(qts, CLINT), ==, 0x123456789abcdefaull);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMELOADLO), ==, 0x9abcdef0);

    /* mtimelo and mtimehi are read-only. */
    qtest_writel(qts, CLINT + MTIMELO, 0);
    g_assert_cmphex(read_mtime(qts, CLINT), ==, 0x123456789abcdefaull);
    qtest_quit(qts);
}

/* MTIME_OVF: set when the count reaches all ones, cleared by writing 0. */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clint_overflow(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CLINT + MTIMELOADHI, 0xffffffff);
    qtest_writel(qts, CLINT + MTIMELOADLO, 0xfffffff0);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECTL), ==, 1);
    qtest_clock_step(qts, 8 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECTL), ==, 1);
    qtest_clock_step(qts, 16 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECTL), ==, 3);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMECTL), ==, 2);
    g_assert_cmphex(read_mtime(qts, CLINT), ==, 8);
    qtest_writel(qts, CLINT + MTIMECTL, 3);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECTL), ==, 3);
    qtest_writel(qts, CLINT + MTIMECTL, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECTL), ==, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMECTL), ==, 2);
    qtest_quit(qts);
}

/* MTIME_SAM: a read of one half samples the other for its next read. */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clint_sampling(void)
{
    QTestState *qts = start();
    uint32_t hi;

    qtest_writel(qts, CLINT + MTIMELOADHI, 0x00000005);
    qtest_writel(qts, CLINT + MTIMELOADLO, 0xffffff00);

    /* No sampling: the high half has moved on. */
    qtest_readl(qts, CLINT + MTIMELO);
    qtest_clock_step(qts, 0x200 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMEHI), ==, 6);

    /* Sampling the high half on reading the low half. */
    qtest_writel(qts, CLINT + MTIMELOADHI, 0x00000005);
    qtest_writel(qts, CLINT + MTIMELOADLO, 0xffffff00);
    qtest_writel(qts, CLINT + MTIMECTL, 1 | (1 << 2));
    qtest_readl(qts, CLINT + MTIMELO);
    qtest_clock_step(qts, 0x200 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMEHI), ==, 5);
    /* The sample is used once. */
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMEHI), ==, 6);

    /* Sampling the low half on reading the high half. */
    qtest_writel(qts, CLINT + MTIMECTL, 1 | (2 << 2));
    hi = qtest_readl(qts, CLINT + MTIMEHI);
    g_assert_cmphex(hi, ==, 6);
    qtest_clock_step(qts, 0x100 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMELO), ==, 0x100);

    /* Core 1's sampling mode is its own. */
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMECTL), ==, 0);
    qtest_quit(qts);
}

/* msip, mtimecmp: per core, read and written through either window. */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clint_registers(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CLINT + MSIP, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CLINT + MSIP), ==, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MSIP), ==, 0);
    /* Core 0's software interrupt is pending in its CLIC: interrupt 3. */
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(3)), ==, 1);
    qtest_writel(qts, CLINT + MSIP, 0);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(3)), ==, 0);
    qtest_writel(qts, CLINT_OTHER + MSIP, 1);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MSIP), ==, 1);
    g_assert_cmphex(qtest_readb(qts, CLIC_OTHER + CLICINT(3)), ==, 1);

    qtest_writel(qts, CLINT + MTIMECMPLO, 0x11223344);
    qtest_writel(qts, CLINT + MTIMECMPHI, 0x55667788);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECMPLO), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECMPHI), ==, 0x55667788);
    g_assert_cmphex(qtest_readl(qts, CLINT_OTHER + MTIMECMPLO), ==,
                    0xffffffff);
    /* A byte write changes its byte only. */
    qtest_writeb(qts, CLINT + MTIMECMPLO + 1, 0xaa);
    g_assert_cmphex(qtest_readl(qts, CLINT + MTIMECMPLO), ==, 0x1122aa44);
    qtest_quit(qts);
}

/*
 * The timer interrupt, CLIC interrupt 7, is pending from the count
 * reaching mtimecmp, exactly then, until mtimecmp is raised above it.
 */
/* [spec:nuos:req:emu.esp32p4.clic/test] */
static void test_clint_timer(void)
{
    QTestState *qts = start();
    uint64_t now;

    qtest_writel(qts, CLINT + MTIMECTL, 0);
    qtest_writel(qts, CLINT + MTIMELOADHI, 0);
    qtest_writel(qts, CLINT + MTIMELOADLO, 0);
    qtest_writel(qts, CLINT + MTIMECMPHI, 0);
    qtest_writel(qts, CLINT + MTIMECMPLO, 1000);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(7)), ==, 0);
    qtest_writel(qts, CLINT + MTIMECTL, 1);
    qtest_clock_step(qts, 999 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(7)), ==, 0);
    now = read_mtime(qts, CLINT);
    g_assert_cmpuint(now, ==, 999);
    qtest_clock_step(qts, CPU_CLK_NS);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(7)), ==, 1);
    qtest_clock_step(qts, 100 * CPU_CLK_NS);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(7)), ==, 1);
    qtest_writel(qts, CLINT + MTIMECMPLO, 5000);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(7)), ==, 0);
    /* Core 1's compare value is its own. */
    g_assert_cmphex(qtest_readb(qts, CLIC_OTHER + CLICINT(7)), ==, 0);
    /* Lowering mtimecmp below the count raises it at once. */
    qtest_writel(qts, CLINT + MTIMECMPLO, 0);
    g_assert_cmphex(qtest_readb(qts, CLIC + CLICINT(7)), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/clic/reset", test_clic_reset);
    qtest_add_func("/esp32p4/clic/registers", test_clic_registers);
    qtest_add_func("/esp32p4/clic/byte-access", test_clic_byte_access);
    qtest_add_func("/esp32p4/clic/other-core", test_clic_other_core);
    qtest_add_func("/esp32p4/clic/user-view", test_clic_user_view);
    qtest_add_func("/esp32p4/clic/gated", test_clic_gated);
    qtest_add_func("/esp32p4/clint/reset", test_clint_reset);
    qtest_add_func("/esp32p4/clint/counter", test_clint_counter);
    qtest_add_func("/esp32p4/clint/overflow", test_clint_overflow);
    qtest_add_func("/esp32p4/clint/sampling", test_clint_sampling);
    qtest_add_func("/esp32p4/clint/registers", test_clint_registers);
    qtest_add_func("/esp32p4/clint/timer", test_clint_timer);
    return g_test_run();
}
