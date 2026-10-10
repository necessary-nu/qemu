/*
 * QTest testcase for the RP2350 CoreSight window and trace capture FIFO
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses go through core 0's bus, so these tests act as core 0:
 * core 1's AHB-AP is usable and core 0's own is refused.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"

#define CS              0x40140000
#define ROM             (CS + 0x0000)
#define AHBAP0          (CS + 0x2000)
#define AHBAP1          (CS + 0x4000)
#define TSGEN           (CS + 0x6000)
#define FUNNEL          (CS + 0x7000)
#define TPIU            (CS + 0x8000)
#define CTI             (CS + 0x9000)
#define APBAP           (CS + 0xa000)
#define TRACE           0x50700000

#define ITCTRL          0xf00
#define CLAIMSET        0xfa0
#define CLAIMCLR        0xfa4
#define LSR             0xfb4
#define AUTHSTATUS      0xfb8
#define DEVARCH         0xfbc
#define DEVID           0xfc8
#define DEVTYPE         0xfcc
#define PIDR4           0xfd0
#define PIDR0           0xfe0
#define CIDR0           0xff0

#define CSW             0xd00
#define TAR             0xd04
#define DRW             0xd0c
#define BD(n)           (0xd10 + 4 * (n))
#define TRR             0xd24
#define CFG             0xdf4
#define BASE            0xdf8
#define IDR             0xdfc

#define SRAM            0x20000000
/* The AP these tests drive and its second logical AP, both core 1's. */
#define AP              AHBAP1
#define AP2             (AHBAP1 + 0x1000)
#define SYSCLK_HZ       150000000ull

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_clocks_init(qts);
    return qts;
}

typedef struct Ident {
    uint64_t base;
    uint8_t cls;
    uint16_t designer;
    uint16_t part;
    uint8_t revision;
    uint32_t devarch;
    uint32_t devtype;
    uint32_t devid;
} Ident;

/*
 * What a probe reads from an RP2350 (pyOCD's walk of a Pico 2: designer
 * is continuation code << 8 | JEP106 code, 0x43b Arm, 0x913 Raspberry Pi).
 */
static const Ident rom_ident = {
    ROM, 0x9, 0x913, 0x004, 0, 0x47700af7, 0x00, 0,
};

static const Ident rom_children[] = {
    { AHBAP0, 0x9, 0x43b, 0x9e3, 3, 0x47700a17, 0x00, 0 },
    { AHBAP1, 0x9, 0x43b, 0x9e3, 3, 0x47700a17, 0x00, 0 },
    { TSGEN,  0xf, 0x43b, 0x193, 0, 0, 0, 0 },
    { FUNNEL, 0x9, 0x43b, 0x9eb, 2, 0, 0x12, 0x34 },
    { TPIU,   0x9, 0x43b, 0x9e7, 2, 0, 0x11, 0x20 },
    { CTI,    0x9, 0x43b, 0x9ed, 3, 0x47701a14, 0x14, 0x01040500 },
};

static const Ident apbap_ident = {
    APBAP, 0x9, 0x43b, 0x9e2, 2, 0x47700a17, 0x00, 0,
};

static void check_ident(QTestState *qts, const Ident *id)
{
    uint32_t pidr[5], cidr[4];
    int i;

    for (i = 0; i < 4; i++) {
        pidr[i] = qtest_readl(qts, id->base + PIDR0 + 4 * i);
        cidr[i] = qtest_readl(qts, id->base + CIDR0 + 4 * i);
    }
    pidr[4] = qtest_readl(qts, id->base + PIDR4);
    for (i = 1; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, id->base + PIDR4 + 4 * i), ==, 0);
    }

    g_assert_cmphex(cidr[0], ==, 0x0d);
    g_assert_cmphex(cidr[1], ==, id->cls << 4);
    g_assert_cmphex(cidr[2], ==, 0x05);
    g_assert_cmphex(cidr[3], ==, 0xb1);

    g_assert_cmphex(pidr[0] | (pidr[1] & 0xf) << 8, ==, id->part);
    g_assert_true(pidr[2] & 0x8);
    g_assert_cmphex((pidr[4] & 0xf) << 8 | (pidr[2] & 0x7) << 4 |
                    pidr[1] >> 4, ==, id->designer);
    g_assert_cmphex(pidr[2] >> 4, ==, id->revision);
    g_assert_cmphex(pidr[3], ==, 0);
    g_assert_cmphex(pidr[4] >> 4, ==, 0);

    g_assert_cmphex(qtest_readl(qts, id->base + DEVARCH), ==, id->devarch);
    g_assert_cmphex(qtest_readl(qts, id->base + DEVTYPE), ==, id->devtype);
    g_assert_cmphex(qtest_readl(qts, id->base + DEVID), ==, id->devid);
    /* SoC-600 has no software lock. */
    g_assert_cmphex(qtest_readl(qts, id->base + LSR), ==, 0);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_rom_walk(void)
{
    QTestState *qts = start();
    int i;

    check_ident(qts, &rom_ident);
    g_assert_cmphex(qtest_readl(qts, ROM + AUTHSTATUS), ==, 0xff);

    for (i = 0; ; i++) {
        uint32_t e = qtest_readl(qts, ROM + 4 * i);

        if (e == 0) {
            break;
        }
        g_assert_cmpint(i, <, ARRAY_SIZE(rom_children));
        /* Class 0x9 entry: present, no power domain. */
        g_assert_cmphex(e & 0xfff, ==, 0x3);
        g_assert_cmphex(ROM + (e & ~0xfffu), ==, rom_children[i].base);
        if (rom_children[i].base == AHBAP0) {
            /* Core 0's own AP: a bus fault, which qtest reads as 0. */
            g_assert_cmphex(qtest_readl(qts, AHBAP0 + CIDR0), ==, 0);
            continue;
        }
        check_ident(qts, &rom_children[i]);
    }
    g_assert_cmpint(i, ==, ARRAY_SIZE(rom_children));

    /* The APB-AP is not listed but still identifies itself. */
    check_ident(qts, &apbap_ident);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_memap_ids(void)
{
    QTestState *qts = start();
    uint64_t aps[] = { AHBAP1, AHBAP1 + 0x1000 };
    int i;

    for (i = 0; i < ARRAY_SIZE(aps); i++) {
        g_assert_cmphex(qtest_readl(qts, aps[i] + IDR), ==, 0x34770008);
        g_assert_cmphex(qtest_readl(qts, aps[i] + CFG), ==, 0x000101a0);
        g_assert_cmphex(qtest_readl(qts, aps[i] + BASE), ==, 0xe00ff003);
        /* Secure, privileged word transfers; both AP enables high. */
        g_assert_cmphex(qtest_readl(qts, aps[i] + CSW), ==, 0x03800042);
        g_assert_cmphex(qtest_readl(qts, aps[i] + TRR), ==, 0);
        g_assert_cmphex(qtest_readl(qts, aps[i] + CLAIMSET), ==, 0x3);
        g_assert_cmphex(qtest_readl(qts, aps[i] + AUTHSTATUS), ==, 0xff);
    }
    g_assert_cmphex(qtest_readl(qts, APBAP + IDR), ==, 0x24770006);
    g_assert_cmphex(qtest_readl(qts, APBAP + BASE), ==, 0x00000002);
    g_assert_cmphex(qtest_readl(qts, APBAP + CSW), ==, 0x30800042);
    g_assert_cmphex(qtest_readl(qts, APBAP + 0x1000 + IDR), ==, 0x24770006);

    /* The two logical APs keep their own writable registers. */
    qtest_writel(qts, AHBAP1 + TAR, 0x12345678);
    qtest_writel(qts, AHBAP1 + CLAIMSET, 0x2);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + TAR), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + CLAIMCLR), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + 0x1000 + CLAIMCLR), ==, 0);
    qtest_writel(qts, AHBAP1 + CLAIMCLR, 0x3);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + CLAIMCLR), ==, 0);

    /* Core 0 may not touch its own AP, in either logical view. */
    qtest_writel(qts, AHBAP0 + TAR, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, AHBAP0 + TAR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AHBAP0 + 0x1000 + IDR), ==, 0);

    /* CSW: only the configurable fields take writes. */
    qtest_writel(qts, AHBAP1 + CSW, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + CSW), ==, 0x5f838077);
    qtest_writel(qts, APBAP + CSW, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, APBAP + CSW), ==, 0x70830072);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_memap_transfers(void)
{
    QTestState *qts = start();

    /* Word writes with single auto-increment. */
    qtest_writel(qts, AP + CSW, 0x03000012);
    qtest_writel(qts, AP + TAR, SRAM + 0x100);
    qtest_writel(qts, AP + DRW, 0x11223344);
    qtest_writel(qts, AP + DRW, 0x55667788);
    g_assert_cmphex(qtest_readl(qts, AP + TAR), ==, SRAM + 0x108);
    g_assert_cmphex(qtest_readl(qts, SRAM + 0x100), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, SRAM + 0x104), ==, 0x55667788);

    /* Byte reads come back on the byte lane the address selects. */
    qtest_writel(qts, AP + CSW, 0x03000010);
    qtest_writel(qts, AP + TAR, SRAM + 0x101);
    g_assert_cmphex(qtest_readl(qts, AP + DRW), ==, 0x00003300);
    g_assert_cmphex(qtest_readl(qts, AP + DRW), ==, 0x00220000);
    qtest_writel(qts, AP + DRW, 0xaa000000);
    g_assert_cmphex(qtest_readl(qts, SRAM + 0x100), ==, 0xaa223344);

    /* TAR increments within a 1 KiB window. */
    qtest_writel(qts, AP + CSW, 0x03000012);
    qtest_writel(qts, AP + TAR, SRAM + 0x3fc);
    qtest_readl(qts, AP + DRW);
    g_assert_cmphex(qtest_readl(qts, AP + TAR), ==, SRAM);

    /* Banked data registers address TAR's 16-byte block, without moving. */
    qtest_writel(qts, AP2 + TAR, SRAM + 0x104);
    g_assert_cmphex(qtest_readl(qts, AP2 + BD(0)), ==, 0xaa223344);
    g_assert_cmphex(qtest_readl(qts, AP2 + BD(1)), ==, 0x55667788);
    qtest_writel(qts, AP2 + BD(3), 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, SRAM + 0x10c), ==, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, AP2 + TAR), ==, SRAM + 0x104);

    /* Direct access registers map TAR's 1 KiB block. */
    qtest_writel(qts, AP2 + TAR, SRAM + 0x2a8);
    g_assert_cmphex(qtest_readl(qts, AP2 + 0x10c), ==, 0xcafef00d);
    qtest_writel(qts, AP2 + 0x3fc, 0x0badf00d);
    g_assert_cmphex(qtest_readl(qts, SRAM + 0x3fc), ==, 0x0badf00d);

    /* Each AP sees its core's PPB: VTOR as reset loaded it. */
    qtest_writel(qts, AP + TAR, 0xe000ed08);
    g_assert_cmphex(qtest_readl(qts, AP + DRW), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_memap_errors(void)
{
    QTestState *qts = start();

    /* An AHB-AP may not reach an AHB-AP through the window. */
    qtest_writel(qts, AP + TAR, AP2 + IDR);
    qtest_readl(qts, AP + DRW);
    g_assert_cmphex(qtest_readl(qts, AP + TRR), ==, 1);
    g_assert_cmphex(qtest_readl(qts, AP + TAR), ==, AP2 + IDR);

    /* ERRSTOP: a logged error stops transfers until TRR is cleared. */
    qtest_writel(qts, AP + CSW, 0x03020002);
    qtest_writel(qts, AP + TAR, SRAM);
    qtest_writel(qts, AP + DRW, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SRAM), ==, 0);
    qtest_writel(qts, AP + TRR, 1);
    g_assert_cmphex(qtest_readl(qts, AP + TRR), ==, 0);
    qtest_writel(qts, AP + DRW, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SRAM), ==, 0x12345678);

    /*
     * A self-hosted transfer to an APB address deadlocks the bridge; the
     * transfer itself completes once the bridge times out.
     */
    qtest_writel(qts, AP + CSW, 0x03000002);
    qtest_writel(qts, AP + TAR, 0x400e0000);
    qtest_writel(qts, AP + DRW, 0x5a5a5a5a);
    g_assert_cmphex(qtest_readl(qts, 0x400e0000), ==, 0x5a5a5a5a);
    g_assert_cmphex(qtest_readl(qts, AP + TRR), ==, 0);

    /* Reserved transfer size. */
    qtest_writel(qts, AP2 + CSW, 0x03000003);
    qtest_readl(qts, AP2 + DRW);
    g_assert_cmphex(qtest_readl(qts, AP2 + TRR), ==, 1);

    /* The APB-AP's Debug Module is absent: transfers complete, RAZ/WI. */
    qtest_writel(qts, APBAP + CSW, 0x30000012);
    qtest_writel(qts, APBAP + TAR, 0x40);
    g_assert_cmphex(qtest_readl(qts, APBAP + DRW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, APBAP + TAR), ==, 0x44);
    g_assert_cmphex(qtest_readl(qts, APBAP + TRR), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_tsgen(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x000), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==, 0);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==, 0);

    /* Counts clk_sys while enabled. */
    qtest_writel(qts, TSGEN + 0x000, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==,
                    SYSCLK_HZ * 1000 / 1000000000);
    qtest_writel(qts, TSGEN + 0x000, 0);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==,
                    SYSCLK_HZ * 1000 / 1000000000);

    /* The 64-bit value is loaded when the upper half is written. */
    qtest_writel(qts, TSGEN + 0x008, 0xfffffff0);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==,
                    SYSCLK_HZ * 1000 / 1000000000);
    qtest_writel(qts, TSGEN + 0x00c, 0x1);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==, 0xfffffff0);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x00c), ==, 0x1);
    qtest_writel(qts, TSGEN + 0x000, 3);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x008), ==,
                    0xfffffff0 + SYSCLK_HZ * 1000 / 1000000000 - (1ull << 32));
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x00c), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x000), ==, 3);

    qtest_writel(qts, TSGEN + 0x020, 150000000);
    g_assert_cmphex(qtest_readl(qts, TSGEN + 0x020), ==, 150000000);
    qtest_writel(qts, TSGEN + ITCTRL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TSGEN + ITCTRL), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_funnel(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, FUNNEL + 0x000), ==, 0x300);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + 0x004), ==, 0);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + CLAIMSET), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + AUTHSTATUS), ==, 0);

    /* Priorities only change while every port is disabled. */
    qtest_writel(qts, FUNNEL + 0x004, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + 0x004), ==, 0xfff);
    qtest_writel(qts, FUNNEL + 0x000, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + 0x000), ==, 0x1f0f);
    qtest_writel(qts, FUNNEL + 0x004, 0x123);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + 0x004), ==, 0xfff);

    /* A byte store is replicated across the word. */
    qtest_writeb(qts, FUNNEL + 0x000, 0x02);
    g_assert_cmphex(qtest_readl(qts, FUNNEL + 0x000), ==, 0x0202);
    g_assert_cmphex(qtest_readb(qts, FUNNEL + 0x001), ==, 0x02);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_tpiu(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, TPIU + 0x000), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x004), ==, 0x1);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x100), ==, 0x11f);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x200), ==, 0x3000f);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x300), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x304), ==, 0x1000);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x308), ==, 0x40);

    /* 32-bit port, for the capture FIFO. */
    qtest_writel(qts, TPIU + 0x004, 0x80000000);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x004), ==, 0x80000000);

    /* A manual flush completes at once: FOnMan and FlInProg read 0. */
    qtest_writel(qts, TPIU + 0x304, 0x1041);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x304), ==, 0x1001);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x300) & 1, ==, 0);
    qtest_writel(qts, TPIU + 0x304, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x304), ==, 0xb733);

    qtest_writel(qts, TPIU + 0x104, 0x1ff);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x104), ==, 0xff);
    qtest_writel(qts, TPIU + 0x108, 0xff);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x108), ==, 0x1f);
    qtest_writel(qts, TPIU + 0x308, 0xffff);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x308), ==, 0xfff);
    qtest_writel(qts, TPIU + 0x404, 0xffff);
    g_assert_cmphex(qtest_readl(qts, TPIU + 0x404), ==, 0xff);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_cti(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CTI + 0x140), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x000), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CTI + AUTHSTATUS), ==, 0xf);

    /* Five trigger mapping registers each way; the rest are RAZ/WI. */
    qtest_writel(qts, CTI + 0x020 + 4 * 4, 0xff);
    qtest_writel(qts, CTI + 0x020 + 4 * 5, 0xff);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x020 + 4 * 4), ==, 0xf);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x020 + 4 * 5), ==, 0);
    qtest_writel(qts, CTI + 0x0a0 + 4 * 2, 0x4);
    qtest_writel(qts, CTI + 0x0a0 + 4 * 6, 0xf);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x0a0 + 4 * 6), ==, 0);

    /* Channel 2 set by software reaches trigger 2 once the CTI is on. */
    qtest_writel(qts, CTI + 0x014, 0x4);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x014), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x134), ==, 0);
    qtest_writel(qts, CTI + 0x000, 1);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x134), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x13c), ==, 0x4);

    /* The gate stops the channel leaving, not the local trigger. */
    qtest_writel(qts, CTI + 0x140, 0xb);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x13c), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x134), ==, 0x4);

    qtest_writel(qts, CTI + 0x018, 0x4);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x014), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CTI + 0x134), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_trace_fifo(void)
{
    QTestState *qts = start();

    /* Held flushed out of reset. */
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x0), ==, 0x1);
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x4), ==, 0);

    /* Atomic aliases on CTRL_STATUS. */
    qtest_writel(qts, TRACE + 0x3000, 0x1);
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x0), ==, 0);
    qtest_writel(qts, TRACE + 0x2000, 0x1);
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x0), ==, 0x1);
    qtest_writel(qts, TRACE + 0x1000, 0x1);
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x0), ==, 0);

    /* Capturing with nothing upstream: the FIFO stays empty. */
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x4), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x0), ==, 0);
    qtest_writel(qts, TRACE + 0x0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TRACE + 0x0), ==, 0x1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-coresight-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/coresight/rom-walk", test_rom_walk);
    qtest_add_func("/rp2350/coresight/memap-ids", test_memap_ids);
    qtest_add_func("/rp2350/coresight/memap-transfers", test_memap_transfers);
    qtest_add_func("/rp2350/coresight/memap-errors", test_memap_errors);
    qtest_add_func("/rp2350/coresight/tsgen", test_tsgen);
    qtest_add_func("/rp2350/coresight/funnel", test_funnel);
    qtest_add_func("/rp2350/coresight/tpiu", test_tpiu);
    qtest_add_func("/rp2350/coresight/cti", test_cti);
    qtest_add_func("/rp2350/coresight/trace-fifo", test_trace_fifo);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
