/*
 * QTest testcase for the RP2350 Cortex-M33 debug components and PPB ROM
 * table, as a debugger reaches them through an AHB-AP
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses go through core 0's bus, so these tests drive core 1's
 * AHB-AP and see core 1's PPB, the way a probe walks from the AP's BASE.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define AHBAP1          0x40144000
#define CSW             0xd00
#define TAR             0xd04
#define DRW             0xd0c
#define TRR             0xd24
#define BASE            0xdf8

/* 32-bit, no address increment, privileged Secure data access. */
#define CSW_PRIV        0x03000002
/* The same, unprivileged. */
#define CSW_USER        0x01000002

#define DEVARCH         0xfbc
#define DEVID           0xfc8
#define DEVTYPE         0xfcc
#define PIDR4           0xfd0
#define PIDR0           0xfe0
#define CIDR0           0xff0

#define SCS             0xe000e000
#define CPUID           0xe000ed00
#define DWT_CTRL        0xe0001000
#define DWT_COMP0       0xe0001020
#define FP_CTRL         0xe0002000
#define CTICONTROL      0xe0042000
#define CTIAPPSET       0xe0042014
#define CTIOUTEN0       0xe00420a0
#define CTITRIGOUTSTATUS 0xe0042134

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    qtest_writel(qts, AHBAP1 + CSW, CSW_PRIV);
    return qts;
}

static uint32_t ap_read(QTestState *qts, uint32_t addr)
{
    qtest_writel(qts, AHBAP1 + TAR, addr);
    return qtest_readl(qts, AHBAP1 + DRW);
}

static void ap_write(QTestState *qts, uint32_t addr, uint32_t v)
{
    qtest_writel(qts, AHBAP1 + TAR, addr);
    qtest_writel(qts, AHBAP1 + DRW, v);
}

typedef struct Ident {
    uint32_t base;
    uint8_t cls;
    uint16_t part;
    uint8_t revision;
    uint32_t devarch;
    uint32_t devtype;
    uint32_t devid;
} Ident;

/* The Cortex-M33 r1p0 components, in ROM table order. */
static const Ident components[] = {
    { 0xe000e000, 0x9, 0xd21, 0, 0x47702a04, 0x00, 0 },          /* SCS */
    { 0xe0001000, 0x9, 0xd21, 0, 0x47701a02, 0x00, 0 },          /* DWT */
    { 0xe0002000, 0x9, 0xd21, 0, 0x47701a03, 0x00, 0 },          /* FPB */
    { 0xe0000000, 0x9, 0xd21, 0, 0x47701a01, 0x43, 0 },          /* ITM */
    { 0 },                                                       /* TPIU */
    { 0xe0041000, 0x9, 0xd21, 2, 0x47724a13, 0x13, 0 },          /* ETM */
    { 0xe0042000, 0x9, 0xd21, 0, 0x47701a14, 0x14, 0x00040800 }, /* CTI */
    { 0 },                                                       /* MTB */
};

static void check_ident(QTestState *qts, const Ident *id)
{
    uint32_t pidr[5], cidr[4];
    int i;

    for (i = 0; i < 4; i++) {
        pidr[i] = ap_read(qts, id->base + PIDR0 + 4 * i);
        cidr[i] = ap_read(qts, id->base + CIDR0 + 4 * i);
    }
    pidr[4] = ap_read(qts, id->base + PIDR4);

    g_assert_cmphex(cidr[0], ==, 0x0d);
    g_assert_cmphex(cidr[1], ==, id->cls << 4);
    g_assert_cmphex(cidr[2], ==, 0x05);
    g_assert_cmphex(cidr[3], ==, 0xb1);
    g_assert_cmphex(pidr[0] | (pidr[1] & 0xf) << 8, ==, id->part);
    g_assert_true(pidr[2] & 0x8);
    /* Arm: JEP106 continuation 4, code 0x3b. */
    g_assert_cmphex((pidr[4] & 0xf) << 8 | (pidr[2] & 0x7) << 4 |
                    pidr[1] >> 4, ==, 0x43b);
    g_assert_cmphex(pidr[2] >> 4, ==, id->revision);
    g_assert_cmphex(pidr[3], ==, 0);
    if (id->cls == 0x9) {
        g_assert_cmphex(ap_read(qts, id->base + DEVARCH), ==, id->devarch);
        g_assert_cmphex(ap_read(qts, id->base + DEVTYPE), ==, id->devtype);
        if (id->base != SCS) {
            g_assert_cmphex(ap_read(qts, id->base + DEVID), ==, id->devid);
        }
    }
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_rom_walk(void)
{
    QTestState *qts = start();
    uint32_t base = qtest_readl(qts, AHBAP1 + BASE);
    Ident rom = { base & ~0xfffu, 0x1, 0x4c9, 0, 0, 0, 0 };
    int i;

    /* BASE: the PPB ROM table, ADIv5 format, present. */
    g_assert_cmphex(base, ==, 0xe00ff003);
    check_ident(qts, &rom);
    g_assert_cmphex(ap_read(qts, rom.base + 0xfcc), ==, 1);

    for (i = 0; ; i++) {
        uint32_t e = ap_read(qts, rom.base + 4 * i);

        if (e == 0) {
            break;
        }
        g_assert_cmpint(i, <, ARRAY_SIZE(components));
        /* 32-bit format; the TPIU and MTB slots are not present. */
        g_assert_cmphex(e & 0x2, ==, 0x2);
        if (!components[i].base) {
            g_assert_cmphex(e & 1, ==, 0);
            continue;
        }
        g_assert_cmphex(e & 1, ==, 1);
        g_assert_cmphex((uint32_t)(rom.base + (e & ~0xfffu)), ==,
                        components[i].base);
        check_ident(qts, &components[i]);
    }
    g_assert_cmpint(i, ==, ARRAY_SIZE(components));
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + TRR), ==, 0);

    /* The core behind it is a Cortex-M33 r1p0. */
    g_assert_cmphex(ap_read(qts, CPUID), ==, 0x411fd210);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.coresight/test] */
static void test_config(void)
{
    QTestState *qts = start();

    g_assert_cmphex(ap_read(qts, DWT_CTRL), ==, 0x40741824);
    ap_write(qts, DWT_COMP0, 0x20000100);
    g_assert_cmphex(ap_read(qts, DWT_COMP0), ==, 0x20000100);

    g_assert_cmphex(ap_read(qts, FP_CTRL), ==, 0x10000080);
    ap_write(qts, FP_CTRL, 0x1);
    g_assert_cmphex(ap_read(qts, FP_CTRL), ==, 0x10000080);
    ap_write(qts, FP_CTRL, 0x3);
    g_assert_cmphex(ap_read(qts, FP_CTRL), ==, 0x10000081);

    ap_write(qts, CTIOUTEN0, 0x1);
    ap_write(qts, CTIAPPSET, 0x1);
    g_assert_cmphex(ap_read(qts, CTITRIGOUTSTATUS), ==, 0);
    ap_write(qts, CTICONTROL, 0x1);
    g_assert_cmphex(ap_read(qts, CTITRIGOUTSTATUS), ==, 0x1);

    /* Identification registers are read-only. */
    ap_write(qts, DWT_CTRL + PIDR0, 0);
    g_assert_cmphex(ap_read(qts, DWT_CTRL + PIDR0), ==, 0x21);

    /* The PPB refuses an unprivileged transfer. */
    qtest_writel(qts, AHBAP1 + CSW, CSW_USER);
    ap_read(qts, DWT_CTRL);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + TRR), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-m33-debug-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/m33-debug/rom-walk", test_rom_walk);
    qtest_add_func("/rp2350/m33-debug/config", test_config);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
