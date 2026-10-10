/*
 * QTest testcase for the RP2350 Cortex-M33 ACTLR and SysTick calibration
 * and reference clock
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The PPB is only on each core's own bus, so these tests reach core 1's
 * through its AHB-AP, as a debugger does. Core 1's SysTick reference is
 * the TICKS PROC1 generator.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"

#define AHBAP1          0x40144000
#define CSW             0xd00
#define TAR             0xd04
#define DRW             0xd0c
#define TRR             0xd24

/* 32-bit, no address increment, privileged Secure data access. */
#define CSW_PRIV        0x03000002
/* The same, unprivileged. */
#define CSW_USER        0x01000002

#define ACTLR           0xe000e008
#define ACTLR_NS        0xe002e008
#define SYST_CSR        0xe000e010
#define SYST_RVR        0xe000e014
#define SYST_CVR        0xe000e018
#define SYST_CALIB      0xe000e01c
#define SYST_CALIB_NS   0xe002e01c
#define CSR_ENABLE      (1u << 0)
#define CSR_CLKSOURCE   (1u << 2)

#define TICKS           0x40108000
#define TICK_CTRL(n)    (TICKS + (n) * 0xc)
#define TICK_CYCLES(n)  (TICKS + (n) * 0xc + 4)
#define TICK_PROC0      0
#define TICK_PROC1      1

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_clocks_init(qts);
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

static void tick_start(QTestState *qts, int tick, uint32_t cycles)
{
    qtest_writel(qts, TICK_CTRL(tick), 0);
    qtest_writel(qts, TICK_CYCLES(tick), cycles);
    qtest_writel(qts, TICK_CTRL(tick), 1);
}

/* [spec:nuos:req:emu.machine+1/test] */
static void test_actlr(void)
{
    QTestState *qts = start();

    g_assert_cmphex(ap_read(qts, ACTLR), ==, 0);
    g_assert_cmphex(ap_read(qts, ACTLR_NS), ==, 0);

    /* EXTEXCLALL, DISITMATBFLUSH, FPEXCODIS, DISOOFP, DISFOLD, DISMCYCINT */
    ap_write(qts, ACTLR, 0xffffffff);
    g_assert_cmphex(ap_read(qts, ACTLR), ==, 0x20001605);
    g_assert_cmphex(ap_read(qts, ACTLR_NS), ==, 0);

    /* Banked: the Non-secure copy is separate. */
    ap_write(qts, ACTLR_NS, 0x20000000);
    g_assert_cmphex(ap_read(qts, ACTLR_NS), ==, 0x20000000);
    g_assert_cmphex(ap_read(qts, ACTLR), ==, 0x20001605);

    /* A system reset clears both banks. */
    qtest_system_reset(qts);
    qtest_writel(qts, AHBAP1 + CSW, CSW_PRIV);
    g_assert_cmphex(ap_read(qts, ACTLR), ==, 0);
    g_assert_cmphex(ap_read(qts, ACTLR_NS), ==, 0);

    /* Privileged only. */
    qtest_writel(qts, AHBAP1 + CSW, CSW_USER);
    ap_read(qts, ACTLR);
    g_assert_cmphex(qtest_readl(qts, AHBAP1 + TRR), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.clocks/test] */
static void test_systick(void)
{
    QTestState *qts = start();
    uint32_t cvr, counted;

    /* TENMS hardwired to 100,000; NOREF and SKEW clear. */
    g_assert_cmphex(ap_read(qts, SYST_CALIB), ==, 100000);
    g_assert_cmphex(ap_read(qts, SYST_CALIB_NS), ==, 100000);

    /* With PROC1 stopped, core 1's reference SysTick does not count. */
    ap_write(qts, SYST_RVR, 0xffffff);
    ap_write(qts, SYST_CVR, 0);
    ap_write(qts, SYST_CSR, CSR_ENABLE);
    g_assert_cmphex(ap_read(qts, SYST_CSR), ==, CSR_ENABLE);
    tick_start(qts, TICK_PROC0, 12);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(ap_read(qts, SYST_CVR), ==, 0);

    /*
     * PROC1 at 12 clk_ref cycles: one tick per microsecond. The steps
     * land mid-tick so that the count read is not on a tick edge.
     */
    tick_start(qts, TICK_PROC1, 12);
    qtest_clock_step(qts, 1500);
    cvr = ap_read(qts, SYST_CVR);
    g_assert_cmphex(cvr, ==, 0xffffff);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(ap_read(qts, SYST_CVR), ==, cvr - 1000);

    /* 48 cycles: one tick every 4 microseconds. */
    tick_start(qts, TICK_PROC1, 48);
    cvr = ap_read(qts, SYST_CVR);
    qtest_clock_step(qts, 4000 * 1000 + 2000);
    g_assert_cmphex(ap_read(qts, SYST_CVR), ==, cvr - 1000);

    /* Stopping PROC1 holds the count. */
    qtest_writel(qts, TICK_CTRL(TICK_PROC1), 0);
    cvr = ap_read(qts, SYST_CVR);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(ap_read(qts, SYST_CVR), ==, cvr);

    /* The processor clock is unaffected by PROC1: 150 MHz. */
    ap_write(qts, SYST_CSR, CSR_ENABLE | CSR_CLKSOURCE);
    cvr = ap_read(qts, SYST_CVR);
    qtest_clock_step(qts, 10 * 1000);
    tick_start(qts, TICK_PROC1, 12);
    qtest_clock_step(qts, 10 * 1000 + 3);
    /* 3000 cycles, give or take the phase of the switch to cpuclk. */
    counted = cvr - ap_read(qts, SYST_CVR);
    g_assert_cmpuint(counted, >=, 2999);
    g_assert_cmpuint(counted, <=, 3001);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-m33-actlr-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/m33/actlr", test_actlr);
    qtest_add_func("/rp2350/m33/systick", test_systick);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
