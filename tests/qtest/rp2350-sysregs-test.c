/*
 * QTest testcase for the RP2350 SYSINFO, SYSCFG, TBMAN, glitch detector
 * and DFT blocks
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define SYSINFO         0x40000000
#define CHIP_ID         (SYSINFO + 0x00)
#define PACKAGE_SEL     (SYSINFO + 0x04)
#define PLATFORM        (SYSINFO + 0x08)
#define GITREF_RP2350   (SYSINFO + 0x14)

#define SYSCFG                  0x40008000
#define PROC_CONFIG             (SYSCFG + 0x00)
#define PROC_IN_SYNC_BYPASS     (SYSCFG + 0x04)
#define PROC_IN_SYNC_BYPASS_HI  (SYSCFG + 0x08)
#define DBGFORCE                (SYSCFG + 0x0c)
#define MEMPOWERDOWN            (SYSCFG + 0x10)
#define AUXCTRL                 (SYSCFG + 0x14)

#define TBMAN_PLATFORM  0x40160000

#define GLITCH          0x40158000
#define GD_ARM          (GLITCH + 0x00)
#define GD_DISARM       (GLITCH + 0x04)
#define GD_SENSITIVITY  (GLITCH + 0x08)
#define GD_LOCK         (GLITCH + 0x0c)
#define GD_TRIG_STATUS  (GLITCH + 0x10)
#define GD_TRIG_FORCE   (GLITCH + 0x14)

#define DFT             0x40150000

#define XOR             0x1000
#define SET             0x2000
#define CLR             0x3000

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_sysinfo(void)
{
    QTestState *qts = start();

    /* An RP2350A at A4: revision 3, part 4, Raspberry Pi's JEP-106 id. */
    g_assert_cmphex(qtest_readl(qts, CHIP_ID), ==, 0x30004927);
    /* QFN-60. */
    g_assert_cmphex(qtest_readl(qts, PACKAGE_SEL), ==, 1);
    /* ASIC, not FPGA or simulation. */
    g_assert_cmphex(qtest_readl(qts, PLATFORM), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, GITREF_RP2350), ==, 0);

    /* Read-only, through the plain window and the aliases. */
    qtest_writel(qts, CHIP_ID, 0);
    qtest_writel(qts, PACKAGE_SEL + CLR, 1);
    qtest_writel(qts, PLATFORM + SET, 0x1f);
    qtest_writel(qts, GITREF_RP2350, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CHIP_ID), ==, 0x30004927);
    g_assert_cmphex(qtest_readl(qts, PACKAGE_SEL), ==, 1);
    g_assert_cmphex(qtest_readl(qts, PLATFORM), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, GITREF_RP2350), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CHIP_ID + XOR), ==, 0x30004927);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_tbman(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, TBMAN_PLATFORM), ==, 0x1);
    qtest_writel(qts, TBMAN_PLATFORM, 0x6);
    g_assert_cmphex(qtest_readl(qts, TBMAN_PLATFORM), ==, 0x1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_syscfg(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, PROC_CONFIG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS_HI), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DBGFORCE), ==, 0x6);
    g_assert_cmphex(qtest_readl(qts, MEMPOWERDOWN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AUXCTRL), ==, 0);

    qtest_writel(qts, PROC_CONFIG, 0x3);
    g_assert_cmphex(qtest_readl(qts, PROC_CONFIG), ==, 0);

    qtest_writel(qts, PROC_IN_SYNC_BYPASS, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS), ==, 0xdeadbeef);
    qtest_writel(qts, PROC_IN_SYNC_BYPASS + CLR, 0xdead0000);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS), ==, 0x0000beef);

    /* Bits 23:16 are reserved. */
    qtest_writel(qts, PROC_IN_SYNC_BYPASS_HI, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS_HI), ==, 0xff00ffff);
    qtest_writel(qts, PROC_IN_SYNC_BYPASS_HI + XOR, 0x0f00000f);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS_HI), ==, 0xf000fff0);

    /* SWDO (bit 0) is read-only; ATTACH, SWCLK and SWDI are RW. */
    qtest_writel(qts, DBGFORCE, 0xf);
    g_assert_cmphex(qtest_readl(qts, DBGFORCE), ==, 0xe);
    qtest_writel(qts, DBGFORCE + CLR, 0x8);
    g_assert_cmphex(qtest_readl(qts, DBGFORCE), ==, 0x6);

    qtest_writel(qts, MEMPOWERDOWN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, MEMPOWERDOWN), ==, 0x1fff);
    qtest_writel(qts, MEMPOWERDOWN + CLR, 0x1fff);
    g_assert_cmphex(qtest_readl(qts, MEMPOWERDOWN), ==, 0);

    qtest_writel(qts, AUXCTRL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AUXCTRL), ==, 0xff);
    qtest_writel(qts, AUXCTRL + CLR, 0xf0);
    g_assert_cmphex(qtest_readl(qts, AUXCTRL), ==, 0x0f);

    /* A system reset restores every reset value. */
    qtest_system_reset(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PROC_IN_SYNC_BYPASS_HI), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DBGFORCE), ==, 0x6);
    g_assert_cmphex(qtest_readl(qts, AUXCTRL), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_glitch_registers(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, GD_ARM), ==, 0x5bad);
    g_assert_cmphex(qtest_readl(qts, GD_DISARM), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_SENSITIVITY), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_LOCK), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_FORCE), ==, 0);

    qtest_writel(qts, GD_DISARM, 0xffffdcaf);
    g_assert_cmphex(qtest_readl(qts, GD_DISARM), ==, 0xdcaf);
    qtest_writel(qts, GD_SENSITIVITY, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GD_SENSITIVITY), ==, 0xff00ffff);
    qtest_writel(qts, GD_SENSITIVITY, 0xde00a659);
    g_assert_cmphex(qtest_readl(qts, GD_SENSITIVITY), ==, 0xde00a659);

    /* Disarmed, a forced trigger only records itself. */
    qtest_writel(qts, GD_TRIG_FORCE, 0x5);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_FORCE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x5);
    qtest_writel(qts, GD_TRIG_FORCE + SET, 0x2);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x7);
    /* Write 1 to clear. */
    qtest_writel(qts, GD_TRIG_STATUS, 0x4);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x3);
    qtest_writel(qts, GD_TRIG_STATUS, 0xf);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0);

    /* LOCK freezes ARM, DISARM, SENSITIVITY and LOCK itself. */
    qtest_writel(qts, GD_LOCK, 0x1ff);
    g_assert_cmphex(qtest_readl(qts, GD_LOCK), ==, 0xff);
    qtest_writel(qts, GD_ARM, 0);
    qtest_writel(qts, GD_DISARM, 0);
    qtest_writel(qts, GD_SENSITIVITY, 0);
    qtest_writel(qts, GD_LOCK, 0);
    qtest_writel(qts, GD_LOCK + CLR, 0xff);
    g_assert_cmphex(qtest_readl(qts, GD_ARM), ==, 0x5bad);
    g_assert_cmphex(qtest_readl(qts, GD_DISARM), ==, 0xdcaf);
    g_assert_cmphex(qtest_readl(qts, GD_SENSITIVITY), ==, 0xde00a659);
    g_assert_cmphex(qtest_readl(qts, GD_LOCK), ==, 0xff);
    /* ...but not the trigger registers. */
    qtest_writel(qts, GD_TRIG_FORCE, 0x8);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x8);
    qtest_writel(qts, GD_TRIG_STATUS, 0x8);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0);

    /* A system reset unlocks and restores the defaults. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, GD_LOCK), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_DISARM), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_SENSITIVITY), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_glitch_reset(void)
{
    QTestState *qts = start();

    /* Any value but 0x5bad arms the detectors; DISARM is then ignored. */
    qtest_writel(qts, GD_ARM, 0x1234);
    qtest_writel(qts, GD_DISARM, 0xdcaf);
    qtest_writel(qts, GD_TRIG_FORCE, 0x2);
    qtest_qmp_eventwait(qts, "RESET");

    /* The detector block keeps its state across the reset it caused. */
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, GD_ARM), ==, 0x1234);

    /* Any other system reset clears it. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GD_ARM), ==, 0x5bad);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_glitch_otp_armed(void)
{
    /* 4096 OTP rows, with CRIT1.GLITCH_DETECTOR_ENABLE in all 8 copies. */
    g_autofree uint32_t *otp = g_new0(uint32_t, 4096);
    g_autofree char *otp_path = NULL;
    GError *err = NULL;
    QTestState *qts;
    int fd, i;

    for (i = 0; i < 8; i++) {
        otp[0x40 + i] = cpu_to_le32(0x10);
    }
    fd = g_file_open_tmp("rp2350-sysregs-otp-XXXXXX.img", &otp_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, otp, 4096 * 4), ==, 4096 * 4);
    close(fd);
    qts = qtest_initf("-M rp2350 -bios %s "
                      "-drive if=none,id=otp,format=raw,file=%s "
                      "-global rp2350-otp.drive=otp", rom_path, otp_path);
    rp2350_unreset(qts, RP2350_RESETS_ALL);

    /* Armed by OTP, DISARM's pattern disarms the detectors. */
    qtest_writel(qts, GD_DISARM, 0xdcaf);
    qtest_writel(qts, GD_TRIG_FORCE, 0x1);
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x1);

    /* Any other DISARM value leaves them armed. */
    qtest_writel(qts, GD_DISARM, 0xdcae);
    qtest_writel(qts, GD_TRIG_FORCE, 0x4);
    qtest_qmp_eventwait(qts, "RESET");
    g_assert_cmphex(qtest_readl(qts, GD_TRIG_STATUS), ==, 0x5);
    qtest_quit(qts);
    unlink(otp_path);
}

/* [spec:nuos:req:emu.system-regs/test] */
static void test_dft(void)
{
    QTestState *qts = start();

    /* Undocumented: reads as zero and ignores writes. */
    qtest_writel(qts, DFT, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DFT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DFT + 0xffc), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-sysregs-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/sysregs/sysinfo", test_sysinfo);
    qtest_add_func("/rp2350/sysregs/tbman", test_tbman);
    qtest_add_func("/rp2350/sysregs/syscfg", test_syscfg);
    qtest_add_func("/rp2350/sysregs/glitch-registers", test_glitch_registers);
    qtest_add_func("/rp2350/sysregs/glitch-reset", test_glitch_reset);
    qtest_add_func("/rp2350/sysregs/glitch-otp-armed", test_glitch_otp_armed);
    qtest_add_func("/rp2350/sysregs/dft", test_dft);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
