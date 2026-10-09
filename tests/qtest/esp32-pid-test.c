/*
 * QTest testcase for the ESP32 PID controller and DPORT's per-PID memory
 * protection registers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define PIDCTRL                 0x3ff1f000
#define PIDCTRL_INT_ENABLE      (PIDCTRL + 0x00)
#define PIDCTRL_INT_ADDR(n)     (PIDCTRL + 4 * (n))
#define PIDCTRL_PID_DELAY       (PIDCTRL + 0x20)
#define PIDCTRL_NMI_DELAY       (PIDCTRL + 0x24)
#define PIDCTRL_LEVEL           (PIDCTRL + 0x28)
#define PIDCTRL_FROM(n)         (PIDCTRL + 0x28 + 4 * (n))
#define PIDCTRL_PID_NEW         (PIDCTRL + 0x48)
#define PIDCTRL_PID_CONFIRM     (PIDCTRL + 0x4c)
#define PIDCTRL_NMI_MASK_ENABLE (PIDCTRL + 0x54)
#define PIDCTRL_NMI_MASK_DISABLE (PIDCTRL + 0x58)

#define DPORT                   0x3ff00000
#define DPORT_IMMU_PAGE_MODE    (DPORT + 0x080)
#define DPORT_DMMU_PAGE_MODE    (DPORT + 0x084)
#define DPORT_AHB_MPU_TABLE_0   (DPORT + 0x0b4)
#define DPORT_AHB_MPU_TABLE_1   (DPORT + 0x0b8)
#define DPORT_AHBLITE_MPU_TABLE_UART (DPORT + 0x32c)
#define DPORT_AHBLITE_MPU_TABLE_PWR  (DPORT + 0x3e4)
#define DPORT_MEM_ACCESS_DBUG0  (DPORT + 0x3e8)
#define DPORT_MEM_ACCESS_DBUG1  (DPORT + 0x3ec)
#define DPORT_IMMU_TABLE(n)     (DPORT + 0x504 + 4 * (n))
#define DPORT_DMMU_TABLE(n)     (DPORT + 0x544 + 4 * (n))
#define DPORT_MMU_IA_INT_EN     (DPORT + 0x598)
#define DPORT_MPU_IA_INT_EN     (DPORT + 0x59c)
#define DPORT_PRO_MMU(n)        (0x3ff10000 + 4 * (n))
#define DPORT_APP_MMU(n)        (0x3ff12000 + 4 * (n))

static QTestState *start(void)
{
    return qtest_init("-M esp32 -nic none");
}

static uint32_t current_pid(QTestState *qts)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/machine/soc/pid0', 'property': 'pid' } }");
    uint32_t pid;

    g_assert(qdict_haskey(r, "return"));
    pid = qdict_get_int(r, "return");
    qobject_unref(r);
    return pid;
}

/* [spec:nuos:req:emu.esp32.memory-protection/test] */
static void test_pidctrl_registers(void)
{
    static const uint32_t vec[] = {
        0x40000340, 0x40000180, 0x400001c0, 0x40000200,
        0x40000240, 0x40000280, 0x400002c0,
    };
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, PIDCTRL_INT_ENABLE), ==, 0);
    for (int n = 1; n <= 7; n++) {
        g_assert_cmphex(qtest_readl(qts, PIDCTRL_INT_ADDR(n)), ==,
                        vec[n - 1]);
        g_assert_cmphex(qtest_readl(qts, PIDCTRL_FROM(n)), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_DELAY), ==, 20);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_NMI_DELAY), ==, 16);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_LEVEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_NEW), ==, 0);
    g_assert_cmpuint(current_pid(qts), ==, 0);

    /* Field widths */
    qtest_writel(qts, PIDCTRL_INT_ENABLE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_INT_ENABLE), ==, 0xfe);
    qtest_writel(qts, PIDCTRL_INT_ADDR(3), 0x40081234);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_INT_ADDR(3)), ==, 0x40081234);
    qtest_writel(qts, PIDCTRL_PID_DELAY, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_DELAY), ==, 0xfff);
    qtest_writel(qts, PIDCTRL_NMI_DELAY, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_NMI_DELAY), ==, 0xfff);
    qtest_writel(qts, PIDCTRL_LEVEL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_LEVEL), ==, 0xf);
    qtest_writel(qts, PIDCTRL_FROM(5), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_FROM(5)), ==, 0x7f);
    qtest_writel(qts, PIDCTRL_PID_NEW, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_NEW), ==, 0x7);

    /* The new PID waits for PID_DELAY instructions, which do not run. */
    qtest_writel(qts, PIDCTRL_PID_CONFIRM, 1);
    g_assert_cmpuint(current_pid(qts), ==, 0);

    /* Write-only */
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_CONFIRM), ==, 0);
    qtest_writel(qts, PIDCTRL_NMI_MASK_ENABLE, 1);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_NMI_MASK_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_NMI_MASK_DISABLE), ==, 0);

    /* A system reset restores the reset values and PID 0. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_INT_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_INT_ADDR(3)), ==, vec[2]);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_DELAY), ==, 20);
    g_assert_cmphex(qtest_readl(qts, PIDCTRL_PID_NEW), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.memory-protection/test] */
static void test_dport_registers(void)
{
    QTestState *qts = start();

    for (int n = 0; n < 16; n++) {
        g_assert_cmphex(qtest_readl(qts, DPORT_IMMU_TABLE(n)), ==,
                        n < 10 ? 0 : n);
        g_assert_cmphex(qtest_readl(qts, DPORT_DMMU_TABLE(n)), ==, n);
    }
    qtest_writel(qts, DPORT_IMMU_TABLE(4), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_IMMU_TABLE(4)), ==, 0x7f);
    qtest_writel(qts, DPORT_DMMU_TABLE(15), 0x35);
    g_assert_cmphex(qtest_readl(qts, DPORT_DMMU_TABLE(15)), ==, 0x35);

    g_assert_cmphex(qtest_readl(qts, DPORT_IMMU_PAGE_MODE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_DMMU_PAGE_MODE), ==, 0);
    qtest_writel(qts, DPORT_IMMU_PAGE_MODE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_IMMU_PAGE_MODE), ==, 0x7);
    qtest_writel(qts, DPORT_DMMU_PAGE_MODE, 2 << 1);
    g_assert_cmphex(qtest_readl(qts, DPORT_DMMU_PAGE_MODE), ==, 4);

    g_assert_cmphex(qtest_readl(qts, DPORT_AHB_MPU_TABLE_0), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_AHB_MPU_TABLE_1), ==, 0x1ff);
    g_assert_cmphex(qtest_readl(qts, DPORT_AHBLITE_MPU_TABLE_UART), ==, 0);
    qtest_writel(qts, DPORT_AHBLITE_MPU_TABLE_UART, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_AHBLITE_MPU_TABLE_UART), ==, 0x3f);
    qtest_writel(qts, DPORT_AHBLITE_MPU_TABLE_PWR, 0x15);
    g_assert_cmphex(qtest_readl(qts, DPORT_AHBLITE_MPU_TABLE_PWR), ==, 0x15);

    /* The flags are read-only; the enables take 24 bits. */
    qtest_writel(qts, DPORT_MEM_ACCESS_DBUG0, 0xffffffff);
    qtest_writel(qts, DPORT_MEM_ACCESS_DBUG1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_MEM_ACCESS_DBUG0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_MEM_ACCESS_DBUG1), ==, 0);
    qtest_writel(qts, DPORT_MMU_IA_INT_EN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_MMU_IA_INT_EN), ==, 0xffffff);
    qtest_writel(qts, DPORT_MPU_IA_INT_EN, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_MPU_IA_INT_EN), ==, 0xffffff);

    /* Every entry of both CPUs' cache MMU tables keeps 9 bits. */
    qtest_writel(qts, DPORT_PRO_MMU(1072), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_PRO_MMU(1072)), ==, 0x1ff);
    qtest_writel(qts, DPORT_APP_MMU(1920 + 127), 0x0a5);
    g_assert_cmphex(qtest_readl(qts, DPORT_APP_MMU(1920 + 127)), ==, 0x0a5);
    g_assert_cmphex(qtest_readl(qts, DPORT_PRO_MMU(1920 + 127)), ==, 0);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, DPORT_IMMU_TABLE(4)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_DMMU_TABLE(15)), ==, 15);
    g_assert_cmphex(qtest_readl(qts, DPORT_DMMU_PAGE_MODE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_AHBLITE_MPU_TABLE_UART), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_PRO_MMU(1072)), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32/pid/pidctrl-registers", test_pidctrl_registers);
    qtest_add_func("esp32/pid/dport-registers", test_dport_registers);
    return g_test_run();
}
