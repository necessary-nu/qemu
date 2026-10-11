/*
 * QTest testcase for the ESP32-P4 LP core's start, the absent LP ROM, the
 * LP interrupt controller and the PMU's LP CPU sleep process
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qobject/qdict.h"
#include "libqtest.h"

#define LP_CPU                  "/machine/lp-cluster/cpu"
/* The CPUs' indices: HP core 0, HP core 1, the LP core. */
#define HP_CPU0_INDEX           0
#define LP_CPU_INDEX            2

#define LP_ROM                  0x50100000u
#define LP_SRAM                 0x50108000u

#define LP_SYS                  0x50110000u
#define LP_SYS_CTRL             (LP_SYS + 0x008)
#define LP_CORE_BOOT_ADDR       (LP_SYS + 0x018)
#define BOOT_ADDR_HP_LP         (LP_SYS + 0x028)
#define LP_CPU_DBG_PC           (LP_SYS + 0x190)

#define PMU                     0x50115000u
#define PMU_LP_INT_RAW          (PMU + 0x174)
#define PMU_LP_INT_ENA          (PMU + 0x17c)
#define PMU_LP_INT_CLR          (PMU + 0x180)
#define PMU_LP_CPU_PWR0         (PMU + 0x184)
#define PMU_LP_CPU_PWR1         (PMU + 0x188)
#define PMU_LP_CPU_PWR2         (PMU + 0x18c)
#define PMU_HP_LP_CPU_COMM      (PMU + 0x19c)
#define LP_CPU_STALL_RDY        (1u << 1)
#define LP_CPU_SLP_STALL_EN     (1u << 29)
#define LP_CPU_SLP_RESET_EN     (1u << 30)
#define LP_CPU_SLP_BYPASS_INTR  (1u << 31)
#define LP_CPU_STALL_WAIT(n)    ((n) << 21)
#define LP_CPU_STALL_WAIT_MASK  (0xffu << 21)
#define SLEEP_REQ               (1u << 31)
#define HP_TRIGGER_LP           (1u << 31)
#define LP_INT_HP_SW_TRIGGER    (1u << 31)
#define WAKE_LP_CPU             22

#define LP_INTR                 0x5012c000u
#define LP_INTR_SW_RAW          (LP_INTR + 0x00)
#define LP_INTR_SW_ST           (LP_INTR + 0x04)
#define LP_INTR_SW_ENA          (LP_INTR + 0x08)
#define LP_INTR_SW_CLR          (LP_INTR + 0x0c)
#define LP_INTR_STATUS          (LP_INTR + 0x10)
#define LP_INTR_DATE            (LP_INTR + 0x3fc)
#define LP_SW_INT               (1u << 31)

#define MB                      0x50118000u
#define MB_LP_INT_CLR           (MB + 0x4c)
#define MB_HP_INT_CLR           (MB + 0x5c)

#define HP_SYS                  0x500e5000u
#define HP_SYS_CPU_INT0         (HP_SYS + 0x10)
#define HP2LP_INTR_GROUP_EN(n)  (HP_SYS + 0x1f0 + 4 * (n))

/* Core 0's view of the interrupt matrix's sources 0 to 31. */
#define INTMTX_STATUS0          0x500d6200u

#define LPPERI_RESET_EN         0x50120008u
#define RST_EN_LP_INTR          (1u << 29)

#define US                      1000ll

static uint32_t dump_reg(const char *regs, int cpu, const char *name)
{
    g_autofree char *head = g_strdup_printf("CPU#%d", cpu);
    g_autofree char *key = g_strdup_printf(" %s ", name);
    const char *p = strstr(regs, head);
    const char *end;
    unsigned long v;

    g_assert(p);
    p = strstr(p, key);
    g_assert(p);
    g_assert_cmpint(qemu_strtoul(p + strlen(key), &end, 16, &v), ==, 0);
    return v;
}

static uint32_t lp_reg(QTestState *qts, const char *name)
{
    g_autofree char *regs = qtest_hmp(qts, "info registers -a");

    return dump_reg(regs, LP_CPU_INDEX, name);
}

/*
 * The LP core leaves reset on its own thread: wait for its PC, as
 * LP_SYSTEM_LP_CPU_DBG_PC shows it, to become @pc.
 */
static void wait_pc(QTestState *qts, uint32_t pc)
{
    for (int i = 0; i < 1000 && qtest_readl(qts, LP_CPU_DBG_PC) != pc; i++) {
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(qts, LP_CPU_DBG_PC), ==, pc);
}

/* Start the LP core through the PMU, as ESP-IDF's ulp_lp_core_run(). */
static void lp_wake(QTestState *qts)
{
    qtest_writel(qts, PMU_LP_CPU_PWR2, 1u << WAKE_LP_CPU);
    qtest_writel(qts, PMU_HP_LP_CPU_COMM, HP_TRIGGER_LP);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
}

/* Its sleep, stalled and reset, and the trigger's wakeup cleared. */
static void lp_sleep(QTestState *qts)
{
    qtest_writel(qts, PMU_LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    qtest_writel(qts, PMU_LP_CPU_PWR0, qtest_readl(qts, PMU_LP_CPU_PWR0) |
                 LP_CPU_SLP_STALL_EN | LP_CPU_SLP_RESET_EN);
    qtest_writel(qts, PMU_LP_CPU_PWR1, SLEEP_REQ);
    qtest_clock_step(qts, 10 * US);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
}

/*
 * The LP core's start: 0x80 into the vector table at the boot address's
 * 256-byte aligned base, mtvec at the table; with the LP ROM absent, a
 * start at its entry goes to the application's boot address.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core/test] */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_boot(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* Held in reset at the LP ROM's entry. */
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert_cmphex(lp_reg(qts, "pc"), ==, LP_ROM + 0x80);
    g_assert_cmphex(lp_reg(qts, "mtvec"), ==, LP_ROM | 1);
    g_assert_cmphex(qtest_readl(qts, LP_CORE_BOOT_ADDR), ==, LP_ROM);

    /* Past the absent LP ROM, to the application's boot address. */
    qtest_writel(qts, BOOT_ADDR_HP_LP, LP_SRAM + 0x80);
    lp_wake(qts);
    wait_pc(qts, LP_SRAM + 0x80);
    g_assert_cmphex(lp_reg(qts, "mtvec"), ==, LP_ROM | 1);
    lp_sleep(qts);

    /* skip_lp_rom_boot: the boot address the application's entry. */
    qtest_writel(qts, LP_CORE_BOOT_ADDR, LP_SRAM + 0x80);
    qtest_writel(qts, BOOT_ADDR_HP_LP, 0x4ff80000);
    lp_wake(qts);
    wait_pc(qts, LP_SRAM + 0x80);
    g_assert_cmphex(lp_reg(qts, "mtvec"), ==, LP_SRAM | 1);
    lp_sleep(qts);

    /* The vector table's own address starts the core at its entry. */
    qtest_writel(qts, LP_CORE_BOOT_ADDR, 0x4ff80000);
    lp_wake(qts);
    wait_pc(qts, 0x4ff80080);
    g_assert_cmphex(lp_reg(qts, "mtvec"), ==, 0x4ff80001);
    lp_sleep(qts);

    /* An application boot address of 0, as software left it, is taken. */
    qtest_writel(qts, LP_CORE_BOOT_ADDR, LP_ROM);
    qtest_writel(qts, BOOT_ADDR_HP_LP, 0);
    lp_wake(qts);
    wait_pc(qts, 0);
    lp_sleep(qts);

    /* A system reset brings the boot address back to the LP ROM. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, LP_CORE_BOOT_ADDR), ==, LP_ROM);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    qtest_quit(qts);
}

/*
 * The word at @addr as CPU @cpu reads it (HMP's xp with it the monitor's
 * CPU), in *@value; false if the CPU cannot reach the address.
 */
static bool cpu_read(QTestState *qts, int cpu, uint32_t addr, uint32_t *value)
{
    g_autofree char *cmd = g_strdup_printf("xp /1wx 0x%x", addr);
    QDict *rsp = qtest_qmp(qts, "{'execute': 'human-monitor-command', "
                           "'arguments': {'command-line': %s, "
                           "'cpu-index': %d}}", cmd, cpu);
    const char *text, *colon, *end;
    unsigned long v;
    bool ok;

    g_assert(qdict_haskey(rsp, "return"));
    text = qdict_get_str(rsp, "return");
    ok = !strstr(text, "Cannot access memory");
    if (ok) {
        colon = strchr(text, ':');
        g_assert(colon);
        g_assert_cmpint(qemu_strtoul(colon + 1, &end, 16, &v), ==, 0);
        *value = v;
    }
    qobject_unref(rsp);
    return ok;
}

/*
 * The absent LP ROM: the LP core reads zero there, each access logged as
 * unavailable; HP core 0, which cannot reach it, gets a bus error, and its
 * write changes nothing.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_lp_rom(void)
{
    g_autofree char *log = NULL;
    g_autofree char *text = NULL;
    GError *err = NULL;
    QTestState *qts;
    uint32_t v = 0x5a5a5a5a;
    int fd = g_file_open_tmp("esp32p4-lp-rom-XXXXXX.log", &log, &err);

    g_assert_no_error(err);
    close(fd);
    qts = qtest_initf("-M esp32p4 -d unimp,guest_errors -D %s", log);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM, &v));
    g_assert_cmphex(v, ==, 0);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM + 0x3ffc, &v));
    g_assert_cmphex(v, ==, 0);
    g_assert(!cpu_read(qts, HP_CPU0_INDEX, LP_ROM, &v));
    g_assert(!cpu_read(qts, HP_CPU0_INDEX, LP_ROM + 0x3ffc, &v));
    qtest_writel(qts, LP_ROM + 0x100, 0x12345678);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM + 0x100, &v));
    g_assert_cmphex(v, ==, 0);
    qtest_quit(qts);

    g_assert(g_file_get_contents(log, &text, NULL, NULL));
    g_assert(strstr(text, "esp32p4.lp-rom: read of 4 bytes at 0x3ffc: the "
                    "LP ROM is unavailable"));
    g_assert(strstr(text, "esp32p4.lp-rom: read of 4 bytes at 0x3ffc: only "
                    "the LP core reaches the LP ROM"));
    g_assert(strstr(text, "esp32p4.lp-rom: write of 4 bytes at 0x100: only "
                    "the LP core reaches the LP ROM"));
    unlink(log);
}

/*
 * LP_INTR: the software interrupt's registers, LPINTR_STATUS showing the
 * sources, and the LP core's mip the IDs they reach; the PMU's sleep
 * process masking them.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core/test] */
static void test_lp_intr(void)
{
    QTestState *qts = qtest_init("-M esp32p4");
    int g;

    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_ENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_DATE), ==, 0);
    qtest_writel(qts, LP_INTR_DATE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_DATE), ==, 1u << 31);

    /* The software interrupt: raw, enabled, LP_SW_INTR (source 18). */
    qtest_writel(qts, LP_INTR_SW_RAW, LP_SW_INT);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_RAW), ==, LP_SW_INT);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 0);
    qtest_writel(qts, LP_INTR_SW_ENA, LP_SW_INT);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_ST), ==, LP_SW_INT);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 1u << 12);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS0) & (1u << 18), ==,
                    1u << 18);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 1u << 3);
    qtest_writel(qts, LP_INTR_SW_CLR, LP_SW_INT);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS0) & (1u << 18), ==, 0);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 0);
    qtest_writel(qts, LP_INTR_SW_RAW, LP_SW_INT);
    qtest_writel(qts, LP_INTR_SW_RAW, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_ST), ==, 0);

    /* A mailbox message: MB_HP_INTR and MB_LP_INTR, both ID 26. */
    qtest_writel(qts, MB, 1);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==,
                    (1u << 26) | (1u << 25));
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 1u << 26);
    qtest_writel(qts, MB_HP_INT_CLR, 0xffff);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 1u << 25);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 1u << 26);
    qtest_writel(qts, MB_LP_INT_CLR, 0xffff);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 0);

    /* The PMU's LP interrupt, PMU_REG_1_INTR (source 7): ID 25. */
    qtest_writel(qts, PMU_LP_INT_CLR, 0xffffe000);
    qtest_writel(qts, PMU_LP_INT_ENA, LP_INT_HP_SW_TRIGGER);
    qtest_writel(qts, PMU_HP_LP_CPU_COMM, HP_TRIGGER_LP);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 1u << 23);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 1u << 25);
    qtest_writel(qts, PMU_LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    qtest_writel(qts, PMU_LP_INT_ENA, 0);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 0);

    /* HP_INTR: an HP source HP2LP_INTR_GROUP enables, ID 30. */
    for (g = 0; g < 4; g++) {
        qtest_writel(qts, HP2LP_INTR_GROUP_EN(g), 0);
    }
    qtest_writel(qts, HP_SYS_CPU_INT0, 1);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 0);
    qtest_writel(qts, HP2LP_INTR_GROUP_EN(1), 1u << 26);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 1u << 31);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 1u << 30);

    /* The PMU's sleep process, with SLP_BYPASS_INTR_EN, masks the IDs. */
    lp_wake(qts);
    qtest_writel(qts, PMU_LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    qtest_writel(qts, PMU_LP_CPU_PWR0, qtest_readl(qts, PMU_LP_CPU_PWR0) |
                 LP_CPU_SLP_BYPASS_INTR);
    qtest_writel(qts, PMU_LP_CPU_PWR1, SLEEP_REQ);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_STATUS), ==, 1u << 31);
    qtest_clock_step(qts, 10 * US);
    lp_wake(qts);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 1u << 30);
    qtest_writel(qts, HP_SYS_CPU_INT0, 0);
    g_assert_cmphex(lp_reg(qts, "mip"), ==, 0);

    /* LPPERI's reset of LP_INTR clears its registers. */
    qtest_writel(qts, LP_INTR_SW_ENA, LP_SW_INT);
    qtest_writel(qts, LPPERI_RESET_EN, RST_EN_LP_INTR);
    qtest_writel(qts, LPPERI_RESET_EN, 0);
    g_assert_cmphex(qtest_readl(qts, LP_INTR_SW_ENA), ==, 0);
    qtest_quit(qts);
}

/*
 * The PMU's sleep process: SLP_STALL_EN stalls the core at once, and its
 * clock stops, and with SLP_RESET_EN its reset is held, SLP_STALL_WAIT
 * cycles of its 40 MHz clock later.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core/test] */
static void test_sleep_process(void)
{
    QTestState *qts = qtest_init("-M esp32p4");
    uint32_t pwr0;

    lp_wake(qts);
    qtest_writel(qts, PMU_LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    pwr0 = qtest_readl(qts, PMU_LP_CPU_PWR0) & ~LP_CPU_STALL_WAIT_MASK;
    qtest_writel(qts, PMU_LP_CPU_PWR0, pwr0 | LP_CPU_STALL_WAIT(40) |
                 LP_CPU_SLP_STALL_EN | LP_CPU_SLP_RESET_EN);
    qtest_writel(qts, PMU_LP_CPU_PWR1, SLEEP_REQ);
    g_assert_cmphex(qtest_readl(qts, PMU_LP_CPU_PWR0) & LP_CPU_STALL_RDY, ==,
                    LP_CPU_STALL_RDY);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    /* 40 cycles of 40 MHz: 1 us. */
    qtest_clock_step(qts, 999);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    qtest_clock_step(qts, 2);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-wfi"));

    /* Without SLP_STALL_EN the core runs on until its clock stops. */
    lp_wake(qts);
    qtest_writel(qts, PMU_LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    qtest_writel(qts, PMU_LP_CPU_PWR0, pwr0 | LP_CPU_STALL_WAIT(40));
    qtest_writel(qts, PMU_LP_CPU_PWR1, SLEEP_REQ);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    g_assert_cmphex(qtest_readl(qts, PMU_LP_CPU_PWR0) & LP_CPU_STALL_RDY, ==,
                    0);
    qtest_clock_step(qts, 1001);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));

    /* LP_SYSTEM's LP_CORE_DISABLE leaves the clock enable as it is. */
    qtest_writel(qts, LP_SYS_CTRL, qtest_readl(qts, LP_SYS_CTRL) | 1);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_CTRL) & 1, ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/lp-core/boot", test_boot);
    qtest_add_func("esp32p4/lp-core/lp-rom", test_lp_rom);
    qtest_add_func("esp32p4/lp-core/lp-intr", test_lp_intr);
    qtest_add_func("esp32p4/lp-core/sleep-process", test_sleep_process);

    return g_test_run();
}
