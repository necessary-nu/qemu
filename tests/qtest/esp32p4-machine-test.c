/*
 * QTest testcase for the Espressif ESP32-P4 machine
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qobject/qdict.h"
#include "libqtest.h"

#define TCM_BASE            0x30100000
#define HP_ROM_BASE         0x4fc00000
#define HP_ROM_DIRECT_BASE  0x8fc00000
#define L2MEM_BASE          0x4ff00000
#define L2MEM_DIRECT_BASE   0x8ff00000
#define L2MEM_SIZE          (768 * 1024)
/* The L2 cache's part of L2MEM from reset: its start, 128 KiB. */
#define L2_CACHE_SIZE       (128 * 1024)
#define L2MEM_RAM           (L2MEM_BASE + L2_CACHE_SIZE)
#define L2MEM_RAM_DIRECT    (L2MEM_DIRECT_BASE + L2_CACHE_SIZE)
#define LP_SRAM_BASE        0x50108000

#define ROM_CHIP_ID         0x4fc00010
#define ROM_LAYOUT_PTR      0x4fc1fffc
#define ROM_LAYOUT          0x4fc1db38
#define ESP32P4_CHIP_ID     18

#define HP_SYS_CLKRST_SOC_CLK_CTRL0 0x500e6014
#define HP_SYS_CLKRST_HP_RST_EN0    0x500e60c0
#define CORE1_CPU_CLK_EN            (1u << 4)
#define RST_EN_CORE1_GLOBAL         (1u << 8)
#define LP_SYS_BOOT_ADDR_HP_CORE1   0x50110164

#define HP_CPU0 "/machine/hp-cluster/cpu[0]"
#define HP_CPU1 "/machine/hp-cluster/cpu[1]"
#define LP_CPU  "/machine/lp-cluster/cpu"

#define FLASH_SIZE (16 * 1024 * 1024)

#define EM_RISCV 243
#define PT_LOAD 1

static void assert_mtree_has(QTestState *qts, const char *needle)
{
    g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");

    if (!strstr(mtree, needle)) {
        g_test_message("'%s' not found in memory tree", needle);
        g_test_fail();
    }
}

static void assert_mtree_lacks(QTestState *qts, const char *needle)
{
    g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");

    if (strstr(mtree, needle)) {
        g_test_message("'%s' found in memory tree", needle);
        g_test_fail();
    }
}

/*
 * Write a 32-bit RISC-V ELF with one segment, at `addr`, holding `jal x0,
 * 0`, and its entry point there.
 */
static char *write_elf(uint32_t addr)
{
    uint8_t elf[52 + 32 + 4] = { 0 };
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("esp32p4-test-XXXXXX.elf", &path, &err);

    memcpy(elf, "\x7f" "ELF", 4);
    elf[4] = 1;                         /* ELFCLASS32 */
    elf[5] = 1;                         /* ELFDATA2LSB */
    elf[6] = 1;                         /* EV_CURRENT */
    stw_le_p(elf + 16, 2);              /* ET_EXEC */
    stw_le_p(elf + 18, EM_RISCV);
    stl_le_p(elf + 20, 1);              /* e_version */
    stl_le_p(elf + 24, addr);           /* e_entry */
    stl_le_p(elf + 28, 52);             /* e_phoff */
    stw_le_p(elf + 40, 52);             /* e_ehsize */
    stw_le_p(elf + 42, 32);             /* e_phentsize */
    stw_le_p(elf + 44, 1);              /* e_phnum */
    stw_le_p(elf + 46, 40);             /* e_shentsize */
    stl_le_p(elf + 52, PT_LOAD);
    stl_le_p(elf + 56, 84);             /* p_offset */
    stl_le_p(elf + 60, addr);           /* p_vaddr */
    stl_le_p(elf + 64, addr);           /* p_paddr */
    stl_le_p(elf + 68, 4);              /* p_filesz */
    stl_le_p(elf + 72, 4);              /* p_memsz */
    stl_le_p(elf + 76, 5);              /* PF_R | PF_X */
    stl_le_p(elf + 80, 4);              /* p_align */
    stl_le_p(elf + 84, 0x0000006f);     /* jal x0, 0 */

    g_assert_no_error(err);
    g_assert_cmpint(write(fd, elf, sizeof(elf)), ==, sizeof(elf));
    close(fd);
    return path;
}

static char *write_blank(size_t size)
{
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("esp32p4-test-XXXXXX.img", &path, &err);

    g_assert_no_error(err);
    g_assert_cmpint(ftruncate(fd, size), ==, 0);
    close(fd);
    return path;
}

/*
 * Run QEMU outside qtest and return its exit status, for configurations
 * the machine must refuse before it starts.
 */
static int run_qemu(const char *args, char **err)
{
    g_autofree char *cmd = g_strdup_printf("%s -display none -serial none "
                                           "-monitor none %s",
                                           qtest_qemu_binary(NULL), args);
    g_auto(GStrv) argv = NULL;
    GError *gerr = NULL;
    int status;

    g_assert(g_shell_parse_argv(cmd, NULL, &argv, &gerr));
    g_assert(g_spawn_sync(NULL, argv, NULL, G_SPAWN_STDOUT_TO_DEV_NULL, NULL,
                          NULL, NULL, err, &status, &gerr));
    g_assert_no_error(gerr);
    return status;
}

static void assert_refused(const char *args, const char *message)
{
    g_autofree char *err = NULL;

    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    if (!strstr(err, message)) {
        g_test_message("'%s' not in: %s", message, err);
        g_test_fail();
    }
}

static uint64_t qom_get_uint(QTestState *qts, const char *path,
                             const char *property)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                           "{ 'path': %s, 'property': %s } }",
                           path, property);
    uint64_t v;

    g_assert(qdict_haskey(rsp, "return"));
    v = qdict_get_uint(rsp, "return");
    qobject_unref(rsp);
    return v;
}

/* A register of core `cpu` in an "info registers -a" dump. */
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

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

/* [spec:nuos:req:emu.esp32p4.machine/test] */
static void test_memory_map(void)
{
    QTestState *qts = start();

    assert_mtree_has(qts, "0000000030100000-0000000030101fff "
                          "(prio 0, ram): esp32p4.tcm");
    assert_mtree_has(qts, "000000004fc00000-000000004fc1ffff "
                          "(prio 0, rom): esp32p4.hp-rom");
    assert_mtree_has(qts, "000000008fc00000-000000008fc1ffff "
                          "(prio 0, rom): esp32p4.hp-rom");
    assert_mtree_has(qts, "000000004ff20000-000000004ffbffff "
                          "(prio 0, ram): esp32p4.l2mem @0000000000020000");
    assert_mtree_has(qts, "000000008ff20000-000000008ffbffff "
                          "(prio 0, ram): esp32p4.l2mem @0000000000020000");
    assert_mtree_has(qts, "0000000050100000-0000000050103fff "
                          "(prio 0, i/o): esp32p4.lp-rom");
    assert_mtree_has(qts, "0000000050108000-000000005010ffff "
                          "(prio 0, ram): esp32p4.lp-sram");

    /*
     * The two addresses of L2MEM are one memory, after the 128 KiB the L2
     * cache takes from reset. qtest's accesses are no core's, so they do
     * not go through the caches.
     */
    assert_mtree_has(qts, "000000004ff00000-000000004ff1ffff "
                          "(prio 1, i/o): esp32p4.l2mem-l2-cache");
    assert_mtree_has(qts, "000000008ff00000-000000008ff1ffff "
                          "(prio 1, i/o): esp32p4.l2mem-direct-l2-cache");
    qtest_writel(qts, L2MEM_RAM_DIRECT + 0x40, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, L2MEM_RAM + 0x40), ==, 0x12345678);
    qtest_writel(qts, L2MEM_DIRECT_BASE + L2MEM_SIZE - 4, 0x9abcdef0);
    g_assert_cmphex(qtest_readl(qts, L2MEM_BASE + L2MEM_SIZE - 4), ==,
                    0x9abcdef0);
    qtest_writel(qts, TCM_BASE, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, TCM_BASE), ==, 0xcafef00d);
    qtest_writel(qts, LP_SRAM_BASE, 0x0badf00d);
    g_assert_cmphex(qtest_readl(qts, LP_SRAM_BASE), ==, 0x0badf00d);

    /* The v3 ROM image, at both its addresses. */
    g_assert_cmphex(qtest_readl(qts, ROM_CHIP_ID), ==, ESP32P4_CHIP_ID);
    g_assert_cmphex(qtest_readl(qts, HP_ROM_DIRECT_BASE +
                                     (ROM_CHIP_ID - HP_ROM_BASE)), ==,
                    ESP32P4_CHIP_ID);
    g_assert_cmphex(qtest_readl(qts, ROM_LAYOUT_PTR), ==, ROM_LAYOUT);

    /*
     * The external memory windows, which the MMUs map, at their direct
     * addresses and, for the bus's masters, their cached ones.
     */
    assert_mtree_has(qts, "0000000080000000-0000000083ffffff "
                          "(prio 0, i/o): esp32p4.flash-window");
    assert_mtree_has(qts, "0000000088000000-000000008bffffff "
                          "(prio 0, i/o): esp32p4.psram-window");
    assert_mtree_has(qts, "0000000040000000-0000000043ffffff "
                          "(prio 0, i/o): esp32p4.flash-window");
    assert_mtree_has(qts, "0000000048000000-000000004bffffff "
                          "(prio 0, i/o): esp32p4.psram-window");

    /* The in-package PSRAM behind them, and the flash chip. */
    g_assert_cmpuint(qom_get_uint(qts, "/machine/unattached/esp32p4.psram[0]",
                                  "size"), ==, 32 * 1024 * 1024);
    {
        g_autofree char *qtree = qtest_hmp(qts, "info qtree");

        g_assert(strstr(qtree, "dev: w25q128"));
    }

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.machine/test] */
static void test_unimplemented(void)
{
    g_autofree char *log = NULL;
    g_autofree char *args = NULL;
    g_autofree char *text = NULL;
    QTestState *qts;
    int fd;

    fd = g_file_open_tmp("esp32p4-test-XXXXXX.log", &log, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    args = g_strdup_printf("-M esp32p4,flash-size=16M -d unimp -D %s", log);
    qts = qtest_init(args);

    /* Every block the chip decodes, the TRM's and ESP-IDF's alike. */
    assert_mtree_has(qts, "00000000500ca000-00000000500cafff "
                          "(prio 0, i/o): esp32p4.uart0");
    assert_mtree_has(qts, "000000005008c000-000000005008cfff "
                          "(prio 0, i/o): esp32p4.flash-spi0");
    assert_mtree_has(qts, "000000005008d000-000000005008dfff "
                          "(prio 0, i/o): esp32p4.flash-spi1");
    assert_mtree_has(qts, "000000005008e000-000000005008efff "
                          "(prio 0, i/o): esp32p4.psram-mspi0");
    assert_mtree_has(qts, "000000005008f000-000000005008ffff "
                          "(prio 0, i/o): esp32p4.psram-mspi1");
    assert_mtree_has(qts, "000000003ff10000-000000003ff10fff "
                          "(prio 0, i/o): esp32p4.cache");
    assert_mtree_has(qts, "0000000050093000-0000000050093fff "
                          "(prio 0, i/o): esp32p4.ecc");
    assert_mtree_has(qts, "0000000050098000-000000005009bfff "
                          "(prio 0, i/o): esp32p4.emac");
    assert_mtree_has(qts, "0000000020800000-000000002081ffff "
                          "(prio 0, i/o): esp32p4.clic");
    assert_mtree_has(qts, "000000005012f000-000000005012ffff "
                          "(prio 0, i/o): esp32p4.lp-tsens");
    assert_mtree_has(qts, "00000000500e6000-00000000500e6fff "
                          "(prio 0, i/o): esp32p4.hp-sys-clkrst");
    assert_mtree_has(qts, "0000000050111000-0000000050111fff "
                          "(prio 0, i/o): esp32p4.lp-clkrst");
    assert_mtree_has(qts, "0000000050120000-0000000050120fff "
                          "(prio 0, i/o): esp32p4.lp-peri");
    assert_mtree_has(qts, "0000000050124000-0000000050124fff "
                          "(prio 0, i/o): esp32p4.lp-i2c-ana-mst");
    assert_mtree_has(qts, "0000000050110000-0000000050110fff "
                          "(prio 0, i/o): esp32p4-lp-sys");

    /* Reserved slots between blocks are not decoded. */
    assert_mtree_lacks(qts, "00000000500d4000-");
    assert_mtree_lacks(qts, "0000000050097000-");
    assert_mtree_lacks(qts, "0000000050117000-");

    /* An access to a block with no model is logged as unimplemented. */
    g_assert_cmphex(qtest_readl(qts, 0x500ca000), ==, 0);
    qtest_writel(qts, 0x500e4000, 1);
    g_assert_cmphex(qtest_readl(qts, 0x50110000), ==, 0);
    qtest_quit(qts);

    g_assert(g_file_get_contents(log, &text, NULL, NULL));
    g_assert(strstr(text, "esp32p4.uart0: unimplemented device read"));
    g_assert(strstr(text, "esp32p4.audio-addc: unimplemented device write"));
    g_assert(strstr(text, "esp32p4-lp-sys: unimplemented read"));
    g_assert(strstr(text, "LP ROM's contents are not published"));
    unlink(log);
}

/* [spec:nuos:req:emu.esp32p4.machine/test] */
static void test_flash(void)
{
    g_autofree char *small = write_blank(4 * 1024 * 1024);
    g_autofree char *right = write_blank(FLASH_SIZE);
    g_autofree char *args = NULL;
    QTestState *qts;

    /* The flash is off-package: its size is the board's, with no default. */
    assert_refused("-M esp32p4,flash-size=3M", "power of two");
    assert_refused("-M esp32p4,flash-size=512K", "power of two");
    assert_refused("-M esp32p4,flash-size=256M", "power of two");

    /* An image backs the whole flash. */
    args = g_strdup_printf("-M esp32p4 -drive if=mtd,format=raw,file=%s",
                           right);
    assert_refused(args, "needs -M esp32p4,flash-size=SIZE");
    g_free(args);

    args = g_strdup_printf("-M esp32p4,flash-size=16M "
                           "-drive if=mtd,format=raw,file=%s", small);
    assert_refused(args, "must be flash-size");
    g_free(args);

    args = g_strdup_printf("-M esp32p4,flash-size=16M "
                           "-drive if=mtd,format=raw,file=%s", right);
    qts = qtest_init(args);
    {
        g_autofree char *qtree = qtest_hmp(qts, "info qtree");

        g_assert(strstr(qtree, "dev: w25q128"));
        g_assert(strstr(qtree, "drive = \"mtd0\""));
    }
    qtest_quit(qts);

    qts = qtest_init("-M esp32p4,flash-size=4M");
    {
        g_autofree char *qtree = qtest_hmp(qts, "info qtree");

        g_assert(strstr(qtree, "dev: w25q32"));
    }
    qtest_quit(qts);

    unlink(small);
    unlink(right);
}

/* [spec:nuos:req:emu.esp32p4.machine/test] */
static void test_no_flash(void)
{
    g_autofree char *log = NULL;
    g_autofree char *args = NULL;
    g_autofree char *text = NULL;
    QTestState *qts;
    int fd;

    fd = g_file_open_tmp("esp32p4-test-XXXXXX.log", &log, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    /* With no flash-size the board has no flash: the bus has no chip. */
    args = g_strdup_printf("-M esp32p4 -d unimp -D %s", log);
    qts = qtest_init(args);
    {
        g_autofree char *qtree = qtest_hmp(qts, "info qtree");

        g_assert(strstr(qtree, "dev: esp32p4-mspi"));
        g_assert(strstr(qtree, "bus: ssi"));
        g_assert(!strstr(qtree, "dev: w25q"));
    }
    /* The rest of the machine is all there. */
    assert_mtree_has(qts, "0000000080000000-0000000083ffffff "
                          "(prio 0, i/o): esp32p4.flash-window");
    g_assert_cmphex(qtest_readl(qts, ROM_CHIP_ID), ==, ESP32P4_CHIP_ID);
    qtest_quit(qts);

    g_assert(g_file_get_contents(log, &text, NULL, NULL));
    g_assert(strstr(text, "no flash is configured"));
    unlink(log);

    /* So is a flash-size of zero. */
    qts = qtest_init("-M esp32p4,flash-size=0");
    {
        g_autofree char *qtree = qtest_hmp(qts, "info qtree");

        g_assert(!strstr(qtree, "dev: w25q"));
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.machine/test] */
static void test_cores(void)
{
    QTestState *qts = start();
    g_autofree char *regs = qtest_hmp(qts, "info registers -a");

    /* Both HP cores start at the HP ROM and the LP core at the LP ROM. */
    g_assert_cmphex(dump_reg(regs, 0, "pc"), ==, HP_ROM_BASE);
    g_assert_cmphex(dump_reg(regs, 1, "pc"), ==, HP_ROM_BASE);
    g_assert_cmphex(dump_reg(regs, 2, "pc"), ==, 0x50100000);
    g_assert_cmphex(dump_reg(regs, 1, "mhartid"), ==, 1);
    g_assert_cmphex(dump_reg(regs, 2, "mhartid"), ==, 0);
    /* RV32IMAFCBU with Espressif's extensions (X), and RV32IMAC. */
    g_assert_cmphex(dump_reg(regs, 0, "misa"), ==, 0x40901127);
    g_assert_cmphex(dump_reg(regs, 2, "misa"), ==, 0x40001105);

    /* Core 0 runs; core 1 is held in reset with its clock gated. */
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert_cmphex(qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0), ==,
                    0xe6df97af);
    g_assert_cmphex(qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0), ==,
                    0x00000100);

    /* Software releases it as ESP-IDF does: clock first, then reset. */
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL0,
                 0xe6df97af | CORE1_CPU_CLK_EN);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0, 0);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));

    /* Byte writes reach their byte only. */
    qtest_writeb(qts, HP_SYS_CLKRST_HP_RST_EN0 + 1, 0x01);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert_cmphex(qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0), ==,
                    RST_EN_CORE1_GLOBAL);
    g_assert_cmphex(qtest_readb(qts, HP_SYS_CLKRST_HP_RST_EN0 + 1), ==, 1);

    /* Core 0 can hold itself, too. */
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0, 1u << 7);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0, 0);

    /* The entry point the ROM on core 1 takes. */
    qtest_writel(qts, LP_SYS_BOOT_ADDR_HP_CORE1, 0xcff80000);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_BOOT_ADDR_HP_CORE1), ==,
                    0xcff80000);

    /* A system reset holds core 1 again. */
    qtest_system_reset(qts);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_BOOT_ADDR_HP_CORE1), ==, 0);

    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.machine/test] */
static void test_kernel(void)
{
    g_autofree char *elf = write_elf(0x4ff80000);
    g_autofree char *bad = write_elf(0x40000000);
    g_autofree char *args = NULL;
    g_autofree char *regs = NULL;
    QTestState *qts;

    args = g_strdup_printf("-M esp32p4,flash-size=16M -kernel %s", elf);
    qts = qtest_init(args);
    regs = qtest_hmp(qts, "info registers -a");
    /* Core 0 starts at the entry point; core 1 still at the ROM. */
    g_assert_cmphex(dump_reg(regs, 0, "pc"), ==, 0x4ff80000);
    g_assert_cmphex(dump_reg(regs, 1, "pc"), ==, HP_ROM_BASE);
    g_assert_cmphex(qtest_readl(qts, 0x4ff80000), ==, 0x0000006f);
    /* The ROM is still there. */
    g_assert_cmphex(qtest_readl(qts, ROM_CHIP_ID), ==, ESP32P4_CHIP_ID);
    qtest_quit(qts);

    /* Only the MMU, which software sets up, maps the flash window. */
    g_free(args);
    args = g_strdup_printf("-M esp32p4,flash-size=16M -kernel %s", bad);
    assert_refused(args, "outside L2MEM, the TCM and LP SRAM");

    unlink(elf);
    unlink(bad);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/memory-map", test_memory_map);
    qtest_add_func("/esp32p4/unimplemented", test_unimplemented);
    qtest_add_func("/esp32p4/flash", test_flash);
    qtest_add_func("/esp32p4/no-flash", test_no_flash);
    qtest_add_func("/esp32p4/cores", test_cores);
    qtest_add_func("/esp32p4/kernel", test_kernel);

    return g_test_run();
}
