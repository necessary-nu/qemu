/*
 * QTest testcase for the Raspberry Pi RP2350 machine
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define ROM_BASE 0x00000000
#define XIP_BASE 0x10000000
#define XIP_NOCACHE_NOALLOC_BASE 0x14000000
#define XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE 0x1c000000
#define SRAM_BASE 0x20000000
#define SRAM_SIZE (520 * 1024)

/* Initial SP at the top of SRAM, reset vector at image offset 8 (Thumb). */
#define TEST_SP (SRAM_BASE + SRAM_SIZE)
#define TEST_RESET_OFFSET 8

static char *write_image(uint32_t base)
{
    uint32_t words[3] = {
        cpu_to_le32(TEST_SP),
        cpu_to_le32(base + TEST_RESET_OFFSET + 1),
        cpu_to_le32(0xe7fee7fe), /* b . */
    };
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("rp2350-test-XXXXXX.bin", &path, &err);

    g_assert_no_error(err);
    g_assert_cmpint(write(fd, words, sizeof(words)), ==, sizeof(words));
    close(fd);
    return path;
}

static QTestState *boot_direct(char **path)
{
    *path = write_image(XIP_BASE);
    return qtest_initf("-M rp2350,flash-size=4M -kernel %s", *path);
}

static void assert_mtree_has(QTestState *qts, const char *needle)
{
    g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");

    if (!strstr(mtree, needle)) {
        g_test_message("'%s' not found in memory tree", needle);
        g_test_fail();
    }
}

static void assert_pc(QTestState *qts, uint32_t pc)
{
    g_autofree char *regs = qtest_hmp(qts, "info registers");
    g_autofree char *expected = g_strdup_printf("R15=%08x", pc);

    if (!strstr(regs, expected)) {
        g_test_message("expected %s in register dump", expected);
        g_test_message("%s", regs);
        g_test_fail();
    }
}

/* [spec:nuos:req:emu.machine+1/test] */
static void test_memory_map(void)
{
    static const struct {
        const char *name;
        uint64_t base;
        uint64_t size;
    } unimplemented[] = {
        { "rp2350.sysinfo",    0x40000000, 0x8000 },
        { "rp2350.psm",        0x40018000, 0x8000 },
        { "rp2350.io_bank0",   0x40028000, 0x8000 },
        { "rp2350.uart0",      0x40070000, 0x8000 },
        { "rp2350.uart1",      0x40078000, 0x8000 },
        { "rp2350.timer0",     0x400b0000, 0x8000 },
        { "rp2350.dma",        0x50000000, 0x100000 },
        { "rp2350.usbctrl",    0x50100000, 0x100000 },
        { "rp2350.pio0",       0x50200000, 0x100000 },
        { "rp2350.watchdog",   0x400d8000, 0x8000 },
    };
    g_autofree char *path = NULL;
    QTestState *qts = boot_direct(&path);
    int i;

    assert_mtree_has(qts, "0000000000000000-0000000000007fff "
                          "(prio 0, rom): rp2350.rom");
    assert_mtree_has(qts, "0000000010000000-00000000103fffff "
                          "(prio 0, rom): rp2350.flash");
    assert_mtree_has(qts, "0000000020000000-0000000020081fff "
                          "(prio 0, ram): rp2350.sram");
    assert_mtree_has(qts, "0000000040020000-0000000040023fff "
                          "(prio 0, i/o): rp2350-resets");
    assert_mtree_has(qts, "0000000040024000-0000000040027fff "
                          "(prio -1000, i/o): rp2350.resets @0000000000004000");
    assert_mtree_has(qts, "00000000d0000000-00000000d0000fff "
                          "(prio 0, i/o): rp2350-sio");
    assert_mtree_has(qts, "00000000d0001000-00000000d001ffff "
                          "(prio -1000, i/o): rp2350.sio @0000000000001000");
    assert_mtree_has(qts, "00000000d0020000-00000000d0020fff "
                          "(prio 0, i/o): rp2350-sio-nonsec");
    assert_mtree_has(qts, "00000000e0080000-00000000e0080fff "
                          "(prio 0, i/o): rp2350.eppb");
    for (i = 0; i < ARRAY_SIZE(unimplemented); i++) {
        g_autofree char *entry = g_strdup_printf(
            "%016" PRIx64 "-%016" PRIx64 " (prio -1000, i/o): %s",
            unimplemented[i].base,
            unimplemented[i].base + unimplemented[i].size - 1,
            unimplemented[i].name);
        assert_mtree_has(qts, entry);
    }

    qtest_quit(qts);
    unlink(path);
}

/* [spec:nuos:req:emu.ram-size/test] */
static void test_sram_size(void)
{
    g_autofree char *path = NULL;
    QTestState *qts = boot_direct(&path);

    qtest_writel(qts, SRAM_BASE, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, SRAM_BASE), ==, 0xdeadbeef);

    qtest_writel(qts, SRAM_BASE + SRAM_SIZE - 4, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, SRAM_BASE + SRAM_SIZE - 4), ==,
                    0xcafef00d);

    qtest_writel(qts, SRAM_BASE + SRAM_SIZE, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SRAM_BASE + SRAM_SIZE), ==, 0);

    qtest_quit(qts);
    unlink(path);
}

/* [spec:nuos:req:emu.direct-load/test] */
static void test_direct_load(void)
{
    g_autofree char *path = NULL;
    QTestState *qts = boot_direct(&path);

    g_assert_cmphex(qtest_readl(qts, XIP_BASE), ==, TEST_SP);
    g_assert_cmphex(qtest_readl(qts, XIP_NOCACHE_NOALLOC_BASE), ==, TEST_SP);
    g_assert_cmphex(qtest_readl(qts, XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE),
                    ==, TEST_SP);
    g_assert_cmphex(qtest_readl(qts, ROM_BASE), ==, 0);
    assert_pc(qts, XIP_BASE + TEST_RESET_OFFSET);

    qtest_quit(qts);
    unlink(path);
}

/* [spec:nuos:req:emu.direct-load/test] */
static void test_boot_rom(void)
{
    g_autofree char *path = write_image(ROM_BASE);
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", path);

    g_assert_cmphex(qtest_readl(qts, ROM_BASE), ==, TEST_SP);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE), ==, 0);
    assert_pc(qts, ROM_BASE + TEST_RESET_OFFSET);

    qtest_quit(qts);
    unlink(path);
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

/* [spec:nuos:req:emu.flash/test] */
static void test_flash_size(void)
{
    g_autofree char *path = write_image(XIP_BASE);
    g_autofree char *args = NULL;
    g_autofree char *err = NULL;
    QTestState *qts;

    args = g_strdup_printf("-M rp2350 -kernel %s", path);
    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    g_assert(strstr(err, "flash-size"));
    g_clear_pointer(&err, g_free);
    g_clear_pointer(&args, g_free);

    args = g_strdup_printf("-M rp2350,flash-size=32M -kernel %s", path);
    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    g_assert(strstr(err, "at most 16 MiB"));

    /* No flash: the XIP windows hold only unimplemented-device stubs. */
    qts = qtest_initf("-M rp2350 -bios %s", path);
    {
        g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");

        g_assert(!strstr(mtree, "rp2350.flash"));
        g_assert(strstr(mtree, "0000000010000000-0000000013ffbfff "
                               "(prio -1000, i/o): rp2350.xip"));
    }
    qtest_quit(qts);

    qts = qtest_initf("-M rp2350,flash-size=2M -kernel %s", path);
    assert_mtree_has(qts, "0000000010000000-00000000101fffff "
                          "(prio 0, rom): rp2350.flash");
    assert_mtree_has(qts, "0000000010200000-0000000013ffbfff "
                          "(prio -1000, i/o): rp2350.xip");
    qtest_quit(qts);
    unlink(path);
}

/* [spec:nuos:req:emu.irq-routing+1/test] */
static void test_irq_routing(void)
{
    /* System-level lines only; core-local lines are tested per model. */
    static const int lines[] = { 1, 14, 33, 51 };
    g_autofree char *path = write_image(XIP_BASE);
    int core, i;

    /*
     * qtest can intercept one device's inputs per run, so one run per
     * core. The NVIC's IRQ inputs are passed through to its armv7m
     * container, which is where they are intercepted.
     */
    for (core = 0; core < 2; core++) {
        g_autofree char *cpu = g_strdup_printf("/machine/soc/armv7m[%d]",
                                                core);
        QTestState *qts = qtest_initf("-M rp2350,flash-size=4M -kernel %s",
                                      path);

        qtest_irq_intercept_in(qts, cpu);
        for (i = 0; i < ARRAY_SIZE(lines); i++) {
            g_assert_false(qtest_get_irq(qts, lines[i]));
            qtest_set_irq_in(qts, "/machine/soc", NULL, lines[i], 1);
            g_assert_true(qtest_get_irq(qts, lines[i]));
            qtest_set_irq_in(qts, "/machine/soc", NULL, lines[i], 0);
            g_assert_false(qtest_get_irq(qts, lines[i]));
        }
        qtest_quit(qts);
    }
    unlink(path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/rp2350/memory-map", test_memory_map);
    qtest_add_func("/rp2350/sram-size", test_sram_size);
    qtest_add_func("/rp2350/direct-load", test_direct_load);
    qtest_add_func("/rp2350/boot-rom", test_boot_rom);
    qtest_add_func("/rp2350/flash-size", test_flash_size);
    qtest_add_func("/rp2350/irq-routing", test_irq_routing);

    return g_test_run();
}
