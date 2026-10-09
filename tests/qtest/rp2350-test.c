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
#include "rp2350-resets.h"

#define ROM_BASE 0x00000000
#define XIP_BASE 0x10000000
#define XIP_NOCACHE_NOALLOC_BASE 0x14000000
#define XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE 0x1c000000
#define SRAM_BASE 0x20000000
#define SRAM_SIZE (520 * 1024)
#define USB_DPRAM_BASE 0x50100000
#define USB_DPRAM_SIZE 0x1000
#define FLASH_SIZE (4 * 1024 * 1024)

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
        { "rp2350.pio0",       0x50200000, 0x100000 },
    };
    g_autofree char *path = NULL;
    QTestState *qts = boot_direct(&path);
    int i;

    /* A subsystem in reset has its window answered for it. */
    assert_mtree_has(qts, "0000000040000000-0000000040007fff "
                          "(prio 1, i/o): rp2350.sysinfo-in-reset");
    rp2350_unreset(qts, RP2350_RESETS_ALL);

    assert_mtree_has(qts, "0000000000000000-0000000000007fff "
                          "(prio 0, rom): rp2350.rom");
    assert_mtree_has(qts, "0000000010000000-00000000103fffff "
                          "(prio 0, romd): rp2350.qspi-cs0");
    assert_mtree_has(qts, "0000000020000000-0000000020081fff "
                          "(prio 0, ram): rp2350.sram");
    assert_mtree_has(qts, "0000000040000000-0000000040003fff "
                          "(prio 0, i/o): rp2350-sysinfo");
    assert_mtree_has(qts, "0000000040004000-0000000040007fff "
                          "(prio -1000, i/o): rp2350.sysinfo @0000000000004000");
    assert_mtree_has(qts, "0000000040158000-000000004015bfff "
                          "(prio 0, i/o): rp2350-glitch-detector");
    assert_mtree_has(qts, "0000000040018000-000000004001bfff "
                          "(prio 0, i/o): rp2350-psm");
    assert_mtree_has(qts, "000000004001c000-000000004001ffff "
                          "(prio -1000, i/o): rp2350.psm @0000000000004000");
    assert_mtree_has(qts, "0000000040020000-0000000040023fff "
                          "(prio 0, i/o): rp2350-resets");
    assert_mtree_has(qts, "0000000040024000-0000000040027fff "
                          "(prio -1000, i/o): rp2350.resets @0000000000004000");
    assert_mtree_has(qts, "0000000040028000-000000004002bfff "
                          "(prio 0, i/o): rp2350-io-bank0");
    assert_mtree_has(qts, "0000000040030000-0000000040033fff "
                          "(prio 0, i/o): rp2350-io-qspi");
    assert_mtree_has(qts, "0000000040038000-000000004003bfff "
                          "(prio 0, i/o): rp2350-pads-bank0");
    assert_mtree_has(qts, "0000000040040000-0000000040043fff "
                          "(prio 0, i/o): rp2350-pads-qspi");
    assert_mtree_has(qts, "0000000040070000-0000000040073fff "
                          "(prio 0, i/o): rp2350-uart0");
    assert_mtree_has(qts, "0000000040078000-000000004007bfff "
                          "(prio 0, i/o): rp2350-uart1");
    assert_mtree_has(qts, "0000000040080000-0000000040083fff "
                          "(prio 0, i/o): rp2350-spi0");
    assert_mtree_has(qts, "0000000040084000-0000000040087fff "
                          "(prio -1000, i/o): rp2350.spi0 @0000000000004000");
    assert_mtree_has(qts, "0000000040088000-000000004008bfff "
                          "(prio 0, i/o): rp2350-spi1");
    assert_mtree_has(qts, "00000000400a8000-00000000400abfff "
                          "(prio 0, i/o): rp2350-pwm");
    assert_mtree_has(qts, "00000000400b0000-00000000400b3fff "
                          "(prio 0, i/o): rp2350-timer");
    assert_mtree_has(qts, "00000000400d8000-00000000400dbfff "
                          "(prio 0, i/o): rp2350-watchdog");
    assert_mtree_has(qts, "00000000400dc000-00000000400dffff "
                          "(prio -1000, i/o): rp2350.watchdog "
                          "@0000000000004000");
    assert_mtree_has(qts, "00000000400b8000-00000000400bbfff "
                          "(prio 0, i/o): rp2350-timer");
    assert_mtree_has(qts, "00000000d0000000-00000000d0000fff "
                          "(prio 0, i/o): rp2350-sio");
    assert_mtree_has(qts, "00000000d0001000-00000000d001ffff "
                          "(prio -1000, i/o): rp2350.sio @0000000000001000");
    assert_mtree_has(qts, "00000000d0020000-00000000d0020fff "
                          "(prio 0, i/o): rp2350-sio-nonsec");
    assert_mtree_has(qts, "00000000e0080000-00000000e0083fff "
                          "(prio 0, i/o): rp2350-eppb");
    assert_mtree_has(qts, "0000000050100000-0000000050100fff "
                          "(prio 0, ram): rp2350.usb-dpram");
    assert_mtree_has(qts, "0000000050101000-00000000501fffff "
                          "(prio -1000, i/o): rp2350.usbctrl "
                          "@0000000000001000");
    assert_mtree_has(qts, "0000000050000000-0000000050003fff "
                          "(prio 0, i/o): rp2350-dma");
    assert_mtree_has(qts, "0000000050004000-00000000500fffff "
                          "(prio -1000, i/o): rp2350.dma @0000000000004000");
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

/* The USB DPRAM is memory to the system bus, at every access size. */
/* [spec:nuos:req:emu.machine+1/test] */
static void test_usb_dpram(void)
{
    g_autofree char *path = NULL;
    QTestState *qts = boot_direct(&path);

    /* USBCTRL, DPRAM included, starts held in reset. */
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    qtest_writel(qts, USB_DPRAM_BASE, 0x11223344);
    qtest_writeb(qts, USB_DPRAM_BASE + 1, 0xaa);
    qtest_writew(qts, USB_DPRAM_BASE + 2, 0xbbcc);
    g_assert_cmphex(qtest_readl(qts, USB_DPRAM_BASE), ==, 0xbbccaa44);
    g_assert_cmphex(qtest_readb(qts, USB_DPRAM_BASE + 3), ==, 0xbb);

    qtest_writel(qts, USB_DPRAM_BASE + USB_DPRAM_SIZE - 4, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, USB_DPRAM_BASE + USB_DPRAM_SIZE - 4),
                    ==, 0xcafef00d);

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
    g_clear_pointer(&err, g_free);
    g_clear_pointer(&args, g_free);

    /* Serial flash parts come in power-of-two sizes. */
    args = g_strdup_printf("-M rp2350,flash-size=3M -kernel %s", path);
    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    g_assert(strstr(err, "power of two"));

    /* No flash: chip select 0 has no device; the XIP space is all I/O. */
    qts = qtest_initf("-M rp2350 -bios %s", path);
    {
        g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");

        g_assert(!strstr(mtree, "rp2350.qspi-cs0"));
        g_assert(strstr(mtree, "0000000010000000-000000001fffffff "
                               "(prio 0, i/o): rp2350-xip-space"));
    }
    qtest_quit(qts);

    /* The device ignores address bits above its size: flash repeats. */
    qts = qtest_initf("-M rp2350,flash-size=2M -kernel %s", path);
    assert_mtree_has(qts, "0000000010000000-00000000101fffff "
                          "(prio 0, romd): rp2350.qspi-cs0");
    assert_mtree_has(qts, "0000000010200000-00000000103fffff "
                          "(prio 0, romd): rp2350.qspi-cs0");
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0x200004), ==,
                    XIP_BASE + TEST_RESET_OFFSET + 1);
    qtest_quit(qts);
    unlink(path);
}

/* Write a flash image of `size` bytes: erased, with markers. */
static char *write_flash_image(size_t size)
{
    g_autofree uint8_t *data = g_malloc(size);
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("rp2350-flash-XXXXXX.img", &path, &err);

    g_assert_no_error(err);
    memset(data, 0xff, size);
    stl_le_p(data, 0x464c5348);
    stl_le_p(data + size - 4, 0x454e4421);
    g_assert_cmpint(write(fd, data, size), ==, size);
    close(fd);
    return path;
}

/*
 * The first -drive if=mtd backs the flash on chip select 0. It must be
 * exactly the flash size, and needs a flash.
 */
/* [spec:nuos:req:emu.flash/test] */
static void test_flash_drive(void)
{
    g_autofree char *rom = write_image(ROM_BASE);
    g_autofree char *flash = write_flash_image(FLASH_SIZE);
    g_autofree char *small = write_flash_image(FLASH_SIZE / 2);
    g_autofree char *args = NULL;
    g_autofree char *err = NULL;
    QTestState *qts;

    qts = qtest_initf("-M rp2350,flash-size=4M -bios %s "
                      "-drive if=mtd,format=raw,file=%s", rom, flash);
    g_assert_cmphex(qtest_readl(qts, XIP_NOCACHE_NOALLOC_BASE), ==,
                    0x464c5348);
    g_assert_cmphex(qtest_readl(qts, XIP_NOCACHE_NOALLOC_BASE +
                                     FLASH_SIZE - 4), ==, 0x454e4421);
    g_assert_cmphex(qtest_readl(qts, XIP_NOCACHE_NOALLOC_BASE + 4), ==,
                    0xffffffff);
    qtest_quit(qts);

    /* Without an image the flash is erased. */
    qts = qtest_initf("-M rp2350,flash-size=4M -bios %s", rom);
    g_assert_cmphex(qtest_readl(qts, XIP_NOCACHE_NOALLOC_BASE), ==,
                    0xffffffff);
    qtest_quit(qts);

    args = g_strdup_printf("-M rp2350 -bios %s "
                           "-drive if=mtd,format=raw,file=%s", rom, flash);
    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    g_assert(strstr(err, "flash-size"));
    g_clear_pointer(&err, g_free);
    g_clear_pointer(&args, g_free);

    args = g_strdup_printf("-M rp2350,flash-size=4M -bios %s "
                           "-drive if=mtd,format=raw,file=%s", rom, small);
    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    g_assert(strstr(err, "requires 4194304 bytes"));

    unlink(rom);
    unlink(flash);
    unlink(small);
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

/* [spec:nuos:req:emu.bootrom+2/test] */
static void test_rom_with_direct_load(void)
{
    g_autofree char *kernel = write_image(XIP_BASE);
    g_autofree char *rom = write_image(ROM_BASE);
    QTestState *qts = qtest_initf("-M rp2350,flash-size=4M -kernel %s "
                                  "-bios %s", kernel, rom);

    /* Both images are in place, and core 0 boots from flash. */
    g_assert_cmphex(qtest_readl(qts, ROM_BASE + 4), ==,
                    ROM_BASE + TEST_RESET_OFFSET + 1);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 4), ==,
                    XIP_BASE + TEST_RESET_OFFSET + 1);
    assert_pc(qts, XIP_BASE + TEST_RESET_OFFSET);
    qtest_quit(qts);
    unlink(kernel);
    unlink(rom);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/rp2350/memory-map", test_memory_map);
    qtest_add_func("/rp2350/sram-size", test_sram_size);
    qtest_add_func("/rp2350/usb-dpram", test_usb_dpram);
    qtest_add_func("/rp2350/direct-load", test_direct_load);
    qtest_add_func("/rp2350/boot-rom", test_boot_rom);
    qtest_add_func("/rp2350/flash-size", test_flash_size);
    qtest_add_func("/rp2350/flash-drive", test_flash_drive);
    qtest_add_func("/rp2350/irq-routing", test_irq_routing);
    qtest_add_func("/rp2350/rom-with-direct-load", test_rom_with_direct_load);

    return g_test_run();
}
