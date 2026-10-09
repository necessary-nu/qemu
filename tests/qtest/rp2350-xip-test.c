/*
 * QTest testcase for the RP2350 XIP subsystem (XIP_CTRL, QMI, XIP_AUX)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses carry no security attribute and are treated as Secure.
 * Bus errors are not visible to qtest; the guest tests cover them.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define XIP_BASE            0x10000000
#define XIP_UNCACHED        0x14000000
#define XIP_MAINT           0x18000000
#define XIP_UNTRANSLATED    0x1c000000
#define XIP_PSRAM           0x11000000
#define XIP_SRAM            0x13ffc000

#define XIP_CTRL_BASE       0x400c8000
#define CTRL                (XIP_CTRL_BASE + 0x00)
#define STAT                (XIP_CTRL_BASE + 0x08)
#define CTR_HIT             (XIP_CTRL_BASE + 0x0c)
#define CTR_ACC             (XIP_CTRL_BASE + 0x10)
#define STREAM_ADDR         (XIP_CTRL_BASE + 0x14)
#define STREAM_CTR          (XIP_CTRL_BASE + 0x18)
#define STREAM_FIFO         (XIP_CTRL_BASE + 0x1c)

#define CTRL_EN_SECURE      (1u << 0)
#define CTRL_EN_NONSECURE   (1u << 1)
#define CTRL_POWER_DOWN     (1u << 3)
#define CTRL_WRITABLE_M0    (1u << 10)
#define CTRL_WRITABLE_M1    (1u << 11)
#define STAT_FIFO_EMPTY     (1u << 1)
#define STAT_FIFO_FULL      (1u << 2)

#define QMI_BASE            0x400d0000
#define DIRECT_CSR          (QMI_BASE + 0x00)
#define DIRECT_TX           (QMI_BASE + 0x04)
#define DIRECT_RX           (QMI_BASE + 0x08)
#define M_TIMING(m)         (QMI_BASE + 0x0c + 0x14 * (m))
#define M_RFMT(m)           (QMI_BASE + 0x10 + 0x14 * (m))
#define M_RCMD(m)           (QMI_BASE + 0x14 + 0x14 * (m))
#define M_WFMT(m)           (QMI_BASE + 0x18 + 0x14 * (m))
#define M_WCMD(m)           (QMI_BASE + 0x1c + 0x14 * (m))
#define ATRANS(n)           (QMI_BASE + 0x34 + 4 * (n))

#define CSR_EN              (1u << 0)
#define CSR_BUSY            (1u << 1)
#define CSR_ASSERT_CS0N     (1u << 2)
#define CSR_ASSERT_CS1N     (1u << 3)
#define CSR_AUTO_CS0N       (1u << 6)
#define CSR_TXFULL          (1u << 10)
#define CSR_TXEMPTY         (1u << 11)
#define CSR_RXEMPTY         (1u << 16)
#define CSR_RXFULL          (1u << 17)
#define CSR_CLKDIV(d)       ((d) << 22)
#define CSR_TXLEVEL(v)      (((v) >> 12) & 7)
#define CSR_RXLEVEL(v)      (((v) >> 18) & 7)

#define XIP_AUX_BASE        0x50500000
#define AUX_STREAM          (XIP_AUX_BASE + 0x0)
#define AUX_DIRECT_TX       (XIP_AUX_BASE + 0x4)
#define AUX_DIRECT_RX       (XIP_AUX_BASE + 0x8)

#define ALIAS_SET           0x2000
#define ALIAS_CLR           0x3000

#define IMAGE_WORDS         4096

/* Every word of the flash image is its own offset, tagged. */
static uint32_t image_word(uint32_t offset)
{
    return 0xa5000000 | offset;
}

static char *image_path;

static void write_image(void)
{
    g_autofree uint32_t *words = g_new(uint32_t, IMAGE_WORDS);
    GError *err = NULL;
    int fd = g_file_open_tmp("rp2350-xip-test-XXXXXX.bin", &image_path,
                             &err);
    int i;

    g_assert_no_error(err);
    for (i = 0; i < IMAGE_WORDS; i++) {
        words[i] = cpu_to_le32(image_word(i * 4));
    }
    g_assert_cmpint(write(fd, words, IMAGE_WORDS * 4), ==, IMAGE_WORDS * 4);
    close(fd);
}

static QTestState *start(const char *opts)
{
    return qtest_initf("-M rp2350,%s -kernel %s", opts, image_path);
}

/* Run QEMU outside qtest, for configurations the machine must refuse. */
static int run_qemu(const char *args, char **err)
{
    g_autofree char *cmd = g_strdup_printf("%s -display none -serial none "
                                           "-monitor none %s -kernel %s",
                                           qtest_qemu_binary(NULL), args,
                                           image_path);
    g_auto(GStrv) argv = NULL;
    GError *gerr = NULL;
    int status;

    g_assert(g_shell_parse_argv(cmd, NULL, &argv, &gerr));
    g_assert(g_spawn_sync(NULL, argv, NULL, G_SPAWN_STDOUT_TO_DEV_NULL, NULL,
                          NULL, NULL, err, &status, &gerr));
    g_assert_no_error(gerr);
    return status;
}

static bool mtree_has(QTestState *qts, const char *needle)
{
    g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");

    return strstr(mtree, needle) != NULL;
}

/* Whether any pinned cache line is mapped as RAM in the XIP space. */
static bool cache_ram_mapped(QTestState *qts)
{
    g_autofree char *mtree = qtest_hmp(qts, "info mtree -f");
    g_auto(GStrv) lines = g_strsplit(mtree, "\n", -1);
    int i;

    for (i = 0; lines[i]; i++) {
        if (g_str_has_prefix(g_strstrip(lines[i]), "000000001") &&
            strstr(lines[i], "ram): rp2350.xip-cache")) {
            return true;
        }
    }
    return false;
}

/* Direct mode, chip select `cs` asserted for one command. */
static void cmd_begin(QTestState *qts, int cs)
{
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN |
                                  (CSR_ASSERT_CS0N << cs));
}

static void cmd_end(QTestState *qts)
{
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
}

static uint8_t xfer(QTestState *qts, uint8_t tx)
{
    qtest_writel(qts, DIRECT_TX, tx);
    g_assert_false(qtest_readl(qts, DIRECT_CSR) & CSR_RXEMPTY);
    return qtest_readl(qts, DIRECT_RX);
}

static void flash_cmd_addr(QTestState *qts, uint8_t cmd, uint32_t addr)
{
    xfer(qts, cmd);
    xfer(qts, addr >> 16);
    xfer(qts, addr >> 8);
    xfer(qts, addr);
}

static void flash_simple(QTestState *qts, uint8_t cmd)
{
    cmd_begin(qts, 0);
    xfer(qts, cmd);
    cmd_end(qts);
}

static uint8_t flash_status(QTestState *qts)
{
    uint8_t sr;

    cmd_begin(qts, 0);
    xfer(qts, 0x05);
    sr = xfer(qts, 0);
    cmd_end(qts);
    return sr;
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_reset(void)
{
    QTestState *qts = start("flash-size=4M");
    int i;

    g_assert_cmphex(qtest_readl(qts, CTRL), ==, 0x83);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FIFO_EMPTY);
    g_assert_cmphex(qtest_readl(qts, CTR_HIT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CTR_ACC), ==, 0);
    g_assert_cmphex(qtest_readl(qts, STREAM_ADDR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, STREAM_CTR), ==, 0);

    g_assert_cmphex(qtest_readl(qts, DIRECT_CSR), ==,
                    0x01800000 | CSR_TXEMPTY | CSR_RXEMPTY);
    for (i = 0; i < 2; i++) {
        g_assert_cmphex(qtest_readl(qts, M_TIMING(i)), ==, 0x40000004);
        g_assert_cmphex(qtest_readl(qts, M_RFMT(i)), ==, 0x00001000);
        g_assert_cmphex(qtest_readl(qts, M_RCMD(i)), ==, 0x0000a003);
        g_assert_cmphex(qtest_readl(qts, M_WFMT(i)), ==, 0x00001000);
        g_assert_cmphex(qtest_readl(qts, M_WCMD(i)), ==, 0x0000a002);
    }
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, ATRANS(i)), ==,
                        0x04000000 | (i % 4) * 0x400);
    }

    /* Read/write masks. */
    qtest_writel(qts, M_TIMING(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, M_TIMING(1)), ==, 0xf3fff7ff);
    qtest_writel(qts, M_RFMT(0), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, M_RFMT(0)), ==, 0x1007d3ff);
    qtest_writel(qts, M_WCMD(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, M_WCMD(1)), ==, 0xffff);
    qtest_writel(qts, CTRL, 0xffffffff & ~CTRL_POWER_DOWN);
    g_assert_cmphex(qtest_readl(qts, CTRL), ==, 0xff3);

    /* Powering the cache down disables it, and it cannot be re-enabled. */
    qtest_writel(qts, CTRL + ALIAS_SET, CTRL_POWER_DOWN);
    g_assert_cmphex(qtest_readl(qts, CTRL), ==, 0xff8);
    qtest_writel(qts, CTRL + ALIAS_SET, CTRL_EN_SECURE);
    g_assert_cmphex(qtest_readl(qts, CTRL), ==, 0xff8);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_atomic_and_counters(void)
{
    QTestState *qts = start("flash-size=4M");

    qtest_writel(qts, ATRANS(2) + ALIAS_CLR, 0x07ff0000);
    g_assert_cmphex(qtest_readl(qts, ATRANS(2)), ==, 0x800);
    qtest_writel(qts, ATRANS(2) + ALIAS_SET, 0x00010000);
    g_assert_cmphex(qtest_readl(qts, ATRANS(2)), ==, 0x00010800);

    /* Accesses the access model sees are counted; any write clears. */
    qtest_readl(qts, XIP_UNTRANSLATED);
    qtest_readl(qts, XIP_UNTRANSLATED + 4);
    g_assert_cmphex(qtest_readl(qts, CTR_ACC), ==, 2);
    g_assert_cmphex(qtest_readl(qts, CTR_HIT), ==, 0);
    qtest_writel(qts, CTR_ACC + ALIAS_SET, 0);
    g_assert_cmphex(qtest_readl(qts, CTR_ACC), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_jedec_id(void)
{
    static const struct {
        const char *size;
        uint32_t id;
    } parts[] = {
        { "1M", 0xef4014 }, { "2M", 0xef4015 }, { "4M", 0xef4016 },
        { "8M", 0xef4017 }, { "16M", 0xef4018 },
    };
    int i;

    for (i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *opts = g_strdup_printf("flash-size=%s",
                                                parts[i].size);
        QTestState *qts = start(opts);
        uint32_t id;

        cmd_begin(qts, 0);
        xfer(qts, 0x9f);
        id = xfer(qts, 0) << 16;
        id |= xfer(qts, 0) << 8;
        id |= xfer(qts, 0);
        cmd_end(qts);
        g_assert_cmphex(id, ==, parts[i].id);

        /* Chip select 1 has no device: the bus reads 0. */
        cmd_begin(qts, 1);
        g_assert_cmphex(xfer(qts, 0x9f), ==, 0);
        g_assert_cmphex(xfer(qts, 0), ==, 0);
        cmd_end(qts);
        qtest_quit(qts);
    }
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_direct_fifos(void)
{
    QTestState *qts = start("flash-size=4M");
    uint32_t csr;
    int i;

    /* Direct mode off: pushes queue, up to the FIFO depth of four. */
    for (i = 0; i < 5; i++) {
        qtest_writel(qts, DIRECT_TX, 0xff);
    }
    csr = qtest_readl(qts, DIRECT_CSR);
    g_assert_cmpuint(CSR_TXLEVEL(csr), ==, 4);
    g_assert_true(csr & CSR_TXFULL);
    g_assert_false(csr & CSR_BUSY);

    /*
     * Enabled, the frames run until the RX FIFO is full; the fifth push
     * was dropped, so all four fit. A 16-bit record then stalls.
     */
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6) | CSR_EN);
    csr = qtest_readl(qts, DIRECT_CSR);
    g_assert_cmpuint(CSR_TXLEVEL(csr), ==, 0);
    g_assert_cmpuint(CSR_RXLEVEL(csr), ==, 4);
    g_assert_true(csr & CSR_RXFULL);
    qtest_writel(qts, DIRECT_TX, (1u << 18) | 0x1234);
    csr = qtest_readl(qts, DIRECT_CSR);
    g_assert_cmpuint(CSR_TXLEVEL(csr), ==, 1);
    g_assert_true(csr & CSR_BUSY);
    qtest_readl(qts, DIRECT_RX);
    csr = qtest_readl(qts, DIRECT_CSR);
    g_assert_cmpuint(CSR_TXLEVEL(csr), ==, 0);
    g_assert_cmpuint(CSR_RXLEVEL(csr), ==, 4);

    /* BUSY lasts the serial time of the frames, at SCK = clk_sys / 6. */
    g_assert_true(qtest_readl(qts, DIRECT_CSR) & CSR_BUSY);
    qtest_clock_step(qts, 10000);
    g_assert_false(qtest_readl(qts, DIRECT_CSR) & CSR_BUSY);

    /* NOPUSH frames leave nothing in the RX FIFO. */
    for (i = 0; i < 4; i++) {
        qtest_readl(qts, AUX_DIRECT_RX);
    }
    g_assert_true(qtest_readl(qts, DIRECT_CSR) & CSR_RXEMPTY);
    qtest_writel(qts, AUX_DIRECT_TX, (1u << 20) | 0x05);
    g_assert_true(qtest_readl(qts, DIRECT_CSR) & CSR_RXEMPTY);

    /* Direct mode takes the bus: the XIP views go away until it ends. */
    g_assert_false(mtree_has(qts, "rp2350.qspi-cs0"));
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6));
    g_assert_true(mtree_has(qts, "rp2350.qspi-cs0"));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_program_erase(void)
{
    QTestState *qts = start("flash-size=4M");
    static const uint32_t addr = 0x1000;
    int i;

    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr), ==, image_word(addr));

    flash_simple(qts, 0x06);
    g_assert_cmphex(flash_status(qts) & 0x02, ==, 0x02);
    cmd_begin(qts, 0);
    flash_cmd_addr(qts, 0x20, addr);
    cmd_end(qts);
    g_assert_cmphex(flash_status(qts) & 0x01, ==, 0);

    flash_simple(qts, 0x06);
    cmd_begin(qts, 0);
    flash_cmd_addr(qts, 0x02, addr);
    for (i = 0; i < 8; i++) {
        xfer(qts, 0x10 + i);
    }
    cmd_end(qts);

    /* A 03h read through direct mode sees the new data. */
    cmd_begin(qts, 0);
    flash_cmd_addr(qts, 0x03, addr);
    g_assert_cmphex(xfer(qts, 0), ==, 0x10);
    g_assert_cmphex(xfer(qts, 0), ==, 0x11);
    cmd_end(qts);
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(6));

    /* So do all three XIP windows; the rest of the sector is erased. */
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr), ==, 0x13121110);
    g_assert_cmphex(qtest_readl(qts, XIP_UNCACHED + addr + 4), ==,
                    0x17161514);
    g_assert_cmphex(qtest_readl(qts, XIP_UNTRANSLATED + addr), ==,
                    0x13121110);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr + 8), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr + 0xffc), ==,
                    0xffffffff);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr - 4), ==,
                    image_word(addr - 4));

    /*
     * Memory-mapped writes: dropped while the window is read-only, and
     * sent as 02h page programs when writable, which flash ignores
     * without a write enable.
     */
    qtest_writel(qts, XIP_BASE + addr + 8, 0);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr + 8), ==, 0xffffffff);
    qtest_writel(qts, CTRL + ALIAS_SET, CTRL_WRITABLE_M0);
    qtest_writel(qts, XIP_BASE + addr + 8, 0);
    qtest_writel(qts, XIP_UNTRANSLATED + addr + 8, 0);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + addr + 8), ==, 0xffffffff);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_atrans(void)
{
    QTestState *qts = start("flash-size=4M");

    /* Pane 0 starting one sector into flash. */
    qtest_writel(qts, ATRANS(0), 0x04000001);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE), ==, image_word(0x1000));
    g_assert_cmphex(qtest_readl(qts, XIP_UNCACHED + 8), ==,
                    image_word(0x1008));
    /* The untranslated window bypasses it. */
    g_assert_cmphex(qtest_readl(qts, XIP_UNTRANSLATED), ==, image_word(0));

    /* One sector in size: the rest of the pane is left unmapped. */
    qtest_writel(qts, ATRANS(0), 0x00010001);
    g_assert_true(mtree_has(qts, "0000000010000000-0000000010000fff "
                                 "(prio 0, romd): rp2350.qspi-cs0"));
    g_assert_true(mtree_has(qts, "0000000010001000-00000000103fffff "
                                 "(prio 0, i/o): rp2350-xip-space"));

    /* Translation wraps at 16 MiB; the 4 MiB part repeats below that. */
    qtest_writel(qts, ATRANS(3), 0x04000fff);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0xc01000), ==,
                    image_word(0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_pinned_lines(void)
{
    QTestState *qts = start("flash-size=4M");
    int i;

    /* Pin the top 16 KiB of the XIP space, outside both QMI windows. */
    for (i = 0; i < 0x4000; i += 8) {
        qtest_writeb(qts, XIP_MAINT + (XIP_SRAM - XIP_BASE) + i + 7, 0);
    }
    g_assert_true(mtree_has(qts, "0000000013ffc000-0000000013ffffff "
                                 "(prio 0, ram): rp2350.xip-cache"));
    qtest_writel(qts, XIP_SRAM, 0x12345678);
    qtest_writel(qts, XIP_SRAM + 0x3ffc, 0x9abcdef0);
    g_assert_cmphex(qtest_readl(qts, XIP_SRAM), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, XIP_SRAM + 0x3ffc), ==, 0x9abcdef0);

    /*
     * Enabled for Secure only, the lines are served by the access model,
     * which counts the hits.
     */
    qtest_writel(qts, CTRL + ALIAS_CLR, CTRL_EN_NONSECURE);
    g_assert_false(cache_ram_mapped(qts));
    qtest_writel(qts, CTR_ACC, 0);
    g_assert_cmphex(qtest_readl(qts, XIP_SRAM), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, CTR_HIT), ==, 1);
    g_assert_cmphex(qtest_readl(qts, CTR_ACC), ==, 1);

    /* The uncached window never queries the cache. */
    qtest_writel(qts, CTR_HIT, 0);
    qtest_readl(qts, XIP_UNCACHED + (XIP_SRAM - XIP_BASE));
    g_assert_cmphex(qtest_readl(qts, CTR_HIT), ==, 0);

    /* Clean operations leave pinned lines alone; invalidates free them. */
    qtest_writel(qts, CTRL + ALIAS_SET, CTRL_EN_NONSECURE);
    qtest_writeb(qts, XIP_MAINT + (XIP_SRAM - XIP_BASE) + 3, 0);
    qtest_writeb(qts, XIP_MAINT + (XIP_SRAM - XIP_BASE) + 1, 0);
    g_assert_cmphex(qtest_readl(qts, XIP_SRAM), ==, 0x12345678);
    qtest_writeb(qts, XIP_MAINT + (XIP_SRAM - XIP_BASE) + 2, 0);
    g_assert_true(mtree_has(qts, "0000000013ffc008-0000000013ffffff "
                                 "(prio 0, ram): rp2350.xip-cache"));
    /* Invalidate by set/way: way 1 holds the upper 8 KiB. */
    for (i = 0; i < 0x2000; i += 8) {
        qtest_writeb(qts, XIP_MAINT + 0x2000 + i, 0);
    }
    g_assert_true(mtree_has(qts, "0000000013ffc008-0000000013ffdfff "
                                 "(prio 0, ram): rp2350.xip-cache"));
    g_assert_false(mtree_has(qts, "rp2350.xip-cache @0000000000002000"));

    /* A pinned line in the flash window is read-only unless writable. */
    qtest_writeb(qts, XIP_MAINT + 0x100 + 7, 0);
    qtest_writel(qts, XIP_BASE + 0x100, 0x55aa55aa);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0x100), !=, 0x55aa55aa);
    qtest_writel(qts, CTRL + ALIAS_SET, CTRL_WRITABLE_M0);
    qtest_writel(qts, XIP_BASE + 0x100, 0x55aa55aa);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0x100), ==, 0x55aa55aa);
    g_assert_cmphex(qtest_readl(qts, XIP_UNCACHED + 0x100), ==,
                    image_word(0x100));
    /* Disabling the cache hides pinned lines. */
    qtest_writel(qts, CTRL + ALIAS_CLR, CTRL_EN_SECURE | CTRL_EN_NONSECURE);
    g_assert_cmphex(qtest_readl(qts, XIP_BASE + 0x100), ==,
                    image_word(0x100));
    g_assert_false(cache_ram_mapped(qts));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_stream(void)
{
    QTestState *qts = start("flash-size=4M");
    int i;

    qtest_writel(qts, STREAM_ADDR, XIP_BASE + 0x40);
    qtest_writel(qts, STREAM_CTR, 5);
    /* Streaming runs in the background, in serial time. */
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FIFO_EMPTY);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FIFO_FULL);
    g_assert_cmphex(qtest_readl(qts, STREAM_CTR), ==, 3);
    g_assert_cmphex(qtest_readl(qts, STREAM_ADDR), ==, XIP_BASE + 0x48);

    for (i = 0; i < 5; i++) {
        uint32_t reg = i % 2 ? AUX_STREAM : STREAM_FIFO;

        g_assert_cmphex(qtest_readl(qts, reg), ==, image_word(0x40 + 4 * i));
        qtest_clock_step(qts, 100000);
    }
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FIFO_EMPTY);
    g_assert_cmphex(qtest_readl(qts, STREAM_CTR), ==, 0);

    /* Writing 0 halts a stream. */
    qtest_writel(qts, STREAM_CTR, 100);
    qtest_writel(qts, STREAM_CTR, 0);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, STAT), ==, STAT_FIFO_EMPTY);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.xip+1/test] */
static void test_psram(void)
{
    g_autofree char *err = NULL;
    QTestState *qts;
    int i;

    qts = start("flash-size=4M,psram-size=8M");

    /* APS6404L read ID: 9Fh, three address bytes, then MF ID and KGD. */
    cmd_begin(qts, 1);
    flash_cmd_addr(qts, 0x9f, 0);
    g_assert_cmphex(xfer(qts, 0xff), ==, 0x0d);
    g_assert_cmphex(xfer(qts, 0xff), ==, 0x5d);
    g_assert_cmphex(xfer(qts, 0xff), ==, 0x26);
    cmd_end(qts);
    /* Into QPI mode, as pico-sdk's PSRAM setup does with AUTO_CS1N. */
    qtest_writel(qts, DIRECT_CSR, CSR_CLKDIV(10) | CSR_EN | (1u << 7));
    qtest_writel(qts, DIRECT_TX, 0x35);
    qtest_clock_step(qts, 10000);
    g_assert_false(qtest_readl(qts, DIRECT_CSR) & CSR_BUSY);
    qtest_writel(qts, DIRECT_CSR, 0);

    /* Read-only by default: writes are dropped. */
    qtest_writel(qts, XIP_PSRAM + 0x10, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, XIP_PSRAM + 0x10), ==, 0);

    /* pico-sdk's quad write format; then the window holds data. */
    qtest_writel(qts, M_WFMT(1), 0x000002aa | (1u << 12));
    qtest_writel(qts, M_WCMD(1), 0x38);
    qtest_writel(qts, CTRL + ALIAS_SET, CTRL_WRITABLE_M1);
    for (i = 0; i < 16; i++) {
        qtest_writel(qts, XIP_PSRAM + 0x7ffff0 + i * 4, 0x01010101 * i);
    }
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(qtest_readl(qts, XIP_PSRAM + 0x7ffff0 + i * 4), ==,
                        0x01010101 * i);
    }
    /* Address bits above the part's 8 MiB are ignored. */
    g_assert_cmphex(qtest_readl(qts, XIP_PSRAM + 0xfffff4), ==, 0x01010101);
    g_assert_cmphex(qtest_readb(qts, XIP_UNCACHED + 0x1000000 + 0x7ffff5),
                    ==, 1);
    qtest_quit(qts);

    /* The APS6404L is the one PSRAM part modelled. */
    g_assert_cmpint(run_qemu("-M rp2350,flash-size=4M,psram-size=4M", &err),
                    !=, 0);
    g_assert(strstr(err, "psram-size"));
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    write_image();

    qtest_add_func("/rp2350-xip/reset", test_reset);
    qtest_add_func("/rp2350-xip/atomic-and-counters",
                   test_atomic_and_counters);
    qtest_add_func("/rp2350-xip/jedec-id", test_jedec_id);
    qtest_add_func("/rp2350-xip/direct-fifos", test_direct_fifos);
    qtest_add_func("/rp2350-xip/program-erase", test_program_erase);
    qtest_add_func("/rp2350-xip/atrans", test_atrans);
    qtest_add_func("/rp2350-xip/pinned-lines", test_pinned_lines);
    qtest_add_func("/rp2350-xip/stream", test_stream);
    qtest_add_func("/rp2350-xip/psram", test_psram);

    ret = g_test_run();
    unlink(image_path);
    g_free(image_path);
    return ret;
}
