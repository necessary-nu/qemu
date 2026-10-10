/*
 * QTest testcase for the ESP32-P4 external memory MMUs and caches
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest's accesses go through HP core 0's address space: at 0x4xxxxxxx
 * they go through its caches, as its cacheable accesses do, and at
 * 0x8xxxxxxx to the MSPI controllers and L2MEM directly. The flash is a
 * raw image with a pattern in each 64 KiB page.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define FLASH_SIZE          (16 * 1024 * 1024)
#define FLASH_CACHED        0x40000000
#define PSRAM_CACHED        0x48000000
#define FLASH_DIRECT        0x80000000
#define PSRAM_DIRECT        0x88000000
#define L2MEM_DIRECT        0x8ff00000
#define PAGE                0x10000

#define SPI0_FLASH          0x5008c000
#define SPI0_PSRAM          0x5008e000
#define CACHE_FCTRL         0x03c
#define INT_CLR             0x0c4
#define INT_RAW             0x0c8
#define AXI_ERR_ADDR        0x170
#define AXI_ERR_RESP_EN     0x17c
#define MMU_ITEM_CONTENT    0x37c
#define MMU_ITEM_INDEX      0x380
#define MMU_POWER_CTRL      0x384

#define CLOSE_AXI_INF_EN    (1u << 31)
#define AXI_RADDR_ERR       (1u << 7)
#define AXI_WADDR_ERR       (1u << 9)
#define FLASH_VALID         (1u << 12)
#define PSRAM_ACCESS        (1u << 10)
#define PSRAM_VALID         (1u << 11)

#define CACHE               0x3ff10000
#define SYNC_CTRL           0x098
#define SYNC_MAP            0x09c
#define SYNC_ADDR           0x0a0
#define SYNC_SIZE           0x0a4
#define L2_CACHESIZE_CONF   0x278
#define SYNC_INVALIDATE     (1u << 0)
#define SYNC_WRITEBACK      (1u << 2)
#define SYNC_DONE           (1u << 4)
#define MAP_L1_D            (1u << 4)
#define MAP_ALL             0x33

#define HP_SYS_CLKRST_HP_RST_EN0    0x500e60c0
#define RST_EN_MSPI_APB             (1u << 24)

#define HP_SYSTEM_L2_MEM_INT_RAW    0x500e509c
#define HP_SYSTEM_L2_MEM_INT_CLR    0x500e50a8
#define HP_SYSTEM_L2_MEM_RECORD0    0x500e50b0

/* The word at `off` of flash page `page` in the image. */
static uint32_t pattern(uint32_t page, uint32_t off)
{
    return 0xf1a50000u ^ (page << 20) ^ off;
}

static char *flash_image(void)
{
    g_autofree uint32_t *img = g_malloc(FLASH_SIZE);
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("esp32p4-mmu-XXXXXX.bin", &path, &err);
    uint32_t i;

    g_assert_no_error(err);
    for (i = 0; i < FLASH_SIZE / 4; i++) {
        img[i] = cpu_to_le32(pattern(i * 4 / PAGE, i * 4 % PAGE));
    }
    g_assert_cmpint(write(fd, img, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);
    return path;
}

static QTestState *start(char **image)
{
    *image = flash_image();
    return qtest_initf("-M esp32p4,flash-size=16M "
                       "-drive if=mtd,format=raw,file=%s", *image);
}

static void finish(QTestState *qts, char *image)
{
    qtest_quit(qts);
    unlink(image);
    g_free(image);
}

static void mmu_set(QTestState *qts, uint32_t spi0, uint32_t entry,
                    uint32_t v)
{
    qtest_writel(qts, spi0 + MMU_ITEM_INDEX, entry);
    qtest_writel(qts, spi0 + MMU_ITEM_CONTENT, v);
}

static uint32_t mmu_get(QTestState *qts, uint32_t spi0, uint32_t entry)
{
    qtest_writel(qts, spi0 + MMU_ITEM_INDEX, entry);
    return qtest_readl(qts, spi0 + MMU_ITEM_CONTENT);
}

static void axi_open(QTestState *qts, uint32_t spi0)
{
    qtest_writel(qts, spi0 + CACHE_FCTRL,
                 qtest_readl(qts, spi0 + CACHE_FCTRL) & ~CLOSE_AXI_INF_EN);
}

static void cache_sync(QTestState *qts, uint32_t op, uint32_t map,
                       uint32_t addr, uint32_t size)
{
    qtest_writel(qts, CACHE + SYNC_ADDR, addr);
    qtest_writel(qts, CACHE + SYNC_SIZE, size);
    qtest_writel(qts, CACHE + SYNC_MAP, map);
    qtest_writel(qts, CACHE + SYNC_CTRL, op);
    g_assert_cmphex(qtest_readl(qts, CACHE + SYNC_CTRL), ==, SYNC_DONE);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_table(void)
{
    char *image;
    QTestState *qts = start(&image);

    g_assert_cmphex(mmu_get(qts, SPI0_FLASH, 7), ==, 0);
    mmu_set(qts, SPI0_FLASH, 7, 0xffffffff);
    g_assert_cmphex(mmu_get(qts, SPI0_FLASH, 7), ==, 0x3fff);
    mmu_set(qts, SPI0_PSRAM, 1023, 0x0c05);
    g_assert_cmphex(mmu_get(qts, SPI0_PSRAM, 1023), ==, 0x0c05);
    g_assert_cmphex(mmu_get(qts, SPI0_FLASH, 1023), ==, 0);
    g_assert_cmphex(mmu_get(qts, SPI0_FLASH, 1024), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + MMU_ITEM_INDEX), ==, 1024);

    /* The controller's reset keeps the table; a system reset clears it. */
    axi_open(qts, SPI0_FLASH);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0) | RST_EN_MSPI_APB);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0) &
                 ~RST_EN_MSPI_APB);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + CACHE_FCTRL) &
                    CLOSE_AXI_INF_EN, ==, CLOSE_AXI_INF_EN);
    g_assert_cmphex(mmu_get(qts, SPI0_FLASH, 7), ==, 0x3fff);
    qtest_system_reset(qts);
    g_assert_cmphex(mmu_get(qts, SPI0_FLASH, 7), ==, 0);
    g_assert_cmphex(mmu_get(qts, SPI0_PSRAM, 1023), ==, 0);
    finish(qts, image);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_flash_pages(void)
{
    char *image;
    QTestState *qts = start(&image);

    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + CACHE_FCTRL) &
                    CLOSE_AXI_INF_EN, ==, CLOSE_AXI_INF_EN);
    g_assert_cmphex(qtest_readl(qts, SPI0_PSRAM + CACHE_FCTRL) &
                    CLOSE_AXI_INF_EN, ==, CLOSE_AXI_INF_EN);
    axi_open(qts, SPI0_FLASH);

    mmu_set(qts, SPI0_FLASH, 0, FLASH_VALID | 3);
    mmu_set(qts, SPI0_FLASH, 1, FLASH_VALID | 3);
    mmu_set(qts, SPI0_FLASH, 9, FLASH_VALID | 255);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 0x1234), ==,
                    pattern(3, 0x1234));
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + PAGE + 0xfffc), ==,
                    pattern(3, 0xfffc));
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 9 * PAGE + 8), ==,
                    pattern(255, 8));

    /* An invalid entry reads as zero, its address recorded. */
    qtest_writel(qts, SPI0_FLASH + INT_CLR, AXI_RADDR_ERR);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 2 * PAGE + 0x40), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + INT_RAW) & AXI_RADDR_ERR,
                    ==, AXI_RADDR_ERR);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + AXI_ERR_ADDR), ==,
                    2 * PAGE + 0x40);
    qtest_writel(qts, SPI0_FLASH + INT_CLR, AXI_RADDR_ERR);
    g_assert_cmphex(qtest_readl(qts, SPI0_FLASH + AXI_ERR_ADDR), ==, 0);

    /* MMU_PAGE_SIZE: 64, 32, 16 and 8 KiB. */
    for (int ps = 1; ps < 4; ps++) {
        uint32_t size = PAGE >> ps;

        qtest_writel(qts, SPI0_FLASH + MMU_POWER_CTRL,
                     (qtest_readl(qts, SPI0_FLASH + MMU_POWER_CTRL) &
                      ~0x18u) | (ps << 3));
        mmu_set(qts, SPI0_FLASH, 5, FLASH_VALID | 7);
        g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 5 * size + 4), ==,
                        pattern(7 * size / PAGE, 7 * size % PAGE + 4));
    }
    qtest_writel(qts, SPI0_FLASH + MMU_POWER_CTRL,
                 qtest_readl(qts, SPI0_FLASH + MMU_POWER_CTRL) & ~0x18u);
    finish(qts, image);
}

/*
 * The caches are virtually tagged: a changed MMU entry is seen through
 * them only once the lines from the old mapping are invalidated.
 */
/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_cached_remap(void)
{
    char *image;
    QTestState *qts = start(&image);

    axi_open(qts, SPI0_FLASH);
    mmu_set(qts, SPI0_FLASH, 2, FLASH_VALID | 3);
    g_assert_cmphex(qtest_readl(qts, FLASH_CACHED + 2 * PAGE + 0x100), ==,
                    pattern(3, 0x100));

    mmu_set(qts, SPI0_FLASH, 2, FLASH_VALID | 5);
    g_assert_cmphex(qtest_readl(qts, FLASH_DIRECT + 2 * PAGE + 0x100), ==,
                    pattern(5, 0x100));
    g_assert_cmphex(qtest_readl(qts, FLASH_CACHED + 2 * PAGE + 0x100), ==,
                    pattern(3, 0x100));
    cache_sync(qts, SYNC_INVALIDATE, MAP_ALL, FLASH_CACHED + 2 * PAGE, PAGE);
    g_assert_cmphex(qtest_readl(qts, FLASH_CACHED + 2 * PAGE + 0x100), ==,
                    pattern(5, 0x100));
    finish(qts, image);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_psram_pages(void)
{
    char *image;
    QTestState *qts = start(&image);

    axi_open(qts, SPI0_PSRAM);
    mmu_set(qts, SPI0_PSRAM, 0, PSRAM_VALID | PSRAM_ACCESS | 9);
    mmu_set(qts, SPI0_PSRAM, 4, PSRAM_VALID | PSRAM_ACCESS | 9);
    qtest_writel(qts, PSRAM_DIRECT + 0x80, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, PSRAM_DIRECT + 4 * PAGE + 0x80), ==,
                    0x12345678);

    /* A write through the caches reaches the PSRAM when written back. */
    qtest_writel(qts, PSRAM_CACHED + 4 * PAGE + 0x80, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, PSRAM_DIRECT + 0x80), ==, 0x12345678);
    cache_sync(qts, SYNC_WRITEBACK, MAP_ALL, PSRAM_CACHED + 4 * PAGE, PAGE);
    g_assert_cmphex(qtest_readl(qts, PSRAM_DIRECT + 0x80), ==, 0xcafef00d);

    /* Writes through an invalid entry: dropped, and recorded. */
    qtest_writel(qts, SPI0_PSRAM + INT_CLR, AXI_WADDR_ERR);
    qtest_writel(qts, PSRAM_DIRECT + 6 * PAGE, 1);
    g_assert_cmphex(qtest_readl(qts, SPI0_PSRAM + INT_RAW) & AXI_WADDR_ERR,
                    ==, AXI_WADDR_ERR);
    g_assert_cmphex(qtest_readl(qts, SPI0_PSRAM + AXI_ERR_ADDR), ==,
                    6 * PAGE);
    finish(qts, image);
}

/* [spec:nuos:req:emu.esp32p4.memory/test] */
static void test_l2mem_split(void)
{
    char *image;
    QTestState *qts = start(&image);
    static const struct {
        uint32_t conf;
        uint32_t size;
    } splits[] = {
        { 1u << 9, 128 * 1024 },
        { 1u << 10, 256 * 1024 },
        { 1u << 11, 512 * 1024 },
        { 0, 0 },
    };

    qtest_writel(qts, L2MEM_DIRECT + 0x7fffc, 0x600df00d);
    for (int i = 0; i < ARRAY_SIZE(splits); i++) {
        uint32_t size = splits[i].size;

        qtest_writel(qts, CACHE + L2_CACHESIZE_CONF, splits[i].conf);
        qtest_writel(qts, HP_SYSTEM_L2_MEM_INT_CLR, 7);
        if (size) {
            g_assert_cmphex(qtest_readl(qts, L2MEM_DIRECT + size - 4), ==, 0);
            g_assert_cmphex(qtest_readl(qts, HP_SYSTEM_L2_MEM_INT_RAW), ==,
                            2);
            g_assert_cmphex(qtest_readl(qts, HP_SYSTEM_L2_MEM_RECORD0), ==,
                            size - 4);
        }
        qtest_writel(qts, L2MEM_DIRECT + size, 0x1000 + i);
        g_assert_cmphex(qtest_readl(qts, L2MEM_DIRECT + size), ==, 0x1000 + i);
    }
    /* L2MEM keeps what it held under the cache. */
    g_assert_cmphex(qtest_readl(qts, L2MEM_DIRECT + 0x7fffc), ==, 0x600df00d);
    finish(qts, image);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/mmu/table", test_table);
    qtest_add_func("/esp32p4/mmu/flash-pages", test_flash_pages);
    qtest_add_func("/esp32p4/mmu/cached-remap", test_cached_remap);
    qtest_add_func("/esp32p4/mmu/psram-pages", test_psram_pages);
    qtest_add_func("/esp32p4/mmu/l2mem-split", test_l2mem_split);
    return g_test_run();
}
