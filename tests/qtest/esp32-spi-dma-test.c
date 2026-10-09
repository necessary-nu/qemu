/*
 * QTest testcase for the ESP32 SPI DMA engine and its DPORT channel selection
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define DPORT_PERIP_RST_EN      0x3ff000c4
#define PERIP_SPI_DMA           (1u << 22)
#define DPORT_SPI_DMA_CHAN_SEL  0x3ff005a8

#define SPI1                    0x3ff42000
#define SPI2                    0x3ff64000
#define SPI3                    0x3ff65000
#define APB_ALIAS(a)            ((a) - 0x3ff40000 + 0x60000000)

#define SPI_CMD                 0x00
#define SPI_ADDR                0x04
#define SPI_USER                0x1c
#define SPI_USER1               0x20
#define SPI_USER2               0x24
#define SPI_MOSI_DLEN           0x28
#define SPI_MISO_DLEN           0x2c
#define SPI_W0                  0x80

#define CMD_USR                 (1u << 18)
#define CMD_SE                  (1u << 24)
#define CMD_WREN                (1u << 30)
#define USER_COMMAND            (1u << 31)
#define USER_ADDR               (1u << 30)
#define USER_MISO               (1u << 28)
#define USER_MOSI               (1u << 27)

#define DMA_CONF                0x100
#define DMA_OUT_LINK            0x104
#define DMA_IN_LINK             0x108
#define DMA_STATUS              0x10c
#define DMA_INT_ENA             0x110
#define DMA_INT_RAW             0x114
#define DMA_INT_ST              0x118
#define DMA_INT_CLR             0x11c
#define IN_SUC_EOF_DES_ADDR     0x124
#define INLINK_DSCR             0x128
#define INLINK_DSCR_BF0         0x12c
#define INLINK_DSCR_BF1         0x130
#define OUT_EOF_BFR_DES_ADDR    0x134
#define OUT_EOF_DES_ADDR        0x138
#define OUTLINK_DSCR            0x13c
#define DMA_RSTATUS             0x148
#define DMA_TSTATUS             0x14c

#define CONF_IN_RST             (1u << 2)
#define CONF_OUT_RST            (1u << 3)
#define CONF_OUT_AUTO_WRBACK    (1u << 8)
#define CONF_OUT_EOF_MODE       (1u << 9)
#define LINK_STOP               (1u << 28)
#define LINK_START              (1u << 29)
#define LINK_RESTART            (1u << 30)
#define STATUS_RX_EN            (1u << 0)
#define STATUS_TX_EN            (1u << 1)

#define INT_INLINK_DSCR_EMPTY   (1u << 0)
#define INT_OUTLINK_DSCR_ERROR  (1u << 1)
#define INT_INLINK_DSCR_ERROR   (1u << 2)
#define INT_IN_DONE             (1u << 3)
#define INT_IN_SUC_EOF          (1u << 5)
#define INT_OUT_DONE            (1u << 6)
#define INT_OUT_EOF             (1u << 7)
#define INT_OUT_TOTAL_EOF       (1u << 8)

#define DW0_OWNER               (1u << 31)
#define DW0_EOF                 (1u << 30)
#define DW0(size, len)          ((size) | ((len) << 12))

/* DMA-capable internal SRAM 2, away from anything the ROM uses */
#define RAM                     0x3ffc0000
#define DSCR_A                  (RAM + 0x000)
#define DSCR_B                  (RAM + 0x010)
#define DSCR_C                  (RAM + 0x020)
#define BUF_A                   (RAM + 0x100)
#define BUF_B                   (RAM + 0x200)
#define BUF_C                   (RAM + 0x300)

#define FLASH_SIZE              (4 * 1024 * 1024)
#define FLASH_DATA              0x10000
#define FLASH_SCRATCH           0x200000

static char *flash_path;

static uint8_t pattern(uint32_t i)
{
    return (uint8_t)(i * 7 + 3);
}

static QTestState *start(void)
{
    return qtest_initf("-M esp32 -nic none "
                       "-drive file=%s,if=mtd,format=raw", flash_path);
}

static void dma_wr(QTestState *qts, uint32_t spi, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, spi + reg, v);
}

static uint32_t dma_rd(QTestState *qts, uint32_t spi, uint32_t reg)
{
    return qtest_readl(qts, spi + reg);
}

static void put_dscr(QTestState *qts, uint32_t at, uint32_t dw0, uint32_t buf,
                     uint32_t next)
{
    qtest_writel(qts, at, dw0);
    qtest_writel(qts, at + 4, buf);
    qtest_writel(qts, at + 8, next);
}

/* Read n bytes of flash from off through SPI1 with the READ command. */
static void flash_read_usr(QTestState *qts, uint32_t off, uint32_t n)
{
    qtest_writel(qts, SPI1 + SPI_USER, USER_COMMAND | USER_ADDR | USER_MISO);
    qtest_writel(qts, SPI1 + SPI_USER1, 23u << 26);
    qtest_writel(qts, SPI1 + SPI_USER2, (7u << 28) | 0x03);
    qtest_writel(qts, SPI1 + SPI_ADDR, off << 8);
    qtest_writel(qts, SPI1 + SPI_MISO_DLEN, n * 8 - 1);
    qtest_writel(qts, SPI1 + SPI_CMD, CMD_USR);
}

static void flash_wren(QTestState *qts)
{
    qtest_writel(qts, SPI1 + SPI_CMD, CMD_WREN);
}

/* [spec:nuos:req:emu.esp32.spi-dma/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    const uint32_t spis[] = { SPI1, SPI2, SPI3 };

    g_assert_cmphex(qtest_readl(qts, DPORT_SPI_DMA_CHAN_SEL), ==, 0);
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DPORT_SPI_DMA_CHAN_SEL), ==, 0x3f);
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 0);

    for (int i = 0; i < 3; i++) {
        uint32_t s = spis[i];

        g_assert_cmphex(dma_rd(qts, s, DMA_CONF), ==, CONF_OUT_EOF_MODE);
        for (uint32_t r = DMA_OUT_LINK; r <= DMA_INT_CLR; r += 4) {
            g_assert_cmphex(dma_rd(qts, s, r), ==, 0);
        }
        for (uint32_t r = 0x120; r <= OUTLINK_DSCR + 8; r += 4) {
            g_assert_cmphex(dma_rd(qts, s, r), ==, 0);
        }
        /* Idle FIFOs are empty. */
        g_assert_cmphex(dma_rd(qts, s, DMA_RSTATUS), ==, 1u << 31);
        g_assert_cmphex(dma_rd(qts, s, DMA_TSTATUS), ==, 1u << 31);

        /* Only the defined bits are writable; RO registers ignore writes. */
        dma_wr(qts, s, DMA_CONF, 0xffffffff & ~(CONF_IN_RST | CONF_OUT_RST |
                                               (3u << 4)));
        g_assert_cmphex(dma_rd(qts, s, DMA_CONF), ==, 0x1dfc0);
        dma_wr(qts, s, DMA_IN_LINK, 0x8fffffff);
        g_assert_cmphex(dma_rd(qts, s, DMA_IN_LINK), ==, 0x1fffff);
        dma_wr(qts, s, DMA_OUT_LINK, 0x8fffffff);
        g_assert_cmphex(dma_rd(qts, s, DMA_OUT_LINK), ==, 0xfffff);
        dma_wr(qts, s, DMA_INT_ENA, 0xffffffff);
        g_assert_cmphex(dma_rd(qts, s, DMA_INT_ENA), ==, 0x1ff);
        dma_wr(qts, s, DMA_INT_RAW, 0xffffffff);
        g_assert_cmphex(dma_rd(qts, s, DMA_INT_RAW), ==, 0);
        dma_wr(qts, s, IN_SUC_EOF_DES_ADDR, 0x1234);
        g_assert_cmphex(dma_rd(qts, s, IN_SUC_EOF_DES_ADDR), ==, 0);
        /* The APB alias reaches the same registers. */
        g_assert_cmphex(qtest_readl(qts, APB_ALIAS(s) + DMA_INT_ENA), ==,
                        0x1ff);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.spi-dma/test] */
static void test_channel_select(void)
{
    QTestState *qts = start();

    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 0), BUF_A, 0);

    /* No channel: START does nothing. */
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, INLINK_DSCR), ==, 0);

    /* The reserved selection 3 connects nothing either. */
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 3u << 2);
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);

    /* Two controllers on one channel: neither gets it. */
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, (1u << 2) | (1u << 4));
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    dma_wr(qts, SPI3, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI3, DMA_STATUS), ==, 0);

    /* SPI2 on channel 1, SPI3 on channel 2. */
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, (1u << 2) | (2u << 4));
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, STATUS_RX_EN);
    g_assert_cmphex(dma_rd(qts, SPI2, INLINK_DSCR), ==, DSCR_A);
    g_assert_cmphex(dma_rd(qts, SPI2, INLINK_DSCR_BF0), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, INLINK_DSCR_BF1), ==, BUF_A);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_TSTATUS), ==,
                    (1u << 31) | (DSCR_A & 0xfffff));
    /* START clears itself; the address stays. */
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_IN_LINK), ==, DSCR_A & 0xfffff);
    dma_wr(qts, SPI3, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI3, DMA_STATUS), ==, STATUS_RX_EN);

    /* STOP disables the link. */
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_STOP);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    /* IN_RST resets the inlink FSM. */
    dma_wr(qts, SPI3, DMA_CONF, CONF_OUT_EOF_MODE | CONF_IN_RST);
    g_assert_cmphex(dma_rd(qts, SPI3, DMA_STATUS), ==, 0);
    /* While held in reset the link cannot start. */
    dma_wr(qts, SPI3, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI3, DMA_STATUS), ==, 0);
    dma_wr(qts, SPI3, DMA_CONF, CONF_OUT_EOF_MODE);
    dma_wr(qts, SPI3, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI3, DMA_STATUS), ==, STATUS_RX_EN);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.spi-dma/test] */
static void test_descriptor_errors(void)
{
    QTestState *qts = start();

    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 1u << 2);
    dma_wr(qts, SPI2, DMA_INT_ENA, INT_INLINK_DSCR_ERROR |
           INT_OUTLINK_DSCR_ERROR);

    /* 0x3ff00000 + 0x10000 is DPORT space, not DMA-capable SRAM. */
    dma_wr(qts, SPI2, DMA_OUT_LINK, LINK_START | 0x10000);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==,
                    INT_OUTLINK_DSCR_ERROR);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_ST), ==,
                    INT_OUTLINK_DSCR_ERROR);
    dma_wr(qts, SPI2, DMA_INT_CLR, INT_OUTLINK_DSCR_ERROR);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==, 0);

    /* Misaligned descriptor. */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 16), BUF_A, 0);
    dma_wr(qts, SPI2, DMA_OUT_LINK, LINK_START | ((DSCR_A + 2) & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==,
                    INT_OUTLINK_DSCR_ERROR);
    dma_wr(qts, SPI2, DMA_INT_CLR, 0x1ff);

    /* A descriptor the CPU owns. */
    put_dscr(qts, DSCR_A, DW0(16, 16), BUF_A, 0);
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==,
                    INT_INLINK_DSCR_ERROR);
    g_assert_cmphex(dma_rd(qts, SPI2, INLINK_DSCR), ==, DSCR_A);
    dma_wr(qts, SPI2, DMA_INT_CLR, 0x1ff);

    /* A buffer in IRAM, outside the DMA address space. */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 16), 0x40080000, 0);
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==,
                    INT_INLINK_DSCR_ERROR);
    dma_wr(qts, SPI2, DMA_INT_CLR, 0x1ff);

    /* A buffer running off the end of SRAM 1. */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(32, 32), 0x3ffffff0, 0);
    dma_wr(qts, SPI2, DMA_OUT_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==,
                    INT_OUTLINK_DSCR_ERROR);
    dma_wr(qts, SPI2, DMA_INT_CLR, 0x1ff);

    /*
     * An outlink whose second descriptor is in IRAM: the error comes when
     * the engine moves on to it.
     */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(0, 0), BUF_A, 0x40080000);
    dma_wr(qts, SPI2, DMA_OUT_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_RAW), ==,
                    INT_OUT_DONE | INT_OUTLINK_DSCR_ERROR);
    g_assert_cmphex(dma_rd(qts, SPI2, OUTLINK_DSCR), ==, 0x40080000);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.spi-dma/test] */
static void test_inlink_read_flash(void)
{
    QTestState *qts = start();
    uint8_t got[64];

    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 2u << 0);
    dma_wr(qts, SPI1, DMA_INT_ENA, 0x1ff);

    /* 24 bytes into a 16-byte buffer and a 32-byte one. */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 0), BUF_A, DSCR_B);
    put_dscr(qts, DSCR_B, DW0_OWNER | DW0(32, 0), BUF_B, 0);
    qtest_memset(qts, BUF_A, 0xee, 64);
    qtest_memset(qts, BUF_B, 0xee, 64);
    dma_wr(qts, SPI1, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, STATUS_RX_EN);

    flash_read_usr(qts, FLASH_DATA, 24);

    qtest_memread(qts, BUF_A, got, 16);
    for (int i = 0; i < 16; i++) {
        g_assert_cmphex(got[i], ==, pattern(i));
    }
    qtest_memread(qts, BUF_B, got, 9);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(got[i], ==, pattern(16 + i));
    }
    g_assert_cmphex(got[8], ==, 0xee);
    /* The CPU buffer is untouched. */
    g_assert_cmphex(qtest_readl(qts, SPI1 + SPI_W0), ==, 0);

    /* Written back: lengths, eof on the last, both handed to the CPU. */
    g_assert_cmphex(qtest_readl(qts, DSCR_A), ==, DW0(16, 16));
    g_assert_cmphex(qtest_readl(qts, DSCR_B), ==, DW0_EOF | DW0(32, 8));
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_INT_RAW), ==,
                    INT_IN_DONE | INT_IN_SUC_EOF);
    g_assert_cmphex(dma_rd(qts, SPI1, IN_SUC_EOF_DES_ADDR), ==, DSCR_B);
    g_assert_cmphex(dma_rd(qts, SPI1, INLINK_DSCR), ==, DSCR_B);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, 0);
    dma_wr(qts, SPI1, DMA_INT_CLR, 0x1ff);

    /* Without the inlink, the data goes to W0..W15 again. */
    flash_read_usr(qts, FLASH_DATA, 4);
    g_assert_cmphex(qtest_readl(qts, SPI1 + SPI_W0), ==,
                    pattern(0) | pattern(1) << 8 | pattern(2) << 16 |
                    (uint32_t)pattern(3) << 24);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_INT_RAW), ==, 0);

    /* More data than descriptors: SPI_INLINK_DSCR_EMPTY_INT. */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(8, 0), BUF_A, 0);
    qtest_memset(qts, BUF_A, 0xee, 64);
    dma_wr(qts, SPI1, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    flash_read_usr(qts, FLASH_DATA, 12);
    qtest_memread(qts, BUF_A, got, 12);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(got[i], ==, pattern(i));
    }
    g_assert_cmphex(got[8], ==, 0xee);
    g_assert_cmphex(qtest_readl(qts, DSCR_A), ==, DW0(8, 8));
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_INT_RAW), ==,
                    INT_IN_DONE | INT_INLINK_DSCR_EMPTY);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.spi-dma/test] */
static void test_outlink_program_flash(void)
{
    QTestState *qts = start();
    uint8_t data[40], got[40];

    for (int i = 0; i < 40; i++) {
        data[i] = 0xa0 ^ i;
    }
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 1u << 0);
    dma_wr(qts, SPI1, DMA_CONF, CONF_OUT_EOF_MODE | CONF_OUT_AUTO_WRBACK);
    dma_wr(qts, SPI1, DMA_INT_ENA, 0x1ff);

    flash_wren(qts);
    qtest_writel(qts, SPI1 + SPI_ADDR, FLASH_SCRATCH);
    qtest_writel(qts, SPI1 + SPI_CMD, CMD_SE);

    /*
     * Two packets of 24 and 16 bytes: the first spans two descriptors,
     * with a zero-length one between them.
     */
    qtest_memwrite(qts, BUF_A, data, 16);
    qtest_memwrite(qts, BUF_B, data + 16, 8);
    qtest_memwrite(qts, BUF_C, data + 24, 16);
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 16), BUF_A, DSCR_A + 0x40);
    put_dscr(qts, DSCR_A + 0x40, DW0_OWNER | DW0(0, 0), BUF_A, DSCR_B);
    put_dscr(qts, DSCR_B, DW0_OWNER | DW0_EOF | DW0(8, 8), BUF_B, DSCR_C);
    put_dscr(qts, DSCR_C, DW0_OWNER | DW0_EOF | DW0(16, 16), BUF_C, 0);
    dma_wr(qts, SPI1, DMA_OUT_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, STATUS_TX_EN);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_RSTATUS), ==, DSCR_A & 0xfffff);

    flash_wren(qts);
    qtest_writel(qts, SPI1 + SPI_USER, USER_COMMAND | USER_ADDR | USER_MOSI);
    qtest_writel(qts, SPI1 + SPI_USER1, 23u << 26);
    qtest_writel(qts, SPI1 + SPI_USER2, (7u << 28) | 0x02);
    qtest_writel(qts, SPI1 + SPI_ADDR, FLASH_SCRATCH << 8);
    qtest_writel(qts, SPI1 + SPI_MOSI_DLEN, 24 * 8 - 1);
    qtest_writel(qts, SPI1 + SPI_CMD, CMD_USR);

    /* The first packet is out; the engine waits on the second. */
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_INT_RAW), ==,
                    INT_OUT_DONE | INT_OUT_EOF);
    g_assert_cmphex(dma_rd(qts, SPI1, OUT_EOF_DES_ADDR), ==, DSCR_B);
    g_assert_cmphex(dma_rd(qts, SPI1, OUT_EOF_BFR_DES_ADDR), ==, BUF_B);
    g_assert_cmphex(dma_rd(qts, SPI1, OUTLINK_DSCR), ==, DSCR_C);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, STATUS_TX_EN);
    /* SPI_OUT_AUTO_WRBACK hands the sent descriptors back. */
    g_assert_cmphex(qtest_readl(qts, DSCR_A), ==, DW0(16, 16));
    g_assert_cmphex(qtest_readl(qts, DSCR_B), ==, DW0_EOF | DW0(8, 8));
    g_assert_cmphex(qtest_readl(qts, DSCR_C), ==,
                    DW0_OWNER | DW0_EOF | DW0(16, 16));
    dma_wr(qts, SPI1, DMA_INT_CLR, 0x1ff);

    flash_wren(qts);
    qtest_writel(qts, SPI1 + SPI_ADDR, (FLASH_SCRATCH + 24) << 8);
    qtest_writel(qts, SPI1 + SPI_MOSI_DLEN, 16 * 8 - 1);
    qtest_writel(qts, SPI1 + SPI_CMD, CMD_USR);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_INT_RAW), ==,
                    INT_OUT_DONE | INT_OUT_EOF | INT_OUT_TOTAL_EOF);
    g_assert_cmphex(dma_rd(qts, SPI1, OUT_EOF_DES_ADDR), ==, DSCR_C);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_RSTATUS), ==,
                    (1u << 31) | (DSCR_C & 0xfffff));
    dma_wr(qts, SPI1, DMA_INT_CLR, 0x1ff);

    /* Read it back through the inlink. */
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(64, 0), BUF_A, 0);
    dma_wr(qts, SPI1, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    flash_read_usr(qts, FLASH_SCRATCH, 40);
    qtest_memread(qts, BUF_A, got, 40);
    g_assert_cmpmem(got, 40, data, 40);

    /* SPI_OUTLINK_RESTART carries on from a descriptor appended later. */
    dma_wr(qts, SPI1, DMA_INT_CLR, 0x1ff);
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0_EOF | DW0(4, 4), BUF_A, 0);
    qtest_writel(qts, DSCR_C + 8, DSCR_A);
    dma_wr(qts, SPI1, DMA_OUT_LINK, LINK_RESTART);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, STATUS_TX_EN);
    g_assert_cmphex(dma_rd(qts, SPI1, OUTLINK_DSCR), ==, DSCR_A);
    dma_wr(qts, SPI1, DMA_CONF, CONF_OUT_EOF_MODE | CONF_OUT_RST);
    g_assert_cmphex(dma_rd(qts, SPI1, DMA_STATUS), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.spi-dma/test] */
static void test_gate_and_reset(void)
{
    QTestState *qts = start();
    uint32_t clk = qtest_readl(qts, DPORT_PERIP_CLK_EN);

    g_assert_true(clk & PERIP_SPI_DMA);
    qtest_writel(qts, DPORT_SPI_DMA_CHAN_SEL, 1u << 2);
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 0), BUF_A, 0);
    dma_wr(qts, SPI2, DMA_INT_ENA, 0x1ff);

    /* Clock off: writes are dropped, registers hold their values. */
    qtest_writel(qts, DPORT_PERIP_CLK_EN, clk & ~PERIP_SPI_DMA);
    dma_wr(qts, SPI2, DMA_INT_ENA, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_ENA), ==, 0x1ff);
    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    /* The SPI controller's own registers are on a different gate. */
    qtest_writel(qts, SPI2 + SPI_W0, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SPI2 + SPI_W0), ==, 0x12345678);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, clk);

    dma_wr(qts, SPI2, DMA_IN_LINK, LINK_START | (DSCR_A & 0xfffff));
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, STATUS_RX_EN);
    dma_wr(qts, SPI3, DMA_INT_ENA, 0x3);

    /* SPI_DMA_RST resets the DMA state of all three controllers. */
    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_SPI_DMA);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_STATUS), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_ENA), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI3, DMA_INT_ENA), ==, 0);
    g_assert_cmphex(dma_rd(qts, SPI2, INLINK_DSCR), ==, 0);
    dma_wr(qts, SPI2, DMA_INT_ENA, 0x1ff);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_ENA), ==, 0);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    dma_wr(qts, SPI2, DMA_INT_ENA, 0x1ff);
    g_assert_cmphex(dma_rd(qts, SPI2, DMA_INT_ENA), ==, 0x1ff);
    /* The SPI controller was not reset. */
    g_assert_cmphex(qtest_readl(qts, SPI2 + SPI_W0), ==, 0x12345678);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_autofree uint8_t *img = g_malloc(FLASH_SIZE);
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    memset(img, 0xff, FLASH_SIZE);
    for (int i = 0; i < 4096; i++) {
        img[FLASH_DATA + i] = pattern(i);
    }
    fd = g_file_open_tmp("qtest.esp32-spi-dma.XXXXXX", &flash_path, NULL);
    g_assert(fd >= 0);
    g_assert(write(fd, img, FLASH_SIZE) == FLASH_SIZE);
    close(fd);

    qtest_add_func("esp32/spi-dma/reset-values", test_reset_values);
    qtest_add_func("esp32/spi-dma/channel-select", test_channel_select);
    qtest_add_func("esp32/spi-dma/descriptor-errors", test_descriptor_errors);
    qtest_add_func("esp32/spi-dma/inlink-read-flash", test_inlink_read_flash);
    qtest_add_func("esp32/spi-dma/outlink-program-flash",
                   test_outlink_program_flash);
    qtest_add_func("esp32/spi-dma/gate-and-reset", test_gate_and_reset);
    ret = g_test_run();
    unlink(flash_path);
    g_free(flash_path);
    return ret;
}
