/*
 * QTest testcase for the RP2350 DMA controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses count as a Secure, privileged debugger. Accesses at lower
 * security levels are made by DMA channels assigned to those levels.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define DMA             0x50000000
#define CH(n)           (DMA + 0x40 * (n))
#define READ_ADDR       0x00
#define WRITE_ADDR      0x04
#define TRANS_COUNT     0x08
#define CTRL_TRIG       0x0c
#define AL1_CTRL        0x10
#define AL1_TRANS_COUNT_TRIG 0x1c
#define AL2_WRITE_ADDR_TRIG 0x2c
#define AL3_TRANS_COUNT 0x38
#define AL3_READ_ADDR_TRIG 0x3c
#define INTR            (DMA + 0x400)
#define INTE(n)         (DMA + 0x404 + 0x10 * (n))
#define INTF(n)         (DMA + 0x408 + 0x10 * (n))
#define INTS(n)         (DMA + 0x40c + 0x10 * (n))
#define TIMER(n)        (DMA + 0x440 + 4 * (n))
#define MULTI_CHAN_TRIGGER (DMA + 0x450)
#define SNIFF_CTRL      (DMA + 0x454)
#define SNIFF_DATA      (DMA + 0x458)
#define FIFO_LEVELS     (DMA + 0x460)
#define CHAN_ABORT      (DMA + 0x464)
#define N_CHANNELS      (DMA + 0x468)
#define SECCFG_CH(n)    (DMA + 0x480 + 4 * (n))
#define SECCFG_IRQ(n)   (DMA + 0x4c0 + 4 * (n))
#define SECCFG_MISC     (DMA + 0x4d0)
#define MPU_CTRL        (DMA + 0x500)
#define MPU_BAR(n)      (DMA + 0x504 + 8 * (n))
#define MPU_LAR(n)      (DMA + 0x508 + 8 * (n))
#define DBG_CTDREQ(n)   (DMA + 0x800 + 0x40 * (n))
#define DBG_TCR(n)      (DMA + 0x804 + 0x40 * (n))
#define SET             0x2000
#define CLR             0x3000

#define EN              (1u << 0)
#define HIGH_PRIORITY   (1u << 1)
#define SIZE_BYTE       (0u << 2)
#define SIZE_HALF       (1u << 2)
#define SIZE_WORD       (2u << 2)
#define INCR_READ       (1u << 4)
#define INCR_READ_REV   (1u << 5)
#define INCR_WRITE      (1u << 6)
#define INCR_WRITE_REV  (1u << 7)
#define RING_SIZE(n)    ((n) << 8)
#define RING_SEL_WRITE  (1u << 12)
#define CHAIN_TO(n)     ((n) << 13)
#define TREQ(n)         ((n) << 17)
#define IRQ_QUIET       (1u << 23)
#define BSWAP           (1u << 24)
#define SNIFF_EN_CH     (1u << 25)
#define BUSY            (1u << 26)
#define WRITE_ERROR     (1u << 29)
#define READ_ERROR      (1u << 30)
#define AHB_ERROR       (1u << 31)

#define TREQ_UART0_TX   28
#define TREQ_UART0_RX   29
#define TREQ_PWM_WRAP0  32
#define TREQ_XIP_STREAM 49
#define TREQ_TIMER0     0x3b
#define TREQ_PERMANENT  0x3f

#define MODE_TRIGGER_SELF (1u << 28)
#define MODE_ENDLESS    (0xfu << 28)

#define SNIFF_EN        (1u << 0)
#define SNIFF_DMACH(n)  ((n) << 1)
#define SNIFF_CALC(n)   ((n) << 5)
#define SNIFF_OUT_REV   (1u << 10)
#define SNIFF_OUT_INV   (1u << 11)

#define LAR_EN          (1u << 0)
#define LAR_P           (1u << 1)
#define LAR_S           (1u << 2)

#define ACCESSCTRL_DMA  0x40060044
#define ACCESSCTRL_UART0 0x400600a0
#define PASSWORD        0xacce0000u

#define UART0           0x40070000
#define UART_DR         0x00
#define UART_IBRD       0x24
#define UART_FBRD       0x28
#define UART_LCR_H      0x2c
#define UART_CR         0x30
#define UART_DMACR      0x48

#define BUSCTRL         0x40068000
#define PERFCTR_EN      (BUSCTRL + 0x08)
#define PERFCTR(n)      (BUSCTRL + 0x0c + 8 * (n))
#define PERFSEL(n)      (BUSCTRL + 0x10 + 8 * (n))

#define XIP_BASE        0x10000000
#define XIP_STREAM_ADDR 0x400c8014
#define XIP_STREAM_CTR  0x400c8018
#define XIP_AUX_STREAM  0x50500000
#define IMAGE_WORDS     256

#define TICKS           0x40108000
#define SIO             0xd0000000

#define SRAM            0x20000000
#define SRC             (SRAM + 0x1000)
#define DST             (SRAM + 0x2000)
#define CB              (SRAM + 0x3000)

#define US              1000

static char *rom_path;
static char *image_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

/* Run the machine long enough for any short transfer to finish. */
static void settle(QTestState *qts)
{
    qtest_clock_step(qts, 100 * US);
}

static void fill(QTestState *qts, uint32_t addr, int words, uint32_t seed)
{
    int i;

    for (i = 0; i < words; i++) {
        qtest_writel(qts, addr + 4 * i, seed + i * 0x01010101u);
    }
}

static void setup(QTestState *qts, int ch, uint32_t from, uint32_t to,
                  uint32_t count)
{
    qtest_writel(qts, CH(ch) + READ_ADDR, from);
    qtest_writel(qts, CH(ch) + WRITE_ADDR, to);
    qtest_writel(qts, CH(ch) + TRANS_COUNT, count);
}

/* An unpaced word copy that chains to nothing. */
static uint32_t copy_ctrl(int ch)
{
    return EN | SIZE_WORD | INCR_READ | INCR_WRITE | CHAIN_TO(ch) |
           TREQ(TREQ_PERMANENT);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_reset(void)
{
    QTestState *qts = start();
    int n;

    for (n = 0; n < 16; n++) {
        g_assert_cmphex(qtest_readl(qts, CH(n) + CTRL_TRIG), ==, 0);
        g_assert_cmphex(qtest_readl(qts, CH(n) + TRANS_COUNT), ==, 0);
        g_assert_cmphex(qtest_readl(qts, SECCFG_CH(n)), ==, 3);
        g_assert_cmphex(qtest_readl(qts, DBG_TCR(n)), ==, 0);
    }
    for (n = 0; n < 4; n++) {
        g_assert_cmphex(qtest_readl(qts, SECCFG_IRQ(n)), ==, 3);
        g_assert_cmphex(qtest_readl(qts, INTE(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, TIMER(n)), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, SECCFG_MISC), ==, 0x3ff);
    g_assert_cmphex(qtest_readl(qts, N_CHANNELS), ==, 16);
    g_assert_cmphex(qtest_readl(qts, MPU_CTRL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, FIFO_LEVELS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SNIFF_CTRL), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_copy_and_irq(void)
{
    QTestState *qts = start();
    int i;

    qtest_irq_intercept_out_named(qts, "/machine/soc/dma", "sysbus-irq");
    fill(qts, SRC, 64, 0x11223344);
    qtest_writel(qts, INTE(1), 1u << 3);
    setup(qts, 3, SRC, DST, 64);
    qtest_writel(qts, CH(3) + CTRL_TRIG, copy_ctrl(3));

    /* Started, and 64 transfers take 64 cycles. */
    g_assert_cmphex(qtest_readl(qts, CH(3) + CTRL_TRIG) & BUSY, ==, BUSY);
    g_assert_false(qtest_get_irq(qts, 1));
    settle(qts);

    for (i = 0; i < 64; i++) {
        g_assert_cmphex(qtest_readl(qts, DST + 4 * i), ==,
                        0x11223344 + i * 0x01010101u);
    }
    g_assert_cmphex(qtest_readl(qts, CH(3) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(3) + TRANS_COUNT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(3) + READ_ADDR), ==, SRC + 256);
    g_assert_cmphex(qtest_readl(qts, CH(3) + WRITE_ADDR), ==, DST + 256);
    g_assert_cmphex(qtest_readl(qts, DBG_TCR(3)), ==, 64);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, INTS(1)), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, INTS(0)), ==, 0);
    g_assert_true(qtest_get_irq(qts, 1));
    g_assert_false(qtest_get_irq(qts, 0));

    /* Writing INTS acknowledges; INTF forces. */
    qtest_writel(qts, INTS(1), 1u << 3);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    g_assert_false(qtest_get_irq(qts, 1));
    qtest_writel(qts, INTE(2), 1u << 7);
    qtest_writel(qts, INTF(2), 1u << 7);
    g_assert_true(qtest_get_irq(qts, 2));
    g_assert_cmphex(qtest_readl(qts, INTS(2)), ==, 1u << 7);
    qtest_writel(qts, INTF(2), 0);
    g_assert_false(qtest_get_irq(qts, 2));

    /* The SET alias of INTE adds a channel. */
    qtest_writel(qts, INTE(1) + SET, 1u << 5);
    g_assert_cmphex(qtest_readl(qts, INTE(1)), ==, (1u << 3) | (1u << 5));

    /* A zero-length trigger completes at once, with its interrupt. */
    qtest_writel(qts, CH(4) + TRANS_COUNT, 0);
    qtest_writel(qts, CH(4) + CTRL_TRIG, copy_ctrl(4));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(4) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1u << 4);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_sizes_and_increments(void)
{
    QTestState *qts = start();
    int i;

    for (i = 0; i < 16; i++) {
        qtest_writeb(qts, SRC + i, i + 1);
    }

    /* Halfwords, reading backwards, writing to every other halfword. */
    setup(qts, 0, SRC + 6, DST, 4);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_HALF | INCR_READ |
                 INCR_READ_REV | INCR_WRITE_REV | TREQ(TREQ_PERMANENT));
    settle(qts);
    g_assert_cmphex(qtest_readw(qts, DST + 0), ==, 0x0807);
    g_assert_cmphex(qtest_readw(qts, DST + 4), ==, 0x0605);
    g_assert_cmphex(qtest_readw(qts, DST + 8), ==, 0x0403);
    g_assert_cmphex(qtest_readw(qts, DST + 12), ==, 0x0201);
    g_assert_cmphex(qtest_readl(qts, CH(0) + READ_ADDR), ==, SRC - 2);
    g_assert_cmphex(qtest_readl(qts, CH(0) + WRITE_ADDR), ==, DST + 16);

    /* Bytes to a fixed address; byte swap of words. */
    setup(qts, 1, SRC, DST + 0x100, 3);
    qtest_writel(qts, CH(1) + CTRL_TRIG, EN | SIZE_BYTE | INCR_READ |
                 CHAIN_TO(1) | TREQ(TREQ_PERMANENT));
    settle(qts);
    g_assert_cmphex(qtest_readb(qts, DST + 0x100), ==, 0x03);
    setup(qts, 2, SRC, DST + 0x200, 1);
    qtest_writel(qts, CH(2) + CTRL_TRIG, copy_ctrl(2) | BSWAP);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, DST + 0x200), ==, 0x01020304);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_ring(void)
{
    QTestState *qts = start();
    int i;

    fill(qts, SRC, 4, 0xa0a0a0a0);

    /* A 16-byte read ring: the source repeats every four words. */
    setup(qts, 0, SRC, DST, 10);
    qtest_writel(qts, CH(0) + CTRL_TRIG, copy_ctrl(0) | RING_SIZE(4));
    settle(qts);
    for (i = 0; i < 10; i++) {
        g_assert_cmphex(qtest_readl(qts, DST + 4 * i), ==,
                        0xa0a0a0a0 + (i % 4) * 0x01010101u);
    }
    g_assert_cmphex(qtest_readl(qts, CH(0) + READ_ADDR), ==, SRC + 8);
    g_assert_cmphex(qtest_readl(qts, CH(0) + WRITE_ADDR), ==, DST + 40);

    /* An 8-byte write ring keeps the last two words. */
    setup(qts, 1, SRC, DST + 0x100, 4);
    qtest_writel(qts, CH(1) + CTRL_TRIG, copy_ctrl(1) | RING_SIZE(3) |
                 RING_SEL_WRITE);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, DST + 0x100), ==, 0xa2a2a2a2);
    g_assert_cmphex(qtest_readl(qts, DST + 0x104), ==, 0xa3a3a3a3);
    g_assert_cmphex(qtest_readl(qts, CH(1) + WRITE_ADDR), ==, DST + 0x100);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_chain_and_control_blocks(void)
{
    QTestState *qts = start();
    static const uint32_t words[] = { 0x10, 0x20, 0x30 };
    int i;

    /* Channel 0 chains to channel 1. */
    fill(qts, SRC, 4, 0x01000000);
    setup(qts, 0, SRC, DST, 2);
    setup(qts, 1, SRC + 8, DST + 8, 2);
    qtest_writel(qts, CH(1) + AL1_CTRL, copy_ctrl(1));
    qtest_writel(qts, CH(0) + CTRL_TRIG, copy_ctrl(0) | CHAIN_TO(1));
    settle(qts);
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, DST + 4 * i), ==,
                        0x01000000 + i * 0x01010101u);
    }
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 3);
    qtest_writel(qts, INTR, 3);

    /*
     * Control blocks: channel 2 writes (TRANS_COUNT, READ_ADDR_TRIG)
     * pairs into channel 3's alias 3; channel 3 gathers each block into
     * a FIFO-like fixed word and chains back. The null block ends the
     * list and, with IRQ_QUIET, raises channel 3's only interrupt.
     */
    for (i = 0; i < 3; i++) {
        qtest_writel(qts, SRC + 0x100 + 4 * i, words[i]);
        qtest_writel(qts, CB + 8 * i, 1);
        qtest_writel(qts, CB + 8 * i + 4, SRC + 0x100 + 4 * i);
    }
    qtest_writel(qts, CB + 24, 0);
    qtest_writel(qts, CB + 28, 0);
    qtest_writel(qts, CH(3) + WRITE_ADDR, DST + 0x200);
    qtest_writel(qts, CH(3) + AL1_CTRL, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(2) | IRQ_QUIET | TREQ(TREQ_PERMANENT));
    setup(qts, 2, CB, CH(3) + AL3_TRANS_COUNT, 2);
    qtest_writel(qts, CH(2) + CTRL_TRIG, EN | SIZE_WORD | INCR_READ |
                 INCR_WRITE | RING_SIZE(3) | RING_SEL_WRITE | CHAIN_TO(2) |
                 IRQ_QUIET | TREQ(TREQ_PERMANENT));
    settle(qts);
    for (i = 0; i < 3; i++) {
        g_assert_cmphex(qtest_readl(qts, DST + 0x200 + 4 * i), ==,
                        words[i]);
    }
    g_assert_cmphex(qtest_readl(qts, CH(2) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(3) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1u << 3);

    /* MULTI_CHAN_TRIGGER starts channels without touching them. */
    qtest_writel(qts, INTR, 0xffff);
    setup(qts, 5, SRC, DST + 0x300, 1);
    setup(qts, 6, SRC + 4, DST + 0x304, 1);
    qtest_writel(qts, CH(5) + AL1_CTRL, copy_ctrl(5));
    qtest_writel(qts, CH(6) + AL1_CTRL, copy_ctrl(6));
    g_assert_cmphex(qtest_readl(qts, CH(5) + CTRL_TRIG) & BUSY, ==, 0);
    qtest_writel(qts, MULTI_CHAN_TRIGGER, (1u << 5) | (1u << 6));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, DST + 0x300), ==, 0x01000000);
    g_assert_cmphex(qtest_readl(qts, DST + 0x304), ==, 0x02010101);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, (1u << 5) | (1u << 6));
    g_assert_cmphex(qtest_readl(qts, MULTI_CHAN_TRIGGER), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_count_modes_and_abort(void)
{
    QTestState *qts = start();

    fill(qts, SRC, 4, 0x55000000);

    /* TRIGGER_SELF restarts after each sequence, interrupting each time. */
    setup(qts, 0, SRC, DST, MODE_TRIGGER_SELF | 4);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_WORD | INCR_READ |
                 RING_SIZE(4) | CHAIN_TO(0) | TREQ(TREQ_TIMER0));
    qtest_writel(qts, TIMER(0), (1u << 16) | 150);    /* one per us */
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(qtest_readl(qts, CH(0) + CTRL_TRIG) & BUSY, ==, BUSY);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1);
    qtest_writel(qts, INTR, 1);
    qtest_clock_step(qts, 4 * US);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT) >> 28, ==, 1);

    /* Abort stops it; the counter clears. */
    qtest_writel(qts, CHAN_ABORT, 1);
    g_assert_cmphex(qtest_readl(qts, CHAN_ABORT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(0) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT) & 0x0fffffff, ==,
                    0);

    /* ENDLESS never counts down or completes. */
    qtest_writel(qts, INTR, 1);
    setup(qts, 1, SRC, DST + 0x100, MODE_ENDLESS | 2);
    qtest_writel(qts, CH(1) + CTRL_TRIG, EN | SIZE_WORD | INCR_READ |
                 RING_SIZE(4) | CHAIN_TO(1) | TREQ(TREQ_PERMANENT));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(1) + CTRL_TRIG) & BUSY, ==, BUSY);
    g_assert_cmphex(qtest_readl(qts, CH(1) + TRANS_COUNT), ==,
                    MODE_ENDLESS | 2);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);

    /* Clearing EN pauses it, BUSY held; abort ends it. */
    qtest_writel(qts, CH(1) + AL1_CTRL + CLR, EN);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(1) + CTRL_TRIG) & BUSY, ==, BUSY);
    qtest_writel(qts, CHAN_ABORT, 2);
    g_assert_cmphex(qtest_readl(qts, CH(1) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    qtest_quit(qts);
}

/* Fill SRAM with `len` bytes and sniff them through channel 0. */
static uint32_t sniff_bytes(QTestState *qts, const char *data, int len,
                            uint32_t calc, uint32_t seed, uint32_t size)
{
    int i;

    for (i = 0; i < len; i++) {
        qtest_writeb(qts, SRC + i, data[i]);
    }
    qtest_writel(qts, SNIFF_CTRL, calc);
    qtest_writel(qts, SNIFF_DATA, seed);
    setup(qts, 0, SRC, DST, len / (size == SIZE_WORD ? 4 : 1));
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | size | INCR_READ |
                 SNIFF_EN_CH | CHAIN_TO(0) | TREQ(TREQ_PERMANENT));
    settle(qts);
    return qtest_readl(qts, SNIFF_DATA);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_sniff(void)
{
    QTestState *qts = start();
    const uint32_t sel = SNIFF_EN | SNIFF_DMACH(0);

    /* The IEEE 802.3 CRC of "123456789", by bytes and by words. */
    g_assert_cmphex(sniff_bytes(qts, "123456789", 9,
                                sel | SNIFF_CALC(1) | SNIFF_OUT_REV |
                                SNIFF_OUT_INV, 0xffffffff, SIZE_BYTE),
                    ==, 0xcbf43926);
    g_assert_cmphex(sniff_bytes(qts, "12345678", 8,
                                sel | SNIFF_CALC(1) | SNIFF_OUT_REV |
                                SNIFF_OUT_INV, 0xffffffff, SIZE_WORD),
                    ==, 0x9ae0daaf);
    /* MSB-first CRC-32 (CRC-32/BZIP2). */
    g_assert_cmphex(sniff_bytes(qts, "123456789", 9,
                                sel | SNIFF_CALC(0) | SNIFF_OUT_INV,
                                0xffffffff, SIZE_BYTE), ==, 0xfc891918);
    /* CRC-16-CCITT (CCITT-FALSE) and its reflected form (X-25). */
    g_assert_cmphex(sniff_bytes(qts, "123456789", 9, sel | SNIFF_CALC(2),
                                0xffff, SIZE_BYTE) & 0xffff, ==, 0x29b1);
    g_assert_cmphex(sniff_bytes(qts, "123456789", 9,
                                sel | SNIFF_CALC(3) | SNIFF_OUT_REV |
                                SNIFF_OUT_INV, 0xffff, SIZE_BYTE) >> 16,
                    ==, 0x906e);
    /* Sum and parity. */
    g_assert_cmphex(sniff_bytes(qts, "123456789", 9, sel | SNIFF_CALC(15),
                                0, SIZE_BYTE), ==, 0x1dd);
    g_assert_cmphex(sniff_bytes(qts, "1", 1, sel | SNIFF_CALC(14),
                                0, SIZE_BYTE), ==, 1);
    g_assert_cmphex(sniff_bytes(qts, "3", 1, sel | SNIFF_CALC(14),
                                0, SIZE_BYTE), ==, 0);
    /* Another channel is not observed. */
    g_assert_cmphex(sniff_bytes(qts, "123456789", 9,
                                SNIFF_EN | SNIFF_DMACH(1) | SNIFF_CALC(15),
                                0, SIZE_BYTE), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_pacing_timer(void)
{
    QTestState *qts = start();
    uint32_t left;

    /* X/Y = 1/150 of 150 MHz: one transfer per microsecond. */
    qtest_writel(qts, TIMER(1), (1u << 16) | 150);
    setup(qts, 0, SRC, DST, 1000);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(0) | TREQ(TREQ_TIMER0 + 1));
    qtest_clock_step(qts, 500 * US);
    left = qtest_readl(qts, CH(0) + TRANS_COUNT);
    g_assert_cmpuint(left, >=, 498);
    g_assert_cmpuint(left, <=, 502);

    /* 3/4 of clk_sys. */
    qtest_writel(qts, CHAN_ABORT, 1);
    qtest_writel(qts, TIMER(1), (3u << 16) | 4);
    setup(qts, 0, SRC, DST, 1000);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(0) | TREQ(TREQ_TIMER0 + 1));
    qtest_clock_step(qts, 4 * US);
    left = qtest_readl(qts, CH(0) + TRANS_COUNT);
    g_assert_cmpuint(left, >=, 1000 - 452);
    g_assert_cmpuint(left, <=, 1000 - 448);

    /*
     * A Non-secure channel does not see a Secure timer: SECCFG_MISC
     * resets every timer to SP.
     */
    qtest_writel(qts, CHAN_ABORT, 1);
    qtest_writel(qts, SECCFG_CH(2), 0);
    setup(qts, 2, SRC, DST, 10);
    qtest_writel(qts, CH(2) + CTRL_TRIG, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(2) | TREQ(TREQ_TIMER0 + 1));
    qtest_clock_step(qts, 20 * US);
    g_assert_cmphex(qtest_readl(qts, CH(2) + TRANS_COUNT), ==, 10);
    g_assert_cmphex(qtest_readl(qts, SECCFG_CH(2)), ==, 4);
    qtest_writel(qts, SECCFG_MISC, 0x3ff & ~(3u << 4));
    qtest_clock_step(qts, 20 * US);
    g_assert_cmphex(qtest_readl(qts, CH(2) + TRANS_COUNT), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_dreq_credits(void)
{
    QTestState *qts = start();
    const char *dma = "/machine/soc/dma";
    int i;

    /* Each DREQ pulse is one transfer; credits accumulate. */
    setup(qts, 0, SRC, DST, 10);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(0) | TREQ(TREQ_PWM_WRAP0));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT), ==, 10);
    for (i = 0; i < 3; i++) {
        qtest_set_irq_in(qts, dma, "dreq", TREQ_PWM_WRAP0, 1);
        qtest_set_irq_in(qts, dma, "dreq", TREQ_PWM_WRAP0, 0);
    }
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT), ==, 7);
    g_assert_cmphex(qtest_readl(qts, DBG_CTDREQ(0)), ==, 0);

    /* With EN clear, credits wait in the counter, which saturates. */
    qtest_writel(qts, CH(0) + AL1_CTRL + CLR, EN);
    for (i = 0; i < 70; i++) {
        qtest_set_irq_in(qts, dma, "dreq", TREQ_PWM_WRAP0, 1);
        qtest_set_irq_in(qts, dma, "dreq", TREQ_PWM_WRAP0, 0);
    }
    g_assert_cmphex(qtest_readl(qts, DBG_CTDREQ(0)), ==, 63);
    qtest_writel(qts, DBG_CTDREQ(0), 0);
    g_assert_cmphex(qtest_readl(qts, DBG_CTDREQ(0)), ==, 0);

    /*
     * DREQs of a block a channel's security level cannot access are
     * disconnected from it: PWM resets to Secure-only.
     */
    qtest_writel(qts, CHAN_ABORT, 1);
    qtest_writel(qts, SECCFG_CH(1), 1);
    setup(qts, 1, SRC, DST, 10);
    qtest_writel(qts, CH(1) + CTRL_TRIG, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(1) | TREQ(TREQ_PWM_WRAP0));
    qtest_set_irq_in(qts, dma, "dreq", TREQ_PWM_WRAP0, 1);
    qtest_set_irq_in(qts, dma, "dreq", TREQ_PWM_WRAP0, 0);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(1) + TRANS_COUNT), ==, 10);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_uart_dreq(void)
{
    QTestState *qts = start();
    int i;

    for (i = 0; i < 6; i++) {
        qtest_writeb(qts, SRC + i, "qemu!\n"[i]);
    }
    /*
     * UART0 in loopback, FIFOs on, 8 bits, at the fastest rate, clk_peri /
     * 16, which UARTLCR_H latches.
     */
    qtest_writel(qts, UART0 + UART_IBRD, 1);
    qtest_writel(qts, UART0 + UART_FBRD, 0);
    qtest_writel(qts, UART0 + UART_LCR_H, 0x70);
    qtest_writel(qts, UART0 + UART_CR, 0x381);

    /* RX first: it waits for data. TX waits for TXDMAE. */
    setup(qts, 1, UART0 + UART_DR, DST, 6);
    qtest_writel(qts, CH(1) + CTRL_TRIG, EN | SIZE_BYTE | INCR_WRITE |
                 CHAIN_TO(1) | TREQ(TREQ_UART0_RX));
    setup(qts, 0, SRC, UART0 + UART_DR, 6);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_BYTE | INCR_READ |
                 CHAIN_TO(0) | TREQ(TREQ_UART0_TX));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT), ==, 6);
    g_assert_cmphex(qtest_readl(qts, CH(1) + TRANS_COUNT), ==, 6);

    qtest_writel(qts, UART0 + UART_DMACR, 3);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(1) + TRANS_COUNT), ==, 0);
    for (i = 0; i < 6; i++) {
        g_assert_cmphex(qtest_readb(qts, DST + i), ==, "qemu!\n"[i]);
    }
    qtest_quit(qts);
}

/* The XIP stream FIFO feeds a channel through DREQ_XIP_STREAM. */
/* [spec:nuos:req:emu.dma/test] */
static void test_xip_stream_dreq(void)
{
    QTestState *qts = qtest_initf("-M rp2350,flash-size=4M -kernel %s",
                                  image_path);
    int i;

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    setup(qts, 0, XIP_AUX_STREAM, DST, 32);
    qtest_writel(qts, CH(0) + CTRL_TRIG, EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(0) | TREQ(TREQ_XIP_STREAM));
    qtest_writel(qts, XIP_STREAM_ADDR, XIP_BASE + 0x40);
    qtest_writel(qts, XIP_STREAM_CTR, 32);
    qtest_clock_step(qts, 10 * 1000 * US);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, XIP_STREAM_CTR), ==, 0);
    for (i = 0; i < 32; i++) {
        g_assert_cmphex(qtest_readl(qts, DST + 4 * i), ==,
                        0xa5000000 | (0x40 + 4 * i));
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_bus_errors(void)
{
    QTestState *qts = start();
    uint32_t ctrl;

    qtest_writel(qts, INTE(0), 0xffff);
    fill(qts, SRC, 4, 0x77000000);

    /* ACCESSCTRL denies the DMA TICKS: a read error, with interrupt. */
    setup(qts, 0, TICKS, DST, 4);
    qtest_writel(qts, CH(0) + CTRL_TRIG, copy_ctrl(0) | CHAIN_TO(1));
    qtest_writel(qts, CH(1) + AL1_CTRL, copy_ctrl(1));
    settle(qts);
    ctrl = qtest_readl(qts, CH(0) + CTRL_TRIG);
    g_assert_cmphex(ctrl & (BUSY | READ_ERROR | WRITE_ERROR | AHB_ERROR),
                    ==, READ_ERROR | AHB_ERROR);
    g_assert_cmphex(qtest_readl(qts, CH(0) + TRANS_COUNT), ==, 4);
    g_assert_cmphex(qtest_readl(qts, CH(0) + READ_ADDR), ==, TICKS + 4);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1);
    /* No chain on error. */
    g_assert_cmphex(qtest_readl(qts, CH(1) + CTRL_TRIG) & BUSY, ==, 0);

    /* The channel refuses triggers until the error is cleared. */
    qtest_writel(qts, CH(0) + READ_ADDR, SRC);
    qtest_writel(qts, CH(0) + AL1_TRANS_COUNT_TRIG, 4);
    g_assert_cmphex(qtest_readl(qts, CH(0) + CTRL_TRIG) & BUSY, ==, 0);
    qtest_writel(qts, CH(0) + AL1_CTRL + SET, READ_ERROR);
    g_assert_cmphex(qtest_readl(qts, CH(0) + CTRL_TRIG) & AHB_ERROR, ==, 0);
    qtest_writel(qts, CH(0) + WRITE_ADDR, DST);
    qtest_writel(qts, CH(0) + AL1_TRANS_COUNT_TRIG, 4);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, DST + 12), ==, 0x7a030303);

    /* SIO is not on the DMA's bus. */
    qtest_writel(qts, INTR, 0xffff);
    setup(qts, 2, SRC, SIO + 0x10, 1);
    qtest_writel(qts, CH(2) + CTRL_TRIG, copy_ctrl(2));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(2) + CTRL_TRIG) &
                    (READ_ERROR | WRITE_ERROR), ==, WRITE_ERROR);
    g_assert_cmphex(qtest_readl(qts, CH(2) + WRITE_ADDR), ==, SIO + 0x14);

    /*
     * The DMA MPU: region 0 makes DST+0x100.. SP-only. An NSU channel's
     * write there fails; an SP channel's succeeds.
     */
    qtest_writel(qts, MPU_BAR(0), DST + 0x100);
    qtest_writel(qts, MPU_LAR(0), (DST + 0x13f) | LAR_S | LAR_P | LAR_EN);
    qtest_writel(qts, SECCFG_CH(3), 0);
    setup(qts, 3, SRC, DST + 0x120, 1);
    qtest_writel(qts, DST + 0x120, 0);
    qtest_writel(qts, CH(3) + CTRL_TRIG, copy_ctrl(3));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(3) + CTRL_TRIG) &
                    (READ_ERROR | WRITE_ERROR), ==, WRITE_ERROR);
    g_assert_cmphex(qtest_readl(qts, DST + 0x120), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR) & (1u << 3), ==, 1u << 3);

    /* Its read of an SP-only source fails too. */
    qtest_writel(qts, SECCFG_CH(4), 0);
    setup(qts, 4, DST + 0x100, DST + 0x200, 1);
    qtest_writel(qts, CH(4) + CTRL_TRIG, copy_ctrl(4));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(4) + CTRL_TRIG) &
                    (READ_ERROR | WRITE_ERROR), ==, READ_ERROR);

    /* The lowest-numbered matching region wins; outside, MPU_CTRL. */
    qtest_writel(qts, MPU_BAR(1), DST + 0x120);
    qtest_writel(qts, MPU_LAR(1), (DST + 0x120) | LAR_EN);
    qtest_writel(qts, SECCFG_CH(5), 0);
    setup(qts, 5, SRC, DST + 0x120, 1);
    qtest_writel(qts, CH(5) + CTRL_TRIG, copy_ctrl(5));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(5) + CTRL_TRIG) & WRITE_ERROR, ==,
                    WRITE_ERROR);
    qtest_writel(qts, MPU_CTRL, 0x6);
    qtest_writel(qts, SECCFG_CH(6), 2);
    setup(qts, 6, SRC, DST + 0x400, 1);
    qtest_writel(qts, CH(6) + CTRL_TRIG, copy_ctrl(6));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(6) + CTRL_TRIG) & READ_ERROR, ==,
                    READ_ERROR);
    qtest_quit(qts);
}

/*
 * Have NSU channel 15 copy one word from `from` to `to`; return its
 * error flags.
 */
static uint32_t nsu_copy(QTestState *qts, uint32_t from, uint32_t to)
{
    qtest_writel(qts, CH(15) + AL1_CTRL + SET, READ_ERROR | WRITE_ERROR);
    setup(qts, 15, from, to, 1);
    qtest_writel(qts, CH(15) + CTRL_TRIG, copy_ctrl(15) | IRQ_QUIET);
    settle(qts);
    return qtest_readl(qts, CH(15) + CTRL_TRIG) & (READ_ERROR | WRITE_ERROR);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_security(void)
{
    QTestState *qts = start();

    /* Open the DMA's registers to Non-secure accesses in ACCESSCTRL. */
    qtest_writel(qts, ACCESSCTRL_DMA, PASSWORD | 0xff);
    qtest_writel(qts, SECCFG_CH(15), 0);
    qtest_writel(qts, SECCFG_CH(14), 0);
    qtest_writel(qts, SRC, 0x12345678);

    /* An SP channel's registers refuse an NSU write, and stay unlocked. */
    g_assert_cmphex(nsu_copy(qts, SRC, CH(0) + READ_ADDR), ==, WRITE_ERROR);
    g_assert_cmphex(qtest_readl(qts, CH(0) + READ_ADDR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SECCFG_CH(0)), ==, 3);
    g_assert_cmphex(nsu_copy(qts, CH(0) + CTRL_TRIG, DST), ==, READ_ERROR);

    /* An NSU channel's registers take it, and lock its SECCFG. */
    g_assert_cmphex(nsu_copy(qts, SRC, CH(14) + READ_ADDR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(14) + READ_ADDR), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SECCFG_CH(14)), ==, 4);
    qtest_writel(qts, SECCFG_CH(14), 3);
    g_assert_cmphex(qtest_readl(qts, SECCFG_CH(14)), ==, 4);

    /* SECCFG takes no unprivileged writes; reads are open. */
    g_assert_cmphex(nsu_copy(qts, SRC, SECCFG_CH(1)), ==, WRITE_ERROR);
    g_assert_cmphex(qtest_readl(qts, SECCFG_CH(1)), ==, 3);
    g_assert_cmphex(nsu_copy(qts, SECCFG_CH(1), DST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DST), ==, 3);

    /* INTR shows an NSU access only NSU channels, and clears only those. */
    qtest_writel(qts, INTR, 0xffff);
    qtest_writel(qts, CH(2) + TRANS_COUNT, 0);
    qtest_writel(qts, CH(2) + CTRL_TRIG, copy_ctrl(2));
    qtest_writel(qts, CH(14) + TRANS_COUNT, 0);
    qtest_writel(qts, CH(14) + CTRL_TRIG, copy_ctrl(14));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, (1u << 2) | (1u << 14));
    g_assert_cmphex(nsu_copy(qts, INTR, DST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DST), ==, 1u << 14);
    qtest_writel(qts, SRC + 4, 0xffff);
    g_assert_cmphex(nsu_copy(qts, SRC + 4, INTR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 1u << 2);

    /* IRQ registers of a Secure IRQ fault; a Non-secure IRQ hides SP. */
    g_assert_cmphex(nsu_copy(qts, INTS(0), DST), ==, READ_ERROR);
    /* Bus errors interrupt even IRQ_QUIET channels. */
    g_assert_cmphex(qtest_readl(qts, INTR), ==, (1u << 2) | (1u << 15));
    qtest_writel(qts, INTR, 1u << 15);
    qtest_writel(qts, SECCFG_IRQ(0), 0);
    qtest_writel(qts, INTE(0), 0xffff);
    g_assert_cmphex(qtest_readl(qts, INTS(0)), ==, 0);
    qtest_writel(qts, SECCFG_IRQ(0), 3);
    g_assert_cmphex(qtest_readl(qts, INTS(0)), ==, 1u << 2);

    /* MPU registers fault for unprivileged accesses. */
    g_assert_cmphex(nsu_copy(qts, MPU_CTRL, DST), ==, READ_ERROR);
    g_assert_cmphex(nsu_copy(qts, TIMER(0), DST), ==, READ_ERROR);
    g_assert_cmphex(nsu_copy(qts, SNIFF_DATA, DST), ==, READ_ERROR);

    /* Chaining never climbs: an NSU channel cannot start an SP one. */
    qtest_writel(qts, CH(3) + TRANS_COUNT, 1);
    qtest_writel(qts, CH(3) + AL1_CTRL, copy_ctrl(3));
    qtest_writel(qts, SECCFG_CH(13), 0);
    setup(qts, 13, SRC, DST + 0x500, 1);
    qtest_writel(qts, CH(13) + CTRL_TRIG, copy_ctrl(13) | CHAIN_TO(3));
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(13) + TRANS_COUNT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(3) + CTRL_TRIG) & BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(3) + TRANS_COUNT), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.dma/test] */
static void test_busctrl_counts_dma(void)
{
    QTestState *qts = start();

    /* SRAM4's ACCESS event: port 9. */
    qtest_writel(qts, PERFSEL(0), 4 * 9 + 3);
    qtest_writel(qts, PERFCTR_EN, 1);
    setup(qts, 0, SRAM + 0x40000, SRAM + 0x40100, 4);
    qtest_writel(qts, CH(0) + CTRL_TRIG, copy_ctrl(0));
    settle(qts);
    g_assert_cmpuint(qtest_readl(qts, PERFCTR(0)), ==, 2);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-dma-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    {
        uint32_t words[IMAGE_WORDS];
        int i;

        for (i = 0; i < IMAGE_WORDS; i++) {
            words[i] = cpu_to_le32(0xa5000000 | (i * 4));
        }
        fd = g_file_open_tmp("rp2350-dma-test-XXXXXX.img", &image_path, &err);
        g_assert_no_error(err);
        g_assert_cmpint(write(fd, words, sizeof(words)), ==, sizeof(words));
        close(fd);
    }

    qtest_add_func("/rp2350/dma/reset", test_reset);
    qtest_add_func("/rp2350/dma/copy-irq", test_copy_and_irq);
    qtest_add_func("/rp2350/dma/sizes-increments", test_sizes_and_increments);
    qtest_add_func("/rp2350/dma/ring", test_ring);
    qtest_add_func("/rp2350/dma/chain", test_chain_and_control_blocks);
    qtest_add_func("/rp2350/dma/modes-abort", test_count_modes_and_abort);
    qtest_add_func("/rp2350/dma/sniff", test_sniff);
    qtest_add_func("/rp2350/dma/pacing", test_pacing_timer);
    qtest_add_func("/rp2350/dma/dreq", test_dreq_credits);
    qtest_add_func("/rp2350/dma/uart-dreq", test_uart_dreq);
    qtest_add_func("/rp2350/dma/xip-stream-dreq", test_xip_stream_dreq);
    qtest_add_func("/rp2350/dma/bus-errors", test_bus_errors);
    qtest_add_func("/rp2350/dma/security", test_security);
    qtest_add_func("/rp2350/dma/busctrl", test_busctrl_counts_dma);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    unlink(image_path);
    g_free(image_path);
    return ret;
}
