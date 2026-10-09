/*
 * QTest testcase for the ESP32 UHCI (UART DMA) engines
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/sockets.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN  0x3ff000c0
#define DPORT_PERIP_RST_EN  0x3ff000c4
#define PERIP_UHCI0         (1u << 8)
#define PERIP_UHCI1         (1u << 12)

#define UHCI0               0x3ff54000
#define UHCI1               0x3ff4c000
#define CONF0               0x00
#define INT_RAW             0x04
#define INT_ST              0x08
#define INT_ENA             0x0c
#define INT_CLR             0x10
#define DMA_OUT_STATUS      0x14
#define DMA_OUT_PUSH        0x18
#define DMA_IN_STATUS       0x1c
#define DMA_IN_POP          0x20
#define DMA_OUT_LINK        0x24
#define DMA_IN_LINK         0x28
#define CONF1               0x2c
#define OUT_EOF_DES_ADDR    0x38
#define IN_SUC_EOF_DES_ADDR 0x3c
#define OUT_EOF_BFR_DES_ADDR 0x44
#define IN_DSCR             0x4c
#define IN_DSCR_BF0         0x50
#define OUT_DSCR            0x58
#define OUT_DSCR_BF0        0x5c
#define OUT_DSCR_BF1        0x60
#define ESCAPE_CONF         0x64
#define HUNG_CONF           0x68
#define QUICK_SENT          0x74
#define Q0_WORD0            0x78
#define Q0_WORD1            0x7c
#define ESC_CONF0           0xb0
#define ESC_CONF1           0xb4
#define ESC_CONF2           0xb8
#define ESC_CONF3           0xbc
#define PKT_THRES           0xc0
#define DATE                0xfc

#define CONF0_OUT_RST       (1u << 1)
#define CONF0_IN_RST        (1u << 0)
#define CONF0_OUT_AUTO_WRBACK (1u << 6)
#define CONF0_OUT_EOF_MODE  (1u << 8)
#define CONF0_UART0_CE      (1u << 9)
#define CONF0_SEPER_EN      (1u << 16)

#define INT_RX_START        (1u << 0)
#define INT_TX_START        (1u << 1)
#define INT_RX_HUNG         (1u << 2)
#define INT_TX_HUNG         (1u << 3)
#define INT_IN_DONE         (1u << 4)
#define INT_IN_SUC_EOF      (1u << 5)
#define INT_OUT_DONE        (1u << 7)
#define INT_OUT_EOF         (1u << 8)
#define INT_IN_DSCR_ERR     (1u << 9)
#define INT_OUT_DSCR_ERR    (1u << 10)
#define INT_IN_DSCR_EMPTY   (1u << 11)
#define INT_OUTLINK_EOF_ERR (1u << 12)
#define INT_OUT_TOTAL_EOF   (1u << 13)
#define INT_SEND_S_REG_Q    (1u << 14)
#define INT_DMA_INFIFO_FULL_WM (1u << 16)

#define LINK_STOP           (1u << 28)
#define LINK_START          (1u << 29)
#define LINK_RESTART        (1u << 30)
#define LINK_PARK           (1u << 31)

#define CONF1_CHECK_OWNER   (1u << 6)

#define DESC_OWNER          (1u << 31)
#define DESC_EOF            (1u << 30)

#define UART0_STATUS        0x3ff4001c

/* DMA-capable SRAM for descriptors and buffers */
#define RAM                 0x3ffc0000

#define US                  1000
#define MS                  (1000 * US)

static QTestState *start(int *sock)
{
    QTestState *qts = qtest_init_with_serial("-M esp32 -nic none", sock);

    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_UHCI0);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) & ~PERIP_UHCI0);
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, UHCI0 + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, UHCI0 + off, v);
}

static void put_desc(QTestState *qts, uint32_t at, uint32_t size,
                     uint32_t length, bool eof, uint32_t buf, uint32_t next)
{
    qtest_writel(qts, at, DESC_OWNER | (eof ? DESC_EOF : 0) |
                 (length << 12) | size);
    qtest_writel(qts, at + 4, buf);
    qtest_writel(qts, at + 8, next);
}

/* Run the machine until n bytes have come out of UART0. */
static void serial_read(QTestState *qts, int sock, uint8_t *buf, size_t n)
{
    size_t got = 0;

    for (int i = 0; i < 2000 && got < n; i++) {
        ssize_t r;

        qtest_clock_step(qts, MS);
        r = recv(sock, buf + got, n - got, MSG_DONTWAIT);
        if (r > 0) {
            got += r;
        }
    }
    g_assert_cmpuint(got, ==, n);
}

/* Nothing more comes out of UART0 within 20 ms. */
static void serial_quiet(QTestState *qts, int sock)
{
    uint8_t b;

    qtest_clock_step(qts, 20 * MS);
    g_assert_cmpint(recv(sock, &b, 1, MSG_DONTWAIT), <=, 0);
}

/* Run the machine until all of the bits are raised. */
static void wait_int(QTestState *qts, uint32_t bits)
{
    for (int i = 0; i < 2000; i++) {
        if ((rd(qts, INT_RAW) & bits) == bits) {
            return;
        }
        g_usleep(1000);
        qtest_clock_step(qts, 100 * US);
    }
    g_assert_cmphex(rd(qts, INT_RAW) & bits, ==, bits);
}

static void send_all(int sock, const void *data, size_t n)
{
    g_assert_cmpint(send(sock, data, n, 0), ==, n);
}

/* [spec:nuos:req:emu.esp32.uhci/test] */
static void test_reset_values(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");
    const uint32_t base[] = { UHCI0, UHCI1 };

    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) |
                 PERIP_UHCI0 | PERIP_UHCI1);
    for (int i = 0; i < 2; i++) {
        uint32_t b = base[i];

        g_assert_cmphex(qtest_readl(qts, b + CONF0), ==, 0x00370100);
        g_assert_cmphex(qtest_readl(qts, b + INT_RAW), ==, 0);
        g_assert_cmphex(qtest_readl(qts, b + INT_ENA), ==, 0);
        g_assert_cmphex(qtest_readl(qts, b + DMA_OUT_STATUS), ==, 2);
        g_assert_cmphex(qtest_readl(qts, b + DMA_IN_STATUS), ==, 2);
        g_assert_cmphex(qtest_readl(qts, b + DMA_OUT_LINK), ==, 0);
        g_assert_cmphex(qtest_readl(qts, b + DMA_IN_LINK), ==, 0x00100000);
        g_assert_cmphex(qtest_readl(qts, b + CONF1), ==, 0x33);
        g_assert_cmphex(qtest_readl(qts, b + ESCAPE_CONF), ==, 0x33);
        g_assert_cmphex(qtest_readl(qts, b + HUNG_CONF), ==, 0x00810810);
        g_assert_cmphex(qtest_readl(qts, b + ESC_CONF0), ==, 0x00dcdbc0);
        g_assert_cmphex(qtest_readl(qts, b + ESC_CONF1), ==, 0x00dddbdb);
        g_assert_cmphex(qtest_readl(qts, b + ESC_CONF2), ==, 0x00dedb11);
        g_assert_cmphex(qtest_readl(qts, b + ESC_CONF3), ==, 0x00dfdb13);
        g_assert_cmphex(qtest_readl(qts, b + PKT_THRES), ==, 0x80);
        g_assert_cmphex(qtest_readl(qts, b + DATE), ==, 0x16041001);
    }

    /* Register access types: RO status, WO clear, write masks */
    wr(qts, INT_RAW, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    wr(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_ENA), ==, 0x1ffff);
    wr(qts, CONF0, 0xffffffff & ~(CONF0_IN_RST | CONF0_OUT_RST | 0xc));
    g_assert_cmphex(rd(qts, CONF0), ==, 0x00fffff0);
    wr(qts, DMA_OUT_LINK, 0xffffffff & ~(LINK_START | LINK_RESTART));
    /* STOP self-clears; PARK is read-only */
    g_assert_cmphex(rd(qts, DMA_OUT_LINK), ==, 0x000fffff);
    wr(qts, PKT_THRES, 0xffffffff);
    g_assert_cmphex(rd(qts, PKT_THRES), ==, 0x1fff);
    wr(qts, QUICK_SENT, 0x77);
    g_assert_cmphex(rd(qts, QUICK_SENT), ==, 0x77);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.uhci/test] */
static void test_gated(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    /* UHCI0's clock is off after reset: writes do not land. */
    wr(qts, PKT_THRES, 0x10);
    g_assert_cmphex(rd(qts, PKT_THRES), ==, 0x80);
    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_UHCI0);
    wr(qts, PKT_THRES, 0x10);
    g_assert_cmphex(rd(qts, PKT_THRES), ==, 0x10);
    /* Its reset bit resets it. */
    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_UHCI0);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    g_assert_cmphex(rd(qts, PKT_THRES), ==, 0x80);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * An outlink with separators: the frame is wrapped in 0xc0 and the data's
 * 0xc0 and 0xdb are escaped.
 */
static void test_out_slip(void)
{
    int sock;
    QTestState *qts = start(&sock);
    static const uint8_t data[] = { 'A', 0xc0, 'B', 0xdb, 'C' };
    static const uint8_t want[] = {
        0xc0, 'A', 0xdb, 0xdc, 'B', 0xdb, 0xdd, 'C', 0xc0
    };
    uint8_t got[sizeof(want)];

    qtest_memwrite(qts, RAM + 0x100, data, sizeof(data));
    put_desc(qts, RAM, 8, sizeof(data), true, RAM + 0x100, 0);
    wr(qts, CONF0, CONF0_UART0_CE | CONF0_SEPER_EN | CONF0_OUT_EOF_MODE);
    wr(qts, INT_ENA, INT_OUT_TOTAL_EOF);
    qtest_irq_intercept_out_named(qts, "/machine/soc/uhci0", "sysbus-irq");
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));

    serial_read(qts, sock, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), want, sizeof(want));
    serial_quiet(qts, sock);

    g_assert_cmphex(rd(qts, INT_RAW), ==,
                    INT_RX_START | INT_OUT_DONE | INT_OUT_EOF |
                    INT_OUT_TOTAL_EOF);
    g_assert_cmphex(rd(qts, INT_ST), ==, INT_OUT_TOTAL_EOF);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, OUT_EOF_DES_ADDR), ==, RAM);
    g_assert_cmphex(rd(qts, OUT_EOF_BFR_DES_ADDR), ==, RAM + 0x100);
    g_assert_cmphex(rd(qts, OUT_DSCR), ==, RAM);
    g_assert_true(rd(qts, DMA_OUT_LINK) & LINK_PARK);
    /* Without OUT_AUTO_WRBACK the descriptor stays the DMA's. */
    g_assert_cmphex(qtest_readl(qts, RAM), &, DESC_OWNER);

    wr(qts, INT_CLR, INT_OUT_TOTAL_EOF);
    g_assert_cmphex(rd(qts, INT_ST), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * A chain of descriptors without separators goes out as it is, with each
 * descriptor written back to the CPU and the history registers following
 * the chain.
 */
static void test_out_chain(void)
{
    int sock;
    QTestState *qts = start(&sock);
    uint8_t got[10];

    qtest_memwrite(qts, RAM + 0x100, "hello", 5);
    qtest_memwrite(qts, RAM + 0x200, " uhci", 5);
    put_desc(qts, RAM, 8, 5, false, RAM + 0x100, RAM + 0x20);
    put_desc(qts, RAM + 0x20, 0, 0, false, 0, RAM + 0x40);
    put_desc(qts, RAM + 0x40, 8, 5, true, RAM + 0x200, 0);
    wr(qts, CONF0, CONF0_UART0_CE | CONF0_OUT_AUTO_WRBACK);
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));

    serial_read(qts, sock, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), "hello uhci", 10);
    g_assert_cmphex(rd(qts, INT_RAW), ==,
                    INT_OUT_DONE | INT_OUT_EOF | INT_OUT_TOTAL_EOF);
    g_assert_cmphex(rd(qts, OUT_DSCR), ==, RAM + 0x40);
    g_assert_cmphex(rd(qts, OUT_DSCR_BF0), ==, RAM + 0x20);
    g_assert_cmphex(rd(qts, OUT_DSCR_BF1), ==, RAM);
    g_assert_cmphex(rd(qts, OUT_EOF_DES_ADDR), ==, RAM + 0x40);
    for (int i = 0; i < 3; i++) {
        g_assert_false(qtest_readl(qts, RAM + i * 0x20) & DESC_OWNER);
    }
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * A list that ends without an EOF descriptor raises OUTLINK_EOF_ERR and
 * parks; RESTART takes up descriptors linked on to it since.
 */
static void test_out_restart(void)
{
    int sock;
    QTestState *qts = start(&sock);
    uint8_t got[6];

    qtest_memwrite(qts, RAM + 0x100, "abc", 3);
    qtest_memwrite(qts, RAM + 0x200, "def", 3);
    put_desc(qts, RAM, 4, 3, false, RAM + 0x100, 0);
    wr(qts, CONF0, CONF0_UART0_CE);
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));
    serial_read(qts, sock, got, 3);
    g_assert_cmpmem(got, 3, "abc", 3);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_OUT_DONE | INT_OUTLINK_EOF_ERR);
    g_assert_true(rd(qts, DMA_OUT_LINK) & LINK_PARK);

    put_desc(qts, RAM + 0x20, 4, 3, true, RAM + 0x200, 0);
    qtest_writel(qts, RAM + 8, RAM + 0x20);
    wr(qts, DMA_OUT_LINK, LINK_RESTART);
    serial_read(qts, sock, got, 3);
    g_assert_cmpmem(got, 3, "def", 3);
    wait_int(qts, INT_OUT_TOTAL_EOF);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * Bad outlink descriptors: one the CPU owns, with CHECK_OWNER, and one
 * outside the RAM the DMA can reach.
 */
static void test_out_dscr_err(void)
{
    int sock;
    QTestState *qts = start(&sock);

    put_desc(qts, RAM, 4, 3, true, RAM + 0x100, 0);
    qtest_writel(qts, RAM, qtest_readl(qts, RAM) & ~DESC_OWNER);
    wr(qts, CONF0, CONF0_UART0_CE);
    wr(qts, CONF1, CONF1_CHECK_OWNER);
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_OUT_DSCR_ERR);
    g_assert_true(rd(qts, DMA_OUT_LINK) & LINK_PARK);
    serial_quiet(qts, sock);

    /* 0x3ff40000 is UART0, not RAM */
    wr(qts, INT_CLR, 0x1ffff);
    wr(qts, DMA_OUT_LINK, LINK_START | 0x40000);
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_OUT_DSCR_ERR);

    /* A buffer outside DMA RAM */
    wr(qts, INT_CLR, 0x1ffff);
    put_desc(qts, RAM, 4, 3, true, 0x40080000, 0);
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));
    g_assert_cmphex(rd(qts, INT_RAW), ==, INT_OUT_DSCR_ERR);
    serial_quiet(qts, sock);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * Receiving a separated frame into an inlink: the separators go, the
 * escapes are decoded, and the descriptor comes back with the length and
 * EOF. A frame larger than a descriptor spills into the next.
 */
static void test_in_slip(void)
{
    int sock;
    QTestState *qts = start(&sock);
    static const uint8_t frame1[] = {
        0xc0, 0xc0, 'h', 'i', 0xdb, 0xdc, '!', 0xdb, 0xdd, 0xc0
    };
    static const uint8_t frame2[] = {
        'x', 0xc0, '1', '2', '3', '4', '5', '6', 0xc0
    };
    uint8_t buf[8];
    uint32_t dw0;

    put_desc(qts, RAM, 8, 0, false, RAM + 0x100, RAM + 0x20);
    put_desc(qts, RAM + 0x20, 4, 0, false, RAM + 0x200, RAM + 0x40);
    put_desc(qts, RAM + 0x40, 4, 0, false, RAM + 0x300, 0);
    wr(qts, CONF0, CONF0_UART0_CE | CONF0_SEPER_EN);
    /* Raise the in FIFO watermark above what a byte in passing reaches. */
    wr(qts, CONF1, 16 << 9);
    wr(qts, DMA_IN_LINK, LINK_START | (RAM & 0xfffff));
    g_assert_false(rd(qts, DMA_IN_LINK) & LINK_PARK);

    send_all(sock, frame1, sizeof(frame1));
    wait_int(qts, INT_IN_SUC_EOF);
    g_assert_cmphex(rd(qts, INT_RAW), ==,
                    INT_TX_START | INT_IN_DONE | INT_IN_SUC_EOF);
    dw0 = qtest_readl(qts, RAM);
    g_assert_cmphex(dw0, ==, DESC_EOF | (5 << 12) | 8);
    qtest_memread(qts, RAM + 0x100, buf, 5);
    g_assert_cmpmem(buf, 5, "hi\xc0!\xdb", 5);
    g_assert_cmphex(rd(qts, IN_SUC_EOF_DES_ADDR), ==, RAM);
    g_assert_cmphex(rd(qts, IN_DSCR), ==, RAM);

    /* 'x' before the opening separator is dropped. */
    wr(qts, INT_CLR, 0x1ffff);
    send_all(sock, frame2, sizeof(frame2));
    wait_int(qts, INT_IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, RAM + 0x20), ==, (4 << 12) | 4);
    g_assert_cmphex(qtest_readl(qts, RAM + 0x40), ==,
                    DESC_EOF | (2 << 12) | 4);
    qtest_memread(qts, RAM + 0x200, buf, 4);
    g_assert_cmpmem(buf, 4, "1234", 4);
    qtest_memread(qts, RAM + 0x300, buf, 2);
    g_assert_cmpmem(buf, 2, "56", 2);
    g_assert_cmphex(rd(qts, IN_SUC_EOF_DES_ADDR), ==, RAM + 0x40);
    g_assert_cmphex(rd(qts, IN_DSCR), ==, RAM + 0x40);
    g_assert_cmphex(rd(qts, IN_DSCR_BF0), ==, RAM + 0x20);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * With no inlink descriptor left, received data waits in the DMA FIFO:
 * IN_DSCR_EMPTY, the FIFO watermark, then TX_HUNG after the timeout.
 * Software can pop the FIFO, and RESTART mounts descriptors linked on.
 */
static void test_in_empty(void)
{
    int sock;
    QTestState *qts = start(&sock);
    uint8_t buf[4];

    put_desc(qts, RAM, 4, 0, false, RAM + 0x100, 0);
    wr(qts, CONF0, CONF0_UART0_CE);
    /* TX_HUNG after 16 ticks of 8000 APB_CLK (40 MHz) cycles: 3.2 ms */
    wr(qts, HUNG_CONF, (1u << 11) | 16);
    wr(qts, DMA_IN_LINK, LINK_START | (RAM & 0xfffff));
    send_all(sock, "abcdefg", 7);
    wait_int(qts, INT_IN_DSCR_EMPTY | INT_DMA_INFIFO_FULL_WM);
    g_assert_cmphex(rd(qts, DMA_IN_STATUS), ==, 0);
    qtest_memread(qts, RAM + 0x100, buf, 4);
    g_assert_cmpmem(buf, 4, "abcd", 4);
    /*
     * The stall began at most 100 us before wait_int saw it. The terminal
     * sends at UART0's reset baud rate, 57600 with APB_CLK at 40 MHz, so
     * "efg" follow within 530 us, each byte into the DMA FIFO restarting
     * the timeout.
     */
    g_assert_false(rd(qts, INT_RAW) & INT_TX_HUNG);
    qtest_clock_step(qts, 3000 * US);
    g_assert_false(rd(qts, INT_RAW) & INT_TX_HUNG);
    qtest_clock_step(qts, 800 * US);
    g_assert_true(rd(qts, INT_RAW) & INT_TX_HUNG);

    wr(qts, DMA_IN_POP, 1u << 16);
    g_assert_cmphex(rd(qts, DMA_IN_POP), ==, 'e');

    put_desc(qts, RAM + 0x20, 4, 0, false, RAM + 0x200, 0);
    qtest_writel(qts, RAM + 8, RAM + 0x20);
    wr(qts, DMA_IN_LINK, LINK_RESTART);
    g_assert_cmphex(rd(qts, DMA_IN_STATUS), ==, 2);
    qtest_memread(qts, RAM + 0x200, buf, 2);
    g_assert_cmpmem(buf, 2, "fg", 2);
    g_assert_cmphex(qtest_readl(qts, RAM), ==, (4 << 12) | 4);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * A UART the UHCI does not serve keeps its data in its own RX FIFO.
 */
static void test_uart_unattached(void)
{
    int sock;
    QTestState *qts = start(&sock);

    put_desc(qts, RAM, 8, 0, false, RAM + 0x100, 0);
    wr(qts, CONF0, 0);
    wr(qts, DMA_IN_LINK, LINK_START | (RAM & 0xfffff));
    send_all(sock, "xyz", 3);
    for (int i = 0; i < 2000 && (qtest_readl(qts, UART0_STATUS) & 0xff) < 3;
         i++) {
        g_usleep(1000);
        qtest_clock_step(qts, 100 * US);
    }
    g_assert_cmphex(qtest_readl(qts, UART0_STATUS) & 0xff, ==, 3);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);

    /* Attaching it lets the inlink take what is waiting. */
    wr(qts, CONF0, CONF0_UART0_CE);
    g_assert_cmphex(qtest_readl(qts, UART0_STATUS) & 0xff, ==, 0);
    g_assert_cmphex(qtest_readl(qts, RAM + 0x100) & 0xffffff, ==, 0x7a7978);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * Quick send: a single short packet goes out as a frame from Qn's words;
 * a byte pushed through DMA_OUT_PUSH goes out as it is.
 */
static void test_quick_and_push(void)
{
    int sock;
    QTestState *qts = start(&sock);
    static const uint8_t want[] = {
        0xc0, 1, 2, 3, 0xdb, 0xdc, 5, 6, 7, 8, 0xc0
    };
    uint8_t got[sizeof(want)];

    wr(qts, CONF0, CONF0_UART0_CE | CONF0_SEPER_EN);
    wr(qts, Q0_WORD0, 0xc0030201);
    wr(qts, Q0_WORD1, 0x08070605);
    wr(qts, QUICK_SENT, 1u << 3);
    serial_read(qts, sock, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), want, sizeof(want));
    wait_int(qts, INT_SEND_S_REG_Q | INT_RX_START);
    g_assert_cmphex(rd(qts, QUICK_SENT), ==, 0);
    serial_quiet(qts, sock);

    wr(qts, DMA_OUT_PUSH, (1u << 16) | 'Z');
    serial_read(qts, sock, got, 1);
    g_assert_cmphex(got[0], ==, 'Z');
    g_assert_cmphex(rd(qts, DMA_OUT_STATUS), ==, 2);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.uhci/test]
 * STOP parks the outlink part way; RESTART carries on from there. OUT_RST
 * abandons the transfer.
 */
static void test_out_stop(void)
{
    int sock;
    QTestState *qts = start(&sock);
    static char data[200];
    uint8_t got[200];

    for (int i = 0; i < sizeof(data); i++) {
        data[i] = 'a' + i % 26;
    }
    qtest_memwrite(qts, RAM + 0x100, data, sizeof(data));
    put_desc(qts, RAM, 200, 200, true, RAM + 0x100, 0);
    wr(qts, CONF0, CONF0_UART0_CE);
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));
    wr(qts, DMA_OUT_LINK, LINK_STOP);
    g_assert_true(rd(qts, DMA_OUT_LINK) & LINK_PARK);
    /* The UART FIFO and the DMA FIFO hold what was read so far. */
    qtest_clock_step(qts, 100 * MS);
    g_assert_false(rd(qts, INT_RAW) & INT_OUT_DONE);
    wr(qts, DMA_OUT_LINK, LINK_RESTART);
    serial_read(qts, sock, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), data, sizeof(data));
    wait_int(qts, INT_OUT_TOTAL_EOF);

    wr(qts, INT_CLR, 0x1ffff);
    wr(qts, DMA_OUT_LINK, LINK_START | (RAM & 0xfffff));
    wr(qts, CONF0, CONF0_UART0_CE | CONF0_OUT_RST);
    wr(qts, CONF0, CONF0_UART0_CE);
    g_assert_cmphex(rd(qts, DMA_OUT_STATUS), ==, 2);
    qtest_clock_step(qts, 100 * MS);
    g_assert_false(rd(qts, INT_RAW) & INT_OUT_TOTAL_EOF);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/uhci/reset-values", test_reset_values);
    qtest_add_func("/esp32/uhci/gated", test_gated);
    qtest_add_func("/esp32/uhci/out-slip", test_out_slip);
    qtest_add_func("/esp32/uhci/out-chain", test_out_chain);
    qtest_add_func("/esp32/uhci/out-restart", test_out_restart);
    qtest_add_func("/esp32/uhci/out-dscr-err", test_out_dscr_err);
    qtest_add_func("/esp32/uhci/out-stop", test_out_stop);
    qtest_add_func("/esp32/uhci/in-slip", test_in_slip);
    qtest_add_func("/esp32/uhci/in-empty", test_in_empty);
    qtest_add_func("/esp32/uhci/uart-unattached", test_uart_unattached);
    qtest_add_func("/esp32/uhci/quick-and-push", test_quick_and_push);

    return g_test_run();
}
