/*
 * QTest testcase for the ESP32-P4 BitScrambler
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The programs are ESP-IDF's BitScrambler test programs and examples, and
 * a few of the test's own, assembled with ESP-IDF's tools/bsasm.py; the
 * loopback runs follow ESP-IDF's bitscrambler_loopback_run(), and their
 * expected results are those ESP-IDF's tests expect of the hardware. The
 * TX and RX paths to a peripheral use the GDMA-AXI controller's qtest
 * hook to play GPSPI2.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define BS                  0x500a3000
#define INST_CFG0(c)        (BS + 0x00 + 8 * (c))
#define INST_CFG1(c)        (BS + 0x04 + 8 * (c))
#define LUT_CFG0(c)         (BS + 0x10 + 8 * (c))
#define LUT_CFG1(c)         (BS + 0x14 + 8 * (c))
#define TAILING_BITS(c)     (BS + 0x20 + 4 * (c))
#define CTRL(c)             (BS + 0x28 + 4 * (c))
#define STATE(c)            (BS + 0x30 + 4 * (c))
#define SYS                 (BS + 0xf8)
#define VERSION             (BS + 0xfc)
#define TX                  0
#define RX                  1

#define ENA                 (1u << 0)
#define PAUSE               (1u << 1)
#define HALT                (1u << 2)
#define EOF_MODE            (1u << 3)
#define COND_MODE           (1u << 4)
#define FETCH_MODE          (1u << 5)
#define HALT_MODE           (1u << 6)
#define RD_DUMMY            (1u << 7)
#define FIFO_RST            (1u << 8)
#define IN_IDLE             (1u << 0)
#define IN_RUN              (1u << 1)
#define IN_WAIT             (1u << 2)
#define IN_PAUSE            (1u << 3)
#define STATE_FIFO          (1u << 4)
#define EOF_GET_CNT(v)      (((v) >> 16) & 0x3fff)
#define EOF_OVERLOAD        (1u << 30)
#define EOF_TRACE_CLR       (1u << 31)
#define LOOP_MODE           (1u << 0)

#define HP_SYS_PERI_SEL     0x500e5140
#define SEL_GPSPI2          1
#define SEL_NONE            15
#define CLKRST              0x500e6000
#define HP_RST_EN2          (CLKRST + 0xc8)
#define RST_EN_BS           (1u << 11)
#define RST_EN_BS_RX        (1u << 12)
#define RST_EN_BS_TX        (1u << 13)

/* GDMA-AXI channel 0; GPSPI2 is its peripheral 1. */
#define AXI                 0x5008a000
#define AXI_IN              (AXI + 0x00)
#define AXI_OUT             (AXI + 0x138)
#define X_RAW               0x00
#define X_CLR               0x0c
#define X_CONF0             0x10
#define X_LINK1             0x20
#define X_LINK2             0x24
#define X_PERI_SEL          0x44
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define IN_SUC_EOF          (1u << 1)
#define OUT_TOTAL_EOF       (1u << 3)
#define PERI_SPI2           1
#define DW0(size, len)      ((size) | (uint32_t)(len) << 12)
#define DW0_LEN(d)          (((d) >> 12) & 0xfff)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)

#define MEM                 0x4ff40000
#define TX_DESC             MEM
#define RX_DESC             (MEM + 0x100)
#define SRC                 (MEM + 0x1000)
#define DST                 (MEM + 0x8000)

typedef struct BsProg {
    const char *name;
    const uint32_t (*inst)[9];
    int ninst;
    const uint32_t *lut;
    int nlut;
    unsigned lut_mode;
    bool prefetch;
    unsigned trailing_bits;
    bool eof_upstream;
} BsProg;

/*
 * trivial.bsasm:
 *
 *   cfg eof_on upstream
 *   cfg trailing_bytes 8
 *   cfg prefetch true
 *   loop:
 *       set 0..31 0..31,
 *       write 32,
 *       read 32,
 *       jmp loop
 */
static const uint32_t trivial_inst[][9] = {
    { 0x40608080, 0x880e1828, 0x68c16284, 0x88901e38, 0x58a94264,
      0x668c982e, 0x3e78e9c3, 0x3c01005f, 0x00000000 },
};
static const BsProg trivial = {
    "trivial", trivial_inst, 1, NULL, 0, 2, true, 64, true,
};

/*
 * eof_upstream.bsasm:
 *
 *   cfg prefetch false
 *   cfg eof_on upstream
 *   cfg trailing_bytes 1
 *   loop:
 *       read 8
 *       set 0..15 L,
 *       set 16..31 H,
 *       write 32,
 *       jmp loop
 */
static const uint32_t eof_upstream_inst[][9] = {
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x04c00000, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xefdfbd7a, 0x7efdfbf7,
      0xf7efdfbf, 0xbf7efdfb, 0x3001005f, 0x00000000 },
};
static const BsProg eof_upstream = {
    "eof_upstream", eof_upstream_inst, 2, NULL, 0, 2, false, 8, true,
};

/*
 * eof_downstream.bsasm:
 *
 *   cfg prefetch false
 *   cfg eof_on downstream
 *   cfg trailing_bytes 4
 *   loop:
 *       read 8
 *       set 0..15 L,
 *       set 16..31 H,
 *       write 32,
 *       jmp loop
 */
static const uint32_t eof_downstream_inst[][9] = {
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x04c00000, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xefdfbd7a, 0x7efdfbf7,
      0xf7efdfbf, 0xbf7efdfb, 0x3001005f, 0x00000000 },
};
static const BsProg eof_downstream = {
    "eof_downstream", eof_downstream_inst, 2, NULL, 0, 2, false, 32, false,
};

/*
 * lut32.bsasm:
 *
 *   cfg prefetch false
 *   cfg eof_on downstream
 *   cfg trailing_bytes 4
 *   cfg lut_width_bits 32
 *   lut 0xF0011111
 *   lut 0xF0022222
 *   lut 0xF0033333
 *   lut 0xF0004444
 *   set 16..17 L
 *   loop:
 *       read 8,
 *       set 31..0 L31..L0,
 *       write 32,
 *       jmp loop
 */
static const uint32_t lut32_inst[][9] = {
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x00c00000, 0x00000000 },
    { 0x446890a0, 0xa84e992a, 0x6ac56a94, 0x98b05eb9, 0xd9ab466c,
      0x6e9cb86e, 0x7ef9ebc7, 0x3421005f, 0x00000001 },
};
static const uint32_t lut32_lut[] = {
    0xf0011111, 0xf0022222, 0xf0033333, 0xf0004444,
};
static const BsProg lut32 = {
    "lut32", lut32_inst, 2, lut32_lut, 4, 2, false, 32, false,
};

/*
 * loop.bsasm:
 *
 *   cfg prefetch false
 *   cfg eof_on downstream
 *   cfg trailing_bytes 32
 *   cfg lut_width_bits 32
 *   lut 0x80030009
 *   lut 0x80090003
 *   lut 0x00F000F0
 *   set 17 H, set 16 L
 *   set 31..0 L31..L0, write 32
 *   byteloop:
 *       read 8, set 16..23 56..63, ldctibl
 *   bitloop:
 *       set 16 B7
 *       set 31..0 L31..L0, write 32
 *       set 16 L, set 17..23 B0..B6, ldctibl
 *       loopa 7 1 bitloop
 *       jmp byteloop
 */
static const uint32_t loop_inst[][9] = {
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xafdebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x00c00000, 0x00000000 },
    { 0x446890a0, 0xa84e992a, 0x6ac56a94, 0x98b05eb9, 0xd9ab466c,
      0x6e9cb86e, 0x7ef9ebc7, 0x30c00000, 0x00000001 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0x9cb8bd7a, 0xf9ebc76e,
      0xd7af5e7e, 0xbd7af5eb, 0x05440000, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf37bd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x80c00000, 0x00000000 },
    { 0x446890a0, 0xa84e992a, 0x6ac56a94, 0x98b05eb9, 0xd9ab466c,
      0x6e9cb86e, 0x7ef9ebc7, 0x30c00000, 0x00000001 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0x585ebd7a, 0xd5a3364c,
      0xd7af5e6c, 0xbd7af5eb, 0x81440000, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x026000e1, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x0041005f, 0x00000000 },
};
static const uint32_t loop_lut[] = {
    0x80030009, 0x80090003, 0x00f000f0,
};
static const BsProg loop = {
    "loop", loop_inst, 8, loop_lut, 3, 2, false, 256, false,
};

/*
 * example.bsasm:
 *
 *   cfg eof_on downstream
 *   cfg trailing_bytes 8
 *   cfg prefetch true
 *   loop:
 *       set 0 0,
 *       set 1 8,
 *       set 2 16,
 *       set 3 24,
 *       set 4 32,
 *       set 5 40,
 *       set 6 48,
 *       set 7 56,
 *       set 8 1,
 *       set 9 9,
 *       set 10 17,
 *       set 11 25,
 *       set 12 33,
 *       set 13 41,
 *       set 14 49,
 *       set 15 57,
 *       set 16 2,
 *       set 17 10,
 *       set 18 18,
 *       set 19 26,
 *       set 20 34,
 *       set 21 42,
 *       set 22 50,
 *       set 23 58,
 *       set 24 3,
 *       set 25 11,
 *       set 26 19,
 *       set 27 27,
 *       set 28 35,
 *       set 29 43,
 *       set 30 51,
 *       set 31 59,
 *       write 32
 *       set 0 4,
 *       set 1 12,
 *       set 2 20,
 *       set 3 28,
 *       set 4 36,
 *       set 5 44,
 *       set 6 52,
 *       set 7 60,
 *       set 8 5,
 *       set 9 13,
 *       set 10 21,
 *       set 11 29,
 *       set 12 37,
 *       set 13 45,
 *       set 14 53,
 *       set 15 61,
 *       set 16 6,
 *       set 17 14,
 *       set 18 22,
 *       set 19 30,
 *       set 20 38,
 *       set 21 46,
 *       set 22 54,
 *       set 23 62,
 *       set 24 7,
 *       set 25 15,
 *       set 26 23,
 *       set 27 31,
 *       set 28 39,
 *       set 29 47,
 *       set 30 55,
 *       set 31 63,
 *       write 32,
 *       read 32
 *       read 32,
 *       jmp loop
 */
static const uint32_t example_inst[][9] = {
    { 0x03040400, 0x8170c142, 0x4a132444, 0x850272c5, 0xc9522344,
      0x64c58374, 0x76cd5a33, 0x30c00000, 0x00000000 },
    { 0x43850604, 0x8578d162, 0x6a53a546, 0x87067ad5, 0xd97263c5,
      0xe5c7877c, 0x7edd7a73, 0x3cc00000, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x0c01005f, 0x00000000 },
};
static const BsProg example = {
    "example", example_inst, 3, NULL, 0, 2, true, 64, false,
};

/*
 * rel.bsasm:
 *
 *   cfg prefetch false
 *   cfg eof_on downstream
 *   cfg trailing_bytes 8
 *   byte:
 *       read 8
 *   bit:
 *       set 0 56+a, set 1..7 L, write 8, loopa 7 1 bit
 *       jmp byte
 */
static const uint32_t rel_inst[][9] = {
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x04c00000, 0x00000000 },
    { 0xebd7af38, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x522000e1, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x0001005f, 0x00000000 },
};
static const BsProg rel = {
    "rel", rel_inst, 3, NULL, 0, 2, false, 64, false,
};

/*
 * filter.bsasm:
 *
 *   cfg prefetch false
 *   cfg eof_on upstream
 *   cfg trailing_bytes 1
 *   top:
 *       read 8
 *       ifn 63 top
 *       set 0..7 56..63, write 8, jmp top
 */
static const uint32_t filter_inst[][9] = {
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x04c00000, 0x00000000 },
    { 0xebd7af5e, 0x5ebd7af5, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x0002003f, 0x00000000 },
    { 0xc76e9cb8, 0x5e7ef9eb, 0xf5ebd7af, 0xaf5ebd7a, 0x7af5ebd7,
      0xd7af5ebd, 0xbd7af5eb, 0x1001005f, 0x00000000 },
};
static const BsProg filter = {
    "filter", filter_inst, 3, NULL, 0, 2, false, 8, true,
};

/*
 * bitrev.bsasm:
 *
 *   cfg prefetch true
 *   cfg eof_on upstream
 *   cfg trailing_bytes 8
 *   loop:
 *       set 0..7 7..0,
 *       set 8..15 15..8,
 *       set 16..23 23..16,
 *       set 24..31 31..24,
 *       write 32,
 *       read 32,
 *       jmp loop
 */
static const uint32_t bitrev_inst[][9] = {
    { 0x30814307, 0x0f000410, 0x50b18347, 0x4b171024, 0x44913285,
      0x874f1f20, 0x3064d1b3, 0x3c01005f, 0x00000000 },
};
static const BsProg bitrev = {
    "bitrev", bitrev_inst, 1, NULL, 0, 2, true, 64, true,
};

/*
 * bswap32.bsasm:
 *
 *   cfg prefetch true
 *   cfg eof_on upstream
 *   cfg trailing_bytes 8
 *   loop:
 *       set 0..7 24..31,
 *       set 8..15 16..23,
 *       set 16..23 8..15,
 *       set 24..31 0..7,
 *       write 32,
 *       read 32,
 *       jmp loop
 */
static const uint32_t bswap32_inst[][9] = {
    { 0xc3668c98, 0x903e78e9, 0xa9426488, 0x84882e58, 0x3868c162,
      0x6080801e, 0x0e182840, 0x3c01005f, 0x00000000 },
};
static const BsProg bswap32 = {
    "bswap32", bswap32_inst, 1, NULL, 0, 2, true, 64, true,
};

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

/* Descriptors of 2 KiB at most, 16 bytes apart, from @at. */
static void chain(QTestState *qts, uint32_t at, uint32_t buf, uint32_t len,
                  bool tx)
{
    for (uint32_t off = 0; off < len; off += 2048, at += 16) {
        uint32_t n = MIN(2048, len - off);
        bool last = off + n == len;

        desc(qts, at, OWNER | DW0(n, tx ? n : 0) | (tx && last ? SUC_EOF : 0),
             buf + off, last ? 0 : at + 16);
    }
}

static void dma_reset(QTestState *qts)
{
    qtest_writel(qts, AXI_IN + X_CONF0, 1);
    qtest_writel(qts, AXI_IN + X_CONF0, 0);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1u << 3);
    qtest_writel(qts, AXI_IN + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_OUT + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_IN + X_PERI_SEL, PERI_SPI2);
    qtest_writel(qts, AXI_OUT + X_PERI_SEL, PERI_SPI2);
}

static void dma_start_tx(QTestState *qts, uint32_t len)
{
    chain(qts, TX_DESC, SRC, len, true);
    qtest_writel(qts, AXI_OUT + X_LINK2, TX_DESC);
    qtest_writel(qts, AXI_OUT + X_LINK1, OUT_LINK_START);
}

static void dma_start_rx(QTestState *qts, uint32_t size)
{
    chain(qts, RX_DESC, DST, size, false);
    qtest_writel(qts, AXI_IN + X_LINK2, RX_DESC);
    qtest_writel(qts, AXI_IN + X_LINK1, IN_LINK_START);
}

/* The bytes the RX descriptors received up to the frame's end, or -1. */
static int rx_received(QTestState *qts)
{
    int n = 0;

    for (uint32_t at = RX_DESC;; at += 16) {
        uint32_t dw0 = qtest_readl(qts, at);

        n += DW0_LEN(dw0);
        if (dw0 & SUC_EOF) {
            return n;
        }
        if (!qtest_readl(qts, at + 8)) {
            return -1;
        }
    }
}

/* Load @p into core @c, as ESP-IDF's bitscrambler_load_program() does. */
static void load(QTestState *qts, int c, const BsProg *p, bool loop)
{
    uint32_t ctrl = qtest_readl(qts, CTRL(c)) & ENA;

    qtest_writel(qts, CTRL(c), ctrl | HALT);
    for (int i = 0; i < p->ninst; i++) {
        for (int w = 0; w < 9; w++) {
            qtest_writel(qts, INST_CFG0(c), i | w << 3);
            qtest_writel(qts, INST_CFG1(c), p->inst[i][w]);
        }
    }
    for (int w = 0; w < p->nlut; w++) {
        qtest_writel(qts, LUT_CFG0(c), w | 2u << 11);
        qtest_writel(qts, LUT_CFG1(c), p->lut[w]);
    }
    qtest_writel(qts, LUT_CFG0(c), p->lut_mode << 11);
    qtest_writel(qts, TAILING_BITS(c), p->trailing_bits);
    qtest_writel(qts, CTRL(c), ctrl | HALT | RD_DUMMY | HALT_MODE |
                 (p->prefetch ? 0 : FETCH_MODE) |
                 (p->eof_upstream ? 0 : EOF_MODE));
    qtest_writel(qts, SYS, loop ? LOOP_MODE : 0);
}

/* Halt core @c, as bitscrambler_reset() does, and empty its FIFOs. */
static void bs_reset(QTestState *qts, int c)
{
    qtest_writel(qts, CTRL(c), (qtest_readl(qts, CTRL(c)) & ~PAUSE) | HALT);
    g_assert_cmphex(qtest_readl(qts, STATE(c)) & 0xf, ==, IN_IDLE);
    qtest_writel(qts, CTRL(c), qtest_readl(qts, CTRL(c)) | FIFO_RST);
    qtest_writel(qts, STATE(c), EOF_TRACE_CLR);
}

static void bs_start(QTestState *qts, int c)
{
    qtest_writel(qts, CTRL(c), qtest_readl(qts, CTRL(c)) & ~(HALT | PAUSE));
}

/*
 * Run @p in loopback on @in, into an RX buffer of @size bytes, as
 * bitscrambler_loopback_run() does: the bytes received, or -1 if the
 * stream did not end.
 */
static int loopback(QTestState *qts, const BsProg *p, const uint8_t *in,
                    uint32_t len, uint8_t *out, uint32_t size)
{
    g_autofree uint8_t *zero = g_malloc0(size);
    int n;

    qtest_writel(qts, HP_SYS_PERI_SEL, SEL_GPSPI2 << 4 | SEL_GPSPI2);
    qtest_writel(qts, CTRL(TX), ENA | HALT);
    load(qts, TX, p, true);
    dma_reset(qts);
    bs_reset(qts, TX);
    qtest_memwrite(qts, SRC, in, len);
    qtest_memwrite(qts, DST, zero, size);
    dma_start_rx(qts, size);
    dma_start_tx(qts, len);
    bs_start(qts, TX);
    qtest_clock_step(qts, 5 * 1000 * 1000);
    n = rx_received(qts);
    if (n > 0) {
        qtest_memread(qts, DST, out, n);
    }
    return n;
}

static void words_eq(const uint8_t *b, int n, const uint32_t *want)
{
    for (int i = 0; i < n / 4; i++) {
        uint32_t w = ldl_le_p(b + 4 * i);

        g_assert_cmphex(w, ==, want[i]);
    }
}

/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    for (int c = 0; c < 2; c++) {
        g_assert_cmphex(qtest_readl(qts, INST_CFG0(c)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, LUT_CFG0(c)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, TAILING_BITS(c)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, CTRL(c)), ==, HALT);
        g_assert_cmphex(qtest_readl(qts, STATE(c)), ==, IN_IDLE);
    }
    g_assert_cmphex(qtest_readl(qts, SYS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VERSION), ==, 0x2303240);
    g_assert_cmphex(qtest_readl(qts, HP_SYS_PERI_SEL), ==, 0xff);

    /* Writable bits; FIFO_RST and EOF_TRACE_CLR read 0. */
    qtest_writel(qts, INST_CFG0(TX), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INST_CFG0(TX)), ==, 0x7f);
    qtest_writel(qts, LUT_CFG0(RX), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LUT_CFG0(RX)), ==, 0x1fff);
    qtest_writel(qts, TAILING_BITS(RX), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TAILING_BITS(RX)), ==, 0xffff);
    qtest_writel(qts, CTRL(RX), 0xffffffff & ~ENA);
    g_assert_cmphex(qtest_readl(qts, CTRL(RX)), ==, 0xfe);
    qtest_writel(qts, STATE(RX), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, STATE(RX)), ==, IN_IDLE);
    qtest_writel(qts, SYS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SYS), ==, 0x80000001);
    qtest_writel(qts, VERSION, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VERSION), ==, 0x0fffffff);
    qtest_writel(qts, HP_SYS_PERI_SEL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_SYS_PERI_SEL), ==, 0xff);

    /* RST_EN_BITSCRAMBLER_RX resets the RX core's registers only. */
    qtest_writel(qts, HP_RST_EN2, RST_EN_BS_RX);
    qtest_writel(qts, HP_RST_EN2, 0);
    g_assert_cmphex(qtest_readl(qts, CTRL(RX)), ==, HALT);
    g_assert_cmphex(qtest_readl(qts, LUT_CFG0(RX)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INST_CFG0(TX)), ==, 0x7f);
    /* RST_EN_BITSCRAMBLER, the whole block. */
    qtest_writel(qts, HP_RST_EN2, RST_EN_BS);
    qtest_writel(qts, HP_RST_EN2, 0);
    g_assert_cmphex(qtest_readl(qts, INST_CFG0(TX)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SYS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VERSION), ==, 0x2303240);
    qtest_quit(qts);
}

/*
 * The instruction memory, nine words an instruction with the 257th bit
 * alone in the ninth; the LUT, addressed in units of LUT_MODE's width
 * over the same 2048 bytes.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_memories(void)
{
    QTestState *qts = start();

    for (int i = 0; i < 8; i++) {
        for (int w = 0; w < 9; w++) {
            qtest_writel(qts, INST_CFG0(RX), i | w << 3);
            qtest_writel(qts, INST_CFG1(RX), 0x01010101u * (i * 9 + w + 1));
        }
    }
    qtest_writel(qts, INST_CFG0(RX), 5 | 3 << 3);
    g_assert_cmphex(qtest_readl(qts, INST_CFG1(RX)), ==, 0x01010101u * 49);
    qtest_writel(qts, INST_CFG0(RX), 5 | 8 << 3);
    g_assert_cmphex(qtest_readl(qts, INST_CFG1(RX)), ==, 0);
    qtest_writel(qts, INST_CFG0(RX), 6 | 8 << 3);
    g_assert_cmphex(qtest_readl(qts, INST_CFG1(RX)), ==, 1);
    /* No tenth word. */
    qtest_writel(qts, INST_CFG0(RX), 9 << 3);
    qtest_writel(qts, INST_CFG1(RX), 0x1234);
    g_assert_cmphex(qtest_readl(qts, INST_CFG1(RX)), ==, 0);
    /* The TX core's memory is its own. */
    qtest_writel(qts, INST_CFG0(TX), 5 | 3 << 3);
    g_assert_cmphex(qtest_readl(qts, INST_CFG1(TX)), ==, 0);

    qtest_writel(qts, LUT_CFG0(TX), 2u << 11 | 1);
    qtest_writel(qts, LUT_CFG1(TX), 0x44332211);
    qtest_writel(qts, LUT_CFG0(TX), 2u << 11 | 2);
    qtest_writel(qts, LUT_CFG1(TX), 0x88776655);
    qtest_writel(qts, LUT_CFG0(TX), 0u << 11 | 6);
    g_assert_cmphex(qtest_readl(qts, LUT_CFG1(TX)), ==, 0x33);
    qtest_writel(qts, LUT_CFG0(TX), 1u << 11 | 4);
    g_assert_cmphex(qtest_readl(qts, LUT_CFG1(TX)), ==, 0x6655);
    qtest_writel(qts, LUT_CFG1(TX), 0xabcd);
    qtest_writel(qts, LUT_CFG0(TX), 2u << 11 | 2);
    g_assert_cmphex(qtest_readl(qts, LUT_CFG1(TX)), ==, 0x8877abcd);
    qtest_writel(qts, LUT_CFG0(TX), 0u << 11 | 2047);
    qtest_writel(qts, LUT_CFG1(TX), 0x5a);
    qtest_writel(qts, LUT_CFG0(TX), 2u << 11 | 511);
    g_assert_cmphex(qtest_readl(qts, LUT_CFG1(TX)), ==, 0x5a000000);
    qtest_quit(qts);
}

/*
 * ESP-IDF's loopback tests, in loopback on GPSPI2's GDMA-AXI channels:
 * the bytes and lengths ESP-IDF's test app expects of the hardware.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_loopback_idf(void)
{
    QTestState *qts = start();
    g_autofree uint8_t *in = g_malloc(0x4000);
    g_autofree uint8_t *out = g_malloc(0x4000);
    static const uint8_t ws[] = { 0x00, 0xff, 0x55 };
    uint32_t want[32];
    int n;

    for (int i = 0; i < 0x4000; i++) {
        in[i] = i * 151 + (i >> 7) + 3;
    }
    n = loopback(qts, &trivial, in, 0x4000, out, 0x4000);
    g_assert_cmpint(n, ==, 0x4000);
    g_assert_cmpmem(in, 0x4000, out, 0x4000);
    g_assert_cmphex(qtest_readl(qts, AXI_IN + X_RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT + X_RAW) & OUT_TOTAL_EOF, ==,
                    OUT_TOTAL_EOF);
    /* The stream ended after the 8 bytes read past the input's end. */
    g_assert_cmpuint(EOF_GET_CNT(qtest_readl(qts, STATE(TX))), ==, 8);
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0xf, ==, IN_IDLE);

    memset(in, 0, 32);
    for (int i = 0; i < 32; i++) {
        want[i] = 0xffff0000;
    }
    g_assert_cmpint(loopback(qts, &eof_upstream, in, 32, out, 128), ==, 128);
    words_eq(out, 128, want);
    g_assert_cmpuint(EOF_GET_CNT(qtest_readl(qts, STATE(TX))), ==, 1);
    g_assert_cmpint(loopback(qts, &eof_downstream, in, 32, out, 128), ==,
                    128);
    words_eq(out, 128, want);
    g_assert_cmpuint(EOF_GET_CNT(qtest_readl(qts, STATE(TX))), ==, 4);
    qtest_writel(qts, STATE(TX), EOF_TRACE_CLR);
    g_assert_cmpuint(EOF_GET_CNT(qtest_readl(qts, STATE(TX))), ==, 0);

    for (int i = 0; i < 32; i++) {
        static const uint32_t lut[4] = {
            0xf0011111, 0xf0022222, 0xf0033333, 0xf0004444,
        };

        want[i] = lut[i % 4];
    }
    g_assert_cmpint(loopback(qts, &lut32, in, 32, out, 128), ==, 128);
    words_eq(out, 128, want);

    want[0] = 0x00f000f0;
    for (int i = 1; i < 25; i++) {
        uint8_t byte = ws[(i - 1) / 8];
        bool bit = (byte >> (7 - (i - 1) % 8)) & 1;

        want[i] = bit ? 0x80090003 : 0x80030009;
    }
    memcpy(in, ws, 3);
    g_assert_cmpint(loopback(qts, &loop, in, 3, out, 3 * 64), ==,
                    3 * 32 + 4);
    words_eq(out, 100, want);
    qtest_quit(qts);
}

/*
 * ESP-IDF's example program, which transposes each 8x8 bit block, and the
 * test's own: bit reversal, byte swapping, counter-relative addressing in
 * a loop, and an IFN on an input bit.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_loopback_programs(void)
{
    QTestState *qts = start();
    uint8_t in[64], out[64], want[64];
    static const uint8_t spread_in[] = { 0x00, 0xff, 0xa5, 0x3c };
    static const uint8_t filter_in[] = { 0x80, 0x01, 0xff, 0x7f, 0xc0, 0x00,
                                         0x81 };
    static const uint8_t filter_out[] = { 0x80, 0xff, 0xc0, 0x81 };

    for (int i = 0; i < 64; i++) {
        in[i] = i * 77 + 5;
    }
    g_assert_cmpint(loopback(qts, &example, in, 64, out, 64), ==, 64);
    for (int blk = 0; blk < 8; blk++) {
        for (int ob = 0; ob < 8; ob++) {
            uint8_t v = 0;

            for (int ib = 0; ib < 8; ib++) {
                v |= ((in[8 * blk + ib] >> ob) & 1) << ib;
            }
            want[8 * blk + ob] = v;
        }
    }
    g_assert_cmpmem(want, 64, out, 64);

    g_assert_cmpint(loopback(qts, &bitrev, in, 64, out, 64), ==, 64);
    for (int i = 0; i < 64; i++) {
        uint8_t r = 0;

        for (int b = 0; b < 8; b++) {
            r |= ((in[i] >> b) & 1) << (7 - b);
        }
        g_assert_cmphex(out[i], ==, r);
    }

    g_assert_cmpint(loopback(qts, &bswap32, in, 64, out, 64), ==, 64);
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(out[i], ==, in[(i & ~3) + 3 - (i & 3)]);
    }

    g_assert_cmpint(loopback(qts, &rel, spread_in, 4, out, 64), ==, 32);
    for (int i = 0; i < 32; i++) {
        g_assert_cmphex(out[i], ==, (spread_in[i / 8] >> (i % 8)) & 1);
    }

    g_assert_cmpint(loopback(qts, &filter, filter_in, 7, out, 64), ==, 4);
    g_assert_cmpmem(filter_out, 4, out, 4);
    qtest_quit(qts);
}

/* Play GPSPI2 through GDMA-AXI's "periph" hook. */
static char *periph_cmd(QTestState *qts, const char *cmd)
{
    QDict *rsp;
    char *r;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': '/machine/axi-gdma', 'property': 'periph', "
                    "'value': %s}}", cmd);
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    rsp = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                    "{'path': '/machine/axi-gdma', 'property': 'periph'}}");
    g_assert(qdict_haskey(rsp, "return"));
    r = g_strdup(qdict_get_str(rsp, "return"));
    qobject_unref(rsp);
    return r;
}

static unsigned periph_pull(QTestState *qts, uint8_t *b, unsigned len,
                            bool *eof)
{
    g_autofree char *cmd = g_strdup_printf("pull %u %u", PERI_SPI2, len);
    g_autofree char *r = periph_cmd(qts, cmd);
    unsigned n, e;
    char *hex;

    g_assert_cmpint(sscanf(r, "%u %u", &n, &e), ==, 2);
    hex = strrchr(r, ' ') + 1;
    for (unsigned i = 0; i < n; i++) {
        b[i] = g_ascii_xdigit_value(hex[2 * i]) << 4 |
               g_ascii_xdigit_value(hex[2 * i + 1]);
    }
    *eof = e;
    return n;
}

static unsigned periph_push(QTestState *qts, const uint8_t *b, unsigned len,
                            unsigned flags)
{
    g_autoptr(GString) cmd = g_string_new(NULL);
    g_autofree char *r = NULL;

    g_string_printf(cmd, "push %u ", PERI_SPI2);
    for (unsigned i = 0; i < len; i++) {
        g_string_append_printf(cmd, "%02x", b[i]);
    }
    g_string_append_printf(cmd, " %u", flags);
    r = periph_cmd(qts, cmd->str);
    return atoi(r);
}

/*
 * The TX core between GDMA and GPSPI2: the peripheral takes the swapped
 * bytes, the frame's end with the last; nothing while the core is halted
 * or paused. With HALT_MODE clear a halt waits (IN_WAIT) for the output
 * to be taken. Detached, the peripheral takes the channel's bytes again.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_tx_path(void)
{
    QTestState *qts = start();
    uint8_t in[64], got[64];
    unsigned n = 0;
    bool eof = false;

    for (int i = 0; i < 64; i++) {
        in[i] = i + 1;
    }
    qtest_memwrite(qts, SRC, in, 64);
    qtest_writel(qts, HP_SYS_PERI_SEL, SEL_GPSPI2 << 4 | SEL_NONE);
    qtest_writel(qts, CTRL(TX), ENA | HALT);
    load(qts, TX, &bswap32, false);
    dma_reset(qts);
    dma_start_tx(qts, 64);
    qtest_clock_step(qts, 100000);
    g_assert_cmpuint(periph_pull(qts, got, 64, &eof), ==, 0);

    bs_start(qts, TX);
    qtest_writel(qts, CTRL(TX), qtest_readl(qts, CTRL(TX)) | PAUSE);
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0xf, ==, IN_PAUSE);
    qtest_clock_step(qts, 100000);
    g_assert_cmpuint(periph_pull(qts, got, 64, &eof), ==, 0);
    qtest_writel(qts, CTRL(TX), qtest_readl(qts, CTRL(TX)) & ~PAUSE);
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0xf, ==, IN_RUN);
    qtest_clock_step(qts, 100000);
    /* The output FIFO is full, the core waiting for the peripheral. */
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0x1f, ==,
                    IN_RUN | STATE_FIFO);
    while (!eof && n < 64) {
        n += periph_pull(qts, got + n, 64 - n, &eof);
        qtest_clock_step(qts, 10000);
    }
    g_assert_true(eof);
    g_assert_cmpuint(n, ==, 64);
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(got[i], ==, in[(i & ~3) + 3 - (i & 3)]);
    }
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0x1f, ==, IN_IDLE);

    /* Halting with write back: IN_WAIT until the output is taken. */
    qtest_writel(qts, CTRL(TX), (qtest_readl(qts, CTRL(TX)) & ~HALT_MODE) |
                 HALT);
    qtest_writel(qts, CTRL(TX), qtest_readl(qts, CTRL(TX)) | FIFO_RST);
    dma_start_tx(qts, 64);
    bs_start(qts, TX);
    qtest_clock_step(qts, 100000);
    qtest_writel(qts, CTRL(TX), qtest_readl(qts, CTRL(TX)) | HALT);
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0x1f, ==,
                    IN_WAIT | STATE_FIFO);
    n = periph_pull(qts, got, 64, &eof);
    g_assert_cmpuint(n, ==, 16);
    g_assert_cmphex(qtest_readl(qts, STATE(TX)) & 0x1f, ==, IN_IDLE);

    /* Detached: the channel's own bytes. */
    qtest_writel(qts, HP_SYS_PERI_SEL, 0xff);
    dma_reset(qts);
    dma_start_tx(qts, 64);
    qtest_clock_step(qts, 100000);
    n = periph_pull(qts, got, 64, &eof);
    g_assert_cmpuint(n, ==, 64);
    g_assert_true(eof);
    g_assert_cmpmem(in, 64, got, 64);
    qtest_quit(qts);
}

/*
 * The RX core between GPSPI2 and GDMA: what the peripheral gives, bit
 * reversed, reaches memory with the frame's end. While halted it takes
 * nothing; its full input FIFO reads in STATE.
 */
/* [spec:nuos:req:emu.esp32p4.parlio/test] */
static void test_rx_path(void)
{
    QTestState *qts = start();
    uint8_t in[40], out[40];
    unsigned n = 0;

    for (int i = 0; i < 40; i++) {
        in[i] = i * 13 + 1;
    }
    qtest_writel(qts, HP_SYS_PERI_SEL, SEL_NONE << 4 | SEL_GPSPI2);
    qtest_writel(qts, CTRL(RX), ENA | HALT);
    load(qts, RX, &bitrev, false);
    dma_reset(qts);
    dma_start_rx(qts, 64);
    g_assert_cmpuint(periph_push(qts, in, 4, 0), ==, 0);
    bs_start(qts, RX);

    /* With the channel stopped the core's input fills. */
    qtest_writel(qts, AXI_IN + X_CONF0, 1);
    n = periph_push(qts, in, 40, 0);
    qtest_clock_step(qts, 100000);
    n += periph_push(qts, in + n, 40 - n, 0);
    g_assert_cmpuint(n, <, 40);
    g_assert_cmphex(qtest_readl(qts, STATE(RX)) & STATE_FIFO, ==,
                    STATE_FIFO);
    qtest_writel(qts, AXI_IN + X_CONF0, 0);
    dma_start_rx(qts, 64);
    /* The frame's end goes with the push that takes the last byte. */
    while (n < 40) {
        qtest_clock_step(qts, 10000);
        n += periph_push(qts, in + n, 40 - n, 1);
    }
    qtest_clock_step(qts, 100000);
    g_assert_cmpint(rx_received(qts), ==, 40);
    qtest_memread(qts, DST, out, 40);
    for (int i = 0; i < 40; i++) {
        uint8_t r = 0;

        for (int b = 0; b < 8; b++) {
            r |= ((in[i] >> b) & 1) << (7 - b);
        }
        g_assert_cmphex(out[i], ==, r);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/bitscrambler/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/bitscrambler/memories", test_memories);
    qtest_add_func("/esp32p4/bitscrambler/loopback-idf", test_loopback_idf);
    qtest_add_func("/esp32p4/bitscrambler/loopback-programs",
                   test_loopback_programs);
    qtest_add_func("/esp32p4/bitscrambler/tx-path", test_tx_path);
    qtest_add_func("/esp32p4/bitscrambler/rx-path", test_rx_path);
    return g_test_run();
}
