/*
 * QTest testcase for the RP2350 SHA-256 accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define XOR                 0x1000
#define SET                 0x2000
#define CLR                 0x3000
#define NO_REPLICATE        0x4000

#define SHA256              0x400f8000
#define CSR                 (SHA256 + 0x00)
#define WDATA               (SHA256 + 0x04)
#define SUM(n)              (SHA256 + 0x08 + 4 * (n))

#define CSR_START           (1u << 0)
#define CSR_WDATA_RDY       (1u << 1)
#define CSR_SUM_VLD         (1u << 2)
#define CSR_ERR             (1u << 4)
#define CSR_DMA_SIZE(n)     ((uint32_t)(n) << 8)
#define CSR_DMA_SIZE_MASK   (3u << 8)
#define CSR_BSWAP           (1u << 12)
#define CSR_RESET           0x1206u

/* 57 cycles of the 150 MHz clk_sys. */
#define DIGEST_NS           380

static char *rom_path;

/* How the message is written to WDATA. */
typedef enum {
    WRITE_WORDS,            /* 32-bit writes, BSWAP on */
    WRITE_HALFWORDS,        /* 16-bit writes, BSWAP on */
    WRITE_BYTES,            /* 8-bit writes, BSWAP on */
    WRITE_SWAPPED_WORDS,    /* big-endian words, BSWAP off */
} WriteMode;

typedef struct {
    const char *msg;
    uint32_t digest[8];
} Vector;

/* FIPS 180-4 example messages (NIST CSRC "SHA256.pdf") and the empty one. */
static const Vector vectors[] = {
    { "", { 0xe3b0c442, 0x98fc1c14, 0x9afbf4c8, 0x996fb924,
            0x27ae41e4, 0x649b934c, 0xa495991b, 0x7852b855 } },
    { "abc", { 0xba7816bf, 0x8f01cfea, 0x414140de, 0x5dae2223,
               0xb00361a3, 0x96177a9c, 0xb410ff61, 0xf20015ad } },
    { "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
      { 0x248d6a61, 0xd20638b8, 0xe5c02693, 0x0c3e6039,
        0xa33ce459, 0x64ff2167, 0xf6ecedd4, 0x19db06c1 } },
    { "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
      "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
      { 0xcf5b16a7, 0x78af8380, 0x036ce59e, 0x7b049237,
        0x0b249b11, 0xe8f07a51, 0xafac4503, 0x7afee9d1 } },
};

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

/* Pad `msg` as FIPS 180-4 section 5.1.1 does; returns the padded length. */
static size_t pad(const char *msg, uint8_t *buf, size_t bufsize)
{
    size_t len = strlen(msg);
    size_t padded = (len + 9 + 63) & ~(size_t)63;
    uint64_t bits = (uint64_t)len * 8;

    g_assert_cmpuint(padded, <=, bufsize);
    memset(buf, 0, padded);
    memcpy(buf, msg, len);
    buf[len] = 0x80;
    for (int i = 0; i < 8; i++) {
        buf[padded - 1 - i] = bits >> (8 * i);
    }
    return padded;
}

static void wait_ready(QTestState *qts)
{
    if (!(qtest_readl(qts, CSR) & CSR_WDATA_RDY)) {
        qtest_clock_step(qts, DIGEST_NS);
    }
    g_assert_true(qtest_readl(qts, CSR) & CSR_WDATA_RDY);
}

static void hash(QTestState *qts, const char *msg, WriteMode mode,
                 uint32_t *digest)
{
    uint8_t buf[256];
    size_t len = pad(msg, buf, sizeof(buf));

    if (mode == WRITE_SWAPPED_WORDS) {
        qtest_writel(qts, CSR + CLR, CSR_BSWAP);
    } else {
        qtest_writel(qts, CSR + SET, CSR_BSWAP);
    }
    qtest_writel(qts, CSR + SET, CSR_START);
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_WDATA_RDY | CSR_SUM_VLD),
                    ==, CSR_WDATA_RDY | CSR_SUM_VLD);
    for (size_t i = 0; i < len; i += 4) {
        wait_ready(qts);
        switch (mode) {
        case WRITE_WORDS:
            qtest_writel(qts, WDATA, ldl_le_p(buf + i));
            break;
        case WRITE_SWAPPED_WORDS:
            qtest_writel(qts, WDATA, ldl_be_p(buf + i));
            break;
        case WRITE_HALFWORDS:
            qtest_writew(qts, WDATA, lduw_le_p(buf + i));
            qtest_writew(qts, WDATA, lduw_le_p(buf + i + 2));
            break;
        case WRITE_BYTES:
            for (int j = 0; j < 4; j++) {
                qtest_writeb(qts, WDATA, buf[i + j]);
            }
            break;
        }
    }
    g_assert_false(qtest_readl(qts, CSR) & CSR_SUM_VLD);
    qtest_clock_step(qts, DIGEST_NS);
    g_assert_true(qtest_readl(qts, CSR) & CSR_SUM_VLD);
    for (int i = 0; i < 8; i++) {
        digest[i] = qtest_readl(qts, SUM(i));
    }
}

static void check_vector(QTestState *qts, const Vector *v, WriteMode mode)
{
    uint32_t digest[8];

    hash(qts, v->msg, mode, digest);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(digest[i], ==, v->digest[i]);
    }
}

/* [spec:nuos:req:emu.sha256/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, CSR), ==, CSR_RESET);
    g_assert_cmphex(qtest_readl(qts, WDATA), ==, 0);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, SUM(i)), ==, 0);
    }
    /* START loads the initial hash value (FIPS 180-4 5.3.3). */
    qtest_writel(qts, CSR, CSR_RESET | CSR_START);
    g_assert_cmphex(qtest_readl(qts, CSR), ==, CSR_RESET);
    g_assert_cmphex(qtest_readl(qts, SUM(0)), ==, 0x6a09e667);
    g_assert_cmphex(qtest_readl(qts, SUM(7)), ==, 0x5be0cd19);
    /* The status bits are read-only and SUMs ignore writes. */
    qtest_writel(qts, CSR, 0);
    g_assert_cmphex(qtest_readl(qts, CSR), ==,
                    CSR_WDATA_RDY | CSR_SUM_VLD);
    qtest_writel(qts, SUM(0), 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SUM(0)), ==, 0x6a09e667);
    /* Narrow reads pick out the register's lanes. */
    g_assert_cmphex(qtest_readb(qts, SUM(0) + 3), ==, 0x6a);
    g_assert_cmphex(qtest_readw(qts, SUM(0) + 2), ==, 0x6a09);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sha256/test] */
static void test_vectors(void)
{
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(vectors); i++) {
        check_vector(qts, &vectors[i], WRITE_WORDS);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sha256/test] */
static void test_narrow_writes(void)
{
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(vectors); i++) {
        check_vector(qts, &vectors[i], WRITE_HALFWORDS);
        check_vector(qts, &vectors[i], WRITE_BYTES);
    }
    qtest_quit(qts);
}

/*
 * Narrow writes take their addressed lanes: a byte written to WDATA+1
 * in the non-replicating window is the data, the zeroed lanes are not.
 */
/* [spec:nuos:req:emu.sha256/test] */
static void test_narrow_lanes(void)
{
    QTestState *qts = start();
    uint8_t buf[64];
    size_t len = pad("abc", buf, sizeof(buf));

    qtest_writel(qts, CSR + SET, CSR_START);
    for (size_t i = 0; i < len; i++) {
        qtest_writeb(qts, WDATA + NO_REPLICATE + (i & 3), buf[i]);
    }
    qtest_clock_step(qts, DIGEST_NS);
    g_assert_cmphex(qtest_readl(qts, SUM(0)), ==, vectors[1].digest[0]);
    g_assert_cmphex(qtest_readl(qts, SUM(7)), ==, vectors[1].digest[7]);
    qtest_quit(qts);
}

/*
 * Moving the shift register from below to above 32 bits in one write
 * loses its oldest data: of three bytes then a halfword, the first byte
 * is lost and the word committed is the other two bytes and the halfword.
 */
/* [spec:nuos:req:emu.sha256/test] */
static void test_mixed_sizes(void)
{
    QTestState *qts = start();
    uint32_t expect[8];
    uint8_t buf[64];
    size_t len = pad("abc", buf, sizeof(buf));

    hash(qts, "abc", WRITE_WORDS, expect);

    qtest_writel(qts, CSR + SET, CSR_START);
    qtest_writeb(qts, WDATA, 0xee);
    qtest_writeb(qts, WDATA, buf[0]);
    qtest_writeb(qts, WDATA, buf[1]);
    qtest_writew(qts, WDATA, lduw_le_p(buf + 2));
    for (size_t i = 4; i < len; i += 4) {
        qtest_writel(qts, WDATA, ldl_le_p(buf + i));
    }
    qtest_clock_step(qts, DIGEST_NS);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, SUM(i)), ==, expect[i]);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.sha256/test] */
static void test_bswap(void)
{
    QTestState *qts = start();
    uint32_t a[8], b[8];

    for (int i = 0; i < ARRAY_SIZE(vectors); i++) {
        check_vector(qts, &vectors[i], WRITE_SWAPPED_WORDS);
    }
    g_assert_false(qtest_readl(qts, CSR) & CSR_BSWAP);

    /* The same little-endian words without BSWAP hash a different message. */
    hash(qts, "abc", WRITE_WORDS, a);
    qtest_writel(qts, CSR + CLR, CSR_BSWAP);
    qtest_writel(qts, CSR + SET, CSR_START);
    {
        uint8_t buf[64];
        size_t len = pad("abc", buf, sizeof(buf));

        for (size_t i = 0; i < len; i += 4) {
            qtest_writel(qts, WDATA, ldl_le_p(buf + i));
        }
    }
    qtest_clock_step(qts, DIGEST_NS);
    for (int i = 0; i < 8; i++) {
        b[i] = qtest_readl(qts, SUM(i));
    }
    g_assert_cmpmem(a, sizeof(a), vectors[1].digest, sizeof(a));
    g_assert_false(memcmp(a, b, sizeof(a)) == 0);

    /* The atomic aliases act on BSWAP and DMA_SIZE. */
    qtest_writel(qts, CSR + XOR, CSR_BSWAP | CSR_DMA_SIZE(3));
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_BSWAP | CSR_DMA_SIZE_MASK),
                    ==, CSR_BSWAP | CSR_DMA_SIZE(1));
    qtest_quit(qts);
}

/*
 * WDATA_RDY is low for 57 clk_sys cycles after a block's 16th word, and
 * SUM_VLD low from the block's first write until its digest is done.
 */
/* [spec:nuos:req:emu.sha256/test] */
static void test_busy_timing(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CSR + SET, CSR_START);
    qtest_writel(qts, WDATA, 0x80);
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_WDATA_RDY | CSR_SUM_VLD),
                    ==, CSR_WDATA_RDY);
    /* The block waits for its data indefinitely. */
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_WDATA_RDY | CSR_SUM_VLD),
                    ==, CSR_WDATA_RDY);
    g_assert_cmphex(qtest_readl(qts, SUM(0)), ==, 0x6a09e667);
    for (int i = 1; i < 16; i++) {
        qtest_writel(qts, WDATA, 0);
    }
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_WDATA_RDY | CSR_SUM_VLD),
                    ==, 0);
    qtest_clock_step(qts, DIGEST_NS - 10);
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_WDATA_RDY | CSR_SUM_VLD),
                    ==, 0);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(qtest_readl(qts, CSR) & (CSR_WDATA_RDY | CSR_SUM_VLD),
                    ==, CSR_WDATA_RDY | CSR_SUM_VLD);
    /* The digest of the empty message. */
    g_assert_cmphex(qtest_readl(qts, SUM(0)), ==, vectors[0].digest[0]);
    qtest_quit(qts);
}

/*
 * A write while WDATA_RDY is low is dropped and sets ERR_WDATA_NOT_RDY,
 * which writing 1 clears, through the clear alias as pico-sdk does.
 */
/* [spec:nuos:req:emu.sha256/test] */
static void test_not_ready_error(void)
{
    QTestState *qts = start();
    uint8_t buf[128];
    size_t len = pad(vectors[2].msg, buf, sizeof(buf));

    g_assert_cmpuint(len, ==, 128);
    qtest_writel(qts, CSR + SET, CSR_START);
    for (size_t i = 0; i < 64; i += 4) {
        qtest_writel(qts, WDATA, ldl_le_p(buf + i));
    }
    g_assert_false(qtest_readl(qts, CSR) & CSR_ERR);
    qtest_writel(qts, WDATA, 0xdeadbeef);
    qtest_writeb(qts, WDATA, 0xde);
    g_assert_cmphex(qtest_readl(qts, CSR), ==,
                    (CSR_RESET & ~(CSR_WDATA_RDY | CSR_SUM_VLD)) | CSR_ERR);

    /* The dropped writes left the message intact. */
    qtest_clock_step(qts, DIGEST_NS);
    for (size_t i = 64; i < len; i += 4) {
        qtest_writel(qts, WDATA, ldl_le_p(buf + i));
    }
    qtest_clock_step(qts, DIGEST_NS);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, SUM(i)), ==, vectors[2].digest[i]);
    }

    /* START leaves the error; the set and clear aliases clear it. */
    qtest_writel(qts, CSR + SET, CSR_START);
    g_assert_true(qtest_readl(qts, CSR) & CSR_ERR);
    qtest_writel(qts, CSR + CLR, CSR_ERR);
    g_assert_false(qtest_readl(qts, CSR) & CSR_ERR);

    for (int i = 0; i < 17; i++) {
        qtest_writel(qts, WDATA, 0);
    }
    g_assert_true(qtest_readl(qts, CSR) & CSR_ERR);
    qtest_writel(qts, CSR, CSR_RESET | CSR_ERR);
    g_assert_false(qtest_readl(qts, CSR) & CSR_ERR);
    qtest_quit(qts);
}

/* START abandons a block in progress, even one being digested. */
/* [spec:nuos:req:emu.sha256/test] */
static void test_restart(void)
{
    QTestState *qts = start();

    qtest_writel(qts, CSR + SET, CSR_START);
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, WDATA, 0x55555555);
    }
    qtest_writel(qts, CSR + SET, CSR_START);
    g_assert_cmphex(qtest_readl(qts, CSR), ==, CSR_RESET);
    qtest_clock_step(qts, DIGEST_NS);
    g_assert_cmphex(qtest_readl(qts, SUM(0)), ==, 0x6a09e667);

    for (int i = 0; i < 5; i++) {
        qtest_writel(qts, WDATA, 0x55555555);
    }
    qtest_writeb(qts, WDATA, 0x55);
    check_vector(qts, &vectors[1], WRITE_WORDS);
    qtest_quit(qts);
}

/*
 * DREQ requests one block's worth of transfers of the CSR.DMA_SIZE size,
 * then drops until the block has been digested.
 */
/* [spec:nuos:req:emu.sha256/test] */
static void test_dreq(void)
{
    static const struct {
        uint32_t dma_size;
        unsigned bytes;
    } sizes[] = { { 0, 1 }, { 1, 2 }, { 2, 4 } };
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, "/machine/soc/sha256", "dreq");
    for (int n = 0; n < ARRAY_SIZE(sizes); n++) {
        unsigned transfers = 64 / sizes[n].bytes;

        qtest_writel(qts, CSR + XOR, (qtest_readl(qts, CSR) ^
                     CSR_DMA_SIZE(sizes[n].dma_size)) & CSR_DMA_SIZE_MASK);
        qtest_writel(qts, CSR + SET, CSR_START);
        for (int block = 0; block < 2; block++) {
            for (unsigned i = 0; i < transfers; i++) {
                g_assert_true(qtest_get_irq(qts, 0));
                switch (sizes[n].bytes) {
                case 1:
                    qtest_writeb(qts, WDATA, 0);
                    break;
                case 2:
                    qtest_writew(qts, WDATA, 0);
                    break;
                default:
                    qtest_writel(qts, WDATA, 0);
                    break;
                }
            }
            g_assert_false(qtest_get_irq(qts, 0));
            qtest_clock_step(qts, DIGEST_NS);
        }
        g_assert_true(qtest_get_irq(qts, 0));
    }

    /* A size mismatch: 32-bit requests are used up by 16 byte writes. */
    qtest_writel(qts, CSR + SET, CSR_START);
    for (int i = 0; i < 16; i++) {
        g_assert_true(qtest_get_irq(qts, 0));
        qtest_writeb(qts, WDATA, 0);
    }
    g_assert_false(qtest_get_irq(qts, 0));
    g_assert_true(qtest_readl(qts, CSR) & CSR_WDATA_RDY);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-sha256-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/sha256/reset", test_reset);
    qtest_add_func("/rp2350/sha256/vectors", test_vectors);
    qtest_add_func("/rp2350/sha256/narrow-writes", test_narrow_writes);
    qtest_add_func("/rp2350/sha256/narrow-lanes", test_narrow_lanes);
    qtest_add_func("/rp2350/sha256/mixed-sizes", test_mixed_sizes);
    qtest_add_func("/rp2350/sha256/bswap", test_bswap);
    qtest_add_func("/rp2350/sha256/busy-timing", test_busy_timing);
    qtest_add_func("/rp2350/sha256/not-ready-error", test_not_ready_error);
    qtest_add_func("/rp2350/sha256/restart", test_restart);
    qtest_add_func("/rp2350/sha256/dreq", test_dreq);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
