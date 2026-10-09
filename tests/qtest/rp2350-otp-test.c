/*
 * QTest testcase for the RP2350 OTP
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest accesses count as a Secure debugger. Bus faults are observed
 * through the monitor's "x" command, which reads through core 0's bus.
 * Rows are programmed through the SBPI bridge the way the boot ROM's
 * otp_access does it.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define OTP             0x40120000
#define SW_LOCK(n)      (OTP + 4 * (n))
#define SBPI_INSTR      (OTP + 0x100)
#define SBPI_WDATA_0    (OTP + 0x104)
#define SBPI_RDATA_0    (OTP + 0x114)
#define SBPI_STATUS     (OTP + 0x124)
#define USR             (OTP + 0x128)
#define DBG             (OTP + 0x12c)
#define BIST            (OTP + 0x134)
#define CRT_KEY_W(n)    (OTP + 0x138 + 4 * (n))
#define CRITICAL        (OTP + 0x148)
#define KEY_VALID       (OTP + 0x14c)
#define DEBUGEN         (OTP + 0x150)
#define DEBUGEN_LOCK    (OTP + 0x154)
#define ARCHSEL         (OTP + 0x158)
#define BOOTDIS         (OTP + 0x160)
#define INTR            (OTP + 0x164)
#define INTE            (OTP + 0x168)
#define INTF            (OTP + 0x16c)
#define INTS            (OTP + 0x170)
#define SET             0x2000
#define CLR             0x3000

#define PSM_FRCE_OFF    0x40018004
#define PSM_WDSEL       0x40018008
#define PSM_OTP         (1u << 1)
#define PSM_PROC0       (1u << 23)
#define PSM_PROC1       (1u << 24)
#define WD_CTRL         0x400d8000
#define WD_CTRL_TRIGGER (1u << 31)

#define DATA            0x40130000
#define DATA_RAW        0x40134000
#define DATA_GUARDED    0x40138000
#define DATA_RAW_GUARDED 0x4013c000
#define ECC_ROW(r)      (DATA + 2 * (r))
#define RAW_ROW(r)      (DATA_RAW + 4 * (r))

#define CS_ROM_AUTHSTATUS 0x40140fb8

#define INSTR_EXEC      (1u << 30)
#define INSTR_IS_WR     (1u << 29)
#define INSTR_HAS_PAYLOAD (1u << 28)
#define STATUS_INSTR_DONE (1u << 4)
#define STATUS_RDATA_VLD  (1u << 0)
#define TARGET_DAP      0x02
#define TARGET_PMC      0x3a

#define INTR_APB_RD_SEC_FAIL (1u << 3)
#define INTR_APB_DCTRL_FAIL  (1u << 2)
#define INTR_SBPI_WR_FAIL    (1u << 1)

#define ROWS            4096
#define ROW_NUM_GPIOS   0x018
#define ROW_INFO_CRC0   0x036
#define ROW_CRIT1       0x040
#define ROW_KEY1_0      0xf48
#define ROW_KEY1_VALID  0xf79
#define ROW_LOCK0(p)    (0xf80 + 2 * (p))
#define ROW_LOCK1(p)    (0xf80 + 2 * (p) + 1)
/* A user page, unlocked on a blank chip. */
#define USER_ROW        0x0c0

static char *rom_path;

/* The datasheet's ECC encoder: 16 data bits, 6 parity bits above them. */
static uint32_t ecc(uint16_t x)
{
    static const uint32_t table[6] = {
        0x00ad5b, 0x00366d, 0x00c78e, 0x0007f0, 0x00f800, 0x1fffff,
    };
    uint32_t p = x;
    int i;

    for (i = 0; i < 6; i++) {
        p |= (uint32_t)(__builtin_popcount(p & table[i]) & 1) << (16 + i);
    }
    return p;
}

/* CRC-32: polynomial 0x04c11db7, reflected, seed and final XOR all-ones. */
static uint32_t crc32_le(const uint8_t *p, size_t len)
{
    uint32_t crc = 0xffffffff;
    int bit;

    while (len--) {
        crc ^= *p++;
        for (bit = 0; bit < 8; bit++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static QTestState *start(void)
{
    return qtest_initf("-M rp2350 -bios %s", rom_path);
}

static QTestState *start_with_image(const char *image)
{
    return qtest_initf("-M rp2350 -bios %s "
                       "-drive if=none,id=otp,format=raw,file=%s "
                       "-global rp2350-otp.drive=otp", rom_path, image);
}

/* An OTP image file holding `rows` (unlisted rows zero). */
static char *make_image(const uint32_t *rows)
{
    g_autofree uint32_t *le = g_new0(uint32_t, ROWS);
    GError *err = NULL;
    char *path;
    int fd, i;

    for (i = 0; rows && i < ROWS; i++) {
        le[i] = cpu_to_le32(rows[i]);
    }
    fd = g_file_open_tmp("rp2350-otp-test-XXXXXX.img", &path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, le, ROWS * 4), ==, ROWS * 4);
    close(fd);
    return path;
}

static uint32_t image_row(const char *path, int row)
{
    g_autofree gchar *buf = NULL;
    gsize len;
    uint32_t v;

    g_assert_true(g_file_get_contents(path, &buf, &len, NULL));
    g_assert_cmpuint(len, >=, ROWS * 4);
    memcpy(&v, buf + 4 * row, 4);
    return le32_to_cpu(v);
}

/* Whether core 0 can read `addr` without a bus fault. */
static bool readable(QTestState *qts, uint32_t addr)
{
    g_autofree char *out = qtest_hmp(qts, "x/1xw 0x%x", addr);

    return !strstr(out, "Cannot access memory");
}

static void sbpi_write(QTestState *qts, int target, int reg, uint8_t byte)
{
    qtest_writel(qts, SBPI_INSTR, INSTR_EXEC | INSTR_IS_WR |
                 INSTR_HAS_PAYLOAD | target << 16 | (0xc0 | reg) << 8 | byte);
    g_assert_cmphex(qtest_readl(qts, SBPI_STATUS) & STATUS_INSTR_DONE, ==,
                    STATUS_INSTR_DONE);
    qtest_writel(qts, SBPI_STATUS, STATUS_INSTR_DONE);
}

static void sbpi_cmd(QTestState *qts, int target, int cmd)
{
    qtest_writel(qts, SBPI_INSTR, INSTR_EXEC | INSTR_IS_WR | target << 16 |
                 cmd << 8);
    qtest_writel(qts, SBPI_STATUS, STATUS_INSTR_DONE);
}

/*
 * Program one row as otp_access does: DAP datapath with ECC disabled, PMC
 * sequence skipping ECC and BRP, then address, data and START/STOP. With
 * `hw_ecc` the hardware generates ECC from the low 16 bits instead.
 */
static void program(QTestState *qts, int row, uint32_t value, bool hw_ecc)
{
    qtest_writel(qts, USR, 0);
    sbpi_write(qts, TARGET_DAP, 0x3a, hw_ecc ? 0x00 : 0x01);
    sbpi_write(qts, TARGET_PMC, 0x3c, hw_ecc ? 0x00 : 0x0a);
    sbpi_write(qts, TARGET_DAP, 0x3c, row & 0xff);
    sbpi_write(qts, TARGET_DAP, 0x3d, row >> 8);
    sbpi_write(qts, TARGET_DAP, 0x00, value);
    sbpi_write(qts, TARGET_DAP, 0x01, value >> 8);
    sbpi_write(qts, TARGET_DAP, 0x20, value >> 16);
    sbpi_cmd(qts, TARGET_PMC, 0x01);
    sbpi_cmd(qts, TARGET_PMC, 0x02);
    qtest_writel(qts, USR, 1);
}

static uint16_t read_ecc(QTestState *qts, int row)
{
    return qtest_readw(qts, ECC_ROW(row));
}

/* [spec:nuos:req:emu.otp/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, USR), ==, 0x1);
    g_assert_cmphex(qtest_readl(qts, SBPI_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BIST), ==, 0x0fff0000);
    g_assert_cmphex(qtest_readl(qts, CRITICAL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, KEY_VALID), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DEBUGEN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ARCHSEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    /* The power-up state machine has finished. */
    g_assert_cmphex(qtest_readl(qts, DBG) & 0x3, ==, 0x3);
    /* Write-only key registers read zero. */
    qtest_writel(qts, CRT_KEY_W(0), 0x12345678);
    g_assert_cmphex(qtest_readl(qts, CRT_KEY_W(0)), ==, 0);
    qtest_quit(qts);
}

/*
 * A blank chip carries the manufacturing-test rows and hard locks; user
 * pages read as zero.
 */
/* [spec:nuos:req:emu.otp/test] */
static void test_factory_rows(void)
{
    QTestState *qts = start();
    uint8_t info[2 * ROW_INFO_CRC0];
    uint32_t crc;
    int i;

    g_assert_cmphex(read_ecc(qts, ROW_NUM_GPIOS), ==, 30);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(ROW_NUM_GPIOS)), ==, ecc(30));
    /* CHIPID is nonzero, and INFO_CRC covers rows 0x00-0x35. */
    g_assert_cmphex(qtest_readl(qts, DATA), !=, 0);
    for (i = 0; i < ROW_INFO_CRC0; i++) {
        uint16_t v = read_ecc(qts, i);

        info[2 * i] = v;
        info[2 * i + 1] = v >> 8;
    }
    crc = crc32_le(info, sizeof(info));
    g_assert_cmphex(qtest_readl(qts, ECC_ROW(ROW_INFO_CRC0)), ==, crc);

    /* Page 0 read-only for all; pages 1, 2 Non-secure read-only. */
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(0)), ==, 0x5);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(1)), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(2)), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(3)), ==, 0x0);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(62)), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(63)), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(ROW_LOCK1(63))), ==, 0x141414);

    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DATA_RAW_GUARDED + 4 * USER_ROW), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.otp/test] */
static void test_ecc_and_raw_views(void)
{
    QTestState *qts = start();
    uint32_t row = USER_ROW;

    /* Raw programming of a software-encoded row. */
    program(qts, row, ecc(0xbeef), false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, ecc(0xbeef));
    g_assert_cmphex(read_ecc(qts, row), ==, 0xbeef);
    /* Guarded reads return the same data. */
    g_assert_cmphex(qtest_readw(qts, DATA_GUARDED + 2 * row), ==, 0xbeef);
    g_assert_cmphex(qtest_readl(qts, DATA_RAW_GUARDED + 4 * row), ==,
                    ecc(0xbeef));

    /* A 32-bit ECC read returns the even/odd row pair. */
    program(qts, row + 1, ecc(0x1234), false);
    g_assert_cmphex(qtest_readl(qts, ECC_ROW(row)), ==, 0x1234beef);
    /* Narrow reads take their lanes from the aligned word. */
    g_assert_cmphex(qtest_readb(qts, ECC_ROW(row) + 3), ==, 0x12);
    g_assert_cmphex(qtest_readb(qts, RAW_ROW(row) + 2), ==,
                    ecc(0xbeef) >> 16);

    /* Hardware ECC generation gives the same encoding. */
    program(qts, row + 2, 0x5a5a, true);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row + 2)), ==, ecc(0x5a5a));

    /* The ECC alias covers 8 KiB only; past it is unpopulated. */
    g_assert_true(readable(qts, DATA + 0x1ffc));
    g_assert_false(readable(qts, DATA + 0x2000));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.otp/test] */
static void test_ecc_correction(void)
{
    QTestState *qts = start();
    uint32_t row = USER_ROW + 4;
    uint32_t enc = ecc(0x00f0);

    /* A single flipped bit (one set that should be clear) is corrected. */
    program(qts, row, enc | 1u << 0, false);
    g_assert_cmphex(read_ecc(qts, row), ==, 0x00f0);
    g_assert_cmphex(qtest_readw(qts, DATA_GUARDED + 2 * row), ==, 0x00f0);

    /* A flipped parity bit leaves the data intact. */
    program(qts, row + 2, ecc(0x0000) | 1u << 17, false);
    g_assert_cmphex(read_ecc(qts, row + 2), ==, 0);

    /*
     * Two flips are detected, not corrected: an unguarded read returns
     * the raw data bits, and a guarded read of either row of the pair
     * faults (RP2350-E17).
     */
    program(qts, row, enc | 1u << 0 | 1u << 1, false);
    g_assert_cmphex(read_ecc(qts, row), ==, 0x00f3);
    g_assert_false(readable(qts, DATA_GUARDED + 2 * row));
    g_assert_false(readable(qts, DATA_GUARDED + 2 * (row + 1)));
    /* Guarded raw reads are not ECC-checked. */
    g_assert_true(readable(qts, DATA_RAW_GUARDED + 4 * row));
    qtest_quit(qts);
}

/*
 * Bit repair by polarity: a row with a stray set bit takes the inverted
 * pattern with bits 23:22 set, and the ECC alias inverts it back.
 */
/* [spec:nuos:req:emu.otp/test] */
static void test_brp(void)
{
    QTestState *qts = start();
    uint32_t row = USER_ROW + 8;
    uint32_t enc = ecc(0x0001);
    uint32_t stray = 1u << 15;

    g_assert_cmphex(enc & stray, ==, 0);
    program(qts, row, stray, false);
    /* Hardware ECC with BRP enabled repairs the polarity. */
    program(qts, row, 0x0001, true);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==,
                    (~enc & 0x3fffff) | 0xc00000);
    g_assert_cmphex(read_ecc(qts, row), ==, 0x0001);
    qtest_quit(qts);
}

/* Fuses only go from 0 to 1. */
/* [spec:nuos:req:emu.otp/test] */
static void test_one_way(void)
{
    QTestState *qts = start();
    uint32_t row = USER_ROW + 12;

    program(qts, row, 0x00ff00, false);
    program(qts, row, 0x0f000f, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, 0x0fff0f);
    program(qts, row, 0, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, 0x0fff0f);
    /* Only 24 bits per row. */
    program(qts, row + 1, 0xffffff, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row + 1)), ==, 0xffffff);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.otp/test] */
static void test_sbpi(void)
{
    QTestState *qts = start();

    /* With USR.DCTRL set, SBPI is disabled and instructions are dropped. */
    qtest_writel(qts, SBPI_INSTR, INSTR_EXEC | INSTR_IS_WR |
                 INSTR_HAS_PAYLOAD | TARGET_DAP << 16 | 0xc0 << 8 | 0x5a);
    g_assert_cmphex(qtest_readl(qts, SBPI_STATUS), ==, 1u << 8);
    /* EXEC self-clears. */
    g_assert_cmphex(qtest_readl(qts, SBPI_INSTR) & INSTR_EXEC, ==, 0);
    qtest_writel(qts, SBPI_STATUS, 1u << 8);
    g_assert_cmphex(qtest_readl(qts, SBPI_STATUS), ==, 0);

    /* With the data interface off, data reads fault and raise DCTRL_FAIL. */
    qtest_writel(qts, USR, 0);
    g_assert_false(readable(qts, DATA));
    g_assert_cmphex(qtest_readl(qts, INTR), ==, INTR_APB_DCTRL_FAIL);
    qtest_writel(qts, INTR, INTR_APB_DCTRL_FAIL);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);

    /* Register writes and multi-byte reads through the payload. */
    sbpi_write(qts, TARGET_DAP, 0x30, 0x6e);
    qtest_writel(qts, SBPI_WDATA_0, 0x44332211);
    qtest_writel(qts, SBPI_INSTR, INSTR_EXEC | INSTR_IS_WR |
                 INSTR_HAS_PAYLOAD | 3 << 24 | TARGET_DAP << 16 |
                 (0xc0 | 0x31) << 8);
    qtest_writel(qts, SBPI_INSTR, INSTR_EXEC | INSTR_HAS_PAYLOAD |
                 4 << 24 | TARGET_DAP << 16 | (0x80 | 0x30) << 8);
    g_assert_cmphex(qtest_readl(qts, SBPI_STATUS), ==,
                    0x6e0000 | STATUS_INSTR_DONE | STATUS_RDATA_VLD);
    g_assert_cmphex(qtest_readl(qts, SBPI_RDATA_0), ==, 0x3322116e);
    g_assert_cmphex(qtest_readl(qts, SBPI_RDATA_0 + 4), ==, 0x44);
    /* Read data clears once read. */
    g_assert_cmphex(qtest_readl(qts, SBPI_RDATA_0), ==, 0);

    /* The PMC status, polled for completion, is not busy. */
    qtest_writel(qts, SBPI_INSTR, INSTR_EXEC | INSTR_HAS_PAYLOAD |
                 TARGET_PMC << 16 | (0x80 | 0x3f) << 8);
    g_assert_cmphex(qtest_readl(qts, SBPI_RDATA_0) & 0x80, ==, 0);
    qtest_writel(qts, USR, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.otp/test] */
static void test_soft_locks(void)
{
    QTestState *qts = start();
    uint32_t row = USER_ROW;
    int page = row / 64;

    program(qts, row, 0x123456, false);

    /* Writes OR into the lock state; it never goes back. */
    qtest_writel(qts, SW_LOCK(page), 0x4);
    qtest_writel(qts, SW_LOCK(page), 0x0);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page)), ==, 0x4);
    qtest_writel(qts, SW_LOCK(page) + CLR, 0xf);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page)), ==, 0x4);
    /* Only bits 3:0 exist. */
    qtest_writel(qts, SW_LOCK(page), 0xf0);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page)), ==, 0x4);

    /* Secure read-only: reads work, programming is refused. */
    qtest_writel(qts, SW_LOCK(page) + SET, 0x1);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, 0x123456);
    program(qts, row, 0xffffff, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, 0x123456);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, INTR_SBPI_WR_FAIL);
    qtest_writel(qts, INTR, INTR_SBPI_WR_FAIL);

    /*
     * Secure inaccessible: unguarded reads return all-ones, guarded reads
     * fault, and both raise APB_RD_SEC_FAIL.
     */
    qtest_writel(qts, SW_LOCK(page), 0x3);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, ECC_ROW(row)), ==, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, INTR_APB_RD_SEC_FAIL);
    g_assert_false(readable(qts, DATA_RAW_GUARDED + 4 * row));
    g_assert_false(readable(qts, DATA_GUARDED + 2 * row));
    g_assert_true(readable(qts, DATA_RAW_GUARDED + 4 * (row + 64)));

    /* The interrupt follows INTE, and INTF forces it. */
    g_assert_cmphex(qtest_readl(qts, INTS), ==, 0);
    qtest_writel(qts, INTE, INTR_APB_RD_SEC_FAIL);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, INTR_APB_RD_SEC_FAIL);
    qtest_writel(qts, INTR, INTR_APB_RD_SEC_FAIL);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, 0);
    qtest_writel(qts, INTF, INTR_APB_RD_SEC_FAIL);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, INTR_APB_RD_SEC_FAIL);

    /* Lock words are always readable; soft locks last until reset. */
    qtest_writel(qts, SW_LOCK(62), 0xf);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(ROW_LOCK1(62))), ==, 0x040404);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(row)), ==, 0x123456);
    qtest_quit(qts);
}

/*
 * Hard locks are programmed into the lock words and take effect at the
 * next OTP reset. A lock word protects its own writes.
 */
/* [spec:nuos:req:emu.otp/test] */
static void test_hard_locks(void)
{
    QTestState *qts = start();
    int page = USER_ROW / 64;

    /* Secure read-only, Non-secure inaccessible, triple-redundant. */
    program(qts, ROW_LOCK1(page), 0x0d0d0d, false);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page)), ==, 0);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page)), ==, 0xd);
    program(qts, USER_ROW, 1, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0);

    /* The lock word is now write-protected by itself. */
    program(qts, ROW_LOCK1(page), 0x0f0f0f, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(ROW_LOCK1(page))), ==, 0x0d0d0d);

    /* One corrupt copy of three is outvoted. */
    program(qts, ROW_LOCK1(page + 1), 0x000003, false);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(page + 1)), ==, 0);
    qtest_quit(qts);
}

/*
 * Access keys: a page registered to a key is readable only while the key
 * is entered in CRT_KEY, and a valid key's own rows are inaccessible.
 */
/* [spec:nuos:req:emu.otp/test] */
static void test_access_keys(void)
{
    g_autofree uint32_t *rows = g_new0(uint32_t, ROWS);
    g_autofree char *image = NULL;
    int page = USER_ROW / 64;
    QTestState *qts;
    int i;

    /* Key 2 = 0x0001 0x0002 ... 0x0008, valid. */
    for (i = 0; i < 8; i++) {
        rows[ROW_KEY1_0 + 8 + i] = ecc(i + 1);
    }
    rows[ROW_KEY1_VALID + 1] = 0x010101;
    /* The user page needs key 2 to read, inaccessible without it. */
    rows[ROW_LOCK0(page)] = 0x505050;
    rows[USER_ROW] = 0xabcdef;
    image = make_image(rows);
    qts = start_with_image(image);

    g_assert_cmphex(qtest_readl(qts, KEY_VALID), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(ROW_KEY1_0 + 8)), ==,
                    0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0xffffffff);
    qtest_writel(qts, INTR, 0x1f);

    qtest_writel(qts, CRT_KEY_W(0), 0x00020001);
    qtest_writel(qts, CRT_KEY_W(1), 0x00040003);
    qtest_writel(qts, CRT_KEY_W(2), 0x00060005);
    qtest_writel(qts, CRT_KEY_W(3), 0x00080007);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0xabcdef);
    /* A read key gives read-only access. */
    program(qts, USER_ROW, 0xffffff, false);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0xabcdef);

    /* Erasing the key locks the page again. */
    qtest_writel(qts, CRT_KEY_W(3), 0);
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0xffffffff);
    qtest_quit(qts);
    unlink(image);
}

/*
 * Rows persist in the backing drive. A blank image is filled with the
 * factory rows.
 */
/* [spec:nuos:req:emu.otp/test] */
static void test_persistence(void)
{
    g_autofree char *image = make_image(NULL);
    QTestState *qts;

    qts = start_with_image(image);
    g_assert_cmphex(read_ecc(qts, ROW_NUM_GPIOS), ==, 30);
    program(qts, USER_ROW, ecc(0xc0de), false);
    qtest_quit(qts);

    g_assert_cmphex(image_row(image, ROW_NUM_GPIOS), ==, ecc(30));
    g_assert_cmphex(image_row(image, ROW_LOCK1(0)), ==, 0x151515);
    g_assert_cmphex(image_row(image, USER_ROW), ==, ecc(0xc0de));

    qts = start_with_image(image);
    g_assert_cmphex(read_ecc(qts, USER_ROW), ==, 0xc0de);
    program(qts, USER_ROW + 1, 0x800000, false);
    qtest_quit(qts);
    g_assert_cmphex(image_row(image, USER_ROW + 1), ==, 0x800000);

    /* Without a drive, nothing persists. */
    qts = start();
    g_assert_cmphex(qtest_readl(qts, RAW_ROW(USER_ROW)), ==, 0);
    qtest_quit(qts);
    unlink(image);
}

/*
 * Critical flags are three-of-eight votes, latched at OTP reset; the
 * debug disables reach CoreSight, and DEBUGEN re-enables debug unless
 * DEBUGEN_LOCK holds it.
 */
/* [spec:nuos:req:emu.otp/test] */
static void test_critical_flags(void)
{
    g_autofree uint32_t *rows = g_new0(uint32_t, ROWS);
    g_autofree char *image = NULL;
    QTestState *qts;

    /* DEBUG_DISABLE in three rows; SECURE_DEBUG_DISABLE in only two. */
    rows[ROW_CRIT1 + 0] = 0x6;
    rows[ROW_CRIT1 + 3] = 0x6;
    rows[ROW_CRIT1 + 7] = 0x4;
    image = make_image(rows);
    qts = start_with_image(image);

    g_assert_cmphex(qtest_readl(qts, CRITICAL), ==, 0x4);
    g_assert_cmphex(qtest_readl(qts, CS_ROM_AUTHSTATUS), ==, 0xaa);
    /* Re-enabling Secure debug alone is not enough. */
    qtest_writel(qts, DEBUGEN, 0x00a);
    g_assert_cmphex(qtest_readl(qts, CS_ROM_AUTHSTATUS), ==, 0xaa);
    qtest_writel(qts, DEBUGEN, 0x10f);
    g_assert_cmphex(qtest_readl(qts, CS_ROM_AUTHSTATUS), ==, 0xff);
    /* Locked bits keep their value. */
    qtest_writel(qts, DEBUGEN_LOCK, 0x001);
    qtest_writel(qts, DEBUGEN_LOCK, 0x000);
    g_assert_cmphex(qtest_readl(qts, DEBUGEN_LOCK), ==, 0x001);
    qtest_writel(qts, DEBUGEN, 0);
    g_assert_cmphex(qtest_readl(qts, DEBUGEN), ==, 0x001);
    g_assert_cmphex(qtest_readl(qts, CS_ROM_AUTHSTATUS), ==, 0xaa);

    /* Programming a third SECURE_DEBUG_DISABLE copy counts after reset. */
    program(qts, ROW_CRIT1 + 5, 0x2, false);
    g_assert_cmphex(qtest_readl(qts, CRITICAL), ==, 0x4);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, CRITICAL), ==, 0x6);
    g_assert_cmphex(qtest_readl(qts, DEBUGEN), ==, 0);
    qtest_quit(qts);
    unlink(image);
}

/* [spec:nuos:req:emu.otp/test] */
static void test_secure_debug_disable(void)
{
    g_autofree uint32_t *rows = g_new0(uint32_t, ROWS);
    g_autofree char *image = NULL;
    QTestState *qts;
    int i;

    for (i = 0; i < 8; i++) {
        rows[ROW_CRIT1 + i] = 0x3;
    }
    image = make_image(rows);
    qts = start_with_image(image);

    g_assert_cmphex(qtest_readl(qts, CRITICAL), ==, 0x3);
    /* Non-secure debug stays enabled. */
    g_assert_cmphex(qtest_readl(qts, CS_ROM_AUTHSTATUS), ==, 0xaf);
    qtest_writel(qts, DEBUGEN, 0x00a);
    g_assert_cmphex(qtest_readl(qts, CS_ROM_AUTHSTATUS), ==, 0xff);
    /* SECURE_BOOT_ENABLE forces ARCHSEL to Arm. */
    qtest_writel(qts, ARCHSEL, 0x3);
    g_assert_cmphex(qtest_readl(qts, ARCHSEL), ==, 0);
    qtest_quit(qts);
    unlink(image);
}

/* [spec:nuos:req:emu.otp/test] */
static void test_bootdis(void)
{
    QTestState *qts = start();

    /* NEXT can be set but not cleared; NOW is write-1-to-clear. */
    qtest_writel(qts, BOOTDIS, 0x2);
    qtest_writel(qts, BOOTDIS, 0x0);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0x2);
    qtest_writel(qts, BOOTDIS + CLR, 0x2);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0x2);
    qtest_writel(qts, SW_LOCK(3), 0x3);

    /* A reset of the processors alone leaves the OTP stage alone. */
    qtest_writel(qts, PSM_FRCE_OFF + SET, PSM_PROC0 | PSM_PROC1);
    qtest_writel(qts, PSM_FRCE_OFF + CLR, PSM_PROC0 | PSM_PROC1);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0x2);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(3)), ==, 0x3);

    /*
     * Resetting the OTP stage reruns the power-up sequence, which reopens
     * the soft locks, and moves NEXT to NOW.
     */
    qtest_writel(qts, PSM_FRCE_OFF + SET, PSM_OTP);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0x1);
    g_assert_cmphex(qtest_readl(qts, SW_LOCK(3)), ==, 0);
    qtest_writel(qts, BOOTDIS, 0x1);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0);
    qtest_writel(qts, PSM_FRCE_OFF + CLR, PSM_OTP);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0);

    /* The watchdog resets the OTP stage when WDSEL selects it. */
    qtest_writel(qts, BOOTDIS, 0x2);
    qtest_writel(qts, PSM_WDSEL, PSM_OTP);
    qtest_writel(qts, WD_CTRL + SET, WD_CTRL_TRIGGER);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0x1);

    /* A chip-level cold reset clears both. */
    qtest_writel(qts, BOOTDIS, 0x2);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, BOOTDIS), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-otp-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/otp/reset-values", test_reset_values);
    qtest_add_func("/rp2350/otp/factory-rows", test_factory_rows);
    qtest_add_func("/rp2350/otp/ecc-and-raw-views", test_ecc_and_raw_views);
    qtest_add_func("/rp2350/otp/ecc-correction", test_ecc_correction);
    qtest_add_func("/rp2350/otp/brp", test_brp);
    qtest_add_func("/rp2350/otp/one-way", test_one_way);
    qtest_add_func("/rp2350/otp/sbpi", test_sbpi);
    qtest_add_func("/rp2350/otp/soft-locks", test_soft_locks);
    qtest_add_func("/rp2350/otp/hard-locks", test_hard_locks);
    qtest_add_func("/rp2350/otp/access-keys", test_access_keys);
    qtest_add_func("/rp2350/otp/persistence", test_persistence);
    qtest_add_func("/rp2350/otp/critical-flags", test_critical_flags);
    qtest_add_func("/rp2350/otp/secure-debug-disable",
                   test_secure_debug_disable);
    qtest_add_func("/rp2350/otp/bootdis", test_bootdis);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
