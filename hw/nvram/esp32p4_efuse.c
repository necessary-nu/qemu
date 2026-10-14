/*
 * ESP32-P4 eFuse controller and its 4096-bit eFuse array
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), section
 * 9.4 (eFuse Controller), ESP-IDF's efuse_reg.h for v3, its eFuse field
 * table esp_efuse_table_v3.0.csv and esptool's esp32p4_v3.0.yaml (which
 * give each BLOCK0 field's write-protection bit), and ESP-IDF's eFuse HAL
 * and ROM routines, which show how software drives the controller.
 *
 * The array holds 4096 bits: EFUSE_WR_DIS (32 bits, stored once); four
 * copies of BLOCK0's other 152 bits; and BLOCK1 to BLOCK10, each a
 * Reed-Solomon RS(44, 32) codeword of data and 12 parity bytes (BLOCK1 has
 * 24 data bytes and is coded as if 8 zero bytes followed them).
 *
 *   - Programming: software writes EFUSE_PGM_DATAn and, for BLOCK1 to 10,
 *     the parity in EFUSE_PGM_CHECK_VALUEn, sets EFUSE_OP_CODE to 0x5A5A
 *     and EFUSE_PGM_CMD with the block number. Bits only go from 0 to 1,
 *     and the bits EFUSE_WR_DIS protects (as loaded by the last read) are
 *     not programmed. The parity is programmed as given: the controller
 *     does not compute it, so a wrong one, or a second programming of a
 *     coded block, leaves a codeword the next read finds errors in.
 *   - Reading: EFUSE_OP_CODE 0x5AA5 and EFUSE_READ_CMD load the RD_*
 *     registers from the array, taking BLOCK0's parameters as the OR of
 *     their four copies (the TRM does not say how it combines them; a bit
 *     that fails to program stays 0, so one copy that took it is taken
 *     to hold it) and recording where the copies differ in
 *     EFUSE_RD_REPEAT_ERRn, and correcting up to six bad bytes of each
 *     coded block, counted in EFUSE_RD_RS_ERRn, which has a fail bit for
 *     a block it cannot correct (whose data is then loaded as stored, and
 *     whose count is left zero, as the TRM does not give one).
 *     EFUSE_RD_DIS hides BLOCK4 to 10 from software, which reads them as
 *     zero; the cryptography modules still take their keys (the hooks in
 *     esp32p4_efuse.h). Protection changes take effect at the next read.
 *   - Both operations take time, from the timing registers, in cycles of
 *     the controller's 20 MHz clock: EFUSE_CMD's bit clears and
 *     EFUSE_INT_RAW's done bit sets when one ends. A read takes
 *     EFUSE_READ_INIT_NUM cycles, then TSUR_A + TRD + THR_A for each of
 *     the array's 128 words (one more cycle between BLOCK0 and the coded
 *     blocks); programming takes EFUSE_PWR_ON_NUM, then for each bit of
 *     the block as stored TSUP_A + TPGM + THP_A if it is programmed and
 *     TPGM_INACTIVE if not, then EFUSE_PWR_OFF_NUM. The registers that
 *     time the state machine take effect when EFUSE_UPDATE is written, as
 *     that bit's description ("update multi-bit register signals") has it.
 *   - APB2OTP: with EFUSE_APB2OTP_EN set, the APB2OTP registers read the
 *     array as stored: EFUSE_WR_DIS, BLOCK0's four copies, and each coded
 *     block's data words then its parity words. Read-protected blocks
 *     read as zero there too: the TRM does not say, and reading keys
 *     past EFUSE_RD_DIS would defeat it.
 *
 * The array is backed by a raw file, attached as Espressif's QEMU and
 * ESP-IDF's "idf.py qemu" attach theirs:
 *
 *   -drive file=efuse.bin,if=none,format=raw,id=efuse
 *   -global driver=nvram.esp32p4.efuse,property=drive,value=efuse
 *
 * Its first 336 bytes are the layout of Espressif's QEMU for the ESP32-C3
 * and C6 (and of esp-emu's --efuse): the RD_* registers' words, BLOCK0 to
 * BLOCK10, little-endian; for BLOCK0 the first copy and for the coded
 * blocks their data as stored. A file longer than 512 bytes (1 KiB, as
 * "qemu-img create -f raw efuse.bin 1K" or ESP-IDF makes them, will do)
 * also keeps the rest of the array, as differences that are zero for a
 * consistent array, so that files from Espressif's tools read clean:
 *
 *   336..395  BLOCK0's copies 2 to 4, XOR copy 1, five words each
 *   396..515  each coded block's 12 parity bytes, BLOCK1 to BLOCK10, XOR
 *             the parity of its data
 *
 * A file that is all zero is a blank one: it is given the defaults of a
 * factory-fresh chip, as is the volatile array used without a file. A
 * file of 512 bytes or less keeps only the data, and the array is then
 * taken to be consistent each time it is loaded; the block layer sizes
 * files in 512-byte sectors, so this is the size it sees. Such a file
 * reads as zero past its end, and grows to 336 bytes when written. An
 * empty file is refused. Programming writes the file when it ends.
 *
 * The defaults are those of a factory-fresh ESP32-P4NRW32X of the
 * revision the "chip-revision" property gives, v3.0 unless set,
 * everything else zero:
 *
 *   - The wafer version: chip-revision's major number in
 *     EFUSE_WAFER_VERSION_MAJOR_LO and _HI, its minor number in _MINOR.
 *     Each v3.x revision has its own boot ROM, which prints its ECO:
 *     esp32p4-eco5-20250430 on v3.0 (the boot log of ESP-IDF's flash
 *     encryption guide for the P4), eco6-20251011 on v3.1 and
 *     eco7-20260109 on v3.2 (as users' logs from those revisions show), so
 *     the machine sets the property from the ECO of the HP ROM image it
 *     runs: v3.0 for the ROM it ships, whose ECO is 5. ESP-IDF's default
 *     minimum revision is v3.1 (CONFIG_ESP32P4_REV_MIN_301), so its
 *     default builds refuse to run on v3.0, as they do on the chip.
 *   - Block version 1.1: ESP-IDF's bootloader accepts up to v1.99, takes
 *     ADC and LDO calibration from v0.1, and the PMU's from v0.2 except
 *     v1.0, which the v3.x silicon's code singles out.
 *   - Package version 0, which esptool names "ESP32-P4"; operating
 *     temperature 0, the -40 to 85 C range; PSRAM capacity 2 and vendor
 *     1. The PSRAM fields' encoding is not published and ESP-IDF does not
 *     read them (it asks the PSRAM die): 2 stands for the second of the
 *     two in-package sizes, 32 MB, and 1 for AP Memory, the vendor
 *     ESP-IDF's driver expects.
 *   - MAC 24:0A:C4:00:00:01, from Espressif's OUI. The chip has no MAC
 *     CRC: BLOCK1's parity covers the MAC, and the default array carries
 *     valid parity for every block.
 *   - Calibration: values ESP-IDF's calibration code turns into nominal
 *     ones, not those of a real part: the LDO K, VOS and C trims at their
 *     nominal 1.000, 0 and 1.000; VO1 and VO2 at 1.8 V (DREF 8, MUL 4);
 *     the PMU's active HP and LP DBIAS at its defaults, 24 and 29 (fuse
 *     values 8 and 9) and an LP to DCDC gap of 0; ADC init codes of about
 *     1800, HI_DOUT offsets and per-channel differences of a few codes,
 *     and a temperature sensor offset of 2.
 *
 * Not modelled, as nothing documents them: what EFUSE_PGM_DATAn hold
 * after a read, which the TRM says uses them as storage (they keep what
 * software wrote); EFUSE_BLK0_VALID_BIT_CNT and the OTP_* signal bits of
 * EFUSE_STATUS (zero); EFUSE_STATE while programming (6, as the published
 * states 0 to 5 are all of reading); RS correction bypass
 * (EFUSE_BYPASS_RS_CORRECTION, logged as unimplemented when set); the
 * eFuse SRAM power controls of EFUSE_CLK_REG (kept). The ECDSA block
 * selection reads back as configured (EFUSE_CUR_ECDSA_* = _CFG_). A
 * command with the wrong EFUSE_OP_CODE is refused and its bit cleared.
 * Write protection follows esptool's field table, except EFUSE_USB_PHY_SEL,
 * which the table leaves unprotected and TRM table 9.4-1 gives bit 30.
 *
 * The controller's clock comes in on the "clk" clock input, 20 MHz when
 * nothing drives it; while it is stopped an operation stalls. The
 * "reset" GPIO input holds the controller in reset; released, it loads
 * the array as at power-on, taking a read's time. A power-on or system
 * reset loads it at once, as the chip has before its cores start.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/gpio/esp32_line.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"

#define A_PGM_DATA0             0x000
#define A_PGM_CHECK_VALUE0      0x020
#define A_RD_WR_DIS             0x02c
#define A_RD_REPEAT_ERR0        0x17c
#define A_RD_REPEAT_ERR4        0x18c
#define A_ECDSA                 0x1b0
#define A_RD_RS_ERR0            0x1c0
#define A_RD_RS_ERR1            0x1c4
#define A_CLK                   0x1c8
#define A_CONF                  0x1cc
#define A_STATUS                0x1d0
#define A_CMD                   0x1d4
#define A_INT_RAW               0x1d8
#define A_INT_ST                0x1dc
#define A_INT_ENA               0x1e0
#define A_INT_CLR               0x1e4
#define A_DAC_CONF              0x1e8
#define A_RD_TIM_CONF           0x1ec
#define A_WR_TIM_CONF1          0x1f0
#define A_WR_TIM_CONF2          0x1f4
#define A_WR_TIM_CONF0_RS_BYPASS 0x1f8
#define A_DATE                  0x1fc
#define A_APB2OTP_WR_DIS        0x800
#define A_APB2OTP_BLK10_W11     0xa00
#define A_APB2OTP_EN            0xa08

#define READ_OP_CODE            0x5aa5
#define PGM_OP_CODE             0x5a5a

#define CMD_READ                BIT(0)
#define CMD_PGM                 BIT(1)
#define CMD_BLK_NUM_SHIFT       2
#define CMD_MASK                0x3f

#define INT_READ_DONE           BIT(0)
#define INT_PGM_DONE            BIT(1)
#define INT_MASK                0x3

#define CLK_MEM_FORCE_PD        BIT(0)
#define CLK_MASK                0x10007

#define RS_BYPASS_CORRECTION    BIT(0)
#define RS_BYPASS_UPDATE        BIT(12)
#define RS_BYPASS_MASK          0x1fefff

#define DAC_CONF_MASK           0x3ffff
#define DATE_MASK               0x0fffffff
#define ECDSA_CFG_MASK          0xffff

#define DAC_CONF_RESET          0x0001fe17
#define RD_TIM_CONF_RESET       0x0f010201
#define WR_TIM_CONF1_RESET      0x01266701
#define WR_TIM_CONF2_RESET      0x00a00140
#define WR_TIM_CONF0_RESET      0x00002000
#define DATE_RESET              0x02502190

/* The controller's clock when nothing drives the clock input. */
#define EFUSE_CLK_HZ            20000000

/* EFUSE_STATE (ESP-IDF's efuse_controller_state_t). */
enum {
    STATE_RESET = 0,
    STATE_IDLE = 1,
    STATE_READ_INIT = 2,
    STATE_READ_BLK0 = 3,
    STATE_BLK0_CRC_CHECK = 4,
    STATE_READ_RS_BLK = 5,
    STATE_PGM = 6,
};

enum {
    OP_NONE,
    OP_READ,        /* EFUSE_READ_CMD */
    OP_LOAD,        /* the load as the controller leaves reset */
    OP_PGM,
};

/*
 * The array's 32-bit words in the order a read takes them: 20 of
 * EFUSE_WR_DIS and BLOCK0's copies, then 108 of the coded blocks.
 */
#define BLK0_ARRAY_WORDS        20
#define RS_ARRAY_WORDS          108

/* BLOCK0's last word holds 24 of the 152 bits; the rest do not exist. */
#define BLK0_W5_MASK            0x00ffffffu

/* The bits of BLOCK0 and of each coded block as stored. */
#define BLK0_BITS               (32 + ESP32P4_EFUSE_BLK0_BACKUPS * 152)
#define BLK1_BITS               ((24 + 12) * 8)
#define RS_BLK_BITS             ((32 + 12) * 8)

/* The backing file. */
#define FILE_DATA_WORDS         ESP32P4_EFUSE_RD_WORDS
#define FILE_BACKUP_WORDS       ((ESP32P4_EFUSE_BLK0_BACKUPS - 1) * 5)
#define FILE_CHECK_WORDS        ((ESP32P4_EFUSE_BLOCKS - 1) * 3)
#define FILE_WORDS              (FILE_DATA_WORDS + FILE_BACKUP_WORDS + \
                                 FILE_CHECK_WORDS)
#define FILE_DATA_SIZE          (FILE_DATA_WORDS * 4)
#define FILE_SIZE               (FILE_WORDS * 4)

/* EFUSE_RD_DIS: bits 6:0 of EFUSE_RD_REPEAT_DATA0, one per BLOCK4 to 10. */
#define RD_DIS_MASK             0x7f

/* The first RD_* word of each block, and its number of words. */
static const uint8_t rd_word_base[ESP32P4_EFUSE_BLOCKS] = {
    0, 6, 12, 20, 28, 36, 44, 52, 60, 68, 76,
};

static unsigned block_words(unsigned block)
{
    return block < 2 ? 6 : 8;
}

/* The block whose RD_* registers hold word `word`. */
static unsigned rd_word_block(unsigned word)
{
    unsigned block = ESP32P4_EFUSE_BLOCKS - 1;

    while (rd_word_base[block] > word) {
        block--;
    }
    return block;
}

/*
 * The bits of EFUSE_RD_REPEAT_DATA0..4 that each EFUSE_WR_DIS bit
 * protects, from esptool's esp32p4_v3.0.yaml (ESP-IDF's
 * esp_efuse_table_v3.0.csv agrees) and, for EFUSE_USB_PHY_SEL, TRM table
 * 9.4-1. EFUSE_WR_DIS itself, the recovery bootloader sector, secure boot
 * SHA-384 and the reserved bits are not protected by any.
 */
static const struct {
    uint8_t wr_dis_bit;
    uint8_t word;           /* 0 is EFUSE_RD_REPEAT_DATA0 */
    uint32_t mask;
} blk0_wr_dis_map[] = {
    { 0,  0, 0x0000007f },  /* RD_DIS */
    { 1,  1, 0x00007ff0 },  /* key manager */
    { 1,  2, 0x00c00000 },
    { 1,  3, 0x00000008 },
    { 1,  4, 0x00000f80 },
    { 2,  0, 0x0018fa00 },  /* JTAG, download, TWAI, ... */
    { 2,  1, 0x00020000 },
    { 2,  3, 0x04000000 },
    { 2,  4, 0x00300003 },
    { 3,  4, 0x00c00004 },  /* PVT glitch monitor */
    { 4,  1, 0x001c0000 },  /* SPI_BOOT_CRYPT_CNT */
    { 5,  1, 0x00200000 },  /* SECURE_BOOT_KEY_REVOKE0 */
    { 6,  1, 0x00400000 },
    { 7,  1, 0x00800000 },
    { 8,  1, 0x0f000000 },  /* KEY_PURPOSE_0 */
    { 8,  3, 0x08000000 },
    { 9,  1, 0xf0000000 },
    { 9,  3, 0x10000000 },
    { 10, 2, 0x0000000f },
    { 10, 3, 0x20000000 },
    { 11, 2, 0x000000f0 },
    { 11, 3, 0x40000000 },
    { 12, 2, 0x00000f00 },
    { 12, 3, 0x80000000 },
    { 13, 2, 0x0000f000 },
    { 13, 4, 0x00000010 },  /* KEY_PURPOSE_5 */
    { 14, 1, 0x00008000 },  /* DPA and constant-time ECC */
    { 14, 2, 0x000b0000 },
    { 14, 4, 0x00030000 },
    { 15, 2, 0x00100000 },  /* SECURE_BOOT_EN */
    { 16, 2, 0x00200000 },  /* SECURE_BOOT_AGGRESSIVE_REVOKE */
    { 17, 4, 0x00040000 },  /* HP_PWR_SRC_SEL */
    { 18, 2, 0xfc000000 },  /* flash, download modes, secure version */
    { 18, 3, 0x03fffff7 },
    { 19, 0, 0x7c000000 },  /* HUK_GEN_STATE */
    { 30, 0, 0x02000000 },  /* USB_PHY_SEL */
    { 31, 0, 0x00070000 },  /* SOFT_DIS_JTAG */
};

/* The fields EFUSE_RD_REPEAT_ERR0..4 report, per their register layout. */
static const uint32_t repeat_err_mask[ESP32P4_EFUSE_BLK0_WORDS - 1] = {
    0xffffffff, 0xfffeffff, 0xfcfbffff, 0xffffffff, 0x00ff0f97,
};

/*
 * Reed-Solomon RS(44, 32) over GF(2^8) with primitive polynomial
 * x^8 + x^4 + x^3 + x^2 + 1 (TRM section 9.4.3.1.3), generator roots
 * alpha^0 to alpha^11. Byte 0 of EFUSE_PGM_DATA0 is the codeword's first,
 * highest-order symbol, and parity byte 0 (byte 0 of
 * EFUSE_PGM_CHECK_VALUE0) the first parity symbol: the code of esptool's
 * reedsolo and of the ROM's ets_efuse_rs_calculate().
 */
#define RS_N        44
#define RS_K        32
#define RS_NSYM     12
#define RS_T        (RS_NSYM / 2)

static uint8_t gf_exp[512];
static uint8_t gf_log[256];
static uint8_t rs_gen[RS_NSYM + 1];

static uint8_t gf_mul(uint8_t a, uint8_t b)
{
    if (!a || !b) {
        return 0;
    }
    return gf_exp[gf_log[a] + gf_log[b]];
}

static uint8_t gf_div(uint8_t a, uint8_t b)
{
    if (!a) {
        return 0;
    }
    return gf_exp[(gf_log[a] + 255 - gf_log[b]) % 255];
}

static uint8_t gf_alpha_pow(int e)
{
    e %= 255;
    if (e < 0) {
        e += 255;
    }
    return gf_exp[e];
}

static void rs_init_tables(void)
{
    unsigned x = 1;
    int i, j;

    for (i = 0; i < 255; i++) {
        gf_exp[i] = x;
        gf_log[x] = i;
        x <<= 1;
        if (x & 0x100) {
            x ^= 0x11d;
        }
    }
    for (i = 255; i < 512; i++) {
        gf_exp[i] = gf_exp[i - 255];
    }

    /* g(x) = (x - a^0)...(x - a^11), highest-order coefficient first. */
    memset(rs_gen, 0, sizeof(rs_gen));
    rs_gen[0] = 1;
    for (i = 0; i < RS_NSYM; i++) {
        for (j = i + 1; j > 0; j--) {
            rs_gen[j] ^= gf_mul(rs_gen[j - 1], gf_exp[i]);
        }
    }
}

static void rs_words_to_bytes(const uint32_t *words, unsigned n, uint8_t *b)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        stl_le_p(b + 4 * i, words[i]);
    }
}

/* The parity of 8 data words (BLOCK1's last two zero). */
static void rs_parity(const uint32_t data[ESP32P4_EFUSE_RS_WORDS],
                      uint32_t check[ESP32P4_EFUSE_RS_CHECK_WORDS])
{
    uint8_t msg[RS_K + RS_NSYM] = { 0 };
    int i, j;

    rs_words_to_bytes(data, ESP32P4_EFUSE_RS_WORDS, msg);
    for (i = 0; i < RS_K; i++) {
        uint8_t coef = msg[i];

        if (coef) {
            for (j = 1; j <= RS_NSYM; j++) {
                msg[i + j] ^= gf_mul(rs_gen[j], coef);
            }
        }
    }
    for (i = 0; i < ESP32P4_EFUSE_RS_CHECK_WORDS; i++) {
        check[i] = ldl_le_p(msg + RS_K + 4 * i);
    }
}

/*
 * Correct codeword `cw` in place. Bytes from `virt_lo` to `virt_hi` are
 * not stored (BLOCK1's missing data, zero by definition), so an error
 * found there means the codeword cannot be corrected. Returns the number
 * of bytes corrected, or -1 if it cannot be corrected.
 */
static int rs_correct(uint8_t cw[RS_N], int virt_lo, int virt_hi)
{
    uint8_t synd[RS_NSYM];
    uint8_t lambda[RS_NSYM + 1] = { 1 };
    uint8_t prev[RS_NSYM + 1] = { 1 };
    uint8_t omega[RS_NSYM];
    int pos[RS_T];
    int nerr = 0, deg = 0, shift = 1;
    uint8_t prev_disc = 1;
    bool clean = true;
    int i, j, n;

    /* S_j = c(a^j), the codeword's first byte its x^43 coefficient. */
    for (j = 0; j < RS_NSYM; j++) {
        uint8_t s = 0;

        for (i = 0; i < RS_N; i++) {
            s = gf_mul(s, gf_exp[j]) ^ cw[i];
        }
        synd[j] = s;
        clean &= !s;
    }
    if (clean) {
        return 0;
    }

    /* Berlekamp-Massey: the error locator, lowest order first. */
    for (n = 0; n < RS_NSYM; n++) {
        uint8_t disc = synd[n];
        uint8_t tmp[RS_NSYM + 1];

        for (i = 1; i <= deg; i++) {
            disc ^= gf_mul(lambda[i], synd[n - i]);
        }
        if (!disc) {
            shift++;
            continue;
        }
        memcpy(tmp, lambda, sizeof(tmp));
        for (i = 0; i + shift <= RS_NSYM; i++) {
            lambda[i + shift] ^= gf_mul(gf_div(disc, prev_disc), prev[i]);
        }
        if (2 * deg <= n) {
            deg = n + 1 - deg;
            memcpy(prev, tmp, sizeof(prev));
            prev_disc = disc;
            shift = 1;
        } else {
            shift++;
        }
    }
    if (deg > RS_T) {
        return -1;
    }

    /* Chien search over the 44 positions, byte i being x^(43 - i). */
    for (i = 0; i < RS_N; i++) {
        uint8_t xinv = gf_alpha_pow(-(RS_N - 1 - i));
        uint8_t v = 0;

        for (j = deg; j >= 0; j--) {
            v = gf_mul(v, xinv) ^ lambda[j];
        }
        if (!v) {
            if (nerr == RS_T || (i >= virt_lo && i <= virt_hi)) {
                return -1;
            }
            pos[nerr++] = i;
        }
    }
    if (nerr != deg) {
        return -1;
    }

    /* Forney: Y = X * omega(1/X) / lambda'(1/X), for first root a^0. */
    for (i = 0; i < RS_NSYM; i++) {
        omega[i] = 0;
        for (j = 0; j <= i && j <= deg; j++) {
            omega[i] ^= gf_mul(synd[i - j], lambda[j]);
        }
    }
    for (n = 0; n < nerr; n++) {
        uint8_t x = gf_alpha_pow(RS_N - 1 - pos[n]);
        uint8_t xinv = gf_div(1, x);
        uint8_t num = 0, den = 0;

        for (j = RS_NSYM - 1; j >= 0; j--) {
            num = gf_mul(num, xinv) ^ omega[j];
        }
        for (j = deg; j >= 1; j--) {
            if (j & 1) {
                uint8_t term = lambda[j];
                int k;

                for (k = 1; k < j; k++) {
                    term = gf_mul(term, xinv);
                }
                den ^= term;
            }
        }
        if (!den) {
            return -1;
        }
        cw[pos[n]] ^= gf_mul(x, gf_div(num, den));
    }

    /* A miscorrection leaves a non-zero syndrome. */
    for (j = 0; j < RS_NSYM; j++) {
        uint8_t s = 0;

        for (i = 0; i < RS_N; i++) {
            s = gf_mul(s, gf_exp[j]) ^ cw[i];
        }
        if (s) {
            return -1;
        }
    }
    return nerr;
}

static bool block_read_protected(ESP32P4EfuseState *s, unsigned block)
{
    return block >= ESP32P4_EFUSE_KEY_BLOCK0 &&
           (s->rd[1] & RD_DIS_MASK & BIT(block - ESP32P4_EFUSE_KEY_BLOCK0));
}

static bool block_write_protected(ESP32P4EfuseState *s, unsigned block)
{
    return block >= 1 && block <= 10 && (s->rd[0] & BIT(19 + block));
}

/*
 * Load the RD_* registers and the error records from the array, as a
 * read operation ends.
 */
/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_load(ESP32P4EfuseState *s)
{
    unsigned b, w, k;

    s->rd[0] = s->wr_dis;
    for (w = 0; w < ESP32P4_EFUSE_BLK0_WORDS - 1; w++) {
        uint32_t any = 0, all = ~0u;

        for (k = 0; k < ESP32P4_EFUSE_BLK0_BACKUPS; k++) {
            any |= s->blk0[k][w];
            all &= s->blk0[k][w];
        }
        s->rd[1 + w] = any;
        s->repeat_err[w] = (any ^ all) & repeat_err_mask[w];
    }

    s->rs_err[0] = s->rs_err[1] = 0;
    for (b = 1; b < ESP32P4_EFUSE_BLOCKS; b++) {
        uint8_t cw[RS_N];
        uint32_t data[ESP32P4_EFUSE_RS_WORDS];
        unsigned slot = b - 1;
        uint32_t status;
        int fixed;

        rs_words_to_bytes(s->rs_data[b], ESP32P4_EFUSE_RS_WORDS, cw);
        rs_words_to_bytes(s->rs_check[b], ESP32P4_EFUSE_RS_CHECK_WORDS,
                          cw + RS_K);
        fixed = b == 1 ? rs_correct(cw, 24, 31) : rs_correct(cw, RS_N, RS_N);
        if (fixed < 0) {
            memcpy(data, s->rs_data[b], sizeof(data));
            status = BIT(3);
        } else {
            for (w = 0; w < ESP32P4_EFUSE_RS_WORDS; w++) {
                data[w] = ldl_le_p(cw + 4 * w);
            }
            status = fixed;
        }
        for (w = 0; w < block_words(b); w++) {
            s->rd[rd_word_base[b] + w] = data[w];
        }
        s->rs_err[slot / 8] |= status << (4 * (slot % 8));
    }
}

/* Reed-Solomon parity for every coded block's data. */
static void esp32p4_efuse_encode_all(ESP32P4EfuseState *s)
{
    unsigned b;

    for (b = 1; b < ESP32P4_EFUSE_BLOCKS; b++) {
        rs_parity(s->rs_data[b], s->rs_check[b]);
    }
}

/* Fields such as ADC1_AVE_INITCODE_ATTEN3 straddle two words. */
static void set_field(uint32_t *words, unsigned bit, unsigned width,
                      uint32_t value)
{
    while (width) {
        unsigned n = MIN(width, 32 - bit % 32);

        words[bit / 32] = deposit32(words[bit / 32], bit % 32, n, value);
        value >>= n;
        bit += n;
        width -= n;
    }
}

/* The array of a factory-fresh ESP32-P4NRW32X, as described above. */
/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_defaults(ESP32P4EfuseState *s)
{
    static const uint8_t mac[6] = { 0x24, 0x0a, 0xc4, 0x00, 0x00, 0x01 };
    static const uint16_t adc_initcode[2][4] = {
        { 400, 410, 420, 440 }, { 390, 400, 415, 430 },
    };
    static const uint16_t adc1_hi_dout[4] = { 30, 25, 20, 15 };
    static const uint16_t adc2_hi_dout[4] = { 28, 22, 18, 12 };
    static const uint8_t adc1_ch_diff[8] = { 1, 0, 2, 1, 0, 1, 2, 0 };
    static const uint8_t adc2_ch_diff[6] = { 1, 1, 0, 2, 1, 0 };
    uint32_t major = s->chip_revision / 100;
    uint32_t minor = s->chip_revision % 100;
    uint32_t *blk1 = s->rs_data[1];
    uint32_t *blk2 = s->rs_data[2];
    uint32_t *blk10 = s->rs_data[10];
    unsigned i;

    s->wr_dis = 0;
    memset(s->blk0, 0, sizeof(s->blk0));
    memset(s->rs_data, 0, sizeof(s->rs_data));

    /* BLOCK1: MAC, versions, package, PSRAM, LDO and PMU trims. */
    blk1[0] = ldl_be_p(mac + 2);
    blk1[1] = lduw_be_p(mac);
    set_field(blk1, 64, 4, minor);      /* WAFER_VERSION_MINOR */
    set_field(blk1, 68, 2, major);      /* WAFER_VERSION_MAJOR_LO */
    set_field(blk1, 72, 3, 1);          /* BLK_VERSION_MINOR */
    set_field(blk1, 75, 2, 1);          /* BLK_VERSION_MAJOR */
    set_field(blk1, 77, 3, 2);          /* PSRAM_CAP */
    set_field(blk1, 80, 2, 0);          /* TEMP */
    set_field(blk1, 82, 2, 1);          /* PSRAM_VENDOR */
    set_field(blk1, 84, 3, 0);          /* PKG_VERSION */
    set_field(blk1, 87, 1, major >> 2); /* WAFER_VERSION_MAJOR_HI */
    set_field(blk1, 88, 4, 8);          /* LDO_VO1_DREF */
    set_field(blk1, 92, 4, 8);          /* LDO_VO2_DREF */
    set_field(blk1, 96, 3, 4);          /* LDO_VO1_MUL */
    set_field(blk1, 99, 3, 4);          /* LDO_VO2_MUL */
    set_field(blk1, 102, 8, 25);        /* LDO_VO3_K: 975 + 25 */
    set_field(blk1, 110, 6, 3);         /* LDO_VO3_VOS: 3 - 3 */
    set_field(blk1, 116, 6, 10);        /* LDO_VO3_C: 990 + 10 */
    set_field(blk1, 122, 6, 25);        /* LDO_VO4_K, low 6 bits */
    set_field(blk1, 128, 2, 0);         /* LDO_VO4_K, high 2 bits */
    set_field(blk1, 130, 6, 3);         /* LDO_VO4_VOS */
    set_field(blk1, 136, 6, 10);        /* LDO_VO4_C */
    set_field(blk1, 144, 4, 8);         /* ACTIVE_HP_DBIAS: 8 + 16 */
    set_field(blk1, 148, 4, 9);         /* ACTIVE_LP_DBIAS: 9 + 16 + 4 */
    set_field(blk1, 165, 5, 8);         /* LP_DCDC_DBIAS_VOL_GAP: 8 - 8 */

    /* BLOCK2 and BLOCK10: ADC calibration and the temperature sensor. */
    for (i = 0; i < 4; i++) {
        set_field(blk2, 128 + 10 * i, 10, adc_initcode[0][i]);
        set_field(blk2, 168 + 10 * i, 10, adc_initcode[1][i]);
        set_field(blk2, 208 + 10 * i, 10, adc1_hi_dout[i]);
        set_field(blk10, 10 * i, 10, adc2_hi_dout[i]);
    }
    for (i = 0; i < 8; i++) {
        set_field(blk10, 40 + 4 * i, 4, adc1_ch_diff[i]);
    }
    for (i = 0; i < 6; i++) {
        set_field(blk10, 72 + 4 * i, 4, adc2_ch_diff[i]);
    }
    set_field(blk10, 96, 10, 2);        /* TEMPERATURE_SENSOR */

    esp32p4_efuse_encode_all(s);
}

/* The file image of the array (see the top of the file). */
static void esp32p4_efuse_to_file(ESP32P4EfuseState *s,
                                  uint32_t img[FILE_WORDS])
{
    unsigned b, w, k, n = 0;

    img[n++] = s->wr_dis;
    for (w = 0; w < 5; w++) {
        img[n++] = s->blk0[0][w];
    }
    for (b = 1; b < ESP32P4_EFUSE_BLOCKS; b++) {
        for (w = 0; w < block_words(b); w++) {
            img[n++] = s->rs_data[b][w];
        }
    }
    for (k = 1; k < ESP32P4_EFUSE_BLK0_BACKUPS; k++) {
        for (w = 0; w < 5; w++) {
            img[n++] = s->blk0[k][w] ^ s->blk0[0][w];
        }
    }
    for (b = 1; b < ESP32P4_EFUSE_BLOCKS; b++) {
        uint32_t parity[ESP32P4_EFUSE_RS_CHECK_WORDS];

        rs_parity(s->rs_data[b], parity);
        for (w = 0; w < ESP32P4_EFUSE_RS_CHECK_WORDS; w++) {
            img[n++] = s->rs_check[b][w] ^ parity[w];
        }
    }
    assert(n == FILE_WORDS);
    for (n = 0; n < FILE_WORDS; n++) {
        img[n] = cpu_to_le32(img[n]);
    }
}

/*
 * The array from a file image, whose differences past the data are taken
 * as zero if it has only `nwords`.
 */
static void esp32p4_efuse_from_file(ESP32P4EfuseState *s,
                                    const uint32_t *file, unsigned nwords)
{
    uint32_t img[FILE_WORDS] = { 0 };
    unsigned b, w, k, n = 0;

    for (n = 0; n < nwords; n++) {
        img[n] = le32_to_cpu(file[n]);
    }
    n = 0;
    s->wr_dis = img[n++];
    for (w = 0; w < 5; w++) {
        s->blk0[0][w] = img[n++];
    }
    s->blk0[0][4] &= BLK0_W5_MASK;
    memset(s->rs_data, 0, sizeof(s->rs_data));
    for (b = 1; b < ESP32P4_EFUSE_BLOCKS; b++) {
        for (w = 0; w < block_words(b); w++) {
            s->rs_data[b][w] = img[n++];
        }
    }
    for (k = 1; k < ESP32P4_EFUSE_BLK0_BACKUPS; k++) {
        for (w = 0; w < 5; w++) {
            s->blk0[k][w] = s->blk0[0][w] ^ img[n++];
        }
        s->blk0[k][4] &= BLK0_W5_MASK;
    }
    for (b = 1; b < ESP32P4_EFUSE_BLOCKS; b++) {
        rs_parity(s->rs_data[b], s->rs_check[b]);
        for (w = 0; w < ESP32P4_EFUSE_RS_CHECK_WORDS; w++) {
            s->rs_check[b][w] ^= img[n++];
        }
    }
}

/* Write the array to the backing file. */
static void esp32p4_efuse_save(ESP32P4EfuseState *s)
{
    uint32_t img[FILE_WORDS];
    int ret;

    if (!s->blk) {
        return;
    }
    esp32p4_efuse_to_file(s, img);
    ret = blk_pwrite(s->blk, 0, s->blk_has_raw ? FILE_SIZE : FILE_DATA_SIZE,
                     img, 0);
    if (ret < 0) {
        error_report("%s: cannot write the eFuse file: %s",
                     TYPE_ESP32P4_EFUSE, strerror(-ret));
    }
}

static void esp32p4_efuse_update_irq(ESP32P4EfuseState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

/* Cycles the running operation has run, up to now. */
static uint64_t esp32p4_efuse_cycles_run(ESP32P4EfuseState *s)
{
    uint64_t run = s->op_cycles_done +
                   esp32p4_clk_count_at(&s->op_edges,
                                        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

    return MIN(run, s->op_cycles);
}

/* Arm the timer for the rest of the operation, if the clock runs. */
static void esp32p4_efuse_op_schedule(ESP32P4EfuseState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    esp32p4_clk_count_start(&s->op_edges, s->clk, now);
    if (!esp32p4_clk_count_running(&s->op_edges)) {
        timer_del(&s->op_timer);
        return;
    }
    timer_mod_ns(&s->op_timer,
                 esp32p4_clk_count_time(&s->op_edges,
                                        s->op_cycles - s->op_cycles_done));
}

/* The cycles each word of the array takes to read. */
static uint32_t esp32p4_efuse_word_read_cycles(ESP32P4EfuseState *s)
{
    uint32_t t = s->rd_tim_conf_used;

    return extract32(t, 0, 8) + extract32(t, 8, 8) + extract32(t, 16, 8);
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_start_read(ESP32P4EfuseState *s, uint32_t op)
{
    uint64_t word = esp32p4_efuse_word_read_cycles(s);

    s->op = op;
    s->op_cycles = extract32(s->rd_tim_conf_used, 24, 8) +
                   (BLK0_ARRAY_WORDS + RS_ARRAY_WORDS) * word + 1;
    s->op_cycles_done = 0;
    esp32p4_efuse_op_schedule(s);
}

/*
 * The bits of BLOCK0's words EFUSE_WR_DIS protects, from the parameters
 * as last loaded. `mask` has a word for EFUSE_WR_DIS and each of
 * EFUSE_RD_REPEAT_DATA0..4.
 */
static void esp32p4_efuse_blk0_protected(ESP32P4EfuseState *s,
                                         uint32_t mask[6])
{
    unsigned i;

    memset(mask, 0, 6 * sizeof(uint32_t));
    for (i = 0; i < ARRAY_SIZE(blk0_wr_dis_map); i++) {
        if (s->rd[0] & BIT(blk0_wr_dis_map[i].wr_dis_bit)) {
            mask[1 + blk0_wr_dis_map[i].word] |= blk0_wr_dis_map[i].mask;
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_start_pgm(ESP32P4EfuseState *s, unsigned block)
{
    uint32_t tim1 = s->wr_tim_conf1_used, tim2 = s->wr_tim_conf2_used;
    uint32_t pulse = extract32(tim1, 0, 8) + extract32(tim2, 16, 16) +
                     extract32(tim1, 24, 8);
    uint32_t inactive = extract32(s->wr_tim_conf0_used, 13, 8);
    uint32_t dac_cycles = extract32(s->dac_conf_used, 0, 8) *
                          extract32(s->dac_conf_used, 9, 8);
    uint32_t bits, ones = 0;
    unsigned i;

    s->op = OP_PGM;
    s->op_blk = block;
    memset(s->op_pgm, 0, sizeof(s->op_pgm));

    if (block == 0) {
        uint32_t prot[6];

        esp32p4_efuse_blk0_protected(s, prot);
        for (i = 0; i < 6; i++) {
            s->op_pgm[i] = s->pgm_data[i] & ~prot[i];
        }
        s->op_pgm[5] &= BLK0_W5_MASK;
        ones = ctpop32(s->op_pgm[0]);
        for (i = 1; i < 6; i++) {
            ones += ESP32P4_EFUSE_BLK0_BACKUPS * ctpop32(s->op_pgm[i]);
        }
        bits = BLK0_BITS;
    } else if (block < ESP32P4_EFUSE_BLOCKS) {
        if (!block_write_protected(s, block)) {
            memcpy(s->op_pgm, s->pgm_data, sizeof(s->pgm_data));
            memcpy(s->op_pgm + ESP32P4_EFUSE_RS_WORDS, s->pgm_check,
                   sizeof(s->pgm_check));
            if (block == 1) {
                s->op_pgm[6] = s->op_pgm[7] = 0;
            }
        }
        for (i = 0; i < ARRAY_SIZE(s->op_pgm); i++) {
            ones += ctpop32(s->op_pgm[i]);
        }
        bits = block == 1 ? BLK1_BITS : RS_BLK_BITS;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: programming block %u, which "
                      "does not exist\n", TYPE_ESP32P4_EFUSE, block);
        bits = 0;
    }

    if (extract32(tim1, 8, 16) <= dac_cycles) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: EFUSE_PWR_ON_NUM (%u) does not "
                      "exceed EFUSE_DAC_CLK_DIV x EFUSE_DAC_NUM (%u): VDDQ "
                      "has not reached the programming voltage\n",
                      TYPE_ESP32P4_EFUSE, extract32(tim1, 8, 16), dac_cycles);
    }

    s->op_cycles = (uint64_t)extract32(tim1, 8, 16) +
                   (uint64_t)ones * pulse + (uint64_t)(bits - ones) * inactive +
                   extract32(tim2, 0, 16);
    s->op_cycles = MAX(s->op_cycles, 1);
    s->op_cycles_done = 0;
    esp32p4_efuse_op_schedule(s);
}

/* Burn what the programming operation took, as it ends. */
/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_burn(ESP32P4EfuseState *s)
{
    unsigned b = s->op_blk, i, k;

    if (b == 0) {
        s->wr_dis |= s->op_pgm[0];
        for (k = 0; k < ESP32P4_EFUSE_BLK0_BACKUPS; k++) {
            for (i = 0; i < 5; i++) {
                s->blk0[k][i] |= s->op_pgm[1 + i];
            }
        }
    } else if (b < ESP32P4_EFUSE_BLOCKS) {
        for (i = 0; i < ESP32P4_EFUSE_RS_WORDS; i++) {
            s->rs_data[b][i] |= s->op_pgm[i];
        }
        for (i = 0; i < ESP32P4_EFUSE_RS_CHECK_WORDS; i++) {
            s->rs_check[b][i] |= s->op_pgm[ESP32P4_EFUSE_RS_WORDS + i];
        }
    } else {
        return;
    }
    esp32p4_efuse_save(s);
}

static void esp32p4_efuse_op_done(void *opaque)
{
    ESP32P4EfuseState *s = opaque;
    uint32_t op = s->op;

    s->op = OP_NONE;
    s->op_cycles = s->op_cycles_done = 0;
    switch (op) {
    case OP_READ:
        esp32p4_efuse_load(s);
        s->int_raw |= INT_READ_DONE;
        break;
    case OP_LOAD:
        esp32p4_efuse_load(s);
        break;
    case OP_PGM:
        esp32p4_efuse_burn(s);
        s->int_raw |= INT_PGM_DONE;
        break;
    default:
        return;
    }
    s->cmd = 0;
    esp32p4_efuse_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.efuse]
 * End the running read or programming if its cycles have run by the
 * time of the access being made, at the time they ran: without icount
 * its timer can run well after that, while software polls EFUSE_CMD's
 * READ_CMD or PGM_CMD, INT_RAW and the RD_ registers the read loads, as
 * the ROM and ESP-IDF do.
 */
static void esp32p4_efuse_catch_up(ESP32P4EfuseState *s)
{
    int64_t due = timer_expire_time_ns(&s->op_timer);

    if (timer_pending(&s->op_timer) && due <= esp32_line_time()) {
        timer_del(&s->op_timer);
        esp32_line_run_at(due, esp32p4_efuse_op_done, s);
    }
}

/* EFUSE_STATE, from how far the running operation has got. */
static uint32_t esp32p4_efuse_state(ESP32P4EfuseState *s)
{
    uint64_t run, init, blk0;

    if (s->in_reset) {
        return STATE_RESET;
    }
    switch (s->op) {
    case OP_PGM:
        return STATE_PGM;
    case OP_READ:
    case OP_LOAD:
        run = esp32p4_efuse_cycles_run(s);
        init = extract32(s->rd_tim_conf_used, 24, 8);
        blk0 = init + BLK0_ARRAY_WORDS *
               (uint64_t)esp32p4_efuse_word_read_cycles(s);
        if (run < init) {
            return STATE_READ_INIT;
        } else if (run < blk0) {
            return STATE_READ_BLK0;
        } else if (run == blk0) {
            return STATE_BLK0_CRC_CHECK;
        }
        return STATE_READ_RS_BLK;
    default:
        return STATE_IDLE;
    }
}

/* The array word APB2OTP register `addr` reads. */
/* [spec:nuos:req:emu.esp32p4.efuse] */
static uint32_t esp32p4_efuse_apb2otp_read(ESP32P4EfuseState *s,
                                           hwaddr addr)
{
    unsigned idx = (addr - A_APB2OTP_WR_DIS) / 4;
    unsigned block, w;

    if (!(s->apb2otp_en & 1)) {
        return 0;
    }
    if (idx == 0) {
        return s->wr_dis;
    }
    idx--;
    /* BLOCK0's copies, five words each. */
    if (idx < ESP32P4_EFUSE_BLK0_BACKUPS * 5) {
        return s->blk0[idx / 5][idx % 5];
    }
    idx -= ESP32P4_EFUSE_BLK0_BACKUPS * 5;
    /* BLOCK1's six data words and BLOCK2 to 10's eight, then parity. */
    if (idx < 6 + ESP32P4_EFUSE_RS_CHECK_WORDS) {
        block = 1;
        w = idx < 6 ? idx : ESP32P4_EFUSE_RS_WORDS + idx - 6;
    } else {
        idx -= 6 + ESP32P4_EFUSE_RS_CHECK_WORDS;
        block = 2 + idx / (ESP32P4_EFUSE_RS_WORDS +
                           ESP32P4_EFUSE_RS_CHECK_WORDS);
        w = idx % (ESP32P4_EFUSE_RS_WORDS + ESP32P4_EFUSE_RS_CHECK_WORDS);
    }
    if (block_read_protected(s, block)) {
        return 0;
    }
    if (w < ESP32P4_EFUSE_RS_WORDS) {
        return s->rs_data[block][w];
    }
    return s->rs_check[block][w - ESP32P4_EFUSE_RS_WORDS];
}

static bool esp32p4_efuse_is_rd_reg(hwaddr addr)
{
    return addr >= A_RD_WR_DIS &&
           addr < A_RD_WR_DIS + ESP32P4_EFUSE_RD_WORDS * 4;
}

static uint32_t esp32p4_efuse_read_reg(ESP32P4EfuseState *s, hwaddr addr)
{
    if (addr < A_PGM_CHECK_VALUE0) {
        return s->pgm_data[addr / 4];
    }
    if (addr < A_RD_WR_DIS) {
        return s->pgm_check[(addr - A_PGM_CHECK_VALUE0) / 4];
    }
    if (esp32p4_efuse_is_rd_reg(addr)) {
        unsigned word = (addr - A_RD_WR_DIS) / 4;

        if (block_read_protected(s, rd_word_block(word))) {
            return 0;
        }
        return s->rd[word];
    }
    if (addr >= A_RD_REPEAT_ERR0 && addr <= A_RD_REPEAT_ERR4) {
        return s->repeat_err[(addr - A_RD_REPEAT_ERR0) / 4];
    }
    if (addr >= A_APB2OTP_WR_DIS && addr <= A_APB2OTP_BLK10_W11) {
        return esp32p4_efuse_apb2otp_read(s, addr);
    }

    switch (addr) {
    case A_ECDSA:
        return s->ecdsa | (s->ecdsa << 16);
    case A_RD_RS_ERR0:
        return s->rs_err[0];
    case A_RD_RS_ERR1:
        return s->rs_err[1];
    case A_CLK:
        return s->clk_reg;
    case A_CONF:
        return s->conf;
    case A_STATUS:
        return esp32p4_efuse_state(s);
    case A_CMD:
        return s->cmd;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_DAC_CONF:
        return s->dac_conf;
    case A_RD_TIM_CONF:
        return s->rd_tim_conf;
    case A_WR_TIM_CONF1:
        return s->wr_tim_conf1;
    case A_WR_TIM_CONF2:
        return s->wr_tim_conf2;
    case A_WR_TIM_CONF0_RS_BYPASS:
        return s->wr_tim_conf0;
    case A_DATE:
        return s->date;
    case A_APB2OTP_EN:
        return s->apb2otp_en;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unassigned offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_EFUSE, addr);
        return 0;
    }
}

static uint64_t esp32p4_efuse_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4EfuseState *s = opaque;
    uint32_t value;

    esp32p4_efuse_catch_up(s);
    value = esp32p4_efuse_read_reg(s, addr & ~3);

    return extract32(value, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_write_cmd(ESP32P4EfuseState *s, uint32_t value)
{
    uint32_t op_code = s->conf & 0xffff;

    if (s->op != OP_NONE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: EFUSE_CMD written while an "
                      "operation runs\n", TYPE_ESP32P4_EFUSE);
        return;
    }
    s->cmd = value & CMD_MASK;
    if ((s->cmd & CMD_READ) && op_code == READ_OP_CODE) {
        if (s->cmd & CMD_PGM) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: EFUSE_PGM_CMD ignored with "
                          "the read EFUSE_OP_CODE\n", TYPE_ESP32P4_EFUSE);
            s->cmd &= ~CMD_PGM;
        }
        esp32p4_efuse_start_read(s, OP_READ);
    } else if ((s->cmd & CMD_PGM) && op_code == PGM_OP_CODE) {
        if (s->cmd & CMD_READ) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: EFUSE_READ_CMD ignored with "
                          "the programming EFUSE_OP_CODE\n",
                          TYPE_ESP32P4_EFUSE);
            s->cmd &= ~CMD_READ;
        }
        esp32p4_efuse_start_pgm(s, s->cmd >> CMD_BLK_NUM_SHIFT);
    } else if (s->cmd & (CMD_READ | CMD_PGM)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: EFUSE_CMD 0x%x refused: "
                      "EFUSE_OP_CODE is 0x%x\n", TYPE_ESP32P4_EFUSE,
                      s->cmd, op_code);
        s->cmd &= ~(CMD_READ | CMD_PGM);
    }
}

/* The state machine's timing takes the registers' values. */
static void esp32p4_efuse_update_timing(ESP32P4EfuseState *s)
{
    s->dac_conf_used = s->dac_conf;
    s->rd_tim_conf_used = s->rd_tim_conf;
    s->wr_tim_conf1_used = s->wr_tim_conf1;
    s->wr_tim_conf2_used = s->wr_tim_conf2;
    s->wr_tim_conf0_used = s->wr_tim_conf0;
}

/*
 * A write of `value` to the word at `addr`, `mask` the bytes it writes;
 * `merged` is the word with those bytes replaced, for registers that
 * keep the bytes not written.
 */
static void esp32p4_efuse_write_reg(ESP32P4EfuseState *s, hwaddr addr,
                                    uint32_t value, uint32_t mask)
{
    uint32_t merged;

    if (addr < A_PGM_CHECK_VALUE0) {
        s->pgm_data[addr / 4] = (s->pgm_data[addr / 4] & ~mask) | value;
        return;
    }
    if (addr < A_RD_WR_DIS) {
        unsigned i = (addr - A_PGM_CHECK_VALUE0) / 4;

        s->pgm_check[i] = (s->pgm_check[i] & ~mask) | value;
        return;
    }
    if (esp32p4_efuse_is_rd_reg(addr) ||
        (addr >= A_RD_REPEAT_ERR0 && addr <= A_RD_REPEAT_ERR4) ||
        (addr >= A_APB2OTP_WR_DIS && addr <= A_APB2OTP_BLK10_W11) ||
        addr == A_RD_RS_ERR0 || addr == A_RD_RS_ERR1 || addr == A_STATUS ||
        addr == A_INT_ST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_EFUSE, addr);
        return;
    }

    switch (addr) {
    case A_ECDSA:
        merged = (s->ecdsa & ~mask) | value;
        s->ecdsa = merged & ECDSA_CFG_MASK;
        break;
    case A_CLK:
        merged = (s->clk_reg & ~mask) | value;
        s->clk_reg = merged & CLK_MASK;
        if (s->clk_reg & CLK_MEM_FORCE_PD) {
            qemu_log_mask(LOG_UNIMP, "%s: EFUSE_MEM_FORCE_PD is not "
                          "modelled\n", TYPE_ESP32P4_EFUSE);
        }
        break;
    case A_CONF:
        s->conf = ((s->conf & ~mask) | value) & 0xffff;
        break;
    case A_CMD:
        esp32p4_efuse_write_cmd(s, (s->cmd & ~mask) | value);
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(value & INT_MASK);
        esp32p4_efuse_update_irq(s);
        break;
    case A_INT_ENA:
        s->int_ena = ((s->int_ena & ~mask) | value) & INT_MASK;
        esp32p4_efuse_update_irq(s);
        break;
    case A_DAC_CONF:
        s->dac_conf = ((s->dac_conf & ~mask) | value) & DAC_CONF_MASK;
        break;
    case A_RD_TIM_CONF:
        s->rd_tim_conf = (s->rd_tim_conf & ~mask) | value;
        break;
    case A_WR_TIM_CONF1:
        s->wr_tim_conf1 = (s->wr_tim_conf1 & ~mask) | value;
        break;
    case A_WR_TIM_CONF2:
        s->wr_tim_conf2 = (s->wr_tim_conf2 & ~mask) | value;
        break;
    case A_WR_TIM_CONF0_RS_BYPASS:
        merged = (s->wr_tim_conf0 & ~mask) | value;
        s->wr_tim_conf0 = merged & RS_BYPASS_MASK;
        if (s->wr_tim_conf0 & RS_BYPASS_CORRECTION) {
            qemu_log_mask(LOG_UNIMP, "%s: EFUSE_BYPASS_RS_CORRECTION is not "
                          "modelled\n", TYPE_ESP32P4_EFUSE);
        }
        if (value & RS_BYPASS_UPDATE) {
            esp32p4_efuse_update_timing(s);
        }
        break;
    case A_DATE:
        s->date = ((s->date & ~mask) | value) & DATE_MASK;
        break;
    case A_APB2OTP_EN:
        s->apb2otp_en = ((s->apb2otp_en & ~mask) | value) & 1;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unassigned offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_EFUSE, addr);
        break;
    }
}

static void esp32p4_efuse_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    ESP32P4EfuseState *s = opaque;
    unsigned shift = (addr & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);

    esp32p4_efuse_catch_up(s);
    if (s->in_reset) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while the controller is held in reset\n",
                      TYPE_ESP32P4_EFUSE, addr);
        return;
    }
    esp32p4_efuse_write_reg(s, addr & ~3, (value << shift) & mask, mask);
}

static const MemoryRegionOps esp32p4_efuse_ops = {
    .read = esp32p4_efuse_read,
    .write = esp32p4_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* The controller's registers at reset, the array not yet loaded. */
static void esp32p4_efuse_reset_registers(ESP32P4EfuseState *s)
{
    timer_del(&s->op_timer);
    s->op = OP_NONE;
    s->op_cycles = s->op_cycles_done = 0;
    s->op_blk = 0;
    memset(s->op_pgm, 0, sizeof(s->op_pgm));
    memset(s->rd, 0, sizeof(s->rd));
    memset(s->repeat_err, 0, sizeof(s->repeat_err));
    memset(s->rs_err, 0, sizeof(s->rs_err));
    memset(s->pgm_data, 0, sizeof(s->pgm_data));
    memset(s->pgm_check, 0, sizeof(s->pgm_check));
    s->clk_reg = 0;
    s->conf = 0;
    s->cmd = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->dac_conf = DAC_CONF_RESET;
    s->rd_tim_conf = RD_TIM_CONF_RESET;
    s->wr_tim_conf1 = WR_TIM_CONF1_RESET;
    s->wr_tim_conf2 = WR_TIM_CONF2_RESET;
    s->wr_tim_conf0 = WR_TIM_CONF0_RESET;
    s->date = DATE_RESET;
    s->ecdsa = 0;
    s->apb2otp_en = 0;
    esp32p4_efuse_update_timing(s);
    esp32p4_efuse_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_reset_hold(Object *obj, ResetType type)
{
    ESP32P4EfuseState *s = ESP32P4_EFUSE(obj);

    esp32p4_efuse_reset_registers(s);
    if (!s->in_reset) {
        esp32p4_efuse_load(s);
    }
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_set_reset(void *opaque, int n, int level)
{
    ESP32P4EfuseState *s = opaque;

    if (!!level == s->in_reset) {
        return;
    }
    s->in_reset = level;
    esp32p4_efuse_reset_registers(s);
    if (!level) {
        esp32p4_efuse_start_read(s, OP_LOAD);
    }
}

/*
 * The clock changes: a running operation counts the cycles it ran at the
 * old rate, and runs the rest at the new one, or stalls.
 */
static void esp32p4_efuse_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4EfuseState *s = opaque;

    if (event == ClockPreUpdate) {
        esp32p4_efuse_catch_up(s);
    }
    if (s->op == OP_NONE) {
        return;
    }
    if (event == ClockPreUpdate) {
        s->op_cycles_done = esp32p4_efuse_cycles_run(s);
        timer_del(&s->op_timer);
    } else {
        esp32p4_efuse_op_schedule(s);
    }
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_efuse_realize(DeviceState *dev, Error **errp)
{
    ESP32P4EfuseState *s = ESP32P4_EFUSE(dev);
    uint32_t file[FILE_WORDS];
    int64_t len;
    unsigned i;
    int ret;

    if (!clock_has_source(s->clk)) {
        clock_set_hz(s->clk, EFUSE_CLK_HZ);
    }
    if (s->chip_revision / 100 > 7 || s->chip_revision % 100 > 15) {
        error_setg(errp, "%s: chip-revision %u is not a wafer version the "
                   "eFuses can hold (major 0 to 7, minor 0 to 15, as major "
                   "* 100 + minor)", TYPE_ESP32P4_EFUSE, s->chip_revision);
        return;
    }

    if (!s->blk) {
        esp32p4_efuse_defaults(s);
        return;
    }
    if (!blk_supports_write_perm(s->blk)) {
        error_setg(errp, "%s: the eFuse file must be writable",
                   TYPE_ESP32P4_EFUSE);
        return;
    }
    ret = blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                       BLK_PERM_ALL, errp);
    if (ret < 0) {
        return;
    }
    /* The length in whole 512-byte sectors, not the file's own. */
    len = blk_getlength(s->blk);
    if (len < 0) {
        error_setg_errno(errp, -len, "%s: cannot size the eFuse file",
                         TYPE_ESP32P4_EFUSE);
        return;
    }
    if (len < FILE_DATA_SIZE) {
        error_setg(errp, "%s: the eFuse file is empty; give it at least %d "
                   "bytes, or more than 512 to keep the whole array (as "
                   "'qemu-img create -f raw efuse.bin 1K' does)",
                   TYPE_ESP32P4_EFUSE, FILE_DATA_SIZE);
        return;
    }
    s->blk_has_raw = len >= FILE_SIZE;

    memset(file, 0, sizeof(file));
    ret = blk_pread(s->blk, 0, s->blk_has_raw ? FILE_SIZE : FILE_DATA_SIZE,
                    file, 0);
    if (ret < 0) {
        error_setg_errno(errp, -ret, "%s: cannot read the eFuse file",
                         TYPE_ESP32P4_EFUSE);
        return;
    }
    for (i = 0; i < FILE_WORDS && !file[i]; i++) {
    }
    if (i == FILE_WORDS) {
        esp32p4_efuse_defaults(s);
        esp32p4_efuse_save(s);
    } else {
        esp32p4_efuse_from_file(s, file, s->blk_has_raw ? FILE_WORDS
                                                        : FILE_DATA_WORDS);
    }
}

static void esp32p4_efuse_init(Object *obj)
{
    ESP32P4EfuseState *s = ESP32P4_EFUSE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_efuse_ops, s,
                          TYPE_ESP32P4_EFUSE, ESP32P4_EFUSE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(dev, "clk", esp32p4_efuse_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_in_named(dev, esp32p4_efuse_set_reset,
                            ESP32P4_EFUSE_RESET, 1);
    timer_init_ns(&s->op_timer, QEMU_CLOCK_VIRTUAL, esp32p4_efuse_op_done, s);
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
uint32_t esp32p4_efuse_word(ESP32P4EfuseState *s, unsigned block,
                            unsigned word)
{
    assert(block < ESP32P4_EFUSE_BLOCKS && word < block_words(block));
    esp32p4_efuse_catch_up(s);
    return s->rd[rd_word_base[block] + word];
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
uint32_t esp32p4_efuse_key_purpose(ESP32P4EfuseState *s, unsigned block)
{
    /* KEY_PURPOSE_n[3:0], then its fifth bit KEY_PURPOSE_n_H. */
    static const struct {
        uint8_t word, bit, hi_word, hi_bit;
    } field[ESP32P4_EFUSE_KEY_BLOCKS] = {
        { 2, 24, 4, 27 }, { 2, 28, 4, 28 }, { 3, 0, 4, 29 },
        { 3, 4, 4, 30 },  { 3, 8, 4, 31 },  { 3, 12, 5, 4 },
    };
    unsigned k = block - ESP32P4_EFUSE_KEY_BLOCK0;

    assert(k < ESP32P4_EFUSE_KEY_BLOCKS);
    esp32p4_efuse_catch_up(s);
    return extract32(s->rd[field[k].word], field[k].bit, 4) |
           extract32(s->rd[field[k].hi_word], field[k].hi_bit, 1) << 4;
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
void esp32p4_efuse_key(ESP32P4EfuseState *s, unsigned block,
                       uint8_t key[32])
{
    unsigned w;

    assert(block >= ESP32P4_EFUSE_KEY_BLOCK0 &&
           block < ESP32P4_EFUSE_KEY_BLOCK0 + ESP32P4_EFUSE_KEY_BLOCKS);
    esp32p4_efuse_catch_up(s);
    for (w = 0; w < ESP32P4_EFUSE_RS_WORDS; w++) {
        stl_le_p(key + 4 * w, s->rd[rd_word_base[block] + w]);
    }
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
int esp32p4_efuse_find_purpose(ESP32P4EfuseState *s, uint32_t purpose)
{
    unsigned b;

    for (b = ESP32P4_EFUSE_KEY_BLOCK0;
         b < ESP32P4_EFUSE_KEY_BLOCK0 + ESP32P4_EFUSE_KEY_BLOCKS; b++) {
        if (esp32p4_efuse_key_purpose(s, b) == purpose) {
            return b;
        }
    }
    return -1;
}

static const VMStateDescription vmstate_esp32p4_efuse = {
    .name = TYPE_ESP32P4_EFUSE,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(wr_dis, ESP32P4EfuseState),
        VMSTATE_UINT32_2DARRAY(blk0, ESP32P4EfuseState,
                               ESP32P4_EFUSE_BLK0_BACKUPS,
                               ESP32P4_EFUSE_BLK0_WORDS - 1),
        VMSTATE_UINT32_2DARRAY(rs_data, ESP32P4EfuseState,
                               ESP32P4_EFUSE_BLOCKS, ESP32P4_EFUSE_RS_WORDS),
        VMSTATE_UINT32_2DARRAY(rs_check, ESP32P4EfuseState,
                               ESP32P4_EFUSE_BLOCKS,
                               ESP32P4_EFUSE_RS_CHECK_WORDS),
        VMSTATE_UINT32_ARRAY(rd, ESP32P4EfuseState, ESP32P4_EFUSE_RD_WORDS),
        VMSTATE_UINT32_ARRAY(repeat_err, ESP32P4EfuseState,
                             ESP32P4_EFUSE_BLK0_WORDS - 1),
        VMSTATE_UINT32_ARRAY(rs_err, ESP32P4EfuseState, 2),
        VMSTATE_UINT32_ARRAY(pgm_data, ESP32P4EfuseState,
                             ESP32P4_EFUSE_RS_WORDS),
        VMSTATE_UINT32_ARRAY(pgm_check, ESP32P4EfuseState,
                             ESP32P4_EFUSE_RS_CHECK_WORDS),
        VMSTATE_UINT32(clk_reg, ESP32P4EfuseState),
        VMSTATE_UINT32(conf, ESP32P4EfuseState),
        VMSTATE_UINT32(cmd, ESP32P4EfuseState),
        VMSTATE_UINT32(int_raw, ESP32P4EfuseState),
        VMSTATE_UINT32(int_ena, ESP32P4EfuseState),
        VMSTATE_UINT32(dac_conf, ESP32P4EfuseState),
        VMSTATE_UINT32(rd_tim_conf, ESP32P4EfuseState),
        VMSTATE_UINT32(wr_tim_conf1, ESP32P4EfuseState),
        VMSTATE_UINT32(wr_tim_conf2, ESP32P4EfuseState),
        VMSTATE_UINT32(wr_tim_conf0, ESP32P4EfuseState),
        VMSTATE_UINT32(date, ESP32P4EfuseState),
        VMSTATE_UINT32(ecdsa, ESP32P4EfuseState),
        VMSTATE_UINT32(apb2otp_en, ESP32P4EfuseState),
        VMSTATE_UINT32(dac_conf_used, ESP32P4EfuseState),
        VMSTATE_UINT32(rd_tim_conf_used, ESP32P4EfuseState),
        VMSTATE_UINT32(wr_tim_conf1_used, ESP32P4EfuseState),
        VMSTATE_UINT32(wr_tim_conf2_used, ESP32P4EfuseState),
        VMSTATE_UINT32(wr_tim_conf0_used, ESP32P4EfuseState),
        VMSTATE_UINT32(op, ESP32P4EfuseState),
        VMSTATE_UINT64(op_cycles, ESP32P4EfuseState),
        VMSTATE_UINT64(op_cycles_done, ESP32P4EfuseState),
        VMSTATE_ESP32P4_CLK_COUNT(op_edges, ESP32P4EfuseState),
        VMSTATE_UINT32_ARRAY(op_pgm, ESP32P4EfuseState,
                             ESP32P4_EFUSE_RS_WORDS +
                             ESP32P4_EFUSE_RS_CHECK_WORDS),
        VMSTATE_UINT32(op_blk, ESP32P4EfuseState),
        VMSTATE_BOOL(in_reset, ESP32P4EfuseState),
        VMSTATE_TIMER(op_timer, ESP32P4EfuseState),
        VMSTATE_CLOCK(clk, ESP32P4EfuseState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_efuse_properties[] = {
    DEFINE_PROP_DRIVE("drive", ESP32P4EfuseState, blk),
    DEFINE_PROP_UINT32("chip-revision", ESP32P4EfuseState, chip_revision,
                       300),
};

static void esp32p4_efuse_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rs_init_tables();
    rc->phases.hold = esp32p4_efuse_reset_hold;
    dc->realize = esp32p4_efuse_realize;
    dc->vmsd = &vmstate_esp32p4_efuse;
    device_class_set_props(dc, esp32p4_efuse_properties);
}

/* [spec:nuos:req:emu.esp32p4.efuse] */
static const TypeInfo esp32p4_efuse_info = {
    .name          = TYPE_ESP32P4_EFUSE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4EfuseState),
    .instance_init = esp32p4_efuse_init,
    .class_init    = esp32p4_efuse_class_init,
};

static void esp32p4_efuse_register_types(void)
{
    type_register_static(&esp32p4_efuse_info);
}
type_init(esp32p4_efuse_register_types)
