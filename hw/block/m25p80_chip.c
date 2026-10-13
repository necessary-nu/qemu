/*
 * M25P80 datasheet chips: SPI NOR parts modelled as their datasheets give
 * them, with their status registers, command sets, busy times, suspend
 * and resume
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * References: Winbond W25Q32RV datasheet (revision E, November 2025),
 * W25Q16JV datasheet (revision G, March 2018), GigaDevice GD25Q256E
 * datasheet, GD25Q32C datasheet (revision 4.1); the parts are the
 * Raspberry Pi Pico 2's W25Q32RVXHJQ, the Waveshare RP2350-USB-C's
 * W25Q16JVUXIQ, the Waveshare ESP32-P4-WIFI6-POE-ETH's GD25Q256EYIGR and
 * the ESP32-WROOM-32E's 4 MiB GD25Q32C.
 *
 * The M25P80 model hands the chip entries that name one of these parts
 * (w25q16jv, w25q32rv, gd25q256e, gd25q32c) to this file, which replaces
 * its generic command decoder for them; its other entries behave as
 * before. The bus is QEMU's byte-wide SSI bus, so a phase of dummy clocks
 * counts as the bytes those clocks carry on the phase's lines, and the
 * line count of a command (SPI, dual, quad, QPI) is not seen.
 *
 * Commands and registers, per part, as the datasheets' instruction tables
 * give them: the three status registers with their volatile and
 * non-volatile bits (50h then 01h/31h/11h writes the volatile ones at
 * once; 06h then the same writes the stored ones in tW), status register
 * protection (SRP, SRL or SRP1/SRP0 with /WP while QE=0), the block
 * protection of BP/TB/SEC/CMP (Winbond) and BP4-BP0 (GigaDevice), the
 * W25Q16JV's individual block locks (WPS, 36h/39h/3Dh/7Eh/98h), the
 * security registers (3 of 256 bytes, 1024 on the GD25Q32C or 2048 on the
 * GD25Q256E, with their OTP lock bits), the unique ID (64 bits, 128 on the
 * GigaDevice parts; the "unique-id" property, as hex digits, sets it),
 * JEDEC and device IDs (9Fh, 90h, 92h, 94h, ABh), power-down, software
 * reset, continuous read mode (M5-4 = 10b after BBh/EBh/E7h), burst with
 * wrap (77h), the W25Q32RV's QPI mode and read parameters (38h/FFh, C0h,
 * 0Ch), the GD25Q256E's 3- and 4-byte addressing (B7h/E9h, ADS, ADP, the
 * extended address register's A24 and the 4-byte-address commands) and
 * its dummy cycle configuration (DC1/DC0), and the GD25Q32C's quad I/O
 * word read (E7h), fast page program (F2h) and high performance mode
 * (A3h, HPF). Commands that must not be clocked past their last byte
 * (program, erase and status register writes) are not executed when they
 * are. The GD25Q32C's datasheet publishes its SFDP table, which 5Ah reads;
 * the others' is not published (Winbond refers to an application note,
 * GigaDevice to its sales), so 5Ah reads 0xff after its address and dummy
 * byte, a table with no signature.
 *
 * The GD25Q32C lays its status registers out as the Winbond parts do
 * (BP4 and BP3 as SEC and TB, CMP in S14, SRP1 in S8), with GigaDevice's
 * SRP1/SRP0 protection modes and SUS1/SUS2 bits; each of 01h, 31h and 11h
 * writes one register, a second byte leaving the write not executed.
 *
 * Busy times: page program, sector, 32 KiB and 64 KiB block erase, chip
 * erase, status register write and the security registers' program and
 * erase take the datasheets' times in virtual time from the /CS rising
 * edge that ends the command: the typical times, or the maximums with the
 * "max-timing" property. The GD25Q256E's program of n bytes takes
 * min(tPP, tBP1 + (n - 1) * tBP2): its datasheet gives both the byte and
 * the page times but no formula, and its typical byte times exceed the
 * typical tPP for 85 bytes or more. While BUSY, the chip answers only the
 * status register reads, suspend and software reset; it ignores
 * everything else, its outputs then not driven, which the bus reads as 0
 * as it does for a chip select with no device. The array keeps its old
 * contents until the cycle ends; a memory controller mapping it directly
 * (the "memory" link) has the mapping taken away while the chip would
 * ignore a read (busy, powered down, or before tRST, tRES1 or tRES2
 * pass), so its reads reach the chip as commands.
 *
 * Suspend (75h) is accepted during a sector or block erase or a page
 * program, never a chip erase, status register write or security
 * register cycle, and not within the minimum time after a resume (tSUS
 * on the Winbond parts, tRS on the GD25Q256E: the datasheets require it
 * and say nothing of what the chip does otherwise, so the model ignores
 * such a suspend and logs it). SUS (SUS1 for an erase, SUS2 for a program
 * on the GD25Q256E) sets at once, and BUSY clears tSUS later, the
 * maximum the datasheets give; the cycle keeps going until then and
 * completes instead if it ends first. While suspended, the chip runs
 * everything but the commands its datasheet forbids (the Winbond parts:
 * 01h and the erases during an erase suspend, 01h and the programs during
 * a program suspend; the GD25Q256E: status register writes, 44h and the
 * erases during either, and the programs too during a program suspend),
 * and refuses a program or erase reaching the suspended sector or page.
 * Resume (7Ah) sets BUSY at once and the cycle runs for the time it had
 * left.
 *
 * Cells left partly done: a suspended cycle's sector or page while it is
 * suspended, and a cycle cut short, hold its cells part way. A cycle is
 * cut short by a software reset, by a machine reset, which this model
 * takes as a power cycle of the chip (on the boards a watchdog or RUN
 * reset of the SoC leaves the flash powered, but QEMU resets every device
 * with the machine), and by QEMU's shutdown, as a power-off.
 * A cell is done once the fraction f of the cycle's time that has passed
 * exceeds h / 2^32, where h is murmur3's 32-bit finalizer of the cell's
 * bit address (byte address * 8 + bit, security register n's bytes
 * counting from the array size + n * 2048): an erase's done cells read 1
 * and the rest keep their bits, a program's done cells that it clears
 * read 0 and the rest keep theirs. The rule is monotonic, so suspending,
 * resuming and cutting a cycle in any order leaves what one cut at its
 * total elapsed time does. The cells a cut leaves are written back to the
 * drive (-drive if=mtd) as they change, at a shutdown before QEMU exits.
 * A status register write cut short keeps the old values. The stored
 * status register bits and the security registers are not on the drive,
 * which holds the array only: they start from the factory values in each
 * run.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/block/power_cut.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"
#include "system/runstate.h"
#include "m25p80_chip.h"
#include "trace.h"

enum {
    VENDOR_WINBOND,
    VENDOR_GIGADEVICE,
};

/* Features some of the parts have. */
#define F_QPI           BIT(0)  /* QPI mode, C0h read parameters, 0Ch */
#define F_WPS           BIT(1)  /* WPS and the individual block locks */
#define F_4BYTE         BIT(2)  /* 3- and 4-byte addressing */
#define F_WRSR_2B       BIT(3)  /* 01h writes SR1 and SR2 */
#define F_SR3_EPE       BIT(4)  /* SR3's PE and EE program/erase errors */
/* SR1's BP4/BP3 are SEC/TB, SR2's S14 is CMP and S8 SRP1 (Winbond's) */
#define F_WB_SR         BIT(5)
#define F_HPM           BIT(6)  /* A3h high performance mode, HPF in S20 */
#define F_WORD_READ     BIT(7)  /* E7h quad I/O word fast read */
#define F_FPP           BIT(8)  /* F2h fast page program */

/* Typical and maximum times, ns. */
enum { TYP, MAX };

struct M25ChipInfo {
    const char *name;
    uint8_t vendor;
    uint8_t jedec[3];
    uint8_t dev_id;
    uint32_t size;
    uint32_t flags;
    /* Factory values of the stored bits, and which bits WRSR writes. */
    uint8_t sr_default[3];
    uint8_t sr_writable[3];
    /* One-time programmable bits: once 1, they stay 1. */
    uint8_t sr_otp[3];
    uint16_t secreg_size;
    uint8_t uid_len;
    const char *uid_default;
    int64_t t_w[2];
    int64_t t_pp[2];
    int64_t t_bp1[2];
    int64_t t_bp2[2];
    int64_t t_se[2];
    int64_t t_be32[2];
    int64_t t_be64[2];
    int64_t t_ce[2];
    int64_t t_sus;
    int64_t t_sus_gap;
    int64_t t_rst;
    int64_t t_rst_erase;
    int64_t t_dp;
    int64_t t_res1;
    int64_t t_res2;
    /* VCC (min) to CS# low: deaf for that long after its power comes up */
    int64_t t_vsl;
    /* The SFDP table, where the datasheet publishes it */
    const uint8_t *sfdp;
    uint16_t sfdp_len;
};

#define US  1000LL
#define MS  1000000LL
#define S   1000000000LL

/* Status register bits common to the parts. */
#define SR1_BUSY        0x01
#define SR1_WEL         0x02
#define SR2_QE          0x02
#define SR2_LB          0x38

/* Winbond. */
#define W_SR1_BP        0x1c
#define W_SR1_TB        0x20
#define W_SR1_SEC       0x40
#define W_SR1_SRP       0x80
#define W_SR2_SRL       0x01
#define W_SR2_LB0       0x04
#define W_SR2_CMP       0x40
#define W_SR2_SUS       0x80
#define W_SR3_WPS       0x04

/* GigaDevice. */
#define G_SR1_BP        0x7c
#define G_SR1_SRP0      0x80
#define G_SR2_ADS       0x01
#define G_SR2_SUS2      0x04
#define G_SR2_SRP1      0x40
#define G_SR2_SUS1      0x80
#define G_SR3_DC0       0x01
#define G_SR3_PE        0x04
#define G_SR3_EE        0x08
#define G_SR3_ADP       0x10
#define G_SR3_HPF       0x10

/* W25Q32RV-JQ: the Pico 2's W25Q32RVXHJQ. */
/* [spec:nuos:req:emu.flash-chip] */
const M25ChipInfo m25chip_w25q32rv = {
    .name = "W25Q32RV",
    .vendor = VENDOR_WINBOND,
    .jedec = { 0xef, 0x40, 0x16 },
    .dev_id = 0x15,
    .size = 4 * MiB,
    .flags = F_QPI | F_WRSR_2B,
    /* JQ parts ship with QE=1; LB0 (the SFDP lock) is 1; DRV 50 ohm. */
    .sr_default = { 0x00, SR2_QE | W_SR2_LB0, 0x40 },
    .sr_writable = { 0xfc, W_SR2_CMP | SR2_LB | SR2_QE | W_SR2_SRL, 0xe0 },
    .sr_otp = { 0x00, SR2_LB, 0x00 },
    .secreg_size = 256,
    .uid_len = 8,
    .uid_default = "e46b6c81d35c1a2b",
    .t_w = { 1500 * US, 15 * MS },
    .t_pp = { 250 * US, 2 * MS },
    .t_se = { 30 * MS, 240 * MS },
    .t_be32 = { 80 * MS, 800 * MS },
    .t_be64 = { 120 * MS, 1200 * MS },
    .t_ce = { 6 * S, 40 * S },
    .t_sus = 20 * US,
    .t_sus_gap = 20 * US,
    .t_rst = 30 * US,
    .t_rst_erase = 30 * US,
    .t_dp = 3 * US,
    .t_res1 = 3 * US,
    .t_res2 = 1800,
};

/*
 * W25Q16JV-IQ: the Waveshare RP2350-USB-C's W25Q16JVUXIQ, whose QE is
 * fixed at 1 in the factory.
 */
/* [spec:nuos:req:emu.flash-chip] */
const M25ChipInfo m25chip_w25q16jv = {
    .name = "W25Q16JV",
    .vendor = VENDOR_WINBOND,
    .jedec = { 0xef, 0x40, 0x15 },
    .dev_id = 0x14,
    .size = 2 * MiB,
    .flags = F_WPS | F_WRSR_2B,
    /* DRV1, DRV0 = 11b (25%) by default. */
    .sr_default = { 0x00, SR2_QE, 0x60 },
    .sr_writable = { 0xfc, W_SR2_CMP | SR2_LB | W_SR2_SRL, 0x64 },
    .sr_otp = { 0x00, SR2_LB, 0x00 },
    .secreg_size = 256,
    .uid_len = 8,
    .uid_default = "d26a3c1b49e05f17",
    .t_w = { 10 * MS, 15 * MS },
    .t_pp = { 400 * US, 3 * MS },
    .t_se = { 45 * MS, 400 * MS },
    .t_be32 = { 120 * MS, 1600 * MS },
    .t_be64 = { 150 * MS, 2000 * MS },
    .t_ce = { 5 * S, 25 * S },
    .t_sus = 20 * US,
    .t_sus_gap = 20 * US,
    .t_rst = 30 * US,
    .t_rst_erase = 30 * US,
    .t_dp = 3 * US,
    .t_res1 = 3 * US,
    .t_res2 = 1800,
};

/*
 * GD25Q256E, temperature range I (-40 to 85 C): the board's
 * GD25Q256EYIGR. The status registers ship as 00h but DRV0 (S21).
 */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
const M25ChipInfo m25chip_gd25q256e = {
    .name = "GD25Q256E",
    .vendor = VENDOR_GIGADEVICE,
    .jedec = { 0xc8, 0x40, 0x19 },
    .dev_id = 0x18,
    .size = 32 * MiB,
    .flags = F_4BYTE | F_SR3_EPE,
    .sr_default = { 0x00, 0x00, 0x20 },
    .sr_writable = { 0xfc, G_SR2_SRP1 | SR2_LB | SR2_QE, 0xf3 },
    .sr_otp = { 0x00, SR2_LB, 0x00 },
    .secreg_size = 2048,
    .uid_len = 16,
    .uid_default = "c84019e7a53b20d16f8e42c95b07a3d1",
    .t_w = { 5 * MS, 20 * MS },
    .t_pp = { 250 * US, 2 * MS },
    .t_bp1 = { 40 * US, 90 * US },
    .t_bp2 = { 2500, 7 * US },
    .t_se = { 30 * MS, 400 * MS },
    .t_be32 = { 120 * MS, 1200 * MS },
    .t_be64 = { 150 * MS, 1600 * MS },
    .t_ce = { 70 * S, 200 * S },
    .t_sus = 20 * US,
    .t_sus_gap = 100 * US,
    .t_rst = 30 * US,
    .t_rst_erase = 12 * MS,
    .t_dp = 3 * US,
    .t_res1 = 30 * US,
    .t_res2 = 30 * US,
};

/*
 * The GD25Q32C's SFDP table (datasheet tables 3 to 5): the header, the
 * JEDEC parameter table at 30h and GigaDevice's at 60h; the rest of the
 * space reads FFh.
 */
static const uint8_t gd25q32c_sfdp[] = {
    0x53, 0x46, 0x44, 0x50, 0x00, 0x01, 0x01, 0xff,
    0x00, 0x00, 0x01, 0x09, 0x30, 0x00, 0x00, 0xff,
    0xc8, 0x00, 0x01, 0x03, 0x60, 0x00, 0x00, 0xff,
    [0x18 ... 0x2f] = 0xff,
    0xe5, 0x20, 0xf1, 0xff, 0xff, 0xff, 0xff, 0x01,
    0x44, 0xeb, 0x08, 0x6b, 0x08, 0x3b, 0x42, 0xbb,
    0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
    0xff, 0xff, 0x00, 0xff, 0x0c, 0x20, 0x0f, 0x52,
    0x10, 0xd8, 0x00, 0xff,
    [0x54 ... 0x5f] = 0xff,
    0x00, 0x36, 0x00, 0x27, 0x9e, 0xf9, 0x77, 0x64,
    0xfc, 0xeb, 0xff, 0xff,
};

/*
 * GD25Q32C, temperature range I (-40 to 85 C): the 4 MiB flash of the
 * ESP32-WROOM-32E's N4 variant, the part Espressif's ESP32 modules have
 * carried from the first (XMC's XM25QE32C is a second source since
 * Espressif's PCN20220402; the module's datasheet names no part, and its
 * table 17 gives the envelope of the qualified parts, which the GD25Q32C's
 * own times fall within). The status registers ship as 00h but DRV0
 * (S21), so QE is clear and IO3 is HOLD#. VCC is 2.7 to 3.6 V.
 */
/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.flash-timing] */
const M25ChipInfo m25chip_gd25q32c = {
    .name = "GD25Q32C",
    .vendor = VENDOR_GIGADEVICE,
    .jedec = { 0xc8, 0x40, 0x16 },
    .dev_id = 0x15,
    .size = 4 * MiB,
    .flags = F_WB_SR | F_HPM | F_WORD_READ | F_FPP,
    .sr_default = { 0x00, 0x00, 0x20 },
    /* Not S23, S20-S15, S10, S1 or S0 (7.5); LB3-LB1 are OTP. */
    .sr_writable = { 0xfc, W_SR2_CMP | SR2_LB | SR2_QE | W_SR2_SRL, 0x60 },
    .sr_otp = { 0x00, SR2_LB, 0x00 },
    .secreg_size = 1024,
    .uid_len = 16,
    .uid_default = "c8401652e19a7f03b64d2c8e1a5f9037",
    .t_w = { 5 * MS, 30 * MS },
    .t_pp = { 600 * US, 2400 * US },
    .t_bp1 = { 30 * US, 50 * US },
    .t_bp2 = { 2500, 12 * US },
    .t_se = { 50 * MS, 300 * MS },
    .t_be32 = { 150 * MS, 1600 * MS },
    .t_be64 = { 250 * MS, 2000 * MS },
    .t_ce = { 15 * S, 30 * S },
    .t_sus = 20 * US,
    .t_sus_gap = 100 * US,
    .t_rst = 30 * US,
    .t_rst_erase = 12 * MS,
    .t_dp = 20 * US,
    .t_res1 = 20 * US,
    .t_res2 = 20 * US,
    .t_vsl = 1800 * US,
    .sfdp = gd25q32c_sfdp,
    .sfdp_len = sizeof(gd25q32c_sfdp),
};

uint32_t m25chip_size(const M25ChipInfo *info)
{
    return info->size;
}

enum {
    OP_NONE,
    OP_PROGRAM,
    OP_ERASE,
    OP_CHIP_ERASE,
    OP_WRSR,
    OP_SEC_PROGRAM,
    OP_SEC_ERASE,
};

/* Transaction phases. */
enum {
    PH_CMD,
    PH_ADDR,
    PH_SKIP,
    PH_OUT,
    PH_ARRAY,
    PH_SECREG,
    PH_SFDP,
    PH_DATA,
    PH_IGNORE,
};

static int64_t now_ns(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static bool winbond(M25Chip *c)
{
    return c->info->vendor == VENDOR_WINBOND;
}

static bool has(M25Chip *c, uint32_t flag)
{
    return c->info->flags & flag;
}

/* The Winbond status register layout: SEC/TB in SR1, CMP and SRP1 in SR2 */
static bool wb_sr(M25Chip *c)
{
    return winbond(c) || has(c, F_WB_SR);
}

/* A GigaDevice part's SRP1, where its layout puts it */
static uint8_t gd_srp1(M25Chip *c)
{
    return has(c, F_WB_SR) ? W_SR2_SRL : G_SR2_SRP1;
}

static int64_t t(M25Chip *c, const int64_t *times)
{
    return times[c->max_timing ? MAX : TYP];
}

static bool quad_enabled(M25Chip *c)
{
    return c->sr[1] & SR2_QE;
}

/* Status */

static bool busy(M25Chip *c)
{
    return c->op.kind != OP_NONE;
}

static bool erase_kind(uint8_t kind)
{
    return kind == OP_ERASE || kind == OP_CHIP_ERASE || kind == OP_SEC_ERASE;
}

static bool program_kind(uint8_t kind)
{
    return kind == OP_PROGRAM || kind == OP_SEC_PROGRAM;
}

/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static uint8_t sr_read(M25Chip *c, int n)
{
    uint8_t v = c->sr[n];
    bool sus_erase = (c->susp.kind && erase_kind(c->susp.kind)) ||
                     (c->suspending && erase_kind(c->op.kind));
    bool sus_prog = (c->susp.kind && program_kind(c->susp.kind)) ||
                    (c->suspending && program_kind(c->op.kind));

    switch (n) {
    case 0:
        v &= ~(SR1_BUSY | SR1_WEL);
        v |= (busy(c) ? SR1_BUSY : 0) | (c->wel ? SR1_WEL : 0);
        break;
    case 1:
        if (winbond(c)) {
            v &= ~W_SR2_SUS;
            v |= sus_erase || sus_prog ? W_SR2_SUS : 0;
        } else {
            v &= ~(G_SR2_SUS1 | G_SR2_SUS2);
            v |= (sus_erase ? G_SR2_SUS1 : 0) | (sus_prog ? G_SR2_SUS2 : 0);
            if (has(c, F_4BYTE)) {
                v &= ~G_SR2_ADS;
                v |= c->four_byte ? G_SR2_ADS : 0;
            }
        }
        break;
    case 2:
        if (has(c, F_HPM)) {
            v &= ~G_SR3_HPF;
            v |= c->hpm ? G_SR3_HPF : 0;
        }
        break;
    }
    return v;
}

/* Protection */

/*
 * The range [*lo, *hi) the block protect bits protect, before CMP. Returns
 * false for none.
 */
/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static bool bp_range(M25Chip *c, uint32_t *lo, uint32_t *hi)
{
    uint32_t size = c->info->size;
    uint32_t len;

    if (wb_sr(c)) {
        unsigned bp = extract32(c->sr[0], 2, 3);
        unsigned all_bp = ctz32(size / (64 * KiB)) + 1;

        if (!bp) {
            return false;
        }
        if (bp >= all_bp) {
            len = size;
        } else if (c->sr[0] & W_SR1_SEC) {
            len = (4 * KiB) << MIN(bp - 1, 3);
        } else {
            len = (64 * KiB) << (bp - 1);
        }
        *lo = (c->sr[0] & W_SR1_TB) ? 0 : size - len;
    } else {
        unsigned bp = extract32(c->sr[0], 2, 4);

        if (!bp) {
            return false;
        }
        len = bp >= 10 ? size : (64 * KiB) << (bp - 1);
        *lo = (c->sr[0] & 0x40) ? 0 : size - len;
    }
    *hi = *lo + len;
    return true;
}

/* The individual lock of `addr`: sectors in the top and bottom blocks. */
static unsigned lock_index(M25Chip *c, uint32_t addr)
{
    unsigned blocks = c->info->size / (64 * KiB);
    unsigned block = addr / (64 * KiB);

    if (block == 0) {
        return addr / (4 * KiB);
    }
    if (block == blocks - 1) {
        return 16 + (blocks - 2) + (addr / (4 * KiB)) % 16;
    }
    return 16 + block - 1;
}

/* Whether a program or erase of [addr, addr + len) touches protected cells. */
/* [spec:nuos:req:emu.flash-chip] */
static bool protected(M25Chip *c, uint32_t addr, uint32_t len)
{
    uint32_t lo, hi;
    bool in;

    if (has(c, F_WPS) && (c->sr[2] & W_SR3_WPS)) {
        uint32_t a;

        for (a = addr; a < addr + len; a += 4 * KiB) {
            if (c->locks[lock_index(c, a)]) {
                return true;
            }
        }
        return false;
    }
    if (!bp_range(c, &lo, &hi)) {
        lo = hi = 0;
    }
    if (wb_sr(c) && (c->sr[1] & W_SR2_CMP)) {
        /* The complement: everything outside [lo, hi). */
        return addr < lo || addr + len > hi;
    }
    in = addr < hi && addr + len > lo;
    return in;
}

/* Storage */

static uint8_t *op_cells(M25Chip *c, const M25ChipOp *op, uint64_t *bit0)
{
    if (op->kind == OP_SEC_PROGRAM || op->kind == OP_SEC_ERASE) {
        unsigned n = op->addr / M25CHIP_SECREG_MAX;

        *bit0 = ((uint64_t)c->info->size + op->addr) * 8;
        return &c->secreg[n][op->addr % M25CHIP_SECREG_MAX];
    }
    *bit0 = (uint64_t)op->addr * 8;
    return c->storage + op->addr;
}

static bool op_in_array(const M25ChipOp *op)
{
    return op->kind == OP_PROGRAM || op->kind == OP_ERASE ||
           op->kind == OP_CHIP_ERASE;
}

static void write_back_done(void *opaque, int ret)
{
    QEMUIOVector *iov = opaque;

    qemu_iovec_destroy(iov);
    g_free(iov);
}

/*
 * Keep a direct mapping and the drive coherent with changed cells. A
 * mapping that is out of ROMD mode is flushed when it returns to it.
 */
static void cells_changed(M25Chip *c, uint32_t off, uint32_t len, bool now)
{
    if (c->mem && memory_region_is_romd(c->mem)) {
        memory_region_flush_rom_device(c->mem, off, len);
    } else if (c->mem) {
        if (c->dirty_lo >= c->dirty_hi) {
            c->dirty_lo = off;
            c->dirty_hi = off + len;
        } else {
            c->dirty_lo = MIN(c->dirty_lo, off);
            c->dirty_hi = MAX(c->dirty_hi, off + len);
        }
    }
    if (!c->blk || !blk_is_writable(c->blk)) {
        return;
    }
    if (now) {
        if (blk_pwrite(c->blk, off, len, c->storage + off, 0) < 0) {
            error_report("%s: writing the flash's cells back to its drive "
                         "failed", c->info->name);
        }
    } else {
        QEMUIOVector *iov = g_new(QEMUIOVector, 1);

        qemu_iovec_init(iov, 1);
        qemu_iovec_add(iov, c->storage + off, len);
        blk_aio_pwritev(c->blk, off, iov, 0, write_back_done, iov);
    }
}

/*
 * Leave `op`'s cells as they are `done` ns into it: each cell done once
 * the fraction of the cycle passed exceeds its hash's fraction of 2^32
 * (hw/block/power_cut.h).
 */
/* [spec:nuos:req:emu.flash-timing] */
/* [spec:nuos:req:emu.flash-power-cut] */
static void op_partial(M25Chip *c, const M25ChipOp *op, int64_t done,
                       bool now)
{
    uint64_t bit0;
    uint8_t *cells = op_cells(c, op, &bit0);
    uint32_t thr;
    uint32_t i;

    if (done <= 0) {
        return;
    }
    thr = power_cut_threshold(done, op->total);
    for (i = 0; i < op->len; i++) {
        uint8_t mask = power_cut_done_mask(bit0 + i * 8, thr);

        if (erase_kind(op->kind)) {
            cells[i] |= mask;
        } else {
            cells[i] &= ~(mask & ~op->data[i]);
        }
    }
    if (op_in_array(op)) {
        cells_changed(c, op->addr, op->len, now);
    }
}

/* Complete `op`'s effect on the cells. */
static void op_apply(M25Chip *c, const M25ChipOp *op)
{
    uint64_t bit0;
    uint8_t *cells;
    uint32_t i;

    if (op->kind == OP_WRSR) {
        for (i = 0; i < 3; i++) {
            if (op->sr_mask & BIT(i)) {
                c->sr_nv[i] = op->sr[i];
                c->sr[i] = op->sr[i];
            }
        }
        return;
    }
    cells = op_cells(c, op, &bit0);
    if (erase_kind(op->kind)) {
        memset(cells, 0xff, op->len);
    } else {
        for (i = 0; i < op->len; i++) {
            cells[i] &= op->data[i];
        }
    }
    if (op_in_array(op)) {
        cells_changed(c, op->addr, op->len, false);
    }
}

/* Direct mapping */

/* Whether the chip would answer an array read now. */
static bool answers_reads(M25Chip *c)
{
    return !busy(c) && !c->powered_down && now_ns() >= c->deaf_until;
}

/* [spec:nuos:req:emu.flash-timing] */
static void update_romd(M25Chip *c)
{
    bool romd;

    if (!c->mem) {
        return;
    }
    romd = answers_reads(c);
    if (memory_region_is_romd(c->mem) != romd) {
        /* The controller remaps around it in the same transaction. */
        memory_region_transaction_begin();
        memory_region_rom_device_set_romd(c->mem, romd);
        qemu_set_irq(c->mapped, romd);
        memory_region_transaction_commit();
    }
    if (romd && c->dirty_lo < c->dirty_hi) {
        memory_region_flush_rom_device(c->mem, c->dirty_lo,
                                       c->dirty_hi - c->dirty_lo);
        c->dirty_lo = c->dirty_hi = 0;
    }
}

static void deafen(M25Chip *c, int64_t ns)
{
    c->deaf_until = MAX(c->deaf_until, now_ns() + ns);
    timer_mod(c->deaf_timer, c->deaf_until);
    update_romd(c);
}

static void deaf_end(void *opaque)
{
    update_romd(opaque);
}

/* Cycles */

static int64_t op_end(M25Chip *c)
{
    return c->op.started + c->op.total - c->op.done;
}

static void op_schedule(M25Chip *c)
{
    int64_t when;

    if (!busy(c)) {
        timer_del(c->op_timer);
        return;
    }
    when = op_end(c);
    if (c->suspending) {
        when = MIN(when, c->sus_at);
    }
    timer_mod(c->op_timer, when);
}

/* [spec:nuos:req:emu.flash-timing] */
static void op_start(M25Chip *c, uint8_t kind, uint32_t addr, uint32_t len,
                     int64_t total)
{
    c->op.kind = kind;
    c->op.addr = addr;
    c->op.len = len;
    c->op.total = total;
    c->op.done = 0;
    c->op.started = now_ns();
    c->resumed_at = INT64_MIN;
    trace_m25chip_cycle_start(c->info->name, kind, addr, len, total);
    if (!winbond(c) && kind != OP_WRSR) {
        /* "At some unspecified time before the cycle is completed." */
        c->wel = false;
        if (has(c, F_SR3_EPE)) {
            c->sr[2] &= ~(program_kind(kind) ? G_SR3_PE : G_SR3_EE);
        }
    }
    op_schedule(c);
    update_romd(c);
}

/* [spec:nuos:req:emu.flash-timing] */
static void op_complete(M25Chip *c)
{
    M25ChipOp op = c->op;

    c->op.kind = OP_NONE;
    c->suspending = false;
    trace_m25chip_cycle_end(c->info->name, op.kind);
    op_apply(c, &op);
    c->wel = false;
    op_schedule(c);
    update_romd(c);
}

/* [spec:nuos:req:emu.flash-timing] */
static void suspend_complete(M25Chip *c, int64_t when)
{
    c->op.done += when - c->op.started;
    trace_m25chip_suspend(c->info->name, c->op.kind, c->op.done);
    c->susp = c->op;
    c->op.kind = OP_NONE;
    c->suspending = false;
    op_partial(c, &c->susp, c->susp.done, false);
    op_schedule(c);
    update_romd(c);
}

/* [spec:nuos:req:emu.flash-timing] */
static void op_timer_cb(void *opaque)
{
    M25Chip *c = opaque;
    int64_t end;

    if (!busy(c)) {
        return;
    }
    end = op_end(c);
    if (c->suspending && c->sus_at < end) {
        if (now_ns() >= c->sus_at) {
            suspend_complete(c, c->sus_at);
            return;
        }
    } else if (now_ns() >= end) {
        op_complete(c);
        return;
    }
    op_schedule(c);
}

/*
 * Run the cycle's event if its time has passed: the timer may run late on
 * the host's clock, after an access that must find the event happened.
 */
static void catch_up(M25Chip *c)
{
    if (timer_pending(c->op_timer) &&
        timer_expire_time_ns(c->op_timer) <= now_ns()) {
        timer_del(c->op_timer);
        op_timer_cb(c);
    }
}

/*
 * Cut the running and suspended cycles short, leaving their cells part
 * way, written back at once with `now`. Returns whether an erase was
 * among them.
 */
/* [spec:nuos:req:emu.flash-power-cut] */
static bool ops_cut(M25Chip *c, bool now)
{
    bool erase = false;
    int64_t t_now;

    catch_up(c);
    t_now = now_ns();

    if (c->op.kind) {
        int64_t done = c->op.done + (t_now - c->op.started);

        erase |= erase_kind(c->op.kind);
        trace_m25chip_cut(c->info->name, c->op.kind, done, c->op.total);
        if (c->op.kind != OP_WRSR) {
            op_partial(c, &c->op, done, now);
        }
        c->op.kind = OP_NONE;
    }
    if (c->susp.kind) {
        erase |= erase_kind(c->susp.kind);
        trace_m25chip_cut(c->info->name, c->susp.kind, c->susp.done,
                          c->susp.total);
        op_partial(c, &c->susp, c->susp.done, now);
        c->susp.kind = OP_NONE;
    }
    c->suspending = false;
    timer_del(c->op_timer);
    return erase;
}

/* The volatile state as power-up or a software reset leaves it. */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static void reset_volatile(M25Chip *c)
{
    memcpy(c->sr, c->sr_nv, sizeof(c->sr));
    if (has(c, F_SR3_EPE)) {
        c->sr[2] &= ~(G_SR3_PE | G_SR3_EE);
    }
    c->wel = false;
    c->vsr_we = false;
    c->reset_enable = false;
    c->qpi = false;
    c->four_byte = has(c, F_4BYTE) && (c->sr[2] & G_SR3_ADP);
    c->hpm = false;
    c->ear = 0;
    c->read_params = 0;
    c->wrap = 0x10;
    c->cont_cmd = 0;
    c->powered_down = false;
    memset(c->locks, 1, sizeof(c->locks));
    c->phase = PH_CMD;
}

/* Power-up: the volatile state, and the locks a power cycle releases. */
static void power_up(M25Chip *c)
{
    if (winbond(c)) {
        c->sr_nv[1] &= ~W_SR2_SRL;
    } else if ((c->sr_nv[0] & G_SR1_SRP0) == 0 &&
               (c->sr_nv[1] & gd_srp1(c))) {
        /* SRP1, SRP0 = 1, 0: locked down until the next power cycle. */
        c->sr_nv[1] &= ~gd_srp1(c);
    }
    reset_volatile(c);
    c->deaf_until = 0;
    c->resumed_at = INT64_MIN;
    timer_del(c->deaf_timer);
}

/* Commands */

/* The commands the chip answers while BUSY. */
/* [spec:nuos:req:emu.flash-timing] */
static bool allowed_busy(uint8_t cmd)
{
    switch (cmd) {
    case 0x05:
    case 0x35:
    case 0x15:
    case 0x75:
    case 0x66:
    case 0x99:
        return true;
    }
    return false;
}

static bool is_erase_cmd(uint8_t cmd)
{
    switch (cmd) {
    case 0x20:
    case 0x21:
    case 0x52:
    case 0x5c:
    case 0xd8:
    case 0xdc:
    case 0xc7:
    case 0x60:
        return true;
    }
    return false;
}

static bool is_program_cmd(uint8_t cmd)
{
    switch (cmd) {
    case 0x02:
    case 0x12:
    case 0x32:
    case 0x34:
    case 0xf2:
        return true;
    }
    return false;
}

/* The commands the datasheet forbids while a cycle is suspended. */
/* [spec:nuos:req:emu.flash-timing] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static bool forbidden_suspended(M25Chip *c, uint8_t cmd)
{
    bool prog = program_kind(c->susp.kind);

    if (winbond(c)) {
        if (cmd == 0x01) {
            return true;
        }
        if (prog) {
            return is_program_cmd(cmd) || cmd == 0x42;
        }
        return is_erase_cmd(cmd) || cmd == 0x44;
    }
    if (cmd == 0x01 || cmd == 0x31 || cmd == 0x11 || cmd == 0x44 ||
        is_erase_cmd(cmd)) {
        return true;
    }
    return prog && (is_program_cmd(cmd) || cmd == 0x42);
}

/* The W25Q32RV's QPI instruction set. */
static bool qpi_command(uint8_t cmd)
{
    switch (cmd) {
    case 0x06: case 0x50: case 0x04: case 0xab: case 0x90: case 0x9f:
    case 0xc0: case 0x02: case 0x20: case 0x52: case 0xd8: case 0xc7:
    case 0x60: case 0x05: case 0x01: case 0x35: case 0x31: case 0x15:
    case 0x11: case 0xb9: case 0x66: case 0x99: case 0xff: case 0x0b:
    case 0x0c: case 0xeb: case 0x75: case 0x7a:
        return true;
    }
    return false;
}

/* Commands that need QE=1 (the quad commands, and Enter QPI). */
static bool needs_qe(M25Chip *c, uint8_t cmd)
{
    switch (cmd) {
    case 0x6b:
    case 0xeb:
    case 0x32:
    case 0x77:
        return !c->qpi;
    case 0xe7:
        return true;
    case 0x94:
        return wb_sr(c);
    case 0x38:
        return winbond(c);
    case 0x6c:
    case 0xec:
    case 0x34:
        return has(c, F_4BYTE);
    }
    return false;
}

/* The address length of a GD25Q256E command, by the current mode. */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static unsigned gd_addr_len(M25Chip *c, uint8_t cmd)
{
    switch (cmd) {
    case 0x13: case 0x0c: case 0x3c: case 0x6c: case 0xbc: case 0xec:
    case 0x12: case 0x34: case 0x21: case 0x5c: case 0xdc:
        return 4;
    case 0x90:
    case 0x5a:
        return 3;
    }
    return c->four_byte ? 4 : 3;
}

/* The W25Q32RV's dummy clocks for QPI reads and EBh, from P6-P4. */
static unsigned rv_dummy_clocks(M25Chip *c)
{
    unsigned p = extract32(c->read_params, 4, 3);

    return p <= 2 ? 6 : 2 * p + 2;
}

/* What follows the opcode, decoded. */
typedef struct {
    bool known;
    unsigned addr;
    unsigned skip;
    uint8_t next;
} CmdShape;

/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static CmdShape shape(M25Chip *c, uint8_t cmd)
{
    CmdShape sh = { .known = true, .next = PH_DATA };
    unsigned a3 = winbond(c) ? 3 : gd_addr_len(c, cmd);

    switch (cmd) {
    case 0x06: case 0x50: case 0x04: case 0xc7: case 0x60: case 0xb9:
    case 0x75: case 0x7a: case 0x66: case 0x99: case 0x01: case 0x31:
    case 0x11:
        break;
    case 0x05: case 0x35: case 0x15: case 0x9f:
        sh.next = PH_OUT;
        break;
    case 0xab:
        sh.skip = 3;
        sh.next = PH_OUT;
        break;
    case 0x90:
        sh.addr = 3;
        sh.next = PH_OUT;
        break;
    case 0x03:
        sh.addr = a3;
        sh.next = PH_ARRAY;
        break;
    case 0x0b: case 0x3b: case 0x6b:
        sh.addr = a3;
        sh.skip = c->qpi ? rv_dummy_clocks(c) / 2 : 1;
        sh.next = PH_ARRAY;
        break;
    case 0xbb:
        sh.addr = a3;
        sh.skip = winbond(c) ? 1 : (c->sr[2] & G_SR3_DC0 ? 2 : 1);
        sh.next = PH_ARRAY;
        break;
    case 0xeb:
        sh.addr = a3;
        if (has(c, F_QPI)) {
            sh.skip = rv_dummy_clocks(c) / 2;
        } else if (winbond(c)) {
            sh.skip = 3;
        } else {
            sh.skip = c->sr[2] & G_SR3_DC0 ? 5 : 3;
        }
        sh.next = PH_ARRAY;
        break;
    case 0x02: case 0x32: case 0x20: case 0x52: case 0xd8: case 0x44:
    case 0x42:
        sh.addr = a3;
        break;
    case 0x48: case 0x5a:
        sh.addr = a3;
        sh.skip = 1;
        sh.next = cmd == 0x48 ? PH_SECREG : PH_SFDP;
        break;
    case 0x4b:
        if (winbond(c)) {
            sh.skip = 4;
        } else {
            sh.addr = a3;
            sh.skip = 1;
        }
        sh.next = PH_OUT;
        break;
    case 0x77:
        sh.skip = 3;
        break;
    case 0xe7:
        /* A, then M7-M0 and two dummy clocks on four lines */
        sh.known = has(c, F_WORD_READ);
        sh.addr = a3;
        sh.skip = 2;
        sh.next = PH_ARRAY;
        break;
    case 0xf2:
        sh.known = has(c, F_FPP);
        sh.addr = a3;
        break;
    case 0xa3:
        sh.known = has(c, F_HPM);
        sh.skip = 3;
        break;
    default:
        sh.known = false;
        break;
    }
    if (sh.known || has(c, F_4BYTE)) {
        if (!sh.known) {
            sh.known = true;
            switch (cmd) {
            case 0xc8:
                sh.next = PH_OUT;
                break;
            case 0xc5: case 0xb7: case 0xe9:
                break;
            case 0x13:
                sh.addr = 4;
                sh.next = PH_ARRAY;
                break;
            case 0x0c: case 0x3c: case 0x6c:
                sh.addr = 4;
                sh.skip = 1;
                sh.next = PH_ARRAY;
                break;
            case 0xbc:
                sh.addr = 4;
                sh.skip = c->sr[2] & G_SR3_DC0 ? 2 : 1;
                sh.next = PH_ARRAY;
                break;
            case 0xec:
                sh.addr = 4;
                sh.skip = c->sr[2] & G_SR3_DC0 ? 5 : 3;
                sh.next = PH_ARRAY;
                break;
            case 0x12: case 0x34: case 0x21: case 0x5c: case 0xdc:
                sh.addr = 4;
                break;
            default:
                sh.known = false;
                break;
            }
        }
        return sh;
    }
    sh.known = true;
    switch (cmd) {
    case 0x92:
        sh.known = wb_sr(c);
        sh.addr = 3;
        sh.skip = 1;
        sh.next = PH_OUT;
        break;
    case 0x94:
        sh.known = wb_sr(c);
        sh.addr = 3;
        sh.skip = 3;
        sh.next = PH_OUT;
        break;
    case 0x38:
    case 0xc0:
        sh.known = has(c, F_QPI);
        break;
    case 0xff:
        sh.known = c->qpi;
        break;
    case 0x0c:
        sh.known = c->qpi;
        sh.addr = 3;
        sh.skip = rv_dummy_clocks(c) / 2;
        sh.next = PH_ARRAY;
        break;
    case 0x7e: case 0x98:
        sh.known = has(c, F_WPS);
        break;
    case 0x36: case 0x39:
        sh.known = has(c, F_WPS);
        sh.addr = 3;
        break;
    case 0x3d:
        sh.known = has(c, F_WPS);
        sh.addr = 3;
        sh.next = PH_OUT;
        break;
    default:
        sh.known = false;
        break;
    }
    return sh;
}

/* Whether the transaction's opcode is accepted in the chip's state. */
/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.flash-timing] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static bool accepted(M25Chip *c, uint8_t cmd)
{
    if (now_ns() < c->deaf_until) {
        return false;
    }
    if (c->powered_down) {
        return cmd == 0xab ||
               (!winbond(c) && (cmd == 0x66 || cmd == 0x99));
    }
    if (busy(c)) {
        return allowed_busy(cmd);
    }
    if (c->qpi && !qpi_command(cmd)) {
        return false;
    }
    if (!c->qpi && (cmd == 0xff || cmd == 0x0c) && winbond(c)) {
        return false;
    }
    if (c->qpi && (cmd == 0x03 || cmd == 0x32 || cmd == 0x4b)) {
        return false;
    }
    if (needs_qe(c, cmd) && !quad_enabled(c)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: quad command %02xh with QE=0 "
                      "ignored\n", c->info->name, cmd);
        return false;
    }
    if (c->susp.kind && forbidden_suspended(c, cmd)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: command %02xh is not allowed "
                      "while a cycle is suspended; ignored\n",
                      c->info->name, cmd);
        return false;
    }
    return true;
}

/* The array address of a 3-byte command, with the GD25Q256E's A24. */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static uint32_t array_addr(M25Chip *c)
{
    uint32_t a = c->addr;

    if (!winbond(c) && c->addr_len == 3) {
        a |= (uint32_t)(c->ear & 1) << 24;
    }
    return a & (c->info->size - 1);
}

static void set_out(M25Chip *c, const uint8_t *data, unsigned len)
{
    memcpy(c->out, data, len);
    c->out_len = len;
    c->out_pos = 0;
}

/* The data phase of a read command begins. */
/* [spec:nuos:req:emu.flash-chip] */
static void data_phase(M25Chip *c)
{
    CmdShape sh = shape(c, c->cmd);
    uint8_t buf[M25CHIP_UID_MAX];

    c->phase = sh.next;
    c->count = 0;
    switch (c->cmd) {
    case 0x05:
    case 0x35:
    case 0x15:
        buf[0] = sr_read(c, c->cmd == 0x05 ? 0 : c->cmd == 0x35 ? 1 : 2);
        set_out(c, buf, 1);
        break;
    case 0x9f:
        set_out(c, c->info->jedec, 3);
        break;
    case 0xab:
        set_out(c, &c->info->dev_id, 1);
        break;
    case 0x90:
    case 0x92:
    case 0x94:
        buf[c->addr & 1] = c->info->jedec[0];
        buf[!(c->addr & 1)] = c->info->dev_id;
        set_out(c, buf, 2);
        break;
    case 0x4b:
        set_out(c, c->uid, c->info->uid_len);
        break;
    case 0xc8:
        set_out(c, &c->ear, 1);
        break;
    case 0x3d:
        buf[0] = c->locks[lock_index(c, c->addr & (c->info->size - 1))];
        set_out(c, buf, 1);
        break;
    case 0xe7:
        /* "The lowest address bit (A0) must equal 0": the part ignores it. */
        if (c->addr & 1) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: E7h at odd address 0x%"
                          PRIx32 "; A0 taken as 0\n", c->info->name,
                          c->addr);
            c->addr &= ~1u;
        }
        break;
    case 0x48:
    case 0x5a:
    default:
        break;
    }
    if (c->phase == PH_ARRAY) {
        c->addr = array_addr(c);
    }
}

static void begin_command(M25Chip *c, uint8_t cmd)
{
    CmdShape sh;

    c->cmd = cmd;
    c->addr = 0;
    c->addr_got = 0;
    c->count = 0;
    c->mode = 0xff;
    if (cmd != 0x99) {
        c->reset_enable = false;
    }
    if (!accepted(c, cmd)) {
        trace_m25chip_command(c->info->name, cmd, false);
        c->phase = PH_IGNORE;
        return;
    }
    trace_m25chip_command(c->info->name, cmd, true);
    sh = shape(c, cmd);
    if (!sh.known) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unknown command %02xh\n",
                      c->info->name, cmd);
        c->phase = PH_IGNORE;
        return;
    }
    c->addr_len = sh.addr;
    c->skip = sh.skip;
    if (sh.addr) {
        c->phase = PH_ADDR;
    } else if (sh.skip) {
        c->phase = PH_SKIP;
    } else {
        data_phase(c);
    }
}

/* The wrap section of a read, or 0 for none. */
static unsigned read_wrap(M25Chip *c)
{
    if (c->cmd == 0x0c && winbond(c)) {
        return 8 << extract32(c->read_params, 0, 2);
    }
    if ((c->cmd == 0xeb || c->cmd == 0xec || c->cmd == 0xe7) && !c->qpi &&
        !(c->wrap & 0x10)) {
        return 8 << extract32(c->wrap, 5, 2);
    }
    return 0;
}

/* [spec:nuos:req:emu.flash-chip] */
static uint8_t read_array(M25Chip *c)
{
    uint8_t v = c->storage[c->addr];
    unsigned wrap = read_wrap(c);

    if (wrap) {
        c->addr = (c->addr & ~(wrap - 1)) | ((c->addr + 1) & (wrap - 1));
    } else {
        c->addr = (c->addr + 1) & (c->info->size - 1);
    }
    return v;
}

/*
 * A security register byte address: register 1 to 3 in A15-A12, the
 * byte in the bits below, which must be within the register's size.
 * Returns the register's index from 0, or -1 for none.
 */
static int secreg_index(M25Chip *c, uint32_t addr, uint32_t *off)
{
    unsigned n = extract32(addr, 12, 4);
    uint32_t below = addr & 0xfff;

    if (n < 1 || n > M25CHIP_SECREGS || (addr >> 16) ||
        below >= c->info->secreg_size) {
        return -1;
    }
    *off = below;
    return n - 1;
}

/* [spec:nuos:req:emu.flash-chip] */
static uint8_t read_secreg(M25Chip *c)
{
    uint32_t off;
    int n = secreg_index(c, c->addr, &off);
    uint8_t v;

    if (n < 0) {
        return 0;
    }
    v = c->secreg[n][off];
    off = (off + 1) % c->info->secreg_size;
    c->addr = (c->addr & ~0xfffu) | off;
    return v;
}

static void program_byte(M25Chip *c, uint8_t v)
{
    unsigned i = (c->addr + c->count) % M25CHIP_PAGE_SIZE;

    c->pp[i] = v;
    c->pp_set[i / 8] |= BIT(i % 8);
}

/* [spec:nuos:req:emu.flash-chip] */
uint8_t m25chip_transfer(M25Chip *c, uint8_t tx)
{
    uint8_t r = 0;

    catch_up(c);

    switch (c->phase) {
    case PH_CMD:
        begin_command(c, tx);
        break;
    case PH_ADDR:
        c->addr = (c->addr << 8) | tx;
        if (++c->addr_got == c->addr_len) {
            if (c->skip) {
                c->phase = PH_SKIP;
            } else if (shape(c, c->cmd).next == PH_DATA) {
                c->phase = PH_DATA;
                c->count = 0;
                memset(c->pp, 0xff, sizeof(c->pp));
                memset(c->pp_set, 0, sizeof(c->pp_set));
            } else {
                data_phase(c);
            }
        }
        break;
    case PH_SKIP:
        if (c->mode == 0xff && c->addr_len &&
            (c->cmd == 0xbb || c->cmd == 0xbc || c->cmd == 0xeb ||
             c->cmd == 0xec || c->cmd == 0xe7)) {
            /* The first byte after the address is M7-M0. */
            c->mode = tx & 0x30;
        }
        if (--c->skip == 0) {
            if (shape(c, c->cmd).next == PH_DATA) {
                c->phase = PH_DATA;
                c->count = 0;
            } else {
                data_phase(c);
            }
        }
        break;
    case PH_OUT:
        if (c->cmd == 0x05 || c->cmd == 0x35 || c->cmd == 0x15) {
            /* A status register read follows the cycle as it runs. */
            c->out[0] = sr_read(c, c->cmd == 0x05 ? 0 :
                                   c->cmd == 0x35 ? 1 : 2);
        }
        r = c->out[c->out_pos];
        c->out_pos = (c->out_pos + 1) % c->out_len;
        c->count++;
        break;
    case PH_ARRAY:
        r = read_array(c);
        c->count++;
        break;
    case PH_SECREG:
        r = read_secreg(c);
        c->count++;
        break;
    case PH_SFDP:
        r = c->addr < c->info->sfdp_len ? c->info->sfdp[c->addr] : 0xff;
        c->addr = (c->addr + 1) & 0xffffff;
        c->count++;
        break;
    case PH_DATA:
        if (is_program_cmd(c->cmd) || c->cmd == 0x42) {
            program_byte(c, tx);
        } else if (c->count < sizeof(c->in)) {
            c->in[c->count] = tx;
        }
        c->count++;
        break;
    case PH_IGNORE:
    default:
        break;
    }
    return r;
}

/* Execution at /CS high */

/* [spec:nuos:req:emu.flash-timing] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static int64_t program_time(M25Chip *c, unsigned bytes)
{
    int64_t pp = t(c, c->info->t_pp);

    if (c->info->t_bp1[TYP] && bytes) {
        return MIN(pp, t(c, c->info->t_bp1) +
                       (int64_t)(bytes - 1) * t(c, c->info->t_bp2));
    }
    return pp;
}

static bool overlaps(uint32_t a, uint32_t alen, uint32_t b, uint32_t blen)
{
    return a < b + blen && b < a + alen;
}

/* Whether a program or erase of the array may start on [addr, +len). */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static bool may_write(M25Chip *c, uint32_t addr, uint32_t len, bool erase)
{
    if (!c->wel) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s without write enable; "
                      "ignored\n", c->info->name,
                      erase ? "erase" : "program");
        return false;
    }
    if (c->susp.kind && op_in_array(&c->susp) &&
        overlaps(addr, len, c->susp.addr, c->susp.len)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of the suspended cycle's "
                      "cells; ignored\n", c->info->name,
                      erase ? "erase" : "program");
        return false;
    }
    if (protected(c, addr, len)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of protected cells at "
                      "0x%" PRIx32 "; ignored\n", c->info->name,
                      erase ? "erase" : "program", addr);
        if (has(c, F_SR3_EPE)) {
            c->sr[2] |= erase ? G_SR3_EE : G_SR3_PE;
        }
        return false;
    }
    return true;
}

static unsigned pp_bytes(M25Chip *c)
{
    unsigned i, n = 0;

    for (i = 0; i < sizeof(c->pp_set); i++) {
        n += ctpop8(c->pp_set[i]);
    }
    return n;
}

/* [spec:nuos:req:emu.flash-timing] */
static void exec_program(M25Chip *c)
{
    uint32_t page;

    if (c->addr_got < c->addr_len || c->count == 0) {
        return;
    }
    if (c->cmd == 0x42) {
        uint32_t off;
        int n = secreg_index(c, c->addr, &off);

        if (n < 0 || !c->wel) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: security register program "
                          "at 0x%" PRIx32 " ignored\n", c->info->name,
                          c->addr);
            return;
        }
        if (c->sr[1] & (W_SR2_LB0 << (n + 1))) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: program of locked security "
                          "register %d ignored\n", c->info->name, n + 1);
            if (has(c, F_SR3_EPE)) {
                c->sr[2] |= G_SR3_PE;
            }
            return;
        }
        memcpy(c->op.data, c->pp, sizeof(c->pp));
        op_start(c, OP_SEC_PROGRAM,
                 n * M25CHIP_SECREG_MAX + (off & ~(M25CHIP_PAGE_SIZE - 1)),
                 M25CHIP_PAGE_SIZE, program_time(c, pp_bytes(c)));
        return;
    }
    page = array_addr(c) & ~(M25CHIP_PAGE_SIZE - 1);
    if (!may_write(c, page, M25CHIP_PAGE_SIZE, false)) {
        return;
    }
    memcpy(c->op.data, c->pp, sizeof(c->pp));
    op_start(c, OP_PROGRAM, page, M25CHIP_PAGE_SIZE,
             program_time(c, pp_bytes(c)));
}

/* [spec:nuos:req:emu.flash-timing] */
static void exec_erase(M25Chip *c)
{
    uint32_t len;
    int64_t total;
    uint32_t addr;

    if (c->addr_got < c->addr_len || c->count) {
        if (c->count) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: erase %02xh clocked past its "
                          "address; not executed\n", c->info->name, c->cmd);
        }
        return;
    }
    switch (c->cmd) {
    case 0x20:
    case 0x21:
        len = 4 * KiB;
        total = t(c, c->info->t_se);
        break;
    case 0x52:
    case 0x5c:
        len = 32 * KiB;
        total = t(c, c->info->t_be32);
        break;
    case 0xd8:
    case 0xdc:
        len = 64 * KiB;
        total = t(c, c->info->t_be64);
        break;
    default:
        len = c->info->size;
        total = t(c, c->info->t_ce);
        break;
    }
    addr = len == c->info->size ? 0 : array_addr(c) & ~(len - 1);
    if (!may_write(c, addr, len, true)) {
        return;
    }
    op_start(c, len == c->info->size ? OP_CHIP_ERASE : OP_ERASE, addr, len,
             total);
}

/* [spec:nuos:req:emu.flash-timing] */
static void exec_secreg_erase(M25Chip *c)
{
    uint32_t off;
    int n;

    if (c->addr_got < c->addr_len || c->count) {
        return;
    }
    n = secreg_index(c, c->addr, &off);
    if (n < 0 || !c->wel) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: security register erase at "
                      "0x%" PRIx32 " ignored\n", c->info->name, c->addr);
        return;
    }
    if (c->sr[1] & (W_SR2_LB0 << (n + 1))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: erase of locked security "
                      "register %d ignored\n", c->info->name, n + 1);
        if (has(c, F_SR3_EPE)) {
            c->sr[2] |= G_SR3_EE;
        }
        return;
    }
    op_start(c, OP_SEC_ERASE, n * M25CHIP_SECREG_MAX, c->info->secreg_size,
             t(c, c->info->t_se));
}

/* Whether the status registers may be written now. */
/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static bool sr_writable(M25Chip *c)
{
    bool hw = !c->wp_level && !quad_enabled(c);

    if (winbond(c)) {
        if (c->sr[1] & W_SR2_SRL) {
            return false;
        }
        return !((c->sr[0] & W_SR1_SRP) && hw);
    }
    switch (((c->sr[1] & gd_srp1(c)) ? 2 : 0) |
            ((c->sr[0] & G_SR1_SRP0) ? 1 : 0)) {
    case 0:
        return true;
    case 1:
        return !hw;
    default:
        return false;
    }
}

/* [spec:nuos:req:emu.flash-chip] */
static void exec_wrsr(M25Chip *c, bool vol)
{
    unsigned first = c->cmd == 0x01 ? 0 : c->cmd == 0x31 ? 1 : 2;
    unsigned max = c->cmd == 0x01 && has(c, F_WRSR_2B) ? 2 : 1;
    uint8_t sr[3];
    uint8_t mask = 0;
    unsigned i;

    if (c->count == 0) {
        return;
    }
    if (c->count > max) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: status register write %02xh of "
                      "%u bytes; not executed\n", c->info->name, c->cmd,
                      c->count);
        return;
    }
    if (!vol && !c->wel) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: status register write without "
                      "write enable; ignored\n", c->info->name);
        return;
    }
    if (!sr_writable(c)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: status registers are protected; "
                      "write ignored\n", c->info->name);
        return;
    }
    memcpy(sr, vol ? c->sr : c->sr_nv, sizeof(sr));
    for (i = 0; i < c->count; i++) {
        unsigned n = first + i;
        uint8_t w = c->info->sr_writable[n];
        uint8_t v = (sr[n] & ~w) | (c->in[i] & w);

        /* OTP bits, and SRL, never return from 1 to 0. */
        v |= sr[n] & c->info->sr_otp[n];
        if (n == 1 && winbond(c)) {
            v |= sr[n] & W_SR2_SRL;
        }
        if (n == 1 && c->qpi) {
            v |= sr[n] & SR2_QE;
        }
        sr[n] = v;
        mask |= BIT(n);
    }
    if (vol) {
        for (i = 0; i < 3; i++) {
            if (mask & BIT(i)) {
                c->sr[i] = sr[i];
            }
        }
        return;
    }
    memcpy(c->op.sr, sr, sizeof(sr));
    c->op.sr_mask = mask;
    op_start(c, OP_WRSR, 0, 0, t(c, c->info->t_w));
}

/* [spec:nuos:req:emu.flash-timing] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static void exec_suspend(M25Chip *c)
{
    int64_t now = now_ns();

    if (!busy(c) || c->suspending || c->susp.kind) {
        return;
    }
    if (c->op.kind != OP_PROGRAM && c->op.kind != OP_ERASE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: suspend during a cycle that "
                      "cannot be suspended; ignored\n", c->info->name);
        return;
    }
    if (c->resumed_at != INT64_MIN &&
        now - c->resumed_at < c->info->t_sus_gap) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: suspend %" PRId64 " ns after a "
                      "resume, sooner than %" PRId64 " ns; ignored\n",
                      c->info->name, now - c->resumed_at,
                      c->info->t_sus_gap);
        return;
    }
    c->suspending = true;
    c->sus_at = now + c->info->t_sus;
    op_schedule(c);
}

/* [spec:nuos:req:emu.flash-timing] */
static void exec_resume(M25Chip *c)
{
    int64_t now = now_ns();

    if (busy(c) || !c->susp.kind) {
        return;
    }
    c->op = c->susp;
    c->susp.kind = OP_NONE;
    c->op.started = now;
    trace_m25chip_resume(c->info->name, c->op.kind,
                         c->op.total - c->op.done);
    c->resumed_at = now;
    op_schedule(c);
    update_romd(c);
}

/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static void exec_reset(M25Chip *c)
{
    bool erase = ops_cut(c, false);

    reset_volatile(c);
    deafen(c, erase ? c->info->t_rst_erase : c->info->t_rst);
}

/* [spec:nuos:req:emu.flash-chip] */
static void exec_lock(M25Chip *c)
{
    if (!c->wel) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: block lock command %02xh without "
                      "write enable; ignored\n", c->info->name, c->cmd);
        return;
    }
    switch (c->cmd) {
    case 0x7e:
    case 0x98:
        memset(c->locks, c->cmd == 0x7e, sizeof(c->locks));
        break;
    case 0x36:
    case 0x39:
        if (c->addr_got == c->addr_len) {
            c->locks[lock_index(c, c->addr & (c->info->size - 1))] =
                c->cmd == 0x36;
        }
        break;
    }
}

/* [spec:nuos:req:emu.flash-chip] */
/* [spec:nuos:req:emu.esp32p4.flash-chip] */
static void execute(M25Chip *c)
{
    uint8_t cmd = c->cmd;
    bool vol = c->vsr_we;

    if (c->phase == PH_IGNORE) {
        c->vsr_we = false;
        return;
    }
    if (cmd != 0x50) {
        c->vsr_we = false;
    }
    /*
     * Continuous read mode: M5-4 = 10b keeps the command for the next
     * transaction, which starts with its address; anything else in M ends
     * it. M is not latched until it is clocked in.
     */
    if ((cmd == 0xbb || cmd == 0xbc || cmd == 0xeb || cmd == 0xec ||
         cmd == 0xe7) &&
        c->mode != 0xff) {
        c->cont_cmd = c->mode == 0x20 ? cmd : 0;
    }
    switch (cmd) {
    case 0x06:
        c->wel = true;
        break;
    case 0x04:
        c->wel = false;
        break;
    case 0x50:
        c->vsr_we = true;
        break;
    case 0x01:
    case 0x31:
    case 0x11:
        exec_wrsr(c, vol);
        break;
    case 0x02:
    case 0x12:
    case 0x32:
    case 0x34:
    case 0xf2:
    case 0x42:
        exec_program(c);
        break;
    case 0x20: case 0x21: case 0x52: case 0x5c: case 0xd8: case 0xdc:
        exec_erase(c);
        break;
    case 0xc7:
    case 0x60:
        if (c->count == 0) {
            exec_erase(c);
        }
        break;
    case 0x44:
        exec_secreg_erase(c);
        break;
    case 0x75:
        exec_suspend(c);
        break;
    case 0x7a:
        exec_resume(c);
        break;
    case 0x66:
        c->reset_enable = true;
        return;
    case 0x99:
        if (c->reset_enable) {
            exec_reset(c);
        }
        break;
    case 0xb9:
        if (c->count == 0) {
            c->powered_down = true;
            c->hpm = false;
            deafen(c, c->info->t_dp);
        }
        break;
    case 0xab:
        c->hpm = false;
        if (c->powered_down) {
            c->powered_down = false;
            deafen(c, c->count ? c->info->t_res2 : c->info->t_res1);
        }
        break;
    case 0xa3:
        /* Its three dummy bytes clocked in */
        if (c->phase == PH_DATA) {
            c->hpm = true;
        }
        break;
    case 0x38:
        c->qpi = true;
        break;
    case 0xff:
        c->qpi = false;
        break;
    case 0xc0:
        if (c->count) {
            c->read_params = c->qpi ? c->in[0] & 0x73
                                    : (c->read_params & 0x03) |
                                      (c->in[0] & 0x70);
        }
        break;
    case 0x77:
        if (c->count) {
            c->wrap = c->in[0] & 0x70;
        }
        break;
    case 0xb7:
        c->four_byte = true;
        break;
    case 0xe9:
        c->four_byte = false;
        break;
    case 0xc5:
        if (c->count && c->wel) {
            c->ear = c->in[0] & 1;
        }
        break;
    case 0x7e: case 0x98: case 0x36: case 0x39:
        exec_lock(c);
        break;
    }
    c->reset_enable = false;
}

/* [spec:nuos:req:emu.flash-chip] */
void m25chip_set_cs(M25Chip *c, bool high)
{
    catch_up(c);
    if (high) {
        if (c->selected) {
            execute(c);
        }
        c->selected = false;
        c->phase = PH_CMD;
        return;
    }
    c->selected = true;
    c->phase = PH_CMD;
    if (c->cont_cmd) {
        /* The address comes first; the chip ignores it while busy. */
        uint8_t cmd = c->cont_cmd;

        if (!accepted(c, cmd)) {
            c->cmd = cmd;
            c->phase = PH_IGNORE;
            return;
        }
        begin_command(c, cmd);
    }
}

/* Whether IO2 and IO3 are data lines (QE set) rather than WP# and HOLD# */
bool m25chip_quad_enabled(M25Chip *c)
{
    return quad_enabled(c);
}

/* Whether the chip drives its outputs for the next byte clocked */
bool m25chip_drives_output(M25Chip *c)
{
    catch_up(c);
    if (!c->selected) {
        return false;
    }
    switch (c->phase) {
    case PH_OUT:
    case PH_ARRAY:
    case PH_SECREG:
    case PH_SFDP:
        return true;
    }
    return false;
}

/* The read command continuous read mode keeps for the next transaction */
uint8_t m25chip_continuous_cmd(M25Chip *c)
{
    return c->cont_cmd;
}

/* [spec:nuos:req:emu.flash-timing] */
int64_t m25chip_busy_until(M25Chip *c)
{
    int64_t end;

    catch_up(c);
    if (!busy(c)) {
        return 0;
    }
    end = op_end(c);
    return c->suspending ? MIN(end, c->sus_at) : end;
}

/* Power */

/* A machine reset: the chip's power cycles. */
/* [spec:nuos:req:emu.flash-power-cut] */
void m25chip_reset(M25Chip *c)
{
    ops_cut(c, false);
    power_up(c);
    update_romd(c);
}

/*
 * The chip's supply comes up after having been off: as a power cycle, and
 * the chip takes no command before tVSL.
 */
/* [spec:nuos:req:emu.flash-power-cut] */
void m25chip_power_on(M25Chip *c)
{
    m25chip_reset(c);
    if (c->info->t_vsl) {
        deafen(c, c->info->t_vsl);
    }
}

/* QEMU shuts down: the chip loses power. */
/* [spec:nuos:req:emu.flash-power-cut] */
static void m25chip_shutdown(Notifier *n, void *opaque)
{
    M25Chip *c = container_of(n, M25Chip, shutdown);

    ops_cut(c, true);
}

/* [spec:nuos:req:emu.flash-chip] */
bool m25chip_realize(M25Chip *c, Error **errp)
{
    const char *uid = c->uid_prop && *c->uid_prop ? c->uid_prop
                                                   : c->info->uid_default;
    unsigned i;

    if (strlen(uid) != 2 * c->info->uid_len) {
        error_setg(errp, "unique-id must be %u hex digits for the %s",
                   2 * c->info->uid_len, c->info->name);
        return false;
    }
    for (i = 0; i < c->info->uid_len; i++) {
        int hi = g_ascii_xdigit_value(uid[2 * i]);
        int lo = g_ascii_xdigit_value(uid[2 * i + 1]);

        if (hi < 0 || lo < 0) {
            error_setg(errp, "unique-id must be hex digits");
            return false;
        }
        c->uid[i] = hi << 4 | lo;
    }
    c->op_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, op_timer_cb, c);
    c->deaf_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, deaf_end, c);
    c->shutdown.notify = m25chip_shutdown;
    qemu_register_shutdown_notifier(&c->shutdown);

    memcpy(c->sr_nv, c->info->sr_default, sizeof(c->sr_nv));
    memset(c->secreg, 0xff, sizeof(c->secreg));
    c->op.kind = OP_NONE;
    c->susp.kind = OP_NONE;
    power_up(c);
    return true;
}

void m25chip_unrealize(M25Chip *c)
{
    notifier_remove(&c->shutdown);
    timer_free(c->op_timer);
    timer_free(c->deaf_timer);
}

static const VMStateDescription vmstate_m25chip_op = {
    .name = "m25p80-chip-op",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(kind, M25ChipOp),
        VMSTATE_UINT32(addr, M25ChipOp),
        VMSTATE_UINT32(len, M25ChipOp),
        VMSTATE_INT64(total, M25ChipOp),
        VMSTATE_INT64(done, M25ChipOp),
        VMSTATE_INT64(started, M25ChipOp),
        VMSTATE_UINT8_ARRAY(data, M25ChipOp, M25CHIP_PAGE_SIZE),
        VMSTATE_UINT8_ARRAY(sr, M25ChipOp, 3),
        VMSTATE_UINT8(sr_mask, M25ChipOp),
        VMSTATE_END_OF_LIST()
    }
};

static int m25chip_post_load(void *opaque, int version_id)
{
    update_romd(opaque);
    return 0;
}

const VMStateDescription vmstate_m25chip = {
    .name = "m25p80-chip",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = m25chip_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(sr, M25Chip, 3),
        VMSTATE_UINT8_ARRAY(sr_nv, M25Chip, 3),
        VMSTATE_BOOL(wel, M25Chip),
        VMSTATE_BOOL(vsr_we, M25Chip),
        VMSTATE_BOOL(reset_enable, M25Chip),
        VMSTATE_BOOL(qpi, M25Chip),
        VMSTATE_BOOL(four_byte, M25Chip),
        VMSTATE_UINT8(ear, M25Chip),
        VMSTATE_UINT8(read_params, M25Chip),
        VMSTATE_UINT8(wrap, M25Chip),
        VMSTATE_UINT8(cont_cmd, M25Chip),
        VMSTATE_BOOL(powered_down, M25Chip),
        VMSTATE_INT64(deaf_until, M25Chip),
        VMSTATE_UINT8_ARRAY(locks, M25Chip, M25CHIP_LOCKS),
        VMSTATE_UINT8_2DARRAY(secreg, M25Chip, M25CHIP_SECREGS,
                              M25CHIP_SECREG_MAX),
        VMSTATE_STRUCT(op, M25Chip, 1, vmstate_m25chip_op, M25ChipOp),
        VMSTATE_STRUCT(susp, M25Chip, 1, vmstate_m25chip_op, M25ChipOp),
        VMSTATE_BOOL(suspending, M25Chip),
        VMSTATE_INT64(sus_at, M25Chip),
        VMSTATE_INT64(resumed_at, M25Chip),
        VMSTATE_BOOL(selected, M25Chip),
        VMSTATE_UINT8(phase, M25Chip),
        VMSTATE_UINT8(cmd, M25Chip),
        VMSTATE_UINT8(addr_len, M25Chip),
        VMSTATE_UINT8(addr_got, M25Chip),
        VMSTATE_UINT32(addr, M25Chip),
        VMSTATE_UINT8(mode, M25Chip),
        VMSTATE_UINT8(skip, M25Chip),
        VMSTATE_UINT32(count, M25Chip),
        VMSTATE_UINT8_ARRAY(out, M25Chip, M25CHIP_UID_MAX),
        VMSTATE_UINT8(out_len, M25Chip),
        VMSTATE_UINT8(out_pos, M25Chip),
        VMSTATE_UINT8_ARRAY(in, M25Chip, 3),
        VMSTATE_UINT8_ARRAY(pp, M25Chip, M25CHIP_PAGE_SIZE),
        VMSTATE_UINT8_ARRAY(pp_set, M25Chip, M25CHIP_PAGE_SIZE / 8),
        VMSTATE_TIMER_PTR(op_timer, M25Chip),
        VMSTATE_TIMER_PTR(deaf_timer, M25Chip),
        VMSTATE_BOOL_V(hpm, M25Chip, 2),
        VMSTATE_END_OF_LIST()
    }
};
