/*
 * RP2350 one-time-programmable storage (OTP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "OTP" (the address map, lock shim, power-up
 * state machine, critical flags, page locks, access keys, ECC and BRP,
 * RMA) and errata RP2350-E17 and RP2350-E28; register layout and reset
 * values from the pico-sdk otp.h and otp_data.h. The SBPI programming
 * sequence follows the RP2350 boot ROM's otp_access, which is the only
 * published description of the Synopsys macro's SBPI protocol.
 *
 * Two windows:
 *
 *  - The control registers at OTP_BASE, with the usual atomic aliases.
 *    Everything but the SW_LOCKn registers is Secure-only.
 *  - The read data window at OTP_DATA_BASE: bit 14 selects ECC (0) or raw
 *    (1) reads, bit 15 unguarded (0) or guarded (1). ECC reads return the
 *    corrected 16-bit data of the even/odd row pair a 32-bit word covers;
 *    raw reads return one 24-bit row. Narrow reads take their byte lanes
 *    from the aligned word.
 *
 * Programming goes through the SBPI bridge: SBPI_INSTR sends register
 * reads and writes to the DAP and PMC, and the PMC START command programs
 * the DAP's data registers into the row its address registers select.
 * Fuses only go from 0 to 1. The SBPI instructions complete as they are
 * issued: the macro's programming time is not documented, so the PMC
 * never reads as busy.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_otp.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"
#include <zlib.h>

/* Control registers. */
#define A_SW_LOCK0          0x000
#define A_SW_LOCK63         0x0fc
#define A_SBPI_INSTR        0x100
#define A_SBPI_WDATA_0      0x104
#define A_SBPI_WDATA_3      0x110
#define A_SBPI_RDATA_0      0x114
#define A_SBPI_RDATA_3      0x120
#define A_SBPI_STATUS       0x124
#define A_USR               0x128
#define A_DBG               0x12c
#define A_BIST              0x134
#define A_CRT_KEY_W0        0x138
#define A_CRT_KEY_W3        0x144
#define A_CRITICAL          0x148
#define A_KEY_VALID         0x14c
#define A_DEBUGEN           0x150
#define A_DEBUGEN_LOCK      0x154
#define A_ARCHSEL           0x158
#define A_ARCHSEL_STATUS    0x15c
#define A_BOOTDIS           0x160
#define A_INTR              0x164
#define A_INTE              0x168
#define A_INTF              0x16c
#define A_INTS              0x170

#define SW_LOCK_SEC         0x3u
#define SW_LOCK_NSEC        0xcu
#define SW_LOCK_MASK        0xfu

#define SBPI_INSTR_EXEC     (1u << 30)
#define SBPI_INSTR_IS_WR    (1u << 29)
#define SBPI_INSTR_HAS_PAYLOAD (1u << 28)
#define SBPI_INSTR_MASK     0x7fffffffu

#define SBPI_STATUS_FLAG    (1u << 12)
#define SBPI_STATUS_INSTR_MISS (1u << 8)
#define SBPI_STATUS_INSTR_DONE (1u << 4)
#define SBPI_STATUS_RDATA_VLD  (1u << 0)
#define SBPI_STATUS_WC      (SBPI_STATUS_INSTR_MISS | SBPI_STATUS_INSTR_DONE | \
                             SBPI_STATUS_RDATA_VLD)

#define USR_PD              (1u << 4)
#define USR_DCTRL           (1u << 0)
#define USR_MASK            (USR_PD | USR_DCTRL)

#define DBG_CUSTOMER_RMA_FLAG (1u << 12)
#define DBG_ROSC_UP_SEEN    (1u << 2)
#define DBG_BOOT_DONE       (1u << 1)
#define DBG_PSM_DONE        (1u << 0)

#define BIST_CNT_CLR        (1u << 29)
#define BIST_RW_MASK        0x1fff0000u
#define BIST_RESET          0x0fff0000u

#define CRITICAL_SECURE_BOOT_ENABLE   (1u << 0)
#define CRITICAL_SECURE_DEBUG_DISABLE (1u << 1)
#define CRITICAL_DEBUG_DISABLE        (1u << 2)
#define CRITICAL_DEFAULT_ARCHSEL      (1u << 3)
#define CRITICAL_GLITCH_DETECTOR_ENABLE (1u << 4)
#define CRITICAL_ARM_DISABLE          (1u << 16)
#define CRITICAL_RISCV_DISABLE        (1u << 17)

#define DEBUGEN_PROC0       (1u << 0)
#define DEBUGEN_PROC0_SECURE (1u << 1)
#define DEBUGEN_PROC1       (1u << 2)
#define DEBUGEN_PROC1_SECURE (1u << 3)
#define DEBUGEN_MISC        (1u << 8)
#define DEBUGEN_MASK        0x10fu
#define DEBUGEN_SECURE      (DEBUGEN_PROC0_SECURE | DEBUGEN_PROC1_SECURE)

#define ARCHSEL_MASK        0x3u

#define BOOTDIS_NEXT        (1u << 1)
#define BOOTDIS_NOW         (1u << 0)

#define INTR_APB_RD_NSEC_FAIL (1u << 4)
#define INTR_APB_RD_SEC_FAIL  (1u << 3)
#define INTR_APB_DCTRL_FAIL   (1u << 2)
#define INTR_SBPI_WR_FAIL     (1u << 1)
#define INTR_SBPI_FLAG_N      (1u << 0)
#define INTR_WC               0x1eu
#define INT_MASK              0x1fu

/* SBPI targets and the registers the boot ROM's programming path uses. */
#define SBPI_TARGET_DAP     0x02
#define SBPI_TARGET_PMC     0x3a
#define SBPI_CMD_REG_MASK   0xc0
#define SBPI_CMD_REG_READ   0x80
#define SBPI_CMD_REG_WRITE  0xc0
#define SBPI_CMD_REG        0x3f
#define DAP_DR0             0x00
#define DAP_DR1             0x01
#define DAP_ECC             0x20
#define DAP_DPCR            0x3a
#define DAP_CQ0             0x3c
#define DAP_CQ1             0x3d
#define DPCR_ECCDIS         (1u << 0)
#define DPCR_BRPDIS         (1u << 3)
#define PMC_CQ              0x3c
#define PMC_CQ_SKIP_ECC     (1u << 1)
#define PMC_CQ_SKIP_BRP     (1u << 3)
#define PMC_CTRL_STATUS     0x3f
#define PMC_CMD_START       0x01
#define PMC_CMD_STOP        0x02

/* Read data window. */
#define DATA_RAW            0x4000
#define DATA_GUARDED        0x8000
#define DATA_ECC_SIZE       0x2000

/* Rows with a hardware or factory meaning. */
#define ROW_CHIPID0         0x000
#define ROW_RANDID0         0x004
#define ROW_ROSC_CALIB      0x010
#define ROW_LPOSC_CALIB     0x011
#define ROW_NUM_GPIOS       0x018
#define ROW_INFO_CRC0       0x036
#define ROW_INFO_CRC1       0x037
#define ROW_CRIT0           0x038
#define ROW_CRIT1           0x040
#define ROW_KEY1_0          0xf48
#define ROW_KEY1_VALID      0xf79
#define ROW_PAGE0_LOCK0     0xf80
#define ROWS_PER_PAGE       64
#define ROWS_PER_KEY        8
#define RBIT8_ROWS          8
#define ROW_MASK            0xffffffu

#define PAGE_LOCK1(page)    (ROW_PAGE0_LOCK0 + 2 * (page) + 1)
#define PAGE_LOCK0(page)    (ROW_PAGE0_LOCK0 + 2 * (page))

/* Lock word fields (the voted LOCK0 byte). */
#define LOCK0_KEY_W(v)      ((v) & 0x7)
#define LOCK0_KEY_R(v)      (((v) >> 3) & 0x7)
#define LOCK0_NO_KEY_INACCESSIBLE (1u << 6)
#define LOCK0_RMA           (1u << 7)

/* Keys 5 and 6 double as the Secure and Non-secure debug keys. */
#define DEBUG_KEYS          ((1u << 4) | (1u << 5))

/* Lock levels, in order of progression. */
enum {
    LOCK_RW,
    LOCK_RO,
    LOCK_INACCESSIBLE,
};

/* ECC decode outcomes. */
enum {
    ECC_OK,
    ECC_CORRECTED,
    ECC_UNCORRECTABLE,
};

/*
 * The modified Hamming code over the 16 data bits: five parity bits, then
 * an overall parity bit over the data and those five, in bits 21:16.
 */
static const uint32_t ecc_parity_table[6] = {
    0x00ad5b, 0x00366d, 0x00c78e, 0x0007f0, 0x00f800, 0x1fffff,
};

static uint32_t even_parity(uint32_t v)
{
    return ctpop32(v) & 1;
}

static uint32_t otp_ecc_encode(uint16_t data)
{
    uint32_t p = data;
    int i;

    for (i = 0; i < ARRAY_SIZE(ecc_parity_table); i++) {
        p |= even_parity(p & ecc_parity_table[i]) << (16 + i);
    }
    return p;
}

/*
 * Programming an ECC row: if a bit already set in the row is clear in the
 * target, bit repair by polarity programs the inverted target with both
 * BRP bits set instead.
 */
static uint32_t otp_ecc_brp(uint32_t target, uint32_t current)
{
    if (current & ~target) {
        return (~target & 0x3fffff) | 0xc00000;
    }
    return target;
}

/*
 * Reading an ECC row: BRP first (both bits 23:22 set invert the row),
 * then the Hamming syndrome. An odd overall parity is a single flip, of
 * the data bit whose syndrome matches or of a parity bit; an even overall
 * parity with a nonzero syndrome is a double error.
 */
static int otp_ecc_decode(uint32_t raw, uint16_t *data)
{
    uint32_t r = raw & ROW_MASK;
    uint32_t syndrome = 0;
    int i;

    if ((r >> 22) == 3) {
        r = ~r & ROW_MASK;
    }
    *data = r & 0xffff;
    for (i = 0; i < 5; i++) {
        syndrome |= (even_parity(r & ecc_parity_table[i]) ^
                     ((r >> (16 + i)) & 1)) << i;
    }
    if (!even_parity(r & 0x3fffff)) {
        return syndrome ? ECC_UNCORRECTABLE : ECC_OK;
    }
    if (is_power_of_2(syndrome) || syndrome == 0) {
        /* A parity bit flipped; the data is intact. */
        return ECC_CORRECTED;
    }
    for (i = 0; i < 16; i++) {
        uint32_t column = 0;
        int k;

        for (k = 0; k < 5; k++) {
            column |= ((ecc_parity_table[k] >> i) & 1) << k;
        }
        if (column == syndrome) {
            *data ^= 1u << i;
            return ECC_CORRECTED;
        }
    }
    return ECC_UNCORRECTABLE;
}

/* Bitwise 3-way majority vote across the three bytes of a raw row. */
static uint32_t otp_vote3(uint32_t row)
{
    uint32_t a = row & 0xff, b = (row >> 8) & 0xff, c = (row >> 16) & 0xff;

    return (a & b) | (b & c) | (a & c);
}

/* A critical flag is set when at least three of its eight copies are. */
static uint32_t otp_vote_rbit8(RP2350OTPState *s, int row)
{
    uint32_t v = 0;
    int bit, i;

    for (bit = 0; bit < 24; bit++) {
        int n = 0;

        for (i = 0; i < RBIT8_ROWS; i++) {
            n += (s->fuse[row + i] >> bit) & 1;
        }
        if (n >= 3) {
            v |= 1u << bit;
        }
    }
    return v;
}

static unsigned lock_level(uint32_t field)
{
    /* The thermometer code's "do not use" value 2 is inaccessible too. */
    if (field & 2) {
        return LOCK_INACCESSIBLE;
    }
    return field & 1 ? LOCK_RO : LOCK_RW;
}

static bool otp_key_entered(RP2350OTPState *s, unsigned index)
{
    /* Index 7 has no key and never matches. */
    if (index < 1 || index > RP2350_OTP_KEYS ||
        !(s->key_valid & (1u << (index - 1)))) {
        return false;
    }
    return !memcmp(s->crt_key, s->key[index - 1], sizeof(s->crt_key));
}

/*
 * The access-key lock level of a row. Per RP2350-E28 the hardware looks
 * up the key configuration of lock word row / 64 for every row, so the
 * lock words themselves (pages 62 and 63) take theirs from lock words 62
 * and 63.
 */
static unsigned otp_key_level(RP2350OTPState *s, int row)
{
    uint32_t cfg = s->key_cfg[row / ROWS_PER_PAGE];
    unsigned key_w = LOCK0_KEY_W(cfg), key_r = LOCK0_KEY_R(cfg);

    if (!key_w && !key_r) {
        return LOCK_RW;
    }
    if (key_w && otp_key_entered(s, key_w)) {
        return LOCK_RW;
    }
    if (key_r && otp_key_entered(s, key_r)) {
        return LOCK_RO;
    }
    return cfg & LOCK0_NO_KEY_INACCESSIBLE ? LOCK_INACCESSIBLE : LOCK_RO;
}

/*
 * The lock shim: the lock level of `row` for a Secure or Non-secure
 * access. Lock words (rows 0xf80 up) are always readable and protect only
 * their own writes; a valid key's rows are inaccessible; RMA makes the
 * user pages inaccessible.
 */
/* [spec:nuos:req:emu.otp] */
static unsigned otp_row_level(RP2350OTPState *s, int row, bool secure,
                              bool write)
{
    int key_index = (row - ROW_KEY1_0) / ROWS_PER_KEY;
    unsigned level, key;
    int page;

    if (row >= ROW_KEY1_0 && key_index < RP2350_OTP_KEYS &&
        (s->key_valid & (1u << key_index))) {
        return LOCK_INACCESSIBLE;
    }
    if (row >= ROW_PAGE0_LOCK0) {
        page = (row - ROW_PAGE0_LOCK0) / 2;
        level = write ? lock_level(s->sw_lock[page] & SW_LOCK_SEC) : LOCK_RW;
    } else {
        page = row / ROWS_PER_PAGE;
        level = lock_level(secure ? s->sw_lock[page] & SW_LOCK_SEC
                                  : (s->sw_lock[page] & SW_LOCK_NSEC) >> 2);
        if (s->rma && page >= 3 && page <= 60) {
            level = LOCK_INACCESSIBLE;
        }
    }
    key = otp_key_level(s, row);
    return MAX(level, key);
}

static bool otp_secure(MemTxAttrs attrs)
{
    /* Debugger and qtest accesses count as Secure. */
    return attrs.secure || attrs.unspecified;
}

static void otp_update_irq(RP2350OTPState *s)
{
    qemu_set_irq(s->irq, ((s->intr | s->intf) & s->inte) != 0);
}

/*
 * The CoreSight debug enables. The DEBUG_DISABLE flag, or an enrolled
 * debug key (whose value can only be supplied over SWD, which the machine
 * has not got), disables all debug unless DEBUGEN re-enables all of it;
 * SECURE_DEBUG_DISABLE disables Secure debug unless DEBUGEN re-enables
 * Secure debug of both cores.
 */
static void otp_update_debug(RP2350OTPState *s)
{
    bool all_off = (s->critical & CRITICAL_DEBUG_DISABLE) ||
                   (s->key_valid & DEBUG_KEYS);
    bool secure_off = all_off ||
                      (s->critical & CRITICAL_SECURE_DEBUG_DISABLE);

    qemu_set_irq(s->debug_disable,
                 all_off && (s->debugen & DEBUGEN_MASK) != DEBUGEN_MASK);
    qemu_set_irq(s->secure_debug_disable,
                 secure_off &&
                 (s->debugen & DEBUGEN_SECURE) != DEBUGEN_SECURE);
    qemu_set_irq(s->glitch_detector_enable,
                 !!(s->critical & CRITICAL_GLITCH_DETECTOR_ENABLE));
}

static void otp_persist_row(RP2350OTPState *s, int row)
{
    uint32_t le = cpu_to_le32(s->fuse[row]);

    if (!s->blk_writable) {
        return;
    }
    if (blk_pwrite(s->blk, row * 4, 4, &le, 0) < 0) {
        error_report("%s: failed to write OTP row 0x%03x", blk_name(s->blk),
                     row);
    }
}

/* Fuses can only be set. */
/* [spec:nuos:req:emu.otp] */
static void otp_program_row(RP2350OTPState *s, int row, uint32_t value)
{
    uint32_t old = s->fuse[row];

    s->fuse[row] |= value & ROW_MASK;
    if (s->fuse[row] != old) {
        otp_persist_row(s, row);
    }
}

/*
 * PMC START: program the DAP's 24-bit data into the row its address
 * registers select. Raw unless both the DAP datapath and the PMC sequence
 * enable ECC generation, in which case the hardware encodes the low 16
 * bits and applies bit repair by polarity against the row's current
 * contents. SBPI accesses count as Secure for the lock shim.
 */
/* [spec:nuos:req:emu.otp] */
static void otp_pmc_start(RP2350OTPState *s)
{
    int row = ((s->dap[DAP_CQ1] << 8) | s->dap[DAP_CQ0]) % RP2350_OTP_ROWS;
    uint32_t value = s->dap[DAP_ECC] << 16 | s->dap[DAP_DR1] << 8 |
                     s->dap[DAP_DR0];

    if (!(s->dap[DAP_DPCR] & DPCR_ECCDIS) &&
        !(s->pmc[PMC_CQ] & PMC_CQ_SKIP_ECC)) {
        value = otp_ecc_encode(value & 0xffff);
        if (!(s->dap[DAP_DPCR] & DPCR_BRPDIS) &&
            !(s->pmc[PMC_CQ] & PMC_CQ_SKIP_BRP)) {
            value = otp_ecc_brp(value, s->fuse[row]);
        }
    }
    if (otp_row_level(s, row, true, true) != LOCK_RW) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: programming row 0x%03x "
                      "refused by its page lock\n", row);
        s->intr |= INTR_SBPI_WR_FAIL;
        otp_update_irq(s);
        return;
    }
    otp_program_row(s, row, value);
}

static uint8_t otp_sbpi_reg_read(RP2350OTPState *s, uint8_t *regs, int n)
{
    /* The PMC's status: never busy, as programming completes at once. */
    if (regs == s->pmc && n == PMC_CTRL_STATUS) {
        return 0;
    }
    return regs[n];
}

/* [spec:nuos:req:emu.otp] */
static void otp_sbpi_exec(RP2350OTPState *s, uint32_t instr)
{
    unsigned target = (instr >> 16) & 0xff;
    unsigned cmd = (instr >> 8) & 0xff;
    unsigned len = ((instr >> 24) & 0xf) + 1;
    bool payload = instr & SBPI_INSTR_HAS_PAYLOAD;
    bool is_wr = instr & SBPI_INSTR_IS_WR;
    uint8_t *regs;
    unsigned i;

    if (s->usr & USR_DCTRL) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: SBPI instruction 0x%08x "
                      "dropped: USR.DCTRL gives the array to the data "
                      "interface\n", instr);
        s->sbpi_status |= SBPI_STATUS_INSTR_MISS;
        return;
    }
    s->sbpi_status |= SBPI_STATUS_INSTR_DONE;
    switch (target) {
    case SBPI_TARGET_DAP:
        regs = s->dap;
        break;
    case SBPI_TARGET_PMC:
        regs = s->pmc;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: SBPI instruction to "
                      "unknown target 0x%02x\n", target);
        return;
    }

    if (payload && is_wr && (cmd & SBPI_CMD_REG_MASK) == SBPI_CMD_REG_WRITE) {
        for (i = 0; i < len; i++) {
            uint8_t byte = len == 1 ? instr & 0xff
                                    : s->sbpi_wdata[i / 4] >> (8 * (i % 4));

            regs[((cmd & SBPI_CMD_REG) + i) % RP2350_OTP_SBPI_REGS] = byte;
        }
        return;
    }
    if (payload && !is_wr && (cmd & SBPI_CMD_REG_MASK) == SBPI_CMD_REG_READ) {
        memset(s->sbpi_rdata, 0, sizeof(s->sbpi_rdata));
        for (i = 0; i < len; i++) {
            uint8_t byte = otp_sbpi_reg_read(s, regs,
                ((cmd & SBPI_CMD_REG) + i) % RP2350_OTP_SBPI_REGS);

            s->sbpi_rdata[i / 4] |= (uint32_t)byte << (8 * (i % 4));
        }
        s->sbpi_status = (s->sbpi_status & ~0xff0000u) |
                         (s->sbpi_rdata[0] & 0xff) << 16 |
                         SBPI_STATUS_RDATA_VLD;
        return;
    }
    if (!payload && target == SBPI_TARGET_PMC) {
        switch (cmd) {
        case PMC_CMD_START:
            otp_pmc_start(s);
            return;
        case PMC_CMD_STOP:
            return;
        }
    }
    qemu_log_mask(LOG_UNIMP, "rp2350-otp: SBPI instruction 0x%08x (target "
                  "0x%02x command 0x%02x) not implemented\n", instr, target,
                  cmd);
}

/*
 * The power-up state machine, run at every OTP reset: latch the critical
 * flags (three-of-eight votes), the access keys and their valid bits, the
 * RMA flag, and the soft lock state and key configuration from the lock
 * words. Programming any of these takes effect at the next OTP reset.
 */
/* [spec:nuos:req:emu.otp] */
static void otp_psm(RP2350OTPState *s)
{
    uint32_t crit0 = otp_vote_rbit8(s, ROW_CRIT0);
    uint32_t crit1 = otp_vote_rbit8(s, ROW_CRIT1);
    int i, j;

    if ((crit0 & ~0x3u) || (crit1 & ~0x7fu)) {
        qemu_log_mask(LOG_UNIMP, "rp2350-otp: reserved CRIT0/CRIT1 bits are "
                      "set, which makes hardware rerun its power-up state "
                      "machine forever; the machine boots regardless\n");
    }
    s->critical = (crit1 & 0x7f) | (crit0 & 0x3) << 16;

    s->key_valid = 0;
    for (i = 0; i < RP2350_OTP_KEYS; i++) {
        uint32_t valid = s->fuse[ROW_KEY1_VALID + i];

        if (otp_vote3(valid) & 1) {
            s->key_valid |= 1u << i;
        }
        for (j = 0; j < ROWS_PER_KEY; j++) {
            uint16_t half;

            otp_ecc_decode(s->fuse[ROW_KEY1_0 + i * ROWS_PER_KEY + j], &half);
            if (j % 2 == 0) {
                s->key[i][j / 2] = half;
            } else {
                s->key[i][j / 2] |= (uint32_t)half << 16;
            }
        }
    }

    s->rma = otp_vote3(s->fuse[PAGE_LOCK0(63)]) & LOCK0_RMA;
    for (i = 0; i < RP2350_OTP_PAGES; i++) {
        s->sw_lock[i] = otp_vote3(s->fuse[PAGE_LOCK1(i)]) & SW_LOCK_MASK;
        s->key_cfg[i] = otp_vote3(s->fuse[PAGE_LOCK0(i)]);
    }

    /*
     * ARCHSEL resets to the default architecture unless a flag forces Arm.
     * ARM_DISABLE has no effect from A3 on.
     */
    s->archsel = 0;
    if ((s->critical & CRITICAL_RISCV_DISABLE) &&
        (s->critical & CRITICAL_DEFAULT_ARCHSEL)) {
        qemu_log_mask(LOG_UNIMP, "rp2350-otp: RISCV_DISABLE with BOOT_ARCH "
                      "set is an invalid state in which hardware does not "
                      "boot; the machine boots regardless\n");
    } else if (!(s->critical & (CRITICAL_RISCV_DISABLE |
                                CRITICAL_SECURE_BOOT_ENABLE)) &&
               (s->critical & CRITICAL_DEFAULT_ARCHSEL)) {
        s->archsel = ARCHSEL_MASK;
        qemu_log_mask(LOG_UNIMP, "rp2350-otp: BOOT_ARCH selects RISC-V, but "
                      "the machine has no Hazard3 cores; the cores boot as "
                      "Arm\n");
    }
    s->archsel_status = 0;
}

/* The rows written during manufacturing test of every RP2350. */
/* [spec:nuos:req:emu.otp] */
static void otp_factory_program(RP2350OTPState *s)
{
    /* A fixed chip identity, so that runs are reproducible. */
    static const uint16_t chipid[4] = { 0x5a1e, 0x2c93, 0xe7b4, 0x0d68 };
    static const uint16_t randid[8] = {
        0x91c3, 0x4f2a, 0xd805, 0x36be, 0x7b19, 0xa4e0, 0x1fd7, 0xc25c,
    };
    uint8_t info[2 * ROW_INFO_CRC0];
    uint32_t crc;
    int i;

    memset(s->fuse, 0, sizeof(s->fuse));
    for (i = 0; i < ARRAY_SIZE(chipid); i++) {
        s->fuse[ROW_CHIPID0 + i] = otp_ecc_encode(chipid[i]);
    }
    for (i = 0; i < ARRAY_SIZE(randid); i++) {
        s->fuse[ROW_RANDID0 + i] = otp_ecc_encode(randid[i]);
    }
    /* The ROSC in kHz and the LPOSC in Hz at their reset settings. */
    s->fuse[ROW_ROSC_CALIB] = otp_ecc_encode(11000);
    s->fuse[ROW_LPOSC_CALIB] = otp_ecc_encode(32768);
    /* The machine is an RP2350A (QFN-60). */
    s->fuse[ROW_NUM_GPIOS] = otp_ecc_encode(30);

    /* CRC-32 of the ECC data of rows 0x00-0x35 (bytes 0x00-0x6b). */
    for (i = 0; i < ROW_INFO_CRC0; i++) {
        uint16_t half;

        otp_ecc_decode(s->fuse[i], &half);
        info[2 * i] = half & 0xff;
        info[2 * i + 1] = half >> 8;
    }
    crc = crc32(0, info, sizeof(info));
    s->fuse[ROW_INFO_CRC0] = otp_ecc_encode(crc & 0xffff);
    s->fuse[ROW_INFO_CRC1] = otp_ecc_encode(crc >> 16);

    /*
     * Hard locks: page 0 read-only to everyone; pages 1, 2 and 62
     * read-only to Non-secure; page 63 read-only to Non-secure and the
     * bootloader. LOCK1 holds S in bits 1:0, NS in 3:2 and BL in 5:4,
     * triple-redundant across the row's three bytes.
     */
    s->fuse[PAGE_LOCK1(0)] = 0x151515;
    s->fuse[PAGE_LOCK1(1)] = 0x040404;
    s->fuse[PAGE_LOCK1(2)] = 0x040404;
    s->fuse[PAGE_LOCK1(62)] = 0x040404;
    s->fuse[PAGE_LOCK1(63)] = 0x141414;
}

static void otp_data_fault(RP2350OTPState *s, uint32_t intr)
{
    s->intr |= intr;
    otp_update_irq(s);
}

/*
 * Read one 32-bit word of the data window. Returns false for a bus
 * error. The guarded aliases fault where the unguarded ones return
 * all-ones (permission failure), and fault on uncorrectable ECC data in
 * either row of the pair (RP2350-E17).
 */
/* [spec:nuos:req:emu.otp] */
static bool otp_data_word(RP2350OTPState *s, hwaddr addr, MemTxAttrs attrs,
                          uint32_t *value)
{
    bool guarded = addr & DATA_GUARDED;
    bool raw = addr & DATA_RAW;
    bool secure = otp_secure(attrs);
    hwaddr off = addr & (DATA_RAW - 1);
    int row, nrows, i;

    if (!(s->usr & USR_DCTRL)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: data read at 0x%04"
                      HWADDR_PRIx " while USR.DCTRL is clear\n", addr);
        otp_data_fault(s, INTR_APB_DCTRL_FAIL);
        return false;
    }
    if (s->usr & USR_PD) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: data read at 0x%04"
                      HWADDR_PRIx " while USR.PD powers the array down\n",
                      addr);
        return false;
    }
    if (raw) {
        row = off / 4;
        nrows = 1;
    } else {
        if (off >= DATA_ECC_SIZE) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: read of unpopulated "
                          "ECC alias offset 0x%04" HWADDR_PRIx "\n", addr);
            return false;
        }
        row = off / 2;
        nrows = 2;
    }

    for (i = 0; i < nrows; i++) {
        if (otp_row_level(s, row + i, secure, false) == LOCK_INACCESSIBLE) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: %s read of row "
                          "0x%03x refused by its page lock\n",
                          secure ? "Secure" : "Non-secure", row + i);
            otp_data_fault(s, secure ? INTR_APB_RD_SEC_FAIL
                                     : INTR_APB_RD_NSEC_FAIL);
            if (guarded) {
                return false;
            }
            *value = UINT32_MAX;
            return true;
        }
    }

    if (raw) {
        *value = s->fuse[row];
        return true;
    }
    *value = 0;
    for (i = 0; i < nrows; i++) {
        uint16_t half;

        if (otp_ecc_decode(s->fuse[row + i], &half) == ECC_UNCORRECTABLE &&
            guarded) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: guarded read of row "
                          "0x%03x: uncorrectable ECC data\n", row + i);
            return false;
        }
        *value |= (uint32_t)half << (16 * i);
    }
    return true;
}

static MemTxResult rp2350_otp_data_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    RP2350OTPState *s = opaque;
    uint32_t word;

    if (!otp_data_word(s, addr & ~3ull, attrs, &word)) {
        return MEMTX_ERROR;
    }
    *data = extract32(word, (addr & 3) * 8, size * 8);
    return MEMTX_OK;
}

static MemTxResult rp2350_otp_data_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: write to the read-only data "
                  "window at 0x%04" HWADDR_PRIx "\n", addr);
    return MEMTX_ERROR;
}

static const MemoryRegionOps rp2350_otp_data_ops = {
    .read_with_attrs = rp2350_otp_data_read,
    .write_with_attrs = rp2350_otp_data_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint32_t otp_intr(RP2350OTPState *s)
{
    /* SBPI_FLAG_N follows the PMC flag, which never shows busy. */
    return s->intr & ~INTR_SBPI_FLAG_N;
}

/* [spec:nuos:req:emu.otp] */
static MemTxResult rp2350_otp_ctrl_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    RP2350OTPState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v;

    if (reg <= A_SW_LOCK63) {
        /* The soft locks are readable from either security state. */
        *data = s->sw_lock[reg / 4];
        return MEMTX_OK;
    }
    if (!otp_secure(attrs)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: Non-secure read of "
                      "Secure register 0x%03" HWADDR_PRIx "\n", reg);
        return MEMTX_ERROR;
    }
    switch (reg) {
    case A_SBPI_INSTR:
        v = s->sbpi_instr;
        break;
    case A_SBPI_WDATA_0 ... A_SBPI_WDATA_3:
        v = s->sbpi_wdata[(reg - A_SBPI_WDATA_0) / 4];
        break;
    case A_SBPI_RDATA_0 ... A_SBPI_RDATA_3:
        /* Read data clears once read. */
        v = s->sbpi_rdata[(reg - A_SBPI_RDATA_0) / 4];
        s->sbpi_rdata[(reg - A_SBPI_RDATA_0) / 4] = 0;
        break;
    case A_SBPI_STATUS:
        v = s->sbpi_status;
        break;
    case A_USR:
        v = s->usr;
        break;
    case A_DBG:
        v = s->dbg;
        break;
    case A_BIST:
        v = s->bist;
        break;
    case A_CRT_KEY_W0 ... A_CRT_KEY_W3:
        /* Write-only. */
        v = 0;
        break;
    case A_CRITICAL:
        v = s->critical;
        break;
    case A_KEY_VALID:
        v = s->key_valid;
        break;
    case A_DEBUGEN:
        v = s->debugen;
        break;
    case A_DEBUGEN_LOCK:
        v = s->debugen_lock;
        break;
    case A_ARCHSEL:
        v = s->archsel;
        break;
    case A_ARCHSEL_STATUS:
        v = s->archsel_status;
        break;
    case A_BOOTDIS:
        v = s->bootdis;
        break;
    case A_INTR:
        v = otp_intr(s);
        break;
    case A_INTE:
        v = s->inte;
        break;
    case A_INTF:
        v = s->intf;
        break;
    case A_INTS:
        v = (otp_intr(s) | s->intf) & s->inte;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: read of reserved offset "
                      "0x%03" HWADDR_PRIx "\n", reg);
        v = 0;
        break;
    }
    *data = v;
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.otp] */
static MemTxResult rp2350_otp_ctrl_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    RP2350OTPState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    bool secure = otp_secure(attrs);
    /* The bits a write sets, for write-1-to-clear and OR-only fields. */
    uint32_t ones = rp2350_atomic_apply(addr, 0, value);
    uint32_t v;
    int i;

    if (reg <= A_SW_LOCK63) {
        /*
         * Writes OR into the lock state, so it only advances. The Secure
         * field is read-only to Non-secure code.
         */
        i = reg / 4;
        v = rp2350_atomic_apply(addr, s->sw_lock[i], value);
        s->sw_lock[i] |= v & (secure ? SW_LOCK_MASK : SW_LOCK_NSEC);
        return MEMTX_OK;
    }
    if (!secure) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: Non-secure write to "
                      "Secure register 0x%03" HWADDR_PRIx "\n", reg);
        return MEMTX_ERROR;
    }
    switch (reg) {
    case A_SBPI_INSTR:
        v = rp2350_atomic_apply(addr, s->sbpi_instr, value) & SBPI_INSTR_MASK;
        s->sbpi_instr = v & ~SBPI_INSTR_EXEC;
        if (v & SBPI_INSTR_EXEC) {
            otp_sbpi_exec(s, v);
        }
        break;
    case A_SBPI_WDATA_0 ... A_SBPI_WDATA_3:
        i = (reg - A_SBPI_WDATA_0) / 4;
        s->sbpi_wdata[i] = rp2350_atomic_apply(addr, s->sbpi_wdata[i], value);
        break;
    case A_SBPI_STATUS:
        s->sbpi_status &= ~(ones & SBPI_STATUS_WC);
        break;
    case A_USR:
        s->usr = rp2350_atomic_apply(addr, s->usr, value) & USR_MASK;
        break;
    case A_DBG:
        s->dbg &= ~(ones & DBG_ROSC_UP_SEEN);
        break;
    case A_BIST:
        /* No leaky bits: the count stays zero and never fails. */
        v = rp2350_atomic_apply(addr, s->bist, value);
        s->bist = v & BIST_RW_MASK & ~BIST_CNT_CLR;
        break;
    case A_CRT_KEY_W0 ... A_CRT_KEY_W3:
        i = (reg - A_CRT_KEY_W0) / 4;
        s->crt_key[i] = rp2350_atomic_apply(addr, s->crt_key[i], value);
        break;
    case A_DEBUGEN:
        v = rp2350_atomic_apply(addr, s->debugen, value) & DEBUGEN_MASK;
        s->debugen = (s->debugen & s->debugen_lock) |
                     (v & ~s->debugen_lock);
        otp_update_debug(s);
        break;
    case A_DEBUGEN_LOCK:
        s->debugen_lock |= ones & DEBUGEN_MASK;
        break;
    case A_ARCHSEL:
        v = rp2350_atomic_apply(addr, s->archsel, value) & ARCHSEL_MASK;
        if (s->critical & (CRITICAL_RISCV_DISABLE |
                           CRITICAL_SECURE_BOOT_ENABLE)) {
            /* The critical flags force Arm. */
            v = 0;
        }
        if (v & ~s->archsel) {
            qemu_log_mask(LOG_UNIMP, "rp2350-otp: ARCHSEL selects RISC-V, "
                          "but the machine has no Hazard3 cores; cores "
                          "stay Arm across their next reset\n");
        }
        s->archsel = v;
        break;
    case A_BOOTDIS:
        s->bootdis |= ones & BOOTDIS_NEXT;
        s->bootdis &= ~(ones & BOOTDIS_NOW);
        break;
    case A_INTR:
        s->intr &= ~(ones & INTR_WC);
        otp_update_irq(s);
        break;
    case A_INTE:
        s->inte = rp2350_atomic_apply(addr, s->inte, value) & INT_MASK;
        otp_update_irq(s);
        break;
    case A_INTF:
        s->intf = rp2350_atomic_apply(addr, s->intf, value) & INT_MASK;
        otp_update_irq(s);
        break;
    case A_SBPI_RDATA_0 ... A_SBPI_RDATA_3:
    case A_CRITICAL:
    case A_KEY_VALID:
    case A_ARCHSEL_STATUS:
    case A_INTS:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: write to read-only "
                      "register 0x%03" HWADDR_PRIx "\n", reg);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-otp: write to reserved "
                      "offset 0x%03" HWADDR_PRIx "\n", reg);
        break;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_otp_ctrl_ops = {
    .read_with_attrs = rp2350_otp_ctrl_read,
    .write_with_attrs = rp2350_otp_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * A reset of the OTP block by its PSM stage, which every sequence that
 * powers the core down and up again runs: the registers take their reset
 * values and the power-up state machine runs again, re-latching the
 * critical flags and keys and reopening the soft locks. BOOTDIS is the
 * exception: NEXT is ORed into NOW and cleared, so that the boot ROM can
 * tell the OTP was reset and ignore the boot vectors, which would
 * otherwise let a later boot stage skip the stage that soft-locked pages.
 */
/* [spec:nuos:req:emu.otp] */
void rp2350_otp_reset_stage(RP2350OTPState *s)
{
    if (s->bootdis & BOOTDIS_NEXT) {
        s->bootdis = BOOTDIS_NOW;
    }
    s->sbpi_instr = 0;
    memset(s->sbpi_wdata, 0, sizeof(s->sbpi_wdata));
    memset(s->sbpi_rdata, 0, sizeof(s->sbpi_rdata));
    s->sbpi_status = 0;
    s->usr = USR_DCTRL;
    s->bist = BIST_RESET;
    memset(s->crt_key, 0, sizeof(s->crt_key));
    s->debugen = 0;
    s->debugen_lock = 0;
    s->intr = 0;
    s->inte = 0;
    s->intf = 0;
    memset(s->dap, 0, sizeof(s->dap));
    memset(s->pmc, 0, sizeof(s->pmc));

    otp_psm(s);
    s->dbg = DBG_ROSC_UP_SEEN | DBG_BOOT_DONE | DBG_PSM_DONE |
             (s->rma ? DBG_CUSTOMER_RMA_FLAG : 0);
    otp_update_irq(s);
    otp_update_debug(s);
}

/*
 * A system reset of the cold type is a power-on, brownout or RUN pin
 * reset, which clears BOOTDIS. Every other system reset resets the
 * switched core, and with it the OTP stage.
 */
/* [spec:nuos:req:emu.otp] */
static void rp2350_otp_reset_hold(Object *obj, ResetType type)
{
    RP2350OTPState *s = RP2350_OTP(obj);

    if (type != RESET_TYPE_WAKEUP) {
        s->bootdis = 0;
    }
    rp2350_otp_reset_stage(s);
}

/*
 * Load the fuse array from the backing drive, or give the chip its
 * factory contents. An all-zero image becomes a factory-fresh chip.
 */
/* [spec:nuos:req:emu.otp] */
static bool otp_load(RP2350OTPState *s, Error **errp)
{
    int64_t len;
    bool blank = true;
    int i;

    if (!s->blk) {
        otp_factory_program(s);
        return true;
    }
    len = blk_getlength(s->blk);
    if (len < 0) {
        error_setg_errno(errp, -len, "rp2350-otp: cannot size the drive");
        return false;
    }
    if (len < sizeof(s->fuse)) {
        error_setg(errp, "rp2350-otp: the drive must hold at least %zu bytes "
                   "(4096 rows of 4 bytes)", sizeof(s->fuse));
        return false;
    }
    s->blk_writable = blk_supports_write_perm(s->blk) &&
        blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                     BLK_PERM_ALL, NULL) == 0;
    if (!s->blk_writable) {
        warn_report("%s: read-only, so programmed OTP rows will not persist",
                    blk_name(s->blk));
    }
    if (blk_pread(s->blk, 0, sizeof(s->fuse), s->fuse, 0) < 0) {
        error_setg(errp, "rp2350-otp: failed to read the drive");
        return false;
    }
    for (i = 0; i < RP2350_OTP_ROWS; i++) {
        s->fuse[i] = le32_to_cpu(s->fuse[i]) & ROW_MASK;
        blank &= !s->fuse[i];
    }
    if (blank) {
        otp_factory_program(s);
        for (i = 0; i < RP2350_OTP_ROWS; i++) {
            if (s->fuse[i]) {
                otp_persist_row(s, i);
            }
        }
    }
    return true;
}

static void rp2350_otp_realize(DeviceState *dev, Error **errp)
{
    otp_load(RP2350_OTP(dev), errp);
}

static void rp2350_otp_init(Object *obj)
{
    RP2350OTPState *s = RP2350_OTP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ctrl_iomem, obj, &rp2350_otp_ctrl_ops, s,
                          TYPE_RP2350_OTP, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);
    memory_region_init_io(&s->data_iomem, obj, &rp2350_otp_data_ops, s,
                          TYPE_RP2350_OTP "-data", RP2350_OTP_DATA_SIZE);
    sysbus_init_mmio(sbd, &s->data_iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->debug_disable,
                             "debug-disable", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->secure_debug_disable,
                             "secure-debug-disable", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->glitch_detector_enable,
                             "glitch-detector-enable", 1);
}

static const VMStateDescription vmstate_rp2350_otp = {
    .name = TYPE_RP2350_OTP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(fuse, RP2350OTPState, RP2350_OTP_ROWS),
        VMSTATE_UINT32(critical, RP2350OTPState),
        VMSTATE_UINT32(key_valid, RP2350OTPState),
        VMSTATE_UINT32_2DARRAY(key, RP2350OTPState, RP2350_OTP_KEYS, 4),
        VMSTATE_UINT32_ARRAY(key_cfg, RP2350OTPState, RP2350_OTP_PAGES),
        VMSTATE_BOOL(rma, RP2350OTPState),
        VMSTATE_UINT32_ARRAY(sw_lock, RP2350OTPState, RP2350_OTP_PAGES),
        VMSTATE_UINT32(sbpi_instr, RP2350OTPState),
        VMSTATE_UINT32_ARRAY(sbpi_wdata, RP2350OTPState, 4),
        VMSTATE_UINT32_ARRAY(sbpi_rdata, RP2350OTPState, 4),
        VMSTATE_UINT32(sbpi_status, RP2350OTPState),
        VMSTATE_UINT32(usr, RP2350OTPState),
        VMSTATE_UINT32(dbg, RP2350OTPState),
        VMSTATE_UINT32(bist, RP2350OTPState),
        VMSTATE_UINT32_ARRAY(crt_key, RP2350OTPState, 4),
        VMSTATE_UINT32(debugen, RP2350OTPState),
        VMSTATE_UINT32(debugen_lock, RP2350OTPState),
        VMSTATE_UINT32(archsel, RP2350OTPState),
        VMSTATE_UINT32(archsel_status, RP2350OTPState),
        VMSTATE_UINT32(bootdis, RP2350OTPState),
        VMSTATE_UINT32(intr, RP2350OTPState),
        VMSTATE_UINT32(inte, RP2350OTPState),
        VMSTATE_UINT32(intf, RP2350OTPState),
        VMSTATE_UINT8_ARRAY(dap, RP2350OTPState, RP2350_OTP_SBPI_REGS),
        VMSTATE_UINT8_ARRAY(pmc, RP2350OTPState, RP2350_OTP_SBPI_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_otp_properties[] = {
    DEFINE_PROP_DRIVE("drive", RP2350OTPState, blk),
};

static void rp2350_otp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_otp_realize;
    dc->vmsd = &vmstate_rp2350_otp;
    device_class_set_props(dc, rp2350_otp_properties);
    rc->phases.hold = rp2350_otp_reset_hold;
}

/* [spec:nuos:req:emu.otp] */
static const TypeInfo rp2350_otp_info = {
    .name          = TYPE_RP2350_OTP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350OTPState),
    .instance_init = rp2350_otp_init,
    .class_init    = rp2350_otp_class_init,
};

static void rp2350_otp_register_types(void)
{
    type_register_static(&rp2350_otp_info);
}
type_init(rp2350_otp_register_types)
