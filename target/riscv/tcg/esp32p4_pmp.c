/*
 * Espressif ESP32-P4 HP core physical memory protection and attributes
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.10
 * (Memory Protection Unit): 2.10.1, the standard PMP, and 2.10.2, the PMA
 * checker; and 2.6.6.5 (AMO instructions).
 *
 * Every access an HP core makes, instruction fetches included, passes two
 * checks, and an access either refuses raises an access fault:
 *
 * - The PMP: 32 entries (pmpcfg0-7, pmpaddr0-31) at 128-byte granularity,
 *   lowest-numbered entry first. An entry restricts user mode, and machine
 *   mode too once its lock bit L is set; an address no entry matches is
 *   open to machine mode and closed to user mode. L also stops the entry's
 *   registers being written, and, on a TOR entry, the address register
 *   below it; the custom lock XL (bit 5) stops the entry's own registers
 *   being written without applying the entry to machine mode. Only a reset
 *   of the core clears either. The core has no Smepmp: there is no
 *   mseccfg.
 *
 * - The PMA checker: 16 entries (pma_cfg0-15 at 0xBC0, pma_addr0-15 at
 *   0xBD0), matched like PMP entries and lowest-numbered first, but in
 *   every privilege mode: an entry of the invalid type allows nothing, one
 *   of the valid type the accesses its R, W and X bits allow, and an
 *   address no entry matches is open. Its lock only stops the entry's
 *   registers being written. Its four attribute bits go to the cache
 *   controller and do nothing here.
 *
 * Both match OFF, TOR and NAPOT. The TRM lists the NA4 encoding as
 * unsupported without saying what it does; it is kept as written (the PMA
 * entries reset to it) and matches no address. Address bits below the
 * 128-byte grain take no part: TOR ignores the address register's bits
 * 4:0, which read as zero, and NAPOT takes its bits 3:0 as ones, which
 * they read as, so that the smallest region is 128 bytes.
 *
 * An access that crosses a 128-byte boundary, which only a misaligned one
 * can, is checked as the parts on each side, each on its own, as when it
 * crosses a page: it faults with the first refused part's address in
 * mtval, and neither part is made. An AMO that either check refuses, to
 * read or to write, raises a store/AMO access fault, as QEMU's RISC-V AMOs
 * report a refused read as their store's. cm.push, cm.pop and
 * cm.popret check all their accesses before making any (the CPU's
 * zcmp_check_first), so that a refused one leaves memory and registers as
 * they were.
 *
 * The PMP is this model's own rather than QEMU's generic one, which the
 * CPU's configuration leaves off: that one has neither XL nor the PMA
 * checker, and refuses an access that lies partly inside an entry rather
 * than checking it part by part.
 *
 * QEMU's TLB caches a page's permissions only when one entry of each unit,
 * or none, decides the whole page; any other page is checked on every
 * access, and its code runs one instruction per translation.
 *
 * An access the checks allow goes to the bus with the privilege mode it
 * is made in, MemTxAttrs.user set for user mode, as the bus carries it to
 * the blocks that check it (the interrupt matrix's secure fields).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "accel/tcg/cpu-ops.h"
#include "accel/tcg/cpu-loop.h"
#include "internals.h"
#include "target/riscv/esp32p4_cpu.h"

/* The grain both units match at: 128 bytes, the RISC-V G of 5. */
#define ESP32P4_GRAIN      128
#define ESP32P4_G          5

/* Address matching modes of both units' A fields. */
#define ESP32P4_A_OFF      0
#define ESP32P4_A_TOR      1
#define ESP32P4_A_NAPOT    3

/* Accesses: 1 << MMUAccessType. */
#define ESP32P4_R          BIT(MMU_DATA_LOAD)
#define ESP32P4_W          BIT(MMU_DATA_STORE)
#define ESP32P4_X          BIT(MMU_INST_FETCH)
#define ESP32P4_RWX        (ESP32P4_R | ESP32P4_W | ESP32P4_X)

/* pmpXcfg: R, W and X in bits 0 to 2, as ESP32P4_R, W and X. */
#define ESP32P4_PMP_A_SHIFT 3
#define ESP32P4_PMP_XL     BIT(5)
#define ESP32P4_PMP_L      BIT(7)
#define ESP32P4_PMP_CFG_MASK (ESP32P4_RWX | (3 << ESP32P4_PMP_A_SHIFT) | \
                              ESP32P4_PMP_XL | ESP32P4_PMP_L)

/* pma_cfgn */
#define ESP32P4_PMA_TYPE   BIT(0)
#define ESP32P4_PMA_X      BIT(2)
#define ESP32P4_PMA_W      BIT(3)
#define ESP32P4_PMA_R      BIT(4)
#define ESP32P4_PMA_ATTR   (0xfu << 24)
#define ESP32P4_PMA_LOCK   BIT(29)
#define ESP32P4_PMA_A_SHIFT 30
#define ESP32P4_PMA_CFG_MASK (ESP32P4_PMA_TYPE | ESP32P4_PMA_X |          \
                              ESP32P4_PMA_W | ESP32P4_PMA_R |             \
                              ESP32P4_PMA_ATTR | ESP32P4_PMA_LOCK |       \
                              (3u << ESP32P4_PMA_A_SHIFT))
/* NA4 in A, and nothing else: the reset value of every entry. */
#define ESP32P4_PMA_CFG_RESET (2u << ESP32P4_PMA_A_SHIFT)

#define CSR_ESP_PMACFG0    0xbc0
#define CSR_ESP_PMAADDR0   0xbd0

static int esp32p4_pmp_a(uint8_t cfg)
{
    return extract32(cfg, ESP32P4_PMP_A_SHIFT, 2);
}

static int esp32p4_pma_a(uint32_t cfg)
{
    return cfg >> ESP32P4_PMA_A_SHIFT;
}

/*
 * The bytes an entry matches, [*lo, *hi), from its mode, its address
 * register and the address register below it (zero for entry 0); false
 * if it matches none.
 */
static bool esp32p4_region(int a, uint32_t addr, uint32_t prev,
                           uint64_t *lo, uint64_t *hi)
{
    uint64_t napot;

    switch (a) {
    case ESP32P4_A_TOR:
        *lo = (uint64_t)(prev & ~MAKE_64BIT_MASK(0, ESP32P4_G)) << 2;
        *hi = (uint64_t)(addr & ~MAKE_64BIT_MASK(0, ESP32P4_G)) << 2;
        return *lo < *hi;
    case ESP32P4_A_NAPOT:
        napot = ((uint64_t)(addr | MAKE_64BIT_MASK(0, ESP32P4_G - 1)) << 2) |
                3;
        *lo = napot & (napot + 1);
        *hi = (napot | (napot + 1)) + 1;
        return true;
    default:
        return false;
    }
}

/*
 * An address register as read: in OFF and TOR modes its bits below the
 * grain read as zero, in NAPOT mode as ones.
 */
static uint32_t esp32p4_addr_read(int a, uint32_t addr)
{
    switch (a) {
    case ESP32P4_A_OFF:
    case ESP32P4_A_TOR:
        return addr & ~MAKE_64BIT_MASK(0, ESP32P4_G);
    case ESP32P4_A_NAPOT:
        return addr | MAKE_64BIT_MASK(0, ESP32P4_G - 1);
    default:
        return addr;
    }
}

static bool esp32p4_pmp_region(ESP32P4CPU *cpu, int i,
                               uint64_t *lo, uint64_t *hi)
{
    return esp32p4_region(esp32p4_pmp_a(cpu->pmp_cfg[i]), cpu->pmp_addr[i],
                          i ? cpu->pmp_addr[i - 1] : 0, lo, hi);
}

static bool esp32p4_pma_region(ESP32P4CPU *cpu, int i,
                               uint64_t *lo, uint64_t *hi)
{
    return esp32p4_region(esp32p4_pma_a(cpu->pma_cfg[i]), cpu->pma_addr[i],
                          i ? cpu->pma_addr[i - 1] : 0, lo, hi);
}

/* The accesses the PMP allows to the grain at addr in privilege mode priv. */
/* [spec:nuos:req:emu.esp32p4.pmp] */
static int esp32p4_pmp_privs(ESP32P4CPU *cpu, uint64_t addr, int priv)
{
    for (int i = 0; i < ESP32P4_PMP_ENTRIES; i++) {
        uint64_t lo, hi;

        if (esp32p4_pmp_region(cpu, i, &lo, &hi) &&
            addr >= lo && addr < hi) {
            if (priv == PRV_M && !(cpu->pmp_cfg[i] & ESP32P4_PMP_L)) {
                return ESP32P4_RWX;
            }
            return cpu->pmp_cfg[i] & ESP32P4_RWX;
        }
    }
    return priv == PRV_M ? ESP32P4_RWX : 0;
}

/* And those the PMA checker allows, in any privilege mode. */
/* [spec:nuos:req:emu.esp32p4.pmp] */
static int esp32p4_pma_privs(ESP32P4CPU *cpu, uint64_t addr)
{
    for (int i = 0; i < ESP32P4_PMA_ENTRIES; i++) {
        uint32_t cfg = cpu->pma_cfg[i];
        uint64_t lo, hi;

        if (esp32p4_pma_region(cpu, i, &lo, &hi) &&
            addr >= lo && addr < hi) {
            if (!(cfg & ESP32P4_PMA_TYPE)) {
                return 0;
            }
            return (cfg & ESP32P4_PMA_R ? ESP32P4_R : 0) |
                   (cfg & ESP32P4_PMA_W ? ESP32P4_W : 0) |
                   (cfg & ESP32P4_PMA_X ? ESP32P4_X : 0);
        }
    }
    return ESP32P4_RWX;
}

/*
 * Whether a region overlaps [lo, hi), and if so whether it covers all of
 * it.
 */
static bool esp32p4_overlaps(uint64_t lo, uint64_t hi,
                             uint64_t rlo, uint64_t rhi, bool *covers)
{
    if (rhi <= lo || rlo >= hi) {
        return false;
    }
    *covers = rlo <= lo && rhi >= hi;
    return true;
}

/*
 * Whether one entry of each unit, or none, decides every byte of [lo, hi):
 * the first entry to match any byte of it matches all of them.
 */
static bool esp32p4_uniform(ESP32P4CPU *cpu, uint64_t lo, uint64_t hi,
                            int priv)
{
    bool locked = false;
    bool covers;
    uint64_t rlo, rhi;
    int i;

    for (i = 0; i < ESP32P4_PMP_ENTRIES; i++) {
        locked |= cpu->pmp_cfg[i] & ESP32P4_PMP_L;
    }
    /* With no entry locked, machine mode may make any access. */
    if (priv != PRV_M || locked) {
        for (i = 0; i < ESP32P4_PMP_ENTRIES; i++) {
            if (esp32p4_pmp_region(cpu, i, &rlo, &rhi) &&
                esp32p4_overlaps(lo, hi, rlo, rhi, &covers)) {
                if (!covers) {
                    return false;
                }
                break;
            }
        }
    }
    for (i = 0; i < ESP32P4_PMA_ENTRIES; i++) {
        if (esp32p4_pma_region(cpu, i, &rlo, &rhi) &&
            esp32p4_overlaps(lo, hi, rlo, rhi, &covers)) {
            return covers;
        }
    }
    return true;
}

static int esp32p4_page_prot(int privs)
{
    return (privs & ESP32P4_R ? PAGE_READ : 0) |
           (privs & ESP32P4_W ? PAGE_WRITE : 0) |
           (privs & ESP32P4_X ? PAGE_EXEC : 0);
}

/*
 * The HP core's TLB fill: the core has no address translation, so the
 * checks above alone decide an access, which carries its privilege mode.
 */
/* [spec:nuos:req:emu.esp32p4.pmp] */
bool esp32p4_hp_tlb_fill(CPUState *cs, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx,
                         bool probe, uintptr_t retaddr)
{
    ESP32P4CPU *cpu = container_of(cs, ESP32P4CPU, parent_obj.parent_obj);
    CPURISCVState *env = &cpu->parent_obj.env;
    int priv = mmuidx_priv(mmu_idx);
    MemTxAttrs attrs = { .user = priv == PRV_U };
    int privs = ESP32P4_RWX;
    uint64_t end = (uint64_t)address + MAX(size, 1);
    uint64_t page = address & TARGET_PAGE_MASK;
    uint64_t part;

    for (part = address; part < end;
         part = (part | (ESP32P4_GRAIN - 1)) + 1) {
        int allowed = esp32p4_pmp_privs(cpu, part, priv) &
                      esp32p4_pma_privs(cpu, part);

        if (!(allowed & BIT(access_type))) {
            break;
        }
        privs &= allowed;
    }

    if (part >= end) {
        if (esp32p4_uniform(cpu, page, page + TARGET_PAGE_SIZE, priv)) {
            tlb_set_page_with_attrs(cs, page, page, attrs,
                                    esp32p4_page_prot(privs), mmu_idx,
                                    TARGET_PAGE_SIZE);
        } else {
            tlb_set_page_with_attrs(cs, address, address, attrs,
                                    esp32p4_page_prot(privs), mmu_idx, 1);
        }
        return true;
    }
    if (probe) {
        return false;
    }

    if (access_type != MMU_INST_FETCH) {
        cpu_check_watchpoint(cs, address, size, MEMTXATTRS_UNSPECIFIED,
                             access_type == MMU_DATA_LOAD ? BP_MEM_READ
                                                          : BP_MEM_WRITE,
                             retaddr);
    }
    switch (access_type) {
    case MMU_INST_FETCH:
        cs->exception_index = RISCV_EXCP_INST_ACCESS_FAULT;
        break;
    case MMU_DATA_LOAD:
        cs->exception_index = RISCV_EXCP_LOAD_ACCESS_FAULT;
        break;
    default:
        cs->exception_index = RISCV_EXCP_STORE_AMO_ACCESS_FAULT;
        break;
    }
    env->badaddr = part;
    env->two_stage_lookup = false;
    env->two_stage_indirect_lookup = false;
    cpu_loop_exit_restore(cs, retaddr);
}

/* PMP registers */

static bool esp32p4_pmp_locked(ESP32P4CPU *cpu, int n)
{
    return cpu->pmp_cfg[n] & (ESP32P4_PMP_L | ESP32P4_PMP_XL);
}

target_ulong esp32p4_pmpcfg_read(ESP32P4CPU *cpu, int reg)
{
    return ldl_le_p(&cpu->pmp_cfg[reg * 4]);
}

/*
 * Bit 6 of each entry's byte is reserved and reads as zero; an entry with
 * L or XL set keeps its byte.
 */
/* [spec:nuos:req:emu.esp32p4.pmp] */
void esp32p4_pmpcfg_write(ESP32P4CPU *cpu, int reg, target_ulong val)
{
    bool changed = false;

    for (int i = 0; i < 4; i++) {
        int n = reg * 4 + i;
        uint8_t cfg = extract32(val, 8 * i, 8) & ESP32P4_PMP_CFG_MASK;

        if (cfg == cpu->pmp_cfg[n]) {
            continue;
        }
        if (esp32p4_pmp_locked(cpu, n)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4: pmp%dcfg is locked\n",
                          n);
            continue;
        }
        cpu->pmp_cfg[n] = cfg;
        changed = true;
    }
    if (changed) {
        tlb_flush(CPU(cpu));
    }
}

target_ulong esp32p4_pmpaddr_read(ESP32P4CPU *cpu, int n)
{
    return esp32p4_addr_read(esp32p4_pmp_a(cpu->pmp_cfg[n]),
                             cpu->pmp_addr[n]);
}

/*
 * A locked entry keeps its address, as does the entry below an entry
 * that L locks in TOR mode, whose range starts there.
 */
/* [spec:nuos:req:emu.esp32p4.pmp] */
void esp32p4_pmpaddr_write(ESP32P4CPU *cpu, int n, target_ulong val)
{
    if ((uint32_t)val == cpu->pmp_addr[n]) {
        return;
    }
    if (esp32p4_pmp_locked(cpu, n)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4: pmpaddr%d is locked\n", n);
        return;
    }
    if (n + 1 < ESP32P4_PMP_ENTRIES &&
        (cpu->pmp_cfg[n + 1] & ESP32P4_PMP_L) &&
        esp32p4_pmp_a(cpu->pmp_cfg[n + 1]) == ESP32P4_A_TOR) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4: pmpaddr%d is locked by "
                      "the TOR entry above it\n", n);
        return;
    }
    cpu->pmp_addr[n] = val;
    tlb_flush(CPU(cpu));
}

/* PMA registers */

target_ulong esp32p4_pma_read(ESP32P4CPU *cpu, int csrno)
{
    int n;

    if (csrno < CSR_ESP_PMAADDR0) {
        return cpu->pma_cfg[csrno - CSR_ESP_PMACFG0];
    }
    n = csrno - CSR_ESP_PMAADDR0;
    return esp32p4_addr_read(esp32p4_pma_a(cpu->pma_cfg[n]),
                             cpu->pma_addr[n]);
}

/*
 * The reserved bits of pma_cfgn read as zero. LOCK keeps both registers
 * of its entry.
 */
/* [spec:nuos:req:emu.esp32p4.pmp] */
void esp32p4_pma_write(ESP32P4CPU *cpu, int csrno, target_ulong val)
{
    bool cfg = csrno < CSR_ESP_PMAADDR0;
    int n = cfg ? csrno - CSR_ESP_PMACFG0 : csrno - CSR_ESP_PMAADDR0;
    uint32_t *reg = cfg ? &cpu->pma_cfg[n] : &cpu->pma_addr[n];
    uint32_t v = cfg ? (val & ESP32P4_PMA_CFG_MASK) : val;

    if (v == *reg) {
        return;
    }
    if (cpu->pma_cfg[n] & ESP32P4_PMA_LOCK) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4: pma%s%d is locked\n",
                      cfg ? "_cfg" : "_addr", n);
        return;
    }
    *reg = v;
    tlb_flush(CPU(cpu));
}

/*
 * A reset of the core clears every PMP entry, locks included, and puts
 * every PMA entry back to NA4 with the invalid type.
 */
/* [spec:nuos:req:emu.esp32p4.pmp] */
void esp32p4_pmp_reset(ESP32P4CPU *cpu)
{
    memset(cpu->pmp_cfg, 0, sizeof(cpu->pmp_cfg));
    memset(cpu->pmp_addr, 0, sizeof(cpu->pmp_addr));
    for (int i = 0; i < ESP32P4_PMA_ENTRIES; i++) {
        cpu->pma_cfg[i] = ESP32P4_PMA_CFG_RESET;
        cpu->pma_addr[i] = 0;
    }
}
