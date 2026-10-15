/*
 * Espressif ESP32-P4 HP core: interrupts from the CLIC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.9.2
 * (CLIC) and the CLIC CSRs of 2.5; Core-Local Interrupt Controller (CLIC)
 * RISC-V Privileged Architecture Extensions, 10/10/2023, which the core
 * implements with its smclicshv, smclicconfig and suclic extensions.
 *
 * The HP core runs in CLIC mode only: mtvec and utvec read MODE 3, and
 * mie, mip and mideleg read zero. Its CLIC (hw/intc/esp32p4_clic.c)
 * presents the highest-ranked of its pending and enabled interrupts, by
 * privilege mode, then level and priority (clicintctl), then ID, with the
 * interrupt's level and whether it is hardware vectored. The core takes
 * it before its next instruction:
 *
 *   - a machine-mode interrupt in user mode, whatever the user-mode
 *     state, as a higher privilege mode's interrupts always preempt a
 *     lower one's; so user mode can neither mask nor take one;
 *   - an interrupt of the current mode with that mode's interrupt enable
 *     (mstatus.MIE, ustatus.UIE) set and a level above both the current
 *     level (mintstatus.MIL, uintstatus.UIL) and the threshold
 *     (mintthresh, uintthresh);
 *   - a user-mode interrupt in machine mode never.
 *
 * A machine-mode interrupt saves MIE in MPIE, the privilege in MPP, MIL
 * in mcause.MPIL and the PC in mepc, and enters machine mode at the
 * interrupt's level, with mcause holding the interrupt bit and ID; a
 * user-mode interrupt likewise saves UIE in UPIE, UIL in ucause.UPIL and
 * the PC in uepc, staying in user mode. A non-vectored interrupt goes to
 * the trap vector base (xtvec bits 31:6); a vectored one (clicintattr.SHV)
 * clears its pending bit if edge-triggered and goes to the address in the
 * vector table entry at xtvt + 4 * ID, fetched with the handler mode's
 * execute permission. A failing table fetch raises an instruction access
 * fault with the entry's address in mepc and mcause.MINHV set; MRET and
 * URET then fetch the entry again rather than return to xepc, while
 * mexstatus.CLIC_INHV is set.
 *
 * Exceptions, which the core always takes in machine mode, go to mtvec's
 * base, save MIL in mcause.MPIL and leave the level alone in machine mode;
 * from user mode they are taken at level 0. MRET restores MIL from MPIL
 * and, returning to user mode, sets mintthresh to its lowest value; URET
 * restores UIE from UPIE and UIL from UPIL, and returns to user mode.
 *
 * A core in WFI wakes for an interrupt of a higher or lower privilege mode
 * than its own, or of its own mode above its current level and threshold,
 * whatever the interrupt enables say.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/plugin.h"
#include "cpu.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/cpu-mmu-index.h"
#include "accel/tcg/probe.h"
#include "exec/helper-proto.h"
#include "exec/target_page.h"
#include "exec/tlb-flags.h"
#include "system/memory.h"
#include "target/riscv/esp32p4_cpu.h"

#define XTVEC_BASE_MASK  (~(target_ulong)0x3f)
#define CAUSE_INT        BIT(31)

/* The lowest thresholds, which mask no interrupt. */
#define ESP32P4_CLIC_THRESH_MIN 0

static int sel_priv(uint32_t sel)
{
    return extract32(sel, ESP32P4_CLIC_SEL_PRIV_SHIFT, 2);
}

static int sel_level(uint32_t sel)
{
    return extract32(sel, ESP32P4_CLIC_SEL_LEVEL_SHIFT, 8);
}

static int sel_id(uint32_t sel)
{
    return sel & ESP32P4_CLIC_SEL_ID_MASK;
}

/* [spec:nuos:req:emu.esp32p4.clic] */
void esp32p4_clic_cpu_reset(ESP32P4CPU *cpu)
{
    CPURISCVState *env = &cpu->parent_obj.env;

    env->mtvec = 3;
    env->mcause = 0;
    cpu->mtvt = 0;
    cpu->utvec = 3;
    cpu->utvt = 0;
    cpu->uepc = 0;
    cpu->uscratch = 0;
    cpu->ucause = 0;
    cpu->mil = 0;
    cpu->uil = 0;
    qatomic_set(&cpu->mintthresh, ESP32P4_CLIC_THRESH_MIN);
    cpu->uintthresh = ESP32P4_CLIC_THRESH_MIN;
    cpu->mpil = 0;
    cpu->upil = 0;
    cpu->minhv = false;
    cpu->uinhv = false;
    cpu->clic_inhv_fault = false;
}

/*
 * Whether the presented interrupt @sel preempts the core: given whether
 * its mode's interrupt enable counts, as it does for taking the interrupt
 * but not for waking from WFI.
 */
static bool esp32p4_clic_preempts(ESP32P4CPU *cpu, uint32_t sel,
                                  bool enable)
{
    CPURISCVState *env = &cpu->parent_obj.env;
    int priv = sel_priv(sel);
    int level = sel_level(sel);
    bool ie;
    uint8_t il, th;

    if (!(sel & ESP32P4_CLIC_SEL_VALID)) {
        return false;
    }
    if (priv != env->priv) {
        /*
         * A higher mode's interrupt always preempts; a lower mode's is
         * never taken, but wakes the core from WFI.
         */
        return level != 0 && (priv > env->priv || !enable);
    }
    if (priv == PRV_M) {
        ie = env->mstatus & MSTATUS_MIE;
        il = cpu->mil;
        th = qatomic_read(&cpu->mintthresh);
    } else {
        ie = env->mstatus & MSTATUS_UIE;
        il = cpu->uil;
        th = cpu->uintthresh;
    }
    return (ie || !enable) && level > MAX(il, th);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
bool esp32p4_clic_takes(ESP32P4CPU *cpu, uint32_t sel)
{
    return esp32p4_clic_preempts(cpu, sel, true);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
bool esp32p4_clic_wakes(ESP32P4CPU *cpu)
{
    return esp32p4_clic_preempts(cpu, qatomic_read(&cpu->clic_sel), false);
}

static void esp32p4_clic_claim(ESP32P4CPU *cpu, int id)
{
    if (cpu->clic_ops) {
        cpu->clic_ops->claim(cpu->clic_opaque, id);
    }
}

/*
 * Fetch the vector table entry at @addr with the execute permission of
 * the current privilege mode, as an instruction fetch: at the address and
 * with the attributes the core's fetch translation gives it, so through
 * the core's instruction cache where it is cacheable. True with the
 * handler's address, or false with the core set up to raise the
 * instruction access fault the fetch takes.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static bool esp32p4_clic_fetch_vector(ESP32P4CPU *cpu, target_ulong addr,
                                      target_ulong *handler)
{
    CPURISCVState *env = &cpu->parent_obj.env;
    CPUState *cs = CPU(cpu);
    int mmu_idx = cpu_mmu_index(cs, true);
    CPUTLBEntryFull *full;
    void *host;
    MemTxResult res;
    uint32_t entry;

    addr &= ~(target_ulong)3;
    if (probe_access_full(env, addr, 4, MMU_INST_FETCH, mmu_idx, true,
                          &host, &full, 0) & TLB_INVALID_MASK) {
        cpu->bus_error = false;
    } else {
        entry = address_space_ldl_le(cs->as,
                                     (full->phys_addr & TARGET_PAGE_MASK) |
                                     (addr & ~TARGET_PAGE_MASK),
                                     full->attrs, &res);
        if (res == MEMTX_OK) {
            *handler = entry & ~(target_ulong)1;
            return true;
        }
        cpu->bus_error = true;
    }
    cs->exception_index = RISCV_EXCP_INST_ACCESS_FAULT;
    env->badaddr = addr;
    env->pc = addr;
    cpu->clic_inhv_fault = true;
    return false;
}

/*
 * Take the interrupt the CLIC presents, which esp32p4_clic_takes() has
 * allowed; for a machine-mode one, the core's own trap entry effects
 * (mexstatus, tcontrol) have been applied.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
void esp32p4_clic_take(ESP32P4CPU *cpu, uint32_t sel)
{
    CPURISCVState *env = &cpu->parent_obj.env;
    CPUState *cs = CPU(cpu);
    int priv = sel_priv(sel);
    int id = sel_id(sel);
    int prev_priv = env->priv;
    uint64_t last_pc = env->pc;
    target_ulong handler;
    uint64_t s = env->mstatus;

    qemu_log_mask(CPU_LOG_INT, "%s: hart:%" PRIu64 ", CLIC interrupt %d, "
                  "level %d, %s mode, epc:0x%" PRIx64 "\n", __func__,
                  env->mhartid, id, sel_level(sel),
                  priv == PRV_M ? "machine" : "user", env->pc);

    if (priv == PRV_M) {
        s = set_field(s, MSTATUS_MPIE, get_field(s, MSTATUS_MIE));
        s = set_field(s, MSTATUS_MPP, env->priv);
        s = set_field(s, MSTATUS_MIE, 0);
        env->mstatus = s;
        env->mcause = CAUSE_INT | id;
        env->mepc = env->pc;
        env->mtval = 0;
        cpu->mpil = cpu->mil;
        cpu->minhv = false;
        cpu->mil = sel_level(sel);
        riscv_cpu_set_mode(env, PRV_M, false);
        esp32p4_hp_priv_update(cpu);
        handler = env->mtvec & XTVEC_BASE_MASK;
    } else {
        s = set_field(s, MSTATUS_UPIE, get_field(s, MSTATUS_UIE));
        s = set_field(s, MSTATUS_UIE, 0);
        env->mstatus = s;
        cpu->ucause = CAUSE_INT | id;
        cpu->uepc = env->pc;
        cpu->upil = cpu->uil;
        cpu->uinhv = false;
        cpu->uil = sel_level(sel);
        env->load_res = -1;
        handler = cpu->utvec & XTVEC_BASE_MASK;
    }
    qemu_plugin_vcpu_interrupt_cb(cs, last_pc);

    if (sel & ESP32P4_CLIC_SEL_SHV) {
        target_ulong tvt = priv == PRV_M ? cpu->mtvt : cpu->utvt;

        esp32p4_clic_claim(cpu, id);
        if (!esp32p4_clic_fetch_vector(cpu, (tvt & XTVEC_BASE_MASK) + 4 * id,
                                       &handler)) {
            esp32p4_trace_trap(cpu, last_pc, id, true, 0, prev_priv, true);
            esp32p4_hp_do_interrupt(cs);
            return;
        }
    }
    env->pc = handler;
    esp32p4_trace_trap(cpu, last_pc, id, true, 0, prev_priv, true);
}

/*
 * After an exception, which the standard trap entry has taken in machine
 * mode from @prev_priv at level @prev_mil: mcause.MPIL keeps that level,
 * which a trap from user mode leaves for level 0, and MINHV says whether
 * mepc is a vector table entry.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
void esp32p4_clic_exception_taken(ESP32P4CPU *cpu, int prev_priv,
                                  uint8_t prev_mil)
{
    cpu->mpil = prev_mil;
    if (prev_priv != PRV_M) {
        cpu->mil = 0;
    }
    cpu->minhv = cpu->clic_inhv_fault;
    cpu->clic_inhv_fault = false;
    cpu->parent_obj.env.mcause &= CAUSE_INT | 0x3f;
}

/*
 * Return to @retpc, or to the handler in the vector table entry it holds
 * when @inhv: the entry is fetched with the permission of the mode
 * returned to, and a failing fetch raises its instruction access fault
 * from there, with MINHV set again.
 */
static target_ulong esp32p4_clic_return(ESP32P4CPU *cpu, target_ulong retpc,
                                        bool inhv)
{
    target_ulong handler;

    if (!inhv || !(cpu->mexstatus & ESP32P4_MEXSTATUS_CLIC_INHV)) {
        return retpc;
    }
    if (esp32p4_clic_fetch_vector(cpu, retpc, &handler)) {
        return handler;
    }
    cpu_loop_exit(CPU(cpu));
}

/*
 * MRET, after its standard effects: mintstatus.MIL comes back from
 * mcause.MPIL, and a return to user mode leaves mintthresh at its lowest
 * value, so that the user-mode code runs without a machine-mode threshold.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
target_ulong esp32p4_clic_mret(ESP32P4CPU *cpu, target_ulong retpc)
{
    CPURISCVState *env = &cpu->parent_obj.env;

    cpu->mil = cpu->mpil;
    if (env->priv != PRV_M) {
        qatomic_set(&cpu->mintthresh, ESP32P4_CLIC_THRESH_MIN);
    }
    return esp32p4_clic_return(cpu, retpc, cpu->minhv);
}

/*
 * URET, legal in user and machine mode: UIE comes back from UPIE, which
 * becomes set, uintstatus.UIL from ucause.UPIL, and the core continues in
 * user mode at uepc, or at the handler in the vector table entry it holds
 * if ucause.UINHV is set.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
target_ulong helper_esp32p4_uret(CPURISCVState *env)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    target_ulong retpc = cpu->uepc & ~(target_ulong)1;
    uint64_t s = env->mstatus;

    s = set_field(s, MSTATUS_UIE, get_field(s, MSTATUS_UPIE));
    s = set_field(s, MSTATUS_UPIE, 1);
    env->mstatus = s;
    cpu->uil = cpu->upil;
    if (env->priv != PRV_U) {
        riscv_cpu_set_mode(env, PRV_U, false);
        esp32p4_hp_priv_update(cpu);
    }
    return esp32p4_clic_return(cpu, retpc, cpu->uinhv);
}

/*
 * mnxti and unxti (CSRRSI/CSRRCI): the access sets or clears bits of
 * mstatus or ustatus whatever the CLIC presents. If the CLIC presents a
 * non-vectored interrupt of mode @priv above both the previous level in
 * xcause.xPIL and the threshold, the read returns its vector table entry's
 * address, and an access that writes takes the interrupt horizontally:
 * xintstatus.xIL becomes its level, xcause its ID with the interrupt bit
 * set, and an edge-triggered one stops pending. Otherwise the read
 * returns 0.
 */
static target_ulong esp32p4_clic_nxti_entry(ESP32P4CPU *cpu, int priv,
                                            bool commit)
{
    CPURISCVState *env = &cpu->parent_obj.env;
    uint32_t sel = qatomic_read(&cpu->clic_sel);
    uint8_t pil, th;
    target_ulong tvt;

    if (priv == PRV_M) {
        pil = cpu->mpil;
        th = qatomic_read(&cpu->mintthresh);
        tvt = cpu->mtvt;
    } else {
        pil = cpu->upil;
        th = cpu->uintthresh;
        tvt = cpu->utvt;
    }
    if (!(sel & ESP32P4_CLIC_SEL_VALID) || (sel & ESP32P4_CLIC_SEL_SHV) ||
        sel_priv(sel) != priv || sel_level(sel) <= pil ||
        sel_level(sel) <= th) {
        return 0;
    }
    if (commit) {
        if (priv == PRV_M) {
            cpu->mil = sel_level(sel);
            env->mcause = CAUSE_INT | sel_id(sel);
        } else {
            cpu->uil = sel_level(sel);
            cpu->ucause = CAUSE_INT | sel_id(sel);
        }
        esp32p4_clic_claim(cpu, sel_id(sel));
    }
    return (tvt & XTVEC_BASE_MASK) + 4 * sel_id(sel);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
RISCVException esp32p4_clic_nxti(ESP32P4CPU *cpu, int priv,
                                 target_ulong *ret_value,
                                 target_ulong new_value,
                                 target_ulong write_mask)
{
    CPURISCVState *env = &cpu->parent_obj.env;
    uint64_t writable = priv == PRV_M ? MSTATUS_MIE | MSTATUS_UIE |
                                        MSTATUS_UPIE
                                      : MSTATUS_UIE | MSTATUS_UPIE;
    target_ulong entry;

    if (write_mask) {
        BQL_LOCK_GUARD();

        writable &= write_mask;
        env->mstatus = (env->mstatus & ~writable) | (new_value & writable);
        entry = esp32p4_clic_nxti_entry(cpu, priv, true);
    } else {
        entry = esp32p4_clic_nxti_entry(cpu, priv, false);
    }
    if (ret_value) {
        *ret_value = entry;
    }
    return RISCV_EXCP_NONE;
}
