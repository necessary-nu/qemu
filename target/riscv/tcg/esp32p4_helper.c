/*
 * Espressif ESP32-P4 HP core: hardware loop and trap return helpers, and
 * the core's side of the debug assistant and the trace encoder
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.7.1
 * (Hardware Loop), Register 2.67 (mexstatus), 2.11.3 (Hardware Trigger),
 * 2.11.4 (the trace interface), table 2.4-1 (the CPU address map), and
 * chapters 22 (Debug Assistant) and 3 (RISC-V Trace Encoder).
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/icount.h"
#include "exec/target_page.h"
#include "exec/translation-block.h"
#include "accel/tcg/cpu-loop.h"
#include "tcg/insn-start-words.h"
#include "target/riscv/esp32p4_cpu.h"

static void esp32p4_hwlp_dirty(ESP32P4CPU *cpu)
{
    cpu->hwlp_state = (cpu->hwlp_state & ~ESP32P4_HWLP_STATE) |
                      ESP32P4_EXT_DIRTY;
}

/*
 * lp.starti, lp.endi, lp.count, lp.counti, lp.setupi and lp.setup: set
 * the selected registers of one loop. Executing a hardware loop
 * instruction makes the extension's state dirty.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void helper_esp32p4_hwlp_set(CPURISCVState *env, uint32_t desc,
                             target_ulong start, target_ulong end,
                             target_ulong count)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    int loop = desc & 1;

    if (desc & ESP32P4_HWLP_SET_START) {
        cpu->hwlp_start[loop] = start;
    }
    if (desc & ESP32P4_HWLP_SET_END) {
        cpu->hwlp_end[loop] = end;
    }
    if (desc & ESP32P4_HWLP_SET_COUNT) {
        cpu->hwlp_count[loop] = count & ESP32P4_HWLP_COUNT_MASK;
    }
    esp32p4_hwlp_dirty(cpu);
}

/*
 * Going back to a loop's start leaves the TB after the loop's last
 * instruction, which need not be the TB's last, though icount charged
 * the whole TB on entry: give back the instructions after it. Restoring
 * the state to the loop's last instruction gives it back too, and takes
 * it out of the instructions the core has retired, though it completed:
 * charge and count it again, and keep it retiring in the trace.
 */
static void esp32p4_hwlp_charge(CPURISCVState *env, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    if (icount_enabled() && cpu_restore_state(cs, ra)) {
        cs->neg.icount_decr.u16.low--;
        esp32p4_cpu_from_env(env)->insns++;
        esp32p4_trace_unwound(esp32p4_cpu_from_env(env), false);
    }
}

/*
 * The instruction at pc completed and execution continues at next, seq
 * when it fell through. If it fell through and is the last instruction of
 * a counting loop, count one iteration, and continue at the loop's start
 * unless that was the last; loop 0 goes first, so with both ending here
 * loop 1 counts once loop 0 has run out. Completing an iteration makes
 * the extension's state dirty.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
target_ulong helper_esp32p4_hwlp_end(CPURISCVState *env, target_ulong pc,
                                     target_ulong seq, target_ulong next)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    if (next != seq || !esp32p4_hwlp_enabled(cpu)) {
        return next;
    }
    for (int i = 0; i < ESP32P4_HWLP_LOOPS; i++) {
        if (cpu->hwlp_end[i] != (uint32_t)pc || !cpu->hwlp_count[i]) {
            continue;
        }
        cpu->hwlp_count[i]--;
        esp32p4_hwlp_dirty(cpu);
        if (cpu->hwlp_count[i]) {
            esp32p4_hwlp_charge(env, GETPC());
            return (target_ulong)(int32_t)cpu->hwlp_start[i];
        }
    }
    return next;
}

/*
 * A pending bus error (mexstatus.BUS_ERR) raises its access fault again
 * once mexstatus.PBEXE enables it, unless mexstatus.NMFT is set, before
 * the next instruction. The request is taken in the CPU's interrupt
 * hook.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_bus_error_check_pending(ESP32P4CPU *cpu)
{
    CPUState *cs = CPU(cpu);
    uint32_t pending = ESP32P4_MEXSTATUS_BUS_ERR | ESP32P4_MEXSTATUS_PBEXE;

    if ((cpu->mexstatus & (pending | ESP32P4_MEXSTATUS_NMFT)) == pending) {
        BQL_LOCK_GUARD();
        cpu_interrupt(cs, ESP32P4_CPU_INTERRUPT_BUS_ERROR);
    } else {
        cpu_reset_interrupt(cs, ESP32P4_CPU_INTERRUPT_BUS_ERROR);
    }
}

/*
 * After MRET, given mstatus from before it and the address it returns to:
 * the core keeps mstatus.MPRV, as privileged architecture 1.10 has it; the
 * trigger enable tcontrol.MTE comes back from MPTE; mexstatus.PBEXE comes
 * back from PPBEXE, which becomes set; the PMP and fetch error flags
 * clear, and so does the bus error flag when mexstatus.NMFT is set. Then
 * the CLIC's state returns (esp32p4_clic_mret()), which gives the address
 * the core continues at.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.clic] */
target_ulong helper_esp32p4_mret(CPURISCVState *env, uint64_t old_mstatus,
                                 target_ulong retpc)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    env->mstatus = (env->mstatus & ~MSTATUS_MPRV) |
                   (old_mstatus & MSTATUS_MPRV);

    cpu->tcontrol &= ~ESP32P4_TCONTROL_MTE;
    if (cpu->tcontrol & ESP32P4_TCONTROL_MPTE) {
        cpu->tcontrol |= ESP32P4_TCONTROL_MTE;
    }

    cpu->mexstatus &= ~(ESP32P4_MEXSTATUS_PBEXE | ESP32P4_MEXSTATUS_PMP_ERR |
                        ESP32P4_MEXSTATUS_FETCH_ERR);
    if (cpu->mexstatus & ESP32P4_MEXSTATUS_PPBEXE) {
        cpu->mexstatus |= ESP32P4_MEXSTATUS_PBEXE;
    }
    cpu->mexstatus |= ESP32P4_MEXSTATUS_PPBEXE;
    if (cpu->mexstatus & ESP32P4_MEXSTATUS_NMFT) {
        cpu->mexstatus &= ~ESP32P4_MEXSTATUS_BUS_ERR;
    }
    esp32p4_bus_error_check_pending(cpu);
    esp32p4_hp_priv_update(cpu);
    return esp32p4_clic_mret(cpu, retpc);
}

/*
 * The bus an HP core access goes out on (TRM table 2.4-1): the IBUS for
 * instruction fetches and the DBUS for data in the instruction and data
 * regions (0x30000000 to 0x3FEFFFFF, 0x3FF20000 to 0x4FFFFFFF: the TCM,
 * ROM, L2MEM and the external memory windows), the AHB bus for everything
 * else but the core's own CLIC and CLINT (0x20000000 to 0x2FFFFFFF), which
 * are on no bus (-1).
 */
int esp32p4_cpu_bus(uint32_t addr, bool fetch)
{
    if ((addr >= 0x30000000 && addr < 0x3ff00000) ||
        (addr >= 0x3ff20000 && addr < 0x50000000)) {
        return fetch ? ESP32P4_CPU_BUS_IBUS : ESP32P4_CPU_BUS_DBUS;
    }
    if (addr >= 0x20000000 && addr < 0x30000000) {
        return -1;
    }
    return ESP32P4_CPU_BUS_AHB;
}

void esp32p4_assist_connect(ESP32P4CPU *cpu, const ESP32P4AssistOps *ops,
                            void *opaque)
{
    cpu->assist_ops = ops;
    cpu->assist_opaque = opaque;
}

/* The SP monitor's view of SP: below the minimum, above the maximum. */
static uint32_t esp32p4_assist_sp_raw(ESP32P4CPU *cpu)
{
    uint32_t sp = cpu->parent_obj.env.gpr[xSP];
    uint32_t raw = 0;

    if (sp < qatomic_read(&cpu->spmon_min)) {
        raw |= ESP32P4_ASSIST_SP_MIN;
    }
    if (sp > qatomic_read(&cpu->spmon_max)) {
        raw |= ESP32P4_ASSIST_SP_MAX;
    }
    return raw;
}

/*
 * An instruction at pc wrote SP across a monitored bound: report SP's
 * place to the debug assistant.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
void helper_esp32p4_sp_check(CPURISCVState *env, target_ulong pc)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    uint32_t raw = esp32p4_assist_sp_raw(cpu);

    qatomic_set(&cpu->spmon_out, raw != 0);
    if (cpu->assist_ops) {
        BQL_LOCK_GUARD();
        cpu->assist_ops->sp(cpu->assist_opaque, env->mhartid, raw, pc);
    }
}

/*
 * The monitored bounds changed (with the BQL held): SP is compared with
 * them at once, at the core's current PC, so that SP already outside
 * them is reported.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
void esp32p4_assist_sp_update(ESP32P4CPU *cpu)
{
    uint32_t raw = esp32p4_assist_sp_raw(cpu);

    qatomic_set(&cpu->spmon_out, raw != 0);
    if (cpu->assist_ops) {
        cpu->assist_ops->sp(cpu->assist_opaque, cpu->parent_obj.env.mhartid,
                            raw, cpu->parent_obj.env.pc);
    }
}

/*
 * Region r of the debug assistant's monitors (DRAM0 regions 0 and 1, then
 * PIF regions 0 and 1) watches the bus it is on: the DBUS for the DRAM0
 * regions, the AHB bus for the PIF ones.
 */
static int esp32p4_assist_region_bus(int r)
{
    return r < 2 ? ESP32P4_CPU_BUS_DBUS : ESP32P4_CPU_BUS_AHB;
}

/* Whether a monitored region overlaps the page, which is then not cached. */
bool esp32p4_assist_watches_page(ESP32P4CPU *cpu, vaddr page)
{
    uint32_t watch = qatomic_read(&cpu->assist_watch);
    uint64_t last = (uint64_t)page + TARGET_PAGE_SIZE - 1;

    for (int r = 0; r < ESP32P4_ASSIST_REGIONS; r++) {
        if ((watch >> (2 * r)) & 3 &&
            cpu->assist_lo[r] <= last && cpu->assist_hi[r] >= page) {
            return true;
        }
    }
    return false;
}

/*
 * A load or store the core makes, which the PMP and PMA allow: report it
 * to the debug assistant if it touches a byte of a region watching it,
 * with the PC of the instruction making it, found from the translated
 * code without disturbing the core's state, and SP.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
void esp32p4_assist_access(ESP32P4CPU *cpu, vaddr addr, int size,
                           MMUAccessType access_type, uintptr_t retaddr)
{
    CPUState *cs = CPU(cpu);
    CPURISCVState *env = &cpu->parent_obj.env;
    uint32_t watch = qatomic_read(&cpu->assist_watch);
    uint64_t last = (uint64_t)addr + MAX(size, 1) - 1;
    int bit = access_type == MMU_DATA_STORE ? 1 : 0;
    int bus = esp32p4_cpu_bus(addr, false);
    uint32_t hits = 0;
    uint64_t data[INSN_START_WORDS];
    uint32_t pc = env->pc;

    for (int r = 0; r < ESP32P4_ASSIST_REGIONS; r++) {
        if ((watch & BIT(2 * r + bit)) &&
            esp32p4_assist_region_bus(r) == bus &&
            cpu->assist_lo[r] <= last && cpu->assist_hi[r] >= addr) {
            hits |= BIT(2 * r + bit);
        }
    }
    if (!hits || !cpu->assist_ops) {
        return;
    }
    if (retaddr && cpu_unwind_state_data(cs, retaddr, data)) {
        pc = tcg_cflags_has(cs, CF_PCREL)
             ? (env->pc & TARGET_PAGE_MASK) | data[0] : data[0];
    }
    BQL_LOCK_GUARD();
    cpu->assist_ops->region(cpu->assist_opaque, env->mhartid, hits, pc,
                            env->gpr[xSP]);
}

void esp32p4_trace_connect(ESP32P4CPU *cpu, const ESP32P4TraceOps *ops,
                           void *opaque)
{
    cpu->trace_ops = ops;
    cpu->trace_opaque = opaque;
}

/*
 * The trace encoder starts or stops tracing the core: the core leaves its
 * chain of translated blocks, translated with or without the reports, at
 * the next block. What the trace had of the instruction last started
 * goes with the trace that had it.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
void esp32p4_trace_set_enabled(ESP32P4CPU *cpu, bool en)
{
    if (qatomic_read(&cpu->trace_en) != en) {
        qatomic_set(&cpu->trace_started, false);
        qatomic_set(&cpu->trace_unwound, false);
        qatomic_set(&cpu->trace_en, en);
        cpu_exit(CPU(cpu));
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.debug]
 * The core unwinds the instruction it is executing to its start
 * (cpu_restore_state()): it has not retired, and will execute again or
 * trap, its start already reported. @unwound false takes that back, for
 * an instruction that completed though the core unwound to it.
 */
void esp32p4_trace_unwound(ESP32P4CPU *cpu, bool unwound)
{
    cpu->trace_unwound = unwound && cpu->trace_started;
}

/*
 * [spec:nuos:req:emu.esp32p4.debug]
 * An instruction starts: the one unwound starting again is the same
 * instruction, which the trace has, and is not reported twice. The
 * encoder's stall request stops the core before it: the core unwinds to
 * its start, not having executed it, its start reported
 * (esp32p4_restore_state_to_opc()), and waits until the encoder lets it
 * go on (esp32p4_cpu_trace_stall()).
 */
void helper_esp32p4_trace_insn(CPURISCVState *env, target_ulong pc,
                               uint32_t kind)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    CPUState *cs = CPU(cpu);

    if (cpu->trace_unwound) {
        cpu->trace_unwound = false;
        if (pc == cpu->trace_start_pc) {
            return;
        }
    }
    cpu->trace_started = cpu->trace_ops && qatomic_read(&cpu->trace_en);
    if (cpu->trace_started) {
        cpu->trace_start_pc = pc;
        if (cpu->trace_ops->insn(cpu->trace_opaque, pc, kind, env->priv)) {
            cpu_restore_state(cs, GETPC());
            cs->halted = 1;
            cs->exception_index = EXCP_HLT;
            cpu_loop_exit(cs);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.debug]
 * A trap: the instruction last started retired before it as @retired
 * says, unless the core unwound it, when it did not.
 */
void esp32p4_trace_trap(ESP32P4CPU *cpu, uint32_t epc, uint32_t cause,
                        bool interrupt, uint32_t tval, int priv,
                        bool retired)
{
    if (cpu->trace_unwound) {
        retired = false;
    }
    cpu->trace_unwound = false;
    cpu->trace_started = false;
    if (cpu->trace_ops && qatomic_read(&cpu->trace_en)) {
        cpu->trace_ops->trap(cpu->trace_opaque, epc, cause, interrupt, tval,
                             cpu->parent_obj.env.pc, priv, retired);
    }
}
