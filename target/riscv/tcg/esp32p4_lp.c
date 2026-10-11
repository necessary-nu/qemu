/*
 * Espressif ESP32-P4 LP core control and status registers and triggers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 5
 * (Low-Power CPU): 5.3 (CSRs), 5.4 (Interrupts and Exceptions), 5.5
 * (Debugging), 5.6 (Hardware Trigger) and 5.7 (Performance Counter).
 *
 * The LP core has the CSRs of TRM 5.3.1 and no others: mhartid, mstatus,
 * misa, mie, mtvec, mscratch, mepc, mcause, mtval, mip, tselect, tdata1,
 * tdata2, the debug CSRs, mcycle(h), minstret(h), mhpmcounter3 to 12(h)
 * and mcountinhibit. Any other CSR, and the debug CSRs dcsr, dpc,
 * dscratch0 and dscratch1 outside debug mode (which software never
 * executes in), raises an illegal instruction exception. Of those it has:
 *
 *   - mie enables, and mip shows, the interrupts of IDs 3, 7, 11 and 16 to
 *     30; the other bits read zero. mip follows the interrupt lines, which
 *     the sources clear: writing it has no effect.
 *   - mtvec has only the vectored mode, MODE reading 1, and a 256-byte
 *     aligned base (BASE, bits 31:8). An exception enters at the base and
 *     interrupt n at base + 4 * n.
 *   - mcause keeps the interrupt flag and a 5-bit code; an interrupt's code
 *     is its ID, even the IDs RISC-V reserves for the core's own
 *     interrupts (3, 7, 11).
 *   - misa is hard-wired (RV32IMAC), mhartid reads 0, and mstatus, mscratch,
 *     mepc and mtval are the standard registers of a machine-mode-only
 *     core of privileged architecture 1.10.
 *   - mcycle, minstret, the event counters mhpmcounter3 to 12 and
 *     mcountinhibit are both cores' counters, in esp32p4_counters.c.
 *   - The two triggers (tselect, tdata1 as mcontrol, tdata2 as maddress):
 *     only the execute bit of mcontrol is writable. The type reads 2, the
 *     action 1 (enter debug mode) and M 1, the core running in machine mode
 *     alone; the TRM's register table shows M read-only, and a trigger
 *     whose M were clear could never fire. A trigger with execute set fires
 *     before the instruction at exactly maddress, and the core enters
 *     debug mode: dpc takes the PC and dcsr's cause reads 2, and the core
 *     halts there, as the debugger it waits for is not modelled, until it
 *     is reset. Writing tselect a value above 1 selects trigger 1.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "accel/tcg/cpu-loop.h"
#include "exec/breakpoint.h"
#include "target/riscv/esp32p4_cpu.h"

#define LP_MTVEC_BASE           (~(target_ulong)0xff)
#define LP_MTVEC_VECTORED       1
#define LP_MCAUSE_MASK          (BIT(31) | 0x1f)

/* mcontrol (TRM register 5.25). */
#define LP_MCONTROL_TYPE2       (2u << 28)
#define LP_MCONTROL_ACTION_DM   (1u << 12)
#define LP_MCONTROL_M           BIT(6)
#define LP_MCONTROL_EXECUTE     BIT(2)
#define LP_MCONTROL_RESET       (LP_MCONTROL_TYPE2 | LP_MCONTROL_ACTION_DM | \
                                 LP_MCONTROL_M)

/* dcsr: xdebugver 4, prv machine mode, and the cause of entry. */
#define LP_DCSR_XDEBUGVER       (4u << 28)
#define LP_DCSR_CAUSE_SHIFT     6
#define LP_DCSR_CAUSE_TRIGGER   2
#define LP_DCSR_PRV_M           3

/* [spec:nuos:req:emu.esp32p4.lp-core] */
ESP32P4LPCsrKind esp32p4_lp_csr_kind(CPURISCVState *env, int csrno)
{
    if (!esp32p4_env_is_lp(env)) {
        return ESP32P4_LP_CSR_STD;
    }
    switch (csrno) {
    case CSR_MHARTID:
    case CSR_MSTATUS:
    case CSR_MISA:
    case CSR_MSCRATCH:
    case CSR_MEPC:
    case CSR_MTVAL:
        return ESP32P4_LP_CSR_STD;
    case CSR_MIE:
    case CSR_MIP:
    case CSR_MTVEC:
    case CSR_MCAUSE:
    case CSR_MCOUNTINHIBIT:
    case CSR_MCYCLE:
    case CSR_MINSTRET:
    case CSR_MCYCLEH:
    case CSR_MINSTRETH:
    case CSR_MHPMCOUNTER3 ... CSR_MHPMCOUNTER3 + ESP32P4_LP_HPM_COUNTERS - 1:
    case CSR_MHPMCOUNTER3H ... CSR_MHPMCOUNTER3H + ESP32P4_LP_HPM_COUNTERS - 1:
    case CSR_TSELECT:
    case CSR_TDATA1:
    case CSR_TDATA2:
        return ESP32P4_LP_CSR_OWN;
    default:
        return ESP32P4_LP_CSR_ABSENT;
    }
}

/* Triggers */

/* [spec:nuos:req:emu.esp32p4.lp-core] */
void esp32p4_lp_triggers_rebuild(ESP32P4CPU *cpu)
{
    CPUState *cs = CPU(cpu);

    cpu_breakpoint_remove_all(cs, BP_CPU);
    for (int t = 0; t < ESP32P4_LP_TRIGGERS; t++) {
        if (cpu->tdata1[t] & LP_MCONTROL_EXECUTE) {
            cpu_breakpoint_insert(cs, cpu->tdata2[t], BP_CPU, NULL);
        }
    }
}

void esp32p4_lp_triggers_reset(ESP32P4CPU *cpu)
{
    cpu->tselect = 0;
    for (int t = 0; t < ESP32P4_LP_TRIGGERS; t++) {
        cpu->tdata1[t] = LP_MCONTROL_RESET;
        cpu->tdata2[t] = 0;
    }
    cpu->lp_debug_mode = false;
    cpu->lp_dpc = 0;
    cpu->lp_dcsr = LP_DCSR_XDEBUGVER | LP_DCSR_PRV_M;
    esp32p4_lp_triggers_rebuild(cpu);
}

/* An execute trigger matches the instruction about to execute. */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
bool esp32p4_lp_debug_check_breakpoint(CPUState *cs)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);
    target_ulong pc = cpu->parent_obj.env.pc;

    for (int t = 0; t < ESP32P4_LP_TRIGGERS; t++) {
        if ((cpu->tdata1[t] & LP_MCONTROL_EXECUTE) && cpu->tdata2[t] == pc) {
            return true;
        }
    }
    return false;
}

/*
 * A trigger fired: the core enters debug mode before the instruction,
 * with dpc at it and dcsr's cause the trigger, and halts there.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
void esp32p4_lp_debug_excp_handler(CPUState *cs)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);
    CPURISCVState *env = &cpu->parent_obj.env;

    if (cs->watchpoint_hit || !cpu_breakpoint_test(cs, env->pc, BP_CPU)) {
        return;
    }
    cpu->lp_debug_mode = true;
    cpu->lp_dpc = env->pc;
    cpu->lp_dcsr = deposit32(cpu->lp_dcsr, LP_DCSR_CAUSE_SHIFT, 3,
                             LP_DCSR_CAUSE_TRIGGER);
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/* CSR accesses */

/* [spec:nuos:req:emu.esp32p4.lp-core] */
RISCVException esp32p4_lp_csr_read(CPURISCVState *env, int csrno,
                                   target_ulong *val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    switch (csrno) {
    case CSR_MIE:
        *val = env->mie & ESP32P4_LP_IRQ_MASK;
        return RISCV_EXCP_NONE;
    case CSR_MIP:
        *val = env->mip & ESP32P4_LP_IRQ_MASK;
        return RISCV_EXCP_NONE;
    case CSR_MTVEC:
        *val = env->mtvec;
        return RISCV_EXCP_NONE;
    case CSR_MCAUSE:
        *val = env->mcause & LP_MCAUSE_MASK;
        return RISCV_EXCP_NONE;
    case CSR_TSELECT:
        *val = cpu->tselect;
        return RISCV_EXCP_NONE;
    case CSR_TDATA1:
        *val = cpu->tdata1[cpu->tselect];
        return RISCV_EXCP_NONE;
    case CSR_TDATA2:
        *val = cpu->tdata2[cpu->tselect];
        return RISCV_EXCP_NONE;
    default:
        return esp32p4_counters_read(env, csrno, val);
    }
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
RISCVException esp32p4_lp_csr_write(CPURISCVState *env, int csrno,
                                    target_ulong val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    switch (csrno) {
    case CSR_MIE:
        env->mie = (env->mie & ~(uint64_t)ESP32P4_LP_IRQ_MASK) |
                   (val & ESP32P4_LP_IRQ_MASK);
        return RISCV_EXCP_NONE;
    case CSR_MIP:
        return RISCV_EXCP_NONE;
    case CSR_MTVEC:
        env->mtvec = (val & LP_MTVEC_BASE) | LP_MTVEC_VECTORED;
        return RISCV_EXCP_NONE;
    case CSR_MCAUSE:
        env->mcause = val & LP_MCAUSE_MASK;
        return RISCV_EXCP_NONE;
    case CSR_TSELECT:
        cpu->tselect = MIN(val, ESP32P4_LP_TRIGGERS - 1);
        return RISCV_EXCP_NONE;
    case CSR_TDATA1:
        cpu->tdata1[cpu->tselect] = LP_MCONTROL_RESET |
                                    (val & LP_MCONTROL_EXECUTE);
        esp32p4_lp_triggers_rebuild(cpu);
        return RISCV_EXCP_NONE;
    case CSR_TDATA2:
        cpu->tdata2[cpu->tselect] = val;
        esp32p4_lp_triggers_rebuild(cpu);
        return RISCV_EXCP_NONE;
    default:
        return esp32p4_counters_write(env, csrno, val);
    }
}
