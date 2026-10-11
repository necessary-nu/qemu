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
 *   - mcycle counts the core's clock cycles while its clock runs, at the
 *     clock's current frequency; mcountinhibit's CY stops it. The model
 *     executes an instruction a cycle (as icount times the core), so
 *     minstret counts the same cycles, IR stopping it, including those in
 *     which the core waits for an interrupt; no instruction retirement is
 *     tracked apart from them. mhpmcounter3 to 12, 40 bits wide, hold
 *     what is written but do not count their events (wait cycles, memory
 *     reads and writes, jumps, branches, compressed instructions, multiply
 *     and divide waits), which is logged as unimplemented.
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
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "accel/tcg/cpu-loop.h"
#include "exec/breakpoint.h"
#include "hw/core/clock.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "target/riscv/esp32p4_cpu.h"

#define LP_MTVEC_BASE           (~(target_ulong)0xff)
#define LP_MTVEC_VECTORED       1
#define LP_MCAUSE_MASK          (BIT(31) | 0x1f)

/* mcountinhibit: CY, IR and HPM3 to 12. */
#define LP_COUNTINHIBIT_CY      BIT(0)
#define LP_COUNTINHIBIT_IR      BIT(2)
#define LP_COUNTINHIBIT_MASK    (LP_COUNTINHIBIT_CY | LP_COUNTINHIBIT_IR | \
                                 MAKE_64BIT_MASK(ESP32P4_LP_HPM_FIRST, \
                                                 ESP32P4_LP_HPM_COUNTERS))
#define LP_HPM_BITS             40

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

/* The counters mcycle and minstret, by their place in lp_ctr_base[]. */
enum {
    LP_CTR_CYCLE,
    LP_CTR_INSTRET,
};

static const char *const esp32p4_lp_hpm_events[ESP32P4_LP_HPM_COUNTERS] = {
    "memory access wait cycles",
    "instruction fetch wait cycles",
    "memory reads",
    "memory writes",
    "unconditional jumps",
    "branches",
    "taken branches",
    "compressed instructions",
    "multiplication wait cycles",
    "division wait cycles",
};

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

/* Counters */

static bool esp32p4_lp_running(ESP32P4CPU *cpu)
{
    return !cpu->reset && !cpu->clock_stopped && !cpu->disabled;
}

/*
 * Bring lp_cycles up to the present: the cycles of the core's clock while
 * the core runs, counted from when it last started running or its clock
 * changed (lp_edges). Called with the BQL held, before the core starts or
 * stops running and before its clock changes rate.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static uint64_t esp32p4_lp_cycles(ESP32P4CPU *cpu)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t cycles;

    if (!cpu->lp_clk || !esp32p4_lp_running(cpu) ||
        !clock_is_enabled(cpu->lp_clk)) {
        cpu->lp_edges.origin_ns = MAX(now, cpu->lp_edges.origin_ns);
        cpu->lp_synced = 0;
        return cpu->lp_cycles;
    }
    cycles = esp32p4_clk_count_at(&cpu->lp_edges, now);
    if (cycles > cpu->lp_synced) {
        cpu->lp_cycles += cycles - cpu->lp_synced;
        cpu->lp_synced = cycles;
    }
    return cpu->lp_cycles;
}

void esp32p4_lp_counters_update(ESP32P4CPU *cpu)
{
    esp32p4_lp_cycles(cpu);
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
void esp32p4_lp_counters_clock_changed(ESP32P4CPU *cpu)
{
    esp32p4_clk_count_start(&cpu->lp_edges, cpu->lp_clk,
                            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    cpu->lp_synced = 0;
}

static uint64_t esp32p4_lp_ctr(ESP32P4CPU *cpu, int c, uint32_t inhibit)
{
    uint64_t cycles = esp32p4_lp_cycles(cpu);

    if (cpu->lp_mcountinhibit & inhibit) {
        return cpu->lp_ctr_base[c];
    }
    return cpu->lp_ctr_base[c] + cycles - cpu->lp_ctr_ref[c];
}

static void esp32p4_lp_ctr_set(ESP32P4CPU *cpu, int c, uint64_t val)
{
    cpu->lp_ctr_base[c] = val;
    cpu->lp_ctr_ref[c] = esp32p4_lp_cycles(cpu);
}

void esp32p4_lp_counters_reset(ESP32P4CPU *cpu)
{
    cpu->lp_mcountinhibit = 0;
    esp32p4_lp_ctr_set(cpu, LP_CTR_CYCLE, 0);
    esp32p4_lp_ctr_set(cpu, LP_CTR_INSTRET, 0);
    memset(cpu->lp_hpm, 0, sizeof(cpu->lp_hpm));
}

/* mcountinhibit: a counter stops and restarts where it was. */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_set_countinhibit(ESP32P4CPU *cpu, uint32_t val)
{
    static const uint32_t bit[] = {
        [LP_CTR_CYCLE] = LP_COUNTINHIBIT_CY,
        [LP_CTR_INSTRET] = LP_COUNTINHIBIT_IR,
    };
    uint64_t now[2];

    for (int c = 0; c < 2; c++) {
        now[c] = esp32p4_lp_ctr(cpu, c, bit[c]);
    }
    cpu->lp_mcountinhibit = val & LP_COUNTINHIBIT_MASK;
    for (int c = 0; c < 2; c++) {
        esp32p4_lp_ctr_set(cpu, c, now[c]);
    }
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
static uint64_t *esp32p4_lp_hpm(ESP32P4CPU *cpu, int n)
{
    static bool logged[ESP32P4_LP_HPM_COUNTERS];

    if (!(cpu->lp_mcountinhibit & BIT(ESP32P4_LP_HPM_FIRST + n)) &&
        !logged[n]) {
        logged[n] = true;
        qemu_log_mask(LOG_UNIMP, "esp32p4: the LP core's mhpmcounter%d does "
                      "not count %s\n", ESP32P4_LP_HPM_FIRST + n,
                      esp32p4_lp_hpm_events[n]);
    }
    return &cpu->lp_hpm[n];
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
    int n;

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
        break;
    }

    BQL_LOCK_GUARD();
    switch (csrno) {
    case CSR_MCOUNTINHIBIT:
        *val = cpu->lp_mcountinhibit;
        break;
    case CSR_MCYCLE:
        *val = esp32p4_lp_ctr(cpu, LP_CTR_CYCLE, LP_COUNTINHIBIT_CY);
        break;
    case CSR_MCYCLEH:
        *val = esp32p4_lp_ctr(cpu, LP_CTR_CYCLE, LP_COUNTINHIBIT_CY) >> 32;
        break;
    case CSR_MINSTRET:
        *val = esp32p4_lp_ctr(cpu, LP_CTR_INSTRET, LP_COUNTINHIBIT_IR);
        break;
    case CSR_MINSTRETH:
        *val = esp32p4_lp_ctr(cpu, LP_CTR_INSTRET, LP_COUNTINHIBIT_IR) >> 32;
        break;
    case CSR_MHPMCOUNTER3H ... CSR_MHPMCOUNTER3H + ESP32P4_LP_HPM_COUNTERS - 1:
        n = csrno - CSR_MHPMCOUNTER3H;
        *val = *esp32p4_lp_hpm(cpu, n) >> 32;
        break;
    default:
        n = csrno - CSR_MHPMCOUNTER3;
        *val = *esp32p4_lp_hpm(cpu, n);
        break;
    }
    return RISCV_EXCP_NONE;
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
RISCVException esp32p4_lp_csr_write(CPURISCVState *env, int csrno,
                                    target_ulong val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    uint64_t *hpm;
    uint64_t v;
    int n;

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
        break;
    }

    BQL_LOCK_GUARD();
    switch (csrno) {
    case CSR_MCOUNTINHIBIT:
        esp32p4_lp_set_countinhibit(cpu, val);
        break;
    case CSR_MCYCLE:
    case CSR_MCYCLEH:
        v = esp32p4_lp_ctr(cpu, LP_CTR_CYCLE, LP_COUNTINHIBIT_CY);
        v = csrno == CSR_MCYCLE ? deposit64(v, 0, 32, val)
                                : deposit64(v, 32, 32, val);
        esp32p4_lp_ctr_set(cpu, LP_CTR_CYCLE, v);
        break;
    case CSR_MINSTRET:
    case CSR_MINSTRETH:
        v = esp32p4_lp_ctr(cpu, LP_CTR_INSTRET, LP_COUNTINHIBIT_IR);
        v = csrno == CSR_MINSTRET ? deposit64(v, 0, 32, val)
                                  : deposit64(v, 32, 32, val);
        esp32p4_lp_ctr_set(cpu, LP_CTR_INSTRET, v);
        break;
    case CSR_MHPMCOUNTER3H ... CSR_MHPMCOUNTER3H + ESP32P4_LP_HPM_COUNTERS - 1:
        hpm = esp32p4_lp_hpm(cpu, csrno - CSR_MHPMCOUNTER3H);
        *hpm = deposit64(*hpm, 32, LP_HPM_BITS - 32, val);
        break;
    default:
        n = csrno - CSR_MHPMCOUNTER3;
        hpm = esp32p4_lp_hpm(cpu, n);
        *hpm = deposit64(*hpm, 0, 32, val);
        break;
    }
    return RISCV_EXCP_NONE;
}
