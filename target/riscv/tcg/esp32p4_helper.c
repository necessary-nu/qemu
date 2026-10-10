/*
 * Espressif ESP32-P4 HP core: hardware loop and trap return helpers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.7.1
 * (Hardware Loop), Register 2.67 (mexstatus) and 2.11.3 (Hardware
 * Trigger).
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/icount.h"
#include "accel/tcg/cpu-loop.h"
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
 * the whole TB on entry: give back the instructions after it.
 */
static void esp32p4_hwlp_charge(CPURISCVState *env, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    if (icount_enabled() && cpu_restore_state(cs, ra)) {
        cs->neg.icount_decr.u16.low--;
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
 * After MRET, given mstatus from before it: the core keeps mstatus.MPRV,
 * as privileged architecture 1.10 has it; the trigger enable
 * tcontrol.MTE comes back from MPTE; mexstatus.PBEXE comes back from
 * PPBEXE, which becomes set; the PMP and fetch error flags clear, and so
 * does the bus error flag when mexstatus.NMFT is set.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void helper_esp32p4_mret(CPURISCVState *env, uint64_t old_mstatus)
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
}
