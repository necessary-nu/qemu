/*
 * Espressif ESP32-P4 HP and LP RISC-V cores
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 2
 * (High-Performance CPU), chapter 5 (Low-Power CPU) and chapter 11 (Reset
 * and Clock).
 *
 * The two HP cores implement RV32IMAFC with Zcb, Zcmp and Zcmt (Zc
 * specification v1.0.4-3, so with Zca and, on RV32F, Zcf) and the Zb
 * extensions (Zba, Zbb, Zbc and Zbs of the v1.0.0 specification), in
 * machine and user mode, with the hardware loop extension (Xesploop),
 * the Processor Instruction Extensions (Xespv2p2), Espressif's CSRs and a
 * three-trigger debug module; the CSRs, triggers, hardware loops and PIE
 * are in tcg/esp32p4_*.c, as are the PMP and the PMA checker, which check
 * every access the core makes. The LP core implements RV32IMAC in machine
 * mode only, without PMP or PMA. The HP core takes its interrupts from its
 * CLIC (tcg/esp32p4_clic.c).
 *
 * The LP core (TRM chapter 5) takes the interrupts of its interrupt
 * controller as the levels of mip's bits 3, 7, 11 and 16 to 30, which mie
 * enables, vectored through mtvec; the lowest pending ID is taken first.
 * Its CSRs and two triggers are in tcg/esp32p4_lp.c. As its
 * reset is released it starts where the machine says (ESP32P4LPBootFn),
 * from LP_SYSTEM_LP_CORE_BOOT_ADDR, with mtvec at the boot address's
 * vector table, and its reset returns its CSRs to their reset values. An
 * exception records the PC in LP_SYSTEM_LP_CPU_EXC_PC (interrupts do not)
 * and is signalled to the PMU, as its LP CPU exception wakeup source, and
 * to the ETM, as ULP_EVT_ERR_INTR. A trigger takes it into debug mode,
 * where it halts. Misaligned loads and stores are handled by the core; a
 * misaligned atomic instruction, LR.W included, raises cause 6.
 *
 * misa on an HP core reads 0x40901127: RV32 with A, B, C, F, I, M, U and
 * X. The TRM's register table gives B and X but clears F, while the TRM
 * itself and the silicon describe an RV32IMAFC core, and earlier silicon
 * reads 0x40901125, F included; B is new with the Zb extensions of v3.x.
 *
 * QEMU only provides the Zc and Zb extensions to cores of privileged
 * specification 1.12, so the HP cores are of that version, without the
 * 1.11 and 1.12 CSRs (mstatush, menvcfg, mconfigptr) the core lacks and
 * with MRET keeping mstatus.MPRV, as the TRM's 1.10 has it; the LP core is
 * of 1.10.
 *
 * Where the HP core departs from the ratified Zcmp, cm.push stores ra at
 * the highest address and the rest of its list below in register order,
 * and cm.pop and cm.popret load from there: ESP-IDF builds for the core
 * with GCC's -mno-cm-push-reverse, which lays frames out that way.
 * Misaligned loads and stores are handled by the core, the only misaligned
 * exception being cause 6, which a misaligned LR.W raises as well as SC.W
 * and the AMOs.
 *
 * Each core has a reset input and a clock enable, which the chip's reset
 * and clock control drive. A core whose reset is held does not run and
 * stays in its reset state, starting from its reset vector once released;
 * a core whose clock is stopped stops where it is and carries on when the
 * clock restarts. Neither takes an interrupt meanwhile, not even to leave
 * WFI. Both act at the end of the code block the core is executing, so
 * when a core stops itself, the instructions after the store in the same
 * straight-line run complete first. A core whose register write stops
 * another core goes on, from the end of its own code block, only once the
 * other has stopped: on the host's clock each core runs on a thread of
 * its own, which the host may not run for a while, and the stopped core
 * must not be seen to go on running.
 *
 * An HP core tells the interrupt matrix whether it is in machine mode,
 * through its ESP32P4_CPU_M_MODE output, as each trap and trap return
 * changes the mode.
 *
 * Both cores' cycle, instruction and event counters are in
 * tcg/esp32p4_counters.c: each counts its own clock's cycles
 * (ESP32P4_CPU_CLK) and its own retired instructions, which the code QEMU
 * translates for it counts.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/main-loop.h"
#include "exec/icount.h"
#include "hw/core/irq.h"
#include "hw/core/qdev.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/sysemu-cpu-ops.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "system/tcg.h"
#include "exec/translation-block.h"
#include "hw/misc/esp32p4_pms.h"
#include "target/riscv/esp32p4_cpu.h"
#include "internals.h"
#ifdef CONFIG_TCG
#include "accel/tcg/cpu-ops.h"
#include "accel/tcg/cpu-loop.h"
#include "target/riscv/tcg/csr.h"
#endif

#define ESP32P4_HP_MVENDORID 0x612
#define ESP32P4_HP_MARCHID   0x80000003
#define ESP32P4_HP_MIMPID    0x1

/* The HP core's programmable counters: mhpmcounter8, 9 and 13. */
#define ESP32P4_HP_PMU_MASK (BIT(8) | BIT(9) | BIT(13))

#define ESP32P4_MEXSTATUS_RESET (ESP32P4_MEXSTATUS_CLIC_INHV | \
                                 ESP32P4_MEXSTATUS_NMFT |      \
                                 ESP32P4_MEXSTATUS_PPBEXE)

ObjectClass *esp32p4_lp_cpu_class;

/* The clock enable is low or, on the LP core, LP_CORE_DISABLE high. */
static bool esp32p4_cpu_clock_off(ESP32P4CPU *cpu)
{
    return qatomic_read(&cpu->clock_stopped) || qatomic_read(&cpu->disabled);
}

static bool esp32p4_cpu_held(ESP32P4CPU *cpu)
{
    return qatomic_read(&cpu->reset) || esp32p4_cpu_clock_off(cpu);
}

/*
 * A core in WFI wakes for the interrupts that wake it from the CLIC, on
 * the HP core, as well as the standard ones.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
static bool esp32p4_cpu_has_work(CPUState *cs)
{
    ESP32P4CPU *cpu = container_of(cs, ESP32P4CPU, parent_obj.parent_obj);

    if (esp32p4_cpu_held(cpu) || qatomic_read(&cpu->lp_debug_mode) ||
        qatomic_read(&cpu->bus_wait)) {
        return false;
    }
    if (qatomic_read(&cpu->wakeup)) {
        return true;
    }
#ifdef CONFIG_TCG
    if (esp32p4_env_is_hp(&cpu->parent_obj.env) && esp32p4_clic_wakes(cpu)) {
        return true;
    }
#endif
    return riscv_cpu_has_work(cs);
}

void esp32p4_cpu_clic_connect(ESP32P4CPU *cpu, const ESP32P4ClicOps *ops,
                              void *opaque)
{
    cpu->clic_ops = ops;
    cpu->clic_opaque = opaque;
}

/*
 * The CLIC presents a new interrupt, or none: the core reconsiders before
 * its next instruction, and a core in WFI whether to wake.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
void esp32p4_cpu_clic_present(ESP32P4CPU *cpu, uint32_t sel)
{
    CPUState *cs = CPU(cpu);

    qatomic_set(&cpu->clic_sel, sel);
    if (sel & ESP32P4_CLIC_SEL_VALID) {
        cpu_interrupt(cs, ESP32P4_CPU_INTERRUPT_CLIC);
    } else {
        cpu_reset_interrupt(cs, ESP32P4_CPU_INTERRUPT_CLIC);
    }
}

uint8_t esp32p4_cpu_mintthresh(ESP32P4CPU *cpu)
{
    return qatomic_read(&cpu->mintthresh);
}

/*
 * mintthresh written through the CLIC's registers, perhaps by the other
 * core: the core reconsiders the interrupt the CLIC presents.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
void esp32p4_cpu_set_mintthresh(ESP32P4CPU *cpu, uint8_t th)
{
    qatomic_set(&cpu->mintthresh, th);
    esp32p4_cpu_clic_present(cpu, qatomic_read(&cpu->clic_sel));
}

bool esp32p4_cpu_mid_block(void)
{
    return icount_enabled() && current_cpu && !current_cpu->neg.can_do_io;
}

/*
 * [spec:nuos:req:emu.esp32p4.machine]
 * The core whose own load or store this is: the core running on this
 * thread, whose identity the access carries (esp32p4_hp_tlb_fill()), not
 * a block's access that the core's store has set off.
 */
ESP32P4CPU *esp32p4_cpu_bus_waiter(MemTxAttrs attrs)
{
#ifdef CONFIG_TCG
    CPUState *cs = current_cpu;
    ESP32P4CPU *cpu;
    CPURISCVState *env;
    uint16_t rid;

    if (!tcg_enabled() || !cs || !qemu_cpu_is_self(cs) || !cs->mem_io_pc ||
        attrs.unspecified ||
        !object_dynamic_cast(OBJECT(cs), TYPE_ESP32P4_CPU)) {
        return NULL;
    }
    cpu = ESP32P4_CPU(cs);
    env = &cpu->parent_obj.env;
    rid = ESP32P4_REQUESTER_CPU(esp32p4_env_is_hp(env)
                                ? env->mhartid : ESP32P4_MASTER_LP_CORE);
    if (attrs.requester_id == rid) {
        return cpu;
    }
#endif
    return NULL;
}

/*
 * [spec:nuos:req:emu.esp32p4.machine]
 * The access fails, and the core's transaction failure hook, outside the
 * device and the memory system, stops the core there
 * (esp32p4_cpu_bus_wait_check()).
 */
MemTxResult esp32p4_cpu_bus_hold(ESP32P4CPU *cpu)
{
    cpu->bus_hold = true;
    return MEMTX_ERROR;
}

#ifdef CONFIG_TCG
/*
 * [spec:nuos:req:emu.esp32p4.machine]
 * A failed access that a device holds: the core unwinds to the start of
 * the instruction, which has not retired (esp32p4_restore_state_to_opc())
 * and executes again when the core goes on, as QEMU's MIPS ITC storage
 * blocks a core. Its wait begins once it has given back the
 * instruction's cycle.
 */
static void esp32p4_cpu_bus_wait_check(CPUState *cs, uintptr_t retaddr)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (!cpu->bus_hold) {
        return;
    }
    cpu->bus_hold = false;
    qatomic_set(&cpu->bus_wait, true);
    cpu_restore_state(cs, retaddr);
    esp32p4_counters_bus_wait(cpu, true);
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}
#endif

/*
 * Run on the core: its access may go through now. The access completes
 * before the core takes an interrupt, one pending by now included: the
 * core makes it again in a block of that one instruction, translated not
 * to take interrupts first (CF_NOIRQ), as cpu_io_recompile() replays an
 * instruction.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_cpu_bus_go(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (!qatomic_read(&cpu->bus_wait)) {
        return;
    }
    qatomic_set(&cpu->bus_wait, false);
#ifdef CONFIG_TCG
    esp32p4_counters_bus_wait(cpu, false);
    cs->cflags_next_tb = cs->tcg_cflags | CF_NOIRQ | 1;
#endif
    if (!esp32p4_cpu_held(cpu) && !cpu->clock_halted) {
        cs->halted = 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.machine] */
void esp32p4_cpu_bus_release(ESP32P4CPU *cpu)
{
    if (qatomic_read(&cpu->bus_wait)) {
        async_run_on_cpu(CPU(cpu), esp32p4_cpu_bus_go, RUN_ON_CPU_NULL);
    }
}

/* Waiters for stops of cores on other threads: see esp32p4_cpu_await(). */
static QemuCond esp32p4_cpu_stop_cond;

static void esp32p4_cpu_stop_done(ESP32P4CPU *cpu)
{
    qatomic_dec(&cpu->stops_pending);
    qemu_cond_broadcast(&esp32p4_cpu_stop_cond);
}

/* Run on the core when its reset input rises. */
static void esp32p4_cpu_reset_asserted(CPUState *cs, run_on_cpu_data data)
{
    cpu_reset(cs);
    esp32p4_cpu_stop_done(ESP32P4_CPU(cs));
}

/*
 * Run on the core when its reset input falls. The LP core takes its start
 * address and vector table base as it leaves reset.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_cpu_reset_released(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (qatomic_read(&cpu->reset)) {
        return;
    }
    if (cpu->lp_boot) {
        uint32_t pc, mtvec;

        cpu->lp_boot(cpu->lp_boot_opaque, &pc, &mtvec);
        cpu->parent_obj.env.pc = pc;
        cpu->parent_obj.env.mtvec = mtvec;
    }
    if (esp32p4_cpu_clock_off(cpu)) {
        cpu->clock_halted = true;
        return;
    }
    cs->halted = 0;
}

/* Run on the core when its clock stops. */
static void esp32p4_cpu_clock_stop(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (esp32p4_cpu_clock_off(cpu) && !qatomic_read(&cpu->reset) &&
        !cs->halted) {
        cs->halted = 1;
        cpu->clock_halted = true;
    }
    esp32p4_cpu_stop_done(cpu);
}

/* Run on the core when its clock restarts. */
static void esp32p4_cpu_clock_start(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (esp32p4_cpu_clock_off(cpu) || qatomic_read(&cpu->reset) ||
        !cpu->clock_halted) {
        return;
    }
    cpu->clock_halted = false;
    cs->halted = 0;
}

/*
 * Run on a core that stopped another, @data, on a thread of its own, at
 * the end of the code block that did so: wait until the other has acted
 * on its stops, taking any of its own the other has queued meanwhile, so
 * that two cores stopping each other both stop.
 */
static void esp32p4_cpu_await(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *other = data.host_ptr;

    while (qatomic_read(&other->stops_pending)) {
        process_queued_cpu_work(cs);
        if (qatomic_read(&other->stops_pending)) {
            qemu_cond_timedwait_bql(&esp32p4_cpu_stop_cond, 10);
        }
    }
}

/*
 * Stop @cpu with @stop on its own thread. A core on another thread that
 * stopped it waits for that at the end of its code block (see the top of
 * the file); one sharing the thread (under icount, every core) runs only
 * between the other's code blocks anyway, as does the main loop, which
 * runs no guest code that could see the core go on.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_cpu_stop(ESP32P4CPU *cpu, run_on_cpu_func stop)
{
    CPUState *cs = CPU(cpu);

    qatomic_inc(&cpu->stops_pending);
    async_run_on_cpu(cs, stop, RUN_ON_CPU_NULL);
    if (current_cpu && current_cpu != cs && !qemu_cpu_is_self(cs)) {
        async_run_on_cpu(current_cpu, esp32p4_cpu_await,
                         RUN_ON_CPU_HOST_PTR(cpu));
    }
}

/*
 * The inputs take effect on the core's own thread; until then the core
 * neither leaves a halt nor, once it stops at the end of its code block,
 * starts again.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_cpu_set_reset(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;
    bool reset = level;

    if (cpu->reset == reset) {
        return;
    }
#ifdef CONFIG_TCG
    esp32p4_counters_update(cpu);
#endif
    qatomic_set(&cpu->reset, reset);
    if (reset) {
        esp32p4_cpu_stop(cpu, esp32p4_cpu_reset_asserted);
    } else {
        async_run_on_cpu(CPU(cpu), esp32p4_cpu_reset_released,
                         RUN_ON_CPU_NULL);
    }
}

/*
 * The clock enable, or the LP core's disable, changed from @was_off: the
 * core stops or carries on if that stopped or restarted its clock.
 */
static void esp32p4_cpu_clock_changed(ESP32P4CPU *cpu, bool was_off)
{
    bool off = esp32p4_cpu_clock_off(cpu);

    if (off && !was_off) {
        esp32p4_cpu_stop(cpu, esp32p4_cpu_clock_stop);
    } else if (!off && was_off) {
        async_run_on_cpu(CPU(cpu), esp32p4_cpu_clock_start, RUN_ON_CPU_NULL);
    }
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_cpu_set_clock(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;
    bool stopped = !level;
    bool was_off = esp32p4_cpu_clock_off(cpu);

    if (cpu->clock_stopped == stopped) {
        return;
    }
#ifdef CONFIG_TCG
    esp32p4_counters_update(cpu);
#endif
    qatomic_set(&cpu->clock_stopped, stopped);
    esp32p4_cpu_clock_changed(cpu, was_off);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_cpu_set_err_resp_dis(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;

    qatomic_set(&cpu->err_resp_dis,
                deposit32(qatomic_read(&cpu->err_resp_dis), n, 1,
                          level != 0));
}

/*
 * The HP core's dedicated wakeup input: while it is high the core does
 * not wait in WFI, and as it rises a core waiting there wakes.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_cpu_set_wakeup(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;

    qatomic_set(&cpu->wakeup, level != 0);
    if (level) {
        qemu_cpu_kick(CPU(cpu));
    }
}

#ifdef CONFIG_TCG
/*
 * The hooks of the HP core's TCG operations, wrapping the RISC-V ones
 * in esp32p4_tcg_ops.
 */
static TCGCPUOps esp32p4_tcg_ops;
static TCGCPUOps esp32p4_hp_tcg_ops;
static TCGCPUOps esp32p4_lp_tcg_ops;

static ESP32P4CPU *esp32p4_cpu_of(CPUState *cs)
{
    return container_of(cs, ESP32P4CPU, parent_obj.parent_obj);
}

/* The CSR a CSR instruction accesses, or -1 for another instruction. */
static int esp32p4_insn_csr(uint32_t insn)
{
    uint32_t funct3 = extract32(insn, 12, 3);

    if ((insn & 0x7f) != 0x73 || funct3 == 0 || funct3 == 4) {
        return -1;
    }
    return insn >> 20;
}

/* A single-precision instruction of the F extension, or a Zcf one. */
static bool esp32p4_insn_is_fp(uint32_t insn)
{
    if ((insn & 3) != 3) {
        uint32_t funct3 = extract32(insn, 13, 3);

        return (insn & 3) != 1 && (funct3 == 3 || funct3 == 7);
    }
    switch (insn & 0x7f) {
    case 0x07: /* FLW */
    case 0x27: /* FSW */
        return extract32(insn, 12, 3) == 2;
    case 0x43: /* FMADD.S */
    case 0x47: /* FMSUB.S */
    case 0x4b: /* FNMSUB.S */
    case 0x4f: /* FNMADD.S */
    case 0x53: /* OP-FP */
        return extract32(insn, 25, 2) == 0;
    default:
        return false;
    }
}

static bool esp32p4_csr_is_fp(int csr)
{
    return csr == CSR_FFLAGS || csr == CSR_FRM || csr == CSR_FCSR ||
           csr == 0x800;
}

/* A hardware loop instruction (custom-1, funct3 0 to 5). */
static bool esp32p4_insn_is_hwlp(uint32_t insn)
{
    return (insn & 0x7f) == 0x2b && extract32(insn, 12, 3) <= 5;
}

static bool esp32p4_csr_is_hwlp(int csr)
{
    return (csr >= 0x7c6 && csr <= 0x7cb) || (csr >= 0x8c0 && csr <= 0x8c5);
}

/*
 * mext_ill records an illegal instruction exception raised because the
 * instruction, or the CSR it accesses, belongs to an extension that is
 * off: F with mstatus.FS off, the hardware loops with
 * mhwloop_state_reg.STATE off, or PIE with mext_pie_status.STATE off.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.pie] */
static uint32_t esp32p4_illegal_reason(ESP32P4CPU *cpu, uint32_t insn)
{
    CPURISCVState *env = &cpu->parent_obj.env;
    int csr = esp32p4_insn_csr(insn);
    uint32_t reason = 0;

    if (!riscv_cpu_fp_enabled(env) &&
        (esp32p4_insn_is_fp(insn) || esp32p4_csr_is_fp(csr))) {
        reason |= ESP32P4_EXT_ILL_FPU;
    }
    if ((cpu->hwlp_state & ESP32P4_HWLP_STATE) == ESP32P4_EXT_OFF &&
        (esp32p4_insn_is_hwlp(insn) || esp32p4_csr_is_hwlp(csr))) {
        reason |= ESP32P4_EXT_ILL_HWLP;
    }
    if ((cpu->pie_state & ESP32P4_HWLP_STATE) == ESP32P4_EXT_OFF &&
        esp32p4_insn_is_pie(insn)) {
        reason |= ESP32P4_EXT_ILL_PIE;
    }
    return reason;
}

/*
 * Entering a trap, which on the HP core is always taken in machine mode:
 * the debug assistant's LASTPC_BEFORE_EXCEPTION takes the PC the trap is
 * taken at, the excepting instruction's or, for an interrupt, the one it
 * comes before (the core does not say which instruction it last
 * completed); mexstatus records it (EXPT_VLD) and, for an access fault,
 * whether a bus error or the PMP or PMA raised it, saves PBEXE in PPBEXE
 * and clears it; tcontrol saves MTE in MPTE and clears it, disabling the
 * triggers; and mext_ill records why an illegal instruction was illegal.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static void esp32p4_hp_trap_entry(ESP32P4CPU *cpu, int32_t excp)
{
    CPURISCVState *env = &cpu->parent_obj.env;

    cpu->exc_pc = env->pc;
    if (!(excp & RISCV_EXCP_INT_FLAG)) {
        switch (excp) {
        case RISCV_EXCP_ILLEGAL_INST:
            cpu->ext_ill |= esp32p4_illegal_reason(cpu, env->bins);
            break;
        case RISCV_EXCP_LOAD_ACCESS_FAULT:
        case RISCV_EXCP_STORE_AMO_ACCESS_FAULT:
            cpu->mexstatus |= cpu->bus_error ? ESP32P4_MEXSTATUS_BUS_ERR
                                             : ESP32P4_MEXSTATUS_PMP_ERR;
            break;
        case RISCV_EXCP_INST_ACCESS_FAULT:
            if (cpu->bus_error) {
                cpu->mexstatus |= ESP32P4_MEXSTATUS_FETCH_ERR;
            }
            break;
        default:
            break;
        }
    }
    cpu->bus_error = false;

    cpu->mexstatus &= ~ESP32P4_MEXSTATUS_PPBEXE;
    if (cpu->mexstatus & ESP32P4_MEXSTATUS_PBEXE) {
        cpu->mexstatus |= ESP32P4_MEXSTATUS_PPBEXE;
    }
    cpu->mexstatus &= ~ESP32P4_MEXSTATUS_PBEXE;
    cpu->mexstatus |= ESP32P4_MEXSTATUS_EXPT_VLD;

    cpu->tcontrol &= ~ESP32P4_TCONTROL_MPTE;
    if (cpu->tcontrol & ESP32P4_TCONTROL_MTE) {
        cpu->tcontrol |= ESP32P4_TCONTROL_MPTE;
    }
    cpu->tcontrol &= ~ESP32P4_TCONTROL_MTE;
}

/*
 * After a trap or a trap return, which may have changed the core's
 * privilege mode: the ESP32P4_CPU_M_MODE output follows it.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
void esp32p4_hp_priv_update(ESP32P4CPU *cpu)
{
    bool m_mode = cpu->parent_obj.env.priv == PRV_M;

    if (m_mode != cpu->m_mode_level) {
        BQL_LOCK_GUARD();

        cpu->m_mode_level = m_mode;
        qemu_set_irq(cpu->m_mode_irq, m_mode);
    }
}

/*
 * An exception, the only trap that the standard trap entry takes on the
 * HP core: the CLIC's own interrupts are taken by esp32p4_clic_take().
 * The PIE illegal instruction exception (cause 0x1f) leaves the
 * instruction in mtval, which QEMU's trap code only does for its own
 * illegal instruction cause. The trace encoder learns of the trap.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void esp32p4_hp_do_interrupt(CPUState *cs)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    CPURISCVState *env = &cpu->parent_obj.env;
    int prev_priv = env->priv;
    uint8_t prev_mil = cpu->mil;
    int32_t excp = cs->exception_index;

    if (excp == RISCV_EXCP_SEMIHOST) {
        esp32p4_tcg_ops.do_interrupt(cs);
        return;
    }
    esp32p4_hp_trap_entry(cpu, excp);
    esp32p4_tcg_ops.do_interrupt(cs);
    if (excp == ESP32P4_EXCP_PIE_ILLEGAL) {
        env->mtval = env->bins;
    }
    if (!(cs->exception_index & RISCV_EXCP_INT_FLAG)) {
        esp32p4_clic_exception_taken(cpu, prev_priv, prev_mil);
    }
    esp32p4_hp_priv_update(cpu);
    esp32p4_trace_trap(cpu, env->mepc, excp & RISCV_EXCP_INT_MASK, false,
                       env->mtval, prev_priv, cpu->trap_after_retire);
    cpu->trap_after_retire = false;
}

/*
 * A pending bus error that mexstatus.PBEXE enables raises its access
 * fault again, at the instruction about to execute. Otherwise the core
 * takes the interrupt its CLIC presents, if it may.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.clic] */
static bool esp32p4_hp_cpu_exec_interrupt(CPUState *cs, int request)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    CPURISCVState *env = &cpu->parent_obj.env;
    uint32_t pending = ESP32P4_MEXSTATUS_BUS_ERR | ESP32P4_MEXSTATUS_PBEXE;

    if (request & ESP32P4_CPU_INTERRUPT_BUS_ERROR) {
        cpu_reset_interrupt(cs, ESP32P4_CPU_INTERRUPT_BUS_ERROR);
        if ((cpu->mexstatus & (pending | ESP32P4_MEXSTATUS_NMFT)) ==
            pending) {
            if (cpu->bus_error_store) {
                cs->exception_index = RISCV_EXCP_STORE_AMO_ACCESS_FAULT;
                env->badaddr = cpu->sttval[0];
            } else {
                cs->exception_index = RISCV_EXCP_LOAD_ACCESS_FAULT;
                env->badaddr = cpu->ldtval[0];
            }
            env->excp_uw2 = 0;
            cpu->bus_error = true;
            cpu->trap_after_retire = true;
            esp32p4_hp_do_interrupt(cs);
            return true;
        }
    }
    if (request & ESP32P4_CPU_INTERRUPT_CLIC) {
        uint32_t sel = qatomic_read(&cpu->clic_sel);

        if (esp32p4_clic_takes(cpu, sel)) {
            int id = sel & ESP32P4_CLIC_SEL_ID_MASK;

            if (extract32(sel, ESP32P4_CLIC_SEL_PRIV_SHIFT, 2) == PRV_M) {
                esp32p4_hp_trap_entry(cpu, RISCV_EXCP_INT_FLAG | id);
            }
            esp32p4_clic_take(cpu, sel);
            return true;
        }
    }
    return esp32p4_tcg_ops.cpu_exec_interrupt(cs, request);
}

/*
 * A bus error, which the core takes as an access fault unless
 * HP_SYSTEM_CORE_ERR_RESP_DIS disables the error response of the bus the
 * access is on (esp32p4_cpu_bus()), when the access completes as if the
 * slave had answered it, a read giving the 0 the bus returns. It is
 * recorded with the faulting instruction's address and valid bit in ldpc
 * or stpc and the access address in ldtval or sttval, the newest entry
 * first. The core reports load and store bus errors asynchronously unless
 * mhint.SBE is set, so on hardware the fault may be taken a few
 * instructions after the access; here it is always taken at the access.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static void esp32p4_hp_do_transaction_failed(CPUState *cs, hwaddr physaddr,
                                             vaddr addr, unsigned size,
                                             MMUAccessType access_type,
                                             int mmu_idx, MemTxAttrs attrs,
                                             MemTxResult response,
                                             uintptr_t retaddr)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    uint32_t pc;
    int bus = esp32p4_cpu_bus(addr, access_type == MMU_INST_FETCH);

    esp32p4_cpu_bus_wait_check(cs, retaddr);
    if (qatomic_read(&cpu->err_resp_dis) & BIT(bus)) {
        return;
    }
    if (retaddr) {
        cpu_restore_state(cs, retaddr);
    }
    pc = (cpu->parent_obj.env.pc & ~1) | 1;
    if (access_type == MMU_DATA_LOAD) {
        cpu->ldpc[1] = cpu->ldpc[0];
        cpu->ldtval[1] = cpu->ldtval[0];
        cpu->ldpc[0] = pc;
        cpu->ldtval[0] = addr;
        cpu->bus_error_store = false;
    } else if (access_type == MMU_DATA_STORE) {
        for (int i = ARRAY_SIZE(cpu->stpc) - 1; i > 0; i--) {
            cpu->stpc[i] = cpu->stpc[i - 1];
            cpu->sttval[i] = cpu->sttval[i - 1];
        }
        cpu->stpc[0] = pc;
        cpu->sttval[0] = addr;
        cpu->bus_error_store = true;
    }
    cpu->bus_error = true;
    esp32p4_tcg_ops.do_transaction_failed(cs, physaddr, addr, size,
                                          access_type, mmu_idx, attrs,
                                          response, 0);
}

/*
 * The only load that must be aligned is LR.W, whose misaligned address
 * either core reports as cause 6, a store or AMO misaligned address (on
 * the LP core, the misaligned atomic instruction exception).
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static G_NORETURN void esp32p4_do_unaligned_access(CPUState *cs, vaddr addr,
                                                   MMUAccessType access_type,
                                                   int mmu_idx,
                                                   uintptr_t retaddr)
{
    if (access_type == MMU_DATA_LOAD) {
        access_type = MMU_DATA_STORE;
    }
    esp32p4_tcg_ops.do_unaligned_access(cs, addr, access_type, mmu_idx,
                                        retaddr);
}

/*
 * The hardware loop and PIE state the translation of a block depends on:
 * whether the loop instructions may be used, and whether a loop counts;
 * whether the PIE instructions may be used, and whether the PIE state is
 * already dirty; whether the debug assistant monitors SP; whether the
 * trace encoder traces the core; and whether the branch predictor runs.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.pie] */
/* [spec:nuos:req:emu.esp32p4.debug] */
static TCGTBCPUState esp32p4_hp_get_tb_cpu_state(CPUState *cs)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    TCGTBCPUState s = esp32p4_tcg_ops.get_tb_cpu_state(cs);

    if (esp32p4_hwlp_enabled(cpu)) {
        s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS, ESP32P4_HWLP_EN, 1);
        if (cpu->hwlp_count[0] || cpu->hwlp_count[1]) {
            s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS,
                                   ESP32P4_HWLP_ARMED, 1);
        }
    }
    switch (cpu->pie_state & ESP32P4_HWLP_STATE) {
    case ESP32P4_EXT_OFF:
        break;
    case ESP32P4_EXT_DIRTY:
        s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS, ESP32P4_PIE_DIRTY, 1);
        /* fall through */
    default:
        s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS, ESP32P4_PIE_EN, 1);
        break;
    }
    if (qatomic_read(&cpu->spmon_en)) {
        s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS, ESP32P4_SPMON, 1);
    }
    if (qatomic_read(&cpu->trace_en)) {
        s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS, ESP32P4_TRACE, 1);
    }
    if (esp32p4_counters_predicting(cpu)) {
        s.cs_base = FIELD_DP64(s.cs_base, EXT_TB_FLAGS, ESP32P4_BPRED, 1);
    }
    return s;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.boot] */
static void esp32p4_hp_reset_hold(ESP32P4CPU *cpu)
{
    /*
     * mcounteren reads 0x7 from reset on the chip, CY, TM and IR set,
     * where the TRM gives 0 (register 2.32): the boot ROM never writes it,
     * and hands an application that value.
     */
    cpu->parent_obj.env.mcounteren = COUNTEREN_CY | COUNTEREN_TM |
                                     COUNTEREN_IR;
    memset(cpu->hwlp_start, 0, sizeof(cpu->hwlp_start));
    memset(cpu->hwlp_end, 0, sizeof(cpu->hwlp_end));
    memset(cpu->hwlp_count, 0, sizeof(cpu->hwlp_count));
    cpu->hwlp_state = 0;
    cpu->pie_state = 0;
    cpu->ext_ill = 0;
    cpu->mexstatus = ESP32P4_MEXSTATUS_RESET;
    cpu->mhint = 0;
    cpu->mhcr = 0;
    cpu->fxcr_dqnan = false;
    esp32p4_hp_update_fp_status(cpu);
    cpu->gpio_oen = 0;
    cpu->gpio_out = 0;
    memset(cpu->ldpc, 0, sizeof(cpu->ldpc));
    memset(cpu->ldtval, 0, sizeof(cpu->ldtval));
    memset(cpu->stpc, 0, sizeof(cpu->stpc));
    memset(cpu->sttval, 0, sizeof(cpu->sttval));
    cpu->bus_error = false;
    cpu->bus_error_store = false;
    memset(cpu->mhpmevent, 0, sizeof(cpu->mhpmevent));
    esp32p4_triggers_reset(cpu);
    esp32p4_pie_reset(cpu);
    esp32p4_pmp_reset(cpu);
    cpu_reset_interrupt(CPU(cpu), ESP32P4_CPU_INTERRUPT_BUS_ERROR);
    esp32p4_clic_cpu_reset(cpu);
}

/*
 * The LP core's CSRs from reset (TRM 5.3.2), which QEMU's RISC-V reset
 * leaves as they were: mstatus, mie, mscratch, mepc and mtval clear; mtvec
 * at the reset boot address's vector table; the triggers. mip follows the
 * interrupt lines, which reset does not change.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_reset_hold(ESP32P4CPU *cpu)
{
    CPURISCVState *env = &cpu->parent_obj.env;

    env->mstatus &= ~(MSTATUS_MIE | MSTATUS_MPIE);
    env->mie = 0;
    env->mscratch = 0;
    env->mepc = 0;
    env->mtval = 0;
    env->mtvec = (env->resetvec & ~(target_ulong)0xff) | 1;
    esp32p4_lp_triggers_reset(cpu);
}
#endif

/*
 * The reset inputs are not the core's own state and survive its reset: a
 * core reset while its reset is held, or its clock stopped, stays halted.
 */
static void esp32p4_cpu_reset_hold(Object *obj, ResetType type)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);
    ESP32P4CPUClass *ecc = ESP32P4_CPU_GET_CLASS(obj);

    if (ecc->parent_phases.hold) {
        ecc->parent_phases.hold(obj, type);
    }
    CPU(obj)->halted = esp32p4_cpu_held(cpu);
    cpu->clock_halted = !cpu->reset && esp32p4_cpu_clock_off(cpu);
    qatomic_set(&cpu->bus_wait, false);
    cpu->bus_hold = false;
#ifdef CONFIG_TCG
    esp32p4_counters_reset(cpu);
    if (esp32p4_env_is_hp(&cpu->parent_obj.env)) {
        esp32p4_hp_reset_hold(cpu);
    } else if (esp32p4_env_is_lp(&cpu->parent_obj.env)) {
        esp32p4_lp_reset_hold(cpu);
    }
#endif
}

static void esp32p4_cpu_reset_exit(Object *obj, ResetType type)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);
    ESP32P4CPUClass *ecc = ESP32P4_CPU_GET_CLASS(obj);

    if (ecc->parent_phases.exit) {
        ecc->parent_phases.exit(obj, type);
    }
#ifdef CONFIG_TCG
    if (esp32p4_env_is_hp(&cpu->parent_obj.env)) {
        esp32p4_hp_gpio_update(cpu);
        cpu->m_mode_level = cpu->parent_obj.env.priv == PRV_M;
        qemu_set_irq(cpu->m_mode_irq, cpu->m_mode_level);
    }
#endif
}

#ifdef CONFIG_TCG
/*
 * The core's clock is about to change rate: count at the old one first;
 * then count, and execute, at the new one.
 */
static void esp32p4_cpu_clk_update(void *opaque, ClockEvent event)
{
    if (event == ClockPreUpdate) {
        esp32p4_counters_update(opaque);
    } else {
        esp32p4_counters_clock_changed(opaque);
    }
}
#endif

static void esp32p4_cpu_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_cpu_set_reset,
                            ESP32P4_CPU_RESET, 1);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_cpu_set_clock,
                            ESP32P4_CPU_CLOCK, 1);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_cpu_set_err_resp_dis,
                            ESP32P4_CPU_ERR_RESP_DIS, 3);
#ifdef CONFIG_TCG
    ESP32P4_CPU(obj)->clk = qdev_init_clock_in(DEVICE(obj), ESP32P4_CPU_CLK,
                                               esp32p4_cpu_clk_update, obj,
                                               ClockPreUpdate | ClockUpdate);
#endif
}

static bool esp32p4_cpu_get_in_reset(Object *obj, Error **errp)
{
    return ESP32P4_CPU(obj)->reset;
}

static bool esp32p4_cpu_get_clock_running(Object *obj, Error **errp)
{
    return !ESP32P4_CPU(obj)->clock_stopped;
}

/*
 * The core waits for an interrupt in WFI: halted, but neither held, nor
 * stopped by its clock, nor in debug mode.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static bool esp32p4_cpu_get_in_wfi(Object *obj, Error **errp)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);

    return CPU(cpu)->halted && !esp32p4_cpu_held(cpu) &&
           !cpu->clock_halted && !cpu->lp_debug_mode && !cpu->bus_wait;
}

#ifdef CONFIG_TCG
static bool esp32p4_hp_state_needed(void *opaque)
{
    return esp32p4_env_is_hp(&ESP32P4_CPU(opaque)->parent_obj.env);
}

static int esp32p4_hp_state_post_load(void *opaque, int version_id)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(opaque);

    esp32p4_hp_update_fp_status(cpu);
    esp32p4_triggers_rebuild(cpu);
    cpu->m_mode_level = cpu->parent_obj.env.priv == PRV_M;
    return 0;
}

/* [spec:nuos:req:emu.esp32p4.pie] */
static const VMStateDescription vmstate_esp32p4_pie = {
    .name = "esp32p4-cpu/pie",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = esp32p4_hp_state_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64_2DARRAY(pie.q, ESP32P4CPU, ESP32P4_PIE_QREGS, 2),
        VMSTATE_UINT64_ARRAY(pie.qacc, ESP32P4CPU, 8),
        VMSTATE_UINT64_ARRAY(pie.ua_state, ESP32P4CPU, 2),
        VMSTATE_UINT64(pie.xacc, ESP32P4CPU),
        VMSTATE_UINT32(pie.sar, ESP32P4CPU),
        VMSTATE_UINT32(pie.sar_byte, ESP32P4CPU),
        VMSTATE_UINT32(pie.fft_bit_width, ESP32P4CPU),
        VMSTATE_UINT32(pie.cfg, ESP32P4CPU),
        VMSTATE_UINT32(pie.perf, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_hp_state = {
    .name = "esp32p4-cpu/hp",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = esp32p4_hp_state_needed,
    .post_load = esp32p4_hp_state_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(hwlp_start, ESP32P4CPU, ESP32P4_HWLP_LOOPS),
        VMSTATE_UINT32_ARRAY(hwlp_end, ESP32P4CPU, ESP32P4_HWLP_LOOPS),
        VMSTATE_UINT32_ARRAY(hwlp_count, ESP32P4CPU, ESP32P4_HWLP_LOOPS),
        VMSTATE_UINT32(hwlp_state, ESP32P4CPU),
        VMSTATE_UINT32(pie_state, ESP32P4CPU),
        VMSTATE_UINT32(ext_ill, ESP32P4CPU),
        VMSTATE_UINT32(mexstatus, ESP32P4CPU),
        VMSTATE_UINT32(mhint, ESP32P4CPU),
        VMSTATE_UINT32(mhcr, ESP32P4CPU),
        VMSTATE_BOOL(fxcr_dqnan, ESP32P4CPU),
        VMSTATE_UINT32(gpio_oen, ESP32P4CPU),
        VMSTATE_UINT32(gpio_out, ESP32P4CPU),
        VMSTATE_UINT32(gpio_in, ESP32P4CPU),
        VMSTATE_UINT32_ARRAY(ldpc, ESP32P4CPU, 2),
        VMSTATE_UINT32_ARRAY(ldtval, ESP32P4CPU, 2),
        VMSTATE_UINT32_ARRAY(stpc, ESP32P4CPU, 3),
        VMSTATE_UINT32_ARRAY(sttval, ESP32P4CPU, 3),
        VMSTATE_BOOL(bus_error_store, ESP32P4CPU),
        VMSTATE_UINT32_ARRAY(mhpmevent, ESP32P4CPU, ESP32P4_HP_HPM_COUNTERS),
        VMSTATE_UINT32(tselect, ESP32P4CPU),
        VMSTATE_UINT32_ARRAY(tdata1, ESP32P4CPU, ESP32P4_TRIGGERS),
        VMSTATE_UINT32_ARRAY(tdata2, ESP32P4CPU, ESP32P4_TRIGGERS),
        VMSTATE_UINT32_ARRAY(tdata3, ESP32P4CPU, ESP32P4_TRIGGERS),
        VMSTATE_UINT32(tcontrol, ESP32P4CPU),
        VMSTATE_UINT32(mcontext, ESP32P4CPU),
        VMSTATE_UINT8_ARRAY(pmp_cfg, ESP32P4CPU, ESP32P4_PMP_ENTRIES),
        VMSTATE_UINT32_ARRAY(pmp_addr, ESP32P4CPU, ESP32P4_PMP_ENTRIES),
        VMSTATE_UINT32_ARRAY(pma_cfg, ESP32P4CPU, ESP32P4_PMA_ENTRIES),
        VMSTATE_UINT32_ARRAY(pma_addr, ESP32P4CPU, ESP32P4_PMA_ENTRIES),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_hp_clic = {
    .name = "esp32p4-cpu/clic",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = esp32p4_hp_state_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(clic_sel, ESP32P4CPU),
        VMSTATE_UINT32(mtvt, ESP32P4CPU),
        VMSTATE_UINT32(utvec, ESP32P4CPU),
        VMSTATE_UINT32(utvt, ESP32P4CPU),
        VMSTATE_UINT32(uepc, ESP32P4CPU),
        VMSTATE_UINT32(uscratch, ESP32P4CPU),
        VMSTATE_UINT32(ucause, ESP32P4CPU),
        VMSTATE_UINT8(mil, ESP32P4CPU),
        VMSTATE_UINT8(uil, ESP32P4CPU),
        VMSTATE_UINT8(mintthresh, ESP32P4CPU),
        VMSTATE_UINT8(uintthresh, ESP32P4CPU),
        VMSTATE_UINT8(mpil, ESP32P4CPU),
        VMSTATE_UINT8(upil, ESP32P4CPU),
        VMSTATE_BOOL(minhv, ESP32P4CPU),
        VMSTATE_BOOL(uinhv, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
};

static bool esp32p4_lp_state_needed(void *opaque)
{
    return object_get_class(OBJECT(opaque)) == esp32p4_lp_cpu_class;
}

static int esp32p4_lp_state_post_load(void *opaque, int version_id)
{
    esp32p4_lp_triggers_rebuild(ESP32P4_CPU(opaque));
    return 0;
}

static const VMStateDescription vmstate_esp32p4_lp_state = {
    .name = "esp32p4-cpu/lp",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = esp32p4_lp_state_needed,
    .post_load = esp32p4_lp_state_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(disabled, ESP32P4CPU),
        VMSTATE_UINT32(tselect, ESP32P4CPU),
        VMSTATE_UINT32_ARRAY(tdata1, ESP32P4CPU, ESP32P4_TRIGGERS),
        VMSTATE_UINT32_ARRAY(tdata2, ESP32P4CPU, ESP32P4_TRIGGERS),
        VMSTATE_BOOL(lp_debug_mode, ESP32P4CPU),
        VMSTATE_UINT32(lp_dpc, ESP32P4CPU),
        VMSTATE_UINT32(lp_dcsr, ESP32P4CPU),
        VMSTATE_TIMER_PTR(lp_exc_timer, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
};

/* The exact period the core executes at under icount, as it counts. */
static int esp32p4_counters_post_load(void *opaque, int version_id)
{
    ESP32P4CPU *cpu = opaque;
    ESP32P4ClkPeriod p = cpu->clk_period;
    uint64_t lo = p.num_lo, hi = p.num_hi;

    if (esp32p4_clk_period_running(p)) {
        icount_set_cpu_period_frac(CPU(cpu), divu128(&lo, &hi, p.den), p.den);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_counters = {
    .name = "esp32p4-cpu/counters",
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = esp32p4_counters_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4CPU),
        VMSTATE_UINT64(clk_period.num_hi, ESP32P4CPU),
        VMSTATE_UINT64(clk_period.num_lo, ESP32P4CPU),
        VMSTATE_UINT64(clk_period.den, ESP32P4CPU),
        VMSTATE_UINT64(cycles, ESP32P4CPU),
        VMSTATE_UINT64(cycles_synced, ESP32P4CPU),
        VMSTATE_INT64(cycles_ns, ESP32P4CPU),
        VMSTATE_UINT32(cycles_frac, ESP32P4CPU),
        VMSTATE_UINT64(insns, ESP32P4CPU),
        VMSTATE_UINT64_ARRAY(ctr_base, ESP32P4CPU, 2),
        VMSTATE_UINT64_ARRAY(ctr_ref, ESP32P4CPU, 2),
        VMSTATE_UINT64_ARRAY(ev, ESP32P4CPU, ESP32P4_EVENTS),
        VMSTATE_UINT64_ARRAY(hpm_base, ESP32P4CPU, ESP32P4_LP_HPM_COUNTERS),
        VMSTATE_UINT64_ARRAY(hpm_ref, ESP32P4CPU, ESP32P4_LP_HPM_COUNTERS),
        VMSTATE_UINT8_ARRAY(bht, ESP32P4CPU, ESP32P4_BHT_ENTRIES),
        VMSTATE_UINT32(bp_history, ESP32P4CPU),
        VMSTATE_UINT64(bus_wait_from, ESP32P4CPU),
        VMSTATE_UINT32(mcountinhibit, ESP32P4CPU),
        VMSTATE_BOOL(ctr_wfi, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
};
#endif

/* The inputs from the system registers and the debug assistant's state. */
static const VMStateDescription vmstate_esp32p4_debug = {
    .name = "esp32p4-cpu/debug",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(err_resp_dis, ESP32P4CPU),
        VMSTATE_BOOL(wakeup, ESP32P4CPU),
        VMSTATE_BOOL(bus_wait, ESP32P4CPU),
        VMSTATE_UINT32(exc_pc, ESP32P4CPU),
        VMSTATE_UINT32(assist_watch, ESP32P4CPU),
        VMSTATE_UINT32_ARRAY(assist_lo, ESP32P4CPU, ESP32P4_ASSIST_REGIONS),
        VMSTATE_UINT32_ARRAY(assist_hi, ESP32P4CPU, ESP32P4_ASSIST_REGIONS),
        VMSTATE_BOOL(spmon_en, ESP32P4CPU),
        VMSTATE_UINT32(spmon_min, ESP32P4CPU),
        VMSTATE_UINT32(spmon_max, ESP32P4CPU),
        VMSTATE_UINT32(spmon_out, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_cpu = {
    .name = "esp32p4-cpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj.parent_obj, ESP32P4CPU, 0,
                       vmstate_cpu_common, CPUState),
        VMSTATE_BOOL(reset, ESP32P4CPU),
        VMSTATE_BOOL(clock_stopped, ESP32P4CPU),
        VMSTATE_BOOL(clock_halted, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_esp32p4_debug,
#ifdef CONFIG_TCG
        &vmstate_esp32p4_hp_state,
        &vmstate_esp32p4_pie,
        &vmstate_esp32p4_hp_clic,
        &vmstate_esp32p4_lp_state,
        &vmstate_esp32p4_counters,
#endif
        NULL
    },
};

/*
 * The hooks that ask whether a halted core has work, wrapped so that a
 * held core never does. Both types share one copy of each table, which
 * the HP type extends in its own TCG table.
 */
static SysemuCPUOps esp32p4_sysemu_ops;

#ifdef CONFIG_TCG
static void (*esp32p4_riscv_restore_state_to_opc)(CPUState *cs,
                                                  const TranslationBlock *tb,
                                                  const uint64_t *data);

/*
 * The core unwinds from the middle of an instruction's code to its start,
 * as the instruction raises an exception or is to execute again: it has
 * not retired, though it counted itself as it started.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_restore_state_to_opc(CPUState *cs,
                                         const TranslationBlock *tb,
                                         const uint64_t *data)
{
    esp32p4_riscv_restore_state_to_opc(cs, tb, data);
    esp32p4_cpu_of(cs)->insns--;
}
#endif

static void esp32p4_cpu_class_init(ObjectClass *oc, const void *data)
{
    CPUClass *cc = CPU_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);
    ESP32P4CPUClass *ecc = ESP32P4_CPU_CLASS(oc);

    resettable_class_set_parent_phases(RESETTABLE_CLASS(oc), NULL,
                                       esp32p4_cpu_reset_hold,
                                       esp32p4_cpu_reset_exit,
                                       &ecc->parent_phases);
    qemu_cond_init(&esp32p4_cpu_stop_cond);

    esp32p4_sysemu_ops = *cc->sysemu_ops;
    esp32p4_sysemu_ops.has_work = esp32p4_cpu_has_work;
    cc->sysemu_ops = &esp32p4_sysemu_ops;
#ifdef CONFIG_TCG
    esp32p4_tcg_ops = *cc->tcg_ops;
    esp32p4_tcg_ops.cpu_exec_halt = esp32p4_cpu_has_work;
    esp32p4_riscv_restore_state_to_opc = esp32p4_tcg_ops.restore_state_to_opc;
    esp32p4_tcg_ops.restore_state_to_opc = esp32p4_restore_state_to_opc;
    cc->tcg_ops = &esp32p4_tcg_ops;
#endif

    dc->vmsd = &vmstate_esp32p4_cpu;

    object_class_property_add_bool(oc, "in-reset", esp32p4_cpu_get_in_reset,
                                   NULL);
    object_class_property_set_description(oc, "in-reset",
        "The core's reset input is held");
    object_class_property_add_bool(oc, "clock-running",
                                   esp32p4_cpu_get_clock_running, NULL);
    object_class_property_set_description(oc, "clock-running",
        "The core's clock enable is high");
    object_class_property_add_bool(oc, "in-wfi", esp32p4_cpu_get_in_wfi,
                                   NULL);
    object_class_property_set_description(oc, "in-wfi",
        "The core waits for an interrupt in WFI");
}

#ifdef CONFIG_TCG
static void esp32p4_hp_cpu_set_gpio_in(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;

    qatomic_set(&cpu->gpio_in, deposit32(qatomic_read(&cpu->gpio_in), n, 1,
                                         level != 0));
}

/*
 * The core has left the code it was executing: if it now waits in WFI,
 * its counters learn when it started to.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static void esp32p4_hp_cpu_exec_exit(CPUState *cs)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);

    if (esp32p4_tcg_ops.cpu_exec_exit) {
        esp32p4_tcg_ops.cpu_exec_exit(cs);
    }
    if (cs->halted && !cpu->ctr_wfi && !esp32p4_cpu_held(cpu) &&
        !cpu->clock_halted && !cpu->bus_wait) {
        BQL_LOCK_GUARD();
        esp32p4_counters_wfi(cpu, true);
    }
}

/* A halted core asks whether to carry on: one in WFI leaves it as it does. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static bool esp32p4_hp_cpu_exec_halt(CPUState *cs)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    bool wake = esp32p4_cpu_has_work(cs);

    if (wake && cpu->ctr_wfi) {
        BQL_LOCK_GUARD();
        esp32p4_counters_wfi(cpu, false);
    }
    return wake;
}
#endif

/*
 * The HP core's own trigger module replaces QEMU's, which the "debug"
 * property would otherwise enable.
 */
static void esp32p4_hp_cpu_init(Object *obj)
{
#ifdef CONFIG_TCG
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);
    DeviceState *dev = DEVICE(obj);

    cpu->parent_obj.cfg.debug = false;
    cpu->parent_obj.cfg.ext_sdtrig = false;
    qdev_init_gpio_out_named(dev, cpu->gpio_out_irq, ESP32P4_CPU_GPIO_OUT,
                             ESP32P4_CPU_GPIOS);
    qdev_init_gpio_out_named(dev, cpu->gpio_oe_irq, ESP32P4_CPU_GPIO_OE,
                             ESP32P4_CPU_GPIOS);
    qdev_init_gpio_in_named(dev, esp32p4_hp_cpu_set_gpio_in,
                            ESP32P4_CPU_GPIO_IN, ESP32P4_CPU_GPIOS);
    qdev_init_gpio_out_named(dev, &cpu->m_mode_irq, ESP32P4_CPU_M_MODE, 1);
#endif
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_cpu_set_wakeup,
                            ESP32P4_CPU_WAKEUP, 1);
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static void esp32p4_hp_cpu_class_init(ObjectClass *oc, const void *data)
{
#ifdef CONFIG_TCG
    CPUClass *cc = CPU_CLASS(oc);

    esp32p4_hp_tcg_ops = esp32p4_tcg_ops;
    esp32p4_hp_tcg_ops.get_tb_cpu_state = esp32p4_hp_get_tb_cpu_state;
    esp32p4_hp_tcg_ops.do_interrupt = esp32p4_hp_do_interrupt;
    esp32p4_hp_tcg_ops.cpu_exec_interrupt = esp32p4_hp_cpu_exec_interrupt;
    esp32p4_hp_tcg_ops.do_transaction_failed =
        esp32p4_hp_do_transaction_failed;
    esp32p4_hp_tcg_ops.do_unaligned_access = esp32p4_do_unaligned_access;
    esp32p4_hp_tcg_ops.tlb_fill = esp32p4_hp_tlb_fill;
    esp32p4_hp_tcg_ops.debug_excp_handler = esp32p4_debug_excp_handler;
    esp32p4_hp_tcg_ops.debug_check_breakpoint =
        esp32p4_debug_check_breakpoint;
    esp32p4_hp_tcg_ops.debug_check_watchpoint =
        esp32p4_debug_check_watchpoint;
    esp32p4_hp_tcg_ops.cpu_exec_exit = esp32p4_hp_cpu_exec_exit;
    esp32p4_hp_tcg_ops.cpu_exec_halt = esp32p4_hp_cpu_exec_halt;
    cc->tcg_ops = &esp32p4_hp_tcg_ops;
    esp32p4_csr_class_init();
#endif
}

#ifdef CONFIG_TCG
/*
 * A bus error on the LP core: an access fault, unless
 * LP_SYSTEM_LP_CORE_ERR_RESP_DIS disables the error response of the bus
 * the access is on: its IBUS and DBUS for LP ROM and LP SRAM
 * (0x50100000 to 0x5010FFFF), its AHB bus for the rest.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_lp_do_transaction_failed(CPUState *cs, hwaddr physaddr,
                                             vaddr addr, unsigned size,
                                             MMUAccessType access_type,
                                             int mmu_idx, MemTxAttrs attrs,
                                             MemTxResult response,
                                             uintptr_t retaddr)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    int bus = ESP32P4_CPU_BUS_AHB;

    esp32p4_cpu_bus_wait_check(cs, retaddr);
    if (addr >= 0x50100000 && addr < 0x50110000) {
        bus = access_type == MMU_INST_FETCH ? ESP32P4_CPU_BUS_IBUS
                                            : ESP32P4_CPU_BUS_DBUS;
    }
    if (qatomic_read(&cpu->err_resp_dis) & BIT(bus)) {
        return;
    }
    esp32p4_tcg_ops.do_transaction_failed(cs, physaddr, addr, size,
                                          access_type, mmu_idx, attrs,
                                          response, retaddr);
}

static void esp32p4_lp_exception_pulse(ESP32P4CPU *cpu)
{
    qemu_irq_pulse(cpu->lp_exception);
    qemu_irq_pulse(cpu->lp_etm_event);
}

static void esp32p4_lp_exc_timer(void *opaque)
{
    esp32p4_lp_exception_pulse(opaque);
}

/*
 * An exception on the LP core: LP_SYSTEM_LP_CPU_EXC_PC takes its PC, and
 * the PMU and the ETM learn of it, after the end of the code block if the
 * core is in one. An interrupt is not an exception and changes neither.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_do_interrupt(CPUState *cs)
{
    ESP32P4CPU *cpu = esp32p4_cpu_of(cs);
    int32_t excp = cs->exception_index;

    if (!(excp & RISCV_EXCP_INT_FLAG) && excp != RISCV_EXCP_SEMIHOST) {
        cpu->exc_pc = cpu->parent_obj.env.pc;
        if (esp32p4_cpu_mid_block()) {
            timer_mod(cpu->lp_exc_timer, 0);
        } else {
            esp32p4_lp_exception_pulse(cpu);
        }
    }
    esp32p4_tcg_ops.do_interrupt(cs);
}

/* The LP core's interrupt lines, mip's bits. */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_cpu_set_irq(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;
    uint64_t bit = BIT_ULL(n);

    if (ESP32P4_LP_IRQ_MASK & bit) {
        riscv_cpu_update_mip(&cpu->parent_obj.env, bit, level ? bit : 0);
    }
}

/* LP_CORE_DISABLE: the core stops while it is set. */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_cpu_set_disable(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;
    bool was_off = esp32p4_cpu_clock_off(cpu);

    if (cpu->disabled == !!level) {
        return;
    }
    esp32p4_counters_update(cpu);
    qatomic_set(&cpu->disabled, level != 0);
    esp32p4_cpu_clock_changed(cpu, was_off);
}
#endif

void esp32p4_lp_cpu_set_boot(ESP32P4CPU *cpu, ESP32P4LPBootFn fn,
                             void *opaque)
{
    cpu->lp_boot = fn;
    cpu->lp_boot_opaque = opaque;
}

/*
 * The LP core's own trigger module replaces QEMU's, and it has an
 * interrupt controller's lines, LP_CORE_DISABLE and its exception outputs.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_cpu_init(Object *obj)
{
#ifdef CONFIG_TCG
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);
    DeviceState *dev = DEVICE(obj);

    cpu->parent_obj.cfg.debug = false;
    cpu->parent_obj.cfg.ext_sdtrig = false;
    qdev_init_gpio_in_named(dev, esp32p4_lp_cpu_set_irq, ESP32P4_CPU_LP_IRQ,
                            ESP32P4_LP_IRQS);
    qdev_init_gpio_in_named(dev, esp32p4_lp_cpu_set_disable,
                            ESP32P4_CPU_LP_DISABLE, 1);
    qdev_init_gpio_out_named(dev, &cpu->lp_exception,
                             ESP32P4_CPU_LP_EXCEPTION, 1);
    qdev_init_gpio_out_named(dev, &cpu->lp_etm_event, ESP32P4_CPU_ETM_EVENT,
                             1);
    cpu->lp_exc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32p4_lp_exc_timer,
                                     cpu);
#endif
}

static DeviceRealize esp32p4_lp_parent_realize;

/*
 * The LP core completes misaligned loads and stores itself (TRM 5.7
 * counts an unaligned access as two), as QEMU's zicclsm has a core do.
 * QEMU ties zicclsm to privileged architecture 1.11 and drops it from a
 * core of 1.10 as it realizes it, so it is set once the core is realized.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_cpu_realize(DeviceState *dev, Error **errp)
{
    Error *err = NULL;

    esp32p4_lp_parent_realize(dev, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    RISCV_CPU(dev)->cfg.ext_zicclsm = true;
}

static void esp32p4_lp_cpu_finalize(Object *obj)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);

    if (cpu->lp_exc_timer) {
        timer_free(cpu->lp_exc_timer);
    }
}

/*
 * The LP core's TCG table: its accesses carry its identity to the access
 * permission managers, and its traps and triggers are its own.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_cpu_class_init(ObjectClass *oc, const void *data)
{
    esp32p4_lp_cpu_class = oc;
    device_class_set_parent_realize(DEVICE_CLASS(oc), esp32p4_lp_cpu_realize,
                                    &esp32p4_lp_parent_realize);
#ifdef CONFIG_TCG
    CPUClass *cc = CPU_CLASS(oc);

    esp32p4_lp_tcg_ops = esp32p4_tcg_ops;
    esp32p4_lp_tcg_ops.tlb_fill = esp32p4_lp_tlb_fill;
    esp32p4_lp_tcg_ops.do_transaction_failed =
        esp32p4_lp_do_transaction_failed;
    esp32p4_lp_tcg_ops.do_interrupt = esp32p4_lp_do_interrupt;
    esp32p4_lp_tcg_ops.do_unaligned_access = esp32p4_do_unaligned_access;
    esp32p4_lp_tcg_ops.debug_check_breakpoint =
        esp32p4_lp_debug_check_breakpoint;
    esp32p4_lp_tcg_ops.debug_excp_handler = esp32p4_lp_debug_excp_handler;
    cc->tcg_ops = &esp32p4_lp_tcg_ops;
    esp32p4_csr_class_init();
#endif
}

/* [spec:nuos:req:emu.esp32p4.machine] */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static const TypeInfo esp32p4_cpu_types[] = {
    {
        .name = TYPE_ESP32P4_CPU,
        .parent = TYPE_RISCV_VENDOR_CPU,
        .abstract = true,
        .instance_size = sizeof(ESP32P4CPU),
        .instance_init = esp32p4_cpu_init,
        .class_size = sizeof(ESP32P4CPUClass),
        .class_init = esp32p4_cpu_class_init,
    },
    {
        .name = TYPE_ESP32P4_HP_CPU,
        .parent = TYPE_ESP32P4_CPU,
        .instance_init = esp32p4_hp_cpu_init,
        .class_init = esp32p4_hp_cpu_class_init,
        .class_data = &(const RISCVCPUDef) {
            .misa_mxl_max = MXL_RV32,
            .misa_ext = RVI | RVM | RVA | RVF | RVC | RVB | RVU | RVX,
            .priv_spec = PRIV_VERSION_1_12_0,
            .vext_spec = RISCV_PROFILE_ATTR_UNUSED,
            .cfg.max_satp_mode = VM_1_10_MBARE,
            .cfg.ext_zicsr = true,
            .cfg.ext_zifencei = true,
            .cfg.ext_zicclsm = true,
            .cfg.ext_zcb = true,
            .cfg.ext_zcmp = true,
            .cfg.ext_zcmt = true,
            .cfg.zcmp_ra_highest = true,
            .cfg.zcmp_check_first = true,
            .cfg.ext_zba = true,
            .cfg.ext_zbb = true,
            .cfg.ext_zbc = true,
            .cfg.ext_zbs = true,
            .cfg.ext_xesploop = true,
            .cfg.pmu_mask = ESP32P4_HP_PMU_MASK,
            .cfg.mvendorid = ESP32P4_HP_MVENDORID,
            .cfg.marchid = ESP32P4_HP_MARCHID,
            .cfg.mimpid = ESP32P4_HP_MIMPID,
#ifdef CONFIG_TCG
            .custom_csrs = esp32p4_csr_list,
#endif
        },
    },
    {
        .name = TYPE_ESP32P4_LP_CPU,
        .parent = TYPE_ESP32P4_CPU,
        .instance_init = esp32p4_lp_cpu_init,
        .instance_finalize = esp32p4_lp_cpu_finalize,
        .class_init = esp32p4_lp_cpu_class_init,
        .class_data = &(const RISCVCPUDef) {
            .misa_mxl_max = MXL_RV32,
            .misa_ext = RVI | RVM | RVA | RVC,
            .priv_spec = PRIV_VERSION_1_10_0,
            .vext_spec = RISCV_PROFILE_ATTR_UNUSED,
            .cfg.max_satp_mode = VM_1_10_MBARE,
            .cfg.ext_zicsr = true,
            .cfg.ext_zifencei = true,
#ifdef CONFIG_TCG
            .custom_csrs = esp32p4_csr_list,
#endif
        },
    },
};

DEFINE_TYPES(esp32p4_cpu_types)
