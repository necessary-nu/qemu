/*
 * Espressif ESP32-P4 HP and LP core counters
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.5
 * (the HP core's CSRs), 5.3 and 5.7 (the LP core's CSRs and performance
 * counters); ESP32-C6 Technical Reference Manual v1.2, register 1.22
 * (mpcer).
 *
 * Each core counts for itself:
 *
 *   - mcycle and mcycleh (and, on an HP core, their user-mode mirrors
 *     cycle and cycleh) count the cycles of the core's clock
 *     (ESP32P4_CPU_CLK) at its current frequency, while it runs: not while
 *     the core is held in reset, nor while its clock enable is low (a
 *     stalled HP core, a disabled LP core), nor while an HP core waits in
 *     WFI. The P4's TRM says nothing of the cycle counter in WFI; the
 *     ESP32-C6's, of the HP core before this one, says its cycle count
 *     does not increment during WFI, with the core's clock in WFI
 *     (cpu_waiti_clk) forced on from reset by a CPU_WAITI_CONF register
 *     the P4's HP_SYSTEM repeats bit for bit: the count stops with the
 *     core rather than with that clock, which therefore changes nothing
 *     software sees. The LP core's mcycle, of which neither TRM says
 *     more than that it counts clock cycles, counts on in WFI.
 *   - minstret and minstreth (instret and instreth) count the
 *     instructions the core retires: not one that raises an exception.
 *   - mcountinhibit stops a counter where it is and restarts it from
 *     there. TM stops nothing: the time the HP core's time CSR reads is
 *     the CLINT's.
 *   - The event counters, mhpmcounter8, 9 and 13 on an HP core
 *     (conditional branch mispredictions, conditional branches and stores,
 *     each counting as mhpmevent8, 9 and 13 select) and mhpmcounter3 to
 *     12 on the LP core (40 bits each), hold what is written: the events
 *     they count are of the cores' pipelines, which are not modelled. That
 *     is logged as unimplemented.
 *
 * A read of a counter gives its value before the reading instruction
 * retires; a write takes effect after the writing instruction, which
 * the next one reads. A counter's reset value is 0.
 *
 * The cycles of a clock over an interval are the whole periods of it gone
 * by, at the period of its Clock, as every block of the chip counts them
 * (see esp32p4_clk_period.h); a clock that starts, or changes rate,
 * starts its first cycle there. Virtual time is taken to the fraction of a
 * nanosecond icount leaves when CPUs execute at their clocks' rate
 * (icount_get_precise()): under icount, a core executes an instruction a
 * cycle of its clock, so that a core running alone counts one cycle for
 * each instruction it executes, at any frequency. Cores running together
 * take turns of at most a quantum of virtual time (icount_set_cpu_clock()),
 * through which each one's mcycle follows virtual time, as the chip's
 * timers do, and so its own instructions to within that quantum. Without
 * icount virtual time follows the host's clock, as do the cycles.
 *
 * The instructions are counted by the code QEMU translates for the core,
 * which adds one to the core's count as each instruction starts. One that
 * then raises an exception takes it back: through
 * esp32p4_restore_state_to_opc() when it unwinds from the middle of its
 * code, and in the code generate_exception() emits for one that raises
 * its exception there.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "exec/icount.h"
#include "hw/core/clock.h"
#include "target/riscv/esp32p4_cpu.h"

/* The counters mcycle and minstret, by their place in ctr_base[]. */
enum {
    CTR_CYCLE,
    CTR_INSTRET,
};

static const uint32_t ctr_inhibit[] = {
    [CTR_CYCLE] = COUNTEREN_CY,
    [CTR_INSTRET] = COUNTEREN_IR,
};

/* mcountinhibit's bits on an HP core: CY, TM, IR, HPM8, HPM9, HPM13. */
#define HP_COUNTINHIBIT_MASK    (COUNTEREN_CY | COUNTEREN_TM | \
                                 COUNTEREN_IR | BIT(8) | BIT(9) | BIT(13))
/* On the LP core: CY, IR and HPM3 to 12. */
#define LP_COUNTINHIBIT_MASK    (COUNTEREN_CY | COUNTEREN_IR | \
                                 MAKE_64BIT_MASK(ESP32P4_LP_HPM_FIRST, \
                                                 ESP32P4_LP_HPM_COUNTERS))
#define LP_HPM_BITS             40

/* The HP core's event counters, in the order of hpm[]. */
static const int hp_hpm_counters[ESP32P4_HP_HPM_COUNTERS] = { 8, 9, 13 };

static const char *const lp_hpm_events[ESP32P4_LP_HPM_COUNTERS] = {
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

/* A virtual time: nanoseconds and the fraction of one, in 2^-32 ns. */
typedef struct CtrTime {
    int64_t ns;
    uint32_t frac;
} CtrTime;

static CtrTime ctr_now(void)
{
    CtrTime t = { 0, 0 };

    if (icount_enabled()) {
        t.ns = icount_get_precise(&t.frac);
    } else {
        t.ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    return t;
}

static bool ctr_before(CtrTime a, CtrTime b)
{
    return a.ns < b.ns || (a.ns == b.ns && a.frac < b.frac);
}

/* The whole periods of @period units from @from to @to, @to later. */
static uint64_t ctr_periods(CtrTime from, CtrTime to, uint64_t period)
{
    uint64_t ns = to.ns - from.ns;
    uint64_t hi = ns >> 32, lo = ns << 32;

    if (to.frac >= from.frac) {
        uint64_t d = to.frac - from.frac;

        lo += d;
        hi += lo < d;
    } else {
        uint64_t d = from.frac - to.frac;

        hi -= lo < d;
        lo -= d;
    }
    divu128(&lo, &hi, period);
    return hi ? UINT64_MAX : lo;
}

/* @t plus @n periods of @period units. */
static CtrTime ctr_advance(CtrTime t, uint64_t n, uint64_t period)
{
    uint64_t lo, hi, frac;

    mulu64(&lo, &hi, n, period);
    frac = (uint64_t)t.frac + (uint32_t)lo;
    t.ns += (int64_t)((hi << 32) | (lo >> 32)) + (int64_t)(frac >> 32);
    t.frac = (uint32_t)frac;
    return t;
}

static bool ctr_is_hp(ESP32P4CPU *cpu)
{
    return esp32p4_env_is_hp(&cpu->parent_obj.env);
}

/*
 * mcycle counts: the core is out of reset, its clock enabled, and, on an
 * HP core, not waiting in WFI.
 */
static bool ctr_clock_runs(ESP32P4CPU *cpu)
{
    return cpu->clk && clock_get(cpu->clk) && !qatomic_read(&cpu->reset) &&
           !qatomic_read(&cpu->clock_stopped) &&
           !qatomic_read(&cpu->disabled) && !cpu->ctr_wfi;
}

/*
 * Count the cycles of the core's clock up to @now: the whole periods
 * since the end of the last cycle counted, while mcycle counts.
 * cycles_ns and cycles_frac stay at the end of the last whole cycle
 * counted while it counts, and follow @now while it does not.
 */
static void ctr_cycles_sync(ESP32P4CPU *cpu, CtrTime now)
{
    CtrTime from = { cpu->cycles_ns, cpu->cycles_frac };
    uint64_t period, n;

    if (!ctr_before(from, now)) {
        if (ctr_before(now, from)) {
            cpu->cycles_ns = now.ns;
            cpu->cycles_frac = now.frac;
        }
        return;
    }
    if (!ctr_clock_runs(cpu)) {
        cpu->cycles_ns = now.ns;
        cpu->cycles_frac = now.frac;
        return;
    }
    period = clock_get(cpu->clk);
    n = ctr_periods(from, now, period);
    cpu->cycles += n;
    from = ctr_advance(from, n, period);
    cpu->cycles_ns = from.ns;
    cpu->cycles_frac = from.frac;
}

/* The count a counter counts: cycles, or instructions started. */
static uint64_t ctr_raw(ESP32P4CPU *cpu, int c)
{
    if (c == CTR_CYCLE) {
        ctr_cycles_sync(cpu, ctr_now());
        return cpu->cycles;
    }
    return cpu->insns;
}

static bool ctr_counting(ESP32P4CPU *cpu, int c)
{
    return !(cpu->mcountinhibit & ctr_inhibit[c]);
}

/*
 * A counter's value once the instruction accessing it completes; that
 * instruction is among the instructions started.
 */
static uint64_t ctr_value(ESP32P4CPU *cpu, int c)
{
    if (!ctr_counting(cpu, c)) {
        return cpu->ctr_base[c];
    }
    return cpu->ctr_base[c] + ctr_raw(cpu, c) - cpu->ctr_ref[c];
}

/*
 * A counter as the accessing instruction reads it: minstret without that
 * instruction, which has not retired, unless a debugger reads it.
 */
static uint64_t ctr_read(CPURISCVState *env, ESP32P4CPU *cpu, int c)
{
    uint64_t v = ctr_value(cpu, c);

    if (c == CTR_INSTRET && ctr_counting(cpu, c) && !env->debugger) {
        v--;
    }
    return v;
}

/* A counter written: what the next instruction reads. */
static void ctr_set(ESP32P4CPU *cpu, int c, uint64_t v)
{
    cpu->ctr_base[c] = v;
    cpu->ctr_ref[c] = ctr_raw(cpu, c);
}

/*
 * mcountinhibit: a counter stops at its value after the writing
 * instruction, and restarts from where it stopped with the instruction
 * after it.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void ctr_set_countinhibit(ESP32P4CPU *cpu, uint32_t val)
{
    uint32_t mask = ctr_is_hp(cpu) ? HP_COUNTINHIBIT_MASK
                                   : LP_COUNTINHIBIT_MASK;
    uint64_t now[2];

    val &= mask;
    for (int c = 0; c < ARRAY_SIZE(now); c++) {
        now[c] = ctr_value(cpu, c);
    }
    if ((val & COUNTEREN_TM) && !(cpu->mcountinhibit & COUNTEREN_TM)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4: mcountinhibit.TM does not stop "
                      "the CLINT time the time CSR reads\n");
    }
    cpu->mcountinhibit = val;
    for (int c = 0; c < ARRAY_SIZE(now); c++) {
        ctr_set(cpu, c, now[c]);
    }
}

/*
 * The event counter a CSR addresses, as its place in hpm[], or -1; and
 * the counter's number.
 */
static int ctr_hpm(ESP32P4CPU *cpu, int csrno, int *counter)
{
    int n;

    switch (csrno) {
    case CSR_MHPMCOUNTER3 ... CSR_MHPMCOUNTER31:
        n = csrno - CSR_MCYCLE;
        break;
    case CSR_MHPMCOUNTER3H ... CSR_MHPMCOUNTER31H:
        n = csrno - CSR_MCYCLEH;
        break;
    case CSR_HPMCOUNTER3 ... CSR_HPMCOUNTER31:
        n = csrno - CSR_CYCLE;
        break;
    case CSR_HPMCOUNTER3H ... CSR_HPMCOUNTER31H:
        n = csrno - CSR_CYCLEH;
        break;
    default:
        return -1;
    }
    *counter = n;
    if (!ctr_is_hp(cpu)) {
        n -= ESP32P4_LP_HPM_FIRST;
        return n >= 0 && n < ESP32P4_LP_HPM_COUNTERS ? n : -1;
    }
    for (int i = 0; i < ESP32P4_HP_HPM_COUNTERS; i++) {
        if (hp_hpm_counters[i] == n) {
            return i;
        }
    }
    return -1;
}

/* The LP core's event counters do not count their events. */
static void ctr_hpm_unimp(ESP32P4CPU *cpu, int counter)
{
    static bool logged[ESP32P4_LP_HPM_COUNTERS];
    int n = counter - ESP32P4_LP_HPM_FIRST;

    if (!ctr_is_hp(cpu) && !(cpu->mcountinhibit & BIT(counter)) &&
        !logged[n]) {
        logged[n] = true;
        qemu_log_mask(LOG_UNIMP, "esp32p4: the LP core's mhpmcounter%d does "
                      "not count %s\n", counter, lp_hpm_events[n]);
    }
}

static bool ctr_csr_high(int csrno)
{
    return (csrno >= CSR_MCYCLEH && csrno <= CSR_MHPMCOUNTER31H) ||
           (csrno >= CSR_CYCLEH && csrno <= CSR_HPMCOUNTER31H);
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
RISCVException esp32p4_counters_read(CPURISCVState *env, int csrno,
                                     target_ulong *val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    bool high = ctr_csr_high(csrno);
    uint64_t v;
    int n, counter;

    BQL_LOCK_GUARD();
    switch (csrno) {
    case CSR_MCOUNTINHIBIT:
        *val = cpu->mcountinhibit;
        return RISCV_EXCP_NONE;
    case CSR_MCYCLE:
    case CSR_MCYCLEH:
    case CSR_CYCLE:
    case CSR_CYCLEH:
        v = ctr_read(env, cpu, CTR_CYCLE);
        break;
    case CSR_MINSTRET:
    case CSR_MINSTRETH:
    case CSR_INSTRET:
    case CSR_INSTRETH:
        v = ctr_read(env, cpu, CTR_INSTRET);
        break;
    default:
        n = ctr_hpm(cpu, csrno, &counter);
        if (n < 0) {
            return RISCV_EXCP_ILLEGAL_INST;
        }
        ctr_hpm_unimp(cpu, counter);
        v = cpu->hpm[n];
        break;
    }
    *val = high ? v >> 32 : (uint32_t)v;
    return RISCV_EXCP_NONE;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
RISCVException esp32p4_counters_write(CPURISCVState *env, int csrno,
                                      target_ulong val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    bool high = ctr_csr_high(csrno);
    uint64_t v;
    int c, n, counter;

    BQL_LOCK_GUARD();
    switch (csrno) {
    case CSR_MCOUNTINHIBIT:
        ctr_set_countinhibit(cpu, val);
        return RISCV_EXCP_NONE;
    case CSR_MCYCLE:
    case CSR_MCYCLEH:
        c = CTR_CYCLE;
        break;
    case CSR_MINSTRET:
    case CSR_MINSTRETH:
        c = CTR_INSTRET;
        break;
    default:
        n = ctr_hpm(cpu, csrno, &counter);
        if (n < 0) {
            return RISCV_EXCP_ILLEGAL_INST;
        }
        ctr_hpm_unimp(cpu, counter);
        v = deposit64(cpu->hpm[n], high ? 32 : 0, 32, val);
        if (!ctr_is_hp(cpu)) {
            v = extract64(v, 0, LP_HPM_BITS);
        }
        cpu->hpm[n] = v;
        return RISCV_EXCP_NONE;
    }
    v = deposit64(ctr_value(cpu, c), high ? 32 : 0, 32, val);
    ctr_set(cpu, c, v);
    return RISCV_EXCP_NONE;
}

/*
 * The core's clock is about to start, stop or change rate: count the
 * cycles so far, and start the next cycle afresh.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
void esp32p4_counters_update(ESP32P4CPU *cpu)
{
    CtrTime now = ctr_now();

    ctr_cycles_sync(cpu, now);
    cpu->cycles_ns = now.ns;
    cpu->cycles_frac = now.frac;
}

/*
 * An HP core enters WFI, after the WFI's own cycle, or leaves it: mcycle
 * stops, or counts again from the next cycle.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_counters_wfi(ESP32P4CPU *cpu, bool wfi)
{
    CtrTime now = ctr_now();

    ctr_cycles_sync(cpu, now);
    cpu->ctr_wfi = wfi;
    cpu->cycles_ns = now.ns;
    cpu->cycles_frac = now.frac;
}

/* The counters' reset values: 0, all counting. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
void esp32p4_counters_reset(ESP32P4CPU *cpu)
{
    CtrTime now = ctr_now();

    cpu->cycles = 0;
    cpu->cycles_ns = now.ns;
    cpu->cycles_frac = now.frac;
    cpu->insns = 0;
    memset(cpu->ctr_base, 0, sizeof(cpu->ctr_base));
    memset(cpu->ctr_ref, 0, sizeof(cpu->ctr_ref));
    memset(cpu->hpm, 0, sizeof(cpu->hpm));
    cpu->mcountinhibit = 0;
    cpu->ctr_wfi = false;
}
