/*
 * QEMU System Emulator
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/host-utils.h"
#include "hw/core/clock.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/cpus.h"
#include "system/qtest.h"
#include "qemu/main-loop.h"
#include "qemu/option.h"
#include "qemu/seqlock.h"
#include "system/replay.h"
#include "system/runstate.h"
#include "hw/core/cpu.h"
#include "exec/icount.h"
#include "system/cpu-timers-internal.h"
#include "tcg-accel-ops-icount.h"
#include "tcg-accel-ops-rr.h"

/*
 * ICOUNT: Instruction Counter
 *
 * this module is split off from cpu-timers because the icount part
 * is TCG-specific, and does not need to be built for other accels.
 */
static bool icount_sleep = true;
/* Arbitrarily pick 1MIPS as the minimum allowable speed.  */
#define MAX_ICOUNT_SHIFT 10

bool icount_align_option;

/* Do not count executed instructions */
ICountMode use_icount = ICOUNT_DISABLED;

/* The "quantum" option, in ns, or -1 when not given. */
int64_t icount_quantum = -1;

static void icount_enable_precise(void)
{
    /* Fixed conversion of insn to ns via "shift" option */
    use_icount = ICOUNT_PRECISE;
}

static void icount_enable_adaptive(void)
{
    /* Runtime adaptive algorithm to compute shift */
    use_icount = ICOUNT_ADAPTATIVE;
}

/*
 * The current number of executed instructions is based on what we
 * originally budgeted minus the current state of the decrementing
 * icount counters in extra/u16.low.
 */
static int64_t icount_get_executed(CPUState *cpu)
{
    return (cpu->icount_budget -
            (cpu->neg.icount_decr.u16.low + cpu->icount_extra));
}

/* @ns + @frac 2^-32 ns, plus @units 2^-32 ns (below 2^96). */
static void icount_time_add(int64_t *ns, uint32_t *frac, uint64_t lo,
                            uint64_t hi)
{
    uint64_t f = (uint64_t)*frac + (uint32_t)lo;

    *ns += (int64_t)((hi << 32) | (lo >> 32)) + (int64_t)(f >> 32);
    *frac = (uint32_t)f;
}

/*
 * The window of the CPUs with an icount clock (see icount_set_cpu_clock()
 * and tcg-accel-ops-icount.c). Each CPU taking part has run through its
 * own time, icount_own past the window's start: a period of its clock an
 * instruction. One gone idle stays where it stopped until virtual time
 * passes it, and then follows virtual time, as an idle core's does. Over
 * the window, virtual time is the mean of their own times, so that each
 * instruction of a CPU running alongside n - 1 others moves it on by 1/n
 * of the CPU's period, as they take turns, by the whole period while the
 * others are idle, and by nothing while all are idle (which the warp to
 * the next deadline then covers): the least x, past the window's start,
 * with
 *
 *     x = (sum of running own times + sum of idle max(own, x)) / n
 *
 * which is found by trying, in turn, the idle CPUs whose own time x has
 * passed (fewest first): each gives x as the mean of the others' own
 * times over the rest, and the right one is consistent with that.
 */
static int64_t icount_window_x(void)
{
    int64_t running = 0, rest = 0, *idle;
    int n = 0, k = 0;
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        n++;
    }
    idle = g_newa(int64_t, n);
    n = 0;
    CPU_FOREACH(cpu) {
        if (!cpu->icount_in_window) {
            continue;
        }
        n++;
        if (cpu->icount_window_idle) {
            int i = k++;

            while (i > 0 && idle[i - 1] > cpu->icount_own) {
                idle[i] = idle[i - 1];
                i--;
            }
            idle[i] = cpu->icount_own;
            rest += cpu->icount_own;
        } else {
            running += cpu->icount_own;
        }
    }
    if (n == 0) {
        return INT64_MIN;
    }
    for (int m = 0; m <= k; m++) {
        int64_t x;

        if (m > 0) {
            rest -= idle[m - 1];
        }
        if (m == n) {
            return idle[k - 1];
        }
        x = (running + rest) / (n - m);
        if ((m == 0 || idle[m - 1] <= x) && (m == k || x <= idle[m])) {
            return x;
        }
    }
    return INT64_MIN;
}

/*
 * The time of edge @k of a clock whose exact period is @period + @frac /
 * @den 2^-32 ns, in 2^-32 ns from time 0, as *@hi:*@lo, and in *@rem the
 * fraction of a unit beyond it, in 1/@den.
 */
static void icount_edge_time(uint64_t k, uint64_t period, uint64_t frac,
                             uint64_t den, uint64_t *lo, uint64_t *hi,
                             uint64_t *rem)
{
    mulu64(lo, hi, k, period);
    *rem = 0;
    if (frac) {
        uint64_t flo, fhi;

        mulu64(&flo, &fhi, k, frac);
        *rem = divu128(&flo, &fhi, den);
        *lo += flo;
        *hi += fhi + (*lo < flo);
    }
}

/*
 * The first edge of @cpu's icount clock at or after @units 2^-32 ns past
 * the window's start, in the same units. The clock's edges fall at whole
 * multiples of its exact period from time 0 (on the virtual clock, its
 * bias included), the edges devices timed on the clock count. @cpu's
 * remainder is set for that edge, so that its instructions, each taking
 * an exact period, go on from edge to edge.
 */
static int64_t icount_window_edge(CPUState *cpu, int64_t units)
{
    uint64_t period = qatomic_read(&cpu->icount_period);
    uint64_t frac = qatomic_read(&cpu->icount_period_frac);
    uint64_t den = qatomic_read(&cpu->icount_period_den);
    int64_t start_ns = timers_state.qemu_icount_bias + timers_state.window_ns;
    uint64_t tlo, thi, qlo, qhi, elo, ehi, rem, k;

    if (!period || start_ns < 0 || units < 0 || units > INT64_MAX / 2) {
        return units;
    }
    /* The time, in 2^-32 ns from time 0. */
    tlo = ((uint64_t)start_ns << 32) | timers_state.window_frac;
    thi = (uint64_t)start_ns >> 32;
    tlo += (uint64_t)units;
    thi += tlo < (uint64_t)units;
    /* The first edge of the whole period: edges of the exact one are later. */
    qlo = tlo;
    qhi = thi;
    if (divu128(&qlo, &qhi, period)) {
        qlo++;
        qhi += !qlo;
    }
    if (qhi) {
        return units;
    }
    k = qlo;
    while (k > 0) {
        icount_edge_time(k - 1, period, frac, den, &elo, &ehi, &rem);
        if (ehi < thi || (ehi == thi && elo < tlo)) {
            break;
        }
        k--;
    }
    icount_edge_time(k, period, frac, den, &elo, &ehi, &rem);
    cpu->icount_time_rem = rem;
    return units + (int64_t)(elo - tlo);
}

/* Virtual time past the window's start, in 2^-32 ns. */
static int64_t icount_window_elapsed(void)
{
    int64_t ns = timers_state.qemu_icount_time - timers_state.window_ns;

    if (ns >= INT32_MAX) {
        return INT64_MAX;
    }
    return (int64_t)((uint64_t)ns << 32) +
           (int64_t)timers_state.qemu_icount_time_frac -
           (int64_t)timers_state.window_frac;
}

/* Bring virtual time up to the window's CPUs' (icount_window_x()). */
static void icount_window_sync(void)
{
    int64_t x;

    if (!timers_state.window_cpus) {
        return;
    }
    x = icount_window_x();

    if (x > icount_window_elapsed()) {
        int64_t ns = timers_state.window_ns;
        uint32_t frac = timers_state.window_frac;

        icount_time_add(&ns, &frac, x, 0);
        qatomic_set(&timers_state.qemu_icount_time, ns);
        timers_state.qemu_icount_time_frac = frac;
    }
}

/*
 * Add the virtual time @executed instructions of @cpu's current run take.
 * A CPU running in the window of those with an icount clock adds its run's
 * instruction time, a period of its clock, to its own time, which moves
 * virtual time on as icount_window_x() says. Any other adds its run's
 * instruction time, or 2^shift ns, to virtual time, and moves the window
 * on as much. Fractions of a ns carry over, so a clock whose period is not
 * a whole number of ns keeps its rate over any number of instructions.
 */
static void icount_account_time(CPUState *cpu, int64_t executed)
{
    uint64_t insn_time = cpu->icount_insn_time;
    uint64_t lo, hi, carry = 0;

    if (cpu->icount_insn_frac && executed > 0) {
        /*
         * A period that is not a whole number of units: the fractions
         * run up in the CPU's remainder and carry whole units as they
         * make them.
         */
        mulu64(&lo, &hi, executed, cpu->icount_insn_frac);
        lo += cpu->icount_time_rem;
        hi += lo < cpu->icount_time_rem;
        cpu->icount_time_rem = divu128(&lo, &hi, cpu->icount_insn_den);
        carry = lo;
    }
    if (insn_time && cpu->icount_in_window && !cpu->icount_window_idle &&
        timers_state.window_cpus) {
        cpu->icount_own += executed * insn_time + carry;
        icount_window_sync();
        return;
    }
    if (!insn_time) {
        insn_time = (uint64_t)1 << (32 + timers_state.icount_time_shift);
    }
    mulu64(&lo, &hi, executed, insn_time);
    lo += carry;
    hi += lo < carry;
    icount_time_add(&timers_state.qemu_icount_time,
                    &timers_state.qemu_icount_time_frac, lo, hi);
    if (timers_state.window_cpus) {
        icount_time_add(&timers_state.window_ns, &timers_state.window_frac,
                        lo, hi);
    }
}

uint64_t icount_window_start(uint64_t units, int cpus)
{
    uint64_t moved = UINT64_MAX;
    int64_t elapsed;

    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    elapsed = icount_window_elapsed();
    if (timers_state.window_cpus && elapsed >= 0 && elapsed != INT64_MAX) {
        moved = elapsed;
    }
    timers_state.window_ns = timers_state.qemu_icount_time;
    timers_state.window_frac = timers_state.qemu_icount_time_frac;
    timers_state.window_units = units;
    timers_state.window_cpus = cpus;
    timers_state.window_running = cpus;
    timers_state.window_bias = timers_state.qemu_icount_bias;
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);
    return moved;
}

void icount_window_set_idle(CPUState *cpu, bool idle)
{
    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    if (idle) {
        cpu->icount_window_idle = true;
        timers_state.window_running--;
        icount_window_sync();
    } else {
        int64_t elapsed = icount_window_elapsed();

        /* Virtual time has passed it: it wakes on its clock's next edge. */
        if (elapsed > cpu->icount_own) {
            cpu->icount_own = icount_window_edge(cpu, elapsed);
        }
        cpu->icount_window_idle = false;
        timers_state.window_running++;
    }
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);
}

void icount_window_join(CPUState *cpu, uint64_t units)
{
    int64_t elapsed;

    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    elapsed = icount_window_elapsed();
    cpu->icount_own = icount_window_edge(cpu, elapsed);
    cpu->icount_in_window = true;
    cpu->icount_window_idle = false;
    timers_state.window_cpus++;
    timers_state.window_running++;
    if (elapsed >= 0 && (uint64_t)elapsed < timers_state.window_units &&
        units < timers_state.window_units - elapsed) {
        timers_state.window_units = elapsed + units;
    }
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);
}

void icount_window_align(CPUState *cpu)
{
    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    cpu->icount_own = icount_window_edge(cpu, cpu->icount_own);
    cpu->icount_realign = false;
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);
}

/*
 * Update the global shared timer_state.qemu_icount to take into
 * account executed instructions. This is done by the TCG vCPU
 * thread so the main-loop can see time has moved forward.
 */
static void icount_update_locked(CPUState *cpu)
{
    int64_t executed = icount_get_executed(cpu);
    cpu->icount_budget -= executed;

    qatomic_set(&timers_state.qemu_icount,
                timers_state.qemu_icount + executed);
    icount_account_time(cpu, executed);
}

/*
 * Update the global shared timer_state.qemu_icount to take into
 * account executed instructions. This is done by the TCG vCPU
 * thread so the main-loop can see time has moved forward.
 */
void icount_update(CPUState *cpu)
{
    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    icount_update_locked(cpu);
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);
}

static int64_t icount_get_raw_locked(void)
{
    CPUState *cpu = current_cpu;

    if (cpu && cpu->running) {
        if (!cpu->neg.can_do_io) {
            error_report("Bad icount read");
            exit(1);
        }
        /* Take into account what has run */
        icount_update_locked(cpu);
    }
    /* The read is protected by the seqlock, but needs atomic to avoid UB */
    return qatomic_read(&timers_state.qemu_icount);
}

static int64_t icount_get_locked(void)
{
    icount_get_raw_locked();
    return qatomic_read(&timers_state.qemu_icount_bias) +
           qatomic_read(&timers_state.qemu_icount_time);
}

int64_t icount_get_raw(void)
{
    int64_t icount;
    unsigned start;

    do {
        start = seqlock_read_begin(&timers_state.vm_clock_seqlock);
        icount = icount_get_raw_locked();
    } while (seqlock_read_retry(&timers_state.vm_clock_seqlock, start));

    return icount;
}

/*
 * Return the virtual CPU time, based on the instruction counter: with
 * CPUs executing at a clock's rate, the time their instructions have run
 * up to rounded up to the nanosecond, so that an instruction ending on a
 * clock edge ends at the edge's time as devices timed on the clock count
 * it (the first nanosecond at or after the edge).
 */
int64_t icount_get(void)
{
    int64_t icount;
    unsigned start;

    do {
        start = seqlock_read_begin(&timers_state.vm_clock_seqlock);
        icount = icount_get_locked() +
                 (timers_state.qemu_icount_time_frac != 0);
    } while (seqlock_read_retry(&timers_state.vm_clock_seqlock, start));

    return icount;
}

int64_t icount_get_precise(uint32_t *frac)
{
    int64_t icount;
    unsigned start;

    do {
        start = seqlock_read_begin(&timers_state.vm_clock_seqlock);
        icount = icount_get_locked();
        *frac = timers_state.qemu_icount_time_frac;
    } while (seqlock_read_retry(&timers_state.vm_clock_seqlock, start));

    return icount;
}

int64_t icount_to_ns(int64_t icount)
{
    return icount << qatomic_read(&timers_state.icount_time_shift);
}

/*
 * Correlation between real and virtual time is always going to be
 * fairly approximate, so ignore small variation.
 * When the guest is idle real and virtual time will be aligned in
 * the IO wait loop.
 */
#define ICOUNT_WOBBLE (NANOSECONDS_PER_SECOND / 10)

static void icount_adjust(void)
{
    int64_t cur_time;
    int64_t cur_icount;
    int64_t delta;

    /* If the VM is not running, then do nothing.  */
    if (!runstate_is_running()) {
        return;
    }

    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    cur_time = REPLAY_CLOCK_LOCKED(REPLAY_CLOCK_VIRTUAL_RT,
                                   cpu_get_clock_locked());
    cur_icount = icount_get_locked();

    delta = cur_icount - cur_time;
    /* FIXME: This is a very crude algorithm, somewhat prone to oscillation.  */
    if (delta > 0
        && timers_state.last_delta + ICOUNT_WOBBLE < delta * 2
        && timers_state.icount_time_shift > 0) {
        /* The guest is getting too far ahead.  Slow time down.  */
        qatomic_set(&timers_state.icount_time_shift,
                    timers_state.icount_time_shift - 1);
    }
    if (delta < 0
        && timers_state.last_delta - ICOUNT_WOBBLE > delta * 2
        && timers_state.icount_time_shift < MAX_ICOUNT_SHIFT) {
        /* The guest is getting too far behind.  Speed time up.  */
        qatomic_set(&timers_state.icount_time_shift,
                    timers_state.icount_time_shift + 1);
    }
    timers_state.last_delta = delta;
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);
}

static void icount_adjust_rt(void *opaque)
{
    timer_mod(timers_state.icount_rt_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT) + 1000);
    icount_adjust();
}

static void icount_adjust_vm(void *opaque)
{
    timer_mod(timers_state.icount_vm_timer,
                   qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                   NANOSECONDS_PER_SECOND / 10);
    icount_adjust();
}

int64_t icount_round(int64_t count)
{
    int shift = qatomic_read(&timers_state.icount_time_shift);
    return (count + (1 << shift) - 1) >> shift;
}

static void icount_warp_rt(void)
{
    unsigned seq;
    int64_t warp_start;

    /*
     * The icount_warp_timer is rescheduled soon after vm_clock_warp_start
     * changes from -1 to another value, so the race here is okay.
     */
    do {
        seq = seqlock_read_begin(&timers_state.vm_clock_seqlock);
        warp_start = timers_state.vm_clock_warp_start;
    } while (seqlock_read_retry(&timers_state.vm_clock_seqlock, seq));

    if (warp_start == -1) {
        return;
    }

    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    if (runstate_is_running()) {
        int64_t clock = REPLAY_CLOCK_LOCKED(REPLAY_CLOCK_VIRTUAL_RT,
                                            cpu_get_clock_locked());
        int64_t warp_delta;

        warp_delta = clock - timers_state.vm_clock_warp_start;
        if (icount_enabled() == ICOUNT_ADAPTATIVE) {
            /*
             * In adaptive mode, do not let QEMU_CLOCK_VIRTUAL run too far
             * ahead of real time (it might already be ahead so careful not
             * to go backwards).
             */
            int64_t cur_icount = icount_get_locked();
            int64_t delta = clock - cur_icount;

            if (delta < 0) {
                delta = 0;
            }
            warp_delta = MIN(warp_delta, delta);
        }
        qatomic_set(&timers_state.qemu_icount_bias,
                    timers_state.qemu_icount_bias + warp_delta);
    }
    timers_state.vm_clock_warp_start = -1;
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);

    if (qemu_clock_expired(QEMU_CLOCK_VIRTUAL)) {
        qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
    }
}

static void icount_timer_cb(void *opaque)
{
    /*
     * No need for a checkpoint because the timer already synchronizes
     * with CHECKPOINT_CLOCK_VIRTUAL_RT.
     */
    icount_warp_rt();
}

void icount_start_warp_timer(void)
{
    int64_t clock;
    int64_t deadline;

    assert(icount_enabled());

    /*
     * Nothing to do if the VM is stopped: QEMU_CLOCK_VIRTUAL timers
     * do not fire, so computing the deadline does not make sense.
     */
    if (!runstate_is_running()) {
        return;
    }

    /*
     * Nor while a reset, shutdown, suspend, debug stop or vmstop request
     * waits for the main loop to act on it: the vCPUs stopped for it, not
     * to wait for a timer, and it takes effect at the virtual time of its
     * request.
     */
    if (qemu_system_request_pending()) {
        return;
    }

    if (replay_mode != REPLAY_MODE_PLAY) {
        /*
         * Only once every vCPU is idle and the vCPU thread waits for an
         * event. The thread executes the vCPUs without the BQL, and adds
         * the instructions of each run to virtual time after the run, so a
         * vCPU that has halted in its run (in WFI, say) is idle before
         * virtual time includes them. A warp then, from a time that misses
         * them, would overshoot the deadline by them, and whatever reads
         * the time as the vCPU halts (a cycle counter stopping in WFI)
         * would see the warp; the main loop comes here at any host time,
         * for a monitor command or after the timers of the last warp. The
         * thread wakes the main loop as it starts to wait.
         */
        if (!all_cpu_threads_idle() ||
            (first_cpu && !rr_vcpu_thread_waiting())) {
            return;
        }

        if (qtest_enabled()) {
            /* When testing, qtest commands advance icount.  */
            return;
        }

        replay_checkpoint(CHECKPOINT_CLOCK_WARP_START);
    } else {
        /* warp clock deterministically in record/replay mode */
        if (!replay_checkpoint(CHECKPOINT_CLOCK_WARP_START)) {
            /*
             * vCPU is sleeping and warp can't be started.
             * It is probably a race condition: notification sent
             * to vCPU was processed in advance and vCPU went to sleep.
             * Therefore we have to wake it up for doing something.
             */
            if (replay_has_event()) {
                qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
            }
            return;
        }
    }

    /*
     * The earliest deadline of the QEMU_CLOCK_VIRTUAL timers, those with
     * QEMU_TIMER_ATTR_EXTERNAL among them: the timers of what lies outside
     * the machine, such as the user-mode network's (its router
     * advertisements) and the input queue's (send-key's held keys). With
     * every vCPU idle nothing else moves virtual time on to them, so left
     * out they would fire only at the machine's next timer, or never if it
     * has none.
     *
     * Record/replay leaves them out. Replay moves virtual time by the
     * recording's warps only by computing the same deadlines again, and
     * external timers are not part of what is recorded: the network's
     * advertisement interval, for one, is drawn at random afresh.
     */
    clock = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT);
    deadline = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                          replay_mode == REPLAY_MODE_NONE ?
                                          QEMU_TIMER_ATTR_ALL :
                                          ~QEMU_TIMER_ATTR_EXTERNAL);
    if (deadline < 0) {
        if (!icount_sleep) {
            warn_report_once("icount sleep disabled and no active timers");
        }
        return;
    }

    if (deadline > 0) {
        /*
         * Ensure QEMU_CLOCK_VIRTUAL proceeds even when the virtual CPU goes to
         * sleep.  Otherwise, the CPU might be waiting for a future timer
         * interrupt to wake it up, but the interrupt never comes because
         * the vCPU isn't running any insns and thus doesn't advance the
         * QEMU_CLOCK_VIRTUAL.
         */
        if (!icount_sleep) {
            /*
             * We never let VCPUs sleep in no sleep icount mode.
             * If there is a pending QEMU_CLOCK_VIRTUAL timer we just advance
             * to the next QEMU_CLOCK_VIRTUAL event and notify it.
             * It is useful when we want a deterministic execution time,
             * isolated from host latencies.
             */
            seqlock_write_lock(&timers_state.vm_clock_seqlock,
                               &timers_state.vm_clock_lock);
            qatomic_set(&timers_state.qemu_icount_bias,
                        timers_state.qemu_icount_bias + deadline);
            seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                                 &timers_state.vm_clock_lock);
            qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
        } else {
            /*
             * We do stop VCPUs and only advance QEMU_CLOCK_VIRTUAL after some
             * "real" time, (related to the time left until the next event) has
             * passed. The QEMU_CLOCK_VIRTUAL_RT clock will do this.
             * This avoids that the warps are visible externally; for example,
             * you will not be sending network packets continuously instead of
             * every 100ms.
             */
            seqlock_write_lock(&timers_state.vm_clock_seqlock,
                               &timers_state.vm_clock_lock);
            if (timers_state.vm_clock_warp_start == -1
                || timers_state.vm_clock_warp_start > clock) {
                timers_state.vm_clock_warp_start = clock;
            }
            seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                                 &timers_state.vm_clock_lock);
            timer_mod_anticipate(timers_state.icount_warp_timer,
                                 clock + deadline);
        }
    } else if (deadline == 0) {
        qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
    }
}

void icount_account_warp_timer(void)
{
    if (!icount_sleep) {
        return;
    }

    /*
     * Nothing to do if the VM is stopped: QEMU_CLOCK_VIRTUAL timers
     * do not fire, so computing the deadline does not make sense.
     */
    if (!runstate_is_running()) {
        return;
    }

    replay_async_events();

    /* warp clock deterministically in record/replay mode */
    if (!replay_checkpoint(CHECKPOINT_CLOCK_WARP_ACCOUNT)) {
        return;
    }

    timer_del(timers_state.icount_warp_timer);
    icount_warp_rt();
}

bool icount_configure(QemuOpts *opts, Error **errp)
{
    const char *option = qemu_opt_get(opts, "shift");
    bool sleep = qemu_opt_get_bool(opts, "sleep", true);
    bool align = qemu_opt_get_bool(opts, "align", false);
    long time_shift = -1;

    if (!option) {
        if (qemu_opt_get(opts, "align") != NULL) {
            error_setg(errp, "Please specify shift option when using align");
            return false;
        }
        if (qemu_opt_get(opts, "quantum") != NULL) {
            error_setg(errp, "Please specify shift option when using quantum");
            return false;
        }
        return true;
    }

    if (qemu_opt_get(opts, "quantum") != NULL) {
        uint64_t quantum = qemu_opt_get_number(opts, "quantum", 0);

        if (quantum > INT32_MAX) {
            error_setg(errp, "icount: Invalid quantum value");
            return false;
        }
        icount_quantum = quantum;
    }

    if (align && !sleep) {
        error_setg(errp, "align=on and sleep=off are incompatible");
        return false;
    }

    if (strcmp(option, "auto") != 0) {
        if (qemu_strtol(option, NULL, 0, &time_shift) < 0
            || time_shift < 0 || time_shift > MAX_ICOUNT_SHIFT) {
            error_setg(errp, "icount: Invalid shift value");
            return false;
        }
    } else if (icount_align_option) {
        error_setg(errp, "shift=auto and align=on are incompatible");
        return false;
    } else if (!icount_sleep) {
        error_setg(errp, "shift=auto and sleep=off are incompatible");
        return false;
    }

    icount_sleep = sleep;
    if (icount_sleep) {
        timers_state.icount_warp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL_RT,
                                         icount_timer_cb, NULL);
    }

    icount_align_option = align;

    if (time_shift >= 0) {
        timers_state.icount_time_shift = time_shift;
        icount_enable_precise();
        return true;
    }

    icount_enable_adaptive();

    /*
     * 125MIPS seems a reasonable initial guess at the guest speed.
     * It will be corrected fairly quickly anyway.
     */
    timers_state.icount_time_shift = 3;

    /*
     * Have both realtime and virtual time triggers for speed adjustment.
     * The realtime trigger catches emulated time passing too slowly,
     * the virtual time trigger catches emulated time passing too fast.
     * Realtime triggers occur even when idle, so use them less frequently
     * than VM triggers.
     */
    timers_state.vm_clock_warp_start = -1;
    timers_state.icount_rt_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL_RT,
                                   icount_adjust_rt, NULL);
    timer_mod(timers_state.icount_rt_timer,
                   qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT) + 1000);
    timers_state.icount_vm_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        icount_adjust_vm, NULL);
    timer_mod(timers_state.icount_vm_timer,
                   qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                   NANOSECONDS_PER_SECOND / 10);
    return true;
}

/*
 * The CPU's icount clock changed: later runs of the CPU take its new
 * period per instruction. A CPU changing its own clock (a write to a clock
 * control register) leaves its run at the end of the access, so the rest
 * of the run, and its budget to the next timer deadline, are not counted
 * at the old rate. A stopped clock leaves the last period: stopping the
 * clock that executes the CPU is the machine's to model (by halting it).
 */
static void icount_cpu_clock_update(void *opaque, ClockEvent event)
{
    CPUState *cpu = opaque;
    uint64_t period = clock_get(cpu->icount_clock);

    if (period) {
        qatomic_set(&cpu->icount_period, period);
        cpu->icount_realign = true;
        if (icount_enabled() && cpu == current_cpu) {
            cpu_exit(cpu);
        }
    }
}

void icount_set_cpu_period_frac(CPUState *cpu, uint64_t frac, uint64_t den)
{
    assert(den && frac < den);
    qatomic_set(&cpu->icount_period_den, den);
    qatomic_set(&cpu->icount_period_frac, frac);
    cpu->icount_time_rem = 0;
    cpu->icount_realign = true;
}

void icount_set_cpu_clock(CPUState *cpu, Clock *clk)
{
    assert(!cpu->icount_clock);
    cpu->icount_clock = clock_new(OBJECT(cpu), "icount-clock");
    clock_set_callback(cpu->icount_clock, icount_cpu_clock_update, cpu,
                       ClockUpdate);
    clock_set_source(cpu->icount_clock, clk);
    qatomic_set(&cpu->icount_period, clock_get(cpu->icount_clock));
    cpu->icount_period_den = 1;
}

void icount_stall_take(CPUState *cpu)
{
    int64_t n = MIN(cpu->icount_stall,
                    cpu->icount_extra + cpu->neg.icount_decr.u16.low);
    int64_t extra = MIN(n, cpu->icount_extra);

    cpu->icount_extra -= extra;
    cpu->neg.icount_decr.u16.low -= n - extra;
    cpu->icount_stall -= n;
}

void icount_stall(CPUState *cpu, int64_t insns)
{
    if (!icount_enabled() || insns <= 0) {
        return;
    }
    cpu->icount_stall += insns;
    icount_stall_take(cpu);
}

void icount_notify_exit(void)
{
    assert(icount_enabled());

    if (current_cpu) {
        qemu_cpu_kick(current_cpu);
        qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
    }
}
