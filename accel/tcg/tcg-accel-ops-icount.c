/*
 * QEMU TCG Single Threaded vCPUs implementation using instruction counting
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
 * Copyright (c) 2014 Red Hat Inc.
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
#include "system/replay.h"
#include "exec/icount.h"
#include "qemu/host-utils.h"
#include "qemu/main-loop.h"
#include "qemu/guest-random.h"
#include "hw/core/cpu.h"
#include "system/cpus.h"
#include "qemu/seqlock.h"
#include "system/cpu-timers-internal.h"

#include "tcg-accel-ops.h"
#include "tcg-accel-ops-icount.h"
#include "tcg-accel-ops-rr.h"

/* The quantum of CPUs with an icount clock, unless the option sets one. */
#define ICOUNT_CLOCKED_QUANTUM_NS 1000

/* Virtual time, in ns, to the next timer deadline. */
static int64_t icount_get_deadline(void)
{
    int64_t deadline;

    /*
     * Include all the timers, because they may need an attention.
     * Too long CPU execution may create unnecessary delay in UI.
     */
    deadline = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                          QEMU_TIMER_ATTR_ALL);
    /* Check realtime timers, because they help with input processing */
    deadline = qemu_soonest_timeout(deadline,
            qemu_clock_deadline_ns_all(QEMU_CLOCK_REALTIME,
                                       QEMU_TIMER_ATTR_ALL));

    /*
     * Maintain prior (possibly buggy) behaviour where if no deadline
     * was set (as there is no QEMU_CLOCK_VIRTUAL timer) or it is more than
     * INT32_MAX nanoseconds ahead, we still use INT32_MAX
     * nanoseconds.
     */
    if ((deadline < 0) || (deadline > INT32_MAX)) {
        deadline = INT32_MAX;
    }
    return deadline;
}

static int64_t icount_get_limit(void)
{
    if (replay_mode != REPLAY_MODE_PLAY) {
        return icount_round(icount_get_deadline());
    } else {
        return replay_get_instructions();
    }
}

/*
 * How far virtual time is from the next deadline, in 2^-32 ns: from the
 * time the CPUs' instructions have run up to, to the first time that
 * rounds up to the deadline's nanosecond (icount_get()), so that a timer
 * due at a clock edge's time fires at the end of the instruction that
 * ends on the edge. 0 if the deadline has come.
 */
static uint64_t icount_deadline_units(void)
{
    uint64_t deadline = (uint64_t)icount_get_deadline() << 32;
    uint64_t frac = timers_state.qemu_icount_time_frac;
    uint64_t past = frac ? frac - 1 : (1ull << 32) - 1;

    return deadline > past ? deadline - past : 0;
}

/* Whether @cpu executes at its icount clock's rate. */
static bool icount_cpu_clocked(CPUState *cpu)
{
    return icount_enabled() == ICOUNT_PRECISE &&
           replay_mode == REPLAY_MODE_NONE &&
           qatomic_read(&cpu->icount_period) != 0;
}

static void icount_notify_aio_contexts(void)
{
    /* Wake up other AioContexts.  */
    qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
}

void icount_handle_deadline(void)
{
    assert(qemu_in_vcpu_thread());
    int64_t deadline = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                                  QEMU_TIMER_ATTR_ALL);

    /*
     * Instructions, interrupts, and exceptions are processed in cpu-exec.
     * Don't interrupt cpu thread, when these events are waiting
     * (i.e., there is no checkpoint)
     */
    if (deadline == 0) {
        icount_notify_aio_contexts();
    }
}

/*
 * The CPUs with an icount clock run through windows of virtual time
 * together, as parallel cores would. Each CPU not idle as a window starts
 * takes part in it, and runs until its own time, a period of its clock an
 * instruction, reaches the window's end, in as many turns as that takes
 * (a turn ends early at an exit request, or where virtual time reaches a
 * timer's deadline, which runs the timers in the window). Virtual time
 * follows their own times (icount_window_x()), so that when the window
 * ends it is at the end, and each CPU's own time within a period of it,
 * whatever turns they took and whenever one went idle or woke: each CPU's
 * instructions take exactly its clock's periods of virtual time from one
 * window to another. Within a window they take turns, so that a CPU sees
 * virtual time ahead of, or behind, its own by up to the window's length:
 * a quantum while more than one runs (1 us unless the option says
 * otherwise), the time to the next deadline while one does. A CPU that
 * comes out of idle, in or outside the window, joins it at the present
 * virtual time, the others' turns making way for it at once, and the
 * window then ends at most a quantum on.
 */

/* Some CPU wants to join the window or wakes in it. */
static bool icount_window_waiting;

/* Whether @cpu is taking part in its window and has not run to its end. */
static bool icount_window_running(CPUState *cpu)
{
    return timers_state.window_cpus && cpu->icount_in_window &&
           !cpu->icount_window_idle &&
           cpu->icount_own < (int64_t)timers_state.window_units;
}

/* @cpu takes no part in a window. */
static void icount_window_leave(CPUState *cpu)
{
    cpu->icount_in_window = false;
    cpu->icount_window_idle = false;
    cpu->icount_own = 0;
}

/*
 * Whether @cpu is idle as its window counts it: halted with nothing of the
 * guest's to wake it, or stopped. Unlike cpu_thread_is_idle(), a CPU is
 * not taken out of idle by a stop or work another thread has asked for
 * and the vCPU thread has yet to take: those come at moments of the
 * host's, and the vCPUs' schedule follows the guest alone
 * (tcg-accel-ops-rr.c). Called with the BQL held.
 */
static bool icount_cpu_idle(CPUState *cpu)
{
    return cpu->stopped || (cpu->halted && !cpu_has_work(cpu));
}

static void icount_window_update(int64_t quantum)
{
    uint64_t quantum_units, moved;
    bool done = true;
    int runnable = 0;
    int64_t width;
    CPUState *cpu;

    if (quantum < 0) {
        quantum = ICOUNT_CLOCKED_QUANTUM_NS;
    }
    quantum_units = quantum > 0 ? (uint64_t)quantum << 32 : UINT64_MAX;
    icount_window_waiting = false;

    CPU_FOREACH(cpu) {
        bool idle;

        if (!icount_cpu_clocked(cpu)) {
            icount_window_leave(cpu);
            continue;
        }
        idle = icount_cpu_idle(cpu);
        if (cpu->icount_in_window && idle != cpu->icount_window_idle) {
            icount_window_set_idle(cpu, idle);
        }
        if (!idle) {
            runnable++;
            if (icount_window_running(cpu)) {
                done = false;
            }
        }
    }
    if (!done && timers_state.window_cpus) {
        CPU_FOREACH(cpu) {
            if (icount_cpu_clocked(cpu) && !cpu->icount_in_window &&
                !icount_cpu_idle(cpu)) {
                icount_window_join(cpu, quantum_units);
            }
        }
        return;
    }

    width = icount_get_deadline();
    if (runnable > 1 && quantum > 0) {
        width = MIN(width, quantum);
    }
    width = MIN(width, INT32_MAX / MAX(runnable, 1));
    moved = icount_window_start((uint64_t)width << 32, runnable);
    CPU_FOREACH(cpu) {
        if (!icount_cpu_clocked(cpu) || icount_cpu_idle(cpu)) {
            icount_window_leave(cpu);
        } else if (cpu->icount_in_window && !cpu->icount_window_idle &&
                   moved != UINT64_MAX) {
            cpu->icount_own -= moved;
        } else {
            icount_window_leave(cpu);
            cpu->icount_in_window = true;
            icount_window_align(cpu);
        }
    }
}

/*
 * Start a round of the vCPUs. CPUs without an icount clock share the
 * instructions to the next deadline evenly; this returns each one's share.
 * CPUs with one go on through their window, or start the next.
 */
int64_t icount_percpu_budget(int cpu_count)
{
    int64_t limit = icount_get_limit();
    int64_t timeslice = limit / cpu_count;
    int64_t quantum = icount_quantum;

    if (timeslice == 0) {
        timeslice = limit;
    }
    if (cpu_count > 1 && quantum > 0) {
        timeslice = MIN(timeslice, icount_round(quantum));
    }
    if (icount_enabled() == ICOUNT_PRECISE &&
        replay_mode == REPLAY_MODE_NONE) {
        icount_window_update(quantum);
    }
    return timeslice;
}

/*
 * The instructions @cpu executes in @own units of its own time, rounded
 * up: of @period units each, plus the run's fraction of a unit.
 */
static int64_t icount_insns_in(CPUState *cpu, uint64_t own, uint64_t period)
{
    uint64_t n = own / period + (own % period != 0);
    uint64_t frac = cpu->icount_insn_frac, den = cpu->icount_insn_den;

    /* The longer exact period may need fewer: n - 1 of them may do. */
    while (frac && n > 1) {
        uint64_t m = n - 1, flo, fhi, dlo, dhi;

        mulu64(&flo, &fhi, m, frac);
        mulu64(&dlo, &dhi, own - m * period, den);
        if (fhi < dhi || (fhi == dhi && flo < dlo)) {
            break;
        }
        n = m;
    }
    return n;
}

/*
 * A run paused for another thread's request (icount_pause_run()): the rest
 * of its budget, and the time of the next QEMU_CLOCK_VIRTUAL timer as the
 * budget was counted to it.
 */
static int64_t icount_paused_budget;
static int64_t icount_run_timer;

/* The virtual time of the next QEMU_CLOCK_VIRTUAL timer, or INT64_MAX. */
static int64_t icount_next_timer(void)
{
    int64_t deadline = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                                  QEMU_TIMER_ATTR_ALL);

    if (deadline < 0) {
        return INT64_MAX;
    }
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + deadline;
}

/*
 * Let @cpu's run execute @budget instructions; with none, because virtual
 * time is at the next deadline, the timers due run first.
 */
static void icount_set_budget(CPUState *cpu, int64_t budget)
{
    int insns_left = MIN(0xffff, budget);

    cpu->icount_budget = budget;
    cpu->neg.icount_decr.u16.low = insns_left;
    cpu->icount_extra = budget - insns_left;

    if (budget == 0) {
        /* Timer handlers need the BQL, which the caller may not hold. */
        BQL_LOCK_GUARD();

        icount_notify_aio_contexts();
    }
}

void icount_prepare_for_run(CPUState *cpu, int64_t cpu_budget)
{
    /*
     * These should always be cleared by icount_process_data after
     * each vCPU execution. However u16.high can be raised
     * asynchronously by cpu_exit/cpu_interrupt/tcg_handle_interrupt
     */
    g_assert(cpu->neg.icount_decr.u16.low == 0);
    g_assert(cpu->icount_extra == 0);

    replay_mutex_lock();
    icount_run_timer = icount_next_timer();

    if (icount_cpu_clocked(cpu)) {
        /*
         * Each instruction takes a period of the CPU's clock of its own
         * time. The CPU runs to the end of its window in its own time, but
         * stops where virtual time reaches the next deadline, which its
         * own time moves on by at least 1/N of while N CPUs run in the
         * window, rounded up to a whole instruction as without a clock. A
         * CPU outside the window, or idle in it, runs once it has joined
         * or woken in it (icount_window_update()), which the others' turns
         * wait for meanwhile.
         */
        uint64_t period;
        uint64_t left = 0;

        if (cpu->icount_realign && cpu->icount_in_window &&
            !cpu->icount_window_idle) {
            icount_window_align(cpu);
        }
        period = qatomic_read(&cpu->icount_period);
        if (icount_window_running(cpu) && !icount_window_waiting) {
            uint64_t n = MAX(timers_state.window_running, 1);
            uint64_t deadline = icount_deadline_units();

            left = timers_state.window_units - cpu->icount_own;
            if (deadline <= (left - 1) / n) {
                left = deadline * n;
            }
        }
        cpu->icount_insn_time = period;
        cpu->icount_insn_den = qatomic_read(&cpu->icount_period_den);
        cpu->icount_insn_frac = qatomic_read(&cpu->icount_period_frac);
        icount_set_budget(cpu, icount_insns_in(cpu, left, period));
    } else {
        cpu->icount_insn_time = 0;
        cpu->icount_insn_frac = 0;
        icount_set_budget(cpu, MIN(icount_get_limit(), cpu_budget));
    }
}

bool icount_run_left(CPUState *cpu)
{
    return cpu->neg.icount_decr.u16.low + cpu->icount_extra > 0;
}

void icount_pause_run(CPUState *cpu)
{
    /*
     * Account for what the run has executed, as a device reading the clock
     * in the middle of a run does: a run accounted for in parts takes the
     * same time as one accounted for at its end.
     */
    icount_update(cpu);
    icount_paused_budget = cpu->icount_budget;
    cpu->neg.icount_decr.u16.low = 0;
    cpu->icount_extra = 0;
    cpu->icount_budget = 0;
    replay_mutex_unlock();
}

/*
 * The instructions @cpu's run may execute until virtual time reaches the
 * next deadline, as icount_prepare_for_run() counts them.
 */
static int64_t icount_budget_to_deadline(CPUState *cpu)
{
    if (cpu->icount_insn_time) {
        uint64_t n = MAX(timers_state.window_running, 1);
        uint64_t deadline = icount_deadline_units();

        if (deadline > UINT64_MAX / n) {
            return INT64_MAX;
        }
        return icount_insns_in(cpu, deadline * n, cpu->icount_insn_time);
    }
    return icount_get_limit();
}

void icount_resume_run(CPUState *cpu, bool go_on)
{
    int64_t budget = icount_paused_budget;
    int64_t timer;

    replay_mutex_lock();
    if (!go_on) {
        return;
    }
    /*
     * A timer set meanwhile (as a device took input on the main loop) may
     * come before the end of the run: the run stops where virtual time
     * reaches it, as it does for any other.
     */
    timer = icount_next_timer();
    if (timer < icount_run_timer) {
        icount_run_timer = timer;
        budget = MIN(budget, icount_budget_to_deadline(cpu));
    }
    icount_set_budget(cpu, budget);
}

void icount_process_data(CPUState *cpu)
{
    /* Account for executed instructions */
    icount_update(cpu);

    /*
     * A CPU running in its window that has gone idle (in WFI, stopped)
     * stays where it stopped. One outside the window, or idle in it, that
     * has come out of its halt is to join it: the others' turns end until
     * it has.
     */
    if (icount_cpu_clocked(cpu)) {
        if (cpu->icount_in_window && !cpu->icount_window_idle) {
            if (cpu->halted) {
                BQL_LOCK_GUARD();

                if (icount_cpu_idle(cpu)) {
                    icount_window_set_idle(cpu, true);
                }
            }
        } else if (!cpu->halted) {
            icount_window_waiting = true;
        }
    }

    /* Reset the counters */
    cpu->neg.icount_decr.u16.low = 0;
    cpu->icount_extra = 0;
    cpu->icount_budget = 0;

    replay_account_executed_instructions();

    replay_mutex_unlock();
}

void icount_handle_interrupt(CPUState *cpu, int mask)
{
    int old_mask = cpu->interrupt_request;

    tcg_handle_interrupt(cpu, mask);
    /*
     * A CPU with an icount clock interrupting another (an inter-processor
     * interrupt from its register write) ends its turn, so that the other
     * gets its turn at once, as a parallel core would take the interrupt,
     * rather than when the round comes back to it.
     */
    if (current_cpu && current_cpu != cpu && current_cpu->running &&
        icount_cpu_clocked(current_cpu) && icount_cpu_clocked(cpu)) {
        cpu_exit(current_cpu);
    }
    if (qemu_cpu_is_self(cpu) &&
        !cpu->neg.can_do_io
        && (mask & ~old_mask) != 0) {
        cpu_abort(cpu, "Raised interrupt while not in I/O function");
    }
}
