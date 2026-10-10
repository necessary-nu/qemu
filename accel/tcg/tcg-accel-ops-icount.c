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
#include "qemu/main-loop.h"
#include "qemu/guest-random.h"
#include "hw/core/cpu.h"
#include "system/cpus.h"

#include "tcg-accel-ops.h"
#include "tcg-accel-ops-icount.h"
#include "tcg-accel-ops-rr.h"

/* The quantum of CPUs with an icount clock, unless the option sets one. */
#define ICOUNT_CLOCKED_QUANTUM_NS 1000

/*
 * The round of the vCPUs icount_percpu_budget() starts: how many CPUs with
 * an icount clock take turns in it, and how much of its own time (in ns)
 * each of those runs for at most.
 */
static int icount_round_cpus = 1;
static int64_t icount_round_slice;

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
 * Start a round of the vCPUs. CPUs without an icount clock share the
 * instructions to the next deadline evenly; this returns each one's share.
 * CPUs with one each run up to the deadline in their own time, or for a
 * quantum while others with a clock are not idle, so that they move on
 * together.
 */
int64_t icount_percpu_budget(int cpu_count)
{
    int64_t limit = icount_get_limit();
    int64_t timeslice = limit / cpu_count;
    int64_t quantum = icount_quantum;
    CPUState *cpu;
    int clocked = 0;

    if (timeslice == 0) {
        timeslice = limit;
    }
    if (cpu_count > 1 && quantum > 0) {
        timeslice = MIN(timeslice, icount_round(quantum));
    }

    CPU_FOREACH(cpu) {
        if (icount_cpu_clocked(cpu) && !cpu_thread_is_idle(cpu)) {
            clocked++;
        }
    }
    icount_round_cpus = MAX(clocked, 1);
    if (replay_mode == REPLAY_MODE_NONE) {
        icount_round_slice = icount_get_deadline();
        if (quantum < 0) {
            quantum = ICOUNT_CLOCKED_QUANTUM_NS;
        }
        if (clocked > 1 && quantum > 0) {
            icount_round_slice = MIN(icount_round_slice, quantum);
        }
    }

    return timeslice;
}

void icount_prepare_for_run(CPUState *cpu, int64_t cpu_budget)
{
    int insns_left;

    /*
     * These should always be cleared by icount_process_data after
     * each vCPU execution. However u16.high can be raised
     * asynchronously by cpu_exit/cpu_interrupt/tcg_handle_interrupt
     */
    g_assert(cpu->neg.icount_decr.u16.low == 0);
    g_assert(cpu->icount_extra == 0);

    replay_mutex_lock();

    if (icount_cpu_clocked(cpu)) {
        /*
         * Each instruction takes a period of the CPU's clock of its own
         * time, which is 1/N of that of virtual time while N CPUs take
         * turns. The CPU runs for the round's slice of its own time but
         * stops at the next deadline, rounded up to a whole instruction
         * as without a clock.
         */
        uint64_t period = qatomic_read(&cpu->icount_period);
        int64_t own = MIN(icount_round_slice,
                          icount_get_deadline() * icount_round_cpus);

        cpu->icount_insn_time = period / icount_round_cpus;
        cpu->icount_budget = (((uint64_t)own << 32) + period - 1) / period;
    } else {
        cpu->icount_insn_time = 0;
        cpu->icount_budget = MIN(icount_get_limit(), cpu_budget);
    }
    insns_left = MIN(0xffff, cpu->icount_budget);
    cpu->neg.icount_decr.u16.low = insns_left;
    cpu->icount_extra = cpu->icount_budget - insns_left;

    if (cpu->icount_budget == 0) {
        /*
         * We're called without the BQL, so must take it while
         * we're calling timer handlers.
         */
        bql_lock();
        icount_notify_aio_contexts();
        bql_unlock();
    }
}

void icount_process_data(CPUState *cpu)
{
    /* Account for executed instructions */
    icount_update(cpu);

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
    if (qemu_cpu_is_self(cpu) &&
        !cpu->neg.can_do_io
        && (mask & ~old_mask) != 0) {
        cpu_abort(cpu, "Raised interrupt while not in I/O function");
    }
}
