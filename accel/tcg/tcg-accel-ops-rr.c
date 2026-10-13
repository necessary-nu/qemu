/*
 * QEMU TCG Single Threaded vCPUs implementation
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
#include "qemu/lockable.h"
#include "qemu/seqlock.h"
#include "system/tcg.h"
#include "system/replay.h"
#include "exec/icount.h"
#include "system/cpu-timers-internal.h"
#include "qemu/main-loop.h"
#include "qemu/notify.h"
#include "qemu/guest-random.h"
#include "exec/cpu-common.h"
#include "accel/tcg/cpu-loop.h"
#include "tcg/startup.h"
#include "tcg-accel-ops.h"
#include "tcg-accel-ops-rr.h"
#include "tcg-accel-ops-icount.h"

/*
 * Under icount, outside record/replay, what the vCPUs do follows from the
 * guest and the virtual times of its inputs alone, and so must the
 * round-robin schedule: whose turn it is, how far each turn runs, and the
 * windows the CPUs with an icount clock run through (tcg-accel-ops-icount.c).
 * A kick from another thread comes at a moment of the host's: a monitor
 * stopping or continuing the VM, work queued on a vCPU (run_on_cpu(), or
 * the main loop moving a timer earlier), an interrupt a device raises as
 * it takes input on the main loop, the RCU thread hurrying this thread
 * through a grace period. It pauses the schedule without changing it
 * (rr_pause()): the running vCPU leaves its run at the end of a
 * translation block, this thread does what was asked with the BQL held,
 * waiting while the VM is stopped, and the vCPU goes on with the rest of
 * its run in the same turn. No timer runs, no window moves and no turn
 * ends at the pause that would not have otherwise; the only change is
 * where a timer set meanwhile, as a device takes input, cuts the run
 * short, as any timer does. A kick this thread gives itself, as the vCPUs
 * execute or their timers run (an inter-processor interrupt, a timer
 * moved, the guest stopping the VM), comes at the guest's time: it ends
 * the running vCPU's turn, and every other's as the round comes to it,
 * as an exit request always has (cpu->rr_exit_request).
 */
static bool rr_pause_on_kick(void)
{
    return icount_enabled() && replay_mode == REPLAY_MODE_NONE;
}

/* Another thread has kicked since this thread last took its requests. */
static bool rr_host_request;
/*
 * This thread is doing what other threads asked of it, or waking the main
 * loop: a kick it gives itself meanwhile (queuing a bottom half kicks the
 * vCPU under icount, see aio_bh_enqueue()) is theirs, not the guest's.
 */
static bool rr_taking_requests;
/* timers_state.loads as this thread last started its schedule afresh. */
static unsigned rr_loads;

/* Kick all RR vCPUs */
void rr_kick_vcpu_thread(CPUState *unused)
{
    bool own = qemu_in_vcpu_thread() && !rr_taking_requests;
    CPUState *cpu;

    if (!own) {
        qatomic_set_mb(&rr_host_request, true);
    }
    CPU_FOREACH(cpu) {
        if (own) {
            cpu->rr_exit_request = true;
        }
        tcg_kick_vcpu_thread(cpu);
    }
}

/*
 * TCG vCPU kick timer
 *
 * The kick timer is responsible for moving single threaded vCPU
 * emulation on to the next vCPU. If more than one vCPU is running a
 * timer event we force a cpu->exit so the next vCPU can get
 * scheduled.
 *
 * The timer is removed if all vCPUs are idle and restarted again once
 * idleness is complete.
 */

static QEMUTimer *rr_kick_vcpu_timer;
static CPUState *rr_current_cpu;

static inline int64_t rr_next_kick_time(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TCG_KICK_PERIOD;
}

/* Kick the currently round-robin scheduled vCPU to next */
static void rr_kick_next_cpu(void)
{
    CPUState *cpu;
    do {
        cpu = qatomic_read(&rr_current_cpu);
        if (cpu) {
            cpu_exit(cpu);
        }
        /* Finish kicking this cpu before reading again.  */
        smp_mb();
    } while (cpu != qatomic_read(&rr_current_cpu));
}

static void rr_kick_thread(void *opaque)
{
    timer_mod(rr_kick_vcpu_timer, rr_next_kick_time());
    rr_kick_next_cpu();
}

static void rr_start_kick_timer(void)
{
    if (!rr_kick_vcpu_timer && CPU_NEXT(first_cpu)) {
        rr_kick_vcpu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                           rr_kick_thread, NULL);
    }
    if (rr_kick_vcpu_timer && !timer_pending(rr_kick_vcpu_timer)) {
        timer_mod(rr_kick_vcpu_timer, rr_next_kick_time());
    }
}

static void rr_stop_kick_timer(void)
{
    if (rr_kick_vcpu_timer && timer_pending(rr_kick_vcpu_timer)) {
        timer_del(rr_kick_vcpu_timer);
    }
}

/*
 * The thread waits for an event with every vCPU idle (set and cleared with
 * the BQL held). It executes the vCPUs without the BQL, and adds the
 * instructions of each run to virtual time after the run, so a vCPU that
 * has halted in its run (in WFI, say) looks idle to a thread holding the
 * BQL before virtual time includes them. Once this thread waits, it has
 * added them all, and it runs no vCPU until it is woken.
 */
static bool rr_waiting;

bool rr_vcpu_thread_waiting(void)
{
    return rr_waiting;
}

/*
 * With the BQL held, do what other threads have asked of the vCPUs (stop,
 * queued work), and drop the exit requests their kicks left, which end no
 * turn. A kick whose exit requests are dropped before it sets
 * rr_host_request is taken again; one that sets it after it is cleared
 * leaves its exit requests in place, to be taken at the next pause.
 */
static void rr_take_host_requests(void)
{
    CPUState *cpu;

    rr_taking_requests = true;
    do {
        CPU_FOREACH(cpu) {
            if (!cpu->rr_exit_request) {
                qatomic_set(&cpu->exit_request, false);
            }
        }
        CPU_FOREACH(cpu) {
            qemu_process_cpu_events_common(cpu);
        }
    } while (qatomic_xchg(&rr_host_request, false));
    rr_taking_requests = false;
}

/* Whether another thread has asked something of the vCPUs, as @cpu shows. */
static bool rr_host_requested(CPUState *cpu)
{
    return qatomic_read(&rr_host_request) || cpu->stop ||
           !cpu_work_list_empty(cpu) || qatomic_read(&cpu->exit_request);
}

static bool rr_all_stopped(void)
{
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        if (!cpu_is_stopped(cpu)) {
            return false;
        }
    }
    return true;
}

/*
 * With the BQL held, between two runs or in the middle of one: take other
 * threads' requests, waiting while they have the VM stopped. Returns false
 * if state was loaded meanwhile, so that the schedule starts afresh.
 */
static bool rr_pause(void)
{
    for (;;) {
        rr_take_host_requests();
        if (!rr_all_stopped()) {
            break;
        }
        qemu_cond_wait_bql(first_cpu->halt_cond);
    }
    return qatomic_read(&timers_state.loads) == rr_loads;
}

/*
 * Start the schedule afresh, as the machine starts it: a round from the
 * first vCPU, with no exit request pending and the kick timer a period
 * on. State loaded (an incoming migration, loadvm) carries none of the
 * schedule, so it goes on the same in a running QEMU as in a new one; the
 * CPUs with an icount clock start a window afresh too.
 */
static CPUState *rr_restart(void)
{
    CPUState *cpu;

    rr_loads = qatomic_read(&timers_state.loads);
    CPU_FOREACH(cpu) {
        cpu->rr_exit_request = false;
        qatomic_set(&cpu->exit_request, false);
    }
    rr_stop_kick_timer();
    rr_start_kick_timer();
    return first_cpu;
}

/*
 * @cpu's run left cpu_exec() with @excp. Where a kick from another thread
 * rather than the run's end made it leave, pause and go on with the rest
 * of the run, in the same turn; returns how the run finally left. Called
 * without the BQL. Sets *@restart if the schedule is to start afresh.
 */
static int rr_run_through_kicks(CPUState *cpu, int excp, bool *restart)
{
    while (rr_pause_on_kick() && excp == EXCP_INTERRUPT &&
           !cpu->rr_exit_request && icount_run_left(cpu)) {
        bool go_on;

        icount_pause_run(cpu);
        bql_lock();
        go_on = rr_pause();
        *restart = !go_on;
        go_on = go_on && cpu_can_run(cpu);
        if (go_on) {
            qemu_clock_enable(QEMU_CLOCK_VIRTUAL,
                              (cpu->singlestep_flags & SSTEP_NOTIMER) == 0);
        }
        icount_resume_run(cpu, go_on);
        bql_unlock();
        if (!go_on) {
            break;
        }
        excp = tcg_cpu_exec(cpu);
    }
    return excp;
}

/* Every vCPU waits for an event of the guest's, whether stopped or not. */
static bool rr_all_halted(void)
{
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        if (!cpu->halted || cpu_has_work(cpu)) {
            return false;
        }
    }
    return true;
}

static void rr_wait_io_event(void)
{
    CPUState *cpu;

    if (rr_pause_on_kick()) {
        /*
         * Other threads' requests are taken here too, at the round's end,
         * and a wake that leaves every vCPU idle (a cont, a stop, work
         * done at once) leaves the thread waiting, rather than taking a
         * round, and starting a window, that it would not have otherwise.
         * A timer due ends the wait, to run in the round. Each wait wakes
         * the main loop, which may then move virtual time on. The kick
         * timer stops only while the vCPUs wait for the guest: stopped
         * and started again for the VM's stop, it would expire, and its
         * restart kick this thread, at the moments of a monitor's.
         */
        for (;;) {
            rr_take_host_requests();
            if (!all_cpu_threads_idle() ||
                qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                           QEMU_TIMER_ATTR_ALL) == 0) {
                break;
            }
            if (rr_all_halted()) {
                rr_stop_kick_timer();
            }
            rr_waiting = true;
            rr_taking_requests = true;
            qemu_notify_event();
            rr_taking_requests = false;
            qemu_cond_wait_bql(first_cpu->halt_cond);
            rr_waiting = false;
        }
        rr_start_kick_timer();
        return;
    }

    while (all_cpu_threads_idle()) {
        rr_stop_kick_timer();
        rr_waiting = true;
        qemu_cond_wait_bql(first_cpu->halt_cond);
        rr_waiting = false;
    }

    rr_start_kick_timer();

    CPU_FOREACH(cpu) {
        qemu_process_cpu_events_common(cpu);
    }
}

/*
 * Destroy any remaining vCPUs which have been unplugged and have
 * finished running
 */
static void rr_deal_with_unplugged_cpus(void)
{
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        if (cpu->unplug && !cpu_can_run(cpu)) {
            tcg_cpu_destroy(cpu);
            break;
        }
    }
}

static void rr_force_rcu(Notifier *notify, void *data)
{
    rr_kick_next_cpu();
}

/*
 * Calculate the number of CPUs that we will process in a single iteration of
 * the main CPU thread loop so that we can fairly distribute the instruction
 * count across CPUs.
 *
 * The CPU count is cached based on the CPU list generation ID to avoid
 * iterating the list every time.
 */
static int rr_cpu_count(void)
{
    static unsigned int last_gen_id = ~0;
    static int cpu_count;
    CPUState *cpu;

    QEMU_LOCK_GUARD(&qemu_cpu_list_lock);

    if (cpu_list_generation_id_get() != last_gen_id) {
        cpu_count = 0;
        CPU_FOREACH(cpu) {
            ++cpu_count;
        }
        last_gen_id = cpu_list_generation_id_get();
    }

    return cpu_count;
}

/*
 * In the single-threaded case each vCPU is simulated in turn. If
 * there is more than a single vCPU we create a simple timer to kick
 * the vCPU and ensure we don't get stuck in a tight loop in one vCPU.
 * This is done explicitly rather than relying on side-effects
 * elsewhere.
 */

static void *rr_cpu_thread_fn(void *arg)
{
    Notifier force_rcu;
    CPUState *cpu = arg;

    assert(tcg_enabled());
    rcu_register_thread();
    force_rcu.notify = rr_force_rcu;
    rcu_add_force_rcu_notifier(&force_rcu);
    tcg_register_thread();

    bql_lock();
    qemu_thread_get_self(cpu->thread);

    cpu->thread_id = qemu_get_thread_id();
    cpu->neg.can_do_io = true;
    cpu_thread_signal_created(cpu);
    qemu_guest_random_seed_thread_part2(cpu->random_seed);

    /* wait for initial kick-off after machine start */
    while (cpu_is_stopped(first_cpu)) {
        qemu_cond_wait_bql(first_cpu->halt_cond);

        /* process any pending work */
        CPU_FOREACH(cpu) {
            current_cpu = cpu;
            qemu_process_cpu_events_common(cpu);
        }
    }

    rr_start_kick_timer();
    rr_loads = qatomic_read(&timers_state.loads);

    cpu = first_cpu;

    while (1) {
        /* Only used for icount_enabled() */
        int64_t cpu_budget = 0;
        bool restart = false;

        if (cpu) {
            /*
             * This could even reset exit_request for all CPUs, but in practice
             * races between CPU exits and changes to "cpu" are so rare that
             * there's no advantage in doing so.
             */
            qatomic_set(&cpu->exit_request, false);
            cpu->rr_exit_request = false;
        }

        /*
         * Under rr_pause_on_kick(), only for idleness of the guest's: waking
         * the main loop kicks this thread (aio_bh_enqueue()), and the VM's
         * stop leaves every vCPU idle at a moment of the host's.
         */
        if (icount_enabled() &&
            (rr_pause_on_kick() ? rr_all_halted() : all_cpu_threads_idle())) {
            /*
             * When all cpus are sleeping (e.g in WFI), to avoid a deadlock
             * in the main_loop, wake it up in order to start the warp timer.
             */
            qemu_notify_event();
        }

        rr_wait_io_event();
        if (rr_pause_on_kick() &&
            qatomic_read(&timers_state.loads) != rr_loads) {
            cpu = rr_restart();
            continue;
        }
        rr_deal_with_unplugged_cpus();

        /*
         * Under rr_pause_on_kick() there is no replay mutex to take, and the
         * BQL stays held from the wait, which has taken other threads'
         * requests, through the timers, the windows and each run's budget:
         * a stop coming between would have them counted with the virtual
         * clock disabled and the vCPUs about to stop.
         */
        if (!rr_pause_on_kick()) {
            bql_unlock();
            replay_mutex_lock();
            bql_lock();
        }

        if (icount_enabled()) {
            int cpu_count = rr_cpu_count();

            /* Account partial waits to QEMU_CLOCK_VIRTUAL.  */
            icount_account_warp_timer();
            /*
             * Run the timers here.  This is much more efficient than
             * waking up the I/O thread and waiting for completion.
             */
            icount_handle_deadline();

            cpu_budget = icount_percpu_budget(cpu_count);
        }

        if (!rr_pause_on_kick()) {
            replay_mutex_unlock();
        }

        if (!cpu) {
            cpu = first_cpu;
        }

        while (cpu) {
            /*
             * Store rr_current_cpu before evaluating cpu->exit_request.
             * Pairs with rr_kick_next_cpu().
             */
            qatomic_set_mb(&rr_current_cpu, cpu);

            if (rr_pause_on_kick()) {
                if (cpu->rr_exit_request) {
                    break;
                }
                if (rr_host_requested(cpu) && !rr_pause()) {
                    cpu = rr_restart();
                    break;
                }
            } else if (!cpu_work_list_empty(cpu) ||
                       qatomic_load_acquire(&cpu->exit_request)) {
                /* Pairs with store-release in cpu_exit.  */
                break;
            }
            current_cpu = cpu;

            if (cpu_can_run(cpu)) {
                int excp;

                /*
                 * Only for a CPU about to run: pause_all_vcpus() disables
                 * the clock before it stops the vCPUs, so that no timer
                 * runs while the VM is stopped, and a CPU taking its stop
                 * here must not enable it again behind its back. With
                 * the VM stopped no CPU can run, and resume_all_vcpus()
                 * enables the clock as it starts them.
                 */
                qemu_clock_enable(QEMU_CLOCK_VIRTUAL,
                                  (cpu->singlestep_flags & SSTEP_NOTIMER) == 0);
                /* Under rr_pause_on_kick(), with the BQL held, as above. */
                if (icount_enabled() && rr_pause_on_kick()) {
                    icount_prepare_for_run(cpu, cpu_budget);
                }
                bql_unlock();
                if (icount_enabled() && !rr_pause_on_kick()) {
                    icount_prepare_for_run(cpu, cpu_budget);
                }
                excp = tcg_cpu_exec(cpu);
                excp = rr_run_through_kicks(cpu, excp, &restart);
                if (icount_enabled()) {
                    icount_process_data(cpu);
                }
                bql_lock();

                if (restart) {
                    cpu = rr_restart();
                    break;
                } else if (excp == EXCP_DEBUG) {
                    cpu_handle_guest_debug(cpu);
                    break;
                } else if (excp == EXCP_ATOMIC) {
                    bql_unlock();
                    cpu_exec_step_atomic(cpu);
                    bql_lock();
                    break;
                }
            } else if (cpu->stop) {
                if (cpu->unplug) {
                    cpu = CPU_NEXT(cpu);
                }
                break;
            }

            cpu = CPU_NEXT(cpu);
        } /* while (cpu && !cpu->exit_request).. */

        /* Does not need a memory barrier because a spurious wakeup is okay.  */
        qatomic_set(&rr_current_cpu, NULL);
    }

    g_assert_not_reached();
}

void rr_start_vcpu_thread(CPUState *cpu)
{
    char thread_name[VCPU_THREAD_NAME_SIZE];
    static QemuCond *single_tcg_halt_cond;
    static QemuThread *single_tcg_cpu_thread;

    g_assert(tcg_enabled());
    tcg_cpu_init_cflags(cpu, false);

    if (!single_tcg_cpu_thread) {
        single_tcg_halt_cond = cpu->halt_cond;
        single_tcg_cpu_thread = cpu->thread;

        /* share a single thread for all cpus with TCG */
        snprintf(thread_name, VCPU_THREAD_NAME_SIZE, "ALL CPUs/TCG");
        qemu_thread_create(cpu->thread, thread_name,
                           rr_cpu_thread_fn,
                           cpu, QEMU_THREAD_JOINABLE);
    } else {
        /* we share the thread, dump spare data */
        g_free(cpu->thread);
        qemu_cond_destroy(cpu->halt_cond);
        g_free(cpu->halt_cond);
        cpu->thread = single_tcg_cpu_thread;
        cpu->halt_cond = single_tcg_halt_cond;

        /* copy the stuff done at start of rr_cpu_thread_fn */
        cpu->thread_id = first_cpu->thread_id;
        cpu->neg.can_do_io = 1;
        cpu->created = true;
    }
}
