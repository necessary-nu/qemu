/*
 * Dummy cpu thread code
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qemu/rcu.h"
#include "system/cpus.h"
#include "qemu/guest-random.h"
#include "qemu/main-loop.h"
#include "hw/core/cpu.h"
#include "accel/dummy-cpus.h"

/*
 * A dummy vCPU thread executes nothing; it only sleeps until kicked and
 * then handles the CPU's events (stop requests, queued work, unplug).
 *
 * The kick is a per-CPU semaphore rather than SIG_IPI. A signal sent with
 * pthread_kill() to one thread is not reliably received by that thread's
 * sigwait(): on Darwin, sigwait() takes a matching signal that is pending
 * on any thread of the process, so with several dummy vCPU threads one
 * thread's sigwait() can consume the SIG_IPI aimed at another while that
 * other is not yet waiting. The target then sleeps with its kick marked
 * delivered (thread_kicked set, so later kicks send nothing) and every
 * pause_all_vcpus() waits for it forever. A semaphore belongs to one
 * thread's wait, so no kick can be lost to another vCPU.
 */
void dummy_kick_vcpu_thread(CPUState *cpu)
{
    if (qatomic_read(&cpu->thread_kicked)) {
        return;
    }
    qatomic_set(&cpu->thread_kicked, true);
    qemu_sem_post(&cpu->sem);
}

static void *dummy_cpu_thread_fn(void *arg)
{
    CPUState *cpu = arg;

    rcu_register_thread();

    bql_lock();
    qemu_thread_get_self(cpu->thread);
    cpu->thread_id = qemu_get_thread_id();
    current_cpu = cpu;

    /* signal CPU creation */
    cpu_thread_signal_created(cpu);
    qemu_guest_random_seed_thread_part2(cpu->random_seed);

    do {
        qemu_process_cpu_events(cpu);
        bql_unlock();
        qemu_sem_wait(&cpu->sem);
        bql_lock();
    } while (!cpu->unplug);

    bql_unlock();
    rcu_unregister_thread();
    return NULL;
}

void dummy_start_vcpu_thread(CPUState *cpu)
{
    char thread_name[VCPU_THREAD_NAME_SIZE];

    qemu_sem_init(&cpu->sem, 0);
    snprintf(thread_name, VCPU_THREAD_NAME_SIZE, "CPU %d/DUMMY",
             cpu->cpu_index);
    qemu_thread_create(cpu->thread, thread_name, dummy_cpu_thread_fn, cpu,
                       QEMU_THREAD_JOINABLE);
}
