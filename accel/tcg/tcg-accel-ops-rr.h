/*
 * QEMU TCG Single Threaded vCPUs implementation
 *
 * Copyright 2020 SUSE LLC
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef TCG_ACCEL_OPS_RR_H
#define TCG_ACCEL_OPS_RR_H

#define TCG_KICK_PERIOD (NANOSECONDS_PER_SECOND / 10)

/*
 * Kick all RR vCPUs. Under icount, a kick from another thread pauses the
 * vCPUs' schedule, and one the round-robin thread gives itself ends the
 * running vCPU's turn (tcg-accel-ops-rr.c).
 */
void rr_kick_vcpu_thread(CPUState *unused);

/*
 * Whether the round-robin thread waits for an event with every vCPU idle,
 * having accounted for every instruction they ran. Call with the BQL held.
 */
bool rr_vcpu_thread_waiting(void);

/* start the round robin vcpu thread */
void rr_start_vcpu_thread(CPUState *cpu);

#endif /* TCG_ACCEL_OPS_RR_H */
