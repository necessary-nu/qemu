/*
 * QEMU TCG Single Threaded vCPUs implementation using instruction counting
 *
 * Copyright 2020 SUSE LLC
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef TCG_ACCEL_OPS_ICOUNT_H
#define TCG_ACCEL_OPS_ICOUNT_H

/*
 * The "quantum" option: the most virtual time, in ns, a vCPU runs before
 * the next takes its turn, 0 for no limit, or -1 when not given.
 */
extern int64_t icount_quantum;

/*
 * The window of virtual time the vCPUs with an icount clock run through
 * together (icount-common.c). icount_window_start() starts one @units of
 * 2^-32 ns long at the present virtual time for @cpus vCPUs, and returns
 * how far its start is from the previous window's, or UINT64_MAX if
 * there was none or it is far behind. icount_window_set_idle() tells it
 * that @cpu, taking part, has gone idle or woken. icount_window_join()
 * adds @cpu at the present virtual time, and ends the window at most
 * @units on.
 */
uint64_t icount_window_start(uint64_t units, int cpus);
void icount_window_set_idle(CPUState *cpu, bool idle);
void icount_window_join(CPUState *cpu, uint64_t units);

/*
 * Each CPU's instructions start and end on its clock's edges, whole
 * multiples of its exact period from time 0, the edges devices timed on
 * the clock count. A CPU starts in a window on the first edge at or after
 * its start, joins one or wakes in it on the first edge at or after the
 * present, and goes on from edge to edge, a period an instruction.
 * icount_window_align() puts @cpu's own time on its clock's first edge at
 * or after it: as it starts in a window, and after its clock changes.
 */
void icount_window_align(CPUState *cpu);

void icount_handle_deadline(void);
void icount_prepare_for_run(CPUState *cpu, int64_t cpu_budget);
int64_t icount_percpu_budget(int cpu_count);
void icount_process_data(CPUState *cpu);

void icount_handle_interrupt(CPUState *cpu, int mask);

#endif /* TCG_ACCEL_OPS_ICOUNT_H */
