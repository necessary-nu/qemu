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

#ifndef TIMERS_STATE_H
#define TIMERS_STATE_H

/* timers state, for sharing between icount and cpu-timers */

typedef struct TimersState {
    /* Protected by BQL.  */
    int64_t cpu_ticks_prev;
    int64_t cpu_ticks_offset;

    /*
     * Protect fields that can be respectively read outside the
     * BQL, and written from multiple threads.
     */
    QemuSeqLock vm_clock_seqlock;
    QemuSpin vm_clock_lock;

    int16_t cpu_ticks_enabled;

    /* Conversion factor from emulated instructions to virtual clock ticks.  */
    int16_t icount_time_shift;
    /* Icount delta used for shift auto adjust. */
    int64_t last_delta;

    /* Compensate for varying guest execution speed.  */
    int64_t qemu_icount_bias;

    int64_t vm_clock_warp_start;
    int64_t cpu_clock_offset;

    /* Only written by TCG thread */
    int64_t qemu_icount;
    /*
     * The virtual time the executed instructions account for, in ns and
     * 2^-32 ns. Each instruction accounts for its CPU's instruction time
     * (2^icount_time_shift ns, or a cycle of its icount clock) when it
     * executes, so the time already passed stays put when that changes.
     */
    int64_t qemu_icount_time;
    uint32_t qemu_icount_time_frac;

    /*
     * The window of virtual time the CPUs with an icount clock run through
     * together (tcg-accel-ops-icount.c), from window_ns + window_frac
     * 2^-32 ns of qemu_icount_time on, window_units 2^-32 ns long, which
     * window_cpus CPUs take part in, window_running of them not idle. No
     * window while window_cpus is 0. Written only by the vCPU thread; not
     * migrated: a window starts afresh.
     */
    int64_t window_ns;
    uint32_t window_frac;
    uint64_t window_units;
    int window_cpus;
    int window_running;
    /*
     * qemu_icount_bias as the window started. Virtual time warps on only
     * while every vCPU is idle; a warp under a window ends it.
     */
    int64_t window_bias;

    /*
     * How many times icount's state has been loaded (an incoming
     * migration, loadvm), with the VM stopped: TCG's round-robin vCPU
     * thread then starts its schedule afresh, as at the machine's start.
     */
    unsigned loads;

    /* for adjusting icount */
    QEMUTimer *icount_rt_timer;
    QEMUTimer *icount_vm_timer;
    QEMUTimer *icount_warp_timer;
} TimersState;

extern TimersState timers_state;

/*
 * icount needs this internal from cpu-timers when adjusting the icount shift.
 */
int64_t cpu_get_clock_locked(void);

#endif /* TIMERS_STATE_H */
