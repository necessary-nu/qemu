/*
 * icount - Instruction Counter API
 * CPU timers state API
 *
 * Copyright 2020 SUSE LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef EXEC_ICOUNT_H
#define EXEC_ICOUNT_H

/**
 * ICountMode: icount enablement state:
 *
 * @ICOUNT_DISABLED: Disabled - Do not count executed instructions.
 * @ICOUNT_PRECISE: Enabled - Fixed conversion of insn to ns via "shift" option
 * @ICOUNT_ADAPTATIVE: Enabled - Runtime adaptive algorithm to compute shift
 */
typedef enum {
    ICOUNT_DISABLED = 0,
    ICOUNT_PRECISE,
    ICOUNT_ADAPTATIVE,
} ICountMode;

#ifdef CONFIG_TCG
extern ICountMode use_icount;
#define icount_enabled() (use_icount)
#else
#define icount_enabled() ICOUNT_DISABLED
#endif

/* Protect the CONFIG_USER_ONLY test vs poisoning. */
#if defined(COMPILING_PER_TARGET) || defined(COMPILING_SYSTEM_VS_USER)
# ifdef CONFIG_USER_ONLY
#  undef  icount_enabled
#  define icount_enabled() ICOUNT_DISABLED
# endif
#endif

/*
 * Update the icount with the executed instructions. Called by
 * cpus-tcg vCPU thread so the main-loop can see time has moved forward.
 */
void icount_update(CPUState *cpu);

/* get raw icount value */
int64_t icount_get_raw(void);

/* return the virtual CPU time in ns, based on the instruction counter. */
int64_t icount_get(void);
/*
 * icount_get_precise: the virtual time icount_get() returns, and in @frac
 * the fraction of a nanosecond beyond it, in units of 2^-32 ns, that the
 * instructions of CPUs executing at a clock's rate have run up (see
 * icount_set_cpu_clock()); 0 when no CPU has a clock.
 */
int64_t icount_get_precise(uint32_t *frac);
/*
 * convert an instruction counter value to ns, based on the icount shift.
 * This shift is set as a fixed value with the icount "shift" option
 * (precise mode), or it is constantly approximated and corrected at
 * runtime in adaptive mode.
 */
int64_t icount_to_ns(int64_t icount);

/**
 * icount_configure: configure the icount options, including "shift"
 * @opts: Options to parse
 * @errp: pointer to a NULL-initialized error object
 *
 * Return: true on success, else false setting @errp with error
 */
bool icount_configure(QemuOpts *opts, Error **errp);

/* used by tcg vcpu thread to calc icount budget */
int64_t icount_round(int64_t count);

/**
 * icount_set_cpu_clock: execute a CPU's instructions at its clock's rate
 * @cpu: the CPU
 * @clk: the clock the CPU executes one instruction per cycle of
 *
 * With a fixed "shift", each instruction @cpu executes takes one period
 * of @clk, as it is when the instruction executes, instead of 2^shift ns.
 * CPUs with a clock run as if in parallel: while N of them are not idle,
 * each executes at its clock's rate and an instruction moves virtual time
 * on by 1/N of its period, and they take turns at most a quantum of their
 * own time long (the "quantum" option, by default 1 us), so code on one
 * waiting for another sees it make progress as on real cores. In adaptive
 * mode and under record/replay the clock is ignored.
 *
 * Stopping @clk does not stop @cpu: a machine whose clock stops its CPUs
 * halts them itself.
 */
void icount_set_cpu_clock(CPUState *cpu, Clock *clk);

/**
 * icount_set_cpu_period_frac: give the exact period of a CPU's clock
 * @cpu: the CPU, with a clock from icount_set_cpu_clock()
 * @frac: the fraction of a Clock unit the exact period has beyond the
 *   clock's period, @frac / @den, below 1
 * @den: its denominator, not 0
 *
 * A Clock holds its period as a whole number of 2^-32 ns units. For a
 * clock whose period is not one, the machine that knows it exactly gives
 * the fraction of a unit beyond, after each change of the clock: while
 * @cpu executes alone, its instructions then take the exact period each,
 * the fractions carried from one to the next, so that n of them take n
 * exact periods to within a unit however large n is. While other CPUs
 * with a clock take turns with it, an instruction takes 1/N of the
 * clock's period, rounded down to a unit.
 */
void icount_set_cpu_period_frac(CPUState *cpu, uint64_t frac, uint64_t den);

/* if the CPUs are idle, start accounting real time to virtual clock. */
void icount_start_warp_timer(void);
void icount_account_warp_timer(void);
void icount_notify_exit(void);

#endif /* EXEC_ICOUNT_H */
