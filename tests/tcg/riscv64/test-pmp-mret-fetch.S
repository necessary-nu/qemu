/*
 * Test MRET to U-mode while no PMP entry is set
 *
 * No PMP entry matches the U-mode fetch at the return address, so that
 * fetch raises an instruction access fault: MRET itself completes, and
 * the trap is taken from U-mode with mepc and mtval at the return address.
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

	.option	norvc

	.text
	.global	_start
_start:
	lla	t0, trap
	csrw	mtvec, t0

	li	t0, 0x1800	/* MPP */
	csrc	mstatus, t0
	lla	t0, user
	csrw	mepc, t0
	mret

user:
	j	fail

trap:
	csrr	t0, mcause
	li	t1, 1		/* Instruction access fault */
	bne	t0, t1, fail
	lla	t1, user
	csrr	t0, mepc
	bne	t0, t1, fail
	csrr	t0, mtval
	bne	t0, t1, fail
	/* Taken from U-mode, so MPP is U. */
	csrr	t0, mstatus
	li	t1, 0x1800	/* MPP */
	and	t0, t0, t1
	bnez	t0, fail

	li	a0, 0
	j	_exit

fail:
	li	a0, 1

_exit:
	lla	a1, semiargs
	li	t0, 0x20026	/* ADP_Stopped_ApplicationExit */
	sd	t0, 0(a1)
	sd	a0, 8(a1)
	li	a0, 0x20	/* TARGET_SYS_EXIT_EXTENDED */

	/* Semihosting call sequence */
	.balign	16
	slli	zero, zero, 0x1f
	ebreak
	srai	zero, zero, 0x7
	j	.

	.data
	.balign	16
semiargs:
	.space	16
