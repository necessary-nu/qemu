/*
 * Test privilege changes under icount on a hart without triggers
 *
 * With debug=false the hart has no trigger module, so changing privilege
 * mode under icount must not update instruction count triggers. Return to
 * U-mode with MRET and come back to M-mode with ECALL.
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

	/* Let U-mode access all of memory. */
	li	t0, -1
	csrw	pmpaddr0, t0
	li	t0, 0x1f	/* NAPOT, RWX */
	csrw	pmpcfg0, t0

	li	t0, 0x1800	/* MPP */
	csrc	mstatus, t0
	lla	t0, user
	csrw	mepc, t0
	mret

user:
	ecall
	j	fail

trap:
	csrr	t0, mcause
	li	t1, 8		/* Environment call from U-mode */
	bne	t0, t1, fail

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
