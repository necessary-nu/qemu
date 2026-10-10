/*
 * Test that SFENCE.VMA is illegal on a hart without S-mode
 *
 * SFENCE.VMA belongs to the supervisor instruction set: a hart that has
 * no S-mode does not implement it, so it raises an illegal-instruction
 * exception even in M-mode.
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

sfence:
	sfence.vma
	j	fail

trap:
	csrr	t0, mcause
	li	t1, 2		/* Illegal instruction */
	bne	t0, t1, fail
	csrr	t0, mepc
	lla	t1, sfence
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
