/*
 * Test U-mode accesses on a hart with PMP but no PMP entry implemented
 *
 * An S-mode or U-mode access that no PMP entry matches fails only if at
 * least one entry is implemented: with none, U-mode fetches, loads and
 * stores all succeed.
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
	lla	t0, data
	lw	t1, 0(t0)
	sw	t1, 0(t0)
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
data:
	.word	0
