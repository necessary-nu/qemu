/*
 * Test that a TOR PMP entry whose top is not above its bottom matches
 * nothing
 *
 * An entry that matched an access in part would make it fail, so check
 * the empty entries against CBO.CLEAN of the cache block at address 0,
 * which is checked like a load but reaches no memory, so that it succeeds
 * in M-mode wherever no entry matches. Finally lock an entry that does
 * match it, to check that the access is checked at all.
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define PMP_TOR		0x08
#define PMP_L		0x80

	.option	norvc

	.text
	.global	_start
_start:
	lla	t0, trap
	csrw	mtvec, t0
	li	s0, 0		/* A trap is expected */

	/* Entry 0 in TOR mode with pmpaddr0 = 0: [0, 0). */
	csrw	pmpaddr0, zero
	li	t0, PMP_TOR
	csrw	pmpcfg0, t0
	cbo.clean	(zero)

	/* Entry 2 in TOR mode with pmpaddr1 > pmpaddr2, entry 1 off. */
	csrw	pmpcfg0, zero
	li	t0, 0x2000 >> 2
	csrw	pmpaddr1, t0
	li	t0, 0x1000 >> 2
	csrw	pmpaddr2, t0
	li	t0, PMP_TOR << 16
	csrw	pmpcfg0, t0
	cbo.clean	(zero)

	/* Entry 0 locked in TOR mode over [0, 64), allowing nothing. */
	csrw	pmpcfg0, zero
	li	t0, 64 >> 2
	csrw	pmpaddr0, t0
	li	t0, PMP_L | PMP_TOR
	li	s0, 1
	csrw	pmpcfg0, t0
	cbo.clean	(zero)
	j	fail

trap:
	beqz	s0, fail
	csrr	t0, mcause
	li	t1, 7		/* Store/AMO access fault */
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
