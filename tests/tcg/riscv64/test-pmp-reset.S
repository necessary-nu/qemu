/*
 * Test that a reset unlocks and turns off every PMP entry
 *
 * Lock entry 2 alone, so that M-mode may no longer read a region of RAM,
 * and reset the machine through virt's test finisher. After the reset the
 * entry must be off and unlocked, and the region readable. A word of RAM
 * outside the image, which the reset keeps, tells the two boots apart.
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define MARKER		0x80400000
#define MAGIC		0x52534554
#define REGION		0x80600000	/* 4 KiB */
#define FINISHER	0x100000
#define FINISHER_RESET	0x7777

	.option	norvc

	.text
	.global	_start
_start:
	lla	t0, fail
	csrw	mtvec, t0

	li	s0, MARKER
	lw	t0, 0(s0)
	li	t1, MAGIC
	beq	t0, t1, after_reset
	sw	t1, 0(s0)

	li	t0, (REGION >> 2) | 0x1ff
	csrw	pmpaddr2, t0
	li	t0, 0x98 << 16	/* Entry 2: L, NAPOT, no access */
	csrw	pmpcfg0, t0

	li	t0, FINISHER
	li	t1, FINISHER_RESET
	sw	t1, 0(t0)
1:	j	1b

after_reset:
	sw	zero, 0(s0)

	csrr	t0, pmpcfg0
	bnez	t0, fail
	li	t0, REGION
	lw	t0, 0(t0)

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
