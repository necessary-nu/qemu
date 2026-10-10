/*
 * Test which mstatus fields are writable for the privilege modes present
 *
 * SIE, SPIE, SPP, SUM, MXR, TVM, TSR and SXL are read-only zero on a hart
 * without S-mode, and MPRV, TW and UXL on a hart without U-mode; otherwise
 * they are writable. MPP holds M-mode alone on a hart without U-mode.
 * misa says which modes the hart has.
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define MSTATUS_SIE	(1 << 1)
#define MSTATUS_SPIE	(1 << 5)
#define MSTATUS_SPP	(1 << 8)
#define MSTATUS_MPP	(3 << 11)
#define MSTATUS_MPRV	(1 << 17)
#define MSTATUS_SUM	(1 << 18)
#define MSTATUS_MXR	(1 << 19)
#define MSTATUS_TVM	(1 << 20)
#define MSTATUS_TW	(1 << 21)
#define MSTATUS_TSR	(1 << 22)

#define S_FIELDS	(MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_SPP | \
			 MSTATUS_SUM | MSTATUS_MXR | MSTATUS_TVM | \
			 MSTATUS_TSR)
#define U_FIELDS	(MSTATUS_MPRV | MSTATUS_TW)

#define MSTATUS_UXL_SHIFT	32
#define MSTATUS_SXL_SHIFT	34
#define MXL_RV64	2

#define MISA_S		(1 << 18)
#define MISA_U		(1 << 20)

	.option	norvc

	.text
	.global	_start
_start:
	lla	t0, fail
	csrw	mtvec, t0

	/* Without U-mode, MPP holds M-mode from reset on. */
	csrr	t0, misa
	li	t1, MISA_U
	and	t1, t0, t1
	bnez	t1, 0f
	li	t0, MSTATUS_MPP
	csrr	t1, mstatus
	and	t1, t1, t0
	bne	t1, t0, fail
	csrc	mstatus, t0
	csrr	t1, mstatus
	and	t1, t1, t0
	bne	t1, t0, fail
0:

	/* The fields that should stick, given the modes in misa. */
	li	s0, 0
	csrr	t0, misa
	li	t1, MISA_S
	and	t1, t0, t1
	beqz	t1, 1f
	li	t1, S_FIELDS
	or	s0, s0, t1
1:	li	t1, MISA_U
	and	t1, t0, t1
	beqz	t1, 2f
	li	t1, U_FIELDS
	or	s0, s0, t1
2:

	/*
	 * Set every field, read back which stuck and clear them again
	 * before any memory access, which MPRV would affect.
	 */
	li	t0, S_FIELDS | U_FIELDS
	csrs	mstatus, t0
	csrr	t1, mstatus
	csrc	mstatus, t0
	and	t1, t1, t0
	bne	t1, s0, fail

	csrr	t1, mstatus
	and	t1, t1, t0
	bnez	t1, fail

	/* SXL and UXL hold RV64 for each mode present, from reset on. */
	li	s0, 0
	csrr	t0, misa
	li	t1, MISA_S
	and	t1, t0, t1
	beqz	t1, 3f
	li	t1, MXL_RV64 << MSTATUS_SXL_SHIFT
	or	s0, s0, t1
3:	li	t1, MISA_U
	and	t1, t0, t1
	beqz	t1, 4f
	li	t1, MXL_RV64 << MSTATUS_UXL_SHIFT
	or	s0, s0, t1
4:
	li	t0, (3 << MSTATUS_SXL_SHIFT) | (3 << MSTATUS_UXL_SHIFT)
	csrr	t1, mstatus
	and	t1, t1, t0
	bne	t1, s0, fail
	li	t1, (MXL_RV64 << MSTATUS_SXL_SHIFT) | \
		    (MXL_RV64 << MSTATUS_UXL_SHIFT)
	csrs	mstatus, t1
	csrr	t1, mstatus
	and	t1, t1, t0
	bne	t1, s0, fail

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
