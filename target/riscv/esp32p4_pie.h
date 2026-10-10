/*
 * Espressif ESP32-P4 HP core: Processor Instruction Extensions (PIE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The PIE registers (TRM 4.4.1) and the operand descriptors the
 * translator passes to the PIE helpers in esp32p4_pie.c.
 */

#ifndef TARGET_RISCV_ESP32P4_PIE_H
#define TARGET_RISCV_ESP32P4_PIE_H

#include "hw/core/registerfields.h"

#define ESP32P4_PIE_QREGS 8

/*
 * Every vector is kept as 64-bit host-endian words holding its bits in
 * little-endian order: word 0 is bits 63:0, so that element i of width w
 * is bits w * i + w - 1 : w * i, as in the TRM's notation.
 */
typedef struct ESP32P4PIE {
    /* q0-q7 */
    uint64_t q[ESP32P4_PIE_QREGS][2];
    /*
     * QACC_L in words 0-3 and QACC_H in words 4-7, so that the 512 bits
     * are the sixteen 32-bit or eight 64-bit accumulators of the MAC
     * instructions in element order.
     */
    uint64_t qacc[8];
    /* UA_STATE, the unaligned data ESP.FFT.AMS.S16.LD.INCP.UAUP keeps. */
    uint64_t ua_state[2];
    /* XACC: 40 bits. */
    uint64_t xacc;
    /* SAR: 6 bits. */
    uint32_t sar;
    /* SAR_BYTE: 4 bits. */
    uint32_t sar_byte;
    /* FFT_BIT_WIDTH: 4 bits. */
    uint32_t fft_bit_width;
    /* CFG (Register 4.1) */
    uint32_t cfg;
    /* PERF, which ESP.MOVX.R.PERF and ESP.MOVX.W.PERF access. */
    uint32_t perf;
} ESP32P4PIE;

/* CFG */
#define ESP32P4_PIE_CFG_MIS_ST   BIT(0)
#define ESP32P4_PIE_CFG_MIS_LD   BIT(1)
#define ESP32P4_PIE_CFG_VXSAT    BIT(2)
#define ESP32P4_PIE_CFG_RM_EXC   BIT(3)
#define ESP32P4_PIE_CFG_VXRM_SHIFT 4
#define ESP32P4_PIE_CFG_VXRM     (0xf << ESP32P4_PIE_CFG_VXRM_SHIFT)
#define ESP32P4_PIE_CFG_VXSAT_EN BIT(8)
#define ESP32P4_PIE_CFG_WMASK    (ESP32P4_PIE_CFG_MIS_ST |     \
                                  ESP32P4_PIE_CFG_MIS_LD |     \
                                  ESP32P4_PIE_CFG_VXRM |       \
                                  ESP32P4_PIE_CFG_VXSAT_EN)

/* The rounding modes of an instruction's rm field and of CFG.vxrm. */
enum {
    ESP32P4_PIE_RM_FLOOR,
    ESP32P4_PIE_RM_CEILING,
    ESP32P4_PIE_RM_UP,
    ESP32P4_PIE_RM_DOWN,
    ESP32P4_PIE_RM_HALF_UP,
    ESP32P4_PIE_RM_HALF_DOWN,
    ESP32P4_PIE_RM_HALF_EVEN,
    ESP32P4_PIE_RM_DYNAMIC,
};

/*
 * The cause of the PIE illegal instruction exception: a PIE instruction
 * whose register or immediate fields are ambiguous or undefined. mtval
 * holds the instruction.
 */
#define ESP32P4_EXCP_PIE_ILLEGAL 0x1f

/*
 * The operand descriptor of a PIE helper: up to six Q registers, the
 * saturation and rounding mode fields and two selectors, and how the
 * instruction accesses memory, with the Q register it loads or stores.
 * Each helper documents which registers it takes in Q0 to Q4.
 */
FIELD(PIE, Q0, 0, 3)
FIELD(PIE, Q1, 3, 3)
FIELD(PIE, Q2, 6, 3)
FIELD(PIE, Q3, 9, 3)
FIELD(PIE, Q4, 12, 3)
FIELD(PIE, QU, 15, 3)
FIELD(PIE, SAT, 18, 1)
FIELD(PIE, RM, 19, 3)
FIELD(PIE, SEL, 22, 4)
FIELD(PIE, SELB, 26, 2)
FIELD(PIE, MEM, 28, 4)

/* PIE.MEM: the memory access of a fused instruction. */
enum {
    ESP32P4_PIE_MEM_NONE,
    /* Load 16 bytes into QU. */
    ESP32P4_PIE_MEM_LD128,
    /* Store QU's 16 bytes. */
    ESP32P4_PIE_MEM_ST128,
    /* Load a byte or a halfword and broadcast it to QU. */
    ESP32P4_PIE_MEM_LDBC8,
    ESP32P4_PIE_MEM_LDBC16,
};

/*
 * The operation of a helper serving several instructions: the element
 * width in bits 1:0 (8, 16, 32 or 64 bits), whether the elements are
 * signed in bit 2, and a helper-specific variant above.
 */
FIELD(PIEOP, ESZ, 0, 2)
FIELD(PIEOP, SIGNED, 2, 1)
FIELD(PIEOP, KIND, 3, 5)
FIELD(PIEOP, X, 8, 8)

/* PIEOP.KIND of helper_esp32p4_pie_mac() */
enum {
    PIE_MAC_QACC,       /* ESP.VMULAS.*.QACC */
    PIE_MAC_XACC,       /* ESP.VMULAS.*.XACC */
    PIE_MAC_SCALAR,     /* ESP.VSMULAS.*.QACC */
    PIE_MAC_CPLX_L,     /* ESP.VCMULAS.*.QACC.L */
    PIE_MAC_CPLX_H,     /* ESP.VCMULAS.*.QACC.H */
};

/* PIEOP.KIND of helper_esp32p4_pie_vmul() */
enum {
    PIE_VMUL_SAME,      /* ESP.VMUL.S8, U8, S16 and U16 */
    PIE_VMUL_S8XS8,     /* ESP.VMUL.S16.S8XS8 */
    PIE_VMUL_S16XS16,   /* ESP.VMUL.S32.S16XS16 */
};

/* PIEOP.KIND of helper_esp32p4_pie_perm() */
enum {
    PIE_PERM_ZIP,
    PIE_PERM_UNZIP,
    PIE_PERM_ZIPT,
    PIE_PERM_UNZIPT,
    PIE_PERM_EXT,
};

/* PIEOP.KIND of the ESP.MOVX helpers: the special register. */
enum {
    PIE_SR_CFG,
    PIE_SR_FFT_BIT_WIDTH,
    PIE_SR_PERF,
    PIE_SR_SAR,
    PIE_SR_SAR_BYTE,
    PIE_SR_XACC_H,
    PIE_SR_XACC_L,
};

/* PIEOP.KIND of helper_esp32p4_pie_load(), and what each loads. */
enum {
    PIE_LD_Q,           /* ESP.VLD.128: QU */
    PIE_LD_H64,         /* ESP.VLD.H.64: QU's high half */
    PIE_LD_L64,         /* ESP.VLD.L.64: QU's low half */
    PIE_LD_BC,          /* ESP.VLDBC.*: an element broadcast to QU */
    PIE_LD_EXT,         /* ESP.VLDEXT.*: QU, Q0 = qz */
    PIE_LD_HBC,         /* ESP.VLDHBC.16.INCP: QU, Q0 = qz */
    PIE_LD_USAR,        /* ESP.LD.128.USAR.*: QU and SAR_BYTE */
    PIE_LD_QA,          /* ESP.LDQA.*: QACC */
    PIE_LD_QACC,        /* ESP.LD.QACC.*: a quarter of QACC, PIEOP.X */
    PIE_LD_XACC,        /* ESP.LD.XACC.IP */
    PIE_LD_UA_STATE,    /* ESP.LD.UA.STATE.IP */
};

/* PIEOP.KIND of helper_esp32p4_pie_store(), and what each stores. */
enum {
    PIE_ST_Q,           /* ESP.VST.128: QU */
    PIE_ST_H64,         /* ESP.VST.H.64: QU's high half */
    PIE_ST_L64,         /* ESP.VST.L.64: QU's low half */
    PIE_ST_QACC,        /* ESP.ST.QACC.*: a quarter of QACC, PIEOP.X */
    PIE_ST_XACC,        /* ESP.ST.S/U.XACC.IP: XACC extended to 64 bits */
    PIE_ST_UA_STATE,    /* ESP.ST.UA.STATE.IP */
};

/* PIEOP.KIND of helper_esp32p4_pie_src() */
enum {
    PIE_SRC_Q,          /* ESP.SRC.Q: qz */
    PIE_SRC_Q_QUP,      /* ESP.SRC.Q.QUP: qz, and qw = qy */
    PIE_SRC_Q_LD,       /* ESP.SRC.Q.LD.*: qw, loading QU */
    PIE_SRC_SLC,        /* ESP.SLCI.2Q, ESP.SLCXXP.2Q */
    PIE_SRC_SRC,        /* ESP.SRCI.2Q, ESP.SRCXXP.2Q */
    PIE_SRC_ST,         /* ESP.SRCQ.128.ST.INCP: stored at addr */
};

/* PIEOP.KIND of helper_esp32p4_pie_vshift() */
enum {
    PIE_VSHIFT_SR,      /* ESP.VSR.S32 and U32 */
    PIE_VSHIFT_SL,      /* ESP.VSL.S32 and U32 */
    PIE_VSHIFT_SLD,     /* ESP.VSLD.* */
    PIE_VSHIFT_SRD,     /* ESP.VSRD.* */
};

/*
 * PIEOP.KIND of helper_esp32p4_pie_fft_ams(): the four forms of
 * ESP.FFT.AMS.S16, each working on one pair of elements.
 */
enum {
    PIE_AMS_LD_INCP_UAUP,   /* elements 0-1 */
    PIE_AMS_LD_INCP,        /* elements 2-3 */
    PIE_AMS_LD_R32_DECP,    /* elements 4-5 */
    PIE_AMS_ST_INCP,        /* elements 6-7 */
};

#endif
