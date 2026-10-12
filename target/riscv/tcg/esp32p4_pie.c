/*
 * Espressif ESP32-P4 HP core: Processor Instruction Extensions (PIE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 4
 * (Processor Instruction Extensions, Xespv2p2) and 2.7.2 (the extension's
 * state, mext_pie_status).
 *
 * The helpers that carry out the PIE instructions, which
 * insn_trans/trans_esp32p4_pie.c.inc decodes. A helper reads its
 * operands, computes every result, then makes the instruction's memory
 * access, and only then writes the registers, so that an access that
 * faults leaves the core as it was. Loads and stores go through the
 * core's MMU index, so that the PMP and the triggers see them. Memory is
 * little-endian.
 *
 * Arithmetic follows the TRM's operation listings: intermediate results
 * are exact, a right shift rounds as the instruction's rm field (or, when
 * it is DYNAMIC, CFG.vxrm) says, and a result that does not fit its
 * element saturates when the instruction's sat field is set and keeps its
 * low bits when it is clear. The listings only show the saturating form;
 * keeping the low bits is what the TRM says of overflow in general (4.4.4)
 * and is the "trunc" form Espressif's assembler emits for sat clear.
 *
 * Where the listings contradict each other or the rest of the chapter,
 * these helpers take the reading noted at the function concerned.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/int128.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "target/riscv/esp32p4_cpu.h"

typedef struct PieVec {
    uint64_t w[2];
} PieVec;

static ESP32P4PIE *pie_of(CPURISCVState *env)
{
    return &esp32p4_cpu_from_env(env)->pie;
}

/* Element widths in bits, by PIEOP.ESZ. */
static int pie_esz(uint32_t op)
{
    return 8 << FIELD_EX32(op, PIEOP, ESZ);
}

static bool pie_signed(uint32_t op)
{
    return FIELD_EX32(op, PIEOP, SIGNED);
}

static uint64_t elt_u(const uint64_t *v, int bits, int i)
{
    int pos = bits * i;

    if (bits == 64) {
        return v[i];
    }
    return extract64(v[pos / 64], pos % 64, bits);
}

static int64_t elt_s(const uint64_t *v, int bits, int i)
{
    int pos = bits * i;

    if (bits == 64) {
        return v[i];
    }
    return sextract64(v[pos / 64], pos % 64, bits);
}

static int64_t elt(const uint64_t *v, int bits, int i, bool is_signed)
{
    return is_signed ? elt_s(v, bits, i) : (int64_t)elt_u(v, bits, i);
}

static void elt_set(uint64_t *v, int bits, int i, uint64_t x)
{
    int pos = bits * i;

    if (bits == 64) {
        v[i] = x;
        return;
    }
    v[pos / 64] = deposit64(v[pos / 64], pos % 64, bits, x);
}

static PieVec pie_q(ESP32P4PIE *pie, int r)
{
    PieVec v = { { pie->q[r][0], pie->q[r][1] } };

    return v;
}

static void pie_set_q(ESP32P4PIE *pie, int r, const PieVec *v)
{
    pie->q[r][0] = v->w[0];
    pie->q[r][1] = v->w[1];
}

/* Saturation and rounding */

/*
 * CFG.vxsat records that a result saturated, while CFG.vxsat_en is set;
 * it stays set until CFG is read.
 */
static void pie_saturated(ESP32P4PIE *pie)
{
    if (pie->cfg & ESP32P4_PIE_CFG_VXSAT_EN) {
        pie->cfg |= ESP32P4_PIE_CFG_VXSAT;
    }
}

/*
 * A signed result for an element of the given width: clamped to the
 * element's range when sat is set, otherwise its low bits.
 */
static int64_t pie_fit_s(ESP32P4PIE *pie, int64_t v, int bits, bool sat)
{
    int64_t max, min;

    if (bits == 64) {
        return v;
    }
    if (!sat) {
        return sextract64(v, 0, bits);
    }
    max = (INT64_C(1) << (bits - 1)) - 1;
    min = -max - 1;
    if (v > max) {
        pie_saturated(pie);
        return max;
    }
    if (v < min) {
        pie_saturated(pie);
        return min;
    }
    return v;
}

/*
 * An unsigned result for an element of the given width, from an exact
 * signed intermediate: clamped to 0 and the element's maximum when sat is
 * set, otherwise its low bits.
 */
static uint64_t pie_fit_u(ESP32P4PIE *pie, int64_t v, int bits, bool sat)
{
    uint64_t max;

    if (!sat) {
        return extract64(v, 0, bits);
    }
    max = MAKE_64BIT_MASK(0, bits);
    if (v < 0) {
        pie_saturated(pie);
        return 0;
    }
    if ((uint64_t)v > max) {
        pie_saturated(pie);
        return max;
    }
    return v;
}

static uint64_t pie_fit(ESP32P4PIE *pie, int64_t v, int bits, bool is_signed,
                        bool sat)
{
    return is_signed ? pie_fit_s(pie, v, bits, sat)
                     : pie_fit_u(pie, v, bits, sat);
}

/*
 * The rounding mode an rm field selects: DYNAMIC takes CFG.vxrm, and
 * vxrm values from DYNAMIC up select UNNECESSARY, which is returned as
 * ESP32P4_PIE_RM_DYNAMIC.
 */
static unsigned pie_rmode(ESP32P4PIE *pie, unsigned rm)
{
    if (rm == ESP32P4_PIE_RM_DYNAMIC) {
        rm = extract32(pie->cfg, ESP32P4_PIE_CFG_VXRM_SHIFT, 4);
        if (rm > ESP32P4_PIE_RM_DYNAMIC) {
            rm = ESP32P4_PIE_RM_DYNAMIC;
        }
    }
    return rm;
}

/*
 * The quotient of an exact value by 2^s rounded as Table 4.4-3 says, from
 * its floor q, the remainder r of that floor and the value's sign:
 * FLOOR and CEILING towards minus and plus infinity, UP and DOWN away
 * from and towards zero, HALF_UP and HALF_DOWN to the nearest with ties
 * away from and towards zero, HALF_EVEN with ties to the even neighbour,
 * and UNNECESSARY as FLOOR, recording an inexact result in CFG.rm_exc.
 */
static int64_t pie_round(ESP32P4PIE *pie, int64_t q, uint64_t r, unsigned s,
                         bool neg, unsigned rm)
{
    uint64_t half = UINT64_C(1) << (s - 1);
    bool inc;

    if (!r) {
        return q;
    }
    switch (pie_rmode(pie, rm)) {
    case ESP32P4_PIE_RM_FLOOR:
        inc = false;
        break;
    case ESP32P4_PIE_RM_CEILING:
        inc = true;
        break;
    case ESP32P4_PIE_RM_UP:
        inc = !neg;
        break;
    case ESP32P4_PIE_RM_DOWN:
        inc = neg;
        break;
    case ESP32P4_PIE_RM_HALF_UP:
        inc = r > half || (r == half && !neg);
        break;
    case ESP32P4_PIE_RM_HALF_DOWN:
        inc = r > half || (r == half && neg);
        break;
    case ESP32P4_PIE_RM_HALF_EVEN:
        inc = r > half || (r == half && (q & 1));
        break;
    default:
        pie->cfg |= ESP32P4_PIE_CFG_RM_EXC;
        inc = false;
        break;
    }
    return q + inc;
}

/* Arithmetic right shift of a signed value by 0 to 63, rounded. */
static int64_t pie_sra(ESP32P4PIE *pie, int64_t v, unsigned s, unsigned rm)
{
    if (s == 0) {
        return v;
    }
    return pie_round(pie, v >> s, v & MAKE_64BIT_MASK(0, s), s, v < 0, rm);
}

/* Logical right shift of an unsigned value by 0 to 63, rounded. */
static uint64_t pie_srl(ESP32P4PIE *pie, uint64_t v, unsigned s, unsigned rm)
{
    if (s == 0) {
        return v;
    }
    return pie_round(pie, v >> s, v & MAKE_64BIT_MASK(0, s), s, false, rm);
}

/* Memory */

/*
 * The address of an access of the given size: with CFG.mis_ld or mis_st
 * clear, the force-alignment mode clears the address's low bits; with it
 * set, the access is made at the address, split as the hardware does.
 */
static target_ulong pie_ea(ESP32P4PIE *pie, target_ulong addr, unsigned size,
                           bool store)
{
    uint32_t mis = store ? ESP32P4_PIE_CFG_MIS_ST : ESP32P4_PIE_CFG_MIS_LD;

    if (!(pie->cfg & mis)) {
        addr &= ~(target_ulong)(size - 1);
    }
    return addr;
}

static int pie_mmu(CPURISCVState *env)
{
    return riscv_env_mmu_index(env, false);
}

static void pie_ld128(CPURISCVState *env, target_ulong addr, PieVec *v,
                      uintptr_t ra)
{
    MemOpIdx oi = make_memop_idx(MO_LE | MO_128 | MO_ATOM_NONE, pie_mmu(env));
    Int128 d = cpu_ld16_mmu(env, addr, oi, ra);

    v->w[0] = int128_getlo(d);
    v->w[1] = int128_gethi(d);
}

static void pie_st128(CPURISCVState *env, target_ulong addr, const PieVec *v,
                      uintptr_t ra)
{
    MemOpIdx oi = make_memop_idx(MO_LE | MO_128 | MO_ATOM_NONE, pie_mmu(env));

    cpu_st16_mmu(env, addr, int128_make128(v->w[0], v->w[1]), oi, ra);
}

/*
 * The instruction has stored, its only store: a store instruction for
 * mhpmcounter13 (esp32p4_counters.c).
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static void pie_stored(CPURISCVState *env)
{
    esp32p4_cpu_from_env(env)->ev[ESP32P4_EV_STORE]++;
}

/* A 16-byte load or store at a base address, aligned as CFG says. */
static void pie_load_q(CPURISCVState *env, target_ulong base, PieVec *v,
                       uintptr_t ra)
{
    pie_ld128(env, pie_ea(pie_of(env), base, 16, false), v, ra);
}

static void pie_store_q(CPURISCVState *env, target_ulong base,
                        const PieVec *v, uintptr_t ra)
{
    pie_st128(env, pie_ea(pie_of(env), base, 16, true), v, ra);
    pie_stored(env);
}

/*
 * The memory half of a fused instruction, PIE.MEM: store QU, or load the
 * value it is to receive into *ld for pie_mem_commit().
 */
static void pie_mem(CPURISCVState *env, uint32_t desc, target_ulong addr,
                    PieVec *ld, uintptr_t ra)
{
    ESP32P4PIE *pie = pie_of(env);
    PieVec st = { };
    uint64_t x;

    switch (FIELD_EX32(desc, PIE, MEM)) {
    case ESP32P4_PIE_MEM_LD128:
        pie_load_q(env, addr, ld, ra);
        break;
    case ESP32P4_PIE_MEM_ST128:
        st = pie_q(pie, FIELD_EX32(desc, PIE, QU));
        pie_store_q(env, addr, &st, ra);
        break;
    case ESP32P4_PIE_MEM_LDBC8:
        x = cpu_ldub_mmuidx_ra(env, addr, pie_mmu(env), ra);
        ld->w[0] = ld->w[1] = x * UINT64_C(0x0101010101010101);
        break;
    case ESP32P4_PIE_MEM_LDBC16:
        x = cpu_lduw_le_mmuidx_ra(env, pie_ea(pie, addr, 2, false),
                                  pie_mmu(env), ra);
        ld->w[0] = ld->w[1] = x * UINT64_C(0x0001000100010001);
        break;
    default:
        break;
    }
}

static void pie_mem_commit(CPURISCVState *env, uint32_t desc,
                           const PieVec *ld)
{
    switch (FIELD_EX32(desc, PIE, MEM)) {
    case ESP32P4_PIE_MEM_LD128:
    case ESP32P4_PIE_MEM_LDBC8:
    case ESP32P4_PIE_MEM_LDBC16:
        pie_set_q(pie_of(env), FIELD_EX32(desc, PIE, QU), ld);
        break;
    default:
        break;
    }
}

/* Multiply-accumulate */

/*
 * One accumulator of the given width plus an exact product, saturated or
 * wrapped. A signed accumulator saturates to its signed range; an
 * unsigned one, which only grows, to its unsigned maximum.
 */
static uint64_t pie_acc(ESP32P4PIE *pie, uint64_t acc, int64_t prod, int bits,
                        bool is_signed, bool sat)
{
    if (bits == 64) {
        if (is_signed) {
            int64_t r;

            if (__builtin_add_overflow((int64_t)acc, prod, &r) && sat) {
                pie_saturated(pie);
                return prod < 0 ? INT64_MIN : INT64_MAX;
            }
            return r;
        } else {
            uint64_t r;

            if (__builtin_add_overflow(acc, (uint64_t)prod, &r) && sat) {
                pie_saturated(pie);
                return UINT64_MAX;
            }
            return r;
        }
    }
    if (is_signed) {
        return pie_fit_s(pie, sextract64(acc, 0, bits) + prod, bits, sat);
    }
    return pie_fit_u(pie, extract64(acc, 0, bits) + prod, bits, sat);
}

/* XACC plus an exact sum, saturated to 40 bits or wrapped. */
static void pie_xacc_add(ESP32P4PIE *pie, int64_t sum, bool is_signed,
                         bool sat)
{
    if (is_signed) {
        pie->xacc = extract64(pie_fit_s(pie, sextract64(pie->xacc, 0, 40) +
                                        sum, 40, sat), 0, 40);
    } else {
        pie->xacc = pie_fit_u(pie, extract64(pie->xacc, 0, 40) + sum, 40,
                              sat);
    }
}

/*
 * ESP.VMULAS.*, ESP.VSMULAS.* and ESP.VCMULAS.*: Q0 = qx, Q1 = qy, SEL the
 * element of qy that ESP.VSMULAS multiplies every element of qx by.
 *
 * Each 8-bit product accumulates into a 32-bit accumulator of QACC and
 * each 16-bit one into a 64-bit accumulator, element i into accumulator
 * i, QACC_L before QACC_H; ESP.VMULAS.*.XACC adds all products to XACC.
 *
 * The TRM's listings give the unsigned QACC forms an upper bound of
 * 2^31 - 1 or 2^63 - 1 in ESP.VMULAS.U8/U16.QACC and its .LD/.ST forms,
 * but 2^32 - 1 and 2^64 - 1 in the .LDBC.INCP forms and ESP.VSMULAS, and
 * describe all 8-bit ones as saturating to an unsigned number; as one
 * accumulator datapath serves them all, all saturate to the unsigned
 * maximum.
 *
 * ESP.VCMULAS multiplies the complex numbers of qx and qy, the real part
 * in each even element and the imaginary part in the odd one after it,
 * and accumulates the real part of each product into an even accumulator
 * and the imaginary part into the odd one. The TRM's listings subtract
 * in every imaginary part but the first of the 16-bit forms, which gives
 * the complex product's imaginary part as the sum; the sum, the complex
 * product ESP.CMUL computes, is taken throughout.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_mac(CPURISCVState *env, uint32_t desc, uint32_t op,
                            target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    int ab = eb * 4;
    int n = 128 / eb;
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned kind = FIELD_EX32(op, PIEOP, KIND);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q0));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    uint64_t acc[8];
    uint64_t saved_xacc = pie->xacc;
    uint32_t saved_cfg = pie->cfg;
    PieVec ld = { };
    int64_t sum = 0;

    memcpy(acc, pie->qacc, sizeof(acc));
    switch (kind) {
    case PIE_MAC_CPLX_L:
    case PIE_MAC_CPLX_H: {
        int base = kind == PIE_MAC_CPLX_H ? n / 2 : 0;

        for (int k = base; k < base + n / 2; k += 2) {
            int64_t a = elt_s(x.w, eb, k), b = elt_s(x.w, eb, k + 1);
            int64_t c = elt_s(y.w, eb, k), d = elt_s(y.w, eb, k + 1);

            elt_set(acc, ab, k, pie_acc(pie, elt_u(acc, ab, k),
                                        a * c - b * d, ab, true, sat));
            elt_set(acc, ab, k + 1, pie_acc(pie, elt_u(acc, ab, k + 1),
                                            b * c + a * d, ab, true, sat));
        }
        break;
    }
    default:
        for (int i = 0; i < n; i++) {
            int j = kind == PIE_MAC_SCALAR ? FIELD_EX32(desc, PIE, SEL) : i;
            int64_t prod = elt(x.w, eb, i, sg) * elt(y.w, eb, j, sg);

            if (kind == PIE_MAC_XACC) {
                sum += prod;
            } else {
                elt_set(acc, ab, i, pie_acc(pie, elt_u(acc, ab, i), prod, ab,
                                            sg, sat));
            }
        }
        if (kind == PIE_MAC_XACC) {
            pie_xacc_add(pie, sum, sg, sat);
        }
        break;
    }

    /* A faulting access must leave XACC and CFG as they were too. */
    if (FIELD_EX32(desc, PIE, MEM) != ESP32P4_PIE_MEM_NONE) {
        uint64_t new_xacc = pie->xacc;
        uint32_t new_cfg = pie->cfg;

        pie->xacc = saved_xacc;
        pie->cfg = saved_cfg;
        pie_mem(env, desc, addr, &ld, GETPC());
        pie->xacc = new_xacc;
        pie->cfg = new_cfg;
    }
    if (kind != PIE_MAC_XACC) {
        memcpy(pie->qacc, acc, sizeof(acc));
    }
    pie_mem_commit(env, desc, &ld);
}

/* Element-wise arithmetic */

/*
 * The computation of a fused instruction runs on copies, so that its
 * memory access may fault before anything changes: snapshot and restore
 * the CFG flags the computation may have set.
 */
static void pie_mem_then_commit(CPURISCVState *env, uint32_t desc,
                                target_ulong addr, uint32_t cfg_before,
                                int qd, const PieVec *res, uintptr_t ra)
{
    ESP32P4PIE *pie = pie_of(env);
    uint32_t cfg_after = pie->cfg;
    PieVec ld = { };

    if (FIELD_EX32(desc, PIE, MEM) != ESP32P4_PIE_MEM_NONE) {
        pie->cfg = cfg_before;
        pie_mem(env, desc, addr, &ld, ra);
        pie->cfg = cfg_after;
    }
    if (qd >= 0) {
        pie_set_q(pie, qd, res);
    }
    pie_mem_commit(env, desc, &ld);
}

/*
 * ESP.VADD.* and ESP.VSUB.*: Q0 = qv, Q1 = qx, Q2 = qy; KIND 1 subtracts.
 * Unsigned results saturate at zero and at the element's maximum.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vaddsub(CPURISCVState *env, uint32_t desc,
                                uint32_t op, target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    uint32_t cfg = pie->cfg;
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    bool sub = FIELD_EX32(op, PIEOP, KIND);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t a = elt(x.w, eb, i, sg), b = elt(y.w, eb, i, sg);

        elt_set(r.w, eb, i, pie_fit(pie, sub ? a - b : a + b, eb, sg, sat));
    }
    pie_mem_then_commit(env, desc, addr, cfg, FIELD_EX32(desc, PIE, Q0), &r,
                        GETPC());
}

/*
 * ESP.VSADDS.* and ESP.VSSUBS.*: Q0 = qv, Q1 = qx, adding or subtracting
 * the element-wide low bits of rs1, signed or not as the instruction is.
 * The TRM's listings of ESP.VSSUBS.U8 and U16 only bound the result above;
 * as subtractions of unsigned saturation they also saturate at zero.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vsaddsub(CPURISCVState *env, uint32_t desc,
                                 uint32_t op, target_ulong val)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    bool sub = FIELD_EX32(op, PIEOP, KIND);
    int64_t b = sg ? sextract64(val, 0, eb) : extract64(val, 0, eb);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t a = elt(x.w, eb, i, sg);

        elt_set(r.w, eb, i, pie_fit(pie, sub ? a - b : a + b, eb, sg, sat));
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/*
 * ESP.VMUL.*: Q0 = qz, Q1 = qx, Q2 = qy, and for the widening forms
 * Q3 = qv. ESP.VMUL.S8, U8, S16 and U16 multiply elements into elements of
 * the same width, shifting each product right by SAR (its low five bits
 * for ESP.VMUL.S8, as the TRM's listing has it, all six otherwise;
 * PIEOP.X holds the mask). ESP.VMUL.S16.S8XS8 and ESP.VMUL.S32.S16XS16
 * keep twice the width, the products of the low half of the elements in
 * qz and of the high half in qv; their products always fit.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vmul(CPURISCVState *env, uint32_t desc, uint32_t op,
                             target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    uint32_t cfg = pie->cfg;
    int eb = pie_esz(op);
    int n = 128 / eb;
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    unsigned kind = FIELD_EX32(op, PIEOP, KIND);
    unsigned s = pie->sar & FIELD_EX32(op, PIEOP, X);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = { }, r2 = { };

    if (kind == PIE_VMUL_SAME) {
        for (int i = 0; i < n; i++) {
            int64_t p = elt(x.w, eb, i, sg) * elt(y.w, eb, i, sg);

            p = sg ? pie_sra(pie, p, s, rm) : pie_srl(pie, p, s, rm);
            elt_set(r.w, eb, i, pie_fit(pie, p, eb, sg, sat));
        }
        pie_mem_then_commit(env, desc, addr, cfg, FIELD_EX32(desc, PIE, Q0),
                            &r, GETPC());
        return;
    }
    for (int i = 0; i < n; i++) {
        int64_t p = pie_sra(pie, elt_s(x.w, eb, i) * elt_s(y.w, eb, i), s,
                            rm);

        elt_set(i < n / 2 ? r.w : r2.w, eb * 2, i % (n / 2), p);
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q3), &r2);
}

/*
 * ESP.CMUL.*: Q0 = qz, Q1 = qx, Q2 = qy, SEL = sel4. Complex numbers sit
 * in element pairs, the real part in the even element; sel4 bit 0 picks
 * the low or the high half of the registers, the only half of qz written,
 * and sel4 bit 1 multiplies qy by qx's conjugate rather than by qx. Each
 * part is shifted right by SAR and saturated or wrapped to the element; an
 * unsigned part that comes out negative saturates at zero.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_cmul(CPURISCVState *env, uint32_t desc, uint32_t op,
                             target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    uint32_t cfg = pie->cfg;
    int eb = pie_esz(op);
    int n = 128 / eb;
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    unsigned sel = FIELD_EX32(desc, PIE, SEL);
    int qz = FIELD_EX32(desc, PIE, Q0);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = pie_q(pie, qz);
    int base = (sel & 1) ? n / 2 : 0;

    for (int k = base; k < base + n / 2; k += 2) {
        int64_t a = elt(x.w, eb, k, sg), b = elt(x.w, eb, k + 1, sg);
        int64_t c = elt(y.w, eb, k, sg), d = elt(y.w, eb, k + 1, sg);
        int64_t re, im;

        if (sel & 2) {
            re = a * c + b * d;
            im = a * d - b * c;
        } else {
            re = a * c - b * d;
            im = a * d + b * c;
        }
        re = pie_sra(pie, re, pie->sar, rm);
        im = pie_sra(pie, im, pie->sar, rm);
        elt_set(r.w, eb, k, pie_fit(pie, re, eb, sg, sat));
        elt_set(r.w, eb, k + 1, pie_fit(pie, im, eb, sg, sat));
    }
    pie_mem_then_commit(env, desc, addr, cfg, qz, &r, GETPC());
}

/* ESP.VMAX.* and ESP.VMIN.*: Q0 = qz, Q1 = qx, Q2 = qy; KIND 1 is VMIN. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vminmax(CPURISCVState *env, uint32_t desc,
                                uint32_t op, target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    bool min = FIELD_EX32(op, PIEOP, KIND);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t a = elt(x.w, eb, i, sg), b = elt(y.w, eb, i, sg);

        elt_set(r.w, eb, i, min ? MIN(a, b) : MAX(a, b));
    }
    pie_mem_then_commit(env, desc, addr, pie->cfg, FIELD_EX32(desc, PIE, Q0),
                        &r, GETPC());
}

/* ESP.VCMP.*: Q0 = qz, Q1 = qx, Q2 = qy; KIND 0 EQ, 1 LT, 2 GT. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vcmp(CPURISCVState *env, uint32_t desc, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    unsigned kind = FIELD_EX32(op, PIEOP, KIND);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t a = elt(x.w, eb, i, sg), b = elt(y.w, eb, i, sg);
        bool t = kind == 0 ? a == b : kind == 1 ? a < b : a > b;

        elt_set(r.w, eb, i, t ? UINT64_MAX : 0);
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/*
 * ESP.MAX.*.A and ESP.MIN.*.A: the largest or (KIND 1) smallest element of
 * Q0 = qw, sign- or zero-extended to 32 bits.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
target_ulong helper_esp32p4_pie_minmax_a(CPURISCVState *env, uint32_t desc,
                                         uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    bool min = FIELD_EX32(op, PIEOP, KIND);
    PieVec w = pie_q(pie, FIELD_EX32(desc, PIE, Q0));
    int64_t m = elt(w.w, eb, 0, sg);

    for (int i = 1; i < 128 / eb; i++) {
        int64_t v = elt(w.w, eb, i, sg);

        m = min ? MIN(m, v) : MAX(m, v);
    }
    return (int32_t)m;
}

/*
 * ESP.VABS.*: Q0 = qv, Q1 = qy. The most negative element has no
 * positive counterpart and stays as it is.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vabs(CPURISCVState *env, uint32_t desc, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t v = elt_s(y.w, eb, i);

        elt_set(r.w, eb, i, v < 0 ? -v : v);
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/*
 * ESP.VCLAMP.S16: Q0 = qz, Q1 = qx, each element clamped to
 * [-2^sel16, 2^sel16 - 1].
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vclamp(CPURISCVState *env, uint32_t desc)
{
    ESP32P4PIE *pie = pie_of(env);
    int64_t hi = (INT64_C(1) << FIELD_EX32(desc, PIE, SEL)) - 1;
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec r = { };

    for (int i = 0; i < 8; i++) {
        elt_set(r.w, 16, i, MIN(MAX(elt_s(x.w, 16, i), -hi - 1), hi));
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/*
 * ESP.VSAT.*: Q0 = qz, Q1 = qx, each element clamped between the
 * element-wide low bits of rs1 and rs2, compared signed or not as the
 * instruction is. The TRM's listing names the larger bound min_t and the
 * smaller max_t, which as written would always give the larger; the
 * description, saturating the elements, is taken.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vsat(CPURISCVState *env, uint32_t desc, uint32_t op,
                             target_ulong rs1, target_ulong rs2)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    int64_t a = sg ? sextract64(rs1, 0, eb) : extract64(rs1, 0, eb);
    int64_t b = sg ? sextract64(rs2, 0, eb) : extract64(rs2, 0, eb);
    int64_t lo = MIN(a, b), hi = MAX(a, b);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        elt_set(r.w, eb, i, MAX(MIN(elt(x.w, eb, i, sg), hi), lo));
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/*
 * ESP.VRELU.*: Q0 = qy, whose elements not above zero are multiplied by
 * the element-wide low bits of rs2 (val) and shifted right by the low
 * five (8-bit) or six (16-bit) bits of rs1 (shift).
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vrelu(CPURISCVState *env, uint32_t desc, uint32_t op,
                              target_ulong val, target_ulong shift)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    int64_t m = sextract64(val, 0, eb);
    unsigned s = shift & (eb == 8 ? 0x1f : 0x3f);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    int qy = FIELD_EX32(desc, PIE, Q0);
    PieVec y = pie_q(pie, qy);

    for (int i = 0; i < 128 / eb; i++) {
        int64_t v = elt_s(y.w, eb, i);

        if (v <= 0) {
            elt_set(y.w, eb, i, pie_fit_s(pie, pie_sra(pie, v * m, s, rm), eb,
                                          sat));
        }
    }
    pie_set_q(pie, qy, &y);
}

/*
 * ESP.VPRELU.*: Q0 = qz, Q1 = qy, Q2 = qx: each element of qx not above
 * zero multiplied by qy's and shifted right by rs1's low five (8-bit) or
 * six (16-bit) bits, the others copied.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vprelu(CPURISCVState *env, uint32_t desc, uint32_t op,
                               target_ulong shift)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    unsigned s = shift & (eb == 8 ? 0x1f : 0x3f);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t v = elt_s(x.w, eb, i);

        if (v <= 0) {
            v = pie_fit_s(pie, pie_sra(pie, v * elt_s(y.w, eb, i), s, rm), eb,
                          sat);
        }
        elt_set(r.w, eb, i, v);
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/* Permutations */

/*
 * The registers: ESP.VZIP and ESP.VUNZIP Q0 = qx, Q1 = qy; ESP.VZIPT and
 * ESP.VUNZIPT Q0 = qx, Q1 = qy, Q2 = qw; ESP.VEXT Q0 = qz, Q1 = qv,
 * Q2 = qw.
 *
 * ESP.VZIP interleaves the elements of qx and qy, the first half of the
 * result going to qx and the second to qy; ESP.VUNZIP takes the even
 * elements of qx followed by qy into qx and the odd ones into qy.
 * ESP.VZIPT and ESP.VUNZIPT do the same over three registers (the TRM's
 * listing of ESP.VZIPT.8 repeats some indices; the pattern of its other
 * lines and of ESP.VZIPT.16 is taken). ESP.VEXT extends the low half of
 * qw's elements to twice their width into qz and the high half into qv.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_perm(CPURISCVState *env, uint32_t desc, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    int n = 128 / eb;
    unsigned kind = FIELD_EX32(op, PIEOP, KIND);
    int r[3] = {
        FIELD_EX32(desc, PIE, Q0), FIELD_EX32(desc, PIE, Q1),
        FIELD_EX32(desc, PIE, Q2),
    };
    int nr = (kind == PIE_PERM_ZIPT || kind == PIE_PERM_UNZIPT) ? 3 : 2;
    uint64_t in[6], out[6];

    for (int k = 0; k < 3; k++) {
        in[2 * k] = pie->q[r[k]][0];
        in[2 * k + 1] = pie->q[r[k]][1];
    }
    switch (kind) {
    case PIE_PERM_ZIP:
    case PIE_PERM_ZIPT:
        for (int i = 0; i < nr * n; i++) {
            elt_set(out, eb, i, elt_u(in + 2 * (i % nr), eb, i / nr));
        }
        break;
    case PIE_PERM_UNZIP:
    case PIE_PERM_UNZIPT:
        for (int i = 0; i < nr * n; i++) {
            elt_set(out + 2 * (i % nr), eb, i / nr, elt_u(in, eb, i));
        }
        break;
    default:
        for (int i = 0; i < n; i++) {
            elt_set(out, eb * 2, i, elt(in + 4, eb, i, pie_signed(op)));
        }
        break;
    }
    for (int k = 0; k < nr; k++) {
        pie->q[r[k]][0] = out[2 * k];
        pie->q[r[k]][1] = out[2 * k + 1];
    }
}

/* Register moves */

/* The PIE special registers ESP.MOVX.R and ESP.MOVX.W access. */
/*
 * Reading CFG clears vxsat, as 4.4.4 says, and rm_exc, the other flag
 * the core sets and software has no other way to clear.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
target_ulong helper_esp32p4_pie_movx_r(CPURISCVState *env, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    uint32_t v;

    switch (FIELD_EX32(op, PIEOP, KIND)) {
    case PIE_SR_CFG:
        v = pie->cfg;
        pie->cfg &= ~(ESP32P4_PIE_CFG_VXSAT | ESP32P4_PIE_CFG_RM_EXC);
        return v;
    case PIE_SR_FFT_BIT_WIDTH:
        return pie->fft_bit_width;
    case PIE_SR_PERF:
        return pie->perf;
    case PIE_SR_SAR:
        return pie->sar;
    case PIE_SR_SAR_BYTE:
        return pie->sar_byte;
    case PIE_SR_XACC_H:
        return extract64(pie->xacc, 32, 8);
    default:
        return (uint32_t)pie->xacc;
    }
}

/* CFG's vxsat and rm_exc are read-only. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_movx_w(CPURISCVState *env, uint32_t op,
                               target_ulong val)
{
    ESP32P4PIE *pie = pie_of(env);

    switch (FIELD_EX32(op, PIEOP, KIND)) {
    case PIE_SR_CFG:
        pie->cfg = (pie->cfg & ~ESP32P4_PIE_CFG_WMASK) |
                   (val & ESP32P4_PIE_CFG_WMASK);
        break;
    case PIE_SR_FFT_BIT_WIDTH:
        pie->fft_bit_width = val & 0xf;
        break;
    case PIE_SR_PERF:
        pie->perf = val;
        break;
    case PIE_SR_SAR:
        pie->sar = val & 0x3f;
        break;
    case PIE_SR_SAR_BYTE:
        pie->sar_byte = val & 0xf;
        break;
    case PIE_SR_XACC_H:
        pie->xacc = deposit64(pie->xacc, 32, 8, val);
        break;
    default:
        pie->xacc = deposit64(pie->xacc, 0, 32, val);
        break;
    }
}

/* ESP.MOVI.*.A: element SEL of Q0 = qy, zero-extended. */
/* [spec:nuos:req:emu.esp32p4.pie] */
target_ulong helper_esp32p4_pie_movi_a(CPURISCVState *env, uint32_t desc,
                                       uint32_t op)
{
    PieVec y = pie_q(pie_of(env), FIELD_EX32(desc, PIE, Q0));

    return elt_u(y.w, pie_esz(op), FIELD_EX32(desc, PIE, SEL));
}

/* ESP.MOVI.*.Q: element SEL of Q0 = qy set from rs1's low bits. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_movi_q(CPURISCVState *env, uint32_t desc, uint32_t op,
                               target_ulong val)
{
    ESP32P4PIE *pie = pie_of(env);
    int q = FIELD_EX32(desc, PIE, Q0);
    PieVec y = pie_q(pie, q);

    elt_set(y.w, pie_esz(op), FIELD_EX32(desc, PIE, SEL), val);
    pie_set_q(pie, q, &y);
}

/* ESP.ZERO.Q (KIND 0, Q0 = qz), ESP.ZERO.QACC (1) and ESP.ZERO.XACC (2). */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_zero(CPURISCVState *env, uint32_t desc, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);

    switch (FIELD_EX32(op, PIEOP, KIND)) {
    case 0:
        memset(pie->q[FIELD_EX32(desc, PIE, Q0)], 0,
               sizeof(pie->q[0]));
        break;
    case 1:
        memset(pie->qacc, 0, sizeof(pie->qacc));
        break;
    default:
        pie->xacc = 0;
        break;
    }
}

/*
 * QACC from the 8-bit elements of a vector extended to 32 bits, or its
 * 16-bit elements extended to 64 bits, element i to accumulator i.
 */
static void pie_qacc_from(ESP32P4PIE *pie, const PieVec *v, uint32_t op)
{
    int eb = pie_esz(op);

    for (int i = 0; i < 128 / eb; i++) {
        elt_set(pie->qacc, eb * 4, i, elt(v->w, eb, i, pie_signed(op)));
    }
}

/* ESP.MOV.*.QACC: QACC from the elements of QU = qu. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_mov_qacc(CPURISCVState *env, uint32_t desc,
                                 uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    PieVec v = pie_q(pie, FIELD_EX32(desc, PIE, QU));

    pie_qacc_from(pie, &v, op);
}

/* Bitwise logic: KIND 0 AND, 1 OR, 2 XOR, 3 NOT; Q0 = qz, Q1 = qx, Q2 = qy. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_logic(CPURISCVState *env, uint32_t desc, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec r = { };

    for (int i = 0; i < 2; i++) {
        switch (FIELD_EX32(op, PIEOP, KIND)) {
        case 0:
            r.w[i] = x.w[i] & y.w[i];
            break;
        case 1:
            r.w[i] = x.w[i] | y.w[i];
            break;
        case 2:
            r.w[i] = x.w[i] ^ y.w[i];
            break;
        default:
            r.w[i] = ~x.w[i];
            break;
        }
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/* Loads and stores */

/*
 * The loads, at the base address. ESP.VLDEXT extends the low half of the
 * loaded elements into QU and the high half into qz; ESP.VLDHBC.16.INCP
 * repeats each of the first four
 * halfwords twice into QU and each of the last four twice into qz.
 * ESP.LD.128.USAR keeps the address's low four bits, the data's offset
 * from alignment, in SAR_BYTE. ESP.LD.QACC.L.L, L.H, H.L and H.H (X 0 to
 * 3) load QACC_L[127:0], QACC_L[255:128], QACC_H[127:0] and
 * QACC_H[255:128]. ESP.LD.XACC.IP loads 64 bits and keeps the low 40.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_load(CPURISCVState *env, uint32_t desc, uint32_t op,
                             target_ulong base)
{
    ESP32P4PIE *pie = pie_of(env);
    uintptr_t ra = GETPC();
    int eb = pie_esz(op);
    int qu = FIELD_EX32(desc, PIE, QU);
    int q0 = FIELD_EX32(desc, PIE, Q0);
    PieVec v = { }, a = { }, b = { };
    uint64_t x;

    switch (FIELD_EX32(op, PIEOP, KIND)) {
    case PIE_LD_Q:
        pie_load_q(env, base, &v, ra);
        pie_set_q(pie, qu, &v);
        break;
    case PIE_LD_H64:
    case PIE_LD_L64:
        x = cpu_ldq_le_mmuidx_ra(env, pie_ea(pie, base, 8, false),
                                 pie_mmu(env), ra);
        pie->q[qu][FIELD_EX32(op, PIEOP, KIND) == PIE_LD_H64] = x;
        break;
    case PIE_LD_BC: {
        target_ulong addr = pie_ea(pie, base, eb / 8, false);

        switch (eb) {
        case 8:
            x = cpu_ldub_mmuidx_ra(env, addr, pie_mmu(env), ra);
            break;
        case 16:
            x = cpu_lduw_le_mmuidx_ra(env, addr, pie_mmu(env), ra);
            break;
        default:
            x = cpu_ldl_le_mmuidx_ra(env, addr, pie_mmu(env), ra);
            break;
        }
        for (int i = 0; i < 128 / eb; i++) {
            elt_set(v.w, eb, i, x);
        }
        pie_set_q(pie, qu, &v);
        break;
    }
    case PIE_LD_EXT:
        pie_load_q(env, base, &v, ra);
        for (int i = 0; i < 128 / eb; i++) {
            elt_set(i < 64 / eb ? a.w : b.w, eb * 2, i % (64 / eb),
                    elt(v.w, eb, i, pie_signed(op)));
        }
        pie_set_q(pie, qu, &a);
        pie_set_q(pie, q0, &b);
        break;
    case PIE_LD_HBC:
        pie_load_q(env, base, &v, ra);
        for (int i = 0; i < 16; i++) {
            elt_set(i < 8 ? a.w : b.w, 16, i % 8, elt_u(v.w, 16, i / 2));
        }
        pie_set_q(pie, qu, &a);
        pie_set_q(pie, q0, &b);
        break;
    case PIE_LD_USAR:
        pie_load_q(env, base, &v, ra);
        pie_set_q(pie, qu, &v);
        pie->sar_byte = base & 0xf;
        break;
    case PIE_LD_QA:
        pie_load_q(env, base, &v, ra);
        pie_qacc_from(pie, &v, op);
        break;
    case PIE_LD_QACC: {
        int part = FIELD_EX32(op, PIEOP, X);

        pie_load_q(env, base, &v, ra);
        pie->qacc[2 * part] = v.w[0];
        pie->qacc[2 * part + 1] = v.w[1];
        break;
    }
    case PIE_LD_XACC:
        x = cpu_ldq_le_mmuidx_ra(env, pie_ea(pie, base, 8, false),
                                 pie_mmu(env), ra);
        pie->xacc = extract64(x, 0, 40);
        break;
    default:
        pie_load_q(env, base, &v, ra);
        pie->ua_state[0] = v.w[0];
        pie->ua_state[1] = v.w[1];
        break;
    }
}

/* The stores, at the base address. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_store(CPURISCVState *env, uint32_t desc, uint32_t op,
                              target_ulong base)
{
    ESP32P4PIE *pie = pie_of(env);
    uintptr_t ra = GETPC();
    int qu = FIELD_EX32(desc, PIE, QU);
    PieVec v = { };
    uint64_t x;

    switch (FIELD_EX32(op, PIEOP, KIND)) {
    case PIE_ST_Q:
        v = pie_q(pie, qu);
        pie_store_q(env, base, &v, ra);
        break;
    case PIE_ST_H64:
    case PIE_ST_L64:
        cpu_stq_le_mmuidx_ra(env, pie_ea(pie, base, 8, true),
                             pie->q[qu][FIELD_EX32(op, PIEOP, KIND) ==
                                        PIE_ST_H64],
                             pie_mmu(env), ra);
        pie_stored(env);
        break;
    case PIE_ST_QACC: {
        int part = FIELD_EX32(op, PIEOP, X);

        v.w[0] = pie->qacc[2 * part];
        v.w[1] = pie->qacc[2 * part + 1];
        pie_store_q(env, base, &v, ra);
        break;
    }
    case PIE_ST_XACC:
        x = pie_signed(op) ? sextract64(pie->xacc, 0, 40)
                           : extract64(pie->xacc, 0, 40);
        cpu_stq_le_mmuidx_ra(env, pie_ea(pie, base, 8, true), x,
                             pie_mmu(env), ra);
        pie_stored(env);
        break;
    default:
        v.w[0] = pie->ua_state[0];
        v.w[1] = pie->ua_state[1];
        pie_store_q(env, base, &v, ra);
        break;
    }
}

/*
 * ESP.LDXQ.32 (KIND 0) and ESP.STXQ.32 (KIND 1): QU = qu, Q0 = qw, SEL =
 * sel8, SELB = sel4. The word at rs1 plus four times qw's 16-bit element
 * sel8, taken as unsigned, is loaded into or stored from qu's word sel4;
 * rs1 itself is not updated.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_xq(CPURISCVState *env, uint32_t desc, uint32_t op,
                           target_ulong base)
{
    ESP32P4PIE *pie = pie_of(env);
    int qu = FIELD_EX32(desc, PIE, QU);
    int word = FIELD_EX32(desc, PIE, SELB);
    PieVec w = pie_q(pie, FIELD_EX32(desc, PIE, Q0));
    PieVec u = pie_q(pie, qu);
    target_ulong addr = base + elt_u(w.w, 16, FIELD_EX32(desc, PIE, SEL)) * 4;
    bool store = FIELD_EX32(op, PIEOP, KIND);

    addr = pie_ea(pie, addr, 4, store);
    if (store) {
        cpu_stl_le_mmuidx_ra(env, addr, elt_u(u.w, 32, word), pie_mmu(env),
                             GETPC());
        pie_stored(env);
    } else {
        elt_set(u.w, 32, word,
                cpu_ldl_le_mmuidx_ra(env, addr, pie_mmu(env), GETPC()));
        pie_set_q(pie, qu, &u);
    }
}

/* Concatenated shifts */

/* The 32 bytes {hi, lo} shifted right by n bytes: the low 16. */
static PieVec pie_concat_shr(const PieVec *lo, const PieVec *hi, unsigned n)
{
    uint64_t in[4] = { lo->w[0], lo->w[1], hi->w[0], hi->w[1] };
    PieVec r = { };

    for (int i = 0; i < 16; i++) {
        elt_set(r.w, 8, i, i + n < 32 ? elt_u(in, 8, i + n) : 0);
    }
    return r;
}

/*
 * The concatenated shifts, with Q0 = qz, Q1 = qw, Q2 = qy.
 * SRC shifts {qy, qw} right by SAR_BYTE bytes; the .2Q forms shift
 * {qy, qw} as a whole, filling with zeros, by SEL + 1 (SLCI, SRCI) or
 * rs1[3:0] + 1 (SLCXXP, SRCXXP) bytes.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_src(CPURISCVState *env, uint32_t desc, uint32_t op,
                            target_ulong addr, target_ulong count)
{
    ESP32P4PIE *pie = pie_of(env);
    int qw = FIELD_EX32(desc, PIE, Q1);
    int qy = FIELD_EX32(desc, PIE, Q2);
    PieVec w = pie_q(pie, qw);
    PieVec y = pie_q(pie, qy);
    PieVec r = { }, hi = { };
    unsigned n;

    switch (FIELD_EX32(op, PIEOP, KIND)) {
    case PIE_SRC_Q:
        r = pie_concat_shr(&w, &y, pie->sar_byte);
        pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
        break;
    case PIE_SRC_Q_QUP:
        r = pie_concat_shr(&w, &y, pie->sar_byte);
        pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
        pie_set_q(pie, qw, &y);
        break;
    case PIE_SRC_Q_LD:
        r = pie_concat_shr(&w, &y, pie->sar_byte);
        pie_mem_then_commit(env, desc, addr, pie->cfg, qw, &r, GETPC());
        break;
    case PIE_SRC_SLC: {
        PieVec zero = { { 0, 0 } };

        n = 16 - ((count & 0xf) + 1);
        r = pie_concat_shr(&zero, &w, n);
        hi = pie_concat_shr(&w, &y, n);
        pie_set_q(pie, qw, &r);
        pie_set_q(pie, qy, &hi);
        break;
    }
    case PIE_SRC_SRC: {
        PieVec zero = { { 0, 0 } };

        n = (count & 0xf) + 1;
        r = pie_concat_shr(&w, &y, n);
        hi = pie_concat_shr(&y, &zero, n);
        pie_set_q(pie, qw, &r);
        pie_set_q(pie, qy, &hi);
        break;
    }
    default:
        r = pie_concat_shr(&w, &y, pie->sar_byte);
        pie_store_q(env, addr, &r, GETPC());
        break;
    }
}

/*
 * ESP.SRCMB.*: QU = qu, Q1 = qw. Each accumulator of QACC, 64-bit for the
 * 16-bit forms and 32-bit for the 8-bit ones, is shifted right, signed or
 * not as the instruction is, and rounded; the shifted value replaces it
 * and, saturated (sat set) or truncated to the element, becomes element i
 * of qu. The shift is rs1's low six (16-bit) or five (8-bit) bits, or
 * (KIND 1, the .Q forms) element i of qw limited to 63 or 31.
 *
 * The TRM's listings of ESP.SRCMB.S16.QACC and U16.QACC give sat clear
 * the unshifted accumulators' low bits, unlike their .Q forms and the
 * 8-bit forms; as the rest, these give the shifted values' low bits.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_srcmb(CPURISCVState *env, uint32_t desc, uint32_t op,
                              target_ulong shift)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    int ab = eb * 4;
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    unsigned smax = eb == 16 ? 0x3f : 0x1f;
    PieVec w = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        unsigned s;
        int64_t v;

        if (FIELD_EX32(op, PIEOP, KIND)) {
            s = MIN(elt_u(w.w, eb, i), smax);
        } else {
            s = shift & smax;
        }
        if (sg) {
            v = pie_sra(pie, elt_s(pie->qacc, ab, i), s, rm);
            elt_set(pie->qacc, ab, i, v);
            elt_set(r.w, eb, i, pie_fit_s(pie, v, eb, sat));
        } else {
            uint64_t u = pie_srl(pie, elt_u(pie->qacc, ab, i), s, rm);

            elt_set(pie->qacc, ab, i, u);
            if (sat && u > MAKE_64BIT_MASK(0, eb)) {
                pie_saturated(pie);
                u = MAKE_64BIT_MASK(0, eb);
            }
            elt_set(r.w, eb, i, u);
        }
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, QU), &r);
}

/*
 * ESP.SRS.S.XACC and ESP.SRS.U.XACC: XACC shifted right, signed or not, by
 * rs1's low six bits and rounded replaces XACC and, saturated to 32 bits
 * (sat set) or truncated, is the result.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
target_ulong helper_esp32p4_pie_srs(CPURISCVState *env, uint32_t desc,
                                    uint32_t op, target_ulong shift)
{
    ESP32P4PIE *pie = pie_of(env);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    unsigned s = shift & 0x3f;

    if (pie_signed(op)) {
        int64_t v = pie_sra(pie, sextract64(pie->xacc, 0, 40), s, rm);

        pie->xacc = extract64(v, 0, 40);
        return (int32_t)pie_fit_s(pie, v, 32, sat);
    } else {
        uint64_t v = pie_srl(pie, extract64(pie->xacc, 0, 40), s, rm);

        pie->xacc = v;
        if (sat && v > UINT32_MAX) {
            pie_saturated(pie);
            v = UINT32_MAX;
        }
        return (int32_t)v;
    }
}

/*
 * The vector shifts, with Q0 = qu, Q1 = qy, Q2 = qw.
 *
 * ESP.VSR and ESP.VSL shift by SAR's low five bits, as 4.4.1.3.1 says of
 * them (the instructions' listings show six). ESP.VSR.U32 shifts
 * logically: its listing repeats ESP.VSR.S32's sign fill, which would make
 * the two the same. ESP.VSLD and ESP.VSRD shift each signed element of qy
 * by the signed low four (8-bit), five (16-bit) or six (32-bit) bits of
 * qw's element: VSLD left for a positive amount and right for a negative
 * one, VSRD the other way round. Right shifts round; left shifts
 * saturate (sat set) or keep the low bits.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_vshift(CPURISCVState *env, uint32_t desc, uint32_t op)
{
    ESP32P4PIE *pie = pie_of(env);
    int eb = pie_esz(op);
    bool sg = pie_signed(op);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned rm = FIELD_EX32(desc, PIE, RM);
    unsigned kind = FIELD_EX32(op, PIEOP, KIND);
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec w = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    int abits = eb == 8 ? 4 : eb == 16 ? 5 : 6;
    PieVec r = { };

    for (int i = 0; i < 128 / eb; i++) {
        int64_t v = elt(y.w, eb, i, sg);
        int amt;

        switch (kind) {
        case PIE_VSHIFT_SR:
            amt = -(int)(pie->sar & 0x1f);
            break;
        case PIE_VSHIFT_SL:
            amt = pie->sar & 0x1f;
            break;
        case PIE_VSHIFT_SLD:
            amt = sextract64(elt_u(w.w, eb, i), 0, abits);
            break;
        default:
            amt = -sextract64(elt_u(w.w, eb, i), 0, abits);
            break;
        }
        if (amt < 0) {
            v = sg ? pie_sra(pie, v, -amt, rm)
                   : (int64_t)pie_srl(pie, v, -amt, rm);
        } else {
            v = pie_fit(pie, v << amt, eb, sg, sat);
        }
        elt_set(r.w, eb, i, v);
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/* FFT */

/*
 * ESP.FFT.R2BF.S16 (KIND 0): Q0 = qz, Q1 = qv, Q2 = qx, Q3 = qy, SEL =
 * sel2. The radix-2 butterflies of operand vectors a and b assembled from
 * qx and qy as sel2 says: qz holds the sums and differences of their
 * elements 0-3 and qv of their elements 4-7.
 *
 * ESP.FFT.R2BF.S16.ST.INCP (KIND 1): Q0 = qz, Q2 = qx, Q3 = qy, SEL =
 * sel4: qz = qx - qy, and qx + qy shifted right by sel4 is stored.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_fft_r2bf(CPURISCVState *env, uint32_t desc,
                                 uint32_t op, target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned sel = FIELD_EX32(desc, PIE, SEL);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q3));
    PieVec z = { }, v = { };

    if (FIELD_EX32(op, PIEOP, KIND) == 0) {
        int64_t a[8], b[8];

        for (int i = 0; i < 4; i++) {
            int lo = sel ? (i / 2) * 4 + i % 2 : i;
            int hi = sel ? lo + 2 : i + 4;

            a[i] = elt_s(x.w, 16, lo);
            a[i + 4] = elt_s(y.w, 16, lo);
            b[i] = elt_s(x.w, 16, hi);
            b[i + 4] = elt_s(y.w, 16, hi);
        }
        for (int i = 0; i < 4; i++) {
            elt_set(z.w, 16, i, pie_fit_s(pie, a[i] + b[i], 16, sat));
            elt_set(z.w, 16, i + 4, pie_fit_s(pie, a[i] - b[i], 16, sat));
            elt_set(v.w, 16, i, pie_fit_s(pie, a[i + 4] + b[i + 4], 16,
                                          sat));
            elt_set(v.w, 16, i + 4, pie_fit_s(pie, a[i + 4] - b[i + 4], 16,
                                              sat));
        }
        pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &z);
        pie_set_q(pie, FIELD_EX32(desc, PIE, Q1), &v);
        return;
    }

    {
        uint32_t cfg = pie->cfg;
        uint32_t cfg_after;
        PieVec st = { };

        for (int i = 0; i < 8; i++) {
            int64_t a = elt_s(x.w, 16, i), b = elt_s(y.w, 16, i);

            elt_set(z.w, 16, i, pie_fit_s(pie, a - b, 16, sat));
            elt_set(st.w, 16, i, pie_fit_s(pie, (a + b) >> sel, 16, sat));
        }
        cfg_after = pie->cfg;
        pie->cfg = cfg;
        pie_store_q(env, addr, &st, GETPC());
        pie->cfg = cfg_after;
        pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &z);
    }
}

/*
 * The complex product of element pair k of qy and qx that
 * ESP.FFT.CMUL.S16 computes for sel8: pair sel8 / 2, by qx's conjugate
 * when sel8 is even. The TRM lists sel8 0 to 5 for the .LD.XP form and 6
 * and 7 for the .ST.XP form; both are taken to compute any pair, as the
 * encoding allows.
 */
static void pie_fft_cmul(ESP32P4PIE *pie, const PieVec *x, const PieVec *y,
                         unsigned sel8, bool sat, int64_t *re, int64_t *im)
{
    int k = (sel8 / 2) * 2;
    int64_t xr = elt_s(x->w, 16, k), xi = elt_s(x->w, 16, k + 1);
    int64_t yr = elt_s(y->w, 16, k), yi = elt_s(y->w, 16, k + 1);
    int64_t a, b;

    if (sel8 & 1) {
        a = yr * xr - yi * xi;
        b = yi * xr + yr * xi;
    } else {
        a = yr * xr + yi * xi;
        b = yi * xr - yr * xi;
    }
    *re = pie_fit_s(pie, a >> (pie->sar & 0x3f), 16, sat);
    *im = pie_fit_s(pie, b >> (pie->sar & 0x3f), 16, sat);
}

/*
 * ESP.FFT.CMUL.S16.LD.XP (KIND 0): Q0 = qz, Q1 = qx, Q2 = qy, SEL = sel8:
 * the product into qz's element pair, loading QU.
 *
 * ESP.FFT.CMUL.S16.ST.XP (KIND 1): Q1 = qx, Q2 = qy, QU = qu, SEL = sel8,
 * SELB = upd4, PIEOP.X = sel4: stores 16 bytes whose top word is the
 * product and whose rest upd4 picks: qu's low 96 bits (0); qy's elements
 * 0-3 shifted right by sel4 and qu's word 2 (1, the last but one radix-2
 * stage); or qy's elements 0-1 shifted, qu's word 2, then qy's elements
 * 2-3 shifted (2, the last stage).
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_fft_cmul(CPURISCVState *env, uint32_t desc,
                                 uint32_t op, target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    uint32_t cfg = pie->cfg;
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned sel8 = FIELD_EX32(desc, PIE, SEL);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q1));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    int64_t re, im;

    pie_fft_cmul(pie, &x, &y, sel8, sat, &re, &im);
    if (FIELD_EX32(op, PIEOP, KIND) == 0) {
        int qz = FIELD_EX32(desc, PIE, Q0);
        PieVec z = pie_q(pie, qz);

        elt_set(z.w, 16, (sel8 / 2) * 2, re);
        elt_set(z.w, 16, (sel8 / 2) * 2 + 1, im);
        pie_mem_then_commit(env, desc, addr, cfg, qz, &z, GETPC());
    } else {
        PieVec u = pie_q(pie, FIELD_EX32(desc, PIE, QU));
        unsigned s = FIELD_EX32(op, PIEOP, X);
        uint32_t cfg_after = pie->cfg;
        PieVec st = { };

        for (int i = 0; i < 4; i++) {
            elt_set(st.w, 16, i, elt_s(y.w, 16, i) >> s);
        }
        switch (FIELD_EX32(desc, PIE, SELB)) {
        case 0:
            st.w[0] = u.w[0];
            elt_set(st.w, 32, 2, elt_u(u.w, 32, 2));
            break;
        case 1:
            elt_set(st.w, 32, 2, elt_u(u.w, 32, 2));
            break;
        default:
            elt_set(st.w, 16, 4, elt_s(y.w, 16, 2) >> s);
            elt_set(st.w, 16, 5, elt_s(y.w, 16, 3) >> s);
            elt_set(st.w, 32, 1, elt_u(u.w, 32, 2));
            break;
        }
        elt_set(st.w, 16, 6, re);
        elt_set(st.w, 16, 7, im);
        pie->cfg = cfg;
        pie_store_q(env, addr, &st, GETPC());
        pie->cfg = cfg_after;
    }
}

/*
 * ESP.FFT.AMS.S16.*: Q0 = qz, Q1 = qv, Q2 = qx, Q3 = qw, Q4 = qy, QU = qu,
 * SEL = sel2, rs2 the value of rs2 (.ST.INCP).
 *
 * The four forms together carry out the add, multiply and subtract step
 * of a real FFT's post-processing over a vector, pair by pair. The TRM's
 * listing of the .LD.INCP form writes its results to elements 0-1, where
 * the .UAUP form writes its own; they go to the elements the form works
 * on, 2-3, as the other forms' do. The listing of .LD.R32.DECP adds qy
 * where the others add qw; qw is taken. Every listing shows the
 * saturating form; sat clear wraps.
 *
 * The loading forms load QU: .LD.INCP the 16 bytes at rs1; .UAUP the 16
 * bytes at the unaligned position SAR_BYTE bytes into the concatenation of
 * UA_STATE and the 16 bytes at rs1, which UA_STATE then keeps; and
 * .LD.R32.DECP the 16 bytes at rs1 forced to 16-byte alignment, in
 * reverse word order. .ST.INCP stores rs2 and qu's elements 0-5, halved
 * when sel2 is clear, and returns the sums of the pair as the new rs2;
 * qz receives the differences.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
target_ulong helper_esp32p4_pie_fft_ams(CPURISCVState *env, uint32_t desc,
                                        uint32_t op, target_ulong addr,
                                        target_ulong rs2)
{
    ESP32P4PIE *pie = pie_of(env);
    uintptr_t ra = GETPC();
    unsigned kind = FIELD_EX32(op, PIEOP, KIND);
    bool sat = FIELD_EX32(desc, PIE, SAT);
    unsigned sel = FIELD_EX32(desc, PIE, SEL);
    int qz = FIELD_EX32(desc, PIE, Q0);
    int qv = FIELD_EX32(desc, PIE, Q1);
    int qu = FIELD_EX32(desc, PIE, QU);
    PieVec x = pie_q(pie, FIELD_EX32(desc, PIE, Q2));
    PieVec w = pie_q(pie, FIELD_EX32(desc, PIE, Q3));
    PieVec y = pie_q(pie, FIELD_EX32(desc, PIE, Q4));
    PieVec z = pie_q(pie, qz);
    PieVec v = pie_q(pie, qv);
    PieVec ld = { }, st = { };
    int e = kind * 2;
    int64_t x0 = elt_s(x.w, 16, e), x1 = elt_s(x.w, 16, e + 1);
    int64_t w0 = elt_s(w.w, 16, e), w1 = elt_s(w.w, 16, e + 1);
    int64_t y0 = elt_s(y.w, 16, e), y1 = elt_s(y.w, 16, e + 1);
    int64_t t0 = x0 + w0, t1 = x1 - w1, d = x0 - w0, s = x1 + w1;
    unsigned sar = pie->sar & 0x3f;
    uint32_t cfg = pie->cfg, cfg_after;
    int64_t t2, t3, add0, add1, sub0, sub1;
    target_ulong new_rs2 = rs2;

    if (sel) {
        t2 = (s * y1 + d * y0) >> sar;
        t3 = (s * y0 - d * y1) >> sar;
    } else {
        t2 = (d * y0 - s * y1) >> sar;
        t3 = (d * y1 + s * y0) >> sar;
    }
    add0 = pie_fit_s(pie, t0 + t2, 16, sat);
    add1 = pie_fit_s(pie, t1 + t3, 16, sat);
    sub0 = pie_fit_s(pie, t0 - t2, 16, sat);
    sub1 = pie_fit_s(pie, t3 - t1, 16, sat);
    cfg_after = pie->cfg;
    pie->cfg = cfg;

    switch (kind) {
    case PIE_AMS_LD_INCP:
        pie_load_q(env, addr, &ld, ra);
        break;
    case PIE_AMS_LD_INCP_UAUP: {
        PieVec ua = { { pie->ua_state[0], pie->ua_state[1] } };
        PieVec data = { };

        pie_load_q(env, addr, &data, ra);
        ld = pie_concat_shr(&ua, &data, pie->sar_byte);
        pie->ua_state[0] = data.w[0];
        pie->ua_state[1] = data.w[1];
        break;
    }
    case PIE_AMS_LD_R32_DECP: {
        PieVec data = { };

        pie_ld128(env, addr & ~(target_ulong)0xf, &data, ra);
        for (int i = 0; i < 4; i++) {
            elt_set(ld.w, 32, i, elt_u(data.w, 32, 3 - i));
        }
        break;
    }
    default: {
        PieVec u = pie_q(pie, qu);
        unsigned half = sel ? 0 : 1;

        elt_set(st.w, 16, 0, sextract64(rs2, 0, 16) >> half);
        elt_set(st.w, 16, 1, sextract64(rs2, 16, 16) >> half);
        for (int i = 0; i < 6; i++) {
            elt_set(st.w, 16, i + 2, elt_s(u.w, 16, i) >> half);
        }
        pie_store_q(env, addr, &st, ra);
        new_rs2 = (int32_t)((uint32_t)extract64(add0, 0, 16) |
                            ((uint32_t)extract64(add1, 0, 16) << 16));
        break;
    }
    }
    pie->cfg = cfg_after;

    if (kind == PIE_AMS_ST_INCP) {
        elt_set(z.w, 16, e, sub0);
        elt_set(z.w, 16, e + 1, sub1);
        pie_set_q(pie, qz, &z);
    } else {
        elt_set(z.w, 16, e, add0);
        elt_set(z.w, 16, e + 1, add1);
        elt_set(v.w, 16, e, sub0);
        elt_set(v.w, 16, e + 1, sub1);
        pie_set_q(pie, qz, &z);
        pie_set_q(pie, qv, &v);
        pie_set_q(pie, qu, &ld);
    }
    return new_rs2;
}

static unsigned pie_bitrev(unsigned v, int bits)
{
    unsigned r = 0;

    for (int i = 0; i < bits; i++) {
        r |= ((v >> i) & 1) << (bits - 1 - i);
    }
    return r;
}

/*
 * ESP.FFT.BITREV: Q0 = qv. Element i is the larger of the low n bits of
 * rs1 + i and their bit reversal, for indices rs1 to rs1 + 7. n is 3 to
 * 10 for FFT_BIT_WIDTH 0 to 7, as 4.4.1.3.3 says (the instruction's
 * listing writes its cases as FFT_BIT_WIDTH == n); its bit 3 is ignored.
 * In the 3-bit mode only elements 0-5 are computed and 6-7 are zero.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_fft_bitrev(CPURISCVState *env, uint32_t desc,
                                   target_ulong val)
{
    ESP32P4PIE *pie = pie_of(env);
    int bits = (pie->fft_bit_width & 7) + 3;
    PieVec r = { { 0, 0 } };

    for (int i = 0; i < (bits == 3 ? 6 : 8); i++) {
        unsigned t = (val + i) & MAKE_64BIT_MASK(0, bits);

        elt_set(r.w, 16, i, MAX(t, pie_bitrev(t, bits)));
    }
    pie_set_q(pie, FIELD_EX32(desc, PIE, Q0), &r);
}

/*
 * ESP.FFT.VST.R32.DECP: QU = qu, SEL = sel2. Stores qu's halfwords shifted
 * right by sel2, its words in reverse order.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
void helper_esp32p4_pie_fft_vst_r32(CPURISCVState *env, uint32_t desc,
                                    target_ulong addr)
{
    ESP32P4PIE *pie = pie_of(env);
    PieVec u = pie_q(pie, FIELD_EX32(desc, PIE, QU));
    unsigned s = FIELD_EX32(desc, PIE, SEL);
    PieVec st = { };

    for (int i = 0; i < 8; i++) {
        int j = (3 - i / 2) * 2 + i % 2;

        elt_set(st.w, 16, i, elt_s(u.w, 16, j) >> s);
    }
    pie_store_q(env, addr, &st, GETPC());
}

/* ESP.SAT: rsd clamped between rs0 and rs1, compared signed. */
/* [spec:nuos:req:emu.esp32p4.pie] */
target_ulong helper_esp32p4_pie_sat(target_ulong rsd, target_ulong rs0,
                                    target_ulong rs1)
{
    int32_t lo = MIN((int32_t)rs0, (int32_t)rs1);
    int32_t hi = MAX((int32_t)rs0, (int32_t)rs1);

    return (int32_t)MAX(MIN((int32_t)rsd, hi), lo);
}

/* State */

/*
 * The encodings of the PIE instructions, the bits their TRM figures fix,
 * as esp32p4.decode decodes them.
 */
static const struct {
    uint32_t mask;
    uint32_t match;
} esp32p4_pie_insns[] = {
    { 0xfe00707f, 0x04000033 }, /* ESP.ADDX2 */
    { 0xfe00707f, 0x08000033 }, /* ESP.ADDX4 */
    { 0x0384607f, 0x0180001f }, /* ESP.ANDQ */
    { 0x03f4607f, 0x0384001f }, /* ESP.CMUL.S16 */
    { 0x0380007f, 0x0180007b }, /* ESP.CMUL.S16.LD.INCP */
    { 0x0380007f, 0x0380007b }, /* ESP.CMUL.S16.ST.INCP */
    { 0x03f4607f, 0x0304001f }, /* ESP.CMUL.S8 */
    { 0x0380007f, 0x0080007b }, /* ESP.CMUL.S8.LD.INCP */
    { 0x0380007f, 0x0280007b }, /* ESP.CMUL.S8.ST.INCP */
    { 0x03f4607f, 0x0380001f }, /* ESP.CMUL.U16 */
    { 0x0380007f, 0x0100007b }, /* ESP.CMUL.U16.LD.INCP */
    { 0x0380007f, 0x0300007b }, /* ESP.CMUL.U16.ST.INCP */
    { 0x03f4607f, 0x0300001f }, /* ESP.CMUL.U8 */
    { 0x0380007f, 0x0000007b }, /* ESP.CMUL.U8.LD.INCP */
    { 0x0380007f, 0x0200007b }, /* ESP.CMUL.U8.ST.INCP */
    { 0x0000207f, 0x0000005b }, /* ESP.FFT.AMS.S16.LD.INCP */
    { 0x0000207f, 0x0000003b }, /* ESP.FFT.AMS.S16.LD.INCP.UAUP */
    { 0x0000207f, 0x0000203b }, /* ESP.FFT.AMS.S16.LD.R32.DECP */
    { 0x0000007f, 0x0000003f }, /* ESP.FFT.AMS.S16.ST.INCP */
    { 0x8000727f, 0x0000401b }, /* ESP.FFT.BITREV */
    { 0x0000207f, 0x0000205b }, /* ESP.FFT.CMUL.S16.LD.XP */
    { 0x0000007f, 0x0000007f }, /* ESP.FFT.CMUL.S16.ST.XP */
    { 0x0380707f, 0x0200001f }, /* ESP.FFT.R2BF.S16 */
    { 0x03f0607f, 0x03d0001f }, /* ESP.FFT.R2BF.S16.ST.INCP */
    { 0x03f0607f, 0x0320001f }, /* ESP.FFT.VST.R32.DECP */
    { 0x03f060ff, 0x03a0001f }, /* ESP.LD.128.USAR.IP */
    { 0xd00862ff, 0x4000601b }, /* ESP.LD.128.USAR.XP */
    { 0x9b10607f, 0x0800001f }, /* ESP.LD.QACC.H.H.128.IP */
    { 0x9b1061ff, 0x0810001f }, /* ESP.LD.QACC.H.L.128.IP */
    { 0x9b10607f, 0x0000001f }, /* ESP.LD.QACC.L.H.128.IP */
    { 0x9b1061ff, 0x0010001f }, /* ESP.LD.QACC.L.L.128.IP */
    { 0x931063ff, 0x0010011f }, /* ESP.LD.UA.STATE.IP */
    { 0x831063ff, 0x0010031f }, /* ESP.LD.XACC.IP */
    { 0x9b1061ff, 0x1810009f }, /* ESP.LDQA.S16.128.IP */
    { 0xf000707f, 0xd000501b }, /* ESP.LDQA.S16.128.XP */
    { 0x9b1061ff, 0x0810009f }, /* ESP.LDQA.S8.128.IP */
    { 0xf000707f, 0x5000501b }, /* ESP.LDQA.S8.128.XP */
    { 0x9b1061ff, 0x1010009f }, /* ESP.LDQA.U16.128.IP */
    { 0xf000707f, 0x9000501b }, /* ESP.LDQA.U16.128.XP */
    { 0x9b1061ff, 0x0010009f }, /* ESP.LDQA.U8.128.IP */
    { 0xf000707f, 0x1000501b }, /* ESP.LDQA.U8.128.XP */
    { 0xd08060ff, 0x4000609b }, /* ESP.LDXQ.32 */
    { 0xf0c4607f, 0xc000201b }, /* ESP.MAX.S16.A */
    { 0xb0c4607f, 0xa000201b }, /* ESP.MAX.S32.A */
    { 0xf0c4607f, 0x4000201b }, /* ESP.MAX.S8.A */
    { 0xf0c4607f, 0x8000201b }, /* ESP.MAX.U16.A */
    { 0xb0c4607f, 0x2000201b }, /* ESP.MAX.U32.A */
    { 0xf0c4607f, 0x0000201b }, /* ESP.MAX.U8.A */
    { 0xf0c4607f, 0xd000201b }, /* ESP.MIN.S16.A */
    { 0xb0c4607f, 0xb000201b }, /* ESP.MIN.S32.A */
    { 0xf0c4607f, 0x5000201b }, /* ESP.MIN.S8.A */
    { 0xf0c4607f, 0x9000201b }, /* ESP.MIN.U16.A */
    { 0xb0c4607f, 0x3000201b }, /* ESP.MIN.U32.A */
    { 0xf0c4607f, 0x1000201b }, /* ESP.MIN.U8.A */
    { 0x8000637f, 0x8000031b }, /* ESP.MOV.S16.QACC */
    { 0x8000637f, 0x8000011b }, /* ESP.MOV.S8.QACC */
    { 0x8000637f, 0x8000021b }, /* ESP.MOV.U16.QACC */
    { 0x8000637f, 0x8000001b }, /* ESP.MOV.U8.QACC */
    { 0x82f0607f, 0x00c0201b }, /* ESP.MOVI.16.A */
    { 0x82f0607f, 0x00d0201b }, /* ESP.MOVI.16.Q */
    { 0x82b0607f, 0x00a0201b }, /* ESP.MOVI.32.A */
    { 0x82b0607f, 0x00b0201b }, /* ESP.MOVI.32.Q */
    { 0x82f0607f, 0x0080201b }, /* ESP.MOVI.8.A */
    { 0x82f0607f, 0x0090201b }, /* ESP.MOVI.8.Q */
    { 0x96f0607f, 0x8080201b }, /* ESP.MOVX.R.CFG */
    { 0x9ef0607f, 0x8480201b }, /* ESP.MOVX.R.FFT.BIT.WIDTH */
    { 0x9ef0607f, 0x8c80201b }, /* ESP.MOVX.R.PERF */
    { 0x9ef0607f, 0x80c0201b }, /* ESP.MOVX.R.SAR */
    { 0x9ef0607f, 0x88c0201b }, /* ESP.MOVX.R.SAR.BYTES */
    { 0x9ef0607f, 0x8cc0201b }, /* ESP.MOVX.R.XACC.H */
    { 0x9ef0607f, 0x84c0201b }, /* ESP.MOVX.R.XACC.L */
    { 0x96f0607f, 0x9080201b }, /* ESP.MOVX.W.CFG */
    { 0x9ef0607f, 0x9480201b }, /* ESP.MOVX.W.FFT.BIT.WIDTH */
    { 0x9ef0607f, 0x9c80201b }, /* ESP.MOVX.W.PERF */
    { 0x9ef0607f, 0x90c0201b }, /* ESP.MOVX.W.SAR */
    { 0x9ef0607f, 0x98c0201b }, /* ESP.MOVX.W.SAR.BYTES */
    { 0x9ef0607f, 0x9cc0201b }, /* ESP.MOVX.W.XACC.H */
    { 0x9ef0607f, 0x94c0201b }, /* ESP.MOVX.W.XACC.L */
    { 0x0384607f, 0x0184001f }, /* ESP.NOTQ */
    { 0x0384607f, 0x0100001f }, /* ESP.ORQ */
    { 0xfe00707f, 0x40002033 }, /* ESP.SAT */
    { 0x80c4617f, 0x0040201b }, /* ESP.SLCI.2Q */
    { 0x4000647f, 0x0000601b }, /* ESP.SLCXXP.2Q */
    { 0x8044707f, 0x8000401b }, /* ESP.SRC.Q */
    { 0x000061ff, 0x0000401f }, /* ESP.SRC.Q.LD.IP */
    { 0xc000637f, 0xc000631b }, /* ESP.SRC.Q.LD.XP */
    { 0x8044707f, 0x8004401b }, /* ESP.SRC.Q.QUP */
    { 0x80c4617f, 0x8040201b }, /* ESP.SRCI.2Q */
    { 0x9884627f, 0x9804221b }, /* ESP.SRCMB.S16.Q.QACC */
    { 0xc3f060ff, 0xc3a0009f }, /* ESP.SRCMB.S16.QACC */
    { 0x9884627f, 0x8804221b }, /* ESP.SRCMB.S8.Q.QACC */
    { 0xc3f060ff, 0x43a0009f }, /* ESP.SRCMB.S8.QACC */
    { 0x9884627f, 0x9004221b }, /* ESP.SRCMB.U16.Q.QACC */
    { 0xc3f060ff, 0x83a0009f }, /* ESP.SRCMB.U16.QACC */
    { 0x9884627f, 0x8004221b }, /* ESP.SRCMB.U8.Q.QACC */
    { 0xc3f060ff, 0x03a0009f }, /* ESP.SRCMB.U8.QACC */
    { 0x8040707f, 0x8040401b }, /* ESP.SRCQ.128.ST.INCP */
    { 0x4000647f, 0x0000641b }, /* ESP.SRCXXP.2Q */
    { 0x9290607f, 0x9090201b }, /* ESP.SRS.S.XACC */
    { 0x9290607f, 0x8090201b }, /* ESP.SRS.U.XACC */
    { 0x9b10607f, 0x1800001f }, /* ESP.ST.QACC.H.H.128.IP */
    { 0x9b1061ff, 0x1810001f }, /* ESP.ST.QACC.H.L.128.IP */
    { 0x9b10607f, 0x1000001f }, /* ESP.ST.QACC.L.H.128.IP */
    { 0x9b1061ff, 0x1010001f }, /* ESP.ST.QACC.L.L.128.IP */
    { 0x931061ff, 0x1010019f }, /* ESP.ST.S.XACC.IP */
    { 0x931061ff, 0x0010019f }, /* ESP.ST.U.XACC.IP */
    { 0x931063ff, 0x1010011f }, /* ESP.ST.UA.STATE.IP */
    { 0xd08060ff, 0x4080609b }, /* ESP.STXQ.32 */
    { 0xfe00707f, 0x44000033 }, /* ESP.SUBX2 */
    { 0xfe00707f, 0x48000033 }, /* ESP.SUBX4 */
    { 0x8006727f, 0x0004421b }, /* ESP.VABS.16 */
    { 0x8002727f, 0x0002421b }, /* ESP.VABS.32 */
    { 0x8006727f, 0x0000421b }, /* ESP.VABS.8 */
    { 0x0286e77f, 0x0286231b }, /* ESP.VADD.S16 */
    { 0x038061ff, 0x0180411f }, /* ESP.VADD.S16.LD.INCP */
    { 0x038061ff, 0x0380411f }, /* ESP.VADD.S16.ST.INCP */
    { 0x0284e77f, 0x0284a31b }, /* ESP.VADD.S32 */
    { 0x030063ff, 0x0100409f }, /* ESP.VADD.S32.LD.INCP */
    { 0x030063ff, 0x0300409f }, /* ESP.VADD.S32.ST.INCP */
    { 0x0286e77f, 0x0282231b }, /* ESP.VADD.S8 */
    { 0x038061ff, 0x0080411f }, /* ESP.VADD.S8.LD.INCP */
    { 0x038061ff, 0x0280411f }, /* ESP.VADD.S8.ST.INCP */
    { 0x0286e77f, 0x0284231b }, /* ESP.VADD.U16 */
    { 0x038061ff, 0x0100411f }, /* ESP.VADD.U16.LD.INCP */
    { 0x038061ff, 0x0300411f }, /* ESP.VADD.U16.ST.INCP */
    { 0x0284e77f, 0x0280a31b }, /* ESP.VADD.U32 */
    { 0x030063ff, 0x0000409f }, /* ESP.VADD.U32.LD.INCP */
    { 0x030063ff, 0x0200409f }, /* ESP.VADD.U32.ST.INCP */
    { 0x0286e77f, 0x0280231b }, /* ESP.VADD.U8 */
    { 0x038061ff, 0x0000411f }, /* ESP.VADD.U8.LD.INCP */
    { 0x038061ff, 0x0200411f }, /* ESP.VADD.U8.ST.INCP */
    { 0x1000707f, 0x0000501b }, /* ESP.VCLAMP.S16 */
    { 0x03f7e87f, 0x0341801f }, /* ESP.VCMP.EQ.S16 */
    { 0x03f7787f, 0x0341081f }, /* ESP.VCMP.EQ.S32 */
    { 0x03f7e87f, 0x0340801f }, /* ESP.VCMP.EQ.S8 */
    { 0x03f7e87f, 0x0341001f }, /* ESP.VCMP.EQ.U16 */
    { 0x03f7787f, 0x0340081f }, /* ESP.VCMP.EQ.U32 */
    { 0x03f7e87f, 0x0340001f }, /* ESP.VCMP.EQ.U8 */
    { 0x03f7e87f, 0x0345801f }, /* ESP.VCMP.GT.S16 */
    { 0x03f7787f, 0x0345081f }, /* ESP.VCMP.GT.S32 */
    { 0x03f7e87f, 0x0344801f }, /* ESP.VCMP.GT.S8 */
    { 0x03f7e87f, 0x0345001f }, /* ESP.VCMP.GT.U16 */
    { 0x03f7787f, 0x0344081f }, /* ESP.VCMP.GT.U32 */
    { 0x03f7e87f, 0x0344001f }, /* ESP.VCMP.GT.U8 */
    { 0x03f7e87f, 0x0343801f }, /* ESP.VCMP.LT.S16 */
    { 0x03f7787f, 0x0343081f }, /* ESP.VCMP.LT.S32 */
    { 0x03f7e87f, 0x0342801f }, /* ESP.VCMP.LT.S8 */
    { 0x03f7e87f, 0x0343001f }, /* ESP.VCMP.LT.U16 */
    { 0x03f7787f, 0x0342081f }, /* ESP.VCMP.LT.U32 */
    { 0x03f7e87f, 0x0342001f }, /* ESP.VCMP.LT.U8 */
    { 0x02a6e57f, 0x0286a01b }, /* ESP.VCMULAS.S16.QACC.H */
    { 0x030062ff, 0x0300229f }, /* ESP.VCMULAS.S16.QACC.H.LD.IP */
    { 0x030063ff, 0x0300439f }, /* ESP.VCMULAS.S16.QACC.H.LD.XP */
    { 0x02a6e57f, 0x0282a01b }, /* ESP.VCMULAS.S16.QACC.L */
    { 0x030062ff, 0x0100229f }, /* ESP.VCMULAS.S16.QACC.L.LD.IP */
    { 0x030063ff, 0x0100439f }, /* ESP.VCMULAS.S16.QACC.L.LD.XP */
    { 0x02a6e57f, 0x0284a01b }, /* ESP.VCMULAS.S8.QACC.H */
    { 0x030062ff, 0x0200229f }, /* ESP.VCMULAS.S8.QACC.H.LD.IP */
    { 0x030063ff, 0x0200439f }, /* ESP.VCMULAS.S8.QACC.H.LD.XP */
    { 0x02a6e57f, 0x0280a01b }, /* ESP.VCMULAS.S8.QACC.L */
    { 0x030062ff, 0x0000229f }, /* ESP.VCMULAS.S8.QACC.L.LD.IP */
    { 0x030063ff, 0x0000439f }, /* ESP.VCMULAS.S8.QACC.L.LD.XP */
    { 0x9884607f, 0x1804201b }, /* ESP.VEXT.S16 */
    { 0x9884607f, 0x0804201b }, /* ESP.VEXT.S8 */
    { 0x9884607f, 0x1004201b }, /* ESP.VEXT.U16 */
    { 0x9884607f, 0x0004201b }, /* ESP.VEXT.U8 */
    { 0x83f0607f, 0x0310001f }, /* ESP.VLD.128.IP */
    { 0xd20862ff, 0x4000621b }, /* ESP.VLD.128.XP */
    { 0xc370607f, 0x4360001f }, /* ESP.VLD.H.64.IP */
    { 0xd30862ff, 0x4108601b }, /* ESP.VLD.H.64.XP */
    { 0xc370607f, 0x0360001f }, /* ESP.VLD.L.64.IP */
    { 0xd30862ff, 0x4008601b }, /* ESP.VLD.L.64.XP */
    { 0x03f0607f, 0x0390001f }, /* ESP.VLDBC.16.IP */
    { 0xd30862ff, 0x4208621b }, /* ESP.VLDBC.16.XP */
    { 0x03f060ff, 0x0350001f }, /* ESP.VLDBC.32.IP */
    { 0xd10862ff, 0x4108621b }, /* ESP.VLDBC.32.XP */
    { 0x03f060ff, 0x0350009f }, /* ESP.VLDBC.8.IP */
    { 0xd30862ff, 0x4008621b }, /* ESP.VLDBC.8.XP */
    { 0x9b10607f, 0x9800001f }, /* ESP.VLDEXT.S16.IP */
    { 0xd300607f, 0x5300601b }, /* ESP.VLDEXT.S16.XP */
    { 0x9b10607f, 0x8800001f }, /* ESP.VLDEXT.S8.IP */
    { 0xd300607f, 0x5100601b }, /* ESP.VLDEXT.S8.XP */
    { 0x9b10607f, 0x9000001f }, /* ESP.VLDEXT.U16.IP */
    { 0xd300607f, 0x5200601b }, /* ESP.VLDEXT.U16.XP */
    { 0x9b10607f, 0x8000001f }, /* ESP.VLDEXT.U8.IP */
    { 0xd300607f, 0x5000601b }, /* ESP.VLDEXT.U8.XP */
    { 0x8310607f, 0x8010001f }, /* ESP.VLDHBC.16.INCP */
    { 0x03f7f87f, 0x0346181f }, /* ESP.VMAX.S16 */
    { 0x02f0207f, 0x0060005f }, /* ESP.VMAX.S16.LD.INCP */
    { 0x02f0207f, 0x00e0005f }, /* ESP.VMAX.S16.ST.INCP */
    { 0x03f5f87f, 0x0345181f }, /* ESP.VMAX.S32 */
    { 0x02d0207f, 0x0050005f }, /* ESP.VMAX.S32.LD.INCP */
    { 0x02d0207f, 0x00d0005f }, /* ESP.VMAX.S32.ST.INCP */
    { 0x03f7f87f, 0x0342181f }, /* ESP.VMAX.S8 */
    { 0x02f0207f, 0x0020005f }, /* ESP.VMAX.S8.LD.INCP */
    { 0x02f0207f, 0x00a0005f }, /* ESP.VMAX.S8.ST.INCP */
    { 0x03f7f87f, 0x0344181f }, /* ESP.VMAX.U16 */
    { 0x02f0207f, 0x0040005f }, /* ESP.VMAX.U16.LD.INCP */
    { 0x02f0207f, 0x00c0005f }, /* ESP.VMAX.U16.ST.INCP */
    { 0x03f5f87f, 0x0341181f }, /* ESP.VMAX.U32 */
    { 0x02d0207f, 0x0010005f }, /* ESP.VMAX.U32.LD.INCP */
    { 0x02d0207f, 0x0090005f }, /* ESP.VMAX.U32.ST.INCP */
    { 0x03f7f87f, 0x0340181f }, /* ESP.VMAX.U8 */
    { 0x02f0207f, 0x0000005f }, /* ESP.VMAX.U8.LD.INCP */
    { 0x02f0207f, 0x0080005f }, /* ESP.VMAX.U8.ST.INCP */
    { 0x03f7f87f, 0x0346981f }, /* ESP.VMIN.S16 */
    { 0x02f0207f, 0x0260005f }, /* ESP.VMIN.S16.LD.INCP */
    { 0x02f0207f, 0x02e0005f }, /* ESP.VMIN.S16.ST.INCP */
    { 0x03f5f87f, 0x0345981f }, /* ESP.VMIN.S32 */
    { 0x02d0207f, 0x0250005f }, /* ESP.VMIN.S32.LD.INCP */
    { 0x02d0207f, 0x02d0005f }, /* ESP.VMIN.S32.ST.INCP */
    { 0x03f7f87f, 0x0342981f }, /* ESP.VMIN.S8 */
    { 0x02f0207f, 0x0220005f }, /* ESP.VMIN.S8.LD.INCP */
    { 0x02f0207f, 0x02a0005f }, /* ESP.VMIN.S8.ST.INCP */
    { 0x03f7f87f, 0x0344981f }, /* ESP.VMIN.U16 */
    { 0x02f0207f, 0x0240005f }, /* ESP.VMIN.U16.LD.INCP */
    { 0x02f0207f, 0x02c0005f }, /* ESP.VMIN.U16.ST.INCP */
    { 0x03f5f87f, 0x0341981f }, /* ESP.VMIN.U32 */
    { 0x02d0207f, 0x0210005f }, /* ESP.VMIN.U32.LD.INCP */
    { 0x02d0207f, 0x0290005f }, /* ESP.VMIN.U32.ST.INCP */
    { 0x03f7f87f, 0x0340981f }, /* ESP.VMIN.U8 */
    { 0x02f0207f, 0x0200005f }, /* ESP.VMIN.U8.LD.INCP */
    { 0x02f0207f, 0x0280005f }, /* ESP.VMIN.U8.ST.INCP */
    { 0x03f6647f, 0x03c6001f }, /* ESP.VMUL.S16 */
    { 0x0380607f, 0x0180205f }, /* ESP.VMUL.S16.LD.INCP */
    { 0x0380f07f, 0x0280001f }, /* ESP.VMUL.S16.S8XS8 */
    { 0x0380607f, 0x0380205f }, /* ESP.VMUL.S16.ST.INCP */
    { 0x0380f07f, 0x0280801f }, /* ESP.VMUL.S32.S16XS16 */
    { 0x03f6647f, 0x03c2001f }, /* ESP.VMUL.S8 */
    { 0x0380607f, 0x0080205f }, /* ESP.VMUL.S8.LD.INCP */
    { 0x0380607f, 0x0280205f }, /* ESP.VMUL.S8.ST.INCP */
    { 0x03f6647f, 0x03c4001f }, /* ESP.VMUL.U16 */
    { 0x0380607f, 0x0100205f }, /* ESP.VMUL.U16.LD.INCP */
    { 0x0380607f, 0x0300205f }, /* ESP.VMUL.U16.ST.INCP */
    { 0x03f6647f, 0x03c0001f }, /* ESP.VMUL.U8 */
    { 0x0380607f, 0x0000205f }, /* ESP.VMUL.U8.LD.INCP */
    { 0x0380607f, 0x0200205f }, /* ESP.VMUL.U8.ST.INCP */
    { 0x02e7657f, 0x02a7201b }, /* ESP.VMULAS.S16.QACC */
    { 0x03c060ff, 0x02c0601f }, /* ESP.VMULAS.S16.QACC.LD.IP */
    { 0x030862ff, 0x0208221f }, /* ESP.VMULAS.S16.QACC.LD.XP */
    { 0x00c061ff, 0x00c0609f }, /* ESP.VMULAS.S16.QACC.LDBC.INCP */
    { 0x03c060ff, 0x03c0601f }, /* ESP.VMULAS.S16.QACC.ST.IP */
    { 0x030862ff, 0x0308221f }, /* ESP.VMULAS.S16.QACC.ST.XP */
    { 0x02e7657f, 0x02a3201b }, /* ESP.VMULAS.S16.XACC */
    { 0x03c060ff, 0x00c0601f }, /* ESP.VMULAS.S16.XACC.LD.IP */
    { 0x030862ff, 0x0008221f }, /* ESP.VMULAS.S16.XACC.LD.XP */
    { 0x03c060ff, 0x01c0601f }, /* ESP.VMULAS.S16.XACC.ST.IP */
    { 0x030862ff, 0x0108221f }, /* ESP.VMULAS.S16.XACC.ST.XP */
    { 0x02e7657f, 0x02a5201b }, /* ESP.VMULAS.S8.QACC */
    { 0x03c060ff, 0x0240601f }, /* ESP.VMULAS.S8.QACC.LD.IP */
    { 0x030862ff, 0x0200221f }, /* ESP.VMULAS.S8.QACC.LD.XP */
    { 0x00c061ff, 0x0040609f }, /* ESP.VMULAS.S8.QACC.LDBC.INCP */
    { 0x03c060ff, 0x0340601f }, /* ESP.VMULAS.S8.QACC.ST.IP */
    { 0x030862ff, 0x0300221f }, /* ESP.VMULAS.S8.QACC.ST.XP */
    { 0x02e7657f, 0x02a1201b }, /* ESP.VMULAS.S8.XACC */
    { 0x03c060ff, 0x0040601f }, /* ESP.VMULAS.S8.XACC.LD.IP */
    { 0x030862ff, 0x0000221f }, /* ESP.VMULAS.S8.XACC.LD.XP */
    { 0x03c060ff, 0x0140601f }, /* ESP.VMULAS.S8.XACC.ST.IP */
    { 0x030862ff, 0x0100221f }, /* ESP.VMULAS.S8.XACC.ST.XP */
    { 0x02e7657f, 0x02a6201b }, /* ESP.VMULAS.U16.QACC */
    { 0x03c060ff, 0x0280601f }, /* ESP.VMULAS.U16.QACC.LD.IP */
    { 0x030862ff, 0x0208201f }, /* ESP.VMULAS.U16.QACC.LD.XP */
    { 0x00c061ff, 0x0080609f }, /* ESP.VMULAS.U16.QACC.LDBC.INCP */
    { 0x03c060ff, 0x0380601f }, /* ESP.VMULAS.U16.QACC.ST.IP */
    { 0x030862ff, 0x0308201f }, /* ESP.VMULAS.U16.QACC.ST.XP */
    { 0x02e7657f, 0x02a2201b }, /* ESP.VMULAS.U16.XACC */
    { 0x03c060ff, 0x0080601f }, /* ESP.VMULAS.U16.XACC.LD.IP */
    { 0x030862ff, 0x0008201f }, /* ESP.VMULAS.U16.XACC.LD.XP */
    { 0x03c060ff, 0x0180601f }, /* ESP.VMULAS.U16.XACC.ST.IP */
    { 0x030862ff, 0x0108201f }, /* ESP.VMULAS.U16.XACC.ST.XP */
    { 0x02e7657f, 0x02a4201b }, /* ESP.VMULAS.U8.QACC */
    { 0x03c060ff, 0x0200601f }, /* ESP.VMULAS.U8.QACC.LD.IP */
    { 0x030862ff, 0x0200201f }, /* ESP.VMULAS.U8.QACC.LD.XP */
    { 0x00c061ff, 0x0000609f }, /* ESP.VMULAS.U8.QACC.LDBC.INCP */
    { 0x03c060ff, 0x0300601f }, /* ESP.VMULAS.U8.QACC.ST.IP */
    { 0x030862ff, 0x0300201f }, /* ESP.VMULAS.U8.QACC.ST.XP */
    { 0x02e7657f, 0x02a0201b }, /* ESP.VMULAS.U8.XACC */
    { 0x03c060ff, 0x0000601f }, /* ESP.VMULAS.U8.XACC.LD.IP */
    { 0x030862ff, 0x0000201f }, /* ESP.VMULAS.U8.XACC.LD.XP */
    { 0x03c060ff, 0x0100601f }, /* ESP.VMULAS.U8.XACC.ST.IP */
    { 0x030862ff, 0x0100201f }, /* ESP.VMULAS.U8.XACC.ST.XP */
    { 0x03b0607f, 0x03b0001f }, /* ESP.VPRELU.S16 */
    { 0x03b0607f, 0x0330001f }, /* ESP.VPRELU.S8 */
    { 0xc200617f, 0xc200601b }, /* ESP.VRELU.S16 */
    { 0xc200617f, 0xc000601b }, /* ESP.VRELU.S8 */
    { 0x1a80677f, 0x1a80211b }, /* ESP.VSADDS.S16 */
    { 0x1a80677f, 0x0a80211b }, /* ESP.VSADDS.S8 */
    { 0x1a80677f, 0x1280211b }, /* ESP.VSADDS.U16 */
    { 0x1a80677f, 0x0280211b }, /* ESP.VSADDS.U8 */
    { 0x1f00707f, 0x1a00101f }, /* ESP.VSAT.S16 */
    { 0x1700707f, 0x1600101f }, /* ESP.VSAT.S32 */
    { 0x1f00707f, 0x0a00101f }, /* ESP.VSAT.S8 */
    { 0x1f00707f, 0x1200101f }, /* ESP.VSAT.U16 */
    { 0x1700707f, 0x0600101f }, /* ESP.VSAT.U32 */
    { 0x1f00707f, 0x0200101f }, /* ESP.VSAT.U8 */
    { 0x8294627f, 0x8204201b }, /* ESP.VSL.S32 */
    { 0x8294627f, 0x8004201b }, /* ESP.VSL.U32 */
    { 0xc0c1637f, 0xc040611b }, /* ESP.VSLD.16 */
    { 0xc081637f, 0xc001611b }, /* ESP.VSLD.32 */
    { 0xc0c1637f, 0xc000611b }, /* ESP.VSLD.8 */
    { 0x02e0657f, 0x02e0241b }, /* ESP.VSMULAS.S16.QACC */
    { 0x030063ff, 0x0300639f }, /* ESP.VSMULAS.S16.QACC.LD.INCP */
    { 0x02e0657f, 0x02a0241b }, /* ESP.VSMULAS.S8.QACC */
    { 0x030063ff, 0x0100639f }, /* ESP.VSMULAS.S8.QACC.LD.INCP */
    { 0x02e0657f, 0x02c0241b }, /* ESP.VSMULAS.U16.QACC */
    { 0x030063ff, 0x0200639f }, /* ESP.VSMULAS.U16.QACC.LD.INCP */
    { 0x02e0657f, 0x0280241b }, /* ESP.VSMULAS.U8.QACC */
    { 0x030063ff, 0x0000639f }, /* ESP.VSMULAS.U8.QACC.LD.INCP */
    { 0x8294627f, 0x8214201b }, /* ESP.VSR.S32 */
    { 0x8294627f, 0x8014201b }, /* ESP.VSR.U32 */
    { 0xc0c1637f, 0xc0c0611b }, /* ESP.VSRD.16 */
    { 0xc081637f, 0xc081611b }, /* ESP.VSRD.32 */
    { 0xc0c1637f, 0xc080611b }, /* ESP.VSRD.8 */
    { 0x1a80677f, 0x1a80251b }, /* ESP.VSSUBS.S16 */
    { 0x1a80677f, 0x0a80251b }, /* ESP.VSSUBS.S8 */
    { 0x1a80677f, 0x1280251b }, /* ESP.VSSUBS.U16 */
    { 0x1a80677f, 0x0280251b }, /* ESP.VSSUBS.U8 */
    { 0x83f0607f, 0x8310001f }, /* ESP.VST.128.IP */
    { 0xd20862ff, 0x4200621b }, /* ESP.VST.128.XP */
    { 0xc370607f, 0xc360001f }, /* ESP.VST.H.64.IP */
    { 0xd30862ff, 0x4308601b }, /* ESP.VST.H.64.XP */
    { 0xc370607f, 0x8360001f }, /* ESP.VST.L.64.IP */
    { 0xd30862ff, 0x4208601b }, /* ESP.VST.L.64.XP */
    { 0x0286e77f, 0x0286271b }, /* ESP.VSUB.S16 */
    { 0x038063ff, 0x0180429f }, /* ESP.VSUB.S16.LD.INCP */
    { 0x038063ff, 0x0380429f }, /* ESP.VSUB.S16.ST.INCP */
    { 0x0284e77f, 0x0284a71b }, /* ESP.VSUB.S32 */
    { 0x030063ff, 0x0100419f }, /* ESP.VSUB.S32.LD.INCP */
    { 0x030063ff, 0x0300419f }, /* ESP.VSUB.S32.ST.INCP */
    { 0x0286e77f, 0x0282271b }, /* ESP.VSUB.S8 */
    { 0x038063ff, 0x0080429f }, /* ESP.VSUB.S8.LD.INCP */
    { 0x038063ff, 0x0280429f }, /* ESP.VSUB.S8.ST.INCP */
    { 0x0286e77f, 0x0284271b }, /* ESP.VSUB.U16 */
    { 0x038063ff, 0x0100429f }, /* ESP.VSUB.U16.LD.INCP */
    { 0x038063ff, 0x0300429f }, /* ESP.VSUB.U16.ST.INCP */
    { 0x0284e77f, 0x0280a71b }, /* ESP.VSUB.U32 */
    { 0x030063ff, 0x0000419f }, /* ESP.VSUB.U32.LD.INCP */
    { 0x030063ff, 0x0200419f }, /* ESP.VSUB.U32.ST.INCP */
    { 0x0286e77f, 0x0280271b }, /* ESP.VSUB.U8 */
    { 0x038063ff, 0x0000429f }, /* ESP.VSUB.U8.LD.INCP */
    { 0x038063ff, 0x0200429f }, /* ESP.VSUB.U8.ST.INCP */
    { 0x02a7e57f, 0x0286201b }, /* ESP.VUNZIP.16 */
    { 0x02a5e57f, 0x0285201b }, /* ESP.VUNZIP.32 */
    { 0x02a7e57f, 0x0284201b }, /* ESP.VUNZIP.8 */
    { 0x00c4677f, 0x0040271b }, /* ESP.VUNZIPT.16 */
    { 0x00c4677f, 0x0040251b }, /* ESP.VUNZIPT.8 */
    { 0x02a7e57f, 0x0282201b }, /* ESP.VZIP.16 */
    { 0x02a5e57f, 0x0281201b }, /* ESP.VZIP.32 */
    { 0x02a7e57f, 0x0280201b }, /* ESP.VZIP.8 */
    { 0x00c4677f, 0x0040231b }, /* ESP.VZIPT.16 */
    { 0x00c4677f, 0x0040211b }, /* ESP.VZIPT.8 */
    { 0x0384607f, 0x0104001f }, /* ESP.XORQ */
    { 0x8000707f, 0x0000101b }, /* ESP.ZERO.Q */
    { 0x8000737f, 0x0000021b }, /* ESP.ZERO.QACC */
    { 0x8000737f, 0x0000001b }, /* ESP.ZERO.XACC */
};

/*
 * The instruction is a PIE instruction, which mext_ill.PIE_ILL blames when
 * it is illegal because the extension is off.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
bool esp32p4_insn_is_pie(uint32_t insn)
{
    for (int i = 0; i < ARRAY_SIZE(esp32p4_pie_insns); i++) {
        if ((insn & esp32p4_pie_insns[i].mask) == esp32p4_pie_insns[i].match) {
            return true;
        }
    }
    return false;
}

/* The PIE registers' reset values, which the TRM gives as zero for CFG. */
/* [spec:nuos:req:emu.esp32p4.pie] */
void esp32p4_pie_reset(ESP32P4CPU *cpu)
{
    memset(&cpu->pie, 0, sizeof(cpu->pie));
}
