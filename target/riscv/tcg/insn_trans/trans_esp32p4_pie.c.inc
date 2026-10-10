/*
 * Espressif ESP32-P4 HP core: Processor Instruction Extensions (PIE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 4
 * (Processor Instruction Extensions) and 2.7.2.3 (Initialization, Context
 * Switching, and Exceptions).
 *
 * The PIE instructions, decoded from the patterns esp32p4.decode takes
 * from the TRM's encoding figures; the helpers in esp32p4_pie.c carry
 * them out. Bits the figures mark "X" are not decoded.
 *
 * While mext_pie_status.STATE is OFF every PIE instruction is an illegal
 * instruction, which mext_ill records as PIE_ILL (esp32p4_cpu.c). In any
 * other state a PIE instruction that completes makes the state DIRTY; the
 * TB flags carry whether the extension is on and already dirty.
 *
 * Only x8-x15 and x24-x31 are PIE address and data registers: the
 * register fields leave out bit 3, which is always set. A fused
 * instruction whose load and arithmetic, or two of whose results, have
 * the same destination register is ambiguous; it raises the PIE illegal
 * instruction exception (mcause 0x1f, mtval the instruction), as do
 * immediates the TRM leaves undefined.
 */

#ifndef CONFIG_USER_ONLY

/* A PIE register field: register bits 4 and 2:0, bit 3 being one. */
static int pie_ar(int field)
{
    return (field & 7) | 8 | ((field & 8) << 1);
}

/*
 * Start a PIE instruction: illegal while the extension is off. Returns
 * whether to go on translating it.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool pie_begin(DisasContext *ctx)
{
    if (!ctx->pie_en) {
        gen_exception_illegal(ctx);
        return false;
    }
    return true;
}

/* The PIE illegal instruction exception, ending the instruction. */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool pie_illegal(DisasContext *ctx)
{
    tcg_gen_st_i64(tcg_constant_i64(ctx->opcode), tcg_env,
                   offsetof(CPURISCVState, bins));
    generate_exception(ctx, (RISCVException)ESP32P4_EXCP_PIE_ILLEGAL);
    return true;
}

/* A completed PIE instruction makes the extension's state dirty. */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool pie_end(DisasContext *ctx)
{
    if (!ctx->pie_dirty) {
        tcg_gen_st_i32(tcg_constant_i32(ESP32P4_EXT_DIRTY), tcg_env,
                       ESP32P4_ENV_OFFSET(pie_state));
        ctx->pie_dirty = true;
    }
    return true;
}

/*
 * Whether any two of n destination registers are the same, which makes
 * the instruction ambiguous.
 */
static bool pie_clash(int n, ...)
{
    int r[6];
    va_list ap;

    va_start(ap, n);
    for (int i = 0; i < n; i++) {
        r[i] = va_arg(ap, int);
        for (int j = 0; j < i; j++) {
            if (r[i] == r[j]) {
                va_end(ap);
                return true;
            }
        }
    }
    va_end(ap);
    return false;
}

/*
 * The memory side of a PIE instruction: how it accesses memory, the Q
 * register it loads or stores, its address register, and how the address
 * register changes afterwards: by rs2 when rs2 is a register field,
 * otherwise by inc.
 */
typedef struct PieMem {
    int kind;
    int qu;
    int rs1;
    int rs2;
    target_long inc;
} PieMem;

#define PIE_NOMEM ((PieMem){ ESP32P4_PIE_MEM_NONE, 0, -1, -1, 0 })
#define PIE_MEM(kind, qu, rs1, inc) ((PieMem){ (kind), (qu), (rs1), -1, (inc) })
#define PIE_MEMX(kind, qu, rs1, rs2) ((PieMem){ (kind), (qu), (rs1), (rs2), 0 })

static bool pie_mem_loads(const PieMem *m)
{
    return m->kind == ESP32P4_PIE_MEM_LD128 ||
           m->kind == ESP32P4_PIE_MEM_LDBC8 ||
           m->kind == ESP32P4_PIE_MEM_LDBC16;
}

static uint32_t pie_desc_mem(const PieMem *m)
{
    uint32_t desc = FIELD_DP32(0, PIE, MEM, m->kind);

    return FIELD_DP32(desc, PIE, QU, m->qu);
}

static uint32_t pie_op(int esz_bits, bool is_signed, int kind)
{
    uint32_t op = FIELD_DP32(0, PIEOP, ESZ, ctz32(esz_bits / 8));

    op = FIELD_DP32(op, PIEOP, SIGNED, is_signed);
    return FIELD_DP32(op, PIEOP, KIND, kind);
}

static uint32_t pie_op_x(int esz_bits, bool is_signed, int kind, int x)
{
    return FIELD_DP32(pie_op(esz_bits, is_signed, kind), PIEOP, X, x);
}

/* The address register's value, for a memory access. */
static TCGv pie_base(DisasContext *ctx, const PieMem *m)
{
    if (m->rs1 < 0) {
        return ctx->zero;
    }
    decode_save_opc(ctx, 0);
    return get_gpr(ctx, pie_ar(m->rs1), EXT_NONE);
}

/* After the access, add the increment to the address register. */
static void pie_post_inc(DisasContext *ctx, const PieMem *m, TCGv base)
{
    TCGv t;

    if (m->rs1 < 0) {
        return;
    }
    t = tcg_temp_new();
    if (m->rs2 >= 0) {
        tcg_gen_add_tl(t, base, get_gpr(ctx, pie_ar(m->rs2), EXT_NONE));
    } else {
        tcg_gen_addi_tl(t, base, m->inc);
    }
    gen_set_gpr(ctx, pie_ar(m->rs1), t);
}

typedef void PieMemFn(TCGv_env, TCGv_i32, TCGv_i32, TCGv);

/* Call a helper that may access memory, then update the address. */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool pie_call_mem(DisasContext *ctx, PieMemFn *fn, uint32_t desc,
                         uint32_t op, const PieMem *m)
{
    TCGv base = pie_base(ctx, m);
    TCGv old = tcg_temp_new();

    tcg_gen_mov_tl(old, base);
    fn(tcg_env, tcg_constant_i32(desc | pie_desc_mem(m)),
       tcg_constant_i32(op), old);
    pie_post_inc(ctx, m, old);
    return pie_end(ctx);
}

static uint32_t pie_desc(int q0, int q1, int q2, int q3, int q4)
{
    uint32_t d = FIELD_DP32(0, PIE, Q0, q0);

    d = FIELD_DP32(d, PIE, Q1, q1);
    d = FIELD_DP32(d, PIE, Q2, q2);
    d = FIELD_DP32(d, PIE, Q3, q3);
    return FIELD_DP32(d, PIE, Q4, q4);
}

static uint32_t pie_desc_sr(uint32_t d, int sat, int rm, int sel)
{
    d = FIELD_DP32(d, PIE, SAT, sat);
    d = FIELD_DP32(d, PIE, RM, rm);
    return FIELD_DP32(d, PIE, SEL, sel);
}

/* Multiply-accumulate: ESP.VMULAS, ESP.VSMULAS, ESP.VCMULAS. */
static bool gen_pie_mac(DisasContext *ctx, int qx, int qy, int sel, int sat,
                        uint32_t op, PieMem m)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    /* ESP.VSMULAS.*16 selects one of eight elements. */
    if (FIELD_EX32(op, PIEOP, ESZ) == 1 && sel > 7) {
        return pie_illegal(ctx);
    }
    return pie_call_mem(ctx, gen_helper_esp32p4_pie_mac,
                        pie_desc_sr(pie_desc(qx, qy, 0, 0, 0), sat, 0, sel),
                        op, &m);
}

/* Element-wise arithmetic writing qd, with an optional fused access. */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool gen_pie_arith(DisasContext *ctx, PieMemFn *fn, int qd,
                          uint32_t desc, uint32_t op, PieMem m)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    if (pie_mem_loads(&m) && m.qu == qd) {
        return pie_illegal(ctx);
    }
    return pie_call_mem(ctx, fn, desc, op, &m);
}

static bool gen_pie_vaddsub(DisasContext *ctx, int qv, int qx, int qy,
                            int sat, uint32_t op, PieMem m)
{
    return gen_pie_arith(ctx, gen_helper_esp32p4_pie_vaddsub, qv,
                         pie_desc_sr(pie_desc(qv, qx, qy, 0, 0), sat, 0, 0),
                         op, m);
}

static bool gen_pie_vmul(DisasContext *ctx, int qz, int qx, int qy, int sat,
                         int rm, uint32_t op, PieMem m)
{
    return gen_pie_arith(ctx, gen_helper_esp32p4_pie_vmul, qz,
                         pie_desc_sr(pie_desc(qz, qx, qy, 0, 0), sat, rm, 0),
                         op, m);
}

static bool gen_pie_cmul(DisasContext *ctx, int qz, int qx, int qy, int sel,
                         int sat, int rm, uint32_t op, PieMem m)
{
    return gen_pie_arith(ctx, gen_helper_esp32p4_pie_cmul, qz,
                         pie_desc_sr(pie_desc(qz, qx, qy, 0, 0), sat, rm,
                                     sel),
                         op, m);
}

static bool gen_pie_vminmax(DisasContext *ctx, int qz, int qx, int qy,
                            uint32_t op, PieMem m)
{
    return gen_pie_arith(ctx, gen_helper_esp32p4_pie_vminmax, qz,
                         pie_desc(qz, qx, qy, 0, 0), op, m);
}

/* ESP.VMUL.S16.S8XS8 and ESP.VMUL.S32.S16XS16 write qz and qv. */
static bool gen_pie_vmul_wide(DisasContext *ctx, int qz, int qv, int qx,
                              int qy, int rm, uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    if (qz == qv) {
        return pie_illegal(ctx);
    }
    gen_helper_esp32p4_pie_vmul(tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(qz, qx, qy, qv, 0), 0, rm, 0)),
        tcg_constant_i32(op), ctx->zero);
    return pie_end(ctx);
}

typedef void PieDescOpFn(TCGv_env, TCGv_i32, TCGv_i32);

/* An instruction on registers only. */
static bool gen_pie_regs(DisasContext *ctx, PieDescOpFn *fn, uint32_t desc,
                         uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    fn(tcg_env, tcg_constant_i32(desc), tcg_constant_i32(op));
    return pie_end(ctx);
}

/* An instruction on registers whose destinations must differ. */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool gen_pie_regs_dst(DisasContext *ctx, PieDescOpFn *fn,
                             uint32_t desc, uint32_t op, bool clash)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    if (clash) {
        return pie_illegal(ctx);
    }
    fn(tcg_env, tcg_constant_i32(desc), tcg_constant_i32(op));
    return pie_end(ctx);
}

static bool gen_pie_vcmp(DisasContext *ctx, int qz, int qx, int qy,
                         uint32_t op)
{
    return gen_pie_regs(ctx, gen_helper_esp32p4_pie_vcmp,
                        pie_desc(qz, qx, qy, 0, 0), op);
}

static bool gen_pie_logic(DisasContext *ctx, int qz, int qx, int qy,
                          uint32_t op)
{
    return gen_pie_regs(ctx, gen_helper_esp32p4_pie_logic,
                        pie_desc(qz, qx, qy, 0, 0), op);
}

static bool gen_pie_vabs(DisasContext *ctx, int qv, int qy, uint32_t op)
{
    return gen_pie_regs(ctx, gen_helper_esp32p4_pie_vabs,
                        pie_desc(qv, qy, 0, 0, 0), op);
}

static bool gen_pie_vclamp(DisasContext *ctx, int qz, int qx, int sel)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_vclamp(tcg_env, tcg_constant_i32(
        pie_desc_sr(pie_desc(qz, qx, 0, 0, 0), 0, 0, sel)));
    return pie_end(ctx);
}

static bool gen_pie_zero(DisasContext *ctx, int qz, uint32_t op)
{
    return gen_pie_regs(ctx, gen_helper_esp32p4_pie_zero,
                        pie_desc(qz, 0, 0, 0, 0), op);
}

static bool gen_pie_mov_qacc(DisasContext *ctx, int qu, uint32_t op)
{
    PieMem m = PIE_MEM(ESP32P4_PIE_MEM_NONE, qu, -1, 0);

    return gen_pie_regs(ctx, gen_helper_esp32p4_pie_mov_qacc,
                        pie_desc_mem(&m), op);
}

/* ESP.VZIP, ESP.VUNZIP: qx, qy; ESP.VZIPT, ESP.VUNZIPT: qx, qy, qw. */
static bool gen_pie_perm(DisasContext *ctx, int q0, int q1, int q2, int nr,
                         uint32_t op)
{
    bool clash = nr == 3 ? pie_clash(3, q0, q1, q2) : q0 == q1;

    return gen_pie_regs_dst(ctx, gen_helper_esp32p4_pie_perm,
                            pie_desc(q0, q1, q2, 0, 0), op, clash);
}

/* A helper on registers taking one or two GPR values. */
static bool gen_pie_vsaddsub(DisasContext *ctx, int qv, int qx, int rs1,
                             int sat, uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_vsaddsub(tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(qv, qx, 0, 0, 0), sat, 0, 0)),
        tcg_constant_i32(op), get_gpr(ctx, pie_ar(rs1), EXT_NONE));
    return pie_end(ctx);
}

static bool gen_pie_vsat(DisasContext *ctx, int qz, int qx, int rs1, int rs2,
                         uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_vsat(tcg_env,
        tcg_constant_i32(pie_desc(qz, qx, 0, 0, 0)), tcg_constant_i32(op),
        get_gpr(ctx, pie_ar(rs1), EXT_NONE),
        get_gpr(ctx, pie_ar(rs2), EXT_NONE));
    return pie_end(ctx);
}

static bool gen_pie_vrelu(DisasContext *ctx, int qy, int rs2, int rs1,
                          int sat, int rm, uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_vrelu(tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(qy, 0, 0, 0, 0), sat, rm, 0)),
        tcg_constant_i32(op), get_gpr(ctx, pie_ar(rs2), EXT_NONE),
        get_gpr(ctx, pie_ar(rs1), EXT_NONE));
    return pie_end(ctx);
}

static bool gen_pie_vprelu(DisasContext *ctx, int qz, int qy, int qx,
                           int rs1, int sat, int rm, uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_vprelu(tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(qz, qy, qx, 0, 0), sat, rm,
                                     0)),
        tcg_constant_i32(op), get_gpr(ctx, pie_ar(rs1), EXT_NONE));
    return pie_end(ctx);
}

/* A helper returning a GPR result. */
static bool pie_set_rd(DisasContext *ctx, int rd, TCGv v)
{
    gen_set_gpr(ctx, pie_ar(rd), v);
    return pie_end(ctx);
}

static bool gen_pie_minmax_a(DisasContext *ctx, int qw, int rd, uint32_t op)
{
    TCGv t;

    if (!pie_begin(ctx)) {
        return true;
    }
    t = tcg_temp_new();
    gen_helper_esp32p4_pie_minmax_a(t, tcg_env,
                                    tcg_constant_i32(pie_desc(qw, 0, 0, 0, 0)),
                                    tcg_constant_i32(op));
    return pie_set_rd(ctx, rd, t);
}

static bool gen_pie_movx_r(DisasContext *ctx, int rd, uint32_t op)
{
    TCGv t;

    if (!pie_begin(ctx)) {
        return true;
    }
    t = tcg_temp_new();
    gen_helper_esp32p4_pie_movx_r(t, tcg_env, tcg_constant_i32(op));
    return pie_set_rd(ctx, rd, t);
}

static bool gen_pie_movx_w(DisasContext *ctx, int rs1, uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_movx_w(tcg_env, tcg_constant_i32(op),
                                  get_gpr(ctx, pie_ar(rs1), EXT_NONE));
    return pie_end(ctx);
}

/*
 * ESP.MOVI: element sel of qy; sel4 or sel16 selects one of the four
 * words, eight halfwords or sixteen bytes, the 16-bit forms' sel16 above
 * seven being undefined.
 */
static bool gen_pie_movi_a(DisasContext *ctx, int qy, int rd, int sel,
                           uint32_t op)
{
    TCGv t;

    if (!pie_begin(ctx)) {
        return true;
    }
    if (sel >= 128 >> (3 + FIELD_EX32(op, PIEOP, ESZ))) {
        return pie_illegal(ctx);
    }
    t = tcg_temp_new();
    gen_helper_esp32p4_pie_movi_a(t, tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(qy, 0, 0, 0, 0), 0, 0, sel)),
        tcg_constant_i32(op));
    return pie_set_rd(ctx, rd, t);
}

static bool gen_pie_movi_q(DisasContext *ctx, int qy, int rs1, int sel,
                           uint32_t op)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    if (sel >= 128 >> (3 + FIELD_EX32(op, PIEOP, ESZ))) {
        return pie_illegal(ctx);
    }
    gen_helper_esp32p4_pie_movi_q(tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(qy, 0, 0, 0, 0), 0, 0, sel)),
        tcg_constant_i32(op), get_gpr(ctx, pie_ar(rs1), EXT_NONE));
    return pie_end(ctx);
}

static bool gen_pie_srs(DisasContext *ctx, int rd, int rs1, int sat, int rm,
                        uint32_t op)
{
    TCGv t;

    if (!pie_begin(ctx)) {
        return true;
    }
    t = tcg_temp_new();
    gen_helper_esp32p4_pie_srs(t, tcg_env,
        tcg_constant_i32(pie_desc_sr(0, sat, rm, 0)), tcg_constant_i32(op),
        get_gpr(ctx, pie_ar(rs1), EXT_NONE));
    return pie_set_rd(ctx, rd, t);
}

/* ESP.SRCMB: the shift from rs1 (rs1 >= 0) or from qw's elements. */
static bool gen_pie_srcmb(DisasContext *ctx, int qu, int qw, int rs1, int sat,
                          int rm, uint32_t op)
{
    PieMem m = PIE_MEM(ESP32P4_PIE_MEM_NONE, qu, -1, 0);

    if (!pie_begin(ctx)) {
        return true;
    }
    gen_helper_esp32p4_pie_srcmb(tcg_env,
        tcg_constant_i32(pie_desc_sr(pie_desc(0, qw, 0, 0, 0), sat, rm, 0) |
                         pie_desc_mem(&m)),
        tcg_constant_i32(op),
        rs1 >= 0 ? get_gpr(ctx, pie_ar(rs1), EXT_NONE) : ctx->zero);
    return pie_end(ctx);
}

static bool gen_pie_vshift(DisasContext *ctx, int qu, int qy, int qw, int sat,
                           int rm, uint32_t op)
{
    return gen_pie_regs(ctx, gen_helper_esp32p4_pie_vshift,
                        pie_desc_sr(pie_desc(qu, qy, qw, 0, 0), sat, rm, 0),
                        op);
}

/* Loads, stores and ESP.LDXQ.32/ESP.STXQ.32. */
static bool gen_pie_load(DisasContext *ctx, int q0, uint32_t op, PieMem m)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    return pie_call_mem(ctx, gen_helper_esp32p4_pie_load,
                        pie_desc(q0, 0, 0, 0, 0), op, &m);
}

/* ESP.VLDEXT and ESP.VLDHBC load both qu and qz. */
static bool gen_pie_load2(DisasContext *ctx, int qz, uint32_t op, PieMem m)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    if (qz == m.qu) {
        return pie_illegal(ctx);
    }
    return pie_call_mem(ctx, gen_helper_esp32p4_pie_load,
                        pie_desc(qz, 0, 0, 0, 0), op, &m);
}

static bool gen_pie_store(DisasContext *ctx, uint32_t op, PieMem m)
{
    if (!pie_begin(ctx)) {
        return true;
    }
    return pie_call_mem(ctx, gen_helper_esp32p4_pie_store, 0, op, &m);
}

static bool gen_pie_xq(DisasContext *ctx, int qu, int qw, int rs1, int sel4,
                       int sel8, bool store)
{
    PieMem m = PIE_MEM(ESP32P4_PIE_MEM_NONE, qu, rs1, 0);
    uint32_t desc = FIELD_DP32(pie_desc(qw, 0, 0, 0, 0), PIE, SEL, sel8);

    if (!pie_begin(ctx)) {
        return true;
    }
    desc = FIELD_DP32(desc, PIE, SELB, sel4);
    gen_helper_esp32p4_pie_xq(tcg_env,
                              tcg_constant_i32(desc | pie_desc_mem(&m)),
                              tcg_constant_i32(pie_op(32, false, store)),
                              pie_base(ctx, &m));
    return pie_end(ctx);
}

/*
 * The concatenated shifts. count is the shift's source for the .2Q
 * forms: an immediate (rs1 < 0) or rs1's low bits.
 */
static bool gen_pie_src(DisasContext *ctx, int qz, int qw, int qy, int kind,
                        int count, int rs1, PieMem m)
{
    TCGv base, old, cnt;

    if (!pie_begin(ctx)) {
        return true;
    }
    switch (kind) {
    case PIE_SRC_Q_QUP:
        if (qz == qw) {
            return pie_illegal(ctx);
        }
        break;
    case PIE_SRC_Q_LD:
        if (m.qu == qw) {
            return pie_illegal(ctx);
        }
        break;
    case PIE_SRC_SLC:
    case PIE_SRC_SRC:
        if (qy == qw) {
            return pie_illegal(ctx);
        }
        break;
    default:
        break;
    }
    if (rs1 >= 0) {
        cnt = get_gpr(ctx, pie_ar(rs1), EXT_NONE);
    } else {
        cnt = tcg_constant_tl(count);
    }
    base = pie_base(ctx, &m);
    old = tcg_temp_new();
    tcg_gen_mov_tl(old, base);
    gen_helper_esp32p4_pie_src(tcg_env,
        tcg_constant_i32(pie_desc(qz, qw, qy, 0, 0) | pie_desc_mem(&m)),
        tcg_constant_i32(pie_op(8, false, kind)), old, cnt);
    pie_post_inc(ctx, &m, old);
    return pie_end(ctx);
}

/* FFT */

static bool gen_pie_r2bf(DisasContext *ctx, int qz, int qv, int qx, int qy,
                         int sel, int sat, PieMem m)
{
    bool st = m.kind != ESP32P4_PIE_MEM_NONE;

    if (!pie_begin(ctx)) {
        return true;
    }
    if (!st && qz == qv) {
        return pie_illegal(ctx);
    }
    m.kind = ESP32P4_PIE_MEM_NONE;
    return pie_call_mem(ctx, gen_helper_esp32p4_pie_fft_r2bf,
                        pie_desc_sr(pie_desc(qz, qv, qx, qy, 0), sat, 0, sel),
                        pie_op(16, true, st), &m);
}

/* ESP.FFT.CMUL.S16.LD.XP (st false) and .ST.XP. */
static bool gen_pie_fft_cmul(DisasContext *ctx, int qz, int qx, int qy,
                             int sel8, int upd4, int sel4, int sat, bool st,
                             PieMem m)
{
    uint32_t desc, op;

    if (!pie_begin(ctx)) {
        return true;
    }
    if (st ? upd4 == 3 : m.qu == qz) {
        return pie_illegal(ctx);
    }
    desc = pie_desc_sr(pie_desc(qz, qx, qy, 0, 0), sat, 0, sel8);
    desc = FIELD_DP32(desc, PIE, SELB, upd4);
    op = FIELD_DP32(pie_op(16, true, st), PIEOP, X, sel4);
    if (st) {
        m.kind = ESP32P4_PIE_MEM_NONE;
    }
    return pie_call_mem(ctx, gen_helper_esp32p4_pie_fft_cmul, desc, op, &m);
}

/*
 * ESP.FFT.AMS.S16.*: the loading forms write qu, qz and qv; the storing
 * form qz, rs2 and rs1.
 */
/* [spec:nuos:req:emu.esp32p4.pie] */
static bool gen_pie_fft_ams(DisasContext *ctx, int kind, int qu, int qz,
                            int qv, int qx, int qw, int qy, int rs1, int rs2,
                            int sel, int sat)
{
    PieMem m = PIE_MEM(ESP32P4_PIE_MEM_NONE, qu, rs1,
                       kind == PIE_AMS_LD_R32_DECP ? -16 : 16);
    TCGv base, old, nrs2;
    uint32_t desc;

    if (!pie_begin(ctx)) {
        return true;
    }
    if (kind == PIE_AMS_ST_INCP ? rs1 == rs2 : pie_clash(3, qu, qz, qv)) {
        return pie_illegal(ctx);
    }
    desc = pie_desc_sr(pie_desc(qz, qv, qx, qw, qy), sat, 0, sel) |
           pie_desc_mem(&m);
    base = pie_base(ctx, &m);
    old = tcg_temp_new();
    nrs2 = tcg_temp_new();
    tcg_gen_mov_tl(old, base);
    gen_helper_esp32p4_pie_fft_ams(nrs2, tcg_env, tcg_constant_i32(desc),
        tcg_constant_i32(pie_op(16, true, kind)), old,
        kind == PIE_AMS_ST_INCP ? get_gpr(ctx, pie_ar(rs2), EXT_NONE)
                                : ctx->zero);
    if (kind == PIE_AMS_ST_INCP) {
        gen_set_gpr(ctx, pie_ar(rs2), nrs2);
    }
    pie_post_inc(ctx, &m, old);
    return pie_end(ctx);
}

static bool gen_pie_fft_bitrev(DisasContext *ctx, int qv, int rs1)
{
    TCGv v, t;

    if (!pie_begin(ctx)) {
        return true;
    }
    v = get_gpr(ctx, pie_ar(rs1), EXT_NONE);
    gen_helper_esp32p4_pie_fft_bitrev(tcg_env,
        tcg_constant_i32(pie_desc(qv, 0, 0, 0, 0)), v);
    t = tcg_temp_new();
    tcg_gen_addi_tl(t, v, 8);
    gen_set_gpr(ctx, pie_ar(rs1), t);
    return pie_end(ctx);
}

static bool gen_pie_fft_vst_r32(DisasContext *ctx, int qu, int rs1, int sel)
{
    PieMem m = PIE_MEM(ESP32P4_PIE_MEM_NONE, qu, rs1, -16);
    TCGv base, old;

    if (!pie_begin(ctx)) {
        return true;
    }
    base = pie_base(ctx, &m);
    old = tcg_temp_new();
    tcg_gen_mov_tl(old, base);
    gen_helper_esp32p4_pie_fft_vst_r32(tcg_env,
        tcg_constant_i32(pie_desc_sr(0, 0, 0, sel) | pie_desc_mem(&m)), old);
    pie_post_inc(ctx, &m, old);
    return pie_end(ctx);
}

/* ESP.ADDX2, ESP.ADDX4, ESP.SUBX2 and ESP.SUBX4: rs1 +/- (rs2 << sh). */
static bool gen_pie_addx(DisasContext *ctx, arg_pie_rd_rs1_rs2 *a, int sh,
                         bool sub)
{
    TCGv t;

    if (!pie_begin(ctx)) {
        return true;
    }
    t = tcg_temp_new();
    tcg_gen_shli_tl(t, get_gpr(ctx, a->rs2, EXT_NONE), sh);
    if (sub) {
        tcg_gen_sub_tl(t, get_gpr(ctx, a->rs1, EXT_NONE), t);
    } else {
        tcg_gen_add_tl(t, get_gpr(ctx, a->rs1, EXT_NONE), t);
    }
    gen_set_gpr(ctx, a->rd, t);
    return pie_end(ctx);
}

/* ESP.SAT rsd, rs0, rs1: rsd in the rs2 field, rs0 in the rd field. */
static bool gen_pie_sat(DisasContext *ctx, arg_pie_rd_rs1_rs2 *a)
{
    TCGv t;

    if (!pie_begin(ctx)) {
        return true;
    }
    t = tcg_temp_new();
    gen_helper_esp32p4_pie_sat(t, get_gpr(ctx, a->rs2, EXT_NONE),
                               get_gpr(ctx, a->rd, EXT_NONE),
                               get_gpr(ctx, a->rs1, EXT_NONE));
    gen_set_gpr(ctx, a->rs2, t);
    return pie_end(ctx);
}

#define PIE_TRANS(NAME, ARGS, ...)                                  \
    static bool trans_##NAME(DisasContext *ctx, arg_##ARGS *a)      \
    {                                                               \
        return __VA_ARGS__;                                         \
    }

#else

#define PIE_TRANS(NAME, ARGS, ...)                                  \
    static bool trans_##NAME(DisasContext *ctx, arg_##ARGS *a)      \
    {                                                               \
        return false;                                               \
    }

#endif

PIE_TRANS(esp_addx2, pie_rd_rs1_rs2,
          gen_pie_addx(ctx, a, 1, false))
PIE_TRANS(esp_addx4, pie_rd_rs1_rs2,
          gen_pie_addx(ctx, a, 2, false))
PIE_TRANS(esp_andq, pie_qx_qy_qz,
          gen_pie_logic(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 0)))
PIE_TRANS(esp_cmul_s16, pie_qx_qy_qz_rm_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(16, true, 0), PIE_NOMEM))
PIE_TRANS(esp_cmul_s16_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(16, true, 0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_s16_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(16, true, 0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_s8, pie_qx_qy_qz_rm_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(8, true, 0), PIE_NOMEM))
PIE_TRANS(esp_cmul_s8_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(8, true, 0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_s8_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(8, true, 0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_u16, pie_qx_qy_qz_rm_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(16, false, 0), PIE_NOMEM))
PIE_TRANS(esp_cmul_u16_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(16, false, 0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_u16_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(16, false, 0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_u8, pie_qx_qy_qz_rm_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(8, false, 0), PIE_NOMEM))
PIE_TRANS(esp_cmul_u8_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(8, false, 0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_cmul_u8_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat_sel4,
          gen_pie_cmul(ctx, a->qz, a->qx, a->qy, a->sel4, a->sat, a->rm,
          pie_op(8, false, 0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          16)))
PIE_TRANS(esp_fft_ams_s16_ld_incp, pie_qu_qv_qw_qx_qy_qz_rs1_sat_sel2,
          gen_pie_fft_ams(ctx, PIE_AMS_LD_INCP, a->qu, a->qz, a->qv, a->qx,
          a->qw, a->qy, a->rs1, -1, a->sel2, a->sat))
PIE_TRANS(esp_fft_ams_s16_ld_incp_uaup, pie_qu_qv_qw_qx_qy_qz_rs1_sat_sel2,
          gen_pie_fft_ams(ctx, PIE_AMS_LD_INCP_UAUP, a->qu, a->qz, a->qv,
          a->qx, a->qw, a->qy, a->rs1, -1, a->sel2, a->sat))
PIE_TRANS(esp_fft_ams_s16_ld_r32_decp, pie_qu_qv_qw_qx_qy_qz_rs1_sat_sel2,
          gen_pie_fft_ams(ctx, PIE_AMS_LD_R32_DECP, a->qu, a->qz, a->qv,
          a->qx, a->qw, a->qy, a->rs1, -1, a->sel2, a->sat))
PIE_TRANS(esp_fft_ams_s16_st_incp, pie_qu_qw_qx_qy_qz_rs1_rs2_sat_sel2,
          gen_pie_fft_ams(ctx, PIE_AMS_ST_INCP, a->qu, a->qz, 0, a->qx, a->qw,
          a->qy, a->rs1, a->rs2, a->sel2, a->sat))
PIE_TRANS(esp_fft_bitrev, pie_qv_rs1,
          gen_pie_fft_bitrev(ctx, a->qv, a->rs1))
PIE_TRANS(esp_fft_cmul_s16_ld_xp, pie_qu_qx_qy_qz_rs1_rs2_sat_sel8,
          gen_pie_fft_cmul(ctx, a->qz, a->qx, a->qy, a->sel8, 0, 0, a->sat,
          false, PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_fft_cmul_s16_st_xp, pie_qu_qx_qy_rs1_rs2_sat_sel4_sel8_upd4,
          gen_pie_fft_cmul(ctx, 0, a->qx, a->qy, a->sel8, a->upd4, a->sel4,
          a->sat, true, PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_fft_r2bf_s16, pie_qv_qx_qy_qz_sat_sel2,
          gen_pie_r2bf(ctx, a->qz, a->qv, a->qx, a->qy, a->sel2, a->sat,
          PIE_NOMEM))
PIE_TRANS(esp_fft_r2bf_s16_st_incp, pie_qx_qy_qz_rs1_sat_sel4,
          gen_pie_r2bf(ctx, a->qz, 0, a->qx, a->qy, a->sel4, a->sat,
          PIE_MEM(ESP32P4_PIE_MEM_ST128, 0, a->rs1, 16)))
PIE_TRANS(esp_fft_vst_r32_decp, pie_qu_rs1_sel2,
          gen_pie_fft_vst_r32(ctx, a->qu, a->rs1, a->sel2))
PIE_TRANS(esp_ld_128_usar_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_USAR),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_ld_128_usar_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_USAR),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_ld_qacc_h_h_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op_x(8, false, PIE_LD_QACC, 3),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ld_qacc_h_l_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op_x(8, false, PIE_LD_QACC, 2),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ld_qacc_l_h_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op_x(8, false, PIE_LD_QACC, 1),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ld_qacc_l_l_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op_x(8, false, PIE_LD_QACC, 0),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ld_ua_state_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_UA_STATE),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ld_xacc_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_XACC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ldqa_s16_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op(16, true, PIE_LD_QA),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ldqa_s16_128_xp, pie_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(16, true, PIE_LD_QA),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->rs2)))
PIE_TRANS(esp_ldqa_s8_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op(8, true, PIE_LD_QA),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ldqa_s8_128_xp, pie_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, true, PIE_LD_QA),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->rs2)))
PIE_TRANS(esp_ldqa_u16_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op(16, false, PIE_LD_QA),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ldqa_u16_128_xp, pie_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(16, false, PIE_LD_QA),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->rs2)))
PIE_TRANS(esp_ldqa_u8_128_ip, pie_imm_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_QA),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_ldqa_u8_128_xp, pie_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_QA),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->rs2)))
PIE_TRANS(esp_ldxq_32, pie_qu_qw_rs1_sel4_sel8,
          gen_pie_xq(ctx, a->qu, a->qw, a->rs1, a->sel4, a->sel8, false))
PIE_TRANS(esp_max_s16_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(16, true, 0)))
PIE_TRANS(esp_max_s32_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(32, true, 0)))
PIE_TRANS(esp_max_s8_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(8, true, 0)))
PIE_TRANS(esp_max_u16_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(16, false, 0)))
PIE_TRANS(esp_max_u32_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(32, false, 0)))
PIE_TRANS(esp_max_u8_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(8, false, 0)))
PIE_TRANS(esp_min_s16_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(16, true, 1)))
PIE_TRANS(esp_min_s32_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(32, true, 1)))
PIE_TRANS(esp_min_s8_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(8, true, 1)))
PIE_TRANS(esp_min_u16_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(16, false, 1)))
PIE_TRANS(esp_min_u32_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(32, false, 1)))
PIE_TRANS(esp_min_u8_a, pie_qw_rd,
          gen_pie_minmax_a(ctx, a->qw, a->rd, pie_op(8, false, 1)))
PIE_TRANS(esp_mov_s16_qacc, pie_qu,
          gen_pie_mov_qacc(ctx, a->qu, pie_op(16, true, 0)))
PIE_TRANS(esp_mov_s8_qacc, pie_qu,
          gen_pie_mov_qacc(ctx, a->qu, pie_op(8, true, 0)))
PIE_TRANS(esp_mov_u16_qacc, pie_qu,
          gen_pie_mov_qacc(ctx, a->qu, pie_op(16, false, 0)))
PIE_TRANS(esp_mov_u8_qacc, pie_qu,
          gen_pie_mov_qacc(ctx, a->qu, pie_op(8, false, 0)))
PIE_TRANS(esp_movi_16_a, pie_qy_rd_sel16,
          gen_pie_movi_a(ctx, a->qy, a->rd, a->sel16, pie_op(16, false, 0)))
PIE_TRANS(esp_movi_16_q, pie_qy_rs1_sel16,
          gen_pie_movi_q(ctx, a->qy, a->rs1, a->sel16, pie_op(16, false, 0)))
PIE_TRANS(esp_movi_32_a, pie_qy_rd_sel4,
          gen_pie_movi_a(ctx, a->qy, a->rd, a->sel4, pie_op(32, false, 0)))
PIE_TRANS(esp_movi_32_q, pie_qy_rs1_sel4,
          gen_pie_movi_q(ctx, a->qy, a->rs1, a->sel4, pie_op(32, false, 0)))
PIE_TRANS(esp_movi_8_a, pie_qy_rd_sel16,
          gen_pie_movi_a(ctx, a->qy, a->rd, a->sel16, pie_op(8, false, 0)))
PIE_TRANS(esp_movi_8_q, pie_qy_rs1_sel16,
          gen_pie_movi_q(ctx, a->qy, a->rs1, a->sel16, pie_op(8, false, 0)))
PIE_TRANS(esp_movx_r_cfg, pie_rd,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_CFG)))
PIE_TRANS(esp_movx_r_fft_bit_width, pie_rd,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_FFT_BIT_WIDTH)))
PIE_TRANS(esp_movx_r_perf, pie_rd_rs1,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_PERF)))
PIE_TRANS(esp_movx_r_sar, pie_rd,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_SAR)))
PIE_TRANS(esp_movx_r_sar_bytes, pie_rd,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_SAR_BYTE)))
PIE_TRANS(esp_movx_r_xacc_h, pie_rd,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_XACC_H)))
PIE_TRANS(esp_movx_r_xacc_l, pie_rd,
          gen_pie_movx_r(ctx, a->rd, pie_op(32, false, PIE_SR_XACC_L)))
PIE_TRANS(esp_movx_w_cfg, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_CFG)))
PIE_TRANS(esp_movx_w_fft_bit_width, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_FFT_BIT_WIDTH)))
PIE_TRANS(esp_movx_w_perf, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_PERF)))
PIE_TRANS(esp_movx_w_sar, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_SAR)))
PIE_TRANS(esp_movx_w_sar_bytes, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_SAR_BYTE)))
PIE_TRANS(esp_movx_w_xacc_h, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_XACC_H)))
PIE_TRANS(esp_movx_w_xacc_l, pie_rs1,
          gen_pie_movx_w(ctx, a->rs1, pie_op(32, false, PIE_SR_XACC_L)))
PIE_TRANS(esp_notq, pie_qx_qz,
          gen_pie_logic(ctx, a->qz, a->qx, 0, pie_op(8, false, 3)))
PIE_TRANS(esp_orq, pie_qx_qy_qz,
          gen_pie_logic(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 1)))
PIE_TRANS(esp_sat, pie_rd_rs1_rs2,
          gen_pie_sat(ctx, a))
PIE_TRANS(esp_slci_2q, pie_qw_qy_sel16,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_SLC, a->sel16, -1,
          PIE_NOMEM))
PIE_TRANS(esp_slcxxp_2q, pie_qw_qy_rs1_rs2,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_SLC, 0, a->rs1,
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->rs2)))
PIE_TRANS(esp_src_q, pie_qw_qy_qz,
          gen_pie_src(ctx, a->qz, a->qw, a->qy, PIE_SRC_Q, 0, -1, PIE_NOMEM))
PIE_TRANS(esp_src_q_ld_ip, pie_imm_qu_qw_qy_rs1,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_Q_LD, 0, -1,
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_src_q_ld_xp, pie_qu_qw_qy_rs1_rs2,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_Q_LD, 0, -1,
          PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_src_q_qup, pie_qw_qy_qz,
          gen_pie_src(ctx, a->qz, a->qw, a->qy, PIE_SRC_Q_QUP, 0, -1,
          PIE_NOMEM))
PIE_TRANS(esp_srci_2q, pie_qw_qy_sel16,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_SRC, a->sel16, -1,
          PIE_NOMEM))
PIE_TRANS(esp_srcmb_s16_q_qacc, pie_qu_qw_rm_sat,
          gen_pie_srcmb(ctx, a->qu, a->qw, -1, a->sat, a->rm, pie_op(16, true,
          1)))
PIE_TRANS(esp_srcmb_s16_qacc, pie_qu_rm_rs1_sat,
          gen_pie_srcmb(ctx, a->qu, 0, a->rs1, a->sat, a->rm, pie_op(16, true,
          0)))
PIE_TRANS(esp_srcmb_s8_q_qacc, pie_qu_qw_rm_sat,
          gen_pie_srcmb(ctx, a->qu, a->qw, -1, a->sat, a->rm, pie_op(8, true,
          1)))
PIE_TRANS(esp_srcmb_s8_qacc, pie_qu_rm_rs1_sat,
          gen_pie_srcmb(ctx, a->qu, 0, a->rs1, a->sat, a->rm, pie_op(8, true,
          0)))
PIE_TRANS(esp_srcmb_u16_q_qacc, pie_qu_qw_rm_sat,
          gen_pie_srcmb(ctx, a->qu, a->qw, -1, a->sat, a->rm, pie_op(16,
          false, 1)))
PIE_TRANS(esp_srcmb_u16_qacc, pie_qu_rm_rs1_sat,
          gen_pie_srcmb(ctx, a->qu, 0, a->rs1, a->sat, a->rm, pie_op(16,
          false, 0)))
PIE_TRANS(esp_srcmb_u8_q_qacc, pie_qu_qw_rm_sat,
          gen_pie_srcmb(ctx, a->qu, a->qw, -1, a->sat, a->rm, pie_op(8, false,
          1)))
PIE_TRANS(esp_srcmb_u8_qacc, pie_qu_rm_rs1_sat,
          gen_pie_srcmb(ctx, a->qu, 0, a->rs1, a->sat, a->rm, pie_op(8, false,
          0)))
PIE_TRANS(esp_srcq_128_st_incp, pie_qw_qy_rs1,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_ST, 0, -1,
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, 16)))
PIE_TRANS(esp_srcxxp_2q, pie_qw_qy_rs1_rs2,
          gen_pie_src(ctx, 0, a->qw, a->qy, PIE_SRC_SRC, 0, a->rs1,
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->rs2)))
PIE_TRANS(esp_srs_s_xacc, pie_rd_rm_rs1_sat,
          gen_pie_srs(ctx, a->rd, a->rs1, a->sat, a->rm, pie_op(32, true, 0)))
PIE_TRANS(esp_srs_u_xacc, pie_rd_rm_rs1_sat,
          gen_pie_srs(ctx, a->rd, a->rs1, a->sat, a->rm, pie_op(32, false, 0)))
PIE_TRANS(esp_st_qacc_h_h_128_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op_x(8, false, PIE_ST_QACC, 3),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_st_qacc_h_l_128_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op_x(8, false, PIE_ST_QACC, 2),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_st_qacc_l_h_128_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op_x(8, false, PIE_ST_QACC, 1),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_st_qacc_l_l_128_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op_x(8, false, PIE_ST_QACC, 0),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_st_s_xacc_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op(8, true, PIE_ST_XACC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_st_u_xacc_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_XACC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_st_ua_state_ip, pie_imm_rs1,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_UA_STATE),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, 0, a->rs1, a->imm)))
PIE_TRANS(esp_stxq_32, pie_qu_qw_rs1_sel4_sel8,
          gen_pie_xq(ctx, a->qu, a->qw, a->rs1, a->sel4, a->sel8, true))
PIE_TRANS(esp_subx2, pie_rd_rs1_rs2,
          gen_pie_addx(ctx, a, 1, true))
PIE_TRANS(esp_subx4, pie_rd_rs1_rs2,
          gen_pie_addx(ctx, a, 2, true))
PIE_TRANS(esp_vabs_16, pie_qv_qy,
          gen_pie_vabs(ctx, a->qv, a->qy, pie_op(16, true, 0)))
PIE_TRANS(esp_vabs_32, pie_qv_qy,
          gen_pie_vabs(ctx, a->qv, a->qy, pie_op(32, true, 0)))
PIE_TRANS(esp_vabs_8, pie_qv_qy,
          gen_pie_vabs(ctx, a->qv, a->qy, pie_op(8, true, 0)))
PIE_TRANS(esp_vadd_s16, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, true,
          0), PIE_NOMEM))
PIE_TRANS(esp_vadd_s16_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, true,
          0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_s16_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, true,
          0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_s32, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, true,
          0), PIE_NOMEM))
PIE_TRANS(esp_vadd_s32_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, true,
          0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_s32_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, true,
          0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_s8, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, true,
          0), PIE_NOMEM))
PIE_TRANS(esp_vadd_s8_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, true,
          0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_s8_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, true,
          0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_u16, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, false,
          0), PIE_NOMEM))
PIE_TRANS(esp_vadd_u16_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, false,
          0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_u16_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, false,
          0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_u32, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, false,
          0), PIE_NOMEM))
PIE_TRANS(esp_vadd_u32_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, false,
          0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_u32_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, false,
          0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_u8, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, false,
          0), PIE_NOMEM))
PIE_TRANS(esp_vadd_u8_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, false,
          0), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vadd_u8_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, false,
          0), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vclamp_s16, pie_qx_qz_sel16,
          gen_pie_vclamp(ctx, a->qz, a->qx, a->sel16))
PIE_TRANS(esp_vcmp_eq_s16, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 0)))
PIE_TRANS(esp_vcmp_eq_s32, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 0)))
PIE_TRANS(esp_vcmp_eq_s8, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 0)))
PIE_TRANS(esp_vcmp_eq_u16, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 0)))
PIE_TRANS(esp_vcmp_eq_u32, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 0)))
PIE_TRANS(esp_vcmp_eq_u8, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 0)))
PIE_TRANS(esp_vcmp_gt_s16, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 2)))
PIE_TRANS(esp_vcmp_gt_s32, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 2)))
PIE_TRANS(esp_vcmp_gt_s8, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 2)))
PIE_TRANS(esp_vcmp_gt_u16, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 2)))
PIE_TRANS(esp_vcmp_gt_u32, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 2)))
PIE_TRANS(esp_vcmp_gt_u8, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 2)))
PIE_TRANS(esp_vcmp_lt_s16, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 1)))
PIE_TRANS(esp_vcmp_lt_s32, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 1)))
PIE_TRANS(esp_vcmp_lt_s8, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 1)))
PIE_TRANS(esp_vcmp_lt_u16, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 1)))
PIE_TRANS(esp_vcmp_lt_u32, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 1)))
PIE_TRANS(esp_vcmp_lt_u8, pie_qx_qy_qz,
          gen_pie_vcmp(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 1)))
PIE_TRANS(esp_vcmulas_s16_qacc_h, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_CPLX_H), PIE_NOMEM))
PIE_TRANS(esp_vcmulas_s16_qacc_h_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_CPLX_H), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vcmulas_s16_qacc_h_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_CPLX_H), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vcmulas_s16_qacc_l, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_CPLX_L), PIE_NOMEM))
PIE_TRANS(esp_vcmulas_s16_qacc_l_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_CPLX_L), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vcmulas_s16_qacc_l_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_CPLX_L), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vcmulas_s8_qacc_h, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_CPLX_H), PIE_NOMEM))
PIE_TRANS(esp_vcmulas_s8_qacc_h_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_CPLX_H), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vcmulas_s8_qacc_h_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_CPLX_H), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vcmulas_s8_qacc_l, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_CPLX_L), PIE_NOMEM))
PIE_TRANS(esp_vcmulas_s8_qacc_l_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_CPLX_L), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vcmulas_s8_qacc_l_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_CPLX_L), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vext_s16, pie_qv_qw_qz,
          gen_pie_perm(ctx, a->qz, a->qv, a->qw, 2, pie_op(16, true,
          PIE_PERM_EXT)))
PIE_TRANS(esp_vext_s8, pie_qv_qw_qz,
          gen_pie_perm(ctx, a->qz, a->qv, a->qw, 2, pie_op(8, true,
          PIE_PERM_EXT)))
PIE_TRANS(esp_vext_u16, pie_qv_qw_qz,
          gen_pie_perm(ctx, a->qz, a->qv, a->qw, 2, pie_op(16, false,
          PIE_PERM_EXT)))
PIE_TRANS(esp_vext_u8, pie_qv_qw_qz,
          gen_pie_perm(ctx, a->qz, a->qv, a->qw, 2, pie_op(8, false,
          PIE_PERM_EXT)))
PIE_TRANS(esp_vld_128_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_Q),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vld_128_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_Q),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vld_h_64_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_H64),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vld_h_64_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_H64),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vld_l_64_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_L64),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vld_l_64_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_L64),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldbc_16_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(16, false, PIE_LD_BC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldbc_16_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(16, false, PIE_LD_BC),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldbc_32_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(32, false, PIE_LD_BC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldbc_32_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(32, false, PIE_LD_BC),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldbc_8_ip, pie_imm_qu_rs1,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_BC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldbc_8_xp, pie_qu_rs1_rs2,
          gen_pie_load(ctx, 0, pie_op(8, false, PIE_LD_BC),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldext_s16_ip, pie_imm_qu_qz_rs1,
          gen_pie_load2(ctx, a->qz, pie_op(16, true, PIE_LD_EXT),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldext_s16_xp, pie_qu_qz_rs1_rs2,
          gen_pie_load2(ctx, a->qz, pie_op(16, true, PIE_LD_EXT),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldext_s8_ip, pie_imm_qu_qz_rs1,
          gen_pie_load2(ctx, a->qz, pie_op(8, true, PIE_LD_EXT),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldext_s8_xp, pie_qu_qz_rs1_rs2,
          gen_pie_load2(ctx, a->qz, pie_op(8, true, PIE_LD_EXT),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldext_u16_ip, pie_imm_qu_qz_rs1,
          gen_pie_load2(ctx, a->qz, pie_op(16, false, PIE_LD_EXT),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldext_u16_xp, pie_qu_qz_rs1_rs2,
          gen_pie_load2(ctx, a->qz, pie_op(16, false, PIE_LD_EXT),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldext_u8_ip, pie_imm_qu_qz_rs1,
          gen_pie_load2(ctx, a->qz, pie_op(8, false, PIE_LD_EXT),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vldext_u8_xp, pie_qu_qz_rs1_rs2,
          gen_pie_load2(ctx, a->qz, pie_op(8, false, PIE_LD_EXT),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vldhbc_16_incp, pie_qu_qz_rs1,
          gen_pie_load2(ctx, a->qz, pie_op(16, false, PIE_LD_HBC),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_s16, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 0),
          PIE_NOMEM))
PIE_TRANS(esp_vmax_s16_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 0),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_s16_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 0),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_s32, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 0),
          PIE_NOMEM))
PIE_TRANS(esp_vmax_s32_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 0),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_s32_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 0),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_s8, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 0),
          PIE_NOMEM))
PIE_TRANS(esp_vmax_s8_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 0),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_s8_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 0),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_u16, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 0),
          PIE_NOMEM))
PIE_TRANS(esp_vmax_u16_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 0),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_u16_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 0),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_u32, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 0),
          PIE_NOMEM))
PIE_TRANS(esp_vmax_u32_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 0),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_u32_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 0),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_u8, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 0),
          PIE_NOMEM))
PIE_TRANS(esp_vmax_u8_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 0),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmax_u8_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 0),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_s16, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 1),
          PIE_NOMEM))
PIE_TRANS(esp_vmin_s16_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 1),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_s16_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, true, 1),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_s32, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 1),
          PIE_NOMEM))
PIE_TRANS(esp_vmin_s32_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 1),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_s32_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, true, 1),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_s8, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 1),
          PIE_NOMEM))
PIE_TRANS(esp_vmin_s8_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 1),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_s8_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, true, 1),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_u16, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 1),
          PIE_NOMEM))
PIE_TRANS(esp_vmin_u16_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 1),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_u16_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(16, false, 1),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_u32, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 1),
          PIE_NOMEM))
PIE_TRANS(esp_vmin_u32_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 1),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_u32_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(32, false, 1),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_u8, pie_qx_qy_qz,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 1),
          PIE_NOMEM))
PIE_TRANS(esp_vmin_u8_ld_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 1),
          PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmin_u8_st_incp, pie_qu_qx_qy_qz_rs1,
          gen_pie_vminmax(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 1),
          PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vmul_s16, pie_qx_qy_qz_rm_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(16,
          true, PIE_VMUL_SAME, 0x3f), PIE_NOMEM))
PIE_TRANS(esp_vmul_s16_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(16,
          true, PIE_VMUL_SAME, 0x3f), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_s16_s8xs8, pie_qv_qx_qy_qz_rm,
          gen_pie_vmul_wide(ctx, a->qz, a->qv, a->qx, a->qy, a->rm,
          pie_op_x(8, true, PIE_VMUL_S8XS8, 0x3f)))
PIE_TRANS(esp_vmul_s16_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(16,
          true, PIE_VMUL_SAME, 0x3f), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_s32_s16xs16, pie_qv_qx_qy_qz_rm,
          gen_pie_vmul_wide(ctx, a->qz, a->qv, a->qx, a->qy, a->rm,
          pie_op_x(16, true, PIE_VMUL_S16XS16, 0x3f)))
PIE_TRANS(esp_vmul_s8, pie_qx_qy_qz_rm_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(8,
          true, PIE_VMUL_SAME, 0x1f), PIE_NOMEM))
PIE_TRANS(esp_vmul_s8_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(8,
          true, PIE_VMUL_SAME, 0x1f), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_s8_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(8,
          true, PIE_VMUL_SAME, 0x1f), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_u16, pie_qx_qy_qz_rm_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(16,
          false, PIE_VMUL_SAME, 0x3f), PIE_NOMEM))
PIE_TRANS(esp_vmul_u16_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(16,
          false, PIE_VMUL_SAME, 0x3f), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_u16_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(16,
          false, PIE_VMUL_SAME, 0x3f), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_u8, pie_qx_qy_qz_rm_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(8,
          false, PIE_VMUL_SAME, 0x3f), PIE_NOMEM))
PIE_TRANS(esp_vmul_u8_ld_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(8,
          false, PIE_VMUL_SAME, 0x3f), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmul_u8_st_incp, pie_qu_qx_qy_qz_rm_rs1_sat,
          gen_pie_vmul(ctx, a->qz, a->qx, a->qy, a->sat, a->rm, pie_op_x(8,
          false, PIE_VMUL_SAME, 0x3f), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu,
          a->rs1, 16)))
PIE_TRANS(esp_vmulas_s16_qacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_QACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_s16_qacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s16_qacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s16_qacc_ldbc_incp, pie_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LDBC16, a->qu, a->rs1, 2)))
PIE_TRANS(esp_vmulas_s16_qacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s16_qacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s16_xacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_XACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_s16_xacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s16_xacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s16_xacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s16_xacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, true,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s8_qacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_QACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_s8_qacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s8_qacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s8_qacc_ldbc_incp, pie_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LDBC8, a->qu, a->rs1, 1)))
PIE_TRANS(esp_vmulas_s8_qacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s8_qacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s8_xacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_XACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_s8_xacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s8_xacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_s8_xacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_s8_xacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, true,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u16_qacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_QACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_u16_qacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u16_qacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u16_qacc_ldbc_incp, pie_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LDBC16, a->qu, a->rs1, 2)))
PIE_TRANS(esp_vmulas_u16_qacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u16_qacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u16_xacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_XACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_u16_xacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u16_xacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u16_xacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u16_xacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(16, false,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u8_qacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_QACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_u8_qacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u8_qacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u8_qacc_ldbc_incp, pie_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_LDBC8, a->qu, a->rs1, 1)))
PIE_TRANS(esp_vmulas_u8_qacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_QACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u8_qacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_QACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u8_xacc, pie_qx_qy_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_XACC), PIE_NOMEM))
PIE_TRANS(esp_vmulas_u8_xacc_ld_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u8_xacc_ld_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vmulas_u8_xacc_st_ip, pie_imm_qu_qx_qy_rs1_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_XACC), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->imm)))
PIE_TRANS(esp_vmulas_u8_xacc_st_xp, pie_qu_qx_qy_rs1_rs2_sat,
          gen_pie_mac(ctx, a->qx, a->qy, 0, a->sat, pie_op(8, false,
          PIE_MAC_XACC), PIE_MEMX(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1,
          a->rs2)))
PIE_TRANS(esp_vprelu_s16, pie_qx_qy_qz_rm_rs1_sat,
          gen_pie_vprelu(ctx, a->qz, a->qy, a->qx, a->rs1, a->sat, a->rm,
          pie_op(16, true, 0)))
PIE_TRANS(esp_vprelu_s8, pie_qx_qy_qz_rm_rs1_sat,
          gen_pie_vprelu(ctx, a->qz, a->qy, a->qx, a->rs1, a->sat, a->rm,
          pie_op(8, true, 0)))
PIE_TRANS(esp_vrelu_s16, pie_qy_rm_rs1_rs2_sat,
          gen_pie_vrelu(ctx, a->qy, a->rs2, a->rs1, a->sat, a->rm, pie_op(16,
          true, 0)))
PIE_TRANS(esp_vrelu_s8, pie_qy_rm_rs1_rs2_sat,
          gen_pie_vrelu(ctx, a->qy, a->rs2, a->rs1, a->sat, a->rm, pie_op(8,
          true, 0)))
PIE_TRANS(esp_vsadds_s16, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(16, true,
          0)))
PIE_TRANS(esp_vsadds_s8, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(8, true,
          0)))
PIE_TRANS(esp_vsadds_u16, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(16,
          false, 0)))
PIE_TRANS(esp_vsadds_u8, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(8, false,
          0)))
PIE_TRANS(esp_vsat_s16, pie_qx_qz_rs1_rs2,
          gen_pie_vsat(ctx, a->qz, a->qx, a->rs1, a->rs2, pie_op(16, true, 0)))
PIE_TRANS(esp_vsat_s32, pie_qx_qz_rs1_rs2,
          gen_pie_vsat(ctx, a->qz, a->qx, a->rs1, a->rs2, pie_op(32, true, 0)))
PIE_TRANS(esp_vsat_s8, pie_qx_qz_rs1_rs2,
          gen_pie_vsat(ctx, a->qz, a->qx, a->rs1, a->rs2, pie_op(8, true, 0)))
PIE_TRANS(esp_vsat_u16, pie_qx_qz_rs1_rs2,
          gen_pie_vsat(ctx, a->qz, a->qx, a->rs1, a->rs2, pie_op(16, false,
          0)))
PIE_TRANS(esp_vsat_u32, pie_qx_qz_rs1_rs2,
          gen_pie_vsat(ctx, a->qz, a->qx, a->rs1, a->rs2, pie_op(32, false,
          0)))
PIE_TRANS(esp_vsat_u8, pie_qx_qz_rs1_rs2,
          gen_pie_vsat(ctx, a->qz, a->qx, a->rs1, a->rs2, pie_op(8, false, 0)))
PIE_TRANS(esp_vsl_s32, pie_qu_qy_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, 0, a->sat, 0, pie_op(32, true,
          PIE_VSHIFT_SL)))
PIE_TRANS(esp_vsl_u32, pie_qu_qy_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, 0, a->sat, 0, pie_op(32, false,
          PIE_VSHIFT_SL)))
PIE_TRANS(esp_vsld_16, pie_qu_qw_qy_rm_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, a->qw, a->sat, a->rm, pie_op(16,
          true, PIE_VSHIFT_SLD)))
PIE_TRANS(esp_vsld_32, pie_qu_qw_qy_rm_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, a->qw, a->sat, a->rm, pie_op(32,
          true, PIE_VSHIFT_SLD)))
PIE_TRANS(esp_vsld_8, pie_qu_qw_qy_rm_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, a->qw, a->sat, a->rm, pie_op(8,
          true, PIE_VSHIFT_SLD)))
PIE_TRANS(esp_vsmulas_s16_qacc, pie_qx_qy_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(16, true,
          PIE_MAC_SCALAR), PIE_NOMEM))
PIE_TRANS(esp_vsmulas_s16_qacc_ld_incp, pie_qu_qx_qy_rs1_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(16, true,
          PIE_MAC_SCALAR), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsmulas_s8_qacc, pie_qx_qy_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(8, true,
          PIE_MAC_SCALAR), PIE_NOMEM))
PIE_TRANS(esp_vsmulas_s8_qacc_ld_incp, pie_qu_qx_qy_rs1_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(8, true,
          PIE_MAC_SCALAR), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsmulas_u16_qacc, pie_qx_qy_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(16, false,
          PIE_MAC_SCALAR), PIE_NOMEM))
PIE_TRANS(esp_vsmulas_u16_qacc_ld_incp, pie_qu_qx_qy_rs1_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(16, false,
          PIE_MAC_SCALAR), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsmulas_u8_qacc, pie_qx_qy_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(8, false,
          PIE_MAC_SCALAR), PIE_NOMEM))
PIE_TRANS(esp_vsmulas_u8_qacc_ld_incp, pie_qu_qx_qy_rs1_sat_sel16,
          gen_pie_mac(ctx, a->qx, a->qy, a->sel16, a->sat, pie_op(8, false,
          PIE_MAC_SCALAR), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsr_s32, pie_qu_qy_rm,
          gen_pie_vshift(ctx, a->qu, a->qy, 0, 0, a->rm, pie_op(32, true,
          PIE_VSHIFT_SR)))
PIE_TRANS(esp_vsr_u32, pie_qu_qy_rm,
          gen_pie_vshift(ctx, a->qu, a->qy, 0, 0, a->rm, pie_op(32, false,
          PIE_VSHIFT_SR)))
PIE_TRANS(esp_vsrd_16, pie_qu_qw_qy_rm_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, a->qw, a->sat, a->rm, pie_op(16,
          true, PIE_VSHIFT_SRD)))
PIE_TRANS(esp_vsrd_32, pie_qu_qw_qy_rm_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, a->qw, a->sat, a->rm, pie_op(32,
          true, PIE_VSHIFT_SRD)))
PIE_TRANS(esp_vsrd_8, pie_qu_qw_qy_rm_sat,
          gen_pie_vshift(ctx, a->qu, a->qy, a->qw, a->sat, a->rm, pie_op(8,
          true, PIE_VSHIFT_SRD)))
PIE_TRANS(esp_vssubs_s16, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(16, true,
          1)))
PIE_TRANS(esp_vssubs_s8, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(8, true,
          1)))
PIE_TRANS(esp_vssubs_u16, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(16,
          false, 1)))
PIE_TRANS(esp_vssubs_u8, pie_qv_qx_rs1_sat,
          gen_pie_vsaddsub(ctx, a->qv, a->qx, a->rs1, a->sat, pie_op(8, false,
          1)))
PIE_TRANS(esp_vst_128_ip, pie_imm_qu_rs1,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_Q),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vst_128_xp, pie_qu_rs1_rs2,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_Q),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vst_h_64_ip, pie_imm_qu_rs1,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_H64),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vst_h_64_xp, pie_qu_rs1_rs2,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_H64),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vst_l_64_ip, pie_imm_qu_rs1,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_L64),
          PIE_MEM(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->imm)))
PIE_TRANS(esp_vst_l_64_xp, pie_qu_rs1_rs2,
          gen_pie_store(ctx, pie_op(8, false, PIE_ST_L64),
          PIE_MEMX(ESP32P4_PIE_MEM_NONE, a->qu, a->rs1, a->rs2)))
PIE_TRANS(esp_vsub_s16, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, true,
          1), PIE_NOMEM))
PIE_TRANS(esp_vsub_s16_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, true,
          1), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_s16_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, true,
          1), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_s32, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, true,
          1), PIE_NOMEM))
PIE_TRANS(esp_vsub_s32_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, true,
          1), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_s32_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, true,
          1), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_s8, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, true,
          1), PIE_NOMEM))
PIE_TRANS(esp_vsub_s8_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, true,
          1), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_s8_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, true,
          1), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_u16, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, false,
          1), PIE_NOMEM))
PIE_TRANS(esp_vsub_u16_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, false,
          1), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_u16_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(16, false,
          1), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_u32, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, false,
          1), PIE_NOMEM))
PIE_TRANS(esp_vsub_u32_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, false,
          1), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_u32_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(32, false,
          1), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_u8, pie_qv_qx_qy_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, false,
          1), PIE_NOMEM))
PIE_TRANS(esp_vsub_u8_ld_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, false,
          1), PIE_MEM(ESP32P4_PIE_MEM_LD128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vsub_u8_st_incp, pie_qu_qv_qx_qy_rs1_sat,
          gen_pie_vaddsub(ctx, a->qv, a->qx, a->qy, a->sat, pie_op(8, false,
          1), PIE_MEM(ESP32P4_PIE_MEM_ST128, a->qu, a->rs1, 16)))
PIE_TRANS(esp_vunzip_16, pie_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, 0, 2, pie_op(16, false,
          PIE_PERM_UNZIP)))
PIE_TRANS(esp_vunzip_32, pie_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, 0, 2, pie_op(32, false,
          PIE_PERM_UNZIP)))
PIE_TRANS(esp_vunzip_8, pie_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, 0, 2, pie_op(8, false,
          PIE_PERM_UNZIP)))
PIE_TRANS(esp_vunzipt_16, pie_qw_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, a->qw, 3, pie_op(16, false,
          PIE_PERM_UNZIPT)))
PIE_TRANS(esp_vunzipt_8, pie_qw_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, a->qw, 3, pie_op(8, false,
          PIE_PERM_UNZIPT)))
PIE_TRANS(esp_vzip_16, pie_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, 0, 2, pie_op(16, false,
          PIE_PERM_ZIP)))
PIE_TRANS(esp_vzip_32, pie_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, 0, 2, pie_op(32, false,
          PIE_PERM_ZIP)))
PIE_TRANS(esp_vzip_8, pie_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, 0, 2, pie_op(8, false,
          PIE_PERM_ZIP)))
PIE_TRANS(esp_vzipt_16, pie_qw_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, a->qw, 3, pie_op(16, false,
          PIE_PERM_ZIPT)))
PIE_TRANS(esp_vzipt_8, pie_qw_qx_qy,
          gen_pie_perm(ctx, a->qx, a->qy, a->qw, 3, pie_op(8, false,
          PIE_PERM_ZIPT)))
PIE_TRANS(esp_xorq, pie_qx_qy_qz,
          gen_pie_logic(ctx, a->qz, a->qx, a->qy, pie_op(8, false, 2)))
PIE_TRANS(esp_zero_q, pie_qz,
          gen_pie_zero(ctx, a->qz, pie_op(8, false, 0)))
PIE_TRANS(esp_zero_qacc, esp_zero_qacc,
          gen_pie_zero(ctx, 0, pie_op(8, false, 1)))
PIE_TRANS(esp_zero_xacc, esp_zero_xacc,
          gen_pie_zero(ctx, 0, pie_op(8, false, 2)))
