/*
 * Espressif ESP32-P4 HP core: hardware loop instructions and MRET
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A hardware loop goes back to its start after its last instruction, at
 * its end address, completes without branching, while its count is not
 * zero, taking one from the count each time. Loop 0 is checked first, so
 * an inner loop 0 sharing its last instruction with an outer loop 1 runs
 * out before loop 1 counts. The checks run after the instruction, so an
 * exception it raises leaves the count alone, and an interrupt taken next
 * sees the start in mepc.
 *
 * Loop ends are not part of the TB key; instead, while any loop counts
 * (ESP32P4_HWLP_ARMED in the TB flags), every instruction compares its
 * address with both end addresses. An instruction that ends its TB and
 * continues with the next one checks the loops in the helper itself.
 * The helper checks the end, count and state again, so stale checks do
 * nothing.
 */

#ifndef CONFIG_USER_ONLY

/*
 * Add @n to the instructions an ESP32-P4 core has retired (minstret,
 * esp32p4_counters.c): 1 as each instruction starts, -1 as one raises its
 * exception.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_count_insn(DisasContext *ctx, int n)
{
    tcg_gen_addi_i64(esp32p4_insns, esp32p4_insns, n);
}

/*
 * The events an ESP32-P4 core's event counters count (ESP32P4_EV_*,
 * esp32p4_counters.c): each instruction reports its events once it can
 * no longer raise an exception, so that one that raises its exception
 * reports none, as it does not retire. The HP core counts conditional
 * branches, their mispredictions and store instructions; the LP core
 * memory reads and writes, unconditional jumps, conditional branches and
 * those taken, and compressed instructions.
 */

/* Add @n to the core's count of event @e. */
static void gen_esp32p4_event_add(int e, TCGv_i64 n)
{
    TCGv_i64 t = tcg_temp_new_i64();
    intptr_t ofs = ESP32P4_ENV_OFFSET(ev) + e * sizeof(uint64_t);

    tcg_gen_ld_i64(t, tcg_env, ofs);
    tcg_gen_add_i64(t, t, n);
    tcg_gen_st_i64(t, tcg_env, ofs);
}

static void gen_esp32p4_event(int e)
{
    gen_esp32p4_event_add(e, tcg_constant_i64(1));
}

/*
 * A memory read or write of the LP core at @addr: one access, or two for
 * an unaligned one (TRM 5.7).
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_mem_event(DisasContext *ctx, bool write, TCGv addr,
                                  MemOp memop)
{
    int e = write ? ESP32P4_EV_MEM_WRITE : ESP32P4_EV_MEM_READ;
    unsigned size = memop_size(memop);
    TCGv t;
    TCGv_i64 n;

    if (!ctx->esp32p4_lp) {
        return;
    }
    if (size == 1) {
        gen_esp32p4_event(e);
        return;
    }
    t = tcg_temp_new();
    n = tcg_temp_new_i64();
    tcg_gen_andi_tl(t, addr, size - 1);
    tcg_gen_setcondi_tl(TCG_COND_NE, t, t, 0);
    tcg_gen_addi_tl(t, t, 1);
    tcg_gen_extu_tl_i64(n, t);
    gen_esp32p4_event_add(e, n);
}

/* An atomic memory operation of the LP core: a read and a write. */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_amo_event(DisasContext *ctx)
{
    if (ctx->esp32p4_lp) {
        gen_esp32p4_event(ESP32P4_EV_MEM_READ);
        gen_esp32p4_event(ESP32P4_EV_MEM_WRITE);
    }
}

/* SC.W on the LP core: a write, unless it failed and stored nothing. */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_sc_event(DisasContext *ctx, TCGv failed)
{
    TCGv t;
    TCGv_i64 n;

    if (!ctx->esp32p4_lp) {
        return;
    }
    t = tcg_temp_new();
    n = tcg_temp_new_i64();
    tcg_gen_xori_tl(t, failed, 1);
    tcg_gen_extu_tl_i64(n, t);
    gen_esp32p4_event_add(ESP32P4_EV_MEM_WRITE, n);
}

/* A store instruction of the HP core (mhpmevent13's 0xB). */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static void gen_esp32p4_store_insn(DisasContext *ctx)
{
    if (ctx->esp32p4_hp) {
        gen_esp32p4_event(ESP32P4_EV_STORE);
    }
}

/* A compressed instruction of the LP core retires. */
static void gen_esp32p4_compressed(DisasContext *ctx)
{
    if (ctx->esp32p4_lp && ctx->cur_insn_len == 2) {
        gen_esp32p4_event(ESP32P4_EV_COMPRESSED);
    }
}

/*
 * An unconditional jump (JAL, JALR and their compressed forms) of the LP
 * core, which ends its TB.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_jump_event(DisasContext *ctx)
{
    if (ctx->esp32p4_lp) {
        gen_esp32p4_event(ESP32P4_EV_JUMP);
        gen_esp32p4_compressed(ctx);
    }
}

/* A conditional branch, which ends its TB. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_branch_event(DisasContext *ctx)
{
    if (ctx->esp32p4_hp || ctx->esp32p4_lp) {
        gen_esp32p4_event(ESP32P4_EV_BRANCH);
        gen_esp32p4_compressed(ctx);
    }
}

/*
 * The way the conditional branch goes: count it if taken, and run the
 * HP core's branch predictor with it while it counts mispredictions.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_branch_outcome(DisasContext *ctx, bool taken)
{
    if (ctx->esp32p4_lp && taken) {
        gen_esp32p4_event(ESP32P4_EV_TAKEN);
    }
    if (ctx->esp32p4_bpred) {
        gen_helper_esp32p4_branch(tcg_env, tcg_constant_i32(taken));
    }
}

/*
 * The instruction has completed without ending its TB: a compressed one
 * of the LP core retires.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void gen_esp32p4_insn_done(DisasContext *ctx)
{
    gen_esp32p4_compressed(ctx);
}

/*
 * Continue after the current instruction at next, the next instruction
 * when next is NULL, or a loop's start if the instruction ends one, and
 * end the TB.
 */
static void gen_esp32p4_hwlp_end(DisasContext *ctx, TCGv next, bool main_loop)
{
    TCGv pc = tcg_temp_new();
    TCGv seq = tcg_temp_new();
    TCGv dest = tcg_temp_new();

    gen_pc_plus_diff(pc, ctx, 0);
    gen_pc_plus_diff(seq, ctx, ctx->cur_insn_len);
    gen_helper_esp32p4_hwlp_end(dest, tcg_env, pc, seq, next ? next : seq);
    tcg_gen_mov_tl(cpu_pc, dest);
    if (main_loop) {
        exit_tb(ctx);
    } else {
        lookup_and_goto_ptr(ctx);
    }
    ctx->base.is_jmp = DISAS_NORETURN;
}

/*
 * After an instruction that falls through to the next one, go to a loop's
 * start instead if the instruction ends a counting loop.
 */
static void gen_esp32p4_hwlp_check(DisasContext *ctx)
{
    TCGLabel *cont = gen_new_label();
    TCGLabel *at_end = gen_new_label();
    TCGv pc = tcg_temp_new();
    TCGv end = tcg_temp_new();
    TCGv seq = tcg_temp_new();
    TCGv dest = tcg_temp_new();

    gen_pc_plus_diff(pc, ctx, 0);
    tcg_gen_ld32s_tl(end, tcg_env, ESP32P4_ENV_OFFSET(hwlp_end[0]));
    tcg_gen_brcond_tl(TCG_COND_EQ, end, pc, at_end);
    tcg_gen_ld32s_tl(end, tcg_env, ESP32P4_ENV_OFFSET(hwlp_end[1]));
    tcg_gen_brcond_tl(TCG_COND_NE, end, pc, cont);

    gen_set_label(at_end);
    gen_pc_plus_diff(seq, ctx, ctx->cur_insn_len);
    gen_helper_esp32p4_hwlp_end(dest, tcg_env, pc, seq, seq);
    tcg_gen_brcond_tl(TCG_COND_EQ, dest, seq, cont);
    tcg_gen_mov_tl(cpu_pc, dest);
    lookup_and_goto_ptr(ctx);

    gen_set_label(cont);
}

/*
 * The debug assistant's SP monitor (hw/misc/esp32p4_assist_debug.c), after
 * an instruction writes SP while it is enabled: compare SP with the
 * monitored bounds, and call the helper only when SP has crossed one since
 * the last report, so that code keeping its stack in bounds pays two
 * compares per SP write. The helper takes the instruction's address, for
 * the PC the monitor records. Its report may raise this core's interrupt,
 * which is taken after the instruction: the instruction ends its block
 * (as under icount it must, to raise it).
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void gen_esp32p4_sp_check(DisasContext *ctx)
{
    TCGLabel *done = gen_new_label();
    TCGv sp = tcg_temp_new();
    TCGv bound = tcg_temp_new();
    TCGv out = tcg_temp_new();
    TCGv above = tcg_temp_new();

    translator_io_start(&ctx->base);
    tcg_gen_ext32u_tl(sp, cpu_gpr[xSP]);
    tcg_gen_ld32u_tl(bound, tcg_env, ESP32P4_ENV_OFFSET(spmon_min));
    tcg_gen_setcond_tl(TCG_COND_LTU, out, sp, bound);
    tcg_gen_ld32u_tl(bound, tcg_env, ESP32P4_ENV_OFFSET(spmon_max));
    tcg_gen_setcond_tl(TCG_COND_GTU, above, sp, bound);
    tcg_gen_or_tl(out, out, above);
    tcg_gen_ld32u_tl(bound, tcg_env, ESP32P4_ENV_OFFSET(spmon_out));
    tcg_gen_brcond_tl(TCG_COND_EQ, out, bound, done);
    gen_pc_plus_diff(sp, ctx, 0);
    gen_helper_esp32p4_sp_check(tcg_env, sp);
    gen_set_label(done);
}

/*
 * The kind of the instruction at pc_next, for the trace encoder: a
 * conditional branch, a jump whose target the program gives (JAL, C.J,
 * C.JAL), one whose target it does not (JALR, C.JR, C.JALR, Zcmp's
 * cm.popret and cm.popretz, Zcmt's table jumps), a trap return, or none
 * of these; and its length.
 */
static uint32_t esp32p4_trace_kind(DisasContext *ctx, CPURISCVState *env)
{
    uint16_t lo = translator_lduw_end(env, &ctx->base, ctx->base.pc_next,
                                      MO_LE);
    uint32_t insn;

    if ((lo & 3) != 3) {
        unsigned funct3 = lo >> 13;

        switch (lo & 3) {
        case 1:
            if (funct3 == 1 || funct3 == 5) {
                return ESP32P4_TRACE_KIND_JAL;
            }
            if (funct3 == 6 || funct3 == 7) {
                return ESP32P4_TRACE_KIND_BRANCH;
            }
            break;
        case 2:
            if (funct3 == 4 && !extract32(lo, 2, 5) && extract32(lo, 7, 5)) {
                return ESP32P4_TRACE_KIND_JALR;
            }
            if ((lo & 0xfc03) == 0xa002 || (lo & 0xff03) == 0xbe02 ||
                (lo & 0xff03) == 0xbc02) {
                return ESP32P4_TRACE_KIND_JALR;
            }
            break;
        default:
            break;
        }
        return ESP32P4_TRACE_KIND_PLAIN;
    }
    insn = lo | ((uint32_t)translator_lduw_end(env, &ctx->base,
                                               ctx->base.pc_next + 2,
                                               MO_LE) << 16);
    switch (insn & 0x7f) {
    case 0x63:
        return ESP32P4_TRACE_KIND_BRANCH | ESP32P4_TRACE_LEN_4;
    case 0x6f:
        return ESP32P4_TRACE_KIND_JAL | ESP32P4_TRACE_LEN_4;
    case 0x67:
        return ESP32P4_TRACE_KIND_JALR | ESP32P4_TRACE_LEN_4;
    case 0x73:
        /* MRET, SRET, URET and DRET. */
        if (insn == 0x30200073 || insn == 0x10200073 ||
            insn == 0x00200073 || insn == 0x7b200073) {
            return ESP32P4_TRACE_KIND_XRET | ESP32P4_TRACE_LEN_4;
        }
        break;
    default:
        break;
    }
    return ESP32P4_TRACE_KIND_PLAIN | ESP32P4_TRACE_LEN_4;
}

/*
 * While the trace encoder traces the core, each instruction reports its
 * start, and with it the retirement of the one before; the encoder learns
 * from the address each starts at whether a branch was taken and where a
 * jump went.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void gen_esp32p4_trace_insn(DisasContext *ctx, CPURISCVState *env)
{
    TCGv pc = tcg_temp_new();

    gen_pc_plus_diff(pc, ctx, 0);
    gen_helper_esp32p4_trace_insn(tcg_env, pc,
                                  tcg_constant_i32(esp32p4_trace_kind(ctx,
                                                                      env)));
}

/*
 * Set the given registers of a loop, ending the TB since the loop may
 * start counting. A disabled extension, or user mode without
 * mhwloop_state_reg.URW, makes the instruction illegal.
 */
static bool gen_esp32p4_hwlp_set(DisasContext *ctx, int loop, int set,
                                 TCGv start, TCGv end, TCGv count, TCGv next)
{
    if (!ctx->hwlp_en) {
        return false;
    }
    gen_helper_esp32p4_hwlp_set(tcg_env, tcg_constant_i32(loop | set),
                                start ? start : ctx->zero,
                                end ? end : ctx->zero,
                                count ? count : ctx->zero);
    gen_esp32p4_hwlp_end(ctx, next, false);
    return true;
}

static TCGv esp32p4_pc_plus(DisasContext *ctx, target_long diff)
{
    TCGv t = tcg_temp_new();

    gen_pc_plus_diff(t, ctx, diff);
    return t;
}

static bool trans_esp_lp_starti(DisasContext *ctx, arg_lp_imm *a)
{
    return gen_esp32p4_hwlp_set(ctx, a->loop, ESP32P4_HWLP_SET_START,
                                esp32p4_pc_plus(ctx, a->imm), NULL, NULL,
                                NULL);
}

static bool trans_esp_lp_endi(DisasContext *ctx, arg_lp_imm *a)
{
    return gen_esp32p4_hwlp_set(ctx, a->loop, ESP32P4_HWLP_SET_END, NULL,
                                esp32p4_pc_plus(ctx, a->imm), NULL, NULL);
}

static bool trans_esp_lp_count(DisasContext *ctx, arg_lp_reg *a)
{
    return gen_esp32p4_hwlp_set(ctx, a->loop, ESP32P4_HWLP_SET_COUNT, NULL,
                                NULL, get_gpr(ctx, a->rs1, EXT_NONE), NULL);
}

static bool trans_esp_lp_counti(DisasContext *ctx, arg_lp_imm *a)
{
    return gen_esp32p4_hwlp_set(ctx, a->loop, ESP32P4_HWLP_SET_COUNT, NULL,
                                NULL, tcg_constant_tl(a->imm), NULL);
}

static bool trans_esp_lp_setupi(DisasContext *ctx, arg_lp_setupi *a)
{
    return gen_esp32p4_hwlp_set(ctx, a->loop,
                                ESP32P4_HWLP_SET_START | ESP32P4_HWLP_SET_END |
                                ESP32P4_HWLP_SET_COUNT,
                                esp32p4_pc_plus(ctx, ctx->cur_insn_len),
                                esp32p4_pc_plus(ctx, a->off),
                                tcg_constant_tl(a->count), NULL);
}

/*
 * lp.setup, and lp.setup.beqz which is the same instruction: a zero count
 * skips the loop, to the instruction after its last.
 */
static bool trans_esp_lp_setup(DisasContext *ctx, arg_lp_setup *a)
{
    TCGv count = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv next = tcg_temp_new();

    tcg_gen_movcond_tl(TCG_COND_EQ, next, count, ctx->zero,
                       esp32p4_pc_plus(ctx, a->off + (a->size32 ? 4 : 2)),
                       esp32p4_pc_plus(ctx, ctx->cur_insn_len));
    return gen_esp32p4_hwlp_set(ctx, a->loop,
                                ESP32P4_HWLP_SET_START | ESP32P4_HWLP_SET_END |
                                ESP32P4_HWLP_SET_COUNT,
                                esp32p4_pc_plus(ctx, ctx->cur_insn_len),
                                esp32p4_pc_plus(ctx, a->off), count, next);
}

/*
 * MRET, followed by the core's own trap-return effects on the trigger
 * enable, the bus-error state and the CLIC, given mstatus from before the
 * return.
 */
static bool trans_esp_mret(DisasContext *ctx, arg_esp_mret *a)
{
    TCGv_i64 mstatus = tcg_temp_new_i64();

    decode_save_opc(ctx, 0);
    translator_io_start(&ctx->base);
    gen_update_pc(ctx, 0);
    tcg_gen_ld_i64(mstatus, tcg_env, offsetof(CPURISCVState, mstatus));
    gen_helper_mret(cpu_pc, tcg_env);
    gen_helper_esp32p4_mret(cpu_pc, tcg_env, mstatus, cpu_pc);
    exit_tb(ctx);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

/* URET, which the core has for its user-mode interrupt handlers. */
static bool trans_esp_uret(DisasContext *ctx, arg_esp_uret *a)
{
    decode_save_opc(ctx, 0);
    translator_io_start(&ctx->base);
    gen_update_pc(ctx, 0);
    gen_helper_esp32p4_uret(cpu_pc, tcg_env);
    exit_tb(ctx);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

#else

static bool trans_esp_lp_starti(DisasContext *ctx, arg_lp_imm *a)
{
    return false;
}

static bool trans_esp_lp_endi(DisasContext *ctx, arg_lp_imm *a)
{
    return false;
}

static bool trans_esp_lp_count(DisasContext *ctx, arg_lp_reg *a)
{
    return false;
}

static bool trans_esp_lp_counti(DisasContext *ctx, arg_lp_imm *a)
{
    return false;
}

static bool trans_esp_lp_setupi(DisasContext *ctx, arg_lp_setupi *a)
{
    return false;
}

static bool trans_esp_lp_setup(DisasContext *ctx, arg_lp_setup *a)
{
    return false;
}

static bool trans_esp_mret(DisasContext *ctx, arg_esp_mret *a)
{
    return false;
}

static bool trans_esp_uret(DisasContext *ctx, arg_esp_uret *a)
{
    return false;
}

#endif

#include "trans_esp32p4_pie.c.inc"
