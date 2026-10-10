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
