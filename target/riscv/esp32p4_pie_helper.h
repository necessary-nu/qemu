/*
 * Espressif ESP32-P4 HP core: Processor Instruction Extensions (PIE)
 * helpers, which helper.h includes.
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The helpers that access memory may fault, and so must see the TCG
 * globals in memory; the others neither read nor write them.
 */

DEF_HELPER_FLAGS_4(esp32p4_pie_mac, TCG_CALL_NO_WG, void, env, i32, i32, tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_vaddsub, TCG_CALL_NO_WG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_vsaddsub, TCG_CALL_NO_RWG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_vmul, TCG_CALL_NO_WG, void, env, i32, i32, tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_cmul, TCG_CALL_NO_WG, void, env, i32, i32, tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_vminmax, TCG_CALL_NO_WG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_vcmp, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_3(esp32p4_pie_minmax_a, TCG_CALL_NO_RWG, tl, env, i32, i32)
DEF_HELPER_FLAGS_3(esp32p4_pie_vabs, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_2(esp32p4_pie_vclamp, TCG_CALL_NO_RWG, void, env, i32)
DEF_HELPER_FLAGS_5(esp32p4_pie_vsat, TCG_CALL_NO_RWG, void, env, i32, i32,
                   tl, tl)
DEF_HELPER_FLAGS_5(esp32p4_pie_vrelu, TCG_CALL_NO_RWG, void, env, i32, i32,
                   tl, tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_vprelu, TCG_CALL_NO_RWG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_perm, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_2(esp32p4_pie_movx_r, TCG_CALL_NO_RWG, tl, env, i32)
DEF_HELPER_FLAGS_3(esp32p4_pie_movx_w, TCG_CALL_NO_RWG, void, env, i32, tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_movi_a, TCG_CALL_NO_RWG, tl, env, i32, i32)
DEF_HELPER_FLAGS_4(esp32p4_pie_movi_q, TCG_CALL_NO_RWG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_zero, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_3(esp32p4_pie_mov_qacc, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_3(esp32p4_pie_logic, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_4(esp32p4_pie_load, TCG_CALL_NO_WG, void, env, i32, i32, tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_store, TCG_CALL_NO_WG, void, env, i32, i32, tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_xq, TCG_CALL_NO_WG, void, env, i32, i32, tl)
DEF_HELPER_FLAGS_5(esp32p4_pie_src, TCG_CALL_NO_WG, void, env, i32, i32, tl,
                   tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_srcmb, TCG_CALL_NO_RWG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_srs, TCG_CALL_NO_RWG, tl, env, i32, i32, tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_vshift, TCG_CALL_NO_RWG, void, env, i32, i32)
DEF_HELPER_FLAGS_4(esp32p4_pie_fft_r2bf, TCG_CALL_NO_WG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_4(esp32p4_pie_fft_cmul, TCG_CALL_NO_WG, void, env, i32, i32,
                   tl)
DEF_HELPER_FLAGS_5(esp32p4_pie_fft_ams, TCG_CALL_NO_WG, tl, env, i32, i32, tl,
                   tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_fft_bitrev, TCG_CALL_NO_RWG, void, env, i32,
                   tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_fft_vst_r32, TCG_CALL_NO_WG, void, env, i32,
                   tl)
DEF_HELPER_FLAGS_3(esp32p4_pie_sat, TCG_CALL_NO_RWG_SE, tl, tl, tl, tl)
