/*
 * RISC-V Zcmt Extension Helper for QEMU.
 *
 * Copyright (c) 2021-2022 PLCT Lab
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "internals.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/probe.h"
#include "exec/target_page.h"

/*
 * An exception the instruction raises, here or fetching its table entry,
 * unwinds it: it does not retire, and icount does not charge it.
 */
target_ulong HELPER(cm_jalt)(CPURISCVState *env, uint32_t index)
{
    unsigned mmu_index = cpu_mmu_index(env_cpu(env), true);
    MemOp endian = mo_endian_env(env);
    uintptr_t ra = GETPC();
    MemOpIdx oi;

#if !defined(CONFIG_USER_ONLY)
    RISCVException ret = smstateen_acc_ok(env, 0, SMSTATEEN0_JVT);
    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, ra);
    }
#endif

    target_ulong target;
    target_ulong val = env->jvt;
    int xlen = riscv_cpu_xlen(env);
    uint8_t mode = get_field(val, JVT_MODE);
    target_ulong base = val & JVT_BASE;
    target_ulong t0;

    if (mode != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, ra);
    }

    if (xlen == 32) {
        oi = make_memop_idx(MO_UL | endian, mmu_index);
        t0 = base + (index << 2);
        target = cpu_ldl_code_mmu(env, t0, oi, ra);
    } else {
        oi = make_memop_idx(MO_UQ | endian, mmu_index);
        t0 = base + (index << 3);
        target = cpu_ldq_code_mmu(env, t0, oi, ra);
    }

    return target & ~0x1;
}

/*
 * Check the count accesses of size bytes a cm.push or cm.pop makes, from
 * addr downwards in the order it makes them, raising the fault the first
 * refused one raises.
 */
void HELPER(cm_check)(CPURISCVState *env, target_ulong addr, uint32_t count,
                      uint32_t size, uint32_t access_type)
{
    int mmu_idx = riscv_env_mmu_index(env, false);
    uintptr_t ra = GETPC();
    void *host;

    for (uint32_t i = 0; i < count; i++, addr -= size) {
        target_ulong a = addr;
        uint32_t left = size;

        while (left) {
            uint32_t n = MIN(left, TARGET_PAGE_SIZE - (a & ~TARGET_PAGE_MASK));

            probe_access_flags(env, a, n, access_type, mmu_idx, false, &host,
                               ra);
            a += n;
            left -= n;
        }
    }
}
