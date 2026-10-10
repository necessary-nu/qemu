/*
 * Espressif ESP32-P4 HP core trigger module
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.11.3
 * (Hardware Trigger).
 *
 * Three triggers, all of the address match type (mcontrol, type 2). Each
 * matches the address of an instruction about to execute, or of a load
 * or store, either exactly or anywhere in a naturally aligned power-of-two
 * range of up to 256 bytes (maskmax is 8); an exact data match needs one
 * byte of the access at the address. A trigger fires in the modes its M
 * and U bits select and, with tdata3.mselect set, only while the low bits
 * of mcontext equal tdata3.mvalue. Firing sets its hit bit.
 *
 * Action 0 raises a breakpoint exception, before the instruction, or for
 * a load or store with timing set, after it. These triggers fire only
 * while tcontrol.MTE is set; a trap saves MTE in MPTE and clears it, and
 * MRET restores it. The debugger alone may set dmode and the debug-mode
 * action 1, which M-mode writes turn into action 0; the trace actions 2
 * to 4 are kept, but no trace is modelled.
 *
 * The access size filter, sizelo, is kept but not applied: QEMU's
 * watchpoint check does not report the size of the access.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/breakpoint.h"
#include "exec/watchpoint.h"
#include "target/riscv/esp32p4_cpu.h"

#define MCONTROL_TYPE2          (2u << 28)
#define MCONTROL_DMODE          BIT(27)
#define MCONTROL_MASKMAX        (8u << 21)
#define MCONTROL_HIT            BIT(20)
#define MCONTROL_TIMING         BIT(18)
#define MCONTROL_SIZELO         (3u << 16)
#define MCONTROL_ACTION_SHIFT   12
#define MCONTROL_MATCH_SHIFT    7
#define MCONTROL_M              BIT(6)
#define MCONTROL_U              BIT(3)
#define MCONTROL_EXECUTE        BIT(2)
#define MCONTROL_STORE          BIT(1)
#define MCONTROL_LOAD           BIT(0)

#define MCONTROL_RESET          (MCONTROL_TYPE2 | MCONTROL_MASKMAX)
#define MCONTROL_WMASK          (MCONTROL_HIT | MCONTROL_TIMING |       \
                                 MCONTROL_SIZELO | MCONTROL_M |          \
                                 MCONTROL_U | MCONTROL_EXECUTE |         \
                                 MCONTROL_STORE | MCONTROL_LOAD)

#define TDATA3_MVALUE_SHIFT     26
#define TDATA3_MSELECT          BIT(25)
#define TDATA3_WMASK            ((0x3fu << TDATA3_MVALUE_SHIFT) | \
                                 TDATA3_MSELECT)

/* The largest NAPOT range, 2^maskmax bytes. */
#define ESP32P4_TRIGGER_NAPOT_MAX_SHIFT 8

static unsigned trigger_action(uint32_t ctrl)
{
    return extract32(ctrl, MCONTROL_ACTION_SHIFT, 4);
}

/* The bytes a trigger's address covers. */
static void trigger_range(ESP32P4CPU *cpu, int t, uint32_t *base,
                          uint32_t *len)
{
    uint32_t addr = cpu->tdata2[t];

    if (extract32(cpu->tdata1[t], MCONTROL_MATCH_SHIFT, 4) == 0) {
        *base = addr;
        *len = 1;
    } else {
        int shift = MIN(cto32(addr) + 1, ESP32P4_TRIGGER_NAPOT_MAX_SHIFT);

        *len = 1u << shift;
        *base = addr & ~(*len - 1);
    }
}

static bool trigger_covers(ESP32P4CPU *cpu, int t, vaddr addr)
{
    uint32_t base, len;

    trigger_range(cpu, t, &base, &len);
    return (uint32_t)addr - base < len;
}

/*
 * The trigger is enabled in the current privilege mode and its context
 * condition holds.
 */
static bool trigger_live(ESP32P4CPU *cpu, int t)
{
    uint32_t ctrl = cpu->tdata1[t];
    uint32_t tdata3 = cpu->tdata3[t];
    privilege_mode_t priv = cpu->parent_obj.env.priv;

    if (!(priv == PRV_M ? ctrl & MCONTROL_M : ctrl & MCONTROL_U)) {
        return false;
    }
    return !(tdata3 & TDATA3_MSELECT) ||
           extract32(tdata3, TDATA3_MVALUE_SHIFT, 6) == cpu->mcontext;
}

/*
 * A matching trigger fires: one with action 0 is taken as a breakpoint
 * exception while tcontrol.MTE allows; one with a trace action only sets
 * its hit bit.
 */
static bool trigger_fire(ESP32P4CPU *cpu, int t)
{
    if (trigger_action(cpu->tdata1[t]) != 0) {
        cpu->tdata1[t] |= MCONTROL_HIT;
        qemu_log_mask(LOG_UNIMP, "esp32p4: trigger %d fired with trace "
                      "action %u, which is not modelled\n", t,
                      trigger_action(cpu->tdata1[t]));
        return false;
    }
    if (!(cpu->tcontrol & ESP32P4_TCONTROL_MTE)) {
        return false;
    }
    cpu->triggers_fired |= BIT(t);
    return true;
}

/*
 * Install QEMU breakpoints and watchpoints for the enabled triggers: a
 * breakpoint at every halfword an execute trigger covers, and a
 * watchpoint over the bytes a load or store trigger covers, taken before
 * the access unless the trigger's timing is set.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_triggers_rebuild(ESP32P4CPU *cpu)
{
    CPUState *cs = CPU(cpu);

    cpu_breakpoint_remove_all(cs, BP_CPU);
    cpu_watchpoint_remove_all(cs, BP_CPU);
    for (int t = 0; t < ESP32P4_TRIGGERS; t++) {
        uint32_t ctrl = cpu->tdata1[t];
        uint32_t base, len;
        int flags = BP_CPU;

        if (!(ctrl & (MCONTROL_M | MCONTROL_U))) {
            continue;
        }
        trigger_range(cpu, t, &base, &len);
        if (ctrl & MCONTROL_EXECUTE) {
            for (uint32_t off = 0; off < len; off += 2) {
                cpu_breakpoint_insert(cs, base + off, BP_CPU, NULL);
            }
        }
        if (ctrl & MCONTROL_LOAD) {
            flags |= BP_MEM_READ;
        }
        if (ctrl & MCONTROL_STORE) {
            flags |= BP_MEM_WRITE;
        }
        if (!(ctrl & MCONTROL_TIMING)) {
            flags |= BP_STOP_BEFORE_ACCESS;
        }
        if (flags & BP_MEM_ACCESS) {
            cpu_watchpoint_insert(cs, base, len, flags, NULL);
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_triggers_reset(ESP32P4CPU *cpu)
{
    cpu->tselect = 0;
    for (int t = 0; t < ESP32P4_TRIGGERS; t++) {
        cpu->tdata1[t] = MCONTROL_RESET;
        cpu->tdata2[t] = 0;
        cpu->tdata3[t] = 0;
    }
    cpu->tcontrol = 0;
    cpu->mcontext = 0;
    cpu->triggers_fired = 0;
    esp32p4_triggers_rebuild(cpu);
}

/* An index past the last trigger selects the last. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_tselect_write(ESP32P4CPU *cpu, target_ulong val)
{
    cpu->tselect = MIN(val, ESP32P4_TRIGGERS - 1);
}

target_ulong esp32p4_tdata_read(ESP32P4CPU *cpu, int index)
{
    switch (index) {
    case 0:
        return cpu->tdata1[cpu->tselect];
    case 1:
        return cpu->tdata2[cpu->tselect];
    default:
        return cpu->tdata3[cpu->tselect];
    }
}

/*
 * The type, dmode and maskmax are read-only; an action other than 0 and
 * the trace actions becomes 0, and a match mode above NAPOT becomes NAPOT.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_tdata_write(ESP32P4CPU *cpu, int index, target_ulong val)
{
    int t = cpu->tselect;
    unsigned action, match;

    if (cpu->tdata1[t] & MCONTROL_DMODE) {
        return;
    }
    switch (index) {
    case 0:
        action = extract32(val, MCONTROL_ACTION_SHIFT, 4);
        if (action != 0 && (action < 2 || action > 4)) {
            action = 0;
        }
        match = MIN(extract32(val, MCONTROL_MATCH_SHIFT, 4), 1);
        if (val & MCONTROL_SIZELO) {
            qemu_log_mask(LOG_UNIMP, "esp32p4: trigger %d matches accesses "
                          "of any size, not only those sizelo selects\n", t);
        }
        cpu->tdata1[t] = MCONTROL_RESET | (val & MCONTROL_WMASK) |
                         (action << MCONTROL_ACTION_SHIFT) |
                         (match << MCONTROL_MATCH_SHIFT);
        break;
    case 1:
        cpu->tdata2[t] = val;
        break;
    default:
        cpu->tdata3[t] = val & TDATA3_WMASK;
        return;
    }
    esp32p4_triggers_rebuild(cpu);
}

/* An execute trigger matches the instruction at pc. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
bool esp32p4_debug_check_breakpoint(CPUState *cs)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);
    CPURISCVState *env = &cpu->parent_obj.env;
    bool taken = false;

    cpu->triggers_fired = 0;
    for (int t = 0; t < ESP32P4_TRIGGERS; t++) {
        if ((cpu->tdata1[t] & MCONTROL_EXECUTE) && trigger_live(cpu, t) &&
            trigger_covers(cpu, t, env->pc)) {
            taken |= trigger_fire(cpu, t);
        }
    }
    if (taken) {
        env->badaddr = env->pc;
    }
    return taken;
}

/* A load or store trigger matches the access the watchpoint caught. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
bool esp32p4_debug_check_watchpoint(CPUState *cs, CPUWatchpoint *wp)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);
    uint32_t kind = (wp->flags & BP_WATCHPOINT_HIT_READ) ? MCONTROL_LOAD
                                                         : MCONTROL_STORE;
    uint32_t timing = (wp->flags & BP_STOP_BEFORE_ACCESS) ? 0
                                                          : MCONTROL_TIMING;
    bool taken = false;

    cpu->triggers_fired = 0;
    for (int t = 0; t < ESP32P4_TRIGGERS; t++) {
        uint32_t ctrl = cpu->tdata1[t];

        if ((ctrl & kind) && (ctrl & MCONTROL_TIMING) == timing &&
            trigger_live(cpu, t) && trigger_covers(cpu, t, wp->hitaddr)) {
            taken |= trigger_fire(cpu, t);
        }
    }
    return taken;
}

/*
 * Take the breakpoint exception for the triggers that fired, setting
 * their hit bits.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_debug_excp_handler(CPUState *cs)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);
    CPURISCVState *env = &cpu->parent_obj.env;

    if (cs->watchpoint_hit) {
        if (!(cs->watchpoint_hit->flags & BP_CPU)) {
            return;
        }
    } else if (!cpu_breakpoint_test(cs, env->pc, BP_CPU)) {
        return;
    }
    for (int t = 0; t < ESP32P4_TRIGGERS; t++) {
        if (cpu->triggers_fired & BIT(t)) {
            cpu->tdata1[t] |= MCONTROL_HIT;
        }
    }
    cpu->triggers_fired = 0;
    riscv_raise_exception(env, RISCV_EXCP_BREAKPOINT, 0);
}
