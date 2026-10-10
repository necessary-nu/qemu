/*
 * Espressif ESP32-P4 HP core control and status registers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.5
 * (Configuration and Status Registers), 2.6.3 (F Extension), 2.7.1
 * (Hardware Loop) and 2.11.3 (Hardware Trigger).
 *
 * The CSRs the HP core has beyond, or differently from, the standard set
 * QEMU's RISC-V CPU provides for its configuration. QEMU's vendor CSR
 * table replaces entries of the CSR table every RISC-V CPU uses, so each
 * function here checks for an HP core: on any other core a CSR the HP core
 * adds does not exist, and one it changes behaves as it would without
 * this table.
 *
 * Absent on the HP core, and so illegal: mstatush, menvcfg, menvcfgh and
 * mconfigptr (the core follows privileged architecture 1.10), every
 * event selector but mhpmevent8, 9 and 13, and pmpcfg8-15 and
 * pmpaddr32-63 beyond its 32 PMP entries. The counters other than those
 * three are left out by the CPU's PMU mask.
 *
 * An access the core does not permit raises an illegal instruction
 * exception: QEMU's CSR access check covers the privilege the CSR number
 * encodes and writes to read-only numbers, and the functions here add the
 * rest, such as the hardware loop CSRs while the extension is off.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "fpu/softfloat-helpers.h"
#include "hw/core/irq.h"
#include "target/riscv/tcg/csr.h"
#include "target/riscv/esp32p4_cpu.h"

#define CSR_ESP_MHCR            0x7c1
#define CSR_ESP_MHINT           0x7c5
#define CSR_ESP_MHWLP_START0    0x7c6
#define CSR_ESP_MHWLP_END0      0x7c7
#define CSR_ESP_MHWLP_COUNT0    0x7c8
#define CSR_ESP_MHWLP_START1    0x7c9
#define CSR_ESP_MHWLP_END1      0x7ca
#define CSR_ESP_MHWLP_COUNT1    0x7cb
#define CSR_ESP_MEXSTATUS       0x7e1
#define CSR_ESP_MEXT_ILL        0x7f0
#define CSR_ESP_MHWLP_STATE     0x7f1
#define CSR_ESP_MEXT_PIE_STATUS 0x7f2
#define CSR_ESP_FXCR            0x800
#define CSR_ESP_GPIO_OEN        0x803
#define CSR_ESP_GPIO_IN         0x804
#define CSR_ESP_GPIO_OUT        0x805
#define CSR_ESP_UHWLP_START0    0x8c0
#define CSR_ESP_UHWLP_END0      0x8c1
#define CSR_ESP_UHWLP_COUNT0    0x8c2
#define CSR_ESP_UHWLP_START1    0x8c3
#define CSR_ESP_UHWLP_END1      0x8c4
#define CSR_ESP_UHWLP_COUNT1    0x8c5
#define CSR_ESP_UHWLP_STATE     0x8c6
#define CSR_ESP_LDPC0           0xbe0
#define CSR_ESP_LDPC1           0xbe1
#define CSR_ESP_LDTVAL0         0xbe8
#define CSR_ESP_LDTVAL1         0xbe9
#define CSR_ESP_STPC0           0xbf0
#define CSR_ESP_STPC1           0xbf1
#define CSR_ESP_STPC2           0xbf2
#define CSR_ESP_STTVAL0         0xbf8
#define CSR_ESP_STTVAL1         0xbf9
#define CSR_ESP_STTVAL2         0xbfa
#define CSR_ESP_TCONTROL        0x7a5
#define CSR_ESP_PMACFG0         0xbc0
#define CSR_ESP_PMAADDR0        0xbd0

/* mstatus fields the core implements, other than the read-only XS and SD. */
#define ESP32P4_MSTATUS_MASK (MSTATUS_MIE | MSTATUS_MPIE | MSTATUS_MPP | \
                              MSTATUS_FS | MSTATUS_MPRV | MSTATUS_TW)

/* mhcr: the return stack, branch prediction and branch target buffer. */
#define ESP32P4_MHCR_MASK (BIT(4) | BIT(5) | BIT(12))

/* mhint.SBE: load and store bus errors raise synchronous exceptions. */
#define ESP32P4_MHINT_MASK BIT(20)

#define ESP32P4_MEXSTATUS_WMASK                                     \
    (ESP32P4_MEXSTATUS_BUS_ERR | ESP32P4_MEXSTATUS_PBEXE |          \
     ESP32P4_MEXSTATUS_PPBEXE | ESP32P4_MEXSTATUS_PMP_ERR |         \
     ESP32P4_MEXSTATUS_FETCH_ERR | ESP32P4_MEXSTATUS_NMFT |         \
     ESP32P4_MEXSTATUS_CLIC_INHV)

/* fxcr */
#define ESP32P4_FXCR_FE      BIT(5)
#define ESP32P4_FXCR_DQNAN   BIT(23)
#define ESP32P4_FXCR_RM_SHIFT 24

/* mhpmevent8, 9 and 13: the event selector and the event each counts. */
#define ESP32P4_MHPMEVENT_MASK 0x1f
static const struct {
    int counter;
    uint32_t event;
    const char *what;
} esp32p4_hpm_events[] = {
    { 8, 0x6, "conditional branch mispredictions" },
    { 9, 0x7, "conditional branch instructions" },
    { 13, 0xb, "store instructions" },
};

/*
 * The operations the CSRs of this table had before it was installed, in
 * the table's order.
 */
static riscv_csr_operations *esp32p4_std_ops;

static const riscv_csr_operations *esp32p4_std(int csrno)
{
    for (int i = 0; esp32p4_hp_csr_list[i].csr_ops.name; i++) {
        if (esp32p4_hp_csr_list[i].csrno == csrno) {
            return &esp32p4_std_ops[i];
        }
    }
    g_assert_not_reached();
}

static RISCVException std_predicate(CPURISCVState *env, int csrno)
{
    const riscv_csr_operations *ops = esp32p4_std(csrno);

    if (!ops->predicate || env->priv_ver < ops->min_priv_ver) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    return ops->predicate(env, csrno);
}

static RISCVException std_read(CPURISCVState *env, int csrno,
                               target_ulong *val)
{
    const riscv_csr_operations *ops = esp32p4_std(csrno);

    if (!ops->read) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    return ops->read(env, csrno, val);
}

static RISCVException std_write(CPURISCVState *env, int csrno,
                                target_ulong val, uintptr_t ra)
{
    const riscv_csr_operations *ops = esp32p4_std(csrno);

    if (!ops->write) {
        return RISCV_EXCP_NONE;
    }
    return ops->write(env, csrno, val, ra);
}

/*
 * Save the standard operations of the CSRs the table below replaces,
 * before any HP core installs it.
 */
void esp32p4_hp_csr_class_init(void)
{
    int n = 0;

    if (esp32p4_std_ops) {
        return;
    }
    while (esp32p4_hp_csr_list[n].csr_ops.name) {
        n++;
    }
    esp32p4_std_ops = g_new(riscv_csr_operations, n);
    for (int i = 0; i < n; i++) {
        riscv_get_csr_ops(esp32p4_hp_csr_list[i].csrno, &esp32p4_std_ops[i]);
    }
}

/* A CSR the HP core adds: absent on other cores. */
static RISCVException hp(CPURISCVState *env, int csrno)
{
    return esp32p4_env_is_hp(env) ? RISCV_EXCP_NONE : RISCV_EXCP_ILLEGAL_INST;
}

/* A standard CSR the HP core does not implement. */
static RISCVException hp_absent(CPURISCVState *env, int csrno)
{
    if (esp32p4_env_is_hp(env)) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    return std_predicate(env, csrno);
}

/* A standard CSR whose behaviour the HP core changes. */
static RISCVException hp_std(CPURISCVState *env, int csrno)
{
    if (esp32p4_env_is_hp(env)) {
        return RISCV_EXCP_NONE;
    }
    return std_predicate(env, csrno);
}

/* mstatus */

/*
 * XS summarises the hardware loop and PIE extension states, the dirtier
 * of the two; SD is set when FS or XS is dirty. Only the HP core's fields
 * are writable, and MPP's upper bit follows its lower one, so that MPP is
 * either user or machine mode.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException read_mstatus(CPURISCVState *env, int csrno,
                                   target_ulong *val)
{
    ESP32P4CPU *cpu;
    uint32_t xs;
    RISCVException ret = std_read(env, csrno, val);

    if (ret != RISCV_EXCP_NONE || !esp32p4_env_is_hp(env)) {
        return ret;
    }
    cpu = esp32p4_cpu_from_env(env);
    xs = MAX(cpu->hwlp_state & ESP32P4_HWLP_STATE,
             cpu->pie_state & ESP32P4_HWLP_STATE);
    *val = set_field(*val, MSTATUS_XS, xs);
    if (xs == ESP32P4_EXT_DIRTY ||
        get_field(*val, MSTATUS_FS) == ESP32P4_EXT_DIRTY) {
        *val |= MSTATUS32_SD;
    }
    return RISCV_EXCP_NONE;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException write_mstatus(CPURISCVState *env, int csrno,
                                    target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        val = (env->mstatus & ~ESP32P4_MSTATUS_MASK) |
              (val & ESP32P4_MSTATUS_MASK);
        val = set_field(val, MSTATUS_MPP,
                        (val & BIT(11)) ? PRV_M : PRV_U);
    }
    return std_write(env, csrno, val, ra);
}

static RISCVException read_absent(CPURISCVState *env, int csrno,
                                  target_ulong *val)
{
    return std_read(env, csrno, val);
}

static RISCVException write_absent(CPURISCVState *env, int csrno,
                                   target_ulong val, uintptr_t ra)
{
    return std_write(env, csrno, val, ra);
}

/* Counters */

/*
 * mcountinhibit.TM is writable on the HP core, but the time the core reads
 * through the time CSR is not its own to stop.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException read_mcountinhibit(CPURISCVState *env, int csrno,
                                         target_ulong *val)
{
    RISCVException ret = std_read(env, csrno, val);

    if (ret == RISCV_EXCP_NONE && esp32p4_env_is_hp(env) &&
        esp32p4_cpu_from_env(env)->mcountinhibit_tm) {
        *val |= COUNTEREN_TM;
    }
    return ret;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException write_mcountinhibit(CPURISCVState *env, int csrno,
                                          target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

        cpu->mcountinhibit_tm = val & COUNTEREN_TM;
        if (cpu->mcountinhibit_tm) {
            qemu_log_mask(LOG_UNIMP, "esp32p4: mcountinhibit.TM does not "
                          "stop the CLINT time the time CSR reads\n");
        }
    }
    return std_write(env, csrno, val, ra);
}

static int esp32p4_hpm_event_index(int csrno)
{
    for (int i = 0; i < ARRAY_SIZE(esp32p4_hpm_events); i++) {
        if (csrno == CSR_MCOUNTINHIBIT + esp32p4_hpm_events[i].counter) {
            return i;
        }
    }
    return -1;
}

/*
 * Only mhpmevent8, 9 and 13 exist on the HP core, each with a five-bit
 * selector of which one value is a valid event.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException mhpmevent_predicate(CPURISCVState *env, int csrno)
{
    if (esp32p4_env_is_hp(env)) {
        return esp32p4_hpm_event_index(csrno) < 0 ? RISCV_EXCP_ILLEGAL_INST
                                                  : RISCV_EXCP_NONE;
    }
    return std_predicate(env, csrno);
}

static RISCVException read_mhpmevent(CPURISCVState *env, int csrno,
                                     target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = esp32p4_cpu_from_env(env)->mhpmevent[
            esp32p4_hpm_event_index(csrno)];
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_mhpmevent(CPURISCVState *env, int csrno,
                                      target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        int i = esp32p4_hpm_event_index(csrno);

        val &= ESP32P4_MHPMEVENT_MASK;
        esp32p4_cpu_from_env(env)->mhpmevent[i] = val;
        if (val == esp32p4_hpm_events[i].event) {
            qemu_log_mask(LOG_UNIMP, "esp32p4: mhpmcounter%d does not count "
                          "%s\n", esp32p4_hpm_events[i].counter,
                          esp32p4_hpm_events[i].what);
        }
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

/* Memory protection */

/*
 * pmpcfg0-7 and pmpaddr0-31 are the HP core's 32 PMP entries, which
 * esp32p4_pmp.c models; pmpcfg8-15 and pmpaddr32-63 do not exist on it.
 */
/* [spec:nuos:req:emu.esp32p4.pmp] */
static RISCVException read_pmpcfg(CPURISCVState *env, int csrno,
                                  target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = esp32p4_pmpcfg_read(esp32p4_cpu_from_env(env),
                                   csrno - CSR_PMPCFG0);
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_pmpcfg(CPURISCVState *env, int csrno,
                                   target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        esp32p4_pmpcfg_write(esp32p4_cpu_from_env(env), csrno - CSR_PMPCFG0,
                             val);
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

static RISCVException read_pmpaddr(CPURISCVState *env, int csrno,
                                   target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = esp32p4_pmpaddr_read(esp32p4_cpu_from_env(env),
                                    csrno - CSR_PMPADDR0);
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_pmpaddr(CPURISCVState *env, int csrno,
                                    target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        esp32p4_pmpaddr_write(esp32p4_cpu_from_env(env), csrno - CSR_PMPADDR0,
                              val);
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

/* pma_cfg0-15 and pma_addr0-15: the HP core's PMA checker. */
/* [spec:nuos:req:emu.esp32p4.pmp] */
static RISCVException read_pma(CPURISCVState *env, int csrno,
                               target_ulong *val)
{
    *val = esp32p4_pma_read(esp32p4_cpu_from_env(env), csrno);
    return RISCV_EXCP_NONE;
}

static RISCVException write_pma(CPURISCVState *env, int csrno,
                                target_ulong val, uintptr_t ra)
{
    esp32p4_pma_write(esp32p4_cpu_from_env(env), csrno, val);
    return RISCV_EXCP_NONE;
}

/* Trigger module */

static RISCVException read_tselect(CPURISCVState *env, int csrno,
                                   target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = esp32p4_cpu_from_env(env)->tselect;
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_tselect(CPURISCVState *env, int csrno,
                                    target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        esp32p4_tselect_write(esp32p4_cpu_from_env(env), val);
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

static RISCVException read_tdata(CPURISCVState *env, int csrno,
                                 target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = esp32p4_tdata_read(esp32p4_cpu_from_env(env),
                                  csrno - CSR_TDATA1);
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_tdata(CPURISCVState *env, int csrno,
                                  target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        esp32p4_tdata_write(esp32p4_cpu_from_env(env), csrno - CSR_TDATA1,
                            val);
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

/* tinfo: the only trigger type is the address match type, 2. */
static RISCVException read_tinfo(CPURISCVState *env, int csrno,
                                 target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = BIT(2);
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_tinfo(CPURISCVState *env, int csrno,
                                  target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

static RISCVException read_tcontrol(CPURISCVState *env, int csrno,
                                    target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->tcontrol;
    return RISCV_EXCP_NONE;
}

static RISCVException write_tcontrol(CPURISCVState *env, int csrno,
                                     target_ulong val, uintptr_t ra)
{
    esp32p4_cpu_from_env(env)->tcontrol =
        val & (ESP32P4_TCONTROL_MTE | ESP32P4_TCONTROL_MPTE);
    return RISCV_EXCP_NONE;
}

/* mcontext: the six-bit context a trigger's tdata3 may require. */
static RISCVException read_mcontext(CPURISCVState *env, int csrno,
                                    target_ulong *val)
{
    if (esp32p4_env_is_hp(env)) {
        *val = esp32p4_cpu_from_env(env)->mcontext;
        return RISCV_EXCP_NONE;
    }
    return std_read(env, csrno, val);
}

static RISCVException write_mcontext(CPURISCVState *env, int csrno,
                                     target_ulong val, uintptr_t ra)
{
    if (esp32p4_env_is_hp(env)) {
        esp32p4_cpu_from_env(env)->mcontext = val & 0x3f;
        return RISCV_EXCP_NONE;
    }
    return std_write(env, csrno, val, ra);
}

/* Hardware loops */

/*
 * The machine-mode loop CSRs exist while the extension is not off; the
 * user-mode views also need mhwloop_state_reg.URW below machine mode, as
 * does the user-mode state CSR. A debugger may always read them.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException hwlp_m(CPURISCVState *env, int csrno)
{
    if (!esp32p4_env_is_hp(env)) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    if (env->debugger) {
        return RISCV_EXCP_NONE;
    }
    if ((esp32p4_cpu_from_env(env)->hwlp_state & ESP32P4_HWLP_STATE) ==
        ESP32P4_EXT_OFF) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    return RISCV_EXCP_NONE;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException hwlp_u_state(CPURISCVState *env, int csrno)
{
    if (!esp32p4_env_is_hp(env)) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    if (env->debugger || env->priv == PRV_M ||
        (esp32p4_cpu_from_env(env)->hwlp_state & ESP32P4_HWLP_URW)) {
        return RISCV_EXCP_NONE;
    }
    return RISCV_EXCP_ILLEGAL_INST;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException hwlp_u(CPURISCVState *env, int csrno)
{
    RISCVException ret = hwlp_u_state(env, csrno);

    if (ret != RISCV_EXCP_NONE) {
        return ret;
    }
    return hwlp_m(env, csrno);
}

/* The loop and register a loop CSR maps, 0 to 2 for start, end, count. */
static void hwlp_csr_decode(int csrno, int *loop, int *reg)
{
    int index = csrno >= CSR_ESP_UHWLP_START0 ? csrno - CSR_ESP_UHWLP_START0
                                              : csrno - CSR_ESP_MHWLP_START0;

    *loop = index / 3;
    *reg = index % 3;
}

static RISCVException read_hwlp(CPURISCVState *env, int csrno,
                                target_ulong *val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    int loop, reg;

    hwlp_csr_decode(csrno, &loop, &reg);
    switch (reg) {
    case 0:
        *val = cpu->hwlp_start[loop];
        break;
    case 1:
        *val = cpu->hwlp_end[loop];
        break;
    default:
        *val = cpu->hwlp_count[loop];
        break;
    }
    return RISCV_EXCP_NONE;
}

/* Writing a loop CSR makes the extension's state dirty. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException write_hwlp(CPURISCVState *env, int csrno,
                                 target_ulong val, uintptr_t ra)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    int loop, reg;

    hwlp_csr_decode(csrno, &loop, &reg);
    switch (reg) {
    case 0:
        cpu->hwlp_start[loop] = val;
        break;
    case 1:
        cpu->hwlp_end[loop] = val;
        break;
    default:
        cpu->hwlp_count[loop] = val & ESP32P4_HWLP_COUNT_MASK;
        break;
    }
    cpu->hwlp_state = (cpu->hwlp_state & ~ESP32P4_HWLP_STATE) |
                      ESP32P4_EXT_DIRTY;
    return RISCV_EXCP_NONE;
}

static RISCVException read_mhwlp_state(CPURISCVState *env, int csrno,
                                       target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->hwlp_state;
    return RISCV_EXCP_NONE;
}

static RISCVException write_mhwlp_state(CPURISCVState *env, int csrno,
                                        target_ulong val, uintptr_t ra)
{
    esp32p4_cpu_from_env(env)->hwlp_state =
        val & (ESP32P4_HWLP_STATE | ESP32P4_HWLP_URW);
    return RISCV_EXCP_NONE;
}

/* uhwloop_state_reg is the STATE field of mhwloop_state_reg. */
static RISCVException read_uhwlp_state(CPURISCVState *env, int csrno,
                                       target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->hwlp_state & ESP32P4_HWLP_STATE;
    return RISCV_EXCP_NONE;
}

static RISCVException write_uhwlp_state(CPURISCVState *env, int csrno,
                                        target_ulong val, uintptr_t ra)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    cpu->hwlp_state = (cpu->hwlp_state & ~ESP32P4_HWLP_STATE) |
                      (val & ESP32P4_HWLP_STATE);
    return RISCV_EXCP_NONE;
}

/* Extension state */

static RISCVException read_mext_ill(CPURISCVState *env, int csrno,
                                    target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->ext_ill;
    return RISCV_EXCP_NONE;
}

static RISCVException write_mext_ill(CPURISCVState *env, int csrno,
                                     target_ulong val, uintptr_t ra)
{
    esp32p4_cpu_from_env(env)->ext_ill =
        val & (ESP32P4_EXT_ILL_FPU | ESP32P4_EXT_ILL_HWLP |
               ESP32P4_EXT_ILL_PIE);
    return RISCV_EXCP_NONE;
}

static RISCVException read_mext_pie_status(CPURISCVState *env, int csrno,
                                           target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->pie_state;
    return RISCV_EXCP_NONE;
}

static RISCVException write_mext_pie_status(CPURISCVState *env, int csrno,
                                            target_ulong val, uintptr_t ra)
{
    esp32p4_cpu_from_env(env)->pie_state = val & ESP32P4_HWLP_STATE;
    return RISCV_EXCP_NONE;
}

/* mexstatus: EXPT_VLD and LOCKUP are only writable in debug mode. */
static RISCVException read_mexstatus(CPURISCVState *env, int csrno,
                                     target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->mexstatus;
    return RISCV_EXCP_NONE;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException write_mexstatus(CPURISCVState *env, int csrno,
                                      target_ulong val, uintptr_t ra)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    cpu->mexstatus = (cpu->mexstatus & ~ESP32P4_MEXSTATUS_WMASK) |
                     (val & ESP32P4_MEXSTATUS_WMASK);
    esp32p4_bus_error_check_pending(cpu);
    return RISCV_EXCP_NONE;
}

static RISCVException read_mhint(CPURISCVState *env, int csrno,
                                 target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->mhint;
    return RISCV_EXCP_NONE;
}

static RISCVException write_mhint(CPURISCVState *env, int csrno,
                                  target_ulong val, uintptr_t ra)
{
    esp32p4_cpu_from_env(env)->mhint = val & ESP32P4_MHINT_MASK;
    return RISCV_EXCP_NONE;
}

/*
 * mhcr only steers the branch predictor, which has no architectural
 * effect beyond the register itself.
 */
static RISCVException read_mhcr(CPURISCVState *env, int csrno,
                                target_ulong *val)
{
    *val = esp32p4_cpu_from_env(env)->mhcr;
    return RISCV_EXCP_NONE;
}

static RISCVException write_mhcr(CPURISCVState *env, int csrno,
                                 target_ulong val, uintptr_t ra)
{
    esp32p4_cpu_from_env(env)->mhcr = val & ESP32P4_MHCR_MASK;
    return RISCV_EXCP_NONE;
}

/* Floating point */

/*
 * fxcr, like fcsr, needs the floating-point unit enabled in mstatus.FS.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException fxcr_predicate(CPURISCVState *env, int csrno)
{
    if (!esp32p4_env_is_hp(env)) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    if (!env->debugger && !riscv_cpu_fp_enabled(env)) {
        return RISCV_EXCP_ILLEGAL_INST;
    }
    return RISCV_EXCP_NONE;
}

/*
 * NaN results: with fxcr.DQNaN clear the RISC-V canonical NaN; with it
 * set, as IEEE 754 allows, a quietened NaN operand, the first signalling
 * one if any, otherwise the first quiet one, and the default NaN for an
 * invalid infinity times zero.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_hp_update_fp_status(ESP32P4CPU *cpu)
{
    float_status *s = &cpu->parent_obj.env.fp_status;

    set_default_nan_mode(!cpu->fxcr_dqnan, s);
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, s);
    set_float_3nan_prop_rule(float_3nan_prop_s_abc, s);
    set_float_infzeronan_rule(float_infzeronan_dnan_always, s);
}

/*
 * fxcr maps frm onto RM and fflags onto its low bits, with FE set while
 * any exception flag is.
 */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException read_fxcr(CPURISCVState *env, int csrno,
                                target_ulong *val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);
    uint32_t flags = riscv_cpu_get_fflags(env);

    *val = flags | (flags ? ESP32P4_FXCR_FE : 0) |
           (cpu->fxcr_dqnan ? ESP32P4_FXCR_DQNAN : 0) |
           (env->frm << ESP32P4_FXCR_RM_SHIFT);
    return RISCV_EXCP_NONE;
}

/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException write_fxcr(CPURISCVState *env, int csrno,
                                 target_ulong val, uintptr_t ra)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    env->mstatus |= MSTATUS_FS;
    env->frm = extract32(val, ESP32P4_FXCR_RM_SHIFT, 3);
    riscv_cpu_set_fflags(env, val & FSR_AEXC);
    cpu->fxcr_dqnan = val & ESP32P4_FXCR_DQNAN;
    esp32p4_hp_update_fp_status(cpu);
    return RISCV_EXCP_NONE;
}

/* Fast GPIO */

/* [spec:nuos:req:emu.esp32p4.cpu] */
void esp32p4_hp_gpio_update(ESP32P4CPU *cpu)
{
    for (int i = 0; i < ESP32P4_CPU_GPIOS; i++) {
        qemu_set_irq(cpu->gpio_out_irq[i], extract32(cpu->gpio_out, i, 1));
        qemu_set_irq(cpu->gpio_oe_irq[i], extract32(cpu->gpio_oen, i, 1));
    }
}

static RISCVException read_gpio(CPURISCVState *env, int csrno,
                                target_ulong *val)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    switch (csrno) {
    case CSR_ESP_GPIO_OEN:
        *val = cpu->gpio_oen;
        break;
    case CSR_ESP_GPIO_IN:
        *val = cpu->gpio_in;
        break;
    default:
        *val = cpu->gpio_out;
        break;
    }
    return RISCV_EXCP_NONE;
}

/* cpu_gpio_in is read-only: writing it is illegal. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static RISCVException write_gpio(CPURISCVState *env, int csrno,
                                 target_ulong val, uintptr_t ra)
{
    ESP32P4CPU *cpu = esp32p4_cpu_from_env(env);

    switch (csrno) {
    case CSR_ESP_GPIO_OEN:
        cpu->gpio_oen = val & MAKE_64BIT_MASK(0, ESP32P4_CPU_GPIOS);
        break;
    case CSR_ESP_GPIO_IN:
        return RISCV_EXCP_ILLEGAL_INST;
    default:
        cpu->gpio_out = val & MAKE_64BIT_MASK(0, ESP32P4_CPU_GPIOS);
        break;
    }
    esp32p4_hp_gpio_update(cpu);
    return RISCV_EXCP_NONE;
}

/* Bus errors */

static uint32_t *bus_error_csr(ESP32P4CPU *cpu, int csrno)
{
    switch (csrno) {
    case CSR_ESP_LDPC0:
    case CSR_ESP_LDPC1:
        return &cpu->ldpc[csrno - CSR_ESP_LDPC0];
    case CSR_ESP_LDTVAL0:
    case CSR_ESP_LDTVAL1:
        return &cpu->ldtval[csrno - CSR_ESP_LDTVAL0];
    case CSR_ESP_STPC0 ... CSR_ESP_STPC2:
        return &cpu->stpc[csrno - CSR_ESP_STPC0];
    default:
        return &cpu->sttval[csrno - CSR_ESP_STTVAL0];
    }
}

static RISCVException read_bus_error(CPURISCVState *env, int csrno,
                                     target_ulong *val)
{
    *val = *bus_error_csr(esp32p4_cpu_from_env(env), csrno);
    return RISCV_EXCP_NONE;
}

static RISCVException write_bus_error(CPURISCVState *env, int csrno,
                                      target_ulong val, uintptr_t ra)
{
    *bus_error_csr(esp32p4_cpu_from_env(env), csrno) = val;
    return RISCV_EXCP_NONE;
}

#define HP_CSR(no, name, pred, rd, wr) \
    { .csrno = (no), .csr_ops = { (name), (pred), (rd), (wr) } }

#define MHPMEVENT(n) \
    HP_CSR(CSR_MCOUNTINHIBIT + (n), "mhpmevent" #n, mhpmevent_predicate, \
           read_mhpmevent, write_mhpmevent)

#define PMPCFG(n) \
    HP_CSR(CSR_PMPCFG0 + (n), "pmpcfg" #n, hp_std, read_pmpcfg, write_pmpcfg)
#define PMPADDR(n) \
    HP_CSR(CSR_PMPADDR0 + (n), "pmpaddr" #n, hp_std, read_pmpaddr, \
           write_pmpaddr)
#define PMPCFG_ABSENT(n) \
    HP_CSR(CSR_PMPCFG0 + (n), "pmpcfg" #n, hp_absent, read_absent, \
           write_absent)
#define PMPADDR_ABSENT(n) \
    HP_CSR(CSR_PMPADDR0 + (n), "pmpaddr" #n, hp_absent, read_absent, \
           write_absent)
#define PMACFG(n) \
    HP_CSR(CSR_ESP_PMACFG0 + (n), "pma_cfg" #n, hp, read_pma, write_pma)
#define PMAADDR(n) \
    HP_CSR(CSR_ESP_PMAADDR0 + (n), "pma_addr" #n, hp, read_pma, write_pma)

/* [spec:nuos:req:emu.esp32p4.cpu] */
const RISCVCSR esp32p4_hp_csr_list[] = {
    HP_CSR(CSR_MSTATUS, "mstatus", hp_std, read_mstatus, write_mstatus),
    HP_CSR(CSR_MSTATUSH, "mstatush", hp_absent, read_absent, write_absent),
    HP_CSR(CSR_MENVCFG, "menvcfg", hp_absent, read_absent, write_absent),
    HP_CSR(CSR_MENVCFGH, "menvcfgh", hp_absent, read_absent, write_absent),
    HP_CSR(CSR_MCONFIGPTR, "mconfigptr", hp_absent, read_absent,
           write_absent),
    HP_CSR(CSR_MCOUNTINHIBIT, "mcountinhibit", hp_std, read_mcountinhibit,
           write_mcountinhibit),
    MHPMEVENT(3), MHPMEVENT(4), MHPMEVENT(5), MHPMEVENT(6), MHPMEVENT(7),
    MHPMEVENT(8), MHPMEVENT(9), MHPMEVENT(10), MHPMEVENT(11), MHPMEVENT(12),
    MHPMEVENT(13), MHPMEVENT(14), MHPMEVENT(15), MHPMEVENT(16),
    MHPMEVENT(17), MHPMEVENT(18), MHPMEVENT(19), MHPMEVENT(20),
    MHPMEVENT(21), MHPMEVENT(22), MHPMEVENT(23), MHPMEVENT(24),
    MHPMEVENT(25), MHPMEVENT(26), MHPMEVENT(27), MHPMEVENT(28),
    MHPMEVENT(29), MHPMEVENT(30), MHPMEVENT(31),
    PMPCFG(0), PMPCFG(1), PMPCFG(2), PMPCFG(3),
    PMPCFG(4), PMPCFG(5), PMPCFG(6), PMPCFG(7),
    PMPCFG_ABSENT(8), PMPCFG_ABSENT(9), PMPCFG_ABSENT(10),
    PMPCFG_ABSENT(11), PMPCFG_ABSENT(12), PMPCFG_ABSENT(13),
    PMPCFG_ABSENT(14), PMPCFG_ABSENT(15),
    PMPADDR(0), PMPADDR(1), PMPADDR(2), PMPADDR(3), PMPADDR(4), PMPADDR(5),
    PMPADDR(6), PMPADDR(7), PMPADDR(8), PMPADDR(9), PMPADDR(10),
    PMPADDR(11), PMPADDR(12), PMPADDR(13), PMPADDR(14), PMPADDR(15),
    PMPADDR(16), PMPADDR(17), PMPADDR(18), PMPADDR(19), PMPADDR(20),
    PMPADDR(21), PMPADDR(22), PMPADDR(23), PMPADDR(24), PMPADDR(25),
    PMPADDR(26), PMPADDR(27), PMPADDR(28), PMPADDR(29), PMPADDR(30),
    PMPADDR(31),
    PMPADDR_ABSENT(32), PMPADDR_ABSENT(33), PMPADDR_ABSENT(34),
    PMPADDR_ABSENT(35), PMPADDR_ABSENT(36), PMPADDR_ABSENT(37),
    PMPADDR_ABSENT(38), PMPADDR_ABSENT(39), PMPADDR_ABSENT(40),
    PMPADDR_ABSENT(41), PMPADDR_ABSENT(42), PMPADDR_ABSENT(43),
    PMPADDR_ABSENT(44), PMPADDR_ABSENT(45), PMPADDR_ABSENT(46),
    PMPADDR_ABSENT(47), PMPADDR_ABSENT(48), PMPADDR_ABSENT(49),
    PMPADDR_ABSENT(50), PMPADDR_ABSENT(51), PMPADDR_ABSENT(52),
    PMPADDR_ABSENT(53), PMPADDR_ABSENT(54), PMPADDR_ABSENT(55),
    PMPADDR_ABSENT(56), PMPADDR_ABSENT(57), PMPADDR_ABSENT(58),
    PMPADDR_ABSENT(59), PMPADDR_ABSENT(60), PMPADDR_ABSENT(61),
    PMPADDR_ABSENT(62), PMPADDR_ABSENT(63),
    PMACFG(0), PMACFG(1), PMACFG(2), PMACFG(3), PMACFG(4), PMACFG(5),
    PMACFG(6), PMACFG(7), PMACFG(8), PMACFG(9), PMACFG(10), PMACFG(11),
    PMACFG(12), PMACFG(13), PMACFG(14), PMACFG(15),
    PMAADDR(0), PMAADDR(1), PMAADDR(2), PMAADDR(3), PMAADDR(4), PMAADDR(5),
    PMAADDR(6), PMAADDR(7), PMAADDR(8), PMAADDR(9), PMAADDR(10),
    PMAADDR(11), PMAADDR(12), PMAADDR(13), PMAADDR(14), PMAADDR(15),
    HP_CSR(CSR_TSELECT, "tselect", hp_std, read_tselect, write_tselect),
    HP_CSR(CSR_TDATA1, "tdata1", hp_std, read_tdata, write_tdata),
    HP_CSR(CSR_TDATA2, "tdata2", hp_std, read_tdata, write_tdata),
    HP_CSR(CSR_TDATA3, "tdata3", hp_std, read_tdata, write_tdata),
    HP_CSR(CSR_TINFO, "tinfo", hp_std, read_tinfo, write_tinfo),
    HP_CSR(CSR_ESP_TCONTROL, "tcontrol", hp, read_tcontrol, write_tcontrol),
    HP_CSR(CSR_MCONTEXT, "mcontext", hp_std, read_mcontext, write_mcontext),

    HP_CSR(CSR_ESP_MHCR, "mhcr", hp, read_mhcr, write_mhcr),
    HP_CSR(CSR_ESP_MHINT, "mhint", hp, read_mhint, write_mhint),
    HP_CSR(CSR_ESP_MHWLP_START0, "mhwloop0_start_addr", hwlp_m, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_MHWLP_END0, "mhwloop0_end_addr", hwlp_m, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_MHWLP_COUNT0, "mhwloop0_count", hwlp_m, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_MHWLP_START1, "mhwloop1_start_addr", hwlp_m, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_MHWLP_END1, "mhwloop1_end_addr", hwlp_m, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_MHWLP_COUNT1, "mhwloop1_count", hwlp_m, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_MEXSTATUS, "mexstatus", hp, read_mexstatus,
           write_mexstatus),
    HP_CSR(CSR_ESP_MEXT_ILL, "mext_ill", hp, read_mext_ill, write_mext_ill),
    HP_CSR(CSR_ESP_MHWLP_STATE, "mhwloop_state", hp, read_mhwlp_state,
           write_mhwlp_state),
    HP_CSR(CSR_ESP_MEXT_PIE_STATUS, "mext_pie_status", hp,
           read_mext_pie_status, write_mext_pie_status),
    HP_CSR(CSR_ESP_FXCR, "fxcr", fxcr_predicate, read_fxcr, write_fxcr),
    HP_CSR(CSR_ESP_GPIO_OEN, "cpu_gpio_oen", hp, read_gpio, write_gpio),
    HP_CSR(CSR_ESP_GPIO_IN, "cpu_gpio_in", hp, read_gpio, write_gpio),
    HP_CSR(CSR_ESP_GPIO_OUT, "cpu_gpio_out", hp, read_gpio, write_gpio),
    HP_CSR(CSR_ESP_UHWLP_START0, "uhwloop0_start_addr", hwlp_u, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_UHWLP_END0, "uhwloop0_end_addr", hwlp_u, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_UHWLP_COUNT0, "uhwloop0_count", hwlp_u, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_UHWLP_START1, "uhwloop1_start_addr", hwlp_u, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_UHWLP_END1, "uhwloop1_end_addr", hwlp_u, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_UHWLP_COUNT1, "uhwloop1_count", hwlp_u, read_hwlp,
           write_hwlp),
    HP_CSR(CSR_ESP_UHWLP_STATE, "uhwloop_state", hwlp_u_state,
           read_uhwlp_state, write_uhwlp_state),
    HP_CSR(CSR_ESP_LDPC0, "ldpc0", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_LDPC1, "ldpc1", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_LDTVAL0, "ldtval0", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_LDTVAL1, "ldtval1", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_STPC0, "stpc0", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_STPC1, "stpc1", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_STPC2, "stpc2", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_STTVAL0, "sttval0", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_STTVAL1, "sttval1", hp, read_bus_error, write_bus_error),
    HP_CSR(CSR_ESP_STTVAL2, "sttval2", hp, read_bus_error, write_bus_error),
    { },
};
