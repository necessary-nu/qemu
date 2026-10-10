/*
 * Espressif ESP32-P4 HP and LP RISC-V cores
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TARGET_RISCV_ESP32P4_CPU_H
#define TARGET_RISCV_ESP32P4_CPU_H

#include "target/riscv/cpu.h"
#include "target/riscv/esp32p4_pie.h"
#include "qom/object.h"

#define TYPE_ESP32P4_CPU "esp32p4-cpu"
#define TYPE_ESP32P4_HP_CPU RISCV_CPU_TYPE_NAME("esp32p4-hp")
#define TYPE_ESP32P4_LP_CPU RISCV_CPU_TYPE_NAME("esp32p4-lp")

OBJECT_DECLARE_TYPE(ESP32P4CPU, ESP32P4CPUClass, ESP32P4_CPU)

/*
 * Named GPIO inputs: the core's reset, held while high, and its clock
 * enable, the core's clock running while high. A core starts with its
 * reset released and its clock running.
 */
#define ESP32P4_CPU_RESET "reset"
#define ESP32P4_CPU_CLOCK "clock"

/*
 * The HP core's fast GPIO (TRM 2.5, cpu_gpio_* CSRs): eight outputs each
 * with an output enable, driven by cpu_gpio_out and cpu_gpio_oen, and
 * eight inputs read through cpu_gpio_in. Named GPIO outputs carry each
 * bit's level; the named GPIO input sets each input bit.
 */
#define ESP32P4_CPU_GPIOS 8
#define ESP32P4_CPU_GPIO_OUT "cpu-gpio-out"
#define ESP32P4_CPU_GPIO_OE "cpu-gpio-oe"
#define ESP32P4_CPU_GPIO_IN "cpu-gpio-in"

/*
 * Named GPIO output of the HP core: high while it executes in machine
 * mode, for the interrupt matrix's remapping (TRM 13.4.4).
 */
#define ESP32P4_CPU_M_MODE "m-mode"

/* The HP core's hardware loops (TRM 2.7.1) and debug triggers (2.11.3). */
#define ESP32P4_HWLP_LOOPS 2
#define ESP32P4_TRIGGERS 3

/* The HP core's PMP and PMA checker entries (TRM 2.10). */
#define ESP32P4_PMP_ENTRIES 32
#define ESP32P4_PMA_ENTRIES 16

/* The state field of mhwloop_state_reg and mext_pie_status. */
#define ESP32P4_EXT_OFF     0
#define ESP32P4_EXT_INITIAL 1
#define ESP32P4_EXT_CLEAN   2
#define ESP32P4_EXT_DIRTY   3

/* mext_ill: which disabled extension an illegal instruction belongs to. */
#define ESP32P4_EXT_ILL_FPU  BIT(0)
#define ESP32P4_EXT_ILL_HWLP BIT(1)
#define ESP32P4_EXT_ILL_PIE  BIT(2)

/* mhwloop_state_reg */
#define ESP32P4_HWLP_STATE  0x3
#define ESP32P4_HWLP_URW    BIT(2)

/* A loop's iteration count is 24 bits wide. */
#define ESP32P4_HWLP_COUNT_MASK 0xffffff

/*
 * The registers of a loop that helper_esp32p4_hwlp_set() sets, with the
 * loop's index in bit 0.
 */
#define ESP32P4_HWLP_SET_START BIT(1)
#define ESP32P4_HWLP_SET_END   BIT(2)
#define ESP32P4_HWLP_SET_COUNT BIT(3)

/* mexstatus (0x7E1) */
#define ESP32P4_MEXSTATUS_EXPT_VLD  BIT(5)
#define ESP32P4_MEXSTATUS_LOCKUP    BIT(6)
#define ESP32P4_MEXSTATUS_BUS_ERR   BIT(8)
#define ESP32P4_MEXSTATUS_PBEXE     BIT(9)
#define ESP32P4_MEXSTATUS_PPBEXE    BIT(10)
#define ESP32P4_MEXSTATUS_PMP_ERR   BIT(11)
#define ESP32P4_MEXSTATUS_FETCH_ERR BIT(12)
#define ESP32P4_MEXSTATUS_NMFT      BIT(13)
#define ESP32P4_MEXSTATUS_CLIC_INHV BIT(20)

/* tcontrol (0x7A5) */
#define ESP32P4_TCONTROL_MTE  BIT(3)
#define ESP32P4_TCONTROL_MPTE BIT(7)

/*
 * A request, through the CPU's interrupt_request, to take the exception
 * a pending bus error raises (mexstatus.PBEXE) before the next
 * instruction.
 */
#define ESP32P4_CPU_INTERRUPT_BUS_ERROR CPU_INTERRUPT_TGT_INT_0

/*
 * A request, through the CPU's interrupt_request, held while the core's
 * CLIC presents an interrupt: the core decides whether to take it before
 * the next instruction, and a core in WFI whether to wake.
 */
#define ESP32P4_CPU_INTERRUPT_CLIC CPU_INTERRUPT_TGT_EXT_1

/*
 * The interrupt the CLIC presents to its core, its highest-ranked pending
 * and enabled one (esp32p4_cpu_clic_present()), packed in a word: whether
 * there is one, whether it is selectively hardware vectored, its privilege
 * mode, level and ID.
 */
#define ESP32P4_CLIC_SEL_VALID      BIT(31)
#define ESP32P4_CLIC_SEL_SHV        BIT(30)
#define ESP32P4_CLIC_SEL_PRIV_SHIFT 24
#define ESP32P4_CLIC_SEL_LEVEL_SHIFT 16
#define ESP32P4_CLIC_SEL_ID_MASK    0xff

static inline uint32_t esp32p4_clic_sel(bool shv, int priv, int level, int id)
{
    return ESP32P4_CLIC_SEL_VALID | (shv ? ESP32P4_CLIC_SEL_SHV : 0) |
           (priv << ESP32P4_CLIC_SEL_PRIV_SHIFT) |
           (level << ESP32P4_CLIC_SEL_LEVEL_SHIFT) | id;
}

/* The CLIC's interface to its core. */
typedef struct ESP32P4ClicOps {
    /*
     * The core services interrupt @id, taking it hardware vectored or
     * through mnxti/unxti: an edge-triggered interrupt stops pending.
     */
    void (*claim)(void *opaque, int id);
} ESP32P4ClicOps;

/* The base addresses mclicbase and uclicbase hold. */
#define ESP32P4_MCLICBASE 0x20800000
#define ESP32P4_UCLICBASE 0x20b00000

struct ESP32P4CPU {
    RISCVCPU parent_obj;

    /* The reset input is high: the core is held in its reset state. */
    bool reset;
    /* The clock enable is low: the core does not execute. */
    bool clock_stopped;
    /*
     * The core was executing when its clock stopped, so it carries on
     * when the clock restarts; a core waiting in WFI goes on waiting.
     */
    bool clock_halted;

    /*
     * The HP core's Espressif-specific state; the LP core has none of it.
     */

    /*
     * Hardware loops: the first and last instruction of each loop and the
     * iterations left. The machine (0x7C6-0x7CB) and user (0x8C0-0x8C5)
     * CSRs are two views of these.
     */
    uint32_t hwlp_start[ESP32P4_HWLP_LOOPS];
    uint32_t hwlp_end[ESP32P4_HWLP_LOOPS];
    uint32_t hwlp_count[ESP32P4_HWLP_LOOPS];
    /* mhwloop_state_reg: STATE and URW. */
    uint32_t hwlp_state;
    /* mext_pie_status.STATE, the PIE extension's context status. */
    uint32_t pie_state;
    /* The PIE extension's registers. */
    ESP32P4PIE pie;
    /* mext_ill */
    uint32_t ext_ill;
    uint32_t mexstatus;
    uint32_t mhint;
    /* 0x7C1, the branch predictor control the boot ROM sets. */
    uint32_t mhcr;
    /* fxcr.DQNaN: NaN results propagate an operand's payload. */
    bool fxcr_dqnan;
    uint32_t gpio_oen;
    uint32_t gpio_out;
    uint32_t gpio_in;
    /* Load and store bus errors, the newest in entry 0. */
    uint32_t ldpc[2];
    uint32_t ldtval[2];
    uint32_t stpc[3];
    uint32_t sttval[3];
    /*
     * The access fault being raised comes from a bus error rather than
     * the PMP or PMA; and whether the latest bus error was a store.
     */
    bool bus_error;
    bool bus_error_store;
    /* mhpmevent8, mhpmevent9 and mhpmevent13. */
    uint32_t mhpmevent[3];
    /* mcountinhibit.TM, which the core keeps but the time is not its. */
    bool mcountinhibit_tm;
    /* Trigger module. */
    uint32_t tselect;
    uint32_t tdata1[ESP32P4_TRIGGERS];
    uint32_t tdata2[ESP32P4_TRIGGERS];
    uint32_t tdata3[ESP32P4_TRIGGERS];
    uint32_t tcontrol;
    uint32_t mcontext;
    /* Triggers that matched the breakpoint or watchpoint being taken. */
    uint32_t triggers_fired;

    /*
     * The CLIC (TRM 2.9): the interrupt it presents, ESP32P4_CLIC_SEL_*,
     * which the CLIC sets under the BQL from any thread; and its
     * interface.
     */
    uint32_t clic_sel;
    const ESP32P4ClicOps *clic_ops;
    void *clic_opaque;
    /*
     * The CLIC CSRs. mtvec, mepc, mscratch and the INT and code fields of
     * mcause are the standard CSRs' state; ucause keeps its own INT and
     * code. The interrupt levels of mintstatus (MIL) and uintstatus (UIL),
     * the thresholds, and the previous levels and table-entry flags of
     * mcause and ucause. mintthresh is also memory mapped, so any core may
     * write it.
     */
    uint32_t mtvt;
    uint32_t utvec;
    uint32_t utvt;
    uint32_t uepc;
    uint32_t uscratch;
    uint32_t ucause;
    uint8_t mil;
    uint8_t uil;
    uint8_t mintthresh;
    uint8_t uintthresh;
    uint8_t mpil;
    uint8_t upil;
    bool minhv;
    bool uinhv;
    /* The exception being raised comes from a vector table fetch. */
    bool clic_inhv_fault;

    /* PMP: pmpXcfg and pmpaddrn; PMA checker: pma_cfgn and pma_addrn. */
    uint8_t pmp_cfg[ESP32P4_PMP_ENTRIES];
    uint32_t pmp_addr[ESP32P4_PMP_ENTRIES];
    uint32_t pma_cfg[ESP32P4_PMA_ENTRIES];
    uint32_t pma_addr[ESP32P4_PMA_ENTRIES];

    qemu_irq gpio_out_irq[ESP32P4_CPU_GPIOS];
    qemu_irq gpio_oe_irq[ESP32P4_CPU_GPIOS];
    /* ESP32P4_CPU_M_MODE and the level last driven on it. */
    qemu_irq m_mode_irq;
    bool m_mode_level;
};

struct ESP32P4CPUClass {
    RISCVCPUClass parent_class;

    ResettablePhases parent_phases;
};

/* The offset of an ESP32P4CPU field from the CPU's env, for TCG code. */
#define ESP32P4_ENV_OFFSET(field) \
    (offsetof(ESP32P4CPU, field) - offsetof(ESP32P4CPU, parent_obj.env))

static inline ESP32P4CPU *esp32p4_cpu_from_env(CPURISCVState *env)
{
    return container_of(env_archcpu(env), ESP32P4CPU, parent_obj);
}

/* The core is an HP core: it carries the state above. */
static inline bool esp32p4_env_is_hp(CPURISCVState *env)
{
    return riscv_cpu_cfg(env)->ext_xesploop;
}

/*
 * Hardware loops act and their instructions and CSRs may be used: the
 * extension is not off, and user mode may use it.
 */
static inline bool esp32p4_hwlp_enabled(ESP32P4CPU *cpu)
{
    return (cpu->hwlp_state & ESP32P4_HWLP_STATE) != ESP32P4_EXT_OFF &&
           (cpu->parent_obj.env.priv == PRV_M ||
            (cpu->hwlp_state & ESP32P4_HWLP_URW));
}

#ifndef CONFIG_USER_ONLY
/*
 * esp32p4_cpu.c: the CLIC's side. The CLIC connects itself to its core,
 * presents its highest-ranked pending and enabled interrupt (0 for none),
 * and reads and writes mintthresh, which it also maps; it calls these
 * with the BQL held.
 */
void esp32p4_cpu_clic_connect(ESP32P4CPU *cpu, const ESP32P4ClicOps *ops,
                              void *opaque);
void esp32p4_cpu_clic_present(ESP32P4CPU *cpu, uint32_t sel);
uint8_t esp32p4_cpu_mintthresh(ESP32P4CPU *cpu);
void esp32p4_cpu_set_mintthresh(ESP32P4CPU *cpu, uint8_t th);

/* esp32p4_csr.c */
void esp32p4_hp_csr_class_init(void);
void esp32p4_hp_update_fp_status(ESP32P4CPU *cpu);
void esp32p4_hp_gpio_update(ESP32P4CPU *cpu);

/* esp32p4_trigger.c */
void esp32p4_triggers_rebuild(ESP32P4CPU *cpu);
void esp32p4_triggers_reset(ESP32P4CPU *cpu);
bool esp32p4_debug_check_breakpoint(CPUState *cs);
bool esp32p4_debug_check_watchpoint(CPUState *cs, CPUWatchpoint *wp);
void esp32p4_debug_excp_handler(CPUState *cs);
void esp32p4_tselect_write(ESP32P4CPU *cpu, target_ulong val);
target_ulong esp32p4_tdata_read(ESP32P4CPU *cpu, int index);
void esp32p4_tdata_write(ESP32P4CPU *cpu, int index, target_ulong val);

/* esp32p4_helper.c */
void esp32p4_bus_error_check_pending(ESP32P4CPU *cpu);

/* esp32p4_pie.c */
bool esp32p4_insn_is_pie(uint32_t insn);
void esp32p4_pie_reset(ESP32P4CPU *cpu);

/* esp32p4_cpu.c */
void esp32p4_hp_do_interrupt(CPUState *cs);
void esp32p4_hp_priv_update(ESP32P4CPU *cpu);

/* esp32p4_clic.c */
void esp32p4_clic_cpu_reset(ESP32P4CPU *cpu);
bool esp32p4_clic_takes(ESP32P4CPU *cpu, uint32_t sel);
bool esp32p4_clic_wakes(ESP32P4CPU *cpu);
void esp32p4_clic_take(ESP32P4CPU *cpu, uint32_t sel);
void esp32p4_clic_exception_taken(ESP32P4CPU *cpu, int prev_priv,
                                  uint8_t prev_mil);
target_ulong esp32p4_clic_mret(ESP32P4CPU *cpu, target_ulong retpc);
RISCVException esp32p4_clic_nxti(ESP32P4CPU *cpu, int priv,
                                 target_ulong *ret_value,
                                 target_ulong new_value,
                                 target_ulong write_mask);

/* esp32p4_pmp.c */
bool esp32p4_hp_tlb_fill(CPUState *cs, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx,
                         bool probe, uintptr_t retaddr);
bool esp32p4_lp_tlb_fill(CPUState *cs, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx,
                         bool probe, uintptr_t retaddr);
target_ulong esp32p4_pmpcfg_read(ESP32P4CPU *cpu, int reg);
void esp32p4_pmpcfg_write(ESP32P4CPU *cpu, int reg, target_ulong val);
target_ulong esp32p4_pmpaddr_read(ESP32P4CPU *cpu, int n);
void esp32p4_pmpaddr_write(ESP32P4CPU *cpu, int n, target_ulong val);
target_ulong esp32p4_pma_read(ESP32P4CPU *cpu, int csrno);
void esp32p4_pma_write(ESP32P4CPU *cpu, int csrno, target_ulong val);
void esp32p4_pmp_reset(ESP32P4CPU *cpu);
#endif

#endif
