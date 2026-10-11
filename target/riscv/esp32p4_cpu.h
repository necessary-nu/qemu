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
 * Clock input of both cores: the core's clock, CPU_CLK as HP_SYS_CLKRST
 * gates it for each HP core and LP_CORE's clock for the LP core, whose
 * cycles mcycle counts at the clock's current frequency.
 */
#define ESP32P4_CPU_CLK "clk"

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

/*
 * Named GPIO input of both cores: line 0 the IBUS, 1 the DBUS and 2 the
 * AHB bus, each high while the bus's error response is disabled
 * (HP_SYSTEM_CORE_ERR_RESP_DIS for the HP cores,
 * LP_SYSTEM_LP_CORE_ERR_RESP_DIS for the LP core): an access the bus
 * would answer with an error completes without one, a read giving 0, and
 * the core takes no access fault.
 */
#define ESP32P4_CPU_ERR_RESP_DIS "err-resp-dis"
#define ESP32P4_CPU_BUS_IBUS 0
#define ESP32P4_CPU_BUS_DBUS 1
#define ESP32P4_CPU_BUS_AHB  2

/*
 * Named GPIO input of the HP core: its dedicated wakeup input
 * (HP_SYSTEM_CPU_WAKEUP_EVENT), which wakes it from WFI; while it is high
 * a WFI does not wait.
 */
#define ESP32P4_CPU_WAKEUP "wakeup"

/*
 * The LP core's interrupt controller (TRM 5.4.1): named GPIO input
 * ESP32P4_CPU_LP_IRQ, line n the level of interrupt ID n, of which IDs 3,
 * 7, 11 and 16 to 30 exist; mip shows them and mie enables them.
 */
#define ESP32P4_CPU_LP_IRQ "lp-irq"
#define ESP32P4_LP_IRQS 32
#define ESP32P4_LP_IRQ_MASK (BIT(3) | BIT(7) | BIT(11) | \
                             MAKE_64BIT_MASK(16, 15))

/*
 * The LP core's other connections:
 *
 * ESP32P4_CPU_LP_DISABLE: named GPIO input, LP_SYSTEM's LP_CORE_DISABLE,
 *   which while high stops the core's clock as the clock enable does.
 * ESP32P4_CPU_LP_EXCEPTION: named GPIO output, pulsed as the core takes
 *   an exception, for the PMU's LP CPU exception wakeup source.
 * ESP32P4_CPU_ETM_EVENT: named GPIO output, one line, the ETM event
 *   ULP_EVT_ERR_INTR, pulsed as the core takes an exception.
 */
#define ESP32P4_CPU_LP_DISABLE "lp-disable"
#define ESP32P4_CPU_LP_EXCEPTION "lp-exception"
#define ESP32P4_CPU_ETM_EVENT "etm-event"

/*
 * The LP core's start: as its reset is released, the machine gives the
 * address it starts at and the trap vector base mtvec takes, from the
 * boot address LP_SYSTEM_LP_CORE_BOOT_ADDR holds and the LP ROM.
 */
typedef void (*ESP32P4LPBootFn)(void *opaque, uint32_t *pc,
                                uint32_t *mtvec);

/* The HP core's hardware loops (TRM 2.7.1) and debug triggers (2.11.3). */
#define ESP32P4_HWLP_LOOPS 2
#define ESP32P4_TRIGGERS 3

/* The LP core's debug triggers (TRM 5.6) and event counters (5.7). */
#define ESP32P4_LP_TRIGGERS 2
#define ESP32P4_LP_HPM_FIRST 3
#define ESP32P4_LP_HPM_COUNTERS 10

/* The HP core's event counters, mhpmcounter8, 9 and 13 (TRM 2.5). */
#define ESP32P4_HP_HPM_COUNTERS 3

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

/*
 * The debug assistant's interface to an HP core
 * (hw/misc/esp32p4_assist_debug.c). The core reports, with the BQL held:
 *
 * sp: its SP left or re-entered the bounds in spmon_min and spmon_max, at
 *   the instruction at @pc; @raw has ESP32P4_ASSIST_SP_MIN set while SP is
 *   below spmon_min and ESP32P4_ASSIST_SP_MAX while above spmon_max.
 * region: a load (@write false) or store by the instruction at @pc, with
 *   SP @sp, touched a region assist_watch watches; @hits holds the
 *   matching bits of assist_watch.
 */
#define ESP32P4_ASSIST_SP_MIN   BIT(8)
#define ESP32P4_ASSIST_SP_MAX   BIT(9)
#define ESP32P4_ASSIST_REGIONS  4

typedef struct ESP32P4AssistOps {
    void (*sp)(void *opaque, int core, uint32_t raw, uint32_t pc);
    void (*region)(void *opaque, int core, uint32_t hits, uint32_t pc,
                   uint32_t sp);
} ESP32P4AssistOps;

/*
 * The trace encoder's interface to an HP core (hw/misc/esp32p4_trace.c).
 * While trace_en is set the core reports, without the BQL:
 *
 * insn: an instruction at @pc in privilege mode @priv starts, so the one
 *   before it has retired; @kind is the instruction's ESP32P4_TRACE_KIND_*
 *   and its length (ESP32P4_TRACE_LEN_4 for 4 bytes, else 2).
 * trap: a trap is taken to @handler: an interrupt (@interrupt) or an
 *   exception, its cause @cause and trap value @tval, at @epc, from
 *   privilege mode @priv; @retired says whether the instruction last
 *   started retired (an interrupt, or an exception taken between
 *   instructions) or raised the exception.
 * trigger: a debug trigger with trace action @action (2 on, 3 off, 4
 *   notify) fired, before the instruction about to start (@before, an
 *   execute trigger) or during the one started (a load or store trigger).
 */
enum {
    ESP32P4_TRACE_KIND_PLAIN,
    ESP32P4_TRACE_KIND_BRANCH,
    ESP32P4_TRACE_KIND_JAL,
    ESP32P4_TRACE_KIND_JALR,
    ESP32P4_TRACE_KIND_XRET,
};
#define ESP32P4_TRACE_KIND_MASK 0x7
#define ESP32P4_TRACE_LEN_4     BIT(3)

typedef struct ESP32P4TraceOps {
    void (*insn)(void *opaque, uint32_t pc, uint32_t kind, int priv);
    void (*trap)(void *opaque, uint32_t epc, uint32_t cause, bool interrupt,
                 uint32_t tval, uint32_t handler, int priv, bool retired);
    void (*trigger)(void *opaque, int action, bool before);
} ESP32P4TraceOps;

/* The base addresses mclicbase and uclicbase hold. */
#define ESP32P4_MCLICBASE 0x20800000
#define ESP32P4_UCLICBASE 0x20b00000

struct ESP32P4CPU {
    RISCVCPU parent_obj;

    /* The reset input is high: the core is held in its reset state. */
    bool reset;
    /* The clock enable is low: the core does not execute. */
    bool clock_stopped;
    /* The LP core's ESP32P4_CPU_LP_DISABLE is high, which also stops it. */
    bool disabled;
    /*
     * The core was executing when its clock stopped, so it carries on
     * when the clock restarts; a core waiting in WFI goes on waiting.
     */
    bool clock_halted;
    /*
     * Stops (a reset asserted, a clock stopped) queued on the core's own
     * thread and not yet acted on; not migrated, as a migration happens
     * with every queue empty.
     */
    int stops_pending;

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
    uint32_t mhpmevent[ESP32P4_HP_HPM_COUNTERS];
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

    /* The ESP32P4_CPU_ERR_RESP_DIS inputs, bit n for line n. */
    uint32_t err_resp_dis;
    /* The HP core's ESP32P4_CPU_WAKEUP input is high. */
    bool wakeup;
    /*
     * The PC at the core's last trap: LP_SYSTEM_LP_CPU_EXC_PC on the LP
     * core, the debug assistant's LASTPC_BEFORE_EXCEPTION on an HP core.
     */
    uint32_t exc_pc;

    /*
     * The debug assistant (HP cores): its interface; the region monitors
     * it enables, ASSIST_DEBUG_CORE_n_INTR_ENA's bits 7:0 (read and write
     * of DRAM0 regions 0 and 1, then of PIF regions 0 and 1), and their
     * bounds, inclusive; and the SP monitor: whether it is enabled, the
     * bounds SP must stay within (0 and 0xFFFFFFFF for a bound not
     * monitored), and whether SP was last outside them.
     */
    const ESP32P4AssistOps *assist_ops;
    void *assist_opaque;
    uint32_t assist_watch;
    uint32_t assist_lo[ESP32P4_ASSIST_REGIONS];
    uint32_t assist_hi[ESP32P4_ASSIST_REGIONS];
    bool spmon_en;
    uint32_t spmon_min;
    uint32_t spmon_max;
    uint32_t spmon_out;

    /*
     * The trace encoder (HP cores): its interface, whether it traces the
     * core, and whether the trap being taken comes after the instruction
     * last started retired (an exception taken between instructions).
     */
    const ESP32P4TraceOps *trace_ops;
    void *trace_opaque;
    bool trace_en;
    bool trap_after_retire;

    /*
     * The LP core's own state (tselect, tdata1 and tdata2 above serve its
     * two triggers).
     */
    ESP32P4LPBootFn lp_boot;
    void *lp_boot_opaque;
    qemu_irq lp_exception;
    qemu_irq lp_etm_event;
    /* An exception's pulses, waiting for the end of the code block. */
    QEMUTimer *lp_exc_timer;
    /*
     * Debug mode, which a trigger enters (TRM 5.6.3): the core halts with
     * dpc and dcsr's cause recording why, until it is reset.
     */
    bool lp_debug_mode;
    uint32_t lp_dpc;
    uint32_t lp_dcsr;

    /*
     * Both cores' counters (tcg/esp32p4_counters.c). The core's clock
     * (ESP32P4_CPU_CLK), and the cycles of it counted up to the virtual
     * time cycles_ns plus cycles_frac 2^-32 ns, the end of the last whole
     * cycle counted while the clock runs. insns counts the instructions
     * the core has started, less those that did not retire; the code
     * QEMU translates for the core adds to it. mcycle and minstret are
     * ctr_base[] plus the cycles or instructions since ctr_ref[] while
     * they count, and ctr_base[] while mcountinhibit stops them; the
     * event counters (mhpmcounter8, 9 and 13 on the HP core, 3 to 12 on
     * the LP core) hold what is written.
     */
    Clock *clk;
    uint64_t cycles;
    int64_t cycles_ns;
    uint32_t cycles_frac;
    uint64_t insns;
    uint64_t ctr_base[2];
    uint64_t ctr_ref[2];
    uint64_t hpm[ESP32P4_LP_HPM_COUNTERS];
    uint32_t mcountinhibit;
    /* The counters have seen the HP core enter WFI, and not yet leave. */
    bool ctr_wfi;
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

/* The LP core's class, for telling it from any other RISC-V core. */
extern ObjectClass *esp32p4_lp_cpu_class;

static inline bool esp32p4_env_is_lp(CPURISCVState *env)
{
    return object_get_class(OBJECT(env_cpu(env))) == esp32p4_lp_cpu_class;
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

/*
 * Whether the calling thread is a core in the middle of a block of
 * translated code, where under icount it may neither raise an interrupt
 * nor read the virtual clock (QEMU allows them only to the instruction
 * that ends a block). The blocks the cores report to from their
 * instructions defer their interrupts while it holds.
 */
bool esp32p4_cpu_mid_block(void);

/* The LP core's start, which the machine gives it. */
void esp32p4_lp_cpu_set_boot(ESP32P4CPU *cpu, ESP32P4LPBootFn fn,
                             void *opaque);

/* esp32p4_csr.c */
void esp32p4_csr_class_init(void);
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

/*
 * esp32p4_lp.c: the LP core's CSRs, counters and triggers. The chip's CSR
 * table (esp32p4_csr.c) asks, for each CSR it holds, whether the LP core
 * implements it its own way, lacks it, or has the standard one.
 */
typedef enum {
    ESP32P4_LP_CSR_STD,
    ESP32P4_LP_CSR_ABSENT,
    ESP32P4_LP_CSR_OWN,
} ESP32P4LPCsrKind;

ESP32P4LPCsrKind esp32p4_lp_csr_kind(CPURISCVState *env, int csrno);
RISCVException esp32p4_lp_csr_read(CPURISCVState *env, int csrno,
                                   target_ulong *val);
RISCVException esp32p4_lp_csr_write(CPURISCVState *env, int csrno,
                                    target_ulong val);
void esp32p4_lp_triggers_reset(ESP32P4CPU *cpu);
void esp32p4_lp_triggers_rebuild(ESP32P4CPU *cpu);
bool esp32p4_lp_debug_check_breakpoint(CPUState *cs);
void esp32p4_lp_debug_excp_handler(CPUState *cs);

/* esp32p4_helper.c */
void esp32p4_bus_error_check_pending(ESP32P4CPU *cpu);
void esp32p4_assist_connect(ESP32P4CPU *cpu, const ESP32P4AssistOps *ops,
                            void *opaque);
void esp32p4_assist_sp_update(ESP32P4CPU *cpu);
void esp32p4_assist_access(ESP32P4CPU *cpu, vaddr addr, int size,
                           MMUAccessType access_type, uintptr_t retaddr);
bool esp32p4_assist_watches_page(ESP32P4CPU *cpu, vaddr page);
int esp32p4_cpu_bus(uint32_t addr, bool fetch);
void esp32p4_trace_connect(ESP32P4CPU *cpu, const ESP32P4TraceOps *ops,
                           void *opaque);
void esp32p4_trace_set_enabled(ESP32P4CPU *cpu, bool en);
void esp32p4_trace_trap(ESP32P4CPU *cpu, uint32_t epc, uint32_t cause,
                        bool interrupt, uint32_t tval, int priv,
                        bool retired);

/*
 * esp32p4_counters.c: both cores' counters. The counters' CSRs, which the
 * chip's CSR table hands over; the core's state changing, with the BQL
 * held: before its reset or clock enable changes or its clock changes
 * rate (esp32p4_counters_update()), and as an HP core enters WFI and
 * as it leaves; and its reset.
 */
RISCVException esp32p4_counters_read(CPURISCVState *env, int csrno,
                                     target_ulong *val);
RISCVException esp32p4_counters_write(CPURISCVState *env, int csrno,
                                      target_ulong val);
void esp32p4_counters_update(ESP32P4CPU *cpu);
void esp32p4_counters_wfi(ESP32P4CPU *cpu, bool wfi);
void esp32p4_counters_reset(ESP32P4CPU *cpu);

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
